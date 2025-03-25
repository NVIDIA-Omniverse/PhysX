// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"

#include "atomic.cuh"
#include "reduction.cuh"
#include "bvh.cuh"
#include "GuSDF.h"
#include "utils.cuh"

using namespace physx;

extern "C" __host__ void initSdfConstructionKernels0() {}

PX_FORCE_INLINE __device__ PxVec3 getCenter(PxU32 index, const float4* PX_RESTRICT itemLowers, const float4* PX_RESTRICT itemUppers)
{
	PxVec3 lower = PxLoad3(itemLowers[index]);
	PxVec3 upper = PxLoad3(itemUppers[index]);

	PxVec3 center = 0.5f*(lower + upper);
	return center;
}

extern "C" __global__ void bvhCalculateMortonCodes(const float4* PX_RESTRICT itemLowers, const float4* PX_RESTRICT itemUppers, const PxI32* PX_RESTRICT itemPriorities, PxI32 n,
	const PxVec3* gridLower, const PxVec3* gridInvEdges, PxI32* PX_RESTRICT indices, PxI32* PX_RESTRICT keys)
{
	const PxI32 index = blockDim.x*blockIdx.x + threadIdx.x;

	if (index < n)
	{
		PxVec3 center = getCenter(index, itemLowers, itemUppers);

		PxVec3 local = (center - gridLower[0]).multiply(gridInvEdges[0]);

		PxI32 key;
		if (itemPriorities)
		{
			// 9-bit Morton codes stored in lower 27bits (512^3 effective resolution)
			// 5-bit priority code stored in the upper 5-bits			
			key = morton3<512>(local.x, local.y, local.z);

			// we invert priorities (so that higher priority items appear first in sorted order)
			key |= (~itemPriorities[index]) << 27;
		}
		else
		{
			key = morton3<1024>(local.x, local.y, local.z);
		}

		indices[index] = index;
		keys[index] = key;
	}
}

// calculate the index of the first differing bit between two adjacent Morton keys
extern "C" __global__ void bvhCalculateKeyDeltas(const PxI32* PX_RESTRICT keys, PxReal* PX_RESTRICT deltas, PxI32 n)
{
	const PxI32 index = blockDim.x*blockIdx.x + threadIdx.x;

	if (index + 1 < n)
	{
		PxI32 a = keys[index];
		PxI32 b = keys[index + 1];

		//if (a > b)
		//	printf("Elements not sorted\n");

		PxI32 x = a ^ b;

		deltas[index] = PxReal(x); // reinterpret_cast<PxReal&>(x);// This should work since x is positive
	}
}

// calculate the index of the first differing bit between two adjacent Morton keys
extern "C" __global__ void bvhCalculateKeyDeltasSquaredDistance(const PxI32* PX_RESTRICT keys, PxReal* PX_RESTRICT deltas, PxI32 n,
	const float4* PX_RESTRICT itemLowers, const float4* PX_RESTRICT itemUppers)
{
	const PxI32 index = blockDim.x*blockIdx.x + threadIdx.x;

	if (index + 1 < n)
	{
		//PxI32 a = keys[index];
		//PxI32 b = keys[index + 1];

		//itemLowers and itemUppers must be in sorted order
		PxVec3 centerA = getCenter(index, itemLowers, itemUppers);
		PxVec3 centerB = getCenter(index + 1, itemLowers, itemUppers);
		PxReal distanceSquared = (centerA - centerB).magnitudeSquared();

		//if (a > b)
		//	printf("Elements not sorted\n");

		//PxI32 x = a ^ b;

		deltas[index] = distanceSquared;
	}
}

extern "C" __global__ void bvhBuildLeaves(const float4* PX_RESTRICT itemLowers, const float4* PX_RESTRICT itemUppers,
	PxI32 n, const PxI32* PX_RESTRICT indices, PxI32* PX_RESTRICT rangeLefts, PxI32* PX_RESTRICT rangeRights, PxgPackedNodeHalf* PX_RESTRICT lowers, PxgPackedNodeHalf* PX_RESTRICT uppers)
{
	const PxI32 index = blockDim.x*blockIdx.x + threadIdx.x;

	if (index < n)
	{
		const PxI32 item = indices[index];

		const PxVec3 lower = PxLoad3(itemLowers[item]);
		const float4 upper = itemUppers[item];

		// write leaf nodes 
		lowers[index] = makeNode(lower, item, true);
		uppers[index] = makeNode(PxLoad3(upper), upper.w);

		// write leaf key ranges
		rangeLefts[index] = index;
		rangeRights[index] = index;
	}
}

extern "C" __global__ void bvhComputeTriangleBounds(const PxVec3* PX_RESTRICT vertices, const PxU32* PX_RESTRICT triangleIndices, PxU32 numTriangles,
	float4* PX_RESTRICT itemLowers, float4* PX_RESTRICT itemUppers, PxReal margin)
{
	const PxI32 index = blockDim.x*blockIdx.x + threadIdx.x;

	if (index < numTriangles)
	{
		PxVec3 a = vertices[triangleIndices[3 * index + 0]];
		PxVec3 b = vertices[triangleIndices[3 * index + 1]];
		PxVec3 c = vertices[triangleIndices[3 * index + 2]];

		PxBounds3 bounds(a, a);
		bounds.include(b);
		bounds.include(c);
		bounds.fattenFast(margin);

		itemLowers[index] = make_float4(bounds.minimum.x, bounds.minimum.y, bounds.minimum.z, 0.0f);
		itemUppers[index] = make_float4(bounds.maximum.x, bounds.maximum.y, bounds.maximum.z, 0.0f);

		/*printf("%i   %f %f %f   %f %f %f\n", index, bounds.minimum.x, bounds.minimum.y, bounds.minimum.z, bounds.maximum.x, bounds.maximum.y, bounds.maximum.z);*/
		//printf("%i   %i %i %i   %f %f %f\n", index, triangleIndices[3 * index + 0], triangleIndices[3 * index + 1], triangleIndices[3 * index + 2], a.x, a.y, a.z);
	}
}

extern "C" __global__ void bvhBuildHierarchy(PxI32 n, PxI32* root, PxU32* maxTreeDepth, const PxReal* PX_RESTRICT deltas, PxI32* PX_RESTRICT numChildren,
	volatile PxI32* PX_RESTRICT rangeLefts, volatile PxI32* PX_RESTRICT rangeRights, volatile PxgPackedNodeHalf* PX_RESTRICT lowers, volatile PxgPackedNodeHalf* PX_RESTRICT uppers)
{
	buildHierarchy(n, root, maxTreeDepth, deltas, numChildren, rangeLefts, rangeRights, lowers, uppers);
}


extern "C" __global__ void bvhBuildHierarchyAndWindingClusters(PxI32 n, PxI32* root, PxU32* maxTreeDepth, const PxReal* PX_RESTRICT deltas, PxI32* PX_RESTRICT numChildren,
	volatile PxI32* PX_RESTRICT rangeLefts, volatile PxI32* PX_RESTRICT rangeRights, volatile PxgPackedNodeHalf* PX_RESTRICT lowers, volatile PxgPackedNodeHalf* PX_RESTRICT uppers,
	PxgWindingClusterApproximation* clusters, const PxVec3* vertices, const PxU32* indices)
{
	WindingClusterBuilder w(clusters, vertices, indices, n);
	buildHierarchy(n, root, maxTreeDepth, deltas, numChildren, rangeLefts, rangeRights, lowers, uppers, w);
}

PX_FORCE_INLINE __device__ void AtomicMaxVec3(PxVec3* address, const PxVec3 v)
{
	PxReal* arr = reinterpret_cast<PxReal*>(address);
	AtomicMax(&arr[0], v.x);
	AtomicMax(&arr[1], v.y);
	AtomicMax(&arr[2], v.z);
}

PX_FORCE_INLINE __device__ void AtomicMinVec3(PxVec3* address, const PxVec3 v)
{
	PxReal* arr = reinterpret_cast<PxReal*>(address);
	AtomicMin(&arr[0], v.x);
	AtomicMin(&arr[1], v.y);
	AtomicMin(&arr[2], v.z);
}

extern "C" __global__ void bvhComputeTotalBounds(const float4* itemLowers, const float4* itemUppers, PxVec3* totalLower, PxVec3* totalUpper, PxI32 numItems)
{
	const PxI32 blockStart = blockDim.x*blockIdx.x;
	const PxI32 numValid = min(numItems - blockStart, blockDim.x);

	const PxI32 tid = blockStart + threadIdx.x;

	PxU32 mask = __ballot_sync(FULL_MASK, tid < numItems);
	if (tid < numItems)
	{
		PxVec3 lower = PxLoad3(itemLowers[tid]);
		PxVec3 upper = PxLoad3(itemUppers[tid]);

		PxVec3 blockUpper;
		blockUpper.x = blockReduction<MaxOpFloat, PxReal, 256>(mask, upper.x, -FLT_MAX);
		__syncthreads();
		blockUpper.y = blockReduction<MaxOpFloat, PxReal, 256>(mask, upper.y, -FLT_MAX);
		__syncthreads();
		blockUpper.z = blockReduction<MaxOpFloat, PxReal, 256>(mask, upper.z, -FLT_MAX);

		// sync threads because second reduce uses same temp storage as first
		__syncthreads();

		PxVec3 blockLower;
		blockLower.x = blockReduction<MinOpFloat, PxReal, 256>(mask, lower.x, FLT_MAX);
		__syncthreads();
		blockLower.y = blockReduction<MinOpFloat, PxReal, 256>(mask, lower.y, FLT_MAX);
		__syncthreads();
		blockLower.z = blockReduction<MinOpFloat, PxReal, 256>(mask, lower.z, FLT_MAX);

		if (threadIdx.x == 0)
		{
			// write out block results, expanded by the radius
			AtomicMaxVec3(totalUpper, blockUpper);
			AtomicMinVec3(totalLower, blockLower);
		}
	}
}

// compute inverse edge length, this is just done on the GPU to avoid a CPU->GPU sync point
extern "C" __global__ void bvhComputeTotalInvEdges(const PxVec3* totalLower, const PxVec3* totalUpper, PxVec3* totalInvEdges)
{
	PxVec3 edges = (totalUpper[0] - totalLower[0]);
	edges += PxVec3(0.0001f);

	totalInvEdges[0] = PxVec3(1.0f / edges.x, 1.0f / edges.y, 1.0f / edges.z);
}

PX_FORCE_INLINE __device__ PxReal getDistanceOffset(const PxgPackedNodeHalf& o)
{
	return reinterpret_cast<const float4&>(o).w;
}

PX_FORCE_INLINE __device__ void setDistanceOffset(PxgPackedNodeHalf& o, PxReal distanceOffset)
{
	reinterpret_cast<float4&>(o).w = distanceOffset;
}

//The point is encoded as the center of a leaf node bounding box
struct ClosestDistanceToPointCloudTraversalWithOffset
{
public:
	PxVec3 mQueryPoint;
	PxReal mClosestDistance;

	PX_FORCE_INLINE __device__ ClosestDistanceToPointCloudTraversalWithOffset()
	{
	}

	PX_FORCE_INLINE __device__ ClosestDistanceToPointCloudTraversalWithOffset(const PxVec3& queryPoint, PxReal initialClosestDistance = 100000000000.0f)
		: mQueryPoint(queryPoint), mClosestDistance(initialClosestDistance)
	{
	}

	PX_FORCE_INLINE __device__ PxReal distancePointBoxSquared(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& point)
	{
		PxVec3 closestPt = minimum.maximum(maximum.minimum(point));
		return (closestPt - point).magnitudeSquared();
	}	

	PX_FORCE_INLINE __device__ BvhTraversalControl::Enum operator()(const PxgPackedNodeHalf& lower, const PxgPackedNodeHalf& upper, PxI32 nodeIndex)
	{
		if (distancePointBoxSquared(PxVec3(lower.x, lower.y, lower.z), PxVec3(upper.x, upper.y, upper.z), mQueryPoint) >= mClosestDistance * mClosestDistance)
			return BvhTraversalControl::eDontGoDeeper;

		if (lower.b)
		{
			const PxVec3 point = PxVec3(0.5f * (lower.x + upper.x), 0.5f * (lower.y + upper.y), 0.5f * (lower.z + upper.z));

			PxReal distanceOffset = getDistanceOffset(upper);
			PxReal distSq = (mQueryPoint - point).magnitudeSquared() + distanceOffset * distanceOffset;
			if (distSq < mClosestDistance * mClosestDistance)
			{
				mClosestDistance = PxSqrt(distSq);
			}

			return BvhTraversalControl::eDontGoDeeper;
		}

		return BvhTraversalControl::eGoDeeper;
	}
};

PX_FORCE_INLINE __device__ bool traceInteriorRay(const PxgBvhTriangleMesh& mesh, const PxVec3& origin, const PxVec3& dir, PxI32* stack, PxU32 stackSize, PxReal& closestDotProduct, bool& closestPointOnTriangleEdge)
{
	ClosestRayIntersectionTraversal query(mesh.mVertices, mesh.mTriangles, origin, dir, true);
	queryBVH(mesh.mBvh, query, stack, stackSize);

	closestDotProduct = query.closestDotProduct;
	closestPointOnTriangleEdge = query.closestPointOnTriangleEdge;
	return query.hasHit();
}

extern "C" __global__ __launch_bounds__(256, 1) void sdfCalculateDenseGridHybrid(PxgBvhTriangleMesh mesh, const PxgWindingClusterApproximation* PX_RESTRICT windingNumberClusters,
	Gu::GridQueryPointSampler sampler, PxU32 sizeX, PxU32 sizeY, PxU32 sizeZ, PxReal* PX_RESTRICT sdfData)
{
	const PxU32 stackSize = 47;
	//__shared__ PxI32 stackMem[256 * stackSize];
	PxI32 stackMem[stackSize];

	// block addressing
	const PxI32 x = blockIdx.x*blockDim.x + threadIdx.x;
	const PxI32 y = blockIdx.y*blockDim.y + threadIdx.y;
	const PxI32 z = blockIdx.z*blockDim.z + threadIdx.z;

	//const PxI32 threadId = threadIdx.z * 8 * 8 + threadIdx.y * 8 + threadIdx.x;

	if (x < sizeX && y < sizeY && z < sizeZ)
	{
		PxU32 roodNodeId = *mesh.mBvh.mRootNode;
		PxVec3 meshBoundsMin = mesh.mBvh.mNodeLowers[roodNodeId].getXYZ();
		PxVec3 meshBoundsMax = mesh.mBvh.mNodeUppers[roodNodeId].getXYZ();

		const PxVec3 p = sampler.getPoint(x, y, z);
		
		PxI32* stack = &stackMem[/*stackSize * threadId*/0];

		ClosestDistanceToTriangleMeshTraversal distQuery(mesh.mTriangles, mesh.mVertices, p);
		queryBVH(mesh.mBvh, distQuery, stack, stackSize);
		PxReal d = PxSqrt(distQuery.mClosestDistanceSquared);
		
		PxReal sign = 1.0f;

		bool repeatInsideTest = false;

		PxI32 parity = 0;
		PxReal threshold = 0.01f;		
		PxReal closestDotProduct;
		bool closestPointOnTriangleEdge;

		// x-axis
		if (traceInteriorRay(mesh, p, PxVec3(PxAbs(p.x - meshBoundsMin.x) < PxAbs(meshBoundsMax.x - p.x) ? -1.0f : 1.0f, 0.0f, 0.0f), stack, stackSize, closestDotProduct, closestPointOnTriangleEdge))
		{
			if (closestDotProduct < 0.0f)
				parity++;
			if (closestPointOnTriangleEdge || PxAbs(closestDotProduct) <= threshold)
				repeatInsideTest = true;
		}
			
		// y-axis
		if (!repeatInsideTest && traceInteriorRay(mesh, p, PxVec3(0.0f, PxAbs(p.y - meshBoundsMin.y) < PxAbs(meshBoundsMax.y - p.y) ? -1.0f : 1.0f, 0.0f), stack, stackSize, closestDotProduct, closestPointOnTriangleEdge))
		{
			if (closestDotProduct < 0.0f)
				parity++;
			if (closestPointOnTriangleEdge || PxAbs(closestDotProduct) <= threshold)
				repeatInsideTest = true;
		}
			
		// z-axis
		if (!repeatInsideTest && traceInteriorRay(mesh, p, PxVec3(0.0f, 0.0f, PxAbs(p.z - meshBoundsMin.z) < PxAbs(meshBoundsMax.z - p.z) ? -1.0f : 1.0f), stack, stackSize, closestDotProduct, closestPointOnTriangleEdge))
		{
			if (closestDotProduct < 0.0f)
				parity++;
			if (closestPointOnTriangleEdge || PxAbs(closestDotProduct) <= threshold)
				repeatInsideTest = true;
		}

		if (parity == 3)
			sign = -1.0f;
		else if (parity != 0)
		{
			repeatInsideTest = true;				
		}

		if (repeatInsideTest)
		{
			//Fall back to winding numbers for problematic points
			WindingNumberTraversal windingNumber(mesh.mTriangles, mesh.mNumTriangles, mesh.mVertices, windingNumberClusters, p);
			queryBVH(mesh.mBvh, windingNumber, stack, stackSize);
			bool inside = windingNumber.mWindingNumber > 0.5f;
			if (inside)
				sign = -1.0f;
		}			

		sdfData[Gu::idx3D(x, y, z, sizeX, sizeY)] = d * sign;
	}
	__syncthreads();
}

extern "C" __global__ __launch_bounds__(256, 1) void sdfCalculateDenseGridBlocks(PxgBvhTriangleMesh mesh, const PxgWindingClusterApproximation* PX_RESTRICT windingNumberClusters,
	Gu::GridQueryPointSampler sampler, PxU32 sizeX, PxU32 sizeY, PxU32 sizeZ, PxReal* PX_RESTRICT sdfData, PxReal* PX_RESTRICT windingNumbers)
{
	const PxU32 stackSize = 47;
	//__shared__ PxI32 stackMem[256 * stackSize];
	PxI32 stackMem[stackSize];

	const PxU32 x = blockIdx.x*blockDim.x + threadIdx.x;
	const PxU32 y = blockIdx.y*blockDim.y + threadIdx.y;
	const PxU32 z = blockIdx.z*blockDim.z + threadIdx.z;

	//const PxI32 threadId = threadIdx.z * 8 * 8 + threadIdx.y * 8 + threadIdx.x;

	if (x < sizeX && y < sizeY && z < sizeZ)
	{
		PxVec3 p = sampler.getPoint(x, y, z);

		PxI32* stack = &stackMem[/*stackSize * threadId*/0];
		
		ClosestDistanceToTriangleMeshTraversal distQuery(mesh.mTriangles, mesh.mVertices, p);
		queryBVH(mesh.mBvh, distQuery, stack, stackSize);
		PxReal closestDistance = PxSqrt(distQuery.mClosestDistanceSquared);

		WindingNumberTraversal windingNumber(mesh.mTriangles, mesh.mNumTriangles, mesh.mVertices, windingNumberClusters, p);
		queryBVH(mesh.mBvh, windingNumber, stack, stackSize);

		
		PxU32 resultIndex = Gu::idx3D(x, y, z, sizeX, sizeY);

		bool inside = windingNumber.mWindingNumber > 0.5f;
		sdfData[resultIndex] = (inside ? -1.0f : 1.0f) * closestDistance;

		if (windingNumbers)
			windingNumbers[resultIndex] = windingNumber.mWindingNumber;
	}
}

PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 pow3(PxU32 i)
{
	return i * i * i;
}

PX_FORCE_INLINE PX_CUDA_CALLABLE bool rangesOverlaps(PxReal minA, PxReal maxA, PxReal minB, PxReal maxB)
{
	return  !(minA > maxB || minB > maxA);
}

extern "C" __global__ void sdfPopulateBackgroundSDF(PxU32 cellsPerSubgrid, PxReal* PX_RESTRICT backgroundSDF, PxU32 backgroundSizeX, PxU32 backgroundSizeY, PxU32 backgroundSizeZ,
	const PxReal* PX_RESTRICT sdf, PxU32 width, PxU32 height, PxU32 depth)
{
	PxI32 id = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (id < backgroundSizeX * backgroundSizeY * backgroundSizeZ)
	{
		PxU32 xBlock, yBlock, zBlock;
		Gu::idToXYZ(id, backgroundSizeX, backgroundSizeY, xBlock, yBlock, zBlock);

		const PxU32 index = Gu::idx3D(xBlock * cellsPerSubgrid, yBlock * cellsPerSubgrid, zBlock * cellsPerSubgrid, width + 1, height + 1);
		backgroundSDF[id] = sdf[index];
	}
}


extern "C" __global__ void
__launch_bounds__(PxgBVHKernelBlockDim::BUILD_SDF, 1)
sdfMarkRequiredSdfSubgrids(PxReal* PX_RESTRICT backgroundSDF, const PxReal* PX_RESTRICT sdf, PxU32* PX_RESTRICT subgridInfo, PxU8* PX_RESTRICT subgridActive, PxU32 cellsPerSubgrid, PxU32 width, PxU32 height, PxU32 depth,
	PxU32 backgroundSizeX, PxU32 backgroundSizeY, PxU32 backgroundSizeZ, PxReal narrowBandThickness, PxReal* subgridGlobalMinValue, PxReal* subgridGlobalMaxValue, PxReal errorThreshold)
{
	__shared__ PxReal sharedMemoryX[PxgBVHKernelBlockDim::BUILD_SDF / WARP_SIZE];
	__shared__ PxReal sharedMemoryY[PxgBVHKernelBlockDim::BUILD_SDF / WARP_SIZE];
	__shared__ PxReal sharedMemoryZ[PxgBVHKernelBlockDim::BUILD_SDF / WARP_SIZE];

	Gu::DenseSDF coarseEval(backgroundSizeX, backgroundSizeY, backgroundSizeZ, backgroundSDF); //TODO: Replace with 3d texture?
	PxReal s = 1.0f / cellsPerSubgrid;

	//A subgrid has pow3(cellsPerSubgrid) cells but pow3(cellsPerSubgrid + 1) samples
	PxU32 numSamplesPerSubgrid = pow3(cellsPerSubgrid + 1);

	PxReal sdfMin = FLT_MAX;
	PxReal sdfMax = -FLT_MAX;
	PxReal maxAbsError = 0.0f;

	for (PxU32 i = threadIdx.x; i < numSamplesPerSubgrid; i += blockDim.x)
	{
		PxU32 xLocal, yLocal, zLocal;
		Gu::idToXYZ(i, cellsPerSubgrid + 1, cellsPerSubgrid + 1, xLocal, yLocal, zLocal);

		PxU32 x = blockIdx.x * cellsPerSubgrid + xLocal;
		PxU32 y = blockIdx.y * cellsPerSubgrid + yLocal;
		PxU32 z = blockIdx.z * cellsPerSubgrid + zLocal;

		const PxU32 index = Gu::idx3D(x, y, z, width + 1, height + 1);
		PxReal sdfValue = sdf[index];

		sdfMin = PxMin(sdfMin, sdfValue);
		sdfMax = PxMax(sdfMax, sdfValue);

		maxAbsError = PxMax(maxAbsError, PxAbs(sdfValue - coarseEval.sampleSDFDirect(PxVec3(blockIdx.x + xLocal * s, blockIdx.y + yLocal * s, blockIdx.z + zLocal * s))));
	}

	__syncthreads();

	sdfMin = blockReduction<MinOpFloat, PxReal>(FULL_MASK, sdfMin, FLT_MAX, blockDim.x, sharedMemoryX);
	sdfMax = blockReduction<MaxOpFloat, PxReal>(FULL_MASK, sdfMax, -FLT_MAX, blockDim.x, sharedMemoryY);
	maxAbsError = blockReduction<MaxOpFloat, PxReal>(FULL_MASK, maxAbsError, -FLT_MAX, blockDim.x, sharedMemoryZ);

	__syncthreads();

	if (threadIdx.x == 0)
	{		
		bool subgridRequired = rangesOverlaps(sdfMin, sdfMax, -narrowBandThickness, narrowBandThickness);
		if (maxAbsError < errorThreshold)
			subgridRequired = false; //No need for a subgrid if the coarse SDF is already almost exact

		PxU32 index = Gu::idx3D(blockIdx.x, blockIdx.y, blockIdx.z, backgroundSizeX - 1, backgroundSizeY - 1);

		if (subgridRequired)
		{
			AtomicMin(subgridGlobalMinValue, sdfMin);
			AtomicMax(subgridGlobalMaxValue, sdfMax);
			
			subgridInfo[index] = 1;
			subgridActive[index] = 1;
		}	
		else
		{
			subgridInfo[index] = 0;
			subgridActive[index] = 0;
		}
	}
}

PX_FORCE_INLINE __device__ void storeQuantized(void* PX_RESTRICT destination, PxU32 index, PxReal vNormalized, PxU32 bytesPerSubgridPixel)
{
	switch (bytesPerSubgridPixel)
	{
	case 1:
	{
		PxU8* ptr8 = reinterpret_cast<PxU8*>(destination);
		ptr8[index] = PxU8(255.0f * PxClamp(vNormalized, 0.0f, 1.0f));
	}
		break;
	case 2:
	{
		PxU16* ptr16 = reinterpret_cast<PxU16*>(destination);
		ptr16[index] = PxU16(65535.0f * PxClamp(vNormalized, 0.0f, 1.0f));
	}
		break;
	default:
		assert(0);
		break;	
	}
}

extern "C" __global__ 
__launch_bounds__(PxgBVHKernelBlockDim::BUILD_SDF, 1)
void sdfPopulateSdfSubgrids(const PxReal* PX_RESTRICT denseSDF, PxU32 width, PxU32 height, PxU32 depth, PxU32* PX_RESTRICT subgridInfo, PxU8* PX_RESTRICT subgridActive, PxU32 subgridSize, PxU32 w, PxU32 h, PxU32 d,
	void* PX_RESTRICT quantizedSparseSDFIn3DTextureFormat, PxU32 numSubgridsX, PxU32 numSubgridsY, PxU32 numSubgridsZ, const PxReal* subgridsMinSdfValue,
	const PxReal* subgridsMaxSdfValue, PxU32 bytesPerSubgridPixel, PxU32 outputSize)
{
	const PxU32 idx = Gu::idx3D(blockIdx.x, blockIdx.y, blockIdx.z, w, h);
	//if (idx >= w*h*d)
	//	printf("out of range 1\n");

	const PxU32 addressInfo = subgridInfo[idx];

	__syncthreads(); //Make sure that all threads in thread block have read the addressInfo

	if (subgridActive[idx] == 0) 
	{
		subgridInfo[idx] = 0xFFFFFFFF;
		return; //Subgrid does not need to be created
	}

	//if (addressInfo == 0xFFFFFFFF)
	//	printf("address %i   %i      %i %i\n", addressInfo, PxU32(activeSubgrids[idx]), w, h);

	PxU32 addressX, addressY, addressZ;
	Gu::idToXYZ(addressInfo, numSubgridsX, numSubgridsY, addressX, addressY, addressZ);

	if (threadIdx.x == 0)
		subgridInfo[idx] = Gu::encodeTriple(addressX, addressY, addressZ);

	//if (addressX >= numSubgridsX || addressY >= numSubgridsY || addressZ >= numSubgridsZ)
	//	printf("kernel, subgrid index out of bounds %i %i %i %i\n", addressX, addressY, addressZ, addressInfo);

	addressX *= (subgridSize + 1);
	addressY *= (subgridSize + 1);
	addressZ *= (subgridSize + 1);

	//A subgrid has pow3(subgridSize) cells but pow3(subgridSize + 1) samples
	PxU32 numSamplesPerSubgrid = pow3(subgridSize + 1);

	PxU32 tex3DsizeX = numSubgridsX * (subgridSize + 1);
	PxU32 tex3DsizeY = numSubgridsY * (subgridSize + 1);
	//PxU32 tex3DsizeZ = numSubgridsZ * (subgridSize + 1);

	for (PxU32 i = threadIdx.x; i < numSamplesPerSubgrid; i += blockDim.x)
	{
		PxU32 xLocal, yLocal, zLocal;
		Gu::idToXYZ(i, subgridSize + 1, subgridSize + 1, xLocal, yLocal, zLocal);

		const PxU32 index = Gu::idx3D(
			blockIdx.x * subgridSize + xLocal, 
			blockIdx.y * subgridSize + yLocal,
			blockIdx.z * subgridSize + zLocal,
			width + 1, height + 1);
	
		/*if(index >= (width+1)*(height + 1)*(depth + 1))
			printf("out of range 2\n");*/

		PxReal sdfValue = denseSDF[index];	
		PxU32 outputIndex = Gu::idx3D(addressX + xLocal, addressY + yLocal, addressZ + zLocal, tex3DsizeX, tex3DsizeY);

		if (outputIndex * bytesPerSubgridPixel < outputSize)
		{
			if (bytesPerSubgridPixel == 4)
			{
				PxReal* ptr32 = reinterpret_cast<PxReal*>(quantizedSparseSDFIn3DTextureFormat);
				ptr32[outputIndex] = sdfValue;
			}
			else
			{
				PxReal s = 1.0f / (subgridsMaxSdfValue[0] - subgridsMinSdfValue[0]);
				PxReal vNormalized = (sdfValue - subgridsMinSdfValue[0]) * s;
				storeQuantized(quantizedSparseSDFIn3DTextureFormat, outputIndex, vNormalized, bytesPerSubgridPixel);
			}
		}
		/*else 
		{
			printf("out of range %i %i   %i %i   %i %i          %i  %i    %i %i\n", addressX, xLocal, addressY, yLocal, addressZ, zLocal, bytesPerSubgridPixel, outputIndex, addressInfo, PxU32(activeSubgrids[idx]));
		}*/
	}
	//__syncthreads();
}

__device__ void findHoles(const PxReal* PX_RESTRICT sdf, const PxU32 width, const PxU32 height, const PxU32 depth, const PxVec3 cellSize,
	PxU32* atomicCounter, const Gu::GridQueryPointSampler* sampler, float4* PX_RESTRICT itemLowers, float4* PX_RESTRICT itemUppers, PxU32 capacity)
{
	PxI32 id = ((blockIdx.x * blockDim.x) + threadIdx.x);

	bool valueChanged = false;
	PxReal newValue = 0.0f;
	PxU32 px, py, pz;

	if (id < width * height * depth)
	{
		PxReal initialValue = sdf[id];
		newValue = PxAbs(initialValue);

		Gu::idToXYZ(id, width, height, px, py, pz);

		for (PxU32 z = PxMax(1u, pz) - 1; z <= PxMin(depth - 1, pz + 1); ++z)
			for (PxU32 y = PxMax(1u, py) - 1; y <= PxMin(height - 1, py + 1); ++y)
				for (PxU32 x = PxMax(1u, px) - 1; x <= PxMin(width - 1, px + 1); ++x)
				{
					if (x == px && y == py && z == pz)
						continue;

					PxU32 index = Gu::idx3D(x, y, z, width, height);
					if (index >= width * height * depth)
						continue;

					PxReal value = sdf[index];

					if (PxSign(initialValue) != PxSign(value))
					{
						PxReal distance = 0;
						if (x != px)
							distance += cellSize.x * cellSize.x;
						if (y != py)
							distance += cellSize.y * cellSize.y;
						if (z != pz)
							distance += cellSize.z * cellSize.z;

						distance = PxSqrt(distance);

						PxReal delta = PxAbs(value - initialValue);

						if (0.99f * delta > distance)
						{
							PxReal scaling = distance / delta;
							PxReal v = 0.99f * scaling * initialValue;
							newValue = PxMin(newValue, PxAbs(v));
						}
					}
				}

		if (initialValue < 0)
			newValue = -newValue;

		valueChanged = newValue != initialValue;
	}

	PxU32 outputIdx = globalScanExclusive<PxgBVHKernelBlockDim::SDF_FIX_HOLES / WARP_SIZE>(valueChanged, atomicCounter);

	if (valueChanged && itemLowers && itemUppers)
	{
		const PxVec3 p = sampler->getPoint(px, py, pz);

		assert(outputIdx < capacity);

		itemLowers[outputIdx] = make_float4(p.x, p.y, p.z, 0.0f);
		itemUppers[outputIdx] = make_float4(px, py, pz, newValue);
	}
}

extern "C" __global__
__launch_bounds__(PxgBVHKernelBlockDim::SDF_FIX_HOLES, 1)
void sdfCountHoles(const PxReal* PX_RESTRICT sdf, const PxU32 width, const PxU32 height, const PxU32 depth, const PxVec3 cellSize,
	PxU32* atomicCounter)
{
	findHoles(sdf, width, height, depth, cellSize, atomicCounter, NULL, NULL, NULL, 0);
}


//If the triangle, mesh which is used to compute the SDF, has a hole, then the sdf values near a sign change will not satisfy the eikonal equation
//This kernel fixes those jumps along sign changes
//Afterwards a jump flood algorithm can be used to fix the vincinity of the signe change
extern "C" __global__ 
__launch_bounds__(PxgBVHKernelBlockDim::SDF_FIX_HOLES, 1)
void sdfFindHoles(const PxReal* PX_RESTRICT sdf, const PxU32 width, const PxU32 height, const PxU32 depth, const PxVec3 cellSize,
	PxU32* atomicCounter, const Gu::GridQueryPointSampler sampler,
	float4* PX_RESTRICT itemLowers, float4* PX_RESTRICT itemUppers, PxU32 capacity)
{
	findHoles(sdf, width, height, depth, cellSize, atomicCounter, &sampler, itemLowers, itemUppers, capacity);
}

extern "C" __global__ void sdfApplyHoleCorrections(PxReal* PX_RESTRICT sdf, PxU32 width, PxU32 height, PxU32 depth, 
	Gu::GridQueryPointSampler sampler,
	PxVec4* PX_RESTRICT itemUppers, PxU32 numCorrections)
{
	PxI32 id = ((blockIdx.x * blockDim.x) + threadIdx.x);

	if (id < numCorrections)
	{
		PxVec4 upper = itemUppers[id];
		PxU32 x = PxU32(upper.x);
		PxU32 y = PxU32(upper.y);
		PxU32 z = PxU32(upper.z);

		const PxVec3 p = sampler.getPoint(x, y, z);

		sdf[Gu::idx3D(x, y, z, width, height)] = upper.w;

		itemUppers[id] = PxVec4(p.x, p.y, p.z, upper.w);
	}
}

//This can be launched on an existing SDF to fix distances given a point cloud where every leaf node was corrected due to a sign change in the SDF causing a gap in distance values larger than the cell size.
//These kind of gaps can occur at places where the input triangle mesh has holes. Watertight meshes don't need this kind of post process repair.
//The fast marching method or jump flood could be used as well to fix those defects but they need either many kernel launches or much more memory compared to the point cloud tree.
extern "C" __global__ __launch_bounds__(256, 1) void sdfCalculateDenseGridPointCloud(PxgBVH bvh, 
	Gu::GridQueryPointSampler sampler, PxU32 sizeX, PxU32 sizeY, PxU32 sizeZ, PxReal* PX_RESTRICT sdfData)
{
	const PxU32 stackSize = 47;
	//__shared__ PxI32 stackMem[256 * stackSize];
	PxI32 stackMem[stackSize];

	// block addressing
	const PxI32 x = blockIdx.x*blockDim.x + threadIdx.x;
	const PxI32 y = blockIdx.y*blockDim.y + threadIdx.y;
	const PxI32 z = blockIdx.z*blockDim.z + threadIdx.z;

	//const PxI32 threadId = threadIdx.z * 8 * 8 + threadIdx.y * 8 + threadIdx.x;

	if (x < sizeX && y < sizeY && z < sizeZ)
	{
		const PxReal prevSdfValue = sdfData[Gu::idx3D(x, y, z, sizeX, sizeY)];

		const PxVec3 p = sampler.getPoint(x, y, z); 

		PxI32* stack = &stackMem[/*stackSize * threadId*/0];

		ClosestDistanceToPointCloudTraversalWithOffset distQuery(p, PxAbs(prevSdfValue));
		queryBVH(bvh, distQuery, stack, stackSize);
		
		PxReal d = distQuery.mClosestDistance;
		if (d < PxAbs(prevSdfValue))
		{
			if (prevSdfValue < 0.0f)
				d = -d;

			sdfData[Gu::idx3D(x, y, z, sizeX, sizeY)] = d;
		}
	}
}
