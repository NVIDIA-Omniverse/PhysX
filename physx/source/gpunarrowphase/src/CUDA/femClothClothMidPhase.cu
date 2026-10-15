// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxBounds3.h"
#include "foundation/PxMath.h"
#include "foundation/PxPlane.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxMeshScale.h"

#include "GuDistancePointTriangle.h"
#include "GuBV32.h"
#include "cudaNpCommon.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgFEMCloth.h"
#include "PxgDeformableContactWriters.h"
#include "PxgNpKernelIndices.h"
#include "PxgSimulationCoreDesc.h"

#include "PxsCachedTransform.h"

#include <vector_types.h>

#include "PxgCommonDefines.h"
#include "dataReadWriteHelper.cuh"
#include "femMidphaseScratch.cuh"
#include "utils.cuh"
#include "deformableElementFilter.cuh"
#include "deformableCollision.cuh"
#include "triangletriangle.cuh"

#include "bv32Traversal.cuh"
#include "triangleMesh.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels21() {}

__device__ void computeTriangleBound2(const uint4* const PX_RESTRICT triIndices, const PxU32 triangleIdx, const float4* const PX_RESTRICT triangleVerts,
	PxBounds3& triangleBound)
{
	const uint4 vertInds = triIndices[triangleIdx];

	const PxVec3 worldV0 = PxLoad3(triangleVerts[vertInds.x]);
	const PxVec3 worldV1 = PxLoad3(triangleVerts[vertInds.y]);
	const PxVec3 worldV2 = PxLoad3(triangleVerts[vertInds.z]);

	triangleBound = triBoundingBox(worldV0, worldV1, worldV2);
}

struct TriangleLeafBoundMinMaxTraverser
{
	femMidphaseScratch* PX_RESTRICT s_warpScratch;
	Triangle* mTriangle;
	const PxVec3 tMin;
	PxReal mContactDistance;
	const PxVec3 tMax;
	PxReal mBestDistance;
	const physx::PxU32 triangleIndex;

	//output
	PxU32 mPrimIndex;

	PX_FORCE_INLINE __device__ TriangleLeafBoundMinMaxTraverser(femMidphaseScratch* PX_RESTRICT s_warpScratch, physx::PxU32 triangleIndex, const PxBounds3& triangleBound,
		Triangle* triangle, const PxReal contactDistance) :
		s_warpScratch(s_warpScratch), triangleIndex(triangleIndex), tMin(triangleBound.minimum), tMax(triangleBound.maximum), mTriangle(triangle), mContactDistance(contactDistance), mBestDistance(PX_MAX_F32),
		mPrimIndex(0xFFFFFFFF)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			using namespace physx;

			const uint4* trimeshTriangleIndices = s_warpScratch->meshVertsIndices;
			const float4 * trimeshVerts = s_warpScratch->meshVerts;

			const uint4 triIdx = trimeshTriangleIndices[primitiveIndex];

			const PxVec3 v0InVertexSpace = PxLoad3(trimeshVerts[triIdx.x]);
			const PxVec3 v1InVertexSpace = PxLoad3(trimeshVerts[triIdx.y]);
			const PxVec3 v2InVertexSpace = PxLoad3(trimeshVerts[triIdx.z]);

			Triangle triangle1;
			triangle1.verts[0] = v0InVertexSpace;
			triangle1.verts[1] = v1InVertexSpace;
			triangle1.verts[2] = v2InVertexSpace;
			triangle1.triPlane = PxPlane(v0InVertexSpace, v1InVertexSpace, v2InVertexSpace);

			//get rid of double-sided triangle
			if (triangle1.triPlane.distance(mTriangle->centroid) >= 0)
			{
				const PxReal separation = satIntersect<true>(*mTriangle, triangle1, PxMin(mBestDistance, mContactDistance));

				if (separation < mBestDistance)
				{
					mBestDistance = separation;
					mPrimIndex = primitiveIndex;
				}
			}
		}

		//return mBestDistance < mContactDistance;
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return !(min.x > tMax.x || tMin.x > max.x || min.y > tMax.y || tMin.y > max.y || min.z > tMax.z || tMin.z > max.z);
		else
			return false;
	}
};

template<unsigned int WarpsPerBlock>
__device__ static inline void femClothTriMeshMidphaseCore(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,				//output
	PxU32* PX_RESTRICT midphasePairsNum,	//output
	femMidphaseScratch*	s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;
	// each block deal with one pair
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;

		PxgShape shape0, shape1;
		PxgShape* trimeshShape = NULL;
		PxgShape* clothShape = NULL;
		PxU32 trimeshCacheRef, clothCacheRef;
		{
			PxgContactManagerInput npWorkItem;
			PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);
			PxgShape_ReadWarp(shape0, gpuShapes + npWorkItem.shapeRef0);
			PxgShape_ReadWarp(shape1, gpuShapes + npWorkItem.shapeRef1);
			assert(shape0.type == PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eTRIANGLEMESH);

			if(shape0.particleOrSoftbodyId != 0xffffffff)
			{
				trimeshShape = &shape1;
				trimeshCacheRef = npWorkItem.transformCacheRef1;
				clothShape = &shape0;
				clothCacheRef = npWorkItem.transformCacheRef0;
			}
			else
			{
				trimeshShape = &shape0;
				trimeshCacheRef = npWorkItem.transformCacheRef0;
				clothShape = &shape1;
				clothCacheRef = npWorkItem.transformCacheRef1;
			}
			assert(trimeshShape->particleOrSoftbodyId == 0xffffffff);
		}

		PxU32 clothId = clothShape->particleOrSoftbodyId;
		const PxgFEMCloth* cloth = &femClothes[clothId];

		PxsCachedTransform trimeshTransformCache;
		PxsCachedTransform_ReadWarp(trimeshTransformCache, transformCache + trimeshCacheRef);

		if (threadIdx.x == 0)
		{
			const PxU8* triGeomPtr = reinterpret_cast<const PxU8*>(trimeshShape->hullOrMeshPtr);			
			readTriangleMesh(triGeomPtr, s_warpScratch->bv32PackedNodes, s_warpScratch->meshVerts, s_warpScratch->meshVertsIndices);
		}
		__syncthreads();

		const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[trimeshCacheRef];

		const PxU32 nbTriangles = cloth->mNbTriangles; // s_warpScratch->nbPrimitives[0];

		const PxU32 NbWarps = blockDim.y * gridDim.x;

		const uint4* vertIndices = cloth->mTriangleVertexIndices_ownedEdges;   // s_warpScratch->trimeshVertsIndices[0];
		const float4* triangleVerts = cloth->mPosition_InvMass; // s_warpScratch->trimeshVerts[0];

		PxBounds3 triangleBound;
		PxTransform& trimeshToWorld = trimeshTransformCache.transform;
		const PxMeshScale& trimeshScale = trimeshShape->scale;

		for(PxU32 triangleIdx = globalWarpIdx; triangleIdx < nbTriangles; triangleIdx += NbWarps)
		{
			const uint4 vertInds = vertIndices[triangleIdx];

			const PxVec3 worldV0 = PxLoad3(triangleVerts[vertInds.x]);
			const PxVec3 worldV1 = PxLoad3(triangleVerts[vertInds.y]);
			const PxVec3 worldV2 = PxLoad3(triangleVerts[vertInds.z]);

			/// transform those point to triangle vertex space
			const PxVec3 v0 =
			    shape2Vertex(trimeshToWorld.transformInv(worldV0), trimeshScale.scale, trimeshScale.rotation);
			const PxVec3 v1 =
			    shape2Vertex(trimeshToWorld.transformInv(worldV1), trimeshScale.scale, trimeshScale.rotation);
			const PxVec3 v2 =
			    shape2Vertex(trimeshToWorld.transformInv(worldV2), trimeshScale.scale, trimeshScale.rotation);

			PxReal tX0 = PxMin(v0.x, v1.x);
			PxReal tY0 = PxMin(v0.y, v1.y);
			PxReal tZ0 = PxMin(v0.z, v1.z);

			triangleBound.minimum.x = PxMin(tX0, v2.x);
			triangleBound.minimum.y = PxMin(tY0, v2.y);
			triangleBound.minimum.z = PxMin(tZ0, v2.z);

			// compute max
			tX0 = PxMax(v0.x, v1.x);
			tY0 = PxMax(v0.y, v1.y);
			tZ0 = PxMax(v0.z, v1.z);

			triangleBound.maximum.x = PxMax(tX0, v2.x);
			triangleBound.maximum.y = PxMax(tY0, v2.y);
			triangleBound.maximum.z = PxMax(tZ0, v2.z);

			// bound in triangel vertext space
			triangleBound.fattenFast(contactDist);

			uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
			const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

			Triangle triangle;
			triangle.verts[0] = v0;
			triangle.verts[1] = v1;
			triangle.verts[2] = v2;
			triangle.triPlane = PxPlane(v0, v1, v2);
			triangle.centroid = (v0 + v1 + v2) / 3.f;

			TriangleLeafBoundMinMaxTraverser op(s_warpScratch, triangleIdx, triangleBound, &triangle, contactDist);
			bv32TreeTraversal<TriangleLeafBoundMinMaxTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes,
			                                                                   s_warpScratch->sBv32Nodes, op);

			const PxU32 MaxNumPairs = 4;

			PxU32 mask = __ballot_sync(FULL_MASK, op.mBestDistance < contactDist);

			PxU32 count = PxMin(PxU32(__popc(mask)), MaxNumPairs);

			if(count)
			{
				PxU32 index;
				if(threadIdx.x == 0)
				{
					index = atomicAdd(midphasePairsNum, count);
				}

				index = __shfl_sync(FULL_MASK, index, 0);

				PxReal bestSep = -1.f;
				for(PxU32 i = 0; i < count && bestSep < contactDist; i++)
				{
					PxU32 winnerLane = minIndex(op.mBestDistance, FULL_MASK, bestSep);

					if(threadIdx.x == winnerLane)
					{
						if(op.mBestDistance < contactDist)
						{
							if((index + i) < stackSize)
								tStackPtr[index + i] = make_uint4(cmIdx, triangleIdx, op.mPrimIndex, 0);
							op.mBestDistance = PX_MAX_F32;
						}
					}
				}
			}
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_meshMidphaseGeneratePairsLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	femClothTriMeshMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		femClothes,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

// Computes the squared distance between a vertex and a triangle in rest pose.
// Approximates rest distances using vertex distances for better performance.
__device__ PxReal distSqInRestPose(const float4* PX_RESTRICT restVerts, const float4* PX_RESTRICT restVertexBuffer, PxU32 restVertexIndex,
								   const uint4 triIndex)
{
	if(!restVerts)
	{
		return PX_MAX_REAL;
	}

	const PxVec3 x1 = PxLoad3(restVerts[triIndex.x]);
	const PxVec3 x2 = PxLoad3(restVerts[triIndex.y]);
	const PxVec3 x3 = PxLoad3(restVerts[triIndex.z]);

	const PxVec3 x0 = PxLoad3(restVertexBuffer[restVertexIndex]);

	const PxReal d1 = (x1 - x0).magnitudeSquared();
	const PxReal d2 = (x2 - x0).magnitudeSquared();
	const PxReal d3 = (x3 - x0).magnitudeSquared();

	return PxMin(PxMin(d1, d2), d3);
}

// Computes the squared distance between two edges in rest pose.
// Approximates rest distances using vertex distances for better performance.
__device__ PxReal distSqInRestPose(const float4* PX_RESTRICT restVerts0, const float4* PX_RESTRICT restVerts1, PxU32 edge0_vertexIndex0,
								   PxU32 edge0_vertexIndex1, PxU32 edge1_vertexIndex0, PxU32 edge1_vertexIndex1)
{
	if(!restVerts0)
	{
		return PX_MAX_REAL;
	}

	const PxVec3 e0_x0 = PxLoad3(restVerts0[edge0_vertexIndex0]);
	const PxVec3 e0_x1 = PxLoad3(restVerts0[edge0_vertexIndex1]);

	const PxVec3 e1_x0 = PxLoad3(restVerts1[edge1_vertexIndex0]);
	const PxVec3 e1_x1 = PxLoad3(restVerts1[edge1_vertexIndex1]);

	// Compute squared distances in parallel
	const PxReal d0 = (e0_x0 - e1_x0).magnitudeSquared();
	const PxReal d1 = (e0_x0 - e1_x1).magnitudeSquared();
	const PxReal d2 = (e0_x1 - e1_x0).magnitudeSquared();
	const PxReal d3 = (e0_x1 - e1_x1).magnitudeSquared();

	return PxMin(PxMin(d0, d1), PxMin(d2, d3));
}

// Cloth0 vertices traverse the BVH of cloth1 triangles.
struct ClothTreeVertexTraverser
{
	const PxU32 contactSize;
	physx::PxU32 vertexIdx;
	const PxReal contactDist;
	const PxReal filterDistSq;
	const PxU32 clothId0;
	const PxU32 clothId1;
	const float4* restVerts0;
	const uint4* triIndices;
	PxVec3 vertex;
	const PxgDeformablePairInfo* surfaceSurfaceFilterPairs;
	const PxU32 numSurfaceSurfaceFilterPairs;
	PxgDeformablePairInfo* outContactInfo;	// output
	PxU32* totalContactCount;					// output
	const float4* vertices1;
	const float4* restVerts1;

	PX_FORCE_INLINE __device__ ClothTreeVertexTraverser(
		const PxU32 contactSize, const PxReal contactDist,
		const PxReal filterDistSq, const PxU32 clothId0, const PxU32 clothId1,
		const float4* restVerts0, const uint4* triIndices, const PxgDeformablePairInfo* surfaceSurfaceFilterPairs,
		const PxU32 numSurfaceSurfaceFilterPairs,
		PxgDeformablePairInfo* outContactInfo,	// output
		PxU32* totalContactCount,					// output
		const float4* vertices1,
		const float4* restVerts1)
	: contactSize(contactSize)
	, contactDist(contactDist)
	, filterDistSq(filterDistSq)
	, clothId0(clothId0)
	, clothId1(clothId1)
	, restVerts0(restVerts0)
	, triIndices(triIndices)
	, surfaceSurfaceFilterPairs(surfaceSurfaceFilterPairs)
	, numSurfaceSurfaceFilterPairs(numSurfaceSurfaceFilterPairs)
	, outContactInfo(outContactInfo)
	, totalContactCount(totalContactCount)
	, vertices1(vertices1)
	, restVerts1(restVerts1)
	{
	}

	// Filters triangles of the other surface, given the input vertex of this surface. Also checks
	// whether the other surface's triangle is filtered against any.
	PX_FORCE_INLINE __device__ bool isFiltered(PxU32 triangleIndex1) const
	{
		return findSurfaceSurfaceVertexTriangleFilter(surfaceSurfaceFilterPairs, numSurfaceSurfaceFilterPairs,
				   clothId0, vertexIdx, clothId1, triangleIndex1) ||
			   findSurfaceSurfaceVertexTriangleFilter(surfaceSurfaceFilterPairs, numSurfaceSurfaceFilterPairs,
				   clothId0, PxgDeformableRef::ANY_FEATURE, clothId1, triangleIndex1);
	}

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		PxReal s, t;

		bool intersect = false;
		if(primitiveIndex != 0xFFFFFFFF)
		{
			const uint4 triVtxIndices1 = triIndices[primitiveIndex];

			// Check if the vertex is part of the triangle
			if(clothId0 != clothId1 | !(vertexIdx == triVtxIndices1.x | vertexIdx == triVtxIndices1.y | vertexIdx == triVtxIndices1.z))
			{
				const PxVec3 x1 = PxLoad3(vertices1[triVtxIndices1.x]);
				const PxVec3 x2 = PxLoad3(vertices1[triVtxIndices1.y]);
				const PxVec3 x3 = PxLoad3(vertices1[triVtxIndices1.z]);

				PxBounds3 bounds;
				bounds.minimum = x1;
				bounds.maximum = x1;
				bounds.include(x2);
				bounds.include(x3);
				bounds.fattenFast(contactDist);
				if(bounds.contains(vertex) && !isFiltered(primitiveIndex))
				{
					const PxVec3 closestPt = closestPtPointTriangle(vertex, x1, x2, x3, s, t);

					PxVec3 dir = vertex - closestPt;
					const PxReal distSq = dir.magnitudeSquared();

					if(distSq < contactDist * contactDist && distSq > 1.0e-14f)
					{
						// Compute approximated rest distance for filtering
						const PxReal tempRestDistSq = distSqInRestPose(restVerts0, restVerts1, vertexIdx, triVtxIndices1);

						// Check if the objects are closer than the filter distance threshold.
						if(tempRestDistSq > filterDistSq)
						{
							intersect = true;
						}
					}
				}
			}
		}

		const PxU32 index = globalScanExclusiveSingleWarp(intersect, totalContactCount);

		if(intersect & index < contactSize)
		{
			PxgDeformablePairInfo contactInfo;

			contactInfo.ref0 = PxgEncodeDeformableVertex(clothId0, vertexIdx);
			contactInfo.ref1 = PxgEncodeDeformableTriangle(clothId1, primitiveIndex);

			outContactInfo[index] = contactInfo;
		}
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if(hasBox)
		{
			PxVec3 closest = min.maximum(vertex.minimum(max));
			return (closest - vertex).magnitudeSquared() <= contactDist * contactDist;
		}
		else
			return false;
	}
};

// Cloth0 edges (pair0) traverse the BVH of Cloth1 edges (pair1).
// Triangle BVH is used, as edges are encoded per triangle.

struct ClothTreeEdgeTraverser
{
	const PxU32 contactSize;
	const PxReal contactDist;
	const PxReal filterDistSq;
	const PxU32 clothId0;
	const PxU32 clothId1;
	const float4* restVerts0;
	PxVec3 t0_points[3];							// set by setEdge0()
	PxBounds3 t0_bounds;							// set by setEdge0()
	PxU32 t0_index;									// set by setEdge0()
	uint4 t0_triVertIndices;						// set by setEdge0()
	const PxgDeformablePairInfo* surfaceSurfaceFilterPairs;
	const PxU32 numSurfaceSurfaceFilterPairs;
	PxgDeformablePairInfo* outContactInfo;			// output
	PxU32* totalContactCount;							// output
	const float4* vertices1;
	const float4* restVerts1;
	const uint4* triVertIndices1;

	PX_FORCE_INLINE __device__ ClothTreeEdgeTraverser(
		const PxU32 contactSize, const PxReal contactDist,
		const PxReal filterDistSq, const PxU32 clothId0, const PxU32 clothId1,
		const float4* restVerts0,
		const PxgDeformablePairInfo* surfaceSurfaceFilterPairs,
		const PxU32 numSurfaceSurfaceFilterPairs,
		PxgDeformablePairInfo* outContactInfo,		// output
		PxU32* totalContactCount,						// output
		const float4* vertices1,
		const float4* restVerts1,
		const uint4* triVertIndices1)
		: contactSize(contactSize)
		, contactDist(contactDist)
		, filterDistSq(filterDistSq)
		, clothId0(clothId0)
		, clothId1(clothId1)
		, restVerts0(restVerts0)
		, surfaceSurfaceFilterPairs(surfaceSurfaceFilterPairs)
		, numSurfaceSurfaceFilterPairs(numSurfaceSurfaceFilterPairs)
		, outContactInfo(outContactInfo)
		, totalContactCount(totalContactCount)
		, vertices1(vertices1)
		, restVerts1(restVerts1)
		, triVertIndices1(triVertIndices1)
	{
	}

	PX_FORCE_INLINE __device__ void setEdge0(const uint4 triVertIndices, const float4* PX_RESTRICT vertices, PxU32 triIdx)
	{
		t0_triVertIndices = triVertIndices;
		t0_index = triIdx;

		t0_points[0] = PxLoad3(vertices[triVertIndices.x]);
		t0_points[1] = PxLoad3(vertices[triVertIndices.y]);
		t0_points[2] = PxLoad3(vertices[triVertIndices.z]);

		t0_bounds.minimum = t0_points[0];
		t0_bounds.maximum = t0_points[0];
		t0_bounds.include(t0_points[1]);
		t0_bounds.include(t0_points[2]);
		t0_bounds.fattenFast(contactDist);
	}

	// Filters edges of the other surface, given the input edge of this surface. Also checks whether either
	// edge is filtered against any.
	PX_FORCE_INLINE __device__ bool isFiltered(PxU32 localEdge0, PxU32 triangleIndex1, PxU32 localEdge1) const
	{
		return findSurfaceSurfaceEdgeEdgeFilter(surfaceSurfaceFilterPairs, numSurfaceSurfaceFilterPairs,
				   clothId0, t0_index, localEdge0, clothId1, triangleIndex1, localEdge1) ||
			   findSurfaceSurfaceEdgeEdgeFilter(surfaceSurfaceFilterPairs, numSurfaceSurfaceFilterPairs,
				   clothId0, t0_index, localEdge0, clothId1, PxgDeformableRef::ANY_FEATURE, 0) ||
			   findSurfaceSurfaceEdgeEdgeFilter(surfaceSurfaceFilterPairs, numSurfaceSurfaceFilterPairs,
				   clothId0, PxgDeformableRef::ANY_FEATURE, 0, clothId1, triangleIndex1, localEdge1);
	}

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		uint2 localEdgeIndices[9]; // Up to three edges per triangle
		PxU32 intersectCount = 0;

		if(primitiveIndex != 0xFFFFFFFF & (clothId0 != clothId1 | t0_index > primitiveIndex))
		{
			const uint4 t1_triVertIndices = triVertIndices1[primitiveIndex];
			const PxU32 t1_edgeOwnership = t1_triVertIndices.w;

			static const PxU32 nextE_lookup[3] = { 1, 2, 0 };
			const PxU32 t1_vIndices[3] = { t1_triVertIndices.x, t1_triVertIndices.y, t1_triVertIndices.z };

			PxBounds3 t1_bounds;

			const PxVec3 t1_points[3] = { PxLoad3(vertices1[t1_vIndices[0]]), PxLoad3(vertices1[t1_vIndices[1]]),
										  PxLoad3(vertices1[t1_vIndices[2]]) };

			t1_bounds.minimum = t1_points[0];
			t1_bounds.maximum = t1_points[0];
			t1_bounds.include(t1_points[1]);
			t1_bounds.include(t1_points[2]);

			if(t0_bounds.intersects(t1_bounds))
			{
				const PxReal contactDistSq = contactDist * contactDist;

				const PxU32 t0_edgeOwnership = t0_triVertIndices.w;
				const PxU32 t0_vIndices[3] = { t0_triVertIndices.x, t0_triVertIndices.y, t0_triVertIndices.z };

				// Iterate through edges of triangle 1
				for(PxU32 e1 = 0; e1 < 3; ++e1)
				{
					const bool edgeOwned1 = ownsEdge(t1_edgeOwnership, e1);
					if(!edgeOwned1)
					{
						continue;
					}

					const PxU32 nextE1 = nextE_lookup[e1];
					const PxU32 e1_v0 = t1_vIndices[e1];
					const PxU32 e1_v1 = t1_vIndices[nextE1];

					const PxVec3& e1_x0 = t1_points[e1];
					const PxVec3& e1_x1 = t1_points[nextE1];

#pragma unroll
					// Iterate through edges of triangle 0
					for(PxU32 e0 = 0; e0 < 3; ++e0)
					{
						const bool edgeOwned0 = ownsEdge(t0_edgeOwnership, e0);
						if(!edgeOwned0)
						{
							continue;
						}

						const PxU32 nextE0 = nextE_lookup[e0];

						const PxU32 e0_v0 = t0_vIndices[e0];
						const PxU32 e0_v1 = t0_vIndices[nextE0];

						// Skip connected edges within the same cloth
						if((clothId0 == clothId1) & ((e0_v0 == e1_v0) | (e0_v0 == e1_v1) | (e0_v1 == e1_v0) | (e0_v1 == e1_v1)))
						{
							continue;
						}

						const PxVec3& e0_x0 = t0_points[e0];
						const PxVec3& e0_x1 = t0_points[nextE0];

						PxReal s, t;
						PxReal distSq;

						closestPtLineLine(e0_x0, e0_x1, e1_x0, e1_x1, s, t, distSq);
						const bool isOutside = ((s < DEFORMABLE_BARYCENTRIC_THRESHOLD) || (s > DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD) ||
												(t < DEFORMABLE_BARYCENTRIC_THRESHOLD) || (t > DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD));

						// Vertex-edge collisions are handled in the vertex-triangle query.
						if(isOutside)
						{
							continue;
						}

						if(distSq < contactDistSq && distSq > 1.0e-14f)
						{
							// Compute approximated rest distance for filtering
							const PxReal tempRestDistSq = distSqInRestPose(restVerts0, restVerts1, e0_v0, e0_v1, e1_v0, e1_v1);

							if(tempRestDistSq > filterDistSq && !isFiltered(e0, primitiveIndex, e1))
							{
								localEdgeIndices[intersectCount] = make_uint2(e0, e1);
								++intersectCount;
							}
						}
					}
				}
			}
		}

		// Exit early if there is no intersection.
		const PxU32 resultWarp = __ballot_sync(FULL_MASK, intersectCount > 0);
		const PxU32 validCount = __popc(resultWarp);
		if(validCount != 0)
		{
			// Add multiple elements per thread.
			const PxU32 index = globalScanExclusiveSingleWarp(intersectCount, totalContactCount);

			// Per-thread guard: validCount above is warp-aggregate, so lanes without an intersection
			// reach here. Those must not encode a contact, because their primitiveIndex can be the
			// 0xFFFFFFFF empty-leaf sentinel, which trips the encoder's assert. Bug 6156209 / OMPE-92966.
			if(index < contactSize && intersectCount)
			{
				// The scan reserves intersectCount slots from index, so the whole run has to fit.
				// Bounding only the first slot lets the tail write past outContactInfo when the
				// buffer is nearly full. The dropped contacts are reported through the count, which
				// cloth_clothContactPrepareLaunch caps afterwards.
				const PxU32 writeCount = PxMin(intersectCount, contactSize - index);

				PxgDeformablePairInfo contactInfo;

#pragma unroll
				for(PxU32 localIndex = 0; localIndex < writeCount; ++localIndex)
				{
					const uint2 localEdges = localEdgeIndices[localIndex];
					contactInfo.ref0 = PxgEncodeDeformableTriangleEdge(clothId0, t0_index, localEdges.x);
					contactInfo.ref1 = PxgEncodeDeformableTriangleEdge(clothId1, primitiveIndex, localEdges.y);

					const PxU32 outIndex = index + localIndex;
					outContactInfo[outIndex] = contactInfo;
				}
			}
		}
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if(!hasBox)
		{
			return false;
		}

		PxBounds3 t1_bounds(min, max);
		return t1_bounds.intersects(t0_bounds);
	}
};

template<unsigned int WarpsPerBlock>
__device__ static inline void cloth_selfCollisionMidphaseCoreVT(
	const PxU32						maxContacts,
	const PxU32*					activeClothes,
	const PxReal* PX_RESTRICT		contactDistance,
	const PxgFEMCloth* PX_RESTRICT	clothes,
	PxgDeformablePairInfo*			outContactInfo,		//output
	PxU32*							totalContactCount,	//output
	femMidphaseScratch*				s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;
	const PxU32 clothId = activeClothes[cmIdx];
	const PxgFEMCloth* cloth = &clothes[clothId];

	if(cloth->mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION)
	{
		return;
	}

	const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 numVerts = cloth->mNbVerts;

	if(globalWarpIdx >= numVerts)
	{
		return;
	}

	const PxU32 elementInd = cloth->mElementIndex;
	const float4* restPosition = cloth->mRestPosition;
	const PxReal contactDist = contactDistance[elementInd] * 2.f;
	const PxReal filterDistance = cloth->mSelfCollisionFilterDistance;

	if(threadIdx.x == 0)
	{
		s_warpScratch->meshVerts = cloth->mPosition_InvMass;
		s_warpScratch->meshVertsIndices = cloth->mTriangleVertexIndices_ownedEdges;

		PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(cloth->mTriMeshData);
		trimeshGeomPtr += sizeof(uint4);

		Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
		s_warpScratch->bv32PackedNodes = bv32PackedNodes;
	}

	__syncthreads();

	// Checks whether the own vertex is filtered against any.
	if(findSurfaceSurfaceVertexTriangleFilter(cloth->mSurfaceSurfaceFilterPairs, cloth->mNumSurfaceSurfaceFilterPairs,
		clothId, globalWarpIdx, clothId, PxgDeformableRef::ANY_FEATURE))
	{
		return;
	}

	const float4* triVerts = s_warpScratch->meshVerts;
	ClothTreeVertexTraverser traverser(maxContacts, contactDist, filterDistance * filterDistance, clothId, clothId, restPosition,
									   s_warpScratch->meshVertsIndices, cloth->mSurfaceSurfaceFilterPairs,
									   cloth->mNumSurfaceSurfaceFilterPairs, outContactInfo,
									   totalContactCount, s_warpScratch->meshVerts, restPosition);

	traverser.vertexIdx = globalWarpIdx;
	traverser.vertex = PxLoad3(triVerts[globalWarpIdx]);
	bv32TreeTraversal<ClothTreeVertexTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
}

template<unsigned int WarpsPerBlock>
__device__ static inline void cloth_selfCollisionMidphaseCoreEE(
	const PxU32						maxContacts,
	const PxU32*					activeClothes,
	const PxReal* PX_RESTRICT		contactDistance,
	const PxgFEMCloth* PX_RESTRICT	clothes,
	PxgDeformablePairInfo*			outContactInfo,		//output
	PxU32*							totalContactCount,	//output
	femMidphaseScratch*				s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;
	const PxU32 clothId = activeClothes[cmIdx];
	const PxgFEMCloth* cloth = &clothes[clothId];

	if(cloth->mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION)
	{
		return;
	}

	const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 numTriangles = cloth->mNbTrianglesWithOwnedEdges;

	if(globalWarpIdx >= numTriangles)
	{
		return;
	}

	const PxU32 elementInd = cloth->mElementIndex;
	const float4* restPosition = cloth->mRestPosition;
	const PxReal contactDist = contactDistance[elementInd] * 2.f;
	const PxReal filterDistance = cloth->mSelfCollisionFilterDistance;

	if (threadIdx.x == 0)
	{
		s_warpScratch->meshVerts = cloth->mPosition_InvMass;
		s_warpScratch->meshVertsIndices = cloth->mTriangleVertexIndices_ownedEdges;

		PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(cloth->mTriMeshData);
		trimeshGeomPtr += sizeof(uint4);

		Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
		s_warpScratch->bv32PackedNodes = bv32PackedNodes;
	}

	__syncthreads();

	ClothTreeEdgeTraverser traverser(maxContacts, contactDist, filterDistance * filterDistance, clothId, clothId, restPosition,
									 cloth->mSurfaceSurfaceFilterPairs, cloth->mNumSurfaceSurfaceFilterPairs,
									 outContactInfo, totalContactCount, s_warpScratch->meshVerts, restPosition,
									 s_warpScratch->meshVertsIndices);

	const PxU32 triIdx = cloth->mTrianglesWithOwnedEdges[globalWarpIdx];
	const uint4 triVertices = s_warpScratch->meshVertsIndices[triIdx];
	traverser.setEdge0(triVertices, s_warpScratch->meshVerts, triIdx);
	bv32TreeTraversal<ClothTreeEdgeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
}

extern "C" __global__
__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 16) //At least 16 blocks per multiprocessor helps with performance
void cloth_selfCollisionMidphaseVTLaunch(
	const PxU32						maxContacts,
	const PxU32*					activeClothes,
	const PxReal* PX_RESTRICT		contactDistance,
	const PxgFEMCloth* PX_RESTRICT	clothes,
	const PxU8*						updateContactPairs,
	PxgDeformablePairInfo*			outContactInfo,		//output
	PxU32*							totalContactCount	//output
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	cloth_selfCollisionMidphaseCoreVT<MIDPHASE_WARPS_PER_BLOCK>(
		maxContacts,
		activeClothes,
		contactDistance,
		clothes,
		outContactInfo,
		totalContactCount,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

extern "C" __global__
__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 16) //At least 16 blocks per multiprocessor helps with performance
void cloth_selfCollisionMidphaseEELaunch(
	const PxU32						maxContacts,
	const PxU32*					activeClothes,
	const PxReal* PX_RESTRICT		contactDistance,
	const PxgFEMCloth* PX_RESTRICT	clothes,
	const PxU8*						updateContactPairs,
	PxgDeformablePairInfo*			outContactInfo,		//output
	PxU32*							totalContactCount	//output
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	cloth_selfCollisionMidphaseCoreEE<MIDPHASE_WARPS_PER_BLOCK>(
		maxContacts,
		activeClothes,
		contactDistance,
		clothes,
		outContactInfo,
		totalContactCount,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

template<unsigned int WarpsPerBlock>
__device__ static inline void femDifferentClothCollisionCoreVT(
	const PxU32									maxContacts,
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,
	PxgDeformablePairInfo*						outContactInfo,		//output
	PxU32*										totalContactCount,	//output
	femMidphaseScratch*							s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;

	const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;

	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

	PxgShape shape0;
	PxgShape_ReadWarp(shape0, gpuShapes + npWorkItem.shapeRef0);

	PxgShape shape1;
	PxgShape_ReadWarp(shape1, gpuShapes + npWorkItem.shapeRef1);

	assert(shape0.type == PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eTRIANGLEMESH);

	const int clothId0 = shape0.particleOrSoftbodyId;
	const int clothId1 = shape1.particleOrSoftbodyId;

	const PxgFEMCloth* cloth0 = &femClothes[clothId0];
	const PxgFEMCloth* cloth1 = &femClothes[clothId1];

	const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];

	const PxU32 numVertices0 = cloth0->mNbVerts;
	const PxU32 numVertices1 = cloth1->mNbVerts;

	// VT
	if(globalWarpIdx < numVertices0 + numVertices1)
	{
		const bool isCloth0 = globalWarpIdx < numVertices0;

		PxU32 vertexIndex = isCloth0 ? globalWarpIdx : globalWarpIdx - numVertices0;
		const PxgFEMCloth* myCloth = isCloth0 ? cloth0 : cloth1;
		const PxgFEMCloth* otherCloth = isCloth0 ? cloth1 : cloth0;

		const int mask = static_cast<int>(!isCloth0);
		const int myClothId = mask * (clothId1 - clothId0) + clothId0;
		const int otherClothId = mask * (clothId0 - clothId1) + clothId1;

		if(threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = otherCloth->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = otherCloth->mTriangleVertexIndices_ownedEdges;

			PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(otherCloth->mTriMeshData);

			trimeshGeomPtr += sizeof(uint4);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		// Checks whether the own vertex is filtered against any.
		if(findSurfaceSurfaceVertexTriangleFilter(myCloth->mSurfaceSurfaceFilterPairs, myCloth->mNumSurfaceSurfaceFilterPairs,
			myClothId, vertexIndex, otherClothId, PxgDeformableRef::ANY_FEATURE))
		{
			return;
		}

		const float4* verts0 = myCloth->mPosition_InvMass;
		ClothTreeVertexTraverser traverser(maxContacts, contactDist, 0.0f, myClothId, otherClothId, NULL, s_warpScratch->meshVertsIndices,
										   myCloth->mSurfaceSurfaceFilterPairs, myCloth->mNumSurfaceSurfaceFilterPairs,
										   outContactInfo, totalContactCount, s_warpScratch->meshVerts,
										   NULL);

		traverser.vertexIdx = vertexIndex;
		traverser.vertex = PxLoad3(verts0[vertexIndex]);
		bv32TreeTraversal<ClothTreeVertexTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
	}
}

template<unsigned int WarpsPerBlock>
__device__ static inline void femDifferentClothCollisionCoreEE(
	const PxU32									maxContacts,
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,
	PxgDeformablePairInfo*						outContactInfo,		//output
	PxU32*										totalContactCount,	//output
	femMidphaseScratch*							s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;
	const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;

	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

	PxgShape shape0;
	PxgShape_ReadWarp(shape0, gpuShapes + npWorkItem.shapeRef0);

	PxgShape shape1;
	PxgShape_ReadWarp(shape1, gpuShapes + npWorkItem.shapeRef1);

	assert(shape0.type == PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eTRIANGLEMESH);

	const int clothId0 = shape0.particleOrSoftbodyId;
	const int clothId1 = shape1.particleOrSoftbodyId;

	const PxgFEMCloth* cloth0 = &femClothes[clothId0];
	const PxgFEMCloth* cloth1 = &femClothes[clothId1];

	const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];

	const PxU32 numTriangles0 = cloth0->mNbTrianglesWithOwnedEdges;
	const PxU32 numTriangles1 = cloth1->mNbTrianglesWithOwnedEdges;

	if(globalWarpIdx < PxMin(numTriangles0, numTriangles1))
	{
		const bool isCloth0 = numTriangles0 <= numTriangles1;

		const PxgFEMCloth* myCloth = isCloth0 ? cloth0 : cloth1;
		const PxgFEMCloth* otherCloth = isCloth0 ? cloth1 : cloth0;

		const int mask = static_cast<int>(!isCloth0);
		const int myClothId = mask * (clothId1 - clothId0) + clothId0;
		const int otherClothId = mask * (clothId0 - clothId1) + clothId1;

		if(threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = otherCloth->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = otherCloth->mTriangleVertexIndices_ownedEdges;

			PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(otherCloth->mTriMeshData);

			trimeshGeomPtr += sizeof(uint4);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		const float4* verts0 = myCloth->mPosition_InvMass;
		ClothTreeEdgeTraverser traverser(maxContacts, contactDist, 0.0f, myClothId, otherClothId, NULL,
										 myCloth->mSurfaceSurfaceFilterPairs, myCloth->mNumSurfaceSurfaceFilterPairs,
										 outContactInfo, totalContactCount, s_warpScratch->meshVerts, NULL,
										 s_warpScratch->meshVertsIndices);

		const PxU32 triIdx = myCloth->mTrianglesWithOwnedEdges[globalWarpIdx];
		const uint4 triVertices = myCloth->mTriangleVertexIndices_ownedEdges[triIdx];
		traverser.setEdge0(triVertices, verts0, triIdx);
		bv32TreeTraversal<ClothTreeEdgeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
	}
}

extern "C" __global__
__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 16) //At least 16 blocks per multiprocessor helps with performance
void cloth_differentClothCollisionVTLaunch(
	const PxU32									maxContacts,
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,
	const PxU8*									updateContactPairs,
	PxgDeformablePairInfo*						outContactInfo,		//output
	PxU32*										totalContactCount	//output
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	femDifferentClothCollisionCoreVT<MIDPHASE_WARPS_PER_BLOCK>(
		maxContacts,
		tolerenceLength,
		cmInputs,
		transformCache,
		contactDistance,
		gpuShapes,
		femClothes,
		outContactInfo,
		totalContactCount,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

extern "C" __global__
__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 16) //At least 16 blocks per multiprocessor helps with performance
void cloth_differentClothCollisionEELaunch(
	const PxU32									maxContacts,
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,
	const PxU8*									updateContactPairs,
	PxgDeformablePairInfo*						outContactInfo,		//output
	PxU32*										totalContactCount	//output
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	femDifferentClothCollisionCoreEE<MIDPHASE_WARPS_PER_BLOCK>(
		maxContacts,
		tolerenceLength,
		cmInputs,
		transformCache,
		contactDistance,
		gpuShapes,
		femClothes,
		outContactInfo,
		totalContactCount,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}