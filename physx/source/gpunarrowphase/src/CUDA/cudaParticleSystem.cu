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

#include "foundation/PxBasicTemplates.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMath.h"
#include "foundation/PxMat33.h"
#include "foundation/PxQuat.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxHeightFieldSample.h"
#include "geometry/PxMeshScale.h"

#include "PxNodeIndex.h"

#include "AlignedTransform.h"
#include "GuBV32.h"
#include "schlockShared.h"

#include <vector_types.h>

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemCore.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgSolverBody.h"
#include "PxgSolverCoreDesc.h"

#include "PxsContactManagerState.h"
#include "PxsMaterialCore.h"
#include "PxsTransformCache.h"

#include "PxgCommonDefines.h"
#include "copy.cuh"
#include "cuda.h"
#include "dataReadWriteHelper.cuh"
#include "epa.cuh"
#include "gridCal.cuh"
#include "heightfieldUtil.cuh"
#include "utils.cuh"
#include "reduction.cuh"
#include "deformableElementFilter.cuh"
#include "sphereCollision.cuh"
#include "triangleMesh.cuh"
#include "particleCollision.cuh"


using namespace schlock;
using namespace physx;

extern "C" __host__ void initNarrowphaseKernels10() {}

PX_FORCE_INLINE __device__ PxBounds3 combine(const PxBounds3& bound0, const PxBounds3& bound1)
{
	PxBounds3 overlapBound;
	overlapBound.minimum.x = PxMax(bound0.minimum.x, bound1.minimum.x);
	overlapBound.minimum.y = PxMax(bound0.minimum.y, bound1.minimum.y);
	overlapBound.minimum.z = PxMax(bound0.minimum.z, bound1.minimum.z);

	overlapBound.maximum.x = PxMin(bound0.maximum.x, bound1.maximum.x);
	overlapBound.maximum.y = PxMin(bound0.maximum.y, bound1.maximum.y);
	overlapBound.maximum.z = PxMin(bound0.maximum.z, bound1.maximum.z);
	return overlapBound;
}

PX_FORCE_INLINE __device__ PxU32 calcGridHashInBounds(const PxBounds3& bounds, PxReal cellWidth, PxU32 cellOffset, uint3 wrappedGridSize)
{
	int3 gridPosMin, gridPosMax;
	calcGridRange(gridPosMin, gridPosMax, bounds, cellWidth);
	const uint3 rangeSize = calcWrappedGridRangeSize(gridPosMin, gridPosMax, wrappedGridSize);
	const uint3 offset = calcGridOffsetInRange(rangeSize, cellOffset);
	const int3 gridPos = make_int3(gridPosMin.x + offset.x, gridPosMin.y + offset.y, gridPosMin.z + offset.z);
	return calcGridHash(gridPos, wrappedGridSize);
}

extern "C" __global__ void __launch_bounds__(PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE, 1) ps_primitivesBoundFirstPassLaunch(
	const PxU32									numTests,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxgShape* PX_RESTRICT					shapes,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxBounds3* PX_RESTRICT				bounds,
	const PxReal*								contactDistance,
	const PxgParticleSystem*					particleSystems,
	PxU32*										blockOffsets,		// output
	PxU32*										offsets				// output
)
{

	//numWarpsPerBlock can't be larger than 32
	const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE / WARP_SIZE;

	__shared__ PxU32 sWarpAccumulator[numWarpsPerBlock];

	__shared__ PxU32 sBlockAccumulator;

	const PxU32 nbBlocksRequired = (numTests + PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE - 1) / PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE;

	//gridDim should be 64
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE - 1) / PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	PxU32 warpIndex = threadIdx.x / (WARP_SIZE);
	const PxU32 idx = threadIdx.x;

	if (threadIdx.x == 0)
	{
		sBlockAccumulator = 0;
	}

	__syncthreads();

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		PxU32 cellNumCount = 0;
		if (workIndex < numTests)
		{			
			PxgShape particleShape, rigidShape;
			PxU32 particleCacheRef, rigidCacheRef;
			LoadShapePair<PxGeometryType::ePARTICLESYSTEM>(cmInputs, workIndex, shapes,
				particleShape, particleCacheRef, rigidShape, rigidCacheRef);

			const PxU32 particleInd = particleShape.particleOrSoftbodyId;

			const PxgParticleSystem& particleSystem = particleSystems[particleInd];
			const PxReal cellWidth = particleSystem.mCommonData.mGridCellWidth;
			const uint3 wrappedGridSize = make_uint3(particleSystem.mCommonData.mGridSizeX, 
													 particleSystem.mCommonData.mGridSizeY,
													 particleSystem.mCommonData.mGridSizeZ);

			const PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[rigidCacheRef];

			PxBounds3 bound0 = bounds[rigidCacheRef];
			PxBounds3 bound1 = bounds[particleCacheRef];

			PxBounds3 overlapBound = combine(bound0, bound1);
			overlapBound.fattenFast(cDistance);

			int3 gridPosMin, gridPosMax;
			calcGridRange(gridPosMin, gridPosMax, overlapBound, cellWidth);

			uint3 rangeSize = calcWrappedGridRangeSize(gridPosMin, gridPosMax, wrappedGridSize);
			cellNumCount = rangeSize.x * rangeSize.y * rangeSize.z;
		}

		PxU32 offset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, cellNumCount) - cellNumCount;

		if (threadIndexInWarp == (WARP_SIZE - 1))
			sWarpAccumulator[warpIndex] = offset + cellNumCount;

		const PxU32 prevBlockAccumulator = sBlockAccumulator;

		__syncthreads();


		if (warpIndex == 0)
		{
			PxU32 tOffset = threadIndexInWarp < numWarpsPerBlock ? sWarpAccumulator[threadIndexInWarp] : 0;
			
			const PxU32 output = warpScan<AddOpPxU32, PxU32>(FULL_MASK, tOffset) - tOffset;

			if (threadIndexInWarp == (WARP_SIZE - 1))
				sBlockAccumulator += (output + tOffset);

			if (threadIndexInWarp < numWarpsPerBlock)
				sWarpAccumulator[threadIndexInWarp] = output;
		}

		__syncthreads();

		if (workIndex < numTests)
		{
			//Now output both offsets...
			offsets[workIndex] = offset + prevBlockAccumulator + sWarpAccumulator[warpIndex];
		}

	}

	if (threadIdx.x == 0)
	{
		blockOffsets[blockIdx.x] = sBlockAccumulator;
	}
}

//32 blocks. Each block compute the exclusive ransum for the blockOffset
extern "C" __global__ void ps_primitivesBoundSecondPassLaunch(
	const PxU32	numTests,
	const PxU32* blockOffsets,										//input
	PxU32*	offsets,												//output
	PxU32*	totalNumPairs											//output
)
{

	__shared__ PxU32 sBlockAccum[PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE];
	__shared__ PxU32 sTotalPairs;

	const PxU32 idx = threadIdx.x;

	PxU32 val = 0;
	if (idx < PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE)
		val = blockOffsets[idx];

	PxU32 res = warpScan<AddOpPxU32, PxU32>(FULL_MASK, val) - val;
	
	if (idx < PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE)
		sBlockAccum[idx] = res;

	if (idx == (PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE - 1))
		sTotalPairs = res + val;

	const PxU32 totalBlockRequired = (numTests + (PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE - 1)) / PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE - 1)) / PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE;

	__syncthreads();

	PxU32 blockAccum = sBlockAccum[blockIdx.x];

	for (PxU32 i = 0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if (workIndex < numTests)
		{
			offsets[workIndex] = offsets[workIndex] + blockAccum;
		}
	}

	if (blockIdx.x == 0 && idx == 0)
	{
		*totalNumPairs = sTotalPairs;
		//printf("total pairs %i\n", sTotalPairs);
	}

}

__device__ bool particlePrimitiveCollision(
	const PxVec3& particlePos, 
	const PxVec3& cVolumePos, 
	const PxReal cVolumeRadius, 
	const PxTransform& shapeTransform, 
	const PxGeometryType::Enum shapeType,
	const PxVec3& shapeScale,
	const PxgShape& shape,
	const bool enableCCD,
	PxVec3& normal,
	PxReal& dist)
{
	bool intersect = false;
	switch (shapeType)
	{
	case PxGeometryType::ePLANE:
	{
		PxPlane plane = PxPlane(1, 0, 0, 0).transform(shapeTransform);
		dist = plane.distance(cVolumePos);
		if (dist <= cVolumeRadius)
		{
			//intersection test
			if (enableCCD)
			{
				//in case of CCD we replace the dist from the contact volume with
				//the corresponding from the current particle position 
				dist = plane.distance(particlePos);
			}
			normal = plane.n;
			intersect = true;
		}
		break;
	}
	case PxGeometryType::eSPHERE:
	{
		const PxVec3& spherePos = shapeTransform.p;
		const PxReal sphereRadius = shapeScale.x;
		//contact volume test
		intersect = contactPointSphere(normal, dist, cVolumePos, spherePos, sphereRadius, cVolumeRadius);
		if (intersect)
		{
			//intersection test
			if (enableCCD)
			{
				//in case of CCD we replace the dist and normal from the contact volume with
				//the corresponding from the current particle position 
				contactPointSphere(normal, dist, particlePos, spherePos, sphereRadius, PX_MAX_F32);
			}
		}
		break;
	}
	case PxGeometryType::eCAPSULE:
	{
		const PxReal capsuleRadius = shapeScale.y;
		const PxReal capsuleHalfHeight = shapeScale.x;
		const PxVec3 capsuleDir = shapeTransform.q.getBasisVector0();
		const PxVec3& capsulePos = shapeTransform.p;
		intersect = contactPointCapsule(normal, dist, cVolumePos, capsulePos, capsuleDir, capsuleRadius, capsuleHalfHeight, cVolumeRadius);
		if (intersect)
		{
			if (enableCCD)
			{
				//in case of CCD we replace the dist and normal from the contact volume with
				//the corresponding from the current particle position 
				contactPointCapsule(normal, dist, particlePos, capsulePos, capsuleDir, capsuleRadius, capsuleHalfHeight, PX_MAX_F32);
			}
		}
		break;
	}
	case PxGeometryType::eBOX:
	{
		const PxVec3 boxHalfExtents = shapeScale;
		const PxTransform& boxToWorld = shapeTransform;

		intersect = contactPointBox(normal, dist, cVolumePos, boxToWorld, boxHalfExtents, cVolumeRadius);
		if (intersect)
		{
			if (enableCCD)
			{
				contactPointBox(normal, dist, particlePos, boxToWorld, boxHalfExtents, PX_MAX_F32);
			}
		}
		break;
	}
	case PxGeometryType::eCONVEXCORE:
	{
		Gu::ConvexShape convex;
		convex.coreType = Gu::ConvexCore::Type::Enum(shape.hullOrMeshPtr);
		memcpy(convex.coreData, &shape.scale.scale.x, PxConvexCoreGeometry::MAX_CORE_SIZE);
		convex.margin = shape.scale.rotation.w;
		convex.pose = shapeTransform;

		intersect = particleConvexCore(normal, dist, cVolumePos, convex, cVolumeRadius);
		if (intersect)
		{
			if (enableCCD)
			{
				particleConvexCore(normal, dist, particlePos, convex, PX_MAX_F32);
			}
		}
		break;
	}
	default:
		break;
	};

	return intersect;
}

__device__ void psPrimitivesCollision(
	const bool								isTGS,
	const PxU32								gridHash,
	PxgShape&								shape0,
	PxgShape&								shape1,
	const PxU32								transformCacheRef0,
	const PxU32								transformCacheRef1,
	const PxsCachedTransform* PX_RESTRICT	transformCache,
	PxgParticleSystem&						particleSystem,
	const PxU32								particleSystemId,
	const PxReal							cDistance,
	const PxReal							restDistance,
	const PxsMaterialData* PX_RESTRICT		materials,
	const PxU32								workIndex,
	const PxU32								totalComparision,
	const PxNodeIndex*						shapeToRigidRemapTable,
	PxgParticleContactWriter&				writer
)										
{
	PxU32 startIndex = 0xFFFFFFFF;
	PxU32 endIndex = 0;

	float4* currentPositions = NULL;
	float4* predictedPositions = NULL;
	bool enableCCD = false;
	PxGeometryType::Enum type0;
	PxVec3 scale0;
	PxU32 range = 0;
	PxTransform transform0;
	PxNodeIndex rigidId;

	const PxU32* PX_RESTRICT gridParticleIndex = NULL;

	if (workIndex < totalComparision)
	{
		transform0 = transformCache[transformCacheRef0].transform;
		type0 = PxGeometryType::Enum(shape0.type);
		scale0 = shape0.scale.scale;

		const PxU32* cellStart = particleSystem.mCellStart;
		const PxU32* cellEnd = particleSystem.mCellEnd;
		gridParticleIndex = particleSystem.mSortedToUnsortedMapping;

		currentPositions = reinterpret_cast<float4*>(particleSystem.mSortedOriginPos_InvMass);
		predictedPositions = reinterpret_cast<float4*>(particleSystem.mSortedPositions_InvMass);
		enableCCD = (particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;

		// get the start of bucket for this cell
		startIndex = cellStart[gridHash]; // startIndex might be EMPTY_CELL

		if (startIndex != EMPTY_CELL) 
		{
			// get the end of bucket for this cell
			endIndex = cellEnd[gridHash];

			range = endIndex - startIndex;
		}

		rigidId = shapeToRigidRemapTable[transformCacheRef0];
	}

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	range = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, range);

	range = __shfl_sync(FULL_MASK, range, WARP_SIZE - 1);

	for (PxU32 ind = 0; ind < range; ind++)
	{
		bool intersect = false;
		PxVec3 normal;
		PxReal distance;
		PxU64 compressedParticleIndex = 0;

		if(startIndex != EMPTY_CELL)
		{
			PxU32 particleIndex = startIndex + ind;
			if (particleIndex < endIndex)
			{
				PxVec3 currentPos = PxLoad3(currentPositions[particleIndex]);

				PxVec3 cVolumePos;
				PxReal cVolumeRadius;
				if (enableCCD)
				{
					PxVec3 predictedPos = PxLoad3(predictedPositions[particleIndex]);
					cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos,
						currentPos, predictedPos, cDistance, false, isTGS);
				}
				else
				{
					cVolumeRadius = cDistance;
					cVolumePos = currentPos;
				}

				compressedParticleIndex = PxEncodeParticleIndex(particleSystemId, particleIndex);
				const PxU64 particleMask = PxEncodeParticleIndex(particleSystemId, gridParticleIndex[particleIndex]);

				if (!find(particleSystem, rigidId.getInd(), particleMask))
				{
					intersect = particlePrimitiveCollision(currentPos, cVolumePos, cVolumeRadius, transform0, type0, scale0, shape0, enableCCD, normal, distance);
				}
			}
		}

		const PxU32 threadMask = (1 << threadIndexInWarp) - 1;
		PxU32 mask = __ballot_sync(FULL_MASK, intersect);
		PxU32 nbToLock = __popc(mask);
		PxU32 contactIndex = __popc(mask & threadMask);

		int32_t contactStartIndex = 0;
		if (threadIndexInWarp == 0 && nbToLock)
		{
			contactStartIndex = atomicAdd(writer.numTotalContacts, int32_t(nbToLock));
		}

		contactStartIndex = __shfl_sync(FULL_MASK, contactStartIndex, 0);

		//write to particle primitives contact buffer
		if (intersect)
		{
			const PxU32 index = contactStartIndex + contactIndex;
			writer.writeContact(index, PxVec4(normal, distance - restDistance), compressedParticleIndex, rigidId);
		}
	}
}


__device__ void psDiffusePrimitivesCollision(
	const bool								isTGS,
	const PxU32								gridHash,
	PxgShape&								shape0,
	PxgShape&								shape1,
	const PxU32								transformCacheRef0,
	const PxU32								transformCacheRef1,
	const PxsCachedTransform* PX_RESTRICT	transformCache,
	PxgParticleSystem&						particleSystem,
	const PxU32								particleSystemId,
	const PxReal							cDistance,
	const PxReal							restDistance,
	const PxsMaterialData* PX_RESTRICT		materials,
	const PxU32								workIndex,
	const PxU32								totalComparision,
	const PxNodeIndex*						shapeToRigidRemapTable
)
{
	PxU32 startIndex = EMPTY_CELL;
	PxU32 endIndex = 0;

	float4* currentPositions = NULL;
	float4* predictedPositions = NULL;
	bool enableCCD = false;
	PxGeometryType::Enum type0;
	PxVec3 scale0;
	PxU32 range = 0;
	PxTransform transform0;

	PxNodeIndex rigidId;

	if (workIndex < totalComparision)
	{

		//printf("totalComparision %i numParticles %i\n", totalComparision, numParticles);

		transform0 = transformCache[transformCacheRef0].transform;
		type0 = PxGeometryType::Enum(shape0.type);
		scale0 = shape0.scale.scale;

		PxU32* cellStart = particleSystem.mDiffuseCellStart;
		PxU32* cellEnd = particleSystem.mDiffuseCellEnd;

		currentPositions = reinterpret_cast<float4*>(particleSystem.mDiffuseSortedOriginPos_LifeTime); 
		predictedPositions = reinterpret_cast<float4*>(particleSystem.mDiffuseSortedPos_LifeTime);

		PxgParticleSystemData& data = particleSystem.mData;
		enableCCD = (data.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;

		// If there is no diffuse particles or is not using PBD, skip this function
		if (particleSystem.mCommonData.mMaxDiffuseParticles == 0)
			return;

		// get the start of bucket for this cell
		startIndex = cellStart[gridHash]; // startIndex might be 0xFFFFFFFF

		if (startIndex != EMPTY_CELL)
		{
			// get the end of bucket for this cell
			endIndex = cellEnd[gridHash];

			assert(endIndex >= startIndex);
			range = endIndex - startIndex;
		}

		rigidId = shapeToRigidRemapTable[transformCacheRef0];
	}


	PxgParticleContactInfo* PX_RESTRICT infos = NULL;
	PxReal* PX_RESTRICT impulses = NULL;
	PxNodeIndex* PX_RESTRICT nodeIndices = NULL;
	PxU32* PX_RESTRICT counts = NULL;
	PxU32 numParticles = 0;

	if (range != 0)
	{
		infos = particleSystem.mDiffuseOneWayContactInfos;
		impulses = particleSystem.mDiffuseOneWayForces;
		nodeIndices = particleSystem.mDiffuseOneWayNodeIndex;
		counts = particleSystem.mDiffuseOneWayContactCount;
		numParticles = *particleSystem.mNumDiffuseParticles;
	}
	
	for (PxU32 ind = 0; ind < range; ind++)
	{
		bool intersect = false;
		PxVec3 normal;
		PxReal distance;
		PxU32 particleIndex = 0xFFFFFFFF;

		if (startIndex != EMPTY_CELL)
		{
			particleIndex = startIndex + ind;

			
			if (particleIndex < endIndex)
			{
				PxVec3 currentPos = PxLoad3(currentPositions[particleIndex]);

				PxVec3 cVolumePos;
				PxReal cVolumeRadius;
				if (enableCCD)
				{
					PxVec3 predictedPos = PxLoad3(predictedPositions[particleIndex]);
					cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos,
						currentPos, predictedPos, cDistance, true, isTGS);
				}
				else
				{
					cVolumeRadius = cDistance;
					cVolumePos = currentPos;
				}

				intersect = particlePrimitiveCollision(currentPos, cVolumePos, cVolumeRadius, transform0, type0, scale0, shape0, enableCCD, normal, distance);

				if (intersect)
				{
					PxU32 index = atomicAdd(&counts[particleIndex], 1);
				
					if (index < PxgParticleContactInfo::MaxStaticContactsPerParticle)
					{
						const PxU32 outputIndex = particleIndex + index * numParticles;
						float4 normalPen = make_float4(normal.x, normal.y, normal.z, distance - restDistance);
						infos[outputIndex].mNormal_PenW = normalPen;
						impulses[outputIndex] = 0.f;
						nodeIndices[outputIndex] = rigidId;
					}
				}
			}
		}
	}
}

struct ParticleConvexCollideScratch
{
	PxVec3 vA[1]; //sphere center
	PxVec3 vB[CONVEX_MAX_VERTICES_POLYGONS];
	GjkCachedData cachedData;
	GjkOutput gjkOutput;

	PxQuat rot1;
	PxVec3 scale1;

	/*PxTransform aToB;*/
	PxTransform tB;

	PxReal contactDistance;
	PxReal inSphereRadius0;
	PxReal inSphereRadius1;

	PxU8 nbVertices0;
	PxU8 nbVertices1;
	
	__device__ void Clear()
	{
		if (threadIdx.x == 0)
			cachedData.size = 0;
		__syncwarp();
	}
};

__device__ static
bool particleConvexCollision(
	const PxVec3& particlePos,
	const PxVec3& cVolumePos,
	const PxReal cVolumeRadius,
	const bool enableCCD,
	const float4* pVertices1,
	squawk::EpaScratch& ss_epa_scratch,
	ParticleConvexCollideScratch& ss_scratch)
{
	const PxReal convergenceRatio = 1 - 0.000225f;

	assert(ss_scratch.nbVertices0 <= 1);
	assert(ss_scratch.nbVertices1 <= CONVEX_MAX_VERTICES_POLYGONS);

	//rotate all the convex verts into world space
	prepareVertices(ss_scratch.tB, ss_scratch.scale1, ss_scratch.rot1, ss_scratch.nbVertices1, pVertices1, ss_scratch.vB);

	__syncwarp(); // this is to avoid having volatile vertex array

	//contact volume test, cVolumePos is world space
	if (threadIdx.x == 0)
	{
		ss_scratch.vA[0] = cVolumePos;
		ss_scratch.cachedData.size = 0;
	}
	__syncwarp();

	PxVec3 initialDir = PxVec3(1, 0, 0);
		
	PxVec3 aToB_p = cVolumePos - ss_scratch.tB.p;
	if (aToB_p.magnitudeSquared() > 0)
		initialDir = aToB_p; // GJK's initial start dir is usually from the warm cache => lazy normalize inside GJK

	PxReal minSep = cVolumeRadius;
	GjkResult::Enum gjkResult = squawk::gjk(
		ss_scratch.vA, ss_scratch.nbVertices0,
		ss_scratch.vB, ss_scratch.nbVertices1,
		initialDir,
		minSep,
		convergenceRatio,
		ss_scratch.gjkOutput, ss_scratch.cachedData);

	__syncwarp();

	if (gjkResult == GjkResult::eDISTANT)
	{
		return false;
	}

	if (enableCCD)
	{
		//in case of CCD we replace gjkResult, ss_scratch.gjkOutput for the contact volume with
		//the corresponding from the current particle position 

		//particlePos is world space
		if (threadIdx.x == 0)
		{
			ss_scratch.vA[0] = particlePos;
			ss_scratch.cachedData.size = 0;
		}
		__syncwarp();

		PxVec3 initialDir = PxVec3(1, 0, 0);
		PxVec3 aToB_p = particlePos - ss_scratch.tB.p;

		if (aToB_p.magnitudeSquared() > 0)
			initialDir = aToB_p; // GJK's initial start dir is usually from the warm cache => lazy normalize inside GJK

		//make sure to cover the larger volume that includes both the current and the whole cVolume
		minSep = ss_scratch.contactDistance + cVolumeRadius*2.0f;
		gjkResult = squawk::gjk(
			ss_scratch.vA, ss_scratch.nbVertices0,
			ss_scratch.vB, ss_scratch.nbVertices1,
			initialDir,
			minSep,
			convergenceRatio,
			ss_scratch.gjkOutput, ss_scratch.cachedData);

		__syncwarp();

		if (gjkResult == GjkResult::eDISTANT)
		{
			return false;
		}
	}

	bool anomaly = gjkResult == GjkResult::eCLOSE && ss_scratch.gjkOutput.closestPointDir.dot(ss_scratch.gjkOutput.direction) < 0.999f;
	bool separated = gjkResult == GjkResult::eCLOSE;

	if (!separated || anomaly)
	{
		GjkResult::Enum epaResult = squawk::epa(ss_epa_scratch,
			ss_scratch.vA, ss_scratch.nbVertices0,
			ss_scratch.vB, ss_scratch.nbVertices1,
			&(ss_scratch.cachedData),
			convergenceRatio,
			0.5f * (ss_scratch.inSphereRadius0 + ss_scratch.inSphereRadius1),
			ss_scratch.gjkOutput);

		separated = epaResult == GjkResult::eCLOSE;

		__syncwarp();

		if (ss_scratch.gjkOutput.degenerate)
		{
			//we need to re-run gjk epa with other configurations
			ss_scratch.cachedData.size = 0;

			//we need to re-run gjk epa with other configurations
			initialDir = PxVec3(0.f, 1.f, 0.f);

			GjkResult::Enum gjkResult = squawk::gjk(
				ss_scratch.vA, ss_scratch.nbVertices0,
				ss_scratch.vB, ss_scratch.nbVertices1,
				initialDir,
				minSep,
				convergenceRatio,
				ss_scratch.gjkOutput, ss_scratch.cachedData);

			__syncwarp();

			GjkResult::Enum epaResult = squawk::epa(ss_epa_scratch,
				ss_scratch.vA, ss_scratch.nbVertices0,
				ss_scratch.vB, ss_scratch.nbVertices1,
				&(ss_scratch.cachedData),
				convergenceRatio,
				0.5f * (ss_scratch.inSphereRadius0 + ss_scratch.inSphereRadius1),
				ss_scratch.gjkOutput);

			separated = epaResult == GjkResult::eCLOSE;

			__syncwarp();

		}
	}
	return true;
}


__device__ void psConvexCollision(
	const bool								isTGS,
	const PxU32								gridHash,
	PxgShape&								shape0,
	PxgShape&								shape1,
	const PxU32								transformCacheRef0,
	const PxU32								transformCacheRef1,
	const PxsCachedTransform* PX_RESTRICT	transformCache,
	const PxU32								particleSystemId,
	const PxReal							cDistance,
	const PxReal							restDistance,
	const PxsMaterialData* PX_RESTRICT		materials,
	const PxU32								workIndex,
	const PxNodeIndex*						shapeToRigidRemapTable,
	PxgParticlePrimitiveContact*			contacts,								//output
	PxI32*									numTotalContacts,						//output
	PxU64*									contactSortedByParticle,				//output
	PxU32*									tempContactByParticle,					//output
	PxU32*									contactRemapSortedByParticle,			//output
	PxU64*									contactByRigid,							//output
	PxU32*									tempContactByRigid,						//output
	PxU32*									contactRemapSortedByRigid,				//output
	ParticleConvexCollideScratch&			ss_scratch,
	squawk::EpaScratch&						ss_epa_scratch,
	PxgParticleSystem&						ss_particleSystem,
	const PxU32								maxContacts)
{
	//convex hull
	PxsCachedTransform transfCache0;
	PxsCachedTransform_ReadWarp(transfCache0, transformCache + transformCacheRef0);

	const PxTransform& transf0 = transfCache0.transform;
	assert(transf0.isSane());

	size_t hullPtr = shape0.hullOrMeshPtr;
	const PxU8* convexPtr = (PxU8*)hullPtr + sizeof(float4);
	const float4 extents = *((float4*)(convexPtr + sizeof(uint4)));
	const float4* pVertices1 = reinterpret_cast<const float4*>(convexPtr + sizeof(uint4) + sizeof(float4));

	if (threadIdx.x == 0)
	{
		ss_scratch.nbVertices0 = 1;
		
		ss_scratch.inSphereRadius0 = 0.f;


		ss_scratch.scale1 = shape0.scale.scale;
		ss_scratch.rot1 = shape0.scale.rotation;

		const uint4 tmp = *((uint4*)convexPtr);
		const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;

		ss_scratch.nbVertices1 = getNbVerts(polyData0_NbEdgesNbHullVerticesNbPolygons);

		PxReal minScale1 = 1.f;
		if (shape0.type == PxGeometryType::eBOX)
		{
			const PxVec3 scale1 = shape0.scale.scale;
			minScale1 = PxMin(scale1.x, scale1.y);
			minScale1 = PxMin(scale1.z, minScale1);
		}

		ss_scratch.inSphereRadius1 = extents.w * minScale1;

		ss_scratch.contactDistance = cDistance;

		ss_scratch.tB = transf0;
	}

	__syncwarp();

	const PxU32* cellStart = ss_particleSystem.mCellStart;
	const PxU32* cellEnd = ss_particleSystem.mCellEnd;
	const float4* currentPositions = reinterpret_cast<float4*>(ss_particleSystem.mSortedOriginPos_InvMass);
	const float4* predictedPositions = reinterpret_cast<float4*>(ss_particleSystem.mSortedPositions_InvMass);
	const bool enableCCD = (ss_particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;
	const PxU32* PX_RESTRICT gridParticleIndex = ss_particleSystem.mSortedToUnsortedMapping;
	
	// get the start of bucket for this cell
	const PxU32 startIndex = cellStart[gridHash]; // startIndex might be EMPTY_CELL

	if (startIndex != EMPTY_CELL)
	{
		// get the end of bucket for this cell
		const PxU32 endIndex = cellEnd[gridHash];

		const PxU32 range = endIndex - startIndex;
		for (PxU32 ind = 0; ind < range; ind++)
		{
			const PxU32 particleIndex = startIndex + ind;
			const PxNodeIndex rigidId = shapeToRigidRemapTable[transformCacheRef0];
			
			const PxU64 particleMask = PxEncodeParticleIndex(particleSystemId, gridParticleIndex[particleIndex]);

			if(find(ss_particleSystem,rigidId.getInd(), particleMask))
				continue;


			PxVec3 currentPos = PxLoad3(currentPositions[particleIndex]);

			PxVec3 cVolumePos;
			PxReal cVolumeRadius;
			if (enableCCD)
			{
				PxVec3 predictedPos = PxLoad3(predictedPositions[particleIndex]);
				cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos, currentPos, predictedPos, ss_scratch.contactDistance, true, isTGS);
			}
			else
			{
				cVolumePos = currentPos;
				cVolumeRadius = ss_scratch.contactDistance;
			}

			const bool intersect = particleConvexCollision(currentPos, cVolumePos, cVolumeRadius, enableCCD, pVertices1, ss_epa_scratch, ss_scratch);

			if (intersect)
			{
				int32_t index = 0;
				if (threadIdx.x == 0)
				{
					index = atomicAdd(numTotalContacts, 1);
				}

				index = __shfl_sync(FULL_MASK, index, 0);

				if (index < maxContacts)
				{

					//write to particle primitives contact buffer
					PxgParticlePrimitiveContact& contact = contacts[index];

					const PxU64 compressedParticleIndex = PxEncodeParticleIndex(particleSystemId, particleIndex);
					if (threadIdx.x == 0)
					{
						const PxU64 value = rigidId.getInd();
						contactByRigid[index] = value;
						tempContactByRigid[index] = PxU32(value & 0xffffffff);
						contactRemapSortedByRigid[index] = index;
						contactSortedByParticle[index] = compressedParticleIndex;
						tempContactByParticle[index] = PxGetParticleIndex(compressedParticleIndex);
						contactRemapSortedByParticle[index] = index;
					}

					PxReal witnessA = 0.f;
					PxReal witnessB = 0.f;
					PxReal normal = 0.f;

					if (threadIdx.x < 3)
					{
						const float* closestPointA = &ss_scratch.gjkOutput.closestPointA.x;
						const float* closestPointB = &ss_scratch.gjkOutput.closestPointB.x;
						const float* direction = &ss_scratch.gjkOutput.direction.x;
						witnessA = closestPointA[threadIdx.x];
						witnessB = closestPointB[threadIdx.x];
						normal = direction[threadIdx.x];
					}
					PxReal pen = (witnessB - witnessA) * normal;
					pen = __shfl_sync(FULL_MASK, pen, 0) + __shfl_sync(FULL_MASK, pen, 1) + __shfl_sync(FULL_MASK, pen, 2) - restDistance;

					if (threadIdx.x < 4)
					{

						float* normalPen = &(contact.normal_pen.x);
						PxReal outCP = threadIdx.x < 3 ? (-normal) : pen;
						normalPen[threadIdx.x] = outCP;

						if (threadIdx.x == 0)
						{
							contact.particleId = compressedParticleIndex;
							contact.rigidId = rigidId.getInd();
						}
					}
				}
			}
		}
	}
}


__device__ void psConvexDiffuseCollision(
	const bool								isTGS,
	const PxU32								gridHash,
	PxgShape&								shape0,
	PxgShape&								shape1,
	const PxU32								transformCacheRef0,
	const PxU32								transformCacheRef1,
	const PxsCachedTransform* PX_RESTRICT	transformCache,
	const PxU32								particleSystemId,
	const PxReal							cDistance,
	const PxReal							restDistance,
	const PxsMaterialData* PX_RESTRICT		materials,
	const PxU32								workIndex,
	const PxNodeIndex*						shapeToRigidRemapTable,
	ParticleConvexCollideScratch&			ss_scratch,
	squawk::EpaScratch&						ss_epa_scratch,
	PxgParticleSystem&						ss_particleSystem)
{
	//convex hull
	PxsCachedTransform transfCache0;
	PxsCachedTransform_ReadWarp(transfCache0, transformCache + transformCacheRef0);

	const PxTransform& transf0 = transfCache0.transform;
	assert(transf0.isSane());

	size_t hullPtr = shape0.hullOrMeshPtr;
	const PxU8* convexPtr = (PxU8*)hullPtr + sizeof(float4);
	const float4 extents = *((float4*)(convexPtr + sizeof(uint4)));
	const float4* pVertices1 = reinterpret_cast<const float4*>(convexPtr + sizeof(uint4) + sizeof(float4));

	if (threadIdx.x == 0)
	{
		ss_scratch.nbVertices0 = 1;
		ss_scratch.inSphereRadius0 = 0.f;

		ss_scratch.scale1 = shape0.scale.scale;
		ss_scratch.rot1 = shape0.scale.rotation;

		const uint4 tmp = *((uint4*)convexPtr);
		const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;

		ss_scratch.nbVertices1 = getNbVerts(polyData0_NbEdgesNbHullVerticesNbPolygons);

		PxReal minScale1 = 1.f;
		if (shape0.type == PxGeometryType::eBOX)
		{
			const PxVec3 scale1 = shape0.scale.scale;
			minScale1 = PxMin(scale1.x, scale1.y);
			minScale1 = PxMin(scale1.z, minScale1);
		}

		ss_scratch.inSphereRadius1 = extents.w * minScale1;
		ss_scratch.contactDistance = cDistance;
		ss_scratch.tB = transf0;
	}

	__syncwarp();

	PxU32* cellStart = ss_particleSystem.mDiffuseCellStart;
	PxU32* cellEnd = ss_particleSystem.mDiffuseCellEnd;
	const float4* currentPositions = reinterpret_cast<float4*>(ss_particleSystem.mDiffuseSortedOriginPos_LifeTime);
	const float4* predictedPositions = reinterpret_cast<float4*>(ss_particleSystem.mDiffuseSortedPos_LifeTime);
	const bool enableCCD = (ss_particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;

	// get the start of bucket for this cell
	const PxU32 startIndex = cellStart[gridHash]; // startIndex might be EMPTY_CELL

	if (startIndex != EMPTY_CELL)
	{
		PxgParticleContactInfo* infos = ss_particleSystem.mDiffuseOneWayContactInfos;
		float*  impulses = ss_particleSystem.mDiffuseOneWayForces;
		PxNodeIndex*  nodeIndices = ss_particleSystem.mDiffuseOneWayNodeIndex;
		PxU32*  counts = ss_particleSystem.mDiffuseOneWayContactCount;
		const PxU32 numParticles = *ss_particleSystem.mNumDiffuseParticles;

		// get the end of bucket for this cell
		const PxU32 endIndex = cellEnd[gridHash];

		const PxU32 range = endIndex - startIndex;
		for (PxU32 ind = 0; ind < range; ind++)
		{
			const PxU32 particleIndex = startIndex + ind;

			const PxNodeIndex rigidId = shapeToRigidRemapTable[transformCacheRef0];

			PxVec3 currentPos = PxLoad3(currentPositions[particleIndex]);

			PxVec3 cVolumePos;
			PxReal cVolumeRadius;
			if (enableCCD)
			{
				PxVec3 predictedPos = PxLoad3(predictedPositions[particleIndex]);
				cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos, currentPos, predictedPos, ss_scratch.contactDistance, true, isTGS);
			}
			else
			{
				cVolumePos = currentPos;
				cVolumeRadius = ss_scratch.contactDistance;
			}

			const bool intersect = particleConvexCollision(currentPos, cVolumePos, cVolumeRadius, enableCCD, pVertices1, ss_epa_scratch, ss_scratch);

			if (intersect)
			{
				int32_t index = 0;
				if (threadIdx.x == 0)
				{
					index = atomicAdd(&counts[particleIndex], 1);
				}

				index = __shfl_sync(FULL_MASK, index, 0);

				if (index < PxgParticleContactInfo::MaxStaticContactsPerParticle)
				{
					const PxU32 outputIndex = particleIndex + index * numParticles;


					PxReal witnessA = 0.f;
					PxReal witnessB = 0.f;
					PxReal normal = 0.f;

					if (threadIdx.x < 3)
					{
						const float* closestPointA = &ss_scratch.gjkOutput.closestPointA.x;
						const float* closestPointB = &ss_scratch.gjkOutput.closestPointB.x;
						const float* direction = &ss_scratch.gjkOutput.direction.x;
						witnessA = closestPointA[threadIdx.x];
						witnessB = closestPointB[threadIdx.x];
						normal = direction[threadIdx.x];
					}
					PxReal pen = (witnessB - witnessA) * normal;
					pen = __shfl_sync(FULL_MASK, pen, 0) + __shfl_sync(FULL_MASK, pen, 1) + __shfl_sync(FULL_MASK, pen, 2) - restDistance;

					if (threadIdx.x < 4)
					{

						float* normalPen = &(infos[outputIndex].mNormal_PenW.x);
						PxReal outCP = threadIdx.x < 3 ? (-normal) : pen;
						normalPen[threadIdx.x] = outCP;

						if (threadIdx.x == 0)
						{
							impulses[outputIndex] = 0.f;
							nodeIndices[outputIndex] = rigidId;
						}
					}
				}
			}
		}
	}
}

struct PxgCellData
{
	PxU32 gridHash;
	PxgParticleSystem* particleSystem;
	PxReal cDistance;
	PxReal restDistance;
	PxgShape particleShape;
	PxgShape rigidShape;
	PxU32 particleCacheRef = 0xffffffff;
	PxU32 rigidCacheRef = 0xffffffff;
	PxU32 particleSystemId = 0xffffffff;

	PX_FORCE_INLINE __device__ void update(
		PxU32 workIndex,
		const PxU32									numTests,
		const PxgContactManagerInput* PX_RESTRICT	cmInputs,
		PxgShape* PX_RESTRICT						shapes,
		const PxBounds3* PX_RESTRICT				bounds,
		const PxReal* PX_RESTRICT					contactDistance,
		const PxReal* PX_RESTRICT					restDistances,
		const PxU32* PX_RESTRICT					startIndices, //the run sum for cells
		PxgParticleSystem* PX_RESTRICT				particleSystems)
	{
		const PxU32 pos = binarySearch(startIndices, numTests, workIndex);

		//start index for the thread groups
		const PxU32 startIndex = startIndices[pos];

		LoadShapePair<PxGeometryType::ePARTICLESYSTEM>(cmInputs, pos, shapes,
			particleShape, particleCacheRef, rigidShape, rigidCacheRef);

		particleSystemId = particleShape.particleOrSoftbodyId;
		particleSystem = &particleSystems[particleSystemId];

		cDistance = contactDistance[particleCacheRef] + contactDistance[rigidCacheRef];
		restDistance = restDistances[pos];

		PxBounds3 overlapBound = combine(bounds[particleCacheRef], bounds[rigidCacheRef]);
		overlapBound.fattenFast(cDistance);

		const PxReal cellWidth = particleSystem->mCommonData.mGridCellWidth;
		const PxU32 offset = workIndex - startIndex;
		uint3 wrappedGridSize = make_uint3(particleSystem->mCommonData.mGridSizeX, 
										   particleSystem->mCommonData.mGridSizeY,
										   particleSystem->mCommonData.mGridSizeZ);

		gridHash = calcGridHashInBounds(overlapBound, cellWidth, offset, wrappedGridSize);
	}
};


extern "C" __global__ void ps_primitivesCollisionLaunch(
	const bool									isTGS,
	const PxU32									numTests,
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	PxgShape* PX_RESTRICT						shapes,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxBounds3* PX_RESTRICT				bounds,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistances,
	const PxsMaterialData* PX_RESTRICT			materials,
	const PxU32* PX_RESTRICT					startIndices, //the run sum for cells
	const PxU32*								totalNumPairs,
	PxgParticleSystem* PX_RESTRICT				particleSystems,
	const PxNodeIndex*							shapeToRigidRemapTable,
	PxgParticleContactWriter					writer	
	)
{
	
	const PxU32 totalComparision = *totalNumPairs;

	const PxU32 NumWarps = PxgParticleSystemKernelBlockDim::PS_COLLISION / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	__shared__ PxU32 shWorkIndices[PxgParticleSystemKernelBlockDim::PS_COLLISION];
	__shared__ PxU32 shWarpSum[NumWarps];
	PxU32 workCount = 0;

	for (PxU32 i = 0; i < totalComparision; i += blockDim.x * gridDim.x)
	{
		PxU32 workIndex = i + threadIdx.x + blockIdx.x * blockDim.x;

		PxgCellData data;
		PxU32 range = 0;

		if (workIndex < totalComparision)
		{
			data.update(workIndex, numTests, cmInputs, shapes, bounds, contactDistance, restDistances, startIndices, particleSystems);

			const PxU32* cellStart = data.particleSystem->mCellStart;
			const PxU32* cellEnd = data.particleSystem->mCellEnd;
			PxU32 gridHash = data.gridHash;

			// get the start of bucket for this cell
			PxU32 cellStartIndex = cellStart[gridHash]; // cellStartIndex might be 0xFFFFFFFF
			
			if (cellStartIndex != EMPTY_CELL) 
			{
				// get the end of bucket for this cell
				PxU32 cellEndIndex = cellEnd[gridHash];

				range = cellEndIndex - cellStartIndex;
			}			
		}

		//Now we have "range", which tells us if this workIndex is interesting...

		PxU32 rangeMask = __ballot_sync(FULL_MASK, range);

		PxU32 count = __popc(rangeMask);

		if (threadIndexInWarp == 0)
			shWarpSum[threadIdx.x / 32] = count;

		__syncthreads();

		PxU32 warpCount = 0;
		if (threadIndexInWarp < NumWarps)
			warpCount = shWarpSum[threadIndexInWarp];

		PxU32 blockRunSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, warpCount);

		PxU32 totalBlock = __shfl_sync(FULL_MASK, blockRunSum, 31);

		blockRunSum = blockRunSum + workCount - warpCount;

		blockRunSum = __shfl_sync(FULL_MASK, blockRunSum, threadIdx.x / 32);

		PxU32 overflowIndex = 0xFFFFFFFF;
		if (range)
		{
			PxU32 outIndex = blockRunSum + warpScanExclusive(rangeMask, threadIndexInWarp);

			if (outIndex < PxgParticleSystemKernelBlockDim::PS_COLLISION)
			{
				shWorkIndices[outIndex] = workIndex;
			}
			else
				overflowIndex = outIndex - PxgParticleSystemKernelBlockDim::PS_COLLISION;
		}

		__syncthreads();

		workCount = workCount + totalBlock;

		if (workCount >= PxgParticleSystemKernelBlockDim::PS_COLLISION)
		{
			PxU32 thisWorkIndex = workIndex;

			workIndex = shWorkIndices[threadIdx.x];			
			data.update(workIndex, numTests, cmInputs, shapes, bounds, contactDistance, restDistances, startIndices, particleSystems);
			
			psPrimitivesCollision(isTGS, data.gridHash, data.rigidShape, data.particleShape, data.rigidCacheRef, data.particleCacheRef, transformCache,
				*data.particleSystem, data.particleSystemId, data.cDistance, data.restDistance, materials, workIndex,
				totalComparision, shapeToRigidRemapTable, writer);

			workCount -= PxgParticleSystemKernelBlockDim::PS_COLLISION;

			if (overflowIndex != 0xFFFFFFFF)
			{
				shWorkIndices[overflowIndex] = thisWorkIndex;
			}
		}
	}


	if (workCount)
	{
		PxU32 workIndex = 0xFFFFFFFF;
		PxgCellData data;

		if (threadIdx.x < workCount)
		{
			workIndex = shWorkIndices[threadIdx.x];			
			data.update(workIndex, numTests, cmInputs, shapes, bounds, contactDistance, restDistances, startIndices, particleSystems);
		}

		psPrimitivesCollision(isTGS, data.gridHash, data.rigidShape, data.particleShape, data.rigidCacheRef, data.particleCacheRef, transformCache,
			*data.particleSystem, data.particleSystemId, data.cDistance, data.restDistance, materials, workIndex,
			totalComparision, shapeToRigidRemapTable, writer);
	}
}


extern "C" __global__ void ps_primitivesDiffuseCollisionLaunch(
	const bool									isTGS,
	const PxU32									numTests,
	const PxReal								toleranceLength,
	const PxgContactManagerInput * PX_RESTRICT	cmInputs,
	PxgShape * PX_RESTRICT						shapes,
	const PxsCachedTransform * PX_RESTRICT		transformCache,
	const PxBounds3 * PX_RESTRICT				bounds,
	const PxReal * PX_RESTRICT					contactDistance,
	const PxReal * PX_RESTRICT					restDistances,
	const PxsMaterialData * PX_RESTRICT			materials,
	const PxU32 * PX_RESTRICT					startIndices, //the run sum for cells
	const PxU32 *								totalNumPairs,
	PxgParticleSystem * PX_RESTRICT				particleSystems,
	const PxNodeIndex * PX_RESTRICT				shapeToRigidRemapTable
	)
{

	const PxU32 totalComparision = *totalNumPairs;


	const PxU32 NumWarps = PxgParticleSystemKernelBlockDim::PS_COLLISION / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	__shared__ PxU32 shWorkIndices[PxgParticleSystemKernelBlockDim::PS_COLLISION];
	__shared__ PxU32 shWarpSum[NumWarps];
	
	PxU32 workCount = 0;


	for (PxU32 i = 0; i < totalComparision; i += blockDim.x * gridDim.x)
	{
		PxU32 workIndex = i + threadIdx.x + blockIdx.x * blockDim.x;

		PxU32 range = 0;
		PxgCellData data;

		if (workIndex < totalComparision)
		{
			data.update(workIndex, numTests, cmInputs, shapes, bounds, contactDistance, restDistances, startIndices, particleSystems);

			PxU32* cellStart = data.particleSystem->mDiffuseCellStart;
			PxU32* cellEnd = data.particleSystem->mDiffuseCellEnd;

			// If there is no diffuse particles or is not using PBD, skip this function
			if (data.particleSystem->mCommonData.mMaxDiffuseParticles != 0)
			{
				PxU32 gridHash = data.gridHash;

				// get the start of bucket for this cell
				PxU32 startIndex = cellStart[gridHash]; // startIndex might be EMPTY_CELL

				if (startIndex != EMPTY_CELL)
				{
					// get the end of bucket for this cell
					PxU32 endIndex = cellEnd[gridHash];

					range = endIndex - startIndex;
				}
			}
		}

		//Now we have "range", which tells us if this workIndex is interesting...

		PxU32 rangeMask = __ballot_sync(FULL_MASK, range);

		PxU32 count = __popc(rangeMask);

		if (threadIndexInWarp == 0)
			shWarpSum[threadIdx.x / 32] = count;

		__syncthreads();

		PxU32 warpCount = 0;
		if (threadIndexInWarp < NumWarps)
			warpCount = shWarpSum[threadIndexInWarp];

		PxU32 blockRunSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, warpCount);

		PxU32 totalBlock = __shfl_sync(FULL_MASK, blockRunSum, 31);

		blockRunSum = blockRunSum + workCount - warpCount;

		blockRunSum = __shfl_sync(FULL_MASK, blockRunSum, threadIdx.x / 32);

		PxU32 overflowIndex = 0xFFFFFFFF;
		if (range)
		{
			PxU32 outIndex = blockRunSum + warpScanExclusive(rangeMask, threadIndexInWarp);

			if (outIndex < PxgParticleSystemKernelBlockDim::PS_COLLISION)
			{
				shWorkIndices[outIndex] = workIndex;
			}
			else
				overflowIndex = outIndex - PxgParticleSystemKernelBlockDim::PS_COLLISION;
		}

		__syncthreads();

		workCount = workCount + totalBlock;

		if (workCount >= PxgParticleSystemKernelBlockDim::PS_COLLISION)
		{
			PxU32 thisWorkIndex = workIndex;

			workIndex = shWorkIndices[threadIdx.x];
			data.update(workIndex, numTests, cmInputs, shapes, bounds, contactDistance, restDistances, startIndices, particleSystems);
			
			psDiffusePrimitivesCollision(isTGS, data.gridHash, data.rigidShape, data.particleShape, data.rigidCacheRef, data.particleCacheRef, transformCache,
				*data.particleSystem, data.particleSystemId, data.cDistance, data.restDistance, materials, workIndex,
				totalComparision, shapeToRigidRemapTable);

			workCount -= PxgParticleSystemKernelBlockDim::PS_COLLISION;

			if (overflowIndex != 0xFFFFFFFF)
			{
				shWorkIndices[overflowIndex] = thisWorkIndex;
			}
		}

	}


	if (workCount)
	{
		PxU32 workIndex = 0xFFFFFFFF;
		PxgCellData data;

		if (threadIdx.x < workCount)
		{
			workIndex = shWorkIndices[threadIdx.x];
			data.update(workIndex, numTests, cmInputs, shapes, bounds, contactDistance, restDistances, startIndices, particleSystems);
		}

		psDiffusePrimitivesCollision(isTGS, data.gridHash, data.rigidShape, data.particleShape, data.rigidCacheRef, data.particleCacheRef, transformCache,
			*data.particleSystem, data.particleSystemId, data.cDistance, data.restDistance, materials, workIndex,
			totalComparision, shapeToRigidRemapTable);
	}


}


//Each warp deal with one cell
extern "C" __global__ void ps_convexCollisionLaunch(
	const bool									isTGS,
	const PxU32									numTests,
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	PxgShape* PX_RESTRICT						shapes,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxBounds3* PX_RESTRICT				bounds,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxReal* PX_RESTRICT					restDistances,
	const PxsMaterialData* PX_RESTRICT			materials,
	const PxU32* PX_RESTRICT					startIndices,					//the run sum for cells
	const PxU32*								totalNumPairs,
	PxgParticleSystem* PX_RESTRICT				particleSystems,
	const PxNodeIndex*							shapeToRigidRemapTable,			//IG::NodeIndex
	PxgParticlePrimitiveContact* PX_RESTRICT	particleContacts,				//output
	PxI32*										numParticleContacts,			//output
	PxU64*										contactSortedByParticle,		//output
	PxU32*										tempContactByParticle,			//output
	PxU32*										contactRemapSortedByParticle,	//output
	PxU64*										contactByRigid,					//output
	PxU32*										tempContactByRigid,				//output
	PxU32*										contactRemapSortedByRigid,		//output
	const PxU32									maxContacts)
{
	const PxU32 numWarpPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION / WARP_SIZE;
	const PxU32 warpIndex = threadIdx.y;

	__shared__ char sEpa_scratch[sizeof(squawk::EpaScratch) * numWarpPerBlock];

	squawk::EpaScratch* epa_scratch = reinterpret_cast<squawk::EpaScratch*>(sEpa_scratch);
	squawk::EpaScratch& ss_epa_scratch = epa_scratch[warpIndex];

	__shared__ char sScratch[sizeof(ParticleConvexCollideScratch) * numWarpPerBlock];
	ParticleConvexCollideScratch* scratch = reinterpret_cast<ParticleConvexCollideScratch*>(sScratch);

	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem) * numWarpPerBlock];
	PxgParticleSystem* sScratchParticle = reinterpret_cast<PxgParticleSystem*>(particleSystemMemory);


	ParticleConvexCollideScratch& ss_scratch = scratch[warpIndex];
	PxgParticleSystem& ss_particleSystem = sScratchParticle[warpIndex];

	const PxU32 nbWarpsRequired = *totalNumPairs;

	const PxU32 totalNumWarps = PxgParticleSystemKernelGridDim::PS_COLLISION * numWarpPerBlock;

	const PxU32 nbIterationsPerWarps = (nbWarpsRequired + totalNumWarps - 1) / totalNumWarps;

	for (PxU32 i = 0; i < nbIterationsPerWarps; ++i)
	{
		const PxU32 workIndex = i + (warpIndex + numWarpPerBlock * blockIdx.x) * nbIterationsPerWarps;

		if (workIndex < nbWarpsRequired)
		{
			const PxU32 pos = binarySearch(startIndices, numTests, workIndex);

			//start index for the thread groups
			const PxU32 startIndex = startIndices[pos];

			PxgShape particleShape, rigidShape;
			PxU32 particleCacheRef, rigidCacheRef;
			LoadShapePairWarp<PxGeometryType::ePARTICLESYSTEM>(cmInputs, pos, shapes,
				particleShape, particleCacheRef, rigidShape, rigidCacheRef);

			const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;

			//each thread read 16 byte
			uint4* sParticleSystem = reinterpret_cast<uint4*>(&particleSystems[particleSystemId]);
			uint4* dParticleSystem = reinterpret_cast<uint4*>(&ss_particleSystem);
			warpCopy<uint4>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));
			__syncwarp();


			PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[rigidCacheRef];
			const PxReal restDistance = restDistances[pos];

			PxBounds3 bound0;
			PxBounds3_ReadWarp(bound0, bounds + particleCacheRef);

			PxBounds3 bound1;
			PxBounds3_ReadWarp(bound1, bounds + rigidCacheRef);

			PxBounds3 overlapBound = combine(bound0, bound1);
			overlapBound.fattenFast(cDistance);

			const PxReal cellWidth = ss_particleSystem.mCommonData.mGridCellWidth;
			uint3 wrappedGridSize = make_uint3(ss_particleSystem.mCommonData.mGridSizeX,
											   ss_particleSystem.mCommonData.mGridSizeY,
											   ss_particleSystem.mCommonData.mGridSizeZ);

			const PxU32 offset = workIndex - startIndex;
			const PxU32 gridHash = calcGridHashInBounds(overlapBound, cellWidth, offset, wrappedGridSize);

			psConvexCollision(isTGS, gridHash, rigidShape, particleShape, rigidCacheRef, particleCacheRef, transformCache,
				particleSystemId, cDistance, restDistance,
				materials, workIndex,
				shapeToRigidRemapTable,
				particleContacts,
				numParticleContacts, 
				contactSortedByParticle,
				tempContactByParticle,
				contactRemapSortedByParticle,
				contactByRigid,
				tempContactByRigid,
				contactRemapSortedByRigid,
				ss_scratch, ss_epa_scratch,
				ss_particleSystem,
				maxContacts);
		}
	}
}



//Each warp deal with one cell
extern "C" __global__ void ps_convexDiffuseCollisionLaunch(
	const bool									isTGS,
	const PxU32									numTests,
	const PxReal								toleranceLength,
	const PxgContactManagerInput * PX_RESTRICT	cmInputs,
	PxgShape * PX_RESTRICT						shapes,
	const PxsCachedTransform * PX_RESTRICT		transformCache,
	const PxBounds3 * PX_RESTRICT				bounds,
	const PxReal * PX_RESTRICT					contactDistance,
	const PxReal * PX_RESTRICT					restDistances,
	const PxsMaterialData * PX_RESTRICT			materials,
	const PxU32 * PX_RESTRICT					startIndices,					//the run sum for cells
	const PxU32 * totalNumPairs,
	PxgParticleSystem * PX_RESTRICT				particleSystems,
	const PxNodeIndex *							shapeToRigidRemapTable			//IG::NodeIndex
)
{
	const PxU32 numWarpPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION / WARP_SIZE;
	const PxU32 warpIndex = threadIdx.y;

	__shared__ char sEpa_scratch[sizeof(squawk::EpaScratch) * numWarpPerBlock];

	squawk::EpaScratch* epa_scratch = reinterpret_cast<squawk::EpaScratch*>(sEpa_scratch);
	squawk::EpaScratch& ss_epa_scratch = epa_scratch[warpIndex];

	__shared__ char sScratch[sizeof(ParticleConvexCollideScratch) * numWarpPerBlock];

	ParticleConvexCollideScratch* scratch = reinterpret_cast<ParticleConvexCollideScratch*>(sScratch);

	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem) * numWarpPerBlock];
	PxgParticleSystem* sScratchParticle = reinterpret_cast<PxgParticleSystem*>(particleSystemMemory);


	ParticleConvexCollideScratch& ss_scratch = scratch[warpIndex];
	PxgParticleSystem& ss_particleSystem = sScratchParticle[warpIndex];

	const PxU32 nbWarpsRequired = *totalNumPairs;

	const PxU32 totalNumWarps = PxgParticleSystemKernelGridDim::PS_COLLISION * numWarpPerBlock;

	const PxU32 nbIterationsPerWarps = (nbWarpsRequired + totalNumWarps - 1) / totalNumWarps;


	for (PxU32 i = 0; i < nbIterationsPerWarps; ++i)
	{
		const PxU32 workIndex = i + (warpIndex + numWarpPerBlock * blockIdx.x) * nbIterationsPerWarps;

		if (workIndex < nbWarpsRequired)
		{
			const PxU32 pos = binarySearch(startIndices, numTests, workIndex);

			//start index for the thread groups
			const PxU32 startIndex = startIndices[pos];

			PxgShape particleShape, rigidShape;
			PxU32 particleCacheRef, rigidCacheRef;
			LoadShapePairWarp<PxGeometryType::ePARTICLESYSTEM>(cmInputs, pos, shapes,
				particleShape, particleCacheRef, rigidShape, rigidCacheRef);
			
			const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;

			//each thread read 16 byte
			uint4* sParticleSystem = reinterpret_cast<uint4*>(&particleSystems[particleSystemId]);
			uint4* dParticleSystem = reinterpret_cast<uint4*>(&ss_particleSystem);
			warpCopy<uint4>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));
			__syncwarp();

			// If there is no diffuse particles or is not using PBD, skip this function
			if (ss_particleSystem.mCommonData.mMaxDiffuseParticles == 0)
				return;

			PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[rigidCacheRef];
			const PxReal restDistance = restDistances[pos];

			PxBounds3 bound0;
			PxBounds3_ReadWarp(bound0, bounds + particleCacheRef);

			PxBounds3 bound1;
			PxBounds3_ReadWarp(bound1, bounds + rigidCacheRef);

			PxBounds3 overlapBound = combine(bound0, bound1);
			overlapBound.fattenFast(cDistance);

			const PxReal cellWidth = ss_particleSystem.mCommonData.mGridCellWidth;
			const uint3 wrappedGridSize = make_uint3(ss_particleSystem.mCommonData.mGridSizeX,
													 ss_particleSystem.mCommonData.mGridSizeY,
													 ss_particleSystem.mCommonData.mGridSizeZ);

			const PxU32 offset = workIndex - startIndex;
			const PxU32 gridHash = calcGridHashInBounds(overlapBound, cellWidth, offset, wrappedGridSize);

			psConvexDiffuseCollision(isTGS, gridHash, rigidShape, particleShape, rigidCacheRef, particleCacheRef, transformCache,
				particleSystemId, cDistance, restDistance,
				materials, workIndex,
				shapeToRigidRemapTable,
				ss_scratch, ss_epa_scratch,
				ss_particleSystem);
		}
	}
}

extern "C" __global__ void ps_reorderPrimitiveContactsLaunch(
	const PxgParticlePrimitiveContact* PX_RESTRICT particleContacts,
	const PxI32* numParticleContacts,
	const PxU32* remapByRigid,
	const PxU32* remapByParticle,
	PxgParticlePrimitiveContact* PX_RESTRICT sortedContactsByRigid,					//output
	PxgParticlePrimitiveContact* PX_RESTRICT sortedContactsByParticle				//output
	)
{
	
	const PxU32 totalNumContacts = *numParticleContacts;

	const PxU32 numIternations = (totalNumContacts + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;

	//const PxU32 globalThreadIndex = threadIdx.x + blockIdx.x * blockDim.x;
	/*if (globalThreadIndex == 0)
		printf("totalNumContacts %i blockDim.x %i gridDim.x %i\n", totalNumContacts, blockDim.x, gridDim.x);
*/
	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 index = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;
		if (index >= totalNumContacts)
			return;

		sortedContactsByParticle[index] = particleContacts[remapByParticle[index]];
		sortedContactsByRigid[index] = particleContacts[remapByRigid[index]];
	}
}
