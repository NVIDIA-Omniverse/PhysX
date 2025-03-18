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

#include "foundation/PxPreprocessor.h"

#include "PxgCommonDefines.h"
#include "PxgSimulationCoreKernelIndices.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgSolverBody.h"
#include "PxvDynamics.h"
#include "PxgShapeSim.h"
#include "PxgBodySim.h"
#include "cutil_math.h"
#include "reduction.cuh"
#include "updateCacheAndBound.cuh"
#include "PxsRigidBody.h"
#include "PxgArticulation.h"
#include "PxgAggregate.h"
#include "PxgAABBManager.h"
#include <assert.h>
#include <stdio.h>

using namespace physx;

extern "C" __host__ void initSimulationControllerKernels1() {}

extern "C" __global__ void mergeTransformCacheAndBoundArrayChanges(
    PxBounds3* PX_RESTRICT deviceBounds,
    PxsCachedTransform* PX_RESTRICT deviceTransforms,
    const PxBounds3* PX_RESTRICT boundsArray,
    const PxsCachedTransform* PX_RESTRICT transformsArray,
    const PxBoundTransformUpdate* PX_RESTRICT changes,
    const PxU32 numChanges
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numChanges) {
        PxU32 indexTo = changes[idx].indexTo;
        PxU32 indexFrom = changes[idx].indexFrom & 0x7FFFFFFF;
        bool isNew = (changes[idx].indexFrom & (1U << 31)) != 0;
        if (isNew) {
            deviceBounds[indexTo] = boundsArray[indexFrom];
            deviceTransforms[indexTo] = transformsArray[indexFrom];
        } else {
            deviceBounds[indexTo] = deviceBounds[indexFrom];
            deviceTransforms[indexTo] = deviceTransforms[indexFrom];
        }
    }
}

extern "C" __global__ void updateTransformCacheAndBoundArrayLaunch(const PxgSimulationCoreDesc* scDesc)
{
	const PxgSolverBodySleepData* PX_RESTRICT gSleepData = scDesc->mSleepData;

	const PxU32* PX_RESTRICT gBodyDataIndices = scDesc->mBodyDataIndices;
	const PxgShape* PX_RESTRICT gShapes = scDesc->mShapes;

	const PxgBodySim* PX_RESTRICT gBodySimPool = scDesc->mBodySimBufferDeviceData;

	const PxU32 gNumShapes = scDesc->mNbTotalShapes;
	const PxgShapeSim* PX_RESTRICT gShapeSimPool = scDesc->mShapeSimsBufferDeviceData;

	const PxgArticulation* PX_RESTRICT gArticulations = scDesc->mArticulationPool;
	const PxgSolverBodySleepData* PX_RESTRICT gArticulationSleepData = scDesc->mArticulationSleepDataPool;
	
	PxsCachedTransform* PX_RESTRICT gTransformCache = scDesc->mTransformCache;
	PxBounds3* PX_RESTRICT gBounds = scDesc->mBounds;

	//Each shape has a corresponding unfrozen element
	PxU32* PX_RESTRICT frozen = scDesc->mFrozen;
	PxU32* PX_RESTRICT unfrozen = scDesc->mUnfrozen;
	//Each shape has a updated element corresponding to the elementIndex
	PxU32* PX_RESTRICT updated = scDesc->mUpdated;

	//Each body has a corresponding active and deactive element
	PxU32* PX_RESTRICT active = scDesc->mActivate;
	PxU32* PX_RESTRICT deactivate = scDesc->mDeactivate;

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	for(PxU32 i=idx; i<gNumShapes; i+=blockDim.x * gridDim.x)
	{
		const PxgShapeSim& shapeSim = gShapeSimPool[i];

		const PxNodeIndex bodySimNodeIndex = shapeSim.mBodySimIndex; // bodySimIndex is the same as nodeIndex in the IG

		//not static body or deleted shape
		if (!bodySimNodeIndex.isStaticBody())
		{
			const PxU32 elementIndex = i; // this is the transform cache and bound array index
			const PxU32 bodySimIndex = bodySimNodeIndex.index();
			//printf("i %i bodySimIndex %i\n", idx, bodySimIndex );
			
			const PxgBodySim& bodySim = gBodySimPool[bodySimIndex];

			const PxU32 shapeFlags = shapeSim.mShapeFlags;
			bool isBP = (shapeFlags & PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE));

			bool isBPOrSq = (shapeFlags & PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE));

			if (!bodySimNodeIndex.isArticulation())
			{
				const PxU32 activeNodeIndex = gBodyDataIndices[bodySimIndex];
								
				//if activeNodeIndex is valid, which means this node is active
				if (activeNodeIndex != 0xFFFFFFFF)
				{
					const PxU32 internalFlags = gSleepData[activeNodeIndex].internalFlags;
					const PxTransform body2World = bodySim.body2World.getTransform();

					if ((internalFlags & PxsRigidBody::eFREEZE_THIS_FRAME) && (internalFlags &  PxsRigidBody::eFROZEN))
					{
						frozen[i] = 1;
						gTransformCache[elementIndex].flags = PxsTransformFlag::eFROZEN;
					}
					else if (internalFlags & PxsRigidBody::eUNFREEZE_THIS_FRAME)
					{
						unfrozen[i] = 1;
					}

					if (!(internalFlags & PxsRigidBody::eFROZEN) || (internalFlags & PxsRigidBody::eFREEZE_THIS_FRAME))
					{					
						if (isBP)
							updated[elementIndex] = 1;

						const PxTransform absPos = getAbsPose(body2World, shapeSim.mTransform, bodySim.body2Actor_maxImpulseW.getTransform());

						updateCacheAndBound(absPos, shapeSim, elementIndex, gTransformCache, gBounds, gShapes, isBPOrSq);
					}

					if (internalFlags & PxsRigidBody::eACTIVATE_THIS_FRAME)
						active[bodySimIndex] = 1;
					else if (internalFlags & PxsRigidBody::eDEACTIVATE_THIS_FRAME)
						deactivate[bodySimIndex] = 1;
				}
			}
			else
			{			
				//This is articulation
				const PxU32 articulationId = bodySim.articulationRemapId;
				const PxgArticulation& articulation = gArticulations[articulationId];
				const PxgSolverBodySleepData artiSleepData = gArticulationSleepData[articulationId];
				const PxU32 internalFlags = artiSleepData.internalFlags;

				const PxU32 linkId = bodySimNodeIndex.articulationLinkId();

				const PxTransform body2World = articulation.linkBody2Worlds[linkId];

				if (isBP)
					updated[elementIndex] = 1;

				const PxTransform body2Actor = articulation.linkBody2Actors[linkId];

				const PxTransform absPos = getAbsPose(body2World, shapeSim.mTransform, body2Actor);

				updateCacheAndBound(absPos, shapeSim, elementIndex, gTransformCache, gBounds, gShapes, isBPOrSq);

				if (internalFlags & PxsRigidBody::eACTIVATE_THIS_FRAME)
					active[bodySimIndex] = 1;
				else if (internalFlags & PxsRigidBody::eDEACTIVATE_THIS_FRAME)
					deactivate[bodySimIndex] = 1;
			}
		}
	}
}

//after  updateTransformCacheAndBoundArrayLaunch, we need to update the flags in the transform cache
extern "C" __global__ void updateChangedAABBMgrHandlesLaunch(const PxgSimulationCoreDesc* scDesc)
{
	const PxU32 gNumElements = scDesc->mBitMapWordCounts * 32;

	const PxU32* updated = scDesc->mUpdated;

	PxU32* gChangedAABBMgrHandles = scDesc->mChangedAABBMgrHandles;

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x &  (WARP_SIZE -1);

	for (PxU32 i = idx; i<gNumElements; i += blockDim.x * gridDim.x)
	{
		const PxU32 updateBit = updated[i];
		const PxU32 word = __ballot_sync(FULL_MASK, updateBit);

		if(threadIndexInWarp == 0)
		{
			gChangedAABBMgrHandles[i/WARP_SIZE] = word;
		}
	}
}

//This kernel merge direct API updated handle and the CPU API updated handle
extern "C" __global__ void mergeChangedAABBMgrHandlesLaunch(const PxgUpdateActorDataDesc* updateActorDesc)
{
	//max number of shapes
	const PxU32 gNumElements = updateActorDesc->mBitMapWordCounts * 32;

	//This is Direct API changed handles
	const PxU32* updated = updateActorDesc->mUpdated;

	//This is CPU API changed handles
	PxU32* gChangedAABBMgrHandles = updateActorDesc->mChangedAABBMgrHandles;

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x &  (WARP_SIZE - 1);

	for (PxU32 i = idx; i < gNumElements; i += blockDim.x * gridDim.x)
	{
		const PxU32 updateBit = updated[i];

		const PxU32 word = __ballot_sync(FULL_MASK, updateBit);

		if (threadIndexInWarp == 0)
		{
			gChangedAABBMgrHandles[i / WARP_SIZE] |= word;
		}
	}
}

extern "C" __global__ void computeFrozenAndUnfrozenHistogramLaunch(const PxgSimulationCoreDesc* scDesc)
{
	const PxU32 WARP_PERBLOCK_SIZE = PxgSimulationCoreKernelBlockDim::COMPUTE_FROZEN_UNFROZEN_HISTOGRAM/WARP_SIZE;
	const PxU32 LOG2_WARP_PERBLOCK_SIZE = 3;

	assert((1 << LOG2_WARP_PERBLOCK_SIZE) == WARP_PERBLOCK_SIZE);

	__shared__ PxU32 sFrozenWarpAccumulator[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sUnFrozenWarpAccumulator[WARP_PERBLOCK_SIZE];

	__shared__ PxU32 sFrozenBlockAccumulator;
	__shared__ PxU32 sUnfrozenBlockAccumulator;

	PxU32* gFrozen = scDesc->mFrozen;
	PxU32* gUnfrozen = scDesc->mUnfrozen;
	PxU32* gFrozenBlock = scDesc->mFrozenBlockAndRes;
	PxU32* gUnfrozenBlock = scDesc->mUnfrozenBlockAndRes;

	const PxU32 gNbTotalShapes = scDesc->mNbTotalShapes;

	const PxU32 nbBlocksRequired = (gNbTotalShapes + blockDim.x-1)/blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);
	const PxU32 idx = threadIdx.x;

	if(threadIdx.x == 0)
	{
		sFrozenBlockAccumulator = 0;
		sUnfrozenBlockAccumulator = 0;
	}

	__syncthreads();

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		//frozen/unfrozen is either 0 or 1
		PxU32 frozen = 0, unfrozen = 0;
		if(workIndex < gNbTotalShapes)
		{
			frozen = gFrozen[workIndex];
			unfrozen = gUnfrozen[workIndex];
		}

		const PxU32 threadMask = (1<<threadIndexInWarp)-1;

		const PxU32 frozenAccumVal = __popc(__ballot_sync(FULL_MASK, frozen)&threadMask);
		const PxU32 unfrozenAccumVal = __popc(__ballot_sync(FULL_MASK, unfrozen)&threadMask);

		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sFrozenWarpAccumulator[warpIndex] = frozenAccumVal + frozen;
			sUnFrozenWarpAccumulator[warpIndex] = unfrozenAccumVal + unfrozen;
		}

		const PxU32 prevFrozenBlockAccumulator = sFrozenBlockAccumulator;
		const PxU32 prevUnfrozenBlockAccumulator = sUnfrozenBlockAccumulator;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx < WARP_PERBLOCK_SIZE)
		{
			const PxU32 frozenValue = sFrozenWarpAccumulator[threadIndexInWarp];
			const PxU32 unfrozenValue = sUnFrozenWarpAccumulator[threadIndexInWarp];
			const PxU32 frozenOutput = warpScan<AddOpPxU32, PxU32, LOG2_WARP_PERBLOCK_SIZE>(mask_idx, frozenValue) - frozenValue;
			const PxU32 unfrozenOutput = warpScan<AddOpPxU32, PxU32, LOG2_WARP_PERBLOCK_SIZE>(mask_idx, unfrozenValue) - unfrozenValue;
			sFrozenWarpAccumulator[threadIndexInWarp] = frozenOutput;
			sUnFrozenWarpAccumulator[threadIndexInWarp] = unfrozenOutput;
			//const PxU32 output = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(idx, threadIndexInWarp, sWarpAccumulator, value, value);
			if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
			{
				sFrozenBlockAccumulator +=(frozenOutput + frozenValue);
				sUnfrozenBlockAccumulator +=(unfrozenOutput + unfrozenValue);
			}
		}
		
		__syncthreads();

		if(workIndex < gNbTotalShapes)
		{
			//Now output both histograms...
			gFrozen[workIndex] = frozenAccumVal + prevFrozenBlockAccumulator + sFrozenWarpAccumulator[warpIndex];
			gUnfrozen[workIndex] = unfrozenAccumVal + prevUnfrozenBlockAccumulator + sUnFrozenWarpAccumulator[warpIndex];
		}
	}

	if(threadIdx.x == 0)
	{
		gFrozenBlock[blockIdx.x] = sFrozenBlockAccumulator;
		gUnfrozenBlock[blockIdx.x] = sUnfrozenBlockAccumulator;
	}
}

extern "C" __global__ void outputFrozenAndUnfrozenHistogram(PxgSimulationCoreDesc* scDesc)
{
	const PxU32 nbBlocks = PxgSimulationCoreKernelGridDim::OUTPUT_FROZEN_UNFROZEN_HISTOGRAM;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	__shared__ PxU32 sFrozenBlockAccum[nbBlocks];
	__shared__ PxU32 sUnfrozenBlockAccum[nbBlocks];

	const PxU32 idx = threadIdx.x;

	PxU32* gFrozen = scDesc->mFrozen;
	PxU32* gUnfrozen = scDesc->mUnfrozen;
	PxU32* gFrozenBlock = scDesc->mFrozenBlockAndRes;
	PxU32* gUnfrozenBlock = scDesc->mUnfrozenBlockAndRes;

	const PxU32 gNbTotalShapes = scDesc->mNbTotalShapes;
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	PxU32 frozen = 0;
	PxU32 frozenOutput = 0;
	PxU32 unfrozen = 0;
	PxU32 unfrozenOutput = 0;

	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < nbBlocks);
	if(idx < nbBlocks)
	{
		frozen = gFrozenBlock[idx];
		frozenOutput =  warpScan<AddOpPxU32, PxU32>(mask_idx, frozen) - frozen;
		sFrozenBlockAccum[idx] = frozenOutput;

		unfrozen = gUnfrozenBlock[idx];
		unfrozenOutput =  warpScan<AddOpPxU32, PxU32>(mask_idx, unfrozen) - unfrozen;
		sUnfrozenBlockAccum[idx] = unfrozenOutput;
	}

	if(globalThreadIndex == (nbBlocks-1))
	{
		scDesc->mTotalFrozenShapes = frozenOutput + frozen;
		scDesc->mTotalUnfrozenShapes = unfrozenOutput + unfrozen;
	}

	const PxU32 totalBlockRequired = (gNbTotalShapes + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;

	__syncthreads();
	
	const PxU32 frozenBlockAccum = sFrozenBlockAccum[blockIdx.x];
	const PxU32 unfrozenBlockAccum = sUnfrozenBlockAccum[blockIdx.x];

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < gNbTotalShapes)
		{
			gFrozen[workIndex] = gFrozen[workIndex] + frozenBlockAccum;
			gUnfrozen[workIndex] = gUnfrozen[workIndex] + unfrozenBlockAccum;
		}
	}
}

extern "C" __global__ void createFrozenAndUnfrozenArray(const PxgSimulationCoreDesc* scDesc)
{
	PxU32* gFrozen = scDesc->mFrozen;
	PxU32* gUnfrozen = scDesc->mUnfrozen;

	PxU32* gFrozenRes = scDesc->mFrozenBlockAndRes;
	PxU32* gUnfrozenRes = scDesc->mUnfrozenBlockAndRes;

	const PxU32 gNbTotalShapes = scDesc->mNbTotalShapes;
	const PxU32 gNbFrozenTotalShapes = scDesc->mTotalFrozenShapes;
	const PxU32 gNbUnfrozenTotalShapes = scDesc->mTotalUnfrozenShapes;

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	for(PxU32 i=idx; i < gNbFrozenTotalShapes; i+= blockDim.x * gridDim.x)
	{
		gFrozenRes[i] = binarySearch<PxU32>(gFrozen, gNbTotalShapes, i);
	}

	for(PxU32 i=idx; i< gNbUnfrozenTotalShapes; i+= blockDim.x * gridDim.x)
	{
		gUnfrozenRes[i] = binarySearch<PxU32>(gUnfrozen, gNbTotalShapes, i);
	}
}
