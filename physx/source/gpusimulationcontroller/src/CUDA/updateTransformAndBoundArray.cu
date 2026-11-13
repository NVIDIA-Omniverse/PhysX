// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
#include "PxgBoundTransformUpdate.h"
#include "BpFiltering.h"
#include <assert.h>
#include <stdio.h>

using namespace physx;

extern "C" __host__ void initSimulationControllerKernels1() {}

// Shared by the merge and the update after the solver, so that both compute the same shape pose.
static __device__ PxTransform getShapeAbsPose(const PxgShapeSim& shapeSim, const PxgBodySim* PX_RESTRICT bodySims, const PxgArticulation* PX_RESTRICT articulations)
{
	const PxNodeIndex nodeIndex = shapeSim.mBodySimIndex;
	const PxgBodySim& bodySim = bodySims[nodeIndex.index()];
	if (nodeIndex.isArticulation())
	{
		PX_ASSERT(articulations);
		const PxgArticulation& articulation = articulations[bodySim.articulationRemapId];
		const PxU32 linkId = nodeIndex.articulationLinkId();
		return getAbsPose(articulation.linkBody2Worlds[linkId], shapeSim.mTransform, articulation.linkBody2Actors[linkId]);
	}
	return getAbsPose(bodySim.body2World.getTransform(), shapeSim.mTransform, bodySim.body2Actor_maxImpulseW.getTransform());
}

extern "C" __global__ void mergeTransformCacheAndBoundArrayChanges(
	PxBounds3* PX_RESTRICT deviceBounds,
	PxsCachedTransform* PX_RESTRICT deviceTransforms,
	const PxBounds3* PX_RESTRICT boundsArray,
	const PxsCachedTransform* PX_RESTRICT transformsArray,
	const PxgBoundTransformUpdate* PX_RESTRICT changes,
	const PxU32 numChanges,
	const PxgShapeSim* PX_RESTRICT shapeSims,
	const PxgBodySim* PX_RESTRICT bodySims,
	const PxgArticulation* PX_RESTRICT articulations,
	const PxgShape* PX_RESTRICT shapes)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < numChanges)
	{
		PxU32 index = changes[idx].index & PxgBoundTransformUpdate::eINDEX_MASK;
		PxU32 kind = changes[idx].index & PxgBoundTransformUpdate::eKIND_MASK;
		if (kind == PxgBoundTransformUpdate::eCOMPUTE_FROM_GPU_POSE)
		{
			const PxgShapeSim& shapeSim = shapeSims[index];
			// A shape removed after its re-insertion in the same step has no body any more. Its element ID is not reused before
			// the merge, because released IDs return to the pool only in fetchResults().
			if (shapeSim.mBodySimIndex.isValid())
			{
				const PxU32 shapeFlags = shapeSim.mShapeFlags;
				const bool isBPOrSq = shapeFlags & PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE);
				updateCacheAndBound(getShapeAbsPose(shapeSim, bodySims, articulations), shapeSim, index, deviceTransforms, deviceBounds, shapes, isBPOrSq);
			}
		}
		else
		{
			PX_ASSERT(kind == PxgBoundTransformUpdate::eCOPY_FROM_CPU);
			deviceBounds[index] = boundsArray[index];
			deviceTransforms[index] = transformsArray[index];
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

						const PxTransform absPos = getShapeAbsPose(shapeSim, gBodySimPool, NULL);

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
				const PxgSolverBodySleepData artiSleepData = gArticulationSleepData[articulationId];
				const PxU32 internalFlags = artiSleepData.internalFlags;

				if (isBP)
					updated[elementIndex] = 1;

				const PxTransform absPos = getShapeAbsPose(shapeSim, gBodySimPool, gArticulations);

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
extern "C" __global__ void mergeChangedAABBMgrHandlesLaunch(const PxU32* updated, const PxU32 nbUpdated, const PxU32* groups, const PxU32 nbGroups,
	PxU32* changedAABBMgrHandles, const PxU32 nbChangedAABBMgrHandleWords)
{
	// Both bounds are multiples of 32, so whole warps enter and leave the loop together.
	const PxU32 gNumElements = PxMin(nbChangedAABBMgrHandleWords * 32, nbUpdated);

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x &  (WARP_SIZE - 1);

	for (PxU32 i = idx; i < gNumElements; i += blockDim.x * gridDim.x)
	{
		// A shape with neither the simulation nor the trigger flag has no bounds in the broadphase.
		const PxU32 updateBit = updated[i] && i < nbGroups && groups[i] != PxU32(Bp::FilterGroup::eINVALID);

		const PxU32 word = __ballot_sync(FULL_MASK, updateBit);

		if (threadIndexInWarp == 0)
		{
			changedAABBMgrHandles[i / WARP_SIZE] |= word;
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
