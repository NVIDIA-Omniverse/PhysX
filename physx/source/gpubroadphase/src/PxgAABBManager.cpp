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

#include "PxgAABBManager.h"
#include "PxgAggregate.h"
#include "PxgAggregateDesc.h"
#include "PxsHeapMemoryAllocator.h"
#include "common/PxPhysXCommonConfig.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "CudaKernelWrangler.h"
#include "common/PxProfileZone.h"
#include "BpBroadPhaseUpdate.h"
#include "PxgSapBox1D.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxBounds3.h"
#include "vector_types.h"
#include "PxgBroadPhaseKernelIndices.h"
#include "PxSceneDesc.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgCudaUtils.h"
#include "PxgKernelLauncher.h"
#include "PxgCudaMemoryAllocator.h"

#define GPU_AABB_DEBUG 0
#define USE_NEW_LAUNCH_FUNCTION 1

#if GPU_AABB_DEBUG
	#define GPU_DEBUG_STREAM(s, x)									\
	{																\
		const CUresult err = mCudaContext->streamSynchronize(s);	\
		if(err != CUDA_SUCCESS)										\
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, x);	\
	}
#else
	#define GPU_DEBUG_STREAM(s, x)
#endif

#define PROLOG	mGpuKernelWranglerManager->mKernelWrangler, mCudaContext
#if USE_NEW_LAUNCH_FUNCTION
	#define KERNEL_PARAM_TYPE	void*
	#define CUDA_KERNEL_PARAM	PX_CUDA_KERNEL_PARAM2
	#define EPILOG				bpStream, kernelParams, PX_FL
#else
	#define KERNEL_PARAM_TYPE	PxCudaKernelParam
	#define CUDA_KERNEL_PARAM	PX_CUDA_KERNEL_PARAM
	#define EPILOG				bpStream, kernelParams, sizeof(kernelParams), PX_FL
#endif

using namespace physx;
using namespace Bp;

PX_IMPLEMENT_OUTPUT_ERROR

static PX_FORCE_INLINE PxgCudaBroadPhaseSap& getGPUBroadPhase(BroadPhase& bp)
{
	PX_ASSERT(bp.getType()==PxBroadPhaseType::eGPU);
	return static_cast<PxgCudaBroadPhaseSap&>(bp);
}

static void initEnvEntry(PxInt32ArrayPinned& envIDs, BoundsIndex index, PxU32 envID, PxU32 boundsSize)
{
	// PT: we avoid allocating anything when the feature is not used, and allocate everything lazily
	// as soon as a non-default environment ID is needed. We need 'boundsSize' to make sure we allocate
	// a large enough array when setEnvironmentID() is called after some objects have already been created
	// and the scene already simulated. See EnvIDTests_GPU.EnvironmentID_EdgeCase for why is it needed.

	const bool validEntry = envID != PX_INVALID_U32;

	const PxU32 currentSize = envIDs.size();

	if(validEntry || currentSize)
	{
		if((index + 1) >= currentSize)
			envIDs.resize(PxMax(boundsSize, PxNextPowerOfTwo(index + 1)), PX_INVALID_U32);
	}

	if(validEntry || index < envIDs.size())
		envIDs[index] = envID;
}

PxgAggregateBuffer::PxgAggregateBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager) :
	updateBoundIndices(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	boundIndices(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	sortedProjections(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	sortedHandles(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	sapBox1D(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	startMasks(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	comparisons(heapMemoryManager, PxsHeapStats::eBROADPHASE)
{
}

PxgAABBManager::PxgAABBManager(PxgCudaKernelWranglerManager* gpuKernelWrangler,
	PxCudaContextManager* cudaContextManager,
	PxgHeapMemoryAllocatorManager* heapMemoryManager,
	const PxGpuDynamicsMemoryConfig& config,
	BroadPhase& bp, BoundsArray& boundsArray, PxFloatArrayPinned& contactDistance,
	PxU32 maxNbAggregates, PxU32 maxNbShapes, PxVirtualAllocator& allocator, PxU64 contextID,
	PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode) :
	AABBManagerBase				(bp, boundsArray, contactDistance, maxNbAggregates, maxNbShapes, allocator, contextID, kineKineFilteringMode, staticKineFilteringMode),
	mVolumDataBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mGpuKernelWranglerManager	(gpuKernelWrangler),
	mCudaContextManager			(cudaContextManager),
	mCudaContext				(cudaContextManager->getCudaContext()),
	mHeapMemoryManager			(heapMemoryManager),
	mAggregatePairs				(allocator),
	mDirtyAggregateIndices		(allocator),
	mDirtyAggregates			(allocator),
	mFoundPairs					(allocator),
	mLostPairs					(allocator),
	mDirtyBoundIndices			(allocator),
	mDirtyBoundStartIndices		(allocator),
	mRemovedAggregatedBounds	(allocator),
	mAddedAggregatedBounds		(allocator),
	mAggregatedBoundMap			(allocator),
	mAggregateBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mAggregatePairsBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mDirtyAggregateIndiceBuf	(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mDirtyAggregateBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mDirtyBoundIndicesBuf		(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mDirtyBoundStartIndicesBuf	(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRemovedAggregatedBoundsBuf (heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mAddedAggregatedBoundsBuf 	(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mAggPairBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mNumAggPairBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE), 
	mAggregateDescBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mFoundPairsBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mLostPairsBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mFreeIDPool					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mFreeIDs					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRemoveBitmap				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRemoveHistogram			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mAggregatedBoundsBuf		(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mAddedHandleBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRemovedHandleBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mChangedAABBMgrHandlesBuf	(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mMaxFoundLostPairs			(config.foundLostAggregatePairsCapacity),
	mMaxAggPairs				(config.totalAggregatePairsCapacity),
	mFoundPairTask				(this),
	mLostPairTask				(this),
	mGPUStateChanged			(false),
	mPersistentStateChanged		(true)
{
	getGPUBroadPhase(bp).setGPUAABBManager(this);

	mAggregates.resize(100);
	mAggregatePairs.resize(100);
	mAggregateBufferArray.resize(100);

	mFoundPairs.forceSize_Unsafe(0);
	mFoundPairs.reserve(mMaxFoundLostPairs);

	mLostPairs.forceSize_Unsafe(0);
	mLostPairs.reserve(mMaxFoundLostPairs);

	mNumAggregatesSlots = 0;

	PxScopedCudaLock _lock_(*mCudaContextManager);
	mAggregateDescBuf.allocate(sizeof(PxgAggregateDesc), PX_FL);
	mAggPairBuf.allocate(sizeof(PxgAggregatePair) * mMaxAggPairs, PX_FL);
	mFreeIDs.allocate(sizeof(PxU32)*mMaxAggPairs, PX_FL);
	const PxU32 bitMapSize = mMaxAggPairs;
	mRemoveBitmap.allocate(sizeof(PxU32)*bitMapSize, PX_FL);

	// AD: this stores a per-block sum for each iteration we have to do in the kernel
	// to get the number of iterations, we need to figure out how often we have to iterate for a given mMaxAggPairs 
	// then we have to multiply by the number of blocks launched.

	const PxU32 warpSize = 32;
	const PxU32 maxNbWarpsNeeded = (mMaxAggPairs + warpSize -1) / warpSize;
	const PxU32 nbWarpsPerIteration = (PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE / warpSize) * PxgBPKernelGridDim::BP_AGGREGATE_REMOVE;
	const PxU32 nbIterationsNeeded = (maxNbWarpsNeeded + nbWarpsPerIteration - 1)/ nbWarpsPerIteration; // round up
	const PxU32 nbBlocksNeeded = nbIterationsNeeded * PxgBPKernelGridDim::BP_AGGREGATE_REMOVE;
	mRemoveHistogram.allocate(sizeof(PxU32) * nbBlocksNeeded, PX_FL); //We store the sum across all 1024 removed pairs

	mFreeIDPool.allocate(sizeof(PxgFreeBufferList), PX_FL);
	mNumAggPairBuf.allocate(sizeof(PxU32), PX_FL);
	mFoundPairsBuf.allocate(sizeof(PxgBroadPhasePair) * mMaxFoundLostPairs, PX_FL);
	mLostPairsBuf.allocate(sizeof(PxgBroadPhasePair) * mMaxFoundLostPairs, PX_FL);

	mAggregateDesc = PX_PINNED_MEMORY_ALLOC(PxgAggregateDesc, *mCudaContextManager, 1);

	//One-time zeroing. No access to the stream at this point so do it synchronously for now
	mCudaContext->memsetD32(mFreeIDPool.getDevicePtr(), 0, sizeof(PxgFreeBufferList)/sizeof(PxU32));
	mCudaContext->memsetD32(mNumAggPairBuf.getDevicePtr(), 0, 1);
}

void PxgAABBManager::destroy()
{
	for (PxU32 i = 0; i < mAggregateBufferArray.size(); ++i)
	{
		PX_DELETE(mAggregateBufferArray[i]);
	}

	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mAggregateDesc);

	PX_DELETE_THIS;
}

AggregateHandle PxgAABBManager::createAggregate(BoundsIndex index, FilterGroup::Enum group, void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint, PxU32 envID)
{
	const PxU32 handle = mAggregatesIdPool.getNewID();
#if PX_CHECKED || PX_DEBUG
	if (mMaxAggPairs == 0)
	{
		PxGetFoundation().getErrorCallback().reportError(PxErrorCode::eINVALID_OPERATION, "PxgAABBManager::createAggregate() : Attempting to create an aggregate without reserving space for aggregate pairs. Please make sure you assign a suitable value to PxSceneDesc::gpuDynamicsConfig::foundLostAggregatePairsCapacity and PxSceneDesc::gpuDynamicsConfig::totalAggregatePairsCapacity.", PX_FL);
		return 0xFFFFFFFF;
	}
#endif

	if (mAggregates.capacity() <= handle)
	{
		mAggregates.resize(2 * handle + 1);

		mAggregateBufferArray.resize(2 * handle + 1);
	}

	PxgAggregate& aggregate = mAggregates[handle];
	aggregate.reset();

	aggregate.mEnvID = envID;
	aggregate.mIndex = index;
	aggregate.filterHint = filterHint;
	aggregate.updateBoundIndices = PX_ALLOCATE(PxU32, maxNumShapes, "updateBoundIndices");

	PX_ASSERT(aggregate.isNew);

	PxgAggregateBuffer* buffer = mAggregateBufferArray[handle];

	if (!buffer)
	{
		buffer = PX_NEW(PxgAggregateBuffer)(mHeapMemoryManager);

		mAggregateBufferArray[handle] = buffer;
	}

	//allocate device memory
	buffer->updateBoundIndices.allocate(maxNumShapes * sizeof(PxU32), PX_FL);
	buffer->boundIndices[0].allocate(maxNumShapes * sizeof(PxU32), PX_FL);
	buffer->boundIndices[1].allocate(maxNumShapes * sizeof(PxU32), PX_FL);
	buffer->sortedProjections[0].allocate(((maxNumShapes * 2 + 3) / 4) * sizeof(uint4) * 2, PX_FL);
	buffer->sortedProjections[1].allocate(((maxNumShapes * 2 + 3) / 4) * sizeof(uint4) * 2, PX_FL);
	buffer->sortedHandles[0].allocate(((maxNumShapes * 2 + 3) / 4) * sizeof(uint4) * 2, PX_FL);
	buffer->sortedHandles[1].allocate(((maxNumShapes * 2 + 3) / 4) * sizeof(uint4) * 2, PX_FL);
	buffer->sapBox1D[0].allocate(maxNumShapes * sizeof(PxgSapBox1D), PX_FL);
	buffer->sapBox1D[1].allocate(maxNumShapes * sizeof(PxgSapBox1D), PX_FL);
	buffer->startMasks[0].allocate((maxNumShapes*2+31)/32 * sizeof(PxU32), PX_FL);
	buffer->startMasks[1].allocate((maxNumShapes*2+31)/32 * sizeof(PxU32), PX_FL);
	buffer->comparisons[0].allocate(maxNumShapes * sizeof(PxU32), PX_FL);
	buffer->comparisons[1].allocate(maxNumShapes * sizeof(PxU32), PX_FL);

	//PxgAggregateBuffer& buffer = mAggregateBufferArray[handle];
	//buffer.maxNumShapes = maxNumShapes;

#if BP_USE_AGGREGATE_GROUP_TAIL
	initEntry(index, 0.0f, getAggregateGroup(), userData);
	PX_UNUSED(group);
#else
	initEntry(index, 0.0f, group, userData);
#endif

	mVolumeData[index].setAggregate(handle);

	mBoundsArray.setBounds(PxBounds3::empty(), index);	// PT: no need to set mPersistentStateChanged since "setBounds" already does something similar

	mNumAggregatesSlots = PxMax(mNumAggregatesSlots, handle + 1);

	mNbAggregates++;

	if (!mDirtyAggregateBitMap.boundedTest(handle))
	{
		mDirtyAggregateBitMap.growAndSet(handle);
		mDirtyAggregateIndices.pushBack(handle);
	}

	mAddedHandleMap.growAndSet(index);
	mAggregatedBoundMap.growAndReset(index);

	return handle;
}

bool PxgAABBManager::destroyAggregate(BoundsIndex& index_, FilterGroup::Enum& group_, AggregateHandle aggregateHandle)
{
	PxgAggregate& aggregate = mAggregates[aggregateHandle];

#if PX_CHECKED
	if (aggregate.size > 0)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "AABBManager::destroyAggregate - aggregate still has bounds that needs removed\n");
#endif

	const BoundsIndex index = aggregate.mIndex;
	//removeAggregateFromDirtyArray(aggregate, mDirtyAggregates);

	if (mAddedHandleMap.test(index))			// PT: if object had been added this frame...
		mAddedHandleMap.reset(index);		// PT: ...then simply revert the previous operation locally (it hasn't been passed to the BP yet).
	else if (aggregate.size)	// PT: else we need to remove it from the BP if it has been added there. If there's no aggregated
		mRemovedHandleMap.set(index);		// PT: shapes then the aggregate has never been added, or already removed.

	aggregate.reset();

	mAggregatesIdPool.deferredFreeID(aggregateHandle);

//	mAggregates[aggregateHandle] = reinterpret_cast<Aggregate*>(size_t(mFirstFreeAggregate));

	// PT: TODO: shouldn't it be compared to mUsedSize?
	PX_ASSERT(index < mVolumeData.size());

	index_ = index;
	group_ = mGroups[index];

#if BP_USE_AGGREGATE_GROUP_TAIL
	releaseAggregateGroup(mGroups[index]);
#endif
	resetEntry(index);

	mPersistentStateChanged = true;

	PX_ASSERT(mNbAggregates);
	mNbAggregates--;

	return true;
}

bool PxgAABBManager::addBounds(BoundsIndex index, PxReal contactDistance, FilterGroup::Enum group, void* userData, AggregateHandle aggregateHandle, ElementType::Enum volumeType, PxU32 envID)
{
	initEntry(index, contactDistance, group, userData, volumeType);

	if (aggregateHandle == PX_INVALID_U32)
	{
		mVolumeData[index].setSingleActor();

		addBPEntry(index);

		mPersistentStateChanged = true;
		mAggregatedBoundMap.growAndReset(index);

		initEnvEntry(mEnvIDs, index, envID, mBoundsArray.size());
	}
	else
	{
#if PX_CHECKED
		if (aggregateHandle >= mAggregates.size())
			return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "AABBManager::addBounds - aggregateId out of bounds\n");
#endif
		mVolumeData[index].setAggregated(aggregateHandle);

		mPersistentStateChanged = true;		// PT: TODO: do we need this here?

		PxgAggregate& aggregate =  mAggregates[aggregateHandle];

		{
			// PT: schedule the aggregate for BP insertion here, if we just added its first shape
			if (!aggregate.size)
			{
				addBPEntry(aggregate.mIndex);

				// PT: for aggregates we retrieve the environment ID from the aggregate itself.
				initEnvEntry(mEnvIDs, aggregate.mIndex, aggregate.mEnvID, mBoundsArray.size());
			}
			else
			{
				if (!mAddedHandleMap.test(aggregate.mIndex))
					mChangedHandleMap.growAndSet(aggregate.mIndex);
			}
			// AD: we need to make sure the aggregate is being updated in the regular broadphase if we 
			// add some bounds. We could just call addBPEntry and add the same aggregate logic in the 
			// parsing. Not sure why it isn't the case..

			aggregate.updateBoundIndices[aggregate.size++] = index;

			mAggregatedBoundMap.growAndSet(index);

			if (!mDirtyAggregateBitMap.boundedTest(aggregateHandle))
			{
				mDirtyAggregateBitMap.growAndSet(aggregateHandle);
				mDirtyAggregateIndices.pushBack(aggregateHandle);
			}
		}

		// PT: for aggregates we retrieve the environment ID from the aggregate itself.
//		initEnvEntry(mEnvIDs, aggregate.mIndex, aggregate.mEnvID, mBoundsArray.size());
		initEnvEntry(mEnvIDs, index, aggregate.mEnvID, mBoundsArray.size());

		mAddedAggregatedBounds.pushBack(index);
	}

	return true;
}

bool PxgAABBManager::removeBounds(BoundsIndex index)
{
	// PT: TODO: shouldn't it be compared to mUsedSize?
	PX_ASSERT(index < mVolumeData.size());

	bool res = false;
	if(mVolumeData[index].isSingleActor())
	{
		res = removeBPEntry(index);

		mPersistentStateChanged = true;
	}
	else
	{
		PX_ASSERT(mVolumeData[index].isAggregated());

		const AggregateHandle aggregateHandle = mVolumeData[index].getAggregateOwner();
		PxgAggregate& aggregate = mAggregates[aggregateHandle];

		//find and replace with last
		PxU32 i = 0;
		while (i < aggregate.size && aggregate.updateBoundIndices[i] != index)
			++i;

		//can't find index
		if (i == aggregate.size)
			return false;

		//Copy dirty mask
		aggregate.updateBoundIndices[i] = aggregate.updateBoundIndices[--aggregate.size];
		
		// PT: remove empty aggregates, otherwise the BP will crash with empty bounds
		if (!aggregate.size)
		{
			removeBPEntry(aggregate.mIndex);
			mChangedHandleMap.boundedReset(aggregate.mIndex);
		}
		else
		{
			if (!mRemovedHandleMap.test(aggregate.mIndex))
				mChangedHandleMap.growAndSet(aggregate.mIndex);

			mChangedHandleMap.boundedReset(index);
		}
		
		// AD: about the else above - we need to make sure the broadphase is picking up when an
		// aggregate changes due to removed bounds! We cannot call removeBPEntry because we 
		// already cleared all the info, we don't know it's part of an aggregate anymore!

		if (!mDirtyAggregateBitMap.boundedTest(aggregateHandle))
		{
			mDirtyAggregateBitMap.growAndSet(aggregateHandle);
			mDirtyAggregateIndices.pushBack(aggregateHandle);
		}

		// added + removed aggregate in the same step
		if (aggregate.isNew && (aggregate.size == 0))
		{
			mDirtyAggregateBitMap.boundedReset(aggregateHandle);
			mDirtyAggregateIndices.findAndReplaceWithLast(aggregateHandle);
		}

		// AD this is not really nice. But I hope this list is small most of the time.
		// I think the opposite is not needed because remove->add would not allow you to recycle the ID.
		if (!mAddedAggregatedBounds.findAndReplaceWithLast(index))
		{
			mRemovedAggregatedBounds.pushBack(index);
		}

		// AD: need to invalidate on the CPU side as well because otherwise it will get overwritten if we change
		// any other bounds. (Due to the fact that we always update the complete bounds array)
		mBoundsArray.setBounds(PxBounds3::empty(), index);

		mPersistentStateChanged = true;	// PT: TODO: do we need this here?
	}

	mAggregatedBoundMap.reset(index);

	resetEntry(index);

	return res;
}

void PxgAABBManager::updateBPFirstPass(PxU32 /*numCpuTasks*/,
	Cm::FlushPool& /*flushPool*/,
	bool hasContactDistanceUpdated,
	PxBaseTask* /*continuation*/)
{
	mPersistentStateChanged = mPersistentStateChanged || hasContactDistanceUpdated;
	// move aggregate to device, reset found and lost pair count to zero
	gpuDmaDataUp();

	// Add
	{
		PX_PROFILE_ZONE("PxgAABBManager::updateBPFirstPass - add", mContextID);

		mAddedHandles.resetOrClear();

		const PxU32* bits = mAddedHandleMap.getWords();

		if (bits)
		{
			// PT: ### bitmap iterator pattern
			const PxU32 lastSetBit = mAddedHandleMap.findLast();
			for (PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				for (PxU32 b = bits[w]; b; b &= b - 1)
				{
					const BoundsIndex handle = PxU32(w << 5 | PxLowestSetBit(b));
					PX_ASSERT(!mVolumeData[handle].isAggregated());
					mAddedHandles.pushBack(handle);		// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}

	// Update
	{
		PX_PROFILE_ZONE("PxgAABBManager::updateBPFirstPass - update", mContextID);

		//resetOrClear(mUpdatedHandles);
		mUpdatedHandles.forceSize_Unsafe(0);


		if (!mOriginShifted)
		{
			// The GPU BP needs to know that there are updates. Either if any of the bounds have changed on CPU, or we have aggregates,
			// in which case the aggregate bounds have been updates (they always are.)
			if (mNumAggregatesSlots || mChangedHandleMap.hasAnyBitSet())
			{
				mGPUStateChanged = true;
			}
		}
		else
		{
			mOriginShifted = false;
			mPersistentStateChanged = true;

			for (PxU32 i = 0; i < mUsedSize; i++)
			{
				if (mGroups[i] == FilterGroup::eINVALID)
					continue;

				mChangedHandleMap.growAndSet(i);
			}
		}
	}

	// Remove
	{
		PX_PROFILE_ZONE("AABBManager::updateBPFirstPass - remove", mContextID);

		mRemovedHandles.resetOrClear();

		const PxU32* bits = mRemovedHandleMap.getWords();
		if (bits)
		{
			// PT: ### bitmap iterator pattern
			const PxU32 lastSetBit = mRemovedHandleMap.findLast();
			for (PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				for (PxU32 b = bits[w]; b; b &= b - 1)
				{
					const BoundsIndex handle = PxU32(w << 5 | PxLowestSetBit(b));
					PX_ASSERT(!mVolumeData[handle].isAggregated()); // AD this assert is useless because we already reset the volumedata if we removed an aggregated bounds by accident..
					mRemovedHandles.pushBack(handle);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}

	//DMA bound
	preBpUpdate_GPU();

	computeAggregateBounds();
}

// PT: previously known as AABBManager::updateAABBsAndBP
void PxgAABBManager::updateBPSecondPass(PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("PxgAABBManager::updateBPSecondPass", mContextID);

	// PT: TODO: do we need to run these threads when we origin-shifted everything before?
	//finalizeUpdate(numCpuTasks, scratchAllocator, continuation);
	// PT: code below used to be "finalizeUpdate"

	// PT: this is always zero on the GPU !!!
	//printf("%d\n", mUpdatedHandles.size());

	const bool stateChanged = mPersistentStateChanged || mBoundsArray.hasChanged();
	const bool gpuStateChanged = mGPUStateChanged;

	PX_ASSERT(mEnvIDs.size()==0 || mEnvIDs.size()==mBoundsArray.size());

	const BroadPhaseUpdateData updateData(mAddedHandles.begin(), mAddedHandles.size(),
		//mUpdatedHandles.begin(), mUpdatedHandles.size(),
		NULL, 0,	// PT: the GPU code DMAs the bitmap directly, this is always empty
		mRemovedHandles.begin(), mRemovedHandles.size(),
		mBoundsArray.begin(), mGroups.begin(), mContactDistance.begin(), mEnvIDs.begin(), mBoundsArray.size(),
		mFilters,
		stateChanged,
		gpuStateChanged);
	mPersistentStateChanged = false;
	mGPUStateChanged = false;

	// PT: TODO: figure out why we skip bounds validation for the GPU
	PX_ASSERT(updateData.isValid(true));

	const bool b = updateData.getNumCreatedHandles() || updateData.getNumRemovedHandles() || gpuStateChanged;

	// PT: TODO: investigate why "force run" was always true for the GPU
	//const bool mForceRun = true;

	if (gpuStateChanged) 
		markAggregateBoundsBitmap();

	//KS - skip broad phase if there are no updated shapes.   <=== PT: this was a lie because of mForceRun
	// PT: BP UPDATE CALL
	//if(b || updateData.getNumUpdatedHandles() || mForceRun)
	mBroadPhase.update(scratchAllocator, updateData, continuation);

	// PT: decoupling: we now pass a control bool to afterBroadPhase so that we don't need to keep stateChanged/gpuStateChanged in updateData
	const bool control = b || stateChanged;
	//afterBroadPhase(control);
	// PT: the code below used to be in a "postBpStage2" function.


	if (mNumAggregatesSlots > 0)
	{
		PX_PROFILE_ZONE("PxgAABBManager::postBPStage2", mContextID);
		PxScopedCudaLock _lock_(*mCudaContextManager);

		PxgCudaBroadPhaseSap& gpuBP = getGPUBroadPhase(mBroadPhase);
		CUstream bpStream = gpuBP.getBpStream();
		CUdeviceptr bpDescd = gpuBP.getBroadPhaseDescDevicePtr();
		CUdeviceptr aggDescd = mAggregateDescBuf.getDevicePtr();

		// process added/removed aggregated bounds
		// DMA is happening with updateDirtyAggregates
		const PxU32 numRemovedAggregatedBounds = mRemovedAggregatedBounds.size();
		const PxU32 numAddedAggregatedBounds = mAddedAggregatedBounds.size();
		if (numRemovedAggregatedBounds || numAddedAggregatedBounds)
		{

			CUdeviceptr boundsd = mRemovedAggregatedBoundsBuf.getDevicePtr();
			CUdeviceptr addedBoundsd = mAddedAggregatedBoundsBuf.getDevicePtr();

			{
				KERNEL_PARAM_TYPE kernelParams[] = { 
					CUDA_KERNEL_PARAM(bpDescd),
					CUDA_KERNEL_PARAM(aggDescd),
					CUDA_KERNEL_PARAM(boundsd),
					CUDA_KERNEL_PARAM(numRemovedAggregatedBounds),
					CUDA_KERNEL_PARAM(addedBoundsd),
					CUDA_KERNEL_PARAM(numAddedAggregatedBounds)
				};

				const PxU32 numThreadsPerBlockX = 128;
				const PxU32 numThreadsPerBlockY = 2;
				const PxU32 numBlocks = (PxMax(numRemovedAggregatedBounds, numAddedAggregatedBounds) + (numThreadsPerBlockX - 1) / numThreadsPerBlockX);

				_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_MARK_ADDED_DELETED_AGGREGATED_BOUNDS, numBlocks, 1, 1, numThreadsPerBlockX, numThreadsPerBlockY, 1, 0, EPILOG);
			}
		}
		
		//sort aggregate bounds
		{
			KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd), CUDA_KERNEL_PARAM(aggDescd) };

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlocks = PxgBPKernelBlockDim::BP_AGGREGATE_SORT / numThreadsPerWarp;
			const PxU32 numBlocks = (mNumAggregatesSlots + numWarpsPerBlocks - 1) / numWarpsPerBlocks;

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_SORT_UPDATE_PROJECTIONS, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);
		}

		{
			//process self collision
			KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd), CUDA_KERNEL_PARAM(aggDescd) };

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlocks = 16;
			const PxU32 numBlocks = (mNumAggregatesSlots + numWarpsPerBlocks - 1) / numWarpsPerBlocks;

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_SELF_COLLISION, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);

#if GPU_AABB_DEBUG
			//CUresult res = mCudaContext->memcpyDtoH((void*)&mAggregateDesc, aggDescd, sizeof(PxgAggregateDesc));
			//PX_ASSERT(res == CUDA_SUCCESS);

		//	int bob = 0;
			//PX_UNUSED(bob);
#endif
		}

		//create persistent pairs
		// PT: I changed this code to decouple from updateData, ultimately to drop getStateChanged()/getGpuStateChanged(). See calling code in AABBManagerBase::updateBPSecondPass()
		if(control)
		{
			KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(aggDescd), CUDA_KERNEL_PARAM(bpDescd) };

			const PxU32 numBlocks = 64;
			const PxU32 numThreadsPerBlocks = 1024;

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_ADD_AGGPAIRS_STAGE_1, numBlocks, 1, 1, numThreadsPerBlocks, 1, 1, 0, EPILOG);
			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_ADD_AGGPAIRS_STAGE_2, numBlocks, 1, 1, numThreadsPerBlocks, 1, 1, 0, EPILOG);
		}

		//process aggregates vs actors and aggregates vs aggregates
		{
			KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd), CUDA_KERNEL_PARAM(aggDescd) };

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlocks = PxgBPKernelBlockDim::BP_AGGREGATE_SORT / numThreadsPerWarp;
			//const PxU32 numBlocks = (mNumAggregatesSlots + numWarpsPerBlocks - 1) / numWarpsPerBlocks;
			//KS - we have no idea how many agg-actor pairs we might have, so we just have to launch a large-ish grid
			const PxU32 numBlocks = 8192;

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_PAIR_COLLISION, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);

#if GPU_AABB_DEBUG
			CUresult res = mCudaContext->memcpyDtoH((void*)mAggregateDesc, aggDescd, sizeof(PxgAggregateDesc));
			PX_UNUSED(res);
			PX_ASSERT(res == CUDA_SUCCESS);
			PX_UNUSED(res);
#endif
		}

		//remove lost shape pairs in the aggregate
		{
			KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(aggDescd) };

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlocks = PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE / numThreadsPerWarp;

			const PxU32 numBlocks = PxgBPKernelGridDim::BP_AGGREGATE_REMOVE;

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_REMOVE_AGGPAIRS_STAGE_1, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);
			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_REMOVE_AGGPAIRS_STAGE_2, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);
			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_REMOVE_AGGPAIRS_STAGE_3, 1, 1, 1, numThreadsPerWarp, 1, 1, 0, EPILOG);
		}

		//copy back found and lost pair
		{
			KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(aggDescd) };

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::AGG_COPY_REPORTS, 64, 1, 1, 256, 1, 1, 0, EPILOG);

			//dma back descriptor
			mCudaContext->memcpyDtoHAsync((void*)mAggregateDesc, aggDescd, sizeof(PxgAggregateDesc), bpStream);
		}

		clearDirtyAggs();
	}
}

void PxgAABBManager::preBpUpdate_GPU()
{
	struct Local
	{
		static PX_FORCE_INLINE void dmaBitmap(PxCudaContext* ctx, CUstream bpStream, PxgCudaBuffer& dst, const PxBitMapPinned& src)
		{
			const PxU32 nbBytesToMove = sizeof(PxU32)*src.getWordCount();
 			dst.allocate(nbBytesToMove, PX_FL);
			ctx->memcpyHtoDAsync(dst.getDevicePtr(), src.getWords(), nbBytesToMove, bpStream);
		}
	};

	const bool stateChanged = mPersistentStateChanged || mBoundsArray.hasChanged();
	const bool gpuStateChanged = mGPUStateChanged;

	PxgCudaBroadPhaseSap& gpuBP = getGPUBroadPhase(mBroadPhase);

	CUstream bpStream = gpuBP.getBpStream();
	PxScopedCudaLock _lock_(*mCudaContextManager);

	if (mPersistentStateChanged)
	{
		Local::dmaBitmap(mCudaContext, bpStream, mAddedHandleBuf, mAddedHandleMap);
		Local::dmaBitmap(mCudaContext, bpStream, mRemovedHandleBuf, mRemovedHandleMap);
	}

	//KS - skip pre broad phase 
	if(stateChanged)
	{
		PX_ASSERT(mEnvIDs.size()==0 || mEnvIDs.size()==mBoundsArray.size());

		// PT: this updateData is actually only used for preBroadPhase(), which doesn't actually use all the data.
		// PT: the code below does NOT modify e.g. mGPUStateChanged so the bool doesn't need to be in updateData here.
		const BroadPhaseUpdateData updateData(mAddedHandles.begin(), mAddedHandles.size(),
			mUpdatedHandles.begin(), mUpdatedHandles.size(),
			mRemovedHandles.begin(), mRemovedHandles.size(),
			mBoundsArray.begin(), mGroups.begin(), mContactDistance.begin(), mEnvIDs.begin(), mBoundsArray.size(),
			mFilters,
			mBoundsArray.hasChanged(), // store here if there are changes in bounds not yet DMAd
			false);	// PT: this last bool not needed here

		Local::dmaBitmap(mCudaContext, bpStream, mAggregatedBoundsBuf, mAggregatedBoundMap);

		//dma update volume data
		const PxU32 boxesCapacity = updateData.getCapacity();
		mVolumDataBuf.allocate(boxesCapacity * sizeof(VolumeData), PX_FL);
		mCudaContext->memcpyHtoDAsync(mVolumDataBuf.getDevicePtr(), mVolumeData.begin(), sizeof(VolumeData)* boxesCapacity, bpStream);

		gpuBP.preBroadPhase(updateData);
	}

	if (stateChanged || gpuStateChanged)
	{
		//dma changedAABBMgrHandles to GPU.
		Local::dmaBitmap(mCudaContext, bpStream, mChangedAABBMgrHandlesBuf, mChangedHandleMap);
	}
}

void PxgAABBManager::postBroadPhase(PxBaseTask* continuation, Cm::FlushPool& /*flushPool*/)
{
	// PT: TODO: consider merging mCreatedOverlaps & mDestroyedOverlaps
	// PT: TODO: revisit memory management of mCreatedOverlaps & mDestroyedOverlaps

	//KS - if we ran broad phase, fetch the results now

	//bool updated = (mAddedHandles.size() != 0 || mUpdatedHandles.size() != 0 || mRemovedHandles.size() != 0);

	//if (updated)
	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - fetchResults", mContextID);
		mBroadPhase.fetchBroadPhaseResults();
	}

	resizeFoundAndLostPairs();

	if (continuation)
	{
		mFoundPairTask.setContinuation(continuation);
		mLostPairTask.setContinuation(continuation);
		mFoundPairTask.removeReference();
		mLostPairTask.removeReference();
	}
	else
	{
		mFoundPairTask.runInternal();
		mLostPairTask.runInternal();
	}
}

void PxgAABBManager::reallocateChangedAABBMgActorHandleMap(const PxU32 size)
{
	mChangedHandleMap.resizeAndClear(size);
	mChangedAABBMgrHandlesBuf.allocate(size * sizeof(PxU32), PX_FL);
}

void PxgAABBManager::processFoundPairs()
{
	PxgCudaBroadPhaseSap& gpuBP = getGPUBroadPhase(mBroadPhase);

	gpuBP.purgeDuplicateFoundPairs();	// PT: there is already a profile zone in it

	{
		PX_PROFILE_ZONE("PxgAABBManager::processFoundPairs - fill mCreatedOverlaps", mContextID);

		for (PxU32 i = 0; i < ElementType::eCOUNT; i++)
			mCreatedOverlaps[i].resetOrClear();

		PxU32 nbCreatePairs;
		const BroadPhasePair* createdPairs = mBroadPhase.getCreatedPairs(nbCreatePairs);

		for (PxU32 i = 0; i < nbCreatePairs; i++)
		{
			const BroadPhasePair& pair = createdPairs[i];

			PX_ASSERT(!mVolumeData[pair.mVolA].isAggregated());
			PX_ASSERT(!mVolumeData[pair.mVolB].isAggregated());

			//actor vs actor pairs
			const ElementType::Enum volumeType = PxMax(mVolumeData[pair.mVolA].getVolumeType(), mVolumeData[pair.mVolB].getVolumeType());
			mCreatedOverlaps[volumeType].pushBack(AABBOverlap(mVolumeData[pair.mVolA].getUserData(), mVolumeData[pair.mVolB].getUserData()));
		}
	}

	if (mNumAggregatesSlots > 0)
	{
		PX_PROFILE_ZONE("PxgAABBManager::processFoundPairs - process created pairs", mContextID);

		gpuBP.sortPairs(mFoundPairs);

		PxU32 id0 = 0xFFFFFFFF, id1 = 0xFFFFFFFF;
		for (PxU32 i = 0; i < mFoundPairs.size(); ++i)
		{
			const PxgBroadPhasePair& pair = mFoundPairs[i];

			void* userDataA = mVolumeData[pair.mVolA].getUserData();
			void* userDataB = mVolumeData[pair.mVolB].getUserData();

			PX_ASSERT(userDataA);
			PX_ASSERT(userDataB);

			// AD: this might not be needed anymore now.
			if(!userDataA || !userDataB)
			{
				// PT: a bit of defensive coding added for OM-74224 / PX-3571. In theory this should not be needed, as the broadphase is not
				// supposed to return null pointers here. But there seems to be an issue somewhere, most probably in the GPU BP kernels,
				// and this is an attempt at preventing a crash. We could/should remove this eventually.
				// ### DEFENSIVE
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "PxgAABBManager::processFoundPairs: found null elements!");
				continue;
			}

			if (pair.mVolA != id0 || pair.mVolB != id1)
			{
				const ElementType::Enum volumeType = PxMax(mVolumeData[pair.mVolA].getVolumeType(), mVolumeData[pair.mVolB].getVolumeType());

				mCreatedOverlaps[volumeType].pushBack(AABBOverlap(userDataA, userDataB));
				id0 = pair.mVolA;
				id1 = pair.mVolB;
			}
		}
	}

	{
		{
			PX_PROFILE_ZONE("PxgAABBManager::processFoundPairs - clear bitmaps", mContextID);
			mAddedHandleMap.clear();
			mRemovedHandleMap.clear();
		}

		{
			PX_PROFILE_ZONE("PxgAABBManager::processFoundPairs - memsetD32Async", mContextID);
			PxScopedCudaLock _lock_(*mCudaContextManager);

			CUstream bpStream = gpuBP.getBpStream();
			mCudaContext->memsetD32Async(mAddedHandleBuf.getDevicePtr(), 0, mAddedHandleBuf.getSize() / sizeof(PxU32), bpStream);
			mCudaContext->memsetD32Async(mRemovedHandleBuf.getDevicePtr(), 0, mRemovedHandleBuf.getSize() / sizeof(PxU32), bpStream);
		}
	}
}

void PxgAABBManager::processLostPairs()
{
	PxgCudaBroadPhaseSap& gpuBP = getGPUBroadPhase(mBroadPhase);

	gpuBP.purgeDuplicateLostPairs();	// PT: there is already a profile zone in it

	{
		PX_PROFILE_ZONE("PxgAABBManager::processLostPairs - fill mDestroyedOverlaps", mContextID);

		for (PxU32 i = 0; i < ElementType::eCOUNT; i++)
			mDestroyedOverlaps[i].resetOrClear();

		PxU32 nbDeletedPairs;
		const BroadPhasePair* deletedPairs = mBroadPhase.getDeletedPairs(nbDeletedPairs);

		for (PxU32 i = 0; i < nbDeletedPairs; i++)
		{
			const BroadPhasePair& pair = deletedPairs[i];

			PX_ASSERT(!mVolumeData[pair.mVolA].isAggregated());
			PX_ASSERT(!mVolumeData[pair.mVolB].isAggregated());

			//actor vs actor pairs
			void* userDataA = mVolumeData[pair.mVolA].getUserData();
			void* userDataB = mVolumeData[pair.mVolB].getUserData();
			if (userDataA && userDataB)	// PT: TODO: no idea if this is the right thing to do or if it's normal to get null ptrs here
			{
				const ElementType::Enum volumeType = PxMax(mVolumeData[pair.mVolA].getVolumeType(), mVolumeData[pair.mVolB].getVolumeType());
				//		overlaps.pushBack(AABBOverlap(volumeData[id0].userData, volumeData[id1].userData, handle));
				mDestroyedOverlaps[volumeType].pushBack(AABBOverlap(userDataA, userDataB));
			}
		}
	}

	if (mNumAggregatesSlots > 0)
	{
		PX_PROFILE_ZONE("PxgAABBManager::processLostPairs - process lost pairs", mContextID);

		gpuBP.sortPairs(mLostPairs);

		PxU32 id0 = 0xFFFFFFFF; PxU32 id1 = 0xFFFFFFFF;
		for (PxU32 i = 0; i < mLostPairs.size(); ++i)
		{
			PxgBroadPhasePair& pair = mLostPairs[i];

			// AD: a deleted pair should not generate a lost pair as per our specs.
			// so we shouldn't have null here.
			void* userDataA = mVolumeData[pair.mVolA].getUserData();
			void* userDataB = mVolumeData[pair.mVolB].getUserData();

			PX_ASSERT(userDataA);
			PX_ASSERT(userDataB);

			// AD: Added this while working on PX-3571 because I think this should never happen.
			if(!userDataA || !userDataB)
			{
				// PT: a bit of defensive coding added for OM-74224 / PX-3571. In theory this should not be needed, as the broadphase is not
				// supposed to return null pointers here. But there seems to be an issue somewhere, most probably in the GPU BP kernels,
				// and this is an attempt at preventing a crash. We could/should remove this eventually.
				// ### DEFENSIVE
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "PxgAABBManager::processLostPairs: found null elements!");
				continue;
			}

			if (pair.mVolA != id0 || pair.mVolB != id1)
			{
				const ElementType::Enum volumeType = PxMax(mVolumeData[pair.mVolA].getVolumeType(), mVolumeData[pair.mVolB].getVolumeType());
				mDestroyedOverlaps[volumeType].pushBack(AABBOverlap(mVolumeData[pair.mVolA].getUserData(), mVolumeData[pair.mVolB].getUserData()));
			
				id0 = pair.mVolA;
				id1 = pair.mVolB;
			}
		}
	}

	{
		PX_PROFILE_ZONE("PxgAABBManager::processLostPairs - clear bitmaps", mContextID);
		mRemovedHandleMap.clear();
	}
}

void PxgAABBManager::visualize(PxRenderOutput&/* out*/)
{
}

void PxgAABBManager::releaseDeferredAggregateIds()
{
	mAggregatesIdPool.processDeferredIds();

	for (PxU32 i = 0; i < mDirtyAggregateIndices.size(); ++i)
	{
		PxU32 handle = mDirtyAggregateIndices[i];
		mAggregates[handle].isNew = false;
	}

	mDirtyAggregateIndices.forceSize_Unsafe(0);
	mDirtyAggregateBitMap.clear();	
	mDirtyAggregates.forceSize_Unsafe(0);
	mRemovedAggregatedBounds.forceSize_Unsafe(0);
	mAddedAggregatedBounds.forceSize_Unsafe(0);
}

void PxgAABBManager::updateDescriptor(CUstream bpStream)
{
	//create descriptor 
	mAggregateDesc->aggregates = reinterpret_cast<PxgAggregate*>(mAggregateBuf.getDevicePtr());
	mAggregateDesc->numAgregates = mNumAggregatesSlots;
	mAggregateDesc->foundPairReport = reinterpret_cast<PxgBroadPhasePair*>(mFoundPairsBuf.getDevicePtr());
	mAggregateDesc->lostPairReport = reinterpret_cast<PxgBroadPhasePair*>(mLostPairsBuf.getDevicePtr());
	mAggregateDesc->foundPairReportMap = reinterpret_cast<PxgBroadPhasePair*>(getMappedDevicePtr(mCudaContext, mFoundPairs.begin()));
	mAggregateDesc->lostPairReportMap = reinterpret_cast<PxgBroadPhasePair*>(getMappedDevicePtr(mCudaContext, mLostPairs.begin()));
	mAggregateDesc->sharedFoundPairIndex = 0;
	mAggregateDesc->sharedLostPairIndex = 0;
	mAggregateDesc->max_found_lost_pairs = mMaxFoundLostPairs;
	mAggregateDesc->max_agg_pairs = mMaxAggPairs;
	mAggregateDesc->found_pairs_overflow_flags = false;
	mAggregateDesc->lost_pairs_overflow_flags = false;
	mAggregateDesc->agg_pairs_overflow_flags = false;
	mAggregateDesc->freeBufferList = reinterpret_cast<PxgFreeBufferList*>(mFreeIDPool.getDevicePtr());
	mAggregateDesc->freeIndices = reinterpret_cast<PxU32*>(mFreeIDs.getDevicePtr());
	mAggregateDesc->removeBitmap = reinterpret_cast<PxU32*>(mRemoveBitmap.getDevicePtr());
	mAggregateDesc->removeHistogram = reinterpret_cast<PxU32*>(mRemoveHistogram.getDevicePtr());
	mAggregateDesc->nbRemoved = 0;

	mAggregateDesc->aggPairs = reinterpret_cast<PxgAggregatePair*>(mAggPairBuf.getDevicePtr());
	mAggregateDesc->aggPairCount = reinterpret_cast<PxU32*>(mNumAggPairBuf.getDevicePtr());

	mAggregateDesc->aggPairOverflowCount = 0;
	mAggregateDesc->foundCandidatePairOverflowCount = 0;

	mCudaContext->memcpyHtoDAsync(mAggregateDescBuf.getDevicePtr(), mAggregateDesc, sizeof(PxgAggregateDesc), bpStream);
}

void PxgAABBManager::gpuDmaDataUp()
{
	CUstream bpStream = getGPUBroadPhase(mBroadPhase).getBpStream();
	PxScopedCudaLock _lock_(*mCudaContextManager);

	const PxU32 numDirtyAggregates = mDirtyAggregateIndices.size();

	if (numDirtyAggregates)
	{
		const PxU64 oldCapacity = mAggregateBuf.getSize();

		//calculate the size of aggregate
		mAggregateBuf.allocateCopyOldDataAsync(mNumAggregatesSlots * sizeof(PxgAggregate), mCudaContext, bpStream, PX_FL);
		mDirtyAggregateIndiceBuf.allocate(numDirtyAggregates * sizeof(AggregateHandle), PX_FL);
		mDirtyAggregateBuf.allocate(numDirtyAggregates * sizeof(PxgAggregate), PX_FL);

		const PxU32 numRemovedAggregatedBounds = mRemovedAggregatedBounds.size();
		const PxU32 numAddedAggregatedBounds = mAddedAggregatedBounds.size();

		mRemovedAggregatedBoundsBuf.allocate(numRemovedAggregatedBounds * sizeof(PxU32), PX_FL);
		mAddedAggregatedBoundsBuf.allocate(numAddedAggregatedBounds * sizeof(PxU32), PX_FL);

		if (oldCapacity < mAggregateBuf.getSize())
		{
			mCudaContext->memsetD32Async(mAggregateBuf.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mAggregateBuf.getSize() - oldCapacity) / sizeof(PxU32), bpStream);
		}
		
		mDirtyAggregates.reserve(numDirtyAggregates);
		mDirtyAggregates.forceSize_Unsafe(numDirtyAggregates);

		PxU32 totalNumBounds = 0;

		for (PxU32 i = 0; i < numDirtyAggregates; ++i)
		{
			PxU32 gpuRemapIndex = mDirtyAggregateIndices[i];

			PxgAggregate& agg = mAggregates[gpuRemapIndex];

			PxgAggregate& dirtyAggregate = mDirtyAggregates[i];

			totalNumBounds += agg.size;

			dirtyAggregate.filterHint = agg.filterHint;
			dirtyAggregate.mIndex = agg.mIndex;
			dirtyAggregate.size = agg.size;
			dirtyAggregate.prevComparisons = agg.prevComparisons; // AD: this could be overwritten by a stale value from CPU as well.
			dirtyAggregate.prevSize = agg.prevSize; 
			dirtyAggregate.isNew = agg.isNew;

			PxgAggregateBuffer* buffer = mAggregateBufferArray[gpuRemapIndex];
			dirtyAggregate.updateBoundIndices = reinterpret_cast<PxU32*>(buffer->updateBoundIndices.getDevicePtr());

			// AD: we can only do this for new aggregates, otherwise we break the double buffering on the GPU.
			// we never really reallocate because the aggregate size is fixed, so in practive we will never have to update
			// these pointers anyway. The GPU code needs to be just as careful to avoid copying garbage if the aggregate 
			// is not new.
			if (agg.isNew)
			{
				for (PxU32 j = 0; j < 2; ++j)
				{
					dirtyAggregate.boundIndices[j] = reinterpret_cast<PxU32*>(buffer->boundIndices[j].getDevicePtr());
					dirtyAggregate.sortedProjections[j] = reinterpret_cast<PxU32*>(buffer->sortedProjections[j].getDevicePtr());
					dirtyAggregate.sortedHandles[j] = reinterpret_cast<PxU32*>(buffer->sortedHandles[j].getDevicePtr());
					dirtyAggregate.sapBox1D[j] = reinterpret_cast<PxgSapBox1D*>(buffer->sapBox1D[j].getDevicePtr());
					dirtyAggregate.startMasks[j] = reinterpret_cast<PxU32*>(buffer->startMasks[j].getDevicePtr());
					dirtyAggregate.comparisons[j] = reinterpret_cast<PxU32*>(buffer->comparisons[j].getDevicePtr());
				}
			}
		}

		CUdeviceptr aggregated = mAggregateBuf.getDevicePtr();
		CUdeviceptr dirtyAggregateIndiced = mDirtyAggregateIndiceBuf.getDevicePtr();
		CUdeviceptr dirtyAggregated = mDirtyAggregateBuf.getDevicePtr();

		mCudaContext->memcpyHtoDAsync(dirtyAggregateIndiced, mDirtyAggregateIndices.begin(), sizeof(AggregateHandle)* numDirtyAggregates, bpStream);
		mCudaContext->memcpyHtoDAsync(dirtyAggregated, mDirtyAggregates.begin(), sizeof(PxgAggregate) * numDirtyAggregates, bpStream);

		mDirtyBoundStartIndices.reserve(numDirtyAggregates);
		mDirtyBoundStartIndices.forceSize_Unsafe(numDirtyAggregates);
		mDirtyBoundIndices.reserve(totalNumBounds);
		mDirtyBoundIndices.forceSize_Unsafe(totalNumBounds);
		PxU32 offset = 0;
		for (PxU32 i = 0; i < numDirtyAggregates; ++i)
		{
			PxU32 gpuRemapIndex = mDirtyAggregateIndices[i];

			PxgAggregate& agg = mAggregates[gpuRemapIndex];

			mDirtyBoundStartIndices[i] = offset;
			for (PxU32 j = 0; j < agg.size; ++j)
			{
				mDirtyBoundIndices[j + offset] = agg.updateBoundIndices[j];
			}

			offset += agg.size;
		}

		mDirtyBoundIndicesBuf.allocate(sizeof(PxU32) * totalNumBounds, PX_FL);
		mDirtyBoundStartIndicesBuf.allocate(sizeof(PxU32) * numDirtyAggregates, PX_FL);

		CUdeviceptr dirtyBoundIndicesd = mDirtyBoundIndicesBuf.getDevicePtr();
		CUdeviceptr dirtyBoundStartIndicesd = mDirtyBoundStartIndicesBuf.getDevicePtr();
		
		mCudaContext->memcpyHtoDAsync(dirtyBoundIndicesd, mDirtyBoundIndices.begin(), sizeof(PxU32)* totalNumBounds, bpStream);
		mCudaContext->memcpyHtoDAsync(dirtyBoundStartIndicesd, mDirtyBoundStartIndices.begin(), sizeof(PxU32) * numDirtyAggregates, bpStream);

		// AD: copy added/removed aggregated bounds to device here. The actual update kernel
		// is running just before we process aggregates, because we also need the broadphase
		// descriptor to be ready.
		if (numRemovedAggregatedBounds)
		{
			CUdeviceptr removedBoundsd = mRemovedAggregatedBoundsBuf.getDevicePtr();
			mCudaContext->memcpyHtoDAsync(removedBoundsd, mRemovedAggregatedBounds.begin(), numRemovedAggregatedBounds * sizeof(PxU32), bpStream);
		}

		if (numAddedAggregatedBounds)
		{
			CUdeviceptr addedBoundsd = mAddedAggregatedBoundsBuf.getDevicePtr();
			mCudaContext->memcpyHtoDAsync(addedBoundsd, mAddedAggregatedBounds.begin(), numAddedAggregatedBounds * sizeof(PxU32), bpStream);
		}

		//copy cpu data to gpu
		KERNEL_PARAM_TYPE kernelParams[] = {
			CUDA_KERNEL_PARAM(aggregated),
			CUDA_KERNEL_PARAM(mNumAggregatesSlots),
			CUDA_KERNEL_PARAM(dirtyAggregateIndiced),
			CUDA_KERNEL_PARAM(dirtyAggregated),
			CUDA_KERNEL_PARAM(numDirtyAggregates),
			CUDA_KERNEL_PARAM(dirtyBoundIndicesd),
			CUDA_KERNEL_PARAM(dirtyBoundStartIndicesd)
		};

		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlocks = 16;
		const PxU32 numBlocks = (numDirtyAggregates + numWarpsPerBlocks - 1) / numWarpsPerBlocks;

		_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::UPDATE_DIRTY_AGGREGATE, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);
	}
	
	//reset found and lost pair count to zero
	updateDescriptor(bpStream);
}

void PxgAABBManager::clearDirtyAggs()
{
	CUstream bpStream = getGPUBroadPhase(mBroadPhase).getBpStream();
	PxScopedCudaLock _lock_(*mCudaContextManager);

	const PxU32 numDirtyAggregates = mDirtyAggregateIndices.size();
	if (numDirtyAggregates)
	{
		CUdeviceptr aggregated = mAggregateBuf.getDevicePtr();
		CUdeviceptr dirtyAggregateIndiced = mDirtyAggregateIndiceBuf.getDevicePtr();
		
		KERNEL_PARAM_TYPE kernelParams[] = {
			CUDA_KERNEL_PARAM(aggregated),
			CUDA_KERNEL_PARAM(mNumAggregatesSlots),
			CUDA_KERNEL_PARAM(dirtyAggregateIndiced),
			CUDA_KERNEL_PARAM(numDirtyAggregates),
		};

		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlocks = 16;
		const PxU32 numBlocks = (numDirtyAggregates + numWarpsPerBlocks - 1) / numWarpsPerBlocks;

		_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::CLEAR_DIRTY_AGGS, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);
	}
}

void PxgAABBManager::resizeFoundAndLostPairs()
{
	PxU32 sharedFoundPairIndex = mAggregateDesc->sharedFoundPairIndex;
	PxU32 sharedLostPairIndex = mAggregateDesc->sharedLostPairIndex;

	// update Simstats.
	PxU32 maxAggPairsNeeded = PxMax(sharedLostPairIndex, PxMax(sharedFoundPairIndex, mAggregateDesc->foundCandidatePairOverflowCount));
#if PX_ENABLE_SIM_STATS
	mGpuDynamicsLostFoundAggregatePairsStats = PxMax(maxAggPairsNeeded, mGpuDynamicsLostFoundAggregatePairsStats);
	mGpuDynamicsTotalAggregatePairsStats = PxMax(mAggregateDesc->aggPairOverflowCount, mGpuDynamicsTotalAggregatePairsStats);
	mGpuDynamicsLostFoundPairsStats = getGPUBroadPhase(mBroadPhase).getFoundLostPairsStats(); // max is already done in broadphase.
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	if (mAggregateDesc->found_pairs_overflow_flags)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
			"The application needs to increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity to %i, otherwise, the simulation will miss interactions", maxAggPairsNeeded);
		
		// AD: these can be lower than the max, because it can happen that the overflow flag is set because of the candidate pairs overflowing.
		// We can end up with far fewer pairs in the end if afterwards the detailed collisions don't return any overlaps. So we only correct
		// the number here if we actually overflow the final count, because otherwise we will process more pairs than we have!
		sharedFoundPairIndex = PxMin(mAggregateDesc->sharedFoundPairIndex, mMaxFoundLostPairs);
	}

	if (mAggregateDesc->lost_pairs_overflow_flags)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
			"The application needs to increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity buffers to %i, otherwise, the simulation will miss interactions", maxAggPairsNeeded);
		sharedLostPairIndex = mMaxFoundLostPairs;
	}

	if (mAggregateDesc->agg_pairs_overflow_flags)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
			"The application needs to increase PxGpuDynamicsMemoryConfig::totalAggregatePairsCapacity to %i , otherwise, the simulation will miss interactions\n", mAggregateDesc->aggPairOverflowCount);
	}

	mFoundPairs.forceSize_Unsafe(sharedFoundPairIndex);
	mLostPairs.forceSize_Unsafe(sharedLostPairIndex);

	// AD: safety for abort mode.
	if (mCudaContext->isInAbortMode())
	{
		mFoundPairs.forceSize_Unsafe(0);
		mLostPairs.forceSize_Unsafe(0);
		mAggregateDesc->sharedFoundPairIndex = 0;
		mAggregateDesc->sharedLostPairIndex = 0;
	}
}

void PxgAABBManager::computeAggregateBounds()
{
	if (mNumAggregatesSlots > 0)
	{
		PxgCudaBroadPhaseSap& gpuBP = getGPUBroadPhase(mBroadPhase);

		CUstream bpStream = gpuBP.getBpStream();
		PxScopedCudaLock _lock_(*mCudaContextManager);

		CUdeviceptr aggDescd = mAggregateDescBuf.getDevicePtr();

		{
			CUdeviceptr boundsd = gpuBP.getBoundsBuffer().getDevicePtr();
			CUdeviceptr contactDistd = gpuBP.getContactDistBuffer().getDevicePtr();
			//copy cpu data to gpu
			KERNEL_PARAM_TYPE kernelParams[] = {
				CUDA_KERNEL_PARAM(aggDescd),
				CUDA_KERNEL_PARAM(boundsd),
				CUDA_KERNEL_PARAM(contactDistd)
			};

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlocks = 16;
			const PxU32 numBlocks = (mNumAggregatesSlots + numWarpsPerBlocks - 1) / numWarpsPerBlocks;

			_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::UPDATE_AGGREGATE_BOUND, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlocks, 1, 0, EPILOG);
		}
	}
}

void PxgAABBManager::markAggregateBoundsBitmap()
{
	if(mNumAggregatesSlots > 0)
	{
		PxScopedCudaLock _lock_(*mCudaContextManager);

		PxgCudaBroadPhaseSap& gpuBP = getGPUBroadPhase(mBroadPhase);
		CUstream bpStream = gpuBP.getBpStream();

		CUdeviceptr aggDescd = mAggregateDescBuf.getDevicePtr();
		CUdeviceptr changedHandles = mChangedAABBMgrHandlesBuf.getDevicePtr();

		// in the first step, this array does not exist yet, but by definition there are also 
		// no changed handles. Everything is new.
		if (!changedHandles)
			return;
		
		KERNEL_PARAM_TYPE kernelParams[] = {
			CUDA_KERNEL_PARAM(aggDescd),
			CUDA_KERNEL_PARAM(changedHandles)
		};

		const PxU32 nbWarpsPerBlock = 8;
		const PxU32 nbBlocks = (mNumAggregatesSlots + nbWarpsPerBlock-1) / nbWarpsPerBlock;

		_launch<GPU_AABB_DEBUG>(PROLOG, PxgKernelIds::MARK_AGGREGATE_BOUND_BITMAP, nbBlocks, 1, 1, 32, nbWarpsPerBlock, 1, 0, EPILOG);
	}

}

void PxgProcessFoundPairTask::runInternal()
{
	mManager->processFoundPairs();
}

void PxgProcessLostPairTask::runInternal()
{
	mManager->processLostPairs();
}


