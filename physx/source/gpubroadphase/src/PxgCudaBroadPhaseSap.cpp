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

#include "foundation/PxAllocator.h"
#include "foundation/PxTime.h"
#include "foundation/PxMemory.h"
#include "foundation/PxSort.h"

#include "common/PxProfileZone.h"
#include "PxvSimStats.h"

#include "PxgCudaBroadPhaseSap.h"

#include "PxgBroadPhaseKernelIndices.h"
#include "PxgIntegerAABB.h"
#include "PxgBroadPhasePairReport.h"
#include "BpBroadPhaseUpdate.h"
#include "PxgSapBox1D.h"
#include "PxgRadixSortDesc.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxSceneDesc.h"
#include "PxgCudaUtils.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgAABBManager.h"

#include "PxgContext.h"
#include "PxgSimulationCore.h"

#include "CudaKernelWrangler.h"
#include "cudamanager/PxCudaContext.h"

#include "cudamanager/PxCudaContextManager.h"

#include "PxgKernelLauncher.h"

// PT: TODO:
// - most of these functions don't need to be member functions

#define GPU_BP_DEBUG 0
#define USE_NEW_LAUNCH_FUNCTION 1

#if GPU_BP_DEBUG
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
	#define EPILOG				mStream, kernelParams, PX_FL
#else
	#define KERNEL_PARAM_TYPE	PxCudaKernelParam
	#define CUDA_KERNEL_PARAM	PX_CUDA_KERNEL_PARAM
	#define EPILOG				mStream, kernelParams, sizeof(kernelParams), PX_FL
#endif

using namespace physx;

PX_IMPLEMENT_OUTPUT_ERROR

PxgCudaBroadPhaseSap::PxgCudaBroadPhaseSap(const PxGpuBroadPhaseDesc& desc, PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, const PxGpuDynamicsMemoryConfig& init, PxgHeapMemoryAllocatorManager* heapMemoryManager, PxU64 contextID) :
	Bp::BroadPhase						(),

	mContextID							(contextID),

	mDesc								(desc),

	mNumOfBoxes							(0),
	mUpdateData_CreatedHandleSize		(0),
	mUpdateData_RemovedHandleSize		(0),
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
	mUpdateData_UpdatedHandleSize		(0),
#endif
	mUpdateData_BoxesCapacity			(0),

	mGpuKernelWranglerManager			(gpuKernelWrangler),
	mCudaContextManager					(cudaContextManager),
	mCudaContext						(cudaContextManager->getCudaContext()),
	mHeapMemoryManager					(heapMemoryManager),
	mCreatedHandlesBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRemovedHandlesBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
	// PT: looks like this stuff used to be here but got removed for some reason!
	mUpdatedHandlesBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
#endif
	mBoxFpBoundsBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxContactDistancesBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxGroupsBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxEnvIDsBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mNewIntegerBoundsBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mOldIntegerBoundsBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxPtProjectionsBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxProjectionRanksBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxPtHandlesBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mTempBoxPtProjectionBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mTempBoxPtHandlesBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRadixCountBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBoxSapBox1DBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mNewBoxSapBox1DBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mEndPtHistogramBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBlockEndPtHistogramBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mEndPtHandleBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mStartPtHistogramBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBlockStartPtHistogramBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mStartPtHandleBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mTotalEndPtHistogramBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBlockTotalEndPtHistogramBuf		(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mActiveRegionTotalBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mStartRegionsTotalBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mOrderedActiveRegionHandlesTotalBuf	(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mOrderedStartRegionHandlesTotalBuf	(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mOverlapChecksRegionBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBlockOverlapChecksRegionBuf		(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mOverlapChecksHandleRegionBuf		(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mIncrementalComparisons				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mIncrementalBlockComparisons		(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mAggregateReportBlockBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mActorReportBlockBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRegionRangeBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mStartRegionAccumBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBlockStartRegionAccumBuf			(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRegionAccumBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBlockRegionAccumBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mFoundPairsBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mLostPairsBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mFoundAggregateBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mLostAggregateBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mFoundActorBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mLostActorBuf						(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mBPDescBuf							(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRadixSortDescBuf					(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mRadixSortWORDescBuf				(heapMemoryManager, PxsHeapStats::eBROADPHASE),
	mPinnedEvent						(NULL),
	mBpDesc								(NULL),
	mRSDesc								(NULL),
	mRSDescWOR							(NULL),
	mFoundActorPairs					(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators, PxsHeapStats::eBROADPHASE)),
	mLostActorPairs						(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators, PxsHeapStats::eBROADPHASE)),
	mMaxFoundLostPairs					(init.foundLostPairsCapacity),
	mMaxAggFoundLostPairs				(init.foundLostAggregatePairsCapacity),
	mAABBManager						(NULL),
#if PX_ENABLE_SIM_STATS
	mFoundLostPairsStats				(0),
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	mForceUpdate						(true)
{
	PxScopedCudaLock _lock_(*mCudaContextManager);

	for(PxU32 i = 0; i < 3; ++i)
		mRadixCountBuf[i].allocate(sizeof(PxU32) * PxgRadixSortKernelGridDim::RADIX_SORT * 16, PX_FL);
	
	mBPDescBuf.allocate(sizeof(PxgBroadPhaseDesc), PX_FL);

	mRadixSortDescBuf.allocate(sizeof(PxgRadixSortDesc)*6, PX_FL);
	mRadixSortWORDescBuf.allocate(sizeof(PxgRadixSortDesc)*6, PX_FL);

	mBpDesc = reinterpret_cast<PxgBroadPhaseDesc*>(mHeapMemoryManager->mMappedMemoryAllocators->allocate(sizeof(PxgBroadPhaseDesc), PxsHeapStats::eBROADPHASE, PX_FL));
	mRSDesc = reinterpret_cast<PxgRadixSortDesc*>(mHeapMemoryManager->mMappedMemoryAllocators->allocate(sizeof(PxgRadixSortDesc) * 6, PxsHeapStats::eBROADPHASE, PX_FL));
	mRSDescWOR = reinterpret_cast<PxgRadixSortDesc*>(mHeapMemoryManager->mMappedMemoryAllocators->allocate(sizeof(PxgRadixSortDesc) * 6, PxsHeapStats::eBROADPHASE, PX_FL));
	
	mRegionAccumTotal = 0;
	mOverlapChecksTotalRegion = 0;
	mStartRegionAccumTotal = 0;

	mFoundPairsBuf.allocate(mMaxFoundLostPairs * sizeof(PxgBroadPhasePair), PX_FL);
	mLostPairsBuf.allocate(mMaxFoundLostPairs * sizeof(PxgBroadPhasePair), PX_FL);

	mFoundAggregateBuf.allocate(mMaxAggFoundLostPairs * sizeof(PxgBroadPhasePair), PX_FL);
	mLostAggregateBuf.allocate(mMaxAggFoundLostPairs * sizeof(PxgBroadPhasePair), PX_FL);

	mFoundActorBuf.allocate(mMaxFoundLostPairs * sizeof(PxgBroadPhasePair), PX_FL);
	mLostActorBuf.allocate(mMaxFoundLostPairs * sizeof(PxgBroadPhasePair), PX_FL);

	mFoundActorPairs.forceSize_Unsafe(0);
	mFoundActorPairs.reserve(mMaxFoundLostPairs);

	mLostActorPairs.forceSize_Unsafe(0);
	mLostActorPairs.reserve(mMaxFoundLostPairs);

	createGpuStreamsAndEvents();
}

PxgCudaBroadPhaseSap::~PxgCudaBroadPhaseSap()
{
	PxScopedCudaLock _lock_(*mCudaContextManager);
		
	mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mBpDesc);
	mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mRSDesc);
	mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mRSDescWOR);

	releaseGpuStreamsAndEvents();
}

void PxgCudaBroadPhaseSap::release()
{
	this->~PxgCudaBroadPhaseSap();
	PX_FREE_THIS;
}

void PxgCudaBroadPhaseSap::createGpuStreamsAndEvents()
{
	int leastPriority, mostPriority;
	cuCtxGetStreamPriorityRange(&leastPriority, &mostPriority);

	CUresult result = mCudaContext->streamCreateWithPriority(&mStream, CU_STREAM_NON_BLOCKING, mostPriority);

	if (result != CUDA_SUCCESS)
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "GPU Create Stream 0 fail!!\n");

	result = mCudaContext->eventCreate(&mEvent, CU_EVENT_DISABLE_TIMING);

	mPinnedEvent = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);

	if (result != CUDA_SUCCESS)
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "GPU Create Event 0 fail!!\n");
}

void PxgCudaBroadPhaseSap::releaseGpuStreamsAndEvents()
{
	//destroy stream
	mCudaContext->streamDestroy(mStream);
	mStream = NULL;

	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mPinnedEvent);

	//destroy event
	mCudaContext->eventDestroy(mEvent);
	mEvent = NULL;
}

void PxgCudaBroadPhaseSap::gpuDMAUp(const Bp::BroadPhaseUpdateData& updateData, PxgBroadPhaseDesc& bpDesc, PxgRadixSortDesc* rsDescs)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.gpuDMAUp", mContextID);

	//mCudaContext->memcpyHtoDAsync(constraintsPerPartitiond, constraintsPerPartitionIter.begin(), sizeof(PxU32) * numConstraintsPerPartition, mStream);

	mUpdateData_RemovedHandleSize = updateData.getNumRemovedHandles();
	mUpdateData_CreatedHandleSize = updateData.getNumCreatedHandles();
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
	mUpdateData_UpdatedHandleSize = updateData.getNumUpdatedHandles();
#endif

	//mContactDistances = updateData.getContactDistance();
	//mBoxBoundsMinMax = updateData.getAABBs();
	//mBoxGroups = updateData.getGroups();

	mUpdateData_BoxesCapacity = updateData.getCapacity();

	mNumOfBoxes = mNumOfBoxes + mUpdateData_CreatedHandleSize - mUpdateData_RemovedHandleSize; 

	//We need to add on removedHandleSize because these handles are temporarily also in the projection buffer
	const PxU32 nbProjections = (mNumOfBoxes + mUpdateData_RemovedHandleSize) * 2; 
	const PxU32 paddedProjections = (nbProjections + 3)&(~3);

	mOldIntegerBoundsBuf.allocateCopyOldDataAsync(mUpdateData_BoxesCapacity * sizeof(PxgIntegerAABB), mCudaContext, mStream, PX_FL);

	//we need to allocate enough memory (x4) for the radix sort because each thread read 4 elements
		
	for (PxU32 i = 0; i < 3; ++i)
	{
		mBoxSapBox1DBuf[i].allocateCopyOldDataAsync(mUpdateData_BoxesCapacity * sizeof(PxgSapBox1D), mCudaContext, mStream, PX_FL);
		mNewBoxSapBox1DBuf[i].allocate(mUpdateData_BoxesCapacity * sizeof(PxgSapBox1D), PX_FL);

		mBoxPtProjectionsBuf[i].allocateCopyOldDataAsync(paddedProjections * sizeof(int), mCudaContext, mStream, PX_FL);
		
		mBoxProjectionRanksBuf[i].allocate(paddedProjections * sizeof(int), PX_FL);
		mTempBoxPtProjectionBuf[i].allocate(paddedProjections * sizeof(int), PX_FL);
		mTempBoxPtHandlesBuf[i].allocate(paddedProjections * sizeof(int), PX_FL);
			
		for (PxU32 j = 0; j < 2; ++j)
		{
			//mEndPtHistogramBuf[j][i].allocateCopyOldDataAsync(nbProjections * sizeof(int), mStreams.begin(), mHeapMemoryManager);
			//mBlockEndPtHistogramBuf[j][i].allocateCopyOldDataAsync(PxgBPKernelGridDim::BP_OUTPUT_ENDPT_HISTOGRAM * sizeof(int), mStreams.begin(), mHeapMemoryManager); // 32 block
			//mEndPtHandleBuf[j][i].allocateCopyOldDataAsync(mNumOfBoxes*sizeof(int), mStreams.begin(), mHeapMemoryManager);

			//mStartPtHistogramBuf[j][i].allocateCopyOldDataAsync(nbProjections * sizeof(int), mStreams.begin(), mHeapMemoryManager);
			//mBlockStartPtHistogramBuf[j][i].allocateCopyOldDataAsync(PxgBPKernelGridDim::BP_OUTPUT_ENDPT_HISTOGRAM * sizeof(int), mStreams.begin(), mHeapMemoryManager); // 32 block
			//mStartPtHandleBuf[j][i].allocateCopyOldDataAsync(mNumOfBoxes*sizeof(int), mStreams.begin(), mHeapMemoryManager);

			const PxU32 index = j * 3 + i;

			mBoxPtHandlesBuf[index].allocateCopyOldDataAsync(paddedProjections * sizeof(int), mCudaContext, mStream, PX_FL);

			mEndPtHistogramBuf[index].allocateCopyOldDataAsync(nbProjections * sizeof(int), mCudaContext, mStream, PX_FL);
			mBlockEndPtHistogramBuf[index].allocateCopyOldDataAsync(PxgBPKernelGridDim::BP_OUTPUT_ENDPT_HISTOGRAM * sizeof(int), mCudaContext, mStream, PX_FL); // 32 block
			mEndPtHandleBuf[index].allocateCopyOldDataAsync(mNumOfBoxes*sizeof(int), mCudaContext, mStream, PX_FL);

			mStartPtHistogramBuf[index].allocateCopyOldDataAsync(nbProjections * sizeof(int), mCudaContext, mStream, PX_FL);
			mBlockStartPtHistogramBuf[index].allocateCopyOldDataAsync(PxgBPKernelGridDim::BP_OUTPUT_ENDPT_HISTOGRAM * sizeof(int), mCudaContext, mStream, PX_FL); // 32 block
			mStartPtHandleBuf[index].allocateCopyOldDataAsync(mNumOfBoxes*sizeof(int), mCudaContext, mStream, PX_FL);
		}

		mIncrementalComparisons[i].allocate(nbProjections* sizeof(int), PX_FL);
		mIncrementalBlockComparisons[i].allocate(PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1 * sizeof(int), PX_FL);

		mTotalEndPtHistogramBuf[i].allocate(nbProjections*sizeof(PxU32), PX_FL);
		mBlockTotalEndPtHistogramBuf[i].allocate(PxgBPKernelGridDim::BP_OUTPUT_ENDPT_HISTOGRAM*sizeof(PxU32), PX_FL);
	}

	for (PxU32 i = 0; i < 2; ++i)
	{
		mAggregateReportBlockBuf[i].allocate(32 * sizeof(PxU32), PX_FL);
		mActorReportBlockBuf[i].allocate(32 * sizeof(PxU32), PX_FL);
	}

	//each thread read 4 elements so we need to allocate enough memory for it
	const PxU32 totalNbProjectionRegions = (nbProjections * 64 + 3)&(~3);

	mActiveRegionTotalBuf.allocate(totalNbProjectionRegions * sizeof(int), PX_FL);
	mStartRegionsTotalBuf.allocate(totalNbProjectionRegions * sizeof(int), PX_FL);
	mOrderedActiveRegionHandlesTotalBuf.allocate(totalNbProjectionRegions * sizeof(int), PX_FL);
	mOrderedStartRegionHandlesTotalBuf.allocate(totalNbProjectionRegions * sizeof(int), PX_FL);

	mOverlapChecksRegionBuf.allocate(64 * mNumOfBoxes * sizeof(regionOverlapType), PX_FL);
	mBlockOverlapChecksRegionBuf.allocate(PxgBPKernelGridDim::BP_OUTPUT_OVERLAPCHECKS_HISTOGRAM * sizeof(regionOverlapType), PX_FL);
	mOverlapChecksHandleRegionBuf.allocate(64 * mNumOfBoxes * sizeof(PxgHandleRegion), PX_FL);
	mRegionRangeBuf.allocate(mUpdateData_BoxesCapacity * sizeof(PxgIntegerRegion), PX_FL);
	mStartRegionAccumBuf.allocate(nbProjections * sizeof(int), PX_FL);
	mBlockStartRegionAccumBuf.allocate(PxgBPKernelGridDim::BP_OUTPUT_START_REGION_HISTOGRAM * sizeof(int), PX_FL);

	mRegionAccumBuf.allocate(nbProjections * sizeof(int), PX_FL);
	mBlockRegionAccumBuf.allocate(PxgBPKernelGridDim::BP_OUTPUT_REGION_HISTOGRAM * sizeof(int), PX_FL);

	//allocate enough memory for GPU. All this data is input from the AABB manager this frame
	mCreatedHandlesBuf.allocate(mUpdateData_CreatedHandleSize * sizeof(PxU32), PX_FL);
	mRemovedHandlesBuf.allocate(mUpdateData_RemovedHandleSize * sizeof(PxU32), PX_FL);
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
	mUpdatedHandlesBuf.allocate(mUpdateData_UpdatedHandleSize * sizeof(PxU32), PX_FL);
#endif

	mNewIntegerBoundsBuf.allocate(mUpdateData_BoxesCapacity * sizeof(PxgIntegerAABB), PX_FL);
		
	//mBoxFpBoundsBuf and mBoxContactDistancesBuf need to be allocated before particle updateBound kernel
	//we move the allocation to gpuDmaUpSharedData. Particle system don't need mBoxGroupsBuf. However, if
	//we dma up those three buffers based on the state changed so it will make sense to group those buffer together
	//mBoxFpBoundsBuf.allocate(mBoxesCapacity * sizeof(PxBounds3));
	//mBoxContactDistancesBuf.allocate(mBoxesCapacity * sizeof(PxReal));
	//mBoxGroupsBuf.allocate(mBoxesCapacity * sizeof(PxU32));

	updateDescriptor(bpDesc);
	updateRadixSortDesc(rsDescs);

	//DMA the update data to GPU
	mCudaContext->memcpyHtoDAsync(mCreatedHandlesBuf.getDevicePtr(), updateData.getCreatedHandles(), sizeof(int) * mUpdateData_CreatedHandleSize, mStream);
	mCudaContext->memcpyHtoDAsync(mRemovedHandlesBuf.getDevicePtr(), updateData.getRemovedHandles(), sizeof(int) * mUpdateData_RemovedHandleSize, mStream);
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
	mCudaContext->memcpyHtoDAsync(mUpdatedHandlesBuf.getDevicePtr(), updateData.getUpdatedHandles(), sizeof(int) * mUpdateData_UpdatedHandleSize, mStream);
#endif
		
	/*if(updateData.getStateChanged())
	{
		mCudaContext->memcpyHtoDAsync(mBoxContactDistancesBuf.getDevicePtr(), mContactDistances, sizeof(PxReal)* mBoxesCapacity, mStream);
		mCudaContext->memcpyHtoDAsync(mBoxGroupsBuf.getDevicePtr(), mBoxGroups, sizeof(PxU32)* mBoxesCapacity, mStream);
		mCudaContext->memcpyHtoDAsync(mBoxFpBoundsBuf.getDevicePtr(), mBoxBoundsMinMax, sizeof(PxBounds3)* mBoxesCapacity, mStream);			
	}*/
		
	mCudaContext->memcpyHtoDAsync(mBPDescBuf.getDevicePtr(), (void*)&bpDesc, sizeof(PxgBroadPhaseDesc), mStream);

	mCudaContext->memcpyHtoDAsync(mRadixSortDescBuf.getDevicePtr(), rsDescs, sizeof(PxgRadixSortDesc)*6, mStream);
	mCudaContext->memcpyHtoDAsync(mRadixSortWORDescBuf.getDevicePtr(), mRSDescWOR, sizeof(PxgRadixSortDesc) * 6, mStream);
	/*PxCudaStreamFlush(mStreams.begin());*/

#if GPU_BP_DEBUG
	GPU_DEBUG_STREAM(mStream, "GPU radix sort fail!!\n")

	mCudaContext->memcpyDtoH((void*)&bpDesc, mBPDescBuf.getDevicePtr(), sizeof(PxgBroadPhaseDesc));
#endif
}

void PxgCudaBroadPhaseSap::freeBuffers()
{
	mLostActorPairs.forceSize_Unsafe(0);
	mFoundActorPairs.forceSize_Unsafe(0);
}

void PxgCudaBroadPhaseSap::runCopyResultsKernel(PxgBroadPhaseDesc& /*desc*/)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.runCopyResultsKernel", mContextID);

	CUdeviceptr bpBuff = mBPDescBuf.getDevicePtr();
	{
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpBuff) };
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_ACCUMULATE_REPORT_STAGE_1, PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1, 4, 1, PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_ACCUMULATE_REPORT_STAGE_2, PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2, 4, 1, PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2, 1, 1, 0, EPILOG);

#if GPU_BP_DEBUG
		/*mCudaContext->memcpyDtoHAsync((void*)&desc, mBPDescBuf.getDevicePtr(), sizeof(PxgBroadPhaseDesc), mStream);
		resultR = mCudaContext->streamSynchronize(mStream);*/
#endif
	}

	{
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpBuff) };
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COPY_REPORTS, PxgBPKernelGridDim::BP_COPY_REPORTS, 1, 1, PxgBPKernelBlockDim::BP_COPY_REPORTS, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::gpuDMABack(const PxgBroadPhaseDesc& desc)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.gpuDMABack", mContextID);

	//mCudaContext->eventRecord(mEvent, mStream);

	//PxCudaStreamFlush(mStreams.begin());		//KS - dispatch work!

	{
		CUdeviceptr bpBuff = mBPDescBuf.getDevicePtr();

		mCudaContext->memcpyDtoHAsync((void*)&desc, bpBuff, sizeof(PxgBroadPhaseDesc), mStream);
		//resultR = mCudaContext->streamSynchronize(mStream);

		void* devicePtr = getMappedDevicePtr(mCudaContext, mPinnedEvent);
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(devicePtr) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_SIGNAL_COMPLETE, 1, 1, 1, 1, 1, 1, 0, EPILOG);

		mCudaContext->streamFlush(mStream);
	}

	{
		PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.Synchronize", mContextID);
		//mCudaContext->streamSynchronize(mStream);
		volatile PxU32* eventPtr = mPinnedEvent;

		if (!spinWait(*eventPtr, 0.1f))
			mCudaContext->streamSynchronize(mStream);
	}

	mOverlapChecksTotalRegion = desc.overlapChecksTotalRegion;
	mStartRegionAccumTotal = desc.startRegionAccumTotal;
	mRegionAccumTotal = desc.regionAccumTotal;

	// AD: some explanation about the counts here - just to reiterate:
	//
	// internally we have two lists, found pairs and lost pairs. Both contain actor-actor, actor-aggregate and
	// aggregate-aggregate pairs. In the "report" phase, these two lists are built such that all the pairs involving
	// aggregates are first, and actor pairs are after that.
	//
	// desc.sharedFound/LostPairIndex is the total size of the lists.
	// desc.sharedFound/LostAggPairIndex is the number of aggregates in the list.
	//
	// but it gets more complicated. These counts overflow, but internally we only write until the maxLostFoundPairs
	// value to make sure we're not going out of bounds. So the total value in the descriptor is only really useful
	// if we are below the max, otherwise we need to correct.
	//
	// so in the end, the final number of pairs is PxMin(mMaxLostFoundPairs, desc.sharedFoundPairIndex) - desc.sharedFoundAggPairIndex.
	// This works because the aggregate index is always smaller than the max index.

	PX_ASSERT(desc.sharedFoundPairIndex >= desc.sharedFoundAggPairIndex);
	PX_ASSERT(desc.sharedLostPairIndex >= desc.sharedLostAggPairIndex);

	PxU32 foundLostPairsNeeded = PxMax(desc.sharedFoundPairIndex, desc.sharedLostPairIndex);
#if PX_ENABLE_SIM_STATS
	mFoundLostPairsStats = PxMax(mFoundLostPairsStats, foundLostPairsNeeded);
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	if (desc.found_lost_pairs_overflow_flags)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
			"The application needs to increase PxGpuDynamicsMemoryConfig::foundLostPairsCapacity to %i, otherwise, the simulation will miss interactions\n", foundLostPairsNeeded);
	}

	mFoundActorPairs.forceSize_Unsafe(PxMin(mMaxFoundLostPairs, desc.sharedFoundPairIndex) - desc.sharedFoundAggPairIndex);
	mLostActorPairs.forceSize_Unsafe(PxMin(mMaxFoundLostPairs, desc.sharedLostPairIndex) - desc.sharedLostAggPairIndex);

	// AD: safety in case copyReports did not run due to abort mode
	if (mCudaContext->isInAbortMode())
	{
		mFoundActorPairs.forceSize_Unsafe(0);
		mLostActorPairs.forceSize_Unsafe(0);
	}
}

struct ReportMore
{
	bool operator()(const PxgBroadPhasePair& left, const PxgBroadPhasePair& right) const
	{
		return (left.mVolA > right.mVolA) || 
			((left.mVolA == right.mVolA) && (left.mVolB > right.mVolB));
	}
};

/*bool hasDuplicates(PxPinnedArray<PxgBroadPhasePair>& iterator)
{
	for(PxU32 a = 1; a < iterator.size(); ++a)
	{
		PX_ASSERT(iterator[a].mVolA != iterator[a-1].mVolA || 
			iterator[a].mVolB != iterator[a-1].mVolB);
	}
	return false;
}*/

void PxgCudaBroadPhaseSap::sortBuffer(PxgBroadPhasePair* PX_RESTRICT reportBuffer, const PxU32 size)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap::sortBuffer", mContextID);

#if 1
	PxSort(reportBuffer, size, ReportMore());
#else
	const PxU32 SmallBufferLimit = 512;

	if (size < SmallBufferLimit)
		PxSort(reportBuffer, size, ReportMore());
	else
	{
		//Histogram sort...
		mHistogramBuffer.forceSize_Unsafe(0);
		mHistogramBuffer.reserve(mBoxesCapacity);
		mHistogramBuffer.forceSize_Unsafe(mBoxesCapacity);

		PxMemZero(mHistogramBuffer.begin(), sizeof(PxU32) * mBoxesCapacity);

		mTempPairBuffer.forceSize_Unsafe(0);
		mTempPairBuffer.reserve(size);
		mTempPairBuffer.forceSize_Unsafe(size);

		for (PxU32 a = 0; a < size; ++a)
		{
			++mHistogramBuffer[reportBuffer[a].mVolA];
		}

		//Compute runsum
		PxU32 runsum = 0;
		PxU32 boxesCapacity = mBoxesCapacity;
		//for (PxU32 a = 0; a < mBoxesCapacity; ++a)
		while(boxesCapacity--)
		{
			PxU32 value = mHistogramBuffer[boxesCapacity];
			mHistogramBuffer[boxesCapacity] = runsum;
			runsum += value;
		}

		for (PxU32 a = 0; a < size; ++a)
		{
			PxU32 idx = mHistogramBuffer[reportBuffer[a].mVolA]++;
			mTempPairBuffer[idx] = reportBuffer[a];
		}

		PxMemZero(mHistogramBuffer.begin(), sizeof(PxU32) * mBoxesCapacity);

		for (PxU32 a = 0; a < size; ++a)
		{
			++mHistogramBuffer[reportBuffer[a].mVolB];
		}

		runsum = 0;
		boxesCapacity = mBoxesCapacity;
		//for (PxU32 a = 0; a < mBoxesCapacity; ++a)
		while(boxesCapacity--)
		{
			PxU32 value = mHistogramBuffer[boxesCapacity];
			mHistogramBuffer[boxesCapacity] = runsum;
			runsum += value;
		}

		for (PxU32 a = 0; a < size; ++a)
		{
			PxU32 idx = mHistogramBuffer[mTempPairBuffer[a].mVolB]++;
			reportBuffer[idx] = mTempPairBuffer[a];
		}
	}
#endif
}

void PxgCudaBroadPhaseSap::purgeDuplicates(PxPinnedArray<PxgBroadPhasePair>& pairs)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.purgeDuplicates", mContextID);

	const PxU32 nbPairs = pairs.size();
	sortBuffer(pairs.begin(), nbPairs);

	if (nbPairs)
	{
		PxU32 actor0 = pairs[0].mVolA;
		PxU32 actor1 = pairs[0].mVolB;
		PxU32 count = 1;
		for (PxU32 i = 1; i < nbPairs; i++)
		{
			PxgBroadPhasePair& report1 = pairs[i];

			PxU32 newActor0 = report1.mVolA;
			PxU32 newActor1 = report1.mVolB;

			if (newActor0 != actor0 || newActor1 != actor1)
			{
				if (count != i)
				{
					pairs[count].mVolA = newActor0;
					pairs[count].mVolB = newActor1;
				}
				actor0 = newActor0;
				actor1 = newActor1;
				count++;
			}
		}
		pairs.forceSize_Unsafe(count);
	}
}

void PxgCudaBroadPhaseSap::purgeDuplicateFoundPairs()
{
	purgeDuplicates(mFoundActorPairs);
}

void PxgCudaBroadPhaseSap::purgeDuplicateLostPairs()
{
	purgeDuplicates(mLostActorPairs);
}

void PxgCudaBroadPhaseSap::runRadixSort(const PxU32 numOfKeys, CUdeviceptr radixSortDescBuf)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.runRadixSort", mContextID);

	PxU32 startBit = 0;
	const PxU32 numPass = 8;

	for(PxU32 i=0; i<numPass; ++i)
	{
		const PxU32 descIndex = (i & 1)*3; 

		CUdeviceptr rsDesc = radixSortDescBuf + descIndex*sizeof(PxgRadixSortDesc);

		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(rsDesc), CUDA_KERNEL_PARAM(numOfKeys), CUDA_KERNEL_PARAM(startBit) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::RS_MULTIBLOCK_COUNT, PxgRadixSortKernelGridDim::RADIX_SORT, 3, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK_COUNT, PxgRadixSortKernelGridDim::RADIX_SORT, 3, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, EPILOG);

		startBit+=4;
	}

	GPU_DEBUG_STREAM(mStream, "GPU radix sort fail!!\n")
}

void PxgCudaBroadPhaseSap::sortProjectionAndHandlesWRKernel(PxU32 previousNumOfBoxes)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.sortProjectionAndHandlesWRKernel", mContextID);

	//PxU32 numHandles =  mPreviousNumOfBoxes + mCreatedHandleSize;
	//const PxU32 numHandles = mPreviousNumOfBoxes;
	const PxU32 numHandles = previousNumOfBoxes;

	if(numHandles == 0)
		return;

	PxU32 nbProjections = numHandles*2;

	//we need to pad the number of projection to the multiply of 4
	nbProjections = (nbProjections + 3) & (~3);
	
	CUdeviceptr bpBuff = mBPDescBuf.getDevicePtr();
	KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpBuff) };

	_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_INITIALIZE_RANKS, PxgBPKernelGridDim::BP_INITIALIZE_RANKS, 1, 1, PxgBPKernelBlockDim::BP_INITIALIZE_RANKS, 1, 1, 0, EPILOG);

	runRadixSort(nbProjections, mRadixSortDescBuf.getDevicePtr());

	_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_UDPATE_HANDLES, PxgBPKernelGridDim::BP_UDPATE_HANDLES, 1, 1, PxgBPKernelBlockDim::BP_UDPATE_HANDLES, 1, 1, 0, EPILOG);
}

//sort projections and handles without ranks
void PxgCudaBroadPhaseSap::sortProjectionAndHandlesWORKernel(PxU32 previousNumOfBoxes)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.sortProjectionAndHandlesWORKernel", mContextID);

	const PxU32 numHandles =  previousNumOfBoxes + mUpdateData_CreatedHandleSize;
	//const PxU32 numHandles =  mPreviousNumOfBoxes + mUpdateData_CreatedHandleSize;

	if(numHandles == 0)
		return;

	PxU32 nbProjections = numHandles*2;

	//we need to pad the number of projection to the multiply of 4
	nbProjections = (nbProjections + 3) & (~3);

	runRadixSort(nbProjections, mRadixSortWORDescBuf.getDevicePtr());
	GPU_DEBUG_STREAM(mStream, "GPU radix sort fail!!\n")
}


// PT:
// In:
//   bpDesc->boxHandles
// Out:
//   bpDesc->boxNewSapBox1D or bpDesc->boxSapBox1D
void PxgCudaBroadPhaseSap::initializeSapBoxKernel(const PxU32 numHandles, bool isNew)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.initializeSapBoxKernel", mContextID);

	const PxU32 nbBlocks = ((numHandles*2) + PxgBPKernelBlockDim::BP_INITIALIZE_SAPBOX-1)/ PxgBPKernelBlockDim::BP_INITIALIZE_SAPBOX;
	if(nbBlocks)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd), CUDA_KERNEL_PARAM(numHandles), CUDA_KERNEL_PARAM(isNew) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_INITIALIZE_SAPBOX, nbBlocks, 1, 1, PxgBPKernelBlockDim::BP_INITIALIZE_SAPBOX, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::translateAABBsKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.translateAABBsKernel", mContextID);

	if(mUpdateData_BoxesCapacity == 0)
		return;

	const PxBounds3* updateData_fpBounds = reinterpret_cast<PxBounds3*>(mBoxFpBoundsBuf.getDevicePtr());
	PxgIntegerAABB* newIntegerBounds = reinterpret_cast<PxgIntegerAABB*>(mNewIntegerBoundsBuf.getDevicePtr());
	const PxReal* updateData_contactDistances = reinterpret_cast<PxReal*>(mBoxContactDistancesBuf.getDevicePtr());
	const PxU32* updateData_envIDs = reinterpret_cast<PxU32*>(mBoxEnvIDsBuf.getDevicePtr());

	KERNEL_PARAM_TYPE kernelParams[] = {
		CUDA_KERNEL_PARAM(updateData_fpBounds),
		CUDA_KERNEL_PARAM(newIntegerBounds),
		CUDA_KERNEL_PARAM(updateData_contactDistances),
		CUDA_KERNEL_PARAM(updateData_envIDs),
		CUDA_KERNEL_PARAM(mUpdateData_BoxesCapacity),
		CUDA_KERNEL_PARAM(mDesc.gpuBroadPhaseNbBitsShiftX),
		CUDA_KERNEL_PARAM(mDesc.gpuBroadPhaseNbBitsShiftY),
		CUDA_KERNEL_PARAM(mDesc.gpuBroadPhaseNbBitsShiftZ),
		CUDA_KERNEL_PARAM(mDesc.gpuBroadPhaseNbBitsEnvIDX),
		CUDA_KERNEL_PARAM(mDesc.gpuBroadPhaseNbBitsEnvIDY),
		CUDA_KERNEL_PARAM(mDesc.gpuBroadPhaseNbBitsEnvIDZ)
	};

	const PxU32 aabbsPerBlock = PxgBPKernelBlockDim::BP_TRANSLATE_AABBS/8;
	const PxU32 nbBlocks = (mUpdateData_BoxesCapacity + aabbsPerBlock-1)/aabbsPerBlock;	// PT: do we really need mBoxesCapacity here?

	_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_TRANSLATE_AABBS, nbBlocks, 1, 1, PxgBPKernelBlockDim::BP_TRANSLATE_AABBS, 1, 1, 0, EPILOG);
}

void PxgCudaBroadPhaseSap::markRemovedPairsKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.markRemovedPairsKernel", mContextID);

	if(mUpdateData_RemovedHandleSize)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_MARK_DELETEDPAIRS, PxgBPKernelGridDim::BP_UPDATE_DELETEDPAIRS, 1, 1, PxgBPKernelBlockDim::BP_UPDATE_DELETEDPAIRS, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::markRemovedPairsProjectionsKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.markRemovedPairsProjectionsKernel", mContextID);
		
	if(mUpdateData_RemovedHandleSize)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_UPDATE_DELETEDPAIRS, PxgBPKernelGridDim::BP_UPDATE_DELETEDPAIRS, 1, 1, PxgBPKernelBlockDim::BP_UPDATE_DELETEDPAIRS, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::markUpdatedPairsKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.markUpdatedPairsKernel", mContextID);

	//if(mUpdatedHandleSize != 0)	// PT: TODO: why was this removed? ==> probably because the GPU code started reading from the AABB manager bitmap directly (which created the "evil coupling" we found before)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
		// PT: we need a new kernel to use this as a standalone BP and break the coupling between this class and the GPU AABB manager
		if(mBpDesc->updateData_updatedHandles)
			_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_UPDATE_UPDATEDPAIRS2, PxgBPKernelGridDim::BP_UPDATE_UPDATEDPAIRS2, 1, 1, PxgBPKernelBlockDim::BP_UPDATE_UPDATEDPAIRS2, 1, 1, 0, EPILOG);
		else
#endif
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_UPDATE_UPDATEDPAIRS, PxgBPKernelGridDim::BP_UPDATE_UPDATEDPAIRS, 1, 1, PxgBPKernelBlockDim::BP_UPDATE_UPDATEDPAIRS, 1, 1, 0, EPILOG);
	}
}

// PT:
// In:
//    bpDesc->numCreatedHandles
//    bpDesc->numPreviousHandles
//    bpDesc->updateData_createdHandles
//    bpDesc->newIntegerBounds
// Out:
//    bpDesc->boxProjections			// copy of newIntegerBounds but split between X/Y/Z axes
//    bpDesc->boxHandles				// see createHandle(), will have link to CPU index & some flags
//    bpDesc->oldIntegerBounds			// Kernel will set old bounds of new objects to empty
//
// boxProjections & boxHandles are parallel arrays indexed by the GPU index
void PxgCudaBroadPhaseSap::markCreatedPairsKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.markCreatedPairsKernel", mContextID);

	if(mUpdateData_CreatedHandleSize)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_UPDATE_CREATEDPAIRS, PxgBPKernelGridDim::BP_UPDATE_CREATEDPAIRS, 1, 1, PxgBPKernelBlockDim::BP_UPDATE_CREATEDPAIRS, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::calculateEndPtHistogramKernel(const bool isIncremental)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.calculateEndPtHistogramKernel", mContextID);

	CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
	KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd), CUDA_KERNEL_PARAM(isIncremental) };

	_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_ENDPT_HISTOGRAM, PxgBPKernelGridDim::BP_COMPUTE_ENDPT_HISTOGRAM, 3, 1, PxgBPKernelBlockDim::BP_COMPUTE_ENDPT_HISTOGRAM, 1, 1, 0, EPILOG);
	_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_OUTPUT_ENDPT_HISTOGRAM, PxgBPKernelGridDim::BP_OUTPUT_ENDPT_HISTOGRAM, 3, 1, PxgBPKernelBlockDim::BP_OUTPUT_ENDPT_HISTOGRAM, 1, 1, 0, EPILOG);
}

void PxgCudaBroadPhaseSap::computeRegionHistogramKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.computeRegionHistogramKernel", mContextID);

	if(mUpdateData_CreatedHandleSize)
	{
		const PxU32 nbProjections = (mNumOfBoxes + mUpdateData_RemovedHandleSize) * 2; 
		const PxU32 totalNbProjectionRegions = (nbProjections*64 + 3)&(~3);

		//zero regions
		mCudaContext->memsetD32Async(mActiveRegionTotalBuf.getDevicePtr(), 0, totalNbProjectionRegions, mStream);
		mCudaContext->memsetD32Async(mStartRegionsTotalBuf.getDevicePtr(), 0, totalNbProjectionRegions, mStream);

		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		//create regions
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_CREATE_REGIONS, PxgBPKernelGridDim::BP_CREATE_REGIONS, 1, 1, PxgBPKernelBlockDim::BP_CREATE_REGIONS, 1, 1, 0, EPILOG);
		
		//compute start region histogram inside a block
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_START_REGION_HISTOGRAM, PxgBPKernelGridDim::BP_COMPUTE_START_REGION_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_COMPUTE_START_REGION_HISTOGRAM, 1, 1, 0, EPILOG);

		//compute start region histogram between blocks
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_OUTPUT_START_REGION_HISTOGRAM, PxgBPKernelGridDim::BP_OUTPUT_START_REGION_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_OUTPUT_START_REGION_HISTOGRAM, 1, 1, 0, EPILOG);

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_REGION_HISTOGRAM, PxgBPKernelGridDim::BP_COMPUTE_REGION_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_COMPUTE_REGION_HISTOGRAM, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_OUTPUT_REGION_HISTOGRAM, PxgBPKernelGridDim::BP_OUTPUT_REGION_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_OUTPUT_REGION_HISTOGRAM, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::computeStartAndActiveHistogramKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.computeStartAndActiveHistogramKernel", mContextID);

	if(mUpdateData_CreatedHandleSize)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_WRITEOUT_ACTIVE_HISTOGRAM, PxgBPKernelGridDim::BP_WRITEOUT_ACTIVE_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_WRITEOUT_ACTIVE_HISTOGRAM, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_ACTIVE_HISTOGRAM, PxgBPKernelGridDim::BP_COMPUTE_ACTIVE_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_COMPUTE_ACTIVE_HISTOGRAM, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_OUTPUT_ACTIVE_HISTOGRAM, PxgBPKernelGridDim::BP_OUTPUT_ACTIVE_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_OUTPUT_ACTIVE_HISTOGRAM, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::performIncrementalSapKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.performIncrementalSapKernel", mContextID);

	//if(mUpdatedHandleSize != 0)	// PT: TODO: why was this removed?
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1, PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1, 1, 1, PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2, PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2, 1, 1, PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_INCREMENTAL_SAP, PxgBPKernelGridDim::BP_INCREMENTAL_SAP, 3, 1, PxgBPKernelBlockDim::BP_INCREMENTAL_SAP, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::generateNewPairsKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.generateNewPairsKernel", mContextID);

	if(mUpdateData_CreatedHandleSize)
	{
		//Need to generate pairs for created handles...
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_WRITEOUT_OVERLAPCHECKS_HISTOGRAM_NEWBOUNDS, PxgBPKernelGridDim::BP_WRITEOUT_OVERLAPCHECKS_HISTOGRAM_NEWBOUNDS, 1, 1, PxgBPKernelBlockDim::BP_WRITEOUT_OVERLAPCHECKS_HISTOGRAM_NEWBOUNDS, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_COMPUTE_OVERLAPCHECKS_HISTOGRAM, PxgBPKernelGridDim::BP_COMPUTE_OVERLAPCHECKS_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_COMPUTE_OVERLAPCHECKS_HISTOGRAM, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_OUTPUT_OVERLAPCHECKS_HISTOGRAM, PxgBPKernelGridDim::BP_OUTPUT_OVERLAPCHECKS_HISTOGRAM, 1, 1, PxgBPKernelBlockDim::BP_OUTPUT_OVERLAPCHECKS_HISTOGRAM, 1, 1, 0, EPILOG);
		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_GENERATE_FOUNDPAIR_NEWBOUNDS, PxgBPKernelGridDim::BP_GENERATE_FOUNDPAIR_NEWBOUNDS, 1, 1, PxgBPKernelBlockDim::BP_GENERATE_FOUNDPAIR_NEWBOUNDS, 1, 1, 0, EPILOG);
	}

	GPU_DEBUG_STREAM(mStream, "GPU generate new pairs fail!!\n")
}

void PxgCudaBroadPhaseSap::clearNewFlagKernel()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.clearNewFlagKernel", mContextID);

	if(mUpdateData_CreatedHandleSize)
	{
		CUdeviceptr bpDescd = mBPDescBuf.getDevicePtr();
		KERNEL_PARAM_TYPE kernelParams[] = { CUDA_KERNEL_PARAM(bpDescd) };

		_launch<GPU_BP_DEBUG>(PROLOG, PxgKernelIds::BP_CLEAR_NEWFLAG, PxgBPKernelGridDim::BP_CLEAR_NEWFLAG, 1, 1, PxgBPKernelBlockDim::BP_CLEAR_NEWFLAG, 1, 1, 0, EPILOG);
	}
}

void PxgCudaBroadPhaseSap::updateRadixSortDesc(PxgRadixSortDesc* rsDescs)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.updateRadixSortDesc", mContextID);

	for (PxU32 i = 0; i < 3; ++i)
	{
		PxU32 offIndex = i+3;
		CUdeviceptr inputKeyd = mBoxPtProjectionsBuf[i].getDevicePtr();
		CUdeviceptr inputRankd = mBoxProjectionRanksBuf[i].getDevicePtr();
		CUdeviceptr outputKeyd = mTempBoxPtProjectionBuf[i].getDevicePtr();
		CUdeviceptr outputRankd = mTempBoxPtHandlesBuf[i].getDevicePtr();
		CUdeviceptr radixCountd = mRadixCountBuf[i].getDevicePtr();

		rsDescs[i].inputKeys		= reinterpret_cast<PxU32*>(inputKeyd);
		rsDescs[i].inputRanks		= reinterpret_cast<PxU32*>(inputRankd);
		rsDescs[i].outputKeys		= reinterpret_cast<PxU32*>(outputKeyd);
		rsDescs[i].outputRanks		= reinterpret_cast<PxU32*>(outputRankd);
		rsDescs[i].radixBlockCounts	= reinterpret_cast<PxU32*>(radixCountd);

		rsDescs[offIndex].outputKeys		= reinterpret_cast<PxU32*>(inputKeyd);
		rsDescs[offIndex].outputRanks		= reinterpret_cast<PxU32*>(inputRankd);
		rsDescs[offIndex].inputKeys			= reinterpret_cast<PxU32*>(outputKeyd);
		rsDescs[offIndex].inputRanks		= reinterpret_cast<PxU32*>(outputRankd);
		rsDescs[offIndex].radixBlockCounts	= reinterpret_cast<PxU32*>(radixCountd);

		CUdeviceptr inputVald = mBoxPtHandlesBuf[i].getDevicePtr();

		mRSDescWOR[i].inputKeys			= reinterpret_cast<PxU32*>(inputKeyd);
		mRSDescWOR[i].inputRanks		= reinterpret_cast<PxU32*>(inputVald);
		mRSDescWOR[i].outputKeys		= reinterpret_cast<PxU32*>(outputKeyd);
		mRSDescWOR[i].outputRanks		= reinterpret_cast<PxU32*>(outputRankd);
		mRSDescWOR[i].radixBlockCounts	= reinterpret_cast<PxU32*>(radixCountd);

		mRSDescWOR[offIndex].outputKeys			= reinterpret_cast<PxU32*>(inputKeyd);
		mRSDescWOR[offIndex].outputRanks		= reinterpret_cast<PxU32*>(inputVald);
		mRSDescWOR[offIndex].inputKeys			= reinterpret_cast<PxU32*>(outputKeyd);
		mRSDescWOR[offIndex].inputRanks			= reinterpret_cast<PxU32*>(outputRankd);
		mRSDescWOR[offIndex].radixBlockCounts	= reinterpret_cast<PxU32*>(radixCountd);
	}
}

void PxgCudaBroadPhaseSap::updateDescriptor(PxgBroadPhaseDesc& desc)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.updateDescriptor", mContextID);

	// PT: there's some evil coupling between the BP and mAABBManager here. The CUDA code does use
	// these buffers (e.g. volumeData) so effectively the CUDA BP cannot be used alone without the Pxg AABB manager.

	// PT: added this here
	PxMemZero(&desc, sizeof(PxgBroadPhaseDesc));

	const PxU32 previousBoxes = mNumOfBoxes + mUpdateData_RemovedHandleSize - mUpdateData_CreatedHandleSize;
		
	desc.updateData_createdHandles = reinterpret_cast<PxU32*>(mCreatedHandlesBuf.getDevicePtr());
	desc.numCreatedHandles = mUpdateData_CreatedHandleSize;
	desc.updateData_removedHandles = reinterpret_cast<PxU32*>(mRemovedHandlesBuf.getDevicePtr());
	desc.numRemovedHandles = mUpdateData_RemovedHandleSize;
	
	// PT: TODO: replace with adapter? I think this won't work without the bitmaps anyway?
	if(mAABBManager)
	{
		// PT: this data is used in:
		// - markUpdatedPairsLaunch (BP_UPDATE_UPDATEDPAIRS)
		{
			desc.aabbMngr_changedHandleMap = reinterpret_cast<PxU32*>(mAABBManager->getChangedAABBMgrHandles());
			desc.aabbMngr_changedHandleBitMapWordCounts = mAABBManager->getChangedAABBMgActorHandleMap().getWordCount();
			desc.aabbMngr_addedHandleMap = reinterpret_cast<PxU32*>(mAABBManager->getAddedHandles());
			desc.aabbMngr_removedHandleMap = reinterpret_cast<PxU32*>(mAABBManager->getRemovedHandles());
			desc.aabbMngr_aggregatedBoundHandles = reinterpret_cast<PxU32*>(mAABBManager->getAggregatedBounds());
		}

		// PT: this data is used in:
		// - doAggPairCollisions (AGG_PAIR_COLLISION)
		// - accumulateReportsStage_1 (BP_ACCUMULATE_REPORT_STAGE_1)
		// - accumulateReportsStage_2 (BP_ACCUMULATE_REPORT_STAGE_2)
		desc.aabbMngr_volumeData = reinterpret_cast<Bp::VolumeData*>(mAABBManager->mVolumDataBuf.getDevicePtr());
	}
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
	else
	{
		// PT: the GPU AABB manager never passes updated handles, the list is always empty! In this codepath
		// (used for standalone BPs) we make it work again with an array of updated handles.
		desc.updateData_updatedHandles = reinterpret_cast<PxU32*>(mUpdatedHandlesBuf.getDevicePtr());
		desc.numUpdatedHandles = mUpdateData_UpdatedHandleSize;
		//printf("%d\n", desc.numUpdatedHandles);
	}
#endif

	desc.oldIntegerBounds = reinterpret_cast<PxgIntegerAABB*>(mOldIntegerBoundsBuf.getDevicePtr());
	desc.newIntegerBounds = reinterpret_cast<PxgIntegerAABB*>(mNewIntegerBoundsBuf.getDevicePtr());
	desc.updateData_fpBounds = reinterpret_cast<PxBounds3*>(mBoxFpBoundsBuf.getDevicePtr());
	desc.updateData_contactDistances = reinterpret_cast<PxReal*>(mBoxContactDistancesBuf.getDevicePtr());
	desc.updateData_groups = reinterpret_cast<PxU32*>(mBoxGroupsBuf.getDevicePtr());
	desc.updateData_envIDs = reinterpret_cast<PxU32*>(mBoxEnvIDsBuf.getDevicePtr());

	desc.numPreviousHandles = previousBoxes;
	//desc.numHandles = mNumOfBoxes;
	desc.foundPairReport	= reinterpret_cast<PxgBroadPhasePair*>(mFoundPairsBuf.getDevicePtr());
	desc.lostPairReport		= reinterpret_cast<PxgBroadPhasePair*>(mLostPairsBuf.getDevicePtr());

	desc.foundAggPairReport		= reinterpret_cast<PxgBroadPhasePair*>(mFoundAggregateBuf.getDevicePtr());
	desc.lostAggPairReport		= reinterpret_cast<PxgBroadPhasePair*>(mLostAggregateBuf.getDevicePtr());

	desc.foundActorPairReport	= reinterpret_cast<PxgBroadPhasePair*>(mFoundActorBuf.getDevicePtr());
	desc.lostActorPairReport	= reinterpret_cast<PxgBroadPhasePair*>(mLostActorBuf.getDevicePtr());

	desc.foundPairReportMap	= reinterpret_cast<PxgBroadPhasePair*>(getMappedDevicePtr(mCudaContext, mFoundActorPairs.begin()));
	desc.lostPairReportMap	= reinterpret_cast<PxgBroadPhasePair*>(getMappedDevicePtr(mCudaContext, mLostActorPairs.begin()));

	for (PxU32 i = 0; i < 3; ++i)
	{
		/*const PxU32 offset = i*nbProjections;
		desc.boxProjectionRanks[i] = projRanksGpuPtr + offset;*/

		desc.boxSapBox1D[i]			= reinterpret_cast<PxgSapBox1D*>(mBoxSapBox1DBuf[i].getDevicePtr());
		desc.boxNewSapBox1D[i]		= reinterpret_cast<PxgSapBox1D*>(mNewBoxSapBox1DBuf[i].getDevicePtr());
		desc.boxProjectionRanks[i]	= reinterpret_cast<PxU32*>(mBoxProjectionRanksBuf[i].getDevicePtr());
		desc.boxProjections[i]		= reinterpret_cast<PxU32*>(mBoxPtProjectionsBuf[i].getDevicePtr());
		desc.boxHandles[0][i]		= reinterpret_cast<PxU32*>(mBoxPtHandlesBuf[i].getDevicePtr());
		desc.boxHandles[1][i]		= reinterpret_cast<PxU32*>(mBoxPtHandlesBuf[i+3].getDevicePtr());		
		
		{
			desc.totalEndPtHistogram[i] = reinterpret_cast<PxU32*>(mTotalEndPtHistogramBuf[i].getDevicePtr());
			desc.blockTotalEndPtHistogram[i] = reinterpret_cast<PxU32*>(mBlockTotalEndPtHistogramBuf[i].getDevicePtr());

			for (PxU32 j = 0; j < 2; ++j)
			{
				//desc.boxHandles[j][i] = handleGpuPtr[j] + i*projectionCount[j];
				const PxU32 index = j * 3 + i;
				desc.endPtHistogram[j][i] = reinterpret_cast<PxU32*>(mEndPtHistogramBuf[index].getDevicePtr());
				desc.blockEndPtHistogram[j][i] = reinterpret_cast<PxU32*>(mBlockEndPtHistogramBuf[index].getDevicePtr());
				desc.startPtHistogram[j][i] = reinterpret_cast<PxU32*>(mStartPtHistogramBuf[index].getDevicePtr());
				desc.blockStartPtHistogram[j][i] = reinterpret_cast<PxU32*>(mBlockStartPtHistogramBuf[index].getDevicePtr());

				desc.endPointHandles[j][i] = reinterpret_cast<PxU32*>(mEndPtHandleBuf[index].getDevicePtr());
				desc.startPointHandles[j][i] = reinterpret_cast<PxU32*>(mStartPtHandleBuf[index].getDevicePtr());
			}

			desc.incrementalComparisons[i] = reinterpret_cast<PxU32*>(mIncrementalComparisons[i].getDevicePtr());
			desc.incrementalBlockComparisons[i] = reinterpret_cast<PxU32*>(mIncrementalBlockComparisons[i].getDevicePtr());
		}
	}
		
	for (PxU32 i = 0; i < 2; ++i)
	{
		desc.aggReportBlock[i] = reinterpret_cast<PxU32*>(mAggregateReportBlockBuf[i].getDevicePtr());
		desc.actorReportBlock[i] = reinterpret_cast<PxU32*>(mActorReportBlockBuf[i].getDevicePtr());
	}

	desc.activeRegionsHistogram = reinterpret_cast<PxU32*>(mActiveRegionTotalBuf.getDevicePtr());
	desc.startRegionsHistogram = reinterpret_cast<PxU32*>(mStartRegionsTotalBuf.getDevicePtr());
	desc.orderedActiveRegionHandles = reinterpret_cast<PxU32*>(mOrderedActiveRegionHandlesTotalBuf.getDevicePtr());
	desc.orderedStartRegionHandles = reinterpret_cast<PxU32*>(mOrderedStartRegionHandlesTotalBuf.getDevicePtr());

	desc.blockOverlapChecksRegion = reinterpret_cast<regionOverlapType*>(mBlockOverlapChecksRegionBuf.getDevicePtr());
	desc.overlapChecksRegion = reinterpret_cast<regionOverlapType*>(mOverlapChecksRegionBuf.getDevicePtr());
	desc.overlapChecksHandleRegiones = reinterpret_cast<PxgHandleRegion*>(mOverlapChecksHandleRegionBuf.getDevicePtr());
	desc.regionRange = reinterpret_cast<PxgIntegerRegion*>(mRegionRangeBuf.getDevicePtr());
	desc.startRegionAccum = reinterpret_cast<PxU32*>(mStartRegionAccumBuf.getDevicePtr());
	desc.blockStartRegionAccum = reinterpret_cast<PxU32*>(mBlockStartRegionAccumBuf.getDevicePtr());
	desc.regionAccum = reinterpret_cast<PxU32*>(mRegionAccumBuf.getDevicePtr());
	desc.blockRegionAccum = reinterpret_cast<PxU32*>(mBlockRegionAccumBuf.getDevicePtr());

	desc.sharedFoundPairIndex = 0;
	desc.sharedLostPairIndex = 0;
	desc.sharedFoundAggPairIndex = 0;
	desc.sharedLostAggPairIndex = 0;

	desc.startRegionAccumTotal = 0;

	desc.regionAccumTotal = mRegionAccumTotal;
	desc.overlapChecksTotalRegion = mOverlapChecksTotalRegion;
	desc.max_found_lost_pairs = mMaxFoundLostPairs;
	desc.found_lost_pairs_overflow_flags = false;

	desc.max_found_lost_agg_pairs = mMaxAggFoundLostPairs;
}

void PxgCudaBroadPhaseSap::update(PxcScratchAllocator* /*scratchAllocator*/, const Bp::BroadPhaseUpdateData& updateData, PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.update", mContextID);

	// PT: TODO: this function is now the only place left using getGpuStateChanged() and getStateChanged()
	// PT: TODO: could we move this outside of the update call to sever this last connection?
	*mPinnedEvent = 0;

	PxScopedCudaLock _lock_(*mCudaContextManager);

	const PxU32 previousNumOfBoxes = mNumOfBoxes; 
	gpuDMAUp(updateData, *mBpDesc, mRSDesc);

	const bool gpuStateChanged = updateData.getGpuStateChanged();
	bool forcedUpdate = false;

	if((updateData.getNumCreatedHandles() + updateData.getNumRemovedHandles()) == 0 && !updateData.getStateChanged() && !gpuStateChanged)
	{
		if (mForceUpdate)
		{
			forcedUpdate = true;
			//We force a single update after everything has gone to sleep to force through some
			//properties like the double-buffered bounds swap. If we don't do this, then some of the 
			//GPU aggregate logic can fail.
		}
		else
			return;
	}

	mForceUpdate = !forcedUpdate;

	// PT: not all kernels are needed for all cases (added / updated / removed).
	// For an easier time analysing the code, it can be good to only trace the code needed for
	// "one shot queries", ignoring the bits needed for updated & removed objects.
	const bool oneShotQuery = false;

	// translate from FP bounds to integer bounds
	translateAABBsKernel();

	if(!oneShotQuery)
	{
		//we mark pairs as removed but we didn't change their projections in the previous frame's box handles
		markRemovedPairsKernel();

		//we mark pairs as updated(update projections) in the current frame's box handles
		if(gpuStateChanged)
			markUpdatedPairsKernel();

		//sort projections in the current frame and produce ranks which is used in the incremental sap to simulate swap. 
		//Also, we need to use the ranks to update the current frame's box handle
		sortProjectionAndHandlesWRKernel(previousNumOfBoxes);
		
		//histogram for projections end pointsm
		calculateEndPtHistogramKernel(true);

		//perform incremental sap to produce pairs for updated pairs and lost pairs
		if(gpuStateChanged)
			performIncrementalSapKernel();
	}

	if(mUpdateData_RemovedHandleSize)
	{
		//we need to recalculate the sap box because we resort the projections and handles based on the updated pairs
		initializeSapBoxKernel(previousNumOfBoxes, false);

		//we need to reset the projections for the removed pairs so that they can be shuffled to the end of the array after sort
		markRemovedPairsProjectionsKernel();
	}

	if(mUpdateData_CreatedHandleSize || mUpdateData_RemovedHandleSize)
	{
		markCreatedPairsKernel();
	
		sortProjectionAndHandlesWORKernel(previousNumOfBoxes);
	
		calculateEndPtHistogramKernel(false);
	}
		
	initializeSapBoxKernel(mNumOfBoxes, false);

	computeRegionHistogramKernel();

	computeStartAndActiveHistogramKernel();

	generateNewPairsKernel();

	if(!oneShotQuery)
		PxgCudaBuffer::swapBuffer(mNewIntegerBoundsBuf, mOldIntegerBoundsBuf);

	clearNewFlagKernel();

	runCopyResultsKernel(*mBpDesc);

	//mCudaContext->streamFlush(mStream);
}

// PT: called from PxgAABBManager::preBpUpdate_GPU
void PxgCudaBroadPhaseSap::preBroadPhase(const Bp::BroadPhaseUpdateData& updateData)
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.preBroadPhase", mContextID);

	PxScopedCudaLock _lock_(*mCudaContextManager);

	//mPreviousNumOfBoxes = mNumOfBoxes;
	//gpuDMAUp(updateData, *mBpDesc, mRSDesc);

	// PT: the code below used to be in "gpuDmaUpSharedData"

	const PxU32 capacity = updateData.getCapacity();
	mUpdateData_BoxesCapacity = capacity;

	const PxU32 boundsSize = capacity * sizeof(PxBounds3);
	const PxU32 distanceSize = capacity * sizeof(PxReal);
	const PxU32 groupSize = capacity * sizeof(PxU32);
	const PxU32 envIDSize = capacity * sizeof(PxU32);

	mBoxContactDistancesBuf.allocate(distanceSize, PX_FL);
	mBoxGroupsBuf.allocate(groupSize, PX_FL);
	if(updateData.getEnvIDs())
		mBoxEnvIDsBuf.allocate(envIDSize, PX_FL);

	if(updateData.getStateChanged())	// PT: otherwise the call should have been skipped
	{
		mBoxFpBoundsBuf.allocate(boundsSize, PX_FL);
		mCudaContext->memcpyHtoDAsync(mBoxFpBoundsBuf.getDevicePtr(), updateData.getAABBs(), boundsSize, mStream);
	}
	mCudaContext->memcpyHtoDAsync(mBoxContactDistancesBuf.getDevicePtr(), updateData.getContactDistance(), distanceSize, mStream);
	mCudaContext->memcpyHtoDAsync(mBoxGroupsBuf.getDevicePtr(), updateData.getGroups(), groupSize, mStream);
	if(updateData.getEnvIDs())
		mCudaContext->memcpyHtoDAsync(mBoxEnvIDsBuf.getDevicePtr(), updateData.getEnvIDs(), envIDSize, mStream);
}

void PxgCudaBroadPhaseSap::fetchBroadPhaseResults()
{
	PX_PROFILE_ZONE("PxgCudaBroadPhaseSap.fetchBroadPhaseResults", mContextID);

	PxScopedCudaLock _lock_(*mCudaContextManager);

	gpuDMABack(*mBpDesc);

	//purgeDuplicateFoundPairs();
	//purgeDuplicateLostPairs();

	// flip double buffer
	{
		for(PxU32 i=0; i<3; ++i)
		{
			const PxU32 swapId = i + 3;
			PxgCudaBuffer::swapBuffer(mBoxPtHandlesBuf[i], mBoxPtHandlesBuf[swapId]);
			PxgCudaBuffer::swapBuffer(mBlockEndPtHistogramBuf[i], mBlockEndPtHistogramBuf[swapId]);
			PxgCudaBuffer::swapBuffer(mEndPtHistogramBuf[i], mEndPtHistogramBuf[swapId]);
			PxgCudaBuffer::swapBuffer(mEndPtHandleBuf[i], mEndPtHandleBuf[swapId]);

			PxgCudaBuffer::swapBuffer(mBlockStartPtHistogramBuf[i], mBlockStartPtHistogramBuf[swapId]);
			PxgCudaBuffer::swapBuffer(mStartPtHistogramBuf[i], mStartPtHistogramBuf[swapId]);
			PxgCudaBuffer::swapBuffer(mStartPtHandleBuf[i], mStartPtHandleBuf[swapId]);
		}
	}
}
