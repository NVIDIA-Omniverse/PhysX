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

#ifndef PXG_CUDA_BROADPHASE_SAP_H
#define PXG_CUDA_BROADPHASE_SAP_H

#include "foundation/PxPinnedArray.h"
#include "BpBroadPhase.h"
#include "PxgCudaBuffer.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgBroadPhasePairReport.h"
#include "PxgHeapMemAllocator.h"

// PT: for SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
#include "PxgBroadPhaseDesc.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

namespace physx
{
	class KernelWrangler;
	class PxgBpCudaMemoryAllocator;
	class PxgSapPairManager;
	struct PxGpuDynamicsMemoryConfig;
	class PxgAABBManager;

	struct PxgBroadPhaseDesc;
	struct PxgRadixSortDesc;
	class PxgCudaKernelWranglerManager;
	class PxSceneDesc;

	namespace Bp
	{
		class BroadPhaseUpdateData;
	}

class PxgCudaBroadPhaseSap : public Bp::BroadPhase
{
												PX_NOCOPY(PxgCudaBroadPhaseSap)
	public:
												PxgCudaBroadPhaseSap(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, const PxGpuDynamicsMemoryConfig& init, PxgHeapMemoryAllocatorManager* heapMemoryManager, PxU64 contextID);
												~PxgCudaBroadPhaseSap();
	// Bp::BroadPhase
	virtual			PxBroadPhaseType::Enum		getType()				const		PX_OVERRIDE	{ return PxBroadPhaseType::eGPU; }
	virtual			void						release()	PX_OVERRIDE;
	virtual			void						update(PxcScratchAllocator* scratchAllocator, const Bp::BroadPhaseUpdateData& updateData, PxBaseTask* continuation)	PX_OVERRIDE;
	virtual			void						preBroadPhase(const Bp::BroadPhaseUpdateData& updateData)	PX_OVERRIDE;
	virtual			void						fetchBroadPhaseResults()			PX_OVERRIDE;

	virtual			const Bp::BroadPhasePair*	getCreatedPairs(PxU32& nbCreatedPairs)		const 		PX_OVERRIDE
												{
													nbCreatedPairs = mFoundActorPairs.size();
													return reinterpret_cast<const Bp::BroadPhasePair*>(mFoundActorPairs.begin());
												}

	virtual			const Bp::BroadPhasePair*	getDeletedPairs(PxU32& nbDeletedPairs)		const 		PX_OVERRIDE
												{
													nbDeletedPairs = mLostActorPairs.size();
													return reinterpret_cast<const Bp::BroadPhasePair*>(mLostActorPairs.begin());
												}

	virtual			void						freeBuffers()						PX_OVERRIDE;
	// PT: TODO: shift origin for GPU BP?
	virtual			void						shiftOrigin(const PxVec3& /*shift*/, const PxBounds3* /*boundsArray*/, const PxReal* /*contactDistances*/) PX_OVERRIDE	{}
#if PX_CHECKED
	virtual			bool						isValid(const Bp::BroadPhaseUpdateData& updateData) const PX_OVERRIDE	{ PX_UNUSED(updateData); return true; }
#endif
	//~Bp::BroadPhase

	PX_FORCE_INLINE	PxgTypedCudaBuffer<PxBounds3>& getBoundsBuffer()			{ return mBoxFpBoundsBuf;				}
	PX_FORCE_INLINE	PxgTypedCudaBuffer<PxReal>&	getContactDistBuffer()			{ return mBoxContactDistancesBuf;		}
//	PX_FORCE_INLINE	PxCudaContextManager*		getCudaContextManager()			{ return mCudaContextManager;			}
	PX_FORCE_INLINE	CUstream					getBpStream()					{ return mStream;						}
	PX_FORCE_INLINE	PxgDevicePointer<PxgBroadPhaseDesc>	getBroadPhaseDescDevicePtr() { return mBPDescBuf.getTypedDevicePtr();}
//	PX_FORCE_INLINE	CUdeviceptr					getFoundPairsDevicePtr()		{ return mFoundPairsBuf.getDevicePtr();	}
//	PX_FORCE_INLINE	CUdeviceptr					getLostPairsDevicePtr()			{ return mLostPairsBuf.getDevicePtr();	}
	PX_FORCE_INLINE	PxgAABBManager*				getAABBManager()				{ return mAABBManager;					}

					void						purgeDuplicateFoundPairs();
					void						purgeDuplicateLostPairs();

	// PT: TODO: wait, why is this in the BP?
	PX_FORCE_INLINE	void						sortPairs(PxPinnedArray<PxgBroadPhasePair>& pairs)	{ sortBuffer(pairs.begin(), pairs.size());	}
	PX_FORCE_INLINE	void						setGPUAABBManager(PxgAABBManager* manager)			{ mAABBManager = manager;					}

#if PX_ENABLE_SIM_STATS
	PX_FORCE_INLINE PxU32						getFoundLostPairsStats()		{ return mFoundLostPairsStats; }
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	private:
					void						gpuDMAUp(const Bp::BroadPhaseUpdateData& updateData, PxgBroadPhaseDesc& desc, PxgRadixSortDesc* rsDescs);
					void						gpuDMABack(const PxgBroadPhaseDesc& desc);

					void						createGpuStreamsAndEvents();
					void						releaseGpuStreamsAndEvents();

					void						translateAABBsKernel();
					void						markRemovedPairsKernel();
					void						markRemovedPairsProjectionsKernel();
					void						markUpdatedPairsKernel();
					void						markCreatedPairsKernel();

					void						sortProjectionAndHandlesWRKernel(PxU32 previousNumOfBoxes);
					void						sortProjectionAndHandlesWORKernel(PxU32 previousNumOfBoxes);
					void						initializeSapBoxKernel(const PxU32 numHandles, bool isNew);
					void						calculateEndPtHistogramKernel(const bool isIncremental);

					void						computeRegionHistogramKernel();
					void						computeStartAndActiveHistogramKernel();
					void						performIncrementalSapKernel();
					void						generateNewPairsKernel();
					void						clearNewFlagKernel();

					void						updateDescriptor(PxgBroadPhaseDesc& bpDesc);
					void						updateRadixSortDesc(PxgRadixSortDesc* rsDesc);
					void						runRadixSort(const PxU32 numOfKeys, CUdeviceptr radixSortDescBuf);

					void						purgeDuplicates(PxPinnedArray<PxgBroadPhasePair>& pairs);
	
					void						runCopyResultsKernel(PxgBroadPhaseDesc& desc);
					void						sortBuffer(PxgBroadPhasePair* reportBuffer, PxU32 size);

					PxU64						mContextID;

					// PT: from PxgBroadPhaseSap
					PxU32							mNumOfBoxes;		//total number of boxes in the scene
					PxU32							mUpdateData_CreatedHandleSize;
					PxU32							mUpdateData_RemovedHandleSize;
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
					PxU32							mUpdateData_UpdatedHandleSize;
#endif
					PxU32							mUpdateData_BoxesCapacity;

					PxgCudaKernelWranglerManager*	mGpuKernelWranglerManager;

					PxCudaContextManager*			mCudaContextManager;
					PxCudaContext*					mCudaContext;

					PxgHeapMemoryAllocatorManager*	mHeapMemoryManager;

					PxgTypedCudaBuffer<PxU32>       mCreatedHandlesBuf;
					PxgTypedCudaBuffer<PxU32>       mRemovedHandlesBuf;
#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
					PxgTypedCudaBuffer<PxU32>       mUpdatedHandlesBuf;
#endif

					// PT: Description:                         |Comes from:                                |Passed to kernels as:
					// -----------------------------------------|-------------------------------------------|------------------------------
					PxgTypedCudaBuffer<PxBounds3>           mBoxFpBoundsBuf;            // box bounds in device memory              |BroadPhaseUpdateData::getAABBs()           |updateData_fpBounds
					PxgTypedCudaBuffer<PxReal>              mBoxContactDistancesBuf;    // contact distances in device memory       |BroadPhaseUpdateData::getContactDistance() |updateData_contactDistances
					PxgTypedCudaBuffer<PxU32>               mBoxGroupsBuf;              // box groups in device memory              |BroadPhaseUpdateData::getGroups            |updateData_groups
					PxgTypedCudaBuffer<PxU32>               mBoxEnvIDsBuf;              // box env IDs in device memory             |BroadPhaseUpdateData::getEnvIDs            |updateData_envIDs
					PxgTypedCudaBuffer<PxgIntegerAABB>      mNewIntegerBoundsBuf;       // integer bounds in device memory          |translateAABBsLaunch kernal                |newIntegerBounds
					PxgTypedCudaBuffer<PxgIntegerAABB>      mOldIntegerBoundsBuf;       // integer bounds in device memory          |-                                          |oldIntegerBounds
	
					PxgCudaBufferN<3>           mBoxPtProjectionsBuf;       // integer bounds in device memory          |markCreatedPairsLaunch kernal              |boxProjections
					PxgCudaBufferN<3>           mBoxProjectionRanksBuf;
					PxgCudaBufferN<6>           mBoxPtHandlesBuf;           //double buffer
					PxgCudaBufferN<3>           mTempBoxPtProjectionBuf;
					PxgCudaBufferN<3>           mTempBoxPtHandlesBuf;
					PxgCudaBufferN<3>           mRadixCountBuf;
	
					PxgCudaBufferN<3>           mBoxSapBox1DBuf;            //PxgSapBox1D, the size should be the same as handles
					PxgCudaBufferN<3>           mNewBoxSapBox1DBuf;         //PxgSapBox1D, the size should be the same as handles
	
					PxgCudaBufferN<6>           mEndPtHistogramBuf;         //! Histogram for all start handles
					PxgCudaBufferN<6>           mBlockEndPtHistogramBuf;
					PxgCudaBufferN<6>           mEndPtHandleBuf;
	
					PxgCudaBufferN<6>           mStartPtHistogramBuf;
					PxgCudaBufferN<6>           mBlockStartPtHistogramBuf;
					PxgCudaBufferN<6>           mStartPtHandleBuf;
	
					PxgCudaBufferN<6>           mTotalEndPtHistogramBuf;
					PxgCudaBufferN<6>           mBlockTotalEndPtHistogramBuf;
	
					PxgTypedCudaBuffer<int>     mActiveRegionTotalBuf;
					PxgTypedCudaBuffer<int>     mStartRegionsTotalBuf;
					PxgTypedCudaBuffer<int>     mOrderedActiveRegionHandlesTotalBuf;
					PxgTypedCudaBuffer<int>     mOrderedStartRegionHandlesTotalBuf;
	
					PxgTypedCudaBuffer<int>     mOverlapChecksRegionBuf;
					PxgTypedCudaBuffer<int>     mBlockOverlapChecksRegionBuf;
					PxU32                       mOverlapChecksTotalRegion;
	
					PxgTypedCudaBuffer<PxgHandleRegion>  mOverlapChecksHandleRegionBuf;
	
					PxgCudaBufferN<3>           mIncrementalComparisons;
					PxgCudaBufferN<3>           mIncrementalBlockComparisons;
	
					PxgCudaBufferN<2>           mAggregateReportBlockBuf;
					PxgCudaBufferN<2>           mActorReportBlockBuf;
	
					PxgTypedCudaBuffer<PxgIntegerRegion> mRegionRangeBuf;
	
					PxgTypedCudaBuffer<int>               mStartRegionAccumBuf;
					PxgTypedCudaBuffer<int>               mBlockStartRegionAccumBuf;
					PxU32                                 mStartRegionAccumTotal; //need to write back to cpu every frame
														  
					PxgTypedCudaBuffer<int>               mRegionAccumBuf;
					PxgTypedCudaBuffer<int>               mBlockRegionAccumBuf;
					PxU32                                 mRegionAccumTotal; //need to write back to cpu every frame
	
					PxgTypedCudaBuffer<PxgBroadPhasePair> mFoundPairsBuf;     //total found pairs(include actors and aggregates)
					PxgTypedCudaBuffer<PxgBroadPhasePair> mLostPairsBuf;      //total lost pairs(include actors and aggregates)
	
					PxgTypedCudaBuffer<PxgBroadPhasePair> mFoundAggregateBuf;
					PxgTypedCudaBuffer<PxgBroadPhasePair> mLostAggregateBuf;
	
					PxgTypedCudaBuffer<PxgBroadPhasePair> mFoundActorBuf;
					PxgTypedCudaBuffer<PxgBroadPhasePair> mLostActorBuf;
	
					PxgTypedCudaBuffer<PxgBroadPhaseDesc> mBPDescBuf;
					PxgTypedCudaBuffer<PxgRadixSortDesc>  mRadixSortDescBuf;
					PxgTypedCudaBuffer<PxgRadixSortDesc>  mRadixSortWORDescBuf;
	
					CUstream					mStream;
					CUevent						mEvent;			
	
					PxU32*						mPinnedEvent;
		
					PxgBroadPhaseDesc*			mBpDesc;
					PxgRadixSortDesc*			mRSDesc;
					PxgRadixSortDesc*			mRSDescWOR; //wor :: without ranks
	
				    PxPinnedArray<PxgBroadPhasePair>	mFoundActorPairs;
				    PxPinnedArray<PxgBroadPhasePair>	mLostActorPairs;
					//PxArray<PxU32>				mHistogramBuffer;
					//PxArray<PxgBroadPhasePair>	mTempPairBuffer;

					PxU32						mMaxFoundLostPairs;
					PxU32						mMaxAggFoundLostPairs;

					PxgAABBManager*				mAABBManager;

#if PX_ENABLE_SIM_STATS
					PxU32						mFoundLostPairsStats; // keeps track of max lost found pairs value to tune preallocated buffer size.
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

					bool						mForceUpdate;
};

}
#endif
