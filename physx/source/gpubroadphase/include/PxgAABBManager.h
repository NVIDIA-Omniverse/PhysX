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

#ifndef PXG_AABBMANAGER_H
#define PXG_AABBMANAGER_H

#include "BpAABBManagerBase.h"
#include "PxgCudaBuffer.h"
#include "PxgAggregate.h"
#include "CmIDPool.h"
#include "PxgBroadPhasePairReport.h"
#include "PxgAggregateDesc.h"
#include "CmTask.h"
#include "foundation/PxPinnedArray.h"

namespace physx
{
	class PxgCudaKernelWranglerManager;
	class PxCudaContextManager;
	class PxgCudaBroadPhaseSap;
	struct PxGpuDynamicsMemoryConfig;

	class PxgAABBManager;
	class PxSceneDesc;

	struct PxBoundTransformUpdate
	{
		PxU32 indexTo;
		PxU32 indexFrom; //MSB stores info if the bound is new. New is copied from cpu array, not new - from gpu array
	};

	class PxgProcessFoundPairTask : public Cm::Task 
	{
		PxgAABBManager* mManager;
	public:
		PxgProcessFoundPairTask(PxgAABBManager* manager) : Cm::Task(0), mManager(manager)
		{
		}

		virtual void runInternal();
		virtual const char* getName() const { return "PxgProcessFoundPairTask"; }
	};

	class PxgProcessLostPairTask : public Cm::Task
	{
		PxgAABBManager* mManager;
	public:
		PxgProcessLostPairTask(PxgAABBManager* manager) : Cm::Task(0), mManager(manager)
		{
		}

		virtual void runInternal();
		virtual const char* getName() const { return "PxgProcessLostPairTask"; }
	};
	
	class PxgAABBManager : public Bp::AABBManagerBase
	{
	public:
											PxgAABBManager(PxgCudaKernelWranglerManager* gpuKernelWrangler,
												PxCudaContextManager* cudaContextManager,
												PxgHeapMemoryAllocatorManager* heapMemoryManager,
												const PxGpuDynamicsMemoryConfig& config,
												Bp::BroadPhase& bp, Bp::BoundsArray& boundsArray, PxFloatArrayPinned& contactDistance,
												PxU32 maxNbAggregates, PxU32 maxNbShapes, PxVirtualAllocator& allocator, PxU64 contextID,
												PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode);

		virtual								~PxgAABBManager() {}

		// AABBManagerBase
		virtual			void				destroy()	PX_OVERRIDE;
		virtual			Bp::AggregateHandle	createAggregate(Bp::BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint, PxU32 envID)	PX_OVERRIDE;
		virtual			bool				destroyAggregate(Bp::BoundsIndex& index, Bp::FilterGroup::Enum& group, Bp::AggregateHandle aggregateHandle)	PX_OVERRIDE;
		virtual			bool				addBounds(Bp::BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData, Bp::AggregateHandle aggregateHandle, Bp::ElementType::Enum volumeType, PxU32 envID)	PX_OVERRIDE;
		virtual			bool				removeBounds(Bp::BoundsIndex index)	PX_OVERRIDE;
		virtual			void				updateBPFirstPass(PxU32 numCpuTasks, Cm::FlushPool& flushPool, bool hasContactDistanceUpdated, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual			void				updateBPSecondPass(PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual			void				postBroadPhase(PxBaseTask*, Cm::FlushPool& flushPool)	PX_OVERRIDE;
		virtual			void				reallocateChangedAABBMgActorHandleMap(const PxU32 size)	PX_OVERRIDE;
		virtual			void				visualize(PxRenderOutput& out)	PX_OVERRIDE;
		virtual			void				releaseDeferredAggregateIds()	PX_OVERRIDE;
		virtual			void				setGPUStateChanged()			PX_OVERRIDE	{ mGPUStateChanged = true;			}
		virtual			void				setPersistentStateChanged()		PX_OVERRIDE	{ mPersistentStateChanged = true;	}
		//~AABBManagerBase

						PxgCudaBuffer		mVolumDataBuf;

						void				markAggregateBoundsBitmap();

						void				processFoundPairs();
						void				processLostPairs();

//		PX_FORCE_INLINE	PxBitMapPinned&		getAggregatedBoundMap()				{ return mAggregatedBoundMap; }
		PX_FORCE_INLINE	CUdeviceptr			getAggregatedBounds()				{ return mAggregatedBoundsBuf.getDevicePtr(); }
		PX_FORCE_INLINE	CUdeviceptr			getAddedHandles()					{ return mAddedHandleBuf.getDevicePtr(); }
		PX_FORCE_INLINE	CUdeviceptr			getRemovedHandles()					{ return mRemovedHandleBuf.getDevicePtr(); }
		PX_FORCE_INLINE	CUdeviceptr			getChangedAABBMgrHandles()			{ return mChangedAABBMgrHandlesBuf.getDevicePtr(); }
		PX_FORCE_INLINE	PxU32				getChangedAABBMgrHandlesWordCount()	{ return mChangedHandleMap.getWordCount(); }

	private:
						void				preBpUpdate_GPU();
						void				gpuDmaDataUp();
						void				clearDirtyAggs();
						void				resizeFoundAndLostPairs();
						void				computeAggregateBounds();
						void				updateDescriptor(CUstream bpStream);
		
		PxgCudaKernelWranglerManager*		mGpuKernelWranglerManager;
		PxCudaContextManager*				mCudaContextManager;
		PxCudaContext*						mCudaContext;
		PxgHeapMemoryAllocatorManager*		mHeapMemoryManager;

		PxArray<PxgAggregate>				mAggregates;				//cpu mirror

		PxPinnedArray<PxgAggregatePair>		mAggregatePairs;
		PxPinnedArray<Bp::AggregateHandle>	mDirtyAggregateIndices;
		PxPinnedArray<PxgAggregate>			mDirtyAggregates;

		//found and lost pairs for agg vs agg and agg vs actor
		PxPinnedArray<PxgBroadPhasePair>	mFoundPairs;
		PxPinnedArray<PxgBroadPhasePair>	mLostPairs;

						PxInt32ArrayPinned	mDirtyBoundIndices;
						PxInt32ArrayPinned	mDirtyBoundStartIndices; //start index of each aggregate in the dirty bound indices list

						PxInt32ArrayPinned  mRemovedAggregatedBounds;
						PxInt32ArrayPinned	mAddedAggregatedBounds;

						Cm::DeferredIDPool	mAggregatesIdPool; //generate the remap id between pxgbodysim and pxgaggregate
						PxBitMap			mDirtyAggregateBitMap;
						PxBitMapPinned		mAggregatedBoundMap;

		PxArray<PxgAggregateBuffer*>		mAggregateBufferArray;

						PxgAggregateDesc*	mAggregateDesc;
		
						PxgCudaBuffer		mAggregateBuf;
						PxgCudaBuffer		mAggregatePairsBuf;
		
						PxgCudaBuffer		mDirtyAggregateIndiceBuf;
						PxgCudaBuffer		mDirtyAggregateBuf;

						PxgCudaBuffer		mDirtyBoundIndicesBuf;
						PxgCudaBuffer		mDirtyBoundStartIndicesBuf;

						PxgCudaBuffer		mRemovedAggregatedBoundsBuf;
						PxgCudaBuffer		mAddedAggregatedBoundsBuf;
		
						PxgCudaBuffer		mAggPairBuf;	//persistent pairs
						PxgCudaBuffer		mNumAggPairBuf; // number of app pairs
						PxgCudaBuffer		mAggregateDescBuf;

						PxgCudaBuffer		mFoundPairsBuf;
						PxgCudaBuffer		mLostPairsBuf;

						PxgCudaBuffer		mFreeIDPool;
						PxgCudaBuffer		mFreeIDs;

						PxgCudaBuffer		mRemoveBitmap;
						PxgCudaBuffer		mRemoveHistogram;

						PxgCudaBuffer		mAggregatedBoundsBuf;
						PxgCudaBuffer		mAddedHandleBuf;
						PxgCudaBuffer		mRemovedHandleBuf;
						PxgCudaBuffer		mChangedAABBMgrHandlesBuf;

						PxU32				mNumAggregatesSlots;

						PxU32				mMaxFoundLostPairs;
						PxU32				mMaxAggPairs;

					PxgProcessFoundPairTask	mFoundPairTask;
					PxgProcessLostPairTask	mLostPairTask;

						// PT: this flag is set:
						// - via setGPUStateChanged():
						//     - in PxgSimulationCore::applyActorData() when body data is changed 
						//     - in FEMClothShapeSim::updateBoundsInAABBMgr()
						//     - in ParticleSystemShapeSim::updateBoundsInAABBMgr
						//     - in SoftBodyShapeSim::updateBoundsInAABBMgr()
						// - when mOriginShifted is false and hasActive is true in PxgAABBManager::updateBPFirstPass
						//
						// It is unclear whether we need to set the flag in all of these cases.
						//
						// The flag is used:
						// - in PxgAABBManager::preBpUpdate_GPU
						//     - to skip DMA of mChangedHandleMap
						// - in PxgAABBManager::updateBPSecondPass
						//     - in the GPU broadphase (passed there as part of the updateData structure)
						//         - TODO: investigate what happens there
						//     - to skip kernel launches of AGG_ADD_AGGPAIRS_STAGE_1/AGG_ADD_AGGPAIRS_STAGE_2
						bool				mGPUStateChanged; //non-rigid body, direct API calls

						// PT: this flag is set:
						// - via setPersistentStateChanged():
						//     - when a contact distance changes
						// - when an aggregate is removed (PxgAABBManager::destroyAggregate)
						// - when an object is added (PxgAABBManager::addBounds)
						// - when an aggregated is added (PxgAABBManager::addBounds)
						// - when an object is removed (PxgAABBManager::removeBounds)
						// - when an aggregated is removed (PxgAABBManager::removeBounds)
						// - in PxgAABBManager::updateBPFirstPass if hasContactDistanceUpdated is true
						// - when mOriginShifted is true, during PxgAABBManager::updateBPFirstPass
						//
						// It is unclear whether we need to set the flag in all of these cases.
						//
						// The flag is used:
						// - in PxgAABBManager::preBpUpdate_GPU
						//     - to skip DMAs of mAddedHandleMap/mRemovedHandleMap
						//     - to skip DMA of mAggregatedBoundMap / mVolumeData
						//     - to skip the preBroadphase call
						//     - to skip DMA of mChangedHandleMap
						// - in PxgAABBManager::updateBPSecondPass
						//     - in the GPU broadphase (passed there as part of the updateData structure)
						//         - TODO: investigate what happens there
						//     - to skip kernel launches of AGG_ADD_AGGPAIRS_STAGE_1/AGG_ADD_AGGPAIRS_STAGE_2
						bool				mPersistentStateChanged;
	};

	class PxgBoundsArray : public Bp::BoundsArray
	{
				  PX_NOCOPY(PxgBoundsArray)

				  public:
					  PxgBoundsArray(PxVirtualAllocator& allocator)
						: BoundsArray(allocator)
						, mIsFirstCopy(true)
						, mChanges(allocator)
						{
						}

						virtual ~PxgBoundsArray() PX_OVERRIDE
						{
							mChanges.clear();
						}

						void updateBounds(const PxTransform& transform, const PxGeometry& geom, PxU32 index,
										  PxU32 indexFrom) PX_OVERRIDE PX_FINAL 
						{	
							if(indexFrom == index) // new, needs to be copied from CPU
							{
								Gu::computeBounds(mBounds[index], geom, transform, 0.0f, 1.0f);
								updateChanges(index, indexFrom, true);
							}
							else
							{
								updateChanges(index, indexFrom, false);
							}							
						}
 
						void setBounds(const PxBounds3& bounds, PxU32 index) PX_OVERRIDE PX_FINAL
						{															
							mBounds[index] = bounds;						
						    updateChanges(index, index, true);
						}

						PX_FORCE_INLINE PxU32 getNumberOfChanges()
						{ 
							return mChanges.size();
						}

						PX_FORCE_INLINE PxArray<PxBoundTransformUpdate, PxVirtualAllocator>& getStagingBuffer()
						{					
							return mChanges;
						}

						PX_FORCE_INLINE void resetChanges() 
						{ 
							mChanges.reset();
						}

						PX_FORCE_INLINE bool isFirstCopy() 
						{ 
							return mIsFirstCopy;
						} 

						PX_FORCE_INLINE void setCopied() 
						{ 
							mIsFirstCopy = false; 
						}

				  private:
						PX_FORCE_INLINE void updateChanges(PxU32 indexTo, PxU32 indexFrom, bool isNew)
						{
							if(!mIsFirstCopy)
							{
									PxBoundTransformUpdate update;
									update.indexTo = indexTo;
									update.indexFrom = indexFrom & 0x7FFFFFFF; 
									if(isNew)
										{
											update.indexFrom |= (1U << 31);
										}
									mChanges.pushBack(update);
							}
						}

						bool mIsFirstCopy;
						PxArray<PxBoundTransformUpdate, PxVirtualAllocator> mChanges;
	};
}

#endif
