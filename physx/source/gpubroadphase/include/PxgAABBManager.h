// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_AABBMANAGER_H
#define PXG_AABBMANAGER_H

#include "BpAABBManagerBase.h"
#include "PxgCudaBuffer.h"
#include "PxgAggregate.h"
#include "PxgAggregateBuffer.h"
#include "PxgAggregateDesc.h"
#include "PxgBoundTransformUpdate.h"
#include "PxgBroadPhasePairReport.h"
#include "CmIDPool.h"
#include "CmTask.h"
#include "CmPinnableArray.h"
#include "CmPinnableBitMap.h"
#include "CmPinnableObject.h"
#include "PxsHeapStats.h"
#include "foundation/PxHashMap.h"

namespace physx
{
	class PxCudaContextManager;

	class PxgCudaKernelWranglerManager;
	class PxgCudaBroadPhaseSap;
	class PxgAABBManager;
	class PxgHeapMemoryAllocator;
	struct PxgAllocatorDesc;
	struct PxGpuDynamicsMemoryConfig;

	class PxgProcessFoundPairTask : public Cm::Task 
	{
		PxgAABBManager* mManager;
	public:
		PxgProcessFoundPairTask(PxgAABBManager* manager) : Cm::Task(0), mManager(manager)
		{
		}

		virtual void runInternal() PX_OVERRIDE;
		virtual const char* getName() const PX_OVERRIDE { return "PxgProcessFoundPairTask"; }
	};

	class PxgProcessLostPairTask : public Cm::Task
	{
		PxgAABBManager* mManager;
	public:
		PxgProcessLostPairTask(PxgAABBManager* manager) : Cm::Task(0), mManager(manager)
		{
		}

		virtual void runInternal() PX_OVERRIDE;
		virtual const char* getName() const PX_OVERRIDE { return "PxgProcessLostPairTask"; }
	};
	
	class PxgAABBManager : public Bp::AABBManagerBase
	{
	public:
											PxgAABBManager(PxgCudaKernelWranglerManager* gpuKernelWrangler,
												PxCudaContextManager* cudaContextManager,
												PxgAllocatorDesc& allocDesc,
												const PxGpuDynamicsMemoryConfig& config,
												Bp::BroadPhase& bp, Bp::BoundsArray& boundsArray, Cm::PinnableArray<PxReal>& contactDistance,
												PxU32 maxNbAggregates, PxU32 maxNbShapes,
												PxU64 contextID, PxPairFilteringMode::Enum kineKineFilteringMode, 
												PxPairFilteringMode::Enum staticKineFilteringMode);

		virtual								~PxgAABBManager() {}

		// AABBManagerBase
		virtual			void				destroy()	PX_OVERRIDE;
		virtual			Bp::AggregateHandle	createAggregate(Bp::BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint, PxU32 envID)	PX_OVERRIDE;
		virtual			bool				destroyAggregate(Bp::BoundsIndex& index, Bp::FilterGroup::Enum& group, Bp::AggregateHandle aggregateHandle)	PX_OVERRIDE;
		virtual			bool				addBounds(Bp::BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData, Bp::AggregateHandle aggregateHandle, Bp::ElementType::Enum volumeType, PxU32 envID)	PX_OVERRIDE;
		virtual			bool				removeBounds(Bp::BoundsIndex index)	PX_OVERRIDE;
		virtual			void				updateBPFirstPass(PxU32 numCpuTasks, Cm::FlushPool& flushPool, bool hasContactDistanceUpdated, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual			void				updateBPSecondPass(PxU32 numCpuTasks, Cm::FlushPool& flushPool, PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual			void				postBroadPhase(PxBaseTask*, Cm::FlushPool& flushPool)	PX_OVERRIDE;
		virtual			void				reallocateChangedAABBMgActorHandleMap(const PxU32 size)	PX_OVERRIDE;
		virtual			void				visualize(PxRenderOutput& out)	PX_OVERRIDE;
		virtual			void				releaseDeferredAggregateIds()	PX_OVERRIDE;
		virtual			void				setGPUStateChanged()			PX_OVERRIDE	{ mGPUStateChanged = true;			}
		virtual			void				setPersistentStateChanged()		PX_OVERRIDE	{ mPersistentStateChanged = true;	}
		virtual			bool				isGPUManager()		const		PX_OVERRIDE	{ return true;						}
		//~AABBManagerBase

						void				markAggregateBoundsBitmap();

						void				processFoundPairs();
						void				processLostPairs();

		PX_FORCE_INLINE	CUdeviceptr			getAggregatedBounds()				{ return mAggregatedBoundsBuf.getDevicePtr(); }
		PX_FORCE_INLINE	CUdeviceptr			getAddedHandles()					{ return mAddedHandleBuf.getDevicePtr(); }
		PX_FORCE_INLINE	CUdeviceptr			getRemovedHandles()					{ return mRemovedHandleBuf.getDevicePtr(); }
		PX_FORCE_INLINE	CUdeviceptr			getChangedAABBMgrHandles()			{ return mChangedAABBMgrHandlesBuf.getDevicePtr(); }
		PX_FORCE_INLINE	PxU32				getChangedAABBMgrHandlesWordCount()	{ return mChangedHandleMap.getWordCount(); }
		PX_FORCE_INLINE	PxU32				getChangedAABBMgrHandlesDeviceWordCount()	{ return PxU32(mChangedAABBMgrHandlesBuf.getSize() / sizeof(PxU32)); }
		PX_FORCE_INLINE CUdeviceptr			getVolumeData()						{ return mVolumeDataBuf.getDevicePtr(); }

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
		PxgHeapMemoryAllocator&				mDeviceAlloc;

		PxArray<PxgAggregate>				mAggregates;				//cpu mirror

		Cm::PinnableArray<PxgAggregatePair>		mAggregatePairs;
		Cm::PinnableArray<Bp::AggregateHandle>	mDirtyAggregateIndices;
		Cm::PinnableArray<PxgAggregate>			mDirtyAggregates;

		//found and lost pairs for agg vs agg and agg vs actor, need to be device mapped memory
		Cm::PinnableArray<PxgBroadPhasePair>	mFoundPairsMapped;
		Cm::PinnableArray<PxgBroadPhasePair>	mLostPairsMapped;

		Cm::PinnableArray<PxU32>				mDirtyBoundIndices;
		Cm::PinnableArray<PxU32>				mDirtyBoundStartIndices; //start index of each aggregate in the dirty bound indices list

		Cm::PinnableArray<PxU32>				mRemovedAggregatedBounds;
		Cm::PinnableArray<PxU32>				mAddedAggregatedBounds;

		Cm::DeferredIDPool						mAggregatesIdPool; //generate the remap id between pxgbodysim and pxgaggregate
		PxBitMap								mDirtyAggregateBitMap;
		Cm::PinnableBitMap						mAggregatedBoundMap;

		PxArray<PxgAggregateBuffer*>			mAggregateBufferArray;

		Cm::PinnableObject<PxgAggregateDesc>	mAggregateDesc;
		
		PxgCudaBuffer						mVolumeDataBuf;

		PxgCudaBuffer						mAggregateBuf;
		PxgCudaBuffer						mAggregatePairsBuf;
		
		PxgCudaBuffer						mDirtyAggregateIndiceBuf;
		PxgCudaBuffer						mDirtyAggregateBuf;

		PxgCudaBuffer						mDirtyBoundIndicesBuf;
		PxgCudaBuffer						mDirtyBoundStartIndicesBuf;

		PxgCudaBuffer						mRemovedAggregatedBoundsBuf;
		PxgCudaBuffer						mAddedAggregatedBoundsBuf;
		
		PxgCudaBuffer						mAggPairBuf;	//persistent pairs
		PxgCudaBuffer						mNumAggPairBuf; // number of app pairs
		PxgCudaBuffer						mAggregateDescBuf;

		PxgCudaBuffer						mFoundPairsBuf;
		PxgCudaBuffer						mLostPairsBuf;

		PxgCudaBuffer						mFreeIDPool;
		PxgCudaBuffer						mFreeIDs;

		PxgCudaBuffer						mRemoveBitmap;
		PxgCudaBuffer						mRemoveHistogram;

		PxgCudaBuffer						mAggregatedBoundsBuf;
		PxgCudaBuffer						mAddedHandleBuf;
		PxgCudaBuffer						mRemovedHandleBuf;
		PxgCudaBuffer						mChangedAABBMgrHandlesBuf;

		PxU32								mNumAggregatesSlots;

		PxU32								mMaxFoundLostPairs;
		PxU32								mMaxAggPairs;

		PxgProcessFoundPairTask				mFoundPairTask;
		PxgProcessLostPairTask				mLostPairTask;

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
		bool								mGPUStateChanged; //non-rigid body, direct API calls

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
		bool								mPersistentStateChanged;
	};

	class PxgBoundsArray : public Bp::BoundsArray
	{
	PX_NOCOPY(PxgBoundsArray)

	public:
	PxgBoundsArray(Cm::VirtualAllocatorCallback& mappedAllocator)
	  : BoundsArray(mappedAllocator, PxsHeapStats::eBROADPHASE, Cm::PinnableAllocatorFallback::eDISABLED)
	  , mEnableChangeTracking(false)
	  , mChangesMapped(mappedAllocator, PxsHeapStats::eBROADPHASE, Cm::PinnableAllocatorFallback::eDISABLED)
	{}

		virtual ~PxgBoundsArray() PX_OVERRIDE
		{
			mChangesMapped.clear();
		}

		// Called at the insertion and re-insertion of an element, also when a re-insertion keeps the element ID. Changes of an inserted
		// element, such as a local pose change, are marked with markForUpdate() instead.
		// PxgSimulationController::mergeBoundsAndTransformsChanges() applies the marks at the start of the next simulation step.
		// - isNew: the actor of the element is new, it has not been uploaded to the GPU yet, so the GPU has no pose of it. The bounds
		//   are computed here from transform and geom and copied to the GPU together with the transform.
		// - isDynamicRigidShape: the element is a shape of a rigid dynamic or an articulation link. If it is not new, its bounds and
		//   transform are computed on the GPU from the pose of its actor there, as for markForUpdate(), because the CPU pose of a dynamic
		//   rigid is not updated from the GPU. This way a shape attached or re-inserted after the actor moved on the GPU is placed there.
		// Any other element is copied from the CPU.
		virtual void updateBounds(const PxTransform& transform, const PxGeometry& geom, PxU32 index, bool isNew, bool isDynamicRigidShape) PX_OVERRIDE PX_FINAL
		{
			if(mAllocFailed)
				return;

			if(!isNew && isDynamicRigidShape)
			{
				updateChanges(index, PxgBoundTransformUpdate::eCOMPUTE_FROM_GPU_POSE);
			}
			else
			{
				Gu::computeBounds(mBounds[index], geom, transform, 0.0f, 1.0f);
				updateChanges(index, PxgBoundTransformUpdate::eCOPY_FROM_CPU);
			}
		}
 
		virtual void setBounds(const PxBounds3& bounds, PxU32 index) PX_OVERRIDE PX_FINAL
		{
			if(mAllocFailed)
				return;

			mBounds[index] = bounds;
			updateChanges(index, PxgBoundTransformUpdate::eCOPY_FROM_CPU);
		}

		// The copy is on the GPU. The shape of a dynamic rigid is computed there from the GPU pose of its actor, because the CPU
		// pose of a dynamic rigid is not updated from the GPU.
		virtual void markForUpdate(PxU32 index, bool isDynamicRigidShape) PX_OVERRIDE PX_FINAL
		{
			if(mAllocFailed)
			{
				return;
			}

			// An element added or re-inserted in this step already takes its current bounds and transform.
			if(mChangeMap.find(index))
			{
				return;
			}

			updateChanges(index, isDynamicRigidShape ? PxgBoundTransformUpdate::eCOMPUTE_FROM_GPU_POSE : PxgBoundTransformUpdate::eCOPY_FROM_CPU);
		}

		PX_FORCE_INLINE PxU32 getNumberOfChanges() const
		{ 
			return mChangesMapped.size();
		}

		PX_FORCE_INLINE const PxgBoundTransformUpdate* getStagingBuffer()
		{
			return mChangesMapped.begin();
		}

		PX_FORCE_INLINE void resetChanges() 
		{ 
			mChangesMapped.clear();
			mChangeMap.clear();
		}

		virtual bool isChangeTrackingEnabled() const PX_OVERRIDE PX_FINAL
		{ 
			return mEnableChangeTracking;
		} 

		/*
		* Returns whether change tracking has been successfully enabled 
		* i.e. pinned change buffer successfully allocated.
		*/
		PX_FORCE_INLINE bool enableChangeTracking()
		{
			if (!mEnableChangeTracking)
			{
				bool allocated = mChangesMapped.reserve(mBounds.size());
				mEnableChangeTracking = allocated;
				mAllocFailed |= !allocated;
				return allocated;
			}
			return true;
		}

	private:

		virtual bool resize(PxU32 size) PX_OVERRIDE PX_FINAL
		{
			//PxgBoundsArray is only used with directAPI,
			//so mBounds and mChangesMapped must both be allocated as
			//pinned device mapped memory (without pageable fallback).
			if(mBounds.reserve(size))
			{
				mBounds.forceSize_Unsafe(size);
				if(mEnableChangeTracking && !mChangesMapped.reserve(size))
				{
					mAllocFailed = true;
					return false;
				}
				return true;
			}
			mAllocFailed = true;
			return false;
		}

		PX_FORCE_INLINE void updateChanges(PxU32 index, PxgBoundTransformUpdate::Kind kind)
		{
			PX_ASSERT(!mAllocFailed);
			if(mEnableChangeTracking)
			{
				PxgBoundTransformUpdate update;
				update.index = (index & PxgBoundTransformUpdate::eINDEX_MASK) | kind;

				//Uniquify changes, last one wins.
				const PxHashMap<PxU32, PxU32>::Entry* e = mChangeMap.find(index);
				if(e)
				{
					PxgBoundTransformUpdate& oldUpdate = mChangesMapped[e->second];
					oldUpdate = update;
				}
				else
				{
					// elementIDs definitely fit in mChanges, which is resized with bounds array
					PX_ASSERT(mChangesMapped.capacity() > mChangesMapped.size());
					mChangeMap.insert(index, mChangesMapped.size());
					mChangesMapped.pushBack(update);
				}
			}
		}

		bool mEnableChangeTracking;
		Cm::PinnableArray<PxgBoundTransformUpdate> mChangesMapped;
		PxHashMap<PxU32, PxU32> mChangeMap;
	};
}

#endif
