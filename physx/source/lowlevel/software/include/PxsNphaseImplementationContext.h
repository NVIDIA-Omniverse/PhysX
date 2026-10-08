// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_NPHASE_IMPLEMENTATION_CONTEXT_H
#define PXS_NPHASE_IMPLEMENTATION_CONTEXT_H

#include "PxvNphaseImplementationContext.h" 
#include "PxsContactManagerState.h"
#include "PxcNpCache.h"
#include "CmPinnableArray.h"

class PxsCMDiscreteUpdateTask;

namespace physx
{

struct PxsContactManagers : PxsContactManagerBase
{
	PxArray<PxsContactManagerOutput>	mOutputContactManagers;
	PxArray<PxsContactManager*>			mContactManagerMapping;
	PxArray<Gu::Cache>					mCaches;

	// PT: these buffers should be in pinned memory but may not be if pinned allocation failed.
	Cm::PinnableArray<const Sc::ShapeInteraction*>	mShapeInteractionsGPU;
	Cm::PinnableArray<PxReal>						mRestDistancesGPU;
	Cm::PinnableArray<PxsTorsionalFrictionData>		mTorsionalPropertiesGPU;

	PxsContactManagers(const PxU32 bucketId, Cm::VirtualAllocatorCallback& alloc) : PxsContactManagerBase(bucketId),
		mOutputContactManagers	("mOutputContactManagers"),
		mContactManagerMapping	("mContactManagerMapping"),
		mCaches					("mCaches"),
		mShapeInteractionsGPU	(alloc, PxsHeapStats::eNARROWPHASE),
		mRestDistancesGPU		(alloc, PxsHeapStats::eNARROWPHASE),
		mTorsionalPropertiesGPU	(alloc, PxsHeapStats::eNARROWPHASE)
	{
	}
		
	void clear()
	{
		mOutputContactManagers.forceSize_Unsafe(0);
		mContactManagerMapping.forceSize_Unsafe(0);
		mCaches.forceSize_Unsafe(0);
		mShapeInteractionsGPU.forceSize_Unsafe(0);
		mRestDistancesGPU.forceSize_Unsafe(0);
		mTorsionalPropertiesGPU.forceSize_Unsafe(0);
	}
private:
	PX_NOCOPY(PxsContactManagers)
};

class PxsNphaseImplementationContext : public PxvNphaseImplementationFallback
{
	PX_NOCOPY(PxsNphaseImplementationContext)
public:
											PxsNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, Cm::VirtualAllocatorCallback& alloc, PxU32 index, bool gpu) :
											PxvNphaseImplementationFallback	(context), 
											mNarrowPhasePairs				(index, alloc), 
											mNewNarrowPhasePairs			(index, alloc),
											mModifyCallback					(NULL),
											mIslandSim						(islandSim),
											mGPU							(gpu)
											{}

	// PxvNphaseImplementationContext
	virtual void							destroy()	PX_OVERRIDE	PX_FINAL;
	virtual void							updateContactManager(PxReal dt, bool hasContactDistanceChanged, PxBaseTask* continuation, 
																PxBaseTask* firstPassContinuation, Cm::FanoutTask* updateBoundAndShape)	PX_OVERRIDE	PX_FINAL;
	virtual void							postBroadPhaseUpdateContactManager(PxBaseTask*) PX_OVERRIDE	PX_FINAL	{}
	virtual void							secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)	PX_OVERRIDE	PX_FINAL;
	virtual void							fetchUpdateContactManager() PX_OVERRIDE	PX_FINAL	{}
	virtual void							registerContactManager(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxI32 touching, PxU32 numPatches)	PX_OVERRIDE	PX_FINAL;
//	virtual void							registerContactManagers(PxsContactManager** cm, Sc::ShapeInteraction** shapeInteractions, PxU32 nbContactManagers, PxU32 maxContactManagerId);
	virtual void							unregisterContactManager(PxsContactManager* cm)	PX_OVERRIDE	PX_FINAL;
	virtual void							refreshContactManager(PxsContactManager* cm)	PX_OVERRIDE	PX_FINAL;

	virtual void							registerShape(const PxNodeIndex& /*nodeIndex*/, const PxsShapeCore& /*shapeCore*/, const PxU32 /*transformCacheID*/, PxActor* /*actor*/, const bool /*isDeformableSurface*/) PX_OVERRIDE	PX_FINAL	{}
	virtual void							unregisterShape(const PxsShapeCore& /*shapeCore*/, const PxU32 /*transformCacheID*/, const bool /*isDeformableSurface*/)	PX_OVERRIDE	PX_FINAL		{}

	virtual void							registerAggregate(const PxU32 /*transformCacheID*/)		PX_OVERRIDE	PX_FINAL	{}

	virtual void							registerMaterial(const PxsMaterialCore&)				PX_OVERRIDE	PX_FINAL	{}
	virtual void							updateMaterial(const PxsMaterialCore&)					PX_OVERRIDE	PX_FINAL	{}
	virtual void							unregisterMaterial(const PxsMaterialCore&)				PX_OVERRIDE	PX_FINAL	{}

	virtual void							registerMaterial(const PxsDeformableSurfaceMaterialCore&)		PX_OVERRIDE	PX_FINAL	{}
	virtual void							updateMaterial(const PxsDeformableSurfaceMaterialCore&)			PX_OVERRIDE	PX_FINAL	{}
	virtual void							unregisterMaterial(const PxsDeformableSurfaceMaterialCore&)		PX_OVERRIDE	PX_FINAL	{}

	virtual void							registerMaterial(const PxsDeformableVolumeMaterialCore&)		PX_OVERRIDE	PX_FINAL	{}
	virtual void							updateMaterial(const PxsDeformableVolumeMaterialCore&)			PX_OVERRIDE	PX_FINAL	{}
	virtual void							unregisterMaterial(const PxsDeformableVolumeMaterialCore&)		PX_OVERRIDE	PX_FINAL	{}

	virtual void							registerMaterial(const PxsPBDMaterialCore&)				PX_OVERRIDE	PX_FINAL	{}
	virtual void							updateMaterial(const PxsPBDMaterialCore&)				PX_OVERRIDE	PX_FINAL	{}
	virtual void							unregisterMaterial(const PxsPBDMaterialCore&)			PX_OVERRIDE	PX_FINAL	{}

	virtual void							updateShapeMaterial(const PxsShapeCore&)				PX_OVERRIDE	PX_FINAL	{}

	virtual void							startNarrowPhaseTasks()									PX_OVERRIDE	PX_FINAL	{}

	virtual void							appendContactManagers()	PX_OVERRIDE	PX_FINAL;

	virtual PxsContactManagerOutput&		getNewContactManagerOutput(PxU32 npIndex)	PX_OVERRIDE	PX_FINAL;

	virtual PxsContactManagerOutputIterator getContactManagerOutputs()	PX_OVERRIDE	PX_FINAL;
	virtual void							setContactModifyCallback(PxContactModifyCallback* callback) PX_OVERRIDE	PX_FINAL	{ mModifyCallback = callback; }

	virtual void							acquireContext()	PX_OVERRIDE	PX_FINAL	{}
	virtual void							releaseContext()	PX_OVERRIDE	PX_FINAL	{}
	virtual void							preallocateNewBuffers(PxU32 /*nbNewPairs*/, PxU32 /*maxIndex*/) PX_OVERRIDE	PX_FINAL	{ /*TODO - implement if it's useful to do so*/}
	virtual void							lock()		PX_OVERRIDE	PX_FINAL	{ mContactManagerMutex.lock();		}
	virtual void							unlock()	PX_OVERRIDE	PX_FINAL	{ mContactManagerMutex.unlock();	}

	virtual PxsContactManagerOutputCounts*	getLostFoundPatchOutputCounts()	PX_OVERRIDE	PX_FINAL	{ return mGPU ? mGPU_CmFoundLostOutputCounts.begin() : NULL; }
	virtual PxsContactManager**				getLostFoundPatchManagers()		PX_OVERRIDE	PX_FINAL	{ return mGPU ? mGPU_CmFoundLost.begin() : NULL; }
	virtual PxU32							getNbLostFoundPatchManagers()	PX_OVERRIDE	PX_FINAL	{ return mGPU ? mGPU_CmFoundLost.size() : 0; }

	virtual PxsContactManagerOutput*		getGPUContactManagerOutputBase()	PX_OVERRIDE	PX_FINAL	{ return NULL; }
	virtual PxReal*							getGPURestDistances()				PX_OVERRIDE	PX_FINAL	{ return NULL; }
	virtual Sc::ShapeInteraction**			getGPUShapeInteractions()			PX_OVERRIDE	PX_FINAL	{ return NULL; }
	virtual PxsTorsionalFrictionData*		getGPUTorsionalData()				PX_OVERRIDE	PX_FINAL	{ return NULL; }
	//~PxvNphaseImplementationContext

	// PxvNphaseImplementationFallback
	virtual	void							processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation)	PX_OVERRIDE	PX_FINAL;
	virtual	void							processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation)	PX_OVERRIDE	PX_FINAL;
	virtual void							unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE	PX_FINAL;
	virtual void							refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE	PX_FINAL;
	virtual void							appendContactManagersFallback(PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE	PX_FINAL;
	virtual void							removeContactManagersFallback(PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE	PX_FINAL;
	virtual const Sc::ShapeInteraction*const*	getShapeInteractionsGPU()	const	PX_OVERRIDE	PX_FINAL	{ return mNarrowPhasePairs.mShapeInteractionsGPU.begin();	}
	virtual const PxReal*						getRestDistancesGPU()		const	PX_OVERRIDE	PX_FINAL	{ return mNarrowPhasePairs.mRestDistancesGPU.begin();		}
	virtual const PxsTorsionalFrictionData*		getTorsionalDataGPU()		const	PX_OVERRIDE	PX_FINAL	{ return mNarrowPhasePairs.mTorsionalPropertiesGPU.begin();	}
	//~PxvNphaseImplementationFallback

			PxArray<PxU32>					mRemovedContactManagers;
			PxsContactManagers				mNarrowPhasePairs;
			PxsContactManagers				mNewNarrowPhasePairs;

			PxContactModifyCallback*		mModifyCallback;

			IG::IslandSim*					mIslandSim;

			PxMutex							mContactManagerMutex;

			PxArray<PxsCMDiscreteUpdateTask*> mGPU_CmTasks;
			PxArray<PxsContactManagerOutputCounts> mGPU_CmFoundLostOutputCounts;
			PxArray<PxsContactManager*>		mGPU_CmFoundLost;

			const bool						mGPU;
private:
			//Returns false when npIndex named no live pair and nothing was removed.
			bool							unregisterContactManagerInternal(PxU32 npIndex, PxsContactManagers& managers, PxsContactManagerOutput* cmOutputs);

			PX_FORCE_INLINE void			unregisterAndForceSize(PxsContactManagers& cms, PxU32 index)
			{
				//Only shrink when something was actually removed. unregisterContactManagerInternal() bails out on
				//the sentinel and on an out-of-range index, and shrinking regardless underflowed an empty list's
				//size to 0xFFFFFFFF - which the next registerContactManager() turns into a four-billion-element
				//copy inside PxArray::recreate().
				// ### DEFENSIVE (NvBug 6163965)
				if(unregisterContactManagerInternal(index, cms, cms.mOutputContactManagers.begin()))
					cms.mOutputContactManagers.forceSize_Unsafe(cms.mOutputContactManagers.size()-1);
			}

			void							appendNewLostPairs();
};

}

#endif
