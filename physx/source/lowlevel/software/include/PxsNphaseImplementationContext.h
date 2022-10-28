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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_NPHASE_IMPLEMENTATION_CONTEXT_H
#define PXS_NPHASE_IMPLEMENTATION_CONTEXT_H

#include "PxvNphaseImplementationContext.h" 
#include "PxsContactManagerState.h"
#include "PxcNpCache.h"
#include "foundation/PxPinnedArray.h"

class PxsCMDiscreteUpdateTask;

namespace physx
{

struct PxsContactManagers : PxsContactManagerBase
{
	PxArray<PxsContactManagerOutput>		mOutputContactManagers;
	PxArray<PxsContactManager*>				mContactManagerMapping;
	PxArray<Gu::Cache>						mCaches;
	PxPinnedArray<Sc::ShapeInteraction*>	mShapeInteractions;
	PxFloatArrayPinned						mRestDistances;
	PxPinnedArray<PxsTorsionalFrictionData>	mTorsionalProperties;

	PxsContactManagers(const PxU32 bucketId, PxVirtualAllocatorCallback* callback) : PxsContactManagerBase(bucketId),
		mOutputContactManagers	("mOutputContactManagers"),
		mContactManagerMapping	("mContactManagerMapping"),
		mCaches					("mCaches"),
		mShapeInteractions		(PxVirtualAllocator(callback)),
		mRestDistances			(callback),
		mTorsionalProperties	(callback)
	{
	}
		
	void clear()
	{
		mOutputContactManagers.forceSize_Unsafe(0);
		mContactManagerMapping.forceSize_Unsafe(0);
		mCaches.forceSize_Unsafe(0);
		mShapeInteractions.forceSize_Unsafe(0);
		mRestDistances.forceSize_Unsafe(0);
		mTorsionalProperties.forceSize_Unsafe(0);
		
	}
private:
	PX_NOCOPY(PxsContactManagers)
};

class PxsNphaseImplementationContext: public PxvNphaseImplementationContextUsableAsFallback
{
public:
	static PxsNphaseImplementationContext*	create(PxsContext& context, IG::IslandSim* islandSim, PxVirtualAllocatorCallback* allocator);

											PxsNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, PxVirtualAllocatorCallback* callback, PxU32 index = 0) :
											PxvNphaseImplementationContextUsableAsFallback	(context), 
											mNarrowPhasePairs								(index, callback), 
											mNewNarrowPhasePairs							(index, callback),
											mModifyCallback									(NULL),
											mIslandSim(islandSim)							{}

	virtual void							destroy();
	virtual void							updateContactManager(PxReal dt, bool hasBoundsArrayChanged, bool hasContactDistanceChanged, PxBaseTask* continuation, 
																PxBaseTask* firstPassContinuation, Cm::FanoutTask* updateBoundAndShape);
	virtual void							postBroadPhaseUpdateContactManager(PxBaseTask*) {}
	virtual void							secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation);

	virtual void							registerContactManager(PxsContactManager* cm, Sc::ShapeInteraction* shapeInteraction, PxI32 touching, PxU32 numPatches);
//	virtual void							registerContactManagers(PxsContactManager** cm, Sc::ShapeInteraction** shapeInteractions, PxU32 nbContactManagers, PxU32 maxContactManagerId);
	virtual void							unregisterContactManager(PxsContactManager* cm);
	virtual void							unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs);

	virtual void							refreshContactManager(PxsContactManager* cm);
	virtual void							refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs);

	virtual void							registerShape(const PxNodeIndex& /*nodeIndex*/, const PxsShapeCore& /*shapeCore*/, const PxU32 /*transformCacheID*/, PxActor* /*actor*/, const bool /*isFemCloth*/) {}
	virtual void							unregisterShape(const PxsShapeCore& /*shapeCore*/, const PxU32 /*transformCacheID*/, const bool /*isFemCloth*/)									{}

	virtual void							registerAggregate(const PxU32 /*transformCacheID*/)		{}
	
	virtual void							updateShapeMaterial(const PxsShapeCore&)				{}
	virtual void							updateShapeContactOffset(const PxsShapeCore&)			{}

	virtual void							registerMaterial(const PxsMaterialCore&)				{}
	virtual void							updateMaterial(const PxsMaterialCore&)					{}
	virtual void							unregisterMaterial(const PxsMaterialCore&)				{}

	virtual void							registerMaterial(const PxsFEMSoftBodyMaterialCore&)		{}
	virtual void							updateMaterial(const PxsFEMSoftBodyMaterialCore&)		{}
	virtual void							unregisterMaterial(const PxsFEMSoftBodyMaterialCore&)	{}

	virtual void							registerMaterial(const PxsFEMClothMaterialCore&)		{}
	virtual void							updateMaterial(const PxsFEMClothMaterialCore&)			{}
	virtual void							unregisterMaterial(const PxsFEMClothMaterialCore&)		{}

	virtual void							registerMaterial(const PxsPBDMaterialCore&)				{}
	virtual void							updateMaterial(const PxsPBDMaterialCore&)				{}
	virtual void							unregisterMaterial(const PxsPBDMaterialCore&)			{}

	virtual void							registerMaterial(const PxsFLIPMaterialCore&)			{}
	virtual void							updateMaterial(const PxsFLIPMaterialCore&)				{}
	virtual void							unregisterMaterial(const PxsFLIPMaterialCore&)			{}

	virtual void							registerMaterial(const PxsMPMMaterialCore&)				{}
	virtual void							updateMaterial(const PxsMPMMaterialCore&)				{}
	virtual void							unregisterMaterial(const PxsMPMMaterialCore&)			{}

	virtual void							registerMaterial(const PxsCustomMaterialCore&)			{}
	virtual void							updateMaterial(const PxsCustomMaterialCore&)			{}
	virtual void							unregisterMaterial(const PxsCustomMaterialCore&)		{}

	virtual void							appendContactManagers();
	virtual void							appendContactManagersFallback(PxsContactManagerOutput* cmOutputs);

	virtual void							removeContactManagersFallback(PxsContactManagerOutput* cmOutputs);

	virtual void							setContactModifyCallback(PxContactModifyCallback* callback) { mModifyCallback = callback; }

	virtual PxsContactManagerOutputIterator getContactManagerOutputs();

	virtual PxsContactManagerOutput&		getNewContactManagerOutput(PxU32 npIndex);

	virtual void							acquireContext(){}
	virtual void							releaseContext(){}
	virtual void							preallocateNewBuffers(PxU32 /*nbNewPairs*/, PxU32 /*maxIndex*/) { /*TODO - implement if it's useful to do so*/}

	virtual PxsContactManagerOutputCounts*	getFoundPatchOutputCounts() { return mCmFoundLostOutputCounts.begin(); }
	virtual PxsContactManager**				getFoundPatchManagers() { return mCmFoundLost.begin(); }
	virtual PxU32							getNbFoundPatchManagers() { return mCmFoundLost.size(); }

			void							processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation);
			void							processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation);
			void							fetchUpdateContactManager() {}

			void							appendNewLostPairs();

			void							startNarrowPhaseTasks() {}

	virtual void							lock()		{ mContactManagerMutex.lock();		}
	virtual void							unlock()	{ mContactManagerMutex.unlock();	}

	virtual Sc::ShapeInteraction**			getShapeInteractions()	{ return mNarrowPhasePairs.mShapeInteractions.begin();	}
	virtual PxReal*							getRestDistances()		{ return mNarrowPhasePairs.mRestDistances.begin();		}
	virtual PxsTorsionalFrictionData*		getTorsionalData()		{ return mNarrowPhasePairs.mTorsionalProperties.begin(); }

	virtual PxsContactManagerOutput*		getGPUContactManagerOutputBase()	{ return NULL; }
	virtual PxReal*							getGPURestDistances()				{ return NULL; }
	virtual Sc::ShapeInteraction**			getGPUShapeInteractions()			{ return NULL; }
	virtual PxsTorsionalFrictionData*		getGPUTorsionalData()				{ return NULL; }
	
			PxArray<PxU32>					mRemovedContactManagers;
			PxsContactManagers				mNarrowPhasePairs;
			PxsContactManagers				mNewNarrowPhasePairs;

			PxContactModifyCallback*		mModifyCallback;

			IG::IslandSim*					mIslandSim;

			PxMutex							mContactManagerMutex;

			PxArray<PxsCMDiscreteUpdateTask*> mCmTasks;

			PxArray<PxsContactManagerOutputCounts> mCmFoundLostOutputCounts;
			PxArray<PxsContactManager*>		mCmFoundLost;
private:
			void							unregisterContactManagerInternal(PxU32 npIndex, PxsContactManagers& managers, PxsContactManagerOutput* cmOutputs);

			PX_FORCE_INLINE void			unregisterAndForceSize(PxsContactManagers& cms, PxU32 index)
			{
				unregisterContactManagerInternal(index, cms, cms.mOutputContactManagers.begin());
				cms.mOutputContactManagers.forceSize_Unsafe(cms.mOutputContactManagers.size()-1);
			}

	PX_NOCOPY(PxsNphaseImplementationContext)
};

}

#endif
