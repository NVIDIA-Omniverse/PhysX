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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
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

class PxsNphaseImplementationContext : public PxvNphaseImplementationContextUsableAsFallback
{
public:
											PxsNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, PxVirtualAllocatorCallback* callback, PxU32 index, bool gpu) :
											PxvNphaseImplementationContextUsableAsFallback	(context), 
											mNarrowPhasePairs								(index, callback), 
											mNewNarrowPhasePairs							(index, callback),
											mModifyCallback									(NULL),
											mIslandSim										(islandSim),
											mGPU											(gpu)
											{}

	// PxvNphaseImplementationContext
	virtual void							destroy()	PX_OVERRIDE;
	virtual void							updateContactManager(PxReal dt, bool hasContactDistanceChanged, PxBaseTask* continuation, 
																PxBaseTask* firstPassContinuation, Cm::FanoutTask* updateBoundAndShape)	PX_OVERRIDE;
	virtual void							postBroadPhaseUpdateContactManager(PxBaseTask*) PX_OVERRIDE	{}
	virtual void							secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)	PX_OVERRIDE;
	virtual void							fetchUpdateContactManager() PX_OVERRIDE	{}
	virtual void							registerContactManager(PxsContactManager* cm, Sc::ShapeInteraction* shapeInteraction, PxI32 touching, PxU32 numPatches)	PX_OVERRIDE;
//	virtual void							registerContactManagers(PxsContactManager** cm, Sc::ShapeInteraction** shapeInteractions, PxU32 nbContactManagers, PxU32 maxContactManagerId);
	virtual void							unregisterContactManager(PxsContactManager* cm)	PX_OVERRIDE;
	virtual void							refreshContactManager(PxsContactManager* cm)	PX_OVERRIDE;

	virtual void							registerShape(const PxNodeIndex& /*nodeIndex*/, const PxsShapeCore& /*shapeCore*/, const PxU32 /*transformCacheID*/, PxActor* /*actor*/, const bool /*isFemCloth*/) PX_OVERRIDE	{}
	virtual void							unregisterShape(const PxsShapeCore& /*shapeCore*/, const PxU32 /*transformCacheID*/, const bool /*isFemCloth*/)		PX_OVERRIDE			{}

	virtual void							registerAggregate(const PxU32 /*transformCacheID*/)		PX_OVERRIDE	{}

	virtual void							registerMaterial(const PxsMaterialCore&)				PX_OVERRIDE	{}
	virtual void							updateMaterial(const PxsMaterialCore&)					PX_OVERRIDE	{}
	virtual void							unregisterMaterial(const PxsMaterialCore&)				PX_OVERRIDE	{}

	virtual void							registerMaterial(const PxsFEMSoftBodyMaterialCore&)		PX_OVERRIDE	{}
	virtual void							updateMaterial(const PxsFEMSoftBodyMaterialCore&)		PX_OVERRIDE	{}
	virtual void							unregisterMaterial(const PxsFEMSoftBodyMaterialCore&)	PX_OVERRIDE	{}

	virtual void							registerMaterial(const PxsFEMClothMaterialCore&)		PX_OVERRIDE	{}
	virtual void							updateMaterial(const PxsFEMClothMaterialCore&)			PX_OVERRIDE	{}
	virtual void							unregisterMaterial(const PxsFEMClothMaterialCore&)		PX_OVERRIDE	{}

	virtual void							registerMaterial(const PxsPBDMaterialCore&)				PX_OVERRIDE	{}
	virtual void							updateMaterial(const PxsPBDMaterialCore&)				PX_OVERRIDE	{}
	virtual void							unregisterMaterial(const PxsPBDMaterialCore&)			PX_OVERRIDE	{}

	virtual void							registerMaterial(const PxsFLIPMaterialCore&)			PX_OVERRIDE	{}
	virtual void							updateMaterial(const PxsFLIPMaterialCore&)				PX_OVERRIDE	{}
	virtual void							unregisterMaterial(const PxsFLIPMaterialCore&)			PX_OVERRIDE	{}

	virtual void							registerMaterial(const PxsMPMMaterialCore&)				PX_OVERRIDE	{}
	virtual void							updateMaterial(const PxsMPMMaterialCore&)				PX_OVERRIDE	{}
	virtual void							unregisterMaterial(const PxsMPMMaterialCore&)			PX_OVERRIDE	{}

	virtual void							updateShapeMaterial(const PxsShapeCore&)				PX_OVERRIDE	{}

	virtual void							startNarrowPhaseTasks()									PX_OVERRIDE	{}

	virtual void							appendContactManagers()	PX_OVERRIDE;

	virtual PxsContactManagerOutput&		getNewContactManagerOutput(PxU32 npIndex)	PX_OVERRIDE;

	virtual PxsContactManagerOutputIterator getContactManagerOutputs()	PX_OVERRIDE;
	virtual void							setContactModifyCallback(PxContactModifyCallback* callback) PX_OVERRIDE	{ mModifyCallback = callback; }

	virtual void							acquireContext()	PX_OVERRIDE	{}
	virtual void							releaseContext()	PX_OVERRIDE	{}
	virtual void							preallocateNewBuffers(PxU32 /*nbNewPairs*/, PxU32 /*maxIndex*/) PX_OVERRIDE	{ /*TODO - implement if it's useful to do so*/}
	virtual void							lock()		PX_OVERRIDE	{ mContactManagerMutex.lock();		}
	virtual void							unlock()	PX_OVERRIDE	{ mContactManagerMutex.unlock();	}

	virtual PxsContactManagerOutputCounts*	getFoundPatchOutputCounts()	PX_OVERRIDE	{ return mCmFoundLostOutputCounts.begin(); }
	virtual PxsContactManager**				getFoundPatchManagers()		PX_OVERRIDE	{ return mCmFoundLost.begin(); }
	virtual PxU32							getNbFoundPatchManagers()	PX_OVERRIDE	{ return mCmFoundLost.size(); }

	virtual PxsContactManagerOutput*		getGPUContactManagerOutputBase()	PX_OVERRIDE	{ return NULL; }
	virtual PxReal*							getGPURestDistances()				PX_OVERRIDE	{ return NULL; }
	virtual Sc::ShapeInteraction**			getGPUShapeInteractions()			PX_OVERRIDE	{ return NULL; }
	virtual PxsTorsionalFrictionData*		getGPUTorsionalData()				PX_OVERRIDE	{ return NULL; }
	//~PxvNphaseImplementationContext

	// PxvNphaseImplementationFallback
	virtual	void							processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation)	PX_OVERRIDE;
	virtual	void							processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation)	PX_OVERRIDE;
	virtual void							unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE;
	virtual void							refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE;
	virtual void							appendContactManagersFallback(PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE;
	virtual void							removeContactManagersFallback(PxsContactManagerOutput* cmOutputs)	PX_OVERRIDE;
	virtual Sc::ShapeInteraction**			getShapeInteractions()	PX_OVERRIDE	{ return mNarrowPhasePairs.mShapeInteractions.begin();		}
	virtual PxReal*							getRestDistances()		PX_OVERRIDE	{ return mNarrowPhasePairs.mRestDistances.begin();			}
	virtual PxsTorsionalFrictionData*		getTorsionalData()		PX_OVERRIDE	{ return mNarrowPhasePairs.mTorsionalProperties.begin();	}
	//~PxvNphaseImplementationFallback

			PxArray<PxU32>					mRemovedContactManagers;
			PxsContactManagers				mNarrowPhasePairs;
			PxsContactManagers				mNewNarrowPhasePairs;

			PxContactModifyCallback*		mModifyCallback;

			IG::IslandSim*					mIslandSim;

			PxMutex							mContactManagerMutex;

			PxArray<PxsCMDiscreteUpdateTask*> mCmTasks;

			PxArray<PxsContactManagerOutputCounts> mCmFoundLostOutputCounts;
			PxArray<PxsContactManager*>		mCmFoundLost;

			const bool						mGPU;

private:
			void							unregisterContactManagerInternal(PxU32 npIndex, PxsContactManagers& managers, PxsContactManagerOutput* cmOutputs);

			PX_FORCE_INLINE void			unregisterAndForceSize(PxsContactManagers& cms, PxU32 index)
			{
				unregisterContactManagerInternal(index, cms, cms.mOutputContactManagers.begin());
				cms.mOutputContactManagers.forceSize_Unsafe(cms.mOutputContactManagers.size()-1);
			}

			void							appendNewLostPairs();

	PX_NOCOPY(PxsNphaseImplementationContext)
};

}

#endif
