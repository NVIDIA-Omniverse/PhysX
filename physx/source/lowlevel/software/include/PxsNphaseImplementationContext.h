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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
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
	PxArray<PxsContactManagerOutput>	mOutputContactManagers;
	PxArray<PxsContactManager*>			mContactManagerMapping;
	PxArray<Gu::Cache>					mCaches;

	// PT: these buffers should be in pinned memory but may not be if pinned allocation failed.
	PxPinnedArraySafe<const Sc::ShapeInteraction*>	mShapeInteractionsGPU;
	PxFloatArrayPinnedSafe							mRestDistancesGPU;
	PxPinnedArraySafe<PxsTorsionalFrictionData>		mTorsionalPropertiesGPU;

	PxsContactManagers(const PxU32 bucketId, PxVirtualAllocatorCallback* callback) : PxsContactManagerBase(bucketId),
		mOutputContactManagers	("mOutputContactManagers"),
		mContactManagerMapping	("mContactManagerMapping"),
		mCaches					("mCaches"),
		mShapeInteractionsGPU	(callback),
		mRestDistancesGPU		(callback),
		mTorsionalPropertiesGPU	(callback)
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
											PxsNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, PxVirtualAllocatorCallback* callback, PxU32 index, bool gpu) :
											PxvNphaseImplementationFallback	(context), 
											mNarrowPhasePairs				(index, callback), 
											mNewNarrowPhasePairs			(index, callback),
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

	virtual PxsContactManagerOutputCounts*	getLostFoundPatchOutputCounts()	PX_OVERRIDE	PX_FINAL	{ return mGPU ? mCmFoundLostOutputCounts.begin() : NULL; }
	virtual PxsContactManager**				getLostFoundPatchManagers()		PX_OVERRIDE	PX_FINAL	{ return mGPU ? mCmFoundLost.begin() : NULL; }
	virtual PxU32							getNbLostFoundPatchManagers()	PX_OVERRIDE	PX_FINAL	{ return mGPU ? mCmFoundLost.size() : 0; }

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
};

}

#endif
