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

// PT: this file contains most Scene functions called during the simulate() call, i.e. the "pipeline" functions.
// Ideally they should be listed in the order in which they are called in the single-threaded version, to help
// understanding and following the pipeline.

#include "ScScene.h"
#include "BpBroadPhase.h"
#include "ScArticulationSim.h"
#include "ScSimStats.h"
#include "PxsCCD.h"

#if defined(__APPLE__) && defined(__POWERPC__)
	#include <ppc_intrinsics.h>
#endif

#if PX_SUPPORT_GPU_PHYSX
	#include "PxPhysXGpu.h"
	#include "PxsKernelWrangler.h"
	#include "PxsHeapMemoryAllocator.h"
	#include "cudamanager/PxCudaContextManager.h"
#endif

#include "ScShapeInteraction.h"
#include "ScElementInteractionMarker.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "PxSoftBody.h"
	#include "ScSoftBodySim.h"
	#include "DySoftBody.h"
	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
		#include "PxFEMCloth.h"
		#include "PxHairSystem.h"
	#endif
	#include "ScFEMClothSim.h"
	#include "DyFEMCloth.h"
	#include "ScParticleSystemSim.h"
	#include "DyParticleSystem.h"
	#include "ScHairSystemSim.h"
	#include "DyHairSystem.h"
#endif

using namespace physx;
using namespace physx::Cm;
using namespace physx::Dy;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

void PxcClearContactCacheStats();
void Sc::Scene::stepSetupCollide(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.stepSetupCollide", mContextId);

	{
		PX_PROFILE_ZONE("Sim.prepareCollide", mContextId);
		mReportShapePairTimeStamp++;	// deleted actors/shapes should get separate pair entries in contact reports
		mContactReportsNeedPostSolverVelocity = false;

		getRenderBuffer().clear();

		// Clear broken constraint list:
		clearBrokenConstraintBuffer();

		visualizeStartStep();
	
		PxcClearContactCacheStats();
	}

	kinematicsSetup(continuation);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	// Update all dirty interactions
	mNPhaseCore->updateDirtyInteractions(outputs);
	mInternalFlags &= ~(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE | SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION);
}

void Sc::Scene::simulate(PxReal timeStep, PxBaseTask* continuation)
{
	if(timeStep != 0.0f)
	{
		setElapsedTime(timeStep);
		mDynamicsContext->setDt(timeStep);

		mAdvanceStep.setContinuation(continuation);

		stepSetupCollide(&mAdvanceStep); 
		
		mCollideStep.setContinuation(&mAdvanceStep);

		mAdvanceStep.removeReference();
		mCollideStep.removeReference();
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::collideStep(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.collideQueueTasks", mContextId);
	PX_PROFILE_START_CROSSTHREAD("Basic.collision", mContextId);

	mStats->simStart();
	mLLContext->beginUpdate();

	mSimulationController->flushInsertions();

	mPostNarrowPhase.setTaskManager(*continuation->getTaskManager());
	mPostNarrowPhase.addReference();

	mFinalizationPhase.setTaskManager(*continuation->getTaskManager());
	mFinalizationPhase.addReference();

	mRigidBodyNarrowPhase.setContinuation(continuation);
	mPreRigidBodyNarrowPhase.setContinuation(&mRigidBodyNarrowPhase);
	mUpdateShapes.setContinuation(&mPreRigidBodyNarrowPhase);

	mRigidBodyNarrowPhase.removeReference();
	mPreRigidBodyNarrowPhase.removeReference();
	mUpdateShapes.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateShapes(PxBaseTask* continuation)
{
	//dma shapes data to gpu
	mSimulationController->updateShapes(continuation);
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
class DirtyShapeUpdatesTask : public Cm::Task
{
public:
	static const PxU32 MaxShapes = 256;

	PxsTransformCache&	mCache;
	Bp::BoundsArray&	mBoundsArray;
	ShapeSim*			mShapes[MaxShapes];
	PxU32				mNbShapes;

	DirtyShapeUpdatesTask(PxU64 contextID, PxsTransformCache& cache, Bp::BoundsArray& boundsArray) : 
		Cm::Task	(contextID),
		mCache		(cache),
		mBoundsArray(boundsArray),
		mNbShapes	(0)
	{
	}

	virtual void runInternal() 
	{
		for (PxU32 a = 0; a < mNbShapes; ++a)
			mShapes[a]->updateCached(mCache, mBoundsArray);
	}

	virtual const char* getName() const { return "DirtyShapeUpdatesTask";  }

private:
	PX_NOCOPY(DirtyShapeUpdatesTask)
};
}

static DirtyShapeUpdatesTask* createDirtyShapeUpdateTask(Cm::FlushPool& pool, PxU64 contextID, PxsTransformCache& cache, Bp::BoundsArray& boundsArray)
{
	return PX_PLACEMENT_NEW(pool.allocate(sizeof(DirtyShapeUpdatesTask)), DirtyShapeUpdatesTask)(contextID, cache, boundsArray);
}

void Sc::Scene::updateDirtyShapes(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Scene.updateDirtyShapes", mContextId);

	// PT: it is quite unfortunate that we cannot shortcut parsing the bitmaps. We should consider switching to arrays.

	//Process dirty shapeSims...
	PxBitMap::Iterator dirtyShapeIter(mDirtyShapeSimMap);

	PxsTransformCache& cache = mLLContext->getTransformCache();
	Bp::BoundsArray& boundsArray = mAABBManager->getBoundsArray();

	Cm::FlushPool& pool = mLLContext->getTaskPool();
	PxBitMapPinned& changedMap = mAABBManager->getChangedAABBMgActorHandleMap();

	DirtyShapeUpdatesTask* task = createDirtyShapeUpdateTask(pool, mContextId, cache, boundsArray);

	// PT: TASK-CREATION TAG
	bool hasDirtyShapes = false;
	PxU32 nbDirtyShapes = 0;
	PxU32 index;
	while((index = dirtyShapeIter.getNext()) != PxBitMap::Iterator::DONE)
	{
		ShapeSim* shapeSim = reinterpret_cast<ShapeSim*>(mAABBManager->getUserData(index));
		if(shapeSim)
		{
			hasDirtyShapes = true;
			changedMap.growAndSet(index);
			task->mShapes[nbDirtyShapes++] = shapeSim;

			// PT: consider better load balancing?
			if(nbDirtyShapes == DirtyShapeUpdatesTask::MaxShapes)
			{
				task->mNbShapes = nbDirtyShapes;
				nbDirtyShapes = 0;
				startTask(task, continuation);

				task = createDirtyShapeUpdateTask(pool, mContextId, cache, boundsArray);
			}
		}
	}

	if(hasDirtyShapes)
	{
		//Setting the boundsArray and transform cache as dirty so that they get DMAd to GPU if GPU dynamics and BP are being used respectively.
		//These bits are no longer set when we update the cached state for actors due to an optimization avoiding setting these dirty bits multiple times.
		getBoundsArray().setChangedState();
		getLowLevelContext()->getTransformCache().setChangedState();
	}

	if(nbDirtyShapes)
	{
		task->mNbShapes = nbDirtyShapes;
		startTask(task, continuation);
	}

	// PT: we clear the map but we don't shrink it, bad because we always parse it above
	mDirtyShapeSimMap.clear();
}

void Sc::Scene::preRigidBodyNarrowPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Scene.preNarrowPhase", mContextId);

	updateContactDistances(continuation);
	updateDirtyShapes(continuation);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::rigidBodyNarrowPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.narrowPhase", mContextId);

	mCCDPass = 0;

	mPostBroadPhase3.addDependent(*continuation);
	mPostBroadPhase2.setContinuation(&mPostBroadPhase3);
	mPostBroadPhaseCont.setContinuation(&mPostBroadPhase2);
	mPostBroadPhase.setContinuation(&mPostBroadPhaseCont);
	mBroadPhase.setContinuation(&mPostBroadPhase);

	mRigidBodyNPhaseUnlock.setContinuation(continuation);
	mRigidBodyNPhaseUnlock.addReference();

	mUpdateBoundAndShapeTask.addDependent(mBroadPhase);

	mLLContext->resetThreadContexts();

	mLLContext->updateContactManager(mDt, mBoundsArray->hasChanged(), mHasContactDistanceChanged, continuation, 
		&mRigidBodyNPhaseUnlock, &mUpdateBoundAndShapeTask); // Starts update of contact managers

	mPostBroadPhase3.removeReference();
	mPostBroadPhase2.removeReference();
	mPostBroadPhaseCont.removeReference();
	mPostBroadPhase.removeReference();
	mBroadPhase.removeReference();

	mUpdateBoundAndShapeTask.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateBoundsAndShapes(PxBaseTask* /*continuation*/)
{
	//if the scene doesn't use gpu dynamic and gpu broad phase and the user enables the direct API,
	//the sdk will refuse to create the scene.
	const bool useDirectGpuApi = mPublicFlags & PxSceneFlag::eENABLE_DIRECT_GPU_API;
	mSimulationController->updateBoundsAndShapes(*mAABBManager, mUseGpuBp, useDirectGpuApi);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::broadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.broadPhase", mContextId);

	mProcessLostPatchesTask.setContinuation(&mPostNarrowPhase);
	mProcessLostPatchesTask.removeReference();

#if PX_SUPPORT_GPU_PHYSX
	gpu_updateBounds();
#endif

	mCCDBp = false;

	mBpSecondPass.setContinuation(continuation);
	mBpFirstPass.setContinuation(&mBpSecondPass);

	mBpSecondPass.removeReference();
	mBpFirstPass.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processFoundSolverPatches(PxBaseTask* /*continuation*/)
{
	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();
	mDynamicsContext->processFoundPatches(*mSimpleIslandManager, nphase->getFoundPatchManagers(), nphase->getNbFoundPatchManagers(), nphase->getFoundPatchOutputCounts());
}

void Sc::Scene::processLostSolverPatches(PxBaseTask* /*continuation*/)
{
	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();
	mDynamicsContext->processLostPatches(*mSimpleIslandManager, nphase->getFoundPatchManagers(), nphase->getNbFoundPatchManagers(), nphase->getFoundPatchOutputCounts());
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::broadPhaseFirstPass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.broadPhaseFirstPass", mContextId);

	const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
	mAABBManager->updateBPFirstPass(numCpuTasks, mLLContext->getTaskPool(), mHasContactDistanceChanged, continuation);
	
	const PxU32 maxAABBHandles = PxMax(mAABBManager->getChangedAABBMgActorHandleMap().getWordCount() * 32, getElementIDPool().getMaxID());
	
	mSimulationController->mergeChangedAABBMgHandle(maxAABBHandles, mPublicFlags & PxSceneFlag::eENABLE_DIRECT_GPU_API);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::broadPhaseSecondPass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.broadPhaseSecondPass", mContextId);

	mBpUpdate.setContinuation(continuation);
	mPreIntegrate.setContinuation(&mBpUpdate);

	mPreIntegrate.removeReference();
	mBpUpdate.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::preIntegrate(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.preIntegrate", mContextId);

	if (!mCCDBp && isUsingGpuDynamicsOrBp())
		mSimulationController->preIntegrateAndUpdateBound(continuation, mGravity, mDt);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.updateBroadPhase", mContextId);

	PxBaseTask* rigidBodyNPhaseUnlock = mCCDPass ? NULL : &mRigidBodyNPhaseUnlock;

	mAABBManager->updateBPSecondPass(&mLLContext->getScratchAllocator(), continuation);

	// PT: decoupling: I moved this back from updateBPSecondPass
	//if this is mCCDPass, narrowPhaseUnlockTask will be NULL
	if(rigidBodyNPhaseUnlock)
		rigidBodyNPhaseUnlock->removeReference();

	if(!mCCDBp && isUsingGpuDynamicsOrBp())
		mSimulationController->updateParticleSystemsAndSoftBodies();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::unblockNarrowPhase(PxBaseTask*)
{
	/*if (!mCCDBp && mUseGpuRigidBodies)
		mSimulationController->updateParticleSystemsAndSoftBodies();*/
	//
	mLLContext->getNphaseImplementationContext()->startNarrowPhaseTasks();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::postBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.postBroadPhase", mContextId);

	//Notify narrow phase that broad phase has completed
	mLLContext->getNphaseImplementationContext()->postBroadPhaseUpdateContactManager(continuation);

	mAABBManager->postBroadPhase(continuation, *getFlushPool());
}

///////////////////////////////////////////////////////////////////////////////

	class OverlapFilterTask : public Cm::Task
	{
	public:
		static const PxU32 MaxPairs = 512;
		NPhaseCore*				mNPhaseCore;
		const Bp::AABBOverlap*	mPairs;

		PxU32					mNbToProcess;

		PxU32					mKeepMap[MaxPairs/32];

		FilterInfo*				mFinfo;

		PxU32					mNbToKeep;
		PxU32					mNbToSuppress;

		OverlapFilterTask*		mNext;

		OverlapFilterTask(PxU64 contextID, NPhaseCore* nPhaseCore, FilterInfo* fInfo, const Bp::AABBOverlap* pairs, PxU32 nbToProcess) :
			Cm::Task		(contextID),
			mNPhaseCore		(nPhaseCore),
			mPairs			(pairs),
			mNbToProcess	(nbToProcess),
			mFinfo			(fInfo),
			mNbToKeep		(0),
			mNbToSuppress	(0),
			mNext			(NULL)
		{
			PxMemZero(mKeepMap, sizeof(mKeepMap));
		}

		virtual void runInternal()
		{
			mNPhaseCore->runOverlapFilters(	mNbToProcess, mPairs, mFinfo, mNbToKeep, mNbToSuppress, mKeepMap);
		}

		virtual const char* getName() const { return "OverlapFilterTask"; }
	};

void Sc::Scene::finishBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::finishBroadPhase", mContextId);

	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps", mContextId);

		// PT: we process "trigger pairs" immediately, sequentially. Both the filtering and the creation of trigger
		// interactions happen at the same time in onTriggerOverlapCreated.
		// PT: could we drop trigger interactions or parallelize this? I am not sure why Kier decided to treat trigger
		// interactions differently here, in my eyes it is pretty much the same as regular interactions, and they
		// could have been kept in the same multithreaded pipeline. Regular shape interactions also call "registerInActors"
		// by default, so the below comment is not very convincing - we worked around this for ShapeInteraction, we
		// could have worked around this as well for TriggerInteraction.
		{
			//KS - these functions call "registerInActors", while OverlapFilterTask reads the list of interactions
			//in an actor. This could lead to a race condition and a crash if they occur at the same time, so we 
			//serialize these operations
			PX_PROFILE_ZONE("Sim.processNewOverlaps.createOverlapsNoShapeInteractions", mContextId);
			{
				PxU32 createdOverlapCount;
				const Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(Bp::ElementType::eTRIGGER, createdOverlapCount);
				if(createdOverlapCount)
				{
					mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;
					mNPhaseCore->onTriggerOverlapCreated(p, createdOverlapCount);
				}
			}
		}

		// PT: for regular shapes the code has been multithreaded and split into different parts, making it harder to follow.
		// Basically this is the same code as the above for triggers, but scattered over multiple Sc::Scene functions and
		// tasks. As far as I can tell the steps are:
		// - "first stage" filtering (right here below)
		// - "second stage" filtering and creation of ShapeInteractions in preallocateContactManagers
		// - some cleanup in postBroadPhaseStage2
		{
			PxU32 createdOverlapCount;
			const Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(Bp::ElementType::eSHAPE, createdOverlapCount);

			// PT: removed this because it's pointless at this stage?
			if(0)
			{
				//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
				mPreallocatedContactManagers.reserve(1);
				mPreallocatedShapeInteractions.reserve(1);
				mPreallocatedInteractionMarkers.reserve(1);

				mPreallocatedContactManagers.forceSize_Unsafe(1);
				mPreallocatedShapeInteractions.forceSize_Unsafe(1);
				mPreallocatedInteractionMarkers.forceSize_Unsafe(1);
			}

			mPreallocateContactManagers.setContinuation(continuation);

			// PT: this is a temporary member value used to pass the OverlapFilterTasks to the next stage of the pipeline (preallocateContactManagers).
			// It ideally shouldn't be a class member but just a user-data passed from one task to the next. The task manager doesn't support that though (AFAIK),
			// so instead it just lies there in Sc::Scene as a class member. It's only used in finishBroadPhase & preallocateContactManagers though.
			mOverlapFilterTaskHead = NULL;

			if(createdOverlapCount)
			{
				mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;

				Cm::FlushPool& flushPool = mLLContext->getTaskPool();

				// PT: temporary data, similar to mOverlapFilterTaskHead. Will be filled with filter info for each pair by the OverlapFilterTask.
				mFilterInfo.forceSize_Unsafe(0);
				mFilterInfo.reserve(createdOverlapCount);
				mFilterInfo.forceSize_Unsafe(createdOverlapCount);

				// PT: TASK-CREATION TAG
				const PxU32 nbPairsPerTask = OverlapFilterTask::MaxPairs;
				OverlapFilterTask* previousTask = NULL;
				for(PxU32 a=0; a<createdOverlapCount; a+=nbPairsPerTask)
				{
					const PxU32 nbToProcess = PxMin(createdOverlapCount - a, nbPairsPerTask);
					OverlapFilterTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OverlapFilterTask)), OverlapFilterTask)(mContextId, mNPhaseCore, mFilterInfo.begin() + a, p + a, nbToProcess);

					task->setContinuation(&mPreallocateContactManagers);
					task->removeReference();

					// PT: setup a linked-list of OverlapFilterTasks, will be parsed in preallocateContactManagers
					if(previousTask)
						previousTask->mNext = task;
					else
						mOverlapFilterTaskHead = task;

					previousTask = task;
				}
			}
		}

		mPreallocateContactManagers.removeReference();
	}	
}

void Sc::Scene::postBroadPhaseContinuation(PxBaseTask* continuation)
{
	mAABBManager->getChangedAABBMgActorHandleMap().clear();

	// - Finishes broadphase update
	// - Adds new interactions (and thereby contact managers if needed)
	finishBroadPhase(continuation);
}

///////////////////////////////////////////////////////////////////////////////

template<class T>
static PX_FORCE_INLINE	T*		markPointerAsUsed(T* ptr)			{ return reinterpret_cast<T*>(size_t(ptr) | 1);	}

static PX_FORCE_INLINE	size_t	isPointerMarkedAsUsed(void* ptr)	{ return size_t(ptr) & 1;						}

template<class T>
static PX_FORCE_INLINE	T*	getUsedPointer(T* ptr)
{
	const size_t address = size_t(ptr);
	return address & 1 ? reinterpret_cast<T*>(address & size_t(~1)) : NULL;
}

namespace
{
	class OnOverlapCreatedTask : public Cm::Task
	{
	public:
		NPhaseCore*					mNPhaseCore;
		const Bp::AABBOverlap*		mPairs;
		const FilterInfo*			mFinfo;
		PxsContactManager**			mContactManagers;
		ShapeInteraction**			mShapeInteractions;
		ElementInteractionMarker**	mInteractionMarkers;
		PxU32						mNbToProcess;

		OnOverlapCreatedTask(PxU64 contextID, NPhaseCore* nPhaseCore, const Bp::AABBOverlap* pairs, const FilterInfo* fInfo, PxsContactManager** contactManagers,
							ShapeInteraction** shapeInteractions, ElementInteractionMarker** interactionMarkers, PxU32 nbToProcess) :
			Cm::Task			(contextID),
			mNPhaseCore			(nPhaseCore),
			mPairs				(pairs),
			mFinfo				(fInfo),
			mContactManagers	(contactManagers),
			mShapeInteractions	(shapeInteractions),
			mInteractionMarkers	(interactionMarkers),
			mNbToProcess		(nbToProcess)
		{
		}

		virtual void runInternal()
		{
			PxsContactManager** currentCm = mContactManagers;
			ShapeInteraction** currentSI = mShapeInteractions;
			ElementInteractionMarker** currentEI = mInteractionMarkers;

			for(PxU32 i=0; i<mNbToProcess; i++)
			{
				const Bp::AABBOverlap& pair = mPairs[i];
				ShapeSimBase* s0 = reinterpret_cast<ShapeSimBase*>(pair.mUserData1);
				ShapeSimBase* s1 = reinterpret_cast<ShapeSimBase*>(pair.mUserData0);

				ElementSimInteraction* interaction = mNPhaseCore->createRbElementInteraction(mFinfo[i], *s0, *s1, *currentCm, *currentSI, *currentEI, false);
				if(interaction)
				{
					const InteractionType::Enum type = interaction->getType();
					if(type == InteractionType::eOVERLAP)
					{
						PX_ASSERT(interaction==*currentSI);

						*currentSI = markPointerAsUsed(*currentSI);
						currentSI++;

						if(static_cast<ShapeInteraction*>(interaction)->getContactManager())
						{
							PX_ASSERT(static_cast<ShapeInteraction*>(interaction)->getContactManager()==*currentCm);

							*currentCm = markPointerAsUsed(*currentCm);
							currentCm++;
						}
					}
					else if(type == InteractionType::eMARKER)
					{
						*currentEI = markPointerAsUsed(*currentEI);
						currentEI++;
					}
				}
			}
		}

		virtual const char* getName() const { return "OnOverlapCreatedTask"; }
	};
}

void Sc::Scene::preallocateContactManagers(PxBaseTask* continuation)
{
	//Iterate over all filter tasks and work out how many pairs we need...

	PxU32 totalCreatedPairs = 0;
	PxU32 totalSuppressPairs = 0;

	OverlapFilterTask* task = mOverlapFilterTaskHead;
	while(task)
	{
		totalCreatedPairs += task->mNbToKeep;
		totalSuppressPairs += task->mNbToSuppress;
		task = task->mNext;
	}

	{
		//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
		// PT: this has to do with the way we dereference currentCm, currentSI and currentEI in OnOverlapCreatedTask
		// before we know which type of interaction will be created. That is, we need room for at least one of each type
		// even if no interaction of that type will be created.
		// PT: don't we preallocate 2 to 3 times as much memory as needed here then?
		// PT: also doesn't it mean we're going to allocate & deallocate ALL the interaction markers most of the time?
		mPreallocatedContactManagers.forceSize_Unsafe(0);
		mPreallocatedShapeInteractions.forceSize_Unsafe(0);
		mPreallocatedInteractionMarkers.forceSize_Unsafe(0);

		mPreallocatedContactManagers.reserve(totalCreatedPairs+1);
		mPreallocatedShapeInteractions.reserve(totalCreatedPairs+1);
		mPreallocatedInteractionMarkers.reserve(totalSuppressPairs+1);

		mPreallocatedContactManagers.forceSize_Unsafe(totalCreatedPairs);
		mPreallocatedShapeInteractions.forceSize_Unsafe(totalCreatedPairs);
		mPreallocatedInteractionMarkers.forceSize_Unsafe(totalSuppressPairs);
	}

	PxU32 overlapCount;
	Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(Bp::ElementType::eSHAPE, overlapCount);
	if(!overlapCount)
		return;

	struct Local
	{
		static void processBatch(const PxU32 createdCurrIdx, PxU32& createdStartIdx, const PxU32 suppressedCurrIdx, PxU32& suppressedStartIdx, const PxU32 batchSize,
			PxsContext* const context, NPhaseCore* const core, OnOverlapCreatedTask* const createTask, PxBaseTask* const continuation_,
			PxsContactManager** const cms_, ShapeInteraction** const shapeInter_, ElementInteractionMarker** const markerIter_)
		{
			const PxU32 nbToCreate = createdCurrIdx - createdStartIdx;
			const PxU32 nbToSuppress = suppressedCurrIdx - suppressedStartIdx;

			context->getContactManagerPool().preallocate(nbToCreate, cms_ + createdStartIdx);

			for (PxU32 i = 0; i < nbToCreate; ++i)
				shapeInter_[createdStartIdx + i] = core->mShapeInteractionPool.allocate();

			for (PxU32 i = 0; i < nbToSuppress; ++i)
				markerIter_[suppressedStartIdx + i] = core->mInteractionMarkerPool.allocate();
				
			createdStartIdx = createdCurrIdx;
			suppressedStartIdx = suppressedCurrIdx;

			createTask->mNbToProcess = batchSize;
			startTask(createTask, continuation_);
		}
	};

	const PxU32 nbPairsPerTask = 256;
	PxsContactManager** cms = mPreallocatedContactManagers.begin();
	ShapeInteraction** shapeInter = mPreallocatedShapeInteractions.begin();
	ElementInteractionMarker** markerIter = mPreallocatedInteractionMarkers.begin();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	FilterInfo* fInfo = mFilterInfo.begin();

	// PT: TODO: why do we create the task immediately? Why not create it only when a batch is full?
	// PT: it's the same pattern as for CCD, kinematics, etc
	OnOverlapCreatedTask* createTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(mContextId, mNPhaseCore, p, fInfo, cms, shapeInter, markerIter, 0);

	PxU32 batchSize = 0;
	PxU32 suppressedStartIdx = 0;
	PxU32 createdStartIdx = 0;
	PxU32 suppressedCurrIdx = 0;
	PxU32 createdCurrIdx = 0;
	PxU32 currentReadIdx = 0;

	PxU32 createdOverlapCount = 0;

	// PT: TASK-CREATION TAG
	task = mOverlapFilterTaskHead;
	while(task)
	{
		if(task->mNbToKeep || task->mNbToSuppress)
		{
			for(PxU32 w = 0; w < (OverlapFilterTask::MaxPairs/32); ++w)
			{
				for(PxU32 b = task->mKeepMap[w]; b; b &= b-1)
				{
					const PxU32 index = (w<<5) + PxLowestSetBit(b);

					if(createdOverlapCount < (index + currentReadIdx))
					{
						p[createdOverlapCount] = task->mPairs[index];
						fInfo[createdOverlapCount] = task->mFinfo[index];
					}
					createdOverlapCount++;
					batchSize++;
				}
			}

			suppressedCurrIdx += task->mNbToSuppress;
			createdCurrIdx += task->mNbToKeep;

			if(batchSize >= nbPairsPerTask)
			{
				Local::processBatch(createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize, mLLContext, mNPhaseCore, createTask, continuation, cms, shapeInter, markerIter);

				createTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(mContextId, mNPhaseCore, p + createdOverlapCount,
					fInfo + createdOverlapCount, cms + createdStartIdx, shapeInter + createdStartIdx, markerIter + suppressedStartIdx, 0);

				batchSize = 0;
			}
		}
		currentReadIdx += OverlapFilterTask::MaxPairs;
		task = task->mNext;
	}

	if(batchSize)
		Local::processBatch(createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize, mLLContext, mNPhaseCore, createTask, continuation, cms, shapeInter, markerIter);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processLostTouchPairs()
{
	PX_PROFILE_ZONE("Sc::Scene::processLostTouchPairs", mContextId);

	const PxU32 nb = mLostTouchPairs.size();
	const SimpleBodyPair* pairs = mLostTouchPairs.begin();
	for(PxU32 i=0; i<nb; ++i)
	{
		ActorSim* body1 = pairs[i].body1;
		ActorSim* body2 = pairs[i].body2;

		// If one has been deleted, we wake the other one
		const PxIntBool deletedBody1 = mLostTouchPairsDeletedBodyIDs.boundedTest(pairs[i].body1ID);
		const PxIntBool deletedBody2 = mLostTouchPairsDeletedBodyIDs.boundedTest(pairs[i].body2ID);
		if(deletedBody1 || deletedBody2)
		{
			if(!deletedBody1) 
				body1->internalWakeUp();
			if(!deletedBody2) 
				body2->internalWakeUp();
			continue;
		}

		const bool b1Active = body1->isActive();
		const bool b2Active = body2->isActive();

		// If both are sleeping, we let them sleep
		// (for example, two sleeping objects touch and the user teleports one (without waking it up))
		if(!b1Active && !b2Active)
			continue;

		// If only one has fallen asleep, we wake them both
		if(!b1Active || !b2Active)
		{
			body1->internalWakeUp();
			body2->internalWakeUp();
		}
	}

	mLostTouchPairs.clear();
	mLostTouchPairsDeletedBodyIDs.clear();
}

void Sc::Scene::postBroadPhaseStage2(PxBaseTask* continuation)
{
	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();

	mIslandInsertion.setContinuation(continuation);
	mRegisterContactManagers.setContinuation(continuation);
	mRegisterInteractions.setContinuation(continuation);
	mRegisterSceneInteractions.setContinuation(continuation);
	mIslandInsertion.removeReference();
	mRegisterContactManagers.removeReference();
	mRegisterInteractions.removeReference();
	mRegisterSceneInteractions.removeReference();

	//Release unused Cms back to the pool (later, this needs to be done in a thread-safe way from multiple worker threads
	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.release", mContextId);

		{
			PxU32 nb = mPreallocatedContactManagers.size();
			PxsContactManager** managers = mPreallocatedContactManagers.begin();
			Cm::PoolList<PxsContactManager, PxsContext>& pool = mLLContext->getContactManagerPool();
			while(nb--)
			{
				PxsContactManager* current = *managers++;
				if(!isPointerMarkedAsUsed(current))
					pool.put(current);
			}
		}

		{
			PxU32 nb = mPreallocatedShapeInteractions.size();
			ShapeInteraction** interactions = mPreallocatedShapeInteractions.begin();
			PxPool<ShapeInteraction>& pool = mNPhaseCore->mShapeInteractionPool;
			while(nb--)
			{
				ShapeInteraction* current = *interactions++;
				if(!isPointerMarkedAsUsed(current))
					pool.deallocate(current);
			}
		}
		{
			PxU32 nb = mPreallocatedInteractionMarkers.size();
			ElementInteractionMarker** interactions = mPreallocatedInteractionMarkers.begin();
			PxPool<ElementInteractionMarker>& pool = mNPhaseCore->mInteractionMarkerPool;
			while(nb--)
			{
				ElementInteractionMarker* current = *interactions++;
				if(!isPointerMarkedAsUsed(current))
					pool.deallocate(current);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

// PT: islandInsertion / registerContactManagers / registerInteractions / registerSceneInteractions run in parallel
void Sc::Scene::islandInsertion(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.processNewOverlaps.islandInsertion", mContextId);

	const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
	for(PxU32 a = 0; a < nbShapeIdxCreated; ++a)
	{
		ShapeInteraction* interaction = getUsedPointer(mPreallocatedShapeInteractions[a]);
		if(interaction)
		{
			PxsContactManager* contactManager = const_cast<PxsContactManager*>(interaction->getContactManager());

			const ActorSim& bs0 = interaction->getShape0().getActor();
			const ActorSim& bs1 = interaction->getShape1().getActor();

			const PxActorType::Enum actorTypeLargest = PxMax(bs0.getActorType(), bs1.getActorType());

			PxNodeIndex nodeIndexB;
			if (!bs1.isStaticRigid())
				nodeIndexB = bs1.getNodeIndex();

			IG::Edge::EdgeType type = IG::Edge::eCONTACT_MANAGER;
#if PX_SUPPORT_GPU_PHYSX
			if(actorTypeLargest == PxActorType::eSOFTBODY)
				type = IG::Edge::eSOFT_BODY_CONTACT;
			else if (actorTypeLargest == PxActorType::eFEMCLOTH)
				type = IG::Edge::eFEM_CLOTH_CONTACT;
			else if(isParticleSystem(actorTypeLargest))
				type = IG::Edge::ePARTICLE_SYSTEM_CONTACT;
			else if (actorTypeLargest == PxActorType::eHAIRSYSTEM)
				type = IG::Edge::eHAIR_SYSTEM_CONTACT;
#endif
			IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(contactManager, bs0.getNodeIndex(), nodeIndexB, interaction, type);

			interaction->mEdgeIndex = edgeIdx;

			if(contactManager)
				contactManager->getWorkUnit().mEdgeIndex = edgeIdx;

			//If it is a soft body or particle overlap, treat it as a contact for now (we can hook up touch found/lost events later maybe)
			if(actorTypeLargest > PxActorType::eARTICULATION_LINK)
				mSimpleIslandManager->setEdgeConnected(edgeIdx, type);
		}
	}

	// - Wakes actors that lost touch if appropriate
	//processLostTouchPairs();

	if(mCCDPass == 0)
		mSimpleIslandManager->firstPassIslandGen();
}

///////////////////////////////////////////////////////////////////////////////

// PT: islandInsertion / registerContactManagers / registerInteractions / registerSceneInteractions run in parallel
void Sc::Scene::registerContactManagers(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.processNewOverlaps.registerCms", mContextId);

	// PT: we sometimes iterate over this array in vain (all ptrs are unused). Would be better
	// to store used pointers maybe in the overlap created tasks, and reuse these tasks here to
	// process only used pointers.

	PxvNphaseImplementationContext* nphaseContext = mLLContext->getNphaseImplementationContext();
	nphaseContext->lock();
	//nphaseContext->registerContactManagers(mPreallocatedContactManagers.begin(), mPreallocatedContactManagers.size(), mLLContext->getContactManagerPool().getMaxUsedIndex());
	const PxU32 nbCmsCreated = mPreallocatedContactManagers.size();
	for(PxU32 a = 0; a < nbCmsCreated; ++a)
	{
		PxsContactManager* cm = getUsedPointer(mPreallocatedContactManagers[a]);
		if(cm)
		{
			ShapeInteraction* interaction = getUsedPointer(mPreallocatedShapeInteractions[a]);

			nphaseContext->registerContactManager(cm, interaction, 0, 0);
		}
	}
	nphaseContext->unlock();
}

///////////////////////////////////////////////////////////////////////////////

// PT: islandInsertion / registerContactManagers / registerInteractions / registerSceneInteractions run in parallel
void Sc::Scene::registerInteractions(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.processNewOverlaps.registerInteractions", mContextId);

	const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
	for(PxU32 a = 0; a < nbShapeIdxCreated; ++a)
	{
		ShapeInteraction* interaction = getUsedPointer(mPreallocatedShapeInteractions[a]);
		if(interaction)
		{
			// PT: this is similar to interaction->registerInActors(), which is usually called from
			// interaction ctors.
			ActorSim& actorSim0 = interaction->getActorSim0();
			ActorSim& actorSim1 = interaction->getActorSim1();
			actorSim0.registerInteractionInActor(interaction);
			actorSim1.registerInteractionInActor(interaction);

			// PT: the number of counted interactions is used for the sleeping system
			if(actorSim0.isDynamicRigid())
				static_cast<BodySim*>(&actorSim0)->registerCountedInteraction();

			if(actorSim1.isDynamicRigid())
				static_cast<BodySim*>(&actorSim1)->registerCountedInteraction();
		}
	}

	const PxU32 nbMarkersCreated = mPreallocatedInteractionMarkers.size();
	for(PxU32 a = 0; a < nbMarkersCreated; ++a)
	{
		ElementInteractionMarker* interaction = getUsedPointer(mPreallocatedInteractionMarkers[a]);
		if(interaction)
		{
			// PT: no call to "interaction->onActivate()" here because it doesn't do anything
			interaction->registerInActors();
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

// PT: islandInsertion / registerContactManagers / registerInteractions / registerSceneInteractions run in parallel
void Sc::Scene::registerSceneInteractions(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.processNewOverlaps.registerInteractionsScene", mContextId);

	const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
	for(PxU32 a = 0; a < nbShapeIdxCreated; ++a)
	{
		ShapeInteraction* interaction = getUsedPointer(mPreallocatedShapeInteractions[a]);
		if(interaction)
		{
			registerInteraction(interaction, interaction->getContactManager() != NULL);
			
			const PxsContactManager* cm = interaction->getContactManager();
			if(cm)
				mLLContext->setActiveContactManager(cm, cm->getCCD());
		}
	}

	const PxU32 nbInteractionMarkers = mPreallocatedInteractionMarkers.size();
	for(PxU32 a = 0; a < nbInteractionMarkers; ++a)
	{
		ElementInteractionMarker* interaction = getUsedPointer(mPreallocatedInteractionMarkers[a]);
		if(interaction)
			registerInteraction(interaction, false);
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::finishBroadPhaseStage2(PxU32 ccdPass)
{
	PX_PROFILE_ZONE("Sc::Scene::finishBroadPhase2", mContextId);

	Bp::AABBManagerBase* aabbMgr = mAABBManager;

	PxU32 nbLostPairs = 0;
	for(PxU32 i=0; i<Bp::ElementType::eCOUNT; i++)
	{
		PxU32 destroyedOverlapCount;
		aabbMgr->getDestroyedOverlaps(Bp::ElementType::Enum(i), destroyedOverlapCount);
		nbLostPairs += destroyedOverlapCount;
	}
	mLLContext->getSimStats().mNbLostPairs += nbLostPairs;

	// PT: TODO: move this to ccd file?
	//KS - we need to defer processing lost overlaps until later!
	if (ccdPass)
	{
		PX_PROFILE_ZONE("Sim.processLostOverlaps", mContextId);
		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		PxU32 destroyedOverlapCount;

		// PT: for regular shapes
		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);

			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				//KS - this is a bit ugly. We split the "onOverlapRemoved" for shape interactions to parallelize it and that means
				//that we have to call each of the individual stages of the remove here.

				//First, we have to get the interaction pointer...
				ElementSimInteraction* interaction = mNPhaseCore->findInteraction(volume0, volume1);
				p->mPairUserData = interaction;
				if(interaction)
				{
					if(interaction->getType() == InteractionType::eOVERLAP || interaction->getType() == InteractionType::eMARKER)
					{
						//If it's a standard "overlap" interaction, we have to send a lost touch report, unregister it, and destroy its manager and island gen data.
						if(interaction->getType() == InteractionType::eOVERLAP)
						{
							ShapeInteraction* si = static_cast<ShapeInteraction*>(interaction);
							mNPhaseCore->lostTouchReports(si, PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH), NULL, 0, outputs);

							//We must check to see if we have a contact manager here. There is an edge case where actors could be put to 
							//sleep after discrete simulation, prior to CCD, causing their contactManager() to be destroyed. If their bounds
							//also ceased overlapping, then this code will try to destroy the manager again.
							if(si->getContactManager())
								si->destroyManager();
							si->clearIslandGenData();
						}

						unregisterInteraction(interaction);
					}

					//Then call "onOverlapRemoved" to actually free the interaction
					mNPhaseCore->onOverlapRemoved(volume0, volume1, ccdPass, interaction, outputs);
				}
				p++;
			}
		}

		// PT: for triggers
		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eTRIGGER, destroyedOverlapCount);

			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				p->mPairUserData = NULL;

				//KS - this is a bit ugly. 
				mNPhaseCore->onOverlapRemoved(volume0, volume1, ccdPass, NULL, outputs);
				p++;
			}
		}
	}

	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();

	if (ccdPass)
		aabbMgr->freeBuffers();
}

void Sc::Scene::postBroadPhaseStage3(PxBaseTask* /*continuation*/)
{
	finishBroadPhaseStage2(0);

	PX_PROFILE_STOP_CROSSTHREAD("Basic.postBroadPhase", mContextId);
	PX_PROFILE_STOP_CROSSTHREAD("Basic.broadPhase", mContextId);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::advanceStep(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.solveQueueTasks", mContextId);

	if(mDt != 0.0f)
	{
		mFinalizationPhase.addDependent(*continuation);
		mFinalizationPhase.removeReference();

		if(mPublicFlags & PxSceneFlag::eENABLE_CCD)
		{
			mUpdateCCDMultiPass.setContinuation(&mFinalizationPhase);
			mAfterIntegration.setContinuation(&mUpdateCCDMultiPass);
			mUpdateCCDMultiPass.removeReference();
		}
		else
		{
			mAfterIntegration.setContinuation(&mFinalizationPhase);
		}

		mPostSolver.setContinuation(&mAfterIntegration);
		mUpdateSimulationController.setContinuation(&mPostSolver);
		mUpdateDynamics.setContinuation(&mUpdateSimulationController);
		mUpdateBodies.setContinuation(&mUpdateDynamics);
		mSolver.setContinuation(&mUpdateBodies);
		mPostIslandGen.setContinuation(&mSolver);
		mIslandGen.setContinuation(&mPostIslandGen);
		mPostNarrowPhase.addDependent(mIslandGen);
		mPostNarrowPhase.removeReference();

		mSecondPassNarrowPhase.setContinuation(&mPostNarrowPhase);

		mFinalizationPhase.removeReference();
		mAfterIntegration.removeReference();
		mPostSolver.removeReference();
		mUpdateSimulationController.removeReference();
		mUpdateDynamics.removeReference();
		mUpdateBodies.removeReference();
		mSolver.removeReference();
		mPostIslandGen.removeReference();
		mIslandGen.removeReference();
		mPostNarrowPhase.removeReference();
		mSecondPassNarrowPhase.removeReference();
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::activateEdgesInternal(const IG::EdgeIndex* activatingEdges, const PxU32 nbActivatingEdges)
{
	const IG::IslandSim& speculativeSim = mSimpleIslandManager->getSpeculativeIslandSim();
	for(PxU32 i = 0; i < nbActivatingEdges; ++i)
	{
		Interaction* interaction = mSimpleIslandManager->getInteraction(activatingEdges[i]);

		if(interaction && !interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			if(speculativeSim.getEdge(activatingEdges[i]).isActive())
			{
				const bool proceed = activateInteraction(interaction, NULL);

				if(proceed && (interaction->getType() < InteractionType::eTRACKED_IN_SCENE_COUNT))
					notifyInteractionActivated(interaction);
			}
		}
	}
}

void Sc::Scene::secondPassNarrowPhase(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.secondPassNarrowPhase", mContextId);
	{
		PX_PROFILE_ZONE("Sim.postIslandGen", mContextId);

		mSimpleIslandManager->additionalSpeculativeActivation();

		// wake interactions
		{
			PX_PROFILE_ZONE("ScScene.wakeInteractions", mContextId);
			const IG::IslandSim& speculativeSim = mSimpleIslandManager->getSpeculativeIslandSim();

			//KS - only wake contact managers based on speculative state to trigger contact gen. Waking actors based on accurate state
			//should activate and joints.
			{
				//Wake speculatively based on rigid contacts, soft contacts and particle contacts
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER), speculativeSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER));
#if PX_SUPPORT_GPU_PHYSX
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eSOFT_BODY_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::eSOFT_BODY_CONTACT));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eFEM_CLOTH_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::eFEM_CLOTH_CONTACT));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::ePARTICLE_SYSTEM_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::ePARTICLE_SYSTEM_CONTACT));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eHAIR_SYSTEM_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::eHAIR_SYSTEM_CONTACT));
#endif
			}
		}
	}
	mLLContext->secondPassUpdateContactManager(mDt, &mPostNarrowPhase); // Starts update of contact managers
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::releaseConstraints(bool endOfScene)
{
	PX_ASSERT(mLLContext);

	if(mEnableStabilization)
	{
		//If stabilization is enabled, we're caching contacts for next frame
		if(!endOfScene)
		{
			//So we only clear memory (flip buffers) when not at the end-of-scene.
			//This means we clear after narrow phase completed so we can 
			//release the previous frame's contact buffers before we enter the solve phase.
			mLLContext->getNpMemBlockPool().releaseContacts();
		}
	}
	else if(endOfScene)
	{
		//We now have a double-buffered pool of mem blocks so we must
		//release both pools (which actually triggers the memory used this 
		//frame to be released 
		mLLContext->getNpMemBlockPool().releaseContacts();
		mLLContext->getNpMemBlockPool().releaseContacts();
	}
}

void Sc::Scene::postNarrowPhase(PxBaseTask* /*continuation*/)
{
	setCollisionPhaseToInactive();

	mHasContactDistanceChanged = false;
	mLLContext->fetchUpdateContactManager(); //Sync on contact gen results!

	if(!mCCDBp && isUsingGpuDynamicsOrBp())
		mSimulationController->sortContacts();

	releaseConstraints(false);

	PX_PROFILE_STOP_CROSSTHREAD("Basic.narrowPhase", mContextId);
	PX_PROFILE_STOP_CROSSTHREAD("Basic.collision", mContextId);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processNarrowPhaseTouchEvents()
{
	PX_PROFILE_ZONE("Sim.preIslandGen", mContextId);

	PxsContext* context = mLLContext;

	// Update touch states from LL
	PxU32 newTouchCount, lostTouchCount;
	PxU32 ccdTouchCount = 0;
	{
		PX_PROFILE_ZONE("Sim.preIslandGen.managerTouchEvents", mContextId);
		context->getManagerTouchEventCount(reinterpret_cast<PxI32*>(&newTouchCount), reinterpret_cast<PxI32*>(&lostTouchCount), NULL);
		//PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
		//PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);

		mTouchFoundEvents.forceSize_Unsafe(0);
		mTouchFoundEvents.reserve(newTouchCount);
		mTouchFoundEvents.forceSize_Unsafe(newTouchCount);

		mTouchLostEvents.forceSize_Unsafe(0);
		mTouchLostEvents.reserve(lostTouchCount);
		mTouchLostEvents.forceSize_Unsafe(lostTouchCount);

		context->fillManagerTouchEvents(mTouchFoundEvents.begin(), reinterpret_cast<PxI32&>(newTouchCount), mTouchLostEvents.begin(),
			reinterpret_cast<PxI32&>(lostTouchCount), NULL, reinterpret_cast<PxI32&>(ccdTouchCount));

		mTouchFoundEvents.forceSize_Unsafe(newTouchCount);
		mTouchLostEvents.forceSize_Unsafe(lostTouchCount);
	}

	context->getSimStats().mNbNewTouches = newTouchCount;
	context->getSimStats().mNbLostTouches = lostTouchCount;
}

void Sc::Scene::islandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::islandGen", mContextId);

	//mLLContext->runModifiableContactManagers(); //KS - moved here so that we can get up-to-date touch found/lost events in IG

	/*mProcessLostPatchesTask.setContinuation(&mUpdateDynamics);
	mProcessLostPatchesTask.removeReference();*/
	
	processNarrowPhaseTouchEvents();

	// PT: could we merge processNarrowPhaseTouchEventsStage2 with processNarrowPhaseTouchEvents ?
	mProcessFoundPatchesTask.setContinuation(continuation);
	mProcessFoundPatchesTask.removeReference();

	processNarrowPhaseTouchEventsStage2(&mPostSolver);	
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE ShapeInteraction* getSI(PxvContactManagerTouchEvent& evt)
{
	return reinterpret_cast<ShapeInteraction*>(evt.getCMTouchEventUserData());
}

namespace
{
	class InteractionNewTouchTask : public Cm::Task
	{
		PxvContactManagerTouchEvent*	mEvents;
		const PxU32						mNbEvents;
		PxsContactManagerOutputIterator	mOutputs;
		NPhaseCore*						mNphaseCore;

	public:
		InteractionNewTouchTask(PxU64 contextID, PxvContactManagerTouchEvent* events, PxU32 nbEvents, PxsContactManagerOutputIterator& outputs, NPhaseCore* nPhaseCore) :
			Cm::Task	(contextID),
			mEvents		(events),
			mNbEvents	(nbEvents),
			mOutputs	(outputs),
			mNphaseCore	(nPhaseCore)
		{
		}

		virtual const char* getName() const
		{
			return "InteractionNewTouchTask";
		}
	
		virtual void runInternal()
		{
			mNphaseCore->lockReports();
			for(PxU32 i = 0; i < mNbEvents; ++i)
			{
				ShapeInteraction* si = getSI(mEvents[i]);
				PX_ASSERT(si);
				mNphaseCore->managerNewTouch(*si);
				si->managerNewTouch(0, true, mOutputs);
			}
			mNphaseCore->unlockReports();
		}
	private:
		PX_NOCOPY(InteractionNewTouchTask)
	};
}

void Sc::Scene::processNarrowPhaseTouchEventsStage2(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processNarrowPhaseTouchEventsStage2", mContextId);

	PxvNphaseImplementationContext*	ctx = mLLContext->getNphaseImplementationContext();

	PxsContactManagerOutputIterator outputs = ctx->getContactManagerOutputs();

	const PxU32 newTouchCount = mTouchFoundEvents.size();

	{
		Cm::FlushPool& flushPool = mLLContext->getTaskPool();

		// PT: why not a delegate task here? We seem to be creating a single InteractionNewTouchTask ?
		InteractionNewTouchTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(InteractionNewTouchTask)), InteractionNewTouchTask)(mContextId, mTouchFoundEvents.begin(), newTouchCount, outputs, mNPhaseCore);
		startTask(task, continuation);
	}

	/*{
		PX_PROFILE_ZONE("Sim.preIslandGen.newTouchesInteraction", mContextId);
		for (PxU32 i = 0; i < newTouchCount; ++i)
		{
			ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchFoundEvents[i].userData);
			PX_ASSERT(si);
			mNPhaseCore->managerNewTouch(*si);
			si->managerNewTouch(0, true, outputs, useAdaptiveForce);
		}
	}*/
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::postIslandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.postIslandGen", mContextId);

	mSetEdgesConnectedTask.setContinuation(continuation);
	mSetEdgesConnectedTask.removeReference();

	// - Performs collision detection for trigger interactions
	mNPhaseCore->processTriggerInteractions(continuation);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::setEdgesConnected(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.preIslandGen.islandTouches", mContextId);
	{
		PX_PROFILE_ZONE("Sim.preIslandGen.setEdgesConnected", mContextId);

		const PxU32 newTouchCount = mTouchFoundEvents.size();
		for(PxU32 i = 0; i < newTouchCount; ++i)
		{
			ShapeInteraction* si = getSI(mTouchFoundEvents[i]);
			if(!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex(), IG::Edge::eCONTACT_MANAGER);
		}
	}

	mSimpleIslandManager->secondPassIslandGen();

	wakeObjectsUp();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::solver(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.rigidBodySolver", mContextId);

	//Update forces per body in parallel. This can overlap with the other work in this phase.
	beforeSolver(continuation);

	PX_PROFILE_ZONE("Sim.postNarrowPhaseSecondPass", mContextId);

	//Narrowphase is completely finished so the streams can be swapped.
	mLLContext->swapStreams();

	//PxsContactManagerOutputIterator outputs = this->mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class ScBeforeSolverTask : public Cm::Task
	{
	public:
		static const PxU32 MaxBodiesPerTask = 256;
		PxNodeIndex					mBodies[MaxBodiesPerTask];
		PxU32						mNumBodies;
		const PxReal				mDt;
		IG::SimpleIslandManager*	mIslandManager;
		PxsSimulationController*	mSimulationController;

	public:

		ScBeforeSolverTask(PxReal dt, IG::SimpleIslandManager* islandManager, PxsSimulationController* simulationController, PxU64 contextID) : 
			Cm::Task				(contextID),
			mDt						(dt),
			mIslandManager			(islandManager),
			mSimulationController	(simulationController)
		{
		}

		virtual void runInternal()
		{
			PX_PROFILE_ZONE("Sim.ScBeforeSolverTask", mContextID);

			const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();
			const PxU32 rigidBodyOffset = BodySim::getRigidBodyOffset();

			PxsRigidBody* updatedBodySims[MaxBodiesPerTask];
			PxU32 updatedBodyNodeIndices[MaxBodiesPerTask];
			PxU32 nbUpdatedBodySims = 0;

			PxU32 nb = mNumBodies;
			const PxNodeIndex* bodies = mBodies;
			while(nb--)
			{
				const PxNodeIndex index = *bodies++;

				if(islandSim.getActiveNodeIndex(index) != PX_INVALID_NODE)
				{
					if(islandSim.getNode(index).mType == IG::Node::eRIGID_BODY_TYPE)
					{
						PxsRigidBody* body = islandSim.getRigidBody(index);
						BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(body) - rigidBodyOffset);
						bodySim->updateForces(mDt, updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims, NULL);
					}
				}
			}

			if(nbUpdatedBodySims)
				mSimulationController->updateBodies(updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims);
		}

		virtual const char* getName() const
		{
			return "ScScene.beforeSolver";
		}

	private:
		PX_NOCOPY(ScBeforeSolverTask)
	};

	class ScArticBeforeSolverTask : public Cm::Task
	{
	public:
		ArticulationSim* const*		mArticSims;
		const PxU32					mNumArticulations;
		const PxReal				mDt;
		IG::SimpleIslandManager*	mIslandManager;

	public:

		ScArticBeforeSolverTask(ArticulationSim* const* articSims, PxU32 nbArtics, PxReal dt, IG::SimpleIslandManager* islandManager, PxU64 contextID) :
			Cm::Task(contextID),
			mArticSims(articSims),
			mNumArticulations(nbArtics),
			mDt(dt),
			mIslandManager(islandManager)
		{
		}

		virtual void runInternal()
		{
			PX_PROFILE_ZONE("Sim.ScArticBeforeSolverTask", mContextID);
			//const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();

			for(PxU32 a = 0; a < mNumArticulations; ++a)
			{
				ArticulationSim* PX_RESTRICT articSim = mArticSims[a];
				//articSim->checkResize();
				articSim->updateForces(mDt, false);
				articSim->setDirtyFlag(ArticulationSimDirtyFlag::eNONE);
			}
		}

		virtual const char* getName() const
		{
			return "ScScene.ScArticBeforeSolverTask";
		}

	private:
		PX_NOCOPY(ScArticBeforeSolverTask)
	};

	class ScArticBeforeSolverCCDTask : public Cm::Task
	{
	public:
		const PxNodeIndex* const	mArticIndices;
		const PxU32					mNumArticulations;
		const PxReal				mDt;
		IG::SimpleIslandManager*	mIslandManager;

	public:

		ScArticBeforeSolverCCDTask(const PxNodeIndex* const	articIndices, PxU32 nbArtics, PxReal dt, IG::SimpleIslandManager* islandManager, PxU64 contextID) :
			Cm::Task(contextID),
			mArticIndices(articIndices),
			mNumArticulations(nbArtics),
			mDt(dt),
			mIslandManager(islandManager)
		{
		}

		virtual void runInternal()
		{
			PX_PROFILE_ZONE("Sim.ScArticBeforeSolverCCDTask", mContextID);
			const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();

			for(PxU32 a = 0; a < mNumArticulations; ++a)
			{
				ArticulationSim* articSim = islandSim.getArticulationSim(mArticIndices[a]);

				articSim->saveLastCCDTransform();
			}
		}

		virtual const char* getName() const
		{
			return "ScScene.ScArticBeforeSolverCCDTask";
		}

	private:
		PX_NOCOPY(ScArticBeforeSolverCCDTask)
	};
}

void Sc::Scene::beforeSolver(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateForces", mContextId);

	// Note: For contact notifications it is important that force threshold checks are done after new/lost touches have been processed
	//       because pairs might get added to the list processed below

	// Atoms that passed contact force threshold
	ThresholdStream& thresholdStream = mDynamicsContext->getThresholdStream();
	thresholdStream.clear();

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbActiveBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	mNumDeactivatingNodes[IG::Node::eRIGID_BODY_TYPE] = 0;//islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	mNumDeactivatingNodes[IG::Node::eARTICULATION_TYPE] = 0;//islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);
//#if PX_SUPPORT_GPU_PHYSX
	mNumDeactivatingNodes[IG::Node::eSOFTBODY_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::eFEMCLOTH_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::ePARTICLESYSTEM_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::eHAIRSYSTEM_TYPE] = 0;
//#endif

	const PxU32 MaxBodiesPerTask = ScBeforeSolverTask::MaxBodiesPerTask;

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mSimulationController->reserve(nbActiveBodies);

	{
		PxBitMap::Iterator iter(mVelocityModifyMap);

		// PT: TASK-CREATION TAG
		for (PxU32 i = iter.getNext(); i != PxBitMap::Iterator::DONE; /*i = iter.getNext()*/)
		{
			ScBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScBeforeSolverTask)), ScBeforeSolverTask(mDt, mSimpleIslandManager, mSimulationController, mContextId));
			PxU32 count = 0;
			for(; count < MaxBodiesPerTask && i != PxBitMap::Iterator::DONE; i = iter.getNext())
			{
				PxsRigidBody* body = islandSim.getRigidBody(PxNodeIndex(i));
				bool retainsAccelerations = false;
				if(body)
				{
					task->mBodies[count++] = PxNodeIndex(i);

					retainsAccelerations = (body->mCore->mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS);
				}

				if(!retainsAccelerations)
					mVelocityModifyMap.reset(i);
			}
			task->mNumBodies = count;
			startTask(task, continuation);
		}
	}

	// PT: TASK-CREATION TAG
	const PxU32 nbArticsPerTask = 32;
	const PxU32 nbDirtyArticulations = mDirtyArticulationSims.size();
	ArticulationSim* const* artiSim = mDirtyArticulationSims.getEntries();
	for(PxU32 a = 0; a < nbDirtyArticulations; a += nbArticsPerTask)
	{
		const PxU32 nbToProcess = PxMin(PxU32(nbDirtyArticulations - a), nbArticsPerTask);

		ScArticBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScArticBeforeSolverTask)), ScArticBeforeSolverTask(artiSim + a, nbToProcess,
			mDt, mSimpleIslandManager, mContextId));

		startTask(task, continuation);
	}

	//if the scene has ccd flag on, we should call ScArticBeforeSolverCCDTask to copy the last transform to the current transform
	if(mPublicFlags & PxSceneFlag::eENABLE_CCD)
	{
		//CCD
		const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);
		const PxNodeIndex* const articIndices = islandSim.getActiveNodes(IG::Node::eARTICULATION_TYPE);

		// PT: TASK-CREATION TAG
		for(PxU32 a = 0; a < nbActiveArticulations; a += nbArticsPerTask)
		{
			const PxU32 nbToProcess = PxMin(PxU32(nbActiveArticulations - a), nbArticsPerTask);
			ScArticBeforeSolverCCDTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScArticBeforeSolverCCDTask)), ScArticBeforeSolverCCDTask(articIndices + a, nbToProcess,
				mDt, mSimpleIslandManager, mContextId));

			startTask(task, continuation);
		}
	}

	for(PxU32 a = 0; a < nbDirtyArticulations; a++)
		mSimulationController->updateArticulationExtAccel(artiSim[a]->getLowLevelArticulation(), artiSim[a]->getIslandNodeIndex());
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateBodies(PxBaseTask* continuation)
{
	//dma bodies and articulation data to gpu
	mSimulationController->updateBodies(continuation);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateDynamics(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.dynamics", mContextId);

	//Allow processLostContactsTask to run until after 2nd pass of solver completes (update bodies, run sleeping logic etc.)
	mProcessLostContactsTask3.setContinuation(static_cast<PxLightCpuTask*>(continuation)->getContinuation());
	mProcessLostContactsTask2.setContinuation(&mProcessLostContactsTask3);
	mProcessLostContactsTask.setContinuation(&mProcessLostContactsTask2);

	////dma bodies and shapes data to gpu
	//mSimulationController->updateBodiesAndShapes();

	mLLContext->getNpMemBlockPool().acquireConstraintMemory();

	const PxU32 maxPatchCount = mLLContext->getMaxPatchCount();

	mAABBManager->reallocateChangedAABBMgActorHandleMap(getElementIDPool().getMaxID());

	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);

	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();

	mDynamicsContext->update(*mSimpleIslandManager, continuation, &mProcessLostContactsTask,
		nphase,	maxPatchCount, mMaxNbArticulationLinks, mDt, mGravity, mAABBManager->getChangedAABBMgActorHandleMap());

	mSimpleIslandManager->clearDestroyedEdges();

	mProcessLostContactsTask3.removeReference();
	mProcessLostContactsTask2.removeReference();
	mProcessLostContactsTask.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processLostContacts(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processLostContacts", mContextId);

	mProcessNarrowPhaseLostTouchTasks.setContinuation(continuation);
	mProcessNarrowPhaseLostTouchTasks.removeReference();

	//mLostTouchReportsTask.setContinuation(&mProcessLostContactsTask3);
	mProcessNPLostTouchEvents.setContinuation(continuation);
	mProcessNPLostTouchEvents.removeReference();

	{
		PX_PROFILE_ZONE("Sim.findInteractionsPtrs", mContextId);

		Bp::AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;
		Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
			ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

			// PT: this looks useless on lost pairs but it is used in processLostContacts2 and processLostContacts3
			// PT: it seems very questionable to store this within the BP structures at this point. If anything
			// we should have stored that there when the overlap was created, and we wouldn't have to look for the
			// interaction here.
			p->mPairUserData = mNPhaseCore->findInteraction(volume0, volume1);
			p++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processNarrowPhaseLostTouchEventsIslands(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sc::Scene.islandLostTouches", mContextId);

	const PxU32 count = mTouchLostEvents.size();
	for(PxU32 i=0; i <count; ++i)
	{
		ShapeInteraction* si = getSI(mTouchLostEvents[i]);
		mSimpleIslandManager->setEdgeDisconnected(si->getEdgeIndex());
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::addToLostTouchList(ActorSim& body1, ActorSim& body2)
{
	PX_ASSERT(!body1.isStaticRigid());
	PX_ASSERT(!body2.isStaticRigid());
	SimpleBodyPair p = { &body1, &body2, body1.getActorID(), body2.getActorID() };
	mLostTouchPairs.pushBack(p);
}

void Sc::Scene::processNarrowPhaseLostTouchEvents(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sc::Scene.processNarrowPhaseLostTouchEvents", mContextId);

	PxvNphaseImplementationContext*	ctx = mLLContext->getNphaseImplementationContext();

	PxsContactManagerOutputIterator outputs = ctx->getContactManagerOutputs();
	const PxU32 count = mTouchLostEvents.size();
	for(PxU32 i=0; i<count; ++i)
	{
		ShapeInteraction* si = getSI(mTouchLostEvents[i]);
		PX_ASSERT(si);
		if(si->managerLostTouch(0, true, outputs) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
			addToLostTouchList(si->getShape0().getActor(), si->getShape1().getActor());
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processLostContacts2(PxBaseTask* continuation)
{
	mDestroyManagersTask.setContinuation(continuation);
	mLostTouchReportsTask.setContinuation(&mDestroyManagersTask);
	mLostTouchReportsTask.removeReference();

	mUnregisterInteractionsTask.setContinuation(continuation);
	mUnregisterInteractionsTask.removeReference();
	
	{
		PX_PROFILE_ZONE("Sim.clearIslandData", mContextId);

		Bp::AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;
		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSimInteraction* pair = reinterpret_cast<ElementSimInteraction*>(p->mPairUserData);
				if(pair)
				{
					if(pair->getType() == InteractionType::eOVERLAP)
					{
						ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);
						si->clearIslandGenData();
					}
				}
				p++;
			}
		}
	}

	mDestroyManagersTask.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::lostTouchReports(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.lostTouchReports", mContextId);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	mNPhaseCore->lockReports();
	{
		PxU32 destroyedOverlapCount;
		const Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			if(p->mPairUserData)
			{
				ElementSimInteraction* elemInteraction = reinterpret_cast<ElementSimInteraction*>(p->mPairUserData);
				if(elemInteraction->getType() == InteractionType::eOVERLAP)
					mNPhaseCore->lostTouchReports(static_cast<ShapeInteraction*>(elemInteraction), PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH), NULL, 0, outputs);
			}
			p++;
		}
	}
	mNPhaseCore->unlockReports();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::unregisterInteractions(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.unregisterInteractions", mContextId);

	PxU32 destroyedOverlapCount;
	const Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);

	while(destroyedOverlapCount--)
	{
		if(p->mPairUserData)
		{
			ElementSimInteraction* elemInteraction = reinterpret_cast<ElementSimInteraction*>(p->mPairUserData);
			if(elemInteraction->getType() == InteractionType::eOVERLAP || elemInteraction->getType() == InteractionType::eMARKER)
				unregisterInteraction(elemInteraction);
		}
		p++;
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::destroyManagers(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.destroyManagers", mContextId);

	mPostThirdPassIslandGenTask.setContinuation(mProcessLostContactsTask3.getContinuation());

	mSimpleIslandManager->thirdPassIslandGen(&mPostThirdPassIslandGenTask);

	PxU32 destroyedOverlapCount;
	const Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);

	while(destroyedOverlapCount--)
	{
		if(p->mPairUserData)
		{
			ElementSimInteraction* elemInteraction = reinterpret_cast<ElementSimInteraction*>(p->mPairUserData);
			if(elemInteraction->getType() == InteractionType::eOVERLAP)
			{
				ShapeInteraction* si = static_cast<ShapeInteraction*>(elemInteraction);
				if(si->getContactManager())
					si->destroyManager();
			}
		}
		p++;
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processLostContacts3(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processLostOverlapsStage2", mContextId);

		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		Bp::AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;

		// PT: for regular shapes
		{
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				mNPhaseCore->onOverlapRemoved(volume0, volume1, false, p->mPairUserData, outputs);
				p++;
			}
		}

		// PT: for triggers
		{
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eTRIGGER, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				mNPhaseCore->onOverlapRemoved(volume0, volume1, false, NULL, outputs);
				p++;
			}
		}

		aabbMgr->freeBuffers();
	}

	mPostThirdPassIslandGenTask.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

/*static*/ bool deactivateInteraction(Interaction* interaction, const InteractionType::Enum type);

void Sc::Scene::postThirdPassIslandGen(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sc::Scene::postThirdPassIslandGen", mContextId);

	putObjectsToSleep();

	{
		PX_PROFILE_ZONE("Sc::Scene::putInteractionsToSleep", mContextId);
		const IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();

		//KS - only deactivate contact managers based on speculative state to trigger contact gen. When the actors were deactivated based on accurate state
		//joints should have been deactivated.

		const PxU32 NbTypes = 5;
		const IG::Edge::EdgeType types[NbTypes] = {
			IG::Edge::eCONTACT_MANAGER,
			IG::Edge::eSOFT_BODY_CONTACT,
			IG::Edge::eFEM_CLOTH_CONTACT,
			IG::Edge::ePARTICLE_SYSTEM_CONTACT,
			IG::Edge::eHAIR_SYSTEM_CONTACT };

		for(PxU32 t = 0; t < NbTypes; ++t)
		{
			const PxU32 nbDeactivatingEdges = islandSim.getNbDeactivatingEdges(types[t]);
			const IG::EdgeIndex* deactivatingEdgeIds = islandSim.getDeactivatingEdges(types[t]);

			for(PxU32 i = 0; i < nbDeactivatingEdges; ++i)
			{
				Interaction* interaction = mSimpleIslandManager->getInteraction(deactivatingEdgeIds[i]);

				if(interaction && interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
				{
					if(!islandSim.getEdge(deactivatingEdgeIds[i]).isActive())
					{
						const InteractionType::Enum type = interaction->getType();
						const bool proceed = deactivateInteraction(interaction, type);
						if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
							notifyInteractionDeactivated(interaction);
					}
				}
			}
		}
	}

	PxvNphaseImplementationContext*	implCtx = mLLContext->getNphaseImplementationContext();
	PxsContactManagerOutputIterator outputs = implCtx->getContactManagerOutputs();
	mNPhaseCore->processPersistentContactEvents(outputs);
}

///////////////////////////////////////////////////////////////////////////////

//This is called after solver finish
void Sc::Scene::updateSimulationController(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateSimulationController", mContextId);
	
	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	Bp::BoundsArray& boundArray = getBoundsArray();

	PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();

	mSimulationController->gpuDmabackData(cache, boundArray, changedAABBMgrActorHandles, mPublicFlags & PxSceneFlag::eENABLE_DIRECT_GPU_API);

	//for pxgdynamicscontext: copy solver body data to body core 
	{
		PX_PROFILE_ZONE("Sim.updateBodyCore", mContextId);
		mDynamicsContext->updateBodyCore(continuation);
	}
	//mSimulationController->update(cache, boundArray, changedAABBMgrActorHandles);

	/*mProcessLostPatchesTask.setContinuation(&mFinalizationPhase);
	mProcessLostPatchesTask.removeReference();*/
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::postSolver(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sc::Scene::postSolver", mContextId);

	PxcNpMemBlockPool& blockPool = mLLContext->getNpMemBlockPool();

	//Merge...
	mDynamicsContext->mergeResults();
	blockPool.releaseConstraintMemory();
	//Swap friction!
	blockPool.swapFrictionStreams();

	mCcdBodies.clear();

#if PX_ENABLE_SIM_STATS
	mLLContext->getSimStats().mPeakConstraintBlockAllocations = blockPool.getPeakConstraintBlockCount();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	integrateKinematicPose();

	{
		const PxU32 size = mDirtyArticulationSims.size();
		ArticulationSim* const* articSims = mDirtyArticulationSims.getEntries();
		//clear the acceleration term for articulation if the application raised PxForceMode::eIMPULSE in addForce function. This change
		//will make sure articulation and rigid body behave the same
		const float dt = mDt;
		for(PxU32 i=0; i<size; ++i)
			articSims[i]->clearAcceleration(dt);

		//clear the dirty articulation list
		mDirtyArticulationSims.clear();
	}

	//afterIntegration(continuation);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::constraintProjection(PxBaseTask* /*continuation*/)
{
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::checkForceThresholdContactEvents(PxU32 ccdPass)
{
	PX_PROFILE_ZONE("Sim.checkForceThresholdContactEvents", mContextId);

	// Note: For contact notifications it is important that force threshold checks are done after new/lost touches have been processed
	//       because pairs might get added to the list processed below

	// Bodies that passed contact force threshold

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	ThresholdStream& thresholdStream = mDynamicsContext->getForceChangedThresholdStream();

	const PxU32 nbThresholdElements = thresholdStream.size();

	for(PxU32 i = 0; i< nbThresholdElements; ++i)
	{
		ThresholdStreamElement& elem = thresholdStream[i];
		ShapeInteraction* si = elem.shapeInteraction;

		//If there is a shapeInteraction and the shapeInteraction points to a contactManager (i.e. the CM was not destroyed in parallel with the solver)
		if(si)
		{
			PxU32 pairFlags = si->getPairFlags();
			if(pairFlags & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS)
			{
				si->swapAndClearForceThresholdExceeded();

				if(elem.accumulatedForce > elem.threshold * mDt)
				{
					si->raiseFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_NOW);

					PX_ASSERT(si->hasTouch());

					//If the accumulatedForce is large than the threshold in the current frame and the accumulatedForce is less than the threshold in the previous frame, 
					//and the user request notify for found event, we will raise eNOTIFY_THRESHOLD_FORCE_FOUND
					if((!si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE)) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND))
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND, 0, false, ccdPass, false, outputs);
					else if(si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS))
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS, 0, false, ccdPass, false, outputs);
				}
				else
				{
					//If the accumulatedForce is less than the threshold in the current frame and the accumulatedForce is large than the threshold in the previous frame, 
					//and the user request notify for found event, we will raise eNOTIFY_THRESHOLD_FORCE_LOST
					if(si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST))
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST, 0, false, ccdPass, false, outputs);
				}
			}
		}
	}
}

void Sc::Scene::afterIntegration(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::afterIntegration", mContextId);

	mLLContext->getTransformCache().resetChangedState(); //Reset the changed state. If anything outside of the GPU kernels updates any shape's transforms, this will be raised again
	getBoundsArray().resetChangedState();

	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	Bp::BoundsArray& boundArray = getBoundsArray();

	{
		PX_PROFILE_ZONE("AfterIntegration::lockStage", mContextId);
		mLLContext->getLock().lock();
		
		{
			PX_PROFILE_ZONE("SimController", mContextId);
			mSimulationController->updateScBodyAndShapeSim(cache, boundArray, continuation);
		}

		const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

		const PxU32 rigidBodyOffset = BodySim::getRigidBodyOffset();

		const PxU32 numBodiesToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

		const PxNodeIndex*const deactivatingIndices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

		PxU32 previousNumBodiesToDeactivate = mNumDeactivatingNodes[IG::Node::eRIGID_BODY_TYPE];

		{
			PX_PROFILE_ZONE("AfterIntegration::deactivateStage", mContextId);

			PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
			for(PxU32 i = previousNumBodiesToDeactivate; i < numBodiesToDeactivate; i++)
			{
				PxsRigidBody* rigid = islandSim.getRigidBody(deactivatingIndices[i]);
				BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
				//we need to set the rigid body back to the previous pose for the deactivated objects. This emulates the previous behavior where island gen ran before the solver, ensuring
				//that bodies that should be deactivated this frame never reach the solver. We now run the solver in parallel with island gen, so objects that should be deactivated this frame
				//still reach the solver and are integrated. However, on the frame when they should be deactivated, we roll back to their state at the beginning of the frame to ensure that the
				//user perceives the same behavior as before.

				PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();

				//if(!islandSim.getNode(bodySim->getNodeIndex()).isActive())
				rigid->setPose(rigid->getLastCCDTransform());

				bodySim->updateCached(&changedAABBMgrActorHandles);
				updateBodySim(*bodySim);

				//solver is running in parallel with IG(so solver might solving the body which IG identify as deactivatedNodes). After we moved sleepCheck into the solver after integration, sleepChecks
				//might have processed bodies that are now considered deactivated. This could have resulted in either freezing or unfreezing one of these bodies this frame, so we need to process those
				//events to ensure that the SqManager's bounds arrays are consistently maintained. Also, we need to clear the frame flags for these bodies.

				if(rigid->isFreezeThisFrame())
					bodySim->freezeTransforms(&mAABBManager->getChangedAABBMgActorHandleMap());

				//KS - the IG deactivates bodies in parallel with the solver. It appears that under certain circumstances, the solver's integration (which performs
				//sleep checks) could decide that the body is no longer a candidate for sleeping on the same frame that the island gen decides to deactivate the island
				//that the body is contained in. This is a rare occurrence but the behavior we want to emulate is that of IG running before solver so we should therefore
				//permit the IG to make the authoritative decision over whether the body should be active or inactive.
				bodyCore.wakeCounter = 0.0f;
				bodyCore.linearVelocity = PxVec3(0.0f);
				bodyCore.angularVelocity = PxVec3(0.0f);

				rigid->clearAllFrameFlags();
			}
		}

		updateKinematicCached(continuation);

		mLLContext->getLock().unlock();
	}

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);

	if(nbActiveArticulations)
		mSimulationController->updateArticulationAfterIntegration(mLLContext, mAABBManager, mCcdBodies, continuation, islandSim, mDt);

	const PxU32 numArticsToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	const PxNodeIndex*const deactivatingArticIndices = islandSim.getNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	PxU32 previousNumArticsToDeactivate = mNumDeactivatingNodes[IG::Node::eARTICULATION_TYPE];

	for(PxU32 i = previousNumArticsToDeactivate; i < numArticsToDeactivate; ++i)
	{
		ArticulationSim* artic = islandSim.getArticulationSim(deactivatingArticIndices[i]);

		artic->putToSleep();
	}

	//PxU32 previousNumClothToDeactivate = mNumDeactivatingNodes[IG::Node::eFEMCLOTH_TYPE];
	//const PxU32 numClothToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eFEMCLOTH_TYPE);
	//const IG::NodeIndex*const deactivatingClothIndices = islandSim.getNodesToDeactivate(IG::Node::eFEMCLOTH_TYPE);

	//for (PxU32 i = previousNumClothToDeactivate; i < numClothToDeactivate; ++i)
	//{
	//	FEMCloth* cloth = islandSim.getLLFEMCloth(deactivatingClothIndices[i]);
	//	mSimulationController->deactivateCloth(cloth);
	//}

	//PxU32 previousNumSoftBodiesToDeactivate = mNumDeactivatingNodes[IG::Node::eSOFTBODY_TYPE];
	//const PxU32 numSoftBodiesToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eSOFTBODY_TYPE);
	//const IG::NodeIndex*const deactivatingSoftBodiesIndices = islandSim.getNodesToDeactivate(IG::Node::eSOFTBODY_TYPE);

	//for (PxU32 i = previousNumSoftBodiesToDeactivate; i < numSoftBodiesToDeactivate; ++i)
	//{
	//	Dy::SoftBody* softbody = islandSim.getLLSoftBody(deactivatingSoftBodiesIndices[i]);
	//	printf("after Integration: Deactivating soft body %i\n", softbody->getGpuRemapId());
	//	//mSimulationController->deactivateSoftbody(softbody);
	//	softbody->getSoftBodySim()->setActive(false, 0);
	//}

	PX_PROFILE_STOP_CROSSTHREAD("Basic.dynamics", mContextId);

	checkForceThresholdContactEvents(0); 		
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::fireOnAdvanceCallback()
{
	if(!mSimulationEventCallback)
		return;

	const PxU32 nbPosePreviews = mPosePreviewBodies.size();
	if(!nbPosePreviews)
		return;

	mClientPosePreviewBodies.clear();
	mClientPosePreviewBodies.reserve(nbPosePreviews);

	mClientPosePreviewBuffer.clear();
	mClientPosePreviewBuffer.reserve(nbPosePreviews);
		
	const BodySim*const* PX_RESTRICT posePreviewBodies = mPosePreviewBodies.getEntries();
	for(PxU32 i=0; i<nbPosePreviews; i++)
	{
		const BodySim& b = *posePreviewBodies[i];
		if(!b.isFrozen())
		{
			PxsBodyCore& c = b.getBodyCore().getCore();
			mClientPosePreviewBodies.pushBack(static_cast<const PxRigidBody*>(b.getPxActor()));
			// PT:: tag: scalar transform*transform
			mClientPosePreviewBuffer.pushBack(c.body2World * c.getBody2Actor().getInverse());
		}
	}

	const PxU32 bodyCount = mClientPosePreviewBodies.size();
	if(bodyCount)
		mSimulationEventCallback->onAdvance(mClientPosePreviewBodies.begin(), mClientPosePreviewBuffer.begin(), bodyCount);
}

void Sc::Scene::finalizationPhase(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.sceneFinalization", mContextId);

	if(mCCDContext)
	{
		if(mSimulationController->mGPU)	// PT: skip this on CPU, see empty CPU function called in updateBodySim
		{
			//KS - force simulation controller to update any bodies updated by the CCD. When running GPU simulation, this would be required
			//to ensure that cached body states are updated
			const PxU32 nbUpdatedBodies = mCCDContext->getNumUpdatedBodies();
			PxsRigidBody*const* updatedBodies = mCCDContext->getUpdatedBodies();

			const PxU32 rigidBodyOffset = BodySim::getRigidBodyOffset();			

			for(PxU32 a=0; a<nbUpdatedBodies; ++a)
			{
				BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(updatedBodies[a]) - rigidBodyOffset);
				updateBodySim(*bodySim);
			}
		}

		mCCDContext->clearUpdatedBodies();
	}

	fireOnAdvanceCallback();  // placed here because it needs to be done after sleep check and after potential CCD passes

	checkConstraintBreakage(); // Performs breakage tests on breakable constraints

	PX_PROFILE_STOP_CROSSTHREAD("Basic.rigidBodySolver", mContextId);

	mTaskPool.clear();

	mReportShapePairTimeStamp++;	// important to do this before fetchResults() is called to make sure that delayed deleted actors/shapes get
									// separate pair entries in contact reports
}

///////////////////////////////////////////////////////////////////////////////
