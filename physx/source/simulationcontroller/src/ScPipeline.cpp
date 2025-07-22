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
	#include "PxDeformableSurface.h"
	#include "ScDeformableSurfaceSim.h"
	#include "DyDeformableSurface.h"
	#include "PxDeformableVolume.h"
	#include "ScDeformableVolumeSim.h"
	#include "DyDeformableVolume.h"
	#include "ScParticleSystemSim.h"
	#include "DyParticleSystem.h"
#endif

#include "ScArticulationCore.h"
#include "ScArticulationSim.h"
#include "ScConstraintCore.h"
#include "ScConstraintSim.h"
#include "DyIslandManager.h"

using namespace physx;
using namespace Cm;
using namespace Dy;
using namespace Sc;
using namespace Bp;

PX_IMPLEMENT_OUTPUT_ERROR

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

	const bool useGpu = isUsingGpuDynamicsOrBp();

	mRigidBodyNarrowPhase.setContinuation(continuation);
	mPreRigidBodyNarrowPhase.setContinuation(&mRigidBodyNarrowPhase);
	if(useGpu)
		mUpdateShapes.setContinuation(&mPreRigidBodyNarrowPhase);

	mRigidBodyNarrowPhase.removeReference();
	mPreRigidBodyNarrowPhase.removeReference();
	if(useGpu)
		mUpdateShapes.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateShapes(PxBaseTask* continuation)
{
	PX_ASSERT(isUsingGpuDynamicsOrBp());	// PT: this is not called anymore in the CPU pipeline

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
	BoundsArray&		mBoundsArray;
	ShapeSim*			mShapes[MaxShapes];
	PxU32				mNbShapes;

	DirtyShapeUpdatesTask(PxU64 contextID, PxsTransformCache& cache, BoundsArray& boundsArray) : 
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

static DirtyShapeUpdatesTask* createDirtyShapeUpdateTask(Cm::FlushPool& pool, PxU64 contextID, PxsTransformCache& cache, BoundsArray& boundsArray)
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
	BoundsArray& boundsArray = mAABBManager->getBoundsArray();

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

	mLLContext->updateContactManager(mDt, mHasContactDistanceChanged, continuation, 
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
	mSimulationController->updateBoundsAndShapes(*mAABBManager, isDirectGPUAPIInitialized());
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::setupBroadPhaseFirstAndSecondPassTasks(PxBaseTask* continuation)
{
	// PT: on the CPU, mBpSecondPass only starts mBpUpdate, so we can use that directly.
	if(isUsingGpuDynamicsOrBp())
	{
		mBpSecondPass.setContinuation(continuation);
		mBpFirstPass.setContinuation(&mBpSecondPass);

		mBpSecondPass.removeReference();
		mBpFirstPass.removeReference();
	}
	else
	{
		mBpUpdate.setContinuation(continuation);
		mBpFirstPass.setContinuation(&mBpUpdate);

		mBpUpdate.removeReference();
		mBpFirstPass.removeReference();
	}
}

void Sc::Scene::broadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.broadPhase", mContextId);

#if PX_SUPPORT_GPU_PHYSX
	gpu_updateBounds();
#endif

	mCCDBp = false;

	setupBroadPhaseFirstAndSecondPassTasks(continuation);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::broadPhaseFirstPass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.broadPhaseFirstPass", mContextId);
	const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
	mAABBManager->updateBPFirstPass(numCpuTasks, mLLContext->getTaskPool(), mHasContactDistanceChanged, continuation);
	
	// AD: this combines the update flags of the normal pipeline with the update flags
	// marking updated bounds for the direct-GPU API.
	if (isDirectGPUAPIInitialized())
	{
		mSimulationController->mergeChangedAABBMgHandle();
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::broadPhaseSecondPass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.broadPhaseSecondPass", mContextId);

	PX_ASSERT(isUsingGpuDynamicsOrBp());	// PT: this is not called anymore in the CPU pipeline

	mBpUpdate.setContinuation(continuation);
	mPreIntegrate.setContinuation(&mBpUpdate);

	mPreIntegrate.removeReference();
	mBpUpdate.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::preIntegrate(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.preIntegrate", mContextId);

	PX_ASSERT(isUsingGpuDynamicsOrBp());	// PT: this is not called anymore in the CPU pipeline

	if(!mCCDBp)
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

namespace
{
	// PT: design that doesn't use a bitmap and does not require a MaxPairs limit per task. In this version the surviving pairs
	// are moved to the front of the input buffers.
	class OverlapFilterTask : public Cm::Task
	{
	public:
		// PT: TODO: we already have an old JIRA ticket about this but this design here is one reason why PhysX doesn't
		// scale well to many cores. Imagine you have a a relatively heavy scene with 4K new pairs to process this frame.
		// We fill up MaxPairs per task no matter what. With MaxPairs = 512 we won't use more than 1 task below that limit.
		// With 4K pairs we're going to use at best 4096/512 = 8 tasks. That's it. You could have 72 cores available and
		// we'd still create only 8 tasks. Very questionable design here.
		//static const PxU32 MaxPairs = 512;
		static const PxU32 MaxPairs = 64;		// PT: lower number to scale to more threads
		//static const PxU32 MaxPairs = 32;
		const NPhaseCore*		mNPhaseCore;
		AABBOverlap*			mPairs;		// PT: pointers to sections of AABBManagerBase::mCreatedOverlaps
		const PxU32				mNbToProcess;
		FilterInfo*				mFinfo;		// PT: pointers to sections of Sc::Scene::mFilterInfo.begin()
		PxU32					mNbToKeep;
		PxU32					mNbToSuppress;
		OverlapFilterTask*		mNext;

		OverlapFilterTask(PxU64 contextID, NPhaseCore* nPhaseCore, FilterInfo* fInfo, AABBOverlap* pairs, PxU32 nbToProcess) :
			Cm::Task		(contextID),
			mNPhaseCore		(nPhaseCore),
			mPairs			(pairs),
			mNbToProcess	(nbToProcess),
			mFinfo			(fInfo),
			mNbToKeep		(0),
			mNbToSuppress	(0),
			mNext			(NULL)
		{
		}

		virtual void runInternal()
		{
			// PT: after this call we have mNbToKeep + mNbToSuppress surviving pairs moved to the start of mPairs,
			// with corresponding filtering data at the start of mFinfo.
			mNPhaseCore->runOverlapFilters(mNbToProcess, mPairs, mFinfo, mNbToKeep, mNbToSuppress);
		}

		virtual const char* getName() const { return "OverlapFilterTask"; }
	};
}

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
				const AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(ElementType::eTRIGGER, createdOverlapCount);
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
			AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(ElementType::eSHAPE, createdOverlapCount);

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
			// PT: the same design is now used for these other tasks
			mOverlapCreatedTaskHead = NULL;
			mIslandInsertionTaskHead = NULL;

			if(createdOverlapCount)
			{
				mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;

				Cm::FlushPool& flushPool = mLLContext->getTaskPool();

				// PT: temporary data, similar to mOverlapFilterTaskHead. Will be filled with filter info for each pair by the OverlapFilterTask.
				// PT: TODO: revisit this pattern forceSize_Unsafe / reserve / forceSize_Unsafe - why??
				mFilterInfo.forceSize_Unsafe(0);
				mFilterInfo.reserve(createdOverlapCount);
				mFilterInfo.forceSize_Unsafe(createdOverlapCount);

				// PT: TASK-CREATION TAG
				// PT: TODO: revisit task creation here
				const PxU32 nbPairsPerTask = OverlapFilterTask::MaxPairs;
				OverlapFilterTask* previousTask = NULL;
				for(PxU32 a=0; a<createdOverlapCount; a+=nbPairsPerTask)
				{
					const PxU32 nbToProcess = PxMin(createdOverlapCount - a, nbPairsPerTask);
					OverlapFilterTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OverlapFilterTask)), OverlapFilterTask)(mContextId, mNPhaseCore, mFilterInfo.begin() + a, p + a, nbToProcess);

					startTask(task, &mPreallocateContactManagers);

					// PT: setup a linked-list of OverlapFilterTasks, will be parsed in preallocateContactManagers
					updateTaskLinkedList(previousTask, task, mOverlapFilterTaskHead);
				}
			}
		}

		mPreallocateContactManagers.removeReference();

#if PX_ENABLE_SIM_STATS
		mLLContext->getSimStats().mGpuDynamicsFoundLostPairs = mAABBManager->getGpuDynamicsLostFoundPairsStats();
		mLLContext->getSimStats().mGpuDynamicsTotalAggregatePairs = mAABBManager->getGpuDynamicsTotalAggregatePairsStats();
		mLLContext->getSimStats().mGpuDynamicsFoundLostAggregatePairs = mAABBManager->getGpuDynamicsLostFoundAggregatePairsStats();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
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
		const AABBOverlap*			mPairs;
		const FilterInfo*			mFinfo;
		PxsContactManager**			mContactManagers;
		ShapeInteraction**			mShapeInteractions;
		ElementInteractionMarker**	mInteractionMarkers;
		const PxU32					mNbToProcess;
		// PT: we maintain these to quickly compute afterwards how many shape interactions we're dealing with (to preallocate, etc)
		PxU32						mNbShapeInteractions;
		OnOverlapCreatedTask*		mNext;

		OnOverlapCreatedTask(PxU64 contextID, NPhaseCore* nPhaseCore, const AABBOverlap* pairs, const FilterInfo* fInfo, PxsContactManager** contactManagers,
							ShapeInteraction** shapeInteractions, ElementInteractionMarker** interactionMarkers, PxU32 nbToProcess) :
			Cm::Task			(contextID),
			mNPhaseCore			(nPhaseCore),
			mPairs				(pairs),
			mFinfo				(fInfo),
			mContactManagers	(contactManagers),
			mShapeInteractions	(shapeInteractions),
			mInteractionMarkers	(interactionMarkers),
			mNbToProcess		(nbToProcess),
			mNbShapeInteractions(0),
			mNext				(NULL)
		{
		}

		virtual void runInternal()
		{
			PxsContactManager** currentCm = mContactManagers;
			ShapeInteraction** currentSI = mShapeInteractions;
			ElementInteractionMarker** currentEI = mInteractionMarkers;

			const PxU32 nbToProcess = mNbToProcess;
			for(PxU32 i=0; i<nbToProcess; i++)
			{
				const AABBOverlap& pair = mPairs[i];
				// PT: TODO: why did we switch 0/1 here? => undoing this makes FilteringTestsIllegalFlags.eKILL_and_eSUPPRESS fail
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

			// PT: TODO: perhaps an atomic would be better actually
			mNbShapeInteractions = PxU32(currentSI - mShapeInteractions);
		}

		virtual const char* getName() const { return "OnOverlapCreatedTask"; }
	};
}

void Sc::Scene::preallocateContactManagers(PxBaseTask* continuation)
{
	//Iterate over all filter tasks and work out how many pairs we need...
	PxU32 totalCreatedPairs = 0;
	PxU32 totalSuppressPairs = 0;
	{
		OverlapFilterTask* task = mOverlapFilterTaskHead;
		while(task)
		{
			totalCreatedPairs += task->mNbToKeep;
			totalSuppressPairs += task->mNbToSuppress;
			task = task->mNext;
		}
	}

	{
		//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
		// PT: this has to do with the way we dereference currentCm, currentSI and currentEI in OnOverlapCreatedTask
		// before we know which type of interaction will be created. That is, we need room for at least one of each type
		// even if no interaction of that type will be created.
		// PT: TODO: don't we preallocate 2 to 3 times as much memory as needed here then?
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
		// PT: TODO: revisit this pattern forceSize_Unsafe / reserve / forceSize_Unsafe - why??
	}

	// PT: beware, we compacted this array during filtering so the overlap count is misleading now. Only reliable as zero / non-zero.
	PxU32 overlapCount_unused;
	AABBOverlap* const PX_RESTRICT pairs = mAABBManager->getCreatedOverlaps(ElementType::eSHAPE, overlapCount_unused);
	if(!overlapCount_unused)
		return;

	struct OverlapTaskCreator
	{
		NPhaseCore* const					mCore;
		Cm::PoolList<PxsContactManager>&	mCMPool;
		Cm::FlushPool&						mFlushPool;
		PxBaseTask* const					mContinuation;
		const AABBOverlap* const			mPairs;
		const FilterInfo* const				mFilterInfo;
		PxsContactManager** const			mContactManagers;
		ShapeInteraction** const			mShapeInteractions;
		ElementInteractionMarker** const	mMarkerIteractions;
		const PxU64							mContextId;

		OverlapTaskCreator(
			NPhaseCore* const PX_RESTRICT core, Cm::PoolList<PxsContactManager>& cmPool, Cm::FlushPool& flushPool, PxBaseTask* const continuation,
			const  AABBOverlap* const pairs, const  FilterInfo* const fInfo,
			PxsContactManager** const cms, ShapeInteraction** const shapeInter, ElementInteractionMarker** const markerIter,
			PxU64 contextId) :
			mCore(core), mCMPool(cmPool), mFlushPool(flushPool), mContinuation(continuation),
			mPairs(pairs), mFilterInfo(fInfo),
			mContactManagers(cms), mShapeInteractions(shapeInter), mMarkerIteractions(markerIter),
			mContextId(contextId)
		{
		}

		OnOverlapCreatedTask* processBatch(	PxU32 nextCreatedOverlapCount, PxU32 nextCreatedStartIdx, PxU32 nextSuppressedStartIdx,
											PxU32 createdCurrIdx, PxU32& createdStartIdx, PxU32 suppressedCurrIdx, PxU32& suppressedStartIdx, PxU32 batchSize)
		{
			OnOverlapCreatedTask* createTask = PX_PLACEMENT_NEW(mFlushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(mContextId, mCore,
																																		mPairs + nextCreatedOverlapCount,
																																		mFilterInfo + nextCreatedOverlapCount,
																																		mContactManagers + nextCreatedStartIdx,
																																		mShapeInteractions + nextCreatedStartIdx,
																																		mMarkerIteractions + nextSuppressedStartIdx,
																																		batchSize);
			const PxU32 nbToCreate = createdCurrIdx - createdStartIdx;
			const PxU32 nbToSuppress = suppressedCurrIdx - suppressedStartIdx;

			mCMPool.preallocate(nbToCreate, mContactManagers + createdStartIdx);

			for (PxU32 i = 0; i < nbToCreate; ++i)
				mShapeInteractions[createdStartIdx + i] = mCore->mShapeInteractionPool.allocate();

			for (PxU32 i = 0; i < nbToSuppress; ++i)
				mMarkerIteractions[suppressedStartIdx + i] = mCore->mInteractionMarkerPool.allocate();
				
			createdStartIdx = createdCurrIdx;
			suppressedStartIdx = suppressedCurrIdx;

			startTask(createTask, mContinuation);
			return createTask;
		}
	};

	const PxU32 nbPairsPerTask = 256;	// PT: TODO: refine this

	PxsContactManager** const cms = mPreallocatedContactManagers.begin();
	ShapeInteraction** const shapeInter = mPreallocatedShapeInteractions.begin();
	ElementInteractionMarker** const markerIter = mPreallocatedInteractionMarkers.begin();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();
	Cm::PoolList<PxsContactManager>& cmPool = mLLContext->getContactManagerPool();

	FilterInfo* const fInfo = mFilterInfo.begin();

	OverlapTaskCreator overlapTaskCreator(mNPhaseCore, cmPool, flushPool, continuation, pairs, fInfo, cms, shapeInter, markerIter, mContextId);

	PxU32 batchSize = 0;
	PxU32 suppressedStartIdx = 0;
	PxU32 createdStartIdx = 0;
	PxU32 suppressedCurrIdx = 0;
	PxU32 createdCurrIdx = 0;
	PxU32 createdOverlapCount = 0;

	PxU32 nextCreatedOverlapCount = 0;
	PxU32 nextCreatedStartIdx = 0;
	PxU32 nextSuppressedStartIdx = 0;

	OnOverlapCreatedTask* previousTask = NULL;

	// PT: TASK-CREATION TAG
	// PT: this compacts the pairs that passed filtering to the front of the pairs & fInfo arrays
	OverlapFilterTask* filterTask = mOverlapFilterTaskHead;
	while(filterTask)
	{
		if(filterTask->mNbToKeep || filterTask->mNbToSuppress)
		{
			// PT: we pre-compacted surviving pairs in each filtering task so a memcopy is enough here now.
			const PxU32 nb = filterTask->mNbToKeep + filterTask->mNbToSuppress;
			if(pairs + createdOverlapCount != filterTask->mPairs)	// PT: always happens for first task, sometimes for all tasks if nothing was filtered
			{
				PxMemCopy(pairs + createdOverlapCount, filterTask->mPairs, sizeof(AABBOverlap) * nb);
				PxMemCopy(fInfo + createdOverlapCount, filterTask->mFinfo, sizeof(FilterInfo) * nb);
			}
			createdOverlapCount += nb;
			batchSize += nb;

			suppressedCurrIdx += filterTask->mNbToSuppress;
			createdCurrIdx += filterTask->mNbToKeep;

			if(batchSize >= nbPairsPerTask)
			{
				OnOverlapCreatedTask* task = overlapTaskCreator.processBatch(	nextCreatedOverlapCount, nextCreatedStartIdx, nextSuppressedStartIdx,
																				createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize);

				updateTaskLinkedList(previousTask, task, mOverlapCreatedTaskHead);

				nextCreatedOverlapCount = createdOverlapCount;
				nextCreatedStartIdx = createdStartIdx;
				nextSuppressedStartIdx = suppressedStartIdx;
				batchSize = 0;
			}
		}
		filterTask = filterTask->mNext;
	}

	if(batchSize)
	{
		OnOverlapCreatedTask* task = overlapTaskCreator.processBatch(	nextCreatedOverlapCount, nextCreatedStartIdx, nextSuppressedStartIdx,
																		createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize);

		updateTaskLinkedList(previousTask, task, mOverlapCreatedTaskHead);
	}
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

namespace
{
	PX_COMPILE_TIME_ASSERT(sizeof(IG::EdgeIndex) <= sizeof(AABBOverlap));	// PT: because we're going to store EdgeIndex in the previous AABBOverlap buffer
	class IslandInsertionTask : public Cm::Task
	{
	public:
		IslandInsertionTask*		mNext;
		IG::SimpleIslandManager*	mSimpleIslandManager;
		ShapeInteraction**			mPreallocatedShapeInteractions;
		const IG::EdgeIndex*		mHandles;
		AABBOverlap*				mPairs;
		const PxU32					mNbToProcess;
		PxU32						mNbDelayed;
		DelayedGPUTypes&			mGPUTypes;

		IslandInsertionTask(PxU64 contextID,  IG::SimpleIslandManager* simpleIslandManager, ShapeInteraction** preallocatedShapeInteractions,
			const IG::EdgeIndex* handles, AABBOverlap* pairs, DelayedGPUTypes& gpuTypes, PxU32 nbToProcess) :
			Cm::Task						(contextID),
			mNext							(NULL),
			mSimpleIslandManager			(simpleIslandManager),
			mPreallocatedShapeInteractions	(preallocatedShapeInteractions),
			mHandles						(handles),
			mPairs							(pairs),
			mNbToProcess					(nbToProcess),
			mNbDelayed						(0),
			mGPUTypes						(gpuTypes)
		{
		}

		PX_FORCE_INLINE	IG::EdgeIndex*	getDelayed()	{ return reinterpret_cast<IG::EdgeIndex*>(mPairs);	}

		virtual void runInternal()
		{
			// PT: the pairs buffer we used to create the shape interactions in OnOverlapCreatedTask is now free to reuse.
			// By construction we have at least one AABBOverlap per shape interaction so there's enough space for edge indices.
			IG::EdgeIndex* delayed = getDelayed();

			const PxU32 nb = mNbToProcess;
			const IG::EdgeIndex* handles = mHandles;
			for(PxU32 a=0; a<nb; a++)
			{
				// PT: this is the first part of Sc::Scene::islandInsertion() multi-threaded
				ShapeInteraction* interaction = getUsedPointer(mPreallocatedShapeInteractions[a]);
				if(interaction)
				{
					const ActorSim& bs0 = interaction->getActor0();
					const ActorSim& bs1 = interaction->getActor1();

					const PxActorType::Enum actorTypeLargest = PxMax(bs0.getActorType(), bs1.getActorType());

					PxNodeIndex nodeIndexB;
					if(!bs1.isStaticRigid())
						nodeIndexB = bs1.getNodeIndex();

					const IG::Edge::EdgeType type = getInteractionEdgeType(actorTypeLargest);

					PxsContactManager* contactManager = const_cast<PxsContactManager*>(interaction->getContactManager());
					// PT: non-MT version is:
					//const IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(contactManager, bs0.getNodeIndex(), nodeIndexB, interaction, type);
					const IG::EdgeIndex edgeIdx = *handles++;
					const bool isDirty = mSimpleIslandManager->addPreallocatedContactManager(edgeIdx, contactManager, bs0.getNodeIndex(), nodeIndexB, interaction, type);
					PX_ASSERT(!contactManager || contactManager->getWorkUnit().mEdgeIndex == edgeIdx);

					if(isDirty)
						*delayed++ = edgeIdx;	// PT: record edges for last part of IslandSim::addConnection() that we couldn't easily MT

					// PT: TODO: do we really need to store the edge index twice here?
					interaction->mEdgeIndex = edgeIdx;

					//If it is a deformable volume or particle overlap, treat it as a contact for now (we can hook up touch found/lost events later maybe)
					if(actorTypeLargest > PxActorType::eARTICULATION_LINK)
					{
						// PT: codepath needed for AttachmentTests.DeformableSurfaceFiltering_GPU, also reached in ParticleCollisionTests.RigidDeltaAccum etc
						mGPUTypes.mLock.lock();	// PT: this is not a common case so we just use a plain mutex for now
							// PT: record what we'll need to call mSimpleIslandManager->setEdgeConnected(edgeIdx, type) later
							DelayedGPUTypes::Data& data = mGPUTypes.mDelayed.insert();
							data.mEdgeIndex = edgeIdx;
							data.mType = type;
						mGPUTypes.mLock.unlock();
					}
				}
			}
			mNbDelayed = PxU32(delayed - getDelayed());
		}

		virtual const char* getName() const { return "IslandInsertionTask"; }
	};
}

void Sc::Scene::postBroadPhaseStage2(PxBaseTask* continuation)
{
	// PT: TODO: can we overlap this with something?
	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();

	// PT: don't bother running tasks if we don't need to
	const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
	const PxU32 nbInteractionMarkers = mPreallocatedInteractionMarkers.size();
	const bool runRegisterSceneInteractions = nbShapeIdxCreated || nbInteractionMarkers;

	const PxU32 nbCmsCreated = mPreallocatedContactManagers.size();
	const bool runRegisterContactManagers = nbCmsCreated!=0;

	// PT: islandInsertion / registerContactManagers / registerInteractions / registerSceneInteractions run in parallel
	mIslandInsertion.setContinuation(continuation);
	if(runRegisterContactManagers)
		mRegisterContactManagers.setContinuation(continuation);
	if(runRegisterSceneInteractions)
	{
		mRegisterInteractions.setContinuation(continuation);
		mRegisterSceneInteractions.setContinuation(continuation);
	}

	// PT: start these ones first
	if(runRegisterContactManagers)
		mRegisterContactManagers.removeReference();
	if(runRegisterSceneInteractions)
	{
		mRegisterInteractions.removeReference();
		mRegisterSceneInteractions.removeReference();
	}

	{
		PX_PROFILE_ZONE("mIslandInsertion prep", mContextId);
		// PT: TODO: maybe replace this loop with atomics in overlap created tasks
		PxU32 totalNbShapeInteractions = 0;
		{
			OnOverlapCreatedTask* task = mOverlapCreatedTaskHead;
			while(task)
			{
				totalNbShapeInteractions += task->mNbShapeInteractions;
				task = task->mNext;
			}
		}

		mGPUTypes.mDelayed.clear();	// PT: TODO: revisit memory usage (clear? reset? scratch block?)
		mPreallocatedHandles.clear();

		if(totalNbShapeInteractions)
		{
			IG::EdgeIndex* handles;
			{
				PX_PROFILE_ZONE("preallocateContactManagers", mContextId);
				// PT: preallocate buffers initially needed in mSimpleIslandManager->addContactManager()
				mPreallocatedHandles.resizeUninitialized(totalNbShapeInteractions);	// PT: look, no need for "forceSize_unsafe"...
				handles = mPreallocatedHandles.begin();
				mSimpleIslandManager->preallocateContactManagers(totalNbShapeInteractions, handles);
			}

			{
				PX_PROFILE_ZONE("CreateIslandInsertionTask", mContextId);

				Cm::FlushPool& flushPool = mLLContext->getTaskPool();

				IslandInsertionTask* previousTask = NULL;
				// PT: we create one IslandInsertionTask for each OnOverlapCreatedTask
				// TODO: merge the two? revisit this choice?
				OnOverlapCreatedTask* task = mOverlapCreatedTaskHead;
				while(task)
				{
					if(task->mNbShapeInteractions)
					{
						IslandInsertionTask* newTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(IslandInsertionTask)), IslandInsertionTask)(
							mContextId, mSimpleIslandManager, task->mShapeInteractions, handles, const_cast<AABBOverlap*>(task->mPairs), mGPUTypes, task->mNbShapeInteractions);

						startTask(newTask, &mIslandInsertion);
						handles += task->mNbShapeInteractions;

						updateTaskLinkedList(previousTask, newTask, mIslandInsertionTaskHead);
					}
					task = task->mNext;
				}
			}
		}
	}

	mIslandInsertion.removeReference();

	//Release unused Cms back to the pool (later, this needs to be done in a thread-safe way from multiple worker threads
	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.release", mContextId);

		{
			PxU32 nb = mPreallocatedContactManagers.size();
			PxsContactManager** managers = mPreallocatedContactManagers.begin();
			Cm::PoolList<PxsContactManager>& pool = mLLContext->getContactManagerPool();
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
			PxPool2<ElementInteractionMarker, 4096>& pool = mNPhaseCore->mInteractionMarkerPool;
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

	// PT: TODO: get rid of this one
	PxU32 nb = 0;
	{
		PX_PROFILE_ZONE("ParseOnOverlapCreatedTask", mContextId);
		OnOverlapCreatedTask* task = mOverlapCreatedTaskHead;
		while(task)
		{
			nb += task->mNbShapeInteractions;
			task = task->mNext;
		}
	}

	if(nb)
	{
		// PT: process delayed work not done directly in IslandInsertionTask
		{
			// PT: TODO: check how costly this is
			PX_PROFILE_ZONE("Process delayed dirty edges", mContextId);

			IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();
			IslandInsertionTask* task = mIslandInsertionTaskHead;
			while(task)
			{
				if(task->mNbDelayed)
					islandSim.addDelayedDirtyEdges(task->mNbDelayed, task->getDelayed());

				task = task->mNext;
			}
		}

		{
			PX_PROFILE_ZONE("Process delayed GPU types", mContextId);

			const PxU32 nbGpu = mGPUTypes.mDelayed.size();
			for(PxU32 i=0;i<nbGpu;i++)
			{
				const DelayedGPUTypes::Data& data = mGPUTypes.mDelayed[i];
				mSimpleIslandManager->setEdgeConnected(data.mEdgeIndex, data.mType);
			}
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
	PX_ASSERT(nbCmsCreated);	// PT: otherwise we should have skipped the task entirely
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

	PX_ASSERT(mPreallocatedShapeInteractions.size() || mPreallocatedInteractionMarkers.size());	// PT: otherwise we should have skipped the task entirely

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

	PX_ASSERT(mPreallocatedShapeInteractions.size() || mPreallocatedInteractionMarkers.size());	// PT: otherwise we should have skipped the task entirely

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

	AABBManagerBase* aabbMgr = mAABBManager;

	PxU32 nbLostPairs = 0;
	for(PxU32 i=0; i<ElementType::eCOUNT; i++)
	{
		PxU32 destroyedOverlapCount;
		aabbMgr->getDestroyedOverlaps(ElementType::Enum(i), destroyedOverlapCount);
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
			AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);

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
							si->clearIslandGenData(*mSimpleIslandManager);
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
			AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(ElementType::eTRIGGER, destroyedOverlapCount);

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
		mFinalizationPhase.setContinuation(continuation);

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

		const bool useGpu = isUsingGpuDynamicsOrBp();

		mPostSolver.setContinuation(&mAfterIntegration);
		if(useGpu)
		{
			mUpdateSimulationController.setContinuation(&mPostSolver);
			mUpdateDynamicsPostPartitioning.setContinuation(&mUpdateSimulationController);
			mUpdateDynamics.setContinuation(&mUpdateDynamicsPostPartitioning);

			mUpdateBodies.setContinuation(&mUpdateDynamics);
			mSolver.setContinuation(&mUpdateBodies);
		}
		else
		{
			mUpdateDynamics.setContinuation(&mPostSolver);

			mSolver.setContinuation(&mUpdateDynamics);
		}
#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
		mIslandGen.setContinuation(&mSolver);
#else
		mPostIslandGen.setContinuation(&mSolver);
		mIslandGen.setContinuation(&mPostIslandGen);
#endif
		mPostNarrowPhase.setContinuation(&mIslandGen);
		mSecondPassNarrowPhase.setContinuation(&mPostNarrowPhase);

		mFinalizationPhase.removeReference();
		mAfterIntegration.removeReference();
		mPostSolver.removeReference();
		if(useGpu)
		{
			mUpdateSimulationController.removeReference();
			mUpdateDynamicsPostPartitioning.removeReference();
		}
		mUpdateDynamics.removeReference();
		if(useGpu)
			mUpdateBodies.removeReference();
		mSolver.removeReference();
#if !USE_SPLIT_SECOND_PASS_ISLAND_GEN
		mPostIslandGen.removeReference();
#endif
		mIslandGen.removeReference();
		mPostNarrowPhase.removeReference();
		mSecondPassNarrowPhase.removeReference();
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::activateEdgesInternal(IG::Edge::EdgeType type)
{
	const IG::IslandSim& speculativeSim = mSimpleIslandManager->getSpeculativeIslandSim();

	const IG::EdgeIndex* activatingEdges = speculativeSim.getActivatedEdges(type);
	const PxU32 nbActivatingEdges = speculativeSim.getNbActivatedEdges(type);

	for(PxU32 i = 0; i < nbActivatingEdges; ++i)
	{
		Interaction* interaction = mSimpleIslandManager->getInteractionFromEdgeIndex(activatingEdges[i]);

		if(interaction && !interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			if(speculativeSim.getEdge(activatingEdges[i]).isActive())
			{
				const bool proceed = activateInteraction(interaction);

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

			//KS - only wake contact managers based on speculative state to trigger contact gen. Waking actors based on accurate state
			//should activate and joints.
			{
				//Wake speculatively based on rigid contacts, soft contacts and particle contacts
				activateEdgesInternal(IG::Edge::eCONTACT_MANAGER);
#if PX_SUPPORT_GPU_PHYSX
				activateEdgesInternal(IG::Edge::eSOFT_BODY_CONTACT);
				activateEdgesInternal(IG::Edge::eFEM_CLOTH_CONTACT);
				activateEdgesInternal(IG::Edge::ePARTICLE_SYSTEM_CONTACT);
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

void Sc::Scene::islandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::islandGen", mContextId);

	//mLLContext->runModifiableContactManagers(); //KS - moved here so that we can get up-to-date touch found/lost events in IG

	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();
	mDynamicsContext->processPatches(mLLContext->getTaskPool(), continuation, nphase->getLostFoundPatchManagers(), nphase->getNbLostFoundPatchManagers(), nphase->getLostFoundPatchOutputCounts());

	// extracting information for the contact callbacks must happen before the solver writes the post-solve
	// velocities and positions into the solver bodies
	processNarrowPhaseTouchEvents(&mUpdateDynamics);

#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
	// PT: in this version we run postIslandGen directly here in parallel with "islandGen" tasks (rather than just after them).
	postIslandGen(&mSolver);
#endif
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
		PX_NOCOPY(InteractionNewTouchTask)

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
				si->managerNewTouch(0, mOutputs);
			}
			mNphaseCore->unlockReports();
		}
	};
}

void Sc::Scene::processNarrowPhaseTouchEvents(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processNarrowPhaseTouchEvents", mContextId);

	PxsContext* context = mLLContext;

	{
		PX_PROFILE_ZONE("Sim.preIslandGen", mContextId);

		// Update touch states from LL
		PxU32 newTouchCount, lostTouchCount;
		PxU32 ccdTouchCount = 0;
		{
			PX_PROFILE_ZONE("Sim.preIslandGen.managerTouchEvents", mContextId);
			context->getManagerTouchEventCount(&newTouchCount, &lostTouchCount, NULL);
			//PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
			//PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);

			mTouchFoundEvents.forceSize_Unsafe(0);
			mTouchFoundEvents.reserve(newTouchCount);
			mTouchFoundEvents.forceSize_Unsafe(newTouchCount);

			mTouchLostEvents.forceSize_Unsafe(0);
			mTouchLostEvents.reserve(lostTouchCount);
			mTouchLostEvents.forceSize_Unsafe(lostTouchCount);

			context->fillManagerTouchEvents(mTouchFoundEvents.begin(), newTouchCount,
											mTouchLostEvents.begin(), lostTouchCount,
											NULL, ccdTouchCount);

			mTouchFoundEvents.forceSize_Unsafe(newTouchCount);
			mTouchLostEvents.forceSize_Unsafe(lostTouchCount);
		}

		context->getSimStats().mNbNewTouches = newTouchCount;
		context->getSimStats().mNbLostTouches = lostTouchCount;
	}

	PxvNphaseImplementationContext*	ctx = context->getNphaseImplementationContext();

	PxsContactManagerOutputIterator outputs = ctx->getContactManagerOutputs();

	const PxU32 newTouchCount = mTouchFoundEvents.size();

	{
		Cm::FlushPool& flushPool = context->getTaskPool();

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
			si->managerNewTouch(0, outputs, useAdaptiveForce);
		}
	}*/
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::postIslandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.postIslandGen", mContextId);

	//
	// Trigger overlap processing (1) shall run in parallel with some parts of island
	// management (2) (connecting edges, running second island gen pass, object activation...)
	// For this to work without clashes, the work has to be split into pieces. Things to
	// keep in mind:
	// 
	// (1) can deactivate trigger pairs while (2) can activate trigger pairs (both might
	// happen for the same pair). The active interaction tracking arrays are not thread safe
	// (Sc::Scene::notifyInteractionDeactivated, ::notifyInteractionActivated) plus the
	// natural order is to process activation first (deactivation should be based on the
	// state after activation). Thus, (1) is split into a part (1a) that does the overlap checks
	// and a part (1b) that checks if trigger pairs can be deactivated. (1a) will run in parallel
	// with (2). (1b) will run after (2).
	// Leaves the question of what happens to the trigger pairs activated in (2)? Should those
	// not get overlap processing too? The rational for why this does not seem necessary is:
	// If a trigger interaction is activated, then it was inactive before. If inactive, the
	// overlap state can not have changed since the end of last sim step, unless:
	// - the user changed the position of one of the invovled actors or shapes
	// - the user changed the geometry of one of the involved shapes
	// - the pair is new
	// However, for all these cases, the trigger interaction is marked in a way that enforces
	// processing and the interaction gets activated too.
	//

	PxBaseTask* setEdgesConnectedContinuationTask = continuation;

	PxBaseTask* concludingTriggerTask = mNPhaseCore->prepareForTriggerInteractionProcessing(continuation);
	if (concludingTriggerTask)
	{
		setEdgesConnectedContinuationTask = concludingTriggerTask;
	}

	mSetEdgesConnectedTask.setContinuation(setEdgesConnectedContinuationTask);
	mSetEdgesConnectedTask.removeReference();

	// - Performs collision detection for trigger interactions
	if (concludingTriggerTask)
	{
		mNPhaseCore->processTriggerInteractions(*concludingTriggerTask);
		concludingTriggerTask->removeReference();
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::setEdgesConnected(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.preIslandGen.islandTouches", mContextId);

#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
	// PT: this is now running in parallel with "islandGen" tasks and the Pxg constraint partitioning code.
#endif
	{
		PX_PROFILE_ZONE("Sim.preIslandGen.setEdgesConnected", mContextId);

		const PxU32 newTouchCount = mTouchFoundEvents.size();
		for(PxU32 i = 0; i < newTouchCount; ++i)
		{
			const ShapeInteraction* si = getSI(mTouchFoundEvents[i]);

			// jcarius: defensive coding for OM-99507. If this assert hits, you maybe hit the same issue, please report!
			// ### DEFENSIVE
			if(si == NULL || si->getEdgeIndex() == IG_INVALID_EDGE)
			{
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Sc::Scene::setEdgesConnected: adding an invalid edge. Skipping.");
				PX_ALWAYS_ASSERT();
				continue;
			}

			if(!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex(), IG::Edge::eCONTACT_MANAGER);
		}
	}

#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
	mSimpleIslandManager->secondPassIslandGenPart1();
#else
	mSimpleIslandManager->secondPassIslandGen();

	wakeObjectsUp();
#endif
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::solver(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.rigidBodySolver", mContextId);

#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
	// PT: we run here the last part of Sc::Scene::setEdgesConnected()
	// PT: TODO: move to a non solver part?
	mSimpleIslandManager->secondPassIslandGenPart2();

	wakeObjectsUp();
#endif

	//Update forces per body in parallel. This can overlap with the other work in this phase.
	beforeSolver(continuation);

	PX_PROFILE_ZONE("Sim.postNarrowPhaseSecondPass", mContextId);

	//Narrowphase is completely finished so the streams can be swapped.
	mLLContext->swapStreams();

	//PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class ScBeforeSolverTask : public Cm::Task
	{
	public:
		static const PxU32 MaxBodiesPerTask = 256;
		PxsRigidBody*				mBodies[MaxBodiesPerTask];
		PxU32						mNumBodies;
		const PxReal				mDt;
		IG::SimpleIslandManager*	mIslandManager;
		PxsSimulationController*	mSimulationController;
		PxsExternalAccelerationProvider* mAccelerationProvider;
	public:

		ScBeforeSolverTask(PxReal dt, IG::SimpleIslandManager* islandManager, PxsSimulationController* simulationController, PxU64 contextID,
			PxsExternalAccelerationProvider* accelerationProvider) :
			Cm::Task				(contextID),
			mDt						(dt),
			mIslandManager			(islandManager),
			mSimulationController	(simulationController),
			mAccelerationProvider	(accelerationProvider)
		{
		}

		virtual void runInternal()
		{
			PX_PROFILE_ZONE("Sim.ScBeforeSolverTask", mContextID);

			const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();
			const PxU32 rigidBodyOffset = BodySim::getRigidBodyOffset();

			PxU32 maxNumBodies = islandSim.getNbNodes();

			PxsRigidBody* updatedBodySims[MaxBodiesPerTask];
			PxU32 updatedBodyNodeIndices[MaxBodiesPerTask];
			PxU32 nbUpdatedBodySims = 0;
		
			PxU32 nb = mNumBodies;
			PxsRigidBody** bodies = mBodies;
			while (nb--)
			{
				PxsRigidBody* body = *bodies++;
				BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(body) - rigidBodyOffset);
				bodySim->updateForces(mDt, updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims, NULL, mAccelerationProvider, maxNumBodies);
			}

			if(nbUpdatedBodySims)
				mSimulationController->updateBodies(updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims, mAccelerationProvider);
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

	public:

		ScArticBeforeSolverTask(ArticulationSim* const* articSims, PxU32 nbArtics, PxReal dt, PxU64 contextID) :
			Cm::Task(contextID),
			mArticSims(articSims),
			mNumArticulations(nbArtics),
			mDt(dt)
		{
		}

		virtual void runInternal()
		{
			PX_PROFILE_ZONE("Sim.ScArticBeforeSolverTask", mContextID);

			for(PxU32 a = 0; a < mNumArticulations; ++a)
			{
				ArticulationSim* PX_RESTRICT articSim = mArticSims[a];
				//articSim->checkResize();
				articSim->updateForces(mDt);
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
				ArticulationSim* articSim = getArticulationSim(islandSim, mArticIndices[a]);

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
	mNumDeactivatingNodes[IG::Node::eDEFORMABLE_SURFACE_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::eDEFORMABLE_VOLUME_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::ePARTICLESYSTEM_TYPE] = 0;
//#endif

	const PxU32 MaxBodiesPerTask = ScBeforeSolverTask::MaxBodiesPerTask;

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mSimulationController->reserve(nbActiveBodies);

	{
		PxBitMap::Iterator iter(mVelocityModifyMap);

		// PT: TASK-CREATION TAG
		for (PxU32 i = iter.getNext(); i != PxBitMap::Iterator::DONE; /*i = iter.getNext()*/)
		{
			ScBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScBeforeSolverTask)), ScBeforeSolverTask(mDt, mSimpleIslandManager, mSimulationController, mContextId, 
				&mDynamicsContext->getExternalRigidAccelerations()));
			PxU32 count = 0;
			for(; count < MaxBodiesPerTask && i != PxBitMap::Iterator::DONE; i = iter.getNext())
			{
				// we need to check if the body is active because putToSleep does raise the
				// velocityModFlags making it appear as if the body needs to update forces. It can also
				// happen that we have inactive bodies with modified velocities if the user does not call
				// autowake when modifying velocities.
				PxNodeIndex index(i);
				bool retainAccelerations = false;
				if(islandSim.getActiveNodeIndex(index) != PX_INVALID_NODE)
				{
					PxsRigidBody* body = getRigidBodyFromIG(islandSim, index);
					PX_ASSERT(body);
					PX_ASSERT(islandSim.getNode(index).mType == IG::Node::eRIGID_BODY_TYPE);

					task->mBodies[count++] = body;

					retainAccelerations = (body->mCore->mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS);
				}

				if(!retainAccelerations)
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
			mDt, mContextId));

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
	
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateBodies(PxBaseTask* continuation)
{
	PX_ASSERT(isUsingGpuDynamicsOrBp());	// PT: this is not called anymore in the CPU pipeline

	// AD: need to raise dirty flags serially because the PxgBodySimManager::updateArticulation() is not thread-safe.
	const PxU32 nbDirtyArticulations = mDirtyArticulationSims.size();
	ArticulationSim* const* artiSim = mDirtyArticulationSims.getEntries();
	for (PxU32 a = 0; a < nbDirtyArticulations; ++a)
	{
		if (artiSim[a]->getLowLevelArticulation()->mGPUDirtyFlags & (Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL))
		{
			mSimulationController->updateArticulationExtAccel(artiSim[a]->getLowLevelArticulation(), artiSim[a]->getIslandNodeIndex());
		}
	}

	//dma bodies and articulation data to gpu
	mSimulationController->updateBodies(continuation);
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::updateDynamics(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.dynamics", mContextId);

	//Allow processLostContactsTask to run until after 2nd pass of solver completes (update bodies, run sleeping logic etc.)
	mProcessLostContactsTask3.setContinuation(&mPostSolver);
	mProcessLostContactsTask2.setContinuation(&mProcessLostContactsTask3);
	mProcessLostContactsTask.setContinuation(&mProcessLostContactsTask2);

	////dma bodies and shapes data to gpu
	//mSimulationController->updateBodiesAndShapes();

	mLLContext->getNpMemBlockPool().acquireConstraintMemory();

	const PxU32 maxPatchCount = mLLContext->getMaxPatchCount();

	mAABBManager->reallocateChangedAABBMgActorHandleMap(getElementIDPool().getMaxID());

	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);

	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();

	// PT: mDynamicsContext->update() is a blocking call but we want to spawn tasks from the constraint
	// partitioning code it calls (for the GPU pipeline only). So we will need to split this function
	// into two. Conceptually:
	// "updateDynamicsPart1" => contains code above this line and calls "mDynamicsContext->updatePart1()"
	// "updateDynamicsPart2" => calls "mDynamicsContext->updatePart2()" and contains code after the current
	// update() call below. That way the code inside update itself can be MT and have a proper continuation
	// task ("updateDynamicsPart2"). The current code executed after the constraint partitioning code within
	// update() has to be moved to "updateDynamicsPart2" as well, as it actually uses the partitioning data
	// (which would not be available while the partitioning tasks are running).
	// Note that this is not needed for the CPU pipeline, so there we just run updateDynamicsPostPartitioning
	// immediately in the same thread to avoid the task overhead.

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	if(isUsingGpuDynamicsOrBp())
	{
		if (getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API)
		{
			mDynamicsContext->setActiveBreakableConstraintCount(mActiveBreakableConstraints.size());
		}

		mDynamicsContext->update(	flushPool, &mUpdateSimulationController, &mUpdateDynamicsPostPartitioning, &mProcessLostContactsTask,
									nphase,	maxPatchCount, mMaxNbArticulationLinks, mDt, mGravity, mAABBManager->getChangedAABBMgActorHandleMap());
	}
	else
	{
		mDynamicsContext->update(	flushPool, &mPostSolver, &mUpdateDynamicsPostPartitioning, &mProcessLostContactsTask,
									nphase,	maxPatchCount, mMaxNbArticulationLinks, mDt, mGravity, mAABBManager->getChangedAABBMgActorHandleMap());

		updateDynamicsPostPartitioning(&mPostSolver);
	}
}

void Sc::Scene::updateDynamicsPostPartitioning(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Scene::updateDynamicsPostPartitioning", mContextId);

	{
		const PxU32 maxPatchCount = mLLContext->getMaxPatchCount();
		PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();

		mDynamicsContext->updatePostPartitioning(&mProcessLostContactsTask,
			nphase,	maxPatchCount, mMaxNbArticulationLinks, mDt, mGravity, mAABBManager->getChangedAABBMgActorHandleMap());
	}

	mSimpleIslandManager->clearDestroyedPartitionEdges();

	mProcessLostContactsTask3.removeReference();
	mProcessLostContactsTask2.removeReference();
	mProcessLostContactsTask.removeReference();
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE void findInteractions(const NPhaseCore& core, PxU32 count, AABBOverlap* PX_RESTRICT overlaps)
{
	while(count--)
	{
		const ElementSim* volume0 = reinterpret_cast<const ElementSim*>(overlaps->mUserData0);
		const ElementSim* volume1 = reinterpret_cast<const ElementSim*>(overlaps->mUserData1);

		// PT: this looks useless on lost pairs but it is used in processLostContacts2 and processLostContacts3
		// PT: it seems very questionable to store this within the BP structures at this point. If anything
		// we should have stored that there when the overlap was created, and we wouldn't have to look for the
		// interaction here.
		overlaps->mPairUserData = core.findInteraction(volume0, volume1);
		overlaps++;
	}
}

void Sc::Scene::processLostContacts(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processLostContacts", mContextId);

	// PT: don't bother starting the tasks if we don't need to
	if(mTouchLostEvents.size())
	{
		mProcessNarrowPhaseLostTouchTasks.setContinuation(continuation);
		mProcessNarrowPhaseLostTouchTasks.removeReference();

		//mLostTouchReportsTask.setContinuation(&mProcessLostContactsTask3);
		mProcessNPLostTouchEvents.setContinuation(continuation);
		mProcessNPLostTouchEvents.removeReference();
	}

	{
		PX_PROFILE_ZONE("Sim.findInteractionsPtrs", mContextId);

		PxU32 destroyedOverlapCount;
		AABBOverlap* PX_RESTRICT destroyedOverlaps = mAABBManager->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);
		if(destroyedOverlapCount)
		{
			const PxU32 numWorkerTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();

			if(numWorkerTasks<2 || destroyedOverlapCount<32)
			{
				::findInteractions(*mNPhaseCore, destroyedOverlapCount, destroyedOverlaps);
			}
			else
			{
				class FindInteractionTask : public Cm::Task
				{
					const NPhaseCore&	mNPhaseCore;
					AABBOverlap*		mOverlaps;
					const PxU32			mCount;
				public:
					FindInteractionTask(PxU64 contextID, const NPhaseCore& core, AABBOverlap* overlaps, PxU32 count) :
						Cm::Task	(contextID),
						mNPhaseCore	(core),
						mOverlaps	(overlaps),
						mCount		(count)
					{
					}

					virtual void runInternal()
					{
						::findInteractions(mNPhaseCore, mCount, mOverlaps);
					}

					virtual const char* getName() const { return "FindInteractionTask"; }
				};

				Cm::FlushPool& flushPool = mLLContext->getTaskPool();

				// PT: TASK-CREATION TAG
				PxU32 maxPerTask = destroyedOverlapCount/(numWorkerTasks*2);
				maxPerTask = PxMax(maxPerTask, 32u);

				while(destroyedOverlapCount)
				{
					const PxU32 localCount = PxMin(destroyedOverlapCount, maxPerTask);
					FindInteractionTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(FindInteractionTask)), FindInteractionTask)(mContextId, *mNPhaseCore, destroyedOverlaps, localCount);
					startTask(task, continuation);
					destroyedOverlaps += localCount;
					destroyedOverlapCount -= localCount;
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processNarrowPhaseLostTouchEventsIslands(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sc::Scene.islandLostTouches", mContextId);

	const PxU32 count = mTouchLostEvents.size();
	PX_ASSERT(count);	// PT: otherwise we should have skipped the task entirely
	for(PxU32 i=0; i <count; ++i)
	{
		ShapeInteraction* si = getSI(mTouchLostEvents[i]);

		// AD: defensive coding for OMPE-12798/PX-5240. If this assert hits, you maybe hit the same issue, please report!
		// ### DEFENSIVE
		if (si == NULL || si->getEdgeIndex() == IG_INVALID_EDGE)
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Sc::Scene::setEdgeDisconnected: removing an invalid edge. Skipping.");
			PX_ALWAYS_ASSERT();
			continue;
		}

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
	PX_ASSERT(count);	// PT: otherwise we should have skipped the task entirely
	for(PxU32 i=0; i<count; ++i)
	{
		ShapeInteraction* si = getSI(mTouchLostEvents[i]);

		// AD: defensive coding for OMPE-12798/PX-5240. If this assert hits, you maybe hit the same issue, please report!
		// ### DEFENSIVE
		if (si == NULL || si->getEdgeIndex() == IG_INVALID_EDGE)
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Sc::Scene::processNarrowPhaseLostTouchEvents: processing an invalid edge. Skipping.");
			PX_ALWAYS_ASSERT();
			continue;
		}

		PX_ASSERT(si);
		if(si->managerLostTouch(0, outputs) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
			addToLostTouchList(si->getActor0(), si->getActor1());
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::processLostContacts2(PxBaseTask* continuation)
{
	PxU32 destroyedOverlapCount;
	AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);

	mDestroyManagersTask.setContinuation(continuation);

	// PT: don't bother starting the tasks if we don't need to
	if(destroyedOverlapCount)
	{
		mLostTouchReportsTask.setContinuation(&mDestroyManagersTask);
		mLostTouchReportsTask.removeReference();

		mUnregisterInteractionsTask.setContinuation(continuation);
		mUnregisterInteractionsTask.removeReference();
	}

	{
		PX_PROFILE_ZONE("Sim.clearIslandData", mContextId);
		{
			while(destroyedOverlapCount--)
			{
				ElementSimInteraction* pair = reinterpret_cast<ElementSimInteraction*>(p->mPairUserData);
				if(pair)
				{
					if(pair->getType() == InteractionType::eOVERLAP)
					{
						ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);
						si->clearIslandGenData(*mSimpleIslandManager);
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
		const AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);
		PX_ASSERT(destroyedOverlapCount);	// PT: otherwise we should have skipped the task entirely

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
	const AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);
	PX_ASSERT(destroyedOverlapCount);	// PT: otherwise we should have skipped the task entirely

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
	const AABBOverlap* PX_RESTRICT p = mAABBManager->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);

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

		AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;

		// PT: for regular shapes
		{
			const AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(ElementType::eSHAPE, destroyedOverlapCount);
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
			const AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(ElementType::eTRIGGER, destroyedOverlapCount);
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

		for(PxU32 t = 0; t < IG::Edge::eEDGE_TYPE_COUNT; ++t)
		{
			const IG::Edge::EdgeType edgeType = IG::Edge::EdgeType(t);
			if(edgeType == IG::Edge::eCONSTRAINT)
				continue;

			const PxU32 nbDeactivatingEdges = islandSim.getNbDeactivatingEdges(edgeType);
			const IG::EdgeIndex* deactivatingEdgeIds = islandSim.getDeactivatingEdges(edgeType);

			for(PxU32 i = 0; i < nbDeactivatingEdges; ++i)
			{
				Interaction* interaction = mSimpleIslandManager->getInteractionFromEdgeIndex(deactivatingEdgeIds[i]);

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
	
	PX_ASSERT(isUsingGpuDynamicsOrBp());	// PT: this is not called anymore in the CPU pipeline

	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	BoundsArray& boundArray = getBoundsArray();

	PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();

	mSimulationController->gpuDmabackData(cache, boundArray, changedAABBMgrActorHandles, mPublicFlags & PxSceneFlag::eENABLE_DIRECT_GPU_API);

	//for pxgdynamicscontext: copy solver body data to body core 
	{
		PX_PROFILE_ZONE("Sim.updateBodyCore", mContextId);
		mDynamicsContext->updateBodyCore(continuation);
	}
	//mSimulationController->update(cache, boundArray, changedAABBMgrActorHandles);
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

	mDynamicsContext->getExternalRigidAccelerations().clearAll();

	//afterIntegration(continuation);
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
	BoundsArray& boundArray = getBoundsArray();

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
				PxsRigidBody* rigid = getRigidBodyFromIG(islandSim, deactivatingIndices[i]);
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
		ArticulationSim* artic = getArticulationSim(islandSim, deactivatingArticIndices[i]);

		artic->putToSleep();
	}

	//PxU32 previousNumClothToDeactivate = mNumDeactivatingNodes[IG::Node::eDEFORMABLE_SURFACE_TYPE];
	//const PxU32 numClothToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eDEFORMABLE_SURFACE_TYPE);
	//const IG::NodeIndex*const deactivatingClothIndices = islandSim.getNodesToDeactivate(IG::Node::eDEFORMABLE_SURFACE_TYPE);

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
	//	Dy::DeformableVolume* deformableVolume = islandSim.getLLDeformableVolume(deactivatingSoftBodiesIndices[i]);
	//	printf("after Integration: Deactivating deformable volume %i\n", softbody->getGpuRemapId());
	//	//mSimulationController->deactivateDeformableVolume(deformableVolume);
	//	deformableVolume->getDeformableVolumeSim()->setActive(false, 0);
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
	{
		PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onAdvance", mContextId);
		mSimulationEventCallback->onAdvance(mClientPosePreviewBodies.begin(), mClientPosePreviewBuffer.begin(), bodyCount);
	}
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
	
	if (getFlags() & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING)
		collectSolverResidual();

	PX_PROFILE_STOP_CROSSTHREAD("Basic.rigidBodySolver", mContextId);

	mTaskPool.clear();

	mReportShapePairTimeStamp++;	// important to do this before fetchResults() is called to make sure that delayed deleted actors/shapes get
									// separate pair entries in contact reports

	// AD: WIP, will be gone once we removed the warm-start with sim step.
	if (mPublicFlags & PxSceneFlag::eENABLE_DIRECT_GPU_API)
		setDirectGPUAPIInitialized();

	// VR: do this at finalizationPhase when all contact and
	// friction impulses and CCD contacts are already computed
	visualizeContacts();
}

void Sc::Scene::collectSolverResidual()
{
	PX_PROFILE_ZONE("Sim.collectSolverResidual", mContextId);

	PxU32 counter = mDynamicsContext->getContactErrorCounter();
	PxReal rmsGlobalResidual = mDynamicsContext->getContactError();
	PxReal maxGlobalResidual = mDynamicsContext->getMaxContactError();


	PxU32 counterPosIter = mDynamicsContext->getContactErrorCounterPosIter();
	PxReal rmsGlobalResidualPosIter = mDynamicsContext->getContactErrorPosIter();
	PxReal maxGlobalResidualPosIter = mDynamicsContext->getMaxContactErrorPosIter();

	PX_ASSERT(PxIsFinite(rmsGlobalResidual) && PxIsFinite(maxGlobalResidual));

	const PxPinnedArray<Dy::ConstraintWriteback>& pool = mDynamicsContext->getConstraintWriteBackPool();
	const PxPinnedArray<PxReal>& poolPosIterResidualGpu = mDynamicsContext->getConstraintPositionIterResidualPoolGpu();

	ConstraintCore* const* constraints = mConstraints.getEntries();
	PxU32 count = mConstraints.size();
	while (count--)
	{
		ConstraintCore* core = constraints[count];
		ConstraintSim* sim = core->getSim();
		const Dy::ConstraintWriteback& solverOutput = pool[sim->getLowLevelConstraint().index];
		PxReal positionIterationResidual = solverOutput.getPositionIterationResidual();
		if (poolPosIterResidualGpu.size() > 0)
			positionIterationResidual += poolPosIterResidualGpu[sim->getLowLevelConstraint().index];

		PxConstraintResidual residual;
		residual.positionIterationResidual = positionIterationResidual;
		residual.velocityIterationResidual = solverOutput.residual;
		core->setSolverResidual(residual);


		rmsGlobalResidual += solverOutput.residual * solverOutput.residual;
		maxGlobalResidual = PxMax(PxAbs(solverOutput.residual), maxGlobalResidual);

		PX_ASSERT(PxIsFinite(rmsGlobalResidual) && PxIsFinite(maxGlobalResidual));


		rmsGlobalResidualPosIter += positionIterationResidual * positionIterationResidual;
		maxGlobalResidualPosIter = PxMax(PxAbs(positionIterationResidual), maxGlobalResidualPosIter);

		PX_ASSERT(PxIsFinite(rmsGlobalResidualPosIter) && PxIsFinite(maxGlobalResidualPosIter));
	}
	counter += mConstraints.size();
	counterPosIter += mConstraints.size();

	ArticulationCore* const* articulations = mArticulations.getEntries();
	count = mArticulations.size();
	while (count--)
	{
		ArticulationCore* core = articulations[count];
		ArticulationSim* sim = core->getSim();
		const Dy::ErrorAccumulator& internalErrVelIter = sim->getLowLevelArticulation()->mInternalErrorAccumulatorVelIter;
		rmsGlobalResidual += internalErrVelIter.mErrorSumOfSquares;
		counter += internalErrVelIter.mCounter;
		maxGlobalResidual = PxMax(maxGlobalResidual, internalErrVelIter.mMaxError);

		const Dy::ErrorAccumulator& contactErrVelIter = sim->getLowLevelArticulation()->mContactErrorAccumulatorVelIter;
		rmsGlobalResidual += contactErrVelIter.mErrorSumOfSquares;
		counter += contactErrVelIter.mCounter;
		maxGlobalResidual = PxMax(maxGlobalResidual, contactErrVelIter.mMaxError);

		PX_ASSERT(PxIsFinite(rmsGlobalResidual) && PxIsFinite(maxGlobalResidual));


		const Dy::ErrorAccumulator& internalErrPosIter = sim->getLowLevelArticulation()->mInternalErrorAccumulatorPosIter;
		rmsGlobalResidualPosIter += internalErrPosIter.mErrorSumOfSquares;
		counterPosIter += internalErrPosIter.mCounter;
		maxGlobalResidualPosIter = PxMax(maxGlobalResidualPosIter, internalErrPosIter.mMaxError);

		const Dy::ErrorAccumulator& contactErrPosIter = sim->getLowLevelArticulation()->mContactErrorAccumulatorPosIter;
		rmsGlobalResidualPosIter += contactErrPosIter.mErrorSumOfSquares;
		counterPosIter += contactErrPosIter.mCounter;
		maxGlobalResidualPosIter = PxMax(maxGlobalResidualPosIter, contactErrPosIter.mMaxError);

		PX_ASSERT(PxIsFinite(rmsGlobalResidualPosIter) && PxIsFinite(maxGlobalResidualPosIter));
	}

	mResidual.velocityIterationResidual.rmsResidual = PxSqrt(1.0f / PxMax(1u, counter) * rmsGlobalResidual);
	mResidual.velocityIterationResidual.maxResidual = maxGlobalResidual;

	mResidual.positionIterationResidual.rmsResidual = PxSqrt(1.0f / PxMax(1u, counterPosIter) * rmsGlobalResidualPosIter);
	mResidual.positionIterationResidual.maxResidual = maxGlobalResidualPosIter;
}

///////////////////////////////////////////////////////////////////////////////
