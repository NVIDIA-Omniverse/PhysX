// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpScene.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "NpAggregate.h"
#include "ScScene.h"
#if PX_SUPPORT_GPU_PHYSX
	#include "NpPBDParticleSystem.h"
	#include "NpParticleBuffer.h"
	#include "NpDeformableSurface.h"
	#include "NpDeformableVolume.h"
	#include "cudamanager/PxCudaContextManager.h"
	#include "cudamanager/PxCudaContext.h"
	#include "geometry/PxTetrahedronMesh.h"
	#include "geometry/PxTriangleMeshGeometry.h"
#endif
#include "ScArticulationSim.h"
#include "ScArticulationTendonSim.h"
#include "CmCollection.h"
#include "PxsSimulationController.h"
#include "common/PxProfileZone.h"
#include "BpBroadPhase.h"
#include "BpAABBManagerBase.h"
#if PX_SUPPORT_OMNI_PVD
	#include "omnipvd/NpOmniPvdSetData.h"
	#include "omnipvd/OmniPvdPxSampler.h"
#endif

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

using namespace physx;

bool NpScene::checkResultsInternal(bool block)
{
	PX_PROFILE_ZONE("Basic.checkResults", getContextId());

	/*if(0 && block)
	{
		PxCpuDispatcher* d = mTaskManager->getCpuDispatcher();
		while(!mPhysicsDone.wait(0))
		{
			PxBaseTask* nextTask = d->fetchNextTask();
			if(nextTask)
			{
				PX_PROFILE_ZONE(nextTask->getName(), getContextId());
				nextTask->run();
				nextTask->release();
			}
		}
	}*/

	return mPhysicsDone.wait(block ? PxSync::waitForever : 0);
}

bool NpScene::checkResults(bool block)
{
	return checkResultsInternal(block);
}

void NpScene::fetchResultsParticleSystem()
{
	// silent if scene state is corrupted.
	NP_CHECK_CORRUPTION_AND_RETURN

	if (!mScene.isUsingGpuDynamics()) // particles are only supported with GPU dynamics.
		return;

	mScene.getSimulationController()->syncParticleData();

#if PX_SUPPORT_GPU_PHYSX
	PxCUresult res = mCudaContextManager->getCudaContext()->getLastError();
	if (res)
	{
		if (mCudaContextManager->getCudaContext()->isInAbortMode())
		{
			PxGetFoundation().error(PxErrorCode::eABORT, PX_FL, "PhysX failed to allocate GPU memory - aborting simulation.");
			mCorruptedState = true;
		}
	}
#endif
}

// The order of the following operations is important!
// 1. Mark the simulation as not running internally to allow reading data which should not be read otherwise
// 2. Fire callbacks with latest state.

void NpScene::fetchResultsPreContactCallbacks()
{
#if PX_SUPPORT_PVD
	mScenePvdClient.updateContacts();
#endif

	mScene.endSimulation();

	setAPIReadToAllowed();

	{
		PX_PROFILE_ZONE("Sim.fireCallbacksPreSync", getContextId());
		{
			PX_PROFILE_ZONE("Sim.fireOutOfBoundsCallbacks", getContextId());
			if(mScene.fireOutOfBoundsCallbacks())
				outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "At least one object is out of the broadphase bounds. To manage those objects, define a PxBroadPhaseCallback for each used client.");
		}
		mScene.fireBrokenConstraintCallbacks();
		mScene.fireTriggerCallbacks();
	}
}

void NpScene::fetchResultsPostContactCallbacks()
{
	mScene.postCallbacksPreSync();

	// Body accelerations are now computed as part of the simulation task graph
	// (see computeBodyAccelerationsAsync). For GPU dynamics, non-kinematic
	// GPU accelerations are lazy-copied from the GPU buffer on first getter access.

	syncSQ();

#if PX_SUPPORT_PVD
	mScenePvdClient.updateSceneQueries();

	mNpSQ.getSingleSqCollector().clear();
#endif

	// fire sleep and wake-up events
	{
		PX_PROFILE_ZONE("Sim.fireCallbacksPostSync", getContextId());
		mScene.fireCallbacksPostSync();
	}

	mScene.postReportsCleanup();

	// build the list of active actors
	{
		PX_PROFILE_ZONE("Sim.buildActiveActors", getContextId());

		const bool buildActiveActors = (mScene.getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS) || OMNI_PVD_ACTIVE;

		if (buildActiveActors && mBuildFrozenActors)
			mScene.buildActiveAndFrozenActors();
		else if (buildActiveActors)
			mScene.buildActiveActors();
	}

	mRenderBuffer.append(mScene.getRenderBuffer());

	PX_ASSERT(getSimulationStage() != Sc::SimulationStage::eCOMPLETE);
	if (mControllingSimulation)
		mTaskManager->stopSimulation();

	setSimulationStage(Sc::SimulationStage::eCOMPLETE);
#if PX_SUPPORT_OMNI_PVD
	mSceneOvdClient.setContactsValid(true);
#endif
	setAPIWriteToAllowed();

	mPhysicsDone.reset();				// allow Physics to run again
	mCollisionDone.reset();
}

#if PX_SUPPORT_OMNI_PVD
void NpScene::fetchResultsOmniPvd()
{
		OmniPvdPxSampler* omniPvdSampler = NpPhysics::getInstance().mOmniPvdSampler;
		if (omniPvdSampler && omniPvdSampler->isSampling()) omniPvdSampler->streamSceneContacts(*this);
		if (omniPvdSampler && NpOmniPvdCaptureScene(*this))
		{
			if (!streamGPUStateToOmniPvd(false)) return;
			const bool directGpu = getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
			const bool selectiveDirectGpu = directGpu && NpOmniPvdCaptureIsSelective();
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

			//send all xforms updated by the sim:
			PxU32 nActiveActors;
			PxActor ** activeActors = mScene.getActiveActors(nActiveActors);
			while (nActiveActors--)
			{
				PxActor * a = *activeActors++;
				const bool capturePayload = NpOmniPvdCaptureActor(*a) &&
					!(selectiveDirectGpu && (a->getType() == PxActorType::eRIGID_DYNAMIC || a->getType() == PxActorType::eARTICULATION_LINK));
				if (capturePayload)
				{
					if ((a->getType() == PxActorType::eRIGID_STATIC) || (a->getType() == PxActorType::eRIGID_DYNAMIC))
					{
						PxRigidActor* ra = static_cast<PxRigidActor*>(a);
						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, *ra, ra->getGlobalPose());

						if (a->getType() == PxActorType::eRIGID_DYNAMIC)
						{
							PxRigidDynamic* rdyn = static_cast<PxRigidDynamic*>(a);
							PxRigidBody& rb = *static_cast<PxRigidBody*>(a);

							OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, rb, rdyn->getLinearVelocity())

							OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, rb, rdyn->getAngularVelocity())

							OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, rb, rdyn->getRigidBodyFlags())

							OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, wakeCounter, *rdyn, rdyn->getWakeCounter());
						}

					}
					else if (a->getType() == PxActorType::eARTICULATION_LINK)
					{
						PxArticulationLink* pxArticulationLink = static_cast<PxArticulationLink*>(a);
						PxArticulationJointReducedCoordinate* pxArticulationJoint = pxArticulationLink->getInboundJoint();
						if (pxArticulationJoint)
						{
							PxArticulationJointReducedCoordinate& jcord = *pxArticulationJoint;
							PxReal vals[PxArticulationAxis::eCOUNT];
							if (NpOmniPvdSampling())
							{
								for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax) vals[ax] = jcord.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
								OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, jcord, vals, PxArticulationAxis::eCOUNT);
							}
							if (NpOmniPvdSampling())
							{
								for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax) vals[ax] = jcord.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
								OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, jcord, vals, PxArticulationAxis::eCOUNT);
							}
						}

						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, static_cast<PxRigidActor&>(*a), pxArticulationLink->getGlobalPose());

						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, static_cast<PxRigidBody&>(*a), pxArticulationLink->getLinearVelocity())

						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, static_cast<PxRigidBody&>(*a), pxArticulationLink->getAngularVelocity())

						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, static_cast<PxRigidBody&>(*a), pxArticulationLink->getRigidBodyFlags())
					}

					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, *a, a->getWorldBounds())
					if (!directGpu && (getFlags() & PxSceneFlag::eENABLE_BODY_ACCELERATIONS))
						if (const PxRigidBody* body = a->is<PxRigidBody>())
						{
							OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearAcceleration, *body, body->getLinearAcceleration());
							OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularAcceleration, *body, body->getAngularAcceleration());
						}

				}

				// update active actors' joints
				const PxRigidActor* ra = a->is<PxRigidActor>();
				if (ra)
				{
					static const PxU32 MAX_CONSTRAINTS = 32;
					PxConstraint* constraints[MAX_CONSTRAINTS];
					PxU32 index = 0;
					while (true)
					{
						PxU32 count = ra->getConstraints(constraints, MAX_CONSTRAINTS, index);
						for (PxU32 i = 0; i < count; ++i)
						{
							const NpConstraint& c = static_cast<const NpConstraint&>(*constraints[i]);
							if (!omniPvdSampler->isObjectRecorded(c) || !c.getCore().getPxConnector()) continue;
							PxRigidActor *ra0, *ra1; c.getActors(ra0, ra1);
							bool ra0static = !ra0 || !!ra0->is<PxRigidStatic>(), ra1static = !ra1 || !!ra1->is<PxRigidStatic>();
							// this check is to not update a joint twice
							if ((ra == ra0 && (ra1static || ra0 > ra1)) || (ra == ra1 && (ra0static || ra1 > ra0)))
								omniPvdSampler->updateConstraintOmniPvdProperties(c);
						}
						if (count == MAX_CONSTRAINTS)
						{
							index += MAX_CONSTRAINTS;
							continue;
						}
						break;
					}
				}
			}

			PxArticulationReducedCoordinate*const* articulations = mArticulations.getEntries();
			const PxU32 nbArticulations = mArticulations.size();
			for( PxU32 i = 0 ; i < nbArticulations ;i++)
			{
				PxArticulationReducedCoordinate* articulation = (articulations[i]);
				if (!NpOmniPvdCaptureArticulation(*articulation) || selectiveDirectGpu) continue;
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, wakeCounter, *articulation, articulation->getWakeCounter());
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, worldBounds, *articulation, articulation->getWorldBounds());
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, *articulation, articulation->isSleeping());
				if (!directGpu) omniPvdSampler->streamArticulationReactions(pvdWriter, pvdRegData, *articulation);
			}
#if PX_SUPPORT_GPU_PHYSX
			omniPvdSampler->beginSceneGpuReadback(*this);
			const PxPBDParticleSystem* const* particleSystems = mPBDParticleSystems.getEntries();
			for (PxU32 i = 0; i < mPBDParticleSystems.size(); ++i)
				if (!omniPvdSampler->streamParticleSystemState(pvdWriter, pvdRegData, *particleSystems[i])) break;
			PxDeformableVolume* const* volumes = mDeformableVolumes.getEntries();
			for (PxU32 i = 0; i < mDeformableVolumes.size(); ++i)
				if (!omniPvdSampler->streamDeformableState(pvdWriter, pvdRegData, *volumes[i])) break;
			PxDeformableSurface* const* surfaces = mDeformableSurfaces.getEntries();
			for (PxU32 i = 0; i < mDeformableSurfaces.size(); ++i)
				if (!omniPvdSampler->streamDeformableState(pvdWriter, pvdRegData, *surfaces[i])) break;
#endif

			NpOmniPvdSceneClient& ovdClient = getSceneOvdClientInternal();
			ovdClient.resetForces();
			OMNI_PVD_WRITE_SCOPE_END
		}
		if (omniPvdSampler && omniPvdSampler->isSampling())
		{
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			PX_UNUSED(pvdRegData);
			getSceneOvdClientInternal().incrementFrame(*pvdWriter, true);
			OMNI_PVD_WRITE_SCOPE_END
		}
}
#endif

bool NpScene::fetchResults(bool block, PxU32* errorState)
{
	NP_CHECK_CORRUPTION_AND_RETURN_VAL(true)

	if(getSimulationStage() != Sc::SimulationStage::eADVANCE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchResults: fetchResults() called illegally! It must be called after advance() or simulate()");

	if(!checkResultsInternal(block)) // this should wait on the mPhysicsDone event, which is set in the SceneCompletion task
		return false;


#if PX_SUPPORT_GPU_PHYSX
	if (!checkSceneStateAndCudaErrors())
		return true;
#endif

	PX_SIMD_GUARD

	{
		// take write check *after* simulation has finished, otherwise
		// we will block simulation callbacks from using the API
		// disallow re-entry to detect callbacks making write calls
		NP_WRITE_CHECK_NOREENTRY(this);

		// we use cross thread profile here, to show the event in cross thread view
		// PT: TODO: why do we want to show it in the cross thread view?
		PX_PROFILE_START_CROSSTHREAD("Basic.fetchResults", getContextId());
		PX_PROFILE_ZONE("Sim.fetchResults", getContextId());

		fetchResultsPreContactCallbacks();

		{
			// PT: TODO: why a cross-thread event here?
			PX_PROFILE_START_CROSSTHREAD("Basic.processCallbacks", getContextId());
			mScene.fireQueuedContactCallbacks();
			PX_PROFILE_STOP_CROSSTHREAD("Basic.processCallbacks", getContextId());
		}

		fetchResultsPostContactCallbacks();

		PX_PROFILE_STOP_CROSSTHREAD("Basic.fetchResults", getContextId());
		PX_PROFILE_STOP_CROSSTHREAD("Basic.simulate", getContextId());

		if(errorState)
			*errorState = 0;

#if PX_SUPPORT_OMNI_PVD
		fetchResultsOmniPvd();
#endif
	}

#if PX_SUPPORT_PVD
	mScenePvdClient.frameEnd();
#endif
	return true;
}

bool NpScene::fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block)
{
	NP_CHECK_CORRUPTION_AND_RETURN_VAL(true)

	if (getSimulationStage() != Sc::SimulationStage::eADVANCE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchResultsStart: fetchResultsStart() called illegally! It must be called after advance() or simulate()");

	if (!checkResultsInternal(block))
		return false;

#if PX_SUPPORT_GPU_PHYSX
	if (!checkSceneStateAndCudaErrors())
		return true;
#endif

	PX_SIMD_GUARD
	NP_WRITE_CHECK(this);

	// we use cross thread profile here, to show the event in cross thread view
	PX_PROFILE_START_CROSSTHREAD("Basic.fetchResults", getContextId());
	PX_PROFILE_ZONE("Sim.fetchResultsStart", getContextId());

	fetchResultsPreContactCallbacks();

	const PxArray<PxContactPairHeader>& pairs = mScene.getQueuedContactPairHeaders();
	nbContactPairs = pairs.size();
	contactPairs = pairs.begin();

	mBetweenFetchResults = true;
	return true;
}

namespace
{
	class NpContactCallbackTask : public physx::PxLightCpuTask
	{
		NpScene*					mScene;
		const PxContactPairHeader*	mContactPairHeaders;
		const PxU32					mNbContactPairHeaders;
	public:

		PX_FORCE_INLINE	NpContactCallbackTask(NpScene* scene, const PxContactPairHeader* contactPairHeaders, PxU64 contextID, PxU32 nbContactPairHeaders) :
			mScene					(scene),
			mContactPairHeaders		(contactPairHeaders),
			mNbContactPairHeaders	(nbContactPairHeaders)
		{
			setContextId(contextID);
		}

		virtual void run()	PX_OVERRIDE PX_FINAL
		{
			PxSimulationEventCallback* callback = mScene->getSimulationEventCallback();
			if (!callback)
				return;

			mScene->lockRead();
			{
				PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onContact", getContextId());
				PxU32 nb = mNbContactPairHeaders;
				for(PxU32 i=0; i<nb; i++)
				{
					const PxContactPairHeader& pairHeader = mContactPairHeaders[i];
					callback->onContact(pairHeader, pairHeader.pairs, pairHeader.nbPairs);
				}
			}
			mScene->unlockRead();
		}

		virtual const char* getName() const	PX_OVERRIDE PX_FINAL
		{
			return "NpContactCallbackTask";
		}
	};
}

void NpScene::processCallbacks(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.processCallbacks", getContextId());
	PX_PROFILE_ZONE("Sim.processCallbacks", getContextId());

	//ML: because Apex destruction callback isn't thread safe so that we make this run single thread first
	const PxArray<PxContactPairHeader>& pairs = mScene.getQueuedContactPairHeaders();
	const PxU32 nbPairs = pairs.size();
	const PxContactPairHeader* contactPairs = pairs.begin();
	const PxU32 nbToProcess = 256;

	Cm::FlushPool* flushPool = mScene.getFlushPool();

	for (PxU32 i = 0; i < nbPairs; i += nbToProcess)
	{
		NpContactCallbackTask* task = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(NpContactCallbackTask)), NpContactCallbackTask)(this, contactPairs+i, getContextId(), PxMin(nbToProcess, nbPairs - i));
		task->setContinuation(continuation);
		task->removeReference();
	}
}

void NpScene::fetchResultsFinish(PxU32* errorState)
{
	NP_CHECK_CORRUPTION_AND_RETURN

	// fetchResultsStart() sets this only after CUDA validation and pre-contact processing succeed.
	if (!mBetweenFetchResults)
		return;

	{
		PX_SIMD_GUARD
		PX_PROFILE_STOP_CROSSTHREAD("Basic.processCallbacks", getContextId());
		PX_PROFILE_ZONE("Basic.fetchResultsFinish", getContextId());

		mBetweenFetchResults = false;
		NP_WRITE_CHECK(this);

		fetchResultsPostContactCallbacks();

		if (errorState)
			*errorState = 0;

		PX_PROFILE_STOP_CROSSTHREAD("Basic.fetchResults", getContextId());
		PX_PROFILE_STOP_CROSSTHREAD("Basic.simulate", getContextId());
#if PX_SUPPORT_OMNI_PVD
		fetchResultsOmniPvd();
#endif
	}

#if PX_SUPPORT_PVD
	mScenePvdClient.frameEnd();
#endif
}

