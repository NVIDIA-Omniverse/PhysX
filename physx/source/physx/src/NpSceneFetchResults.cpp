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

#include "NpScene.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "NpArticulationSensor.h"
#include "NpAggregate.h"
#if PX_SUPPORT_GPU_PHYSX
	#include "NpSoftBody.h"
	#include "NpParticleSystem.h"
	#include "NpFEMCloth.h"
	#include "NpHairSystem.h"
	#include "cudamanager/PxCudaContextManager.h"
	#include "cudamanager/PxCudaContext.h"	
#endif
#include "ScArticulationSim.h"
#include "ScArticulationTendonSim.h"
#include "CmCollection.h"
#include "PxsSimulationController.h"
#include "common/PxProfileZone.h"
#include "BpBroadPhase.h"
#include "BpAABBManagerBase.h"

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
	mScene.getSimulationController()->syncParticleData();
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
	setAPIWriteToAllowed();

	mPhysicsDone.reset();				// allow Physics to run again
	mCollisionDone.reset();
}

bool NpScene::fetchResults(bool block, PxU32* errorState)
{
	if(getSimulationStage() != Sc::SimulationStage::eADVANCE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchResults: fetchResults() called illegally! It must be called after advance() or simulate()");

	if(!checkResultsInternal(block))
		return false;

#if PX_SUPPORT_GPU_PHYSX
	if (mCudaContextManager)
	{
		if (mScene.isUsingGpuDynamicsOrBp())
		{
			PxCUresult res = mCudaContextManager->getCudaContext()->getLastError();
			if (res)
			{
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "PhysX Internal CUDA error. Simulation can not continue!");
				if(errorState)
					*errorState = res;
			}
		}
	}
#endif

	PX_SIMD_GUARD;

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
		OmniPvdPxSampler* omniPvdSampler = NpPhysics::getInstance().mOmniPvdSampler;
		if (omniPvdSampler && omniPvdSampler->isSampling())
		{
			//send all xforms updated by the sim:
			PxU32 nActiveActors;
			PxActor ** activeActors = mScene.getActiveActors(nActiveActors);
			while (nActiveActors--)
			{
				PxActor * a = *activeActors++;
				if ((a->getType() == PxActorType::eRIGID_STATIC) || (a->getType() == PxActorType::eRIGID_DYNAMIC))
				{
					PxRigidActor* ra = static_cast<PxRigidActor*>(a);
					PxTransform t = ra->getGlobalPose();
					OMNI_PVD_SET(PxRigidActor, translation, *ra, t.p)
					OMNI_PVD_SET(PxRigidActor, rotation, *ra, t.q)

					if (a->getType() == PxActorType::eRIGID_DYNAMIC)
					{
						PxRigidDynamic* rdyn = static_cast<PxRigidDynamic*>(a);
						PxRigidBody& rb = *static_cast<PxRigidBody*>(a);
						
						const PxVec3 linVel = rdyn->getLinearVelocity();
						OMNI_PVD_SET(PxRigidBody, linearVelocity, rb, linVel)

						const PxVec3 angVel = rdyn->getAngularVelocity();
						OMNI_PVD_SET(PxRigidBody, angularVelocity, rb, angVel)

						const PxRigidBodyFlags rFlags = rdyn->getRigidBodyFlags();
						OMNI_PVD_SET(PxRigidBody, rigidBodyFlags, rb, rFlags)
					}
					
				}
				else if (a->getType() == PxActorType::eARTICULATION_LINK)
				{
					PxArticulationLink* pxArticulationParentLink = 0;
					PxArticulationLink* pxArticulationLink = static_cast<PxArticulationLink*>(a);
					PxArticulationJointReducedCoordinate* pxArticulationJoint = pxArticulationLink->getInboundJoint();
					if (pxArticulationJoint)
					{
						pxArticulationParentLink = &(pxArticulationJoint->getParentArticulationLink());

						PxArticulationJointReducedCoordinate& jcord = *pxArticulationJoint;
						PxReal vals[6];
						for (PxU32 ax = 0; ax < 6; ++ax)
							vals[ax] = jcord.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
						OMNI_PVD_SETB(PxArticulationJointReducedCoordinate, jointPosition, jcord, vals, sizeof(vals));
						for (PxU32 ax = 0; ax < 6; ++ax)
							vals[ax] = jcord.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
						OMNI_PVD_SETB(PxArticulationJointReducedCoordinate, jointVelocity, jcord, vals, sizeof(vals));
					}

					physx::PxTransform TArtLinkLocal;
					if (pxArticulationParentLink)
					{
						// TGlobal = TFatherGlobal * TLocal
						// Inv(TFatherGlobal)* TGlobal = Inv(TFatherGlobal)*TFatherGlobal * TLocal
						// Inv(TFatherGlobal)* TGlobal = TLocal
						// TLocal = Inv(TFatherGlobal) * TGlobal
						//physx::PxTransform TParentGlobal = pxArticulationParentLink->getGlobalPose();
						physx::PxTransform TParentGlobalInv = pxArticulationParentLink->getGlobalPose().getInverse();
						physx::PxTransform TArtLinkGlobal = pxArticulationLink->getGlobalPose();
						// PT:: tag: scalar transform*transform
						TArtLinkLocal = TParentGlobalInv * TArtLinkGlobal;
					}
					else {
						TArtLinkLocal = pxArticulationLink->getGlobalPose();
						OMNI_PVD_SET(PxArticulationReducedCoordinate, worldBounds, pxArticulationLink->getArticulation(), pxArticulationLink->getArticulation().getWorldBounds());
					}
					OMNI_PVD_SET(PxRigidActor, translation, static_cast<PxRigidActor&>(*a), TArtLinkLocal.p)
					OMNI_PVD_SET(PxRigidActor, rotation, static_cast<PxRigidActor&>(*a), TArtLinkLocal.q)
					
					const PxVec3 linVel = pxArticulationLink->getLinearVelocity();
					OMNI_PVD_SET(PxRigidBody, linearVelocity, static_cast<PxRigidBody&>(*a), linVel)

					const PxVec3 angVel = pxArticulationLink->getAngularVelocity();
					OMNI_PVD_SET(PxRigidBody, angularVelocity, static_cast<PxRigidBody&>(*a), angVel)

					const PxRigidBodyFlags rFlags = pxArticulationLink->getRigidBodyFlags();
					OMNI_PVD_SET(PxRigidBody, rigidBodyFlags, static_cast<PxRigidBody&>(*a), rFlags)
				}

				const PxBounds3 worldBounds = a->getWorldBounds();
				OMNI_PVD_SET(PxActor, worldBounds, *a, worldBounds)

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
							PxRigidActor *ra0, *ra1; c.getActors(ra0, ra1);
							bool ra0static = !ra0 || !!ra0->is<PxRigidStatic>(), ra1static = !ra1 || !!ra1->is<PxRigidStatic>();
							// this check is to not update a joint twice
							if ((ra == ra0 && (ra1static || ra0 > ra1)) || (ra == ra1 && (ra0static || ra1 > ra0)))
								c.getCore().getPxConnector()->updateOmniPvdProperties();
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

			// send contacts info
			omniPvdSampler->streamSceneContacts(*this);

			//end frame
			omniPvdSampler->sampleScene(this);
		}
#endif
	}

#if PX_SUPPORT_PVD
	mScenePvdClient.frameEnd();
#endif
	return true;
}

bool NpScene::fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block)
{
	if (getSimulationStage() != Sc::SimulationStage::eADVANCE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchResultsStart: fetchResultsStart() called illegally! It must be called after advance() or simulate()");

	if (!checkResultsInternal(block))
		return false;

	PX_SIMD_GUARD;
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

void NpContactCallbackTask::setData(NpScene* scene, const PxContactPairHeader* contactPairHeaders, const uint32_t nbContactPairHeaders)
{
	mScene = scene;
	mContactPairHeaders = contactPairHeaders;
	mNbContactPairHeaders = nbContactPairHeaders;
}

void NpContactCallbackTask::run()
{
	PxSimulationEventCallback* callback = mScene->getSimulationEventCallback();
	if (!callback)
		return;

	mScene->lockRead();
	for (uint32_t i = 0; i<mNbContactPairHeaders; ++i)
	{
		const PxContactPairHeader& pairHeader = mContactPairHeaders[i];
		callback->onContact(pairHeader, pairHeader.pairs, pairHeader.nbPairs);
	}
	mScene->unlockRead();
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
		NpContactCallbackTask* task = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(NpContactCallbackTask)), NpContactCallbackTask)();
		task->setData(this, contactPairs+i, PxMin(nbToProcess, nbPairs - i));
		task->setContinuation(continuation);
		task->removeReference();
	}
}

void NpScene::fetchResultsFinish(PxU32* errorState)
{
#if PX_SUPPORT_GPU_PHYSX
	if (mCudaContextManager)
	{
		if (mScene.isUsingGpuDynamicsOrBp())
		{
			if (mCudaContextManager->getCudaContext()->getLastError())
			{
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "PhysX Internal CUDA error. Simulation can not continue!");
				if (errorState)
					*errorState = mCudaContextManager->getCudaContext()->getLastError();
			}
		}
	}
#endif

	{
		PX_SIMD_GUARD;
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
		OmniPvdPxSampler* omniPvdSampler = NpPhysics::getInstance().mOmniPvdSampler;
		if (omniPvdSampler && omniPvdSampler->isSampling())
		{
			omniPvdSampler->sampleScene(this);
		}
#endif
	}

#if PX_SUPPORT_PVD
	mScenePvdClient.frameEnd();
#endif
}

