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
#endif
#include "ScArticulationSim.h"
#include "ScArticulationTendonSim.h"
#include "CmCollection.h"
#include "PxsSimulationController.h"
#include "common/PxProfileZone.h"
#include "BpBroadPhase.h"
#include "BpAABBManagerBase.h"
#include "omnipvd/NpOmniPvdSetData.h"

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
	if (mCorruptedState) // silent if scene state is corrupted.
		return;

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
	// PT: I put this here for now, as initially we even considered making this a PxExtensions helper. To make it more
	// efficient / multithread it we could eventually move this deeper in the Sc-level pipeline.
	if((mScene.getFlags() & PxSceneFlag::eENABLE_BODY_ACCELERATIONS) && !(mScene.getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API))
	{
		// PT: we store the acceleration data in a separate/dedicated array, so that memory usage doesn't increase for
		// people who don't use the flag (i.e. most users). The drawback is that there's more cache misses here during the
		// gather phase (reading velocities) compared to a design where we would compute the accelerations at the same time
		// these velocities are stored back into the core objects. Pros & cons here. Unrolling that loop and adding some
		// prefetch calls could help, if needed.

		const float oneOverDt = mElapsedTime != 0.0f ? 1.0f/mElapsedTime : 0.0f;

		// PT: this version assumes we index mRigidDynamicsAccelerations as we do mRigidDynamics (with getRigidActorArrayIndex), i.e. we
		// need to update that array when objects are removed (see removeFromRigidActorListT)

		// PT: another (archived) version used getRigidActorSceneIndex(). The acceleration array could become larger than necessary,
		// but the index was constant for the lifetime of the object. At this point we could just have used a hashmap.

		PxU32 size = mRigidDynamics.size();
		NpRigidDynamic** rigidDynamics = mRigidDynamics.begin();

		if(mRigidDynamicsAccelerations.size()!=size)
			mRigidDynamicsAccelerations.resize(size);

		Acceleration* accels = mRigidDynamicsAccelerations.begin();
		while(size--)
		{
			const NpRigidDynamic* current = *rigidDynamics++;
			const Sc::BodyCore&	core = current->getCore();

			const PxVec3 linVel = core.getLinearVelocity();
			const PxVec3 angVel = core.getAngularVelocity();

			const PxVec3 deltaLinVel = linVel - accels->mPrevLinVel;
			const PxVec3 deltaAngVel = angVel - accels->mPrevAngVel;

			accels->mLinAccel = deltaLinVel * oneOverDt;
			accels->mAngAccel = deltaAngVel * oneOverDt;

			accels->mPrevLinVel = linVel;
			accels->mPrevAngVel = angVel;

			accels++;
		}
	}

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
	if (mCorruptedState)
		return true;

	if(getSimulationStage() != Sc::SimulationStage::eADVANCE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchResults: fetchResults() called illegally! It must be called after advance() or simulate()");

	if(!checkResultsInternal(block)) // this should wait on the mPhysicsDone event, which is set in the SceneCompletion task
		return false;


#if PX_SUPPORT_GPU_PHYSX
	if (!checkSceneStateAndCudaErrors())
		return true;
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
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)

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
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, *ra, t);

					if (a->getType() == PxActorType::eRIGID_DYNAMIC)
					{
						PxRigidDynamic* rdyn = static_cast<PxRigidDynamic*>(a);
						PxRigidBody& rb = *static_cast<PxRigidBody*>(a);
						
						const PxVec3 linVel = rdyn->getLinearVelocity();
						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, rb, linVel)

						const PxVec3 angVel = rdyn->getAngularVelocity();
						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, rb, angVel)

						const PxRigidBodyFlags rFlags = rdyn->getRigidBodyFlags();
						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, rb, rFlags)

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
						for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
							vals[ax] = jcord.getJointPosition(static_cast<PxArticulationAxis::Enum>(ax));
						OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, jcord, vals, PxArticulationAxis::eCOUNT);
						for (PxU32 ax = 0; ax < PxArticulationAxis::eCOUNT; ++ax)
							vals[ax] = jcord.getJointVelocity(static_cast<PxArticulationAxis::Enum>(ax));
						OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, jcord, vals, PxArticulationAxis::eCOUNT);
					}

					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, static_cast<PxRigidActor&>(*a), pxArticulationLink->getGlobalPose());

					const PxVec3 linVel = pxArticulationLink->getLinearVelocity();
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, static_cast<PxRigidBody&>(*a), linVel)

					const PxVec3 angVel = pxArticulationLink->getAngularVelocity();
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, static_cast<PxRigidBody&>(*a), angVel)

					const PxRigidBodyFlags rFlags = pxArticulationLink->getRigidBodyFlags();
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, rigidBodyFlags, static_cast<PxRigidBody&>(*a), rFlags)
				}

				const PxBounds3 worldBounds = a->getWorldBounds();
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, *a, worldBounds)

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

			PxArticulationReducedCoordinate*const* articulations = mArticulations.getEntries();
			const PxU32 nbArticulations = mArticulations.size();
			for( PxU32 i = 0 ; i < nbArticulations ;i++)
			{				
				PxArticulationReducedCoordinate* articulation = (articulations[i]);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, wakeCounter, *articulation, articulation->getWakeCounter());
				const PxBounds3 worldBounds = articulation->getWorldBounds();
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, worldBounds, *articulation, worldBounds);
				bool isSleeping = articulation->isSleeping();
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, isSleeping, *articulation, isSleeping);
			}
			// send contacts info
			omniPvdSampler->streamSceneContacts(*this);

			// process particle data
			if (mPBDParticleSystems.size() > 0)
			{
				fetchResultsParticleSystem();
				const PxPBDParticleSystem* const* particleSystems = mPBDParticleSystems.getEntries();
				const PxU32 particleSystemCount = mPBDParticleSystems.size();
				for (PxU32 i = 0; i < particleSystemCount; i++)
				{
					const NpPBDParticleSystem* npPs = static_cast<const NpPBDParticleSystem*>(particleSystems[i]);
					{
						const PxBounds3 worldBounds = npPs->getWorldBounds();
						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, *npPs, worldBounds);

						const PxArray<PxsParticleBuffer*>& pxsBuffers = npPs->getCore().getShapeCore().getLLCore().mParticleBuffers;
						const PxArray<NpParticleBuffer*>& npBuffers = npPs->mParticleBuffers;
						for (PxU32 b = 0; b < pxsBuffers.size(); ++b)
						{
							const PxsParticleBuffer& pxsBuffer = *pxsBuffers[b];
							if (pxsBuffer.getPositionInvMassesH())
							{
								PxParticleBuffer* pxBuffer = npBuffers[b];
								PxReal* values = reinterpret_cast<PxReal*>(pxsBuffer.getPositionInvMassesH());
								PxU32 nbValues = pxsBuffer.getNbActiveParticles() * 4;
								OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, positionInvMasses, *pxBuffer, values, nbValues);
								OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, flatListStartIndex, *pxBuffer, pxsBuffer.getFlatListStartIndex());
							}
						}
					}
					{
						const PxArray<PxsParticleBuffer*>& pxsBuffers = npPs->getCore().getShapeCore().getLLCore().mParticleDiffuseBuffers;
						const PxArray<NpParticleAndDiffuseBuffer*>& npBuffers = npPs->mParticleDiffuseBuffers;
						for (PxU32 b = 0; b < pxsBuffers.size(); ++b)
						{
							const PxsParticleBuffer& pxsBuffer = *pxsBuffers[b];
							if (pxsBuffer.getPositionInvMassesH())
							{
								PxParticleBuffer* pxBuffer = npBuffers[b];
								PxReal* values = reinterpret_cast<PxReal*>(pxsBuffer.getPositionInvMassesH());
								PxU32 nbValues = pxsBuffer.getNbActiveParticles() * 4;
								OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, positionInvMasses, *pxBuffer, values, nbValues);
							}
							//TODO add diffuse particles
						}
					}
					{
						const PxArray<PxsParticleBuffer*>& pxsBuffers = npPs->getCore().getShapeCore().getLLCore().mParticleClothBuffers;
						const PxArray<NpParticleClothBuffer*>& npBuffers = npPs->mParticleClothBuffers;
						for (PxU32 b = 0; b < pxsBuffers.size(); ++b)
						{
							const PxsParticleBuffer& pxsBuffer = *pxsBuffers[b];
							if (pxsBuffer.getPositionInvMassesH())
							{
								PxParticleBuffer* pxBuffer = npBuffers[b];
								PxReal* values = reinterpret_cast<PxReal*>(pxsBuffer.getPositionInvMassesH());
								PxU32 nbValues = pxsBuffer.getNbActiveParticles() * 4;
								OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, positionInvMasses, *pxBuffer, values, nbValues);
							}
						}
					}
					{
						const PxArray<PxsParticleBuffer*>& pxsBuffers = npPs->getCore().getShapeCore().getLLCore().mParticleRigidBuffers;
						const PxArray<NpParticleRigidBuffer*>& npBuffers = npPs->mParticleRigidBuffers;
						for (PxU32 b = 0; b < pxsBuffers.size(); ++b)
						{
							const PxsParticleBuffer& pxsBuffer = *pxsBuffers[b];
							if (pxsBuffer.getPositionInvMassesH())
							{
								PxParticleBuffer* pxBuffer = npBuffers[b];
								PxReal* values = reinterpret_cast<PxReal*>(pxsBuffer.getPositionInvMassesH());
								PxU32 nbValues = pxsBuffer.getNbActiveParticles() * 4;
								OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxParticleBuffer, positionInvMasses, *pxBuffer, values, nbValues);
							}
						}
					}
				}

			}

			NpOmniPvdSceneClient& ovdClient = getSceneOvdClientInternal();
			ovdClient.resetForces();
			ovdClient.incrementFrame(*pvdWriter, true);
			OMNI_PVD_WRITE_SCOPE_END
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
	if (mCorruptedState)
		return true;

	if (getSimulationStage() != Sc::SimulationStage::eADVANCE)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchResultsStart: fetchResultsStart() called illegally! It must be called after advance() or simulate()");

	if (!checkResultsInternal(block))
		return false;

#if PX_SUPPORT_GPU_PHYSX
	if (!checkSceneStateAndCudaErrors())
		return true;
#endif

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
	if (mCorruptedState)
		return;

	// AD: we already checked the cuda error state in fetchResultsStart, there is no GPU work going on in-between.

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
			OMNI_PVD_GET_WRITER(pvdWriter)
			if (pvdWriter)
			{
				NpOmniPvdSceneClient& ovdClient = getSceneOvdClientInternal();
				ovdClient.resetForces();
				ovdClient.incrementFrame(*pvdWriter, true);
			}
		}
#endif
	}

#if PX_SUPPORT_PVD
	mScenePvdClient.frameEnd();
#endif
}

