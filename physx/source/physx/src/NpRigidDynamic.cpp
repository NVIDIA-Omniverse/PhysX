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

#include "NpRigidDynamic.h"
#include "NpRigidActorTemplateInternal.h"
#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

NpRigidDynamic::NpRigidDynamic(const PxTransform& bodyPose)
:	NpRigidDynamicT(PxConcreteType::eRIGID_DYNAMIC, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, PxActorType::eRIGID_DYNAMIC, NpType::eBODY, bodyPose)
{}

NpRigidDynamic::~NpRigidDynamic()
{
}

// PX_SERIALIZATION
void NpRigidDynamic::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpRigidDynamicT::requiresObjects(c);
}

void NpRigidDynamic::preExportDataReset()
{
	NpRigidDynamicT::preExportDataReset();
	if (isKinematic() && getNpScene())
	{
		//Restore dynamic data in case the actor is configured as a kinematic.
		//otherwise we would loose the data for switching the kinematic actor back to dynamic
		//after deserialization. Not necessary if the kinematic is not yet part of
		//a scene since the dynamic data will still hold the original values.
		mCore.restoreDynamicData();
	}
}

NpRigidDynamic* NpRigidDynamic::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpRigidDynamic* obj = PX_PLACEMENT_NEW(address, NpRigidDynamic(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpRigidDynamic);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpRigidDynamic::release()
{
	if(releaseRigidActorT<PxRigidDynamic>(*this))
	{
		PX_ASSERT(!isAPIWriteForbidden());  // the code above should return false in that case
		NpDestroyRigidDynamic(this);
	}
}

void NpRigidDynamic::setGlobalPose(const PxTransform& pose, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(pose.isSane(), "PxRigidDynamic::setGlobalPose: pose is not valid.");

#if PX_CHECKED
	if(npScene)
		npScene->checkPositionSanity(*this, pose, "PxRigidDynamic::setGlobalPose");
#endif

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::setGlobalPose() not allowed while simulation is running. Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::setGlobalPose(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	const PxTransform newPose = pose.getNormalized();	//AM: added to fix 1461 where users read and write orientations for no reason.
	
	const PxTransform body2World = newPose * mCore.getBody2Actor();
	scSetBody2World(body2World);

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, *static_cast<PxRigidActor*>(this), newPose);
	OMNI_PVD_WRITE_SCOPE_END

	if(npScene)
		mShapeManager.markActorForSQUpdate(npScene->getSQAPI(), *this);

	// invalidate the pruning structure if the actor bounds changed
	if(mShapeManager.getPruningStructure())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::setGlobalPose: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	if(npScene && autowake && !(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)))
		wakeUpInternal();
}

PX_FORCE_INLINE void NpRigidDynamic::setKinematicTargetInternal(const PxTransform& targetPose)
{
	// The target is actor related. Transform to body related target
	const PxTransform bodyTarget = targetPose * mCore.getBody2Actor();

	scSetKinematicTarget(bodyTarget);

	NpScene* scene = getNpScene();
	if((mCore.getFlags() & PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES) && scene)
		mShapeManager.markActorForSQUpdate(scene->getSQAPI(), *this);
}

void NpRigidDynamic::setKinematicTarget(const PxTransform& destination)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(destination.isSane(), "PxRigidDynamic::setKinematicTarget: destination is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::setKinematicTarget: Body must be in a scene!");
	PX_CHECK_AND_RETURN((mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setKinematicTarget: Body must be kinematic!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::setKinematicTarget: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

#if PX_CHECKED
	if(npScene)
		npScene->checkPositionSanity(*this, destination, "PxRigidDynamic::setKinematicTarget");
#endif

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::setKinematicTarget() not allowed while simulation is running. Call will be ignored.")

	setKinematicTargetInternal(destination.getNormalized());
}

bool NpRigidDynamic::getKinematicTarget(PxTransform& target) const
{
	NP_READ_CHECK(getNpScene());

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		PxTransform bodyTarget;
		if(mCore.getKinematicTarget(bodyTarget))
		{
			// The internal target is body related. Transform to actor related target
			target = bodyTarget * mCore.getBody2Actor().getInverse();
			return true;
		}
	}
	return false;
}

void NpRigidDynamic::setCMassLocalPose(const PxTransform& pose)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(pose.isSane(), "PxRigidDynamic::setCMassLocalPose pose is not valid.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::setCMassLocalPose() not allowed while simulation is running. Call will be ignored.")

	const PxTransform p = pose.getNormalized();

	const PxTransform oldBody2Actor = mCore.getBody2Actor();

	NpRigidDynamicT::setCMassLocalPoseInternal(p);

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		PxTransform bodyTarget;
		if(mCore.getKinematicTarget(bodyTarget))
		{
			PxTransform actorTarget = bodyTarget * oldBody2Actor.getInverse();  // get old target pose for the actor from the body target
			setKinematicTargetInternal(actorTarget);
		}
	}
}

template<const bool linear>
static PX_FORCE_INLINE PxVec3 getAcceleration(const NpRigidDynamic& dynamic)
{
	NpScene* npScene = dynamic.getNpScene();
	if(!npScene || !(npScene->getFlagsFast() & PxSceneFlag::eENABLE_BODY_ACCELERATIONS))
		return PxVec3(0.0f);

	const PxU32 index = dynamic.getRigidActorArrayIndex();
	if(index>=npScene->mRigidDynamicsAccelerations.size())
		return PxVec3(0.0f);

	return linear ?	npScene->mRigidDynamicsAccelerations[index].mLinAccel
				:	npScene->mRigidDynamicsAccelerations[index].mAngAccel;
}

PxVec3 NpRigidDynamic::getLinearAcceleration() const
{
	return getAcceleration<true>(*this);
}

PxVec3 NpRigidDynamic::getAngularAcceleration() const
{
	return getAcceleration<false>(*this);
}

void NpRigidDynamic::setLinearVelocity(const PxVec3& velocity, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(velocity.isFinite(), "PxRigidDynamic::setLinearVelocity: velocity is not valid.");
	PX_CHECK_AND_RETURN(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setLinearVelocity: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::setLinearVelocity: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::setLinearVelocity() not allowed while simulation is running. Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::setLinearVelocity(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	scSetLinearVelocity(velocity);

	if(npScene && npScene->getFlagsFast() & PxSceneFlag::eENABLE_BODY_ACCELERATIONS)
	{
		const PxU32 index = getRigidActorArrayIndex();

		if(index>=npScene->mRigidDynamicsAccelerations.size())
			npScene->mRigidDynamicsAccelerations.resize(index+1);

		npScene->mRigidDynamicsAccelerations[index].mPrevLinVel = velocity;
	}

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, *static_cast<PxRigidBody*>(this), velocity);

	if(npScene)
		wakeUpInternalNoKinematicTest((!velocity.isZero()), autowake);
}

void NpRigidDynamic::setAngularVelocity(const PxVec3& velocity, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(velocity.isFinite(), "PxRigidDynamic::setAngularVelocity: velocity is not valid.");
	PX_CHECK_AND_RETURN(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setAngularVelocity: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::setAngularVelocity: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::setAngularVelocity() not allowed while simulation is running. Call will be ignored.")

	scSetAngularVelocity(velocity);

	if(npScene && npScene->getFlagsFast() & PxSceneFlag::eENABLE_BODY_ACCELERATIONS)
	{
		const PxU32 index = getRigidActorArrayIndex();

		if(index>=npScene->mRigidDynamicsAccelerations.size())
			npScene->mRigidDynamicsAccelerations.resize(index+1);

		npScene->mRigidDynamicsAccelerations[index].mPrevAngVel = velocity;
	}

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, *static_cast<PxRigidBody*>(this), velocity);

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::setAngularVelocity(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	if(npScene)
		wakeUpInternalNoKinematicTest((!velocity.isZero()), autowake);
}

void NpRigidDynamic::addForce(const PxVec3& force, PxForceMode::Enum mode, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(force.isFinite(), "PxRigidDynamic::addForce: force is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::addForce: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::addForce: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::addForce() not allowed while simulation is running. Call will be ignored.")

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRigidDynamic::addForce: Body must be non-kinematic!");
		return;
	}

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::addForce(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	addSpatialForce(&force, NULL, mode);

	wakeUpInternalNoKinematicTest(!force.isZero(), autowake);
}

void NpRigidDynamic::setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(force.isFinite(), "PxRigidDynamic::setForceAndTorque: force is not valid.");
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxRigidDynamic::setForceAndTorque: torque is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::setForceAndTorque: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::setForceAndTorque: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::setForceAndTorque() not allowed while simulation is running. Call will be ignored.")

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::setForceAndTorque(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRigidDynamic::setForceAndTorque: Body must be non-kinematic!");
		return;
	}

	setSpatialForce(&force, &torque, mode);

	wakeUpInternalNoKinematicTest(!force.isZero(), true);
}

void NpRigidDynamic::addTorque(const PxVec3& torque, PxForceMode::Enum mode, bool autowake)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxRigidDynamic::addTorque: torque is not valid.");
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::addTorque: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::addTorque: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::addTorque() not allowed while simulation is running. Call will be ignored.")

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRigidDynamic::addTorque: Body must be non-kinematic!");
		return;
	}

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::addTorque(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	addSpatialForce(NULL, &torque, mode);

	wakeUpInternalNoKinematicTest(!torque.isZero(), autowake);
}

void NpRigidDynamic::clearForce(PxForceMode::Enum mode)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::clearForce: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::clearForce: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::clearForce() not allowed while simulation is running. Call will be ignored.")

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRigidDynamic::clearForce: Body must be non-kinematic!");
		return;
	}
	
	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::clearForce(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	clearSpatialForce(mode, true, false);
}

void NpRigidDynamic::clearTorque(PxForceMode::Enum mode)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::clearTorque: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::clearTorque: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::clearTorque() not allowed while simulation is running. Call will be ignored.")

	if(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxRigidDynamic::clearTorque: Body must be non-kinematic!");
		return;
	}

	if (npScene && (npScene->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API) && npScene->isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxRigidDynamic::clearTorque(): it is illegal to call this method if PxSceneFlag::eENABLE_DIRECT_GPU_API is enabled!");
		return;
	}

	clearSpatialForce(mode, false, true);
}

bool NpRigidDynamic::isSleeping() const
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene(), "PxRigidDynamic::isSleeping: Body must be in a scene.", true);

	PX_CHECK_SCENE_API_READ_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxRigidDynamic::isSleeping() not allowed while simulation is running.", true);

	return mCore.isSleeping();
}

void NpRigidDynamic::setSleepThreshold(PxReal threshold)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(threshold), "PxRigidDynamic::setSleepThreshold: invalid float.");
	PX_CHECK_AND_RETURN(threshold>=0.0f, "PxRigidDynamic::setSleepThreshold: threshold must be non-negative!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::setSleepThreshold() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, sleepThreshold, static_cast<PxRigidDynamic&>(*this), threshold); // @@@

	mCore.setSleepThreshold(threshold);
	UPDATE_PVD_PROPERTY_BODY
}

PxReal NpRigidDynamic::getSleepThreshold() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getSleepThreshold();
}

void NpRigidDynamic::setStabilizationThreshold(PxReal threshold)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(threshold), "PxRigidDynamic::setSleepThreshold: invalid float.");
	PX_CHECK_AND_RETURN(threshold>=0.0f, "PxRigidDynamic::setSleepThreshold: threshold must be non-negative!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::setStabilizationThreshold() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, stabilizationThreshold, static_cast<PxRigidDynamic&>(*this), threshold); // @@@

	mCore.setFreezeThreshold(threshold);
	UPDATE_PVD_PROPERTY_BODY
}

PxReal NpRigidDynamic::getStabilizationThreshold() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getFreezeThreshold();
}

void NpRigidDynamic::setWakeCounter(PxReal wakeCounterValue)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(wakeCounterValue), "PxRigidDynamic::setWakeCounter: invalid float.");
	PX_CHECK_AND_RETURN(wakeCounterValue>=0.0f, "PxRigidDynamic::setWakeCounter: wakeCounterValue must be non-negative!");
	PX_CHECK_AND_RETURN(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setWakeCounter: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::setWakeCounter: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::setWakeCounter() not allowed while simulation is running. Call will be ignored.")

	scSetWakeCounter(wakeCounterValue);

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, wakeCounter, static_cast<PxRigidDynamic&>(*this), wakeCounterValue); // @@@
}

PxReal NpRigidDynamic::getWakeCounter() const
{
	NP_READ_CHECK(getNpScene());

	PX_CHECK_SCENE_API_READ_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxRigidDynamic::getWakeCounter() not allowed while simulation is running.", 0.0f);

	return mCore.getWakeCounter();
}

void NpRigidDynamic::wakeUp()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::wakeUp: Body must be in a scene.");
	PX_CHECK_AND_RETURN(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::wakeUp: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::wakeUp: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, "PxRigidDynamic::wakeUp() not allowed while simulation is running. Call will be ignored.")

	scWakeUp();
}

void NpRigidDynamic::putToSleep()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(npScene, "PxRigidDynamic::putToSleep: Body must be in a scene.");
	PX_CHECK_AND_RETURN(!(mCore.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::putToSleep: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(mCore.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)), "PxRigidDynamic::putToSleep: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::putToSleep() not allowed while simulation is running. Call will be ignored.")

	scPutToSleep();
}

void NpRigidDynamic::setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(positionIters > 0, "PxRigidDynamic::setSolverIterationCounts: positionIters must be more than zero!");
	PX_CHECK_AND_RETURN(positionIters <= 255, "PxRigidDynamic::setSolverIterationCounts: positionIters must be no greater than 255!");
	PX_CHECK_AND_RETURN(velocityIters <= 255, "PxRigidDynamic::setSolverIterationCounts: velocityIters must be no greater than 255!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

	scSetSolverIterationCounts((velocityIters & 0xff) << 8 | (positionIters & 0xff));

	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, positionIterations, static_cast<PxRigidDynamic&>(*this), positionIters); // @@@
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, velocityIterations, static_cast<PxRigidDynamic&>(*this), velocityIters); // @@@
	OMNI_PVD_WRITE_SCOPE_END
}

void NpRigidDynamic::getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const
{
	NP_READ_CHECK(getNpScene());

	PxU16 x = mCore.getSolverIterationCounts();
	velocityIters = PxU32(x >> 8);
	positionIters = PxU32(x & 0xff);
}

void NpRigidDynamic::setContactReportThreshold(PxReal threshold)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(threshold), "PxRigidDynamic::setContactReportThreshold: invalid float.");
	PX_CHECK_AND_RETURN(threshold >= 0.0f, "PxRigidDynamic::setContactReportThreshold: Force threshold must be greater than zero!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxRigidDynamic::setContactReportThreshold() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, contactReportThreshold, static_cast<PxRigidDynamic&>(*this), threshold); // @@@

	mCore.setContactReportThreshold(threshold<0 ? 0 : threshold);
	UPDATE_PVD_PROPERTY_BODY
}

PxReal NpRigidDynamic::getContactReportThreshold() const
{
	NP_READ_CHECK(getNpScene());

	return mCore.getContactReportThreshold();
}

PxU32 physx::NpRigidDynamicGetShapes(NpRigidDynamic& actor, NpShape* const*& shapes, bool* isCompound)
{
	NpShapeManager& sm = actor.getShapeManager();
	shapes = sm.getShapes();
	if(isCompound)
		*isCompound = sm.isSqCompound();
	return sm.getNbShapes();
}

void NpRigidDynamic::switchToNoSim()
{
	NpActor::scSwitchToNoSim();
	scPutToSleepInternal();
}

void NpRigidDynamic::switchFromNoSim()
{
	NpActor::scSwitchFromNoSim();
}

void NpRigidDynamic::wakeUpInternalNoKinematicTest(bool forceWakeUp, bool autowake)
{
	NpScene* scene = getNpScene();
	PX_ASSERT(scene);
	PxReal wakeCounterResetValue = scene->getWakeCounterResetValueInternal();

	PxReal wakeCounter = mCore.getWakeCounter();

	bool needsWakingUp = mCore.isSleeping() && (autowake || forceWakeUp);
	if (autowake && (wakeCounter < wakeCounterResetValue))
	{
		wakeCounter = wakeCounterResetValue;
		needsWakingUp = true;
	}

	if (needsWakingUp)
		scWakeUpInternal(wakeCounter);
}

PxRigidDynamicLockFlags NpRigidDynamic::getRigidDynamicLockFlags() const
{
	return mCore.getRigidDynamicLockFlags();
}

void NpRigidDynamic::setRigidDynamicLockFlags(PxRigidDynamicLockFlags flags)
{
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxRigidDynamic::setRigidDynamicLockFlags() not allowed while simulation is running. Call will be ignored.")
	scSetLockFlags(flags);

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rigidDynamicLockFlags, static_cast<PxRigidDynamic&>(*this), flags); // @@@
}

void NpRigidDynamic::setRigidDynamicLockFlag(PxRigidDynamicLockFlag::Enum flag, bool value)
{
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxRigidDynamic::setRigidDynamicLockFlag() not allowed while simulation is running. Call will be ignored.")
	PxRigidDynamicLockFlags flags = mCore.getRigidDynamicLockFlags();
	if (value)
		flags = flags | flag;
	else
		flags = flags & (~flag);

	scSetLockFlags(flags);

	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxRigidDynamic, rigidDynamicLockFlags, static_cast<PxRigidDynamic&>(*this), flags); // @@@
}

PxRigidDynamicGPUIndex NpRigidDynamic::getGPUIndex() const
{
	NP_READ_CHECK(getNpScene());
	if(getNpScene())
	{
		return mCore.getInternalIslandNodeIndex().index();
	}
	return PX_INVALID_NODE;
}
