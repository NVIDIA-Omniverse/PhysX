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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/commands/PxVehicleCommandStates.h"

#include "vehicle2/drivetrain/PxVehicleDrivetrainParams.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainStates.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle2/physxActor/PxVehiclePhysXActorFunctions.h"
#include "vehicle2/physxActor/PxVehiclePhysXActorStates.h"

#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintStates.h"
#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintHelpers.h"

#include "vehicle2/wheel/PxVehicleWheelParams.h"
#include "vehicle2/wheel/PxVehicleWheelStates.h"

#include "PxRigidDynamic.h"
#include "PxArticulationLink.h"
#include "PxArticulationReducedCoordinate.h"

namespace physx
{
namespace vehicle2
{

void PxVehiclePhysxActorWakeup(
 const PxVehicleCommandState& commands,
 const PxVehicleEngineDriveTransmissionCommandState* transmissionCommands,
 const PxVehicleGearboxParams* gearParams,
 const PxVehicleGearboxState* gearState,
 PxRigidBody& physxActor,
 PxVehiclePhysXSteerState& physxSteerState)
{
	PX_CHECK_AND_RETURN(!(physxActor.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC), "PxVehiclePhysxActorWakeup: physxActor is kinematic. This is not supported");

	PxRigidDynamic* rd = physxActor.is<PxRigidDynamic>();
	PxArticulationLink* link = physxActor.is<PxArticulationLink>();
	bool intentToChangeState = ((commands.throttle != 0.0f) || ((PX_VEHICLE_UNSPECIFIED_STEER_STATE != physxSteerState.previousSteerCommand) && (commands.steer != physxSteerState.previousSteerCommand)));
	if (!intentToChangeState && transmissionCommands)
	{
		PX_ASSERT(gearParams);
		PX_ASSERT(gearState);

		// Manual gear switch (includes going from neutral or reverse to automatic)
		intentToChangeState = (gearState->currentGear == gearState->targetGear) &&
			(((transmissionCommands->targetGear != PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR) && (gearState->targetGear != transmissionCommands->targetGear)) ||
			 ((transmissionCommands->targetGear == PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR) && (gearState->currentGear <= gearParams->neutralGear)));
	}

	if(rd && rd->isSleeping() && intentToChangeState)
	{
		rd->wakeUp();
	}
	else if(link && link->getArticulation().isSleeping() && intentToChangeState)
	{
		link->getArticulation().wakeUp();
	}

	physxSteerState.previousSteerCommand = commands.steer;
}

bool PxVehiclePhysxActorSleepCheck
(const PxVehicleAxleDescription& axleDescription,
 const PxRigidBody& physxActor,
 const PxVehicleEngineParams* engineParams,
 PxVehicleRigidBodyState& rigidBodyState,
 PxVehiclePhysXConstraints& physxConstraints,
 PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
 PxVehicleEngineState* engineState)
{
	PX_CHECK_AND_RETURN_VAL(!(physxActor.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC), "PxVehiclePhysxActorSleepCheck: physxActor is kinematic. This is not supported", false);

	const PxRigidDynamic* rd = physxActor.is<PxRigidDynamic>();
	const PxArticulationLink* link = physxActor.is<PxArticulationLink>();

	bool isSleeping = false;
	if (rd && rd->isSleeping())
	{
		isSleeping = true;
	}
	else if (link && link->getArticulation().isSleeping())
	{
		isSleeping = true;
	}

	if (isSleeping)
	{
		// note: pose is not copied as isSleeping() implies that pose was not changed by the
		//       simulation step (see docu). If a user explicitly calls putToSleep or manually
		//       changes pose, it will only get reflected in the vehicle rigid body state once
		//       the body wakes up.

		rigidBodyState.linearVelocity = PxVec3(PxZero);
		rigidBodyState.angularVelocity = PxVec3(PxZero);
		rigidBodyState.previousLinearVelocity = PxVec3(PxZero);
		rigidBodyState.previousAngularVelocity = PxVec3(PxZero);

		bool markConstraintsDirty = false;
		for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
		{
			const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
			PxVehicleWheelRigidBody1dState& wheelState = wheelRigidBody1dStates[wheelId];
			wheelState.rotationSpeed = 0.0f;
			wheelState.correctedRotationSpeed = 0.0f;

			// disable constraints if there are active ones. The idea is that if something
			// is crashing into a sleeping vehicle and waking it up, then the vehicle should
			// be able to move if the impact was large enough. Thus, there is also no logic
			// to reset the sticky tire timers, for example. If the impact was small, the
			// constraints should potentially kick in again in the subsequent sim step.
			PxVehiclePhysXConstraintState& constraintState = physxConstraints.constraintStates[wheelId];
			if (constraintState.tireActiveStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] ||
				constraintState.tireActiveStatus[PxVehicleTireDirectionModes::eLATERAL] ||
				constraintState.suspActiveStatus)
			{
				constraintState.tireActiveStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] = false;
				constraintState.tireActiveStatus[PxVehicleTireDirectionModes::eLATERAL] = false;
				constraintState.suspActiveStatus = false;

				markConstraintsDirty = true;
			}
		}

		if (markConstraintsDirty)
			PxVehicleConstraintsDirtyStateUpdate(physxConstraints);

		if (engineState)
		{
			PX_ASSERT(engineParams);
			engineState->rotationSpeed = engineParams->idleOmega;
		}
	}

	return isSleeping;
}

PX_FORCE_INLINE static void setWakeCounter(const PxReal wakeCounter, PxRigidDynamic* rd, PxArticulationLink* link)
{
	if (rd && (!(rd->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		rd->setWakeCounter(wakeCounter);
	}
	else if (link)
	{
		link->getArticulation().setWakeCounter(wakeCounter);
	}
}

void PxVehiclePhysxActorKeepAwakeCheck
(const PxVehicleAxleDescription& axleDescription,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
 const PxReal wakeCounterThreshold,
 const PxReal wakeCounterResetValue,
 const PxVehicleGearboxState* gearState,
 const PxReal* throttle,
 PxRigidBody& physxActor)
{
	PX_CHECK_AND_RETURN(!(physxActor.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC), "PxVehiclePhysxActorKeepAwakeCheck: physxActor is kinematic. This is not supported");

	PxRigidDynamic* rd = physxActor.is<PxRigidDynamic>();
	PxArticulationLink* link = physxActor.is<PxArticulationLink>();

	PxReal wakeCounter = PX_MAX_REAL;
	if (rd)
	{
		wakeCounter = rd->getWakeCounter();
	}
	else if (link)
	{
		wakeCounter = link->getArticulation().getWakeCounter();
	}

	if (wakeCounter < wakeCounterThreshold)
	{
		if ((throttle && ((*throttle) > 0.0f)) ||
			(gearState && (gearState->currentGear != gearState->targetGear)))
		{
			setWakeCounter(wakeCounterResetValue, rd, link);

			return;
		}

		for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
		{
			const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
			const PxVehicleWheelParams& wheelParam = wheelParams[wheelId];
			const PxVehicleWheelRigidBody1dState& wheelState = wheelRigidBody1dStates[wheelId];

			// note: the translational part of the energy is ignored here as this is mainly for
			//       scenarios where there is almost no translation but the wheels are spinning

			const PxReal normalizedEnergy = (0.5f * wheelState.correctedRotationSpeed * wheelState.correctedRotationSpeed *
				wheelParam.moi) / wheelParam.mass;

			PxReal sleepThreshold = PX_MAX_REAL;
			if (rd)
			{
				sleepThreshold = rd->getSleepThreshold();
			}
			else if (link)
			{
				sleepThreshold = link->getArticulation().getSleepThreshold();
			}

			if (normalizedEnergy > sleepThreshold)
			{
				setWakeCounter(wakeCounterResetValue, rd, link);
				return;
			}
		}
	}
}

void PxVehicleReadRigidBodyStateFromPhysXActor
(const PxRigidBody& physxActor,
 PxVehicleRigidBodyState& rigidBodyState)
{
	PX_CHECK_AND_RETURN(!(physxActor.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC), "PxVehicleReadRigidBodyStateFromPhysXActor: physxActor is kinematic. This is not supported");

	const PxRigidDynamic* rd = physxActor.is<PxRigidDynamic>();
	const PxArticulationLink* link = physxActor.is<PxArticulationLink>();
	PX_ASSERT(rd || link);

	if(rd)
	{
		rigidBodyState.pose = physxActor.getGlobalPose()*physxActor.getCMassLocalPose();
		rigidBodyState.angularVelocity = rd->getAngularVelocity();
		rigidBodyState.linearVelocity = rd->getLinearVelocity();
	}
	else
	{
		rigidBodyState.pose = physxActor.getGlobalPose()*physxActor.getCMassLocalPose();
		rigidBodyState.angularVelocity = link->getAngularVelocity();
		rigidBodyState.linearVelocity = link->getLinearVelocity();
	}
	rigidBodyState.previousLinearVelocity = rigidBodyState.linearVelocity;
	rigidBodyState.previousAngularVelocity = rigidBodyState.angularVelocity;
}

void PxVehicleWriteWheelLocalPoseToPhysXWheelShape
(const PxTransform& wheelLocalPose, const PxTransform& wheelShapeLocalPose, PxShape* shape)
{
	if(!shape)
		return;

	PxRigidActor* ra = shape->getActor();
	if(!ra)
		return;

	PxRigidBody* rb = ra->is<PxRigidBody>();
	if(!rb)
		return;

	PX_CHECK_AND_RETURN(!(rb->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC), "PxVehicleWriteWheelLocalPoseToPhysXWheelShape: shape is attached to a kinematic actor.  This is not supported");

	//Local pose in actor frame.
	const PxTransform& localPoseCMassFrame = wheelLocalPose*wheelShapeLocalPose;
	const PxTransform cmassLocalPose = rb->getCMassLocalPose();
	const PxTransform localPoseActorFrame = cmassLocalPose * localPoseCMassFrame;

	//Apply the local pose to the shape.
	shape->setLocalPose(localPoseActorFrame);
}

void PxVehicleWriteRigidBodyStateToPhysXActor
(const PxVehiclePhysXActorUpdateMode::Enum updateMode,
 const PxVehicleRigidBodyState& rigidBodyState,
 const PxReal dt,
 PxRigidBody& rb)
{
	PX_CHECK_AND_RETURN(!(rb.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC), "PxVehicleWriteRigidBodyStateToPhysXActor: physxActor is kinematic. This is not supported");

	PxRigidDynamic* rd = rb.is<PxRigidDynamic>();
	PxArticulationLink* link = rb.is<PxArticulationLink>();
	PX_ASSERT(rd || link);

	if(rb.getScene() &&  // check for scene to support immediate mode style vehicles
	   ((rd && rd->isSleeping()) || (link && link->getArticulation().isSleeping())))
	{
		// note: sort of a safety mechanism to be able to keep running the full vehicle pipeline
		//       even if the physx actor fell asleep. Without it, the vehicle state can drift from
		//       physx actor state in the course of multiple simulation steps up to the point
		//       where the physx actor suddenly wakes up.
		return;
	}

	switch (updateMode)
	{
	case PxVehiclePhysXActorUpdateMode::eAPPLY_VELOCITY:
	{
		PX_ASSERT(rd);
		rd->setLinearVelocity(rigidBodyState.linearVelocity, false);
		rd->setAngularVelocity(rigidBodyState.angularVelocity, false);
	}
	break;
	case PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION:
	{
		const PxVec3 linAccel = (rigidBodyState.linearVelocity  - rigidBodyState.previousLinearVelocity)/dt;
		const PxVec3 angAccel = (rigidBodyState.angularVelocity - rigidBodyState.previousAngularVelocity)/dt;
		if (rd)
		{
			rd->addForce(linAccel, PxForceMode::eACCELERATION, false);
			rd->addTorque(angAccel, PxForceMode::eACCELERATION, false);
		}
		else
		{
			PX_ASSERT(link);
			link->addForce(linAccel, PxForceMode::eACCELERATION, false);
			link->addTorque(angAccel, PxForceMode::eACCELERATION, false);
		}
	}
	break;
	default:
		break;
	}
}

} //namespace vehicle2
} //namespace physx
