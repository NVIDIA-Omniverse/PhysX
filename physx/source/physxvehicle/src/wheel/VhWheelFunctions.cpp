// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "vehicle/PxVehicleParams.h"

#include "vehicle/suspension/PxVehicleSuspensionStates.h"

#include "vehicle/tire/PxVehicleTireStates.h"

#include "vehicle/wheel/PxVehicleWheelFunctions.h"
#include "vehicle/wheel/PxVehicleWheelStates.h"
#include "vehicle/wheel/PxVehicleWheelParams.h"

namespace physx
{

void PxVehicleWheelRotationAngleUpdate
(const PxVehicleWheelParams& whlParams,
 const PxVehicleWheelActuationState& actState, const PxVehicleSuspensionState& suspState, const PxVehicleTireSpeedState& trSpeedState,
 const PxReal thresholdForwardSpeedForWheelAngleIntegration, const PxReal dt,
 PxVehicleWheelRigidBody1dState& whlRigidBody1dState)
{
	//At low vehicle forward speeds we have some numerical difficulties getting the 
	//wheel rotation speeds to be correct due to the tire model's difficulties at low vz.
	//The solution is to blend between the rolling speed at the wheel and the wheel's actual rotation speed.
	//If the wheel is 
	//(i)   in the air or, 
	//(ii)  under braking torque or, 
	//(iii) driven by a drive torque
	//then always use the wheel's actual rotation speed.
	//Just to be clear, this means we will blend when the wheel
	//(i)   is on the ground and
	//(ii)  has no brake applied and
	//(iii) has no drive torque and
	//(iv)  is at low forward speed
	const PxF32 jounce = suspState.jounce;
	const bool isBrakeApplied = actState.isBrakeApplied;
	const bool isDriveApplied = actState.isDriveApplied;
	const PxF32 lngSpeed = trSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL];
	const PxF32 absLngSpeed = PxAbs(lngSpeed);
	PxF32 wheelOmega = whlRigidBody1dState.rotationSpeed;
	if (jounce > 0 &&																	//(i)   wheel touching ground
		!isBrakeApplied &&																//(ii)  no brake applied
		!isDriveApplied &&																//(iii) no drive torque applied
		(absLngSpeed < thresholdForwardSpeedForWheelAngleIntegration))					//(iv)  low speed
	{
		const PxF32 wheelRadius = whlParams.radius;
		const PxF32 alpha = absLngSpeed / thresholdForwardSpeedForWheelAngleIntegration;
		wheelOmega = (lngSpeed/wheelRadius)*(1.0f - alpha) + wheelOmega * alpha;
	}

	whlRigidBody1dState.correctedRotationSpeed = wheelOmega;

	//Integrate angle.
	PxF32 newRotAngle = whlRigidBody1dState.rotationAngle + wheelOmega * dt;

	//Clamp in range (-2*Pi,2*Pi)
	newRotAngle = newRotAngle - (PxI32(newRotAngle / PxTwoPi) * PxTwoPi);

	//Set the angle.
	whlRigidBody1dState.rotationAngle = newRotAngle;
}

} //namespace physx
