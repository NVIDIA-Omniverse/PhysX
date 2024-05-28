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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/suspension/PxVehicleSuspensionStates.h"

#include "vehicle2/tire/PxVehicleTireStates.h"

#include "vehicle2/wheel/PxVehicleWheelFunctions.h"
#include "vehicle2/wheel/PxVehicleWheelStates.h"
#include "vehicle2/wheel/PxVehicleWheelParams.h"

namespace physx
{
namespace vehicle2
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

} //namespace vehicle2
} //namespace physx
