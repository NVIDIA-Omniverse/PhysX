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

#include "CustomSuspension.h"

namespace snippetvehicle2
{

void addCustomSuspensionForce
(const PxReal dt,
 const PxVehicleSuspensionParams& suspParams,
 const CustomSuspensionParams& customParams,
 const PxVec3& groundNormal, bool isWheelOnGround, const PxVehicleSuspensionComplianceState& suspComplianceState, const PxVehicleRigidBodyState& rigidBodyState,
 PxVehicleSuspensionForce& suspForce, CustomSuspensionState& customState)
{
	//Work out the oscillating force magnitude at time t.
	const PxF32 magnitude = (1.0f + PxCos(customParams.phase + customState.theta))*0.5f*customParams.amplitude;

	//Compute the custom force and torque.
	const PxVec3 suspDir = isWheelOnGround ? groundNormal : PxVec3(PxZero);
	const PxVec3 customForce = suspDir * magnitude;
	const PxVec3 r = rigidBodyState.pose.rotate(suspParams.suspensionAttachment.transform(suspComplianceState.suspForceAppPoint));
	const PxVec3 customTorque = r.cross(customForce);

	//Increment the phase of the oscillator and clamp it in range (-Pi,Pi)
	PxReal theta = customState.theta + 2.0f*PxPi*customParams.frequency*dt;
	if (theta > PxPi)
	{
		theta -= 2.0f*PxPi;
	}
	else if (theta < -PxPi)
	{
		theta += 2.0f*PxPi;
	}
	customState.theta = theta;

	//Add the custom force to the standard suspension force.
	suspForce.force += customForce;
	suspForce.torque += customTorque;
}


bool CustomSuspensionVehicle::initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial, bool addPhysXBeginEndComponents)
{
	if (!DirectDriveVehicle::initialize(physics, params, defaultMaterial, addPhysXBeginEndComponents))
		return false;

	//Set the custom suspension params for all 4 wheels of the vehicle.
	{
		CustomSuspensionParams frontLeft;
		frontLeft.amplitude = 6000.0f*1.25f;
		frontLeft.frequency = 2.0f;
		frontLeft.phase = 0.0f;
		mCustomSuspensionParams[0] = frontLeft;

		CustomSuspensionParams frontRight;
		frontRight.amplitude = 6000.0f*1.25f;
		frontRight.frequency = 2.0f;
		frontRight.phase = 0.0f;
		mCustomSuspensionParams[1] = frontRight;

		CustomSuspensionParams rearLeft;
		rearLeft.amplitude = 6000.0f*1.25f;
		rearLeft.frequency = 2.0f;
		rearLeft.phase = PxPi * 0.5f;
		mCustomSuspensionParams[2] = rearLeft;

		CustomSuspensionParams rearRight;
		rearRight.amplitude = 6000.0f*1.25f;
		rearRight.frequency = 2.0f;
		rearRight.phase = PxPi * 0.5f;
		mCustomSuspensionParams[3] = rearRight;
	}

	//Initialise the custom suspension state.
	mCustomSuspensionStates[0].setToDefault();
	mCustomSuspensionStates[1].setToDefault();
	mCustomSuspensionStates[2].setToDefault();
	mCustomSuspensionStates[3].setToDefault();

	return true;
}

void CustomSuspensionVehicle::destroy()
{
	DirectDriveVehicle::destroy();
}

void CustomSuspensionVehicle::initComponentSequence(const bool addPhysXBeginEndComponents)
{
	//Wake up the associated PxRigidBody if it is asleep and the vehicle commands signal an
	//intent to change state. 
	//Read from the physx actor and write the state (position, velocity etc) to the vehicle.
	if(addPhysXBeginEndComponents)
		mComponentSequence.add(static_cast<PxVehiclePhysXActorBeginComponent*>(this));

	//Read the input commands (throttle, brake etc) and forward them as torques and angles to the wheels on each axle.
	mComponentSequence.add(static_cast<PxVehicleDirectDriveCommandResponseComponent*>(this));

	//Work out which wheels have a non-zero drive torque and non-zero brake torque.
	//This is used to determine if any tire is to enter the "sticky" regime that will bring the 
	//vehicle to rest.
	mComponentSequence.add(static_cast<PxVehicleDirectDriveActuationStateComponent*>(this));

	//Perform a scene query against the physx scene to determine the plane and friction under each wheel.
	mComponentSequence.add(static_cast<PxVehiclePhysXRoadGeometrySceneQueryComponent*>(this));

	//Start a substep group that can be ticked multiple times per update.
	//In this example, we perform 3 updates of the suspensions, tires and wheels without recalculating 
	//the plane underneath the wheel.  This is useful for stability at low forward speeds and is
	//computationally cheaper than simulating the entire sequence.
	mComponentSequenceSubstepGroupHandle = mComponentSequence.beginSubstepGroup(3);

		//Update the suspension compression given the plane under each wheel.
		//Update the kinematic compliance from the compression state of each suspension.
		//Convert suspension state to suspension force and torque.
		//Add an  additional sinusoidal suspension force that will entice the vehicle to 
		//perform a kind of mechanical dance.
		mComponentSequence.add(static_cast<CustomSuspensionComponent*>(this));

		//Compute the load on the tire, the friction experienced by the tire 
		//and the lateral/longitudinal slip angles.
		//Convert load/friction/slip to tire force and torque.
		//If the vehicle is to come rest then compute the "sticky" velocity constraints to apply to the
		//vehicle.
		mComponentSequence.add(static_cast<PxVehicleTireComponent*>(this));

		//Apply any velocity constraints to a data buffer that will be consumed by the physx scene
		//during the next physx scene update.
		mComponentSequence.add(static_cast<PxVehiclePhysXConstraintComponent*>(this));

		//Apply the tire force, brake force and drive force to each wheel and
		//forward integrate the rotation speed of each wheel.
		mComponentSequence.add(static_cast<PxVehicleDirectDrivetrainComponent*>(this));

		//Apply the suspension and tire forces to the vehicle's rigid body and forward 
		//integrate the state of the rigid body.
		mComponentSequence.add(static_cast<PxVehicleRigidBodyComponent*>(this));

	//Mark the end of the substep group.
	mComponentSequence.endSubstepGroup();

	//Update the rotation angle of the wheel by forwarding integrating the rotational
	//speed of each wheel.
	//Compute the local pose of the wheel in the rigid body frame after accounting 
	//suspension compression and compliance.
	mComponentSequence.add(static_cast<PxVehicleWheelComponent*>(this));

	//Write the local poses of each wheel to the corresponding shapes on the physx actor.
	//Write the momentum change applied to the vehicle's rigid body to the physx actor.
	//The physx scene can now try to apply that change to the physx actor.
	//The physx scene will account for collisions and constraints to be applied to the vehicle 
	//that occur by applying the change.
	if(addPhysXBeginEndComponents)
		mComponentSequence.add(static_cast<PxVehiclePhysXActorEndComponent*>(this));
}

}//namespace snippetvehicle2
