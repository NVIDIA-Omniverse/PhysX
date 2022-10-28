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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "EngineDrivetrain.h"
#include "../base/Base.h"

namespace snippetvehicle2
{

EngineDrivetrainParams EngineDrivetrainParams::transformAndScale(
	const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const
{
	EngineDrivetrainParams r = *this;
	r.autoboxParams = autoboxParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.clutchCommandResponseParams = clutchCommandResponseParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.engineParams = engineParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.gearBoxParams = gearBoxParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.fourWheelDifferentialParams = fourWheelDifferentialParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.multiWheelDifferentialParams = multiWheelDifferentialParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.tankDifferentialParams = tankDifferentialParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.clutchParams = clutchParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	return r;
}


bool EngineDriveVehicle::initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial,
	Enum differentialTye, bool addPhysXBeginEndComponents)
{
	mDifferentialType = differentialTye;

	mTransmissionCommandState.setToDefault();
	mTankDriveTransmissionCommandState.setToDefault();

	if (!PhysXActorVehicle::initialize(physics, params, defaultMaterial))
		return false;

	if (!mEngineDriveParams.isValid(mBaseParams.axleDescription))
		return false;

	//Set the drivetrain state to default.
	mEngineDriveState.setToDefault();

	//Add all the components in sequence that will simulate a vehicle with an engine drive drivetrain.
	initComponentSequence(addPhysXBeginEndComponents);

	return true;
}

void EngineDriveVehicle::destroy()
{
	PhysXActorVehicle::destroy();
}

void EngineDriveVehicle::initComponentSequence(bool addPhysXBeginEndComponents)
{
	//Wake up the associated PxRigidBody if it is asleep and the vehicle commands signal an
	//intent to change state.
	//Read from the physx actor and write the state (position, velocity etc) to the vehicle.
	if(addPhysXBeginEndComponents)
		mComponentSequence.add(static_cast<PxVehiclePhysXActorBeginComponent*>(this));

	//Read the input commands (throttle, brake, steer, clutch etc) and forward them to the drivetrain and steering mechanism.
	//When using automatic transmission, the autobox determines if it wants to begin a gear change. If it does, it will overwrite
	//the target gear command and set throttle to 0 internally.
	mComponentSequence.add(static_cast<PxVehicleEngineDriveCommandResponseComponent*>(this));

	//The differential determines the fraction of available drive torque that will be delivered to each wheel.
	switch (mDifferentialType)
	{
	case eDIFFTYPE_FOURWHEELDRIVE:
		mComponentSequence.add(static_cast<PxVehicleFourWheelDriveDifferentialStateComponent*>(this));
		break;
	case eDIFFTYPE_MULTIWHEELDRIVE:
		mComponentSequence.add(static_cast<PxVehicleMultiWheelDriveDifferentialStateComponent*>(this));
		break;
	case eDIFFTYPE_TANKDRIVE:
		mComponentSequence.add(static_cast<PxVehicleTankDriveDifferentialStateComponent*>(this));
		break;
	default:
		PX_ASSERT(false);
		break;
	}


	//Work out which wheels have a non-zero drive torque and non-zero brake torque.
	//This is used to determine if any tire is to enter the "sticky" regime that will bring the 
	//vehicle to rest.
	mComponentSequence.add(static_cast<PxVehicleEngineDriveActuationStateComponent*>(this));

	//Perform a scene query against the physx scene to determine the plane and friction under each wheel.
	mComponentSequence.add(static_cast<PxVehiclePhysXRoadGeometrySceneQueryComponent*>(this));

	//Start a substep group that can be ticked multiple times per update.
	//Record the handle returned by PxVehicleComponentSequence::beginSubstepGroup() because this 
	//is used later to set the number of substeps for this substep group.
	//In this example, we allow the update of the suspensions, tires and wheels multiple times without recalculating 
	//the plane underneath the wheel.  This is useful for stability at low forward speeds and is much cheaper
	//than setting a smaller timestep for the whole vehicle.
	mComponentSequenceSubstepGroupHandle = mComponentSequence.beginSubstepGroup(3);

		//Update the suspension compression given the plane under each wheel.
		//Update the kinematic compliance from the compression state of each suspension.
		//Convert suspension state to suspension force and torque.
		mComponentSequence.add(static_cast<PxVehicleSuspensionComponent*>(this));

		//Compute the load on the tire, the friction experienced by the tire 
		//and the lateral/longitudinal slip angles.
		//Convert load/friction/slip to tire force and torque.
		//If the vehicle is to come rest then compute the "sticky" velocity constraints to apply to the
		//vehicle.
		mComponentSequence.add(static_cast<PxVehicleTireComponent*>(this));

		//Apply any "sticky" velocity constraints to a data buffer that will be consumed by the physx scene
		//during the next physx scene update.
		mComponentSequence.add(static_cast<PxVehiclePhysXConstraintComponent*>(this));

		//Update the rotational speed of the engine and wheels by applying the available drive torque 
		//to the wheels through the clutch, differential and gears and accounting for the longitudinal
		//tire force that is applied to the wheel's angular momentum.
		mComponentSequence.add(static_cast<PxVehicleEngineDrivetrainComponent*>(this));

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
	if (addPhysXBeginEndComponents)
		mComponentSequence.add(static_cast<PxVehiclePhysXActorEndComponent*>(this));
}

}//namespace snippetvehicle2
