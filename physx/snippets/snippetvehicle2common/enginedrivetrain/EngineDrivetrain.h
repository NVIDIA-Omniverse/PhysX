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

#pragma once

#include "vehicle2/PxVehicleAPI.h"
#include "../physxintegration/PhysXIntegration.h"

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

struct EngineDrivetrainParams
{
	PxVehicleAutoboxParams autoboxParams;
	PxVehicleClutchCommandResponseParams clutchCommandResponseParams;
	PxVehicleEngineParams engineParams;
	PxVehicleGearboxParams gearBoxParams;
	PxVehicleMultiWheelDriveDifferentialParams multiWheelDifferentialParams;
	PxVehicleFourWheelDriveDifferentialParams fourWheelDifferentialParams;
	PxVehicleTankDriveDifferentialParams tankDifferentialParams;
	PxVehicleClutchParams clutchParams;

	EngineDrivetrainParams transformAndScale(
		const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const;

	PX_FORCE_INLINE bool isValid(const PxVehicleAxleDescription& axleDesc) const
	{
		if (!autoboxParams.isValid(gearBoxParams))
			return false;
		if (!clutchCommandResponseParams.isValid())
			return false;
		if (!engineParams.isValid())
			return false;
		if (!gearBoxParams.isValid())
			return false;
		if (!multiWheelDifferentialParams.isValid(axleDesc))
			return false;
		if (!fourWheelDifferentialParams.isValid(axleDesc))
			return false;
		if (!tankDifferentialParams.isValid(axleDesc))
			return false;
		if (!clutchParams.isValid())
			return false;
		return true;
	}
};

struct EngineDrivetrainState
{
	PxVehicleEngineDriveThrottleCommandResponseState throttleCommandResponseState;
	PxVehicleAutoboxState autoboxState;
	PxVehicleClutchCommandResponseState clutchCommandResponseState;
	PxVehicleDifferentialState differentialState;
	PxVehicleWheelConstraintGroupState wheelConstraintGroupState;
	PxVehicleEngineState engineState;
	PxVehicleGearboxState gearboxState;
	PxVehicleClutchSlipState clutchState;

	PX_FORCE_INLINE void setToDefault()
	{
		throttleCommandResponseState.setToDefault();
		autoboxState.setToDefault();
		clutchCommandResponseState.setToDefault();
		differentialState.setToDefault();
		wheelConstraintGroupState.setToDefault();
		engineState.setToDefault();
		gearboxState.setToDefault();
		clutchState.setToDefault();
	}
};


//
//This class holds the parameters, state and logic needed to implement a vehicle that
//is using an engine drivetrain with gears, clutch etc.
//
//See BaseVehicle for more details on the snippet code design.
//
class EngineDriveVehicle
	: public PhysXActorVehicle
	, public PxVehicleEngineDriveCommandResponseComponent
	, public PxVehicleFourWheelDriveDifferentialStateComponent
	, public PxVehicleMultiWheelDriveDifferentialStateComponent
	, public PxVehicleTankDriveDifferentialStateComponent
	, public PxVehicleEngineDriveActuationStateComponent
	, public PxVehicleEngineDrivetrainComponent
{
public:

	enum Enum
	{
		eDIFFTYPE_FOURWHEELDRIVE,
		eDIFFTYPE_MULTIWHEELDRIVE,
		eDIFFTYPE_TANKDRIVE
	};

	bool initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial,
		Enum differentialType, bool addPhysXBeginEndComponents=true);
	virtual void destroy();

	virtual void initComponentSequence(bool addPhysXBeginEndComponents);

	virtual void getDataForPhysXActorBeginComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleCommandState*& commands,
		const PxVehicleEngineDriveTransmissionCommandState*& transmissionCommands,
		const PxVehicleGearboxParams*& gearParams,
		const PxVehicleGearboxState*& gearState,
		const PxVehicleEngineParams*& engineParams,
		PxVehiclePhysXActor*& physxActor,
		PxVehiclePhysXSteerState*& physxSteerState,
		PxVehiclePhysXConstraints*& physxConstraints,
		PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
		PxVehicleEngineState*& engineState)
	{
		axleDescription = &mBaseParams.axleDescription;
		commands = &mCommandState;
		physxActor = &mPhysXState.physxActor;
		physxSteerState = &mPhysXState.physxSteerState;
		physxConstraints = &mPhysXState.physxConstraints;
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);

		transmissionCommands = &mTransmissionCommandState;
		gearParams = &mEngineDriveParams.gearBoxParams;
		gearState =  &mEngineDriveState.gearboxState;
		engineParams = &mEngineDriveParams.engineParams;
		engineState =  &mEngineDriveState.engineState;
	}

	virtual void getDataForPhysXActorEndComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxTransform>& wheelShapeLocalPoses,
		PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
		PxVehicleArrayData<const PxVehicleWheelLocalPose>& wheelLocalPoses,
		const PxVehicleGearboxState*& gearState,
		PxVehiclePhysXActor*& physxActor)
	{
		axleDescription = &mBaseParams.axleDescription;
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelParams.setData(mBaseParams.wheelParams);
		wheelShapeLocalPoses.setData(mPhysXParams.physxWheelShapeLocalPoses);
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);
		wheelLocalPoses.setData(mBaseState.wheelLocalPoses);
		physxActor = &mPhysXState.physxActor;

		gearState = &mEngineDriveState.gearboxState;
	}

	virtual void getDataForEngineDriveCommandResponseComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams>& brakeResponseParams,
		const PxVehicleSteerCommandResponseParams*& steerResponseParams,
		PxVehicleSizedArrayData<const PxVehicleAckermannParams>& ackermannParams,
		const PxVehicleGearboxParams*& gearboxParams,
		const PxVehicleClutchCommandResponseParams*& clutchResponseParams,
		const PxVehicleEngineParams*& engineParams,
		const PxVehicleRigidBodyState*& rigidBodyState,
		const PxVehicleEngineState*& engineState,
		const PxVehicleAutoboxParams*& autoboxParams,
		const PxVehicleCommandState*& commands,
		const PxVehicleEngineDriveTransmissionCommandState*& transmissionCommands,
		PxVehicleArrayData<PxReal>& brakeResponseStates,
		PxVehicleEngineDriveThrottleCommandResponseState*& throttleResponseState,
		PxVehicleArrayData<PxReal>& steerResponseStates,
		PxVehicleGearboxState*& gearboxResponseState,
		PxVehicleClutchCommandResponseState*& clutchResponseState,
		PxVehicleAutoboxState*& autoboxState)
	{
		axleDescription = &mBaseParams.axleDescription;
		brakeResponseParams.setDataAndCount(mBaseParams.brakeResponseParams, sizeof(mBaseParams.brakeResponseParams) / sizeof(PxVehicleBrakeCommandResponseParams));
		steerResponseParams = &mBaseParams.steerResponseParams;
		ackermannParams.setDataAndCount(mBaseParams.ackermannParams, sizeof(mBaseParams.ackermannParams)/sizeof(PxVehicleAckermannParams));
		gearboxParams = &mEngineDriveParams.gearBoxParams;
		clutchResponseParams = &mEngineDriveParams.clutchCommandResponseParams;
		engineParams = &mEngineDriveParams.engineParams;
		rigidBodyState = &mBaseState.rigidBodyState;
		engineState = &mEngineDriveState.engineState;
		autoboxParams = &mEngineDriveParams.autoboxParams;
		commands = &mCommandState;
		transmissionCommands = (Enum::eDIFFTYPE_TANKDRIVE == mDifferentialType) ? &mTankDriveTransmissionCommandState : &mTransmissionCommandState;
		brakeResponseStates.setData(mBaseState.brakeCommandResponseStates);
		throttleResponseState = &mEngineDriveState.throttleCommandResponseState;
		steerResponseStates.setData(mBaseState.steerCommandResponseStates);
		gearboxResponseState = &mEngineDriveState.gearboxState;
		clutchResponseState = &mEngineDriveState.clutchCommandResponseState;
		autoboxState = &mEngineDriveState.autoboxState;
	}

	virtual void getDataForFourWheelDriveDifferentialStateComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleFourWheelDriveDifferentialParams*& differentialParams,
		PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidbody1dStates,
		PxVehicleDifferentialState*& differentialState, PxVehicleWheelConstraintGroupState*& wheelConstraintGroups)
	{
		axleDescription = &mBaseParams.axleDescription;
		differentialParams = &mEngineDriveParams.fourWheelDifferentialParams;
		wheelRigidbody1dStates.setData(mBaseState.wheelRigidBody1dStates);
		differentialState = &mEngineDriveState.differentialState;
		wheelConstraintGroups = &mEngineDriveState.wheelConstraintGroupState;
	}

	virtual void getDataForMultiWheelDriveDifferentialStateComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleMultiWheelDriveDifferentialParams*& differentialParams,
		PxVehicleDifferentialState*& differentialState)
	{
		axleDescription = &mBaseParams.axleDescription;
		differentialParams = &mEngineDriveParams.multiWheelDifferentialParams;
		differentialState = &mEngineDriveState.differentialState;
	}

	virtual void getDataForTankDriveDifferentialStateComponent(
		const PxVehicleAxleDescription *&axleDescription,
		const PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionCommands,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		const PxVehicleTankDriveDifferentialParams *& differentialParams,
		PxVehicleDifferentialState *& differentialState,
		PxVehicleWheelConstraintGroupState*& wheelConstraintGroups)
	{
		axleDescription = &mBaseParams.axleDescription;
		tankDriveTransmissionCommands = &mTankDriveTransmissionCommandState;
		wheelParams.setData(mBaseParams.wheelParams);
		differentialParams = &mEngineDriveParams.tankDifferentialParams;
		differentialState = &mEngineDriveState.differentialState;
		wheelConstraintGroups = &mEngineDriveState.wheelConstraintGroupState;
	}

	virtual void getDataForEngineDriveActuationStateComponent(
		const PxVehicleAxleDescription*& axleDescription, 
		const PxVehicleGearboxParams*& gearboxParams,
		PxVehicleArrayData<const PxReal>& brakeResponseStates,
		const PxVehicleEngineDriveThrottleCommandResponseState*& throttleResponseState,
		const PxVehicleGearboxState*& gearboxState,
		const PxVehicleDifferentialState*& differentialState,
		const PxVehicleClutchCommandResponseState*& clutchResponseState,
		PxVehicleArrayData<PxVehicleWheelActuationState>& actuationStates)
	{
		axleDescription = &mBaseParams.axleDescription;
		gearboxParams = &mEngineDriveParams.gearBoxParams;
		brakeResponseStates.setData(mBaseState.brakeCommandResponseStates);
		throttleResponseState = &mEngineDriveState.throttleCommandResponseState;
		gearboxState = &mEngineDriveState.gearboxState;
		differentialState = &mEngineDriveState.differentialState;
		clutchResponseState = &mEngineDriveState.clutchCommandResponseState;
		actuationStates.setData(mBaseState.actuationStates);
	}

	virtual void getDataForEngineDrivetrainComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		const PxVehicleEngineParams*& engineParams,
		const PxVehicleClutchParams*& clutchParams,
		const PxVehicleGearboxParams*& gearboxParams, 
		PxVehicleArrayData<const PxReal>& brakeResponseStates,
		PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
		PxVehicleArrayData<const PxVehicleTireForce>& tireForces,
		const PxVehicleEngineDriveThrottleCommandResponseState*& throttleResponseState,
		const PxVehicleClutchCommandResponseState*& clutchResponseState,
		const PxVehicleDifferentialState*& differentialState,
		const PxVehicleWheelConstraintGroupState*& constraintGroupState,
		PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
		PxVehicleEngineState*& engineState,
		PxVehicleGearboxState*& gearboxState,
		PxVehicleClutchSlipState*& clutchState)
	{
		axleDescription = &mBaseParams.axleDescription;
		wheelParams.setData(mBaseParams.wheelParams);
		engineParams = &mEngineDriveParams.engineParams;
		clutchParams = &mEngineDriveParams.clutchParams;
		gearboxParams = &mEngineDriveParams.gearBoxParams;
		brakeResponseStates.setData(mBaseState.brakeCommandResponseStates);
		actuationStates.setData(mBaseState.actuationStates);
		tireForces.setData(mBaseState.tireForces);
		throttleResponseState = &mEngineDriveState.throttleCommandResponseState;
		clutchResponseState = &mEngineDriveState.clutchCommandResponseState;
		differentialState = &mEngineDriveState.differentialState;
		constraintGroupState = Enum::eDIFFTYPE_TANKDRIVE == mDifferentialType ? &mEngineDriveState.wheelConstraintGroupState : NULL;
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);
		engineState = &mEngineDriveState.engineState;
		gearboxState = &mEngineDriveState.gearboxState;
		clutchState = &mEngineDriveState.clutchState;
	}


	//Parameters and states of the vehicle's engine drivetrain.
	EngineDrivetrainParams mEngineDriveParams;
	EngineDrivetrainState mEngineDriveState;

	//The commands that will control the vehicle's transmission
	PxVehicleEngineDriveTransmissionCommandState mTransmissionCommandState;
	PxVehicleTankDriveTransmissionCommandState mTankDriveTransmissionCommandState;

	//The type of differential that will be used.
	//If eDIFFTYPE_TANKDRIVE is chosen then the vehicle's transmission
	//commands are stored in mTankDriveTransmissionCommandState.  
	//If eDIFFTYPE_FOURWHEELDRIVE or eDIFFTYPE_MULTIWHEELDRIVE is chosen
	//then the vehicle's transmission commands are stored in 
	//mTransmissionCommandState
	Enum mDifferentialType;
};

}//namespace snippetvehicle2
