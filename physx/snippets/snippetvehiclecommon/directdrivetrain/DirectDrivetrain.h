// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "vehicle/PxVehicleAPI.h"
#include "../physxintegration/PhysXIntegration.h"

namespace snippetvehicle
{

using namespace physx;

struct DirectDrivetrainParams
{
	PxVehicleDirectDriveThrottleCommandResponseParams directDriveThrottleResponseParams;

	DirectDrivetrainParams transformAndScale(
		const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const;

	PX_FORCE_INLINE bool isValid(const PxVehicleAxleDescription& axleDesc) const
	{
		if (!directDriveThrottleResponseParams.isValid(axleDesc))
			return false;

		return true;
	}
};

struct DirectDrivetrainState
{
	PxReal directDriveThrottleResponseStates[PxVehicleLimits::eMAX_NB_WHEELS];

	PX_FORCE_INLINE void setToDefault()
	{
		PxMemZero(this, sizeof(DirectDrivetrainState));
	}
};


//
//This class holds the parameters, state and logic needed to implement a vehicle that
//is using a direct drivetrain.
//
//See BaseVehicle for more details on the snippet code design.
//
class DirectDriveVehicle
	: public PhysXActorVehicle
	, public PxVehicleDirectDriveCommandResponseComponent
	, public PxVehicleDirectDriveActuationStateComponent
	, public PxVehicleDirectDrivetrainComponent
{
public:
	bool initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial, bool addPhysXBeginEndComponents = true);
	virtual void destroy();

	virtual void initComponentSequence(bool addPhysXBeginEndComponents);

	void getDataForDirectDriveCommandResponseComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams>& brakeResponseParams,
		const PxVehicleDirectDriveThrottleCommandResponseParams*& throttleResponseParams,
		const PxVehicleSteerCommandResponseParams*& steerResponseParams,
		PxVehicleSizedArrayData<const PxVehicleAckermannParams>& ackermannParams,
		const PxVehicleCommandState*& commands, const PxVehicleDirectDriveTransmissionCommandState*& transmissionCommands,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<PxReal>& brakeResponseStates, PxVehicleArrayData<PxReal>& throttleResponseStates, 
		PxVehicleArrayData<PxReal>& steerResponseStates)
	{
		axleDescription = &mBaseParams.axleDescription;
		brakeResponseParams.setDataAndCount(mBaseParams.brakeResponseParams, sizeof(mBaseParams.brakeResponseParams) / sizeof(PxVehicleBrakeCommandResponseParams));
		throttleResponseParams = &mDirectDriveParams.directDriveThrottleResponseParams;
		steerResponseParams = &mBaseParams.steerResponseParams;
		ackermannParams.setDataAndCount(mBaseParams.ackermannParams, sizeof(mBaseParams.ackermannParams)/sizeof(PxVehicleAckermannParams));
		commands = &mCommandState;
		transmissionCommands = &mTransmissionCommandState;
		rigidBodyState = &mBaseState.rigidBodyState;
		brakeResponseStates.setData(mBaseState.brakeCommandResponseStates);
		throttleResponseStates.setData(mDirectDriveState.directDriveThrottleResponseStates);
		steerResponseStates.setData(mBaseState.steerCommandResponseStates);
	}

	virtual void getDataForDirectDriveActuationStateComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleArrayData<const PxReal>& brakeResponseStates,
		PxVehicleArrayData<const PxReal>& throttleResponseStates,
		PxVehicleArrayData<PxVehicleWheelActuationState>& actuationStates)
	{
		axleDescription = &mBaseParams.axleDescription;
		brakeResponseStates.setData(mBaseState.brakeCommandResponseStates);
		throttleResponseStates.setData(mDirectDriveState.directDriveThrottleResponseStates);
		actuationStates.setData(mBaseState.actuationStates);
	}

	virtual void getDataForDirectDrivetrainComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleArrayData<const PxReal>& brakeResponseStates,
		PxVehicleArrayData<const PxReal>& throttleResponseStates,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
		PxVehicleArrayData<const PxVehicleTireForce>& tireForces,
		PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates)
	{
		axleDescription = &mBaseParams.axleDescription;
		brakeResponseStates.setData(mBaseState.brakeCommandResponseStates);
		throttleResponseStates.setData(mDirectDriveState.directDriveThrottleResponseStates);
		wheelParams.setData(mBaseParams.wheelParams);
		actuationStates.setData(mBaseState.actuationStates);
		tireForces.setData(mBaseState.tireForces);
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);
	}


	//Parameters and states of the vehicle's direct drivetrain.
	DirectDrivetrainParams mDirectDriveParams;
	DirectDrivetrainState mDirectDriveState;

	//The commands that will control the vehicle's transmission
	PxVehicleDirectDriveTransmissionCommandState mTransmissionCommandState;
};

}//namespace snippetvehicle
