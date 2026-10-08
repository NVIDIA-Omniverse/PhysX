// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "PxPhysicsAPI.h"

#include "../snippetvehiclecommon/directdrivetrain/DirectDrivetrain.h"

#include "CustomTire.h"


namespace snippetvehicle
{

using namespace physx;


//
//This class holds the parameters, state and logic needed to implement a vehicle that
//is using a custom component for the tire model.
//
//See BaseVehicle for more details on the snippet code design.
//
class CustomTireVehicle
	: public DirectDriveVehicle
	, public CustomTireComponent
{
public:
	bool initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial, bool addPhysXBeginEndComponents = true);
	virtual void destroy();

	virtual void initComponentSequence(bool addPhysXBeginEndComponents);

	virtual void getDataForCustomTireComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleArrayData<const PxReal>& steerResponseStates,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
		PxVehicleArrayData<const CustomTireParams>& tireParams,
		PxVehicleArrayData<const PxVehicleRoadGeometryState>& roadGeomStates,
		PxVehicleArrayData<const PxVehicleSuspensionState>& suspensionStates,
		PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspensionComplianceStates,
		PxVehicleArrayData<const PxVehicleSuspensionForce>& suspensionForces,
		PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1DStates,
		PxVehicleArrayData<PxVehicleTireGripState>& tireGripStates,
		PxVehicleArrayData<PxVehicleTireDirectionState>& tireDirectionStates,
		PxVehicleArrayData<PxVehicleTireSpeedState>& tireSpeedStates,
		PxVehicleArrayData<PxVehicleTireSlipState>& tireSlipStates,
		PxVehicleArrayData<PxVehicleTireCamberAngleState>& tireCamberAngleStates, 
		PxVehicleArrayData<PxVehicleTireStickyState>& tireStickyStates,
		PxVehicleArrayData<PxVehicleTireForce>& tireForces)
	{
		axleDescription = &mBaseParams.axleDescription;
		steerResponseStates.setData(mBaseState.steerCommandResponseStates);
		rigidBodyState = &mBaseState.rigidBodyState;
		actuationStates.setData(mBaseState.actuationStates);
		wheelParams.setData(mBaseParams.wheelParams);
		suspensionParams.setData(mBaseParams.suspensionParams);
		tireParams.setData(mTireParamsList);
		roadGeomStates.setData(mBaseState.roadGeomStates);
		suspensionStates.setData(mBaseState.suspensionStates);
		suspensionComplianceStates.setData(mBaseState.suspensionComplianceStates);
		suspensionForces.setData(mBaseState.suspensionForces);
		wheelRigidBody1DStates.setData(mBaseState.wheelRigidBody1dStates);
		tireGripStates.setData(mBaseState.tireGripStates);
		tireDirectionStates.setData(mBaseState.tireDirectionStates);
		tireSpeedStates.setData(mBaseState.tireSpeedStates);
		tireSlipStates.setData(mBaseState.tireSlipStates);
		tireCamberAngleStates.setData(mBaseState.tireCamberAngleStates);
		tireStickyStates.setData(mBaseState.tireStickyStates);
		tireForces.setData(mBaseState.tireForces);
	}


	//Parameters and states of the vehicle's custom tire.
	CustomTireParams mCustomTireParams[2];  //One shared parameter set for front and one for rear wheels.
	CustomTireParams* mTireParamsList[4];
};

}//namespace snippetvehicle
