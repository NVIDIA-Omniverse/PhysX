// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "vehicle/pvd/PxVehiclePvdHelpers.h"
#include "VhPvdWriter.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxVehiclePvdAttributeHandles
{
#if PX_SUPPORT_OMNI_PVD

	/////////////////////
	//RIGID BODY
	/////////////////////////
	
	RigidBodyParams rigidBodyParams;
	RigidBodyState rigidBodyState;

	/////////////////////////
	//SUSP STATE CALC PARAMS
	/////////////////////////

	SuspStateCalcParams suspStateCalcParams;

	/////////////////////////
	//CONTROL ATTRIBUTES
	/////////////////////////

	WheelResponseParams brakeCommandResponseParams;
	WheelResponseParams steerCommandResponseParams;
	AckermannParams ackermannParams;
	WheelResponseStates brakeCommandResponseStates;
	WheelResponseStates steerCommandResponseStates;

	/////////////////////////////////
	//WHEEL ATTACHMENT ATTRIBUTES
	/////////////////////////////////

	WheelParams wheelParams;
	WheelActuationState wheelActuationState;
	WheelRigidBody1dState wheelRigidBody1dState;
	WheelLocalPoseState wheelLocalPoseState;
	RoadGeometryState roadGeomState;
	SuspParams suspParams;
	SuspCompParams suspCompParams;
	SuspForceParams suspForceParams;
	SuspState suspState;
	SuspCompState suspCompState;
	SuspForce suspForce;
	TireParams tireParams;
	TireDirectionState tireDirectionState;
	TireSpeedState tireSpeedState;
	TireSlipState tireSlipState;
	TireStickyState tireStickyState;
	TireGripState tireGripState;
	TireCamberState tireCamberState;
	TireForce tireForce;
	WheelAttachment wheelAttachment;
	
	///////////////////////
	//ANTIROLL BARS
	///////////////////////

	AntiRollParams antiRollParams;
	AntiRollForce antiRollForce;

	///////////////////////////////////
	//DIRECT DRIVETRAIN
	///////////////////////////////////
	
	DirectDriveCommandState directDriveCommandState;
	DirectDriveTransmissionCommandState directDriveTransmissionCommandState;
	WheelResponseParams directDriveThrottleCommandResponseParams;
	DirectDriveThrottleResponseState directDriveThrottleCommandResponseState;
	DirectDrivetrain directDrivetrain;

	//////////////////////////////////
	//ENGINE DRIVETRAIN ATTRIBUTES
	//////////////////////////////////
	
	EngineDriveCommandState engineDriveCommandState;
	EngineDriveTransmissionCommandState engineDriveTransmissionCommandState;
	TankDriveTransmissionCommandState tankDriveTransmissionCommandState;
	ClutchResponseParams clutchCommandResponseParams;
	ClutchParams clutchParams;
	EngineParams engineParams;
	GearboxParams gearboxParams;
	AutoboxParams autoboxParams;
	MultiWheelDiffParams multiwheelDiffParams;
	FourWheelDiffParams fourwheelDiffParams;
	TankDiffParams tankDiffParams;
	ClutchResponseState clutchResponseState;
	ThrottleResponseState throttleResponseState;
	EngineState engineState;
	GearboxState gearboxState;
	AutoboxState autoboxState;
	DiffState diffState;
	ClutchSlipState clutchSlipState;
	EngineDrivetrain engineDrivetrain;

	//////////////////////////////////////
	//PHYSX WHEEL ATTACHMENT INTEGRATION
	//////////////////////////////////////

	PhysXSuspensionLimitConstraintParams physxSuspLimitConstraintParams;
	PhysXWheelShape physxWheelShape;
	PhysXRoadGeomState physxRoadGeomState;
	PhysXConstraintState physxConstraintState;
	PhysXMaterialFriction physxMaterialFriction;
	PhysXWheelAttachment physxWheelAttachment;

	////////////////////
	//PHYSX RIGID ACTOR
	////////////////////
	
	PhysXRoadGeometryQueryParams physxRoadGeometryQueryParams;
	PhysXRigidActor physxRigidActor;
	PhysXSteerState physxSteerState;

	//////////////////////////////////
	//VEHICLE ATTRIBUTES
	//////////////////////////////////

	Vehicle vehicle;

#endif

};

#if !PX_DOXYGEN
} // namespace physx
#endif

