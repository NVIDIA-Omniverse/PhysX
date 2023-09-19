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

#pragma once

#include "vehicle2/pvd/PxVehiclePvdHelpers.h"
#include "VhPvdWriter.h"

/** \addtogroup vehicle2
  @{
*/

#if !PX_DOXYGEN
namespace physx
{
namespace vehicle2
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
} // namespace vehicle2
} // namespace physx
#endif

/** @} */
