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

#include "vehicle2/braking/PxVehicleBrakingParams.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainParams.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainStates.h"
#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintParams.h"
#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintStates.h"
#include "vehicle2/physxRoadGeometry/PxVehiclePhysXRoadGeometryParams.h"
#include "vehicle2/physxRoadGeometry/PxVehiclePhysXRoadGeometryState.h"
#include "vehicle2/rigidBody/PxVehicleRigidBodyParams.h"
#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"
#include "vehicle2/steering/PxVehicleSteeringParams.h"
#include "vehicle2/suspension/PxVehicleSuspensionParams.h"
#include "vehicle2/suspension/PxVehicleSuspensionStates.h"
#include "vehicle2/tire/PxVehicleTireParams.h"
#include "vehicle2/tire/PxVehicleTireStates.h"
#include "vehicle2/wheel/PxVehicleWheelParams.h"
#include "vehicle2/wheel/PxVehicleWheelStates.h"

#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdWriter.h"
#endif

/** \addtogroup vehicle2
  @{
*/

#if PX_SUPPORT_OMNI_PVD

#if !PX_DOXYGEN
namespace physx
{
namespace vehicle2
{
#endif

///////////////////////////////
//RIGID BODY
///////////////////////////////

struct RigidBodyParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle moiAH;
	OmniPvdAttributeHandle massAH;
};

struct RigidBodyState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle posAH;				
	OmniPvdAttributeHandle quatAH;				
	OmniPvdAttributeHandle linearVelocityAH;			
	OmniPvdAttributeHandle angularVelocityAH;			
	OmniPvdAttributeHandle previousLinearVelocityAH;	
	OmniPvdAttributeHandle previousAngularVelocityAH;	
	OmniPvdAttributeHandle externalForceAH;			
	OmniPvdAttributeHandle externalTorqueAH;	
};

RigidBodyParams registerRigidBodyParams(OmniPvdWriter& omniWriter);
RigidBodyState registerRigidBodyState(OmniPvdWriter& omniWriter);

void writeRigidBodyParams
(const PxVehicleRigidBodyParams& rbodyParams,
 const OmniPvdObjectHandle oh, const RigidBodyParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeRigidBodyState
(const PxVehicleRigidBodyState& rbodyParams,
 const OmniPvdObjectHandle oh, const RigidBodyState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);


/////////////////////////////////
//CONTROL
/////////////////////////////////

struct WheelResponseParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle responseMultipliers0To3AH;
	OmniPvdAttributeHandle responseMultipliers4To7AH;
	OmniPvdAttributeHandle responseMultipliers8To11AH;
	OmniPvdAttributeHandle responseMultipliers12To15AH;
	OmniPvdAttributeHandle responseMultipliers16To19AH;
	OmniPvdAttributeHandle maxResponseAH;
};

struct WheelResponseStates
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle responseStates0To3AH;
	OmniPvdAttributeHandle responseStates4To7AH;
	OmniPvdAttributeHandle responseStates8To11AH;
	OmniPvdAttributeHandle responseStates12To15AH;
	OmniPvdAttributeHandle responseStates16To19AH;
};

struct AckermannParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle wheelIdsAH;
	OmniPvdAttributeHandle wheelBaseAH;
	OmniPvdAttributeHandle trackWidthAH;
	OmniPvdAttributeHandle strengthAH;
};



WheelResponseParams registerSteerResponseParams(OmniPvdWriter& omniWriter);
WheelResponseParams registerBrakeResponseParams(OmniPvdWriter& omniWriter);
WheelResponseStates registerSteerResponseStates(OmniPvdWriter& omniWriter);
WheelResponseStates registerBrakeResponseStates(OmniPvdWriter& omniWriter);
AckermannParams registerAckermannParams(OmniPvdWriter&);

void writeSteerResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleSteerCommandResponseParams& steerResponseParams, 
 const OmniPvdObjectHandle oh, const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeBrakeResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleBrakeCommandResponseParams& brakeResponseParams, 
 const OmniPvdObjectHandle oh, const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSteerResponseStates
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<PxReal>& steerResponseStates, 
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeBrakeResponseStates
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeAckermannParams
(const PxVehicleAckermannParams&,
 const OmniPvdObjectHandle, const AckermannParams&, OmniPvdWriter&, OmniPvdContextHandle);

/////////////////////////////////////////////
//WHEEL ATTACHMENTS
/////////////////////////////////////////////

struct WheelParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle wheelRadiusAH;
	OmniPvdAttributeHandle halfWidthAH;
	OmniPvdAttributeHandle massAH;
	OmniPvdAttributeHandle moiAH;
	OmniPvdAttributeHandle dampingRateAH;
};

struct WheelActuationState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle isBrakeAppliedAH;
	OmniPvdAttributeHandle isDriveAppliedAH;
};

struct WheelRigidBody1dState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle rotationSpeedAH;
	OmniPvdAttributeHandle correctedRotationSpeedAH;
	OmniPvdAttributeHandle rotationAngleAH;
};

struct WheelLocalPoseState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle posAH;
	OmniPvdAttributeHandle quatAH;
};

struct RoadGeometryState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle planeAH;
	OmniPvdAttributeHandle frictionAH;
	OmniPvdAttributeHandle velocityAH;
	OmniPvdAttributeHandle hitStateAH;
};

struct SuspParams
{
	OmniPvdClassHandle CH;

	OmniPvdAttributeHandle suspAttachmentPosAH;
	OmniPvdAttributeHandle suspAttachmentQuatAH;
	OmniPvdAttributeHandle suspDirAH;
	OmniPvdAttributeHandle suspTravleDistAH;
	OmniPvdAttributeHandle wheelAttachmentPosAH;
	OmniPvdAttributeHandle wheelAttachmentQuatAH;
};

struct SuspCompParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle toeAngleAH;
	OmniPvdAttributeHandle camberAngleAH;
	OmniPvdAttributeHandle suspForceAppPointAH;
	OmniPvdAttributeHandle tireForceAppPointAH;
};

struct SuspForceParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle stiffnessAH;
	OmniPvdAttributeHandle dampingAH;
	OmniPvdAttributeHandle sprungMassAH;
};

struct SuspState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle jounceAH;
	OmniPvdAttributeHandle jounceSpeedAH;
	OmniPvdAttributeHandle separationAH;
};

struct SuspCompState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle toeAH;
	OmniPvdAttributeHandle camberAH;
	OmniPvdAttributeHandle tireForceAppPointAH;
	OmniPvdAttributeHandle suspForceAppPointAH;
};

struct SuspForce
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle forceAH;
	OmniPvdAttributeHandle torqueAH;
	OmniPvdAttributeHandle normalForceAH;
};

struct TireParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle latStiffXAH;
	OmniPvdAttributeHandle latStiffYAH;
	OmniPvdAttributeHandle longStiffAH;
	OmniPvdAttributeHandle camberStiffAH;
	OmniPvdAttributeHandle frictionVsSlipAH;
	OmniPvdAttributeHandle restLoadAH;
	OmniPvdAttributeHandle loadFilterAH;
};

struct TireDirectionState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle lngDirectionAH;
	OmniPvdAttributeHandle latDirectionAH;
};

struct TireSpeedState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle lngSpeedAH;
	OmniPvdAttributeHandle latSpeedAH;

};

struct TireSlipState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle lngSlipAH;
	OmniPvdAttributeHandle latSlipAH;
};

struct TireGripState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle loadAH;
	OmniPvdAttributeHandle frictionAH;
};

struct TireCamberState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle camberAngleAH;
};

struct TireStickyState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle lngStickyStateTimer;
	OmniPvdAttributeHandle lngStickyStateStatus;
	OmniPvdAttributeHandle latStickyStateTimer;
	OmniPvdAttributeHandle latStickyStateStatus;
};

struct TireForce
{
	OmniPvdClassHandle CH;

	OmniPvdAttributeHandle lngForceAH;
	OmniPvdAttributeHandle lngTorqueAH;

	OmniPvdAttributeHandle latForceAH;
	OmniPvdAttributeHandle latTorqueAH;

	OmniPvdAttributeHandle aligningMomentAH;
	OmniPvdAttributeHandle wheelTorqueAH;
};

struct WheelAttachment
{
	OmniPvdClassHandle CH;

	OmniPvdAttributeHandle wheelParamsAH;
	OmniPvdAttributeHandle wheelActuationStateAH;
	OmniPvdAttributeHandle wheelRigidBody1dStateAH;
	OmniPvdAttributeHandle wheelLocalPoseStateAH;

	OmniPvdAttributeHandle roadGeomStateAH;

	OmniPvdAttributeHandle suspParamsAH;
	OmniPvdAttributeHandle suspCompParamsAH;
	OmniPvdAttributeHandle suspForceParamsAH;
	OmniPvdAttributeHandle suspStateAH;
	OmniPvdAttributeHandle suspCompStateAH;
	OmniPvdAttributeHandle suspForceAH;

	OmniPvdAttributeHandle tireParamsAH;
	OmniPvdAttributeHandle tireDirectionStateAH;
	OmniPvdAttributeHandle tireSpeedStateAH;
	OmniPvdAttributeHandle tireSlipStateAH;
	OmniPvdAttributeHandle tireStickyStateAH;
	OmniPvdAttributeHandle tireGripStateAH;
	OmniPvdAttributeHandle tireCamberStateAH;
	OmniPvdAttributeHandle tireForceAH;
};

WheelParams registerWheelParams(OmniPvdWriter& omniWriter);
WheelActuationState registerWheelActuationState(OmniPvdWriter& omniWriter);
WheelRigidBody1dState registerWheelRigidBody1dState(OmniPvdWriter& omniWriter);
WheelLocalPoseState registerWheelLocalPoseState(OmniPvdWriter& omniWriter);
RoadGeometryState registerRoadGeomState(OmniPvdWriter& omniWriter);
SuspParams registerSuspParams(OmniPvdWriter& omniWriter);
SuspCompParams registerSuspComplianceParams(OmniPvdWriter& omniWriter);
SuspForceParams registerSuspForceParams(OmniPvdWriter& omniWriter);
SuspState registerSuspState(OmniPvdWriter& omniWriter);
SuspCompState registerSuspComplianceState(OmniPvdWriter& omniWriter);
SuspForce registerSuspForce(OmniPvdWriter& omniWriter);
TireParams registerTireParams(OmniPvdWriter& omniWriter);
TireDirectionState registerTireDirectionState(OmniPvdWriter& omniWriter);
TireSpeedState registerTireSpeedState(OmniPvdWriter& omniWriter);
TireSlipState registerTireSlipState(OmniPvdWriter& omniWriter);
TireStickyState registerTireStickyState(OmniPvdWriter& omniWriter);
TireGripState registerTireGripState(OmniPvdWriter& omniWriter);
TireCamberState registerTireCamberState(OmniPvdWriter& omniWriter);
TireForce registerTireForce(OmniPvdWriter& omniWriter);
WheelAttachment registerWheelAttachment(OmniPvdWriter& omniWriter);

void writeWheelParams
(const PxVehicleWheelParams& params, 
 const OmniPvdObjectHandle oh, const WheelParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeWheelActuationState
(const PxVehicleWheelActuationState& actState,
 const OmniPvdObjectHandle oh, const WheelActuationState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeWheelRigidBody1dState
(const PxVehicleWheelRigidBody1dState& rigidBodyState,
 const OmniPvdObjectHandle oh, const WheelRigidBody1dState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeWheelLocalPoseState
(const PxVehicleWheelLocalPose& pose,
 const OmniPvdObjectHandle oh, const WheelLocalPoseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeRoadGeomState
(const PxVehicleRoadGeometryState& roadGeometryState,
 const OmniPvdObjectHandle oh, const RoadGeometryState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSuspParams
(const PxVehicleSuspensionParams& suspParams,
 const OmniPvdObjectHandle oh, const SuspParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSuspComplianceParams
(const PxVehicleSuspensionComplianceParams& compParams,
 const OmniPvdObjectHandle oh, const SuspCompParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSuspForceParams
(const PxVehicleSuspensionForceParams& forceParams, 
 const OmniPvdObjectHandle oh, const SuspForceParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSuspState
(const PxVehicleSuspensionState& suspState, 
 const OmniPvdObjectHandle oh, const SuspState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSuspComplianceState
(const PxVehicleSuspensionComplianceState& suspCompState,
 const OmniPvdObjectHandle oh, const SuspCompState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeSuspForce
(const PxVehicleSuspensionForce& suspForce, 
 const OmniPvdObjectHandle oh, const SuspForce& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireParams
(const PxVehicleTireForceParams& tireParams, 
 const OmniPvdObjectHandle oh, const TireParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireDirectionState
(const PxVehicleTireDirectionState& tireDirState,
 const OmniPvdObjectHandle oh, const TireDirectionState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireSpeedState
(const PxVehicleTireSpeedState& tireSpeedState,
 const OmniPvdObjectHandle oh, const TireSpeedState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireSlipState
(const PxVehicleTireSlipState& tireSlipState,
 const OmniPvdObjectHandle oh, const TireSlipState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireStickyState
(const PxVehicleTireStickyState& tireStickyState, 
 const OmniPvdObjectHandle oh, const TireStickyState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireGripState
(const PxVehicleTireGripState& tireGripState, 
 const OmniPvdObjectHandle oh, const TireGripState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireCamberState
(const PxVehicleTireCamberAngleState& tireCamberState,
 const OmniPvdObjectHandle oh, const TireCamberState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTireForce
(const PxVehicleTireForce& tireForce,
 const OmniPvdObjectHandle oh, const TireForce& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

/////////////////////////////////////////
//ANTIROLL
/////////////////////////////////////////

struct AntiRollParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle wheel0AH;
	OmniPvdAttributeHandle wheel1AH;
	OmniPvdAttributeHandle stiffnessAH;
};

struct AntiRollForce
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle torqueAH;
};

AntiRollParams registerAntiRollParams(OmniPvdWriter& omniWriter);
AntiRollForce registerAntiRollForce(OmniPvdWriter& omniWriter);

void writeAntiRollParams
(const PxVehicleAntiRollForceParams& antiRollParams,
 const OmniPvdObjectHandle oh, const AntiRollParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeAntiRollForce
(const PxVehicleAntiRollTorque& antiRollForce, 
 const OmniPvdObjectHandle oh, const AntiRollForce& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

////////////////////////////////////////
//SUSPENSION STATE CALCULATION
////////////////////////////////////////

struct SuspStateCalcParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle calcTypeAH;
	OmniPvdAttributeHandle limitExpansionValAH;
};

SuspStateCalcParams registerSuspStateCalcParams(OmniPvdWriter& omniWriter);

void writeSuspStateCalcParams
(const PxVehicleSuspensionStateCalculationParams& suspStateCalcParams,
 const OmniPvdObjectHandle oh, const SuspStateCalcParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

/////////////////////////////////////////
//DIRECT DRIVETRAIN
////////////////////////////////////////

struct DirectDriveCommandState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle brakesAH;
	OmniPvdAttributeHandle throttleAH;
	OmniPvdAttributeHandle steerAH;
};

struct DirectDriveTransmissionCommandState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle gearAH;
};

struct DirectDriveThrottleResponseState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle states0To3AH;
	OmniPvdAttributeHandle states4To7AH;
	OmniPvdAttributeHandle states8To11AH;
	OmniPvdAttributeHandle states12To15AH;
	OmniPvdAttributeHandle states16To19AH;
};

struct DirectDrivetrain
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle throttleResponseParamsAH;
	OmniPvdAttributeHandle commandStateAH;
	OmniPvdAttributeHandle transmissionCommandStateAH;
	OmniPvdAttributeHandle throttleResponseStateAH;
};

WheelResponseParams registerDirectDriveThrottleResponseParams(OmniPvdWriter& omniWriter);
DirectDriveCommandState registerDirectDriveCommandState(OmniPvdWriter& omniWriter);
DirectDriveTransmissionCommandState registerDirectDriveTransmissionCommandState(OmniPvdWriter& omniWriter);
DirectDriveThrottleResponseState registerDirectDriveThrottleResponseState(OmniPvdWriter& omniWriter);
DirectDrivetrain registerDirectDrivetrain(OmniPvdWriter& omniWriter);

void writeDirectDriveThrottleResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleDirectDriveThrottleCommandResponseParams& directDriveThrottleResponseParams,
 const OmniPvdObjectHandle oh,  const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeDirectDriveCommandState
(const PxVehicleCommandState& commands,
 const OmniPvdObjectHandle oh, const DirectDriveCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeDirectDriveTransmissionCommandState
(const PxVehicleDirectDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const DirectDriveTransmissionCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeDirectDriveThrottleResponseState
(const PxVehicleAxleDescription& axleDesc, const PxVehicleArrayData<PxReal>& throttleResponseState,
 const OmniPvdObjectHandle oh, const DirectDriveThrottleResponseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);


/////////////////////////////////////////
//ENGINE DRIVETRAIN
////////////////////////////////////////

struct EngineDriveCommandState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle brakesAH;
	OmniPvdAttributeHandle throttleAH;
	OmniPvdAttributeHandle steerAH;
};

struct EngineDriveTransmissionCommandState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle gearAH;
	OmniPvdAttributeHandle clutchAH;
};

struct TankDriveTransmissionCommandState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle thrustsAH;
};

struct ClutchResponseParams	
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle maxResponseAH;
};

struct ClutchParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle accuracyAH;
	OmniPvdAttributeHandle iterationsAH;
};

struct EngineParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle torqueCurveAH;
	OmniPvdAttributeHandle moiAH;
	OmniPvdAttributeHandle peakTorqueAH;
	OmniPvdAttributeHandle idleOmegaAH;
	OmniPvdAttributeHandle maxOmegaAH;
	OmniPvdAttributeHandle dampingRateFullThrottleAH;
	OmniPvdAttributeHandle dampingRateZeroThrottleClutchEngagedAH;
	OmniPvdAttributeHandle dampingRateZeroThrottleClutchDisengagedAH;	
};

struct GearboxParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle ratiosAH;
	OmniPvdAttributeHandle nbRatiosAH;
	OmniPvdAttributeHandle neutralGearAH;
	OmniPvdAttributeHandle finalRatioAH;
	OmniPvdAttributeHandle switchTimeAH;
};

struct AutoboxParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle upRatiosAH;
	OmniPvdAttributeHandle downRatiosAH;
	OmniPvdAttributeHandle latencyAH;
};

struct MultiWheelDiffParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle torqueRatios0To3AH;
	OmniPvdAttributeHandle torqueRatios4To7AH;
	OmniPvdAttributeHandle torqueRatios8To11AH;
	OmniPvdAttributeHandle torqueRatios12To15AH;
	OmniPvdAttributeHandle torqueRatios16To19AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios0To3AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios4To7AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios8To11AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios12To15AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios16To19AH;
};

struct FourWheelDiffParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle frontWheelsAH;
	OmniPvdAttributeHandle rearWheelsAH;
	OmniPvdAttributeHandle frontBiasAH;
	OmniPvdAttributeHandle frontTargetAH;
	OmniPvdAttributeHandle rearBiasAH;
	OmniPvdAttributeHandle rearTargetAH;
	OmniPvdAttributeHandle centreBiasAH;
	OmniPvdAttributeHandle centreTargetAH;
};

struct TankDiffParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle nbTracksAH;
	OmniPvdAttributeHandle thrustIdPerTrackAH;
	OmniPvdAttributeHandle nbWheelsPerTrackAH;
	OmniPvdAttributeHandle trackToWheelIdsAH;
	OmniPvdAttributeHandle wheelIdsInTrackOrderAH;
};

struct ClutchResponseState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle normalisedResponseAH;
	OmniPvdAttributeHandle responseAH;
};

struct ThrottleResponseState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle responseAH;
};

struct EngineState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle rotationSpeedAH;
};

struct GearboxState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle currentGearAH;
	OmniPvdAttributeHandle targetGearAH;
	OmniPvdAttributeHandle switchTimeAH;
};

struct AutoboxState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle timeSinceLastShiftAH;
	OmniPvdAttributeHandle activeAutoboxGearShiftAH;
};

struct DiffState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle torqueRatios0To3AH;
	OmniPvdAttributeHandle torqueRatios4To7AH;
	OmniPvdAttributeHandle torqueRatios8To11AH;
	OmniPvdAttributeHandle torqueRatios12To15AH;
	OmniPvdAttributeHandle torqueRatios16To19AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios0To3AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios4To7AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios8To11AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios12To15AH;
	OmniPvdAttributeHandle aveWheelSpeedRatios16To19AH;
};

struct ClutchSlipState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle slipAH;
};

struct EngineDrivetrain
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle commandStateAH;
	OmniPvdAttributeHandle transmissionCommandStateAH;
	OmniPvdAttributeHandle clutchResponseParamsAH;
	OmniPvdAttributeHandle clutchParamsAH;
	OmniPvdAttributeHandle engineParamsAH;
	OmniPvdAttributeHandle gearboxParamsAH;
	OmniPvdAttributeHandle autoboxParamsAH;
	OmniPvdAttributeHandle differentialParamsAH;
	OmniPvdAttributeHandle clutchResponseStateAH;
	OmniPvdAttributeHandle throttleResponseStateAH;
	OmniPvdAttributeHandle engineStateAH;
	OmniPvdAttributeHandle gearboxStateAH;
	OmniPvdAttributeHandle autoboxStateAH;
	OmniPvdAttributeHandle diffStateAH;
	OmniPvdAttributeHandle clutchSlipStateAH;
};

EngineDriveCommandState registerEngineDriveCommandState(OmniPvdWriter& omniWriter);
EngineDriveTransmissionCommandState registerEngineDriveTransmissionCommandState(OmniPvdWriter& omniWriter);
TankDriveTransmissionCommandState registerTankDriveTransmissionCommandState(OmniPvdWriter&, OmniPvdClassHandle baseClass);
ClutchResponseParams registerClutchResponseParams(OmniPvdWriter& omniWriter);
ClutchParams registerClutchParams(OmniPvdWriter& omniWriter);
EngineParams registerEngineParams(OmniPvdWriter& omniWriter);
GearboxParams registerGearboxParams(OmniPvdWriter& omniWriter);
AutoboxParams registerAutoboxParams(OmniPvdWriter& omniWriter);
MultiWheelDiffParams registerMultiWheelDiffParams(OmniPvdWriter& omniWriter);
FourWheelDiffParams registerFourWheelDiffParams(OmniPvdWriter& omniWriter, OmniPvdClassHandle baseClass);
TankDiffParams registerTankDiffParams(OmniPvdWriter& omniWriter, OmniPvdClassHandle baseClass);
//TankDiffParams registerTankDiffParams(OmniPvdWriter& omniWriter);
ClutchResponseState registerClutchResponseState(OmniPvdWriter& omniWriter);
ThrottleResponseState registerThrottleResponseState(OmniPvdWriter& omniWriter);
EngineState registerEngineState(OmniPvdWriter& omniWriter);
GearboxState registerGearboxState(OmniPvdWriter& omniWriter);
AutoboxState registerAutoboxState(OmniPvdWriter& omniWriter);
DiffState registerDiffState(OmniPvdWriter& omniWriter);
ClutchSlipState registerClutchSlipState(OmniPvdWriter& omniWriter);
EngineDrivetrain registerEngineDrivetrain(OmniPvdWriter& omniWriter);

void writeEngineDriveCommandState
(const PxVehicleCommandState& commandState, 
 const OmniPvdObjectHandle oh, const EngineDriveCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeEngineDriveTransmissionCommandState
(const PxVehicleEngineDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const EngineDriveTransmissionCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTankDriveTransmissionCommandState
(const PxVehicleTankDriveTransmissionCommandState&,	const OmniPvdObjectHandle,
 const EngineDriveTransmissionCommandState&, const TankDriveTransmissionCommandState&,
 OmniPvdWriter&, OmniPvdContextHandle);

void writeClutchResponseParams
(const PxVehicleClutchCommandResponseParams& clutchResponseParams, 
 const OmniPvdObjectHandle oh, const ClutchResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeClutchParams
(const PxVehicleClutchParams& clutchParams, 
 const OmniPvdObjectHandle oh, const ClutchParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeEngineParams
(const PxVehicleEngineParams& engineParams, 
 const OmniPvdObjectHandle oh, const EngineParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeGearboxParams
(const PxVehicleGearboxParams& gearboxParams, 
 const OmniPvdObjectHandle oh, const GearboxParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeAutoboxParams
(const PxVehicleAutoboxParams& gearboxParams, 
 const OmniPvdObjectHandle oh, const AutoboxParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeMultiWheelDiffParams
(const PxVehicleMultiWheelDriveDifferentialParams& diffParams, 
 const OmniPvdObjectHandle oh, const MultiWheelDiffParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeFourWheelDiffParams
(const PxVehicleFourWheelDriveDifferentialParams& diffParams, 
 const OmniPvdObjectHandle oh, const MultiWheelDiffParams&, const FourWheelDiffParams& ah, 
 OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeTankDiffParams
(const PxVehicleTankDriveDifferentialParams&, 
 const OmniPvdObjectHandle, const MultiWheelDiffParams&, const TankDiffParams&, 
 OmniPvdWriter&, OmniPvdContextHandle);

void writeClutchResponseState
(const PxVehicleClutchCommandResponseState& clutchResponseState, 
 const OmniPvdObjectHandle oh, const ClutchResponseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeThrottleResponseState
(const PxVehicleEngineDriveThrottleCommandResponseState& throttleResponseState, 
 const OmniPvdObjectHandle oh, const ThrottleResponseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeEngineState
(const PxVehicleEngineState& engineState, 
 const OmniPvdObjectHandle oh, const EngineState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeGearboxState
(const PxVehicleGearboxState& gearboxState, 
 const OmniPvdObjectHandle oh, const GearboxState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeAutoboxState
(const PxVehicleAutoboxState& gearboxState, 
 const OmniPvdObjectHandle oh, const AutoboxState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeDiffState
(const PxVehicleDifferentialState& diffState, 
 const OmniPvdObjectHandle oh, const DiffState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writeClutchSlipState
(const PxVehicleClutchSlipState& clutchSlipState, 
 const OmniPvdObjectHandle oh, const ClutchSlipState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

///////////////////////////////
//WHEEL ATTACHMENT PHYSX INTEGRATION
///////////////////////////////

struct PhysXSuspensionLimitConstraintParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle restitutionAH;
	OmniPvdAttributeHandle directionForSuspensionLimitConstraintAH;
};

struct PhysXWheelShape
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle shapePtrAH;
};

struct PhysXRoadGeomState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle hitActorPtrAH;
	OmniPvdAttributeHandle hitShapePtrAH;
	OmniPvdAttributeHandle hitMaterialPtrAH;
	OmniPvdAttributeHandle hitPositionAH;
};

struct PhysXConstraintState
{
	OmniPvdClassHandle CH;

	OmniPvdAttributeHandle tireLongActiveStatusAH;
	OmniPvdAttributeHandle tireLongLinearAH;
	OmniPvdAttributeHandle tireLongAngularAH;
	OmniPvdAttributeHandle tireLongDampingAH;

	OmniPvdAttributeHandle tireLatActiveStatusAH;
	OmniPvdAttributeHandle tireLatLinearAH;
	OmniPvdAttributeHandle tireLatAngularAH;
	OmniPvdAttributeHandle tireLatDampingAH;

	OmniPvdAttributeHandle suspActiveStatusAH;
	OmniPvdAttributeHandle suspLinearAH;
	OmniPvdAttributeHandle suspAngularAH;
	OmniPvdAttributeHandle suspGeometricErrorAH;
	OmniPvdAttributeHandle suspRestitutionAH;
};

struct PhysXWheelAttachment
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle physxWeelShapeAH;
	OmniPvdAttributeHandle physxConstraintParamsAH;
	OmniPvdAttributeHandle physxRoadGeometryStateAH;
	OmniPvdAttributeHandle physxConstraintStateAH;
	OmniPvdAttributeHandle physxMaterialFrictionSetAH;
};

struct PhysXMaterialFriction
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle frictionAH;
	OmniPvdAttributeHandle materialPtrAH;
};

PhysXSuspensionLimitConstraintParams registerSuspLimitConstraintParams(OmniPvdWriter& omniWriter);
PhysXWheelShape registerPhysXWheelShape(OmniPvdWriter& omniWriter);
PhysXRoadGeomState registerPhysXRoadGeomState(OmniPvdWriter& omniWriter);
PhysXConstraintState registerPhysXConstraintState(OmniPvdWriter& omniWriter);
PhysXWheelAttachment registerPhysXWheelAttachment(OmniPvdWriter& omniWriter);
PhysXMaterialFriction registerPhysXMaterialFriction(OmniPvdWriter& omniWriter);

void writePhysXSuspLimitConstraintParams
(const PxVehiclePhysXSuspensionLimitConstraintParams& params,
 const OmniPvdObjectHandle oh, const PhysXSuspensionLimitConstraintParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writePhysXWheelShape
(const PxShape* wheelShape, 
 const OmniPvdObjectHandle oh, const PhysXWheelShape& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writePhysXRoadGeomState
(const PxVehiclePhysXRoadGeometryQueryState& roadGeomState,
const OmniPvdObjectHandle oh, const PhysXRoadGeomState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writePhysXConstraintState
(const PxVehiclePhysXConstraintState& roadGeomState,
 const OmniPvdObjectHandle oh, const PhysXConstraintState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writePhysXMaterialFriction
(const PxVehiclePhysXMaterialFriction& materialFriction,
 const OmniPvdObjectHandle oh, const PhysXMaterialFriction& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

//////////////////////////////
//PHYSX RIGID ACTOR
//////////////////////////////

struct PhysXRoadGeometryQueryFilterData
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle word0AH;
	OmniPvdAttributeHandle word1AH;
	OmniPvdAttributeHandle word2AH;
	OmniPvdAttributeHandle word3AH;
	OmniPvdAttributeHandle flagsAH;
};

struct PhysXRoadGeometryQueryParams
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle queryTypeAH;
	PhysXRoadGeometryQueryFilterData filterDataParams;
	OmniPvdAttributeHandle defaultFilterDataAH;
	OmniPvdAttributeHandle filterDataSetAH;
};

struct PhysXRigidActor
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle rigidActorAH;
};

struct PhysXSteerState
{
	OmniPvdClassHandle CH;
	OmniPvdAttributeHandle previousSteerCommandAH;
};


PhysXRoadGeometryQueryParams registerPhysXRoadGeometryQueryParams(OmniPvdWriter& omniWriter);
PhysXRigidActor registerPhysXRigidActor(OmniPvdWriter& omniWriter);
PhysXSteerState registerPhysXSteerState(OmniPvdWriter& omniWriter);

void writePhysXRigidActor
(const PxRigidActor* actor, 
 const OmniPvdObjectHandle oh, const PhysXRigidActor& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writePhysXRoadGeometryQueryFilterData
(const PxQueryFilterData&, 
 const OmniPvdObjectHandle, const PhysXRoadGeometryQueryFilterData&, OmniPvdWriter&, OmniPvdContextHandle);

void writePhysXRoadGeometryQueryParams
(const PxVehiclePhysXRoadGeometryQueryParams&, const PxVehicleAxleDescription& axleDesc,
 const OmniPvdObjectHandle queryParamsOH, const OmniPvdObjectHandle defaultFilterDataOH, const OmniPvdObjectHandle* filterDataOHs,
 const PhysXRoadGeometryQueryParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch);

void writePhysXSteerState
(const PxVehiclePhysXSteerState&, 
 const OmniPvdObjectHandle, const PhysXSteerState&, OmniPvdWriter&, OmniPvdContextHandle);

//////////////////////////////
//VEHICLE
//////////////////////////////

struct Vehicle
{
	OmniPvdClassHandle CH;

	OmniPvdAttributeHandle rigidBodyParamsAH;
	OmniPvdAttributeHandle rigidBodyStateAH;

	OmniPvdAttributeHandle suspStateCalcParamsAH;

	OmniPvdAttributeHandle brakeResponseParamsSetAH;
	OmniPvdAttributeHandle steerResponseParamsAH;
	OmniPvdAttributeHandle brakeResponseStatesAH;
	OmniPvdAttributeHandle steerResponseStatesAH;
	OmniPvdAttributeHandle ackermannParamsAH;

	OmniPvdAttributeHandle wheelAttachmentSetAH;

	OmniPvdAttributeHandle antiRollSetAH;
	OmniPvdAttributeHandle antiRollForceAH;

	OmniPvdAttributeHandle directDrivetrainAH;
	OmniPvdAttributeHandle engineDriveTrainAH;

	OmniPvdAttributeHandle physxWheelAttachmentSetAH;

	OmniPvdAttributeHandle physxRoadGeometryQueryParamsAH;
	OmniPvdAttributeHandle physxRigidActorAH;
	OmniPvdAttributeHandle physxSteerStateAH;
};

Vehicle registerVehicle(OmniPvdWriter& omniWriter);


#if !PX_DOXYGEN
} // namespace vehicle2
} // namespace physx
#endif

#endif //PX_SUPPORT_OMNI_PVD

/** @} */
