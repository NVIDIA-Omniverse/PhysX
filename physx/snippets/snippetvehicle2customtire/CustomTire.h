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

#pragma once

#include "PxPhysicsAPI.h"

#include "VehicleMFTireData.h"


namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;


/**
\brief Struct to configure the templated functions of the Magic Formula Tire Model.

A typedef named "Float" declares the floating point type to use (to configure for
32bit or 64bit precision). Some static boolean members allow to enable/disable code
at compile time depending on the desired feature set and complexity.
*/
struct MFTireConfig
{
	typedef PxF32 Float;

	static const bool supportInflationPressure = false;
	//PhysX vehicles do not model inflation pressure (nore can it be derived/estimated
	//from the PhysX state values).

	static const bool supportCamber = true;

	static const bool supportTurnSlip = false;
	//Turn slip effects will be ignored (same as having all parameters named "zeta..." set to 1).
};

typedef MFTireDataT<MFTireConfig::Float> MFTireData;


/**
\brief Custom method to compute tire grip values.

\note If the suspension cannot place the wheel on the ground, the tire load and friction will be 0.

\param[in] isWheelOnGround True if the wheel is touching the ground.
\param[in] unfilteredLoad The force pushing the tire to the ground.
\param[in] restLoad The nominal vertical load of the tire (the expected load at rest).
\param[in] maxNormalizedLoad The maximum normalized load (load / restLoad).
           Values above will result in the load being clamped. This can avoid instabilities
		   when dealing with discontinuities that generate large load (like the
		   suspension compressing by a large delta for a small simulation timestep).
\param[in] friction The friction coefficient to use.
\param[out] tireGripState The computed load and friction experienced by the tire.
*/
void CustomTireGripUpdate(
	bool isWheelOnGround,
	PxF32 unfilteredLoad, PxF32 restLoad, PxF32 maxNormalizedLoad,
	PxF32 friction,
	PxVehicleTireGripState& tireGripState);

/**
\brief Custom method to compute tire slip values.

\note See MFTireConfig for the configuration this method is defined for.

\param[in] tireData The tire parameters of the Magic Formula Tire Model.
\param[in] tireSpeedState The velocity at the tire contact point projected along the tire's
           longitudinal and lateral axes.
\param[in] wheelOmega The wheel rotation speed in radians per second.
\param[in] tireLoad The force pushing the tire to the ground.
\param[out] tireSlipState The computed tire longitudinal and lateral slips.
\param[out] effectiveRollingRadius The radius re that matches the angular velocity (omega) and the velocity
            at the contact patch (V) under free rolling conditions (no drive/break torque), i.e., 
			re = V / omega. This radius is bounded by the free tire radius of the rotating tire on one end
			and the loaded tire radius on the other end. The effective rolling radius changes with the load
			(fast for low values of load but only marginally at higher load values).
*/
void CustomTireSlipsUpdate(
	const MFTireData& tireData,
	const PxVehicleTireSpeedState& tireSpeedState,
	PxF32 wheelOmega, PxF32 tireLoad,
	PxVehicleTireSlipState& tireSlipState,
	PxF32& effectiveRollingRadius);

/**
\brief Custom method to compute the longitudinal and lateral tire forces using the
       Magic Formula Tire Model.

\note See MFTireConfig for the configuration this method is defined for.

\note This tire model requires running with a high simulation update rate (1kHz is recommended).

\param[in] tireData The tire parameters of the Magic Formula Tire Model.
\param[in] tireSlipState The tire longitudinal and lateral slips.
\param[in] tireSpeedState The velocity at the tire contact point projected along the tire's
           longitudinal and lateral axes.
\param[in] tireDirectionState The tire's longitudinal and lateral directions in the ground plane.
\param[in] tireGripState The load and friction experienced by the tire.
\param[in] tireStickyState Description of the sticky state of the tire in the longitudinal and lateral directions.
\param[in] bodyPose The world transform of the vehicle body.
\param[in] suspensionAttachmentPose The transform of the suspension attachment (in vehicle body space).
\param[in] tireForceApplicationPoint The tire force application point (in suspension attachment space).
\param[in] camber The camber angle of the tire expressed in radians.
\param[in] effectiveRollingRadius The radius under free rolling conditions (see CustomTireSlipsUpdate
           for details).
\param[out] tireForce The computed tire forces in the world frame.
*/
void CustomTireForcesUpdate(
	const MFTireData& tireData,
	const PxVehicleTireSlipState& tireSlipState,
	const PxVehicleTireSpeedState& tireSpeedState,
	const PxVehicleTireDirectionState& tireDirectionState,
	const PxVehicleTireGripState& tireGripState,
	const PxVehicleTireStickyState& tireStickyState,
	const PxTransform& bodyPose,
	const PxTransform& suspensionAttachmentPose,
	const PxVec3& tireForceApplicationPoint,
	PxF32 camber,
	PxF32 effectiveRollingRadius,
	PxVehicleTireForce& tireForce);


struct CustomTireParams
{
	MFTireData mfTireData;
	PxReal maxNormalizedLoad;  // maximum normalized load (load / restLoad). Values above will be clamped.
	                           // Large discontinuities can cause unnaturally large load values which the
	                           // Magic Formula Tire Model does not support (for example having the wheel
	                           // go from one frame with no ground contact to a highly compressed suspension
	                           // in the next frame).
};


class CustomTireComponent : public PxVehicleComponent
{
public:

	CustomTireComponent() : PxVehicleComponent() {}

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
		PxVehicleArrayData<PxVehicleTireForce>& tireForces) = 0;

	virtual bool update(const PxReal dt, const PxVehicleSimulationContext& context)
	{
		const PxVehicleAxleDescription* axleDescription;
		PxVehicleArrayData<const PxReal> steerResponseStates;
		const PxVehicleRigidBodyState* rigidBodyState;
		PxVehicleArrayData<const PxVehicleWheelActuationState> actuationStates;
		PxVehicleArrayData<const PxVehicleWheelParams> wheelParams;
		PxVehicleArrayData<const PxVehicleSuspensionParams> suspensionParams;
		PxVehicleArrayData<const CustomTireParams> tireParams;
		PxVehicleArrayData<const PxVehicleRoadGeometryState> roadGeomStates;
		PxVehicleArrayData<const PxVehicleSuspensionState> suspensionStates;
		PxVehicleArrayData<const PxVehicleSuspensionComplianceState> suspensionComplianceStates;
		PxVehicleArrayData<const PxVehicleSuspensionForce> suspensionForces;
		PxVehicleArrayData<const PxVehicleWheelRigidBody1dState> wheelRigidBody1DStates;
		PxVehicleArrayData<PxVehicleTireGripState> tireGripStates;
		PxVehicleArrayData<PxVehicleTireDirectionState> tireDirectionStates;
		PxVehicleArrayData<PxVehicleTireSpeedState> tireSpeedStates;
		PxVehicleArrayData<PxVehicleTireSlipState> tireSlipStates;
		PxVehicleArrayData<PxVehicleTireCamberAngleState> tireCamberAngleStates;
		PxVehicleArrayData<PxVehicleTireStickyState> tireStickyStates;
		PxVehicleArrayData<PxVehicleTireForce> tireForces;

		getDataForCustomTireComponent(axleDescription, steerResponseStates, 
			rigidBodyState, actuationStates, wheelParams, suspensionParams, tireParams,
			roadGeomStates, suspensionStates, suspensionComplianceStates, suspensionForces,
			wheelRigidBody1DStates, tireGripStates, tireDirectionStates, tireSpeedStates,
			tireSlipStates, tireCamberAngleStates, tireStickyStates, tireForces);

		for (PxU32 i = 0; i < axleDescription->nbWheels; i++)
		{
			const PxU32 wheelId = axleDescription->wheelIdsInAxleOrder[i];
			const PxReal& steerResponseState = steerResponseStates[wheelId];
			const PxVehicleWheelParams& wheelParam = wheelParams[wheelId];
			const PxVehicleSuspensionParams& suspensionParam = suspensionParams[wheelId];
			const CustomTireParams& tireParam = tireParams[wheelId];
			const PxVehicleRoadGeometryState& roadGeomState = roadGeomStates[wheelId];
			const PxVehicleSuspensionState& suspensionState = suspensionStates[wheelId];
			const PxVehicleSuspensionComplianceState& suspensionComplianceState = suspensionComplianceStates[wheelId];
			const PxVehicleWheelRigidBody1dState& wheelRigidBody1dState = wheelRigidBody1DStates[wheelId];

			const bool isWheelOnGround = PxVehicleIsWheelOnGround(suspensionState);

			//Compute the tire slip directions
			PxVehicleTireDirectionState& tireDirectionState = tireDirectionStates[wheelId];
			PxVehicleTireDirsUpdate(
				suspensionParam,
				steerResponseState,
				roadGeomState.plane.n, isWheelOnGround,
				suspensionComplianceState,
				*rigidBodyState,
				context.frame,
				tireDirectionState);

			//Compute the rigid body speeds along the tire slip directions.
			PxVehicleTireSpeedState& tireSpeedState = tireSpeedStates[wheelId];
			PxVehicleTireSlipSpeedsUpdate(
				wheelParam, suspensionParam,
				steerResponseState, suspensionState, tireDirectionState,
				*rigidBodyState, roadGeomState,
				context.frame,
				tireSpeedState);

			//Compute grip state
			PxVehicleTireGripState& tireGripState = tireGripStates[wheelId];
			CustomTireGripUpdate(
				isWheelOnGround,
				suspensionForces[wheelId].normalForce, PxF32(tireParam.mfTireData.sharedParams.fz0), tireParam.maxNormalizedLoad,
				roadGeomState.friction,
				tireGripState);

			//Compute the tire slip values.
			PxVehicleTireSlipState& tireSlipState = tireSlipStates[wheelId];
			PxF32 effectiveRollingRadius;

			//Ensure radius is in sync
			#if PX_ENABLE_ASSERTS  // avoid warning about unusued local typedef
			typedef MFTireConfig::Float TFloat;
			#endif
			PX_ASSERT(PxAbs(tireParam.mfTireData.sharedParams.r0 - TFloat(wheelParam.radius)) < (tireParam.mfTireData.sharedParams.r0 * TFloat(0.01)));

			CustomTireSlipsUpdate(
				tireParam.mfTireData,
				tireSpeedState,
				wheelRigidBody1dState.rotationSpeed, tireGripState.load,
				tireSlipState,
				effectiveRollingRadius);

			//Update the camber angle
			PxVehicleTireCamberAngleState& tireCamberAngleState = tireCamberAngleStates[wheelId];
			PxVehicleTireCamberAnglesUpdate(
				suspensionParam, steerResponseState, 
				roadGeomState.plane.n, isWheelOnGround,
				suspensionComplianceState, *rigidBodyState,
				context.frame,
				tireCamberAngleState);

			//Update the tire sticky state
			//
			//Note: this should be skipped if tires do not use the sticky feature
			PxVehicleTireStickyState& tireStickyState = tireStickyStates[wheelId];
			PxVehicleTireStickyStateUpdate(
				*axleDescription,
				wheelParam,
				context.tireStickyParams,
				actuationStates, tireGripState,
				tireSpeedState, wheelRigidBody1dState,
				dt,
				tireStickyState);

			//If sticky tire is active set the slip values to zero.
			//
			//Note: this should be skipped if tires do not use the sticky feature
			PxVehicleTireSlipsAccountingForStickyStatesUpdate(
				tireStickyState,
				tireSlipState);

			//Compute the tire forces
			PxVehicleTireForce& tireForce = tireForces[wheelId];

			CustomTireForcesUpdate(
				tireParam.mfTireData,
				tireSlipState, tireSpeedState, tireDirectionState, tireGripState, tireStickyState,
				rigidBodyState->pose, suspensionParam.suspensionAttachment, suspensionComplianceState.tireForceAppPoint,
				tireCamberAngleState.camberAngle,
				effectiveRollingRadius,
				tireForce);
		}

		return true;
	}
};

}//namespace snippetvehicle2
