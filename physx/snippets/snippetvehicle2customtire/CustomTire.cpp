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

#include "CustomTire.h"

#include "VehicleMFTire.h"


namespace snippetvehicle2
{

void CustomTireGripUpdate(
	bool isWheelOnGround,
	PxF32 unfilteredLoad, PxF32 restLoad, PxF32 maxNormalizedLoad,
	PxF32 friction,
	PxVehicleTireGripState& trGripState)
{
	trGripState.setToDefault();

	//If the wheel is not touching the ground then carry on with zero grip state.
	if (!isWheelOnGround)
		return;

	//Note: in a future release the tire load might be recomputed here using
	//      mfTireComputeLoad(). The missing piece is the tire normal deflection
	//      value (difference between free rolling radius and loaded radius).
	//      With a two degree of freedom quarter car model, this value could
	//      be estimated using the compression length of the tire spring.

	//Compute load and friction.
	const PxF32 normalizedLoad = unfilteredLoad / restLoad;
	if (normalizedLoad < maxNormalizedLoad)
		trGripState.load = unfilteredLoad;
	else
		trGripState.load = maxNormalizedLoad * restLoad;

	trGripState.friction = friction;
}

void CustomTireSlipsUpdate(
	const MFTireData& tireData,
	const PxVehicleTireSpeedState& tireSpeedState,
	PxF32 wheelOmega, PxF32 tireLoad,
	PxVehicleTireSlipState& tireSlipState,
	PxF32& effectiveRollingRadius)
{
	typedef MFTireConfig::Float TFloat;

	TFloat longSlipTmp, tanLatSlipTmp, effectiveRollingRadiusTmp;
	mfTireComputeSlip<MFTireConfig>(tireData, 
		tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL],
		tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL],
		wheelOmega, tireLoad, tireData.sharedParams.pi0,
		longSlipTmp, tanLatSlipTmp, effectiveRollingRadiusTmp);

	tireSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL] = PxReal(longSlipTmp);
	tireSlipState.slips[PxVehicleTireDirectionModes::eLATERAL] = PxReal(MF_ARCTAN(-tanLatSlipTmp));
	// note: implementation of Magic Formula Tire Model has lateral axis flipped.
	//       Furthermore, to be consistent with the default PhysX states, the angle is returned.

	effectiveRollingRadius = PxF32(effectiveRollingRadiusTmp);
}

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
	PxVehicleTireForce& tireForce)
{
	typedef MFTireConfig::Float TFloat;

	PxF32 wheelTorque;
	PxF32 tireLongForce;
	PxF32 tireLatForce;
	PxF32 tireAlignMoment;

	if ((tireGripState.friction > 0.0f) && (tireGripState.load > 0.0f))
	{
		// note: implementation of Magic Formula Tire Model has lateral axis flipped. Furthermore, it expects the
		//       tangens of the angle.
		const TFloat tanLatSlipNeg = PxTan(-tireSlipState.slips[PxVehicleTireDirectionModes::eLATERAL]);

		TFloat wheelTorqueTmp, tireLongForceTmp, tireLatForceTmp, tireAlignMomentTmp;
		mfTireComputeForce<MFTireConfig>(tireData, tireGripState.friction,
			tireSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL], tanLatSlipNeg, camber,
			effectiveRollingRadius,
			tireGripState.load, tireData.sharedParams.pi0,
			tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL],
			tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL],
			wheelTorqueTmp, tireLongForceTmp, tireLatForceTmp, tireAlignMomentTmp);

		wheelTorque = PxF32(wheelTorqueTmp);
		tireLongForce = PxF32(tireLongForceTmp);
		tireLatForce = PxF32(tireLatForceTmp);
		tireAlignMoment = PxF32(tireAlignMomentTmp);

		// In the Magic Formula Tire Model, having 0 longitudinal slip does not necessarily mean that
		// the longitudinal force will be 0 too. The graph used to compute the force allows for vertical
		// and horizontal shift to model certain effects. Similarly, the lateral force will not necessarily
		// be 0 just because lateral slip and camber are 0. If the 0 => 0 behavior is desired, then the
		// parameters need to be set accordingly (see the parameters related to the Sh, Sv parts of the
		// Magic Formula. The user scaling factors lambdaH and lambdaV can be set to 0, for example, to
		// eliminate the effect of the parameters that shift the graphs).
		//
		// For parameter configurations where 0 slip does not result in 0 force, vehicles might never come
		// fully to rest. The PhysX default tire model has the sticky tire concept that drives the velocity
		// towards 0 once velocities stay below a threshold for a defined amount of time. This might not
		// be enough to cancel the constantly applied force at 0 slip or the sticky tire damping coefficient
		// needs to be very high. Thus, the following code is added to set the forces to 0 when the tire
		// fulfills the "stickiness" condition and overrules the results from the Magic Formula Tire Model.

		const bool clearLngForce = tireStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL];
		const bool clearLatForce = tireStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL];

		if (clearLngForce)
		{
			wheelTorque = 0.0f;
			tireLongForce = 0.0f;
		}

		if (clearLatForce)  // note: small camber angle could also be seen as requirement but the sticky tire active state is seen as reference here
		{
			tireLatForce = 0.0f;
		}

		if (clearLngForce && clearLatForce)
		{
			tireAlignMoment = 0.0f;
		}
	}
	else
	{
		wheelTorque = 0.0f;
		tireLongForce = 0.0f;
		tireLatForce = 0.0f;
		tireAlignMoment = 0.0f;
	}

	const PxVec3 tireLongForceVec = tireDirectionState.directions[PxVehicleTireDirectionModes::eLONGITUDINAL] * tireLongForce;
	const PxVec3 tireLatForceVec = tireDirectionState.directions[PxVehicleTireDirectionModes::eLATERAL] * tireLatForce;

	tireForce.forces[PxVehicleTireDirectionModes::eLONGITUDINAL] = tireLongForceVec;
	tireForce.forces[PxVehicleTireDirectionModes::eLATERAL] = tireLatForceVec;

	const PxVec3 r = bodyPose.rotate(suspensionAttachmentPose.transform(tireForceApplicationPoint));
	tireForce.torques[PxVehicleTireDirectionModes::eLONGITUDINAL] = r.cross(tireLongForceVec);
	tireForce.torques[PxVehicleTireDirectionModes::eLATERAL] = r.cross(tireLatForceVec);

	tireForce.aligningMoment = tireAlignMoment;
	tireForce.wheelTorque = wheelTorque;
}

}//namespace snippetvehicle2
