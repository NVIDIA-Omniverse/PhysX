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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/drivetrain/PxVehicleDrivetrainStates.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle2/roadGeometry/PxVehicleRoadGeometryState.h"

#include "vehicle2/suspension/PxVehicleSuspensionHelpers.h"

#include "vehicle2/tire/PxVehicleTireFunctions.h"
#include "vehicle2/tire/PxVehicleTireParams.h"

#include "vehicle2/wheel/PxVehicleWheelHelpers.h"

namespace physx
{
namespace vehicle2
{

PX_FORCE_INLINE PxF32 computeFilteredNormalisedTireLoad
(const PxF32 xmin, const PxF32 ymin, const PxF32 xmax, const PxF32 ymax, const PxF32 x)
{
	if (x <= xmin)
	{
		return ymin;
	}
	else if (x >= xmax)
	{
		return ymax;
	}
	else
	{
		return (ymin + (x - xmin)*(ymax - ymin)/ (xmax - xmin));
	}
}

void PxVehicleTireDirsLegacyUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxReal steerAngle, const PxVehicleRoadGeometryState& rdGeomState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxVehicleFrame& frame,
 PxVehicleTireDirectionState& trSlipDirs)
{
	trSlipDirs.setToDefault();

	//If there are no hits we'll have no ground plane.
	if (!rdGeomState.hitState)
			return;


	//Compute wheel orientation with zero compliance, zero steer and zero 
	//rotation. Ignore rotation because it plays no role due to radial 
	//symmetry of wheel. Steer will be applied to the pose so we can 
	//ignore when computing the orientation. Compliance ought to be applied but
	//we're not doing this in legacy code.
	const PxQuat wheelOrientation = PxVehicleComputeWheelOrientation(frame, suspParams, 0.0f, 0.0f, 0.0f, 
		rigidBodyState.pose.q, 0.0f);

	//We need lateral dir and hit norm to project wheel into ground plane.
	const PxVec3 latDir = wheelOrientation.rotate(frame.getLatAxis());
	const PxVec3& hitNorm = rdGeomState.plane.n;

	//Compute the tire axes in the ground plane.
	PxVec3 tLongRaw = latDir.cross(hitNorm);
	PxVec3 tLatRaw = hitNorm.cross(tLongRaw);
	tLongRaw.normalize();
	tLatRaw.normalize();

	//Rotate the tire using the steer angle.
	const PxF32 yawAngle = steerAngle;
	const PxF32 cosWheelSteer = PxCos(yawAngle);
	const PxF32 sinWheelSteer = PxSin(yawAngle);
	const PxVec3 tLong = tLongRaw * cosWheelSteer + tLatRaw * sinWheelSteer;
	const PxVec3 tLat = tLatRaw * cosWheelSteer - tLongRaw * sinWheelSteer;

	trSlipDirs.directions[PxVehicleTireDirectionModes::eLATERAL] = tLat;
	trSlipDirs.directions[PxVehicleTireDirectionModes::eLONGITUDINAL] = tLong;
}

void PxVehicleTireDirsUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxReal steerAngle, const PxVec3& groundNormal, bool isWheelOnGround,
 const PxVehicleSuspensionComplianceState& compState,
 const PxVehicleRigidBodyState& rigidBodyState,
 const PxVehicleFrame& frame,
 PxVehicleTireDirectionState& trSlipDirs)
{
	trSlipDirs.setToDefault();

	//Skip if the suspension could not push the wheel to the ground.
	if (!isWheelOnGround)
		return;

	//Compute the wheel quaternion in the world frame.
	//Ignore rotation because it plays no role due to radial symmetry of wheel.
	const PxQuat wheelOrientation = PxVehicleComputeWheelOrientation(frame, suspParams, compState.camber, compState.toe, steerAngle, 
		rigidBodyState.pose.q, 0.0f);

	//We need lateral dir and hit norm to project wheel into ground plane.
	const PxVec3 latDir = wheelOrientation.rotate(frame.getLatAxis());

	//Compute the tire axes in the ground plane.
	PxVec3 lng = latDir.cross(groundNormal);
	PxVec3 lat = groundNormal.cross(lng);
	lng.normalize();
	lat.normalize();

	//Set the direction vectors.
	trSlipDirs.directions[PxVehicleTireDirectionModes::eLATERAL] = lat;
	trSlipDirs.directions[PxVehicleTireDirectionModes::eLONGITUDINAL] = lng;
}

PX_FORCE_INLINE PxF32 computeLateralSlip
(const PxF32 lngSpeed, const PxF32 latSpeed, const PxF32 minLatSlipDenominator)
{
	const PxF32 latSlip = PxAtan(latSpeed / (PxAbs(lngSpeed) + minLatSlipDenominator));
	return latSlip;
}
 
PX_FORCE_INLINE PxF32 computeLongitudionalSlip
(const bool useLegacyLongSlipCalculation,
 const PxF32 longSpeed,
 const PxF32 wheelOmega, const PxF32 wheelRadius, 
 const PxF32 minPassiveLongSlipDenominator, const PxF32 minActiveLongSlipDenominator,
 const bool isAccelApplied, const bool isBrakeApplied)
{
	const PxF32 longSpeedAbs = PxAbs(longSpeed);
	const PxF32 wheelSpeed = wheelOmega * wheelRadius;
	const PxF32 wheelSpeedAbs = PxAbs(wheelSpeed);

	PxF32 lngSlip = 0.0f;

	//If nothing is moving just avoid a divide by zero and set the long slip to zero.
	if (longSpeed == 0 && wheelOmega == 0)
	{
		lngSlip = 0.0f;
	}
	else
	{
		if (isBrakeApplied || isAccelApplied)
		{
			//Wheel experiencing an applied torque.
			//Use the raw denominator value plus an offset to avoid anything approaching a divide by zero.
			//When accelerating from rest the small denominator will generate really quite large 
			//slip values, which will, in turn, generate large longitudinal forces. With large 
			//time-steps this might lead to a temporary oscillation in longSlip direction and an 
			//oscillation in wheel speed direction.  The amplitude of the oscillation should be low
			//unless the timestep is really large.
			//There's not really an obvious solution to this without setting the denominator offset higher 
			//(or decreasing the timestep). Setting the denominator higher affects handling everywhere so 
			//settling for a potential temporary oscillation is probably the least worst compromise.
			if (useLegacyLongSlipCalculation)
			{
				lngSlip = (wheelSpeed - longSpeed) / (PxMax(longSpeedAbs, wheelSpeedAbs) + minActiveLongSlipDenominator);
			}
			else
			{
				lngSlip = (wheelSpeed - longSpeed) / (longSpeedAbs + minActiveLongSlipDenominator);
			}
		}
		else
		{
			//Wheel not experiencing an applied torque.
			//If the denominator becomes too small then the longSlip becomes large and the longitudinal force
			//can overshoot zero at large timesteps.  This can be really noticeable so it's harder to justify 
			//not taking action.  Further, the car isn't being actually driven so there is a strong case to fiddle
			//with the denominator because it doesn't really affect active handling.
			//Don't let the denominator fall below a user-specified value.  This can be tuned upwards until the  
			//oscillation in the sign of longSlip disappears.
			if (useLegacyLongSlipCalculation)
			{
				lngSlip = (wheelSpeed - longSpeed) / (PxMax(minPassiveLongSlipDenominator, PxMax(longSpeedAbs, wheelSpeedAbs)));
			}
			else
			{
				lngSlip = (wheelSpeed - longSpeed) / (longSpeedAbs + minPassiveLongSlipDenominator);
			}
		}
	}
	return lngSlip;
} 

void PxVehicleTireSlipSpeedsUpdate
(const PxVehicleWheelParams& whlParams, const PxVehicleSuspensionParams& suspParams,
 const PxF32 steerAngle, const PxVehicleSuspensionState& suspStates, const PxVehicleTireDirectionState& trSlipDirs,
 const PxVehicleRigidBodyState& rigidBodyState, const PxVehicleRoadGeometryState& roadGeometryState,
 const PxVehicleFrame& frame,
 PxVehicleTireSpeedState& trSpeedState)
{
	trSpeedState.setToDefault();

	//Compute the position of the bottom of the wheel (placed on the ground plane).
	PxVec3 wheelBottomPos;
	{
		PxVec3 v,w;
		PxF32 dist;
		PxVehicleComputeSuspensionRaycast(frame, whlParams, suspParams, steerAngle, rigidBodyState.pose, v, w, dist);
		wheelBottomPos = v + w*(dist - suspStates.jounce);
	}

	//Compute the  rigid body velocity at the bottom of the wheel (placed on the ground plane).
	PxVec3 wheelBottomVel;
	{
		const PxVec3 r = wheelBottomPos - rigidBodyState.pose.p;
		wheelBottomVel = rigidBodyState.linearVelocity + rigidBodyState.angularVelocity.cross(r) - roadGeometryState.velocity;
	}

	//Comput the velocities along lateral and longitudinal tire directions.
	trSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL] = wheelBottomVel.dot(trSlipDirs.directions[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	trSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL] = wheelBottomVel.dot(trSlipDirs.directions[PxVehicleTireDirectionModes::eLATERAL]);
}


void vehicleTireSlipsUpdate
(const PxVehicleWheelParams& whlParams,
 const PxVehicleTireSlipParams& trSlipParams, const bool useLegacyLongSlipCalculation,
 const PxVehicleWheelActuationState& actState, PxVehicleTireSpeedState& trSpeedState, const PxVehicleWheelRigidBody1dState& whlRigidBody1dState,
 PxVehicleTireSlipState& trSlipState)
{
	trSlipState.setToDefault();

	PxF32 latSlip = 0.0f;
	{
		const PxF32 lngSpeed = trSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL];
		const PxF32 latSpeed = trSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL];
		const PxF32 minLatSlipDenominator = trSlipParams.minLatSlipDenominator;
		latSlip = computeLateralSlip(lngSpeed, latSpeed, minLatSlipDenominator);
	}

	PxF32 lngSlip = 0.0f;
	{
		const PxF32 lngSpeed = trSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL];
		const PxF32 wheelRotSpeed = whlRigidBody1dState.rotationSpeed;
		const PxF32 wheelRadius = whlParams.radius;
		const PxF32 minPassiveLngSlipDenominator = trSlipParams.minPassiveLongSlipDenominator;
		const PxF32 minActiveLngSlipDenominator = trSlipParams.minActiveLongSlipDenominator;
		const bool isBrakeApplied = actState.isBrakeApplied;
		const bool isAccelApplied = actState.isDriveApplied;
		lngSlip = computeLongitudionalSlip(
			useLegacyLongSlipCalculation, 
			lngSpeed, wheelRotSpeed, wheelRadius, 
			minPassiveLngSlipDenominator, minActiveLngSlipDenominator, 
			isAccelApplied, isBrakeApplied);
	}

	trSlipState.slips[PxVehicleTireDirectionModes::eLATERAL] = latSlip;
	trSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL] = lngSlip;
}

void PxVehicleTireSlipsUpdate
(const PxVehicleWheelParams& whlParams,
 const PxVehicleTireSlipParams& trSlipParams,
 const PxVehicleWheelActuationState& actState, PxVehicleTireSpeedState& trSpeedState, const PxVehicleWheelRigidBody1dState& whlRigidBody1dState,
 PxVehicleTireSlipState& trSlipState)
{
	vehicleTireSlipsUpdate(
		whlParams, trSlipParams, false,
		actState, trSpeedState, whlRigidBody1dState,
		trSlipState);
}

void PxVehicleTireSlipsLegacyUpdate
(const PxVehicleWheelParams& whlParams,
 const PxVehicleTireSlipParams& trSlipParams,
 const PxVehicleWheelActuationState& actState, PxVehicleTireSpeedState& trSpeedState, const PxVehicleWheelRigidBody1dState& whlRigidBody1dState,
 PxVehicleTireSlipState& trSlipState)
{
	vehicleTireSlipsUpdate(
		whlParams, trSlipParams, true,
		actState, trSpeedState, whlRigidBody1dState,
		trSlipState);

}

void PxVehicleTireCamberAnglesUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxReal steerAngle, const PxVec3& groundNormal, bool isWheelOnGround,
 const PxVehicleSuspensionComplianceState& compState,
 const PxVehicleRigidBodyState& rigidBodyState,
 const PxVehicleFrame& frame,
 PxVehicleTireCamberAngleState& trCamberAngleState)
{
	trCamberAngleState.setToDefault();

	//Use zero camber if the suspension could not push the wheel to the ground.
	if (!isWheelOnGround)
		return;

	//Compute the wheel quaternion in the world frame.
	//Ignore rotation due to radial symmetry.
	const PxQuat wheelOrientation = PxVehicleComputeWheelOrientation(frame, suspParams, compState.camber, compState.toe, steerAngle, 
		rigidBodyState.pose.q, 0.0f);

	//Compute the axes of the wheel.
	const PxVec3 latDir = wheelOrientation.rotate(frame.getLatAxis());
	const PxVec3 lngDir = wheelOrientation.rotate(frame.getLngAxis());

	//Project normal into lateral/vertical plane.
	//Start with:
	//n = lat*alpha + lng*beta + vrt*delta
	//Want to work out 
	//T = n - lng*beta 
	//  = n - lng*(n.dot(lng))
	//Don't forget to normalise T.
	//For the angle theta to look for we have:
	//T.vrtDir = cos(theta)
	//However, the cosine destroys the sign of the angle, thus we
	//use:
	//T.latDir = cos(pi/2 - theta) = sin(theta)  (latDir and vrtDir are perpendicular)
	const PxF32 beta = groundNormal.dot(lngDir);
	PxVec3 T = groundNormal - lngDir * beta;
	T.normalize();
	const PxF32 sinTheta = T.dot(latDir);
	const PxF32 theta = PxAsin(sinTheta);
	trCamberAngleState.camberAngle = theta;
}

PX_FORCE_INLINE PxF32 updateLowLngSpeedTimer
(const PxF32 lngSpeed, const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 lngThresholdSpeed, 
 const bool isIntentionToAccelerate, const PxF32 dt, const PxF32 lowLngSpeedTime)
{
	//If the tire is rotating slowly and the longitudinal speed is slow then increment the slow longitudinal speed timer.
	//If the intention of the driver is to accelerate the vehicle then reset the timer because the intention has been signalled NOT to bring 
	//the wheel to rest.
	PxF32 newLowSpeedTime = 0.0f;
	if ((PxAbs(lngSpeed) < lngThresholdSpeed) && (PxAbs(wheelOmega*wheelRadius) < lngThresholdSpeed) && !isIntentionToAccelerate)
	{
		newLowSpeedTime = lowLngSpeedTime + dt;
	}
	else
	{
		newLowSpeedTime = 0;
	}
	return newLowSpeedTime;
}

PX_FORCE_INLINE void activateStickyFrictionLngConstraint
(const PxF32 longSpeed, const PxF32 wheelOmega, const PxF32 lowLngSpeedTime, const bool isIntentionToBrake, 
 const PxF32 thresholdSpeed, const PxF32 thresholdTime,
 bool& stickyTireActiveFlag)
{
	//Setup the sticky friction constraint to bring the vehicle to rest at the tire contact point.
	//The idea here is to resolve the singularity of the tire long slip at low vz by replacing the long force with a velocity constraint.
	//Only do this if we can guarantee that the intention is to bring the car to rest (no accel pedal applied).
	//We're going to replace the longitudinal tire force with the sticky friction so set the long slip to zero to ensure zero long force.
	//Apply sticky friction to this tire if 
	//(1) the wheel is locked (this means the brake/handbrake must be on) and the longitudinal speed at the tire contact point is vanishingly small.
	//(2) the accumulated time of low longitudinal speed is greater than a threshold.
	stickyTireActiveFlag = false;
	if (((PxAbs(longSpeed) < thresholdSpeed) && (0.0f == wheelOmega) && isIntentionToBrake) || (lowLngSpeedTime > thresholdTime))
	{
		stickyTireActiveFlag = true;
	}
}

PX_FORCE_INLINE PxF32 updateLowLatSpeedTimer
(const PxF32 latSpeed, const bool isIntentionToAccelerate, const PxF32 timestep, const PxF32 thresholdSpeed, const PxF32 lowSpeedTime)
{
	//If the lateral speed is slow then increment the slow lateral speed timer.
	//If the intention of the driver is to accelerate the vehicle then reset the timer because the intention has been signalled NOT to bring 
	//the wheel to rest.
	PxF32 newLowSpeedTime = lowSpeedTime;
	if ((PxAbs(latSpeed) < thresholdSpeed) && !isIntentionToAccelerate)
	{
		newLowSpeedTime += timestep;
	}
	else
	{
		newLowSpeedTime = 0;
	}
	return newLowSpeedTime;
}

PX_FORCE_INLINE void activateStickyFrictionLatConstraint
(const bool lowSpeedLngTimerActive, 
 const PxF32 lowLatSpeedTimer, const PxF32 thresholdTime,
 bool& stickyTireActiveFlag)
{
	//Setup the sticky friction constraint to bring the vehicle to rest at the tire contact point.
	//Only do this if we can guarantee that the intention is to bring the car to rest (no accel pedal applied).
	//We're going to replace the lateral tire force with the sticky friction so set the lat slip to zero to ensure zero lat force.
	//Apply sticky friction to this tire if 
	//(1) the low longitudinal speed timer is > 0.
	//(2) the accumulated time of low longitudinal speed is greater than a threshold.
	stickyTireActiveFlag = false;
	if (lowSpeedLngTimerActive && (lowLatSpeedTimer > thresholdTime))
	{
		stickyTireActiveFlag = true;
	}
}

void PxVehicleTireStickyStateUpdate
(const PxVehicleAxleDescription& axleDescription, const PxVehicleWheelParams& whlParams,
 const PxVehicleTireStickyParams& trStickyParams,
 const PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
 const PxVehicleTireGripState& trGripState, const PxVehicleTireSpeedState& trSpeedState, const PxVehicleWheelRigidBody1dState& whlState,
 const PxReal dt,
 PxVehicleTireStickyState& trStickyState)
{
	trStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] = false;
	trStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL] = false;

	//Only process sticky state if tire can generate force.
	const PxF32 load = trGripState.load;
	const PxF32 friction = trGripState.friction;
	if(0 == load*friction)
	{
		trStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLONGITUDINAL] = 0.0f;
		trStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLATERAL] = 0.0f;
		return;
	}

	//Work out if any wheel is to have a drive applied to it.
	bool isIntentionToAccelerate = false;
	bool isIntentionToBrake = false;
	for(PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
		if(actuationStates[wheelId].isDriveApplied)
			isIntentionToAccelerate = true;
		if (actuationStates[wheelId].isBrakeApplied)
			isIntentionToBrake = true;
	}

	//Long sticky state.
	bool lngTimerActive = false;
	{
		const PxF32 lngSpeed = trSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL];
		const PxF32 wheelOmega = whlState.rotationSpeed;
		const PxF32 wheelRadius = whlParams.radius;
		const PxF32 lngThresholdSpeed = trStickyParams.stickyParams[PxVehicleTireDirectionModes::eLONGITUDINAL].thresholdSpeed;
		const PxF32 lngLowSpeedTime = trStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLONGITUDINAL];
						
		const PxF32 newLowLngSpeedTime = updateLowLngSpeedTimer(
			lngSpeed, wheelOmega, wheelRadius, lngThresholdSpeed, 
			isIntentionToAccelerate, 
			dt, lngLowSpeedTime);
		lngTimerActive = (newLowLngSpeedTime > 0);

		bool lngActiveState = false;
		const PxF32 lngThresholdTime = trStickyParams.stickyParams[PxVehicleTireDirectionModes::eLONGITUDINAL].thresholdTime;
		activateStickyFrictionLngConstraint(
			lngSpeed, wheelOmega, newLowLngSpeedTime, isIntentionToBrake, 
			lngThresholdSpeed, lngThresholdTime,
			lngActiveState);

		trStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLONGITUDINAL] = newLowLngSpeedTime;
		trStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] = lngActiveState;
	}

	//Lateral sticky state
	{
		const PxF32 latSpeed = trSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL];
		const PxF32 latThresholdSpeed = trStickyParams.stickyParams[PxVehicleTireDirectionModes::eLATERAL].thresholdSpeed;
		const PxF32 latLowSpeedTime = trStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLATERAL];
		const PxF32 latNewLowSpeedTime = updateLowLatSpeedTimer(latSpeed, isIntentionToAccelerate, dt, latThresholdSpeed, latLowSpeedTime); 

		bool latActiveState = false;
		const PxF32 latThresholdTime = trStickyParams.stickyParams[PxVehicleTireDirectionModes::eLATERAL].thresholdTime;
		activateStickyFrictionLatConstraint(lngTimerActive, latNewLowSpeedTime, latThresholdTime,
			latActiveState); 

		trStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLATERAL] = latNewLowSpeedTime;
		trStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL] = latActiveState;
	}
}

PX_FORCE_INLINE PxF32 computeTireLoad
(const PxVehicleTireForceParams& trForceParams, const PxVehicleSuspensionForce& suspForce)
{
	//Compute the normalised load.
	const PxF32 rawLoad = suspForce.normalForce;
	const PxF32 restLoad = trForceParams.restLoad;
	const PxF32 normalisedLoad = rawLoad / restLoad;

	//Get the load filter params.
	const PxF32 minNormalisedLoad = trForceParams.loadFilter[0][0];
	const PxF32 minFilteredNormalisedLoad = trForceParams.loadFilter[0][1];
	const PxF32 maxNormalisedLoad = trForceParams.loadFilter[1][0];
	const PxF32 maxFilteredNormalisedLoad = trForceParams.loadFilter[1][1];

	//Compute the filtered load.
	const PxF32 filteredNormalisedLoad = computeFilteredNormalisedTireLoad(minNormalisedLoad, minFilteredNormalisedLoad, maxNormalisedLoad, maxFilteredNormalisedLoad, normalisedLoad);
	const PxF32 filteredLoad = restLoad * filteredNormalisedLoad;

	//Set the load after applying the filter.
	return filteredLoad;
}

PX_FORCE_INLINE PxF32 computeTireFriction
(const PxVehicleTireForceParams& trForceParams, 
 const PxReal frictionCoefficient, const PxVehicleTireSlipState& tireSlipState)
{
	//Interpolate the friction using the long slip value.
	const PxF32 x0 = trForceParams.frictionVsSlip[0][0];
	const PxF32 y0 = trForceParams.frictionVsSlip[0][1];
	const PxF32 x1 = trForceParams.frictionVsSlip[1][0];
	const PxF32 y1 = trForceParams.frictionVsSlip[1][1];
	const PxF32 x2 = trForceParams.frictionVsSlip[2][0];
	const PxF32 y2 = trForceParams.frictionVsSlip[2][1];
	const PxF32 longSlipAbs = PxAbs(tireSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	PxF32 mu;
	if (longSlipAbs < x1)
	{
		mu = y0 + (y1 - y0)*(longSlipAbs - x0) / (x1 - x0);
	}
	else if (longSlipAbs < x2)
	{
		mu = y1 + (y2 - y1)*(longSlipAbs - x1) / (x2 - x1);
	}
	else
	{
		mu = y2;
	}
	PX_ASSERT(mu >= 0);

	const PxF32 tireFriction = frictionCoefficient * mu;
	return tireFriction;
}

void PxVehicleTireGripUpdate
(const PxVehicleTireForceParams& trForceParams,
 const PxReal frictionCoefficient, bool isWheelOnGround, const PxVehicleSuspensionForce& suspForce,
 const PxVehicleTireSlipState& trSlipState,
 PxVehicleTireGripState& trGripState)
{
	trGripState.setToDefault();

	//If the wheel is not touching the ground then carry on with zero grip state.
	if (!isWheelOnGround)
		return;

	//Compute load and friction.
	trGripState.load = computeTireLoad(trForceParams, suspForce);
	trGripState.friction = computeTireFriction(trForceParams, frictionCoefficient, trSlipState);
}

void PxVehicleTireSlipsAccountingForStickyStatesUpdate
(const PxVehicleTireStickyState& trStickyState,
 PxVehicleTireSlipState& trSlipState)
{
	if(trStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL])
		trSlipState.slips[PxVehicleTireDirectionModes::eLATERAL] = 0.f;
	if (trStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL])
		trSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL] = 0.f;
}


////////////////////////////////////////////////////////////////////////////
//Default tire force shader function.
//Taken from Michigan tire model.
//Computes tire long and lat forces plus the aligning moment arising from 
//the lat force and the torque to apply back to the wheel arising from the 
//long force (application of Newton's 3rd law).
////////////////////////////////////////////////////////////////////////////


#define ONE_TWENTYSEVENTH 0.037037f
#define ONE_THIRD 0.33333f
PX_FORCE_INLINE PxF32 smoothingFunction1(const PxF32 K)
{
	//Equation 20 in CarSimEd manual Appendix F.
	//Looks a bit like a curve of sqrt(x) for 0<x<1 but reaching 1.0 on y-axis at K=3. 
	PX_ASSERT(K >= 0.0f);
	return PxMin(1.0f, K - ONE_THIRD * K*K + ONE_TWENTYSEVENTH * K*K*K);
}
PX_FORCE_INLINE PxF32 smoothingFunction2(const PxF32 K)
{
	//Equation 21 in CarSimEd manual Appendix F.
	//Rises to a peak at K=0.75 and falls back to zero by K=3
	PX_ASSERT(K >= 0.0f);
	return (K - K * K + ONE_THIRD * K*K*K - ONE_TWENTYSEVENTH * K*K*K*K);
}

void computeTireForceMichiganModel
(const PxVehicleTireForceParams& tireData,
 const PxF32 tireFriction,
 const PxF32 longSlipUnClamped, const PxF32 latSlipUnClamped, const PxF32 camberUnclamped,
 const PxF32 wheelRadius, 
 const PxF32 tireLoad,
 PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
{
	PX_ASSERT(tireFriction > 0);
	PX_ASSERT(tireLoad > 0);

	wheelTorque = 0.0f;
	tireLongForceMag = 0.0f;
	tireLatForceMag = 0.0f;
	tireAlignMoment = 0.0f;

	//Clamp the slips to a minimum value.
	const PxF32 minimumSlipThreshold = 1e-5f;
	const PxF32 latSlip = PxAbs(latSlipUnClamped) >= minimumSlipThreshold ? latSlipUnClamped : 0.0f;
	const PxF32 longSlip = PxAbs(longSlipUnClamped) >= minimumSlipThreshold ? longSlipUnClamped : 0.0f;
	const PxF32 camber = PxAbs(camberUnclamped) >= minimumSlipThreshold ? camberUnclamped : 0.0f;

	//Normalise the tire load.
	const PxF32 restTireLoad = tireData.restLoad;
	const PxF32 normalisedTireLoad = tireLoad/ restTireLoad;

	//Compute the lateral stiffness
	const PxF32 latStiff = (0.0f == tireData.latStiffX) ? tireData.latStiffY : tireData.latStiffY*smoothingFunction1(normalisedTireLoad*3.0f / tireData.latStiffX);

	//Get the longitudinal stiffness
	const PxF32 longStiff = tireData.longStiff;

	//Get the camber stiffness.
	const PxF32 camberStiff = tireData.camberStiff;

	//If long slip/lat slip/camber are all zero than there will be zero tire force.
	if ((0 == latSlip*latStiff) && (0 == longSlip*longStiff) && (0 == camber*camberStiff))
	{
		return;
	}

	//Carry on and compute the forces.
	const PxF32 TEff = PxTan(latSlip + (camber * camberStiff / latStiff));  // "+" because we define camber stiffness as a positive value
	const PxF32 K = PxSqrt(latStiff*TEff*latStiff*TEff + longStiff * longSlip*longStiff*longSlip) / (tireFriction*tireLoad);
	//const PxF32 KAbs=PxAbs(K);
	PxF32 FBar = smoothingFunction1(K);//K - ONE_THIRD*PxAbs(K)*K + ONE_TWENTYSEVENTH*K*K*K;
	PxF32 MBar = smoothingFunction2(K); //K - KAbs*K + ONE_THIRD*K*K*K - ONE_TWENTYSEVENTH*KAbs*K*K*K;
	//Mbar = PxMin(Mbar, 1.0f);
	PxF32 nu = 1;
	if (K <= 2.0f*PxPi)
	{
		const PxF32 latOverlLong = latStiff/longStiff;
		nu = 0.5f*(1.0f + latOverlLong - (1.0f - latOverlLong)*PxCos(K*0.5f));
	}
	const PxF32 FZero = tireFriction * tireLoad / (PxSqrt(longSlip*longSlip + nu * TEff*nu*TEff));
	const PxF32 fz = longSlip * FBar*FZero;
	const PxF32 fx = -nu * TEff*FBar*FZero;
	//TODO: pneumatic trail.
	const PxF32 pneumaticTrail = 1.0f;
	const PxF32	fMy = nu * pneumaticTrail * TEff * MBar * FZero;

	//We can add the torque to the wheel.
	wheelTorque = -fz * wheelRadius;
	tireLongForceMag = fz;
	tireLatForceMag = fx;
	tireAlignMoment = fMy;
}

void PxVehicleTireForcesUpdate
(const PxVehicleWheelParams& whlParams, const PxVehicleSuspensionParams& suspParams,
 const PxVehicleTireForceParams& trForceParams,
 const PxVehicleSuspensionComplianceState& compState,
 const PxVehicleTireGripState& trGripState, const PxVehicleTireDirectionState& trDirectionState, 
 const PxVehicleTireSlipState& trSlipState, const PxVehicleTireCamberAngleState& cmbAngleState,
 const PxVehicleRigidBodyState& rigidBodyState,
 PxVehicleTireForce& trForce)
{
	trForce.setToDefault();

	//If the tire can generate no force then carry on with zero force.
	if(0 == trGripState.friction*trGripState.load)
		return;

	PxF32 wheelTorque = 0;
	PxF32 tireLongForceMag = 0;
	PxF32 tireLatForceMag = 0;
	PxF32 tireAlignMoment = 0;

	const PxF32 friction = trGripState.friction;
	const PxF32 tireLoad = trGripState.load;
	const PxF32 lngSlip = trSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL];
	const PxF32 latSlip = trSlipState.slips[PxVehicleTireDirectionModes::eLATERAL];
	const PxF32 camber = cmbAngleState.camberAngle;
	const PxF32 wheelRadius = whlParams.radius;
	computeTireForceMichiganModel(trForceParams, friction, lngSlip, latSlip, camber, wheelRadius, tireLoad,
		wheelTorque, tireLongForceMag, tireLatForceMag, tireAlignMoment);

	//Compute the forces.
	const PxVec3 tireLongForce = trDirectionState.directions[PxVehicleTireDirectionModes::eLONGITUDINAL]*tireLongForceMag;
	const PxVec3 tireLatForce = trDirectionState.directions[PxVehicleTireDirectionModes::eLATERAL]*tireLatForceMag;

	//Compute the torques.
	const PxVec3 r = rigidBodyState.pose.rotate(suspParams.suspensionAttachment.transform(compState.tireForceAppPoint));
	const PxVec3 tireLongTorque = r.cross(tireLongForce);
	const PxVec3 tireLatTorque = r.cross(tireLatForce);

	//Set the torques.
	trForce.forces[PxVehicleTireDirectionModes::eLONGITUDINAL] = tireLongForce;
	trForce.torques[PxVehicleTireDirectionModes::eLONGITUDINAL] = tireLongTorque;
	trForce.forces[PxVehicleTireDirectionModes::eLATERAL] = tireLatForce;
	trForce.torques[PxVehicleTireDirectionModes::eLATERAL] = tireLatTorque;
	trForce.aligningMoment = tireAlignMoment;
	trForce.wheelTorque = wheelTorque;
}

} //namespace vehicle2
} //namespace physx

