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

#include "foundation/PxMemory.h"

#include "vehicle2/PxVehicleParams.h"
#include "vehicle2/PxVehicleFunctions.h"
#include "vehicle2/PxVehicleMaths.h"

#include "vehicle2/commands/PxVehicleCommandHelpers.h"

#include "vehicle2/drivetrain/PxVehicleDrivetrainFunctions.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainParams.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainStates.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainHelpers.h"

#include "vehicle2/tire/PxVehicleTireStates.h"

#include "vehicle2/wheel/PxVehicleWheelParams.h"
#include "vehicle2/wheel/PxVehicleWheelStates.h"

namespace physx
{
namespace vehicle2
{

void PxVehicleDirectDriveThrottleCommandResponseUpdate
(const PxReal throttle, const PxVehicleDirectDriveTransmissionCommandState& transmissionCommands, const PxReal longitudinalSpeed,
 const PxU32 wheelId, const PxVehicleDirectDriveThrottleCommandResponseParams& responseParams,
 PxReal& throttleResponse)
{
	//The gearing decides how we will multiply the response.
	PxF32 gearMultiplier = 0.0f;
	switch (transmissionCommands.gear)
	{
	case PxVehicleDirectDriveTransmissionCommandState::eREVERSE:
		gearMultiplier = -1.0f;
		break;
	case PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL:
		gearMultiplier = 0.0f;
		break;
	case PxVehicleDirectDriveTransmissionCommandState::eFORWARD:
		gearMultiplier = 1.0f;
		break;
	}
	throttleResponse = gearMultiplier * PxVehicleNonLinearResponseCompute(throttle, longitudinalSpeed, wheelId, responseParams);
}

void PxVehicleDirectDriveActuationStateUpdate
(const PxReal brakeTorque, const PxReal driveTorque, 
 PxVehicleWheelActuationState& actState)
{
	PxMemZero(&actState, sizeof(PxVehicleWheelActuationState));
	actState.isBrakeApplied = (brakeTorque != 0.0f);
	actState.isDriveApplied = (driveTorque != 0.0f);
}

void PxVehicleDirectDriveUpdate
(const PxVehicleWheelParams& whlParams,
 const PxVehicleWheelActuationState& actState, 
 const PxReal brkTorque, const PxReal drvTorque, const PxVehicleTireForce& trForce,
 const PxF32 dt, 
 PxVehicleWheelRigidBody1dState& whlState)
{

	//w(t+dt) = w(t) + (1/inertia)*(brakeTorque + driveTorque + tireTorque)*dt - (1/inertia)*damping*w(t)*dt )	(1)
	//Apply implicit trick and rearrange.
	//w(t+dt)[1 + (1/inertia)*damping*dt] = w(t) + (1/inertia)*(brakeTorque + driveTorque + tireTorque)*dt		(2)
	const PxF32 wheelRotSpeed = whlState.rotationSpeed;
	const PxF32 dtOverMOI = dt/whlParams.moi;
	const PxF32 tireTorque = trForce.wheelTorque;
	const PxF32 brakeTorque = -brkTorque*PxVehicleComputeSign(wheelRotSpeed);
	const PxF32 driveTorque = drvTorque;
	const PxF32 wheelDampingRate = whlParams.dampingRate;

	//Integrate the wheel rotation speed.
	const PxF32 newRotSpeedNoBrakelock = (wheelRotSpeed + dtOverMOI*(tireTorque + driveTorque + brakeTorque)) / (1.0f + wheelDampingRate*dtOverMOI);
	//If the brake is applied and the sign flipped then lock the brake.
	const bool isBrakeApplied = actState.isBrakeApplied;
	const PxF32 newRotSpeedWithBrakelock = (isBrakeApplied && ((wheelRotSpeed*newRotSpeedNoBrakelock) <= 0)) ? 0.0f : newRotSpeedNoBrakelock;

	whlState.rotationSpeed = newRotSpeedWithBrakelock;
}

void PxVehicleDifferentialStateUpdate
(const PxVehicleFourWheelDriveDifferentialLegacyParams& diffParams,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelStates,
 PxVehicleDifferentialState& diffState)
{
	diffState.setToDefault();

	//4 wheels are connected.
	diffState.connectedWheels[0] = diffParams.frontWheelIds[0];
	diffState.connectedWheels[1] = diffParams.frontWheelIds[1];
	diffState.connectedWheels[2] = diffParams.rearWheelIds[0];
	diffState.connectedWheels[3] = diffParams.rearWheelIds[1];
	diffState.nbConnectedWheels = 4;

	//Compute the contributions to average speed and ratios of available torque.
	PxVehicleLegacyDifferentialWheelSpeedContributionsCompute(diffParams, PxVehicleLimits::eMAX_NB_WHEELS,
		diffState.aveWheelSpeedContributionAllWheels);

	PxVehicleLegacyDifferentialTorqueRatiosCompute(diffParams, wheelStates, PxVehicleLimits::eMAX_NB_WHEELS,
		diffState.torqueRatiosAllWheels);
}

PX_FORCE_INLINE PxF32 computeTargetRatio
(const PxF32 target, const PxF32 dt, const PxF32 strength, const PxF32 ratio)
{
	const PxF32 targ = (PX_VEHICLE_FOUR_WHEEL_DIFFERENTIAL_MAXIMUM_STRENGTH == strength) ? target : PxMax(target, ratio - strength * dt);
	return targ;
}

void PxVehicleDifferentialStateUpdate
(const PxVehicleAxleDescription& axleDescription, const PxVehicleFourWheelDriveDifferentialParams& diffParams,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelStates,
 const PxReal dt,
 PxVehicleDifferentialState& diffState, PxVehicleWheelConstraintGroupState& wheelConstraintGroupState)
{
	diffState.setToDefault();
	wheelConstraintGroupState.setToDefault();

	PxU32 nbConnectedWheels = 0;
	for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
		if (diffParams.torqueRatios[wheelId] != 0.0f)
		{
			diffState.connectedWheels[nbConnectedWheels] = wheelId;
			diffState.aveWheelSpeedContributionAllWheels[wheelId] = diffParams.aveWheelSpeedRatios[wheelId];
			diffState.torqueRatiosAllWheels[wheelId] = diffParams.aveWheelSpeedRatios[wheelId];
			nbConnectedWheels++;
		}
	}
	diffState.nbConnectedWheels = nbConnectedWheels;

	if (0.0f == diffParams.frontBias && 0.0f == diffParams.rearBias && 0.0f == diffParams.centerBias)
		return;

	if (0.0f == diffParams.rate)
		return;

	bool frontBiasBreached = false;
	PxF32 Rf = 0.0f;
	PxF32 Tf = 0.0f;
	{
		const PxF32 bias = diffParams.frontBias;
		if (bias >= 1.0f)
		{
			const PxU32 wheel0 = diffParams.frontWheelIds[0];
			const PxU32 wheel1 = diffParams.frontWheelIds[1];
			const PxReal w0 = wheelStates[wheel0].rotationSpeed;
			const PxReal w1 = wheelStates[wheel1].rotationSpeed;
			const PxReal aw0 = PxAbs(w0);
			const PxReal aw1 = PxAbs(w1);
			const PxReal sw0 = PxVehicleComputeSign(w0);
			const PxReal sw1 = PxVehicleComputeSign(w1);
			if ((w0 != 0.0f) && (sw0 == sw1))
			{
				const PxF32 ratio = PxMax(aw0, aw1) / PxMin(aw0, aw1);
				frontBiasBreached = (ratio > bias);
				//const PxF32 target = diffParams.frontTarget + (1.0f - diffParams.strength) * (ratio - diffParams.frontTarget);
				const PxF32 target = computeTargetRatio(diffParams.frontTarget, dt, diffParams.rate, ratio);
				Tf = (aw0 > aw1) ? 1.0f / target : target;
				Rf = aw1 / aw0;
			}
		}
	}

	bool rearBiasBreached = false;
	PxF32 Rr = 0.0f;
	PxF32 Tr = 0.0f;
	{
		const PxF32 bias = diffParams.rearBias;
		if (bias >= 1.0f)
		{
			const PxU32 wheel2 = diffParams.rearWheelIds[0];
			const PxU32 wheel3 = diffParams.rearWheelIds[1];
			const PxReal w2 = wheelStates[wheel2].rotationSpeed;
			const PxReal w3 = wheelStates[wheel3].rotationSpeed;
			const PxReal aw2 = PxAbs(w2);
			const PxReal aw3 = PxAbs(w3);
			const PxReal sw2 = PxVehicleComputeSign(w2);
			const PxReal sw3 = PxVehicleComputeSign(w3);
			if ((w2 != 0.0f) && (sw2 == sw3))
			{
				const PxF32 ratio = PxMax(aw2, aw3) / PxMin(aw2, aw3);
				rearBiasBreached = (ratio > bias);
				//const PxF32 target = diffParams.rearTarget + (1.0f - diffParams.strength) * (ratio - diffParams.rearTarget);
				const PxF32 target = computeTargetRatio(diffParams.rearTarget, dt, diffParams.rate, ratio);
				Tr = (aw2 > aw3) ? 1.0f / target : target;
				Rr = aw3 / aw2;
			}
		}
	}

	bool centreBiasBrached = false;
	//PxF32 Rc = 0.0f;
	PxF32 Tc = 0.0f;
	{
		const PxF32 bias = diffParams.centerBias;
		if(bias >= 1.0f)
		{
			const PxU32 wheel0 = diffParams.frontWheelIds[0];
			const PxU32 wheel1 = diffParams.frontWheelIds[1];
			const PxU32 wheel2 = diffParams.rearWheelIds[0];
			const PxU32 wheel3 = diffParams.rearWheelIds[1];
			const PxReal w0 = wheelStates[wheel0].rotationSpeed;
			const PxReal w1 = wheelStates[wheel1].rotationSpeed;
			const PxReal w2 = wheelStates[wheel2].rotationSpeed;
			const PxReal w3 = wheelStates[wheel3].rotationSpeed;
			const PxReal aw0 = PxAbs(w0);
			const PxReal aw1 = PxAbs(w1);
			const PxReal aw2 = PxAbs(w2);
			const PxReal aw3 = PxAbs(w3);
			const PxReal sw0 = PxVehicleComputeSign(w0);
			const PxReal sw1 = PxVehicleComputeSign(w1);
			const PxReal sw2 = PxVehicleComputeSign(w2);
			const PxReal sw3 = PxVehicleComputeSign(w3);
			if ((w0 != 0.0f) && (sw0 == sw1) && (sw0 == sw2) && (sw0 == sw3))
			{
				const PxF32 ratio = PxMax(aw0 + aw1, aw2 + aw3) / PxMin(aw0 + aw1, aw2 + aw3);
				centreBiasBrached = (ratio > bias);
				//const PxF32 target = diffParams.centerTarget + (1.0f - diffParams.strength) * (ratio - diffParams.centerTarget);
				const PxF32 target = computeTargetRatio(diffParams.centerTarget, dt, diffParams.rate, ratio);
				Tc = ((aw0 + aw1) > (aw2 + aw3)) ? 1.0f / target : target;
				//Rc = (aw2 + aw3) / (aw0 + aw1);
			}
		}
	}

	if (centreBiasBrached)
	{
		PxF32 f = 0.0f;
		PxF32 r = 0.0f;
		if (frontBiasBreached && rearBiasBreached)
		{
			f = Tf;
			r = Tr;
		}
		else if (frontBiasBreached)
		{
			f = Tf;
			r = Rr;
		}
		else if (rearBiasBreached)
		{
			f = Rf;
			r = Tr;
		}
		else
		{
			f = Rf;
			r = Rr;
		}

		const PxU32  wheel0 = diffParams.frontWheelIds[0];
		const PxU32  wheel1 = diffParams.frontWheelIds[1];
		const PxU32  wheel2 = diffParams.rearWheelIds[0];
		const PxU32  wheel3 = diffParams.rearWheelIds[1];
		const PxU32 wheelIds[4] = { wheel0, wheel1, wheel2,  wheel3 };
		const PxF32 constraintMultipliers[4] = { 1.0f, f, Tc*(1 + f) / (1 + r), r*Tc*(1 + f) / (1 + r) };
		wheelConstraintGroupState.addConstraintGroup(4, wheelIds, constraintMultipliers);
	}
	else
	{
		if (frontBiasBreached)
		{
			const PxU32* wheelIds = diffParams.frontWheelIds;
			const PxF32 constraintMultipliers[2] = { 1.0f, Tf };
			wheelConstraintGroupState.addConstraintGroup(2, wheelIds, constraintMultipliers);
		}

		if (rearBiasBreached)
		{
			const PxU32* wheelIds = diffParams.rearWheelIds;
			const PxF32 constraintMultipliers[2] = { 1.0f, Tr };
			wheelConstraintGroupState.addConstraintGroup(2, wheelIds, constraintMultipliers);
		}
	}
}

void PxVehicleDifferentialStateUpdate
(const PxVehicleAxleDescription& axleDescription, const PxVehicleMultiWheelDriveDifferentialParams& diffParams,
 PxVehicleDifferentialState& diffState)
{
	diffState.setToDefault();

	PxU32 nbConnectedWheels = 0;
	for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
		if (diffParams.torqueRatios[wheelId] != 0.0f)
		{
			diffState.connectedWheels[nbConnectedWheels] = wheelId;
			diffState.aveWheelSpeedContributionAllWheels[wheelId] = diffParams.aveWheelSpeedRatios[wheelId];
			diffState.torqueRatiosAllWheels[wheelId] = diffParams.aveWheelSpeedRatios[wheelId];
			nbConnectedWheels++;
		}
	}
	diffState.nbConnectedWheels = nbConnectedWheels;
}

void PxVehicleDifferentialStateUpdate
(const PxVehicleAxleDescription& axleDescription,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams, const PxVehicleTankDriveDifferentialParams& diffParams,
 const PxReal thrust0, PxReal thrust1,
 PxVehicleDifferentialState& diffState, PxVehicleWheelConstraintGroupState& wheelConstraintGroupState)
{
	diffState.setToDefault();
	wheelConstraintGroupState.setToDefault();

	//Store the tank track id for each wheel.
	//Store the thrust controller id for each wheel.
	//Store 0xffffffff for wheels not in a tank track.
	PxU32 tankTrackIds[PxVehicleLimits::eMAX_NB_WHEELS];
	PxMemSet(tankTrackIds, 0xff, sizeof(tankTrackIds));
	PxU32 thrustControllerIds[PxVehicleLimits::eMAX_NB_WHEELS];
	PxMemSet(thrustControllerIds, 0xff, sizeof(thrustControllerIds));
	for (PxU32 i = 0; i < diffParams.getNbTracks(); i++)
	{
		const PxU32 thrustControllerIndex = diffParams.getThrustControllerIndex(i);
		for (PxU32 j = 0; j < diffParams.getNbWheelsInTrack(i); j++)
		{
			const PxU32 wheelId = diffParams.getWheelInTrack(j, i);
			tankTrackIds[wheelId] = i;
			thrustControllerIds[wheelId] = thrustControllerIndex;
		}
	}

	//Treat every wheel connected to a tank track as connected to the differential but with zero torque split.
	//If a wheel is not connected to the differential but is connected to a tank track then we treat that as a connected wheel with 
	//zero drive torque applied.
	//We do this because we need to treat these wheels as being coupled to other wheels in the linear equations that solve engine+wheels
	PxU32 nbConnectedWheels = 0;
	const PxReal thrusts[2] = { thrust0, thrust1 };
	for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
		const PxU32 tankTrackId = tankTrackIds[wheelId];
		if (diffParams.torqueRatios[wheelId] != 0.0f && 0xffffffff != tankTrackId)
		{
			//Wheel connected to diff and in a tank track.
			//Modify the default torque split using the relevant thrust.
			const PxU32 thrustId = thrustControllerIds[wheelId];
			const PxF32 torqueSplitMultiplier = thrusts[thrustId];
			diffState.aveWheelSpeedContributionAllWheels[wheelId] = diffParams.aveWheelSpeedRatios[wheelId];
			diffState.torqueRatiosAllWheels[wheelId] = diffParams.torqueRatios[wheelId] * torqueSplitMultiplier;
			diffState.connectedWheels[nbConnectedWheels] = wheelId;
			nbConnectedWheels++;
		}
		else if (diffParams.torqueRatios[wheelId] != 0.0f)
		{
			//Wheel connected to diff but not in a tank track.
			//Use the default torque split.
			diffState.aveWheelSpeedContributionAllWheels[wheelId] = diffParams.aveWheelSpeedRatios[wheelId];
			diffState.torqueRatiosAllWheels[wheelId] = diffParams.torqueRatios[wheelId];
			diffState.connectedWheels[nbConnectedWheels] = wheelId;
			nbConnectedWheels++;
		}
		else if (0xffffffff != tankTrackId)
		{
			//Wheel not connected to diff but is in a tank track.
			//Zero torque split.
			diffState.aveWheelSpeedContributionAllWheels[wheelId] = 0.0f;
			diffState.torqueRatiosAllWheels[wheelId] = 0.0f;
			diffState.connectedWheels[nbConnectedWheels] = wheelId;
			nbConnectedWheels++;
		}
	}
	diffState.nbConnectedWheels = nbConnectedWheels;

	//Add each tank track as a constraint group.
	for (PxU32 i = 0; i < diffParams.getNbTracks(); i++)
	{
		const PxU32 nbWheelsInTrack = diffParams.getNbWheelsInTrack(i);
		if (nbWheelsInTrack >= 2)
		{
			const PxU32* wheelsInTrack = diffParams.wheelIdsInTrackOrder + diffParams.trackToWheelIds[i];

			PxF32 multipliers[PxVehicleLimits::eMAX_NB_WHEELS];

			const PxU32 wheelId0 = wheelsInTrack[0];
			const PxF32 wheelRadius0 = wheelParams[wheelId0].radius;

			//for (PxU32 j = 0; j < 1; j++)
			{
				//j = 0 is a special case with multiplier = 1.0
				multipliers[0] = 1.0f;
			}
			for (PxU32 j = 1; j < nbWheelsInTrack; j++)
			{
				const PxU32 wheelIdJ = wheelsInTrack[j];
				const PxF32 wheelRadiusJ = wheelParams[wheelIdJ].radius;
				multipliers[j] = wheelRadius0 / wheelRadiusJ;
			}

			wheelConstraintGroupState.addConstraintGroup(nbWheelsInTrack, wheelsInTrack, multipliers);
		}
	}
}

void PxVehicleEngineDriveActuationStateUpdate
(const PxVehicleAxleDescription& axleDescription,
 const PxVehicleGearboxParams& gearboxParams, 
 const PxVehicleArrayData<const PxReal>& brakeResponseStates,
 const PxVehicleEngineDriveThrottleCommandResponseState& throttleResponseState,
 const PxVehicleGearboxState& gearboxState, const PxVehicleDifferentialState& diffState, const PxVehicleClutchCommandResponseState& clutchResponseState,
 PxVehicleArrayData<PxVehicleWheelActuationState>& actuationStates)
{
	//Which wheels receive drive torque from the engine?
	const PxF32* diffTorqueRatios = diffState.torqueRatiosAllWheels;

	//Work out the clutch strength.
	//If the cutch strength is zero then none of the wheels receive drive torque from the engine.
	const PxF32 K = PxVehicleClutchStrengthCompute(clutchResponseState, gearboxParams, gearboxState);
		
	//Work out the applied throttle
	const PxF32 appliedThrottle = throttleResponseState.commandResponse;

	//Ready to set the boolean actuation state that is used to compute the tire slips. 
	//(Note: for a tire under drive or brake torque we compute the slip with a smaller denominator to accentuate the applied torque).
	//(Note: for a tire that is not under drive or brake torque we compute the sip with a larger denominator to smooth the vehicle slowing down). 
	for(PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];

		//Reset the actuation states.
		PxVehicleWheelActuationState& actState = actuationStates[wheelId];
		PxMemZero(&actState, sizeof(PxVehicleWheelActuationState));

		const PxF32 brakeTorque = brakeResponseStates[wheelId];
		const PxF32 diffTorqueRatio = diffTorqueRatios[wheelId];
		const bool isIntentionToAccelerate = (0.0f == brakeTorque) && (K != 0.0f) && (diffTorqueRatio != 0.0f) && (appliedThrottle != 0.0f);
		actState.isDriveApplied = isIntentionToAccelerate;
		actState.isBrakeApplied = (brakeTorque!= 0.0f);
	}
}

void PxVehicleGearCommandResponseUpdate
(const PxU32 targetGearCommand,
 const PxVehicleGearboxParams& gearboxParams,
 PxVehicleGearboxState& gearboxState)
{
	//Check that we're not halfway through a gear change and we need to execute a change of gear.
	//If we are halfway through a gear change then we cannot execute another until the first is complete.
	//Check that the command stays in a legal gear range.
	if ((gearboxState.currentGear == gearboxState.targetGear) && (targetGearCommand != gearboxState.currentGear) && (targetGearCommand < gearboxParams.nbRatios))
	{
		//We're not executing a gear change and we need to start one.
		//Start the gear change  by 
		//a)setting the target gear.
		//b)putting vehicle in neutral
		//c)set the switch time to PX_VEHICLE_GEAR_SWITCH_INITIATED to flag that we just started a gear change.
		gearboxState.currentGear = gearboxParams.neutralGear;
		gearboxState.targetGear = targetGearCommand;
		gearboxState.gearSwitchTime = PX_VEHICLE_GEAR_SWITCH_INITIATED;
	}
}

void PxVehicleAutoBoxUpdate
(const PxVehicleEngineParams& engineParams, const PxVehicleGearboxParams& gearboxParams, const PxVehicleAutoboxParams& autoboxParams, 
 const PxVehicleEngineState& engineState, const PxVehicleGearboxState& gearboxState,
 const PxReal dt,
 PxU32& targetGearCommand, PxVehicleAutoboxState& autoboxState,
 PxReal& throttle)
{
	if(targetGearCommand != PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR)
	{
		autoboxState.activeAutoboxGearShift = false;
		autoboxState.timeSinceLastShift = PX_VEHICLE_UNSPECIFIED_TIME_SINCE_LAST_SHIFT;
		return;
	}

	//Current and target gear allow us to determine if a gear change is underway.
	const PxU32 currentGear = gearboxState.currentGear;
	const PxU32 targetGear = gearboxState.targetGear;

	//Set to current target gear in case no gear change will be initiated.
	targetGearCommand = targetGear;

	//If the autobox triggered a gear change and the gear change is complete then 
	//reset the corresponding flag.
	if(autoboxState.activeAutoboxGearShift && (currentGear == targetGear))
	{
		autoboxState.activeAutoboxGearShift = false;
	}

	//If the autobox triggered a gear change and the gear change is incomplete then 
	//turn off the throttle pedal.  This happens in autoboxes
	//to stop the driver revving the engine then damaging the 
	//clutch when the clutch re-engages at the end of the gear change.
	if(autoboxState.activeAutoboxGearShift && (currentGear != targetGear))
	{
		throttle = 0.0f;
	}

	//Only process the autobox if no gear change is underway and the time passed since 
	//the last autobox gear change is greater than the autobox latency.
	if (targetGear == currentGear)
	{
		const PxF32 autoBoxSwitchTime = autoboxState.timeSinceLastShift;
		const PxF32 autoBoxLatencyTime = autoboxParams.latency;

		if ((currentGear <= gearboxParams.neutralGear) && ((gearboxParams.neutralGear + 1) < gearboxParams.nbRatios))
		{
			// eAUTOMATIC_GEAR has been set while in neutral or one of the reverse gears
			// => switch to first
			targetGearCommand = gearboxParams.neutralGear + 1;
			throttle = 0.0f;
			autoboxState.timeSinceLastShift = 0.0f;
			autoboxState.activeAutoboxGearShift = true;
		}
		else if (autoBoxSwitchTime > autoBoxLatencyTime)
		{
			const PxF32 normalisedEngineOmega = engineState.rotationSpeed/engineParams.maxOmega;
			const PxU32 neutralGear = gearboxParams.neutralGear;
			const PxU32 nbGears = gearboxParams.nbRatios;
			const PxF32 upRatio = autoboxParams.upRatios[currentGear];
			const PxF32 downRatio = autoboxParams.downRatios[currentGear];

			//If revs too high and not in reverse/neutral and there is a higher gear then switch up. 
			//Note: never switch up from neutral to first
			//Note: never switch up from reverse to neutral
			//Note: never switch up from one reverse gear to another reverse gear.
			PX_ASSERT(currentGear > neutralGear);
			if ((normalisedEngineOmega > upRatio) && ((currentGear + 1) < nbGears))
			{
				targetGearCommand = currentGear + 1;
				throttle = 0.0f;
				autoboxState.timeSinceLastShift = 0.0f;
				autoboxState.activeAutoboxGearShift = true;
			}

			//If revs too low and in gear higher than first then switch down.
			//Note: never switch from forward to neutral
			//Note: never switch from neutral to reverse
			//Note: never switch from reverse gear to reverse gear.
			if ((normalisedEngineOmega < downRatio) && (currentGear > (neutralGear + 1)))
			{
				targetGearCommand = currentGear - 1;
				throttle = 0.0f;
				autoboxState.timeSinceLastShift = 0.0f;
				autoboxState.activeAutoboxGearShift = true;
			}
		}
		else
		{
			autoboxState.timeSinceLastShift += dt;
		}
	}
	else
	{
		autoboxState.timeSinceLastShift += dt;
	}
}

void PxVehicleGearboxUpdate(const PxVehicleGearboxParams& gearboxParams, const PxF32 dt, PxVehicleGearboxState& gearboxState)
{
	if(gearboxState.targetGear != gearboxState.currentGear)
	{
		//If we just started a gear change then set the timer to zero.
		//This replicates legacy behaviour.
		if(gearboxState.gearSwitchTime == PX_VEHICLE_GEAR_SWITCH_INITIATED)
			gearboxState.gearSwitchTime = 0.0f;
		else
			gearboxState.gearSwitchTime += dt;

		//If we've  exceed the switch time then switch to the target gear
		//and reset the timer.
		if (gearboxState.gearSwitchTime > gearboxParams.switchTime)
		{
			gearboxState.currentGear = gearboxState.targetGear;
			gearboxState.gearSwitchTime = PX_VEHICLE_NO_GEAR_SWITCH_PENDING;
		}
	}
}


void countConnectedWheelsNotInConstraintGroup
(const PxU32* connectedWheelIds, const PxU32 nbConnectedWheelIds, const PxVehicleWheelConstraintGroupState& constraintGroups,
 PxU32 connectedWheelsNotInConstraintGroup[PxVehicleLimits::eMAX_NB_WHEELS], PxU32& nbConnectedWheelsNotInConstraintGroup)
{
	//Record the constraint group for each wheel.
	//0xffffffff is reserved for a wheel not in a constraint group.
	PxU32 wheelConstraintGroupIds[PxVehicleLimits::eMAX_NB_WHEELS];
	PxMemSet(wheelConstraintGroupIds, 0xffffffff, sizeof(wheelConstraintGroupIds));
	for (PxU32 i = 0; i < constraintGroups.getNbConstraintGroups(); i++)
	{
		for (PxU32 j = 0; j < constraintGroups.getNbWheelsInConstraintGroup(i); j++)
		{
			const PxU32 wheelId = constraintGroups.getWheelInConstraintGroup(j, i);
			wheelConstraintGroupIds[wheelId] = i;
		}
	}

	//Iterate over all connected wheels and count the number not in a group.
	for (PxU32 i = 0; i < nbConnectedWheelIds; i++)
	{
		const PxU32 wheelId = connectedWheelIds[i];
		const PxU32 constraintGroupId = wheelConstraintGroupIds[wheelId];
		if (0xffffffff == constraintGroupId)
		{
			connectedWheelsNotInConstraintGroup[nbConnectedWheelsNotInConstraintGroup] = wheelId;
			nbConnectedWheelsNotInConstraintGroup++;
		}
	}
}

void PxVehicleEngineDrivetrainUpdate
(const PxVehicleAxleDescription& axleDescription,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
 const PxVehicleEngineParams& engineParams, const PxVehicleClutchParams& clutchParams, const PxVehicleGearboxParams& gearboxParams, 
 const PxVehicleArrayData<const PxReal>& brakeResponseStates,
 const PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
 const PxVehicleArrayData<const PxVehicleTireForce>& tireForces,
 const PxVehicleGearboxState& gearboxState,
 const PxVehicleEngineDriveThrottleCommandResponseState& throttleCommandResponseState, const PxVehicleClutchCommandResponseState& clutchCommandResponseState, 
 const PxVehicleDifferentialState& diffState, const PxVehicleWheelConstraintGroupState* constraintGroupState,
 const PxReal DT,
 PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidbody1dStates,
 PxVehicleEngineState& engineState,
 PxVehicleClutchSlipState& clutchState)
{
	const PxF32 K = PxVehicleClutchStrengthCompute(clutchCommandResponseState, gearboxParams, gearboxState);
	const PxF32 G = PxVehicleGearRatioCompute(gearboxParams, gearboxState);
	const PxF32 engineDriveTorque = PxVehicleEngineDriveTorqueCompute(engineParams, engineState, throttleCommandResponseState);
	const PxF32 engineDampingRate = PxVehicleEngineDampingRateCompute(engineParams, gearboxParams, gearboxState, clutchCommandResponseState, throttleCommandResponseState);

	//Arrange wheel parameters in they order they appear in the connected wheel list.
	const PxU32* connectedWheelIds = diffState.connectedWheels;
	const PxU32 nbConnectedWheelIds = diffState.nbConnectedWheels;
	PxU32 wheelIdToConnectedWheelId[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelDiffTorqueRatios[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelGearings[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelAveWheelSpeedContributions[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelMois[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelRotSpeeds[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelBrakeTorques[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelTireTorques[PxVehicleLimits::eMAX_NB_WHEELS];
	PxF32 connectedWheelDampingRates[PxVehicleLimits::eMAX_NB_WHEELS];
	bool connectedWheelIsBrakeApplied[PxVehicleLimits::eMAX_NB_WHEELS];
	//PxF32 connectedWheelRadii[PxVehicleLimits::eMAX_NB_WHEELS];
	for(PxU32 i = 0; i < nbConnectedWheelIds; i++)
	{
		const PxU32 wheelId = connectedWheelIds[i];
		wheelIdToConnectedWheelId[wheelId] = i;
		connectedWheelDiffTorqueRatios[i] = PxAbs(diffState.torqueRatiosAllWheels[wheelId]);
		connectedWheelGearings[i] = PxVehicleComputeSign(diffState.torqueRatiosAllWheels[wheelId]);
		connectedWheelAveWheelSpeedContributions[i] = diffState.aveWheelSpeedContributionAllWheels[wheelId];
		connectedWheelMois[i] = wheelParams[wheelId].moi;
		connectedWheelRotSpeeds[i] = wheelRigidbody1dStates[wheelId].rotationSpeed;
		connectedWheelBrakeTorques[i] = -PxVehicleComputeSign(wheelRigidbody1dStates[wheelId].rotationSpeed)*brakeResponseStates[wheelId];
		connectedWheelTireTorques[i] = tireForces[wheelId].wheelTorque;
		connectedWheelDampingRates[i] = wheelParams[wheelId].dampingRate;
		connectedWheelIsBrakeApplied[i] = actuationStates[wheelId].isBrakeApplied;
		//connectedWheelRadii[i] = wheelParams[wheelId].radius;
	};

	//Compute the clutch slip
	clutchState.setToDefault();
	PxF32 clutchSlip = 0;
	{
		PxF32 averageWheelSpeed = 0;
		for (PxU32 i = 0; i < nbConnectedWheelIds; i++)
		{
			averageWheelSpeed += connectedWheelRotSpeeds[i] * connectedWheelAveWheelSpeedContributions[i];
		}
		clutchSlip = G*averageWheelSpeed - engineState.rotationSpeed;
	}
	clutchState.clutchSlip = clutchSlip;

	//
	//torque at clutch:  
	//tc = K*{G*[alpha0*w0 + alpha1*w1 + alpha2*w2 + ..... alpha(N-1)*w(N-1)] - wEng}
	//where 
	//(i)   G is the gearing ratio, 
	//(ii)  alphai is the fractional contribution of the ith wheel to the average wheel speed at the clutch (alpha(i) is zero for undriven wheels)
	//(iii) wi is the angular speed of the ith wheel
	//(iv)  K is the clutch strength 
	//(v)   wEng is the angular speed of the engine

	//torque applied to ith wheel is 
	//ti = G*gammai*tc + bt(i) + tt(i) 
	//where
	//gammai is the fractional proportion of the clutch torque that the differential delivers to the ith wheel
	//bt(i) is the brake torque applied to the ith wheel
	//tt(i) is the tire torque applied to the ith wheel

	//acceleration applied to ith wheel is 
	//ai = G*gammai*K*{G*[alpha0*w0 + alpha1*w1 alpha2*w2 + ..... alpha(N-1)*w(N-1)] - wEng}/Ii + (bt(i) + tt(i))/Ii
	//wheer Ii is the moi of the ith wheel

	//express ai as 
	//ai = [wi(t+dt) - wi(t)]/dt
	//and rearrange
	//wi(t+dt) - wi(t)] = dt*G*gammai*K*{G*[alpha0*w0(t+dt) + alpha1*w1(t+dt) + alpha2*w2(t+dt) + ..... alpha(N-1)*w(N-1)(t+dt)] - wEng(t+dt)}/Ii + dt*(bt(i) + tt(i))/Ii

	//Do the same for tEng (torque applied to engine)
	//tEng  = -tc + engineDriveTorque
	//where engineDriveTorque is the drive torque applied to the engine
	//Assuming the engine has unit mass then
	//wEng(t+dt) -wEng(t) = -dt*K*{G*[alpha0*w0(t+dt) + alpha1*w1(t+dt) + alpha2*w2(t+dt) + ..... alpha(N-1)*w(N-1(t+dt))] - wEng(t+dt)}/Ieng + dt*engineDriveTorque]/IEng

	//Introduce the vector w=(w0,w1,w2....w(N-1), wEng)
	//and re-express as a matrix after collecting all unknowns at (t+dt) and knowns at time t.
	//A*w(t+dt)=b(t);

	PxVehicleMatrixNN M(nbConnectedWheelIds + 1);
	PxVehicleVectorN b(nbConnectedWheelIds + 1);
	PxVehicleVectorN result(nbConnectedWheelIds + 1);

	const PxF32 KG = K * G;
	const PxF32 KGG = K * G*G;

	//Wheels
	{
		for (PxU32 i = 0; i < nbConnectedWheelIds; i++)
		{
			const PxF32 dt = DT / connectedWheelMois[i];
			const PxF32 R = connectedWheelDiffTorqueRatios[i];
			const PxF32 g = connectedWheelGearings[i];
			const PxF32 dtKGGRg = dt * KGG*R*g;

			for(PxU32 j = 0; j < nbConnectedWheelIds; j++)
			{
				M.set(i, j, dtKGGRg*connectedWheelAveWheelSpeedContributions[j]*connectedWheelGearings[j]);
			}
			M.set(i, i, 1.0f + dtKGGRg*connectedWheelAveWheelSpeedContributions[i]*connectedWheelGearings[i] + dt * connectedWheelDampingRates[i]);
			M.set(i, nbConnectedWheelIds, -dt*KG*R*g);
			b[i] = connectedWheelRotSpeeds[i] + dt * (connectedWheelBrakeTorques[i] + connectedWheelTireTorques[i]);
			result[i] = connectedWheelRotSpeeds[i];
		}
	}

	//Engine.
	{
		const PxF32 dt = DT / engineParams.moi;
		const PxF32 dtKG = dt*K*G;
		for(PxU32 j = 0; j < nbConnectedWheelIds; j++)
		{
			M.set(nbConnectedWheelIds, j, -dtKG * connectedWheelAveWheelSpeedContributions[j]* connectedWheelGearings[j]);
		}
		M.set(nbConnectedWheelIds, nbConnectedWheelIds, 1.0f + dt * (K + engineDampingRate));
		b[nbConnectedWheelIds] = engineState.rotationSpeed + dt * engineDriveTorque;
		result[nbConnectedWheelIds] = engineState.rotationSpeed;
	}

	if (constraintGroupState && constraintGroupState->getNbConstraintGroups() > 0)
	{
		const PxU32 nbConstraintGroups = constraintGroupState->getNbConstraintGroups();

		//Count the wheels not in a constraint group.
		PxU32 connectedWheelsNotInConstraintGroup[PxVehicleLimits::eMAX_NB_WHEELS];
		PxU32 nbConnectedWheelsNotInConstraintGroup = 0;
		countConnectedWheelsNotInConstraintGroup(
			connectedWheelIds, nbConnectedWheelIds, *constraintGroupState, 
			connectedWheelsNotInConstraintGroup, nbConnectedWheelsNotInConstraintGroup);

		//After applying constraint groups:
		//  number of columns remains nbConnectedWheelIds + 1
		//  each row will be of length nbConnectedWheelsNotInConstraintGroup + nbConstraintGroups + 1
		PxVehicleMatrixNN A(nbConnectedWheelIds + 1);
		for (PxU32 i = 0; i < nbConnectedWheelIds + 1; i++)
		{
			//1 entry for each wheel not in a constraint group.
			for (PxU32 j = 0; j < nbConnectedWheelsNotInConstraintGroup; j++)
			{
				const PxU32 wheelId = connectedWheelsNotInConstraintGroup[j];
				const PxU32 connectedWheelId = wheelIdToConnectedWheelId[wheelId];
				const PxF32 MIJ = M.get(i, connectedWheelId);
				A.set(i, j, MIJ);
			}

			//1 entry for each constraint group.
			for (PxU32 j = nbConnectedWheelsNotInConstraintGroup; j < nbConnectedWheelsNotInConstraintGroup + nbConstraintGroups; j++)
			{
				const PxU32 constraintGroupId = (j - nbConnectedWheelsNotInConstraintGroup);

				PxF32 sum = 0.0f;
				//for (PxU32 k = 0; k < 1; k++)
				{
					//k = 0 is a special case with multiplier = 1.0.
					const PxU32 wheelId = constraintGroupState->getWheelInConstraintGroup(0, constraintGroupId);
					const PxF32 multiplier = 1.0f;
					const PxU32 connectedWheelId = wheelIdToConnectedWheelId[wheelId];
					const PxF32 MIK = M.get(i, connectedWheelId);
					sum += MIK * multiplier;
				}
				for (PxU32 k = 1; k < constraintGroupState->getNbWheelsInConstraintGroup(constraintGroupId); k++)
				{
					const PxU32 wheelId = constraintGroupState->getWheelInConstraintGroup(k, constraintGroupId);
					const PxF32 multiplier = constraintGroupState->getMultiplierInConstraintGroup(k, constraintGroupId);
					const PxU32 connectedWheelId = wheelIdToConnectedWheelId[wheelId];
					const PxF32 MIK = M.get(i, connectedWheelId);
					sum += MIK*multiplier;
				}
				A.set(i, j, sum);
			}

			//1 entry for the engine.
			{
				const PxF32 MIJ = M.get(i, nbConnectedWheelIds);
				A.set(i, nbConnectedWheelsNotInConstraintGroup + nbConstraintGroups, MIJ);
			}
		}

		const PxU32 N = (nbConnectedWheelsNotInConstraintGroup + nbConstraintGroups + 1);
		//Compute A^T * A 
		PxVehicleMatrixNN ATA(N);	
		for (PxU32 i = 0; i < N; i++)
		{
			for (PxU32 j = 0; j < N; j++)
			{
				PxF32 sum = 0.0f;
				for (PxU32 k = 0; k < nbConnectedWheelIds + 1; k++)
				{
					sum += A.get(k, i)*A.get(k, j);
				}
				ATA.set(i, j, sum);
			}
		}

		//Compute A^T*b;
		PxVehicleVectorN ATb(N);
		for (PxU32 i = 0; i < N; i++)
		{
			PxF32 sum = 0;
			for (PxU32 j = 0; j < nbConnectedWheelIds + 1; j++)
			{
				sum += A.get(j, i)*b[j];
			}
			ATb[i] = sum;
		}

		//Solve it.
		PxVehicleMatrixNNLUSolver solver;
		PxVehicleVectorN result2(N);
		solver.decomposeLU(ATA);
		solver.solve(ATb, result2);

		//Map from result2 back to result.
		for (PxU32 j = 0; j < nbConnectedWheelsNotInConstraintGroup; j++)
		{
			const PxU32 wheelId = connectedWheelsNotInConstraintGroup[j];
			const PxU32 connectedWheelId = wheelIdToConnectedWheelId[wheelId];
			result[connectedWheelId] = result2[j];
		}
		for (PxU32 j = nbConnectedWheelsNotInConstraintGroup; j < nbConnectedWheelsNotInConstraintGroup + nbConstraintGroups; j++)
		{
			const PxU32 constraintGroupId = (j - nbConnectedWheelsNotInConstraintGroup);
			//for (PxU32 k = 0; k < 1; k++)
			{
				//k = 0 is a special case with multiplier = 1.0
				const PxU32 wheelId = constraintGroupState->getWheelInConstraintGroup(0, constraintGroupId);
				const PxF32 multiplier = 1.0f;
				const PxU32 connectedWheelId = wheelIdToConnectedWheelId[wheelId];
				result[connectedWheelId] = multiplier * result2[j];
			}
			for (PxU32 k = 1; k < constraintGroupState->getNbWheelsInConstraintGroup(constraintGroupId); k++)
			{
				const PxU32 wheelId = constraintGroupState->getWheelInConstraintGroup(k, constraintGroupId);
				const PxF32 multiplier = constraintGroupState->getMultiplierInConstraintGroup(k, constraintGroupId);
				const PxU32 connectedWheelId = wheelIdToConnectedWheelId[wheelId];
				result[connectedWheelId] = multiplier * result2[j];
			}
		}
		{
			result[nbConnectedWheelIds] = result2[nbConnectedWheelsNotInConstraintGroup + nbConstraintGroups];
		}
	}
	else if (PxVehicleClutchAccuracyMode::eBEST_POSSIBLE == clutchParams.accuracyMode)
	{
		//Solve Aw=b
		PxVehicleMatrixNNLUSolver solver;
		solver.decomposeLU(M);
		solver.solve(b, result);
		//PX_WARN_ONCE_IF(!isValid(A, b, result), "Unable to compute new PxVehicleDrive4W internal rotation speeds.  Please check vehicle sim data, especially clutch strength; engine moi and damping; wheel moi and damping");
	}
	else
	{
		PxVehicleMatrixNGaussSeidelSolver solver;
		solver.solve(clutchParams.estimateIterations, 1e-10f, M, b, result);
	}

	//Check for sanity in the resultant internal rotation speeds.
	//If the brakes are on and the wheels have switched direction then lock them at zero.
	//A consequence of this quick fix is that locked wheels remain locked until the brake is entirely released.
	//This isn't strictly mathematically or physically correct - a more accurate solution would either formulate the 
	//brake as a lcp problem or repeatedly solve with constraints that locked wheels remain at zero rotation speed.
	//The physically correct solution will certainly be more expensive so let's live with the restriction that 
	//locked wheels remain locked until the brake is released. 
	//newOmega=result[i], oldOmega=wheelSpeeds[i], if newOmega*oldOmega<=0 and isBrakeApplied then lock wheel.
	for(PxU32 i = 0; i < nbConnectedWheelIds; i++)
	{
		result[i] = (connectedWheelIsBrakeApplied[i] && (connectedWheelRotSpeeds[i] * result[i] <= 0)) ? 0.0f : result[i];
	}
	//Clamp the engine revs.
	//Again, this is not physically or mathematically correct but the loss in behaviour will be hard to notice.
	//The alternative would be to add constraints to the solver, which would be much more expensive.
	result[nbConnectedWheelIds] = PxClamp(result[nbConnectedWheelIds], engineParams.idleOmega, engineParams.maxOmega);

	//Copy back to the car's internal rotation speeds.
	for (PxU32 i = 0; i < nbConnectedWheelIds; i++)
	{
		const PxU32 wheelId = connectedWheelIds[i];
		wheelRigidbody1dStates[wheelId].rotationSpeed = result[i];
	}
	engineState.rotationSpeed = result[nbConnectedWheelIds];

	//Update the undriven wheels.
	bool isDrivenWheel[PxVehicleLimits::eMAX_NB_WHEELS];
	PxMemZero(isDrivenWheel, sizeof(isDrivenWheel));
	for(PxU32 i = 0; i < nbConnectedWheelIds; i++)
	{
		const PxU32 wheelId = connectedWheelIds[i];
		isDrivenWheel[wheelId] = true;
	}
	for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
		if (!isDrivenWheel[wheelId])
		{
			PxVehicleDirectDriveUpdate(
				wheelParams[wheelId],
				actuationStates[wheelId],
				brakeResponseStates[wheelId], 0.0f,
				tireForces[wheelId], DT, wheelRigidbody1dStates[wheelId]);
		}
	}
}

void PxVehicleClutchCommandResponseLinearUpdate
(const PxReal clutchCommand,
 const PxVehicleClutchCommandResponseParams& clutchResponseParams,
 PxVehicleClutchCommandResponseState& clutchResponse)
{
	clutchResponse.normalisedCommandResponse = (1.0f - clutchCommand);
	clutchResponse.commandResponse = (1.0f - clutchCommand)*clutchResponseParams.maxResponse;
}

void PxVehicleEngineDriveThrottleCommandResponseLinearUpdate
(const PxVehicleCommandState& commands,
 PxVehicleEngineDriveThrottleCommandResponseState& throttleResponse)
{
	throttleResponse.commandResponse = commands.throttle;
}


} //namespace vehicle2
} //namespace physx
