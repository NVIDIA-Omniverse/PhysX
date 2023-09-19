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

#include "PxPhysicsAPI.h"

#include "../snippetvehicle2common/directdrivetrain/DirectDrivetrain.h"

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

struct CustomSuspensionParams
{
	PxReal phase;
	PxReal frequency;
	PxReal amplitude;
};

struct CustomSuspensionState
{
	PxReal theta;

	PX_FORCE_INLINE void setToDefault()
	{
		PxMemZero(this, sizeof(CustomSuspensionState));
	}
};

void addCustomSuspensionForce
(const PxReal dt,
	const PxVehicleSuspensionParams& suspParams,
	const CustomSuspensionParams& customParams,
	const PxVec3& groundNormal, bool isWheelOnGround,
	const PxVehicleSuspensionComplianceState& suspComplianceState, const PxVehicleRigidBodyState& rigidBodyState,
	PxVehicleSuspensionForce& suspForce, CustomSuspensionState& customState);

class CustomSuspensionComponent : public PxVehicleComponent
{
public:

	CustomSuspensionComponent() : PxVehicleComponent() {}

	virtual void getDataForCustomSuspensionComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyParams*& rigidBodyParams,
		const PxVehicleSuspensionStateCalculationParams*& suspensionStateCalculationParams,
		const PxReal*& steerResponseStates,
		const PxVehicleRigidBodyState*& rigidBodyState,
		const PxVehicleWheelParams*& wheelParams,
		const PxVehicleSuspensionParams*& suspensionParams,
		const CustomSuspensionParams*& customSuspensionParams,
		const PxVehicleSuspensionComplianceParams*& suspensionComplianceParams,
		const PxVehicleSuspensionForceParams*& suspensionForceParams,
		const PxVehicleRoadGeometryState*& wheelRoadGeomStates,
		PxVehicleSuspensionState*& suspensionStates,
		CustomSuspensionState*& customSuspensionStates,
		PxVehicleSuspensionComplianceState*& suspensionComplianceStates,
		PxVehicleSuspensionForce*& suspensionForces) = 0;

	virtual bool update(const PxReal dt, const PxVehicleSimulationContext& context)
	{
		const PxVehicleAxleDescription* axleDescription;
		const PxVehicleRigidBodyParams* rigidBodyParams;
		const PxVehicleSuspensionStateCalculationParams* suspensionStateCalculationParams;
		const PxReal* steerResponseStates;
		const PxVehicleRigidBodyState* rigidBodyState;
		const PxVehicleWheelParams* wheelParams;
		const PxVehicleSuspensionParams* suspensionParams;
		const CustomSuspensionParams* customSuspensionParams;
		const PxVehicleSuspensionComplianceParams* suspensionComplianceParams;
		const PxVehicleSuspensionForceParams* suspensionForceParams;
		const PxVehicleRoadGeometryState* wheelRoadGeomStates;
		PxVehicleSuspensionState* suspensionStates;
		CustomSuspensionState* customSuspensionStates;
		PxVehicleSuspensionComplianceState* suspensionComplianceStates;
		PxVehicleSuspensionForce* suspensionForces;

		getDataForCustomSuspensionComponent(axleDescription, rigidBodyParams, suspensionStateCalculationParams,
			steerResponseStates, rigidBodyState, wheelParams, suspensionParams, customSuspensionParams,
			suspensionComplianceParams, suspensionForceParams, wheelRoadGeomStates,
			suspensionStates, customSuspensionStates, suspensionComplianceStates, suspensionForces);

		for (PxU32 i = 0; i < axleDescription->nbWheels; i++)
		{
			const PxU32 wheelId = axleDescription->wheelIdsInAxleOrder[i];

			//Update the suspension state (jounce, jounce speed)
			PxVehicleSuspensionStateUpdate(
				wheelParams[wheelId], suspensionParams[wheelId], *suspensionStateCalculationParams,
				suspensionForceParams[wheelId].stiffness, suspensionForceParams[wheelId].damping,
				steerResponseStates[wheelId], wheelRoadGeomStates[wheelId], 
				*rigidBodyState,
				dt, context.frame, context.gravity,
				suspensionStates[wheelId]);

			//Update the compliance from the suspension state.
			PxVehicleSuspensionComplianceUpdate(
				suspensionParams[wheelId], suspensionComplianceParams[wheelId],
				suspensionStates[wheelId],
				suspensionComplianceStates[wheelId]);

			//Compute the suspension force from the suspension and compliance states.
			PxVehicleSuspensionForceUpdate(
				suspensionParams[wheelId], suspensionForceParams[wheelId],
				wheelRoadGeomStates[wheelId], suspensionStates[wheelId], 
				suspensionComplianceStates[wheelId], *rigidBodyState,
				context.gravity, rigidBodyParams->mass,
				suspensionForces[wheelId]);

			addCustomSuspensionForce(dt,
				suspensionParams[wheelId],
				customSuspensionParams[wheelId],
				wheelRoadGeomStates[wheelId].plane.n, PxVehicleIsWheelOnGround(suspensionStates[wheelId]),
				suspensionComplianceStates[wheelId], *rigidBodyState,
				suspensionForces[wheelId], customSuspensionStates[wheelId]);
		}

		return true;
	}
};


//
//This class holds the parameters, state and logic needed to implement a vehicle that
//is using a custom component for the suspension logic.
//
//See BaseVehicle for more details on the snippet code design.
//
class CustomSuspensionVehicle
	: public DirectDriveVehicle
	, public CustomSuspensionComponent
{
public:
	bool initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial, bool addPhysXBeginEndComponents = true);
	virtual void destroy();

	virtual void initComponentSequence(bool addPhysXBeginEndComponents);

	virtual void getDataForCustomSuspensionComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyParams*& rigidBodyParams,
		const PxVehicleSuspensionStateCalculationParams*& suspensionStateCalculationParams,
		const PxReal*& steerResponseStates,
		const PxVehicleRigidBodyState*& rigidBodyState,
		const PxVehicleWheelParams*& wheelParams,
		const PxVehicleSuspensionParams*& suspensionParams,
		const CustomSuspensionParams*& customSuspensionParams,
		const PxVehicleSuspensionComplianceParams*& suspensionComplianceParams,
		const PxVehicleSuspensionForceParams*& suspensionForceParams,
		const PxVehicleRoadGeometryState*& wheelRoadGeomStates,
		PxVehicleSuspensionState*& suspensionStates,
		CustomSuspensionState*& customSuspensionStates,
		PxVehicleSuspensionComplianceState*& suspensionComplianceStates,
		PxVehicleSuspensionForce*& suspensionForces)
	{
		axleDescription = &mBaseParams.axleDescription;
		rigidBodyParams = &mBaseParams.rigidBodyParams;
		suspensionStateCalculationParams = &mBaseParams.suspensionStateCalculationParams;
		steerResponseStates = mBaseState.steerCommandResponseStates;
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelParams = mBaseParams.wheelParams;
		suspensionParams = mBaseParams.suspensionParams;
		customSuspensionParams = mCustomSuspensionParams;
		suspensionComplianceParams = mBaseParams.suspensionComplianceParams;
		suspensionForceParams = mBaseParams.suspensionForceParams;
		wheelRoadGeomStates = mBaseState.roadGeomStates;
		suspensionStates = mBaseState.suspensionStates;
		customSuspensionStates = mCustomSuspensionStates;
		suspensionComplianceStates = mBaseState.suspensionComplianceStates;
		suspensionForces = mBaseState.suspensionForces;
	}


	//Parameters and states of the vehicle's custom suspension.
	CustomSuspensionParams mCustomSuspensionParams[4];
	CustomSuspensionState mCustomSuspensionStates[4];
};

}//namespace snippetvehicle2
