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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#pragma once

#include "vehicle2/PxVehicleAPI.h"
#include "../physxintegration/PhysXIntegration.h"

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

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

}//namespace snippetvehicle2
