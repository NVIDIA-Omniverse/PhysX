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

#include "../snippetvehicle2common/directdrivetrain/DirectDrivetrain.h"

#include "CustomTire.h"


namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;


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

}//namespace snippetvehicle2
