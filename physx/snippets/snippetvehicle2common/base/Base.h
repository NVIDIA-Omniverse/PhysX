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

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

struct BaseVehicleParams
{
	PxVehicleAxleDescription axleDescription;
	PxVehicleFrame frame;
	PxVehicleScale scale;
	PxVehicleSuspensionStateCalculationParams suspensionStateCalculationParams;

	//Command response
	PxVehicleBrakeCommandResponseParams brakeResponseParams[2];
	PxVehicleSteerCommandResponseParams steerResponseParams;
	PxVehicleAckermannParams ackermannParams[1];

	//Suspension
	PxVehicleSuspensionParams suspensionParams[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleSuspensionComplianceParams suspensionComplianceParams[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleSuspensionForceParams suspensionForceParams[PxVehicleLimits::eMAX_NB_WHEELS];

	//Tires
	PxVehicleTireForceParams tireForceParams[PxVehicleLimits::eMAX_NB_WHEELS];

	//Wheels
	PxVehicleWheelParams wheelParams[PxVehicleLimits::eMAX_NB_WHEELS];

	//Rigid body
	PxVehicleRigidBodyParams rigidBodyParams;

	BaseVehicleParams transformAndScale(
		const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const;

	PX_FORCE_INLINE bool isValid() const
	{
		if (!axleDescription.isValid())
			return false;
		if (!frame.isValid())
			return true;
		if (!scale.isValid())
			return false;
		if (!suspensionStateCalculationParams.isValid())
			return false;

		if (!brakeResponseParams[0].isValid(axleDescription))
			return false;
		if (!brakeResponseParams[1].isValid(axleDescription))
			return false;
		if (!steerResponseParams.isValid(axleDescription))
			return false;
		if (!ackermannParams[0].isValid(axleDescription))
				return false;

		for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
		{
			const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];

			if (!suspensionParams[wheelId].isValid())
				return false;
			if (!suspensionComplianceParams[wheelId].isValid())
				return false;
			if (!suspensionForceParams[wheelId].isValid())
				return false;

			if (!tireForceParams[wheelId].isValid())
				return false;

			if (!wheelParams[wheelId].isValid())
				return false;
		}

		if (!rigidBodyParams.isValid())
			return false;

		return true;
	}
};

struct BaseVehicleState
{
	//Command responses
	PxReal brakeCommandResponseStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxReal steerCommandResponseStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleWheelActuationState actuationStates[PxVehicleLimits::eMAX_NB_WHEELS];

	//Road geometry
	PxVehicleRoadGeometryState roadGeomStates[PxVehicleLimits::eMAX_NB_WHEELS];

	//Suspensions
	PxVehicleSuspensionState suspensionStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleSuspensionComplianceState suspensionComplianceStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleSuspensionForce suspensionForces[PxVehicleLimits::eMAX_NB_WHEELS];

	//Tires
	PxVehicleTireGripState tireGripStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleTireDirectionState tireDirectionStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleTireSpeedState tireSpeedStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleTireSlipState tireSlipStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleTireCamberAngleState tireCamberAngleStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleTireStickyState tireStickyStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleTireForce tireForces[PxVehicleLimits::eMAX_NB_WHEELS];

	//Wheels
	PxVehicleWheelRigidBody1dState wheelRigidBody1dStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehicleWheelLocalPose wheelLocalPoses[PxVehicleLimits::eMAX_NB_WHEELS];

	//Rigid body
	PxVehicleRigidBodyState rigidBodyState;

	PX_FORCE_INLINE void setToDefault()
	{
		for (unsigned int i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
		{
			brakeCommandResponseStates[i] = 0.0;
			steerCommandResponseStates[i] = 0.0f;

			actuationStates[i].setToDefault();

			roadGeomStates[i].setToDefault();

			suspensionStates[i].setToDefault();
			suspensionComplianceStates[i].setToDefault();
			suspensionForces[i].setToDefault();

			tireGripStates[i].setToDefault();
			tireDirectionStates[i].setToDefault();
			tireSpeedStates[i].setToDefault();
			tireSlipStates[i].setToDefault();
			tireCamberAngleStates[i].setToDefault();
			tireStickyStates[i].setToDefault();
			tireForces[i].setToDefault();

			wheelRigidBody1dStates[i].setToDefault();
			wheelLocalPoses[i].setToDefault();
		}

		rigidBodyState.setToDefault();
	}
};


//
//The PhysX Vehicle SDK was designed to offer flexibility on how to combine data and logic
//to implement a vehicle. For the purpose of the snippets, different vehicle types are
//represented using a class hierarchy to share base functionality and data. The vehicle
//classes implement the component interfaces directly such that no separate component
//objects are needed. If the goal was to focus more on modularity, each component
//could be defined as its own class but those classes would need to reference all the
//data necessary to run the component logic.
//This specific class deals with the mechanical base of a vehicle (suspension, tire,
//wheel, vehicle body etc.).
//
class BaseVehicle
	: public PxVehicleRigidBodyComponent
	, public PxVehicleSuspensionComponent
	, public PxVehicleTireComponent
	, public PxVehicleWheelComponent
{
public:
	bool initialize();
	virtual void destroy() {}

	//To be implemented by specific vehicle types that are built on top of this class.
	//The specific vehicle type defines what components to run and in what order.
	virtual void initComponentSequence(bool addPhysXBeginEndComponents) = 0;

	//Run a simulation step
	void step(const PxReal dt, const PxVehicleSimulationContext& context);

	virtual void getDataForRigidBodyComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyParams*& rigidBodyParams,
		PxVehicleArrayData<const PxVehicleSuspensionForce>& suspensionForces,
		PxVehicleArrayData<const PxVehicleTireForce>& tireForces,
		const PxVehicleAntiRollTorque*& antiRollTorque,
		PxVehicleRigidBodyState*& rigidBodyState)
	{
		axleDescription = &mBaseParams.axleDescription;
		rigidBodyParams = &mBaseParams.rigidBodyParams;
		suspensionForces.setData(mBaseState.suspensionForces);
		tireForces.setData(mBaseState.tireForces);
		antiRollTorque = NULL;
		rigidBodyState = &mBaseState.rigidBodyState;
	}

	virtual void getDataForSuspensionComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyParams*& rigidBodyParams,
		const PxVehicleSuspensionStateCalculationParams*& suspensionStateCalculationParams,
		PxVehicleArrayData<const PxReal>& steerResponseStates,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
		PxVehicleArrayData<const PxVehicleSuspensionComplianceParams>& suspensionComplianceParams,
		PxVehicleArrayData<const PxVehicleSuspensionForceParams>& suspensionForceParams,
		PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams>& antiRollForceParams,
		PxVehicleArrayData<const PxVehicleRoadGeometryState>& wheelRoadGeomStates,
		PxVehicleArrayData<PxVehicleSuspensionState>& suspensionStates,
		PxVehicleArrayData<PxVehicleSuspensionComplianceState>& suspensionComplianceStates,
		PxVehicleArrayData<PxVehicleSuspensionForce>& suspensionForces,
		PxVehicleAntiRollTorque*& antiRollTorque)
	{
		axleDescription = &mBaseParams.axleDescription;
		rigidBodyParams = &mBaseParams.rigidBodyParams;
		suspensionStateCalculationParams = &mBaseParams.suspensionStateCalculationParams;
		steerResponseStates.setData(mBaseState.steerCommandResponseStates);
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelParams.setData(mBaseParams.wheelParams);
		suspensionParams.setData(mBaseParams.suspensionParams);
		suspensionComplianceParams.setData(mBaseParams.suspensionComplianceParams);
		suspensionForceParams.setData(mBaseParams.suspensionForceParams);
		antiRollForceParams.setEmpty();
		wheelRoadGeomStates.setData(mBaseState.roadGeomStates);
		suspensionStates.setData(mBaseState.suspensionStates);
		suspensionComplianceStates.setData(mBaseState.suspensionComplianceStates);
		suspensionForces.setData(mBaseState.suspensionForces);
		antiRollTorque = NULL;
	}

	virtual void getDataForTireComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleArrayData<const PxReal>& steerResponseStates,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
		PxVehicleArrayData<const PxVehicleTireForceParams>& tireForceParams,
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
		tireForceParams.setData(mBaseParams.tireForceParams);
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

	virtual void getDataForWheelComponent(
		const PxVehicleAxleDescription*& axleDescription,
		PxVehicleArrayData<const PxReal>& steerResponseStates,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
		PxVehicleArrayData<const PxVehicleWheelActuationState>& actuationStates,
		PxVehicleArrayData<const PxVehicleSuspensionState>& suspensionStates,
		PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspensionComplianceStates,
		PxVehicleArrayData<const PxVehicleTireSpeedState>& tireSpeedStates,
		PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
  	    PxVehicleArrayData<PxVehicleWheelLocalPose>& wheelLocalPoses)
	{
		axleDescription = &mBaseParams.axleDescription;
		steerResponseStates.setData(mBaseState.steerCommandResponseStates);
		wheelParams.setData(mBaseParams.wheelParams);
		suspensionParams.setData(mBaseParams.suspensionParams);
		actuationStates.setData(mBaseState.actuationStates);
		suspensionStates.setData(mBaseState.suspensionStates);
		suspensionComplianceStates.setData(mBaseState.suspensionComplianceStates);
		tireSpeedStates.setData(mBaseState.tireSpeedStates);
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);
		wheelLocalPoses.setData(mBaseState.wheelLocalPoses);
	}


	//Parameters and statess of the vehicle's mechanical base.
	BaseVehicleParams mBaseParams;
	BaseVehicleState mBaseState;

	//The sequence of components that will simulate the vehicle.
	//To be assembled by specific vehicle types that are built
	//on top of this class
	PxVehicleComponentSequence mComponentSequence;
	
	//A sub-group of components can be simulated with multiple substeps
	//to improve simulation fidelity without running the full sequence
	//at a lower timestep.
	PxU8 mComponentSequenceSubstepGroupHandle;
};

}//namespace snippetvehicle2
