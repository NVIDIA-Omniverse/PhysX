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

#include "PxScene.h"
#include "vehicle2/PxVehicleAPI.h"
#include "../base/Base.h"

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

struct PhysXIntegrationParams
{
	PxVehiclePhysXRoadGeometryQueryParams physxRoadGeometryQueryParams;
	PxVehiclePhysXMaterialFrictionParams physxMaterialFrictionParams[PxVehicleLimits::eMAX_NB_WHEELS];
	PxVehiclePhysXSuspensionLimitConstraintParams physxSuspensionLimitConstraintParams[PxVehicleLimits::eMAX_NB_WHEELS];
	PxTransform physxActorCMassLocalPose;
	PxVec3 physxActorBoxShapeHalfExtents;
	PxTransform physxActorBoxShapeLocalPose;
	PxTransform physxWheelShapeLocalPoses[PxVehicleLimits::eMAX_NB_WHEELS];

	void create
		(const PxVehicleAxleDescription& axleDescription,
		 const PxQueryFilterData& queryFilterData, PxQueryFilterCallback* queryFilterCallback,
		 PxVehiclePhysXMaterialFriction* materialFrictions, const PxU32 nbMaterialFrictions, const PxReal defaultFriction,
		 const PxTransform& physXActorCMassLocalPose,
		 const PxVec3& physXActorBoxShapeHalfExtents, const PxTransform& physxActorBoxShapeLocalPose);

	PhysXIntegrationParams transformAndScale(
		const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const;

	PX_FORCE_INLINE bool isValid(const PxVehicleAxleDescription& axleDesc) const
	{
		if (!physxRoadGeometryQueryParams.isValid())
			return false;
		for (PxU32 i = 0; i < axleDesc.nbWheels; i++)
		{
			const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
			if (!physxMaterialFrictionParams[wheelId].isValid())
					return false;
			if (!physxSuspensionLimitConstraintParams[wheelId].isValid())
					return false;
		}
		return true;
	}
};

struct PhysXIntegrationState
{
	PxVehiclePhysXActor physxActor;					//physx actor
	PxVehiclePhysXSteerState physxSteerState;
	PxVehiclePhysXConstraints physxConstraints;		//susp limit and sticky tire constraints

	PX_FORCE_INLINE void setToDefault()
	{
		physxActor.setToDefault();
		physxSteerState.setToDefault();
		physxConstraints.setToDefault();
	}

	void create
		(const BaseVehicleParams& baseParams, const PhysXIntegrationParams& physxParams,
		 PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial);

	void destroy();

};


void setPhysXIntegrationParams(const PxVehicleAxleDescription&,
	PxVehiclePhysXMaterialFriction*, PxU32 nbPhysXMaterialFrictions,
	PxReal physXDefaultMaterialFriction, PhysXIntegrationParams&);


//
//This class holds the parameters, state and logic needed to implement a vehicle that
//is using a PhysX actor to potentially interact with other objects in a PhysX scene.
//
//See BaseVehicle for more details on the snippet code design.
//
class PhysXActorVehicle
	: public BaseVehicle
	, public PxVehiclePhysXActorBeginComponent
	, public PxVehiclePhysXActorEndComponent
	, public PxVehiclePhysXConstraintComponent
	, public PxVehiclePhysXRoadGeometrySceneQueryComponent
{
public:
	bool initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial);
	virtual void destroy();

	void setUpActor(PxScene& scene, const PxTransform& pose, const char* vehicleName);

	virtual void getDataForPhysXActorBeginComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleCommandState*& commands,
		const PxVehicleEngineDriveTransmissionCommandState*& transmissionCommands,
		const PxVehicleGearboxParams*& gearParams,
		const PxVehicleGearboxState*& gearState,
		const PxVehicleEngineParams*& engineParams,
		PxVehiclePhysXActor*& physxActor,
		PxVehiclePhysXSteerState*& physxSteerState,
		PxVehiclePhysXConstraints*& physxConstraints,
		PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
		PxVehicleEngineState*& engineState)
	{
		axleDescription = &mBaseParams.axleDescription;
		commands = &mCommandState;
		physxActor = &mPhysXState.physxActor;
		physxSteerState = &mPhysXState.physxSteerState;
		physxConstraints = &mPhysXState.physxConstraints;
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);

		transmissionCommands = NULL;
		gearParams = NULL;
		gearState = NULL;
		engineParams = NULL;
		engineState = NULL;
	}

	virtual void getDataForPhysXActorEndComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxTransform>& wheelShapeLocalPoses,
		PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
		PxVehicleArrayData<const PxVehicleWheelLocalPose>& wheelLocalPoses,
		const PxVehicleGearboxState*& gearState,
		PxVehiclePhysXActor*& physxActor)
	{
		axleDescription = &mBaseParams.axleDescription;
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelParams.setData(mBaseParams.wheelParams);
		wheelShapeLocalPoses.setData(mPhysXParams.physxWheelShapeLocalPoses);
		wheelRigidBody1dStates.setData(mBaseState.wheelRigidBody1dStates);
		wheelLocalPoses.setData(mBaseState.wheelLocalPoses);
		physxActor = &mPhysXState.physxActor;

		gearState = NULL;
	}

	virtual void getDataForPhysXConstraintComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
		PxVehicleArrayData<const PxVehiclePhysXSuspensionLimitConstraintParams>& suspensionLimitParams,
		PxVehicleArrayData<const PxVehicleSuspensionState>& suspensionStates,
		PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspensionComplianceStates,
		PxVehicleArrayData<const PxVehicleRoadGeometryState>& wheelRoadGeomStates,
		PxVehicleArrayData<const PxVehicleTireDirectionState>& tireDirectionStates,
		PxVehicleArrayData<const PxVehicleTireStickyState>& tireStickyStates,
		PxVehiclePhysXConstraints*& constraints)
	{
		axleDescription = &mBaseParams.axleDescription;
		rigidBodyState = &mBaseState.rigidBodyState;
		suspensionParams.setData(mBaseParams.suspensionParams);
		suspensionLimitParams.setData(mPhysXParams.physxSuspensionLimitConstraintParams);
		suspensionStates.setData(mBaseState.suspensionStates);
		suspensionComplianceStates.setData(mBaseState.suspensionComplianceStates);
		wheelRoadGeomStates.setData(mBaseState.roadGeomStates);
		tireDirectionStates.setData(mBaseState.tireDirectionStates);
		tireStickyStates.setData(mBaseState.tireStickyStates);
		constraints = &mPhysXState.physxConstraints;
	}

	virtual void getDataForPhysXRoadGeometrySceneQueryComponent(
		const PxVehicleAxleDescription*& axleDescription,
		const PxVehiclePhysXRoadGeometryQueryParams*& roadGeomParams,
		PxVehicleArrayData<const PxReal>& steerResponseStates,
		const PxVehicleRigidBodyState*& rigidBodyState,
		PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
		PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
		PxVehicleArrayData<const PxVehiclePhysXMaterialFrictionParams>& materialFrictionParams,
		PxVehicleArrayData<PxVehicleRoadGeometryState>& roadGeometryStates,
		PxVehicleArrayData<PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeometryStates)
	{
		axleDescription = &mBaseParams.axleDescription;
		roadGeomParams = &mPhysXParams.physxRoadGeometryQueryParams;
		steerResponseStates.setData(mBaseState.steerCommandResponseStates);
		rigidBodyState = &mBaseState.rigidBodyState;
		wheelParams.setData(mBaseParams.wheelParams);
		suspensionParams.setData(mBaseParams.suspensionParams);
		materialFrictionParams.setData(mPhysXParams.physxMaterialFrictionParams);
		roadGeometryStates.setData(mBaseState.roadGeomStates);
		physxRoadGeometryStates.setEmpty();
	}


	//Parameters and states of the vehicle's physx integration.
	PhysXIntegrationParams mPhysXParams;
	PhysXIntegrationState mPhysXState;

	//The commands that will control the vehicle
	//
	// Note that this is not related to a PhysX actor based vehicle as such but
	// put in here to be shared by all vehicle types that will be based on this
	// class. It keeps the code simpler for the purpose of the snippets.
	//
	PxVehicleCommandState mCommandState;
};

}//namespace snippetvehicle2
