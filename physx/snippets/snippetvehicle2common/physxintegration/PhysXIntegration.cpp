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

#include "PhysXIntegration.h"

namespace snippetvehicle2
{

void PhysXIntegrationParams::create
(const PxVehicleAxleDescription& axleDescription,
 const PxQueryFilterData& queryFilterData, PxQueryFilterCallback* queryFilterCallback,
 PxVehiclePhysXMaterialFriction* materialFrictions, const PxU32 nbMaterialFrictions, const PxReal defaultFriction,
 const PxTransform& actorCMassLocalPose,
 const PxVec3& actorBoxShapeHalfExtents, const PxTransform& actorBoxShapeLocalPose)
{
	physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
	physxRoadGeometryQueryParams.filterData = queryFilterData;
	physxRoadGeometryQueryParams.filterCallback = queryFilterCallback;

	for(PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32  wheelId = axleDescription.wheelIdsInAxleOrder[i];
		physxMaterialFrictionParams[wheelId].defaultFriction = defaultFriction;
		physxMaterialFrictionParams[wheelId].materialFrictions = materialFrictions;
		physxMaterialFrictionParams[wheelId].nbMaterialFrictions = nbMaterialFrictions;

		physxSuspensionLimitConstraintParams[wheelId].restitution = 0.0f;
		physxSuspensionLimitConstraintParams[wheelId].directionForSuspensionLimitConstraint = PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL;

		physxWheelShapeLocalPoses[wheelId] = PxTransform(PxIdentity);
	}

	physxActorCMassLocalPose = actorCMassLocalPose;
	physxActorBoxShapeHalfExtents = actorBoxShapeHalfExtents;
	physxActorBoxShapeLocalPose = actorBoxShapeLocalPose;
}

PhysXIntegrationParams PhysXIntegrationParams::transformAndScale
(const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const
{
	PhysXIntegrationParams r = *this;
	r.physxRoadGeometryQueryParams = physxRoadGeometryQueryParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
	{
		r.physxSuspensionLimitConstraintParams[i] = physxSuspensionLimitConstraintParams[i].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	}
	r.physxActorCMassLocalPose = PxVehicleTransformFrameToFrame(srcFrame, trgFrame, srcScale, trgScale, physxActorCMassLocalPose);
	r.physxActorBoxShapeHalfExtents = PxVehicleTransformFrameToFrame(srcFrame, trgFrame, srcScale, trgScale, physxActorBoxShapeHalfExtents);
	r.physxActorBoxShapeLocalPose = PxVehicleTransformFrameToFrame(srcFrame, trgFrame, srcScale, trgScale, physxActorBoxShapeLocalPose);
	return r;
}

void PhysXIntegrationState::create
(const BaseVehicleParams& baseParams, const PhysXIntegrationParams& physxParams,
 PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial)
{
	setToDefault();

	//physxActor needs to be populated with an actor and its shapes.
	{
		const PxVehiclePhysXRigidActorParams physxActorParams(baseParams.rigidBodyParams, NULL);
		const PxBoxGeometry boxGeom(physxParams.physxActorBoxShapeHalfExtents);
		const PxVehiclePhysXRigidActorShapeParams physxActorShapeParams(boxGeom, physxParams.physxActorBoxShapeLocalPose, defaultMaterial, PxShapeFlags(0), PxFilterData(), PxFilterData());
		const PxVehiclePhysXWheelParams physxWheelParams(baseParams.axleDescription, baseParams.wheelParams);
		const PxVehiclePhysXWheelShapeParams physxWheelShapeParams(defaultMaterial, PxShapeFlags(0), PxFilterData(), PxFilterData());

		PxVehiclePhysXActorCreate(
			baseParams.frame,
			physxActorParams, physxParams.physxActorCMassLocalPose, physxActorShapeParams,
			physxWheelParams, physxWheelShapeParams,
			physics, params,
			physxActor);
	}

	//physxConstraints needs to be populated with constraints.
	PxVehicleConstraintsCreate(baseParams.axleDescription, physics, *physxActor.rigidBody, physxConstraints);
}

void PhysXIntegrationState::destroy()
{
	PxVehicleConstraintsDestroy(physxConstraints);
	PxVehiclePhysXActorDestroy(physxActor);
}


void setPhysXIntegrationParams(const PxVehicleAxleDescription& axleDescription,
	PxVehiclePhysXMaterialFriction* physXMaterialFrictions, PxU32 nbPhysXMaterialFrictions,
	PxReal physXDefaultMaterialFriction, PhysXIntegrationParams& physXParams)
{
	//The physx integration params are hardcoded rather than loaded from file.
	const PxQueryFilterData queryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC);
	PxQueryFilterCallback* queryFilterCallback = NULL;
	const PxTransform physxActorCMassLocalPose(PxVec3(0.0f, 0.55f, 1.594f), PxQuat(PxIdentity));
	const PxVec3 physxActorBoxShapeHalfExtents(0.84097f, 0.65458f, 2.46971f);
	const PxTransform physxActorBoxShapeLocalPose(PxVec3(0.0f, 0.830066f, 1.37003f), PxQuat(PxIdentity));

	physXParams.create(
		axleDescription,
		queryFilterData, queryFilterCallback,
		physXMaterialFrictions, nbPhysXMaterialFrictions, physXDefaultMaterialFriction,
		physxActorCMassLocalPose,
		physxActorBoxShapeHalfExtents, physxActorBoxShapeLocalPose);
}


bool PhysXActorVehicle::initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial)
{
	mCommandState.setToDefault();

	if (!BaseVehicle::initialize())
		return false;
	
	if (!mPhysXParams.isValid(mBaseParams.axleDescription))
		return false;

	mPhysXState.create(mBaseParams, mPhysXParams, physics, params, defaultMaterial);

	return true;
}

void PhysXActorVehicle::destroy()
{
	mPhysXState.destroy();

	BaseVehicle::destroy();
}

void PhysXActorVehicle::setUpActor(PxScene& scene, const PxTransform& pose, const char* vehicleName)
{
	//Give the vehicle a start pose by appylying a pose to the PxRigidDynamic associated with the vehicle. 
	//This vehicle has components that are configured to read the pose from the PxRigidDynamic 
	//at the start of the vehicle simulation update and to write back an updated pose at the end of the 
	//vehicle simulation update. This allows PhysX to manage any collisions that might happen in-between 
	//each vehicle update. This is not essential but it is anticipated that this will be a typical component 
	//configuration. 
	mPhysXState.physxActor.rigidBody->setGlobalPose(pose);

	//Add the physx actor to the physx scene.
	//As described above, a vehicle may be coupled to a physx scene or it can be simulated without any reference to 
	//to a PxRigidDynamic or PxScene. This snippet vehicle employs a configuration that includes coupling to a PxScene and a 
	//PxRigidDynamic. This being the case, the PxRigidDynamic associated with the vehicle needs to be added to the 
	//PxScene instance.
	scene.addActor(*mPhysXState.physxActor.rigidBody);

	//Give the physx actor a name to help identification in PVD
	mPhysXState.physxActor.rigidBody->setName(vehicleName);
}

}//namespace snippetvehicle2
