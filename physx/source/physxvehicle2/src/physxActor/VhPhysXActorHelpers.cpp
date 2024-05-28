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

#include "vehicle2/rigidBody/PxVehicleRigidBodyParams.h"

#include "vehicle2/suspension/PxVehicleSuspensionHelpers.h"
#include "vehicle2/suspension/PxVehicleSuspensionParams.h"

#include "vehicle2/physxActor/PxVehiclePhysXActorStates.h"
#include "vehicle2/physxActor/PxVehiclePhysXActorHelpers.h"

#include "vehicle2/wheel/PxVehicleWheelParams.h"

#include "cooking/PxCooking.h"
#include "PxPhysics.h"
#include "PxRigidDynamic.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationLink.h"
#include "PxScene.h"
#include "extensions/PxDefaultStreams.h"


namespace physx
{
namespace vehicle2
{

void createShapes(
 const PxVehicleFrame& vehicleFrame,
 const PxVehiclePhysXRigidActorShapeParams& rigidActorShapeParams,
 const PxVehiclePhysXWheelParams& wheelParams, const PxVehiclePhysXWheelShapeParams& wheelShapeParams,
 PxRigidBody* rd,
 PxPhysics& physics, const PxCookingParams& params,
 PxVehiclePhysXActor& vehiclePhysXActor)
{
	//Create a shape for the vehicle body.
	{
		PxShape* shape = physics.createShape(rigidActorShapeParams.geometry, rigidActorShapeParams.material, true);
		shape->setLocalPose(rigidActorShapeParams.localPose);
		shape->setFlags(rigidActorShapeParams.flags);
		shape->setSimulationFilterData(rigidActorShapeParams.simulationFilterData);
		shape->setQueryFilterData(rigidActorShapeParams.queryFilterData);
		rd->attachShape(*shape);
		shape->release();
	}

	//Create shapes for wheels.
	for (PxU32 i = 0; i < wheelParams.axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = wheelParams.axleDescription.wheelIdsInAxleOrder[i];
		const PxF32 radius = wheelParams.wheelParams[wheelId].radius;
		const PxF32 halfWidth = wheelParams.wheelParams[wheelId].halfWidth;

		PxVec3 verts[32];
		for (PxU32 k = 0; k < 16; k++)
		{
			const PxF32 lng = radius * PxCos(k*2.0f*PxPi / 16.0f);
			const PxF32 lat = halfWidth;
			const PxF32 vrt = radius * PxSin(k*2.0f*PxPi / 16.0f);

			const PxVec3 pos0 = vehicleFrame.getFrame()*PxVec3(lng, lat, vrt);
			const PxVec3 pos1 = vehicleFrame.getFrame()*PxVec3(lng, -lat, vrt);
			verts[2 * k + 0] = pos0;
			verts[2 * k + 1] = pos1;
		}

		// Create descriptor for convex mesh
		PxConvexMeshDesc convexDesc;
		convexDesc.points.count = 32;
		convexDesc.points.stride = sizeof(PxVec3);
		convexDesc.points.data = verts;
		convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxConvexMesh* convexMesh = NULL;
		PxDefaultMemoryOutputStream buf;
		if (PxCookConvexMesh(params, convexDesc, buf))
		{
			PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
			convexMesh = physics.createConvexMesh(id);
		}

		PxConvexMeshGeometry convexMeshGeom(convexMesh);
		PxShape* wheelShape = physics.createShape(convexMeshGeom, wheelShapeParams.material, true);
		wheelShape->setFlags(wheelShapeParams.flags);
		wheelShape->setSimulationFilterData(wheelShapeParams.simulationFilterData);
		wheelShape->setQueryFilterData(wheelShapeParams.queryFilterData);

		rd->attachShape(*wheelShape);
		wheelShape->release();
		convexMesh->release();

		vehiclePhysXActor.wheelShapes[wheelId] = wheelShape;
	}
}

void PxVehiclePhysXActorCreate
(const PxVehicleFrame& vehicleFrame,
 const PxVehiclePhysXRigidActorParams& rigidActorParams, const PxTransform& rigidActorCmassLocalPose, 
 const PxVehiclePhysXRigidActorShapeParams& rigidActorShapeParams,
 const PxVehiclePhysXWheelParams& wheelParams, const PxVehiclePhysXWheelShapeParams& wheelShapeParams,
 PxPhysics& physics, const PxCookingParams& params,
 PxVehiclePhysXActor& vehiclePhysXActor)
{
	PxRigidDynamic* rd = physics.createRigidDynamic(PxTransform(PxIdentity));
	vehiclePhysXActor.rigidBody = rd;
	PxVehiclePhysXActorConfigure(rigidActorParams, rigidActorCmassLocalPose, *rd);
	createShapes(vehicleFrame, rigidActorShapeParams, wheelParams, wheelShapeParams, rd, physics, params, vehiclePhysXActor);
}

void PxVehiclePhysXActorConfigure
(const PxVehiclePhysXRigidActorParams& rigidActorParams, const PxTransform& rigidActorCmassLocalPose,
 PxRigidBody& rigidBody)
{
	rigidBody.setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	rigidBody.setCMassLocalPose(rigidActorCmassLocalPose);
	rigidBody.setMass(rigidActorParams.rigidBodyParams.mass);
	rigidBody.setMassSpaceInertiaTensor(rigidActorParams.rigidBodyParams.moi);
	rigidBody.setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	rigidBody.setName(rigidActorParams.physxActorName);
}

void PxVehiclePhysXArticulationLinkCreate
(const PxVehicleFrame& vehicleFrame,
 const PxVehiclePhysXRigidActorParams& rigidActorParams, const PxTransform& rigidActorCmassLocalPose,
 const PxVehiclePhysXRigidActorShapeParams& rigidActorShapeParams,
 const PxVehiclePhysXWheelParams& wheelParams, const PxVehiclePhysXWheelShapeParams& wheelShapeParams,
 PxPhysics& physics, const PxCookingParams& params,
 PxVehiclePhysXActor& vehiclePhysXActor)
{
	PxArticulationReducedCoordinate* art = physics.createArticulationReducedCoordinate();
	PxArticulationLink* link = art->createLink(NULL, PxTransform(PxIdentity));
	vehiclePhysXActor.rigidBody = link;
	PxVehiclePhysXActorConfigure(rigidActorParams, rigidActorCmassLocalPose, *link);
	createShapes(vehicleFrame, rigidActorShapeParams, wheelParams, wheelShapeParams, link, physics, params, vehiclePhysXActor);
}

void PxVehiclePhysXActorDestroy
(PxVehiclePhysXActor& vehiclePhysXActor)
{
	PxRigidDynamic* rd = vehiclePhysXActor.rigidBody->is<PxRigidDynamic>();
	PxArticulationLink* link = vehiclePhysXActor.rigidBody->is<PxArticulationLink>();
	if(rd)
	{
		rd->release();
		vehiclePhysXActor.rigidBody = NULL;
	}
	else if(link)
	{
		PxArticulationReducedCoordinate& articulation = link->getArticulation();
		PxScene* scene = articulation.getScene();
		if(scene)
		{
			scene->removeArticulation(articulation);
		}
		articulation.release();
		vehiclePhysXActor.rigidBody = NULL;
	}
}

} //namespace vehicle2
} //namespace physx
