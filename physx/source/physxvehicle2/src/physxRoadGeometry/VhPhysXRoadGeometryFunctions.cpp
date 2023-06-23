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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/roadGeometry/PxVehicleRoadGeometryState.h"
#include "vehicle2/physxRoadGeometry/PxVehiclePhysXRoadGeometryFunctions.h"
#include "vehicle2/physxRoadGeometry/PxVehiclePhysXRoadGeometryParams.h"
#include "vehicle2/physxRoadGeometry/PxVehiclePhysXRoadGeometryState.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle2/suspension/PxVehicleSuspensionHelpers.h"

#include "extensions/PxRigidBodyExt.h"

#include "PxScene.h"
#include "PxShape.h"
#include "PxRigidActor.h"
#include "PxMaterial.h"
#include "geometry/PxMeshScale.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxGeometryQuery.h"

namespace physx
{
namespace vehicle2
{

PX_FORCE_INLINE PxF32 computeMaterialFriction(const PxShape* hitShape, const PxU32 hitFaceIndex, 
	const PxVehiclePhysXMaterialFrictionParams& materialFrictionParams, PxMaterial*& hitMaterial)
{
	PxBaseMaterial* baseMaterial = hitShape->getMaterialFromInternalFaceIndex(hitFaceIndex);
	PX_ASSERT(!baseMaterial || baseMaterial->getConcreteType()==PxConcreteType::eMATERIAL);
	hitMaterial = static_cast<PxMaterial*>(baseMaterial);
	PxReal hitFriction = materialFrictionParams.defaultFriction;
	for(PxU32 i = 0; i < materialFrictionParams.nbMaterialFrictions; i++)
	{
		if(materialFrictionParams.materialFrictions[i].material == hitMaterial)
		{
			hitFriction = materialFrictionParams.materialFrictions[i].friction;
			break;
		}
	}
	return hitFriction;
}

PX_FORCE_INLINE PxVec3 computeVelocity(const PxRigidActor& actor, const PxVec3& hitPoint)
{
	return actor.is<PxRigidBody>() ? PxRigidBodyExt::getVelocityAtPos(*actor.is<PxRigidBody>(), hitPoint) : PxVec3(PxZero);
}

template<typename THitBuffer>
PX_FORCE_INLINE void copyHitInfo(const THitBuffer& hitBuffer, PxMaterial* hitMaterial,
	PxVehiclePhysXRoadGeometryQueryState& physxRoadGeometryState)
{
	physxRoadGeometryState.actor = hitBuffer.actor;
	physxRoadGeometryState.shape = hitBuffer.shape;
	physxRoadGeometryState.material = hitMaterial;
	physxRoadGeometryState.hitPosition = hitBuffer.position;
}

void PxVehiclePhysXRoadGeometryQueryUpdate
(const PxVehicleWheelParams& wheelParams, const PxVehicleSuspensionParams& suspParams,
 const PxVehiclePhysXRoadGeometryQueryType::Enum queryType, 
 PxQueryFilterCallback* filterCallback, const PxQueryFilterData& filterData,
 const PxVehiclePhysXMaterialFrictionParams& materialFrictionParams,
 const PxF32 steerAngle, const PxVehicleRigidBodyState& rigidBodyState, 
 const PxScene& scene, const PxConvexMesh* unitCylinderSweepMesh, 
 const PxVehicleFrame& frame,
 PxVehicleRoadGeometryState& roadGeomState,
 PxVehiclePhysXRoadGeometryQueryState* physxRoadGeometryState)
{
	if(PxVehiclePhysXRoadGeometryQueryType::eRAYCAST == queryType)
	{
		//Assume no hits until we know otherwise.
		roadGeomState.setToDefault();

		//Compute the start pos, dir and length of raycast.
		PxVec3 v, w;
		PxF32 dist;
		PxVehicleComputeSuspensionRaycast(frame, wheelParams, suspParams, steerAngle, rigidBodyState.pose, v, w, dist);

		//Perform the raycast.
		PxRaycastBuffer buff;
		scene.raycast(v, w, dist, buff, PxHitFlag::eDEFAULT, filterData, filterCallback);

		//Process the raycast result.
		if(buff.hasBlock && buff.block.distance != 0.0f)
		{
			const PxPlane hitPlane(v + w * buff.block.distance, buff.block.normal);
			roadGeomState.plane = hitPlane;
			roadGeomState.hitState = true;
			PxMaterial* hitMaterial;
			roadGeomState.friction = computeMaterialFriction(buff.block.shape, buff.block.faceIndex, materialFrictionParams,
				hitMaterial);
			roadGeomState.velocity = computeVelocity(*buff.block.actor, buff.block.position);

			if (physxRoadGeometryState)
			{
				copyHitInfo(buff.block, hitMaterial, *physxRoadGeometryState);
			}
		}
		else
		{
			if (physxRoadGeometryState)
				physxRoadGeometryState->setToDefault();
		}
	}
	else if(PxVehiclePhysXRoadGeometryQueryType::eSWEEP == queryType)
	{
		PX_ASSERT(unitCylinderSweepMesh);

		//Assume no hits until we know otherwise.
		roadGeomState.setToDefault();

		//Compute the start pose, dir and length of sweep.
		PxTransform T;
		PxVec3 w;
		PxF32 dist;
		PxVehicleComputeSuspensionSweep(frame, suspParams, steerAngle, rigidBodyState.pose, T, w, dist);

		//Scale the unit cylinder.
		const PxVec3 scale = PxVehicleComputeTranslation(frame, wheelParams.radius, wheelParams.halfWidth, wheelParams.radius).abs();
		const PxMeshScale meshScale(scale, PxQuat(PxIdentity));
		const PxConvexMeshGeometry convMeshGeom(const_cast<PxConvexMesh*>(unitCylinderSweepMesh), meshScale);

		//Perform the sweep.
		PxSweepBuffer buff;
		scene.sweep(convMeshGeom, T,  w,  dist, buff, PxHitFlag::eDEFAULT | PxHitFlag::eMTD, filterData, filterCallback);

		//Process the sweep result.
		if (buff.hasBlock && buff.block.distance >= 0.0f)
		{
			//Sweep started outside scene geometry.
			const PxPlane hitPlane(buff.block.position, buff.block.normal);
			roadGeomState.plane = hitPlane;
			roadGeomState.hitState = true;
			PxMaterial* hitMaterial;
			roadGeomState.friction = computeMaterialFriction(buff.block.shape, buff.block.faceIndex, materialFrictionParams,
				hitMaterial);
			roadGeomState.velocity = computeVelocity(*buff.block.actor, buff.block.position);

			if (physxRoadGeometryState)
			{
				copyHitInfo(buff.block, hitMaterial, *physxRoadGeometryState);
			}
		}
		else if (buff.hasBlock && buff.block.distance < 0.0f)
		{
			//The sweep started inside scene geometry.
			//We want to have another go but this time starting outside the hit geometry because this is the most reliable 
			//way to get a hit plane.
			//-buff.block.distance is the distance we need to move along buff.block.normal to be outside the hit geometry.
			//Note that buff.block.distance can be a vanishingly small number.  Moving along the normal by a vanishingly 
			//small number might not push us out of overlap due to numeric precision of the overlap test.
			//We want to move a numerically significant distance to guarantee that we change the overlap status
			//at the start pose of the sweep.
			//We achieve this by choosing a minimum translation that is numerically significant.
			//Any number will do but we choose the wheel radius because this ought to be a numerically significant value.  
			//We're only sweeping against the hit shape and not against the scene
			//so we don't risk hitting other stuff by moving a numerically significant distance.
			const PxVec3 unitDir = -buff.block.normal;
			const PxF32 maxDist = PxMax(wheelParams.radius, -buff.block.distance);
			const PxGeometry& geom0 = convMeshGeom;
			const PxTransform pose0(T.p + buff.block.normal*(maxDist*1.01f), T.q);
			const PxGeometry& geom1 = buff.block.shape->getGeometry();
			const PxTransform pose1 = buff.block.actor->getGlobalPose()*buff.block.shape->getLocalPose();
			PxGeomSweepHit buff2;
			const bool b2 = PxGeometryQuery::sweep(
				unitDir, maxDist*1.02f,
				geom0, pose0, geom1, pose1, buff2, PxHitFlag::eDEFAULT | PxHitFlag::eMTD);

			if (b2 && buff2.distance > 0.0f)
			{
				//Sweep started outside scene geometry.
				const PxPlane hitPlane(buff2.position, buff2.normal);
				roadGeomState.plane = hitPlane;
				roadGeomState.hitState = true;
				PxMaterial* hitMaterial;
				roadGeomState.friction = computeMaterialFriction(buff.block.shape, buff.block.faceIndex, materialFrictionParams,
					hitMaterial);
				roadGeomState.velocity = computeVelocity(*buff.block.actor, buff.block.position);

				if (physxRoadGeometryState)
				{
					copyHitInfo(buff.block, hitMaterial, *physxRoadGeometryState);
				}
			}
			else
			{
				if (physxRoadGeometryState)
					physxRoadGeometryState->setToDefault();
			}
		}
		else
		{
			if (physxRoadGeometryState)
				physxRoadGeometryState->setToDefault();
		}
	}

}

} //namespace vehicle2
} //namespace physx
