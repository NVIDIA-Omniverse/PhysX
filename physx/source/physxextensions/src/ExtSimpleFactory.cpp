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

#include "foundation/PxMathUtils.h"
#include "foundation/PxQuat.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxPlaneGeometry.h"
#include "extensions/PxRigidBodyExt.h"
#include "extensions/PxSimpleFactory.h"
#include "PxPhysics.h"
#include "PxScene.h"
#include "PxRigidStatic.h"
#include "PxRigidStatic.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

#include "foundation/PxUtilities.h"
#include "foundation/PxInlineArray.h"

using namespace physx;

static bool isDynamicGeometry(PxGeometryType::Enum type)
{
	return type == PxGeometryType::eBOX 
		|| type == PxGeometryType::eSPHERE
		|| type == PxGeometryType::eCAPSULE
		|| type == PxGeometryType::eCONVEXMESH;
}

namespace physx
{
PxRigidDynamic* PxCreateDynamic(PxPhysics& sdk, 
								const PxTransform& transform, 
								PxShape& shape,
								PxReal density)
{
	PX_CHECK_AND_RETURN_NULL(transform.isValid(), "PxCreateDynamic: transform is not valid.");

	PxRigidDynamic* actor = sdk.createRigidDynamic(transform);
	if(actor)
	{
		if(!actor->attachShape(shape))
		{
			actor->release();
			return NULL;
		}
		if(!PxRigidBodyExt::updateMassAndInertia(*actor, density))
		{
			actor->release();
			return NULL;
		}
	}
	return actor;
}

PxRigidDynamic* PxCreateDynamic(PxPhysics& sdk, 
								const PxTransform& transform, 
								const PxGeometry& geometry,
							    PxMaterial& material, 
								PxReal density,
								const PxTransform& shapeOffset)
{
	PX_CHECK_AND_RETURN_NULL(transform.isValid(), "PxCreateDynamic: transform is not valid.");
	PX_CHECK_AND_RETURN_NULL(shapeOffset.isValid(), "PxCreateDynamic: shapeOffset is not valid.");

	if(!isDynamicGeometry(geometry.getType()) || density <= 0.0f)
	    return NULL;

	PxShape* shape = sdk.createShape(geometry, material, true);
	if(!shape)
		return NULL;

	shape->setLocalPose(shapeOffset);

	PxRigidDynamic* body = shape ? PxCreateDynamic(sdk, transform, *shape, density) : NULL;
	shape->release();
	return body;
}

PxRigidDynamic* PxCreateKinematic(PxPhysics& sdk, 
								  const PxTransform& transform, 
								  PxShape& shape,
								  PxReal density)
{
	PX_CHECK_AND_RETURN_NULL(transform.isValid(), "PxCreateKinematic: transform is not valid.");

	bool isDynGeom = isDynamicGeometry(shape.getGeometry().getType());
	if(isDynGeom && density <= 0.0f)
	    return NULL;

	PxRigidDynamic* actor = sdk.createRigidDynamic(transform);	
	if(actor)
	{
		actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		if(!isDynGeom)
			shape.setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);

		actor->attachShape(shape);

		if(isDynGeom)
			PxRigidBodyExt::updateMassAndInertia(*actor, density);
		else		
		{
			actor->setMass(1.f);
			actor->setMassSpaceInertiaTensor(PxVec3(1.f,1.f,1.f));
		}
	}

	return actor;
}

PxRigidDynamic* PxCreateKinematic(PxPhysics& sdk, 
								  const PxTransform& transform, 
								  const PxGeometry& geometry, 
								  PxMaterial& material,
								  PxReal density,
								  const PxTransform& shapeOffset)
{
	PX_CHECK_AND_RETURN_NULL(transform.isValid(), "PxCreateKinematic: transform is not valid.");
	PX_CHECK_AND_RETURN_NULL(shapeOffset.isValid(), "PxCreateKinematic: shapeOffset is not valid.");

	bool isDynGeom = isDynamicGeometry(geometry.getType());
	if(isDynGeom && density <= 0.0f)
	    return NULL;

	PxShape* shape = sdk.createShape(geometry, material, true);
	if(!shape)
		return NULL;

	shape->setLocalPose(shapeOffset);

	PxRigidDynamic* body = PxCreateKinematic(sdk, transform, *shape, density);
	shape->release();
	return body;
}

PxRigidStatic* PxCreateStatic(PxPhysics& sdk, 
							  const PxTransform& transform, 
							  PxShape& shape)
{
	PX_CHECK_AND_RETURN_NULL(transform.isValid(), "PxCreateStatic: transform is not valid.");

	PxRigidStatic* s = sdk.createRigidStatic(transform);
	if(s)
		s->attachShape(shape);
	return s;
}

PxRigidStatic*	PxCreateStatic(PxPhysics& sdk,
							   const PxTransform& transform,
							   const PxGeometry& geometry,
							   PxMaterial& material,
							   const PxTransform& shapeOffset)
{
	PX_CHECK_AND_RETURN_NULL(transform.isValid(), "PxCreateStatic: transform is not valid.");
	PX_CHECK_AND_RETURN_NULL(shapeOffset.isValid(), "PxCreateStatic: shapeOffset is not valid.");

	PxShape* shape = sdk.createShape(geometry, material, true);
	if(!shape)
		return NULL;

	shape->setLocalPose(shapeOffset);

	PxRigidStatic* s = PxCreateStatic(sdk, transform, *shape);
	shape->release();
	return s;
}

PxRigidStatic* PxCreatePlane(PxPhysics& sdk,
							 const PxPlane& plane,
							 PxMaterial& material)
{
	PX_CHECK_AND_RETURN_NULL(plane.n.isFinite(), "PxCreatePlane: plane normal is not valid.");

	if (!plane.n.isNormalized())
		return NULL;
	
	return PxCreateStatic(sdk, PxTransformFromPlaneEquation(plane), PxPlaneGeometry(), material);
}

PxShape* PxCloneShape(PxPhysics& physics, const PxShape& from, bool isExclusive)
{
	PxInlineArray<PxMaterial*, 64> materials;
	PxU16 materialCount = from.getNbMaterials();
	materials.resize(materialCount);
	from.getMaterials(materials.begin(), materialCount);

	PxShape* to = physics.createShape(from.getGeometry(), materials.begin(), materialCount, isExclusive, from.getFlags());

	to->setLocalPose(from.getLocalPose());
	to->setContactOffset(from.getContactOffset());
	to->setRestOffset(from.getRestOffset());
	to->setSimulationFilterData(from.getSimulationFilterData());
	to->setQueryFilterData(from.getQueryFilterData());
	to->setTorsionalPatchRadius(from.getTorsionalPatchRadius());
	to->setMinTorsionalPatchRadius(from.getMinTorsionalPatchRadius());
	return to;
}

static void copyStaticProperties(PxPhysics& physics, PxRigidActor& to, const PxRigidActor& from)
{
	PxInlineArray<PxShape*, 64> shapes;
	shapes.resize(from.getNbShapes());

	PxU32 shapeCount = from.getNbShapes();
	from.getShapes(shapes.begin(), shapeCount);

	for(PxU32 i = 0; i < shapeCount; i++)
	{
		PxShape* s = shapes[i];
		if(!s->isExclusive())
			to.attachShape(*s);
		else
		{
			PxShape* newShape = physx::PxCloneShape(physics, *s, true);
			to.attachShape(*newShape);
			newShape->release();		
		}
	}

	to.setActorFlags(from.getActorFlags());
	to.setOwnerClient(from.getOwnerClient());
	to.setDominanceGroup(from.getDominanceGroup());
}

PxRigidStatic* PxCloneStatic(PxPhysics& physicsSDK, 
							 const PxTransform& transform, 
							 const PxRigidActor& from)
{
	PxRigidStatic* to = physicsSDK.createRigidStatic(transform);
	if(!to)
		return NULL;

	copyStaticProperties(physicsSDK, *to, from);

	return to;
}

PxRigidDynamic* PxCloneDynamic(PxPhysics& physicsSDK, 
							   const PxTransform& transform,
							   const PxRigidDynamic& from)
{
	PxRigidDynamic* to = physicsSDK.createRigidDynamic(transform);
	if(!to)
		return NULL;

	copyStaticProperties(physicsSDK, *to, from);

	to->setRigidBodyFlags(from.getRigidBodyFlags());

	to->setMass(from.getMass());
	to->setMassSpaceInertiaTensor(from.getMassSpaceInertiaTensor());
	to->setCMassLocalPose(from.getCMassLocalPose());

	to->setLinearVelocity(from.getLinearVelocity());
	to->setAngularVelocity(from.getAngularVelocity());

	to->setLinearDamping(from.getLinearDamping());
	to->setAngularDamping(from.getAngularDamping());

	PxU32 posIters, velIters;
	from.getSolverIterationCounts(posIters, velIters);
	to->setSolverIterationCounts(posIters, velIters);

	to->setMaxLinearVelocity(from.getMaxLinearVelocity());
	to->setMaxAngularVelocity(from.getMaxAngularVelocity());
	to->setMaxDepenetrationVelocity(from.getMaxDepenetrationVelocity());
	to->setSleepThreshold(from.getSleepThreshold());
	to->setStabilizationThreshold(from.getStabilizationThreshold());
	to->setMinCCDAdvanceCoefficient(from.getMinCCDAdvanceCoefficient());
	to->setContactReportThreshold(from.getContactReportThreshold());
	to->setMaxContactImpulse(from.getMaxContactImpulse());

	PxTransform target;
	if (from.getKinematicTarget(target))
		to->setKinematicTarget(target);

	to->setRigidDynamicLockFlags(from.getRigidDynamicLockFlags());

	return to;
}

static PxTransform scalePosition(const PxTransform& t, PxReal scale)
{
	return PxTransform(t.p*scale, t.q);
}

void PxScaleRigidActor(PxRigidActor& actor, PxReal scale, bool scaleMassProps)
{
	PX_CHECK_AND_RETURN(scale > 0,
		"PxScaleRigidActor requires that the scale parameter is greater than zero");

	PxInlineArray<PxShape*, 64> shapes;
	shapes.resize(actor.getNbShapes());
	actor.getShapes(shapes.begin(), shapes.size());

	for(PxU32 i=0;i<shapes.size();i++)
	{
		shapes[i]->setLocalPose(scalePosition(shapes[i]->getLocalPose(), scale));		
		PxGeometryHolder h(shapes[i]->getGeometry());

		switch(h.getType())
		{
		case PxGeometryType::eSPHERE:	
			h.sphere().radius *= scale;			
			break;
		case PxGeometryType::ePLANE:
			break;
		case PxGeometryType::eCAPSULE:
			h.capsule().halfHeight *= scale;
			h.capsule().radius *= scale;
			break;
		case PxGeometryType::eBOX:
			h.box().halfExtents *= scale;
			break;
		case PxGeometryType::eCONVEXMESH:
			h.convexMesh().scale.scale *= scale;
			break;
		case PxGeometryType::eTRIANGLEMESH:
			h.triangleMesh().scale.scale *= scale;
			break;
		case PxGeometryType::eHEIGHTFIELD:
			h.heightField().heightScale *= scale;
			h.heightField().rowScale *= scale;
			h.heightField().columnScale *= scale;
			break;
		default:
			PX_ASSERT(0);
		}
		shapes[i]->setGeometry(h.any());
	}

	if(!scaleMassProps)
		return;

	PxRigidDynamic* dynamic = (&actor)->is<PxRigidDynamic>();
	if(!dynamic)
		return;

	PxReal scale3 = scale*scale*scale;
	dynamic->setMass(dynamic->getMass()*scale3);
	dynamic->setMassSpaceInertiaTensor(dynamic->getMassSpaceInertiaTensor()*scale3*scale*scale);
	dynamic->setCMassLocalPose(scalePosition(dynamic->getCMassLocalPose(), scale));
}
}
