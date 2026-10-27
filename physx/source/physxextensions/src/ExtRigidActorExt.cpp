// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "extensions/PxRigidActorExt.h"
#include "foundation/PxFPU.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxInlineArray.h"
#include "geometry/PxGeometryQuery.h"

#include "cooking/PxBVHDesc.h"
#include "cooking/PxCooking.h"

using namespace physx;

PxBounds3* PxRigidActorExt::getRigidActorShapeLocalBoundsList(const PxRigidActor& actor, PxU32& numBounds)
{
	const PxU32 numShapes = actor.getNbShapes();
	if(numShapes == 0)
		return NULL;
	
	PxInlineArray<PxShape*, 64> shapes("PxShape*"); 
	shapes.resize(numShapes);

	actor.getShapes(shapes.begin(), shapes.size());

	PxU32 numSqShapes = 0;
	for(PxU32 i=0; i<numShapes; i++)
	{
		if(shapes[i]->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
			numSqShapes++;
	}

	PxBounds3* bounds = PX_ALLOCATE(PxBounds3, numSqShapes, "PxBounds3");

	numSqShapes = 0;
	{
		PX_SIMD_GUARD	// PT: external guard because we use PxGeometryQueryFlag::Enum(0) below
		for(PxU32 i=0; i<numShapes; i++)
		{
			if(shapes[i]->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
				PxGeometryQuery::computeGeomBounds(bounds[numSqShapes++], shapes[i]->getGeometry(), shapes[i]->getLocalPose(), 0.0f, 1.0f, PxGeometryQueryFlag::Enum(0));
		}
	}

	numBounds = numSqShapes;
	return bounds;
}

PxBVH* PxRigidActorExt::createBVHFromActor(PxPhysics& physics, const PxRigidActor& actor)
{
	PxU32 nbBounds = 0;
	PxBounds3* bounds = PxRigidActorExt::getRigidActorShapeLocalBoundsList(actor, nbBounds);

	PxBVHDesc bvhDesc;
	bvhDesc.bounds.count	= nbBounds;
	bvhDesc.bounds.data		= bounds;
	bvhDesc.bounds.stride	= sizeof(PxBounds3);

	PxBVH* bvh = PxCreateBVH(bvhDesc, physics.getPhysicsInsertionCallback());

	PX_FREE(bounds);
	return bvh;
}

