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

#include "common/PxBase.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxHeightField.h"
#include "extensions/PxJoint.h"
#include "extensions/PxConstraintExt.h"
#include "extensions/PxCollectionExt.h"
#include "PxShape.h"
#include "PxMaterial.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxAggregate.h"
#include "PxPhysics.h"
#include "PxScene.h"
#include "PxPruningStructure.h"


#include "foundation/PxArray.h"

using namespace physx;

void PxCollectionExt::releaseObjects(PxCollection& collection, bool releaseExclusiveShapes)
{
	PxArray<PxBase*> releasableObjects;

	for (PxU32 i = 0; i < collection.getNbObjects(); ++i)
	{	
		PxBase* s = &collection.getObject(i);
		// pruning structure must be released before its actors
		if(s->is<PxPruningStructure>())
		{
			if(!releasableObjects.empty())
			{
				PxBase* first = releasableObjects[0];
				releasableObjects.pushBack(first);
				releasableObjects[0] = s;
			}
		}
		else
		{
			if (s->isReleasable() && (releaseExclusiveShapes || !s->is<PxShape>() || !s->is<PxShape>()->isExclusive()))
				releasableObjects.pushBack(s);
		}
	}

	for (PxU32 i = 0; i < releasableObjects.size(); ++i)
		releasableObjects[i]->release();		

	while (collection.getNbObjects() > 0)
		collection.remove(collection.getObject(0));
}


void PxCollectionExt::remove(PxCollection& collection, PxType concreteType, PxCollection* to)
{	
	PxArray<PxBase*> removeObjects;
	
	for (PxU32 i = 0; i < collection.getNbObjects(); i++)
	{
		PxBase& object = collection.getObject(i);
		if(concreteType == object.getConcreteType())
		{
			if(to)
			   to->add(object);	

			removeObjects.pushBack(&object);
		}
	}

	for (PxU32 i = 0; i < removeObjects.size(); ++i)
		collection.remove(*removeObjects[i]);
}

PxCollection* PxCollectionExt::createCollection(PxPhysics& physics)
{
	PxCollection* collection = PxCreateCollection();
	if (!collection)
		return NULL;

	// Collect convexes
	{
		PxArray<PxConvexMesh*> objects(physics.getNbConvexMeshes());
		const PxU32 nb = physics.getConvexMeshes(objects.begin(), objects.size());
		PX_ASSERT(nb == objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}

	// Collect triangle meshes
	{
		PxArray<PxTriangleMesh*> objects(physics.getNbTriangleMeshes());
		const PxU32 nb = physics.getTriangleMeshes(objects.begin(), objects.size());

		PX_ASSERT(nb == objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}

	// Collect heightfields
	{
		PxArray<PxHeightField*> objects(physics.getNbHeightFields());
		const PxU32 nb = physics.getHeightFields(objects.begin(), objects.size());

		PX_ASSERT(nb == objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}

	// Collect materials
	{
		PxArray<PxMaterial*> objects(physics.getNbMaterials());
		const PxU32 nb = physics.getMaterials(objects.begin(), objects.size());

		PX_ASSERT(nb == objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}

	// Collect shapes
	{
		PxArray<PxShape*> objects(physics.getNbShapes());
		const PxU32 nb = physics.getShapes(objects.begin(), objects.size());

		PX_ASSERT(nb == objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}
	return collection;
}

PxCollection* PxCollectionExt::createCollection(PxScene& scene)
{
	PxCollection* collection = PxCreateCollection();
	if (!collection)
		return NULL;

	// Collect actors
	{
		PxActorTypeFlags selectionFlags = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;


		PxArray<PxActor*> objects(scene.getNbActors(selectionFlags));
		const PxU32 nb = scene.getActors(selectionFlags, objects.begin(), objects.size());

		PX_ASSERT(nb==objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}


	// Collect constraints
	{
		PxArray<PxConstraint*> objects(scene.getNbConstraints());
		const PxU32 nb = scene.getConstraints(objects.begin(), objects.size());

		PX_ASSERT(nb==objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
		{
			PxU32 typeId;
			PxJoint* joint = reinterpret_cast<PxJoint*>(objects[i]->getExternalReference(typeId));
			if(typeId == PxConstraintExtIDs::eJOINT)
				collection->add(*joint);
		}
	}

	// Collect articulations
	{
		PxArray<PxArticulationReducedCoordinate*> objects(scene.getNbArticulations());
		const PxU32 nb = scene.getArticulations(objects.begin(), objects.size());

		PX_ASSERT(nb==objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}

	// Collect aggregates
	{
		PxArray<PxAggregate*> objects(scene.getNbAggregates());
		const PxU32 nb = scene.getAggregates(objects.begin(), objects.size());

		PX_ASSERT(nb==objects.size());
		PX_UNUSED(nb);

		for(PxU32 i=0;i<objects.size();i++)
			collection->add(*objects[i]);
	}

	return collection;
}
