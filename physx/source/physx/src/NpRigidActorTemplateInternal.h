// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_RIGID_ACTOR_TEMPLATE_INTERNAL_H
#define NP_RIGID_ACTOR_TEMPLATE_INTERNAL_H

namespace physx
{

template<class APIClass, class T>
static PX_FORCE_INLINE void removeRigidActorT(T& rigidActor)
{
	NpScene* s = rigidActor.getNpScene();
	NP_WRITE_CHECK(s);

	//Remove constraints (if any constraint is attached to the actor).
	rigidActor.NpRigidActorTemplate<APIClass>::removeConstraints(rigidActor);

#if PX_SUPPORT_GPU_PHYSX
	//Remove attachments (if any attachment is attached to the actor).
	rigidActor.NpRigidActorTemplate<APIClass>::removeAttachments(rigidActor, true);

	//Remove element filters (if any element filter is attached to the actor).
	rigidActor.NpRigidActorTemplate<APIClass>::removeElementFilters(rigidActor, true);
#endif

	//Remove from aggregate (if it is in an aggregate).
	rigidActor.NpActorTemplate<APIClass>::removeFromAggregate(rigidActor);

	//Remove from scene (if it is in a scene).
	PxSceneQuerySystem* sqManager = NULL;
	if(s)
	{
		sqManager = &s->getSQAPI();
		const bool noSim = rigidActor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);
		s->scRemoveActor(rigidActor, true, noSim);
	}

	//Remove associated shapes.
	rigidActor.NpRigidActorTemplate<APIClass>::removeShapes(sqManager);
}

template<class APIClass, class T>
static PX_FORCE_INLINE bool releaseRigidActorT(T& rigidActor)
{
	NpScene* s = rigidActor.getNpScene();
	NP_WRITE_CHECK(s);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(s, "PxActor::release() not allowed while simulation is running. Call will be ignored.", false)

	const bool noSim = rigidActor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);
	if(s && noSim)
	{
		// need to do it here because the Np-shape buffer will not be valid anymore after the removal below
		// and unlike simulation objects, there is no shape buffer in the simulation controller
		rigidActor.getShapeManager().clearShapesOnRelease(*s, rigidActor);
	}

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(&rigidActor, rigidActor.userData);

	//Remove constraints, aggregates, scene, shapes. 
	removeRigidActorT<APIClass, T>(rigidActor);

	if (s)
	{
		s->removeFromRigidActorList(rigidActor);
	}

	return true;
}

}

#endif
