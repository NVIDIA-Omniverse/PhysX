// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "ScInteraction.h"
#include "ScNPhaseCore.h"

using namespace physx;

Sc::Interaction::Interaction(ActorSim& actor0, ActorSim& actor1, InteractionType::Enum type, PxU8 flags) :
	mActor0				(actor0),
	mActor1				(actor1), 
	mSceneId			(PX_INVALID_INTERACTION_SCENE_ID), 
	mActorId0			(PX_INVALID_INTERACTION_ACTOR_ID),
	mActorId1			(PX_INVALID_INTERACTION_ACTOR_ID), 
	mInteractionType	(PxTo8(type)),
	mInteractionFlags	(flags),
	mDirtyFlags			(0)
{
	PX_ASSERT_WITH_MESSAGE(&actor0.getScene() == &actor1.getScene(),"Cannot create an interaction between actors belonging to different scenes.");
	PX_ASSERT(PxU32(type)<256);	// PT: type is now stored on a byte
}

void Sc::Interaction::addToDirtyList()
{
	getActorSim0().getScene().getNPhaseCore()->addToDirtyInteractionList(this);		
}

void Sc::Interaction::removeFromDirtyList()
{
	getActorSim0().getScene().getNPhaseCore()->removeFromDirtyInteractionList(this);
}

void Sc::Interaction::setClean(bool removeFromList)
{
	if (readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST))
	{
		if (removeFromList)  // if we process all dirty interactions anyway, then we can just clear the list at the end and save the work here.
			removeFromDirtyList();
		clearInteractionFlag(InteractionFlag::eIN_DIRTY_LIST);
	}

	mDirtyFlags = 0;
}
