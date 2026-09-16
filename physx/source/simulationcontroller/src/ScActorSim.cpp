// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScActorSim.h"
#include "ScActorCore.h"
#include "ScElementSim.h"
#include "ScScene.h"
#include "ScInteraction.h"

using namespace physx;
using namespace Sc;

static const PxFilterObjectType::Enum gFilterType[PxActorType::eACTOR_COUNT] =
{
	PxFilterObjectType::eRIGID_STATIC,		// PxActorType::eRIGID_STATIC
	PxFilterObjectType::eRIGID_DYNAMIC,		// PxActorType::eRIGID_DYNAMIC
	PxFilterObjectType::eARTICULATION,		// PxActorType::eARTICULATION_LINK
	PxFilterObjectType::eDEFORMABLE_SURFACE,// PxActorType::eDEFORMABLE_SURFACE
	PxFilterObjectType::eDEFORMABLE_VOLUME,	// PxActorType::eDEFORMABLE_VOLUME
	PxFilterObjectType::ePARTICLESYSTEM,	// PxActorType::ePBD_PARTICLESYSTEM
};

static const PxU32 gFilterFlagEx[PxActorType::eACTOR_COUNT] =
{
	PxFilterObjectFlagEx::eRIGID_STATIC,										// PxActorType::eRIGID_STATIC
	PxFilterObjectFlagEx::eRIGID_DYNAMIC,										// PxActorType::eRIGID_DYNAMIC
	PxFilterObjectFlagEx::eRIGID_DYNAMIC,										// PxActorType::eARTICULATION_LINK
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::eDEFORMABLE_SURFACE,	// PxActorType::eDEFORMABLE_SURFACE
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::eDEFORMABLE_VOLUME,	// PxActorType::eDEFORMABLE_VOLUME
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::ePARTICLESYSTEM,		// PxActorType::ePBD_PARTICLESYSTEM
};

// PT: if this breaks, you need to update the above table
PX_COMPILE_TIME_ASSERT(PxActorType::eACTOR_COUNT==6);

// PT: make sure that the highest flag fits into 16bit
PX_COMPILE_TIME_ASSERT(PxFilterObjectFlagEx::eLAST<=0xffff);

Sc::ActorSim::ActorSim(Scene& scene, ActorCore& core) :
	mScene					(scene),
	mCore					(core),
	mActiveListIndex		(SC_NOT_IN_SCENE_INDEX),
	mActiveCompoundListIndex(SC_NOT_IN_SCENE_INDEX),
	mNodeIndex				(PX_INVALID_NODE),
	mInternalFlags			(0)
{
	core.setSim(this);
	mId = scene.getActorIDTracker().createID();

	{
		PX_ASSERT(gFilterType[PxActorType::eRIGID_STATIC] == PxFilterObjectType::eRIGID_STATIC);
		PX_ASSERT(gFilterType[PxActorType::eRIGID_DYNAMIC] == PxFilterObjectType::eRIGID_DYNAMIC);
		PX_ASSERT(gFilterType[PxActorType::eARTICULATION_LINK] == PxFilterObjectType::eARTICULATION);
		PX_ASSERT(gFilterType[PxActorType::eDEFORMABLE_SURFACE] == PxFilterObjectType::eDEFORMABLE_SURFACE);
		PX_ASSERT(gFilterType[PxActorType::eDEFORMABLE_VOLUME] == PxFilterObjectType::eDEFORMABLE_VOLUME);
		PX_ASSERT(gFilterType[PxActorType::ePBD_PARTICLESYSTEM] == PxFilterObjectType::ePARTICLESYSTEM);

		const PxActorType::Enum actorType = getActorType();

		PxFilterObjectAttributes filterAttr = 0;
		setFilterObjectAttributeType(filterAttr, gFilterType[actorType]);

		filterAttr |= gFilterFlagEx[actorType];

		mFilterFlags = PxTo16(filterAttr);
	}
}

Sc::ActorSim::~ActorSim()
{
	mInteractions.releaseMem(*this);

	mScene.getActorIDTracker().releaseID(mId);
}

void Sc::ActorSim::registerInteractionInActor(Interaction* interaction)
{
	const PxU32 id = mInteractions.size();
	mInteractions.pushBack(interaction, *this);
	interaction->setActorId(this, id);
}

void Sc::ActorSim::unregisterInteractionFromActor(Interaction* interaction)
{
	const PxU32 i = interaction->getActorId(this);
	PX_ASSERT(i < mInteractions.size());
	mInteractions.replaceWithLast(i); 
	if(i<mInteractions.size())
	{
		if(mInteractions[i])	// ### DEFENSIVE  PT: for OM-122969
			mInteractions[i]->setActorId(this, i);
		else
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Sc::ActorSim::unregisterInteractionFromActor: found null interaction!");
	}
}

void Sc::ActorSim::reallocInteractions(Sc::Interaction**& mem, PxU32& capacity, PxU32 size, PxU32 requiredMinCapacity)
{
	Interaction** newMem;
	PxU32 newCapacity;

	if(requiredMinCapacity==0)
	{
		newCapacity = 0;
		newMem = 0;
	}
	else if(requiredMinCapacity<=INLINE_INTERACTION_CAPACITY)
	{
		newCapacity = INLINE_INTERACTION_CAPACITY;
		newMem = mInlineInteractionMem;
	}
	else
	{
		newCapacity = PxNextPowerOfTwo(requiredMinCapacity-1);
		newMem = reinterpret_cast<Interaction**>(mScene.allocatePointerBlock(newCapacity));
	}

	PX_ASSERT(newCapacity >= requiredMinCapacity && requiredMinCapacity>=size);

	if(mem)
	{
		PxMemCopy(newMem, mem, size*sizeof(Interaction*));

		if(mem!=mInlineInteractionMem)
			mScene.deallocatePointerBlock(reinterpret_cast<void**>(mem), capacity);
	}
	
	capacity = newCapacity;
	mem = newMem;
}

void Sc::ActorSim::setActorsInteractionsDirty(InteractionDirtyFlag::Enum flag, const ActorSim* other, PxU8 interactionFlag)
{
	PxU32 size = getActorInteractionCount();
	Interaction** interactions = getActorInteractions();
	while(size--)
	{
		Interaction* interaction = *interactions++;
		if((!other || other == &interaction->getActorSim0() || other == &interaction->getActorSim1()) && (interaction->readInteractionFlag(interactionFlag)))
			interaction->setDirty(flag);
	}
}
