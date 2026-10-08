// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScActorCore.h"
#include "ScActorSim.h"
#include "ScShapeCore.h"
#include "ScShapeSim.h"
#include "ScBodySim.h"

using namespace physx;

Sc::ActorCore::ActorCore(PxActorType::Enum actorType, PxU8 actorFlags, PxClientID owner, PxDominanceGroup dominanceGroup) :
	mSim			(NULL),
	mPackedIDs		((PxU32(owner)<<SC_FILTERING_ID_SHIFT_BIT)|SC_FILTERING_ID_MASK),
	mActorFlags		(actorFlags),
	mActorType		(PxU8(actorType)),
	mDominanceGroup	(dominanceGroup)
{
	PX_ASSERT((actorType & 0xff) == actorType);
	PX_ASSERT(!hasAggregateID());
}

Sc::ActorCore::~ActorCore()
{
}

void Sc::ActorCore::setActorFlags(PxActorFlags af)	
{ 
	const PxActorFlags old = mActorFlags;
	if(af!=old)
	{
		mActorFlags = af;

		if(mSim)
		{
			if((old ^ af) & PxActorFlag::eVISUALIZATION)
				mSim->setActorsInteractionsDirty(InteractionDirtyFlag::eVISUALIZATION, NULL, InteractionFlag::eFILTERABLE);
			mSim->postActorFlagChange(old, af);
		}
	}
}

void Sc::ActorCore::setDominanceGroup(PxDominanceGroup g)
{
	PX_ASSERT(g<32);

	const bool b = mDominanceGroup.isBitSet()!=0;
	mDominanceGroup = PxBitAndByte(PxU8(g) & 31, b);

	if(mSim)
	{
		//force all related interactions to refresh, so they fetch new dominance values.
		mSim->setActorsInteractionsDirty(InteractionDirtyFlag::eDOMINANCE, NULL, InteractionFlag::eRB_ELEMENT);
	}
}

void Sc::ActorCore::setAggregateID(PxU32 id)
{
	if(id==0xffffffff)
	{
		if(hasAggregateID())
		{
			// PT: this was an aggregate ID and we want to disable it.
			mDominanceGroup.clearBit();

			resetID();
		}
		else
		{
			// PT: this was not an aggregate ID. Make sure it wasn't an env ID either.
			PX_ASSERT((mPackedIDs & SC_FILTERING_ID_MASK) == SC_FILTERING_ID_MASK);
		}
	}
	else
	{
		PX_ASSERT(id<SC_FILTERING_ID_MAX);

		// PT: we want to setup an aggregate ID.
		if(hasAggregateID())
		{
			// PT: this was already an aggregate ID and we want to update it.
		}
		else
		{
			// PT: this was not an aggregate ID. Make sure it wasn't an env ID either.
			PX_ASSERT((mPackedIDs & SC_FILTERING_ID_MASK) == SC_FILTERING_ID_MASK);

			mDominanceGroup.setBit();
		}

		setID(id);
	}
}

void Sc::ActorCore::setEnvID(PxU32 id)
{
	if(id==0xffffffff)
	{
		// PT: we want to disable the env ID
		if(hasAggregateID())
		{
			// PT: this is an aggregate ID => env ID is already disabled.
		}
		else
		{
			// PT: this is not an aggregate ID => disable env ID.
			resetID();
		}
	}
	else
	{
		PX_ASSERT(id<SC_FILTERING_ID_MAX);

		// PT: we want to setup an env ID.
		if(hasAggregateID())
		{
			// PT: this is already an aggregate ID, invalid case
			PX_ASSERT(!"Invalid case, aggregated actors cannot have their own env ID. Setup the env ID on the owner aggregate.");
		}
		else
		{
			setID(id);
		}
	}
}

void Sc::ActorCore::reinsertShapes()
{
	PX_ASSERT(mSim);
	if(!mSim)
		return;

	PxU32 nbElems = mSim->getNbElements();
	ElementSim** elems = mSim->getElements();
	while (nbElems--)
	{
		ShapeSim* current = static_cast<ShapeSim*>(*elems++);
		current->reinsertBroadPhase();
	}
}
