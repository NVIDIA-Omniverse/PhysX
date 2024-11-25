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
			mSim->postActorFlagChange(old, af);
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
