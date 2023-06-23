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
	PxFilterObjectType::eSOFTBODY,			// PxActorType::eSOFTBODY
	PxFilterObjectType::eFEMCLOTH,			// PxActorType::eFEMCLOTH
	PxFilterObjectType::ePARTICLESYSTEM,	// PxActorType::ePBD_PARTICLESYSTEM
	PxFilterObjectType::ePARTICLESYSTEM,	// PxActorType::eFLIP_PARTICLESYSTEM
	PxFilterObjectType::ePARTICLESYSTEM,	// PxActorType::eMPM_PARTICLESYSTEM
	PxFilterObjectType::eHAIRSYSTEM,		// PxActorType::eHAIRSYSTEM
};

static const PxU32 gFilterFlagEx[PxActorType::eACTOR_COUNT] =
{
	PxFilterObjectFlagEx::eRIGID_STATIC,									// PxActorType::eRIGID_STATIC
	PxFilterObjectFlagEx::eRIGID_DYNAMIC,									// PxActorType::eRIGID_DYNAMIC
	PxFilterObjectFlagEx::eRIGID_DYNAMIC,									// PxActorType::eARTICULATION_LINK
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::eSOFTBODY,		// PxActorType::eSOFTBODY
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::eFEMCLOTH,		// PxActorType::eFEMCLOTH
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::ePARTICLESYSTEM,	// PxActorType::ePBD_PARTICLESYSTEM
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::ePARTICLESYSTEM,	// PxActorType::eFLIP_PARTICLESYSTEM
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::ePARTICLESYSTEM,	// PxActorType::eMPM_PARTICLESYSTEM
	PxFilterObjectFlagEx::eNON_RIGID|PxFilterObjectFlagEx::eHAIRSYSTEM,		// PxActorType::eHAIRSYSTEM
};

// PT: if this breaks, you need to update the above table
PX_COMPILE_TIME_ASSERT(PxActorType::eACTOR_COUNT==9);

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
		PX_ASSERT(gFilterType[PxActorType::eSOFTBODY] == PxFilterObjectType::eSOFTBODY);
		PX_ASSERT(gFilterType[PxActorType::eFEMCLOTH] == PxFilterObjectType::eFEMCLOTH);
		PX_ASSERT(gFilterType[PxActorType::ePBD_PARTICLESYSTEM] == PxFilterObjectType::ePARTICLESYSTEM);
		PX_ASSERT(gFilterType[PxActorType::eFLIP_PARTICLESYSTEM] == PxFilterObjectType::ePARTICLESYSTEM);
		PX_ASSERT(gFilterType[PxActorType::eMPM_PARTICLESYSTEM] == PxFilterObjectType::ePARTICLESYSTEM);
		PX_ASSERT(gFilterType[PxActorType::eHAIRSYSTEM] == PxFilterObjectType::eHAIRSYSTEM);

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
	if (i<mInteractions.size())
		mInteractions[i]->setActorId(this, i);
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
