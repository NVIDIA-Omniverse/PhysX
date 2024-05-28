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

#include "ScElementSim.h"
#include "ScElementSimInteraction.h"
#include "ScSimStats.h"

using namespace physx;
using namespace Sc;

static PX_FORCE_INLINE bool interactionHasElement(const Interaction* it, const ElementSim* elem)
{
	if(it->readInteractionFlag(InteractionFlag::eRB_ELEMENT))
	{
		PX_ASSERT(	(it->getType() == InteractionType::eMARKER) ||
					(it->getType() == InteractionType::eOVERLAP) ||
					(it->getType() == InteractionType::eTRIGGER) );

		const ElementSimInteraction* ei = static_cast<const ElementSimInteraction*>(it);
		if((&ei->getElement0() == elem) || (&ei->getElement1() == elem))
			return true;
	}
	return false;
}

Sc::ElementSimInteraction* Sc::ElementSim::ElementInteractionIterator::getNext()
{
	while(mInteractions!=mInteractionsLast)
	{
		Interaction* it = *mInteractions++;
		if(interactionHasElement(it, mElement))
			return static_cast<ElementSimInteraction*>(it);
	}
	return NULL;
}

Sc::ElementSimInteraction* Sc::ElementSim::ElementInteractionReverseIterator::getNext()
{
	while(mInteractions!=mInteractionsLast)
	{
		Interaction* it = *--mInteractionsLast;
		if(interactionHasElement(it, mElement))
			return static_cast<ElementSimInteraction*>(it);
	}
	return NULL;
}

namespace
{
	class ElemSimPtrTableStorageManager : public Cm::PtrTableStorageManager, public PxUserAllocated
	{
		PX_NOCOPY(ElemSimPtrTableStorageManager)

	public:
		ElemSimPtrTableStorageManager() {}
		~ElemSimPtrTableStorageManager() {}

		// PtrTableStorageManager
		virtual	void**	allocate(PxU32 capacity)	PX_OVERRIDE
		{
			return PX_ALLOCATE(void*, capacity, "CmPtrTable pointer array");
		}

		virtual	void	deallocate(void** addr, PxU32 /*capacity*/)	PX_OVERRIDE
		{
			PX_FREE(addr);
		}

		virtual	bool canReuse(PxU32 /*originalCapacity*/, PxU32 /*newCapacity*/)	PX_OVERRIDE
		{
			return false;
		}
		//~PtrTableStorageManager
	};
	ElemSimPtrTableStorageManager gElemSimTableStorageManager;
}

static PX_FORCE_INLINE void onElementAttach(ElementSim& element, ShapeManager& manager)
{
	PX_ASSERT(element.mShapeArrayIndex == 0xffffffff);
	element.mShapeArrayIndex = manager.mShapes.getCount();
	manager.mShapes.add(&element, gElemSimTableStorageManager);
}

void Sc::ShapeManager::onElementDetach(ElementSim& element)
{
	const PxU32 index = element.mShapeArrayIndex;
	PX_ASSERT(index != 0xffffffff);
	PX_ASSERT(mShapes.getCount());
	void** ptrs = mShapes.getPtrs();
	PX_ASSERT(reinterpret_cast<ElementSim*>(ptrs[index]) == &element);

	const PxU32 last = mShapes.getCount() - 1;
	if (index != last)
	{
		ElementSim* moved = reinterpret_cast<ElementSim*>(ptrs[last]);
		PX_ASSERT(moved->mShapeArrayIndex == last);
		moved->mShapeArrayIndex = index;
	}
	mShapes.replaceWithLast(index, gElemSimTableStorageManager);
	element.mShapeArrayIndex = 0xffffffff;
}

Sc::ElementSim::ElementSim(ActorSim& actor) :
	mActor			(actor),
	mInBroadPhase	(false),
	mShapeArrayIndex(0xffffffff)
{
	initID();

	onElementAttach(*this, actor);
}

Sc::ElementSim::~ElementSim()
{
	PX_ASSERT(!mInBroadPhase);
	releaseID();
	mActor.onElementDetach(*this);
}

void Sc::ElementSim::addToAABBMgr(PxReal contactDistance, Bp::FilterGroup::Enum group, Bp::ElementType::Enum type)
{
	Sc::Scene& scene = getScene();
	if(!scene.getAABBManager()->addBounds(mElementID, contactDistance, group, this, mActor.getActorCore().getAggregateID(), type))
		return;

	mInBroadPhase = true;
#if PX_ENABLE_SIM_STATS
	scene.getStatsInternal().incBroadphaseAdds();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
}

bool Sc::ElementSim::removeFromAABBMgr()
{
	PX_ASSERT(mInBroadPhase);
	Sc::Scene& scene = getScene();
	bool res = scene.getAABBManager()->removeBounds(mElementID);
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndReset(mElementID);

	mInBroadPhase = false;
#if PX_ENABLE_SIM_STATS
	scene.getStatsInternal().incBroadphaseRemoves();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	return res;
}
