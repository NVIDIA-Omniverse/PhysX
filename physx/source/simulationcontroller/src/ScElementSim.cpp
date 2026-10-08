// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScElementSim.h"
#include "ScElementSimInteraction.h"
#include "ScSimStats.h"

#if PX_SUPPORT_GPU_PHYSX
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#endif

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
	const PxU32 cachedIndex = element.mShapeArrayIndex;
	PX_ASSERT(cachedIndex != 0xffffffff);
	const PxU32 nbShapes = mShapes.getCount();
	PX_ASSERT(nbShapes);

	// ### DEFENSIVE (OMPE-103062): mShapeArrayIndex is cached on the element at attach time and is never
	// validated against this table. Unlike NpShapeManager::detachShape() we cannot just refuse: our only
	// caller is ~ElementSim, which has no return value and no recovery path, and Sc::Scene::removeShape_ is
	// about to hand this element back to mShapeSimPool. Leaving it in mShapes would therefore leave a
	// dangling pointer that every later walk over getElements() dereferences - strictly worse than the
	// out-of-bounds write we are guarding against. So recover the real position instead. find() is a linear
	// scan, but the table holds a handful of entries and this is only paid on the already-broken path.
	PxU32 index = cachedIndex;
	if(cachedIndex >= nbShapes || reinterpret_cast<ElementSim*>(mShapes.getPtrs()[cachedIndex]) != &element)
	{
		index = mShapes.find(&element);

		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Sc::ShapeManager::onElementDetach: element's cached index (%u) is stale (%u elements in the table); %s.",
			cachedIndex, nbShapes, index == 0xffffffff ? "it is not in the table, nothing to remove" : "recovered its position by searching");

		if(index == 0xffffffff)
		{
			// PT: genuinely absent, i.e. it has already been removed. The table is coherent and doing
			// nothing is the correct outcome here.
			element.mShapeArrayIndex = 0xffffffff;
			return;
		}
	}

	void** ptrs = mShapes.getPtrs();

	const PxU32 last = nbShapes - 1;
	if(index != last)
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
#if PX_SUPPORT_GPU_PHYSX
	if(!initID())
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
								"Sc::ElementSim::ElementSim failed to allocate pinned memory bounds array");
		mActor.getScene().getCudaContextManager()->getCudaContext()->setAbortMode(true);
		// executing onElementAttach below is safe, as initID always sets allocated the elementID successfully, 
		// but might fail to expand the bounds array.
	}
#else
	initID();
#endif
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
	const ActorCore& actorCore = mActor.getActorCore();
	const PxU32 aggregateID = actorCore.getAggregateID();
	const PxU32 envID = actorCore.getEnvID();

	Sc::Scene& scene = getScene();
	if(!scene.getAABBManager()->addBounds(mElementID, contactDistance, group, this, aggregateID, type, envID))
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
