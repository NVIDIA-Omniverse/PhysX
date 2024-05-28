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

#include "ScShapeSimBase.h"
#include "ScSqBoundsManager.h"
#include "ScTriggerInteraction.h"
#include "ScSimulationController.h"
#include "CmTransformUtils.h"
#include "ScShapeInteraction.h"

using namespace physx;
using namespace Sc;

// PT: keep local functions in cpp, no need to pollute the header. Don't force conversions to bool if not necessary.
static PX_FORCE_INLINE PxU32 hasTriggerFlags(PxShapeFlags flags) { return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE); }

void resetElementID(Scene& scene, ShapeSimBase& shapeSim)
{
	PX_ASSERT(!shapeSim.isInBroadPhase());

	//	scene.getDirtyShapeSimMap().reset(shapeSim.getElementID());
	scene.getDirtyShapeSimMap().boundedReset(shapeSim.getElementID());

	if (shapeSim.getSqBoundsId() != PX_INVALID_U32)
		shapeSim.destroySqBounds();
}

PX_INLINE Bp::FilterGroup::Enum getBPGroup(const ShapeSimBase& shapeSim)
{

	const BodySim* bs = shapeSim.getBodySim();

	const RigidSim& rbSim = shapeSim.getRbSim();

	bool isKinematic = bs ? bs->isKinematic() : false;

	if (isKinematic && bs->hasForcedKinematicNotif())
		isKinematic = false;

	return Bp::getFilterGroup(rbSim.getActorType() == PxActorType::eRIGID_STATIC, rbSim.getActorID(), isKinematic);
}

static void setElementInteractionsDirty(Sc::ElementSim& elementSim, InteractionDirtyFlag::Enum flag, PxU8 interactionFlag)
{
	ElementSim::ElementInteractionIterator iter = elementSim.getElemInteractions();
	ElementSimInteraction* interaction = iter.getNext();
	while(interaction)
	{
		if(interaction->readInteractionFlag(interactionFlag))
			interaction->setDirty(flag);

		interaction = iter.getNext();
	}
}

void ShapeSimBase::onFilterDataChange()
{
	setElementInteractionsDirty(*this, InteractionDirtyFlag::eFILTER_STATE, InteractionFlag::eFILTERABLE);
}

void ShapeSimBase::onResetFiltering()
{
	if (isInBroadPhase())
		reinsertBroadPhase();
}

void ShapeSimBase::onMaterialChange()
{
	setElementInteractionsDirty(*this, InteractionDirtyFlag::eMATERIAL, InteractionFlag::eRB_ELEMENT);
}

void ShapeSimBase::onRestOffsetChange()
{
	setElementInteractionsDirty(*this, InteractionDirtyFlag::eREST_OFFSET, InteractionFlag::eRB_ELEMENT);
}

void ShapeSimBase::onContactOffsetChange()
{
	if (isInBroadPhase())
		getScene().getAABBManager()->setContactDistance(getElementID(), getCore().getContactOffset());
}

void ShapeSimBase::removeFromBroadPhase(bool wakeOnLostTouch)
{
	if (isInBroadPhase())
		internalRemoveFromBroadPhase(wakeOnLostTouch);
}

void ShapeSimBase::reinsertBroadPhase()
{
	bool wasPendingInsert = false;
	if (isInBroadPhase())
	{
		wasPendingInsert = internalRemoveFromBroadPhase();
	}
	//	internalAddToBroadPhase();

	Scene& scene = getScene();

	// Scene::removeShape
	{
		//unregisterShapeFromNphase(shape.getCore());

		// PT: "getID" is const but the addShape call used LLShape, which uses elementID, so....
		scene.getSimulationController()->removeShape(getElementID());

		scene.unregisterShapeFromNphase(getCore(), getElementID());
	}

	// Call ShapeSim dtor
	{
		resetElementID(scene, *this);
	}

	// Call ElementSim dtor - only required if this shape was not pending insert (otherwise the elementID is fine to keep)
	if (!wasPendingInsert)
	{
		{
			releaseID();
		}

		// Call ElementSim ctor
		{
			initID();
		}
	}

	// Call ShapeSim ctor
	{
		initSubsystemsDependingOnElementID();
	}

	// Scene::addShape
	{
		scene.getSimulationController()->addShape(&getLLShapeSim(), getElementID());

		// PT: TODO: anything else needed here?
		scene.registerShapeInNphase(&getRbSim().getRigidCore(), getCore(), getElementID());
	}
}

PX_FORCE_INLINE void ShapeSimBase::internalAddToBroadPhase()
{
	PX_ASSERT(!isInBroadPhase());

	addToAABBMgr(getCore().getContactOffset(), getBPGroup(*this), (getCore().getCore().mShapeFlags & PxShapeFlag::eTRIGGER_SHAPE) ? Bp::ElementType::eTRIGGER : Bp::ElementType::eSHAPE);
}

PX_FORCE_INLINE bool ShapeSimBase::internalRemoveFromBroadPhase(bool wakeOnLostTouch)
{
	PX_ASSERT(isInBroadPhase());
	bool res = removeFromAABBMgr();

	Scene& scene = getScene();
	PxsContactManagerOutputIterator outputs = scene.getLowLevelContext()->getNphaseImplementationContext()->getContactManagerOutputs();
	scene.getNPhaseCore()->onVolumeRemoved(this, wakeOnLostTouch ? PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH) : 0, outputs);
	return res;
}

void ShapeSimBase::initSubsystemsDependingOnElementID()
{
	Scene& scScene = getScene();

	Bp::BoundsArray& boundsArray = scScene.getBoundsArray();
	const PxU32 index = getElementID();

	PX_ALIGN(16, PxTransform absPos);
	getAbsPoseAligned(&absPos);

	PxsTransformCache& cache = scScene.getLowLevelContext()->getTransformCache();
	cache.initEntry(index);
	cache.setTransformCache(absPos, 0, index);

	boundsArray.updateBounds(absPos, getCore().getGeometryUnion().getGeometry(), index);

	{
		PX_PROFILE_ZONE("API.simAddShapeToBroadPhase", scScene.getContextId());
		if (isBroadPhase(getCore().getFlags()))
			internalAddToBroadPhase();
		else
			scScene.getAABBManager()->reserveSpaceForBounds(index);
		scScene.updateContactDistance(index, getContactOffset());
	}

	//	if(scScene.getDirtyShapeSimMap().size() <= index)
	//		scScene.getDirtyShapeSimMap().resize(PxMax(index+1, (scScene.getDirtyShapeSimMap().size()+1) * 2u));

	RigidSim& owner = getRbSim();
	if (owner.isDynamicRigid() && static_cast<BodySim&>(owner).isActive())
		createSqBounds();

	// Init LL shape
	{
		mLLShape.mElementIndex_GPU = index;
		mLLShape.mShapeCore = const_cast<PxsShapeCore*>(&getCore().getCore());

		if (owner.getActorType() == PxActorType::eRIGID_STATIC)
		{
			mLLShape.mBodySimIndex_GPU = PxNodeIndex(PX_INVALID_NODE);
		}
		else
		{
			BodySim& bodySim = static_cast<BodySim&>(getActor());
			mLLShape.mBodySimIndex_GPU = bodySim.getNodeIndex();
			//mLLShape.mLocalBound = computeBounds(mCore.getGeometry(), PxTransform(PxIdentity));
		}
	}
}

void ShapeSimBase::getAbsPoseAligned(PxTransform* PX_RESTRICT globalPose) const
{
	// PT: TODO: simplify dynamic case when shape2Actor = idt

	const PxsShapeCore& shapeCore = getCore().getCore();

	const PxTransform& shape2Actor = shapeCore.getTransform();
	const PxTransform* actor2World = NULL;
	if (getActor().getActorType() == PxActorType::eRIGID_STATIC)
	{
		PxsRigidCore& core = static_cast<StaticSim&>(getActor()).getStaticCore().getCore();

		if (shapeCore.mShapeCoreFlags.isSet(PxShapeCoreFlag::eIDT_TRANSFORM))
		{
			PX_ASSERT(shape2Actor.p.isZero() && shape2Actor.q.isIdentity());
			*globalPose = core.body2World;
			return;
		}

		actor2World = &core.body2World;
	}
	else
	{
		PxsBodyCore& core = static_cast<BodySim&>(getActor()).getBodyCore().getCore();
		if (!core.hasIdtBody2Actor())
		{
			Cm::getDynamicGlobalPoseAligned(core.body2World, shape2Actor, core.getBody2Actor(), *globalPose);
			return;
		}
		actor2World = &core.body2World;
	}
	Cm::getStaticGlobalPoseAligned(*actor2World, shape2Actor, *globalPose);
}

void ShapeSimBase::onFlagChange(PxShapeFlags oldFlags)
{
	const PxShapeFlags newFlags = getCore().getFlags();

	const bool oldBp = isBroadPhase(oldFlags) != 0;
	const bool newBp = isBroadPhase(newFlags) != 0;

	// Change of collision shape flags requires removal/add to broadphase
	if (oldBp != newBp)
	{
		if (!oldBp && newBp)
		{
			// A.B. if a trigger was removed and inserted within the same frame we need to reinsert
			if (hasTriggerFlags(newFlags) && getScene().getAABBManager()->isMarkedForRemove(getElementID()))
				reinsertBroadPhase();
			else
				internalAddToBroadPhase();
		}
		else
			internalRemoveFromBroadPhase();
	}
	else
	{
		const bool wasTrigger = hasTriggerFlags(oldFlags) != 0;
		const bool isTrigger = hasTriggerFlags(newFlags) != 0;
		if (wasTrigger != isTrigger)
			reinsertBroadPhase();  // re-insertion is necessary because trigger pairs get killed
	}

	const PxShapeFlags hadSq = oldFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;
	const PxShapeFlags hasSq = newFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;
	if (hasSq && !hadSq)
	{
		BodySim* body = getBodySim();
		if (body &&  body->isActive())
			createSqBounds();
	}
	else if (hadSq && !hasSq)
		destroySqBounds();

	getScene().getSimulationController()->reinsertShape(&getLLShapeSim(), getElementID());
}

BodySim* ShapeSimBase::getBodySim() const
{
	ActorSim& a = getActor();
	return a.isDynamicRigid() ? static_cast<BodySim*>(&a) : NULL;
}

PxsRigidCore& ShapeSimBase::getPxsRigidCore() const
{
	ActorSim& a = getActor();
	return a.isDynamicRigid() ? static_cast<BodySim&>(a).getBodyCore().getCore()
		: static_cast<StaticSim&>(a).getStaticCore().getCore();
}

void ShapeSimBase::updateCached(PxU32 transformCacheFlags, PxBitMapPinned* shapeChangedMap)
{
	PX_ALIGN(16, PxTransform absPose);
	getAbsPoseAligned(&absPose);

	Scene& scene = getScene();
	const PxU32 index = getElementID();

	scene.getLowLevelContext()->getTransformCache().setTransformCache(absPose, transformCacheFlags, index);
	scene.getBoundsArray().updateBounds(absPose, getCore().getGeometryUnion().getGeometry(), index);
	if (shapeChangedMap && isInBroadPhase())
		shapeChangedMap->growAndSet(index);
}

void ShapeSimBase::updateCached(PxsTransformCache& transformCache, Bp::BoundsArray& boundsArray)
{
	const PxU32 index = getElementID();

	PxsCachedTransform& ct = transformCache.getTransformCache(index);
	PxPrefetchLine(&ct);

	getAbsPoseAligned(&ct.transform);

	ct.flags = 0;

	PxBounds3& b = boundsArray.begin()[index];
	Gu::computeBounds(b, getCore().getGeometryUnion().getGeometry(), ct.transform, 0.0f, 1.0f);
}

void ShapeSimBase::updateBPGroup()
{
	if (isInBroadPhase())
	{
		Sc::Scene& scene = getScene();
		scene.getAABBManager()->setBPGroup(getElementID(), getBPGroup(*this));

		reinsertBroadPhase();
//		internalRemoveFromBroadPhase();
//		internalAddToBroadPhase();
	}
}

void ShapeSimBase::markBoundsForUpdate()
{
	Scene& scene = getScene();
	if (isInBroadPhase())
		scene.getDirtyShapeSimMap().growAndSet(getElementID());
}

static PX_FORCE_INLINE void updateInteraction(Scene& scene, Interaction* i, const bool isDynamic, const bool isAsleep)
{
	if (i->getType() == InteractionType::eOVERLAP)
	{
		ShapeInteraction* si = static_cast<ShapeInteraction*>(i);
		si->resetManagerCachedState();

		if (isAsleep)
			si->onShapeChangeWhileSleeping(isDynamic);
	}
	else if (i->getType() == InteractionType::eTRIGGER)
		(static_cast<TriggerInteraction*>(i))->forceProcessingThisFrame(scene);  // trigger pairs need to be checked next frame
}

void ShapeSimBase::onVolumeOrTransformChange()
{
	Scene& scene = getScene();
	BodySim* body = getBodySim();
	const bool isDynamic = (body != NULL);
	const bool isAsleep = body ? !body->isActive() : true;

	ElementSim::ElementInteractionIterator iter = getElemInteractions();
	ElementSimInteraction* i = iter.getNext();
	while (i)
	{
		updateInteraction(scene, i, isDynamic, isAsleep);
		i = iter.getNext();
	}

	markBoundsForUpdate();
	getScene().getSimulationController()->reinsertShape(&getLLShapeSim(), getElementID());
}

void notifyActorInteractionsOfTransformChange(ActorSim& actor)
{
	bool isDynamic;
	bool isAsleep;
	if (actor.isDynamicRigid())
	{
		isDynamic = true;
		isAsleep = !static_cast<BodySim&>(actor).isActive();
	}
	else
	{
		isDynamic = false;
		isAsleep = true;
	}

	Scene& scene = actor.getScene();

	PxU32 nbInteractions = actor.getActorInteractionCount();
	Interaction** interactions = actor.getActorInteractions();
	while (nbInteractions--)
		updateInteraction(scene, *interactions++, isDynamic, isAsleep);
}

void ShapeSimBase::createSqBounds()
{
	if (mSqBoundsId != PX_INVALID_U32)
		return;

	BodySim* bodySim = getBodySim();
	PX_ASSERT(bodySim);

	if (bodySim->usingSqKinematicTarget() || bodySim->isFrozen() || !bodySim->isActive() || bodySim->readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
		return;

	if (getCore().getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
		getScene().getSqBoundsManager().addSyncShape(*this);
}

void ShapeSimBase::destroySqBounds()
{
	if (mSqBoundsId != PX_INVALID_U32)
		getScene().getSqBoundsManager().removeSyncShape(*this);
}

