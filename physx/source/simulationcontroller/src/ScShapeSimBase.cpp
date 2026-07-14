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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.

#include "ScShapeSimBase.h"
#include "ScSqBoundsManager.h"
#include "ScTriggerInteraction.h"
#include "ScSimulationController.h"
#include "CmTransformUtils.h"
#include "ScShapeInteraction.h"
#include "ScArticulationSim.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "cudamanager/PxCudaContextManager.h"
	#include "cudamanager/PxCudaContext.h"
#endif

using namespace physx;
using namespace Sc;
using namespace Cm;

// PT: keep local functions in cpp, no need to pollute the header. Don't force conversions to bool if not necessary.
static PX_FORCE_INLINE PxU32 hasTriggerFlags(PxShapeFlags flags)	{ return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE);	}
static PX_FORCE_INLINE PxU32 isBroadPhase(PxShapeFlags flags)		{ return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE | PxShapeFlag::eSIMULATION_SHAPE);	}
static PX_FORCE_INLINE PxU32 needsBounds(PxShapeFlags flags)		{ return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE);	}

void resetElementID(Scene& scene, ShapeSimBase& shapeSim)
{
	PX_ASSERT(!shapeSim.isInBroadPhase());

	//	scene.getDirtyShapeSimMap().reset(shapeSim.getElementID());
	scene.getDirtyShapeSimMap().boundedReset(shapeSim.getElementID());

	if (shapeSim.getSqBoundsId() != PX_INVALID_U32)
		shapeSim.destroySqBounds();
}

static PX_INLINE Bp::FilterGroup::Enum getBPGroup(const ShapeSimBase& shapeSim)
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

		scene.getSimulationController()->removePxgShape(getElementID());

		scene.unregisterShapeFromNphase(getCore(), getElementID());
	}
	PxU32 indexFrom = getElementID();

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
#if PX_SUPPORT_GPU_PHYSX
			if (!initID())
			{
				PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
										"Sc::ElementSim::ElementSim failed to allocate pinned memory bounds array");
				scene.getCudaContextManager()->getCudaContext()->setAbortMode(true);
				// code below is safe to execute after failure, as elementID is always obtained and bounds array accesses are guarded.
			}
#else
			initID();
#endif
		}
	}

	// Call ShapeSim ctor
	{
		initSubsystemsDependingOnElementID(indexFrom);
	}

	// Scene::addShape
	{
		scene.getSimulationController()->addPxgShape(this, getPxsShapeCore(), getActorNodeIndex(), getElementID());

		// PT: TODO: anything else needed here?
		scene.registerShapeInNphase(&getRbSim().getRigidCore(), getCore(), getElementID()); //  register in narrowphase  getElementID() - transformcacheID. so I guess we must know at this point the definite index 
	}
}

PX_FORCE_INLINE void ShapeSimBase::internalAddToBroadPhase()
{
	PX_ASSERT(!isInBroadPhase());

	addToAABBMgr(getCore().getContactOffset(), getBPGroup(*this), (getCore().mShapeFlags & PxShapeFlag::eTRIGGER_SHAPE) ? Bp::ElementType::eTRIGGER : Bp::ElementType::eSHAPE);
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

void ShapeSimBase::initSubsystemsDependingOnElementID(PxU32 indexFrom)
{
	Scene& scScene = getScene();

	Bp::BoundsArray& boundsArray = scScene.getBoundsArray();
	const PxU32 index = getElementID();

	PX_ALIGN(16, PxTransform absPos);
	getAbsPoseAligned(&absPos);

	PxsTransformCache& cache = scScene.getLowLevelContext()->getTransformCache();
#if PX_SUPPORT_GPU_PHYSX
	PxCudaContextManager* ctxm = scScene.getCudaContextManager();
	if(ctxm && ctxm->getCudaContext()->isInAbortMode())
	{
		return;
	}
	
	if(!cache.initEntry(index))
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
			"ShapeSimBase::initSubsystemsDependingOnElementID: failed to allocate pinned memory transform cache");
		if(ctxm)
		{
			ctxm->getCudaContext()->setAbortMode(true);
		}
		return;
	}
#else
	cache.initEntry(index);
#endif
	cache.setTransformCache(absPos, 0, index);

	boundsArray.updateBounds(absPos, getCore().getGeometryUnion().getGeometry(), index, indexFrom);

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

	ActorSim& owner = mActor;

	if (owner.isDynamicRigid() && static_cast<BodySim&>(owner).isActive())
		createSqBounds();
}

PxNodeIndex ShapeSimBase::getActorNodeIndex() const
{
	ActorSim& owner = mActor;
	return owner.getActorType() == PxActorType::eRIGID_STATIC ? PxNodeIndex(PX_INVALID_NODE) : static_cast<BodySim&>(owner).getNodeIndex();
}

void ShapeSimBase::getAbsPoseAligned(PxTransform* PX_RESTRICT globalPose) const
{
	// PT: TODO: simplify dynamic case when shape2Actor = idt

	const PxsShapeCore& shapeCore = getCore();

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

	getScene().getSimulationController()->addPxgShape(this, getPxsShapeCore(), getActorNodeIndex(), getElementID());
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

///////////////////////////////////////////////////////////////////////////////

void ShapeSimBase::updateCached(const UpdateCachedParams& params, PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics)
{
	///////////////////////////////////////////////////////////////////////////
	// 1) filter out pure visual shapes

	// PT: we don't need to update the transforms & bounds for pure visual shapes
	// We also don't need to update the frozen flags of visual shapes as this is only used for contact gen, not for active actors.
	const ShapeCore& shapeCore = getCore();
	if(!needsBounds(shapeCore.getFlags()))
		return;

	// PT: for SQ we need the transform to update the SQ pose of each object, but we also need the bounds to update the AABB tree.

	const PxU32 index = getElementID();

	///////////////////////////////////////////////////////////////////////////
	// 2) compute new pose, write it to transform cache
	// PT: these bits won't be optimal from multiple threads:
	// - PxsTransformCache::mHasAnythingChanged will be set repeatedly from N threads
	// - contiguous indices could suffer from false sharing when sizeof(PxsCachedTransform) < sizeof(cache line)
	//
	// Original code:
	// PX_ALIGN(16, PxTransform absPose);
	// sim.getAbsPoseAligned(&absPose);
	// params.mTransformCache.setTransformCache(absPose, params.mTransformFlags, index);

	PxsTransformCache& transformCache = params.mTransformCache;
	PxsCachedTransform& ct = transformCache.getTransformCache(index);
	//PxPrefetchLine(&ct);	// PT: disabled because it's questionable here

	// PT: we write the pose directly to the destination address, avoiding a copy. Potential false sharing here.
	getAbsPoseAligned(&ct.transform);

	ct.flags = params.mTransformFlags;

	// PT: PxsTransformCache::mHasAnythingChanged can be set repeatedly from N threads here. Prefer setting it once from the calling code.
	if(!fromTask)
		transformCache.setChangedState();

	///////////////////////////////////////////////////////////////////////////
	// 3) compute new bounds, write them to bounds array
	// PT: these bits won't be safe from multiple threads:
	// - BoundsArray::mHasAnythingChanged will be set repeatedly from N threads
	// - contiguous indices could suffer from false sharing when sizeof(PxBounds3) < sizeof(cache line)
	// - updateBounds is now virtual, and the Pxg version is unsafe (writes to array from N threads)

	// PT: we write the bounds directly to the destination address, avoiding a copy. Potential false sharing here.
	Bp::BoundsArray& boundsArray = params.mBoundsArray;
	if(fromTask)
	{
		// PT: this one bypasses the virtual call (that's why it is thread-safe) and skips the setChangedState() call
		Gu::computeBounds(boundsArray.begin()[index], shapeCore.getGeometryUnion().getGeometry(), ct.transform, 0.0f, 1.0f);
		//boundsArray.setChangedState();
	}
	else
	{
		boundsArray.updateBounds(ct.transform, shapeCore.getGeometryUnion().getGeometry(), index, index);
	}

	///////////////////////////////////////////////////////////////////////////
	// 4) update bitmap

	PX_ASSERT(isInBroadPhase() == (isBroadPhase(shapeCore.getFlags())!=0) );
	if(shapeChangedMap && isInBroadPhase())
	{
		if(useAtomics)
		{
			PxU32* PX_RESTRICT dirtyMap = shapeChangedMap->getWords();
			PX_ASSERT(index < shapeChangedMap->getWordCount() * 32);
			/*PxI32 data = */PxAtomicOr(reinterpret_cast<volatile PxI32*>(&dirtyMap[index >> 5]), 1 << (index & 31));
		}
		else
		{
			// PT: this won't be safe from multiple threads:
			// - the write to the bitmap is not thread safe and could lead to data corruption from N threads
			shapeChangedMap->growAndSet(index);
		}
	}
}

void BodySim::updateCached(const UpdateCachedParams& params, PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics)
{
	if(!(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN))
	{
		PxU32 nbElems = getNbElements();
		ElementSim** elems = getElements();
		while (nbElems--)
		{
			ShapeSim* current = static_cast<ShapeSim*>(*elems++);
			current->updateCached(params, shapeChangedMap, fromTask, useAtomics);
		}
	}
}

void Sc::ArticulationSim::updateCached(const UpdateCachedParams& params, PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics)
{
	const PxU32 nbBodies = mBodies.size();
	BodySim** bodies = mBodies.begin();
	for(PxU32 i=0; i<nbBodies; i++)
	{
		bodies[i]->updateCached(params, shapeChangedMap, fromTask, useAtomics);
	}
}

///////////////////////////////////////////////////////////////////////////////

void BodySim::updateCached_NotThreadSafe(const UpdateCachedParams& params, PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics)
{
	if(!(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN))
	{
		PxU32 nbElems = getNbElements();
		ElementSim** elems = getElements();
		while (nbElems--)
		{
			ShapeSim* current = static_cast<ShapeSim*>(*elems++);
			current->updateCached_NotThreadSafe(params, shapeChangedMap, fromTask, useAtomics);
		}
	}
}

void BodySim::updateCached_ThreadSafe(const UpdateCachedParams& params)
{
	PX_ASSERT(!(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN));	// PT: should not be called otherwise

	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* current = static_cast<ShapeSim*>(*elems++);
		current->updateCached_ThreadSafe(params);
	}
}

void ArticulationSim::updateCached_NotThreadSafe(const UpdateCachedParams& params, PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics)
{
	for(PxU32 i=0; i<mBodies.size(); i++)
		mBodies[i]->updateCached_NotThreadSafe(params, shapeChangedMap, fromTask, useAtomics);
}

///////////////////////////////////////////////////////////////////////////////

#define CHECK_FROZEN_TRANSFORMS	0

// PT: this version doesn't update the cache & bounds, only the transform flags.
// The poses & bounds don't need updating as they are frozen and haven't changed.
void BodySim::freezeTransforms(PxsTransformCache& transformCache)
{
	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* sim = static_cast<ShapeSim*>(*elems++);

		// 1) filter out pure visual shapes
		// PT: we don't need to update the transforms & bounds for pure visual shapes
		// We also don't need to update the frozen flags of visual shapes as this is only used for contact gen, not for active actors.
		const ShapeCore& shapeCore = sim->getCore();
		if(needsBounds(shapeCore.getFlags()))		
		{
			// 2) make sure new pose is the same as current pose (otherwise we should not be frozen)
			const PxU32 index = sim->getElementID();
			PxsCachedTransform& ct = transformCache.getTransformCache(index);
#if CHECK_FROZEN_TRANSFORMS
			PX_ALIGN(16, PxTransform absPose);
			sim->getAbsPoseAligned(&absPose);

			PX_ASSERT(ct.transform == absPose);
#endif
			// PT: this is the only real change: mark transforms as frozen
			// skip bounds update, as the bounds are derived from the pose, which hasn't changed
			// skip bitmap update, as the data hasn't actually changed
			ct.flags = PxsTransformFlag::eFROZEN;
			transformCache.setChangedState();	// PT: TODO: is this necessary?
		}

		sim->destroySqBounds();
	}
}

// PT: this variant (the original code) still updates the data, as it is now used in a case that explicitly
// wants to rollback poses & bounds to their previous version.
void BodySim::freezeTransforms(const UpdateCachedParams& params, PinnableBitMap* shapeChangedMap)
{
	const UpdateCachedParams frozenParams(params.mTransformCache, params.mBoundsArray, PxsTransformFlag::eFROZEN);
	
	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
		// PT: we still need atomic ORs here, as this function is called from a place that can run in parallel with other bitmap updates.
		sim->updateCached_NotThreadSafe(frozenParams, shapeChangedMap, false, true);
		sim->destroySqBounds();
	}
}

///////////////////////////////////////////////////////////////////////////////

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
	getScene().getSimulationController()->addPxgShape(this, getPxsShapeCore(), getActorNodeIndex(), getElementID());
}

void notifyActorInteractionsOfTransformChange(ActorSim& actor)
{
	const bool isDynamic = actor.isDynamicRigid()!=0;
	const bool isAsleep = isDynamic ? !static_cast<BodySim&>(actor).isActive() : true;

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
