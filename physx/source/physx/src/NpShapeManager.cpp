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

#include "NpShapeManager.h"
#include "NpPtrTableStorageManager.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "ScBodySim.h"
#include "GuBounds.h"
#include "NpAggregate.h"
#include "CmTransformUtils.h"
#include "NpRigidStatic.h"
#include "foundation/PxSIMDHelpers.h"

using namespace physx;
using namespace Sq;
using namespace Gu;
using namespace Cm;

// PT: TODO: refactor if we keep it
static void getSQGlobalPose(PxTransform& globalPose, const NpShape& npShape, const NpActor& npActor)
{
	const PxTransform& shape2Actor = npShape.getCore().getShape2Actor();

	PX_ALIGN(16, PxTransform) kinematicTarget;

	// PT: TODO: duplicated from SqBounds.cpp. Refactor.
	const NpType::Enum actorType = npActor.getNpType();
	const PxTransform* actor2World;
	if(actorType==NpType::eRIGID_STATIC)
	{
		actor2World = &static_cast<const NpRigidStatic&>(npActor).getCore().getActor2World();

		if(npShape.getCore().getCore().mShapeCoreFlags.isSet(PxShapeCoreFlag::eIDT_TRANSFORM))
		{
			PX_ASSERT(shape2Actor.p.isZero() && shape2Actor.q.isIdentity());
			globalPose = *actor2World;
			return;
		}

	}
	else
	{
		PX_ASSERT(actorType==NpType::eBODY || actorType == NpType::eBODY_FROM_ARTICULATION_LINK);

		const PxU16 sqktFlags = PxRigidBodyFlag::eKINEMATIC | PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES;

		// PT: TODO: revisit this once the dust has settled
		const Sc::BodyCore& core = actorType==NpType::eBODY ? static_cast<const NpRigidDynamic&>(npActor).getCore() : static_cast<const NpArticulationLink&>(npActor).getCore();
		const bool useTarget = (PxU16(core.getFlags()) & sqktFlags) == sqktFlags;
		const PxTransform& body2World = (useTarget && core.getKinematicTarget(kinematicTarget)) ? kinematicTarget : core.getBody2World();

		if(!core.getCore().hasIdtBody2Actor())
		{
			Cm::getDynamicGlobalPoseAligned(body2World, shape2Actor, core.getBody2Actor(), globalPose);
			return;
		}

		actor2World = &body2World;
	}

	Cm::getStaticGlobalPoseAligned(*actor2World, shape2Actor, globalPose);
}


static PX_FORCE_INLINE bool isSceneQuery(const NpShape& shape) { return shape.getFlagsFast() & PxShapeFlag::eSCENE_QUERY_SHAPE; }

static PX_FORCE_INLINE bool isDynamicActor(const PxRigidActor& actor)
{
	const PxType actorType = actor.getConcreteType();
	return actorType != PxConcreteType::eRIGID_STATIC;
}

static PX_FORCE_INLINE bool isDynamicActor(const NpActor& actor)
{
	const NpType::Enum actorType = actor.getNpType();
	return actorType != NpType::eRIGID_STATIC;
}

NpShapeManager::NpShapeManager() :
	mPruningStructure	(NULL)
{
	setCompoundID(NP_INVALID_COMPOUND_ID);
}

// PX_SERIALIZATION
NpShapeManager::NpShapeManager(const PxEMPTY) :
	mShapes			(PxEmpty)
{	
}

NpShapeManager::~NpShapeManager() 
{ 
	PX_ASSERT(!mPruningStructure);
	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();
	mShapes.clear(sm);
}

void NpShapeManager::preExportDataReset()
{
}

void NpShapeManager::exportExtraData(PxSerializationContext& stream)
{ 
	mShapes.exportExtraData(stream);
}

void NpShapeManager::importExtraData(PxDeserializationContext& context)
{ 
	mShapes.importExtraData(context);	
}
//~PX_SERIALIZATION

static PX_INLINE void onShapeAttach(NpActor& ro, NpShape& shape)
{
	// * if the shape is exclusive, set its Sc control state appropriately.
	// * add the shape to pvd.
	NpScene* npScene = ro.getNpScene();
	if(!npScene)
		return;

	PX_ASSERT(!npScene->isAPIWriteForbidden());

	if(!(ro.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)))
		ro.getScRigidCore().addShapeToScene(shape.getCore());

#if PX_SUPPORT_PVD
	npScene->getScenePvdClientInternal().createPvdInstance(&shape, *ro.getScRigidCore().getPxActor()); 
#endif
	shape.setSceneIfExclusive(npScene);
	
}

static PX_INLINE void onShapeDetach(NpActor& ro, NpShape& shape, bool wakeOnLostTouch)
{
	// see comments in onShapeAttach
	NpScene* npScene = ro.getNpScene();
	if(!npScene)
		return;
	PX_ASSERT(!npScene->isAPIWriteForbidden());

#if PX_SUPPORT_PVD
	npScene->getScenePvdClientInternal().releasePvdInstance(&shape, *ro.getScRigidCore().getPxActor());
#endif
	if(!(ro.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)))
		ro.getScRigidCore().removeShapeFromScene(shape.getCore(), wakeOnLostTouch);

	shape.setSceneIfExclusive(NULL);

}

void NpShapeManager::attachShape(NpShape& shape, PxRigidActor& actor)
{
	PX_ASSERT(!mPruningStructure);

	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();

	const PxU32 index = getNbShapes();
	mShapes.add(&shape, sm);	

	NpActor& ro = NpActor::getFromPxActor(actor);

	NpScene* scene = NpActor::getNpSceneFromActor(actor);
	if(scene && isSceneQuery(shape))
	{
		// PT: SQ_CODEPATH2
		setupSceneQuery_(scene->getSQAPI(), ro, actor, shape);
	}

	onShapeAttach(ro, shape);

	PxAggregate* agg = ro.getAggregate();
	if(agg)
		static_cast<NpAggregate*>(agg)->incShapeCount();

	PX_ASSERT(!shape.isExclusive() || shape.getActor()==NULL);
	shape.onActorAttach(actor);

	shape.setShapeManagerArrayIndex(index);
}

bool NpShapeManager::detachShape(NpShape& s, PxRigidActor& actor, bool wakeOnLostTouch)
{
	PX_ASSERT(!mPruningStructure);

	const PxU32 index = s.getShapeManagerArrayIndex(mShapes);
	if(index==0xffffffff)
		return false;

	NpScene* scene = NpActor::getNpSceneFromActor(actor);
	if(scene && isSceneQuery(s))
	{
		scene->getSQAPI().removeSQShape(actor, s);

		// if this is the last shape of a compound shape, we have to remove the compound id 
		// and in case of a dynamic actor, remove it from the active list
		if(isSqCompound() && (mShapes.getCount() == 1))
		{
			setCompoundID(NP_INVALID_COMPOUND_ID);
			const PxType actorType = actor.getConcreteType();
			// for PxRigidDynamic and PxArticulationLink we need to remove the compound rigid flag and remove them from active list
			if(actorType == PxConcreteType::eRIGID_DYNAMIC)
				static_cast<NpRigidDynamic&>(actor).getCore().getSim()->disableCompound();
			else if(actorType == PxConcreteType::eARTICULATION_LINK)
				static_cast<NpArticulationLink&>(actor).getCore().getSim()->disableCompound();
		} 
	}

	NpActor& ro = NpActor::getFromPxActor(actor);

	onShapeDetach(ro, s, wakeOnLostTouch);

	PxAggregate* agg = ro.getAggregate();
	if (agg)
		static_cast<NpAggregate*>(agg)->decShapeCount();

	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();

	void** ptrs = mShapes.getPtrs();
	PX_ASSERT(reinterpret_cast<NpShape*>(ptrs[index]) == &s);
	const PxU32 last = mShapes.getCount() - 1;
	if (index != last)
	{
		NpShape* moved = reinterpret_cast<NpShape*>(ptrs[last]);
		PX_ASSERT(moved->checkShapeManagerArrayIndex(mShapes));
		moved->setShapeManagerArrayIndex(index);
	}
	mShapes.replaceWithLast(index, sm);
	s.clearShapeManagerArrayIndex();
	
	s.onActorDetach();
	return true;
}

void NpShapeManager::detachAll(PxSceneQuerySystem* pxsq, const PxRigidActor& actor)
{
	// assumes all SQ data has been released, which is currently the responsibility of the owning actor
	const PxU32 nbShapes = getNbShapes();
	NpShape*const *shapes = getShapes();

	if(pxsq)
		teardownAllSceneQuery(*pxsq, actor); 

	// actor cleanup in Sc will remove any outstanding references corresponding to sim objects, so we don't need to do that here.
	for(PxU32 i=0;i<nbShapes;i++)
		shapes[i]->onActorDetach();

	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();

	mShapes.clear(sm);
}

PxU32 NpShapeManager::getShapes(PxShape** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return getArrayOfPointers(buffer, bufferSize, startIndex, getShapes(), getNbShapes());
}

// PT: this one is only used by the API getWorldBounds() functions
PxBounds3 NpShapeManager::getWorldBounds_(const PxRigidActor& actor) const
{
	PxBounds3 bounds(PxBounds3::empty());

	const PxU32 nbShapes = getNbShapes();
	NpShape*const* PX_RESTRICT shapes = getShapes();

	const PxTransform32 actorPose(actor.getGlobalPose());

	for(PxU32 i=0;i<nbShapes;i++)
	{
		PxTransform32 shapeAbsPose;
		aos::transformMultiply<true, true>(shapeAbsPose, actorPose, shapes[i]->getLocalPoseFast());

		bounds.include(computeBounds(shapes[i]->getCore().getGeometry(), shapeAbsPose));
	}

	return bounds;
}

void NpShapeManager::clearShapesOnRelease(NpScene& s, PxRigidActor& r)
{
	PX_ASSERT(NpActor::getFromPxActor(r).getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION));
	
	const PxU32 nbShapes = getNbShapes();
#if PX_SUPPORT_PVD
	NpShape*const* PX_RESTRICT shapes = getShapes();
#endif
	for(PxU32 i=0;i<nbShapes;i++)
	{
#if PX_SUPPORT_PVD
		s.getScenePvdClientInternal().releasePvdInstance(shapes[i], r);
#else
		PX_UNUSED(s);
		PX_UNUSED(r);
#endif
	}
}

void NpShapeManager::releaseExclusiveUserReferences()
{
	// when the factory is torn down, release any shape owner refs that are still outstanding
	const PxU32 nbShapes = getNbShapes();
	NpShape*const* PX_RESTRICT shapes = getShapes();
	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(shapes[i]->isExclusiveFast() && shapes[i]->getReferenceCount()>1)
			shapes[i]->release();
	}
}

void NpShapeManager::setupSceneQuery(PxSceneQuerySystem& pxsq, const NpActor& npActor, const PxRigidActor& actor, const NpShape& shape)
{ 
	PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);

	setupSceneQuery_(pxsq, npActor, actor, shape);
}

// PT: TODO: function called from a single place?
void NpShapeManager::teardownSceneQuery(PxSceneQuerySystem& pxsq, const PxRigidActor& actor, const NpShape& shape)
{
	pxsq.removeSQShape(actor, shape);
}

void NpShapeManager::setupAllSceneQuery(PxSceneQuerySystem& pxsq, const NpActor& npActor, const PxRigidActor& actor, const PruningStructure* ps, const PxBounds3* bounds, bool isDynamic)
{
	const PxU32 nbShapes = getNbShapes();
	NpShape*const *shapes = getShapes();

	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(isSceneQuery(*shapes[i]))
			setupSQShape(pxsq, *shapes[i], npActor, actor, isDynamic, bounds ? bounds + i : NULL, ps);
	}
}

void NpShapeManager::setupAllSceneQuery(PxSceneQuerySystem& pxsq, const PxRigidActor& actor, const PruningStructure* ps, const PxBounds3* bounds, const BVH* bvh)
{ 
	// if BVH was provided, we add shapes into compound pruner
	if(bvh)
		addBVHShapes(pxsq, actor, *bvh);
	else
		setupAllSceneQuery(pxsq, NpActor::getFromPxActor(actor), actor, ps, bounds, isDynamicActor(actor));
}

void NpShapeManager::teardownAllSceneQuery(PxSceneQuerySystem& pxsq, const PxRigidActor& actor)
{
	NpShape*const *shapes = getShapes();
	const PxU32 nbShapes = getNbShapes();

	if(isSqCompound())
	{
		pxsq.removeSQCompound(getCompoundID());
		setCompoundID(NP_INVALID_COMPOUND_ID);
	}
	else
	{
		for(PxU32 i=0;i<nbShapes;i++)
		{
			if(isSceneQuery(*shapes[i]))
				pxsq.removeSQShape(actor, *shapes[i]);
		}
	}
}

void NpShapeManager::markShapeForSQUpdate(PxSceneQuerySystem& pxsq, const PxShape& shape, const PxRigidActor& actor)
{
	// PT: SQ_CODEPATH4

	PX_ALIGN(16, PxTransform) transform;

	const NpShape& nbShape = static_cast<const NpShape&>(shape);

	const NpActor& npActor = NpActor::getFromPxActor(actor);
	if(getCompoundID() == NP_INVALID_COMPOUND_ID)
		getSQGlobalPose(transform, nbShape, npActor);
	else
		transform = nbShape.getCore().getShape2Actor();		

	pxsq.updateSQShape(actor, shape, transform);
}

void NpShapeManager::markActorForSQUpdate(PxSceneQuerySystem& pxsq, const PxRigidActor& actor)
{
	// PT: SQ_CODEPATH5
	if(isSqCompound())
	{
		pxsq.updateSQCompound(getCompoundID(), actor.getGlobalPose());
	}
	else
	{
		const NpActor& npActor = NpActor::getFromPxActor(actor);
		const PxU32 nbShapes = getNbShapes();
		for(PxU32 i=0;i<nbShapes;i++)
		{
			const NpShape& npShape = *getShapes()[i];

			if(isSceneQuery(npShape))
			{
				PX_ALIGN(16, PxTransform) transform;
				getSQGlobalPose(transform, npShape, npActor);

				pxsq.updateSQShape(actor, npShape, transform);
			}
		}
	}
}


//
// internal methods
// 

#include "NpBounds.h"

void NpShapeManager::addBVHShapes(PxSceneQuerySystem& pxsq, const PxRigidActor& actor, const BVH& bvh)
{
	const PxU32 nbShapes = getNbShapes();

	PX_ALLOCA(scShapes, const PxShape*, nbShapes);
	PxU32 numSqShapes = 0;
	{
		for(PxU32 i=0; i<nbShapes; i++)
		{
			const NpShape& shape = *getShapes()[i];
			if(isSceneQuery(shape))
				scShapes[numSqShapes++] = &shape;
		}
		PX_ASSERT(numSqShapes == bvh.getNbBounds());
	}

	PX_ALLOCA(transforms, PxTransform, numSqShapes);
	for(PxU32 i=0; i<numSqShapes; i++)
	{
		const NpShape* npShape = static_cast<const NpShape*>(scShapes[i]);
		transforms[i] = npShape->getLocalPoseFast();
	}

	const PxSQCompoundHandle cid = pxsq.addSQCompound(actor, scShapes, bvh, transforms);
	setCompoundID(cid);
}

void NpShapeManager::setupSQShape(PxSceneQuerySystem& pxsq, const NpShape& shape, const NpActor& npActor, const PxRigidActor& actor, bool dynamic, const PxBounds3* bounds, const PruningStructure* ps)
{
	PX_ALIGN(16, PxTransform) transform;

	PxBounds3 b;
	if(getCompoundID() == NP_INVALID_COMPOUND_ID)
	{
		if(bounds)
			inflateBounds<true>(b, *bounds, SQ_PRUNER_EPSILON);
		else
			(gComputeBoundsTable[dynamic])(b, shape, npActor);

		// PT: TODO: don't recompute it?
		getSQGlobalPose(transform, shape, npActor);
	}
	else
	{
		const PxTransform& shape2Actor = shape.getCore().getShape2Actor();		
		Gu::computeBounds(b, shape.getCore().getGeometry(), shape2Actor, 0.0f, SQ_PRUNER_INFLATION);

		transform = shape2Actor;
	}

	const NpCompoundId cid = getCompoundID();
	pxsq.addSQShape(actor, shape, b, transform, &cid, ps!=NULL);
}

void NpShapeManager::setupSceneQuery_(PxSceneQuerySystem& pxsq, const NpActor& npActor, const PxRigidActor& actor, const NpShape& shape)
{ 
	const bool isDynamic = isDynamicActor(npActor);
	setupSQShape(pxsq, shape, npActor, actor, isDynamic, NULL, NULL);
}

