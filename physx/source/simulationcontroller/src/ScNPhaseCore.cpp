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

#include "ScNPhaseCore.h"
#include "ScShapeInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScElementInteractionMarker.h"
#include "ScConstraintInteraction.h"
#include "ScSimStats.h"

using namespace physx;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

NPhaseCore::NPhaseCore(Scene& scene, const PxSceneDesc& sceneDesc) :
	mOwnerScene									(scene),
	mContactReportActorPairSet					("contactReportPairSet"),
	mPersistentContactEventPairList				("persistentContactEventPairs"),
	mNextFramePersistentContactEventPairIndex	(0),
	mForceThresholdContactEventPairList			("forceThresholdContactEventPairs"),
	mContactReportBuffer						(sceneDesc.contactReportStreamBufferSize, (sceneDesc.flags & PxSceneFlag::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)),
	mActorPairPool								("actorPairPool"),
	mActorPairReportPool						("actorPairReportPool"),
	mShapeInteractionPool						(PxAllocatorTraits<ShapeInteraction>::Type("shapeInteractionPool"), 4096),
	mTriggerInteractionPool						("triggerInteractionPool"),
	mActorPairContactReportDataPool				("actorPairContactReportPool"),
	mInteractionMarkerPool						("interactionMarkerPool"),
	mConcludeTriggerInteractionProcessingTask	(scene.getContextId(), this, "ScNPhaseCore.concludeTriggerInteractionProcessing")
{
}

NPhaseCore::~NPhaseCore()
{
	// Clear pending actor pairs (waiting on contact report callback)
	clearContactReportActorPairs(false);
}

PxU32 NPhaseCore::getDefaultContactReportStreamBufferSize() const
{
	return mContactReportBuffer.getDefaultBufferSize();
}

ElementSimInteraction* NPhaseCore::findInteraction(const ElementSim* element0, const ElementSim* element1) const
{
	const PxHashMap<ElementSimKey, ElementSimInteraction*>::Entry* pair = mElementSimMap.find(ElementSimKey(element0->getElementID(), element1->getElementID()));
	return pair ? pair->second : NULL;
}

void NPhaseCore::registerInteraction(ElementSimInteraction* interaction)
{
	mElementSimMap.insert(ElementSimKey(interaction->getElement0().getElementID(), interaction->getElement1().getElementID()), interaction);
}

void NPhaseCore::unregisterInteraction(ElementSimInteraction* interaction)
{
	mElementSimMap.erase(ElementSimKey(interaction->getElement0().getElementID(), interaction->getElement1().getElementID()));
}

void NPhaseCore::onOverlapRemoved(ElementSim* volume0, ElementSim* volume1, PxU32 ccdPass, void* elemSim, PxsContactManagerOutputIterator& outputs)
{
	ElementSim* elementHi = volume1;
	ElementSim* elementLo = volume0;
	// No actor internal interactions
	PX_ASSERT(&elementHi->getActor() != &elementLo->getActor());

	// PT: TODO: get rid of 'findInteraction', cf US10491
	ElementSimInteraction* interaction = elemSim ? reinterpret_cast<ElementSimInteraction*>(elemSim) : findInteraction(elementHi, elementLo);
	// MS: The check below is necessary since at the moment LowLevel broadphase still tracks
	//     killed pairs and hence reports lost overlaps
	if(interaction)
	{
		PxU32 flags = PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH);
		PX_ASSERT(interaction->isElementInteraction());
		releaseElementPair(static_cast<ElementSimInteraction*>(interaction), flags, NULL, ccdPass, true, outputs);
	}
}

// MS: TODO: optimize this for the actor release case?
void NPhaseCore::onVolumeRemoved(ElementSim* volume, PxU32 flags, PxsContactManagerOutputIterator& outputs)
{
	const PxU32 ccdPass = 0;

	flags |= PairReleaseFlag::eRUN_LOST_TOUCH_LOGIC;

	// Release interactions
	// IMPORTANT: Iterate from the back of the list to the front as we release interactions which
	//            triggers a replace with last
	ElementSim::ElementInteractionReverseIterator iter = volume->getElemInteractionsReverse();
	ElementSimInteraction* interaction = iter.getNext();
	while(interaction)
	{
		PX_ASSERT(	(interaction->getType() == InteractionType::eMARKER) ||
					(interaction->getType() == InteractionType::eOVERLAP) ||
					(interaction->getType() == InteractionType::eTRIGGER) );

		releaseElementPair(interaction, flags, volume, ccdPass, true, outputs);

		interaction = iter.getNext();
	}
}

ElementSimInteraction* NPhaseCore::createRbElementInteraction(const FilterInfo& finfo, ShapeSimBase& s0, ShapeSimBase& s1, PxsContactManager* contactManager, ShapeInteraction* shapeInteraction,
	ElementInteractionMarker* interactionMarker, bool isTriggerPair)
{
	ElementSimInteraction* pair = NULL;

	if((finfo.filterFlags & PxFilterFlag::eSUPPRESS) == false)
	{
		if(!isTriggerPair)
		{
			PX_ASSERT(contactManager);
			PX_ASSERT(shapeInteraction);
			pair = createShapeInteraction(s0, s1, finfo.pairFlags, contactManager, shapeInteraction);
		}
		else
		{
			pair = createTriggerInteraction(s0, s1, finfo.pairFlags);
		}
	}
	else
		pair = createElementInteractionMarker(s0, s1, interactionMarker);

	if(finfo.hasPairID)
	{
		// Mark the pair as a filter callback pair
		pair->raiseInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);
	}

	return pair;
}

void NPhaseCore::managerNewTouch(ShapeInteraction& interaction)
{
	//(1) if the pair hasn't already been assigned, look it up!

	ActorPair* actorPair = interaction.getActorPair();

	if(!actorPair)
	{
		ShapeSim& s0 = static_cast<ShapeSim&>(interaction.getElement0());
		ShapeSim& s1 = static_cast<ShapeSim&>(interaction.getElement1());
		actorPair = findActorPair(&s0, &s1, interaction.isReportPair());
		actorPair->incRefCount(); //It's being referenced by a new pair...
		interaction.setActorPair(*actorPair);
	}	
}

static bool shouldSwapBodies(const ShapeSimBase& s0, const ShapeSimBase& s1)
{
	/*
	This tries to ensure that if one of the bodies is static or kinematic, it will be body B
	There is a further optimization to force all pairs that share the same bodies to have
	the same body ordering.  This reduces the number of required partitions in the parallel solver.
	Sorting rules are:
	If bodyA is static, swap
	If bodyA is rigidDynamic and bodyB is articulation, swap
	If bodyA is in an earlier BP group than bodyB, swap
	*/
	// PT: some of these swaps are here to fulfill requirements from the solver code, and we
	// will get asserts and failures without them. Some others are only optimizations.

	// PT: generally speaking we want the "static" actor to be second in the pair.
	// "Static" can mean either:
	// - a proper static body
	// - a kinematic dynamic body
	// - an articulation link with a fixed base
	ActorSim& rs0 = s0.getActor();
	const PxActorType::Enum actorType0 = rs0.getActorType();
	if(actorType0 == PxActorType::eRIGID_STATIC)
		return true;

	ActorSim& rs1 = s1.getActor();
	const PxActorType::Enum actorType1 = rs1.getActorType();

	const bool isDyna0 = actorType0 == PxActorType::eRIGID_DYNAMIC;
	const bool isDyna1 = actorType1 == PxActorType::eRIGID_DYNAMIC;

	if(actorType0 == PxActorType::eARTICULATION_LINK)
	{
		if(isDyna1 || actorType1 == PxActorType::eARTICULATION_LINK)
		{
			if(static_cast<BodySim&>(rs0).getLowLevelBody().mCore->fixedBaseLink)
				return true;
		}
	}
	else if(isDyna0)
	{
		// PT: this tries to implement this requirement: "If bodyA is rigidDynamic and bodyB is articulation, swap"
		// But we do NOT do that if bodyB has a fixed base. It is unclear whether this particular swap is really needed.
		if(actorType1 == PxActorType::eARTICULATION_LINK)
		{
			if(!static_cast<BodySim&>(rs1).getLowLevelBody().mCore->fixedBaseLink)
				return true;
		}
	}

	// PT: initial code was:
	// if((actorType0 == PxActorType::eRIGID_DYNAMIC && actorType1 == PxActorType::eRIGID_DYNAMIC) && actorAKinematic)
	// But actorAKinematic true implies isDyna0 true, so this is equivalent to
	// if(isDyna1 && actorAKinematic)
	// And we only need actorAKinematic in this expression so it's faster to move its computation inside the if:
	// if(isDyna1 && isDyna0 && static_cast<BodySim&>(rs0).isKinematic())
	if(isDyna1 && isDyna0 && static_cast<BodySim&>(rs0).isKinematic())
		return true;

	// PT: initial code was:
	// if(actorType0 == actorType1 && rs0.getActorID() < rs1.getActorID() && !actorBKinematic)
	// We refactor the code a bit to avoid computing actorBKinematic. We could also test actorBKinematic
	// first and avoid reading actor IDs. Unclear what's best, arbitrary choice for now.
	if((actorType0 == actorType1) && (rs0.getActorID() < rs1.getActorID()))
	{
		const bool actorBKinematic = isDyna1 && static_cast<BodySim&>(rs1).isKinematic();
		if(!actorBKinematic)
			return true;
	}

#if PX_SUPPORT_GPU_PHYSX
	// PT: using rs0.isParticleSystem() instead of isParticleSystem(actorType0) is faster.
	if(actorType1 != PxActorType::eRIGID_STATIC && rs0.isParticleSystem())
		return true;
#endif

	return false;
}

ShapeInteraction* NPhaseCore::createShapeInteraction(ShapeSimBase& s0, ShapeSimBase& s1, PxPairFlags pairFlags, PxsContactManager* contactManager, ShapeInteraction* shapeInteraction)
{
	ShapeSimBase* _s0 = &s0;
	ShapeSimBase* _s1 = &s1;

	if(shouldSwapBodies(s0, s1))
		PxSwap(_s0, _s1);

	PX_ASSERT(_s0->getActor().getNodeIndex().isValid()); // after the swap the first shape must not be part of a static body
	ShapeInteraction* si = shapeInteraction ? shapeInteraction : mShapeInteractionPool.allocate();
	PX_PLACEMENT_NEW(si, ShapeInteraction)(*_s0, *_s1, pairFlags, contactManager);

	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);

	return si;
}

TriggerInteraction* NPhaseCore::createTriggerInteraction(ShapeSimBase& s0, ShapeSimBase& s1, PxPairFlags triggerFlags)
{
	ShapeSimBase* triggerShape;
	ShapeSimBase* otherShape;

	if(s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
	{
		triggerShape = &s1;
		otherShape = &s0;
	}
	else
	{
		triggerShape = &s0;
		otherShape = &s1;
	}
	TriggerInteraction* pair = mTriggerInteractionPool.construct(*triggerShape, *otherShape);
	pair->setTriggerFlags(triggerFlags);
	return pair;
}

ElementInteractionMarker* NPhaseCore::createElementInteractionMarker(ElementSim& e0, ElementSim& e1, ElementInteractionMarker* interactionMarker)
{
	ElementInteractionMarker* pair = interactionMarker ? interactionMarker : mInteractionMarkerPool.allocate();
	PX_PLACEMENT_NEW(pair, ElementInteractionMarker)(e0, e1, interactionMarker != NULL);
	return pair;
}

ActorPair* NPhaseCore::findActorPair(ShapeSimBase* s0, ShapeSimBase* s1, PxIntBool isReportPair)
{
	PX_ASSERT(!(s0->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
		   && !(s1->getFlags() & PxShapeFlag::eTRIGGER_SHAPE));
	
	ActorSim* aLess = &s0->getActor();
	ActorSim* aMore = &s1->getActor();

	if(aLess->getActorID() > aMore->getActorID())
		PxSwap(aLess, aMore);

	const BodyPairKey key(aLess->getActorID(), aMore->getActorID());

	ActorPair*& actorPair = mActorPairMap[key];
	
	if(actorPair == NULL)
	{
		if(!isReportPair)
			actorPair = mActorPairPool.construct();
		else
			actorPair = mActorPairReportPool.construct(s0->getActor(), s1->getActor());
	}

	if(!isReportPair || actorPair->isReportPair())
		return actorPair;
	else
	{
		PxU32 size = aLess->getActorInteractionCount();
		Interaction** interactions = aLess->getActorInteractions();
		
		ActorPairReport* actorPairReport = mActorPairReportPool.construct(s0->getActor(), s1->getActor());
		actorPairReport->convert(*actorPair);

		while(size--)
		{
			Interaction* interaction = *interactions++;
			if((&interaction->getActorSim0() == aMore) || (&interaction->getActorSim1() == aMore))
			{
				PX_ASSERT(((&interaction->getActorSim0() == aLess) || (&interaction->getActorSim1() == aLess)));

				if(interaction->getType() == InteractionType::eOVERLAP)
				{
					ShapeInteraction* si = static_cast<ShapeInteraction*>(interaction);
					if(si->getActorPair() != NULL)
						si->setActorPair(*actorPairReport);
				}
			}
		}

		PX_ASSERT(!actorPair->isReportPair());
		mActorPairPool.destroy(actorPair);

		actorPair = actorPairReport;
	}
	return actorPair;
}

// PT: this is called from various places. In some of them like Sc::Scene::lostTouchReports() there is an explicit lock (mNPhaseCore->lockReports())
// so there is no need to lock like in createActorPairContactReportData(). In some others however it is quite unclear and perhaps a lock is missing.
static PX_FORCE_INLINE void destroyActorPairReport(	ActorPairReport& aPair, PxPool<ActorPairContactReportData>& actorPairContactReportDataPool,
													PxPool<ActorPairReport>& actorPairReportPool)
{
	PX_ASSERT(aPair.isReportPair());
	
	if(aPair.mReportData)
	{
		actorPairContactReportDataPool.destroy(aPair.mReportData);
		aPair.mReportData = NULL;
	}

	actorPairReportPool.destroy(&aPair);
}

ElementSimInteraction* NPhaseCore::convert(ElementSimInteraction* pair, InteractionType::Enum newType, FilterInfo& filterInfo, bool removeFromDirtyList,
	PxsContactManagerOutputIterator& outputs)
{
	PX_ASSERT(newType != pair->getType());

	ElementSim& elementA = pair->getElement0();
	ElementSim& elementB = pair->getElement1();

	// Wake up the actors of the pair
	if((pair->getActorSim0().getActorType() == PxActorType::eRIGID_DYNAMIC) && !(static_cast<BodySim&>(pair->getActorSim0()).isActive()))
		pair->getActorSim0().internalWakeUp();
	if((pair->getActorSim1().getActorType() == PxActorType::eRIGID_DYNAMIC) && !(static_cast<BodySim&>(pair->getActorSim1()).isActive()))
		pair->getActorSim1().internalWakeUp();

	// Since the FilterPair struct might have been re-used in the newly created interaction, we need to clear
	// the filter pair marker of the old interaction to avoid that the FilterPair gets deleted by the releaseElementPair()
	// call that follows.
	pair->clearInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);

	// PT: we need to unregister the old interaction *before* creating the new one, because Sc::NPhaseCore::registerInteraction will use
	// ElementSim pointers which are the same for both. Since "releaseElementPair" will call the unregister function from
	// the element's dtor, we don't need to do it explicitly here. Just release the object.
	releaseElementPair(pair, PairReleaseFlag::eWAKE_ON_LOST_TOUCH | PairReleaseFlag::eRUN_LOST_TOUCH_LOGIC, NULL, 0, removeFromDirtyList, outputs);

	ElementSimInteraction* result = NULL;
	switch(newType)
	{
		case InteractionType::eINVALID:
			// This means the pair should get killed
			break;
		case InteractionType::eMARKER:
			{
			result = createElementInteractionMarker(elementA, elementB, NULL);
			break;
			}
		case InteractionType::eOVERLAP:
			{
			result = createShapeInteraction(static_cast<ShapeSim&>(elementA), static_cast<ShapeSim&>(elementB), filterInfo.pairFlags, NULL, NULL);
			break;
			}
		case InteractionType::eTRIGGER:
			{
			result = createTriggerInteraction(static_cast<ShapeSim&>(elementA), static_cast<ShapeSim&>(elementB), filterInfo.pairFlags);
			break;
			}
		case InteractionType::eCONSTRAINTSHADER:
		case InteractionType::eARTICULATION:
		case InteractionType::eTRACKED_IN_SCENE_COUNT:
			PX_ASSERT(0);
			break;
	};

	if(filterInfo.hasPairID)
	{
		PX_ASSERT(result);
		// If a filter callback pair is going to get killed, then the FilterPair struct should already have
		// been deleted.

		// Mark the new interaction as a filter callback pair
		result->raiseInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);
	}
	return result;
}

namespace physx
{
namespace Sc
{
static bool findTriggerContacts(TriggerInteraction* tri, bool toBeDeleted, bool volumeRemoved,
								PxTriggerPair& triggerPair, TriggerPairExtraData& triggerPairExtra,
								SimStats::TriggerPairCountsNonVolatile& triggerPairStats,
								const PxsTransformCache& transformCache)
{
	ShapeSimBase& s0 = tri->getTriggerShape();
	ShapeSimBase& s1 = tri->getOtherShape();

	const PxPairFlags pairFlags = tri->getTriggerFlags();
	PxPairFlags pairEvent;

	bool overlap;
	PxU8 testForRemovedShapes = 0;
	if(toBeDeleted)
	{
		// The trigger interaction is to lie down in its tomb, hence we know that the overlap is gone.
		// What remains is to check whether the interaction was deleted because of a shape removal in
		// which case we need to later check for removed shapes.

		overlap = false;

		if(volumeRemoved)
		{
			// Note: only the first removed volume can be detected when the trigger interaction is deleted but at a later point the second volume might get removed too.
			testForRemovedShapes = TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES;
		}
	}
	else
	{
#if PX_ENABLE_SIM_STATS
		PX_ASSERT(s0.getGeometryType() < PxGeometryType::eCONVEXMESH+1);  // The first has to be the trigger shape
		triggerPairStats[s0.getGeometryType()][s1.getGeometryType()]++;
#else
		PX_UNUSED(triggerPairStats);
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		ShapeSimBase* primitive0 = &s0;
		ShapeSimBase* primitive1 = &s1;

		PX_ASSERT(primitive0->getFlags() & PxShapeFlag::eTRIGGER_SHAPE
			   || primitive1->getFlags() & PxShapeFlag::eTRIGGER_SHAPE);

		// Reorder them if needed
		if(primitive0->getGeometryType() > primitive1->getGeometryType())
			PxSwap(primitive0, primitive1);

		const Gu::GeomOverlapFunc overlapFunc =
			Gu::getOverlapFuncTable()[primitive0->getGeometryType()][primitive1->getGeometryType()];

		const PxU32 elementID0 = primitive0->getElementID();
		const PxU32 elementID1 = primitive1->getElementID();

		const PxTransform& globalPose0 = transformCache.getTransformCache(elementID0).transform;

		const PxTransform& globalPose1 = transformCache.getTransformCache(elementID1).transform;

		PX_ASSERT(overlapFunc);
		overlap = overlapFunc(	primitive0->getCore().getGeometry(), globalPose0,
								primitive1->getCore().getGeometry(), globalPose1,
								&tri->getTriggerCache(), UNUSED_OVERLAP_THREAD_CONTEXT);
	}

	const bool hadOverlap = tri->lastFrameHadContacts();
	if(hadOverlap)
	{
		if(!overlap)
			pairEvent = PxPairFlag::eNOTIFY_TOUCH_LOST;
	}
	else
	{
		if(overlap)
			pairEvent = PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}
	tri->updateLastFrameHadContacts(overlap);

	const PxPairFlags triggeredFlags = pairEvent & pairFlags;
	if(triggeredFlags)
	{
		triggerPair.triggerShape = s0.getPxShape();
		triggerPair.otherShape = s1.getPxShape();
		triggerPair.status = PxPairFlag::Enum(PxU32(pairEvent));
		triggerPair.flags = PxTriggerPairFlags(testForRemovedShapes);

		const ActorCore& actorCore0 = s0.getActor().getActorCore();
		const ActorCore& actorCore1 = s1.getActor().getActorCore();

#if PX_SUPPORT_GPU_PHYSX
		if (actorCore0.getActorCoreType() == PxActorType::eDEFORMABLE_VOLUME)
			triggerPair.triggerActor = static_cast<const DeformableVolumeCore&>(actorCore0).getPxActor();
		else if (actorCore0.getActorCoreType() == PxActorType::eDEFORMABLE_SURFACE)
			triggerPair.triggerActor = static_cast<const DeformableSurfaceCore&>(actorCore0).getPxActor();
		else
#endif
			triggerPair.triggerActor = static_cast<const RigidCore&>(actorCore0).getPxActor();

#if PX_SUPPORT_GPU_PHYSX
		if (actorCore1.getActorCoreType() == PxActorType::eDEFORMABLE_VOLUME)
			triggerPair.otherActor = static_cast<const DeformableVolumeCore&>(actorCore1).getPxActor();
		else if (actorCore1.getActorCoreType() == PxActorType::eDEFORMABLE_SURFACE)
			triggerPair.otherActor = static_cast<const DeformableSurfaceCore&>(actorCore1).getPxActor();
		else
#endif
			triggerPair.otherActor = static_cast<const RigidCore&>(actorCore1).getPxActor();
		

		triggerPairExtra = TriggerPairExtraData(s0.getElementID(), s1.getElementID(),
							actorCore0.getOwnerClient(), actorCore1.getOwnerClient());
		return true;
	}
	return false;
}

class TriggerContactTask : public Cm::Task
{
	PX_NOCOPY(TriggerContactTask)
public:
	TriggerContactTask(TriggerInteraction* const* triggerPairs, PxU32 triggerPairCount, PxMutex& lock,
		Scene& scene, PxsTransformCache& transformCache) :
		Cm::Task				(scene.getContextId()),
		mTriggerPairs			(triggerPairs),
		mTriggerPairCount		(triggerPairCount),
		mLock					(lock),
		mScene					(scene),
		mTransformCache			(transformCache)
	{
	}

	virtual void runInternal()
	{
		SimStats::TriggerPairCountsNonVolatile triggerPairStats;
#if PX_ENABLE_SIM_STATS
		PxMemZero(&triggerPairStats, sizeof(SimStats::TriggerPairCountsNonVolatile));
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
		PxTriggerPair triggerPair[sTriggerPairsPerTask];
		TriggerPairExtraData triggerPairExtra[sTriggerPairsPerTask];
		PxU32 triggerReportItemCount = 0;

		for(PxU32 i=0; i < mTriggerPairCount; i++)
		{
			TriggerInteraction* tri = mTriggerPairs[i];

			PX_ASSERT(tri->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
			
			if (findTriggerContacts(tri, false, false, triggerPair[triggerReportItemCount],
				triggerPairExtra[triggerReportItemCount], triggerPairStats, mTransformCache))
			{
				triggerReportItemCount++;
			}
		}

		if(triggerReportItemCount)
		{
			PxTriggerPair* triggerPairBuffer;
			TriggerPairExtraData* triggerPairExtraBuffer;

			{
				PxMutex::ScopedLock lock(mLock);

				mScene.reserveTriggerReportBufferSpace(triggerReportItemCount, triggerPairBuffer, triggerPairExtraBuffer);

				PxMemCopy(triggerPairBuffer, triggerPair, sizeof(PxTriggerPair) * triggerReportItemCount);
				PxMemCopy(triggerPairExtraBuffer, triggerPairExtra, sizeof(TriggerPairExtraData) * triggerReportItemCount);
			}
		}

#if PX_ENABLE_SIM_STATS
		SimStats& simStats = mScene.getStatsInternal();
		for(PxU32 i=0; i < PxGeometryType::eCONVEXMESH+1; i++)
		{
			for(PxU32 j=0; j < PxGeometryType::eGEOMETRY_COUNT; j++)
			{
				if(triggerPairStats[i][j] != 0)
					PxAtomicAdd(&simStats.numTriggerPairs[i][j], triggerPairStats[i][j]);
			}
		}
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	}

	virtual const char* getName() const
	{
		return "ScNPhaseCore.triggerInteractionWork";
	}

public:
	static const PxU32 sTriggerPairsPerTask = 64;

private:
	TriggerInteraction* const*		mTriggerPairs;
	const PxU32						mTriggerPairCount;
	PxMutex&						mLock;
	Scene&							mScene;
	PxsTransformCache&				mTransformCache;
};

}  // namespace Sc
}  // namespace physx

bool TriggerProcessingContext::initialize(TriggerInteraction** interactions, PxU32 pairCount, PxcScratchAllocator& allocator)
{
	PX_ASSERT(!mTmpTriggerProcessingBlock);
	PX_ASSERT(mTmpTriggerPairCount == 0);
	PX_ASSERT(pairCount > 0);

	const PxU32 taskCountWithoutRemainder = pairCount / TriggerContactTask::sTriggerPairsPerTask;
	const PxU32 maxTaskCount = taskCountWithoutRemainder + 1;
	const PxU32 pairPtrSize = pairCount * sizeof(TriggerInteraction*);
	const PxU32 memBlockSize = pairPtrSize + (maxTaskCount * sizeof(TriggerContactTask));
	PxU8* triggerProcessingBlock = reinterpret_cast<PxU8*>(allocator.alloc(memBlockSize, true));
	if (triggerProcessingBlock)
	{
		PxMemCopy(triggerProcessingBlock, interactions, pairPtrSize);  // needs to get copied because other tasks may change the source list
		                                                               // while trigger overlap tests run
		mTmpTriggerProcessingBlock = triggerProcessingBlock;  // note: gets released in deinitialize
		mTmpTriggerPairCount = pairCount;
		return true;
	}
	else
	{
		outputError<PxErrorCode::eOUT_OF_MEMORY>(__LINE__, "Temporary memory for trigger pair processing could not be allocated. Trigger overlap tests will not take place.");
	}

	return false;
}

void TriggerProcessingContext::deinitialize(PxcScratchAllocator& allocator)
{
	PX_ASSERT(mTmpTriggerProcessingBlock);
	PX_ASSERT(mTmpTriggerPairCount > 0);

	allocator.free(mTmpTriggerProcessingBlock);
	mTmpTriggerProcessingBlock = NULL;
	mTmpTriggerPairCount = 0;
}

PxBaseTask* NPhaseCore::prepareForTriggerInteractionProcessing(PxBaseTask* continuation)
{
	// Triggers
	TriggerInteraction** triggerInteractions = reinterpret_cast<TriggerInteraction**>(mOwnerScene.getActiveInteractions(InteractionType::eTRIGGER));
	const PxU32 pairCount = mOwnerScene.getNbActiveInteractions(InteractionType::eTRIGGER);

	if (pairCount > 0)
	{
		if (mTriggerProcessingContext.initialize(triggerInteractions, pairCount, mOwnerScene.getLowLevelContext()->getScratchAllocator()))
		{
			mConcludeTriggerInteractionProcessingTask.setContinuation(continuation);
			return &mConcludeTriggerInteractionProcessingTask;
		}
	}

	return NULL;
}

void NPhaseCore::processTriggerInteractions(PxBaseTask& continuation)
{
	TriggerInteraction* const* triggerInteractions = mTriggerProcessingContext.getTriggerInteractions();
	const PxU32 pairCount = mTriggerProcessingContext.getTriggerInteractionCount();
	TriggerContactTask* triggerContactTaskBuffer = mTriggerProcessingContext.getTriggerContactTasks();
	PxMutex& triggerWriteBackLock = mTriggerProcessingContext.getTriggerWriteBackLock();

	PX_ASSERT(triggerInteractions);
	PX_ASSERT(pairCount > 0);
	PX_ASSERT(triggerContactTaskBuffer);

	// PT: TASK-CREATION TAG
	const bool hasMultipleThreads = mOwnerScene.getTaskManager().getCpuDispatcher()->getWorkerCount() > 1;
	const bool moreThanOneBatch = pairCount > TriggerContactTask::sTriggerPairsPerTask;
	const bool scheduleTasks = hasMultipleThreads && moreThanOneBatch;
	// when running on a single thread, the task system seems to cause the main overhead (locking and atomic operations
	// seemed less of an issue). Hence, the tasks get run directly in that case. Same if there is only one batch.

	PxsTransformCache& transformCache = mOwnerScene.getLowLevelContext()->getTransformCache();

	PxU32 remainder = pairCount;
	while(remainder)
	{
		const PxU32 nb = remainder > TriggerContactTask::sTriggerPairsPerTask ? TriggerContactTask::sTriggerPairsPerTask : remainder;
		remainder -= nb;

		TriggerContactTask* task = triggerContactTaskBuffer;
		task = PX_PLACEMENT_NEW(task, TriggerContactTask(	triggerInteractions, nb, triggerWriteBackLock,
															mOwnerScene, transformCache));
		if(scheduleTasks)
		{
			task->setContinuation(&continuation);
			task->removeReference();
		}
		else
			task->runInternal();

		triggerContactTaskBuffer++;
		triggerInteractions += nb;
	}
}

void NPhaseCore::concludeTriggerInteractionProcessing(PxBaseTask*)
{
	// check if active trigger pairs can be deactivated (until woken up again)

	TriggerInteraction* const* triggerInteractions = mTriggerProcessingContext.getTriggerInteractions();
	const PxU32 pairCount = mTriggerProcessingContext.getTriggerInteractionCount();

	PX_ASSERT(triggerInteractions);
	PX_ASSERT(pairCount > 0);

	for (PxU32 i = 0; i < pairCount; i++)
	{
		TriggerInteraction* tri = triggerInteractions[i];

		PX_ASSERT(tri->readInteractionFlag(InteractionFlag::eIS_ACTIVE));

		if (!(tri->readFlag(TriggerInteraction::PROCESS_THIS_FRAME)))
		{
			// active trigger pairs for which overlap tests were not forced should remain in the active list
			// to catch transitions between overlap and no overlap
			continue;
		}
		else
		{
			tri->clearFlag(TriggerInteraction::PROCESS_THIS_FRAME);

			// explicitly scheduled overlap test is done (after object creation, teleport, ...). Check if trigger pair should remain active or not.

			if (!tri->onActivate())
			{
				PX_ASSERT(tri->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
				// Why is the assert enough?
				// Once an explicit overlap test is scheduled, the interaction can not get deactivated anymore until it got processed.

				tri->clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
				mOwnerScene.notifyInteractionDeactivated(tri);
			}
		}
	}

	mTriggerProcessingContext.deinitialize(mOwnerScene.getLowLevelContext()->getScratchAllocator());
}

void NPhaseCore::processPersistentContactEvents(PxsContactManagerOutputIterator& outputs)
{
	PX_PROFILE_ZONE("Sc::NPhaseCore::processPersistentContactEvents", mOwnerScene.getContextId());
	
	// Go through ShapeInteractions which requested persistent contact event reports. This is necessary since there are no low level events for persistent contact.
	ShapeInteraction*const* persistentEventPairs = getCurrentPersistentContactEventPairs();
	PxU32 size = getCurrentPersistentContactEventPairCount();
	while (size--)
	{
		ShapeInteraction* pair = *persistentEventPairs++;
		if (size)
		{
			ShapeInteraction* nextPair = *persistentEventPairs;
			PxPrefetchLine(nextPair);
		}

		ActorPair* aPair = pair->getActorPair();
		PxPrefetchLine(aPair);

		PX_ASSERT(pair->hasTouch());
		PX_ASSERT(pair->isReportPair());

		const PxU32 pairFlags = pair->getPairFlags();
		if ((pairFlags & PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_DISCRETE_CONTACT)) == PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_DISCRETE_CONTACT))
		{
			// do not process the pair if only eDETECT_CCD_CONTACT is enabled because at this point CCD did not run yet. Plus the current CCD implementation can not reliably provide eNOTIFY_TOUCH_PERSISTS events
			// for performance reasons.
			//KS - filter based on edge activity!

			const ActorSim& actorSim0= pair->getShape0().getActor();
			const ActorSim& actorSim1 = pair->getShape1().getActor();

			if (actorSim0.isActive() || ((!actorSim1.isStaticRigid()) && actorSim1.isActive()))
				pair->processUserNotification(PxPairFlag::eNOTIFY_TOUCH_PERSISTS, 0, false, 0, false, outputs);
		}
	}
}

void NPhaseCore::addToDirtyInteractionList(Interaction* pair)
{
	mDirtyInteractions.insert(pair);
}

void NPhaseCore::removeFromDirtyInteractionList(Interaction* pair)
{
	PX_ASSERT(mDirtyInteractions.contains(pair));

	mDirtyInteractions.erase(pair);
}

void NPhaseCore::updateDirtyInteractions(PxsContactManagerOutputIterator& outputs)
{
	// The sleeping SIs will be updated on activation
	// clow: Sleeping SIs are not awaken for visualization updates
	const bool dirtyDominance = mOwnerScene.readInternalFlag(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE);
	const bool dirtyVisualization = mOwnerScene.readInternalFlag(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION);
	if(dirtyDominance || dirtyVisualization)
	{
		// Update all interactions.

		const PxU8 mask = PxTo8((dirtyDominance ? InteractionDirtyFlag::eDOMINANCE : 0) | (dirtyVisualization ? InteractionDirtyFlag::eVISUALIZATION : 0));

		ElementSimInteraction** it = mOwnerScene.getInteractions(InteractionType::eOVERLAP);
		PxU32 size = mOwnerScene.getNbInteractions(InteractionType::eOVERLAP);
		while(size--)
		{
			ElementSimInteraction* pair = *it++;

			PX_ASSERT(pair->getType() == InteractionType::eOVERLAP);

			if(!pair->readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST))
			{
				PX_ASSERT(!pair->getDirtyFlags());
				static_cast<ShapeInteraction*>(pair)->updateState(mask);
			}
			else
				pair->setDirty(mask);  // the pair will get processed further below anyway, so just mark the flags dirty
		}
	}

	// Update all interactions in the dirty list
	const PxU32 dirtyItcCount = mDirtyInteractions.size();
	Interaction* const* dirtyInteractions = mDirtyInteractions.getEntries();
	for(PxU32 i = 0; i < dirtyItcCount; i++)
	{
		Interaction* refInt = dirtyInteractions[i];
		Interaction* interaction = refInt;

		if(interaction->isElementInteraction() && interaction->needsRefiltering())
		{
			ElementSimInteraction* pair = static_cast<ElementSimInteraction*>(interaction);

			refInt = refilterInteraction(pair, NULL, false, outputs);
		}

		if(interaction == refInt)  // Refiltering might convert the pair to another type and kill the old one. In that case we don't want to update the new pair since it has been updated on creation.
		{
			const InteractionType::Enum iType = interaction->getType();
			if (iType == InteractionType::eOVERLAP)
				static_cast<ShapeInteraction*>(interaction)->updateState(0);
			else if (iType == InteractionType::eCONSTRAINTSHADER)
				static_cast<ConstraintInteraction*>(interaction)->updateState();

			interaction->setClean(false);  // false because the dirty interactions list gets cleard further below
		}
	}

	mDirtyInteractions.clear();
}

void NPhaseCore::releaseElementPair(ElementSimInteraction* pair, PxU32 flags, ElementSim* removedElement, PxU32 ccdPass, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs)
{
	pair->setClean(removeFromDirtyList);  // Removes the pair from the dirty interaction list etc.

	if(pair->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR))
	{
		// Check if this is a filter callback pair
		ShapeSimBase& s0 = static_cast<ShapeSimBase&>(pair->getElement0());
		ShapeSimBase& s1 = static_cast<ShapeSimBase&>(pair->getElement1());

		callPairLost(s0, s1, removedElement != NULL);
	}

	switch(pair->getType())
	{
		case InteractionType::eTRIGGER:
			{
				PxsTransformCache& transformCache = mOwnerScene.getLowLevelContext()->getTransformCache();
				TriggerInteraction* tri = static_cast<TriggerInteraction*>(pair);
				PxTriggerPair triggerPair;
				TriggerPairExtraData triggerPairExtra;
				if (findTriggerContacts(tri, true, (removedElement != NULL),
										triggerPair, triggerPairExtra, 
										const_cast<SimStats::TriggerPairCountsNonVolatile&>(mOwnerScene.getStatsInternal().numTriggerPairs), 
										transformCache))
										// cast away volatile-ness (this is fine since the method does not run in parallel)
				{
					mOwnerScene.getTriggerBufferAPI().pushBack(triggerPair);
					mOwnerScene.getTriggerBufferExtraData().pushBack(triggerPairExtra);
				}
				mTriggerInteractionPool.destroy(tri);
			}
			break;
		case InteractionType::eMARKER:
			{
				ElementInteractionMarker* interactionMarker = static_cast<ElementInteractionMarker*>(pair);
				mInteractionMarkerPool.destroy(interactionMarker);
			}
			break;
		case InteractionType::eOVERLAP:
			{
				ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);
				if(flags & PairReleaseFlag::eRUN_LOST_TOUCH_LOGIC)
					lostTouchReports(si, flags, removedElement, ccdPass, outputs);

				mShapeInteractionPool.destroy(si);
			}
			break;
		case InteractionType::eCONSTRAINTSHADER:
		case InteractionType::eARTICULATION:
		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
			PX_ASSERT(0);
			return;
	}
}

void NPhaseCore::lostTouchReports(ShapeInteraction* si, PxU32 flags, ElementSim* removedElement, PxU32 ccdPass, PxsContactManagerOutputIterator& outputs)
{
	ActorPair* aPair = si->getActorPair();

	if(si->hasTouch())
	{
		if(si->isReportPair())
			si->sendLostTouchReport((removedElement != NULL), ccdPass, outputs);

		if(aPair)
			si->adjustCountersOnLostTouch();
	}

	if(aPair && aPair->decRefCount() == 0)
	{
		RigidSim* sim0 = static_cast<RigidSim*>(&si->getActorSim0());
		RigidSim* sim1 = static_cast<RigidSim*>(&si->getActorSim1());

		if(sim0->getActorID() > sim1->getActorID())
			PxSwap(sim0, sim1);

		const BodyPairKey pair(sim0->getActorID(), sim1->getActorID());

		mActorPairMap.erase(pair);

		if(!aPair->isReportPair())
		{
			mActorPairPool.destroy(aPair);
		}
		else
		{
			ActorPairReport& apr = ActorPairReport::cast(*aPair);
			destroyActorPairReport(apr, mActorPairContactReportDataPool, mActorPairReportPool);
		}
	}
	si->clearActorPair();

	if(si->hasTouch() || (!si->hasKnownTouchState()))
	{
		ActorSim& b0 = si->getShape0().getActor();
		ActorSim& b1 = si->getShape1().getActor();

		if(flags & PairReleaseFlag::eWAKE_ON_LOST_TOUCH)
		{
			// we rely on shape pair ordering here, where the first body is never static
			// (see createShapeInteraction())
			PX_ASSERT(!b0.isStaticRigid());

			if (removedElement == NULL)
			{
				if (b1.isStaticRigid())  // no check for b0 being static, see assert further above
				{
					// given wake-on-lost-touch has been requested:
					// if one is static, we wake up the other immediately

					b0.internalWakeUp();
				}
				else if(!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				{
					mOwnerScene.addToLostTouchList(b0, b1);
				}
			}
			else
			{
				// given wake-on-lost-touch has been requested:
				// if an element (broadphase volume) has been removed, we wake the other actor up

				PX_ASSERT((removedElement == &si->getShape0()) || (removedElement == &si->getShape1()));

				if (&si->getShape0() == removedElement)
				{
					if (!b1.isStaticRigid())
						b1.internalWakeUp();
				}
				else
					b0.internalWakeUp();   // no check for b0 being non-static, see assert further above
			}
		}
	}
}

void NPhaseCore::clearContactReportActorPairs(bool shrinkToZero)
{
	for(PxU32 i=0; i < mContactReportActorPairSet.size(); i++)
	{
		//TODO: prefetch?
		ActorPairReport* aPair = mContactReportActorPairSet[i];
		const PxU32 refCount = aPair->getRefCount();
		PX_ASSERT(aPair->isInContactReportActorPairSet());
		PX_ASSERT(refCount > 0);
		aPair->decRefCount();  // Reference held by contact callback
		if(refCount > 1)
		{
			aPair->clearInContactReportActorPairSet();
		}
		else
		{
			const PxU32 actorAID = aPair->getActorAID();
			const PxU32 actorBID = aPair->getActorBID();
			const BodyPairKey pair(PxMin(actorAID, actorBID), PxMax(actorAID, actorBID));

			mActorPairMap.erase(pair);
			destroyActorPairReport(*aPair, mActorPairContactReportDataPool, mActorPairReportPool);
		}
	}

	if(!shrinkToZero)
		mContactReportActorPairSet.clear();
	else
		mContactReportActorPairSet.reset();
}

void NPhaseCore::addToPersistentContactEventPairs(ShapeInteraction* si)
{
	// Pairs which request events which do not get triggered by the sdk and thus need to be tested actively every frame.
	PX_ASSERT(si->getPairFlags() & (PxPairFlag::eNOTIFY_TOUCH_PERSISTS | ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS));
	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch()); // only pairs which can from now on lose or keep contact should be in this list

	si->raiseFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST);
	if(mPersistentContactEventPairList.size() == mNextFramePersistentContactEventPairIndex)
	{
		si->mReportPairIndex = mPersistentContactEventPairList.size();
		mPersistentContactEventPairList.pushBack(si);
	}
	else
	{
		//swap with first entry that will be active next frame
		ShapeInteraction* firstDelayedSi = mPersistentContactEventPairList[mNextFramePersistentContactEventPairIndex];
		firstDelayedSi->mReportPairIndex = mPersistentContactEventPairList.size();
		mPersistentContactEventPairList.pushBack(firstDelayedSi);
		si->mReportPairIndex = mNextFramePersistentContactEventPairIndex;
		mPersistentContactEventPairList[mNextFramePersistentContactEventPairIndex] = si;
	}

	mNextFramePersistentContactEventPairIndex++;
}

void NPhaseCore::addToPersistentContactEventPairsDelayed(ShapeInteraction* si)
{
	// Pairs which request events which do not get triggered by the sdk and thus need to be tested actively every frame.
	PX_ASSERT(si->getPairFlags() & (PxPairFlag::eNOTIFY_TOUCH_PERSISTS | ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS));
	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch()); // only pairs which can from now on lose or keep contact should be in this list

	si->raiseFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST);
	si->mReportPairIndex = mPersistentContactEventPairList.size();
	mPersistentContactEventPairList.pushBack(si);
}

void NPhaseCore::removeFromPersistentContactEventPairs(ShapeInteraction* si)
{
	PX_ASSERT(si->getPairFlags() & (PxPairFlag::eNOTIFY_TOUCH_PERSISTS | ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS));
	PX_ASSERT(si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch()); // only pairs which could lose or keep contact should be in this list

	PxU32 index = si->mReportPairIndex;
	PX_ASSERT(index != INVALID_REPORT_PAIR_ID);

	if(index < mNextFramePersistentContactEventPairIndex)
	{
		const PxU32 replaceIdx = mNextFramePersistentContactEventPairIndex - 1;

		if((mNextFramePersistentContactEventPairIndex < mPersistentContactEventPairList.size()) && (index != replaceIdx))
		{
			// keep next frame persistent pairs at the back of the list
			ShapeInteraction* tmp = mPersistentContactEventPairList[replaceIdx];
			mPersistentContactEventPairList[index] = tmp;
			tmp->mReportPairIndex = index;
			index = replaceIdx;
		}

		mNextFramePersistentContactEventPairIndex--;
	}

	si->clearFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST);
	si->mReportPairIndex = INVALID_REPORT_PAIR_ID;
	mPersistentContactEventPairList.replaceWithLast(index);
	if(index < mPersistentContactEventPairList.size()) // Only adjust the index if the removed SIP was not at the end of the list
		mPersistentContactEventPairList[index]->mReportPairIndex = index;
}

void NPhaseCore::addToForceThresholdContactEventPairs(ShapeInteraction* si)
{
	PX_ASSERT(si->getPairFlags() & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS);
	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch());

	si->raiseFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST);
	si->mReportPairIndex = mForceThresholdContactEventPairList.size();
	mForceThresholdContactEventPairList.pushBack(si);
}

void NPhaseCore::removeFromForceThresholdContactEventPairs(ShapeInteraction* si)
{
	PX_ASSERT(si->getPairFlags() & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS);
	PX_ASSERT(si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(si->hasTouch());

	const PxU32 index = si->mReportPairIndex;
	PX_ASSERT(index != INVALID_REPORT_PAIR_ID);

	si->clearFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST);
	si->mReportPairIndex = INVALID_REPORT_PAIR_ID;
	mForceThresholdContactEventPairList.replaceWithLast(index);
	if(index < mForceThresholdContactEventPairList.size()) // Only adjust the index if the removed SIP was not at the end of the list
		mForceThresholdContactEventPairList[index]->mReportPairIndex = index;
}

PxU8* NPhaseCore::reserveContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, PxU32& bufferIndex, ContactReportAllocationManager* alloc)
{
	extraDataSize = ContactStreamManager::computeExtraDataBlockSize(extraDataSize);
	return alloc ? alloc->allocate(extraDataSize + (pairCount * sizeof(ContactShapePair)), bufferIndex) : mContactReportBuffer.allocateNotThreadSafe(extraDataSize + (pairCount * sizeof(ContactShapePair)), bufferIndex);
}

PxU8* NPhaseCore::resizeContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, ContactStreamManager& csm)
{
	PX_ASSERT((pairCount > csm.maxPairCount) || (extraDataSize > csm.getMaxExtraDataSize()));
	PX_ASSERT((csm.currentPairCount == csm.maxPairCount) || (extraDataSize > csm.getMaxExtraDataSize()));
	PX_ASSERT(extraDataSize >= csm.getMaxExtraDataSize()); // we do not support stealing memory from the extra data part when the memory for pair info runs out

	PxU32 bufferIndex;
	PxPrefetch(mContactReportBuffer.getData(csm.bufferIndex));

	extraDataSize = ContactStreamManager::computeExtraDataBlockSize(extraDataSize);
	PxU8* stream = mContactReportBuffer.reallocateNotThreadSafe(extraDataSize + (pairCount * sizeof(ContactShapePair)), bufferIndex, 16, csm.bufferIndex);
	PxU8* oldStream = mContactReportBuffer.getData(csm.bufferIndex);
	if(stream)
	{
		const PxU32 maxExtraDataSize = csm.getMaxExtraDataSize();
		if(csm.bufferIndex != bufferIndex)
		{
			if(extraDataSize <= maxExtraDataSize)
				PxMemCopy(stream, oldStream, maxExtraDataSize + (csm.currentPairCount * sizeof(ContactShapePair)));
			else
			{
				PxMemCopy(stream, oldStream, csm.extraDataSize);
				PxMemCopy(stream + extraDataSize, oldStream + maxExtraDataSize, csm.currentPairCount * sizeof(ContactShapePair));
			}
			csm.bufferIndex = bufferIndex;
		}
		else if(extraDataSize > maxExtraDataSize)
			PxMemMove(stream + extraDataSize, oldStream + maxExtraDataSize, csm.currentPairCount * sizeof(ContactShapePair));

		if(pairCount > csm.maxPairCount)
			csm.maxPairCount = pairCount;
		if(extraDataSize > maxExtraDataSize)
			csm.setMaxExtraDataSize(extraDataSize);
	}
	return stream;
}

ActorPairContactReportData* NPhaseCore::createActorPairContactReportData()
{
	PxMutex::ScopedLock lock(mReportAllocLock);
	return mActorPairContactReportDataPool.construct();
}


