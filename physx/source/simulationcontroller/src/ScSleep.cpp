// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScScene.h"
#include "ScArticulationSim.h"
#include "ScBodySim.h"
#include "ScArticulationCore.h"
#include "ScShapeInteraction.h"
#include "DyIslandManager.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "ScDeformableSurfaceSim.h"
	#include "ScDeformableVolumeSim.h"
	#include "ScParticleSystemSim.h"
#endif

#include "common/PxProfileZone.h"

using namespace physx;
using namespace Cm;
using namespace Dy;
using namespace Sc;

/*static*/ bool deactivateInteraction(Interaction* interaction, const InteractionType::Enum type);

// PT: "setActive()" moved from ActorSim to BodySim because GPU classes silently re-implement this in a very different way (see below),
// i.e. it defeats the purpose of the virtual activate()/deactivate() functions.
void Sc::BodySim::setActive(bool active, bool asPartOfCreation)
{
	PX_ASSERT(!active || isDynamicRigid());  // Currently there should be no need to activate an actor that does not take part in island generation

	if(asPartOfCreation || isActive() != active)
	{
		PX_ASSERT(!asPartOfCreation || (getActorInteractionCount() == 0)); // On creation or destruction there should be no interactions

		if(active)
		{
			if(!asPartOfCreation)
				getScene().addToActiveList(*this);	// Inactive => Active

			activate();

			PX_ASSERT(asPartOfCreation || isActive());
		}
		else
		{
			if(!asPartOfCreation)
				getScene().removeFromActiveList(*this);	// Active => Inactive

			deactivate();

			PX_ASSERT(asPartOfCreation || (!isActive()));
		}
	}
}

void Sc::ArticulationSim::setActive(bool b, bool asPartOfCreation)
{
	const PxReal wakeCounter = mCore.getWakeCounter();
	const PxU32 nbBodies = mBodies.size();
	for(PxU32 i=0;i<nbBodies;i++)
	{
		if(i+1 < nbBodies)
		{
			PxPrefetchLine(mBodies[i+1],0);
			PxPrefetchLine(mBodies[i+1],128);
		}
		//KS - force in the wake counter from the articulation to its links. This is required because
		//GPU articulation simulation does not DMA back wake counters for each link - it just brings back a global wake counter
		mBodies[i]->getBodyCore().setWakeCounterFromSim(wakeCounter);
		mBodies[i]->setActive(b, asPartOfCreation);
	}
}

// PT: moving all the sleeping-related implementations to the same file clearly exposes the inconsistencies between them
#if PX_SUPPORT_GPU_PHYSX
void Sc::ParticleSystemSim::setActive(bool /*active*/, bool /*asPartOfCreation*/)
{
}

void Sc::DeformableSurfaceSim::activate()
{
	mScene.getSimulationController()->activateCloth(mLLDeformableSurface);

	activateInteractions(*this);
}

void Sc::DeformableSurfaceSim::deactivate()
{
	mScene.getSimulationController()->deactivateCloth(mLLDeformableSurface);

	deactivateInteractions(*this);
}

void Sc::DeformableSurfaceSim::setActive(bool active, bool /*asPartOfCreation*/)
{
	if(active)
		activate();
	else
		deactivate();
}

void Sc::DeformableVolumeSim::setActive(bool active, bool /*asPartOfCreation*/)
{
	if(active)
		getScene().getSimulationController()->activateSoftbody(mLLDeformableVolume);
	else
		getScene().getSimulationController()->deactivateSoftbody(mLLDeformableVolume);
}

#endif

namespace
{
struct GetRigidSim	{ static PX_FORCE_INLINE BodySim* getSim(const IG::Node& node)			{ return reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(node.mObject) - BodySim::getRigidBodyOffset());		}	};
struct GetArticSim	{ static PX_FORCE_INLINE ArticulationSim* getSim(const IG::Node& node)	{ return static_cast<ArticulationSim*>(getObjectFromIG<FeatherstoneArticulation>(node));	}	};
#if PX_SUPPORT_GPU_PHYSX
struct GetDeformableSurfaceSim	{ static PX_FORCE_INLINE DeformableSurfaceSim* getSim(const IG::Node& node)	{ return getObjectFromIG<DeformableSurface>(node)->getSim();	}	};
struct GetDeformableVolumeSim	{ static PX_FORCE_INLINE DeformableVolumeSim* getSim(const IG::Node& node)	{ return getObjectFromIG<DeformableVolume>(node)->getSim();		}	};
#endif
}

// Rigid-body version of the setActive() template below for deactivation, with a prefetch pipeline. The stages run a
// few bodies ahead of the deactivation, each one fetching what the next one dereferences: the island node, the sim,
// the body core and the interaction and shape arrays, then the shapes, and the interactions of the bodies that have
// some to deactivate.
static void deactivateRigidBodies(PxU32& nbModified, const IG::IslandSim& islandSim)
{
	const PxU32 nbToProcess = islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	const PxNodeIndex* indices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

	const PxU32 nodeDistance = 8;
	const PxU32 simDistance = 6;
	const PxU32 arrayDistance = 4;
	const PxU32 elementDistance = 2;

	for(PxU32 i=0; i<nbToProcess; i++)
	{
		if(i + nodeDistance < nbToProcess)
			islandSim.prefetchNode(indices[i + nodeDistance]);
		if(i + simDistance < nbToProcess)
			GetRigidSim::getSim(islandSim.getNode(indices[i + simDistance]))->prefetchSelf();
		if(i + arrayDistance < nbToProcess)
			GetRigidSim::getSim(islandSim.getNode(indices[i + arrayDistance]))->prefetchCoreAndArrays();
		if(i + elementDistance < nbToProcess)
		{
			const BodySim* sim = GetRigidSim::getSim(islandSim.getNode(indices[i + elementDistance]));
			if(sim->readInternalFlag(ActorSim::BF_HAS_NON_IG_INTERACTIONS))
				sim->prefetchInteractions();
			sim->prefetchShapes();
		}

		const IG::Node& node = islandSim.getNode(indices[i]);
		PX_ASSERT(node.mType == IG::Node::eRIGID_BODY_TYPE);
		if(!node.isActive())
		{
			BodySim* sim = GetRigidSim::getSim(node);
			if(sim)
			{
				sim->setActive(false);
				nbModified++;
			}
		}
	}
}

template<class SimT, class SimAccessT, const bool active>
static void setActive(PxU32& nbModified, const IG::IslandSim& islandSim, IG::Node::NodeType type)
{
	PxU32 nbToProcess = active ? islandSim.getNbNodesToActivate(type) : islandSim.getNbNodesToDeactivate(type);
	const PxNodeIndex* indices = active ? islandSim.getNodesToActivate(type) : islandSim.getNodesToDeactivate(type);

	while(nbToProcess--)
	{
		const IG::Node& node = islandSim.getNode(*indices++);
		PX_ASSERT(node.mType == type);
		if((!!node.isActive())==active)
		{
			SimT* sim = SimAccessT::getSim(node);
			if(sim)
			{
				sim->setActive(active);
				nbModified++;
			}
		}
	}
}

#ifdef BATCHED
namespace
{
struct SetActiveRigidSim
{
	template<const bool active>
	static void setActive(Scene& /*scene*/, PxU32 nbObjects, BodySim** objects)
	{
		if(1)
		{
			while(nbObjects--)
			{
				(*objects)->setActive(active);
				objects++;
			}
		}
		else
		{
			if(active)
			{
//				scene.addToActiveList(*this);
//				activate();
//				PX_ASSERT(isActive());
			}
			else
			{
//				scene.removeFromActiveList(*this);
//				deactivate();
//				PX_ASSERT(!isActive());
			}
		}
	}
};
}

template<class SimT, class SimAccessT, class SetActiveBatchedT, const bool active>
static void setActiveBatched(Scene& scene, PxU32& nbModified, const IG::IslandSim& islandSim, IG::Node::NodeType type)
{
	PxU32 nbToProcess = active ? islandSim.getNbNodesToActivate(type) : islandSim.getNbNodesToDeactivate(type);
	const PxNodeIndex* indices = active ? islandSim.getNodesToActivate(type) : islandSim.getNodesToDeactivate(type);

	PX_ALLOCA(batch, SimT*, nbToProcess);

	PxU32 nb = 0;
	while(nbToProcess--)
	{
		const IG::Node& node = islandSim.getNode(*indices++);
		PX_ASSERT(node.mType == type);
		if(node.isActive()==active)
		{
			SimT* sim = SimAccessT::getSim(node);
			if(sim && sim->isActive()!=active)
				batch.mPointer[nb++] = sim;
		}
	}

	SetActiveBatchedT::setActive<active>(scene, nb, batch.mPointer);

	nbModified = nb;
}

/*
Batched version would be just:
a) addToActiveList(batched objects)
b) activate(batched objects)

void Sc::ActorSim::setActive(bool active)
{
	PX_ASSERT(!active || isDynamicRigid());  // Currently there should be no need to activate an actor that does not take part in island generation

	if(isActive() != active)
	{
		if(active)
		{
			// Inactive => Active
			getScene().addToActiveList(*this);

			activate();

			PX_ASSERT(isActive());
		}
		else
		{
			// Active => Inactive
			getScene().removeFromActiveList(*this);

			deactivate();

			PX_ASSERT(!isActive());
		}
	}
}
*/
#endif

void Sc::Scene::putObjectsToSleep()
{
	if(mPublicFlags & PxSceneFlag::eDISABLE_SLEEPING)
		return;

	PX_PROFILE_ZONE("Sc::Scene::putObjectsToSleep", mContextId);

	//Set to sleep all bodies that were in awake islands that have just been put to sleep.

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	PxU32 nbBodiesDeactivated = 0;
	//setActiveBatched<BodySim, GetRigidSim, SetActiveRigidSim, false>(*this, nbBodiesDeactivated, islandSim, IG::Node::eRIGID_BODY_TYPE);
	deactivateRigidBodies(nbBodiesDeactivated, islandSim);
	setActive<ArticulationSim, GetArticSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eARTICULATION_TYPE);

#if PX_SUPPORT_GPU_PHYSX
	setActive<DeformableSurfaceSim, GetDeformableSurfaceSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eDEFORMABLE_SURFACE_TYPE);
	setActive<DeformableVolumeSim, GetDeformableVolumeSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eDEFORMABLE_VOLUME_TYPE);
#endif

	if(nbBodiesDeactivated)
		mDynamicsContext->setStateDirty(true);
}

// Deactivates the interactions of the contact edges that island generation deactivated this step, for one edge type.
void Sc::Scene::deactivateEdgeInteractions(IG::Edge::EdgeType edgeType)
{
	const IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();

	const PxU32 nbDeactivatingEdges = islandSim.getNbDeactivatingEdges(edgeType);
	const IG::EdgeIndex* deactivatingEdgeIds = islandSim.getDeactivatingEdges(edgeType);

	// Three prefetch stages run ahead of the loop. Each one resolves the pointers the next stage needs: the
	// interaction table slot and the edge, then the interaction, then the actors and contact manager that
	// onDeactivate() reads.
	const PxU32 slotDistance = 16;
	const PxU32 interactionDistance = 8;
	const PxU32 objectDistance = 4;

	for(PxU32 i = 0; i < nbDeactivatingEdges; ++i)
	{
		if(i + slotDistance < nbDeactivatingEdges)
		{
			const IG::EdgeIndex edgeId = deactivatingEdgeIds[i + slotDistance];
			mSimpleIslandManager->prefetchInteractionFromEdgeIndex(edgeId);
			PxPrefetchLine(&islandSim.getEdge(edgeId));
		}
		if(i + interactionDistance < nbDeactivatingEdges)
		{
			const Interaction* next = mSimpleIslandManager->getInteractionFromEdgeIndex(deactivatingEdgeIds[i + interactionDistance]);
			if(next)
			{
				PxPrefetchLine(next);
				PxPrefetchLine(next, 64);
			}
		}
		if(i + objectDistance < nbDeactivatingEdges)
		{
			const Interaction* next = mSimpleIslandManager->getInteractionFromEdgeIndex(deactivatingEdgeIds[i + objectDistance]);
			if(next && next->getType() == InteractionType::eOVERLAP)
				static_cast<const ShapeInteraction*>(next)->prefetchDeactivationData();
		}

		Interaction* interaction = mSimpleIslandManager->getInteractionFromEdgeIndex(deactivatingEdgeIds[i]);

		if(interaction && interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			if(!islandSim.getEdge(deactivatingEdgeIds[i]).isActive())
			{
				const InteractionType::Enum type = interaction->getType();
				const bool proceed = (type == InteractionType::eOVERLAP) ?
					static_cast<ShapeInteraction*>(interaction)->onDeactivate(true) :
					deactivateInteraction(interaction, type);
				if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
					notifyInteractionDeactivated(interaction);
			}
		}
	}
}

void Sc::Scene::putInteractionsToSleep()
{
	PX_PROFILE_ZONE("Sc::Scene::putInteractionsToSleep", mContextId);

	//KS - only deactivate contact managers based on speculative state to trigger contact gen. When the actors were deactivated based on accurate state
	//joints should have been deactivated.

	// Shape interactions only mark their contact managers for removal here. The narrowphase then compacts its pair
	// arrays once for the whole island, instead of removing the pairs one at a time.
	for(PxU32 t = 0; t < IG::Edge::eEDGE_TYPE_COUNT; ++t)
	{
		const IG::Edge::EdgeType edgeType = IG::Edge::EdgeType(t);
		if(edgeType != IG::Edge::eCONSTRAINT)
			deactivateEdgeInteractions(edgeType);
	}

	{
		PX_PROFILE_ZONE("Sc::Scene::flushDeferredUnregisters", mContextId);
		mLLContext->getNphaseImplementationContext()->flushDeferredUnregisters();
	}
}

///////////////////////////////////////////////////////////////////////////////

// Parallel sleep path. For a large island the serial loops above are memory bound: nearly every body, pair and contact
// manager they touch is a cache miss. The tasks below do the part of the work that only writes to the object at hand
// from several threads, and leave the updates of shared scene data to two single-threaded passes:
//
// - body tasks: BodySim::deactivateThreadSafe() on each body, one BodySleepRecord per body.
// - finishPutObjectsToSleep(): compacts the active body list, finishes the bodies that have pending work, deactivates
//   the articulations and deformables, then starts the pair tasks.
// - pair tasks: ShapeInteraction::onDeactivateThreadSafe() on each pair, one record per pair. The detached contact
//   managers are marked for removal in the narrowphase here, with atomic bit sets.
// - finishPutInteractionsToSleep(): returns the manifolds and the contact managers to their pools in edge order,
//   which keeps the pools' free lists the same as with the serial path, handles the report pairs and the remaining
//   edge types, flushes the narrowphase and processes the persistent contact events.
namespace
{
	static const PxU32 gMinBodiesForParallelSleep = 1024;
	static const PxU32 gMinPairsForParallelSleep = 4096;
	static const PxU32 gMaxBodiesPerSleepTask = 512;
	static const PxU32 gMaxPairsPerSleepTask = 1024;

	// Scene::PairSleepRecord::mFlags bits. A record without flags has nothing to finish.
	static const PxU32 gPairRecordHasManager = 1 << 0;		// mManagerIndex and mManifold are valid
	static const PxU32 gPairRecordCCD = 1 << 1;				// the contact manager was registered for CCD
	static const PxU32 gPairRecordNeedsSerialWork = 1 << 2;	// the finish pass has to revisit the interaction

	// Same prefetch pipeline as deactivateRigidBodies(), without the interactions: the finish pass only visits them
	// for the bodies that have some to deactivate.
	void deactivateRigidBodiesThreadSafe(const IG::IslandSim& islandSim, const PxNodeIndex* indices, PxU32 nbBodies, Scene::BodySleepRecord* records)
	{
		const PxU32 nodeDistance = 8;
		const PxU32 simDistance = 6;
		const PxU32 arrayDistance = 4;
		const PxU32 shapeDistance = 2;

		for(PxU32 i=0; i<nbBodies; i++)
		{
			if(i + nodeDistance < nbBodies)
				islandSim.prefetchNode(indices[i + nodeDistance]);
			if(i + simDistance < nbBodies)
				GetRigidSim::getSim(islandSim.getNode(indices[i + simDistance]))->prefetchSelf();
			if(i + arrayDistance < nbBodies)
				GetRigidSim::getSim(islandSim.getNode(indices[i + arrayDistance]))->prefetchCoreAndShapeArray();
			if(i + shapeDistance < nbBodies)
				GetRigidSim::getSim(islandSim.getNode(indices[i + shapeDistance]))->prefetchShapes();

			Scene::BodySleepRecord& record = records[i];
			record.mActiveListIndex = PX_INVALID_U32;
			record.mPendingWork = 0;

			const IG::Node& node = islandSim.getNode(indices[i]);
			PX_ASSERT(node.mType == IG::Node::eRIGID_BODY_TYPE);
			if(!node.isActive())
			{
				BodySim* sim = GetRigidSim::getSim(node);
				if(sim && sim->isActive())
					record.mPendingWork = sim->deactivateThreadSafe(record.mActiveListIndex);
			}
		}
	}

	class PutBodiesToSleepTask : public Cm::Task
	{
	public:
		PutBodiesToSleepTask(PxU64 contextID, const IG::IslandSim& islandSim, const PxNodeIndex* indices, PxU32 nbBodies, Scene::BodySleepRecord* records) :
			Cm::Task	(contextID),
			mIslandSim	(islandSim),
			mIndices	(indices),
			mRecords	(records),
			mNbBodies	(nbBodies)
		{
		}

		virtual void runInternal() PX_OVERRIDE
		{
			PX_PROFILE_ZONE("Sc::Scene::putBodiesToSleepTask", mContextID);
			deactivateRigidBodiesThreadSafe(mIslandSim, mIndices, mNbBodies, mRecords);
		}

		virtual const char* getName() const PX_OVERRIDE { return "ScScene.putBodiesToSleepTask"; }

	private:
		const IG::IslandSim&			mIslandSim;
		const PxNodeIndex* const		mIndices;
		Scene::BodySleepRecord* const	mRecords;
		const PxU32						mNbBodies;

		PX_NOCOPY(PutBodiesToSleepTask)
	};

	// Same prefetch pipeline as Scene::deactivateEdgeInteractions(), plus the narrowphase slot of each pair's contact manager.
	void deactivatePairsThreadSafe(Scene& scene, const IG::IslandSim& islandSim, const IG::EdgeIndex* edgeIds, PxU32 nbEdges, Scene::PairSleepRecord* records)
	{
		IG::SimpleIslandManager* islandManager = scene.getSimpleIslandManager();
		PxvNphaseImplementationContext* nphase = scene.getLowLevelContext()->getNphaseImplementationContext();

		PxU32 nbRemoved = 0;

		const PxU32 slotDistance = 16;
		const PxU32 interactionDistance = 8;
		const PxU32 objectDistance = 4;
		const PxU32 cacheDistance = 2;

		for(PxU32 i=0; i<nbEdges; i++)
		{
			if(i + slotDistance < nbEdges)
			{
				const IG::EdgeIndex edgeId = edgeIds[i + slotDistance];
				islandManager->prefetchInteractionFromEdgeIndex(edgeId);
				PxPrefetchLine(&islandSim.getEdge(edgeId));
			}
			if(i + interactionDistance < nbEdges)
			{
				const Interaction* next = islandManager->getInteractionFromEdgeIndex(edgeIds[i + interactionDistance]);
				if(next)
				{
					PxPrefetchLine(next);
					PxPrefetchLine(next, 64);
				}
			}
			if(i + objectDistance < nbEdges)
			{
				const Interaction* next = islandManager->getInteractionFromEdgeIndex(edgeIds[i + objectDistance]);
				if(next && next->getType() == InteractionType::eOVERLAP)
					static_cast<const ShapeInteraction*>(next)->prefetchDeactivationData();
			}
			if(i + cacheDistance < nbEdges)
			{
				const Interaction* next = islandManager->getInteractionFromEdgeIndex(edgeIds[i + cacheDistance]);
				if(next && next->getType() == InteractionType::eOVERLAP)
				{
					const PxsContactManager* cm = static_cast<const ShapeInteraction*>(next)->getContactManager();
					if(cm)
						nphase->prefetchUnregisterSlot(cm);
				}
			}

			Scene::PairSleepRecord& record = records[i];
			record.mManifold = NULL;
			record.mManagerIndex = 0;
			record.mFlags = 0;

			const IG::EdgeIndex edgeId = edgeIds[i];
			Interaction* interaction = islandManager->getInteractionFromEdgeIndex(edgeId);
			if(interaction && interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE) && !islandSim.getEdge(edgeId).isActive())
			{
				if(interaction->getType() == InteractionType::eOVERLAP)
				{
					PxsContactManager* detachedManager;
					bool needsReportListRemoval;
					if(static_cast<ShapeInteraction*>(interaction)->onDeactivateThreadSafe(detachedManager, needsReportListRemoval))
					{
						if(detachedManager)
						{
							if(nphase->unregisterContactManagerDeferredThreadSafe(detachedManager, record.mManifold))
								nbRemoved++;
							record.mManagerIndex = detachedManager->getIndex();
							record.mFlags = detachedManager->getCCD() ? (gPairRecordHasManager | gPairRecordCCD) : gPairRecordHasManager;
						}
						if(needsReportListRemoval)
							record.mFlags |= gPairRecordNeedsSerialWork;
					}
				}
				else
				{
					// The other interaction types update shared scene data when they deactivate.
					record.mFlags = gPairRecordNeedsSerialWork;
				}
			}
		}

		nphase->addDeferredUnregisters(nbRemoved);
	}

	class PutPairsToSleepTask : public Cm::Task
	{
	public:
		PutPairsToSleepTask(Scene& scene, const IG::EdgeIndex* edgeIds, PxU32 nbEdges, Scene::PairSleepRecord* records) :
			Cm::Task	(scene.getContextId()),
			mScene		(scene),
			mEdgeIds	(edgeIds),
			mRecords	(records),
			mNbEdges	(nbEdges)
		{
		}

		virtual void runInternal() PX_OVERRIDE
		{
			PX_PROFILE_ZONE("Sc::Scene::putPairsToSleepTask", mContextID);
			deactivatePairsThreadSafe(mScene, mScene.getSimpleIslandManager()->getSpeculativeIslandSim(), mEdgeIds, mNbEdges, mRecords);
		}

		virtual const char* getName() const PX_OVERRIDE { return "ScScene.putPairsToSleepTask"; }

	private:
		Scene&							mScene;
		const IG::EdgeIndex* const		mEdgeIds;
		Scene::PairSleepRecord* const	mRecords;
		const PxU32						mNbEdges;

		PX_NOCOPY(PutPairsToSleepTask)
	};
}

bool Sc::Scene::putObjectsAndInteractionsToSleepParallel(PxBaseTask* continuation)
{
	PX_ASSERT(!(mPublicFlags & PxSceneFlag::eDISABLE_SLEEPING));

	// GPU scenes keep the serial path, see ShapeInteraction::onDeactivateThreadSafe().
	if(isUsingGpuDynamicsOrBp())
		return false;

	const IG::IslandSim& accurateSim = mSimpleIslandManager->getAccurateIslandSim();
	const PxU32 nbBodies = accurateSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	const PxU32 nbPairs = mSimpleIslandManager->getSpeculativeIslandSim().getNbDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);
	if(nbBodies < gMinBodiesForParallelSleep && nbPairs < gMinPairsForParallelSleep)
		return false;

	PX_PROFILE_ZONE("Sc::Scene::startPutObjectsToSleep", mContextId);

	// The chain runs the body tasks, finishPutObjectsToSleep(), the pair tasks, finishPutInteractionsToSleep() and then
	// the continuation of postThirdPassIslandGen().
	mFinishPutInteractionsToSleepTask.setContinuation(continuation);
	mFinishPutObjectsToSleepTask.setContinuation(&mFinishPutInteractionsToSleepTask);

	mBodySleepRecords.resizeUninitialized(nbBodies);
	if(nbBodies)
	{
		const PxNodeIndex* indices = accurateSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
		Cm::FlushPool& flushPool = mLLContext->getTaskPool();
		for(PxU32 start = 0; start < nbBodies; start += gMaxBodiesPerSleepTask)
		{
			const PxU32 nb = PxMin(gMaxBodiesPerSleepTask, nbBodies - start);
			PutBodiesToSleepTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PutBodiesToSleepTask)), PutBodiesToSleepTask)(mContextId, accurateSim, indices + start, nb, mBodySleepRecords.begin() + start);
			startTask(task, &mFinishPutObjectsToSleepTask);
		}
	}

	mFinishPutObjectsToSleepTask.removeReference();
	mFinishPutInteractionsToSleepTask.removeReference();
	return true;
}

void Sc::Scene::finishPutObjectsToSleep(PxBaseTask* continuation)
{
	{
		PX_PROFILE_ZONE("Sc::Scene::finishPutObjectsToSleep", mContextId);

		const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();
		const PxNodeIndex* indices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
		const PxU32 nbBodies = mBodySleepRecords.size();
		const BodySleepRecord* records = mBodySleepRecords.begin();

		PxU32 nbDeactivated = 0;
		PxU32 nbKinematicHoles = 0;
		PxBitMap holes;
		holes.resizeAndClear(mActiveBodies.size());

		// Only the bodies with pending work are visited again, so only their nodes and sims are prefetched.
		const PxU32 nodeDistance = 6;
		const PxU32 simDistance = 3;

		for(PxU32 i=0; i<nbBodies; i++)
		{
			if(i + nodeDistance < nbBodies && records[i + nodeDistance].mPendingWork)
				islandSim.prefetchNode(indices[i + nodeDistance]);
			if(i + simDistance < nbBodies && records[i + simDistance].mPendingWork)
				GetRigidSim::getSim(islandSim.getNode(indices[i + simDistance]))->prefetchSelf();

			const BodySleepRecord& record = records[i];
			if(record.mActiveListIndex == PX_INVALID_U32)
				continue;

			nbDeactivated++;
			holes.set(record.mActiveListIndex);
			if(record.mActiveListIndex < mActiveKinematicBodyCount)
				nbKinematicHoles++;

			if(record.mPendingWork)
				GetRigidSim::getSim(islandSim.getNode(indices[i]))->finishDeactivation(record.mPendingWork);
		}

		if(nbDeactivated)
			compactActiveBodies(holes, nbDeactivated, nbKinematicHoles);

		mBodySleepRecords.reset();

		setActive<ArticulationSim, GetArticSim, false>(nbDeactivated, islandSim, IG::Node::eARTICULATION_TYPE);
#if PX_SUPPORT_GPU_PHYSX
		setActive<DeformableSurfaceSim, GetDeformableSurfaceSim, false>(nbDeactivated, islandSim, IG::Node::eDEFORMABLE_SURFACE_TYPE);
		setActive<DeformableVolumeSim, GetDeformableVolumeSim, false>(nbDeactivated, islandSim, IG::Node::eDEFORMABLE_VOLUME_TYPE);
#endif

		if(nbDeactivated)
			mDynamicsContext->setStateDirty(true);
	}

	// The pairs can only be deactivated once all the bodies are, since each pair checks the state of its two actors.
	{
		const IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();
		const PxU32 nbPairs = islandSim.getNbDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);
		mPairSleepRecords.resizeUninitialized(nbPairs);
		if(nbPairs)
		{
			mLLContext->getNphaseImplementationContext()->beginDeferredUnregistersThreadSafe();

			const IG::EdgeIndex* edgeIds = islandSim.getDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);
			Cm::FlushPool& flushPool = mLLContext->getTaskPool();
			for(PxU32 start = 0; start < nbPairs; start += gMaxPairsPerSleepTask)
			{
				const PxU32 nb = PxMin(gMaxPairsPerSleepTask, nbPairs - start);
				PutPairsToSleepTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PutPairsToSleepTask)), PutPairsToSleepTask)(*this, edgeIds + start, nb, mPairSleepRecords.begin() + start);
				startTask(task, continuation);
			}
		}
	}
}

void Sc::Scene::finishPutInteractionsToSleep(PxBaseTask*)
{
	{
		PX_PROFILE_ZONE("Sc::Scene::finishPutInteractionsToSleep", mContextId);

		const IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();
		const IG::EdgeIndex* edgeIds = islandSim.getDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);
		const PxU32 nbPairs = mPairSleepRecords.size();
		const PairSleepRecord* records = mPairSleepRecords.begin();

		// The pools write their free-list link at the start of the manifold, and destroyManifold() reads the capacity further in.
		const PxU32 manifoldDistance = 8;

		for(PxU32 i=0; i<nbPairs; i++)
		{
			if(i + manifoldDistance < nbPairs)
			{
				const Gu::PersistentContactManifold* manifold = records[i + manifoldDistance].mManifold;
				if(manifold)
				{
					PxPrefetchLine(manifold);
					PxPrefetchLine(manifold, 64);
				}
			}

			const PairSleepRecord& record = records[i];
			if(!record.mFlags)
				continue;

			if(record.mFlags & gPairRecordHasManager)
			{
				if(record.mManifold)
					mLLContext->destroyManifold(record.mManifold);
				mLLContext->destroyContactManager(record.mManagerIndex, (record.mFlags & gPairRecordCCD) != 0);
			}

			if(record.mFlags & gPairRecordNeedsSerialWork)
			{
				Interaction* interaction = mSimpleIslandManager->getInteractionFromEdgeIndex(edgeIds[i]);
				PX_ASSERT(interaction);
				const InteractionType::Enum type = interaction->getType();
				if(type == InteractionType::eOVERLAP)
					static_cast<ShapeInteraction*>(interaction)->removeFromReportPairList();
				else if(interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE) && !islandSim.getEdge(edgeIds[i]).isActive())
				{
					if(deactivateInteraction(interaction, type) && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
						notifyInteractionDeactivated(interaction);
				}
			}
		}

		mPairSleepRecords.reset();

		// The tasks only cover the contact edges.
		for(PxU32 t = 0; t < IG::Edge::eEDGE_TYPE_COUNT; ++t)
		{
			const IG::Edge::EdgeType edgeType = IG::Edge::EdgeType(t);
			if(edgeType != IG::Edge::eCONSTRAINT && edgeType != IG::Edge::eCONTACT_MANAGER)
				deactivateEdgeInteractions(edgeType);
		}

		{
			PX_PROFILE_ZONE("Sc::Scene::flushDeferredUnregisters", mContextId);
			mLLContext->getNphaseImplementationContext()->flushDeferredUnregisters();
		}
	}

	PxvNphaseImplementationContext*	implCtx = mLLContext->getNphaseImplementationContext();
	PxsContactManagerOutputIterator outputs = implCtx->getContactManagerOutputs();
	mNPhaseCore->processPersistentContactEvents(outputs);
}

void Sc::Scene::wakeObjectsUp()
{
	PX_PROFILE_ZONE("Sc::Scene::wakeObjectsUp", mContextId);

	//Wake up all bodies that were in sleeping islands that have just been hit by a moving object.

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	PxU32 nbBodiesWoken = 0;
	setActive<BodySim, GetRigidSim, true>(nbBodiesWoken, islandSim, IG::Node::eRIGID_BODY_TYPE);
	setActive<ArticulationSim, GetArticSim, true>(nbBodiesWoken, islandSim, IG::Node::eARTICULATION_TYPE);

#if PX_SUPPORT_GPU_PHYSX
	setActive<DeformableSurfaceSim, GetDeformableSurfaceSim, true>(nbBodiesWoken, islandSim, IG::Node::eDEFORMABLE_SURFACE_TYPE);
	setActive<DeformableVolumeSim, GetDeformableVolumeSim, true>(nbBodiesWoken, islandSim, IG::Node::eDEFORMABLE_VOLUME_TYPE);
#endif

	if(nbBodiesWoken)
		mDynamicsContext->setStateDirty(true);
}

