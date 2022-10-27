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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_SCENE_H
#define SC_SCENE_H

#include "PxPhysXConfig.h"
#include "PxScene.h"
#include "PxSceneDesc.h"
#include "PxSimulationEventCallback.h"
#include "foundation/PxPool.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "CmTask.h"
#include "CmFlushPool.h"
#include "CmPreallocatingPool.h"
#include "foundation/PxBitMap.h"
#include "ScIterators.h"
#include "PxsMaterialManager.h"
#include "PxvManager.h"
#include "ScBodyCore.h"
#include "PxAggregate.h"
#include "PxsContext.h"
#include "PxsIslandSim.h"
#include "GuPrunerTypedef.h"

#define PX_MAX_DOMINANCE_GROUP 32

class OverlapFilterTask;

namespace physx
{
	class NpShape;

// PT: TODO: move INVALID_FILTER_PAIR_INDEX out of public API
struct PxFilterInfo
{
	PX_FORCE_INLINE	PxFilterInfo()								:	filterFlags(0), pairFlags(0), filterPairIndex(INVALID_FILTER_PAIR_INDEX)			{}
	PX_FORCE_INLINE	PxFilterInfo(PxFilterFlags filterFlags_)	:	filterFlags(filterFlags_), pairFlags(0), filterPairIndex(INVALID_FILTER_PAIR_INDEX)	{}

	PxFilterFlags	filterFlags;
	PxPairFlags		pairFlags;
	PxU32			filterPairIndex;
};

struct PxTriggerPair;

class PxsIslandManager;
class PxsSimulationController;
class PxsSimulationControllerCallback;
class PxsMemoryManager;

struct PxConeLimitedConstraint;

#if PX_SUPPORT_GPU_PHYSX
class PxsKernelWranglerManager;
class PxsHeapMemoryAllocatorManager;
#endif

namespace IG
{
	class SimpleIslandManager;
	class NodeIndex;
	typedef PxU32 EdgeIndex;
}

class PxsCCDContext;

namespace Cm
{
	class IDPool;
}

namespace Bp
{
	class AABBManagerBase;
	class BroadPhase;
	class BoundsArray;
}

namespace Dy
{
	class FeatherstoneArticulation;
	class Context;
#if PX_SUPPORT_GPU_PHYSX
	class SoftBody;
	class FEMCloth;
	class ParticleSystem;
	class HairSystem;
#endif
}

namespace Sc
{
	class ActorSim;
	class ElementSim;
	class Interaction;

	class ShapeCore;
	class RigidCore;
	class StaticCore;
	class ConstraintCore;
	class ArticulationCore;
	class ArticulationJointCore;
	class ArticulationSpatialTendonCore;
	class ArticulationFixedTendonCore;
	class ArticulationSensorCore;
	class LLArticulationPool;
	class LLArticulationRCPool;
	class LLSoftBodyPool;
	class LLFEMClothPool;
	class LLParticleSystemPool;
	class LLHairSystemPool;

	class BodyCore;
	class SoftBodyCore;
	class FEMClothCore;
	class ParticleSystemCore;
	class HairSystemCore;

	class NPhaseCore;
	class ConstraintInteraction;

	class BodySim;
	class ShapeSim;
	class RigidSim;
	class StaticSim;
	class ConstraintSim;
	struct ConstraintGroupNode;
	class ConstraintProjectionManager;
	struct TriggerPairExtraData;
	class ObjectIDTracker;
	class ActorPairReport;
	class ContactStreamManager;
	class SqBoundsManager;
	class ShapeInteraction;
	class ElementInteractionMarker;
	class ArticulationSim;
	class SoftBodySim;
	class FEMClothSim;
	class ParticleSystemSim;
	class HairSystemSim;

	class SimStats;

	struct SimStateData;

	struct BatchInsertionState
	{
		BodySim*  bodySim;
		StaticSim*staticSim; 
		ShapeSim* shapeSim;
		ptrdiff_t staticActorOffset;
		ptrdiff_t staticShapeTableOffset;
		ptrdiff_t dynamicActorOffset;
		ptrdiff_t dynamicShapeTableOffset;
		ptrdiff_t shapeOffset;
	};

	struct BatchRemoveState
	{
		PxInlineArray<ShapeSim*, 64>			bufferedShapes;
		PxInlineArray<const ShapeCore*, 64>		removedShapes;
	};

	struct InteractionType
	{
		enum Enum
		{
			eOVERLAP		= 0,		// corresponds to ShapeInteraction
			eTRIGGER,					// corresponds to TriggerInteraction
			eMARKER,					// corresponds to ElementInteractionMarker
			eTRACKED_IN_SCENE_COUNT,	// not a real type, interactions above this limit are tracked in the scene
			eCONSTRAINTSHADER,			// corresponds to ConstraintInteraction
			eARTICULATION,				// corresponds to ArticulationJointSim

			eINVALID
		};
	};

	struct SceneInternalFlag
	{
		enum Enum
		{
			eSCENE_SIP_STATES_DIRTY_DOMINANCE		= (1<<1),
			eSCENE_SIP_STATES_DIRTY_VISUALIZATION	= (1<<2),
			eSCENE_DEFAULT							= 0
		};
	};

	struct SimulationStage
	{
		enum Enum
		{
			eCOMPLETE,
			eCOLLIDE,
			eFETCHCOLLIDE,
			eADVANCE,
			eFETCHRESULT
		};
	};

	struct SqBoundsSync;
	struct SqRefFinder;

	struct ParticleOrSoftBodyRigidInteraction
	{
		IG::EdgeIndex mIndex;
		PxU32 mCount;

		ParticleOrSoftBodyRigidInteraction() : mCount(0) {}
	};

	class Scene : public PxUserAllocated
	{
		struct SimpleBodyPair
		{
			ActorSim* body1;
			ActorSim* body2;
			PxU32 body1ID;
			PxU32 body2ID;
		};

		PX_NOCOPY(Scene)

		//---------------------------------------------------------------------------------
		// External interface
		//---------------------------------------------------------------------------------
	public:
					void						release();

	PX_FORCE_INLINE	void						setGravity(const PxVec3& g)			{ mGravity = g;	mBodyGravityDirty = true;			}
	PX_FORCE_INLINE	PxVec3						getGravity()				const	{ return mGravity;									}
	PX_FORCE_INLINE void						setElapsedTime(const PxReal t)		{ mDt = t; mOneOverDt = t > 0.0f ? 1.0f/t : 0.0f;	}

					void						setBounceThresholdVelocity(const PxReal t);
					PxReal						getBounceThresholdVelocity() const;

					void						setPublicFlags(PxSceneFlags flags);
	PX_FORCE_INLINE	PxSceneFlags				getPublicFlags()			const	{ return mPublicFlags;	}
	PX_FORCE_INLINE	PxSceneFlags				getFlags()					const	{ return mPublicFlags;	}

					PxSolverType::Enum 			getSolverType() const;

					void						setFrictionType(PxFrictionType::Enum model);
					PxFrictionType::Enum 		getFrictionType() const;
	PX_FORCE_INLINE	void						setPCM(bool enabled)			{ mLLContext->setPCM(enabled);			}
	PX_FORCE_INLINE	void						setContactCache(bool enabled)	{ mLLContext->setContactCache(enabled);	}

					void						addStatic(StaticCore&, NpShape*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds);
					void						removeStatic(StaticCore&, PxInlineArray<const ShapeCore*,64>& removedShapes, bool wakeOnLostTouch);

					void						addBody(BodyCore&, NpShape*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds, bool compound);
					void						removeBody(BodyCore&, PxInlineArray<const ShapeCore*,64>& removedShapes, bool wakeOnLostTouch);

					// Batch insertion API.
					// the bounds generated here are the uninflated bounds for the shapes, *if* they are trigger or sim shapes. 
					// It's up to the caller to ensure the bounds array is big enough.
					// Some care is required in handling these since sim and SQ tweak the bounds in slightly different ways.

					void						startBatchInsertion(BatchInsertionState&);
					void						addStatic(PxActor* actor, BatchInsertionState&, PxBounds3* outBounds);
					void						addBody(PxActor* actor, BatchInsertionState&, PxBounds3* outBounds, bool compound);
					void						finishBatchInsertion(BatchInsertionState&);

					// Batch remove helpers
	PX_FORCE_INLINE	void						setBatchRemove(BatchRemoveState* bs)	{ mBatchRemoveState = bs;	}
	PX_FORCE_INLINE	BatchRemoveState*			getBatchRemove()		const			{ return mBatchRemoveState;	}

					void						addConstraint(ConstraintCore&, RigidCore*, RigidCore*);
					void						removeConstraint(ConstraintCore&);

					void						addSoftBody(SoftBodyCore&);
					void						removeSoftBody(SoftBodyCore&);

					void						addFEMCloth(FEMClothCore&);
					void						removeFEMCloth(FEMClothCore&);

					void						addParticleSystem(ParticleSystemCore&);
					void						removeParticleSystem(ParticleSystemCore&);

					void						addHairSystem(HairSystemCore&);
					void						removeHairSystem(HairSystemCore&);

					void						addArticulation(ArticulationCore&, BodyCore& root);
					void						removeArticulation(ArticulationCore&);

					void						addArticulationJoint(ArticulationJointCore&, BodyCore& parent, BodyCore& child);
					void						removeArticulationJoint(ArticulationJointCore&);

					void						addArticulationTendon(ArticulationSpatialTendonCore&);
					void						removeArticulationTendon(ArticulationSpatialTendonCore&);

					void						addArticulationTendon(ArticulationFixedTendonCore&);
					void						removeArticulationTendon(ArticulationFixedTendonCore&);

					void						addArticulationSensor(ArticulationSensorCore&);
					void						removeArticulationSensor(ArticulationSensorCore&);

					void						addArticulationSimControl(ArticulationCore& core);
					void						removeArticulationSimControl(ArticulationCore& core);
					void						addSoftBodySimControl(SoftBodyCore& core);
					void						removeSoftBodySimControl(SoftBodyCore& core);
					void						addFEMClothSimControl(FEMClothCore& core);
					void						removeFEMClothSimControl(FEMClothCore& core);
					void						addParticleSystemSimControl(ParticleSystemCore& core);
					void						removeParticleSystemSimControl(ParticleSystemCore& core);
					void						addHairSystemSimControl(HairSystemCore& core);
					void						removeHairSystemSimControl(HairSystemCore& core);

					void						updateBodySim(BodySim& sim);

	PX_FORCE_INLINE	PxU32						getNbArticulations() const	{ return mArticulations.size();			}
	PX_FORCE_INLINE	ArticulationCore* const*	getArticulations()			{ return mArticulations.getEntries();	}

#if PX_SUPPORT_GPU_PHYSX
	PX_FORCE_INLINE	PxU32						getNbSoftBodies()	const	{ return mSoftBodies.size();		}
	PX_FORCE_INLINE	SoftBodyCore* const*		getSoftBodies()				{ return mSoftBodies.getEntries();	}

	PX_FORCE_INLINE	PxU32						getNbFEMCloths()	const	{ return mFEMCloths.size();			}
	PX_FORCE_INLINE	FEMClothCore* const*		getFEMCloths()				{ return mFEMCloths.getEntries();	}

	PX_FORCE_INLINE	PxU32						getNbParticleSystems()	const	{ return mParticleSystems.size();		}
	PX_FORCE_INLINE	ParticleSystemCore* const*	getParticleSystems()			{ return mParticleSystems.getEntries();	}

	PX_FORCE_INLINE	PxU32						getNbHairSystems()	const	{ return mHairSystems.size(); }
	PX_FORCE_INLINE	HairSystemCore* const*		getHairSystems()			{ return mHairSystems.getEntries(); }
#endif
	
	PX_FORCE_INLINE	PxU32						getNbConstraints()	const	{ return mConstraints.size();		}
	PX_FORCE_INLINE	ConstraintCore*const*		getConstraints()	const	{ return mConstraints.getEntries();	}
	PX_FORCE_INLINE	ConstraintCore*const*		getConstraints()			{ return mConstraints.getEntries();	}

					void						initContactsIterator(ContactIterator&, PxsContactManagerOutputIterator&);

		// Simulation events
					void						setSimulationEventCallback(PxSimulationEventCallback* callback);
					PxSimulationEventCallback*	getSimulationEventCallback() const;

		// Contact modification
	PX_FORCE_INLINE	void						setContactModifyCallback(PxContactModifyCallback* callback)	{ mLLContext->setContactModifyCallback(callback);	}
	PX_FORCE_INLINE	PxContactModifyCallback*	getContactModifyCallback()							const	{ return mLLContext->getContactModifyCallback();	}

					void						setCCDContactModifyCallback(PxCCDContactModifyCallback* callback);
					PxCCDContactModifyCallback*	getCCDContactModifyCallback() const;

					void						setCCDMaxPasses(PxU32 ccdMaxPasses);
					PxU32						getCCDMaxPasses() const;

					void						setCCDThreshold(PxReal t);
					PxReal						getCCDThreshold() const;

		// Broad-phase callback
	PX_FORCE_INLINE	void						setBroadPhaseCallback(PxBroadPhaseCallback* callback)	{ mBroadPhaseCallback = callback;	}
	PX_FORCE_INLINE	PxBroadPhaseCallback*		getBroadPhaseCallback()	const							{ return mBroadPhaseCallback;		}

		// Broad-phase management
					void						finishBroadPhase(PxBaseTask* continuation);
					void						finishBroadPhaseStage2(const PxU32 ccdPass);
					void						preallocateContactManagers(PxBaseTask* continuation);

					void						islandInsertion(PxBaseTask* continuation);
					void						registerContactManagers(PxBaseTask* continuation);
					void						registerInteractions(PxBaseTask* continuation);
					void						registerSceneInteractions(PxBaseTask* continuation);

					void						secondPassNarrowPhase(PxBaseTask* continuation);

		// Groups
					void						setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance);
					PxDominanceGroupPair		getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const;

					void						setSolverBatchSize(PxU32 solverBatchSize);
					PxU32						getSolverBatchSize() const;

					void						setSolverArticBatchSize(PxU32 solverBatchSize);
					PxU32						getSolverArticBatchSize() const;

					void						setDynamicsDirty();

	PX_FORCE_INLINE	void						setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)
												{
													mVisualizationParameterChanged = true;
													mLLContext->setVisualizationParameter(param, value);
												}

	PX_FORCE_INLINE	PxReal						getVisualizationParameter(PxVisualizationParameter::Enum param) const	{ return mLLContext->getVisualizationParameter(param);	}
	PX_FORCE_INLINE	void						setVisualizationCullingBox(const PxBounds3& box)						{ mLLContext->setVisualizationCullingBox(box);			}
	PX_FORCE_INLINE	const PxBounds3&			getVisualizationCullingBox()									const	{ return mLLContext->getVisualizationCullingBox();		}
	PX_FORCE_INLINE	PxReal						getVisualizationScale()											const	{ return mLLContext->getRenderScale();					}

		// Run
					void						simulate(PxReal timeStep, PxBaseTask* continuation);
					void						advance(PxReal timeStep, PxBaseTask* continuation);
					void						collide(PxReal timeStep, PxBaseTask* continuation);
					void						endSimulation();
					void						flush(bool sendPendingReports);
					void						fireBrokenConstraintCallbacks();
					void						fireTriggerCallbacks();
					void						fireQueuedContactCallbacks();
					void						fireOnAdvanceCallback();

					const PxArray<PxContactPairHeader>&
												getQueuedContactPairHeaders();

					bool						fireOutOfBoundsCallbacks();
					void						prepareOutOfBoundsCallbacks();
					void						postCallbacksPreSync();
					void						postReportsCleanup();
					void						fireCallbacksPostSync();
					void						syncSceneQueryBounds(SqBoundsSync& sync, SqRefFinder& finder);					

					PxU32						getDefaultContactReportStreamBufferSize() const;

					void						setCCDMaxSeparation(PxReal separation);
					PxReal						getCCDMaxSeparation()			const;
					void						setMaxBiasCoefficient(PxReal coeff);
					PxReal						getMaxBiasCoefficient()			const;
					void						setFrictionOffsetThreshold(PxReal t);
					PxReal						getFrictionOffsetThreshold()	const;
					void						setFrictionCorrelationDistance(PxReal t);
					PxReal						getFrictionCorrelationDistance()	const;

	PX_FORCE_INLINE	void						setLimits(const PxSceneLimits& limits)	{ mLimits = limits;	}
	PX_FORCE_INLINE	const PxSceneLimits&		getLimits()						const	{ return mLimits;	}

					void						visualizeStartStep();
	PX_FORCE_INLINE	PxRenderBuffer&				getRenderBuffer()						{ return mLLContext->getRenderBuffer();	}

					void						setNbContactDataBlocks(PxU32 blockCount);
					PxU32						getNbContactDataBlocksUsed() const;
					PxU32						getMaxNbContactDataBlocksUsed() const;
					PxU32						getMaxNbConstraintDataBlocksUsed() const;

					void						setMaxArticulationLinks(const PxU32 maxLinks) { mMaxNbArticulationLinks = maxLinks;  }
					PxU32						getMaxArticulationLinks() const { return mMaxNbArticulationLinks; }

					void						setScratchBlock(void* addr, PxU32 size);

// PX_ENABLE_SIM_STATS
					void						getStats(PxSimulationStatistics& stats) const;
	PX_FORCE_INLINE	SimStats&					getStatsInternal() { return *mStats; }
// PX_ENABLE_SIM_STATS

					void						buildActiveActors();
					void						buildActiveAndFrozenActors();
					PxActor**					getActiveActors(PxU32& nbActorsOut);
					void						setActiveActors(PxActor** actors, PxU32 nbActors);

					PxActor**					getActiveSoftBodyActors(PxU32& nbActorsOut);
					void						setActiveSoftBodyActors(PxActor** actors, PxU32 nbActors);

					//PxActor**					getActiveFEMClothActors(PxU32& nbActorsOut);
					//void						setActiveFEMClothActors(PxActor** actors, PxU32 nbActors);

					PxActor**					getFrozenActors(PxU32& nbActorsOut);

					void						finalizeContactStreamAndCreateHeader(PxContactPairHeader& header, 
						const ActorPairReport& aPair, 
						ContactStreamManager& cs, PxU32 removedShapeTestMask);

					PxTaskManager*				getTaskManagerPtr() const { return mTaskManager; }
					PxCudaContextManager*		getCudaContextManager() const { return mCudaContextManager; }

					void						shiftOrigin(const PxVec3& shift);

	PX_FORCE_INLINE	PxPool<SimStateData>*		getSimStateDataPool() { return mSimStateDataPool; }

	PX_FORCE_INLINE bool isCollisionPhaseActive() const { return mIsCollisionPhaseActive; }
	PX_FORCE_INLINE void setCollisionPhaseToActive() { PX_ASSERT(!mIsCollisionPhaseActive); mIsCollisionPhaseActive = true; }
	PX_FORCE_INLINE void setCollisionPhaseToInactive() { PX_ASSERT(mIsCollisionPhaseActive); mIsCollisionPhaseActive = false; }

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------							
		//internal public methods:
	public:
												Scene(const PxSceneDesc& desc, PxU64 contextID);
												~Scene() {}	//use release() plz.

					void						preAllocate(PxU32 nbStatics, PxU32 nbBodies, PxU32 nbStaticShapes, PxU32 nbDynamicShapes);

	PX_FORCE_INLINE	PxsContext*					getLowLevelContext() { return mLLContext; }
	PX_FORCE_INLINE const PxsContext*			getLowLevelContext() const { return mLLContext; }

	PX_FORCE_INLINE	Dy::Context*				getDynamicsContext() { return mDynamicsContext; }
	PX_FORCE_INLINE const Dy::Context*			getDynamicsContext() const { return mDynamicsContext; }

	PX_FORCE_INLINE	PxsSimulationController*	getSimulationController() { return mSimulationController; }
	PX_FORCE_INLINE	const PxsSimulationController*	getSimulationController() const { return mSimulationController; }

	PX_FORCE_INLINE	Bp::AABBManagerBase*		getAABBManager()			{ return mAABBManager;	}
	PX_FORCE_INLINE const Bp::AABBManagerBase*	getAABBManager()	const	{ return mAABBManager;	}
	PX_FORCE_INLINE PxArray<BodySim*>&			getCcdBodies()				{ return mCcdBodies;	}

	PX_FORCE_INLINE	IG::SimpleIslandManager*	getSimpleIslandManager()	{ return mSimpleIslandManager; }
	PX_FORCE_INLINE	const IG::SimpleIslandManager*	getSimpleIslandManager() const { return mSimpleIslandManager; }

	PX_FORCE_INLINE SimulationStage::Enum		getSimulationStage() const { return mSimulationStage; }
	PX_FORCE_INLINE void						setSimulationStage(SimulationStage::Enum stage) { mSimulationStage = stage; }

					void						addShape_(RigidSim&, ShapeCore&);
					void						removeShape_(ShapeSim&, bool wakeOnLostTouch);

					void						registerShapeInNphase(RigidCore* rigidCore, const ShapeCore& shapeCore, const PxU32 transformCacheID);
					void						unregisterShapeFromNphase(const ShapeCore& shapeCore, const PxU32 transformCacheID);

					void						notifyNphaseOnUpdateShapeMaterial(const ShapeCore& shapeCore);

	// Get an array of the active actors.
	PX_FORCE_INLINE	BodyCore*const*				getActiveBodiesArray()			const	{ return mActiveBodies.begin();			}
	PX_FORCE_INLINE	PxU32						getNumActiveBodies()			const	{ return mActiveBodies.size();			}

#if PX_SUPPORT_GPU_PHYSX
	PX_FORCE_INLINE	SoftBodyCore*const*			getActiveSoftBodiesArray() const { return mActiveSoftBodies.begin(); }
	PX_FORCE_INLINE	PxU32						getNumActiveSoftBodies() const { return mActiveSoftBodies.size(); }

	PX_FORCE_INLINE	FEMClothCore*const*			getActiveFEMClothsArray() const { return mActiveFEMCloths.begin(); }
	PX_FORCE_INLINE	PxU32						getNumActiveFEMCloths() const { return mActiveFEMCloths.size(); }
	
	PX_FORCE_INLINE HairSystemCore*const*		getActiveHairSystemsArray() const { return mActiveHairSystems.begin(); }
	PX_FORCE_INLINE	PxU32						getNumActiveHairSystems() const { return mActiveHairSystems.size(); }
#endif

	PX_FORCE_INLINE	BodyCore*const*				getActiveCompoundBodiesArray()	const	{ return mActiveCompoundBodies.begin();	}
	PX_FORCE_INLINE	PxU32						getNumActiveCompoundBodies()	const	{ return mActiveCompoundBodies.size();	}

	PX_FORCE_INLINE	PxU32						getNbInteractions(InteractionType::Enum type)		const	{ return mInteractions[type].size();	}
	PX_FORCE_INLINE	PxU32						getNbActiveInteractions(InteractionType::Enum type)	const	{ return mActiveInteractionCount[type];	}
	// Get all interactions of a certain type
	PX_FORCE_INLINE	Interaction**				getInteractions(InteractionType::Enum type)					{ return mInteractions[type].begin();	}
	PX_FORCE_INLINE	Interaction**				getActiveInteractions(InteractionType::Enum type)			{ return mInteractions[type].begin();	}

					void						registerInteraction(Interaction* interaction, bool active);
					void						unregisterInteraction(Interaction* interaction);

					void						notifyInteractionActivated(Interaction* interaction);
					void						notifyInteractionDeactivated(Interaction* interaction);

	// for pool management of interaction arrays, a major cause of dynamic allocation
					void**						allocatePointerBlock(PxU32 size);
					void						deallocatePointerBlock(void**, PxU32 size);
	private:
	// Get the number of active one-way dominator actors
	PX_FORCE_INLINE	PxU32						getActiveKinematicBodiesCount() const { return mActiveKinematicBodyCount; }

	// Get an iterator to the active one-way dominator actors
	PX_FORCE_INLINE	BodyCore*const*				getActiveKinematicBodies() const { return mActiveBodies.begin(); }

	// Get the number of active non-kinematic actors
	PX_FORCE_INLINE	PxU32						getActiveDynamicBodiesCount() const { return mActiveBodies.size() - mActiveKinematicBodyCount; }

	// Get the active non-kinematic actors
	PX_FORCE_INLINE	BodyCore*const*				getActiveDynamicBodies() const { return mActiveBodies.begin() + mActiveKinematicBodyCount; }

#if PX_SUPPORT_GPU_PHYSX
	// Get the active soft body actors
	PX_FORCE_INLINE	SoftBodyCore*const*			getActiveSoftBodies() const { return mActiveSoftBodies.begin(); }

	// Get the active FEM-cloth actors
	PX_FORCE_INLINE	FEMClothCore*const*			getActiveFEMCloths() const { return mActiveFEMCloths.begin(); }
#endif

					void						swapInteractionArrayIndices(PxU32 id1, PxU32 id2, InteractionType::Enum type);
	public:
					PxReal						getLengthScale() const;

	PX_FORCE_INLINE PxBitMap&					getDirtyShapeSimMap() { return mDirtyShapeSimMap; }

					void						addDirtyArticulationSim(ArticulationSim* artiSim);
					void						removeDirtyArticulationSim(ArticulationSim* artiSim);
		
					void						addToActiveList(ActorSim& actor);
					void						removeFromActiveList(ActorSim& actor);
					void						removeFromActiveCompoundBodyList(BodySim& actor);
					void						swapInActiveBodyList(BodySim& body); // call when an active body gets switched from dynamic to kinematic or vice versa

					void						addBrokenConstraint(ConstraintCore*);
					void						addActiveBreakableConstraint(ConstraintSim*, ConstraintInteraction*);
					void						removeActiveBreakableConstraint(ConstraintSim*);
		//the Actor should register its top level shapes with these.
					void						removeBody(BodySim&);

					void						raiseSceneFlag(SceneInternalFlag::Enum flag) { mInternalFlags |= flag; }

					//lists of actors woken up or put to sleep last simulate
					void                        onBodyWakeUp(BodySim* body);
					void                        onBodySleep(BodySim* body);

	PX_FORCE_INLINE	bool						isValid() const	{ return (mLLContext != NULL);	}

					void						addToLostTouchList(ActorSim& body1, ActorSim& body2);

					PxU32						createAggregate(void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint);

					void						deleteAggregate(PxU32 id);

					Dy::FeatherstoneArticulation*	createLLArticulation(ArticulationSim* sim);
					void							destroyLLArticulation(Dy::FeatherstoneArticulation&);

#if PX_SUPPORT_GPU_PHYSX
					Dy::SoftBody*				createLLSoftBody(SoftBodySim* sim);
					void						destroyLLSoftBody(Dy::SoftBody& softBody);

					Dy::FEMCloth*				createLLFEMCloth(FEMClothSim* sim);
					void						destroyLLFEMCloth(Dy::FEMCloth& femCloth);

					Dy::ParticleSystem*			createLLParticleSystem(ParticleSystemSim* sim);
					void						destroyLLParticleSystem(Dy::ParticleSystem& softBody);

					Dy::HairSystem*				createLLHairSystem(HairSystemSim* sim);
					void						destroyLLHairSystem(Dy::HairSystem& hairSystem);
#endif

		PX_FORCE_INLINE	PxPool<ConstraintInteraction>*	getConstraintInteractionPool()			const	{ return mConstraintInteractionPool;	}
	public:
		PX_FORCE_INLINE	const PxsMaterialManager&			getMaterialManager()				const	{ return mMaterialManager;			}
		PX_FORCE_INLINE	PxsMaterialManager&					getMaterialManager()						{ return mMaterialManager;			}
#if PX_SUPPORT_GPU_PHYSX
		PX_FORCE_INLINE	const PxsFEMMaterialManager&		getFEMMaterialManager()				const	{ return mFEMMaterialManager;		}
		PX_FORCE_INLINE	PxsFEMMaterialManager&				getFEMMaterialManager()						{ return mFEMMaterialManager;		}

		PX_FORCE_INLINE	const PxsFEMClothMaterialManager&	getFEMClothMaterialManager()		const	{ return mFEMClothMaterialManager;	}
		PX_FORCE_INLINE	PxsFEMClothMaterialManager&			getFEMClothMaterialManager()				{ return mFEMClothMaterialManager;	}

		PX_FORCE_INLINE	const PxsPBDMaterialManager&		getPBDMaterialManager()				const	{ return mPBDMaterialManager;		}
		PX_FORCE_INLINE	PxsPBDMaterialManager&				getPBDMaterialManager()						{ return mPBDMaterialManager;		}

		PX_FORCE_INLINE	const PxsFLIPMaterialManager&		getFLIPMaterialManager()			const	{ return mFLIPMaterialManager;		}
		PX_FORCE_INLINE	PxsFLIPMaterialManager&				getFLIPMaterialManager()					{ return mFLIPMaterialManager;		}

		PX_FORCE_INLINE	const PxsMPMMaterialManager&		getMPMMaterialManager()				const	{ return mMPMMaterialManager;		}
		PX_FORCE_INLINE	PxsMPMMaterialManager&				getMPMMaterialManager()						{ return mMPMMaterialManager;		}

		PX_FORCE_INLINE	const PxsCustomMaterialManager&		getCustomMaterialManager()			const	{ return mCustomMaterialManager;	}
		PX_FORCE_INLINE	PxsCustomMaterialManager&			getCustomMaterialManager()					{ return mCustomMaterialManager;	}
#endif
		// Collision filtering
						void						setFilterShaderData(const void* data, PxU32 dataSize);
		PX_FORCE_INLINE	const void*					getFilterShaderDataFast()				const	{ return mFilterShaderData;				}
		PX_FORCE_INLINE	PxU32						getFilterShaderDataSizeFast()			const	{ return mFilterShaderDataSize;			}
		PX_FORCE_INLINE	PxSimulationFilterShader	getFilterShaderFast()					const	{ return mFilterShader;					}
		PX_FORCE_INLINE	PxSimulationFilterCallback*	getFilterCallbackFast()					const	{ return mFilterCallback;				}
		PX_FORCE_INLINE	PxPairFilteringMode::Enum	getKineKineFilteringMode()				const	{ return mKineKineFilteringMode;		}
		PX_FORCE_INLINE	PxPairFilteringMode::Enum	getStaticKineFilteringMode()			const	{ return mStaticKineFilteringMode;		}

		PX_FORCE_INLINE	PxU32						getTimeStamp()							const	{ return mTimeStamp;					}
		PX_FORCE_INLINE	PxU32						getReportShapePairTimeStamp()			const	{ return mReportShapePairTimeStamp;		}

		PX_FORCE_INLINE	PxReal						getOneOverDt()							const	{ return mOneOverDt;					}
		PX_FORCE_INLINE	PxReal						getDt()									const	{ return mDt;							}

		PX_FORCE_INLINE	const PxVec3&				getGravityFast()						const	{ return mGravity;						}
		PX_FORCE_INLINE	bool						readFlag(SceneInternalFlag::Enum flag)	const	{ return (mInternalFlags & flag) != 0;	}
		PX_FORCE_INLINE	bool						readPublicFlag(PxSceneFlag::Enum flag)	const	{ return (mPublicFlags & flag);			}

		PX_FORCE_INLINE	NPhaseCore*					getNPhaseCore()							const	{ return mNPhaseCore;					}

						void						checkConstraintBreakage();

		PX_FORCE_INLINE	PxArray<TriggerPairExtraData>&		
													getTriggerBufferExtraData()						{ return *mTriggerBufferExtraData;		}
		PX_FORCE_INLINE	PxArray<PxTriggerPair>&	getTriggerBufferAPI()							{ return mTriggerBufferAPI;				}
						void						reserveTriggerReportBufferSpace(const PxU32 pairCount, PxTriggerPair*& triggerPairBuffer, TriggerPairExtraData*& triggerPairExtraBuffer);

		PX_FORCE_INLINE	ObjectIDTracker&			getActorIDTracker()								{ return *mActorIDTracker;				}

		PX_FORCE_INLINE	void						markReleasedBodyIDForLostTouch(PxU32 id)		{ mLostTouchPairsDeletedBodyIDs.growAndSet(id); }
						void						resizeReleasedBodyIDMaps(PxU32 maxActors, PxU32 numActors);

		PX_FORCE_INLINE	StaticSim&					getStaticAnchor()								{ return *mStaticAnchor;				}

		PX_FORCE_INLINE	ConstraintProjectionManager& getProjectionManager()							{ return *mProjectionManager;			}

		PX_FORCE_INLINE Bp::BoundsArray&			getBoundsArray()						const	{ return *mBoundsArray; }
		PX_FORCE_INLINE void						updateContactDistance(PxU32 idx, PxReal distance)	{ (*mContactDistance)[idx] = distance; mHasContactDistanceChanged = true; }
		PX_FORCE_INLINE SqBoundsManager&			getSqBoundsManager()					const	{ return *mSqBoundsManager; }

		PX_FORCE_INLINE BodyCore* const*			getSleepBodiesArray(PxU32& count)				{ count = mSleepBodies.size(); return mSleepBodies.getEntries(); }

		PX_FORCE_INLINE SoftBodyCore* const*		getSleepSoftBodiesArray(PxU32& count)			{ count = mSleepSoftBodies.size(); return mSleepSoftBodies.getEntries(); }

		PX_FORCE_INLINE PxTaskManager&				getTaskManager()						const	{ PX_ASSERT(mTaskManager); return *mTaskManager; }

		Cm::FlushPool*								getFlushPool();
	
		PX_FORCE_INLINE bool						getStabilizationEnabled()				const	{ return mEnableStabilization; }

		PX_FORCE_INLINE void						setPostSolverVelocityNeeded()					{ mContactReportsNeedPostSolverVelocity = true; }

		PX_FORCE_INLINE	ObjectIDTracker&			getConstraintIDTracker()						{ return *mConstraintIDTracker; }
		PX_FORCE_INLINE	ObjectIDTracker&			getElementIDPool()								{ return *mElementIDPool; }
		PX_FORCE_INLINE PxBitMap&					getVelocityModifyMap()							{ return mVelocityModifyMap; }

						void*						allocateConstraintBlock(PxU32 size);
						void						deallocateConstraintBlock(void* addr, PxU32 size);

					void							stepSetupCollide(PxBaseTask* continuation);//This is very important to guarantee thread safty in the collide
		PX_FORCE_INLINE void						addToPosePreviewList(BodySim& b)				{ PX_ASSERT(!mPosePreviewBodies.contains(&b)); mPosePreviewBodies.insert(&b); }
		PX_FORCE_INLINE void						removeFromPosePreviewList(BodySim& b)			{ PX_ASSERT(mPosePreviewBodies.contains(&b)); mPosePreviewBodies.erase(&b); }
#if PX_DEBUG
		PX_FORCE_INLINE bool						isInPosePreviewList(BodySim& b)			const	{ return mPosePreviewBodies.contains(&b); }
#endif

		PX_FORCE_INLINE	void						setSpeculativeCCDRigidBody(const PxU32 index) { mSpeculativeCCDRigidBodyBitMap.growAndSet(index); }
		PX_FORCE_INLINE void						resetSpeculativeCCDRigidBody(const PxU32 index) { if(index < mSpeculativeCCDRigidBodyBitMap.size()) mSpeculativeCCDRigidBodyBitMap.reset(index); }

		PX_FORCE_INLINE	void						setSpeculativeCCDArticulationLink(const PxU32 index) { mSpeculativeCDDArticulationBitMap.growAndSet(index); }
		PX_FORCE_INLINE void						resetSpeculativeCCDArticulationLink(const PxU32 index) { if(index < mSpeculativeCDDArticulationBitMap.size()) mSpeculativeCDDArticulationBitMap.reset(index); }

		PX_FORCE_INLINE	PxU64						getContextId() const { return mContextId; }
		PX_FORCE_INLINE bool						isUsingGpuDynamicsOrBp() const { return mUseGpuBp || mUseGpuDynamics; }
		PX_FORCE_INLINE bool						isUsingGpuDynamicsAndBp() const { return mUseGpuBp && mUseGpuDynamics; }
		PX_FORCE_INLINE bool						isUsingGpuDynamics() const { return mUseGpuDynamics; }
		PX_FORCE_INLINE bool						isUsingGpuBp() const { return mUseGpuBp; }

		// statistics counters increase/decrease
		PX_FORCE_INLINE	void						increaseNumKinematicsCounter() { mNbRigidKinematic++; }
		PX_FORCE_INLINE	void						decreaseNumKinematicsCounter() { mNbRigidKinematic--; }
		PX_FORCE_INLINE	void						increaseNumDynamicsCounter() { mNbRigidDynamics++; }
		PX_FORCE_INLINE	void						decreaseNumDynamicsCounter() { mNbRigidDynamics--; }

						void						addParticleFilter(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId);
						void						removeParticleFilter(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId);

						PxU32						addParticleAttachment(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric);
						void						removeParticleAttachment(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 handle);

						void						addRigidFilter(BodyCore* core, SoftBodySim& sim, PxU32 vertId);
						void						removeRigidFilter(BodyCore* core, SoftBodySim& sim, PxU32 vertId);

						PxU32						addRigidAttachment(BodyCore* core, SoftBodySim& sim, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
						void						removeRigidAttachment(BodyCore* core, SoftBodySim& sim, PxU32 handle);
						
						void						addTetRigidFilter(BodyCore* core, SoftBodySim& sim, PxU32 tetIdx);
						void						removeTetRigidFilter(BodyCore* core, SoftBodySim& sim, PxU32 tetIdx);

						PxU32						addTetRigidAttachment(BodyCore* core, SoftBodySim& sim, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
						
						void						addSoftBodyFilter(SoftBodyCore& core, PxU32 tetIdx0,  SoftBodySim& sim, PxU32 tetIdx1);
						void						removeSoftBodyFilter(SoftBodyCore& core, PxU32 tetIdx0, SoftBodySim& sim, PxU32 tetIdx1);
						void						addSoftBodyFilters(SoftBodyCore& core, SoftBodySim& sim, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
						void						removeSoftBodyFilters(SoftBodyCore& core, SoftBodySim& sim, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

						PxU32						addSoftBodyAttachment(SoftBodyCore& core, PxU32 tetIdx0, const PxVec4& triBarycentric0, SoftBodySim& sim, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
													PxConeLimitedConstraint* constraint);
						void						removeSoftBodyAttachment(SoftBodyCore& core, SoftBodySim& sim, PxU32 handle);

						void						addClothFilter(Sc::FEMClothCore& core, PxU32 triIdx, Sc::SoftBodySim& sim, PxU32 tetIdx);
						void						removeClothFilter(Sc::FEMClothCore& core, PxU32 triIdx, Sc::SoftBodySim& sim, PxU32 tetIdx);

						PxU32						addClothAttachment(FEMClothCore& core, PxU32 triIdx, const PxVec4& triBarycentric, SoftBodySim& sim, PxU32 tetIdx, const PxVec4& tetBarycentric,
													PxConeLimitedConstraint* constraint);
						void						removeClothAttachment(FEMClothCore& core, SoftBodySim& sim, PxU32 handle);

						void						addRigidFilter(BodyCore* core, FEMClothSim& sim, PxU32 vertId);
						void						removeRigidFilter(BodyCore* core, FEMClothSim& sim, PxU32 vertId);

						PxU32						addRigidAttachment(BodyCore* core, FEMClothSim& sim, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
						void						removeRigidAttachment(BodyCore* core, FEMClothSim& sim, PxU32 handle);

						void						addClothFilter(FEMClothCore& core0, PxU32 triIdx0, Sc::FEMClothSim& sim1, PxU32 triIdx1);
						void						removeClothFilter(FEMClothCore& core, PxU32 triIdx0, FEMClothSim& sim1, PxU32 triIdx1);

						PxU32						addTriClothAttachment(FEMClothCore& core0, PxU32 triIdx0, const PxVec4& barycentric0, Sc::FEMClothSim& sim1, PxU32 triIdx1, const PxVec4& barycentric1);
						void						removeTriClothAttachment(FEMClothCore& core, FEMClothSim& sim1, PxU32 handle);

						void						addTriRigidFilter(BodyCore* core, FEMClothSim& sim, PxU32 triIdx);
						void						removeTriRigidFilter(BodyCore* core, FEMClothSim& sim, PxU32 triIdx);

						PxU32						addTriRigidAttachment(BodyCore* core, FEMClothSim& sim, PxU32 triIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
						void						removeTriRigidAttachment(BodyCore* core, FEMClothSim& sim, PxU32 handle);

						void						addRigidAttachment(BodyCore* core, ParticleSystemSim& sim);
						void						removeRigidAttachment(BodyCore* core, ParticleSystemSim& sim);

						void						addRigidAttachment(const BodyCore* core, const HairSystemSim& sim);
						void						removeRigidAttachment(const BodyCore* core, const HairSystemSim& sim);

						ConstraintCore*				findConstraintCore(const ActorSim* sim0, const ActorSim* sim1);

		//internal private methods:
	private:
					void						activateEdgesInternal(const IG::EdgeIndex* activatingEdges, const PxU32 nbActivatingEdges);
					void						releaseConstraints(bool endOfScene);
		PX_INLINE	void						clearBrokenConstraintBuffer();

		/////////////////////////////////////////////////////////////

					void						prepareCollide();

					void						collideStep(PxBaseTask* continuation);
					void						advanceStep(PxBaseTask* continuation);

		// subroutines of collideStep/solveStep:
					void						kinematicsSetup(PxBaseTask* continuation);
					void						stepSetupSolve(PxBaseTask* continuation);
					//void						stepSetupSimulate();

					void						fetchPatchEvents(PxBaseTask*);
					void						processNarrowPhaseTouchEvents();
					void						processNarrowPhaseTouchEventsStage2(PxBaseTask*);
					void						setEdgesConnected(PxBaseTask*);
					void						processNarrowPhaseLostTouchEvents(PxBaseTask*);
					void						processNarrowPhaseLostTouchEventsIslands(PxBaseTask*);
					void						processLostTouchPairs();
					void						integrateKinematicPose();
					void						updateKinematicCached(PxBaseTask* task);

					void						beforeSolver(PxBaseTask* continuation);
					void						checkForceThresholdContactEvents(const PxU32 ccdPass);
					void						endStep();
	private:
					void						putObjectsToSleep(PxU32 infoFlag);
					void						wakeObjectsUp(PxU32 infoFlag);

					void						collectPostSolverVelocitiesBeforeCCD();

					void						clearSleepWakeBodies(void);
		PX_INLINE	void						cleanUpSleepBodies();
		PX_INLINE	void						cleanUpWokenBodies();
		PX_INLINE	void						cleanUpSleepOrWokenBodies(PxCoalescedHashSet<BodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker);

#if PX_SUPPORT_GPU_PHYSX
		// PT: TODO: why inline these ones?
		PX_INLINE	void						cleanUpSleepSoftBodies();
		PX_INLINE	void						cleanUpWokenSoftBodies();
		PX_INLINE	void						cleanUpSleepOrWokenSoftBodies(PxCoalescedHashSet<SoftBodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker);

		PX_INLINE	void						cleanUpSleepHairSystems();
		PX_INLINE	void						cleanUpWokenHairSystems();
		PX_INLINE	void						cleanUpSleepOrWokenHairSystems(PxCoalescedHashSet<HairSystemCore*>& bodyList, PxU32 removeFlag, bool& validMarker);
#endif

		//internal variables:
	private:
		// Material manager
					PX_ALIGN(16, PxsMaterialManager	mMaterialManager);
#if PX_SUPPORT_GPU_PHYSX
					PX_ALIGN(16, PxsFEMMaterialManager	mFEMMaterialManager);

					PX_ALIGN(16, PxsFEMClothMaterialManager	mFEMClothMaterialManager);

					PX_ALIGN(16, PxsPBDMaterialManager	mPBDMaterialManager);

					PX_ALIGN(16, PxsFLIPMaterialManager	mFLIPMaterialManager);

					PX_ALIGN(16, PxsMPMMaterialManager	mMPMMaterialManager);

					PX_ALIGN(16, PxsCustomMaterialManager	mCustomMaterialManager);
#endif
					PxU64						mContextId;

					PxArray<BodyCore*>			mActiveBodies;  // Sorted: kinematic before dynamic
					PxU32						mActiveKinematicBodyCount;  // Number of active kinematics. This is also the index in mActiveBodies where the active dynamic bodies start.
					PxU32						mActiveDynamicBodyCount;  // Number of active dynamics. This is also the index in mActiveBodies where the active soft bodies start.
					PxArray<BodyCore*>			mActiveCompoundBodies;

					BodyCore**					mActiveKinematicsCopy;
					PxU32						mActiveKinematicsCopyCapacity;

#if PX_SUPPORT_GPU_PHYSX
					PxArray<SoftBodyCore*>			mActiveSoftBodies;
					PxArray<FEMClothCore*>			mActiveFEMCloths;
					PxArray<ParticleSystemCore*>	mActiveParticleSystems;
					PxArray<HairSystemCore*>		mActiveHairSystems;
#endif
					PxArray<Interaction*>		mInteractions[InteractionType::eTRACKED_IN_SCENE_COUNT];
					PxU32						mActiveInteractionCount[InteractionType::eTRACKED_IN_SCENE_COUNT]; // Interactions with id < activeInteractionCount are active

					template <typename T, PxU32 size>
					struct Block
					{
						PxU8 mem[sizeof(T)*size];
						Block() {}	// get around VS warning C4345, otherwise useless
					};

					typedef Block<void*, 8>	PointerBlock8;
					typedef Block<void*, 16> PointerBlock16;
					typedef Block<void*, 32> PointerBlock32;

					PxPool<PointerBlock8>	mPointerBlock8Pool;
					PxPool<PointerBlock16>	mPointerBlock16Pool;
					PxPool<PointerBlock32>	mPointerBlock32Pool;

					PxsContext*					mLLContext;

					Bp::AABBManagerBase*		mAABBManager;
					PxsCCDContext*				mCCDContext;
					PxI32						mNumFastMovingShapes;
					PxU32						mCCDPass;

					//PxsIslandManager*			mIslandManager;

					IG::SimpleIslandManager*	mSimpleIslandManager;

					Dy::Context*				mDynamicsContext;


					PxsMemoryManager*			mMemoryManager;

#if PX_SUPPORT_GPU_PHYSX
					PxsKernelWranglerManager*				mGpuWranglerManagers;
					PxsHeapMemoryAllocatorManager*			mHeapMemoryAllocationManager;
#endif

					PxsSimulationController*	mSimulationController;

					PxsSimulationControllerCallback*	mSimulationControllerCallback;

					PxSceneLimits				mLimits;

					PxVec3						mGravity;			//!< Gravity vector
					PxU32						mBodyGravityDirty; // Set to true before body->updateForces() when the mGravity has changed					

					PxArray<PxContactPairHeader>
												mQueuedContactPairHeaders;
		//time:
		//constants set with setTiming():
					PxReal						mDt;						//delta time for current step.
					PxReal						mOneOverDt;					//inverse of dt.
		//stepping / counters:
					PxU32						mTimeStamp;					//Counts number of steps.
					PxU32						mReportShapePairTimeStamp;	//Timestamp for refreshing the shape pair report structure. Updated before delayed shape/actor deletion and before CCD passes.
		//containers:
		// Those ones contain shape ptrs from Actor, i.e. compound level, not subparts

					PxCoalescedHashSet<ConstraintCore*>		mConstraints;
												
					ConstraintProjectionManager*			mProjectionManager;
					Bp::BoundsArray*						mBoundsArray;
					PxFloatArrayPinned*						mContactDistance;
					bool									mHasContactDistanceChanged;
					SqBoundsManager*						mSqBoundsManager;

					PxArray<BodySim*>				mCcdBodies;
					PxArray<BodySim*>				mProjectedBodies;
					PxArray<PxTriggerPair>			mTriggerBufferAPI;
					PxArray<TriggerPairExtraData>*	mTriggerBufferExtraData;

					PxCoalescedHashSet<ArticulationCore*> mArticulations;
					PxCoalescedHashSet<ArticulationSim*> mDirtyArticulationSims;

#if PX_SUPPORT_GPU_PHYSX
					PxCoalescedHashSet<SoftBodyCore*> mSoftBodies;
					PxCoalescedHashSet<FEMClothCore*> mFEMCloths;
					PxCoalescedHashSet<ParticleSystemCore*> mParticleSystems;
					PxCoalescedHashSet<HairSystemCore*> mHairSystems;
#endif
					PxArray<ConstraintCore*>	mBrokenConstraints;
					PxCoalescedHashSet<ConstraintSim*> mActiveBreakableConstraints;

					// pools for joint buffers
					// Fixed joint is 92 bytes, D6 is 364 bytes right now. So these three pools cover all the internal cases
					typedef Block<PxU8, 128> MemBlock128;
					typedef Block<PxU8, 256> MemBlock256;
					typedef Block<PxU8, 384> MemBlock384;

					PxPool2<MemBlock128, 8192>	mMemBlock128Pool;
					PxPool2<MemBlock256, 8192>	mMemBlock256Pool;
					PxPool2<MemBlock384, 8192>	mMemBlock384Pool;

					// broad phase data:
					NPhaseCore*					mNPhaseCore;

					// Collision filtering
					void*						mFilterShaderData;
					PxU32						mFilterShaderDataSize;
					PxU32						mFilterShaderDataCapacity;
					PxSimulationFilterShader	mFilterShader;
					PxSimulationFilterCallback*	mFilterCallback;

			const	PxPairFilteringMode::Enum	mKineKineFilteringMode;
			const	PxPairFilteringMode::Enum	mStaticKineFilteringMode;

					PxCoalescedHashSet<BodyCore*> mSleepBodies;
					PxCoalescedHashSet<BodyCore*> mWokeBodies;

					PxCoalescedHashSet<SoftBodyCore*> mSleepSoftBodies;
					PxCoalescedHashSet<SoftBodyCore*> mWokeSoftBodies;
					PxCoalescedHashSet<HairSystemCore*> mSleepHairSystems;
					PxCoalescedHashSet<HairSystemCore*> mWokeHairSystems;
					bool						mWokeBodyListValid;
					bool						mSleepBodyListValid;
					bool						mWokeSoftBodyListValid;
					bool						mSleepSoftBodyListValid;
					bool						mWokeHairSystemListValid;
					bool						mSleepHairSystemListValid;
			const	bool						mEnableStabilization;

						PxArray<PxActor*>				mActiveActors;
						PxArray<PxActor*>				mFrozenActors;

#if PX_SUPPORT_GPU_PHYSX
						PxArray<PxActor*>				mActiveSoftBodyActors;
						PxArray<PxActor*>				mActiveFEMClothActors;
						PxArray<PxActor*>				mActiveHairSystemActors;
#endif
						PxArray<const PxRigidBody*>	mClientPosePreviewBodies;	// buffer for bodies that requested early report of the integrated pose (eENABLE_POSE_INTEGRATION_PREVIEW).
																			// This buffer gets exposed to users. Is officially accessible from PxSimulationEventCallback::onAdvance()
																			// until the next simulate()/advance().
						PxArray<PxTransform>			mClientPosePreviewBuffer;	// buffer of newly integrated poses for the bodies that requested a preview. This buffer gets exposed
																			// to users.

						PxSimulationEventCallback*	mSimulationEventCallback;
						PxBroadPhaseCallback*		mBroadPhaseCallback;

					SimStats*					mStats;
					PxU32						mInternalFlags;	//!< Combination of ::SceneFlag
					PxSceneFlags				mPublicFlags;	//copy of PxSceneDesc::flags, of type PxSceneFlag

					ObjectIDTracker*			mConstraintIDTracker;
					ObjectIDTracker*			mActorIDTracker;
					ObjectIDTracker*			mElementIDPool;

					StaticSim*					mStaticAnchor;

					Cm::PreallocatingPool<ShapeSim>*	mShapeSimPool;
					Cm::PreallocatingPool<StaticSim>*	mStaticSimPool;
					Cm::PreallocatingPool<BodySim>*		mBodySimPool;
					PxPool<ConstraintSim>*				mConstraintSimPool;
					LLArticulationRCPool*				mLLArticulationRCPool;
#if PX_SUPPORT_GPU_PHYSX
					LLSoftBodyPool*						mLLSoftBodyPool;
					LLFEMClothPool*						mLLFEMClothPool;
					LLParticleSystemPool*				mLLParticleSystemPool;
					LLHairSystemPool*					mLLHairSystemPool;
#endif
					PxHashMap<PxPair<PxU32, PxU32>, ParticleOrSoftBodyRigidInteraction> mParticleOrSoftBodyRigidInteractionMap;

					PxHashMap<PxPair<const ActorSim*, const ActorSim*>, ConstraintCore*> mConstraintMap;
														
					PxPool<ConstraintInteraction>*	mConstraintInteractionPool;

					PxPool<SimStateData>*		mSimStateDataPool;

					BatchRemoveState*			mBatchRemoveState;

					PxArray<SimpleBodyPair>	mLostTouchPairs;
					PxBitMap					mLostTouchPairsDeletedBodyIDs;	// Need to know which bodies have been deleted when processing the lost touch pair list.
																				// Can't use the existing rigid object ID tracker class since this map needs to be cleared at
																				// another point in time.
					PxBitMap					mVelocityModifyMap;

					PxArray<PxvContactManagerTouchEvent> mTouchFoundEvents;
					PxArray<PxvContactManagerTouchEvent> mTouchLostEvents;

					PxArray<PxU32>							mOutOfBoundsIDs;

					PxBitMap								mDirtyShapeSimMap;

					PxU32						mDominanceBitMatrix[PX_MAX_DOMINANCE_GROUP];

					bool						mVisualizationParameterChanged;

					PxU32						mMaxNbArticulationLinks;

					// statics:
					PxU32						mNbRigidStatics;
					PxU32						mNbRigidDynamics;
					PxU32						mNbRigidKinematic;
					PxU32						mNbGeometries[PxGeometryType::eGEOMETRY_COUNT];

					//IG::Node::eTYPE_COUNT
					PxU32						mNumDeactivatingNodes[IG::Node::eTYPE_COUNT];

					// task decomposition
					void						broadPhase(PxBaseTask* continuation);
					void						broadPhaseFirstPass(PxBaseTask* continuation);
					void						broadPhaseSecondPass(PxBaseTask* continuation);
					void						updateBroadPhase(PxBaseTask* continuation);
					void						preIntegrate(PxBaseTask* continuation);
					void						postBroadPhase(PxBaseTask* continuation);
					void						postBroadPhaseContinuation(PxBaseTask* continuation);
					void						preRigidBodyNarrowPhase(PxBaseTask* continuation);
					void						postBroadPhaseStage2(PxBaseTask* continuation);
					void						postBroadPhaseStage3(PxBaseTask* continuation);
					void						updateBoundsAndShapes(PxBaseTask* continuation);
					void						rigidBodyNarrowPhase(PxBaseTask* continuation);
					void						unblockNarrowPhase(PxBaseTask* continuation);
					void						islandGen(PxBaseTask* continuation);
					void						processLostSolverPatches(PxBaseTask* continuation);
					void						processFoundSolverPatches(PxBaseTask* continuation);
					void						postIslandGen(PxBaseTask* continuation);
					//void						processTriggerInteractions(PxBaseTask* continuation);
					void						solver(PxBaseTask* continuation);
					void						updateBodies(PxBaseTask* continuation);
					void						updateShapes(PxBaseTask* continuation);
					void						updateSimulationController(PxBaseTask* continuation);
					void						updateDynamics(PxBaseTask* continuation);
					void						processLostContacts(PxBaseTask*);
					void						processLostContacts2(PxBaseTask*);
					void						processLostContacts3(PxBaseTask*);
					void						destroyManagers(PxBaseTask*);
					void						lostTouchReports(PxBaseTask*);
					void						unregisterInteractions(PxBaseTask*);
					void						postThirdPassIslandGen(PxBaseTask*);
					void						postSolver(PxBaseTask* continuation);
					void						constraintProjection(PxBaseTask* continuation);
					void						afterIntegration(PxBaseTask* continuation);  // performs sleep check, for instance
					void						postCCDPass(PxBaseTask* continuation);
					void						ccdBroadPhaseAABB(PxBaseTask* continuation);
					void						ccdBroadPhase(PxBaseTask* continuation);
					void						updateCCDMultiPass(PxBaseTask* continuation);
					void						updateCCDSinglePass(PxBaseTask* continuation);
					void						updateCCDSinglePassStage2(PxBaseTask* continuation);
					void						updateCCDSinglePassStage3(PxBaseTask* continuation);
					void						finalizationPhase(PxBaseTask* continuation);

					void						postNarrowPhase(PxBaseTask* continuation);

					void						addShapes(NpShape*const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& sim, PxBounds3* outBounds);
					void						removeShapes(RigidSim& , PxInlineArray<ShapeSim*, 64>& , PxInlineArray<const ShapeCore*, 64>&, bool wakeOnLostTouch);

	private:
					void						addShapes(NpShape*const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& sim, ShapeSim*& prefetchedShapeSim, PxBounds3* outBounds);
					void						updateContactDistances(PxBaseTask* continuation);

					Cm::DelegateTask<Scene, &Scene::secondPassNarrowPhase>		mSecondPassNarrowPhase;
					Cm::DelegateFanoutTask<Scene, &Scene::postNarrowPhase>		mPostNarrowPhase;
					Cm::DelegateFanoutTask<Scene, &Scene::finalizationPhase>	mFinalizationPhase;
					Cm::DelegateTask<Scene, &Scene::updateCCDMultiPass>			mUpdateCCDMultiPass;

					//multi-pass ccd stuff
					PxArray<Cm::DelegateTask<Scene, &Scene::updateCCDSinglePass> >			mUpdateCCDSinglePass;
					PxArray<Cm::DelegateTask<Scene, &Scene::updateCCDSinglePassStage2> >	mUpdateCCDSinglePass2;
					PxArray<Cm::DelegateTask<Scene, &Scene::updateCCDSinglePassStage3> >	mUpdateCCDSinglePass3;
					PxArray<Cm::DelegateTask<Scene, &Scene::ccdBroadPhaseAABB> >			mCCDBroadPhaseAABB;
					PxArray<Cm::DelegateTask<Scene, &Scene::ccdBroadPhase> >				mCCDBroadPhase;
					PxArray<Cm::DelegateTask<Scene, &Scene::postCCDPass> >					mPostCCDPass;

					Cm::DelegateTask<Scene, &Scene::afterIntegration>					mAfterIntegration;
					Cm::DelegateTask<Scene, &Scene::constraintProjection>				mConstraintProjection;
					Cm::DelegateTask<Scene, &Scene::postSolver>							mPostSolver;
					Cm::DelegateTask<Scene, &Scene::solver>								mSolver;
					Cm::DelegateTask<Scene, &Scene::updateBodies>						mUpdateBodies;
					Cm::DelegateTask<Scene, &Scene::updateShapes>						mUpdateShapes;
					Cm::DelegateTask<Scene, &Scene::updateSimulationController>			mUpdateSimulationController;
					Cm::DelegateTask<Scene, &Scene::updateDynamics>						mUpdateDynamics;
					Cm::DelegateTask<Scene, &Scene::processLostContacts>				mProcessLostContactsTask;
					Cm::DelegateTask<Scene, &Scene::processLostContacts2>				mProcessLostContactsTask2;
					Cm::DelegateTask<Scene, &Scene::processLostContacts3>				mProcessLostContactsTask3;
					Cm::DelegateTask<Scene, &Scene::destroyManagers>					mDestroyManagersTask;
					Cm::DelegateTask<Scene, &Scene::lostTouchReports>					mLostTouchReportsTask;
					Cm::DelegateTask<Scene, &Scene::unregisterInteractions>				mUnregisterInteractionsTask;
					Cm::DelegateTask<Scene,
						&Scene::processNarrowPhaseLostTouchEventsIslands>					mProcessNarrowPhaseLostTouchTasks;
					Cm::DelegateTask<Scene,
						&Scene::processNarrowPhaseLostTouchEvents>							mProcessNPLostTouchEvents;
					Cm::DelegateTask<Scene, &Scene::postThirdPassIslandGen>				mPostThirdPassIslandGenTask;
					Cm::DelegateTask<Scene, &Scene::postIslandGen>						mPostIslandGen;
					Cm::DelegateTask<Scene, &Scene::islandGen>							mIslandGen;
					Cm::DelegateTask<Scene, &Scene::preRigidBodyNarrowPhase>			mPreRigidBodyNarrowPhase;
					Cm::DelegateTask<Scene, &Scene::setEdgesConnected>					mSetEdgesConnectedTask;
					Cm::DelegateTask<Scene, &Scene::fetchPatchEvents>					mFetchPatchEventsTask;
					Cm::DelegateTask<Scene, &Scene::processLostSolverPatches>			mProcessLostPatchesTask;
					Cm::DelegateTask<Scene, &Scene::processFoundSolverPatches>			mProcessFoundPatchesTask;
					Cm::DelegateFanoutTask<Scene, &Scene::updateBoundsAndShapes>		mUpdateBoundAndShapeTask;
					Cm::DelegateTask<Scene, &Scene::rigidBodyNarrowPhase>				mRigidBodyNarrowPhase;
					Cm::DelegateTask<Scene, &Scene::unblockNarrowPhase>					mRigidBodyNPhaseUnlock;
					Cm::DelegateTask<Scene, &Scene::postBroadPhase>						mPostBroadPhase;
					Cm::DelegateTask<Scene, &Scene::postBroadPhaseContinuation>			mPostBroadPhaseCont;
					Cm::DelegateTask<Scene, &Scene::postBroadPhaseStage2>				mPostBroadPhase2;
					Cm::DelegateFanoutTask<Scene, &Scene::postBroadPhaseStage3>			mPostBroadPhase3;
					Cm::DelegateTask<Scene, &Scene::preallocateContactManagers>			mPreallocateContactManagers;
					Cm::DelegateTask<Scene, &Scene::islandInsertion>					mIslandInsertion;
					Cm::DelegateTask<Scene, &Scene::registerContactManagers>			mRegisterContactManagers;
					Cm::DelegateTask<Scene, &Scene::registerInteractions>				mRegisterInteractions;
					Cm::DelegateTask<Scene, &Scene::registerSceneInteractions>			mRegisterSceneInteractions;
					Cm::DelegateTask<Scene, &Scene::broadPhase>							mBroadPhase;
					Cm::DelegateTask<Scene, &Scene::advanceStep>						mAdvanceStep;
					Cm::DelegateTask<Scene, &Scene::collideStep>						mCollideStep;
					Cm::DelegateTask<Scene, &Scene::broadPhaseFirstPass>				mBpFirstPass;
					Cm::DelegateTask<Scene, &Scene::broadPhaseSecondPass>				mBpSecondPass;
					Cm::DelegateTask<Scene, &Scene::updateBroadPhase>					mBpUpdate;
					Cm::DelegateTask<Scene, &Scene::preIntegrate>	                    mPreIntegrate;

					Cm::FlushPool														mTaskPool;
					PxTaskManager*														mTaskManager;
					PxCudaContextManager*												mCudaContextManager;

					bool																mContactReportsNeedPostSolverVelocity;			
					bool																mUseGpuDynamics;
					bool																mUseGpuBp;
					bool																mCCDBp;

					SimulationStage::Enum												mSimulationStage;

					ConstraintGroupNode**												mTmpConstraintGroupRootBuffer;  // temporary list of constraint group roots, used for constraint projection

					PxCoalescedHashSet<const BodySim*>									mPosePreviewBodies;  // list of bodies that requested early report of the integrated pose (eENABLE_POSE_INTEGRATION_PREVIEW).

					PxArray<PxsContactManager*>											mPreallocatedContactManagers;
					PxArray<ShapeInteraction*>											mPreallocatedShapeInteractions;
					PxArray<ElementInteractionMarker*>									mPreallocatedInteractionMarkers;

					OverlapFilterTask*														mOverlapFilterTaskHead;
					PxArray<PxFilterInfo>													mFilterInfo;
					PxBitMap																mSpeculativeCCDRigidBodyBitMap;
					PxBitMap																mSpeculativeCDDArticulationBitMap;

					bool																	mIsCollisionPhaseActive;
					// Set to true as long as collision phase is active (used as an indicator that it is OK to read object pose, 
					// velocity etc. compared to the solver phase where these properties might get written to).

				public:
					// For OmniPVD. To notify NpScene that actor's sleeping state has changed.
					typedef void(*SleepingStateChangedCallback)(PxActor*, bool);
					SleepingStateChangedCallback mOnSleepingStateChanged;
	};

	bool	activateInteraction(Interaction* interaction, void* data);
	void	activateInteractions(Sc::ActorSim& actorSim);
	void	deactivateInteractions(Sc::ActorSim& actorSim);

} // namespace Sc

}

#endif
