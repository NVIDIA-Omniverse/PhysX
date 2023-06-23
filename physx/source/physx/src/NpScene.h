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

#ifndef NP_SCENE_H
#define NP_SCENE_H

#define NEW_DIRTY_SHADERS_CODE

#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxSync.h"
#include "foundation/PxArray.h"
#include "foundation/PxThread.h"
#include "PxPhysXConfig.h"

#include "CmRenderBuffer.h"
#include "CmIDPool.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "device/PhysXIndicator.h"
#endif

#include "NpSceneQueries.h"
#include "NpSceneAccessor.h"
#include "NpPruningStructure.h"

#if PX_SUPPORT_PVD
	#include "PxPhysics.h"
	#include "NpPvdSceneClient.h"
#endif

#include "ScScene.h"

namespace physx
{
namespace Sc
{
	class Joint;
	class ConstraintBreakEvent;
}

namespace Sq
{
	class PrunerManager;
}

class PhysicsThread;
class NpMaterial;
class NpScene;
class NpArticulationReducedCoordinate;
class NpAggregate;
class NpObjectFactory;
class NpRigidStatic;
class NpRigidDynamic;
class NpConstraint;
class NpArticulationLink;
class NpArticulationJointReducedCoordinate;
class NpArticulationAttachment;
class NpArticulationTendonJoint;
class NpArticulationSpatialTendon;
class NpArticulationFixedTendon;
class NpArticulationSensor;
class NpShapeManager;
class NpBatchQuery;
class NpActor;
class NpShape;
class NpPhysics;

#if PX_SUPPORT_GPU_PHYSX
class NpSoftBody;
class NpFEMCloth;
class NpHairSystem;

class NpPBDParticleSystem;
class NpFLIPParticleSystem;
class NpMPMParticleSystem;
class NpFEMSoftBodyMaterial;
class NpFEMClothMaterial;
class NpPBDMaterial;
class NpFLIPMaterial;
class NpMPMMaterial;
#endif

class NpContactCallbackTask : public physx::PxLightCpuTask
{
	NpScene*	mScene;
	const PxContactPairHeader* mContactPairHeaders;
	uint32_t mNbContactPairHeaders;

public:

	void setData(NpScene* scene, const PxContactPairHeader* contactPairHeaders, const uint32_t nbContactPairHeaders);

	virtual void run();

	virtual const char* getName() const
	{
		return "NpContactCallbackTask";
	}
};

class NpScene : public NpSceneAccessor, public PxUserAllocated
{
	//virtual interfaces:

	PX_NOCOPY(NpScene)
	public:

	virtual			void							release();

	virtual			void							setFlag(PxSceneFlag::Enum flag, bool value);
	virtual			PxSceneFlags					getFlags() const;

	virtual			void							setName(const char* name);
	virtual			const char*						getName() const;

	// implement PxScene:

	virtual			void							setGravity(const PxVec3&);
	virtual			PxVec3							getGravity() const;

	virtual			void							setBounceThresholdVelocity(const PxReal t);
	virtual			PxReal							getBounceThresholdVelocity() const;
	virtual			void							setMaxBiasCoefficient(const PxReal t);
	virtual			PxReal							getMaxBiasCoefficient() const;
	virtual			void							setFrictionOffsetThreshold(const PxReal t);
	virtual			PxReal							getFrictionOffsetThreshold() const;
	virtual			void							setFrictionCorrelationDistance(const PxReal t);
	virtual			PxReal							getFrictionCorrelationDistance() const;

	virtual			void							setLimits(const PxSceneLimits& limits);
	virtual			PxSceneLimits					getLimits() const;

	virtual			bool							addActor(PxActor& actor, const PxBVH* bvh);
	virtual			void							removeActor(PxActor& actor, bool wakeOnLostTouch);

	virtual			PxU32							getNbConstraints() const;
	virtual			PxU32							getConstraints(PxConstraint** buffer, PxU32 bufferSize, PxU32 startIndex=0) const;

	virtual			bool							addArticulation(PxArticulationReducedCoordinate&);
	virtual			void							removeArticulation(PxArticulationReducedCoordinate&, bool wakeOnLostTouch);

	virtual			PxU32							getNbArticulations() const;
	virtual			PxU32							getArticulations(PxArticulationReducedCoordinate** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

	virtual			PxU32							getNbSoftBodies() const;
	virtual			PxU32							getSoftBodies(PxSoftBody** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const;

	virtual			PxU32							getNbParticleSystems(PxParticleSolverType::Enum type) const;
	virtual			PxU32							getParticleSystems(PxParticleSolverType::Enum type, PxParticleSystem** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const;

	virtual			PxU32							getNbFEMCloths() const;
	virtual			PxU32							getFEMCloths(PxFEMCloth** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const;

	virtual			PxU32							getNbHairSystems() const;
	virtual			PxU32							getHairSystems(PxHairSystem** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const;

	// Aggregates
    virtual			bool							addAggregate(PxAggregate&);
	virtual			void							removeAggregate(PxAggregate&, bool wakeOnLostTouch);
	virtual			PxU32							getNbAggregates()	const;
	virtual			PxU32							getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const;
	
	virtual			bool							addCollection(const PxCollection& collection);

	// Groups
	virtual			void							setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance);
	virtual			PxDominanceGroupPair			getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const;

	// Actors
	virtual			PxU32							getNbActors(PxActorTypeFlags types) const;
	virtual			PxU32							getActors(PxActorTypeFlags types, PxActor** buffer, PxU32 bufferSize, PxU32 startIndex=0) const;
	virtual			PxActor**						getActiveActors(PxU32& nbActorsOut);

	// Run
	virtual			void							getSimulationStatistics(PxSimulationStatistics& s) const;

	// Multiclient 
	virtual			PxClientID						createClient();

	// FrictionModel
	virtual			PxFrictionType::Enum			getFrictionType() const;

	// Callbacks
	virtual			void							setSimulationEventCallback(PxSimulationEventCallback* callback);
	virtual			PxSimulationEventCallback*		getSimulationEventCallback()	const;
	virtual			void							setContactModifyCallback(PxContactModifyCallback* callback);
	virtual			PxContactModifyCallback*		getContactModifyCallback()	const;
	virtual			void							setCCDContactModifyCallback(PxCCDContactModifyCallback* callback);
	virtual			PxCCDContactModifyCallback*		getCCDContactModifyCallback()	const;
	virtual			void							setBroadPhaseCallback(PxBroadPhaseCallback* callback);
	virtual			PxBroadPhaseCallback*			getBroadPhaseCallback()		const;

	//CCD
	virtual			void							setCCDMaxPasses(PxU32 ccdMaxPasses);
	virtual			PxU32							getCCDMaxPasses()	const;
	virtual			void							setCCDMaxSeparation(const PxReal t);
	virtual			PxReal							getCCDMaxSeparation() const;
	virtual			void							setCCDThreshold(const PxReal t);
	virtual			PxReal							getCCDThreshold() const;

	// Collision filtering
	virtual			void							setFilterShaderData(const void* data, PxU32 dataSize);
	virtual			const void*						getFilterShaderData() const;
	virtual			PxU32							getFilterShaderDataSize() const;
	virtual			PxSimulationFilterShader		getFilterShader() const;
	virtual			PxSimulationFilterCallback*		getFilterCallback() const;
	virtual			bool							resetFiltering(PxActor& actor);
	virtual			bool							resetFiltering(PxRigidActor& actor, PxShape*const* shapes, PxU32 shapeCount);
	virtual			PxPairFilteringMode::Enum		getKinematicKinematicFilteringMode()	const;
	virtual			PxPairFilteringMode::Enum		getStaticKinematicFilteringMode()		const;

	// Get Physics SDK
	virtual			PxPhysics&						getPhysics();

	// new API methods
	virtual			bool							simulate(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation);
	virtual			bool							advance(physx::PxBaseTask* completionTask);
	virtual			bool							collide(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation = true);
	virtual			bool							checkResults(bool block);
	virtual			bool							checkCollision(bool block);
	virtual			bool							fetchCollision(bool block);
	virtual			bool							fetchResults(bool block, PxU32* errorState);
	virtual			bool							fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block = false);
	virtual			void							processCallbacks(physx::PxBaseTask* continuation);
	virtual			void							fetchResultsFinish(PxU32* errorState = 0);

	virtual			void							flush(bool sendPendingReports) { flushSimulation(sendPendingReports); }
	virtual			void							flushSimulation(bool sendPendingReports);
	virtual			const PxRenderBuffer&			getRenderBuffer();

	virtual			void							setSolverBatchSize(PxU32 solverBatchSize);
	virtual			PxU32							getSolverBatchSize(void) const;

	virtual			void							setSolverArticulationBatchSize(PxU32 solverBatchSize);
	virtual			PxU32							getSolverArticulationBatchSize(void) const;

	virtual			bool							setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value);
	virtual			PxReal							getVisualizationParameter(PxVisualizationParameter::Enum param) const;

	virtual			void							setVisualizationCullingBox(const PxBounds3& box);
	virtual			PxBounds3						getVisualizationCullingBox() const;

	virtual			PxTaskManager*					getTaskManager()	const	{ return mTaskManager; }
					void							checkBeginWrite()	const	{}

	virtual			PxCudaContextManager*			getCudaContextManager() { return mCudaContextManager; }
					
	virtual         void							setNbContactDataBlocks(PxU32 numBlocks);
	virtual         PxU32							getNbContactDataBlocksUsed() const;
	virtual         PxU32							getMaxNbContactDataBlocksUsed() const;

	virtual			PxU32							getContactReportStreamBufferSize() const;

	virtual			PxU32							getTimestamp()	const;

	virtual			PxCpuDispatcher*				getCpuDispatcher() const;
	virtual			PxCudaContextManager*			getCudaContextManager() const;


	virtual			PxBroadPhaseType::Enum			getBroadPhaseType()									const;
	virtual			bool							getBroadPhaseCaps(PxBroadPhaseCaps& caps)			const;
	virtual			PxU32							getNbBroadPhaseRegions()							const;
	virtual			PxU32							getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;
	virtual			PxU32							addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion);
	virtual			bool							removeBroadPhaseRegion(PxU32 handle);

	virtual			bool							addActors(PxActor*const* actors, PxU32 nbActors);
	virtual			bool							addActors(const PxPruningStructure& prunerStructure);
	virtual			void							removeActors(PxActor*const* actors, PxU32 nbActors, bool wakeOnLostTouch);

	virtual			void							lockRead(const char* file=NULL, PxU32 line=0);
	virtual			void							unlockRead();

	virtual			void							lockWrite(const char* file=NULL, PxU32 line=0);
	virtual			void							unlockWrite();

	virtual			PxReal							getWakeCounterResetValue() const;

	virtual			void							shiftOrigin(const PxVec3& shift);

	virtual         PxPvdSceneClient*				getScenePvdClient();

	virtual			void							copyArticulationData(void* data, void* index, PxArticulationGpuDataType::Enum dataType, const PxU32 nbCopyArticulations, void* copyEvent);
	virtual			void							applyArticulationData(void* data, void* index, PxArticulationGpuDataType::Enum dataType, const PxU32 nbUpdatedArticulations, void* waitEvent, void* signalEvent);
	virtual			void							copyContactData(void* data, const PxU32 numContactPatches, void* numContactPairs, void* copyEvent);
	
	virtual			void							copySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, void* copyEvent);
	virtual			void							applySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, void* applyEvent);

	virtual			void							copyBodyData(PxGpuBodyData* data, PxGpuActorPair* index, const PxU32 nbCopyActors, void* copyEvent);	
	virtual			void							applyActorData(void* data, PxGpuActorPair* index, PxActorCacheFlag::Enum flag, const PxU32 nbUpdatedActors, void* waitEvent, void* signalEvent);

	virtual			void							evaluateSDFDistances(const PxU32* sdfShapeIds, const PxU32 nbShapes, const PxVec4* samplePointsConcatenated, const PxU32* samplePointCountPerShape, const PxU32 maxPointCount, PxVec4* localGradientAndSDFConcatenated, void* event);

	virtual			void							computeDenseJacobians(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent);
	virtual			void							computeGeneralizedMassMatrices(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent);
	virtual			void							computeGeneralizedGravityForces(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent);
	virtual			void							computeCoriolisAndCentrifugalForces(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent);
	virtual			void							applyParticleBufferData(const PxU32* indices, const PxGpuParticleBufferIndexPair* bufferIndexPairs, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, void* waitEvent, void* signalEvent);

	virtual			PxSolverType::Enum				getSolverType()	const;

	// NpSceneAccessor
	virtual			PxsSimulationController*		getSimulationController();
	virtual			void							setActiveActors(PxActor** actors, PxU32 nbActors);
	virtual			PxActor**						getFrozenActors(PxU32& nbActorsOut);
	virtual			void							setFrozenActorFlag(const bool buildFrozenActors);
	virtual			void							forceSceneQueryRebuild();
	virtual			void							frameEnd();
	//~NpSceneAccessor

	// PxSceneQuerySystemBase
	virtual			void							setDynamicTreeRebuildRateHint(PxU32 dynamicTreeRebuildRateHint);
	virtual			PxU32							getDynamicTreeRebuildRateHint() const;
	virtual			void							forceRebuildDynamicTree(PxU32 prunerIndex);
	virtual			void							setUpdateMode(PxSceneQueryUpdateMode::Enum updateMode);
	virtual			PxSceneQueryUpdateMode::Enum	getUpdateMode() const;
	virtual			PxU32							getStaticTimestamp()	const;
	virtual			void							flushUpdates();
	virtual			bool							raycast(
														const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxRaycastCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags) const;

	virtual			bool							sweep(
														const PxGeometry& geometry, const PxTransform& pose,	// GeomObject data
														const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxSweepCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const;

	virtual			bool							overlap(
														const PxGeometry& geometry, const PxTransform& transform,	// GeomObject data
														PxOverlapCallback& hitCall, 
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags) const;
	//~PxSceneQuerySystemBase

	// PxSceneSQSystem
	virtual			PxPruningStructureType::Enum	getStaticStructure()	const;
	virtual			PxPruningStructureType::Enum	getDynamicStructure()	const;
	virtual			void							sceneQueriesUpdate(physx::PxBaseTask* completionTask, bool controlSimulation);
	virtual			bool							checkQueries(bool block);
	virtual			bool							fetchQueries(bool block);
	//~PxSceneSQSystem

	public:
													NpScene(const PxSceneDesc& desc, NpPhysics&);
													~NpScene();

	PX_FORCE_INLINE	NpSceneQueries&					getNpSQ()											{ return mNpSQ;							}
	PX_FORCE_INLINE	const NpSceneQueries&			getNpSQ()					const					{ return mNpSQ;							}

	PX_FORCE_INLINE	PxSceneQuerySystem&				getSQAPI()											{ return mNpSQ.getSQAPI();				}
	PX_FORCE_INLINE	const PxSceneQuerySystem&		getSQAPI()					const					{ return mNpSQ.getSQAPI();				}

	PX_FORCE_INLINE	PxU64							getContextId()				const					{ return PxU64(this);					}

	PX_FORCE_INLINE	PxTaskManager*					getTaskManagerFast()		const					{ return mTaskManager;					}

	PX_FORCE_INLINE Sc::SimulationStage::Enum		getSimulationStage()		const					{ return mScene.getSimulationStage();	}
	PX_FORCE_INLINE void							setSimulationStage(Sc::SimulationStage::Enum stage)	{ mScene.setSimulationStage(stage);		}

					bool							addActorInternal(PxActor& actor, const PxBVH* bvh);
					void							removeActorInternal(PxActor& actor, bool wakeOnLostTouch, bool removeFromAggregate);
					bool							addActorsInternal(PxActor*const* PX_RESTRICT actors, PxU32 nbActors, const Sq::PruningStructure* ps = NULL);

					bool							addArticulationInternal(PxArticulationReducedCoordinate&);
					void							removeArticulationInternal(PxArticulationReducedCoordinate&, bool wakeOnLostTouch,  bool removeFromAggregate);
	// materials
					void							addMaterial(const NpMaterial& mat);
					void							updateMaterial(const NpMaterial& mat);
					void							removeMaterial(const NpMaterial& mat);
#if PX_SUPPORT_GPU_PHYSX
					void							addMaterial(const NpFEMSoftBodyMaterial& mat);
					void							updateMaterial(const NpFEMSoftBodyMaterial& mat);
					void							removeMaterial(const NpFEMSoftBodyMaterial& mat);

					void							addMaterial(const NpFEMClothMaterial& mat);
					void							updateMaterial(const NpFEMClothMaterial& mat);
					void							removeMaterial(const NpFEMClothMaterial& mat);
					
					void							addMaterial(const NpPBDMaterial& mat);
					void							updateMaterial(const NpPBDMaterial& mat);
					void							removeMaterial(const NpPBDMaterial& mat);
					
					void							addMaterial(const NpFLIPMaterial& mat);
					void							updateMaterial(const NpFLIPMaterial& mat);
					void							removeMaterial(const NpFLIPMaterial& mat);
					
					void							addMaterial(const NpMPMMaterial& mat);
					void							updateMaterial(const NpMPMMaterial& mat);
					void							removeMaterial(const NpMPMMaterial& mat);
#endif

					void							executeScene(PxBaseTask* continuation);
					void							executeCollide(PxBaseTask* continuation);
					void							executeAdvance(PxBaseTask* continuation);
					void							constraintBreakEventNotify(PxConstraint *const *constraints, PxU32 count);

					bool							loadFromDesc(const PxSceneDesc&);

					template<typename T>
					void							removeFromRigidActorList(T&);
					void							removeFromRigidDynamicList(NpRigidDynamic&);
					void							removeFromRigidStaticList(NpRigidStatic&);
	PX_FORCE_INLINE	void							removeFromArticulationList(PxArticulationReducedCoordinate&);
	PX_FORCE_INLINE	void							removeFromSoftBodyList(PxSoftBody&);
	PX_FORCE_INLINE	void							removeFromFEMClothList(PxFEMCloth&);
	PX_FORCE_INLINE	void							removeFromParticleSystemList(PxPBDParticleSystem&);
	PX_FORCE_INLINE	void							removeFromParticleSystemList(PxFLIPParticleSystem&);
	PX_FORCE_INLINE	void							removeFromParticleSystemList(PxMPMParticleSystem&);
	PX_FORCE_INLINE	void							removeFromHairSystemList(PxHairSystem&);
	PX_FORCE_INLINE	void							removeFromAggregateList(PxAggregate&);

#ifdef NEW_DIRTY_SHADERS_CODE
					void							addDirtyConstraint(NpConstraint* constraint);
#endif
					void							addToConstraintList(PxConstraint&);
					void							removeFromConstraintList(PxConstraint&);

					void							addArticulationLink(NpArticulationLink& link);
					void							addArticulationLinkBody(NpArticulationLink& link);
					void							addArticulationLinkConstraint(NpArticulationLink& link);
					void							removeArticulationLink(NpArticulationLink& link, bool wakeOnLostTouch);

					void							addArticulationAttachment(NpArticulationAttachment& attachment);
					void							removeArticulationAttachment(NpArticulationAttachment& attachment);

					void							addArticulationTendonJoint(NpArticulationTendonJoint& joint);
					void							removeArticulationTendonJoint(NpArticulationTendonJoint& joint);

					void							removeArticulationTendons(PxArticulationReducedCoordinate& articulation);
					void							removeArticulationSensors(PxArticulationReducedCoordinate& articulation);

					struct StartWriteResult
					{
						enum Enum { eOK, eNO_LOCK, eIN_FETCHRESULTS, eRACE_DETECTED };
					};

					StartWriteResult::Enum			startWrite(bool allowReentry);
					void							stopWrite(bool allowReentry);

					bool							startRead() const;
					void							stopRead() const;

					PxU32							getReadWriteErrorCount() const { return PxU32(mConcurrentErrorCount); }

#if PX_CHECKED
					void							checkPositionSanity(const PxRigidActor& a, const PxTransform& pose, const char* fnName) const;
#endif

#if PX_SUPPORT_GPU_PHYSX
					void							updatePhysXIndicator();
#else
	PX_FORCE_INLINE	void							updatePhysXIndicator() {}
#endif

					void							scAddAggregate(NpAggregate&);
					void							scRemoveAggregate(NpAggregate&);

					void							scSwitchRigidToNoSim(NpActor&);
					void							scSwitchRigidFromNoSim(NpActor&);

#if PX_SUPPORT_PVD
	PX_FORCE_INLINE	Vd::PvdSceneClient&				getScenePvdClientInternal()					{ return mScenePvdClient;			}
	PX_FORCE_INLINE	const Vd::PvdSceneClient&		getScenePvdClientInternal()			const	{ return mScenePvdClient;			}
#endif
	PX_FORCE_INLINE bool							isAPIReadForbidden()				const	{ return mIsAPIReadForbidden;		}
	PX_FORCE_INLINE void							setAPIReadToForbidden()						{ mIsAPIReadForbidden = true;		}
	PX_FORCE_INLINE void							setAPIReadToAllowed()						{ mIsAPIReadForbidden = false;		}

	PX_FORCE_INLINE bool							isCollisionPhaseActive()			const	{ return mScene.isCollisionPhaseActive(); }

	PX_FORCE_INLINE bool							isAPIWriteForbidden()				const	{ return mIsAPIWriteForbidden;		}
	PX_FORCE_INLINE void							setAPIWriteToForbidden()					{ mIsAPIWriteForbidden = true;		}
	PX_FORCE_INLINE void							setAPIWriteToAllowed()						{ mIsAPIWriteForbidden = false;		}

	PX_FORCE_INLINE	const Sc::Scene&				getScScene()						const	{ return mScene;					}
	PX_FORCE_INLINE	Sc::Scene&						getScScene()								{ return mScene;					}

	PX_FORCE_INLINE	PxU32							getFlagsFast()						const	{ return mScene.getFlags();			}
	PX_FORCE_INLINE PxReal							getWakeCounterResetValueInternal()	const	{ return mWakeCounterResetValue;	}

					// PT: TODO: consider merging the "sc" methods with the np ones, as we did for constraints

					void 							scAddActor(NpRigidStatic&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVH* bvh);
					void 							scRemoveActor(NpRigidStatic&, bool wakeOnLostTouch, bool noSim);
					void 							scAddActor(NpRigidDynamic&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVH* bvh);
					void 							scRemoveActor(NpRigidDynamic&, bool wakeOnLostTouch, bool noSim);
					void 							scAddActor(NpArticulationLink&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVH* bvh);
					void 							scRemoveActor(NpArticulationLink&, bool wakeOnLostTouch, bool noSim);

#if PX_SUPPORT_GPU_PHYSX
					void							scAddSoftBody(NpSoftBody&);
					void							scRemoveSoftBody(NpSoftBody&);
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
					void							scAddFEMCloth(NpScene* npScene, NpFEMCloth&);
					void							scRemoveFEMCloth(NpFEMCloth&);
#endif
					void							scAddParticleSystem(NpPBDParticleSystem&);
					void							scRemoveParticleSystem(NpPBDParticleSystem&);
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
					void							scAddParticleSystem(NpFLIPParticleSystem&);
					void							scRemoveParticleSystem(NpFLIPParticleSystem&);

					void							scAddParticleSystem(NpMPMParticleSystem&);
					void							scRemoveParticleSystem(NpMPMParticleSystem&);
#endif
					void							scAddHairSystem(NpHairSystem&);
					void							scRemoveHairSystem(NpHairSystem&);
#endif
					void							scAddArticulation(NpArticulationReducedCoordinate&);
					void							scRemoveArticulation(NpArticulationReducedCoordinate&);

					void							scAddArticulationJoint(NpArticulationJointReducedCoordinate&);
					void							scRemoveArticulationJoint(NpArticulationJointReducedCoordinate&);

					void							scAddArticulationSpatialTendon(NpArticulationSpatialTendon&);
					void							scRemoveArticulationSpatialTendon(NpArticulationSpatialTendon&);

					void							scAddArticulationFixedTendon(NpArticulationFixedTendon&);
					void							scRemoveArticulationFixedTendon(NpArticulationFixedTendon&);

					void							scAddArticulationSensor(NpArticulationSensor&);
					void							scRemoveArticulationSensor(NpArticulationSensor&);

					void							createInOmniPVD(const PxSceneDesc& desc);
	PX_FORCE_INLINE	void							updatePvdProperties()
													{
#if PX_SUPPORT_PVD
														// PT: TODO: shouldn't we test PxPvdInstrumentationFlag::eDEBUG here?						
														if(mScenePvdClient.isConnected())
															mScenePvdClient.updatePvdProperties(); 
#endif
													}

					void							updateConstants(const PxArray<NpConstraint*>& constraints);

					virtual		PxgDynamicsMemoryConfig getGpuDynamicsConfig() const { return mGpuDynamicsConfig; }
private:
					bool							checkResultsInternal(bool block);
					bool							checkCollisionInternal(bool block);
					bool							checkSceneQueriesInternal(bool block);
					bool							simulateOrCollide(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation, const char* invalidCallMsg, Sc::SimulationStage::Enum simStage);

					bool							addRigidStatic(NpRigidStatic& , const Gu::BVH* bvh, const Sq::PruningStructure* ps = NULL);
					void							removeRigidStatic(NpRigidStatic&, bool wakeOnLostTouch, bool removeFromAggregate);
					bool							addRigidDynamic(NpRigidDynamic& , const Gu::BVH* bvh, const Sq::PruningStructure* ps = NULL);
					void							removeRigidDynamic(NpRigidDynamic&, bool wakeOnLostTouch, bool removeFromAggregate);

					bool							addSoftBody(PxSoftBody&);
					void							removeSoftBody(PxSoftBody&, bool wakeOnLostTouch);

					bool							addParticleSystem(PxParticleSystem& particleSystem);
					void							removeParticleSystem(PxParticleSystem& particleSystem, bool wakeOnLostTouch);
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
					bool							addFEMCloth(PxFEMCloth&);
					void							removeFEMCloth(PxFEMCloth&, bool wakeOnLostTouch);

					bool							addHairSystem(PxHairSystem&);
					void							removeHairSystem(PxHairSystem&, bool wakeOnLostTouch);
#endif
					void							visualize();

					void							updateDirtyShaders();

					void							fetchResultsPreContactCallbacks();
					void							fetchResultsPostContactCallbacks();
					void							fetchResultsParticleSystem();

					bool							addSpatialTendonInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim);
					bool							addFixedTendonInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim);

					bool							addArticulationSensorInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim);

					void							syncSQ();
					void							sceneQueriesStaticPrunerUpdate(PxBaseTask* continuation);
					void							sceneQueriesDynamicPrunerUpdate(PxBaseTask* continuation);

					void							syncMaterialEvents();

					NpSceneQueries					mNpSQ;
					PxPruningStructureType::Enum	mPrunerType[2];
					typedef Cm::DelegateTask<NpScene, &NpScene::sceneQueriesStaticPrunerUpdate> SceneQueriesStaticPrunerUpdate;
					typedef Cm::DelegateTask<NpScene, &NpScene::sceneQueriesDynamicPrunerUpdate> SceneQueriesDynamicPrunerUpdate;
					SceneQueriesStaticPrunerUpdate	mSceneQueriesStaticPrunerUpdate;
					SceneQueriesDynamicPrunerUpdate	mSceneQueriesDynamicPrunerUpdate;

					Cm::RenderBuffer				mRenderBuffer;
	public:
					Cm::IDPool						mRigidActorIndexPool;
	private:
					PxArray<NpRigidDynamic*>								mRigidDynamics;  // no hash set used because it would be quite a bit slower when adding a large number of actors
					PxArray<NpRigidStatic*>									mRigidStatics;  // no hash set used because it would be quite a bit slower when adding a large number of actors
					PxCoalescedHashSet<PxArticulationReducedCoordinate*>	mArticulations;
					PxCoalescedHashSet<PxSoftBody*>							mSoftBodies;
					PxCoalescedHashSet<PxFEMCloth*>							mFEMCloths;
					PxCoalescedHashSet<PxPBDParticleSystem*>				mPBDParticleSystems;
					PxCoalescedHashSet<PxFLIPParticleSystem*>				mFLIPParticleSystems;
					PxCoalescedHashSet<PxMPMParticleSystem*>				mMPMParticleSystems;
					PxCoalescedHashSet<PxHairSystem*>						mHairSystems;
					PxCoalescedHashSet<PxAggregate*>						mAggregates;

#ifdef NEW_DIRTY_SHADERS_CODE
					PxArray<NpConstraint*>									mAlwaysUpdatedConstraints;
					PxArray<NpConstraint*>									mDirtyConstraints;
					PxMutex													mDirtyConstraintsLock;
#endif

					PxBounds3						mSanityBounds;
#if PX_SUPPORT_GPU_PHYSX
					PhysXIndicator					mPhysXIndicator;
#endif

					PxSync							mPhysicsDone;		// physics thread signals this when update ready
					PxSync							mCollisionDone;		// physics thread signals this when all collisions ready
					PxSync							mSceneQueriesDone;	// physics thread signals this when all scene queries update ready

		//legacy timing settings:
					PxReal							mElapsedTime;		//needed to transfer the elapsed time param from the user to the sim thread.

					PxU32							mNbClients;		// Tracks reserved clients for multiclient support.

					struct SceneCompletion : public Cm::Task
					{
						SceneCompletion(PxU64 contextId, PxSync& sync) : Cm::Task(contextId), mSync(sync){}
						virtual void runInternal() {}
						//ML: As soon as mSync.set is called, and the scene is shutting down,
						//the scene may be deleted. That means this running task may also be deleted.
						//As such, we call mSync.set() inside release() to avoid a crash because the v-table on this
						//task might be deleted between the call to runInternal() and release() in the worker thread.
						virtual void release() 
						{ 
							//We cache the continuation pointer because this class may be deleted 
							//as soon as mSync.set() is called if the application releases the scene.
							PxBaseTask* c = mCont; 
							//once mSync.set(), fetchResults() will be allowed to run.
							mSync.set(); 
							//Call the continuation task that we cached above. If we use mCont or 
							//any other member variable of this class, there is a small chance
							//that the variables might have become corrupted if the class
							//was deleted.
							if(c) c->removeReference(); 
						}
						virtual const char* getName() const { return "NpScene.completion"; }

						//	//This method just is called in the split sim approach as a way to set continuation after the task has been initialized
						void setDependent(PxBaseTask* task){PX_ASSERT(mCont == NULL); mCont = task; if(task)task->addReference();}
						PxSync& mSync;
					private:
						SceneCompletion& operator=(const SceneCompletion&);
					};

					typedef Cm::DelegateTask<NpScene, &NpScene::executeScene> SceneExecution;
					typedef Cm::DelegateTask<NpScene, &NpScene::executeCollide> SceneCollide;
					typedef Cm::DelegateTask<NpScene, &NpScene::executeAdvance> SceneAdvance;
					
					PxTaskManager*					mTaskManager;
					PxCudaContextManager*			mCudaContextManager;
					SceneCompletion					mSceneCompletion;
					SceneCompletion					mCollisionCompletion;
					SceneCompletion					mSceneQueriesCompletion;
					SceneExecution					mSceneExecution;
					SceneCollide					mSceneCollide;
					SceneAdvance					mSceneAdvance;
					PxSQBuildStepHandle				mStaticBuildStepHandle;
					PxSQBuildStepHandle				mDynamicBuildStepHandle;
					bool                            mControllingSimulation;
					bool							mIsAPIReadForbidden;	// Set to true when the user is not allowed to call certain read APIs
																			// (properties that get written to during the simulation. Search the macros PX_CHECK_SCENE_API_READ_FORBIDDEN... 
																			// to see which calls)
					bool							mIsAPIWriteForbidden;  // Set to true when the user is not allowed to do API write calls

					PxU32							mSimThreadStackSize;

					volatile PxI32					mConcurrentWriteCount;
					mutable volatile PxI32			mConcurrentReadCount;					
					mutable volatile PxI32			mConcurrentErrorCount;

					// TLS slot index, keeps track of re-entry depth for this thread
					PxU32							mThreadReadWriteDepth;
					PxThread::Id					mCurrentWriter;
					PxReadWriteLock					mRWLock;

					bool							mSQUpdateRunning;

					bool							mBetweenFetchResults;
					bool							mBuildFrozenActors;

					//
	public:
					enum MATERIAL_EVENT
					{
						MATERIAL_ADD,
						MATERIAL_UPDATE,
						MATERIAL_REMOVE
					};

					class MaterialEvent
					{
					public:
						PX_FORCE_INLINE	MaterialEvent(PxU16 handle, MATERIAL_EVENT type) : mHandle(handle), mType(type)	{}
						PX_FORCE_INLINE	MaterialEvent()																	{}

						PxU16			mHandle;//handle to the master material table
						MATERIAL_EVENT	mType;
					};
	private:
					PxArray<MaterialEvent>		mSceneMaterialBuffer;
					PxArray<MaterialEvent>		mSceneFEMSoftBodyMaterialBuffer;
					PxArray<MaterialEvent>		mSceneFEMClothMaterialBuffer;
					PxArray<MaterialEvent>		mScenePBDMaterialBuffer;
					PxArray<MaterialEvent>		mSceneFLIPMaterialBuffer;
					PxArray<MaterialEvent>		mSceneMPMMaterialBuffer;
					PxMutex						mSceneMaterialBufferLock;
					PxMutex						mSceneFEMSoftBodyMaterialBufferLock;
					PxMutex						mSceneFEMClothMaterialBufferLock;
					PxMutex						mScenePBDMaterialBufferLock;
					PxMutex						mSceneFLIPMaterialBufferLock;
					PxMutex						mSceneMPMMaterialBufferLock;
					Sc::Scene					mScene;
#if PX_SUPPORT_PVD
					Vd::PvdSceneClient			mScenePvdClient;
#endif
					const PxReal				mWakeCounterResetValue;

					PxgDynamicsMemoryConfig		mGpuDynamicsConfig;

					NpPhysics&					mPhysics;
					const char*				    mName;
};

template<>
PX_FORCE_INLINE void NpScene::removeFromRigidActorList<NpRigidDynamic>(NpRigidDynamic& rigidDynamic)
{
	removeFromRigidDynamicList(rigidDynamic);
}

template<>
PX_FORCE_INLINE void NpScene::removeFromRigidActorList<NpRigidStatic>(NpRigidStatic& rigidStatic)
{
	removeFromRigidStaticList(rigidStatic);
}

PX_FORCE_INLINE void NpScene::removeFromArticulationList(PxArticulationReducedCoordinate& articulation)
{
	const bool exists = mArticulations.erase(&articulation);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE	void NpScene::removeFromSoftBodyList(PxSoftBody& softBody)
{
	const bool exists = mSoftBodies.erase(&softBody);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE	void NpScene::removeFromFEMClothList(PxFEMCloth& femCloth)
{
	const bool exists = mFEMCloths.erase(&femCloth);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE	void NpScene::removeFromParticleSystemList(PxPBDParticleSystem& particleSystem)
{
	const bool exists = mPBDParticleSystems.erase(&particleSystem);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PX_FORCE_INLINE	void NpScene::removeFromParticleSystemList(PxFLIPParticleSystem& particleSystem)
{
	const bool exists = mFLIPParticleSystems.erase(&particleSystem);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE	void NpScene::removeFromParticleSystemList(PxMPMParticleSystem& particleSystem)
{
	const bool exists = mMPMParticleSystems.erase(&particleSystem);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}
#endif

PX_FORCE_INLINE void NpScene::removeFromHairSystemList(PxHairSystem& hairSystem)
{
	const bool exists = mHairSystems.erase(&hairSystem);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE void NpScene::removeFromAggregateList(PxAggregate& aggregate)
{
	const bool exists = mAggregates.erase(&aggregate);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}


	PxU32 NpRigidStaticGetShapes(NpRigidStatic& rigid, NpShape* const *& shapes);
	PxU32 NpRigidDynamicGetShapes(NpRigidDynamic& actor, NpShape* const *& shapes, bool* isCompound = NULL);
	PxU32 NpArticulationGetShapes(NpArticulationLink& actor, NpShape* const *& shapes, bool* isCompound = NULL);

}

#endif
