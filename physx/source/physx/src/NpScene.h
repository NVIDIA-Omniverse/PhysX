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
#include "NpDirectGPUAPI.h"

#if PX_SUPPORT_PVD
	#include "PxPhysics.h"
	#include "NpPvdSceneClient.h"
#endif

#if PX_SUPPORT_OMNI_PVD
	#include "omnipvd/OmniPvdPxSampler.h"
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
class NpArticulationMimicJoint;
class NpShapeManager;
class NpBatchQuery;
class NpActor;
class NpShape;
class NpPhysics;

#if PX_SUPPORT_GPU_PHYSX
class NpDeformableSurface;
class NpDeformableVolume;
class NpPBDParticleSystem;
class NpDeformableSurfaceMaterial;
class NpDeformableVolumeMaterial;
class NpPBDMaterial;
#endif

struct NpInternalAttachmentType
{
	enum Enum
	{
		eUNDEFINED,
		eSURFACE_TYPE = 0x10000000,
		eSURFACE_TRI_RIGID_BODY,
		eSURFACE_TRI_GLOBAL_POSE,
		eSURFACE_TRI_SURFACE_TRI,
		eSURFACE_TRI_SURFACE_VTX,	// Not Implemented
		eSURFACE_VTX_RIGID_BODY,
		eSURFACE_VTX_GLOBAL_POSE,
		eSURFACE_VTX_SURFACE_VTX,	// Not Implemented
		eVOLUME_TYPE = 0x20000000,
		eVOLUME_TET_RIGID_BODY,
		eVOLUME_TET_GLOBAL_POSE,
		eVOLUME_TET_VOLUME_TET,
		eVOLUME_TET_VOLUME_VTX,		// Not Implemented
		eVOLUME_TET_SURFACE_TRI,
		eVOLUME_TET_SURFACE_VTX,	// Not Implemented
		eVOLUME_VTX_RIGID_BODY,
		eVOLUME_VTX_GLOBAL_POSE,
		eVOLUME_VTX_VOLUME_VTX,		// Not Implemented
		eVOLUME_VTX_SURFACE_VTX,	// Not Implemented
	};
};

// returns an error if scene state is corrupted due to GPU errors.
#define NP_CHECK_SCENE_CORRUPTION if(mCorruptedState) { return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Scene state is corrupted. Simulation cannot continue!"); }	

class NpScene : public NpSceneAccessor, public PxUserAllocated
{
	//virtual interfaces:

	PX_NOCOPY(NpScene)
	public:

	virtual			void							release()	PX_OVERRIDE PX_FINAL;

	virtual			void							setFlag(PxSceneFlag::Enum flag, bool value)	PX_OVERRIDE PX_FINAL;
	virtual			PxSceneFlags					getFlags() const							PX_OVERRIDE PX_FINAL;

	virtual			void							setName(const char* name)	PX_OVERRIDE PX_FINAL;
	virtual			const char*						getName() const				PX_OVERRIDE PX_FINAL;

	// implement PxScene:

	virtual			void							setGravity(const PxVec3&)	PX_OVERRIDE PX_FINAL;
	virtual			PxVec3							getGravity() const			PX_OVERRIDE PX_FINAL;

	virtual			void							setBounceThresholdVelocity(const PxReal t)		PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getBounceThresholdVelocity() const				PX_OVERRIDE PX_FINAL;
	virtual			void							setMaxBiasCoefficient(const PxReal t)			PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getMaxBiasCoefficient() const					PX_OVERRIDE PX_FINAL;
	virtual			void							setFrictionOffsetThreshold(const PxReal t)		PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getFrictionOffsetThreshold() const				PX_OVERRIDE PX_FINAL;
	virtual			void							setFrictionCorrelationDistance(const PxReal t)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getFrictionCorrelationDistance() const			PX_OVERRIDE PX_FINAL;

	virtual			void							setLimits(const PxSceneLimits& limits)	PX_OVERRIDE PX_FINAL;
	virtual			PxSceneLimits					getLimits() const						PX_OVERRIDE PX_FINAL;

	virtual			bool							addActor(PxActor& actor, const PxBVH* bvh)	PX_OVERRIDE PX_FINAL;
	virtual			void							removeActor(PxActor& actor, bool wakeOnLostTouch)	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getNbConstraints() const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getConstraints(PxConstraint** buffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE PX_FINAL;

	virtual			bool							addArticulation(PxArticulationReducedCoordinate&)	PX_OVERRIDE PX_FINAL;
	virtual			void							removeArticulation(PxArticulationReducedCoordinate&, bool wakeOnLostTouch)	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getNbArticulations() const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getArticulations(PxArticulationReducedCoordinate** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getNbDeformableSurfaces() const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getDeformableSurfaces(PxDeformableSurface** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getNbDeformableVolumes() const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getDeformableVolumes(PxDeformableVolume** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getNbPBDParticleSystems() const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getPBDParticleSystems(PxPBDParticleSystem** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getNbParticleSystems(PxParticleSolverType::Enum type) const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getParticleSystems(PxParticleSolverType::Enum type, PxPBDParticleSystem** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE PX_FINAL;

	// Aggregates
    virtual			bool							addAggregate(PxAggregate&)	PX_OVERRIDE PX_FINAL;
	virtual			void							removeAggregate(PxAggregate&, bool wakeOnLostTouch)	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getNbAggregates()	const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const	PX_OVERRIDE PX_FINAL;
	
	virtual			bool							addCollection(const PxCollection& collection)	PX_OVERRIDE PX_FINAL;

	// Groups
	virtual			void							setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)	PX_OVERRIDE PX_FINAL;
	virtual			PxDominanceGroupPair			getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const	PX_OVERRIDE PX_FINAL;

	// Actors
	virtual			PxU32							getNbActors(PxActorTypeFlags types) const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getActors(PxActorTypeFlags types, PxActor** buffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE PX_FINAL;
	virtual			PxActor**						getActiveActors(PxU32& nbActorsOut)	PX_OVERRIDE PX_FINAL;

	// Run
	virtual			void							getSimulationStatistics(PxSimulationStatistics& s) const	PX_OVERRIDE PX_FINAL;
	virtual			PxSceneResidual					getSolverResidual() const PX_OVERRIDE PX_FINAL { return mScene.getSolverResidual(); }

	// Multiclient 
	virtual			PxClientID						createClient()	PX_OVERRIDE PX_FINAL;

	// FrictionModel
	virtual			PxFrictionType::Enum			getFrictionType() const	PX_OVERRIDE PX_FINAL;

	// Callbacks
	virtual			void							setSimulationEventCallback(PxSimulationEventCallback* callback)		PX_OVERRIDE PX_FINAL;
	virtual			PxSimulationEventCallback*		getSimulationEventCallback()	const								PX_OVERRIDE PX_FINAL;
	virtual			void							setContactModifyCallback(PxContactModifyCallback* callback)			PX_OVERRIDE PX_FINAL;
	virtual			PxContactModifyCallback*		getContactModifyCallback()	const									PX_OVERRIDE PX_FINAL;
	virtual			void							setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)	PX_OVERRIDE PX_FINAL;
	virtual			PxCCDContactModifyCallback*		getCCDContactModifyCallback()	const								PX_OVERRIDE PX_FINAL;
	virtual			void							setBroadPhaseCallback(PxBroadPhaseCallback* callback)				PX_OVERRIDE PX_FINAL;
	virtual			PxBroadPhaseCallback*			getBroadPhaseCallback()		const									PX_OVERRIDE PX_FINAL;

	//CCD
	virtual			void							setCCDMaxPasses(PxU32 ccdMaxPasses)	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getCCDMaxPasses()	const			PX_OVERRIDE PX_FINAL;
	virtual			void							setCCDMaxSeparation(const PxReal t)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getCCDMaxSeparation() const			PX_OVERRIDE PX_FINAL;
	virtual			void							setCCDThreshold(const PxReal t)		PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getCCDThreshold() const				PX_OVERRIDE PX_FINAL;

	// Collision filtering
	virtual			void							setFilterShaderData(const void* data, PxU32 dataSize)							PX_OVERRIDE PX_FINAL;
	virtual			const void*						getFilterShaderData() const														PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getFilterShaderDataSize() const													PX_OVERRIDE PX_FINAL;
	virtual			PxSimulationFilterShader		getFilterShader() const															PX_OVERRIDE PX_FINAL;
	virtual			PxSimulationFilterCallback*		getFilterCallback() const														PX_OVERRIDE PX_FINAL;
	virtual			bool							resetFiltering(PxActor& actor)													PX_OVERRIDE PX_FINAL;
	virtual			bool							resetFiltering(PxRigidActor& actor, PxShape*const* shapes, PxU32 shapeCount)	PX_OVERRIDE PX_FINAL;
	virtual			PxPairFilteringMode::Enum		getKinematicKinematicFilteringMode()	const									PX_OVERRIDE PX_FINAL;
	virtual			PxPairFilteringMode::Enum		getStaticKinematicFilteringMode()		const									PX_OVERRIDE PX_FINAL;

	// Get Physics SDK
	virtual			PxPhysics&						getPhysics()	PX_OVERRIDE PX_FINAL;

	// new API methods
	virtual			bool							simulate(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation)	PX_OVERRIDE PX_FINAL;
	virtual			bool							advance(physx::PxBaseTask* completionTask)	PX_OVERRIDE PX_FINAL;
	virtual			bool							collide(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation = true)	PX_OVERRIDE PX_FINAL;
	virtual			bool							checkResults(bool block)	PX_OVERRIDE PX_FINAL;
	virtual			bool							fetchCollision(bool block)	PX_OVERRIDE PX_FINAL;
	virtual			bool							fetchResults(bool block, PxU32* errorState)	PX_OVERRIDE PX_FINAL;
	virtual			bool							fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block = false)	PX_OVERRIDE PX_FINAL;
	virtual			void							processCallbacks(physx::PxBaseTask* continuation)	PX_OVERRIDE PX_FINAL;
	virtual			void							fetchResultsFinish(PxU32* errorState = 0)	PX_OVERRIDE PX_FINAL;

	virtual			void							flushSimulation(bool sendPendingReports)	PX_OVERRIDE PX_FINAL;
	virtual			const PxRenderBuffer&			getRenderBuffer()	PX_OVERRIDE PX_FINAL;

	virtual			void							setSolverBatchSize(PxU32 solverBatchSize)	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getSolverBatchSize() const					PX_OVERRIDE PX_FINAL;

	virtual			void							setSolverArticulationBatchSize(PxU32 solverBatchSize)	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getSolverArticulationBatchSize() const					PX_OVERRIDE PX_FINAL;

	virtual			bool							setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal							getVisualizationParameter(PxVisualizationParameter::Enum param) const			PX_OVERRIDE PX_FINAL;

	virtual			void							setVisualizationCullingBox(const PxBounds3& box)	PX_OVERRIDE PX_FINAL;
	virtual			PxBounds3						getVisualizationCullingBox() const					PX_OVERRIDE PX_FINAL;

	virtual			PxTaskManager*					getTaskManager()	const	PX_OVERRIDE PX_FINAL	{ return mTaskManager; }

	virtual         void							setNbContactDataBlocks(PxU32 numBlocks)	PX_OVERRIDE PX_FINAL;
	virtual         PxU32							getNbContactDataBlocksUsed() const	PX_OVERRIDE PX_FINAL;
	virtual         PxU32							getMaxNbContactDataBlocksUsed() const	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getContactReportStreamBufferSize() const	PX_OVERRIDE PX_FINAL;

	virtual			PxU32							getTimestamp()	const	PX_OVERRIDE PX_FINAL;

	virtual			PxCpuDispatcher*				getCpuDispatcher() const	PX_OVERRIDE PX_FINAL;
	virtual			PxCudaContextManager*			getCudaContextManager() const	PX_OVERRIDE PX_FINAL	{ return mCudaContextManager;	}

	virtual			PxBroadPhaseType::Enum			getBroadPhaseType()									const	PX_OVERRIDE PX_FINAL;
	virtual			bool							getBroadPhaseCaps(PxBroadPhaseCaps& caps)			const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getNbBroadPhaseRegions()							const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion)	PX_OVERRIDE PX_FINAL;
	virtual			bool							removeBroadPhaseRegion(PxU32 handle)	PX_OVERRIDE PX_FINAL;

	virtual			bool							addActors(PxActor*const* actors, PxU32 nbActors)	PX_OVERRIDE PX_FINAL;
	virtual			bool							addActors(const PxPruningStructure& prunerStructure)	PX_OVERRIDE PX_FINAL;
	virtual			void							removeActors(PxActor*const* actors, PxU32 nbActors, bool wakeOnLostTouch)	PX_OVERRIDE PX_FINAL;

	virtual			void							lockRead(const char* file=NULL, PxU32 line=0)	PX_OVERRIDE PX_FINAL;
	virtual			void							unlockRead()	PX_OVERRIDE PX_FINAL;

	virtual			void							lockWrite(const char* file=NULL, PxU32 line=0)	PX_OVERRIDE PX_FINAL;
	virtual			void							unlockWrite()	PX_OVERRIDE PX_FINAL;

	virtual			PxReal							getWakeCounterResetValue() const	PX_OVERRIDE PX_FINAL;

	virtual			void							shiftOrigin(const PxVec3& shift)	PX_OVERRIDE PX_FINAL;

	virtual         PxPvdSceneClient*				getScenePvdClient()	PX_OVERRIDE PX_FINAL;

	PX_DEPRECATED	virtual	void					copyArticulationData(void* data, void* index, PxArticulationGpuDataType::Enum dataType, const PxU32 nbCopyArticulations, CUevent copyEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					applyArticulationData(void* data, void* index, PxArticulationGpuDataType::Enum dataType, const PxU32 nbUpdatedArticulations, CUevent waitEvent, CUevent signalEvent)	PX_OVERRIDE	PX_FINAL;

	PX_DEPRECATED	virtual	void					updateArticulationsKinematic(CUevent signalEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					copyContactData(void* data, const PxU32 numContactPatches, void* numContactPairs, CUevent copyEvent)	PX_OVERRIDE	PX_FINAL;
	
	PX_DEPRECATED	virtual	void					copySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, CUevent copyEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					applySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, CUevent applyEvent, CUevent signalEvent)	PX_OVERRIDE	PX_FINAL;

	PX_DEPRECATED	virtual	void					copyBodyData(PxGpuBodyData* data, PxGpuActorPair* index, const PxU32 nbCopyActors, CUevent copyEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					applyActorData(void* data, PxGpuActorPair* index, PxActorCacheFlag::Enum flag, const PxU32 nbUpdatedActors, CUevent waitEvent, CUevent signalEvent)	PX_OVERRIDE	PX_FINAL;

	PX_DEPRECATED	virtual	void					evaluateSDFDistances(const PxU32* sdfShapeIds, const PxU32 nbShapes, const PxVec4* samplePointsConcatenated, const PxU32* samplePointCountPerShape, const PxU32 maxPointCount, PxVec4* localGradientAndSDFConcatenated, CUevent event)	PX_OVERRIDE	PX_FINAL;

	PX_DEPRECATED	virtual	void					computeDenseJacobians(const PxIndexDataPair* indices, PxU32 nbIndices, CUevent computeEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					computeGeneralizedMassMatrices(const PxIndexDataPair* indices, PxU32 nbIndices, CUevent computeEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					computeGeneralizedGravityForces(const PxIndexDataPair* indices, PxU32 nbIndices, CUevent computeEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					computeCoriolisAndCentrifugalForces(const PxIndexDataPair* indices, PxU32 nbIndices, CUevent computeEvent)	PX_OVERRIDE	PX_FINAL;
	PX_DEPRECATED	virtual	void					applyParticleBufferData(const PxU32* indices, const PxGpuParticleBufferIndexPair* bufferIndexPairs, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, CUevent waitEvent, CUevent signalEvent)	PX_OVERRIDE	PX_FINAL;

	virtual	void									setDeformableSurfaceGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback)	PX_OVERRIDE	PX_FINAL;
	virtual	void									setDeformableVolumeGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback)	PX_OVERRIDE	PX_FINAL;

	virtual			PxSolverType::Enum				getSolverType()	const	PX_OVERRIDE PX_FINAL;

	virtual 		PxDirectGPUAPI&					getDirectGPUAPI()	PX_OVERRIDE	PX_FINAL;

	// NpSceneAccessor
	virtual			PxsSimulationController*		getSimulationController()	PX_OVERRIDE PX_FINAL;
	virtual			void							setActiveActors(PxActor** actors, PxU32 nbActors)	PX_OVERRIDE PX_FINAL;
	virtual			PxActor**						getFrozenActors(PxU32& nbActorsOut)	PX_OVERRIDE PX_FINAL;
	virtual			void							setFrozenActorFlag(const bool buildFrozenActors)	PX_OVERRIDE PX_FINAL;
	virtual			void							forceSceneQueryRebuild()	PX_OVERRIDE PX_FINAL;
	virtual			void							frameEnd()	PX_OVERRIDE PX_FINAL;
	//~NpSceneAccessor

	// PxSceneQuerySystemBase
	virtual			void							setDynamicTreeRebuildRateHint(PxU32 dynamicTreeRebuildRateHint)	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getDynamicTreeRebuildRateHint() const	PX_OVERRIDE PX_FINAL;
	virtual			void							forceRebuildDynamicTree(PxU32 prunerIndex)	PX_OVERRIDE PX_FINAL;
	virtual			void							setUpdateMode(PxSceneQueryUpdateMode::Enum updateMode)	PX_OVERRIDE PX_FINAL;
	virtual			PxSceneQueryUpdateMode::Enum	getUpdateMode() const	PX_OVERRIDE PX_FINAL;
	virtual			PxU32							getStaticTimestamp()	const	PX_OVERRIDE PX_FINAL;
	virtual			void							flushUpdates()	PX_OVERRIDE PX_FINAL;
	virtual			bool							raycast(
														const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxRaycastCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags) const	PX_OVERRIDE PX_FINAL;

	virtual			bool							sweep(
														const PxGeometry& geometry, const PxTransform& pose,	// GeomObject data
														const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxSweepCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const	PX_OVERRIDE PX_FINAL;

	virtual			bool							overlap(
														const PxGeometry& geometry, const PxTransform& transform,	// GeomObject data
														PxOverlapCallback& hitCall, 
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags) const	PX_OVERRIDE PX_FINAL;
	//~PxSceneQuerySystemBase

	// PxSceneSQSystem
	virtual			PxPruningStructureType::Enum	getStaticStructure()	const	PX_OVERRIDE PX_FINAL;
	virtual			PxPruningStructureType::Enum	getDynamicStructure()	const	PX_OVERRIDE PX_FINAL;
	virtual			void							sceneQueriesUpdate(physx::PxBaseTask* completionTask, bool controlSimulation)	PX_OVERRIDE PX_FINAL;
	virtual			bool							checkQueries(bool block)	PX_OVERRIDE PX_FINAL;
	virtual			bool							fetchQueries(bool block)	PX_OVERRIDE PX_FINAL;
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
					void							removeArticulationInternal(PxArticulationReducedCoordinate&, bool wakeOnLostTouch, bool removeFromAggregate);
	// materials
					void							addMaterial(const NpMaterial& mat);
					void							updateMaterial(const NpMaterial& mat);
					void							removeMaterial(const NpMaterial& mat);
#if PX_SUPPORT_GPU_PHYSX
					void							addMaterial(const NpDeformableSurfaceMaterial& mat);
					void							updateMaterial(const NpDeformableSurfaceMaterial& mat);
					void							removeMaterial(const NpDeformableSurfaceMaterial& mat);

					void							addMaterial(const NpDeformableVolumeMaterial& mat);
					void							updateMaterial(const NpDeformableVolumeMaterial& mat);
					void							removeMaterial(const NpDeformableVolumeMaterial& mat);

					void							addMaterial(const NpPBDMaterial& mat);
					void							updateMaterial(const NpPBDMaterial& mat);
					void							removeMaterial(const NpPBDMaterial& mat);
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
	PX_FORCE_INLINE	void							removeFromDeformableSurfaceList(PxDeformableSurface&);
	PX_FORCE_INLINE	void							removeFromDeformableVolumeList(PxDeformableVolume&);
	PX_FORCE_INLINE	void							removeFromParticleSystemList(PxPBDParticleSystem&);
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
					void							removeArticulationMimicJoints(PxArticulationReducedCoordinate& articulation);

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

#if PX_SUPPORT_OMNI_PVD
	PX_FORCE_INLINE	NpOmniPvdSceneClient&			getSceneOvdClientInternal()					{ return mSceneOvdClient;			}
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

	PX_FORCE_INLINE	PxSceneFlags					getFlagsFast()						const	{ return mScene.getFlags();			}
	PX_FORCE_INLINE PxReal							getWakeCounterResetValueInternal()	const	{ return mWakeCounterResetValue;	}

	PX_FORCE_INLINE bool							isDirectGPUAPIInitialized()				 	{ return mScene.isDirectGPUAPIInitialized(); }

	PX_FORCE_INLINE PxReal							getElapsedTime()					const	{ return mElapsedTime;				}

					// PT: TODO: consider merging the "sc" methods with the np ones, as we did for constraints

					void 							scAddActor(NpRigidStatic&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVH* bvh);
					void 							scRemoveActor(NpRigidStatic&, bool wakeOnLostTouch, bool noSim);
					void 							scAddActor(NpRigidDynamic&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVH* bvh);
					void 							scRemoveActor(NpRigidDynamic&, bool wakeOnLostTouch, bool noSim);
					void 							scAddActor(NpArticulationLink&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVH* bvh);
					void 							scRemoveActor(NpArticulationLink&, bool wakeOnLostTouch, bool noSim);

#if PX_SUPPORT_GPU_PHYSX
					void							scAddDeformableSurface(NpScene* npScene, NpDeformableSurface&);
					void							scRemoveDeformableSurface(NpDeformableSurface&);

					void							scAddDeformableVolume(NpDeformableVolume&);
					void							scRemoveDeformableVolume(NpDeformableVolume&);

					void							scAddParticleSystem(NpPBDParticleSystem&);
					void							scRemoveParticleSystem(NpPBDParticleSystem&);

					void							addToAttachmentList(PxDeformableAttachment&);
					void							removeFromAttachmentList(PxDeformableAttachment&);

					void							addToElementFilterList(PxDeformableElementFilter&);
					void							removeFromElementFilterList(PxDeformableElementFilter&);
#endif
					void							scAddArticulation(NpArticulationReducedCoordinate&);
					void							scRemoveArticulation(NpArticulationReducedCoordinate&);

					void							scAddArticulationJoint(NpArticulationJointReducedCoordinate&);
					void							scRemoveArticulationJoint(NpArticulationJointReducedCoordinate&);

					void							scAddArticulationSpatialTendon(NpArticulationSpatialTendon&);
					void							scRemoveArticulationSpatialTendon(NpArticulationSpatialTendon&);

					void							scAddArticulationFixedTendon(NpArticulationFixedTendon&);
					void							scRemoveArticulationFixedTendon(NpArticulationFixedTendon&);

					void							scAddArticulationMimicJoint(NpArticulationMimicJoint&);
					void							scRemoveArticulationMimicJoint(NpArticulationMimicJoint&);

#if PX_SUPPORT_OMNI_PVD
					void							createInOmniPVD(const PxSceneDesc& desc);
#endif

	PX_FORCE_INLINE	void							updatePvdProperties()
													{
#if PX_SUPPORT_PVD
														// PT: TODO: shouldn't we test PxPvdInstrumentationFlag::eDEBUG here?						
														if(mScenePvdClient.isConnected())
															mScenePvdClient.updatePvdProperties(); 
#endif
													}

					void							updateConstants(const PxArray<NpConstraint*>& constraints);

	virtual			PxGpuDynamicsMemoryConfig		getGpuDynamicsConfig() const	PX_OVERRIDE PX_FINAL	{ return mGpuDynamicsConfig; }

private:
					bool							checkResultsInternal(bool block);
					bool							checkCollisionInternal(bool block);
					bool							checkSceneQueriesInternal(bool block);
					bool							simulateOrCollide(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation, const char* invalidCallMsg, Sc::SimulationStage::Enum simStage);
					bool 							checkSceneStateAndCudaErrors(bool isCollide = false);
					bool							checkGpuErrorsPreSim(bool isCollide = false);

					bool							addRigidStatic(NpRigidStatic& , const Gu::BVH* bvh, const Sq::PruningStructure* ps = NULL);
					void							removeRigidStatic(NpRigidStatic&, bool wakeOnLostTouch, bool removeFromAggregate);
					bool							addRigidDynamic(NpRigidDynamic& , const Gu::BVH* bvh, const Sq::PruningStructure* ps = NULL);
					void							removeRigidDynamic(NpRigidDynamic&, bool wakeOnLostTouch, bool removeFromAggregate);

					bool							addDeformableSurface(PxDeformableSurface&);
					void							removeDeformableSurface(PxDeformableSurface&, bool wakeOnLostTouch);

					bool							addDeformableVolume(PxDeformableVolume&);
					void							removeDeformableVolume(PxDeformableVolume&, bool wakeOnLostTouch);

					bool							addParticleSystem(PxPBDParticleSystem& particleSystem);
					void							removeParticleSystem(PxPBDParticleSystem& particleSystem, bool wakeOnLostTouch);

					void							visualize();

					void							updateDirtyShaders();

					void							fetchResultsPreContactCallbacks();
					void							fetchResultsPostContactCallbacks();
					void							fetchResultsParticleSystem();

					bool							addSpatialTendonInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim);
					bool							addFixedTendonInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim);

					bool							addArticulationMimicJointInternal(NpArticulationReducedCoordinate* npaRC, Sc::ArticulationSim* scArtSim);

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
					struct Acceleration
					{
						PX_FORCE_INLINE	Acceleration() : mLinAccel(0.0f), mAngAccel(0.0f), mPrevLinVel(0.0f), mPrevAngVel(0.0f)
						{}
						PxVec3	mLinAccel;
						PxVec3	mAngAccel;
						PxVec3	mPrevLinVel;
						PxVec3	mPrevAngVel;
					};
	private:
					PxArray<NpRigidDynamic*>								mRigidDynamics;	// no hash set used because it would be quite a bit slower when adding a large number of actors
					PxArray<NpRigidStatic*>									mRigidStatics;	// no hash set used because it would be quite a bit slower when adding a large number of actors
					PxCoalescedHashSet<PxArticulationReducedCoordinate*>	mArticulations;
					PxCoalescedHashSet<PxDeformableSurface*>				mDeformableSurfaces;
					PxCoalescedHashSet<PxDeformableVolume*>					mDeformableVolumes;
					PxCoalescedHashSet<PxPBDParticleSystem*>				mPBDParticleSystems;
					PxCoalescedHashSet<PxAggregate*>						mAggregates;
	public:
					PxArray<Acceleration>									mRigidDynamicsAccelerations;
	private:
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
						virtual void release()	PX_OVERRIDE PX_FINAL
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
						virtual const char* getName() const	PX_OVERRIDE PX_FINAL	{ return "NpScene.completion"; }

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
					bool							mCorruptedState; // true in case we abort simulation.

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
					PxArray<MaterialEvent>		mSceneDeformableSurfaceMaterialBuffer;
					PxArray<MaterialEvent>		mSceneDeformableVolumeMaterialBuffer;
					PxArray<MaterialEvent>		mScenePBDMaterialBuffer;
					Sc::Scene					mScene;
					NpDirectGPUAPI*				mDirectGPUAPI;
#if PX_SUPPORT_PVD
					Vd::PvdSceneClient			mScenePvdClient;
#endif

#if PX_SUPPORT_OMNI_PVD
					NpOmniPvdSceneClient		mSceneOvdClient;
#endif
					const PxReal				mWakeCounterResetValue;

					PxGpuDynamicsMemoryConfig		mGpuDynamicsConfig;

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

PX_FORCE_INLINE	void NpScene::removeFromDeformableSurfaceList(PxDeformableSurface& deformableSurface)
{
	const bool exists = mDeformableSurfaces.erase(&deformableSurface);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE	void NpScene::removeFromDeformableVolumeList(PxDeformableVolume& deformableVolume)
{
	const bool exists = mDeformableVolumes.erase(&deformableVolume);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
}

PX_FORCE_INLINE	void NpScene::removeFromParticleSystemList(PxPBDParticleSystem& particleSystem)
{
	const bool exists = mPBDParticleSystems.erase(&particleSystem);
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

#endif // NP_SCENE_H
