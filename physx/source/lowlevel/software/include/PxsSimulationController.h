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

#ifndef PXS_SIMULATION_CONTROLLER_H
#define PXS_SIMULATION_CONTROLLER_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxTransform.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxPinnedArray.h"
#include "foundation/PxUserAllocated.h"
#include "PxScene.h"
#include "PxParticleSystem.h"

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	#include "PxFLIPParticleSystem.h"
	#include "PxMPMParticleSystem.h"

	// if these assert fail adjust the type here and in the forward declaration of the #else section just below
	PX_COMPILE_TIME_ASSERT(sizeof(physx::PxMPMParticleDataFlag::Enum) == sizeof(physx::PxU32));
	PX_COMPILE_TIME_ASSERT(sizeof(physx::PxSparseGridDataFlag::Enum) == sizeof(physx::PxU32));
#else
	namespace PxMPMParticleDataFlag { enum Enum : physx::PxU32; }
	namespace PxSparseGridDataFlag { enum Enum : physx::PxU32; }
#endif

#include "PxParticleSolverType.h"

namespace physx
{
	namespace Dy
	{
		class Context;
		struct Constraint;
		class FeatherstoneArticulation;
		struct ArticulationJointCore;
		class ParticleSystemCore;
		class ParticleSystem;
		
#if PX_SUPPORT_GPU_PHYSX
		class SoftBody;
		class FEMCloth;
		class HairSystem;
#endif
	}

	namespace Cm
	{
		class EventProfiler;
	}

	namespace Bp
	{
		class BoundsArray;
		class BroadPhase;
		class AABBManagerBase;
	}

	namespace IG
	{
		class SimpleIslandManager;
		class IslandSim;
	}

	namespace Sc
	{
		class BodySim;
	}

	class PxNodeIndex;
	class PxsTransformCache;
	class PxvNphaseImplementationContext;
	class PxBaseTask;
	class PxsContext;

	struct PxsShapeSim;
	class PxsRigidBody;
	class PxsKernelWranglerManager;
	class PxsHeapMemoryAllocatorManager;
	class PxgParticleSystemCore;
	struct PxConeLimitedConstraint;

	class PxPhysXGpu;

	struct PxgSolverConstraintManagerConstants;
	
	class PxsSimulationControllerCallback : public PxUserAllocated
	{
	public:
		virtual void updateScBodyAndShapeSim(PxBaseTask* continuation) = 0;
		virtual PxU32	getNbCcdBodies() = 0;

		virtual ~PxsSimulationControllerCallback() {}
	};


	class PxsSimulationController : public PxUserAllocated
	{
	public:
		PxsSimulationController(PxsSimulationControllerCallback* callback): mCallback(callback){}
		virtual ~PxsSimulationController(){}

		virtual void addJoint(const PxU32 edgeIndex, Dy::Constraint* constraint, IG::IslandSim& islandSim, PxArray<PxU32>& jointIndices, 
			PxPinnedArray<PxgSolverConstraintManagerConstants>& managerIter, PxU32 uniqueId) = 0;
		virtual void removeJoint(const PxU32 edgeIndex, Dy::Constraint* constraint, PxArray<PxU32>& jointIndices, IG::IslandSim& islandSim) = 0;
		virtual void addShape(PxsShapeSim* shapeSim, const PxU32 index) = 0;
		virtual void reinsertShape(PxsShapeSim* shapeSim, const PxU32 index) = 0;
		virtual void updateShape(PxsShapeSim& /*shapeSim*/, const PxNodeIndex& /*index*/) {}
		virtual void removeShape(const PxU32 index) = 0;

		virtual void addDynamic(PxsRigidBody* rigidBody, const PxNodeIndex& nodeIndex) = 0;
		virtual void addDynamics(PxsRigidBody** rigidBody, const PxU32* nodeIndex, PxU32 nbToProcess) = 0;
		virtual void addArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex) = 0;
		virtual void releaseArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex) = 0;
		virtual void releaseDeferredArticulationIds() = 0;

#if PX_SUPPORT_GPU_PHYSX
		virtual void addSoftBody(Dy::SoftBody* softBody, const PxNodeIndex& nodeIndex) = 0;
		virtual void releaseSoftBody(Dy::SoftBody* softBody) = 0;
		virtual void releaseDeferredSoftBodyIds() = 0;
		virtual void activateSoftbody(Dy::SoftBody*) = 0;
		virtual void deactivateSoftbody(Dy::SoftBody*) = 0;
		virtual void activateSoftbodySelfCollision(Dy::SoftBody*) = 0;
		virtual void deactivateSoftbodySelfCollision(Dy::SoftBody*) = 0;
		virtual void setSoftBodyWakeCounter(Dy::SoftBody*) = 0;

		virtual void addParticleFilter(Dy::SoftBody*softBodySystem, Dy::ParticleSystem* particleSystem,
			PxU32 particleId, PxU32 userBufferId, PxU32 tetId) = 0;
		virtual void removeParticleFilter(Dy::SoftBody* softBodySystem,
			const Dy::ParticleSystem* particleSystem, PxU32 particleId, PxU32 userBufferId, PxU32 tetId) = 0;

		virtual PxU32 addParticleAttachment(Dy::SoftBody*softBodySystem, const Dy::ParticleSystem* particleSystem,
			PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentrics, const bool isActive) = 0;
		virtual void removeParticleAttachment(Dy::SoftBody* softBody, PxU32 handle) = 0;

		virtual void addRigidFilter(Dy::SoftBody*softBodySystem, const PxNodeIndex& softBodyNodeIndex,
			const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex) = 0;
		virtual void removeRigidFilter(Dy::SoftBody* softBodySystem, 
			const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex) = 0;

		virtual PxU32 addRigidAttachment(Dy::SoftBody*softBodySystem, const PxNodeIndex& softBodyNodeIndex,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex, const PxVec3& actorSpacePose,
			PxConeLimitedConstraint* constraint, const bool isActive) = 0;
		virtual void removeRigidAttachment(Dy::SoftBody* softBody, PxU32 handle) = 0;

		virtual void addTetRigidFilter(Dy::SoftBody* softBodySystem,
			const PxNodeIndex& rigidNodeIndex, PxU32 tetId) = 0;

		virtual PxU32 addTetRigidAttachment(Dy::SoftBody* softBodySystem,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 tetIdx, 
			const PxVec4& barycentrics, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint,
			const bool isActive) = 0;

		virtual void removeTetRigidFilter(Dy::SoftBody* softBody, 
			const PxNodeIndex& rigidNodeIndex, PxU32 tetId) = 0;

		virtual void addSoftBodyFilter(Dy::SoftBody* softBody0, Dy::SoftBody* softBody1, PxU32 tetIdx0, 
			PxU32 tetIdx1) = 0;
		virtual void removeSoftBodyFilter(Dy::SoftBody* softBody0, Dy::SoftBody* softBody1, PxU32 tetIdx0,
			PxU32 tetId1) = 0;
		virtual void addSoftBodyFilters(Dy::SoftBody* softBody0, Dy::SoftBody* softBody1, PxU32* tetIndices0, PxU32* tetIndices1,
			PxU32 tetIndicesSize) = 0;
		virtual void removeSoftBodyFilters(Dy::SoftBody* softBody0, Dy::SoftBody* softBody1, PxU32* tetIndices0, PxU32* tetIndices1,
			PxU32 tetIndicesSize) = 0;

		virtual PxU32 addSoftBodyAttachment(Dy::SoftBody* softBody0, Dy::SoftBody* softBody1, PxU32 tetIdx0, PxU32 tetIdx1,
			const PxVec4& tetBarycentric0, const PxVec4& tetBarycentric1, PxConeLimitedConstraint* constraint, const bool isActive) = 0;
		virtual void removeSoftBodyAttachment(Dy::SoftBody* softBody0, PxU32 handle) = 0;

		virtual void addClothFilter(Dy::SoftBody* softBody, Dy::FEMCloth* cloth, PxU32 triIdx,
			PxU32 tetIdx) = 0;
		virtual void removeClothFilter(Dy::SoftBody* softBody, Dy::FEMCloth*, PxU32 triId,
			PxU32 tetId) = 0;

		virtual PxU32 addClothAttachment(Dy::SoftBody* softBody, Dy::FEMCloth* cloth, PxU32 triIdx,
			const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric, PxConeLimitedConstraint* constraint,
			const bool isActive) = 0;
		virtual void removeClothAttachment(Dy::SoftBody* softBody,PxU32 handle) = 0;

		virtual void addFEMCloth(Dy::FEMCloth* femCloth, const PxNodeIndex& nodeIndex) = 0;
		virtual void releaseFEMCloth(Dy::FEMCloth* femCloth) = 0;
		virtual void releaseDeferredFEMClothIds() = 0;
		virtual void activateCloth(Dy::FEMCloth* femCloth) = 0;
		virtual void deactivateCloth(Dy::FEMCloth* femCloth) = 0;
		virtual void setClothWakeCounter(Dy::FEMCloth*) = 0;

		virtual void addRigidFilter(Dy::FEMCloth* cloth,
			const PxNodeIndex& rigidNodeIndex, PxU32 vertId) = 0;

		virtual void removeRigidFilter(Dy::FEMCloth* cloth,
			const PxNodeIndex& rigidNodeIndex, PxU32 vertId) = 0;

		virtual PxU32 addRigidAttachment(Dy::FEMCloth* cloth, const PxNodeIndex& clothNodeIndex,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex, const PxVec3& actorSpacePose,
			PxConeLimitedConstraint* constraint, const bool isActive) = 0;
		virtual void removeRigidAttachment(Dy::FEMCloth* cloth, PxU32 handle) = 0;

		virtual void addTriRigidFilter(Dy::FEMCloth* cloth,
			const PxNodeIndex& rigidNodeIndex, PxU32 triIdx) = 0;

		virtual void removeTriRigidFilter(Dy::FEMCloth* cloth, 
			const PxNodeIndex& rigidNodeIndex, PxU32 triIdx) = 0;

		virtual PxU32 addTriRigidAttachment(Dy::FEMCloth* cloth,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 triIdx, const PxVec4& barycentrics, 
			const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint,
			const bool isActive) = 0;

		virtual void removeTriRigidAttachment(Dy::FEMCloth* cloth, PxU32 handle) = 0;

		virtual void addClothFilter(Dy::FEMCloth* cloth0, Dy::FEMCloth* cloth1, PxU32 triIdx0, PxU32 triIdx1) = 0;
		virtual void removeClothFilter(Dy::FEMCloth* cloth0, Dy::FEMCloth* cloth1, PxU32 triIdx0, PxU32 triId1) = 0;

		virtual PxU32 addTriClothAttachment(Dy::FEMCloth* cloth0, Dy::FEMCloth* cloth1, PxU32 triIdx0, PxU32 triIdx1,
			const PxVec4& triBarycentric0, const PxVec4& triBarycentric1, const bool addToActive) = 0;

		virtual void removeTriClothAttachment(Dy::FEMCloth* cloth0, PxU32 handle) = 0;


		virtual void addParticleSystem(Dy::ParticleSystem* particleSystem, const PxNodeIndex& nodeIndex, PxParticleSolverType::Enum type) = 0;
		virtual void releaseParticleSystem(Dy::ParticleSystem* particleSystem, PxParticleSolverType::Enum type) = 0;
		virtual void releaseDeferredParticleSystemIds() = 0;

		virtual void addHairSystem(Dy::HairSystem* hairSystem, const PxNodeIndex& nodeIndex) = 0;
		virtual void releaseHairSystem(Dy::HairSystem* hairSystem) = 0;
		virtual void releaseDeferredHairSystemIds() = 0;
		virtual void activateHairSystem(Dy::HairSystem*) = 0;
		virtual void deactivateHairSystem(Dy::HairSystem*) = 0;
		virtual void setHairSystemWakeCounter(Dy::HairSystem*) = 0;
#endif

		virtual void flush() = 0;

		virtual void updateDynamic(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex) = 0;
		virtual void updateJoint(const PxU32 edgeIndex, Dy::Constraint* constraint) = 0;
		virtual void updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies) = 0;
		virtual void updateBodies(PxBaseTask* continuation) = 0;
		virtual void updateShapes(PxBaseTask* continuation) = 0;
		virtual void preIntegrateAndUpdateBound(PxBaseTask* continuation, const PxVec3 gravity, const PxReal dt) = 0;
		virtual void updateParticleSystemsAndSoftBodies() = 0;
		virtual void sortContacts() = 0;
		virtual void update(PxBitMapPinned& changedHandleMap) = 0;
		virtual void updateArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex) = 0;
		virtual void updateArticulationJoint(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex) = 0;
		virtual void updateArticulationExtAccel(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex) = 0;
		virtual void updateArticulationAfterIntegration(PxsContext*	llContext, Bp::AABBManagerBase* aabbManager, PxArray<Sc::BodySim*>& ccdBodies,
			PxBaseTask* continuation, IG::IslandSim& islandSim, const float dt) = 0;
		
		virtual void mergeChangedAABBMgHandle(const PxU32 maxAABBMgHandles, const bool suppressedReadback) = 0;
		virtual void gpuDmabackData(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBitMapPinned&  changedAABBMgrHandles, bool suppressReadback) = 0;
		virtual void updateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation) = 0;
		virtual PxU32* getActiveBodies() = 0;
		virtual PxU32* getDeactiveBodies() = 0;
		virtual void** getRigidBodies() = 0;
		virtual PxU32	getNbBodies() = 0;

		virtual PxU32*	getUnfrozenShapes() = 0;
		virtual PxU32*	getFrozenShapes() = 0;
		virtual PxsShapeSim** getShapeSims() = 0;
		virtual PxU32	getNbFrozenShapes() = 0;
		virtual PxU32	getNbUnfrozenShapes() = 0;
		virtual PxU32	getNbShapes() = 0;

		virtual void	clear() = 0;
		virtual void	setBounds(Bp::BoundsArray* boundArray) = 0;
		virtual void	reserve(const PxU32 nbBodies) = 0;

		virtual PxU32   getArticulationRemapIndex(const PxU32 nodeIndex) = 0;
		//virtual void	setParticleSystemManager(PxgParticleSystemCore* psCore) = 0;

		virtual void	copyArticulationData(void* data, void* index, PxArticulationGpuDataType::Enum dataType, const PxU32 nbCopyArticulations, void* copyEvent) = 0;
		virtual void	applyArticulationData(void* data, void* index, PxArticulationGpuDataType::Enum dataType, const PxU32 nbUpdatedArticulations, 
			void* waitEvent, void* signalEvent) = 0;

		//KS - the methods below here should probably be wrapped in if PX_SUPPORT_GPU_PHYSX

		virtual void	copySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, void* copyEvent) = 0;
		virtual void	applySoftBodyData(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, void* applyEvent) = 0;
		virtual	void	copyContactData(Dy::Context* dyContext, void* data, const PxU32 maxContactPairs, void* numContactPairs, void* copyEvent) = 0;
		virtual	void	copyBodyData(PxGpuBodyData* data, PxGpuActorPair* index, const PxU32 nbUpdatedActors, void* copyEvent) = 0;
		virtual	void	applyActorData(void* data, PxGpuActorPair* index, PxActorCacheFlag::Enum flag, const PxU32 nbUpdatedActors, void* waitEvent, void* signalEvent) = 0;
		
		virtual void	syncParticleData() = 0;

		virtual void    updateBoundsAndShapes(Bp::AABBManagerBase& aabbManager, const bool useGpuBp, const bool useDirectApi) = 0;

		virtual	void	computeDenseJacobians(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent) = 0;
		virtual	void	computeGeneralizedMassMatrices(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent) = 0;
		virtual	void	computeGeneralizedGravityForces(const PxIndexDataPair* indices, PxU32 nbIndices, const PxVec3& gravity, void* computeEvent) = 0;
		virtual	void	computeCoriolisAndCentrifugalForces(const PxIndexDataPair* indices, PxU32 nbIndices, void* computeEvent) = 0;		
		virtual void    applyParticleBufferData(const PxU32* indices, const PxGpuParticleBufferIndexPair* indexPairs, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, void* waitEvent, void* signalEvent) = 0;

		virtual void	flushInsertions() = 0;

#if PX_SUPPORT_GPU_PHYSX
		virtual PxU32	getNbDeactivatedFEMCloth() const = 0;
		virtual PxU32	getNbActivatedFEMCloth() const = 0;

		virtual Dy::FEMCloth**	getDeactivatedFEMCloths() const = 0;
		virtual Dy::FEMCloth**	getActivatedFEMCloths() const = 0;


		virtual PxU32	getNbDeactivatedSoftbodies() const = 0;
		virtual PxU32	getNbActivatedSoftbodies() const = 0;

		virtual const PxReal*	getSoftBodyWakeCounters() const = 0;

		virtual Dy::SoftBody**	getDeactivatedSoftbodies() const = 0;
		virtual Dy::SoftBody**	getActivatedSoftbodies() const = 0;

		virtual bool hasFEMCloth() const = 0;
		virtual bool hasSoftBodies() const = 0;

		virtual PxU32				getNbDeactivatedHairSystems() const = 0;
		virtual PxU32				getNbActivatedHairSystems() const = 0;
		virtual Dy::HairSystem**	getDeactivatedHairSystems() const = 0;
		virtual Dy::HairSystem**	getActivatedHairSystems() const = 0;
		virtual bool				hasHairSystems() const = 0;

#endif

		virtual void*	getMPMDataPointer(const Dy::ParticleSystem& psLL, PxMPMParticleDataFlag::Enum flags) = 0;
		virtual void*	getSparseGridDataPointer(const Dy::ParticleSystem& psLL, PxSparseGridDataFlag::Enum flags, PxParticleSolverType::Enum type) = 0;

	protected:
		PxsSimulationControllerCallback* mCallback;
	};
}

#endif
