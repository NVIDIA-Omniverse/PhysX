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

	struct PxsShapeCore;

	class PxPhysXGpu;

	struct PxgSolverConstraintManagerConstants;
	
	class PxsSimulationControllerCallback : public PxUserAllocated
	{
	public:
		virtual void	updateScBodyAndShapeSim(PxBaseTask* continuation) = 0;
		virtual PxU32	getNbCcdBodies() = 0;

		virtual ~PxsSimulationControllerCallback() {}
	};

	class PxsSimulationController : public PxUserAllocated
	{
	public:
					PxsSimulationController(PxsSimulationControllerCallback* callback, PxIntBool gpu) : mCallback(callback), mGPU(gpu)	{}
		virtual		~PxsSimulationController(){}

		virtual void addJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/, IG::IslandSim& /*islandSim*/, PxArray<PxU32>& /*jointIndices*/,
			PxPinnedArray<PxgSolverConstraintManagerConstants>& /*managerIter*/, PxU32 /*uniqueId*/){}
		virtual void removeJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/, PxArray<PxU32>& /*jointIndices*/, IG::IslandSim& /*islandSim*/){}
		virtual void addShape(PxsShapeSim* /*shapeSim*/, const PxU32 /*index*/){}
		virtual void reinsertShape(PxsShapeSim* /*shapeSim*/, const PxU32 /*index*/) {}
		virtual void updateShape(PxsShapeSim& /*shapeSim*/, const PxNodeIndex& /*index*/) {}
		virtual void removeShape(const PxU32 /*index*/){}

		virtual void addDynamic(PxsRigidBody* /*rigidBody*/, const PxNodeIndex& /*nodeIndex*/){}
		virtual void addDynamics(PxsRigidBody** /*rigidBody*/, const PxU32* /*nodeIndex*/, PxU32 /*nbBodies*/) {}
		virtual void addArticulation(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void releaseArticulation(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void releaseDeferredArticulationIds() {}

#if PX_SUPPORT_GPU_PHYSX
		virtual void addSoftBody(Dy::SoftBody* /*softBody*/, const PxNodeIndex& /*nodeIndex*/)	{}
		virtual void releaseSoftBody(Dy::SoftBody* /*softBody*/) 	{}
		virtual void releaseDeferredSoftBodyIds() 	{}
		virtual void activateSoftbody(Dy::SoftBody*) 	{}
		virtual void deactivateSoftbody(Dy::SoftBody*) 	{}
		virtual void activateSoftbodySelfCollision(Dy::SoftBody*) 	{}
		virtual void deactivateSoftbodySelfCollision(Dy::SoftBody*) 	{}
		virtual void setSoftBodyWakeCounter(Dy::SoftBody*) 	{}

		virtual void addParticleFilter(Dy::SoftBody* /*softBodySystem*/, Dy::ParticleSystem* /*particleSystem*/,
			PxU32 /*particleId*/, PxU32 /*userBufferId*/, PxU32 /*tetId*/) 	{}
		virtual void removeParticleFilter(Dy::SoftBody* /*softBodySystem*/,
			const Dy::ParticleSystem* /*particleSystem*/, PxU32 /*particleId*/, PxU32 /*userBufferId*/, PxU32 /*tetId*/) 	{}

		virtual PxU32 addParticleAttachment(Dy::SoftBody* /*softBodySystem*/, const Dy::ParticleSystem* /*particleSystem*/,
			PxU32 /*particleId*/, PxU32 /*userBufferId*/, PxU32 /*tetId*/, const PxVec4& /*barycentrics*/, const bool /*isActive*/) 	{ return 0; }
		virtual void removeParticleAttachment(Dy::SoftBody* /*softBody*/, PxU32 /*handle*/) 	{}

		virtual void addRigidFilter(Dy::SoftBody* /*softBodySystem*/, const PxNodeIndex& /*softBodyNodeIndex*/,
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*vertIndex*/) 	{}
		virtual void removeRigidFilter(Dy::SoftBody* /*softBodySystem*/, 
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*vertIndex*/) 	{}

		virtual PxU32 addRigidAttachment(Dy::SoftBody* /*softBodySystem*/, const PxNodeIndex& /*softBodyNodeIndex*/,
			PxsRigidBody* /*rigidBody*/, const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*vertIndex*/, const PxVec3& /*actorSpacePose*/,
			PxConeLimitedConstraint* /*constraint*/, const bool /*isActive*/) 	{ return 0; }
		virtual void removeRigidAttachment(Dy::SoftBody* /*softBody*/, PxU32 /*handle*/) 	{}

		virtual void addTetRigidFilter(Dy::SoftBody* /*softBodySystem*/,
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*tetId*/) 	{}

		virtual PxU32 addTetRigidAttachment(Dy::SoftBody* /*softBodySystem*/,
			PxsRigidBody* /*rigidBody*/, const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*tetIdx*/, 
			const PxVec4& /*barycentrics*/, const PxVec3& /*actorSpacePose*/, PxConeLimitedConstraint* /*constraint*/,
			const bool /*isActive*/) 	{ return 0; }

		virtual void removeTetRigidFilter(Dy::SoftBody* /*softBody*/, 
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*tetId*/) 	{}

		virtual void addSoftBodyFilter(Dy::SoftBody* /*softBody0*/, Dy::SoftBody* /*softBody1*/, PxU32 /*tetIdx0*/, 
			PxU32 /*tetIdx1*/) 	{}
		virtual void removeSoftBodyFilter(Dy::SoftBody* /*softBody0*/, Dy::SoftBody* /*softBody1*/, PxU32 /*tetIdx0*/,
			PxU32 /*tetId1*/) 	{}
		virtual void addSoftBodyFilters(Dy::SoftBody* /*softBody0*/, Dy::SoftBody* /*softBody1*/, PxU32* /*tetIndices0*/, PxU32* /*tetIndices1*/,
			PxU32 /*tetIndicesSize*/) 	{}
		virtual void removeSoftBodyFilters(Dy::SoftBody* /*softBody0*/, Dy::SoftBody* /*softBody1*/, PxU32* /*tetIndices0*/, PxU32* /*tetIndices1*/,
			PxU32 /*tetIndicesSize*/) 	{}

		virtual PxU32 addSoftBodyAttachment(Dy::SoftBody* /*softBody0*/, Dy::SoftBody* /*softBody1*/, PxU32 /*tetIdx0*/, PxU32 /*tetIdx1*/,
			const PxVec4& /*tetBarycentric0*/, const PxVec4& /*tetBarycentric1*/,
			PxConeLimitedConstraint* /*constraint*/, PxReal /*constraintOffset*/, const bool /*isActive*/) 	{ return 0; }

		virtual void removeSoftBodyAttachment(Dy::SoftBody* /*softBody0*/, PxU32 /*handle*/) 	{}

		virtual void addClothFilter(Dy::SoftBody* /*softBody*/, Dy::FEMCloth* /*cloth*/, PxU32 /*triIdx*/,
			PxU32 /*tetIdx*/) 	{}
		virtual void removeClothFilter(Dy::SoftBody* /*softBody*/, Dy::FEMCloth*, PxU32 /*triId*/,
			PxU32 /*tetId*/) 	{}

		virtual PxU32 addClothAttachment(Dy::SoftBody* /*softBody*/, Dy::FEMCloth* /*cloth*/, PxU32 /*triIdx*/,
			const PxVec4& /*triBarycentric*/, PxU32 /*tetIdx*/, const PxVec4& /*tetBarycentric*/, 
			PxConeLimitedConstraint* /*constraint*/, PxReal /*constraintOffset*/,
			const bool /*isActive*/) 	{ return 0; }

		virtual void removeClothAttachment(Dy::SoftBody* /*softBody*/,PxU32 /*handle*/) 	{}

		virtual void addFEMCloth(Dy::FEMCloth* /*femCloth*/, const PxNodeIndex& /*nodeIndex*/) 	{}
		virtual void releaseFEMCloth(Dy::FEMCloth* /*femCloth*/) 	{}
		virtual void releaseDeferredFEMClothIds() 	{}
		virtual void activateCloth(Dy::FEMCloth* /*femCloth*/) 	{}
		virtual void deactivateCloth(Dy::FEMCloth* /*femCloth*/) 	{}
		virtual void setClothWakeCounter(Dy::FEMCloth*) 	{}

		virtual void addRigidFilter(Dy::FEMCloth* /*cloth*/,
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*vertId*/) 	{}

		virtual void removeRigidFilter(Dy::FEMCloth* /*cloth*/,
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*vertId*/) 	{}

		virtual PxU32 addRigidAttachment(Dy::FEMCloth* /*cloth*/, const PxNodeIndex& /*clothNodeIndex*/,
			PxsRigidBody* /*rigidBody*/, const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*vertIndex*/, const PxVec3& /*actorSpacePose*/,
			PxConeLimitedConstraint* /*constraint*/, const bool /*isActive*/) 	{ return 0; }
		virtual void removeRigidAttachment(Dy::FEMCloth* /*cloth*/, PxU32 /*handle*/) 	{}

		virtual void addTriRigidFilter(Dy::FEMCloth* /*cloth*/,
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*triIdx*/) 	{}

		virtual void removeTriRigidFilter(Dy::FEMCloth* /*cloth*/, 
			const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*triIdx*/) 	{}

		virtual PxU32 addTriRigidAttachment(Dy::FEMCloth* /*cloth*/,
			PxsRigidBody* /*rigidBody*/, const PxNodeIndex& /*rigidNodeIndex*/, PxU32 /*triIdx*/, const PxVec4& /*barycentrics*/, 
			const PxVec3& /*actorSpacePose*/, PxConeLimitedConstraint* /*constraint*/,
			const bool /*isActive*/) 	{ return 0; }

		virtual void removeTriRigidAttachment(Dy::FEMCloth* /*cloth*/, PxU32 /*handle*/) 	{}

		virtual void addClothFilter(Dy::FEMCloth* /*cloth0*/, Dy::FEMCloth* /*cloth1*/, PxU32 /*triIdx0*/, PxU32 /*triIdx1*/) 	{}
		virtual void removeClothFilter(Dy::FEMCloth* /*cloth0*/, Dy::FEMCloth* /*cloth1*/, PxU32 /*triIdx0*/, PxU32 /*triIdx1*/) 	{}

		virtual PxU32 addTriClothAttachment(Dy::FEMCloth* /*cloth0*/, Dy::FEMCloth* /*cloth1*/, PxU32 /*triIdx0*/, PxU32 /*triIdx1*/,
			const PxVec4& /*triBarycentric0*/, const PxVec4& /*triBarycentric1*/, const bool /*addToActive*/) 	{ return 0; }

		virtual void removeTriClothAttachment(Dy::FEMCloth* /*cloth*/, PxU32 /*handle*/) 	{}

		virtual void addParticleSystem(Dy::ParticleSystem* /*particleSystem*/, const PxNodeIndex& /*nodeIndex*/, PxParticleSolverType::Enum /*type*/) 	{}
		virtual void releaseParticleSystem(Dy::ParticleSystem* /*particleSystem*/, PxParticleSolverType::Enum /*type*/) 	{}
		virtual void releaseDeferredParticleSystemIds() 	{}

		virtual void addHairSystem(Dy::HairSystem* /*hairSystem*/, const PxNodeIndex& /*nodeIndex*/) 	{}
		virtual void releaseHairSystem(Dy::HairSystem* /*hairSystem*/) 	{}
		virtual void releaseDeferredHairSystemIds() 	{}
		virtual void activateHairSystem(Dy::HairSystem*) 	{}
		virtual void deactivateHairSystem(Dy::HairSystem*) 	{}
		virtual void setHairSystemWakeCounter(Dy::HairSystem*) 	{}
#endif

		virtual void flush() {}

		virtual void updateDynamic(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/){}
		virtual void updateBodies(PxsRigidBody** /*rigidBodies*/,  PxU32* /*nodeIndices*/, const PxU32 /*nbBodies*/) {}
//		virtual void updateBody(PxsRigidBody* /*rigidBody*/, const PxU32 /*nodeIndex*/) {}
		virtual void updateBodies(PxBaseTask* /*continuation*/){}
		virtual void updateShapes(PxBaseTask* /*continuation*/) {}
		virtual	void preIntegrateAndUpdateBound(PxBaseTask* /*continuation*/, const PxVec3 /*gravity*/, const PxReal /*dt*/){}
		virtual void updateParticleSystemsAndSoftBodies(){}
		virtual void sortContacts(){}
		virtual void update(PxBitMapPinned& /*changedHandleMap*/){}
		virtual void updateArticulation(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationJoint(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
//		virtual void updateArticulationTendon(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationExtAccel(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationAfterIntegration(PxsContext*	/*llContext*/, Bp::AABBManagerBase* /*aabbManager*/,
			PxArray<Sc::BodySim*>& /*ccdBodies*/, PxBaseTask* /*continuation*/, IG::IslandSim& /*islandSim*/, float /*dt*/)	{}

		virtual void mergeChangedAABBMgHandle(const PxU32 /*maxAABBMgHandles*/, const bool /*enableDirectGPUAPI*/) {}
		virtual void gpuDmabackData(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, PxBitMapPinned& /*changedAABBMgrHandles*/, bool /*enableDirectGPUAPI*/){}
		virtual void	updateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation) = 0;
		virtual PxU32* getActiveBodies()		{ return NULL;	}
		virtual PxU32* getDeactiveBodies()		{ return NULL;	}
		virtual void** getRigidBodies()			{ return NULL;	}
		virtual PxU32	getNbBodies()			{ return 0;	}

		virtual PxU32*	getUnfrozenShapes()		{ return NULL;	}
		virtual PxU32*	getFrozenShapes()		{ return NULL;	}
		virtual PxsShapeSim** getShapeSims()	{ return NULL;	}
		virtual PxU32	getNbFrozenShapes()		{ return 0;	}
		virtual PxU32	getNbUnfrozenShapes()	{ return 0;	}
		virtual PxU32	getNbShapes()			{ return 0;	}

		virtual void	clear() { }
		virtual void	setBounds(Bp::BoundsArray* /*boundArray*/){}
		virtual void	reserve(const PxU32 /*nbBodies*/) {}

		virtual PxU32   getArticulationRemapIndex(const PxU32 /*nodeIndex*/) { return PX_INVALID_U32;}
		//virtual void	setParticleSystemManager(PxgParticleSystemCore* psCore) = 0;

		virtual void	copyArticulationData(void* /*jointData*/, void* /*index*/, PxArticulationGpuDataType::Enum /*dataType*/, const PxU32 /*nbUpdatedArticulations*/, void* /*copyEvent*/) {}
		virtual void	applyArticulationData(void* /*data*/, void* /*index*/, PxArticulationGpuDataType::Enum /*dataType*/, const PxU32 /*nbUpdatedArticulations*/, void* /*waitEvent*/, void* /*signalEvent*/) {}

		//KS - the methods below here should probably be wrapped in if PX_SUPPORT_GPU_PHYSX
		// PT: isn't the whole class only needed for GPU anyway?

		virtual void	copySoftBodyData(void** /*data*/, void* /*dataSizes*/, void* /*softBodyIndices*/, PxSoftBodyGpuDataFlag::Enum /*flag*/, const PxU32 /*nbCopySoftBodies*/, const PxU32 /*maxSize*/, void* /*copyEvent*/) {}
		virtual void	applySoftBodyData(void** /*data*/, void* /*dataSizes*/, void* /*softBodyIndices*/, PxSoftBodyGpuDataFlag::Enum /*flag*/, const PxU32 /*nbUpdatedSoftBodies*/, const PxU32 /*maxSize*/, void* /*applyEvent*/) {}
		virtual	void	copyContactData(Dy::Context* /*dyContext*/, void* /*data*/, const PxU32 /*maxContactPairs*/, void* /*numContactPairs*/, void* /*copyEvent*/) {}
		virtual	void	copyBodyData(PxGpuBodyData* /*data*/, PxGpuActorPair* /*index*/, const PxU32 /*nbCopyActors*/, void* /*copyEvent*/){}
		virtual	void	applyActorData(void* /*data*/, PxGpuActorPair* /*index*/, PxActorCacheFlag::Enum /*flag*/, const PxU32 /*nbUpdatedActors*/, void* /*waitEvent*/, void* /*signalEvent*/) {}

		virtual	void	evaluateSDFDistances(const PxU32* /*sdfShapeIds*/, const PxU32 /*nbShapes*/, const PxVec4* /*samplePointsConcatenated*/,
			const PxU32* /*samplePointCountPerShape*/, const PxU32 /*maxPointCount*/, PxVec4* /*localGradientAndSDFConcatenated*/, void* /*event*/)	{}
		virtual	PxU32	getInternalShapeIndex(const PxsShapeCore& /*shapeCore*/)	{ return PX_INVALID_U32;	}

		virtual void	syncParticleData()	{}

		virtual void    updateBoundsAndShapes(Bp::AABBManagerBase& /*aabbManager*/, bool/* useGpuBp*/, bool /*useDirectApi*/){}

		virtual	void	computeDenseJacobians(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, void* /*computeEvent*/){}
		virtual	void	computeGeneralizedMassMatrices(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, void* /*computeEvent*/){}
		virtual	void	computeGeneralizedGravityForces(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, const PxVec3& /*gravity*/, void* /*computeEvent*/){}
		virtual	void	computeCoriolisAndCentrifugalForces(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, void* /*computeEvent*/) {}
		virtual void    applyParticleBufferData(const PxU32* /*indices*/, const PxGpuParticleBufferIndexPair* /*indexPairs*/, const PxParticleBufferFlags* /*flags*/, PxU32 /*nbUpdatedBuffers*/, void* /*waitEvent*/, void* /*signalEvent*/) {}

		virtual void	flushInsertions()	{}

#if PX_SUPPORT_GPU_PHYSX
		virtual PxU32				getNbDeactivatedFEMCloth()		const	{ return 0;		}
		virtual PxU32				getNbActivatedFEMCloth()		const	{ return 0;		}

		virtual Dy::FEMCloth**		getDeactivatedFEMCloths()		const	{ return NULL;	}
		virtual Dy::FEMCloth**		getActivatedFEMCloths()			const	{ return NULL;	}

		virtual PxU32				getNbDeactivatedSoftbodies()	const	{ return 0;		}
		virtual PxU32				getNbActivatedSoftbodies()		const	{ return 0;		}

		virtual const PxReal*		getSoftBodyWakeCounters()		const	{ return NULL;	}
		virtual const PxReal*		getHairSystemWakeCounters()		const	{ return NULL;	}

		virtual Dy::SoftBody**		getDeactivatedSoftbodies()		const	{ return NULL;	}
		virtual Dy::SoftBody**		getActivatedSoftbodies()		const	{ return NULL;	}

		virtual bool				hasFEMCloth()					const	{ return false;	}
		virtual bool				hasSoftBodies()					const	{ return false;	}

		virtual PxU32				getNbDeactivatedHairSystems()	const	{ return 0;		}
		virtual PxU32				getNbActivatedHairSystems()		const	{ return 0;		}
		virtual Dy::HairSystem**	getDeactivatedHairSystems()		const	{ return NULL;	}
		virtual Dy::HairSystem**	getActivatedHairSystems()		const	{ return NULL;	}
		virtual bool				hasHairSystems()				const	{ return false;	}
#endif

		virtual void*	getMPMDataPointer(const Dy::ParticleSystem& /*psLL*/, PxMPMParticleDataFlag::Enum /*flags*/) { return NULL; }
		virtual void*	getSparseGridDataPointer(const Dy::ParticleSystem& /*psLL*/, PxSparseGridDataFlag::Enum /*flags*/, PxParticleSolverType::Enum /*type*/) { return NULL; }

	protected:
		PxsSimulationControllerCallback*	mCallback;
	public:
		const PxIntBool						mGPU;	// PT: true for GPU version, used to quickly skip calls for CPU version
	};
}

#endif
