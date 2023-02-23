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

#define NOMINMAX

#include "common/PxProfileZone.h"
#include "foundation/PxTime.h"
#include "ScPhysics.h"
#include "ScScene.h"
#include "BpAABBManager.h"
#include "BpBroadPhase.h"
#include "ScStaticSim.h"
#include "ScConstraintSim.h"
#include "ScConstraintProjectionManager.h"
#include "ScConstraintCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointCore.h"
#include "ScArticulationTendonCore.h"
#include "ScArticulationSensor.h"
#include "ScArticulationSim.h"
#include "ScArticulationJointSim.h"
#include "foundation/PxTime.h"
#include "ScArticulationTendonSim.h"
#include "ScArticulationSensorSim.h"
#include "ScConstraintInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScSimStats.h"
#include "ScTriggerPairs.h"
#include "ScObjectIDTracker.h"
#include "PxvManager.h"
#include "PxvGlobals.h"
#include "DyContext.h"
#include "PxsCCD.h"
#include "PxsSimpleIslandManager.h"
#include "ScSimulationController.h"
#include "ScSqBoundsManager.h"
#include "ScElementSim.h"

#if defined(__APPLE__) && defined(__POWERPC__)
#include <ppc_intrinsics.h>
#endif

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#include "PxsKernelWrangler.h"
#include "PxsHeapMemoryAllocator.h"
#include "cudamanager/PxCudaContextManager.h"
#endif

#include "PxsMemoryManager.h"

////////////

#include "PxvNphaseImplementationContext.h"
#include "ScShapeInteraction.h"
#include "ScElementInteractionMarker.h"
#include "PxsContext.h"

#include "PxRigidDynamic.h"

#include "PxvDynamics.h"
#include "DyFeatherstoneArticulation.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxSoftBody.h"
#include "ScSoftBodySim.h"
#include "DySoftBody.h"
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFEMCloth.h"
#include "PxHairSystem.h"
#endif
#include "ScFEMClothSim.h"
#include "DyFEMCloth.h"
#include "ScParticleSystemSim.h"
#include "DyParticleSystem.h"
#include "ScHairSystemSim.h"
#include "DyHairSystem.h"
#endif

#include "CmPtrTable.h"
#include "CmTransformUtils.h"

using namespace physx;
using namespace physx::Cm;
using namespace physx::Dy;

PX_IMPLEMENT_OUTPUT_ERROR

namespace physx { 
namespace Sc {

class LLArticulationRCPool : public PxPool<FeatherstoneArticulation, PxAlignedAllocator<64> >
{
public:
	LLArticulationRCPool() {}
};

#if PX_SUPPORT_GPU_PHYSX
	class LLSoftBodyPool : public PxPool<SoftBody, PxAlignedAllocator<64> >
	{
	public:
		LLSoftBodyPool() {}
	};

	class LLFEMClothPool : public PxPool<FEMCloth, PxAlignedAllocator<64> >
	{
	public:
		LLFEMClothPool() {}
	};

	class LLParticleSystemPool : public PxPool<ParticleSystem, PxAlignedAllocator<64> >
	{
	public:
		LLParticleSystemPool() {}
	};

	class LLHairSystemPool : public PxPool<HairSystem, PxAlignedAllocator<64> >
	{
	public:
		LLHairSystemPool() {}
	};
#endif

static const char* sFilterShaderDataMemAllocId = "SceneDesc filterShaderData";

}}

void PxcClearContactCacheStats();
void PxcDisplayContactCacheStats();

class ScAfterIntegrationTask :  public Cm::Task
{
public:
	static const PxU32 MaxTasks = 256;
private:
	const PxNodeIndex* const	mIndices;
	const PxU32					mNumBodies;
	PxsContext*					mContext;
	Context*					mDynamicsContext;
	PxsTransformCache&			mCache;
	Sc::Scene&					mScene;
	
public:

	ScAfterIntegrationTask(const PxNodeIndex* const indices, PxU32 numBodies, PxsContext* context, Context* dynamicsContext, PxsTransformCache& cache, Sc::Scene& scene) :
		Cm::Task		(scene.getContextId()),
		mIndices		(indices),
		mNumBodies		(numBodies),
		mContext		(context),
		mDynamicsContext(dynamicsContext),
		mCache			(cache),
		mScene			(scene)
	{
	}

	virtual void runInternal()
	{		
		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		Sc::BodySim* bpUpdates[MaxTasks];
		Sc::BodySim* ccdBodies[MaxTasks];
		Sc::BodySim* activateBodies[MaxTasks];
		Sc::BodySim* deactivateBodies[MaxTasks];
		PxU32 nbBpUpdates = 0, nbCcdBodies = 0;

		IG::SimpleIslandManager& manager = *mScene.getSimpleIslandManager();
		const IG::IslandSim& islandSim = manager.getAccurateIslandSim();
		Bp::BoundsArray& boundsArray = mScene.getBoundsArray();

		Sc::BodySim* frozen[MaxTasks], * unfrozen[MaxTasks];
		PxU32 nbFrozen = 0, nbUnfrozen = 0;
		PxU32 nbActivated = 0, nbDeactivated = 0;

		for(PxU32 i = 0; i < mNumBodies; i++)
		{
			PxsRigidBody* rigid = islandSim.getRigidBody(mIndices[i]);
			Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
			//This move to PxgPostSolveWorkerTask for the gpu dynamic
			//bodySim->sleepCheck(mDt, mOneOverDt, mEnableStabilization);
		
			PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();
			//If we got in this code, then this is an active object this frame. The solver computed the new wakeCounter and we 
			//commit it at this stage. We need to do it this way to avoid a race condition between the solver and the island gen, where
			//the island gen may have deactivated a body while the solver decided to change its wake counter.
			bodyCore.wakeCounter = bodyCore.solverWakeCounter;
			PxsRigidBody& llBody = bodySim->getLowLevelBody();

			const PxIntBool isFrozen = bodySim->isFrozen();
			if(!isFrozen)
			{
				bpUpdates[nbBpUpdates++] = bodySim;

				// PT: TODO: remove duplicate "isFrozen" test inside updateCached
//				bodySim->updateCached(NULL);
				bodySim->updateCached(mCache, boundsArray);
			}

			if(llBody.isFreezeThisFrame() && isFrozen)
				frozen[nbFrozen++] = bodySim;
			else if(llBody.isUnfreezeThisFrame())
				unfrozen[nbUnfrozen++] = bodySim;

			if(bodyCore.mFlags & PxRigidBodyFlag::eENABLE_CCD)
				ccdBodies[nbCcdBodies++] = bodySim;

			if(llBody.isActivateThisFrame())
			{
				PX_ASSERT(!llBody.isDeactivateThisFrame());
				activateBodies[nbActivated++] = bodySim;
			}
			else if(llBody.isDeactivateThisFrame())
			{
				deactivateBodies[nbDeactivated++] = bodySim;
			}
			llBody.clearAllFrameFlags();
		}
		if(nbBpUpdates)
		{
			mCache.setChangedState();
			boundsArray.setChangedState();
		}

		if(nbBpUpdates>0 || nbFrozen > 0 || nbCcdBodies>0 || nbActivated>0 || nbDeactivated>0)
		{
			//Write active bodies to changed actor map
			mContext->getLock().lock();
			PxBitMapPinned& changedAABBMgrHandles = mScene.getAABBManager()->getChangedAABBMgActorHandleMap();
			
			for(PxU32 i = 0; i < nbBpUpdates; i++)
			{
				// PT: ### changedMap pattern #1
				PxU32 nbElems = bpUpdates[i]->getNbElements();
				Sc::ElementSim** elems = bpUpdates[i]->getElements();
				while (nbElems--)
				{
					Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(*elems++);
					// PT: TODO: what's the difference between this test and "isInBroadphase" as used in bodySim->updateCached ?
					// PT: Also, shouldn't it be "isInAABBManager" rather than BP ?
					if (sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))	// TODO: need trigger shape here?
						changedAABBMgrHandles.growAndSet(sim->getElementID());
				}
			}

			PxArray<Sc::BodySim*>& sceneCcdBodies = mScene.getCcdBodies();
			for (PxU32 i = 0; i < nbCcdBodies; i++)
				sceneCcdBodies.pushBack(ccdBodies[i]);

			for(PxU32 i=0;i<nbFrozen;i++)
			{
				PX_ASSERT(frozen[i]->isFrozen());
				frozen[i]->freezeTransforms(&changedAABBMgrHandles);
			}

			for(PxU32 i=0;i<nbUnfrozen;i++)
			{
				PX_ASSERT(!unfrozen[i]->isFrozen());
				unfrozen[i]->createSqBounds();
			}
			
			for(PxU32 i = 0; i < nbActivated; ++i)
				activateBodies[i]->notifyNotReadyForSleeping();

			for(PxU32 i = 0; i < nbDeactivated; ++i)
				deactivateBodies[i]->notifyReadyForSleeping();

			mContext->getLock().unlock();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.afterIntegrationTask";
	}

private:
	PX_NOCOPY(ScAfterIntegrationTask)
};

static const bool gUseNewTaskAllocationScheme = false;

class ScSimulationControllerCallback : public PxsSimulationControllerCallback
{
	Sc::Scene* mScene; 
public:

	ScSimulationControllerCallback(Sc::Scene* scene) : mScene(scene)
	{
	}
	
	virtual void updateScBodyAndShapeSim(PxBaseTask* continuation)
	{
		PxsContext* contextLL =  mScene->getLowLevelContext();
		IG::SimpleIslandManager* islandManager = mScene->getSimpleIslandManager();
		Dy::Context* dynamicContext = mScene->getDynamicsContext();

		Cm::FlushPool& flushPool = contextLL->getTaskPool();

		const PxU32 MaxBodiesPerTask = ScAfterIntegrationTask::MaxTasks;

		PxsTransformCache& cache = contextLL->getTransformCache();

		const IG::IslandSim& islandSim = islandManager->getAccurateIslandSim();

		/*const*/ PxU32 numBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

		const PxNodeIndex*const nodeIndices = islandSim.getActiveNodes(IG::Node::eRIGID_BODY_TYPE);

		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		if(!gUseNewTaskAllocationScheme)
		{
			PxU32 nbShapes = 0;
			PxU32 startIdx = 0;
			for (PxU32 i = 0; i < numBodies; i++)
			{
				if (nbShapes >= MaxBodiesPerTask)
				{
					ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices + startIdx, i - startIdx,
						contextLL, dynamicContext, cache, *mScene));
					task->setContinuation(continuation);
					task->removeReference();
					startIdx = i;
					nbShapes = 0;
				}
				PxsRigidBody* rigid = islandSim.getRigidBody(nodeIndices[i]);
				Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
				nbShapes += PxMax(1u, bodySim->getNbShapes()); //Always add at least 1 shape in, even if the body has zero shapes because there is still some per-body overhead
			}

			if (nbShapes)
			{
				ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices + startIdx, numBodies - startIdx,
					contextLL, dynamicContext, cache, *mScene));
				task->setContinuation(continuation);
				task->removeReference();
			}
		}
		else
		{
			// PT:
			const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();

			PxU32 nbPerTask;
			if(numCpuTasks)
				nbPerTask = numBodies > numCpuTasks ? numBodies / numCpuTasks : numBodies;
			else
				nbPerTask = numBodies;

			// PT: we need to respect that limit even with a single thread, because of hardcoded buffer limits in ScAfterIntegrationTask.
			if(nbPerTask>MaxBodiesPerTask)
				nbPerTask = MaxBodiesPerTask;

			PxU32 start = 0;
			while(numBodies)
			{
				const PxU32 nb = numBodies < nbPerTask ? numBodies : nbPerTask;

				ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices+start, nb, 
					contextLL, dynamicContext, cache, *mScene));

				start += nb;
				numBodies -= nb;

				task->setContinuation(continuation);
				task->removeReference();
			}
		}
	}

	virtual PxU32 getNbCcdBodies()	
	{ 
		return mScene->getCcdBodies().size(); 
	}
};

class PxgUpdateBodyAndShapeStatusTask :  public Cm::Task
{
public:
	static const PxU32 MaxTasks = 2048;
private:
	const PxNodeIndex* const mNodeIndices;
	const PxU32 mNumBodies;
	Sc::Scene& mScene;
	void**	mRigidBodyLL;
	PxU32*	mActivatedBodies;
	PxU32*	mDeactivatedBodies;
	PxI32&	mCCDBodyWriteIndex;
	
public:

	PxgUpdateBodyAndShapeStatusTask(const PxNodeIndex* const indices, PxU32 numBodies, void** rigidBodyLL, PxU32* activatedBodies, PxU32* deactivatedBodies, Sc::Scene& scene, PxI32& ccdBodyWriteIndex) : 
		Cm::Task			(scene.getContextId()),
		mNodeIndices		(indices),
		mNumBodies			(numBodies),
		mScene				(scene),
		mRigidBodyLL		(rigidBodyLL),
		mActivatedBodies	(activatedBodies),  
		mDeactivatedBodies	(deactivatedBodies),
		mCCDBodyWriteIndex	(ccdBodyWriteIndex)
	{
	}

	virtual void runInternal()
	{		
		IG::SimpleIslandManager& islandManager = *mScene.getSimpleIslandManager();
		const IG::IslandSim& islandSim = islandManager.getAccurateIslandSim();

		PxU32 nbCcdBodies = 0;

		PxArray<Sc::BodySim*>& sceneCcdBodies = mScene.getCcdBodies();
		Sc::BodySim* ccdBodies[MaxTasks];

		const size_t bodyOffset =  PX_OFFSET_OF_RT(Sc::BodySim, getLowLevelBody());

		for(PxU32 i=0; i<mNumBodies; ++i)
		{
			const PxU32 nodeIndex = mNodeIndices[i].index();
			PxsRigidBody* rigidLL = reinterpret_cast<PxsRigidBody*>(mRigidBodyLL[nodeIndex]);

			PxsBodyCore* bodyCore = &rigidLL->getCore();
			bodyCore->wakeCounter = bodyCore->solverWakeCounter;
			//we can set the frozen/unfrozen flag in GPU, but we have copied the internalflags
			//from the solverbodysleepdata to pxsbodycore, so we just need to clear the frozen flag in here
			rigidLL->clearAllFrameFlags();

			PX_ASSERT(mActivatedBodies[nodeIndex] <= 1);
			PX_ASSERT(mDeactivatedBodies[nodeIndex] <= 1);
			if(mActivatedBodies[nodeIndex])
			{
				PX_ASSERT(bodyCore->wakeCounter > 0.0f);
				islandManager.activateNode(mNodeIndices[i]);
			}
			else if(mDeactivatedBodies[nodeIndex])
			{
				//KS - the CPU code can reset the wake counter due to lost touches in parallel with the solver, so we need to verify
				//that the wakeCounter is still 0 before deactivating the node
				if (bodyCore->wakeCounter == 0.0f)
				{
					islandManager.deactivateNode(mNodeIndices[i]);
				}
			}

			if (bodyCore->mFlags & PxRigidBodyFlag::eENABLE_CCD)
			{
				PxsRigidBody* rigidBody = islandSim.getRigidBody(mNodeIndices[i]);
				Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - bodyOffset);
				ccdBodies[nbCcdBodies++] = bodySim;
			}
		}
		if(nbCcdBodies > 0)
		{
			PxI32 startIndex = PxAtomicAdd(&mCCDBodyWriteIndex, PxI32(nbCcdBodies)) - PxI32(nbCcdBodies);
			for(PxU32 a = 0; a < nbCcdBodies; ++a)
			{
				sceneCcdBodies[startIndex + a] = ccdBodies[a];
			}
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.PxgUpdateBodyAndShapeStatusTask";
	}

private:
	PX_NOCOPY(PxgUpdateBodyAndShapeStatusTask)
};

class PxgSimulationControllerCallback : public PxsSimulationControllerCallback
{
	Sc::Scene* mScene; 
	PxI32 mCcdBodyWriteIndex;

public:
	PxgSimulationControllerCallback(Sc::Scene* scene) : mScene(scene), mCcdBodyWriteIndex(0)
	{
	}

	virtual void updateScBodyAndShapeSim(PxBaseTask* continuation)
	{
		IG::SimpleIslandManager* islandManager = mScene->getSimpleIslandManager();
		PxsSimulationController* simulationController = mScene->getSimulationController();
		PxsContext*	contextLL = mScene->getLowLevelContext();
		IG::IslandSim& islandSim = islandManager->getAccurateIslandSim();
		const PxU32 numBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
		const PxNodeIndex*const nodeIndices = islandSim.getActiveNodes(IG::Node::eRIGID_BODY_TYPE);

		PxU32* activatedBodies = simulationController->getActiveBodies();
		PxU32* deactivatedBodies = simulationController->getDeactiveBodies();

		//PxsRigidBody** rigidBodyLL = simulationController->getRigidBodies();
		void** rigidBodyLL = simulationController->getRigidBodies();

		Cm::FlushPool& flushPool = contextLL->getTaskPool();

		PxArray<Sc::BodySim*>& ccdBodies = mScene->getCcdBodies();
		ccdBodies.forceSize_Unsafe(0);
		ccdBodies.reserve(numBodies);
		ccdBodies.forceSize_Unsafe(numBodies);

		mCcdBodyWriteIndex = 0;

		for(PxU32 i = 0; i < numBodies; i+=PxgUpdateBodyAndShapeStatusTask::MaxTasks)
		{
			PxgUpdateBodyAndShapeStatusTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgUpdateBodyAndShapeStatusTask)), PxgUpdateBodyAndShapeStatusTask(nodeIndices + i, 
				PxMin(PxgUpdateBodyAndShapeStatusTask::MaxTasks, numBodies - i), rigidBodyLL, activatedBodies, deactivatedBodies, *mScene, mCcdBodyWriteIndex));
			task->setContinuation(continuation);
			task->removeReference();
		}
		
		PxU32* unfrozenShapeIndices = simulationController->getUnfrozenShapes();
		PxU32* frozenShapeIndices = simulationController->getFrozenShapes();
		const PxU32 nbFrozenShapes = simulationController->getNbFrozenShapes();
		const PxU32 nbUnfrozenShapes = simulationController->getNbUnfrozenShapes();

		PxsShapeSim** shapeSimsLL = simulationController->getShapeSims();
	
		const size_t shapeOffset = PX_OFFSET_OF_RT(Sc::ShapeSim, getLLShapeSim());

		for(PxU32 i=0; i<nbFrozenShapes; ++i)
		{
			const PxU32 shapeIndex = frozenShapeIndices[i];
			PxsShapeSim* shapeLL = shapeSimsLL[shapeIndex];
			Sc::ShapeSim* shape = reinterpret_cast<Sc::ShapeSim*>(reinterpret_cast<PxU8*>(shapeLL) - shapeOffset);
			shape->destroySqBounds();
		}

		for(PxU32 i=0; i<nbUnfrozenShapes; ++i)
		{
			const PxU32 shapeIndex = unfrozenShapeIndices[i];
			PxsShapeSim* shapeLL = shapeSimsLL[shapeIndex];
			Sc::ShapeSim* shape = reinterpret_cast<Sc::ShapeSim*>(reinterpret_cast<PxU8*>(shapeLL) - shapeOffset);
			shape->createSqBounds();
		}


#if PX_SUPPORT_GPU_PHYSX
		if (simulationController->hasFEMCloth())
		{

			//KS - technically, there's a race condition calling activateNode/deactivateNode, but we know that it is 
			//safe because these deactivate/activate calls came from the solver. This means that we know that the 
			//actors are active currently, so at most we are just clearing/setting the ready for sleeping flag.
			//None of the more complex logic that touching shared state will be executed.
			const PxU32 nbActivatedCloth = simulationController->getNbActivatedFEMCloth();
			Dy::FEMCloth** activatedCloths = simulationController->getActivatedFEMCloths();

			for (PxU32 i = 0; i < nbActivatedCloth; ++i)
			{
				PxNodeIndex nodeIndex = activatedCloths[i]->getFEMClothSim()->getNodeIndex();

				islandManager->activateNode(nodeIndex);
			}

			const PxU32 nbDeactivatedCloth = simulationController->getNbDeactivatedFEMCloth();
			Dy::FEMCloth** deactivatedCloths = simulationController->getDeactivatedFEMCloths();

			for (PxU32 i = 0; i < nbDeactivatedCloth; ++i)
			{
				PxNodeIndex nodeIndex = deactivatedCloths[i]->getFEMClothSim()->getNodeIndex();

				islandManager->deactivateNode(nodeIndex);
			}
		}

		if (simulationController->hasSoftBodies())
		{

			//KS - technically, there's a race condition calling activateNode/deactivateNode, but we know that it is 
			//safe because these deactivate/activate calls came from the solver. This means that we know that the 
			//actors are active currently, so at most we are just clearing/setting the ready for sleeping flag.
			//None of the more complex logic that touching shared state will be executed.

			const PxU32 nbDeactivatedSB = simulationController->getNbDeactivatedSoftbodies();
			Dy::SoftBody** deactivatedSB = simulationController->getDeactivatedSoftbodies();

			for (PxU32 i = 0; i < nbDeactivatedSB; ++i)
			{
				PxNodeIndex nodeIndex = deactivatedSB[i]->getSoftBodySim()->getNodeIndex();
				
				islandManager->deactivateNode(nodeIndex);
			}

			const PxU32 nbActivatedSB = simulationController->getNbActivatedSoftbodies();
			Dy::SoftBody** activatedSB = simulationController->getActivatedSoftbodies();

			for (PxU32 i = 0; i < nbActivatedSB; ++i)
			{
				PxNodeIndex nodeIndex = activatedSB[i]->getSoftBodySim()->getNodeIndex();

				islandManager->activateNode(nodeIndex);
			}
		}

		if (simulationController->hasHairSystems())
		{
			// comment from KS regarding race condition applies here, too
			const PxU32 nbDeactivatedHS = simulationController->getNbDeactivatedHairSystems();
			Dy::HairSystem** deactivatedHS = simulationController->getDeactivatedHairSystems();
			for (PxU32 i = 0; i < nbDeactivatedHS; ++i)
			{
				PxNodeIndex nodeIndex = deactivatedHS[i]->getHairSystemSim()->getNodeIndex();
				islandManager->deactivateNode(nodeIndex);
			}

			const PxU32 nbActivatedHS = simulationController->getNbActivatedHairSystems();
			Dy::HairSystem** activatedHS = simulationController->getActivatedHairSystems();
			for (PxU32 i = 0; i < nbActivatedHS; ++i)
			{
				PxNodeIndex nodeIndex = activatedHS[i]->getHairSystemSim()->getNodeIndex();
				islandManager->activateNode(nodeIndex);
			}
		}
#endif

	}

	virtual PxU32	getNbCcdBodies()
	{
		return PxU32(mCcdBodyWriteIndex);
	}
};

static Bp::AABBManagerBase* createAABBManagerCPU(const PxSceneDesc& desc, Bp::BroadPhase* broadPhase, Bp::BoundsArray* boundsArray, PxFloatArrayPinned* contactDistances, PxVirtualAllocator& allocator, PxU64 contextID)
{
	return PX_NEW(Bp::AABBManager)(*broadPhase, *boundsArray, *contactDistances,
		desc.limits.maxNbAggregates, desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes, allocator, contextID,
		desc.kineKineFilteringMode, desc.staticKineFilteringMode);
}

#if PX_SUPPORT_GPU_PHYSX
static Bp::AABBManagerBase* createAABBManagerGPU(PxsKernelWranglerManager* kernelWrangler, PxCudaContextManager* cudaContextManager, PxsHeapMemoryAllocatorManager* heapMemoryAllocationManager,
												const PxSceneDesc& desc, Bp::BroadPhase* broadPhase, Bp::BoundsArray* boundsArray, PxFloatArrayPinned* contactDistances, PxVirtualAllocator& allocator, PxU64 contextID)
{
	return PxvGetPhysXGpu(true)->createGpuAABBManager(
		kernelWrangler,
		cudaContextManager,
		desc.gpuComputeVersion,
		desc.gpuDynamicsConfig,
		heapMemoryAllocationManager,
		*broadPhase, *boundsArray, *contactDistances,
		desc.limits.maxNbAggregates, desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes, allocator, contextID,
		desc.kineKineFilteringMode, desc.staticKineFilteringMode);
}
#endif

Sc::Scene::Scene(const PxSceneDesc& desc, PxU64 contextID) :
	mContextId						(contextID),
	mActiveBodies					("sceneActiveBodies"),
	mActiveKinematicBodyCount		(0),
	mActiveDynamicBodyCount			(0),
	mActiveKinematicsCopy			(NULL),
	mActiveKinematicsCopyCapacity	(0),
	mPointerBlock8Pool				("scenePointerBlock8Pool"),
	mPointerBlock16Pool				("scenePointerBlock16Pool"),
	mPointerBlock32Pool				("scenePointerBlock32Pool"),
	mLLContext						(NULL),
	mAABBManager					(NULL),
	mCCDContext						(NULL),
	mNumFastMovingShapes			(0),
	mCCDPass						(0),
	mSimpleIslandManager			(NULL),
	mDynamicsContext				(NULL),
	mMemoryManager					(NULL),
#if PX_SUPPORT_GPU_PHYSX
	mGpuWranglerManagers			(NULL),
	mHeapMemoryAllocationManager	(NULL),
#endif
	mSimulationController			(NULL),
	mSimulationControllerCallback	(NULL),
	mGravity						(PxVec3(0.0f)),
	mBodyGravityDirty				(true),	
	mDt								(0),
	mOneOverDt						(0),
	mTimeStamp						(1),		// PT: has to start to 1 to fix determinism bug. I don't know why yet but it works.
	mReportShapePairTimeStamp		(0),
	mTriggerBufferAPI				("sceneTriggerBufferAPI"),
	mArticulations					("sceneArticulations"),
#if PX_SUPPORT_GPU_PHYSX
	mSoftBodies						("sceneSoftBodies"),
	mFEMCloths       	            ("sceneFEMCloths"), 
	mParticleSystems				("sceneParticleSystems"),
	mHairSystems					("sceneHairSystems"),
#endif
	mBrokenConstraints				("sceneBrokenConstraints"),
	mActiveBreakableConstraints		("sceneActiveBreakableConstraints"),
	mMemBlock128Pool				("PxsContext ConstraintBlock128Pool"),
	mMemBlock256Pool				("PxsContext ConstraintBlock256Pool"),
	mMemBlock384Pool				("PxsContext ConstraintBlock384Pool"),
	mNPhaseCore						(NULL),
	mKineKineFilteringMode			(desc.kineKineFilteringMode),
	mStaticKineFilteringMode		(desc.staticKineFilteringMode),
	mSleepBodies					("sceneSleepBodies"),
	mWokeBodies						("sceneWokeBodies"),
	mEnableStabilization			(desc.flags & PxSceneFlag::eENABLE_STABILIZATION),
	mActiveActors					("clientActiveActors"),
	mFrozenActors					("clientFrozenActors"),
	mClientPosePreviewBodies		("clientPosePreviewBodies"),
	mClientPosePreviewBuffer		("clientPosePreviewBuffer"),
	mSimulationEventCallback		(NULL),
	mBroadPhaseCallback				(NULL),
	mInternalFlags					(SceneInternalFlag::eSCENE_DEFAULT),
	mPublicFlags					(desc.flags),
	mStaticAnchor					(NULL),
	mBatchRemoveState				(NULL),
	mLostTouchPairs					("sceneLostTouchPairs"),
	mOutOfBoundsIDs					("sceneOutOfBoundsIds"),
	mVisualizationParameterChanged	(false),
	mMaxNbArticulationLinks			(0),
	mNbRigidStatics					(0),
	mNbRigidDynamics				(0),
	mNbRigidKinematic				(0),
	mSecondPassNarrowPhase			(contextID, this, "ScScene.secondPassNarrowPhase"),
	mPostNarrowPhase				(contextID, this, "ScScene.postNarrowPhase"),
	mFinalizationPhase				(contextID, this, "ScScene.finalizationPhase"),
	mUpdateCCDMultiPass				(contextID, this, "ScScene.updateCCDMultiPass"),
	mAfterIntegration				(contextID, this, "ScScene.afterIntegration"),
	mConstraintProjection			(contextID, this, "ScScene.constraintProjection"),
	mPostSolver						(contextID, this, "ScScene.postSolver"),
	mSolver							(contextID, this, "ScScene.rigidBodySolver"),
	mUpdateBodies					(contextID, this, "ScScene.updateBodies"),
	mUpdateShapes					(contextID, this, "ScScene.updateShapes"),
	mUpdateSimulationController		(contextID, this, "ScScene.updateSimulationController"),
	mUpdateDynamics					(contextID, this, "ScScene.updateDynamics"),
	mProcessLostContactsTask		(contextID, this, "ScScene.processLostContact"),
	mProcessLostContactsTask2		(contextID, this, "ScScene.processLostContact2"),
	mProcessLostContactsTask3		(contextID, this, "ScScene.processLostContact3"),
	mDestroyManagersTask			(contextID, this, "ScScene.destroyManagers"),
	mLostTouchReportsTask			(contextID, this, "ScScene.lostTouchReports"),
	mUnregisterInteractionsTask		(contextID, this, "ScScene.unregisterInteractions"),
	mProcessNarrowPhaseLostTouchTasks(contextID, this, "ScScene.processNpLostTouchTask"),
	mProcessNPLostTouchEvents		(contextID, this, "ScScene.processNPLostTouchEvents"),
	mPostThirdPassIslandGenTask		(contextID, this, "ScScene.postThirdPassIslandGenTask"),
	mPostIslandGen					(contextID, this, "ScScene.postIslandGen"),
	mIslandGen						(contextID, this, "ScScene.islandGen"),
	mPreRigidBodyNarrowPhase		(contextID, this, "ScScene.preRigidBodyNarrowPhase"),
	mSetEdgesConnectedTask			(contextID, this, "ScScene.setEdgesConnectedTask"),
	mFetchPatchEventsTask			(contextID, this, "ScScene.fetchPatchEventsTask"),
	mProcessLostPatchesTask			(contextID, this, "ScScene.processLostSolverPatchesTask"),
	mProcessFoundPatchesTask		(contextID, this, "ScScene.processFoundSolverPatchesTask"),
	mUpdateBoundAndShapeTask		(contextID, this, "ScScene.updateBoundsAndShapesTask"),
	mRigidBodyNarrowPhase			(contextID, this, "ScScene.rigidBodyNarrowPhase"),
	mRigidBodyNPhaseUnlock			(contextID, this, "ScScene.unblockNarrowPhase"),
	mPostBroadPhase					(contextID, this, "ScScene.postBroadPhase"),
	mPostBroadPhaseCont				(contextID, this, "ScScene.postBroadPhaseCont"),
	mPostBroadPhase2				(contextID, this, "ScScene.postBroadPhase2"),
	mPostBroadPhase3				(contextID, this, "ScScene.postBroadPhase3"),
	mPreallocateContactManagers		(contextID, this, "ScScene.preallocateContactManagers"),
	mIslandInsertion				(contextID, this, "ScScene.islandInsertion"),
	mRegisterContactManagers		(contextID, this, "ScScene.registerContactManagers"),
	mRegisterInteractions			(contextID, this, "ScScene.registerInteractions"),
	mRegisterSceneInteractions		(contextID, this, "ScScene.registerSceneInteractions"),
	mBroadPhase						(contextID, this, "ScScene.broadPhase"),
	mAdvanceStep					(contextID, this, "ScScene.advanceStep"),
	mCollideStep					(contextID, this, "ScScene.collideStep"),	
	mBpFirstPass					(contextID, this, "ScScene.broadPhaseFirstPass"),
	mBpSecondPass					(contextID, this, "ScScene.broadPhaseSecondPass"),
	mBpUpdate						(contextID, this, "ScScene.updateBroadPhase"),
	mPreIntegrate                   (contextID, this, "ScScene.preIntegrate"),
	mTaskPool						(16384),
	mTaskManager					(NULL),
	mCudaContextManager				(desc.cudaContextManager),
	mContactReportsNeedPostSolverVelocity(false),
	mUseGpuDynamics(false),
	mUseGpuBp						(false),
	mCCDBp							(false),
	mSimulationStage				(SimulationStage::eCOMPLETE),
	mTmpConstraintGroupRootBuffer	(NULL),
	mPosePreviewBodies				("scenePosePreviewBodies"),
	mOverlapFilterTaskHead			(NULL),
	mIsCollisionPhaseActive			(false),
	mOnSleepingStateChanged			(NULL)
{
	for(PxU32 type = 0; type < InteractionType::eTRACKED_IN_SCENE_COUNT; ++type)
		mInteractions[type].reserve(64);

	for (int i=0; i < InteractionType::eTRACKED_IN_SCENE_COUNT; ++i)
		mActiveInteractionCount[i] = 0;

	mStats						= PX_NEW(SimStats);
	mConstraintIDTracker		= PX_NEW(ObjectIDTracker);
	mActorIDTracker				= PX_NEW(ObjectIDTracker);
	mElementIDPool				= PX_NEW(ObjectIDTracker);

	mTriggerBufferExtraData		= reinterpret_cast<TriggerBufferExtraData*>(PX_ALLOC(sizeof(TriggerBufferExtraData), "ScScene::TriggerBufferExtraData"));
	PX_PLACEMENT_NEW(mTriggerBufferExtraData, TriggerBufferExtraData("ScScene::TriggerPairExtraData"));

	mStaticSimPool				= PX_NEW(PreallocatingPool<StaticSim>)(64, "StaticSim");
	mBodySimPool				= PX_NEW(PreallocatingPool<BodySim>)(64, "BodySim");
	mShapeSimPool				= PX_NEW(PreallocatingPool<ShapeSim>)(64, "ShapeSim");
	mConstraintSimPool			= PX_NEW(PxPool<ConstraintSim>)("ScScene::ConstraintSim");
	mConstraintInteractionPool	= PX_NEW(PxPool<ConstraintInteraction>)("ScScene::ConstraintInteraction");
	mLLArticulationRCPool		= PX_NEW(LLArticulationRCPool);
#if PX_SUPPORT_GPU_PHYSX
	mLLSoftBodyPool				= PX_NEW(LLSoftBodyPool);
	mLLFEMClothPool             = PX_NEW(LLFEMClothPool);
	mLLParticleSystemPool		= PX_NEW(LLParticleSystemPool);
	mLLHairSystemPool			= PX_NEW(LLHairSystemPool);
#endif
	mSimStateDataPool			= PX_NEW(PxPool<SimStateData>)("ScScene::SimStateData");

	mProjectionManager			= PX_NEW(ConstraintProjectionManager)();

	mSqBoundsManager			= PX_NEW(SqBoundsManager);

	mTaskManager				= physx::PxTaskManager::createTaskManager(*PxGetErrorCallback(), desc.cpuDispatcher);

	for(PxU32 i=0; i<PxGeometryType::eGEOMETRY_COUNT; i++)
		mNbGeometries[i] = 0;

	bool useGpuDynamics = false;
	bool useGpuBroadphase = false;

#if PX_SUPPORT_GPU_PHYSX
	if(desc.flags & PxSceneFlag::eENABLE_GPU_DYNAMICS)
	{
		if(!mCudaContextManager)
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "GPU solver pipeline failed, switching to software");
		else if(mCudaContextManager->supportsArchSM30())
			useGpuDynamics = true;
	}

	if(desc.broadPhaseType == PxBroadPhaseType::eGPU)
	{
		if(!mCudaContextManager)
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "GPU Bp pipeline failed, switching to software");
		else if(mCudaContextManager->supportsArchSM30())
			useGpuBroadphase = true;
	}
#endif

	mUseGpuDynamics = useGpuDynamics;
	mUseGpuBp = useGpuBroadphase;

	mLLContext = PX_NEW(PxsContext)(desc, mTaskManager, mTaskPool, mCudaContextManager, desc.contactPairSlabSize, contextID);
	
	if (mLLContext == 0)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Failed to create context!");
		return;
	}
	mLLContext->setMaterialManager(&getMaterialManager());

#if PX_SUPPORT_GPU_PHYSX
	if (useGpuBroadphase || useGpuDynamics)
	{
		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);

		// PT: this creates a PxgMemoryManager, whose host memory allocator is a PxgCudaHostMemoryAllocatorCallback
		mMemoryManager = physxGpu->createGpuMemoryManager(mLLContext->getCudaContextManager());
		mGpuWranglerManagers = physxGpu->createGpuKernelWranglerManager(mLLContext->getCudaContextManager(), *PxGetErrorCallback(), desc.gpuComputeVersion);
		// PT: this creates a PxgHeapMemoryAllocatorManager
		mHeapMemoryAllocationManager = physxGpu->createGpuHeapMemoryAllocatorManager(desc.gpuDynamicsConfig.heapCapacity, mMemoryManager, desc.gpuComputeVersion);
	}
	else
#endif
	{
		// PT: this creates a PxsDefaultMemoryManager
		mMemoryManager = createDefaultMemoryManager();
	}

	Bp::BroadPhase* broadPhase = NULL;

	//Note: broadphase should be independent of AABBManager.  MBP uses it to call getBPBounds but it has 
	//already been passed all bounds in BroadPhase::update() so should use that instead.
	// PT: above comment is obsolete: MBP now doesn't call getBPBounds anymore (except in commented out code)
	// and it is instead the GPU broadphase which is not independent from the GPU AABB manager.......
	if(!useGpuBroadphase)
	{
		PxBroadPhaseType::Enum broadPhaseType = desc.broadPhaseType;

		if (broadPhaseType == PxBroadPhaseType::eGPU)
			broadPhaseType = PxBroadPhaseType::eABP;

		broadPhase = Bp::BroadPhase::create(
			broadPhaseType, 
			desc.limits.maxNbRegions, 
			desc.limits.maxNbBroadPhaseOverlaps, 
			desc.limits.maxNbStaticShapes, 
			desc.limits.maxNbDynamicShapes,
			contextID);
	}
#if PX_SUPPORT_GPU_PHYSX
	else
	{
		broadPhase = PxvGetPhysXGpu(true)->createGpuBroadPhase(	mGpuWranglerManagers, mLLContext->getCudaContextManager(),
																desc.gpuComputeVersion, desc.gpuDynamicsConfig,
																mHeapMemoryAllocationManager, contextID);
	}
#endif

	//create allocator
	PxVirtualAllocatorCallback* allocatorCallback = mMemoryManager->getHostMemoryAllocator();
	PxVirtualAllocator allocator(allocatorCallback);

	mBoundsArray = PX_NEW(Bp::BoundsArray)(allocator);
	mContactDistance = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxFloatArrayPinned), "ContactDistance"), PxFloatArrayPinned)(allocator);
	mHasContactDistanceChanged = false;

	const bool useEnhancedDeterminism = mPublicFlags & PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;

	mSimpleIslandManager = PX_NEW(IG::SimpleIslandManager)(useEnhancedDeterminism, contextID);

	if (!useGpuDynamics)
	{
		if (desc.solverType == PxSolverType::ePGS)
		{
			mDynamicsContext = createDynamicsContext
			(&mLLContext->getNpMemBlockPool(), mLLContext->getScratchAllocator(),
				mLLContext->getTaskPool(), mLLContext->getSimStats(), &mLLContext->getTaskManager(), allocatorCallback, &getMaterialManager(),
				mSimpleIslandManager, contextID, mEnableStabilization, useEnhancedDeterminism, desc.maxBiasCoefficient,
				!!(desc.flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION), desc.getTolerancesScale().length);
		}
		else
		{
			mDynamicsContext = createTGSDynamicsContext
			(&mLLContext->getNpMemBlockPool(), mLLContext->getScratchAllocator(),
				mLLContext->getTaskPool(), mLLContext->getSimStats(), &mLLContext->getTaskManager(), allocatorCallback, &getMaterialManager(),
				mSimpleIslandManager, contextID, mEnableStabilization, useEnhancedDeterminism,
				desc.getTolerancesScale().length);
		}

		mLLContext->setNphaseImplementationContext(createNphaseImplementationContext(*mLLContext, &mSimpleIslandManager->getAccurateIslandSim(), allocatorCallback));

		mSimulationControllerCallback = PX_NEW(ScSimulationControllerCallback)(this);
		mSimulationController = PX_NEW(SimulationController)(mSimulationControllerCallback);

		if (!useGpuBroadphase)
			mAABBManager = createAABBManagerCPU(desc, broadPhase, mBoundsArray, mContactDistance, allocator, contextID);
#if PX_SUPPORT_GPU_PHYSX
		else
			mAABBManager = createAABBManagerGPU(mGpuWranglerManagers, mLLContext->getCudaContextManager(), mHeapMemoryAllocationManager, desc, broadPhase, mBoundsArray, mContactDistance, allocator, contextID);
#endif
	}
	else
	{
#if PX_SUPPORT_GPU_PHYSX
		mDynamicsContext = PxvGetPhysXGpu(true)->createGpuDynamicsContext(mLLContext->getTaskPool(), mGpuWranglerManagers, mLLContext->getCudaContextManager(),
			desc.gpuDynamicsConfig, mSimpleIslandManager, desc.gpuMaxNumPartitions, desc.gpuMaxNumStaticPartitions, mEnableStabilization, useEnhancedDeterminism, desc.maxBiasCoefficient, desc.gpuComputeVersion, mLLContext->getSimStats(),
			mHeapMemoryAllocationManager, !!(desc.flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION), desc.solverType, desc.getTolerancesScale().length);

		void* contactStreamBase = NULL;
		void* patchStreamBase = NULL;
		void* forceAndIndiceStreamBase = NULL;

		mDynamicsContext->getDataStreamBase(contactStreamBase, patchStreamBase, forceAndIndiceStreamBase);

		PxvNphaseImplementationContextUsableAsFallback* cpuNphaseImplementation = createNphaseImplementationContext(*mLLContext, &mSimpleIslandManager->getAccurateIslandSim(), allocatorCallback);
		mLLContext->setNphaseFallbackImplementationContext(cpuNphaseImplementation);

		PxvNphaseImplementationContext* gpuNphaseImplementation = PxvGetPhysXGpu(true)->createGpuNphaseImplementationContext(*mLLContext, mGpuWranglerManagers, cpuNphaseImplementation, desc.gpuDynamicsConfig, contactStreamBase, patchStreamBase,
			forceAndIndiceStreamBase, getBoundsArray().getBounds(), &mSimpleIslandManager->getAccurateIslandSim(), mDynamicsContext, desc.gpuComputeVersion, mHeapMemoryAllocationManager, useGpuBroadphase);

		mSimulationControllerCallback = PX_NEW(PxgSimulationControllerCallback)(this);

		mSimulationController = PxvGetPhysXGpu(true)->createGpuSimulationController(mGpuWranglerManagers, mLLContext->getCudaContextManager(),
			mDynamicsContext, gpuNphaseImplementation, broadPhase, useGpuBroadphase, mSimpleIslandManager, mSimulationControllerCallback, desc.gpuComputeVersion, mHeapMemoryAllocationManager,
			desc.gpuDynamicsConfig.maxSoftBodyContacts, desc.gpuDynamicsConfig.maxFemClothContacts, desc.gpuDynamicsConfig.maxParticleContacts, desc.gpuDynamicsConfig.maxHairContacts);

		mSimulationController->setBounds(mBoundsArray);
		mDynamicsContext->setSimulationController(mSimulationController);

		mLLContext->setNphaseImplementationContext(gpuNphaseImplementation);

		mLLContext->mContactStreamPool = &mDynamicsContext->getContactStreamPool();
		mLLContext->mPatchStreamPool = &mDynamicsContext->getPatchStreamPool();
		mLLContext->mForceAndIndiceStreamPool = &mDynamicsContext->getForceStreamPool();

		// PT: TODO: what's the difference between this allocator and "allocator" above?
		PxVirtualAllocator tAllocator(mHeapMemoryAllocationManager->mMappedMemoryAllocators, PxsHeapStats::eBROADPHASE);

		if (!useGpuBroadphase)
			// PT: TODO: we're using a CUDA allocator in the CPU broadphase, and a different allocator for the bounds array?
			mAABBManager = createAABBManagerCPU(desc, broadPhase, mBoundsArray, mContactDistance, tAllocator, contextID);
		else
			mAABBManager = createAABBManagerGPU(mGpuWranglerManagers, mLLContext->getCudaContextManager(), mHeapMemoryAllocationManager, desc, broadPhase, mBoundsArray, mContactDistance, tAllocator, contextID);
#endif
	}

	bool suppressReadback = mPublicFlags & PxSceneFlag::eSUPPRESS_READBACK;
	bool forceReadback = mPublicFlags & PxSceneFlag::eFORCE_READBACK;

	if(suppressReadback && forceReadback)
		suppressReadback = false;

	mDynamicsContext->setSuppressReadback(suppressReadback);

	//Construct the bitmap of updated actors required as input to the broadphase update
	if(desc.limits.maxNbBodies)
	{
		// PT: TODO: revisit this. Why do we handle the added/removed and updated bitmaps entirely differently, in different places? And what is this weird formula here?
		mAABBManager->getChangedAABBMgActorHandleMap().resize((2*desc.limits.maxNbBodies + 256) & ~255);
	}

	//mLLContext->createTransformCache(mDynamicsContext->getAllocatorCallback());

	mLLContext->createTransformCache(*allocatorCallback);
	mLLContext->setContactDistance(mContactDistance);

	mCCDContext = PX_NEW(PxsCCDContext)(mLLContext, mDynamicsContext->getThresholdStream(), *mLLContext->getNphaseImplementationContext(), desc.ccdThreshold);
	
	setSolverBatchSize(desc.solverBatchSize);
	setSolverArticBatchSize(desc.solverArticulationBatchSize);
	mDynamicsContext->setFrictionOffsetThreshold(desc.frictionOffsetThreshold);
	mDynamicsContext->setCCDSeparationThreshold(desc.ccdMaxSeparation);
	mDynamicsContext->setCorrelationDistance(desc.frictionCorrelationDistance);

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mLLContext->setMeshContactMargin(0.01f * scale.length);
	mLLContext->setToleranceLength(scale.length);

	// the original descriptor uses 
	//    bounce iff impact velocity  > threshold
	// but LL use 
	//    bounce iff separation velocity < -threshold 
	// hence we negate here.

	mDynamicsContext->setBounceThreshold(-desc.bounceThresholdVelocity);

	StaticCore* anchorCore = PX_NEW(StaticCore)(PxTransform(PxIdentity));

	mStaticAnchor = mStaticSimPool->construct(*this, *anchorCore);

	mNPhaseCore = PX_NEW(NPhaseCore)(*this, desc);

	// Init dominance matrix
	{
		//init all dominance pairs such that:
		//if g1 == g2, then (1.0f, 1.0f) is returned
		//if g1 <  g2, then (0.0f, 1.0f) is returned
		//if g1 >  g2, then (1.0f, 0.0f) is returned

		PxU32 mask = ~PxU32(1);
		for (unsigned i = 0; i < PX_MAX_DOMINANCE_GROUP; ++i, mask <<= 1)
			mDominanceBitMatrix[i] = ~mask;
	}
		
//	DeterminismDebugger::begin();

	mWokeBodyListValid = true;
	mSleepBodyListValid = true;

	mWokeSoftBodyListValid = true;
	mSleepSoftBodyListValid = true;

	mWokeHairSystemListValid = true;
	mSleepHairSystemListValid = true;

	//load from desc:
	setLimits(desc.limits);

	// Create broad phase
	setBroadPhaseCallback(desc.broadPhaseCallback);

	setGravity(desc.gravity);

	setFrictionType(desc.frictionType);

	setPCM(desc.flags & PxSceneFlag::eENABLE_PCM);

	setContactCache(!(desc.flags & PxSceneFlag::eDISABLE_CONTACT_CACHE));
	setSimulationEventCallback(desc.simulationEventCallback);
	setContactModifyCallback(desc.contactModifyCallback);
	setCCDContactModifyCallback(desc.ccdContactModifyCallback);
	setCCDMaxPasses(desc.ccdMaxPasses);
	PX_ASSERT(mNPhaseCore); // refactor paranoia
	
	PX_ASSERT(	((desc.filterShaderData) && (desc.filterShaderDataSize > 0)) ||
				(!(desc.filterShaderData) && (desc.filterShaderDataSize == 0))	);
	if (desc.filterShaderData)
	{
		mFilterShaderData = PX_ALLOC(desc.filterShaderDataSize, sFilterShaderDataMemAllocId);
		PxMemCopy(mFilterShaderData, desc.filterShaderData, desc.filterShaderDataSize);
		mFilterShaderDataSize = desc.filterShaderDataSize;
		mFilterShaderDataCapacity = desc.filterShaderDataSize;
	}
	else
	{
		mFilterShaderData = NULL;
		mFilterShaderDataSize = 0;
		mFilterShaderDataCapacity = 0;
	}
	mFilterShader = desc.filterShader;
	mFilterCallback = desc.filterCallback;
}

void Sc::Scene::release()
{
	// TODO: PT: check virtual stuff

	mTimeStamp++;

	//collisionSpace.purgeAllPairs();

	//purgePairs();
	//releaseTagData();

	// We know release all the shapes before the collision space
	//collisionSpace.deleteAllShapes();

	//collisionSpace.release();

	//DeterminismDebugger::end();

	PX_FREE(mActiveKinematicsCopy);

	///clear broken constraint list:
	clearBrokenConstraintBuffer();

	PX_DELETE(mNPhaseCore);

	PX_FREE(mFilterShaderData);

	if (mStaticAnchor)
	{
		StaticCore& core = mStaticAnchor->getStaticCore();
		mStaticSimPool->destroy(mStaticAnchor);
		delete &core;
	}

	// Free object IDs and the deleted object id map
	postReportsCleanup();

	//before the task manager
	if (mLLContext)
	{
		if(mLLContext->getNphaseFallbackImplementationContext())
		{
			mLLContext->getNphaseFallbackImplementationContext()->destroy();
			mLLContext->setNphaseFallbackImplementationContext(NULL);
		}

		if(mLLContext->getNphaseImplementationContext())
		{
			mLLContext->getNphaseImplementationContext()->destroy();
			mLLContext->setNphaseImplementationContext(NULL);
		}
	}

	PX_DELETE(mProjectionManager);
	PX_DELETE(mSqBoundsManager);
	PX_DELETE(mBoundsArray);

	PX_DELETE(mConstraintInteractionPool);
	PX_DELETE(mConstraintSimPool);
	PX_DELETE(mSimStateDataPool);
	PX_DELETE(mStaticSimPool);
	PX_DELETE(mShapeSimPool);
	PX_DELETE(mBodySimPool);
	PX_DELETE(mLLArticulationRCPool);
#if PX_SUPPORT_GPU_PHYSX
	PX_DELETE(mLLSoftBodyPool);
	PX_DELETE(mLLFEMClothPool);
	PX_DELETE(mLLParticleSystemPool);
	PX_DELETE(mLLHairSystemPool);
#endif
	mTriggerBufferExtraData->~TriggerBufferExtraData();
	PX_FREE(mTriggerBufferExtraData);

	PX_DELETE(mElementIDPool);
	PX_DELETE(mActorIDTracker);
	PX_DELETE(mConstraintIDTracker);
	PX_DELETE(mStats);

	Bp::BroadPhase* broadPhase = mAABBManager->getBroadPhase();
	mAABBManager->destroy();
	PX_RELEASE(broadPhase);

	PX_DELETE(mSimulationControllerCallback);
	PX_DELETE(mSimulationController);

	mDynamicsContext->destroy();

	PX_DELETE(mCCDContext);

	PX_DELETE(mSimpleIslandManager);

#if PX_SUPPORT_GPU_PHYSX
	PX_DELETE(mGpuWranglerManagers);
	PX_DELETE(mHeapMemoryAllocationManager);
#endif

	PX_RELEASE(mTaskManager);
	PX_DELETE(mLLContext);

	mContactDistance->~PxArray();
	PX_FREE(mContactDistance);

	PX_DELETE(mMemoryManager);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::preAllocate(PxU32 nbStatics, PxU32 nbBodies, PxU32 nbStaticShapes, PxU32 nbDynamicShapes)
{
	// PT: TODO: this is only used for my addActors benchmark for now. Pre-allocate more arrays here.

	mActiveBodies.reserve(PxMax<PxU32>(64,nbBodies));

	mStaticSimPool->preAllocate(nbStatics);

	mBodySimPool->preAllocate(nbBodies);

	mShapeSimPool->preAllocate(nbStaticShapes + nbDynamicShapes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::addDirtyArticulationSim(Sc::ArticulationSim* artiSim)
{
	artiSim->setDirtyFlag(ArticulationSimDirtyFlag::eUPDATE);
	mDirtyArticulationSims.insert(artiSim);
}

void Sc::Scene::removeDirtyArticulationSim(Sc::ArticulationSim* artiSim)
{
	artiSim->setDirtyFlag(ArticulationSimDirtyFlag::eNONE);
	mDirtyArticulationSims.erase(artiSim);
}

void Sc::Scene::addToActiveList(ActorSim& actorSim)
{
	PX_ASSERT(actorSim.getActiveListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);

	ActorCore* appendedActorCore = &actorSim.getActorCore();

	if (actorSim.isDynamicRigid())
	{
		// Sort: kinematic before dynamic
		const PxU32 size = mActiveBodies.size();
		PxU32 incomingBodyActiveListIndex = size;							// PT: by default we append the incoming body at the end of the current array.

		BodySim& bodySim = static_cast<BodySim&>(actorSim);
		if (bodySim.isKinematic())											// PT: Except if incoming body is kinematic, in which case:
		{
			const PxU32 nbKinematics = mActiveKinematicBodyCount++;			// PT: - we increase their number
			if (nbKinematics != size)										// PT: - if there's at least one dynamic in the array...
			{
				PX_ASSERT(appendedActorCore != mActiveBodies[nbKinematics]);
				appendedActorCore = mActiveBodies[nbKinematics];			// PT: ...then we grab the first dynamic after the kinematics...
				appendedActorCore->getSim()->setActiveListIndex(size);		// PT: ...and we move that one back to the end of the array...

				mActiveBodies[nbKinematics] = static_cast<BodyCore*>(&actorSim.getActorCore());			// PT: ...while the incoming kine replaces the dynamic we moved out.
				incomingBodyActiveListIndex = nbKinematics;					// PT: ...thus the incoming kine's index is the prev #kines.
			}
		}
		
		// for active compound rigids add to separate array, so we dont have to traverse all active actors
		if (bodySim.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
		{
			PX_ASSERT(actorSim.getActiveCompoundListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);
			const PxU32 compoundIndex = mActiveCompoundBodies.size();
			mActiveCompoundBodies.pushBack(static_cast<BodyCore*>(appendedActorCore));
			actorSim.setActiveCompoundListIndex(compoundIndex);
		}

		actorSim.setActiveListIndex(incomingBodyActiveListIndex);			// PT: will be 'size' or 'nbKinematics', 'dynamicIndex'
		mActiveBodies.pushBack(static_cast<BodyCore*>(appendedActorCore));	// PT: will be the incoming object or the first dynamic we moved out.
	}
#if PX_SUPPORT_GPU_PHYSX
	else if (actorSim.isSoftBody())
	{
		PxU32 activeListIndex = mActiveSoftBodies.size();
		actorSim.setActiveListIndex(activeListIndex);
		mActiveSoftBodies.pushBack(static_cast<SoftBodyCore*>(appendedActorCore));
	}
	else if (actorSim.isFEMCloth())
	{
		PxU32 activeListIndex = mActiveFEMCloths.size();
		actorSim.setActiveListIndex(activeListIndex);
		mActiveFEMCloths.pushBack(static_cast<FEMClothCore*>(appendedActorCore));
	}
	else if (actorSim.isParticleSystem())
	{
		PxU32 activeListIndex = mActiveParticleSystems.size();
		actorSim.setActiveListIndex(activeListIndex);
		mActiveParticleSystems.pushBack(static_cast<ParticleSystemCore*>(appendedActorCore));
	}
	else if (actorSim.isHairSystem())
	{
		PxU32 activeListIndex = mActiveHairSystems.size();
		actorSim.setActiveListIndex(activeListIndex);
		mActiveHairSystems.pushBack(static_cast<HairSystemCore*>(appendedActorCore));
	}
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void removeFromActiveCompoundBodyList(Sc::ActorSim& actorSim, PxArray<Sc::BodyCore*>& activeCompoundBodies)
{
	const PxU32 removedCompoundIndex = actorSim.getActiveCompoundListIndex();
	PX_ASSERT(removedCompoundIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
	actorSim.setActiveCompoundListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

	const PxU32 newCompoundSize = activeCompoundBodies.size() - 1;

	if(removedCompoundIndex != newCompoundSize)
	{
		Sc::BodyCore* lastBody = activeCompoundBodies[newCompoundSize];
		activeCompoundBodies[removedCompoundIndex] = lastBody;
		lastBody->getSim()->setActiveCompoundListIndex(removedCompoundIndex);
	}
	activeCompoundBodies.forceSize_Unsafe(newCompoundSize);
}

void Sc::Scene::removeFromActiveCompoundBodyList(BodySim& body)
{
	::removeFromActiveCompoundBodyList(body, mActiveCompoundBodies);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::removeFromActiveList(ActorSim& actorSim)
{
	PxU32 removedActiveIndex = actorSim.getActiveListIndex();
	PX_ASSERT(removedActiveIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
	actorSim.setActiveListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

	if (actorSim.isDynamicRigid())
	{
		PX_ASSERT(mActiveBodies[removedActiveIndex] == &actorSim.getActorCore());

		const PxU32 newSize = mActiveBodies.size() - 1;

		BodySim& bodySim = static_cast<BodySim&>(actorSim);

		// Sort: kinematic before dynamic,
		if (removedActiveIndex < mActiveKinematicBodyCount)	// PT: same as 'body.isKinematic()' but without accessing the Core data
		{
			PX_ASSERT(mActiveKinematicBodyCount);
			PX_ASSERT(bodySim.isKinematic());
			const PxU32 swapIndex = --mActiveKinematicBodyCount;
			if (newSize != swapIndex				// PT: equal if the array only contains kinematics
				&& removedActiveIndex < swapIndex)	// PT: i.e. "if we don't remove the last kinematic"
			{
				BodyCore* swapBody = mActiveBodies[swapIndex];
				swapBody->getSim()->setActiveListIndex(removedActiveIndex);
				mActiveBodies[removedActiveIndex] = swapBody;
				removedActiveIndex = swapIndex;
			}
		}

		// for active compound rigids add to separate array, so we dont have to traverse all active actors
		// A.B. TODO we should handle kinematic switch, no need to hold kinematic rigids in compound list
		if(bodySim.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
			::removeFromActiveCompoundBodyList(actorSim, mActiveCompoundBodies);

		if (removedActiveIndex != newSize)
		{
			Sc::BodyCore* lastBody = mActiveBodies[newSize];
			mActiveBodies[removedActiveIndex] = lastBody;
			lastBody->getSim()->setActiveListIndex(removedActiveIndex);
		}
		mActiveBodies.forceSize_Unsafe(newSize);
	}
#if PX_SUPPORT_GPU_PHYSX
	else if(actorSim.isSoftBody())
	{
		const PxU32 newSoftBodySize = mActiveSoftBodies.size() - 1;

		if (removedActiveIndex != newSoftBodySize)
		{
			Sc::SoftBodyCore* lastBody = mActiveSoftBodies[newSoftBodySize];
			mActiveSoftBodies[removedActiveIndex] = lastBody;
			lastBody->getSim()->setActiveListIndex(removedActiveIndex);
		}
		mActiveSoftBodies.forceSize_Unsafe(newSoftBodySize);
	}
	else if (actorSim.isFEMCloth())
	{
		const PxU32 newFEMClothSize = mActiveFEMCloths.size() - 1;

		if (removedActiveIndex != newFEMClothSize)
		{
			Sc::FEMClothCore* lastBody = mActiveFEMCloths[newFEMClothSize];
			mActiveFEMCloths[removedActiveIndex] = lastBody;
			lastBody->getSim()->setActiveListIndex(removedActiveIndex);
		}
		mActiveFEMCloths.forceSize_Unsafe(newFEMClothSize);
	}
	else if (actorSim.isParticleSystem())
	{
		
		const PxU32 newParticleSystemSize = mActiveParticleSystems.size() - 1;

		if (removedActiveIndex != newParticleSystemSize)
		{
			Sc::ParticleSystemCore* lastBody = mActiveParticleSystems[newParticleSystemSize];
			mActiveParticleSystems[removedActiveIndex] = lastBody;
			lastBody->getSim()->setActiveListIndex(removedActiveIndex);
		}
		mActiveParticleSystems.forceSize_Unsafe(newParticleSystemSize);
		
	}
	else if (actorSim.isHairSystem())
	{
		const PxU32 newHairSystemSize = mActiveHairSystems.size() - 1;

		if (removedActiveIndex != newHairSystemSize)
		{
			Sc::HairSystemCore* lastHairSystem = mActiveHairSystems[newHairSystemSize];
			mActiveHairSystems[removedActiveIndex] = lastHairSystem;
			lastHairSystem->getSim()->setActiveListIndex(removedActiveIndex);
		}
		mActiveHairSystems.forceSize_Unsafe(newHairSystemSize);
	}
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::swapInActiveBodyList(BodySim& body)
{
	PX_ASSERT(!body.isStaticRigid() && !body.isSoftBody() && !body.isFEMCloth() && !body.isParticleSystem() && !body.isHairSystem());
	const PxU32 activeListIndex = body.getActiveListIndex();
	PX_ASSERT(activeListIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);

	PxU32 swapIndex;
	PxU32 newActiveKinematicBodyCount;
	if(activeListIndex < mActiveKinematicBodyCount)
	{
		// kinematic -> dynamic
		PX_ASSERT(!body.isKinematic());  // the corresponding flag gets switched before this call
		PX_ASSERT(mActiveKinematicBodyCount > 0);  // there has to be at least one kinematic body

		swapIndex = mActiveKinematicBodyCount - 1;
		newActiveKinematicBodyCount = swapIndex;
	}
	else
	{
		// dynamic -> kinematic
		PX_ASSERT(body.isKinematic());  // the corresponding flag gets switched before this call
		PX_ASSERT(mActiveKinematicBodyCount < mActiveBodies.size());  // there has to be at least one dynamic body

		swapIndex = mActiveKinematicBodyCount;
		newActiveKinematicBodyCount = swapIndex + 1;
	}

	BodyCore*& swapBodyRef = mActiveBodies[swapIndex];
	body.setActiveListIndex(swapIndex);
	BodyCore* swapBody = swapBodyRef;
	swapBodyRef = &body.getBodyCore();

	swapBody->getSim()->setActiveListIndex(activeListIndex);
	mActiveBodies[activeListIndex] = swapBody;

	mActiveKinematicBodyCount = newActiveKinematicBodyCount;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::registerInteraction(Interaction* interaction, bool active)
{
	const InteractionType::Enum type = interaction->getType();
	const PxU32 sceneArrayIndex = mInteractions[type].size();
	interaction->setInteractionId(sceneArrayIndex);

	mInteractions[type].pushBack(interaction);
	if (active)
	{
		if (sceneArrayIndex > mActiveInteractionCount[type])
			swapInteractionArrayIndices(sceneArrayIndex, mActiveInteractionCount[type], type);
		mActiveInteractionCount[type]++;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::unregisterInteraction(Interaction* interaction)
{
	const InteractionType::Enum type = interaction->getType();
	const PxU32 sceneArrayIndex = interaction->getInteractionId();
	PX_ASSERT(sceneArrayIndex != PX_INVALID_INTERACTION_SCENE_ID);
//	if(sceneArrayIndex==PX_INVALID_INTERACTION_SCENE_ID)
//		return;
	mInteractions[type].replaceWithLast(sceneArrayIndex);
	interaction->setInteractionId(PX_INVALID_INTERACTION_SCENE_ID);
	if (sceneArrayIndex<mInteractions[type].size()) // The removed interaction was the last one, do not reset its sceneArrayIndex
		mInteractions[type][sceneArrayIndex]->setInteractionId(sceneArrayIndex);
	if (sceneArrayIndex<mActiveInteractionCount[type])
	{
		mActiveInteractionCount[type]--;
		if (mActiveInteractionCount[type]<mInteractions[type].size())
			swapInteractionArrayIndices(sceneArrayIndex, mActiveInteractionCount[type], type);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::swapInteractionArrayIndices(PxU32 id1, PxU32 id2, InteractionType::Enum type)
{
	PxArray<Interaction*>& interArray = mInteractions[type];
	Interaction* interaction1 = interArray[id1];
	Interaction* interaction2 = interArray[id2];
	interArray[id1] = interaction2;
	interArray[id2] = interaction1;
	interaction1->setInteractionId(id2);
	interaction2->setInteractionId(id1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxReal Sc::Scene::getLengthScale() const
{
	return mDynamicsContext->getLengthScale();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::notifyInteractionActivated(Interaction* interaction)
{
	PX_ASSERT((interaction->getType() == InteractionType::eOVERLAP) || (interaction->getType() == InteractionType::eTRIGGER));
	PX_ASSERT(interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_ASSERT(interaction->getInteractionId() != PX_INVALID_INTERACTION_SCENE_ID);

	const InteractionType::Enum type = interaction->getType();

	PX_ASSERT(interaction->getInteractionId() >= mActiveInteractionCount[type]);
	
	if (mActiveInteractionCount[type] < mInteractions[type].size())
		swapInteractionArrayIndices(mActiveInteractionCount[type], interaction->getInteractionId(), type);
	mActiveInteractionCount[type]++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::notifyInteractionDeactivated(Interaction* interaction)
{
	PX_ASSERT((interaction->getType() == InteractionType::eOVERLAP) || (interaction->getType() == InteractionType::eTRIGGER));
	PX_ASSERT(!interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_ASSERT(interaction->getInteractionId() != PX_INVALID_INTERACTION_SCENE_ID);

	const InteractionType::Enum type = interaction->getType();
	PX_ASSERT(interaction->getInteractionId() < mActiveInteractionCount[type]);

	if (mActiveInteractionCount[type] > 1)
		swapInteractionArrayIndices(mActiveInteractionCount[type]-1, interaction->getInteractionId(), type);
	mActiveInteractionCount[type]--;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void** Sc::Scene::allocatePointerBlock(PxU32 size)
{
	PX_ASSERT(size>32 || size == 32 || size == 16 || size == 8);
	void* ptr;
	if(size==8)
		ptr = mPointerBlock8Pool.construct();
	else if(size == 16)
		ptr = mPointerBlock16Pool.construct();
	else if(size == 32)
		ptr = mPointerBlock32Pool.construct();
	else
		ptr = PX_ALLOC(size * sizeof(void*), "void*");

	return reinterpret_cast<void**>(ptr);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::deallocatePointerBlock(void** block, PxU32 size)
{
	PX_ASSERT(size>32 || size == 32 || size == 16 || size == 8);
	if(size==8)
		mPointerBlock8Pool.destroy(reinterpret_cast<PointerBlock8*>(block));
	else if(size == 16)
		mPointerBlock16Pool.destroy(reinterpret_cast<PointerBlock16*>(block));
	else if(size == 32)
		mPointerBlock32Pool.destroy(reinterpret_cast<PointerBlock32*>(block));
	else
		PX_FREE(block);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::setFilterShaderData(const void* data, PxU32 dataSize)
{
	PX_UNUSED(sFilterShaderDataMemAllocId);

	if (data)
	{
		PX_ASSERT(dataSize > 0);

		void* buffer;

		if (dataSize <= mFilterShaderDataCapacity)
			buffer = mFilterShaderData;
		else
		{
			buffer = PX_ALLOC(dataSize, sFilterShaderDataMemAllocId);
			if (buffer)
			{
				mFilterShaderDataCapacity = dataSize;
				PX_FREE(mFilterShaderData);
			}
			else
			{
				outputError<PxErrorCode::eOUT_OF_MEMORY>(__LINE__, "Failed to allocate memory for filter shader data!");
				return;
			}
		}

		PxMemCopy(buffer, data, dataSize);
		mFilterShaderData = buffer;
		mFilterShaderDataSize = dataSize;
	}
	else
	{
		PX_ASSERT(dataSize == 0);

		PX_FREE(mFilterShaderData);
		mFilterShaderDataSize = 0;
		mFilterShaderDataCapacity = 0;
	}
}

void Sc::Scene::setPublicFlags(PxSceneFlags flags)
{
	mPublicFlags = flags;
	mDynamicsContext->setSuppressReadback(flags & PxSceneFlag::eSUPPRESS_READBACK);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::prepareCollide()
{
	mReportShapePairTimeStamp++;	// deleted actors/shapes should get separate pair entries in contact reports
	mContactReportsNeedPostSolverVelocity = false;

	getRenderBuffer().clear();

	// Clear broken constraint list:
	clearBrokenConstraintBuffer();

	// Update from visualization parameters
	if(mVisualizationParameterChanged)
	{
		mVisualizationParameterChanged = false;

		// Update SIPs if visualization is enabled
		if(	getVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT) || getVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL) || 
			getVisualizationParameter(PxVisualizationParameter::eCONTACT_ERROR) || getVisualizationParameter(PxVisualizationParameter::eCONTACT_FORCE))
			mInternalFlags |= SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION;
	}

	visualizeStartStep();
	
	PxcClearContactCacheStats();
}

void Sc::Scene::simulate(PxReal timeStep, PxBaseTask* continuation)
{
	if(timeStep != 0.0f)
	{
		mDt = timeStep;
		mOneOverDt = 0.0f < mDt ? 1.0f/mDt : 0.0f;
		mDynamicsContext->setDt(mDt);

		mAdvanceStep.setContinuation(continuation);

		prepareCollide();
		stepSetupCollide(&mAdvanceStep); 
		
		mCollideStep.setContinuation(&mAdvanceStep);

		mAdvanceStep.removeReference();
		mCollideStep.removeReference();
	}
}

void Sc::Scene::advance(PxReal timeStep, PxBaseTask* continuation)
{
	if(timeStep != 0.0f)
	{
		mDt = timeStep;
		mOneOverDt = 0.0f < mDt ? 1.0f/mDt : 0.0f;

		mAdvanceStep.setContinuation(continuation);

		stepSetupSolve(&mAdvanceStep);		
		
		mAdvanceStep.removeReference();
	}
}

void Sc::Scene::setBounceThresholdVelocity(const PxReal t)
{
	mDynamicsContext->setBounceThreshold(-t);
}

PxReal Sc::Scene::getBounceThresholdVelocity() const
{
	return -mDynamicsContext->getBounceThreshold();
}

void Sc::Scene::collide(PxReal timeStep, PxBaseTask* continuation)
{
	mDt = timeStep;

	prepareCollide();
	stepSetupCollide(continuation);

	mLLContext->beginUpdate();

	mCollideStep.setContinuation(continuation);
	mCollideStep.removeReference();
}

PxSolverType::Enum Sc::Scene::getSolverType() const
{
	return mDynamicsContext->getSolverType();
}

void Sc::Scene::setFrictionType(PxFrictionType::Enum model)
{
	mDynamicsContext->setFrictionType(model);
}

PxFrictionType::Enum Sc::Scene::getFrictionType() const
{
	return mDynamicsContext->getFrictionType();
}

void Sc::Scene::endSimulation()
{
	// Handle user contact filtering
	// Note: Do this before the contact callbacks get fired since the filter callback might
	//       trigger contact reports (touch lost due to re-filtering)

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	mNPhaseCore->fireCustomFilteringCallbacks(outputs);

	mNPhaseCore->preparePersistentContactEventListForNextFrame();

	mSimulationController->releaseDeferredArticulationIds();

#if PX_SUPPORT_GPU_PHYSX
	mSimulationController->releaseDeferredSoftBodyIds();

	mSimulationController->releaseDeferredFEMClothIds();

	mSimulationController->releaseDeferredParticleSystemIds();

	mSimulationController->releaseDeferredHairSystemIds();

	mAABBManager->releaseDeferredAggregateIds();
#endif

	endStep();	// - Update time stamps

	PxcDisplayContactCacheStats();
}

void Sc::Scene::flush(bool sendPendingReports)
{
	if (sendPendingReports)
	{
		fireQueuedContactCallbacks();
		mNPhaseCore->clearContactReportStream();
		mNPhaseCore->clearContactReportActorPairs(true);

		fireTriggerCallbacks();
	}
	else
	{
		mNPhaseCore->clearContactReportActorPairs(true);  // To clear the actor pair set
	}
	postReportsCleanup();
	mNPhaseCore->freeContactReportStreamMemory();

	mTriggerBufferAPI.reset();
	mTriggerBufferExtraData->reset();

	clearBrokenConstraintBuffer();
	mBrokenConstraints.reset();

	clearSleepWakeBodies();  //!!! If we send out these reports on flush then this would not be necessary

	mActorIDTracker->reset();
	mElementIDPool->reset();

	processLostTouchPairs();  // Processes the lost touch bodies
	PX_ASSERT(mLostTouchPairs.size() == 0);
	mLostTouchPairs.reset();
	// Does not seem worth deleting the bitmap for the lost touch pair list

	mActiveBodies.shrink();

	for(PxU32 i=0; i < InteractionType::eTRACKED_IN_SCENE_COUNT; i++)
	{
		mInteractions[i].shrink();
	}

	//!!! TODO: look into retrieving memory from the NPhaseCore & Broadphase class (all the pools in there etc.)

	mLLContext->getNpMemBlockPool().releaseUnusedBlocks();
}

// User callbacks

void Sc::Scene::setSimulationEventCallback(PxSimulationEventCallback* callback)
{
	if(!mSimulationEventCallback && callback)
	{
		// if there was no callback before, the sleeping bodies have to be prepared for potential notification events (no shortcut possible anymore)
		BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
		for (PxU32 i = 0; i < mSleepBodies.size(); i++)
		{
			sleepingBodies[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		}

#if PX_SUPPORT_GPU_PHYSX
		SoftBodyCore* const* sleepingSoftBodies = mSleepSoftBodies.getEntries();
		for (PxU32 i = 0; i < mSleepSoftBodies.size(); i++)
		{
			sleepingSoftBodies[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		}

		//FEMClothCore* const* sleepingFEMCloths = mSleepFEMCloths.getEntries();
		//for (PxU32 i = 0; i < mSleepFEMCloths.size(); i++)
		//{
		//	sleepingFEMCloths[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		//}

		HairSystemCore* const* sleepingHairSystems = mSleepHairSystems.getEntries();
		for (PxU32 i = 0; i < mSleepHairSystems.size(); i++)
		{
			sleepingHairSystems[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		}
#endif
	}

	mSimulationEventCallback = callback;
}

PxSimulationEventCallback* Sc::Scene::getSimulationEventCallback() const
{
	return mSimulationEventCallback;
}

//CCD
void Sc::Scene::setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)
{
	mCCDContext->setCCDContactModifyCallback(callback);
}

//CCD
PxCCDContactModifyCallback* Sc::Scene::getCCDContactModifyCallback() const
{
	return mCCDContext->getCCDContactModifyCallback();
}

//CCD
void Sc::Scene::setCCDMaxPasses(PxU32 ccdMaxPasses)
{
	mCCDContext->setCCDMaxPasses(ccdMaxPasses);
}

//CCD
PxU32 Sc::Scene::getCCDMaxPasses() const
{
	return mCCDContext->getCCDMaxPasses();
}

//CCD
void Sc::Scene::setCCDThreshold(PxReal t)
{
	mCCDContext->setCCDThreshold(t);
}

//CCD
PxReal Sc::Scene::getCCDThreshold() const
{
	return mCCDContext->getCCDThreshold();
}

void Sc::Scene::removeBody(BodySim& body)	//this also notifies any connected joints!
{
	ConstraintGroupNode* node = body.getConstraintGroup();
	if (node)
	{
		//invalidate the constraint group:
		//this adds all constraints of the group to the dirty list such that groups get re-generated next frame
		getProjectionManager().invalidateGroup(*node, NULL);
	}

	BodyCore& core = body.getBodyCore();

	// Remove from sleepBodies array
	mSleepBodies.erase(&core);
	PX_ASSERT(!mSleepBodies.contains(&core));

	// Remove from wokeBodies array
	mWokeBodies.erase(&core);
	PX_ASSERT(!mWokeBodies.contains(&core));

	if (body.isActive() && (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW))
		removeFromPosePreviewList(body);
	else
		PX_ASSERT(!isInPosePreviewList(body));

	markReleasedBodyIDForLostTouch(body.getActorID());
}

void Sc::Scene::addConstraint(ConstraintCore& constraint, RigidCore* body0, RigidCore* body1)
{
	ConstraintSim* sim = mConstraintSimPool->construct(constraint, body0, body1, *this);
	PX_UNUSED(sim);

	PxNodeIndex nodeIndex0, nodeIndex1;

	ActorSim* sim0 = NULL;
	ActorSim* sim1 = NULL;

	if (body0)
	{
		sim0 = body0->getSim();
		nodeIndex0 = sim0->getNodeIndex();
	}
	if (body1)
	{
		sim1 = body1->getSim();
		nodeIndex1 = sim1->getNodeIndex();
	}

	if (nodeIndex1 < nodeIndex0)
		PxSwap(sim0, sim1);

	mConstraintMap.insert(PxPair<const Sc::ActorSim*, const Sc::ActorSim*>(sim0, sim1), &constraint);

	mConstraints.insert(&constraint);
}

void Sc::Scene::removeConstraint(ConstraintCore& constraint)
{
	ConstraintSim* cSim = constraint.getSim();

	if (cSim)
	{
		{
			PxNodeIndex nodeIndex0, nodeIndex1;

			const ConstraintInteraction* interaction = cSim->getInteraction();

			Sc::ActorSim* bSim = &interaction->getActorSim0();
			Sc::ActorSim* bSim1 = &interaction->getActorSim1();

			if (bSim)
				nodeIndex0 = bSim->getNodeIndex();
			if (bSim1)
				nodeIndex1 = bSim1->getNodeIndex();

			if (nodeIndex1 < nodeIndex0)
				PxSwap(bSim, bSim1);

			mConstraintMap.erase(PxPair<const Sc::ActorSim*, const Sc::ActorSim*>(bSim, bSim1));
		}

		BodySim* b = cSim->getAnyBody();
		ConstraintGroupNode* n = b->getConstraintGroup();
		
		if (n)
			getProjectionManager().invalidateGroup(*n, cSim);
		mConstraintSimPool->destroy(cSim);
	}

	mConstraints.erase(&constraint);
}

#if PX_SUPPORT_GPU_PHYSX
void Sc::Scene::addSoftBody(SoftBodyCore& softBody)
{
	SoftBodySim* sim = PX_NEW(SoftBodySim)(softBody, *this);

	if (sim && (sim->getLowLevelSoftBody() == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mSoftBodies.insert(&softBody);
	mStats->gpuMemSizeSoftBodies += softBody.getGpuMemStat();
}

void Sc::Scene::removeSoftBody(SoftBodyCore& softBody)
{
	SoftBodySim* a = softBody.getSim();
	PX_DELETE(a);
	mSoftBodies.erase(&softBody);
	mStats->gpuMemSizeSoftBodies -= softBody.getGpuMemStat();
}

void Sc::Scene::addFEMCloth(FEMClothCore& femCloth)
{
	FEMClothSim* sim = PX_NEW(FEMClothSim)(femCloth, *this);

	if (sim && (sim->getLowLevelFEMCloth() == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mFEMCloths.insert(&femCloth);
	mStats->gpuMemSizeFEMCloths += femCloth.getGpuMemStat();
}

void Sc::Scene::removeFEMCloth(FEMClothCore& femCloth)
{
	FEMClothSim* a = femCloth.getSim();
	PX_DELETE(a);
	mFEMCloths.erase(&femCloth);
	mStats->gpuMemSizeFEMCloths -= femCloth.getGpuMemStat();
}

void Sc::Scene::addParticleSystem(ParticleSystemCore& particleSystem)
{
	ParticleSystemSim* sim = PX_NEW(ParticleSystemSim)(particleSystem, *this);

	Dy::ParticleSystem* dyParticleSystem = sim->getLowLevelParticleSystem();

	if (sim && (dyParticleSystem == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mParticleSystems.insert(&particleSystem);
	mStats->gpuMemSizeParticles += particleSystem.getShapeCore().getGpuMemStat();
}

void Sc::Scene::removeParticleSystem(ParticleSystemCore& particleSystem)
{
	ParticleSystemSim* a = particleSystem.getSim();
	PX_DELETE(a);
	mParticleSystems.erase(&particleSystem);
	mStats->gpuMemSizeParticles -= particleSystem.getShapeCore().getGpuMemStat();
}

void Sc::Scene::addHairSystem(HairSystemCore& hairSystem)
{
	HairSystemSim* sim = PX_NEW(HairSystemSim)(hairSystem, *this);

	if (sim && (sim->getLowLevelHairSystem() == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mHairSystems.insert(&hairSystem);
	mStats->gpuMemSizeHairSystems += hairSystem.getShapeCore().getGpuMemStat();
}

void Sc::Scene::removeHairSystem(HairSystemCore& hairSystem)
{
	HairSystemSim* sim = hairSystem.getSim();
	PX_DELETE(sim);
	mHairSystems.erase(&hairSystem);
	mStats->gpuMemSizeHairSystems -= hairSystem.getShapeCore().getGpuMemStat();
}
#endif

void Sc::Scene::addArticulation(ArticulationCore& articulation, BodyCore& root)
{
	ArticulationSim* sim = PX_NEW(ArticulationSim)(articulation, *this, root);

	if (sim && (sim->getLowLevelArticulation() == NULL))
	{
		PX_DELETE(sim);
		return;
	}
	mArticulations.insert(&articulation);

	addDirtyArticulationSim(sim);
}

void Sc::Scene::removeArticulation(ArticulationCore& articulation)
{
	ArticulationSim* a = articulation.getSim();

	Sc::ArticulationSimDirtyFlags dirtyFlags = a->getDirtyFlag();
	const bool isDirty = (dirtyFlags & Sc::ArticulationSimDirtyFlag::eUPDATE);
	if(isDirty)
		removeDirtyArticulationSim(a);

	PX_DELETE(a);
	mArticulations.erase(&articulation);
}

void Sc::Scene::addArticulationJoint(ArticulationJointCore& joint, BodyCore& parent, BodyCore& child)
{
	ArticulationJointSim* sim = PX_NEW(ArticulationJointSim)(joint, *parent.getSim(), *child.getSim());
	PX_UNUSED(sim);
}

void Sc::Scene::removeArticulationJoint(ArticulationJointCore& joint)
{
	ArticulationJointSim* sim = joint.getSim();
	PX_DELETE(sim);
}

void Sc::Scene::addArticulationTendon(ArticulationSpatialTendonCore& tendon)
{
	ArticulationSpatialTendonSim* sim = PX_NEW(ArticulationSpatialTendonSim)(tendon, *this);

	PX_UNUSED(sim);
}

void Sc::Scene::removeArticulationTendon(ArticulationSpatialTendonCore& tendon)
{
	ArticulationSpatialTendonSim* sim = tendon.getSim();
	PX_DELETE(sim);
}

void Sc::Scene::addArticulationTendon(ArticulationFixedTendonCore& tendon)
{
	ArticulationFixedTendonSim* sim = PX_NEW(ArticulationFixedTendonSim)(tendon, *this);

	PX_UNUSED(sim);
}

void Sc::Scene::removeArticulationTendon(ArticulationFixedTendonCore& tendon)
{
	ArticulationFixedTendonSim* sim = tendon.getSim();
	PX_DELETE(sim);
}

void Sc::Scene::addArticulationSensor(ArticulationSensorCore& sensor)
{
	ArticulationSensorSim* sim = PX_NEW(ArticulationSensorSim)(sensor, *this);
	PX_UNUSED(sim);
}

void Sc::Scene::removeArticulationSensor(ArticulationSensorCore& sensor)
{
	ArticulationSensorSim* sim = sensor.getSim();
	PX_DELETE(sim);
}

void Sc::Scene::addArticulationSimControl(Sc::ArticulationCore& core)
{
	Sc::ArticulationSim* sim = core.getSim();
	if (sim)
		mSimulationController->addArticulation(sim->getLowLevelArticulation(), sim->getIslandNodeIndex());
}

void Sc::Scene::removeArticulationSimControl(Sc::ArticulationCore& core)
{
	Sc::ArticulationSim* sim = core.getSim();
	if (sim)
		mSimulationController->releaseArticulation(sim->getLowLevelArticulation(), sim->getIslandNodeIndex());
}

#if PX_SUPPORT_GPU_PHYSX
void Sc::Scene::addSoftBodySimControl(Sc::SoftBodyCore& core)
{
	Sc::SoftBodySim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addSoftBody(sim->getLowLevelSoftBody(), sim->getNodeIndex());

		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), sim->getPxActor());
	}
}

void Sc::Scene::removeSoftBodySimControl(Sc::SoftBodyCore& core)
{
	Sc::SoftBodySim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID());
		mSimulationController->releaseSoftBody(sim->getLowLevelSoftBody());
	}
}

void Sc::Scene::addFEMClothSimControl(Sc::FEMClothCore& core)
{
	Sc::FEMClothSim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addFEMCloth(sim->getLowLevelFEMCloth(), sim->getNodeIndex());

		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), sim->getPxActor(), true);
	}
}

void Sc::Scene::removeFEMClothSimControl(Sc::FEMClothCore& core)
{
	Sc::FEMClothSim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), true);
		mSimulationController->releaseFEMCloth(sim->getLowLevelFEMCloth());
	}
}

void Sc::Scene::addParticleFilter(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	mSimulationController->addParticleFilter(sim.getLowLevelSoftBody(), core->getSim()->getLowLevelParticleSystem(),
		particleId, userBufferId, tetId);
}

void Sc::Scene::removeParticleFilter(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	mSimulationController->removeParticleFilter(sim.getLowLevelSoftBody(), core->getSim()->getLowLevelParticleSystem(), particleId, userBufferId, tetId);
}

PxU32 Sc::Scene::addParticleAttachment(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric)
{
	PxNodeIndex nodeIndex = core->getSim()->getNodeIndex();

	PxU32 handle = mSimulationController->addParticleAttachment(sim.getLowLevelSoftBody(), core->getSim()->getLowLevelParticleSystem(),
		particleId, userBufferId, tetId, barycentric, sim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::eSOFT_BODY_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eSOFT_BODY_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
	return handle;
}

void Sc::Scene::removeParticleAttachment(Sc::ParticleSystemCore* core, SoftBodySim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex = core->getSim()->getNodeIndex();
	
	mSimulationController->removeParticleAttachment(sim.getLowLevelSoftBody(), handle);

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addRigidFilter(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 vertId)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addRigidFilter(sim.getLowLevelSoftBody(), sim.getNodeIndex(),
		nodeIndex, vertId);
}

void Sc::Scene::removeRigidFilter(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 vertId)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidFilter(sim.getLowLevelSoftBody(), nodeIndex, vertId);
}

PxU32 Sc::Scene::addRigidAttachment(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 vertId, const PxVec3& actorSpacePose,
	PxConeLimitedConstraint* constraint)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addRigidAttachment(sim.getLowLevelSoftBody(), sim.getNodeIndex(), body, 
		nodeIndex, vertId, actorSpacePose, constraint, sim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::eSOFT_BODY_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eSOFT_BODY_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
	return handle;
}

void Sc::Scene::removeRigidAttachment(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex;
	
	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidAttachment(sim.getLowLevelSoftBody(), handle);

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addTetRigidFilter(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 tetIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addTetRigidFilter(sim.getLowLevelSoftBody(), nodeIndex, tetIdx);
}

void Sc::Scene::removeTetRigidFilter(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 tetIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}
	mSimulationController->removeTetRigidFilter(sim.getLowLevelSoftBody(), nodeIndex, tetIdx);
}

PxU32 Sc::Scene::addTetRigidAttachment(Sc::BodyCore* core, Sc::SoftBodySim& sim, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, 
	PxConeLimitedConstraint* constraint)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addTetRigidAttachment(sim.getLowLevelSoftBody(), body, nodeIndex,
		tetIdx, barycentric, actorSpacePose, constraint, sim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::eSOFT_BODY_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eSOFT_BODY_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
	return handle;
}

void Sc::Scene::addSoftBodyFilter(SoftBodyCore& core, PxU32 tetIdx0, SoftBodySim& sim, PxU32 tetIdx1)
{
	Sc::SoftBodySim& bSim = *core.getSim();

	mSimulationController->addSoftBodyFilter(bSim.getLowLevelSoftBody(), sim.getLowLevelSoftBody(), tetIdx0, tetIdx1);
}

void Sc::Scene::removeSoftBodyFilter(SoftBodyCore& core, PxU32 tetIdx0, SoftBodySim& sim, PxU32 tetIdx1)
{
	Sc::SoftBodySim& bSim = *core.getSim();
	mSimulationController->removeSoftBodyFilter(bSim.getLowLevelSoftBody(), sim.getLowLevelSoftBody(), tetIdx0, tetIdx1);
}

void Sc::Scene::addSoftBodyFilters(SoftBodyCore& core, SoftBodySim& sim, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	Sc::SoftBodySim& bSim = *core.getSim();

	mSimulationController->addSoftBodyFilters(bSim.getLowLevelSoftBody(), sim.getLowLevelSoftBody(), tetIndices0, tetIndices1, tetIndicesSize);
}

void Sc::Scene::removeSoftBodyFilters(SoftBodyCore& core, SoftBodySim& sim, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	Sc::SoftBodySim& bSim = *core.getSim();
	mSimulationController->removeSoftBodyFilters(bSim.getLowLevelSoftBody(), sim.getLowLevelSoftBody(), tetIndices0, tetIndices1, tetIndicesSize);
}

PxU32 Sc::Scene::addSoftBodyAttachment(SoftBodyCore& core, PxU32 tetIdx0, const PxVec4& tetBarycentric0, Sc::SoftBodySim& sim, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
	PxConeLimitedConstraint* constraint)
{
	Sc::SoftBodySim& bSim = *core.getSim();

	PxU32 handle = mSimulationController->addSoftBodyAttachment(bSim.getLowLevelSoftBody(), sim.getLowLevelSoftBody(), tetIdx0, tetIdx1, 
		tetBarycentric0, tetBarycentric1, constraint, sim.isActive() || bSim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), bSim.getNodeIndex().index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), bSim.getNodeIndex(), NULL, IG::Edge::eSOFT_BODY_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eSOFT_BODY_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;

	return handle;
}

void Sc::Scene::removeSoftBodyAttachment(SoftBodyCore& core,  Sc::SoftBodySim& sim, PxU32 handle)
{
	Sc::SoftBodySim& bSim = *core.getSim();
	mSimulationController->removeSoftBodyAttachment(bSim.getLowLevelSoftBody(), handle);

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), bSim.getNodeIndex().index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addClothFilter(Sc::FEMClothCore& core, PxU32 triIdx, Sc::SoftBodySim& sim, PxU32 tetIdx)
{
	Sc::FEMClothSim& bSim = *core.getSim();

	mSimulationController->addClothFilter(sim.getLowLevelSoftBody(), bSim.getLowLevelFEMCloth(), triIdx,tetIdx);
}

void Sc::Scene::removeClothFilter(Sc::FEMClothCore& core, PxU32 triIdx, Sc::SoftBodySim& sim, PxU32 tetIdx)
{
	Sc::FEMClothSim& bSim = *core.getSim();
	mSimulationController->removeClothFilter(sim.getLowLevelSoftBody(), bSim.getLowLevelFEMCloth(), triIdx, tetIdx);
}

PxU32 Sc::Scene::addClothAttachment(Sc::FEMClothCore& core, PxU32 triIdx, const PxVec4& triBarycentric, Sc::SoftBodySim& sim, PxU32 tetIdx, 
	const PxVec4& tetBarycentric, PxConeLimitedConstraint* constraint)
{
	Sc::FEMClothSim& bSim = *core.getSim();

	PxU32 handle = mSimulationController->addClothAttachment(sim.getLowLevelSoftBody(), bSim.getLowLevelFEMCloth(), triIdx, triBarycentric,
		tetIdx, tetBarycentric, constraint, sim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), bSim.getNodeIndex().index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), bSim.getNodeIndex(), NULL, IG::Edge::eFEM_CLOTH_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eFEM_CLOTH_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;

	return handle;
}

void Sc::Scene::removeClothAttachment(Sc::FEMClothCore& core, Sc::SoftBodySim& sim, PxU32 handle)
{
	PX_UNUSED(core);
	Sc::FEMClothSim& bSim = *core.getSim();
	mSimulationController->removeClothAttachment(sim.getLowLevelSoftBody(), handle);

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), bSim.getNodeIndex().index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addRigidFilter(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 vertId)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addRigidFilter(sim.getLowLevelFEMCloth(), nodeIndex, vertId);
}

void Sc::Scene::removeRigidFilter(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 vertId)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidFilter(sim.getLowLevelFEMCloth(), nodeIndex, vertId);
}

PxU32 Sc::Scene::addRigidAttachment(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addRigidAttachment(sim.getLowLevelFEMCloth(), sim.getNodeIndex(), body, nodeIndex,
		vertId, actorSpacePose, constraint, sim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::eFEM_CLOTH_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eFEM_CLOTH_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
	return handle;
}

void Sc::Scene::removeRigidAttachment(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidAttachment(sim.getLowLevelFEMCloth(), handle);

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addTriRigidFilter(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 triIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addTriRigidFilter(sim.getLowLevelFEMCloth(), nodeIndex, triIdx);
}

void Sc::Scene::removeTriRigidFilter(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 triIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeTriRigidFilter(sim.getLowLevelFEMCloth(), nodeIndex, triIdx);
}

PxU32 Sc::Scene::addTriRigidAttachment(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 triIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addTriRigidAttachment(sim.getLowLevelFEMCloth(), body, nodeIndex,
		triIdx, barycentric, actorSpacePose, constraint, sim.isActive());

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::eFEM_CLOTH_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eFEM_CLOTH_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
	return handle;
}

void Sc::Scene::removeTriRigidAttachment(Sc::BodyCore* core, Sc::FEMClothSim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeTriRigidAttachment(sim.getLowLevelFEMCloth(), handle);

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addClothFilter(FEMClothCore& core0, PxU32 triIdx0, Sc::FEMClothSim& sim1, PxU32 triIdx1)
{
	Sc::FEMClothSim& sim0 = *core0.getSim();

	mSimulationController->addClothFilter(sim0.getLowLevelFEMCloth(), sim1.getLowLevelFEMCloth(), triIdx0, triIdx1);
}

void Sc::Scene::removeClothFilter(FEMClothCore& core, PxU32 triIdx0, FEMClothSim& sim1, PxU32 triIdx1)
{
	Sc::FEMClothSim& sim0 = *core.getSim();
	mSimulationController->removeClothFilter(sim0.getLowLevelFEMCloth(), sim1.getLowLevelFEMCloth(), triIdx0, triIdx1);
}

PxU32 Sc::Scene::addTriClothAttachment(FEMClothCore& core, PxU32 triIdx0, const PxVec4& barycentric0, Sc::FEMClothSim& sim1, PxU32 triIdx1, const PxVec4& barycentric1)
{
	Sc::FEMClothSim& sim0 = *core.getSim();

	PxU32 handle = mSimulationController->addTriClothAttachment(sim0.getLowLevelFEMCloth(), sim1.getLowLevelFEMCloth(), triIdx0, triIdx1,
		barycentric0, barycentric1, sim1.isActive() || sim0.isActive());

	//return handle;

	PxPair<PxU32, PxU32> pair(sim0.getNodeIndex().index(), sim1.getNodeIndex().index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim0.getNodeIndex(), sim1.getNodeIndex(), NULL, IG::Edge::eFEM_CLOTH_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eFEM_CLOTH_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
	return handle;
}

void Sc::Scene::removeTriClothAttachment(FEMClothCore& core, FEMClothSim& sim1, PxU32 handle)
{
	Sc::FEMClothSim& sim0 = *core.getSim();
	mSimulationController->removeTriClothAttachment(sim0.getLowLevelFEMCloth(), handle);

	PxPair<PxU32, PxU32> pair(sim0.getNodeIndex().index(), sim1.getNodeIndex().index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addParticleSystemSimControl(Sc::ParticleSystemCore& core)
{
	Sc::ParticleSystemSim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addParticleSystem(sim->getLowLevelParticleSystem(), sim->getNodeIndex(), core.getSolverType());
		
		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getCore().getShapeCore().getCore(), sim->getLowLevelParticleSystem()->getElementId(), sim->getPxActor());
	}
}

void Sc::Scene::removeParticleSystemSimControl(Sc::ParticleSystemCore& core)
{
	Sc::ParticleSystemSim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getCore().getShapeCore().getCore(), sim->getShapeSim().getElementID());
		mSimulationController->releaseParticleSystem(sim->getLowLevelParticleSystem(), core.getSolverType());
	}
}


void Sc::Scene::addRigidAttachment(Sc::BodyCore* core, Sc::ParticleSystemSim& sim)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}
	
	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::ePARTICLE_SYSTEM_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::ePARTICLE_SYSTEM_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
}

void Sc::Scene::removeRigidAttachment(Sc::BodyCore* core, Sc::ParticleSystemSim& sim)
{
	PxNodeIndex nodeIndex;
	if (core)
		nodeIndex = core->getSim()->getNodeIndex();

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		mSimpleIslandManager->removeConnection(interaction.mIndex);
		mParticleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

void Sc::Scene::addHairSystemSimControl(Sc::HairSystemCore& core)
{
	Sc::HairSystemSim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addHairSystem(sim->getLowLevelHairSystem(), sim->getNodeIndex());
		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), sim->getPxActor());
	}
}

void Sc::Scene::removeHairSystemSimControl(Sc::HairSystemCore& core)
{
	Sc::HairSystemSim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID());
		mSimulationController->releaseHairSystem(sim->getLowLevelHairSystem());
	}
}

void Sc::Scene::addRigidAttachment(const Sc::BodyCore* core, const Sc::HairSystemSim& sim)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, IG::Edge::eHAIR_SYSTEM_CONTACT);
		mSimpleIslandManager->setEdgeConnected(edgeIdx, IG::Edge::eHAIR_SYSTEM_CONTACT);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
}

void Sc::Scene::removeRigidAttachment(const Sc::BodyCore* core, const Sc::HairSystemSim& sim)
{
	PxNodeIndex nodeIndex;

	if(core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	if(mParticleOrSoftBodyRigidInteractionMap.find(pair)) // find returns pointer to const so we cannot use it directly
	{
		ParticleOrSoftBodyRigidInteraction& interaction = mParticleOrSoftBodyRigidInteractionMap[pair];
		PX_ASSERT(interaction.mCount > 0);
		interaction.mCount--;
		if(interaction.mCount == 0)
		{
			mSimpleIslandManager->removeConnection(interaction.mIndex);
			mParticleOrSoftBodyRigidInteractionMap.erase(pair);
		}
	}
}
#endif

void Sc::Scene::addBrokenConstraint(Sc::ConstraintCore* c)
{
	PX_ASSERT(mBrokenConstraints.find(c) == mBrokenConstraints.end());
	mBrokenConstraints.pushBack(c);
}

void Sc::Scene::addActiveBreakableConstraint(Sc::ConstraintSim* c, Sc::ConstraintInteraction* ci)
{
	PX_ASSERT(ci && ci->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_UNUSED(ci);
	PX_ASSERT(!mActiveBreakableConstraints.contains(c));
	PX_ASSERT(!c->isBroken());
	mActiveBreakableConstraints.insert(c);
	c->setFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED);
}

void Sc::Scene::removeActiveBreakableConstraint(Sc::ConstraintSim* c)
{
	const bool exists = mActiveBreakableConstraints.erase(c);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
	c->clearFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED);
}

void* Sc::Scene::allocateConstraintBlock(PxU32 size)
{
	if(size<=128)
		return mMemBlock128Pool.construct();
	else if(size<=256)
		return mMemBlock256Pool.construct();
	else  if(size<=384)
		return mMemBlock384Pool.construct();
	else
		return PX_ALLOC(size, "ConstraintBlock");
}

void Sc::Scene::deallocateConstraintBlock(void* ptr, PxU32 size)
{
	if(size<=128)
		mMemBlock128Pool.destroy(reinterpret_cast<MemBlock128*>(ptr));
	else if(size<=256)
		mMemBlock256Pool.destroy(reinterpret_cast<MemBlock256*>(ptr));
	else  if(size<=384)
		mMemBlock384Pool.destroy(reinterpret_cast<MemBlock384*>(ptr));
	else
		PX_FREE(ptr);
}

/*-------------------------------*\
| Adam's explanation of the RB solver:
| This is a novel idea of mine, 
| a combination of ideas on
| Milenkovic's Optimization
| Based Animation, and Trinkle's 
| time stepping schemes.
|
| A time step goes like this:
|
| Taking no substeps:
| 0) Compute contact points.
| 1) Update external forces. This may include friction.
| 2) Integrate external forces to current velocities.
| 3) Solve for impulses at contacts which will prevent 
|	interpenetration at next timestep given some 
|	velocity integration scheme.
| 4) Use the integration scheme on velocity to
|	reach the next state. Here we should not have any
|   interpenetration at the old contacts, but perhaps
|	at new contacts. If interpenetrating at new contacts,
|	just add these to the contact list; no need to repeat
|	the time step, because the scheme will get rid of the
|	penetration by the next step.
|
|
| Advantages:
| + Large steps, LOD realism.
| + very simple.
|
\*-------------------------------*/

void Sc::Scene::advanceStep(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.solveQueueTasks", getContextId());

	if (mDt != 0.0f)
	{
		mFinalizationPhase.addDependent(*continuation);
		mFinalizationPhase.removeReference();

		if (mPublicFlags & PxSceneFlag::eENABLE_CCD)
		{
			mUpdateCCDMultiPass.setContinuation(&mFinalizationPhase);
			mAfterIntegration.setContinuation(&mUpdateCCDMultiPass);
			mUpdateCCDMultiPass.removeReference();
		}
		else
		{
			mAfterIntegration.setContinuation(&mFinalizationPhase);
		}

		mPostSolver.setContinuation(&mAfterIntegration);
		mUpdateSimulationController.setContinuation(&mPostSolver);
		mUpdateDynamics.setContinuation(&mUpdateSimulationController);
		mUpdateBodies.setContinuation(&mUpdateDynamics);
		mSolver.setContinuation(&mUpdateBodies);
		mPostIslandGen.setContinuation(&mSolver);
		mIslandGen.setContinuation(&mPostIslandGen);
		mPostNarrowPhase.addDependent(mIslandGen);
		mPostNarrowPhase.removeReference();

		mSecondPassNarrowPhase.setContinuation(&mPostNarrowPhase);

		mFinalizationPhase.removeReference();
		mAfterIntegration.removeReference();
		mPostSolver.removeReference();
		mUpdateSimulationController.removeReference();
		mUpdateDynamics.removeReference();
		mUpdateBodies.removeReference();
		mSolver.removeReference();
		mPostIslandGen.removeReference();
		mIslandGen.removeReference();
		mPostNarrowPhase.removeReference();
		mSecondPassNarrowPhase.removeReference();
	}
}

void Sc::Scene::collideStep(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.collideQueueTasks", getContextId());
	PX_PROFILE_START_CROSSTHREAD("Basic.collision", getContextId());

	mStats->simStart();
	mLLContext->beginUpdate();

	mSimulationController->flushInsertions();

	mPostNarrowPhase.setTaskManager(*continuation->getTaskManager());
	mPostNarrowPhase.addReference();

	mFinalizationPhase.setTaskManager(*continuation->getTaskManager());
	mFinalizationPhase.addReference();

	mRigidBodyNarrowPhase.setContinuation(continuation);
	mPreRigidBodyNarrowPhase.setContinuation(&mRigidBodyNarrowPhase);
	mUpdateShapes.setContinuation(&mPreRigidBodyNarrowPhase);

	mRigidBodyNarrowPhase.removeReference();
	mPreRigidBodyNarrowPhase.removeReference();
	mUpdateShapes.removeReference();
}

void Sc::Scene::broadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.broadPhase", getContextId());

	mProcessLostPatchesTask.setContinuation(&mPostNarrowPhase);
	mProcessLostPatchesTask.removeReference();

#if PX_SUPPORT_GPU_PHYSX
	//update soft bodies world bound
	Sc::SoftBodyCore* const* softBodies = mSoftBodies.getEntries();
	PxU32 size = mSoftBodies.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			softBodies[i]->getSim()->updateBoundsInAABBMgr();
		}
	}
	else
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			softBodies[i]->getSim()->updateBounds();
		}
	}

	// update FEM-cloth world bound
	Sc::FEMClothCore* const* femCloths = mFEMCloths.getEntries();
	size = mFEMCloths.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			femCloths[i]->getSim()->updateBoundsInAABBMgr();
		}
	}
	else
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			femCloths[i]->getSim()->updateBounds();
		}
	}

	//upate the actor handle of particle system in AABB manager 
	Sc::ParticleSystemCore* const* particleSystems = mParticleSystems.getEntries();

	size = mParticleSystems.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < size; ++i)
			particleSystems[i]->getSim()->updateBoundsInAABBMgr();
	}
	else
	{
		for (PxU32 i = 0; i < size; ++i)
			particleSystems[i]->getSim()->updateBounds();
	}

	//update hair system world bound
	Sc::HairSystemCore* const* hairSystems = mHairSystems.getEntries();
	PxU32 nHairSystems = mHairSystems.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < nHairSystems; ++i)
		{
			hairSystems[i]->getSim()->updateBoundsInAABBMgr();
		}
	}
	else
	{
		for (PxU32 i = 0; i < nHairSystems; ++i)
		{
			hairSystems[i]->getSim()->updateBounds();
		}
	}

#endif

	mCCDBp = false;

	mBpSecondPass.setContinuation(continuation);
	mBpFirstPass.setContinuation(&mBpSecondPass);

	mBpSecondPass.removeReference();
	mBpFirstPass.removeReference();
}

void Sc::Scene::broadPhaseFirstPass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.broadPhaseFirstPass", getContextId());

	const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
	mAABBManager->updateBPFirstPass(numCpuTasks, mLLContext->getTaskPool(), mHasContactDistanceChanged, continuation);
	
	PxU32 maxAABBHandles = PxMax(mAABBManager->getChangedAABBMgActorHandleMap().getWordCount() * 32, getElementIDPool().getMaxID());
	
	mSimulationController->mergeChangedAABBMgHandle(maxAABBHandles, mPublicFlags & PxSceneFlag::eSUPPRESS_READBACK);
}

void Sc::Scene::broadPhaseSecondPass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Basic.broadPhaseSecondPass", getContextId());

	mBpUpdate.setContinuation(continuation);
	mPreIntegrate.setContinuation(&mBpUpdate);

	mPreIntegrate.removeReference();
	mBpUpdate.removeReference();
}

void Sc::Scene::preIntegrate(PxBaseTask* continuation)
{
	if (!mCCDBp && isUsingGpuDynamicsOrBp())
		mSimulationController->preIntegrateAndUpdateBound(continuation, mGravity, mDt);
}

void Sc::Scene::updateBroadPhase(PxBaseTask* continuation)
{
	PxBaseTask* rigidBodyNPhaseUnlock = mCCDPass ? NULL : &mRigidBodyNPhaseUnlock;

	const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();

	mAABBManager->updateBPSecondPass(numCpuTasks, &mLLContext->getScratchAllocator(), continuation);

	// PT: decoupling: I moved this back from updateBPSecondPass
	//if this is mCCDPass, narrowPhaseUnlockTask will be NULL
	if(rigidBodyNPhaseUnlock)
		rigidBodyNPhaseUnlock->removeReference();

	if(!mCCDBp && isUsingGpuDynamicsOrBp())
		mSimulationController->updateParticleSystemsAndSoftBodies();
}

void Sc::Scene::postBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.postBroadPhase", getContextId());

	//Notify narrow phase that broad phase has completed
	mLLContext->getNphaseImplementationContext()->postBroadPhaseUpdateContactManager(continuation);
	mAABBManager->postBroadPhase(continuation, *getFlushPool());
}

void Sc::Scene::postBroadPhaseContinuation(PxBaseTask* continuation)
{
	mAABBManager->getChangedAABBMgActorHandleMap().clear();

	// - Finishes broadphase update
	// - Adds new interactions (and thereby contact managers if needed)
	finishBroadPhase(continuation);
}

void Sc::Scene::postBroadPhaseStage2(PxBaseTask* continuation)
{
	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();
	//Release unused Cms back to the pool (later, this needs to be done in a thread-safe way from multiple worker threads
	mIslandInsertion.setContinuation(continuation);
	mRegisterContactManagers.setContinuation(continuation);
	mRegisterInteractions.setContinuation(continuation);
	mRegisterSceneInteractions.setContinuation(continuation);
	mIslandInsertion.removeReference();
	mRegisterContactManagers.removeReference();
	mRegisterInteractions.removeReference();
	mRegisterSceneInteractions.removeReference();

	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.release", getContextId());
		for (PxU32 a = 0; a < mPreallocatedContactManagers.size(); ++a)
		{
			if ((size_t(mPreallocatedContactManagers[a]) & 1) == 0)
				mLLContext->getContactManagerPool().put(mPreallocatedContactManagers[a]);
		}

		for (PxU32 a = 0; a < mPreallocatedShapeInteractions.size(); ++a)
		{
			if ((size_t(mPreallocatedShapeInteractions[a]) & 1) == 0)
				mNPhaseCore->mShapeInteractionPool.deallocate(mPreallocatedShapeInteractions[a]);
		}

		for (PxU32 a = 0; a < mPreallocatedInteractionMarkers.size(); ++a)
		{
			if ((size_t(mPreallocatedInteractionMarkers[a]) & 1) == 0)
				mNPhaseCore->mInteractionMarkerPool.deallocate(mPreallocatedInteractionMarkers[a]);
		}
	}
}

void Sc::Scene::postBroadPhaseStage3(PxBaseTask* /*continuation*/)
{
	finishBroadPhaseStage2(0);

	PX_PROFILE_STOP_CROSSTHREAD("Basic.postBroadPhase", getContextId());
	PX_PROFILE_STOP_CROSSTHREAD("Basic.broadPhase", getContextId());
}

class DirtyShapeUpdatesTask : public Cm::Task
{
public:
	static const PxU32 MaxShapes = 256;

	PxsTransformCache& mCache;
	Bp::BoundsArray& mBoundsArray;
	Sc::ShapeSim* mShapes[MaxShapes];
	PxU32 mNbShapes;

	DirtyShapeUpdatesTask(PxU64 contextID, PxsTransformCache& cache, Bp::BoundsArray& boundsArray) : 
		Cm::Task	(contextID),
		mCache		(cache),
		mBoundsArray(boundsArray),
		mNbShapes	(0)
	{
	}

	virtual void runInternal() 
	{
		for (PxU32 a = 0; a < mNbShapes; ++a)
		{
			mShapes[a]->updateCached(mCache, mBoundsArray);
		}
	}

	virtual const char* getName() const { return "DirtyShapeUpdatesTask";  }

private:
	PX_NOCOPY(DirtyShapeUpdatesTask)
};

void Sc::Scene::preRigidBodyNarrowPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Scene.preNarrowPhase", getContextId());

	updateContactDistances(continuation);
	
	//Process dirty shapeSims...
	PxBitMap::Iterator dirtyShapeIter(mDirtyShapeSimMap);

	PxsTransformCache& cache = mLLContext->getTransformCache();
	Bp::BoundsArray& boundsArray = mAABBManager->getBoundsArray();

	Cm::FlushPool& pool = mLLContext->getTaskPool();
	PxBitMapPinned& changedMap = mAABBManager->getChangedAABBMgActorHandleMap();

	DirtyShapeUpdatesTask* task = PX_PLACEMENT_NEW(pool.allocate(sizeof(DirtyShapeUpdatesTask)), DirtyShapeUpdatesTask)(getContextId(), cache, boundsArray);

	bool hasDirtyShapes = false;
	PxU32 index;
	while ((index = dirtyShapeIter.getNext()) != PxBitMap::Iterator::DONE)
	{
		Sc::ShapeSim* shapeSim = reinterpret_cast<Sc::ShapeSim*>(mAABBManager->getUserData(index));
		if (shapeSim)
		{
			hasDirtyShapes = true;
			changedMap.growAndSet(index);
			task->mShapes[task->mNbShapes++] = shapeSim;
			if (task->mNbShapes == DirtyShapeUpdatesTask::MaxShapes)
			{
				task->setContinuation(continuation);
				task->removeReference();
				task = PX_PLACEMENT_NEW(pool.allocate(sizeof(DirtyShapeUpdatesTask)), DirtyShapeUpdatesTask)(getContextId(), cache, boundsArray);
			}
		}
	}

	if (hasDirtyShapes)
	{
		//Setting the boundsArray and transform cache as dirty so that they get DMAd to GPU if GPU dynamics and BP are being used respectively.
		//These bits are no longer set when we update the cached state for actors due to an optimization avoiding setting these dirty bits multiple times.
		getBoundsArray().setChangedState();
		getLowLevelContext()->getTransformCache().setChangedState();
	}

	if (task->mNbShapes != 0)
	{
		task->setContinuation(continuation);
		task->removeReference();
	}

	mDirtyShapeSimMap.clear();
}

void Sc::Scene::updateBoundsAndShapes(PxBaseTask* /*continuation*/)
{
	//if the scene isn't use gpu dynamic and gpu broad phase and the user raise suppress readback flag,
	//the sdk will refuse to create the scene.
	const bool useDirectGpuApi = mPublicFlags & PxSceneFlag::eSUPPRESS_READBACK;
	mSimulationController->updateBoundsAndShapes(*mAABBManager, mUseGpuBp, useDirectGpuApi);
}

void Sc::Scene::rigidBodyNarrowPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.narrowPhase", getContextId());

	mCCDPass = 0;

	mPostBroadPhase3.addDependent(*continuation);
	mPostBroadPhase2.setContinuation(&mPostBroadPhase3);
	mPostBroadPhaseCont.setContinuation(&mPostBroadPhase2);
	mPostBroadPhase.setContinuation(&mPostBroadPhaseCont);
	mBroadPhase.setContinuation(&mPostBroadPhase);

	mRigidBodyNPhaseUnlock.setContinuation(continuation);
	mRigidBodyNPhaseUnlock.addReference();

	mUpdateBoundAndShapeTask.addDependent(mBroadPhase);

	mLLContext->resetThreadContexts();

	mLLContext->updateContactManager(mDt, mBoundsArray->hasChanged(), mHasContactDistanceChanged, continuation, 
		&mRigidBodyNPhaseUnlock, &mUpdateBoundAndShapeTask); // Starts update of contact managers

	mPostBroadPhase3.removeReference();
	mPostBroadPhase2.removeReference();
	mPostBroadPhaseCont.removeReference();
	mPostBroadPhase.removeReference();
	mBroadPhase.removeReference();

	mUpdateBoundAndShapeTask.removeReference();
}

void Sc::Scene::unblockNarrowPhase(PxBaseTask*)
{
	/*if (!mCCDBp && mUseGpuRigidBodies)
		mSimulationController->updateParticleSystemsAndSoftBodies();*/
	//
	mLLContext->getNphaseImplementationContext()->startNarrowPhaseTasks();
}

void Sc::Scene::postNarrowPhase(PxBaseTask* /*continuation*/)
{
	setCollisionPhaseToInactive();

	mHasContactDistanceChanged = false;
	mLLContext->fetchUpdateContactManager(); //Sync on contact gen results!

	if (!mCCDBp && isUsingGpuDynamicsOrBp())
	{
		mSimulationController->sortContacts();
	}

	releaseConstraints(false);

	PX_PROFILE_STOP_CROSSTHREAD("Basic.narrowPhase", getContextId());
	PX_PROFILE_STOP_CROSSTHREAD("Basic.collision", getContextId());
}

void Sc::Scene::fetchPatchEvents(PxBaseTask*)
{
	//PxU32 foundPatchCount, lostPatchCount;

	//{
	//	PX_PROFILE_ZONE("Sim.preIslandGen.managerPatchEvents", getContextId());
	//	mLLContext->getManagerPatchEventCount(foundPatchCount, lostPatchCount);

	//	//mLLContext->fillManagerPatchChangedEvents(mFoundPatchManagers.begin(), foundPatchCount, mLostPatchManagers.begin(), lostPatchCount);
	//}
}

void Sc::Scene::processNarrowPhaseTouchEvents()
{
	PX_PROFILE_ZONE("Sim.preIslandGen", getContextId());

	PxsContext* context = mLLContext;

	// Update touch states from LL
	PxU32 newTouchCount, lostTouchCount;
	PxU32 ccdTouchCount = 0;
	{
		PX_PROFILE_ZONE("Sim.preIslandGen.managerTouchEvents", getContextId());
		context->getManagerTouchEventCount(reinterpret_cast<PxI32*>(&newTouchCount), reinterpret_cast<PxI32*>(&lostTouchCount), NULL);
		//PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
		//PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);

		mTouchFoundEvents.forceSize_Unsafe(0);
		mTouchFoundEvents.reserve(newTouchCount);
		mTouchFoundEvents.forceSize_Unsafe(newTouchCount);

		mTouchLostEvents.forceSize_Unsafe(0);
		mTouchLostEvents.reserve(lostTouchCount);
		mTouchLostEvents.forceSize_Unsafe(lostTouchCount);

		context->fillManagerTouchEvents(mTouchFoundEvents.begin(), reinterpret_cast<PxI32&>(newTouchCount), mTouchLostEvents.begin(),
			reinterpret_cast<PxI32&>(lostTouchCount), NULL, reinterpret_cast<PxI32&>(ccdTouchCount));

		mTouchFoundEvents.forceSize_Unsafe(newTouchCount);
		mTouchLostEvents.forceSize_Unsafe(lostTouchCount);
	}

	context->getSimStats().mNbNewTouches = newTouchCount;
	context->getSimStats().mNbLostTouches = lostTouchCount;
}

static PX_FORCE_INLINE Sc::ShapeInteraction* getSI(PxvContactManagerTouchEvent& evt)
{
	return reinterpret_cast<Sc::ShapeInteraction*>(evt.getCMTouchEventUserData());
}

class InteractionNewTouchTask : public Cm::Task
{
	PxvContactManagerTouchEvent* mEvents;
	const PxU32 mNbEvents;
	PxsContactManagerOutputIterator mOutputs;
	Sc::NPhaseCore* mNphaseCore;

public:
	InteractionNewTouchTask(PxU64 contextID, PxvContactManagerTouchEvent* events, PxU32 nbEvents, PxsContactManagerOutputIterator& outputs, Sc::NPhaseCore* nPhaseCore) :
		Cm::Task	(contextID),
		mEvents		(events),
		mNbEvents	(nbEvents),
		mOutputs	(outputs),
		mNphaseCore	(nPhaseCore)
	{
	}

	virtual const char* getName() const
	{
		return "InteractionNewTouchTask";
	}
	
	void hackInContinuation(PxBaseTask* cont)
	{
		PX_ASSERT(mCont == NULL);
		mCont = cont;
		if (mCont)
			mCont->addReference();
	}

	virtual void runInternal()
	{
		mNphaseCore->lockReports();
		for (PxU32 i = 0; i < mNbEvents; ++i)
		{
			Sc::ShapeInteraction* si = getSI(mEvents[i]);
			PX_ASSERT(si);
			mNphaseCore->managerNewTouch(*si);
			si->managerNewTouch(0, true, mOutputs);
		}
		mNphaseCore->unlockReports();
	}
private:
	PX_NOCOPY(InteractionNewTouchTask)
};

void Sc::Scene::processNarrowPhaseTouchEventsStage2(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processNarrowPhaseTouchEventsStage2", getContextId());
	mLLContext->getNphaseImplementationContext()->waitForContactsReady();
	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	const PxU32 newTouchCount = mTouchFoundEvents.size();

	{
		Cm::FlushPool& flushPool = mLLContext->getTaskPool();

		InteractionNewTouchTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(InteractionNewTouchTask)), InteractionNewTouchTask)(getContextId(), mTouchFoundEvents.begin(), newTouchCount, outputs, mNPhaseCore);
		task->setContinuation(continuation);
		task->removeReference();
	}

	/*{
		PX_PROFILE_ZONE("Sim.preIslandGen.newTouchesInteraction", getContextId());
		for (PxU32 i = 0; i < newTouchCount; ++i)
		{
			ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchFoundEvents[i].userData);
			PX_ASSERT(si);
			mNPhaseCore->managerNewTouch(*si);
			si->managerNewTouch(0, true, outputs, useAdaptiveForce);
		}
	}*/
	
}

void Sc::Scene::setEdgesConnected(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.preIslandGen.islandTouches", getContextId());
	{
		PX_PROFILE_ZONE("Sim.preIslandGen.setEdgesConnected", getContextId());
		const PxU32 newTouchCount = mTouchFoundEvents.size();
		for(PxU32 i = 0; i < newTouchCount; ++i)
		{
			ShapeInteraction* si = getSI(mTouchFoundEvents[i]);
			if(!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex(), IG::Edge::eCONTACT_MANAGER);
		}
	}

	mSimpleIslandManager->secondPassIslandGen();

	wakeObjectsUp(ActorSim::AS_PART_OF_ISLAND_GEN);
}

void Sc::Scene::processNarrowPhaseLostTouchEventsIslands(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sc::Scene.islandLostTouches", getContextId());
	const PxU32 count = mTouchLostEvents.size();
	for(PxU32 i=0; i <count; ++i)
	{
		ShapeInteraction* si = getSI(mTouchLostEvents[i]);
		mSimpleIslandManager->setEdgeDisconnected(si->getEdgeIndex());
	}
}

void Sc::Scene::processNarrowPhaseLostTouchEvents(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sc::Scene.processNarrowPhaseLostTouchEvents", getContextId());
	mLLContext->getNphaseImplementationContext()->waitForContactsReady();
	PxsContactManagerOutputIterator outputs = this->mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	const PxU32 count = mTouchLostEvents.size();
	for(PxU32 i=0; i<count; ++i)
	{
		ShapeInteraction* si = getSI(mTouchLostEvents[i]);
		PX_ASSERT(si);
		if(si->managerLostTouch(0, true, outputs) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
			addToLostTouchList(si->getShape0().getActor(), si->getShape1().getActor());
	}
}

void Sc::Scene::processLostSolverPatches(PxBaseTask* /*continuation*/)
{
	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();
	mDynamicsContext->processLostPatches(*mSimpleIslandManager, nphase->getFoundPatchManagers(), nphase->getNbFoundPatchManagers(), nphase->getFoundPatchOutputCounts());
}

void Sc::Scene::processFoundSolverPatches(PxBaseTask* /*continuation*/)
{
	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();
	mDynamicsContext->processFoundPatches(*mSimpleIslandManager, nphase->getFoundPatchManagers(), nphase->getNbFoundPatchManagers(), nphase->getFoundPatchOutputCounts());
}

void Sc::Scene::islandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::islandGen", getContextId());
//	PX_PROFILE_START_CROSSTHREAD("Basic.rigidBodySolver", getContextId());

	//mLLContext->runModifiableContactManagers(); //KS - moved here so that we can get up-to-date touch found/lost events in IG

	/*mProcessLostPatchesTask.setContinuation(&mUpdateDynamics);
	mProcessLostPatchesTask.removeReference();*/
	//mFetchPatchEventsTask.setContinuation(&mProcessLostPatchesTask);
	
	//mFetchPatchEventsTask.removeReference();
	processNarrowPhaseTouchEvents();

	mProcessFoundPatchesTask.setContinuation(continuation);
	mProcessFoundPatchesTask.removeReference();

	processNarrowPhaseTouchEventsStage2(&mPostSolver);	
}

void Sc::Scene::putObjectsToSleep(PxU32 infoFlag)
{
	PX_PROFILE_ZONE("Sc::Scene::putObjectsToSleep", getContextId());
	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	//Set to sleep all bodies that were in awake islands that have just been put to sleep.
	const PxU32 nbBodiesToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	const PxNodeIndex*const bodyIndices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

	PxU32 nbBodiesDeactivated = 0;
	for(PxU32 i=0;i<nbBodiesToSleep;i++)
	{
		PxsRigidBody* rigidBody = islandSim.getRigidBody(bodyIndices[i]);
		if (rigidBody && !islandSim.getNode(bodyIndices[i]).isActive())
		{
			Sc::BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - Sc::BodySim::getRigidBodyOffset());
			bodySim->setActive(false, infoFlag);
			nbBodiesDeactivated++;
		}
	}

	const PxU32 nbArticulationsToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);
	const PxNodeIndex*const articIndices = islandSim.getNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	for(PxU32 i=0;i<nbArticulationsToSleep;i++)
	{
		Sc::ArticulationSim* articSim = islandSim.getArticulationSim(articIndices[i]);

		if (articSim && !islandSim.getNode(articIndices[i]).isActive())
		{
			articSim->setActive(false, infoFlag);
			nbBodiesDeactivated++;
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	const PxU32 nbSoftBodiesToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eSOFTBODY_TYPE);
	const PxNodeIndex*const softBodiesIndices = islandSim.getNodesToDeactivate(IG::Node::eSOFTBODY_TYPE);

	for (PxU32 i = 0; i<nbSoftBodiesToSleep; i++)
	{
		Sc::SoftBodySim* softBodySim = islandSim.getLLSoftBody(softBodiesIndices[i])->getSoftBodySim();
		if (softBodySim && !islandSim.getNode(softBodiesIndices[i]).isActive())
		{
			softBodySim->setActive(false, infoFlag);
			nbBodiesDeactivated++;
		}
	}

	const PxU32 nbFemClothesToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eFEMCLOTH_TYPE);
	const PxNodeIndex*const femClothesIndices = islandSim.getNodesToDeactivate(IG::Node::eFEMCLOTH_TYPE);

	for (PxU32 i = 0; i < nbFemClothesToSleep; i++)
	{
		Sc::FEMClothSim* femClothSim = islandSim.getLLFEMCloth(femClothesIndices[i])->getFEMClothSim();
		if (femClothSim && !islandSim.getNode(femClothesIndices[i]).isActive())
		{
			femClothSim->setActive(false, infoFlag);
			nbBodiesDeactivated++;
		}
	}

	const PxU32 nbHairSystemsToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eHAIRSYSTEM_TYPE);
	const PxNodeIndex*const hairSystemIndices = islandSim.getNodesToDeactivate(IG::Node::eHAIRSYSTEM_TYPE);

	for (PxU32 i = 0; i < nbHairSystemsToSleep; i++)
	{
		Sc::HairSystemSim* hairSystemSim = islandSim.getLLHairSystem(hairSystemIndices[i])->getHairSystemSim();
		if (hairSystemSim && !islandSim.getNode(hairSystemIndices[i]).isActive())
		{
			hairSystemSim->setActive(false, infoFlag);
			nbBodiesDeactivated++;
		}
	}
#endif

	if (nbBodiesDeactivated != 0)
		mDynamicsContext->setStateDirty(true);
}

void Sc::Scene::wakeObjectsUp(PxU32 infoFlag)
{
	//Wake up all bodies that were in sleeping islands that have just been hit by a moving object.

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbBodiesToWake = islandSim.getNbNodesToActivate(IG::Node::eRIGID_BODY_TYPE);
	const PxNodeIndex*const bodyIndices = islandSim.getNodesToActivate(IG::Node::eRIGID_BODY_TYPE);

	PxU32 nbBodiesWoken = 0;
	for(PxU32 i=0;i<nbBodiesToWake;i++)
	{
		PxsRigidBody* rigidBody = islandSim.getRigidBody(bodyIndices[i]);
		if (rigidBody && islandSim.getNode(bodyIndices[i]).isActive())
		{
			Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - Sc::BodySim::getRigidBodyOffset());
			bodySim->setActive(true, infoFlag);
			nbBodiesWoken++;
		}
	}

	const PxU32 nbArticulationsToWake = islandSim.getNbNodesToActivate(IG::Node::eARTICULATION_TYPE);
	const PxNodeIndex*const articIndices = islandSim.getNodesToActivate(IG::Node::eARTICULATION_TYPE);

	for(PxU32 i=0;i<nbArticulationsToWake;i++)
	{
		Sc::ArticulationSim* articSim = islandSim.getArticulationSim(articIndices[i]);

		if (articSim && islandSim.getNode(articIndices[i]).isActive())
		{
			articSim->setActive(true, infoFlag);
			nbBodiesWoken++;
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	const PxU32 nbSoftBodyToWake = islandSim.getNbNodesToActivate(IG::Node::eSOFTBODY_TYPE);
	const PxNodeIndex*const softBodyIndices = islandSim.getNodesToActivate(IG::Node::eSOFTBODY_TYPE);

	for (PxU32 i = 0; i<nbSoftBodyToWake; i++)
	{
		Sc::SoftBodySim* softBodySim = islandSim.getLLSoftBody(softBodyIndices[i])->getSoftBodySim();
		if (softBodySim && islandSim.getNode(softBodyIndices[i]).isActive())
		{
			softBodySim->setActive(true, infoFlag);
			nbBodiesWoken++;
		}
	}

	const PxU32 nbFEMClothToWake = islandSim.getNbNodesToActivate(IG::Node::eFEMCLOTH_TYPE);
	const PxNodeIndex*const femClothIndices = islandSim.getNodesToActivate(IG::Node::eFEMCLOTH_TYPE);

	for (PxU32 i = 0; i < nbFEMClothToWake; i++)
	{
		Sc::FEMClothSim* femClothSim = islandSim.getLLFEMCloth(femClothIndices[i])->getFEMClothSim();
		if (femClothSim && islandSim.getNode(femClothIndices[i]).isActive())
		{
			femClothSim->setActive(true, infoFlag);
			nbBodiesWoken++;
		}
	}

	const PxU32 nbHairSystemsToWake = islandSim.getNbNodesToActivate(IG::Node::eHAIRSYSTEM_TYPE);
	const PxNodeIndex*const hairSystemIndices = islandSim.getNodesToActivate(IG::Node::eHAIRSYSTEM_TYPE);

	for (PxU32 i = 0; i < nbHairSystemsToWake; i++)
	{
		Sc::HairSystemSim* hairSystemSim = islandSim.getLLHairSystem(hairSystemIndices[i])->getHairSystemSim();
		if (hairSystemSim && islandSim.getNode(hairSystemIndices[i]).isActive())
		{
			hairSystemSim->setActive(true, infoFlag);
			nbBodiesWoken++;
		}
	}
#endif

	if(nbBodiesWoken != 0)
		mDynamicsContext->setStateDirty(true);
}

void Sc::Scene::postIslandGen(PxBaseTask* continuationTask)
{
	PX_PROFILE_ZONE("Sim.postIslandGen", getContextId());

	mSetEdgesConnectedTask.setContinuation(continuationTask);
	mSetEdgesConnectedTask.removeReference();

	// - Performs collision detection for trigger interactions
	mNPhaseCore->processTriggerInteractions(continuationTask);
}

void Sc::Scene::solver(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.rigidBodySolver", getContextId());
	//Update forces per body in parallel. This can overlap with the other work in this phase.
	beforeSolver(continuation);

	PX_PROFILE_ZONE("Sim.postNarrowPhaseSecondPass", getContextId());
	//Narrowphase is completely finished so the streams can be swapped.
	mLLContext->swapStreams();

	//PxsContactManagerOutputIterator outputs = this->mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);
}

void Sc::Scene::updateBodies(PxBaseTask* continuation)
{
	PX_UNUSED(continuation);

	//dma bodies and articulation data to gpu
	mSimulationController->updateBodies(continuation);
}

void Sc::Scene::updateShapes(PxBaseTask* continuation)
{
	//dma shapes data to gpu
	mSimulationController->updateShapes(continuation);
}

Cm::FlushPool* Sc::Scene::getFlushPool()
{
	return &mLLContext->getTaskPool();
}

bool Sc::activateInteraction(Sc::Interaction* interaction, void* data)
{
	switch(interaction->getType())
	{
		case InteractionType::eOVERLAP:
			return static_cast<Sc::ShapeInteraction*>(interaction)->onActivate_(data);

		case InteractionType::eTRIGGER:
			return static_cast<Sc::TriggerInteraction*>(interaction)->onActivate_(data);

		case InteractionType::eMARKER:
			return static_cast<Sc::ElementInteractionMarker*>(interaction)->onActivate_(data);

		case InteractionType::eCONSTRAINTSHADER:
			return static_cast<Sc::ConstraintInteraction*>(interaction)->onActivate_(data);

		case InteractionType::eARTICULATION:
			return static_cast<Sc::ArticulationJointSim*>(interaction)->onActivate_(data);

		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
		PX_ASSERT(0);
		break;
	}
	return false;
}

static bool deactivateInteraction(Sc::Interaction* interaction, const Sc::InteractionType::Enum type)
{
	using namespace Sc;

	switch(type)
	{
		case InteractionType::eOVERLAP:
			return static_cast<Sc::ShapeInteraction*>(interaction)->onDeactivate_();

		case InteractionType::eTRIGGER:
			return static_cast<Sc::TriggerInteraction*>(interaction)->onDeactivate_();

		case InteractionType::eMARKER:
			return static_cast<Sc::ElementInteractionMarker*>(interaction)->onDeactivate_();

		case InteractionType::eCONSTRAINTSHADER:
			return static_cast<Sc::ConstraintInteraction*>(interaction)->onDeactivate_();

		case InteractionType::eARTICULATION:
			return static_cast<Sc::ArticulationJointSim*>(interaction)->onDeactivate_();

		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
		PX_ASSERT(0);
		break;
	}
	return false;
}

void Sc::Scene::postThirdPassIslandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::postThirdPassIslandGen", getContextId());
	putObjectsToSleep(ActorSim::AS_PART_OF_ISLAND_GEN);

	{
		PX_PROFILE_ZONE("Sc::Scene::putInteractionsToSleep", getContextId());
		const IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();

		//KS - only deactivate contact managers based on speculative state to trigger contact gen. When the actors were deactivated based on accurate state
		//joints should have been deactivated.

		const PxU32 NbTypes = 5;
		const IG::Edge::EdgeType types[NbTypes] = {
			IG::Edge::eCONTACT_MANAGER,
			IG::Edge::eSOFT_BODY_CONTACT,
			IG::Edge::eFEM_CLOTH_CONTACT,
			IG::Edge::ePARTICLE_SYSTEM_CONTACT,
			IG::Edge::eHAIR_SYSTEM_CONTACT };

		for(PxU32 t = 0; t < NbTypes; ++t)
		{
			const PxU32 nbDeactivatingEdges = islandSim.getNbDeactivatingEdges(types[t]);
			const IG::EdgeIndex* deactivatingEdgeIds = islandSim.getDeactivatingEdges(types[t]);

			for(PxU32 i = 0; i < nbDeactivatingEdges; ++i)
			{
				Sc::Interaction* interaction = mSimpleIslandManager->getInteraction(deactivatingEdgeIds[i]);

				if(interaction && interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
				{
					if(!islandSim.getEdge(deactivatingEdgeIds[i]).isActive())
					{
						const InteractionType::Enum type = interaction->getType();
						const bool proceed = deactivateInteraction(interaction, type);
						if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
							notifyInteractionDeactivated(interaction);
					}
				}
			}
		}
	}

	PxvNphaseImplementationContext*	implCtx = mLLContext->getNphaseImplementationContext();
	implCtx->waitForContactsReady();
	PxsContactManagerOutputIterator outputs = implCtx->getContactManagerOutputs();
	mNPhaseCore->processPersistentContactEvents(outputs, continuation);
}

void Sc::Scene::processLostContacts(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processLostContacts", getContextId());
	mProcessNarrowPhaseLostTouchTasks.setContinuation(continuation);
	mProcessNarrowPhaseLostTouchTasks.removeReference();

	//mLostTouchReportsTask.setContinuation(&mProcessLostContactsTask3);
	mProcessNPLostTouchEvents.setContinuation(continuation);
	mProcessNPLostTouchEvents.removeReference();

	{
		PX_PROFILE_ZONE("Sim.findInteractionsPtrs", getContextId());

		Bp::AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;
		Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
			ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);
			p->mPairUserData = mNPhaseCore->onOverlapRemovedStage1(volume0, volume1);
			p++;
		}
	}
}

void Sc::Scene::lostTouchReports(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.lostTouchReports", getContextId());
	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	Bp::AABBManagerBase* aabbMgr = mAABBManager;
	PxU32 destroyedOverlapCount;

	mNPhaseCore->lockReports();

	{
		const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			if(p->mPairUserData)
			{
				Sc::ElementSimInteraction* elemInteraction = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
				if(elemInteraction->getType() == Sc::InteractionType::eOVERLAP)
					mNPhaseCore->lostTouchReports(static_cast<Sc::ShapeInteraction*>(elemInteraction), PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH), NULL, 0, outputs);
			}
			p++;
		}
	}
	mNPhaseCore->unlockReports();
}

void Sc::Scene::unregisterInteractions(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.unregisterInteractions", getContextId());

	Bp::AABBManagerBase* aabbMgr = mAABBManager;
	PxU32 destroyedOverlapCount;

	{
		const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			if(p->mPairUserData)
			{
				Sc::ElementSimInteraction* elemInteraction = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
				if(elemInteraction->getType() == Sc::InteractionType::eOVERLAP || elemInteraction->getType() == Sc::InteractionType::eMARKER)
				{
					unregisterInteraction(elemInteraction);
					mNPhaseCore->unregisterInteraction(elemInteraction);
				}
			}
			p++;
		}
	}
}

void Sc::Scene::destroyManagers(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.destroyManagers", getContextId());

	mPostThirdPassIslandGenTask.setContinuation(mProcessLostContactsTask3.getContinuation());

	mSimpleIslandManager->thirdPassIslandGen(&mPostThirdPassIslandGenTask);

	Bp::AABBManagerBase* aabbMgr = mAABBManager;
	PxU32 destroyedOverlapCount;
	const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
	while(destroyedOverlapCount--)
	{
		if(p->mPairUserData)
		{
			Sc::ElementSimInteraction* elemInteraction = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
			if(elemInteraction->getType() == Sc::InteractionType::eOVERLAP)
			{
				Sc::ShapeInteraction* si = static_cast<Sc::ShapeInteraction*>(elemInteraction);
				if(si->getContactManager())
					si->destroyManager();
			}
		}
		p++;
	}
}

void Sc::Scene::processLostContacts2(PxBaseTask* continuation)
{
	mDestroyManagersTask.setContinuation(continuation);
	mLostTouchReportsTask.setContinuation(&mDestroyManagersTask);
	mLostTouchReportsTask.removeReference();

	mUnregisterInteractionsTask.setContinuation(continuation);
	mUnregisterInteractionsTask.removeReference();
	
	{
		PX_PROFILE_ZONE("Sim.clearIslandData", getContextId());
//		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		Bp::AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;
		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				Sc::ElementSimInteraction* pair = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
				if(pair)
				{
					if(pair->getType() == InteractionType::eOVERLAP)
					{
						ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);
						si->clearIslandGenData();
					}
				}
				p++;
			}
		}
	}

	mDestroyManagersTask.removeReference();
}

void Sc::Scene::processLostContacts3(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processLostOverlapsStage2", getContextId());

		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		Bp::AABBManagerBase* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;

		{
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				mNPhaseCore->onOverlapRemoved(volume0, volume1, false, p->mPairUserData, outputs);
				p++;
			}
		}

		{
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eTRIGGER, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				mNPhaseCore->onOverlapRemoved(volume0, volume1, false, NULL, outputs);
				p++;
			}
		}

		aabbMgr->freeBuffers();
	}

	mPostThirdPassIslandGenTask.removeReference();
}

//This is called after solver finish
void Sc::Scene::updateSimulationController(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateSimulationController", getContextId());
	
	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	Bp::BoundsArray& boundArray = getBoundsArray();

	PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();

	mSimulationController->gpuDmabackData(cache, boundArray, changedAABBMgrActorHandles, mPublicFlags & PxSceneFlag::eSUPPRESS_READBACK);

	//for pxgdynamicscontext: copy solver body data to body core 
	{
		PX_PROFILE_ZONE("Sim.updateBodyCore", getContextId());
		mDynamicsContext->updateBodyCore(continuation);
	}
	//mSimulationController->update(cache, boundArray, changedAABBMgrActorHandles);

	/*mProcessLostPatchesTask.setContinuation(&mFinalizationPhase);
	mProcessLostPatchesTask.removeReference();*/
}

void Sc::Scene::updateDynamics(PxBaseTask* continuation)
{
	//Allow processLostContactsTask to run until after 2nd pass of solver completes (update bodies, run sleeping logic etc.)
	mProcessLostContactsTask3.setContinuation(static_cast<PxLightCpuTask*>(continuation)->getContinuation());
	mProcessLostContactsTask2.setContinuation(&mProcessLostContactsTask3);
	mProcessLostContactsTask.setContinuation(&mProcessLostContactsTask2);

	////dma bodies and shapes data to gpu
	//mSimulationController->updateBodiesAndShapes();

	mLLContext->getNpMemBlockPool().acquireConstraintMemory();

	PX_PROFILE_START_CROSSTHREAD("Basic.dynamics", getContextId());
	PxU32 maxPatchCount = mLLContext->getMaxPatchCount();

	mAABBManager->reallocateChangedAABBMgActorHandleMap(getElementIDPool().getMaxID());

	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);

	PxvNphaseImplementationContext* nphase = mLLContext->getNphaseImplementationContext();

	mDynamicsContext->update(*mSimpleIslandManager, continuation, &mProcessLostContactsTask,
		nphase,	maxPatchCount, mMaxNbArticulationLinks, mDt, mGravity, mAABBManager->getChangedAABBMgActorHandleMap());

	mSimpleIslandManager->clearDestroyedEdges();

	mProcessLostContactsTask3.removeReference();
	mProcessLostContactsTask2.removeReference();
	mProcessLostContactsTask.removeReference();
}

//CCD
void Sc::Scene::updateCCDMultiPass(PxBaseTask* parentContinuation)
{
	getCcdBodies().forceSize_Unsafe(mSimulationControllerCallback->getNbCcdBodies());
	
	// second run of the broadphase for making sure objects we have integrated did not tunnel.
	if(mPublicFlags & PxSceneFlag::eENABLE_CCD)
	{
		if (mContactReportsNeedPostSolverVelocity)
		{
			// the CCD code will overwrite the post solver body velocities, hence, we need to extract the info
			// first if any CCD enabled pair requested it.
			collectPostSolverVelocitiesBeforeCCD();
		}

		//We use 2 CCD task chains to be able to chain together an arbitrary number of ccd passes
		if(mPostCCDPass.size() != 2)
		{
			mPostCCDPass.clear();
			mUpdateCCDSinglePass.clear();
			mCCDBroadPhase.clear();
			mCCDBroadPhaseAABB.clear();
			mPostCCDPass.reserve(2);
			mUpdateCCDSinglePass.reserve(2);
			mUpdateCCDSinglePass2.reserve(2);
			mUpdateCCDSinglePass3.reserve(2);
			mCCDBroadPhase.reserve(2);
			mCCDBroadPhaseAABB.reserve(2);
			for (int j = 0; j < 2; j++)
			{
				mPostCCDPass.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::postCCDPass>(getContextId(), this, "ScScene.postCCDPass"));
				mUpdateCCDSinglePass.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePass>(getContextId(), this, "ScScene.updateCCDSinglePass"));
				mUpdateCCDSinglePass2.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage2>(getContextId(), this, "ScScene.updateCCDSinglePassStage2"));
				mUpdateCCDSinglePass3.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage3>(getContextId(), this, "ScScene.updateCCDSinglePassStage3"));
				mCCDBroadPhase.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhase>(getContextId(), this, "ScScene.ccdBroadPhase"));
				mCCDBroadPhaseAABB.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhaseAABB>(getContextId(), this, "ScScene.ccdBroadPhaseAABB"));
			}
		}

		//reset thread context in a place we know all tasks possibly accessing it, are in sync with. (see US6664)
		mLLContext->resetThreadContexts();

		mCCDContext->updateCCDBegin();

		mCCDBroadPhase[0].setContinuation(parentContinuation);
		mCCDBroadPhaseAABB[0].setContinuation(&mCCDBroadPhase[0]);
		mCCDBroadPhase[0].removeReference();
		mCCDBroadPhaseAABB[0].removeReference();
	}
}

//CCD
class UpdateCCDBoundsTask : public Cm::Task
{
	Bp::BoundsArray*	mBoundArray;
	PxsTransformCache*	mTransformCache;
	Sc::BodySim**		mBodySims;
	PxU32				mNbToProcess;
	PxI32*				mNumFastMovingShapes;

public:

	static const PxU32 MaxPerTask = 256;

	UpdateCCDBoundsTask(PxU64 contextID, Bp::BoundsArray* boundsArray, PxsTransformCache* transformCache, Sc::BodySim** bodySims, PxU32 nbToProcess, PxI32* numFastMovingShapes) :
		Cm::Task			(contextID),
		mBoundArray			(boundsArray),
		mTransformCache		(transformCache),
		mBodySims			(bodySims), 
		mNbToProcess		(nbToProcess),
		mNumFastMovingShapes(numFastMovingShapes)
	{
	}

	virtual const char* getName() const { return "UpdateCCDBoundsTask";}

	PxIntBool	updateSweptBounds(Sc::ShapeSim* sim, Sc::BodySim* body)
	{
		PX_ASSERT(body==sim->getBodySim());

		const PxU32 elementID = sim->getElementID();

		const Sc::ShapeCore& shapeCore = sim->getCore();
		const PxTransform& endPose = mTransformCache->getTransformCache(elementID).transform;

		const PxGeometry& shapeGeom = shapeCore.getGeometry();

		const PxsRigidBody& rigidBody = body->getLowLevelBody();
		const PxsBodyCore& bodyCore = body->getBodyCore().getCore();
		PX_ALIGN(16, PxTransform shape2World);
		Cm::getDynamicGlobalPoseAligned(rigidBody.mLastTransform, shapeCore.getShape2Actor(), bodyCore.getBody2Actor(), shape2World);

		const float ccdThreshold = computeCCDThreshold(shapeGeom);
		PxBounds3 bounds = Gu::computeBounds(shapeGeom, endPose);
		PxIntBool isFastMoving;
		if(1)
		{
			// PT: this alternative implementation avoids computing the start bounds for slow moving objects.
			isFastMoving = (shape2World.p - endPose.p).magnitudeSquared() >= ccdThreshold * ccdThreshold ? 1 : 0;
			if (isFastMoving)
			{
				const PxBounds3 startBounds = Gu::computeBounds(shapeGeom, shape2World);
				bounds.include(startBounds);
			}
		}
		else
		{
			const PxBounds3 startBounds = Gu::computeBounds(shapeGeom, shape2World);

			isFastMoving = (startBounds.getCenter() - bounds.getCenter()).magnitudeSquared() >= ccdThreshold * ccdThreshold ? 1 : 0;

			if(isFastMoving)
				bounds.include(startBounds);
		}

		PX_ASSERT(bounds.minimum.x <= bounds.maximum.x
			&&	  bounds.minimum.y <= bounds.maximum.y
			&&	  bounds.minimum.z <= bounds.maximum.z);

		mBoundArray->setBounds(bounds, elementID);

		return isFastMoving;
	}

	virtual void runInternal()
	{
		PxU32 activeShapes = 0;
		const PxU32 nb = mNbToProcess;
		for(PxU32 i=0; i<nb; i++)
		{
			PxU32 isFastMoving = 0;
			Sc::BodySim& bodySim = *mBodySims[i];

			PxU32 nbElems = bodySim.getNbElements();
			Sc::ElementSim** elems = bodySim.getElements();
			while(nbElems--)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(*elems++);
				if(sim->getFlags() & PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
				{
					const PxIntBool fastMovingShape = updateSweptBounds(sim, &bodySim);
					activeShapes += fastMovingShape;

					isFastMoving = isFastMoving | fastMovingShape;
				}
			}

			bodySim.getLowLevelBody().getCore().isFastMoving = isFastMoving!=0;
		}

		PxAtomicAdd(mNumFastMovingShapes, PxI32(activeShapes));
	}
};

//CCD
void Sc::Scene::ccdBroadPhaseAABB(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Sim.ccdBroadPhaseComplete", getContextId());
	PX_PROFILE_ZONE("Sim.ccdBroadPhaseAABB", getContextId());
	PX_UNUSED(continuation);

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mNumFastMovingShapes = 0;

	//If we are on the 1st pass or we had some sweep hits previous CCD pass, we need to run CCD again
	if( currentPass == 0 || mCCDContext->getNumSweepHits())
	{
		PxsTransformCache& transformCache = getLowLevelContext()->getTransformCache();
		for (PxU32 i = 0; i < mCcdBodies.size(); i+= UpdateCCDBoundsTask::MaxPerTask)
		{
			const PxU32 nbToProcess = PxMin(UpdateCCDBoundsTask::MaxPerTask, mCcdBodies.size() - i);
			UpdateCCDBoundsTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateCCDBoundsTask)), UpdateCCDBoundsTask)(getContextId(), mBoundsArray, &transformCache, &mCcdBodies[i], nbToProcess, &mNumFastMovingShapes);
			task->setContinuation(continuation);
			task->removeReference();
		}
	}
}

//CCD
void Sc::Scene::ccdBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.ccdBroadPhase", getContextId());

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();
	const PxU32 ccdMaxPasses = mCCDContext->getCCDMaxPasses();
	mCCDPass = currentPass+1;

	//If we are on the 1st pass or we had some sweep hits previous CCD pass, we need to run CCD again
	if( (currentPass == 0 || mCCDContext->getNumSweepHits()) && mNumFastMovingShapes != 0)
	{
		const PxU32 currIndex = currentPass & 1;
		const PxU32 nextIndex = 1 - currIndex;
		//Initialize the CCD task chain unless this is the final pass
		if(currentPass != (ccdMaxPasses - 1))
		{
			mCCDBroadPhase[nextIndex].setContinuation(continuation);
			mCCDBroadPhaseAABB[nextIndex].setContinuation(&mCCDBroadPhase[nextIndex]);
		}
		mPostCCDPass[currIndex].setContinuation(currentPass == ccdMaxPasses-1 ? continuation : &mCCDBroadPhaseAABB[nextIndex]);
		mUpdateCCDSinglePass3[currIndex].setContinuation(&mPostCCDPass[currIndex]);
		mUpdateCCDSinglePass2[currIndex].setContinuation(&mUpdateCCDSinglePass3[currIndex]);
		mUpdateCCDSinglePass[currIndex].setContinuation(&mUpdateCCDSinglePass2[currIndex]);

		//Do the actual broad phase
		PxBaseTask* continuationTask = &mUpdateCCDSinglePass[currIndex];
//		const PxU32 numCpuTasks = continuationTask->getTaskManager()->getCpuDispatcher()->getWorkerCount();

		mCCDBp = true;

		mBpSecondPass.setContinuation(continuationTask);
		mBpFirstPass.setContinuation(&mBpSecondPass);

		mBpSecondPass.removeReference();
		mBpFirstPass.removeReference();
		
		//mAABBManager->updateAABBsAndBP(numCpuTasks, mLLContext->getTaskPool(), &mLLContext->getScratchAllocator(), false, continuationTask, NULL);

		//Allow the CCD task chain to continue
		mPostCCDPass[currIndex].removeReference();
		mUpdateCCDSinglePass3[currIndex].removeReference();
		mUpdateCCDSinglePass2[currIndex].removeReference();
		mUpdateCCDSinglePass[currIndex].removeReference();
		if(currentPass != (ccdMaxPasses - 1))
		{
			mCCDBroadPhase[nextIndex].removeReference();
			mCCDBroadPhaseAABB[nextIndex].removeReference();
		}
	}
	else if (currentPass == 0)
	{
		PX_PROFILE_STOP_CROSSTHREAD("Sim.ccdBroadPhaseComplete", getContextId());
		mCCDContext->resetContactManagers();
	}
}

//CCD
void Sc::Scene::updateCCDSinglePass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePass", getContextId());
	mReportShapePairTimeStamp++;  // This will makes sure that new report pairs will get created instead of re-using the existing ones.

	mAABBManager->postBroadPhase(NULL, *getFlushPool());
	finishBroadPhase(continuation);
	
	const PxU32 currentPass = mCCDContext->getCurrentCCDPass() + 1;  // 0 is reserved for discrete collision phase
	if(currentPass == 1)		// reset the handle map so we only update CCD objects from here on
	{
		PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
		//changedAABBMgrActorHandles.clear();
		for(PxU32 i = 0; i < mCcdBodies.size();i++)
		{
			// PT: ### changedMap pattern #1
			PxU32 nbElems = mCcdBodies[i]->getNbElements();
			Sc::ElementSim** elems = mCcdBodies[i]->getElements();
			while (nbElems--)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(*elems++);
				if (sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))	// TODO: need trigger shape here?
					changedAABBMgrActorHandles.growAndSet(sim->getElementID());
			}
		}
	}
}

//CCD
void Sc::Scene::updateCCDSinglePassStage2(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePassStage2", getContextId());
	postBroadPhaseStage2(continuation);
}

//CCD
void Sc::Scene::updateCCDSinglePassStage3(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePassStage3", getContextId());
	mReportShapePairTimeStamp++;  // This will makes sure that new report pairs will get created instead of re-using the existing ones.

	const PxU32 currentPass = mCCDContext->getCurrentCCDPass() + 1;  // 0 is reserved for discrete collision phase
	finishBroadPhaseStage2(currentPass);
	PX_PROFILE_STOP_CROSSTHREAD("Sim.ccdBroadPhaseComplete", getContextId());

	//reset thread context in a place we know all tasks possibly accessing it, are in sync with. (see US6664)
	mLLContext->resetThreadContexts();

	mCCDContext->updateCCD(mDt, continuation, mSimpleIslandManager->getAccurateIslandSim(), (mPublicFlags & PxSceneFlag::eDISABLE_CCD_RESWEEP), mNumFastMovingShapes);
}

class ScKinematicPoseUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*		mKinematics;
	PxU32					mNbKinematics;

public:
	static const PxU32 NbKinematicsPerTask = 1024;

	ScKinematicPoseUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbKinematics; ++a)
		{
			if ((a + 16) < mNbKinematics)
			{
				PxPrefetchLine(static_cast<Sc::BodyCore* const>(mKinematics[a + 16]));

				if ((a + 4) < mNbKinematics)
				{
					PxPrefetchLine(static_cast<Sc::BodyCore* const>(mKinematics[a + 4])->getSim());
					PxPrefetchLine(static_cast<Sc::BodyCore* const>(mKinematics[a + 4])->getSim()->getSimStateData_Unchecked());
				}
			}
			Sc::BodyCore* b = static_cast<Sc::BodyCore* const>(mKinematics[a]);
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());
			b->getSim()->updateKinematicPose();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.ScKinematicPoseUpdateTask";
	}
};

void Sc::Scene::integrateKinematicPose()
{
	PX_PROFILE_ZONE("Sim.integrateKinematicPose", getContextId());

	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	for(PxU32 i=0; i<nbKinematics; i+= ScKinematicPoseUpdateTask::NbKinematicsPerTask)
	{
		ScKinematicPoseUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicPoseUpdateTask)), ScKinematicPoseUpdateTask)
			(kinematics + i, PxMin(nbKinematics - i, ScKinematicPoseUpdateTask::NbKinematicsPerTask), mContextId);
		task->setContinuation(&mConstraintProjection);
		task->removeReference();
	}
}

class ScKinematicShapeUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*		mKinematics;
	PxU32					mNbKinematics;
	PxsTransformCache&		mCache;
	Bp::BoundsArray&		mBoundsArray;

	PX_NOCOPY(ScKinematicShapeUpdateTask)

public:
	static const PxU32 NbKinematicsShapesPerTask = 1024;

	ScKinematicShapeUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxsTransformCache& cache, Bp::BoundsArray& boundsArray, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mCache(cache), mBoundsArray(boundsArray)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbKinematics; ++a)
		{
			Sc::BodyCore* b = static_cast<Sc::BodyCore*>(mKinematics[a]);
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());

			b->getSim()->updateCached(mCache, mBoundsArray);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicShapeUpdateTask";
	}
};

void Sc::Scene::updateKinematicCached(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateKinematicCached", getContextId());

	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();
	
	PxU32 startIndex = 0;
	PxU32 nbShapes = 0;

	{
		PX_PROFILE_ZONE("ShapeUpdate", getContextId());
		for(PxU32 i=0; i<nbKinematics; i++)
		{
			Sc::BodySim* sim = static_cast<Sc::BodyCore*>(kinematics[i])->getSim();
			PX_ASSERT(sim->isKinematic());
			PX_ASSERT(sim->isActive());

			nbShapes += sim->getNbShapes();

			if (nbShapes >= ScKinematicShapeUpdateTask::NbKinematicsShapesPerTask)
			{
				ScKinematicShapeUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicShapeUpdateTask)), ScKinematicShapeUpdateTask)
					(kinematics + startIndex, (i + 1) - startIndex, mLLContext->getTransformCache(), *mBoundsArray, mContextId);

				task->setContinuation(continuation);
				task->removeReference();
				startIndex = i + 1;
				nbShapes = 0;
			}
		}

		if(nbShapes)
		{
			ScKinematicShapeUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicShapeUpdateTask)), ScKinematicShapeUpdateTask)
				(kinematics + startIndex, nbKinematics - startIndex, mLLContext->getTransformCache(), *mBoundsArray, mContextId);

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	if(nbKinematics)
	{
		PxBitMapPinned& changedAABBMap = mAABBManager->getChangedAABBMgActorHandleMap();
		mLLContext->getTransformCache().setChangedState();
		mBoundsArray->setChangedState();
		for (PxU32 i = 0; i < nbKinematics; ++i)
		{
			Sc::BodySim* bodySim = static_cast<Sc::BodyCore*>(kinematics[i])->getSim();

			if ((i+16) < nbKinematics)
			{
				PxPrefetchLine(kinematics[i + 16]);
				if ((i + 8) < nbKinematics)
				{
					PxPrefetchLine(kinematics[i + 8]->getSim());
				}
			}

			// PT: ### changedMap pattern #1
			PxU32 nbElems = bodySim->getNbElements();
			Sc::ElementSim** elems = bodySim->getElements();
			while (nbElems--)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(*elems++);
				//KS - TODO - can we parallelize this? The problem with parallelizing is that it's a bit operation,
				//so we would either need to use atomic operations or have some high-level concept that guarantees 
				//that threads don't write to the same word in the map simultaneously
				if (sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
				{
					changedAABBMap.set(sim->getElementID());
				}
			}

			mSimulationController->updateDynamic(NULL, bodySim->getNodeIndex());
		}
	}
}

class ConstraintProjectionTask : public Cm::Task
{
private:
	PX_NOCOPY(ConstraintProjectionTask)

public:
	ConstraintProjectionTask(Sc::ConstraintGroupNode* const* projectionRoots, PxU32 projectionRootCount, PxArray<Sc::BodySim*>& projectedBodies, PxsContext* llContext) :
		Cm::Task			(llContext->getContextId()),
		mProjectionRoots	(projectionRoots),
		mProjectionRootCount(projectionRootCount),
		mProjectedBodies	(projectedBodies),
		mLLContext			(llContext)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("ConstraintProjection", mContextID);
		PxcNpThreadContext* context = mLLContext->getNpThreadContext();
		PxArray<Sc::BodySim*>& tempArray = context->mBodySimPool;
		tempArray.forceSize_Unsafe(0);
		for(PxU32 i=0; i < mProjectionRootCount; i++)
		{
			PX_ASSERT(mProjectionRoots[i]->hasProjectionTreeRoot());  // else, it must not be in the projection root list
			Sc::ConstraintGroupNode::projectPose(*mProjectionRoots[i], tempArray);
			mProjectionRoots[i]->clearFlag(Sc::ConstraintGroupNode::eIN_PROJECTION_PASS_LIST);
		}

		if (tempArray.size() > 0)
		{
			mLLContext->getLock().lock();
			for (PxU32 a = 0; a < tempArray.size(); ++a)
				mProjectedBodies.pushBack(tempArray[a]);
			mLLContext->getLock().unlock();
		}

		mLLContext->putNpThreadContext(context);
	}

	virtual const char* getName() const
	{
		return "ScScene.constraintProjectionWork";
	}

public:
	static const PxU32 sProjectingConstraintsPerTask = 256;  // just a guideline, will not match exactly most of the time

private:
	Sc::ConstraintGroupNode* const* mProjectionRoots;
	const PxU32 mProjectionRootCount;
	PxArray<Sc::BodySim*>& mProjectedBodies;
	PxsContext* mLLContext;
};

void Sc::Scene::constraintProjection(PxBaseTask* continuation)
{
	if(mConstraints.size() == 0)
		return;
	PxU32 constraintGroupRootCount = 0;
	//BodyCore*const* activeBodies = getActiveBodiesArray();
	//PxU32 activeBodyCount = getNumActiveBodies();
	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();
	PxU32 activeBodyCount = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
	const PxNodeIndex* activeNodeIds = islandSim.getActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	PX_ASSERT(!mTmpConstraintGroupRootBuffer);
	PxU32 index = 0;

	const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

	if(activeBodyCount)
	{
		mTmpConstraintGroupRootBuffer = reinterpret_cast<ConstraintGroupNode**>(mLLContext->getScratchAllocator().alloc(sizeof(ConstraintGroupNode*) * activeBodyCount, true));
		if(mTmpConstraintGroupRootBuffer)
		{
			while(activeBodyCount--)
			{
				PxsRigidBody* rBody = islandSim.getRigidBody(activeNodeIds[index++]);

				Sc::BodySim* sim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rBody) - rigidBodyOffset);
				//This move to PxgPostSolveWorkerTask for the gpu dynamic
				//bodySim->sleepCheck(mDt, mOneOverDt, mEnableStabilization);

				if(sim->getConstraintGroup())
				{
					ConstraintGroupNode& root = sim->getConstraintGroup()->getRoot();
					if(!root.readFlag(ConstraintGroupNode::eIN_PROJECTION_PASS_LIST) && root.hasProjectionTreeRoot())
					{
						mTmpConstraintGroupRootBuffer[constraintGroupRootCount++] = &root;
						root.raiseFlag(ConstraintGroupNode::eIN_PROJECTION_PASS_LIST);
					}
				}
			}

			Cm::FlushPool& flushPool = mLLContext->getTaskPool();

			PxU32 constraintsToProjectCount = 0;
			PxU32 startIndex = 0;
			for(PxU32 i=0; i < constraintGroupRootCount; i++)
			{
				ConstraintGroupNode* root = mTmpConstraintGroupRootBuffer[i];

				constraintsToProjectCount += root->getProjectionCountHint();  // for load balancing
				if (constraintsToProjectCount >= ConstraintProjectionTask::sProjectingConstraintsPerTask)
				{
					ConstraintProjectionTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ConstraintProjectionTask)), 
																		ConstraintProjectionTask(mTmpConstraintGroupRootBuffer + startIndex, i - startIndex + 1, mProjectedBodies, mLLContext));
					task->setContinuation(continuation);
					task->removeReference();

					constraintsToProjectCount = 0;
					startIndex = i + 1;
				}
			}

			if (constraintsToProjectCount)
			{
				PX_ASSERT(startIndex < constraintGroupRootCount);

				ConstraintProjectionTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ConstraintProjectionTask)), 
																	ConstraintProjectionTask(mTmpConstraintGroupRootBuffer + startIndex, constraintGroupRootCount - startIndex, mProjectedBodies, mLLContext));
				task->setContinuation(continuation);
				task->removeReference();
			}
		}
		else
		{
			outputError<PxErrorCode::eOUT_OF_MEMORY>(__LINE__, "List for collecting constraint projection roots could not be allocated. No projection will take place.");
		}
	}
}

void Sc::Scene::postSolver(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::postSolver", getContextId());
	PxcNpMemBlockPool& blockPool = mLLContext->getNpMemBlockPool();

	//Merge...
	mDynamicsContext->mergeResults();
	blockPool.releaseConstraintMemory();
	//Swap friction!
	blockPool.swapFrictionStreams();

	mCcdBodies.clear();
	mProjectedBodies.clear();

#if PX_ENABLE_SIM_STATS
	mLLContext->getSimStats().mPeakConstraintBlockAllocations = blockPool.getPeakConstraintBlockCount();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	mConstraintProjection.setContinuation(continuation);

	integrateKinematicPose();

	mConstraintProjection.removeReference();

	PxU32 size = mDirtyArticulationSims.size();
	Sc::ArticulationSim* const* articSims = mDirtyArticulationSims.getEntries();
	//clear the acceleration term for articulation if the application raised PxForceMode::eIMPULSE in addForce function. This change
	//will make sure articulation and rigid body behave the same
	for (PxU32 i = 0; i < size; ++i)
	{
		Sc::ArticulationSim* PX_RESTRICT articSim = articSims[i];
		articSim->clearAcceleration(mDt);
	}

	//clear the dirty articulation list
	mDirtyArticulationSims.clear();

	//afterIntegration(continuation);
}

//CCD
void Sc::Scene::postCCDPass(PxBaseTask* /*continuation*/)
{
	// - Performs sleep check
	// - Updates touch flags

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();
	PX_ASSERT(currentPass > 0); // to make sure changes to the CCD pass counting get noticed. For contact reports, 0 means discrete collision phase.

	int newTouchCount, lostTouchCount, ccdTouchCount;
	mLLContext->getManagerTouchEventCount(&newTouchCount, &lostTouchCount, &ccdTouchCount);
	PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
	PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);
	PX_ALLOCA(ccdTouches, PxvContactManagerTouchEvent, ccdTouchCount);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	// Note: For contact notifications it is important that the new touch pairs get processed before the lost touch pairs.
	//       This allows to know for sure if a pair of actors lost all touch (see eACTOR_PAIR_LOST_TOUCH).
	mLLContext->fillManagerTouchEvents(newTouches, newTouchCount, lostTouches, lostTouchCount, ccdTouches, ccdTouchCount);
	for(PxI32 i=0; i<newTouchCount; ++i)
	{
		ShapeInteraction* si = getSI(newTouches[i]);
		PX_ASSERT(si);
		mNPhaseCore->managerNewTouch(*si);
		si->managerNewTouch(currentPass, true, outputs);
		if (!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
		{
			mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex(), IG::Edge::eCONTACT_MANAGER);
		}
	}
	for(PxI32 i=0; i<lostTouchCount; ++i)
	{
		ShapeInteraction* si = getSI(lostTouches[i]);
		PX_ASSERT(si);
		if (si->managerLostTouch(currentPass, true, outputs) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
			addToLostTouchList(si->getShape0().getActor(), si->getShape1().getActor());

		mSimpleIslandManager->setEdgeDisconnected(si->getEdgeIndex());
	}
	for(PxI32 i=0; i<ccdTouchCount; ++i)
	{
		ShapeInteraction* si = getSI(ccdTouches[i]);
		PX_ASSERT(si);
		si->sendCCDRetouch(currentPass, outputs);
	}
	checkForceThresholdContactEvents(currentPass);
	{
		PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();

		for (PxU32 i = 0, s = mCcdBodies.size(); i < s; i++)
		{
			BodySim*const body = mCcdBodies[i];
			if(i+8 < s)
				PxPrefetch(mCcdBodies[i+8], 512);

			PX_ASSERT(body->getBody2World().p.isFinite());
			PX_ASSERT(body->getBody2World().q.isFinite());

			body->updateCached(&changedAABBMgrActorHandles);
		}

		ArticulationCore* const* articList = mArticulations.getEntries();
		for(PxU32 i=0;i<mArticulations.size();i++)
			articList[i]->getSim()->updateCached(&changedAABBMgrActorHandles);
	}
}

void Sc::Scene::finalizationPhase(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.sceneFinalization", getContextId());

	if (mCCDContext)
	{
		//KS - force simulation controller to update any bodies updated by the CCD. When running GPU simulation, this would be required
		//to ensure that cached body states are updated
		const PxU32 nbUpdatedBodies = mCCDContext->getNumUpdatedBodies();
		PxsRigidBody*const* updatedBodies = mCCDContext->getUpdatedBodies();

		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();			

		for (PxU32 a = 0; a < nbUpdatedBodies; ++a)
		{
			Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(updatedBodies[a]) - rigidBodyOffset);
			updateBodySim(*bodySim);
		}

		mCCDContext->clearUpdatedBodies();
	}

	if (mTmpConstraintGroupRootBuffer)
	{
		mLLContext->getScratchAllocator().free(mTmpConstraintGroupRootBuffer);
		mTmpConstraintGroupRootBuffer = NULL;
	}

	fireOnAdvanceCallback();  // placed here because it needs to be done after sleep check and after potential CCD passes

	checkConstraintBreakage(); // Performs breakage tests on breakable constraints

	PX_PROFILE_STOP_CROSSTHREAD("Basic.rigidBodySolver", getContextId());

	mTaskPool.clear();

	mReportShapePairTimeStamp++;	// important to do this before fetchResults() is called to make sure that delayed deleted actors/shapes get
									// separate pair entries in contact reports
}

void Sc::Scene::postReportsCleanup()
{
	mElementIDPool->processPendingReleases();
	mElementIDPool->clearDeletedIDMap();

	mActorIDTracker->processPendingReleases();
	mActorIDTracker->clearDeletedIDMap();

	mConstraintIDTracker->processPendingReleases();
	mConstraintIDTracker->clearDeletedIDMap();

	mSimulationController->flush();
}

PX_COMPILE_TIME_ASSERT(sizeof(PxTransform32)==sizeof(PxsCachedTransform));

// PT: TODO: move this out of Sc? this is only called by Np
void Sc::Scene::syncSceneQueryBounds(SqBoundsSync& sync, SqRefFinder& finder)
{
	const PxsTransformCache& cache = mLLContext->getTransformCache();

	mSqBoundsManager->syncBounds(sync, finder, mBoundsArray->begin(), reinterpret_cast<const PxTransform32*>(cache.getTransforms()), getContextId(), mDirtyShapeSimMap);
}

class ScKinematicUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*	mKinematics;
	const PxU32			mNbKinematics;
	const PxReal		mOneOverDt;

	PX_NOCOPY(ScKinematicUpdateTask)
public:

	static const PxU32 NbKinematicsPerTask = 1024;
	
	ScKinematicUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxReal oneOverDt, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mOneOverDt(oneOverDt)
	{
	}

	virtual void runInternal()
	{
		Sc::BodyCore*const*	kinematics = mKinematics;
		PxU32 nb = mNbKinematics;
		const float oneOverDt = mOneOverDt;

		while(nb--)
		{
			Sc::BodyCore* b = *kinematics++;
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());

			b->getSim()->calculateKinematicVelocity(oneOverDt);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicUpdateTask";
	}
};

void Sc::Scene::kinematicsSetup(PxBaseTask* continuation)
{
	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	if(!nbKinematics)
		return;

	BodyCore*const* kinematics = getActiveKinematicBodies();

	// PT: create a copy of active bodies for the taks to operate on while the main array is
	// potentially resized by operations running in parallel.
	if(mActiveKinematicsCopyCapacity<nbKinematics)
	{
		PX_FREE(mActiveKinematicsCopy);
		mActiveKinematicsCopy = PX_ALLOCATE(BodyCore*, nbKinematics, "Sc::Scene::mActiveKinematicsCopy");
		mActiveKinematicsCopyCapacity = nbKinematics;
	}
	PxMemCopy(mActiveKinematicsCopy, kinematics, nbKinematics*sizeof(BodyCore*));
	kinematics = mActiveKinematicsCopy;

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	// PT: TODO: better load balancing? This will be single threaded for less than 1K kinematics
	for(PxU32 i = 0; i < nbKinematics; i += ScKinematicUpdateTask::NbKinematicsPerTask)
	{
		ScKinematicUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicUpdateTask)), ScKinematicUpdateTask)
			(kinematics + i, PxMin(ScKinematicUpdateTask::NbKinematicsPerTask, nbKinematics - i), mOneOverDt, mContextId);

		task->setContinuation(continuation);
		task->removeReference();
	}

	if((mPublicFlags & PxSceneFlag::eENABLE_GPU_DYNAMICS))
	{
		// PT: running this serially for now because it's unsafe: mNPhaseCore->updateDirtyInteractions() (called after this)
		// can also call mSimulationController.updateDynamic() via BodySim::internalWakeUpBase
		PxU32 nb = nbKinematics;
		while(nb--)
		{
			Sc::BodyCore* b = *kinematics++;
			Sc::BodySim* bodySim = b->getSim();
			PX_ASSERT(!bodySim->getArticulation());
			mSimulationController->updateDynamic(NULL, bodySim->getNodeIndex());
		}
	}
}

//stepSetup is called in solve, but not collide
void Sc::Scene::stepSetupSolve(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.stepSetupSolve", getContextId());

	kinematicsSetup(continuation);
}

void Sc::Scene::stepSetupCollide(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.stepSetupCollide", getContextId());

	{
		PX_PROFILE_ZONE("Sim.projectionTreeUpdates", getContextId());
		mProjectionManager->processPendingUpdates(mLLContext->getScratchAllocator());
	}

	kinematicsSetup(continuation);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	// Update all dirty interactions
	mNPhaseCore->updateDirtyInteractions(outputs);
	mInternalFlags &= ~(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE | SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION);
}

void Sc::Scene::processLostTouchPairs()
{
	PX_PROFILE_ZONE("Sc::Scene::processLostTouchPairs", getContextId());
	for (PxU32 i=0; i<mLostTouchPairs.size(); ++i)
	{
		// If one has been deleted, we wake the other one
		const PxIntBool deletedBody1 = mLostTouchPairsDeletedBodyIDs.boundedTest(mLostTouchPairs[i].body1ID);
		const PxIntBool deletedBody2 = mLostTouchPairsDeletedBodyIDs.boundedTest(mLostTouchPairs[i].body2ID);
		if (deletedBody1 || deletedBody2)
		{
			if (!deletedBody1) 
				mLostTouchPairs[i].body1->internalWakeUp();
			if (!deletedBody2) 
				mLostTouchPairs[i].body2->internalWakeUp();
			continue;
		}

		// If both are sleeping, we let them sleep
		// (for example, two sleeping objects touch and the user teleports one (without waking it up))
		if (!mLostTouchPairs[i].body1->isActive() &&
			!mLostTouchPairs[i].body2->isActive())
		{
			continue;
		}

		// If only one has fallen asleep, we wake them both
		if (!mLostTouchPairs[i].body1->isActive() ||
			!mLostTouchPairs[i].body2->isActive())
		{
			mLostTouchPairs[i].body1->internalWakeUp();
			mLostTouchPairs[i].body2->internalWakeUp();
		}
	}

	mLostTouchPairs.clear();
	mLostTouchPairsDeletedBodyIDs.clear();
}

class ScBeforeSolverTask :  public Cm::Task
{
public:
	static const PxU32 MaxBodiesPerTask = 256;
	PxNodeIndex				mBodies[MaxBodiesPerTask];
	PxU32						mNumBodies;
	const PxReal				mDt;
	IG::SimpleIslandManager*	mIslandManager;
	PxsSimulationController*	mSimulationController;

public:

	ScBeforeSolverTask(PxReal dt, IG::SimpleIslandManager* islandManager, PxsSimulationController* simulationController, PxU64 contextID) : 
		Cm::Task				(contextID),
		mDt						(dt),
		mIslandManager			(islandManager),
		mSimulationController	(simulationController)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.ScBeforeSolverTask", mContextID);
		const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();
		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		PxsRigidBody* updatedBodySims[MaxBodiesPerTask];
		PxU32 updatedBodyNodeIndices[MaxBodiesPerTask];
		PxU32 nbUpdatedBodySims = 0;

		PxU32 nb = mNumBodies;
		const PxNodeIndex* bodies = mBodies;
		while(nb--)
		{
			const PxNodeIndex index = *bodies++;

			if(islandSim.getActiveNodeIndex(index) != PX_INVALID_NODE)
			{
				if (islandSim.getNode(index).mType == IG::Node::eRIGID_BODY_TYPE)
				{
					PxsRigidBody* body = islandSim.getRigidBody(index);
					Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(body) - rigidBodyOffset);
					bodySim->updateForces(mDt, updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims, NULL);
				}
			}
		}

		if(nbUpdatedBodySims)
			mSimulationController->updateBodies(updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims);
	}

	virtual const char* getName() const
	{
		return "ScScene.beforeSolver";
	}

private:
	PX_NOCOPY(ScBeforeSolverTask)
};

class ScArticBeforeSolverCCDTask : public Cm::Task
{
public:
	const PxNodeIndex* const	mArticIndices;
	const PxU32					mNumArticulations;
	const PxReal				mDt;
	IG::SimpleIslandManager*	mIslandManager;

public:

	ScArticBeforeSolverCCDTask(const PxNodeIndex* const	articIndices, PxU32 nbArtics, PxReal dt, IG::SimpleIslandManager* islandManager, PxU64 contextID) :
		Cm::Task(contextID),
		mArticIndices(articIndices),
		mNumArticulations(nbArtics),
		mDt(dt),
		mIslandManager(islandManager)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.ScArticBeforeSolverCCDTask", mContextID);
		const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();

		for (PxU32 a = 0; a < mNumArticulations; ++a)
		{
			Sc::ArticulationSim* articSim = islandSim.getArticulationSim(mArticIndices[a]);

			articSim->saveLastCCDTransform();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.ScArticBeforeSolverCCDTask";
	}

private:
	PX_NOCOPY(ScArticBeforeSolverCCDTask)
};

class ScArticBeforeSolverTask : public Cm::Task
{
public:
	Sc::ArticulationSim* const*			mArticSims;
	const PxU32							mNumArticulations;
	const PxReal						mDt;
	IG::SimpleIslandManager*			mIslandManager;

public:

	ScArticBeforeSolverTask(Sc::ArticulationSim* const* articSims, PxU32 nbArtics, PxReal dt, IG::SimpleIslandManager* islandManager, PxU64 contextID) :
		Cm::Task(contextID),
		mArticSims(articSims),
		mNumArticulations(nbArtics),
		mDt(dt),
		mIslandManager(islandManager)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.ScArticBeforeSolverTask", mContextID);
		//const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();

		for (PxU32 a = 0; a < mNumArticulations; ++a)
		{
			Sc::ArticulationSim* PX_RESTRICT articSim = mArticSims[a];
			//articSim->checkResize();
			articSim->updateForces(mDt, false);
			articSim->setDirtyFlag(Sc::ArticulationSimDirtyFlag::eNONE);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.ScArticBeforeSolverTask";
	}

private:
	PX_NOCOPY(ScArticBeforeSolverTask)
};

void Sc::Scene::beforeSolver(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateForces", getContextId());

	// Note: For contact notifications it is important that force threshold checks are done after new/lost touches have been processed
	//       because pairs might get added to the list processed below

	// Atoms that passed contact force threshold
	ThresholdStream& thresholdStream = mDynamicsContext->getThresholdStream();
	thresholdStream.clear();

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbActiveBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	mNumDeactivatingNodes[IG::Node::eRIGID_BODY_TYPE] = 0;//islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	mNumDeactivatingNodes[IG::Node::eARTICULATION_TYPE] = 0;//islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);
	mNumDeactivatingNodes[IG::Node::eSOFTBODY_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::eFEMCLOTH_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::ePARTICLESYSTEM_TYPE] = 0;
	mNumDeactivatingNodes[IG::Node::eHAIRSYSTEM_TYPE] = 0;

	const PxU32 MaxBodiesPerTask = ScBeforeSolverTask::MaxBodiesPerTask;

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mSimulationController->reserve(nbActiveBodies);

	{
		PxBitMap::Iterator iter(mVelocityModifyMap);

		for (PxU32 i = iter.getNext(); i != PxBitMap::Iterator::DONE; /*i = iter.getNext()*/)
		{
			ScBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScBeforeSolverTask)), ScBeforeSolverTask(mDt, mSimpleIslandManager, mSimulationController, getContextId()));
			PxU32 count = 0;
			for (; count < MaxBodiesPerTask && i != PxBitMap::Iterator::DONE; i = iter.getNext())
			{
				PxsRigidBody* body = islandSim.getRigidBody(PxNodeIndex(i));
				bool retainsAccelerations = false;
				if (body)
				{
					task->mBodies[count++] = PxNodeIndex(i);

					retainsAccelerations = (body->mCore->mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS);
				}

				if(!retainsAccelerations)
				{
					mVelocityModifyMap.reset(i);
				}
			}
			task->mNumBodies = count;
			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	const PxU32 nbArticsPerTask = 32;

	const PxU32 nbDirtyArticulations = mDirtyArticulationSims.size();
	Sc::ArticulationSim* const* artiSim = mDirtyArticulationSims.getEntries();
	for (PxU32 a = 0; a < nbDirtyArticulations; a += nbArticsPerTask)
	{
		const PxU32 nbToProcess = PxMin(PxU32(nbDirtyArticulations - a), nbArticsPerTask);

		ScArticBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScArticBeforeSolverTask)), ScArticBeforeSolverTask(artiSim + a, nbToProcess,
			mDt, mSimpleIslandManager, getContextId()));

		task->setContinuation(continuation);
		task->removeReference();
	}

	//if the scene has ccd flag on, we should call ScArticBeforeSolverCCDTask to copy the last transform to the current transform
	if (mPublicFlags & PxSceneFlag::eENABLE_CCD)
	{
		//CCD
		const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);
		const PxNodeIndex* const articIndices = islandSim.getActiveNodes(IG::Node::eARTICULATION_TYPE);

		for (PxU32 a = 0; a < nbActiveArticulations; a += nbArticsPerTask)
		{
			const PxU32 nbToProcess = PxMin(PxU32(nbActiveArticulations - a), nbArticsPerTask);
			ScArticBeforeSolverCCDTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScArticBeforeSolverCCDTask)), ScArticBeforeSolverCCDTask(articIndices + a, nbToProcess,
				mDt, mSimpleIslandManager, getContextId()));

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	for (PxU32 a = 0; a < nbDirtyArticulations; a++)
	{
		mSimulationController->updateArticulationExtAccel(artiSim[a]->getLowLevelArticulation(), artiSim[a]->getIslandNodeIndex());
	}

	mBodyGravityDirty = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class UpdateProjectedPoseTask : public Cm::Task
{
	Sc::BodySim** mProjectedBodies;
	const PxU32 mNbBodiesToProcess;

	PX_NOCOPY(UpdateProjectedPoseTask)
public:

	UpdateProjectedPoseTask(PxU64 contextID, Sc::BodySim** projectedBodies, PxU32 nbBodiesToProcess) :
		Cm::Task			(contextID),
		mProjectedBodies	(projectedBodies),
		mNbBodiesToProcess	(nbBodiesToProcess)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbBodiesToProcess; ++a)
		{
			mProjectedBodies[a]->updateCached(NULL);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.UpdateProjectedPoseTask";
	}
};

void Sc::Scene::afterIntegration(PxBaseTask* continuation)
{		
	PX_PROFILE_ZONE("Sc::Scene::afterIntegration", getContextId());
	mLLContext->getTransformCache().resetChangedState(); //Reset the changed state. If anything outside of the GPU kernels updates any shape's transforms, this will be raised again
	getBoundsArray().resetChangedState();

	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	Bp::BoundsArray& boundArray = getBoundsArray();

	{
		PX_PROFILE_ZONE("AfterIntegration::lockStage", getContextId());
		mLLContext->getLock().lock();
		
		{
			PX_PROFILE_ZONE("SimController", getContextId());
			mSimulationController->updateScBodyAndShapeSim(cache, boundArray, continuation);
		}

		const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		const PxU32 numBodiesToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

		const PxNodeIndex*const deactivatingIndices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

		PxU32 previousNumBodiesToDeactivate = mNumDeactivatingNodes[IG::Node::eRIGID_BODY_TYPE];

		{
			PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
			PX_PROFILE_ZONE("AfterIntegration::deactivateStage", getContextId());
			for (PxU32 i = previousNumBodiesToDeactivate; i < numBodiesToDeactivate; i++)
			{
				PxsRigidBody* rigid = islandSim.getRigidBody(deactivatingIndices[i]);
				Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
				//we need to set the rigid body back to the previous pose for the deactivated objects. This emulates the previous behavior where island gen ran before the solver, ensuring
				//that bodies that should be deactivated this frame never reach the solver. We now run the solver in parallel with island gen, so objects that should be deactivated this frame
				//still reach the solver and are integrated. However, on the frame when they should be deactivated, we roll back to their state at the beginning of the frame to ensure that the
				//user perceives the same behavior as before.

				PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();

				//if(!islandSim.getNode(bodySim->getNodeIndex()).isActive())
				rigid->setPose(rigid->getLastCCDTransform());

				bodySim->updateCached(&changedAABBMgrActorHandles);
				updateBodySim(*bodySim);

				//solver is running in parallel with IG(so solver might solving the body which IG identify as deactivatedNodes). After we moved sleepCheck into the solver after integration, sleepChecks
				//might have processed bodies that are now considered deactivated. This could have resulted in either freezing or unfreezing one of these bodies this frame, so we need to process those
				//events to ensure that the SqManager's bounds arrays are consistently maintained. Also, we need to clear the frame flags for these bodies.

				if (rigid->isFreezeThisFrame())
					bodySim->freezeTransforms(&mAABBManager->getChangedAABBMgActorHandleMap());

				//KS - the IG deactivates bodies in parallel with the solver. It appears that under certain circumstances, the solver's integration (which performs
				//sleep checks) could decide that the body is no longer a candidate for sleeping on the same frame that the island gen decides to deactivate the island
				//that the body is contained in. This is a rare occurrence but the behavior we want to emulate is that of IG running before solver so we should therefore
				//permit the IG to make the authoritative decision over whether the body should be active or inactive.
				bodyCore.wakeCounter = 0.0f;
				bodyCore.linearVelocity = PxVec3(0.0f);
				bodyCore.angularVelocity = PxVec3(0.0f);

				rigid->clearAllFrameFlags();
			}
		}

		const PxU32 maxBodiesPerTask = 256;

		Cm::FlushPool& flushPool = mLLContext->getTaskPool();

		{
			PX_PROFILE_ZONE("AfterIntegration::dispatchTasks", getContextId());
			for (PxU32 a = 0; a < mProjectedBodies.size(); a += maxBodiesPerTask)
			{
				UpdateProjectedPoseTask* task =
					PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateProjectedPoseTask)), UpdateProjectedPoseTask)(getContextId(), &mProjectedBodies[a], PxMin(maxBodiesPerTask, mProjectedBodies.size() - a));
				task->setContinuation(continuation);
				task->removeReference();
			}
		}

		{
			PxBitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
			PX_PROFILE_ZONE("AfterIntegration::growAndSet", getContextId());
			for (PxU32 a = 0; a < mProjectedBodies.size(); ++a)
			{
				if (!mProjectedBodies[a]->isFrozen())
				{
					// PT: ### changedMap pattern #1
					PxU32 nbElems = mProjectedBodies[a]->getNbElements();
					Sc::ElementSim** elems = mProjectedBodies[a]->getElements();
					while (nbElems--)
					{
						Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(*elems++);
						if (sim->isInBroadPhase())
							changedAABBMgrActorHandles.growAndSet(sim->getElementID());
					}
				}
			}
		}
		{
			PX_PROFILE_ZONE("AfterIntegration::managerAndDynamic", getContextId());
			const PxU32 unrollSize = 256;
			for (PxU32 a = 0; a < mProjectedBodies.size(); a += unrollSize)
			{
				PxsRigidBody* tempBodies[unrollSize];
				PxU32 nodeIds[unrollSize];
				const PxU32 nbToProcess = PxMin(unrollSize, mProjectedBodies.size() - a);
				for (PxU32 i = 0; i < nbToProcess; ++i)
				{
					tempBodies[i] = &mProjectedBodies[a + i]->getLowLevelBody();
					nodeIds[i] = mProjectedBodies[a + i]->getNodeIndex().index();
				}
				//KS - it seems that grabbing the CUDA context/releasing it is expensive so we should minimize how
				//frequently we do that. Batch processing like this helps
				mSimulationController->updateBodies(tempBodies, nodeIds, nbToProcess);
			}
		}

		updateKinematicCached(continuation);

		mLLContext->getLock().unlock();
	}

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);

	if(nbActiveArticulations)
	{
		mSimulationController->updateArticulationAfterIntegration(mLLContext, mAABBManager, mCcdBodies, continuation, islandSim, mDt);
	}

	const PxU32 numArticsToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	const PxNodeIndex*const deactivatingArticIndices = islandSim.getNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	PxU32 previousNumArticsToDeactivate = mNumDeactivatingNodes[IG::Node::eARTICULATION_TYPE];

	for (PxU32 i = previousNumArticsToDeactivate; i < numArticsToDeactivate; ++i)
	{
		Sc::ArticulationSim* artic = islandSim.getArticulationSim(deactivatingArticIndices[i]);

		artic->putToSleep();
	}

	//PxU32 previousNumClothToDeactivate = mNumDeactivatingNodes[IG::Node::eFEMCLOTH_TYPE];
	//const PxU32 numClothToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eFEMCLOTH_TYPE);
	//const IG::NodeIndex*const deactivatingClothIndices = islandSim.getNodesToDeactivate(IG::Node::eFEMCLOTH_TYPE);

	//for (PxU32 i = previousNumClothToDeactivate; i < numClothToDeactivate; ++i)
	//{
	//	FEMCloth* cloth = islandSim.getLLFEMCloth(deactivatingClothIndices[i]);
	//	mSimulationController->deactivateCloth(cloth);
	//}

	//PxU32 previousNumSoftBodiesToDeactivate = mNumDeactivatingNodes[IG::Node::eSOFTBODY_TYPE];
	//const PxU32 numSoftBodiesToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eSOFTBODY_TYPE);
	//const IG::NodeIndex*const deactivatingSoftBodiesIndices = islandSim.getNodesToDeactivate(IG::Node::eSOFTBODY_TYPE);

	//for (PxU32 i = previousNumSoftBodiesToDeactivate; i < numSoftBodiesToDeactivate; ++i)
	//{
	//	Dy::SoftBody* softbody = islandSim.getLLSoftBody(deactivatingSoftBodiesIndices[i]);
	//	printf("after Integration: Deactivating soft body %i\n", softbody->getGpuRemapId());
	//	//mSimulationController->deactivateSoftbody(softbody);
	//	softbody->getSoftBodySim()->setActive(false, 0);
	//}

	PX_PROFILE_STOP_CROSSTHREAD("Basic.dynamics", getContextId());

	checkForceThresholdContactEvents(0); 		
}

void Sc::Scene::checkForceThresholdContactEvents(const PxU32 ccdPass)
{
	PX_PROFILE_ZONE("Sim.checkForceThresholdContactEvents", getContextId());

	// Note: For contact notifications it is important that force threshold checks are done after new/lost touches have been processed
	//       because pairs might get added to the list processed below

	// Bodies that passed contact force threshold

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	ThresholdStream& thresholdStream = mDynamicsContext->getForceChangedThresholdStream();

	const PxU32 nbThresholdElements = thresholdStream.size();

	for (PxU32 i = 0; i< nbThresholdElements; ++i)
	{
		ThresholdStreamElement& elem = thresholdStream[i];
		ShapeInteraction* si = elem.shapeInteraction;

		//If there is a shapeInteraction and the shapeInteraction points to a contactManager (i.e. the CM was not destroyed in parallel with the solver)
		if (si != NULL)
		{
			PxU32 pairFlags = si->getPairFlags();
			if (pairFlags & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS)
			{
				si->swapAndClearForceThresholdExceeded();

				if (elem.accumulatedForce > elem.threshold * mDt)
				{
					si->raiseFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_NOW);

					PX_ASSERT(si->hasTouch());

					//If the accumulatedForce is large than the threshold in the current frame and the accumulatedForce is less than the threshold in the previous frame, 
					//and the user request notify for found event, we will raise eNOTIFY_THRESHOLD_FORCE_FOUND
					if ((!si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE)) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND))
					{
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND, 0, false, ccdPass, false, outputs);
					}
					else if (si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS))
					{
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS, 0, false, ccdPass, false, outputs);
					}
				}
				else
				{
					//If the accumulatedForce is less than the threshold in the current frame and the accumulatedForce is large than the threshold in the previous frame, 
					//and the user request notify for found event, we will raise eNOTIFY_THRESHOLD_FORCE_LOST
					if (si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST))
					{
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST, 0, false, ccdPass, false, outputs);
					}
				}
			}
		}
	}
}

void Sc::Scene::endStep()
{
	mTimeStamp++;
//  INVALID_SLEEP_COUNTER is 0xffffffff. Therefore the last bit is masked. Look at Body::isForcedToSleep() for example.
//	if(timeStamp==PX_INVALID_U32)	timeStamp = 0;	// Reserve INVALID_ID for something else
	mTimeStamp &= 0x7fffffff;

	mReportShapePairTimeStamp++;  // to make sure that deleted shapes/actors after fetchResults() create new report pairs
}

void Sc::Scene::resizeReleasedBodyIDMaps(PxU32 maxActors, PxU32 numActors)
{ 
	mLostTouchPairsDeletedBodyIDs.resize(maxActors);
	mActorIDTracker->resizeDeletedIDMap(maxActors,numActors); 
	mElementIDPool->resizeDeletedIDMap(maxActors,numActors);
}

/**
Render objects before simulation starts
*/
void Sc::Scene::visualizeStartStep()
{
	PX_PROFILE_ZONE("Sim.visualizeStartStep", getContextId());

#if PX_ENABLE_DEBUG_VISUALIZATION
	if(!getVisualizationScale())
	{
		// make sure visualization inside simulate was skipped
		PX_ASSERT(getRenderBuffer().empty()); 
		return; // early out if visualization scale is 0
	}

	PxRenderOutput out(getRenderBuffer());

	if(getVisualizationParameter(PxVisualizationParameter::eCOLLISION_COMPOUNDS))
		mAABBManager->visualize(out);

	// Visualize joints
	Sc::ConstraintCore*const * constraints = mConstraints.getEntries();
	for(PxU32 i=0, size = mConstraints.size();i<size; i++)
		constraints[i]->getSim()->visualize(getRenderBuffer());

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	mNPhaseCore->visualize(out, outputs);
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
}

//CCD
void Sc::Scene::collectPostSolverVelocitiesBeforeCCD()
{
	if (mContactReportsNeedPostSolverVelocity)
	{
		ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
		PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
		for(PxU32 i=0; i < nbActorPairs; i++)
		{
			if (i < (nbActorPairs - 1))
				PxPrefetchLine(actorPairs[i+1]);

			ActorPairReport* aPair = actorPairs[i];

			ContactStreamManager& cs = aPair->getContactStreamManager();

			PxU32 streamManagerFlag = cs.getFlags();
			if(streamManagerFlag & ContactStreamManagerFlag::eINVALID_STREAM)
				continue;

			PxU8* stream = mNPhaseCore->getContactReportPairData(cs.bufferIndex);
			
			if(i + 1 < nbActorPairs)
				PxPrefetch(&(actorPairs[i+1]->getContactStreamManager()));

			if (!cs.extraDataSize)
				continue;
			else if (streamManagerFlag & ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY)
				cs.setContactReportPostSolverVelocity(stream, aPair->getActorA(), aPair->getActorB());
		}
	}
}

void Sc::Scene::finalizeContactStreamAndCreateHeader(PxContactPairHeader& header, const ActorPairReport& aPair, ContactStreamManager& cs, PxU32 removedShapeTestMask)
{
	PxU8* stream = mNPhaseCore->getContactReportPairData(cs.bufferIndex);
	PxU32 streamManagerFlag = cs.getFlags();
	ContactShapePair* contactPairs = cs.getShapePairs(stream);
	const PxU16 nbShapePairs = cs.currentPairCount;
	PX_ASSERT(nbShapePairs > 0);

	if (streamManagerFlag & removedShapeTestMask)
	{
		// At least one shape of this actor pair has been deleted. Need to traverse the contact buffer,
		// find the pairs which contain deleted shapes and set the flags accordingly.

		ContactStreamManager::convertDeletedShapesInContactStream(contactPairs, nbShapePairs, getElementIDPool());
	}
	PX_ASSERT(contactPairs);

	ObjectIDTracker& ActorIDTracker = getActorIDTracker();
	header.actors[0] = aPair.getPxActorA();
	header.actors[1] = aPair.getPxActorB();
	PxU16 headerFlags = 0;
	if (ActorIDTracker.isDeletedID(aPair.getActorAID()))
		headerFlags |= PxContactPairHeaderFlag::eREMOVED_ACTOR_0;
	if (ActorIDTracker.isDeletedID(aPair.getActorBID()))
		headerFlags |= PxContactPairHeaderFlag::eREMOVED_ACTOR_1;
	header.flags = PxContactPairHeaderFlags(headerFlags);
	header.pairs = reinterpret_cast<PxContactPair*>(contactPairs);
	header.nbPairs = nbShapePairs;

	PxU16 extraDataSize = cs.extraDataSize;
	if (!extraDataSize)
		header.extraDataStream = NULL;
	else
	{
		PX_ASSERT(extraDataSize >= sizeof(ContactStreamHeader));
		extraDataSize -= sizeof(ContactStreamHeader);
		header.extraDataStream = stream + sizeof(ContactStreamHeader);

		if (streamManagerFlag & ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY)
		{
			PX_ASSERT(!(headerFlags & PxTo16(PxContactPairHeaderFlag::eREMOVED_ACTOR_0 | PxContactPairHeaderFlag::eREMOVED_ACTOR_1)));
			cs.setContactReportPostSolverVelocity(stream, aPair.getActorA(), aPair.getActorB());
		}
	}
	header.extraDataStreamSize = extraDataSize;
}

const PxArray<PxContactPairHeader>& Sc::Scene::getQueuedContactPairHeaders()
{
	const PxU32 removedShapeTestMask = PxU32(ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES);

	ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
	PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
	mQueuedContactPairHeaders.reserve(nbActorPairs);
	mQueuedContactPairHeaders.clear();

	for (PxU32 i = 0; i < nbActorPairs; i++)
	{
		if (i < (nbActorPairs - 1))
			PxPrefetchLine(actorPairs[i + 1]);

		ActorPairReport* aPair = actorPairs[i];
		ContactStreamManager& cs = aPair->getContactStreamManager();
		if (cs.getFlags() & ContactStreamManagerFlag::eINVALID_STREAM)
			continue;

		if (i + 1 < nbActorPairs)
			PxPrefetch(&(actorPairs[i + 1]->getContactStreamManager()));

		PxContactPairHeader &pairHeader = mQueuedContactPairHeaders.insert();
		finalizeContactStreamAndCreateHeader(pairHeader, *aPair, cs, removedShapeTestMask);

		cs.maxPairCount = cs.currentPairCount;
		cs.setMaxExtraDataSize(cs.extraDataSize);
	}

	return mQueuedContactPairHeaders;
}

/*
Threading: called in the context of the user thread, but only after the physics thread has finished its run
*/
void Sc::Scene::fireQueuedContactCallbacks()
{
	if(mSimulationEventCallback)
	{
		const PxU32 removedShapeTestMask = PxU32(ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES);

		ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
		PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
		for(PxU32 i=0; i < nbActorPairs; i++)
		{
			if (i < (nbActorPairs - 1))
				PxPrefetchLine(actorPairs[i+1]);

			ActorPairReport* aPair = actorPairs[i];
			ContactStreamManager* cs = &aPair->getContactStreamManager();
			if (cs == NULL || cs->getFlags() & ContactStreamManagerFlag::eINVALID_STREAM)
				continue;
			
			if (i + 1 < nbActorPairs)
				PxPrefetch(&(actorPairs[i+1]->getContactStreamManager()));

			PxContactPairHeader pairHeader;
			finalizeContactStreamAndCreateHeader(pairHeader, *aPair, *cs, removedShapeTestMask);

			mSimulationEventCallback->onContact(pairHeader, pairHeader.pairs, pairHeader.nbPairs);

			// estimates for next frame
			cs->maxPairCount = cs->currentPairCount;
			cs->setMaxExtraDataSize(cs->extraDataSize);
		}
	}
}

PX_FORCE_INLINE void markDeletedShapes(Sc::ObjectIDTracker& idTracker, Sc::TriggerPairExtraData& tped, PxTriggerPair& pair)
{
	PxTriggerPairFlags::InternalType flags = 0;
	if (idTracker.isDeletedID(tped.shape0ID))
		flags |= PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER;
	if (idTracker.isDeletedID(tped.shape1ID))
		flags |= PxTriggerPairFlag::eREMOVED_SHAPE_OTHER;

	pair.flags = PxTriggerPairFlags(flags);
}

void Sc::Scene::fireTriggerCallbacks()
{
	// triggers
	const PxU32 nbTriggerPairs = mTriggerBufferAPI.size();
	PX_ASSERT(nbTriggerPairs == mTriggerBufferExtraData->size());
	if(nbTriggerPairs) 
	{
		// cases to take into account:
		// - no simulation/trigger shape has been removed -> no need to test shape references for removed shapes
		// - simulation/trigger shapes have been removed  -> test the events that have 
		//   a marker for removed shapes set
		//
		const bool hasRemovedShapes = mElementIDPool->getDeletedIDCount() > 0;

		if(mSimulationEventCallback)
		{
			if (!hasRemovedShapes)
				mSimulationEventCallback->onTrigger(mTriggerBufferAPI.begin(), nbTriggerPairs);
			else
			{
				for(PxU32 i = 0; i < nbTriggerPairs; i++)
				{
					PxTriggerPair& triggerPair = mTriggerBufferAPI[i];

					if ((PxTriggerPairFlags::InternalType(triggerPair.flags) & TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES))
						markDeletedShapes(*mElementIDPool, (*mTriggerBufferExtraData)[i], triggerPair);
				}

				mSimulationEventCallback->onTrigger(mTriggerBufferAPI.begin(), nbTriggerPairs);
			}
		}
	}

	// PT: clear the buffer **even when there's no simulationEventCallback**.
	mTriggerBufferAPI.clear();
	mTriggerBufferExtraData->clear();
}

void Sc::Scene::fireBrokenConstraintCallbacks()
{
	if(!mSimulationEventCallback)
		return;

	const PxU32 count = mBrokenConstraints.size();
	for(PxU32 i=0;i<count;i++)
	{
		Sc::ConstraintCore* c = mBrokenConstraints[i];
		PX_ASSERT(c->getSim());

		PxU32 typeID = 0xffffffff;
		void* externalRef = c->getPxConnector()->getExternalReference(typeID);
		PX_CHECK_MSG(typeID != 0xffffffff, "onConstraintBreak: Invalid constraint type ID.");

		PxConstraintInfo constraintInfo(c->getPxConstraint(), externalRef, typeID);
		mSimulationEventCallback->onConstraintBreak(&constraintInfo, 1);
	}
}

/*
Threading: called in the context of the user thread, but only after the physics thread has finished its run
*/
void Sc::Scene::fireCallbacksPostSync()
{
	//
	// Fire sleep & woken callbacks
	//

	// A body should be either in the sleep or the woken list. If it is in both, remove it from the list it was
	// least recently added to.

	if(!mSleepBodyListValid)
		cleanUpSleepBodies();

	if(!mWokeBodyListValid)
		cleanUpWokenBodies();

#if PX_SUPPORT_GPU_PHYSX
	if (!mSleepSoftBodyListValid)
		cleanUpSleepSoftBodies();

	if (!mWokeBodyListValid)
		cleanUpWokenSoftBodies();

	if (!mSleepHairSystemListValid)
		cleanUpSleepHairSystems();

	if (!mWokeHairSystemListValid) // TODO(jcarius) should this be mWokeBodyListValid?
		cleanUpWokenHairSystems();
#endif

	if(mSimulationEventCallback || mOnSleepingStateChanged)
	{
		// allocate temporary data
		const PxU32 nbSleep = mSleepBodies.size();
		const PxU32 nbWoken = mWokeBodies.size();
#if PX_SUPPORT_GPU_PHYSX
		const PxU32 nbHairSystemSleep = mSleepHairSystems.size();
		const PxU32 nbHairSystemWoken = mWokeHairSystems.size();
		const PxU32 nbSoftBodySleep = mSleepSoftBodies.size();
		const PxU32 nbSoftBodyWoken = mWokeSoftBodies.size();
		const PxU32 arrSize = PxMax(PxMax(nbSleep, nbWoken), PxMax(nbSoftBodySleep, nbHairSystemSleep));
#else
		const PxU32 arrSize = PxMax(nbSleep, nbWoken);
#endif
		PxActor** actors = arrSize ? reinterpret_cast<PxActor**>(PX_ALLOC(arrSize*sizeof(PxActor*), "PxActor*")) : NULL;
		if(actors)
		{
			if(nbSleep)
			{
				PxU32 destSlot = 0;
				BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
				for(PxU32 i=0; i<nbSleep; i++)
				{
					BodyCore* body = sleepingBodies[i];
					if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(body->getPxActor(), true);
				}

				if(destSlot && mSimulationEventCallback)
					mSimulationEventCallback->onSleep(actors, destSlot);

				//if (PX_DBG_IS_CONNECTED())
				//{
				//	for (PxU32 i = 0; i < nbSleep; ++i)
				//	{
				//		BodyCore* body = mSleepBodies[i];
				//		PX_ASSERT(body->getActorType() == PxActorType::eRIGID_DYNAMIC);
				//	}
				//}
			}

#if PX_SUPPORT_GPU_PHYSX
			//ML: need to create and API for the onSleep for softbody
			if (nbSoftBodySleep)
			{
				PxU32 destSlot = 0;
				SoftBodyCore* const* sleepingSoftBodies = mSleepSoftBodies.getEntries();
				for (PxU32 i = 0; i<nbSoftBodySleep; i++)
				{
					SoftBodyCore* body = sleepingSoftBodies[i];
					if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(body->getPxActor(), true);
				}

				if (destSlot && mSimulationEventCallback)
					mSimulationEventCallback->onSleep(actors, destSlot);
			}

			if (nbHairSystemSleep)
			{
				PxU32 destSlot = 0;
				HairSystemCore* const* sleepingHairSystems = mSleepHairSystems.getEntries();
				for (PxU32 i = 0; i<nbHairSystemSleep; i++)
				{
					HairSystemCore* body = sleepingHairSystems[i];
					if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(body->getPxActor(), true);
				}

				if (destSlot && mSimulationEventCallback)
					mSimulationEventCallback->onSleep(actors, destSlot);
			}
#endif

			// do the same thing for bodies that have just woken up

			if(nbWoken)
			{
				PxU32 destSlot = 0;
				BodyCore* const* wokenBodies = mWokeBodies.getEntries();
				for(PxU32 i=0; i<nbWoken; i++)
				{
					BodyCore* body = wokenBodies[i];
					if(body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(body->getPxActor(), false);
				}

				if(destSlot && mSimulationEventCallback)
					mSimulationEventCallback->onWake(actors, destSlot);
			}

#if PX_SUPPORT_GPU_PHYSX
			//ML: need to create an API for woken soft body
			if (nbSoftBodyWoken)
			{
				PxU32 destSlot = 0;
				SoftBodyCore* const* wokenSoftBodies = mWokeSoftBodies.getEntries();
				for (PxU32 i = 0; i<nbSoftBodyWoken; i++)
				{
					SoftBodyCore* body = wokenSoftBodies[i];
					if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(body->getPxActor(), false);
				}

				if (destSlot && mSimulationEventCallback)
					mSimulationEventCallback->onWake(actors, destSlot);
			}

			if (nbHairSystemWoken)
			{
				PxU32 destSlot = 0;
				HairSystemCore* const* wokenHairSystems = mWokeHairSystems.getEntries();
				for (PxU32 i = 0; i<nbHairSystemWoken; i++)
				{
					HairSystemCore* body = wokenHairSystems[i];
					if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(body->getPxActor(), false);
				}

				if (destSlot && mSimulationEventCallback)
					mSimulationEventCallback->onWake(actors, destSlot);
			}
#endif
			PX_FREE(actors);
		}
	}

	clearSleepWakeBodies();
}

void Sc::Scene::prepareOutOfBoundsCallbacks()
{
	PxU32 nbOut0;
	void** outObjects = mAABBManager->getOutOfBoundsObjects(nbOut0);

	mOutOfBoundsIDs.clear();
	for(PxU32 i=0;i<nbOut0;i++)
	{
		ElementSim* volume = reinterpret_cast<ElementSim*>(outObjects[i]);

		Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(volume);
		PxU32 id = sim->getElementID();
		mOutOfBoundsIDs.pushBack(id);
	}
}

bool Sc::Scene::fireOutOfBoundsCallbacks()
{
	bool outputWarning = false;

	// Actors
	{
		PxU32 nbOut0;
		void** outObjects = mAABBManager->getOutOfBoundsObjects(nbOut0);

		const ObjectIDTracker& tracker = getElementIDPool();

		PxBroadPhaseCallback* cb = mBroadPhaseCallback;
		for(PxU32 i=0;i<nbOut0;i++)
		{
			ElementSim* volume = reinterpret_cast<ElementSim*>(outObjects[i]);

			Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(volume);
			if(tracker.isDeletedID(mOutOfBoundsIDs[i]))
				continue;

			if(cb)
			{
				ActorSim& actor = volume->getActor();
				RigidSim& rigidSim = static_cast<RigidSim&>(actor);
				PxActor* pxActor = rigidSim.getPxActor();
				PxShape* px = sim->getPxShape();
				cb->onObjectOutOfBounds(*px, *pxActor);
			}
			else
			{
				outputWarning = true;
			}
		}
		mAABBManager->clearOutOfBoundsObjects();
	}

	return outputWarning;
}

void Sc::Scene::fireOnAdvanceCallback()
{
	if(!mSimulationEventCallback)
		return;

	const PxU32 nbPosePreviews = mPosePreviewBodies.size();
	if(!nbPosePreviews)
		return;

	mClientPosePreviewBodies.clear();
	mClientPosePreviewBodies.reserve(nbPosePreviews);

	mClientPosePreviewBuffer.clear();
	mClientPosePreviewBuffer.reserve(nbPosePreviews);
		
	const BodySim*const* PX_RESTRICT posePreviewBodies = mPosePreviewBodies.getEntries();
	for(PxU32 i=0; i<nbPosePreviews; i++)
	{
		const BodySim& b = *posePreviewBodies[i];
		if(!b.isFrozen())
		{
			PxsBodyCore& c = b.getBodyCore().getCore();
			mClientPosePreviewBodies.pushBack(static_cast<const PxRigidBody*>(b.getPxActor()));
			mClientPosePreviewBuffer.pushBack(c.body2World * c.getBody2Actor().getInverse());
		}
	}

	const PxU32 bodyCount = mClientPosePreviewBodies.size();
	if(bodyCount)
		mSimulationEventCallback->onAdvance(mClientPosePreviewBodies.begin(), mClientPosePreviewBuffer.begin(), bodyCount);
}

void Sc::Scene::postCallbacksPreSync()
{
	PX_PROFILE_ZONE("Sim.postCallbackPreSync", mContextId);
	// clear contact stream data
	mNPhaseCore->clearContactReportStream();
	mNPhaseCore->clearContactReportActorPairs(false);

	// Put/prepare kinematics to/for sleep and invalidate target pose
	// note: this needs to get done after the contact callbacks because
	//       the target might get read there.
	//
	PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	//KS - this method must run over the kinematic actors in reverse.
	while(nbKinematics--)
	{
		if(nbKinematics > 16)
		{
			PxPrefetchLine(static_cast<BodyCore*>(kinematics[nbKinematics-16]));
		}
		if (nbKinematics > 4)
		{
			PxPrefetchLine((static_cast<BodyCore*>(kinematics[nbKinematics - 4]))->getSim());
			PxPrefetchLine((static_cast<BodyCore*>(kinematics[nbKinematics - 4]))->getSim()->getSimStateData_Unchecked());
		}

		BodyCore* b = static_cast<BodyCore*>(kinematics[nbKinematics]);
		//kinematics++;
		PX_ASSERT(b->getSim()->isKinematic());
		PX_ASSERT(b->getSim()->isActive());

		b->invalidateKinematicTarget();
		b->getSim()->deactivateKinematic();
	}

	releaseConstraints(true); //release constraint blocks at the end of the frame, so user can retrieve the blocks
}

void Sc::Scene::setNbContactDataBlocks(PxU32 numBlocks)
{
	mLLContext->getNpMemBlockPool().setBlockCount(numBlocks);
}

PxU32 Sc::Scene::getNbContactDataBlocksUsed() const
{
	return mLLContext->getNpMemBlockPool().getUsedBlockCount();
}

PxU32 Sc::Scene::getMaxNbContactDataBlocksUsed() const
{
	return mLLContext->getNpMemBlockPool().getMaxUsedBlockCount();
}

PxU32 Sc::Scene::getMaxNbConstraintDataBlocksUsed() const
{
	return mLLContext->getNpMemBlockPool().getPeakConstraintBlockCount();
}

void Sc::Scene::setScratchBlock(void* addr, PxU32 size)
{
	return mLLContext->setScratchBlock(addr, size);
}

void Sc::Scene::checkConstraintBreakage()
{
	PX_PROFILE_ZONE("Sim.checkConstraintBreakage", getContextId());

	PxU32 count = mActiveBreakableConstraints.size();
	ConstraintSim* const* constraints = mActiveBreakableConstraints.getEntries(); 
	while(count)
	{
		count--;
		constraints[count]->checkMaxForceExceeded();  // start from the back because broken constraints get removed from the list
	}
}

void Sc::Scene::getStats(PxSimulationStatistics& s) const
{
	mStats->readOut(s, mLLContext->getSimStats());
	s.nbStaticBodies = mNbRigidStatics;
	s.nbDynamicBodies = mNbRigidDynamics;
	s.nbKinematicBodies = mNbRigidKinematic;
	s.nbArticulations = mArticulations.size(); 

	s.nbAggregates = mAABBManager->getNbActiveAggregates();
	for(PxU32 i=0; i<PxGeometryType::eGEOMETRY_COUNT; i++)
		s.nbShapes[i] = mNbGeometries[i];

#if PX_SUPPORT_GPU_PHYSX
	if (mHeapMemoryAllocationManager)
	{
		s.gpuMemHeap = mHeapMemoryAllocationManager->getDeviceMemorySize();

		const PxsHeapStats& deviceHeapStats = mHeapMemoryAllocationManager->getDeviceHeapStats();
		s.gpuMemHeapBroadPhase = deviceHeapStats.stats[PxsHeapStats::eBROADPHASE];
		s.gpuMemHeapNarrowPhase = deviceHeapStats.stats[PxsHeapStats::eNARROWPHASE];
		s.gpuMemHeapSolver = deviceHeapStats.stats[PxsHeapStats::eSOLVER];
		s.gpuMemHeapArticulation = deviceHeapStats.stats[PxsHeapStats::eARTICULATION];
		s.gpuMemHeapSimulation = deviceHeapStats.stats[PxsHeapStats::eSIMULATION];
		s.gpuMemHeapSimulationArticulation = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_ARTICULATION];
		s.gpuMemHeapSimulationParticles = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_PARTICLES];
		s.gpuMemHeapSimulationSoftBody = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_SOFTBODY];
		s.gpuMemHeapSimulationFEMCloth = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_FEMCLOTH];
		s.gpuMemHeapSimulationHairSystem = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_HAIRSYSTEM];
		s.gpuMemHeapParticles = deviceHeapStats.stats[PxsHeapStats::eSHARED_PARTICLES];
		s.gpuMemHeapFEMCloths = deviceHeapStats.stats[PxsHeapStats::eSHARED_FEMCLOTH];
		s.gpuMemHeapSoftBodies = deviceHeapStats.stats[PxsHeapStats::eSHARED_SOFTBODY];
		s.gpuMemHeapHairSystems = deviceHeapStats.stats[PxsHeapStats::eSHARED_HAIRSYSTEM];
		s.gpuMemHeapOther = deviceHeapStats.stats[PxsHeapStats::eOTHER];
	}
	else
#endif
	{
		s.gpuMemHeap = 0;
		s.gpuMemParticles = 0;
		s.gpuMemSoftBodies = 0;
		s.gpuMemFEMCloths = 0;
		s.gpuMemHairSystems = 0;
		s.gpuMemHeapBroadPhase = 0;
		s.gpuMemHeapNarrowPhase = 0;
		s.gpuMemHeapSolver = 0;
		s.gpuMemHeapArticulation = 0;
		s.gpuMemHeapSimulation = 0;
		s.gpuMemHeapSimulationArticulation = 0;
		s.gpuMemHeapSimulationParticles = 0;
		s.gpuMemHeapSimulationSoftBody = 0;
		s.gpuMemHeapSimulationFEMCloth = 0;
		s.gpuMemHeapSimulationHairSystem = 0;
		s.gpuMemHeapParticles = 0;
		s.gpuMemHeapSoftBodies = 0;
		s.gpuMemHeapFEMCloths = 0;
		s.gpuMemHeapHairSystems = 0;
		s.gpuMemHeapOther = 0;
	}
}

void Sc::Scene::addShapes(NpShape *const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& bodySim, PxBounds3* outBounds)
{
	const PxNodeIndex nodeIndex = bodySim.getNodeIndex();
	
	PxvNphaseImplementationContext* context = mLLContext->getNphaseImplementationContext();

	for(PxU32 i=0;i<nbShapes;i++)
	{
		// PT: TODO: drop the offsets and let me include NpShape.h from here! This is just NpShape::getCore()....
		ShapeCore& sc = *reinterpret_cast<ShapeCore*>(size_t(shapes[i])+ptrOffset);

		//PxBounds3* target = uninflatedBounds ? uninflatedBounds + i : uninflatedBounds;
		//mShapeSimPool->construct(sim, sc, llBody, target);

		ShapeSim* shapeSim = mShapeSimPool->construct(bodySim, sc);
		mNbGeometries[sc.getGeometryType()]++;

		mSimulationController->addShape(&shapeSim->getLLShapeSim(), shapeSim->getElementID());

		if (outBounds)
			outBounds[i] = mBoundsArray->getBounds(shapeSim->getElementID());
		
		//I register the shape if its either not an articulation link or if the nodeIndex has already been
		//assigned. On insertion, the articulation will not have this nodeIndex correctly assigned at this stage
		if (bodySim.getActorType() != PxActorType::eARTICULATION_LINK || !nodeIndex.isStaticBody())
			context->registerShape(nodeIndex, sc.getCore(), shapeSim->getElementID(), bodySim.getPxActor());
	}
}

void Sc::Scene::removeShapes(Sc::RigidSim& sim, PxInlineArray<Sc::ShapeSim*, 64>& shapesBuffer , PxInlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	PxU32 nbElems = sim.getNbElements();
	Sc::ElementSim** elems = sim.getElements();
	while (nbElems--)
	{
		Sc::ShapeSim* s = static_cast<Sc::ShapeSim*>(*elems++);
		// can do two 2x the allocs in the worst case, but actors with >64 shapes are not common
		shapesBuffer.pushBack(s);
		removedShapes.pushBack(&s->getCore());
	}

	for(PxU32 i=0;i<shapesBuffer.size();i++)
		removeShape_(*shapesBuffer[i], wakeOnLostTouch);
}

void Sc::Scene::addStatic(StaticCore& ro, NpShape*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds)
{
	PX_ASSERT(ro.getActorCoreType() == PxActorType::eRIGID_STATIC);

	// sim objects do all the necessary work of adding themselves to broad phase,
	// activation, registering with the interaction system, etc

	StaticSim* sim = mStaticSimPool->construct(*this, ro);
	
	mNbRigidStatics++;
	addShapes(shapes, nbShapes, shapePtrOffset, *sim, uninflatedBounds);
}

void Sc::Scene::removeStatic(StaticCore& ro, PxInlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	PX_ASSERT(ro.getActorCoreType() == PxActorType::eRIGID_STATIC);

	StaticSim* sim = ro.getSim();
	if(sim)
	{
		if(mBatchRemoveState)
		{
			removeShapes(*sim, mBatchRemoveState->bufferedShapes ,removedShapes, wakeOnLostTouch);
		}
		else
		{
			PxInlineArray<Sc::ShapeSim*, 64>  shapesBuffer;
			removeShapes(*sim, shapesBuffer ,removedShapes, wakeOnLostTouch);
		}		
		mStaticSimPool->destroy(static_cast<Sc::StaticSim*>(ro.getSim()));
		mNbRigidStatics--;
	}
}

void Sc::Scene::setDynamicsDirty()
{
	mDynamicsContext->setStateDirty(true);
}

void Sc::Scene::addBody(BodyCore& body, NpShape*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* outBounds, bool compound)
{
	// sim objects do all the necessary work of adding themselves to broad phase,
	// activation, registering with the interaction system, etc

	BodySim* sim = mBodySimPool->construct(*this, body, compound);

	const bool isArticulationLink = sim->isArticulationLink();

	if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD && sim->isActive())
	{
		if (isArticulationLink)
		{
			if (sim->getNodeIndex().isValid())
				mSpeculativeCDDArticulationBitMap.growAndSet(sim->getNodeIndex().index());
		}
		else
			mSpeculativeCCDRigidBodyBitMap.growAndSet(sim->getNodeIndex().index());
	}
	//if rigid body is articulation link, the node index will be invalid. We should add the link to the scene after we add the
	//articulation for gpu
	if(sim->getNodeIndex().isValid())
		mSimulationController->addDynamic(&sim->getLowLevelBody(), sim->getNodeIndex());
	if(sim->getSimStateData(true) && sim->getSimStateData(true)->isKine())
		mNbRigidKinematic++;
	else
		mNbRigidDynamics++;
	addShapes(shapes, nbShapes, shapePtrOffset, *sim, outBounds);

	mDynamicsContext->setStateDirty(true);
}

void Sc::Scene::removeBody(BodyCore& body, PxInlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	BodySim *sim = body.getSim();	
	if(sim)
	{
		if(mBatchRemoveState)
		{
			removeShapes(*sim, mBatchRemoveState->bufferedShapes ,removedShapes, wakeOnLostTouch);
		}
		else
		{
			PxInlineArray<Sc::ShapeSim*, 64>  shapesBuffer;
			removeShapes(*sim,shapesBuffer, removedShapes, wakeOnLostTouch);
		}

		if (!sim->isArticulationLink())
		{
			//clear bit map
			if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
				sim->getScene().resetSpeculativeCCDRigidBody(sim->getNodeIndex().index());				
		}		
		else
		{
			sim->getArticulation()->removeBody(*sim);
		}
		if(sim->getSimStateData(true) && sim->getSimStateData(true)->isKine())
		{
			body.onRemoveKinematicFromScene();

			mNbRigidKinematic--;
		}
		else
			mNbRigidDynamics--;
		mBodySimPool->destroy(sim);

		mDynamicsContext->setStateDirty(true);
	}
}

// PT: TODO: refactor with addShapes
void Sc::Scene::addShape_(RigidSim& owner, ShapeCore& shapeCore)
{
	ShapeSim* sim = mShapeSimPool->construct(owner, shapeCore);
	mNbGeometries[shapeCore.getGeometryType()]++;

	//register shape
	mSimulationController->addShape(&sim->getLLShapeSim(), sim->getElementID());

	registerShapeInNphase(&owner.getRigidCore(), shapeCore, sim->getElementID());
}

// PT: TODO: refactor with removeShapes
void Sc::Scene::removeShape_(ShapeSim& shape, bool wakeOnLostTouch)
{
	//BodySim* body = shape.getBodySim();
	//if(body)
	//	body->postShapeDetach();
	
	unregisterShapeFromNphase(shape.getCore(), shape.getElementID());

	mSimulationController->removeShape(shape.getElementID());

	mNbGeometries[shape.getCore().getGeometryType()]--;
	shape.removeFromBroadPhase(wakeOnLostTouch);
	mShapeSimPool->destroy(&shape);
}

void Sc::Scene::registerShapeInNphase(Sc::RigidCore* rigidCore, const ShapeCore& shape, const PxU32 transformCacheID)
{
	RigidSim* sim = rigidCore->getSim();
	if(sim)
		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), shape.getCore(), transformCacheID, sim->getPxActor());
}

void Sc::Scene::unregisterShapeFromNphase(const ShapeCore& shape, const PxU32 transformCacheID)
{
	mLLContext->getNphaseImplementationContext()->unregisterShape(shape.getCore(), transformCacheID);
}

void Sc::Scene::notifyNphaseOnUpdateShapeMaterial(const ShapeCore& shapeCore)
{
	mLLContext->getNphaseImplementationContext()->updateShapeMaterial(shapeCore.getCore());
}

void Sc::Scene::startBatchInsertion(BatchInsertionState&state)
{
	state.shapeSim = mShapeSimPool->allocateAndPrefetch();
	state.staticSim = mStaticSimPool->allocateAndPrefetch();
	state.bodySim = mBodySimPool->allocateAndPrefetch();														   
}

void Sc::Scene::addShapes(NpShape*const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& rigidSim, ShapeSim*& prefetchedShapeSim, PxBounds3* outBounds)
{
	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(i+1<nbShapes)
			PxPrefetch(shapes[i+1], PxU32(ptrOffset+sizeof(Sc::ShapeCore)));
		ShapeSim* nextShapeSim = mShapeSimPool->allocateAndPrefetch();
		ShapeCore& sc = *PxPointerOffset<ShapeCore*>(shapes[i], ptrdiff_t(ptrOffset));
		PX_PLACEMENT_NEW(prefetchedShapeSim, ShapeSim(rigidSim, sc));
		const PxU32 elementID = prefetchedShapeSim->getElementID();

		outBounds[i] = mBoundsArray->getBounds(elementID);

		mSimulationController->addShape(&prefetchedShapeSim->getLLShapeSim(), elementID);
		mLLContext->getNphaseImplementationContext()->registerShape(rigidSim.getNodeIndex(), sc.getCore(), elementID, rigidSim.getPxActor());

		prefetchedShapeSim = nextShapeSim;
		mNbGeometries[sc.getGeometryType()]++;
	}	
}

void Sc::Scene::addStatic(PxActor* actor, BatchInsertionState& s, PxBounds3* outBounds)
{
	// static core has been prefetched by caller
	Sc::StaticSim* sim = s.staticSim;		// static core has been prefetched by the caller

	const Cm::PtrTable* shapeTable = PxPointerOffset<const Cm::PtrTable*>(actor, s.staticShapeTableOffset);
	void*const* shapes = shapeTable->getPtrs();

	mStaticSimPool->construct(sim, *this, *PxPointerOffset<Sc::StaticCore*>(actor, s.staticActorOffset));
	s.staticSim = mStaticSimPool->allocateAndPrefetch();

	addShapes(reinterpret_cast<NpShape*const*>(shapes), shapeTable->getCount(), size_t(s.shapeOffset), *sim, s.shapeSim, outBounds);
	mNbRigidStatics++;
}

void Sc::Scene::addBody(PxActor* actor, BatchInsertionState& s, PxBounds3* outBounds, bool compound)
{
	Sc::BodySim* sim = s.bodySim;		// body core has been prefetched by the caller

	const Cm::PtrTable* shapeTable = PxPointerOffset<const Cm::PtrTable*>(actor, s.dynamicShapeTableOffset);
	void*const* shapes = shapeTable->getPtrs();

	Sc::BodyCore* bodyCore = PxPointerOffset<Sc::BodyCore*>(actor, s.dynamicActorOffset);
	mBodySimPool->construct(sim, *this, *bodyCore, compound);	
	s.bodySim = mBodySimPool->allocateAndPrefetch();

	if(sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
	{
		if(sim->isArticulationLink())
			mSpeculativeCDDArticulationBitMap.growAndSet(sim->getNodeIndex().index());
		else
			mSpeculativeCCDRigidBodyBitMap.growAndSet(sim->getNodeIndex().index());
	}

	if(sim->getNodeIndex().isValid())
		mSimulationController->addDynamic(&sim->getLowLevelBody(), sim->getNodeIndex());

	addShapes(reinterpret_cast<NpShape*const*>(shapes), shapeTable->getCount(), size_t(s.shapeOffset), *sim, s.shapeSim, outBounds);

	const SimStateData* simStateData = bodyCore->getSim()->getSimStateData(true);
	if(simStateData && simStateData->isKine())
		mNbRigidKinematic++;
	else
		mNbRigidDynamics++;

	mDynamicsContext->setStateDirty(true);
}

void Sc::Scene::finishBatchInsertion(BatchInsertionState& state)
{
	// a little bit lazy - we could deal with the last one in the batch specially to avoid overallocating by one.
	
	mStaticSimPool->releasePreallocated(static_cast<Sc::StaticSim*>(state.staticSim));	
	mBodySimPool->releasePreallocated(static_cast<Sc::BodySim*>(state.bodySim));
	mShapeSimPool->releasePreallocated(state.shapeSim);
}

// PT: TODO: why is this in Sc?
void Sc::Scene::initContactsIterator(ContactIterator& contactIterator, PxsContactManagerOutputIterator& outputs)
{
	outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	Interaction** first = mInteractions[Sc::InteractionType::eOVERLAP].begin();
	contactIterator = ContactIterator(first, first + mActiveInteractionCount[Sc::InteractionType::eOVERLAP], outputs);
}

void Sc::Scene::setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)
{
	struct {
		void operator()(PxU32& bits, PxDominanceGroup shift, PxReal weight)
		{
			if(weight != PxReal(0))
				bits |=  (PxU32(1) << shift);
			else 
				bits &= ~(PxU32(1) << shift);
		}
	} bitsetter;

	bitsetter(mDominanceBitMatrix[group1], group2, dominance.dominance0);
	bitsetter(mDominanceBitMatrix[group2], group1, dominance.dominance1);

	mInternalFlags |= SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE;		//force an update on all interactions on matrix change -- very expensive but we have no choice!!
}

PxDominanceGroupPair Sc::Scene::getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const
{
	const PxU8 dom0 = PxU8((mDominanceBitMatrix[group1]>>group2) & 0x1 ? 1u : 0u);
	const PxU8 dom1 = PxU8((mDominanceBitMatrix[group2]>>group1) & 0x1 ? 1u : 0u);
	return PxDominanceGroupPair(dom0, dom1);
}

void Sc::Scene::setSolverBatchSize(PxU32 solverBatchSize)
{
	mDynamicsContext->setSolverBatchSize(solverBatchSize);
}

PxU32 Sc::Scene::getSolverBatchSize() const
{
	return mDynamicsContext->getSolverBatchSize();
}

void Sc::Scene::setSolverArticBatchSize(PxU32 solverBatchSize)
{
	mDynamicsContext->setSolverArticBatchSize(solverBatchSize);
}

PxU32 Sc::Scene::getSolverArticBatchSize() const
{
	return mDynamicsContext->getSolverArticBatchSize();
}

void Sc::Scene::setCCDMaxSeparation(PxReal separation)
{
	mDynamicsContext->setCCDSeparationThreshold(separation);
}

PxReal Sc::Scene::getCCDMaxSeparation() const
{
	return mDynamicsContext->getCCDSeparationThreshold();
}

void Sc::Scene::setMaxBiasCoefficient(PxReal coeff)
{
	mDynamicsContext->setMaxBiasCoefficient(coeff);
}

PxReal Sc::Scene::getMaxBiasCoefficient() const
{
	return mDynamicsContext->getMaxBiasCoefficient();
}

void Sc::Scene::setFrictionOffsetThreshold(PxReal t)
{
	mDynamicsContext->setFrictionOffsetThreshold(t);
}

PxReal Sc::Scene::getFrictionOffsetThreshold() const
{
	return mDynamicsContext->getFrictionOffsetThreshold();
}

void Sc::Scene::setFrictionCorrelationDistance(PxReal t)
{
	mDynamicsContext->setCorrelationDistance(t);
}

PxReal Sc::Scene::getFrictionCorrelationDistance() const
{
	return mDynamicsContext->getCorrelationDistance();
}

PxU32 Sc::Scene::getDefaultContactReportStreamBufferSize() const
{
	return mNPhaseCore->getDefaultContactReportStreamBufferSize();
}

void Sc::Scene::buildActiveActors()
{
	{
		PxU32 numActiveBodies = 0;
		BodyCore*const* PX_RESTRICT activeBodies;
		if (!(getPublicFlags() & PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS))
		{
			numActiveBodies = getNumActiveBodies();
			activeBodies = getActiveBodiesArray();
		}
		else
		{
			numActiveBodies = getActiveDynamicBodiesCount();
			activeBodies = getActiveDynamicBodies();
		}

		mActiveActors.clear();

		for (PxU32 i = 0; i < numActiveBodies; i++)
		{
			if (!activeBodies[i]->isFrozen())
			{
				PxRigidActor* ra = static_cast<PxRigidActor*>(activeBodies[i]->getPxActor());
				PX_ASSERT(ra);
				mActiveActors.pushBack(ra);
			}
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	{
		PxU32 numActiveSoftBodies = getNumActiveSoftBodies();
		SoftBodyCore*const* PX_RESTRICT activeSoftBodies = getActiveSoftBodiesArray();

		mActiveSoftBodyActors.clear();

		for (PxU32 i = 0; i < numActiveSoftBodies; i++)
		{
			PxActor* ra = activeSoftBodies[i]->getPxActor();
			mActiveSoftBodyActors.pushBack(ra);
		}
	}
	{
		PxU32 numActiveHairSystems = getNumActiveHairSystems();
		HairSystemCore*const* PX_RESTRICT activeHairSystems = getActiveHairSystemsArray();

		mActiveHairSystemActors.clear();

		for (PxU32 i = 0; i < numActiveHairSystems; i++)
		{
			PxActor* ra = activeHairSystems[i]->getPxActor();
			mActiveHairSystemActors.pushBack(ra);
		}
	}

#endif
}

// PT: TODO: unify buildActiveActors & buildActiveAndFrozenActors
void Sc::Scene::buildActiveAndFrozenActors()
{
	{
		PxU32 numActiveBodies = 0;
		BodyCore*const* PX_RESTRICT activeBodies;
		if (!(getPublicFlags() & PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS))
		{
			numActiveBodies = getNumActiveBodies();
			activeBodies = getActiveBodiesArray();
		}
		else
		{
			numActiveBodies = getActiveDynamicBodiesCount();
			activeBodies = getActiveDynamicBodies();
		}

		mActiveActors.clear();
		mFrozenActors.clear();

		for (PxU32 i = 0; i < numActiveBodies; i++)
		{
			PxRigidActor* ra = static_cast<PxRigidActor*>(activeBodies[i]->getPxActor());
			PX_ASSERT(ra);

			if (!activeBodies[i]->isFrozen())
				mActiveActors.pushBack(ra);
			else
				mFrozenActors.pushBack(ra);
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	{
		PxU32 numActiveSoftBodies = getNumActiveSoftBodies();
		SoftBodyCore*const* PX_RESTRICT activeSoftBodies = getActiveSoftBodiesArray();
		
		mActiveSoftBodyActors.clear();

		for (PxU32 i = 0; i < numActiveSoftBodies; i++)
		{
			PxActor* ra = activeSoftBodies[i]->getPxActor();
			mActiveActors.pushBack(ra);
		}
	}
	{
		PxU32 numActiveHairSystems = getNumActiveHairSystems();
		HairSystemCore*const* PX_RESTRICT activeHairSystems = getActiveHairSystemsArray();

		mActiveHairSystemActors.clear();

		for (PxU32 i = 0; i < numActiveHairSystems; i++)
		{
			PxActor* ra = activeHairSystems[i]->getPxActor();
			mActiveHairSystemActors.pushBack(ra);
		}
	}
#endif
}

PxActor** Sc::Scene::getActiveActors(PxU32& nbActorsOut)
{
	nbActorsOut = mActiveActors.size();
	
	if(!nbActorsOut)
		return NULL;

	return mActiveActors.begin();
}

void Sc::Scene::setActiveActors(PxActor** actors, PxU32 nbActors)
{
	mActiveActors.forceSize_Unsafe(0);
	mActiveActors.resize(nbActors);
	PxMemCopy(mActiveActors.begin(), actors, sizeof(PxActor*) * nbActors);
}

#if PX_SUPPORT_GPU_PHYSX
PxActor** Sc::Scene::getActiveSoftBodyActors(PxU32& nbActorsOut)
{
	nbActorsOut = mActiveSoftBodyActors.size();

	if (!nbActorsOut)
		return NULL;

	return mActiveSoftBodyActors.begin();
}

void Sc::Scene::setActiveSoftBodyActors(PxActor** actors, PxU32 nbActors)
{
	mActiveSoftBodyActors.forceSize_Unsafe(0);
	mActiveSoftBodyActors.resize(nbActors);
	PxMemCopy(mActiveSoftBodyActors.begin(), actors, sizeof(PxActor*) * nbActors);
}

//PxActor** Sc::Scene::getActiveFEMClothActors(PxU32& nbActorsOut)
//{
//	nbActorsOut = mActiveFEMClothActors.size();
//
//	if (!nbActorsOut)
//		return NULL;
//
//	return mActiveFEMClothActors.begin();
//}
//
//void Sc::Scene::setActiveFEMClothActors(PxActor** actors, PxU32 nbActors)
//{
//	mActiveFEMClothActors.forceSize_Unsafe(0);
//	mActiveFEMClothActors.resize(nbActors);
//	PxMemCopy(mActiveFEMClothActors.begin(), actors, sizeof(PxActor*) * nbActors);
//}
#endif

PxActor** Sc::Scene::getFrozenActors(PxU32& nbActorsOut)
{
	nbActorsOut = mFrozenActors.size();

	if(!nbActorsOut)
		return NULL;

	return mFrozenActors.begin();
}

void Sc::Scene::reserveTriggerReportBufferSpace(const PxU32 pairCount, PxTriggerPair*& triggerPairBuffer, TriggerPairExtraData*& triggerPairExtraBuffer)
{
	const PxU32 oldSize = mTriggerBufferAPI.size();
	const PxU32 newSize = oldSize + pairCount;
	const PxU32 newCapacity = PxU32(newSize * 1.5f);
	mTriggerBufferAPI.reserve(newCapacity);
	mTriggerBufferAPI.forceSize_Unsafe(newSize);
	triggerPairBuffer = mTriggerBufferAPI.begin() + oldSize;

	PX_ASSERT(oldSize == mTriggerBufferExtraData->size());
	mTriggerBufferExtraData->reserve(newCapacity);
	mTriggerBufferExtraData->forceSize_Unsafe(newSize);
	triggerPairExtraBuffer = mTriggerBufferExtraData->begin() + oldSize;
}

void Sc::Scene::clearSleepWakeBodies(void)
{
	// PT: TODO: refactor/templatize all that stuff

	// Clear sleep/woken marker flags
	BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
	for(PxU32 i=0; i < mSleepBodies.size(); i++)
	{
		ActorSim* body = sleepingBodies[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(ActorSim::BF_WAKEUP_NOTIFY));
		body->clearInternalFlag(ActorSim::BF_SLEEP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST);
        body->clearInternalFlag(ActorSim::BF_IS_IN_WAKEUP_LIST);
	}

#if PX_SUPPORT_GPU_PHYSX
	SoftBodyCore* const* sleepingSoftBodies = mSleepSoftBodies.getEntries();
	for (PxU32 i = 0; i < mSleepSoftBodies.size(); i++)
	{
		ActorSim* body = sleepingSoftBodies[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(ActorSim::BF_WAKEUP_NOTIFY));
		body->clearInternalFlag(ActorSim::BF_SLEEP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST);
		body->clearInternalFlag(ActorSim::BF_IS_IN_WAKEUP_LIST);
	}

	HairSystemCore* const* sleepingHairSystems = mSleepHairSystems.getEntries();
	for (PxU32 i = 0; i < mSleepHairSystems.size(); i++)
	{
		ActorSim* body = sleepingHairSystems[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(ActorSim::BF_WAKEUP_NOTIFY));
		body->clearInternalFlag(ActorSim::BF_SLEEP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST);
		body->clearInternalFlag(ActorSim::BF_IS_IN_WAKEUP_LIST);
	}
#endif

	BodyCore* const* wokenBodies = mWokeBodies.getEntries();
	for(PxU32 i=0; i < mWokeBodies.size(); i++)
	{
		BodySim* body = wokenBodies[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY));
		body->clearInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST);
        body->clearInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}

#if PX_SUPPORT_GPU_PHYSX
	SoftBodyCore* const* wokenSoftBodies = mWokeSoftBodies.getEntries();
	for (PxU32 i = 0; i < mWokeSoftBodies.size(); i++)
	{
		SoftBodySim* body = wokenSoftBodies[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY));
		body->clearInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST);
		body->clearInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}

	HairSystemCore* const* wokenHairSystems = mWokeHairSystems.getEntries();
	for (PxU32 i = 0; i < mWokeHairSystems.size(); i++)
	{
		HairSystemSim* body = wokenHairSystems[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY));
		body->clearInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST);
		body->clearInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}
#endif

	mSleepBodies.clear();
	mWokeBodies.clear();
	mWokeBodyListValid = true;
	mSleepBodyListValid = true;

	// PT: TODO: why aren't these inside PX_SUPPORT_GPU_PHYSX?
	mSleepSoftBodies.clear();
	mWokeSoftBodies.clear();
	mWokeSoftBodyListValid = true;
	mSleepSoftBodyListValid = true;

	// PT: TODO: why aren't these inside PX_SUPPORT_GPU_PHYSX?
	mSleepHairSystems.clear();
	mWokeHairSystems.clear();
	mWokeHairSystemListValid = true;
	mSleepHairSystemListValid = true;
}

void Sc::Scene::onBodySleep(BodySim* body)
{
	if (!mSimulationEventCallback && !mOnSleepingStateChanged)
		return;

	if (body->readInternalFlag(ActorSim::BF_WAKEUP_NOTIFY))
	{
		PX_ASSERT(!body->readInternalFlag(ActorSim::BF_SLEEP_NOTIFY));

		// Body is in the list of woken bodies, hence, mark this list as dirty such that it gets cleaned up before
		// being sent to the user
		body->clearInternalFlag(ActorSim::BF_WAKEUP_NOTIFY);
		mWokeBodyListValid = false;
	}

	body->raiseInternalFlag(ActorSim::BF_SLEEP_NOTIFY);

	// Avoid multiple insertion (the user can do multiple transitions between asleep and awake)
	if (!body->readInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST))
	{
		PX_ASSERT(!mSleepBodies.contains(&body->getBodyCore()));
		mSleepBodies.insert(&body->getBodyCore());
		body->raiseInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST);
	}
}

void Sc::Scene::onBodyWakeUp(BodySim* body)
{
	if(!mSimulationEventCallback && !mOnSleepingStateChanged)
		return;

	if (body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY))
	{
		PX_ASSERT(!body->readInternalFlag(BodySim::BF_WAKEUP_NOTIFY));

		// Body is in the list of sleeping bodies, hence, mark this list as dirty such it gets cleaned up before
		// being sent to the user
		body->clearInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		mSleepBodyListValid = false;
	}

	body->raiseInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

	// Avoid multiple insertion (the user can do multiple transitions between asleep and awake)
	if (!body->readInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST))
	{
		PX_ASSERT(!mWokeBodies.contains(&body->getBodyCore()));
		mWokeBodies.insert(&body->getBodyCore());
		body->raiseInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}
}

PX_INLINE void Sc::Scene::cleanUpSleepBodies()
{
	BodyCore* const* bodyArray = mSleepBodies.getEntries();
	PxU32 bodyCount = mSleepBodies.size();

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	while (bodyCount--)
	{
		ActorSim* actor = bodyArray[bodyCount]->getSim();
		BodySim* body = static_cast<BodySim*>(actor);

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_WAKEUP_NOTIFY)))
		{
			body->clearInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_IS_IN_WAKEUP_LIST));
			mSleepBodies.erase(bodyArray[bodyCount]);
		}
		else if (islandSim.getNode(body->getNodeIndex()).isActive())
		{
			//This body is still active in the island simulation, so the request to deactivate the actor by the application must have failed. Recover by undoing this
			mSleepBodies.erase(bodyArray[bodyCount]);
			actor->internalWakeUp();

		}
	}

	mSleepBodyListValid = true;
}

PX_INLINE void Sc::Scene::cleanUpWokenBodies()
{
	cleanUpSleepOrWokenBodies(mWokeBodies, BodySim::BF_SLEEP_NOTIFY, mWokeBodyListValid);
}

PX_INLINE void Sc::Scene::cleanUpSleepOrWokenBodies(PxCoalescedHashSet<BodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker)
{
	// With our current logic it can happen that a body is added to the sleep as well as the woken body list in the
	// same frame.
	//
	// Examples:
	// - Kinematic is created (added to woken list) but has not target (-> deactivation -> added to sleep list)
	// - Dynamic is created (added to woken list) but is forced to sleep by user (-> deactivation -> added to sleep list)
	//
	// This code traverses the sleep/woken body list and removes bodies which have been initially added to the given
	// list but do not belong to it anymore.

	BodyCore* const* bodyArray = bodyList.getEntries();
	PxU32 bodyCount = bodyList.size();
	while (bodyCount--)
	{
		BodySim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(removeFlag)))
			bodyList.erase(bodyArray[bodyCount]);
	}

	validMarker = true;
}

#if PX_SUPPORT_GPU_PHYSX
void Sc::Scene::cleanUpSleepHairSystems()
{
	HairSystemCore* const* hairSystemArray = mSleepHairSystems.getEntries();
	PxU32 bodyCount = mSleepBodies.size();

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	while (bodyCount--)
	{
		
		HairSystemSim* hairSystem = hairSystemArray[bodyCount]->getSim();

		if (hairSystem->readInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_WAKEUP_NOTIFY)))
		{
			hairSystem->clearInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_IS_IN_WAKEUP_LIST));
			mSleepHairSystems.erase(hairSystemArray[bodyCount]);
		}
		else if (islandSim.getNode(hairSystem->getNodeIndex()).isActive())
		{
			//This hairSystem is still active in the island simulation, so the request to deactivate the actor by the application must have failed. Recover by undoing this
			mSleepHairSystems.erase(hairSystemArray[bodyCount]);
			hairSystem->internalWakeUp();
		}
	}
	mSleepBodyListValid = true;
}

void Sc::Scene::cleanUpWokenHairSystems()
{
	cleanUpSleepOrWokenHairSystems(mWokeHairSystems, BodySim::BF_SLEEP_NOTIFY, mWokeHairSystemListValid);
}

void Sc::Scene::cleanUpSleepOrWokenHairSystems(PxCoalescedHashSet<HairSystemCore*>& bodyList, PxU32 removeFlag, bool& validMarker)
{
	HairSystemCore* const* hairSystemArray = bodyList.getEntries();
	PxU32 bodyCount = bodyList.size();
	while (bodyCount--)
	{
		HairSystemSim* hairSystem = hairSystemArray[bodyCount]->getSim();

		if (hairSystem->readInternalFlag(static_cast<BodySim::InternalFlags>(removeFlag)))
			bodyList.erase(hairSystemArray[bodyCount]);
	}

	validMarker = true;
}

PX_INLINE void Sc::Scene::cleanUpSleepSoftBodies()
{
	SoftBodyCore* const* bodyArray = mSleepSoftBodies.getEntries();
	PxU32 bodyCount = mSleepBodies.size();

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	while (bodyCount--)
	{
		SoftBodySim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_WAKEUP_NOTIFY)))
		{
			body->clearInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_IS_IN_WAKEUP_LIST));
			mSleepSoftBodies.erase(bodyArray[bodyCount]);
		}
		else if (islandSim.getNode(body->getNodeIndex()).isActive())
		{
			//This body is still active in the island simulation, so the request to deactivate the actor by the application must have failed. Recover by undoing this
			mSleepSoftBodies.erase(bodyArray[bodyCount]);
			body->internalWakeUp();
		}
	}

	mSleepBodyListValid = true;
}

PX_INLINE void Sc::Scene::cleanUpWokenSoftBodies()
{
	cleanUpSleepOrWokenSoftBodies(mWokeSoftBodies, BodySim::BF_SLEEP_NOTIFY, mWokeSoftBodyListValid);
}

PX_INLINE void Sc::Scene::cleanUpSleepOrWokenSoftBodies(PxCoalescedHashSet<SoftBodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker)
{
	// With our current logic it can happen that a body is added to the sleep as well as the woken body list in the
	// same frame.
	//
	// Examples:
	// - Kinematic is created (added to woken list) but has not target (-> deactivation -> added to sleep list)
	// - Dynamic is created (added to woken list) but is forced to sleep by user (-> deactivation -> added to sleep list)
	//
	// This code traverses the sleep/woken body list and removes bodies which have been initially added to the given
	// list but do not belong to it anymore.

	SoftBodyCore* const* bodyArray = bodyList.getEntries();
	PxU32 bodyCount = bodyList.size();
	while (bodyCount--)
	{
		SoftBodySim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(removeFlag)))
			bodyList.erase(bodyArray[bodyCount]);
	}

	validMarker = true;
}
#endif //PX_SUPPORT_GPU_PHYSX

void Sc::Scene::releaseConstraints(bool endOfScene)
{
	PX_ASSERT(mLLContext);

	if(getStabilizationEnabled())
	{
		//If stabilization is enabled, we're caching contacts for next frame
		if(!endOfScene)
		{
			//So we only clear memory (flip buffers) when not at the end-of-scene.
			//This means we clear after narrow phase completed so we can 
			//release the previous frame's contact buffers before we enter the solve phase.
			mLLContext->getNpMemBlockPool().releaseContacts();
		}
	}
	else if(endOfScene)
	{
		//We now have a double-buffered pool of mem blocks so we must
		//release both pools (which actually triggers the memory used this 
		//frame to be released 
		mLLContext->getNpMemBlockPool().releaseContacts();
		mLLContext->getNpMemBlockPool().releaseContacts();
	}
}

PX_INLINE void Sc::Scene::clearBrokenConstraintBuffer()
{
	mBrokenConstraints.clear();
}

void Sc::Scene::addToLostTouchList(ActorSim& body1, ActorSim& body2)
{
	PX_ASSERT(!body1.isStaticRigid());
	PX_ASSERT(!body2.isStaticRigid());
	SimpleBodyPair p = { &body1, &body2, body1.getActorID(), body2.getActorID() };
	mLostTouchPairs.pushBack(p);
}

FeatherstoneArticulation* Sc::Scene::createLLArticulation(Sc::ArticulationSim* sim)
{
	return mLLArticulationRCPool->construct(sim);
}

void Sc::Scene::destroyLLArticulation(FeatherstoneArticulation& articulation)
{
	mLLArticulationRCPool->destroy(static_cast<Dy::FeatherstoneArticulation*>(&articulation));
}

#if PX_SUPPORT_GPU_PHYSX

Dy::SoftBody* Sc::Scene::createLLSoftBody(Sc::SoftBodySim* sim)
{
	return mLLSoftBodyPool->construct(sim, sim->getCore().getCore());
}

void Sc::Scene::destroyLLSoftBody(Dy::SoftBody& softBody)
{
	mLLSoftBodyPool->destroy(&softBody);
}

Dy::FEMCloth* Sc::Scene::createLLFEMCloth(Sc::FEMClothSim* sim)
{
	return mLLFEMClothPool->construct(sim, sim->getCore().getCore());
}

void Sc::Scene::destroyLLFEMCloth(Dy::FEMCloth& femCloth)
{
	mLLFEMClothPool->destroy(&femCloth);
}

Dy::ParticleSystem*	Sc::Scene::createLLParticleSystem(Sc::ParticleSystemSim* sim)
{
	return mLLParticleSystemPool->construct(sim->getCore().getShapeCore().getLLCore());
}

void Sc::Scene::destroyLLParticleSystem(Dy::ParticleSystem& particleSystem)
{
	return mLLParticleSystemPool->destroy(&particleSystem);
}

Dy::HairSystem* Sc::Scene::createLLHairSystem(Sc::HairSystemSim* sim)
{
	return mLLHairSystemPool->construct(sim, sim->getCore().getShapeCore().getLLCore());
}

void Sc::Scene::destroyLLHairSystem(Dy::HairSystem& hairSystem)
{
	mLLHairSystemPool->destroy(&hairSystem);
}

#endif //PX_SUPPORT_GPU_PHYSX

PxU32 Sc::Scene::createAggregate(void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint)
{
	const physx::Bp::BoundsIndex index = getElementIDPool().createID();
	mBoundsArray->initEntry(index);
	mLLContext->getNphaseImplementationContext()->registerAggregate(index);
#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	return mAABBManager->createAggregate(index, Bp::FilterGroup::eINVALID, userData, maxNumShapes, filterHint);
#else
	// PT: TODO: ideally a static compound would have a static group
	const PxU32 rigidId	= getRigidIDTracker().createID();
	const Bp::FilterGroup::Enum bpGroup = Bp::FilterGroup::Enum(rigidId + Bp::FilterGroup::eDYNAMICS_BASE);
	return mAABBManager->createAggregate(index, bpGroup, userData, selfCollisions);
#endif
}

void Sc::Scene::deleteAggregate(PxU32 id)
{
	Bp::BoundsIndex index;
	Bp::FilterGroup::Enum bpGroup;
#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	if(mAABBManager->destroyAggregate(index, bpGroup, id))
	{
		getElementIDPool().releaseID(index);
	}
#else
	if(mAABBManager->destroyAggregate(index, bpGroup, id))
	{
		getElementIDPool().releaseID(index);

		// PT: this is clumsy....
		const PxU32 rigidId	= PxU32(bpGroup) - Bp::FilterGroup::eDYNAMICS_BASE;
		getRigidIDTracker().releaseID(rigidId);
	}
#endif
}

void Sc::Scene::shiftOrigin(const PxVec3& shift)
{
	// adjust low level context
	mLLContext->shiftOrigin(shift);

	// adjust bounds array
	mBoundsArray->shiftOrigin(shift);

	// adjust broadphase
	mAABBManager->shiftOrigin(shift);

	// adjust constraints
	ConstraintCore*const * constraints = mConstraints.getEntries();
	for(PxU32 i=0, size = mConstraints.size(); i < size; i++)
		constraints[i]->getPxConnector()->onOriginShift(shift);
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE bool isParticleSystem(const PxActorType::Enum actorType)
{
	return actorType == PxActorType::ePBD_PARTICLESYSTEM || actorType == PxActorType::eFLIP_PARTICLESYSTEM
		|| actorType == PxActorType::eMPM_PARTICLESYSTEM || actorType == PxActorType::eCUSTOM_PARTICLESYSTEM;
}

void Sc::Scene::islandInsertion(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.islandInsertion", getContextId());

		const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
		for (PxU32 a = 0; a < nbShapeIdxCreated; ++a)
		{
			size_t address = size_t(mPreallocatedShapeInteractions[a]);
			if (address & 1)
			{
				ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address&size_t(~1));

				PxsContactManager* contactManager = const_cast<PxsContactManager*>(interaction->getContactManager());

				Sc::ActorSim& bs0 = interaction->getShape0().getActor();
				Sc::ActorSim& bs1 = interaction->getShape1().getActor();

				PxActorType::Enum actorTypeLargest = PxMax(bs0.getActorType(), bs1.getActorType());

				PxNodeIndex nodeIndexB;
				if (!bs1.isStaticRigid())
					nodeIndexB = bs1.getNodeIndex();

				IG::Edge::EdgeType type = IG::Edge::eCONTACT_MANAGER;
				if(actorTypeLargest == PxActorType::eSOFTBODY)
					type = IG::Edge::eSOFT_BODY_CONTACT;
				else if (actorTypeLargest == PxActorType::eFEMCLOTH)
					type = IG::Edge::eFEM_CLOTH_CONTACT;
				else if(isParticleSystem(actorTypeLargest))
					type = IG::Edge::ePARTICLE_SYSTEM_CONTACT;
				else if (actorTypeLargest == PxActorType::eHAIRSYSTEM)
					type = IG::Edge::eHAIR_SYSTEM_CONTACT;

				IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(contactManager, bs0.getNodeIndex(), nodeIndexB, interaction, type);

				interaction->mEdgeIndex = edgeIdx;

				if (contactManager)
					contactManager->getWorkUnit().mEdgeIndex = edgeIdx;

				//If it is a soft body or particle overlap, treat it as a contact for now (we can hook up touch found/lost events later maybe)
				if (actorTypeLargest > PxActorType::eARTICULATION_LINK)
					mSimpleIslandManager->setEdgeConnected(edgeIdx, type);
			}
		}

		// - Wakes actors that lost touch if appropriate
		//processLostTouchPairs();

		if(mCCDPass == 0)
		{
			mSimpleIslandManager->firstPassIslandGen();
		}
	}
}

void Sc::Scene::registerContactManagers(PxBaseTask* /*continuation*/)
{
	{
		PxvNphaseImplementationContext* nphaseContext = mLLContext->getNphaseImplementationContext();
		nphaseContext->lock();
		PX_PROFILE_ZONE("Sim.processNewOverlaps.registerCms", getContextId());
		//nphaseContext->registerContactManagers(mPreallocatedContactManagers.begin(), mPreallocatedContactManagers.size(), mLLContext->getContactManagerPool().getMaxUsedIndex());
		const PxU32 nbCmsCreated = mPreallocatedContactManagers.size();
		for (PxU32 a = 0; a < nbCmsCreated; ++a)
		{
			size_t address = size_t(mPreallocatedContactManagers[a]);
			if (address & 1)
			{
				PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(address&size_t(~1));
				size_t address2 = size_t(mPreallocatedShapeInteractions[a]);
				ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address2&size_t(~1));
				nphaseContext->registerContactManager(cm, interaction, 0, 0);
			}
		}
		nphaseContext->unlock();
	}
}

void Sc::Scene::registerInteractions(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.registerInteractions", getContextId());
		const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
		for (PxU32 a = 0; a < nbShapeIdxCreated; ++a)
		{
			size_t address = size_t(mPreallocatedShapeInteractions[a]);
			if (address & 1)
			{
				ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address&size_t(~1));

				ActorSim& actorSim0 = interaction->getActorSim0();
				ActorSim& actorSim1 = interaction->getActorSim1();
				actorSim0.registerInteractionInActor(interaction);
				actorSim1.registerInteractionInActor(interaction);

				/*Sc::BodySim* bs0 = actorSim0.isDynamicRigid() ? static_cast<BodySim*>(&actorSim0) : NULL;
				Sc::BodySim* bs1 = actorSim1.isDynamicRigid() ? static_cast<BodySim*>(&actorSim1) : NULL;*/

				if (actorSim0.isDynamicRigid())
				{
					Sc::BodySim* bs0 = static_cast<BodySim*>(&actorSim0);
					bs0->registerCountedInteraction();
				}

				if (actorSim1.isDynamicRigid())
				{
					Sc::BodySim* bs1 = static_cast<BodySim*>(&actorSim1);
					bs1->registerCountedInteraction();
				}
			}
		}

		const PxU32 nbMarkersCreated = mPreallocatedInteractionMarkers.size();
		for (PxU32 a = 0; a < nbMarkersCreated; ++a)
		{
			size_t address = size_t(mPreallocatedInteractionMarkers[a]);
			if (address & 1)
			{
				ElementInteractionMarker* interaction = reinterpret_cast<ElementInteractionMarker*>(address&size_t(~1));
				interaction->registerInActors(NULL);
			}
		}
	}
}

void Sc::Scene::registerSceneInteractions(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.processNewOverlaps.registerInteractionsScene", getContextId());
	const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
	for (PxU32 a = 0; a < nbShapeIdxCreated; ++a)
	{
		size_t address = size_t(mPreallocatedShapeInteractions[a]);
		if (address & 1)
		{
			ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address&size_t(~1));
			registerInteraction(interaction, interaction->getContactManager() != NULL);
			mNPhaseCore->registerInteraction(interaction);
			
			const PxsContactManager* cm = interaction->getContactManager();
			if(cm)
				mLLContext->setActiveContactManager(cm, cm->getCCD());
		}
	}

	const PxU32 nbInteractionMarkers = mPreallocatedInteractionMarkers.size();
	for (PxU32 a = 0; a < nbInteractionMarkers; ++a)
	{
		size_t address = size_t(mPreallocatedInteractionMarkers[a]);
		if (address & 1)
		{
			ElementInteractionMarker* interaction = reinterpret_cast<ElementInteractionMarker*>(address&size_t(~1));
			registerInteraction(interaction, false);
			mNPhaseCore->registerInteraction(interaction);
		}
	}
}

class OverlapFilterTask : public Cm::Task
{
public:
	static const PxU32 MaxPairs = 512;
	Sc::NPhaseCore*			mNPhaseCore;
	const Bp::AABBOverlap*	mPairs;

	PxU32					mNbToProcess;

	PxU32					mKeepMap[MaxPairs/32];
	PxU32					mCallbackMap[MaxPairs/32];

	PxFilterInfo*			mFinfo;

	PxU32					mNbToKeep;
	PxU32					mNbToSuppress;
	PxU32					mNbToCallback;

	OverlapFilterTask*		mNext;

	OverlapFilterTask(PxU64 contextID, Sc::NPhaseCore* nPhaseCore, PxFilterInfo* fInfo, const Bp::AABBOverlap* pairs, const PxU32 nbToProcess) :
		Cm::Task		(contextID),
		mNPhaseCore		(nPhaseCore),
		mPairs			(pairs),
		mNbToProcess	(nbToProcess),
		mFinfo			(fInfo),
		mNbToKeep		(0),
		mNbToSuppress	(0),
		mNbToCallback	(0),
		mNext			(NULL)
	{
		PxMemZero(mKeepMap, sizeof(mKeepMap));
		PxMemZero(mCallbackMap, sizeof(mCallbackMap));
	}

	virtual void runInternal()
	{
		mNPhaseCore->runOverlapFilters(	mNbToProcess, mPairs, mFinfo, mNbToKeep, mNbToSuppress, mNbToCallback, mKeepMap, mCallbackMap);
	}

	virtual const char* getName() const { return "OverlapFilterTask"; }
};

class OnOverlapCreatedTask : public Cm::Task
{
public:
	Sc::NPhaseCore*					mNPhaseCore;
	const Bp::AABBOverlap*			mPairs;
	const PxFilterInfo*				mFinfo;
	PxsContactManager**				mContactManagers;
	Sc::ShapeInteraction**			mShapeInteractions;
	Sc::ElementInteractionMarker**	mInteractionMarkers;
	PxU32							mNbToProcess;

	OnOverlapCreatedTask(PxU64 contextID, Sc::NPhaseCore* nPhaseCore, const Bp::AABBOverlap* pairs, const PxFilterInfo* fInfo, PxsContactManager** contactManagers, Sc::ShapeInteraction** shapeInteractions, Sc::ElementInteractionMarker** interactionMarkers,
		PxU32 nbToProcess) :
		Cm::Task			(contextID),
		mNPhaseCore			(nPhaseCore),
		mPairs				(pairs),
		mFinfo				(fInfo),
		mContactManagers	(contactManagers),
		mShapeInteractions	(shapeInteractions),
		mInteractionMarkers	(interactionMarkers),
		mNbToProcess		(nbToProcess)
	{
	}

	virtual void runInternal()
	{
		PxsContactManager** currentCm = mContactManagers;
		Sc::ShapeInteraction** currentSI = mShapeInteractions;
		Sc::ElementInteractionMarker** currentEI = mInteractionMarkers;

		for(PxU32 i=0; i<mNbToProcess; i++)
		{
			const Bp::AABBOverlap& pair = mPairs[i];
			Sc::ShapeSimBase* s0 = reinterpret_cast<Sc::ShapeSimBase*>(pair.mUserData1);
			Sc::ShapeSimBase* s1 = reinterpret_cast<Sc::ShapeSimBase*>(pair.mUserData0);

			Sc::ElementSimInteraction* interaction = mNPhaseCore->createRbElementInteraction(mFinfo[i], *s0, *s1, *currentCm, *currentSI, *currentEI, 0);
			if(interaction)
			{
				if(interaction->getType() == Sc::InteractionType::eOVERLAP)
				{
					*currentSI = reinterpret_cast<Sc::ShapeInteraction*>(size_t(*currentSI) | 1);
					currentSI++;

					if(static_cast<Sc::ShapeInteraction*>(interaction)->getContactManager())
					{
						*currentCm = reinterpret_cast<PxsContactManager*>(size_t(*currentCm) | 1);
						currentCm++;
					}
				}
				else if(interaction->getType() == Sc::InteractionType::eMARKER)
				{
					*currentEI = reinterpret_cast<Sc::ElementInteractionMarker*>(size_t(*currentEI) | 1);
					currentEI++;
				}
			}
		}
	}

	virtual const char* getName() const { return "OnOverlapCreatedTask"; }
};

void Sc::Scene::finishBroadPhase(PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
	PX_PROFILE_ZONE("Sc::Scene::finishBroadPhase", getContextId());

	Bp::AABBManagerBase* aabbMgr = mAABBManager;

	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps", getContextId());

		{
			//KS - these functions call "registerInActors", while OverlapFilterTask reads the list of interactions
			//in an actor. This could lead to a race condition and a crash if they occur at the same time, so we 
			//serialize these operations
			PX_PROFILE_ZONE("Sim.processNewOverlaps.createOverlapsNoShapeInteractions", getContextId());
			{
				PxU32 createdOverlapCount;
				const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getCreatedOverlaps(Bp::ElementType::eTRIGGER, createdOverlapCount);

				mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;
				mNPhaseCore->onTriggerOverlapCreated(p, createdOverlapCount);
			}
		}

		{
			PxU32 createdOverlapCount;
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getCreatedOverlaps(Bp::ElementType::eSHAPE, createdOverlapCount);
			{
				//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
				mPreallocatedContactManagers.reserve(1);
				mPreallocatedShapeInteractions.reserve(1);
				mPreallocatedInteractionMarkers.reserve(1);

				mPreallocatedContactManagers.forceSize_Unsafe(1);
				mPreallocatedShapeInteractions.forceSize_Unsafe(1);
				mPreallocatedInteractionMarkers.forceSize_Unsafe(1);
			}

			mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;

			mPreallocateContactManagers.setContinuation(continuation);
			Cm::FlushPool& flushPool = mLLContext->getTaskPool();

			mFilterInfo.forceSize_Unsafe(0);
			mFilterInfo.reserve(createdOverlapCount);
			mFilterInfo.forceSize_Unsafe(createdOverlapCount);

			const PxU32 nbPairsPerTask = OverlapFilterTask::MaxPairs;
			mOverlapFilterTaskHead = NULL;
			OverlapFilterTask* previousTask = NULL;
			for(PxU32 a=0; a<createdOverlapCount; a+=nbPairsPerTask)
			{
				PxU32 nbToProcess = PxMin(createdOverlapCount - a, nbPairsPerTask);
				OverlapFilterTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OverlapFilterTask)), OverlapFilterTask)(getContextId(), mNPhaseCore, mFilterInfo.begin() + a, p + a, nbToProcess);

				task->setContinuation(&mPreallocateContactManagers);
				task->removeReference();

				if(previousTask)
					previousTask->mNext = task;
				else
					mOverlapFilterTaskHead = task;

				previousTask = task;
			}
		}

		mPreallocateContactManagers.removeReference();
	}	
}

void Sc::Scene::preallocateContactManagers(PxBaseTask* continuation)
{
	//Iterate over all filter tasks and work out how many pairs we need...

	PxU32 createdOverlapCount = 0;

	PxU32 totalCreatedPairs = 0;
	PxU32 totalSuppressPairs = 0;

	PxU32 overlapCount;
	Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(Bp::ElementType::eSHAPE, overlapCount);
	PxFilterInfo* fInfo = mFilterInfo.begin();

	OverlapFilterTask* task = mOverlapFilterTaskHead;
	while(task)
	{
		if(task->mNbToCallback)
		{
			//Iterate and process callbacks. Refilter then increment the results, setting the appropriate settings
			const FilteringContext context(*this, mNPhaseCore->mFilterPairManager);

			for(PxU32 w = 0; w < (OverlapFilterTask::MaxPairs / 32); ++w)
			{
				for(PxU32 b = task->mCallbackMap[w]; b; b &= b - 1)
				{
					const PxU32 index = (w << 5) + PxLowestSetBit(b);
					const Bp::AABBOverlap& pair = task->mPairs[index];
					Sc::ShapeSim* s0 = reinterpret_cast<Sc::ShapeSim*>(pair.mUserData0);
					Sc::ShapeSim* s1 = reinterpret_cast<Sc::ShapeSim*>(pair.mUserData1);

					bool isNonRigid = s0->getActor().isNonRigid() || s1->getActor().isNonRigid();
					

					const PxFilterInfo finfo = filterRbCollisionPairSecondStage(context, *s0, *s1, s0->getActor(), s1->getActor(), INVALID_FILTER_PAIR_INDEX, true, isNonRigid);

					task->mFinfo[index] = finfo;

					if(!(finfo.filterFlags & PxFilterFlag::eKILL))
					{
						if((finfo.filterFlags & PxFilterFlag::eSUPPRESS) == false)
							task->mNbToKeep++;
						else
							task->mNbToSuppress++;
						task->mKeepMap[index / 32] |= (1 << (index & 31));
					}
				}
			}
		}

		totalCreatedPairs += task->mNbToKeep;
		totalSuppressPairs += task->mNbToSuppress;
		task = task->mNext;
	}

	{
		//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
		mPreallocatedContactManagers.forceSize_Unsafe(0);
		mPreallocatedShapeInteractions.forceSize_Unsafe(0);
		mPreallocatedInteractionMarkers.forceSize_Unsafe(0);

		mPreallocatedContactManagers.reserve(totalCreatedPairs+1);
		mPreallocatedShapeInteractions.reserve(totalCreatedPairs+1);
		mPreallocatedInteractionMarkers.reserve(totalSuppressPairs+1);

		mPreallocatedContactManagers.forceSize_Unsafe(totalCreatedPairs);
		mPreallocatedShapeInteractions.forceSize_Unsafe(totalCreatedPairs);
		mPreallocatedInteractionMarkers.forceSize_Unsafe(totalSuppressPairs);
	}

	const PxU32 nbPairsPerTask = 256;
	PxsContactManager** cms = mPreallocatedContactManagers.begin();
	Sc::ShapeInteraction** shapeInter = mPreallocatedShapeInteractions.begin();
	Sc::ElementInteractionMarker** markerIter = mPreallocatedInteractionMarkers.begin();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	struct Local
	{
		static void processBatch(const PxU32 createdCurrIdx, PxU32& createdStartIdx, const PxU32 suppressedCurrIdx, PxU32& suppressedStartIdx, const PxU32 batchSize,
			PxsContext* const context, NPhaseCore* const core, OnOverlapCreatedTask* const createTask, PxBaseTask* const continuation_,
			PxsContactManager** const cms_, Sc::ShapeInteraction** const shapeInter_, Sc::ElementInteractionMarker** const markerIter_)
		{
			const PxU32 nbToCreate = createdCurrIdx - createdStartIdx;
			const PxU32 nbToSuppress = suppressedCurrIdx - suppressedStartIdx;

			{
				context->getContactManagerPool().preallocate(nbToCreate, cms_ + createdStartIdx);
			}

			{
				for (PxU32 i = 0; i < nbToCreate; ++i)
					shapeInter_[createdStartIdx + i] = core->mShapeInteractionPool.allocate();
			}
			{
				for (PxU32 i = 0; i < nbToSuppress; ++i)
					markerIter_[suppressedStartIdx + i] = core->mInteractionMarkerPool.allocate();
			}
				
			createdStartIdx = createdCurrIdx;
			suppressedStartIdx = suppressedCurrIdx;

			createTask->mNbToProcess = batchSize;
			createTask->setContinuation(continuation_);
			createTask->removeReference();
		}
	};

	// PT: TODO: why do we create the task immediately? Why not create it only when a batch is full?
	OnOverlapCreatedTask* createTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(getContextId(), mNPhaseCore, p, fInfo, cms, shapeInter, markerIter, 0);

	PxU32 batchSize = 0;
	PxU32 suppressedStartIdx = 0;
	PxU32 createdStartIdx = 0;
	PxU32 suppressedCurrIdx = 0;
	PxU32 createdCurrIdx = 0;
	PxU32 currentReadIdx = 0;

	task = mOverlapFilterTaskHead;
	while(task)
	{
		if(task->mNbToKeep || task->mNbToSuppress)
		{
			for(PxU32 w = 0; w < (OverlapFilterTask::MaxPairs/32); ++w)
			{
				for(PxU32 b = task->mKeepMap[w]; b; b &= b-1)
				{
					const PxU32 index = (w<<5) + PxLowestSetBit(b);

					if(createdOverlapCount < (index + currentReadIdx))
					{
						p[createdOverlapCount] = task->mPairs[index];
						fInfo[createdOverlapCount] = task->mFinfo[index];
					}
					createdOverlapCount++;
					batchSize++;
				}
			}

			suppressedCurrIdx += task->mNbToSuppress;
			createdCurrIdx += task->mNbToKeep;

			if(batchSize >= nbPairsPerTask)
			{
				Local::processBatch(createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize, mLLContext, mNPhaseCore, createTask, continuation, cms, shapeInter, markerIter);

				createTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(getContextId(), mNPhaseCore, p + createdOverlapCount,
					fInfo + createdOverlapCount, cms + createdStartIdx, shapeInter + createdStartIdx, markerIter + suppressedStartIdx, 0);

				batchSize = 0;
			}
		}
		currentReadIdx += OverlapFilterTask::MaxPairs;
		task = task->mNext;
	}

	if(batchSize)
		Local::processBatch(createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize, mLLContext, mNPhaseCore, createTask, continuation, cms, shapeInter, markerIter);
}

void Sc::Scene::finishBroadPhaseStage2(const PxU32 ccdPass)
{
	PX_PROFILE_ZONE("Sc::Scene::finishBroadPhase2", getContextId());

	Bp::AABBManagerBase* aabbMgr = mAABBManager;

	for(PxU32 i=0; i<Bp::ElementType::eCOUNT; i++)
	{
		PxU32 destroyedOverlapCount;
		aabbMgr->getDestroyedOverlaps(Bp::ElementType::Enum(i), destroyedOverlapCount);
		mLLContext->getSimStats().mNbLostPairs += destroyedOverlapCount;
	}

	//KS - we need to defer processing lost overlaps until later!
	if (ccdPass)
	{
		PX_PROFILE_ZONE("Sim.processLostOverlaps", getContextId());
		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		PxU32 destroyedOverlapCount;

		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);

			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				//KS - this is a bit ugly. We split the "onOverlapRemoved" for shape interactions to parallelize it and that means
				//that we have to call each of the individual stages of the remove here.

				//First, we have to get the interaction pointer...
				Sc::ElementSimInteraction* interaction = mNPhaseCore->onOverlapRemovedStage1(volume0, volume1);
				p->mPairUserData = interaction;
				if(interaction)
				{
					if(interaction->getType() == Sc::InteractionType::eOVERLAP || interaction->getType() == Sc::InteractionType::eMARKER)
					{
						//If it's a standard "overlap" interaction, we have to send a lost touch report, unregister it, and destroy its manager and island gen data.
						if(interaction->getType() == Sc::InteractionType::eOVERLAP)
						{
							Sc::ShapeInteraction* si = static_cast<Sc::ShapeInteraction*>(interaction);
							mNPhaseCore->lostTouchReports(si, PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH), NULL, 0, outputs);

							//We must check to see if we have a contact manager here. There is an edge case where actors could be put to 
							//sleep after discrete simulation, prior to CCD, causing their contactManager() to be destroyed. If their bounds
							//also ceased overlapping, then this code will try to destroy the manager again.
							if(si->getContactManager())
								si->destroyManager();
							si->clearIslandGenData();
						}

						unregisterInteraction(interaction);
						mNPhaseCore->unregisterInteraction(interaction);
					}

					//Then call "onOverlapRemoved" to actually free the interaction
					mNPhaseCore->onOverlapRemoved(volume0, volume1, ccdPass, interaction, outputs);
				}
				p++;
			}
		}

		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eTRIGGER, destroyedOverlapCount);

			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				p->mPairUserData = NULL;

				//KS - this is a bit ugly. 
				mNPhaseCore->onOverlapRemoved(volume0, volume1, ccdPass, NULL, outputs);
				p++;
			}
		}
	}

	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();

	if (ccdPass)
		aabbMgr->freeBuffers();
}

void Sc::Scene::activateEdgesInternal(const IG::EdgeIndex* activatingEdges, const PxU32 nbActivatingEdges)
{
	const IG::IslandSim& speculativeSim = mSimpleIslandManager->getSpeculativeIslandSim();
	for (PxU32 i = 0; i < nbActivatingEdges; ++i)
	{
		Sc::Interaction* interaction = mSimpleIslandManager->getInteraction(activatingEdges[i]);

		if (interaction && !interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			if (speculativeSim.getEdge(activatingEdges[i]).isActive())
			{
				const bool proceed = activateInteraction(interaction, NULL);

				if (proceed && (interaction->getType() < InteractionType::eTRACKED_IN_SCENE_COUNT))
					notifyInteractionActivated(interaction);
			}
		}
	}
}

void Sc::Scene::secondPassNarrowPhase(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.postIslandGen", getContextId());
		mSimpleIslandManager->additionalSpeculativeActivation();
		// wake interactions
		{
			PX_PROFILE_ZONE("ScScene.wakeInteractions", getContextId());
			const IG::IslandSim& speculativeSim = mSimpleIslandManager->getSpeculativeIslandSim();

			//KS - only wake contact managers based on speculative state to trigger contact gen. Waking actors based on accurate state
			//should activate and joints.
			{
				//Wake speculatively based on rigid contacts, soft contacts and particle contacts
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER), speculativeSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eSOFT_BODY_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::eSOFT_BODY_CONTACT));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eFEM_CLOTH_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::eFEM_CLOTH_CONTACT));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::ePARTICLE_SYSTEM_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::ePARTICLE_SYSTEM_CONTACT));
				activateEdgesInternal(speculativeSim.getActivatedEdges(IG::Edge::eHAIR_SYSTEM_CONTACT), speculativeSim.getNbActivatedEdges(IG::Edge::eHAIR_SYSTEM_CONTACT));
			}
		}
	}
	mLLContext->secondPassUpdateContactManager(mDt, &mPostNarrowPhase); // Starts update of contact managers
}

//~BROADPHASE

void Sc::activateInteractions(Sc::ActorSim& actorSim)
{
	using namespace Sc;

	const PxU32 nbInteractions = actorSim.getActorInteractionCount();
	if(!nbInteractions)
		return;

	Interaction** interactions = actorSim.getActorInteractions();
	Scene& scene = actorSim.getScene();

	for(PxU32 i=0; i<nbInteractions; ++i)
	{
		PxPrefetchLine(interactions[PxMin(i+1,nbInteractions-1)]);
		Interaction* interaction = interactions[i];

		if(!interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			const InteractionType::Enum type = interaction->getType();
			const bool isNotIGControlled = type != InteractionType::eOVERLAP && type != InteractionType::eMARKER;

			if((isNotIGControlled))
			{
				const bool proceed = activateInteraction(interaction, NULL);
				if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
					scene.notifyInteractionActivated(interaction);
			}
		}
	}
}

void Sc::deactivateInteractions(Sc::ActorSim& actorSim)
{
	using namespace Sc;

	const PxU32 nbInteractions = actorSim.getActorInteractionCount();
	if(!nbInteractions)
		return;

	Interaction** interactions = actorSim.getActorInteractions();
	Scene& scene = actorSim.getScene();

	for(PxU32 i=0; i<nbInteractions; ++i)
	{
		PxPrefetchLine(interactions[PxMin(i+1,nbInteractions-1)]);
		Interaction* interaction = interactions[i];

		if(interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			const InteractionType::Enum type = interaction->getType();
			const bool isNotIGControlled = type != InteractionType::eOVERLAP && type != InteractionType::eMARKER;
			if(isNotIGControlled)
			{
				const bool proceed = deactivateInteraction(interaction, type);
				if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
					scene.notifyInteractionDeactivated(interaction);
			}
		}
	}
}

Sc::ConstraintCore*	Sc::Scene::findConstraintCore(const Sc::ActorSim* sim0, const Sc::ActorSim* sim1)
{
	const PxNodeIndex ind0 = sim0->getNodeIndex();
	const PxNodeIndex ind1 = sim1->getNodeIndex();

	if(ind1 < ind0)
		PxSwap(sim0, sim1);

	const PxHashMap<PxPair<const Sc::ActorSim*, const Sc::ActorSim*>, Sc::ConstraintCore*>::Entry* entry = mConstraintMap.find(PxPair<const Sc::ActorSim*, const Sc::ActorSim*>(sim0, sim1));
	return entry ? entry->second : NULL;
}

void Sc::Scene::updateBodySim(Sc::BodySim& bodySim)
{
	Dy::FeatherstoneArticulation* arti = NULL;
	Sc::ArticulationSim* artiSim = bodySim.getArticulation();
	if (artiSim)
		arti = artiSim->getLowLevelArticulation();
	mSimulationController->updateDynamic(arti, bodySim.getNodeIndex());
}
