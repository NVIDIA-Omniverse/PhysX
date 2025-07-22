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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

#include "PxgSimulationController.h"
#include "PxDirectGPUAPI.h"
#include "PxsRigidBody.h"
#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "cudamanager/PxCudaContextManager.h"
#include "foundation/PxAllocator.h"
#include "PxgSimulationCore.h"
#include "PxgDynamicsContext.h"
#include "PxgNphaseImplementationContext.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgCudaSolverCore.h"
#include "PxgBodySim.h"
#include "PxgShapeSim.h"
#include "PxsTransformCache.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgKernelWrangler.h"
#include "GuBounds.h"
#include "GuTriangleMesh.h"
#include "GuTetrahedronMesh.h"
#include "GuTetrahedronMeshUtils.h"
#include "CmFlushPool.h"
#include "DyFeatherstoneArticulation.h"
#include "DyDeformableSurface.h"
#include "DyDeformableVolume.h"
#include "DyParticleSystem.h"
#include "PxgArticulationCore.h"
#include "PxgSoftBody.h"
#include "PxgSoftBodyCore.h"
#include "PxgFEMCloth.h"
#include "PxgFEMClothCore.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemCore.h"
#include "PxArticulationTendonData.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxMathUtils.h"
#include "GuDistancePointTetrahedron.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgPBDParticleSystemCore.h"

#include "foundation/PxPreprocessor.h"
#include "common/PxProfileZone.h"
#include "PxsContext.h"
#include "PxgSimulationCore.h"
#include "PxPhysXGpu.h"
#include "cudamanager/PxCudaContext.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgCudaUtils.h"
#include "BpAABBManagerBase.h"
#include "PxgAABBManager.h"
#include "PxgSimulationCoreKernelIndices.h"

// PT: TODO: don't indent the whole damn file

using namespace physx;

static PX_FORCE_INLINE void getGRBVertexIndices(PxU32& vref0, PxU32& vref1, PxU32& vref2, PxU32 triIndex, const Gu::TriangleMesh* triangleMesh)
{
	const PxU32 offset = triIndex * 3;
	if(triangleMesh->has16BitIndices())
	{
		const PxU16* triInds = reinterpret_cast<PxU16*>(triangleMesh->mGRB_triIndices);
		vref0 = triInds[offset + 0];
		vref1 = triInds[offset + 1];
		vref2 = triInds[offset + 2];
	}
	else
	{
		const PxU32* triInds = reinterpret_cast<PxU32*>(triangleMesh->mGRB_triIndices);
		vref0 = triInds[offset + 0];
		vref1 = triInds[offset + 1];
		vref2 = triInds[offset + 2];
	}
}

namespace physx
{
	extern "C" void initSimulationControllerKernels0();
	extern "C" void initSimulationControllerKernels1();
	extern "C" void initSimulationControllerKernels2();

	void createPxgSimulationController()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXSimulationControllerGpu linkage as Static Library!
		initSimulationControllerKernels0();
		initSimulationControllerKernels1();
		initSimulationControllerKernels2();
#endif
	}

	PxgSimulationController::PxgSimulationController(PxsKernelWranglerManager* gpuWranglerManager, PxCudaContextManager* cudaContextManager, PxgGpuContext* dynamicContext,
		PxgNphaseImplementationContext* npContext, Bp::BroadPhase* bp, bool useGpuBroadphase, PxsSimulationControllerCallback* callback,
		PxgHeapMemoryAllocatorManager* heapMemoryManager, PxU32 maxSoftBodyContacts, PxU32 maxFemClothContacts, PxU32 maxParticleContacts,
		PxU32 collisionStackSizeBytes, bool enableBodyAccelerations) :
		PxsSimulationController(callback, PxIntTrue),
		mPostCopyShapeSimTask(*this),
		mPostCopyBodySimTask(*this, enableBodyAccelerations),
		mPostUpdateParticleSystemTask(*this),
		mBodySimManager(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mJointManager(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators), dynamicContext->getEnableDirectGPUAPI()),
		mSoftBodyCore(NULL),
		mFEMClothCore(NULL),
		mPBDParticleSystemCore(NULL),
		mDynamicContext(dynamicContext), mNpContext(npContext),
		mNewBodySimPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinksPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkWakeCounterPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkAccelPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkPropPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkChildPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkParentPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkBody2WorldPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkBody2ActorPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mJointPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mJointDataPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mLinkJointIndexPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mArticulationPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mSpatialTendonParamPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mSpatialTendonPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mAttachmentFixedPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mAttachmentModPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mTendonAttachmentMapPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mFixedTendonParamPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mFixedTendonPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mTendonJointFixedDataPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mTendonJointCoefficientDataPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mTendonTendonJointMapPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mPathToRootPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mMimicJointPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mArticulationUpdatePool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mArticulationDofDataPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mNewSoftBodyPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mSoftBodyPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mSoftBodyElementIndexPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mParticleSoftBodyAttachments(heapMemoryManager),
		mSoftBodyParticleFilterPairs(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mRigidSoftBodyAttachments(heapMemoryManager),
		mSoftBodyRigidFilterPairs(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mSoftBodySoftBodyAttachments(heapMemoryManager),
		mSoftBodyClothAttachments(heapMemoryManager),
		mSoftBodyClothTetVertFilterPairs(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mClothClothAttachments(heapMemoryManager),
		mClothClothVertTriFilterPairs(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mNewFEMClothPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mFEMClothPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mFEMClothElementIndexPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mClothRigidAttachments(heapMemoryManager),
		mClothRigidFilterPairs(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mFrozenPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mUnfrozenPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mActivatePool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mDeactivatePool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mHasBeenSimulated(false),
		mGpuWranglerManager(static_cast<PxgCudaKernelWranglerManager*>(gpuWranglerManager)),
		mCudaContextManager(cudaContextManager),
		mHeapMemoryManager(heapMemoryManager),
		mBroadPhase(NULL),
		mMaxLinks(0),
		mMaxDofs(0),
		mMaxMimicJoints(0),
		mMaxSpatialTendons(0),
		mMaxAttachments(0),
		mMaxFixedTendons(0),
		mMaxTendonJoints(0),
		mMaxPathToRoots(0),
		mMaxSoftBodyContacts(maxSoftBodyContacts),
		mMaxFemClothContacts(maxFemClothContacts),
		mMaxParticleContacts(maxParticleContacts),
		mCollisionStackSizeBytes(collisionStackSizeBytes),
		mRecomputeArticulationBlockFormat(false),
		mEnableOVDReadback(false),
		mEnableOVDCollisionReadback(false)
#if PX_SUPPORT_OMNI_PVD
		,mOvdCallbacks(NULL)
#endif
	{
		if(bp->getType() == PxBroadPhaseType::eGPU)
			mBroadPhase = static_cast<PxgCudaBroadPhaseSap*>(bp);

		mSimulationCore = PX_NEW(PxgSimulationCore)(mGpuWranglerManager, mCudaContextManager, mHeapMemoryManager, mDynamicContext, useGpuBroadphase);

		mNpContext->setSimulationCore(mSimulationCore);

		mDynamicContext->mGpuSimulationCore = mSimulationCore;
		mDynamicContext->mGpuBp = mBroadPhase;
	}

	PxgSimulationController::~PxgSimulationController()
	{
		PX_DELETE(mSimulationCore);

		if (mPBDParticleSystemCore)
		{
			mPBDParticleSystemCore->~PxgPBDParticleSystemCore();
			PX_FREE(mPBDParticleSystemCore);
		}

		if (mSoftBodyCore)
		{
			mSoftBodyCore->~PxgSoftBodyCore();
			PX_FREE(mSoftBodyCore);
		}

		if (mFEMClothCore)
		{
			mFEMClothCore->~PxgFEMClothCore();
			PX_FREE(mFEMClothCore);
		}
	}

	void PxgSimulationController::addPxgShape(Sc::ShapeSimBase* shapeSimBase, const PxsShapeCore* shapeCore, PxNodeIndex nodeIndex, PxU32 index)
	{
		mSimulationCore->mPxgShapeSimManager.addPxgShape(shapeSimBase, shapeCore, nodeIndex, index);
	}

	void PxgSimulationController::setPxgShapeBodyNodeIndex(PxNodeIndex nodeIndex, PxU32 index)
	{
		mSimulationCore->mPxgShapeSimManager.setPxgShapeBodyNodeIndex(nodeIndex, index);
	}

	void PxgSimulationController::removePxgShape(PxU32 index)
	{
		mSimulationCore->mPxgShapeSimManager.removePxgShape(index);
	}

	void PxgSimulationController::addDynamic(PxsRigidBody* rigidBody, const PxNodeIndex& nodeIndex)
	{
		mBodySimManager.addBody(rigidBody, nodeIndex.index());
	}

	void PxgSimulationController::addDynamics(PxsRigidBody** rigidBody, const PxU32* nodeIndex, PxU32 nbBodies)
	{
		for (PxU32 a = 0; a < nbBodies; ++a)
			mBodySimManager.addBody(rigidBody[a], nodeIndex[a]);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////articulation

	void PxgSimulationController::addArticulation(Dy::FeatherstoneArticulation* artBase, const PxNodeIndex& nodeIndex)
	{
		Dy::FeatherstoneArticulation* articulation = static_cast<Dy::FeatherstoneArticulation*>(artBase);
		mBodySimManager.addArticulation(articulation, nodeIndex.index(), getEnableOVDReadback());
	}

	void PxgSimulationController::releaseArticulation(Dy::FeatherstoneArticulation* artBase, const PxNodeIndex& nodeIndex)
	{
		Dy::FeatherstoneArticulation* articulation = static_cast<Dy::FeatherstoneArticulation*>(artBase);
		mBodySimManager.releaseArticulation(articulation, nodeIndex.index());
	}

	void PxgSimulationController::releaseDeferredArticulationIds()
	{
		mBodySimManager.releaseDeferredArticulationIds();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////soft body

	void PxgSimulationController::createDeformableVolumeCore()
	{
		if (!mSoftBodyCore)
		{
			bool isTGS = mDynamicContext->isTGS();
			const PxU32 maxSoftBodyContacts = mMaxSoftBodyContacts;
			mSoftBodyCore = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgSoftBodyCore), "PxgSoftBodyCore"),
				PxgSoftBodyCore(mGpuWranglerManager, mCudaContextManager, mHeapMemoryManager, this,
					mDynamicContext, maxSoftBodyContacts, mCollisionStackSizeBytes, isTGS));
		}
	}

	void PxgSimulationController::addSoftBody(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& nodeIndex)
	{
		createDeformableVolumeCore();
		mBodySimManager.addSoftBody(deformableVolume, nodeIndex.index());
	}

	void PxgSimulationController::releaseSoftBody(Dy::DeformableVolume* deformableVolume)
	{
		mBodySimManager.releaseSoftBody(deformableVolume);
	}

	void PxgSimulationController::releaseDeferredSoftBodyIds()
	{
		mBodySimManager.releaseDeferredSoftBodyIds();
	}

	void PxgSimulationController::activateSoftbody(Dy::DeformableVolume* deformableVolume)
	{
		if (mBodySimManager.activateSoftbody(deformableVolume))
		{
			const PxU32 numSoftbodies = mBodySimManager.mTotalNumSoftBodies;
			mSimulationCore->setSoftBodyWakeCounter(deformableVolume->getGpuRemapId(), deformableVolume->getCore().wakeCounter, numSoftbodies);
			
			for (PxU32 i = 0; i < deformableVolume->mParticleVolumeAttachments.size(); ++i)
			{
				mParticleSoftBodyAttachments.activateAttachment(deformableVolume->mParticleVolumeAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mRigidVolumeAttachments.size(); ++i)
			{
				mRigidSoftBodyAttachments.activateAttachment(deformableVolume->mRigidVolumeAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mSurfaceVolumeAttachments.size(); ++i)
			{
				mSoftBodyClothAttachments.activateAttachment(deformableVolume->mSurfaceVolumeAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mVolumeVolumeAttachments.size(); ++i)
			{
				mSoftBodySoftBodyAttachments.activateAttachment(deformableVolume->mVolumeVolumeAttachments[i]);
			}
		}
	}

	void PxgSimulationController::deactivateSoftbody(Dy::DeformableVolume* deformableVolume)
	{
		if (mBodySimManager.deactivateSoftbody(deformableVolume))
		{
			for (PxU32 i = 0; i < deformableVolume->mParticleVolumeAttachments.size(); ++i)
			{
				mParticleSoftBodyAttachments.deactivateAttachment(deformableVolume->mParticleVolumeAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mRigidVolumeAttachments.size(); ++i)
			{
				mRigidSoftBodyAttachments.deactivateAttachment(deformableVolume->mRigidVolumeAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mSurfaceVolumeAttachments.size(); ++i)
			{
				mSoftBodyClothAttachments.deactivateAttachment(deformableVolume->mSurfaceVolumeAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mVolumeVolumeAttachments.size(); ++i)
			{
				mSoftBodySoftBodyAttachments.deactivateAttachment(deformableVolume->mVolumeVolumeAttachments[i]);
			}
		}
	}

	void PxgSimulationController::activateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume)
	{
		mBodySimManager.activateSoftbodySelfCollision(deformableVolume);
	}

	void PxgSimulationController::deactivateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume)
	{
		mBodySimManager.deactivateSoftbodySelfCollision(deformableVolume);
	}

	void PxgSimulationController::setSoftBodyWakeCounter(Dy::DeformableVolume* deformableVolume)
	{
		PxReal wakeCounter = deformableVolume->getCore().wakeCounter;
		const PxU32 numSoftBodies = mBodySimManager.mTotalNumSoftBodies;
		mSimulationCore->setSoftBodyWakeCounter(deformableVolume->getGpuRemapId(), wakeCounter, numSoftBodies);
		mBodySimManager.mActiveSoftbodiesDirty = true;

		if (wakeCounter > 0.f)
		{
			activateSoftbody(deformableVolume);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////fem cloth

	void PxgSimulationController::createDeformableSurfaceCore()
	{
		if (!mFEMClothCore)
		{
			bool isTGS = mDynamicContext->isTGS();
			const PxU32 maxFemClothContacts = mMaxFemClothContacts;
			mFEMClothCore = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgFEMClothCore), "PxgFEMClothCore"),
				PxgFEMClothCore(mGpuWranglerManager, mCudaContextManager, mHeapMemoryManager, this,
					mDynamicContext, maxFemClothContacts, mCollisionStackSizeBytes, isTGS));
		}
	}

	void PxgSimulationController::addFEMCloth(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& nodeIndex)
	{
		createDeformableSurfaceCore();
		mBodySimManager.addFEMCloth(deformableSurface, nodeIndex.index());
	}

	void PxgSimulationController::releaseFEMCloth(Dy::DeformableSurface* deformableSurface)
	{
		mBodySimManager.releaseFEMCloth(deformableSurface);
	}

	void PxgSimulationController::releaseDeferredFEMClothIds()
	{
		mBodySimManager.releaseDeferredFEMClothIds();
	}

	void PxgSimulationController::activateCloth(Dy::DeformableSurface* deformableSurface)
	{
		if (mBodySimManager.activateCloth(deformableSurface))
		{
			const PxU32 numClothes = mBodySimManager.mTotalNumFEMCloths;
			mSimulationCore->setFEMClothWakeCounter(deformableSurface->getGpuRemapId(), deformableSurface->getCore().wakeCounter, numClothes);
			for (PxU32 i = 0; i < deformableSurface->mAttachmentHandles.size(); ++i)
			{
				mClothRigidAttachments.activateAttachment(deformableSurface->mAttachmentHandles[i]);
			}

			for (PxU32 i = 0; i < deformableSurface->mSurfaceSurfaceAttachments.size(); ++i)
			{
				mClothClothAttachments.activateAttachment(deformableSurface->mSurfaceSurfaceAttachments[i]);
			}
		}
	}

	void PxgSimulationController::deactivateCloth(Dy::DeformableSurface* deformableSurface)
	{
		if (mBodySimManager.deactivateCloth(deformableSurface))
		{
			for (PxU32 i = 0; i < deformableSurface->mAttachmentHandles.size(); ++i)
			{
				mClothRigidAttachments.deactivateAttachment(deformableSurface->mAttachmentHandles[i]);
			}

			for (PxU32 i = 0; i < deformableSurface->mSurfaceSurfaceAttachments.size(); ++i)
			{
				mClothClothAttachments.deactivateAttachment(deformableSurface->mSurfaceSurfaceAttachments[i]);
			}
		}
	}

	void PxgSimulationController::setClothWakeCounter(Dy::DeformableSurface* deformableSurface)
	{
		PxReal wakeCounter = deformableSurface->getCore().wakeCounter;
		const PxU32 numClothes = mBodySimManager.mTotalNumFEMCloths;
		mSimulationCore->setFEMClothWakeCounter(deformableSurface->getGpuRemapId(), wakeCounter, numClothes);
		mBodySimManager.mActiveFEMClothsDirty = true;

		if (wakeCounter > 0.f)
		{
			activateCloth(deformableSurface);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////particle system

	void PxgSimulationController::addParticleSystem(Dy::ParticleSystem* particleSystem, const PxNodeIndex& nodeIndex)
	{
		if (!mPBDParticleSystemCore)
		{
			mPBDParticleSystemCore = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgPBDParticleSystemCore), "PxgPBDParticleSystemCore"),
				PxgPBDParticleSystemCore(mGpuWranglerManager, mCudaContextManager, mHeapMemoryManager, this,
					mDynamicContext, mMaxParticleContacts));
		}

		mBodySimManager.addPBDParticleSystem(particleSystem, nodeIndex.index());
	}

	void PxgSimulationController::releaseParticleSystem(Dy::ParticleSystem* particleSystem)
	{
		mBodySimManager.releasePBDParticleSystem(particleSystem);
	}

	void PxgSimulationController::releaseDeferredParticleSystemIds()
	{
		mBodySimManager.releaseDeferredPBDParticleSystemIds();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////soft body

	void PxgSimulationController::updateDynamic(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		if (articulation)
		{
			Dy::FeatherstoneArticulation* featherstoneArtic = static_cast<Dy::FeatherstoneArticulation*>(articulation);
			featherstoneArtic->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_LINKS);
			mBodySimManager.updateArticulation(featherstoneArtic, nodeIndex.index());
		}
		else
			mBodySimManager.updateBody(nodeIndex);
	}

	void PxgSimulationController::addJoint(const Dy::Constraint& constraint)
	{
		mJointManager.registerJoint(constraint);
	}

	void PxgSimulationController::updateJoint(const PxU32 edgeIndex, Dy::Constraint* constraint)
	{
		mJointManager.updateJoint(edgeIndex, constraint);
	}

	void PxgSimulationController::updateArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		Dy::FeatherstoneArticulation* featherstoneArtic = static_cast<Dy::FeatherstoneArticulation*>(articulation);
		mBodySimManager.updateArticulation(featherstoneArtic, nodeIndex.index());
	}

	void PxgSimulationController::updateArticulationJoint(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		Dy::FeatherstoneArticulation* featherstoneArtic = static_cast<Dy::FeatherstoneArticulation*>(articulation);
		featherstoneArtic->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_JOINTS);
		mBodySimManager.updateArticulation(featherstoneArtic, nodeIndex.index());
	}

	void PxgSimulationController::updateArticulationExtAccel(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		Dy::FeatherstoneArticulation* featherstoneArtic = static_cast<Dy::FeatherstoneArticulation*>(articulation);
		featherstoneArtic->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL);
		mBodySimManager.updateArticulation(featherstoneArtic, nodeIndex.index());
	}

	void PxgSimulationController::updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies, 
		PxsExternalAccelerationProvider* externalAccelerations)
	{
		mBodySimManager.updateBodies(rigidBodies, nodeIndices, nbBodies, externalAccelerations);
	}

	void PxgSimulationController::reserve(const PxU32 nbBodies)
	{
		PxgSolverCore* gpuSolverCore = mDynamicContext->getGpuSolverCore();
		gpuSolverCore->acquireContext();
		mBodySimManager.reserve(nbBodies);
		gpuSolverCore->releaseContext();
	}

	void PxgSimulationController::setDeformableVolumeGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback)
	{
		createDeformableVolumeCore();
		mDynamicContext->mGpuSoftBodyCore->mPostSolveCallback = postSolveCallback;
	}
	
	void PxgSimulationController::setDeformableSurfaceGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback)
	{
		createDeformableSurfaceCore();
		mDynamicContext->mGpuFEMClothCore->mPostSolveCallback = postSolveCallback;		
	}

	void PxgSimulationController::copySoftBodyDataDEPRECATED(void** data, void* dataEndIndices, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, CUevent copyEvent)
	{
		PxgSolverCore* gpuSolverCore = mDynamicContext->getGpuSolverCore();
		PxgSoftBodyCore* softBodyCore = mDynamicContext->mGpuSoftBodyCore;

		gpuSolverCore->acquireContext();
		softBodyCore->copySoftBodyDataDEPRECATED(data, dataEndIndices, softBodyIndices, flag, nbCopySoftBodies, maxSize, copyEvent);
		gpuSolverCore->releaseContext();
	}

	void PxgSimulationController::applySoftBodyDataDEPRECATED(void** data, void* dataEndIndices, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, CUevent applyEvent, CUevent signalEvent)
	{
		PxgSolverCore* gpuSolverCore = mDynamicContext->getGpuSolverCore();
		PxgSoftBodyCore* softBodyCore = mDynamicContext->mGpuSoftBodyCore;

		gpuSolverCore->acquireContext();
		softBodyCore->applySoftBodyDataDEPRECATED(data, dataEndIndices, softBodyIndices, flag, nbUpdatedSoftBodies, maxSize, applyEvent, signalEvent);
		gpuSolverCore->releaseContext();
	}

	bool PxgSimulationController::getRigidDynamicData(void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, float oneOverDt, CUevent startEvent, CUevent finishEvent) const
	{
		return mSimulationCore->getRigidDynamicData(data, gpuIndices, dataType, nbElements, oneOverDt, startEvent, finishEvent);
	}

	bool PxgSimulationController::setRigidDynamicData(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent)
	{
		return mSimulationCore->setRigidDynamicData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent);
	}

	bool PxgSimulationController::getArticulationData(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const
	{
		const PxgArticulationCore* artiCore = mDynamicContext->getArticulationCore();

		return artiCore->getArticulationData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent, mMaxLinks, mMaxDofs, mMaxFixedTendons, mMaxTendonJoints, mMaxSpatialTendons, mMaxAttachments);
	}

	bool PxgSimulationController::setArticulationData(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent)
	{
		PxgArticulationCore* artiCore = mDynamicContext->getArticulationCore();

		return artiCore->setArticulationData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent, mMaxLinks, mMaxDofs, mMaxFixedTendons, mMaxTendonJoints, mMaxSpatialTendons, mMaxAttachments);
	}

	// PT: TODO: revisit this? It is unfortunate that the generic API now takes e.g. "gravity" as a parameter
	bool PxgSimulationController::computeArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements, CUevent startEvent, CUevent finishEvent)
	{
		PxgArticulationCore* artiCore = mDynamicContext->getArticulationCore();

		return artiCore->computeArticulationData(data, gpuIndices, operation, nbElements, mMaxLinks, mMaxDofs, startEvent, finishEvent);
	}

	bool PxgSimulationController::copyContactData(void* PX_RESTRICT data, PxU32* PX_RESTRICT numContactPairs, const PxU32 maxContactPairs, CUevent startEvent, CUevent copyEvent)
	{
		PxgGpuNarrowphaseCore* npCore = mNpContext->getGpuNarrowphaseCore();

		const PxU32 ind = 1 - mDynamicContext->getCurrentContactStreamIndex();

		PxU8* patchStream = mDynamicContext->getPatchStream(ind);
		PxU8* contactStream = mDynamicContext->getContactStream(ind);

		return npCore->copyContactData(data, numContactPairs, maxContactPairs, startEvent, copyEvent,
			patchStream,
			contactStream,
			mNpContext->getContext().mForceAndIndiceStreamPool->mDataStream);
	}

	bool PxgSimulationController::evaluateSDFDistances(PxVec4* PX_RESTRICT localGradientAndSDFConcatenated, const PxShapeGPUIndex* PX_RESTRICT shapeIndices, const PxVec4* PX_RESTRICT localSamplePointsConcatenated, const PxU32* PX_RESTRICT samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent, CUevent finishEvent)
	{
		PxgGpuNarrowphaseCore* npCore = mNpContext->getGpuNarrowphaseCore();
		// need to ack context in there!
		return npCore->evaluateSDFDistances(localGradientAndSDFConcatenated, shapeIndices, localSamplePointsConcatenated, samplePointCountPerShape, nbElements, maxPointCount, startEvent, finishEvent);
	}

	PxArticulationGPUAPIMaxCounts PxgSimulationController::getArticulationGPUAPIMaxCounts() const
	{
		PxArticulationGPUAPIMaxCounts maxCounts;
		maxCounts.maxDofs = mMaxDofs;
		maxCounts.maxLinks = mMaxLinks;
		maxCounts.maxFixedTendons = mMaxFixedTendons;
		maxCounts.maxFixedTendonJoints = mMaxTendonJoints;
		maxCounts.maxSpatialTendons = mMaxSpatialTendons;
		maxCounts.maxSpatialTendonAttachments = mMaxAttachments;

		return maxCounts;
	}

	bool PxgSimulationController::getD6JointData(void* data, const PxD6JointGPUIndex* gpuIndices, PxD6JointGPUAPIReadType::Enum dataType, PxU32 nbElements, PxF32 oneOverDt, CUevent startEvent, CUevent finishEvent) const
	{
		return mSimulationCore->getD6JointData(data, gpuIndices, dataType, nbElements, oneOverDt, mJointManager.getGpuConstraintIdMapHost().size(),
			startEvent, finishEvent);
	}

	PxU32 PxgSimulationController::getInternalShapeIndex(const PxsShapeCore& shapeCore)
	{
		PxgGpuNarrowphaseCore* npCore = mNpContext->getGpuNarrowphaseCore();
		return npCore->getShapeIndex(shapeCore);
	}

	void PxgSimulationController::syncParticleData()
	{
		PxScopedCudaLock _lock(*mCudaContextManager);

		PxU32 contactCountNeeded = 0;

		if (mPBDParticleSystemCore)
		{
			CUresult syncResult = mPBDParticleSystemCore->mCudaContext->streamSynchronize(mPBDParticleSystemCore->getFinalizeStream());
			
			if (syncResult != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "PhysX Internal CUDA error. Simulation can not continue! Error code %i!\n", PxI32(syncResult));

			contactCountNeeded = mPBDParticleSystemCore->getHostContactCount();
		}

		if (contactCountNeeded > mMaxParticleContacts)
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "Particle system contact buffer overflow detected, please increase PxGpuDynamicsMemoryConfig::maxParticleContacts to at least %u\n", contactCountNeeded);
		}

		// add to sim stats
		mDynamicContext->getSimStats().mGpuDynamicsParticleContacts = PxMax(mDynamicContext->getSimStats().mGpuDynamicsParticleContacts, contactCountNeeded);
	}
	
	void PxgSimulationController::applyParticleBufferDataDEPRECATED(const PxU32* indices, const PxGpuParticleBufferIndexPair* indexPairs, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, CUevent waitEvent, CUevent signalEvent)
	{
		PxScopedCudaLock lock(*mPBDParticleSystemCore->mCudaContextManager);
		mPBDParticleSystemCore->applyParticleBufferDataDEPRECATED(indices, indexPairs, flags, nbUpdatedBuffers, waitEvent, signalEvent);
	}

	void PxgSimulationController::updateBoundsAndTransformCache(Bp::AABBManagerBase& aabbManager, CUstream npStream,
																PxsTransformCache& transformCache, PxgCudaBuffer& gpuTransformCache)
	{
		Bp::BoundsArray& boundsArray = aabbManager.getBoundsArray();
		const PxU32 numBounds = boundsArray.size();
		const PxU32 boundsArraySize = numBounds * sizeof(PxBounds3);
		PxU32 totalTransformCacheSize = transformCache.getTotalSize();

		if(mDynamicContext->getEnableDirectGPUAPI())
		{
			PxgBoundsArray& directGPUBoundsArray = static_cast<PxgBoundsArray&>(boundsArray);
			const PxU32 numChanges = directGPUBoundsArray.getNumberOfChanges();

			if(directGPUBoundsArray.isFirstCopy())
			{
				copyBoundsAndTransforms(boundsArray, transformCache, gpuTransformCache, boundsArraySize, totalTransformCacheSize, npStream);
				directGPUBoundsArray.setCopied();			}

			else if(numChanges)
			{
				mergeBoundsAndTransformsChanges(directGPUBoundsArray, transformCache, gpuTransformCache, boundsArraySize,
												totalTransformCacheSize, numChanges, npStream);
			}
		}
		else
		{
			copyBoundsAndTransforms(boundsArray, transformCache, gpuTransformCache, boundsArraySize, totalTransformCacheSize,
											   npStream);
		}
	}

	void PxgSimulationController::mergeBoundsAndTransformsChanges(PxgBoundsArray& directGPUBoundsArray,
																  PxsTransformCache& transformCache,
																  PxgCudaBuffer& gpuTransformCache, PxU32 boundsArraySize,
																  PxU32 totalTransformCacheSize, PxU32 numChanges, CUstream npStream)
	{

		PxBoundTransformUpdate* changes = reinterpret_cast<PxBoundTransformUpdate*>(
			getMappedDevicePtr(mCudaContextManager->getCudaContext(), directGPUBoundsArray.getStagingBuffer().begin()));


		PxBounds3* cpuBounds =
			reinterpret_cast<PxBounds3*>(getMappedDevicePtr(mCudaContextManager->getCudaContext(), directGPUBoundsArray.getBounds().begin()));

		PxsCachedTransform* cpuTransforms = reinterpret_cast<PxsCachedTransform*>(getMappedDevicePtr(mCudaContextManager->getCudaContext(), transformCache.getTransforms()));
		
		
		PxgCudaBuffer* aabbBounds = mSimulationCore->getBoundArrayBuffer();
		aabbBounds->allocateCopyOldDataAsync(boundsArraySize, mCudaContextManager->getCudaContext(), npStream, PX_FL);
		CUdeviceptr boundsPtr = aabbBounds->getDevicePtr();


		gpuTransformCache.allocateCopyOldDataAsync(sizeof(PxsCachedTransform) * totalTransformCacheSize, mCudaContextManager->getCudaContext(), npStream, PX_FL);
		CUdeviceptr transformCachePtr = gpuTransformCache.getDevicePtr();

		CUfunction kernelFunction = mSimulationCore->mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
			PxgKernelIds::MERGE_TRANSFORMCACHE_AND_BOUNDARRAY_CHANGES);

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(boundsPtr),	 PX_CUDA_KERNEL_PARAM(transformCachePtr),
											 PX_CUDA_KERNEL_PARAM(cpuBounds), PX_CUDA_KERNEL_PARAM(cpuTransforms),
											 PX_CUDA_KERNEL_PARAM(changes),	 PX_CUDA_KERNEL_PARAM(numChanges) };

		int gridDim = (numChanges + PxgSimulationCoreKernelBlockDim::MERGE_TRANSFORMCACHE_AND_BOUNDARRAY_CHANGES - 1) /
					  PxgSimulationCoreKernelBlockDim::MERGE_TRANSFORMCACHE_AND_BOUNDARRAY_CHANGES;
		CUresult result = mCudaContextManager->getCudaContext()->launchKernel(
			kernelFunction, gridDim, 1, 1, PxgSimulationCoreKernelBlockDim::MERGE_TRANSFORMCACHE_AND_BOUNDARRAY_CHANGES, 1, 1, 0, npStream,
			kernelParams, sizeof(kernelParams), 0, PX_FL);

		PX_UNUSED(result);
		PX_ASSERT(result == CUDA_SUCCESS);
	}

	void PxgSimulationController::copyBoundsAndTransforms(Bp::BoundsArray& boundsArray, PxsTransformCache& transformCache,
															   PxgCudaBuffer& gpuTransformCache, PxU32 boundsArraySize,
															   PxU32 totalTransformCacheSize, CUstream npStream)
	{
		if(boundsArray.hasChanged())
		{
			boundsArray.resetChangedState();
			mSimulationCore->getBoundArrayBuffer()->allocate(boundsArraySize, PX_FL);
			mCudaContextManager->getCudaContext()->memcpyHtoDAsync(mSimulationCore->getBoundArrayBuffer()->getDevicePtr(),
																   boundsArray.getBounds().begin(), boundsArraySize, npStream);
		}
		if(transformCache.hasChanged())
		{
			gpuTransformCache.allocate(totalTransformCacheSize * sizeof(PxsCachedTransform), PX_FL);
			if(totalTransformCacheSize)
			{
				mCudaContextManager->getCudaContext()->memcpyHtoDAsync(gpuTransformCache.getDevicePtr(),
																	   reinterpret_cast<void*>(transformCache.getTransforms()),
																	   sizeof(PxsCachedTransform) * totalTransformCacheSize, npStream);
			}
		}
	}

	void PxgSimulationController::updateBoundsAndShapes(Bp::AABBManagerBase& aabbManager, bool isDirectApiInitialized)
	{
		PxScopedCudaLock _lock(*mCudaContextManager);

		PxgGpuNarrowphaseCore* npCore = mDynamicContext->getNarrowphaseCore();
		CUstream npStream = npCore->getStream();
		const bool hasShapeInstanceChanged = npCore->mGpuShapesManager.mHasShapeInstanceChanged; // we reset it in updateNarrowPhaseShape, but cache for computeRigidsToShapes.

		// we are in Pxg-land, so GPU NP and dynamics is implied. Upload to GPU if dirty.
		// The bounds array is shared by NP and BP, and at this point we are before NP and BP.
		if (!isDirectApiInitialized || hasShapeInstanceChanged)
			updateBoundsAndTransformCache(aabbManager, npStream, mNpContext->getContext().getTransformCache(), npCore->getTransformCache());
			
		mNpContext->updateNarrowPhaseShape();

		// AD TODO: remove this if again once we have the warm-start implemented, or find a way to avoid doing this alltogether.
		// this needs to run even if direct-GPU API is not initialized, because it is part of the initialization.
		if (mDynamicContext->mEnableDirectGPUAPI && hasShapeInstanceChanged)
		{
			//run in np stream
			npCore->computeRigidsToShapes();
		}

		if (isDirectApiInitialized)
		{
			// we have a data-dependency between computeRigidsToShapes and updateArticulationsKinematic - so we need to make sure the articulation stream does not start too early.
			PxgArticulationCore* artiCore = mDynamicContext->getArticulationCore();
			npCore->synchronizedStreams(artiCore->getStream());

			// will run on articulation stream.
			// do not zero the sim state here as it will be overwritten afterwards anyway.
			artiCore->updateArticulationsKinematic(false);

			//broad phase and narrowphase need to wait for kinematic update to finish.
			PxgCudaBroadPhaseSap* bp = static_cast<PxgCudaBroadPhaseSap*>(aabbManager.getBroadPhase());
			CUstream bpStream = bp->getBpStream();

			// this will record an event on the articulation stream and make sure the BP and NP stream wait until updateArticulationsKinematic is done.
			artiCore->synchronizedStreams(bpStream, npStream);
		}

		// schedule particle material uploads on BP stream, because they are already needed by 
		// pre integration. We can probably remove that dependency in the future
		// when moving gravityScale and damping to the actor.
		npCore->uploadDataChunksToGpuBp();
	}

	void PxgSimulationController::copyToGpuBodySim(PxBaseTask* continuation)
	{
		PX_PROFILE_ZONE("GpuSimulationController.copyToGpuBodies", 0);

		// AD: this is used downstream by the solver controllers to force recomputing
		// the block format for articulations. Could still be true from the last frame.
		mRecomputeArticulationBlockFormat = false;

		const PxU32 nbNewArticulations = mBodySimManager.mNewArticulationSims.size();
		const PxU32 nbNewBodies = mBodySimManager.mNewOrUpdatedBodySims.size();

		const PxU32 totalNumNewBodies = nbNewBodies + nbNewArticulations;

		mNewBodySimPool.reserve(totalNumNewBodies);
		mNewBodySimPool.forceSize_Unsafe(totalNumNewBodies);

		PxU32 nbNewLinks = 0;
		PxU32 nbUpdatedDofs = 0;
		PxU32 nbNewSpatialTendons = 0;
		PxU32 nbNewAttachments = 0;

		PxU32 nbNewFixedTendons = 0;
		PxU32 nbNewTendonJoints = 0;
		PxU32 nbNewMimicJoints = 0;

		PxU32 nbNewPathToRoots = 0;

		PxU32 maxLinks = mMaxLinks;
		PxU32 maxDofs = mMaxDofs;
		PxU32 maxMimicJoints = mMaxMimicJoints;
		PxU32 maxSpatialTendons = mMaxSpatialTendons;
		PxU32 maxAttachments = mMaxAttachments;
		PxU32 maxFixedTendons = mMaxFixedTendons;
		PxU32 maxTendonJoints = mMaxTendonJoints;
		PxU32 maxPathToRoots = mMaxPathToRoots;

		mArticulationPool.forceSize_Unsafe(0);
		mArticulationPool.reserve(nbNewArticulations);
		mArticulationPool.forceSize_Unsafe(nbNewArticulations);

		void** bodies = mBodySimManager.mBodies.begin();

		for (PxU32 i = 0; i < nbNewArticulations; ++i)
		{
			const PxgArticulationIndices& index = mBodySimManager.mNewArticulationSims[i];

			Dy::FeatherstoneArticulation* articulation = reinterpret_cast<Dy::FeatherstoneArticulation*>(bodies[index.nodeIndex]);

			Dy::ArticulationData& data = articulation->getArticulationData();

			const PxU32 nbLinks = data.getLinkCount();
			const PxU32 nbDofs = data.getDofs();
			const PxU32 nbSpatialTendons = data.getSpatialTendonCount();
			const PxU32 nbFixedTendons = data.getFixedTendonCount();
			const PxU32 nbMimicJoints = data.getMimicJointCount();
			nbNewLinks += nbLinks;
			nbUpdatedDofs += nbDofs * 5;
			nbNewSpatialTendons += nbSpatialTendons;
			nbNewFixedTendons += nbFixedTendons;
			nbNewMimicJoints += nbMimicJoints;

			const PxU32 nbPathToRootElems = data.getPathToRootElementCount();
			nbNewPathToRoots += nbPathToRootElems;

			Dy::ArticulationSpatialTendon** tendons = data.getSpatialTendons();

			for (PxU32 j = 0; j < nbSpatialTendons; ++j)
			{
				Dy::ArticulationSpatialTendon* tendon = tendons[j];
				const PxU32 nbAttachements = tendon->getNumAttachments();
				nbNewAttachments += nbAttachements;

				maxAttachments = PxMax(maxAttachments, nbAttachements);
			}

			Dy::ArticulationFixedTendon** fixedTendons = data.getFixedTendons();

			for (PxU32 j = 0; j < nbFixedTendons; ++j)
			{
				Dy::ArticulationFixedTendon* tendon = fixedTendons[j];
				const PxU32 nbTendonJoints = tendon->getNumJoints();
				nbNewTendonJoints += nbTendonJoints;

				maxTendonJoints = PxMax(maxTendonJoints, nbTendonJoints);
			}

			maxLinks = PxMax(maxLinks, nbLinks);
			maxDofs = PxMax(maxDofs, nbDofs);

			maxSpatialTendons = PxMax(maxSpatialTendons, nbSpatialTendons);
			maxFixedTendons = PxMax(maxFixedTendons, nbFixedTendons);
			maxMimicJoints = PxMax(maxMimicJoints, nbMimicJoints);
			maxPathToRoots = PxMax(maxPathToRoots, nbNewPathToRoots);

			// if we add an articulation, we need to redo the whole block format layout.
			// In that case we need to copy everything again because the resize of the entire 
			// block-format buffer will potentially change the location in memory.
			mRecomputeArticulationBlockFormat = true;
		}

		// At this point we know whether the max counts changed because the new articulations are
		// processed.
		mMaxLinks = maxLinks;
		mMaxDofs = maxDofs;
		mMaxMimicJoints = maxMimicJoints;
		mMaxSpatialTendons = maxSpatialTendons;
		mMaxAttachments =  maxAttachments;
		mMaxFixedTendons = maxFixedTendons;
		mMaxTendonJoints = maxTendonJoints;
		mMaxPathToRoots = maxPathToRoots;

		const PxU32 nbUpdatedArticulations = mBodySimManager.mUpdatedArticulations.size();
		for (PxU32 i = 0; i < nbUpdatedArticulations; ++i)
		{
			Dy::FeatherstoneArticulation* articulation = mBodySimManager.mUpdatedArticulations[i].articulation;

			Dy::ArticulationData& data = articulation->getArticulationData();
			const PxU32 nbLinks = data.getLinkCount();
			const PxU32 nbDofs = data.getDofs();
			const PxU32 nbSpatialTendons = data.getSpatialTendonCount();
			const PxU32 nbFixedTendons = data.getFixedTendonCount();
			const PxU32 nbMimicJoints = data.getMimicJointCount();

			nbNewLinks += nbLinks;
			//We need space for 5x sets of dofs - force, velocity and position, targetpos, target vel
			nbUpdatedDofs += nbDofs * 5;
			nbNewSpatialTendons += nbSpatialTendons;
			nbNewFixedTendons += nbFixedTendons;
			nbNewMimicJoints += nbMimicJoints;
			
			Dy::ArticulationSpatialTendon** tendons = data.getSpatialTendons();

			for (PxU32 j = 0; j < nbSpatialTendons; ++j)
			{
				Dy::ArticulationSpatialTendon* tendon = tendons[j];
				const PxU32 nbAttachements = tendon->getNumAttachments();
				nbNewAttachments += nbAttachements;
			}

			Dy::ArticulationFixedTendon** fixedTendons = data.getFixedTendons();

			for (PxU32 j = 0; j < nbFixedTendons; ++j)
			{
				Dy::ArticulationFixedTendon* tendon = fixedTendons[j];
				const PxU32 nbTendonJoints = tendon->getNumJoints();
				nbNewTendonJoints += nbTendonJoints;
			}
		}

		mLinksPool.forceSize_Unsafe(0);
		mLinksPool.reserve(nbNewLinks);
		mLinksPool.forceSize_Unsafe(nbNewLinks);

		mLinkWakeCounterPool.forceSize_Unsafe(0);
		mLinkWakeCounterPool.reserve(nbNewLinks);
		mLinkWakeCounterPool.forceSize_Unsafe(nbNewLinks);

		mLinkAccelPool.forceSize_Unsafe(0);
		mLinkAccelPool.reserve(nbNewLinks);
		mLinkAccelPool.forceSize_Unsafe(nbNewLinks);

		mLinkPropPool.forceSize_Unsafe(0);
		mLinkPropPool.reserve(nbNewLinks);
		mLinkPropPool.forceSize_Unsafe(nbNewLinks);

		mLinkParentPool.forceSize_Unsafe(0);
		mLinkParentPool.reserve(nbNewLinks);
		mLinkParentPool.forceSize_Unsafe(nbNewLinks);

		mLinkChildPool.forceSize_Unsafe(0);
		mLinkChildPool.reserve(nbNewLinks);
		mLinkChildPool.forceSize_Unsafe(nbNewLinks);

		mLinkBody2WorldPool.forceSize_Unsafe(0);
		mLinkBody2WorldPool.reserve(nbNewLinks);
		mLinkBody2WorldPool.forceSize_Unsafe(nbNewLinks);

		mLinkBody2ActorPool.forceSize_Unsafe(0);
		mLinkBody2ActorPool.reserve(nbNewLinks);
		mLinkBody2ActorPool.forceSize_Unsafe(nbNewLinks);

		mPathToRootPool.forceSize_Unsafe(0);
		mPathToRootPool.reserve(nbNewPathToRoots);
		mPathToRootPool.forceSize_Unsafe(nbNewPathToRoots);

		mJointPool.forceSize_Unsafe(0);
		mJointPool.reserve(nbNewLinks);
		mJointPool.forceSize_Unsafe(nbNewLinks);

		mJointDataPool.forceSize_Unsafe(0);
		mJointDataPool.reserve(nbNewLinks);
		mJointDataPool.forceSize_Unsafe(nbNewLinks);

		mLinkJointIndexPool.forceSize_Unsafe(0);
		mLinkJointIndexPool.reserve(nbNewArticulations);
		mLinkJointIndexPool.forceSize_Unsafe(nbNewArticulations);

		mArticulationUpdatePool.forceSize_Unsafe(0);
		mArticulationUpdatePool.reserve(nbUpdatedArticulations);
		mArticulationUpdatePool.forceSize_Unsafe(nbUpdatedArticulations);

		mArticulationDofDataPool.forceSize_Unsafe(0);
		mArticulationDofDataPool.reserve(nbUpdatedDofs);
		mArticulationDofDataPool.forceSize_Unsafe(nbUpdatedDofs);

		mSpatialTendonParamPool.forceSize_Unsafe(0);
		mSpatialTendonParamPool.reserve(nbNewSpatialTendons);
		mSpatialTendonParamPool.forceSize_Unsafe(nbNewSpatialTendons);

		mSpatialTendonPool.forceSize_Unsafe(0);
		mSpatialTendonPool.reserve(nbNewSpatialTendons);
		mSpatialTendonPool.forceSize_Unsafe(nbNewSpatialTendons);

		mTendonAttachmentMapPool.forceSize_Unsafe(0);
		mTendonAttachmentMapPool.reserve(nbNewSpatialTendons);
		mTendonAttachmentMapPool.forceSize_Unsafe(nbNewSpatialTendons);

		mAttachmentFixedPool.forceSize_Unsafe(0);
		mAttachmentFixedPool.reserve(nbNewAttachments);
		mAttachmentFixedPool.forceSize_Unsafe(nbNewAttachments);

		mAttachmentModPool.forceSize_Unsafe(0);
		mAttachmentModPool.reserve(nbNewAttachments);
		mAttachmentModPool.forceSize_Unsafe(nbNewAttachments);

		mFixedTendonParamPool.forceSize_Unsafe(0);
		mFixedTendonParamPool.reserve(nbNewFixedTendons);
		mFixedTendonParamPool.forceSize_Unsafe(nbNewFixedTendons);

		mFixedTendonPool.forceSize_Unsafe(0);
		mFixedTendonPool.reserve(nbNewFixedTendons);
		mFixedTendonPool.forceSize_Unsafe(nbNewFixedTendons);

		mTendonTendonJointMapPool.forceSize_Unsafe(0);
		mTendonTendonJointMapPool.reserve(nbNewFixedTendons);
		mTendonTendonJointMapPool.forceSize_Unsafe(nbNewFixedTendons);

		mMimicJointPool.forceSize_Unsafe(0);
		mMimicJointPool.reserve(nbNewMimicJoints);
		mMimicJointPool.forceSize_Unsafe(nbNewMimicJoints);

		mTendonJointFixedDataPool.forceSize_Unsafe(0);
		mTendonJointFixedDataPool.reserve(nbNewTendonJoints);
		mTendonJointFixedDataPool.forceSize_Unsafe(nbNewTendonJoints);

		mTendonJointCoefficientDataPool.forceSize_Unsafe(0);
		mTendonJointCoefficientDataPool.reserve(nbNewTendonJoints);
		mTendonJointCoefficientDataPool.forceSize_Unsafe(nbNewTendonJoints);

		const PxU32 maxElementsPerTask = 1024;

		Cm::FlushPool& flushPool = mDynamicContext->getFlushPool();

		PxU32 bodySimOffset = 0;

		for (PxU32 i = 0; i < nbNewBodies; i += maxElementsPerTask)
		{
			PxgCopyToBodySimTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgCopyToBodySimTask)), PxgCopyToBodySimTask)(*this,
					bodySimOffset, i, PxMin(maxElementsPerTask, nbNewBodies - i));

			task->setContinuation(continuation);
			task->removeReference();
		}

		const PxU32 maxArticulationPerTask = 128;

		mSharedLinkIndex = 0;
		mSharedDofIndex = 0;
		mSharedSpatialTendonIndex = 0;
		mSharedSpatialAttachmentIndex = 0;
		mSharedFixedTendonIndex = 0;
		mSharedFixedTendonJointIndex = 0;
		mSharedMimicJointIndex = 0;
		mSharedPathToRootIndex = 0;

		bodySimOffset += nbNewBodies;
		for (PxU32 i = 0; i < nbNewArticulations; i += maxArticulationPerTask)
		{
			PxgCopyToArticulationSimTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgCopyToArticulationSimTask)), PxgCopyToArticulationSimTask)(*this,
					bodySimOffset, i, PxMin(maxArticulationPerTask, nbNewArticulations - i), &mSharedLinkIndex, &mSharedDofIndex,
					&mSharedSpatialTendonIndex, &mSharedSpatialAttachmentIndex, &mSharedFixedTendonIndex, &mSharedFixedTendonJointIndex,
					&mSharedMimicJointIndex, &mSharedPathToRootIndex);

			task->setContinuation(continuation);
			task->removeReference();
		}

		mBodySimManager.mUpdatedMap.clear();

		for (PxU32 i = 0; i < nbUpdatedArticulations; i += maxArticulationPerTask)
		{
			PxgUpdateArticulationSimTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgUpdateArticulationSimTask)), PxgUpdateArticulationSimTask)(*this,
					i, PxMin(maxArticulationPerTask, nbUpdatedArticulations - i), &mSharedLinkIndex, &mSharedDofIndex, &mSharedSpatialTendonIndex,
					&mSharedSpatialAttachmentIndex, &mSharedFixedTendonIndex, &mSharedFixedTendonJointIndex, &mSharedMimicJointIndex);

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	void PxgSimulationController::copyToGpuParticleSystem(PxBaseTask* continuation)
	{
		PX_PROFILE_ZONE("GpuSimulationController.copyToGpuParticleSystem", 0);

		const PxU32 nbNewPBDParticleSystems = mBodySimManager.mNewPBDParticleSystemSims.size();

		if (mPBDParticleSystemCore)
		{
			//we have to clear new data every frame.
			mPBDParticleSystemCore->mNewParticleSystemPool.forceSize_Unsafe(0);
			mPBDParticleSystemCore->mNewParticleSystemPool.reserve(nbNewPBDParticleSystems);
			mPBDParticleSystemCore->mNewParticleSystemPool.forceSize_Unsafe(nbNewPBDParticleSystems);

			mPBDParticleSystemCore->mNewParticleSystemNodeIndexPool.forceSize_Unsafe(0);
			mPBDParticleSystemCore->mNewParticleSystemNodeIndexPool.reserve(nbNewPBDParticleSystems);
			mPBDParticleSystemCore->mNewParticleSystemNodeIndexPool.forceSize_Unsafe(nbNewPBDParticleSystems);
		}

		Cm::FlushPool& flushPool = mDynamicContext->getFlushPool();

		const PxU32 maxParticleSystemPerTask = 50;

		for (PxU32 i = 0; i < nbNewPBDParticleSystems; i += maxParticleSystemPerTask)
		{
			PxgCopyToPBDParticleSystemSimTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgCopyToPBDParticleSystemSimTask)), PxgCopyToPBDParticleSystemSimTask)(*this,
					i, PxMin(maxParticleSystemPerTask, nbNewPBDParticleSystems - i));

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	void PxgSimulationController::copyToGpuSoftBody(PxBaseTask* continuation)
	{
		PX_PROFILE_ZONE("GpuSimulationController.copyToGpuSoftBody", 0);

		const PxU32 nbNewSoftBodies = mBodySimManager.mNewSoftBodySims.size();

		mNewSoftBodyPool.forceSize_Unsafe(0);
		mNewSoftBodyPool.reserve(nbNewSoftBodies);
		mNewSoftBodyPool.forceSize_Unsafe(nbNewSoftBodies);

		mNewSoftBodyDataPool.forceSize_Unsafe(0);
		mNewSoftBodyDataPool.reserve(nbNewSoftBodies);
		mNewSoftBodyDataPool.forceSize_Unsafe(nbNewSoftBodies);

		mNewSoftBodyNodeIndexPool.forceSize_Unsafe(0);
		mNewSoftBodyNodeIndexPool.reserve(nbNewSoftBodies);
		mNewSoftBodyNodeIndexPool.forceSize_Unsafe(nbNewSoftBodies);

		mNewSoftBodyElementIndexPool.forceSize_Unsafe(0);
		mNewSoftBodyElementIndexPool.reserve(nbNewSoftBodies);
		mNewSoftBodyElementIndexPool.forceSize_Unsafe(nbNewSoftBodies);

		mNewTetMeshByteSizePool.forceSize_Unsafe(0);
		mNewTetMeshByteSizePool.reserve(nbNewSoftBodies);
		mNewTetMeshByteSizePool.forceSize_Unsafe(nbNewSoftBodies);

		Cm::FlushPool& flushPool = mDynamicContext->getFlushPool();

		const PxU32 maxSoftBodyPerTask = PxgCopyToSoftBodySimTask::NbSoftBodiesPerTask;

		for (PxU32 i = 0; i < nbNewSoftBodies; i += maxSoftBodyPerTask)
		{
			PxgCopyToSoftBodySimTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgCopyToSoftBodySimTask)), PxgCopyToSoftBodySimTask)(*this,
					i, PxMin(maxSoftBodyPerTask, nbNewSoftBodies - i));

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	void PxgSimulationController::copyToGpuFEMCloth(PxBaseTask* continuation)
	{
		PX_PROFILE_ZONE("GpuSimulationController.copyToGpuFEMCloth", 0);

		const PxU32 nbNewFEMCloths = mBodySimManager.mNewFEMClothSims.size();

		mNewFEMClothPool.forceSize_Unsafe(0);
		mNewFEMClothPool.reserve(nbNewFEMCloths);
		mNewFEMClothPool.forceSize_Unsafe(nbNewFEMCloths);

		mNewFEMClothDataPool.forceSize_Unsafe(0);
		mNewFEMClothDataPool.reserve(nbNewFEMCloths);
		mNewFEMClothDataPool.forceSize_Unsafe(nbNewFEMCloths);

		mNewFEMClothNodeIndexPool.forceSize_Unsafe(0);
		mNewFEMClothNodeIndexPool.reserve(nbNewFEMCloths);
		mNewFEMClothNodeIndexPool.forceSize_Unsafe(nbNewFEMCloths);

		mNewFEMClothElementIndexPool.forceSize_Unsafe(0);
		mNewFEMClothElementIndexPool.reserve(nbNewFEMCloths);
		mNewFEMClothElementIndexPool.forceSize_Unsafe(nbNewFEMCloths);

		mNewTriangleMeshByteSizePool.forceSize_Unsafe(0);
		mNewTriangleMeshByteSizePool.reserve(nbNewFEMCloths);
		mNewTriangleMeshByteSizePool.forceSize_Unsafe(nbNewFEMCloths);

		Cm::FlushPool& flushPool = mDynamicContext->getFlushPool();

		const PxU32 maxFEMClothsPerTask = PxgCopyToFEMClothSimTask::NbFEMClothsPerTask;

		for (PxU32 i = 0; i < nbNewFEMCloths; i += maxFEMClothsPerTask)
		{
			PxgCopyToFEMClothSimTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgCopyToFEMClothSimTask)), PxgCopyToFEMClothSimTask)(*this,
					i, PxMin(maxFEMClothsPerTask, nbNewFEMCloths - i));

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	// AD: this is only for new articulations.
	void PxgSimulationController::copyToGpuArticulationSim(const PxU32 bodySimOffset, PxU32 startIndex, PxU32 nbToCopy,
		PxI32* sharedArticulationLinksIndex, PxI32* sharedArticulationDofIndex,
		PxI32* sharedArticulationSpatialTendonIndex,
		PxI32* sharedArticulationAttachmentIndex,
		PxI32* sharedArticulationFixedTendonIndex,
		PxI32* sharedArticulationTendonJointIndex,
		PxI32* sharedArticulationMimicJointIndex,
		PxI32* sharedArticulationPathToRootIndex)
	{
		void** bodySimsLL = mBodySimManager.mBodies.begin();
		PxArray<PxgArticulationIndices>& newArticulationSims = mBodySimManager.mNewArticulationSims;

		PxU32 endIndex = startIndex + nbToCopy;

		PxU32 nbLinks = 0;
		PxU32 totalDofNeeded = 0;
		PxU32 nbSpatialTendons = 0;
		PxU32 nbAttachments = 0;
		PxU32 nbFixedTendons = 0;
		PxU32 nbTendonJoints = 0;
		PxU32 nbMimicJoints = 0;
		PxU32 nbPathToRoot = 0;

		for (PxU32 i = startIndex; i < endIndex; ++i)
		{
			//index is the node index
			const PxgArticulationIndices& index = newArticulationSims[i];

			PxgBodySim& bodySim = mNewBodySimPool[i + bodySimOffset];

			PxgArticulation& arti = mArticulationPool[i];

			Dy::FeatherstoneArticulation* articulation = reinterpret_cast<Dy::FeatherstoneArticulation*>(bodySimsLL[index.nodeIndex]);

			const Dy::ArticulationCore* core = articulation->getCore();
			//fill in body sim data
			bodySim.freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex = make_float4(0.f, core->wakeCounter, core->sleepThreshold, reinterpret_cast<const PxReal&>(index.nodeIndex));
			bodySim.articulationRemapId = index.remapIndex;
			bodySim.internalFlags = PxsRigidBody::eFIRST_BODY_COPY_GPU;
			//Note: we can raise eFIRST_BODY_COPY_GPU here because this function only processes new articulations.
			//We therefore know that every articulation encountered here is a new articulation.
			//There is no equivalent for rigid dynamics because it processes new and updated bodies into a single function.

			//need to know how many links are in the articulations
			const Dy::ArticulationData& data = articulation->getArticulationData();
			const PxU32 linkCount = data.getLinkCount();
			const PxU32 spatialTendonCount = data.getSpatialTendonCount();
			const PxU32 fixedTendonCount = data.getFixedTendonCount();
			const PxU32 mimicJointCount = data.getMimicJointCount();
			const PxU32 pathToRootCount = data.getPathToRootElementCount();

			PxgArticulationData& artiData = arti.data;
			//fill in pxg articulation data
			artiData.bodySimIndex = index.nodeIndex;
			artiData.numLinks = PxU16(linkCount);
			artiData.numJointDofs = PxU16(data.getDofs());
			artiData.index = index.remapIndex;
			artiData.numSpatialTendons = PxU16(spatialTendonCount);
			artiData.numFixedTendons = PxU16(fixedTendonCount);
			artiData.numMimicJoints = PxU16(mimicJointCount);
			artiData.numPathToRoots = pathToRootCount;
			
			// AD: these comments don't really make sense anymore but leaving them here until we clean up jcalc for real, maybe it'll help understanding things.
			artiData.confiDirty = true; //when the articulation inserts into the scene, raised this flag will trigger the whole logic in jcal
			artiData.gpuDirtyFlag = 0; //this flag will trigger copying articulation data to the block format in jcalc in the GPU
			artiData.updateDirty = 0;// Dy::ArticulationDirtyFlag::eALL;

			artiData.flags = data.getArticulationFlags();

			nbLinks += linkCount;
			nbSpatialTendons += spatialTendonCount;
			nbFixedTendons += fixedTendonCount;

			nbMimicJoints += mimicJointCount;
			nbPathToRoot += data.getPathToRootElementCount();

			for (PxU32 b = 0; b < spatialTendonCount; ++b)
			{
				Dy::ArticulationSpatialTendon* tendon = data.getSpatialTendon(b);
				nbAttachments += tendon->getNumAttachments();
			}

			for (PxU32 b = 0; b < fixedTendonCount; ++b)
			{
				Dy::ArticulationFixedTendon* tendon = data.getFixedTendon(b);
				nbTendonJoints += tendon->getNumJoints();
			}

			const PxU32 dofs = articulation->getArticulationData().getDofs();

			PxU32 dirtyFlags = articulation->mGPUDirtyFlags;

			//For each dof-based info that is dirty, we must reserve a number of dofs
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
				totalDofNeeded += dofs;
		}

		int32_t linkBegin = physx::PxAtomicAdd(sharedArticulationLinksIndex, int(nbLinks)) - int32_t(nbLinks);
		PxU32 dofBegin = totalDofNeeded ? PxU32(physx::PxAtomicAdd(sharedArticulationDofIndex, PxI32(totalDofNeeded)) - PxI32(totalDofNeeded)) : 0;
		int32_t spatialTendonBegin = physx::PxAtomicAdd(sharedArticulationSpatialTendonIndex, int(nbSpatialTendons)) - int32_t(nbSpatialTendons);
		int32_t attachmentBegin = physx::PxAtomicAdd(sharedArticulationAttachmentIndex, int(nbAttachments)) - int32_t(nbAttachments);
		int32_t fixedTendonBegin = physx::PxAtomicAdd(sharedArticulationFixedTendonIndex, int(nbFixedTendons)) - int32_t(nbFixedTendons);
		int32_t tendonJointBegin = physx::PxAtomicAdd(sharedArticulationTendonJointIndex, int(nbTendonJoints)) - int32_t(nbTendonJoints);
		int32_t mimicJointBegin = physx::PxAtomicAdd(sharedArticulationMimicJointIndex, int(nbMimicJoints)) - int32_t(nbMimicJoints);
		int32_t pathToRootBegin = physx::PxAtomicAdd(sharedArticulationPathToRootIndex, int(nbPathToRoot)) - int32_t(nbPathToRoot);

		for (PxU32 i = startIndex; i < endIndex; ++i)
		{
			//index is the node index
			const PxU32 nodeIndex = newArticulationSims[i].nodeIndex;

			Dy::FeatherstoneArticulation* articulation = reinterpret_cast<Dy::FeatherstoneArticulation*>(bodySimsLL[nodeIndex]);

			//need to know how many links are in the articulations
			const Dy::ArticulationData& data = articulation->getArticulationData();

			PxgArticulationSimUpdate& update = mLinkJointIndexPool[i];

			update.articulationIndex = i; //KS - not sure what to put in here yet...
			update.dirtyFlags = articulation->mGPUDirtyFlags;
			update.linkStartIndex = linkBegin;
			update.dofDataStartIndex = dofBegin;
			update.spatialTendonStartIndex = spatialTendonBegin;
			update.fixedTendonStartIndex = fixedTendonBegin;
			update.mimicJointStartIndex = mimicJointBegin;
			update.pathToRootIndex = pathToRootBegin;

			const PxU32 spatialTendonCount = data.getSpatialTendonCount();
			for (PxU32 a = 0; a < spatialTendonCount; ++a)
			{
				Dy::ArticulationSpatialTendon* cpuTendon = data.getSpatialTendon(a);
				const PxU32 index = spatialTendonBegin + a;

				PxGpuSpatialTendonData& tendonParam = mSpatialTendonParamPool[index];
				PxgArticulationTendon& tendon = mSpatialTendonPool[index];

				mTendonAttachmentMapPool[index] = attachmentBegin;

				const PxU32 numAttachments = cpuTendon->getNumAttachments();
				tendon.mNbElements = numAttachments;
				tendonParam.stiffness = cpuTendon->mStiffness;
				tendonParam.damping = cpuTendon->mDamping;
				tendonParam.limitStiffness = cpuTendon->mLimitStiffness;
				tendonParam.offset = cpuTendon->mOffset;

				for (PxU32 b = 0; b < numAttachments; ++b)
				{
					Dy::ArticulationAttachment& cpuAttachment = cpuTendon->getAttachment(b);
					const PxU32 attachIndex = attachmentBegin + b;
					PxgArticulationTendonElementFixedData& fData = mAttachmentFixedPool[attachIndex];
					PxGpuTendonAttachmentData& mData = mAttachmentModPool[attachIndex];

					fData.linkInd = cpuAttachment.linkInd;
					fData.parent = cpuAttachment.parent;
					fData.children = cpuAttachment.children;

					mData.relativeOffset = cpuAttachment.relativeOffset;
					mData.restLength = cpuAttachment.restLength;
					mData.coefficient = cpuAttachment.coefficient;
					mData.lowLimit = cpuAttachment.lowLimit;
					mData.highLimit = cpuAttachment.highLimit;
				}

				//compute next tendon's offset
				attachmentBegin += numAttachments;
			}

			spatialTendonBegin = spatialTendonBegin + spatialTendonCount;

			const PxU32 fixedTendonCount = data.getFixedTendonCount();
			for (PxU32 a = 0; a < fixedTendonCount; ++a)
			{
				Dy::ArticulationFixedTendon* cpuTendon = data.getFixedTendon(a);
				const PxU32 index = fixedTendonBegin + a;

				PxGpuFixedTendonData& tendonParam = mFixedTendonParamPool[index];
				PxgArticulationTendon& tendon = mFixedTendonPool[index];

				mTendonTendonJointMapPool[index] = tendonJointBegin;

				const PxU32 numJoints = cpuTendon->getNumJoints();
				tendon.mNbElements = numJoints;
				tendonParam.stiffness = cpuTendon->mStiffness;
				tendonParam.damping = cpuTendon->mDamping;
				tendonParam.limitStiffness = cpuTendon->mLimitStiffness;
				tendonParam.offset = cpuTendon->mOffset;
				tendonParam.lowLimit = cpuTendon->mLowLimit;
				tendonParam.highLimit = cpuTendon->mHighLimit;
				tendonParam.restLength = cpuTendon->mRestLength;

				for (PxU32 b = 0; b < numJoints; ++b)
				{
					Dy::ArticulationTendonJoint& cpuTendonJoint = cpuTendon->getTendonJoint(b);
					const PxU32 tendonJointIndex = tendonJointBegin + b;
					PxgArticulationTendonElementFixedData& fixedData = mTendonJointFixedDataPool[tendonJointIndex];

					fixedData.linkInd = cpuTendonJoint.linkInd;
					fixedData.parent = cpuTendonJoint.parent;
					fixedData.children = cpuTendonJoint.children;

					PxGpuTendonJointCoefficientData& coefficientData = mTendonJointCoefficientDataPool[tendonJointIndex];
					coefficientData.axis = cpuTendonJoint.axis;
					coefficientData.coefficient = cpuTendonJoint.coefficient;
					coefficientData.recipCoefficient = cpuTendonJoint.recipCoefficient;
				}

				//compute next tendon's offset
				tendonJointBegin += numJoints;
			}

			fixedTendonBegin = fixedTendonBegin + fixedTendonCount;

			const PxU32 linkCount = data.getLinkCount();
			for (PxU32 a = 0; a < linkCount; ++a)
			{
				Dy::ArticulationLink& cpuLink = data.getLink(a);

				const PxU32 index = linkBegin + a;
				PxgArticulationLink& link = mLinksPool[index];
				PxgArticulationLinkProp& linkProp = mLinkPropPool[index];
				PxReal& linkWakeCounter = mLinkWakeCounterPool[index];
				PxTransform& body2World = mLinkBody2WorldPool[index];
				PxTransform& body2Actor = mLinkBody2ActorPool[index];
				Cm::UnAlignedSpatialVector& accel = mLinkAccelPool[index];
				const Cm::SpatialVector& inAccel = data.getExternalAcceleration(a);
				accel.top = inAccel.linear;
				accel.bottom = inAccel.angular;

				PxU32& parent = mLinkParentPool[index];

				PxsBodyCore& core = *cpuLink.bodyCore;

				body2World = core.body2World;
				body2Actor = core.getBody2Actor();

				link.initialLinVel = core.linearVelocity;
				link.initialAngVel = core.angularVelocity;
				link.invMass = core.inverseMass;
				//link.invInertia = core.inverseInertia;
				link.penBiasClamp = core.maxPenBias;
				link.maxAngularVelocitySq = core.maxAngularVelocitySq;
				link.maxLinearVelocitySq = core.maxLinearVelocitySq;
				link.angularDamping = core.angularDamping;
				link.linearDamping = core.linearDamping;
				link.offsetSlop = core.offsetSlop;

				link.pathToRootOffset = cpuLink.mPathToRootStartIndex;
				link.numPathToRoot = cpuLink.mPathToRootCount;
				link.childrenOffset = cpuLink.mChildrenStartIndex;
				link.numChildren = cpuLink.mNumChildren;
				//link.linkIndex = linkBegin + a;

				parent = cpuLink.parent;

				link.cfmScale = core.cfmScale;

				link.disableGravity = !!core.disableGravity;
				link.retainsAccelerations = core.mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS;

				linkWakeCounter = core.wakeCounter;

				linkProp.invInertia = core.inverseInertia;
				linkProp.invMass = core.inverseMass;

				Dy::ArticulationJointCore* joint = cpuLink.inboundJoint;
				Dy::ArticulationJointCoreData& jointDatum = data.getJointData(a);

				if (joint)
				{
					//New Joint - force GPU to update everything!
					joint->jCalcUpdateFrames = true;
					mJointPool[index] = *joint;
					mJointDataPool[index] = jointDatum;
					joint->jCalcUpdateFrames = false;
				}
			}

			linkBegin = linkBegin + linkCount;

			const PxU32 dofs = data.getDofs();

			PxU32 dirtyFlags = articulation->mGPUDirtyFlags;

			if (dofs)
			{
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointVelocities(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointForces(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointTargetPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointTargetVelocities(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
			}

			const PxU32 mimicJointCount = data.getMimicJointCount();
			if(mimicJointCount)
			{
				Dy::ArticulationMimicJointCore** cpuMimicJointCores = data.getMimicJointCores();
				for(PxU32 a = 0; a < mimicJointCount; a++)
				{
					const Dy::ArticulationMimicJointCore& cpuMimicJointCore = *cpuMimicJointCores[a];
					mMimicJointPool[a + mimicJointBegin] = cpuMimicJointCore;
				}
				mimicJointBegin += mimicJointCount;
			}

			const PxU32 pathToRootCount = data.getPathToRootElementCount();

			if (pathToRootCount)
			{
				const PxU32* pathToRoots = data.getPathToRootElements();
				PxMemCopy(mPathToRootPool.begin() + pathToRootBegin, pathToRoots, pathToRootCount * sizeof(PxU32));
				pathToRootBegin += pathToRootCount;
			}

			articulation->clearGPUDirtyFlags();
		}
	}

	void PxgSimulationController::copyToGpuSoftBodySim(PxU32 startIndex, PxU32 nbToCopy)
	{
		void** bodySimsLL = mBodySimManager.mBodies.begin();
		PxArray<PxgSoftBodyIndices>& newSoftBodySims = mBodySimManager.mNewSoftBodySims;
	
		PxU32 endIndex = startIndex + nbToCopy;

		PxsHeapMemoryAllocator* alloc = mSoftBodyCore->mHeapMemoryManager->mMappedMemoryAllocators;

		for (PxU32 i = startIndex; i<endIndex; ++i)
		{
			//index is the node index
			const PxgSoftBodyIndices& index = newSoftBodySims[i];

			PxgSoftBody& gpuSoftBody = mNewSoftBodyPool[i];
			PxgSoftBodyData& gpuSoftBodyData = mNewSoftBodyDataPool[i];

			mNewSoftBodyNodeIndexPool[i] = index.nodeIndex;

			Dy::DeformableVolume* deformableVolume = reinterpret_cast<Dy::DeformableVolume*>(bodySimsLL[index.nodeIndex]);

			PxTetrahedronMeshGeometryLL& tetGeom = deformableVolume->getShapeCore().mGeometry.get<PxTetrahedronMeshGeometryLL>();
			const Gu::BVTetrahedronMesh* colTetMesh = static_cast<const Gu::BVTetrahedronMesh*>(Gu::_getTetraMeshData(tetGeom));

			//copy tetMesh data to mapped memory
			PxU32 byteSize = PxgSoftBodyUtil::computeTetMeshByteSize(colTetMesh);
			byteSize = (byteSize + 255) & ~255;

			const Gu::TetrahedronMesh* simTetMesh = static_cast<const Gu::TetrahedronMesh*>(deformableVolume->getSimulationMesh());
			const Gu::DeformableVolumeAuxData* softBodyAuxData = static_cast<const Gu::DeformableVolumeAuxData*>(deformableVolume->getAuxData());

			const PxU32 nbTets = colTetMesh->getNbTetrahedronsFast();
			gpuSoftBody.mTetIndices = reinterpret_cast<uint4*>( alloc->allocate(sizeof(uint4) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mTetMeshSurfaceHint = reinterpret_cast<PxU8*>( alloc->allocate(sizeof(PxU8) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mTetIndicesRemapTable = reinterpret_cast<PxU32*>( alloc->allocate(sizeof(float) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			
			gpuSoftBody.mTetraRestPoses = reinterpret_cast<PxMat33*>(alloc->allocate(sizeof(PxMat33) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			
			const PxU32 nbVerts = colTetMesh->getNbVerticesFast();

			const PxU32 nbTetsGM = simTetMesh->getNbTetrahedronsFast();
			const PxU32 nbVertsGM = simTetMesh->getNbVerticesFast();
			const PxU32 nbPartitionsGM = softBodyAuxData->getNbGMPartitionFast();
			const PxU32 nbTetsPerElementGM = softBodyAuxData->mNumTetsPerElement;
			const PxU32 nbElementsGM = nbTetsGM / nbTetsPerElementGM;
			const PxU32 nbVertsPerElementGM = nbTetsPerElementGM == 1 ? 4 : 8;

			const PxU32 nbRemapOutputSize = softBodyAuxData->getGMRemapOutputSizeFast();
			const PxU32 nbTetRemapSize = softBodyAuxData->getNbTetRemapSizeFast();

			gpuSoftBody.mVertsBarycentricInGridModel = reinterpret_cast<float4*>(alloc->allocate(sizeof(float4) * nbVerts, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mVertsRemapInGridModel = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));

			gpuSoftBody.mTetsRemapColToSim = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbTetRemapSize, PxsHeapStats::eSIMULATION, PX_FL));
			gpuSoftBody.mTetsAccumulatedRemapColToSim = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbTets, PxsHeapStats::eSIMULATION, PX_FL));

			gpuSoftBody.mSurfaceVertsHint = reinterpret_cast<PxU8*>(alloc->allocate(sizeof(PxU8) * nbVerts, PxsHeapStats::eSIMULATION, PX_FL));
			gpuSoftBody.mSurfaceVertToTetRemap = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION, PX_FL));

			gpuSoftBody.mOrderedMaterialIndices = reinterpret_cast<PxU16*>(alloc->allocate(sizeof(PxU16) * nbTetsGM, PxsHeapStats::eSIMULATION, PX_FL));
			gpuSoftBody.mMaterialIndices = reinterpret_cast<PxU16*>(alloc->allocate(sizeof(PxU16) * nbTetsGM, PxsHeapStats::eSIMULATION, PX_FL));

			gpuSoftBody.mSimTetIndices = reinterpret_cast<uint4*>(alloc->allocate(sizeof(uint4) * nbTetsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mSimTetraRestPoses = reinterpret_cast<PxgMat33Block*>(alloc->allocate(sizeof(PxgMat33Block) * ((nbTetsGM+31)/32), PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			
			gpuSoftBody.mSimOrderedTetrahedrons = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbElementsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mSimPullIndices = reinterpret_cast<uint4*>(alloc->allocate(sizeof(PxU32) * nbElementsGM * nbVertsPerElementGM, PxsHeapStats::eSIMULATION, PX_FL));

			//we need to allocate more memory for mGMOrderedVertInvMassCP for the accumulated buffer for duplicate verts 
			//gpuSoftBody.mGMOrderedVertInvMassCP = reinterpret_cast<float4*>(alloc->allocate(sizeof(float4) * nbRemapOutputSize, PX_FL));

			gpuSoftBody.mSimAccumulatedPartitionsCP = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbPartitionsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));

			if (nbRemapOutputSize) // used for tet mesh only
			{
				gpuSoftBody.mSimRemapOutputCP = reinterpret_cast<uint4*>(alloc->allocate(sizeof(PxU32) * nbElementsGM * nbVertsPerElementGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
				gpuSoftBody.mSimAccumulatedCopiesCP = reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbVertsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			}

			const Dy::DeformableVolumeCore& core = deformableVolume->getCore();

			PxgSoftBodyUtil::initialTetData(gpuSoftBody, colTetMesh, simTetMesh, softBodyAuxData, 
				core.materialHandles.begin(), alloc);
			//copy tetMesh data to mapped memory
			//const PxU32 byteSize = PxgSoftBodyUtil::computeTetMeshByteSize(tetMesh);
			//void* mem = alloc->allocate(byteSize, PX_FL);

			//PxgSoftBodyUtil::computeBasisMatrix(restPoses.begin(), tetMesh);

			PxU8* tMem = reinterpret_cast<PxU8*>(alloc->allocate(byteSize, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			mNewTetMeshByteSizePool[i] = byteSize;

			const PxU32 nbPackedNodes = PxgSoftBodyUtil::loadOutTetMesh(tMem, colTetMesh);

			gpuSoftBody.mTetMeshData = tMem;

			gpuSoftBody.mActorFlags = core.actorFlags;
			gpuSoftBody.mBodyFlags = core.bodyFlags;
			gpuSoftBody.mVolumeFlags = core.volumeFlags;

			gpuSoftBody.mLinearDamping = core.linearDamping;
			gpuSoftBody.mMaxLinearVelocity = core.maxLinearVelocity;
			gpuSoftBody.mPenBiasClamp = core.maxPenetrationBias;

			gpuSoftBody.mSettlingThreshold = core.settlingThreshold;
			gpuSoftBody.mSleepThreshold = core.sleepThreshold;
			gpuSoftBody.mSettlingDamping = core.settlingDamping;
			gpuSoftBody.mSelfCollisionFilterDistance = core.selfCollisionFilterDistance;
			gpuSoftBody.mSelfCollisionStressTolerance = core.selfCollisionStressTolerance;

			gpuSoftBody.mInitialRotation = core.initialRotation;

			gpuSoftBody.mNumVerts = nbVerts;
			gpuSoftBody.mNumTets = nbTets;
			gpuSoftBody.mElementIndex = deformableVolume->getElementId();
			gpuSoftBody.mGpuRemapIndex = deformableVolume->getGpuRemapId();
			gpuSoftBody.mNumVertsGM = nbVertsGM;
			gpuSoftBody.mNumTetsGM = nbTetsGM;
			gpuSoftBody.mNumPartitionsGM = nbPartitionsGM;
			gpuSoftBodyData.mRemapOutputSizeGM = nbRemapOutputSize;
		
			gpuSoftBodyData.mMaxTetsPerPartitionsGM = softBodyAuxData->getGMMaxTetsPerPartitionsFast();

			gpuSoftBodyData.mNbPackedNodes = nbPackedNodes;

			gpuSoftBodyData.mTetsRemapSize = nbTetRemapSize;

			gpuSoftBody.mRestDistance = deformableVolume->getShapeCore().mRestOffset;

			gpuSoftBody.mOriginalContactOffset = deformableVolume->getShapeCore().mContactOffset;
		
			mNewSoftBodyElementIndexPool[i] = deformableVolume->getElementId();
		}
	}

	void PxgSimulationController::copyToGpuFEMClothSim(PxU32 startIndex, PxU32 nbToCopy)
	{
		void** bodySimsLL = mBodySimManager.mBodies.begin();
		PxArray<PxgFEMClothIndices>& newFEMClothSims = mBodySimManager.mNewFEMClothSims;

		PxU32 endIndex = startIndex + nbToCopy;

		PxsHeapMemoryAllocator* alloc = mFEMClothCore->mHeapMemoryManager->mMappedMemoryAllocators;
		PxgGpuNarrowphaseCore* npCore = mDynamicContext->getNarrowphaseCore();

		PxsDeformableSurfaceMaterialData* materials = npCore->mGpuFEMClothMaterialManager.getCPUMaterialData();

		for(PxU32 i = startIndex; i < endIndex; ++i)
		{
			// index is the node index
			const PxgFEMClothIndices& index = newFEMClothSims[i];

			PxgFEMCloth& gpuFEMCloth = mNewFEMClothPool[i];
			PxgFEMClothData& gpuFEMClothData = mNewFEMClothDataPool[i];

			mNewFEMClothNodeIndexPool[i] = index.nodeIndex;

			Dy::DeformableSurface* deformableSurface = reinterpret_cast<Dy::DeformableSurface*>(bodySimsLL[index.nodeIndex]);

			const Dy::DeformableSurfaceCore& core = deformableSurface->getCore();

			gpuFEMCloth.mLinearDamping = core.linearDamping;
			gpuFEMCloth.mMaxLinearVelocity = core.maxLinearVelocity;
			gpuFEMCloth.mPenBiasClamp = core.maxPenetrationBias;

			gpuFEMCloth.mSettlingThreshold = core.settlingThreshold;
			gpuFEMCloth.mSleepThreshold = core.sleepThreshold;
			gpuFEMCloth.mSettlingDamping = core.settlingDamping;
			gpuFEMCloth.mSelfCollisionFilterDistance = core.selfCollisionFilterDistance;

			gpuFEMCloth.mRestDistance = deformableSurface->getShapeCore().mRestOffset;
			gpuFEMCloth.mOriginalContactOffset = deformableSurface->getShapeCore().mContactOffset;

			gpuFEMCloth.mNbCollisionPairUpdatesPerTimestep = core.nbCollisionPairUpdatesPerTimestep;
			gpuFEMCloth.mNbCollisionSubsteps = core.nbCollisionSubsteps;

			PxTriangleMeshGeometryLL& triangleGeom = deformableSurface->getShapeCore().mGeometry.get<PxTriangleMeshGeometryLL>();
			const Gu::TriangleMesh* const triangleMesh = _getMeshData(triangleGeom);

			// copy triangleMesh data to mapped memory
			PxU32 byteSize = PxgFEMClothUtil::computeTriangleMeshByteSize(triangleMesh);
			byteSize = (byteSize + 255) & ~255;

			mNewTriangleMeshByteSizePool[i] = byteSize;

			const PxU32 nbTriangles = triangleMesh->getNbTrianglesFast();
			const PxU32 nbVerts = triangleMesh->getNbVerticesFast();

			gpuFEMCloth.mTriangleVertexIndices =
				reinterpret_cast<uint4*>(alloc->allocate(sizeof(uint4) * nbTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			gpuFEMCloth.mMaterialIndices = reinterpret_cast<PxU16*>(
				alloc->allocate(sizeof(PxU16) * nbTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			gpuFEMCloth.mDynamicFrictions = reinterpret_cast<float*>(
				alloc->allocate(sizeof(float) * nbVerts, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			const PxU16* materialHandles = core.materialHandles.begin();
			const PxU32 nbMaterials = core.materialHandles.size();

			PxU8* tMem = reinterpret_cast<PxU8*>(
				alloc->allocate(sizeof(PxU8) * byteSize, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			const PxU32 nbPackedNodes = PxgFEMClothUtil::loadOutTriangleMesh(tMem, triangleMesh);
			gpuFEMCloth.mTriMeshData = tMem;


			PxArray<uint2> trianglePairTriangleIndices;
			PxArray<uint4> trianglePairVertexIndices;

			// initial "unordered" triangle and triangle-pair data
			const PxU32 nbTrianglePairs =
				PxgFEMClothUtil::initialTriangleData(gpuFEMCloth, trianglePairTriangleIndices, trianglePairVertexIndices, triangleMesh,
													 materialHandles, materials, nbMaterials, alloc);

			PxArray<PxU32> sharedTrianglePairs;
			PxArray<PxU32> nonSharedTriangles;
			PxArray<PxU32> nonSharedTrianglePairs;

			PxgFEMClothUtil::categorizeClothConstraints(sharedTrianglePairs, nonSharedTriangles, nonSharedTrianglePairs, gpuFEMCloth,
														trianglePairTriangleIndices);

			const PxU32 nbNonSharedTriangles = nonSharedTriangles.size();
			const PxU32 nbSharedTrianglePairs = sharedTrianglePairs.size();
			const PxU32 nbNonSharedTrianglePairs = nonSharedTrianglePairs.size();

			PX_ASSERT(nbTrianglePairs == nbSharedTrianglePairs + nbNonSharedTrianglePairs);
			PX_UNUSED(nbTrianglePairs);

			gpuFEMCloth.mNbNonSharedTriangles = nbNonSharedTriangles;
			gpuFEMCloth.mNbSharedTrianglePairs = nbSharedTrianglePairs;
			gpuFEMCloth.mNbNonSharedTrianglePairs = nbNonSharedTrianglePairs;

			if (nbNonSharedTriangles)
			{
				gpuFEMCloth.mOrderedNonSharedTriangleVertexIndices_triIndex =
					reinterpret_cast<uint4*>(alloc->allocate(sizeof(uint4) * nbNonSharedTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedNonSharedTriangleRestPoseInv =
					reinterpret_cast<float4*>(alloc->allocate(sizeof(float4) * nbNonSharedTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			}

			if (nbSharedTrianglePairs)
			{
				gpuFEMCloth.mOrderedSharedTrianglePairVertexIndices = reinterpret_cast<uint4*>(alloc->allocate(
					sizeof(uint4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping = reinterpret_cast<float4*>(alloc->allocate(
					sizeof(float4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedSharedRestEdge0_edge1 = reinterpret_cast<float4*>(alloc->allocate(
					sizeof(float4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedSharedRestEdgeLength_material0_material1 = reinterpret_cast<float4*>(alloc->allocate(
					sizeof(float4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mSharedTriPairAccumulatedCopiesCP =
					reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			}

			if (nbNonSharedTrianglePairs)
			{
				gpuFEMCloth.mOrderedNonSharedTrianglePairVertexIndices = reinterpret_cast<uint4*>(alloc->allocate(
					sizeof(uint4) * nbNonSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping = reinterpret_cast<float4*>(alloc->allocate(
					sizeof(float4) * nbNonSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mNonSharedTriPairAccumulatedCopiesCP =
					reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			}

			PxArray<PxU32> orderedNonSharedTriangles;
			getFEMClothCore()->partitionTriangleSimData(gpuFEMCloth, gpuFEMClothData, orderedNonSharedTriangles, nonSharedTriangles, alloc);

			PxgFEMClothUtil::computeNonSharedTriangleConfiguration(gpuFEMCloth, orderedNonSharedTriangles, nonSharedTriangles, triangleMesh);

			// triangle-pair partition
			const PxU32 nbSharedTriPairPartitions = 8;
			const PxU32 nbNonSharedTriPairPartitions = 8;

			PxArray<PxU32> orderedSharedTrianglePairs;
			PxArray<PxU32> orderedNonSharedTrianglePairs;

			getFEMClothCore()->partitionTrianglePairSimData(gpuFEMCloth, gpuFEMClothData, nbSharedTriPairPartitions,
															orderedSharedTrianglePairs, sharedTrianglePairs, trianglePairVertexIndices,
															true, alloc);
			getFEMClothCore()->partitionTrianglePairSimData(gpuFEMCloth, gpuFEMClothData, nbNonSharedTriPairPartitions,
															orderedNonSharedTrianglePairs, nonSharedTrianglePairs,
															trianglePairVertexIndices, false, alloc);

			PxgFEMClothUtil::computeTrianglePairConfiguration(gpuFEMCloth, trianglePairTriangleIndices, trianglePairVertexIndices,
															  orderedSharedTrianglePairs, sharedTrianglePairs, triangleMesh, materials,
															  core.surfaceFlags & PxDeformableSurfaceFlag::eENABLE_FLATTENING, true);
			PxgFEMClothUtil::computeTrianglePairConfiguration(gpuFEMCloth, trianglePairTriangleIndices, trianglePairVertexIndices,
															  orderedNonSharedTrianglePairs, nonSharedTrianglePairs, triangleMesh, materials,
															  core.surfaceFlags & PxDeformableSurfaceFlag::eENABLE_FLATTENING, false);

			gpuFEMCloth.mElementIndex = deformableSurface->getElementId();
			gpuFEMCloth.mGpuRemapIndex = deformableSurface->getGpuRemapId();
			gpuFEMCloth.mActorFlags = core.actorFlags;
			gpuFEMCloth.mBodyFlags = core.bodyFlags;
			gpuFEMCloth.mSurfaceFlags = core.surfaceFlags;

			gpuFEMClothData.mNbPackedNodes = nbPackedNodes;

			mNewFEMClothElementIndexPool[i] = deformableSurface->getElementId();
		}
	}

	void PxgSimulationController::updateGpuArticulationSim(PxU32 startIndex, PxU32 nbToCopy,
		PxI32* sharedArticulationLinksIndex, PxI32* sharedArticulationDofIndex,
		PxI32* sharedSpatialIndex, PxI32* sharedAttachmentIndex,
		PxI32* sharedFixedIndex, PxI32* sharedFixedTendonJointIndex,
		PxI32* sharedMimicJointIndex)
	{
		PxgArticulationUpdate* updatedArtics = &mBodySimManager.mUpdatedArticulations[startIndex];
		PxU32 totalDofNeeded = 0;
		PxU32 totalLinks = 0;
		PxU32 totalSpatialTendon = 0;
		PxU32 totalAttachments = 0;
		PxU32 totalFixedTendon = 0;
		PxU32 totalTendonJoints = 0;
		PxU32 totalMimicJoints = 0;
		for (PxU32 i = 0; i < nbToCopy; ++i)
		{
			//Sum up number of links

			Dy::FeatherstoneArticulation* artic = updatedArtics[i].articulation;
			Dy::ArticulationData& data = artic->getArticulationData();

			PxU32 dirtyFlags = artic->mGPUDirtyFlags;

			//If we have dirty joints or dirty root link, we need to reserve space for the links in the articulation
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINTS || dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_LINKS
				|| dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS || dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL)
				totalLinks += data.getLinkCount();
			else if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM
				|| dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES)
				totalLinks++; //We need space for the root

			// need to check for tendon flags individually as combinations of them may be raised:
			if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON)
				totalSpatialTendon += data.getSpatialTendonCount();
			if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT)
			{
				PxU32 numAttachments = 0u;
				Dy::ArticulationSpatialTendon** spatialTendons = data.getSpatialTendons();
				for(PxU32 j = 0u; j < data.getSpatialTendonCount(); ++j)
				{
					numAttachments += spatialTendons[j]->getNumAttachments();
				}
				totalAttachments += numAttachments;
			}
			if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON)
				totalFixedTendon += data.getFixedTendonCount();
			if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT)
			{
				PxU32 numTendonJoints = 0u;
				Dy::ArticulationFixedTendon** fixedTendons = data.getFixedTendons();
				for(PxU32 j = 0u; j < data.getFixedTendonCount(); ++j)
				{
					numTendonJoints += fixedTendons[j]->getNumJoints();
				}
				totalTendonJoints += numTendonJoints;
			}

			if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT)
				totalMimicJoints += data.getMimicJointCount();

			const PxU32 dofs = artic->getArticulationData().getDofs();

			//For each dof-based info that is dirty, we must reserve a number of dofs
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
				totalDofNeeded += dofs;
			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
				totalDofNeeded += dofs;
		}

		//Output index in dirty buffer...
		PxU32 linkBegin = totalLinks ? PxU32(physx::PxAtomicAdd(sharedArticulationLinksIndex, PxI32(totalLinks)) - PxI32(totalLinks)) : 0;
		PxU32 dofBegin = totalDofNeeded ? PxU32(physx::PxAtomicAdd(sharedArticulationDofIndex, PxI32(totalDofNeeded)) - PxI32(totalDofNeeded)) : 0;
		PxU32 spatialTendonBegin = totalSpatialTendon ? PxU32(physx::PxAtomicAdd(sharedSpatialIndex, PxI32(totalSpatialTendon)) - PxI32(totalSpatialTendon)) : 0;
		PxU32 spatialTendonAttachmentBegin = totalAttachments ? PxU32(physx::PxAtomicAdd(sharedAttachmentIndex, PxI32(totalAttachments)) - PxI32(totalAttachments)) : 0;
		PxU32 fixedTendonBegin = totalFixedTendon ? PxU32(physx::PxAtomicAdd(sharedFixedIndex, PxI32(totalFixedTendon)) - PxI32(totalFixedTendon)) : 0;
		PxU32 fixedTendonJointBegin = totalTendonJoints ? PxU32(physx::PxAtomicAdd(sharedFixedTendonJointIndex, PxI32(totalTendonJoints)) - PxI32(totalTendonJoints)) : 0;
		PxU32 mimicJointBegin = totalMimicJoints ? PxU32(physx::PxAtomicAdd(sharedMimicJointIndex, PxI32(totalMimicJoints)) - PxI32(totalMimicJoints)) : 0;

		for (PxU32 i = 0; i < nbToCopy; ++i)
		{
			Dy::FeatherstoneArticulation* artic = updatedArtics[i].articulation;

			//Copy data...
			PxU32 dirtyFlags = artic->mGPUDirtyFlags;

			PxgArticulationSimUpdate& update = mArticulationUpdatePool[startIndex + i];

			update.articulationIndex = updatedArtics[i].articulationIndex;
			update.dirtyFlags = artic->mGPUDirtyFlags;
			update.linkStartIndex = linkBegin;
			update.dofDataStartIndex = dofBegin;
			update.spatialTendonStartIndex = spatialTendonBegin;
			update.spatialTendonAttachmentStartIndex = spatialTendonAttachmentBegin;
			update.fixedTendonStartIndex = fixedTendonBegin;
			update.fixedTendonJointStartIndex = fixedTendonJointBegin;
			update.mimicJointStartIndex = mimicJointBegin;

			const Dy::ArticulationData& data = artic->getArticulationData();
			const PxU32 dofs = data.getDofs();
			//If we have dirty joints or dirty root link, we need to reserve space for the links in the articulation
			if (dirtyFlags & (Dy::ArticulationDirtyFlag::eDIRTY_JOINTS | Dy::ArticulationDirtyFlag::eDIRTY_LINKS))
			{
				const PxU32 linkCount = data.getLinkCount();
				for (PxU32 a = 0; a < linkCount; ++a)
				{
					Dy::ArticulationLink& cpuLink = data.getLink(a);

					if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_LINKS)
					{
						const PxU32 index = linkBegin + a;
						PxgArticulationLink& link = mLinksPool[index];
						PxgArticulationLinkProp& linkProp = mLinkPropPool[index];
						PxTransform& body2World = mLinkBody2WorldPool[index];
						PxTransform& body2Actor = mLinkBody2ActorPool[index];

						PxU32& parent = mLinkParentPool[index];
						parent = cpuLink.parent;
						
						PxsBodyCore& core = *cpuLink.bodyCore;

						body2World = core.body2World;
						body2Actor = core.getBody2Actor();

						link.initialLinVel = core.linearVelocity;
						link.initialAngVel = core.angularVelocity;
						link.invMass = core.inverseMass;
						
						link.penBiasClamp = core.maxPenBias;
						link.maxAngularVelocitySq = core.maxAngularVelocitySq;
						link.maxLinearVelocitySq = core.maxLinearVelocitySq;
						link.angularDamping = core.angularDamping;
						link.linearDamping = core.linearDamping;
						link.disableGravity = !!core.disableGravity;
						link.retainsAccelerations = core.mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS;
						
						link.childrenOffset = cpuLink.mChildrenStartIndex;
						link.numChildren = cpuLink.mNumChildren;

						linkProp.invInertia = core.inverseInertia;
						linkProp.invMass = core.inverseMass;
					}

					//Skip root link because it doesn't have any joints
					if ((dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINTS) && a != 0)
					{
						Dy::ArticulationJointCore* joint = cpuLink.inboundJoint;
						Dy::ArticulationJointCoreData& jointDatum = data.getJointData(a);

						// AD hack/remove again: if we have dirty joints, we always raise the frame flag because we might have to recompute motionMatrix and RelativeQuats. 
						// this could have been lowered again already though because we might have done jcalc on CPU.
						joint->jCalcUpdateFrames = true;

						mJointPool[linkBegin + a] = *joint;
						mJointDataPool[linkBegin + a] = jointDatum;

						//Clear dirty flag
						joint->jCalcUpdateFrames = false;
					}
				}
				linkBegin += linkCount;
			}

			//KS - if we didn't update links above, we need to check to see if root transforms or joint positions changed, in which case
			//link states still need updating...
			if (!(dirtyFlags & (Dy::ArticulationDirtyFlag::eDIRTY_LINKS)))
			{
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM
					|| dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES)
				{
					Dy::ArticulationLink& cpuLink = data.getLink(0);

					const PxU32 index = linkBegin;
					PxgArticulationLink& link = mLinksPool[index];
					PxgArticulationLinkProp& linkProp = mLinkPropPool[index];
					PxTransform& body2World = mLinkBody2WorldPool[index];
					PxTransform& body2Actor = mLinkBody2ActorPool[index];
					
					PxsBodyCore& core = *cpuLink.bodyCore;

					body2World = core.body2World;
					body2Actor = core.getBody2Actor();

					link.initialLinVel = core.linearVelocity;
					link.initialAngVel = core.angularVelocity;
					link.invMass = core.inverseMass;
					
					link.penBiasClamp = core.maxPenBias;
					link.maxAngularVelocitySq = core.maxAngularVelocitySq;
					link.maxLinearVelocitySq = core.maxLinearVelocitySq;
					link.angularDamping = core.angularDamping;
					link.linearDamping = core.linearDamping;
					link.disableGravity = !!core.disableGravity;
					link.retainsAccelerations = core.mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS;

					linkProp.invInertia = core.inverseInertia;
					linkProp.invMass = core.inverseMass;

					if (!(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS))
						linkBegin++;
				}

				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
				{
					const PxU32 linkCount = data.getLinkCount();
					PxU32 index = update.linkStartIndex;

					for (PxU32 a = 0; a < linkCount; ++a)
					{
						Dy::ArticulationLink& cpuLink = data.getLink(a);

						PxTransform& body2World = mLinkBody2WorldPool[index++];
						PxsBodyCore& core = *cpuLink.bodyCore;
						body2World = core.body2World;
					}

					linkBegin = index;
				}
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER)
			{
				const PxU32 linkCount = data.getLinkCount();
				PxU32 index = update.linkStartIndex;
				const PxReal wakeCounter = artic->getCore()->wakeCounter;

				for (PxU32 a = 0; a < linkCount; ++a)
				{
					PxReal& wc = mLinkWakeCounterPool[index++];
					wc = wakeCounter;
				}
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_USER_FLAGS)
			{
				update.userFlags = artic->getCore()->flags;
			}

			if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT)
			{
				Dy::ArticulationMimicJointCore** cpuMimicJointCores = data.getMimicJointCores(); 
				const PxU32 mimicJointCount = data.getMimicJointCount();
				PxU32 index = update.mimicJointStartIndex;
				for(PxU32 a = 0; a < mimicJointCount; a++)
				{
					const Dy::ArticulationMimicJointCore& cpuMimicJointCore = *cpuMimicJointCores[a];
					Dy::ArticulationMimicJointCore& mimicJointCore = mMimicJointPool[index++];
					mimicJointCore = cpuMimicJointCore;
				}
				mimicJointBegin += mimicJointCount;
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON)
			{
				const PxU32 tendonCount = data.getSpatialTendonCount();
				PxU32 index = update.spatialTendonStartIndex;
				Dy::ArticulationSpatialTendon** spatialTendons = data.getSpatialTendons();

				for (PxU32 a = 0; a < tendonCount; ++a)
				{
					Dy::ArticulationSpatialTendon* cpuTendon = spatialTendons[a];
					PxGpuSpatialTendonData& tendon = mSpatialTendonParamPool[index++];
					tendon.damping = cpuTendon->mDamping;
					tendon.stiffness = cpuTendon->mStiffness;
					tendon.offset = cpuTendon->mOffset;
					tendon.limitStiffness = cpuTendon->mLimitStiffness;
				}
				spatialTendonBegin += tendonCount;
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT)
			{
				const PxU32 tendonCount = data.getSpatialTendonCount();
				PxU32 index = update.spatialTendonAttachmentStartIndex;
				Dy::ArticulationSpatialTendon** spatialTendons = data.getSpatialTendons();

				for (PxU32 a = 0; a < tendonCount; ++a)
				{
					Dy::ArticulationSpatialTendon* cpuTendon = spatialTendons[a];

					const PxU32 numAttachments = cpuTendon->getNumAttachments();

					for (PxU32 b = 0; b < numAttachments; ++b)
					{
						Dy::ArticulationAttachment& cpuAttachment = cpuTendon->getAttachment(b);
						const PxU32 tendonJointIndex = index + b;

						PxGpuTendonAttachmentData& cData = mAttachmentModPool[tendonJointIndex];
						cData.coefficient = cpuAttachment.coefficient;
						cData.lowLimit = cpuAttachment.lowLimit;
						cData.highLimit = cpuAttachment.highLimit;
						cData.restLength = cpuAttachment.restLength;
						cData.relativeOffset = cpuAttachment.relativeOffset;
					}

					index += numAttachments;
				}
				spatialTendonAttachmentBegin = index;
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON)
			{
				const PxU32 tendonCount = data.getFixedTendonCount();
				PxU32 index = update.fixedTendonStartIndex;
				Dy::ArticulationFixedTendon** fixedTendons = data.getFixedTendons();

				for (PxU32 a = 0; a < tendonCount; ++a)
				{
					Dy::ArticulationFixedTendon* cpuTendon = fixedTendons[a];
					PxGpuFixedTendonData& tendon = mFixedTendonParamPool[index++];
					tendon.damping = cpuTendon->mDamping;
					tendon.stiffness = cpuTendon->mStiffness;
					tendon.offset = cpuTendon->mOffset;
					tendon.limitStiffness = cpuTendon->mLimitStiffness;
					tendon.lowLimit = cpuTendon->mLowLimit;
					tendon.highLimit = cpuTendon->mHighLimit;
					tendon.restLength = cpuTendon->mRestLength;
				}
				fixedTendonBegin += tendonCount;
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT)
			{
				const PxU32 tendonCount = data.getFixedTendonCount();
				PxU32 index = update.fixedTendonJointStartIndex;
				Dy::ArticulationFixedTendon** fixedTendons = data.getFixedTendons();


				for (PxU32 a = 0; a < tendonCount; ++a)
				{
					Dy::ArticulationFixedTendon* cpuTendon = fixedTendons[a];

					const PxU32 numJoints = cpuTendon->getNumJoints();

					for (PxU32 b = 0; b < numJoints; ++b)
					{
						Dy::ArticulationTendonJoint& cpuTendonJoint = cpuTendon->getTendonJoint(b);
						const PxU32 tendonJointIndex = index + b;

						PxGpuTendonJointCoefficientData& cData = mTendonJointCoefficientDataPool[tendonJointIndex];
						cData.axis = cpuTendonJoint.axis;
						cData.coefficient = cpuTendonJoint.coefficient;
						cData.recipCoefficient = cpuTendonJoint.recipCoefficient;
					}

					index += numJoints;
				}
				fixedTendonJointBegin = index;
			}

			if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL)
			{
				const PxU32 linkCount = data.getLinkCount();
				PxU32 index = update.linkStartIndex;

				for (PxU32 a = 0; a < linkCount; ++a)
				{
					Cm::UnAlignedSpatialVector& accel = mLinkAccelPool[index++];

					const Cm::SpatialVector& inAccel = data.getExternalAcceleration(a);
					accel.top = inAccel.linear;
					accel.bottom = inAccel.angular;
				}

				linkBegin = index;
			}

			if (dofs)
			{
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}

				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointVelocities(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointForces(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointTargetPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
				{
					PxMemCopy(&mArticulationDofDataPool[dofBegin], data.getJointTargetVelocities(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
			}

			artic->clearGPUDirtyFlags();
		}
	}

	void PxgSimulationController::copyToGpuPBDParticleSystemSim(PxU32 startIndex, PxU32 nbToCopy)
	{
		void** bodySimsLL = mBodySimManager.mBodies.begin();
		PxArray<PxgParticleSystemIndices>& newParticleSystemSims = mBodySimManager.mNewPBDParticleSystemSims;

		PxU32 endIndex = startIndex + nbToCopy;

		PxgParticleSystem* newParticleSystemPool = mPBDParticleSystemCore->mNewParticleSystemPool.begin();
		
		PxU32* newPBDParticleSystemNodeIndexPool = mPBDParticleSystemCore->mNewParticleSystemNodeIndexPool.begin();

		for (PxU32 i = startIndex; i < endIndex; ++i)
		{
			//index is the node index
			const PxgParticleSystemIndices& index = newParticleSystemSims[i];

			PxgParticleSystem& newParticleSystem = newParticleSystemPool[i];

			newPBDParticleSystemNodeIndexPool[i] = index.nodeIndex;

			Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[index.nodeIndex]);
			Dy::ParticleSystemCore& dyParticleSystemCore = dyParticleSystem->getCore();

			//need to fill in PxgParticleSystem data
			PxgParticleSystemData& data = newParticleSystem.mData;

			data.mElementIndex = dyParticleSystem->getElementId();
			data.mRemapIndex = dyParticleSystem->getGpuRemapId();

			newParticleSystem.mCommonData.mGridCellWidth = dyParticleSystemCore.particleContactOffset * 2.0f * dyParticleSystemCore.mNeighborhoodScale;
			newParticleSystem.mCommonData.mGridSizeX = dyParticleSystemCore.gridSizeX;
			newParticleSystem.mCommonData.mGridSizeY = dyParticleSystemCore.gridSizeY;
			newParticleSystem.mCommonData.mGridSizeZ = dyParticleSystemCore.gridSizeZ;
			newParticleSystem.mCommonData.mMaxNeighborhood = dyParticleSystemCore.mMaxNeighborhood;

			PxgParticleBuffer** particleBuffers = reinterpret_cast<PxgParticleBuffer**>(dyParticleSystemCore.mParticleBuffers.begin());
			PxU32 numActiveParticles = 0;
			PxU32 numMaxParticles = 0;
			PxU32 numMaxDiffuseParticles = 0;
			const PxU32 numParticleBuffers = dyParticleSystemCore.mParticleBuffers.size();

			for (PxU32 j = 0; j < numParticleBuffers; ++j)
			{
				numActiveParticles += particleBuffers[j]->mNumActiveParticles;
				numMaxParticles += particleBuffers[j]->mMaxNumParticles;
			}

			PxgParticleClothBuffer** clothBuffers = reinterpret_cast<PxgParticleClothBuffer**>(dyParticleSystemCore.mParticleClothBuffers.begin());
			const PxU32 numClothBuffers = dyParticleSystemCore.mParticleClothBuffers.size();
			for (PxU32 j = 0; j < numClothBuffers; ++j)
			{
				numActiveParticles += clothBuffers[j]->mNumActiveParticles;
				numMaxParticles += clothBuffers[j]->mMaxNumParticles;
			}

			PxgParticleRigidBuffer** rigidBuffers = reinterpret_cast<PxgParticleRigidBuffer**>(dyParticleSystemCore.mParticleRigidBuffers.begin());
			const PxU32 numRigidBuffers = dyParticleSystemCore.mParticleRigidBuffers.size();
			for (PxU32 j = 0; j < numRigidBuffers; ++j)
			{
				numActiveParticles += rigidBuffers[j]->mNumActiveParticles;
				numMaxParticles += rigidBuffers[j]->mMaxNumParticles;
			}

			PxgParticleAndDiffuseBuffer** diffuseBuffers = reinterpret_cast<PxgParticleAndDiffuseBuffer**>(dyParticleSystemCore.mParticleDiffuseBuffers.begin());
			const PxU32 numDiffuseBuffers = dyParticleSystemCore.mParticleDiffuseBuffers.size();
			for (PxU32 j = 0; j < numDiffuseBuffers; ++j)
			{
				numActiveParticles += diffuseBuffers[j]->mNumActiveParticles;
				numMaxParticles += diffuseBuffers[j]->mMaxNumParticles;
				numMaxDiffuseParticles += diffuseBuffers[j]->mMaxNumDiffuseParticles;
			}

			newParticleSystem.mCommonData.mNumParticles = numActiveParticles;
			newParticleSystem.mCommonData.mMaxParticles = numMaxParticles;
			newParticleSystem.mCommonData.mNumParticleBuffers = numParticleBuffers + numClothBuffers + numRigidBuffers + numDiffuseBuffers;
			newParticleSystem.mNumClothBuffers = numClothBuffers;
			newParticleSystem.mNumRigidBuffers = numRigidBuffers;
			newParticleSystem.mNumDiffuseBuffers = numDiffuseBuffers;
			newParticleSystem.mCommonData.mMaxDiffuseParticles = numMaxDiffuseParticles;
			
			// allocate pinned memory(s)
			getPBDParticleSystemCore()->updateParticleSystemData(newParticleSystem, dyParticleSystemCore);
		}
	}

	void PxgSimulationController::copyToGpuBodySim(const PxU32 bodySimOffset, PxU32 bodyStartIndex, PxU32 nbToCopy)
	{
		void** bodySimsLL = mBodySimManager.mBodies.begin();
		PxArray<PxU32>& newBodySims = mBodySimManager.mNewOrUpdatedBodySims;

		PxU32 endIndex = bodyStartIndex + nbToCopy;
		const uint invalidIndex = 0xfffffff;

		for (PxU32 i = bodyStartIndex; i < endIndex; ++i)
		{
			PxgBodySim& bodySim = mNewBodySimPool[i + bodySimOffset];
			const PxU32 index = newBodySims[i];

			PxsRigidBody& rbLL = *reinterpret_cast<PxsRigidBody*>(bodySimsLL[index]);
			PxsBodyCore& bcLL = rbLL.getCore();

			bodySim.linearVelocityXYZ_inverseMassW = make_float4(reinterpret_cast<float3&>(bcLL.linearVelocity), bcLL.inverseMass);
			bodySim.angularVelocityXYZ_maxPenBiasW = make_float4(reinterpret_cast<float3&>(bcLL.angularVelocity), bcLL.maxPenBias);
			bodySim.maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW = make_float4(bcLL.maxLinearVelocitySq, bcLL.maxAngularVelocitySq, bcLL.linearDamping, bcLL.angularDamping);
			bodySim.inverseInertiaXYZ_contactReportThresholdW = make_float4(reinterpret_cast<float3&>(bcLL.inverseInertia), bcLL.contactReportThreshold);
			bodySim.body2World = PxAlignedTransform(bcLL.body2World.p.x, bcLL.body2World.p.y, bcLL.body2World.p.z,
				PxAlignedQuat(bcLL.body2World.q.x, bcLL.body2World.q.y, bcLL.body2World.q.z, bcLL.body2World.q.w));
			const PxTransform& body2Actor = bcLL.getBody2Actor();
			bodySim.body2Actor_maxImpulseW = PxAlignedTransform(body2Actor.p.x, body2Actor.p.y, body2Actor.p.z,
				PxAlignedQuat(body2Actor.q.x, body2Actor.q.y, body2Actor.q.z, body2Actor.q.w));
			bodySim.body2Actor_maxImpulseW.p.w = bcLL.maxContactImpulse;
			
			bodySim.externalLinearAcceleration = make_float4(0.0f);
			bodySim.externalAngularAcceleration = make_float4(0.0f);
			if (mBodySimManager.mExternalAccelerations && mBodySimManager.mExternalAccelerations->hasAccelerations())
			{
				const PxsRigidBodyExternalAcceleration& acc = mBodySimManager.mExternalAccelerations->get(index);
				bodySim.externalLinearAcceleration = make_float4(acc.linearAcceleration.x, acc.linearAcceleration.y, acc.linearAcceleration.z, 0.0f);
				bodySim.externalAngularAcceleration = make_float4(acc.angularAcceleration.x, acc.angularAcceleration.y, acc.angularAcceleration.z, 0.0f);				
			}

			bodySim.freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex = make_float4(bcLL.freezeThreshold, bcLL.wakeCounter, bcLL.sleepThreshold, reinterpret_cast<const PxReal&>(index));
			bodySim.sleepLinVelAccXYZ_freezeCountW = make_float4(reinterpret_cast<float3&>(rbLL.mSleepLinVelAcc), rbLL.mFreezeCount);
			bodySim.sleepAngVelAccXYZ_accelScaleW = make_float4(reinterpret_cast<float3&>(rbLL.mSleepAngVelAcc), rbLL.mAccelScale);
			bodySim.disableGravity = bcLL.disableGravity;
			bodySim.lockFlags = bcLL.lockFlags;
			bodySim.internalFlags = rbLL.mInternalFlags;
			//Note: it might be tempting to copy the pattern for new articulations and use this opportunity to raise bodySim.internalFlags with eFIRST_BODY_COPY_GPU.
			//This is possible for new articulations because we process new articulations in a single function that iterates over mBodySimManager.mNewArticulationSims.
			//It is not possible for rigid bodies because we bundle new and updated bodies here so we cannot tell which bodies are new and which are updated.
			//The way we distinguish new and updated bodies is that we raise eFIRST_BODY_COPY_GPU when the add the body through the cpu api.
			rbLL.mInternalFlags &= ~PxsRigidBody::eFIRST_BODY_COPY_GPU;
			rbLL.mInternalFlags &= ~PxsRigidBody::eVELOCITY_COPY_GPU;
			bodySim.offsetSlop = bcLL.offsetSlop;

			//if the body sim is not articulation, remapId will be invalid 
			bodySim.articulationRemapId = invalidIndex;
		}
	}

	void PxgSimulationController::postCopyToShapeSim()
	{
		mCudaContextManager->acquireContext();

		mSimulationCore->mPxgShapeSimManager.gpuMemDmaUpShapeSim(
			mSimulationCore->mCudaContext,
			mSimulationCore->getStream(),
			mSimulationCore->mGpuKernelWranglerManager->getKernelWrangler()
		);

		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::postCopyToBodySim(bool enableBodyAccelerations)
	{
		mCudaContextManager->acquireContext();

		//ML: we reserve the mNewUpdatedBodies before we kick off the beforeSolver task, which update the the mNewUpdatedBodies using multiple thread. After the beforeSolver task,
		//we kick off updateBodiesAndShapes tasks. We need to set the size of mNewUpdatedBodies here 
		PxPinnedArray<PxgBodySimVelocityUpdate>& updatedBodySimPool = mBodySimManager.mNewUpdatedBodies;
		updatedBodySimPool.forceSize_Unsafe(mBodySimManager.mNbUpdatedBodies);

		const PxU32 nbTotalBodies = mBodySimManager.mTotalNumBodies;
		const PxU32 nbTotalArticulations = mBodySimManager.mTotalNumArticulations;

		mSimulationCore->gpuMemDmaUpBodySim(updatedBodySimPool, mNewBodySimPool,
			mLinksPool, mLinkWakeCounterPool, mLinkAccelPool, mLinkPropPool, mLinkParentPool, mLinkChildPool,
			mLinkBody2WorldPool, mLinkBody2ActorPool,
			mJointPool, mJointDataPool, mLinkJointIndexPool,
			mArticulationPool, mSpatialTendonParamPool, mSpatialTendonPool, mAttachmentFixedPool, mAttachmentModPool,
			mTendonAttachmentMapPool, mFixedTendonParamPool, mFixedTendonPool, mTendonJointFixedDataPool,
			mTendonJointCoefficientDataPool, mTendonTendonJointMapPool,
			mMimicJointPool, mPathToRootPool, nbTotalBodies, nbTotalArticulations,
			mMaxLinks, mMaxDofs, mMaxMimicJoints, mMaxSpatialTendons, mMaxAttachments,
			mMaxFixedTendons, mMaxTendonJoints, enableBodyAccelerations);

		mSimulationCore->updateBodies(updatedBodySimPool.size(), mNewBodySimPool.size());

		mSimulationCore->updateArticulations(mBodySimManager.mNewArticulationSims.size(), mArticulationUpdatePool.begin(),
			mArticulationUpdatePool.size(), mArticulationDofDataPool.begin());

		//this is for kinematic bodies. Because for kinematic bodies, we still update the transform everyframe but we don't need to simulate(solve and integrate) so mHasBeenSimulated
		//flag will be false, but we still need to reset the new bodies/shapes sim managers
		mBodySimManager.reset();
		//mShapeSimManager.reset();
		mCudaContextManager->releaseContext();
	}

	//This is called before broad phase
	void PxgSimulationController::preIntegrateAndUpdateBoundParticleSystem(const PxVec3 gravity, const PxReal dt)
	{
		const PxU32 nbTotalActiveParticleSystems = mBodySimManager.getNumActiveParticleSystem();

		if (nbTotalActiveParticleSystems == 0)
			return;

		mCudaContextManager->acquireContext();
		mSimulationCore->gpuMemDmaUpParticleSystem(mBodySimManager);

		if (mPBDParticleSystemCore)
		{
			PxU32* activePBDParticleSystems = mBodySimManager.mActivePBDParticleSystems.begin();
			const PxU32 nbActivePBDParticleSystems = mBodySimManager.mActivePBDParticleSystems.size();

			mPBDParticleSystemCore->preIntegrateSystems(nbActivePBDParticleSystems, gravity, dt);

			mPBDParticleSystemCore->updateBounds(mPBDParticleSystemCore->mParticleSystemPool.begin(), activePBDParticleSystems, nbActivePBDParticleSystems);
		}
		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::preIntegrateAndUpdateBoundSoftBody(const PxVec3 gravity, const PxReal dt)
	{
		PX_UNUSED(gravity);
		PX_UNUSED(dt);
		const PxU32 nbActiveSoftbodies = mBodySimManager.mActiveSoftbodiesStaging.size();

		if (nbActiveSoftbodies == 0 && mBodySimManager.mActiveSoftbodies.size() == 0)
			return;

		mCudaContextManager->acquireContext();

		SoftBodyAttachmentAndFilterData dmaData;
		dmaData.rigidAttachments = &mRigidSoftBodyAttachments.mAttachments;
		dmaData.rigidFilterPairs = &mSoftBodyRigidFilterPairs;
		dmaData.dirtyRigidAttachments = mRigidSoftBodyAttachments.mAttachmentsDirty;
		dmaData.activeRigidAttachments = &mRigidSoftBodyAttachments.mActiveAttachments;
		dmaData.dirtyActiveRigidAttachments = mRigidSoftBodyAttachments.mActiveAttachmentsDirty;
		dmaData.softBodyAttachments = &mSoftBodySoftBodyAttachments.mAttachments;
		dmaData.dirtySoftBodyAttachments = mSoftBodySoftBodyAttachments.mAttachmentsDirty;
		dmaData.activeSoftBodyAttachments = &mSoftBodySoftBodyAttachments.mActiveAttachments;
		dmaData.dirtyActiveSoftBodyAttachments = mSoftBodySoftBodyAttachments.mActiveAttachmentsDirty;

		dmaData.dirtyDeformableVolumeForFilterPairs = &mDirtyDeformableVolumeForFilterPairs;
		dmaData.clothAttachments = &mSoftBodyClothAttachments.mAttachments;
		dmaData.clothFilterPairs = &mSoftBodyClothTetVertFilterPairs;
		dmaData.dirtyClothAttachments = mSoftBodyClothAttachments.mAttachmentsDirty;
		dmaData.activeClothAttachments = &mSoftBodyClothAttachments.mActiveAttachments;
		dmaData.dirtyActiveClothAttachments = mSoftBodyClothAttachments.mActiveAttachmentsDirty;

		dmaData.particleAttachments = &mParticleSoftBodyAttachments.mAttachments;
		dmaData.particleFilterPairs = &mSoftBodyParticleFilterPairs;
		dmaData.dirtyParticleAttachments = mParticleSoftBodyAttachments.mAttachmentsDirty;
		dmaData.activeParticleAttachments = &mParticleSoftBodyAttachments.mActiveAttachments;
		dmaData.dirtyActiveParticleAttachments = mParticleSoftBodyAttachments.mActiveAttachmentsDirty;

		mSimulationCore->gpuMemDmaUpSoftBodies(mNewSoftBodyPool,
			mNewTetMeshByteSizePool.begin(),
			mNewSoftBodyDataPool,
			mNewSoftBodyNodeIndexPool,
			mNewSoftBodyElementIndexPool,
			mSoftBodyPool,
			mSoftBodyDataPool,
			mSoftBodyElementIndexPool,
			mSoftBodyNodeIndexPool,
			mBodySimManager,
			dmaData
			);

		mRigidSoftBodyAttachments.mAttachmentsDirty = false;
		mRigidSoftBodyAttachments.mActiveAttachmentsDirty = false;

		mSoftBodyClothAttachments.mAttachmentsDirty = false;
		mSoftBodyClothAttachments.mActiveAttachmentsDirty = false;

		mParticleSoftBodyAttachments.mAttachmentsDirty = false;
		mParticleSoftBodyAttachments.mActiveAttachmentsDirty = false;

		mSoftBodySoftBodyAttachments.mAttachmentsDirty = false;
		mSoftBodySoftBodyAttachments.mActiveAttachmentsDirty = false;

		//clear dirty soft body filter pairs
		mDirtyDeformableVolumeForFilterPairs.forceSize_Unsafe(0);

		PxU32* activeSoftbodies = mBodySimManager.mActiveSoftbodies.begin();

		if(nbActiveSoftbodies == 0 && mBodySimManager.mActiveSoftbodies.size() == 0)
		{
			mCudaContextManager->releaseContext();
			return;
		}

		//Need to implement this
		mSoftBodyCore->preIntegrateSystems(mSoftBodyPool.begin(), activeSoftbodies, nbActiveSoftbodies, gravity, dt);

		mSoftBodyCore->refitBound(mSoftBodyPool.begin(), nbActiveSoftbodies);

		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::preIntegrateAndUpdateBoundFEMCloth(const PxVec3 gravity, const PxReal dt)
	{
		const PxU32 nbActiveFEMCloths = mBodySimManager.mActiveFEMClothStaging.size();

		if (nbActiveFEMCloths == 0 && mBodySimManager.mActiveFEMCloths.size() == 0)
			return;

		mCudaContextManager->acquireContext();

		mSimulationCore->gpuMemDmaUpFEMCloths(
			mNewFEMClothPool,
			mNewTriangleMeshByteSizePool.begin(),
			mNewFEMClothDataPool,
			mNewFEMClothNodeIndexPool,
			mNewFEMClothElementIndexPool,
			mFEMClothPool,
			mFEMClothDataPool,
			mFEMClothElementIndexPool,
			mFEMClothNodeIndexPool,
			mBodySimManager,
			mClothRigidAttachments.mAttachments,
			mClothRigidFilterPairs,
			mClothRigidAttachments.mAttachmentsDirty,
			mClothRigidAttachments.mActiveAttachments,
			mClothRigidAttachments.mActiveAttachmentsDirty,
			mClothClothAttachments.mAttachments,
			mClothClothVertTriFilterPairs,
			mClothClothAttachments.mAttachmentsDirty,
			mClothClothAttachments.mActiveAttachments,
			mClothClothAttachments.mActiveAttachmentsDirty
		);

		mClothRigidAttachments.mAttachmentsDirty = false;
		mClothRigidAttachments.mActiveAttachmentsDirty = false;
		mClothClothAttachments.mAttachmentsDirty = false;
		mClothClothAttachments.mActiveAttachmentsDirty = false;

		mFEMClothCore->preIntegrateSystems(nbActiveFEMCloths, gravity, dt);

		//PxRenderBuffer& renderBuffer = mNpContext->getContext().getRenderBuffer();
		if (mBroadPhase)
		{
			CUstream bpStream = mBroadPhase->getBpStream();
			mFEMClothCore->refitBound(nbActiveFEMCloths, bpStream);
		}

		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::updateShapes(PxBaseTask* continuation)
	{
		mPostCopyShapeSimTask.setContinuation(continuation);

		{
			PX_PROFILE_ZONE("GpuSimulationController.copyToGpuShapeSim", 0);
			mSimulationCore->mPxgShapeSimManager.copyToGpuShapeSim(mNpContext->getGpuNarrowphaseCore(), &mPostCopyShapeSimTask, mDynamicContext->getFlushPool());
		}

		mPostCopyShapeSimTask.removeReference();
	}

	//this is called before updateDynamic
	void PxgSimulationController::updateBodies(PxBaseTask* continuation)
	{
		PX_UNUSED(continuation);
		PX_PROFILE_ZONE("GpuSimulationController.updateBodiesAndShapes", 0);
		//mDynamicContext->getGpuSolverCore()->acquireContext();

		mPostCopyBodySimTask.setContinuation(continuation);

		copyToGpuBodySim(&mPostCopyBodySimTask);

		//mDynamicContext->getGpuSolverCore()->releaseContext();

		mPostCopyBodySimTask.removeReference();
	}

	void PxgSimulationController::preIntegrateAndUpdateBound(PxBaseTask* continuation, const PxVec3 gravity, const PxReal dt)
	{
		PX_PROFILE_ZONE("GpuSimulationController.integrateAndUpdateBoundParticleSystems", 0);

		mPostUpdateParticleSystemTask.setGravity(gravity);
		mPostUpdateParticleSystemTask.setDt(dt);

		mPostUpdateParticleSystemTask.setContinuation(continuation);

		//this fills the host mirrors for particle system
		copyToGpuParticleSystem(&mPostUpdateParticleSystemTask);

		//this fills the host mirrors for soft body
		copyToGpuSoftBody(&mPostUpdateParticleSystemTask);

		//this fills the host mirrors for FEM-cloth
		copyToGpuFEMCloth(&mPostUpdateParticleSystemTask);

		mPostUpdateParticleSystemTask.removeReference();
	}

	//This is called after integrateAndUpdateBoundParticleSystem
	void PxgSimulationController::updateParticleSystemsAndSoftBodies()
	{
		PX_PROFILE_ZONE("GpuSimulationController.updateParticleSystemsAndSoftBodies", 0);

		const PxU32 nbActiveSoftbodies = mBodySimManager.mActiveSoftbodies.size();		

		mCudaContextManager->acquireContext();

		//This make sure updateBounds kernel are finished in GPU

		if (mBodySimManager.mActivePBDParticleSystems.size() > 0)
		{
			mPBDParticleSystemCore->updateGrid();

			mPBDParticleSystemCore->resetContactCounts();

			mPBDParticleSystemCore->selfCollision();
		}

		if (mSoftBodyCore)
		{
			mSoftBodyCore->resetContactCounts();

			if (nbActiveSoftbodies > 0)
			{
				mSoftBodyCore->selfCollision();
			}
		}

		if (mFEMClothCore)
		{
			mFEMClothCore->resetClothVsNonclothContactCounts();

			// selfCollision called in mFEMClothCore->solve()
		}

		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::sortContacts()
	{
		const PxU32 nbActivePBDParticleSystems = mBodySimManager.mActivePBDParticleSystems.size();
		const PxU32 nbActiveSoftbodies = mBodySimManager.mActiveSoftbodies.size();
		const PxU32 nbActiveFemClothes = mBodySimManager.mActiveFEMCloths.size();

		mCudaContextManager->acquireContext();
		
		if (nbActivePBDParticleSystems > 0)
		{
			PX_PROFILE_ZONE("GpuSimulationController.sortParticleContacts", 0);
			mPBDParticleSystemCore->sortContacts(nbActivePBDParticleSystems);
			//KS - technically, we want this call to be in the kernel above but it currently blocks NP results gathering. Need to figure
			//out how to handle that better, e.g. by avoiding multiple syncs, using a NP stream etc.
			//mParticleSystemCore->selfCollision();
		}

		if (nbActiveSoftbodies > 0)
		{

			PX_PROFILE_ZONE("GpuSimulationController.sortSoftbodyContacts", 0);
			mSoftBodyCore->sortContacts(nbActiveSoftbodies);
		}

		if (nbActiveFemClothes > 0)
		{
			
			PX_PROFILE_ZONE("GpuSimulationController.sortClothContacts", 0);
			mFEMClothCore->sortContacts(nbActiveFemClothes);
		}

		mCudaContextManager->releaseContext();
	}

	//this is called after updateIncrementalIslands because island sim will add/remove joints based on the body activation
	void PxgSimulationController::updateJointsAndSyncData()
	{
		PX_PROFILE_ZONE("PxgSimulationController.updateGPUJoints", 0);
		mCudaContextManager->acquireContext();

		const PxInt32ArrayPinned& updatedRigidJointIndices = mJointManager.getDirtyGPURigidJointDataIndices();
		const PxPinnedArray<PxgD6JointData>& rigidJointData = mJointManager.getGpuRigidJointData();
		const PxPinnedArray<PxgConstraintPrePrep>& rigidJointPrePrep = mJointManager.getGpuRigidJointPrePrep();

		const PxInt32ArrayPinned& updatedArtiJointIndices = mJointManager.getDirtyGPUArtiJointDataIndices();
		const PxPinnedArray<PxgD6JointData>& artiJointData = mJointManager.getGpuArtiJointData();
		const PxPinnedArray<PxgConstraintPrePrep>& artiJointPrePrep = mJointManager.getGpuArtiJointPrePrep();

		mSimulationCore->updateJointsAndSyncData(rigidJointData, updatedRigidJointIndices, artiJointData, updatedArtiJointIndices, 
			rigidJointPrePrep, artiJointPrePrep, mJointManager.getGpuConstraintIdMapHost(), mJointManager.getAndClearConstraintIdMapDirtyFlag(),
			mJointManager.getGpuNbRigidConstraints(), mJointManager.getGpuNbArtiConstraints());

		mJointManager.reset();

		mCudaContextManager->releaseContext();
	}

	//This is called after solver integration gpu task
	void PxgSimulationController::update(PxBitMapPinned& changedHandleMap)
	{
		PX_PROFILE_ZONE("GpuSimulationController.update", 0);

		mCudaContextManager->acquireContext();

		const PxU32 nbTotalBodies = mBodySimManager.mTotalNumBodies;
		const PxU32 nbTotalShapes = mSimulationCore->mPxgShapeSimManager.getTotalNbShapes();

		mHasBeenSimulated = true;

		const bool enableDirectGPUAPI = mDynamicContext->getEnableDirectGPUAPI();
		mSimulationCore->gpuMemDmaUp(nbTotalBodies, nbTotalShapes, changedHandleMap, enableDirectGPUAPI);

		mSimulationCore->update(enableDirectGPUAPI);

		mCudaContextManager->releaseContext();
	}

#if PXG_SC_DEBUG
	void PxgSimulationController::validateCacheAndBounds(PxBoundsArrayPinned& boundArray, PxCachedTransformArrayPinned& cachedTransform)
	{

		for (PxU32 i = 0; i < mShapeSimManager.mTotalNumShapes; ++i)
		{
			PxsShapeSim& shapeSim = mShapeSimManager.mShapeSims[i];
			if (shapeSim.mBodySimIndex.index() != 0xffffffff)
			{

				PxBounds3& bound = boundArray[shapeSim.mElementIndex];
				if (!bound.isValid())
				{
					int bob = 0;
					PX_UNUSED(bob);
				}
				PX_ASSERT(bound.isValid());
				PxsCachedTransform& cache = cachedTransform[shapeSim.mElementIndex];
				PX_ASSERT(cache.transform.isSane());
				if (!cache.transform.isSane())
				{
					int bob = 0;
					PX_UNUSED(bob);
				}
			}
		}
	}
#endif

	void PxgSimulationController::mergeChangedAABBMgHandle()
	{
		PxScopedCudaLock _lock(*mCudaContextManager);
	
		mSimulationCore->mergeChangedAABBMgHandle();
	}

	void PxgSimulationController::gpuDmabackData(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBitMapPinned&  changedAABBMgrHandles,
		bool enableDirectGPUAPI)
	{
		if (mHasBeenSimulated)
		{
			mCudaContextManager->acquireContext();

			const PxU32 nbTotalBodies = mBodySimManager.mTotalNumBodies;
			const PxU32 nbTotalShapes = mSimulationCore->mPxgShapeSimManager.getTotalNbShapes();

			mFrozenPool.forceSize_Unsafe(0);
			mFrozenPool.reserve(nbTotalShapes);
			mFrozenPool.forceSize_Unsafe(nbTotalShapes);

			mUnfrozenPool.forceSize_Unsafe(0);
			mUnfrozenPool.reserve(nbTotalShapes);
			mUnfrozenPool.forceSize_Unsafe(nbTotalShapes);

			mActivatePool.forceSize_Unsafe(0);
			mActivatePool.reserve(nbTotalBodies);
			mActivatePool.forceSize_Unsafe(nbTotalBodies);

			mDeactivatePool.forceSize_Unsafe(0);
			mDeactivatePool.reserve(nbTotalBodies);
			mDeactivatePool.forceSize_Unsafe(nbTotalBodies);

			PxCachedTransformArrayPinned* cachedTransform = cache.getCachedTransformArray();

#if PXG_SC_DEBUG
			validateCacheAndBounds(bounds, *cachedTransform);
#endif // PXG_SC_BEBUG

			mSimulationCore->gpuMemDmaBack(mFrozenPool, mUnfrozenPool, mActivatePool, mDeactivatePool, cachedTransform, cache.getTotalSize(), boundArray, changedAABBMgrHandles, nbTotalShapes,
				nbTotalBodies, enableDirectGPUAPI);


			mCudaContextManager->releaseContext();
		}
	}

	void PxgSimulationController::updateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation)
	{
		PX_UNUSED(cache);
		PX_UNUSED(boundArray);

		PX_PROFILE_ZONE("GpuSimulationController.updateScBodyAndShapeSim", 0);

		mCudaContextManager->acquireContext();
		mSimulationCore->syncDmaback(mNbFrozenShapes, mNbUnfrozenShapes, mHasBeenSimulated);
		if (mFEMClothCore)
			mFEMClothCore->syncCloths();
		if (mSoftBodyCore)
			mSoftBodyCore->syncSoftBodies();

		if (mFEMClothCore)
			mFEMClothCore->checkBufferOverflows();
		if (mSoftBodyCore)
			mSoftBodyCore->checkBufferOverflows();

		if (mHasBeenSimulated)
		{
#if PXG_SC_DEBUG
			validateCacheAndBounds(boundArray.getBounds(), *cache.getCachedTransformArray());
#endif // PXG_SC_BEBUG
			if ( (!mDynamicContext->getEnableDirectGPUAPI()) || mEnableOVDReadback)
			{
				mCallback->updateScBodyAndShapeSim(continuation);
				//Now update sleep state for FEM cloth...
				if (mFEMClothCore)
					mFEMClothCore->createActivatedDeactivatedLists();
				if (mSoftBodyCore)
					mSoftBodyCore->createActivatedDeactivatedLists();
			}
			mHasBeenSimulated = false;
		}
		else
		{
			clear();
		}

		mCudaContextManager->releaseContext();
	}

	PxU32* PxgSimulationController::getActiveBodies()
	{
		return mActivatePool.begin();
	}

	PxU32* PxgSimulationController::getDeactiveBodies()
	{
		return mDeactivatePool.begin();
	}

	void** PxgSimulationController::getRigidBodies()
	{
		return mBodySimManager.mBodies.begin();
	}

	PxU32 PxgSimulationController::getNbBodies()
	{
		return mBodySimManager.mTotalNumBodies;
	}

	PxU32* PxgSimulationController::getUnfrozenShapes()
	{
		return mUnfrozenPool.begin();
	}

	PxU32* PxgSimulationController::getFrozenShapes()
	{
		return mFrozenPool.begin();
	}

	Sc::ShapeSimBase** PxgSimulationController::getShapeSims()
	{
		return mSimulationCore->mPxgShapeSimManager.getShapeSims();
	}

	PxU32 PxgSimulationController::getNbFrozenShapes()
	{
		return mNbFrozenShapes;
	}

	PxU32 PxgSimulationController::getNbUnfrozenShapes()
	{
		return mNbUnfrozenShapes;
	}

	PxU32 PxgSimulationController::getNbShapes()
	{
		return mSimulationCore->mPxgShapeSimManager.getTotalNbShapes();
	}

	void PxgSimulationController::setBounds(Bp::BoundsArray* boundArray)
	{
		mSimulationCore->setBounds(boundArray);
	}

	void PxgPostCopyToShapeSimTask::runInternal()
	{
		mController.postCopyToShapeSim();
	}

	void PxgPostCopyToBodySimTask::runInternal()
	{
		mController.postCopyToBodySim(mEnableBodyAccelerations);
	}

	void PxgPostUpdateParticleAndSoftBodyTask::runInternal()
	{
		mController.preIntegrateAndUpdateBoundParticleSystem(mGravity, mDt);
		mController.preIntegrateAndUpdateBoundSoftBody(mGravity, mDt);
		mController.preIntegrateAndUpdateBoundFEMCloth(mGravity, mDt);
	}

	void PxgCopyToArticulationSimTask::runInternal()
	{
		mController.copyToGpuArticulationSim(mNewBodySimOffset, mStartIndex, mNbToProcess,
			mSharedArticulationLinksIndex, mSharedDofIndex, mSharedSpatialTendonIndex,
			mSharedSpatialTendonAttachmentIndex, mSharedFixedTendonIndex, mSharedFixedTendonJointIndex,
			mSharedArticulationMimicJointIndex, mSharedPathToRootIndex);
	}

	void PxgUpdateArticulationSimTask::runInternal()
	{
		mController.updateGpuArticulationSim(mStartIndex, mNbToProcess, mSharedArticulationLinksIndex,
			mSharedArticulationDofIndex, mSharedSpatialTendonIndex, mSharedSpatialTendonAttachmentIndex,
			mSharedFixedTendonIndex, mSharedFixedTendonJointIndex, mSharedMimicJointIndex);
	}

	void PxgCopyToSoftBodySimTask::runInternal()
	{
		mController.copyToGpuSoftBodySim(mStartIndex, mNbToProcess);
	}

	void PxgCopyToFEMClothSimTask::runInternal()
	{
		mController.copyToGpuFEMClothSim(mStartIndex, mNbToProcess);
	}

	void PxgCopyToPBDParticleSystemSimTask::runInternal()
	{
		mController.copyToGpuPBDParticleSystemSim(mStartIndex, mNbToProcess);
	}

	void PxgCopyToBodySimTask::runInternal()
	{
		mController.copyToGpuBodySim(mNewBodySimOffset, mStartIndex, mNbToProcess);
	}

	PxU32 PxgSimulationController::addRigidAttachmentInternal(const PxU32 nonRigidId, const PxU32 elemId, const bool isVertex, const PxVec4& barycentric, PxsRigidBody* rigidBody,
		const PxNodeIndex& rigidNodeIndex, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint, AttachmentManager<PxgFEMRigidAttachment>& attachments, bool addToActive)
	{
		PX_ASSERT(isVertex == barycentric.isZero());

		PxgFEMRigidAttachment attachment;
		const PxVec3 localPose = rigidBody ? rigidBody->getCore().getBody2Actor().transformInv(actorSpacePose) : actorSpacePose;
		attachment.localPose0 = make_float4(localPose.x, localPose.y, localPose.z, 0.f);
		attachment.index1 = PxEncodeSoftBodyIndex(nonRigidId, elemId);
		PxU32 handle = attachments.mBaseHandle++;
		attachment.index2 = handle;
		attachment.index0 = reinterpret_cast<const PxU64&>(rigidNodeIndex);
		attachment.baryOrType1.x = isVertex ? 0.0f : barycentric.x;
		attachment.baryOrType1.y = isVertex ? 0.0f : barycentric.y;
		attachment.baryOrType1.z = isVertex ? 0.0f : barycentric.z;
		attachment.baryOrType1.w = isVertex ? 0.0f : barycentric.w;

		if (constraint)
		{
			attachment.coneLimitParams.low_high_limits = make_float4(constraint->mLowLimit, constraint->mHighLimit, 0.f, 0.f);
			attachment.coneLimitParams.axis_angle = make_float4(constraint->mAxis.x, constraint->mAxis.y, constraint->mAxis.z, constraint->mAngle);
		}
		else
		{
			attachment.coneLimitParams.low_high_limits = make_float4(0.f, 0.f, 0.f, 0.f);
			attachment.coneLimitParams.axis_angle = make_float4(0.f, 0.f, 0.f, -1.f);
		}

		attachments.addAttachment(attachment, handle);
		if (addToActive)
			attachments.activateAttachment(handle);

		return handle;
	}

	template<class FilterPair, class FilterPairArray>
	void addFilterPairInternal(FilterPair& pair, FilterPairArray& filterPairs,
		PxArray<PxU32>& references, bool& dirtyFlag)
	{
		PxI32 left = 0, right = filterPairs.size();
		while (left < right)
		{
			PxI32 mid = (left + right) / 2;
			PxI32 cmp = pair.compare(filterPairs[mid]);
			if (cmp < 1)
				right = mid;
			else
				left = mid + 1;
		}

		if (PxU32(left) == filterPairs.size() || filterPairs[left].compare(pair) != 0)
		{
			filterPairs.insert();
			references.insert();

			for (PxI32 j = filterPairs.size() - 1; j > left; --j)
			{
				filterPairs[j] = filterPairs[j - 1];
				references[j] = references[j - 1];
			}

			filterPairs[left] = pair;
			references[left] = 1;
			dirtyFlag = true;
		}
		else
			references[left]++;
	}

	template<class FilterPair, class FilterPairArray>
	void releaseFilterPairInternal(FilterPair& pair, FilterPairArray& filterPairs,
		PxArray<PxU32>& references, bool& dirtyFlag, PxU32 refCount = 1)
	{

		PxI32 left = 0, right = filterPairs.size();
		while (left < right)
		{
			PxI32 mid = (left + right) / 2;
			PxI32 cmp = pair.compare(filterPairs[mid]);
			if (cmp < 0)
				right = mid - 1;
			else if (cmp > 0)
				left = mid + 1;
			else
			{
				left = right = mid;
				break;
			}
		}

		if (PxU32(left) < filterPairs.size() && filterPairs[left].compare(pair) == 0)
		{
			if (references[left] == refCount)
			{
				//Found so we remove...
				for (PxI32 j = left, size = PxI32(filterPairs.size() - 1); j < size; ++j)
				{
					filterPairs[j] = filterPairs[j + 1];
					references[j] = references[j + 1];
				}
				filterPairs.forceSize_Unsafe(filterPairs.size() - 1);
				references.forceSize_Unsafe(references.size() - 1);
				dirtyFlag = true;
			}
			else
				references[left] -= refCount;
		}
	}

	void PxgSimulationController::computeSoftBodySimMeshData(Dy::DeformableVolume* deformableVolume, PxU32 tetId, const PxVec4& tetBarycentric, PxU32& outTetId, PxVec4& outTetBarycentric)
	{
		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		const Gu::TetrahedronMesh* simMesh = static_cast<const Gu::TetrahedronMesh*>(deformableVolume->getSimulationMesh());
		const Gu::DeformableVolumeAuxData* simulationState = static_cast<const Gu::DeformableVolumeAuxData*>(deformableVolume->getAuxData());

		Gu::convertDeformableVolumeCollisionToSimMeshTets(*simMesh, *simulationState, *tetMesh, tetId, tetBarycentric, outTetId, outTetBarycentric, false);
	}

	void PxgSimulationController::addTetRigidFilter(Dy::DeformableVolume* deformableVolume,
		const PxNodeIndex& rigidNodeIndex, PxU32 tetId)
	{
		if (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			return;
		}

		PxU32 softBodyId = deformableVolume->getGpuRemapId();

		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId = (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId : static_cast<PxU32*>(tetMesh->mGRB_faceRemapInverse)[tetId];

		PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetId); //soft body and tet index

		PxgRigidFilterPair pair;
		pair.index1 = tetHandle;
		pair.index0 = reinterpret_cast<const PxU64&>(rigidNodeIndex);

		addFilterPairInternal(pair, mSoftBodyRigidFilterPairs, mSoftBodyRigidFilterRefs, mRigidSoftBodyAttachments.mAttachmentsDirty);
	}

	void PxgSimulationController::removeTetRigidFilter(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& rigidNodeIndex,
		PxU32 tetId)
	{
		if (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			return;
		}

		PxU32 softBodyId = deformableVolume->getGpuRemapId();
		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId = (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId : static_cast<PxU32*>(tetMesh->mGRB_faceRemapInverse)[tetId];

		PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetId); //soft body and tet index

		PxgRigidFilterPair pair;
		pair.index1 = tetHandle;
		pair.index0 = reinterpret_cast<const PxU64&>(rigidNodeIndex);
		releaseFilterPairInternal(pair, mSoftBodyRigidFilterPairs, mSoftBodyRigidFilterRefs, mRigidSoftBodyAttachments.mAttachmentsDirty);
	}

	PxU32 PxgSimulationController::addTetRigidAttachment(Dy::DeformableVolume* deformableVolume, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 tetId, const PxVec4& barycentric,
		const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint, const bool isActive, bool doConversion)
	{
		PxU32 simTetIdx = tetId;
		PxVec4 simBarycentric = barycentric;
		if (doConversion)
		{
			computeSoftBodySimMeshData(deformableVolume, tetId, barycentric, simTetIdx, simBarycentric);
		}

		const bool isVertex = false;
		PxU32 handle = addRigidAttachmentInternal(deformableVolume->getGpuRemapId(), simTetIdx, isVertex, simBarycentric, rigidBody, rigidNodeIndex, actorSpacePose, constraint, mRigidSoftBodyAttachments, isActive);
		deformableVolume->mRigidVolumeAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::addSoftBodyFilter(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32 tetId0,
		PxU32 tetId1)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(Dy::VolumeVolumeFilter) == sizeof(PxgNonRigidFilterPair));
		const PxU32 softBodyId0 = deformableVolume0->getGpuRemapId();
		const PxU32 softBodyId1 = deformableVolume1->getGpuRemapId();

		if (deformableVolume0->mVolumeVolumeFilterPairs == NULL)
		{
			deformableVolume0->mVolumeVolumeFilterPairs = PX_PLACEMENT_NEW(PX_ALLOCATE(Dy::VolumeVolumeFilterArray, 1, "VolumeVolumeFilterArray"), Dy::VolumeVolumeFilterArray)(mHeapMemoryManager->mMappedMemoryAllocators);
		
			deformableVolume0->mDirtyVolumeForFilterPairs = &mDirtyDeformableVolumeForFilterPairs;
		}

		if (deformableVolume1->mVolumeVolumeFilterPairs == NULL)
		{
			deformableVolume1->mVolumeVolumeFilterPairs = PX_PLACEMENT_NEW(PX_ALLOCATE(Dy::VolumeVolumeFilterArray, 1, "VolumeVolumeFilterArray"), Dy::VolumeVolumeFilterArray)(mHeapMemoryManager->mMappedMemoryAllocators);
		
			deformableVolume1->mDirtyVolumeForFilterPairs = &mDirtyDeformableVolumeForFilterPairs;
		}

		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh0 = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume0->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId0 = (tetId0 == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId0: static_cast<PxU32*>(tetMesh0->mGRB_faceRemapInverse)[tetId0];

		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh1 = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume1->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId1 = (tetId1 == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId1 : static_cast<PxU32*>(tetMesh1->mGRB_faceRemapInverse)[tetId1];

		const PxU32 colTetHandle0 = PxEncodeSoftBodyIndex(softBodyId0, tetId0);
		const PxU32 colTetHandle1 = PxEncodeSoftBodyIndex(softBodyId1, tetId1);
		
		if (tetId0 != PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			uint4* collInds0 = reinterpret_cast<uint4*>(tetMesh0->mGRB_tetraIndices);
			PxPinnedArray<PxgNonRigidFilterPair>* softBody0FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume0->mVolumeVolumeFilterPairs);
			uint4 ind0 = collInds0[tetId0];
			int* index0 = reinterpret_cast<int*>(&ind0);
			for (PxU32 i = 0; i < 4; ++i)
			{
				const PxU32 colTetVertHandle = PxEncodeSoftBodyIndex(softBodyId0, index0[i]);
				//create attachment pair for collision filtering
				PxgNonRigidFilterPair pair;
				pair.index0 = colTetVertHandle;
				pair.index1 = colTetHandle1;

				//printf("CPU softbodyId0 %i vertIdx %i softbodyId1 %i, primitiveIdx %i\n", softBodyId0, index0[i], softBodyId1, tetId1);
				addFilterPairInternal(pair, *softBody0FilterPairs, deformableVolume0->mVolumeVolumeAttachmentIdReferences, deformableVolume0->mFilterDirty);
			}
		}

		if (tetId1 != PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			uint4* collInds1 = reinterpret_cast<uint4*>(tetMesh1->mGRB_tetraIndices);
			PxPinnedArray<PxgNonRigidFilterPair>* softBody1FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume1->mVolumeVolumeFilterPairs);
			uint4 ind1 = collInds1[tetId1];
			int* index1 = reinterpret_cast<int*>(&ind1);
			for (PxU32 i = 0; i < 4; ++i)
			{
				const PxU32 colTetVertHandle = PxEncodeSoftBodyIndex(softBodyId1, index1[i]);
				//create attachment pair for collision filtering
				PxgNonRigidFilterPair pair;
				pair.index0 = colTetVertHandle;
				pair.index1 = colTetHandle0;

				//printf("CPU softbodyId1 %i vertIdx %i softbodyId0 %i, primitiveIdx %i\n", softBodyId1, index1[i], softBodyId0, tetId0);
				addFilterPairInternal(pair, *softBody1FilterPairs, deformableVolume1->mVolumeVolumeAttachmentIdReferences, deformableVolume1->mFilterDirty);
			}
		}

		if (deformableVolume0->mFilterDirty && (!deformableVolume0->mFilterInDirtyList))
		{
			mDirtyDeformableVolumeForFilterPairs.pushBack(deformableVolume0);
			deformableVolume0->mFilterInDirtyList = true;
		}

		if (deformableVolume1->mFilterDirty && (!deformableVolume1->mFilterInDirtyList))
		{
			mDirtyDeformableVolumeForFilterPairs.pushBack(deformableVolume1);
			deformableVolume1->mFilterInDirtyList = true;
		}
	}

	void PxgSimulationController::removeSoftBodyFilter(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32 tetId0, PxU32 tetId1)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(Dy::VolumeVolumeFilter) == sizeof(PxgNonRigidFilterPair));
		PxU32 softBodyId0 = deformableVolume0->getGpuRemapId();
		
		PxU32 softBodyId1 = deformableVolume1->getGpuRemapId();
		
		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh0 = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume0->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId0 = (tetId0 == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId0 : static_cast<PxU32*>(tetMesh0->mGRB_faceRemapInverse)[tetId0];

		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh1 = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume1->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId1 = (tetId1 == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId1 : static_cast<PxU32*>(tetMesh1->mGRB_faceRemapInverse)[tetId1];
		
		const PxU32 colTetHandle0 = PxEncodeSoftBodyIndex(softBodyId0, tetId0);
		const PxU32 colTetHandle1 = PxEncodeSoftBodyIndex(softBodyId1, tetId1);

		if (tetId0 != PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			uint4* collInds0 = reinterpret_cast<uint4*>(tetMesh0->mGRB_tetraIndices);

			PxPinnedArray<PxgNonRigidFilterPair>* softBody0FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume0->mVolumeVolumeFilterPairs);
			uint4 ind0 = collInds0[tetId0];

			int* index0 = reinterpret_cast<int*>(&ind0);
			for (PxU32 i = 0; i < 4; ++i)
			{
				PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId0, index0[i]);

				PxgNonRigidFilterPair pair;
				pair.index0 = tetHandle;
				pair.index1 = colTetHandle1;

				releaseFilterPairInternal(pair, *softBody0FilterPairs, deformableVolume0->mVolumeVolumeAttachmentIdReferences, deformableVolume0->mFilterDirty);
			}
		}

		if (tetId1 != PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			uint4* collInds1 = reinterpret_cast<uint4*>(tetMesh1->mGRB_tetraIndices);
			PxPinnedArray<PxgNonRigidFilterPair>* softBody1FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume1->mVolumeVolumeFilterPairs);
			uint4 ind1 = collInds1[tetId1];
			int* index1 = reinterpret_cast<int*>(&ind1);
			for (PxU32 i = 0; i < 4; ++i)
			{
				PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId1, index1[i]);

				PxgNonRigidFilterPair pair;
				pair.index0 = tetHandle;
				pair.index1 = colTetHandle0;

				releaseFilterPairInternal(pair, *softBody1FilterPairs, deformableVolume1->mVolumeVolumeAttachmentIdReferences, deformableVolume1->mFilterDirty);
			}
		}

		if (deformableVolume0->mFilterDirty && (!deformableVolume0->mFilterInDirtyList))
		{
			mDirtyDeformableVolumeForFilterPairs.pushBack(deformableVolume0);
			deformableVolume0->mFilterInDirtyList = true;
		}

		if (deformableVolume1->mFilterDirty && (!deformableVolume1->mFilterInDirtyList))
		{
			mDirtyDeformableVolumeForFilterPairs.pushBack(deformableVolume1);
			deformableVolume1->mFilterInDirtyList = true;
		}
	}

	struct SortPxgNonRigidFilterPair
	{
		bool operator()(const PxgNonRigidFilterPair& first, const PxgNonRigidFilterPair& second) const
		{
			if (((const PxgNonRigidFilterPair&)first).compare(second) < 0)
				return true;
			else
				return false;
		}
	};

	void PxgSimulationController::addSoftBodyFiltersInternal(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices, PxU32 size)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(Dy::VolumeVolumeFilter) == sizeof(PxgNonRigidFilterPair));
		const PxU32 softBodyId0 = deformableVolume0->getGpuRemapId();
		const PxU32 softBodyId1 = deformableVolume1->getGpuRemapId();
		const PxU32 colTetHandle1 = PxEncodeSoftBodyIndex(softBodyId1, PX_MAX_NB_DEFORMABLE_VOLUME_TET);

		if (deformableVolume0->mVolumeVolumeFilterPairs == NULL)
		{
			deformableVolume0->mVolumeVolumeFilterPairs = PX_PLACEMENT_NEW(PX_ALLOCATE(Dy::VolumeVolumeFilterArray, 1, "VolumeVolumeFilterArray"), Dy::VolumeVolumeFilterArray)(mHeapMemoryManager->mMappedMemoryAllocators);

			deformableVolume0->mDirtyVolumeForFilterPairs = &mDirtyDeformableVolumeForFilterPairs;

			PxPinnedArray<PxgNonRigidFilterPair>* softBody0FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume0->mVolumeVolumeFilterPairs);
			softBody0FilterPairs->reserve(size * 4);
			deformableVolume0->mVolumeVolumeAttachmentIdReferences.reserve(size * 4);
		}
		else
		{
			PxPinnedArray<PxgNonRigidFilterPair>* softBody0FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume0->mVolumeVolumeFilterPairs);
			PxU32 maxSize = softBody0FilterPairs->size() + (size * 4);
			if (softBody0FilterPairs->capacity() < maxSize)
			{
				softBody0FilterPairs->reserve(maxSize);
				deformableVolume0->mVolumeVolumeAttachmentIdReferences.reserve(maxSize);
			}
		}

		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh0 = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume0->getCollisionMesh());
		uint4* collInds0 = reinterpret_cast<uint4*>(tetMesh0->mGRB_tetraIndices);
		PxPinnedArray<PxgNonRigidFilterPair>* softBody0FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume0->mVolumeVolumeFilterPairs);

		PxArray<PxgNonRigidFilterPair> sortList;
		sortList.reserve(softBody0FilterPairs->capacity() * 4);

		for (PxU32 i = 0; i < softBody0FilterPairs->size(); ++i)
		{
			for (PxU32 j = 0; j < deformableVolume0->mVolumeVolumeAttachmentIdReferences[i]; ++j)
				sortList.pushBack((*softBody0FilterPairs)[i]);
		}

		softBody0FilterPairs->clear();
		deformableVolume0->mVolumeVolumeAttachmentIdReferences.clear();

		for (PxU32 iter = 0; iter < size; ++iter)
		{
			PxU32 tetId0 = tetIndices[iter];

			//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
			tetId0 = (tetId0 == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId0 : static_cast<PxU32*>(tetMesh0->mGRB_faceRemapInverse)[tetId0];

			if (tetId0 != PX_MAX_NB_DEFORMABLE_VOLUME_TET)
			{
				uint4 ind0 = collInds0[tetId0];
				int* index0 = reinterpret_cast<int*>(&ind0);
				for (PxU32 i = 0; i < 4; ++i)
				{
					const PxU32 colTetVertHandle = PxEncodeSoftBodyIndex(softBodyId0, index0[i]);
					//create attachment pair for collision filtering
					PxgNonRigidFilterPair pair;
					pair.index0 = colTetVertHandle;
					pair.index1 = colTetHandle1;

					sortList.pushBack(pair);
				}
			}
		}

		if (sortList.size())
		{
			SortPxgNonRigidFilterPair predicate;
			PxSort(sortList.begin(), sortList.size(), predicate);

			deformableVolume0->mFilterDirty = true;
			mDirtyDeformableVolumeForFilterPairs.pushBack(deformableVolume0);
			deformableVolume0->mFilterInDirtyList = true;
		}

		for (PxU32 i = 0; i < sortList.size(); ++i)
		{
			PxU32 refCount = 1;
			softBody0FilterPairs->pushBack(sortList[i]);
		
			while (i < (sortList.size() - 1) && sortList[i].compare(sortList[i + 1]) == 0)
			{
				refCount++;
				i++;
			}
			deformableVolume0->mVolumeVolumeAttachmentIdReferences.pushBack(refCount);
		}
	}

	void PxgSimulationController::addSoftBodyFilters(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
	{
		PxArray<unsigned int> tetraFilter0Ids;
		PxArray<unsigned int> tetraFilter1Ids;
		tetraFilter0Ids.reserve(tetIndicesSize);
		tetraFilter1Ids.reserve(tetIndicesSize);

		for (size_t i = 0; i < tetIndicesSize; i++)
		{
			PxU32 tetId0 = tetIndices0[i];
			PxU32 tetId1 = tetIndices1[i];

			if (tetId1 == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
			{
				tetraFilter0Ids.pushBack(tetId0);
			}
			else if (tetId0 == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
			{
				tetraFilter1Ids.pushBack(tetId1);
			}
			else
			{
				addSoftBodyFilter(deformableVolume0, deformableVolume1, tetId0, tetId1);
			}
		}

		if (!tetraFilter0Ids.empty())
		{
			addSoftBodyFiltersInternal(deformableVolume0, deformableVolume1, &tetraFilter0Ids[0], tetraFilter0Ids.size());
		}

		if (!tetraFilter1Ids.empty())
		{
			addSoftBodyFiltersInternal(deformableVolume1, deformableVolume0, &tetraFilter1Ids[0], tetraFilter1Ids.size());
		}
	}

	void PxgSimulationController::removeSoftBodyFiltersInternal(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices, PxU32 size)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(Dy::VolumeVolumeFilter) == sizeof(PxgNonRigidFilterPair));
		const PxU32 softBodyId0 = deformableVolume0->getGpuRemapId();
		const PxU32 softBodyId1 = deformableVolume1->getGpuRemapId();
		const PxU32 colTetHandle1 = PxEncodeSoftBodyIndex(softBodyId1, PX_MAX_NB_DEFORMABLE_VOLUME_TET);

		//Convert from coll mesh to sim mesh now...
		const Gu::BVTetrahedronMesh* tetMesh0 = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume0->getCollisionMesh());
		uint4* collInds0 = reinterpret_cast<uint4*>(tetMesh0->mGRB_tetraIndices);
		PxPinnedArray<PxgNonRigidFilterPair>* softBody0FilterPairs = reinterpret_cast<PxPinnedArray<PxgNonRigidFilterPair>*>(deformableVolume0->mVolumeVolumeFilterPairs);

		PxArray<PxgNonRigidFilterPair> sortList;
		sortList.reserve(softBody0FilterPairs->capacity() * 4);

		for (PxU32 iter = 0; iter < size; ++iter)
		{
			PxU32 tetId0 = tetIndices[iter];

			//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
			tetId0 = (tetId0 == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId0 : static_cast<PxU32*>(tetMesh0->mGRB_faceRemapInverse)[tetId0];

			if (tetId0 != PX_MAX_NB_DEFORMABLE_VOLUME_TET)
			{
				uint4 ind0 = collInds0[tetId0];
				int* index0 = reinterpret_cast<int*>(&ind0);
				for (PxU32 i = 0; i < 4; ++i)
				{
					const PxU32 colTetVertHandle = PxEncodeSoftBodyIndex(softBodyId0, index0[i]);
					//create attachment pair for collision filtering
					PxgNonRigidFilterPair pair;
					pair.index0 = colTetVertHandle;
					pair.index1 = colTetHandle1;

					sortList.pushBack(pair);
				}
			}
		}

		if (sortList.size())
		{
			SortPxgNonRigidFilterPair predicate;
			PxSort(sortList.begin(), sortList.size(), predicate);
		}

		PxArray<PxgNonRigidFilterPair> removeList;
		PxArray<PxU32> refCountList;

		removeList.reserve(softBody0FilterPairs->capacity());
		refCountList.reserve(softBody0FilterPairs->capacity());

		for (PxU32 i = 0; i < sortList.size(); ++i)
		{
			PxU32 refCount = 1;
			removeList.pushBack(sortList[i]);

			while (i < (sortList.size() - 1) && sortList[i].compare(sortList[i + 1]) == 0)
			{
				refCount++;
				i++;
			}
			refCountList.pushBack(refCount);
		}

		for (PxI32 i = removeList.size() - 1; i >= 0; --i)
		{
			releaseFilterPairInternal(removeList[i], *softBody0FilterPairs, deformableVolume0->mVolumeVolumeAttachmentIdReferences, deformableVolume0->mFilterDirty, refCountList[i]);
		}

		if (deformableVolume0->mFilterDirty && (!deformableVolume0->mFilterInDirtyList))
		{
			mDirtyDeformableVolumeForFilterPairs.pushBack(deformableVolume0);
			deformableVolume0->mFilterInDirtyList = true;
		}
	}

	void PxgSimulationController::removeSoftBodyFilters(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
	{
		PxArray<unsigned int> tetraFilter0Ids;
		PxArray<unsigned int> tetraFilter1Ids;
		tetraFilter0Ids.reserve(tetIndicesSize);
		tetraFilter1Ids.reserve(tetIndicesSize);

		for (size_t i = 0; i < tetIndicesSize; i++)
		{
			PxU32 tetId0 = tetIndices0[i];
			PxU32 tetId1 = tetIndices1[i];

			if (tetId1 == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
			{
				tetraFilter0Ids.pushBack(tetId0);
			}
			else if (tetId0 == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
			{
				tetraFilter1Ids.pushBack(tetId1);
			}
			else
			{
				removeSoftBodyFilter(deformableVolume0, deformableVolume1, tetId0, tetId1);
			}
		}

		if (!tetraFilter0Ids.empty())
		{
			removeSoftBodyFiltersInternal(deformableVolume0, deformableVolume1, &tetraFilter0Ids[0], tetraFilter0Ids.size());
		}

		if (!tetraFilter1Ids.empty())
		{
			removeSoftBodyFiltersInternal(deformableVolume1, deformableVolume0, &tetraFilter1Ids[0], tetraFilter1Ids.size());
		}
	}

	PxU32 PxgSimulationController::addSoftBodyAttachment(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32 tetId0, PxU32 tetId1,
		const PxVec4& tetBarycentric0, const PxVec4& tetBarycentric1, PxConeLimitedConstraint* constraint, PxReal constraintOffset, const bool addToActive, bool doConversion)
	{
		// Convert from coll mesh to sim mesh now...

		PxU32 outTetIdx0 = 0xFFFFFFFF;
		PxVec4 outBarycentric0;

		if (doConversion)
		{
			computeSoftBodySimMeshData(deformableVolume0, tetId0, tetBarycentric0, outTetIdx0, outBarycentric0);
		}
		else
		{
			outTetIdx0 = tetId0;
			outBarycentric0 = tetBarycentric0;
		}

		PxU32 outTetIdx1 = 0xFFFFFFFF;
		PxVec4 outBarycentric1;

		if (doConversion)
		{
			computeSoftBodySimMeshData(deformableVolume1, tetId1, tetBarycentric1, outTetIdx1, outBarycentric1);
		}
		else
		{
			outTetIdx1 = tetId1;
			outBarycentric1 = tetBarycentric1;
		}

		//create attachment
		PxgFEMFEMAttachment attachment;

		PxU32 softBodyId0 = deformableVolume0->getGpuRemapId();
		PxU32 tetHandle0 = PxEncodeSoftBodyIndex(softBodyId0, outTetIdx0);

		PxU32 softBodyId1 = deformableVolume1->getGpuRemapId();
		PxU32 tetHandle1 = PxEncodeSoftBodyIndex(softBodyId1, outTetIdx1);

		attachment.index0 = tetHandle0;
		attachment.index1 = tetHandle1;
		attachment.constraintOffset = constraintOffset;
		PxU32 handle = mSoftBodySoftBodyAttachments.mBaseHandle++;

		attachment.barycentricCoordinates0 = make_float4(outBarycentric0.x, outBarycentric0.y, outBarycentric0.z, outBarycentric0.w);
		attachment.barycentricCoordinates1 = make_float4(outBarycentric1.x, outBarycentric1.y, outBarycentric1.z, outBarycentric1.w);

		//for solve body, user define axis in world space. We need to generate a point based on the direction and compute the barycentric for that.
		if (constraint)
		{
			attachment.coneLimitParams.low_high_angle = make_float4(constraint->mLowLimit, constraint->mHighLimit, constraint->mAngle, 0.f);

			if (constraint->mAngle >= 0.f)
			{
				const Gu::TetrahedronMesh* simMesh = static_cast<const Gu::TetrahedronMesh*>(deformableVolume0->getSimulationMesh());
				PxVec4 barycentric = Gu::addAxisToSimMeshBarycentric(*simMesh, outTetIdx0, outBarycentric0, constraint->mAxis.getNormalized());
				attachment.coneLimitParams.barycentric = make_float4(barycentric.x, barycentric.y, barycentric.z, barycentric.w);
			}
			else
			{
				attachment.coneLimitParams.barycentric = make_float4(0.f, 0.f, 0.f, 0.f);
			}
		}
		else
		{
			attachment.coneLimitParams.low_high_angle = make_float4(0.f, 0.f, -1.f, 0.f);
			attachment.coneLimitParams.barycentric = make_float4(0.f, 0.f, 0.f, 0.f);
		}
		//These are used for activate and deactivate soft body
		mSoftBodySoftBodyAttachments.addAttachment(attachment, handle);
		if (addToActive)
			mSoftBodySoftBodyAttachments.activateAttachment(handle);
		
		deformableVolume0->mVolumeVolumeAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeSoftBodyAttachment(Dy::DeformableVolume* deformableVolume0, PxU32 handle)
	{
		if (mSoftBodySoftBodyAttachments.removeAttachment(handle))
			deformableVolume0->mVolumeVolumeAttachments.findAndReplaceWithLast(handle);
	}

	void PxgSimulationController::addClothFilter(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface, PxU32 triIdx, PxU32 tetId)
	{
		if (triIdx == PX_MAX_NB_DEFORMABLE_SURFACE_TRI && tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			return;
		}
		PxU32 softBodyId = deformableVolume->getGpuRemapId();
		PxU32 clothId = deformableSurface->getGpuRemapId();
		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId = (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId : static_cast<PxU32*>(tetMesh->mGRB_faceRemapInverse)[tetId];

		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx = (triIdx == PX_MAX_NB_DEFORMABLE_SURFACE_TRI) ? triIdx : static_cast<PxU32*>(triangleMesh->mGRB_faceRemapInverse)[triIdx];

		const PxU32 softBodyTetHandle = PxEncodeSoftBodyIndex(softBodyId, tetId);

		if (triIdx != PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxU32 triVertInd[3];
			getGRBVertexIndices(triVertInd[0], triVertInd[1], triVertInd[2], triIdx, triangleMesh);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxU32 clothTriVertHandle = PxEncodeClothIndex(clothId, triVertInd[i]);
				PxgNonRigidFilterPair tetVertPair;
				tetVertPair.index0 = softBodyTetHandle;
				tetVertPair.index1 = clothTriVertHandle;
				addFilterPairInternal(tetVertPair, mSoftBodyClothTetVertFilterPairs, mSoftBodyClothTetVertFilterRefs, mSoftBodyClothAttachments.mAttachmentsDirty);
			}
		}
		else
		{
			const PxU32 clothVertFullMask = PxEncodeClothIndex(clothId, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
			PxgNonRigidFilterPair tetVertPair;
			tetVertPair.index0 = softBodyTetHandle;
			tetVertPair.index1 = clothVertFullMask;
			addFilterPairInternal(tetVertPair, mSoftBodyClothTetVertFilterPairs, mSoftBodyClothTetVertFilterRefs, mSoftBodyClothAttachments.mAttachmentsDirty);
		}
	}

	void PxgSimulationController::removeClothFilter(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface, PxU32 triId, PxU32 tetId)
	{
		if (triId == PX_MAX_NB_DEFORMABLE_SURFACE_TRI && tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			return;
		}
		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId = (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET) ? tetId : static_cast<PxU32*>(tetMesh->mGRB_faceRemapInverse)[tetId];

		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		//PxU32 originalTriIdx = triIdx;
		triId = (triId == PX_MAX_NB_DEFORMABLE_SURFACE_TRI) ? triId : static_cast<PxU32*>(triangleMesh->mGRB_faceRemapInverse)[triId];

		PxU32 softBodyId = deformableVolume->getGpuRemapId();
		PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetId);
		PxU32 clothId = deformableSurface->getGpuRemapId();

		if (triId != PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxU32 triVertInd[3];
			getGRBVertexIndices(triVertInd[0], triVertInd[1], triVertInd[2], triId, triangleMesh);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxU32 triVertHandle = PxEncodeClothIndex(clothId, triVertInd[i]);
				PxgNonRigidFilterPair tetVertPair;
				tetVertPair.index0 = tetHandle;
				tetVertPair.index1 = triVertHandle;
				releaseFilterPairInternal(tetVertPair, mSoftBodyClothTetVertFilterPairs, mSoftBodyClothTetVertFilterRefs, mSoftBodyClothAttachments.mAttachmentsDirty);
			}
		}
		else
		{
			const PxU32 clothVertFullMask = PxEncodeClothIndex(clothId, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
			PxgNonRigidFilterPair tetVertPair;
			tetVertPair.index0 = tetHandle;
			tetVertPair.index1 = clothVertFullMask;
			releaseFilterPairInternal(tetVertPair, mSoftBodyClothTetVertFilterPairs, mSoftBodyClothTetVertFilterRefs, mSoftBodyClothAttachments.mAttachmentsDirty);
		}
	}

	PxU32 PxgSimulationController::addClothAttachment(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface, PxU32 triIdx,
		const PxVec4& triBary, PxU32 tetId, const PxVec4& tetBary, PxConeLimitedConstraint* constraint, PxReal constraintOffset, const bool isActive, bool doConversion)
	{
		PxU32 simTetIdx = 0xFFFFFFFF;
		PxVec4 simTetBary;

		if (doConversion)
		{
			computeSoftBodySimMeshData(deformableVolume, tetId, tetBary, simTetIdx, simTetBary);
		}
		else
		{
			simTetIdx = tetId;
			simTetBary = tetBary;
		}
		
		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx = static_cast<PxU32*>(triangleMesh->mGRB_faceRemapInverse)[triIdx];
		
		//create attachment
		PxgFEMFEMAttachment attachment;

		PxU32 softBodyId = deformableVolume->getGpuRemapId();
		PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, simTetIdx);

		PxU32 clothId = deformableSurface->getGpuRemapId();
		PxU32 triHandle = PxEncodeClothIndex(clothId, triIdx);
		
		attachment.index0 = triHandle;
		attachment.index1 = tetHandle;
		attachment.constraintOffset = constraintOffset;
		PxU32 handle = mSoftBodyClothAttachments.mBaseHandle++;

		attachment.barycentricCoordinates0 = make_float4(triBary.x, triBary.y, triBary.z, 0.f);
		attachment.barycentricCoordinates1 = make_float4(simTetBary.x, simTetBary.y, simTetBary.z, simTetBary.w);
		
		if (constraint)
		{
			attachment.coneLimitParams.low_high_angle = make_float4(constraint->mLowLimit, constraint->mHighLimit, constraint->mAngle, 0.f);
			if (constraint->mAngle >= 0.f)
			{
				const Gu::TetrahedronMesh* simMesh = static_cast<const Gu::TetrahedronMesh*>(deformableVolume->getSimulationMesh());
				PxVec4 barycentric = Gu::addAxisToSimMeshBarycentric(*simMesh, simTetIdx, simTetBary, constraint->mAxis.getNormalized());
				attachment.coneLimitParams.barycentric = make_float4(barycentric.x, barycentric.y, barycentric.z, barycentric.w);
			}
			else
			{
				attachment.coneLimitParams.barycentric = make_float4(0.f, 0.f, 0.f, 0.f);
			}
		}
		else
		{
			attachment.coneLimitParams.low_high_angle = make_float4(0.f, 0.f, -1.0f, 0.f);
			attachment.coneLimitParams.barycentric = make_float4(0.f, 0.f, 0.f, 0.f);
		}
		mSoftBodyClothAttachments.addAttachment(attachment, handle);
		if(isActive)
			mSoftBodyClothAttachments.activateAttachment(handle);

		deformableVolume->mSurfaceVolumeAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeClothAttachment(Dy::DeformableVolume* deformableVolume, PxU32 handle)
	{
		if(mSoftBodyClothAttachments.removeAttachment(handle))
			deformableVolume->mSurfaceVolumeAttachments.findAndReplaceWithLast(handle);
	}

	void PxgSimulationController::addParticleFilter(Dy::DeformableVolume* deformableVolume, Dy::ParticleSystem* particleSystem,
		PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
	{
		const PxU32 softBodyId = deformableVolume->getGpuRemapId();
		const PxU32 particleSystemId = particleSystem->getGpuRemapId();

		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		
		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId = (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET ? tetId : static_cast<PxU32*>(tetMesh->mGRB_faceRemapInverse)[tetId]);
		PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetId); //soft body and tet index
		PxU64 particleHandle = PxEncodeParticleIndex(particleSystemId, particleId);
		
		PxgNonRigidFilterPair pair;
		pair.index1 = tetHandle;
		pair.index0 = particleHandle;
		pair.index2 = userBufferId;

		addFilterPairInternal(pair, mSoftBodyParticleFilterPairs, mSoftBodyParticleFilterRefs, mParticleSoftBodyAttachments.mAttachmentsDirty);
	}

	void PxgSimulationController::removeParticleFilter(Dy::DeformableVolume* deformableVolume,
		const Dy::ParticleSystem* particleSystem, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
	{
		const PxU32 softBodyId = deformableVolume->getGpuRemapId();
		const PxU32 particleSystemId = particleSystem->getGpuRemapId();

		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());

		//Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in the BV32 mesh)
		tetId = (tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET ? tetId : static_cast<PxU32*>(tetMesh->mGRB_faceRemapInverse)[tetId]);

		PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetId);
		PxU64 particleHandle = PxEncodeParticleIndex(particleSystemId, particleId);
		PxgNonRigidFilterPair pair;
		pair.index1 = tetHandle;
		pair.index0 = particleHandle;
		pair.index2 = userBufferId;

		releaseFilterPairInternal(pair, mSoftBodyParticleFilterPairs, mSoftBodyParticleFilterRefs, mParticleSoftBodyAttachments.mAttachmentsDirty);
	}

	PxU32 PxgSimulationController::addParticleAttachment(Dy::DeformableVolume* deformableVolume, const Dy::ParticleSystem* particleSystem,
		PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric, const bool isActive)
	{
		PxU32 outTetIdx = 0xFFFFFFFF;
		PxVec4 outBarycentric;

		computeSoftBodySimMeshData(deformableVolume, tetId, barycentric, outTetIdx, outBarycentric);

		const PxU32 softBodyId = deformableVolume->getGpuRemapId();
		const PxU32 particleSystemId = particleSystem->getGpuRemapId();

		const PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, outTetIdx);
		const PxU64 particleHandle = PxEncodeParticleIndex(particleSystemId, particleId);
		//create attachment
		PxgFEMFEMAttachment attachment;
		attachment.index0 = particleHandle;
		attachment.index1 = tetHandle;
		PxU32 handle = mParticleSoftBodyAttachments.mBaseHandle++;
		attachment.barycentricCoordinates0 = make_float4(PxReal(userBufferId), 0.f, 0.f, 1.f); 
		attachment.barycentricCoordinates1 = make_float4(outBarycentric.x, outBarycentric.y, outBarycentric.z, outBarycentric.w);

		mParticleSoftBodyAttachments.addAttachment(attachment, handle);
		if (isActive)
			mParticleSoftBodyAttachments.activateAttachment(handle);

		deformableVolume->mParticleVolumeAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeParticleAttachment(Dy::DeformableVolume* deformableVolume, PxU32 handle)
	{
		if (mParticleSoftBodyAttachments.removeAttachment(handle))
			deformableVolume->mParticleVolumeAttachments.findAndReplaceWithLast(handle);
	}

	//DEPRECATED
	void PxgSimulationController::addRigidFilter(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex)
	{
		PxU32 softBodyId = deformableVolume->getGpuRemapId();

		const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		const Gu::DeformableVolumeAuxData* softBodyAuxData = static_cast<const Gu::DeformableVolumeAuxData*>(deformableVolume->getAuxData());

		PxU32* accumulatedTetRefs = softBodyAuxData->getCollisionAccumulatedTetrahedronRefs();
		PxU32* tetRefs = softBodyAuxData->getCollisionTetrahedronRefs();
		const PxU32 nbTetRefs = softBodyAuxData->getCollisionNbTetrahedronRefs();
		const PxU32 nbVerts = tetMesh->getNbVerticesFast();

		const PxU32 startIndex = accumulatedTetRefs[vertIndex];
		const PxU32 endIndex = (vertIndex == nbVerts - 1) ? nbTetRefs : accumulatedTetRefs[vertIndex + 1];

		for (PxU32 i = startIndex; i < endIndex; ++i)
		{
			const PxU32 tetInd = tetRefs[i];
			PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetInd); //soft body and tet index

			PxgRigidFilterPair pair;
			pair.index1 = tetHandle;
			pair.index0 = reinterpret_cast<const PxU64&>(rigidNodeIndex);

			addFilterPairInternal(pair, mSoftBodyRigidFilterPairs, mSoftBodyRigidFilterRefs, mRigidSoftBodyAttachments.mAttachmentsDirty);
		}
	}

	//DEPRECATED
	void PxgSimulationController::removeRigidFilter(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex)
	{
		PxU32 softBodyId = deformableVolume->getGpuRemapId();
		const Gu::TetrahedronMesh* tetMesh = static_cast<const Gu::TetrahedronMesh*>(deformableVolume->getCollisionMesh());
		const Gu::DeformableVolumeAuxData* softBodyAuxData = static_cast<const Gu::DeformableVolumeAuxData*>(deformableVolume->getAuxData());

		{
			PxU32* accumulatedTetRefs = softBodyAuxData->getCollisionAccumulatedTetrahedronRefs();
			PxU32* tetRefs = softBodyAuxData->getCollisionTetrahedronRefs();
			const PxU32 nbTetRefs = softBodyAuxData->getCollisionNbTetrahedronRefs();
			const PxU32 nbVerts = tetMesh->getNbVerticesFast();

			const PxU32 startIndex = accumulatedTetRefs[vertIndex];
			const PxU32 endIndex = (vertIndex == nbVerts - 1) ? nbTetRefs : accumulatedTetRefs[vertIndex + 1];

			for (PxU32 i = startIndex; i < endIndex; ++i)
			{
				const PxU32 tetInd = tetRefs[i];
				PxU32 tetHandle = PxEncodeSoftBodyIndex(softBodyId, tetInd);

				PxgRigidFilterPair pair;
				pair.index1 = tetHandle;
				pair.index0 = reinterpret_cast<const PxU64&>(rigidNodeIndex);

				releaseFilterPairInternal(pair, mSoftBodyRigidFilterPairs, mSoftBodyRigidFilterRefs, mRigidSoftBodyAttachments.mAttachmentsDirty);
			}
		}
	}

	PxU32 PxgSimulationController::addRigidAttachment(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& softBodyNodeIndex,
		PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex, const PxVec3& actorSpacePose,
		PxConeLimitedConstraint* constraint, const bool isActive, bool doConversion)
	{
		PX_UNUSED(softBodyNodeIndex);
		//PxU32 softBodyId = deformableVolume->getGpuRemapId();

		//const Gu::BVTetrahedronMesh* tetMesh = static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		const Gu::DeformableVolumeAuxData* softBodyAuxData = static_cast<const Gu::DeformableVolumeAuxData*>(deformableVolume->getAuxData());
		
		//Now we make sure that the bodies are sorted...
		PxU32 handle = 0xFFFFFFFF;
		
		PxU32 elemIdx = vertIndex;
		bool isVertex = true;
		PxVec4 barycentric(0.0f);
		if (doConversion)
		{
			//deprecated code paths only ever uses tets
			isVertex = false;
			elemIdx = softBodyAuxData->mVertsRemapInGridModel[vertIndex];
			barycentric = reinterpret_cast<PxVec4*>(softBodyAuxData->mVertsBarycentricInGridModel)[vertIndex];
		}

		handle = addRigidAttachmentInternal(deformableVolume->getGpuRemapId(), elemIdx, isVertex, barycentric,
			rigidBody, rigidNodeIndex, actorSpacePose, constraint, mRigidSoftBodyAttachments, isActive);

		deformableVolume->mRigidVolumeAttachments.pushBack(handle);
		return handle;
	}

	void PxgSimulationController::removeRigidAttachment(Dy::DeformableVolume* deformableVolume, PxU32 handle)
	{
		if(mRigidSoftBodyAttachments.removeAttachment(handle))
			deformableVolume->mRigidVolumeAttachments.findAndReplaceWithLast(handle);
	}

	PxU32 PxgSimulationController::addRigidAttachment(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& clothNodeIndex,
		PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex, const PxVec3& actorSpacePose,
		PxConeLimitedConstraint* constraint, const bool addToActive)
	{
		PX_UNUSED(clothNodeIndex);

		const bool isVertex = true;
		const PxVec4 barycentric(0.0f);
		PxU32 handle = addRigidAttachmentInternal(deformableSurface->getGpuRemapId(), vertIndex, isVertex, barycentric,
			rigidBody, rigidNodeIndex, actorSpacePose, constraint, mClothRigidAttachments, addToActive);

		deformableSurface->addAttachmentHandle(handle);
		return handle;
	}

	void PxgSimulationController::removeRigidAttachment(Dy::DeformableSurface* deformableSurface, PxU32 handle)
	{
		if(mClothRigidAttachments.removeAttachment(handle))
			deformableSurface->removeAttachmentHandle(handle);
	}

	void PxgSimulationController::addTriRigidFilter(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& rigidNodeIndex, PxU32 triIdx)
	{
		if (triIdx == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			return;
		}

		const PxU32 clothId = deformableSurface->getGpuRemapId();

		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx = static_cast<PxU32*>(triangleMesh->mGRB_faceRemapInverse)[triIdx];

		const PxU64 rigidIdx = reinterpret_cast<const PxU64&>(rigidNodeIndex);

		PxU32 vInd[3];
		getGRBVertexIndices(vInd[0], vInd[1], vInd[2], triIdx, triangleMesh);

		for (PxU32 i = 0; i < 3; ++i)
		{
			const PxU32 vertHandle = PxEncodeClothIndex(clothId, vInd[i]); //cloth and vert index
			PxgRigidFilterPair pair;
			pair.index1 = vertHandle;
			pair.index0 = rigidIdx;
			addFilterPairInternal(pair, mClothRigidFilterPairs, mClothRigidFilterRefs, mClothRigidAttachments.mAttachmentsDirty);
		}
	}

	void PxgSimulationController::removeTriRigidFilter(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& rigidNodeIndex, PxU32 triIdx)
	{
		if (triIdx == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			return;
		}

		PxU32 clothId = deformableSurface->getGpuRemapId();

		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx = static_cast<PxU32*>(triangleMesh->mGRB_faceRemapInverse)[triIdx];
		const PxU64 rigidIdx = reinterpret_cast<const PxU64&>(rigidNodeIndex);

		PxU32 vInd[3];
		getGRBVertexIndices(vInd[0], vInd[1], vInd[2], triIdx, triangleMesh);

		for (PxU32 i = 0; i < 3; ++i)
		{
			const PxU32 vertHandle = PxEncodeClothIndex(clothId, vInd[i]); //cloth and vert index
			PxgRigidFilterPair pair;
			pair.index1 = vertHandle;
			pair.index0 = rigidIdx;
			releaseFilterPairInternal(pair, mClothRigidFilterPairs, mClothRigidFilterRefs, mClothRigidAttachments.mAttachmentsDirty);
		}
	}

	PxU32 PxgSimulationController::addTriRigidAttachment(Dy::DeformableSurface* deformableSurface, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 triIdx, const PxVec4& barycentric,
		const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint, const bool isActive)
	{
		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx = static_cast<PxU32*>(triangleMesh->mGRB_faceRemapInverse)[triIdx];

		const bool isVertex = false;
		PxU32 handle = addRigidAttachmentInternal(deformableSurface->getGpuRemapId(), triIdx, isVertex, barycentric,
			rigidBody, rigidNodeIndex, actorSpacePose, constraint, mClothRigidAttachments, isActive);
		
		deformableSurface->addAttachmentHandle(handle);
		return handle;
	}

	void PxgSimulationController::removeTriRigidAttachment(Dy::DeformableSurface* deformableSurface, PxU32 handle)
	{
		if(mClothRigidAttachments.removeAttachment(handle))
			deformableSurface->removeAttachmentHandle(handle);
	 }

	void PxgSimulationController::addClothFilter(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1, PxU32 triIdx0, PxU32 triIdx1)
	{
		if (triIdx0 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI && triIdx1 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			return;
		}

		PxsShapeCore& shapeCore0 = deformableSurface0->getShapeCore();
		const Gu::TriangleMesh* triangleMesh0 = _getMeshData(shapeCore0.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		//PxU32 originalTriIdx = triIdx;
		triIdx0 = (triIdx0 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI) ? triIdx0 : static_cast<PxU32*>(triangleMesh0->mGRB_faceRemapInverse)[triIdx0];

		PxU32 clothId0 = deformableSurface0->getGpuRemapId();
		PxU32 triHandle0 = PxEncodeClothIndex(clothId0, triIdx0);

		PxsShapeCore& shapeCore1 = deformableSurface1->getShapeCore();
		const Gu::TriangleMesh* triangleMesh1 = _getMeshData(shapeCore1.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		//PxU32 originalTriIdx = triIdx;
		triIdx1 = (triIdx1 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI) ? triIdx1 : static_cast<PxU32*>(triangleMesh1->mGRB_faceRemapInverse)[triIdx1];

		PxU32 clothId1 = deformableSurface1->getGpuRemapId();
		PxU32 triHandle1 = PxEncodeClothIndex(clothId1, triIdx1);

		//add vert vs triangle pair filter
		if (triIdx0 != PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxU32 triVertInd0[3];
			getGRBVertexIndices(triVertInd0[0], triVertInd0[1], triVertInd0[2], triIdx0, triangleMesh0);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxU32 triVertHandle = PxEncodeClothIndex(clothId0, triVertInd0[i]);
				PxgNonRigidFilterPair pair;
				pair.index0 = triVertHandle;
				pair.index1 = triHandle1;
				addFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}

			if (triIdx1 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
			{
				PxU32 vtxMaskFull1 = PxEncodeClothIndex(clothId1, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
				PxgNonRigidFilterPair pair;
				pair.index0 = vtxMaskFull1; 
				pair.index1 = triHandle0;
				addFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}
		}

		if (triIdx1 != PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxU32 triVertInd1[3];
			getGRBVertexIndices(triVertInd1[0], triVertInd1[1], triVertInd1[2], triIdx1, triangleMesh1);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxU32 triVertHandle = PxEncodeClothIndex(clothId1, triVertInd1[i]);
				PxgNonRigidFilterPair pair;
				pair.index0 = triVertHandle;
				pair.index1 = triHandle0;
				addFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}

			if (triIdx0 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
			{
				PxU32 vtxMaskFull0 = PxEncodeClothIndex(clothId0, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
				PxgNonRigidFilterPair pair;
				pair.index0 = vtxMaskFull0;
				pair.index1 = triHandle1;
				addFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}
		}
	}

	void PxgSimulationController::removeClothFilter(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1, PxU32 triIdx0, PxU32 triIdx1)
	{
		PxsShapeCore& shapeCore0 = deformableSurface0->getShapeCore();
		const Gu::TriangleMesh* triangleMesh0 = _getMeshData(shapeCore0.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		//PxU32 originalTriIdx = triIdx;
		triIdx0 = (triIdx0 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI) ? triIdx0 : static_cast<PxU32*>(triangleMesh0->mGRB_faceRemapInverse)[triIdx0];

		PxU32 clothId0 = deformableSurface0->getGpuRemapId();
		PxU32 triHandle0 = PxEncodeClothIndex(clothId0, triIdx0);

		PxsShapeCore& shapeCore1 = deformableSurface1->getShapeCore();
		const Gu::TriangleMesh* triangleMesh1 = _getMeshData(shapeCore1.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		//PxU32 originalTriIdx = triIdx;
		triIdx1 = (triIdx1 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI) ? triIdx1 : static_cast<PxU32*>(triangleMesh1->mGRB_faceRemapInverse)[triIdx1];

		PxU32 clothId1 = deformableSurface1->getGpuRemapId();
		PxU32 triHandle1 = PxEncodeClothIndex(clothId1, triIdx1);

		if (triIdx0 != PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxU32 triVertInd0[3];
			getGRBVertexIndices(triVertInd0[0], triVertInd0[1], triVertInd0[2], triIdx0, triangleMesh0);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxU32 triVertHandle = PxEncodeClothIndex(clothId0, triVertInd0[i]);
				PxgNonRigidFilterPair pair;
				pair.index0 = triVertHandle;
				pair.index1 = triHandle1;
				releaseFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}

			if (triIdx1 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
			{
				PxU32 vtxMaskFull1 = PxEncodeClothIndex(clothId1, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
				PxgNonRigidFilterPair pair;
				pair.index0 = vtxMaskFull1;
				pair.index1 = triHandle0;
				releaseFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}
		}

		if (triIdx1 != PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxU32 triVertInd1[3];
			getGRBVertexIndices(triVertInd1[0], triVertInd1[1], triVertInd1[2], triIdx1, triangleMesh1);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxU32 triVertHandle = PxEncodeClothIndex(clothId1, triVertInd1[i]);
				PxgNonRigidFilterPair pair;
				pair.index0 = triVertHandle;
				pair.index1 = triHandle0;
				releaseFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}

			if (triIdx0 == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
			{
				PxU32 vtxMaskFull0 = PxEncodeClothIndex(clothId0, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
				PxgNonRigidFilterPair pair;
				pair.index0 = vtxMaskFull0;
				pair.index1 = triHandle1;
				releaseFilterPairInternal(pair, mClothClothVertTriFilterPairs, mClothClothVertTriFilterRefs, mClothClothAttachments.mAttachmentsDirty);
			}
		}
	}

	PxU32 PxgSimulationController::addTriClothAttachment(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1, PxU32 triIdx0, PxU32 triIdx1,
		const PxVec4& triBarycentric0, const PxVec4& triBarycentric1, const bool addToActive)
	{	
		PxsShapeCore& shapeCore0 = deformableSurface0->getShapeCore();
		const Gu::TriangleMesh* triangleMesh0 = _getMeshData(shapeCore0.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx0 = static_cast<PxU32*>(triangleMesh0->mGRB_faceRemapInverse)[triIdx0];

		PxU32 clothId0 = deformableSurface0->getGpuRemapId();
		PxU32 triHandle0 = PxEncodeClothIndex(clothId0, triIdx0);

		PxsShapeCore& shapeCore1 = deformableSurface1->getShapeCore();
		const Gu::TriangleMesh* triangleMesh1 = _getMeshData(shapeCore1.mGeometry.get<PxTriangleMeshGeometry>());

		//Map from CPU triangle ID (corresponds to the ID in the BV4 mesh) to GPU triangle ID (corresponds to the ID in the BV32 mesh)
		triIdx1 = static_cast<PxU32*>(triangleMesh1->mGRB_faceRemapInverse)[triIdx1];

		PxU32 clothId1 = deformableSurface1->getGpuRemapId();
		PxU32 triHandle1 = PxEncodeClothIndex(clothId1, triIdx1);

		//create attachment
		PxgFEMFEMAttachment attachment;
		attachment.index0 = triHandle0;
		attachment.index1 = triHandle1;
		PxU32 handle = mClothClothAttachments.mBaseHandle++;
		//attachment.handle = handle;
		//attachment.referenceCount = 1;
		attachment.barycentricCoordinates0 = make_float4(triBarycentric0.x, triBarycentric0.y, triBarycentric0.z, 0.f);
		attachment.barycentricCoordinates1 = make_float4(triBarycentric1.x, triBarycentric1.y, triBarycentric1.z, 0.f);

		mClothClothAttachments.addAttachment(attachment, handle);
		if (addToActive)
			mClothClothAttachments.activateAttachment(handle);

		//add the attachments to cloth 0. The island gen will wake up both pieces of cloth
		//and we just need to add the attchment handle to one of them so we can active the attachment
		deformableSurface0->mSurfaceSurfaceAttachments.pushBack(handle);
	
		return handle;
	}

	void PxgSimulationController::removeTriClothAttachment(Dy::DeformableSurface* deformableSurface0, PxU32 handle)
	{
		if (mClothClothAttachments.removeAttachment(handle))
		{
			//because we just add the handle to cloth0's attachment, so we can just remove the handle from cloth0's list
			deformableSurface0->mSurfaceSurfaceAttachments.findAndReplaceWithLast(handle);
		}
	}

	PxU32 PxgSimulationController::getNbDeactivatedDeformableSurfaces() const
	{
		return mFEMClothCore->mDeactivatingDeformableSurfaces.size();
	}

	PxU32 PxgSimulationController::getNbActivatedDeformableSurfaces() const
	{
		return mFEMClothCore->mActivatingDeformableSurfaces.size();
	}

	Dy::DeformableSurface** PxgSimulationController::getDeactivatedDeformableSurfaces() const
	{
		return mFEMClothCore->mDeactivatingDeformableSurfaces.begin();
	}

	Dy::DeformableSurface** PxgSimulationController::getActivatedDeformableSurfaces() const
	{
		return mFEMClothCore->mActivatingDeformableSurfaces.begin();
	}

	PxU32 PxgSimulationController::getNbDeactivatedDeformableVolumes() const
	{
		return mSoftBodyCore->mDeactivatingDeformableVolumes.size();
	}

	PxU32 PxgSimulationController::getNbActivatedDeformableVolumes() const
	{
		return mSoftBodyCore->mActivatingDeformableVolumes.size();
	}

	Dy::DeformableVolume** PxgSimulationController::getDeactivatedDeformableVolumes() const
	{
		return mSoftBodyCore->mDeactivatingDeformableVolumes.begin();
	}
	Dy::DeformableVolume** PxgSimulationController::getActivatedDeformableVolumes() const
	{
		return mSoftBodyCore->mActivatingDeformableVolumes.begin();
	}

	const PxReal* PxgSimulationController::getDeformableVolumeWakeCounters() const
	{ 
		return mSimulationCore->getActiveSBWakeCountsCPU();
	}
	void PxgSimulationController::setEnableOVDReadback(bool enableOVDReadback)
	{
		mEnableOVDReadback = enableOVDReadback;
	}

	bool PxgSimulationController::getEnableOVDReadback() const
	{
		return mEnableOVDReadback;
	}

	void PxgSimulationController::setEnableOVDCollisionReadback(bool enableOVDCollisionsReadback)
	{
		mEnableOVDCollisionReadback = enableOVDCollisionsReadback;
	}

	bool PxgSimulationController::getEnableOVDCollisionReadback() const
	{
		return mEnableOVDCollisionReadback;
	}
#if PX_SUPPORT_OMNI_PVD
	void PxgSimulationController::setOVDCallbacks(PxsSimulationControllerOVDCallbacks& ovdCallbacks)
	{
		mOvdCallbacks = &ovdCallbacks;
	}
#endif
}


