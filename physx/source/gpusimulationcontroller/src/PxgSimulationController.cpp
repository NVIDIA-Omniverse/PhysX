// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgSimulationController.h"
#include "PxgNarrowphaseCore.h"
#include "PxDirectGPUAPI.h"
#include "PxsRigidBody.h"
#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "cudamanager/PxCudaContextManager.h"
#include "foundation/PxAllocator.h"
#include "PxgSimulationCore.h"
#include "PxgArticulation.h"
#include "PxgDynamicsContext.h"
#include "PxgNphaseImplementationContext.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgCudaSolverCore.h"
#include "PxgBodySim.h"
#include "PxgShapeSim.h"
#include "PxsTransformCache.h"
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
#include "DyArticulationMimicJointCore.h"
#include "PxgArticulationCore.h"
#include "PxgSoftBody.h"
#include "PxgSoftBodyCore.h"
#include "PxgFEMCloth.h"
#include "PxgFEMClothCore.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemBuffer.h"
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

// TODO: Consider refactoring indentation for improved code readability

using namespace physx;

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
		PxgAllocatorDesc& allocDesc, PxU32 maxSoftBodyContacts, PxU32 maxFemClothContacts, PxU32 maxParticleContacts,
		PxU32 collisionStackSizeBytes, bool enableBodyAccelerations) :
		PxsSimulationController(callback, PxIntTrue),
		mPostCopyShapeSimTask(*this),
		mPostCopyBodySimTask(*this, enableBodyAccelerations),
		mPostUpdateParticleSystemTask(*this),
		mBodySimManager(allocDesc.hostAlloc),
		mJointManager(allocDesc.hostAlloc, allocDesc.hostMappedAlloc, dynamicContext->getEnableDirectGPUAPI(), cudaContextManager->getCudaContext()),
		mSoftBodyCore(NULL),
		mFEMClothCore(NULL),
		mPBDParticleSystemCore(NULL),
		mDynamicContext(dynamicContext), mNpContext(npContext),
		mNewBodySimPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinksPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkWakeCounterPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkAccelPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkPropPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkSleepDataPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkChildPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkParentPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkBody2WorldPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkBody2ActorPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mJointPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mJointDataPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mLinkJointIndexPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mArticulationPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
		mSpatialTendonParamPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mSpatialTendonPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mAttachmentFixedPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mAttachmentModPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mTendonAttachmentMapPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mFixedTendonParamPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mFixedTendonPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mTendonJointFixedDataPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mTendonJointCoefficientDataPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mTendonTendonJointMapPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mPathToRootPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mMimicJointPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mArticulationUpdatePoolMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eSIMULATION_ARTICULATION, Cm::PinnableAllocatorFallback::eDISABLED),
		mArticulationDofDataPoolMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eSIMULATION_ARTICULATION, Cm::PinnableAllocatorFallback::eDISABLED),
		mNewSoftBodyPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mSoftBodyPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mSoftBodyElementIndexPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mVolumeRigidAttachments(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mVolumeVolumeAttachments(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mVolumeSurfaceAttachments(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mVolumeRigidFilters(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mVolumeVolumeFilters(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mVolumeSurfaceFilters(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
		mNewFEMClothPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mFEMClothPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mFEMClothElementIndexPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mSurfaceRigidAttachments(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mSurfaceSurfaceAttachments(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mSurfaceRigidFilters(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mSurfaceSurfaceFilters(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
		mFrozenPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mUnfrozenPool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mActivatePool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mDeactivatePool(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
		mHasBeenSimulated(false),
		mGpuWranglerManager(static_cast<PxgCudaKernelWranglerManager*>(gpuWranglerManager)),
		mCudaContextManager(cudaContextManager),
		mAllocDesc(allocDesc),
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

		mSimulationCore = PX_NEW(PxgSimulationCore)(mGpuWranglerManager, mCudaContextManager, mAllocDesc, mDynamicContext, useGpuBroadphase);

		mNpContext->setSimulationCore(mSimulationCore);

		mDynamicContext->mGpuSimulationCore = mSimulationCore;
		mDynamicContext->mGpuBp = mBroadPhase;
	}

	PxgSimulationController::~PxgSimulationController()
	{
		// Deleting all topologies, even though they should already have been deleted by the prior surface
		// removal.
		for (PxU32 i = 0; i < mSurfaceTopologies.size(); ++i)
			PX_DELETE(mSurfaceTopologies[i]);

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
				PxgSoftBodyCore(mGpuWranglerManager, mCudaContextManager, mAllocDesc, this,
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
		const PxU32 remapId = deformableVolume->getGpuRemapId();

		mVolumeRigidFilters.release(remapId);
		mVolumeVolumeFilters.release(remapId);
		mVolumeSurfaceFilters.release(remapId);

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
			
			for (PxU32 i = 0; i < deformableVolume->mRigidAttachments.size(); ++i)
			{
				mVolumeRigidAttachments.activateAttachments(deformableVolume->mRigidAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mSurfaceAttachments.size(); ++i)
			{
				mVolumeSurfaceAttachments.activateAttachments(deformableVolume->mSurfaceAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mVolumeAttachments.size(); ++i)
			{
				mVolumeVolumeAttachments.activateAttachments(deformableVolume->mVolumeAttachments[i]);
			}
		}
	}

	void PxgSimulationController::deactivateSoftbody(Dy::DeformableVolume* deformableVolume)
	{
		if (mBodySimManager.deactivateSoftbody(deformableVolume))
		{
			for (PxU32 i = 0; i < deformableVolume->mRigidAttachments.size(); ++i)
			{
				mVolumeRigidAttachments.deactivateAttachments(deformableVolume->mRigidAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mSurfaceAttachments.size(); ++i)
			{
				mVolumeSurfaceAttachments.deactivateAttachments(deformableVolume->mSurfaceAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableVolume->mVolumeAttachments.size(); ++i)
			{
				mVolumeVolumeAttachments.deactivateAttachments(deformableVolume->mVolumeAttachments[i]);
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
				PxgFEMClothCore(mGpuWranglerManager, mCudaContextManager, mAllocDesc, this,
					mDynamicContext, maxFemClothContacts, mCollisionStackSizeBytes, isTGS));
		}
	}

	void PxgSimulationController::addFEMCloth(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& nodeIndex)
	{
		createDeformableSurfaceCore();

		mBodySimManager.addFEMCloth(deformableSurface, nodeIndex.index());

		// Compute additional mesh topology information for surface deformables. This is done on adding the surface to the
		// scene such that then added element collision filters can be setup using that information (specifically triangle
		// edge ownership).
		const PxU32 surfaceId = deformableSurface->getGpuRemapId();
		if (mSurfaceTopologies.size() <= surfaceId)
			mSurfaceTopologies.resize(surfaceId + 1, NULL);

		PX_ASSERT(mSurfaceTopologies[surfaceId] == NULL);
		PxsShapeCore& shapeCore = deformableSurface->getShapeCore();
		const Gu::TriangleMesh* triangleMesh = _getMeshData(shapeCore.mGeometry.get<PxTriangleMeshGeometry>());
		mSurfaceTopologies[surfaceId] = PX_NEW(PxgDeformableSurfaceTopology)(*triangleMesh);
	}

	void PxgSimulationController::releaseFEMCloth(Dy::DeformableSurface* deformableSurface)
	{
		const PxU32 surfaceId = deformableSurface->getGpuRemapId();

		mSurfaceRigidFilters.release(surfaceId);
		mSurfaceSurfaceFilters.release(surfaceId);

		if (surfaceId < mSurfaceTopologies.size())
			PX_DELETE(mSurfaceTopologies[surfaceId]);

		mBodySimManager.releaseFEMCloth(deformableSurface);

		if (mFEMClothCore)
			mFEMClothCore->onClothRemoved();
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
			for (PxU32 i = 0; i < deformableSurface->mRigidAttachments.size(); ++i)
			{
				mSurfaceRigidAttachments.activateAttachments(deformableSurface->mRigidAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableSurface->mSurfaceAttachments.size(); ++i)
			{
				mSurfaceSurfaceAttachments.activateAttachments(deformableSurface->mSurfaceAttachments[i]);
			}
		}
	}

	void PxgSimulationController::deactivateCloth(Dy::DeformableSurface* deformableSurface)
	{
		if (mBodySimManager.deactivateCloth(deformableSurface))
		{
			for (PxU32 i = 0; i < deformableSurface->mRigidAttachments.size(); ++i)
			{
				mSurfaceRigidAttachments.deactivateAttachments(deformableSurface->mRigidAttachments[i]);
			}

			for (PxU32 i = 0; i < deformableSurface->mSurfaceAttachments.size(); ++i)
			{
				mSurfaceSurfaceAttachments.deactivateAttachments(deformableSurface->mSurfaceAttachments[i]);
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
				PxgPBDParticleSystemCore(mGpuWranglerManager, mCudaContextManager, mAllocDesc, this,
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
			articulation->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_LINKS);
			mBodySimManager.updateArticulation(articulation, nodeIndex.index());
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

	const void* PxgSimulationController::getRigidBodyAccelerations() const
	{
		return mSimulationCore->getRigidBodyAccelerations();
	}

	void PxgSimulationController::updateArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		mBodySimManager.updateArticulation(articulation, nodeIndex.index());
	}

	void PxgSimulationController::updateArticulationJoint(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		articulation->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_JOINTS);
		mBodySimManager.updateArticulation(articulation, nodeIndex.index());
	}

	void PxgSimulationController::updateArticulationExtAccel(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)
	{
		articulation->raiseGPUDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL);
		mBodySimManager.updateArticulation(articulation, nodeIndex.index());
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

	bool PxgSimulationController::getRigidDynamicData(void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const
	{
		return mSimulationCore->getRigidDynamicData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent);
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

			if(!directGPUBoundsArray.isChangeTrackingEnabled())
			{
				copyBoundsAndTransforms(boundsArray, transformCache, gpuTransformCache, boundsArraySize, totalTransformCacheSize, npStream);
				if(!directGPUBoundsArray.enableChangeTracking())
				{
					PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY,
						PX_FL, "PxgSimulationController::updateBoundsAndTransformCache: failed to allocate pinned host buffer for bounds change tracking");
					mCudaContextManager->getCudaContext()->setAbortMode(true);
				}
			}
			else if(numChanges)
			{
				CUdeviceptr updatedActorDescd = mSimulationCore->getUpdatedActorDescDesc();
				mergeBoundsAndTransformsChanges(directGPUBoundsArray, transformCache, gpuTransformCache, boundsArraySize,
												totalTransformCacheSize, numChanges, updatedActorDescd, npStream);
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
																  PxU32 totalTransformCacheSize, PxU32 numChanges, CUdeviceptr updatedActorDescd, CUstream npStream)
	{

		const PxgBoundTransformUpdate* changes = reinterpret_cast<const PxgBoundTransformUpdate*>(
			getMappedDeviceConstPtr(mCudaContextManager->getCudaContext(), directGPUBoundsArray.getStagingBuffer()));


		PxBounds3* cpuBounds =
			reinterpret_cast<PxBounds3*>(getMappedDevicePtr(mCudaContextManager->getCudaContext(), directGPUBoundsArray.getBounds()));

		PxsCachedTransform* cpuTransforms = reinterpret_cast<PxsCachedTransform*>(getMappedDevicePtr(mCudaContextManager->getCudaContext(), transformCache.getTransforms()));
		
		
		PxgCudaBuffer* aabbBounds = mSimulationCore->getBoundArrayBuffer();
		aabbBounds->allocateCopyOldDataAsync(boundsArraySize, mCudaContextManager->getCudaContext(), npStream, PX_FL);
		CUdeviceptr boundsPtr = aabbBounds->getDevicePtr();


		gpuTransformCache.allocateCopyOldDataAsync(sizeof(PxsCachedTransform) * totalTransformCacheSize, mCudaContextManager->getCudaContext(), npStream, PX_FL);
		CUdeviceptr transformCachePtr = gpuTransformCache.getDevicePtr();

		CUfunction kernelFunction = mSimulationCore->mGpuKernelWranglerManager->getCuFunction(
			PxgKernelIds::MERGE_TRANSFORMCACHE_AND_BOUNDARRAY_CHANGES);

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(boundsPtr), PX_CUDA_KERNEL_PARAM(transformCachePtr), PX_CUDA_KERNEL_PARAM(updatedActorDescd),
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
																   boundsArray.getBounds(), boundsArraySize, npStream);
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

		if (!mArticulationUpdatePoolMapped.resize(nbUpdatedArticulations))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
				"PxgSimulationController::copyToGpuBodySim: failed to allocate pinned host buffer for articulation update");
			mCudaContextManager->getCudaContext()->setAbortMode(true);
			return;
		}

		if(!mArticulationDofDataPoolMapped.resize(nbUpdatedDofs))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
				"PxgSimulationController::copyToGpuBodySim: failed to allocate pinned host buffer for articulation dof data");
			mCudaContextManager->getCudaContext()->setAbortMode(true);
			return;
		}

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
				const Dy::ArticulationLink& cpuLink = data.getLink(a);

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
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointVelocities(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointForces(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointTargetPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointTargetVelocities(), dofs * sizeof(PxReal));
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

		Cm::VirtualAllocatorCallback& hostAlloc = mAllocDesc.hostAlloc;

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
			gpuSoftBody.mTetIndices = reinterpret_cast<uint4*>(hostAlloc.allocate(sizeof(uint4) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mTetMeshSurfaceHint = reinterpret_cast<PxU8*>(hostAlloc.allocate(sizeof(PxU8) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mTetIndicesRemapTable = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(float) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			
			gpuSoftBody.mTetraRestPoses = reinterpret_cast<PxMat33*>(hostAlloc.allocate(sizeof(PxMat33) * nbTets, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			
			const PxU32 nbVerts = colTetMesh->getNbVerticesFast();

			const PxU32 nbTetsGM = simTetMesh->getNbTetrahedronsFast();
			const PxU32 nbVertsGM = simTetMesh->getNbVerticesFast();
			const PxU32 nbPartitionsGM = softBodyAuxData->getNbGMPartitionFast();
			const PxU32 nbTetsPerElementGM = softBodyAuxData->mNumTetsPerElement;
			const PxU32 nbElementsGM = nbTetsGM / nbTetsPerElementGM;
			const PxU32 nbVertsPerElementGM = nbTetsPerElementGM == 1 ? 4 : 8;

			const PxU32 nbRemapOutputSize = softBodyAuxData->getGMRemapOutputSizeFast();
			const PxU32 nbTetRemapSize = softBodyAuxData->getNbTetRemapSizeFast();

			gpuSoftBody.mVertsBarycentricInGridModel = reinterpret_cast<float4*>(hostAlloc.allocate(sizeof(float4) * nbVerts, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mVertsRemapInGridModel = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));

			gpuSoftBody.mTetsRemapColToSim = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbTetRemapSize, PxsHeapStats::eSIMULATION, PX_FL));
			gpuSoftBody.mTetsAccumulatedRemapColToSim = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbTets, PxsHeapStats::eSIMULATION, PX_FL));

			gpuSoftBody.mSurfaceVertsHint = reinterpret_cast<PxU8*>(hostAlloc.allocate(sizeof(PxU8) * nbVerts, PxsHeapStats::eSIMULATION, PX_FL));
			gpuSoftBody.mSurfaceVertToTetRemap = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION, PX_FL));

			gpuSoftBody.mOrderedMaterialIndices = reinterpret_cast<PxU16*>(hostAlloc.allocate(sizeof(PxU16) * nbTetsGM, PxsHeapStats::eSIMULATION, PX_FL));
			gpuSoftBody.mMaterialIndices = reinterpret_cast<PxU16*>(hostAlloc.allocate(sizeof(PxU16) * nbTetsGM, PxsHeapStats::eSIMULATION, PX_FL));

			gpuSoftBody.mSimTetIndices = reinterpret_cast<uint4*>(hostAlloc.allocate(sizeof(uint4) * nbTetsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mSimTetraRestPoses = reinterpret_cast<PxgMat33Block*>(hostAlloc.allocate(sizeof(PxgMat33Block) * ((nbTetsGM+31)/32), PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			
			gpuSoftBody.mSimOrderedTetrahedrons = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbElementsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			gpuSoftBody.mSimPullIndices = reinterpret_cast<uint4*>(hostAlloc.allocate(sizeof(PxU32) * nbElementsGM * nbVertsPerElementGM, PxsHeapStats::eSIMULATION, PX_FL));

			//we need to allocate more memory for mGMOrderedVertInvMassCP for the accumulated buffer for duplicate verts 
			//gpuSoftBody.mGMOrderedVertInvMassCP = reinterpret_cast<float4*>(alloc->allocate(sizeof(float4) * nbRemapOutputSize, PX_FL));

			gpuSoftBody.mSimAccumulatedPartitionsCP = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbPartitionsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));

			if (nbRemapOutputSize) // used for tet mesh only
			{
				gpuSoftBody.mSimRemapOutputCP = reinterpret_cast<uint4*>(hostAlloc.allocate(sizeof(PxU32) * nbElementsGM * nbVertsPerElementGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
				gpuSoftBody.mSimAccumulatedCopiesCP = reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbVertsGM, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
			}

			const Dy::DeformableVolumeCore& core = deformableVolume->getCore();
			// SDK indices can differ after material reuse or across scenes.
			PxArray<PxU16> gpuMaterialHandles(core.materialHandles.size());
			mDynamicContext->getNarrowphaseCore()->mapMaterialIndices<PxsDeformableVolumeMaterialData>(
				gpuMaterialHandles.begin(), core.materialHandles.begin(), core.materialHandles.size());

			PxgSoftBodyUtil::initialTetData(gpuSoftBody, colTetMesh, simTetMesh, softBodyAuxData, 
				gpuMaterialHandles.begin(), hostAlloc);

			PxU8* tMem = reinterpret_cast<PxU8*>(hostAlloc.allocate(byteSize, PxsHeapStats::eSIMULATION_SOFTBODY, PX_FL));
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

			gpuSoftBody.mRestOffset = deformableVolume->getShapeCore().mRestOffset;

			gpuSoftBody.mOriginalContactOffset = deformableVolume->getShapeCore().mContactOffset;
		
			mNewSoftBodyElementIndexPool[i] = deformableVolume->getElementId();
		}
	}

	void PxgSimulationController::copyToGpuFEMClothSim(PxU32 startIndex, PxU32 nbToCopy)
	{
		void** bodySimsLL = mBodySimManager.mBodies.begin();
		PxArray<PxgFEMClothIndices>& newFEMClothSims = mBodySimManager.mNewFEMClothSims;

		PxU32 endIndex = startIndex + nbToCopy;

		Cm::VirtualAllocatorCallback& hostAlloc = mAllocDesc.hostAlloc;
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

			gpuFEMCloth.mRestOffset = deformableSurface->getShapeCore().mRestOffset;
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

			gpuFEMCloth.mTriangleVertexIndices_ownedEdges =
				reinterpret_cast<uint4*>(hostAlloc.allocate(sizeof(uint4) * nbTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			gpuFEMCloth.mMaterialIndices = reinterpret_cast<PxU16*>(
				hostAlloc.allocate(sizeof(PxU16) * nbTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			gpuFEMCloth.mDynamicFrictions = reinterpret_cast<float*>(
				hostAlloc.allocate(sizeof(float) * nbVerts, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			const PxU32 nbMaterials = core.materialHandles.size();
			PxArray<PxU16> gpuMaterialHandles(nbMaterials);
			npCore->mapMaterialIndices<PxsDeformableSurfaceMaterialData>(
				gpuMaterialHandles.begin(), core.materialHandles.begin(), nbMaterials);
			const PxU16* materialHandles = gpuMaterialHandles.begin();

			PxU8* tMem = reinterpret_cast<PxU8*>(
				hostAlloc.allocate(sizeof(PxU8) * byteSize, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			const PxU32 nbPackedNodes = PxgFEMClothUtil::loadOutTriangleMesh(tMem, triangleMesh);
			gpuFEMCloth.mTriMeshData = tMem;



			// initial "unordered" triangle and triangle-pair data
			PxgDeformableSurfaceTopology& topology = *mSurfaceTopologies[deformableSurface->getGpuRemapId()];

			// The topology owns the bending pairs until the upload releases them, so they are used in
			// place rather than copied out.
			PxArray<uint2>& trianglePairTriangleIndices = topology.getTrianglePairTriangleIndices();
			PxArray<uint4>& trianglePairVertexIndices = topology.getTrianglePairVertexIndices();

			const PxU32 nbTrianglePairs =
				PxgFEMClothUtil::initialTriangleData(gpuFEMCloth, triangleMesh, topology, materialHandles, materials, nbMaterials,
													 hostAlloc);

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
					reinterpret_cast<uint4*>(hostAlloc.allocate(sizeof(uint4) * nbNonSharedTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedNonSharedTriangleRestPoseInv =
					reinterpret_cast<float4*>(hostAlloc.allocate(sizeof(float4) * nbNonSharedTriangles, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			}

			if (nbSharedTrianglePairs)
			{
				gpuFEMCloth.mOrderedSharedTrianglePairVertexIndices = reinterpret_cast<uint4*>(hostAlloc.allocate(
					sizeof(uint4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping = reinterpret_cast<float4*>(hostAlloc.allocate(
					sizeof(float4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedSharedRestEdge0_edge1 = reinterpret_cast<float4*>(hostAlloc.allocate(
					sizeof(float4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedSharedRestEdgeLength_material0_material1 = reinterpret_cast<float4*>(hostAlloc.allocate(
					sizeof(float4) * nbSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mSharedTriPairAccumulatedCopiesCP =
					reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			}

			if (nbNonSharedTrianglePairs)
			{
				gpuFEMCloth.mOrderedNonSharedTrianglePairVertexIndices = reinterpret_cast<uint4*>(hostAlloc.allocate(
					sizeof(uint4) * nbNonSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping = reinterpret_cast<float4*>(hostAlloc.allocate(
					sizeof(float4) * nbNonSharedTrianglePairs, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
				gpuFEMCloth.mNonSharedTriPairAccumulatedCopiesCP =
					reinterpret_cast<PxU32*>(hostAlloc.allocate(sizeof(PxU32) * nbVerts, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));
			}

			PxArray<PxU32> orderedNonSharedTriangles;
			getFEMClothCore()->partitionTriangleSimData(gpuFEMCloth, gpuFEMClothData, orderedNonSharedTriangles, nonSharedTriangles);

			PxgFEMClothUtil::computeNonSharedTriangleConfiguration(gpuFEMCloth, orderedNonSharedTriangles, nonSharedTriangles, triangleMesh);

			// triangle-pair partition
			const PxU32 nbSharedTriPairPartitions = 8;
			const PxU32 nbNonSharedTriPairPartitions = 8;

			PxArray<PxU32> orderedSharedTrianglePairs;
			PxArray<PxU32> orderedNonSharedTrianglePairs;

			getFEMClothCore()->partitionTrianglePairSimData(gpuFEMCloth, gpuFEMClothData, nbSharedTriPairPartitions,
															orderedSharedTrianglePairs, sharedTrianglePairs, trianglePairVertexIndices,
															true);

			getFEMClothCore()->partitionTrianglePairSimData(gpuFEMCloth, gpuFEMClothData, nbNonSharedTriPairPartitions,
															orderedNonSharedTrianglePairs, nonSharedTrianglePairs,
															trianglePairVertexIndices, false);

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
		if(mCudaContextManager->getCudaContext()->isInAbortMode())
		{
			return;
		}

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

			PxgArticulationSimUpdate& update = mArticulationUpdatePoolMapped[startIndex + i];

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
					const Dy::ArticulationLink& cpuLink = data.getLink(a);

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
					const Dy::ArticulationLink& cpuLink = data.getLink(0);

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
						const Dy::ArticulationLink& cpuLink = data.getLink(a);

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
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}

				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointVelocities(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointForces(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointTargetPositions(), dofs * sizeof(PxReal));
					dofBegin += dofs;
				}
				if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
				{
					PxMemCopy(&mArticulationDofDataPoolMapped[dofBegin], data.getJointTargetVelocities(), dofs * sizeof(PxReal));
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
			newParticleSystem.mCommonData.mNumParticleBuffers = numParticleBuffers + numDiffuseBuffers;
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
			mSimulationCore->mGpuKernelWranglerManager
		);

		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::postCopyToBodySim(bool enableBodyAccelerations)
	{
		if(mCudaContextManager->getCudaContext()->isInAbortMode())
		{
			return;
		}
		mCudaContextManager->acquireContext();

		//ML: we reserve the mNewUpdatedBodies before we kick off the beforeSolver task, which update the the mNewUpdatedBodies using multiple thread. After the beforeSolver task,
		//we kick off updateBodiesAndShapes tasks. We need to set the size of mNewUpdatedBodies here 
		Cm::PinnableArray<PxgBodySimVelocityUpdate>& updatedBodySimPool = mBodySimManager.mNewUpdatedBodies;
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

		mSimulationCore->updateArticulations(mBodySimManager.mNewArticulationSims.size(), mArticulationUpdatePoolMapped.begin(),
			mArticulationUpdatePoolMapped.size(), mArticulationDofDataPoolMapped.begin());

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

		// Rebuild the mass-splitting counts before the DMA-up below.
		mVolumeRigidAttachments.buildRigidRefCounts();

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
			mVolumeRigidAttachments,
			mVolumeVolumeAttachments,
			mVolumeSurfaceAttachments,
			mVolumeRigidFilters,
			mVolumeVolumeFilters,
			mVolumeSurfaceFilters
			);

		mVolumeRigidAttachments.mAttachmentsDirty = false;
		mVolumeRigidAttachments.mActiveAttachmentsDirty = false;

		mVolumeSurfaceAttachments.mAttachmentsDirty = false;
		mVolumeSurfaceAttachments.mActiveAttachmentsDirty = false;

		mVolumeVolumeAttachments.mAttachmentsDirty = false;
		mVolumeVolumeAttachments.mActiveAttachmentsDirty = false;

		mVolumeRigidFilters.clearDirtyIds();
		mVolumeVolumeFilters.clearDirtyIds();
		mVolumeSurfaceFilters.clearDirtyIds();

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

		// Rebuild the mass-splitting counts before the DMA-up below.
		mSurfaceRigidAttachments.buildRigidRefCounts();

		// Volume-surface attachments and filters are uploaded by gpuMemDmaUpSoftBodies.
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
			mSurfaceRigidAttachments,
			mSurfaceSurfaceAttachments,
			mSurfaceRigidFilters,
			mSurfaceSurfaceFilters,
			mSurfaceTopologies
		);

		mSurfaceRigidAttachments.mAttachmentsDirty = false;
		mSurfaceRigidAttachments.mActiveAttachmentsDirty = false;
		mSurfaceRigidFilters.clearDirtyIds();
		mSurfaceSurfaceFilters.clearDirtyIds();
		mSurfaceSurfaceAttachments.mAttachmentsDirty = false;
		mSurfaceSurfaceAttachments.mActiveAttachmentsDirty = false;

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
		
		// PdHC: Copy velocities to "previous" buffer BEFORE uploading CPU changes.
		// This captures end-of-last-frame velocities, so velocity-delta will correctly
		// reflect forces applied via addForce/addLocalForceAtLocalPos between frames.
		// Works for both DirectGPU and non-DirectGPU (CPU Frontend + GPU Backend) modes.
		if(mBodySimManager.mTotalNumBodies > 0 && mSimulationCore->hasAccelerationBuffers())
		{
			mCudaContextManager->acquireContext();
			mSimulationCore->launchCopyRigidBodyVelocitiesToPrevious(mBodySimManager.mTotalNumBodies);
			mCudaContextManager->releaseContext();
		}
		
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
		const PxU32 nbActiveFemCloths = mBodySimManager.mActiveFEMCloths.size();

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

		if (nbActiveFemCloths > 0)
		{
			
			PX_PROFILE_ZONE("GpuSimulationController.sortClothContacts", 0);
			mFEMClothCore->sortContacts(nbActiveFemCloths);
		}

		mCudaContextManager->releaseContext();
	}

	//this is called after updateIncrementalIslands because island sim will add/remove joints based on the body activation
	void PxgSimulationController::updateJointsAndSyncData()
	{
		PX_PROFILE_ZONE("PxgSimulationController.updateGPUJoints", 0);
		if(mCudaContextManager->getCudaContext()->isInAbortMode())
		{
			return;
		}
		mCudaContextManager->acquireContext();

		const Cm::PinnableArray<PxU32>& updatedRigidJointIndices = mJointManager.getDirtyGPURigidJointDataIndices();
		const Cm::PinnableArray<PxgD6JointData>& rigidJointData = mJointManager.getGpuRigidJointData();
		const Cm::PinnableArray<PxgConstraintPrePrep>& rigidJointPrePrep = mJointManager.getGpuRigidJointPrePrep();

		const Cm::PinnableArray<PxU32>& updatedArtiJointIndices = mJointManager.getDirtyGPUArtiJointDataIndices();
		const Cm::PinnableArray<PxgD6JointData>& artiJointData = mJointManager.getGpuArtiJointData();
		const Cm::PinnableArray<PxgConstraintPrePrep>& artiJointPrePrep = mJointManager.getGpuArtiJointPrePrep();

		mSimulationCore->updateJointsAndSyncData(rigidJointData, updatedRigidJointIndices, artiJointData, updatedArtiJointIndices, 
			rigidJointPrePrep, artiJointPrePrep, mJointManager.getGpuConstraintIdMapHost(), mJointManager.getAndClearConstraintIdMapDirtyFlag(),
			mJointManager.getGpuNbRigidConstraints(), mJointManager.getGpuNbArtiConstraints());

		mJointManager.reset();

		mCudaContextManager->releaseContext();
	}

	//This is called after solver integration gpu task
	void PxgSimulationController::update(Cm::PinnableBitMap& changedHandleMap)
	{
		PX_PROFILE_ZONE("GpuSimulationController.update", 0);
		if(mSimulationCore->mCudaContext->isInAbortMode())
		{
			return;
		}

		mCudaContextManager->acquireContext();

		const PxU32 nbTotalBodies = mBodySimManager.mTotalNumBodies;
		const PxU32 nbTotalShapes = mSimulationCore->mPxgShapeSimManager.getTotalNbShapes();

		mHasBeenSimulated = true;

		const bool enableDirectGPUAPI = mDynamicContext->getEnableDirectGPUAPI();
		mSimulationCore->gpuMemDmaUp(nbTotalBodies, nbTotalShapes, changedHandleMap, enableDirectGPUAPI);
		
		mSimulationCore->update(enableDirectGPUAPI);

		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::initDirectGPUAPIState()
	{
		if(mSimulationCore->mCudaContext->isInAbortMode())
			return;

		mCudaContextManager->acquireContext();
		// mHasBeenSimulated is intentionally NOT set here. It is set in update() when
		// the solver actually runs and gpuMemDmaUp() sizes the device buffers. Setting
		// it here would cause gpuMemDmaBack() to run even when the solver was skipped
		// (e.g. needsSolve() == false), leading to memcpy from undersized device buffers.
		mSimulationCore->initDirectGPUAPIDescriptor();
		mCudaContextManager->releaseContext();
	}

	void PxgSimulationController::mergeChangedAABBMgHandle()
	{
		PxScopedCudaLock _lock(*mCudaContextManager);
	
		mSimulationCore->mergeChangedAABBMgHandle();
	}

	void PxgSimulationController::gpuDmabackData(PxsTransformCache& cache, Bp::BoundsArray& boundArray, Cm::PinnableBitMap&  changedAABBMgrHandles,
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

			PxsCachedTransform* cachedTransforms = cache.getTransforms();

#if PXG_SC_DEBUG
			mSimulationCore->mPxgShapeSimManager.validateCacheAndBounds(boundArray.getBounds(), cachedTransforms);
#endif // PXG_SC_BEBUG

			mSimulationCore->gpuMemDmaBack(mFrozenPool, mUnfrozenPool, mActivatePool, mDeactivatePool, cachedTransforms, cache.getTotalSize(), boundArray, changedAABBMgrHandles, nbTotalShapes,
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
			mSimulationCore->mPxgShapeSimManager.validateCacheAndBounds(boundArray.getBounds(), cache.getTransforms());
#endif
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

	// Adds a batch of rigid attachments for one deformable, recording the batch handle on the actor so that
	// the attachments can be activated when the actor is woken up.
	PxU32 PxgSimulationController::addVolumeRigidAttachments(Dy::DeformableVolume* deformableVolume, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex,
		PxDeformableAttachmentTargetType::Enum feature, const PxU32* indices, const PxVec4* barycentrics,
		const PxTransform& frame, const PxVec4* coords, PxU32 count, const bool isActive)
	{
		const PxU32 handle = mVolumeRigidAttachments.addVolumeRigidAttachments(deformableVolume->getGpuRemapId(), feature,
			indices, barycentrics, rigidBody, rigidNodeIndex, frame, coords, count, isActive);

		deformableVolume->mRigidAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeVolumeRigidAttachments(Dy::DeformableVolume* deformableVolume, PxU32 handle)
	{
		mVolumeRigidAttachments.removeAttachments(handle);
		deformableVolume->mRigidAttachments.findAndReplaceWithLast(handle);
	}

	PxU32 PxgSimulationController::addVolumeVolumeAttachments(Dy::DeformableVolume* deformableVolume0,
		PxDeformableAttachmentTargetType::Enum feature0, const PxU32* indices0, const PxVec4* barycentrics0,
		Dy::DeformableVolume* deformableVolume1, PxDeformableAttachmentTargetType::Enum feature1,
		const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count, const bool addToActive)
	{
		const PxU32 handle = mVolumeVolumeAttachments.addVolumeVolumeAttachments(deformableVolume0->getGpuRemapId(),
			feature0, indices0, barycentrics0, deformableVolume1->getGpuRemapId(), feature1, indices1, barycentrics1,
			count, addToActive);

		deformableVolume0->mVolumeAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeVolumeVolumeAttachments(Dy::DeformableVolume* deformableVolume0, PxU32 handle)
	{
		mVolumeVolumeAttachments.removeAttachments(handle);
		deformableVolume0->mVolumeAttachments.findAndReplaceWithLast(handle);
	}

	PxU32 PxgSimulationController::addVolumeSurfaceAttachments(Dy::DeformableVolume* deformableVolume,
		PxDeformableAttachmentTargetType::Enum feature0, const PxU32* indices0, const PxVec4* barycentrics0,
		Dy::DeformableSurface* deformableSurface, PxDeformableAttachmentTargetType::Enum feature1,
		const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count, const bool isActive)
	{
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		const PxU32 handle = mVolumeSurfaceAttachments.addVolumeSurfaceAttachments(deformableVolume->getGpuRemapId(),
			feature0, indices0, barycentrics0, deformableSurface->getGpuRemapId(), surfaceMesh, feature1, indices1,
			barycentrics1, count, isActive);

		deformableVolume->mSurfaceAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeVolumeSurfaceAttachments(Dy::DeformableVolume* deformableVolume, PxU32 handle)
	{
		mVolumeSurfaceAttachments.removeAttachments(handle);
		deformableVolume->mSurfaceAttachments.findAndReplaceWithLast(handle);
	}

	void PxgSimulationController::addVolumeRigidFilters(Dy::DeformableVolume* deformableVolume,
		const PxNodeIndex& rigidNodeIndex, PxU32* tetIndices, PxU32 tetIndicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());

		mVolumeRigidFilters.updateVolumeRigid(volumeMesh, deformableVolume->getGpuRemapId(), rigidNodeIndex,
			tetIndices, tetIndicesSize, /*add*/ true);
	}

	void PxgSimulationController::removeVolumeRigidFilters(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& rigidNodeIndex,
		PxU32* tetIndices, PxU32 tetIndicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());

		mVolumeRigidFilters.updateVolumeRigid(volumeMesh, deformableVolume->getGpuRemapId(), rigidNodeIndex,
			tetIndices, tetIndicesSize, /*add*/ false);
	}

	void PxgSimulationController::addVolumeStaticShapeFilters(Dy::DeformableVolume* deformableVolume,
		const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* tetIndices, PxU32 tetIndicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());

		mVolumeRigidFilters.updateVolumeStaticShapes(volumeMesh, deformableVolume->getGpuRemapId(), shapeCacheIds, nbShapes,
			tetIndices, tetIndicesSize, /*add*/ true);
	}

	void PxgSimulationController::removeVolumeStaticShapeFilters(Dy::DeformableVolume* deformableVolume,
		const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* tetIndices, PxU32 tetIndicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());

		mVolumeRigidFilters.updateVolumeStaticShapes(volumeMesh, deformableVolume->getGpuRemapId(), shapeCacheIds, nbShapes,
			tetIndices, tetIndicesSize, /*add*/ false);
	}

	void PxgSimulationController::addVolumeVolumeFilters(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh0 =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume0->getCollisionMesh());
		const Gu::BVTetrahedronMesh& volumeMesh1 =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume1->getCollisionMesh());

		mVolumeVolumeFilters.updateVolumeVolume(volumeMesh0, deformableVolume0->getGpuRemapId(),
			volumeMesh1, deformableVolume1->getGpuRemapId(), tetIndices0, tetIndices1, tetIndicesSize, /*add*/ true);
	}

	void PxgSimulationController::removeVolumeVolumeFilters(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh0 =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume0->getCollisionMesh());
		const Gu::BVTetrahedronMesh& volumeMesh1 =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume1->getCollisionMesh());

		mVolumeVolumeFilters.updateVolumeVolume(volumeMesh0, deformableVolume0->getGpuRemapId(),
			volumeMesh1, deformableVolume1->getGpuRemapId(), tetIndices0, tetIndices1, tetIndicesSize, /*add*/ false);
	}

	void PxgSimulationController::addVolumeSurfaceFilters(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface,
		PxU32* triIndices, PxU32* tetIndices, PxU32 indicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		mVolumeSurfaceFilters.updateVolumeSurface(volumeMesh, deformableVolume->getGpuRemapId(),
			surfaceMesh, deformableSurface->getGpuRemapId(), triIndices, tetIndices, indicesSize, /*add*/ true);
	}

	void PxgSimulationController::removeVolumeSurfaceFilters(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface,
		PxU32* triIndices, PxU32* tetIndices, PxU32 indicesSize)
	{
		const Gu::BVTetrahedronMesh& volumeMesh =
			*static_cast<const Gu::BVTetrahedronMesh*>(deformableVolume->getCollisionMesh());
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		mVolumeSurfaceFilters.updateVolumeSurface(volumeMesh, deformableVolume->getGpuRemapId(),
			surfaceMesh, deformableSurface->getGpuRemapId(), triIndices, tetIndices, indicesSize, /*add*/ false);
	}

	PxU32 PxgSimulationController::addSurfaceRigidAttachments(Dy::DeformableSurface* deformableSurface, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex,
		PxDeformableAttachmentTargetType::Enum feature, const PxU32* indices, const PxVec4* barycentrics,
		const PxTransform& frame, const PxVec4* coords, PxU32 count, const bool isActive)
	{
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		const PxU32 handle = mSurfaceRigidAttachments.addSurfaceRigidAttachments(deformableSurface->getGpuRemapId(), surfaceMesh,
			feature, indices, barycentrics, rigidBody, rigidNodeIndex, frame, coords, count, isActive);

		deformableSurface->mRigidAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeSurfaceRigidAttachments(Dy::DeformableSurface* deformableSurface, PxU32 handle)
	{
		mSurfaceRigidAttachments.removeAttachments(handle);
		deformableSurface->mRigidAttachments.findAndReplaceWithLast(handle);
	}

	PxU32 PxgSimulationController::addSurfaceSurfaceAttachments(Dy::DeformableSurface* deformableSurface0,
		PxDeformableAttachmentTargetType::Enum feature0, const PxU32* indices0, const PxVec4* barycentrics0,
		Dy::DeformableSurface* deformableSurface1, PxDeformableAttachmentTargetType::Enum feature1,
		const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count, const bool addToActive)
	{
		const Gu::TriangleMesh& surfaceMesh0 = *static_cast<const Gu::TriangleMesh*>(deformableSurface0->getTriangleMesh());
		const Gu::TriangleMesh& surfaceMesh1 = *static_cast<const Gu::TriangleMesh*>(deformableSurface1->getTriangleMesh());

		const PxU32 handle = mSurfaceSurfaceAttachments.addSurfaceSurfaceAttachments(deformableSurface0->getGpuRemapId(),
			surfaceMesh0, feature0, indices0, barycentrics0, deformableSurface1->getGpuRemapId(), surfaceMesh1, feature1,
			indices1, barycentrics1, count, addToActive);

		// The island gen wakes both surfaces, so the batch only has to hang off one of them.
		deformableSurface0->mSurfaceAttachments.pushBack(handle);

		return handle;
	}

	void PxgSimulationController::removeSurfaceSurfaceAttachments(Dy::DeformableSurface* deformableSurface0, PxU32 handle)
	{
		mSurfaceSurfaceAttachments.removeAttachments(handle);

		// The batch hangs off surface 0, which is where it was added.
		deformableSurface0->mSurfaceAttachments.findAndReplaceWithLast(handle);
	}

	void PxgSimulationController::addSurfaceRigidFilters(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& rigidNodeIndex,
		PxU32* triIndices, PxU32 triIndicesSize)
	{
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		mSurfaceRigidFilters.updateSurfaceRigid(surfaceMesh, deformableSurface->getGpuRemapId(), rigidNodeIndex,
			triIndices, triIndicesSize, /*add*/ true);
	}

	void PxgSimulationController::removeSurfaceRigidFilters(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& rigidNodeIndex,
		PxU32* triIndices, PxU32 triIndicesSize)
	{
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		mSurfaceRigidFilters.updateSurfaceRigid(surfaceMesh, deformableSurface->getGpuRemapId(), rigidNodeIndex,
			triIndices, triIndicesSize, /*add*/ false);
	}

	void PxgSimulationController::addSurfaceStaticShapeFilters(Dy::DeformableSurface* deformableSurface,
		const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* triIndices, PxU32 triIndicesSize)
	{
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		mSurfaceRigidFilters.updateSurfaceStaticShapes(surfaceMesh, deformableSurface->getGpuRemapId(), shapeCacheIds, nbShapes,
			triIndices, triIndicesSize, /*add*/ true);
	}

	void PxgSimulationController::removeSurfaceStaticShapeFilters(Dy::DeformableSurface* deformableSurface,
		const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* triIndices, PxU32 triIndicesSize)
	{
		const Gu::TriangleMesh& surfaceMesh = *static_cast<const Gu::TriangleMesh*>(deformableSurface->getTriangleMesh());

		mSurfaceRigidFilters.updateSurfaceStaticShapes(surfaceMesh, deformableSurface->getGpuRemapId(), shapeCacheIds, nbShapes,
			triIndices, triIndicesSize, /*add*/ false);
	}

	void PxgSimulationController::addSurfaceSurfaceFilters(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1,
		PxU32* triIndices0, PxU32* triIndices1, PxU32 triIndicesSize)
	{
		const PxU32 surfaceId0 = deformableSurface0->getGpuRemapId();
		const PxU32 surfaceId1 = deformableSurface1->getGpuRemapId();

		// A filter only registers once both surfaces are in the scene, and addFEMCloth builds the
		// topology as each one enters.
		PX_ASSERT(mSurfaceTopologies[surfaceId0] && mSurfaceTopologies[surfaceId1]);

		const Gu::TriangleMesh& surfaceMesh0 = *static_cast<const Gu::TriangleMesh*>(deformableSurface0->getTriangleMesh());
		const Gu::TriangleMesh& surfaceMesh1 = *static_cast<const Gu::TriangleMesh*>(deformableSurface1->getTriangleMesh());

		mSurfaceSurfaceFilters.updateSurfaceSurface(surfaceMesh0, *mSurfaceTopologies[surfaceId0], surfaceId0,
			surfaceMesh1, *mSurfaceTopologies[surfaceId1], surfaceId1, triIndices0, triIndices1, triIndicesSize,
			/*add*/ true);
	}

	void PxgSimulationController::removeSurfaceSurfaceFilters(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1,
		PxU32* triIndices0, PxU32* triIndices1, PxU32 triIndicesSize)
	{
		const PxU32 surfaceId0 = deformableSurface0->getGpuRemapId();
		const PxU32 surfaceId1 = deformableSurface1->getGpuRemapId();

		// A filter only registers once both surfaces are in the scene, and addFEMCloth builds the
		// topology as each one enters.
		PX_ASSERT(mSurfaceTopologies[surfaceId0] && mSurfaceTopologies[surfaceId1]);

		const Gu::TriangleMesh& surfaceMesh0 = *static_cast<const Gu::TriangleMesh*>(deformableSurface0->getTriangleMesh());
		const Gu::TriangleMesh& surfaceMesh1 = *static_cast<const Gu::TriangleMesh*>(deformableSurface1->getTriangleMesh());

		mSurfaceSurfaceFilters.updateSurfaceSurface(surfaceMesh0, *mSurfaceTopologies[surfaceId0], surfaceId0,
			surfaceMesh1, *mSurfaceTopologies[surfaceId1], surfaceId1, triIndices0, triIndices1, triIndicesSize,
			/*add*/ false);
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

	void PxgSimulationController::ovdSnapshotRigidDynamicForces(const PxRigidDynamicGPUIndex* gpuIndices, PxU32 nbElements)
	{
		mSimulationCore->ovdSnapshotRigidDynamicForces(gpuIndices, nbElements);
	}

	void PxgSimulationController::ovdSnapshotArticulationForces(const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements)
	{
		PxgArticulationCore* artiCore = mDynamicContext->getArticulationCore();
		if (artiCore)
			artiCore->ovdSnapshotArticulationForces(gpuIndices, nbElements, mMaxLinks, mMaxDofs, mMaxFixedTendons, mMaxTendonJoints, mMaxSpatialTendons, mMaxAttachments);
	}
#endif
}


