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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "PxgFEMClothCore.h"
#include "CudaKernelWrangler.h"
#include "DyDeformableSurface.h"
#include "GuTriangleMesh.h"
#include "PxgArticulationCore.h"
#include "PxgContext.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgSimulationCore.h"
#include "PxgCudaSolverCore.h"
#include "PxgCudaUtils.h"
#include "PxgFEMCloth.h"
#include "PxgFEMClothCoreKernelIndices.h"
#include "PxgKernelIndices.h"
#include "PxgKernelWrangler.h"
#include "PxgNarrowphaseCore.h"
#include "PxgNpKernelIndices.h"
#include "PxgNphaseImplementationContext.h"
#include "PxgPBDParticleSystemCore.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgSimulationController.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxsContext.h"
#include "common/PxProfileZone.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "foundation/PxAssert.h"

#include "PxDeformableSurfaceFlag.h"

#define CLOTH_GPU_DEBUG 0

namespace physx
{
	extern "C" void initFEMClothKernels0();
	extern "C" void initFEMClothKernels1();
	extern "C" void initFEMClothKernels2();

	void createPxgFEMCloth()
	{
	#if !PX_PHYSX_GPU_EXPORTS
		// this call is needed to force PhysXSimulationControllerGpu linkage as Static Library!
		initFEMClothKernels0();
		initFEMClothKernels1();
		initFEMClothKernels2();
	#endif
	}

	PxgFEMClothCore::PxgFEMClothCore(PxgCudaKernelWranglerManager* gpuKernelWrangler,
									 PxCudaContextManager* cudaContextManager,
									 PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController,
									 PxgGpuContext* gpuContext, PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS)
	: PxgFEMCore(gpuKernelWrangler, cudaContextManager, heapMemoryManager, simController, gpuContext, maxContacts, collisionStackSize,
				 isTGS, PxsHeapStats::eSHARED_FEMCLOTH), 
		mUpdateClothContactPairs(heapMemoryManager, PxsHeapStats::eSHARED_FEMCLOTH),
		mPostSolveCallback(NULL)
	{
		mCudaContextManager->acquireContext();
		mGpuContext->mGpuFEMClothCore = this;

		int leastPriority, mostPriority;
		cuCtxGetStreamPriorityRange(&leastPriority, &mostPriority);

		mCudaContext->streamCreateWithPriority(&mStream, CU_STREAM_NON_BLOCKING, leastPriority);

		mUpdateClothContactPairs.allocate(sizeof(PxU8), PX_FL);

		// KS - we divide by 32 because this is a block data format
		mFemConstraintBuf.allocate(((maxContacts + 31) / 32) * sizeof(PxgClothConstraintBlock), PX_FL);

		mCudaContext->eventCreate(&mBoundUpdateEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveClothEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveRigidEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mConstraintPrepParticleEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveParticleEvent, CU_EVENT_DISABLE_TIMING);

		mCudaContextManager->releaseContext();
	}

	PxgFEMClothCore::~PxgFEMClothCore()
	{
		mCudaContextManager->acquireContext();

		// destroy stream
		mCudaContext->eventDestroy(mBoundUpdateEvent);
		mBoundUpdateEvent = NULL;

		mCudaContext->eventDestroy(mSolveClothEvent);
		mSolveClothEvent = NULL;

		mCudaContext->eventDestroy(mSolveRigidEvent);
		mSolveRigidEvent = NULL;

		mCudaContext->eventDestroy(mConstraintPrepParticleEvent);
		mConstraintPrepParticleEvent = NULL;

		mCudaContext->eventDestroy(mSolveParticleEvent);
		mSolveParticleEvent = NULL;

		mCudaContextManager->releaseContext();
	}

	void PxgFEMClothCore::preIntegrateSystems(PxU32 nbActiveFEMCloths, const PxVec3& gravity, PxReal dt)
	{
		// integrateSystems run on the broad phase stream so we don't need to have an extra event to sync in updateBounds
		CUstream bpStream = mGpuContext->mGpuBp->getBpStream();
		PxgSimulationCore* core = mSimController->getSimulationCore();

		PxgCudaBuffer& femClothsBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(femClothsBuffer.getDevicePtr());
		PxU32* activeIndices = reinterpret_cast<PxU32*>(core->getActiveFEMClothBuffer().getDevicePtr());
		const PxU32 maxClothVerts = core->getMaxClothVerts();

		preIntegrateSystem(femClothsd, activeIndices, nbActiveFEMCloths, maxClothVerts, gravity, dt, bpStream);
	}

	void PxgFEMClothCore::preIntegrateSystem(PxgFEMCloth* femClothsd, PxU32* activeFemCloths, PxU32 nbActiveFemCloths,
											 PxU32 maxVertices, const PxVec3& gravity, PxReal dt, CUstream bpStream)
	{
		if(nbActiveFemCloths)
		{
			const PxU32 totalNumFemCloths = mGpuContext->getSimulationCore()->getNumTotalFEMCloths();

			// mSpeculativeCCDContactOffset is used for refitting the bound.
			// Note: Contact distance is *not* affected by mSpeculativeCCDContactOffset, as it can easily lead to contact buffer overflow.
			mSpeculativeCCDContactOffset.allocate(totalNumFemCloths * sizeof(PxReal), PX_FL);

			// Initialize mSpeculativeCCDContactOffset.
			PxgDevicePointer<PxReal> speculativeCCDContactOffsetd = mSpeculativeCCDContactOffset.getTypedDevicePtr();
			mCudaContext->memsetD32Async(speculativeCCDContactOffsetd.mPtr, 0, totalNumFemCloths * sizeof(PxReal) / sizeof(PxU32), bpStream);

			const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled() && mGpuContext->isTGS();

			PxgSimulationCore* core = mSimController->getSimulationCore();
			const PxU32 maxNbCollisionPairUpdatesPerTimestep = core->getMaxNbCollisionPairUpdatesPerTimestep();

			// If maxNbCollisionPairUpdatesPerTimestep is zero, update collision pairs adaptively and automatically. Otherwise, update them
			// maxNbCollisionPairUpdatesPerTimestep times per time step.
			const bool adaptiveCollisionPairUpdate = maxNbCollisionPairUpdatesPerTimestep == 0u;

			PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();
			
			PxgDevicePointer<PxU32> totalFemContactCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();

			CUfunction preIntegrateKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_PREINTEGRATION);

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlock = PxgFEMClothKernelBlockDim::CLOTH_PREINTEGRATION / numThreadsPerWarp;
			const PxU32 numBlocks = (maxVertices + PxgFEMClothKernelBlockDim::CLOTH_PREINTEGRATION - 1) /
									PxgFEMClothKernelBlockDim::CLOTH_PREINTEGRATION;

			PxCudaKernelParam kernelParams[] =
			{ 
				PX_CUDA_KERNEL_PARAM(femClothsd), 
				PX_CUDA_KERNEL_PARAM(activeFemCloths),
				PX_CUDA_KERNEL_PARAM(gravity),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(mIsTGS),
				PX_CUDA_KERNEL_PARAM(speculativeCCDContactOffsetd),
				PX_CUDA_KERNEL_PARAM(externalForcesEveryTgsIterationEnabled),
				PX_CUDA_KERNEL_PARAM(adaptiveCollisionPairUpdate),
				PX_CUDA_KERNEL_PARAM(updateClothContactPairsd),
				PX_CUDA_KERNEL_PARAM(totalFemContactCountsd)
			};

			CUresult result = mCudaContext->launchKernel(preIntegrateKernelFunction, numBlocks, nbActiveFemCloths, 1, numThreadsPerWarp,
														 numWarpsPerBlock, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(bpStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_preIntegrateLaunch kernel fail!\n");
	#endif
		}
	}

	void PxgFEMClothCore::prepRigidAttachmentConstraints(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxReal /*invDt*/,
		PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream stream)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbActiveRigidClothAttachments();

		if(nbRigidAttachments)
		{
			PxgDevicePointer<PxgFEMCloth> clothesd = simCore->getFEMClothBuffer().getTypedDevicePtr();
			PxgDevicePointer<PxgFEMRigidAttachment> rigidAttachments = simCore->getRigidClothAttachments();
			PxgDevicePointer<PxU32> activeRigidAttachments = simCore->getActiveRigidClothAttachments();
			PxgDevicePointer<PxgFEMRigidAttachmentConstraint> constraintsd = simCore->getClothRigidConstraints();
			PxgDevicePointer<PxNodeIndex> rigidAttachmentIds = simCore->getClothRigidAttachmentIds();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PX_ASSERT(simCore->getNbActiveRigidClothAttachments() * 2 <= mRigidDeltaVelBuf.getNbElements());

			// prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					PxgKernelIds::CLOTH_RIGID_ATTACHMENT_CONSTRAINT_PREP);
				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd),
													 PX_CUDA_KERNEL_PARAM(rigidAttachments),
													 PX_CUDA_KERNEL_PARAM(activeRigidAttachments),
													 PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
													 PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
													 PX_CUDA_KERNEL_PARAM(constraintsd),
													 PX_CUDA_KERNEL_PARAM(prePrepDescd),
													 PX_CUDA_KERNEL_PARAM(prepDescd),
													 PX_CUDA_KERNEL_PARAM(sharedDescd),
													 PX_CUDA_KERNEL_PARAM(deltaVd) };

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result =
					mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0,
											   stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU cloth_rigidAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);

	#endif
			}
		}
	}

	void PxgFEMClothCore::prepClothAttachmentConstraints(CUstream stream)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		const PxU32 nbClothAttachments = simCore->getNbActiveClothClothAttachments();

		if(nbClothAttachments)
		{
			PxgDevicePointer<PxgFEMFEMAttachment> clothAttachments = simCore->getClothClothAttachments();
			PxgDevicePointer<PxU32> activeClothAttachments = simCore->getActiveClothClothAttachments();
			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getClothClothConstraints();
			// PxgDevicePointer<float4> clothesd = simCore->getFEMClothBuffer().getTypedDevicePtr();
			// PxgDevicePointer<PxU32> clothAttachmentIds = simCore->getClothClothAttachmentIds();

			// prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					PxgKernelIds::FEM_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothAttachments),
													 PX_CUDA_KERNEL_PARAM(activeClothAttachments),
													 PX_CUDA_KERNEL_PARAM(nbClothAttachments),
													 PX_CUDA_KERNEL_PARAM(constraintsd) };

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result =
					mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0,
											   stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU cloth_rigidAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);

	#endif
			}
		}
	}

	void PxgFEMClothCore::prepRigidContactConstraint(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxReal invDt,
													 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream, PxU32 numSolverBodies, PxU32 numArticulations)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		PxgDevicePointer<PxgFEMCloth> femClothesd = simCore->getFEMClothBuffer().getTypedDevicePtr();

		// prepare rigid body vs cloth contacts
		{
			PxgDevicePointer<float4> contactsd = mRigidSortedContactPointBuf.getTypedDevicePtr();
			PxgDevicePointer<float4> normalpensd = mRigidSortedContactNormalPenBuf.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentricsd = mRigidSortedContactBarycentricBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();

			PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

			PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<float4> rigidLambdaNs = mRigidLambdaNBuf.getTypedDevicePtr();
			PxgDevicePointer<float4> clothLambdaNs = mFemLambdaNBuf.getTypedDevicePtr();

			// Allocating femRigidContactCount based on the size of rigid bodies plus articulations.
			if (mFemRigidReferenceCount.getNbElements() != numSolverBodies + numArticulations)
			{
				mFemRigidReferenceCount.allocateElements(numSolverBodies + numArticulations, PX_FL);
			}

			const CUfunction rigidContactPrepKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_RIGID_CONTACTPREPARE);

			PxCudaKernelParam kernelParams[] = {
			    PX_CUDA_KERNEL_PARAM(femClothesd),   PX_CUDA_KERNEL_PARAM(contactsd),
			    PX_CUDA_KERNEL_PARAM(normalpensd),   PX_CUDA_KERNEL_PARAM(barycentricsd),
			    PX_CUDA_KERNEL_PARAM(contactInfosd), PX_CUDA_KERNEL_PARAM(totalContactCountsd),
			    PX_CUDA_KERNEL_PARAM(constraintsd),  PX_CUDA_KERNEL_PARAM(prePrepDescd),
			    PX_CUDA_KERNEL_PARAM(prepDescd),     PX_CUDA_KERNEL_PARAM(rigidLambdaNs),
			    PX_CUDA_KERNEL_PARAM(clothLambdaNs), PX_CUDA_KERNEL_PARAM(invDt),
			    PX_CUDA_KERNEL_PARAM(sharedDescd),   PX_CUDA_KERNEL_PARAM(mIsTGS)
		    };

			CUresult result = mCudaContext->launchKernel(
				rigidContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
				PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(solverStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_rigidContactPrepareLaunch kernel fail!\n");

			int bob = 0;
			PX_UNUSED(bob);
	#endif
		}
	}

	// prepare soft body vs particle system contact constraints
	void PxgFEMClothCore::prepClothParticleConstraint()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		PxgDevicePointer<PxgFEMCloth> clothsd = simCore->getFEMClothBuffer().getTypedDevicePtr();

		PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();

		if(particleCore)
		{
			CUdeviceptr particlesystemsd = particleCore->getParticleSystemBuffer().getDevicePtr();

			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxU32> totalContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

			{
				PxgDevicePointer<float4> contactsd = mParticleSortedContactPointBuffer.getTypedDevicePtr();
				PxgDevicePointer<float4> normalpensd = mParticleSortedContactNormalPenBuffer.getTypedDevicePtr();
				PxgDevicePointer<float4> barycentricd = mParticleSortedContactBarycentricBuffer.getTypedDevicePtr();

				PxgDevicePointer<PxgFEMParticleConstraintBlock> constraintsd = mParticleConstraintBuf.getTypedDevicePtr();
				PxgDevicePointer<float4> clothAppliedForced = mParticleAppliedFEMForcesBuf.getTypedDevicePtr();
				PxgDevicePointer<float4> particleAppliedForced = mParticleAppliedParticleForcesBuf.getTypedDevicePtr();

				const CUfunction femContactPrepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					PxgKernelIds::CLOTH_PARTICLE_CONTACTPREPARE);

				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothsd),
													 PX_CUDA_KERNEL_PARAM(particlesystemsd),
													 PX_CUDA_KERNEL_PARAM(contactsd),
													 PX_CUDA_KERNEL_PARAM(normalpensd),
													 PX_CUDA_KERNEL_PARAM(barycentricd),
													 PX_CUDA_KERNEL_PARAM(contactInfosd),
													 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
													 PX_CUDA_KERNEL_PARAM(constraintsd),
													 PX_CUDA_KERNEL_PARAM(clothAppliedForced),
													 PX_CUDA_KERNEL_PARAM(particleAppliedForced) };

				CUresult result = mCudaContext->launchKernel(
					femContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
					PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sb_particleContactPrepareLaunch first pass kernel fail!\n");
	#endif
			}

			// PxgParticleSystemCore* particleSystemCore = mSimController->getParticleSystemCore();
			// CUstream particleStream = particleSystemCore->getStream();
			// compute start and end index for sorted contact based on particle id
			{
				////particley stream need to wait till soft body vs particle constraint
				// mCudaContext->streamWaitEvent(particleStream, mConstraintPrepSoftBodyParticleEvent);

				// PxgDevicePointer<float4> contactsd = mSPSortedContactPointBuffer.getTypedDevicePtr();
				PxgDevicePointer<PxU32> blockOffsetd = mTempBlockCellsHistogramBuf.getTypedDevicePtr();
				CUdeviceptr offsetd = mTempCellsHistogramBuf.getDevicePtr();
				PxgDevicePointer<PxU32> pairCountd = mTempHistogramCountBuf.getTypedDevicePtr();
				PxgDevicePointer<PxU32> startd = mTempContactBuf.getTypedDevicePtr();
				PxgDevicePointer<PxU32> endd = mTempContactRemapBuf.getTypedDevicePtr();

				// compute blockOffset and offset array for particle
				{
					const CUfunction findStartEndFirstKernelFunction =
						mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
							PxgKernelIds::PS_FIND_RANGESTARTEND_FEM_FIRST);

					PxCudaKernelParam kernelParams[] = {
						PX_CUDA_KERNEL_PARAM(particlesystemsd),
						PX_CUDA_KERNEL_PARAM(contactInfosd),
						PX_CUDA_KERNEL_PARAM(totalContactCountsd),
						PX_CUDA_KERNEL_PARAM(blockOffsetd),
						PX_CUDA_KERNEL_PARAM(offsetd),
					};

					const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA;
					const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA;
					CUresult result =
						mCudaContext->launchKernel(findStartEndFirstKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1,
												   1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
	#if CLOTH_GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if(result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
												"GPU ps_findStartEndParticleFirst kernel fail!\n");

	#endif
				}

				// compute start and end range for particle
				{
					const CUfunction findStartEndSecondKernelFunction =
						mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
							PxgKernelIds::PS_RANGESTARTEND_FEM_SECONE);

					PxCudaKernelParam kernelParams[] = {
						PX_CUDA_KERNEL_PARAM(contactInfosd), PX_CUDA_KERNEL_PARAM(totalContactCountsd),
						PX_CUDA_KERNEL_PARAM(blockOffsetd),  PX_CUDA_KERNEL_PARAM(offsetd),
						PX_CUDA_KERNEL_PARAM(pairCountd),    PX_CUDA_KERNEL_PARAM(startd),
						PX_CUDA_KERNEL_PARAM(endd)
					};

					const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA;
					const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA;
					CUresult result =
						mCudaContext->launchKernel(findStartEndSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock,
												   1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
	#if CLOTH_GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if(result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
												"GPU ps_findStartEndParticleSecond kernel fail!\n");

	#endif
				}
			}
		}

		mCudaContext->eventRecord(mConstraintPrepParticleEvent, mStream);
	}

	void PxgFEMClothCore::prepClothContactConstraint()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		PxgDevicePointer<PxgFEMCloth> clothesd = simCore->getFEMClothBuffer().getTypedDevicePtr();
		{
			PxgDevicePointer<float4> contactsd = mFemContactPointBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> normalpensd = mFemContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentric0d = mFemContactBarycentric0Buffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentric1d = mFemContactBarycentric1Buffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mFemContactInfoBuffer.getTypedDevicePtr();

			PxgDevicePointer<PxU32> totalContactCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();

			CUdeviceptr constraintsd = mFemConstraintBuf.getDevicePtr();
			const CUfunction clothContactPrepKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_CLOTH_CONTACTPREPARE);

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd),
												 PX_CUDA_KERNEL_PARAM(contactsd),
												 PX_CUDA_KERNEL_PARAM(normalpensd),
												 PX_CUDA_KERNEL_PARAM(barycentric0d),
												 PX_CUDA_KERNEL_PARAM(barycentric1d),
												 PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
												 PX_CUDA_KERNEL_PARAM(mMaxContacts),
												 PX_CUDA_KERNEL_PARAM(constraintsd),
												 PX_CUDA_KERNEL_PARAM(materials),
												 PX_CUDA_KERNEL_PARAM(updateClothContactPairsd) };

			CUresult result = mCudaContext->launchKernel(
				clothContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
				PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_clothContactPrepareLaunch first pass kernel fail!\n");

			PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, totalContactCountsd, sizeof(PxU32));

			if(numContacts > 0)
			{
				int bob = 0;
				PX_UNUSED(bob);
			}
	#endif
		}
	}

	void PxgFEMClothCore::refitBound(PxU32 nbActiveFEMCloths, CUstream stream)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		CUdeviceptr femClothesd = core->getFEMClothBuffer().getDevicePtr();
		CUdeviceptr activeFemClothesd = core->getActiveFEMClothBuffer().getDevicePtr();
		PxgCudaBuffer& clothElementIndexBuffer = core->getFEMClothElementIndexBuffer();
		PxU32* clothElementIndexsd = reinterpret_cast<PxU32*>(clothElementIndexBuffer.getDevicePtr());

		CUdeviceptr boundsd = mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr();
		CUdeviceptr contactDistd = mGpuContext->mGpuBp->getContactDistBuffer().getDevicePtr();

		PxgDevicePointer<PxReal> speculativeCCDContactOffsetd = mSpeculativeCCDContactOffset.getTypedDevicePtr();
		PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();

		const PxU32 numBlocks = nbActiveFEMCloths;
		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlock = SB_REFIT_WAPRS_PER_BLOCK;
		const PxReal maxNbCollisionPairUpdatesPerTimestep =
			static_cast<PxReal>(core->getMaxNbCollisionPairUpdatesPerTimestep());

		const CUfunction refitBoundKernelFunction =
			mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_REFIT_BOUND);

		if(numBlocks)
		{
			PxCudaKernelParam kernelParams[] = {
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(activeFemClothesd),
				PX_CUDA_KERNEL_PARAM(nbActiveFEMCloths),
				PX_CUDA_KERNEL_PARAM(contactDistd),
				PX_CUDA_KERNEL_PARAM(speculativeCCDContactOffsetd),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(clothElementIndexsd),
				PX_CUDA_KERNEL_PARAM(maxNbCollisionPairUpdatesPerTimestep),
				PX_CUDA_KERNEL_PARAM(updateClothContactPairsd),
			};

			CUresult result =
				mCudaContext->launchKernel(refitBoundKernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock,
										   1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(stream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_refitBoundLaunch kernel fail!\n");

	#endif
		}
	}

	void PxgFEMClothCore::resetClothVsNonclothContactCounts()
	{
		// total contact count for particle and cloth
		PxgDevicePointer<PxU32> totalParticleContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();
		mCudaContext->memsetD32Async(totalParticleContactCountsd.mPtr, 0, 1, mStream);

		// total contact count for rigid body and cloth
		PxgDevicePointer<PxU32> totalRigidContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();
		mCudaContext->memsetD32Async(totalRigidContactCountsd.mPtr, 0, 1, mStream);

		mCudaContext->memsetD32Async(mStackSizeNeededOnDevice.getDevicePtr(), 0, 1, mStream);
	}

	void PxgFEMClothCore::checkBufferOverflows()
	{
		PxU32 contactCountNeeded = PxMax(*mParticleContactCountPrevTimestep, PxMax(*mRigidContactCountPrevTimestep, *mFemContactCountPrevTimestep));
		if (contactCountNeeded >= mMaxContacts) 
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "Deformable surface contact buffer overflow detected, please increase PxGpuDynamicsMemoryConfig::maxDeformableSurfaceContacts to at least %u\n", contactCountNeeded);
		}

		if (*mStackSizeNeededPinned > mCollisionStackSizeBytes)
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow detected, please increase its size to at least %i in the scene desc! Contacts have been dropped.\n", *mStackSizeNeededPinned);
		}

#if PX_ENABLE_SIM_STATS
		mContactCountStats = PxMax(mContactCountStats, contactCountNeeded);
		mGpuContext->getSimStats().mGpuDynamicsDeformableSurfaceContacts = mContactCountStats;

		mCollisionStackSizeBytesStats = PxMax(*mStackSizeNeededPinned, mCollisionStackSizeBytesStats);
		mGpuContext->getSimStats().mGpuDynamicsCollisionStackSize = PxMax(mCollisionStackSizeBytesStats, mGpuContext->getSimStats().mGpuDynamicsCollisionStackSize); // max because we also write this from other places.
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	}

	void PxgFEMClothCore::updateClothContactPairValidity()
	{
		PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();
		PxgDevicePointer<PxU32> totalFemContactCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& femClothBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(femClothBuffer.getDevicePtr());

		CUdeviceptr activeFEMClothsd = core->getActiveFEMClothBuffer().getDevicePtr();
		const PxU32 maxClothVerts = core->getMaxClothVerts();
		const PxU32 nbActiveFEMCloths = mSimController->getNbActiveFEMCloths();

		// Update contact validity
		{
			CUfunction updateValidityKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_UPDATE_CLOTH_CONTACT_VALIDITY);

			const PxU32 numBlocks = (maxClothVerts + PxgFEMClothKernelBlockDim::CLOTH_STEP - 1) / PxgFEMClothKernelBlockDim::CLOTH_STEP;

			{
				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothsd), PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
													 PX_CUDA_KERNEL_PARAM(updateClothContactPairsd),
													 PX_CUDA_KERNEL_PARAM(totalFemContactCountsd) };

				CUresult result = mCudaContext->launchKernel(updateValidityKernelFunction, numBlocks, nbActiveFEMCloths, 1,
															 PxgFEMClothKernelBlockDim::CLOTH_STEP, 1, 1, 0, mStream, kernelParams,
															 sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_updateClothContactValidityLaunch kernel fail!\n");
#endif
			}
		}
	}

	void PxgFEMClothCore::selfCollision()
	{
		const PxU32 nbActiveClothes = mSimController->getNbActiveFEMCloths();

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& clothesBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(clothesBuffer.getDevicePtr());
		PxReal* contactDistd = reinterpret_cast<PxReal*>(mGpuContext->mGpuBp->getContactDistBuffer().getDevicePtr());
		PxU32* activeClothesd = reinterpret_cast<PxU32*>(core->getActiveFEMClothBuffer().getDevicePtr());

	#if 0
			const PxU32 stackSizeBytes = 64 * 1024 * 1024;
			CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));

			//initialize gpu variables
			CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
			mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, mStream);

			if (0)
			{
				CUfunction clothMidphaseFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SELFCOLLISION_MIDPHASE);

				PxCudaKernelParam clothMidphaseKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(activeClothesd),
					PX_CUDA_KERNEL_PARAM(contactDistd),
					PX_CUDA_KERNEL_PARAM(clothesd),
					PX_CUDA_KERNEL_PARAM(stackSizeBytes),
					PX_CUDA_KERNEL_PARAM(gpuIntermStack),
					PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
				};

				PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
				PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;

				CUresult result = mCudaContext->launchKernel(clothMidphaseFunction, numBlocks, nbActiveClothes, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, clothMidphaseKernelParams, sizeof(clothMidphaseKernelParams), 0);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_selfCollisionMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_selfCollisionMidphaseGeneratePairsLaunch fail!!\n");

				PxU32 numPairs;
				mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

				if (numPairs > 0)
				{
					int bob = 0;
					PX_UNUSED(bob);
				}
	#endif
			}

			if (0)
			{
				CUdeviceptr contactsd = mFemContactPointBuffer.getDevicePtr();
				CUdeviceptr normalsd = mFemContactNormalPenBuffer.getDevicePtr();
				CUdeviceptr barycentric0d = mFemContactBarycentric0Buffer.getDevicePtr();
				CUdeviceptr barycentric1d = mFemContactBarycentric1Buffer.getDevicePtr();
				CUdeviceptr contactInfosd = mFemContactInfoBuffer.getDevicePtr();
				CUdeviceptr totalNumContactsd = mFemTotalContactCountBuffer.getDevicePtr();
				CUdeviceptr prevContactsd = mPrevFemContactCountBuffer.getDevicePtr();
				mCudaContext->memcpyDtoDAsync(prevContactsd, totalNumContactsd, sizeof(PxU32), mStream);

				CUfunction selfCollisionFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SELFCOLLISION_CG);

				PxCudaKernelParam selfCollisionKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(mMaxContacts),
					PX_CUDA_KERNEL_PARAM(contactDistd),
					PX_CUDA_KERNEL_PARAM(clothesd),
					PX_CUDA_KERNEL_PARAM(stackSizeBytes),
					PX_CUDA_KERNEL_PARAM(gpuIntermStack),
					PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
					PX_CUDA_KERNEL_PARAM(contactsd),
					PX_CUDA_KERNEL_PARAM(normalsd),
					PX_CUDA_KERNEL_PARAM(barycentric0d),
					PX_CUDA_KERNEL_PARAM(barycentric1d),
					PX_CUDA_KERNEL_PARAM(contactInfosd),
					PX_CUDA_KERNEL_PARAM(totalNumContactsd)
				};

				PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
				PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBCG;

				CUresult result = mCudaContext->launchKernel(selfCollisionFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, selfCollisionKernelParams, sizeof(selfCollisionKernelParams), 0, PX_FL);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_selfCollisionContactGenLaunch fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_selfCollisionContactGenLaunch fail!!\n");

				PxU32 numContacts;
				mCudaContext->memcpyDtoH(&numContacts, totalNumContactsd, sizeof(PxU32));

				if (numContacts > 0)
				{
					int bob = 0;
					PX_UNUSED(bob);
				}
	#endif

			}
			mIntermStackAlloc.reset();

	#endif

		PxgDevicePointer<PxgFemContactInfo> contactInfosd = mFemContactInfoBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxU32> totalNumContactsd = mFemTotalContactCountBuffer.getTypedDevicePtr();

		PxgDevicePointer<PxgNonRigidFilterPair> pairs = core->getClothClothVertTriFilters();
		const PxU32 nbPairs = core->getNbClothClothVertTriFilters();
		CUfunction selfCollisionFunction =
			mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_VERTEXSELFCOLLISION_MIDPHASE);

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;

		PxgDevicePointer<float4> contactsd = mFemContactPointBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> normalsd = mFemContactNormalPenBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentric0d = mFemContactBarycentric0Buffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentric1d = mFemContactBarycentric1Buffer.getTypedDevicePtr();

		//PxgDevicePointer<PxU32> prevContactsd = mPrevFemContactCountBuffer.getTypedDevicePtr();
		//mCudaContext->memcpyDtoDAsync(prevContactsd.mPtr, totalNumContactsd.mPtr, sizeof(PxU32), mStream);

		PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();
		PxCudaKernelParam selfCollisionKernelParams[] = {
			PX_CUDA_KERNEL_PARAM(mMaxContacts),
			PX_CUDA_KERNEL_PARAM(activeClothesd),
			PX_CUDA_KERNEL_PARAM(contactDistd),
			PX_CUDA_KERNEL_PARAM(clothesd),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(updateClothContactPairsd),
			PX_CUDA_KERNEL_PARAM(contactsd),
			PX_CUDA_KERNEL_PARAM(normalsd),
			PX_CUDA_KERNEL_PARAM(barycentric0d),
			PX_CUDA_KERNEL_PARAM(barycentric1d),
			PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(totalNumContactsd),
		};

		CUresult result =
			mCudaContext->launchKernel(selfCollisionFunction, numBlocks, nbActiveClothes, 1, WARP_SIZE, numWarpsPerBlock, 1,
									   0, mStream, selfCollisionKernelParams, sizeof(selfCollisionKernelParams), 0, PX_FL);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
									"GPU cloth_selfCollisionContactGenLaunch fail to launch kernel!!\n");


	#if CLOTH_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_selfCollisionContactGenLaunch fail!!\n");

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumContactsd, sizeof(PxU32));

		if(numContacts > 0)
		{
			int bob = 0;
			PX_UNUSED(bob);
		}
	#endif

	#if 0
			mCudaContext->streamSynchronize(mStream);
			PxU32 numFEMClothContactCounts;
			mCudaContext->memcpyDtoH(&numFEMClothContactCounts, totalNumContactsd, sizeof(PxU32));

			if (numFEMClothContactCounts >= mMaxContacts)
			{
				printf("contact counts: %i / %i : %f\n", numFEMClothContactCounts, mMaxContacts, PxReal(numFEMClothContactCounts) / PxReal(mMaxContacts));
				exit(0);
			}
	#endif
	}

	void PxgFEMClothCore::differentClothCollision()
	{
		GPU_BUCKET_ID::Enum type = GPU_BUCKET_ID::eFemClothes;
		PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
		PxgContactManagers& managers = npCore->getExistingContactManagers(type);
		PxgGpuContactManagers& gpuManagers = npCore->getExistingGpuContactManagers(type);
		const PxU32 numTests = managers.mCpuContactManagerMapping.size();

		if(numTests)
		{
			const PxReal toleranceLength = npCore->mNphaseImplContext->getToleranceLength();

			const PxgContactManagerInput* cmInputs =
				reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
			PX_ASSERT(cmInputs);

			const PxsCachedTransform* transformCached =
				reinterpret_cast<PxsCachedTransform*>(npCore->mGpuTransformCache.getDevicePtr());
			const PxReal* contactDistanced = reinterpret_cast<PxReal*>(npCore->mGpuContactDistance.getDevicePtr());
			PX_ASSERT(transformCached);

			PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(npCore->mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgDevicePointer<PxgFEMCloth> femClothesd = core->getFEMClothBuffer().getTypedDevicePtr();
			PxgDevicePointer<PxgNonRigidFilterPair> filterVertTriPairs = core->getClothClothVertTriFilters();
			const PxU32 nbFilterVertTriPairs = core->getNbClothClothVertTriFilters();

			CUresult result;
			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mFemContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxU32> totalNumCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();

			CUfunction fcCollisionKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_CLOTHVERTEXCOLLISION);

			PxgDevicePointer<float4> contactsd = mFemContactPointBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> normalPensd = mFemContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentrics0d = mFemContactBarycentric0Buffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentrics1d = mFemContactBarycentric1Buffer.getTypedDevicePtr();
			PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(mMaxContacts),
												 PX_CUDA_KERNEL_PARAM(toleranceLength),
												 PX_CUDA_KERNEL_PARAM(cmInputs),
												 PX_CUDA_KERNEL_PARAM(transformCached),
												 PX_CUDA_KERNEL_PARAM(contactDistanced),
												 PX_CUDA_KERNEL_PARAM(gpuShapes),
												 PX_CUDA_KERNEL_PARAM(femClothesd),
												 PX_CUDA_KERNEL_PARAM(filterVertTriPairs),
												 PX_CUDA_KERNEL_PARAM(nbFilterVertTriPairs),
												 PX_CUDA_KERNEL_PARAM(updateClothContactPairsd),
												 PX_CUDA_KERNEL_PARAM(contactsd),
												 PX_CUDA_KERNEL_PARAM(normalPensd),
												 PX_CUDA_KERNEL_PARAM(barycentrics0d),
												 PX_CUDA_KERNEL_PARAM(barycentrics1d),
												 PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(totalNumCountsd)

			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;

			// each warp deal with one test.
			result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, numTests, 1, WARP_SIZE,
												numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_clothVertexCollisionLaunch fail to launch kernel!!\n");

	#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(mStream);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_clothVertexCollisionLaunch kernel fail!!!\n");
	#endif

	#if 0
				mCudaContext->streamSynchronize(mStream);
				PxU32 numFEMClothContactCounts;
				mCudaContext->memcpyDtoH(&numFEMClothContactCounts, totalNumCountsd, sizeof(PxU32));

				if (numFEMClothContactCounts >= mMaxContacts)
				{
					printf("contact counts: %i / %i : %f\n", numFEMClothContactCounts, mMaxContacts, PxReal(numFEMClothContactCounts) / PxReal(mMaxContacts));
					exit(0);
				}
	#endif
		}
	}

	void PxgFEMClothCore::clampContactCounts()
	{
		// AD: we might want to do this with one kernel for all cloths.
		PxgDevicePointer<PxU32> totalFemNumCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxU32> totalRigidNumCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> totalParticleNumCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

		// This might exceed the max contact size, we need to clamp the contact count after contact gen
		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(totalFemNumCountsd),
											 PX_CUDA_KERNEL_PARAM(totalRigidNumCountsd),
											 PX_CUDA_KERNEL_PARAM(totalParticleNumCountsd),
											 PX_CUDA_KERNEL_PARAM(mMaxContacts) };

		CUfunction clampCountactCountFun =
			mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLAMP_MAX_VALUES);

		CUresult resultR = mCudaContext->launchKernel(clampCountactCountFun, 1, 1, 1, 1, 1, 1, 0, mStream, kernelParams,
													  sizeof(kernelParams), 0, PX_FL);
		if(resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU clampMaxValues fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU clampMaxValues fail!!\n");
	#endif
	}

	void PxgFEMClothCore::sortContacts(PxU32 nbActiveFemClothes)
	{
		clampContactCounts();

		// we need 2x rsDesc on the host fem cloth. The reason for this is that, while the sorting occurs synchronously
		// on the same stream on the device, the host-side buffers could get changed prior to the DMAs having occurred
		// due to device latency
		const PxU32 nbRequired = (nbActiveFemClothes) + 2u;
		mRSDesc.resize(nbRequired * 2u);

		mRadixCountTotalBuf.allocate(mRadixCountSize * nbRequired, PX_FL);

		for(PxU32 i = 0; i < 2; ++i)
		{
			mRadixSortDescBuf[i].allocate(sizeof(PxgRadixSortBlockDesc) * nbRequired, PX_FL);
		}

		// total number of rigid vs fem cloth contacts
		PxgDevicePointer<PxU32> totalRFContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		// total number of particle vs fem cloth contacts
		PxgDevicePointer<PxU32> totalPFContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

		// sort contacts based on rigid id
		PxgDevicePointer<PxU32> inputKeyd = mTempContactByRigidBitBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> inputRankd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputKeyd = mTempContactBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputRankd = mTempContactRemapBuf.getTypedDevicePtr();

		updateGPURadixSortBlockDesc(mStream, inputKeyd, inputRankd, outputKeyd, outputRankd,
									mRadixCountTotalBuf.getDevicePtr(), totalRFContactCountsd,
									&mRSDesc[2 * nbActiveFemClothes], mRadixSortDescBuf[0].getDevicePtr(),
									mRadixSortDescBuf[1].getDevicePtr());

		// sorted contacts based on particle id
		PxgDevicePointer<PxU32> inputKeyd2 = mTempContactByParticleBitBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> inputRankd2 = mContactRemapSortedByParticleBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputKeyd2 = mTempContactBuf2.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputRankd2 = mTempContactRemapBuf2.getTypedDevicePtr();

		updateGPURadixSortBlockDesc(mStream, inputKeyd2, inputRankd2, outputKeyd2, outputRankd2,
									mRadixCountTotalBuf.getDevicePtr() + mRadixCountSize, totalPFContactCountsd,
									&mRSDesc[2 * nbActiveFemClothes + 2],
									mRadixSortDescBuf[0].getDevicePtr() + sizeof(PxgRadixSortBlockDesc),
									mRadixSortDescBuf[1].getDevicePtr() + sizeof(PxgRadixSortBlockDesc));

		PxgCudaBuffer* radixSortDescBuf = mRadixSortDescBuf.begin();

		CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK);
		CUfunction calculateRanksFunction =
			mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK);

		{
			PxU32 startBit = 0;
			const PxU32 numPass = 8;

			for(PxU32 i = 0; i < numPass; ++i)
			{
				const PxU32 descIndex = i & 1;

				CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

				PxCudaKernelParam radixSortKernelParams[] = { PX_CUDA_KERNEL_PARAM(rsDesc), PX_CUDA_KERNEL_PARAM(startBit) };

				CUresult resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1,
															  PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream,
															  radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if(resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU fem cloth sortContacts fail to launch kernel!!\n");

				resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1,
													 PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream,
													 radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if(resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU fem cloth sortContacts fail to launch kernel!!\n");

				startBit += 4;
			}
	#if CLOTH_GPU_DEBUG
			CUresult result = mCudaContext->streamSynchronize(mStream);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortFemClothContacts fail!!\n");
	#endif
		}

		PxgDevicePointer<PxNodeIndex> contactByRigidd = mContactByRigidBuf.getTypedDevicePtr();

		{
			// copy the higher 32 bit to the temp contact rigid index buffer
			PxgDevicePointer<PxU32> tempContactByRigidd = mTempContactByRigidBitBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> rankd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();

			CUfunction copyFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_HIGH_32BITS);

			PxCudaKernelParam copyKernelParams[] = { PX_CUDA_KERNEL_PARAM(contactByRigidd),
													 PX_CUDA_KERNEL_PARAM(tempContactByRigidd), PX_CUDA_KERNEL_PARAM(rankd),
													 PX_CUDA_KERNEL_PARAM(totalRFContactCountsd) };

			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1,
														  1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU radixSortCopyBits fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail!!\n");
	#endif
		}

		{
			// sort tempContactByRidid again
			PxU32 startBit = 0;
			const PxU32 numPass = 8;

			for(PxU32 i = 0; i < numPass; ++i)
			{
				const PxU32 descIndex = i & 1;

				CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

				PxCudaKernelParam radixSortKernelParams[] = { PX_CUDA_KERNEL_PARAM(rsDesc), PX_CUDA_KERNEL_PARAM(startBit) };

				CUresult resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1,
															  PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream,
															  radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if(resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sortParticleContacts fail to launch kernel!!\n");

				resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1,
													 PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream,
													 radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if(resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sortParticleContacts fail to launch kernel!!\n");

				startBit += 4;
			}
		}

		{
			// copy the original rigidId to the sorted buffer based on mContactRemapSortedByRigidBuf
			PxgDevicePointer<PxNodeIndex> outContactByRigidd = mContactSortedByRigidBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> rankd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();

			CUfunction copyFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_VALUE);

			PxCudaKernelParam copyKernelParams[] = { PX_CUDA_KERNEL_PARAM(contactByRigidd),
													 PX_CUDA_KERNEL_PARAM(outContactByRigidd), PX_CUDA_KERNEL_PARAM(rankd),
													 PX_CUDA_KERNEL_PARAM(totalRFContactCountsd) };

			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1,
														  1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail!!\n");
	#endif
		}

		reorderRigidContacts();

		CUdeviceptr contactByParticled = mContactSortedByParticleBuf.getDevicePtr();

		{
			// copy the higher 32 bit to the team contact particle buffer
			CUdeviceptr tempContactByParticled = mTempContactByParticleBitBuf.getDevicePtr();
			CUdeviceptr rankd = mContactRemapSortedByParticleBuf.getDevicePtr();

			CUfunction copyFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_HIGH_32BITS);

			PxCudaKernelParam copyKernelParams[] = { PX_CUDA_KERNEL_PARAM(contactByParticled),
													 PX_CUDA_KERNEL_PARAM(tempContactByParticled),
													 PX_CUDA_KERNEL_PARAM(rankd),
													 PX_CUDA_KERNEL_PARAM(totalPFContactCountsd) };

			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1,
														  1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU radixSortCopyBits fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail!!\n");
	#endif
		}

		{
			// sort tempContactByParticled again
			PxU32 startBit = 0;
			const PxU32 numPass = 8;

			const PxU32 descSize = sizeof(PxgRadixSortBlockDesc);

			for(PxU32 i = 0; i < numPass; ++i)
			{
				const PxU32 descIndex = i & 1;

				CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr() + descSize;

				PxCudaKernelParam radixSortKernelParams[] = { PX_CUDA_KERNEL_PARAM(rsDesc), PX_CUDA_KERNEL_PARAM(startBit) };

				CUresult resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1,
															  PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream,
															  radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if(resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sortParticleContacts fail to launch kernel!!\n");

				resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1,
													 PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream,
													 radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if(resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sortParticleContacts fail to launch kernel!!\n");

				startBit += 4;
			}
		}

		{
			// copy the original rigidId to the sorted buffer based on mContactRemapSortedByRigidBuf
			CUdeviceptr outContactByParticled = mContactSortedByParticleBuf.getDevicePtr();
			CUdeviceptr rankd = mContactRemapSortedByParticleBuf.getDevicePtr();

			CUfunction copyFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_VALUE);

			PxCudaKernelParam copyKernelParams[] = { PX_CUDA_KERNEL_PARAM(contactByParticled),
													 PX_CUDA_KERNEL_PARAM(outContactByParticled), PX_CUDA_KERNEL_PARAM(rankd),
													 PX_CUDA_KERNEL_PARAM(totalPFContactCountsd) };

			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1,
														  1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail!!\n");
	#endif
		}

		{
			PxgDevicePointer<float4> contactsd = mParticleContactPointBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> normPensd = mParticleContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentricsd = mParticleContactBarycentricBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFemContactInfo> infosd = mParticleContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> sortedContactsd = mParticleSortedContactPointBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> sortedNormPensd = mParticleSortedContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> sortedBarycentricsd = mParticleSortedContactBarycentricBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFemContactInfo> sortedInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxU32> remapByParticleId = mContactRemapSortedByParticleBuf.getTypedDevicePtr();

			CUfunction reorderFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_REORDER_PS_CONTACTS);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(contactsd),
												 PX_CUDA_KERNEL_PARAM(normPensd),
												 PX_CUDA_KERNEL_PARAM(barycentricsd),
												 PX_CUDA_KERNEL_PARAM(infosd),
												 PX_CUDA_KERNEL_PARAM(totalPFContactCountsd),
												 PX_CUDA_KERNEL_PARAM(remapByParticleId),
												 PX_CUDA_KERNEL_PARAM(sortedContactsd),
												 PX_CUDA_KERNEL_PARAM(sortedNormPensd),
												 PX_CUDA_KERNEL_PARAM(sortedBarycentricsd),
												 PX_CUDA_KERNEL_PARAM(sortedInfosd) };

			CUresult resultR = mCudaContext->launchKernel(reorderFunction, PxgSoftBodyKernelGridDim::SB_REORDERCONTACTS, 1,
														  1, PxgSoftBodyKernelBlockDim::SB_REORDERCONTACTS, 1, 1, 0,
														  mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU fc_reorderPSContactsLaunch fail to launch kernel!!\n");

	#if CLOTH_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if(resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU fc_reorderParticleContactsLaunch fail!!\n");
	#endif
		}
	}

	void PxgFEMClothCore::preIteration()
	{
		const PxU32 nbActiveFEMCloths = mSimController->getNbActiveFEMCloths();
		if(nbActiveFEMCloths == 0)
			return;

		const PxU32* activeFEMCloths = mSimController->getActiveFEMCloths();
		PxgFEMCloth* femCloths = mSimController->getFEMCloths();

		for(PxU32 i = 0; i < nbActiveFEMCloths; ++i)
		{
			PxgFEMCloth& femCloth = femCloths[activeFEMCloths[i]];
			const PxU32 nbSharedTriangles = femCloth.mNbTriangles - femCloth.mNbNonSharedTriangles;

			if(femCloth.mNbNonSharedTriangles)
			{
				mCudaContext->memsetD32Async((CUdeviceptr)femCloth.mOrderedNonSharedTriangleLambdas, 0,
					(sizeof(float2) * femCloth.mNbNonSharedTriangles) / sizeof(PxU32), mStream);
			}

			if (femCloth.mNbSharedTrianglePairs)
			{
				mCudaContext->memsetD32Async((CUdeviceptr)femCloth.mOrderedSharedTriangleLambdas, 0,
					(sizeof(float2) * nbSharedTriangles) / sizeof(PxU32), mStream);

				mCudaContext->memsetD32Async((CUdeviceptr)femCloth.mSharedBendingLambdas, 0,
					(sizeof(float) * femCloth.mNbSharedTrianglePairs) / sizeof(PxU32), mStream);
			}

			if (femCloth.mNbNonSharedTrianglePairs)
			{
				mCudaContext->memsetD32Async((CUdeviceptr)femCloth.mNonSharedBendingLambdas, 0,
					(sizeof(float) * femCloth.mNbNonSharedTrianglePairs) / sizeof(PxU32), mStream);
			}
		}
	}

	void PxgFEMClothCore::constraintPrep(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxReal invDt,
										 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream, PxU32 numSolverBodies, PxU32 numArticulations)
	{
		PxgBodySimManager& bodySimManager = mSimController->getBodySimManager();

		const PxU32 nbActiveFemClothes = bodySimManager.mActiveFEMCloths.size();

		if(nbActiveFemClothes == 0)
			return;

		const PxU32 nbActiveParticleSystems = bodySimManager.mActivePBDParticleSystems.size();
		if(nbActiveParticleSystems != 0)
			prepClothParticleConstraint();

		// cloth contact constraints prep is done in solve()

		// wait for sorting to have completed on mStream before primitivePrep can run
		synchronizeStreams(mCudaContext, solverStream, mStream);

		// wait for DMA of prePrepDescd and prepDescd before rigid body vs soft body constraint prep can run
		synchronizeStreams(mCudaContext, mStream, solverStream);

		prepRigidContactConstraint(prePrepDescd, prepDescd, invDt, sharedDescd, solverStream, numSolverBodies, numArticulations);

		prepRigidAttachmentConstraints(prePrepDescd, prepDescd, invDt, sharedDescd, solverStream);

		prepClothAttachmentConstraints(solverStream);

		synchronizeStreams(mCudaContext, solverStream, mStream);
	}

	void PxgFEMClothCore::finalizeVelocities(PxReal dt)
	{
		const PxU32 nbActiveFEMCloths = mSimController->getBodySimManager().mActiveFEMCloths.size();
		if(nbActiveFEMCloths == 0)
			return;

		const PxU32 totalCloths = mSimController->getBodySimManager().mTotalNumFEMCloths;

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& femClothBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(femClothBuffer.getDevicePtr());

		PxU32* stateChanged = core->getActiveClothStateChangedMap().getWords();
		PxgDevicePointer<PxReal> wakeCountersGPU = core->getActiveClothWakeCountsGPU();
		PxReal* wakeCountersCPU = core->getActiveClothWakeCountsCPU();

		CUdeviceptr activeFEMClothsd = core->getActiveFEMClothBuffer().getDevicePtr();
		const PxU32 maxClothVerts = core->getMaxClothVerts();

		const PxReal invDt = 1.f / dt;

		const bool alwaysRunVelocityAveraging = mIsTGS && !mGpuContext->isExternalForcesEveryTgsIterationEnabled();

		{
			CUfunction finalizeKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_FINALIZE_VELOCITIES);

			const PxU32 numBlocks =
				(maxClothVerts + PxgFEMClothKernelBlockDim::CLOTH_STEP - 1) / PxgFEMClothKernelBlockDim::CLOTH_STEP;

			{
				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothsd),
													 PX_CUDA_KERNEL_PARAM(activeFEMClothsd), PX_CUDA_KERNEL_PARAM(invDt),
													 PX_CUDA_KERNEL_PARAM(dt), PX_CUDA_KERNEL_PARAM(alwaysRunVelocityAveraging) };

				CUresult result = mCudaContext->launchKernel(finalizeKernelFunction, numBlocks, nbActiveFEMCloths, 1,
															 PxgFEMClothKernelBlockDim::CLOTH_STEP, 1, 1, 0, mStream,
															 kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU cloth_finalizeVelocitiesLaunch kernel fail!\n");
	#endif
			}
		}


		{
			CUfunction sleepKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SLEEPING);

			const PxU32 numBlocks =
				(nbActiveFEMCloths + PxgFEMClothKernelBlockDim::CLOTH_STEP - 1) / PxgFEMClothKernelBlockDim::CLOTH_STEP;

			const PxReal resetCounter = 0.4f;
			{
				PxCudaKernelParam kernelParams[] = {
					PX_CUDA_KERNEL_PARAM(femClothsd),       PX_CUDA_KERNEL_PARAM(nbActiveFEMCloths),
					PX_CUDA_KERNEL_PARAM(activeFEMClothsd), PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(resetCounter),     PX_CUDA_KERNEL_PARAM(wakeCountersGPU),
					PX_CUDA_KERNEL_PARAM(stateChanged)
				};

				CUresult result =
					mCudaContext->launchKernel(sleepKernelFunction, numBlocks, 1, 1, PxgFEMClothKernelBlockDim::CLOTH_STEP,
											   1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_sleeping kernel fail!\n");
	#endif
			}

			mCudaContext->memcpyDtoHAsync(wakeCountersCPU, wakeCountersGPU.mPtr,
										  totalCloths * sizeof(PxReal), mStream);

			// record event - wait in host code beofre reading wake counters/state changed...
			mCudaContext->eventRecord(mFinalizeEvent, mStream);

			if (mPostSolveCallback)			
				mPostSolveCallback->onPostSolve(mFinalizeEvent);
		}
	}

	void PxgFEMClothCore::syncCloths()
	{
		PX_PROFILE_ZONE("PxgFEMClothCore::syncCloths", 0);
		mCudaContextManager->acquireContext();
		mCudaContext->eventSynchronize(mFinalizeEvent);
		mCudaContextManager->releaseContext();
	}

	void PxgFEMClothCore::createActivatedDeactivatedLists()
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxBitMapPinned& clothChangedMap = core->getActiveClothStateChangedMap();

		PxArray<Dy::DeformableSurface*>& deformableSurfaces = mSimController->getBodySimManager().mDeformableSurfaces;

		PxgBodySimManager& bodyManager = mSimController->getBodySimManager();

		PxReal* wakeCounters = core->getActiveClothWakeCountsCPU();

		mActivatingDeformableSurfaces.forceSize_Unsafe(0);
		mDeactivatingDeformableSurfaces.forceSize_Unsafe(0);

		PxBitMapPinned::Iterator iter(clothChangedMap);

		PxU32 dirtyIdx;
		while((dirtyIdx = iter.getNext()) != PxBitMapPinned::Iterator::DONE)
		{
			PX_ASSERT(dirtyIdx < bodyManager.mActiveFEMClothIndex.size());
			PxU32 idx = bodyManager.mActiveFEMCloths[dirtyIdx];
			if(wakeCounters[idx] == 0.f)
				mDeactivatingDeformableSurfaces.pushBack(deformableSurfaces[idx]);
			else
				mActivatingDeformableSurfaces.pushBack(deformableSurfaces[idx]);
		}

		clothChangedMap.clear();
	}

	void PxgFEMClothCore::solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
								PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
								PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt, CUstream solverStream, const PxU32 iter,
								const PxU32 maxIter, const bool isVelocityIteration, const PxVec3& gravity)
	{
		PX_UNUSED(isVelocityIteration);

		const PxU32 nbActiveFEMCloths = mSimController->getBodySimManager().mActiveFEMCloths.size();
		if(nbActiveFEMCloths == 0)
			return;

		// Ensure the relation v = (x - x0) / dt is maintained at all times, where x0 is:
		// TGS: The position at the beginning of each sub-timestep.
		// PGS: The position at the beginning of the entire time step.
		// Any velocity changes or filtering, if needed, are handled separately in solve_velocity().
		solve_position(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, dt, solverStream, iter, maxIter, gravity);

		// Apply additional velocity changes or filtering, such as cloth internal energy damping.
		solve_velocity(iter, maxIter, dt);

		synchronizeStreams(mCudaContext, mStream, solverStream);
	}

	// Ensure the relation v = (x - x0) / dt is maintained at all times.
	// Any velocity changes or filtering, if needed, are handled separately in solve_velocity().
	void PxgFEMClothCore::solve_position(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
										 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
										 PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt, CUstream solverStream,
										 const PxU32 iter, const PxU32 maxIter, const PxVec3& gravity)
	{
		const PxU32 nbActiveFEMCloths = mSimController->getBodySimManager().mActiveFEMCloths.size();

		PxgSimulationCore* core = mSimController->getSimulationCore();

		const PxU32 nbCollisionPairUpdatesPerTimestep = mIsTGS ? core->getMaxNbCollisionPairUpdatesPerTimestep() : 1;

		// If nbCollisionPairUpdatesPerTimestep is zero, update collision pairs adaptively and automatically. 
		// Otherwise, update the contact pairs "nbCollisionPairUpdatesPerTimestep" times per time step.
		const bool adaptiveCollisionPairUpdate = nbCollisionPairUpdatesPerTimestep == 0;

		// When adaptive updates are not used:
		// PGS: Update collision pairs only at the beginning of each time step.
		// TGS: Update collision pairs multiple times per time step, based on nbCollisionPairUpdatesPerTimestep.
		bool forceUpdateClothContactPairs = iter == 0; // PGS

		if(mIsTGS)
		{
			forceUpdateClothContactPairs = false;
			if(!adaptiveCollisionPairUpdate)
			{
				PxU32 divisor =
					static_cast<PxU32>(PxCeil(static_cast<PxReal>(maxIter) / static_cast<PxReal>(nbCollisionPairUpdatesPerTimestep)));
				divisor = PxMax(1u, divisor);

				if(iter % divisor == 0)
				{
					forceUpdateClothContactPairs = true;
				}
			}

			step(dt, mStream, nbActiveFEMCloths, gravity, forceUpdateClothContactPairs);
		}

		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());
		PxgDevicePointer<PxU32> activeFEMClothsd = core->getActiveFEMClothBuffer().getTypedDevicePtr();

		// Cloth-only: No iteraction with other actors.
		{
			// Cloth internal energies
			solveShellEnergy(femClothsd, activeFEMClothsd, nbActiveFEMCloths, dt);

			// Cloth attachment
			solveClothAttachmentDelta();

			// Apply delta changes + check cloth collision validity
			applyExternalDeltaAndCheckClothCollisionValidity(nbActiveFEMCloths, dt, adaptiveCollisionPairUpdate);

			// Cloth-cloth collision
			if(forceUpdateClothContactPairs || adaptiveCollisionPairUpdate)
				prepareClothClothCollision(forceUpdateClothContactPairs, adaptiveCollisionPairUpdate);

			solveClothClothCollision(nbActiveFEMCloths, dt);
		}

		// Interaction with particle system: outdated
		{
			const PxU32 nbActiveParticleSystem = mSimController->getBodySimManager().mActivePBDParticleSystems.size();

			if(nbActiveParticleSystem > 0)
			{
				PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();
				if(particleCore)
				{
					CUstream particleStream = particleCore->getStream();

					// Solve soft body vs particle contact in soft body stream
					solveParticleContactsOutputClothDelta(particleStream);

					// Solve soft body vs particle contact in particle stream
					solveParticleContactsOutputParticleDelta(particleStream);

					// FEM cloth stream need to wait till soft body vs particle finish in the particle stream
					mCudaContext->streamWaitEvent(mStream, mSolveParticleEvent);

					// Force particle stream to wait for FEM cloth body stream to finish before updating particle states
					synchronizeStreams(mCudaContext, mStream, particleStream);

					// This function is going to update the pos and vel for the FEM verts
					applyExternalDelta(nbActiveFEMCloths, dt, mStream);
				}
			}
		}

		// Interaction with rigid body
		{
			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Solve rigid attachment at cloth stream
			solveRigidAttachmentClothDelta(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, dt);

			// Solve rigid attachment at solver stream
			solveRigidAttachmentRigidDelta(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			// Apply delta changes
			applyExternalDelta(nbActiveFEMCloths, dt, mStream);

			synchronizeStreams(mCudaContext, solverStream, mStream);

			// Query reference count
			queryRigidContactReferenceCount(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, dt);

			// Solve rigid body contacts for cloth at cloth stream
			solveRigidContactsOutputClothDelta(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, dt);

			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Solve rigid body vs cloth contacts at solver stream
			solveRigidContactsOutputRigidDelta(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);

			// Cloth stream need to wait till cloth vs rigid finish in the solver stream
			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			// Apply delta changes + max velocity clamping to all vertices
			applyExternalDeltaWithVelocityClamping(nbActiveFEMCloths, dt, mStream);
		}
	}

	// Apply additional velocity changes or filtering, such as cloth internal energy damping.
	void PxgFEMClothCore::solve_velocity(PxU32 iter, PxU32 maxIter, PxReal dt)
	{
		const PxU32 nbActiveFEMCloths = mSimController->getBodySimManager().mActiveFEMCloths.size();

		if (mIsTGS || iter == maxIter - 1)
		{
			applyDamping(nbActiveFEMCloths, dt, mStream);
		}
	}

	void PxgFEMClothCore::step(PxReal dt, CUstream stream, PxU32 nbActiveFEMCloths, const PxVec3& gravity, bool forceUpdateClothContactPairs)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& femClothBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(femClothBuffer.getDevicePtr());

		PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();

		CUdeviceptr activeFEMClothsd = core->getActiveFEMClothBuffer().getDevicePtr();
		const PxU32 maxClothVerts = core->getMaxClothVerts();

		const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled() && mGpuContext->isTGS();

		{
			CUfunction stepKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_STEP);

			const PxU32 numBlocks =
				(maxClothVerts + PxgFEMClothKernelBlockDim::CLOTH_STEP - 1) / PxgFEMClothKernelBlockDim::CLOTH_STEP;

			{
				PxCudaKernelParam kernelParams[] = { 
					PX_CUDA_KERNEL_PARAM(femClothsd),
					PX_CUDA_KERNEL_PARAM(activeFEMClothsd), 
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(gravity),
					PX_CUDA_KERNEL_PARAM(externalForcesEveryTgsIterationEnabled),
					PX_CUDA_KERNEL_PARAM(forceUpdateClothContactPairs),
					PX_CUDA_KERNEL_PARAM(updateClothContactPairsd)
				};

				CUresult result = mCudaContext->launchKernel(stepKernelFunction, numBlocks, nbActiveFEMCloths, 1,
															 PxgFEMClothKernelBlockDim::CLOTH_STEP, 1, 1, 0, stream,
															 kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU step kernel fail!\n");
	#endif
			}
		}
	}

	void PxgFEMClothCore::solveNonSharedTriangles(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd,
												  PxU32 nbActiveFEMCloths, PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
		CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

		const PxU32 maxNonSharedPartitions = core->getMaxNonSharedTrianglePartitions();
		const PxU32 maxNonSharedTrianglesPerPartition = core->getMaxNonSharedTrianglesPerPartition();
		const PxU32 numThreadsPerBlock = PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL;
		const PxU32 numBlocks = (maxNonSharedTrianglesPerPartition + numThreadsPerBlock - 1) / numThreadsPerBlock;

		const PxU32 firstClusterId = core->getMaxNonSharedTriangleClusterId();

		// Solving for partitions with many elements (>= PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
		for(PxU32 i = 0; i < firstClusterId; ++i)
		{
			CUfunction solveTriEnergyFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_NONSHARED_TRIANGLE_ENERGY_SOLVE);

			PxCudaKernelParam solvetriEnergyKernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothsd), PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
															   PX_CUDA_KERNEL_PARAM(materials),	 PX_CUDA_KERNEL_PARAM(dt),
															   PX_CUDA_KERNEL_PARAM(i),			 PX_CUDA_KERNEL_PARAM(mIsTGS) };

			CUresult partitionResult = mCudaContext->launchKernel(solveTriEnergyFunction, numBlocks, nbActiveFEMCloths, 1,
																  PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL, 1, 1, 0, mStream,
																  solvetriEnergyKernelParams, sizeof(solvetriEnergyKernelParams), 0, PX_FL);
			PX_ASSERT(partitionResult == CUDA_SUCCESS);
			PX_UNUSED(partitionResult);
		}

		// Solving for partitions with small elements (< PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
		if(firstClusterId < maxNonSharedPartitions)
		{
			CUfunction solveTriEnergyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
				PxgKernelIds::CLOTH_SIM_NONSHARED_TRIANGLE_ENERGY_SOLVE_CLUSTER);

			PxCudaKernelParam solvetriEnergyKernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothsd),	 PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
															   PX_CUDA_KERNEL_PARAM(materials),		 PX_CUDA_KERNEL_PARAM(dt),
															   PX_CUDA_KERNEL_PARAM(firstClusterId), PX_CUDA_KERNEL_PARAM(mIsTGS) };

			CUresult partitionResult = mCudaContext->launchKernel(solveTriEnergyFunction, numBlocks, nbActiveFEMCloths, 1,
																  PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL, 1, 1, 0, mStream,
																  solvetriEnergyKernelParams, sizeof(solvetriEnergyKernelParams), 0, PX_FL);
			PX_ASSERT(partitionResult == CUDA_SUCCESS);
			PX_UNUSED(partitionResult);
		}

#if CLOTH_GPU_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_solveNonSharedTriangleEnergyLaunch kernel fail!\n");
#endif
	}

	void PxgFEMClothCore::solveTrianglePairs(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths,
		PxReal dt, bool isSharedTrianglePair)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
		CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

		const PxU32 maxTrianglePairsPerPartition =
			isSharedTrianglePair ? core->getMaxSharedTrianglePairsPerPartition() : core->getMaxNonSharedTrianglePairsPerPartition();
		const PxU32 maxPartitions =
			isSharedTrianglePair ? core->getMaxSharedTrianglePairPartitions() : core->getMaxNonSharedTrianglePairPartitions();
		const PxU32 firstClusterId =
			isSharedTrianglePair ? core->getMaxSharedTrianglePairClusterId() : core->getMaxNonSharedTrianglePairClusterId();

		// Solving for constraints
		{
			const PxU32 numThreadsPerBlock = PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL;
			const PxU32 numBlocks = (maxTrianglePairsPerPartition + numThreadsPerBlock - 1) / numThreadsPerBlock;

			// Solving for partitions with many elements (>= PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
			for (PxU32 i = 0; i < firstClusterId; ++i)
			{
				CUfunction triPairEnergyFunction =
					mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_TRIANGLEPAIR_ENERGY_SOLVE);

				PxCudaKernelParam solveTriPairEnergyKernelParams[] = {
					PX_CUDA_KERNEL_PARAM(femClothsd), PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
					PX_CUDA_KERNEL_PARAM(materials),  PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(i),		  PX_CUDA_KERNEL_PARAM(isSharedTrianglePair),
					PX_CUDA_KERNEL_PARAM(mIsTGS)
				};

				CUresult partitionResult = mCudaContext->launchKernel(
					triPairEnergyFunction, numBlocks, nbActiveFEMCloths, 1, PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL, 1, 1, 0, mStream,
					solveTriPairEnergyKernelParams, sizeof(solveTriPairEnergyKernelParams), 0, PX_FL);
				PX_ASSERT(partitionResult == CUDA_SUCCESS);
				PX_UNUSED(partitionResult);
			}

			// Solving for partitions with small elements (< PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
			if (firstClusterId < maxPartitions)
			{
				CUfunction triPairEnergyFunction =
					mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_TRIANGLEPAIR_ENERGY_SOLVE_CLUSTER);

				PxCudaKernelParam solveTriPairEnergyKernelParams[] = {
					PX_CUDA_KERNEL_PARAM(femClothsd), PX_CUDA_KERNEL_PARAM(activeFEMClothsd), PX_CUDA_KERNEL_PARAM(materials),
					PX_CUDA_KERNEL_PARAM(dt),		  PX_CUDA_KERNEL_PARAM(firstClusterId),	  PX_CUDA_KERNEL_PARAM(isSharedTrianglePair),
					PX_CUDA_KERNEL_PARAM(mIsTGS)
				};

				CUresult partitionResult = mCudaContext->launchKernel(
					triPairEnergyFunction, numBlocks, nbActiveFEMCloths, 1, PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL, 1, 1, 0, mStream,
					solveTriPairEnergyKernelParams, sizeof(solveTriPairEnergyKernelParams), 0, PX_FL);
				PX_ASSERT(partitionResult == CUDA_SUCCESS);
				PX_UNUSED(partitionResult);
			}

#if CLOTH_GPU_DEBUG
			CUresult result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_solveTrianglePairEnergyLaunch kernel fail!\n");
#endif
		}

		// Combining results
		{
			const PxU32 maxClothVerts = core->getMaxClothVerts();
			const PxU32 numThreadsPerBlock = PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL;
			const PxU32 numBlocks = (maxClothVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;
			const PxReal dtInv = 1.f / dt;

			CUfunction averageVertsFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_TRIANGLEPAIR_AVERAGE_VERTS);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothsd), PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
												 PX_CUDA_KERNEL_PARAM(dtInv), PX_CUDA_KERNEL_PARAM(isSharedTrianglePair) };

			CUresult result =
				mCudaContext->launchKernel(averageVertsFunction, numBlocks, nbActiveFEMCloths, 1, PxgFEMClothKernelBlockDim::CLOTH_STEP, 1, 1,
					0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_averageTrianglePairVertsLaunch kernel fail!\n");
#endif
		}
	}

	void PxgFEMClothCore::solveShellEnergy(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths,
										   PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		const PxU32 maxNonSharedTriangles = core->getMaxNbNonSharedTriangles();		
		const PxU32 maxSharedTrianglePairs = core->getMaxNbSharedTrianglePairs();
		const PxU32 maxNonSharedTrianglePairs = core->getMaxNbNonSharedTrianglePairs();

		// Solving for non-shared triangle constraints
		if(maxNonSharedTriangles > 0)
		{
			solveNonSharedTriangles(femClothsd, activeFEMClothsd, nbActiveFEMCloths, dt);
		} 

		// Solving for shared triangle-pair constraints
		if(maxSharedTrianglePairs > 0)
		{
			const bool isSharedTrianglePairs = true;
			solveTrianglePairs(femClothsd, activeFEMClothsd, nbActiveFEMCloths, dt, isSharedTrianglePairs);
		}

		const bool hasActiveBendingPairs = core->hasActiveBendingPairs();

		// Solving for non-shared triangle-pair constraints
		if(maxNonSharedTrianglePairs > 0 && hasActiveBendingPairs)
		{
			const bool isSharedTrianglePairs = false;
			solveTrianglePairs(femClothsd, activeFEMClothsd, nbActiveFEMCloths, dt, isSharedTrianglePairs);
		}
	}

	void PxgFEMClothCore::queryRigidContactReferenceCount(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
														  PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
														  PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
														  PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt)
	{
		PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidReferenceCount.getDevicePtr();
		mCudaContext->memsetD32Async(femRigidContactCount.mPtr, 0, mFemRigidReferenceCount.getNbElements(), mStream);

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgDevicePointer<PxgFEMCloth> femClothesd = core->getFEMClothBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		const CUfunction kernelFunction =
			mIsTGS
				? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_QUERY_RIGID_CLOTH_REFERENCE_COUNT_TGS)
				: mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_QUERY_RIGID_CLOTH_REFERENCE_COUNT);

		PxgDevicePointer<PxgFemContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
		PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();

		PxgDevicePointer<float4> lambdaNs = mRigidLambdaNBuf.getTypedDevicePtr();

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd),
											 PX_CUDA_KERNEL_PARAM(contactInfosd),
											 PX_CUDA_KERNEL_PARAM(constraintsd),
											 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
											 PX_CUDA_KERNEL_PARAM(prePrepDescd),
											 PX_CUDA_KERNEL_PARAM(solverCoreDescd),
											 PX_CUDA_KERNEL_PARAM(artiCoreDescd),
											 PX_CUDA_KERNEL_PARAM(sharedDescd),
											 PX_CUDA_KERNEL_PARAM(dt),
											 PX_CUDA_KERNEL_PARAM(lambdaNs),
											 PX_CUDA_KERNEL_PARAM(femRigidContactCount)
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
													 PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams,
													 sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_queryRigidClothContactReferenceCountLaunch kernel fail!\n");
#endif
	}

	void PxgFEMClothCore::solveRigidContactsOutputClothDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
															 PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
															 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
															 PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgDevicePointer<PxgFEMCloth> femClothesd = core->getFEMClothBuffer().getTypedDevicePtr();

		// solve rigid body and soft body contacts
		{
			PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

			const CUfunction solveOutputClothDeltaKernelFunction =
				mIsTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_RIGID_CLOTH_DELTA_TGS)
					   : mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_RIGID_CLOTH_DELTA);

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			PxsMaterialData* rigidBodyMaterials = reinterpret_cast<PxsMaterialData*>(npCore->mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<float4> lambdaNs = mRigidLambdaNBuf.getTypedDevicePtr();

			PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidReferenceCount.getDevicePtr();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd),
												 PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(constraintsd),
												 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
												 PX_CUDA_KERNEL_PARAM(prePrepDescd),
												 PX_CUDA_KERNEL_PARAM(solverCoreDescd),
												 PX_CUDA_KERNEL_PARAM(artiCoreDescd),
												 PX_CUDA_KERNEL_PARAM(sharedDescd),
												 PX_CUDA_KERNEL_PARAM(dt),
												 PX_CUDA_KERNEL_PARAM(deltaVd),
												 PX_CUDA_KERNEL_PARAM(lambdaNs),
												 PX_CUDA_KERNEL_PARAM(femRigidContactCount),
												 PX_CUDA_KERNEL_PARAM(materials),
												 PX_CUDA_KERNEL_PARAM(rigidBodyMaterials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputClothDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION,
														 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams,
														 sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			mCudaContext->eventRecord(mSolveClothEvent, mStream);

#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_solveOutputClothDeltaVLaunch kernel fail!\n");
#endif
		}
	}

	// solve cloth vs rigid body contact and output to rigid delta buffer
	void PxgFEMClothCore::solveRigidContactsOutputRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
															 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
															 CUstream solverStream, PxReal dt)
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		{
			const CUfunction solveOutputRigidDeltaKernelFunction =
				mIsTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_RIGID_RIGID_DELTAV_TGS)
					   : mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_RIGID_RIGID_DELTAV);

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgDevicePointer<PxgFEMCloth> femClothesd = core->getFEMClothBuffer().getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			PxsMaterialData* rigidBodyMaterials = reinterpret_cast<PxsMaterialData*>(npCore->mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PxgDevicePointer<float4> lambdaNs = mFemLambdaNBuf.getTypedDevicePtr();
			
			PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidReferenceCount.getDevicePtr();

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd),
												 PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(constraintsd),
												 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
												 PX_CUDA_KERNEL_PARAM(prePrepDescd),
												 PX_CUDA_KERNEL_PARAM(solverCoreDescd),
												 PX_CUDA_KERNEL_PARAM(artiCoreDescd),
												 PX_CUDA_KERNEL_PARAM(sharedDescd),
												 PX_CUDA_KERNEL_PARAM(deltaVd),
												 PX_CUDA_KERNEL_PARAM(lambdaNs),
												 PX_CUDA_KERNEL_PARAM(femRigidContactCount),
												 PX_CUDA_KERNEL_PARAM(dt),
												 PX_CUDA_KERNEL_PARAM(materials),
												 PX_CUDA_KERNEL_PARAM(rigidBodyMaterials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputRigidDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION,
														 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, solverStream,
														 kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			mCudaContext->eventRecord(mSolveRigidEvent, solverStream);

#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_solveOutputRigidDeltaVLaunch kernel fail!\n");

			int bob = 0;
			PX_UNUSED(bob);
#endif
		}

		// accumulate velocity delta for rigid body and impulse delta for articulation link
		accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, mRigidSortedRigidIdBuf.getDevicePtr(),
							  mRigidTotalContactCountBuf.getDevicePtr(), solverStream, mSolveClothEvent, mIsTGS);

		// if the contact is between articulation and soft body, after accumulated all the related contact's
		// impulse, we need to propagate the accumulated impulse to the articulation block solver
		mGpuContext->mGpuArticulationCore->pushImpulse(solverStream);
	}
	


	void PxgFEMClothCore::solveRigidAttachmentRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
														 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
														 CUstream solverStream, PxReal dt)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbActiveRigidClothAttachments();

		if(nbRigidAttachments)
		{
			PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(simCore->getFEMClothBuffer().getDevicePtr());

			PxgDevicePointer<PxgFEMRigidAttachmentConstraint> constraintsd = simCore->getClothRigidConstraints();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();

			{
				const CUfunction solvePCRigidKernelFunction =
					mIsTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
								 PxgKernelIds::CLOTH_SOLVE_ATTACHMENT_RIGID_DELTA_TGS)
						   : mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
								 PxgKernelIds::CLOTH_SOLVE_ATTACHMENT_RIGID_DELTA);

				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd),
													 PX_CUDA_KERNEL_PARAM(constraintsd),
													 PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
													 PX_CUDA_KERNEL_PARAM(prePrepDescd),
													 PX_CUDA_KERNEL_PARAM(solverCoreDescd),
													 PX_CUDA_KERNEL_PARAM(artiCoreDescd),
													 PX_CUDA_KERNEL_PARAM(sharedDescd),
													 PX_CUDA_KERNEL_PARAM(dt),
													 PX_CUDA_KERNEL_PARAM(deltaVd) };

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock,
															 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU cloth_solveOutputAttachmentRigidDeltaVLaunch kernel fail!\n");

	#endif
			}

			mCudaContext->eventRecord(mSolveRigidEvent, solverStream);

			PxgDevicePointer<PxNodeIndex> rigidAttachmentIds = simCore->getClothRigidAttachmentIds();
			PxgDevicePointer<PxU32> totalRigidAttachmentsd = simCore->getGpuClothRigidCounter();

			accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, rigidAttachmentIds,
								  totalRigidAttachmentsd, solverStream, mSolveClothEvent, mIsTGS);
		}
	}

	void PxgFEMClothCore::solveRigidAttachmentClothDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
														 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbActiveRigidClothAttachments();

		if(nbRigidAttachments)
		{
			PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(simCore->getFEMClothBuffer().getDevicePtr());

			PxgDevicePointer<PxgFEMRigidAttachmentConstraint> constraintsd = simCore->getClothRigidConstraints();
			{
				const CUfunction solvePCRigidKernelFunction =
					mIsTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
								 PxgKernelIds::CLOTH_SOLVE_ATTACHMENT_CLOTH_DELTA_TGS)
						   : mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
								 PxgKernelIds::CLOTH_SOLVE_ATTACHMENT_CLOTH_DELTA);

				PxCudaKernelParam kernelParams[] = {
					PX_CUDA_KERNEL_PARAM(clothesd),           PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments), PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),    PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),        PX_CUDA_KERNEL_PARAM(dt)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock,
															 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				mCudaContext->eventRecord(mSolveClothEvent, mStream);
	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU cloth_solveOutputAttachmentClothDeltaVLaunch kernel fail!\n");
	#endif
			}
		}
	}

	void PxgFEMClothCore::solveClothAttachmentDelta()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbClothAttachments = simCore->getNbActiveClothClothAttachments();

		if(nbClothAttachments)
		{
			PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(simCore->getFEMClothBuffer().getDevicePtr());

			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getClothClothConstraints();

			{
				const CUfunction solveClothAttachmentKernelFunction =
					mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
						PxgKernelIds::CLOTH_SOLVE_ATTACHMENT_CLOTH_CLOTH_DELTA);

				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd), PX_CUDA_KERNEL_PARAM(constraintsd),
													 PX_CUDA_KERNEL_PARAM(nbClothAttachments) };

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result =
					mCudaContext->launchKernel(solveClothAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1,
											   1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sb_solveOutputAttachmentSoftDeltaVLaunchTGS kernel fail!\n");
	#endif
			}
		}
	}

	void PxgFEMClothCore::rewindCloth(PxU32 nbActiveFEMCloths)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& femClothBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(femClothBuffer.getDevicePtr());

		CUdeviceptr activeFEMClothsd = core->getActiveFEMClothBuffer().getDevicePtr();
		const PxU32 maxClothVerts = core->getMaxClothVerts();

		{
			CUfunction rewindKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_REWIND);

			const PxU32 numBlocks = (maxClothVerts + PxgFEMClothKernelBlockDim::CLOTH_STEP - 1) / PxgFEMClothKernelBlockDim::CLOTH_STEP;

			{
				PxCudaKernelParam kernelParams[] = {
					PX_CUDA_KERNEL_PARAM(femClothsd),
					PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
				};

				CUresult result =
					mCudaContext->launchKernel(rewindKernelFunction, numBlocks, nbActiveFEMCloths, 1, PxgFEMClothKernelBlockDim::CLOTH_STEP,
											   1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_rewindLaunch kernel fail!\n");
#endif
			}
		}
	}

	void PxgFEMClothCore::advanceSubstep(PxU32 nbActiveFEMCloths, PxU32 nbCollisionSubsteps, PxReal dt)
	{
		PX_ASSERT(nbCollisionSubsteps);
		const PxReal nbCollisionSubstepsInv = 1.0f / static_cast<PxReal>(nbCollisionSubsteps);
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& femClothBuffer = core->getFEMClothBuffer();
		PxgFEMCloth* femClothsd = reinterpret_cast<PxgFEMCloth*>(femClothBuffer.getDevicePtr());

		CUdeviceptr activeFEMClothsd = core->getActiveFEMClothBuffer().getDevicePtr();
		const PxU32 maxClothVerts = core->getMaxClothVerts();

		{
			CUfunction advanceKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SIM_SUBSTEP);

			const PxU32 numBlocks = (maxClothVerts + PxgFEMClothKernelBlockDim::CLOTH_STEP - 1) / PxgFEMClothKernelBlockDim::CLOTH_STEP;

			{
				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothsd), PX_CUDA_KERNEL_PARAM(activeFEMClothsd),
													 PX_CUDA_KERNEL_PARAM(nbCollisionSubstepsInv), PX_CUDA_KERNEL_PARAM(dt) };

				CUresult result = mCudaContext->launchKernel(advanceKernelFunction, numBlocks, nbActiveFEMCloths, 1,
															 PxgFEMClothKernelBlockDim::CLOTH_STEP, 1, 1, 0, mStream, kernelParams,
															 sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_subStepLaunch kernel fail!\n");
#endif
			}
		}
	}

	// This function is called when either forceUpdateClothContactPairs or adaptiveCollisionPairUpdate is true.
	void PxgFEMClothCore::prepareClothClothCollision(bool forceUpdateClothContactPairs, bool adaptiveCollisionPairUpdate)
	{
		const PxU32 nbActiveFEMCloths = mSimController->getNbActiveFEMCloths();
		refitBound(nbActiveFEMCloths, mStream);

		if(adaptiveCollisionPairUpdate) // Update data needed for adaptive/automatic contact pair generation.
										// Reset contact count before updating contact pairs.
		{
			updateClothContactPairValidity();
		}
		else if(forceUpdateClothContactPairs) // Reset contact count before updating contact pairs.
		{
			PxgDevicePointer<PxU32> totalFemContactCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();
			mCudaContext->memsetD32Async(totalFemContactCountsd.mPtr, 0, 1, mStream);
		}

		// Perform midphase contact pair generation.
		selfCollision();
		differentClothCollision();

		// Store additional contact data.
		prepClothContactConstraint();
	}

	void PxgFEMClothCore::solveClothClothCollision(PxU32 nbActiveFEMCloths, PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		const PxU32 nbCollisionSubsteps = core->getMaxNbCollisionSubsteps();

		if(nbCollisionSubsteps > 1)
		{
#if 1 // Multiple iterations for cloth-cloth collision
			for (PxU32 subIt = 0; subIt < nbCollisionSubsteps; ++subIt)
			{
				solveClothContactsOutputClothDelta(dt);
				applyExternalDelta(nbActiveFEMCloths, dt, mStream);
			}

#else // Multiple sub-steps for cloth-cloth collision.
			rewindCloth(nbActiveFEMCloths);
			for(PxU32 subIt = 0; subIt < nbCollisionSubsteps; ++subIt)
			{
				advanceSubstep(nbActiveFEMCloths, nbCollisionSubsteps, dt);

				// dt is used instead of subDt, to avoid additional floating precission issues, excessive velocities, etc.
				solveClothContactsOutputClothDelta(dt);
				applyExternalDelta(nbActiveFEMCloths, dt, mStream);
			}
#endif
		}
		else
		{
			solveClothContactsOutputClothDelta(dt);
			applyExternalDelta(nbActiveFEMCloths, dt, mStream);
		}
	}

	void PxgFEMClothCore::solveClothContactsOutputClothDelta(PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());
		PxgDevicePointer<PxU32> totalFemContactCountsd = mFemTotalContactCountBuffer.getTypedDevicePtr();

		// Solve self collision contacts or cloth vs cloth contacts
		{
			const CUfunction solveOutputClothDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
				PxgKernelIds::CLOTH_CLOTH_TRIANGLE_CLOTH_VERTEX_DELTA);
			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mFemContactInfoBuffer.getTypedDevicePtr();
			CUdeviceptr constraintsd = mFemConstraintBuf.getDevicePtr();
			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd), PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(constraintsd), PX_CUDA_KERNEL_PARAM(totalFemContactCountsd),
												 PX_CUDA_KERNEL_PARAM(dt) };

			CUresult result = mCudaContext->launchKernel(
				solveOutputClothDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
				PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if 0
				mCudaContext->streamSynchronize(mStream);
				PxU32 numFEMClothContactCounts;
				mCudaContext->memcpyDtoH(&numFEMClothContactCounts, totalFemContactCountsd, sizeof(PxU32));

				if (numFEMClothContactCounts >= mMaxContacts)
				{
					printf("contact counts: %i / %i : %f\n", numFEMClothContactCounts, mMaxContacts, PxReal(numFEMClothContactCounts) / PxReal(mMaxContacts));
					exit(0);
				}
	#endif

	#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_solveClothTriClothVertDeltaVLaunch kernel fail!\n");
	#endif
		}
	}

	void PxgFEMClothCore::applyDamping(PxU32 nbActiveFemClothes, PxReal dt, CUstream stream)
	{
		//Velocity adjustment after position adjustment as described in algorithm 2 in "Detailed Rigid Body Simulation with Extended Position Based Dynamics"

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgFEMCloth* femClothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());
		PxU32* activeFemClothesd = reinterpret_cast<PxU32*>(core->getActiveFEMClothBuffer().getDevicePtr());

		//Accumulate in-plane damping
		{
			const PxU32 maxNbTriangles = core->getMaxNbSharedTrianglePairs() * 2 + core->getMaxNbNonSharedTriangles();

			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
			const PxU32 numBlocks = (maxNbTriangles + numThreadsPerBlock - 1) / numThreadsPerBlock;

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			//PxReal inPlaneDamping = 1.0f /dt;// 0.1f;

			const CUfunction kernel =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_IN_PLANE_DAMPING);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd), PX_CUDA_KERNEL_PARAM(activeFemClothesd),
												 PX_CUDA_KERNEL_PARAM(materials), PX_CUDA_KERNEL_PARAM(dt) };

			CUresult result =
				mCudaContext->launchKernel(kernel, numBlocks, nbActiveFemClothes, 1, numThreadsPerBlock,
					1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
		}

		//Accumulate bending damping
		{
			PxU32 maxNbTrianglePairs = core->getMaxNbSharedTrianglePairs() + core->getMaxNbNonSharedTrianglePairs(); // nbMaxSharedTrianglePairs + nbMaxNonSharedTrianglePairs;

			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
			const PxU32 numBlocks = (maxNbTrianglePairs + numThreadsPerBlock - 1) / numThreadsPerBlock;

			//PxReal bendingDamping = 1.0f / dt;// 1.0f / dt;//0.1f;

			const CUfunction kernel =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_BENDING_DAMPING);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd), PX_CUDA_KERNEL_PARAM(activeFemClothesd),
												 /*PX_CUDA_KERNEL_PARAM(bendingDamping),*/ PX_CUDA_KERNEL_PARAM(dt) };

			CUresult result =
				mCudaContext->launchKernel(kernel, numBlocks, nbActiveFemClothes, 1, numThreadsPerBlock,
					1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
		}

		//Apply the accumulated velocity change due to damping
		{
			const PxU32 maxVerts = core->getMaxClothVerts();

			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
			const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

			const CUfunction kernel =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_APPLY_ACCUMULATED_DAMPING);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd), PX_CUDA_KERNEL_PARAM(activeFemClothesd) };

			CUresult result =
				mCudaContext->launchKernel(kernel, numBlocks, nbActiveFemClothes, 1, numThreadsPerBlock,
					1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
		}
	}

	void PxgFEMClothCore::applyExternalDelta(PxU32 nbActiveFemClothes, PxReal dt, CUstream stream)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgFEMCloth* femClothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());
		PxU32* activeFemClothesd = reinterpret_cast<PxU32*>(core->getActiveFEMClothBuffer().getDevicePtr());

		const PxU32 maxVerts = core->getMaxClothVerts();

		const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
		const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

		{
			const CUfunction applyDeltaKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_APPLY_EXTERNAL_DELTAS);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd), PX_CUDA_KERNEL_PARAM(activeFemClothesd),
												 PX_CUDA_KERNEL_PARAM(dt) };

			CUresult result =
				mCudaContext->launchKernel(applyDeltaKernelFunction, numBlocks, nbActiveFemClothes, 1, numThreadsPerBlock,
										   1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_applyExternalDeltasLaunch kernel fail!\n");
	#endif
		}
	}

	void PxgFEMClothCore::applyExternalDeltaAndCheckClothCollisionValidity(PxU32 nbActiveFemClothes, PxReal dt,
																		   bool adaptiveCollisionPairUpdate)
	{
		PxgDevicePointer<PxU8> updateClothContactPairsd = mUpdateClothContactPairs.getTypedDevicePtr();

		if (adaptiveCollisionPairUpdate)
		{
			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgFEMCloth* femClothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());
			PxU32* activeFemClothesd = reinterpret_cast<PxU32*>(core->getActiveFEMClothBuffer().getDevicePtr());

			const PxU32 maxVerts = core->getMaxClothVerts();

			const PxU32 numThreadsPerBlock = PxgFEMClothKernelBlockDim::CLOTH_STEP;
			const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

			const CUfunction applyDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
				PxgKernelIds::CLOTH_APPLY_EXTERNAL_DELTAS_AND_CHECK_CLOTH_COLLISION_VALIDITY);

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd), PX_CUDA_KERNEL_PARAM(activeFemClothesd),
												 PX_CUDA_KERNEL_PARAM(dt), PX_CUDA_KERNEL_PARAM(updateClothContactPairsd) };

			CUresult result = mCudaContext->launchKernel(applyDeltaKernelFunction, numBlocks, nbActiveFemClothes, 1, numThreadsPerBlock, 1,
														 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU cloth_applyExternalDeltasAndCheckClothCollisionValidityLaunch kernel fail!\n");
#endif

		}
		else
		{
			applyExternalDelta(nbActiveFemClothes, dt, mStream);
		}
	}

	void PxgFEMClothCore::applyExternalDeltaWithVelocityClamping(PxU32 nbActiveFemClothes, PxReal dt, CUstream stream)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgFEMCloth* femClothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());
		PxU32* activeFemClothesd = reinterpret_cast<PxU32*>(core->getActiveFEMClothBuffer().getDevicePtr());

		const PxU32 maxVerts = core->getMaxClothVerts();
		const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
		const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

		const CUfunction applyDeltaKernelFunction =
			mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_APPLY_EXTERNAL_DELTAS_WITH_VELOCITY_CLAMPING);

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(femClothesd), PX_CUDA_KERNEL_PARAM(activeFemClothesd),
											 PX_CUDA_KERNEL_PARAM(dt) };

		CUresult result = mCudaContext->launchKernel(applyDeltaKernelFunction, numBlocks, nbActiveFemClothes, 1, numThreadsPerBlock, 1, 1,
													 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
									"GPU cloth_applyExternalDeltasWithVelocityClampingLaunch kernel fail!\n");
#endif
	}

	// solve cloth vs. particle contact and output to cloth delta buffer
	void PxgFEMClothCore::solveParticleContactsOutputClothDelta(CUstream particleStream)
	{
		PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());

		PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalParticleContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

		synchronizeStreams(mCudaContext, mStream, particleStream);

		// solve cloth vs. particle contact in the cloth stream and update delta and applied force for soft body
		{
			const CUfunction solveOutputClothDeltaKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_PARTICLE_CLOTH_DELTA);

			PxgDevicePointer<PxgFemContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFEMParticleConstraintBlock> constraintsd = mParticleConstraintBuf.getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxgDevicePointer<float4> appliedForced = mParticleAppliedFEMForcesBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd),
												 PX_CUDA_KERNEL_PARAM(particlesystemsd),
												 PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(constraintsd),
												 PX_CUDA_KERNEL_PARAM(totalParticleContactCountsd),
												 PX_CUDA_KERNEL_PARAM(appliedForced),
												 PX_CUDA_KERNEL_PARAM(materials) };

			CUresult result = mCudaContext->launchKernel(
				solveOutputClothDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
				PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

	#if CLOTH_GPU_DEBUG
			PxU32 numSoftCount;
			mCudaContext->memcpyDtoH(&numSoftCount, totalParticleContactCountsd, sizeof(PxU32));

			if(numSoftCount > 0)
			{
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sb_solveOutputSSDeltaVLaunch kernel fail!\n");
			}
	#endif
		}
	}

	// solve cloth vs particle contact and output to particle delta buffer
	void PxgFEMClothCore::solveParticleContactsOutputParticleDelta(CUstream particleStream)
	{
		// solve soft body vs particle contact in the particle system stream and update selfCollision delta for particle
		// system
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());

		PxgParticleSystemCore* particleSystemCore = mSimController->getPBDParticleSystemCore();

		PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleSystemCore->getParticleSystemBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxgFemContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();

		PxgDevicePointer<float4> deltaVd = particleSystemCore->getDeltaVelParticle();

		PxgDevicePointer<PxU32> totalParticleContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

	#if CLOTH_GPU_DEBUG
		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalParticleContactCountsd, sizeof(PxU32));
	#endif

		{
			// particley stream need to wait till cloth vs particle constraint prep finish in cloth stream
			mCudaContext->streamWaitEvent(particleStream, mConstraintPrepParticleEvent);

			const CUfunction solveOutputParticleDeltaKernelFunction =
				mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_PARTICLE_PARTICLE_DELTA);

			PxgDevicePointer<PxgFEMParticleConstraintBlock> constraintsd = mParticleConstraintBuf.getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxgDevicePointer<float4> appliedForced = mParticleAppliedParticleForcesBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(clothesd),
												 PX_CUDA_KERNEL_PARAM(particlesystemsd),
												 PX_CUDA_KERNEL_PARAM(contactInfosd),
												 PX_CUDA_KERNEL_PARAM(constraintsd),
												 PX_CUDA_KERNEL_PARAM(totalParticleContactCountsd),
												 PX_CUDA_KERNEL_PARAM(deltaVd),
												 PX_CUDA_KERNEL_PARAM(appliedForced),
												 PX_CUDA_KERNEL_PARAM(materials) };

			CUresult result = mCudaContext->launchKernel(solveOutputParticleDeltaKernelFunction,
														 PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
														 PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0,
														 particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			mCudaContext->eventRecord(mSolveParticleEvent, particleStream);

	#if CLOTH_GPU_DEBUG

			if(numContacts > 0)
			{
				result = mCudaContext->streamSynchronize(particleStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if(result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
											"GPU sb_solveOutputSSDeltaVLaunch kernel fail!\n");

				PxArray<PxVec4> deltaV;
				deltaV.reserve(numContacts);
				deltaV.forceSize_Unsafe(numContacts);

				mCudaContext->memcpyDtoH(deltaV.begin(), deltaVd, sizeof(float4) * numContacts);

				int bob;
				PX_UNUSED(bob);
			}
	#endif
		}

		{
			// those temp buffer store the start and end index for the particle vs soft body range sorted by particle id
			PxgDevicePointer<PxU32> pairCountd = mTempHistogramCountBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> startd = mTempContactBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> endd = mTempContactRemapBuf.getTypedDevicePtr();

			// accumulate deltaV changes for particle
			{
				const CUfunction accumulatedDeltaVKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					PxgKernelIds::PS_ACCUMULATE_FEM_PARTICLE_DELTA);

				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(particlesystemsd),
													 PX_CUDA_KERNEL_PARAM(contactInfosd),
													 PX_CUDA_KERNEL_PARAM(pairCountd),
													 PX_CUDA_KERNEL_PARAM(startd),
													 PX_CUDA_KERNEL_PARAM(endd),
													 PX_CUDA_KERNEL_PARAM(deltaVd) };

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA;
				CUresult result =
					mCudaContext->launchKernel(accumulatedDeltaVKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1,
											   0, particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}

			// synchronizeStreams(mCudaContext, particleStream, mStream);
		}
	}

	bool PxgFEMClothCore::updateUserData(PxPinnedArray<PxgFEMCloth>& femClothPool, PxArray<PxU32>& femClothNodeIndexPool,
										 const PxU32* activeFEMCloths, PxU32 nbActiveFEMCloths, void** bodySimsLL)
	{
		bool anyDirty = false;
		for(PxU32 i = 0; i < nbActiveFEMCloths; ++i)
		{
			PxgFEMCloth& femCloth = femClothPool[activeFEMCloths[i]];
			PxU32 nodeIndex = femClothNodeIndexPool[femCloth.mGpuRemapIndex];
			Dy::DeformableSurface* dyDeformableSurface = reinterpret_cast<Dy::DeformableSurface*>(bodySimsLL[nodeIndex]);
			Dy::DeformableSurfaceCore& dyDeformableSurfaceCore = dyDeformableSurface->getCore();

			if(dyDeformableSurfaceCore.dirty)
			{
				femCloth.mLinearDamping = dyDeformableSurfaceCore.linearDamping;
				femCloth.mMaxLinearVelocity = dyDeformableSurfaceCore.maxLinearVelocity;
				femCloth.mPenBiasClamp = dyDeformableSurfaceCore.maxPenetrationBias;

				femCloth.mSettlingThreshold = dyDeformableSurfaceCore.settlingThreshold;
				femCloth.mSleepThreshold = dyDeformableSurfaceCore.sleepThreshold;
				femCloth.mSettlingDamping = dyDeformableSurfaceCore.settlingDamping;
				femCloth.mSelfCollisionFilterDistance = dyDeformableSurfaceCore.selfCollisionFilterDistance;

				femCloth.mActorFlags = dyDeformableSurfaceCore.actorFlags;
				femCloth.mBodyFlags = dyDeformableSurfaceCore.bodyFlags;
				femCloth.mSurfaceFlags = dyDeformableSurfaceCore.surfaceFlags;

				femCloth.mNbCollisionPairUpdatesPerTimestep = dyDeformableSurfaceCore.nbCollisionPairUpdatesPerTimestep;
				femCloth.mNbCollisionSubsteps = dyDeformableSurfaceCore.nbCollisionSubsteps;

				anyDirty = true;
				dyDeformableSurfaceCore.dirty = false;
			}

			// This would be a place to do actual things with the dirty flags.
			if (dyDeformableSurfaceCore.dirtyFlags)
				dyDeformableSurfaceCore.dirtyFlags = PxDeformableSurfaceDataFlags(0);
		}

		return anyDirty;
	}

	/*******************************************************************************
	 *
	 *
	 * Triangle partition
	 *
	 *
	 ******************************************************************************/

#define FEMCLOTH_TEMP_PARTITION_SIZE 32

	PxU32 computeTrianglePartition(const uint4& triangle, const PxU32 partitionStartIndex, PxU32* partitionProgresses)
	{
		PxU32 partitionA = partitionProgresses[triangle.x];
		PxU32 partitionB = partitionProgresses[triangle.y];
		PxU32 partitionC = partitionProgresses[triangle.z];

		const PxU32 combinedMask = (~partitionA & ~partitionB & ~partitionC);
		PxU32 availablePartition = combinedMask == 0 ? FEMCLOTH_TEMP_PARTITION_SIZE : PxLowestSetBit(combinedMask);

		if(availablePartition == FEMCLOTH_TEMP_PARTITION_SIZE)
		{
			return 0xFFFFFFFF;
		}

		const PxU32 partitionBit = (1u << availablePartition);
		partitionA |= partitionBit;
		partitionB |= partitionBit;
		partitionC |= partitionBit;

		availablePartition += partitionStartIndex;

		partitionProgresses[triangle.x] = partitionA;
		partitionProgresses[triangle.y] = partitionB;
		partitionProgresses[triangle.z] = partitionC;

		return availablePartition;
	}

	void classifyTriangles(const uint4* const triangles, const PxArray<PxU32>& activeTriangles, PxU32 nbVerts, PxU32* partitionProgresses,
						   PxU32* tempTriangles, PxArray<PxU32>& trianglesPerPartition)
	{
		// initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * nbVerts);
		PxU32 nbUnpartitionedTriangles = 0;

		for(PxU32 it = 0; it < activeTriangles.size(); ++it)
		{
			const PxU32 i = activeTriangles[it];
			const uint4& triangle = triangles[i];
			const PxU32 availablePartition = computeTrianglePartition(triangle, 0, partitionProgresses);

			if(availablePartition == 0xFFFFFFFF)
			{
				tempTriangles[nbUnpartitionedTriangles++] = it;
				continue;
			}

			trianglesPerPartition[availablePartition]++;
		}

		PxU32 partitionStartIndex = 0;
		while(nbUnpartitionedTriangles > 0)
		{
			// initialize the partition progress counter to be zero
			PxMemZero(partitionProgresses, sizeof(PxU32) * nbVerts);

			partitionStartIndex += FEMCLOTH_TEMP_PARTITION_SIZE;

			// keep partitioning the un-partitioned constraints and blat the whole thing to 0!
			trianglesPerPartition.resize(FEMCLOTH_TEMP_PARTITION_SIZE + trianglesPerPartition.size());
			PxMemZero(trianglesPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * FEMCLOTH_TEMP_PARTITION_SIZE);

			PxU32 newNbUnpartitionedConstraints = 0;

			for(PxU32 i = 0; i < nbUnpartitionedTriangles; ++i)
			{
				const PxU32 triangleInd = tempTriangles[i];
				const uint4& triangle = triangles[activeTriangles[triangleInd]];
				const PxU32 availablePartition = computeTrianglePartition(triangle, partitionStartIndex, partitionProgresses);

				if(availablePartition == 0xFFFFFFFF)
				{
					tempTriangles[newNbUnpartitionedConstraints++] = triangleInd;
					continue;
				}

				trianglesPerPartition[availablePartition]++;
			}

			nbUnpartitionedTriangles = newNbUnpartitionedConstraints;
		}
	}

	void writeTriangles(const uint4* const triangles, const PxArray<PxU32>& activeTriangles, const PxU32 numVerts,
						PxU32* partitionProgresses, PxU32* tempTriangles, PxArray<PxU32>& orderedTriangles, PxU32* accumulatedTrianglesPerPartition)
	{
		// initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

		PxU32 numUnpartitionedTriangles = 0;

		for(PxU32 it = 0; it < activeTriangles.size(); ++it)
		{
			const PxU32 i = activeTriangles[it];
			const uint4& triangle = triangles[i];

			const PxU32 availablePartition = computeTrianglePartition(triangle, 0, partitionProgresses);

			if(availablePartition == 0xFFFFFFFF)
			{
				tempTriangles[numUnpartitionedTriangles++] = it;
				continue;
			}

			// output triangle
			orderedTriangles[accumulatedTrianglesPerPartition[availablePartition]++] = it;
		}

		PxU32 partitionStartIndex = 0;

		while(numUnpartitionedTriangles > 0)
		{
			// initialize the partition progress counter to be zero
			PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

			partitionStartIndex += FEMCLOTH_TEMP_PARTITION_SIZE;

			PxU32 newNumUnpartitionedConstraints = 0;

			for(PxU32 i = 0; i < numUnpartitionedTriangles; ++i)
			{
				const PxU32 triangleInd = tempTriangles[i];
				const uint4& triangle = triangles[activeTriangles[triangleInd]];

				const PxU32 availablePartition = computeTrianglePartition(triangle, partitionStartIndex, partitionProgresses);

				if(availablePartition == 0xFFFFFFFF)
				{
					tempTriangles[newNumUnpartitionedConstraints++] = triangleInd;
					continue;
				}

				// output triangles
				orderedTriangles[accumulatedTrianglesPerPartition[availablePartition]++] = triangleInd;
			}

			numUnpartitionedTriangles = newNumUnpartitionedConstraints;
		}
	}

	void PxgFEMClothCore::partitionTriangleSimData(PxgFEMCloth& femCloth, PxgFEMClothData& clothData, PxArray<PxU32>& orderedTriangles,
												   const PxArray<PxU32>& activeTriangles, PxsHeapMemoryAllocator* alloc)
	{
		if (activeTriangles.empty())
		{
			clothData.mMaxNbNonSharedTrisPerPartition = 0;
			femCloth.mNbNonSharedTriPartitions = 0;
			femCloth.mNonSharedTriClusterId = 0;
			return;
		}

		const PxU32 nbTriangles = activeTriangles.size();
		const PxU32 nbVerts = femCloth.mNbVerts;

		orderedTriangles.reserve(activeTriangles.size());
		orderedTriangles.forceSize_Unsafe(activeTriangles.size());

		// each vert has a partition progress counter
		PxU32* partitionProgresses = PX_ALLOCATE(PxU32, nbVerts, "partitionProgress");

		// this store the triangle index for the unpartitioned triangles
		PxU32* tempTriangles = PX_ALLOCATE(PxU32, nbTriangles, "tempTriangles");

		PxArray<PxU32> trianglesPerPartition;
		trianglesPerPartition.reserve(FEMCLOTH_TEMP_PARTITION_SIZE);
		trianglesPerPartition.forceSize_Unsafe(FEMCLOTH_TEMP_PARTITION_SIZE);

		PxMemZero(trianglesPerPartition.begin(), sizeof(PxU32) * FEMCLOTH_TEMP_PARTITION_SIZE);

		classifyTriangles(femCloth.mTriangleVertexIndices, activeTriangles, nbVerts, partitionProgresses, tempTriangles,
			trianglesPerPartition);

		// compute number of partitions
		PxU32 maxPartition = 0;
		for (PxU32 a = 0; a < trianglesPerPartition.size(); ++a, ++maxPartition)
		{
			if (trianglesPerPartition[a] == 0)
				break;
		}

		femCloth.mNonSharedTriAccumulatedPartitionsCP =
			reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * maxPartition, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

		// compute run sum
		PxU32 accumulation = 0;
		PxU32 maxTrianglesPerPartition = 0;
		PxU32 clusterId = maxPartition;

		for (PxU32 a = 0; a < maxPartition; ++a)
		{
			PxU32 count = trianglesPerPartition[a];
			femCloth.mNonSharedTriAccumulatedPartitionsCP[a] = accumulation;
			accumulation += count;

			maxTrianglesPerPartition = PxMax(count, maxTrianglesPerPartition);

			if (clusterId == maxPartition && count < PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
			{
				clusterId = a;
			}
			if (count >= PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
			{
				clusterId = maxPartition;
			}
		}

		femCloth.mNonSharedTriClusterId = clusterId;

		PX_ASSERT(accumulation == nbTriangles);

		femCloth.mNbNonSharedTriPartitions = maxPartition;
		clothData.mMaxNbNonSharedTrisPerPartition = maxTrianglesPerPartition;

		writeTriangles(femCloth.mTriangleVertexIndices, activeTriangles, nbVerts, partitionProgresses, tempTriangles, orderedTriangles,
			femCloth.mNonSharedTriAccumulatedPartitionsCP);

		//printf("DEBUG PRINTING Non-Shared Triangle Partitions: %i Triangles: %i, First Cluster Id: %i\n", maxPartition, activeTriangles.size(), clusterId);

		PX_FREE(partitionProgresses);
		PX_FREE(tempTriangles);
	}

	/*******************************************************************************
	 *
	 *
	 * Triangle-pair partition
	 *
	 *
	 ******************************************************************************/

	PxU32 computeTrianglePairPartition(const uint4& vertexIndices, const PxU32 partitionStartIndex, PxU32* partitionProgresses)
	{
		PxU32 partitionA = partitionProgresses[vertexIndices.x];
		PxU32 partitionB = partitionProgresses[vertexIndices.y];
		PxU32 partitionC = partitionProgresses[vertexIndices.z];
		PxU32 partitionD = partitionProgresses[vertexIndices.w];

		const PxU32 combinedMask = (~partitionA & ~partitionB & ~partitionC & ~partitionD);
		PxU32 availablePartition = combinedMask == 0 ? FEMCLOTH_TEMP_PARTITION_SIZE : PxLowestSetBit(combinedMask);

		if(availablePartition == FEMCLOTH_TEMP_PARTITION_SIZE)
		{
			return 0xFFFFFFFF;
		}

		const PxU32 partitionBit = (1u << availablePartition);
		partitionA |= partitionBit;
		partitionB |= partitionBit;
		partitionC |= partitionBit;
		partitionD |= partitionBit;

		availablePartition += partitionStartIndex;

		partitionProgresses[vertexIndices.x] = partitionA;
		partitionProgresses[vertexIndices.y] = partitionB;
		partitionProgresses[vertexIndices.z] = partitionC;
		partitionProgresses[vertexIndices.w] = partitionD;

		return availablePartition;
	}

	void classifyTrianglePairs(const PxArray<uint4>& trianglePairVertices, const PxArray<PxU32>& activeTrianglePairs, PxU32 nbVerts,
							   PxU32* partitionProgresses, PxU32* tempTrianglePairs, PxArray<PxU32>& trianglePairsPerPartition)
	{
		// initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * nbVerts);
		PxU32 nbUnpartitionedTrianglePairs = 0;

		for(PxU32 it = 0; it < activeTrianglePairs.size(); ++it)
		{
			const PxU32 i = activeTrianglePairs[it];
			const uint4& vertexIndices = trianglePairVertices[i];
			const PxU32 availablePartition = computeTrianglePairPartition(vertexIndices, 0, partitionProgresses);

			if(availablePartition == 0xFFFFFFFF)
			{
				tempTrianglePairs[nbUnpartitionedTrianglePairs++] = it;
				continue;
			}

			trianglePairsPerPartition[availablePartition]++;
		}

		PxU32 partitionStartIndex = 0;
		while(nbUnpartitionedTrianglePairs > 0)
		{
			// initialize the partition progress counter to be zero
			PxMemZero(partitionProgresses, sizeof(PxU32) * nbVerts);

			partitionStartIndex += FEMCLOTH_TEMP_PARTITION_SIZE;

			// keep partitioning the un-partitioned constraints and blat the whole thing to 0!
			trianglePairsPerPartition.resize(FEMCLOTH_TEMP_PARTITION_SIZE + trianglePairsPerPartition.size());
			PxMemZero(trianglePairsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * FEMCLOTH_TEMP_PARTITION_SIZE);

			PxU32 newNbUnpartitionedConstraints = 0;

			for(PxU32 i = 0; i < nbUnpartitionedTrianglePairs; ++i)
			{
				const PxU32 trianglePairInd = tempTrianglePairs[i];
				const uint4& vertexIndices = trianglePairVertices[activeTrianglePairs[trianglePairInd]];
				const PxU32 availablePartition =
					computeTrianglePairPartition(vertexIndices, partitionStartIndex, partitionProgresses);

				if(availablePartition == 0xFFFFFFFF)
				{
					tempTrianglePairs[newNbUnpartitionedConstraints++] = trianglePairInd;
					continue;
				}

				trianglePairsPerPartition[availablePartition]++;
			}

			nbUnpartitionedTrianglePairs = newNbUnpartitionedConstraints;
		}
	}

	void writeTrianglePairs(const PxArray<uint4>& trianglePairVertices, const PxArray<PxU32>& activeTrianglePairs, const PxU32 numVerts,
							PxU32* partitionProgresses, PxU32* tempTrianglePairs, PxArray<PxU32>& orderedTrianglePairs,
							PxU32* accumulatedTrianglePairsPerPartition)
	{
		// initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

		PxU32 numUnpartitionedTrianglePairs = 0;

		for(PxU32 it = 0; it < activeTrianglePairs.size(); ++it)
		{
			const PxU32 i = activeTrianglePairs[it];
			const uint4& vertexIndices = trianglePairVertices[i];
			const PxU32 availablePartition = computeTrianglePairPartition(vertexIndices, 0, partitionProgresses);

			if(availablePartition == 0xFFFFFFFF)
			{
				tempTrianglePairs[numUnpartitionedTrianglePairs++] = it;
				continue;
			}

			// output triangle-pair
			orderedTrianglePairs[accumulatedTrianglePairsPerPartition[availablePartition]++] = it;
		}

		PxU32 partitionStartIndex = 0;

		while(numUnpartitionedTrianglePairs > 0)
		{
			// initialize the partition progress counter to be zero
			PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

			partitionStartIndex += FEMCLOTH_TEMP_PARTITION_SIZE;

			PxU32 newNumUnpartitionedConstraints = 0;

			for(PxU32 i = 0; i < numUnpartitionedTrianglePairs; ++i)
			{
				const PxU32 trianglePairInd = tempTrianglePairs[i];
				const uint4& vertexIndices = trianglePairVertices[activeTrianglePairs[trianglePairInd]];

				const PxU32 availablePartition =
					computeTrianglePairPartition(vertexIndices, partitionStartIndex, partitionProgresses);

				if(availablePartition == 0xFFFFFFFF)
				{
					tempTrianglePairs[newNumUnpartitionedConstraints++] = trianglePairInd;
					continue;
				}

				// output triangles
				orderedTrianglePairs[accumulatedTrianglePairsPerPartition[availablePartition]++] = trianglePairInd;
			}

			numUnpartitionedTrianglePairs = newNumUnpartitionedConstraints;
		}
	}

	PxU32* trianglePairPartitions(PxgFEMCloth& femCloth, PxArray<PxU32>& orderedTrianglePairs, const PxArray<PxU32>& activeTrianglePairs,
								  const PxArray<uint4>& trianglePairVertexIndices, bool isSharedTrianglePair)
	{
		const PxU32 nbTrianglePairs = activeTrianglePairs.size();
		const PxU32 nbVerts = femCloth.mNbVerts;

		// each vert has a partition progress counter
		PxU32* partitionProgresses = PX_ALLOCATE(PxU32, nbVerts, "partitionProgress");

		// this store the triangle-pair index for the unpartitioned triangle-pairs
		PxU32* tempTrianglePairs = PX_ALLOCATE(PxU32, nbTrianglePairs, "tempTrianglePairs");

		PxArray<PxU32> trianglePairsPerPartition;
		trianglePairsPerPartition.reserve(FEMCLOTH_TEMP_PARTITION_SIZE);
		trianglePairsPerPartition.forceSize_Unsafe(FEMCLOTH_TEMP_PARTITION_SIZE);

		PxMemZero(trianglePairsPerPartition.begin(), sizeof(PxU32) * FEMCLOTH_TEMP_PARTITION_SIZE);

		classifyTrianglePairs(trianglePairVertexIndices, activeTrianglePairs, nbVerts, partitionProgresses,
							  tempTrianglePairs, trianglePairsPerPartition);

		// compute number of partitions
		PxU32 maxPartition = 0;
		for(PxU32 a = 0; a < trianglePairsPerPartition.size(); ++a, ++maxPartition)
		{
			if(trianglePairsPerPartition[a] == 0)
				break;
		}

		PxU32* accumulatedTrianglePairsPerPartition =
			PX_ALLOCATE(PxU32, maxPartition, "accumulatedTrianglePairsPerPartition");

		// compute run sum
		PxU32 accumulation = 0;
		for(PxU32 a = 0; a < maxPartition; ++a)
		{
			PxU32 count = trianglePairsPerPartition[a];
			accumulatedTrianglePairsPerPartition[a] = accumulation;
			accumulation += count;
		}

		PX_ASSERT(accumulation == nbTrianglePairs);

		if(isSharedTrianglePair)
		{
			femCloth.mNbSharedTriPairPartitions = maxPartition;
			writeTrianglePairs(trianglePairVertexIndices, activeTrianglePairs, nbVerts, partitionProgresses, tempTrianglePairs,
							   orderedTrianglePairs, accumulatedTrianglePairsPerPartition);
		}
		else
		{
			femCloth.mNbNonSharedTriPairPartitions = maxPartition;
			writeTrianglePairs(trianglePairVertexIndices, activeTrianglePairs, nbVerts, partitionProgresses, tempTrianglePairs,
							   orderedTrianglePairs, accumulatedTrianglePairsPerPartition);
		}

		PX_FREE(partitionProgresses);
		PX_FREE(tempTrianglePairs);

		return accumulatedTrianglePairsPerPartition;
	}

	void combineTrianglePairPartitions(PxgFEMCloth& femCloth, PxArray<PxU32>& orderedTrianglePairs,
									   PxU32* accumulatedTrianglePairsPerPartition, PxgFEMClothData& clothData, PxU32 maximumPartitions,
									   const PxArray<PxU32>& activeTrianglePairs, const PxArray<uint4>& trianglePairVertexIndices,
									   bool isSharedTrianglePair, PxsHeapMemoryAllocator* alloc)
	{
		const PxU32 nbTrianglePairs = activeTrianglePairs.size();
		const PxU32 nbVerts = femCloth.mNbVerts;

		PxU32 nbPartitions;
		PxU32* combineAccumulatedTrianglePairsPerPartition;
		//PxU32* orderedTrianglePairs;
		PxU32* accumulatedCopiesEachVerts;
		PxU32* remapOutput;

		if(isSharedTrianglePair)
		{
			nbPartitions = femCloth.mNbSharedTriPairPartitions;
			femCloth.mSharedTriPairAccumulatedPartitionsCP =
				reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbPartitions, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			combineAccumulatedTrianglePairsPerPartition = femCloth.mSharedTriPairAccumulatedPartitionsCP;
			accumulatedCopiesEachVerts = femCloth.mSharedTriPairAccumulatedCopiesCP;
		}
		else
		{
			nbPartitions = femCloth.mNbNonSharedTriPairPartitions;
			femCloth.mNonSharedTriPairAccumulatedPartitionsCP =
				reinterpret_cast<PxU32*>(alloc->allocate(sizeof(PxU32) * nbPartitions, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			combineAccumulatedTrianglePairsPerPartition = femCloth.mNonSharedTriPairAccumulatedPartitionsCP;
			accumulatedCopiesEachVerts = femCloth.mNonSharedTriPairAccumulatedCopiesCP;
		}

		PxMemZero(combineAccumulatedTrianglePairsPerPartition, sizeof(PxU32) * maximumPartitions);

		PxArray<PxU32> tempOrderedTrianglePairs;
		tempOrderedTrianglePairs.reserve(orderedTrianglePairs.size());
		tempOrderedTrianglePairs.forceSize_Unsafe(orderedTrianglePairs.size());

		const PxU32 maxAccumulatedCP = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

		const PxU32 partitionArraySize = maxAccumulatedCP * maximumPartitions;
		const PxU32 nbPartitionTables = partitionArraySize * nbVerts;

		PxU32* tempPartitionTablePerVert = PX_ALLOCATE(PxU32, nbPartitionTables, "tempPartitionTablePerVert");
		PxU32* tempRemapTablePerVert = PX_ALLOCATE(PxU32, nbPartitionTables, "tempRemapTablePerVert");

		// initialize partitionTablePerVert
		for(PxU32 i = 0; i < nbPartitionTables; ++i)
		{
			tempPartitionTablePerVert[i] = 0xffffffff;
			tempRemapTablePerVert[i] = 0xffffffff;
		}

		PxU32 maxTrianglePairsPerPartition = 0;
		PxU32 count = 0;
		PxU32 clusterId = maximumPartitions;

		for(PxU32 i = 0; i < maximumPartitions; ++i)
		{
			PxU32 totalTrianglePairs = 0;
			for(PxU32 j = 0; j < maxAccumulatedCP; ++j)
			{
				PxU32 partitionId = i + maximumPartitions * j;
				if(partitionId < nbPartitions)
				{
					const PxU32 startInd = partitionId == 0 ? 0 : accumulatedTrianglePairsPerPartition[partitionId - 1];
					const PxU32 endInd = accumulatedTrianglePairsPerPartition[partitionId];

					for(PxU32 k = startInd; k < endInd; ++k)
					{
						const PxU32 trianglePairInd = orderedTrianglePairs[k];
						tempOrderedTrianglePairs[count] = trianglePairInd;

						PxU32 index = i * maxAccumulatedCP + j;

						const uint4& vertexIndices = trianglePairVertexIndices[activeTrianglePairs[trianglePairInd]];
						tempPartitionTablePerVert[vertexIndices.x * partitionArraySize + index] = count;
						tempPartitionTablePerVert[vertexIndices.y * partitionArraySize + index] = count + nbTrianglePairs;
						tempPartitionTablePerVert[vertexIndices.z * partitionArraySize + index] = count + nbTrianglePairs * 2;
						tempPartitionTablePerVert[vertexIndices.w * partitionArraySize + index] = count + nbTrianglePairs * 3;
						count++;
					}

					totalTrianglePairs += (endInd - startInd);
				}
			}

			combineAccumulatedTrianglePairsPerPartition[i] = count;
			maxTrianglePairsPerPartition = PxMax(maxTrianglePairsPerPartition, totalTrianglePairs);

			if (clusterId == maximumPartitions && count < PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
			{
				clusterId = i;
			}
			if (count >= PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL)
			{
				clusterId = maximumPartitions;
			}
		}

		PX_ASSERT(count == nbTrianglePairs);

		PxMemCopy(orderedTrianglePairs.begin(), &tempOrderedTrianglePairs[0], sizeof(PxU32) * tempOrderedTrianglePairs.size());

		const PxU32 totalNumVerts = nbTrianglePairs * 4;

		PxU32* tempNumCopiesEachVerts = PX_ALLOCATE(PxU32, nbVerts, "numCopiesEachVerts");
		PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) * nbVerts);

		bool* tempHasOccupied = PX_ALLOCATE(bool, partitionArraySize, "tempHasOccupied");

		// compute num of copies and remap index
		for(PxU32 i = 0; i < nbVerts; ++i)
		{
			PxMemZero(tempHasOccupied, sizeof(bool) * partitionArraySize);

			PxU32* partitionTable = &tempPartitionTablePerVert[i * partitionArraySize];
			PxU32* remapTable = &tempRemapTablePerVert[i * partitionArraySize];
			for(PxU32 j = 0; j < maximumPartitions; ++j)
			{
				const PxU32 startInd = j * maxAccumulatedCP;
				PxU32 nextStartInd = (j + 1) * maxAccumulatedCP;

				for(PxU32 k = 0; k < maxAccumulatedCP; ++k)
				{
					const PxU32 index = startInd + k;
					if(partitionTable[index] != 0xffffffff)
					{
						bool found = false;
						for(PxU32 h = nextStartInd; h < partitionArraySize; ++h)
						{
							const PxU32 remapInd = partitionTable[h];
							if(remapInd != 0xffffffff && !tempHasOccupied[h])
							{
								// find it
								remapTable[index] = remapInd;
								found = true;
								tempHasOccupied[h] = true;
								nextStartInd++;

								break;
							}
						}

						if(!found)
						{
							tempNumCopiesEachVerts[i]++;
						}
					}
				}
			}
		}

		// compute runSum for the copies
		PxU32 totalCopies = 0;
		for(PxU32 i = 0; i < nbVerts; ++i)
		{
			totalCopies += tempNumCopiesEachVerts[i];
			accumulatedCopiesEachVerts[i] = totalCopies;
		}

		if(isSharedTrianglePair)
		{
			clothData.mMaxNbSharedTriPairsPerPartition = maxTrianglePairsPerPartition;
			femCloth.mNbSharedTriPairPartitions = PxMin(maximumPartitions, femCloth.mNbSharedTriPairPartitions);

			clothData.mSharedTriPairRemapOutputSize = totalNumVerts + totalCopies;

			femCloth.mSharedTriPairRemapOutputCP = reinterpret_cast<PxU32*>(
				alloc->allocate(sizeof(PxU32) * clothData.mSharedTriPairRemapOutputSize, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			remapOutput = femCloth.mSharedTriPairRemapOutputCP;
			femCloth.mSharedTriPairClusterId = clusterId;
			//printf("DEBUG PRINTING Shared Tripair Partitions: %i Tripairs: %i, First Cluster: %i\n", nbPartitions, activeTrianglePairs.size(), clusterId);
		}
		else
		{
			clothData.mMaxNbNonSharedTriPairsPerPartition = maxTrianglePairsPerPartition;
			femCloth.mNbNonSharedTriPairPartitions = PxMin(maximumPartitions, femCloth.mNbNonSharedTriPairPartitions);
			clothData.mNonSharedTriPairRemapOutputSize = totalNumVerts + totalCopies;

			femCloth.mNonSharedTriPairRemapOutputCP = reinterpret_cast<PxU32*>(
				alloc->allocate(sizeof(PxU32) * clothData.mNonSharedTriPairRemapOutputSize, PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

			remapOutput = femCloth.mNonSharedTriPairRemapOutputCP;
			femCloth.mNonSharedTriPairClusterId = clusterId;
			//printf("DEBUG PRINTING Non-Shared Tripair Partitions: %i Tripairs: %i, First Cluster: %i\n", nbPartitions, activeTrianglePairs.size(), clusterId);
		}

		for(PxU32 i = 0; i < nbVerts; ++i)
		{
			const PxU32 index = i * partitionArraySize;
			PxU32* partitionTable = &tempPartitionTablePerVert[index];
			PxU32* remapTable = &tempRemapTablePerVert[index];

			PxU32 accumulatedCount = 0;
			for(PxU32 j = 0; j < partitionArraySize; ++j)
			{
				const PxU32 vertInd = partitionTable[j];
				if(vertInd != 0xffffffff)
				{
					PxU32 remapInd = remapTable[j];

					// this remap is in the accumulation buffer
					if(remapInd == 0xffffffff)
					{
						const PxU32 start = (i == 0) ? 0 : accumulatedCopiesEachVerts[i - 1];
						remapInd = totalNumVerts + start + accumulatedCount;
						accumulatedCount++;
					}
					remapOutput[vertInd] = remapInd;
				}
			}
		}

		PX_FREE(tempNumCopiesEachVerts);
		PX_FREE(tempHasOccupied);
		PX_FREE(tempPartitionTablePerVert);
		PX_FREE(tempRemapTablePerVert);
	}

	void PxgFEMClothCore::partitionTrianglePairSimData(PxgFEMCloth& femCloth, PxgFEMClothData& clothData, PxU32 maximumPartitions,
													   PxArray<PxU32>& orderedTrianglePairs, const PxArray<PxU32>& activeTrianglePairs,
													   const PxArray<uint4>& trianglePairVertexIndices, bool isSharedTrianglePair,
													   PxsHeapMemoryAllocator* alloc)
	{
		if (activeTrianglePairs.empty())
		{
			if (isSharedTrianglePair)
			{
				clothData.mMaxNbSharedTriPairsPerPartition = 0;
				femCloth.mNbSharedTriPairPartitions = 0;
				clothData.mSharedTriPairRemapOutputSize = 0;
				femCloth.mSharedTriPairClusterId = 0;
			}
			else
			{
				clothData.mMaxNbNonSharedTriPairsPerPartition = 0;
				femCloth.mNbNonSharedTriPairPartitions = 0;
				clothData.mNonSharedTriPairRemapOutputSize = 0;
				femCloth.mNonSharedTriPairClusterId = 0;
			}

			return;
		}

		orderedTrianglePairs.reserve(activeTrianglePairs.size());
		orderedTrianglePairs.forceSize_Unsafe(activeTrianglePairs.size());

		PxU32* accumulatedTrianglePairsPerPartition = trianglePairPartitions(femCloth, orderedTrianglePairs, activeTrianglePairs, trianglePairVertexIndices, isSharedTrianglePair);

		combineTrianglePairPartitions(femCloth, orderedTrianglePairs, accumulatedTrianglePairsPerPartition, clothData, maximumPartitions,
									  activeTrianglePairs, trianglePairVertexIndices, isSharedTrianglePair, alloc);

		PX_FREE(accumulatedTrianglePairsPerPartition);
	}

	void PxgFEMClothCore::drawContacts(PxRenderOutput& out)
	{
		PX_UNUSED(out);

	#if 0 // Vertext contacts stored in femCloth are no longer used. Having them for debug-render purpose only would be a
		  // waste.

			const PxU32 nbActiveFEMCloths = mSimController->getBodySimManager().mActiveFEMCloths.size();
			if (nbActiveFEMCloths == 0)
				return;

			PxgSimulationCore* simCore = mSimController->getSimulationCore();
			CUdeviceptr femClothsd = simCore->getFEMClothBuffer().getDevicePtr();

			PxArray<PxgFEMCloth> clothes(nbActiveFEMCloths);
			mCudaContext->memcpyDtoH(clothes.begin(), femClothsd, sizeof(PxgFEMCloth) * nbActiveFEMCloths);


			for (PxU32 i = 0; i < nbActiveFEMCloths; ++i)
			{
				PxgFEMCloth* cloth = &clothes[i];
				CUdeviceptr contactsd = reinterpret_cast<CUdeviceptr>(cloth->mRigidContactVertexes_restW);
				CUdeviceptr normalPensd = reinterpret_cast<CUdeviceptr>(cloth->mRigidContactVertexNormalPens);
				CUdeviceptr contactCountsd = reinterpret_cast<CUdeviceptr>(cloth->mRigidContactVertexCounts);

				const PxU32 maxCounts = cloth->mNbVerts * cloth->mMaxNumRigidContactPerVertex;

				PxArray<float4> points(maxCounts);
				PxArray<float4> normalPens(maxCounts);
				PxArray<PxU32> contactCounts(cloth->mNbVerts);

				mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * maxCounts);
				mCudaContext->memcpyDtoH(normalPens.begin(), normalPensd, sizeof(float4) * maxCounts);
				mCudaContext->memcpyDtoH(contactCounts.begin(), contactCountsd, sizeof(PxU32) * cloth->mNbVerts);

				const PxReal size = 0.02f;
				const PxU32 color = 0xff00ffff;

				for (PxU32 j = 0; j < cloth->mNbVerts; ++j)
				{
					const PxU32 offset = j * cloth->mMaxNumRigidContactPerVertex;
					float4* tPoints = &points[offset];
					float4* tNormalPens = &normalPens[offset];
					const PxU32 numContacts = contactCounts[j];
					for (PxU32 k = 0; k < numContacts; ++k)
					{
						PxVec3 a(tPoints[k].x, tPoints[k].y, tPoints[k].z);
						PxVec3 n(tNormalPens[k].x, tNormalPens[k].y, tNormalPens[k].z);
						const PxReal pen = tNormalPens[k].w;

						PxVec3 b = a - n * pen;

						const PxVec3 up(0.f, size, 0.f);
						const PxVec3 right(size, 0.f, 0.f);
						const PxVec3 forwards(0.f, 0.f, size);

						const PxMat44 m(PxIdentity);

						out << color << m << PxRenderOutput::LINES << a + up << a - up;
						out << color << m << PxRenderOutput::LINES << a + right << a - right;
						out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;


						out << color << m << PxRenderOutput::LINES << b + up << b - up;
						out << color << m << PxRenderOutput::LINES << b + right << b - right;
						out << color << m << PxRenderOutput::LINES << b + forwards << b - forwards;

						out << color << m << PxRenderOutput::LINES << a << b;
					}
				}
			}
	#endif
	}

} // namespace physx
