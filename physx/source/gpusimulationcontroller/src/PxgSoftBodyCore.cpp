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

#include "PxgNarrowphaseCore.h"
#include "PxgNphaseImplementationContext.h"
#include "PxsContext.h"
#include "PxgSoftBodyCore.h"
#include "foundation/PxAssert.h"
#include "common/PxProfileZone.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgCudaSolverCore.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgSimulationController.h"
#include "PxgSimulationCore.h"
#include "PxgNarrowphaseCore.h"
#include "PxgKernelWrangler.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgNpKernelIndices.h"
#include "PxgRadixSortDesc.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgCudaUtils.h"
#include "DyParticleSystem.h"
#include "DyDeformableVolume.h"
#include "GuTetrahedronMesh.h"
#include "PxgParticleSystemCore.h"
#include "PxgPBDParticleSystemCore.h"
#include "PxgFEMClothCore.h"
#include "PxgContext.h"
#include "PxgArticulationCore.h"

#include "cudamanager/PxCudaContext.h"

#define SB_GPU_DEBUG 0
#define DRAW_GRID	0

namespace physx
{
	extern "C" void initSoftBodyKernels0();
	extern "C" void initSoftBodyKernels1();

	void createPxgSoftBody()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXSimulationControllerGpu linkage as Static Library!
		initSoftBodyKernels0();
		initSoftBodyKernels1();
#endif
	}

	PxgSoftBodyCore::PxgSoftBodyCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
		PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* gpuContext, const PxU32 maxContacts, const PxU32 collisionStackSize, const bool isTGS) :
		PxgFEMCore(gpuKernelWrangler, cudaContextManager, heapMemoryManager, simController, gpuContext, maxContacts, collisionStackSize, isTGS, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactPointBuffer(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactNormalPenBuffer(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactBarycentricBuffer0(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactBarycentricBuffer1(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactInfoBuffer(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCTotalContactCountBuffer(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mPrevSCContactCountBuffer(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCConstraintBuf(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY),
		mSCLambdaNBuf(heapMemoryManager, PxsHeapStats::eSHARED_SOFTBODY), 
		mPostSolveCallback(NULL)
	{
		mGpuContext->mGpuSoftBodyCore = this;

		PxScopedCudaLock _lock(*mCudaContextManager);
		
		int leastPriority, mostPriority;
		cuCtxGetStreamPriorityRange(&leastPriority, &mostPriority);

		mCudaContext->streamCreateWithPriority(&mStream, CU_STREAM_NON_BLOCKING, leastPriority);

		mCudaContext->eventCreate(&mBoundUpdateEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveRigidEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mConstraintPrepSoftBodyParticleEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveSoftBodyParticleEvent, CU_EVENT_DISABLE_TIMING);

		const PxU32 contactSize = maxContacts * sizeof(float4);

		//soft body vs cloth contact buffer
		mSCContactPointBuffer.allocate(contactSize, PX_FL);
		mSCContactNormalPenBuffer.allocate(contactSize, PX_FL);
		mSCContactBarycentricBuffer0.allocate(contactSize, PX_FL);
		mSCContactBarycentricBuffer1.allocate(contactSize, PX_FL);
		mSCContactInfoBuffer.allocate(maxContacts * sizeof(PxgFemFemContactInfo), PX_FL);
		mSCTotalContactCountBuffer.allocate(sizeof(PxU32), PX_FL);
		mPrevSCContactCountBuffer.allocate(sizeof(PxU32), PX_FL);
		mSCLambdaNBuf.allocate(maxContacts * sizeof(PxReal), PX_FL);
	
		//KS - we divide by 32 because this is a block data format
		mFemConstraintBuf.allocate(((maxContacts + 31) / 32) * sizeof(PxgSoftBodySoftBodyConstraintBlock), PX_FL);

		//don't know what it should be yet!
		mSCConstraintBuf.allocate(((maxContacts + 31) / 32) * sizeof(PxgSoftBodySoftBodyConstraintBlock), PX_FL);
	}

	PxgSoftBodyCore::~PxgSoftBodyCore()
	{
		PxScopedCudaLock _lock(*mCudaContextManager);
		
		mCudaContext->eventDestroy(mBoundUpdateEvent);
		mBoundUpdateEvent = NULL;

		mCudaContext->eventDestroy(mSolveRigidEvent);
		mSolveRigidEvent = NULL;

		mCudaContext->eventDestroy(mConstraintPrepSoftBodyParticleEvent);
		mConstraintPrepSoftBodyParticleEvent = NULL;

		mCudaContext->eventDestroy(mSolveSoftBodyParticleEvent);
		mSolveSoftBodyParticleEvent = NULL;
	}


	void PxgSoftBodyCore::preIntegrateSystem(PxgDevicePointer<PxgSoftBody> softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
		const PxU32 nbActiveSoftBodies, const PxU32 maxVerts, const PxVec3 gravity, const PxReal dt, CUstream bpStream) 
	{
		{
			const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled() && mGpuContext->isTGS();

			const PxU32 totalNumSoftbodies = mGpuContext->getSimulationCore()->getNumTotalSoftbodies();
			
			const PxU32 totalSize = totalNumSoftbodies * sizeof(PxReal);
			mSpeculativeCCDContactOffset.allocate(totalSize, PX_FL);
			
			PxgDevicePointer<PxReal> speculativeCCDContactOffsetd = mSpeculativeCCDContactOffset.getTypedDevicePtr();
			
			mCudaContext->memsetD32Async(speculativeCCDContactOffsetd, 0, totalSize / sizeof(PxU32), bpStream);
			
			const CUfunction GMPreIntegrateKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SIM_PREINTEGRATION);

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlock = PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION / numThreadsPerWarp;
		
			const PxU32 numBlocks = (maxVerts + PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION - 1) / PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION;

			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
					PX_CUDA_KERNEL_PARAM(gravity),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(mIsTGS),
					PX_CUDA_KERNEL_PARAM(speculativeCCDContactOffsetd),
					PX_CUDA_KERNEL_PARAM(externalForcesEveryTgsIterationEnabled)
				};

				CUresult result = mCudaContext->launchKernel(GMPreIntegrateKernelFunction, numBlocks, nbActiveSoftBodies, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(bpStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU preIntegrateSystem kernel fail!\n");
#endif
			}

			if (1)
			{
				//4 threads to deal with one tetrahedron
				PxgSimulationCore* core = mSimController->getSimulationCore();
				const PxU32 maxTetrahedrons = core->getGMMaxTetrahedrons();
				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
				const PxU32 numBlocks2 = (maxTetrahedrons * 4 + numThreadsPerBlock - 1) / numThreadsPerBlock;
				//update duplicate verts in the combined partions from grid model
				const CUfunction GMUpdateCPVertsFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_ZERO_TETMULTIPLIERS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd)
				};

				CUresult result = mCudaContext->launchKernel(GMUpdateCPVertsFunction, numBlocks2, nbActiveSoftBodies, 1, numThreadsPerBlock, 1, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_gm_updateGMVertsLaunch kernel fail!\n");

				/*const PxU32 totalVerts = softbodies[0].mRemapOutputSizeGM;
				PxArray<float4> accumulatedBuffer;
				accumulatedBuffer.reserve(totalVerts);
				accumulatedBuffer.forceSize_Unsafe(totalVerts);

				result = mCudaContext->memcpyDtoH(accumulatedBuffer.begin(), (CUdeviceptr)softbodies[0].mGMPosition_InvMassCP[1 - mCurrentPosIndex], sizeof(float4) * totalVerts);*/

#endif
			}
		}
	}


	void PxgSoftBodyCore::preIntegrateSystems(PxgSoftBody* softbodies, PxU32* activeSoftbodies, const PxU32 nbActiveSoftbodies,
		const PxVec3 gravity, const PxReal dt)
	{

		PX_UNUSED(activeSoftbodies);
		PX_UNUSED(softbodies);

		//integrateSystems run on the broad phase stream so we don't need to have an extra event to sync in updateBounds
		CUstream bpStream = 0;
		if(mGpuContext->mGpuBp)
			bpStream = mGpuContext->mGpuBp->getBpStream();
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgTypedCudaBuffer<PxgSoftBody>& softBodiesBuffer = core->getSoftBodyBuffer();
		PxgDevicePointer<PxgSoftBody> softbodiesd = softBodiesBuffer.getTypedDevicePtr();

		PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

		//KS - if TGS, we do not pre-integrate here. Instead, we handle all integration inside the solver stepping scheme
		const PxU32 maxTetraVerts = core->getGMMaxTetraVerts();
		preIntegrateSystem(softbodiesd, activeSoftBodiesd, nbActiveSoftbodies, maxTetraVerts, gravity, dt, bpStream);

		updateTetModelVerts(softbodiesd, activeSoftBodiesd, nbActiveSoftbodies, bpStream);

	}

	void PxgSoftBodyCore::refitBound(PxgSoftBody* softbodies, const PxU32 nbActiveSoftbodies)
	{
		PX_UNUSED(softbodies);
		PX_UNUSED(nbActiveSoftbodies);

		//boundsd  and contactDistd are guaranteed in the GPU if updateBound run on the broad phase stream
		//update bounds need to run on the broad phase stream so the broad phase can kick off after particle update bound
		
		if (mGpuContext->mGpuBp == NULL)
			return;

		CUstream bpStream = mGpuContext->mGpuBp->getBpStream();

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgDevicePointer<PxgSoftBody> softBodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();
		PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();
		PxgCudaBuffer& sbElementIndexBuffer = core->getSoftBodyElementIndexBuffer();
		PxU32* sbElementIndexsd = reinterpret_cast<PxU32*>(sbElementIndexBuffer.getDevicePtr());

		CUdeviceptr boundsd = mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr();
		CUdeviceptr bpContactDistd = mGpuContext->mGpuBp->getContactDistBuffer().getDevicePtr();
		CUdeviceptr npContactDistd = mGpuContext->mGpuNpCore->mGpuContactDistance.getDevicePtr();
		PxgDevicePointer<PxReal> speculativeCCDContactOffsetd = mSpeculativeCCDContactOffset.getTypedDevicePtr();

		const PxU32 numBlocks = nbActiveSoftbodies;
		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlock = SB_REFIT_WAPRS_PER_BLOCK;

		const CUfunction refitBoundKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_REFITBOUND);

		{

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softBodiesd),
				PX_CUDA_KERNEL_PARAM(activeSoftBodiesd),
				PX_CUDA_KERNEL_PARAM(nbActiveSoftbodies),
				PX_CUDA_KERNEL_PARAM(bpContactDistd),
				PX_CUDA_KERNEL_PARAM(npContactDistd),
				PX_CUDA_KERNEL_PARAM(speculativeCCDContactOffsetd),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(sbElementIndexsd)
			};

			CUresult result = mCudaContext->launchKernel(refitBoundKernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(bpStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateBound first pass kernel fail!\n");

			//Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

#endif

		}
	}

	void PxgSoftBodyCore::resetContactCounts()
	{
		//total contact count for selfCollision and softbody vs softbody
		CUdeviceptr totalSSContactCountsd = mVolumeContactOrVTContactCountBuffer.getDevicePtr();
		//CUdeviceptr prevSSContactsd = mPrevFemContactCountBuffer.getDevicePtr();

		//total contact count for particle and soft body
		CUdeviceptr totalSPContactCountsd = mParticleTotalContactCountBuffer.getDevicePtr();
		//CUdeviceptr prevSPContactsd = mPrevSPContactCountBuffer.getDevicePtr();

		//total contact count for rigid body and soft body
		CUdeviceptr totalRSContactCountsd = mRigidTotalContactCountBuf.getDevicePtr();
		//CUdeviceptr prevRSContactCountsd = mRigidPrevContactCountBuf.getDevicePtr();

		//total contact count for cloth vs soft body
		CUdeviceptr totalSCContactCountsd = mSCTotalContactCountBuffer.getDevicePtr();
		//CUdeviceptr prevSCCountCountsd = mPrevSCContactCountBuffer.getDevicePtr();

		mCudaContext->memsetD32Async(totalSSContactCountsd, 0, 1, mStream);
		//mCudaContext->memsetD32Async(prevSSContactsd, 0, 1, mStream);
		mCudaContext->memsetD32Async(totalSPContactCountsd, 0, 1, mStream);
		//mCudaContext->memsetD32Async(prevSPContactsd, 0, 1, mStream);
		mCudaContext->memsetD32Async(totalRSContactCountsd, 0, 1, mStream);
		//mCudaContext->memsetD32Async(prevRSContactCountsd, 0, 1, mStream);
		mCudaContext->memsetD32Async(totalSCContactCountsd, 0, 1, mStream);
		//mCudaContext->memsetD32Async(prevSCCountCountsd, 0, 1, mStream);

		mCudaContext->memsetD32Async(mStackSizeNeededOnDevice.getDevicePtr(), 0, 1, mStream);
	}

	void PxgSoftBodyCore::checkBufferOverflows()
	{
		PxU32 contactCountNeeded = PxMax(*mParticleContactCountPrevTimestep, PxMax(*mRigidContactCountPrevTimestep, *mVolumeContactorVTContactCountPrevTimestep));
		if (contactCountNeeded >= mMaxContacts) 
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "Deformable volume contact buffer overflow detected, please increase PxGpuDynamicsMemoryConfig::maxDeformableVolumeContacts to at least %u\n", contactCountNeeded);
		}

		if (*mStackSizeNeededPinned > mCollisionStackSizeBytes)
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow detected, please increase its size to at least %i in the scene desc! Contacts have been dropped.\n", *mStackSizeNeededPinned);
		}

#if PX_ENABLE_SIM_STATS
		mContactCountStats = PxMax(mContactCountStats, contactCountNeeded);
		mGpuContext->getSimStats().mGpuDynamicsDeformableVolumeContacts = mContactCountStats;

		mCollisionStackSizeBytesStats = PxMax(*mStackSizeNeededPinned, mCollisionStackSizeBytesStats);
		mGpuContext->getSimStats().mGpuDynamicsCollisionStackSize = PxMax(mCollisionStackSizeBytesStats, mGpuContext->getSimStats().mGpuDynamicsCollisionStackSize);
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	}

	void PxgSoftBodyCore::selfCollision()
	{
		PX_PROFILE_ZONE("PxgSoftBodyCore::selfCollision", 0);
		//This make sure refitBound kernel are finished in GPU
		if (mGpuContext->mGpuBp == NULL)
			return;
		CUstream bpStream = mGpuContext->mGpuBp->getBpStream();

		synchronizeStreams(mCudaContext, bpStream, mStream, mBoundUpdateEvent);

		const PxU32 nbActiveSelfCollisionSoftbodies = mSimController->getBodySimManager().mActiveSelfCollisionSoftBodiesStaging.size();

		//const PxU32* activeSoftbodies = mSimController->getActiveSoftBodies();
		//PxgSoftBody* softboides = mSimController->getSoftBodies();

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& softBodiesBuffer = core->getSoftBodyBuffer();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(softBodiesBuffer.getDevicePtr());
		PxReal* contactDistd = reinterpret_cast<PxReal*>(mGpuContext->mGpuBp->getContactDistBuffer().getDevicePtr());
		PxU32* activeSoftBodiesd = reinterpret_cast<PxU32*>(core->getActiveSelfCollisionSoftBodyBuffer().getDevicePtr());


		PxgDevicePointer<PxU32> totalNumContactsd = mVolumeContactOrVTContactCountBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxU32> prevContactsd = mPrevFemContactCountBuffer.getTypedDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevContactsd, totalNumContactsd, sizeof(PxU32), mStream);

		if (nbActiveSelfCollisionSoftbodies)
		{
			PxgSoftBodyContactWriter writer(this);

			CUfunction scMidphaseFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SELFCOLLISION_MIDPHASE);

			PxCudaKernelParam scMidphaseKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(activeSoftBodiesd),
				PX_CUDA_KERNEL_PARAM(contactDistd),
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(writer)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;

			CUresult result = mCudaContext->launchKernel(scMidphaseFunction, numBlocks, nbActiveSelfCollisionSoftbodies, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, scMidphaseKernelParams, sizeof(scMidphaseKernelParams), 0, PX_FL);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_selfCollisionMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_selfCollisionMidphaseGeneratePairsLaunch fail!!\n");

#endif
		}


		if(nbActiveSelfCollisionSoftbodies)
		{
			PxgDevicePointer<float4> barycentric0d = mFemContactBarycentric0Buffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentric1d = mFemContactBarycentric1Buffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mVolumeContactOrVTContactInfoBuffer.getTypedDevicePtr();

			mGpuContext->getNarrowphaseCore()->softbodyFemContactApplyCollisionToSimMeshMapping(barycentric0d, barycentric1d, contactInfosd, 
				totalNumContactsd, prevContactsd, true, false);
		}
	}


	void PxgSoftBodyCore::sortContacts(const PxU32 nbActiveSoftbodies)
	{

		//We need 2x rsDesc on the host soft body system. The reason for this is that, while the sorting occurs synchronously on the 
		//same stream on the device, the host-side buffers could get changed prior to the DMAs having occurred due to device latency
		const PxU32 nbRequired = (nbActiveSoftbodies)+2u;
		mRSDesc.resize(nbRequired * 2u);

		//mRadixCountSize = sizeof(PxU32) * PxgRadixSortKernelGridDim::RADIX_SORT * 16;

		mRadixCountTotalBuf.allocate(mRadixCountSize * nbRequired, PX_FL);

		for (PxU32 i = 0; i < 2; ++i)
		{
			mRadixSortDescBuf[i].allocate(sizeof(PxgRadixSortBlockDesc) * nbRequired, PX_FL);
		}

		//total number of rigid vs soft body contacts
		PxgDevicePointer<PxU32> totalRSContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		//total number of particle vs soft body contacts
		PxgDevicePointer<PxU32> totalSPContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

		//total number of FEM Cloth vs soft body contacts
		PxgDevicePointer<PxU32> totalSCContactCountsd = mSCTotalContactCountBuffer.getTypedDevicePtr();

		CUfunction clampFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLAMP_MAX_VALUES);
		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(totalRSContactCountsd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
				PX_CUDA_KERNEL_PARAM(totalSCContactCountsd),
				PX_CUDA_KERNEL_PARAM(mMaxContacts)
			};

			CUresult resultR = mCudaContext->launchKernel(clampFunction, 1, 1, 1, 1, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU clampMaxValues fail to launch kernel!!\n");

#if SB_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU clampMaxValues fail!!\n");
#endif
		}

		//sort contacts based on rigid id
		PxgDevicePointer<PxU32> inputKeyd = mTempContactByRigidBitBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> inputRankd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputKeyd = mTempContactBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputRankd = mTempContactRemapBuf.getTypedDevicePtr();

		updateGPURadixSortBlockDesc(mStream, inputKeyd, inputRankd, outputKeyd, outputRankd, mRadixCountTotalBuf.getDevicePtr(),
			totalRSContactCountsd, &mRSDesc[2 * nbActiveSoftbodies], mRadixSortDescBuf[0].getDevicePtr(), mRadixSortDescBuf[1].getDevicePtr());

		//sorted contacts based on particle id
		PxgDevicePointer<PxU32> inputKeyd2 = mTempContactByParticleBitBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> inputRankd2 = mContactRemapSortedByParticleBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputKeyd2 = mTempContactBuf2.getTypedDevicePtr();
		PxgDevicePointer<PxU32> outputRankd2 = mTempContactRemapBuf2.getTypedDevicePtr();

		updateGPURadixSortBlockDesc(mStream, inputKeyd2, inputRankd2, outputKeyd2, outputRankd2, mRadixCountTotalBuf.getDevicePtr() + mRadixCountSize,
			totalSPContactCountsd, &mRSDesc[2 * nbActiveSoftbodies + 2], mRadixSortDescBuf[0].getDevicePtr() + sizeof(PxgRadixSortBlockDesc),
			mRadixSortDescBuf[1].getDevicePtr() + sizeof(PxgRadixSortBlockDesc));

		PxgCudaBuffer* radixSortDescBuf = mRadixSortDescBuf.begin();

		CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK);
		CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK);

		{
			PxU32 startBit = 0;
			const PxU32 numPass = 8;

			for (PxU32 i = 0; i < numPass; ++i)
			{
				const PxU32 descIndex = i & 1;

				CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

				PxCudaKernelParam radixSortKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rsDesc),
					PX_CUDA_KERNEL_PARAM(startBit)
				};

				CUresult resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortSoftBodyContacts fail to launch kernel!!\n");

				resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortSoftBodyContacts fail to launch kernel!!\n");

				startBit += 4;

			}
#if SB_GPU_DEBUG
			CUresult result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortSoftBodyContacts fail!!\n");
#endif
		}

		PxgDevicePointer<PxNodeIndex> contactByRigidd = mContactByRigidBuf.getTypedDevicePtr();
		{
			//copy the higher 32 bit to the team contact rigid index buffer
			PxgDevicePointer<PxU32> tempContactByRigidd = mTempContactByRigidBitBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> rankd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();
		
			CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_HIGH_32BITS);
		
			PxCudaKernelParam copyKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(contactByRigidd),
				PX_CUDA_KERNEL_PARAM(tempContactByRigidd),
				PX_CUDA_KERNEL_PARAM(rankd),
				PX_CUDA_KERNEL_PARAM(totalRSContactCountsd)
			};
		
			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail to launch kernel!!\n");
		
#if SB_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail!!\n");
#endif
		}
		
		{
			//sort tempContactByRidid again
			PxU32 startBit = 0;
			const PxU32 numPass = 8;
		
			for (PxU32 i = 0; i < numPass; ++i)
			{
				const PxU32 descIndex = i & 1;
		
				CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();
		
				PxCudaKernelParam radixSortKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rsDesc),
					PX_CUDA_KERNEL_PARAM(startBit)
				};
		
				CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");
		
				resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");
		
				startBit += 4;
			}
		}

		{
			//copy the original rigidId to the sorted buffer based on mContactRemapSortedByRigidBuf
			PxgDevicePointer<PxNodeIndex> outContactByRigidd = mContactSortedByRigidBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> rankd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();

			CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_VALUE);

			PxCudaKernelParam copyKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(contactByRigidd),
				PX_CUDA_KERNEL_PARAM(outContactByRigidd),
				PX_CUDA_KERNEL_PARAM(rankd),
				PX_CUDA_KERNEL_PARAM(totalRSContactCountsd)
			};

			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail to launch kernel!!\n");

#if SB_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail!!\n");
#endif
		}


		reorderRigidContacts();


		PxgDevicePointer<PxU64> contactByParticled = mContactSortedByParticleBuf.getTypedDevicePtr();
		{
			//copy the higher 32 bit to the team contact rigid index buffer
			PxgDevicePointer<PxU32> tempContactByParticled = mTempContactByParticleBitBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> rankd = mContactRemapSortedByParticleBuf.getTypedDevicePtr();

			CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_HIGH_32BITS);

			PxCudaKernelParam copyKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(contactByParticled),
				PX_CUDA_KERNEL_PARAM(tempContactByParticled),
				PX_CUDA_KERNEL_PARAM(rankd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd)
			};

			CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail to launch kernel!!\n");

#if SB_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail!!\n");
#endif
		}

		{
			//sort tempContactByParticle again
			PxU32 startBit = 0;
			const PxU32 numPass = 8;

			const PxU32 descSize = sizeof(PxgRadixSortBlockDesc);

			for (PxU32 i = 0; i < numPass; ++i)
			{
				const PxU32 descIndex = i & 1;

				CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr() + descSize;

				PxCudaKernelParam radixSortKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rsDesc),
					PX_CUDA_KERNEL_PARAM(startBit)
				};

				CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

				resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

				startBit += 4;
			}
		}

		{
			PxgDevicePointer<float4> contactsd = mParticleContactPointBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> normPensd = mParticleContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentricsd = mParticleContactBarycentricBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFemOtherContactInfo> infosd = mParticleContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> sortedContactsd = mParticleSortedContactPointBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> sortedNormPensd = mParticleSortedContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> sortedBarycentricsd = mParticleSortedContactBarycentricBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFemOtherContactInfo> sortedInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxU32> remapByParticleId = mContactRemapSortedByParticleBuf.getTypedDevicePtr();

			CUfunction reorderFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_REORDER_PS_CONTACTS);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(normPensd),
				PX_CUDA_KERNEL_PARAM(barycentricsd),
				PX_CUDA_KERNEL_PARAM(infosd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
				PX_CUDA_KERNEL_PARAM(remapByParticleId),
				PX_CUDA_KERNEL_PARAM(sortedContactsd),
				PX_CUDA_KERNEL_PARAM(sortedNormPensd),
				PX_CUDA_KERNEL_PARAM(sortedBarycentricsd),
				PX_CUDA_KERNEL_PARAM(sortedInfosd)
			};

			CUresult  resultR = mCudaContext->launchKernel(reorderFunction, PxgSoftBodyKernelGridDim::SB_REORDERCONTACTS, 1, 1, PxgSoftBodyKernelBlockDim::SB_REORDERCONTACTS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_reorderPSContactsLaunch fail to launch kernel!!\n");
#if SB_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_reorderPSContactsLaunch fail!!\n");
#endif
		}
	}

	void PxgSoftBodyCore::updateSimTetraRotations()
	{

		const PxU32 nbActiveSoftbodies = mSimController->getNbActiveSoftBodies();
		if (nbActiveSoftbodies == 0)
			return;

		PxgSimulationCore* core = mSimController->getSimulationCore();


		PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();
		PxgDevicePointer<PxU32> activeSoftbodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

		const PxU32 maxSimTetrahedrons = core->getGMMaxTetrahedrons();

		if (maxSimTetrahedrons > 0)
		{
			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
			const PxU32 numBlocks = (maxSimTetrahedrons + numThreadsPerBlock - 1) / numThreadsPerBlock;

			const CUfunction updateGMTRKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_UPDATETETRAROTATIONS);

			{

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd)
				};

				CUresult result = mCudaContext->launchKernel(updateGMTRKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_gm_updateTetraRotationsLaunch kernel fail!\n");
#endif

			}
		}
	}

	void PxgSoftBodyCore::updateTetraRotations()
	{
		const PxU32 nbActiveSoftbodies = mSimController->getNbActiveSoftBodies();
		if (nbActiveSoftbodies == 0)
			return;

		PxgSimulationCore* core = mSimController->getSimulationCore();

		PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();
		PxgDevicePointer<PxU32> activeSoftbodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

		const PxU32 maxTetrahedrons = core->getMaxTetrahedrons();
		if (maxTetrahedrons > 0)
		{
			{
				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = (maxTetrahedrons + numThreadsPerBlock - 1) / numThreadsPerBlock;

				const CUfunction updateTRKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_UPDATETETRAROTATIONS);

				{

					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(softbodiesd),
						PX_CUDA_KERNEL_PARAM(activeSoftbodiesd)
					};

					CUresult result = mCudaContext->launchKernel(updateTRKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);

#if SB_GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					PX_ASSERT(result == CUDA_SUCCESS);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_updateTetraRotationsLaunch kernel fail!\n");
#endif
				}
			}
		}

	}

	void PxgSoftBodyCore::updateTetModelVerts(PxgDevicePointer<PxgSoftBody> softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
		const PxU32 nbActiveSoftbodies, CUstream updateStream) 
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();

		const PxU32 maxTetrahedrons = core->getGMMaxTetrahedrons();
		if (maxTetrahedrons > 0)
		{
			//original tet model
			const PxU32 maxTetraVerts = core->getMaxTetraVerts();
			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
			const PxU32 numBlocks = (maxTetraVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

			//const PxU32 numBlocks = nbActiveSoftbodies * NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
			{

				const CUfunction updateTetModelKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_UPDATETETMODELVERTS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd)
				};

				CUresult result = mCudaContext->launchKernel(updateTetModelKernelFunction, numBlocks, nbActiveSoftbodies, 1, PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION, 1, 1, 0, updateStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(updateStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_updateTetModelVertsLaunch kernel fail!\n");

#endif

			}
		}
	}


	void PxgSoftBodyCore::applyExternalTetraDeltaGM(const PxU32 nbActiveSoftbodies, const PxReal dt, CUstream stream)
	{
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.applyExternalTetra", 0);
#endif
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxU32* activeSoftbodiesd = reinterpret_cast<PxU32*>(core->getActiveSoftBodyBuffer().getDevicePtr());

		const PxReal invDt = 1.f / dt;
		//const PxU32 numBlocks = nbActiveSoftbodies * NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;

		const PxU32 maxVerts = core->getGMMaxTetraVerts();

		const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
		const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

		{

			const CUfunction solveTetraKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_APPLY_EXTERNAL_DELTAS);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
				PX_CUDA_KERNEL_PARAM(invDt)
			};

			CUresult result = mCudaContext->launchKernel(solveTetraKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_applyExternalDeltasLaunch first pass kernel fail!\n");
#endif
		}
	}

	void PxgSoftBodyCore::finalizeVelocities(const PxReal dt, const PxReal scale, const bool isTGS)
	{
		PX_ASSERT(scale >= 0.f && scale <= 1.f);
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.finalizeVelocities", 0);
#endif
		PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();
		if (nbActiveSoftbodies > 0)
		{

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();
			PxU32* activeSoftbodiesd = reinterpret_cast<PxU32*>(core->getActiveSoftBodyBuffer().getDevicePtr());

			const PxReal invDt = 1.f / dt;

			const PxU32 maxVerts = core->getGMMaxTetraVerts();

			const PxU32 totalSoftBodies = mSimController->getBodySimManager().mTotalNumSoftBodies;

			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION;

			const bool alwaysRunVelocityAveraging = isTGS && !mGpuContext->isExternalForcesEveryTgsIterationEnabled();

			{
				const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUfunction finalizeVelocitiesKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_FINALIZE_VELOCITIES);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
					PX_CUDA_KERNEL_PARAM(invDt),
					PX_CUDA_KERNEL_PARAM(scale),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(alwaysRunVelocityAveraging)
				};

				CUresult result = mCudaContext->launchKernel(finalizeVelocitiesKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_gm_finalizeVelocitiesLaunch kernel fail!\n");
#endif
			}


			{
				const PxReal resetCounter = 0.4f;
				const PxU32 numBlocks = (nbActiveSoftbodies + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUfunction softBodySleepingFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SLEEPING);

				PxU32* stateChanged = core->getActiveSBStateChangedMap().getWords();
				PxgDevicePointer<PxReal> wakeCountersGPU = core->getActiveSBWakeCountsGPU();
				PxReal* wakeCountersCPU = core->getActiveSBWakeCountsCPU();

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(nbActiveSoftbodies),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(resetCounter),
					PX_CUDA_KERNEL_PARAM(wakeCountersGPU),
					PX_CUDA_KERNEL_PARAM(stateChanged)
				};

				CUresult result = mCudaContext->launchKernel(softBodySleepingFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_sleeping kernel fail!\n");
#endif

				// AD: this memcopy will probably flush the queues on windows, maybe there is a better place for this..
				mCudaContext->memcpyDtoHAsync(wakeCountersCPU, wakeCountersGPU.mPtr, totalSoftBodies * sizeof(PxReal), mStream);
			}

			updateTetModelVerts(softbodiesd, reinterpret_cast<CUdeviceptr>(activeSoftbodiesd), nbActiveSoftbodies, mStream);

			//calculate stress for collision mesh
			calculateStress();

			// AD this doesn't really run so let's not launch it.
			//plasticDeformation();
		}

		//Record event - wait in host code before reading wake counters/state changed...
		mCudaContext->eventRecord(mFinalizeEvent, mStream);
		mCudaContext->streamFlush(mStream);

		if (mPostSolveCallback) 
			mPostSolveCallback->onPostSolve(mFinalizeEvent);
	}

	void PxgSoftBodyCore::syncSoftBodies()
	{
		PX_PROFILE_ZONE("PxgSoftBodyCore::syncSoftBodies", 0);
		mCudaContextManager->acquireContext();
		mCudaContext->eventSynchronize(mFinalizeEvent);
		mCudaContextManager->releaseContext();
	}

	void PxgSoftBodyCore::createActivatedDeactivatedLists()
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxBitMapPinned& sbChangedMap = core->getActiveSBStateChangedMap();

		PxArray<Dy::DeformableVolume*>& deformableVolumes = mSimController->getBodySimManager().mDeformableVolumes;

		PxgBodySimManager& bodyManager = mSimController->getBodySimManager();

		PxReal* wakeCounters = core->getActiveSBWakeCountsCPU();

		mActivatingDeformableVolumes.forceSize_Unsafe(0);
		mDeactivatingDeformableVolumes.forceSize_Unsafe(0);

		PxBitMapPinned::Iterator iter(sbChangedMap);

		PxU32 dirtyIdx;
		while ((dirtyIdx = iter.getNext()) != PxBitMapPinned::Iterator::DONE)
		{
			PX_ASSERT(dirtyIdx < bodyManager.mActiveSoftbodies.size());
			PxU32 idx = bodyManager.mActiveSoftbodies[dirtyIdx];
			if (wakeCounters[idx] == 0.f)
				mDeactivatingDeformableVolumes.pushBack(deformableVolumes[idx]);
			else
				mActivatingDeformableVolumes.pushBack(deformableVolumes[idx]);
		}

		sbChangedMap.clear();
	}
	
	void PxgSoftBodyCore::solveRSContactsOutputRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
		PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt) 
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();
		{
			const CUfunction solveOutputRigidDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_COLLISION);

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());


			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			PxsMaterialData* rigidBodyMaterials = reinterpret_cast<PxsMaterialData*>(npCore->mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();


			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PxgDevicePointer<PxReal> appliedForced = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidReferenceCount.getDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(solverCoreDescd),
				PX_CUDA_KERNEL_PARAM(artiCoreDescd),
				PX_CUDA_KERNEL_PARAM(sharedDescd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(femRigidContactCount),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(materials),
				PX_CUDA_KERNEL_PARAM(rigidBodyMaterials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputRigidDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			mCudaContext->eventRecord(mSolveRigidEvent, solverStream);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveRigidSoftCollisionLaunch first pass kernel fail!\n");

			int bob = 0;
			PX_UNUSED(bob);
#endif
		}

		//accumulate velocity delta for rigid body and impulse delta for articulation link
		accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, mRigidSortedRigidIdBuf.getDevicePtr(),
							  mRigidTotalContactCountBuf.getDevicePtr(), solverStream, false);
	}

	void PxgSoftBodyCore::solveRSContactsOutputRigidDeltaTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
		PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt) 
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.solveRigidContactOutputRigid", 0);
#endif
		{
			const CUfunction solveOutputRigidDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_COLLISION_TGS);

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			PxsMaterialData* rigidBodyMaterials = reinterpret_cast<PxsMaterialData*>(npCore->mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PxgDevicePointer<PxReal> appliedForced = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidReferenceCount.getDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(solverCoreDescd),
				PX_CUDA_KERNEL_PARAM(artiCoreDescd),
				PX_CUDA_KERNEL_PARAM(sharedDescd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(femRigidContactCount),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(materials),
				PX_CUDA_KERNEL_PARAM(rigidBodyMaterials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputRigidDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			mCudaContext->eventRecord(mSolveRigidEvent, solverStream);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveRigidSoftCollisionLaunch first pass kernel fail!\n");

			int bob = 0;
			PX_UNUSED(bob);
#endif
		}

		//if the contact is between articulation and soft body, after accumulated all the related contact's
		//impulse, we need to propagate the accumulated impulse to the articulation block solver
		accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, mRigidSortedRigidIdBuf.getDevicePtr(),
							  mRigidTotalContactCountBuf.getDevicePtr(), solverStream, true);
	}


	void PxgSoftBodyCore::solveCorotationalFEM(PxgSoftBody* softbodies, PxgSoftBody* softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
		const PxU32 nbActiveSoftbodies, const PxReal dt, CUstream stream, const bool isTGS, const bool isFirstIteration) 
	{
		PX_UNUSED(softbodies);

		PxgSimulationCore* core = mSimController->getSimulationCore();

#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.solveCorotational", 0);
#endif

		const PxU32 maxTetrahedrons = core->getGMMaxTetrahedrons();
		if (maxTetrahedrons > 0)
		{
			const PxReal invDt = 1.0f / dt;

			//multiple blocks solve one of the soft bodies
			{
				PxU32 maxPartitions = core->getGMMaxTetraPartitions();

				bool hasExtraJacobi = maxPartitions > SB_PARTITION_LIMIT; // 8th partition is for Jacobi-style update.
				maxPartitions = PxMin(maxPartitions, PxU32(SB_PARTITION_LIMIT));

				PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
				CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

				{
					const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA_LOW;
					const PxU32 maxTetetrahedronsPerPartitions = core->getGMMaxTetrahedronsPerPartition();
					const PxU32 numBlocks = (maxTetetrahedronsPerPartitions + numThreadsPerBlock - 1) / numThreadsPerBlock;

					for (PxU32 i = 0; i < maxPartitions; ++i)
					{
						const CUfunction GMCPSolveTetraKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_CP_SOLVE_TETRA);

						PxCudaKernelParam kernelParams[] =
						{
							PX_CUDA_KERNEL_PARAM(softbodiesd),
							PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
							PX_CUDA_KERNEL_PARAM(nbActiveSoftbodies),
							PX_CUDA_KERNEL_PARAM(invDt),
							PX_CUDA_KERNEL_PARAM(i),
							PX_CUDA_KERNEL_PARAM(isTGS),
							PX_CUDA_KERNEL_PARAM(isFirstIteration),
							PX_CUDA_KERNEL_PARAM(materials)
						};

						CUresult result = mCudaContext->launchKernel(GMCPSolveTetraKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
						PX_ASSERT(result == CUDA_SUCCESS);
						PX_UNUSED(result);
					}

#if SB_GPU_DEBUG
					CUresult result = mCudaContext->streamSynchronize(mStream);
					PX_ASSERT(result == CUDA_SUCCESS);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveTetrahedronsPartitionLaunch1 kernel fail!\n");
#endif
				}

				if(hasExtraJacobi) // the cost of this should be relatively light or negligible in comparison with 8
			                       // partition runs above in most cases.
				{
				    // compute delta x, Jacobi style.
				    {
					    const PxU32 maxJacobiTets = core->getGMMaxJacobiTetrahedrons();
					    const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA_LOW;
					    const PxU32 numBlocks = (maxJacobiTets + numThreadsPerBlock - 1) / numThreadsPerBlock;

						const CUfunction GMCPSolveJacobiKernelFunction =
					        mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					            PxgKernelIds::SB_GM_CP_SOLVE_TETRA_JACOBI_PARTITION);

					    PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(softbodiesd),
						                                     PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
						                                     PX_CUDA_KERNEL_PARAM(nbActiveSoftbodies),
						                                     PX_CUDA_KERNEL_PARAM(invDt),
						                                     PX_CUDA_KERNEL_PARAM(isTGS),
						                                     PX_CUDA_KERNEL_PARAM(materials) };

					    CUresult result = mCudaContext->launchKernel(
					        GMCPSolveJacobiKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1,
					        0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					    PX_ASSERT(result == CUDA_SUCCESS);
					    PX_UNUSED(result);

#if SB_GPU_DEBUG
					    result = mCudaContext->streamSynchronize(mStream);
					    PX_ASSERT(result == CUDA_SUCCESS);
					    if(result != CUDA_SUCCESS)
						    PxGetFoundation().error(
						        PxErrorCode::eINTERNAL_ERROR, PX_FL,
						        "GPU sb_gm_cp_solveTetrahedronsJacobiPartitionLaunch kernel fail!\n");
#endif
				    }

				    // apply deformation delta with precomputed scale.
				    {
					    const PxU32 maxVerts = core->getGMMaxJacobiVertices();
					    const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
					    const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

					    const CUfunction solveTetraKernelFunction =
					        mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					            PxgKernelIds::SB_GM_APPLY_DEFORMATION_DELTAS);
					    PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(softbodiesd),
						                                     PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
						                                     PX_CUDA_KERNEL_PARAM(invDt) };

					    CUresult result = mCudaContext->launchKernel(
					        solveTetraKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0,
					        stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					    PX_ASSERT(result == CUDA_SUCCESS);
					    PX_UNUSED(result);

#if SB_GPU_DEBUG
					    result = mCudaContext->streamSynchronize(mStream);
					    PX_ASSERT(result == CUDA_SUCCESS);
					    if(result != CUDA_SUCCESS)
						    PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
						                            "GPU sb_gm_applyDeformationDeltasLaunch kernel fail!\n");
#endif
				    }
			    }
		    }

			// average verts
			if (core->getGMUsePartitionAveraging())
			{
				const PxU32 maxTetraVerts = core->getGMMaxTetraVerts();
				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_SOLVETETRA;
				const PxU32 numBlocks = (maxTetraVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;

				const CUfunction GMCPAverageKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_CP_AVERAGEVERTS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
					PX_CUDA_KERNEL_PARAM(invDt)
				};

				CUresult result = mCudaContext->launchKernel(GMCPAverageKernelFunction, numBlocks, nbActiveSoftbodies, 1, PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_averageVertsLaunch kernel fail!\n");

#endif
			}
			
#if SB_GPU_DEBUG & DRAW_GRID

			CUresult result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_averageVertsLaunch kernel fail!\n");

			PxgNphaseImplementationContext* npContext = mNpCore->mNphaseImplContext;

			PxsContext& context = npContext->getContext();
			RenderOutput render = context.getRenderOutput();

			PxgSoftBody& sb = softbodies[1];
			const PxU32 numTetsGM = sb.mNumTetsGM;
			const PxU32 numVertsGM = sb.mNumVertsGM;
			PxArray<PxVec4> pos;
			pos.reserve(numVertsGM);
			pos.forceSize_Unsafe(numVertsGM);
			PxArray<uint4> tetIndices;
			tetIndices.reserve(numTetsGM);
			tetIndices.forceSize_Unsafe(numTetsGM);

			PxArray<PxU32> orderedTets;
			orderedTets.reserve(numTetsGM);
			orderedTets.forceSize_Unsafe(numTetsGM);

			mCudaContext->memcpyDtoH(pos.begin(), (CUdeviceptr)sb.mGridModelPosition_InvMass, sizeof(PxVec4) * numVertsGM);
			mCudaContext->memcpyDtoH(tetIndices.begin(), (CUdeviceptr)sb.mGridModelTetIndices, sizeof(uint4)*numTetsGM);
			mCudaContext->memcpyDtoH(orderedTets.begin(), (CUdeviceptr)sb.mGridModelOrderedTetrahedrons, sizeof(PxU32) * numTetsGM);

			const PxMat44 m(PxIdentity);

			//draw tet
			for (PxU32 i = 0; i < numTetsGM; ++i)
			{
				const PxU32 tetrahedronInd = orderedTets[i];
				uint4 tetInd = tetIndices[tetrahedronInd];
				const PxVec3 p0 = pos[tetInd.x].getXYZ();
				const PxVec3 p1 = pos[tetInd.y].getXYZ();
				const PxVec3 p2 = pos[tetInd.z].getXYZ();
				const PxVec3 p3 = pos[tetInd.w].getXYZ();


				render << 0xffff00ff << m << RenderOutput::LINES << p0 << p1;
				render << 0xffff00ff << m << RenderOutput::LINES << p0 << p2;
				render << 0xffff00ff << m << RenderOutput::LINES << p0 << p3;
				render << 0xffff00ff << m << RenderOutput::LINES << p1 << p2;
				render << 0xffff00ff << m << RenderOutput::LINES << p1 << p3;
				render << 0xffff00ff << m << RenderOutput::LINES << p2 << p3;
			}
#endif
		}
	}

	void PxgSoftBodyCore::solveSPContactsOutputSoftBodyDelta(const PxReal dt, const PxReal biasCoefficient)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();
		
		PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalSPContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

		//solve soft body vs particle contact in the soft body stream and update delta and applied force for soft body
		{

			const CUfunction solveOutputSoftBodyDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_PARTICLE_SOFT_DELTA);

			//PxgSimulationCore* core = mSimController->getSimulationCore();
			//PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgFEMParticleConstraintBlock> constraintsd = mParticleConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<float4> appliedForced = mParticleAppliedFEMForcesBuf.getTypedDevicePtr();

			const PxReal relaxation = biasCoefficient * dt;

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(particlesystemsd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(relaxation),
				PX_CUDA_KERNEL_PARAM(materials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputSoftBodyDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			PxU32 numSoftCount;
			mCudaContext->memcpyDtoH(&numSoftCount, totalSPContactCountsd, sizeof(PxU32));

			if (numSoftCount > 0)
			{
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputSSDeltaVLaunch kernel fail!\n");

				/*PxgSoftBody* softbodies = mSimController->getSoftBodies();
				PxU32* activeSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.begin();
				PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();

				PxArray<float4> deltaV;

				for (PxU32 i = 0; i < nbActiveSoftbodies; ++i)
				{
					const PxU32 index = activeSoftbodies[i];
					PxgSoftBody& softbody = softbodies[index];
					const PxU32 numVerts = softbody.mNumVerts;

					deltaV.reserve(numVerts);
					deltaV.forceSize_Unsafe(numVerts);

					mCudaContext->memcpyDtoH(deltaV.begin(), (CUdeviceptr)softbodies[i].mDelta, sizeof(float4)*numVerts);

					int bob = 0;
					PX_UNUSED(bob);
				}*/
			}
#endif
			
		}
	}

	//solve soft body vs particle system in particle stream
	void PxgSoftBodyCore::solveSPContactsOutputParticleDelta(const PxReal dt, const PxReal biasCoefficient, CUstream particleStream)
	{
		//solve soft body vs particle contact in the particle system stream and update selfCollision delta for particle system
		PxgSimulationCore* core = mGpuContext->getSimulationCore(); 
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		
		PxgPBDParticleSystemCore* particleCore =  mSimController->getPBDParticleSystemCore();

		PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();

		PxgDevicePointer<float4> deltaVd = particleCore->getDeltaVelParticle();

		PxgDevicePointer<PxU32> totalSPContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

#if SB_GPU_DEBUG
		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalSPContactCountsd, sizeof(PxU32));
#endif 

		{

			//particley stream need to wait till soft body vs particle constraint prep finish in soft body stream
			mCudaContext->streamWaitEvent(particleStream, mConstraintPrepSoftBodyParticleEvent);
			//synchronizeStreams(mStream, particleStream);

			const CUfunction solveOutputParticleDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_PARTICLE_PARTICLE_DELTA);

			PxgDevicePointer<PxgFEMParticleConstraintBlock> constraintsd = mParticleConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<float4> appliedForced = mParticleAppliedParticleForcesBuf.getTypedDevicePtr();

			const PxReal relaxation = dt * biasCoefficient;

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(particlesystemsd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(relaxation),
				PX_CUDA_KERNEL_PARAM(materials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputParticleDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			mCudaContext->eventRecord(mSolveSoftBodyParticleEvent, particleStream);

#if SB_GPU_DEBUG

			if (numContacts > 0)
			{
				result = mCudaContext->streamSynchronize(particleStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputSSDeltaVLaunch kernel fail!\n");

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
			//those temp buffer store the start and end index for the particle vs soft body range sorted by particle id
			PxgDevicePointer<PxU32> pairCountd = mTempHistogramCountBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> startd = mTempContactBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> endd = mTempContactRemapBuf.getTypedDevicePtr();

			//accumulate deltaV changes for particle
			{
				const CUfunction accumulatedDeltaVKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_ACCUMULATE_FEM_PARTICLE_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particlesystemsd),
					PX_CUDA_KERNEL_PARAM(contactInfosd),
					PX_CUDA_KERNEL_PARAM(pairCountd),
					PX_CUDA_KERNEL_PARAM(startd),
					PX_CUDA_KERNEL_PARAM(endd),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(dt)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(accumulatedDeltaVKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG

				//if (numContacts > 0)
				//{
				//	result = mCudaContext->streamSynchronize(particleStream);
				//	if (result != CUDA_SUCCESS)
				//		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_accumulateParticleDeltaVLaunch kernel fail!\n");
				//
				//	PxgParticleSystem* particleSystems = mSimController->getParticleSystems();
				//	PxU32* activeParticleSystems = mSimController->getBodySimManager().mActiveParticleSystems.begin();
				//	PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActiveParticleSystems.size();

				//	PxArray<float4> deltaV;

				//	for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
				//	{
				//		const PxU32 index = activeParticleSystems[i];
				//		PxgParticleSystem& particleSystem = particleSystems[index];
				//		const PxU32 numVerts = particleSystem.mData.mNumParticles;

				//		deltaV.reserve(numVerts);
				//		deltaV.forceSize_Unsafe(numVerts);

				//		mCudaContext->memcpyDtoH(deltaV.begin(), (CUdeviceptr)particleSystem.mAccumDeltaV, sizeof(float4)*numVerts);

				//		int bob = 0;
				//		PX_UNUSED(bob);
				//	}
				//}
#endif
			}
			
		}

	}

	
	void PxgSoftBodyCore::solveSSContactsOutputSoftBodyDelta(const PxReal dt, const float biasCoefficient,
		const bool isTGS)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
	
		PxgDevicePointer<PxU32> totalSSContactCountsd = mVolumeContactOrVTContactCountBuffer.getTypedDevicePtr();

		//solve self collision contacts or soft body vs soft body contacts
		{


			const CUfunction solveOutputSoftBodyDeltaKernelFunction = isTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_SOFT_SOFT_BOTH_DELTA_TGS)
				: mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_SOFT_SOFT_BOTH_DELTA);

			//PxgSimulationCore* core = mSimController->getSimulationCore();
			//PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mVolumeContactOrVTContactInfoBuffer.getTypedDevicePtr();
			CUdeviceptr constraintsd = mFemConstraintBuf.getDevicePtr();

			PxgDevicePointer<float4> appliedForced = mFemAppliedForcesBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalSSContactCountsd),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(biasCoefficient),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(materials)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputSoftBodyDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			PxU32 numSoftCount;
			mCudaContext->memcpyDtoH(&numSoftCount, totalSSContactCountsd, sizeof(PxU32));

			if (numSoftCount > 0)
			{
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputSSDeltaVLaunch kernel fail!\n");

				PxgSoftBody* softbodies = mSimController->getSoftBodies();
				PxU32* activeSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.begin();
				PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();

				PxArray<float4> deltaV;

				for (PxU32 i = 0; i < nbActiveSoftbodies; ++i)
				{
					const PxU32 index = activeSoftbodies[i];
					PxgSoftBody& softbody = softbodies[index];
					const PxU32 numVerts = softbody.mNumVerts;

					deltaV.reserve(numVerts);
					deltaV.forceSize_Unsafe(numVerts);

					mCudaContext->memcpyDtoH(deltaV.begin(), (CUdeviceptr)softbodies[i].mSimDelta, sizeof(float4)*numVerts);

					int bob = 0;
					PX_UNUSED(bob);
				}
			}
#endif
		}
	}


	void PxgSoftBodyCore::solveSCContactsOutputDelta()
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());

		PxgDevicePointer<PxU32> totalSCContactCountsd = mSCTotalContactCountBuffer.getTypedDevicePtr();

		//solve soft body vs cloth contacts
		{
			const CUfunction solveOutputSoftBodyDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_SOFT_CLOTH_BOTH_DELTA);
			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mSCContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgSoftBodySoftBodyConstraintBlock> constraintsd = mSCConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<PxReal> lambdaNs = mSCLambdaNBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(clothesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalSCContactCountsd),
				PX_CUDA_KERNEL_PARAM(lambdaNs)
			};

			CUresult result = mCudaContext->launchKernel(solveOutputSoftBodyDeltaKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			PxU32 numSoftCount;
			mCudaContext->memcpyDtoH(&numSoftCount, totalSCContactCountsd, sizeof(PxU32));

			if (numSoftCount > 0)
			{
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputSSDeltaVLaunch kernel fail!\n");

				PxgSoftBody* softbodies = mSimController->getSoftBodies();
				PxU32* activeSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.begin();
				PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();

				PxArray<float4> deltaV;

				for (PxU32 i = 0; i < nbActiveSoftbodies; ++i)
				{
					const PxU32 index = activeSoftbodies[i];
					PxgSoftBody& softbody = softbodies[index];
					const PxU32 numVerts = softbody.mNumVerts;

					deltaV.reserve(numVerts);
					deltaV.forceSize_Unsafe(numVerts);

					mCudaContext->memcpyDtoH(deltaV.begin(), (CUdeviceptr)softbodies[i].mSimDelta, sizeof(float4)*numVerts);

					int bob = 0;
					PX_UNUSED(bob);
				}
			}
#endif
		}
	}

	void PxgSoftBodyCore::queryRigidContactReferenceCount(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
		PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
		PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt)
	{
		PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidReferenceCount.getDevicePtr();
		mCudaContext->memsetD32Async(femRigidContactCount.mPtr, 0, mFemRigidReferenceCount.getNbElements(), solverStream);

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		const CUfunction kernelFunction =
			mIsTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_QUERY_RIGID_SOFT_REFERENCE_COUNT_TGS)
				   : mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_QUERY_RIGID_SOFT_REFERENCE_COUNT);

		PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
		PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();

		PxgDevicePointer<PxReal> lambdaNs = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(softbodiesd),
											 PX_CUDA_KERNEL_PARAM(contactInfosd),
											 PX_CUDA_KERNEL_PARAM(constraintsd),
											 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
											 PX_CUDA_KERNEL_PARAM(prePrepDescd),
											 PX_CUDA_KERNEL_PARAM(solverCoreDescd),
											 PX_CUDA_KERNEL_PARAM(artiCoreDescd),
											 PX_CUDA_KERNEL_PARAM(sharedDescd),
											 PX_CUDA_KERNEL_PARAM(dt),
											 PX_CUDA_KERNEL_PARAM(lambdaNs),
											 PX_CUDA_KERNEL_PARAM(femRigidContactCount) };

		CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
			PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, solverStream, kernelParams,
			sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
		result = mCudaContext->streamSynchronize(solverStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_queryRigidSoftBodyContactReferenceCountLaunch kernel fail!\n");
#endif
	}

	void PxgSoftBodyCore::solveRigidAttachment(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt) 
	{
		
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbRigidSoftBodyAttachments();

		if(nbRigidAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
			
			PxgDevicePointer<PxgFEMRigidAttachmentConstraint> constraintsd = simCore->getSoftBodyRigidConstraints();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			
			{
				const CUfunction solvePCRigidKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_ATTACHMENT);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(deltaVd)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solvePCOutputRigidDeltaVLaunch kernel fail!\n");
				
#endif
			}

			mCudaContext->eventRecord(mSolveRigidEvent, solverStream);


			PxgDevicePointer<PxNodeIndex> rigidAttachmentIds = simCore->getSoftBodyRigidAttachmentIds();
			PxgDevicePointer<PxU32> totalRigidAttachmentsd = simCore->getGpuSoftBodyRigidCounter();

			//we need to wait for mSolveSoftBodyEvent to indicate the kernel in solveRigidAttachmentSoftBodyDelta() finish reading
			//solver body velocites before we update them
			accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, rigidAttachmentIds, totalRigidAttachmentsd,
								  solverStream, false);
		}
	}

	void PxgSoftBodyCore::solveRigidAttachmentTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt, const PxReal biasCoefficient,
		bool isVelocityIteration)
	{
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.rigidAttachmentRigidbodyDelta", 0);
#endif
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbRigidSoftBodyAttachments();

		if (nbRigidAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());

			PxgDevicePointer<PxgFEMRigidAttachmentConstraint> constraintsd = simCore->getSoftBodyRigidConstraints();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();


			{
				const CUfunction solvePCRigidKernelFunction =
					mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_ATTACHMENT_TGS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(biasCoefficient),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(isVelocityIteration)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solvePCOutputRigidDeltaVLaunch kernel fail!\n");
#endif
			}


			PxgDevicePointer<PxNodeIndex> rigidAttachmentIds = simCore->getSoftBodyRigidAttachmentIds();
			PxgDevicePointer<PxU32> totalRigidAttachmentsd = simCore->getGpuSoftBodyRigidCounter();

			mCudaContext->eventRecord(mSolveRigidEvent, solverStream);

			accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, rigidAttachmentIds, totalRigidAttachmentsd,
								  solverStream, true);
		}
	}

	void PxgSoftBodyCore::solveSoftBodyAttachmentDelta()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbSoftBodyAttachments = simCore->getNbSoftBodySoftBodyAttachments();

		if (nbSoftBodyAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());

			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getSoftBodySoftBodyConstraints();

			{
				const CUfunction solvePCRigidKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_SOFTBODY_ATTACHMENT_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbSoftBodyAttachments)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputAttachmentSoftDeltaVLaunchTGS kernel fail!\n");
#endif
			}
		}
	}

	void PxgSoftBodyCore::solveParticleAttachmentDelta()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbParticleAttachments = simCore->getNbSoftBodyParticleAttachments();

		if (nbParticleAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
			
			PxgParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();
			
			PxgDevicePointer<PxgParticleSystem> particlesd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getSoftBodyParticleConstraints();

			{
				const CUfunction solveParticleAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_PARTICLE_ATTACHMENT_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(particlesd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbParticleAttachments)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solveParticleAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputAttachmentSoftDeltaVLaunchTGS kernel fail!\n");
#endif
			}
	
		}
	}

	void PxgSoftBodyCore::solveClothAttachmentDelta()
	{

		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbClothAttachments = simCore->getNbClothSoftBodyAttachments();

		if (nbClothAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
			PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(simCore->getFEMClothBuffer().getDevicePtr());

			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getSoftBodyClothConstraints();

			{
				const CUfunction solvePCRigidKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SOLVE_CLOTH_ATTACHMENT_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(clothesd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbClothAttachments)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveOutputAttachmentSoftDeltaVLaunchTGS kernel fail!\n");
#endif
			}
		}
	}

	void PxgSoftBodyCore::prepRigidAttachmentConstraints(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
		const PxReal /*invDt*/, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream stream, bool /*isTGS*/) 
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbRigidSoftBodyAttachments();

		if (nbRigidAttachments)
		{

			PxgDevicePointer<PxgSoftBody> softbodiesd = simCore->getSoftBodyBuffer().getTypedDevicePtr();

			PxgDevicePointer<PxgFEMRigidAttachment> rigidAttachments = simCore->getRigidSoftBodyAttachments();
			PxgDevicePointer<PxU32> activeRigidAttachments = simCore->getActiveRigidSoftBodyAttachments();
			PxgDevicePointer<PxgFEMRigidAttachmentConstraint> constraintsd = simCore->getSoftBodyRigidConstraints();
			PxgDevicePointer<PxNodeIndex> rigidAttachmentIds = simCore->getSoftBodyRigidAttachmentIds();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PX_ASSERT(simCore->getNbRigidSoftBodyAttachments() * 2 <= mRigidDeltaVelBuf.getNbElements());

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_RIGID_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(rigidAttachments),
					PX_CUDA_KERNEL_PARAM(activeRigidAttachments),
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(prepDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(deltaVd)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepRigidAttachments ps_rigidAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);
#endif
			}
		}
	}

	void PxgSoftBodyCore::prepSoftBodyAttachmentConstraints(CUstream stream)
	{

		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbSoftBodyAttachments = simCore->getNbSoftBodySoftBodyAttachments();

		if (nbSoftBodyAttachments)
		{

			PxgDevicePointer<PxgFEMFEMAttachment> softBodyAttachments = simCore->getSoftBodySoftBodyAttachments();
			PxgDevicePointer<PxU32> activeSoftBodyAttachments = simCore->getActiveSoftBodySoftAttachments();
			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getSoftBodySoftBodyConstraints();
			//PxgDevicePointer<PxU32> softBodyAttachmentIds = simCore->getSoftBodySoftBodyAttachmentIds().getTypedDevicePtr();

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::FEM_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{

					PX_CUDA_KERNEL_PARAM(softBodyAttachments),
					PX_CUDA_KERNEL_PARAM(activeSoftBodyAttachments),
				/*	PX_CUDA_KERNEL_PARAM(softBodyAttachmentIds),*/
					PX_CUDA_KERNEL_PARAM(nbSoftBodyAttachments),
					PX_CUDA_KERNEL_PARAM(constraintsd)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothAttachmentPrepareLaunch sb_femAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);
#endif
			}
		}

	}

	void PxgSoftBodyCore::prepClothAttachmentConstraints(CUstream stream)
	{

		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbClothAttachments = simCore->getNbClothSoftBodyAttachments();

		if (nbClothAttachments)
		{

			PxgDevicePointer<PxgFEMFEMAttachment> clothAttachments = simCore->getClothSoftBodyAttachments();
			PxgDevicePointer<PxU32> activeClothAttachments = simCore->getActiveClothSoftBodyAttachments();
			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getSoftBodyClothConstraints();
			//PxgDevicePointer<PxU32> clothAttachmentIds = simCore->getSoftBodyClothAttachmentIds().getTypedDevicePtr();

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::FEM_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{
		
					PX_CUDA_KERNEL_PARAM(clothAttachments),
					PX_CUDA_KERNEL_PARAM(activeClothAttachments),
					/*PX_CUDA_KERNEL_PARAM(clothAttachmentIds),*/
					PX_CUDA_KERNEL_PARAM(nbClothAttachments),
					PX_CUDA_KERNEL_PARAM(constraintsd)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothAttachmentPrepareLaunch ps_rigidAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);
#endif
			}
		}

	}

	void PxgSoftBodyCore::prepParticleAttachmentConstraints(CUstream stream)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbParticleAttachments = simCore->getNbSoftBodyParticleAttachments();

		if (nbParticleAttachments)
		{

			PxgDevicePointer<PxgFEMFEMAttachment> particleAttachments = simCore->getSoftBodyParticleAttachments();
			PxgDevicePointer<PxU32> activeParticleAttachments = simCore->getActiveSoftBodyParticleAttachments();
			PxgDevicePointer<PxgFEMFEMAttachmentConstraint> constraintsd = simCore->getSoftBodyParticleConstraints();
			//PxgDevicePointer<PxU32> particleAttachmentIds = simCore->getSoftBodyParticleAttachmentIds().getTypedDevicePtr();

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::FEM_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{

					PX_CUDA_KERNEL_PARAM(particleAttachments),
					PX_CUDA_KERNEL_PARAM(activeParticleAttachments),
			/*		PX_CUDA_KERNEL_PARAM(particleAttachmentIds),*/
					PX_CUDA_KERNEL_PARAM(nbParticleAttachments),
					PX_CUDA_KERNEL_PARAM(constraintsd)
				};

				const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
				const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
				CUresult result = mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothAttachmentPrepareLaunch ps_rigidAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);
#endif
			}
		}
	}

	void PxgSoftBodyCore::prepRigidContactConstraint(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
		const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream, const bool isTGS, PxU32 numSolverBodies, PxU32 numArticulations)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		PxgDevicePointer<PxgSoftBody> softbodiesd = simCore->getSoftBodyBuffer().getTypedDevicePtr();

		//Prepare rigid body vs soft body contacts
		{
			PxgDevicePointer<float4> contactsd = mRigidSortedContactPointBuf.getTypedDevicePtr();
			PxgDevicePointer<float4> normalpensd = mRigidSortedContactNormalPenBuf.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentricsd = mRigidSortedContactBarycentricBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

			PxgDevicePointer<PxgFemRigidConstraintBlock> constraintsd = mRigidConstraintBuf.getTypedDevicePtr();
			PxgDevicePointer<PxReal> rigidAppliedForced = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();

			// Allocating femRigidContactCount based on the size of rigid bodies plus articulations.
			if (mFemRigidReferenceCount.getNbElements() != numSolverBodies + numArticulations)
			{
				mFemRigidReferenceCount.allocateElements(numSolverBodies + numArticulations, PX_FL);
			}

			const CUfunction rigidContactPrepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_RS_CONTACTPREPARE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(normalpensd),
				PX_CUDA_KERNEL_PARAM(barycentricsd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(prepDescd),
				PX_CUDA_KERNEL_PARAM(rigidAppliedForced),
				PX_CUDA_KERNEL_PARAM(invDt),
				PX_CUDA_KERNEL_PARAM(sharedDescd),
				PX_CUDA_KERNEL_PARAM(isTGS)
			};

			CUresult result = mCudaContext->launchKernel(rigidContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(solverStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_prepContacts first pass kernel fail!\n");


			int bob = 0;
			PX_UNUSED(bob);
#endif
		}

	}


	//Prepare soft body vs particle system contact constraints
	void PxgSoftBodyCore::prepSoftBodyParticleConstraint()
	{
		PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();
		if (particleCore)
		{

			PxgSimulationCore* simCore = mSimController->getSimulationCore();
			PxgDevicePointer<PxgSoftBody> softbodiesd = simCore->getSoftBodyBuffer().getTypedDevicePtr();
			PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxU32> totalContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();
			{

				PxgDevicePointer<float4> contactsd = mParticleSortedContactPointBuffer.getTypedDevicePtr();
				PxgDevicePointer<float4> normalpensd = mParticleSortedContactNormalPenBuffer.getTypedDevicePtr();
				PxgDevicePointer<float4> barycentricd = mParticleSortedContactBarycentricBuffer.getTypedDevicePtr();

				PxgDevicePointer<PxgFEMParticleConstraintBlock> constraintsd = mParticleConstraintBuf.getTypedDevicePtr();
				PxgDevicePointer<float4> softbodyAppliedForced = mParticleAppliedFEMForcesBuf.getTypedDevicePtr();
				PxgDevicePointer<float4> particleAppliedForced = mParticleAppliedParticleForcesBuf.getTypedDevicePtr();

				const CUfunction softbodyContactPrepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SP_CONTACTPREPARE);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(particlesystemsd),
					PX_CUDA_KERNEL_PARAM(contactsd),
					PX_CUDA_KERNEL_PARAM(normalpensd),
					PX_CUDA_KERNEL_PARAM(barycentricd),
					PX_CUDA_KERNEL_PARAM(contactInfosd),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(softbodyAppliedForced),
					PX_CUDA_KERNEL_PARAM(particleAppliedForced)
				};

				CUresult result = mCudaContext->launchKernel(softbodyContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_particleContactPrepareLaunch first pass kernel fail!\n");

				/*PxU32 numContacts;
				mCudaContext->memcpyDtoH(&numContacts, totalContactCountsd, sizeof(PxU32));

				if (numContacts > 0)
				{
					int bob = 0;
					PX_UNUSED(bob);
				}*/

#endif

			}

			//PxgParticleSystemCore* particleSystemCore = mSimController->getParticleSystemCore();
			//CUstream particleStream = particleSystemCore->getStream();
			//compute start and end index for sorted contact based on particle id
			{

				////particley stream need to wait till soft body vs particle constraint 
				//mCudaContext->streamWaitEvent(particleStream, mConstraintPrepSoftBodyParticleEvent);

				//CUdeviceptr contactsd = mSPSortedContactPointBuffer.getDevicePtr();
				PxgDevicePointer<PxU32> blockOffsetd = mTempBlockCellsHistogramBuf.getTypedDevicePtr();
				CUdeviceptr offsetd = mTempCellsHistogramBuf.getDevicePtr();
				PxgDevicePointer<PxU32> pairCountd = mTempHistogramCountBuf.getTypedDevicePtr();
				PxgDevicePointer<PxU32> startd = mTempContactBuf.getTypedDevicePtr();
				PxgDevicePointer<PxU32> endd = mTempContactRemapBuf.getTypedDevicePtr();

				//compute blockOffset and offset array for particle
				{
					const CUfunction findStartEndFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_FIND_RANGESTARTEND_FEM_FIRST);

					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(particlesystemsd),
						PX_CUDA_KERNEL_PARAM(contactInfosd),
						PX_CUDA_KERNEL_PARAM(totalContactCountsd),
						PX_CUDA_KERNEL_PARAM(blockOffsetd),
						PX_CUDA_KERNEL_PARAM(offsetd),
					};

					const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA;
					const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA;
					CUresult result = mCudaContext->launchKernel(findStartEndFirstKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
#if SB_GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_findStartEndParticleFirst kernel fail!\n");

					/*PxU32 totalNumContacts;
					mCudaContext->memcpyDtoH(&totalNumContacts, totalContactCountsd, sizeof(PxU32));

					if (totalNumContacts > 0)
					{
					PxArray<PxU32> offsets;
					offsets.reserve(totalNumContacts);
					offsets.forceSize_Unsafe(totalNumContacts);

					PxU32 blockOffset[32];
					mCudaContext->memcpyDtoH(offsets.begin(), offsetd, sizeof(PxU32) * totalNumContacts);
					mCudaContext->memcpyDtoH(blockOffset, blockOffsetd, sizeof(PxU32) * 32);

					int bob = 0;
					PX_UNUSED(bob);
					}*/
#endif
				}


				//compute start and end range for particle
				{
					const CUfunction findStartEndSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_RANGESTARTEND_FEM_SECONE);

					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(contactInfosd),
						PX_CUDA_KERNEL_PARAM(totalContactCountsd),
						PX_CUDA_KERNEL_PARAM(blockOffsetd),
						PX_CUDA_KERNEL_PARAM(offsetd),
						PX_CUDA_KERNEL_PARAM(pairCountd),
						PX_CUDA_KERNEL_PARAM(startd),
						PX_CUDA_KERNEL_PARAM(endd)
					};

					const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA;
					const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA;
					CUresult result = mCudaContext->launchKernel(findStartEndSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
#if SB_GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_findStartEndParticleSecond kernel fail!\n");

					/*PxU32 pairCount;
					mCudaContext->memcpyDtoH(&pairCount, pairCountd, sizeof(PxU32));

					PxArray<PxU32> rangeStart;
					rangeStart.reserve(pairCount);
					rangeStart.forceSize_Unsafe(pairCount);

					PxArray<PxU32> rangeEnd;
					rangeEnd.reserve(pairCount);
					rangeEnd.forceSize_Unsafe(pairCount);

					mCudaContext->memcpyDtoH(rangeStart.begin(), startd, sizeof(PxU32) * pairCount);
					mCudaContext->memcpyDtoH(rangeEnd.begin(), endd, sizeof(PxU32) * pairCount);

					int bob = 0;
					PX_UNUSED(bob);*/

#endif
				}
			}
		}

		mCudaContext->eventRecord(mConstraintPrepSoftBodyParticleEvent, mStream);
	}


	//Prepare soft body vs cloth contact constraints
	void PxgSoftBodyCore::prepSoftBodyClothConstraint()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		PxgDevicePointer<PxgSoftBody> softbodiesd = simCore->getSoftBodyBuffer().getTypedDevicePtr();
		PxgDevicePointer<PxgFEMCloth> clothesd = simCore->getFEMClothBuffer().getTypedDevicePtr();

		{
			PxgDevicePointer<float4> normalpensd = mSCContactNormalPenBuffer.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentric0d = mSCContactBarycentricBuffer0.getTypedDevicePtr();
			PxgDevicePointer<float4> barycentric1d = mSCContactBarycentricBuffer1.getTypedDevicePtr();
			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mSCContactInfoBuffer.getTypedDevicePtr();

			PxgDevicePointer<PxU32> totalContactCountsd = mSCTotalContactCountBuffer.getTypedDevicePtr();

			PxgDevicePointer<PxgSoftBodySoftBodyConstraintBlock> constraintsd = mSCConstraintBuf.getTypedDevicePtr();

			PxgDevicePointer<PxReal> lambdaNs = mSCLambdaNBuf.getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();

			CUdeviceptr softbodyMaterials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			CUdeviceptr clothMaterials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			const CUfunction softbodyClothContactPrepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SC_CONTACTPREPARE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(clothesd),
				PX_CUDA_KERNEL_PARAM(normalpensd),
				PX_CUDA_KERNEL_PARAM(barycentric0d),
				PX_CUDA_KERNEL_PARAM(barycentric1d),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(lambdaNs),
				PX_CUDA_KERNEL_PARAM(mMaxContacts),
				PX_CUDA_KERNEL_PARAM(softbodyMaterials),
				PX_CUDA_KERNEL_PARAM(clothMaterials)
			};

			CUresult result = mCudaContext->launchKernel(softbodyClothContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothContactPrepareLaunch kernel fail!\n");

			PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, totalContactCountsd, sizeof(PxU32));

			if (numContacts > 0)
			{
				int bob = 0;
				PX_UNUSED(bob);
			}

#endif
		}
	}


	//Prepare soft body self collision/soft body contact constraints
	void PxgSoftBodyCore::prepSoftbodyContactConstraint()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		CUdeviceptr softbodiesd = simCore->getSoftBodyBuffer().getDevicePtr();

		PxgDevicePointer<float4> contactsd = mFemContactPointBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> normalpensd = mFemContactNormalPenBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentric0d = mFemContactBarycentric0Buffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentric1d = mFemContactBarycentric1Buffer.getTypedDevicePtr();
		PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mVolumeContactOrVTContactInfoBuffer.getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalContactCountsd = mVolumeContactOrVTContactCountBuffer.getTypedDevicePtr();

		CUdeviceptr constraintsd = mFemConstraintBuf.getDevicePtr();
		PxgDevicePointer<float4> softbodyAppliedForced = mFemAppliedForcesBuf.getTypedDevicePtr();

		const CUfunction softbodyContactPrepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SS_CONTACTPREPARE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(contactsd),
			PX_CUDA_KERNEL_PARAM(normalpensd),
			PX_CUDA_KERNEL_PARAM(barycentric0d),
			PX_CUDA_KERNEL_PARAM(barycentric1d),
			PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(totalContactCountsd),
			PX_CUDA_KERNEL_PARAM(constraintsd),
			PX_CUDA_KERNEL_PARAM(softbodyAppliedForced),
			PX_CUDA_KERNEL_PARAM(mMaxContacts)
		};

		CUresult result = mCudaContext->launchKernel(softbodyContactPrepKernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1, PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_solveTetrahedron first pass kernel fail!\n");

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalContactCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			int bob = 0;
			PX_UNUSED(bob);
		}

#endif
	}


	void PxgSoftBodyCore::constraintPrep(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
		const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream, const bool isTGS, PxU32 numSolverBodies, PxU32 numArticulations)
	{
		PX_UNUSED(invDt);
		PX_UNUSED(prePrepDescd);
		PX_UNUSED(prepDescd);

		PxgBodySimManager& bodySimManager = mSimController->getBodySimManager();

		const PxU32 nbActiveSoftbodies = bodySimManager.mActiveSoftbodies.size();

		if (nbActiveSoftbodies == 0)
			return;

		const PxU32 nbActiveParticleSystems = bodySimManager.mActivePBDParticleSystems.size();

		if (nbActiveParticleSystems != 0 )
			prepSoftBodyParticleConstraint();

		const PxU32 nbActiveClothes = bodySimManager.mActiveFEMCloths.size();

		if (nbActiveClothes != 0)
			prepSoftBodyClothConstraint();

		//This run in soft body stream
		prepSoftbodyContactConstraint();

		//Wait for sorting to have completed on mStream before primitivePrep can run
		synchronizeStreams(mCudaContext, solverStream, mStream);
		//Wait for DMA of prePrepDescd and prepDescd before rigid body vs soft body constraint prep can run
		synchronizeStreams(mCudaContext, mStream, solverStream);
		
		prepRigidContactConstraint(prePrepDescd, prepDescd, invDt, sharedDescd, solverStream, isTGS, numSolverBodies, numArticulations);

		prepRigidAttachmentConstraints(prePrepDescd, prepDescd, invDt, sharedDescd, solverStream, isTGS);

		prepSoftBodyAttachmentConstraints(solverStream);

		prepClothAttachmentConstraints(solverStream);

		prepParticleAttachmentConstraints(solverStream);

		synchronizeStreams(mCudaContext, solverStream, mStream);
	}

	void PxgSoftBodyCore::step(const PxReal dt, CUstream stream, const PxU32 nbActiveSoftbodies, const PxVec3& gravity)
	{
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.step", 0);
#endif
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgCudaBuffer& softBodiesBuffer = core->getSoftBodyBuffer();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(softBodiesBuffer.getDevicePtr());

		PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

		const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled() && mGpuContext->isTGS();

		//KS - if TGS, we do not pre-integrate here. Instead, we handle all integration inside the solver stepping scheme
		const PxU32 maxTetraVerts = core->getGMMaxTetraVerts();

		{
			const CUfunction GMPreIntegrateKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_GM_STEPSOFTBODY);

			const PxU32 numBlocks = (maxTetraVerts + PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION - 1) / PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION;

			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftBodiesd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(gravity),
					PX_CUDA_KERNEL_PARAM(externalForcesEveryTgsIterationEnabled)
				};

				CUresult result = mCudaContext->launchKernel(GMPreIntegrateKernelFunction, numBlocks, nbActiveSoftbodies, 1,
					PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if SB_GPU_DEBUG
				result = mCudaContext->streamSynchronize(stream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU preIntegrateSystem kernel fail!\n");
#endif
			}
		}
	}

	void PxgSoftBodyCore::solveTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal dt, CUstream solverStream,
		bool isVelocityIteration, const PxReal biasCoefficient, const bool isFirstIteration, const PxVec3& gravity)
	{
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.solveTGS", 0);
#endif
		PX_UNUSED(prepDescd);
		const PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();

		if (nbActiveSoftbodies == 0)
			return;

		//mGpuContext->mGpuArticulationCore->outputVelocity(solverCoreDescd, solerStream, true);

		if (!isVelocityIteration)
		{
			step(dt, mStream, nbActiveSoftbodies, gravity);
		}

		PxgSoftBody* softbodies = mSimController->getSoftBodies();

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();
		
		solveCorotationalFEM(softbodies, softbodiesd, activeSoftBodiesd, nbActiveSoftbodies, dt, mStream, true, isFirstIteration);

		// Interaction with rigid body
		{
			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Solve soft-rigid attachment constraints (runs on solverStream)
			solveRigidAttachmentTGS(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt, biasCoefficient, isVelocityIteration);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Solve soft-rigid collision constraints (runs on solverStream)
			queryRigidContactReferenceCount(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);
			solveRSContactsOutputRigidDeltaTGS(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
		}

		{
			//solve soft body attachment at soft body stream
			solveSoftBodyAttachmentDelta();

			//This function is going to update the pos and vel for the FEM verts
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			//solve soft body vs soft body and soft body selfcollision
			solveSSContactsOutputSoftBodyDelta(dt, biasCoefficient, true);

			//This function is going to update the pos and vel for the FEM verts
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
		}

		{
			const PxU32 nbActiveParticleSystem = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
			PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();

			if (nbActiveParticleSystem > 0 && particleCore)
			{
				CUstream particleStream = particleCore->getStream();
				PxgDevicePointer<PxgParticleSystem> particleSystemd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();
				PxgDevicePointer<PxU32> activeParticleSystemd = particleCore->getActiveParticleSystemBuffer().getTypedDevicePtr();

				synchronizeStreams(mCudaContext, particleStream, mStream);

				//solve particle attachment at soft body stream
				solveParticleAttachmentDelta();

				synchronizeStreams(mCudaContext, mStream, particleStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				particleCore->applyDeltas(particleSystemd, activeParticleSystemd, nbActiveParticleSystem, dt, particleStream);

				//clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, particleStream, mStream);

				//solve soft body vs particle contact in soft body stream
				solveSPContactsOutputSoftBodyDelta(dt, biasCoefficient);

				//solve soft body vs particle contact in particle stream
				solveSPContactsOutputParticleDelta(dt, biasCoefficient, particleStream);
				//solveSPContactsOutputParticleDelta(dt, mStream);

				//soft body stream need to wait till soft body vs particle finish in the particle stream
				mCudaContext->streamWaitEvent(mStream, mSolveSoftBodyParticleEvent);

				//Force particle stream to wait for soft body stream to finish
				synchronizeStreams(mCudaContext, mStream, particleStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
				
			}
		}

		{
			const PxU32 nbActiveClothes = mSimController->getBodySimManager().mActiveFEMCloths.size();

			if (nbActiveClothes > 0)
			{
				PX_UNUSED(nbActiveClothes);

				PxgFEMClothCore* clothCore = mSimController->getFEMClothCore();
				CUstream clothStream = clothCore->getStream();

				synchronizeStreams(mCudaContext, clothStream, mStream);

				//solve cloth attachment at soft body stream
				solveClothAttachmentDelta();

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, clothStream, mStream);

				//solve soft body vs cloth contact in soft body stream
				solveSCContactsOutputDelta();

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, mStream, clothStream);
			}
		}
	}


	void PxgSoftBodyCore::solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal dt, CUstream solverStream,
		const bool isFirstIteration)
	{
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.solve", 0);
#endif
		PX_UNUSED(prepDescd);
		PX_UNUSED(sharedDescd);
		PX_UNUSED(solverCoreDescd);
		PX_UNUSED(prePrepDescd);

		const PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();

		if (nbActiveSoftbodies == 0)
			return;

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

		PxgSoftBody* softbodies = mSimController->getSoftBodies();


		//mGpuContext->mGpuArticulationCore->outputVelocity(solverCoreDescd, solverStream, false);

		solveCorotationalFEM(softbodies, softbodiesd, activeSoftBodiesd, nbActiveSoftbodies, dt, mStream, false, isFirstIteration);

		// Interaction with rigid body
		{
			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Solve soft-rigid attachment constraints (runs on solverStream)
			solveRigidAttachment(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Solve soft-rigid collision constraints (runs on solverStream)
			queryRigidContactReferenceCount(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);
			solveRSContactsOutputRigidDelta(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
		}

		{

			//solve soft body attachment at soft body stream
			solveSoftBodyAttachmentDelta();

			//This function is going to update the pos and vel for the FEM verts
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			//solve soft body vs soft body and soft body selfcollision
			solveSSContactsOutputSoftBodyDelta(dt, 1.f/dt, false);

			//This function is going to update the pos and vel for the FEM verts
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
		}

		{
			const PxU32 nbActiveParticleSystem = mSimController->getBodySimManager().mActivePBDParticleSystems.size();

			PxgPBDParticleSystemCore* particleSystemCore = mSimController->getPBDParticleSystemCore();

			if (nbActiveParticleSystem > 0)
			{
				CUstream particleStream = particleSystemCore->getStream();

				PxgDevicePointer<PxgParticleSystem> particleSystemd = particleSystemCore->getParticleSystemBuffer().getTypedDevicePtr();
				PxgDevicePointer<PxU32> activeParticleSystemd = particleSystemCore->getActiveParticleSystemBuffer().getTypedDevicePtr();


				synchronizeStreams(mCudaContext, particleStream, mStream);

				//solve particle attachment at soft body stream
				solveParticleAttachmentDelta();

				synchronizeStreams(mCudaContext, mStream, particleStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				particleSystemCore->applyDeltas(particleSystemd, activeParticleSystemd, nbActiveParticleSystem, dt, particleStream);

				synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, particleStream, mStream);

				//solve soft body vs particle contact in soft body stream
				solveSPContactsOutputSoftBodyDelta(dt, 0.7f/dt);

				//solve soft body vs particle contact in particle stream
				solveSPContactsOutputParticleDelta(dt, 0.7f/dt, particleStream);
				//solveSPContactsOutputParticleDelta(dt, mStream);

				//soft body stream need to wait till soft body vs particle finish in the particle stream
				mCudaContext->streamWaitEvent(mStream, mSolveSoftBodyParticleEvent);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			}
		}

		synchronizeStreams(mCudaContext, mStream, solverStream);
		
		{
			const PxU32 nbActiveClothes = mSimController->getBodySimManager().mActiveFEMCloths.size();

			if (nbActiveClothes > 0)
			{
				PX_UNUSED(nbActiveClothes);

				PxgFEMClothCore* clothCore = mSimController->getFEMClothCore();
				CUstream clothStream = clothCore->getStream();

				synchronizeStreams(mCudaContext, clothStream, mStream);

				//solve cloth attachment at soft body stream
				solveClothAttachmentDelta();

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				//synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, clothStream, mStream);

				//solve soft body vs cloth contact in soft body stream
				solveSCContactsOutputDelta();

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, mStream, clothStream);

			}
		}

		synchronizeStreams(mCudaContext, mStream, solverStream);

	}

	void PxgSoftBodyCore::calculateStress()
	{
		const PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();
		PxgSimulationCore* core = mSimController->getSimulationCore();
		
		const PxU32 maxTetrahedrons = core->getMaxTetrahedrons();

		if (maxTetrahedrons > 0)
		{
			PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();
			PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr matData = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			//PxgSoftBody* softbodies = mSimController->getSoftBodies();

			const CUfunction calculateTetraStressKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_CALC_STRESS);

			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
			const PxU32 numBlocks = (maxTetrahedrons + numThreadsPerBlock - 1) / numThreadsPerBlock;

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(activeSoftBodiesd),
				PX_CUDA_KERNEL_PARAM(matData)
			};

			CUresult result = mCudaContext->launchKernel(calculateTetraStressKernelFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_calculateStressLaunch kernel fail!\n");

#endif

		}

	}

	void PxgSoftBodyCore::plasticDeformation()
	{
		const PxU32 nbActiveSoftbodies = mSimController->getBodySimManager().mActiveSoftbodies.size();

		if (nbActiveSoftbodies == 0)
			return;

		PxgSimulationCore* core = mSimController->getSimulationCore();

		const PxU32 maxTetrahedrons = core->getGMMaxTetrahedrons();

		if (maxTetrahedrons > 0)
		{
			PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();
			PxgDevicePointer<PxU32> activeSoftBodiesd = core->getActiveSoftBodyBuffer().getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			//PxgSoftBody* softbodies = mSimController->getSoftBodies();

			const CUfunction plasticDeformFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_PLASTIC_DEFORM);
			const CUfunction plasticDeformFunction2 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_PLASTIC_DEFORM2);

			const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
			const PxU32 numBlocks = (maxTetrahedrons + numThreadsPerBlock - 1) / numThreadsPerBlock;

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(activeSoftBodiesd),
				PX_CUDA_KERNEL_PARAM(materials)
			};

			CUresult result = mCudaContext->launchKernel(plasticDeformFunction, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_plasticDeform1Launch kernel fail!\n");

#endif

			result = mCudaContext->launchKernel(plasticDeformFunction2, numBlocks, nbActiveSoftbodies, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if SB_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_plasticDeform2Launch kernel fail!\n");

#endif

		}

	}

	bool PxgSoftBodyCore::updateUserData(PxPinnedArray<PxgSoftBody>& softBodyPool, PxArray<PxU32>& softBodyNodeIndexPool,
		const PxU32* activeSoftBodies, const PxU32 nbActiveSoftBodies, void** bodySimsLL)
	{
		bool anyDirty = false;
		for (PxU32 i = 0; i < nbActiveSoftBodies; ++i)
		{
			PxgSoftBody& softBody = softBodyPool[activeSoftBodies[i]];
			PxU32 nodeIndex = softBodyNodeIndexPool[softBody.mGpuRemapIndex];
			Dy::DeformableVolume* dySoftBody = reinterpret_cast<Dy::DeformableVolume*>(bodySimsLL[nodeIndex]);
			Dy::DeformableVolumeCore& dyDeformableVolumeCore = dySoftBody->getCore();

			if (dyDeformableVolumeCore.dirty)
			{
				softBody.mLinearDamping = dyDeformableVolumeCore.linearDamping;
				softBody.mMaxLinearVelocity = dyDeformableVolumeCore.maxLinearVelocity;
				softBody.mPenBiasClamp = dyDeformableVolumeCore.maxPenetrationBias;

				softBody.mSettlingThreshold = dyDeformableVolumeCore.settlingThreshold;
				softBody.mSleepThreshold = dyDeformableVolumeCore.sleepThreshold;
				softBody.mSettlingDamping = dyDeformableVolumeCore.settlingDamping;
				softBody.mSelfCollisionFilterDistance = dyDeformableVolumeCore.selfCollisionFilterDistance;
				softBody.mSelfCollisionStressTolerance = dyDeformableVolumeCore.selfCollisionStressTolerance;

				softBody.mActorFlags = dyDeformableVolumeCore.actorFlags;
				softBody.mBodyFlags = dyDeformableVolumeCore.bodyFlags;
				softBody.mVolumeFlags = dyDeformableVolumeCore.volumeFlags;
				softBody.mSimKinematicTarget = reinterpret_cast<const float4*>(dyDeformableVolumeCore.kinematicTarget);

				anyDirty = true;
				dyDeformableVolumeCore.dirty = false;
			}

			// This would be a place to do actual things with the dirty flags.
			if (dyDeformableVolumeCore.dirtyFlags)
				dyDeformableVolumeCore.dirtyFlags = PxDeformableVolumeDataFlags(0);
		}

		return anyDirty;
	}
	
	void PxgSoftBodyCore::copyOrApplySoftBodyDataDEPRECATED(PxU32 dataIndex, PxU32* softBodyIndices, PxU8** data, PxU32* dataSizes, PxU32 maxSizeInBytes, const PxU32 nbSoftbodies, const PxU32 applyDataToSoftBodies)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_COPY_OR_APPLY_SOFTBODY_DATA_DEPRECATED);

		PxgSimulationCore* core = mSimController->getSimulationCore();

		PxU32 maxElementsToCopyPerThread = 4;
		PxU32 maxSize = (maxSizeInBytes + 4 - 1) / 4; //We copy 4 bytes at a time

		PxU32 threadsPerSoftBody = (maxSize + maxElementsToCopyPerThread - 1) / maxElementsToCopyPerThread;
		PxU32 totalNumberOfThreads = nbSoftbodies * threadsPerSoftBody;
		PxU32 threadsPerBlock = 128;
		PxU32 numBlocks = (totalNumberOfThreads + threadsPerBlock - 1) / threadsPerBlock;

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(core->getSoftBodyBuffer()),
			PX_CUDA_KERNEL_PARAM(dataIndex),
			PX_CUDA_KERNEL_PARAM(softBodyIndices),
			PX_CUDA_KERNEL_PARAM(nbSoftbodies),
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(dataSizes),
			PX_CUDA_KERNEL_PARAM(threadsPerSoftBody),
			PX_CUDA_KERNEL_PARAM(applyDataToSoftBodies)
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, threadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU softbody direct API kernel fail!\n");

#endif
	}	

	void PxgSoftBodyCore::copySoftBodyDataDEPRECATED(void** data, void* dataEndIndices, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, CUevent copyEvent)
	{
		PxU32 dataIndex = PxgSoftBody::dataIndexFromFlagDEPRECATED(flag);

		copyOrApplySoftBodyDataDEPRECATED(dataIndex, reinterpret_cast<PxU32*>(softBodyIndices), reinterpret_cast<PxU8**>(data),
			reinterpret_cast<PxU32*>(dataEndIndices), maxSize, nbCopySoftBodies, 0);

		if (copyEvent)
		{
			mCudaContext->eventRecord(copyEvent, mStream);
		}
		else
		{
			CUresult result = mCudaContext->streamSynchronize(mStream);
			PX_UNUSED(result);
		}
	}

	void PxgSoftBodyCore::applySoftBodyDataDEPRECATED(void** data, void* dataEndIndices, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, CUevent applyEvent, CUevent signalEvent)
	{
		if (applyEvent)
			mCudaContext->streamWaitEvent(mStream, applyEvent);

		PxU32 dataIndex = PxgSoftBody::dataIndexFromFlagDEPRECATED(flag);

		copyOrApplySoftBodyDataDEPRECATED(dataIndex, reinterpret_cast<PxU32*>(softBodyIndices), reinterpret_cast<PxU8**>(data),
			reinterpret_cast<PxU32*>(dataEndIndices), maxSize, nbUpdatedSoftBodies, 1);

		if (signalEvent)
			mCudaContext->eventRecord(signalEvent, mStream);
		else
		{
			CUresult result = mCudaContext->streamSynchronize(mStream);
			PX_UNUSED(result);
		}

	}

}

//#pragma optimize("", on)
