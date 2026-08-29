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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
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
#include "PxgIslandContext.h"
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
		PxgAllocatorDesc& allocDesc, PxgSimulationController* simController, PxgGpuContext* gpuContext, const PxU32 maxContacts, const PxU32 collisionStackSize, const bool isTGS) :
		PxgFEMCore(gpuKernelWrangler, cudaContextManager, allocDesc, simController, gpuContext, maxContacts, collisionStackSize, isTGS, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactPointBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactNormalPenBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactBarycentricBuffer0(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactBarycentricBuffer1(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactInfoBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCTotalContactCountBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mPrevSCContactCountBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSSContactBlocks(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCContactBlocks(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
		mSCLambdaNBuf(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY), 
		mBoundUpdateEvent(NULL),
		mSolveRigidEvent(NULL),
		mConstraintPrepSoftBodyParticleEvent(NULL),
		mSolveSoftBodyParticleEvent(NULL),
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
		mSCLambdaNBuf.allocate(maxContacts * sizeof(float2), PX_FL);
	
		// One block per 32 contacts (SIMD-32 block format); both SS and SC
		// share the maxContacts budget.
		mSSContactBlocks.allocateElements((maxContacts + 31) / 32, PX_FL);
		mSCContactBlocks.allocateElements((maxContacts + 31) / 32, PX_FL);
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
			
			const CUfunction GMPreIntegrateKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SIM_PREINTEGRATION);

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
				const CUfunction GMUpdateCPVertsFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_ZERO_TETMULTIPLIERS);

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

		const CUfunction refitBoundKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_REFITBOUND);

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
		PxU32 contactCountNeeded = PxMax(mContactCountsPrevTimestep[ContactCounts::ePARTICLE],
										 PxMax(mContactCountsPrevTimestep[ContactCounts::eRIGID],
											   mContactCountsPrevTimestep[ContactCounts::eVOLUME_CONTACTOR_VT]));
		
		if (contactCountNeeded >= mMaxContacts) 
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "Deformable volume contact buffer overflow detected, please increase PxGpuDynamicsMemoryConfig::maxDeformableVolumeContacts to at least %u\n", contactCountNeeded);
		}

		if (mStackSizeNeededPinned.get() > mCollisionStackSizeBytes)
		{
			PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow detected, please increase its size to at least %i in the scene desc! Contacts have been dropped.\n", mStackSizeNeededPinned.get());
		}

#if PX_ENABLE_SIM_STATS
		mContactCountStats = PxMax(mContactCountStats, contactCountNeeded);
		mGpuContext->getSimStats().mGpuDynamicsDeformableVolumeContacts = mContactCountStats;

		mCollisionStackSizeBytesStats = PxMax(mStackSizeNeededPinned.get(), mCollisionStackSizeBytesStats);
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
			PxgSoftBodyContactWriter writer = createSoftBodyContactWriter();

			CUfunction scMidphaseFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SELFCOLLISION_MIDPHASE);

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

		CUfunction clampFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::CLAMP_MAX_VALUES);
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

		CUfunction radixFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::RS_MULTIBLOCK);
		CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK);

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
		
			CUfunction copyFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::RS_COPY_HIGH_32BITS);
		
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

			CUfunction copyFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::RS_COPY_VALUE);

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

			CUfunction copyFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::RS_COPY_HIGH_32BITS);

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

			CUfunction reorderFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_REORDER_PS_CONTACTS);

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

			const CUfunction updateGMTRKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_UPDATETETRAROTATIONS);

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

				const CUfunction updateTRKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_UPDATETETRAROTATIONS);

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

				const CUfunction updateTetModelKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_UPDATETETMODELVERTS);

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

			const CUfunction solveTetraKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_APPLY_EXTERNAL_DELTAS);

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

	void PxgSoftBodyCore::finalizeVelocities(const PxReal dt)
	{
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

			const bool alwaysRunVelocityAveraging = mIsTGS && !mGpuContext->isExternalForcesEveryTgsIterationEnabled();
			const PxU32 nbPosIters = mGpuContext->mIslandContextPool[0].mNumPositionIterations;

			{
				const PxU32 numBlocks = (maxVerts + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUfunction finalizeVelocitiesKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_FINALIZE_VELOCITIES);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(activeSoftbodiesd),
					PX_CUDA_KERNEL_PARAM(invDt),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(alwaysRunVelocityAveraging),
					PX_CUDA_KERNEL_PARAM(nbPosIters)
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


			if(!mGpuContext->isSleepingDisabled())
			{
				const PxReal resetCounter = 0.4f;
				const PxU32 numBlocks = (nbActiveSoftbodies + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUfunction softBodySleepingFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SLEEPING);

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
		Cm::PinnableBitMap& sbChangedMap = core->getActiveSBStateChangedMap();

		PxArray<Dy::DeformableVolume*>& deformableVolumes = mSimController->getBodySimManager().mDeformableVolumes;

		PxgBodySimManager& bodyManager = mSimController->getBodySimManager();

		PxReal* wakeCounters = core->getActiveSBWakeCountsCPU();

		mActivatingDeformableVolumes.forceSize_Unsafe(0);
		mDeactivatingDeformableVolumes.forceSize_Unsafe(0);

		Cm::PinnableBitMap::Iterator iter(sbChangedMap);

		PxU32 dirtyIdx;
		while ((dirtyIdx = iter.getNext()) != Cm::PinnableBitMap::Iterator::DONE)
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
		PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt)
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();
		{
			const CUfunction solveOutputRigidDeltaKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_COLLISION);

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());


			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			PxsMaterialData* rigidBodyMaterials = reinterpret_cast<PxsMaterialData*>(npCore->mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgDbRigidContactBlock> contactBlocksd = mRigidContactBlocks.getTypedDevicePtr();


			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PxgDevicePointer<PxReal> appliedForced = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidRefCount.getDevicePtr();

			float4* solverBodyVelPoold = mGpuContext->getGpuSolverCore()->getSolverBodyVelPoolDevPtr();
			const bool isTGS = false;

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(solverCoreDescd),
				PX_CUDA_KERNEL_PARAM(artiCoreDescd),
				PX_CUDA_KERNEL_PARAM(solverBodyVelPoold),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(femRigidContactCount),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(materials),
				PX_CUDA_KERNEL_PARAM(rigidBodyMaterials),
				PX_CUDA_KERNEL_PARAM(isTGS)
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
		accumulateRigidDeltas(prePrepDescd, solverCoreDescd, artiCoreDescd, mRigidSortedRigidIdBuf.getDevicePtr(),
							  mRigidTotalContactCountBuf.getDevicePtr(), solverStream, false);

		// if the contact is between articulation and soft body, after accumulated all the related contact's
		// impulse, we need to propagate the accumulated impulse to the articulation block solver
		mGpuContext->mGpuArticulationCore->pushImpulse(solverStream);
	}

	void PxgSoftBodyCore::solveRSContactsOutputRigidDeltaTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
		PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt)
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.solveRigidContactOutputRigid", 0);
#endif
		{
			const CUfunction solveOutputRigidDeltaKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_COLLISION);

			PxgSimulationCore* core = mSimController->getSimulationCore();
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			PxsMaterialData* rigidBodyMaterials = reinterpret_cast<PxsMaterialData*>(npCore->mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
			PxgDevicePointer<PxgDbRigidContactBlock> contactBlocksd = mRigidContactBlocks.getTypedDevicePtr();

			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PxgDevicePointer<PxReal> appliedForced = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();
			PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidRefCount.getDevicePtr();

			float4* solverBodyVelPoold = mGpuContext->getGpuSolverCore()->getSolverBodyVelPoolDevPtr();
			const bool isTGS = true;

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(solverCoreDescd),
				PX_CUDA_KERNEL_PARAM(artiCoreDescd),
				PX_CUDA_KERNEL_PARAM(solverBodyVelPoold),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(femRigidContactCount),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(materials),
				PX_CUDA_KERNEL_PARAM(rigidBodyMaterials),
				PX_CUDA_KERNEL_PARAM(isTGS)
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
		accumulateRigidDeltas(prePrepDescd, solverCoreDescd, artiCoreDescd, mRigidSortedRigidIdBuf.getDevicePtr(),
							  mRigidTotalContactCountBuf.getDevicePtr(), solverStream, true);

		mGpuContext->mGpuArticulationCore->pushImpulse(solverStream);
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
						const CUfunction GMCPSolveTetraKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_CP_SOLVE_TETRA);

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
					        mGpuKernelWranglerManager->getCuFunction(
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
					        mGpuKernelWranglerManager->getCuFunction(
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

				const CUfunction GMCPAverageKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_CP_AVERAGEVERTS);

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

	void PxgSoftBodyCore::querySPContactReferenceCount(const PxReal dt)
	{
		// SB-particle pre-count pass. Bumps softbody.mSimDelta[v].w per touched
		// tet vertex for each active contact. Runs on mStream, which also owns
		// the SP softbody-side solve.
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

		PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();
		PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalSPContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgDbParticleContactBlock> contactBlocksd = mParticleContactBlocks.getTypedDevicePtr();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_SP_REFERENCE_COUNT);

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(particlesystemsd),
			PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(contactBlocksd),
			PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
			PX_CUDA_KERNEL_PARAM(dt)
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
			PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams,
			sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_querySBParticleContactReferenceCountLaunch kernel fail!\n");
#endif
	}


	void PxgSoftBodyCore::solveSPContactsOutputSoftBodyDelta(const PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgPBDParticleSystemCore* particleCore = mSimController->getPBDParticleSystemCore();

		PxgDevicePointer<PxgParticleSystem> particlesystemsd = particleCore->getParticleSystemBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalSPContactCountsd = mParticleTotalContactCountBuffer.getTypedDevicePtr();

		//solve soft body vs particle contact in the soft body stream and update delta and applied force for soft body
		{

			const CUfunction solveOutputSoftBodyDeltaKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_PARTICLE_SOFT_DELTA);

			//PxgSimulationCore* core = mSimController->getSimulationCore();
			//PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mParticleSortedContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgDbParticleContactBlock> contactBlocksd = mParticleContactBlocks.getTypedDevicePtr();

			PxgDevicePointer<float2> appliedForced = mParticleAppliedFEMForcesBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(particlesystemsd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(dt),
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
	void PxgSoftBodyCore::solveSPContactsOutputParticleDelta(const PxReal dt, CUstream particleStream)
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

			const CUfunction solveOutputParticleDeltaKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_PARTICLE_PARTICLE_DELTA);

			PxgDevicePointer<PxgDbParticleContactBlock> contactBlocksd = mParticleContactBlocks.getTypedDevicePtr();

			PxgDevicePointer<float2> appliedForced = mParticleAppliedParticleForcesBuf.getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(particlesystemsd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(totalSPContactCountsd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(dt),
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
				const CUfunction accumulatedDeltaVKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::PS_ACCUMULATE_FEM_PARTICLE_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particlesystemsd),
					PX_CUDA_KERNEL_PARAM(contactInfosd),
					PX_CUDA_KERNEL_PARAM(pairCountd),
					PX_CUDA_KERNEL_PARAM(startd),
					PX_CUDA_KERNEL_PARAM(endd),
					PX_CUDA_KERNEL_PARAM(deltaVd)
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

	
	void PxgSoftBodyCore::querySSContactReferenceCount(const PxReal dt)
	{
		// SB-SB pre-count pass. Bumps softbody{0,1}.mSimDelta[v].w per touched
		// tet vertex for each active contact. Single kernel serves both PGS
		// and TGS; buffers shared with solveSSContactsOutputSoftBodyDelta.
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

		PxgDevicePointer<PxU32> totalSSContactCountsd = mVolumeContactOrVTContactCountBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mVolumeContactOrVTContactInfoBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgDbDbContactBlock> contactBlocksd = mSSContactBlocks.getTypedDevicePtr();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_SS_REFERENCE_COUNT);

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(contactBlocksd),
			PX_CUDA_KERNEL_PARAM(totalSSContactCountsd),
			PX_CUDA_KERNEL_PARAM(dt)
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
			PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams,
			sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_querySSContactReferenceCountLaunch kernel fail!\n");
#endif
	}


	void PxgSoftBodyCore::solveSSContactsOutputSoftBodyDelta(const PxReal dt, const bool isTGS)
	{
		PX_UNUSED(isTGS); // Single SS solve kernel serves both PGS and TGS dispatches.

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());

		PxgDevicePointer<PxU32> totalSSContactCountsd = mVolumeContactOrVTContactCountBuffer.getTypedDevicePtr();

		//solve self collision contacts or soft body vs soft body contacts
		{
			const CUfunction solveOutputSoftBodyDeltaKernelFunction =
				mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_SOFT_SOFT_BOTH_DELTA);

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr materials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mVolumeContactOrVTContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgDbDbContactBlock> contactBlocksd = mSSContactBlocks.getTypedDevicePtr();

			PxgDevicePointer<float2> appliedForced = mFemAppliedForcesBuf.getTypedDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(totalSSContactCountsd),
				PX_CUDA_KERNEL_PARAM(dt),
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


	void PxgSoftBodyCore::querySCContactReferenceCount(const PxReal dt)
	{
		// SB-cloth pre-count pass. Bumps cloth.mDeltaPos[v].w and
		// softbody.mSimDelta[v].w per touched vertex for each active contact.
		// Runs on mStream, which also owns the SC solve.
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());

		PxgDevicePointer<PxU32> totalSCContactCountsd = mSCTotalContactCountBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mSCContactInfoBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgDbDbContactBlock> contactBlocksd = mSCContactBlocks.getTypedDevicePtr();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_SC_REFERENCE_COUNT);

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(clothesd),
			PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(contactBlocksd),
			PX_CUDA_KERNEL_PARAM(totalSCContactCountsd),
			PX_CUDA_KERNEL_PARAM(dt)
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSoftBodyKernelGridDim::SB_UPDATEROTATION, 1, 1,
			PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION, 1, 1, 0, mStream, kernelParams,
			sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_querySCContactReferenceCountLaunch kernel fail!\n");
#endif
	}


	void PxgSoftBodyCore::solveSCContactsOutputDelta(const PxReal dt)
	{
		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(core->getSoftBodyBuffer().getDevicePtr());
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(core->getFEMClothBuffer().getDevicePtr());

		PxgDevicePointer<PxU32> totalSCContactCountsd = mSCTotalContactCountBuffer.getTypedDevicePtr();

		//solve soft body vs cloth contacts
		{
			const CUfunction solveOutputSoftBodyDeltaKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_SOFT_CLOTH_BOTH_DELTA);
			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mSCContactInfoBuffer.getTypedDevicePtr();
			PxgDevicePointer<PxgDbDbContactBlock> contactBlocksd = mSCContactBlocks.getTypedDevicePtr();

			PxgDevicePointer<float2> appliedForces = mSCLambdaNBuf.getTypedDevicePtr();

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr sbMaterials = npCore->mGpuFEMMaterialManager.mGpuMaterialBuffer.getDevicePtr();
			CUdeviceptr clothMaterials = npCore->mGpuFEMClothMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(clothesd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(totalSCContactCountsd),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(appliedForces),
				PX_CUDA_KERNEL_PARAM(sbMaterials),
				PX_CUDA_KERNEL_PARAM(clothMaterials)
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
		PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt)
	{
		PxgDevicePointer<PxU32> femRigidContactCount = mFemRigidRefCount.getDevicePtr();
		mCudaContext->memsetD32Async(femRigidContactCount.mPtr, 0, mFemRigidRefCount.getNbElements(), solverStream);

		PxgSimulationCore* core = mSimController->getSimulationCore();
		PxgDevicePointer<PxgSoftBody> softbodiesd = core->getSoftBodyBuffer().getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_RIGID_SOFT_REFERENCE_COUNT);

		PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();
		PxgDevicePointer<PxgDbRigidContactBlock> contactBlocksd = mRigidContactBlocks.getTypedDevicePtr();

		PxgDevicePointer<PxReal> lambdaNs = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();

		float4* solverBodyVelPoold = mGpuContext->getGpuSolverCore()->getSolverBodyVelPoolDevPtr();
		const bool isTGS = mIsTGS;

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(softbodiesd),
											 PX_CUDA_KERNEL_PARAM(contactInfosd),
											 PX_CUDA_KERNEL_PARAM(contactBlocksd),
											 PX_CUDA_KERNEL_PARAM(totalContactCountsd),
											 PX_CUDA_KERNEL_PARAM(prePrepDescd),
											 PX_CUDA_KERNEL_PARAM(solverCoreDescd),
											 PX_CUDA_KERNEL_PARAM(artiCoreDescd),
											 PX_CUDA_KERNEL_PARAM(solverBodyVelPoold),
											 PX_CUDA_KERNEL_PARAM(dt),
											 PX_CUDA_KERNEL_PARAM(lambdaNs),
											 PX_CUDA_KERNEL_PARAM(femRigidContactCount),
											 PX_CUDA_KERNEL_PARAM(isTGS) };

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

	// Pre-count per-vertex refCount for rigid-soft attachments, bumping
	// softbody.mSimDelta[v].w. The solve reads .w as the Jacobi mass-splitting
	// factor (PGS + TGS). Must run after the FEM finalize (so .w starts from 0)
	// and before both attach + contact solves (which read the combined .w).
	void PxgSoftBodyCore::queryRigidAttachmentReferenceCount(CUstream solverStream)
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		const PxU32 nbRigidAttachments = simCore->getNbRigidSoftBodyAttachments();
		if(nbRigidAttachments == 0)
			return;

		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
		PxgDevicePointer<PxgDbRigidAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyRigidAttachmentBlocks();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_RIGID_SOFT_ATTACHMENT_REFERENCE_COUNT);

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
			PX_CUDA_KERNEL_PARAM(nbRigidAttachments)
		};

		const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
		const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0,
			solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(solverStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_queryRigidSoftAttachmentReferenceCountLaunch kernel fail!\n");
#endif
	}

	void PxgSoftBodyCore::solveRigidAttachment(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt, const PxReal biasCoefficient)
	{
		
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbRigidSoftBodyAttachments();

		if(nbRigidAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
			
			PxgDevicePointer<PxgDbRigidAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyRigidAttachmentBlocks();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			
			{
				const CUfunction solvePCRigidKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_ATTACHMENT);

				const bool isVelocityIteration = false;
				const bool isTGS = false;
				float4* solverBodyVelPoold = mGpuContext->getGpuSolverCore()->getSolverBodyVelPoolDevPtr();

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(solverBodyVelPoold),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(biasCoefficient),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(isVelocityIteration),
					PX_CUDA_KERNEL_PARAM(isTGS)
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
			accumulateRigidDeltas(prePrepDescd, solverCoreDescd, artiCoreDescd, rigidAttachmentIds, totalRigidAttachmentsd,
								  solverStream, false);

			// Flush mScratchImpulse so a same-iter contact-path pushImpulse doesn't read stale link velocity.
			mGpuContext->mGpuArticulationCore->pushImpulse(solverStream);
		}
	}

	void PxgSoftBodyCore::solveRigidAttachmentTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt, const PxReal biasCoefficient,
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

			PxgDevicePointer<PxgDbRigidAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyRigidAttachmentBlocks();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();


			{
				const CUfunction solvePCRigidKernelFunction =
					mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_RIGID_SOFT_ATTACHMENT);

				const bool isTGS = true;
				float4* solverBodyVelPoold = mGpuContext->getGpuSolverCore()->getSolverBodyVelPoolDevPtr();

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(solverBodyVelPoold),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(biasCoefficient),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(isVelocityIteration),
					PX_CUDA_KERNEL_PARAM(isTGS)
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

			accumulateRigidDeltas(prePrepDescd, solverCoreDescd, artiCoreDescd, rigidAttachmentIds, totalRigidAttachmentsd,
								  solverStream, true);

			// Flush mScratchImpulse so a same-iter contact-path pushImpulse doesn't read stale link velocity.
			mGpuContext->mGpuArticulationCore->pushImpulse(solverStream);
		}
	}

	// Pre-count pass for DB-DB softbody-softbody attachments: bumps
	// softbody.mSimDelta[v].w via atomicAdd. Pairs with
	// sb_solveOutputSoftBodyAttachmentDeltaVLaunch which reads .w as the
	// per-vertex Jacobi mass-splitting factor. Must run after the FEM
	// finalize (so .w starts at 0) and before the attach solve.
	void PxgSoftBodyCore::querySoftBodyAttachmentReferenceCount()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		const PxU32 nbSoftBodyAttachments = simCore->getNbSoftBodySoftBodyAttachments();
		if(nbSoftBodyAttachments == 0)
			return;

		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
		PxgDevicePointer<PxgDbDbAttachmentBlock> attachmentBlocksd = simCore->getSoftBodySoftBodyAttachmentBlocks();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_SOFTBODY_ATTACHMENT_REFERENCE_COUNT);

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
			PX_CUDA_KERNEL_PARAM(nbSoftBodyAttachments)
		};

		const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
		const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0,
			mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_querySoftBodyAttachmentReferenceCountLaunch kernel fail!\n");
#endif
	}

	// Pre-count pass for DB-DB cloth-softbody attachments: bumps
	// cloth.mDeltaPos[v].w + softbody.mSimDelta[v].w via atomicAdd. Both
	// buffers must be zero on entry.
	void PxgSoftBodyCore::queryClothAttachmentReferenceCount()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		const PxU32 nbClothAttachments = simCore->getNbClothSoftBodyAttachments();
		if(nbClothAttachments == 0)
			return;

		PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
		PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(simCore->getFEMClothBuffer().getDevicePtr());
		PxgDevicePointer<PxgDbDbAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyClothAttachmentBlocks();

		const CUfunction kernelFunction =
			mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_QUERY_CLOTH_ATTACHMENT_REFERENCE_COUNT);

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(clothesd),
			PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
			PX_CUDA_KERNEL_PARAM(nbClothAttachments)
		};

		const PxU32 numThreadsPerBlock = PxgSoftBodyKernelBlockDim::SB_UPDATEROTATION;
		const PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_UPDATEROTATION;
		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0,
			mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if SB_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_queryClothAttachmentReferenceCountLaunch kernel fail!\n");
#endif
	}

	void PxgSoftBodyCore::solveSoftBodyAttachmentDelta()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbSoftBodyAttachments = simCore->getNbSoftBodySoftBodyAttachments();

		if (nbSoftBodyAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());

			PxgDevicePointer<PxgDbDbAttachmentBlock> attachmentBlocksd = simCore->getSoftBodySoftBodyAttachmentBlocks();

			{
				const CUfunction solvePCRigidKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_SOFTBODY_ATTACHMENT_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
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

	void PxgSoftBodyCore::solveClothAttachmentDelta()
	{

		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbClothAttachments = simCore->getNbClothSoftBodyAttachments();

		if (nbClothAttachments)
		{
			PxgSoftBody* softbodiesd = reinterpret_cast<PxgSoftBody*>(simCore->getSoftBodyBuffer().getDevicePtr());
			PxgFEMCloth* clothesd = reinterpret_cast<PxgFEMCloth*>(simCore->getFEMClothBuffer().getDevicePtr());

			PxgDevicePointer<PxgDbDbAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyClothAttachmentBlocks();

			{
				const CUfunction solvePCRigidKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SOLVE_CLOTH_ATTACHMENT_DELTA);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(clothesd),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
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

	void PxgSoftBodyCore::prepRigidAttachmentBlocks(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
		const PxReal /*invDt*/, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream stream, bool /*isTGS*/) 
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbRigidAttachments = simCore->getNbRigidSoftBodyAttachments();

		if (nbRigidAttachments)
		{

			PxgDevicePointer<PxgSoftBody> softbodiesd = simCore->getSoftBodyBuffer().getTypedDevicePtr();

			PxgDevicePointer<PxgFEMRigidAttachment> rigidAttachments = simCore->getRigidSoftBodyAttachments();
			PxgDevicePointer<PxU32> activeRigidAttachments = simCore->getActiveRigidSoftBodyAttachments();
			PxgDevicePointer<PxgDbRigidAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyRigidAttachmentBlocks();
			PxgDevicePointer<PxNodeIndex> rigidAttachmentIds = simCore->getSoftBodyRigidAttachmentIds();
			PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
			PX_ASSERT(simCore->getNbRigidSoftBodyAttachments() * 2 <= mRigidDeltaVelBuf.getNbElements());

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_RIGID_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(rigidAttachments),
					PX_CUDA_KERNEL_PARAM(activeRigidAttachments),
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd),
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

	void PxgSoftBodyCore::prepSoftBodyAttachmentBlocks(CUstream stream)
	{

		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbSoftBodyAttachments = simCore->getNbSoftBodySoftBodyAttachments();

		if (nbSoftBodyAttachments)
		{

			PxgDevicePointer<PxgFEMFEMAttachment> softBodyAttachments = simCore->getSoftBodySoftBodyAttachments();
			PxgDevicePointer<PxU32> activeSoftBodyAttachments = simCore->getActiveSoftBodySoftAttachments();
			PxgDevicePointer<PxgDbDbAttachmentBlock> attachmentBlocksd = simCore->getSoftBodySoftBodyAttachmentBlocks();

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::FEM_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{

					PX_CUDA_KERNEL_PARAM(softBodyAttachments),
					PX_CUDA_KERNEL_PARAM(activeSoftBodyAttachments),
					PX_CUDA_KERNEL_PARAM(nbSoftBodyAttachments),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd)
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

	void PxgSoftBodyCore::prepClothAttachmentBlocks(CUstream stream)
	{

		PxgSimulationCore* simCore = mSimController->getSimulationCore();

		const PxU32 nbClothAttachments = simCore->getNbClothSoftBodyAttachments();

		if (nbClothAttachments)
		{

			PxgDevicePointer<PxgFEMFEMAttachment> clothAttachments = simCore->getClothSoftBodyAttachments();
			PxgDevicePointer<PxU32> activeClothAttachments = simCore->getActiveClothSoftBodyAttachments();
			PxgDevicePointer<PxgDbDbAttachmentBlock> attachmentBlocksd = simCore->getSoftBodyClothAttachmentBlocks();
			//PxgDevicePointer<PxU32> clothAttachmentIds = simCore->getSoftBodyClothAttachmentIds().getTypedDevicePtr();

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::FEM_ATTACHMENT_CONSTRAINT_PREP);

				PxCudaKernelParam kernelParams[] =
				{
		
					PX_CUDA_KERNEL_PARAM(clothAttachments),
					PX_CUDA_KERNEL_PARAM(activeClothAttachments),
					/*PX_CUDA_KERNEL_PARAM(clothAttachmentIds),*/
					PX_CUDA_KERNEL_PARAM(nbClothAttachments),
					PX_CUDA_KERNEL_PARAM(attachmentBlocksd)
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

	void PxgSoftBodyCore::prepRigidContactBlocks(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
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

			PxgDevicePointer<PxgDbRigidContactBlock> contactBlocksd = mRigidContactBlocks.getTypedDevicePtr();
			PxgDevicePointer<PxReal> rigidAppliedForced = mRigidFEMAppliedForcesBuf.getTypedDevicePtr();

			// Per-link buckets for articulations (not per-articulation).
			// Layout: [0..numSolverBodies) rigid bodies, then numArticulations * maxLinks
			// slots so that contacts on different links of the same articulation
			// don't share a reference count and double-count against the per-link
			// averaging in accumulateRigidDeltas.
			const PxU32 maxLinksPerArti = mSimController->getSimulationCore()->getMaxArticulationLinks();
			const PxU32 femRefCountElems = numSolverBodies + numArticulations * maxLinksPerArti;
			if (mFemRigidRefCount.getNbElements() != femRefCountElems)
			{
				mFemRigidRefCount.allocateElements(femRefCountElems, PX_FL);
			}

			const CUfunction rigidContactPrepKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_RS_CONTACTPREPARE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(normalpensd),
				PX_CUDA_KERNEL_PARAM(barycentricsd),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
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
	void PxgSoftBodyCore::prepSoftBodyParticleBlocks()
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

				PxgDevicePointer<PxgDbParticleContactBlock> contactBlocksd = mParticleContactBlocks.getTypedDevicePtr();
				PxgDevicePointer<float2> softbodyAppliedForced = mParticleAppliedFEMForcesBuf.getTypedDevicePtr();
				PxgDevicePointer<float2> particleAppliedForced = mParticleAppliedParticleForcesBuf.getTypedDevicePtr();

				const CUfunction softbodyContactPrepKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SP_CONTACTPREPARE);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(softbodiesd),
					PX_CUDA_KERNEL_PARAM(particlesystemsd),
					PX_CUDA_KERNEL_PARAM(contactsd),
					PX_CUDA_KERNEL_PARAM(normalpensd),
					PX_CUDA_KERNEL_PARAM(barycentricd),
					PX_CUDA_KERNEL_PARAM(contactInfosd),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd),
					PX_CUDA_KERNEL_PARAM(contactBlocksd),
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
					const CUfunction findStartEndFirstKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::PS_FIND_RANGESTARTEND_FEM_FIRST);

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
					const CUfunction findStartEndSecondKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::PS_RANGESTARTEND_FEM_SECONE);

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
	void PxgSoftBodyCore::prepSoftBodyClothBlocks()
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

			PxgDevicePointer<PxgDbDbContactBlock> contactBlocksd = mSCContactBlocks.getTypedDevicePtr();

			PxgDevicePointer<float2> appliedForces = mSCLambdaNBuf.getTypedDevicePtr();

			const CUfunction softbodyClothContactPrepKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SC_CONTACTPREPARE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(softbodiesd),
				PX_CUDA_KERNEL_PARAM(clothesd),
				PX_CUDA_KERNEL_PARAM(normalpensd),
				PX_CUDA_KERNEL_PARAM(barycentric0d),
				PX_CUDA_KERNEL_PARAM(barycentric1d),
				PX_CUDA_KERNEL_PARAM(contactInfosd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(contactBlocksd),
				PX_CUDA_KERNEL_PARAM(appliedForces),
				PX_CUDA_KERNEL_PARAM(mMaxContacts)
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
	void PxgSoftBodyCore::prepSoftBodyContactBlocks()
	{
		PxgSimulationCore* simCore = mSimController->getSimulationCore();
		CUdeviceptr softbodiesd = simCore->getSoftBodyBuffer().getDevicePtr();

		PxgDevicePointer<float4> contactsd = mFemContactPointBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> normalpensd = mFemContactNormalPenBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentric0d = mFemContactBarycentric0Buffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentric1d = mFemContactBarycentric1Buffer.getTypedDevicePtr();
		PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = mVolumeContactOrVTContactInfoBuffer.getTypedDevicePtr();

		PxgDevicePointer<PxU32> totalContactCountsd = mVolumeContactOrVTContactCountBuffer.getTypedDevicePtr();

		PxgDevicePointer<PxgDbDbContactBlock> contactBlocksd = mSSContactBlocks.getTypedDevicePtr();
		PxgDevicePointer<float2> softbodyAppliedForced = mFemAppliedForcesBuf.getTypedDevicePtr();

		const CUfunction softbodyContactPrepKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_SS_CONTACTPREPARE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(softbodiesd),
			PX_CUDA_KERNEL_PARAM(contactsd),
			PX_CUDA_KERNEL_PARAM(normalpensd),
			PX_CUDA_KERNEL_PARAM(barycentric0d),
			PX_CUDA_KERNEL_PARAM(barycentric1d),
			PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(totalContactCountsd),
			PX_CUDA_KERNEL_PARAM(contactBlocksd),
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
			prepSoftBodyParticleBlocks();

		const PxU32 nbActiveClothes = bodySimManager.mActiveFEMCloths.size();

		if (nbActiveClothes != 0)
			prepSoftBodyClothBlocks();

		//This run in soft body stream
		prepSoftBodyContactBlocks();

		//Wait for sorting to have completed on mStream before primitivePrep can run
		synchronizeStreams(mCudaContext, solverStream, mStream);
		//Wait for DMA of prePrepDescd and prepDescd before rigid body vs soft body constraint prep can run
		synchronizeStreams(mCudaContext, mStream, solverStream);
		
		prepRigidContactBlocks(prePrepDescd, prepDescd, invDt, sharedDescd, solverStream, isTGS, numSolverBodies, numArticulations);

		prepRigidAttachmentBlocks(prePrepDescd, prepDescd, invDt, sharedDescd, solverStream, isTGS);

		prepSoftBodyAttachmentBlocks(solverStream);

		prepClothAttachmentBlocks(solverStream);

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
			const CUfunction GMPreIntegrateKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_GM_STEPSOFTBODY);

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
		PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal dt, CUstream solverStream,
		bool isVelocityIteration, const PxReal attachBiasCoefficient, const bool isFirstIteration, const PxVec3& gravity)
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

		// Interaction with rigid body. Attach + contact share a single merged
		// refcount phase: both pre-counts populate softbody.mSimDelta[v].w,
		// both solves read it, and one finalize closes the phase. The FEM
		// finalize runs first so .w starts at 0 for the pre-counts.
		{
			// Finalize FEM so .w is zero before the pre-counts run.
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Pre-count refCount per vertex. Both attach + contact bump .w on
			// the same buffer; mixed-writer vertices end up with the combined count.
			queryRigidAttachmentReferenceCount(solverStream);
			queryRigidContactReferenceCount(prePrepDescd, solverCoreDescd, artiCoreDescd, solverStream, dt);

			// Solve both: each reads the combined .w, writes inflated .xyz,
			// does not bump .w. Order is irrelevant for correctness.
			solveRigidAttachmentTGS(prePrepDescd, solverCoreDescd, artiCoreDescd, solverStream, dt, attachBiasCoefficient, isVelocityIteration);
			solveRSContactsOutputRigidDeltaTGS(prePrepDescd, solverCoreDescd, artiCoreDescd, solverStream, dt);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			// Single finalize for the merged phase: pos += .xyz / max(.w, 1);
			// zero both .xyz and .w.
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
		}

		{
			// Pre-count DB-DB attach refcount onto softbody.mSimDelta[v].w.
			// The preceding applyExternalTetraDeltaGM zeroed .w, so this is
			// the first writer. solveSoftBodyAttachmentDelta then reads .w
			// as the mass-splitting factor and scatters .xyz only.
			querySoftBodyAttachmentReferenceCount();

			//solve soft body attachment at soft body stream
			solveSoftBodyAttachmentDelta();

			//This function is going to update the pos and vel for the FEM verts
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			// SB-SB pre-count populates softbody.mSimDelta[v].w for mass-splitting; SS solve below consumes it.
			querySSContactReferenceCount(dt);

			//solve soft body vs soft body and soft body selfcollision
			solveSSContactsOutputSoftBodyDelta(dt, true);

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

				synchronizeStreams(mCudaContext, mStream, particleStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				particleCore->applyDeltas(particleSystemd, activeParticleSystemd, nbActiveParticleSystem, dt, particleStream);

				//clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, particleStream, mStream);

				// SP pre-count populates softbody.mSimDelta[v].w for mass-splitting (zeroed by the preceding applyExternalTetraDeltaGM).
				querySPContactReferenceCount(dt);

				// The particle-side SP solve (particleStream) reads the soft-body refCount (mSimDelta.w)
				// the pre-count just wrote on mStream -- order the streams so the read can't race the
				// write. Only manifests with multiple deformable volumes; one volume happens to be safe.
				synchronizeStreams(mCudaContext, mStream, particleStream);

				//solve soft body vs particle contact in soft body stream
				solveSPContactsOutputSoftBodyDelta(dt);

				//solve soft body vs particle contact in particle stream
				solveSPContactsOutputParticleDelta(dt, particleStream);
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

				// Cloth-softbody attach pre-count onto cloth.mDeltaPos[v].w + softbody.mSimDelta[v].w (each zeroed by its prior finalize).
				queryClothAttachmentReferenceCount();

				//solve cloth attachment at soft body stream
				solveClothAttachmentDelta();

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, clothStream, mStream);

				// SB-cloth pre-count populates both .w buffers; SC solve consumes both.
				querySCContactReferenceCount(dt);

				//solve soft body vs cloth contact in soft body stream
				solveSCContactsOutputDelta(dt);

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				synchronizeStreams(mCudaContext, mStream, clothStream);
			}
		}
	}


	void PxgSoftBodyCore::solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
		PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal dt, CUstream solverStream,
		const PxReal attachBiasCoefficient, const bool isFirstIteration)
	{
#if SB_GPU_DEBUG
		PX_PROFILE_ZONE("PxgSoftBodyCore.solve", 0);
#endif
		PX_UNUSED(prepDescd);
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

		// Interaction with rigid body. Attach + contact share a single merged
		// refcount phase: both pre-counts populate softbody.mSimDelta[v].w,
		// both solves read it, and one finalize closes the phase. The FEM
		// finalize runs first so .w starts at 0 for the pre-counts.
		{
			// Finalize FEM so .w is zero before the pre-counts run.
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			synchronizeStreams(mCudaContext, mStream, solverStream);

			// Pre-count refCount per vertex. Both attach + contact bump .w on
			// the same buffer; mixed-writer vertices end up with the combined count.
			queryRigidAttachmentReferenceCount(solverStream);
			queryRigidContactReferenceCount(prePrepDescd, solverCoreDescd, artiCoreDescd, solverStream, dt);

			// Solve both: each reads the combined .w, writes inflated .xyz,
			// does not bump .w.
			solveRigidAttachment(prePrepDescd, solverCoreDescd, artiCoreDescd, solverStream, dt, attachBiasCoefficient);
			solveRSContactsOutputRigidDelta(prePrepDescd, solverCoreDescd, artiCoreDescd, solverStream, dt);

			mCudaContext->streamWaitEvent(mStream, mSolveRigidEvent);

			// Single finalize for the merged phase.
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);
		}

		{
			// SB-SB attach pre-count (PGS path). See the TGS sibling above
			// for rationale.
			querySoftBodyAttachmentReferenceCount();

			//solve soft body attachment at soft body stream
			solveSoftBodyAttachmentDelta();

			//This function is going to update the pos and vel for the FEM verts
			applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

			// SB-SB pre-count populates softbody.mSimDelta[v].w for
			// mass-splitting; SS solve below consumes it.
			querySSContactReferenceCount(dt);

			//solve soft body vs soft body and soft body selfcollision
			solveSSContactsOutputSoftBodyDelta(dt, false);

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

				synchronizeStreams(mCudaContext, mStream, particleStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				particleSystemCore->applyDeltas(particleSystemd, activeParticleSystemd, nbActiveParticleSystem, dt, particleStream);

				synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, particleStream, mStream);

				// SP pre-count populates softbody.mSimDelta[v].w for mass-splitting (zeroed by the preceding applyExternalTetraDeltaGM).
				querySPContactReferenceCount(dt);

				// The particle-side SP solve (particleStream) reads the soft-body refCount (mSimDelta.w)
				// the pre-count just wrote on mStream -- order the streams so the read can't race the
				// write. Only manifests with multiple deformable volumes; one volume happens to be safe.
				synchronizeStreams(mCudaContext, mStream, particleStream);

				//solve soft body vs particle contact in soft body stream
				solveSPContactsOutputSoftBodyDelta(dt);

				//solve soft body vs particle contact in particle stream
				solveSPContactsOutputParticleDelta(dt, particleStream);
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

				// Cloth-softbody attach pre-count onto cloth.mDeltaPos[v].w + softbody.mSimDelta[v].w (each zeroed by its prior finalize).
				queryClothAttachmentReferenceCount();

				//solve cloth attachment at soft body stream
				solveClothAttachmentDelta();

				synchronizeStreams(mCudaContext, mStream, clothStream);

				//This function is going to update the pos and vel for the FEM verts
				applyExternalTetraDeltaGM(nbActiveSoftbodies, dt, mStream);

				clothCore->applyExternalDelta(nbActiveClothes, dt, clothStream);

				//synchronizeStreams(mCudaContext, solverStream, mStream);
				synchronizeStreams(mCudaContext, clothStream, mStream);

				// SB-cloth pre-count populates both .w buffers; SC solve consumes both.
				querySCContactReferenceCount(dt);

				//solve soft body vs cloth contact in soft body stream
				solveSCContactsOutputDelta(dt);

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

			const CUfunction calculateTetraStressKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_CALC_STRESS);

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

			const CUfunction plasticDeformFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_PLASTIC_DEFORM);
			const CUfunction plasticDeformFunction2 = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::SB_PLASTIC_DEFORM2);

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

	bool PxgSoftBodyCore::updateUserData(Cm::PinnableArray<PxgSoftBody>& softBodyPool, PxArray<PxU32>& softBodyNodeIndexPool,
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

	PxgSoftBodyContactWriter PxgSoftBodyCore::createSoftBodyContactWriter()
	{
		PxgSoftBodyContactWriter writer;
		writer.totalContactCount = getVolumeContactOrVTContactCount().getTypedPtr();
		writer.outPoint = getFemContacts().getTypedPtr();
		writer.outNormalPen = getFemNormalPens().getTypedPtr();
		writer.outBarycentric0 = getFemBarycentrics0().getTypedPtr();
		writer.outBarycentric1 = getFemBarycentrics1().getTypedPtr();
		writer.outContactInfo = getVolumeContactOrVTContactInfos().getTypedPtr();
		writer.maxContacts = mMaxContacts;
		return writer;
	}

	PxgSoftBodyContactWriter PxgSoftBodyCore::createClothVsSoftBodyContactWriter(PxU32 clothMaxContacts)
	{
		PxgSoftBodyContactWriter writer;
		writer.totalContactCount = getClothVsSoftBodyContactCount().getTypedPtr();
		writer.outPoint = getClothVsSoftBodyContacts().getTypedPtr();
		writer.outNormalPen = getClothVsSoftBodyNormalPens().getTypedPtr();
		writer.outBarycentric0 = getClothVsSoftBodyBarycentrics0().getTypedPtr();
		writer.outBarycentric1 = getClothVsSoftBodyBarycentrics1().getTypedPtr();
		writer.outContactInfo = getClothVsSoftBodyContactInfos().getTypedPtr();
		writer.maxContacts = PxMin(mMaxContacts, clothMaxContacts);
		return writer;
	}

}

//#pragma optimize("", on)
