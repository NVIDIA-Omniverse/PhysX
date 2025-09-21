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

#include "common/PxProfileZone.h"

#include "DyFeatherstoneArticulationJointData.h"
#include "DyFeatherstoneArticulation.h"
#include "PxgArticulationCore.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgHeapMemAllocator.h"
#include "PxgCudaSolverCore.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgNarrowphaseCore.h"
#include "PxgSimulationController.h"
#include "PxgSimulationCore.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgArticulationCoreKernelIndices.h"
#include "PxgCudaUtils.h"
#include "CudaKernelWrangler.h"
#include "PxSpatialMatrix.h"
#include "PxgContext.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxArticulationTendonData.h"
#include "PxDirectGPUAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "foundation/PxAssert.h"
#include "foundation/PxSimpleTypes.h"

#define ARTI_GPU_DEBUG 0

#define USE_NEW_LAUNCH_FUNCTION 1

#if USE_NEW_LAUNCH_FUNCTION
	#define KERNEL_PARAM_TYPE	void*
	#define CUDA_KERNEL_PARAM	PX_CUDA_KERNEL_PARAM2
	#define EPILOG				kernelParams, 0, PX_FL
#else
	#define KERNEL_PARAM_TYPE	PxCudaKernelParam
	#define CUDA_KERNEL_PARAM	PX_CUDA_KERNEL_PARAM
	#define EPILOG				kernelParams, sizeof(kernelParams), 0, PX_FL
#endif

using namespace physx;

#if ARTI_GPU_DEBUG
static PX_FORCE_INLINE void gpuDebugStreamSync(PxCudaContext* context, CUstream stream, const char* errorMsg)
{
	const CUresult result = context->streamSynchronize(stream);
	if(result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, errorMsg);
}
#else
static PX_FORCE_INLINE void gpuDebugStreamSync(PxCudaContext*, CUstream, const char*)
{
}
#endif

namespace physx
{
	extern "C" void initArticulationKernels1();
	extern "C" void initArticulationKernels2();
	extern "C" void initArticulationKernels3();
	extern "C" void initArticulationKernels4();

	void createPxgArticulation()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXArticulationGpu linkage as Static Library!
		initArticulationKernels1();
		initArticulationKernels2();
		initArticulationKernels3();
		initArticulationKernels4();
#endif
	}

	PxgArticulationCore::PxgArticulationCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, PxgHeapMemoryAllocatorManager* heapMemoryManager) :
		mGpuKernelWranglerManager(gpuKernelWrangler), mCudaContextManager(cudaContextManager), mCudaContext(cudaContextManager->getCudaContext()),
		mArticulationCoreDescd(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mArticulationOutputDescd(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mNbActiveArticulation(0),
		mDeltaVs(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mSlabHasChanges(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mSlabDirtyMasks(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mPathToRootPerPartition(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mDirtyLinksPerPartition(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mImpulseScalePerPartition(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempContactUniqueIndicesBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempConstraintUniqueIndicesBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempContactHeaderBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempConstraintHeaderBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempSelfContactUniqueIndicesBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempSelfConstraintUniqueIndicesBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempSelfContactHeaderBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mTempSelfConstraintHeaderBlockBuffer(heapMemoryManager, PxsHeapStats::eARTICULATION),
		mNeedsKinematicUpdate(false)
#if PX_SUPPORT_OMNI_PVD		
		,mOvdDataBuffer(heapMemoryManager->mMappedMemoryAllocators)
		,mOvdIndexBuffer(heapMemoryManager->mMappedMemoryAllocators)
#endif
	{
		PxScopedCudaLock _lock(*mCudaContextManager);

		//create stream
		mCudaContext->streamCreate(&mStream, CU_STREAM_NON_BLOCKING);

		mCudaContext->eventCreate(&mFinishEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mFlushArticulationDataEvent, CU_EVENT_DISABLE_TIMING);

		mCudaContext->eventCreate(&mComputeUnconstrainedEvent, CU_EVENT_DISABLE_TIMING);

		mArticulationCoreDescd.allocate(sizeof(PxgArticulationCoreDesc), PX_FL);
		mArticulationCoreDesc = PX_PINNED_MEMORY_ALLOC(PxgArticulationCoreDesc, *mCudaContextManager, 1);

		mArticulationOutputDescd.allocate(sizeof(PxgArticulationOutputDesc), PX_FL);
		mArticulationOutputDesc = PX_PINNED_MEMORY_ALLOC(PxgArticulationOutputDesc, *mCudaContextManager, 1);
	}

	PxgArticulationCore::~PxgArticulationCore()
	{
		PxScopedCudaLock lock(*mCudaContextManager);

		PX_PINNED_MEMORY_FREE(*mCudaContextManager, mArticulationCoreDesc);
		PX_PINNED_MEMORY_FREE(*mCudaContextManager, mArticulationOutputDesc);

		//destroy stream
		mCudaContext->streamDestroy(mStream);

		mCudaContext->eventDestroy(mFinishEvent);
		mCudaContext->eventDestroy(mFlushArticulationDataEvent);
		mCudaContext->eventDestroy(mComputeUnconstrainedEvent);
#if PX_SUPPORT_OMNI_PVD		
		mOvdDataBuffer.reset();
		mOvdIndexBuffer.reset();
#endif
	}

	void PxgArticulationCore::allocDeltaVBuffer(const PxU32 nbSlabs, const PxU32 nbPartitions, CUstream stream)
	{
		PxgSimulationController* controller = mGpuContext->getSimulationController();
		//KS - technically, this could (and probably should) be based on the number of active articulations
		//rather than the total number. In many cases, the number of actives should be lower!
		const PxU32 totalArticulations = controller->getBodySimManager().mTotalNumArticulations;

		const PxU32 maxLinks = controller->getSimulationCore()->getMaxArticulationLinks();

		mDeltaVs.allocate(totalArticulations * maxLinks * nbSlabs * sizeof(Cm::UnAlignedSpatialVector), PX_FL);
		mSlabHasChanges.allocate(totalArticulations*nbSlabs* sizeof(uint2), PX_FL);
		
		const PxU32 wordSize = (maxLinks + 63) / 64;
		const PxU32 nbArticulationBatches = (totalArticulations + 31) / 32;
		mPathToRootPerPartition.allocate(nbPartitions * nbArticulationBatches* sizeof(PxgArticulationBitFieldStackData) * wordSize, PX_FL);
		mDirtyLinksPerPartition.allocate(totalArticulations*nbPartitions*sizeof(PxU32), PX_FL);
		mImpulseScalePerPartition.allocate(totalArticulations*nbPartitions*sizeof(PxReal), PX_FL);

		mSlabDirtyMasks.allocate(totalArticulations*nbSlabs*nbPartitions*sizeof(uint4), PX_FL);

		mCudaContext->memsetD32Async(mSlabDirtyMasks.getDevicePtr(), 0xFFFFFFFF, totalArticulations*nbSlabs*nbPartitions * 4, stream);
	}

	void PxgArticulationCore::layoutDeltaVBuffer(const PxU32 nbSlabs, const PxU32 nbPartitions, CUstream stream)
	{
		PxgSimulationController* controller = mGpuContext->getSimulationController();

		//KS - technically, this could (and probably should) be based on the number of active articulations
		//rather than the total number. In many cases, the number of actives should be lower!
		const PxU32 totalArticulations = controller->getBodySimManager().mTotalNumArticulations;

		const PxU32 maxLinks = mGpuContext->getSimulationCore()->getMaxArticulationLinks();

		mCudaContext->memsetD32Async(mDeltaVs.getDevicePtr(), 0, (totalArticulations * maxLinks * nbSlabs * sizeof(Cm::UnAlignedSpatialVector)) / sizeof(PxU32), stream);
		mCudaContext->memsetD32Async(mSlabHasChanges.getDevicePtr(), 0xFFFFFFFF, totalArticulations*nbSlabs * 2, stream);

		//KS - this is a bit sucky. We already DMAd up this buffer, but now we need to do it again to get the updated deltaV impulse buffer.
		//The previous one may have been reallocated
		mArticulationCoreDesc->impulses = reinterpret_cast<Cm::UnAlignedSpatialVector*>(mDeltaVs.getDevicePtr());
		mArticulationCoreDesc->slabHasChanges = reinterpret_cast<uint2*>(mSlabHasChanges.getDevicePtr());
		mArticulationCoreDesc->slabDirtyMasks = reinterpret_cast<uint4*>(mSlabDirtyMasks.getDevicePtr());
		mArticulationCoreDesc->nbSlabs = nbSlabs;
		mArticulationCoreDesc->nbPartitions = nbPartitions;
		
		mArticulationCoreDesc->mPathToRootsPerPartition = reinterpret_cast<PxgArticulationBitFieldStackData*>(mPathToRootPerPartition.getDevicePtr());
		mArticulationCoreDesc->mImpulseHoldingLink = reinterpret_cast<PxU32*>(mDirtyLinksPerPartition.getDevicePtr());
		mArticulationCoreDesc->mPartitionAverageScale = reinterpret_cast<PxReal*>(mImpulseScalePerPartition.getDevicePtr());

		mCudaContext->memcpyHtoDAsync(mArticulationCoreDescd.getDevicePtr(), mArticulationCoreDesc, sizeof(PxgArticulationCoreDesc), stream);
	}

	void PxgArticulationCore::gpuMemDmaUpArticulationDesc(const PxU32 offset, const PxU32 nbArticulations, PxReal dt, const PxVec3& gravity,
		const PxReal invLengthScale, const bool isExternalForcesEveryTgsIterationEnabled)
	{
		CUstream stream = mStream; //*mSolverStream
		//CUstream stream = *mSolverStream;

		PxgSolverCore* solverCore = mGpuContext->getGpuSolverCore();
		PxgSimulationCore* simulationCore = mGpuContext->getSimulationCore();

		//articulation, link and joints
		CUdeviceptr articulationd = simulationCore->getArticulationBuffer().getDevicePtr();
		/*CUdeviceptr	linkd = simulationCore->getLinkBuffer().getDevicePtr();
		CUdeviceptr	jointd = simulationCore->getJointCoreBuffer().getDevicePtr();*/

		//active island node indices
		CUdeviceptr islandNodeIndicesd = CUdeviceptr(solverCore->getGpuIslandNodeIndices());

		// AD: why this hardcoded 32?
		const PxU32 numBlocks = 32;

		mTempContactUniqueIndicesBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempConstraintUniqueIndicesBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempContactHeaderBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempConstraintHeaderBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempSelfContactUniqueIndicesBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempSelfConstraintUniqueIndicesBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempSelfContactHeaderBlockBuffer.allocateElements(numBlocks, PX_FL);
		mTempSelfConstraintHeaderBlockBuffer.allocateElements(numBlocks, PX_FL);

		//bodySim has the index to the link buffer and joint buffer
		mArticulationCoreDesc->mBodySimBufferDeviceData = simulationCore->getBodySimBufferDeviceData().getPointer();
		mArticulationCoreDesc->articulationSleepData = reinterpret_cast<PxgSolverBodySleepData*>(simulationCore->getArticulationSleepDataBuffer().getDevicePtr());
		mArticulationCoreDesc->islandNodeIndices = reinterpret_cast<PxNodeIndex*>(islandNodeIndicesd);
		mArticulationCoreDesc->articulations = reinterpret_cast<PxgArticulation*>(articulationd);

		mArticulationCoreDesc->articulationOffset = offset;
		mArticulationCoreDesc->nbArticulations = nbArticulations;
		mArticulationCoreDesc->dt = dt;
		mArticulationCoreDesc->gravity = gravity;
		mArticulationCoreDesc->invLengthScale = invLengthScale;
		mArticulationCoreDesc->isExternalForcesEveryTgsIterationEnabled = isExternalForcesEveryTgsIterationEnabled;
		mArticulationCoreDesc->impulses = reinterpret_cast<Cm::UnAlignedSpatialVector*>(mDeltaVs.getDevicePtr());
		mArticulationCoreDesc->slabHasChanges = reinterpret_cast<uint2*>(mSlabHasChanges.getDevicePtr());
		//mArticulationCoreDesc->nbSlabs = nbSlabs;
		//mArticulationCoreDesc->nbPartitions = nbPartitions;

		mArticulationCoreDesc->mArticulationBlocks = reinterpret_cast<PxgArticulationBlockData*>(simulationCore->getArticulationBatchData().getDevicePtr());
		mArticulationCoreDesc->mArticulationLinkBlocks = reinterpret_cast<PxgArticulationBlockLinkData*>(simulationCore->getArticulationBatchLinkData().getDevicePtr());
		mArticulationCoreDesc->mArticulationTraversalStackBlocks = reinterpret_cast<PxgArticulationTraversalStackData*>(simulationCore->getArticulationTraversalStackData().getDevicePtr());
		mArticulationCoreDesc->mTempPathToRootBitFieldBlocks = reinterpret_cast<PxgArticulationBitFieldStackData*>(simulationCore->getTempPathToRootBitFieldStackData().getDevicePtr());
		mArticulationCoreDesc->mTempSharedBitFieldBlocks = reinterpret_cast<PxgArticulationBitFieldStackData*>(simulationCore->getTempSharedBitFieldStackData().getDevicePtr());
		mArticulationCoreDesc->mTempRootBitFieldBlocks = reinterpret_cast<PxgArticulationBitFieldStackData*>(simulationCore->getTempRootBitFieldStackData().getDevicePtr());
		mArticulationCoreDesc->mPathToRootBitFieldBlocks = reinterpret_cast<PxgArticulationBitFieldData*>(simulationCore->getPathToRootBitFieldStackData().getDevicePtr());
		mArticulationCoreDesc->mArticulationDofBlocks = reinterpret_cast<PxgArticulationBlockDofData*>(simulationCore->getArticulationBatchDofData().getDevicePtr());
		mArticulationCoreDesc->mArticulationMimicJointBlocks =  reinterpret_cast<PxgArticulationBlockMimicJointData*>(simulationCore->getArticulationBatchMimicJointData().getDevicePtr());
		mArticulationCoreDesc->mArticulationSpatialTendonBlocks = reinterpret_cast<PxgArticulationBlockSpatialTendonData*>(simulationCore->getArticulationBatchSpatialTendonData().getDevicePtr());
		mArticulationCoreDesc->mArticulationSpatialTendonConstraintBlocks = reinterpret_cast<PxgArticulationInternalTendonConstraintData*>(simulationCore->getArticulationBatchSpatialTendonConstraintData().getDevicePtr());
		mArticulationCoreDesc->mArticulationAttachmentBlocks = reinterpret_cast<PxgArticulationBlockAttachmentData*>(simulationCore->getArticulationBatchAttachmentData().getDevicePtr());
		
		mArticulationCoreDesc->mArticulationFixedTendonBlocks = reinterpret_cast<PxgArticulationBlockFixedTendonData*>(simulationCore->getArticulationBatchFixedTendonData().getDevicePtr());
		mArticulationCoreDesc->mArticulationFixedTendonConstraintBlocks = reinterpret_cast<PxgArticulationInternalTendonConstraintData*>(simulationCore->getArticulationBatchFixedTendonConstraintData().getDevicePtr());
		mArticulationCoreDesc->mArticulationTendonJointBlocks = reinterpret_cast<PxgArticulationBlockTendonJointData*>(simulationCore->getArticulationBatchTendonJointData().getDevicePtr());

		mArticulationCoreDesc->mMaxLinksPerArticulation = simulationCore->getMaxArticulationLinks();
		mArticulationCoreDesc->mMaxDofsPerArticulation = simulationCore->getMaxArticulationDofs();
		mArticulationCoreDesc->mMaxMimicJointsPerArticulation = simulationCore->getMaxArticulationMimicJoints();
		mArticulationCoreDesc->mMaxSpatialTendonsPerArticulation = simulationCore->getMaxArticuationSpatialTendons();
		mArticulationCoreDesc->mMaxAttachmentPerArticulation = simulationCore->getMaxArticuationAttachments();
		mArticulationCoreDesc->mMaxFixedTendonsPerArticulation = simulationCore->getMaxArticuationFixedTendons();
		mArticulationCoreDesc->mMaxTendonJointPerArticulation = simulationCore->getMaxArticuationTendonJoints();
		mArticulationCoreDesc->solverBodyIndices = solverCore->getSolverBodyIndices().getPointer();

		mArticulationCoreDesc->mTempContactUniqueIndicesBlock = reinterpret_cast<PxU32*>(mTempContactUniqueIndicesBlockBuffer.getDevicePtr());
		mArticulationCoreDesc->mTempConstraintUniqueIndicesBlock = reinterpret_cast<PxU32*>(mTempConstraintUniqueIndicesBlockBuffer.getDevicePtr());
		mArticulationCoreDesc->mTempContactHeaderBlock = reinterpret_cast<PxU32*>(mTempContactHeaderBlockBuffer.getDevicePtr());
		mArticulationCoreDesc->mTempConstraintHeaderBlock = reinterpret_cast<PxU32*>(mTempConstraintHeaderBlockBuffer.getDevicePtr());
		
		mArticulationCoreDesc->mTempSelfContactUniqueIndicesBlock = reinterpret_cast<PxU32*>(mTempSelfContactUniqueIndicesBlockBuffer.getDevicePtr());
		mArticulationCoreDesc->mTempSelfConstraintUniqueIndicesBlock = reinterpret_cast<PxU32*>(mTempSelfConstraintUniqueIndicesBlockBuffer.getDevicePtr());
		mArticulationCoreDesc->mTempSelfContactHeaderBlock = reinterpret_cast<PxU32*>(mTempSelfContactHeaderBlockBuffer.getDevicePtr());
		mArticulationCoreDesc->mTempSelfConstraintHeaderBlock = reinterpret_cast<PxU32*>(mTempSelfConstraintHeaderBlockBuffer.getDevicePtr());

		//DMA descriptor up
		mCudaContext->memcpyHtoDAsync(mArticulationCoreDescd.getDevicePtr(), mArticulationCoreDesc, sizeof(PxgArticulationCoreDesc), stream);
	}

	void PxgArticulationCore::createStaticContactAndConstraintsBatch(const PxU32 nbArticulations)
	{
		if (nbArticulations)
		{
			const PxU32 numBlocks = 32;

			PxgSolverCore* solverCore = mGpuContext->getGpuSolverCore();
			CUdeviceptr prePrepDescptr = solverCore->getPrePrepDescDeviceptr();
			CUdeviceptr prepDescptr = solverCore->getPrepDescDeviceptr();
			CUdeviceptr solverCoreptr = solverCore->getSolverCoreDescDeviceptr();
			CUdeviceptr artiCoreDescptr = mArticulationCoreDescd.getDevicePtr();

			KernelWrangler* wrangler = mGpuKernelWranglerManager->getKernelWrangler();

			{
				PX_PROFILE_ZONE("GpuDynamics.artiSumInternalContactAndJointBatches1Launch", 0);

				const PxU32 numThreadsPerBlocks = PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;

				const CUfunction kernelFunction = wrangler->getCuFunction(PxgKernelIds::ARTI_STATIC_BATCH_PREP_FIRST);

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(artiCoreDescptr),
					CUDA_KERNEL_PARAM(prePrepDescptr),
					CUDA_KERNEL_PARAM(nbArticulations)
				};

				//In this set up, 32 blocks, each block has 16 warp, each warp has 32 threads  
				const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlocks, 1, 1, 0, *mSolverStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU artiSumInternalContactAndJointBatches1Launch kernel fail!\n");
			}

			{
				PX_PROFILE_ZONE("GpuDynamics.artiSumInternalContactAndJointBatches2Launch", 0);
				const PxU32 numThreadsPerBlocks = PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;
				
				const CUfunction kernelFunction = wrangler->getCuFunction(PxgKernelIds::ARTI_STATIC_BATCH_PREP_SECOND);
				const CUfunction selfKernelFunction = wrangler->getCuFunction(PxgKernelIds::ARTI_SUM_SELF);

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(artiCoreDescptr),
					CUDA_KERNEL_PARAM(solverCoreptr),
					CUDA_KERNEL_PARAM(prePrepDescptr),
					CUDA_KERNEL_PARAM(prepDescptr),
					CUDA_KERNEL_PARAM(nbArticulations),
				};

				//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
				//CUresult result = cuLaunchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, mStream, kernelParams, 0);
				CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlocks, 1, 1, 0, *mSolverStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU artiSumSelfContactAndJointBatches kernel fail!\n");

				result = mCudaContext->launchKernel(selfKernelFunction, numBlocks, 1, 1, numThreadsPerBlocks, 1, 1, 0, *mSolverStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU artiSumSelfContactAndJointBatches kernel fail!\n");
			}
		}
	}

	void PxgArticulationCore::precomputeDependencies(const PxU32 nbPartitions)
	{
		if (nbPartitions > 0)
		{
			// 1 thread per articulation
			const PxU32 numThreadsPerWarp = WARP_SIZE;
			const PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
			const PxU32 num1TBlocks = (mNbActiveArticulation + PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES - 1) / PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES;

			CUstream stream = *mSolverStream;

			CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_COMPUTE_DEPENDENCIES);

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
				CUDA_KERNEL_PARAM(nbPartitions)
			};

			if (num1TBlocks)
			{
				//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
				const CUresult result = mCudaContext->launchKernel(kernelFunction, num1TBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, stream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, stream, "GPU precomputeDependencies kernel fail!\n");
			}
		}
	}

	PxU32 PxgArticulationCore::computeUnconstrainedVelocities(const PxU32 offset, const PxU32 nbArticulations, PxReal dt, const PxVec3& gravity, const PxReal invLengthScale, const bool isExternalForcesEveryTgsIterationEnabled, bool recomputeBlockFormat)
	{
		mNbActiveArticulation = nbArticulations;

		const PxU32 numThreadsPerWarp = WARP_SIZE;
		PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
		PxU32 numBlocks = (nbArticulations + numWarpsPerBlock - 1) / numWarpsPerBlock;

		if (numBlocks)
		{
			PX_PROFILE_ZONE("GpuDynamics.computeUnconstrainedVelocities", 0);

			PxU32 num1TBlocks = (nbArticulations + 63) / 64;

			CUstream stream = mStream; //*mSolverStream
			//CUstream stream = *mSolverStream;

			//layoutDeltaVBuffer(offset, nbArticulations, nbSlabs, /**mSolverStream*/mStream);
			gpuMemDmaUpArticulationDesc(offset, nbArticulations, dt, gravity, invLengthScale, isExternalForcesEveryTgsIterationEnabled);

			KernelWrangler* wrangler = mGpuKernelWranglerManager->getKernelWrangler();

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			{
				const bool directAPI = mGpuContext->getEnableDirectGPUAPI();
				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(descptr),
					CUDA_KERNEL_PARAM(directAPI),
					CUDA_KERNEL_PARAM(recomputeBlockFormat)
				};

				//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
				const CUfunction computeVelocitiesKernelFunction1T = wrangler->getCuFunction(PxgKernelIds::ARTI_COMPUTE_UNCONSTRAINED);
				const CUresult result = mCudaContext->launchKernel(computeVelocitiesKernelFunction1T, num1TBlocks, 1, 1, numThreadsPerWarp, 2, 1, 0, stream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, stream, "GPU computeUnconstrainedVelocities kernel fail!\n");
			}

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
			};

			{
				const CUfunction computeSpatialInertiaPartialKernelFunction1T = wrangler->getCuFunction(PxgKernelIds::ARTI_COMPUTE_SPATIAL_PARTIAL);
				const CUresult result = mCudaContext->launchKernel(computeSpatialInertiaPartialKernelFunction1T, num1TBlocks, 1, 1, numThreadsPerWarp, 2, 1, 0, stream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, stream, "GPU computeUnconstrainedSpatialInertiaKernel fail!\n");
			}
			
			{
				const CUfunction computeSpatialInertiaKernelFunction1T = wrangler->getCuFunction(PxgKernelIds::ARTI_COMPUTE_UNCONSTRAINED_SPATIAL_INERTIA);
				const CUresult result = mCudaContext->launchKernel(computeSpatialInertiaKernelFunction1T, num1TBlocks * 2, 1, 1, numThreadsPerWarp, 1, 1, 0, stream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, stream, "GPU computeUnconstrainedSpatialInertiaKernel fail!\n");
			}

			{
				const CUfunction computeMassMatrix1T = wrangler->getCuFunction(PxgKernelIds::ARTI_COMPUTE_MASS_MATRIX);
				const CUresult result = mCudaContext->launchKernel(computeMassMatrix1T, num1TBlocks * 2, 1, 1, numThreadsPerWarp, 1, 1, 0, stream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, stream, "GPU computeUnconstrainedSpatialInertiaKernel fail!\n");
			}

			{
				const CUfunction computeAccelerationsKernelFunction1T = wrangler->getCuFunction(PxgKernelIds::ARTI_COMPUTE_UNCONSTRAINED_ACCEL);
				const CUresult result = mCudaContext->launchKernel(computeAccelerationsKernelFunction1T, num1TBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, stream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, stream, "GPU computeUnconstrainedAccelerationsKernel fail!\n");
			}
		}

		return 0;
	}

	PxU32 PxgArticulationCore::setupInternalConstraints(const PxU32 nbArticulations, const PxReal stepDt, const PxReal dt, const PxReal invDt, const bool isTGSSolver)
	{
		const PxU32 numThreadsPerWarp = 32;
		PxU32 numWarpsPerBlock = 1;
		PxU32 numBlocks = (nbArticulations + numThreadsPerWarp - 1) / numThreadsPerWarp;

		if (numBlocks)
		{
			PX_PROFILE_ZONE("GpuDynamics.setupInternalConstraints", 0);

			CUstream stream = mStream; //*mSolverStream

			const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SETUP_INTERNAL);

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
				CUDA_KERNEL_PARAM(stepDt),
				CUDA_KERNEL_PARAM(dt),
				CUDA_KERNEL_PARAM(invDt),
				CUDA_KERNEL_PARAM(isTGSSolver)
			};

			//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
			//CUresult result = cuLaunchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, mStream, kernelParams, 0);
			const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, stream, EPILOG);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			gpuDebugStreamSync(mCudaContext, stream, "GPU setupInternalConstraints kernel fail!\n");
		}

		return 0;
	}

	void PxgArticulationCore::syncStream()
	{
		PX_PROFILE_ZONE("PxgArticulationCore.syncStream", 0);

		//Make mSolverStream wait for mStream to finish its work before continuing
		synchronizeStreams(mCudaContext, mStream, *mSolverStream, mFinishEvent);
	}

	void PxgArticulationCore::syncUnconstrainedVelocities()
	{
		PX_PROFILE_ZONE("PxgArticulationCore.syncUnconstrainedVelocities", 0);
		//KS - technically, we could hoist this forward to just after unconstrained, but then we'd need another event to sync on in the internal solver
		mCudaContext->eventRecord(mComputeUnconstrainedEvent, mStream);
		//Make mSolverStream wait for compute unconstrained velocities to finish!
		mCudaContext->streamWaitEvent(*mSolverStream, mComputeUnconstrainedEvent);

	}

	void PxgArticulationCore::synchronizedStreams(CUstream bpStream, CUstream npStream)
	{
		PX_PROFILE_ZONE("PxgArticulationCore.synchronizedStreams", 0);

		CUresult result = mCudaContext->eventRecord(mFlushArticulationDataEvent, mStream);
		PX_ASSERT(result == CUDA_SUCCESS);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuEventRecord failed\n");

		result = mCudaContext->streamWaitEvent(bpStream, mFlushArticulationDataEvent);
		PX_ASSERT(result == CUDA_SUCCESS);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuStreamWaitEvent failed\n");

		result = mCudaContext->streamWaitEvent(npStream, mFlushArticulationDataEvent);
		PX_ASSERT(result == CUDA_SUCCESS);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuStreamWaitEvent failed\n");
	}


	void PxgArticulationCore::saveVelocities()
	{
		PX_PROFILE_ZONE("GpuArticulationCore.saveVelocities", 0);
	
		// PGS only
		CUfunction artiSaveVelocitiesFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SAVE_VELOCITY_PGS);

		const PxU32 numThreadsPerWarp = 32;
		PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
		//PxU32 numBlocks = (mNbActiveArticulation + numWarpsPerBlock - 1) / numWarpsPerBlock;
		PxU32 num1TBlocks = (mNbActiveArticulation + PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES - 1) / PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES;

		if (num1TBlocks)
		{
			//validateData();

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
			};

			const CUresult result = mCudaContext->launchKernel(artiSaveVelocitiesFunction, num1TBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiSaveVelocitiesFunction fail to launch kernel!!\n");

			gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU artiSaveVelocitiesFunction kernel fail!\n");
		}

#if 0
		if (numBlocks)
		{
			//validateData();

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			void* artiKernelParams[] =
			{
				(void*)&descptr,
			};

			const CUresult result = cuLaunchKernel(artiSaveVelocitiesFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, artiKernelParams, 0);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiSaveVelocitiesFunction fail to launch kernel!!\n");

			gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU artiSaveVelocitiesFunction kernel fail!\n");
		}
#endif
	}

	// Before solving internal constraints, ensure to propagate remaining impulses from solvePartitions to the
	// articulation root via "averageLinkImpulsesAndPropagate"
	// This ensures that future impulse propagations do not originate from rigid body or articulation impulses,
	// and that there are no lingering impulses from the rigid body or articulation solver (i.e.,
	// solvePartitions).

	void PxgArticulationCore::propagateRigidBodyImpulsesAndSolveInternalConstraints(const PxReal dt, const PxReal invDt, const bool velocityIteration,
		const PxReal elapsedTime, const PxReal biasCoefficient, PxU32* staticContactUniqueIds, PxU32* staticJointUniqueIds, 
		CUdeviceptr sharedDesc, bool doFriction, bool isTGS, bool residualReportingEnabled, bool isExternalForcesEveryTgsIterationEnabled)
	{
#if ARTI_GPU_DEBUG
		PX_PROFILE_ZONE("GpuArticulationCore.solveInternalConstraints", 0);
#endif
		
		const PxU32 numThreadsPerWarp = WARP_SIZE;
		const PxU32 numWarpsPerBlock = 1;
		PxU32 numArticsPerBlock = PxgArticulationCoreKernelBlockDim::SOLVE_INTERNAL_CONSTRAINTS;// / numThreadsPerWarp;
		PxU32 numBlocks = (mNbActiveArticulation + numArticsPerBlock - 1) / numArticsPerBlock;

		if (numBlocks)
		{
			KernelWrangler* wrangler = mGpuKernelWranglerManager->getKernelWrangler();

			// In the first kernel called, propagate rigid body impulses as well.
			const CUfunction artiPropagateRigidImpulseAndSolveSelfConstraintsFunction = isTGS ? wrangler->getCuFunction(PxgKernelIds::ARTI_PROPAGATE_RIGID_IMPULSES_AND_SOLVE_SELF_TGS) :
																								wrangler->getCuFunction(PxgKernelIds::ARTI_PROPAGATE_RIGID_IMPULSES_AND_SOLVE_SELF);

			const CUfunction artiSolveInternalConstraintsFunction = isTGS ? wrangler->getCuFunction(PxgKernelIds::ARTI_SOLVE_INTERNAL_CONSTRAINTS_TGS) :
																			wrangler->getCuFunction(PxgKernelIds::ARTI_SOLVE_INTERNAL_CONSTRAINTS);

			const CUfunction artiSolveInternalTendonAndMimicJointConstraintsFunction = wrangler->getCuFunction(PxgKernelIds::ARTI_SOLVE_INTERNAL_TENDON_AND_MIMIC_JOINT);

			CUdeviceptr artiCoreDescptr = mArticulationCoreDescd.getDevicePtr();
			CUdeviceptr solverCoreDescptr = mGpuContext->getGpuSolverCore()->getSolverCoreDescDeviceptr();

			{
				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(artiCoreDescptr),
					CUDA_KERNEL_PARAM(solverCoreDescptr),
					CUDA_KERNEL_PARAM(velocityIteration),
					CUDA_KERNEL_PARAM(elapsedTime),
					CUDA_KERNEL_PARAM(sharedDesc),
					CUDA_KERNEL_PARAM(doFriction),
				};

				// In the first kernel called, propagate rigid body impulses as well.
				const CUresult result = mCudaContext->launchKernel(artiPropagateRigidImpulseAndSolveSelfConstraintsFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveSelfConstraints fail to launch kernel!!\n");

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU solveSelfConstraints kernel fail!\n");
			}

			{
				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(artiCoreDescptr),
					CUDA_KERNEL_PARAM(biasCoefficient),
					CUDA_KERNEL_PARAM(dt),
					CUDA_KERNEL_PARAM(invDt),
					CUDA_KERNEL_PARAM(velocityIteration),
					CUDA_KERNEL_PARAM(isTGS)
				};

				const CUresult result = mCudaContext->launchKernel(artiSolveInternalTendonAndMimicJointConstraintsFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiSolveInternalTendonAndMimicJointConstraints1T fail to launch kernel!!\n");

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU artiSolveInternalTendonAndMimicJointConstraints1T kernel fail!\n");
			}

			{
				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(artiCoreDescptr),
					CUDA_KERNEL_PARAM(dt),
					CUDA_KERNEL_PARAM(invDt),
					CUDA_KERNEL_PARAM(velocityIteration),
					CUDA_KERNEL_PARAM(elapsedTime),
					CUDA_KERNEL_PARAM(biasCoefficient),
					CUDA_KERNEL_PARAM(staticContactUniqueIds),
					CUDA_KERNEL_PARAM(staticJointUniqueIds),
					CUDA_KERNEL_PARAM(sharedDesc),
					CUDA_KERNEL_PARAM(doFriction),
					CUDA_KERNEL_PARAM(residualReportingEnabled),
					CUDA_KERNEL_PARAM(isExternalForcesEveryTgsIterationEnabled)
				};

				const CUresult result = mCudaContext->launchKernel(artiSolveInternalConstraintsFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveInternalConstraints fail to launch kernel!!\n");

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU solveInternalConstraints kernel fail!\n");
			}
		}
	}

	void PxgArticulationCore::outputVelocity(CUdeviceptr sharedDesc, CUstream solverStream, bool isTGS)
	{
#if ARTI_GPU_DEBUG
		PX_PROFILE_ZONE("GpuArticulationCore.outputVelocity", 0);
#endif

		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlock = 1;
		PxU32 numArticsPerBlock = PxgArticulationCoreKernelBlockDim::SOLVE_INTERNAL_CONSTRAINTS;// / numThreadsPerWarp;
		PxU32 numBlocks = (mNbActiveArticulation + numArticsPerBlock - 1) / numArticsPerBlock;

		if (numBlocks)
		{
			const CUfunction artiOutputVelocityFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_OUTPUT_VELOCITY);

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			{
				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(descptr),
					CUDA_KERNEL_PARAM(sharedDesc),
					CUDA_KERNEL_PARAM(isTGS)
				};

				const CUresult result = mCudaContext->launchKernel(artiOutputVelocityFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, solverStream, EPILOG);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiOutputVelocity fail to launch kernel!!\n");

				gpuDebugStreamSync(mCudaContext, solverStream, "GPU artiOutputVelocity kernel fail!\n");
			}
		}
	}

	void PxgArticulationCore::pushImpulse(CUstream solverStream)
	{
		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlock = 1;
		PxU32 numArticsPerBlock = PxgArticulationCoreKernelBlockDim::SOLVE_INTERNAL_CONSTRAINTS;// / numThreadsPerWarp;
		PxU32 numBlocks = (mNbActiveArticulation + numArticsPerBlock - 1) / numArticsPerBlock;

		if (numBlocks)
		{
			const CUfunction artiPushImpulseFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_PUSH_IMPULSE);

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			{
				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(descptr)
				};

				const CUresult result = mCudaContext->launchKernel(artiPushImpulseFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, solverStream, EPILOG);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiPushImpulse fail to launch kernel!!\n");

				gpuDebugStreamSync(mCudaContext, solverStream, "GPU artiPushImpulse kernel fail!\n");
			}
		}
	}

	void PxgArticulationCore::stepArticulation(const PxReal stepDt)
	{
		// TGS only

#if ARTI_GPU_DEBUG
		PX_PROFILE_ZONE("GpuArticulationCore.stepArticulation", 0);
#endif
		const PxU32 numThreadsPerWarp = WARP_SIZE;
		PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
		PxU32 numBlocks = (mNbActiveArticulation + PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES - 1) / PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES;

		if (numBlocks)
		{
			//validateData();

			CUfunction stepKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_STEP_TGS);

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
				CUDA_KERNEL_PARAM(stepDt)
			};

			//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
			const CUresult result = mCudaContext->launchKernel(stepKernel, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU stepArticulation kernel fail!\n");
		}
	}

	void PxgArticulationCore::applyTgsSubstepForces(PxReal stepDt, CUstream stream)
	{
		// one thread per articulation
		const PxU32 numThreadsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES;
		const PxU32 numBlocks = (mNbActiveArticulation + numThreadsPerBlock - 1) / numThreadsPerBlock;

		if (numBlocks)
		{
			CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_APPLY_TGS_SUBSTEP_FORCES);
			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
				CUDA_KERNEL_PARAM(stepDt),
			};

			const CUresult result = mCudaContext->launchKernel(kernelFunction,
				numBlocks, 1, 1, numThreadsPerBlock, 1, 1,
				0, stream, EPILOG);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiApplyTgsSubstepForces failed to launch kernel!! %i\n", result);

			gpuDebugStreamSync(mCudaContext, stream, "GPU artiApplyTgsSubstepForces kernel fail!\n");
		}
	}


	void PxgArticulationCore::averageDeltaV(const PxU32 nbSlabs, CUstream stream, float4* velocities, const PxU32 partitionId, bool isTGS, CUdeviceptr sharedDescd)
	{
		const PxU32 numThreadsPerWarp = WARP_SIZE;
		PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
		PxU32 numBlocks = (mNbActiveArticulation + PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES - 1) / PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES;

		if (numBlocks)
		{
			KernelWrangler* wrangler = mGpuKernelWranglerManager->getKernelWrangler();

			CUfunction propagateImpulseKernel = isTGS?	wrangler->getCuFunction(PxgKernelIds::ARTI_PROPAGATE_IMPULSE_TGS) : 
														wrangler->getCuFunction(PxgKernelIds::ARTI_PROPAGATE_IMPULSE_PGS);
			CUfunction propagateVelocityKernel = isTGS ?	wrangler->getCuFunction(PxgKernelIds::ARTI_PROPAGATE_VELOCITY_TGS) :
															wrangler->getCuFunction(PxgKernelIds::ARTI_PROPAGATE_VELOCITY);

			CUdeviceptr descptr = mArticulationCoreDescd.getDevicePtr();
			CUdeviceptr solverCoreDescptr = mGpuContext->getGpuSolverCore()->getSolverCoreDescDeviceptr();

			{
			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
				CUDA_KERNEL_PARAM(solverCoreDescptr),
				CUDA_KERNEL_PARAM(sharedDescd),
				CUDA_KERNEL_PARAM(partitionId)
			};

			const CUresult result = mCudaContext->launchKernel(propagateImpulseKernel, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, stream, EPILOG);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiPropagateImpulses2 fail to launch kernel!! %i\n", result);

			gpuDebugStreamSync(mCudaContext, stream, "GPU artiPropagateImpulses2 kernel fail!\n");
			}
			{
			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(descptr),
				CUDA_KERNEL_PARAM(partitionId),
				CUDA_KERNEL_PARAM(velocities)
			};

			const CUresult result = mCudaContext->launchKernel(propagateVelocityKernel, numBlocks, nbSlabs, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, stream, EPILOG);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiPropagateVelocity fail to launch kernel!! %i\n", result);

			gpuDebugStreamSync(mCudaContext, stream, "GPU artiPropagateVelocity kernel fail!\n");
			}
		}
	}

	void PxgArticulationCore::updateBodies(PxReal dt, bool integrate, bool /*suppressReadback*/)
	{
		// PT: COMPUTE_UNCONSTRAINED_VELOCITES ?
		const PxU32 numThreadsPerWarp = WARP_SIZE;
		const PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;

		// PT: TODO: is mNbActiveArticulation only the non-sleeping ones??
		const PxU32 num1TBlocks = (mNbActiveArticulation + PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES - 1) / PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES;

		if (num1TBlocks)
		{
			//validateData();

			PX_PROFILE_ZONE("GpuArticulationCore.updateBodies1T", 0);

			const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_UPDATE_BODIES);

			CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(coreDescptr),
				CUDA_KERNEL_PARAM(dt),
				CUDA_KERNEL_PARAM(integrate),
			};

			//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
			const CUresult result = mCudaContext->launchKernel(kernelFunction, num1TBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU updatebodies kernel fail!\n");
		}

		if(1)
		{
			if (num1TBlocks)
			{
				PX_PROFILE_ZONE("GpuArticulationCore.updateBodiesPart2", 0);

				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_UPDATE_BODIES2);

				CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(coreDescptr),
					CUDA_KERNEL_PARAM(dt),
					CUDA_KERNEL_PARAM(integrate),
				};

				const PxU32 nbThreadsZ = 4;	// PT: we reach the 1024 threads/block limit with 16

				const PxU32 gridDimX = num1TBlocks;
				const PxU32 gridDimY = 1;
				const PxU32 gridDimZ = (mArticulationCoreDesc->mMaxLinksPerArticulation + nbThreadsZ - 1) / nbThreadsZ;
				const PxU32 blockDimX = numThreadsPerWarp;
				const PxU32 blockDimY = numWarpsPerBlock;
				const PxU32 blockDimZ = nbThreadsZ;

				const CUresult result = mCudaContext->launchKernel(kernelFunction, gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ, 0, *mSolverStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU updatebodies kernel fail!\n");
			}

			if (num1TBlocks)
			{
				PX_PROFILE_ZONE("GpuArticulationCore.updateBodiesPart3", 0);

				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_UPDATE_BODIES3);

				CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(coreDescptr),
					CUDA_KERNEL_PARAM(dt),
					CUDA_KERNEL_PARAM(integrate),
				};

				const PxU32 gridDimX = num1TBlocks;
				const PxU32 gridDimY = 1;
				const PxU32 gridDimZ = 1;
				const PxU32 blockDimX = numThreadsPerWarp;
				const PxU32 blockDimY = numWarpsPerBlock;
				const PxU32 blockDimZ = 1;

				const CUresult result = mCudaContext->launchKernel(kernelFunction, gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ, 0, *mSolverStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU updatebodies kernel fail!\n");
			}

		}
	}

	void PxgArticulationCore::gpuMemDMAbackArticulation(
		PxInt8ArrayPinned& linkAndJointAndRootStateData,
		PxPinnedArray<PxgSolverBodySleepData>& sleepPool, PxPinnedArray<Dy::ErrorAccumulator>& internalResidualPerArticulation, 
		PxPinnedArray<Dy::ErrorAccumulator>& contactResidual)
	{
		const PxU32 numThreadsPerWarp = 32;
		PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
		PxU32 numBlocks = (mNbActiveArticulation + numWarpsPerBlock - 1) / numWarpsPerBlock;

		if (numBlocks)
		{
			mArticulationOutputDesc->linkAndJointAndRootStateData = linkAndJointAndRootStateData.begin();
			mArticulationOutputDesc->sleepData = sleepPool.begin();
			if (contactResidual.size())
			{
				mArticulationOutputDesc->errorAccumulator = internalResidualPerArticulation.begin();
				mArticulationOutputDesc->contactResidualAccumulator = contactResidual.begin();
			}
			else
			{
				mArticulationOutputDesc->errorAccumulator = NULL;
				mArticulationOutputDesc->contactResidualAccumulator = NULL;
			}

			//dma output desc to gpu
			mCudaContext->memcpyHtoDAsync(mArticulationOutputDescd.getDevicePtr(), mArticulationOutputDesc, sizeof(PxgArticulationOutputDesc), *mSolverStream);

			PX_PROFILE_ZONE("GpuArticulationCore.gpuMemDMAbackArticulation", 0);

			const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_DMA_DATA);

			CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();
			CUdeviceptr outputDescptr = mArticulationOutputDescd.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(coreDescptr),
				CUDA_KERNEL_PARAM(outputDescptr)
			};

			//In this set up, each blocks has two warps, each warps has 32 threads. Each warp will work on one articulation.  
			const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, *mSolverStream, EPILOG);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			gpuDebugStreamSync(mCudaContext, *mSolverStream, "GPU gpuMemDMAbackArticulation kernel fail!\n");
		}
	}

	bool PxgArticulationCore::getArticulationData(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent, PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments) const
	{
		PxScopedCudaLock _lock(*mCudaContextManager);
		bool success = true;

		if(startEvent)
		{
			mCudaContext->streamWaitEvent(mStream, startEvent);
		}

		switch(dataType)
		{
			case PxArticulationGPUAPIReadType::eJOINT_POSITION:			//!< The joint positions. 1 PxReal per Dof.
			case PxArticulationGPUAPIReadType::eJOINT_VELOCITY:			//!< The joint velocities. 1 PxReal per Dof.
			case PxArticulationGPUAPIReadType::eJOINT_ACCELERATION:		//!< The joint accelerations. 1 PxReal per Dof.
			case PxArticulationGPUAPIReadType::eJOINT_FORCE:			//!< The joint force. 1 PxReal per dof.
			case PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION:	//!< The velocity targets for the joint drives. 1 PxReal per dof.
			case PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY:	//!< The position targets for the joint drives. 1 PxReal per dof.
			{
				success = getDofStates(data, gpuIndices, nbElements, maxDofs, dataType);
				break;
			}
			case PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE:			//!< The root link global pose. 1 PxTransform per articulation.
			{
				// attention - needs maxLinks = 1 for launch config!
				success = getTransformStates(data, gpuIndices, nbElements, 1, dataType);
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE:			//!< The link global pose including root link. 1 PxTransform per link.
			{
				success = getTransformStates(data, gpuIndices, nbElements, maxLinks, dataType);
				break;
			}
			case PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY:		//!< The root link linear velocity. 1 PxVec3 per articulation.
        	case PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY:     //!< The roor link angular velocity. 1 PxVec3 per articulation.
			{
				// attention - needs maxLinks = 1 for launch config!
				success = getLinkVelocityStates(data, gpuIndices, nbElements, 1, dataType); 
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY:		//!< The link linear velocities including root link. 1 PxVec3 per link.
        	case PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY:		//!< The link angular velocities including root link. 1 PxVec3 per link.
			case PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION:	//!< The link linear accelerations including root link. 1 PxVec3 per link.
        	case PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION:	//!< The link angular accelerations including root link. 1 PxVec3 per link.
			{
				success = getLinkVelocityStates(data, gpuIndices, nbElements, maxLinks, dataType); 
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE:	//!< The link incoming joint forces including root link. 6 PxReals per link.
			{
				success = getLinkSpatialForceStates(data, gpuIndices, nbElements, maxLinks, dataType);
				break;
			}
			case PxArticulationGPUAPIReadType::eFIXED_TENDON:				//!< Fixed tendon data.
			{
				success = getTendonStates(data, gpuIndices, nbElements, maxFixedTendons, dataType);
				break;
			}
			case PxArticulationGPUAPIReadType::eFIXED_TENDON_JOINT:		//!< Fixed tendon joint data.
			{
				success = getFixedTendonJointStates(data, gpuIndices, nbElements, maxFixedTendons * maxTendonJoints);
				break;
			}
			case PxArticulationGPUAPIReadType::eSPATIAL_TENDON:			//!< Spatial tendon data.
			{
				success = getTendonStates(data, gpuIndices, nbElements, maxSpatialTendons, dataType);
				break;
			}
			case PxArticulationGPUAPIReadType::eSPATIAL_TENDON_ATTACHMENT:  //!< Spatial tendon attachment data.
			{
				success = getSpatialTendonAttachmentStates(data, gpuIndices, nbElements, maxSpatialTendons * maxSpatialTendonAttachments);
				break;
			}
			default:
				PX_ALWAYS_ASSERT();
		}

		if(finishEvent)
		{
			mCudaContext->eventRecord(finishEvent, mStream);
		}
		else
		{
			const CUresult result = mCudaContext->streamSynchronize(mStream);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "getArticulationData: CUDA error, code %u\n", result);

			success = (result == CUDA_SUCCESS);
		}

		return success;
	}

	bool PxgArticulationCore::getDofStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxDofs, PxArticulationGPUAPIReadType::Enum dataType) const
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_DOF_STATES);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		// 1 thread per Dof.
		const PxU32 numBlocks = (nbElements * maxDofs + PxgArticulationCoreKernelBlockDim::ARTI_GET_DOF_STATES - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_DOF_STATES;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxDofs),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_DOF_STATES, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::getTransformStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxLinks, PxArticulationGPUAPIReadType::Enum dataType) const
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_TRANSFORM_STATES);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		// 1 thread per transform.
		const PxU32 numBlocks = (nbElements * maxLinks + PxgArticulationCoreKernelBlockDim::ARTI_GET_TRANSFORM_STATES - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_TRANSFORM_STATES;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxLinks),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_TRANSFORM_STATES, 1, 1, 0, mStream, EPILOG);
		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::getLinkVelocityStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxLinks, PxArticulationGPUAPIReadType::Enum dataType) const
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_VELOCITY_STATES);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		PX_COMPILE_TIME_ASSERT((PxgArticulationCoreKernelBlockDim::ARTI_GET_VELOCITY_STATES % 3) == 0);

		// 3 threads per link
		const PxU32 numBlocks = (3 * nbElements * maxLinks + PxgArticulationCoreKernelBlockDim::ARTI_GET_VELOCITY_STATES - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_VELOCITY_STATES;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxLinks),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_VELOCITY_STATES, 1, 1, 0, mStream, EPILOG);
		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::getLinkSpatialForceStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxLinks, PxArticulationGPUAPIReadType::Enum dataType) const
	{
		PX_UNUSED(dataType); // right now only link incoming joint force.

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_SPATIAL_FORCE_STATES);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		// 1 thread per link
		const PxU32 numThreadsPerElement = sizeof(Cm::UnAlignedSpatialVector) / 4;
		const PxU32 numBlocks = (numThreadsPerElement * nbElements * maxLinks + PxgArticulationCoreKernelBlockDim::ARTI_GET_SPATIAL_FORCE_STATES - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_SPATIAL_FORCE_STATES;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxLinks),
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_SPATIAL_FORCE_STATES, 1, 1, 0, mStream, EPILOG);
		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

#if PX_SUPPORT_OMNI_PVD

	void PxgArticulationCore::ovdArticulationCallback(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
		PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
		PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments)
	{
		PxgSimulationController* controller = mGpuContext->getSimulationController();
		const PxsSimulationControllerOVDCallbacks* ovdCallback = controller->getOVDCallbacks();
		if (controller->getEnableOVDReadback() && ovdCallback)
		{
			PxU32 nbrSubElementsPerBlock;
			PxU32 blockSize;
			ovdCallback->getArticulationDataElements(dataType, maxLinks, maxDofs, maxFixedTendons, maxTendonJoints, maxSpatialTendons, maxSpatialTendonAttachments, nbrSubElementsPerBlock, blockSize);

			PxU32 dataBufferBytes = PxU32(blockSize) * PxU32(nbElements);
			mOvdDataBuffer.resizeUninitialized(dataBufferBytes);

			PxU32 indexBufferBytes = sizeof(PxArticulationGPUIndex) * nbElements;
			mOvdIndexBuffer.resizeUninitialized(indexBufferBytes);

			if (mOvdDataBuffer.begin() && mOvdIndexBuffer.begin())
			{				
				////////////////////////////////////////////////////////////////////////////////
				// Copy the forces and gpuIndices from GPU -> CPU
				////////////////////////////////////////////////////////////////////////////////
				PxCUresult resultData = mCudaContext->memcpyDtoH(mOvdDataBuffer.begin(), CUdeviceptr(data), dataBufferBytes);
				PxCUresult resultIndices = mCudaContext->memcpyDtoH(mOvdIndexBuffer.begin(), CUdeviceptr(gpuIndices), indexBufferBytes);
				if (!resultData && !resultIndices)
				{
					////////////////////////////////////////////////////////////////////////////////
					// Transform from GPUIndices to indices into the PxgBodySimManager mBody array
					////////////////////////////////////////////////////////////////////////////////
					PxgBodySimManager& bodyManager = mGpuContext->getSimulationController()->getBodySimManager();
					PxArticulationGPUIndex* gpuIndicesCPU = reinterpret_cast<PxArticulationGPUIndex*>(mOvdIndexBuffer.begin());
					for (PxU32 i = 0; i < nbElements; i++)
					{
						gpuIndicesCPU[i] = bodyManager.mRemapToNodeMap[gpuIndicesCPU[i]];
					}

					////////////////////////////////////////////////////////////////////////////////
					// Call NpDirectGPUAPI layer callback
					////////////////////////////////////////////////////////////////////////////////
					controller->getOVDCallbacks()->processArticulationSet(reinterpret_cast<Dy::FeatherstoneArticulation**>(controller->getBodySimManager().mBodies.begin()),
						mOvdDataBuffer.begin(), reinterpret_cast<PxArticulationGPUIndex*>(mOvdIndexBuffer.begin()), dataType, nbElements,
						maxLinks, maxDofs, maxFixedTendons, maxTendonJoints, maxSpatialTendons, maxSpatialTendonAttachments);
				}
			}
		}
	}

#endif

	bool PxgArticulationCore::setArticulationData(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent, PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments)
	{
		PxScopedCudaLock _lock(*mCudaContextManager);
		bool success = true;

		if(startEvent)
		{
			mCudaContext->streamWaitEvent(mStream, startEvent);
		}

#if PX_SUPPORT_OMNI_PVD
		ovdArticulationCallback(data, gpuIndices, dataType, nbElements, maxLinks, maxDofs, maxFixedTendons, maxTendonJoints, maxSpatialTendons, maxSpatialTendonAttachments);
#endif

		switch(dataType)
		{
			case PxArticulationGPUAPIWriteType::eJOINT_POSITION:
			case PxArticulationGPUAPIWriteType::eJOINT_VELOCITY:
			{
				setDofStates(data, gpuIndices, nbElements, maxDofs, dataType);
				mNeedsKinematicUpdate = true;
				break;
			}
			case PxArticulationGPUAPIWriteType::eJOINT_FORCE:
			case PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY:
			case PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION:
			{
				setDofStates(data, gpuIndices, nbElements, maxDofs, dataType);
				break;
			}
			case PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE:
			{
				setRootGlobalPoseStates(data, gpuIndices, nbElements);
				mNeedsKinematicUpdate = true;
				break;
			}
        	case PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY:
			case PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY:
			{
				setRootVelocityStates(data, gpuIndices, nbElements, dataType);
				mNeedsKinematicUpdate = true;
				break;
			}
			case PxArticulationGPUAPIWriteType::eLINK_FORCE:
			{
				setLinkForceStates(data, gpuIndices, nbElements, maxLinks);
				break;
			}
			case PxArticulationGPUAPIWriteType::eLINK_TORQUE:
			{
				setLinkTorqueStates(data, gpuIndices, nbElements, maxLinks);
				break;
			}
			case PxArticulationGPUAPIWriteType::eFIXED_TENDON:				//!< Fixed tendon data.
			{
				setTendonStates(data, gpuIndices, nbElements, maxFixedTendons, dataType);
				break;
			}
			case PxArticulationGPUAPIWriteType::eFIXED_TENDON_JOINT:		//!< Fixed tendon joint data.
			{
				setFixedTendonJointStates(data, gpuIndices, nbElements, maxFixedTendons * maxTendonJoints);
				break;
			}
			case PxArticulationGPUAPIWriteType::eSPATIAL_TENDON:			//!< Spatial tendon data.
			{
				setTendonStates(data, gpuIndices, nbElements, maxSpatialTendons, dataType);
				break;
			}
			case PxArticulationGPUAPIWriteType::eSPATIAL_TENDON_ATTACHMENT:  //!< Spatial tendon attachment data.
			{
				setSpatialTendonAttachmentStates(data, gpuIndices, nbElements, maxSpatialTendons * maxSpatialTendonAttachments);
				break;
			}
			default:
				PX_ALWAYS_ASSERT();
		}

		if(finishEvent)
		{
			mCudaContext->eventRecord(finishEvent, mStream);
		}
		else
		{
			const CUresult result = mCudaContext->streamSynchronize(mStream);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "setArticulationData: CUDA error, code %u\n", result);

			success = (result == CUDA_SUCCESS);
		}

		return success;
	}

	bool PxgArticulationCore::setDofStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxDofs, PxArticulationGPUAPIWriteType::Enum dataType)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_DOF_STATES);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		// 1 thread per Dof.
		const PxU32 numBlocks = (nbElements * maxDofs + PxgArticulationCoreKernelBlockDim::ARTI_SET_DOF_STATES - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_DOF_STATES;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxDofs),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_DOF_STATES, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::setRootGlobalPoseStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_ROOT_GLOBAL_POSE_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numBlocks = (nbElements + PxgArticulationCoreKernelBlockDim::ARTI_SET_ROOT_GLOBAL_POSE_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_ROOT_GLOBAL_POSE_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::APPLY_ARTI_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::setRootVelocityStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxArticulationGPUAPIWriteType::Enum dataType)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_ROOT_VELOCITY_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 nbThreadsPerElement = 3u;
		const PxU32 numBlocks = (nbThreadsPerElement * nbElements + PxgArticulationCoreKernelBlockDim::ARTI_SET_ROOT_VELOCITY_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_ROOT_VELOCITY_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_ROOT_VELOCITY_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::setLinkForceStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxLinks)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_LINK_FORCE_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 nbThreadsPerElement = 3u;
		const PxU32 numBlocks = (nbThreadsPerElement * nbElements * maxLinks + PxgArticulationCoreKernelBlockDim::ARTI_SET_LINK_FORCE_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_LINK_FORCE_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxLinks)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_LINK_FORCE_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::setLinkTorqueStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxLinks)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_LINK_TORQUE_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numBlocks = (nbElements * maxLinks + PxgArticulationCoreKernelBlockDim::ARTI_SET_LINK_TORQUE_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_LINK_TORQUE_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxLinks)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_LINK_TORQUE_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::setTendonStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxTendons, PxArticulationGPUAPIWriteType::Enum dataType)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_TENDON_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numBlocks = (nbElements * maxTendons + PxgArticulationCoreKernelBlockDim::ARTI_SET_TENDON_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_TENDON_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxTendons),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_TENDON_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::getTendonStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxTendons, PxArticulationGPUAPIReadType::Enum dataType) const
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_TENDON_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numBlocks = (nbElements * maxTendons + PxgArticulationCoreKernelBlockDim::ARTI_GET_TENDON_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_TENDON_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxTendons),
			CUDA_KERNEL_PARAM(dataType)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_TENDON_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return result == CUDA_SUCCESS;
	}

	bool PxgArticulationCore::setSpatialTendonAttachmentStates(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxTendonsXmaxAttachments)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_SPATIAL_TENDON_ATTACHMENT_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numThreadsPerElement = maxTendonsXmaxAttachments;
		const PxU32 numBlocks = (nbElements * numThreadsPerElement + PxgArticulationCoreKernelBlockDim::ARTI_SET_SPATIAL_TENDON_ATTACHMENT_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_SPATIAL_TENDON_ATTACHMENT_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxTendonsXmaxAttachments)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_SPATIAL_TENDON_ATTACHMENT_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return (result == CUDA_SUCCESS);
	}

	bool PxgArticulationCore::getSpatialTendonAttachmentStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxTendonsXmaxAttachments) const
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_SPATIAL_TENDON_ATTACHMENT_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numThreadsPerElement = maxTendonsXmaxAttachments;
		const PxU32 numBlocks = (nbElements * numThreadsPerElement + PxgArticulationCoreKernelBlockDim::ARTI_GET_SPATIAL_TENDON_ATTACHMENT_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_SPATIAL_TENDON_ATTACHMENT_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements),
			CUDA_KERNEL_PARAM(maxTendonsXmaxAttachments)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_SPATIAL_TENDON_ATTACHMENT_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return (result == CUDA_SUCCESS);
	}

	bool PxgArticulationCore::setFixedTendonJointStates(const void* PX_RESTRICT data , const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxFixedTendonsXmaxTendonJoints)
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_SET_FIXED_TENDON_JOINT_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numThreadsPerElement = maxFixedTendonsXmaxTendonJoints;
		const PxU32 numBlocks = (nbElements * numThreadsPerElement + PxgArticulationCoreKernelBlockDim::ARTI_SET_FIXED_TENDON_JOINT_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_SET_FIXED_TENDON_JOINT_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_SET_FIXED_TENDON_JOINT_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return (result == CUDA_SUCCESS);
	}

	bool PxgArticulationCore::getFixedTendonJointStates(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements, PxU32 maxFixedTendonsXmaxTendonJoints) const
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_GET_FIXED_TENDON_JOINT_STATE);

		CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();

		const PxU32 numThreadsPerElement = maxFixedTendonsXmaxTendonJoints;
		const PxU32 numBlocks = (nbElements * numThreadsPerElement + PxgArticulationCoreKernelBlockDim::ARTI_GET_FIXED_TENDON_JOINT_STATE - 1) / PxgArticulationCoreKernelBlockDim::ARTI_GET_FIXED_TENDON_JOINT_STATE;

		KERNEL_PARAM_TYPE kernelParams[] =
		{
			CUDA_KERNEL_PARAM(coreDescptr),
			CUDA_KERNEL_PARAM(data),
			CUDA_KERNEL_PARAM(gpuIndices),
			CUDA_KERNEL_PARAM(nbElements)
		};

		const CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgArticulationCoreKernelBlockDim::ARTI_GET_FIXED_TENDON_JOINT_STATE, 1, 1, 0, mStream, EPILOG);

		PX_ASSERT(result == CUDA_SUCCESS);

		return (result == CUDA_SUCCESS);
	}

	void PxgArticulationCore::updateArticulationsKinematic(bool zeroSimOutput, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxU32 nbElements)
	{
		//if either root transform or joint position changes, we need to recalculate other link's transform
		if (mNeedsKinematicUpdate)
		{
			mNeedsKinematicUpdate = false;

			CUdeviceptr coreDescptr = mArticulationCoreDescd.getDevicePtr();
			CUfunction updateKinematicKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_UPDATE_KINEMATIC);

			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			PxgSimulationCore* simCore = mGpuContext->getSimulationCore();
			const PxU32 numTotalShapes = simCore->getNumTotalShapes();
			CUdeviceptr bounds = mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr();

			//device ptr
			CUdeviceptr shapes = npCore->mGpuShapesManager.mGpuShapesBuffer.getDevicePtr();
			CUdeviceptr shapeSims = simCore->mPxgShapeSimManager.getShapeSimsDevicePtr();
			CUdeviceptr transformCache = npCore->getTransformCache().getDevicePtr();
			PxgShapeManager& shapeManager = npCore->getGpuShapeManager();
			CUdeviceptr sortedRigidNodeIndices = shapeManager.mGpuRigidIndiceBuffer.getDevicePtr();
			CUdeviceptr shapeIndices = shapeManager.mGpuShapeIndiceBuffer.getDevicePtr();

			KERNEL_PARAM_TYPE kernelParams[] =
			{
				CUDA_KERNEL_PARAM(coreDescptr),
				CUDA_KERNEL_PARAM(shapeSims),
				CUDA_KERNEL_PARAM(shapes),
				CUDA_KERNEL_PARAM(transformCache),
				CUDA_KERNEL_PARAM(sortedRigidNodeIndices),
				CUDA_KERNEL_PARAM(shapeIndices),
				CUDA_KERNEL_PARAM(numTotalShapes),
				CUDA_KERNEL_PARAM(bounds),
				CUDA_KERNEL_PARAM(gpuIndices),
				CUDA_KERNEL_PARAM(nbElements),
				CUDA_KERNEL_PARAM(zeroSimOutput)
			};

			// we launch with one warp per articulation, and two warps per block.
			// we launch a big grid and loop.

			const PxU32 numThreadPerWarp = 32;
			const PxU32 numWarpPerBlock = 2;

			// most GPUs we care about can do 16 blocks per SM, assuming 40 SMs (mid-range 30 series) we can run 640 blocks concurrently.
			// This will obviouly depend on the hardware. So we oversubscribe a bit and see where it's going.
			// AD: could potentially query the device properties to optimize this, but not sure whether this is worth it.
			// in the end, the tradeoff is also about not launching too many blocks that don't do anything, so a smaller
			// grid that loops is probably better.

			// but we still clamp if the number of articulations in the scene is smaller than what we would launch otherwise.
			const PxU32 numBlockNeeded = (mArticulationCoreDesc->nbArticulations + numWarpPerBlock - 1) / numWarpPerBlock;
			const PxU32 gridDim = PxMin<PxU32>(PxgArticulationCoreKernelGridDim::UPDATE_KINEMATIC, numBlockNeeded);

			const CUresult result = mCudaContext->launchKernel(updateKinematicKernelFunction, gridDim, 1, 1, numThreadPerWarp, numWarpPerBlock, 1, 0, mStream, EPILOG);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			gpuDebugStreamSync(mCudaContext, mStream, "GPU artiUpdateDataLaunch kernel fail!\n");
		}
	}

	bool PxgArticulationCore::computeArticulationData(
		void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements,
		PxU32 maxLinks, PxU32 maxDofs, CUevent startEvent, CUevent finishEvent)
	{
		PxScopedCudaLock _lock(*mCudaContextManager);
		bool success = true;

		if(startEvent)
		{
			mCudaContext->streamWaitEvent(mStream, startEvent);
		}

		switch(operation)
		{
			case PxArticulationGPUAPIComputeType::eUPDATE_KINEMATIC:
			{
				PX_ASSERT(!data);	// PT: data is unused in this case
				updateArticulationsKinematic(true, gpuIndices, nbElements);
			}
			break;

			case PxArticulationGPUAPIComputeType::eDENSE_JACOBIANS:
			{
				// Needed in case joint positions have been changed before the call
				updateArticulationsKinematic(true, gpuIndices, nbElements);

				CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ARTI_DENSE_JACOBIANS);

				const PxU32 threadsPerWarp = 32;
				const PxU32 warpsPerBlock = 16;
				const PxU32 blocks = (nbElements + warpsPerBlock - 1) / warpsPerBlock;

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(data),
					CUDA_KERNEL_PARAM(gpuIndices),
					CUDA_KERNEL_PARAM(nbElements),
					CUDA_KERNEL_PARAM(maxLinks),
					CUDA_KERNEL_PARAM(maxDofs),
					CUDA_KERNEL_PARAM(mArticulationCoreDesc->articulations),
				};

				const CUresult result = mCudaContext->launchKernel(kernelFunction, blocks, 1, 1, threadsPerWarp, warpsPerBlock, 1, 0, mStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}
			break;

			// This enum has been deprecated, replaced with PxArticulationGPUAPIComputeType::eMASS_MATRICES
			case PxArticulationGPUAPIComputeType::eGENERALIZED_MASS_MATRICES:
			case PxArticulationGPUAPIComputeType::eMASS_MATRICES:
			{
				// Needed in case joint positions have been changed before the call
				updateArticulationsKinematic(true, gpuIndices, nbElements);

				CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ARTI_MASS_MATRICES);

				const PxU32 threadsPerWarp = 32;
				const PxU32 warpsPerBlock = 8;
				const PxU32 blocks = (nbElements + warpsPerBlock - 1) / warpsPerBlock;
				const bool rootMotion = (operation == PxArticulationGPUAPIComputeType::eMASS_MATRICES);

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(nbElements),
					CUDA_KERNEL_PARAM(data),
					CUDA_KERNEL_PARAM(gpuIndices),
					CUDA_KERNEL_PARAM(maxDofs),
					CUDA_KERNEL_PARAM(rootMotion),
					CUDA_KERNEL_PARAM(mArticulationCoreDesc->articulations),
				};

				const CUresult result = mCudaContext->launchKernel(kernelFunction, blocks, 1, 1, threadsPerWarp, warpsPerBlock, 1, 0, mStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}
			break;

			// This enum has been deprecated, replaced with PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION
			case PxArticulationGPUAPIComputeType::eGENERALIZED_GRAVITY_FORCES:
			case PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION:
			{
				// Needed in case joint positions have been changed before the call
				updateArticulationsKinematic(true, gpuIndices, nbElements);

				CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ARTI_GRAVITY_FORCES);

				const PxU32 threadsPerWarp = 32;
				const PxU32 warpsPerBlock = 8;
				const PxU32 blocks = (nbElements + warpsPerBlock - 1) / warpsPerBlock;
				const bool rootMotion = (operation == PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION);

				const PxVec3 gravity = mGpuContext->getGravity();

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(nbElements),
					CUDA_KERNEL_PARAM(data),
					CUDA_KERNEL_PARAM(gpuIndices),
					CUDA_KERNEL_PARAM(maxDofs),
					CUDA_KERNEL_PARAM(rootMotion),
					CUDA_KERNEL_PARAM(mArticulationCoreDesc->articulations),
					CUDA_KERNEL_PARAM(gravity),
				};

				const CUresult result = mCudaContext->launchKernel(kernelFunction, blocks, 1, 1, threadsPerWarp, warpsPerBlock, 1, 0, mStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}
			break;

			// This enum has been deprecated, replaced with PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION
			case PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_FORCES:
			case PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION:
			{
				// Needed in case joint positions have been changed before the call
				// AD should we zero the state here?
				updateArticulationsKinematic(true, gpuIndices, nbElements);

				CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ARTI_CENTRIFUGAL_FORCES);

				const PxU32 threadsPerWarp = 32;
				const PxU32 warpsPerBlock = 8;
				const PxU32 blocks = (nbElements + warpsPerBlock - 1) / warpsPerBlock;
				const bool rootMotion = (operation == PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION);

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(nbElements),
					CUDA_KERNEL_PARAM(data),
					CUDA_KERNEL_PARAM(gpuIndices),
					CUDA_KERNEL_PARAM(maxDofs),
					CUDA_KERNEL_PARAM(rootMotion),
					CUDA_KERNEL_PARAM(mArticulationCoreDesc->articulations),
				};

				const CUresult result = mCudaContext->launchKernel(kernelFunction, blocks, 1, 1, threadsPerWarp, warpsPerBlock, 1, 0, mStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}
			break;

			case PxArticulationGPUAPIComputeType::eARTICULATION_COMS_WORLD_FRAME:
			case PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME:
			{
				// Needed in case joint positions have been changed before the call
				updateArticulationsKinematic(true, gpuIndices, nbElements);

				CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ARTI_COM);

				const PxU32 threadsPerWarp = 32;
				const PxU32 warpsPerBlock = 8;
				const PxU32 blocks = (nbElements + warpsPerBlock - 1) / warpsPerBlock;
				const bool rootFrame = (operation == PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME);

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(data),
					CUDA_KERNEL_PARAM(gpuIndices),
					CUDA_KERNEL_PARAM(rootFrame),
					CUDA_KERNEL_PARAM(mArticulationCoreDesc->articulations),
				};

				const CUresult result = mCudaContext->launchKernel(kernelFunction, blocks, 1, 1, threadsPerWarp, warpsPerBlock, 1, 0, mStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}
			break;

			case PxArticulationGPUAPIComputeType::eCENTROIDAL_MOMENTUM_MATRICES:
			{
				// Needed in case joint positions have been changed before the call
				updateArticulationsKinematic(true, gpuIndices, nbElements);

				CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ARTI_CENTROIDAL_MATRICES);

				const PxU32 threadsPerWarp = 32;
				const PxU32 warpsPerBlock = 8;
				const PxU32 blocks = (nbElements + warpsPerBlock - 1) / warpsPerBlock;

				KERNEL_PARAM_TYPE kernelParams[] =
				{
					CUDA_KERNEL_PARAM(data),
					CUDA_KERNEL_PARAM(gpuIndices),
					CUDA_KERNEL_PARAM(nbElements),
					CUDA_KERNEL_PARAM(maxDofs),
					CUDA_KERNEL_PARAM(mArticulationCoreDesc->articulations),
				};

				const CUresult result = mCudaContext->launchKernel(kernelFunction, blocks, 1, 1, threadsPerWarp, warpsPerBlock, 1, 0, mStream, EPILOG);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}
			break;

			default:
				PX_ALWAYS_ASSERT();
		}

		if(finishEvent)
		{
			mCudaContext->eventRecord(finishEvent, mStream);
		}
		else
		{
			const CUresult result = mCudaContext->streamSynchronize(mStream);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "computeArticulationData: CUDA error, code %u\n", result);

			success = (result == CUDA_SUCCESS);
		}

		return success;
	}
}

