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

#include "PxgFEMCore.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgKernelWrangler.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgCudaHelpers.h"

#define FEM_GPU_DEBUG 0

using namespace physx;

PxgFEMCore::PxgFEMCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* gpuContext,
	const PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS, PxsHeapStats::Enum statType) :
	PxgNonRigidCore(gpuKernelWrangler, cudaContextManager, heapMemoryManager, simController, gpuContext, maxContacts, collisionStackSize, statType),
	mRigidContactPointBuf(heapMemoryManager, statType),
	mRigidContactNormalPenBuf(heapMemoryManager, statType),
	mRigidContactBarycentricBuf(heapMemoryManager, statType),
	mRigidContactInfoBuf(heapMemoryManager, statType),
	mRigidTotalContactCountBuf(heapMemoryManager, statType),
	mRigidPrevContactCountBuf(heapMemoryManager, statType),
	mRigidSortedContactPointBuf(heapMemoryManager, statType),
	mRigidSortedContactNormalPenBuf(heapMemoryManager, statType),
	mRigidSortedContactBarycentricBuf(heapMemoryManager, statType),
	mRigidSortedRigidIdBuf(heapMemoryManager, statType),
	mRigidSortedContactInfoBuf(heapMemoryManager, statType),
	mRigidLambdaNBuf(heapMemoryManager, statType),
	mFemLambdaNBuf(heapMemoryManager, statType),
	mFemRigidReferenceCount(heapMemoryManager, statType),
	mFemContactPointBuffer(heapMemoryManager, statType),
	mFemContactNormalPenBuffer(heapMemoryManager, statType),
	mFemContactBarycentric0Buffer(heapMemoryManager, statType),
	mFemContactBarycentric1Buffer(heapMemoryManager, statType),
	mFemContactInfoBuffer(heapMemoryManager, statType),
	mFemTotalContactCountBuffer(heapMemoryManager, statType),
	mPrevFemContactCountBuffer(heapMemoryManager, statType),
	mSpeculativeCCDContactOffset(heapMemoryManager, statType),
	mParticleContactPointBuffer(heapMemoryManager, statType),
	mParticleContactNormalPenBuffer(heapMemoryManager, statType),
	mParticleContactBarycentricBuffer(heapMemoryManager, statType),
	mParticleContactInfoBuffer(heapMemoryManager, statType),
	mParticleTotalContactCountBuffer(heapMemoryManager, statType),
	mPrevParticleContactCountBuffer(heapMemoryManager, statType),
	mParticleSortedContactPointBuffer(heapMemoryManager, statType),
	mParticleSortedContactBarycentricBuffer(heapMemoryManager, statType),
	mParticleSortedContactNormalPenBuffer(heapMemoryManager, statType),
	mParticleSortedContactInfoBuffer(heapMemoryManager, statType),
	mRigidConstraintBuf(heapMemoryManager, statType),
	mFemConstraintBuf(heapMemoryManager, statType),
	mParticleConstraintBuf(heapMemoryManager, statType),
	mRigidAppliedRigidForcesBuf(heapMemoryManager, statType),
	mRigidAppliedFEMForcesBuf(heapMemoryManager, statType),
	mFemAppliedForcesBuf(heapMemoryManager, statType),
	mParticleAppliedFEMForcesBuf(heapMemoryManager, statType),
	mParticleAppliedParticleForcesBuf(heapMemoryManager, statType),
	mRigidDeltaVelBuf(heapMemoryManager, statType),
	mTempBlockDeltaVelBuf(heapMemoryManager, statType),
	mTempBlockRigidIdBuf(heapMemoryManager, statType),
	mTempCellsHistogramBuf(heapMemoryManager, statType),
	mTempBlockCellsHistogramBuf(heapMemoryManager, statType),
	mTempHistogramCountBuf(heapMemoryManager, statType),
	mIsTGS(isTGS),
	mRigidContactCountPrevTimestep(NULL),
	mFemContactCountPrevTimestep(NULL),
	mParticleContactCountPrevTimestep(NULL)
#if PX_ENABLE_SIM_STATS
	, mContactCountStats(0)
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
{
	mCudaContextManager->acquireContext();

	mRigidContactCountPrevTimestep = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);
	mFemContactCountPrevTimestep = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);
	mParticleContactCountPrevTimestep = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);
	*mRigidContactCountPrevTimestep = 0;
	*mFemContactCountPrevTimestep = 0;
	*mParticleContactCountPrevTimestep = 0;

	//fem vs rigid contact buffer
	mRigidContactPointBuf.allocateElements(maxContacts, PX_FL);
	mRigidContactNormalPenBuf.allocateElements(maxContacts, PX_FL);
	mRigidContactBarycentricBuf.allocateElements(maxContacts, PX_FL);
	mRigidContactInfoBuf.allocateElements(maxContacts, PX_FL);

	mRigidSortedContactPointBuf.allocateElements(maxContacts, PX_FL);
	mRigidSortedContactNormalPenBuf.allocateElements(maxContacts, PX_FL);
	mRigidSortedContactBarycentricBuf.allocateElements(maxContacts, PX_FL);
	mRigidSortedContactInfoBuf.allocateElements(maxContacts, PX_FL);

	mRigidLambdaNBuf.allocateElements(maxContacts, PX_FL);
	mFemLambdaNBuf.allocateElements(maxContacts, PX_FL);
	
	//PxNodeIndex is sizeof(PxU64)
	mRigidSortedRigidIdBuf.allocateElements(maxContacts, PX_FL);

	mRigidTotalContactCountBuf.allocateElements(1, PX_FL);
	mRigidPrevContactCountBuf.allocateElements(1, PX_FL);

	//fem vs fem contact buffer
	mFemContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mFemContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mFemContactBarycentric0Buffer.allocateElements(maxContacts, PX_FL);
	mFemContactBarycentric1Buffer.allocateElements(maxContacts, PX_FL);

	mFemContactInfoBuffer.allocateElements(maxContacts, PX_FL);
	mFemTotalContactCountBuffer.allocateElements(1, PX_FL);
	mPrevFemContactCountBuffer.allocateElements(1, PX_FL);

	//fem vs particle contact buffer
	mParticleContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mParticleContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mParticleContactBarycentricBuffer.allocateElements(maxContacts, PX_FL);
	mParticleContactInfoBuffer.allocateElements(maxContacts, PX_FL);
	mParticleTotalContactCountBuffer.allocateElements(1, PX_FL);
	mPrevParticleContactCountBuffer.allocateElements(1, PX_FL);

	mParticleSortedContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mParticleSortedContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mParticleSortedContactBarycentricBuffer.allocateElements(maxContacts, PX_FL);
	mParticleSortedContactInfoBuffer.allocateElements(maxContacts, PX_FL);

	////KS - this will now store an array of 3 floats - normal force + 2* friction force
	mRigidAppliedRigidForcesBuf.allocateElements(maxContacts, PX_FL);
	mRigidAppliedFEMForcesBuf.allocateElements(maxContacts, PX_FL);
	mFemAppliedForcesBuf.allocateElements(maxContacts, PX_FL);
	mParticleAppliedFEMForcesBuf.allocateElements(maxContacts, PX_FL);
	mParticleAppliedParticleForcesBuf.allocateElements(maxContacts, PX_FL);

	//linear and angular delta change for rigid body
	mRigidDeltaVelBuf.allocateElements(maxContacts * 2, PX_FL);

	mTempBlockDeltaVelBuf.allocateElements(2 * PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA, PX_FL);
	mTempBlockRigidIdBuf.allocateElements(PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA, PX_FL);

	mTempCellsHistogramBuf.allocate(1024 * 1024 * 64, PX_FL);
	mTempBlockCellsHistogramBuf.allocateElements(32, PX_FL);
	mTempHistogramCountBuf.allocateElements(1, PX_FL);

	//KS - we divide by 32 because this is a block data format
	mRigidConstraintBuf.allocateElements((maxContacts + 31) / 32, PX_FL);
	mParticleConstraintBuf.allocateElements((maxContacts + 31) / 32, PX_FL);

	mCudaContext->eventCreate(&mFinalizeEvent, CU_EVENT_DISABLE_TIMING);

	mCudaContextManager->releaseContext();
}

PxgFEMCore::~PxgFEMCore()
{
	mCudaContextManager->acquireContext();

	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mRigidContactCountPrevTimestep);
	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mFemContactCountPrevTimestep);
	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mParticleContactCountPrevTimestep);

	mCudaContext->eventDestroy(mFinalizeEvent);
	mFinalizeEvent = NULL;
	mCudaContextManager->releaseContext();
}

void PxgFEMCore::copyContactCountsToHost()
{
	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mRigidContactCountPrevTimestep, reinterpret_cast<PxU32*>(getRigidContactCount().getDevicePtr()), 1, mStream);
	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mFemContactCountPrevTimestep, reinterpret_cast<PxU32*>(getFemContactCount().getDevicePtr()), 1, mStream);
	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mParticleContactCountPrevTimestep, reinterpret_cast<PxU32*>(getParticleContactCount().getDevicePtr()), 1, mStream);

	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mStackSizeNeededPinned, mStackSizeNeededOnDevice.getTypedPtr(), 1, mStream);
}

void PxgFEMCore::reserveRigidDeltaVelBuf(PxU32 newCapacity)
{
	PxU32 newCapacityElts = PxMax(mMaxContacts, newCapacity) * 2;
	mRigidDeltaVelBuf.allocateElements(newCapacityElts, PX_FL);
}

void PxgFEMCore::reorderRigidContacts()
{
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidTotalContactCountBuf.getTypedDevicePtr();

		//rigid body and fem contacts
		PxgDevicePointer<float4> contactsd = mRigidContactPointBuf.getTypedDevicePtr();
		PxgDevicePointer<float4> normPensd = mRigidContactNormalPenBuf.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentricsd = mRigidContactBarycentricBuf.getTypedDevicePtr();
		PxgDevicePointer<PxgFemContactInfo> infosd = mRigidContactInfoBuf.getTypedDevicePtr();
		PxgDevicePointer<float4> sortedContactsd = mRigidSortedContactPointBuf.getTypedDevicePtr();
		PxgDevicePointer<float4> sortedNormPensd = mRigidSortedContactNormalPenBuf.getTypedDevicePtr();
		PxgDevicePointer<float4> sortedBarycentricsd = mRigidSortedContactBarycentricBuf.getTypedDevicePtr();
		PxgDevicePointer<PxgFemContactInfo> sortedInfosd = mRigidSortedContactInfoBuf.getTypedDevicePtr();

		//sortedInfosd already store the rigid id. However, we are sharing the code with the particle system
		//for rigid delta accumulation so we need to store the sorted rigid id as a PxU32 array. 
		PxgDevicePointer<PxU64> sortedRigidsIdd = mRigidSortedRigidIdBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU32> remapByRigidd = mContactRemapSortedByRigidBuf.getTypedDevicePtr();

		CUfunction reorderFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::FEM_REORDER_RS_CONTACTS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(contactsd),
			PX_CUDA_KERNEL_PARAM(normPensd),
			PX_CUDA_KERNEL_PARAM(barycentricsd),
			PX_CUDA_KERNEL_PARAM(infosd),
			PX_CUDA_KERNEL_PARAM(totalContactCountsd),
			PX_CUDA_KERNEL_PARAM(remapByRigidd),
			PX_CUDA_KERNEL_PARAM(sortedContactsd),
			PX_CUDA_KERNEL_PARAM(sortedNormPensd),
			PX_CUDA_KERNEL_PARAM(sortedBarycentricsd),
			PX_CUDA_KERNEL_PARAM(sortedInfosd),
			PX_CUDA_KERNEL_PARAM(sortedRigidsIdd)
		};

		CUresult  result = mCudaContext->launchKernel(reorderFunction, PxgSoftBodyKernelGridDim::SB_REORDERCONTACTS, 1, 1, PxgSoftBodyKernelBlockDim::SB_REORDERCONTACTS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_reorderRSContactsLaunch fail to launch kernel!!\n");
#if FEM_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_reorderRSContactsLaunch fail!!\n");
#endif
	}
}

void PxgFEMCore::accumulateRigidDeltas(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, 
	PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxgDevicePointer<PxNodeIndex> rigidIdsd, PxgDevicePointer<PxU32> numIdsd, CUstream stream, CUevent waitEvent, bool isTGS)
{
	PX_UNUSED(rigidIdsd);

	{
		//CUdeviceptr contactInfosd = mRSSortedContactInfoBuffer.getDevicePtr();
		PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxVec4> blockDeltaVd = mTempBlockDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU64> blockRigidIdd = mTempBlockRigidIdBuf.getTypedDevicePtr();

		const CUfunction rigidFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_FIRST);

		PxCudaKernelParam kernelParams[] =
		{
			//PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(rigidIdsd),
			PX_CUDA_KERNEL_PARAM(numIdsd),
			PX_CUDA_KERNEL_PARAM(deltaVd),
			PX_CUDA_KERNEL_PARAM(blockDeltaVd),
			PX_CUDA_KERNEL_PARAM(blockRigidIdd)
		};

		CUresult result = mCudaContext->launchKernel(rigidFirstKernelFunction, PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA, 1, 1, PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if FEM_GPU_DEBUG
		result = mCudaContext->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU fem accumulateDeltaVRigidFirstLaunch kernel fail!\n");

		int bob = 0;
		PX_UNUSED(bob);
#endif
	}

	mCudaContext->streamWaitEvent(stream, waitEvent);

	{
		//CUdeviceptr contactInfosd = mRSSortedContactInfoBuffer.getDevicePtr();
		PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxVec4> blockDeltaVd = mTempBlockDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU64> blockRigidIdd = mTempBlockRigidIdBuf.getTypedDevicePtr();

		//mCudaContext->streamWaitEvent(solverStream, mSolveSoftBodyRigidEvent);

		const CUfunction rigidSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_SECOND);

		PxCudaKernelParam kernelParams[] =
		{
			//PX_CUDA_KERNEL_PARAM(contactInfosd),
			PX_CUDA_KERNEL_PARAM(rigidIdsd),
			PX_CUDA_KERNEL_PARAM(numIdsd),
			PX_CUDA_KERNEL_PARAM(deltaVd),
			PX_CUDA_KERNEL_PARAM(blockDeltaVd),
			PX_CUDA_KERNEL_PARAM(blockRigidIdd),
			PX_CUDA_KERNEL_PARAM(prePrepDescd),
			PX_CUDA_KERNEL_PARAM(solverCoreDescd),
			PX_CUDA_KERNEL_PARAM(artiCoreDescd),
			PX_CUDA_KERNEL_PARAM(sharedDescd),
			PX_CUDA_KERNEL_PARAM(isTGS)
		};

		CUresult result = mCudaContext->launchKernel(rigidSecondKernelFunction, PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA, 1, 1, PxgSoftBodyKernelBlockDim::SB_ACCUMULATE_DELTA, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if FEM_GPU_DEBUG
		result = mCudaContext->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU fem accumulateDeltaVRigidSecondLaunch kernel fail!\n");

		int bob = 0;
		PX_UNUSED(bob);
#endif
	}
}
