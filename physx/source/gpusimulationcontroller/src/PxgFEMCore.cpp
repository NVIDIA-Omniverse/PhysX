// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgFEMCore.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgContext.h"
#include "PxgSolverCore.h"
#include "PxNodeIndex.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgKernelWrangler.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgCudaHelpers.h"

#define FEM_GPU_DEBUG 0

using namespace physx;

PxgFEMCore::PxgFEMCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxgAllocatorDesc& allocDesc, PxgSimulationController* simController, PxgGpuContext* gpuContext,
	const PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS, PxsHeapStats::Enum statType) :
	PxgNonRigidCore(gpuKernelWrangler, cudaContextManager, allocDesc, simController, gpuContext, maxContacts, collisionStackSize, statType),
	mRigidDeformableContactPointBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableContactNormalPenBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableContactBarycentricBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableContactInfoBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableContactRigidMaterialBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableContactCountBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformablePrevContactCountBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformablePrevContactCountPinned(allocDesc.hostAlloc, statType),
	mRigidDeformableSortedContactPointBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableSortedContactNormalPenBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableSortedContactBarycentricBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableSortedContactInfoBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableSortedContactRigidMaterialBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableSortedRigidIdBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableContactBlocks(allocDesc.deviceAlloc, statType),
	mRigidDeformableAppliedForcesBuffer(allocDesc.deviceAlloc, statType),
	mRigidDeformableRefCountBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableContactPointBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableContactNormalPenBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableContactBarycentricBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableContactInfoBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableContactCountBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformablePrevContactCountBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformablePrevContactCountPinned(allocDesc.hostAlloc, statType),
	mParticleDeformableSortedContactPointBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableSortedContactNormalPenBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableSortedContactBarycentricBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableSortedContactInfoBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableContactBlocks(allocDesc.deviceAlloc, statType),
	mParticleDeformableAppliedDeformableForcesBuffer(allocDesc.deviceAlloc, statType),
	mParticleDeformableAppliedParticleForcesBuffer(allocDesc.deviceAlloc, statType),
	mSpeculativeCCDContactOffset(allocDesc.deviceAlloc, statType),
	mRigidDeltaVelBuf(allocDesc.deviceAlloc, statType),
	mTempBlockDeltaVelBuf(allocDesc.deviceAlloc, statType),
	mTempBlockRigidIdBuf(allocDesc.deviceAlloc, statType),
	mTempCellsHistogramBuf(allocDesc.deviceAlloc, statType),
	mTempBlockCellsHistogramBuf(allocDesc.deviceAlloc, statType),
	mTempHistogramCountBuf(allocDesc.deviceAlloc, statType),
	mIsTGS(isTGS)
#if PX_ENABLE_SIM_STATS
	, mContactCountStats(0)
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	, mFinalizeEvent(NULL)
{
	mCudaContextManager->acquireContext();

	mRigidDeformablePrevContactCountPinned.get() = 0;
	mParticleDeformablePrevContactCountPinned.get() = 0;

	//fem vs rigid contact buffer
	mRigidDeformableContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableContactBarycentricBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableContactInfoBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableContactRigidMaterialBuffer.allocateElements(maxContacts, PX_FL);

	// Initialize rigid contact info and flags to zero (no contacts)
	mCudaContextManager->getCudaContext()->memsetD8(mRigidDeformableContactInfoBuffer.getDevicePtr(), static_cast<unsigned char>(0x0), mRigidDeformableContactInfoBuffer.getSize());

	// A vertex contact leaves its barycentric slot untouched, having none, while the sort copies the
	// slot of every contact. Defined once here so that copy never reads an undefined one.
	mCudaContextManager->getCudaContext()->memsetD8(mRigidDeformableContactBarycentricBuffer.getDevicePtr(), static_cast<unsigned char>(0x0), mRigidDeformableContactBarycentricBuffer.getSize());

	mRigidDeformableSortedContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableSortedContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableSortedContactBarycentricBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableSortedContactInfoBuffer.allocateElements(maxContacts, PX_FL);
	mRigidDeformableSortedContactRigidMaterialBuffer.allocateElements(maxContacts, PX_FL);

	//PxNodeIndex is sizeof(PxU64)
	mRigidDeformableSortedRigidIdBuffer.allocateElements(maxContacts, PX_FL);

	mRigidDeformableContactCountBuffer.allocateElements(1, PX_FL);
	mRigidDeformablePrevContactCountBuffer.allocateElements(1, PX_FL);

	//fem vs particle contact buffer
	mParticleDeformableContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableContactBarycentricBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableContactInfoBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableContactCountBuffer.allocateElements(1, PX_FL);
	mParticleDeformablePrevContactCountBuffer.allocateElements(1, PX_FL);

	mParticleDeformableSortedContactPointBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableSortedContactNormalPenBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableSortedContactBarycentricBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableSortedContactInfoBuffer.allocateElements(maxContacts, PX_FL);

	mRigidDeformableAppliedForcesBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableAppliedDeformableForcesBuffer.allocateElements(maxContacts, PX_FL);
	mParticleDeformableAppliedParticleForcesBuffer.allocateElements(maxContacts, PX_FL);

	//linear and angular delta change for rigid body
	mRigidDeltaVelBuf.allocateElements(maxContacts * 2, PX_FL);

	mTempBlockDeltaVelBuf.allocateElements(2 * PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA, PX_FL);
	mTempBlockRigidIdBuf.allocateElements(PxgSoftBodyKernelGridDim::SB_ACCUMULATE_DELTA, PX_FL);

	mTempCellsHistogramBuf.allocate(1024 * 1024 * 64, PX_FL);
	mTempBlockCellsHistogramBuf.allocateElements(32, PX_FL);
	mTempHistogramCountBuf.allocateElements(1, PX_FL);

	//KS - we divide by 32 because this is a block data format
	mRigidDeformableContactBlocks.allocateElements((maxContacts + 31) / 32, PX_FL);
	mParticleDeformableContactBlocks.allocateElements((maxContacts + 31) / 32, PX_FL);

	mCudaContext->eventCreate(&mFinalizeEvent, CU_EVENT_DISABLE_TIMING);


	mCudaContextManager->releaseContext();
}

PxgFEMCore::~PxgFEMCore()
{
	mCudaContextManager->acquireContext();
	mCudaContext->eventDestroy(mFinalizeEvent);
	mFinalizeEvent = NULL;
	mCudaContextManager->releaseContext();
}

void PxgFEMCore::copyRigidAndParticleContactCountsToHost()
{
	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mRigidDeformablePrevContactCountPinned.data(), reinterpret_cast<PxU32*>(getRigidDeformableContactCount().getDevicePtr()), 1, mStream);
	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mParticleDeformablePrevContactCountPinned.data(), reinterpret_cast<PxU32*>(getParticleDeformableContactCount().getDevicePtr()), 1, mStream);

	PxgCudaHelpers::copyDToHAsync(*mCudaContextManager->getCudaContext(), mStackSizeNeededPinned.data(), mStackSizeNeededOnDevice.getTypedPtr(), 1, mStream);
}

void PxgFEMCore::reserveRigidDeltaVelBuf(PxU32 newCapacity)
{
	PxU32 newCapacityElts = PxMax(mMaxContacts, newCapacity) * 2;
	mRigidDeltaVelBuf.allocateElements(newCapacityElts, PX_FL);
}

void PxgFEMCore::reorderRigidContacts()
{
	{
		PxgDevicePointer<PxU32> totalContactCountsd = mRigidDeformableContactCountBuffer.getTypedDevicePtr();

		//rigid body and fem contacts
		PxgDevicePointer<float4> contactsd = mRigidDeformableContactPointBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> normPensd = mRigidDeformableContactNormalPenBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> barycentricsd = mRigidDeformableContactBarycentricBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgDeformablePairInfo> infosd = mRigidDeformableContactInfoBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxU16> materialsd = mRigidDeformableContactRigidMaterialBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> sortedContactsd = mRigidDeformableSortedContactPointBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> sortedNormPensd = mRigidDeformableSortedContactNormalPenBuffer.getTypedDevicePtr();
		PxgDevicePointer<float4> sortedBarycentricsd = mRigidDeformableSortedContactBarycentricBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxgDeformablePairInfo> sortedInfosd = mRigidDeformableSortedContactInfoBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxU16> sortedMaterialsd = mRigidDeformableSortedContactRigidMaterialBuffer.getTypedDevicePtr();

		//sortedInfosd already store the rigid id. However, we are sharing the code with the particle system
		//for rigid delta accumulation so we need to store the sorted rigid id as a PxU32 array. 
		PxgDevicePointer<PxU64> sortedRigidsIdd = mRigidDeformableSortedRigidIdBuffer.getTypedDevicePtr();
		PxgDevicePointer<PxU32> remapByRigidd = mContactRemapSortedByRigidBuffer.getTypedDevicePtr();

		CUfunction reorderFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::FEM_REORDER_RS_CONTACTS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(contactsd),
			PX_CUDA_KERNEL_PARAM(normPensd),
			PX_CUDA_KERNEL_PARAM(barycentricsd),
			PX_CUDA_KERNEL_PARAM(infosd),
			PX_CUDA_KERNEL_PARAM(materialsd),
			PX_CUDA_KERNEL_PARAM(totalContactCountsd),
			PX_CUDA_KERNEL_PARAM(remapByRigidd),
			PX_CUDA_KERNEL_PARAM(sortedContactsd),
			PX_CUDA_KERNEL_PARAM(sortedNormPensd),
			PX_CUDA_KERNEL_PARAM(sortedBarycentricsd),
			PX_CUDA_KERNEL_PARAM(sortedInfosd),
			PX_CUDA_KERNEL_PARAM(sortedMaterialsd),
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

void PxgFEMCore::accumulateRigidDeltas(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
	PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxgDevicePointer<PxNodeIndex> rigidIdsd,
	PxgDevicePointer<PxU32> numIdsd, CUstream stream, bool isTGS)
{
	{
		//CUdeviceptr contactInfosd = mRSSortedContactInfoBuffer.getDevicePtr();
		PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxVec4> blockDeltaVd = mTempBlockDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU64> blockRigidIdd = mTempBlockRigidIdBuf.getTypedDevicePtr();

		const CUfunction rigidFirstKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_FIRST);

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

	{
		//CUdeviceptr contactInfosd = mRSSortedContactInfoBuffer.getDevicePtr();
		PxgDevicePointer<float4> deltaVd = mRigidDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxVec4> blockDeltaVd = mTempBlockDeltaVelBuf.getTypedDevicePtr();
		PxgDevicePointer<PxU64> blockRigidIdd = mTempBlockRigidIdBuf.getTypedDevicePtr();

		const CUfunction rigidSecondKernelFunction = mGpuKernelWranglerManager->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_SECOND);

		float4* solverBodyVelPoold = mGpuContext->getGpuSolverCore()->getSolverBodyVelPoolDevPtr();

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
			PX_CUDA_KERNEL_PARAM(solverBodyVelPoold),
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

PxgParticleDbContactWriter PxgFEMCore::createParticleDbContactWriter()
{
	//Ensure a rigid index has the same size as an encoded particle index
	PX_COMPILE_TIME_ASSERT(sizeof(PxNodeIndex) == sizeof(PxU64));

	PxgParticleDbContactWriter writer;
	writer.outPoint = getParticleDeformableContacts().getTypedPtr();
	writer.outNormalPen = getParticleDeformableNormalPens().getTypedPtr();
	writer.outBarycentric = getParticleDeformableBarycentrics().getTypedPtr();
	writer.outContactInfo = getParticleDeformableContactInfos().getTypedPtr();
	writer.totalContactCount = getParticleDeformableContactCount().getTypedPtr();

	writer.contactByParticle = getContactByParticle().getTypedPtr();
	writer.tempContactByParticle = getTempContactByParticle().getTypedPtr();
	writer.contactIndexSortedByParticle = getContactRemapSortedByParticle().getTypedPtr();

	writer.maxNumContacts = mMaxContacts;
	return writer;
}

PxgRigidDbContactWriter PxgFEMCore::createRigidDbContactWriter()
{
	//Ensure a rigid index has the same size as an encoded particle index
	PX_COMPILE_TIME_ASSERT(sizeof(PxNodeIndex) == sizeof(PxU64));

	PxgRigidDbContactWriter writer;
	writer.outPoint = getRigidDeformableContacts().getTypedPtr();
	writer.outNormalPen = getRigidDeformableNormalPens().getTypedPtr();
	writer.outBarycentric = getRigidDeformableBarycentrics().getTypedPtr();
	writer.outContactInfo = getRigidDeformableContactInfos().getTypedPtr();
	writer.outRigidMaterialIndex = getRigidDeformableContactRigidMaterials().getTypedPtr();
	writer.totalContactCount = getRigidDeformableContactCount().getTypedPtr();

	writer.contactByRigid = reinterpret_cast<PxU64*>(getContactByRigid().getTypedPtr());
	writer.tempContactByRigid = getTempContactByRigid().getTypedPtr();
	writer.contactIndexSortedByRigid = getContactRemapSortedByRigid().getTypedPtr();

	writer.maxNumContacts = mMaxContacts;
	return writer;
}
