// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgNonRigidCoreCommon.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgRadixSortDesc.h"
#include "PxgRadixSortKernelIndices.h"
#include "common/PxPhysXCommonConfig.h"

using namespace physx;

PxgEssentialCore::PxgEssentialCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxgAllocatorDesc& allocDesc, PxgSimulationController* simController, PxgGpuContext* gpuContext) :
	mGpuKernelWranglerManager(gpuKernelWrangler),
	mCudaContextManager(cudaContextManager),
	mCudaContext(cudaContextManager->getCudaContext()),
	mSimController(simController),
	mGpuContext(gpuContext),
	mAllocDesc(allocDesc),
	mStream(0)
{
}

PxgNonRigidCore::PxgNonRigidCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxgAllocatorDesc& allocDesc, PxgSimulationController* simController, PxgGpuContext* gpuContext,
	const PxU32 maxContacts, const PxU32 collisionStackSize, PxU32 statType) :
	PxgEssentialCore(gpuKernelWrangler, cudaContextManager, allocDesc, simController, gpuContext),
	mIntermStackAlloc(allocDesc.deviceAlloc, statType, collisionStackSize),
	mStackSizeNeededOnDevice(allocDesc.deviceAlloc, statType),
	mStackSizeNeededPinned(allocDesc.hostAlloc, statType),
	mMaxContacts(maxContacts),
	mCollisionStackSizeBytes(collisionStackSize),
	mRSDesc(allocDesc.hostAlloc, statType),
	mRadixSortDescBuf(allocDesc.deviceAlloc, statType),
	mRadixCountTotalBuf(allocDesc.deviceAlloc, statType),
	mContactByRigidBuffer(allocDesc.deviceAlloc, statType),
	mTempContactByRigidBitBuffer(allocDesc.deviceAlloc, statType),
	mContactRemapSortedByRigidBuffer(allocDesc.deviceAlloc, statType),
	mContactByParticleBuffer(allocDesc.deviceAlloc, statType),
	mTempContactByParticleBitBuffer(allocDesc.deviceAlloc, statType),
	mContactRemapSortedByParticleBuffer(allocDesc.deviceAlloc, statType),
	mTempContactBuffer(allocDesc.deviceAlloc, statType),
	mTempContactRemapBuffer(allocDesc.deviceAlloc, statType),
	mTempContactBuffer2(allocDesc.deviceAlloc, statType),
	mTempContactRemapBuffer2(allocDesc.deviceAlloc, statType),
#if PX_ENABLE_SIM_STATS
	mCollisionStackSizeBytesStats(0)
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
{
	mStackSizeNeededOnDevice.allocateElements(1, PX_FL);
	mStackSizeNeededPinned.get() = 0;

	mRadixCountSize = sizeof(PxU32) * PxgRadixSortKernelGridDim::RADIX_SORT * 16;

	const PxU32 maxContactsRoundedUp4 = (mMaxContacts + 3) & ~(4 - 1);

	mContactByRigidBuffer.allocateElements(mMaxContacts, PX_FL);							// PxNodeIndex, no radix sort.
	mTempContactByRigidBitBuffer.allocateElements(maxContactsRoundedUp4, PX_FL);			// PxU32, used in radix sort
	mContactRemapSortedByRigidBuffer.allocateElements(maxContactsRoundedUp4, PX_FL);		// PxU32, used in radix sort

	mContactByParticleBuffer.allocateElements((mMaxContacts + 1) & ~(2 - 1), PX_FL);
	mTempContactByParticleBitBuffer.allocateElements(maxContactsRoundedUp4, PX_FL);		// PxU32, used in radix sort
	mContactRemapSortedByParticleBuffer.allocateElements(maxContactsRoundedUp4, PX_FL);	// PxU32, used in radix sort
		
	mTempContactBuffer.allocateElements(maxContactsRoundedUp4, PX_FL);			// PxU32, used in radix sort
	mTempContactRemapBuffer.allocateElements(maxContactsRoundedUp4, PX_FL);	// PxU32, used in radix sort
	mTempContactBuffer2.allocateElements(maxContactsRoundedUp4, PX_FL);		// PxU32, used in radix sort
	mTempContactRemapBuffer2.allocateElements(maxContactsRoundedUp4, PX_FL); 	// PxU32, used in radix sort
}

PxgNonRigidCore::~PxgNonRigidCore()
{
	mCudaContextManager->acquireContext();

	//destroy stream
	mCudaContext->streamDestroy(mStream);
	mStream = NULL;

	mCudaContextManager->releaseContext();
}

void PxgNonRigidCore::updateGPURadixSortBlockDesc(CUstream stream, CUdeviceptr inputKeyd, CUdeviceptr inputRankd,
	CUdeviceptr outputKeyd, CUdeviceptr outputRankd, CUdeviceptr radixCountd, CUdeviceptr numKeysd,
	PxgRadixSortBlockDesc* rsDescs, CUdeviceptr radixSortDescBuf0, CUdeviceptr radixSortDescBuf1)
{
	rsDescs[0].inputKeys = reinterpret_cast<PxU32*>(inputKeyd);
	rsDescs[0].inputRanks = reinterpret_cast<PxU32*>(inputRankd);
	rsDescs[0].outputKeys = reinterpret_cast<PxU32*>(outputKeyd);
	rsDescs[0].outputRanks = reinterpret_cast<PxU32*>(outputRankd);
	rsDescs[0].radixBlockCounts = reinterpret_cast<PxU32*>(radixCountd);
	rsDescs[0].numKeys = reinterpret_cast<PxU32*>(numKeysd);

	rsDescs[1].outputKeys = reinterpret_cast<PxU32*>(inputKeyd);
	rsDescs[1].outputRanks = reinterpret_cast<PxU32*>(inputRankd);
	rsDescs[1].inputKeys = reinterpret_cast<PxU32*>(outputKeyd);
	rsDescs[1].inputRanks = reinterpret_cast<PxU32*>(outputRankd);
	rsDescs[1].radixBlockCounts = reinterpret_cast<PxU32*>(radixCountd);
	rsDescs[1].numKeys = reinterpret_cast<PxU32*>(numKeysd);

	mCudaContext->memcpyHtoDAsync(radixSortDescBuf0, (void*)&rsDescs[0], sizeof(PxgRadixSortBlockDesc), stream);
	mCudaContext->memcpyHtoDAsync(radixSortDescBuf1, (void*)&rsDescs[1], sizeof(PxgRadixSortBlockDesc), stream);
}
