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

#include "PxgNonRigidCoreCommon.h"
#include "PxgCudaMemoryAllocator.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgRadixSortKernelIndices.h"
#include "common/PxPhysXCommonConfig.h"

using namespace physx;

PxgEssentialCore::PxgEssentialCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* gpuContext) :
	mGpuKernelWranglerManager(gpuKernelWrangler),
	mCudaContextManager(cudaContextManager),
	mCudaContext(cudaContextManager->getCudaContext()),
	mHeapMemoryManager(heapMemoryManager),
	mSimController(simController),
	mGpuContext(gpuContext),
	mStream(0)
{
}

PxgNonRigidCore::PxgNonRigidCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* gpuContext,
	const PxU32 maxContacts, const PxU32 collisionStackSize, PxsHeapStats::Enum statType) :		
	PxgEssentialCore(gpuKernelWrangler, cudaContextManager, heapMemoryManager, simController, gpuContext),
	mIntermStackAlloc(*heapMemoryManager->mDeviceMemoryAllocators, collisionStackSize),
	mStackSizeNeededOnDevice(heapMemoryManager, statType),
	mStackSizeNeededPinned(NULL),	
	mMaxContacts(maxContacts),
	mCollisionStackSizeBytes(collisionStackSize),
	mRSDesc(heapMemoryManager->mMappedMemoryAllocators),
	mRadixSortDescBuf(heapMemoryManager, statType),
	mRadixCountTotalBuf(heapMemoryManager, statType),
	mContactByRigidBuf(heapMemoryManager, statType),
	mContactSortedByRigidBuf(heapMemoryManager, statType),
	mTempContactByRigidBitBuf(heapMemoryManager, statType),
	mContactRemapSortedByRigidBuf(heapMemoryManager, statType),
	mContactSortedByParticleBuf(heapMemoryManager, statType),
	mTempContactByParticleBitBuf(heapMemoryManager, statType),
	mContactRemapSortedByParticleBuf(heapMemoryManager, statType),
	mTempContactBuf(heapMemoryManager, statType),
	mTempContactRemapBuf(heapMemoryManager, statType),
	mTempContactBuf2(heapMemoryManager, statType),
	mTempContactRemapBuf2(heapMemoryManager, statType),
#if PX_ENABLE_SIM_STATS
	mCollisionStackSizeBytesStats(0)
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
{
	mStackSizeNeededOnDevice.allocateElements(1, PX_FL);
	mStackSizeNeededPinned = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);
	*mStackSizeNeededPinned = 0;

	mRadixCountSize = sizeof(PxU32) * PxgRadixSortKernelGridDim::RADIX_SORT * 16;

	const PxU32 maxContactsRoundedUp4 = (mMaxContacts + 3) & ~(4 - 1);

	mContactByRigidBuf.allocateElements(mMaxContacts, PX_FL);							// PxNodeIndex, no radix sort.
	mContactSortedByRigidBuf.allocateElements(mMaxContacts, PX_FL);						// PxNodeIndex, no radix sort.
	mTempContactByRigidBitBuf.allocateElements(maxContactsRoundedUp4, PX_FL);			// PxU32, used in radix sort
	mContactRemapSortedByRigidBuf.allocateElements(maxContactsRoundedUp4, PX_FL);		// PxU32, used in radix sort

	mContactSortedByParticleBuf.allocateElements((mMaxContacts + 1) & ~(2 - 1), PX_FL);
	mTempContactByParticleBitBuf.allocateElements(maxContactsRoundedUp4, PX_FL);		// PxU32, used in radix sort
	mContactRemapSortedByParticleBuf.allocateElements(maxContactsRoundedUp4, PX_FL);	// PxU32, used in radix sort
		
	mTempContactBuf.allocateElements(maxContactsRoundedUp4, PX_FL);			// PxU32, used in radix sort
	mTempContactRemapBuf.allocateElements(maxContactsRoundedUp4, PX_FL);	// PxU32, used in radix sort
	mTempContactBuf2.allocateElements(maxContactsRoundedUp4, PX_FL);		// PxU32, used in radix sort
	mTempContactRemapBuf2.allocateElements(maxContactsRoundedUp4, PX_FL); 	// PxU32, used in radix sort
}

PxgNonRigidCore::~PxgNonRigidCore()
{
	mCudaContextManager->acquireContext();

	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mStackSizeNeededPinned);

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
