// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include "CudaKernelWrangler.h"
#include "PxgCopyManager.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgKernelIndices.h"
#include "PxgCudaUtils.h"
#include "PxgCommonDefines.h"
#include "PxsHeapStats.h"

#include "cudamanager/PxCudaContext.h"
#include "foundation/PxMath.h"

#define DEBUG_COPY_MANAGER 0

using namespace physx;

PxgCopyManager::PxgCopyManager(Cm::VirtualAllocatorCallback& hostMappedAlloc) :
						mAllocFailed(false),
						mDescriptorsQueueMapped(hostMappedAlloc, PxsHeapStats::eOTHER, Cm::PinnableAllocatorFallback::eDISABLED),
						mNumDescriptors(0),
						mFinishedEvent(0),
						mEventRecorded(false)
{
}

void PxgCopyManager::createFinishedEvent(PxCudaContext* cudaContext)
{
	cudaContext->eventCreate(&mFinishedEvent, CU_EVENT_DEFAULT);
}

void PxgCopyManager::destroyFinishedEvent(PxCudaContext* cudaContext)
{
	cudaContext->eventDestroy(mFinishedEvent);
}
							
void PxgCopyManager::pushDeferredHtoD(const PxgCopyDesc& desc)
{
	PxU32 newSize = (mNumDescriptors + 1) * sizeof(PxgCopyDesc);
	newSize = (newSize + 255) & ~255; //round up to ensure 256-bytes alignment of the following array
	newSize += (mNumDescriptors + 1) * sizeof(PxU32); //run-sum array

	if (newSize > mDescriptorsQueueMapped.size())
	{
		newSize = PxMax<PxU32>(newSize * 2, 256u);
		if(!mDescriptorsQueueMapped.resize(newSize))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgCopyManager: failed to allocate pinned host buffer for descriptors queue");
			mAllocFailed = true;
			return;
		}
	}

	PxgCopyDesc* descsCPU = reinterpret_cast<PxgCopyDesc*>(mDescriptorsQueueMapped.begin());
	descsCPU[mNumDescriptors++] = desc;
}


bool PxgCopyManager::hasFinishedCopying(PxCudaContext* cudaContext) const
{
	CUresult result = cudaContext->eventQuery(mFinishedEvent);
	PX_ASSERT(result == CUDA_SUCCESS || result == CUDA_ERROR_NOT_READY);

	return result != CUDA_ERROR_NOT_READY;
}

void PxgCopyManager::waitAndReset(PxCudaContext* cudaContext)
{
	if(mEventRecorded)
	{
		CUresult result = cudaContext->eventSynchronize(mFinishedEvent);
		PX_UNUSED(result);
		PX_ASSERT(result == CUDA_SUCCESS);
	}
	resetUnsafe();
}
				
	
void PxgCopyManager::dispatchCopy(CUstream stream, PxCudaContextManager* cudaContextManager, KernelWrangler* kernelWrangler)
{
	PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

	if (mAllocFailed)
	{
		cudaContext->setAbortMode(true);
		resetUnsafe();
		return;
	}

	PX_ASSERT(hasFinishedCopying(cudaContext));

	PxU32 numDescs = mNumDescriptors;
	mEventRecorded = false;
	
	if (!numDescs)
		return;
	
	PxU32 numWarpsPerBlock = COPY_KERNEL_WARPS_PER_BLOCK;
	PxU32 numBlocks = numDescs;
	PxU32 numExtraShared = cudaContextManager->supportsArchSM30() ? 0 : numWarpsPerBlock * WARP_SIZE * sizeof(PxU32);

	CUfunction kernelFunction = kernelWrangler->getCuFunction(PxgKernelIds::MEM_COPY_BALANCED_KERNEL);

	{
		PxgCopyDesc* descsGPU = reinterpret_cast<PxgCopyDesc*>(getMappedDevicePtr(cudaContext, mDescriptorsQueueMapped.begin()));
				
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(descsGPU),
			PX_CUDA_KERNEL_PARAM(numDescs)
		};

		CUresult result = cudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, numExtraShared, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if(result != CUDA_SUCCESS)
			printf("GPU MemCopyBalanced fail to launch kernel!!\n");

#if DEBUG_COPY_MANAGER
		result = cudaContext->streamSynchronize(stream);
		if (result != CUDA_SUCCESS)
			printf("GPU MemCopyBalanced died!!\n");
#endif
	}

	CUresult result = cudaContext->eventRecord(mFinishedEvent, stream);
	mEventRecorded = true;
	PX_UNUSED(result);
	PX_ASSERT(result == CUDA_SUCCESS);
}
