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

#include <stdio.h>
#include "CudaKernelWrangler.h"
#include "PxgCopyManager.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgKernelIndices.h"
#include "PxgHeapMemAllocator.h"
#include "PxgCudaUtils.h"
#include "PxgCommonDefines.h"

#include "cudamanager/PxCudaContext.h"

#define DEBUG_COPY_MANAGER 0

using namespace physx;

PxgCopyManager::PxgCopyManager(PxgHeapMemoryAllocatorManager* heapMemoryManager) :
						mDescriptorsQueue(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
						mNumDescriptors(0),
						mFinishedEvent(0),
						mEventRecorded(false),
						mHeapMemoryManager(heapMemoryManager)

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
							
void PxgCopyManager::pushDeferredHtoD(const CopyDesc& desc)
{
	PxU32 newSize = (mNumDescriptors + 1) * sizeof(CopyDesc);
	newSize = (newSize + 255) & ~255; //round up to ensure 256-bytes alignment of the following array
	newSize += (mNumDescriptors + 1) * sizeof(PxU32); //run-sum array

	if (newSize > mDescriptorsQueue.size())
	{
		mDescriptorsQueue.resize(newSize * 2);
	}

	CopyDesc* descsCPU = reinterpret_cast<CopyDesc*>(mDescriptorsQueue.begin());
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
		CopyDesc* descsGPU = reinterpret_cast<CopyDesc*>(getMappedDevicePtr(cudaContext, mDescriptorsQueue.begin()));
				
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
