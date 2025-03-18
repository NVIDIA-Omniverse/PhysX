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

#include "PxgMemCopyDispatcher.h"

#include "cudamanager/PxCudaContext.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"

namespace physx
{
	void PxgMemCopyDispatcher::flushCommands(CUstream stream, PxCudaContext* cudaContext, KernelWrangler* kernelWrangler)
	{
		// AD - this assumes the context lock is already held?
		if (mPinnedCopyBuffer.size())
		{
			mDeviceCopyCommands.allocate(mPinnedCopyBuffer.size() * sizeof(PxgPtrPair), PX_FL);
			cudaContext->memcpyHtoDAsync(mDeviceCopyCommands.getDevicePtr(), mPinnedCopyBuffer.begin(), mPinnedCopyBuffer.size() * sizeof(PxgPtrPair), stream);

			CUfunction function = kernelWrangler->getCuFunction(PxgKernelIds::COPY_USER_DATA);

			PX_ASSERT(mMaxSize <= PX_MAX_U32);
			const PxU32 maxS = PxU32(mMaxSize);

			const PxU32 blockSize = 256;
			const PxU32 numBlocks = ((maxS/4) + blockSize-1)/ blockSize;

			CUdeviceptr ptr = mDeviceCopyCommands.getDevicePtr();
			PxU32 count = mPinnedCopyBuffer.size();

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(ptr),
				PX_CUDA_KERNEL_PARAM(count)
			};

			CUresult launchResult = cudaContext->launchKernel(function, numBlocks, count, 1, 256, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(launchResult == CUDA_SUCCESS);
			PX_UNUSED(launchResult);
		}
		mPinnedCopyBuffer.forceSize_Unsafe(0);
		mMaxSize = 0;
	}
}
