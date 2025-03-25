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

#ifndef PXG_MEM_COPY_DISPATCHER
#define PXG_MEM_COPY_DISPATCHER

#include "foundation/PxUserAllocated.h"
#include "foundation/PxPinnedArray.h"
#include "PxgCudaBuffer.h"

namespace physx
{
	class PxCudaContext;
	class KernelWrangler;

	struct PxgPtrPair
	{
		void* src;
		void* dst;
		size_t size;
	};

	class PxgMemCopyDispatcher : public PxUserAllocated
	{
		PxPinnedArray<PxgPtrPair> mPinnedCopyBuffer;
		PxgCudaBuffer mDeviceCopyCommands;
		size_t mMaxSize;

	public:

		PxgMemCopyDispatcher(PxgHeapMemoryAllocatorManager* gpuHeapAllocator, PxVirtualAllocatorCallback* hostAllocator) :
			mPinnedCopyBuffer(PxVirtualAllocator(hostAllocator)), mDeviceCopyCommands(gpuHeapAllocator, PxsHeapStats::eOTHER),
			mMaxSize(0)
		{
		}

		void addCommand(PxgPtrPair& command)
		{
			mPinnedCopyBuffer.pushBack(command);
			mMaxSize = PxMax(mMaxSize, command.size);
		}

		void flushCommands(CUstream stream, PxCudaContext* cudaContext, KernelWrangler* kernelWrangler);
	};

}
#endif
