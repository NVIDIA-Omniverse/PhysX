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

#include "PxgCudaMemoryAllocator.h"
#include "foundation/PxErrors.h"
#include "foundation/PxMath.h"
#include "foundation/PxPreprocessor.h"

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
#include <cuda.h>
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif
#include "foundation/PxAllocator.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxAssert.h"
#include "cudamanager/PxCudaContextManager.h"

#include "cudamanager/PxCudaContext.h"
#include "common/PxPhysXCommonConfig.h"

using namespace physx;

// memory tracking.
#if PX_DEBUG
#include "PxgMemoryTracker.h"
static MemTracker deviceMemTracker;
static MemTracker hostMemTracker;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* physx::PxgPinnedMemoryAllocate(PxCudaContext& cudaContext, size_t size, const char* filename, PxI32 line)
{
	PxU8* ptr = NULL;
	CUresult result = cudaContext.memHostAlloc((void**)&ptr, size, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
	if (result != CUDA_SUCCESS || !ptr)
	{
		PxGetFoundation().error(PX_WARN, PX_FL, "Failed to allocate pinned memory.");
		return NULL;
	}

	PX_ASSERT((size_t)(ptr) % 256 == 0); //alignment check. I believe it should be guaranteed

#if PX_STOMP_ALLOCATED_MEMORY
	// fill pinned memory with markers to catch uninitialized memory earlier. 
	// use alternating pattern to avoid pairs of start, end values to cancel each other out.
	PxU32 pat[2] = { 0xcdcdcdcd, 0xdcdcdcdc };
	for (size_t i = 0; i < (size/4); ++i)
		reinterpret_cast<PxU32*>(ptr)[i] = pat[i % 2];
#endif

#if PX_DEBUG
	hostMemTracker.registerMemory(ptr, false, size, filename, line);
#else
	PX_UNUSED(filename);
	PX_UNUSED(line);
#endif

	return ptr;
}

void physx::PxgPinnedMemoryDeallocate(PxCudaContext& cudaContext, void* ptr)
{
	if (ptr == NULL)
		return;

	CUresult result = cudaContext.memFreeHost(ptr);
	PX_UNUSED(result);
	PX_ASSERT(result == CUDA_SUCCESS);
#if PX_DEBUG
	hostMemTracker.unregisterMemory(ptr, false);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* physx::PxgCudaDeviceMemoryAllocate(PxCudaContext& cudaContext, size_t size, const char* filename, PxI32 line)
{
	if (cudaContext.isInAbortMode())
		return NULL;

	PxDeviceAllocatorCallback* callback = cudaContext.getAllocatorCallback();
	if (callback)
	{
		void* ptr = NULL;
		bool result = callback->memAlloc(&ptr, size);
		if (!result)
		{
			cudaContext.setAbortMode(true);
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxDeviceAllocatorCallback failed to allocate memory %zu bytes!", size);
			return NULL;
		}
#if PX_DEBUG
		if (result)
			deviceMemTracker.registerMemory(ptr, true, size, filename, line);
#else
	PX_UNUSED(filename);
	PX_UNUSED(line);
#endif
		return ptr;
	}
	else
	{
		CUdeviceptr ptr = 0;
		CUresult result = cudaContext.memAlloc(&ptr, size);
		PX_ASSERT((ptr & 127) == 0);
		if (result != CUDA_SUCCESS)
		{
			cudaContext.setAbortMode(true);
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgCudaDeviceMemoryAllocator failed to allocate memory %zu bytes! Result = %i", size, result);
			return NULL;
		}
#if PX_DEBUG
		if (result == CUDA_SUCCESS)
			deviceMemTracker.registerMemory(reinterpret_cast<void*>(ptr), true, size, filename, line);
#else
	PX_UNUSED(filename);
	PX_UNUSED(line);
#endif
		return reinterpret_cast<void*>(ptr);
	}
}

void physx::PxgCudaDeviceMemoryDeallocate(PxCudaContext& cudaContext, void* ptr)
{
	PxDeviceAllocatorCallback* callback = cudaContext.getAllocatorCallback();
	if (callback)
	{
		bool result = callback->memFree(ptr);
		if (!result)
			PxGetFoundation().error(PX_WARN, PX_FL, "PxDeviceAllocatorCallback fail to deallocate memory!!\n");
	}
	else
	{
		CUresult result = cudaContext.memFree(reinterpret_cast<CUdeviceptr>(ptr));
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PX_WARN, PX_FL, "PxgCudaDeviceMemoryDeallocate fail to deallocate memory!! Result = %i\n", result);
	}
#if PX_DEBUG
	if (ptr) 
		deviceMemTracker.unregisterMemory(ptr, true);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxgPinnedHostLinearMemoryAllocator::PxgPinnedHostLinearMemoryAllocator(PxCudaContextManager* contextManager, const PxU64 size) : 
	mCudaContext(contextManager->getCudaContext())
{
	reserve(size);
}

PxgPinnedHostLinearMemoryAllocator::~PxgPinnedHostLinearMemoryAllocator()
{
	deallocate();
}

void PxgPinnedHostLinearMemoryAllocator::reserveAndGrow(const PxU64 size)
{
	// only reallocate when the new size is larger than what we had before.
	if (size > mTotalSize)
	{
		deallocate();

		const PxU64 newSize = PxMax(size, PxU64(PxCeil(mTotalSize * 1.5f))); 

		mStart = reinterpret_cast<PxU8*>(PxgPinnedMemoryAllocate(*mCudaContext, newSize, PX_FL));

		mTotalSize = newSize;
		mCurrentSize = 0;
	}
}

void PxgPinnedHostLinearMemoryAllocator::reserve(const PxU64 size)
{
	PX_COMPILE_TIME_ASSERT(sizeof(size_t) == sizeof(PxU64));

	mStart = reinterpret_cast<PxU8*>(PxgPinnedMemoryAllocate(*mCudaContext, size, PX_FL));

	mTotalSize = size;
	mCurrentSize = 0;
}

void PxgPinnedHostLinearMemoryAllocator::reset()
{
	mCurrentSize = 0;
}

void* PxgPinnedHostLinearMemoryAllocator::allocate(const PxU64 size, const PxU64 alignment)
{
	if(size > 0)
	{
		const PxI64 alignedSize = PxI64(size + alignment);
		PxU64 baseOffset = PxU64(physx::PxAtomicAdd(reinterpret_cast<PxI64*>(&mCurrentSize), alignedSize));

		if (baseOffset > mTotalSize)
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgPinnedHostLinearMemoryAllocator: overflowing initial allocation size, increase capacity to at least %u\n", baseOffset);
			return NULL;
		}

		// this takes baseOffset again because of the atomic.
		uintptr_t startAddress = (uintptr_t(mStart)) + (baseOffset - alignedSize);
		startAddress = (startAddress + alignment-1) & (~(alignment - 1));
		return (void*)startAddress;
	}
	return NULL;
}

void PxgPinnedHostLinearMemoryAllocator::deallocate()
{
	if(mTotalSize && mStart)
		PxgPinnedMemoryDeallocate(*mCudaContext, mStart);
}
