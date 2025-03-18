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

#include "PxgCudaBuffer.h"
#include "foundation/PxMath.h"
#include "foundation/PxAssert.h"

#include "cudamanager/PxCudaContext.h"
#include "PxPhysXGpu.h"
#include "PxsKernelWrangler.h"
#include "common/PxPhysXCommonConfig.h"
#include "PxgMemoryManager.h"

#define MEMCHECK_SUPPORT 0

using namespace physx;

void PxgCudaBuffer::allocate(const PxU64 size, const char* filename, PxI32 line)
{
	PX_ASSERT(mHeapMemoryAllocator);

	if (mSize < size)
	{
		if (mSize > 0 && mPtr) 
		{
#if MEMCHECK_SUPPORT
			PX_UNUSED(filename);
			PX_UNUSED(line);
			PxgCudaAllocatorCallbackBase* alloc = reinterpret_cast<PxgCudaAllocatorCallbackBase*>(mHeapMemoryAllocator->getAllocator());
			PxU8* ptr = reinterpret_cast<PxU8*>(mPtr);
			alloc->mContextManager->freeDeviceBuffer(ptr);
			mPtr = NULL;
#else
			mHeapMemoryAllocator->deallocate(reinterpret_cast<void*>(mPtr));
#endif
		}
			
		//Allocate either double current size or the requested size, depending on which is larger
		mSize = PxMax(size, mSize * 2);
			
#if MEMCHECK_SUPPORT
		PxgCudaAllocatorCallbackBase* alloc = reinterpret_cast<PxgCudaAllocatorCallbackBase*>(mHeapMemoryAllocator->getAllocator());
		mPtr = CUdeviceptr(alloc->mContextManager->allocDeviceBuffer<PxU8>(PxU32(mSize)));
		PX_ASSERT(mPtr);
#else
		mPtr = reinterpret_cast<CUdeviceptr>(mHeapMemoryAllocator->allocate(mSize, mStatGroup, filename, line));
#endif

#if PX_STOMP_ALLOCATED_MEMORY
		if (mPtr)
		{
			PxgCudaAllocatorCallbackBase* alloc = reinterpret_cast<PxgCudaAllocatorCallbackBase*>(mHeapMemoryAllocator->getAllocator());
			PxCudaContextManager* ccm = alloc->mContextManager;
			if (ccm) 
			{
				PxScopedCudaLock scl(*ccm);
				CUresult result = ccm->getCudaContext()->memsetD8(mPtr, static_cast<unsigned char>(0xcd), mSize);
				if (result != 0)
					PX_ASSERT(result == 0);
			}
			else
			{
				PxGetFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, PX_FL,
					"Not possible to stomp PxgCudaBufferMemory because not cuda context manager is available.");
			}
		}
#endif
	}
}

void PxgCudaBuffer::allocateCopyOldDataAsync(const PxU64 size, PxCudaContext* cudaContext, CUstream stream, const char* filename, PxI32 line)
{
	PX_ASSERT(mHeapMemoryAllocator);

	PxU64 oldSize = mSize;

	//Allocate either double current size or the requested size, depending on which is larger
	mSize = (oldSize < size) ? PxMax(size, mSize * 2) : mSize;

	if (oldSize < size)
	{
		CUdeviceptr oldPtr = mPtr;
#if MEMCHECK_SUPPORT
		PX_UNUSED(filename);
		PX_UNUSED(line);
		PxgCudaAllocatorCallbackBase* alloc = reinterpret_cast<PxgCudaAllocatorCallbackBase*>(mHeapMemoryAllocator->getAllocator());
		mPtr = CUdeviceptr(alloc->mContextManager->allocDeviceBuffer<PxU8>(PxU32(mSize)));
		PX_ASSERT(mPtr);
#else
		mPtr = reinterpret_cast<CUdeviceptr>(mHeapMemoryAllocator->allocate(mSize, mStatGroup, filename, line));
#endif

		if (oldSize > 0 && oldPtr)
		{
			cudaContext->memcpyDtoDAsync(mPtr, oldPtr, oldSize, stream);
			//Defer deletion. This makes sure nothing else gets this memory until after the memcopy has completed

#if MEMCHECK_SUPPORT
//Since MEMCHECK_SUPPORT is only active for invalid memory access debugging, let it leak for now
#else
			mHeapMemoryAllocator->deallocateDeferred(reinterpret_cast<void*>(oldPtr));
#endif
		}	
	}
}

void PxgCudaBuffer::deallocate()
{
	PX_ASSERT(mHeapMemoryAllocator);
	if (mSize && mPtr)	
	{
#if MEMCHECK_SUPPORT
		PxgCudaAllocatorCallbackBase* alloc = reinterpret_cast<PxgCudaAllocatorCallbackBase*>(mHeapMemoryAllocator->getAllocator());
		PxU8* ptr = reinterpret_cast<PxU8*>(mPtr);
		alloc->mContextManager->freeDeviceBuffer(ptr);
#else
		mHeapMemoryAllocator->deallocate(reinterpret_cast<void*>(mPtr));
#endif		
		mPtr = 0;
		mSize = 0;
	}
}

void PxgCudaBuffer::deallocateDeferred()
{
#if MEMCHECK_SUPPORT
	//Since MEMCHECK_SUPPORT is only active for invalid memory access debugging, let it leak for now
#else
	PX_ASSERT(mHeapMemoryAllocator);
	if (mSize && mPtr)	
		mHeapMemoryAllocator->deallocateDeferred(reinterpret_cast<void*>(mPtr));
#endif
}

PxgCudaBuffer::~PxgCudaBuffer()
{
	deallocate();
}

