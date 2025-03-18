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

#ifndef PXG_CUDA_MEMORY_ALLOCATOR_H
#define PXG_CUDA_MEMORY_ALLOCATOR_H

#include "foundation/PxPreprocessor.h"

#include "foundation/PxAllocator.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxUserAllocated.h"

#include "cudamanager/PxCudaContextManager.h"

namespace physx
{
	class PxCudaContext;

	// Whenever possible, use the macros provided below instead of these functions.
	void*	PxgCudaDeviceMemoryAllocate(PxCudaContext& cudaContext, size_t size, const char* filename, PxI32 line);
	void	PxgCudaDeviceMemoryDeallocate(PxCudaContext& cudaContext, void* ptr);
	void*	PxgPinnedMemoryAllocate(PxCudaContext& cudaContext, size_t size, const char* filename, PxI32 line);
	void	PxgPinnedMemoryDeallocate(PxCudaContext& cudaContext, void* ptr);

	// AD: templated easy-access to the allocation functions:
	template<typename T>
	T*		PxgCudaDeviceMemoryAllocate(PxCudaContextManager& cudaContextManager, PxU64 numElements, const char* filename, PxI32 line)
	{
		PxScopedCudaLock _lock(cudaContextManager);
		return reinterpret_cast<T*>(PxgCudaDeviceMemoryAllocate(*cudaContextManager.getCudaContext(), numElements * sizeof(T), filename, line));
	}

	template<typename T>
	void 	PxgCudaDeviceMemoryDeallocate(PxCudaContextManager& cudaContextManager, T*& ptr)
	{
		if (ptr)
		{
			PxScopedCudaLock _lock(cudaContextManager);
			PxgCudaDeviceMemoryDeallocate(*cudaContextManager.getCudaContext(), ptr);
			ptr = NULL;
		}
	}

	template<typename T>
	T*		PxgPinnedMemoryAllocate(PxCudaContextManager& cudaContextManager, PxU64 numElements, const char* filename, PxI32 line)
	{
		PxScopedCudaLock _lock(cudaContextManager);
		return reinterpret_cast<T*>(PxgPinnedMemoryAllocate(*cudaContextManager.getCudaContext(), numElements * sizeof(T), filename, line));
	}

	template<typename T>
	void 	PxgPinnedMemoryDeallocate(PxCudaContextManager& cudaContextManager, T*& ptr)
	{
		if (ptr)
		{
			PxScopedCudaLock _lock(cudaContextManager);
			PxgPinnedMemoryDeallocate(*cudaContextManager.getCudaContext(), ptr);
			ptr = NULL;
		}
	}

	// Pinned Memory allocator - allocates a large block of memory and then suballocates to consumers.
	// Can only be grown using reserveAndGrow - no copy will be performed and the grow operation is most
	// likely a large allocation - think about performance.
	// Grows linearly, only possible to release all the memory at once at the end.
	// Consider this a stack-based allocator for all means.
	//
	// We use this for contact/patch/force streams.
	class PxgPinnedHostLinearMemoryAllocator : public PxUserAllocated
	{
	public:

		PxgPinnedHostLinearMemoryAllocator(PxCudaContextManager* contextManager, const PxU64 size);

		~PxgPinnedHostLinearMemoryAllocator();

		// both of these reserve* operations will invalidate all existing allocations.
		void reserve(const PxU64 size);
		void reserveAndGrow(const PxU64 size);

		void reset(); // will invalidate all allocations.
		void* allocate(const PxU64 size, const PxU64 alignment);

	private:
		void deallocate(); // will deallocate the large base allocation, not the individual chunks!

		PxCudaContext*			mCudaContext;

	public:
		PxU8*					mStart;
		PxU64					mCurrentSize;
		PxU64					mTotalSize;	
	};
}

#define PX_DEVICE_MEMORY_ALLOC(T, cudaContextManager, numElements) PxgCudaDeviceMemoryAllocate<T>(cudaContextManager, numElements, PX_FL)
#define PX_DEVICE_MEMORY_FREE(cudaContextManager, deviceBuffer) PxgCudaDeviceMemoryDeallocate(cudaContextManager, deviceBuffer)

#define PX_PINNED_MEMORY_ALLOC(T, cudaContextManager, numElements) PxgPinnedMemoryAllocate<T>(cudaContextManager, numElements, PX_FL)
#define PX_PINNED_MEMORY_FREE(cudaContextManager, ptr) PxgPinnedMemoryDeallocate(cudaContextManager, ptr)	

#endif