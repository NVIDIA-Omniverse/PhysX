// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	void*	PxgPinnedMemoryAllocate(PxCudaContext& cudaContext, size_t size, PxU32 flags, const char* filename, PxI32 line);
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
	T*		PxgPinnedMemoryAllocate(PxCudaContextManager& cudaContextManager, PxU64 numElements, PxU32 flags, const char* filename, PxI32 line)
	{
		PxScopedCudaLock _lock(cudaContextManager);
		return reinterpret_cast<T*>(PxgPinnedMemoryAllocate(*cudaContextManager.getCudaContext(), numElements * sizeof(T), flags, filename, line));
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

#define PX_PINNED_MEMORY_ALLOC(T, cudaContextManager, numElements) \
	PxgPinnedMemoryAllocate<T>(cudaContextManager, numElements, CU_MEMHOSTALLOC_PORTABLE | CU_MEMHOSTALLOC_DEVICEMAP, PX_FL)

#define PX_PINNED_MEMORY_ALLOC_FLAGS(T, cudaContextManager, numElements, flags) \
	PxgPinnedMemoryAllocate<T>(cudaContextManager, numElements, flags, PX_FL)

#define PX_PINNED_MEMORY_FREE(cudaContextManager, ptr) PxgPinnedMemoryDeallocate(cudaContextManager, ptr)

#endif