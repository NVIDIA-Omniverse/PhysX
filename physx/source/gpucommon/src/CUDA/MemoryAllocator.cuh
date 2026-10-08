// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CU_MEMORY_ALLOCATOR_CUH__
#define __CU_MEMORY_ALLOCATOR_CUH__

#include "cutil_math.h"
#include "stdio.h"

class ScratchMemoryAllocator
{
public:
	__device__ ScratchMemoryAllocator(uchar* mem, uint allocatedSize) : startPtr(mem),
		totalAllocatedSize(allocatedSize), currentSize(0)
	{

	}

	template <typename T>
	__device__  T* alloc(uint requestedSize)
	{
		T* ptr = reinterpret_cast<T*>(startPtr + currentSize);

		if (totalAllocatedSize < (currentSize + requestedSize))
		{
			printf("alloc out of sharedMemory !\n");
			return NULL;
		}

		currentSize += requestedSize;
		return ptr;
	}

	template <typename T>
	__device__ T* allocAligned(uint requestedSize, size_t alignment = 4)
	{
		size_t baseAddress = size_t(startPtr + currentSize);
		size_t alignedAddress = (baseAddress + size_t(alignment - 1)) & (~(size_t(alignment-1)));
		uint paddingBytes = uint(alignedAddress - baseAddress);

		const uint newRequestedSize = requestedSize + paddingBytes;
		
		if (totalAllocatedSize < (currentSize + newRequestedSize))
		{
#if 1
			printf("allocAligned out of sharedMemory allocating %i bytes!\n", requestedSize);
#endif
			return NULL;
		}

		currentSize += newRequestedSize;

		T* ptr = reinterpret_cast<T*>(alignedAddress);
		return ptr;
	}

	uchar* startPtr;
	uint totalAllocatedSize;
	uint currentSize;
	
};

class ScratchMemoryMarker
{

	uint currentSize;
	ScratchMemoryAllocator& alloc;
public:
	__device__  ScratchMemoryMarker(ScratchMemoryAllocator& allocator) : alloc(allocator)
	{
		currentSize = alloc.currentSize;
	}


	__device__  ~ScratchMemoryMarker()
	{
		alloc.currentSize = currentSize;
	}

	__device__  void reset()
	{
		alloc.currentSize = currentSize;
	}
};

template <typename Type, int SharedCapacity, int Capacity>
class HybridSharedArray
{
	Type* sharedBuffer;
	Type locBuff[Capacity - SharedCapacity];

public:

	HybridSharedArray(Type* shBuff) : sharedBuffer(shBuff)
	{
	}

	PX_FORCE_INLINE const Type& operator[] (const uint index) const { return index < SharedCapacity ? sharedBuffer[index] : locBuff[index-SharedCapacity];}
	PX_FORCE_INLINE Type& operator[] (const uint index) { return index < SharedCapacity ? sharedBuffer[index] : locBuff[index - SharedCapacity]; }
};

#endif

