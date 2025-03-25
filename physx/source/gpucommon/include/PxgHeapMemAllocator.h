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

#ifndef	PXG_HEAP_MEM_ALLOCATOR_H
#define	PXG_HEAP_MEM_ALLOCATOR_H

#include "foundation/PxAssert.h"
#include "foundation/PxBitUtils.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxPool.h"
#include "foundation/PxMutex.h"
#include "PxsHeapMemoryAllocator.h"
#include "common/PxPhysXCommonConfig.h"

#if PX_DEBUG
#include "PxgMemoryTracker.h"
#endif

namespace physx
{
	class PxsMemoryManager;

	class BlockHeader
	{
	public:
		PX_FORCE_INLINE BlockHeader(const PxU32 offset, PxU32 rootIndex) : mOffset(offset), mRootIndex(rootIndex), mPrev(NULL), mNext(NULL)
		{
		}

		PX_FORCE_INLINE void initialize(const PxU32 rootIndex, const PxU32 offset)
		{
			mOffset = offset;
			mRootIndex = rootIndex;
			mPrev = NULL;
			mNext = NULL;
		}

		PxU32 mOffset;
		PxU32 mRootIndex;
		BlockHeader* mPrev;
		BlockHeader* mNext;
	};

#define PXG_INVALID_BLOCK 0xFFFFFFFF

	class Block
	{
	public:
		PX_FORCE_INLINE Block() : mStartHeader(NULL), mEndHeader(NULL), mHeaderSizes(0) {}
		
		PX_FORCE_INLINE bool isEmpty() { return mHeaderSizes == 0; }
		void insertBlockHeader(const PxU32 rootIndex, const PxU32 offset, PxPool<BlockHeader>& pool);
		void removeBlockHeader(BlockHeader* header, PxPool<BlockHeader>& pool);
		BlockHeader* findBuddy(const PxU32 offsetToFind, const PxU32 rootIndex);
		BlockHeader* getFreeBlocks(){ return mEndHeader; }

		bool isValid();

		BlockHeader* mStartHeader;
		BlockHeader* mEndHeader;
		PxU32 mHeaderSizes;
		PxU32 mBlockSize;
		PxU32 mBlockIndex;
	};

	class AllocationValue
	{
	public:
		PX_FORCE_INLINE AllocationValue(const PxU32 blockIndex, const PxU32 rootIndex, const size_t byteSize, const int group)
			: mBlockIndex(blockIndex), mRootIndex(rootIndex), mByteSize(byteSize), mGroup(group)
		{}
		PxU32 mBlockIndex;
		PxU32 mRootIndex;
		size_t mByteSize;
		int mGroup;
	};

	struct ExceptionalAlloc
	{
		void* address;
		size_t size;
	};

	class PxgHeapMemoryAllocator : public PxsHeapMemoryAllocator
	{
	public:

		PxgHeapMemoryAllocator(const PxU32 byteSize, PxVirtualAllocatorCallback* allocator);
		~PxgHeapMemoryAllocator();
	
		void initializeBlocks(const PxU32 rootIndex);
		//return a free block index
		PxU32 getNextFreeBlock(const PxU32 blockIndex, const PxU32 allocationSize, const char* file, const int line);
	
		// PxVirtualAllocatorCallback
		virtual	void* allocate(const size_t byteSize, const int group, const char* file, const int line)	PX_OVERRIDE;
		virtual	void deallocate(void* ptr)																	PX_OVERRIDE;
		//~PxVirtualAllocatorCallback

		void deallocateDeferred(void* ptr);

		void flushDeferredDeallocs();

		PxU64 getTotalSize();
		PxsHeapStats& getHeapStats() { return mHeapStats; }

#if PX_DEBUG || PX_STOMP_ALLOCATED_MEMORY
		PxVirtualAllocatorCallback* getAllocator() { return mAllocator; } //Used for memcheck support
#endif

	private:

		PxHashMap<void*, AllocationValue> mHashMap;//this is used to look up where the block is
		PxArray<Block> mBlocks; //this is used to store fix size slots 
		PxVirtualAllocatorCallback* mAllocator;
		PxArray<void*> mRoots;
		PxArray<ExceptionalAlloc> mExceptionalAllocs;
		PxArray<void*> deferredDeallocs;
		PxU32 mAllocationSize;
		PxU32 mBitfield;
		PxU64 mTotalMem;
		PxsHeapStats mHeapStats;

		PxPool<BlockHeader> mBlockHeaderPool;

		PxMutex mMutex;

#if PX_DEBUG
		MemTracker mMemTracker;
#endif

		PX_NOCOPY(PxgHeapMemoryAllocator)
	};

	class PxgHeapMemoryAllocatorManager : public PxsHeapMemoryAllocatorManager
	{ 
	public:
		PxgHeapMemoryAllocatorManager(PxU32 heapCapacity, PxsMemoryManager* memoryManager);

		virtual ~PxgHeapMemoryAllocatorManager();

		// PxsHeapMemoryAllocatorManager
		virtual PxU64			getDeviceMemorySize()	const	PX_OVERRIDE PX_FINAL;
		virtual PxsHeapStats	getDeviceHeapStats()	const	PX_OVERRIDE PX_FINAL;
		virtual void			flushDeferredDeallocs()			PX_OVERRIDE PX_FINAL;
		//~PxsHeapMemoryAllocatorManager

		PxgHeapMemoryAllocator*	mDeviceMemoryAllocators;
	};
}

#endif
