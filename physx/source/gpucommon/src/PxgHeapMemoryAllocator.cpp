// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgHeapMemAllocator.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxErrors.h"
#include "foundation/PxMath.h"
#include "common/PxProfileZone.h"
#include "PxgMemoryManager.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaTypes.h"

using namespace physx;

#define EXCEPTIONAL_ALLOC_FACTOR 2

//### DEFENSIVE - OMPE-104415
//PT: PxHashMap::insert() does not overwrite an existing key - it leaves the old value in place and returns
//false. An address that is already live therefore cannot be recorded a second time, and handing it out anyway
//would give two owners the same buffer: the first deallocate() releases it while the second is still reading
//and writing it, and the second deallocate() then finds no entry at all - with PX_ASSERT compiled out in every
//configuration except Debug, all it can do is report the block as unknown and abandon it (see deallocate()
//below). So the allocation is refused instead, and reported here - the only point at which the offending block
//is still known. blockIndex is PXG_INVALID_BLOCK for the exceptional-allocation path.
//The context goes into abort mode with it, because that is what makes the returned NULL harmless: callers of
//allocate() do not test it (PxgCudaBuffer::allocate stores it and reports the size it asked for), and abort
//mode is what short-circuits every CUDA call that would then run on a device address of 0. This mirrors the
//out-of-memory path in PxgCudaDeviceMemoryAllocate, until now the only source of NULL from this heap.
static void reportDuplicateAllocation(PxgCudaAllocatorCallbackBase* allocator, const void* address, PxU32 blockIndex)
{
	PX_ASSERT(0);
	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
		"PxgHeapMemoryAllocator: block %u handed out address %p, which is already a live allocation. Heap bookkeeping is corrupt.",
		blockIndex, address);

	allocator->mCudaContext.setAbortMode(true);
}

bool Block::isValid()
{
	BlockHeader* current = mStartHeader;
	while (current)
	{
		BlockHeader* next = current->mNext;
		if (next)
		{
			if ((current->mRootIndex > next->mRootIndex) || ((current->mRootIndex == next->mRootIndex) && (current->mOffset >= next->mOffset)))
			{
				return false;
			}
			else
			{
				current = next;
				next = current->mNext;
			}
		}
		else
		{
			current = NULL;
		}
	}
	return true;
}

void Block::insertBlockHeader(const PxU32 rootIndex, const PxU32 offset, PxPool<BlockHeader>& pool)
{
	PX_PROFILE_ZONE("Block::insertBlockHeader", 0);

	BlockHeader* newHeader = pool.allocate();
	newHeader->initialize(rootIndex, offset);

	if (mStartHeader)
	{
		BlockHeader* header = mStartHeader;

		while (header && ((header->mRootIndex < rootIndex) || (header->mRootIndex == rootIndex && header->mOffset < offset)))
		{
			header = header->mNext;
		}

		//if we found a header, we need to insert a new header in front of the found header
		if (header)
		{
			BlockHeader* prevHeader = header->mPrev;

			newHeader->mNext = header;
			newHeader->mPrev = prevHeader;

			if (prevHeader)
			{
				prevHeader->mNext = newHeader;
			}
			else
			{
				mStartHeader = newHeader;
			}

			header->mPrev = newHeader;
		}
		else
		{
			//if we didn't found an appropriated header, we need to insert this new header at the end of the linked list
			mEndHeader->mNext = newHeader;
			newHeader->mPrev = mEndHeader;
			mEndHeader = newHeader;
		}
	}
	else
	{
		mStartHeader = newHeader;
		mEndHeader = newHeader;
	}

	PX_ASSERT(isValid());
	mHeaderSizes++;
}

void Block::removeBlockHeader(BlockHeader* header, PxPool<BlockHeader>& pool)
{
	BlockHeader* mPrev = header->mPrev;
	BlockHeader* mNext = header->mNext;

	if (mPrev)
		mPrev->mNext = mNext;
	else
		mStartHeader = mNext;

	if (mNext)
		mNext->mPrev = mPrev;
	else
		mEndHeader = mPrev;

	pool.deallocate(header);

	mHeaderSizes--;
}

BlockHeader* Block::findBuddy(const PxU32 offsetToFind, const PxU32 rootIndex)
{
	BlockHeader* header = mStartHeader;

	while (header && (header->mOffset != offsetToFind || header->mRootIndex != rootIndex))
	{
		header = header->mNext;
	}

	return header;
}

PxgHeapMemoryAllocator::PxgHeapMemoryAllocator(const PxU32 byteSize, PxgCudaAllocatorCallbackBase& allocator)
: mBlockHeaderPool(PxAllocatorTraits<BlockHeader>::Type(), 128)
{
	if(byteSize > 0)
	{
		PX_ASSERT(PxIsPowerOfTwo(byteSize));
		PX_ASSERT(byteSize >= 128);
	}
	mAllocationSize = byteSize;
	mAllocator = &allocator;

	PX_PROFILE_ZONE("PxgHeapMemoryAllocator::initialization", 0);
	void* memory = NULL;
	if(byteSize)
	{
		memory = mAllocator->allocate(mAllocationSize, 0, PX_FL);
	}
	
	// AD: the allocation above can fail.
	if (memory)
	{
		mRoots.pushBack(memory);
		mTotalMem = mAllocationSize;
		initializeBlocks(0);
	}
	else
	{
		mTotalMem = 0;
		mBitfield = 0;
	}

}

PxgHeapMemoryAllocator::~PxgHeapMemoryAllocator()
{
	if (mAllocator)
	{
		for (PxU32 i = 0; i < mRoots.size(); ++i)
		{
			mAllocator->deallocate(mRoots[i]);
		}
		for (PxU32 i = 0; i < mExceptionalAllocs.size(); ++i)
		{
			if(mExceptionalAllocs[i].address)
				mAllocator->deallocate(mExceptionalAllocs[i].address);
		}
		mRoots.clear();
		mExceptionalAllocs.clear();
		mAllocator = NULL;
	}
}

void PxgHeapMemoryAllocator::initializeBlocks(const PxU32 rootIndex)
{
	//calculate how many slots do we need, the smallest blockSize in a slot will be 128 byte. the 120 = pow(2, 7)
	const PxU32 highestBit = PxHighestSetBit(mAllocationSize) + 1 - 7;

	mBlocks.resize(highestBit);

	//initialize all blocks
	for (PxU32 i = 0; i < highestBit; ++i)
	{
		mBlocks[i].mBlockSize = 1u << (i + 7u);
		mBlocks[i].mBlockIndex = i;
	}

	//all blocks are empty beside the highestBit
	mBitfield = (1u << (highestBit - 1u));

	mBlocks[highestBit - 1].insertBlockHeader(rootIndex, 0, mBlockHeaderPool);
}

PxU32 PxgHeapMemoryAllocator::getNextFreeBlock(const PxU32 blockIndex, const PxU32 allocationSize, const char* file, const int line)
{
	PX_ASSERT(PxIsPowerOfTwo(allocationSize));
	const PxU32 bits = mBitfield & (~((1 << blockIndex) - 1));
	//no bigger slot avaiable
	if (bits == 0)
	{
		PX_PROFILE_ZONE("PxgHeapMemoryAllocator::getNextFreeBlock", 0);
		//we can't find any free blocks, we allocate more memory
		const PxU32 maxAllocationSize = PxMax(allocationSize, mAllocationSize);
		void* memorys = mAllocator->allocate(maxAllocationSize, 0, file, line);
		if (!memorys)
			return PXG_INVALID_BLOCK;

		mRoots.pushBack(memorys);

		mTotalMem += maxAllocationSize;

		const PxU32 newBlockIndex = PxU32(PxMax(PxI32(PxHighestSetBit(maxAllocationSize)) - 7, 0));

		//if the allocationSize is bigger than the default allocation size(mAllocationSize), we need to increase
		//the block slots.
		//PT: the new root goes into the slot for maxAllocationSize (newBlockIndex), which is not necessarily the
		//slot the caller asked for (blockIndex): maxAllocationSize is PxMax(allocationSize, mAllocationSize), so
		//newBlockIndex > blockIndex whenever allocationSize < mAllocationSize. We must therefore size the array
		//for the slot we are about to touch, not just for the requested one. Sizing for blockIndex alone made
		//mBlocks[newBlockIndex] below read past the end of the array whenever mBlocks had not been sized for
		//mAllocationSize - which is the case when the initial allocation in the ctor failed and
		//initializeBlocks() was skipped, leaving mBlocks empty (OMPE-100438).
		const PxU32 requiredSize = PxMax(blockIndex, newBlockIndex) + 1;
		if (requiredSize > mBlocks.size())
		{
			const PxU32 oldSize = mBlocks.size();
			mBlocks.resize(requiredSize);

			for (PxU32 i = oldSize; i < requiredSize; ++i)
			{
				//blockSize is power of two
				mBlocks[i].mBlockSize = 1u << (i + 7u);
				mBlocks[i].mBlockIndex = i;
			}
		}

		const PxU32 rootIndex = mRoots.size() - 1;
		Block* block = &mBlocks[newBlockIndex];

		block->insertBlockHeader(rootIndex, 0, mBlockHeaderPool);
		mBitfield = mBitfield | (1u << newBlockIndex);

		return newBlockIndex;
	}
	else
	{
		return PxLowestSetBit(bits);
	}
}

void* PxgHeapMemoryAllocator::allocate(const size_t byteSize, const int group, const char* file, const int line)
{
	if (byteSize == 0)
		return NULL;

	PX_PROFILE_ZONE("PxgHeapMemoryAllocator::allocate", 0);

	PxMutex::ScopedLock myLock(mMutex);

	PX_ASSERT(group >= 0 && group < PxsHeapStats::eHEAPSTATS_COUNT);
	mHeapStats.stats[group] += byteSize;

	if ((byteSize * EXCEPTIONAL_ALLOC_FACTOR) > mAllocationSize)
	{
		PX_PROFILE_ZONE("PxgHeapMemoryAllocator::exceptionalAlloc", 0);
		//We are allocating over half the size of a page. In this case, we'll use a whole page so we might
		//as well just allocate an exceptional block for this using the built-in allocator...
		void* memorys = mAllocator->allocate(byteSize, 0, file, line);
		if (!memorys)
			return NULL;

		mTotalMem += byteSize;

		PxU32 index = mExceptionalAllocs.size();
		ExceptionalAlloc alloc;
		alloc.address = memorys;
		alloc.size = byteSize;
		mExceptionalAllocs.pushBack(alloc);

		if(!mHashMap.insert(memorys, AllocationValue(PXG_INVALID_BLOCK, index, byteSize, group)))
		{
			reportDuplicateAllocation(mAllocator, memorys, PXG_INVALID_BLOCK);

			//PT: the entry pushed just above is unreachable from the map, which still names the first owner's
			//slot, and mExceptionalAllocs is never compacted - so the destructor would free this address a
			//second time. It is provably the last element, the array only ever being appended to under mMutex.
			//The address itself must not be freed here: it may well still belong to that first owner.
			mExceptionalAllocs.popBack();
			mTotalMem -= byteSize;
			mHeapStats.stats[group] -= byteSize;
			return NULL;
		}

#if PX_DEBUG
		mMemTracker.registerMemory(reinterpret_cast<void*>(memorys), true, byteSize, file, line);
#endif

		return memorys;
	}

	const PxU32 maxSize = PxIsPowerOfTwo(PxU32(byteSize)) ? PxU32(byteSize) : PxNextPowerOfTwo(PxU32(byteSize));

	//get the slot index
	const PxU32 blockIndex = PxU32(PxMax(PxI32(PxHighestSetBit(maxSize)) - 7, 0));

	//Reserve enough memory for this block if it is needed
	const PxU32 freeBlockIndex = getNextFreeBlock(blockIndex, maxSize, file, line);

	// if the allocation of the free block failed, make sure we pass the error along.
	if (freeBlockIndex == PXG_INVALID_BLOCK)
		return NULL;

	if (mBlocks[blockIndex].isEmpty())
	{
		//We don't have a slot of this size, so recursively split higher blocks until we get to the desired size.
		//The above getNextFreeBlock(...) call will ensure that there is a suitable block to use.
		Block& tBlock = mBlocks[blockIndex];

		Block* freeBlock = &mBlocks[freeBlockIndex];

		PxU32 cBlockSize = freeBlock->mBlockSize;

		//remove the last free header

		BlockHeader* newBlockHeader = freeBlock->getFreeBlocks();

		const PxU32 rootIndex = newBlockHeader->mRootIndex;
		const PxU32 offset = newBlockHeader->mOffset;

		freeBlock->removeBlockHeader(newBlockHeader, mBlockHeaderPool);

		if (freeBlock->isEmpty())
		{
			mBitfield = mBitfield & (~(1u << freeBlockIndex));
		}

		void* freeAddress = reinterpret_cast<void*>(reinterpret_cast<PxU8*>(mRoots[rootIndex]) + offset);
		PX_ASSERT(!mHashMap.find(freeAddress));

		mBlocks[blockIndex].insertBlockHeader(rootIndex, tBlock.mBlockSize + offset, mBlockHeaderPool);

		mBitfield = mBitfield | (1u << blockIndex);

		//PT: the address may already be live, which means the heap bookkeeping is corrupt. The allocation is
		//refused, but only after the split below: that loop is what puts the rest of the block being carved up
		//back into the free lists, and skipping it would strand everything above the buddy inserted just now -
		//up to a whole heap page when this request is the one that allocated a fresh root.
		const bool duplicateAddress = !mHashMap.insert(freeAddress, AllocationValue(blockIndex, rootIndex, byteSize, group));

		//recursively split blocks
		PxU32 cOffset = offset;
		PxU32 cBlockIndex = freeBlock->mBlockIndex;

		const PxU32 tBlockSize = tBlock.mBlockSize << 1;

		while (cBlockSize > tBlockSize)
		{
			cBlockSize = cBlockSize >> 1;
			cOffset = cBlockSize + offset;
			cBlockIndex = cBlockIndex - 1;
			mBlocks[cBlockIndex].insertBlockHeader(rootIndex, cOffset, mBlockHeaderPool);
			mBitfield = mBitfield | (1u << cBlockIndex);
		}

		if(duplicateAddress)
		{
			reportDuplicateAllocation(mAllocator, freeAddress, blockIndex);

			//PT: this one slot has no free-list header left and is not worth stitching back, so it stays out of
			//circulation. Leaking it is a far better outcome than handing the address to a second owner.
			mHeapStats.stats[group] -= byteSize;
			return NULL;
		}

#if PX_DEBUG
		mMemTracker.registerMemory(reinterpret_cast<void*>(freeAddress), true, byteSize, file, line);
#endif

		return freeAddress;
	}
	else
	{
		Block& tBlock = mBlocks[blockIndex];

		BlockHeader* newHeader = tBlock.getFreeBlocks();
		const PxU32 rootIndex = newHeader->mRootIndex;
		const PxU32 offset = newHeader->mOffset;

		tBlock.removeBlockHeader(newHeader, mBlockHeaderPool);
		if (tBlock.isEmpty())
		{
			mBitfield = mBitfield & (~(1u << blockIndex));
		}
		void* address = reinterpret_cast<void*>(reinterpret_cast<PxU8*>(mRoots[rootIndex]) + offset);
		PX_ASSERT(!mHashMap.find(address));

		if(!mHashMap.insert(address, AllocationValue(blockIndex, rootIndex, byteSize, group)))
		{
			reportDuplicateAllocation(mAllocator, address, blockIndex);

			//PT: as above, the slot stays out of circulation rather than being handed to a second owner. Nothing
			//was carved up on this path, so that single slot is all it abandons.
			mHeapStats.stats[group] -= byteSize;
			return NULL;
		}

#if PX_DEBUG
		mMemTracker.registerMemory(reinterpret_cast<void*>(address), true, byteSize, file, line);
#endif

		return address;
	}
}
	
void PxgHeapMemoryAllocator::deallocateDeferred(void* ptr)
{
	deferredDeallocs.pushBack(ptr);
}

void PxgHeapMemoryAllocator::flushDeferredDeallocs()
{
	for (PxU32 i = 0; i < deferredDeallocs.size(); ++i)
		deallocate(deferredDeallocs[i]);
	deferredDeallocs.forceSize_Unsafe(0);
}

void PxgHeapMemoryAllocator::deallocate(void* ptr)
{
	PX_PROFILE_ZONE("PxgHeapMemoryAllocator::deallocate", 0);
	if (ptr == NULL)
		return;

	PxMutex::ScopedLock myLock(mMutex);

	//found the block index
	const PxHashMap<void*, AllocationValue>::Entry* entry = mHashMap.find(ptr);
	if (!entry)
	{
		//This heap has no record of the block: the pointer was never handed out by it, or it has already been
		//freed, or the heap's own bookkeeping is corrupt (the same address registered twice).
		//deallocate() is only reachable from inside the SDK, so every one of those is a library fault rather
		//than a caller one. Releasing the block again would corrupt the bookkeeping further, so report it and
		//bail out instead of running into a null dereference.
		//In the corrupt case this deliberately abandons the block: with no map entry its size and group are
		//unknown, so it stays charged (surfacing as inflated PxSimulationStatistics::gpuMemHeap*) and never
		//returns to the free list. Guessing either value would corrupt the heap further. The other two cases
		//have nothing to release: a foreign block was never charged here, and an already-freed one was
		//un-charged by its first release.
		// ### DEFENSIVE (OMPE-104415/NvBugs 6558251)
		PX_ASSERT(0);
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
			"PxgHeapMemoryAllocator::deallocate: %p is not a live allocation of this heap - foreign or "
			"already-freed pointer, or corrupt heap bookkeeping; ignoring it.", ptr);
		return;
	}

	const AllocationValue value = entry->second;

	mHeapStats.stats[value.mGroup] -= value.mByteSize;

	mHashMap.erase(ptr);

	if (value.mBlockIndex == PXG_INVALID_BLOCK)
	{
		//Exceptional allocation, we just release it back to the CUDA allocator...

		mTotalMem -= mExceptionalAllocs[value.mRootIndex].size;

		mExceptionalAllocs[value.mRootIndex].address = NULL;
		mExceptionalAllocs[value.mRootIndex].size = 0;
		mAllocator->deallocate(ptr);

#if PX_DEBUG
		mMemTracker.unregisterMemory(ptr, true);
#endif
		return;
	}

	const PxU32 rootIndex = value.mRootIndex;
	PxU32 blockIndex = value.mBlockIndex;

	PxU32 offset = PxU32(reinterpret_cast<PxU8*>(ptr)-reinterpret_cast<PxU8*>(mRoots[rootIndex]));

	Block* block = &mBlocks[blockIndex];

	do
	{
		const PxU32 offsetToFind = (((offset / block->mBlockSize) & 1) == 0) ? offset + block->mBlockSize : offset - block->mBlockSize;

		BlockHeader* buddyHeader = block->findBuddy(offsetToFind, rootIndex);

		if (buddyHeader)
		{
			//current block need to remove the merged free header
			block->removeBlockHeader(buddyHeader, mBlockHeaderPool);

			if (block->isEmpty())
			{
				mBitfield = mBitfield & (~(1u << blockIndex));
			}

			offset = PxMin(offsetToFind, offset);

			blockIndex = blockIndex + 1;

			if (blockIndex < mBlocks.size())
			{
				block = &mBlocks[blockIndex];
			}
			else
			{
				block->insertBlockHeader(rootIndex, offset, mBlockHeaderPool);
				mBitfield = mBitfield | (1u << blockIndex);
				break;
			}
		}
		else
		{
			PX_ASSERT(buddyHeader == NULL);
			//just put it back to the block
			block->insertBlockHeader(rootIndex, offset, mBlockHeaderPool);
			mBitfield = mBitfield | (1u << blockIndex);
			break;
		}
	} while (1);

#if PX_DEBUG
	mMemTracker.unregisterMemory(ptr, true);
#endif
}

PxU64 PxgHeapMemoryAllocator::getTotalSize()
{
	return mTotalMem;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxgHeapMemoryAllocatorManager::PxgHeapMemoryAllocatorManager(PxU32 heapCapacity, PxgMemoryManager& memoryManager)
{
	mDeviceMemoryAllocator = PX_NEW(PxgHeapMemoryAllocator)(heapCapacity, *memoryManager.getCudaDeviceMemoryAllocator());

	const bool debugMode = (heapCapacity == 0);
	if(debugMode) 
	{
		const PxU32 defaultFlags = CU_MEMHOSTALLOC_PORTABLE;
		const PxU32 mappedFlags = CU_MEMHOSTALLOC_PORTABLE | CU_MEMHOSTALLOC_DEVICEMAP;
		mPinnedHostMemoryAllocator = PX_NEW(PxgHeapMemoryAllocator)(heapCapacity, *memoryManager.getCudaHostMemoryAllocator(defaultFlags));
		mPinnedHostMappedMemoryAllocator = PX_NEW(PxgHeapMemoryAllocator)(heapCapacity, *memoryManager.getCudaHostMemoryAllocator(mappedFlags));
	}
	else
	{
		const PxU32 flags = CU_MEMHOSTALLOC_PORTABLE | CU_MEMHOSTALLOC_DEVICEMAP;
		mPinnedHostMemoryAllocator = PX_NEW(PxgHeapMemoryAllocator)(heapCapacity, *memoryManager.getCudaHostMemoryAllocator(flags));
		mPinnedHostMappedMemoryAllocator = mPinnedHostMemoryAllocator;
	}
}

PxgHeapMemoryAllocatorManager::~PxgHeapMemoryAllocatorManager()
{
	PX_DELETE(mDeviceMemoryAllocator);

	if(mPinnedHostMappedMemoryAllocator != mPinnedHostMemoryAllocator)
	{
		PX_DELETE(mPinnedHostMappedMemoryAllocator);
	}
	
	PX_DELETE(mPinnedHostMemoryAllocator);
}

PxU64 PxgHeapMemoryAllocatorManager::getDeviceMemorySize() const
{
	return mDeviceMemoryAllocator ? mDeviceMemoryAllocator->getTotalSize() : 0;
}

PxsHeapStats PxgHeapMemoryAllocatorManager::getDeviceHeapStats() const
{
	if(mDeviceMemoryAllocator)
		return mDeviceMemoryAllocator->getHeapStats();
	else
		return PxsHeapStats();
}

void PxgHeapMemoryAllocatorManager::flushDeferredDeallocs()
{
	if (mDeviceMemoryAllocator) // this should actually never be null...
		mDeviceMemoryAllocator->flushDeferredDeallocs();
}
