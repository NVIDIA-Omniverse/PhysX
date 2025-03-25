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

#include "PxgHeapMemAllocator.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxMath.h"
#include "common/PxProfileZone.h"
#include "PxsMemoryManager.h"

using namespace physx;

#define EXCEPTIONAL_ALLOC_FACTOR 2

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

PxgHeapMemoryAllocator::PxgHeapMemoryAllocator(const PxU32 byteSize, PxVirtualAllocatorCallback* allocator) : mBlockHeaderPool(PxAllocatorTraits<BlockHeader>::Type(), 128)
{
	PX_ASSERT(PxIsPowerOfTwo(byteSize));
	PX_ASSERT(byteSize >= 128);
	mAllocationSize = byteSize;
	mAllocator = allocator;

	PX_PROFILE_ZONE("PxgHeapMemoryAllocator::initialization", 0);
	void* memory = mAllocator->allocate(mAllocationSize, 0, PX_FL);
	
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

		//if the allocationSize is bigger than the default allocation size(mAllocationSize), we need to increase
		//the block slots
		if (blockIndex >= mBlocks.size())
		{
			PxU32 oldSize = mBlocks.size();
			mBlocks.resize(blockIndex + 1);

			for (PxU32 i = oldSize; i <= blockIndex; ++i)
			{
				//blockSize is power of two
				mBlocks[i].mBlockSize = 1u << (i + 7u);
				mBlocks[i].mBlockIndex = i;
			}
		}

		const PxU32 newBlockIndex = PxU32(PxMax(PxI32(PxHighestSetBit(maxAllocationSize)) - 7, 0));
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

		mHashMap.insert(memorys, AllocationValue(PXG_INVALID_BLOCK, index, byteSize, group));

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

		mHashMap.insert(freeAddress, AllocationValue(blockIndex, rootIndex, byteSize, group));

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

		mHashMap.insert(address, AllocationValue(blockIndex, rootIndex, byteSize, group));

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

	PX_ASSERT(mHashMap.find(ptr));
	//found the block index
	AllocationValue value = mHashMap.find(ptr)->second;

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

PxgHeapMemoryAllocatorManager::PxgHeapMemoryAllocatorManager(PxU32 heapCapacity, PxsMemoryManager* memoryManager)
{
	mDeviceMemoryAllocators = PX_NEW(PxgHeapMemoryAllocator)(heapCapacity, memoryManager->getDeviceMemoryAllocator());
	mMappedMemoryAllocators = PX_NEW(PxgHeapMemoryAllocator)(heapCapacity, memoryManager->getHostMemoryAllocator());
}

PxgHeapMemoryAllocatorManager::~PxgHeapMemoryAllocatorManager()
{
	PX_DELETE(mDeviceMemoryAllocators);
	PX_DELETE(mMappedMemoryAllocators);
}

PxU64 PxgHeapMemoryAllocatorManager::getDeviceMemorySize() const
{
	return mDeviceMemoryAllocators ? mDeviceMemoryAllocators->getTotalSize() : 0;
}

PxsHeapStats PxgHeapMemoryAllocatorManager::getDeviceHeapStats() const
{
	if(mDeviceMemoryAllocators)
		return mDeviceMemoryAllocators->getHeapStats();
	else
		return PxsHeapStats();
}

void PxgHeapMemoryAllocatorManager::flushDeferredDeallocs()
{
	if (mDeviceMemoryAllocators) // this should actually never be null...
		mDeviceMemoryAllocators->flushDeferredDeallocs();
}
