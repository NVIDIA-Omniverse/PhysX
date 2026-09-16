// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxMath.h"
#include "foundation/PxIntrinsics.h"
#include "foundation/PxBitUtils.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxTempAllocator.h"

#include "FdFoundation.h"

#if PX_VC
#pragma warning(disable : 4706) // assignment within conditional expression
#endif

physx::AllocFreeTable& getTempAllocFreeTable();
physx::Mutex& getTempAllocMutex();

namespace physx
{
union PxTempAllocatorChunk
{
	PxTempAllocatorChunk* mNext; // while chunk is free
	PxU32 mIndex;           // while chunk is allocated
	PxU8 mPad[16];          // 16 byte aligned allocations
};
namespace
{
typedef PxTempAllocatorChunk Chunk;

const PxU32 sMinIndex = 8;  // 256B min
const PxU32 sMaxIndex = 17; // 128kB max
}

void* PxTempAllocator::allocate(size_t size, const char* filename, PxI32 line)
{
	if(!size)
		return 0;

	PxU32 index = PxMax(PxHighestSetBit(PxU32(size) + sizeof(Chunk) - 1), sMinIndex);

	Chunk* chunk = 0;
	if(index < sMaxIndex)
	{
		Mutex::ScopedLock lock(getTempAllocMutex());

		// find chunk up to 16x bigger than necessary
		AllocFreeTable& freeTable = getTempAllocFreeTable();
		Chunk** it = freeTable.begin() + index - sMinIndex;
		Chunk** end = PxMin(it + 3, freeTable.end());
		while(it < end && !(*it))
			++it;

		if(it < end)
		{
			// pop top off freelist
			chunk = *it;
			*it = chunk->mNext;
			index = PxU32(it - freeTable.begin() + sMinIndex);
		}
		else
			// create new chunk
			chunk = reinterpret_cast<Chunk*>(PxAllocator().allocate(size_t(2 << index), filename, line));
	}
	else
	{
		// too big for temp allocation, forward to base allocator
		chunk = reinterpret_cast<Chunk*>(PxAllocator().allocate(size + sizeof(Chunk), filename, line));
	}

	chunk->mIndex = index;
	void* ret = chunk + 1;
	PX_ASSERT((size_t(ret) & 0xf) == 0); // SDK types require at minimum 16 byte alignment.
	return ret;
}

void PxTempAllocator::deallocate(void* ptr)
{
	if(!ptr)
		return;

	Chunk* chunk = reinterpret_cast<Chunk*>(ptr) - 1;
	PxU32 index = chunk->mIndex;

	if(index >= sMaxIndex)
		return PxAllocator().deallocate(chunk);

	Mutex::ScopedLock lock(getTempAllocMutex());

	index -= sMinIndex;

	AllocFreeTable& freeTable = getTempAllocFreeTable();

	if(freeTable.size() <= index)
		freeTable.resize(index + 1);

	chunk->mNext = freeTable[index];
	freeTable[index] = chunk;
}

} // namespace physx

using namespace physx;

void deallocateTempBufferAllocations(AllocFreeTable& mTempAllocFreeTable)
{
	PxAllocator alloc;
	for(PxU32 i = 0; i < mTempAllocFreeTable.size(); ++i)
	{
		for(PxTempAllocatorChunk* ptr = mTempAllocFreeTable[i]; ptr;)
		{
			PxTempAllocatorChunk* next = ptr->mNext;
			alloc.deallocate(ptr);
			ptr = next;
		}
	}
	mTempAllocFreeTable.reset();
}
