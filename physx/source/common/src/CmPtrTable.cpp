// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "foundation/PxBitUtils.h"
#include "CmPtrTable.h"
#include "CmUtils.h"

using namespace physx;
using namespace Cm;

PtrTable::PtrTable() :
	mList		(NULL),
	mCount		(0),
	mOwnsMemory	(true),
	mBufferUsed	(false)
{
}

PtrTable::~PtrTable()
{
	PX_ASSERT(mOwnsMemory);
	PX_ASSERT(mCount == 0);
	PX_ASSERT(mList == NULL);
}

void PtrTable::clear(PtrTableStorageManager& sm)
{
	if(mOwnsMemory && mCount>1)
	{
		const PxU32 implicitCapacity = PxNextPowerOfTwo(PxU32(mCount)-1);
		sm.deallocate(mList, implicitCapacity);
	}

	mList = NULL;
	mOwnsMemory = true;
	mCount = 0;
}

PxU32 PtrTable::find(const void* ptr) const
{
	const PxU32 nbPtrs = mCount;
	void*const * PX_RESTRICT ptrs = getPtrs();

	for(PxU32 i=0; i<nbPtrs; i++)
	{
		if(ptrs[i] == ptr)
			return i;
	}
	return 0xffffffff;
}

void PtrTable::exportExtraData(PxSerializationContext& stream)
{
	if(mCount>1)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mList, sizeof(void*)*mCount);
	}
}

void PtrTable::importExtraData(PxDeserializationContext& context)
{
	if(mCount>1)
		mList = context.readExtraData<void*, PX_SERIAL_ALIGN>(mCount);
}

void PtrTable::realloc(PxU32 oldCapacity, PxU32 newCapacity, PtrTableStorageManager& sm)
{
	PX_ASSERT((mOwnsMemory && oldCapacity) || (!mOwnsMemory && oldCapacity == 0));
	PX_ASSERT(newCapacity);

	if(mOwnsMemory && sm.canReuse(oldCapacity, newCapacity))
		return;

	void** newMem = sm.allocate(newCapacity);
	PxMemCopy(newMem, mList, mCount * sizeof(void*));

	if(mOwnsMemory)
		sm.deallocate(mList, oldCapacity);

	mList = newMem;
	mOwnsMemory = true;
}

void PtrTable::add(void* ptr, PtrTableStorageManager& sm)
{
	if(mCount == 0)										// 0 -> 1, easy case
	{
		PX_ASSERT(mOwnsMemory);
		PX_ASSERT(mList == NULL);
		PX_ASSERT(!mBufferUsed);
		mSingle = ptr;
		mCount = 1;
		mBufferUsed = true;
		return;
	}
	
	if(mCount == 1)										// 1 -> 2, easy case
	{
		PX_ASSERT(mOwnsMemory);
		PX_ASSERT(mBufferUsed);

		void* single = mSingle;
		mList = sm.allocate(2);
		mList[0] = single;
		mBufferUsed = false;
		mOwnsMemory = true;
	} 
	else 
	{
		PX_ASSERT(!mBufferUsed);

		if(!mOwnsMemory)										// don't own the memory, must always alloc
			realloc(0, PxNextPowerOfTwo((uint32_t)mCount), sm);	// we're guaranteed nextPowerOfTwo(x) > x

		else if(PxIsPowerOfTwo(mCount))					// count is at implicit capacity, so realloc
			realloc(mCount, PxU32(mCount)*2, sm);		// ... to next higher power of 2

		PX_ASSERT(mOwnsMemory);
	}

	mList[mCount++] = ptr;
}

void PtrTable::replaceWithLast(PxU32 index, PtrTableStorageManager& sm)
{
	PX_ASSERT(mCount!=0);

	if(mCount == 1)												// 1 -> 0 easy case
	{
		PX_ASSERT(mOwnsMemory);
		PX_ASSERT(mBufferUsed);

		mList = NULL;
		mCount = 0;
		mBufferUsed = false;
	}
	else if(mCount == 2)										// 2 -> 1 easy case
	{
		PX_ASSERT(!mBufferUsed);
		void* ptr = mList[1-index];
		if(mOwnsMemory)
			sm.deallocate(mList, 2);
		mSingle = ptr;
		mCount = 1;
		mBufferUsed = true;
		mOwnsMemory = true;
	} 
	else
	{
		PX_ASSERT(!mBufferUsed);

		mList[index] = mList[--mCount];							// remove before adjusting memory

		if(!mOwnsMemory)										// don't own the memory, must alloc
			realloc(0, PxNextPowerOfTwo(PxU32(mCount)-1), sm);	// if currently a power of 2, don't jump to the next one

		else if(PxIsPowerOfTwo(mCount))							// own the memory, and implicit capacity requires that we downsize
			realloc(PxU32(mCount)*2, PxU32(mCount), sm);		// ... from the next power of 2, which was the old implicit capacity

		PX_ASSERT(mOwnsMemory);
	}
}

