// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_SCRATCH_ALLOCATOR_H
#define PXC_SCRATCH_ALLOCATOR_H

#include "foundation/PxAssert.h"
#include "PxPhysXConfig.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
class PxcScratchAllocator : public PxUserAllocated
{
	PX_NOCOPY(PxcScratchAllocator)
public:
	PxcScratchAllocator() : mStack("PxcScratchAllocator"), mStart(NULL), mSize(0)
	{
		mStack.reserve(64);
		mStack.pushBack(0);
	}

	void setBlock(void* addr, PxU32 size)
	{
		PX_ASSERT(!(size&15));

		// if the stack is not empty then some scratch memory was not freed on the previous frame. That's 
		// likely indicative of a problem, because when the scratch block is too small the memory will have
		// come from the heap

		PX_ASSERT(mStack.size()==1);
		mStack.popBack();

		mStart = reinterpret_cast<PxU8*>(addr);
		mSize = size;
		mStack.pushBack(mStart + size);
	}

	void* allocAll(PxU32& size)
	{
		PxMutex::ScopedLock lock(mLock);
		PX_ASSERT(mStack.size()>0);
		size = PxU32(mStack.back()-mStart);

		if(size==0)
			return NULL;

		mStack.pushBack(mStart);
		return mStart;
	}

	void* alloc(PxU32 requestedSize, bool fallBackToHeap = false)
	{
		requestedSize = (requestedSize+15)&~15;

		PxMutex::ScopedLock lock(mLock);
		PX_ASSERT(mStack.size()>=1);

		PxU8* top = mStack.back();

		if(top - mStart >= ptrdiff_t(requestedSize))
		{
			PxU8* addr = top - requestedSize;
			mStack.pushBack(addr);
			return addr;
		}

		if(!fallBackToHeap)
			return NULL;

		return PX_ALLOC(requestedSize, "Scratch Block Fallback");
	}

	void free(void* addr)
	{
		PX_ASSERT(addr!=NULL);
		if(!isScratchAddr(addr))
		{
			PX_FREE(addr);
			return;
		}

		PxMutex::ScopedLock lock(mLock);
		PX_ASSERT(mStack.size()>1);

		PxU32 i=mStack.size()-1;		
		while(mStack[i]<addr)
			i--;

		PX_ASSERT(mStack[i]==addr);
		mStack.remove(i);
	}

	bool isScratchAddr(void* addr) const
	{
		PxU8* a = reinterpret_cast<PxU8*>(addr);
		return a>= mStart && a<mStart+mSize;
	}

private:
	PxMutex				mLock;
	PxArray<PxU8*>		mStack;
	PxU8*				mStart;
	PxU32				mSize;
};

}

#endif
