// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_BLOCK_ARRAY_H
#define CM_BLOCK_ARRAY_H

#include "foundation/PxAssert.h"
#include "foundation/PxMath.h"
#include "foundation/PxMemory.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxIntrinsics.h"
#include "foundation/PxArray.h"

namespace physx
{
namespace Cm
{

template <typename T, PxU32 SlabSize = 4096>
class BlockArray
{
	PxArray<T*> mBlocks;
	PxU32 mSize;
	PxU32 mCapacity;

public:

	BlockArray() : mSize(0), mCapacity(0)
	{
	}

	~BlockArray()
	{
		for (PxU32 a = 0; a < mBlocks.size(); ++a)
		{
			for (PxU32 i = 0; i < SlabSize; ++i)
			{
				mBlocks[a][i].~T();
			}
			PX_FREE(mBlocks[a]);
		}
		mBlocks.resize(0);
	}

	PX_NOINLINE void reserve(PxU32 capacity)
	{
		if (capacity > mCapacity)
		{
			PxU32 nbSlabsRequired = (capacity + SlabSize - 1) / SlabSize;

			PxU32 nbSlabsToAllocate = nbSlabsRequired - mBlocks.size();

			mCapacity += nbSlabsToAllocate * SlabSize;

			for (PxU32 a = 0; a < nbSlabsToAllocate; ++a)
			{
				T* ts = reinterpret_cast<T*>(PX_ALLOC(sizeof(T) * SlabSize, "BlockArray"));
				for(PxU32 i = 0; i < SlabSize; ++i)
					PX_PLACEMENT_NEW(ts+i, T)();
				mBlocks.pushBack(ts);
			}
		}
	}

	PX_NOINLINE void resize(PxU32 size)
	{
		if(size != mSize)
		{
			reserve(size);
			for (PxU32 a = mSize; a < size; ++a)
			{
				mBlocks[a / SlabSize][a&(SlabSize - 1)].~T();
				mBlocks[a / SlabSize][a&(SlabSize - 1)] = T();
			}
			mSize = size;
		}
	}

	void forceSize_Unsafe(PxU32 size)
	{
		PX_ASSERT(size <= mCapacity);
		mSize = size;
	}

	void remove(PxU32 idx)
	{
		PX_ASSERT(idx < mSize);
		for (PxU32 a = idx; a < mSize; ++a)
		{
			mBlocks[a / SlabSize][a&(SlabSize-1)] = mBlocks[(a + 1) / SlabSize][(a + 1) &(SlabSize-1)];
		}

		mSize--;
		mBlocks[mSize / SlabSize][mSize&(SlabSize - 1)].~T();
	}

	void replaceWithLast(PxU32 idx)
	{
		PX_ASSERT(idx < mSize);
		--mSize;
		mBlocks[idx / SlabSize][idx%SlabSize] = mBlocks[mSize / SlabSize][mSize%SlabSize];
	}

	T& operator [] (const PxU32 idx)
	{
		PX_ASSERT(idx < mSize);

		return mBlocks[idx / SlabSize][idx%SlabSize];
	}

	const T& operator [] (const PxU32 idx) const
	{
		PX_ASSERT(idx < mSize);

		return mBlocks[idx / SlabSize][idx%SlabSize];
	}

	void pushBack(const T& item)
	{
		reserve(mSize + 1);
		mBlocks[mSize / SlabSize][mSize%SlabSize] = item;
		mSize++;
	}

	PxU32 capacity() const { return mCapacity; }

	PxU32 size() const { return mSize;  }
};

}
}

#endif

