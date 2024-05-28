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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

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

	void reserve(PxU32 capacity)
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

	void resize(PxU32 size)
	{
		reserve(size);
		for (PxU32 a = mSize; a < size; ++a)
		{
			mBlocks[a / SlabSize][a&(SlabSize - 1)].~T();
			mBlocks[a / SlabSize][a&(SlabSize-1)] = T();
		}
		mSize = size;
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

