// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_FLUSH_POOL_H
#define CM_FLUSH_POOL_H

#include "foundation/PxUserAllocated.h"
#include "foundation/PxBitUtils.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"

/*
Pool used to allocate variable sized tasks. It's intended to be cleared after a short period (time step).
*/

namespace physx
{
namespace Cm
{
	static const PxU32 sSpareChunkCount = 2;

	class FlushPool
	{
		PX_NOCOPY(FlushPool)
	public:
		FlushPool(PxU32 chunkSize) : mChunks("FlushPoolChunk"), mChunkIndex(0), mOffset(0), mChunkSize(chunkSize)
		{
			mChunks.pushBack(static_cast<PxU8*>(PX_ALLOC(mChunkSize, "PxU8")));
		}

		~FlushPool()
		{
			for (PxU32 i = 0; i < mChunks.size(); ++i)
				PX_FREE(mChunks[i]);
		}

		// alignment must be a power of two
		void* allocate(PxU32 size, PxU32 alignment=16)
		{
			PxMutex::ScopedLock lock(mMutex);
			return allocateNotThreadSafe(size, alignment);
		}

		// alignment must be a power of two
		void* allocateNotThreadSafe(PxU32 size, PxU32 alignment=16)
		{
			PX_ASSERT(PxIsPowerOfTwo(alignment));
			PX_ASSERT(size <= mChunkSize && !mChunks.empty());
			
			// padding for alignment
			size_t unalignedStart = size_t(mChunks[mChunkIndex]+mOffset);
			PxU32 pad = PxU32(((unalignedStart+alignment-1)&~(size_t(alignment)-1)) - unalignedStart);

			if (mOffset + size + pad > mChunkSize)
			{
				mChunkIndex++;
				mOffset = 0;
				if (mChunkIndex >= mChunks.size())
					mChunks.pushBack(static_cast<PxU8*>(PX_ALLOC(mChunkSize, "PxU8")));

				// update padding to ensure new alloc is aligned
				unalignedStart = size_t(mChunks[mChunkIndex]);
				pad = PxU32(((unalignedStart+alignment-1)&~(size_t(alignment)-1)) - unalignedStart);
			}

			void* ptr = mChunks[mChunkIndex] + mOffset + pad;
			PX_ASSERT((size_t(ptr)&(size_t(alignment)-1)) == 0);
			mOffset += size + pad;
			return ptr;
		}

		void clear(PxU32 spareChunkCount = sSpareChunkCount)
		{
			PxMutex::ScopedLock lock(mMutex);
			
			clearNotThreadSafe(spareChunkCount);
		}

		void clearNotThreadSafe(PxU32 spareChunkCount = sSpareChunkCount)
		{
			//release memory not used previously
			PxU32 targetSize = mChunkIndex+spareChunkCount;
			while (mChunks.size() > targetSize)
			{
				PxU8* ptr = mChunks.popBack();
				PX_FREE(ptr);
			}

			mChunkIndex = 0;
			mOffset = 0;
		}

		void resetNotThreadSafe()
		{
			PxU8* firstChunk = mChunks[0];

			for (PxU32 i = 1; i < mChunks.size(); ++i)
				PX_FREE(mChunks[i]);

			mChunks.clear();
			mChunks.pushBack(firstChunk);
			mChunkIndex = 0;
			mOffset = 0;
		}

		void lock()
		{
			mMutex.lock();
		}

		void unlock()
		{
			mMutex.unlock();	
		}

	private:
		PxMutex mMutex;
		PxArray<PxU8*> mChunks;
		PxU32 mChunkIndex;
		PxU32 mOffset;
		PxU32 mChunkSize;
	};

	
} // namespace Cm

}

#endif
