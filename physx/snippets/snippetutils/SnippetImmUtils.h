// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPhysXConfig.h"
#include "foundation/PxArray.h"
#include "foundation/PxUserAllocated.h"
#include "collision/PxCollisionDefs.h"
#include "solver/PxSolverDefs.h"

namespace physx
{
namespace SnippetImmUtils
{

	class BlockBasedAllocator
	{
		struct AllocationPage : PxUserAllocated
		{
			static const PxU32 PageSize = 32 * 1024;
			PxU8 mPage[PageSize];

			PxU32 currentIndex;

			AllocationPage() : currentIndex(0){}

			PxU8* allocate(const PxU32 size)
			{
				PxU32 alignedSize = (size + 15)&(~15);
				if ((currentIndex + alignedSize) < PageSize)
				{
					PxU8* ret = &mPage[currentIndex];
					currentIndex += alignedSize;
					return ret;
				}
				return NULL;
			}
		};

		AllocationPage* currentPage;

		PxArray<AllocationPage*> mAllocatedBlocks;
		PxU32 mCurrentIndex;

	public:
		BlockBasedAllocator() : currentPage(NULL), mCurrentIndex(0)
		{
		}

		virtual PxU8* allocate(const PxU32 byteSize)
		{
			if (currentPage)
			{
				PxU8* data = currentPage->allocate(byteSize);
				if (data)
					return data;
			}

			if (mCurrentIndex < mAllocatedBlocks.size())
			{
				currentPage = mAllocatedBlocks[mCurrentIndex++];
				currentPage->currentIndex = 0;
				return currentPage->allocate(byteSize);
			}
			currentPage = PX_NEW(AllocationPage);
			mAllocatedBlocks.pushBack(currentPage);
			mCurrentIndex = mAllocatedBlocks.size();

			return currentPage->allocate(byteSize);
		}

		void release()
		{
			for (PxU32 a = 0; a < mAllocatedBlocks.size(); ++a)
				PX_DELETE(mAllocatedBlocks[a]);
			mAllocatedBlocks.clear();
			currentPage = NULL;
			mCurrentIndex = 0;
		}

		void reset()
		{
			currentPage = NULL;
			mCurrentIndex = 0;
		}

		virtual ~BlockBasedAllocator()
		{
			release();
		}
	};

	class TestCacheAllocator : public PxCacheAllocator
	{
		BlockBasedAllocator mAllocator[2];
		PxU32 currIdx;

	public:

		TestCacheAllocator() : currIdx(0)
		{
		}

		virtual PxU8* allocateCacheData(const PxU32 byteSize)
		{
			return mAllocator[currIdx].allocate(byteSize);
		}

		void release() { currIdx = 1 - currIdx; mAllocator[currIdx].release(); }

		void reset() { currIdx = 1 - currIdx; mAllocator[currIdx].reset(); }

		virtual ~TestCacheAllocator(){}
	};

	class TestConstraintAllocator : public PxConstraintAllocator
	{
		BlockBasedAllocator mConstraintAllocator;
		BlockBasedAllocator mFrictionAllocator[2];

		PxU32 currIdx;
	public:

		TestConstraintAllocator() : currIdx(0)
		{
		}
		virtual PxU8* reserveConstraintData(const PxU32 byteSize){ return mConstraintAllocator.allocate(byteSize); }
		virtual PxU8* reserveFrictionData(const PxU32 byteSize){ return mFrictionAllocator[currIdx].allocate(byteSize); }

		void release() { currIdx = 1 - currIdx; mConstraintAllocator.release(); mFrictionAllocator[currIdx].release(); }

		virtual ~TestConstraintAllocator() {}
	};

}
}
