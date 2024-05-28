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
