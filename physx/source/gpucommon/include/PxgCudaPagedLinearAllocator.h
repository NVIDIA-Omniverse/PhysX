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

#ifndef PXG_CUDA_PAGED_LINEAR_ALLOCATOR_H
#define PXG_CUDA_PAGED_LINEAR_ALLOCATOR_H

#include "foundation/PxArray.h"
#include "foundation/PxMath.h"
#include "foundation/PxMutex.h"

namespace physx
{

template<typename AllocT>
class PxgCudaPagedLinearAllocator
{
	PX_NOCOPY(PxgCudaPagedLinearAllocator)
public:

	PxgCudaPagedLinearAllocator(AllocT&	alloc, const size_t defaultPageBytesize = 1024 * 1024 ) :
		mAlloc(alloc),
		mCurrOffsetBytes(0),
		mCurrPage(0),
		mDefaultPageBytesize(defaultPageBytesize)
	{}
	
	virtual ~PxgCudaPagedLinearAllocator()
	{
		resetAndRelease();
	}

	void resetAndRelease()
	{
		reset();

		for (PxU32 i = 0; i < mPages.size(); ++i)
		{
			mAlloc.deallocate(mPages[i]);
		}

		mPages.resize(0);
	}

	void reset()
	{
		mCurrPage = 0;
		mCurrOffsetBytes = 0;
		mCurrPageSize = mPagesSize.size() == 0 ? 0 : mPagesSize[0];
	}

	//attention: no alignment!
	void* allocate(size_t byteSize)
	{
		bool outOfMem = false;

		bool empty = mPages.empty();

		if (!empty && (mCurrOffsetBytes + byteSize) >= mCurrPageSize)
		{
			mCurrOffsetBytes = 0;
			++mCurrPage;
			if (mCurrPage < mPages.size())
				mCurrPageSize = mPagesSize[mCurrPage];
			else
				mCurrPageSize = 0;
		}

		if (empty || mCurrOffsetBytes + byteSize >= mCurrPageSize)
		{
			//Let's first iterate through all the pages to find if any are large-enough
			bool found = false;
			for (PxU32 i = mCurrPage; i < mPages.size(); ++i)
			{
				if (mPagesSize[i] >= byteSize)
				{
					found = true;
					mCurrPage = i;
					mCurrPageSize = mPagesSize[i];
					break;
				}
			}
			if (!found)
				outOfMem = !addNewPage(byteSize);
		}

		if (outOfMem)
		{
			return NULL;
		}

		ptrdiff_t offs = (ptrdiff_t)mCurrOffsetBytes;
		mCurrOffsetBytes += byteSize;

		return mPages[mCurrPage] + offs;
	}

	void* allocateAligned(size_t alignment, size_t byteSize)
	{
#if 0
		size_t pad = alignment - 1 + sizeof(size_t); // store offset for delete.
#else
		size_t pad = alignment - 1;
#endif
		size_t newByteSize = byteSize + pad;
			
		PxU8* basePtr = reinterpret_cast<PxU8*>(allocate(newByteSize));

#if 0
		size_t ptrAligningOffset = basePtr.getAligningOffset(alignment, sizeof(size_t));
		typename AllocT::template Pointer<PxU8> offsetPtr = basePtr + ptrAligningOffset;

		// wide mask
		((size_t*)ptr)[-1] = size_t(ptr - base); // store offset
#else
		size_t alignOffs = alignment - (size_t(basePtr) & (alignment - 1));

		size_t ptrAligningOffset = (alignOffs & (alignment - 1));
	
		PxU8* offsetPtr = basePtr + (ptrdiff_t) ptrAligningOffset;
#endif

		return offsetPtr;
	}

	PxMutex											mMutex;

protected:
	
	bool addNewPage(size_t requestedAllocByteSize)
	{
		const size_t size = PxMax(requestedAllocByteSize, mDefaultPageBytesize);
		mPages.pushBack(reinterpret_cast<PxU8*>(mAlloc.allocate(size, 0, PX_FL)));
		mPagesSize.pushBack(size);
		PX_ASSERT(mPages.back() != 0);

		if (mPages.back() == 0)
		{
			mPages.popBack();

			return false;
		}

		mCurrOffsetBytes = 0;
		mCurrPage = mPages.size() - 1;
		mCurrPageSize = size;

		return true;
	}

	AllocT&											mAlloc;
	PxArray<PxU8*>									mPages;
	PxArray<size_t>									mPagesSize;
	size_t											mCurrOffsetBytes;
	PxU32											mCurrPage;
	size_t											mCurrPageSize;
	size_t											mDefaultPageBytesize;
	
};


}

#endif
