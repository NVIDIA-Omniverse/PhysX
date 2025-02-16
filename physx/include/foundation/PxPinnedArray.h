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

#ifndef PX_PINNED_ARRAY_H
#define PX_PINNED_ARRAY_H

#include "foundation/PxArray.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxBounds3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	// PT: the default pinned-memory arrays are defined as PxPinnedArray = PxArray<T, PxVirtualAllocator>.
	// The PxVirtualAllocator ultimately uses cuMemHostAlloc via PxgCudaHostMemoryAllocatorCallback / PxgPinnedMemoryAllocate.
	// We use the CU_MEMHOSTALLOC_DEVICEMAP flag there so cuMemHostGetDevicePointer() can later be used on returned ptr.
	//
	// The new pinned-memory arrays are defined as PxPinnedArraySafe = PxArray<T, PxPinnedAllocator<T> >. This uses a new
	// allocator that allocates either from cuMemHostAlloc, *or* fallbacks to regular allocs when we run out of pinned memory.
	// The issue is that in the second case cuMemHostGetDevicePointer() will fail, so we cannot use this everywhere.
	// 
	// I think this exposes issues in PxArray itself, for example in the swap function (we don't swap the allocator data there,
	// so when using a PxVirtualAllocator with PxArray the PxVirtualAllocator members are not swapped).

	// PT: this class uses the fact that PxArray inherits from the allocator to add new members to the array class. In particular
	// PxPinnedAllocator::mPinned describes where PxArray::mData has been allocated. The class is mostly designed to be used in
	// conjunction with PxArray, not as a standalone allocator.
	template<class T>
	class PxPinnedAllocator
	{
		public:
		PxPinnedAllocator(PxVirtualAllocatorCallback* callback = NULL, int group = 0) : mCallback(callback), mGroup(group), mPinned(0)	{}

		PX_INLINE	void* allocate(size_t size, const char* file, int line, uint32_t* cookie=NULL)
		{
			PX_ASSERT(mCallback);
			// PT: returns *previous* pinned value. It will be passed back to the deallocate function.
			if(cookie)
				*cookie = mPinned;

			if(!size)
			{
				mPinned = 0xffffffff;
				return NULL;
			}

			// PT: first, try with the pinned-memory allocator
			void* ptr = mCallback->allocate(size, mGroup, file, line);
			if(ptr)
			{
				mPinned = 1;
				return ptr;
			}

			// PT: if it fails, fallback to regular allocator
			mPinned = 0;
			return PxReflectionAllocator<T>::allocate(size, file, line);
		}

		PX_INLINE	void deallocate(void* ptr, uint32_t* cookie=NULL)
		{
			PX_ASSERT(mCallback);
			if(ptr)
			{
				// PT: by default use the internal value, except if we're given an explicit cookie
				const uint32_t pinned = cookie ? *cookie : mPinned;
				if(pinned==1)
					mCallback->deallocate(ptr);
				else
					PxReflectionAllocator<T>::deallocate(ptr);
			}
		}

		PX_FORCE_INLINE	void setCallback(PxVirtualAllocatorCallback* callback)
		{
			mCallback = callback;
		}

		PX_FORCE_INLINE	PxVirtualAllocatorCallback* getCallback()
		{
			return mCallback;
		}

		private:
		PxVirtualAllocatorCallback* mCallback;
		const int mGroup;
		uint32_t mPinned;
		PxPinnedAllocator& operator=(const PxPinnedAllocator&);
	};

	struct PxsCachedTransform;

	// PT: default versions:
	template<class T>
	using PxPinnedArray = PxArray<T, PxVirtualAllocator>;

	typedef	PxArray<PxsCachedTransform, PxVirtualAllocator>	PxCachedTransformArrayPinned;
	typedef PxArray<PxBounds3, PxVirtualAllocator>			PxBoundsArrayPinned;
	typedef	PxArray<PxReal, PxVirtualAllocator>				PxFloatArrayPinned;
	typedef	PxArray<PxU32, PxVirtualAllocator>				PxInt32ArrayPinned;
    typedef PxArray<PxU16, PxVirtualAllocator>				PxInt16ArrayPinned;
	typedef	PxArray<PxU8, PxVirtualAllocator>				PxInt8ArrayPinned;

	// PT: new versions
	template<class T>
	using PxPinnedArraySafe = PxArray<T, PxPinnedAllocator<T> >;

	typedef	PxArray<PxsCachedTransform, PxPinnedAllocator<PxsCachedTransform> >	PxCachedTransformArrayPinnedSafe;
	typedef PxArray<PxBounds3, PxPinnedAllocator<PxBounds3> >					PxBoundsArrayPinnedSafe;
	typedef	PxArray<PxReal, PxPinnedAllocator<PxReal> >							PxFloatArrayPinnedSafe;
	typedef	PxArray<PxU32, PxPinnedAllocator<PxU32> >							PxInt32ArrayPinnedSafe;
    typedef PxArray<PxU16, PxPinnedAllocator<PxU16> >							PxInt16ArrayPinnedSafe;
	typedef	PxArray<PxU8, PxPinnedAllocator<PxU8> >								PxInt8ArrayPinnedSafe;

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif

