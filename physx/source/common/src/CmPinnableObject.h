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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef CM_PINNABLE_OBJECT_H
#define CM_PINNABLE_OBJECT_H

#include "CmPinnableAllocator.h"

namespace physx
{
namespace Cm
{

/*!
\brief Container for a single object that forwards allocations to VirtualAllocatorCallback
with optional default allocator fallback.

This is a RAII wrapper around PinnableAllocator which is useful when a single object needs to live in cuda host memory.
The object is allocated in the constructor and destroyed automatically in the destructor.

The container is intended to be backed by CUDA host memory. Ideally, this memory is non-pageable
(pinned) to support asynchronous data transfers to GPU device memory. When fallback is enabled,
pageable memory may be used instead, in which case data transfers become synchronous and may incur
a performance penalty.

If the memory is accessed directly by CUDA kernels, it must be pinned and mapped into the CUDA
device address space. In this configuration, allocator fallback is typically disabled.

\see PinnableAllocator, VirtualAllocatorCallback
*/
template <class T, class Alloc = PinnableAllocator<T> >
class PinnableObject : protected Alloc
{
	PX_NOCOPY(PinnableObject)

public:
	/*!
	\brief Construct the wrapper and attempt to allocate and default-construct the object.
	\param callback Memory allocator callback used with priority
	\param group Internal memory stats group
	\param fallback Configures whether fallback to default allocation is enabled or not
	*/
	PX_INLINE explicit PinnableObject(VirtualAllocatorCallback& callback, int group = 0,
									  PinnableAllocatorFallback::Enum fallback = PinnableAllocatorFallback::eENABLED)
	: Alloc(callback, group, fallback), mPtr(NULL)
	{
		void* mem = Alloc::allocate(sizeof(T), PX_FL);
		if(mem)
			mPtr = PX_PLACEMENT_NEW(mem, T)();
	}

	//! Destructor
	PX_INLINE ~PinnableObject() { destroy(); }

	//! Returns true if an object is currently allocated.
	PX_FORCE_INLINE bool isValid() const { return mPtr != NULL; }

	//! Access the stored object. The wrapper must be valid (isValid() == true).
	PX_FORCE_INLINE T& get()
	{
		PX_ASSERT(mPtr);
		return *mPtr;
	}

	//! Access the stored object. The wrapper must be valid (isValid() == true).
	PX_FORCE_INLINE const T& get() const
	{
		PX_ASSERT(mPtr);
		return *mPtr;
	}

	//! Returns the raw pointer (may be NULL if no object was created).
	PX_FORCE_INLINE T* data() { return mPtr; }

	//! Returns the raw pointer (may be NULL if no object was created).
	PX_FORCE_INLINE const T* data() const { return mPtr; }

	/**
	\brief Explicitly destroy the contained object (if any) and free its memory.
	*/
	PX_INLINE void destroy()
	{
		if(mPtr)
		{
			mPtr->~T();
			Alloc::deallocate(mPtr);
			mPtr = NULL;
		}
	}

private:
	T* mPtr;
};

} // namespace Cm
} // namespace physx

#endif

