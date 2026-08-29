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

#ifndef CM_PINNABLE_ALLOCATOR_H
#define CM_PINNABLE_ALLOCATOR_H

#include "CmVirtualAllocatorCallback.h"

namespace physx
{
namespace Cm
{

/*!
\brief Enumeration for enabling/disabling fallback behavior of Cm::PinnableAllocator
*/
struct PinnableAllocatorFallback
{
	enum Enum
	{
		eENABLED,
		eDISABLED
	};
};

/*!
\brief Allocator specialized for containers whose data may be copied to GPU device memory or
accessed directly by GPU kernels.

This allocator serves as a base class for GPU host-memory-backed containers. Memory is allocated using a
VirtualAllocatorCallback instance. If an allocation fails, PxReflectionAllocator is used as a fallback
when fallback behavior is enabled via Cm::PinnableAllocatorFallback::eENABLED.

The allocator tracks whether VirtualAllocatorCallback or PxReflectionAllocator was used for the most
recent allocation in order to ensure that deallocation is performed with the corresponding allocator.
This mechanism assumes that each allocation is followed by a matching deallocation. To support container
resize operations - where allocations and deallocations may occur out of order - a cookie value is provided
by the container to supply the required allocator information.

Typical usage patterns include:
* VirtualAllocatorCallback points to a CUDA host allocator for memory that is asynchronously copied to
  GPU device memory. In this configuration, memory is pinned (non-pageable). Cm::PinnableAllocatorFallback::eENABLED
  is typically used, as copy operations automatically synchronize when pageable memory is supplied and
  can still complete successfully.
* VirtualAllocatorCallback points to a CUDA host allocator for memory that may be accessed directly by
  GPU kernels. In this configuration, pinned (non-pageable) memory that can be mapped into the GPU device
  address space is required, and PinnableAllocatorFallback::eDISABLED is used.

\see PinnableArray, PinnableBitMap, PinnableObject
*/
template <class T>
class PinnableAllocator
{
public:
	PinnableAllocator(VirtualAllocatorCallback& callback, int group, PinnableAllocatorFallback::Enum fallback)
	: mCallback(callback), mGroup(group), mFlags(fallback == PinnableAllocatorFallback::eENABLED ? PxU32(Flag::eFALLBACK) : 0u)
	{
	}

	PX_INLINE void* allocate(size_t size, const char* file, int line, uint32_t* cookie = NULL)
	{
		if(cookie)
			*cookie = mFlags;

		if(!size)
		{
			// keep fallback flag, clear pinned-state bits
			mFlags &= PxU32(Flag::eFALLBACK);
			return NULL;
		}

		// PT: first, try with the pinned-memory allocator
		void* ptr = mCallback.allocate(size, mGroup, file, line);
		if(ptr)
		{
			mFlags |= PxU32(Flag::ePINNED);
			return ptr;
		}

		// PT: if it fails, optionally fallback to regular allocator
		if(mFlags & PxU32(Flag::eFALLBACK))
		{
			mFlags &= PxU32(Flag::eFALLBACK); // pinned-state bit = 0 -> non-pinned allocation
			return PxReflectionAllocator<T>::allocate(size, file, line);
		}

		// No fallback requested, forwarding pinned OOM
		// Can't clear pinned flag, because the array still holds its old memory!
		return NULL;
	}

	PX_INLINE void deallocate(void* ptr, uint32_t* cookie = NULL)
	{
		if(ptr)
		{
			// PT: by default use the internal value, except if we're given an explicit cookie
			const uint32_t flags = cookie ? *cookie : mFlags;
			const uint32_t pinnedState = flags & PxU32(Flag::ePINNED);
			if(pinnedState)
				mCallback.deallocate(ptr);
			else
				PxReflectionAllocator<T>::deallocate(ptr);
		}
	}

private:
	struct Flag
	{
		enum Enum : PxU32
		{
			ePINNED = (1u << 0),
			eFALLBACK = (1u << 31)
		};
	};

	VirtualAllocatorCallback& mCallback;
	const int mGroup;
	PxU32 mFlags;
	PinnableAllocator& operator=(const PinnableAllocator&);
};

} // namespace Cm
} // namespace physx


#endif

