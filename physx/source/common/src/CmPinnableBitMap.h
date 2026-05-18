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

#ifndef CM_PINNABLE_BITMAP_H
#define CM_PINNABLE_BITMAP_H

#include "foundation/PxBitMap.h"
#include "CmPinnableAllocator.h"

namespace physx
{
namespace Cm
{

/*!
\brief PxBitmapBase-derived container that forwards allocations to VirtualAllocatorCallback
with optional default allocator fallback.

This container uses PinnableAllocator to manage its memory. Allocations are forwarded to a
VirtualAllocatorCallback instance, with optional fallback to the default allocator when fallback
behavior is enabled. See the PinnableAllocator documentation for details on allocator selection
and fallback behavior.

The container is intended to be backed by CUDA host memory. Ideally, this memory is non-pageable
(pinned) to support asynchronous data transfers to GPU device memory. When fallback is enabled,
pageable memory may be used instead, in which case data transfers become synchronous and may incur
a performance penalty.

If the memory is accessed directly by CUDA kernels, it must be pinned and mapped into the CUDA
device address space. In this configuration, allocator fallback is typically disabled.

\see PxBitMapBase, PinnableAllocator, VirtualAllocatorCallback
*/
class PinnableBitMap : public PxBitMapBase<PinnableAllocator<PxU32> >
{
	PX_NOCOPY(PinnableBitMap)
	typedef PinnableAllocator<PxU32> Alloc;
	typedef PxBitMapBase<Alloc> Base;

public:
	/*!
	\brief Constructor for bitmap.
	\param callback Memory allocator callback used with priority
	\param group Internal memory stats group
	\param fallback Configures whether fallback to default allocation is enabled or not
	*/
	PX_INLINE explicit PinnableBitMap(VirtualAllocatorCallback& callback, PxI32 group = 0,
									  PinnableAllocatorFallback::Enum fallback = PinnableAllocatorFallback::eENABLED)
	: Base(Alloc(callback, group, fallback))
	{
	}
};

} // namespace Cm
} // namespace physx

#endif // PX_PINNED_BITMAP_H
