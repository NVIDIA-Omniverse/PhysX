// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_PINNABLE_ARRAY_H
#define CM_PINNABLE_ARRAY_H

#include "foundation/PxArray.h"
#include "CmPinnableAllocator.h"

namespace physx
{
namespace Cm
{

/*!
\brief PxArray-derived container that forwards allocations to VirtualAllocatorCallback
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

\see PxArray, PinnableAllocator, VirtualAllocatorCallback
*/
template <class T>
class PinnableArray : public PxArray<T, PinnableAllocator<T> >
{
	typedef PinnableAllocator<T> Alloc;
	typedef PxArray<T, Alloc> Base;
	PX_NOCOPY(PinnableArray)

public:
	/*!
	\brief Constructor for pinnable array.
	\param callback Memory allocator callback used with priority
	\param group Internal memory stats group
	\param fallback Configures whether fallback to default allocation is enabled or not
	*/
	PX_INLINE explicit PinnableArray(VirtualAllocatorCallback& callback, int group = 0,
									 PinnableAllocatorFallback::Enum fallback = PinnableAllocatorFallback::eENABLED)
	: Base(Alloc(callback, group, fallback))
	{
	}
};

} // namespace Cm
} // namespace physx


#endif

