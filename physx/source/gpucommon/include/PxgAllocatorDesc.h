// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef	PXG_ALLOCATOR_DESC_H
#define PXG_ALLOCATOR_DESC_H

#include "foundation/PxPreprocessor.h"

namespace physx
{
	namespace Cm
	{
		class VirtualAllocatorCallback;
	}

	class PxgHeapMemoryAllocator;

	// Simple helper to pass around allocators
	struct PxgAllocatorDesc
	{
		PxgAllocatorDesc(PxgHeapMemoryAllocator& deviceAllocator,
						 Cm::VirtualAllocatorCallback& hostAllocator,
						 Cm::VirtualAllocatorCallback& hostMappedAllocator)
		:
			deviceAlloc(deviceAllocator),
			hostAlloc(hostAllocator),
			hostMappedAlloc(hostMappedAllocator)
		{}

		PxgHeapMemoryAllocator& deviceAlloc;				// device mem allocation always goes through the heap memory allocator
		Cm::VirtualAllocatorCallback& hostAlloc;			// pinned host mem allocation may go through other allocators for testing
		Cm::VirtualAllocatorCallback& hostMappedAlloc;		// pinned host mem allocation may go through other allocators for testing
	};
}

#endif
