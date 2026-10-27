// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_HEAP_MEMORY_ALLOCATOR_H
#define PXS_HEAP_MEMORY_ALLOCATOR_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxUserAllocated.h"
#include "PxsHeapStats.h"

namespace physx
{
#if PX_SUPPORT_GPU_PHYSX

	namespace Cm
	{
		class VirtualAllocatorCallback;
	}

	class PxsHeapMemoryAllocatorManager : public PxUserAllocated
	{
	public:
		virtual ~PxsHeapMemoryAllocatorManager() {}

		virtual PxU64 getDeviceMemorySize() const = 0;
		virtual PxsHeapStats getDeviceHeapStats() const = 0;
		virtual void flushDeferredDeallocs() = 0;

		Cm::VirtualAllocatorCallback* mPinnedHostMemoryAllocator;
		Cm::VirtualAllocatorCallback* mPinnedHostMappedMemoryAllocator;
	};
#endif
}

#endif
