// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_VIRTUAL_ALLOCATOR_CALLBACK_H
#define CM_VIRTUAL_ALLOCATOR_CALLBACK_H

#include "foundation/PxAllocator.h"

namespace physx
{
namespace Cm
{

/**
\brief	Virtual allocator callback used to provide run-time defined allocators to foundation types like Array or Bitmap.
*/
class VirtualAllocatorCallback
{
  public:
	virtual ~VirtualAllocatorCallback() {}

	virtual void* allocate(size_t size, int group, const char* file, int line) = 0;
	virtual void deallocate(void* ptr) = 0;
};

} // namespace Cm
} // namespace physx


#endif

