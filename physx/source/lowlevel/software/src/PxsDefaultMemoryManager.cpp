// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxsMemoryManager.h"
#include "CmVirtualAllocatorCallback.h"

using namespace physx;

namespace
{
	class PxsDefaultMemoryAllocator : public Cm::VirtualAllocatorCallback
	{
	public:
		virtual void* allocate(size_t size, int, const char*, int)	PX_OVERRIDE	PX_FINAL	{ return PX_ALLOC(size, "unused");	}
		virtual void deallocate(void* ptr)							PX_OVERRIDE	PX_FINAL	{ PX_FREE(ptr);						}
	};

	class PxsDefaultMemoryManager : public PxsMemoryManager
	{
	public:
		// PxsMemoryManager
		virtual Cm::VirtualAllocatorCallback* getPinnedHostMemoryAllocator()	PX_OVERRIDE	PX_FINAL	{ return &mDefaultMemoryAllocator; }
		virtual Cm::VirtualAllocatorCallback* getDeviceMemoryAllocator()		PX_OVERRIDE	PX_FINAL	{ return NULL; }
		//~PxsMemoryManager
		PxsDefaultMemoryAllocator	mDefaultMemoryAllocator;
	};
}

PxsMemoryManager* physx::createDefaultMemoryManager()
{
	return PX_NEW(PxsDefaultMemoryManager);
}
