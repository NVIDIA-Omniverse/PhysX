// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_MEMORY_MANAGER_H
#define PXS_MEMORY_MANAGER_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
	namespace Cm
	{
		class VirtualAllocatorCallback;
	}

	class PxsMemoryManager : public PxUserAllocated
	{
	public:
		virtual								~PxsMemoryManager(){}
		virtual	Cm::VirtualAllocatorCallback* getPinnedHostMemoryAllocator()	= 0;
		virtual	Cm::VirtualAllocatorCallback* getDeviceMemoryAllocator()	= 0;
	};

	// PT: this is for CPU, see createPxgMemoryManager for GPU
	PxsMemoryManager* createDefaultMemoryManager();
}

#endif
