// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_MEMORY_ALLOCATOR_H
#define PXG_MEMORY_ALLOCATOR_H

#include "PxsMemoryManager.h"
#include "foundation/PxArray.h"
#include "CmVirtualAllocatorCallback.h"

namespace physx
{
	class PxCudaContextManager;
	class PxCudaContext;

	// PT: this is for GPU, see createDefaultMemoryManager for CPU
	PxsMemoryManager* createPxgMemoryManager(PxCudaContextManager* cudaContextManager);


	class PxgCudaAllocatorCallbackBase : public Cm::VirtualAllocatorCallback, public PxUserAllocated
	{
	public:
		PxgCudaAllocatorCallbackBase(PxCudaContextManager& contextManager);
		virtual					~PxgCudaAllocatorCallbackBase() {}
		PxCudaContextManager& mContextManager;
		PxCudaContext& mCudaContext;
	};

	class PxgMemoryManager : public PxsMemoryManager
	{
	public:
		
		PxgMemoryManager(PxCudaContextManager& cudaContextManager);
		virtual ~PxgMemoryManager();

		// PxsMemoryManager
		virtual Cm::VirtualAllocatorCallback* getPinnedHostMemoryAllocator() PX_OVERRIDE
		{
			return mPinnedHostMemoryAllocator;
		}

		virtual Cm::VirtualAllocatorCallback* getDeviceMemoryAllocator() PX_OVERRIDE
		{
			return mDeviceMemoryAllocator;
		}
		//~PxsMemoryManager
		
		// accessors for the PxgCudaAllocatorCallbackBase typed allocators.
		PxgCudaAllocatorCallbackBase* getCudaHostMemoryAllocator(const PxU32 flags);
		PxgCudaAllocatorCallbackBase* getCudaDeviceMemoryAllocator();

	private:
		PxgCudaAllocatorCallbackBase*		mPinnedHostMemoryAllocator;
		PxgCudaAllocatorCallbackBase*		mPinnedHostMappedMemoryAllocator;
		PxgCudaAllocatorCallbackBase*		mDeviceMemoryAllocator;
	};
}

#endif