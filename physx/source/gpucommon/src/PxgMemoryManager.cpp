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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxgMemoryManager.h"
#include "PxgCudaMemoryAllocator.h"

using namespace physx;

namespace physx
{
	PxgCudaAllocatorCallbackBase::PxgCudaAllocatorCallbackBase(PxCudaContextManager* contextManager) : mContextManager(contextManager), mCudaContext(contextManager->getCudaContext()) {}
}

namespace
{	
	// PT: this one calls PxgPinnedMemoryAllocate/PxgPinnedMemoryDeallocate, i.e. cuMemHostAlloc/cuMemFreeHost
	class PxgCudaHostMemoryAllocatorCallback : public PxgCudaAllocatorCallbackBase
	{
	public:
		PxgCudaHostMemoryAllocatorCallback(PxCudaContextManager* contextManager) : PxgCudaAllocatorCallbackBase(contextManager)	{}

		// PxVirtualAllocatorCallback
		virtual	void*	allocate(size_t size, int, const char* file, int line)	PX_OVERRIDE PX_FINAL
		{
			PxScopedCudaLock lock(*mContextManager);
			return PxgPinnedMemoryAllocate(*mCudaContext, size, file, line);
		}

		virtual	void	deallocate(void* ptr)	PX_OVERRIDE PX_FINAL
		{
			if(ptr)
			{
				PxScopedCudaLock lock(*mContextManager);
				PxgPinnedMemoryDeallocate(*mCudaContext, ptr);
			}
		}
		//~PxVirtualAllocatorCallback
	};

	// PT: this one calls PxgCudaDeviceMemoryAllocate/PxgCudaDeviceMemoryDeallocate, i.e. cuMemAlloc/cuMemFree
	class PxgCudaDeviceMemoryAllocatorCallback : public PxgCudaAllocatorCallbackBase
	{
	public:
		PxgCudaDeviceMemoryAllocatorCallback(PxCudaContextManager* contextManager) : PxgCudaAllocatorCallbackBase(contextManager)	{}
		// PxVirtualAllocatorCallback

		virtual	void*	allocate(size_t size, int, const char* file, int line)	PX_OVERRIDE PX_FINAL
		{
			PxScopedCudaLock lock(*mContextManager);
			return PxgCudaDeviceMemoryAllocate(*mCudaContext, size, file, line);
		}

		virtual	void	deallocate(void* ptr)	PX_OVERRIDE PX_FINAL
		{
			if(ptr)
			{
				PxScopedCudaLock lock(*mContextManager);
				PxgCudaDeviceMemoryDeallocate(*mCudaContext, ptr);
			}
		}
		//~PxVirtualAllocatorCallback
	};

	class PxgMemoryManager : public PxsMemoryManager
	{
	public:
				PxgMemoryManager(PxCudaContextManager* cudaContextManager) : mCudaContextManager(cudaContextManager), mHostMemoryAllocator(cudaContextManager), mDeviceMemoryAllocator(cudaContextManager)	{}
		virtual	~PxgMemoryManager()	{}

		// PxsMemoryManager
		virtual PxVirtualAllocatorCallback*	getHostMemoryAllocator()	PX_OVERRIDE	{ return &mHostMemoryAllocator;		}
		virtual PxVirtualAllocatorCallback*	getDeviceMemoryAllocator()	PX_OVERRIDE	{ return &mDeviceMemoryAllocator;	}
		//~PxsMemoryManager
		
		PxCudaContextManager*					mCudaContextManager;
		PxgCudaHostMemoryAllocatorCallback		mHostMemoryAllocator;
		PxgCudaDeviceMemoryAllocatorCallback	mDeviceMemoryAllocator;
	};
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxsMemoryManager* physx::createPxgMemoryManager(PxCudaContextManager* cudaContextManager)
{
	return PX_NEW(PxgMemoryManager)(cudaContextManager);
}

