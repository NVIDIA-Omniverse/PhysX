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

#include "PxgMemoryManager.h"
#include "PxgCudaMemoryAllocator.h"

using namespace physx;

namespace physx
{
	PxgCudaAllocatorCallbackBase::PxgCudaAllocatorCallbackBase(PxCudaContextManager& contextManager) : mContextManager(contextManager), mCudaContext(*contextManager.getCudaContext()) {}
}

namespace
{	
	// PT: this one calls PxgPinnedMemoryAllocate/PxgPinnedMemoryDeallocate, i.e. cuMemHostAlloc/cuMemFreeHost
	template<PxU32 Flags>
	class PxgCudaHostMemoryAllocatorCallback : public PxgCudaAllocatorCallbackBase
	{
	public:
		PxgCudaHostMemoryAllocatorCallback(PxCudaContextManager& contextManager) : PxgCudaAllocatorCallbackBase(contextManager)	{}

		// Cm::VirtualAllocatorCallback
		virtual	void*	allocate(size_t size, int, const char* file, int line)	PX_OVERRIDE PX_FINAL
		{
			PxScopedCudaLock lock(mContextManager);
			return PxgPinnedMemoryAllocate(mCudaContext, size, Flags, file, line);
		}

		virtual	void	deallocate(void* ptr)	PX_OVERRIDE PX_FINAL
		{
			if(ptr)
			{
				PxScopedCudaLock lock(mContextManager);
				PxgPinnedMemoryDeallocate(mCudaContext, ptr);
			}
		}
		//~Cm::VirtualAllocatorCallback
	};

	// PT: this one calls PxgCudaDeviceMemoryAllocate/PxgCudaDeviceMemoryDeallocate, i.e. cuMemAlloc/cuMemFree
	class PxgCudaDeviceMemoryAllocatorCallback : public PxgCudaAllocatorCallbackBase
	{
	public:
		PxgCudaDeviceMemoryAllocatorCallback(PxCudaContextManager& contextManager) : PxgCudaAllocatorCallbackBase(contextManager)	{}
		// Cm::VirtualAllocatorCallback

		virtual	void*	allocate(size_t size, int, const char* file, int line)	PX_OVERRIDE PX_FINAL
		{
			PxScopedCudaLock lock(mContextManager);
			return PxgCudaDeviceMemoryAllocate(mCudaContext, size, file, line);
		}

		virtual	void	deallocate(void* ptr)	PX_OVERRIDE PX_FINAL
		{
			if(ptr)
			{
				PxScopedCudaLock lock(mContextManager);
				PxgCudaDeviceMemoryDeallocate(mCudaContext, ptr);
			}
		}
		//~Cm::VirtualAllocatorCallback
	};

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxgMemoryManager::PxgMemoryManager(PxCudaContextManager& cudaContextManager)
{
	mPinnedHostMemoryAllocator = PX_NEW(PxgCudaHostMemoryAllocatorCallback<CU_MEMHOSTALLOC_PORTABLE>)(cudaContextManager);
	mPinnedHostMappedMemoryAllocator =
		PX_NEW(PxgCudaHostMemoryAllocatorCallback<CU_MEMHOSTALLOC_PORTABLE | CU_MEMHOSTALLOC_DEVICEMAP>)(cudaContextManager);
	mDeviceMemoryAllocator = PX_NEW(PxgCudaDeviceMemoryAllocatorCallback)(cudaContextManager);
}

PxgMemoryManager::~PxgMemoryManager()
{
	PX_DELETE(mPinnedHostMemoryAllocator);
	PX_DELETE(mPinnedHostMappedMemoryAllocator);
	PX_DELETE(mDeviceMemoryAllocator);
}

PxgCudaAllocatorCallbackBase* PxgMemoryManager::getCudaHostMemoryAllocator(const PxU32 flags)
{
	if((flags & CU_MEMHOSTALLOC_PORTABLE) > 0 && (flags & CU_MEMHOSTALLOC_DEVICEMAP) > 0)
	{
		return mPinnedHostMappedMemoryAllocator;
	}
	else if((flags & CU_MEMHOSTALLOC_PORTABLE) > 0)
	{
		return mPinnedHostMemoryAllocator;
	}
	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Unsupported pinned memory allocator requested.");
	return NULL;
}

PxgCudaAllocatorCallbackBase* PxgMemoryManager::getCudaDeviceMemoryAllocator()
{
	return mDeviceMemoryAllocator;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxsMemoryManager* physx::createPxgMemoryManager(PxCudaContextManager* cudaContextManager)
{
	if(!cudaContextManager)
		return NULL;

	return PX_NEW(PxgMemoryManager)(*cudaContextManager);
}

