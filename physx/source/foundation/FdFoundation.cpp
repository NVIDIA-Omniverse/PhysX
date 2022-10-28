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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxProfiler.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxString.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxPhysicsVersion.h"
#include "FdFoundation.h"

namespace physx
{

Foundation::Foundation(PxErrorCallback& errc, PxAllocatorCallback& alloc)
: mAllocatorCallback(alloc)
, mErrorCallback(errc)
, mBroadcastingAllocator(alloc, errc)
, mBroadcastingError(errc)
,
#if PX_CHECKED
    mReportAllocationNames(true),
#else
    mReportAllocationNames(false),
#endif
    mErrorMask(PxErrorCode::Enum(~0))
, mErrorMutex("Foundation::mErrorMutex")
, mTempAllocMutex("Foundation::mTempAllocMutex")
, mRefCount(0)
{
}

Foundation::~Foundation()
{
	// deallocate temp buffer allocations
	PxAllocator alloc;
	for(PxU32 i = 0; i < mTempAllocFreeTable.size(); ++i)
	{
		for(PxTempAllocatorChunk* ptr = mTempAllocFreeTable[i]; ptr;)
		{
			PxTempAllocatorChunk* next = ptr->mNext;
			alloc.deallocate(ptr);
			ptr = next;
		}
	}
	mTempAllocFreeTable.reset();
}

Foundation& Foundation::getInstance()
{
	PX_ASSERT(mInstance);
	return *mInstance;
}

void Foundation::setInstance(Foundation& foundation)
{
	mInstance = &foundation;
}

PxU32 Foundation::getWarnOnceTimestamp()
{
	PX_ASSERT(mInstance != NULL);
	return mWarnOnceTimestap;
}

bool Foundation::error(PxErrorCode::Enum c, const char* file, int line, const char* messageFmt, ...)
{
	va_list va;
	va_start(va, messageFmt);
	error(c, file, line, messageFmt, va);
	va_end(va);
	return false;
}

bool Foundation::error(PxErrorCode::Enum e, const char* file, int line, const char* messageFmt, va_list va)
{
	PX_ASSERT(messageFmt);
	if(e & mErrorMask)
	{
		// this function is reentrant but user's error callback may not be, so...
		Mutex::ScopedLock lock(mErrorMutex);

		// using a static fixed size buffer here because:
		// 1. vsnprintf return values differ between platforms
		// 2. va_start is only usable in functions with ellipses
		// 3. ellipses (...) cannot be passed to called function
		// which would be necessary to dynamically grow the buffer here

		static const size_t bufSize = 1024;
		char stringBuffer[bufSize];
		Pxvsnprintf(stringBuffer, bufSize, messageFmt, va);

		mBroadcastingError.reportError(e, stringBuffer, file, line);
	}
	return false;
}

Foundation* Foundation::createInstance(PxU32 version, PxErrorCallback& errc, PxAllocatorCallback& alloc)
{
	if(version != PX_PHYSICS_VERSION)
	{
		char* buffer = new char[256];
		physx::Pxsnprintf(buffer, 256, "Wrong version: physics version is 0x%08x, tried to create 0x%08x",
			PX_PHYSICS_VERSION, version);
		errc.reportError(PxErrorCode::eINVALID_PARAMETER, buffer, __FILE__, __LINE__);
		return 0;
	}

	if(!mInstance)
	{
		// if we don't assign this here, the Foundation object can't create member
		// subobjects which require the allocator

		mInstance = reinterpret_cast<Foundation*>(alloc.allocate(sizeof(Foundation), "Foundation", __FILE__, __LINE__));

		if(mInstance)
		{
			PX_PLACEMENT_NEW(mInstance, Foundation)(errc, alloc);

			PX_ASSERT(mInstance->mRefCount == 0);
			mInstance->mRefCount = 1;

			// skip 0 which marks uninitialized timestaps in PX_WARN_ONCE
			mWarnOnceTimestap = (mWarnOnceTimestap == PX_MAX_U32) ? 1 : mWarnOnceTimestap + 1;

			return mInstance;
		}
		else
		{
			errc.reportError(PxErrorCode::eINTERNAL_ERROR, "Memory allocation for foundation object failed.", __FILE__,
			                 __LINE__);
		}
	}
	else
	{
		errc.reportError(PxErrorCode::eINVALID_OPERATION,
		                 "Foundation object exists already. Only one instance per process can be created.", __FILE__,
		                 __LINE__);
	}

	return 0;
}

void Foundation::destroyInstance()
{
	PX_ASSERT(mInstance != NULL);

	if(mInstance->mRefCount == 1)
	{
		PxAllocatorCallback& alloc = mInstance->getAllocatorCallback();
		mInstance->~Foundation();
		alloc.deallocate(mInstance);
        mInstance = 0;
	}
	else
	{
		mInstance->error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
		                 "Foundation destruction failed due to pending module references. Close/release all depending "
		                 "modules first.");
	}
}

void Foundation::incRefCount()
{
	PX_ASSERT(mInstance != NULL);

	if(mInstance->mRefCount > 0)
        mInstance->mRefCount++;
	else
		mInstance->error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Foundation: Invalid registration detected.");
}

void Foundation::decRefCount()
{
	PX_ASSERT(mInstance != NULL);

	if(mInstance->mRefCount > 0)
        mInstance->mRefCount--;
	else
		mInstance->error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Foundation: Invalid deregistration detected.");
}

void Foundation::release()
{
	Foundation::destroyInstance();
}

PxU32 Foundation::getRefCount()
{
	return mInstance->mRefCount;
}

Foundation* Foundation::mInstance = NULL;
PxU32 Foundation::mWarnOnceTimestap = 0;

void Foundation::registerAllocationListener(physx::PxAllocationListener& listener)
{
	Mutex::ScopedLock lock(mListenerMutex);
	mBroadcastingAllocator.registerListener(listener);
}

void Foundation::deregisterAllocationListener(physx::PxAllocationListener& listener)
{
	Mutex::ScopedLock lock(mListenerMutex);
	mBroadcastingAllocator.deregisterListener(listener);
}

void Foundation::registerErrorCallback(PxErrorCallback& callback)
{
	Mutex::ScopedLock lock(mListenerMutex);
	mBroadcastingError.registerListener(callback);
}

void Foundation::deregisterErrorCallback(PxErrorCallback& callback)
{
	Mutex::ScopedLock lock(mListenerMutex);
	mBroadcastingError.deregisterListener(callback);
}

physx::PxProfilerCallback* gProfilerCallback = NULL;

} // namespace physx

physx::PxFoundation* PxCreateFoundation(physx::PxU32 version, physx::PxAllocatorCallback& allocator,
                                        physx::PxErrorCallback& errorCallback)
{
	return physx::Foundation::createInstance(version, errorCallback, allocator);
}

void PxSetFoundationInstance(physx::PxFoundation& foundation)
{
	physx::Foundation::setInstance(static_cast<physx::Foundation&>(foundation));
}

physx::PxAllocatorCallback* PxGetAllocatorCallback()
{
	return &physx::Foundation::getInstance().getAllocatorCallback();
}

physx::PxAllocatorCallback* PxGetBroadcastAllocator()
{
	return &physx::Foundation::getInstance().getBroadcastAllocator();
}

physx::PxErrorCallback* PX_CALL_CONV PxGetErrorCallback()
{
	return &physx::Foundation::getInstance().getErrorCallback();
}

physx::PxErrorCallback* PX_CALL_CONV PxGetBroadcastError()
{
	return &physx::Foundation::getInstance().getInternalErrorCallback();
}

physx::PxFoundation& PxGetFoundation()
{
	return physx::Foundation::getInstance();
}

physx::PxProfilerCallback* PxGetProfilerCallback()
{
	return physx::gProfilerCallback;
}

void PxSetProfilerCallback(physx::PxProfilerCallback* profiler)
{
	physx::gProfilerCallback = profiler;
}

physx::PxU32 PxGetWarnOnceTimeStamp()
{
	return physx::Foundation::getWarnOnceTimestamp();
}

void PxDecFoundationRefCount()
{
	physx::Foundation::decRefCount();
}

void PxIncFoundationRefCount()
{
	physx::Foundation::incRefCount();
}

