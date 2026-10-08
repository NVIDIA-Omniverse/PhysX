// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "FdFoundation.h"
#include "foundation/PxString.h"
#include "foundation/PxPhysicsVersion.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxBroadcast.h"

namespace physx
{
#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4251) // class needs to have dll-interface to be used by clients of class
#endif

class PX_FOUNDATION_API Foundation : public PxFoundation, public PxUserAllocated
{
	PX_NOCOPY(Foundation)

  public:
	// PxFoundation
	virtual	void					release()								PX_OVERRIDE;
	virtual PxErrorCallback&		getErrorCallback()						PX_OVERRIDE	{ return mErrorCallback;			}
	virtual void					setErrorLevel(PxErrorCode::Enum mask)	PX_OVERRIDE	{ mErrorMask = mask;				}
	virtual	PxErrorCode::Enum		getErrorLevel() const					PX_OVERRIDE	{ return mErrorMask;				}
	virtual PxAllocatorCallback&	getAllocatorCallback()					PX_OVERRIDE	{ return mAllocatorCallback;		}
	virtual bool					getReportAllocationNames() const		PX_OVERRIDE	{ return mReportAllocationNames;	}
	virtual void					setReportAllocationNames(bool value)	PX_OVERRIDE	{ mReportAllocationNames = value;	}
	virtual void					registerAllocationListener(physx::PxAllocationListener& listener)	PX_OVERRIDE;
	virtual void					deregisterAllocationListener(physx::PxAllocationListener& listener)	PX_OVERRIDE;
	virtual void					registerErrorCallback(PxErrorCallback& listener)	PX_OVERRIDE;
	virtual void					deregisterErrorCallback(PxErrorCallback& listener)	PX_OVERRIDE;
	virtual bool					error(PxErrorCode::Enum, const char* file, int line, const char* messageFmt, ...)		PX_OVERRIDE;
	virtual bool					error(PxErrorCode::Enum, const char* file, int line, const char* messageFmt, va_list)	PX_OVERRIDE;
	//~PxFoundation

	Foundation(PxErrorCallback& errc, PxAllocatorCallback& alloc);
	~Foundation();

	// init order is tricky here: the mutexes require the allocator, the allocator may require the error stream
	PxAllocatorCallback& mAllocatorCallback;
	PxErrorCallback& mErrorCallback;

	PxBroadcastingAllocator mBroadcastingAllocator;
	PxBroadcastingErrorCallback mBroadcastingError;

	bool mReportAllocationNames;

	PxErrorCode::Enum mErrorMask;
	Mutex mErrorMutex;

	AllocFreeTable mTempAllocFreeTable;
	Mutex mTempAllocMutex;

	Mutex mListenerMutex;

    PxU32 mRefCount;
	static PxU32 mWarnOnceTimestap;
};
#if PX_VC
#pragma warning(pop)
#endif

} // namespace physx

using namespace physx;

static PxProfilerCallback* gProfilerCallback = NULL;
static Foundation* gInstance = NULL;

// PT: not in header so that people don't use it, only for temp allocator, will be removed
AllocFreeTable& getTempAllocFreeTable()
{
	PX_ASSERT(gInstance);
	return gInstance->mTempAllocFreeTable;
}

// PT: not in header so that people don't use it, only for temp allocator, will be removed
Mutex& getTempAllocMutex()
{
	PX_ASSERT(gInstance);
	return gInstance->mTempAllocMutex;
}

Foundation::Foundation(PxErrorCallback& errc, PxAllocatorCallback& alloc) :
	mAllocatorCallback		(alloc),
	mErrorCallback			(errc),
	mBroadcastingAllocator	(alloc, errc),
	mBroadcastingError		(errc),
#if PX_CHECKED
    mReportAllocationNames	(true),
#else
    mReportAllocationNames	(false),
#endif
    mErrorMask				(PxErrorCode::Enum(~0)),
	mErrorMutex				("Foundation::mErrorMutex"),
	mTempAllocMutex			("Foundation::mTempAllocMutex"),
	mRefCount				(0)
{
}

void deallocateTempBufferAllocations(AllocFreeTable& mTempAllocFreeTable);

Foundation::~Foundation()
{
	deallocateTempBufferAllocations(mTempAllocFreeTable);
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

void Foundation::release()
{
	PX_ASSERT(gInstance);

	if(gInstance->mRefCount == 1)
	{
		PxAllocatorCallback& alloc = gInstance->getAllocatorCallback();
		gInstance->~Foundation();
		alloc.deallocate(gInstance);
        gInstance = NULL;
	}
	else
	{
		gInstance->error(PxErrorCode::eINVALID_OPERATION, PX_FL,
		                 "Foundation destruction failed due to pending module references. Close/release all depending modules first.");
	}
}

PxU32 Foundation::mWarnOnceTimestap = 0;

void Foundation::registerAllocationListener(PxAllocationListener& listener)
{
	Mutex::ScopedLock lock(mListenerMutex);
	mBroadcastingAllocator.registerListener(listener);
}

void Foundation::deregisterAllocationListener(PxAllocationListener& listener)
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

PxFoundation* PxCreateFoundation(PxU32 version, PxAllocatorCallback& allocator, PxErrorCallback& errorCallback)
{
	if(version != PX_PHYSICS_VERSION)
	{
		char buffer[256];
		Pxsnprintf(buffer, 256, "Wrong version: physics version is 0x%08x, tried to create 0x%08x", PX_PHYSICS_VERSION, version);
		errorCallback.reportError(PxErrorCode::eINVALID_PARAMETER, buffer, PX_FL);
		return 0;
	}

	if(!gInstance)
	{
		// if we don't assign this here, the Foundation object can't create member
		// subobjects which require the allocator

		gInstance = reinterpret_cast<Foundation*>(allocator.allocate(sizeof(Foundation), "Foundation", PX_FL));

		if(gInstance)
		{
			PX_PLACEMENT_NEW(gInstance, Foundation)(errorCallback, allocator);

			PX_ASSERT(gInstance->mRefCount == 0);
			gInstance->mRefCount = 1;

			// skip 0 which marks uninitialized timestaps in PX_WARN_ONCE
			gInstance->mWarnOnceTimestap = (gInstance->mWarnOnceTimestap == PX_MAX_U32) ? 1 : gInstance->mWarnOnceTimestap + 1;

			return gInstance;
		}
		else
		{
			errorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, "Memory allocation for foundation object failed.", PX_FL);
		}
	}
	else
	{
		errorCallback.reportError(PxErrorCode::eINVALID_OPERATION, "Foundation object exists already. Only one instance per process can be created.", PX_FL);
	}

	return 0;
}

void PxSetFoundationInstance(PxFoundation& foundation)
{
	gInstance = &static_cast<Foundation&>(foundation);
}

PxAllocatorCallback* PxGetAllocatorCallback()
{
	return &gInstance->getAllocatorCallback();
}

PxAllocatorCallback* PxGetBroadcastAllocator(bool* reportAllocationNames)
{
	PX_ASSERT(gInstance);
	if(reportAllocationNames)
		*reportAllocationNames = gInstance->mReportAllocationNames;

	return &gInstance->mBroadcastingAllocator;
}

PxErrorCallback* PX_CALL_CONV PxGetErrorCallback()
{
	return &gInstance->getErrorCallback();
}

PxErrorCallback* PX_CALL_CONV PxGetBroadcastError()
{
	return &gInstance->mBroadcastingError;
}

PxFoundation& PxGetFoundation()
{
	PX_ASSERT(gInstance);
	return *gInstance;
}

PxFoundation* PxIsFoundationValid()
{
	return gInstance;
}

PxProfilerCallback* PxGetProfilerCallback()
{
	return gProfilerCallback;
}

void PxSetProfilerCallback(PxProfilerCallback* profiler)
{
	gProfilerCallback = profiler;
}

PxU32 PxGetWarnOnceTimeStamp()
{
	PX_ASSERT(gInstance);
	return gInstance->mWarnOnceTimestap;
}

void PxDecFoundationRefCount()
{
	PX_ASSERT(gInstance);

	if(gInstance->mRefCount > 0)
        gInstance->mRefCount--;
	else
		gInstance->error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Foundation: Invalid deregistration detected.");
}

void PxIncFoundationRefCount()
{
	PX_ASSERT(gInstance);

	if(gInstance->mRefCount > 0)
        gInstance->mRefCount++;
	else
		gInstance->error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Foundation: Invalid registration detected.");
}

// Private method to set the global foundation instance to NULL
PX_C_EXPORT PX_FOUNDATION_API void PX_CALL_CONV PxResetFoundationInstance()
{
	gInstance = NULL;
}
