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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PX_FOUNDATION_PSFOUNDATION_H
#define PX_FOUNDATION_PSFOUNDATION_H

#include "foundation/PxErrors.h"
#include "foundation/PxProfiler.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxBroadcast.h"
#include "foundation/PxTempAllocator.h"
#include "foundation/PxMutex.h"

#include <stdarg.h>

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
	typedef PxMutexT<PxAllocator> Mutex;
	typedef PxArray<PxTempAllocatorChunk*, PxAllocator> AllocFreeTable;

  public:
	// factory
	// note, you MUST eventually call release if createInstance returned true!
	static Foundation* createInstance(PxU32 version, PxErrorCallback& errc, PxAllocatorCallback& alloc);
	static void setInstance(Foundation& foundation);
	void release();
	static void incRefCount(); // this call requires a foundation object to exist already
	static void decRefCount(); // this call requires a foundation object to exist already
	static PxU32 getRefCount();

	// Begin Errors
	virtual PxErrorCallback& getErrorCallback()
	{
		return mErrorCallback;
	} // Return the user's error callback
	PxErrorCallback& getInternalErrorCallback()
	{
		return mBroadcastingError;
	} // Return the broadcasting error callback

	virtual void registerErrorCallback(PxErrorCallback& listener);
	virtual void deregisterErrorCallback(PxErrorCallback& listener);

	virtual void setErrorLevel(PxErrorCode::Enum mask)
	{
		mErrorMask = mask;
	}
	virtual PxErrorCode::Enum getErrorLevel() const
	{
		return mErrorMask;
	}

	virtual bool error(PxErrorCode::Enum, const char* file, int line, const char* messageFmt, ...);		// Report errors with the
									                                                                    // broadcasting
	virtual bool error(PxErrorCode::Enum, const char* file, int line, const char* messageFmt, va_list); // error callback

	static PxU32 getWarnOnceTimestamp();

	// End errors

	// Begin Allocations
	virtual PxAllocatorCallback& getAllocatorCallback()
	{
		return mAllocatorCallback;
	} // Return the user's allocator callback
	PxAllocatorCallback& getBroadcastAllocator()
	{
		return mBroadcastingAllocator;
	} // Return the broadcasting allocator

	virtual void registerAllocationListener(physx::PxAllocationListener& listener);
	virtual void deregisterAllocationListener(physx::PxAllocationListener& listener);

	virtual bool getReportAllocationNames() const
	{
		return mReportAllocationNames;
	}
	virtual void setReportAllocationNames(bool value)
	{
		mReportAllocationNames = value;
	}

	PX_INLINE AllocFreeTable& getTempAllocFreeTable()
	{
		return mTempAllocFreeTable;
	}
	PX_INLINE Mutex& getTempAllocMutex()
	{
		return mTempAllocMutex;
	}
	// End allocations

  //private:
	static void destroyInstance();

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

Foundation& getFoundation();

} // namespace physx


#endif
