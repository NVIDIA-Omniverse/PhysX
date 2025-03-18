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

#ifndef PXG_MEMORY_TRACKER_H
#define PXG_MEMORY_TRACKER_H

#include "foundation/PxAllocator.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxMutex.h"
#include "foundation/PxMemory.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxString.h"
#include "foundation/PxAssert.h"
#include <stdio.h>

// usage:
//
// create a static MemTracker object in your allocator .cpp
// use registerMemory/unregisterMemory in your allocation/deallocation functions.
//
// please wrap all tracking code in PX_DEBUG to avoid contaminating release builds.
//

struct AllocInfo
{
	const void* mPtr;
	bool mIsGpuPointer;
	physx::PxU64 mNumBytes;
	const char* mFileName;
	physx::PxI32 mLineNumber;

	AllocInfo(const void* ptr, bool isGpuPointer, physx::PxU64 numBytes, const char* fileName, physx::PxI32 lineNumber) :
		mPtr(ptr), mIsGpuPointer(isGpuPointer), mNumBytes(numBytes), mFileName(fileName), mLineNumber(lineNumber)
	{
	}

	PX_FORCE_INLINE void operator = (const AllocInfo& other)
	{
		mPtr = other.mPtr;
		mIsGpuPointer = other.mIsGpuPointer;
		mNumBytes = other.mNumBytes;
		mFileName = other.mFileName;
		mLineNumber = other.mLineNumber;
	}
};

class MemTracker
{
	AllocInfo*		mMemBlockList;
	physx::PxU32 mCapacity;
	physx::PxU32 mNumElementsInUse;
	physx::PxRawAllocator mAllocator;
	physx::PxMutexT<physx::PxRawAllocator> mMutex;

	void doubleSize()
	{
		mCapacity = 2 * mCapacity;
		AllocInfo* mNewPtr = (AllocInfo*)mAllocator.allocate(mCapacity * sizeof(AllocInfo), PX_FL);

		physx::PxMemCopy(reinterpret_cast<void*>(mNewPtr), reinterpret_cast<const void*>(mMemBlockList), mNumElementsInUse * sizeof(AllocInfo));

		mAllocator.deallocate(mMemBlockList);
		mMemBlockList = mNewPtr;
	}

public:
	MemTracker()
	{
		mCapacity = 64;
		mMemBlockList = (AllocInfo*)mAllocator.allocate(mCapacity * sizeof(AllocInfo), PX_FL);
		mNumElementsInUse = 0;
	}

	void registerMemory(void* ptr, bool isGpuMemory, physx::PxU64 numBytes, const char* filename, physx::PxI32 lineNumber)
	{
		physx::PxMutexT<physx::PxRawAllocator>::ScopedLock lock(mMutex);

		if (mNumElementsInUse == mCapacity)
			doubleSize();

		mMemBlockList[mNumElementsInUse] = AllocInfo(ptr, isGpuMemory, numBytes, filename, lineNumber);
		++mNumElementsInUse;
	}

	bool unregisterMemory(void* ptr, bool isGpuMemory)
	{
		physx::PxMutexT<physx::PxRawAllocator>::ScopedLock lock(mMutex);

		if (mMemBlockList)
			for (physx::PxU32 i = 0; i < mNumElementsInUse; ++i)
			{
				if (mMemBlockList[i].mPtr == ptr && mMemBlockList[i].mIsGpuPointer == isGpuMemory)
				{
					mMemBlockList[i] = mMemBlockList[mNumElementsInUse - 1];
					--mNumElementsInUse;
					return true;
				}
			}
		return false;
	}

	void checkForLeaks()
	{
		physx::PxMutexT<physx::PxRawAllocator>::ScopedLock lock(mMutex);

		if (mMemBlockList)
		{
			for (physx::PxU32 i = 0; i < mNumElementsInUse; ++i)
			{
				const AllocInfo& info = mMemBlockList[i];

				if(PxIsFoundationValid()) // error callback requires foundation
				{
					char msg[512];
					physx::Pxsnprintf(msg, 512, "Memory not freed: Ptr: %p, numBytes: %zu, file: %s, line: %i isDeviceMem %u\n", info.mPtr, info.mNumBytes, info.mFileName, info.mLineNumber, info.mIsGpuPointer);
					PxGetErrorCallback()->reportError(physx::PxErrorCode::eINTERNAL_ERROR, msg, PX_FL);
				}
				else
				{
					printf("Memory not freed: Ptr: %p, numBytes: %zu, file: %s, line: %i isDeviceMem %u\n", info.mPtr, info.mNumBytes, info.mFileName, info.mLineNumber, info.mIsGpuPointer);
				}
			}

			// assert to make tests fail.
			//if (mNumElementsInUse > 0)
			//	PX_ALWAYS_ASSERT();
		}
	}

	~MemTracker()
	{
		checkForLeaks();

		if (mMemBlockList)
		{
			mAllocator.deallocate(mMemBlockList);
			mMemBlockList = NULL;
		}
	}
};

#endif
