// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
