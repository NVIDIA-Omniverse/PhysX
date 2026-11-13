// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_MEMORY_TRACKER_H
#define PXG_MEMORY_TRACKER_H

#include "foundation/PxAllocator.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"
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
	typedef physx::PxHashMap<size_t, AllocInfo, physx::PxHash<size_t>, physx::PxRawAllocator> AllocMap;

	// Indexed by the isGpuMemory flag, so that a host and a device allocation at the same address stay apart.
	AllocMap mAllocs[2];
	physx::PxMutexT<physx::PxRawAllocator> mMutex;

	static void report(const char* problem, const AllocInfo& info)
	{
		if(PxIsFoundationValid()) // error callback requires foundation
		{
			char msg[512];
			physx::Pxsnprintf(msg, 512, "%s: Ptr: %p, numBytes: %zu, file: %s, line: %i isDeviceMem %u\n", problem, info.mPtr, info.mNumBytes, info.mFileName, info.mLineNumber, info.mIsGpuPointer);
			PxGetErrorCallback()->reportError(physx::PxErrorCode::eINTERNAL_ERROR, msg, PX_FL);
		}
		else
		{
			printf("%s: Ptr: %p, numBytes: %zu, file: %s, line: %i isDeviceMem %u\n", problem, info.mPtr, info.mNumBytes, info.mFileName, info.mLineNumber, info.mIsGpuPointer);
		}
	}

public:
	void registerMemory(void* ptr, bool isGpuMemory, physx::PxU64 numBytes, const char* filename, physx::PxI32 lineNumber)
	{
		physx::PxMutexT<physx::PxRawAllocator>::ScopedLock lock(mMutex);

		AllocMap& allocs = mAllocs[isGpuMemory];
		const AllocMap::Entry* previous = allocs.find(size_t(ptr));
		if (previous)
		{
			// The previous owner of the address freed it without unregistering it.
			report("Memory registered again without being unregistered", previous->second);
			allocs.erase(size_t(ptr));
		}
		allocs.insert(size_t(ptr), AllocInfo(ptr, isGpuMemory, numBytes, filename, lineNumber));
	}

	bool unregisterMemory(void* ptr, bool isGpuMemory)
	{
		physx::PxMutexT<physx::PxRawAllocator>::ScopedLock lock(mMutex);

		return mAllocs[isGpuMemory].erase(size_t(ptr));
	}

	void checkForLeaks()
	{
		physx::PxMutexT<physx::PxRawAllocator>::ScopedLock lock(mMutex);

		for (physx::PxU32 i = 0; i < 2; ++i)
		{
			for (AllocMap::Iterator iter = mAllocs[i].getIterator(); !iter.done(); ++iter)
			{
				report("Memory not freed", iter->second);
			}
		}

		// assert to make tests fail.
		//if (mAllocs[0].size() + mAllocs[1].size() > 0)
		//	PX_ALWAYS_ASSERT();
	}

	~MemTracker()
	{
		checkForLeaks();
	}
};

#endif
