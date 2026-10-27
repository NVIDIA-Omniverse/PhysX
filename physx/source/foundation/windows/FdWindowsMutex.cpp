// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/windows/PxWindowsInclude.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxMutex.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxThread.h"

namespace physx
{
namespace
{
struct MutexWinImpl
{
	CRITICAL_SECTION mLock;
	PxThread::Id mOwner;
};
}

static PX_FORCE_INLINE MutexWinImpl* getMutex(PxMutexImpl* impl)
{
	return reinterpret_cast<MutexWinImpl*>(impl);
}

PxMutexImpl::PxMutexImpl()
{
	InitializeCriticalSection(&getMutex(this)->mLock);
	getMutex(this)->mOwner = 0;
}

PxMutexImpl::~PxMutexImpl()
{
	DeleteCriticalSection(&getMutex(this)->mLock);
}

void PxMutexImpl::lock()
{
	EnterCriticalSection(&getMutex(this)->mLock);

#if PX_DEBUG
	getMutex(this)->mOwner = PxThread::getId();
#endif
}

bool PxMutexImpl::trylock()
{
	bool success = TryEnterCriticalSection(&getMutex(this)->mLock) != 0;
#if PX_DEBUG
	if(success)
		getMutex(this)->mOwner = PxThread::getId();
#endif
	return success;
}

void PxMutexImpl::unlock()
{
#if PX_DEBUG
	// ensure we are already holding the lock
	if(getMutex(this)->mOwner != PxThread::getId())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Mutex must be unlocked only by thread that has already acquired lock");
		return;
	}
#endif

	LeaveCriticalSection(&getMutex(this)->mLock);
}

uint32_t PxMutexImpl::getSize()
{
	return sizeof(MutexWinImpl);
}

class ReadWriteLockImpl
{
	PX_NOCOPY(ReadWriteLockImpl)
  public:
	ReadWriteLockImpl()
	{
	}
	PxMutex mutex;
	volatile LONG readerCount; // handle recursive writer locking
};

PxReadWriteLock::PxReadWriteLock()
{
	mImpl = reinterpret_cast<ReadWriteLockImpl*>(PX_ALLOC(sizeof(ReadWriteLockImpl), "ReadWriteLockImpl"));
	PX_PLACEMENT_NEW(mImpl, ReadWriteLockImpl);

	mImpl->readerCount = 0;
}

PxReadWriteLock::~PxReadWriteLock()
{
	mImpl->~ReadWriteLockImpl();
	PX_FREE(mImpl);
}

void PxReadWriteLock::lockReader(bool takeLock)
{
	if(takeLock)
		mImpl->mutex.lock();

	InterlockedIncrement(&mImpl->readerCount);

	if(takeLock)
		mImpl->mutex.unlock();
}

void PxReadWriteLock::lockWriter()
{
	mImpl->mutex.lock();

	// spin lock until no readers
	while(mImpl->readerCount);
}

void PxReadWriteLock::unlockReader()
{
	InterlockedDecrement(&mImpl->readerCount);
}

void PxReadWriteLock::unlockWriter()
{
	mImpl->mutex.unlock();
}

} // namespace physx
