// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAssert.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxMutex.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxThread.h"

#include <pthread.h>

namespace physx
{

#if PX_LINUX

#include <sched.h>

static int gMutexProtocol = PTHREAD_PRIO_INHERIT;

PX_FORCE_INLINE bool isLegalProtocol(const int mutexProtocol)
{
	return
	(
		(PTHREAD_PRIO_NONE == mutexProtocol) ||
		(PTHREAD_PRIO_INHERIT == mutexProtocol) ||
		((PTHREAD_PRIO_PROTECT == mutexProtocol) &&  ((sched_getscheduler(0) == SCHED_FIFO) || (sched_getscheduler(0) == SCHED_RR)))
	);
}

bool PxSetMutexProtocol(const int mutexProtocol)
{
	if(isLegalProtocol(mutexProtocol))
	{
		gMutexProtocol = mutexProtocol;
		return true;
	}
	return false;
}

int PxGetMutexProtocol()
{
	return gMutexProtocol;
}

#endif //PX_LINUX


namespace
{
struct MutexUnixImpl
{
	pthread_mutex_t lock;
	PxThread::Id owner;
};

MutexUnixImpl* getMutex(PxMutexImpl* impl)
{
	return reinterpret_cast<MutexUnixImpl*>(impl);
}
}

PxMutexImpl::PxMutexImpl()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
#if PX_LINUX
	pthread_mutexattr_setprotocol(&attr, gMutexProtocol);
	pthread_mutexattr_setprioceiling(&attr, 0);
#endif
	pthread_mutex_init(&getMutex(this)->lock, &attr);
	pthread_mutexattr_destroy(&attr);
}

PxMutexImpl::~PxMutexImpl()
{
	pthread_mutex_destroy(&getMutex(this)->lock);
}

void PxMutexImpl::lock()
{
	int err = pthread_mutex_lock(&getMutex(this)->lock);
	PX_ASSERT(!err);
	PX_UNUSED(err);

#if PX_DEBUG
	getMutex(this)->owner = PxThread::getId();
#endif
}

bool PxMutexImpl::trylock()
{
	bool success = !pthread_mutex_trylock(&getMutex(this)->lock);
#if PX_DEBUG
	if(success)
		getMutex(this)->owner = PxThread::getId();
#endif
	return success;
}

void PxMutexImpl::unlock()
{
#if PX_DEBUG
	if(getMutex(this)->owner != PxThread::getId())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
		                              "Mutex must be unlocked only by thread that has already acquired lock");
		return;
	}
#endif

	int err = pthread_mutex_unlock(&getMutex(this)->lock);
	PX_ASSERT(!err);
	PX_UNUSED(err);
}

uint32_t PxMutexImpl::getSize()
{
	return sizeof(MutexUnixImpl);
}

class ReadWriteLockImpl
{
  public:
	PxMutex mutex;
	volatile int readerCounter;
};

PxReadWriteLock::PxReadWriteLock()
{
	mImpl = reinterpret_cast<ReadWriteLockImpl*>(PX_ALLOC(sizeof(ReadWriteLockImpl), "ReadWriteLockImpl"));
	PX_PLACEMENT_NEW(mImpl, ReadWriteLockImpl);

	mImpl->readerCounter = 0;
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

	PxAtomicIncrement(&mImpl->readerCounter);

	if(takeLock)
		mImpl->mutex.unlock();
}

void PxReadWriteLock::lockWriter()
{
	mImpl->mutex.lock();

	// spin lock until no readers
	while(mImpl->readerCounter);
}

void PxReadWriteLock::unlockReader()
{
	PxAtomicDecrement(&mImpl->readerCounter);
}

void PxReadWriteLock::unlockWriter()
{
	mImpl->mutex.unlock();
}

} // namespace physx
