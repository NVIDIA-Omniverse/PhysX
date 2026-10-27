// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAllocator.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxSList.h"
#include "foundation/PxThread.h"
#include <pthread.h>

#if PX_EMSCRIPTEN
#define USE_MUTEX
#endif

namespace physx
{
namespace
{
#if defined(USE_MUTEX)
class ScopedMutexLock
{
	pthread_mutex_t& mMutex;

  public:
	PX_INLINE ScopedMutexLock(pthread_mutex_t& mutex) : mMutex(mutex)
	{
		pthread_mutex_lock(&mMutex);
	}

	PX_INLINE ~ScopedMutexLock()
	{
		pthread_mutex_unlock(&mMutex);
	}
};

typedef ScopedMutexLock ScopedLock;
#else
struct ScopedSpinLock
{
	PX_FORCE_INLINE ScopedSpinLock(volatile int32_t& lock) : mLock(lock)
	{
		while(__sync_lock_test_and_set(&mLock, 1))
		{
			// spinning without atomics is usually
			// causing less bus traffic. -> only one
			// CPU is modifying the cache line.
			while(lock)
				PxSpinLockPause();
		}
	}

	PX_FORCE_INLINE ~ScopedSpinLock()
	{
		__sync_lock_release(&mLock);
	}

  private:
	volatile int32_t& mLock;
};

typedef ScopedSpinLock ScopedLock;
#endif

struct SListDetail
{
	PxSListEntry* head;
#if defined(USE_MUTEX)
	pthread_mutex_t lock;
#else
	volatile int32_t lock;
#endif
};

template <typename T>
SListDetail* getDetail(T* impl)
{
	return reinterpret_cast<SListDetail*>(impl);
}
}

PxSListImpl::PxSListImpl()
{
	getDetail(this)->head = NULL;

#if defined(USE_MUTEX)
	pthread_mutex_init(&getDetail(this)->lock, NULL);
#else
	getDetail(this)->lock = 0; // 0 == unlocked
#endif
}

PxSListImpl::~PxSListImpl()
{
#if defined(USE_MUTEX)
	pthread_mutex_destroy(&getDetail(this)->lock);
#endif
}

void PxSListImpl::push(PxSListEntry* entry)
{
	ScopedLock lock(getDetail(this)->lock);
	entry->mNext = getDetail(this)->head;
	getDetail(this)->head = entry;
}

PxSListEntry* PxSListImpl::pop()
{
	ScopedLock lock(getDetail(this)->lock);
	PxSListEntry* result = getDetail(this)->head;
	if(result != NULL)
		getDetail(this)->head = result->mNext;
	return result;
}

PxSListEntry* PxSListImpl::flush()
{
	ScopedLock lock(getDetail(this)->lock);
	PxSListEntry* result = getDetail(this)->head;
	getDetail(this)->head = NULL;
	return result;
}

uint32_t PxSListImpl::getSize()
{
	return sizeof(SListDetail);
}

} // namespace physx
