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
