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

#include "foundation/PxAssert.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxThread.h"

#include <math.h>
#if !PX_APPLE_FAMILY && !defined(__CYGWIN__) && !PX_EMSCRIPTEN
#include <bits/local_lim.h> // PTHREAD_STACK_MIN
#endif
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/syscall.h>
#if !PX_APPLE_FAMILY && !PX_EMSCRIPTEN
#include <asm/unistd.h>
#include <sys/resource.h>
#endif

#if PX_APPLE_FAMILY
#include <sys/types.h>
#include <sys/sysctl.h>
#include <TargetConditionals.h>
#include <pthread.h>
#endif

#define PxSpinLockPause() asm("nop")

namespace physx
{

namespace
{

typedef enum
{
	ePxThreadNotStarted,
	ePxThreadStarted,
	ePxThreadStopped
} PxThreadState;

class ThreadImpl
{
  public:
	PxThreadImpl::ExecuteFn fn;
	void* arg;
	volatile int32_t quitNow;
	volatile int32_t threadStarted;
	volatile int32_t state;

	pthread_t thread;
	pid_t tid;

	uint32_t affinityMask;
	const char* name;
};

ThreadImpl* getThread(PxThreadImpl* impl)
{
	return reinterpret_cast<ThreadImpl*>(impl);
}

static void setTid(ThreadImpl& threadImpl)
{
// query TID
// AM: TODO: neither of the below are implemented
#if PX_APPLE_FAMILY
	threadImpl.tid = syscall(SYS_gettid);
#elif PX_EMSCRIPTEN
	threadImpl.tid = pthread_self();
#else
	threadImpl.tid = syscall(__NR_gettid);
#endif

	// notify/unblock parent thread
	PxAtomicCompareExchange(&(threadImpl.threadStarted), 1, 0);
}

void* PxThreadStart(void* arg)
{
	ThreadImpl* impl = getThread(reinterpret_cast<PxThreadImpl*>(arg));
	impl->state = ePxThreadStarted;

	// run setTid in thread's context
	setTid(*impl);

	// then run either the passed in function or execute from the derived class (Runnable).
	if(impl->fn)
		(*impl->fn)(impl->arg);
	else if(impl->arg)
		(reinterpret_cast<PxRunnable*>(impl->arg))->execute();
	return 0;
}
}

uint32_t PxThreadImpl::getSize()
{
	return sizeof(ThreadImpl);
}

PxThreadImpl::Id PxThreadImpl::getId()
{
	return Id(pthread_self());
}

PxThreadImpl::PxThreadImpl()
{
	getThread(this)->thread = 0;
	getThread(this)->tid = 0;
	getThread(this)->state = ePxThreadNotStarted;
	getThread(this)->quitNow = 0;
	getThread(this)->threadStarted = 0;
	getThread(this)->fn = NULL;
	getThread(this)->arg = NULL;
	getThread(this)->affinityMask = 0;
	getThread(this)->name = "set my name before starting me";
}

PxThreadImpl::PxThreadImpl(PxThreadImpl::ExecuteFn fn, void* arg, const char* name)
{
	getThread(this)->thread = 0;
	getThread(this)->tid = 0;
	getThread(this)->state = ePxThreadNotStarted;
	getThread(this)->quitNow = 0;
	getThread(this)->threadStarted = 0;
	getThread(this)->fn = fn;
	getThread(this)->arg = arg;
	getThread(this)->affinityMask = 0;
	getThread(this)->name = name;

	start(0, NULL);
}

PxThreadImpl::~PxThreadImpl()
{
	if(getThread(this)->state == ePxThreadStarted)
		kill();
}

void PxThreadImpl::start(uint32_t stackSize, PxRunnable* runnable)
{
	if(getThread(this)->state != ePxThreadNotStarted)
		return;

	if(stackSize == 0)
		stackSize = getDefaultStackSize();

#if defined(PTHREAD_STACK_MIN)
	if(stackSize < PTHREAD_STACK_MIN)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
		                              "PxThreadImpl::start(): stack size was set below PTHREAD_STACK_MIN");
		stackSize = PTHREAD_STACK_MIN;
	}
#endif

	if(runnable && !getThread(this)->arg && !getThread(this)->fn)
		getThread(this)->arg = runnable;

	pthread_attr_t attr;
	int status = pthread_attr_init(&attr);
	PX_ASSERT(!status);
	PX_UNUSED(status);

	status = pthread_attr_setstacksize(&attr, stackSize);
	PX_ASSERT(!status);
	status = pthread_create(&getThread(this)->thread, &attr, PxThreadStart, this);
	PX_ASSERT(!status);

	// wait for thread to startup and write out TID
	// otherwise TID dependent calls like setAffinity will fail.
	while(PxAtomicCompareExchange(&(getThread(this)->threadStarted), 1, 1) == 0)
		yield();

	// here we are sure that getThread(this)->state >= ePxThreadStarted

	status = pthread_attr_destroy(&attr);
	PX_ASSERT(!status);

	// apply stored affinity mask
	if(getThread(this)->affinityMask)
		setAffinityMask(getThread(this)->affinityMask);

	if (getThread(this)->name)
		setName(getThread(this)->name);
}

void PxThreadImpl::signalQuit()
{
	PxAtomicIncrement(&(getThread(this)->quitNow));
}

bool PxThreadImpl::waitForQuit()
{
	if(getThread(this)->state == ePxThreadNotStarted)
		return false;

	// works also with a stopped/exited thread if the handle is still valid
	pthread_join(getThread(this)->thread, NULL);
	getThread(this)->state = ePxThreadStopped;
	return true;
}

bool PxThreadImpl::quitIsSignalled()
{
	return PxAtomicCompareExchange(&(getThread(this)->quitNow), 0, 0) != 0;
}

#if defined(PX_GCC_FAMILY)
__attribute__((noreturn))
#endif
    void PxThreadImpl::quit()
{
	getThread(this)->state = ePxThreadStopped;
	pthread_exit(0);
}

void PxThreadImpl::kill()
{
	if(getThread(this)->state == ePxThreadStarted)
		pthread_cancel(getThread(this)->thread);
	getThread(this)->state = ePxThreadStopped;
}

void PxThreadImpl::sleep(uint32_t ms)
{
	timespec sleepTime;
	uint32_t remainder = ms % 1000;
	sleepTime.tv_sec = ms - remainder;
	sleepTime.tv_nsec = remainder * 1000000L;

	while(nanosleep(&sleepTime, &sleepTime) == -1)
		continue;
}

void PxThreadImpl::yield()
{
	sched_yield();
}

void PxThreadImpl::yieldProcessor()
{
#if (PX_ARM || PX_A64)
	__asm__ __volatile__("yield");
#else
	__asm__ __volatile__("pause");
#endif
}

uint32_t PxThreadImpl::setAffinityMask(uint32_t mask)
{
	// Same as windows impl if mask is zero
	if(!mask)
		return 0;

	getThread(this)->affinityMask = mask;

	uint64_t prevMask = 0;

	if(getThread(this)->state == ePxThreadStarted)
	{
#if PX_EMSCRIPTEN
		// not supported
#elif !PX_APPLE_FAMILY // Apple doesn't support syscall with getaffinity and setaffinity
		int32_t errGet = syscall(__NR_sched_getaffinity, getThread(this)->tid, sizeof(prevMask), &prevMask);
		if(errGet < 0)
			return 0;

		int32_t errSet = syscall(__NR_sched_setaffinity, getThread(this)->tid, sizeof(mask), &mask);
		if(errSet != 0)
			return 0;
#endif
	}

	return uint32_t(prevMask);
}

void PxThreadImpl::setName(const char* name)
{
	getThread(this)->name = name;

	if (getThread(this)->state == ePxThreadStarted)
	{
		// not implemented because most unix APIs expect setName()
		// to be called from the thread's context. Example see next comment:

		// this works only with the current thread and can rename
		// the main process if used in the wrong context:
		// prctl(PR_SET_NAME, reinterpret_cast<unsigned long>(name) ,0,0,0);
		PX_UNUSED(name);
	}
}

#if !PX_APPLE_FAMILY
static PxThreadPriority::Enum convertPriorityFromLinux(uint32_t inPrio, int policy)
{
	PX_COMPILE_TIME_ASSERT(PxThreadPriority::eLOW > PxThreadPriority::eHIGH);
	PX_COMPILE_TIME_ASSERT(PxThreadPriority::eHIGH == 0);

	int maxL = sched_get_priority_max(policy);
	int minL = sched_get_priority_min(policy);
	int rangeL = maxL - minL;
	int rangeNv = PxThreadPriority::eLOW - PxThreadPriority::eHIGH;

	// case for default scheduler policy
	if(rangeL == 0)
		return PxThreadPriority::eNORMAL;

	float floatPrio = (float(maxL - inPrio) * float(rangeNv)) / float(rangeL);

	return PxThreadPriority::Enum(int(roundf(floatPrio)));
}

static int convertPriorityToLinux(PxThreadPriority::Enum inPrio, int policy)
{
	int maxL = sched_get_priority_max(policy);
	int minL = sched_get_priority_min(policy);
	int rangeL = maxL - minL;
	int rangeNv = PxThreadPriority::eLOW - PxThreadPriority::eHIGH;

	// case for default scheduler policy
	if(rangeL == 0)
		return 0;

	float floatPrio = (float(PxThreadPriority::eLOW - inPrio) * float(rangeL)) / float(rangeNv);

	return minL + int(roundf(floatPrio));
}
#endif

void PxThreadImpl::setPriority(PxThreadPriority::Enum val)
{
	PX_UNUSED(val);
#if !PX_APPLE_FAMILY
	int policy;
	sched_param s_param;
	pthread_getschedparam(getThread(this)->thread, &policy, &s_param);
	s_param.sched_priority = convertPriorityToLinux(val, policy);
	pthread_setschedparam(getThread(this)->thread, policy, &s_param);
#endif
}

PxThreadPriority::Enum PxThreadImpl::getPriority(Id pthread)
{
	PX_UNUSED(pthread);
#if !PX_APPLE_FAMILY
	int policy;
	sched_param s_param;
	int ret = pthread_getschedparam(pthread_t(pthread), &policy, &s_param);
	if(ret == 0)
		return convertPriorityFromLinux(s_param.sched_priority, policy);
	else
		return PxThreadPriority::eNORMAL;
#else
	return PxThreadPriority::eNORMAL;
#endif
}

uint32_t PxThreadImpl::getNbPhysicalCores()
{
#if PX_APPLE_FAMILY
	int count;
	size_t size = sizeof(count);
	return sysctlbyname("hw.physicalcpu", &count, &size, NULL, 0) ? 0 : count;
#else
	// Linux exposes CPU topology using /sys/devices/system/cpu
	// https://www.kernel.org/doc/Documentation/cputopology.txt
	if(FILE* f = fopen("/sys/devices/system/cpu/possible", "r"))
	{
		int minIndex, maxIndex;
		int n = fscanf(f, "%d-%d", &minIndex, &maxIndex);
		fclose(f);

		if(n == 2)
			return (maxIndex - minIndex) + 1;
		else if(n == 1)
			return minIndex + 1;
	}

	// For non-Linux kernels this fallback is possibly the best we can do
	// but will report logical (hyper-threaded) counts
	int n = sysconf(_SC_NPROCESSORS_CONF);
	if(n < 0)
		return 0;
	else
		return n;
#endif
}


PxU32 PxTlsAlloc()
{
	pthread_key_t key;
	int status = pthread_key_create(&key, NULL);
	PX_ASSERT(!status);
	PX_UNUSED(status);
	return PxU32(key);
}

void PxTlsFree(PxU32 index)
{
	int status = pthread_key_delete(pthread_key_t(index));
	PX_ASSERT(!status);
	PX_UNUSED(status);
}

void* PxTlsGet(PxU32 index)
{
	return reinterpret_cast<void*>(pthread_getspecific(pthread_key_t(index)));
}

size_t PxTlsGetValue(PxU32 index)
{
	return reinterpret_cast<size_t>(pthread_getspecific(pthread_key_t(index)));
}

PxU32 PxTlsSet(PxU32 index, void* value)
{
	int status = pthread_setspecific(pthread_key_t(index), value);
	PX_ASSERT(!status);
	return !status;
}

PxU32 PxTlsSetValue(PxU32 index, size_t value)
{
	int status = pthread_setspecific(pthread_key_t(index), reinterpret_cast<void*>(value));
	PX_ASSERT(!status);
	return !status;
}

// DM: On Linux x86-32, without implementation-specific restrictions
// the default stack size for a new thread should be 2 megabytes (kernel.org).
// NOTE: take care of this value on other architectures!
PxU32 PxThreadImpl::getDefaultStackSize()
{
	return 1 << 21;
}

} // namespace physx
