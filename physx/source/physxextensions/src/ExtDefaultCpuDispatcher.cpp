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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ExtDefaultCpuDispatcher.h"
#include "ExtCpuWorkerThread.h"
#include "ExtTaskQueueHelper.h"
#include "foundation/PxString.h"

using namespace physx;

PxDefaultCpuDispatcher* physx::PxDefaultCpuDispatcherCreate(PxU32 numThreads, PxU32* affinityMasks, PxDefaultCpuDispatcherWaitForWorkMode::Enum mode, PxU32 yieldProcessorCount)
{
	return PX_NEW(Ext::DefaultCpuDispatcher)(numThreads, affinityMasks, mode, yieldProcessorCount);
}

#if !PX_SWITCH
void Ext::DefaultCpuDispatcher::getAffinityMasks(PxU32* affinityMasks, PxU32 threadCount)
{
	for(PxU32 i=0; i < threadCount; i++)
	{
		affinityMasks[i] = 0;
	}
}
#endif

Ext::DefaultCpuDispatcher::DefaultCpuDispatcher(PxU32 numThreads, PxU32* affinityMasks, PxDefaultCpuDispatcherWaitForWorkMode::Enum mode, PxU32 yieldProcessorCount)
	: mQueueEntryPool(EXT_TASK_QUEUE_ENTRY_POOL_SIZE, "QueueEntryPool"), mNumThreads(numThreads), mShuttingDown(false)
#if PX_PROFILE
	,mRunProfiled(true)
#else
	,mRunProfiled(false)
#endif
	, mWaitForWorkMode(mode)
	, mYieldProcessorCount(yieldProcessorCount)
{
	PX_CHECK_MSG((((PxDefaultCpuDispatcherWaitForWorkMode::eYIELD_PROCESSOR == mWaitForWorkMode) && (mYieldProcessorCount > 0)) ||
					(((PxDefaultCpuDispatcherWaitForWorkMode::eYIELD_THREAD == mWaitForWorkMode) || (PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == mWaitForWorkMode)) && (0 == mYieldProcessorCount))), "Illegal yield processor count for chosen execute mode");

	PxU32* defaultAffinityMasks = NULL;

	if(!affinityMasks)
	{
		defaultAffinityMasks = PX_ALLOCATE(PxU32, numThreads, "ThreadAffinityMasks");
		getAffinityMasks(defaultAffinityMasks, numThreads);
		affinityMasks = defaultAffinityMasks;
	}
	 
	// initialize threads first, then start

	mWorkerThreads = PX_ALLOCATE(CpuWorkerThread, numThreads, "CpuWorkerThread");
	const PxU32 nameLength = 32;
	mThreadNames = PX_ALLOCATE(PxU8, nameLength * numThreads, "CpuWorkerThreadName");

	if (mWorkerThreads)
	{
		for(PxU32 i = 0; i < numThreads; ++i)
		{
			PX_PLACEMENT_NEW(mWorkerThreads+i, CpuWorkerThread)();
			mWorkerThreads[i].initialize(this);
		}

		for(PxU32 i = 0; i < numThreads; ++i)
		{
			if (mThreadNames)
			{
				char* threadName = reinterpret_cast<char*>(mThreadNames + (i*nameLength));
				Pxsnprintf(threadName, nameLength, "PxWorker%02d", i);
				mWorkerThreads[i].setName(threadName);
			}

			mWorkerThreads[i].setAffinityMask(affinityMasks[i]);
			mWorkerThreads[i].start(PxThread::getDefaultStackSize());
		}

		PX_FREE(defaultAffinityMasks);
	}
	else
	{
		mNumThreads = 0;
	}
}

Ext::DefaultCpuDispatcher::~DefaultCpuDispatcher()
{
	for(PxU32 i = 0; i < mNumThreads; ++i)
		mWorkerThreads[i].signalQuit();

	mShuttingDown = true;
	if(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == mWaitForWorkMode)
		mWorkReady.set();
	for(PxU32 i = 0; i < mNumThreads; ++i)
		mWorkerThreads[i].waitForQuit();

	for(PxU32 i = 0; i < mNumThreads; ++i)
		mWorkerThreads[i].~CpuWorkerThread();

	PX_FREE(mWorkerThreads);
	PX_FREE(mThreadNames);
}

void Ext::DefaultCpuDispatcher::release()
{
	PX_DELETE_THIS;
}

void Ext::DefaultCpuDispatcher::submitTask(PxBaseTask& task)
{
	if(!mNumThreads)
	{
		// no worker threads, run directly
		runTask(task);
		task.release();
		return;
	}	

	// TODO: Could use TLS to make this more efficient
	const PxThread::Id currentThread = PxThread::getId();
	const PxU32 nbThreads = mNumThreads;
	for(PxU32 i=0; i<nbThreads; ++i)
	{
		if(mWorkerThreads[i].tryAcceptJobToLocalQueue(task, currentThread))
		{
			if(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == mWaitForWorkMode)
			{
				return mWorkReady.set();
			}
			else
			{
				PX_ASSERT(PxDefaultCpuDispatcherWaitForWorkMode::eYIELD_PROCESSOR == mWaitForWorkMode || PxDefaultCpuDispatcherWaitForWorkMode::eYIELD_THREAD == mWaitForWorkMode);
				return;
			}
		}
	}

	SharedQueueEntry* entry = mQueueEntryPool.getEntry(&task);
	if(entry)
	{
		mJobList.push(*entry);
		if(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == mWaitForWorkMode)
			mWorkReady.set();
	}
}

PxBaseTask* Ext::DefaultCpuDispatcher::fetchNextTask()
{
	PxBaseTask* task = getJob();

	if(!task)
		task = stealJob();

	return task;
}

PxBaseTask* Ext::DefaultCpuDispatcher::getJob()
{
	return TaskQueueHelper::fetchTask(mJobList, mQueueEntryPool);
}

PxBaseTask* Ext::DefaultCpuDispatcher::stealJob()
{
	const PxU32 nbThreads = mNumThreads;
	for(PxU32 i=0; i<nbThreads; ++i)
	{
		PxBaseTask* ret = mWorkerThreads[i].giveUpJob();
		if(ret)
			return ret;
	}
	return NULL;
}

void Ext::DefaultCpuDispatcher::resetWakeSignal()
{
	PX_ASSERT(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == mWaitForWorkMode);
	mWorkReady.reset();
	
	// The code below is necessary to avoid deadlocks on shut down.
	// A thread usually loops as follows:
	// while quit is not signaled
	// 1)  reset wake signal
	// 2)  fetch work
	// 3)  if work -> process
	// 4)  else -> wait for wake signal
	//
	// If a thread reaches 1) after the thread pool signaled wake up,
	// the wake up sync gets reset and all other threads which have not
	// passed 4) already will wait forever.
	// The code below makes sure that on shutdown, the wake up signal gets
	// sent again after it was reset
	//
	if (mShuttingDown)
		mWorkReady.set();
}
