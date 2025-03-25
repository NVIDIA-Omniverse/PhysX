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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "task/PxTask.h"
#include "ExtCpuWorkerThread.h"
#include "ExtDefaultCpuDispatcher.h"
#include "foundation/PxFPU.h"

using namespace physx;

Ext::CpuWorkerThread::CpuWorkerThread() : mOwner(NULL), mThreadId(0)
{
}

Ext::CpuWorkerThread::~CpuWorkerThread()
{
}

#define HighPriority	true
#define RegularPriority	false

void Ext::CpuWorkerThread::execute()
{
	mThreadId = getId();

	const PxDefaultCpuDispatcherWaitForWorkMode::Enum ownerWaitForWorkMode = mOwner->getWaitForWorkMode();

	while(!quitIsSignalled())
    {
		if(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == ownerWaitForWorkMode)
			mOwner->resetWakeSignal();

		// PT: look for high priority tasks first, across threads
		PxBaseTask* task = getJob<HighPriority>();
		if(!task)
			task = mOwner->fetchNextTask<HighPriority>();

		// PT: then look for regular tasks
		if(!task)
			task = getJob<RegularPriority>();
		if(!task)
			task = mOwner->fetchNextTask<RegularPriority>();

		if(task)
		{
			mOwner->runTask(*task);
			task->release();
		}
		else if(PxDefaultCpuDispatcherWaitForWorkMode::eYIELD_THREAD == ownerWaitForWorkMode)
		{
			PxThread::yield();
		}
		else if(PxDefaultCpuDispatcherWaitForWorkMode::eYIELD_PROCESSOR == ownerWaitForWorkMode)
		{
			const PxU32 pauseCounter = mOwner->getYieldProcessorCount();
			for(PxU32 j = 0; j < pauseCounter; j++)
				PxThread::yieldProcesor();
		}
		else
		{
			PX_ASSERT(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == ownerWaitForWorkMode);
			mOwner->waitForWork();
		}
	}

	quit();
}
