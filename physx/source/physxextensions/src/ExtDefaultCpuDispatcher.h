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

#ifndef EXT_DEFAULT_CPU_DISPATCHER_H
#define EXT_DEFAULT_CPU_DISPATCHER_H

#include "common/PxProfileZone.h"
#include "task/PxTask.h"
#include "extensions/PxDefaultCpuDispatcher.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxSync.h"
#include "foundation/PxSList.h"
#include "ExtSharedQueueEntryPool.h"

namespace physx
{
namespace Ext
{
	class CpuWorkerThread;

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif							// Because of the SList member I assume

	class DefaultCpuDispatcher : public PxDefaultCpuDispatcher, public PxUserAllocated
	{
		friend class TaskQueueHelper;
																		PX_NOCOPY(DefaultCpuDispatcher)
	private:
																		~DefaultCpuDispatcher();
	public:
																		DefaultCpuDispatcher(PxU32 numThreads, PxU32* affinityMasks, PxDefaultCpuDispatcherWaitForWorkMode::Enum mode = PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK, PxU32 yieldProcessorCount = 0);

		// PxCpuDispatcher
		virtual			void											submitTask(PxBaseTask& task)		PX_OVERRIDE;
		virtual			PxU32											getWorkerCount()	const			PX_OVERRIDE	{ return mNumThreads;			}
		//~PxCpuDispatcher

		// PxDefaultCpuDispatcher
		virtual			void											release()							PX_OVERRIDE;
		virtual			void											setRunProfiled(bool runProfiled)	PX_OVERRIDE	{ mRunProfiled = runProfiled;	}
		virtual			bool											getRunProfiled()	const			PX_OVERRIDE	{ return mRunProfiled;			}
		//~PxDefaultCpuDispatcher

						PxBaseTask*										getJob();
						PxBaseTask*										stealJob();
						PxBaseTask*										fetchNextTask();

		PX_FORCE_INLINE	void											runTask(PxBaseTask& task)
																		{
																			if(mRunProfiled)
																			{
																				PX_PROFILE_ZONE(task.getName(), task.getContextId());
																				task.run();
																			}
																			else
																				task.run();
																		}

    					void											waitForWork()						{ PX_ASSERT(PxDefaultCpuDispatcherWaitForWorkMode::eWAIT_FOR_WORK == mWaitForWorkMode); mWorkReady.wait(); }
						void											resetWakeSignal();

		static			void											getAffinityMasks(PxU32* affinityMasks, PxU32 threadCount);

		PX_FORCE_INLINE	PxDefaultCpuDispatcherWaitForWorkMode::Enum		getWaitForWorkMode()		const	{ return mWaitForWorkMode;		}
		PX_FORCE_INLINE	PxU32											getYieldProcessorCount()	const	{ return mYieldProcessorCount;	}

	protected:
						CpuWorkerThread*								mWorkerThreads;
						SharedQueueEntryPool<>							mQueueEntryPool;
						PxSList											mJobList;
						PxSync											mWorkReady;
						PxU8*											mThreadNames;
						PxU32											mNumThreads;
						bool											mShuttingDown;
						bool											mRunProfiled;
		const			PxDefaultCpuDispatcherWaitForWorkMode::Enum		mWaitForWorkMode;
		const			PxU32											mYieldProcessorCount;
	};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Ext
}

#endif
