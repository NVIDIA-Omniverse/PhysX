// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_CPU_WORKER_THREAD_H
#define EXT_CPU_WORKER_THREAD_H

#include "foundation/PxThread.h"
#include "ExtTaskQueueHelper.h"
#include "ExtSharedQueueEntryPool.h"

namespace physx
{
namespace Ext
{
class DefaultCpuDispatcher;

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif							// Because of the SList member I assume

	class CpuWorkerThread : public PxThread
	{
	public:
												CpuWorkerThread();
												~CpuWorkerThread();
		
		PX_FORCE_INLINE	void					initialize(DefaultCpuDispatcher* ownerDispatcher)		{ mOwner = ownerDispatcher;	}
		PX_FORCE_INLINE	PxThread::Id			getWorkerThreadId()								const	{ return mThreadId;			}

		template<const bool highPriorityT>
		PX_FORCE_INLINE	PxBaseTask*				getJob()	{ return mHelper.fetchTask<highPriorityT>();	}

						void					execute();

		PX_FORCE_INLINE	bool					tryAcceptJobToLocalQueue(PxBaseTask& task, PxThread::Id taskSubmitionThread)
												{
													if(taskSubmitionThread == mThreadId)
														return mHelper.tryAcceptJobToQueue(task);
													return false;
												}
	protected:
						DefaultCpuDispatcher*	mOwner;
						TaskQueueHelper			mHelper;
						PxThread::Id			mThreadId;
	};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Ext

}

#endif
