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
		
		PX_FORCE_INLINE	void					initialize(DefaultCpuDispatcher* ownerDispatcher)		{ mOwner = ownerDispatcher;				}
		PX_FORCE_INLINE	PxThread::Id			getWorkerThreadId()								const	{ return mThreadId;						}

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
