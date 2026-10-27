// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_TASK_QUEUE_HELPER_H
#define EXT_TASK_QUEUE_HELPER_H

#include "task/PxTask.h"
#include "ExtSharedQueueEntryPool.h"

namespace physx
{

#define EXT_TASK_QUEUE_ENTRY_POOL_SIZE 128
#define EXT_TASK_QUEUE_ENTRY_HIGH_PRIORITY_POOL_SIZE 32

namespace Ext
{
	class TaskQueueHelper
	{
		SharedQueueEntryPool<>	mQueueEntryPool;
		PxSList					mJobList;

		SharedQueueEntryPool<>	mHighPriorityQueueEntryPool;
		PxSList					mHighPriorityJobList;

	public:

		TaskQueueHelper() : mQueueEntryPool(EXT_TASK_QUEUE_ENTRY_POOL_SIZE, "QueueEntryPool"),
							mHighPriorityQueueEntryPool(EXT_TASK_QUEUE_ENTRY_HIGH_PRIORITY_POOL_SIZE, "HighPriorityQueueEntryPool")
		{}

		PX_FORCE_INLINE	bool	tryAcceptJobToQueue(PxBaseTask& task)
		{
			if(task.isHighPriority())
			{
				SharedQueueEntry* entry = mHighPriorityQueueEntryPool.getEntry(&task);
				if(entry)
				{
					mHighPriorityJobList.push(*entry);
					return true;
				}
			}

			SharedQueueEntry* entry = mQueueEntryPool.getEntry(&task);
			if(entry)
			{
				mJobList.push(*entry);
				return true;
			}
			else
			{
				return false;	// PT: we never actually reach this
			}
		}

		template<const bool highPriorityT>
		PxBaseTask* fetchTask()
		{
			SharedQueueEntry* entry = highPriorityT ? static_cast<SharedQueueEntry*>(mHighPriorityJobList.pop()) : static_cast<SharedQueueEntry*>(mJobList.pop());
			if(entry)
			{
				PxBaseTask* task = reinterpret_cast<PxBaseTask*>(entry->mObjectRef);
				if(highPriorityT)
					mHighPriorityQueueEntryPool.putEntry(*entry);
				else
					mQueueEntryPool.putEntry(*entry);
				return task;
			}
			return NULL;
		}
	};

} // namespace Ext

}

#endif
