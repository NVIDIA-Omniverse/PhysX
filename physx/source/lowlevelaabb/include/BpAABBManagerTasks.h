// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BP_AABB_MANAGER_TASKS_H
#define BP_AABB_MANAGER_TASKS_H

#include "CmTask.h"

namespace physx
{
namespace Bp
{
	class AABBManager;
	class Aggregate;

	class AggregateBoundsComputationTask : public Cm::Task
	{
		PX_NOCOPY(AggregateBoundsComputationTask)
		public:
								AggregateBoundsComputationTask(PxU64 contextId) :
									Cm::Task	(contextId),
									mManager	(NULL),
									mStart		(0),
									mNbToGo		(0),
									mAggregates	(NULL)
								{}

		virtual const char*		getName() const PX_OVERRIDE { return "AggregateBoundsComputationTask"; }
		virtual void			runInternal() PX_OVERRIDE;

				void			Init(AABBManager* manager, PxU32 start, PxU32 nb, Aggregate** aggregates)
								{
									mManager	= manager;
									mStart		= start;
									mNbToGo		= nb;
									mAggregates	= aggregates;
								}
		private:
				AABBManager*	mManager;
				PxU32			mStart;
				PxU32			mNbToGo;
				Aggregate**		mAggregates;
	};

	class PreBpUpdateTask : public Cm::Task
	{
		PX_NOCOPY(PreBpUpdateTask)
	public:
								PreBpUpdateTask(PxU64 contextId) : Cm::Task(contextId), mManager(NULL), mNumCpuTasks(0)	{}

		virtual const char*		getName() const PX_OVERRIDE { return "PreBpUpdateTask"; }
		virtual void			runInternal() PX_OVERRIDE;

				void			Init(AABBManager* manager, PxU32 numCpuTasks)
								{
									mManager = manager;
									mNumCpuTasks = numCpuTasks;
								}
	private:
				AABBManager*	mManager;
				PxU32			mNumCpuTasks;
	};

}
} //namespace physx

#endif // BP_AABB_MANAGER_TASKS_H
