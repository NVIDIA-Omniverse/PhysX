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

#include "task/PxTask.h"
#include "foundation/PxErrors.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"

#include "foundation/PxThread.h"

#define LOCK()  PxMutex::ScopedLock _lock_(mMutex)

namespace physx
{
    const int EOL = -1;
	typedef PxHashMap<const char *, PxTaskID> PxTaskNameToIDMap;

	struct PxTaskDepTableRow
	{
		PxTaskID    mTaskID;
		int       mNextDep;
	};
	typedef PxArray<PxTaskDepTableRow> PxTaskDepTable;

	class PxTaskTableRow
	{
	public:
		PxTaskTableRow() : mRefCount( 1 ), mStartDep(EOL), mLastDep(EOL) {}
		void addDependency( PxTaskDepTable& depTable, PxTaskID taskID )
		{
			int newDep = int(depTable.size());
			PxTaskDepTableRow row;
			row.mTaskID = taskID;
			row.mNextDep = EOL;
			depTable.pushBack( row );

			if( mLastDep == EOL )
			{
				mStartDep = mLastDep = newDep;
			}
			else
			{
				depTable[ uint32_t(mLastDep) ].mNextDep = newDep;
				mLastDep = newDep;
			}
		}

		PxTask *    mTask;
		volatile int mRefCount;
		PxTaskType::Enum mType;
		int       mStartDep;
		int       mLastDep;
	};
	typedef PxArray<PxTaskTableRow> PxTaskTable;


/* Implementation of PxTaskManager abstract API */
class PxTaskMgr : public PxTaskManager, public PxUserAllocated
{
	PX_NOCOPY(PxTaskMgr)
public:
	PxTaskMgr(PxErrorCallback& , PxCpuDispatcher*);
	~PxTaskMgr();

	void     setCpuDispatcher( PxCpuDispatcher& ref )
	{
		mCpuDispatcher = &ref;
	}

	PxCpuDispatcher* getCpuDispatcher() const
	{
		return mCpuDispatcher;
	}

	void	resetDependencies();
	void	startSimulation();
	void	stopSimulation();
	void	taskCompleted( PxTask& task );

	PxTaskID  getNamedTask( const char *name );
	PxTaskID  submitNamedTask( PxTask *task, const char *name, PxTaskType::Enum type = PxTaskType::eCPU );
	PxTaskID  submitUnnamedTask( PxTask& task, PxTaskType::Enum type = PxTaskType::eCPU );
	PxTask*   getTaskFromID( PxTaskID );

	void    dispatchTask( PxTaskID taskID );
	void    resolveRow( PxTaskID taskID );

	void    release();

	void	finishBefore( PxTask& task, PxTaskID taskID );
	void	startAfter( PxTask& task, PxTaskID taskID );

	void	addReference( PxTaskID taskID );
	void	decrReference( PxTaskID taskID );
	int32_t	getReference( PxTaskID taskID ) const;

	void	decrReference( PxLightCpuTask& lighttask );
	void	addReference( PxLightCpuTask& lighttask );		

	PxErrorCallback&			mErrorCallback;
	PxCpuDispatcher           *mCpuDispatcher;
	PxTaskNameToIDMap          mName2IDmap;
	volatile int			 mPendingTasks;
    PxMutex            mMutex;

	PxTaskDepTable				 mDepTable;
	PxTaskTable				 mTaskTable;

	PxArray<PxTaskID>	 mStartDispatch;
	};

PxTaskManager* PxTaskManager::createTaskManager(PxErrorCallback& errorCallback, PxCpuDispatcher* cpuDispatcher)
{
	return PX_NEW(PxTaskMgr)(errorCallback, cpuDispatcher);
}

PxTaskMgr::PxTaskMgr(PxErrorCallback& errorCallback, PxCpuDispatcher* cpuDispatcher)
	: mErrorCallback (errorCallback)
	, mCpuDispatcher( cpuDispatcher )
	, mPendingTasks( 0 )
	, mDepTable("PxTaskDepTable")
	, mTaskTable("PxTaskTable")
	, mStartDispatch("StartDispatch")
{
}

PxTaskMgr::~PxTaskMgr()
{
}

void PxTaskMgr::release()
{
	PX_DELETE_THIS;
}

void PxTaskMgr::decrReference(PxLightCpuTask& lighttask)
{
	/* This does not need a lock! */
	if (!PxAtomicDecrement(&lighttask.mRefCount))
	{
		PX_ASSERT(mCpuDispatcher);
		if (mCpuDispatcher)
		{
			mCpuDispatcher->submitTask(lighttask);
		}
		else
		{
			lighttask.release();
		}
	}
}

void PxTaskMgr::addReference(PxLightCpuTask& lighttask)
{
	/* This does not need a lock! */
	PxAtomicIncrement(&lighttask.mRefCount);
}

/*
 * Called by the owner (Scene) at the start of every frame, before
 * asking for tasks to be submitted.
 */
void PxTaskMgr::resetDependencies()
{
	PX_ASSERT( !mPendingTasks ); // only valid if you don't resubmit named tasks, this is true for the SDK
    PX_ASSERT( mCpuDispatcher );
    mTaskTable.clear();
    mDepTable.clear();
    mName2IDmap.clear();
    mPendingTasks = 0;
}

/* 
 * Called by the owner (Scene) to start simulating the task graph.
 * Dispatch all tasks with refCount == 1
 */
void PxTaskMgr::startSimulation()
{
    PX_ASSERT( mCpuDispatcher );

	/* Handle empty task graph */
	if( mPendingTasks == 0 )
		return;

    for( PxTaskID i = 0 ; i < mTaskTable.size() ; i++ )
    {
		if(	mTaskTable[ i ].mType == PxTaskType::eCOMPLETED )
		{
			continue;
		}
		if( !PxAtomicDecrement( &mTaskTable[ i ].mRefCount ) )
		{
			mStartDispatch.pushBack(i);
		}
	}
	for( uint32_t i=0; i<mStartDispatch.size(); ++i)
	{
		dispatchTask( mStartDispatch[i] );
	}
	//mStartDispatch.resize(0);
	mStartDispatch.forceSize_Unsafe(0);
}

void PxTaskMgr::stopSimulation()
{
}

PxTaskID PxTaskMgr::getNamedTask( const char *name )
{
	const PxTaskNameToIDMap::Entry *ret;
    {
        LOCK();
		ret = mName2IDmap.find( name );
    }
    if( ret )
	{
        return ret->second;
	}
    else
	{
        // create named entry in task table, without a task
        return submitNamedTask( NULL, name, PxTaskType::eNOT_PRESENT );
	}
}

PxTask* PxTaskMgr::getTaskFromID( PxTaskID id )
{
	LOCK(); // todo: reader lock necessary?
	return mTaskTable[ id ].mTask;
}

/* If called at runtime, must be thread-safe */
PxTaskID PxTaskMgr::submitNamedTask( PxTask *task, const char *name, PxTaskType::Enum type )
{
    if( task )
    {
        task->mTm = this;
        task->submitted();
    }

    LOCK();

	const PxTaskNameToIDMap::Entry *ret = mName2IDmap.find( name );
    if( ret )
    {
		PxTaskID prereg = ret->second;
		if( task )
		{
			/* name was registered for us by a dependent task */
			PX_ASSERT( !mTaskTable[ prereg ].mTask );
			PX_ASSERT( mTaskTable[ prereg ].mType == PxTaskType::eNOT_PRESENT );
			mTaskTable[ prereg ].mTask = task;
			mTaskTable[ prereg ].mType = type;
			task->mTaskID = prereg;
		}
		return prereg;
    }
    else
    {
        PxAtomicIncrement(&mPendingTasks);
        PxTaskID id = static_cast<PxTaskID>(mTaskTable.size());
        mName2IDmap[ name ] = id;
        if( task )
		{
            task->mTaskID = id;
		}
        PxTaskTableRow r;
        r.mTask = task;
        r.mType = type;
		mTaskTable.pushBack(r);
        return id;
    }
}

/*
 * Add an unnamed task to the task table
 */
PxTaskID PxTaskMgr::submitUnnamedTask( PxTask& task, PxTaskType::Enum type )
{
    PxAtomicIncrement(&mPendingTasks);

	task.mTm = this;
    task.submitted();
    
	LOCK();
    task.mTaskID = static_cast<PxTaskID>(mTaskTable.size());
    PxTaskTableRow r;
    r.mTask = &task;
    r.mType = type;
    mTaskTable.pushBack(r);
    return task.mTaskID;
}

/* Called by worker threads (or cooperating application threads) when a
 * PxTask has completed.  Propogate depdenencies, decrementing all
 * referenced tasks' refCounts.  If any of those reach zero, activate
 * those tasks.
 */
void PxTaskMgr::taskCompleted( PxTask& task )
{
    LOCK();
	resolveRow(task.mTaskID);
}

/* ================== Private Functions ======================= */

/*
 * Add a dependency to force 'task' to complete before the
 * referenced 'taskID' is allowed to be dispatched.
 */
void PxTaskMgr::finishBefore( PxTask& task, PxTaskID taskID )
{
    LOCK();
	PX_ASSERT( mTaskTable[ taskID ].mType != PxTaskType::eCOMPLETED );

    mTaskTable[ task.mTaskID ].addDependency( mDepTable, taskID );
	PxAtomicIncrement( &mTaskTable[ taskID ].mRefCount );
}

/*
 * Add a dependency to force 'task' to wait for the referenced 'taskID'
 * to complete before it is allowed to be dispatched.
 */
void PxTaskMgr::startAfter( PxTask& task, PxTaskID taskID )
{
    LOCK();
	PX_ASSERT( mTaskTable[ taskID ].mType != PxTaskType::eCOMPLETED );

    mTaskTable[ taskID ].addDependency( mDepTable, task.mTaskID );
	PxAtomicIncrement( &mTaskTable[ task.mTaskID ].mRefCount );
}

void PxTaskMgr::addReference( PxTaskID taskID )
{
    LOCK();
    PxAtomicIncrement( &mTaskTable[ taskID ].mRefCount );
}

/*
 * Remove one reference count from a task. Must be done here to make it thread safe.
 */
void PxTaskMgr::decrReference( PxTaskID taskID )
{
    LOCK();

    if( !PxAtomicDecrement( &mTaskTable[ taskID ].mRefCount ) )
    {
		dispatchTask(taskID);
    }
}

int32_t PxTaskMgr::getReference(PxTaskID taskID) const
{
	return mTaskTable[ taskID ].mRefCount;
}

/*
 * A task has completed, decrement all dependencies and submit tasks
 * that are ready to run.  Signal simulation end if ther are no more
 * pending tasks.
 */
void PxTaskMgr::resolveRow( PxTaskID taskID )
{
    int depRow = mTaskTable[ taskID ].mStartDep;

    while( depRow != EOL )
    {
        PxTaskDepTableRow& row = mDepTable[ uint32_t(depRow) ];
        PxTaskTableRow& dtt = mTaskTable[ row.mTaskID ];

        if( !PxAtomicDecrement( &dtt.mRefCount ) )
		{
			dispatchTask( row.mTaskID );
		}

        depRow = row.mNextDep;
    }

    PxAtomicDecrement( &mPendingTasks );
}

/*
 * Submit a ready task to its appropriate dispatcher.
 */
void PxTaskMgr::dispatchTask( PxTaskID taskID )
{
	LOCK(); // todo: reader lock necessary?
    PxTaskTableRow& tt = mTaskTable[ taskID ];

	// prevent re-submission
    if( tt.mType == PxTaskType::eCOMPLETED )
    {		
		mErrorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "PxTask dispatched twice", PX_FL);
		return;
    }

    switch ( tt.mType )
    {
    case PxTaskType::eCPU:
        mCpuDispatcher->submitTask( *tt.mTask );
        break;
    case PxTaskType::eNOT_PRESENT:
		/* No task registered with this taskID, resolve its dependencies */
		PX_ASSERT(!tt.mTask);
		//PxGetFoundation().error(PX_INFO, "unregistered task resolved");
        resolveRow( taskID );
		break;
	case PxTaskType::eCOMPLETED:
    default:
        mErrorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "Unknown task type", PX_FL);
        resolveRow( taskID );
        break;
    }

    tt.mType = PxTaskType::eCOMPLETED;
}

}// end physx namespace
