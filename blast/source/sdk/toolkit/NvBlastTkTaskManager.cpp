//
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
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
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

#include "NvTask.h"
#include "NvTaskDefine.h"
#include "NvCpuDispatcher.h"
#include "NvGpuDispatcher.h"
#include "NvErrorCallback.h"

#include "NvBlastGlobals.h"
#include "NvBlastAssert.h"
#include "NvBlastAtomic.h"
#include "NvBlastAllocator.h"
#include "NvBlastArray.h"
#include "NvBlastHashMap.h"

#include <mutex>

using namespace nvidia;
using namespace nvidia::task;

namespace Nv
{
namespace Blast
{

class MutexScopedLock
{
    std::mutex& mMutex;
    NV_NOCOPY(MutexScopedLock)

public:
    NV_INLINE MutexScopedLock(std::mutex& mutex) : mMutex(mutex) { mMutex.lock(); }
    NV_INLINE ~MutexScopedLock() { mMutex.unlock(); }
};

#define LOCK()  MutexScopedLock __lock__(mMutex)

constexpr int EOL = -1;
typedef HashMap<const char *, NvTaskID>::type NvBlastTkTaskNameToIDMap;

struct NvBlastTkTaskDepTableRow
{
    NvTaskID    mTaskID;
    int         mNextDep;
};
typedef Array<NvBlastTkTaskDepTableRow>::type NvBlastTkTaskDepTable;


struct NvTaskAccess : public NvTask
{
    void setTaskID(NvTaskID taskID) { mTaskID = taskID; }
    void setTm(NvTaskManager* tm) { mTm = tm; }
};
NvTaskAccess& ACCESS(NvTask& task) { return reinterpret_cast<NvTaskAccess&>(task); }
NvTaskAccess* ACCESS(NvTask* task) { return reinterpret_cast<NvTaskAccess*>(task); }

struct NvLightCpuTaskAccess : public NvLightCpuTask
{
    bool atomicIncrementRefCount() { return Nv::Blast::atomicIncrement(&mRefCount); }
    bool atomicDecrementRefCount() { return Nv::Blast::atomicDecrement(&mRefCount); }
};
NvLightCpuTaskAccess& ACCESS(NvLightCpuTask& task) { return reinterpret_cast<NvLightCpuTaskAccess&>(task); }

class NvBlastTkTaskTableRow
{
public:
    NvBlastTkTaskTableRow() : mRefCount( 1 ), mStartDep(EOL), mLastDep(EOL) {}
    void addDependency( NvBlastTkTaskDepTable& depTable, NvTaskID taskID )
    {
        int newDep = int(depTable.size());
        NvBlastTkTaskDepTableRow row;
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

    NvTask *    mTask;
    volatile int mRefCount;
    NvTaskType::Enum mType;
    int       mStartDep;
    int       mLastDep;
};
typedef Array<NvBlastTkTaskTableRow>::type NvTaskTable;


/* Implementation of NvTaskManager abstract API */
class NvBlastTkTaskManager : public NvTaskManager
{
    NV_NOCOPY(NvBlastTkTaskManager)
public:
    NvBlastTkTaskManager(NvErrorCallback& , NvCpuDispatcher*, NvGpuDispatcher*);
    ~NvBlastTkTaskManager();

    void     setCpuDispatcher( NvCpuDispatcher& ref )
    {
        mCpuDispatcher = &ref;
    }

    NvCpuDispatcher* getCpuDispatcher() const
    {
        return mCpuDispatcher;
    }

    void     setGpuDispatcher( NvGpuDispatcher& ref )
    {
        mGpuDispatcher = &ref;
    }

    NvGpuDispatcher* getGpuDispatcher() const
    {
        return mGpuDispatcher;
    }

    void    resetDependencies();
    void    startSimulation();
    void    stopSimulation();
    void    taskCompleted( NvTask& task );

    NvTaskID  getNamedTask( const char *name );
    NvTaskID  submitNamedTask( NvTask *task, const char *name, NvTaskType::Enum type = NvTaskType::TT_CPU );
    NvTaskID  submitUnnamedTask( NvTask& task, NvTaskType::Enum type = NvTaskType::TT_CPU );
    NvTask*   getTaskFromID( NvTaskID );

    bool    dispatchTask( NvTaskID taskID, bool gpuGroupStart );
    bool    resolveRow( NvTaskID taskID, bool gpuGroupStart );

    void    release();

    void    finishBefore( NvTask& task, NvTaskID taskID );
    void    startAfter( NvTask& task, NvTaskID taskID );

    void    addReference( NvTaskID taskID );
    void    decrReference( NvTaskID taskID );
    int32_t getReference( NvTaskID taskID ) const;

    void    decrReference( NvLightCpuTask& lighttask );
    void    addReference( NvLightCpuTask& lighttask );      

    void    emitStartEvent(NvBaseTask& basetask, uint32_t threadId);
    void    emitStopEvent(NvBaseTask& basetask, uint32_t threadId);

    NvErrorCallback&            mErrorCallback;
    NvCpuDispatcher*            mCpuDispatcher;
    NvGpuDispatcher*            mGpuDispatcher;
    NvBlastTkTaskNameToIDMap    mName2IDmap;
    volatile int                mPendingTasks;
    std::mutex                  mMutex;

    NvBlastTkTaskDepTable       mDepTable;
    NvTaskTable                 mTaskTable;

    Array<NvTaskID>::type       mStartDispatch;
};

NvBlastTkTaskManager::NvBlastTkTaskManager(NvErrorCallback& errorCallback, NvCpuDispatcher* cpuDispatcher, NvGpuDispatcher* gpuDispatcher)
    : mErrorCallback (errorCallback)
    , mCpuDispatcher( cpuDispatcher )
    , mGpuDispatcher( gpuDispatcher )
    , mPendingTasks( 0 )
    , mDepTable(NV_DEBUG_EXP("NvBlastTkTaskDepTable"))
    , mTaskTable(NV_DEBUG_EXP("NvTaskTable"))   
    , mStartDispatch(NV_DEBUG_EXP("StartDispatch"))
{
}

NvBlastTkTaskManager::~NvBlastTkTaskManager()
{
}

void NvBlastTkTaskManager::release()
{
    NVBLAST_DELETE(this, NvBlastTkTaskManager);
}

void NvBlastTkTaskManager::decrReference(NvLightCpuTask& lighttask)
{
    /* This does not need a lock! */
    if (!ACCESS(lighttask).atomicDecrementRefCount())
    {
        NVBLAST_ASSERT(mCpuDispatcher);
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

void NvBlastTkTaskManager::addReference(NvLightCpuTask& lighttask)
{
    /* This does not need a lock! */
    ACCESS(lighttask).atomicIncrementRefCount();
}

void NvBlastTkTaskManager::emitStartEvent(NvBaseTask& basetask, uint32_t threadId)
{
    NvBaseTask* tmp = &basetask;
    NV_UNUSED(tmp);
    NV_UNUSED(threadId);

    /* This does not need a lock! */
#if NV_SUPPORT_NVTASK_PROFILING && NV_PROFILE
    //NV_COMPILE_TIME_ASSERT(sizeof(NvProfileEventId::mEventId) == sizeof(NvBaseTask::mEventID));
    if (NvBlastGlobalGetProfilerCallback()) NvBlastGlobalGetProfilerCallback()->zoneStart(basetask.getName(), true, 0);
#endif
}

void NvBlastTkTaskManager::emitStopEvent(NvBaseTask& basetask, uint32_t threadId)
{
    NvBaseTask* tmp = &basetask;
    NV_UNUSED(tmp);
    NV_UNUSED(threadId);

    /* This does not need a lock! */
    if (NvBlastGlobalGetProfilerCallback()) NvBlastGlobalGetProfilerCallback()->zoneEnd(nullptr, basetask.getName(), true, 0);
#if NV_SUPPORT_NVTASK_PROFILING && NV_PROFILE
    //NV_COMPILE_TIME_ASSERT(sizeof(NvProfileEventId::mEventId) == sizeof(NvBaseTask::mEventID));
#endif
}

/*
 * Called by the owner (Scene) at the start of every frame, before
 * asking for tasks to be submitted.
 */
void NvBlastTkTaskManager::resetDependencies()
{
    NVBLAST_ASSERT( !mPendingTasks ); // only valid if you don't resubmit named tasks, this is true for the SDK
    NVBLAST_ASSERT( mCpuDispatcher );
    mTaskTable.clear();
    mDepTable.clear();
    mName2IDmap.clear();
    mPendingTasks = 0;
}

/* 
 * Called by the owner (Scene) to start simulating the task graph.
 * Dispatch all tasks with refCount == 1
 */
void NvBlastTkTaskManager::startSimulation()
{
    NVBLAST_ASSERT( mCpuDispatcher );

    if( mGpuDispatcher )
    {
        mGpuDispatcher->startSimulation();
    }

    /* Handle empty task graph */
    if( mPendingTasks == 0 )
    {

        return;
    }

    bool gpuDispatch = false;
    for( NvTaskID i = 0 ; i < mTaskTable.size() ; i++ )
    {
        if( mTaskTable[ i ].mType == NvTaskType::TT_COMPLETED )
        {
            continue;
        }
        if( !Nv::Blast::atomicDecrement( &mTaskTable[ i ].mRefCount ) )
        {
            mStartDispatch.pushBack(i);
        }
    }
    for( uint32_t i=0; i<mStartDispatch.size(); ++i)
    {
        gpuDispatch |= dispatchTask( mStartDispatch[i], gpuDispatch );
    }
    //mStartDispatch.resize(0);
    mStartDispatch.forceSize_Unsafe(0);

    if( mGpuDispatcher && gpuDispatch )
    {
        mGpuDispatcher->finishGroup();
    }
}

void NvBlastTkTaskManager::stopSimulation()
{
    if( mGpuDispatcher )
    {
        mGpuDispatcher->stopSimulation();
    }
}

NvTaskID NvBlastTkTaskManager::getNamedTask( const char *name )
{
    const NvBlastTkTaskNameToIDMap::Entry *ret;
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
        return submitNamedTask( NULL, name, NvTaskType::TT_NOT_PRESENT );
}
}

NvTask* NvBlastTkTaskManager::getTaskFromID( NvTaskID id )
{
    LOCK(); // todo: reader lock necessary?
    return mTaskTable[ id ].mTask;
}


/* If called at runtime, must be thread-safe */
NvTaskID NvBlastTkTaskManager::submitNamedTask( NvTask *task, const char *name, NvTaskType::Enum type )
{
    if( task )
    {
        ACCESS(task)->setTm(this);
        task->submitted();
    }

    LOCK();

    const NvBlastTkTaskNameToIDMap::Entry *ret = mName2IDmap.find( name );
    if( ret )
    {
        NvTaskID prereg = ret->second;
        if( task )
        {
            /* name was registered for us by a dependent task */
            NVBLAST_ASSERT( !mTaskTable[ prereg ].mTask );
            NVBLAST_ASSERT( mTaskTable[ prereg ].mType == NvTaskType::TT_NOT_PRESENT );
            mTaskTable[ prereg ].mTask = task;
            mTaskTable[ prereg ].mType = type;
            ACCESS(task)->setTaskID(prereg);
        }
        return prereg;
    }
    else
    {
        Nv::Blast::atomicIncrement(&mPendingTasks);
        NvTaskID id = static_cast<NvTaskID>(mTaskTable.size());
        mName2IDmap[ name ] = id;
        if( task )
        {
            ACCESS(task)->setTaskID(id);
        }
        NvBlastTkTaskTableRow r;
        r.mTask = task;
        r.mType = type;
        mTaskTable.pushBack(r);
        return id;
    }
}

/*
 * Add an unnamed task to the task table
 */
NvTaskID NvBlastTkTaskManager::submitUnnamedTask( NvTask& task, NvTaskType::Enum type )
{
    Nv::Blast::atomicIncrement(&mPendingTasks);

    ACCESS(task).setTm(this);
    task.submitted();
    
    LOCK();
    ACCESS(task).setTaskID(static_cast<NvTaskID>(mTaskTable.size()));
    NvBlastTkTaskTableRow r;
    r.mTask = &task;
    r.mType = type;
    mTaskTable.pushBack(r);
    return task.getTaskID();
}


/* Called by worker threads (or cooperating application threads) when a
 * NvTask has completed.  Propogate depdenencies, decrementing all
 * referenced tasks' refCounts.  If any of those reach zero, activate
 * those tasks.
 */
void NvBlastTkTaskManager::taskCompleted( NvTask& task )
{
    LOCK();
    if( resolveRow( task.getTaskID(), false ) )
    {
        mGpuDispatcher->finishGroup();
    }
}

/* ================== Private Functions ======================= */

/*
 * Add a dependency to force 'task' to complete before the
 * referenced 'taskID' is allowed to be dispatched.
 */
void NvBlastTkTaskManager::finishBefore( NvTask& task, NvTaskID taskID )
{
    LOCK();
    NVBLAST_ASSERT( mTaskTable[ taskID ].mType != NvTaskType::TT_COMPLETED );

    mTaskTable[ task.getTaskID() ].addDependency( mDepTable, taskID );
    Nv::Blast::atomicIncrement( &mTaskTable[ taskID ].mRefCount );
}


/*
 * Add a dependency to force 'task' to wait for the referenced 'taskID'
 * to complete before it is allowed to be dispatched.
 */
void NvBlastTkTaskManager::startAfter( NvTask& task, NvTaskID taskID )
{
    LOCK();
    NVBLAST_ASSERT( mTaskTable[ taskID ].mType != NvTaskType::TT_COMPLETED );

    mTaskTable[ taskID ].addDependency( mDepTable, task.getTaskID() );
    Nv::Blast::atomicIncrement( &mTaskTable[ task.getTaskID() ].mRefCount );
}


void NvBlastTkTaskManager::addReference( NvTaskID taskID )
{
    LOCK();
    Nv::Blast::atomicIncrement( &mTaskTable[ taskID ].mRefCount );
}

/*
 * Remove one reference count from a task. Must be done here to make it thread safe.
 */
void NvBlastTkTaskManager::decrReference( NvTaskID taskID )
{
    LOCK();

    if( !Nv::Blast::atomicDecrement( &mTaskTable[ taskID ].mRefCount ) )
    {
        if( dispatchTask( taskID, false ) )
        {
            mGpuDispatcher->finishGroup();
        }
    }
}

int32_t NvBlastTkTaskManager::getReference(NvTaskID taskID) const
{
    return mTaskTable[ taskID ].mRefCount;
}

/*
 * A task has completed, decrement all dependencies and submit tasks
 * that are ready to run.  Signal simulation end if ther are no more
 * pending tasks.
 */
bool NvBlastTkTaskManager::resolveRow( NvTaskID taskID, bool gpuGroupStart )
{
    int depRow = mTaskTable[ taskID ].mStartDep;


    while( depRow != EOL )
    {
        NvBlastTkTaskDepTableRow& row = mDepTable[ uint32_t(depRow) ];
        NvBlastTkTaskTableRow& dtt = mTaskTable[ row.mTaskID ];

        if( !Nv::Blast::atomicDecrement( &dtt.mRefCount ) )
        {
            gpuGroupStart |= dispatchTask( row.mTaskID, gpuGroupStart );
        }

        depRow = row.mNextDep;
    }

    Nv::Blast::atomicDecrement( &mPendingTasks );
    return gpuGroupStart;
}

/*
 * Submit a ready task to its appropriate dispatcher.
 */
bool NvBlastTkTaskManager::dispatchTask( NvTaskID taskID, bool gpuGroupStart )
{
    LOCK(); // todo: reader lock necessary?
    NvBlastTkTaskTableRow& tt = mTaskTable[ taskID ];

    // prevent re-submission
    if( tt.mType == NvTaskType::TT_COMPLETED )
    {       
        mErrorCallback.reportError(NvErrorCode::eDEBUG_WARNING, "NvTask dispatched twice", __FILE__, __LINE__);
        return false;
    }

    switch ( tt.mType )
    {
    case NvTaskType::TT_CPU:
        mCpuDispatcher->submitTask( *tt.mTask );
        break;
    case NvTaskType::TT_GPU:
#if NV_WINDOWS_FAMILY
        if( mGpuDispatcher )
        {
            if( !gpuGroupStart )
            {
                mGpuDispatcher->startGroup();
            }
            mGpuDispatcher->submitTask( *tt.mTask );
            gpuGroupStart = true;
        }
        else
#endif
        {
            mErrorCallback.reportError(NvErrorCode::eDEBUG_WARNING, "No GPU dispatcher", __FILE__, __LINE__);
        }
        break;
    case NvTaskType::TT_NOT_PRESENT:
        /* No task registered with this taskID, resolve its dependencies */
        NVBLAST_ASSERT(!tt.mTask);
        gpuGroupStart |= resolveRow( taskID, gpuGroupStart );
        break;
    case NvTaskType::TT_COMPLETED:
    default:
        mErrorCallback.reportError(NvErrorCode::eDEBUG_WARNING, "Unknown task type", __FILE__, __LINE__);
        gpuGroupStart |= resolveRow( taskID, gpuGroupStart );
        break;
    }

    tt.mType = NvTaskType::TT_COMPLETED;
    return gpuGroupStart;
}

} // namespace Blast
} // namespace Nv

// Implement NvTaskManager factory
namespace nvidia
{
namespace task
{

NvTaskManager* NvTaskManager::createTaskManager(NvErrorCallback& errorCallback, NvCpuDispatcher* cpuDispatcher, NvGpuDispatcher* gpuDispatcher)
{
    return NVBLAST_NEW(Nv::Blast::NvBlastTkTaskManager)(errorCallback, cpuDispatcher, gpuDispatcher);
}

}
}
