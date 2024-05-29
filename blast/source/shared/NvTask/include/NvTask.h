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

#ifndef NV_TASK_H
#define NV_TASK_H

#include "NvTaskDefine.h"
#include "NvTaskManager.h"
#include "NvAssert.h"

namespace nvidia
{
    namespace task
    {       

/**
 * \brief Base class of all task types
 *
 * NvBaseTask defines a runnable reference counted task with built-in profiling.
 */
class NvBaseTask
{
public:
    NvBaseTask() : mEventID(0xFFFF), mProfileStat(0), mTm(0) {}
    virtual ~NvBaseTask() {}

    /**
     * \brief The user-implemented run method where the task's work should be performed
     *
     * run() methods must be thread safe, stack friendly (no alloca, etc), and
     * must never block.
     */
    virtual void        run() = 0;

    /**
     * \brief Return a user-provided task name for profiling purposes.
     *
     * It does not have to be unique, but unique names are helpful.
     *
     * \return The name of this task
     */
    virtual const char* getName() const = 0;

    //! \brief Implemented by derived implementation classes
    virtual void        addReference() = 0;
    //! \brief Implemented by derived implementation classes
    virtual void        removeReference() = 0;
    //! \brief Implemented by derived implementation classes
    virtual int32_t     getReference() const = 0;

    /** \brief Implemented by derived implementation classes
     *
     * A task may assume in its release() method that the task system no longer holds 
     * references to it - so it may safely run its destructor, recycle itself, etc.
     * provided no additional user references to the task exist
     */

    virtual void        release() = 0;

    /**
     * \brief Execute user run method with wrapping profiling events.
     *
     * Optional entry point for use by CpuDispatchers.
     *
     * \param[in] threadId The threadId of the thread that executed the task.
     */
    NV_INLINE void runProfiled(uint32_t threadId=0)
    {       
        mTm->emitStartEvent(*this, threadId);
        run();
        mTm->emitStopEvent(*this, threadId);
    }

    /**
     * \brief Specify stop event statistic
     *
     * If called before or while the task is executing, the given value
     * will appear in the task's event bar in the profile viewer
     *
     * \param[in] stat The stat to signal when the task is finished
     */
    NV_INLINE void setProfileStat( uint16_t stat )
    {
        mProfileStat = stat;
    }

    /**
     * \brief Return NvTaskManager to which this task was submitted
     *
     * Note, can return NULL if task was not submitted, or has been
     * completed.
     */
    NV_INLINE NvTaskManager* getTaskManager() const
    {
        return mTm;
    }

protected:
    uint16_t            mEventID;       //!< Registered profile event ID
    uint16_t            mProfileStat;   //!< Profiling statistic
    NvTaskManager*      mTm;            //!< Owning NvTaskManager instance

    friend class NvTaskMgr;
};


/**
 * \brief A NvBaseTask implementation with deferred execution and full dependencies
 *
 * A NvTask must be submitted to a NvTaskManager to to be executed, Tasks may
 * optionally be named when they are submitted.
 */
class NvTask : public NvBaseTask
{
public:
    NvTask() : mTaskID(0) {}
    virtual ~NvTask() {}

    //! \brief Release method implementation
    virtual void release()
    {
        NV_ASSERT(mTm);

        // clear mTm before calling taskCompleted() for safety
        NvTaskManager* save = mTm;
        mTm = NULL;
        save->taskCompleted( *this );
    }

    //! \brief Inform the NvTaskManager this task must finish before the given
    //         task is allowed to start.
    NV_INLINE void finishBefore( NvTaskID taskID )
    {
        NV_ASSERT(mTm);
        mTm->finishBefore( *this, taskID);
    }

    //! \brief Inform the NvTaskManager this task cannot start until the given
    //         task has completed.
    NV_INLINE void startAfter( NvTaskID taskID )
    {
        NV_ASSERT(mTm);
        mTm->startAfter( *this, taskID );
    }

    /**
     * \brief Manually increment this task's reference count.  The task will
     * not be allowed to run until removeReference() is called.
     */
    NV_INLINE void addReference()
    {
        NV_ASSERT(mTm);
        mTm->addReference( mTaskID );
    }

    /**
     * \brief Manually decrement this task's reference count.  If the reference
     * count reaches zero, the task will be dispatched.
     */
    NV_INLINE void removeReference()
    {
        NV_ASSERT(mTm);
        mTm->decrReference( mTaskID );
    }

    /** 
     * \brief Return the ref-count for this task 
     */
    NV_INLINE int32_t getReference() const
    {
        return mTm->getReference( mTaskID );
    }
    
    /**
     * \brief Return the unique ID for this task
     */
    NV_INLINE NvTaskID      getTaskID() const
    {
        return mTaskID;
    }

    /**
     * \brief Called by NvTaskManager at submission time for initialization
     *
     * Perform simulation step initialization here.
     */
    virtual void submitted()
    {
        mStreamIndex = 0;
        mPreSyncRequired = false;
        mProfileStat = 0;
    }

    /**
     * \brief Specify that the GpuTask sync flag be set
     */
    NV_INLINE void      requestSyncPoint()
    {
        mPreSyncRequired = true;
    }


protected:
    NvTaskID            mTaskID;            //!< ID assigned at submission
    uint32_t            mStreamIndex;       //!< GpuTask CUDA stream index
    bool                mPreSyncRequired;   //!< GpuTask sync flag

    friend class NvTaskMgr;
    friend class NvGpuWorkerThread;
};


/**
 * \brief A NvBaseTask implementation with immediate execution and simple dependencies
 *
 * A NvLightCpuTask bypasses the NvTaskManager launch dependencies and will be
 * submitted directly to your scene's CpuDispatcher.  When the run() function
 * completes, it will decrement the reference count of the specified
 * continuation task.
 *
 * You must use a full-blown NvTask if you want your task to be resolved
 * by another NvTask, or you need more than a single dependency to be
 * resolved when your task completes, or your task will not run on the
 * CpuDispatcher.
 */
class NvLightCpuTask : public NvBaseTask
{
public:
    NvLightCpuTask()
        : mCont( NULL )
        , mRefCount( 0 )
    {
    }
    virtual ~NvLightCpuTask()
    {
        mTm = NULL;
    }

    /**
     * \brief Initialize this task and specify the task that will have its ref count decremented on completion.
     *
     * Submission is deferred until the task's mRefCount is decremented to zero.  
     * Note that we only use the NvTaskManager to query the appropriate dispatcher.
     *
     * \param[in] tm The NvTaskManager this task is managed by
     * \param[in] c The task to be executed when this task has finished running
     */
    NV_INLINE void setContinuation(NvTaskManager& tm, NvBaseTask* c)
    {
        NV_ASSERT( mRefCount == 0 );
        mRefCount = 1;
        mCont = c;
        mTm = &tm;
        if( mCont )
        {
            mCont->addReference();
        }
    }

    /**
     * \brief Initialize this task and specify the task that will have its ref count decremented on completion.
     *
     * This overload of setContinuation() queries the NvTaskManager from the continuation
     * task, which cannot be NULL.
     * \param[in] c The task to be executed after this task has finished running
     */
    NV_INLINE void setContinuation( NvBaseTask* c )
    {
        NV_ASSERT( c );
        NV_ASSERT( mRefCount == 0 );
        mRefCount = 1;
        mCont = c;
        if( mCont )
        {
            mCont->addReference();
            mTm = mCont->getTaskManager();
            NV_ASSERT( mTm );
        }
    }

    /**
     * \brief Retrieves continuation task
     */
    NV_INLINE NvBaseTask*   getContinuation()   const
    {
        return mCont;
    }

    /**
     * \brief Manually decrement this task's reference count.  If the reference
     * count reaches zero, the task will be dispatched.
     */
    NV_INLINE void removeReference()
    {
        mTm->decrReference(*this);
    }

    /** \brief Return the ref-count for this task */
    NV_INLINE int32_t getReference() const
    {
        return mRefCount;
    }

    /**
     * \brief Manually increment this task's reference count.  The task will
     * not be allowed to run until removeReference() is called.
     */
    NV_INLINE void addReference()
    {
        mTm->addReference(*this);
    }

    /**
     * \brief called by CpuDispatcher after run method has completed
     *
     * Decrements the continuation task's reference count, if specified.
     */
    NV_INLINE void release()
    {
        if( mCont )
        {
            mCont->removeReference();
        }
    }

protected:

    NvBaseTask*         mCont;          //!< Continuation task, can be NULL
    volatile int32_t    mRefCount;      //!< NvTask is dispatched when reaches 0

    friend class NvTaskMgr;
};


} }// end physx namespace


#endif
