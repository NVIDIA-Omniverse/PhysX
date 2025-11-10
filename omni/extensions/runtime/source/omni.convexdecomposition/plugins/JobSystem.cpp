// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include "JobSystem.h"
#include <queue>
#include <atomic>

#ifdef _MSC_VER
#    pragma warning(disable : 4100)
#endif

using namespace carb::tasking;

// Scoped mutex lock
using lock_guard = std::lock_guard<MutexWrapper>;

namespace vcd
{


class SimpleJob
{
public:
    SimpleJob(void* userPtr, SJS_jobCallback callback) : mUserPointer(userPtr), mCallback(callback)
    {
    }

    void execute(void)
    {
        (mCallback)(mUserPointer);
    }

    void* mUserPointer{ nullptr };
    SJS_jobCallback mCallback;
};

typedef std::queue<SimpleJob*> SimpleJobQueue;

class FindJobs
{
public:
    virtual SimpleJob* getSimpleJob(void) = 0;
    virtual void simpleJobComplete(SimpleJob* sj) = 0;
};

class SimpleJobThread
{
public:
    SimpleJobThread(void)
    {
        mTasking = carb::getCachedInterface<ITasking>();
    }

    ~SimpleJobThread(void)
    {
        stopThread();
    }

    void stopThread(void)
    {
        if (mThread.valid())
        {
            mExit = true;
            mHaveWork.notify_one();
            mThread.wait();
        }
    }

    void start(FindJobs* fj)
    {
        mFindJobs = fj;
        if (!mThread.valid())
        {
            mThread = mTasking->addTask(carb::tasking::Priority::eDefault, {}, [this]() { runThread(); });
        }
        mHaveWork.notify_all();
    }

    void runThread(void)
    {
        while (!mExit)
        {
            // Process jobs while jobs are available.
            // If no more jobs are available, go to sleep until fresh work is provided
            SimpleJob* sj = mFindJobs->getSimpleJob();
            while (sj)
            {
                sj->execute();
                mFindJobs->simpleJobComplete(sj);
                sj = mFindJobs->getSimpleJob();
            }
            {
                std::unique_lock<MutexWrapper> lock(mWorkMutex);
                mHaveWork.wait(mWorkMutex);
            }
        }
    }


    FindJobs* mFindJobs{ nullptr };
    std::atomic<bool> mExit{ false };
    Future<> mThread;
    MutexWrapper mWorkMutex;
    ConditionVariableWrapper mHaveWork;

    ITasking* mTasking{ nullptr };
};

class JobSystemImpl : public JobSystem, public FindJobs
{
public:
    JobSystemImpl(uint32_t maxThreads)
        : mMaxThreads(maxThreads)
    {
        mThreads = new SimpleJobThread[mMaxThreads];
    }

    virtual ~JobSystemImpl(void)
    {
        waitForJobsToComplete();
        delete[] mThreads;
    }

    // Add a job to the queue of jobs to be performed, does not actually start the job yet.
    virtual void addJob(void* userPtr, SJS_jobCallback callback) final
    {
        lock_guard _lock(mSimpleJobMutex);
        SimpleJob* sj = new SimpleJob(userPtr, callback);
        mPendingJobs.push(sj);
    }

    // Start the jobs that have been posted, returns how many jobs are pending.
    virtual uint32_t startJobs(void) final
    {
        lock_guard _lock(mSimpleJobMutex);
        mPendingJobCount += uint32_t(mPendingJobs.size()); // Set the number of pending jobs counter.
        uint32_t ret = mPendingJobCount;
        // Start all threads working on the available jobs
        for (uint32_t i = 0; i < mMaxThreads; i++)
        {
            mThreads[i].start(this);
        }
        return ret;
    }

    // Sleeps until all of the pending jobs have completed.
    virtual void waitForJobsToComplete(void) final
    {
        if (mPendingJobCount == 0)
        {
            return;
        }
        while (mPendingJobCount.load() )
        {
            // std::this_thread::sleep_for(std::chrono::nanoseconds(10000)); // s
            for (uint32_t i = 0; i < mMaxThreads; i++)
            {
                if (mThreads[i].mThread.valid())
                    mThreads[i].mThread.wait_for(std::chrono::nanoseconds(10000));
            }
        }
    }

    // Releases the JobSystem instance
    virtual void release(void) final
    {
        delete this;
    }

    virtual SimpleJob* getSimpleJob(void) final
    {
        SimpleJob* ret = nullptr;
        lock_guard _lock(mSimpleJobMutex);
        if (!mPendingJobs.empty())
        {
            ret = mPendingJobs.front();
            mPendingJobs.pop();
        }
        return ret;
    }

    // This job is complete, delete the pointer and decrement
    // the total pending job count. If it goes to zero, then raise the 'mHaveWork'
    virtual void simpleJobComplete(SimpleJob* sj) final
    {
        delete sj;
        mPendingJobCount.fetch_sub(1);
    }

    std::atomic<uint32_t> mPendingJobCount{ 0 };
    uint32_t mMaxThreads{ 8 };
    MutexWrapper mSimpleJobMutex;
    SimpleJobQueue mPendingJobs;
    SimpleJobThread* mThreads{ nullptr };
};

// Create in instance of the JobSystem with the number of threads specified.
// More threads than available cores is not particularly beneficial.
JobSystem* JobSystem::create(uint32_t maxThreads)
{
    auto ret = new JobSystemImpl(maxThreads);
    return static_cast<JobSystem*>(ret);
}


} // namespace simplejobsystem
