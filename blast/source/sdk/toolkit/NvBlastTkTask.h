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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTTKTASK_H
#define NVBLASTTKTASK_H

#include "NvBlastTkGroupTaskManager.h"
#include "NvTask.h"
#include "NvBlastTkGroup.h"

#include <atomic>
#include <mutex>
#include <condition_variable>

namespace Nv
{
namespace Blast
{

/**
Counting synchronization object for waiting on TkWorkers to finish.
*/
class TkTaskSync
{
public:
    /**
    Initializes with an expected number of notifications.
    */
    TkTaskSync(uint32_t count) : m_count(count) {}

    /**
    Blocks until the expected number of notifications happened.
    */
    void wait()
    {
        std::unique_lock<std::mutex> lk(m_mutex);
        m_cv.wait(lk, [&] { return m_count == 0; });
    }

    /**
    Decrement the wait() count by one.
    */
    void notify()
    {
        //PERF_SCOPE_H("TaskSync::notify");
        std::unique_lock<std::mutex> lk(m_mutex);
        if (m_count > 0)
        {
            m_count--;
        }
        if (m_count == 0)
        {
            lk.unlock();
            m_cv.notify_one();
        }
    }

    /**
    Peek if notifications are pending.
    */
    bool isDone()
    {
        std::unique_lock<std::mutex> lk(m_mutex);
        return m_count == 0;
    }

    /**
    Sets the expected number of notifications for wait() to unblock.
    */
    void setCount(uint32_t count)
    {
        m_count = count;
    }

private:
    std::mutex              m_mutex;
    std::condition_variable m_cv;
    uint32_t                m_count;
};


/**
Common job counter for all tasks.
*/
class TkAtomicCounter
{
public:
    TkAtomicCounter() : m_current(0), m_maxCount(0) {}

    bool isValid(uint32_t val)
    {
        return val < m_maxCount;
    }

    uint32_t next()
    {
        return m_current.fetch_add(1);
    }

    void reset(uint32_t maxCount)
    {
        m_maxCount = maxCount;
        m_current = 0;
    }
private:
    std::atomic<uint32_t> m_current;
    uint32_t m_maxCount;
};


/**
A task running one group job after the other until done. Synchronizes atomically with its siblings.
*/
class TkGroupWorkerTask : public nvidia::task::NvLightCpuTask
{
public:
    TkGroupWorkerTask() : NvLightCpuTask(), m_group(nullptr), m_counter(nullptr), m_sync(nullptr)
    {
    }

    void setup(TkGroup* group, TkAtomicCounter* counter, TkTaskSync* sync)
    {
        m_group = group;
        m_counter = counter;
        m_sync = sync;
    }

    virtual void run() override
    {
        Nv::Blast::TkGroupWorker* worker = m_group->acquireWorker();
        uint32_t jobID = m_counter->next();
        while (m_counter->isValid(jobID))
        {
            worker->process(jobID);
            jobID = m_counter->next();
        }
        m_group->returnWorker(worker);
    }

    virtual void release() override
    {
        NvLightCpuTask::release();

        // release the sync last
        m_sync->notify();
    }

    virtual const char* getName() const override { return "BlastGroupWorkerTask"; }

private:
    TkGroup* m_group;
    TkAtomicCounter* m_counter;
    TkTaskSync* m_sync;
};


/**
Implements TkGroupTaskManager
*/
class TkGroupTaskManagerImpl : public TkGroupTaskManager
{
public:
    TkGroupTaskManagerImpl(nvidia::task::NvTaskManager& taskManager, TkGroup* group)
        : m_taskManager(taskManager), m_sync(0), m_group(group) {}

    // TkGroupTaskManager API
    virtual void setGroup(TkGroup*) override;
    virtual uint32_t process(uint32_t) override;
    virtual void release() override;
    virtual bool wait(bool block) override;

private:
    static const uint32_t           TASKS_MAX_COUNT = 16;
    nvidia::task::NvTaskManager&    m_taskManager;
    TkAtomicCounter                 m_counter;
    TkGroupWorkerTask               m_tasks[TASKS_MAX_COUNT];
    TkTaskSync                      m_sync;
    TkGroup*                        m_group;
};

} // namespace Blast
} // namespace Nv

#endif // NVBLASTTKTASK_H
