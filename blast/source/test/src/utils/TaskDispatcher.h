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


#pragma once

#include <thread>
#include <mutex>
#include <queue>
#include <list>
#include <future>
#include <condition_variable>
#include <memory>
#include <atomic>

class TaskDispatcher
{
public:
    class Task
    {
    public:
        virtual void process() = 0;
        virtual ~Task() {};
    };

    typedef std::function<void(TaskDispatcher& dispatcher, std::unique_ptr<Task>)> OnTaskFinishedFunction;

    TaskDispatcher(uint32_t threadCount, OnTaskFinishedFunction onTaskFinished) :
        m_workingThreadsCount(0), m_onTaskFinished(onTaskFinished)
    {
        m_threads.resize(threadCount);
        for (uint32_t i = 0; i < threadCount; i++)
        {
            m_threads[i] = std::unique_ptr<Thread>(new Thread(i, m_completionSemaphore));
            m_threads[i]->start();
            m_freeThreads.push(m_threads[i].get());
        }
    }

    void addTask(std::unique_ptr<Task> task)
    {
        m_tasks.push(std::move(task));
    }

    void process()
    {
        // main loop
        while (m_tasks.size() > 0 || m_workingThreadsCount > 0)
        {
            // assign tasks
            while (!(m_tasks.empty() || m_freeThreads.empty()))
            {
                auto task = std::move(m_tasks.front());
                m_tasks.pop();

                Thread* freeThread = m_freeThreads.front();
                m_freeThreads.pop();

                freeThread->processTask(std::move(task));
                m_workingThreadsCount++;
            }

            m_completionSemaphore.wait();

            // check for completion
            for (std::unique_ptr<Thread>& thread : m_threads)
            {
                if (thread->isTaskFinished())
                {
                    std::unique_ptr<Task> task;
                    thread->collectTask(task);
                    m_onTaskFinished(*this, std::move(task));

                    m_freeThreads.push(thread.get());
                    m_workingThreadsCount--;
                    break;
                }
            }
        }
    }

private:
    class Semaphore
    {
    public:
        Semaphore(int count_ = 0)
            : m_count(count_) {}

        inline void notify()
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_count++;
            m_cv.notify_one();
        }

        inline void wait()
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            while (m_count == 0){
                m_cv.wait(lock);
            }
            m_count--;
        }


    private:
        std::mutex              m_mutex;
        std::condition_variable m_cv;
        int                     m_count;
    };

    class Thread
    {
    public:
        Thread(uint32_t id_, Semaphore& completionSemaphore) : m_id(id_), m_completionSemaphore(completionSemaphore), m_running(false), m_taskFinished(false) {}
        virtual ~Thread() { stop(); }

        void start()
        {
            if (!m_running)
            {
                m_running = true;
                m_thread = std::thread(&Thread::body, this);
            }
        }

        void stop()
        {
            if (m_running)
            {
                m_running = false;
                m_newTaskSemaphore.notify();
                m_thread.join();
            }
        }

        void processTask(std::unique_ptr<Task> task)
        {
            m_task = std::move(task);
            m_taskFinished = false;
            m_newTaskSemaphore.notify();
        }

        void collectTask(std::unique_ptr<Task>& task)
        {
            task = std::move(m_task);
            m_task = nullptr;
            m_taskFinished = false;
        }

        bool hasTask() const { return m_task != nullptr; }

        bool isTaskFinished() const { return m_taskFinished; }

    private:
        void body()
        {
            while (1)
            {
                m_newTaskSemaphore.wait();

                if (!m_running)
                    return;

                m_task->process();
                m_taskFinished = true;

                m_completionSemaphore.notify();
            }
        }

        uint32_t              m_id;
        Semaphore&            m_completionSemaphore;
        std::thread           m_thread;
        bool                  m_running;

        std::unique_ptr<Task> m_task;
        std::atomic<bool>     m_taskFinished;

        Semaphore             m_newTaskSemaphore;
    };

private:
    uint32_t                             m_workingThreadsCount;

    std::queue<std::unique_ptr<Task>>    m_tasks;
    OnTaskFinishedFunction               m_onTaskFinished;

    std::vector<std::unique_ptr<Thread>> m_threads;
    std::queue<Thread*>                  m_freeThreads;

    Semaphore                            m_completionSemaphore;
};

