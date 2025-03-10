// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
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

#include "NvFlowExt.h"

#include "NvFlowArray.h"

#if !defined(__aarch64__)
#include <xmmintrin.h>
#include <pmmintrin.h>
#else
// TODO: support flush denorm on aarch64
#endif

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace NvFlowThreadPoolDefault
{
    struct Thread
    {
        std::thread thread;
        void* sharedMem;
    };

    struct ThreadPool
    {
        NvFlowArray<Thread> threads;

        int activeCount = 0;
        std::mutex mutex;
        std::condition_variable cond_fork;
        std::condition_variable cond_join;
        void(*task)(NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata) = nullptr;
        void* userdata = nullptr;

        NvFlowUint64 sharedMemorySizeInBytes = 0llu;
        void* sharedMem;

        std::atomic<NvFlowUint> taskIdx;
        NvFlowUint taskCount = 0u;
        NvFlowUint taskGranularity = 0u;
    };

    NV_FLOW_CAST_PAIR(NvFlowThreadPool, ThreadPool)

    static const NvFlowUint defaultMaxThreads = 32u; // limit for high core count CPUs

    NvFlowUint getDefaultThreadCount()
    {
        NvFlowUint defaultThreadCount = std::thread::hardware_concurrency();
        if (defaultThreadCount > defaultMaxThreads)
        {
            defaultThreadCount = defaultMaxThreads;
        }
        return defaultThreadCount;
    }

    static void threadMain(NvFlowUint threadIdx, ThreadPool* pool)
    {
        Thread* ptr = &pool->threads[threadIdx];

#if !defined(__aarch64__)
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#else
        // TODO: support flush denorm on aarch64
#endif

        while (1)
        {
            {
                std::unique_lock<std::mutex> lk(pool->mutex);
                if (pool->activeCount > 0)
                {
                    pool->activeCount--;
                }
                if (pool->activeCount == 0)
                {
                    pool->cond_join.notify_one();
                }
                pool->cond_fork.wait(lk);
                if (pool->activeCount < 0)
                {
                    return;
                }
            }
            if (pool->task)
            {
                while (pool->taskIdx < pool->taskCount)
                {
                    NvFlowUint taskIdx = pool->taskIdx.fetch_add(pool->taskGranularity);
                    NvFlowUint taskIdx_max = taskIdx + pool->taskGranularity;
                    if (taskIdx_max > pool->taskCount)
                    {
                        taskIdx_max = pool->taskCount;
                    }
                    while (taskIdx < taskIdx_max)
                    {
                        pool->task(taskIdx, threadIdx, ptr->sharedMem, pool->userdata);
                        taskIdx++;
                    }
                }
            }
        }
    }

    NvFlowThreadPool* create(NvFlowUint threadCountIn, NvFlowUint64 sharedMemorySizeInBytesIn)
    {
        auto ptr = new ThreadPool();

        ptr->taskIdx = 0u;

        ptr->sharedMemorySizeInBytes = sharedMemorySizeInBytesIn;
        if (ptr->sharedMemorySizeInBytes == 0llu)
        {
            ptr->sharedMemorySizeInBytes = 1024u * 1024u;
        }
        ptr->sharedMem = malloc(ptr->sharedMemorySizeInBytes);

        NvFlowUint threadCount = threadCountIn;
        if (threadCount == 0u)
        {
            threadCount = getDefaultThreadCount();
        }
        ptr->threads.reserve(threadCount);
        ptr->threads.size = threadCount;

        {
            std::unique_lock<std::mutex> lk(ptr->mutex);
            ptr->activeCount = (NvFlowUint)ptr->threads.size;
            for (NvFlowUint i = 0; i < ptr->threads.size; i++)
            {
                ptr->threads[i].sharedMem = malloc(ptr->sharedMemorySizeInBytes);
                ptr->threads[i].thread = std::thread(threadMain, i, ptr);
            }
            // join
            ptr->cond_join.wait(lk, [&] {return ptr->activeCount == 0; });
        }
        return cast(ptr);
    }

    void destroy(NvFlowThreadPool* pool)
    {
        auto ptr = cast(pool);

        {
            std::unique_lock<std::mutex> lk(ptr->mutex);
            ptr->taskIdx = 0u;
            ptr->taskCount = 0u;
            ptr->activeCount = -1;
            ptr->cond_fork.notify_all();
        }
        for (NvFlowUint i = 0; i < ptr->threads.size; i++)
        {
            ptr->threads[i].thread.join();
            free(ptr->threads[i].sharedMem);
        }
        free(ptr->sharedMem);

        delete ptr;
    }

    NvFlowUint getThreadCount(NvFlowThreadPool* pool)
    {
        auto ptr = cast(pool);
        return (NvFlowUint)ptr->threads.size;
    }

    void execute(NvFlowThreadPool* pool, NvFlowUint taskCount, NvFlowUint taskGranularity, NvFlowThreadPoolTask_t task, void* userdata)
    {
        auto ptr = cast(pool);
        if (taskCount < taskGranularity)
        {
            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                task(taskIdx, 0u, ptr->sharedMem, userdata);
            }
        }
        else
        {
            std::unique_lock<std::mutex> lk(ptr->mutex);
            // dispatch
            ptr->taskIdx = 0u;
            ptr->taskCount = taskCount;
            ptr->taskGranularity = taskGranularity == 0u ? 1u : taskGranularity;
            ptr->task = task;
            ptr->userdata = userdata;
            ptr->activeCount = (NvFlowUint)ptr->threads.size;
            ptr->cond_fork.notify_all();
            // join
            ptr->cond_join.wait(lk, [&] {return ptr->activeCount == 0; });
            task = nullptr;
            userdata = nullptr;
        }
    }
}

NvFlowThreadPoolInterface* NvFlowGetThreadPoolInterface()
{
    using namespace NvFlowThreadPoolDefault;
    static NvFlowThreadPoolInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowThreadPoolInterface) };
    iface.getDefaultThreadCount = getDefaultThreadCount;
    iface.create = create;
    iface.destroy = destroy;
    iface.getThreadCount = getThreadCount;
    iface.execute = execute;
    return &iface;
}
