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


#include "NvBlastGlobals.h"
#include "NvBlastTkTask.h"
#include "NvCpuDispatcher.h"
#include "NvBlastTkGroup.h"

using namespace Nv::Blast;


uint32_t TkGroupTaskManagerImpl::process(uint32_t workerCount)
{
    NVBLAST_CHECK_WARNING(m_group != nullptr, "TkGroupTaskManager::process cannot process, no group set.", return 0);
    NVBLAST_CHECK_WARNING(m_sync.isDone(), "TkGroupTaskManager::process group is already being processed.", return 0);

    // at least one task must start, even when dispatcher has none specified
    uint32_t dispatcherThreads = m_taskManager.getCpuDispatcher()->getWorkerCount();
    dispatcherThreads = dispatcherThreads > 0 ? dispatcherThreads : 1;

    // not expecting an arbitrary amount of tasks
    uint32_t availableTasks = TASKS_MAX_COUNT;

    // use workerCount tasks, unless dispatcher has less threads or less tasks are available
    uint32_t requestedTasks = workerCount > 0 ? workerCount : dispatcherThreads;
    requestedTasks = requestedTasks > dispatcherThreads ? dispatcherThreads : requestedTasks;
    requestedTasks = requestedTasks > availableTasks ? availableTasks : requestedTasks;

    // ensure the group has enough memory allocated for concurrent processing
    m_group->setWorkerCount(requestedTasks);

    // check if there is work to do
    uint32_t jobCount = m_group->startProcess();

    if (jobCount)
    {
        // don't start more tasks than jobs are available
        requestedTasks = requestedTasks > jobCount ? jobCount : requestedTasks;

        // common counter for all tasks
        m_counter.reset(jobCount);

        // set to busy state
        m_sync.setCount(requestedTasks);

        // set up tasks
        for (uint32_t i = 0; i < requestedTasks; i++)
        {
            m_tasks[i].setup(m_group, &m_counter, &m_sync);
            m_tasks[i].setContinuation(m_taskManager, nullptr);
            m_tasks[i].removeReference();
        }

        return requestedTasks;
    }

    // there was no work to be done
    return 0;
}


bool TkGroupTaskManagerImpl::wait(bool block)
{
    if (block && !m_sync.isDone())
    {
        m_sync.wait();
    }
    if (m_sync.isDone())
    {
        return m_group->endProcess();
    }
    return false;
}


void TkGroupTaskManagerImpl::setGroup(TkGroup* group)
{
    NVBLAST_CHECK_WARNING(m_sync.isDone(), "TkGroupTaskManager::setGroup trying to change group while processing.", return);

    m_group = group;
}


TkGroupTaskManager* TkGroupTaskManager::create(nvidia::task::NvTaskManager& taskManager, TkGroup* group)
{
    return NVBLAST_NEW(TkGroupTaskManagerImpl) (taskManager, group);
}


void TkGroupTaskManagerImpl::release()
{
    NVBLAST_CHECK_WARNING(m_sync.isDone(), "TkGroupTaskManager::release group is still being processed.", return);

    NVBLAST_DELETE(this, TkGroupTaskManagerImpl);
}
