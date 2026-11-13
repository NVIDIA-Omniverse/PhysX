// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-ATTACH-SERVICES-001
 * @covers AC-3
 */

#include <carb/profiler/Profile.h>

#include "UjitsoProcessManager.h"
#include "UjitsoResourceManager.h"

using namespace carb::ujitso;

namespace omni
{
namespace physx
{

// UjitsoProcessManager static members

std::atomic<size_t> UjitsoProcessManager::s_finishedProcessCount = 0;


// UjitsoProcessManager methods

UjitsoProcessManager::UjitsoProcessManager(const std::string& name, size_t maxRunningProcessCount /*= 0*/) :
    m_name(name), m_maxRunningProcessCount(maxRunningProcessCount)
{
    if (!m_maxRunningProcessCount) m_maxRunningProcessCount = s_defaultMaxRunningProcessCount;
}

UjitsoProcessManager::~UjitsoProcessManager()
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::~UjitsoProcessManager");

    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);
    // Drain without scheduling pending work. cancel(false) suppresses application
    // callbacks; onComplete releases the request and its captured inputs, first waiting
    // for the callback of a build that is still running.
    for (auto& entry : m_ownerMap)
    {
        if (entry.second)
        {
            entry.second->cancel(false);
            entry.second->onComplete();
        }
    }
    m_ownerMap.clear();
    for (auto* process : m_queue)
    {
        process->cancel(false);
        process->onComplete();
    }
    m_queue.clear();
    m_processMap.clear();
    m_requestMap.clear();
}

void UjitsoProcessManager::setMaxRunningProcessCount(size_t maxRunningProcessCount)
{
    m_maxRunningProcessCount = maxRunningProcessCount ? maxRunningProcessCount : s_defaultMaxRunningProcessCount;
}

size_t UjitsoProcessManager::getQueuedProcessCount()
{
    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    return _getQueuedProcessCount(queueLock);
}

bool UjitsoProcessManager::queued(UjitsoProcessContext* process)
{
    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    CHECK_RETURN_FALSE_ON_FAIL(process);
    return m_processMap.count(process) == 1;
}

UjitsoProcessContext* UjitsoProcessManager::find(UjitsoProcessContext::RequestId requestId)
{
    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    return _find(queueLock, requestId);
}

bool UjitsoProcessManager::schedule(UjitsoProcessContext* process)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::schedule");

    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    CHECK_RETURN_FALSE_ON_FAIL(process);

    if (process->getState() != UjitsoProcessContext::Unqueued)
    {
        CARB_LOG_ERROR("UjitsoProcessManager::schedule: process already scheduled.");
        return false;
    }

    const UjitsoProcessContext::RequestId requestId = process->requestId();
    const UjitsoProcessContext::OwnerId ownerId = process->ownerId();

    if (_find(queueLock, requestId))
    {
        CARB_LOG_ERROR("UjitsoProcessManager::schedule: process with same build request already scheduled.");
        return false;
    }

    if (ownerId)
    {
        // See if there's already a process with this owner in the queue
        OwnerMap::iterator o = m_ownerMap.find(ownerId);
        if (o != m_ownerMap.end())
        {
            UjitsoProcessContext* pending = o->second;
            // Set the pending process to the one being scheduled
            o->second = process;
            // If there was already a pending process for this owner, we call onComplete() for it
            if (pending) pending->onComplete();
            return true;    // Done; when the owned process completes it will start the pending process
        }

        // Otherwise we'll schedule this process.  It has an owner, so map it
        m_ownerMap[ownerId] = nullptr;  // Signifies that the owner has a process in the queue only, nothing pending
    }

    _insert(queueLock, process);

    return true;
}

size_t UjitsoProcessManager::pump()
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::pump");

    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    // Run through process queue, adding converting waiting processes to running until the maximum is reached.
    // Note, running processes are always before waiting processes in the queue.  This way, processes don't need
    // to be reordered.
    size_t runningProcessCount = 0;
    bool traversing = true;
    for (Queue::iterator q = m_queue.begin(); q != m_queue.end() && traversing;)
    {
        UjitsoProcessContext* process = *q++;
        switch (process->getState())
        {
        default:
        case UjitsoProcessContext::Unqueued:
            CARB_LOG_ERROR("UjitsoProcessManager::pump: process with invalid state (%d) in queue.",
                           process->getState());
            break;
        case UjitsoProcessContext::Pending:
            traversing = runningProcessCount < m_maxRunningProcessCount;
            if (traversing)
            {
                switch (process->requestBuild(0))
                {
                case UjitsoProcessContext::BuildRequestResult::eSuccess:
                    ++runningProcessCount;
                    break;
                case UjitsoProcessContext::BuildRequestResult::eResourcesUnavailable:
                case UjitsoProcessContext::BuildRequestResult::eTimedOut:
                    traversing = false; // Resources unavailable; stop processing queue
                    break;
                default:
                    // Request failed for some other reason; remove process from queue
                    CARB_LOG_ERROR("UjitsoProcessManager::pump: process request failed.");
                    m_queue.erase(std::prev(q));
                    _remove(queueLock, process);
                }
            }
            break;
        case UjitsoProcessContext::Running:
            ++runningProcessCount;
            break;
        case UjitsoProcessContext::Finishing:
            // A process canceled while running is Finishing before its build is: leave it queued
            // until the build's callback has returned instead of blocking this pump on it
            if (process->callbackPending())
            {
                ++runningProcessCount;
                break;
            }
            process->onBuildFinished();
            m_queue.erase(std::prev(q));
            _remove(queueLock, process);
            ++s_finishedProcessCount;
            break;
        }
    }

    return _getQueuedProcessCount(queueLock);
}

bool UjitsoProcessManager::waitForProcess(UjitsoProcessContext* process, uint32_t timeOutMs /*= kTimeOutInfinite*/)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::waitForProcess");

    // Every wait below takes what is left of the caller's timeOutMs
    const auto start = std::chrono::steady_clock::now();
    auto remainingMs = [&]() -> uint32_t
    {
        if (timeOutMs == kTimeOutInfinite)
            return kTimeOutInfinite;
        const int64_t elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        return (uint32_t)std::max((int64_t)0, (int64_t)timeOutMs - elapsedMs);
    };

    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    ProcessMap::iterator p = m_processMap.find(process);
    if (p == m_processMap.end())
    {
        CARB_LOG_ERROR("UjitsoProcessManager::waitForProcess: process %p not found in process map for "
                        "context \"%s\"",  process, m_name.c_str());
        return false;
    }

    m_queue.erase(p->second);

    // Not finished within timeOutMs: put the process at the head of the queue, so that it will be
    // removed in pump() when done
    auto requeue = [&]()
    {
        CARB_LOG_WARN("UjitsoProcessManager::waitForProcess: Timeout occurred (%dms) while waiting "
                      "for process %p from \"%s\" context", timeOutMs, process, m_name.c_str());
        m_processMap[process] = m_queue.insert(m_queue.begin(), process);
        return false;
    };

    bool result = true;

    switch (process->getState())
    {
    default:
    case UjitsoProcessContext::Unqueued:
        CARB_LOG_WARN("UjitsoProcessManager::waitForProcess: Process %p was not queued in context \"%s\"",
                      process, m_name.c_str());
        result = false;
        break;
    case UjitsoProcessContext::Pending:
        switch (process->requestBuild(remainingMs()))
        {
        case UjitsoProcessContext::BuildRequestResult::eSuccess:
            break;
        case UjitsoProcessContext::BuildRequestResult::eTimedOut:
            CARB_LOG_ERROR("UjitsoProcessManager::waitForProcess: process request timed out.");
        case UjitsoProcessContext::BuildRequestResult::eResourcesUnavailable:
        default:
            // Request failed for some other reason; remove process from queue
            CARB_LOG_ERROR("UjitsoProcessManager::waitForProcess: process request failed.  Terminating.");
            result = false;
            break;
        }
        if (!result)
            break;
        // Fall through
    case UjitsoProcessContext::Running:
        process->waitRequest(remainingMs());
        if (process->getState() != UjitsoProcessContext::Finishing)
            return requeue();
        // Fall through
    case UjitsoProcessContext::Finishing:
        // A process canceled while running is Finishing before its build is: its callback may still
        // be running, so the process cannot be released before that returns
        if (!process->waitForCallback(remainingMs()))
            return requeue();
        process->onBuildFinished();
        ++s_finishedProcessCount;
        break;
    }

    _remove(queueLock, process);

    return result;
}

bool UjitsoProcessManager::cancel(UjitsoProcessContext* process, bool invokeCallbacks)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::cancel");

    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    ProcessMap::iterator p = m_processMap.find(process);
    if (p != m_processMap.end())
        return _cancel(queueLock, p->second, invokeCallbacks);

    CARB_LOG_WARN("UjitsoProcessManager::cancel: process %p not found in process map for context \"%s\"",
                  process, m_name.c_str());

    return false;
}

size_t UjitsoProcessManager::cancelAll()
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::cancelAll");

    std::lock_guard<carb::tasking::MutexWrapper> queueLock(m_queueMutex);

    size_t numberCanceled = 0;

    for (Queue::iterator q = m_queue.begin(); q != m_queue.end();)
        if (_cancel(queueLock, q++, false)) ++numberCanceled;    // Increment q _before_ calling cancel, which invalidates q

    return numberCanceled;
}

// Private methods

void UjitsoProcessManager::_insert(lock_guard&, UjitsoProcessContext* process)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::insert");

    Queue::iterator q = m_queue.insert(m_queue.end(), process);

    m_requestMap[process->requestId()] = process;
    m_processMap[process] = q;

    process->onQueue();
}

void UjitsoProcessManager::_remove(lock_guard& queueLock, UjitsoProcessContext* process)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::remove");

    // A canceled process gave its request ID up in _cancel(), maybe to a newer process by now
    if (!_releaseRequestId(queueLock, process) && !process->canceled())
    {
        CARB_LOG_ERROR("UjitsoProcessManager::remove: request ID for process was not found.");
    }

    if (m_processMap.erase(process) != 1)
    {
        CARB_LOG_ERROR("UjitsoProcessManager::remove: process was not found in process map.");
    }

    // If there's a pending processes, schedule it
    OwnerMap::iterator o = m_ownerMap.find(process->ownerId());
    if (o != m_ownerMap.end())
    {
        UjitsoProcessContext* pending = o->second;
        o->second = nullptr;
        if (pending) _insert(queueLock, pending);
        else m_ownerMap.erase(o);
    }
    else
    if (process->ownerId() != 0)
    {
        CARB_LOG_ERROR("UjitsoProcessManager::remove: process owner was not found in owner map.");
    }

    process->onComplete();
}

bool UjitsoProcessManager::_cancel(lock_guard& queueLock, Queue::iterator q, bool invokeCallbacks)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessManager::cancel");

    UjitsoProcessContext* process = *q;

    if (!process)
    {
        CARB_LOG_ERROR("UjitsoProcessManager::cancel: null process pointer in queue.");
        return false;
    }

    process->cancel(invokeCallbacks);

    // A canceled process delivers no cooked data. It can stay queued until its build callback returns:
    // a new request for the same build must start its own process instead of being added to this one.
    _releaseRequestId(queueLock, process);

    bool result = true;

    switch (process->getState())
    {
    default:
    case UjitsoProcessContext::Unqueued:
        CARB_LOG_WARN("UjitsoProcessManager::cancel: Process %p was not queued in context \"%s\"",
                      process, m_name.c_str());
        result = false;
        break;
    case UjitsoProcessContext::Pending:
        m_queue.erase(q);
        _remove(queueLock, process);
        break;
    case UjitsoProcessContext::Running:
        CARB_LOG_INFO("UjitsoProcessManager::cancel: canceling running process.  Process must wait to finish.");
        break;
    case UjitsoProcessContext::Finishing:
        CARB_LOG_INFO("UjitsoProcessManager::cancel: canceling finishing process.");
        break;
    }

    return result;
}

// Removes the process's entry from the request map, unless another process has taken that entry
bool UjitsoProcessManager::_releaseRequestId(lock_guard&, UjitsoProcessContext* process)
{
    RequestMap::iterator r = m_requestMap.find(process->requestId());

    if (r == m_requestMap.end() || r->second != process) return false;

    m_requestMap.erase(r);
    return true;
}

UjitsoProcessContext* UjitsoProcessManager::_find(lock_guard&, UjitsoProcessContext::RequestId requestId) const
{
    RequestMap::const_iterator r = m_requestMap.find(requestId);

    return r != m_requestMap.end() ? r->second : nullptr;
}

} // namespace physx
} // namespace omni
