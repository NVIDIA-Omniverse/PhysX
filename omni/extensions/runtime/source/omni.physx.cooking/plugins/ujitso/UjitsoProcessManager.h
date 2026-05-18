// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UjitsoProcessContext.h"
#include "UjitsoServiceUtils.h"
#include <carb/tasking/TaskingUtils.h>

namespace omni
{
namespace physx
{

struct UjitsoResourceManager;

/**
 * Class to manage ujitso build processes (UjitsoProcessContext).  It also manages ujitso Agents to be used when the
 * process requests a build.
 */
struct UjitsoProcessManager
{
    typedef std::lock_guard<carb::tasking::MutexWrapper> lock_guard;

    /**
     * ctor
     *
     * \param[in]   name                    Unique name to give this process manager.  This may be used by
     *                                      the cooking service to identify multiple process managers.
     * \param[in]   maxRunningProcessCount  The maximum number of queued processes which may be running.  If zero
     *                                      is passed in, the default value of s_defaultMaxRunningProcessCount is used.
     */
    UjitsoProcessManager(const std::string& name, size_t maxRunningProcessCount = 0);

    virtual ~UjitsoProcessManager();

    /**
     * The name of this manager.
     *
     * \return the name for this manager passed into the constructor.
     */
    const std::string& getName() const
    {
        return m_name;
    }

    /**
     * Set the maximum number of running processes in this context.
     *
     * \param[in]   maxRunningProcessCount  The maximum number of running processes.  If 0, the default value given by
     *                                      s_defaultMaxRunningProcessCount is used.
     */
    void setMaxRunningProcessCount(size_t maxRunningProcessCount);

    /**
     * Returns the number of processes in the queue, regardless of their state (Pending, Running, or Finishing).
     *
     * \return the number of queued processes.
     */
    size_t getQueuedProcessCount();

    /**
     * Determines if the given process is queued in this process manager.
     *
     * \return true if this process is queued in this process manager, false otherwise.
     */
    bool queued(UjitsoProcessContext* process);

    /**
     * Find a process by requestId.  The requestId may be set by the user in the process implementation, to
     * match, for example, a data + build parameter hash.
     *
     * \param[in]   requestId   The user-set ID.
     *
     * \return the process that has that ID if found, nullptr otherwise.
     */
    UjitsoProcessContext* find(UjitsoProcessContext::RequestId requestId);

    /**
     * Places a process in the queue.  Subsequent calls to pump() will advance the process and run it in order of
     * insertion, when the maximum running process count allows.  The process must be in the Pending state.
     *
     * \param[in]   process The process to schedule.
     *
     * \return true iff successful.  (Process cannot have been scheduled already.)
     */
    bool schedule(UjitsoProcessContext* process);

    /**
     * Update the process queue:
     * Pending -> Running (if no pause was requested, see requestPause()) and the maximum running count isn't reached)
     * Running -> Finishing (after ujitso returns a build)
     * Finishing -> call process->onComplete() and remove from queue (if no pause was requested)
     */
    size_t pump();

    /**
     * Push a single process through the queue.  Returns true if the process is in the queue and was successfully
     * started (if it wasn't already started) and was be completed in the time given.
     *
     * \param[in]   process     The process to complete.
     * \param[in]   timeOutMs   The time to wait until the process completes.
     *
     * \return true iff the process completes in the time given.
     */
    bool waitForProcess(UjitsoProcessContext* process, uint32_t timeOutMs = carb::ujitso::kTimeOutInfinite);

    /**
     * Cancel a queued process.  (Removes it from the queue.)  Calls process->cancel(invokeCallbacks), leaving it
     * up to the process implementation to determine what to do with the invokeCallbacks parameter.
     *
     * \param[in]   process         The process to cancel.
     * \param[in]   invokeCallbacks Parameter to pass into process->cancel(...).
     *
     * \return true iff the process was in the queue and was successfully canceled.
     */
    bool cancel(UjitsoProcessContext* process, bool invokeCallbacks);

    /**
     * Cancels all processes in the queue, calling process->cancel(false).
     *
     * \return the number of processes successfully canceled.
     */
    size_t cancelAll();

    /**
     * TEMPORARY
     *
     * Public member variable which holds a fallback implementation.
     * This may be removed if the ujitso service completely replaces the existing service.
     */
    void* m_fallbackContext;

    /**
     * Zeros out finished process count.
     */
    static void zeroFinishedProcessCount()
    {
        s_finishedProcessCount = 0;
    }

    /**
     * Reads finished process count.
     */
    static size_t getFinishedProcessCount()
    {
        return (size_t)s_finishedProcessCount;
    }

private:
    typedef std::list<UjitsoProcessContext*> Queue;
    typedef std::unordered_map<UjitsoProcessContext*, Queue::iterator> ProcessMap;
    typedef std::unordered_map<UjitsoProcessContext::RequestId, Queue::iterator> RequestMap;
    typedef std::unordered_map<UjitsoProcessContext::OwnerId, UjitsoProcessContext*> OwnerMap;

    void _insert(lock_guard&, UjitsoProcessContext* process);
    void _remove(lock_guard&, UjitsoProcessContext* process);
    bool _cancel(lock_guard&, Queue::iterator q, bool invokeCallbacks);
    UjitsoProcessContext* _find(lock_guard&, UjitsoProcessContext::RequestId requestId) const;
    size_t _getQueuedProcessCount(lock_guard&) const
    {
        return m_queue.size();
    }

    static constexpr size_t s_defaultMaxRunningProcessCount = 16;

    carb::tasking::MutexWrapper m_queueMutex;
    const std::string m_name;
    Queue m_queue; // Holds all processes
    ProcessMap m_processMap; // Maps from process to iterator in m_queue
    RequestMap m_requestMap; // Maps from RequestId to iterator in m_queue
    OwnerMap m_ownerMap; // Maps from OwnerId to process
    size_t m_maxRunningProcessCount;

    // Ever-increasing global quantity to aid with cooking compute service implementation
    static std::atomic<size_t> s_finishedProcessCount;
};

} // namespace physx
} // namespace omni
