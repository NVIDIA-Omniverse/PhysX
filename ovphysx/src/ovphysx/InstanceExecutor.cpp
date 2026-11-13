// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1
 */

#include "internal/sdk/InstanceExecutor.hpp"

#include <carb/tasking/IThreadPool.h>
#include <carb/thread/SetName.h>

#include <algorithm>
#include <exception>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace ovphysx
{

namespace
{

std::atomic<const ExecutorWaitHooks*> g_waitHooks{nullptr};

// Its address identifies the calling OS thread. uint64_t keeps the address even,
// so it never equals a tagged (odd) Carbonite task identity.
thread_local uint64_t t_threadIdentityMarker = 0;

const ExecutorWaitHooks* waitHooks() noexcept
{
    return g_waitHooks.load(std::memory_order_acquire);
}

// Runs job; an exception becomes its failure unless it already recorded one.
void runCapturingFailure(InstanceExecutor::Job& job) noexcept
{
    try
    {
        job.run();
    }
    catch (const std::exception& error)
    {
        if (!job.failed)
        {
            job.failed = true;
            try
            {
                job.failure = error.what();
            }
            catch (...)
            {
                job.failure.clear();
            }
        }
    }
    catch (...)
    {
        if (!job.failed)
        {
            job.failed = true;
            try
            {
                job.failure = "Unknown exception during task execution";
            }
            catch (...)
            {
                job.failure.clear();
            }
        }
    }
}

} // namespace

void setExecutorWaitHooks(const ExecutorWaitHooks* hooks) noexcept
{
    g_waitHooks.store(hooks, std::memory_order_release);
}

WaitDeadline WaitDeadline::fromTimeoutNs(uint64_t timeoutNs, std::chrono::steady_clock::time_point start) noexcept
{
    if (timeoutNs == 0)
        return poll();
    if (timeoutNs == UINT64_MAX)
        return infinite();
    // Beyond a century the deadline cannot be observed; treat it as unbounded rather
    // than risk clock arithmetic overflow.
    constexpr uint64_t kCenturyNs = 100ull * 365ull * 24ull * 3600ull * 1000000000ull;
    if (timeoutNs >= kCenturyNs)
        return infinite();
    const std::chrono::nanoseconds requested(static_cast<std::chrono::nanoseconds::rep>(timeoutNs));
    return {Kind::kFinite, start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(requested)};
}

// ---------------------------------------------------------------------------
// TaskTable
// ---------------------------------------------------------------------------

bool InstanceExecutor::TaskTable::isCollectedInRanges(uint64_t index) const
{
    std::map<uint64_t, uint64_t>::const_iterator range = collectedRanges.upper_bound(index);
    return range != collectedRanges.begin() && index <= std::prev(range)->second;
}

void InstanceExecutor::TaskTable::collectRangeMerging(uint64_t first, uint64_t last)
{
    if (first > last)
        return;
    std::map<uint64_t, uint64_t>& ranges = collectedRanges;
    std::map<uint64_t, uint64_t>::iterator begin = ranges.lower_bound(first);
    if (begin != ranges.begin() && std::prev(begin)->second >= first - 1)
        --begin;
    std::map<uint64_t, uint64_t>::iterator end = begin;
    while (end != ranges.end() && end->first <= last + 1)
    {
        first = std::min(first, end->first);
        last = std::max(last, end->second);
        ++end;
    }
    if (first <= collectedPrefix + 1)
    {
        collectedPrefix = std::max(collectedPrefix, last);
        ranges.erase(begin, end);
    }
    else if (begin != end && begin->first == first)
    {
        begin->second = last;
        ranges.erase(std::next(begin), end);
    }
    else
    {
        ranges.emplace_hint(begin, first, last);
        ranges.erase(begin, end);
    }
}

// ---------------------------------------------------------------------------
// Executor
// ---------------------------------------------------------------------------

std::shared_ptr<InstanceExecutor> InstanceExecutor::create(carb::tasking::IThreadPool& threads, size_t maxQueuedJobs)
{
    std::shared_ptr<InstanceExecutor> executor(new InstanceExecutor(maxQueuedJobs));
    // The worker's job owns a reference, so a parked worker never outlives its state: its
    // mutex, condition variables and queue stay valid until the job itself returns. Only
    // an explicit teardown (instance destruction or ovphysx_shutdown) stops the worker and
    // ends its pool thread. Process exit without that teardown destroys no running
    // worker's state and ends no thread; the parked thread ends with the process. The
    // worker runs code of this library, so unloading the library requires a completed
    // ovphysx_shutdown().
    std::unique_ptr<std::shared_ptr<InstanceExecutor>> self(new std::shared_ptr<InstanceExecutor>(executor));
    executor->m_threads = &threads;
    executor->m_pool = threads.createEx(1);
    if (!executor->m_pool)
        throw std::runtime_error("Carbonite could not create the worker thread");
    if (!threads.enqueueJob(executor->m_pool, &InstanceExecutor::workerJob, self.get()))
    {
        threads.destroy(executor->m_pool);
        throw std::runtime_error("Carbonite could not start the worker");
    }
    self.release();
    return executor;
}

void InstanceExecutor::workerJob(void* self)
{
    const std::unique_ptr<std::shared_ptr<InstanceExecutor>> executor(
        static_cast<std::shared_ptr<InstanceExecutor>*>(self));
    carb::this_thread::setName("ovphysx worker");
    (*executor)->workerMain();
}

std::shared_ptr<InstanceExecutor> InstanceExecutor::createWithoutWorker()
{
    std::shared_ptr<InstanceExecutor> executor(new InstanceExecutor(kDefaultMaxQueuedJobs));
    // No worker ever runs, so stopAndJoin() has nothing to wait for.
    executor->m_workerExited = true;
    return executor;
}

InstanceExecutor::InstanceExecutor(size_t maxQueuedJobs) : m_maxQueuedJobs(maxQueuedJobs == 0 ? 1 : maxQueuedJobs)
{
}

InstanceExecutor::~InstanceExecutor()
{
    // Never ends the pool here: this destructor also runs during static destruction at
    // process exit, on Windows possibly under the loader lock, and on the worker thread
    // itself when its job drops the last reference. Explicit teardown calls stopAndJoin()
    // first. The worker holds a reference, so a pool left here is that of a worker that
    // stopped itself, and its parked thread ends with the process.
}

uint64_t InstanceExecutor::identityOf(uint64_t task) noexcept
{
    if (task != 0)
        return (task << 1) | 1u;
    return static_cast<uint64_t>(reinterpret_cast<uintptr_t>(&t_threadIdentityMarker));
}

uint64_t InstanceExecutor::currentIdentity() noexcept
{
    const ExecutorWaitHooks* hooks = waitHooks();
    return identityOf(hooks && hooks->currentTask ? hooks->currentTask() : 0);
}

bool InstanceExecutor::taskAwareWaitAvailable() const noexcept
{
    const ExecutorWaitHooks* hooks = waitHooks();
    return hooks && hooks->currentTask && hooks->wait && hooks->wakeAll && hooks->currentTask() != 0;
}

bool InstanceExecutor::pinCurrentTask() noexcept
{
    const ExecutorWaitHooks* hooks = waitHooks();
    return hooks && hooks->pin && hooks->unpin && hooks->pin();
}

void InstanceExecutor::unpinCurrentTask() noexcept
{
    const ExecutorWaitHooks* hooks = waitHooks();
    if (hooks && hooks->unpin)
        hooks->unpin();
}

void InstanceExecutor::suspendCurrentTask(uint32_t seenEpoch, uint64_t timeoutNs) noexcept
{
    const ExecutorWaitHooks* hooks = waitHooks();
    if (hooks && hooks->wait)
        hooks->wait(&m_epoch, seenEpoch, timeoutNs);
}

InstanceExecutor::Owner InstanceExecutor::owner() const noexcept
{
    return static_cast<Owner>(m_state.load(std::memory_order_acquire) & kOwnerMask);
}

bool InstanceExecutor::heldByCurrent() const noexcept
{
    return owner() != Owner::kFree && m_holder.load(std::memory_order_relaxed) == currentIdentity();
}

uint32_t InstanceExecutor::attentionLocked() const noexcept
{
    const bool attention = !m_queue.empty() || m_laneWaiters != 0 || m_preemptRequested || m_hold;
    return attention ? kAttention : 0u;
}

// With the mutex held: set the attention bit so no lock-free fast path can change the
// owner, and return the owner. Every owner change under the mutex starts here.
uint32_t InstanceExecutor::freezeLocked() noexcept
{
    return m_state.fetch_or(kAttention, std::memory_order_acq_rel) & kOwnerMask;
}

// With the mutex held, after freezeLocked(): publish the owner and recompute whether
// releases must take the locked path.
void InstanceExecutor::publishOwnerLocked(Owner owner) noexcept
{
    m_state.store(static_cast<uint32_t>(owner) | attentionLocked(), std::memory_order_release);
}

void InstanceExecutor::notifyWaitersLocked() noexcept
{
    m_epoch.fetch_add(1, std::memory_order_acq_rel);
    if (m_threadWaiters != 0)
        m_waitCv.notify_all();
    if (m_taskWaiters != 0)
    {
        const ExecutorWaitHooks* hooks = waitHooks();
        if (hooks && hooks->wakeAll)
            hooks->wakeAll(&m_epoch);
    }
}

InstanceExecutor::TaskState InstanceExecutor::taskState(uint64_t index) const
{
    if (index == 0 || index >= m_tasks.nextIndex.load(std::memory_order_relaxed) || m_tasks.isCollected(index))
        return TaskState::kNotFound;
    if (index == m_runningTask)
        return TaskState::kRunning;
    if (isUnfinished(index))
        return TaskState::kPending;
    return m_tasks.failures.count(index) != 0 ? TaskState::kFailed : TaskState::kSucceeded;
}

bool InstanceExecutor::isUnfinished(uint64_t index) const
{
    if (index == m_runningTask)
        return true;
    std::deque<std::unique_ptr<Job>>::const_iterator queued = std::lower_bound(
        m_queue.begin(), m_queue.end(), index,
        [](const std::unique_ptr<Job>& job, uint64_t value) { return job->taskIndex < value; });
    return queued != m_queue.end() && (*queued)->taskIndex == index;
}

uint64_t InstanceExecutor::firstUnfinished(uint64_t cutoff) const
{
    // The running job left the head of the queue, so it precedes every queued job.
    const uint64_t first = m_runningTask ? m_runningTask : (m_queue.empty() ? 0 : m_queue.front()->taskIndex);
    return first <= cutoff ? first : 0;
}

bool InstanceExecutor::workerCanRunLocked() const
{
    return !m_queue.empty() && !m_preemptRequested && !m_hold &&
           static_cast<Owner>(m_state.load(std::memory_order_relaxed) & kOwnerMask) == Owner::kWorker;
}

std::unique_ptr<InstanceExecutor::Job> InstanceExecutor::popHeadLocked()
{
    std::unique_ptr<Job> job = std::move(m_queue.front());
    m_queue.pop_front();
    m_runningTask = job->taskIndex;
    return job;
}

void InstanceExecutor::publishOutcomeLocked(Job& job) noexcept
{
    m_runningTask = 0;
    std::map<uint64_t, std::string>::iterator slot = m_tasks.failures.find(job.taskIndex);
    if (slot == m_tasks.failures.end())
        return;
    if (job.failed)
        slot->second.swap(job.failure);
    else
        m_tasks.failures.erase(slot);
}

InstanceExecutor::LaneHold InstanceExecutor::acquireBusyLane(uint64_t me)
{
    if ((m_state.load(std::memory_order_acquire) & kOwnerMask) != static_cast<uint32_t>(Owner::kFree) &&
        m_holder.load(std::memory_order_relaxed) == me)
    {
        return LaneHold(this, true);
    }

    std::unique_lock<std::mutex> guard(m_mutex);
    ++m_laneWaiters;
    freezeLocked();
    waitLocked(
        guard,
        [this]() {
            return (m_state.load(std::memory_order_relaxed) & kOwnerMask) == static_cast<uint32_t>(Owner::kFree);
        },
        WaitDeadline::infinite());
    --m_laneWaiters;
    freezeLocked();
    publishOwnerLocked(Owner::kCaller);
    m_holder.store(me, std::memory_order_relaxed);
    return LaneHold(this, false);
}

InstanceExecutor::LaneHold InstanceExecutor::preemptLane()
{
    const uint64_t me = currentIdentity();
    if ((m_state.load(std::memory_order_acquire) & kOwnerMask) != static_cast<uint32_t>(Owner::kFree) &&
        m_holder.load(std::memory_order_relaxed) == me)
    {
        return LaneHold(this, true);
    }

    std::unique_lock<std::mutex> guard(m_mutex);
    ++m_laneWaiters;
    m_preemptRequested = true;
    freezeLocked();
    waitLocked(
        guard,
        [this]() {
            const Owner current = static_cast<Owner>(m_state.load(std::memory_order_relaxed) & kOwnerMask);
            return !m_running && (current == Owner::kFree || current == Owner::kWorker);
        },
        WaitDeadline::infinite());
    --m_laneWaiters;
    m_preemptRequested = false;
    freezeLocked();
    publishOwnerLocked(Owner::kCaller);
    m_holder.store(me, std::memory_order_relaxed);
    return LaneHold(this, false);
}

void InstanceExecutor::releaseLaneLocked() noexcept
{
    std::lock_guard<std::mutex> guard(m_mutex);
    handBackLocked();
}

void InstanceExecutor::handBackLocked() noexcept
{
    freezeLocked();
    if (!m_queue.empty())
    {
        publishOwnerLocked(Owner::kWorker);
        m_workerCv.notify_one();
    }
    else
    {
        publishOwnerLocked(Owner::kFree);
    }
    notifyWaitersLocked();
}

uint64_t InstanceExecutor::submit(std::unique_ptr<Job>& job, SubmitStatus& status)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (m_stopping)
    {
        status = SubmitStatus::kStopped;
        return 0;
    }
    if (m_queue.size() >= m_maxQueuedJobs)
    {
        status = SubmitStatus::kQueueFull;
        return 0;
    }
    const uint64_t index = m_tasks.nextIndex.load(std::memory_order_relaxed);
    if (index == UINT64_MAX)
    {
        status = SubmitStatus::kIndexExhausted;
        return 0;
    }
    // Allocate every node before publishing anything, so a failure leaves no trace.
    m_tasks.failures.try_emplace(index);
    job->taskIndex = index;
    try
    {
        m_queue.push_back(std::move(job));
    }
    catch (...)
    {
        m_tasks.failures.erase(index);
        throw;
    }
    m_tasks.nextIndex.store(index + 1, std::memory_order_relaxed);
    if (m_holdAfterSubmit)
    {
        m_holdAfterSubmit = false;
        m_hold = true;
    }

    const Owner current = static_cast<Owner>(freezeLocked());
    if (current == Owner::kFree)
    {
        publishOwnerLocked(Owner::kWorker);
        m_workerCv.notify_one();
    }
    else
    {
        // A worker that owns the lane takes the job after its current one; a lane
        // holder's release hands the lane over and wakes the worker then.
        publishOwnerLocked(current);
    }
    status = SubmitStatus::kAccepted;
    return index;
}

size_t InstanceExecutor::cancelQueued(const char* message)
{
    std::deque<std::unique_ptr<Job>> cancelled;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        cancelled.swap(m_queue);
        for (const std::unique_ptr<Job>& job : cancelled)
        {
            std::map<uint64_t, std::string>::iterator slot = m_tasks.failures.find(job->taskIndex);
            if (slot != m_tasks.failures.end())
            {
                try
                {
                    slot->second = message ? message : "";
                }
                catch (...)
                {
                    // The reserved empty slot still reports the failure.
                }
            }
        }
        // A running job keeps the worker's lane; it frees the lane itself when it finds
        // the queue empty.
        const Owner current = static_cast<Owner>(freezeLocked());
        publishOwnerLocked(current == Owner::kWorker && !m_running ? Owner::kFree : current);
        notifyWaitersLocked();
    }
    const size_t count = cancelled.size();
    cancelled.clear();
    return count;
}

bool InstanceExecutor::headStartableLocked(uint64_t last) const
{
    return !m_queue.empty() && m_queue.front()->taskIndex <= last && !m_preemptRequested && !m_stopping &&
           !m_running && static_cast<Owner>(m_state.load(std::memory_order_relaxed) & kOwnerMask) == Owner::kWorker;
}

void InstanceExecutor::runUnstartedJobs(std::unique_lock<std::mutex>& lock, uint64_t last)
{
    if (headStartableLocked(last) && !taskAwareWaitAvailable())
        runHeadJobsLocked(lock, last);
}

void InstanceExecutor::runSubmittedJobs(uint64_t last)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    for (;;)
    {
        // A caller that holds the lane hands it to the worker when it releases it with jobs
        // queued; with no worker thread this thread takes it then.
        waitLocked(
            lock, [this, last]() { return !isUnfinished(last) || headStartableLocked(last); },
            WaitDeadline::infinite());
        if (!isUnfinished(last))
            return;
        runHeadJobsLocked(lock, last);
    }
}

void InstanceExecutor::runHeadJobsLocked(std::unique_lock<std::mutex>& lock, uint64_t last)
{
    const auto headWaitedFor = [this, last]() {
        return !m_queue.empty() && m_queue.front()->taskIndex <= last &&
               !m_preemptRequested && !m_stopping;
    };
    // Taking the lane keeps the worker from starting a job meanwhile.
    freezeLocked();
    publishOwnerLocked(Owner::kCaller);
    m_holder.store(currentIdentity(), std::memory_order_relaxed);
    for (bool more = true; more;)
    {
        std::unique_ptr<Job> job = popHeadLocked();
        lock.unlock();
        runCapturingFailure(*job);
        lock.lock();
        publishOutcomeLocked(*job);
        more = headWaitedFor();
        if (more)
        {
            notifyWaitersLocked();
        }
        else
        {
            m_holder.store(0, std::memory_order_relaxed);
            handBackLocked();
        }
        lock.unlock();
        job.reset();
        lock.lock();
    }
}

void InstanceExecutor::stopAndJoin()
{
    std::deque<std::unique_ptr<Job>> leftover;
    const bool onWorker = isWorkerThread();
    carb::tasking::ThreadPool* pool = nullptr;
    {
        std::unique_lock<std::mutex> guard(m_mutex);
        m_stopping = true;
        m_stopped.store(true, std::memory_order_release);
        leftover.swap(m_queue);
        for (const std::unique_ptr<Job>& job : leftover)
        {
            std::map<uint64_t, std::string>::iterator slot = m_tasks.failures.find(job->taskIndex);
            if (slot != m_tasks.failures.end())
            {
                try
                {
                    slot->second = "Task cancelled: the instance worker stopped before it started";
                }
                catch (...)
                {
                    // The reserved empty slot still reports the failure.
                }
            }
        }
        m_workerCv.notify_all();
        notifyWaitersLocked();
        // A running job finishes first. Waiting here rather than in destroy() keeps a
        // calling Carbonite task from blocking its thread meanwhile.
        if (!onWorker)
        {
            waitLocked(guard, [this]() { return m_workerExited; }, WaitDeadline::infinite());
            pool = m_pool;
            m_pool = nullptr;
        }
    }
    leftover.clear();
    if (pool)
        m_threads->destroy(pool);
}

void InstanceExecutor::setHold(bool hold)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_hold = hold;
    const Owner current = static_cast<Owner>(freezeLocked());
    publishOwnerLocked(current);
    m_workerCv.notify_all();
    notifyWaitersLocked();
}

void InstanceExecutor::holdAfterNextSubmit()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_holdAfterSubmit = true;
}

bool InstanceExecutor::workerExited() const
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_workerExited;
}

uint32_t InstanceExecutor::waitingTaskCount() const
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_taskWaiters;
}

uint32_t InstanceExecutor::waitingThreadCount() const
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_threadWaiters;
}

uint64_t InstanceExecutor::beginInlineTaskAllocating(FailureSlot& slot)
{
    const uint64_t index = m_tasks.nextIndex.load(std::memory_order_relaxed);
    if (index == UINT64_MAX)
        throw std::overflow_error("Task index exhausted");
    if (!slot)
    {
        if (m_spareFailureSlot)
        {
            slot = std::move(m_spareFailureSlot);
        }
        else
        {
            // The only way to allocate a node is through a map; extracting it keeps the
            // allocation and leaves the scratch map empty.
            std::map<uint64_t, std::string> scratch;
            scratch.try_emplace(0);
            slot = scratch.extract(scratch.begin());
        }
    }
    m_tasks.nextIndex.store(index + 1, std::memory_order_relaxed);
    return index;
}

void InstanceExecutor::endInlineTaskPublishing(uint64_t index, FailureSlot& slot, const char* failure) noexcept
{
    // beginInlineTask() always reserves a slot, so only a second outcome for the same
    // receipt arrives without one; the first outcome stands.
    if (!slot)
        return;
    if (!failure)
    {
        if (!m_spareFailureSlot)
            m_spareFailureSlot = std::move(slot);
        else
            slot = FailureSlot();
        return;
    }
    slot.key() = index;
    try
    {
        slot.mapped() = failure;
    }
    catch (...)
    {
        // The empty entry still reports the failure.
        slot.mapped().clear();
    }
    std::lock_guard<std::mutex> guard(m_mutex);
    m_tasks.failures.insert(std::move(slot));
}

uint64_t InstanceExecutor::createInlineTask()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    const uint64_t index = m_tasks.nextIndex.load(std::memory_order_relaxed);
    if (index == UINT64_MAX)
        throw std::overflow_error("Task index exhausted");
    m_tasks.failures.try_emplace(index);
    m_tasks.nextIndex.store(index + 1, std::memory_order_relaxed);
    return index;
}

void InstanceExecutor::finishInlineTask(uint64_t index, const char* failure) noexcept
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (failure)
    {
        std::map<uint64_t, std::string>::iterator slot = m_tasks.failures.find(index);
        if (slot != m_tasks.failures.end())
        {
            try
            {
                slot->second = failure;
            }
            catch (...)
            {
                // The reserved empty slot still reports the failure.
            }
        }
    }
    else
    {
        m_tasks.failures.erase(index);
    }
}

void InstanceExecutor::workerMain()
{
    const uint64_t me = currentIdentity();
    m_workerIdentity.store(me, std::memory_order_relaxed);
    std::unique_lock<std::mutex> guard(m_mutex);
    for (;;)
    {
        m_workerCv.wait(guard, [this]() { return m_stopping || workerCanRunLocked(); });
        if (m_stopping)
            break;

        std::unique_ptr<Job> job = popHeadLocked();
        m_running = true;
        m_holder.store(me, std::memory_order_relaxed);
        guard.unlock();

        runCapturingFailure(*job);

        guard.lock();
        publishOutcomeLocked(*job);
        // With nothing queued behind the job, the lane is handed back together with its
        // outcome, after it: a caller that the publication unblocks finds the instance idle
        // and runs its next call itself instead of queuing it behind the job's cleanup.
        const bool handedBack = m_queue.empty();
        if (handedBack)
        {
            m_holder.store(0, std::memory_order_relaxed);
            const Owner current = static_cast<Owner>(freezeLocked());
            publishOwnerLocked(current == Owner::kWorker ? Owner::kFree : current);
        }
        notifyWaitersLocked();
        guard.unlock();
        // Captured resources are released after the outcome is visible and outside every
        // executor lock, without the lane once it was handed back above. m_running stays
        // set until then, so preemptLane() (teardown) still waits for them.
        job.reset();
        guard.lock();

        m_running = false;
        if (!handedBack)
        {
            m_holder.store(0, std::memory_order_relaxed);
            if (m_queue.empty())
            {
                const Owner current = static_cast<Owner>(freezeLocked());
                publishOwnerLocked(current == Owner::kWorker ? Owner::kFree : current);
            }
        }
        // Waiters for the outcome were woken above. Only a teardown waits for the
        // destruction, and a lane waiter for a lane freed here.
        if (m_preemptRequested || (!handedBack && m_queue.empty()))
            notifyWaitersLocked();
    }

    const Owner current = static_cast<Owner>(freezeLocked());
    publishOwnerLocked(current == Owner::kWorker && m_queue.empty() ? Owner::kFree : current);
    m_workerExited = true;
    m_workerIdentity.store(0, std::memory_order_relaxed);
    notifyWaitersLocked();
}

} // namespace ovphysx
