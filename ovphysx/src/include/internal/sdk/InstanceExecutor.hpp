// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace carb::tasking
{
struct IThreadPool;
class ThreadPool;
} // namespace carb::tasking

namespace ovphysx
{

// Hooks that keep executor waits from blocking a Carbonite worker thread. The SDK
// installs them once Carbonite tasking runs; without them every wait blocks the
// calling OS thread. Isolated tests install fakes.
struct ExecutorWaitHooks
{
    // Nonzero identity of the calling Carbonite task, or zero outside a task.
    uint64_t (*currentTask)() = nullptr;
    // Suspend the calling task until *word differs from expected, a wake-up arrives,
    // or timeoutNs elapses. UINT64_MAX waits without a deadline. Spurious returns are allowed.
    void (*wait)(const std::atomic<uint32_t>* word, uint32_t expected, uint64_t timeoutNs) = nullptr;
    // Wake every task suspended in wait() on word.
    void (*wakeAll)(const std::atomic<uint32_t>* word) = nullptr;
    // Keep the calling task on its OS thread while it waits, so thread-bound caller
    // state (current CUDA context, per-thread default stream) is unchanged when it
    // resumes. Returns true when this call pinned the task.
    bool (*pin)() = nullptr;
    void (*unpin)() = nullptr;
};

// Installs process-wide wait hooks. The pointed-to table must stay valid until it
// is replaced. Pass nullptr to remove them.
void setExecutorWaitHooks(const ExecutorWaitHooks* hooks) noexcept;

// A readiness deadline for executor waits.
struct WaitDeadline
{
    enum class Kind : uint8_t
    {
        kPoll,
        kFinite,
        kInfinite,
    };
    Kind kind = Kind::kInfinite;
    std::chrono::steady_clock::time_point at{};

    static WaitDeadline poll() noexcept { return {Kind::kPoll, {}}; }
    static WaitDeadline infinite() noexcept { return {Kind::kInfinite, {}}; }
    // Zero is a poll and UINT64_MAX never expires. Large finite values saturate.
    static WaitDeadline fromTimeoutNs(uint64_t timeoutNs, std::chrono::steady_clock::time_point start) noexcept;
};

// One worker thread per instance, the dedicated thread of a one-thread Carbonite pool
// (carb::tasking::IThreadPool), a FIFO of jobs, and the instance "lane": a single
// word that names who runs engine code for the instance right now. A thread that waits
// without a deadline for jobs the worker has not started runs them itself (see
// runUnstartedJobs()). An executor from createWithoutWorker() has no worker thread: the
// thread that submits a job runs it (see runSubmittedJobs()).
//
// Lane invariants:
//   - Only the lane holder runs engine code for the instance.
//   - A free lane implies an empty queue.
//   - The worker frees the lane only while holding the queue mutex with the queue empty.
//   - A submission that finds the lane free hands it to the worker and wakes the worker
//     inside the same critical section, so no wake-up is lost and no later call can run
//     before earlier queued work.
//   - A caller that takes the lane waits until the worker drained the queue.
//
// The executor also owns the task table of its instance: receipt indices, reserved
// failure slots, unfinished jobs and collected ranges. The worker touches it only under
// the executor mutex while it owns the lane, and publishes a job outcome before it can
// release the lane. Calls on one instance are externally serialized, so the caller
// needs the mutex only while the worker may publish: work the caller runs itself draws
// its receipt without it (beginInlineTask), and a caller that finds the lane idle reads
// and collects the table without it (laneIdle).
class InstanceExecutor
{
public:
    enum class Owner : uint32_t
    {
        kFree = 0,
        kCaller = 1,
        kWorker = 2,
    };

    enum class TaskState : uint32_t
    {
        kNotFound = 0,
        kPending = 1,
        kRunning = 2,
        kSucceeded = 3,
        kFailed = 4,
    };

    enum class SubmitStatus : uint32_t
    {
        kAccepted,
        kStopped,
        kQueueFull,
        kIndexExhausted,
    };

    // Work executed by the lane holder: the worker thread, or a waiting thread that runs
    // the job in its place (runUnstartedJobs()). run() executes without the executor mutex,
    // records its outcome in failed/failure and may throw; an exception becomes the
    // failure. The job object is destroyed after its outcome was published, outside every
    // executor lock. The lane may have been handed back by then, so destroying a job must
    // not need the lane (its resources are inputs and bookkeeping, never engine state);
    // teardown still waits for it.
    class Job
    {
    public:
        virtual ~Job() = default;
        virtual void run() = 0;

        // Assigned at submission.
        uint64_t taskIndex = 0;
        bool failed = false;
        std::string failure;
    };

    struct TaskTable
    {
        // Written only by the externally serialized caller (receipts and submissions).
        // Atomic, so readers that hold the mutex on another thread never race with it.
        std::atomic<uint64_t> nextIndex{1};
        // Reserved failure slots of jobs and createInlineTask() receipts (empty until a
        // failure is recorded) and recorded failures. A terminal task without an entry
        // succeeded.
        std::map<uint64_t, std::string> failures;
        uint64_t collectedPrefix = 0;
        std::map<uint64_t, uint64_t> collectedRanges;

        // Inline for the common case of a caller collecting in order: everything up to
        // the prefix is collected and no out-of-order range exists.
        bool isCollected(uint64_t index) const
        {
            return index <= collectedPrefix || (!collectedRanges.empty() && isCollectedInRanges(index));
        }
        // Inserts at most one interval; the node is allocated before existing state
        // changes, so merging and prefix compaction cannot throw.
        void collectRange(uint64_t first, uint64_t last)
        {
            if (first == collectedPrefix + 1 && collectedRanges.empty())
            {
                if (last >= first)
                    collectedPrefix = last;
                return;
            }
            collectRangeMerging(first, last);
        }

    private:
        bool isCollectedInRanges(uint64_t index) const;
        void collectRangeMerging(uint64_t first, uint64_t last);
    };

    // Movable ownership of the lane. Releasing hands the lane to the worker when jobs
    // are queued and wakes callers waiting for it. Inline, so that a free-lane hold costs
    // the two compare-and-swaps and no calls.
    class LaneHold
    {
    public:
        LaneHold() = default;
        LaneHold(const LaneHold&) = delete;
        LaneHold& operator=(const LaneHold&) = delete;
        LaneHold(LaneHold&& other) noexcept : m_executor(other.m_executor), m_nested(other.m_nested)
        {
            other.m_executor = nullptr;
            other.m_nested = false;
        }
        LaneHold& operator=(LaneHold&& other) noexcept
        {
            if (this != &other)
            {
                release();
                m_executor = other.m_executor;
                m_nested = other.m_nested;
                other.m_executor = nullptr;
                other.m_nested = false;
            }
            return *this;
        }
        ~LaneHold() { release(); }

        void release() noexcept
        {
            InstanceExecutor* executor = m_executor;
            if (!executor)
                return;
            m_executor = nullptr;
            if (m_nested)
            {
                m_nested = false;
                return;
            }
            executor->releaseLane();
        }
        bool held() const noexcept { return m_executor != nullptr; }
        bool nested() const noexcept { return m_nested; }

    private:
        friend class InstanceExecutor;
        LaneHold(InstanceExecutor* executor, bool nested) noexcept : m_executor(executor), m_nested(nested) {}
        InstanceExecutor* m_executor = nullptr;
        bool m_nested = false;
    };

    // Starts the worker loop as the one job of a new one-thread pool from threads. Throws
    // when the pool or its job cannot be created.
    static std::shared_ptr<InstanceExecutor> create(carb::tasking::IThreadPool& threads,
                                                    size_t maxQueuedJobs = kDefaultMaxQueuedJobs);
    // An executor without a worker thread.
    static std::shared_ptr<InstanceExecutor> createWithoutWorker();
    ~InstanceExecutor();

    InstanceExecutor(const InstanceExecutor&) = delete;
    InstanceExecutor& operator=(const InstanceExecutor&) = delete;

    static constexpr size_t kDefaultMaxQueuedJobs = 1u << 16;

    // ---- Lane ----

    // Waits until the worker drained the queue and no other holder remains, then takes
    // the lane for the calling task or thread. A holder that asks again gets a nested
    // hold, which is a no-op.
    LaneHold acquireLane() { return acquireLane(currentIdentity()); }
    // The same for a caller that already resolved its identity (identityOf()). Taking a
    // free lane is one compare-and-swap, inline.
    LaneHold acquireLane(uint64_t identity)
    {
        uint32_t expected = static_cast<uint32_t>(Owner::kFree);
        if (m_state.compare_exchange_strong(expected, static_cast<uint32_t>(Owner::kCaller), std::memory_order_acquire,
                                            std::memory_order_relaxed))
        {
            m_holder.store(identity, std::memory_order_relaxed);
            return LaneHold(this, false);
        }
        return acquireBusyLane(identity);
    }
    // Takes the lane for a caller that resolved its identity, only when the instance is
    // idle right now (see laneIdle()), with the one compare-and-swap of a free lane.
    // Otherwise returns an empty hold without waiting; a hold never nests here.
    LaneHold tryAcquireIdleLane(uint64_t identity) noexcept
    {
        uint32_t expected = static_cast<uint32_t>(Owner::kFree);
        if (!m_state.compare_exchange_strong(expected, static_cast<uint32_t>(Owner::kCaller), std::memory_order_acquire,
                                             std::memory_order_relaxed))
            return LaneHold();
        m_holder.store(identity, std::memory_order_relaxed);
        return LaneHold(this, false);
    }
    // Takes the lane between jobs without draining the queue: waits for the running job
    // to finish, then holds the lane while queued jobs stay queued. Releasing the hold
    // resumes the worker. Used by instance teardown, which then either cancels the queue
    // or releases the hold with the queue intact.
    LaneHold preemptLane();
    Owner owner() const noexcept;
    // True when the lane is free with nothing queued and nobody waiting for it or
    // holding the worker. The worker then cannot touch the task table before the
    // externally serialized caller submits again, so that caller may read and update the
    // table without the mutex. The acquire load makes everything the worker published
    // before it freed the lane visible.
    bool laneIdle() const noexcept
    {
        return m_state.load(std::memory_order_acquire) == static_cast<uint32_t>(Owner::kFree);
    }
    // True when the calling task or thread holds the lane (including the worker inside a job).
    bool heldByCurrent() const noexcept;
    // True on this executor's own worker thread while its loop runs, whether or not it is
    // running a job.
    bool isWorkerThread() const noexcept { return isWorkerIdentity(identityOf(0)); }
    bool hasWorker() const noexcept { return m_threads != nullptr; }
    // The same for a caller that already resolved its identity: the worker is a native
    // thread, so no Carbonite task ever matches.
    bool isWorkerIdentity(uint64_t identity) const noexcept
    {
        return identity == m_workerIdentity.load(std::memory_order_relaxed);
    }

    // The identity the lane records for its holder: the Carbonite task whose context is
    // task (nonzero), which may move between threads, or else the calling native thread.
    static uint64_t identityOf(uint64_t task) noexcept;
    // identityOf() for the calling task or thread, as reported by the wait hooks.
    static uint64_t currentIdentity() noexcept;

    // ---- Jobs ----

    // Accepts a job: assigns its receipt index, reserves its failure slot and queues it.
    // Returns the index, or zero with status set when the job was rejected (the job is then
    // destroyed by the caller's unique_ptr).
    uint64_t submit(std::unique_ptr<Job>& job, SubmitStatus& status);

    // Cancels every queued job that has not started and returns how many it removed. Their
    // task entries are retired as failures with the given message. The caller must hold the
    // lane (normally through preemptLane()). Jobs are destroyed outside the executor mutex.
    size_t cancelQueued(const char* message);

    // For a native thread about to wait without a deadline for the jobs up to index last,
    // with lock (from lock()) held: runs those jobs on the calling thread while the worker
    // has not started them, so the waiting thread does not wait for the worker to wake up.
    // Each runs only while it is at the head of the queue and the worker owns the lane
    // without running a job (a held worker included) and no teardown preempts it: the
    // calling thread takes the lane from the worker, runs the job outside the mutex,
    // publishes its outcome as the worker would and hands the lane back, to the worker
    // when jobs remain queued. Inside a Carbonite task it runs nothing: such a task waits
    // suspended. Returns with lock held, after the last job it ran was destroyed.
    void runUnstartedJobs(std::unique_lock<std::mutex>& lock, uint64_t last);

    // For an executor without a worker thread, on the thread that submitted the jobs up to
    // index last: runs them on that thread, as runUnstartedJobs() does, also inside a
    // Carbonite task. While another caller holds the lane it waits until that caller hands
    // the lane over. Returns once job last finished.
    void runSubmittedJobs(uint64_t last);

    // Stops the worker after its current job and ends its pool thread (IThreadPool
    // destroy(), which joins it). Queued jobs must already be drained or cancelled. Later
    // submissions are rejected. Safe to call more than once. The caller waits for the
    // worker to leave its loop as waitLocked() waits (inside a Carbonite task the task is
    // suspended), so the join only covers the job's return and the thread's exit. On the
    // worker thread itself it only stops the loop: a pool thread cannot end itself, so a
    // later call from another thread ends it.
    void stopAndJoin();
    // Lock-free. A caller that takes the lane after a stopAndJoin() made while the lane
    // was held sees true.
    bool stopped() const noexcept { return m_stopped.load(std::memory_order_acquire); }

    // Test gate: while held the worker starts no job; submissions still queue.
    void setHold(bool hold);
    // Test gate: the next accepted submission sets the hold as it queues its job, so a
    // job queued on a free lane stays queued without the hold having made the lane busy
    // before the submission.
    void holdAfterNextSubmit();
    // Diagnostics for tests: true once the worker thread left its loop for good.
    bool workerExited() const;
    // Diagnostics for tests: callers currently suspended in a task-aware wait, and
    // threads currently blocked in a condition-variable wait.
    uint32_t waitingTaskCount() const;
    uint32_t waitingThreadCount() const;

    // ---- Task table ----

    // Storage for one task failure: a TaskTable::failures node, reserved before the task
    // runs so that recording its failure never allocates.
    using FailureSlot = std::map<uint64_t, std::string>::node_type;

    // Reserves a receipt index and failure storage for work the calling thread runs and
    // finishes before its call returns. Only the externally serialized caller uses it: it
    // touches nothing the worker reads, so it takes no mutex, and it reuses a slot cached
    // by an earlier success instead of allocating one. Throws std::overflow_error on
    // index exhaustion and std::bad_alloc when no slot can be reserved; either leaves no
    // trace. Inline when the cached slot is available.
    uint64_t beginInlineTask(FailureSlot& slot)
    {
        const uint64_t index = m_tasks.nextIndex.load(std::memory_order_relaxed);
        if (index == UINT64_MAX || (!slot && !m_spareFailureSlot))
            return beginInlineTaskAllocating(slot);
        if (!slot)
            slot = std::move(m_spareFailureSlot);
        m_tasks.nextIndex.store(index + 1, std::memory_order_relaxed);
        return index;
    }
    // Records the outcome of a receipt from beginInlineTask(). A success publishes nothing
    // (a terminal task without a failure entry succeeded) and caches the slot; a failure
    // moves the slot, holding the message, into the table under the mutex. Never throws;
    // a message that cannot be stored leaves the entry empty, which still reports failure.
    void endInlineTask(uint64_t index, FailureSlot& slot, const char* failure) noexcept
    {
        if (!failure && slot && !m_spareFailureSlot)
        {
            m_spareFailureSlot = std::move(slot);
            return;
        }
        endInlineTaskPublishing(index, slot, failure);
    }

    // Reserves a receipt index and its failure slot in the table, under the mutex, for
    // work the calling thread runs without the cached slot of beginInlineTask(). Throws
    // std::overflow_error on index exhaustion and std::bad_alloc when the slot cannot be
    // reserved.
    uint64_t createInlineTask();
    // Records the outcome of a receipt from createInlineTask(). failure == nullptr means
    // success. Never throws; the reserved empty slot preserves a failure whose text cannot
    // be stored.
    void finishInlineTask(uint64_t index, const char* failure) noexcept;

    // Locks the executor mutex, which guards the task table.
    std::unique_lock<std::mutex> lock() { return std::unique_lock<std::mutex>(m_mutex); }
    TaskTable& tasks() noexcept { return m_tasks; }
    const TaskTable& tasks() const noexcept { return m_tasks; }
    // With lock() held: the state of task index. A job is pending while queued and running
    // until its outcome is published.
    TaskState taskState(uint64_t index) const;
    // With lock() held: whether the job of a nonzero task index is queued or running.
    bool isUnfinished(uint64_t index) const;
    // With lock() held: the smallest unfinished task index at or below cutoff, or zero.
    uint64_t firstUnfinished(uint64_t cutoff) const;

    // Waits with lock held (from lock()) until pred() is true or the deadline passes, and
    // returns pred(). Never runs work. Inside a Carbonite task the task is suspended
    // instead of blocking its OS thread.
    template <typename Predicate>
    bool waitLocked(std::unique_lock<std::mutex>& lock, Predicate pred, const WaitDeadline& deadline)
    {
        if (pred())
            return true;
        if (deadline.kind == WaitDeadline::Kind::kPoll)
            return false;
        if (taskAwareWaitAvailable())
            return waitAsTask(lock, pred, deadline);
        ++m_threadWaiters;
        bool satisfied = false;
        if (deadline.kind == WaitDeadline::Kind::kInfinite)
        {
            m_waitCv.wait(lock, pred);
            satisfied = true;
        }
        else
        {
            satisfied = m_waitCv.wait_until(lock, deadline.at, pred);
        }
        --m_threadWaiters;
        return satisfied;
    }

private:
    static constexpr uint32_t kOwnerMask = 0x3u;
    static constexpr uint32_t kAttention = 0x4u;

    explicit InstanceExecutor(size_t maxQueuedJobs);
    // The pool job of create(): self is the heap reference create() made for the worker.
    static void workerJob(void* self);
    void workerMain();
    // Inline release of a held lane: one compare-and-swap when nobody waits and nothing
    // is queued, otherwise the locked hand-over.
    void releaseLane() noexcept
    {
        m_holder.store(0, std::memory_order_relaxed);
        uint32_t expected = static_cast<uint32_t>(Owner::kCaller);
        if (!m_state.compare_exchange_strong(expected, static_cast<uint32_t>(Owner::kFree), std::memory_order_release,
                                             std::memory_order_relaxed))
            releaseLaneLocked();
    }
    void releaseLaneLocked() noexcept;
    // With the mutex held: hands the lane to the worker when jobs are queued, frees it
    // otherwise, and wakes every waiter.
    void handBackLocked() noexcept;
    // acquireLane() when the lane is not free: a nested hold for the current holder,
    // otherwise the locked wait.
    LaneHold acquireBusyLane(uint64_t identity);
    // The rest of beginInlineTask() and endInlineTask(): allocating a slot, and the
    // outcomes that do not just cache it.
    uint64_t beginInlineTaskAllocating(FailureSlot& slot);
    void endInlineTaskPublishing(uint64_t index, FailureSlot& slot, const char* failure) noexcept;
    bool workerCanRunLocked() const;
    // With the mutex held: whether the job at the head of the queue, up to index last, may
    // run on a thread other than the worker: the worker owns the lane but runs no job (it has
    // not woken up yet, a test holds it, or there is no worker thread) and no teardown
    // preempts it.
    bool headStartableLocked(uint64_t last) const;
    // With lock held and headStartableLocked(last): takes the lane from the worker, runs the
    // queued jobs up to last on the calling thread as the worker would, publishing each
    // outcome, and hands the lane back. Returns with lock held, after the last job it ran
    // was destroyed.
    void runHeadJobsLocked(std::unique_lock<std::mutex>& lock, uint64_t last);
    // With the mutex held: pops the job at the head of the queue and marks it running.
    std::unique_ptr<Job> popHeadLocked();
    // With the mutex held, after job ran: retires its task entry as succeeded or failed.
    void publishOutcomeLocked(Job& job) noexcept;
    uint32_t freezeLocked() noexcept;
    void publishOwnerLocked(Owner owner) noexcept;
    uint32_t attentionLocked() const noexcept;
    void notifyWaitersLocked() noexcept;
    bool taskAwareWaitAvailable() const noexcept;

    template <typename Predicate>
    bool waitAsTask(std::unique_lock<std::mutex>& lock, Predicate& pred, const WaitDeadline& deadline);
    bool pinCurrentTask() noexcept;
    void unpinCurrentTask() noexcept;
    void suspendCurrentTask(uint32_t seenEpoch, uint64_t timeoutNs) noexcept;

    const size_t m_maxQueuedJobs;
    std::atomic<uint32_t> m_state{0};
    std::atomic<uint64_t> m_holder{0};

    mutable std::mutex m_mutex;
    std::condition_variable m_workerCv;
    std::condition_variable m_waitCv;
    std::atomic<uint32_t> m_epoch{0};
    uint32_t m_threadWaiters = 0;
    uint32_t m_taskWaiters = 0;
    uint32_t m_laneWaiters = 0;
    // In ascending task index order.
    std::deque<std::unique_ptr<Job>> m_queue;
    TaskTable m_tasks;
    // The task index of the job popped from the queue whose outcome is not published yet,
    // or zero. There is at most one: only the lane holder runs jobs.
    uint64_t m_runningTask = 0;
    // A failure slot left by a succeeded calling-thread receipt, for the next one.
    // Touched only by the externally serialized caller (beginInlineTask, endInlineTask).
    FailureSlot m_spareFailureSlot;
    bool m_running = false;
    bool m_preemptRequested = false;
    bool m_hold = false;
    bool m_holdAfterSubmit = false;
    bool m_stopping = false;
    // Mirrors m_stopping for lock-free readers.
    std::atomic<bool> m_stopped{false};
    bool m_workerExited = false;
    // Written once by create() before the executor is shared with any other thread; null
    // for an executor without a worker.
    carb::tasking::IThreadPool* m_threads = nullptr;
    // The worker's pool until stopAndJoin() ends it; touched only by create() and, under
    // the mutex, by stopAndJoin().
    carb::tasking::ThreadPool* m_pool = nullptr;
    // The worker's identity while it runs, written by the worker itself; zero before it
    // starts and after it exits, so it never matches a later thread.
    std::atomic<uint64_t> m_workerIdentity{0};
};

template <typename Predicate>
bool InstanceExecutor::waitAsTask(std::unique_lock<std::mutex>& lock, Predicate& pred, const WaitDeadline& deadline)
{
    const bool pinned = pinCurrentTask();
    ++m_taskWaiters;
    bool satisfied = pred();
    while (!satisfied)
    {
        uint64_t timeoutNs = UINT64_MAX;
        if (deadline.kind == WaitDeadline::Kind::kFinite)
        {
            const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if (now >= deadline.at)
                break;
            const std::chrono::nanoseconds remaining =
                std::chrono::duration_cast<std::chrono::nanoseconds>(deadline.at - now);
            timeoutNs = remaining.count() > 0 ? static_cast<uint64_t>(remaining.count()) : 0;
        }
        const uint32_t seen = m_epoch.load(std::memory_order_acquire);
        lock.unlock();
        suspendCurrentTask(seen, timeoutNs);
        lock.lock();
        satisfied = pred();
    }
    --m_taskWaiters;
    if (pinned)
        unpinCurrentTask();
    return satisfied;
}

} // namespace ovphysx
