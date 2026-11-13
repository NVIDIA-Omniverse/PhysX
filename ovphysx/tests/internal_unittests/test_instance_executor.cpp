// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1
 *
 * TEST-CAPI-EXECUTOR-001: executor order, wake-ups, publication and teardown in isolation.
 */

// Isolated tests of the per-instance worker, FIFO and lane. They link only the
// executor source and a stand-in for Carbonite's thread pool, so lane bugs show up
// without Carbonite, PhysX or a stage.

#include "internal/sdk/InstanceExecutor.hpp"

#include <carb/tasking/IThreadPool.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if defined(__linux__)
#    include <pthread.h>
#endif

namespace
{

using ovphysx::InstanceExecutor;
using ovphysx::WaitDeadline;
using Owner = InstanceExecutor::Owner;
using TaskState = InstanceExecutor::TaskState;

class FunctionJob : public InstanceExecutor::Job
{
public:
    explicit FunctionJob(std::function<void(FunctionJob&)> body, std::function<void()> onDestroy = {})
        : m_body(std::move(body)), m_onDestroy(std::move(onDestroy))
    {
    }
    ~FunctionJob() override
    {
        if (m_onDestroy)
            m_onDestroy();
    }
    void run() override { m_body(*this); }

private:
    std::function<void(FunctionJob&)> m_body;
    std::function<void()> m_onDestroy;
};

uint64_t submitFn(InstanceExecutor& executor, std::function<void(FunctionJob&)> body,
                  std::function<void()> onDestroy = {})
{
    std::unique_ptr<InstanceExecutor::Job> job(new FunctionJob(std::move(body), std::move(onDestroy)));
    InstanceExecutor::SubmitStatus status = InstanceExecutor::SubmitStatus::kStopped;
    const uint64_t index = executor.submit(job, status);
    EXPECT_EQ(status, InstanceExecutor::SubmitStatus::kAccepted);
    return index;
}

TaskState stateOf(InstanceExecutor& executor, uint64_t index)
{
    std::unique_lock<std::mutex> lock = executor.lock();
    return executor.taskState(index);
}

bool waitTerminal(InstanceExecutor& executor, uint64_t index, std::chrono::milliseconds timeout = std::chrono::seconds(30))
{
    std::unique_lock<std::mutex> lock = executor.lock();
    const WaitDeadline deadline{WaitDeadline::Kind::kFinite, std::chrono::steady_clock::now() + timeout};
    return executor.waitLocked(
        lock, [&]() { return !executor.isUnfinished(index); }, deadline);
}

void drain(InstanceExecutor& executor)
{
    InstanceExecutor::LaneHold hold = executor.acquireLane();
    ASSERT_TRUE(hold.held());
}

// drain() and also waits until the worker destroyed the job it ran last: with nothing
// queued behind it, a finished job is destroyed after the lane was handed back.
void settle(InstanceExecutor& executor)
{
    InstanceExecutor::LaneHold hold = executor.preemptLane();
    ASSERT_TRUE(hold.held());
}

// Simple latch usable from jobs.
class Gate
{
public:
    void open()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_open = true;
        m_cv.notify_all();
    }
    bool waitOpen(std::chrono::milliseconds timeout = std::chrono::seconds(30))
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_cv.wait_for(lock, timeout, [this]() { return m_open; });
    }

private:
    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_open = false;
};

// Stands in for Carbonite's carb::tasking::IThreadPool, from which each executor takes its
// worker thread. The executor enqueues one job per pool, so each job gets a thread of its
// own, which destroy() joins before it frees the pool. g_poolCalls counts the calls;
// tests create executors on the main test thread, so only the counters are atomic.
struct TestPool
{
    std::thread thread;
};

struct PoolCalls
{
    std::atomic<int> created{0};
    std::atomic<size_t> workers{0};
    std::atomic<int> enqueued{0};
    std::atomic<int> destroyed{0};
    std::thread::id lastThread;
    bool failCreate = false;
    bool failEnqueue = false;

    void reset()
    {
        created = 0;
        workers = 0;
        enqueued = 0;
        destroyed = 0;
        lastThread = std::thread::id();
        failCreate = false;
        failEnqueue = false;
    }
};
PoolCalls g_poolCalls;

carb::tasking::ThreadPool* testCreateEx(size_t workers)
{
    if (g_poolCalls.failCreate)
        return nullptr;
    ++g_poolCalls.created;
    g_poolCalls.workers = workers;
    return reinterpret_cast<carb::tasking::ThreadPool*>(new TestPool);
}

void testWaitUntilFinished(carb::tasking::ThreadPool* pool)
{
    TestPool* testPool = reinterpret_cast<TestPool*>(pool);
    if (testPool->thread.joinable())
        testPool->thread.join();
}

void testDestroy(carb::tasking::ThreadPool* pool)
{
    ++g_poolCalls.destroyed;
    TestPool* testPool = reinterpret_cast<TestPool*>(pool);
    if (testPool->thread.joinable())
        testPool->thread.join();
    delete testPool;
}

size_t testWorkerCount(carb::tasking::ThreadPool*)
{
    return 1;
}

bool testEnqueueJob(carb::tasking::ThreadPool* pool, carb::tasking::JobFn fn, void* data)
{
    if (g_poolCalls.failEnqueue)
        return false;
    ++g_poolCalls.enqueued;
    TestPool* testPool = reinterpret_cast<TestPool*>(pool);
    testPool->thread = std::thread(fn, data);
    g_poolCalls.lastThread = testPool->thread.get_id();
    return true;
}

size_t testRunningJobCount(carb::tasking::ThreadPool*)
{
    return 0;
}

size_t testDefaultWorkerCount()
{
    return 1;
}

carb::tasking::IThreadPool g_testThreads{testCreateEx,   testDestroy,         testDefaultWorkerCount, testWorkerCount,
                                         testEnqueueJob, testRunningJobCount, testWaitUntilFinished};

class ExecutorTest : public ::testing::Test
{
protected:
    void SetUp() override { executor = InstanceExecutor::create(g_testThreads); }
    void TearDown() override
    {
        if (executor)
        {
            executor->setHold(false);
            drain(*executor);
            executor->stopAndJoin();
        }
        ovphysx::setExecutorWaitHooks(nullptr);
    }
    std::shared_ptr<InstanceExecutor> executor;
};

TEST_F(ExecutorTest, FreeLaneIsTakenAndReleasedWithoutTheWorker)
{
    EXPECT_EQ(executor->owner(), Owner::kFree);
    {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        EXPECT_TRUE(hold.held());
        EXPECT_FALSE(hold.nested());
        EXPECT_EQ(executor->owner(), Owner::kCaller);
        EXPECT_TRUE(executor->heldByCurrent());
    }
    EXPECT_EQ(executor->owner(), Owner::kFree);
    EXPECT_FALSE(executor->heldByCurrent());
}

TEST_F(ExecutorTest, JobRunsOnTheWorkerAndPublishesSuccess)
{
    std::atomic<std::thread::id> ranOn{};
    const uint64_t index = submitFn(*executor, [&](FunctionJob&) { ranOn = std::this_thread::get_id(); });
    ASSERT_NE(index, 0u);
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_EQ(stateOf(*executor, index), TaskState::kSucceeded);
    EXPECT_NE(ranOn.load(), std::this_thread::get_id());
    drain(*executor);
    EXPECT_EQ(executor->owner(), Owner::kFree);
}

TEST_F(ExecutorTest, HeldWorkerLeavesTheTaskPendingUntilReleased)
{
    executor->setHold(true);
    std::atomic<bool> ran{false};
    const uint64_t index = submitFn(*executor, [&](FunctionJob&) { ran = true; });
    EXPECT_EQ(stateOf(*executor, index), TaskState::kPending);
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        EXPECT_FALSE(executor->waitLocked(
            lock, [&]() { return !executor->isUnfinished(index); }, WaitDeadline::poll()));
        const WaitDeadline shortWait{WaitDeadline::Kind::kFinite,
                                     std::chrono::steady_clock::now() + std::chrono::milliseconds(20)};
        EXPECT_FALSE(executor->waitLocked(
            lock, [&]() { return !executor->isUnfinished(index); }, shortWait));
    }
    EXPECT_FALSE(ran.load());
    EXPECT_EQ(executor->owner(), Owner::kWorker);
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_TRUE(ran.load());
    EXPECT_EQ(stateOf(*executor, index), TaskState::kSucceeded);
}

TEST_F(ExecutorTest, RunningJobReportsRunning)
{
    Gate entered;
    Gate release;
    const uint64_t index = submitFn(*executor, [&](FunctionJob&) {
        entered.open();
        release.waitOpen();
    });
    const uint64_t queued = submitFn(*executor, [](FunctionJob&) {});
    ASSERT_TRUE(entered.waitOpen());
    EXPECT_EQ(stateOf(*executor, index), TaskState::kRunning);
    EXPECT_EQ(stateOf(*executor, queued), TaskState::kPending);
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        EXPECT_EQ(executor->firstUnfinished(index - 1), 0u);
        EXPECT_EQ(executor->firstUnfinished(queued), index) << "the running job comes before the queued one";
    }
    release.open();
    ASSERT_TRUE(waitTerminal(*executor, queued));
    std::unique_lock<std::mutex> lock = executor->lock();
    EXPECT_EQ(executor->firstUnfinished(queued), 0u);
}

TEST_F(ExecutorTest, JobsRunInSubmissionOrder)
{
    std::vector<int> order;
    std::mutex orderMutex;
    constexpr int kJobs = 500;
    uint64_t last = 0;
    for (int i = 0; i < kJobs; ++i)
    {
        last = submitFn(*executor, [&, i](FunctionJob&) {
            std::lock_guard<std::mutex> lock(orderMutex);
            order.push_back(i);
        });
    }
    ASSERT_TRUE(waitTerminal(*executor, last));
    drain(*executor);
    ASSERT_EQ(order.size(), static_cast<size_t>(kJobs));
    for (int i = 0; i < kJobs; ++i)
        EXPECT_EQ(order[static_cast<size_t>(i)], i);
}

TEST_F(ExecutorTest, CallerLaneWaitsForEarlierJobs)
{
    executor->setHold(true);
    std::atomic<int> sequence{0};
    std::atomic<int> jobSeen{-1};
    std::atomic<int> callerSeen{-1};
    submitFn(*executor, [&](FunctionJob&) { jobSeen = sequence.fetch_add(1); });
    std::atomic<bool> callerStarted{false};
    std::thread caller([&]() {
        callerStarted = true;
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        callerSeen = sequence.fetch_add(1);
    });
    while (!callerStarted.load())
        std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_EQ(callerSeen.load(), -1) << "caller must not take the lane while a job is queued";
    executor->setHold(false);
    caller.join();
    EXPECT_EQ(jobSeen.load(), 0);
    EXPECT_EQ(callerSeen.load(), 1);
}

TEST_F(ExecutorTest, SubmissionWhileCallerHoldsTheLaneRunsAfterRelease)
{
    std::atomic<bool> ran{false};
    uint64_t index = 0;
    {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        index = submitFn(*executor, [&](FunctionJob&) { ran = true; });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        EXPECT_FALSE(ran.load());
        EXPECT_EQ(executor->owner(), Owner::kCaller);
    }
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_TRUE(ran.load());
}

TEST_F(ExecutorTest, NestedAcquisitionIsANoOp)
{
    InstanceExecutor::LaneHold outer = executor->acquireLane();
    {
        InstanceExecutor::LaneHold inner = executor->acquireLane();
        EXPECT_TRUE(inner.nested());
    }
    EXPECT_EQ(executor->owner(), Owner::kCaller);
    outer.release();
    EXPECT_EQ(executor->owner(), Owner::kFree);
}

TEST_F(ExecutorTest, WorkerThreadIsRecognizedOnlyOnTheWorker)
{
    EXPECT_FALSE(executor->isWorkerThread());
    EXPECT_FALSE(executor->isWorkerIdentity(InstanceExecutor::currentIdentity()));
    std::atomic<bool> onWorker{false};
    std::atomic<bool> identityOnWorker{false};
    std::atomic<uint64_t> workerIdentity{0};
    const uint64_t index = submitFn(*executor, [&](FunctionJob&) {
        onWorker = executor->isWorkerThread();
        workerIdentity = InstanceExecutor::currentIdentity();
        identityOnWorker = executor->isWorkerIdentity(workerIdentity.load());
    });
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_TRUE(onWorker.load());
    EXPECT_TRUE(identityOnWorker.load());
    EXPECT_FALSE(executor->isWorkerIdentity(InstanceExecutor::identityOf(42))) << "a task is never the worker";
    std::thread other([&]() {
        EXPECT_FALSE(executor->isWorkerThread());
        EXPECT_FALSE(executor->isWorkerIdentity(InstanceExecutor::currentIdentity()));
    });
    other.join();
    drain(*executor);
    executor->stopAndJoin();
    EXPECT_FALSE(executor->isWorkerIdentity(workerIdentity.load())) << "an exited worker matches no later thread";
}

TEST_F(ExecutorTest, JobMayReacquireItsOwnLane)
{
    std::atomic<bool> nested{false};
    const uint64_t index = submitFn(*executor, [&](FunctionJob&) {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        nested = hold.nested();
    });
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_TRUE(nested.load());
}

TEST_F(ExecutorTest, IdleLaneIsTakenOnlyWithoutQueuedRunningOrHeldWork)
{
    const uint64_t me = InstanceExecutor::currentIdentity();
    {
        InstanceExecutor::LaneHold idle = executor->tryAcquireIdleLane(me);
        ASSERT_TRUE(idle.held());
        EXPECT_FALSE(idle.nested());
        EXPECT_EQ(executor->owner(), Owner::kCaller);
        EXPECT_TRUE(executor->heldByCurrent());
        EXPECT_FALSE(executor->tryAcquireIdleLane(me).held()) << "an idle-lane hold never nests";
        // A job queued while the hold lasts runs once the hold hands the lane over.
        std::atomic<bool> ran{false};
        const uint64_t index = submitFn(*executor, [&](FunctionJob&) { ran = true; });
        EXPECT_FALSE(ran.load());
        idle.release();
        ASSERT_TRUE(waitTerminal(*executor, index));
        EXPECT_TRUE(ran.load());
    }
    drain(*executor);

    executor->setHold(true);
    EXPECT_FALSE(executor->tryAcquireIdleLane(me).held()) << "a held worker is not idle";
    const uint64_t queued = submitFn(*executor, [](FunctionJob&) {});
    EXPECT_FALSE(executor->tryAcquireIdleLane(me).held()) << "queued work is not idle";
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, queued));

    Gate entered;
    Gate release;
    const uint64_t running = submitFn(*executor, [&](FunctionJob&) {
        entered.open();
        release.waitOpen();
    });
    ASSERT_TRUE(entered.waitOpen());
    EXPECT_FALSE(executor->tryAcquireIdleLane(me).held()) << "a running job is not idle";
    release.open();
    ASSERT_TRUE(waitTerminal(*executor, running));
    drain(*executor);
    EXPECT_TRUE(executor->tryAcquireIdleLane(me).held());
}

TEST_F(ExecutorTest, ExceptionsBecomeTaskFailures)
{
    const uint64_t thrown = submitFn(*executor, [](FunctionJob&) { throw std::runtime_error("boom"); });
    const uint64_t recorded = submitFn(*executor, [](FunctionJob& job) {
        job.failed = true;
        job.failure = "recorded failure";
    });
    const uint64_t unknown = submitFn(*executor, [](FunctionJob&) { throw 7; });
    ASSERT_TRUE(waitTerminal(*executor, unknown));
    std::unique_lock<std::mutex> lock = executor->lock();
    EXPECT_EQ(executor->taskState(thrown), TaskState::kFailed);
    EXPECT_EQ(executor->tasks().failures.at(thrown), "boom");
    EXPECT_EQ(executor->tasks().failures.at(recorded), "recorded failure");
    EXPECT_EQ(executor->taskState(unknown), TaskState::kFailed);
    EXPECT_FALSE(executor->tasks().failures.at(unknown).empty());
}

TEST_F(ExecutorTest, JobResourcesAreDestroyedAfterPublicationOutsideTheLock)
{
    std::atomic<int> observed{-1};
    std::shared_ptr<InstanceExecutor> self = executor;
    uint64_t index = 0;
    // Hold the worker until index is stored: the job's destructor reads it.
    executor->setHold(true);
    index = submitFn(
        *executor, [](FunctionJob&) {},
        [&observed, self, &index]() {
            // Taking the executor mutex here would deadlock if the worker held it.
            std::unique_lock<std::mutex> lock = self->lock();
            observed = static_cast<int>(self->taskState(index));
        });
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, index));
    settle(*executor);
    EXPECT_EQ(observed.load(), static_cast<int>(TaskState::kSucceeded));
}

// With nothing queued behind a job, its outcome and the free lane are published together:
// a caller that its wait woke finds the instance idle even while the job is destroyed.
TEST_F(ExecutorTest, LaneIsHandedBackWithTheOutcomeBeforeTheJobIsDestroyed)
{
    Gate destroying;
    Gate finishDestroy;
    std::atomic<bool> destroyed{false};
    const uint64_t index = submitFn(
        *executor, [](FunctionJob&) {},
        [&]() {
            destroying.open();
            finishDestroy.waitOpen();
            destroyed = true;
        });
    ASSERT_TRUE(waitTerminal(*executor, index));
    ASSERT_TRUE(destroying.waitOpen());
    EXPECT_TRUE(executor->laneIdle()) << "the waiter must find the lane free while the job is destroyed";
    {
        InstanceExecutor::LaneHold idle = executor->tryAcquireIdleLane(InstanceExecutor::currentIdentity());
        EXPECT_TRUE(idle.held());
        EXPECT_FALSE(destroyed.load());
    }
    finishDestroy.open();
    settle(*executor);
    EXPECT_TRUE(destroyed.load());
    EXPECT_EQ(stateOf(*executor, index), TaskState::kSucceeded);
}

// Teardown still waits for a finished job's destruction, which may run after the lane
// was handed back, before it takes the lane between jobs.
TEST_F(ExecutorTest, PreemptWaitsUntilTheFinishedJobIsDestroyed)
{
    Gate destroying;
    Gate finishDestroy;
    std::atomic<bool> destroyed{false};
    const uint64_t index = submitFn(
        *executor, [](FunctionJob&) {},
        [&]() {
            destroying.open();
            finishDestroy.waitOpen();
            destroyed = true;
        });
    ASSERT_TRUE(waitTerminal(*executor, index));
    ASSERT_TRUE(destroying.waitOpen());
    std::atomic<bool> preempted{false};
    std::atomic<bool> destroyedWhenPreempted{false};
    std::thread preempter([&]() {
        InstanceExecutor::LaneHold hold = executor->preemptLane();
        destroyedWhenPreempted = destroyed.load();
        preempted = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(preempted.load()) << "preemption must wait for the finished job's destruction";
    finishDestroy.open();
    preempter.join();
    EXPECT_TRUE(preempted.load());
    EXPECT_TRUE(destroyedWhenPreempted.load());
}

// A job queued behind the finished one keeps the lane with the worker: nothing overtakes it.
TEST_F(ExecutorTest, QueuedSuccessorKeepsTheLaneWhenAJobFinishes)
{
    Gate destroying;
    Gate finishDestroy;
    Gate secondEntered;
    Gate secondRelease;
    executor->setHold(true);
    const uint64_t first = submitFn(
        *executor, [](FunctionJob&) {},
        [&]() {
            destroying.open();
            finishDestroy.waitOpen();
        });
    const uint64_t second = submitFn(*executor, [&](FunctionJob&) {
        secondEntered.open();
        secondRelease.waitOpen();
    });
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, first));
    ASSERT_TRUE(destroying.waitOpen());
    EXPECT_FALSE(executor->tryAcquireIdleLane(InstanceExecutor::currentIdentity()).held());
    EXPECT_EQ(executor->owner(), Owner::kWorker);
    finishDestroy.open();
    ASSERT_TRUE(secondEntered.waitOpen());
    EXPECT_FALSE(executor->tryAcquireIdleLane(InstanceExecutor::currentIdentity()).held());
    secondRelease.open();
    ASSERT_TRUE(waitTerminal(*executor, second));
    drain(*executor);
}

TEST_F(ExecutorTest, PreemptTakesTheLaneBetweenJobsAndKeepsTheQueue)
{
    Gate entered;
    Gate release;
    std::atomic<int> ran{0};
    const uint64_t first = submitFn(*executor, [&](FunctionJob&) {
        entered.open();
        release.waitOpen();
        ++ran;
    });
    const uint64_t second = submitFn(*executor, [&](FunctionJob&) { ++ran; });
    const uint64_t third = submitFn(*executor, [&](FunctionJob&) { ++ran; });
    ASSERT_TRUE(entered.waitOpen());

    std::atomic<bool> preempted{false};
    InstanceExecutor::LaneHold hold;
    std::thread preempter([&]() {
        hold = executor->preemptLane();
        preempted = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(preempted.load()) << "preemption must wait for the running job";
    release.open();
    preempter.join();
    EXPECT_TRUE(preempted.load());
    EXPECT_EQ(executor->owner(), Owner::kCaller);
    EXPECT_EQ(stateOf(*executor, first), TaskState::kSucceeded);
    EXPECT_EQ(stateOf(*executor, second), TaskState::kPending);
    EXPECT_EQ(stateOf(*executor, third), TaskState::kPending);
    EXPECT_EQ(ran.load(), 1);

    // Releasing with the queue intact resumes the worker in order.
    hold.release();
    ASSERT_TRUE(waitTerminal(*executor, third));
    EXPECT_EQ(ran.load(), 3);
}

TEST_F(ExecutorTest, CancelAfterPreemptRetiresQueuedJobs)
{
    executor->setHold(true);
    std::atomic<int> ran{0};
    std::atomic<int> destroyed{0};
    const uint64_t first = submitFn(*executor, [&](FunctionJob&) { ++ran; }, [&]() { ++destroyed; });
    const uint64_t second = submitFn(*executor, [&](FunctionJob&) { ++ran; }, [&]() { ++destroyed; });
    InstanceExecutor::LaneHold hold = executor->preemptLane();
    executor->setHold(false);
    EXPECT_EQ(executor->cancelQueued("cancelled by teardown"), 2u);
    EXPECT_EQ(destroyed.load(), 2);
    EXPECT_EQ(ran.load(), 0);
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        EXPECT_EQ(executor->taskState(first), TaskState::kFailed);
        EXPECT_EQ(executor->tasks().failures.at(second), "cancelled by teardown");
    }
    executor->stopAndJoin();
    hold.release();
    EXPECT_EQ(executor->owner(), Owner::kFree);
    std::unique_ptr<InstanceExecutor::Job> late(new FunctionJob([](FunctionJob&) {}));
    InstanceExecutor::SubmitStatus status = InstanceExecutor::SubmitStatus::kAccepted;
    EXPECT_EQ(executor->submit(late, status), 0u);
    EXPECT_EQ(status, InstanceExecutor::SubmitStatus::kStopped);
    executor.reset();
}

// A thread that waits without a deadline runs the jobs up to its own on its own thread
// while the worker has not started them (here a test hold keeps the worker from starting
// any), in order; the lane then goes back to the worker, which runs the rest.
TEST_F(ExecutorTest, WaiterRunsUnstartedJobsUpToItsOwnAndHandsTheLaneBack)
{
    executor->setHold(true);
    std::vector<std::pair<int, std::thread::id>> ran;
    std::mutex ranMutex;
    auto record = [&](int id) {
        return [&ran, &ranMutex, id](FunctionJob&) {
            std::lock_guard<std::mutex> lock(ranMutex);
            ran.emplace_back(id, std::this_thread::get_id());
        };
    };
    const uint64_t first = submitFn(*executor, record(1));
    const uint64_t second = submitFn(*executor, record(2));
    const uint64_t third = submitFn(*executor, record(3));
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        executor->runUnstartedJobs(lock, second);
        EXPECT_EQ(executor->taskState(first), TaskState::kSucceeded);
        EXPECT_EQ(executor->taskState(second), TaskState::kSucceeded);
        EXPECT_EQ(executor->taskState(third), TaskState::kPending) << "a job behind the waited one stays queued";
    }
    EXPECT_EQ(executor->owner(), Owner::kWorker) << "the lane goes back to the worker with the queued job";
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, third));
    std::lock_guard<std::mutex> lock(ranMutex);
    ASSERT_EQ(ran.size(), 3u);
    EXPECT_EQ(ran[0], std::make_pair(1, std::this_thread::get_id()));
    EXPECT_EQ(ran[1], std::make_pair(2, std::this_thread::get_id()));
    EXPECT_EQ(ran[2].first, 3);
    EXPECT_NE(ran[2].second, std::this_thread::get_id()) << "the worker runs the job nobody waited for";
}

// With nothing queued behind them, the jobs a waiter ran leave the lane free: the next
// submission wakes the worker as usual.
TEST_F(ExecutorTest, WaiterThatEmptiedTheQueueLeavesTheLaneFree)
{
    executor->setHold(true);
    const uint64_t taken = submitFn(*executor, [](FunctionJob&) {});
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        executor->runUnstartedJobs(lock, taken);
        EXPECT_EQ(executor->taskState(taken), TaskState::kSucceeded);
    }
    executor->setHold(false);
    EXPECT_TRUE(executor->laneIdle());
    std::thread::id ranOn;
    const uint64_t next = submitFn(*executor, [&ranOn](FunctionJob&) { ranOn = std::this_thread::get_id(); });
    ASSERT_TRUE(waitTerminal(*executor, next));
    EXPECT_NE(ranOn, std::this_thread::get_id());
}

// While the worker runs a job, a waiting thread runs nothing: the worker goes on with the
// jobs queued behind it.
TEST_F(ExecutorTest, WaiterRunsNothingWhileTheWorkerRunsAJob)
{
    Gate entered;
    Gate release;
    const uint64_t running = submitFn(*executor, [&](FunctionJob&) {
        entered.open();
        release.waitOpen();
    });
    std::thread::id ranOn;
    const uint64_t queued = submitFn(*executor, [&ranOn](FunctionJob&) { ranOn = std::this_thread::get_id(); });
    ASSERT_TRUE(entered.waitOpen());
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        executor->runUnstartedJobs(lock, queued);
        EXPECT_EQ(executor->taskState(queued), TaskState::kPending);
    }
    release.open();
    ASSERT_TRUE(waitTerminal(*executor, queued));
    EXPECT_EQ(stateOf(*executor, running), TaskState::kSucceeded);
    EXPECT_NE(ranOn, std::this_thread::get_id());
}

TEST_F(ExecutorTest, HoldAfterNextSubmitLeavesTheLaneIdleUntilTheSubmission)
{
    executor->holdAfterNextSubmit();
    EXPECT_TRUE(executor->laneIdle()) << "the armed hold does not make the lane busy yet";
    {
        InstanceExecutor::LaneHold idle = executor->tryAcquireIdleLane(InstanceExecutor::currentIdentity());
        ASSERT_TRUE(idle.held());
        const uint64_t index = submitFn(*executor, [](FunctionJob&) {});
        idle.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        EXPECT_EQ(stateOf(*executor, index), TaskState::kPending) << "the submission held the worker";
        InstanceExecutor::LaneHold attempt = executor->tryAcquireIdleLane(InstanceExecutor::currentIdentity());
        EXPECT_FALSE(attempt.held());
        executor->setHold(false);
        ASSERT_TRUE(waitTerminal(*executor, index));
    }
    // One submission consumes it: the next job runs without a hold.
    const uint64_t next = submitFn(*executor, [](FunctionJob&) {});
    ASSERT_TRUE(waitTerminal(*executor, next));
}

TEST_F(ExecutorTest, StopRetiresJobsThatNeverStarted)
{
    executor->setHold(true);
    const uint64_t index = submitFn(*executor, [](FunctionJob&) {});
    executor->stopAndJoin();
    std::unique_lock<std::mutex> lock = executor->lock();
    EXPECT_EQ(executor->taskState(index), TaskState::kFailed);
    EXPECT_NE(executor->tasks().failures.at(index).find("cancelled"), std::string::npos);
}

TEST_F(ExecutorTest, CancelWhileAJobRunsKeepsTheWorkerLane)
{
    Gate entered;
    Gate release;
    const uint64_t running = submitFn(*executor, [&](FunctionJob&) {
        entered.open();
        release.waitOpen();
    });
    const uint64_t queued = submitFn(*executor, [](FunctionJob&) {});
    ASSERT_TRUE(entered.waitOpen());
    EXPECT_EQ(executor->cancelQueued("cancelled"), 1u);
    EXPECT_EQ(executor->owner(), Owner::kWorker) << "the running job still owns the lane";
    EXPECT_FALSE(executor->tryAcquireIdleLane(InstanceExecutor::currentIdentity()).held());
    release.open();
    ASSERT_TRUE(waitTerminal(*executor, running));
    EXPECT_EQ(stateOf(*executor, queued), TaskState::kFailed);
    drain(*executor);
    EXPECT_EQ(executor->owner(), Owner::kFree);
}

TEST_F(ExecutorTest, QueueLimitRejectsSubmission)
{
    executor->stopAndJoin();
    executor = InstanceExecutor::create(g_testThreads, 2);
    executor->setHold(true);
    submitFn(*executor, [](FunctionJob&) {});
    submitFn(*executor, [](FunctionJob&) {});
    std::unique_ptr<InstanceExecutor::Job> third(new FunctionJob([](FunctionJob&) {}));
    InstanceExecutor::SubmitStatus status = InstanceExecutor::SubmitStatus::kAccepted;
    EXPECT_EQ(executor->submit(third, status), 0u);
    EXPECT_EQ(status, InstanceExecutor::SubmitStatus::kQueueFull);
    EXPECT_TRUE(third != nullptr) << "a rejected job stays with the caller";
}

TEST_F(ExecutorTest, InlineTasksShareTheReceiptSequence)
{
    InstanceExecutor::LaneHold hold = executor->acquireLane();
    const uint64_t ok = executor->createInlineTask();
    const uint64_t bad = executor->createInlineTask();
    EXPECT_EQ(bad, ok + 1);
    executor->finishInlineTask(ok, nullptr);
    executor->finishInlineTask(bad, "inline failure");
    std::unique_lock<std::mutex> lock = executor->lock();
    EXPECT_EQ(executor->taskState(ok), TaskState::kSucceeded);
    EXPECT_EQ(executor->taskState(bad), TaskState::kFailed);
    EXPECT_EQ(executor->taskState(bad + 1), TaskState::kNotFound);
    EXPECT_EQ(executor->taskState(0), TaskState::kNotFound);
}

TEST_F(ExecutorTest, CallingThreadReceiptsReuseOneSlotAndPublishOnlyFailures)
{
    InstanceExecutor::LaneHold hold = executor->acquireLane();
    InstanceExecutor::FailureSlot slot;
    const uint64_t ok = executor->beginInlineTask(slot);
    ASSERT_TRUE(slot);
    const void* storage = &slot.mapped();
    executor->endInlineTask(ok, slot, nullptr);
    EXPECT_FALSE(slot) << "a success caches its slot for the next receipt";

    InstanceExecutor::FailureSlot next;
    const uint64_t bad = executor->beginInlineTask(next);
    EXPECT_EQ(bad, ok + 1);
    ASSERT_TRUE(next);
    EXPECT_EQ(&next.mapped(), storage) << "the cached slot is reused, not allocated again";
    // A nested receipt while the cached slot is in use gets a slot of its own.
    InstanceExecutor::FailureSlot nested;
    const uint64_t inner = executor->beginInlineTask(nested);
    EXPECT_EQ(inner, bad + 1);
    ASSERT_TRUE(nested);
    executor->endInlineTask(inner, nested, "inner failure");
    executor->endInlineTask(bad, next, "outer failure");
    EXPECT_FALSE(next);
    executor->endInlineTask(bad, next, nullptr);

    // A later receipt from the submission path continues the same sequence.
    const uint64_t job = submitFn(*executor, [](FunctionJob&) {});
    EXPECT_EQ(job, inner + 1);
    hold.release();
    ASSERT_TRUE(waitTerminal(*executor, job));

    std::unique_lock<std::mutex> lock = executor->lock();
    EXPECT_EQ(executor->taskState(ok), TaskState::kSucceeded);
    EXPECT_EQ(executor->taskState(bad), TaskState::kFailed) << "a second outcome never overrides the first";
    EXPECT_EQ(executor->tasks().failures.at(bad), "outer failure");
    EXPECT_EQ(executor->tasks().failures.at(inner), "inner failure");
    EXPECT_EQ(executor->tasks().failures.count(ok), 0u);
    EXPECT_EQ(executor->taskState(job), TaskState::kSucceeded);
}

TEST_F(ExecutorTest, LaneIsIdleOnlyWithoutHolderJobsOrHold)
{
    EXPECT_TRUE(executor->laneIdle());
    {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        EXPECT_FALSE(executor->laneIdle()) << "a held lane is not idle";
    }
    EXPECT_TRUE(executor->laneIdle());
    executor->setHold(true);
    EXPECT_FALSE(executor->laneIdle()) << "a held worker is not idle";
    const uint64_t index = submitFn(*executor, [](FunctionJob&) {});
    EXPECT_FALSE(executor->laneIdle()) << "a queued job keeps the lane busy";
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, index));
    drain(*executor);
    EXPECT_TRUE(executor->laneIdle());
}

TEST(ExecutorTaskTable, CollectedRangesCompactIntoThePrefix)
{
    InstanceExecutor::TaskTable table;
    table.nextIndex = 20;
    table.collectRange(5, 6);
    EXPECT_TRUE(table.isCollected(5));
    EXPECT_FALSE(table.isCollected(4));
    table.collectRange(1, 4);
    EXPECT_EQ(table.collectedPrefix, 6u);
    EXPECT_TRUE(table.collectedRanges.empty());
    table.collectRange(9, 9);
    table.collectRange(8, 8);
    EXPECT_FALSE(table.isCollected(7));
    EXPECT_TRUE(table.isCollected(8));
    table.collectRange(7, 7);
    EXPECT_EQ(table.collectedPrefix, 9u);
}

TEST(ExecutorStress, NoLostWakeUpsBetweenCallerAndWorker)
{
    std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::create(g_testThreads);
    std::atomic<uint64_t> executed{0};
    uint64_t submitted = 0;
    uint64_t last = 0;
    std::atomic<bool> stop{false};
    // A process-wide caller keeps taking and releasing the lane concurrently.
    std::thread external([&]() {
        while (!stop.load())
        {
            InstanceExecutor::LaneHold hold = executor->acquireLane();
            std::this_thread::yield();
        }
    });
    for (int round = 0; round < 3000; ++round)
    {
        if (round % 3 == 0)
        {
            InstanceExecutor::LaneHold hold = executor->acquireLane();
            EXPECT_EQ(executed.load(), submitted) << "a caller must observe every earlier job";
        }
        last = submitFn(*executor, [&](FunctionJob&) { ++executed; });
        ++submitted;
    }
    {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        EXPECT_EQ(executed.load(), submitted);
    }
    stop = true;
    external.join();
    EXPECT_EQ(stateOf(*executor, last), TaskState::kSucceeded);
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        EXPECT_EQ(executor->firstUnfinished(UINT64_MAX), 0u);
    }
    EXPECT_EQ(executor->owner(), Owner::kFree);
    executor->stopAndJoin();
}

// Waits that run unstarted jobs race the worker's wake-up for every job, while another
// caller keeps taking the lane: whichever thread runs a job, every job runs once and in
// submission order, and the jobs nobody waits for still run on the worker.
TEST(ExecutorStress, WaitersRunningJobsLoseNoWakeUps)
{
    std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::create(g_testThreads);
    std::atomic<uint64_t> executed{0};
    std::atomic<bool> outOfOrder{false};
    std::atomic<bool> stop{false};
    std::thread external([&]() {
        while (!stop.load())
        {
            InstanceExecutor::LaneHold hold = executor->acquireLane();
            std::this_thread::yield();
        }
    });
    uint64_t submitted = 0;
    for (int round = 0; round < 3000; ++round)
    {
        const uint64_t earlier = submitted;
        const uint64_t index = submitFn(*executor, [&executed, &outOfOrder, earlier](FunctionJob&) {
            if (executed.fetch_add(1) != earlier)
                outOfOrder = true;
        });
        ++submitted;
        // Most rounds wait at once, as a step waited for at once does; the others leave
        // their job to the worker or to a later wait.
        if (round % 4 == 3)
            continue;
        std::unique_lock<std::mutex> lock = executor->lock();
        executor->runUnstartedJobs(lock, index);
        executor->waitLocked(
            lock, [&]() { return !executor->isUnfinished(index); }, WaitDeadline::infinite());
    }
    {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        EXPECT_EQ(executed.load(), submitted);
    }
    stop = true;
    external.join();
    EXPECT_FALSE(outOfOrder.load()) << "a job ran before an earlier one";
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        EXPECT_EQ(executor->firstUnfinished(UINT64_MAX), 0u);
    }
    EXPECT_EQ(executor->owner(), Owner::kFree);
    executor->stopAndJoin();
}

// The caller's paths without the mutex: calling-thread receipts and, while the lane is
// idle, reading and collecting the task table, interleaved with worker jobs that publish
// under the mutex. Under TSan this checks that every worker publication happens before
// the lane is seen idle and that the next submission orders the caller's writes before
// the worker's.
TEST(ExecutorStress, IdleCallerTableAccessNeverRacesTheWorker)
{
    std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::create(g_testThreads);
    uint64_t collected = 0;
    size_t failuresSeen = 0;
    for (int round = 0; round < 2000; ++round)
    {
        const bool fail = round % 5 == 0;
        submitFn(*executor, [fail](FunctionJob& job) {
            if (fail)
            {
                job.failed = true;
                job.failure = "job failure";
            }
        });
        if (round % 3 == 0)
        {
            InstanceExecutor::LaneHold hold = executor->acquireLane();
            InstanceExecutor::FailureSlot slot;
            const uint64_t index = executor->beginInlineTask(slot);
            executor->endInlineTask(index, slot, round % 2 ? "inline failure" : nullptr);
        }
        if (executor->laneIdle())
        {
            InstanceExecutor::TaskTable& table = executor->tasks();
            EXPECT_EQ(executor->firstUnfinished(UINT64_MAX), 0u) << "an idle lane leaves no unfinished job";
            const uint64_t cutoff = table.nextIndex.load(std::memory_order_relaxed) - 1;
            failuresSeen += table.failures.size();
            table.failures.clear();
            table.collectRange(table.collectedPrefix + 1, cutoff);
            collected = cutoff;
        }
    }
    {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
    }
    EXPECT_TRUE(executor->laneIdle());
    EXPECT_GT(collected, 0u) << "the lane was seen idle at least once";
    EXPECT_GT(failuresSeen, 0u);
    executor->stopAndJoin();
}

// Fake Carbonite: the waiting test thread pretends to be a task. Waits suspend on a
// global condition variable keyed by the executor's epoch word.
std::mutex g_fakeTaskMutex;
std::condition_variable g_fakeTaskCv;
thread_local bool t_fakeTask = false;
std::atomic<int> g_fakeWaits{0};
std::atomic<int> g_fakePins{0};
std::atomic<int> g_fakeUnpins{0};

uint64_t fakeCurrentTask()
{
    return t_fakeTask ? 42u : 0u;
}

void fakeWait(const std::atomic<uint32_t>* word, uint32_t expected, uint64_t timeoutNs)
{
    ++g_fakeWaits;
    std::unique_lock<std::mutex> lock(g_fakeTaskMutex);
    const std::chrono::nanoseconds limit(
        timeoutNs == UINT64_MAX ? std::chrono::nanoseconds(std::chrono::hours(1))
                                : std::chrono::nanoseconds(static_cast<int64_t>(timeoutNs)));
    g_fakeTaskCv.wait_for(lock, limit, [&]() { return word->load() != expected; });
}

void fakeWakeAll(const std::atomic<uint32_t>*)
{
    std::lock_guard<std::mutex> lock(g_fakeTaskMutex);
    g_fakeTaskCv.notify_all();
}

bool fakePin()
{
    ++g_fakePins;
    return true;
}

void fakeUnpin()
{
    ++g_fakeUnpins;
}

TEST_F(ExecutorTest, TaskAwareWaitsSuspendInsteadOfBlocking)
{
    static const ovphysx::ExecutorWaitHooks hooks{fakeCurrentTask, fakeWait, fakeWakeAll, fakePin, fakeUnpin};
    ovphysx::setExecutorWaitHooks(&hooks);
    executor->setHold(true);
    const uint64_t index = submitFn(*executor, [](FunctionJob&) {});
    std::atomic<bool> laneTaken{false};
    std::thread task([&]() {
        t_fakeTask = true;
        {
            std::unique_lock<std::mutex> lock = executor->lock();
            EXPECT_TRUE(executor->waitLocked(
                lock, [&]() { return !executor->isUnfinished(index); }, WaitDeadline::infinite()));
        }
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        laneTaken = hold.held();
        t_fakeTask = false;
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (executor->waitingTaskCount() != 1 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_EQ(executor->waitingTaskCount(), 1u);
    EXPECT_EQ(executor->waitingThreadCount(), 0u);
    executor->setHold(false);
    task.join();
    EXPECT_TRUE(laneTaken.load());
    EXPECT_GT(g_fakeWaits.load(), 0);
    EXPECT_EQ(g_fakePins.load(), g_fakeUnpins.load());
    ovphysx::setExecutorWaitHooks(nullptr);
}

TEST_F(ExecutorTest, StopAndJoinWaitsForTheWorkerAsATask)
{
    static const ovphysx::ExecutorWaitHooks hooks{fakeCurrentTask, fakeWait, fakeWakeAll, fakePin, fakeUnpin};
    ovphysx::setExecutorWaitHooks(&hooks);
    Gate entered;
    Gate release;
    const uint64_t running = submitFn(*executor, [&](FunctionJob&) {
        entered.open();
        release.waitOpen();
    });
    ASSERT_TRUE(entered.waitOpen());
    std::atomic<bool> stopped{false};
    std::thread task([&]() {
        t_fakeTask = true;
        executor->stopAndJoin();
        stopped = true;
        t_fakeTask = false;
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (executor->waitingTaskCount() != 1 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_EQ(executor->waitingTaskCount(), 1u) << "stopping waits for the running job as a suspended task";
    EXPECT_EQ(executor->waitingThreadCount(), 0u);
    EXPECT_FALSE(stopped.load());
    EXPECT_FALSE(executor->workerExited());
    release.open();
    task.join();
    EXPECT_TRUE(stopped.load());
    EXPECT_TRUE(executor->workerExited());
    EXPECT_EQ(stateOf(*executor, running), TaskState::kSucceeded);
    ovphysx::setExecutorWaitHooks(nullptr);
}

// A waiting Carbonite task runs nothing: it is suspended while the worker runs the job.
TEST_F(ExecutorTest, WaitingTaskRunsNoJob)
{
    static const ovphysx::ExecutorWaitHooks hooks{fakeCurrentTask, fakeWait, fakeWakeAll, fakePin, fakeUnpin};
    ovphysx::setExecutorWaitHooks(&hooks);
    executor->setHold(true);
    const uint64_t index = submitFn(*executor, [](FunctionJob&) {});
    t_fakeTask = true;
    {
        std::unique_lock<std::mutex> lock = executor->lock();
        executor->runUnstartedJobs(lock, index);
        EXPECT_EQ(executor->taskState(index), TaskState::kPending);
    }
    t_fakeTask = false;
    ovphysx::setExecutorWaitHooks(nullptr);
    executor->setHold(false);
    ASSERT_TRUE(waitTerminal(*executor, index));
}

// Without a worker thread the submitting thread runs its jobs, inside a Carbonite task
// too; a job submitted while another caller holds the lane runs once that caller hands the
// lane over. Stopping joins nothing.
TEST(ExecutorWithoutWorker, SubmittingThreadRunsItsJobs)
{
    std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::createWithoutWorker();
    EXPECT_FALSE(executor->hasWorker());
    std::thread::id ranOn;
    const uint64_t first = submitFn(*executor, [&ranOn](FunctionJob&) { ranOn = std::this_thread::get_id(); });
    EXPECT_EQ(stateOf(*executor, first), TaskState::kPending);
    executor->runSubmittedJobs(first);
    EXPECT_EQ(stateOf(*executor, first), TaskState::kSucceeded);
    EXPECT_EQ(ranOn, std::this_thread::get_id());
    EXPECT_TRUE(executor->laneIdle());

    Gate held;
    Gate release;
    std::thread holder([&]() {
        InstanceExecutor::LaneHold hold = executor->acquireLane();
        held.open();
        release.waitOpen();
    });
    ASSERT_TRUE(held.waitOpen());
    std::atomic<bool> ran{false};
    std::atomic<bool> ranWhileHeld{false};
    const uint64_t second = submitFn(*executor, [&ran](FunctionJob&) { ran = true; });
    std::thread releaser([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ranWhileHeld = ran.load();
        release.open();
    });
    executor->runSubmittedJobs(second);
    releaser.join();
    holder.join();
    EXPECT_FALSE(ranWhileHeld.load());
    EXPECT_TRUE(ran.load());
    EXPECT_TRUE(executor->laneIdle());

    static const ovphysx::ExecutorWaitHooks hooks{fakeCurrentTask, fakeWait, fakeWakeAll, fakePin, fakeUnpin};
    ovphysx::setExecutorWaitHooks(&hooks);
    t_fakeTask = true;
    const uint64_t third = submitFn(*executor, [](FunctionJob&) {});
    executor->runSubmittedJobs(third);
    t_fakeTask = false;
    ovphysx::setExecutorWaitHooks(nullptr);
    EXPECT_EQ(stateOf(*executor, third), TaskState::kSucceeded);

    executor->stopAndJoin();
    EXPECT_TRUE(executor->workerExited());
}

TEST(ExecutorLifetime, AbandonedExecutorKeepsItsWorker)
{
    // Process exit drops executors without stopAndJoin(); the destructor must not end
    // the pool, and the worker keeps its own reference.
    std::weak_ptr<InstanceExecutor> weak;
    {
        std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::create(g_testThreads);
        weak = executor;
    }
    EXPECT_FALSE(weak.expired()) << "the parked worker keeps its executor alive";
    // Stop it explicitly so the test process does not keep a parked thread.
    std::shared_ptr<InstanceExecutor> revived = weak.lock();
    ASSERT_TRUE(revived);
    revived->stopAndJoin();
}

// A pool thread cannot end itself: a stop on the worker only stops its loop, and a
// later stop from another thread ends the pool.
TEST(ExecutorLifetime, StoppingFromTheWorkerItselfLeavesThePoolToALaterStop)
{
    g_poolCalls.reset();
    std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::create(g_testThreads);
    std::shared_ptr<InstanceExecutor> self = executor;
    const uint64_t index = submitFn(*executor, [self](FunctionJob&) { self->stopAndJoin(); });
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_TRUE(executor->stopped());
    EXPECT_EQ(g_poolCalls.destroyed.load(), 0);
    executor->stopAndJoin();
    EXPECT_TRUE(executor->workerExited());
    EXPECT_EQ(g_poolCalls.destroyed.load(), 1);
}

// The worker loop is the one job of a one-thread pool from the IThreadPool given to
// create(): jobs run on that pool's thread, which the loop names, and stopping destroys
// the pool, once.
TEST(ExecutorThreads, WorkerIsTheOneJobOfAOneThreadPool)
{
    g_poolCalls.reset();
    std::shared_ptr<InstanceExecutor> executor = InstanceExecutor::create(g_testThreads);
    EXPECT_TRUE(executor->hasWorker());
    EXPECT_EQ(g_poolCalls.created.load(), 1);
    EXPECT_EQ(g_poolCalls.workers.load(), 1u);
    EXPECT_EQ(g_poolCalls.enqueued.load(), 1);
    std::thread::id ranOn;
    std::string name;
    const uint64_t index = submitFn(*executor, [&](FunctionJob&) {
        ranOn = std::this_thread::get_id();
#if defined(__linux__)
        char buffer[16] = {};
        pthread_getname_np(pthread_self(), buffer, sizeof(buffer));
        name = buffer;
#endif
    });
    ASSERT_TRUE(waitTerminal(*executor, index));
    EXPECT_EQ(ranOn, g_poolCalls.lastThread);
#if defined(__linux__)
    EXPECT_EQ(name, "ovphysx worker");
#endif
    drain(*executor);
    executor->stopAndJoin();
    EXPECT_EQ(g_poolCalls.destroyed.load(), 1);
    executor->stopAndJoin();
    EXPECT_EQ(g_poolCalls.destroyed.load(), 1) << "the pool is destroyed once";
}

// create() throws when the pool cannot be created or its job cannot start; a pool whose
// job did not start is destroyed at once.
TEST(ExecutorThreads, CreateFailsWithoutAPoolThread)
{
    g_poolCalls.reset();
    g_poolCalls.failCreate = true;
    EXPECT_THROW(InstanceExecutor::create(g_testThreads), std::runtime_error);
    g_poolCalls.reset();
    g_poolCalls.failEnqueue = true;
    EXPECT_THROW(InstanceExecutor::create(g_testThreads), std::runtime_error);
    EXPECT_EQ(g_poolCalls.created.load(), 1);
    EXPECT_EQ(g_poolCalls.destroyed.load(), 1);
    g_poolCalls.reset();
}

TEST(ExecutorDeadline, TimeoutConversion)
{
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    EXPECT_EQ(WaitDeadline::fromTimeoutNs(0, start).kind, WaitDeadline::Kind::kPoll);
    EXPECT_EQ(WaitDeadline::fromTimeoutNs(UINT64_MAX, start).kind, WaitDeadline::Kind::kInfinite);
    EXPECT_EQ(WaitDeadline::fromTimeoutNs(UINT64_MAX - 1, start).kind, WaitDeadline::Kind::kInfinite);
    const WaitDeadline finite = WaitDeadline::fromTimeoutNs(1000000, start);
    EXPECT_EQ(finite.kind, WaitDeadline::Kind::kFinite);
    EXPECT_EQ(finite.at - start, std::chrono::milliseconds(1));
}

} // namespace
