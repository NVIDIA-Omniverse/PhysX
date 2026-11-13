// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-TASK-001
 * @covers AC-3 AC-5 AC-9 AC-10
 *
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-1 AC-6 AC-8
 *
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-5
 *
 * TEST-CAPI-TASK-005 and TEST-CAPI-EXECUTOR-001: worker jobs through the public task contract.
 */

// Worker jobs through the public task contract: a held worker leaves its receipt
// pending, finite waits and polls observe completion without running work, a wait
// without a deadline runs the jobs the worker has not started yet, the status query never
// collects, and waits made from Carbonite tasks release their worker threads.

#include "gated_simulation.h"
#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "test_utilities.h"

#include <chrono>
#include <string>
#include <thread>

namespace
{

class WorkerHold
{
public:
    explicit WorkerHold(ovphysx_instance_handle_t handle) : m_handle(handle)
    {
        held = ovphysx_set_worker_hold_for_test_internal(m_handle, true);
    }
    ~WorkerHold() { release(); }
    void release()
    {
        if (held)
            ovphysx_set_worker_hold_for_test_internal(m_handle, false);
        held = false;
    }
    bool held = false;

private:
    ovphysx_instance_handle_t m_handle;
};

ovphysx_task_status_t statusOf(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_NOT_FOUND;
    EXPECT_EQ(ovphysx_query_task_status(handle, task, &status).status, OVPHYSX_API_SUCCESS);
    return status;
}

// Waits, without collecting, until task has finished (or 30 s passed) and returns its state.
ovphysx_task_status_t statusWhenFinished(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    ovphysx_task_status_t status = statusOf(handle, task);
    while ((status == OVPHYSX_TASK_STATUS_PENDING || status == OVPHYSX_TASK_STATUS_RUNNING) &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        status = statusOf(handle, task);
    }
    return status;
}

ovphysx_api_status_t waitStatus(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
                                ovphysx_timeout_t timeout, ovphysx_wait_task_result_t* kept = nullptr)
{
    ovphysx_wait_task_result_t result{};
    const ovphysx_api_status_t status = ovphysx_wait_task(handle, task, timeout, &result).status;
    if (kept)
        *kept = result;
    else
        ovphysx_destroy_wait_task_result(&result);
    return status;
}

class ExecutorTasks : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        // The CPU instance is shared across suites; start from an empty task table so
        // aggregate waits below only see this test's tasks.
        ovphysx_wait_task_result_t residual{};
        ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &residual);
        ovphysx_destroy_wait_task_result(&residual);
    }
};

using WorkerWaits = ExecutorTasks;

ovphysx_enqueue_result_t submitRecorded(ovphysx_instance_handle_t handle, RanOn& where)
{
    return ovphysx_submit_test_callback_job_for_test_internal(handle, &RanOn::record, &where);
}

TEST_F(ExecutorTasks, HeldJobReturnsPendingThenSucceeds)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t task = ovphysx_submit_test_job_for_test_internal(m_handle, nullptr, 0);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(task.op_index, 0u);
    EXPECT_EQ(task.instance, m_handle);
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_PENDING);

    ovphysx_wait_task_result_t polled{};
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_POLL, &polled), OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(polled.pending_task.op_index, task.op_index);
    EXPECT_EQ(polled.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&polled);

    const auto before = std::chrono::steady_clock::now();
    EXPECT_EQ(waitStatus(m_handle, task, 20'000'000ULL), OVPHYSX_API_TIMEOUT);
    EXPECT_GE(std::chrono::steady_clock::now() - before, std::chrono::milliseconds(15));
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_PENDING) << "a timed-out wait must not collect";

    hold.release();
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_NOT_FOUND);
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_NOT_FOUND);
}

TEST_F(ExecutorTasks, RunningJobReportsRunningAndFiniteWaitSucceeds)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_enqueue_result_t task = ovphysx_submit_test_job_for_test_internal(m_handle, nullptr, 300);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    bool sawRunning = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!sawRunning && std::chrono::steady_clock::now() < deadline)
    {
        const ovphysx_task_status_t status = statusOf(m_handle, task);
        sawRunning = status == OVPHYSX_TASK_STATUS_RUNNING;
        if (status == OVPHYSX_TASK_STATUS_SUCCEEDED)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(sawRunning);
    EXPECT_EQ(waitStatus(m_handle, task, 30'000'000'000ULL), OVPHYSX_API_SUCCESS);
}

TEST_F(ExecutorTasks, FailedJobIsReportedOnceWithApiAndIndex)
{
    const ovphysx_enqueue_result_t task = ovphysx_submit_test_job_for_test_internal(m_handle, "planted failure", 0);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (statusOf(m_handle, task) != OVPHYSX_TASK_STATUS_FAILED && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_FAILED);

    ovphysx_wait_task_result_t result{};
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_INFINITE, &result), OVPHYSX_API_ERROR);
    ASSERT_EQ(result.num_errors, 1u);
    EXPECT_EQ(result.error_tasks[0].op_index, task.op_index);
    const ovphysx_string_t message = ovphysx_get_last_task_error(m_handle, task);
    const std::string expected = "test_job (task " + std::to_string(task.op_index) + "): planted failure";
    EXPECT_EQ(std::string(message.ptr, message.length), expected);
    ovphysx_destroy_wait_task_result(&result);
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_NOT_FOUND);
}

TEST_F(ExecutorTasks, WaitAllStopsAtTheEarliestUnfinishedJob)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    // An absent release is a no-op task, finished when the call returns on the idle
    // instance (under the always-queue test switch, once the worker ran it).
    const ovphysx_enqueue_result_t inlineTask = ovphysx_release_query(m_handle, 0);
    ASSERT_EQ(inlineTask.status, OVPHYSX_API_SUCCESS);
    if (!ovphysxIdleCallsRunOnCaller())
        statusWhenFinished(m_handle, inlineTask);
    EXPECT_EQ(statusOf(m_handle, inlineTask), OVPHYSX_TASK_STATUS_SUCCEEDED);

    WorkerHold hold(m_handle);
    const ovphysx_enqueue_result_t first = ovphysx_submit_test_job_for_test_internal(m_handle, nullptr, 0);
    const ovphysx_enqueue_result_t second = ovphysx_submit_test_job_for_test_internal(m_handle, "second fails", 0);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(second.status, OVPHYSX_API_SUCCESS);

    ovphysx_wait_task_result_t polled{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_POLL, &polled).status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(polled.pending_task.op_index, first.op_index);
    EXPECT_EQ(polled.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&polled);
    EXPECT_EQ(statusOf(m_handle, inlineTask), OVPHYSX_TASK_STATUS_NOT_FOUND) << "work before the cutoff is collected";
    EXPECT_EQ(statusOf(m_handle, second), OVPHYSX_TASK_STATUS_PENDING);

    hold.release();
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &all).status, OVPHYSX_API_ERROR);
    ASSERT_EQ(all.num_errors, 1u);
    EXPECT_EQ(all.error_tasks[0].op_index, second.op_index);
    ovphysx_destroy_wait_task_result(&all);
    EXPECT_EQ(statusOf(m_handle, first), OVPHYSX_TASK_STATUS_NOT_FOUND);
}

TEST_F(ExecutorTasks, StatusQueryValidatesArguments)
{
    const ovphysx_enqueue_result_t task = ovphysx_release_query(m_handle, 0);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    if (!ovphysxIdleCallsRunOnCaller())
        statusWhenFinished(m_handle, task);
    EXPECT_EQ(ovphysx_query_task_status(m_handle, task, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_SUCCEEDED;
    EXPECT_EQ(ovphysx_query_task_status(m_handle, ovphysx_enqueue_result_t{}, &status).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(status, OVPHYSX_TASK_STATUS_NOT_FOUND);
    ovphysx_enqueue_result_t foreign = task;
    foreign.instance = m_handle + 1000;
    EXPECT_EQ(ovphysx_query_task_status(m_handle, foreign, &status).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_query_task_status(OVPHYSX_INVALID_HANDLE, task, &status).status, OVPHYSX_API_NOT_FOUND);
    ovphysx_enqueue_result_t neverIssued = task;
    neverIssued.op_index = task.op_index + 100000;
    status = OVPHYSX_TASK_STATUS_SUCCEEDED;
    EXPECT_EQ(ovphysx_query_task_status(m_handle, neverIssued, &status).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(status, OVPHYSX_TASK_STATUS_NOT_FOUND);

    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_SUCCEEDED) << "status queries never collect";
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_NOT_FOUND);
}

TEST_F(ExecutorTasks, StepJobReportsPendingWhileQueuedAndRunningWhileItRuns)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    physx::PxScene* const scene = static_cast<physx::PxScene*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene"));
    physx::PxRigidDynamic* const actor = static_cast<physx::PxRigidDynamic*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(scene, nullptr);
    ASSERT_NE(actor, nullptr);
    // The gated step blocks inside its simulation until the gate opens.
    test_utils::GatedSimulationScope gate(m_handle, scene, actor);

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_PENDING) << "a queued step has not started";

    gate.set_pending_task(step);
    gate.start_watchdog();
    hold.release();
    ASSERT_TRUE(gate.wait_until_entered()) << "the step job never reached its simulation";
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_RUNNING) << "the worker runs the step";
    EXPECT_FALSE(gate.open_and_join_watchdog()) << "the watchdog had to open the gate";

    // Opening the gate lets the step finish; the scope's wait collects it.
    EXPECT_EQ(gate.finish(), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_NOT_FOUND);
}

TEST_F(ExecutorTasks, WaitsInsideCarboniteTasksReleaseWorkerThreads)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    uint32_t waiters = 0;
    EXPECT_TRUE(ovphysx_wait_from_saturated_carbonite_pool_for_test_internal(m_handle, &waiters));
    EXPECT_GT(waiters, 2u);
}

// A wait without a deadline runs the job it waits for on the waiting thread when the worker
// has not started it. The test hold keeps the worker from starting any job, which makes
// that deterministic; it does not hold a waiting thread.
TEST_F(WorkerWaits, InfiniteWaitRunsAJobTheWorkerHasNotStartedOnTheWaitingThread)
{
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    RanOn job;
    const ovphysx_enqueue_result_t task = submitRecorded(m_handle, job);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(job.onThisThread()) << "the waiting thread runs the job";

    // wait_all runs every job up to its cutoff, in order, the same way.
    RanOn first;
    RanOn second;
    ASSERT_EQ(submitRecorded(m_handle, first).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t failing = ovphysx_submit_test_job_for_test_internal(m_handle, "planted", 0);
    ASSERT_EQ(submitRecorded(m_handle, second).status, OVPHYSX_API_SUCCESS);
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &all).status, OVPHYSX_API_ERROR);
    ASSERT_EQ(all.num_errors, 1u);
    EXPECT_EQ(all.error_tasks[0].op_index, failing.op_index) << "a job run by the waiter publishes its failure";
    ovphysx_destroy_wait_task_result(&all);
    EXPECT_TRUE(first.onThisThread());
    EXPECT_TRUE(second.onThisThread());
}

// A job nobody waits for runs on the worker.
TEST_F(WorkerWaits, JobNobodyWaitsForRunsOnTheWorker)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    RanOn job;
    const ovphysx_enqueue_result_t task = submitRecorded(m_handle, job);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusWhenFinished(m_handle, task), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_TRUE(job.ran.load());
    EXPECT_NE(job.thread, std::this_thread::get_id());
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_SUCCESS);
}

// Polls and finite waits never run a job, not even one the worker has not started.
TEST_F(WorkerWaits, FiniteWaitsAndPollsNeverRunJobs)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    RanOn job;
    const ovphysx_enqueue_result_t task = submitRecorded(m_handle, job);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(waitStatus(m_handle, task, 20'000'000ULL), OVPHYSX_API_TIMEOUT);
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_POLL, &all).status, OVPHYSX_API_TIMEOUT);
    ovphysx_destroy_wait_task_result(&all);
    EXPECT_EQ(ovphysx_wait_all(m_handle, 20'000'000ULL, &all).status, OVPHYSX_API_TIMEOUT);
    ovphysx_destroy_wait_task_result(&all);
    EXPECT_FALSE(job.ran.load());
    EXPECT_EQ(statusOf(m_handle, task), OVPHYSX_TASK_STATUS_PENDING);

    hold.release();
    EXPECT_EQ(statusWhenFinished(m_handle, task), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_NE(job.thread, std::this_thread::get_id()) << "the worker ran it";
    EXPECT_EQ(waitStatus(m_handle, task, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_SUCCESS);
}

// A wait runs only the jobs up to its own; the lane then goes back to the worker, which
// runs the job behind them once it may, and runs later jobs as before.
TEST_F(WorkerWaits, WaitRunsJobsUpToItsOwnAndTheWorkerRunsTheRest)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    RanOn first;
    RanOn waited;
    RanOn behind;
    const ovphysx_enqueue_result_t firstTask = submitRecorded(m_handle, first);
    const ovphysx_enqueue_result_t waitedTask = submitRecorded(m_handle, waited);
    const ovphysx_enqueue_result_t behindTask = submitRecorded(m_handle, behind);
    EXPECT_EQ(waitStatus(m_handle, waitedTask, OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(first.onThisThread());
    EXPECT_TRUE(waited.onThisThread());
    EXPECT_FALSE(behind.ran.load()) << "a job behind the waited one stays queued";
    EXPECT_EQ(statusOf(m_handle, firstTask), OVPHYSX_TASK_STATUS_SUCCEEDED) << "a wait collects only its task";
    EXPECT_EQ(statusOf(m_handle, behindTask), OVPHYSX_TASK_STATUS_PENDING);

    hold.release();
    EXPECT_EQ(statusWhenFinished(m_handle, behindTask), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_NE(behind.thread, std::this_thread::get_id());
    RanOn later;
    const ovphysx_enqueue_result_t laterTask = submitRecorded(m_handle, later);
    EXPECT_EQ(statusWhenFinished(m_handle, laterTask), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_NE(later.thread, std::this_thread::get_id());
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_POLL, &all).status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&all);
}

TEST(ExecutorTasksLifecycle, DestroyCancelsQueuedJobsAndJoinsTheWorker)
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ASSERT_EQ(ovphysx_create_instance_with_args(&args, &handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(handle, true));
    const ovphysx_enqueue_result_t first = ovphysx_submit_test_job_for_test_internal(handle, nullptr, 0);
    const ovphysx_enqueue_result_t second = ovphysx_submit_test_job_for_test_internal(handle, nullptr, 0);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(second.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_PENDING;
    EXPECT_EQ(ovphysx_query_task_status(handle, first, &status).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(status, OVPHYSX_TASK_STATUS_NOT_FOUND);
    EXPECT_EQ(waitStatus(handle, second, OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(ovphysx_set_worker_hold_for_test_internal(handle, false));
}

} // namespace
