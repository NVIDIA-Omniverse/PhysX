// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-5 AC-6
 *
 * @implements REQ-CAPI-TASK-001
 * @covers AC-3 AC-6 AC-9 AC-10
 *
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-CONTACT-003
 * @covers AC-3
 *
 * TEST-CAPI-ASYNC-003: ovphysx_step queues one job on the instance worker, which runs
 * simulate and fetchResults, so a step completes without any wait.
 */

// The step as a worker job: it returns at once and stays pending while the worker is
// held, completes with no wait, keeps submission order and the step state of a chain,
// is collected exactly once by polling or after a synchronous drain, and a callback
// that the step job runs on the worker cannot wait for its own instance. The call runs
// no part of the step, also on an idle instance.

#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "session_test_helpers.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>

#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <task/PxCpuDispatcher.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

namespace
{

constexpr const char* kBoxes = "tests/data/boxes_falling_on_groundplane.usda";
constexpr float kDt = 1.0f / 60.0f;
constexpr double kGravity = 9.81;

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

// Waits for task and returns the wait status; the result is destroyed unless kept.
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

// One component of Cube1's attribute, read through a session on the calling thread.
double cubeComponent(ovphysx_instance_handle_t handle, const char* attribute, size_t component)
{
    session_test::Column column;
    if (!session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, attribute, column, "/World/Cube1"))
    {
        ADD_FAILURE() << "reading " << attribute << " of /World/Cube1 failed";
        return std::nan("");
    }
    const auto found = column.find("/World/Cube1");
    if (found == column.end() || found->second.size() <= component)
    {
        ADD_FAILURE() << "no " << attribute << " row for /World/Cube1";
        return std::nan("");
    }
    return found->second[component];
}

// Enables contact processing, a process-wide setting that another suite may have left
// disabled, and restores the previous value on destruction.
class ContactProcessingEnabled
{
public:
    ContactProcessingEnabled()
    {
        enabled = ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING, &m_previous).status ==
                      OVPHYSX_API_SUCCESS &&
                  ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING,
                                            ovphysx_config_value_bool(false))
                          .status == OVPHYSX_API_SUCCESS;
    }
    ~ContactProcessingEnabled()
    {
        if (enabled)
            ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING, m_previous);
    }
    bool enabled = false;

private:
    ovphysx_config_value_t m_previous{};
};

class WorkerStep : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        // The CPU instance is shared across suites; start from an empty task table.
        ovphysx_wait_task_result_t residual{};
        ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &residual);
        ovphysx_destroy_wait_task_result(&residual);
    }

    void loadScene() { ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kBoxes)); }
};

enum class FetchErrorClobber
{
    kClear,
    kReplace,
};

struct StepErrorProbe
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    FetchErrorClobber clobber = FetchErrorClobber::kClear;
    int preCalls = 0;
    int postCalls = 0;
    ovphysx_api_status_t metadataStatus = OVPHYSX_API_ERROR;
    ovphysx_enqueue_result_t nestedStep{};
};

void plantDispatchError(float, void* userData)
{
    StepErrorProbe& probe = *static_cast<StepErrorProbe*>(userData);
    ++probe.preCalls;
    ovphysx_record_runtime_error_for_test_internal("planted dispatch failure");
}

void clobberFetchError(float, void* userData)
{
    StepErrorProbe& probe = *static_cast<StepErrorProbe*>(userData);
    ++probe.postCalls;
    if (probe.clobber == FetchErrorClobber::kClear)
    {
        ovphysx_config_value_t value{};
        probe.metadataStatus =
            ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, &value).status;
    }
    else
    {
        probe.nestedStep = ovphysx_step(probe.handle, kDt);
    }
}

class StepEventSubscriptions
{
public:
    StepEventSubscriptions(ovphysx_instance_handle_t handle, StepErrorProbe& probe)
    {
        m_pre = ovphysx_subscribe_step_event_for_test_internal(handle, true, &plantDispatchError, &probe);
        m_post = ovphysx_subscribe_step_event_for_test_internal(handle, false, &clobberFetchError, &probe);
    }
    ~StepEventSubscriptions()
    {
        if (m_post)
            ovphysx_unsubscribe_step_event_for_test_internal(m_post);
        if (m_pre)
            ovphysx_unsubscribe_step_event_for_test_internal(m_pre);
    }

    bool valid() const noexcept
    {
        return m_pre != 0 && m_post != 0;
    }

private:
    uint64_t m_pre = 0;
    uint64_t m_post = 0;
};

// fetchResults delivers post-step callbacks on the executing thread. Neither a
// successful getter that clears TLS nor a rejected nested call may replace the
// dispatch failure that the step task reports.
TEST_F(WorkerStep, DispatchErrorSurvivesFetchCallbacks)
{
    loadScene();
    ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    float simulationTime = 0.0f;
    float contactDt = 0.0f;
    ASSERT_TRUE(ovphysx_read_step_bookkeeping_for_test_internal(m_handle, &simulationTime, &contactDt));
    EXPECT_FLOAT_EQ(contactDt, kDt);

    for (const FetchErrorClobber clobber : {FetchErrorClobber::kClear, FetchErrorClobber::kReplace})
    {
        StepErrorProbe probe;
        probe.handle = m_handle;
        probe.clobber = clobber;
        StepEventSubscriptions callbacks(m_handle, probe);
        ASSERT_TRUE(callbacks.valid());

        const float nextTime = simulationTime + 2.0f * kDt;
        const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 2.0f * kDt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(waitForTaskFailure(m_handle, step, "planted dispatch failure"));
        ASSERT_TRUE(ovphysx_read_step_bookkeeping_for_test_internal(m_handle, &simulationTime, &contactDt));
        EXPECT_FLOAT_EQ(contactDt, kDt);
        EXPECT_FLOAT_EQ(simulationTime, nextTime);
        EXPECT_EQ(probe.preCalls, 1);
        EXPECT_EQ(probe.postCalls, 1);
        if (clobber == FetchErrorClobber::kClear)
        {
            EXPECT_EQ(probe.metadataStatus, OVPHYSX_API_SUCCESS);
        }
        else
        {
            EXPECT_EQ(probe.nestedStep.status, OVPHYSX_API_INVALID_STATE);
            EXPECT_EQ(probe.nestedStep.op_index, 0u);
        }
    }
}

TEST_F(WorkerStep, ReturnsWhilePending)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);

    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(step.op_index, 0u);
    EXPECT_EQ(step.instance, m_handle);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_PENDING);

    ovphysx_wait_task_result_t polled{};
    EXPECT_EQ(waitStatus(m_handle, step, OVPHYSX_TIMEOUT_POLL, &polled), OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(polled.pending_task.op_index, step.op_index);
    EXPECT_EQ(polled.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&polled);
    // A finite wait ends at its deadline instead of finishing the step itself.
    const auto before = std::chrono::steady_clock::now();
    EXPECT_EQ(waitStatus(m_handle, step, 20'000'000ULL), OVPHYSX_API_TIMEOUT);
    EXPECT_GE(std::chrono::steady_clock::now() - before, std::chrono::milliseconds(15));
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_PENDING) << "timed-out waits must not collect or run it";

    hold.release();
    EXPECT_EQ(waitStatus(m_handle, step, OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_NOT_FOUND);
}

TEST_F(WorkerStep, ProgressesWithoutWait)
{
    loadScene();
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);

    // Only non-collecting status queries: no wait ever runs or finishes the step.
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_PENDING;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (status != OVPHYSX_TASK_STATUS_SUCCEEDED && status != OVPHYSX_TASK_STATUS_FAILED &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        status = statusOf(m_handle, step);
    }
    EXPECT_EQ(status, OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_EQ(waitStatus(m_handle, step, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_SUCCESS);
}

// dt, 0 and dt steps submitted without waits run in order with their own timesteps:
// Cube1 falls freely for exactly two steps, and the zero step simulates nothing.
// The last-positive-dt rule then keeps the contact force of a resting body exact
// across a trailing zero step.
TEST_F(WorkerStep, OrderedChain)
{
    ContactProcessingEnabled contactProcessing;
    ASSERT_TRUE(contactProcessing.enabled);
    loadScene();
    const double initialVelocity = cubeComponent(m_handle, OVPHYSX_ATTR_LINEAR_VELOCITY, 2);

    const ovphysx_enqueue_result_t first = ovphysx_step(m_handle, kDt);
    const ovphysx_enqueue_result_t zero = ovphysx_step(m_handle, 0.0f);
    const ovphysx_enqueue_result_t second = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(zero.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(second.status, OVPHYSX_API_SUCCESS);
    EXPECT_LT(first.op_index, zero.op_index);
    EXPECT_LT(zero.op_index, second.op_index);
    ovphysx_wait_task_result_t all{};
    ASSERT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &all).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(all.num_errors, 0u);
    EXPECT_EQ(all.pending_task.op_index, 0u);
    ovphysx_destroy_wait_task_result(&all);
    for (const ovphysx_enqueue_result_t& step : { first, zero, second })
        EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_NOT_FOUND);

    const double gainedSpeed = initialVelocity - cubeComponent(m_handle, OVPHYSX_ATTR_LINEAR_VELOCITY, 2);
    EXPECT_NEAR(gainedSpeed, 2.0 * kGravity * kDt, 0.02 * 2.0 * kGravity * kDt);

    // Let Cube1 land and settle, then read its resting contact force.
    ovphysx_string_t sensor = OVPHYSX_LITERAL("/World/Cube1");
    ovphysx_contact_binding_handle_t contacts = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(m_handle, &sensor, 1, nullptr, 0, 0, &contacts).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 240; ++i)
        ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    float force[3] = {};
    int64_t shape[2] = { 1, 3 };
    DLTensor dst{};
    dst.data = force;
    dst.device = { kDLCPU, 0 };
    dst.dtype = { kDLFloat, 32, 1 };
    dst.ndim = 2;
    dst.shape = shape;
    ASSERT_EQ(ovphysx_read_contact_net_normal_forces(m_handle, contacts, &dst).status, OVPHYSX_API_SUCCESS);
    const float resting = force[2];
    ASSERT_GT(resting, 1.0f) << "Cube1 should rest on its support";

    // A positive step, then a zero step, both unwaited: the zero step keeps the divisor
    // of the step that simulated, so the force stays the body's weight.
    ASSERT_EQ(ovphysx_step(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step(m_handle, 0.0f).status, OVPHYSX_API_SUCCESS);
    ovphysx_wait_task_result_t settled{};
    ASSERT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &settled).status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&settled);
    force[2] = 0.0f;
    ASSERT_EQ(ovphysx_read_contact_net_normal_forces(m_handle, contacts, &dst).status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(force[2], resting, 0.05f * resting);
    EXPECT_EQ(ovphysx_destroy_contact_binding(m_handle, contacts).status, OVPHYSX_API_SUCCESS);
}

// Isaac Sim's pattern: poll the step with a zero timeout until it is no longer
// TIMEOUT. The step is collected exactly once.
TEST_F(WorkerStep, PollThenCollectOnce)
{
    loadScene();
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);

    ovphysx_api_status_t status = OVPHYSX_API_TIMEOUT;
    int successes = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (status == OVPHYSX_API_TIMEOUT && std::chrono::steady_clock::now() < deadline)
    {
        ovphysx_wait_task_result_t result{};
        status = ovphysx_wait_task(m_handle, step, OVPHYSX_TIMEOUT_POLL, &result).status;
        if (status == OVPHYSX_API_TIMEOUT)
        {
            EXPECT_EQ(result.pending_task.op_index, step.op_index);
        }
        if (status == OVPHYSX_API_SUCCESS)
            ++successes;
        ovphysx_destroy_wait_task_result(&result);
    }
    EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(successes, 1);
    EXPECT_EQ(waitStatus(m_handle, step, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_NOT_FOUND);
}

// wait_all with a zero timeout collects the finished prefix, a failure included, and
// stops at the held step, which stays retryable.
TEST_F(WorkerStep, WaitAllTimeoutPrefix)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_enqueue_result_t failed =
        ovphysx_submit_test_job_for_test_internal(m_handle, "planted failure", 0);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (statusOf(m_handle, failed) != OVPHYSX_TASK_STATUS_FAILED && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(statusOf(m_handle, failed), OVPHYSX_TASK_STATUS_FAILED);

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);

    ovphysx_wait_task_result_t prefix{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_POLL, &prefix).status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(prefix.pending_task.op_index, step.op_index);
    EXPECT_EQ(prefix.pending_task.instance, m_handle);
    ASSERT_EQ(prefix.num_errors, 1u);
    EXPECT_EQ(prefix.error_tasks[0].op_index, failed.op_index);
    const ovphysx_string_t message = ovphysx_get_last_task_error(m_handle, failed);
    EXPECT_NE(std::string(message.ptr ? message.ptr : "", message.length).find("planted failure"), std::string::npos);
    ovphysx_destroy_wait_task_result(&prefix);
    EXPECT_EQ(statusOf(m_handle, failed), OVPHYSX_TASK_STATUS_NOT_FOUND);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_PENDING);

    hold.release();
    ovphysx_wait_task_result_t rest{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &rest).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(rest.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&rest);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_NOT_FOUND);
}

// step_sync waits for the step job without collecting it.
TEST_F(WorkerStep, SyncDrainDoesNotCollect)
{
    loadScene();
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_EQ(waitStatus(m_handle, step, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(waitStatus(m_handle, step, OVPHYSX_TIMEOUT_POLL), OVPHYSX_API_NOT_FOUND);
}

// A query and a read submitted right after an unwaited step run after it and see its
// result; the step stays collectable.
TEST_F(WorkerStep, ShortCallAfterUnwaitedStep)
{
    loadScene();
    const double before = cubeComponent(m_handle, OVPHYSX_ATTR_POSITION, 2);
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    const double after = cubeComponent(m_handle, OVPHYSX_ATTR_POSITION, 2);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_LT(after, before) << "the read must see the falling cube after the step";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, step));
}

// Destroying an instance right after an unwaited step, still queued or running on the
// worker, tears down cleanly; a peer instance keeps stepping.
TEST_F(WorkerStep, DestroyRightAfterUnwaitedStep)
{
    loadScene();
    for (int round = 0; round < 3; ++round)
    {
        ovphysx_instance_handle_t victim = OVPHYSX_INVALID_HANDLE;
        ASSERT_EQ(ovphysx_create_instance(&victim).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(victim, kBoxes, 1, "worker-step-destroy"));
        ASSERT_EQ(ovphysx_step_sync(victim, kDt).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_step(victim, kDt).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(ovphysx_destroy_instance(victim).status, OVPHYSX_API_SUCCESS);
        test_utils::destroy_ovstage_test_stages(victim);
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_step(m_handle, kDt)));
    }
}

// What a log callback inside a step job observed.
struct WorkerCallbackProbe
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_enqueue_result_t step{};
    std::thread::id testThread;
    std::atomic<bool> called{false};
    bool onOtherThread = false;
    ovphysx_api_status_t waitAll = OVPHYSX_API_SUCCESS;
    std::string waitAllError;
    ovphysx_api_status_t waitStep = OVPHYSX_API_SUCCESS;
    ovphysx_api_status_t stepSync = OVPHYSX_API_SUCCESS;
    ovphysx_enqueue_result_t nestedStep{ OVPHYSX_API_SUCCESS, 1, 1 };
    ovphysx_api_status_t statusQuery = OVPHYSX_API_ERROR;
    ovphysx_task_status_t stepStatus = OVPHYSX_TASK_STATUS_NOT_FOUND;
};

// Object-change callbacks do not fire inside a step of this fixture (the initial
// population is silent and edits apply during updates), so the callback that the step
// job runs on the worker is the log callback for the initial scene parse.
TEST_F(WorkerStep, CallbackOnWorkerCannotWaitOwnInstance)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    WorkerCallbackProbe probe;
    probe.handle = m_handle;
    probe.testThread = std::this_thread::get_id();

    const uint32_t logLevel = ovphysx_get_log_level();
    struct Restore
    {
        uint32_t level;
        ~Restore()
        {
            ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
            ovphysx_set_log_level(level);
        }
    } restore{ logLevel };
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_INFO).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_log_callback(
                  OVPHYSX_LOG_INFO, nullptr,
                  [](ovphysx_log_level_t, ovphysx_string_t message, ovphysx_string_t, double, void* data) {
                      const std::string text(message.ptr ? message.ptr : "", message.length);
                      WorkerCallbackProbe& probe = *static_cast<WorkerCallbackProbe*>(data);
                      if (text.find("scene parse completed") == std::string::npos || probe.called.exchange(true))
                          return;
                      probe.onOtherThread = std::this_thread::get_id() != probe.testThread;
                      ovphysx_wait_task_result_t waited{};
                      probe.waitAll = ovphysx_wait_all(probe.handle, OVPHYSX_TIMEOUT_INFINITE, &waited).status;
                      const ovphysx_string_t error = ovphysx_get_last_error();
                      probe.waitAllError.assign(error.ptr ? error.ptr : "", error.length);
                      ovphysx_destroy_wait_task_result(&waited);
                      probe.waitStep = waitStatus(probe.handle, probe.step, OVPHYSX_TIMEOUT_INFINITE);
                      probe.stepSync = ovphysx_step_sync(probe.handle, kDt).status;
                      probe.nestedStep = ovphysx_step(probe.handle, kDt);
                      probe.statusQuery =
                          ovphysx_query_task_status(probe.handle, probe.step, &probe.stepStatus).status;
                  },
                  &probe).status,
              OVPHYSX_API_SUCCESS);

    // The worker is held until the receipt is stored, so the callback can name it, and
    // the hold queues the whole step as a job, so the worker runs the initial parse.
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    probe.step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(probe.step.status, OVPHYSX_API_SUCCESS);
    hold.release();
    EXPECT_TRUE(waitForTaskSuccess(m_handle, probe.step, kWorkerRunsIt));
    ASSERT_EQ(ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE).status, OVPHYSX_API_SUCCESS);

    ASSERT_TRUE(probe.called.load()) << "the step job logged no initial scene parse";
    EXPECT_TRUE(probe.onOtherThread) << "the step must run on the instance worker";
    EXPECT_EQ(probe.waitAll, OVPHYSX_API_INVALID_STATE);
    EXPECT_NE(probe.waitAllError.find("callback"), std::string::npos) << probe.waitAllError;
    EXPECT_EQ(probe.waitStep, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.stepSync, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.nestedStep.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.nestedStep.op_index, 0u);
    EXPECT_EQ(probe.nestedStep.instance, 0u);
    EXPECT_EQ(probe.statusQuery, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(probe.stepStatus, OVPHYSX_TASK_STATUS_RUNNING);
}

// A step whose lazy stage attach fails reports the failure, synchronously and as a task:
// the attach clears the attachment under the instance map's exclusive lock, which the step
// does not hold meanwhile. The attachment is faked with a backing stage id no stage has.
TEST_F(WorkerStep, FailedLazyStageAttachFailsTheStep)
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(ovphysx_set_ovstage_attachment_state_internal(handle, true, 42));
    EXPECT_EQ(ovphysx_step_sync(handle, kDt).status, OVPHYSX_API_ERROR);
    ASSERT_TRUE(ovphysx_set_ovstage_attachment_state_internal(handle, true, 42));
    EXPECT_EQ(waitStatus(handle, ovphysx_step(handle, kDt), OVPHYSX_TIMEOUT_INFINITE), OVPHYSX_API_ERROR);
    ASSERT_TRUE(ovphysx_set_ovstage_attachment_state_internal(handle, false, 0));
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

// A step on an idle instance whose scene is parsed runs nothing inside the call, even with
// zero solver threads, where the runtime runs the whole simulation on the thread that
// starts it. The step job is held as it is queued, so a falling cube keeps its pose until
// the hold ends.
TEST_F(WorkerStep, IdleStepRunsNothingInTheCallWithZeroSolverThreads)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    // The solver thread count changes only while no Stage is attached.
    ovphysx_config_value_t threads{};
    ASSERT_EQ(ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &threads).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(0)).status,
              OVPHYSX_API_SUCCESS);
    struct Restore
    {
        ovphysx_instance_handle_t handle;
        ovphysx_config_value_t threads;
        ~Restore()
        {
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
            EXPECT_EQ(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, threads).status,
                      OVPHYSX_API_SUCCESS);
        }
    } restore{ m_handle, threads };
    loadScene();
    ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    physx::PxScene* const scene = static_cast<physx::PxScene*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene"));
    physx::PxRigidDynamic* const cube = static_cast<physx::PxRigidDynamic*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(scene, nullptr);
    ASSERT_NE(cube, nullptr);
    ASSERT_EQ(scene->getCpuDispatcher()->getWorkerCount(), 0u);
    const float before = cube->getGlobalPose().p.z;

    ASSERT_TRUE(ovphysx_hold_worker_after_next_submit_for_test_internal(m_handle));
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(cube->getGlobalPose().p.z, before) << "the step ran inside the call";
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_PENDING);
    ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(m_handle, false));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, step));
    EXPECT_LT(cube->getGlobalPose().p.z, before);
}

#if OVPHYSX_ENABLE_GPU_TESTS

// The articulation query-type probe of a kinematic update runs only after an unwaited
// GPU step finished on the worker.
// The first step after an attach runs the initial scene parse, which can take long on a
// large scene, so it runs whole on the worker even on an idle instance: the call returns
// before the parse ends. The parse's log callback waits for the call to return; were the
// parse inside the call, that wait could only end at its 10 s limit.
TEST_F(WorkerStep, InitialSceneParseRunsOnTheWorkerAfterTheCallReturns)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    struct ParseProbe
    {
        std::thread::id caller;
        std::atomic<bool> callReturned{false};
        std::atomic<bool> parsed{false};
        std::atomic<bool> onCaller{false};
        std::atomic<bool> sawReturn{false};
    } probe;
    probe.caller = std::this_thread::get_id();

    const uint32_t logLevel = ovphysx_get_log_level();
    struct Restore
    {
        uint32_t level;
        ~Restore()
        {
            ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
            ovphysx_set_log_level(level);
        }
    } restore{ logLevel };
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_INFO).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_log_callback(
                  OVPHYSX_LOG_INFO, nullptr,
                  [](ovphysx_log_level_t, ovphysx_string_t message, ovphysx_string_t, double, void* data) {
                      const std::string text(message.ptr ? message.ptr : "", message.length);
                      ParseProbe& probe = *static_cast<ParseProbe*>(data);
                      if (text.find("scene parse completed") == std::string::npos || probe.parsed.exchange(true))
                          return;
                      probe.onCaller = std::this_thread::get_id() == probe.caller;
                      const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(10);
                      while (!probe.callReturned.load() && std::chrono::steady_clock::now() < limit)
                          std::this_thread::sleep_for(std::chrono::milliseconds(1));
                      probe.sawReturn = probe.callReturned.load();
                  },
                  &probe).status,
              OVPHYSX_API_SUCCESS);

    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    probe.callReturned = true;
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, step, kWorkerRunsIt));
    ASSERT_EQ(ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE).status, OVPHYSX_API_SUCCESS);

    ASSERT_TRUE(probe.parsed.load()) << "the step logged no initial scene parse";
    EXPECT_FALSE(probe.onCaller.load()) << "the parse ran on the calling thread";
    EXPECT_TRUE(probe.sawReturn.load()) << "the step call waited for the initial scene parse";

    // Later steps complete as before.
    const ovphysx_enqueue_result_t next = ovphysx_step(m_handle, kDt);
    ASSERT_EQ(next.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, next));
}

TEST_F(WorkerStep, KinematicUpdateAfterUnwaitedStepGpuTest)
{
    if (!ovphysx::test_cuda::cudaAvailable())
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for the GPU kinematic update after an unwaited step";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    ovphysx_instance_handle_t gpu = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&gpu).status, OVPHYSX_API_SUCCESS);
    struct Destroy
    {
        ovphysx_instance_handle_t handle;
        ~Destroy()
        {
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
            EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
            test_utils::destroy_ovstage_test_stages(handle);
        }
    } destroy{ gpu };
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(gpu, "tests/data/two_articulations_gpu.usda"));
    ASSERT_EQ(ovphysx_step_sync(gpu, kDt).status, OVPHYSX_API_SUCCESS);
    ovphysx_query_handle_t articulations = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        gpu, ovphysx_query(gpu, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &articulations, nullptr)));
    ASSERT_NE(articulations, 0u);

    const ovphysx_enqueue_result_t step = ovphysx_step(gpu, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t update = ovphysx_update_articulations_kinematic(
        gpu, articulations, OVPHYSX_ARTICULATION_KINEMATIC_POSITION, ovstage_cuda_sync_t{ 0, 0 });
    EXPECT_EQ(update.status, OVPHYSX_API_SUCCESS)
        << std::string(ovphysx_get_last_error().ptr, ovphysx_get_last_error().length);
    if (update.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_TRUE(waitForTaskSuccess(gpu, update));
    }
    EXPECT_TRUE(waitForTaskSuccess(gpu, step));
    EXPECT_TRUE(waitForTaskSuccess(gpu, ovphysx_release_query(gpu, articulations)));
}

#endif // OVPHYSX_ENABLE_GPU_TESTS

} // namespace
