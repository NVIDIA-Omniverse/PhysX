// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-9 AC-10 AC-11 AC-12 AC-13 AC-15
 *
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-2
 *
 * TEST-CAPI-ASYNC-006: short async calls run on the calling thread when their instance
 * is idle and are queued behind its work otherwise.
 */

// Short async calls (queries, reads, writes, fetches, releases and unmasked commits of
// host groups with nothing outstanding): on an idle instance each runs on the calling
// thread and returns a finished task; behind queued, running or held work it is queued
// with copies of its small inputs, returns a pending task at once, writes its outputs when
// it runs and keeps submission order with every other call. A masked commit, a commit
// with a CUDA handoff or of a write session that handed out device memory, the
// articulation update and the SDF evaluation still wait for earlier work on the calling
// thread. On both paths a call reports a state that earlier work set through its task,
// with the same text. Cases about one venue skip themselves under the test mode
// (OVPHYSX_TEST_EXECUTION) that removes it.

#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "session_test_helpers.h"
#include "test_utilities.h"
#if OVPHYSX_ENABLE_GPU_TESTS
#include "cuda_test_helpers.h"
#endif

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>
#include <ovphysx/ovphysx_geometry_query.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr const char* kBoxes = "tests/data/boxes_falling_on_groundplane.usda";
constexpr float kDt = 1.0f / 60.0f;
constexpr uint32_t kJobMs = 200;

// Why a case about the calling thread's venue skips itself under the private test switch
// that removes that venue (see ovphysxIdleCallsRunOnCaller()).
constexpr const char* kEveryCallQueued = "OVPHYSX_TEST_EXECUTION=queue: no short call runs on the calling thread";

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
        if (held.exchange(false))
            ovphysx_set_worker_hold_for_test_internal(m_handle, false);
    }
    std::atomic<bool> held{false};

private:
    ovphysx_instance_handle_t m_handle;
};

ovphysx_task_status_t statusOf(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_NOT_FOUND;
    EXPECT_EQ(ovphysx_query_task_status(handle, task, &status).status, OVPHYSX_API_SUCCESS);
    return status;
}

// Collects every task of the instance; failures are tolerated.
void drainTasks(ovphysx_instance_handle_t handle)
{
    ovphysx_wait_task_result_t residual{};
    ovphysx_wait_all(handle, OVPHYSX_TIMEOUT_INFINITE, &residual);
    ovphysx_destroy_wait_task_result(&residual);
}

ovstage_cuda_sync_t noSync()
{
    return ovstage_cuda_sync_t{0, 0};
}

ovx_token_or_string_t attributeName(const char* name)
{
    ovx_token_or_string_t attribute{};
    attribute.string = {name, std::strlen(name)};
    return attribute;
}

std::string lastError()
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    return std::string(error.ptr ? error.ptr : "", error.length);
}

// Collects task, which must have failed, and returns its failure text (empty when it did
// not fail). The text is owned by the wait result, so it is copied before that goes.
std::string collectFailure(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_wait_task_result_t result{};
    const ovphysx_api_status_t status = ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &result).status;
    std::string text;
    if (status == OVPHYSX_API_ERROR)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, task);
        text.assign(error.ptr ? error.ptr : "", error.length);
    }
    ovphysx_destroy_wait_task_result(&result);
    return text;
}

// The text a task of api fails with: "<api> (task <index>): <cause>", on both paths.
std::string taskText(const char* api, ovphysx_enqueue_result_t task, const std::string& cause)
{
    return std::string(api) + " (task " + std::to_string(task.op_index) + "): " + cause;
}

// A queued call: accepted with a receipt and still pending while the worker is held.
::testing::AssertionResult queued(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS || task.op_index == 0 || task.instance != handle)
        return ::testing::AssertionFailure() << "rejected with status " << task.status << ": " << lastError();
    const ovphysx_task_status_t status = statusOf(handle, task);
    if (status != OVPHYSX_TASK_STATUS_PENDING)
        return ::testing::AssertionFailure() << "task " << task.op_index << " is " << status << ", not pending";
    return ::testing::AssertionSuccess();
}

// A call that ran on the calling thread: its task finished before the call returned.
::testing::AssertionResult ranInline(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS || task.op_index == 0)
        return ::testing::AssertionFailure() << "rejected with status " << task.status << ": " << lastError();
    const ovphysx_task_status_t status = statusOf(handle, task);
    if (status != OVPHYSX_TASK_STATUS_SUCCEEDED)
        return ::testing::AssertionFailure() << "task " << task.op_index << " is " << status << ", not finished";
    return ::testing::AssertionSuccess();
}

// Holds the worker of handle with a sleeping test job queued, makes call on a helper
// thread and releases the hold after 50 ms. Succeeds when the call returned only once
// the job had finished: it waited for earlier work on the calling thread instead of
// queuing behind it. The call's own status is the caller's to check.
template <typename Call>
::testing::AssertionResult waitsForEarlierWork(ovphysx_instance_handle_t handle, Call&& call)
{
    WorkerHold hold(handle);
    if (!hold.held)
        return ::testing::AssertionFailure() << "the worker hold is unavailable";
    const ovphysx_enqueue_result_t job = ovphysx_submit_test_job_for_test_internal(handle, nullptr, kJobMs);
    if (job.status != OVPHYSX_API_SUCCESS)
        return ::testing::AssertionFailure() << "the test job was rejected: " << job.status;
    std::atomic<bool> returned{false};
    ovphysx_task_status_t jobAtReturn = OVPHYSX_TASK_STATUS_NOT_FOUND;
    std::thread caller([&] {
        call();
        jobAtReturn = statusOf(handle, job);
        returned = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const bool returnedWhileHeld = returned.load();
    hold.release();
    caller.join();
    if (returnedWhileHeld)
        return ::testing::AssertionFailure() << "the call returned before the earlier job ran";
    if (jobAtReturn != OVPHYSX_TASK_STATUS_SUCCEEDED)
        return ::testing::AssertionFailure() << "the earlier job was not finished when the call returned: "
                                             << jobAtReturn;
    return ::testing::AssertionSuccess();
}

// Every float of the first group of a host read of attribute over query.
std::vector<float> readFloats(ovphysx_instance_handle_t handle, ovphysx_query_handle_t query, const char* attribute)
{
    std::vector<float> values;
    const ovx_token_or_string_t name = attributeName(attribute);
    ovphysx_read_handle_t read = 0;
    EXPECT_TRUE(waitForTaskSuccess(
        handle, ovphysx_read(handle, query, &name, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
    const ovstage_read_group_t* group = nullptr;
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_fetch_read_next(handle, read, &group)));
    if (group && group->columns[0].data.tensor_count && group->columns[0].data.tensors[0].device.device_type == kDLCPU)
    {
        const DLTensor& tensor = group->columns[0].data.tensors[0];
        const float* data = static_cast<const float*>(tensor.data);
        values.assign(data, data + session_test::detail::elementCount(tensor));
    }
    if (group)
    {
        EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_group(handle, read, group->read_group_id)));
    }
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_read(handle, read)));
    return values;
}

class WorkerShortCalls : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        // The CPU instance is shared across suites; start from an empty task table.
        drainTasks(m_handle);
    }

    void loadScene()
    {
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kBoxes));
        ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_query_handle_t openQuery(ovphysx_query_target_t type = OVPHYSX_QUERY_TARGET_RIGID_BODY)
    {
        ovphysx_query_handle_t query = 0;
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(m_handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)));
        EXPECT_NE(query, 0u);
        return query;
    }

    ovphysx_query_handle_t openSelected(const char* pattern)
    {
        ovphysx_query_selection_t selection{};
        selection.pattern = {pattern, std::strlen(pattern)};
        ovphysx_query_handle_t query = 0;
        EXPECT_TRUE(waitForTaskSuccess(
            m_handle, ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &selection, &query, nullptr)));
        EXPECT_NE(query, 0u);
        return query;
    }

    // A write session over query and its first group, both opened on an idle instance.
    void openWrite(ovphysx_query_handle_t query, const char* attribute, ovphysx_write_handle_t& write,
                   ovphysx_fetched_write_group_t& group)
    {
        const ovx_token_or_string_t name = attributeName(attribute);
        write = 0;
        group = {};
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, &name, 1, &write, nullptr)));
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, write, &group)));
        ASSERT_NE(group.group, nullptr);
        ASSERT_EQ(group.group->data.tensors[0].device.device_type, kDLCPU);
    }
};

// On an idle instance every short call runs on the calling thread: its task has finished
// when the call returns, and its outputs are written.
TEST_F(WorkerShortCalls, IdleInstanceRunsShortCallsInline)
{
    if (!ovphysxIdleCallsRunOnCaller())
        GTEST_SKIP() << kEveryCallQueued;
    loadScene();
    ovphysx_query_handle_t query = 0;
    EXPECT_TRUE(ranInline(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query,
                                                  nullptr)));
    ASSERT_NE(query, 0u);
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    EXPECT_TRUE(ranInline(
        m_handle, ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
    ASSERT_NE(read, 0u);
    const ovstage_read_group_t* group = nullptr;
    EXPECT_TRUE(ranInline(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
    ASSERT_NE(group, nullptr);
    EXPECT_TRUE(ranInline(m_handle, ovphysx_release_group(m_handle, read, group->read_group_id)));
    EXPECT_TRUE(ranInline(m_handle, ovphysx_release_read(m_handle, read)));

    const ovx_token_or_string_t velocity = attributeName(OVPHYSX_ATTR_LINEAR_VELOCITY);
    ovphysx_write_handle_t write = 0;
    EXPECT_TRUE(ranInline(m_handle, ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr)));
    ASSERT_NE(write, 0u);
    ovphysx_fetched_write_group_t fetched{};
    EXPECT_TRUE(ranInline(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
    ASSERT_NE(fetched.group, nullptr);
    const DLTensor& column = fetched.group->data.tensors[0];
    std::fill_n(static_cast<float*>(column.data), session_test::detail::elementCount(column), 0.0f);
    EXPECT_TRUE(ranInline(m_handle, ovphysx_commit_group(m_handle, write, fetched.group, noSync())));
    EXPECT_TRUE(ranInline(m_handle, ovphysx_release_write(m_handle, write)));
    EXPECT_TRUE(ranInline(m_handle, ovphysx_release_query(m_handle, query)));
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_POLL, &all).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(all.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&all);
}

// Behind held work every short call is queued: it returns a pending receipt at once, a
// query or read with its handle, leaves its other outputs cleared until it runs, and then
// writes them.
TEST_F(WorkerShortCalls, QueuedCallsReturnPendingAndWriteOutputsWhenTheyRun)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    ovphysx_write_handle_t write = 0;
    ovphysx_fetched_write_group_t first{};
    openWrite(query, OVPHYSX_ATTR_LINEAR_VELOCITY, write, first);
    const DLTensor& column = first.group->data.tensors[0];
    std::fill_n(static_cast<float*>(column.data), session_test::detail::elementCount(column), 0.0f);

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    ovphysx_query_handle_t second = 0;
    ovstage_query_result_t summary{};
    const ovphysx_enqueue_result_t queuedQuery =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &second, &summary);
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    const ovphysx_enqueue_result_t queuedRead =
        ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    const ovphysx_enqueue_result_t queuedCommit = ovphysx_commit_group(m_handle, write, first.group, noSync());
    ovphysx_fetched_write_group_t next{};
    const ovphysx_enqueue_result_t queuedFetch = ovphysx_fetch_write_next(m_handle, write, &next);
    const ovphysx_enqueue_result_t queuedRelease = ovphysx_release_write(m_handle, write);
    uint64_t previous = 0;
    for (const ovphysx_enqueue_result_t& task : {queuedQuery, queuedRead, queuedCommit, queuedFetch, queuedRelease})
    {
        EXPECT_TRUE(queued(m_handle, task));
        EXPECT_GT(task.op_index, previous) << "receipts follow submission order";
        previous = task.op_index;
    }
    EXPECT_NE(second, 0u);
    EXPECT_EQ(summary.total_prim_count, 0u);
    EXPECT_NE(read, 0u);
    EXPECT_EQ(next.group, nullptr);
    EXPECT_EQ(next.attribute_index, SIZE_MAX);

    hold.release();
    for (const ovphysx_enqueue_result_t& task : {queuedQuery, queuedRead, queuedCommit, queuedFetch, queuedRelease})
        EXPECT_TRUE(waitForTaskSuccess(m_handle, task));
    EXPECT_NE(second, 0u);
    EXPECT_GT(summary.total_prim_count, 0u);
    ASSERT_NE(read, 0u);
    const ovstage_read_group_t* group = nullptr;
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
    EXPECT_NE(group, nullptr);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, second)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// Queued and later calls keep submission order: a read queued behind a step sees the
// stepped state, and the release of a query is accepted when it is submitted, so a read of
// that query submitted after it is rejected at once, as for an unknown handle, even while
// the release is still queued.
TEST_F(WorkerShortCalls, QueuedCallsKeepSubmissionOrder)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t cube = openSelected("/World/Cube1");
    const std::vector<float> before = readFloats(m_handle, cube, OVPHYSX_ATTR_POSITION);
    ASSERT_EQ(before.size(), 3u);

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t afterStep = 0;
    const ovphysx_enqueue_result_t readAfterStep =
        ovphysx_read(m_handle, cube, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &afterStep, nullptr);
    const ovphysx_enqueue_result_t release = ovphysx_release_query(m_handle, cube);
    for (const ovphysx_enqueue_result_t& task : {step, readAfterStep, release})
        EXPECT_TRUE(queued(m_handle, task));
    ovphysx_read_handle_t afterRelease = 7;
    const ovphysx_enqueue_result_t readAfterRelease =
        ovphysx_read(m_handle, cube, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &afterRelease, nullptr);
    EXPECT_EQ(readAfterRelease.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(readAfterRelease.op_index, 0u);
    EXPECT_NE(lastError().find("query is not owned by this instance"), std::string::npos) << lastError();
    EXPECT_EQ(afterRelease, 0u);
    hold.release();

    EXPECT_TRUE(waitForTaskSuccess(m_handle, step));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, readAfterStep));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, release));

    ASSERT_NE(afterStep, 0u);
    const ovstage_read_group_t* group = nullptr;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, afterStep, &group)));
    ASSERT_NE(group, nullptr);
    const float* stepped = static_cast<const float*>(group->columns[0].data.tensors[0].data);
    EXPECT_LT(stepped[2], before[2]) << "the read queued behind the step must see the falling cube";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, afterStep)));
}

// A short call made right after a wait for a step has returned runs on the calling
// thread: the worker hands the lane back with the step's outcome, so nothing is left for
// the call to queue behind.
TEST_F(WorkerShortCalls, ShortCallAfterAWaitedStepRunsInline)
{
    if (!ovphysxIdleCallsRunOnCaller())
        GTEST_SKIP() << kEveryCallQueued;
    loadScene();
    for (int i = 0; i < 200; ++i)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_step(m_handle, kDt)));
        ovphysx_query_handle_t query = 0;
        ASSERT_TRUE(ranInline(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL,
                                                      &query, nullptr)))
            << "iteration " << i;
        ASSERT_NE(query, 0u);
        ASSERT_TRUE(ranInline(m_handle, ovphysx_release_query(m_handle, query))) << "iteration " << i;
        drainTasks(m_handle);
    }
}

// A query on an idle instance runs on the calling thread also when it first runs the lazy
// physics attach and the initial scene parse of a newly attached scene.
TEST_F(WorkerShortCalls, QueryThatParsesTheSceneRunsInlineOnAnIdleInstance)
{
    if (!ovphysxIdleCallsRunOnCaller())
        GTEST_SKIP() << kEveryCallQueued;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kBoxes));
    ovphysx_query_handle_t query = 0;
    EXPECT_TRUE(ranInline(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL,
                                                  &query, nullptr)));
    ASSERT_NE(query, 0u);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A queued call copies its small inputs at submission: the caller may overwrite and free
// the attribute arrays and their strings, the selections and their patterns right after
// the call returns.
TEST_F(WorkerShortCalls, InputsAreCopiedAtSubmission)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    ovx_token_t positionToken{0};
    {
        const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
        ovphysx_read_handle_t read = 0;
        ASSERT_TRUE(waitForTaskSuccess(
            m_handle, ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
        const ovstage_read_group_t* group = nullptr;
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
        ASSERT_NE(group, nullptr);
        positionToken = group->columns[0].attribute;
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
    }

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    auto clobber = [](std::unique_ptr<char[]>& text, size_t length) {
        std::memset(text.get(), 'x', length);
        text.reset();
    };

    // Read: the attribute array and its string.
    const size_t positionLength = std::strlen(OVPHYSX_ATTR_POSITION);
    std::unique_ptr<char[]> readName(new char[positionLength]);
    std::memcpy(readName.get(), OVPHYSX_ATTR_POSITION, positionLength);
    std::unique_ptr<ovx_token_or_string_t[]> readAttributes(new ovx_token_or_string_t[1]);
    readAttributes[0] = ovx_token_or_string_t{};
    readAttributes[0].string = {readName.get(), positionLength};
    ovphysx_read_handle_t read = 0;
    const ovphysx_enqueue_result_t readTask =
        ovphysx_read(m_handle, query, readAttributes.get(), 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    readAttributes[0].string = {"mass", 4};
    readAttributes.reset();
    clobber(readName, positionLength);

    // Read by token: a NULL string selects the token, and the length beside it is never read,
    // so it may be anything.
    ovx_token_or_string_t byToken{};
    byToken.token = positionToken;
    byToken.string = {nullptr, positionLength};
    ovphysx_read_handle_t tokenRead = 0;
    const ovphysx_enqueue_result_t tokenReadTask =
        ovphysx_read(m_handle, query, &byToken, 1, ovstage_cuda_sync_t{ 0, 0 }, &tokenRead, nullptr);

    // Selected query: the selection and its pattern.
    const std::string cubePattern = "/World/Cube1";
    std::unique_ptr<char[]> pattern(new char[cubePattern.size()]);
    std::memcpy(pattern.get(), cubePattern.data(), cubePattern.size());
    std::unique_ptr<ovphysx_query_selection_t> selection(new ovphysx_query_selection_t{});
    selection->pattern = {pattern.get(), cubePattern.size()};
    ovphysx_query_handle_t selected = 0;
    ovstage_query_result_t selectedSummary{};
    const ovphysx_enqueue_result_t selectedTask = ovphysx_query_selection(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, selection.get(), &selected, &selectedSummary);
    selection->pattern = {"/World", 6};
    selection.reset();
    clobber(pattern, cubePattern.size());

    // Write: the attribute array and its string.
    const size_t velocityLength = std::strlen(OVPHYSX_ATTR_LINEAR_VELOCITY);
    std::unique_ptr<char[]> writeName(new char[velocityLength]);
    std::memcpy(writeName.get(), OVPHYSX_ATTR_LINEAR_VELOCITY, velocityLength);
    std::unique_ptr<ovx_token_or_string_t[]> writeAttributes(new ovx_token_or_string_t[1]);
    writeAttributes[0] = ovx_token_or_string_t{};
    writeAttributes[0].string = {writeName.get(), velocityLength};
    ovphysx_write_handle_t write = 0;
    ovphysx_write_result_t writeSummary{};
    const ovphysx_enqueue_result_t writeTask =
        ovphysx_write(m_handle, query, writeAttributes.get(), 1, &write, &writeSummary);
    writeAttributes.reset();
    clobber(writeName, velocityLength);

    // Contact query: the sensor selection array and its pattern.
    std::unique_ptr<char[]> sensorPattern(new char[cubePattern.size()]);
    std::memcpy(sensorPattern.get(), cubePattern.data(), cubePattern.size());
    std::unique_ptr<ovphysx_contact_sensor_selection_t[]> sensors(new ovphysx_contact_sensor_selection_t[1]);
    sensors[0] = ovphysx_contact_sensor_selection_t{};
    sensors[0].sensors.pattern = {sensorPattern.get(), cubePattern.size()};
    ovphysx_query_handle_t contacts = 0;
    const ovphysx_enqueue_result_t contactsTask =
        ovphysx_query_contacts(m_handle, sensors.get(), 1, 0, &contacts, nullptr);
    sensors.reset();
    clobber(sensorPattern, cubePattern.size());

    for (const ovphysx_enqueue_result_t& task : {readTask, tokenReadTask, selectedTask, writeTask, contactsTask})
        EXPECT_TRUE(queued(m_handle, task));
    hold.release();

    ASSERT_TRUE(waitForTaskSuccess(m_handle, readTask));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, tokenReadTask));
    for (const ovphysx_read_handle_t session : {read, tokenRead})
    {
        const ovstage_read_group_t* group = nullptr;
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, session, &group)));
        ASSERT_NE(group, nullptr) << "the queued read lost its attribute";
        EXPECT_EQ(group->columns[0].attribute, positionToken);
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, session)));
    }

    ASSERT_TRUE(waitForTaskSuccess(m_handle, selectedTask));
    EXPECT_EQ(selectedSummary.total_prim_count, 1u) << "the queued selection lost its pattern";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, selected)));

    ASSERT_TRUE(waitForTaskSuccess(m_handle, writeTask));
    ASSERT_EQ(writeSummary.attribute_count, 1u);
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
    EXPECT_NE(fetched.group, nullptr) << "the queued write lost its attribute";
    EXPECT_EQ(fetched.attribute_index, 0u);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));

    ASSERT_TRUE(waitForTaskSuccess(m_handle, contactsTask));
    EXPECT_NE(contacts, 0u);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, contacts)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A masked commit always runs on the calling thread after earlier work, where the runtime
// checks its mask against the group. A mask that does not fit (here one entry too many) is
// rejected with no task and the same text on the idle instance and behind held work, and
// the group stays live; a mask that fits commits only the rows it selects.
TEST_F(WorkerShortCalls, MaskedCommitRunsOnTheCallingThreadAfterEarlierWork)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    const std::vector<float> before = readFloats(m_handle, query, OVPHYSX_ATTR_POSITION);
    ASSERT_FALSE(before.empty());
    ovphysx_write_handle_t write = 0;
    ovphysx_fetched_write_group_t fetched{};
    openWrite(query, OVPHYSX_ATTR_POSITION, write, fetched);
    const DLTensor& column = fetched.group->data.tensors[0];
    std::fill_n(static_cast<float*>(column.data), session_test::detail::elementCount(column), 777.0f);

    std::vector<uint8_t> tooLong(static_cast<size_t>(column.shape[0]) + 1, 1);
    int64_t tooLongRows = static_cast<int64_t>(tooLong.size());
    DLTensor refusedMask{};
    refusedMask.data = tooLong.data();
    refusedMask.device = DLDevice{kDLCPU, 0};
    refusedMask.ndim = 1;
    refusedMask.dtype = DLDataType{kDLUInt, 8, 1};
    refusedMask.shape = &tooLongRows;
    const ovphysx_enqueue_result_t idle =
        ovphysx_commit_group_masked(m_handle, write, fetched.group, &refusedMask, noSync());
    const std::string idleError = lastError();
    EXPECT_EQ(idle.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(idle.op_index, 0u);
    EXPECT_NE(idleError.find("the row mask is rejected"), std::string::npos) << idleError;
    ovphysx_enqueue_result_t busy{};
    std::string busyError;
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        busy = ovphysx_commit_group_masked(m_handle, write, fetched.group, &refusedMask, noSync());
        busyError = lastError();
    }));
    EXPECT_EQ(busy.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(busy.op_index, 0u);
    EXPECT_EQ(busyError, idleError);

    std::vector<uint8_t> rows(static_cast<size_t>(column.shape[0]), 0);
    for (size_t i = 0; i < rows.size(); i += 2)
        rows[i] = 1;
    int64_t rowCount = static_cast<int64_t>(rows.size());
    DLTensor mask = refusedMask;
    mask.data = rows.data();
    mask.shape = &rowCount;
    ovphysx_enqueue_result_t commit{};
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        commit = ovphysx_commit_group_masked(m_handle, write, fetched.group, &mask, noSync());
    }));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, commit)) << "a refused commit leaves the group live";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));

    const std::vector<float> after = readFloats(m_handle, query, OVPHYSX_ATTR_POSITION);
    ASSERT_EQ(after.size(), before.size());
    size_t changed = 0;
    for (size_t row = 0; row < after.size() / 3; ++row)
    {
        bool rowChanged = false;
        for (size_t c = 0; c < 3; ++c)
            rowChanged = rowChanged || after[row * 3 + c] == 777.0f;
        changed += rowChanged ? 1 : 0;
    }
    EXPECT_EQ(changed, (rows.size() + 1) / 2);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A commit with a CUDA handoff runs on the calling thread after earlier work, while an
// unmasked commit of a host group with nothing outstanding is queued.
TEST_F(WorkerShortCalls, CommitVenueFollowsItsHandoff)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    ovphysx_write_handle_t hostWrite = 0;
    ovphysx_fetched_write_group_t hostGroup{};
    openWrite(query, OVPHYSX_ATTR_LINEAR_VELOCITY, hostWrite, hostGroup);
    const DLTensor& column = hostGroup.group->data.tensors[0];
    std::fill_n(static_cast<float*>(column.data), session_test::detail::elementCount(column), 0.0f);
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t commit = ovphysx_commit_group(m_handle, hostWrite, hostGroup.group, noSync());
        EXPECT_TRUE(queued(m_handle, commit));
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(m_handle, commit));
    }

    // The stream sentinel 1 is the legacy default stream: whatever the commit then reports
    // in this process (without a CUDA driver it spends the group and its release keeps the
    // storage), it must not overtake the earlier job.
    ovphysx_write_handle_t streamWrite = 0;
    ovphysx_fetched_write_group_t streamGroup{};
    openWrite(query, OVPHYSX_ATTR_LINEAR_VELOCITY, streamWrite, streamGroup);
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        ovphysx_commit_group(m_handle, streamWrite, streamGroup.group, ovstage_cuda_sync_t{1, 0});
    }));
    EXPECT_EQ(ovphysx_release_write(m_handle, streamWrite).status, OVPHYSX_API_SUCCESS);
    drainTasks(m_handle);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, hostWrite)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// The articulation update and the SDF evaluation stay barriers: behind earlier work they
// wait for it on the calling thread. The SDF tensors are described on a CUDA device but
// never read: the unknown query is refused after the wait.
TEST_F(WorkerShortCalls, KinematicUpdateAndSdfEvaluationWaitForEarlierWork)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t articulations = openQuery(OVPHYSX_QUERY_TARGET_ARTICULATION);
    ovphysx_enqueue_result_t update{};
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        update = ovphysx_update_articulations_kinematic(m_handle, articulations, OVPHYSX_ARTICULATION_KINEMATIC_POSITION,
                                                        noSync());
    }));
    EXPECT_EQ(update.status, OVPHYSX_API_SUCCESS) << lastError();
    if (update.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_TRUE(waitForTaskSuccess(m_handle, update));
    }

    alignas(16) float storage[4] = {};
    int64_t pointsShape[3] = {1, 1, 3};
    int64_t outShape[3] = {1, 1, 4};
    DLTensor points{};
    points.data = storage;
    points.device = DLDevice{kDLCUDA, 0};
    points.ndim = 3;
    points.dtype = DLDataType{kDLFloat, 32, 1};
    points.shape = pointsShape;
    DLTensor out = points;
    out.shape = outShape;
    ovphysx_enqueue_result_t sdf{};
    EXPECT_TRUE(waitsForEarlierWork(
        m_handle, [&] { sdf = ovphysx_evaluate_sdf_shapes(m_handle, 0, &points, &out, noSync()); }));
    EXPECT_EQ(sdf.status, OVPHYSX_API_NOT_FOUND) << lastError();
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, articulations)));
}

// Unknown handles and a full queue are still rejected at submission, with nothing queued
// and the outputs cleared.
TEST_F(WorkerShortCalls, RejectionsAtSubmissionQueueNothing)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t occupied = ovphysx_submit_test_job_for_test_internal(m_handle, nullptr, 0);
    ASSERT_TRUE(queued(m_handle, occupied));

    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 7;
    const ovphysx_enqueue_result_t unknownQuery =
        ovphysx_read(m_handle, m_handle, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    EXPECT_EQ(unknownQuery.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(unknownQuery.op_index, 0u);
    EXPECT_EQ(read, 0u);
    const ovstage_read_group_t* group = reinterpret_cast<const ovstage_read_group_t*>(&read);
    const ovphysx_enqueue_result_t unknownRead = ovphysx_fetch_read_next(m_handle, m_handle, &group);
    EXPECT_EQ(unknownRead.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(group, nullptr);
    EXPECT_EQ(ovphysx_release_group(m_handle, m_handle, ovstage_read_group_id_t{ 1 }).status, OVPHYSX_API_SUCCESS)
        << "releasing an absent read's group is a successful no-op";

    // Fill the instance's queue (one job is already queued), then a short call is refused.
    uint64_t accepted = 1;
    for (;;)
    {
        const ovphysx_enqueue_result_t job = ovphysx_submit_test_job_for_test_internal(m_handle, nullptr, 0);
        if (job.status != OVPHYSX_API_SUCCESS)
        {
            EXPECT_EQ(job.status, OVPHYSX_API_INVALID_STATE);
            break;
        }
        ++accepted;
        ASSERT_LE(accepted, 1u << 20) << "the queue never filled";
    }
    read = 7;
    const ovphysx_enqueue_result_t full =
        ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    EXPECT_EQ(full.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(full.op_index, 0u);
    EXPECT_EQ(read, 0u);
    EXPECT_NE(lastError().find("queue is full"), std::string::npos) << lastError();
    hold.release();
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &all).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(all.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&all);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A short call made on the instance's own worker thread is still refused before it
// could queue behind the job that makes it.
struct WorkerCallProbe
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_enqueue_result_t query{OVPHYSX_API_SUCCESS, 1, 1};
    ovphysx_query_handle_t queryHandle = 1;
    std::string error;
};

TEST_F(WorkerShortCalls, CallOnTheOwnWorkerIsRefused)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    WorkerCallProbe probe;
    probe.handle = m_handle;
    EXPECT_TRUE(waitForTaskSuccess(m_handle,
                                   ovphysx_submit_test_callback_job_for_test_internal(
                                       m_handle,
                                       [](void* user) {
                                           WorkerCallProbe& probe = *static_cast<WorkerCallProbe*>(user);
                                           probe.query = ovphysx_query(probe.handle, OVPHYSX_QUERY_TARGET_RIGID_BODY,
                                                                       OVPHYSX_SCOPE_ALL, &probe.queryHandle, nullptr);
                                           probe.error = lastError();
                                       },
                                       &probe),
                                   kWorkerRunsIt));
    EXPECT_EQ(probe.query.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.query.op_index, 0u);
    EXPECT_EQ(probe.queryHandle, 0u);
    EXPECT_NE(probe.error.find("own worker thread"), std::string::npos) << probe.error;
}

// A queued short call is polled like any task: a zero-timeout wait reports it pending,
// repeatedly, until the worker published it; it is collected exactly once.
TEST_F(WorkerShortCalls, PollingAQueuedCallCollectsItOnce)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr);
    ASSERT_TRUE(queued(m_handle, opened));
    for (int poll = 0; poll < 3; ++poll)
    {
        ovphysx_wait_task_result_t result{};
        EXPECT_EQ(ovphysx_wait_task(m_handle, opened, OVPHYSX_TIMEOUT_POLL, &result).status, OVPHYSX_API_TIMEOUT);
        EXPECT_EQ(result.pending_task.op_index, opened.op_index);
        ovphysx_destroy_wait_task_result(&result);
    }
    hold.release();
    int successes = 0;
    ovphysx_api_status_t status = OVPHYSX_API_TIMEOUT;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (status == OVPHYSX_API_TIMEOUT && std::chrono::steady_clock::now() < deadline)
    {
        ovphysx_wait_task_result_t result{};
        status = ovphysx_wait_task(m_handle, opened, OVPHYSX_TIMEOUT_POLL, &result).status;
        successes += status == OVPHYSX_API_SUCCESS ? 1 : 0;
        ovphysx_destroy_wait_task_result(&result);
    }
    EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(successes, 1);
    ovphysx_wait_task_result_t again{};
    EXPECT_EQ(ovphysx_wait_task(m_handle, opened, OVPHYSX_TIMEOUT_POLL, &again).status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&again);
    ASSERT_NE(query, 0u);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// Destroying an instance cancels its queued short calls: they never run, so the outputs
// they would write when running stay cleared (handles were written at submission).
TEST_F(WorkerShortCalls, DestroyCancelsQueuedShortCalls)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    ovphysx_instance_handle_t victim = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&victim).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(victim, kBoxes, 1, "short-call-destroy"));
    ASSERT_EQ(ovphysx_step_sync(victim, kDt).status, OVPHYSX_API_SUCCESS);
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        victim, ovphysx_query(victim, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(victim, true));
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_result_t readSummary{};
    ovphysx_read_handle_t read = 0;
    EXPECT_TRUE(queued(victim, ovphysx_read(victim, query, &position, 1,
                                           ovstage_cuda_sync_t{ 0, 0 }, &read, &readSummary)));
    ovstage_query_result_t querySummary{};
    ovphysx_query_handle_t second = 0;
    EXPECT_TRUE(queued(victim, ovphysx_query(victim, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &second,
                                             &querySummary)));
    EXPECT_NE(read, 0u);
    EXPECT_NE(second, 0u);
    EXPECT_EQ(ovphysx_destroy_instance(victim).status, OVPHYSX_API_SUCCESS);
    test_utils::destroy_ovstage_test_stages(victim);
    EXPECT_EQ(readSummary.attribute_count, 0u);
    EXPECT_EQ(querySummary.total_prim_count, 0u);
}

// One error path: a short call that finds a state earlier work set (here a query of each
// kind on an instance with no stage attached) is accepted and fails its task with the same
// text whether it ran on the calling thread of the idle instance or queued behind held
// work. Either way it returns its handle, which names the failed producer.
TEST_F(WorkerShortCalls, StateChecksFailTheTaskWithOneTextOnBothPaths)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    ovphysx_instance_handle_t bare = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&bare).status, OVPHYSX_API_SUCCESS) << lastError();
    struct Destroy
    {
        ovphysx_instance_handle_t handle;
        ~Destroy() { EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS); }
    } destroy{bare};

    const char* const pattern = "/World/Cube1";
    ovphysx_query_selection_t selection{};
    selection.pattern = {pattern, std::strlen(pattern)};
    ovphysx_contact_sensor_selection_t sensors{};
    sensors.sensors = selection;
    const char* const apis[] = {"ovphysx_query", "ovphysx_query_selection", "ovphysx_query_contacts"};
    for (const char* api : apis)
    {
        const auto submit = [&](ovphysx_query_handle_t* out) {
            const std::string name(api);
            if (name == "ovphysx_query")
                return ovphysx_query(bare, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, out, nullptr);
            if (name == "ovphysx_query_selection")
                return ovphysx_query_selection(bare, OVPHYSX_QUERY_TARGET_RIGID_BODY, &selection, out, nullptr);
            return ovphysx_query_contacts(bare, &sensors, 1, 0, out, nullptr);
        };
        const std::string cause = std::string(api) + ": attach an ovstage Stage first";
        ovphysx_query_handle_t idleQuery = 0;
        const ovphysx_enqueue_result_t idle = submit(&idleQuery);
        ASSERT_EQ(idle.status, OVPHYSX_API_SUCCESS) << api << ": " << lastError();
        if (ovphysxIdleCallsRunOnCaller())
        {
            EXPECT_EQ(statusOf(bare, idle), OVPHYSX_TASK_STATUS_FAILED) << api << " ran on the calling thread";
        }
        ovphysx_query_handle_t queuedQuery = 0;
        ovphysx_enqueue_result_t held{};
        {
            WorkerHold hold(bare);
            ASSERT_TRUE(hold.held);
            held = submit(&queuedQuery);
            EXPECT_TRUE(queued(bare, held)) << api;
        }
        EXPECT_NE(idleQuery, 0u) << api;
        EXPECT_NE(queuedQuery, 0u) << api;
        EXPECT_EQ(collectFailure(bare, idle), taskText(api, idle, cause));
        EXPECT_EQ(collectFailure(bare, held), taskText(api, held, cause));
        EXPECT_TRUE(waitForTaskSuccess(bare, ovphysx_release_query(bare, idleQuery)));
        EXPECT_TRUE(waitForTaskSuccess(bare, ovphysx_release_query(bare, queuedQuery)));
    }
}

// A detach closes the write sessions opened before it. Their handles stay known until they
// are released, so a call that names one is accepted whether or not the detach already
// ran, and fails its task with the same text queued behind the detach as on the idle
// instance after it. The release succeeds; after it the handle is unknown.
TEST_F(WorkerShortCalls, ClosedWriteSessionFailsItsCallsWithOneTextOnBothPaths)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    ovphysx_write_handle_t write = 0;
    ovphysx_fetched_write_group_t fetched{};
    openWrite(query, OVPHYSX_ATTR_LINEAR_VELOCITY, write, fetched);

    ovphysx_enqueue_result_t detached{};
    ovphysx_enqueue_result_t queuedFetch{};
    ovphysx_enqueue_result_t queuedCommit{};
    ovphysx_fetched_write_group_t next{};
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        detached = ovphysx_detach_ovstage(m_handle);
        ASSERT_TRUE(queued(m_handle, detached));
        queuedFetch = ovphysx_fetch_write_next(m_handle, write, &next);
        EXPECT_TRUE(queued(m_handle, queuedFetch));
        queuedCommit = ovphysx_commit_group(m_handle, write, fetched.group, noSync());
        EXPECT_TRUE(queued(m_handle, queuedCommit));
    }
    ASSERT_TRUE(waitForTaskSuccess(m_handle, detached));
    const std::string cause = "input write " + std::to_string(write) + " was closed when its stage was detached";
    EXPECT_EQ(collectFailure(m_handle, queuedFetch), taskText("ovphysx_fetch_write_next", queuedFetch, cause));
    EXPECT_EQ(collectFailure(m_handle, queuedCommit), taskText("ovphysx_commit_group", queuedCommit, cause));
    EXPECT_EQ(next.group, nullptr);

    const ovphysx_enqueue_result_t idleFetch = ovphysx_fetch_write_next(m_handle, write, &next);
    ASSERT_EQ(idleFetch.status, OVPHYSX_API_SUCCESS) << lastError();
    if (ovphysxIdleCallsRunOnCaller())
    {
        EXPECT_EQ(statusOf(m_handle, idleFetch), OVPHYSX_TASK_STATUS_FAILED);
    }
    EXPECT_EQ(collectFailure(m_handle, idleFetch), taskText("ovphysx_fetch_write_next", idleFetch, cause));
    const ovphysx_enqueue_result_t idleCommit = ovphysx_commit_group(m_handle, write, fetched.group, noSync());
    ASSERT_EQ(idleCommit.status, OVPHYSX_API_SUCCESS) << lastError();
    EXPECT_EQ(collectFailure(m_handle, idleCommit), taskText("ovphysx_commit_group", idleCommit, cause));
    EXPECT_EQ(next.group, nullptr);

    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));
    const ovphysx_enqueue_result_t afterRelease = ovphysx_fetch_write_next(m_handle, write, &next);
    EXPECT_EQ(afterRelease.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(afterRelease.op_index, 0u);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A release submitted later does not change the outcome of an earlier call: the detach
// queued before both closes the session and keeps it until the release runs, so the
// earlier fetch fails with the closed-session text.
TEST_F(WorkerShortCalls, LaterReleaseLeavesEarlierCallsTheClosedSession)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery();
    ovphysx_write_handle_t write = 0;
    ovphysx_fetched_write_group_t fetched{};
    openWrite(query, OVPHYSX_ATTR_LINEAR_VELOCITY, write, fetched);

    ovphysx_enqueue_result_t detached{};
    ovphysx_enqueue_result_t queuedFetch{};
    ovphysx_enqueue_result_t released{};
    ovphysx_fetched_write_group_t next{};
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        detached = ovphysx_detach_ovstage(m_handle);
        queuedFetch = ovphysx_fetch_write_next(m_handle, write, &next);
        released = ovphysx_release_write(m_handle, write);
        for (const ovphysx_enqueue_result_t& task : {detached, queuedFetch, released})
            EXPECT_TRUE(queued(m_handle, task));
    }
    ASSERT_TRUE(waitForTaskSuccess(m_handle, detached));
    const std::string cause = "input write " + std::to_string(write) + " was closed when its stage was detached";
    EXPECT_EQ(collectFailure(m_handle, queuedFetch), taskText("ovphysx_fetch_write_next", queuedFetch, cause));
    EXPECT_EQ(next.group, nullptr);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, released));
    EXPECT_EQ(ovphysx_fetch_write_next(m_handle, write, &next).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

#if OVPHYSX_ENABLE_GPU_TESTS

// A DirectGPU instance of its own, so a held worker never affects another suite.
class WorkerShortCallsGpuTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t entries[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true")),
        };
        args.config_entries = entries;
        args.config_entry_count = sizeof(entries) / sizeof(entries[0]);
        ASSERT_EQ(ovphysx_create_instance_with_args(&args, &m_handle).status, OVPHYSX_API_SUCCESS) << lastError();
        // The CUDA shim is known once an instance started the runtime.
        if (!ovphysx::test_cuda::cudaAvailable())
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "CUDA is required for the GPU short-call cases";
            GTEST_SKIP() << "CUDA is unavailable";
        }
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", 1,
                                                        "short-call-gpu"));
        ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
        m_context = session_test::cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
        ASSERT_NE(m_context, 0u);
    }
    void TearDown() override
    {
        if (m_handle == OVPHYSX_INVALID_HANDLE)
            return;
        drainTasks(m_handle);
        test_utils::destroy_ovstage_test_attachments(m_handle);
        EXPECT_EQ(ovphysx_destroy_instance(m_handle).status, OVPHYSX_API_SUCCESS);
        test_utils::destroy_ovstage_test_stages(m_handle);
    }

    // The sorted values of a read's device groups.
    std::vector<float> deviceValues(ovphysx_read_handle_t read)
    {
        std::vector<float> values;
        for (;;)
        {
            const ovstage_read_group_t* group = nullptr;
            EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
            if (!group)
                break;
            for (uint32_t t = 0; t < group->columns[0].data.tensor_count; ++t)
            {
                const DLTensor& tensor = group->columns[0].data.tensors[t];
                EXPECT_EQ(tensor.device.device_type, kDLCUDA);
                std::vector<unsigned char> bytes;
                EXPECT_TRUE(session_test::detail::hostBytes(tensor, group->columns[0].data.cuda_sync, m_context,
                                                            bytes));
                const float* data = reinterpret_cast<const float*>(bytes.data());
                values.insert(values.end(), data, data + bytes.size() / sizeof(float));
            }
        }
        std::sort(values.begin(), values.end());
        return values;
    }

    ovphysx_instance_handle_t m_handle = OVPHYSX_INVALID_HANDLE;
    uintptr_t m_context = 0;
};

// A device read with an empty handoff may queue behind held work. A read with a
// producer event waits for earlier work and runs on the caller. Both return the
// same values. Read release remains a short call.
TEST_F(WorkerShortCallsGpuTest, QueuedDeviceReadMatchesTheCallingThreadRead)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t inlineRead = 0;
    const ovphysx_enqueue_result_t inlineOpened =
        ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &inlineRead, nullptr);
    if (ovphysxIdleCallsRunOnCaller())
    {
        ASSERT_TRUE(ranInline(m_handle, inlineOpened));
    }
    ASSERT_TRUE(waitForTaskSuccess(m_handle, inlineOpened));
    const std::vector<float> expected = deviceValues(inlineRead);
    ASSERT_FALSE(expected.empty());

    ovphysx_read_handle_t queuedRead = 0;
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t read =
            ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &queuedRead, nullptr);
        EXPECT_TRUE(queued(m_handle, read));
        hold.release();
        ASSERT_TRUE(waitForTaskSuccess(m_handle, read, kWorkerRunsIt));
    }
    EXPECT_EQ(deviceValues(queuedRead), expected);

    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    uintptr_t event = 0;
    {
        ovphysx::test_cuda::ScopedCudaContextPush context(cuda, m_context);
        ASSERT_TRUE(context.ok());
        ASSERT_TRUE(cuda->eventCreate(&event, 0, nullptr));
        ASSERT_TRUE(cuda->eventRecord(event, 0, nullptr));
    }
    ovphysx_read_handle_t callerRead = 0;
    ovphysx_enqueue_result_t callerReadTask{};
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        callerReadTask = ovphysx_read(
            m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, event }, &callerRead, nullptr);
    }));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, callerReadTask));
    EXPECT_EQ(deviceValues(callerRead), expected);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, callerRead)));
    {
        ovphysx::test_cuda::ScopedCudaContextPush context(cuda, m_context);
        EXPECT_TRUE(cuda->eventDestroy(event, nullptr));
    }

    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t released = ovphysx_release_read(m_handle, queuedRead);
        EXPECT_TRUE(queued(m_handle, released));
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(m_handle, released, kWorkerRunsIt));
    }
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, inlineRead)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A write of device columns opened and fetched behind held work runs on the worker like any
// short call, and so does the release of the session. Its commit is ordered on the caller's
// stream, so it runs on the calling thread after earlier work even with nothing outstanding,
// and the bodies then hold the written positions. A host column of the same scene with
// nothing outstanding is queued; with a CUDA event to wait for it runs on the calling thread.
TEST_F(WorkerShortCallsGpuTest, CommitVenueFollowsDeviceMemoryAndHandoff)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));

    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t deviceWrite = 0;
    ovphysx_fetched_write_group_t deviceGroup{};
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t opened = ovphysx_write(m_handle, query, &position, 1, &deviceWrite, nullptr);
        const ovphysx_enqueue_result_t fetched = ovphysx_fetch_write_next(m_handle, deviceWrite, &deviceGroup);
        EXPECT_TRUE(queued(m_handle, opened));
        EXPECT_TRUE(queued(m_handle, fetched));
        hold.release();
        ASSERT_TRUE(waitForTaskSuccess(m_handle, opened, kWorkerRunsIt));
        ASSERT_TRUE(waitForTaskSuccess(m_handle, fetched, kWorkerRunsIt));
    }
    ASSERT_NE(deviceGroup.group, nullptr);
    const DLTensor& deviceColumn = deviceGroup.group->data.tensors[0];
    ASSERT_EQ(deviceColumn.device.device_type, kDLCUDA);
    std::vector<float> written(session_test::detail::elementCount(deviceColumn), 2.5f);
    std::vector<unsigned char> bytes(written.size() * sizeof(float));
    std::memcpy(bytes.data(), written.data(), bytes.size());
    ASSERT_TRUE(session_test::detail::deviceUpload(deviceColumn, bytes, m_context));
    ovphysx_enqueue_result_t deviceCommit{};
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        deviceCommit = ovphysx_commit_group(m_handle, deviceWrite, deviceGroup.group, noSync());
    }));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, deviceCommit));
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t released = ovphysx_release_write(m_handle, deviceWrite);
        EXPECT_TRUE(queued(m_handle, released));
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(m_handle, released, kWorkerRunsIt));
    }
    ovphysx_read_handle_t read = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        m_handle, ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
    EXPECT_EQ(deviceValues(read), written);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));

    const ovx_token_or_string_t mass = attributeName(OVPHYSX_ATTR_MASS);
    ovphysx_write_handle_t hostWrites[2] = {};
    ovphysx_fetched_write_group_t hostGroups[2] = {};
    for (int i = 0; i < 2; ++i)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, &mass, 1, &hostWrites[i], nullptr)));
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, hostWrites[i], &hostGroups[i])));
        ASSERT_NE(hostGroups[i].group, nullptr);
        const DLTensor& column = hostGroups[i].group->data.tensors[0];
        ASSERT_EQ(column.device.device_type, kDLCPU);
        std::fill_n(static_cast<float*>(column.data), session_test::detail::elementCount(column), 1.0f);
    }
    {
        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t commit =
            ovphysx_commit_group(m_handle, hostWrites[0], hostGroups[0].group, noSync());
        EXPECT_TRUE(queued(m_handle, commit));
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(m_handle, commit));
    }

    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    uintptr_t event = 0;
    {
        ovphysx::test_cuda::ScopedCudaContextPush context(cuda, m_context);
        ASSERT_TRUE(context.ok());
        ASSERT_TRUE(cuda->eventCreate(&event, 0, nullptr));
        ASSERT_TRUE(cuda->eventRecord(event, 0, nullptr));
    }
    ovphysx_enqueue_result_t eventCommit{};
    EXPECT_TRUE(waitsForEarlierWork(m_handle, [&] {
        eventCommit = ovphysx_commit_group(m_handle, hostWrites[1], hostGroups[1].group, ovstage_cuda_sync_t{0, event});
    }));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, eventCommit));
    {
        ovphysx::test_cuda::ScopedCudaContextPush context(cuda, m_context);
        cuda->eventDestroy(event, nullptr);
    }
    for (const ovphysx_write_handle_t write : hostWrites)
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A masked commit always runs on the calling thread, so one the runtime refuses as given
// (here a device row mask longer than the group) is rejected at submission with no task, as
// for a host group. The group stays live.
TEST_F(WorkerShortCallsGpuTest, RefusedDeviceCommitIsRejectedWithoutATask)
{
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_token_or_string_t velocity = attributeName(OVPHYSX_ATTR_LINEAR_VELOCITY);
    ovphysx_write_handle_t write = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr)));
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
    ASSERT_NE(fetched.group, nullptr);
    const DLTensor& column = fetched.group->data.tensors[0];
    ASSERT_EQ(column.device.device_type, kDLCUDA);
    const std::vector<unsigned char> zeros(session_test::detail::elementCount(column) * sizeof(float), 0);
    ASSERT_TRUE(session_test::detail::deviceUpload(column, zeros, m_context));

    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    const size_t rows = static_cast<size_t>(column.shape[0]) + 1;
    uintptr_t maskMemory = 0;
    {
        ovphysx::test_cuda::ScopedCudaContextPush context(cuda, m_context);
        ASSERT_TRUE(context.ok());
        ASSERT_TRUE(cuda->memAlloc(&maskMemory, rows, nullptr));
        const std::vector<uint8_t> ones(rows, 1);
        ASSERT_TRUE(cuda->memcpyHtoD(maskMemory, ones.data(), rows, nullptr));
    }
    int64_t rowCount = static_cast<int64_t>(rows);
    DLTensor mask{};
    mask.data = reinterpret_cast<void*>(maskMemory);
    mask.device = column.device;
    mask.ndim = 1;
    mask.dtype = DLDataType{kDLUInt, 8, 1};
    mask.shape = &rowCount;
    const ovphysx_enqueue_result_t refused =
        ovphysx_commit_group_masked(m_handle, write, fetched.group, &mask, noSync());
    EXPECT_EQ(refused.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(refused.op_index, 0u);
    EXPECT_NE(lastError().find("the row mask is rejected"), std::string::npos) << lastError();
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_commit_group(m_handle, write, fetched.group, noSync())))
        << "a refused commit leaves the group live";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));
    {
        ovphysx::test_cuda::ScopedCudaContextPush context(cuda, m_context);
        EXPECT_TRUE(cuda->memFree(maskMemory, nullptr));
    }
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

#endif // OVPHYSX_ENABLE_GPU_TESTS

} // namespace
