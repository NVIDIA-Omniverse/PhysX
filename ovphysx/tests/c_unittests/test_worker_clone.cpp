// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-6
 *
 * TEST-CAPI-ASYNC-004: ovphysx_clone and ovphysx_update_from_ovstage queue one job each
 * on the instance worker, and a clone owns copies of its inputs once it returns.
 */

// Clone and update as worker jobs: both return while the worker is held, a clone reads
// only the copies of its inputs made at submission, checks that depend on earlier work
// (attachment, warmup or first step) fail the task with their former rejection text in
// submission order, and a failed job does not stop the valid work queued behind it.

#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "session_test_helpers.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

namespace
{

constexpr const char* kEnvs = "tests/data/basic_simulation.usda";
constexpr const char* kTables = "/World/envs/env*/table";
constexpr float kDt = 1.0f / 60.0f;

class WorkerHold
{
public:
    explicit WorkerHold(ovphysx_instance_handle_t handle, bool hold = true) : m_handle(handle)
    {
        held = hold && ovphysx_set_worker_hold_for_test_internal(m_handle, true);
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

// A heap copy of text without a terminator, for an input the test frees after the call.
std::unique_ptr<char[]> heapText(const std::string& text)
{
    std::unique_ptr<char[]> copy(new char[text.size()]);
    std::memcpy(copy.get(), text.data(), text.size());
    return copy;
}

// The world position of every table body, keyed by body path.
session_test::Column tablePositions(ovphysx_instance_handle_t handle)
{
    session_test::Column positions;
    EXPECT_TRUE(
        session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, positions, kTables));
    return positions;
}

class WorkerClone : public PhysXTestFixture
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

    void loadScene() { ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kEnvs)); }
};

class WorkerUpdate : public WorkerClone
{
};

// Every input lives on the heap. Before the held worker runs the job, the test overwrites
// each one with a value the clone would reject or misplace (an invalid path, an empty
// target array, far-away poses, environment ids the runtime refuses) and frees it. The
// clone still succeeds at the poses passed at submission.
TEST_F(WorkerClone, OwnsInputsAfterSubmit)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);

    const std::string sourcePath = "/World/envs/env0";
    const std::string targetPaths[2] = { "/World/envs/env1", "/World/envs/env2" };
    std::unique_ptr<char[]> source = heapText(sourcePath);
    std::unique_ptr<char[]> targetText[2] = { heapText(targetPaths[0]), heapText(targetPaths[1]) };
    std::unique_ptr<ovphysx_string_t[]> targets(new ovphysx_string_t[2]);
    for (int i = 0; i < 2; ++i)
        targets[i] = { targetText[i].get(), targetPaths[i].size() };
    // env1 at (5, 0, 0) and env2 at (10, 0, 0), identity rotation.
    std::unique_ptr<float[]> anchors(new float[14]{ 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                                    10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f });
    // Arbitrary logical ids, deliberately non-positional.
    std::unique_ptr<uint32_t[]> envIds(new uint32_t[2]{ 7u, 3u });

    const ovphysx_enqueue_result_t clone = ovphysx_clone(m_handle, { source.get(), sourcePath.size() }, targets.get(), 2,
                                                         anchors.get(), envIds.get());
    ASSERT_EQ(clone.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(clone.op_index, 0u);
    EXPECT_EQ(clone.instance, m_handle);
    EXPECT_EQ(statusOf(m_handle, clone), OVPHYSX_TASK_STATUS_PENDING);

    std::memset(source.get(), '#', sourcePath.size());
    for (int i = 0; i < 2; ++i)
    {
        std::memset(targetText[i].get(), '#', targetPaths[i].size());
        targets[i] = { nullptr, 0 };
    }
    std::fill(anchors.get(), anchors.get() + 14, 1.0e6f);
    std::fill(envIds.get(), envIds.get() + 2, UINT32_MAX);
    source.reset();
    for (std::unique_ptr<char[]>& text : targetText)
        text.reset();
    targets.reset();
    anchors.reset();
    envIds.reset();

    hold.release();
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone));

    // Each copy sits at its anchor relative to the source, as the bodies all move alike.
    const session_test::Column positions = tablePositions(m_handle);
    ASSERT_EQ(positions.size(), 3u);
    const auto source0 = positions.find("/World/envs/env0/table");
    const auto copy1 = positions.find("/World/envs/env1/table");
    const auto copy2 = positions.find("/World/envs/env2/table");
    ASSERT_TRUE(source0 != positions.end() && copy1 != positions.end() && copy2 != positions.end());
    ASSERT_EQ(source0->second.size(), 3u);
    ASSERT_EQ(copy1->second.size(), 3u);
    ASSERT_EQ(copy2->second.size(), 3u);
    const double tolerance = 0.05;
    EXPECT_NEAR(copy1->second[0] - source0->second[0], 5.0, tolerance);
    EXPECT_NEAR(copy2->second[0] - source0->second[0], 10.0, tolerance);
    for (int axis = 1; axis < 3; ++axis)
    {
        EXPECT_NEAR(copy1->second[axis], source0->second[axis], tolerance);
        EXPECT_NEAR(copy2->second[axis], source0->second[axis], tolerance);
    }

    // The copies simulate with the environment ids of the call. This CPU scene does not
    // filter collisions by id, but the runtime fails a clone whose ids are out of range.
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_step(m_handle, kDt)));
    EXPECT_EQ(tablePositions(m_handle).size(), 3u);
}

// A clone submitted right after an unwaited step is accepted and fails its task, because
// the step ran first: with the worker free to start the step at once, and behind a held
// worker.
TEST_F(WorkerClone, AfterUnwaitedStepFailsAsTask)
{
    for (const bool heldWorker : { false, true })
    {
        SCOPED_TRACE(heldWorker ? "step queued behind a held worker" : "step on an idle instance");
        loadScene();
        {
            WorkerHold hold(m_handle, heldWorker);
            ASSERT_EQ(hold.held, heldWorker);
            const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, kDt);
            ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
            ovphysx_string_t target = OVPHYSX_LITERAL("/World/envs/env1");
            const ovphysx_enqueue_result_t clone =
                ovphysx_clone(m_handle, OVPHYSX_LITERAL("/World/envs/env0"), &target, 1, nullptr, nullptr);
            ASSERT_EQ(clone.status, OVPHYSX_API_SUCCESS) << "the after-step check belongs to the clone job";
            EXPECT_GT(clone.op_index, step.op_index);
            hold.release();
            EXPECT_TRUE(
                waitForTaskFailure(m_handle, clone, "clone() must be called before warmup() and the first step()"));
            EXPECT_TRUE(waitForTaskSuccess(m_handle, step));
        }
        // The failed clone created nothing.
        const session_test::Column positions = tablePositions(m_handle);
        EXPECT_EQ(positions.count("/World/envs/env0/table"), 1u);
        EXPECT_EQ(positions.count("/World/envs/env1/table"), 0u);
        ASSERT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
    }
}

// A clone of a missing source fails, and the valid clone queued right after it still
// runs: wait_all reports exactly the first failure.
TEST_F(WorkerClone, RecoverableFailureThenValidWork)
{
    loadScene();
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    ovphysx_string_t orphan = OVPHYSX_LITERAL("/World/envs/envOfMissingSource");
    const ovphysx_enqueue_result_t failed =
        ovphysx_clone(m_handle, OVPHYSX_LITERAL("/World/envs/missingSource"), &orphan, 1, nullptr, nullptr);
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/envs/env1");
    const ovphysx_enqueue_result_t valid =
        ovphysx_clone(m_handle, OVPHYSX_LITERAL("/World/envs/env0"), &target, 1, nullptr, nullptr);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(valid.status, OVPHYSX_API_SUCCESS);
    EXPECT_LT(failed.op_index, valid.op_index);
    hold.release();

    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &all).status, OVPHYSX_API_ERROR);
    ASSERT_EQ(all.num_errors, 1u);
    EXPECT_EQ(all.error_tasks[0].op_index, failed.op_index);
    const ovphysx_string_t message = ovphysx_get_last_task_error(m_handle, failed);
    const std::string error(message.ptr ? message.ptr : "", message.length);
    EXPECT_NE(error.find("/World/envs/missingSource"), std::string::npos) << error;
    ovphysx_destroy_wait_task_result(&all);
    EXPECT_EQ(statusOf(m_handle, valid), OVPHYSX_TASK_STATUS_NOT_FOUND) << "wait_all collects the valid clone too";

    const session_test::Column positions = tablePositions(m_handle);
    EXPECT_EQ(positions.count("/World/envs/env1/table"), 1u);
    EXPECT_EQ(positions.count("/World/envs/envOfMissingSource/table"), 0u);
}

// An update queues even without an attachment: it is pending while the worker is held,
// and its task fails with the former rejection text once the job runs. Argument errors
// are still rejected at submission.
TEST_F(WorkerUpdate, ReturnsWhilePendingAndFailsWithoutAttach)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    bool attached = true;
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    ASSERT_FALSE(attached);
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);

    const ovphysx_enqueue_result_t update = ovphysx_update_from_ovstage(m_handle, ovstage_ordinal_range_t{ 1, 1, true });
    ASSERT_EQ(update.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(update.op_index, 0u);
    EXPECT_EQ(update.instance, m_handle);
    EXPECT_EQ(statusOf(m_handle, update), OVPHYSX_TASK_STATUS_PENDING);

    const ovphysx_enqueue_result_t reversed =
        ovphysx_update_from_ovstage(m_handle, ovstage_ordinal_range_t{ 2, 1, true });
    EXPECT_EQ(reversed.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(reversed.op_index, 0u);
    EXPECT_EQ(reversed.instance, 0u);

    hold.release();
    EXPECT_TRUE(waitForTaskFailure(m_handle, update, "update_from_ovstage: no ovstage is attached"));
}

} // namespace
