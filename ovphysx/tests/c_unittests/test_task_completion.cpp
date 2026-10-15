// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-STRING-001
 * @covers AC-2
 * @implements REQ-CAPI-TASK-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

#include <gtest/gtest.h>

#include <string>
#include <atomic>
#include <cstring>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "ovphysx/experimental/ovphysx.hpp"
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"

namespace
{
// Use real inline producers so collection tests also exercise admission.
ovphysx_enqueue_result_t completed_task(ovphysx_instance_handle_t handle, bool fail,
    const char* missing_source = "/World/TaskMissingSource")
{
    if (!fail)
        return ovphysx_release_query(handle, 0);
    bool attach = false;
    if (ovphysx_is_stage_attached(handle, nullptr, &attach).status != OVPHYSX_API_SUCCESS)
        return {OVPHYSX_API_ERROR, 0, 0};
    if (!attach && !test_utils::attach_usd_with_ovstage(
            handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"))
        return {OVPHYSX_API_ERROR, 0, 0};
    static std::atomic<unsigned> sequence{0};
    const std::string path = "/World/TaskCloneTarget" + std::to_string(++sequence);
    ovphysx_string_t target{path.c_str(), path.size()};
    return ovphysx_clone(handle, {missing_source, std::strlen(missing_source)}, &target, 1, nullptr, nullptr);
}

ovphysx_enqueue_result_t enqueue_missing_source_clone(ovphysx_instance_handle_t handle)
{
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/TaskCloneTarget");
    return ovphysx_clone(handle, OVPHYSX_LITERAL("/World/TaskMissingSource"), &target, 1, nullptr, nullptr);
}

std::string task_error(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    const ovphysx_string_t error = ovphysx_get_last_task_error(handle, task);
    return std::string(error.ptr, error.length);
}
} // namespace

TEST_F(PhysXTestFixture, TaskWaitReportsOnlyItsTargetAndWaitAllRetainsPriorFailure)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));
    const ovphysx_enqueue_result_t failed = enqueue_missing_source_clone(m_handle);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t succeeded = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(succeeded.status, OVPHYSX_API_SUCCESS);

    ovphysx::physx::WaitResult target;
    EXPECT_EQ(target.waitTask(m_handle, succeeded), OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(target.hasErrors());

    ovphysx::physx::WaitResult all;
    ASSERT_EQ(all.waitAll(m_handle), OVPHYSX_API_ERROR);
    ASSERT_EQ(all.errorCount(), 1u);
    EXPECT_EQ(all.errorTaskAt(0).op_index, failed.op_index);
    EXPECT_EQ(all.errorTaskAt(0).instance, m_handle);
    EXPECT_FALSE(task_error(m_handle, all.errorTaskAt(0)).empty());

    ovphysx::physx::WaitResult repeated;
    EXPECT_EQ(repeated.waitAll(m_handle), OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(repeated.hasErrors());
    EXPECT_EQ(repeated.waitTask(m_handle, failed, 0), OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(repeated.waitTask(m_handle, succeeded, 0), OVPHYSX_API_NOT_FOUND);
}

// @implements REQ-CAPI-DETACH-001
// @covers AC-1 AC-3
TEST_F(PhysXTestFixture, TaskDetachReturnsAcceptedIdentityWhenAlreadyDetached)
{
    const ovphysx_enqueue_result_t first = ovphysx_detach_ovstage(m_handle);
    const ovphysx_enqueue_result_t second = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(second.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(first.instance, m_handle);
    EXPECT_EQ(second.instance, m_handle);
    EXPECT_NE(first.op_index, 0u);
    EXPECT_GT(second.op_index, first.op_index);
    bool attached = true;
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attached);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, second));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, first));
    ovphysx::physx::WaitResult collected;
    EXPECT_EQ(collected.waitTask(m_handle, first, 0), OVPHYSX_API_NOT_FOUND);

    const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, second);
    ASSERT_NE(error.ptr, nullptr);
    EXPECT_EQ(error.length, 0u);
    EXPECT_EQ(error.ptr[error.length], '\0');
}

// @implements REQ-CAPI-OVSTAGE-ATTACH-001
// @covers AC-5
// @implements REQ-CAPI-OVSTAGE-UPDATE-001
// @covers AC-3
// @implements REQ-CAPI-DETACH-001
// @covers AC-5
TEST_F(PhysXTestFixture, LifecycleInvalidArgumentsAndMissingAttachmentRejectWithoutTaskIdentity)
{
    const ovstage_ordinal_range_t range{1, 1, true};
    const ovstage_ordinal_range_t reversed{2, 1, true};
    const ovphysx_enqueue_result_t rejected[] = {
        ovphysx_attach_ovstage(m_handle, nullptr, 1),
        ovphysx_update_from_ovstage(m_handle, reversed),
        ovphysx_update_from_ovstage(m_handle, range),
        ovphysx_update_from_ovstage(0, range),
        ovphysx_detach_ovstage(0),
    };
    for (const ovphysx_enqueue_result_t task : rejected)
    {
        EXPECT_NE(task.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(task.op_index, 0u);
        EXPECT_EQ(task.instance, 0u);
    }
    ovphysx::physx::WaitResult all;
    EXPECT_EQ(all.waitAll(m_handle), OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(all.hasErrors());
}

TEST_F(PhysXTestFixture, TaskWaitAllFindsDiscardedReceiptsInSubmissionOrder)
{
    const ovphysx_op_index_t first = completed_task(m_handle, true).op_index;
    completed_task(m_handle, false);
    const ovphysx_op_index_t second = completed_task(m_handle, true).op_index;
    ASSERT_NE(first, 0u);
    ASSERT_GT(second, first);
    ovphysx::physx::WaitResult result;
    ASSERT_EQ(result.waitAll(m_handle), OVPHYSX_API_ERROR);
    ASSERT_EQ(result.errorCount(), 2u);
    EXPECT_EQ(result.errorTaskAt(0).op_index, first);
    EXPECT_EQ(result.errorTaskAt(1).op_index, second);
    EXPECT_FALSE(task_error(m_handle, result.errorTaskAt(0)).empty());
    EXPECT_FALSE(task_error(m_handle, result.errorTaskAt(1)).empty());
}

TEST_F(PhysXTestFixture, TaskOutOfOrderCollectionPreservesEarlierOutcomesAndErrorText)
{
    const ovphysx_enqueue_result_t first = completed_task(m_handle, true);
    const ovphysx_enqueue_result_t middle = completed_task(m_handle, false);
    const ovphysx_enqueue_result_t last = completed_task(m_handle, true);
    ovphysx::physx::WaitResult last_result;
    ASSERT_EQ(last_result.waitTask(m_handle, last), OVPHYSX_API_ERROR);
    ASSERT_EQ(last_result.errorCount(), 1u);
    const ovphysx_string_t borrowed = ovphysx_get_last_task_error(m_handle, last);
    ASSERT_GT(borrowed.length, 0u);
    const std::string expected(borrowed.ptr, borrowed.length);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, middle));
    ovphysx::physx::WaitResult first_result;
    ASSERT_EQ(first_result.waitTask(m_handle, first), OVPHYSX_API_ERROR);
    EXPECT_EQ(std::string(borrowed.ptr, borrowed.length), expected);
    EXPECT_EQ(task_error(m_handle, last), expected);
    ovphysx::physx::WaitResult all;
    EXPECT_EQ(all.waitAll(m_handle), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(std::string(borrowed.ptr, borrowed.length), expected);
}

TEST_F(PhysXTestFixture, TaskErrorLifetimeEndsAtResultDestruction)
{
    const ovphysx_enqueue_result_t task = completed_task(m_handle, true);
    EXPECT_TRUE(task_error(m_handle, task).empty());
    {
        ovphysx::physx::WaitResult result;
        ASSERT_EQ(result.waitTask(m_handle, task), OVPHYSX_API_ERROR);
        EXPECT_FALSE(task_error(m_handle, task).empty());
    }
    EXPECT_TRUE(task_error(m_handle, task).empty());
}

TEST_F(PhysXTestFixture, TaskErrorTextAndResultDestructionSurviveInstanceTeardown)
{
    ovphysx_instance_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ovphysx::PhysX instance(handle);
    const ovphysx_enqueue_result_t task = completed_task(handle, true);
    ovphysx::physx::WaitResult result = instance.waitTask(task);
    ASSERT_EQ(result.status(), OVPHYSX_API_ERROR);
    const ovphysx_string_t borrowed = ovphysx_get_last_task_error(handle, task);
    ASSERT_GT(borrowed.length, 0u);
    const std::string expected(borrowed.ptr, borrowed.length);
    test_utils::destroy_ovstage_test_attachments(handle);
    instance.reset();
    EXPECT_EQ(std::string(borrowed.ptr, borrowed.length), expected);
    EXPECT_EQ(ovphysx_get_last_task_error(handle, task).length, 0u);
    EXPECT_GT(ovphysx_get_last_error().length, 0u);
    ovphysx::physx::WaitResult invalid;
    EXPECT_EQ(invalid.waitTask(handle, task), OVPHYSX_API_NOT_FOUND);
}

TEST_F(PhysXTestFixture, TaskWaitAllArrayOwnershipSurvivesGrowthMoveAndTeardown)
{
    ovphysx_instance_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ovphysx::PhysX instance(handle);
    constexpr size_t count = 32;
    std::vector<ovphysx_enqueue_result_t> failed;
    std::vector<std::string> missing_sources;
    for (size_t index = 0; index < count; ++index) {
        missing_sources.push_back("/World/TaskMissingSourceWithDistinctLongNameForRetainedDiagnostic_" +
            std::to_string(index));
        failed.push_back(completed_task(handle, true, missing_sources.back().c_str()));
        ASSERT_EQ(failed.back().status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(failed.back().op_index, 0u);
        completed_task(handle, false);
    }

    ovphysx::physx::WaitResult original = instance.waitAll();
    ASSERT_EQ(original.status(), OVPHYSX_API_ERROR);
    ASSERT_EQ(original.errorCount(), count);
    const ovphysx_enqueue_result_t* const receipts = original.get()->error_tasks;
    std::vector<ovphysx_string_t> messages;
    std::vector<std::string> expected;
    for (size_t index = 0; index < count; ++index) {
        EXPECT_EQ(receipts[index].status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(receipts[index].instance, handle);
        EXPECT_EQ(receipts[index].op_index, failed[index].op_index);
        messages.push_back(ovphysx_get_last_task_error(handle, receipts[index]));
        ASSERT_GT(messages.back().length, 0u);
        expected.emplace_back(messages.back().ptr, messages.back().length);
        EXPECT_NE(expected.back().find(missing_sources[index]), std::string::npos);
    }

    ovphysx::physx::WaitResult moved(std::move(original));
    EXPECT_EQ(original.waitAll(handle), OVPHYSX_API_SUCCESS);
    EXPECT_EQ(original.get()->error_tasks, nullptr);
    test_utils::destroy_ovstage_test_attachments(handle);
    instance.reset();
    EXPECT_EQ(moved.get()->error_tasks, receipts);
    for (size_t index = 0; index < count; ++index) {
        EXPECT_EQ(receipts[index].op_index, failed[index].op_index);
        EXPECT_EQ(std::string(messages[index].ptr, messages[index].length), expected[index]);
    }
    moved = ovphysx::physx::WaitResult{};
    EXPECT_EQ(moved.get()->error_tasks, nullptr);
    EXPECT_EQ(moved.errorCount(), 0u);
}

TEST_F(PhysXTestFixture, TaskWaitRejectsInvalidReceiptsAndPreservesOwnedOutput)
{
    const ovphysx_enqueue_result_t task = completed_task(m_handle, true);
    ovphysx_wait_task_result_t result{};
    EXPECT_EQ(ovphysx_wait_task(m_handle, task, 0, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_wait_task(m_handle, {}, 0, &result).status, OVPHYSX_API_INVALID_ARGUMENT);
    ovphysx_enqueue_result_t wrong = task;
    wrong.instance = m_handle + 1;
    EXPECT_EQ(ovphysx_wait_task(m_handle, wrong, 0, &result).status, OVPHYSX_API_INVALID_ARGUMENT);
    wrong = task;
    wrong.status = OVPHYSX_API_ERROR;
    EXPECT_EQ(ovphysx_wait_task(m_handle, wrong, 0, &result).status, OVPHYSX_API_INVALID_ARGUMENT);
    wrong = task;
    wrong.op_index = UINT64_MAX;
    EXPECT_EQ(ovphysx_wait_task(m_handle, wrong, 0, &result).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_wait_task(0, {}, 0, nullptr).status, OVPHYSX_API_NOT_FOUND);
    ASSERT_EQ(ovphysx_wait_task(m_handle, task, 0, &result).status, OVPHYSX_API_ERROR);
    const ovphysx_enqueue_result_t* const storage = result.error_tasks;
    ASSERT_NE(storage, nullptr);
    EXPECT_EQ(ovphysx_wait_all(m_handle, 0, &result).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(result.error_tasks, storage);
    EXPECT_EQ(result.num_errors, 1u);
    EXPECT_FALSE(task_error(m_handle, task).empty());
    ovphysx_destroy_wait_task_result(&result);
    EXPECT_EQ(result.error_tasks, nullptr);
    EXPECT_EQ(result.num_errors, 0u);
    EXPECT_EQ(result.pending_task.op_index, 0u);
    EXPECT_EQ(result.pending_task.instance, 0u);
    ovphysx_destroy_wait_task_result(&result);
    ovphysx_destroy_wait_task_result(nullptr);
}

TEST_F(PhysXTestFixture, TaskReceiptFromAnotherLiveInstanceIsRejected)
{
    ovphysx_instance_handle_t other_handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&other_handle).status, OVPHYSX_API_SUCCESS);
    ovphysx::PhysX other(other_handle);
    const ovphysx_enqueue_result_t task = ovphysx_detach_ovstage(other_handle);
    ovphysx::physx::WaitResult result;
    EXPECT_EQ(result.waitTask(m_handle, task), OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_get_last_task_error(m_handle, task).length, 0u);
    EXPECT_GT(ovphysx_get_last_error().length, 0u);
    EXPECT_EQ(result.waitTask(other_handle, task), OVPHYSX_API_SUCCESS);
}

TEST_F(PhysXTestFixture, TaskInlineFailureIsAcceptedButInvalidSubmissionHasNoIdentity)
{
    const ovphysx_enqueue_result_t rejected = ovphysx_clone(m_handle, {}, nullptr, 0, nullptr, nullptr);
    EXPECT_EQ(rejected.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(rejected.op_index, 0u);
    EXPECT_EQ(rejected.instance, 0u);
    EXPECT_GT(ovphysx_get_last_error().length, 0u);

    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));
    const ovphysx_enqueue_result_t accepted = enqueue_missing_source_clone(m_handle);
    ASSERT_EQ(accepted.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(accepted.instance, m_handle);
    EXPECT_NE(accepted.op_index, 0u);
    ovphysx::physx::WaitResult result;
    EXPECT_EQ(result.waitTask(m_handle, accepted), OVPHYSX_API_ERROR);
    EXPECT_FALSE(task_error(m_handle, accepted).empty());
}


TEST_F(PhysXTestFixture, StepWaitSucceedsWhilePriorCloneFailureRemainsUncollected)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));
    const ovphysx_enqueue_result_t failed = enqueue_missing_source_clone(m_handle);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, step));
    ovphysx::physx::WaitResult all;
    ASSERT_EQ(all.waitAll(m_handle), OVPHYSX_API_ERROR);
    ASSERT_EQ(all.errorCount(), 1u);
    EXPECT_EQ(all.errorTaskAt(0).op_index, failed.op_index);
}


TEST_F(PhysXTestFixture, CppTaskWaitResultMovesStatusAndOwnership)
{
    static_assert(!std::is_copy_constructible<ovphysx::physx::WaitResult>::value, "Result owns its storage");
    static_assert(!std::is_copy_assignable<ovphysx::physx::WaitResult>::value, "Result owns its storage");
    const ovphysx_enqueue_result_t task = completed_task(m_handle, true);
    ovphysx::physx::WaitResult first;
    ASSERT_EQ(first.waitTask(m_handle, task), OVPHYSX_API_ERROR);
    ovphysx::physx::WaitResult second(std::move(first));
    EXPECT_EQ(first.errorCount(), 0u);
    EXPECT_EQ(second.status(), OVPHYSX_API_ERROR);
    EXPECT_EQ(second.errorTaskAt(0).op_index, task.op_index);
    ovphysx::physx::WaitResult third;
    third = std::move(second);
    EXPECT_EQ(second.errorCount(), 0u);
    EXPECT_EQ(third.status(), OVPHYSX_API_ERROR);
    EXPECT_FALSE(task_error(m_handle, task).empty());
    ovphysx::PhysX invalid;
    EXPECT_EQ(invalid.waitAll().status(), OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(invalid.waitTask(task).status(), OVPHYSX_API_NOT_FOUND);
}

TEST_F(PhysXTestFixture, TaskCloneFailureIsCollectedWithLoggingDisabled)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));
    const uint32_t previous_level = ovphysx_get_log_level();
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_NONE).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_clone(m_handle, {}, nullptr, 0, nullptr, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    const ovphysx_enqueue_result_t task = enqueue_missing_source_clone(m_handle);
    ovphysx::physx::WaitResult result;
    const ovphysx_api_status_t status = result.waitTask(m_handle, task);
    const std::string error = task_error(m_handle, task);
    EXPECT_EQ(ovphysx_set_log_level(previous_level).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(task.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(task.op_index, 0u);
    EXPECT_EQ(status, OVPHYSX_API_ERROR);
    EXPECT_NE(error.find("/World/TaskMissingSource"), std::string::npos);
}

TEST_F(PhysXTestFixture, TaskWaitAndErrorLookupMayMoveAcrossSerializedThreads)
{
    const ovphysx_enqueue_result_t task = completed_task(m_handle, true);
    ovphysx::physx::WaitResult result;
    std::thread worker([this, task, &result] {
        result.waitTask(m_handle, task);
    });
    worker.join();
    ASSERT_EQ(result.status(), OVPHYSX_API_ERROR);
    ASSERT_EQ(result.errorCount(), 1u);
    EXPECT_FALSE(task_error(m_handle, result.errorTaskAt(0)).empty());
    ovphysx::physx::WaitResult all;
    EXPECT_EQ(all.waitAll(m_handle), OVPHYSX_API_SUCCESS);
}

TEST_F(PhysXTestFixture, TaskSparseCollectionKeepsHolesAndCompactsAtWaitAll)
{
    std::vector<ovphysx_enqueue_result_t> tasks;
    for (size_t index = 0; index < 4096; ++index)
        tasks.push_back(ovphysx_release_query(m_handle, 0));
    for (size_t index = 1; index < tasks.size(); index += 2)
        ASSERT_TRUE(waitForTaskSuccess(m_handle, tasks[index]));
    // Filling holes merges neighboring ranges without losing their single-use state.
    for (size_t index = 0; index < tasks.size() / 2; index += 2)
        ASSERT_TRUE(waitForTaskSuccess(m_handle, tasks[index]));
    ovphysx::physx::WaitResult all;
    ASSERT_EQ(all.waitAll(m_handle), OVPHYSX_API_SUCCESS);
    for (const auto task : tasks) {
        ovphysx::physx::WaitResult repeated;
        EXPECT_EQ(repeated.waitTask(m_handle, task, 0), OVPHYSX_API_NOT_FOUND);
    }
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, 0)));
}

// @implements REQ-CAPI-DETACH-001
// @covers AC-3
TEST_F(PhysXTestFixture, AttachmentStateHasNoRuntimeIdentityAndRejectsInvalidArguments)
{
    bool attached = true;
    EXPECT_EQ(ovphysx_is_stage_attached(0, nullptr, &attached).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_FALSE(attached);
    EXPECT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attached);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attached);
    EXPECT_EQ(ovphysx_is_stage_attached(m_handle, reinterpret_cast<const void*>(1), &attached).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attached); // Comparison only: the candidate is never dereferenced.
    const ovphysx_enqueue_result_t failed = enqueue_missing_source_clone(m_handle);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attached);
    // Draining for teardown preserves the uncollected task's execution error.
    ovphysx::physx::WaitResult result;
    EXPECT_EQ(result.waitTask(m_handle, failed), OVPHYSX_API_ERROR);
    EXPECT_TRUE(result.hasErrors());
}
