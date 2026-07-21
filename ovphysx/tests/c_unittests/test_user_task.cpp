// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <gtest/gtest.h>

#include <atomic>

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "global_test_environment.h"

namespace
{
struct UserTaskContext
{
    ovphysx_handle_t expected_handle = 0;
    ovphysx_op_index_t callback_op_index = 0;
    std::atomic<int> callback_count{0};
};

ovphysx_result_t test_user_task_callback(ovphysx_handle_t handle, ovphysx_op_index_t op_index, void* user_data)
{
    auto* ctx = static_cast<UserTaskContext*>(user_data);
    if (!ctx || handle != ctx->expected_handle || op_index == 0)
    {
        return {OVPHYSX_API_ERROR};
    }

    ctx->callback_op_index = op_index;
    ctx->callback_count.fetch_add(1, std::memory_order_relaxed);
    return {OVPHYSX_API_SUCCESS};
}

ovphysx_result_t test_user_task_failure_callback(ovphysx_handle_t handle, ovphysx_op_index_t op_index, void* user_data)
{
    auto* ctx = static_cast<UserTaskContext*>(user_data);
    if (ctx) {
        ctx->callback_op_index = op_index;
        ctx->callback_count.fetch_add(1, std::memory_order_relaxed);
    }
    return {OVPHYSX_API_ERROR};
}
} // namespace

TEST_F(PhysXTestFixture, AddUserTaskSuccess)
{
    UserTaskContext context;
    context.expected_handle = m_handle;

    ovphysx_user_task_desc_t desc{};
    desc.run = test_user_task_callback;
    desc.user_data = &context;

    ovphysx_enqueue_result_t enqueue_result = ovphysx_add_user_task(m_handle, &desc);
    ASSERT_EQ(enqueue_result.status, OVPHYSX_API_SUCCESS);

    EXPECT_NE(enqueue_result.op_index, 0u);
    EXPECT_EQ(context.callback_count.load(std::memory_order_relaxed), 1);
    EXPECT_EQ(context.callback_op_index, enqueue_result.op_index);

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t wait_status = ovphysx_wait_op(m_handle, enqueue_result.op_index, 1000000000ULL, &wait_result);
    EXPECT_EQ(wait_status.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST_F(PhysXTestFixture, AddUserTaskFailurePropagatesWaitErrorText)
{
    UserTaskContext context;
    context.expected_handle = m_handle;

    ovphysx_user_task_desc_t desc{};
    desc.run = test_user_task_failure_callback;
    desc.user_data = &context;

    ovphysx_enqueue_result_t enqueue_result = ovphysx_add_user_task(m_handle, &desc);
    ASSERT_EQ(enqueue_result.status, OVPHYSX_API_ERROR);
    ASSERT_NE(enqueue_result.op_index, 0u);
    ASSERT_EQ(context.callback_count.load(std::memory_order_relaxed), 1);

    // The TLS error should contain a message (set by ovphysx_add_user_task)
    {
        ovphysx_string_t err = ovphysx_get_last_error();
        EXPECT_NE(err.ptr, nullptr);
        EXPECT_GT(err.length, 0u);
    }

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t wait_status = ovphysx_wait_op(m_handle, enqueue_result.op_index, 1000000000ULL, &wait_result);
    EXPECT_EQ(wait_status.status, OVPHYSX_API_ERROR);

    ASSERT_EQ(wait_result.num_errors, 1u);
    ovphysx_string_t op_err = ovphysx_get_last_op_error(wait_result.error_op_indices[0]);
    EXPECT_NE(op_err.ptr, nullptr);
    EXPECT_GT(op_err.length, 0u);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST_F(PhysXTestFixture, StepWaitReportsPriorUserTaskFailure)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t initial_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(initial_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t initial_wait =
        ovphysx_wait_op(m_handle, initial_step.op_index, UINT64_MAX, &wait_result);
    ASSERT_EQ(initial_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    UserTaskContext context;
    context.expected_handle = m_handle;

    ovphysx_user_task_desc_t desc{};
    desc.run = test_user_task_failure_callback;
    desc.user_data = &context;

    ovphysx_enqueue_result_t failed_task = ovphysx_add_user_task(m_handle, &desc);
    ASSERT_EQ(failed_task.status, OVPHYSX_API_ERROR);

    ovphysx_enqueue_result_t later_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(later_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t later_wait =
        ovphysx_wait_op(m_handle, later_step.op_index, UINT64_MAX, &wait_result);
    EXPECT_EQ(later_wait.status, OVPHYSX_API_ERROR);
    ASSERT_EQ(wait_result.num_errors, 1u);
    EXPECT_EQ(wait_result.error_op_indices[0], failed_task.op_index);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t repeated_wait =
        ovphysx_wait_op(m_handle, failed_task.op_index, 0, &wait_result);
    EXPECT_EQ(repeated_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST_F(PhysXTestFixture, AddUserTaskRejectsInvalidArgs)
{
    ovphysx_enqueue_result_t null_desc_result = ovphysx_add_user_task(m_handle, nullptr);
    EXPECT_EQ(null_desc_result.status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_user_task_desc_t null_callback_desc{};
    null_callback_desc.run = nullptr;
    null_callback_desc.user_data = nullptr;

    ovphysx_enqueue_result_t null_run_result = ovphysx_add_user_task(m_handle, &null_callback_desc);
    EXPECT_EQ(null_run_result.status, OVPHYSX_API_INVALID_ARGUMENT);
}
