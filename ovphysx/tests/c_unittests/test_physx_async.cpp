// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "AsyncEventManager/AsyncEventManager.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "global_test_environment.h"
#include "test_utilities.h"

using namespace test_utils;

TEST_F(PhysXTestFixture, PhysXAsync_StepAfterOvstageAttach) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(step_result.op_index, 0);
    
    // Poll for completion
    int poll_count = 0;
    const int max_polls = 50; // 5 seconds max
    bool completed = false;
    
    while (poll_count < max_polls) {
        ovphysx_op_wait_result_t wait_result;
        ovphysx_result_t poll_result = ovphysx_wait_op(m_handle, step_result.op_index, 0, &wait_result); // 0 timeout = poll
        
        if (poll_result.status == OVPHYSX_API_SUCCESS) {
            completed = true;
            ovphysx_destroy_wait_result(&wait_result);
            break;
        } else if (poll_result.status == OVPHYSX_API_TIMEOUT) {
            // Still pending, continue polling
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            poll_count++;
        } else {
            break;
        }
    }
    
    EXPECT_TRUE(completed);
    EXPECT_LT(poll_count, max_polls);
    
    // Cleanup handled by fixture TearDown
}

TEST_F(PhysXTestFixture, PhysXAsync_ConsumedStepOpReturnsNotFound) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t first_wait = ovphysx_wait_op(m_handle, step_result.op_index, UINT64_MAX, &wait_result);
    ASSERT_EQ(first_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t second_wait = ovphysx_wait_op(m_handle, step_result.op_index, 0, &wait_result);
    EXPECT_EQ(second_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST_F(PhysXTestFixture, PhysXAsync_ConsumedOpsRemainSingleUseAfterInternalSync) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t consumed_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(consumed_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t first_wait = ovphysx_wait_op(m_handle, consumed_step.op_index, UINT64_MAX, &wait_result);
    ASSERT_EQ(first_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_enqueue_result_t internally_synced_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(internally_synced_step.status, OVPHYSX_API_SUCCESS);
    ovphysx_enqueue_result_t pending_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(pending_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t reused_wait = ovphysx_wait_op(m_handle, consumed_step.op_index, 0, &wait_result);
    EXPECT_EQ(reused_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t internal_wait =
        ovphysx_wait_op(m_handle, internally_synced_step.op_index, UINT64_MAX, &wait_result);
    ASSERT_EQ(internal_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t repeated_internal_wait =
        ovphysx_wait_op(m_handle, internally_synced_step.op_index, 0, &wait_result);
    EXPECT_EQ(repeated_internal_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t pending_wait =
        ovphysx_wait_op(m_handle, pending_step.op_index, UINT64_MAX, &wait_result);
    EXPECT_EQ(pending_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_enqueue_result_t wait_all_consumed_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(wait_all_consumed_step.status, OVPHYSX_API_SUCCESS);
    ovphysx_result_t wait_all =
        ovphysx_wait_op(m_handle, OVPHYSX_OP_INDEX_ALL, UINT64_MAX, &wait_result);
    ASSERT_EQ(wait_all.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_enqueue_result_t later_internally_synced_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(later_internally_synced_step.status, OVPHYSX_API_SUCCESS);
    ovphysx_enqueue_result_t later_pending_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(later_pending_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t reused_wait_all_index =
        ovphysx_wait_op(m_handle, wait_all_consumed_step.op_index, 0, &wait_result);
    EXPECT_EQ(reused_wait_all_index.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t later_pending_wait =
        ovphysx_wait_op(m_handle, later_pending_step.op_index, UINT64_MAX, &wait_result);
    EXPECT_EQ(later_pending_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST_F(PhysXTestFixture, PhysXAsync_WaitAllConsumesInternallySynchronizedIndex) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t detach_result = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(detach_result.status, OVPHYSX_API_SUCCESS);

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t wait_all_result =
        ovphysx_wait_op(m_handle, OVPHYSX_OP_INDEX_ALL, UINT64_MAX, &wait_result);
    ASSERT_EQ(wait_all_result.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t repeated_wait =
        ovphysx_wait_op(m_handle, step_result.op_index, 0, &wait_result);
    EXPECT_EQ(repeated_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST_F(PhysXTestFixture, PhysXAsync_TimeoutConsumesInternallySynchronizedPrefix) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t internally_synced_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(internally_synced_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t detach_result = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(detach_result.status, OVPHYSX_API_SUCCESS);

    async_event_handle_t pending_event = ovphysx::async::AsyncEventManager::create_event();
    ASSERT_NE(pending_event, 0u);
    ovphysx_op_index_t pending_op = ovphysx::async::register_operation(m_handle, pending_event);
    ASSERT_NE(pending_op, 0u);

    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t timeout_result = ovphysx_wait_op(m_handle, pending_op, 0, &wait_result);
    EXPECT_EQ(timeout_result.status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(wait_result.lowest_pending_op_index, pending_op);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx_result_t prefix_wait =
        ovphysx_wait_op(m_handle, internally_synced_step.op_index, 0, &wait_result);
    EXPECT_EQ(prefix_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_result(&wait_result);

    ovphysx::async::AsyncEventManager::complete_event(pending_event, true);
    ovphysx_result_t pending_wait =
        ovphysx_wait_op(m_handle, pending_op, UINT64_MAX, &wait_result);
    EXPECT_EQ(pending_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_result(&wait_result);
}

TEST(PhysXAsync, InvalidHandles) {
    ovphysx_op_wait_result_t wait_result;
    ovphysx_result_t result = ovphysx_wait_op(0, 1, 0, &wait_result); // invalid handle
    EXPECT_EQ(result.status, OVPHYSX_API_NOT_FOUND);
    // Error details available via ovphysx_get_last_error() if needed
}

TEST_F(PhysXTestFixture, PhysXAsync_MultipleEvents) {
    // Use per-test PhysX instance from fixture
    
    // Exercise multiple async step operations across reset/reattach cycles.
    const int num_ops = 3;
    
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    
    bool all_completed = true;
    for (int i = 0; i < num_ops; ++i) {
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));
        ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);
        
        ovphysx_op_wait_result_t wait_result;
        ovphysx_result_t result = ovphysx_wait_op(m_handle, step_result.op_index, 2000000000ULL, &wait_result); // 2 sec
        
        if (result.status != OVPHYSX_API_SUCCESS) {
            all_completed = false;
        }
        
        ovphysx_destroy_wait_result(&wait_result);

        if (!all_completed) {
            break;
        }
        
        ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
        ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS)
            << "Failed to enqueue reset_stage in iteration " << (i + 1);
        ovphysx_result_t reset_wait = ovphysx_wait_op(m_handle, reset_result.op_index, 2000000000ULL, &wait_result);
        ASSERT_EQ(reset_wait.status, OVPHYSX_API_SUCCESS)
            << "reset_stage wait failed in iteration " << (i + 1);
        ASSERT_EQ(wait_result.num_errors, 0u)
            << "reset_stage completed with operation errors in iteration " << (i + 1);
        ovphysx_destroy_wait_result(&wait_result);
        destroy_ovstage_test_attachments(m_handle);
    }
    
    EXPECT_TRUE(all_completed);
    
    // Cleanup handled by fixture TearDown
}
