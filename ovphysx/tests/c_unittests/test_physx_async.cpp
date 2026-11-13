// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-3 AC-4 AC-5
 * @implements REQ-CAPI-TASK-001
 * @covers AC-2 AC-3 AC-8
 */

#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <task/PxCpuDispatcher.h>
#include "ovphysx/experimental/Helpers.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "gated_simulation.h"
#include "global_test_environment.h"
#include "test_utilities.h"

using namespace test_utils;

namespace {

struct PollOutcome
{
    ovphysx_api_status_t status = OVPHYSX_API_ERROR;
    ovphysx_op_index_t lowest_pending_op_index = 0;
    size_t error_count = 0;
};

PollOutcome wait_with_timeout(
    ovphysx_instance_handle_t handle,
    ovphysx_enqueue_result_t task,
    ovphysx_timeout_t timeout) noexcept
{
    ovphysx_wait_task_result_t wait_result{};
    const ovphysx_result_t result = ovphysx_wait_task(handle, task, timeout, &wait_result);
    const PollOutcome outcome{
        result.status,
        wait_result.pending_task.op_index,
        wait_result.num_errors};
    ovphysx_destroy_wait_task_result(&wait_result);
    return outcome;
}

} // namespace

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
        ovphysx_wait_task_result_t wait_result{};
        ovphysx_result_t poll_result = ovphysx_wait_task(m_handle, step_result, 0, &wait_result); // 0 timeout = poll
        
        if (poll_result.status == OVPHYSX_API_SUCCESS) {
            completed = true;
            ovphysx_destroy_wait_task_result(&wait_result);
            break;
        } else if (poll_result.status == OVPHYSX_API_TIMEOUT) {
            ovphysx_destroy_wait_task_result(&wait_result);
            // Still pending, continue polling
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            poll_count++;
        } else {
            ovphysx_destroy_wait_task_result(&wait_result);
            break;
        }
    }
    
    EXPECT_TRUE(completed);
    EXPECT_LT(poll_count, max_polls);
    
    // Cleanup handled by fixture TearDown
}

TEST_F(PhysXTestFixture, PhysXAsync_PendingSimulationTimeoutsDoNotBlockOrConsume)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/boxes_falling_on_groundplane.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_string_t target = OVPHYSX_LITERAL("/World/AbsentCloneTarget");
    const auto failed = ovphysx_clone(m_handle, OVPHYSX_LITERAL("/World/AbsentSource"),
                                     &target, 1, nullptr, nullptr);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t initial_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(initial_step.status, OVPHYSX_API_SUCCESS);
    // step_sync waits for the step job without collecting it, so the first step stays
    // available for collection.
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    void* scene_pointer = test_utils::read_physx_pointer(
        m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene");
    ASSERT_NE(scene_pointer, nullptr);

    physx::PxScene* const scene = static_cast<physx::PxScene*>(scene_pointer);
    physx::PxCpuDispatcher* const dispatcher = scene->getCpuDispatcher();
    ASSERT_NE(dispatcher, nullptr);
    if (dispatcher->getWorkerCount() == 0)
        GTEST_SKIP() << "CPU dispatcher has no worker thread for an asynchronous onAdvance callback";

    void* actor_pointer = test_utils::read_physx_pointer(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(actor_pointer, nullptr);

    physx::PxRigidDynamic* const actor = static_cast<physx::PxRigidDynamic*>(actor_pointer);
    GatedSimulationScope simulation_scope(m_handle, scene, actor);

    ovphysx_enqueue_result_t pending_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    if (pending_step.status != OVPHYSX_API_SUCCESS)
        FAIL() << "Failed to submit the callback-gated simulation";
    simulation_scope.set_pending_task(pending_step);

    if (!simulation_scope.wait_until_entered())
        FAIL() << "PhysX did not invoke onAdvance before the timeout test deadline";

    simulation_scope.start_watchdog();
    const PollOutcome poll_outcome = wait_with_timeout(
        m_handle,
        pending_step,
        OVPHYSX_TIMEOUT_POLL);

    bool finite_wait_ran = false;
    PollOutcome finite_outcome{};
    if (poll_outcome.status == OVPHYSX_API_TIMEOUT)
    {
        finite_outcome = wait_with_timeout(m_handle, pending_step, 1'000'000);
        finite_wait_ran = true;
    }

    EXPECT_TRUE(waitForTaskSuccess(m_handle, initial_step, 0));
    ovphysx::physx::WaitResult original;
    EXPECT_EQ(original.waitTask(m_handle, pending_step, 0), OVPHYSX_API_TIMEOUT);
    ovphysx::physx::WaitResult moved(std::move(original));
    EXPECT_EQ(moved.status(), OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(moved.pendingTask().op_index, pending_step.op_index);
    EXPECT_EQ(original.pendingTask().op_index, 0u);

    ovphysx_wait_task_result_t partial{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, 0, &partial).status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(partial.pending_task.op_index, pending_step.op_index);
    EXPECT_EQ(partial.num_errors, 1u);
    if (partial.num_errors) {
        EXPECT_EQ(partial.error_tasks[0].op_index, failed.op_index);
    }
    const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, failed);
    EXPECT_GT(error.length, 0u);
    ovphysx_wait_task_result_t repeated{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, 0, &repeated).status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(repeated.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&repeated);
    ovphysx_destroy_wait_task_result(&partial);

    const bool watchdog_fired = simulation_scope.open_and_join_watchdog();
    const ovphysx_api_status_t final_status = simulation_scope.finish();
    // A release runs after earlier work on the instance like every other call. The gated
    // step job has finished, so on the idle instance it runs on the calling thread and a poll
    // collects it; under the always-queue test switch it is queued, and the wait waits for it.
    const auto later = ovphysx_release_query(m_handle, 0);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, later, ovphysxIdleCallsRunOnCaller() ? 0 : OVPHYSX_TIMEOUT_INFINITE));

    ASSERT_FALSE(watchdog_fired);
    ASSERT_EQ(poll_outcome.status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(poll_outcome.lowest_pending_op_index, pending_step.op_index);
    EXPECT_EQ(poll_outcome.error_count, 0u);
    ASSERT_TRUE(finite_wait_ran);
    ASSERT_EQ(finite_outcome.status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(finite_outcome.lowest_pending_op_index, pending_step.op_index);
    EXPECT_EQ(finite_outcome.error_count, 0u);

    EXPECT_EQ(final_status, OVPHYSX_API_SUCCESS);
}

TEST_F(PhysXTestFixture, PhysXAsync_ConsumedStepOpReturnsNotFound) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);

    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t first_wait = ovphysx_wait_task(m_handle, step_result, UINT64_MAX, &wait_result);
    ASSERT_EQ(first_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_result_t second_wait = ovphysx_wait_task(m_handle, step_result, 0, &wait_result);
    EXPECT_EQ(second_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&wait_result);
}

TEST_F(PhysXTestFixture, PhysXAsync_ConsumedOpsRemainSingleUseAfterInternalSync) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t consumed_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(consumed_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t first_wait = ovphysx_wait_task(m_handle, consumed_step, UINT64_MAX, &wait_result);
    ASSERT_EQ(first_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_enqueue_result_t internally_synced_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(internally_synced_step.status, OVPHYSX_API_SUCCESS);
    ovphysx_enqueue_result_t pending_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(pending_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t reused_wait = ovphysx_wait_task(m_handle, consumed_step, 0, &wait_result);
    EXPECT_EQ(reused_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_result_t internal_wait =
        ovphysx_wait_task(m_handle, internally_synced_step, UINT64_MAX, &wait_result);
    ASSERT_EQ(internal_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_result_t repeated_internal_wait =
        ovphysx_wait_task(m_handle, internally_synced_step, 0, &wait_result);
    EXPECT_EQ(repeated_internal_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_result_t pending_wait =
        ovphysx_wait_task(m_handle, pending_step, UINT64_MAX, &wait_result);
    EXPECT_EQ(pending_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_enqueue_result_t wait_all_consumed_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(wait_all_consumed_step.status, OVPHYSX_API_SUCCESS);
    ovphysx_result_t wait_all =
        ovphysx_wait_all(m_handle, UINT64_MAX, &wait_result);
    ASSERT_EQ(wait_all.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_enqueue_result_t later_internally_synced_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(later_internally_synced_step.status, OVPHYSX_API_SUCCESS);
    ovphysx_enqueue_result_t later_pending_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(later_pending_step.status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t reused_wait_all_index =
        ovphysx_wait_task(m_handle, wait_all_consumed_step, 0, &wait_result);
    EXPECT_EQ(reused_wait_all_index.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_result_t later_pending_wait =
        ovphysx_wait_task(m_handle, later_pending_step, UINT64_MAX, &wait_result);
    EXPECT_EQ(later_pending_wait.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);
}

TEST_F(PhysXTestFixture, PhysXAsync_WaitAllConsumesInternallySynchronizedIndex) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);

    const ovphysx_result_t detach_result = test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle));
    ASSERT_EQ(detach_result.status, OVPHYSX_API_SUCCESS);

    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t wait_all_result =
        ovphysx_wait_all(m_handle, UINT64_MAX, &wait_result);
    ASSERT_EQ(wait_all_result.status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait_result);

    ovphysx_result_t repeated_wait =
        ovphysx_wait_task(m_handle, step_result, 0, &wait_result);
    EXPECT_EQ(repeated_wait.status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&wait_result);
}

TEST(PhysXAsync, InvalidHandles) {
    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t result = ovphysx_wait_task(0, {}, 0, &wait_result); // invalid handle
    EXPECT_EQ(result.status, OVPHYSX_API_NOT_FOUND);
    // Error details available via ovphysx_get_last_error() if needed
}

TEST_F(PhysXTestFixture, PhysXAsync_MultipleEvents) {
    // Exercise multiple async step operations across reset/reattach cycles.
    const int num_ops = 3;
    
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    
    bool all_completed = true;
    for (int i = 0; i < num_ops; ++i) {
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));
        ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);
        
        ovphysx_wait_task_result_t wait_result{};
        ovphysx_result_t result = ovphysx_wait_task(m_handle, step_result, 2000000000ULL, &wait_result); // 2 sec
        
        if (result.status != OVPHYSX_API_SUCCESS) {
            all_completed = false;
        }
        
        ovphysx_destroy_wait_task_result(&wait_result);

        if (!all_completed) {
            break;
        }
        
        ovphysx_enqueue_result_t reset_result = ovphysx_detach_ovstage(m_handle);
        ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS)
            << "Failed to enqueue detach_ovstage in iteration " << (i + 1);
        ovphysx_result_t reset_wait = ovphysx_wait_task(m_handle, reset_result, 2000000000ULL, &wait_result);
        ASSERT_EQ(reset_wait.status, OVPHYSX_API_SUCCESS)
            << "detach_ovstage wait failed in iteration " << (i + 1);
        ASSERT_EQ(wait_result.num_errors, 0u)
            << "detach_ovstage completed with operation errors in iteration " << (i + 1);
        ovphysx_destroy_wait_task_result(&wait_result);
        destroy_ovstage_test_attachments(m_handle);
    }
    
    EXPECT_TRUE(all_completed);
    
    // Cleanup handled by fixture TearDown
}
