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
#include <PxSimulationEventCallback.h>
#include "internal/sdk/ovphysxAsyncWait.hpp"
#include "ovphysx/experimental/Helpers.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
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

struct AdvanceGate
{
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool open = false;
    bool watchdog_fired = false;
};

class GatedSimulationCallback final : public physx::PxSimulationEventCallback
{
public:
    GatedSimulationCallback(
        physx::PxSimulationEventCallback* original,
        std::shared_ptr<AdvanceGate> gate)
        : m_original(original), m_gate(std::move(gate))
    {
    }

    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onConstraintBreak(constraints, count);
    }

    void onWake(physx::PxActor** actors, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onWake(actors, count);
    }

    void onSleep(physx::PxActor** actors, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onSleep(actors, count);
    }

    void onContact(
        const physx::PxContactPairHeader& pair_header,
        const physx::PxContactPair* pairs,
        physx::PxU32 pair_count) override
    {
        if (m_original != nullptr)
            m_original->onContact(pair_header, pairs, pair_count);
    }

    void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onTrigger(pairs, count);
    }

    void onAdvance(
        const physx::PxRigidBody* const* bodies,
        const physx::PxTransform* poses,
        physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onAdvance(bodies, poses, count);

        std::unique_lock<std::mutex> lock(m_gate->mutex);
        m_gate->entered = true;
        m_gate->condition.notify_all();
        m_gate->condition.wait(lock, [this] { return m_gate->open; });
    }

private:
    physx::PxSimulationEventCallback* m_original;
    std::shared_ptr<AdvanceGate> m_gate;
};

void open_advance_gate(const std::shared_ptr<AdvanceGate>& gate)
{
    {
        std::lock_guard<std::mutex> lock(gate->mutex);
        gate->open = true;
    }
    gate->condition.notify_all();
}

bool wait_for_advance_gate(const std::shared_ptr<AdvanceGate>& gate)
{
    std::unique_lock<std::mutex> lock(gate->mutex);
    return gate->condition.wait_for(
        lock,
        std::chrono::seconds(5),
        [gate] { return gate->entered; });
}

class GatedSimulationScope final
{
public:
    GatedSimulationScope(
        ovphysx_instance_handle_t handle,
        physx::PxScene* scene,
        physx::PxRigidDynamic* actor)
        : m_handle(handle),
          m_scene(scene),
          m_actor(actor),
          m_originalCallback(scene->getSimulationEventCallback()),
          m_previewWasSet(actor->getRigidBodyFlags().isSet(
              physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)),
          m_gate(std::make_shared<AdvanceGate>()),
          m_callback(m_originalCallback, m_gate)
    {
        m_actor->setRigidBodyFlag(
            physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW,
            true);
        m_actor->wakeUp();
        m_scene->setSimulationEventCallback(&m_callback);
    }

    ~GatedSimulationScope()
    {
        finish();
    }

    GatedSimulationScope(const GatedSimulationScope&) = delete;
    GatedSimulationScope& operator=(const GatedSimulationScope&) = delete;

    void set_pending_task(ovphysx_enqueue_result_t task)
    {
        m_pendingTask = task;
    }

    bool wait_until_entered()
    {
        return wait_for_advance_gate(m_gate);
    }

    void start_watchdog()
    {
        std::shared_ptr<AdvanceGate> gate = m_gate;
        m_watchdog = std::thread([gate] {
            std::unique_lock<std::mutex> lock(gate->mutex);
            if (!gate->condition.wait_for(
                    lock,
                    std::chrono::seconds(5),
                    [gate] { return gate->open; }))
            {
                gate->watchdog_fired = true;
                gate->open = true;
                lock.unlock();
                gate->condition.notify_all();
            }
        });
    }

    bool open_and_join_watchdog() noexcept
    {
        open_advance_gate(m_gate);
        if (m_watchdog.joinable())
            m_watchdog.join();

        std::lock_guard<std::mutex> lock(m_gate->mutex);
        return m_gate->watchdog_fired;
    }

    ovphysx_api_status_t finish() noexcept
    {
        if (m_restored)
            return m_finalStatus;

        open_and_join_watchdog();

        if (m_pendingTask.op_index != 0)
        {
            ovphysx_wait_task_result_t wait_result{};
            const ovphysx_result_t result = ovphysx_wait_task(
                m_handle,
                m_pendingTask,
                OVPHYSX_TIMEOUT_INFINITE,
                &wait_result);
            ovphysx_destroy_wait_task_result(&wait_result);
            m_finalStatus = result.status;
            m_pendingTask = {};
        }

        m_scene->setSimulationEventCallback(m_originalCallback);
        m_actor->setRigidBodyFlag(
            physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW,
            m_previewWasSet);
        m_restored = true;
        return m_finalStatus;
    }

private:
    ovphysx_instance_handle_t m_handle;
    physx::PxScene* m_scene;
    physx::PxRigidDynamic* m_actor;
    physx::PxSimulationEventCallback* m_originalCallback;
    bool m_previewWasSet;
    std::shared_ptr<AdvanceGate> m_gate;
    GatedSimulationCallback m_callback;
    std::thread m_watchdog;
    ovphysx_enqueue_result_t m_pendingTask{};
    ovphysx_api_status_t m_finalStatus = OVPHYSX_API_SUCCESS;
    bool m_restored = false;
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

TEST(PhysXAsync, ReadinessObservedAtDeadlineWins)
{
    const std::chrono::steady_clock::time_point start{};
    std::chrono::steady_clock::time_point now = start;
    size_t readiness_checks = 0;

    const bool ready = ovphysx::async::detail::wait_until_simulation_ready(
        false,
        std::chrono::nanoseconds(1),
        start,
        [&readiness_checks]() {
            ++readiness_checks;
            return readiness_checks == 2;
        },
        [&now]() { return now; },
        [&now](std::chrono::steady_clock::duration duration) { now += duration; });

    EXPECT_TRUE(ready);
    EXPECT_EQ(readiness_checks, 2u);
    EXPECT_EQ(now - start, std::chrono::nanoseconds(1));
}

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
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/boxes_falling_on_groundplane.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    ovphysx_string_t target = OVPHYSX_LITERAL("/World/AbsentCloneTarget");
    const auto failed = ovphysx_clone(m_handle, OVPHYSX_LITERAL("/World/AbsentSource"),
                                     &target, 1, nullptr, nullptr);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t initial_step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(initial_step.status, OVPHYSX_API_SUCCESS);
    // Internal synchronization leaves the first step available for collection.
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
    const auto later = ovphysx_release_query(m_handle, 0);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, later, 0));
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

TEST(PhysXAsync, TaskWaitAllUsesOneReadinessDeadline)
{
    const std::chrono::steady_clock::time_point start{};
    const std::chrono::nanoseconds budget(100);
    std::chrono::steady_clock::time_point now = start + std::chrono::nanoseconds(30);
    EXPECT_EQ(ovphysx::async::detail::remaining_readiness_timeout(budget, false, start, now), 70u);
    now += std::chrono::nanoseconds(50);
    EXPECT_EQ(ovphysx::async::detail::remaining_readiness_timeout(budget, false, start, now), 20u);
    now += std::chrono::nanoseconds(20);
    EXPECT_EQ(ovphysx::async::detail::remaining_readiness_timeout(budget, false, start, now), 0u);
    now += std::chrono::nanoseconds(100);
    EXPECT_EQ(ovphysx::async::detail::remaining_readiness_timeout(budget, false, start, now), 0u);
    EXPECT_EQ(ovphysx::async::detail::remaining_readiness_timeout(budget, true, start, now), UINT64_MAX);
}
