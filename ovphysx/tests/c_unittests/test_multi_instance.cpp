// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PARTIALLY DEPRECATED (tensor-binding-deprecation): the binding-isolation foil retires with the binding.

/**
 * @file test_multi_instance.cpp
 * @brief Tests for creating and destroying multiple PhysX instances
 *
 * This test verifies that:
 * 1. Multiple instances can be created and destroyed sequentially
 * 2. USD stages can be loaded/unloaded across instance lifecycles
 * 3. Resources are properly cleaned up between instances
 */

/**
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 *
 * @implements REQ-CAPI-ATTACH-OWNER-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-CAPI-DETACH-001
 * @covers AC-6
 *
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-10
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-6
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "debug_render_session_shim.h"
#include "ovphysx_test_utils.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

// Private binding ABI; deliberately absent from the public headers.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_is_query_valid(
    ovphysx_instance_handle_t handle, ovphysx_query_handle_t query, bool* out_valid);

using namespace test_utils;

TEST(MultiInstance, SequentialCreateDestroy) {
    const int NUM_ITERATIONS = 3;
    
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        ovphysx_instance_handle_t handle = 0;
        ovphysx_result_t create_result = ovphysx_create_instance(&handle);
        ASSERT_EQ(create_result.status, OVPHYSX_API_SUCCESS) << "Failed to create instance " << (i + 1);
        ASSERT_NE(handle, 0) << "Instance handle should not be 0 for iteration " << (i + 1);
        
        ovphysx_result_t destroy_result = ovphysx_destroy_instance(handle);
        ASSERT_EQ(destroy_result.status, OVPHYSX_API_SUCCESS) << "Failed to destroy instance " << (i + 1);
    }
}

TEST(MultiInstance, SequentialWithUSDLoad) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";
    const int NUM_ITERATIONS = 2;
    
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        ovphysx_instance_handle_t handle = 0;
        ovphysx_result_t create_result = ovphysx_create_instance(&handle);
        ASSERT_EQ(create_result.status, OVPHYSX_API_SUCCESS) << "Failed to create instance " << (i + 1);
        ASSERT_NE(handle, 0) << "Instance handle should not be 0 for iteration " << (i + 1);
        
        ASSERT_TRUE(attach_usd_with_ovstage(handle, usd_path))
            << "ovstage attach/update failed in iteration " << (i + 1);
        
        // NOTE: No separate sync() call needed - operations are stream-ordered
        
        // Unload USD (reset clears all)
        ovphysx_enqueue_result_t reset_result = ovphysx_detach_ovstage(handle);
        ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS) << "Failed to reset in iteration " << (i + 1);
        ASSERT_TRUE(waitForTaskSuccess(handle, reset_result, 5000000000ULL))
            << "Reset failed in iteration " << (i + 1);
        destroy_ovstage_test_attachments(handle);
        
        ovphysx_result_t destroy_result = ovphysx_destroy_instance(handle);
        ASSERT_EQ(destroy_result.status, OVPHYSX_API_SUCCESS) << "Failed to destroy instance " << (i + 1);
    }
}

TEST(MultiInstance, MultipleSimultaneousInstances) {
    const int NUM_INSTANCES = 3;
    ovphysx_instance_handle_t handles[NUM_INSTANCES] = {0};

    for (int i = 0; i < NUM_INSTANCES; ++i) {
        ovphysx_result_t result = ovphysx_create_instance(&handles[i]);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create instance " << (i + 1);
        ASSERT_NE(handles[i], 0) << "Instance handle should not be 0 for instance " << (i + 1);
    }

    for (int i = 0; i < NUM_INSTANCES; ++i) {
        ovphysx_result_t result = ovphysx_destroy_instance(handles[i]);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to destroy instance " << (i + 1);
    }
}

// Hold a real runtime entry on B, including callback context on its thread.
// A must finish its entire output lifecycle before B is allowed to return.
TEST(MultiInstance, OutputCompletesWhilePeerRuntimeCallbackIsBlocked)
{
    const char* scene = OVPHYSX_SOURCE_DIR "/tests/data/boxes_falling_on_groundplane.usda";
    ovphysx_instance_handle_t handles[2]{};
    struct Cleanup
    {
        ovphysx_instance_handle_t* handles;
        ~Cleanup()
        {
            for (int i = 0; i < 2; ++i)
                if (handles[i])
                {
                    complete_task(handles[i], ovphysx_detach_ovstage(handles[i]));
                    destroy_ovstage_test_stages(handles[i]);
                    ovphysx_destroy_instance(handles[i]);
                }
        }
    } cleanup{handles};
    for (int i = 0; i < 2; ++i)
    {
        ASSERT_EQ(ovphysx_create_instance(&handles[i]).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(attach_usd_with_ovstage(handles[i], scene));
        ASSERT_EQ(ovphysx_step_sync(handles[i], 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    }
    const ovphysx_instance_handle_t owner = handles[0];
    const ovx_string_or_token_t mass{0, {OVPHYSX_ATTR_MASS, sizeof(OVPHYSX_ATTR_MASS) - 1}};
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(owner,
        ovphysx_query(owner, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));

    struct Gate
    {
        std::mutex mutex;
        std::condition_variable condition;
        ovphysx_instance_handle_t peer;
        bool entered = false;
        bool open = false;
        bool timedOut = false;
        ovphysx_api_status_t reentry = OVPHYSX_API_SUCCESS;
    } gate;
    gate.peer = owner;
    ovphysx_object_change_callbacks_t callbacks{};
    callbacks.user_data = &gate;
    callbacks.on_all_objects_destroyed = [](void* data) {
        Gate& state = *static_cast<Gate*>(data);
        state.reentry = ovphysx_release_query(state.peer, 0).status;
        std::unique_lock<std::mutex> lock(state.mutex);
        state.entered = true;
        state.condition.notify_all();
        state.timedOut = !state.condition.wait_for(lock, std::chrono::seconds(15), [&] { return state.open; });
    };
    ovphysx_subscription_id_t subscription = 0;
    ASSERT_EQ(ovphysx_subscribe_object_changes(handles[1], &callbacks, &subscription).status,
              OVPHYSX_API_SUCCESS);
    struct Unsubscribe
    {
        ovphysx_instance_handle_t owner;
        ovphysx_subscription_id_t subscription;
        ~Unsubscribe() { ovphysx_unsubscribe_object_changes(owner, subscription); }
    } unsubscribe{handles[1], subscription};
    ovphysx_api_status_t retired = OVPHYSX_API_ERROR;
    std::thread worker([&] {
        retired = complete_task(handles[1], ovphysx_detach_ovstage(handles[1])).status;
    });
    struct Join
    {
        Gate& gate;
        std::thread& worker;
        ~Join()
        {
            {
                std::lock_guard<std::mutex> lock(gate.mutex);
                gate.open = true;
            }
            gate.condition.notify_all();
            worker.join();
        }
    };
    {
        Join join{gate, worker};
        {
            std::unique_lock<std::mutex> lock(gate.mutex);
            ASSERT_TRUE(gate.condition.wait_for(lock, std::chrono::seconds(15), [&] { return gate.entered; }));
        }
        EXPECT_EQ(gate.reentry, OVPHYSX_API_INVALID_STATE);
        bool valid = false;
        EXPECT_EQ(ovphysx_internal_is_query_valid(owner, query, &valid).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(valid);
        // A fresh query exercises the entry used by read_sync/write_sync.
        ovphysx_query_handle_t fresh = 0;
        EXPECT_TRUE(waitForTaskSuccess(owner,
            ovphysx_query(owner, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &fresh, nullptr)));
        EXPECT_TRUE(waitForTaskSuccess(owner, ovphysx_release_query(owner, fresh)));
        ovphysx_write_handle_t write = 0;
        ASSERT_TRUE(waitForTaskSuccess(owner, ovphysx_write(owner, query, &mass, 1, &write, nullptr)));
        ovphysx_fetched_write_group_t fetched{};
        ASSERT_TRUE(waitForTaskSuccess(owner, ovphysx_fetch_write_next(owner, write, &fetched)));
        ASSERT_NE(fetched.group, nullptr);
        ASSERT_EQ(fetched.group->data.tensor_count, 1u);
        const DLTensor& tensor = fetched.group->data.tensors[0];
        ASSERT_EQ(tensor.device.device_type, kDLCPU);
        ASSERT_EQ(tensor.dtype.bits, 32);
        for (int64_t i = 0; i < tensor.shape[0]; ++i)
            static_cast<float*>(tensor.data)[i] = 7.0f;
        EXPECT_TRUE(waitForTaskSuccess(owner, ovphysx_commit_group(owner, write, fetched.group, {})));
        EXPECT_TRUE(waitForTaskSuccess(owner, ovphysx_release_write(owner, write)));
        ovphysx_read_handle_t read = 0;
        ASSERT_TRUE(waitForTaskSuccess(owner, ovphysx_read(owner, query, &mass, 1, &read, nullptr)));
        const ovstage_read_group_t* group = nullptr;
        ASSERT_TRUE(waitForTaskSuccess(owner, ovphysx_fetch_read_next(owner, read, &group)));
        ASSERT_NE(group, nullptr);
        ASSERT_EQ(group->data.tensor_count, 1u);
        const DLTensor& values = group->data.tensors[0];
        for (int64_t i = 0; i < values.shape[0]; ++i)
            EXPECT_FLOAT_EQ(static_cast<const float*>(values.data)[i], 7.0f);
        EXPECT_TRUE(waitForTaskSuccess(owner, ovphysx_release_group(owner, read, group->read_group_id)));
        EXPECT_TRUE(waitForTaskSuccess(owner, ovphysx_release_read(owner, read)));
        EXPECT_TRUE(waitForTaskSuccess(owner, ovphysx_release_query(owner, query)));
    }
    EXPECT_FALSE(gate.timedOut) << "A must finish before B leaves its runtime callback";
    EXPECT_EQ(retired, OVPHYSX_API_SUCCESS);
}

// Exercise the real log dispatcher without needing a CUDA failure or holding
// a backend mutex: rejected nested calls must never reach that backend.
TEST_F(PhysXTestFixture, OutputReentryFromLogCallbackIsRejected)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle,
        OVPHYSX_SOURCE_DIR "/tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    struct State
    {
        ovphysx_instance_handle_t handle;
        ovphysx_query_handle_t query = 0;
        const ovx_string_or_token_t mass{0, {OVPHYSX_ATTR_MASS, sizeof(OVPHYSX_ATTR_MASS) - 1}};
        bool called = false;
        bool nestedValid = true;
        ovphysx_result_t nestedMetadata{OVPHYSX_API_SUCCESS};
        ovphysx_read_handle_t nestedRead = 0;
        ovphysx_enqueue_result_t nestedReceipt{};
        bool otherValid = false;
        ovphysx_result_t otherMetadata{OVPHYSX_API_ERROR};
        ovphysx_result_t otherRead{OVPHYSX_API_ERROR};
    } state{m_handle};
    ASSERT_TRUE(waitForTaskSuccess(m_handle,
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &state.query, nullptr)));
    struct Cleanup
    {
        State& state;
        ~Cleanup()
        {
            ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
            if (state.nestedRead)
            {
                complete_task(state.handle, state.nestedReceipt);
                complete_task(state.handle, ovphysx_release_read(state.handle, state.nestedRead));
            }
            complete_task(state.handle, ovphysx_release_query(state.handle, state.query));
        }
    } cleanup{state};
    // Initialize the tensor backend before installing the callback, so the
    // separate-thread control does not emit initialization diagnostics while
    // the logger's callback-serialization mutex is held by this thread.
    ovphysx_read_handle_t warmup = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle,
        ovphysx_read(m_handle, state.query, &state.mass, 1, &warmup, nullptr)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, warmup)));
    ASSERT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_WARNING, nullptr,
        [](ovphysx_log_level_t, ovphysx_string_t, ovphysx_string_t, double, void* data) {
            State& state = *static_cast<State*>(data);
            if (state.called)
                return;
            state.called = true;
            state.nestedMetadata = ovphysx_internal_is_query_valid(state.handle, state.query, &state.nestedValid);
            state.nestedReceipt = ovphysx_read(state.handle, state.query, &state.mass, 1, &state.nestedRead, nullptr);

            // The same idle attachment remains usable from a different client
            // thread while this callback is still active. No global barrier.
            std::thread client([&] {
                state.otherMetadata = ovphysx_internal_is_query_valid(state.handle, state.query, &state.otherValid);
                ovphysx_read_handle_t read = 0;
                state.otherRead = complete_task(state.handle,
                    ovphysx_read(state.handle, state.query, &state.mass, 1, &read, nullptr));
                if (read)
                    complete_task(state.handle, ovphysx_release_read(state.handle, read));
            });
            client.join();
        }, &state).status, OVPHYSX_API_SUCCESS);
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(state.called);
    EXPECT_EQ(state.nestedMetadata.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(state.nestedValid);
    EXPECT_EQ(state.nestedReceipt.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(state.nestedReceipt.op_index, 0u);
    EXPECT_EQ(state.nestedRead, 0u);
    EXPECT_EQ(state.otherMetadata.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(state.otherValid);
    EXPECT_EQ(state.otherRead.status, OVPHYSX_API_SUCCESS);
}

// Regression (NVBugs 6433668): a second, stage-less handle's async
// ovphysx_step() must be rejected rather than silently advancing whatever
// stage another handle has attached. IPhysxSimulation is a process-wide
// singleton (CarboniteLoader::loadPhysxPlugins()), so without the rejection a
// stage-less handle B's ovphysx_step() falls through ensure_physics_attached()'s
// "no stage -> trivial success" branch straight into physxSim->simulate(). That
// steps handle A's stage while marking only B's own first_step_done, which lets
// clone() on A's real stage pass its after-step guard although A has been
// stepped through B.
TEST(MultiInstance, StagelessHandleAsyncStepRejectedDoesNotAdvanceOtherHandleStage) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";

    // Handle A: owns the only attached stage.
    ovphysx_instance_handle_t handle_a = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    // Handle B: separate instance, never attaches anything.
    ovphysx_instance_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle_b).status, OVPHYSX_API_SUCCESS);

    // B's async step() must be rejected outright. It must never reach
    // physxSim->simulate() and silently advance A's stage.
    ovphysx_enqueue_result_t step_b = ovphysx_step(handle_b, 1.0f / 60.0f);
    EXPECT_NE(step_b.status, OVPHYSX_API_SUCCESS)
        << "Stage-less handle B's async step() should be rejected, not silently advance A's stage";

    // A's stage was never touched by B's rejected call, so A must still accept
    // clone(). It would be rejected if A's first_step_done/warmup_done had been
    // flipped by B's step.
    ovphysx_string_t target1 = ovphysx_cstr("/World/envs/env1");
    ovphysx_enqueue_result_t clone_res = ovphysx_clone(
        handle_a, ovphysx_cstr("/World/envs/env0"), &target1, 1, nullptr, nullptr);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_SUCCESS)
        << "A's clone() guard should be unaffected by B's rejected stage-less step";
    if (clone_res.status == OVPHYSX_API_SUCCESS) {
        EXPECT_TRUE(waitForTaskSuccess(handle_a, clone_res, 5'000'000'000ULL));
    }

    // Step A for real, then clone() on A is rejected in all modes. The guard
    // enforces the same ordering contract on CPU and GPU.
    ovphysx_enqueue_result_t step_a = ovphysx_step(handle_a, 1.0f / 60.0f);
    ASSERT_EQ(step_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, step_a, 5'000'000'000ULL));

    ovphysx_string_t target2 = ovphysx_cstr("/World/envs/env2");
    ovphysx_enqueue_result_t clone_res2 = ovphysx_clone(
        handle_a, ovphysx_cstr("/World/envs/env0"), &target2, 1, nullptr, nullptr);
    EXPECT_EQ(clone_res2.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "clone() after step() must be rejected in all modes";

    ovphysx_destroy_instance(handle_a);
    ovphysx_destroy_instance(handle_b);
}

// NVBug 6504951, second half. Tensor-binding handles come from one process-wide,
// never-reused sequence. If they counted from 1 inside each instance, the first
// binding of a destroyed instance and the first binding of the next instance
// would share the number 1 and a stale token from the first would resolve the
// second instance's live binding. With the process-wide sequence the spec lookup
// below misses the map and returns OVPHYSX_API_NOT_FOUND.
TEST(MultiInstance, TensorBindingHandleIsNotReusedByAnotherInstance)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/envs/env0/table");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_instance_handle_t handle_a = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_tensor_binding_handle_t binding_a = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle_a, &desc, &binding_a).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_a = ovphysx_detach_ovstage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, reset_a, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_a);
    ASSERT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);

    ovphysx_instance_handle_t handle_b = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle_b).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_b, usd_path));

    ovphysx_tensor_binding_handle_t binding_b = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle_b, &desc, &binding_b).status, OVPHYSX_API_SUCCESS);

    EXPECT_NE(binding_a, binding_b);

    ovphysx_tensor_spec_t spec{};
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(handle_b, binding_a, &spec).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(handle_b, binding_b, &spec).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(handle_b, binding_b).status, OVPHYSX_API_SUCCESS);
    ovphysx_enqueue_result_t reset_b = ovphysx_detach_ovstage(handle_b);
    ASSERT_EQ(reset_b.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_b, reset_b, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_b);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

namespace
{
std::atomic<int> g_allDestroyedA{0};
std::atomic<int> g_allDestroyedB{0};
std::atomic<int> g_detachDestroyedA{0};
std::atomic<int> g_detachDestroyedB{0};

void countAllDestroyedA(void*)
{
    g_allDestroyedA.fetch_add(1, std::memory_order_relaxed);
}

void countAllDestroyedB(void*)
{
    g_allDestroyedB.fetch_add(1, std::memory_order_relaxed);
}

void countDetachDestroyedA(void*)
{
    g_detachDestroyedA.fetch_add(1, std::memory_order_relaxed);
}

void countDetachDestroyedB(void*)
{
    g_detachDestroyedB.fetch_add(1, std::memory_order_relaxed);
}

void expectAttachmentState(ovphysx_instance_handle_t instance, ovphysx_query_handle_t query, bool expected)
{
    bool attached = !expected;
    EXPECT_EQ(ovphysx_is_stage_attached(instance, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(attached, expected);
    bool valid = !expected;
    EXPECT_EQ(ovphysx_internal_is_query_valid(instance, query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(valid, expected);
}
} // namespace

TEST(MultiInstance, QueryValidityRejectsInvalidArgumentsAndUnknownQueries)
{
    bool valid = true;
    EXPECT_EQ(ovphysx_internal_is_query_valid(OVPHYSX_INVALID_HANDLE, 0, &valid).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_FALSE(valid);

    ovphysx_instance_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_internal_is_query_valid(handle, 0, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    valid = true;
    EXPECT_EQ(ovphysx_internal_is_query_valid(handle, 0, &valid).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(valid);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

TEST(MultiInstance, QueryValidityRetainsOriginalAttachmentAndTaskOutcomes)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_instance_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle, usd_path));
    ovstage_instance_t* stage = nullptr;
    {
        std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
        stage = ovstage_test_attachments().at(handle).front().stage;
    }
    ASSERT_NE(stage, nullptr);

    ovphysx_query_handle_t original_query = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle,
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &original_query, nullptr)));

    // An earlier task failure must remain separately collectible after the validity check.
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/MissingCloneTarget");
    const ovphysx_enqueue_result_t failed =
        ovphysx_clone(handle, OVPHYSX_LITERAL("/World/MissingCloneSource"), &target, 1, nullptr, nullptr);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    bool valid = false;
    ASSERT_EQ(ovphysx_internal_is_query_valid(handle, original_query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(valid);
    ovphysx_wait_task_result_t failed_result{};
    EXPECT_EQ(ovphysx_wait_task(handle, failed, 5'000'000'000ULL, &failed_result).status, OVPHYSX_API_ERROR);
    EXPECT_GE(failed_result.num_errors, 1u);
    ovphysx_destroy_wait_task_result(&failed_result);

    // The check also finishes pending simulation without collecting its receipt.
    const ovphysx_enqueue_result_t step = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    valid = false;
    ASSERT_EQ(ovphysx_internal_is_query_valid(handle, original_query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(valid);
    EXPECT_TRUE(waitForTaskSuccess(handle, step));

    ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_detach_ovstage(handle)));
    valid = true;
    ASSERT_EQ(ovphysx_internal_is_query_valid(handle, original_query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(valid);
    ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_attach_ovstage(handle, stage, 1)));
    bool attached = false;
    ASSERT_EQ(ovphysx_is_stage_attached(handle, stage, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attached);
    valid = true;
    ASSERT_EQ(ovphysx_internal_is_query_valid(handle, original_query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(valid) << "Reattaching the same Stage must not revive the original query";

    ovphysx_query_handle_t current_query = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle,
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &current_query, nullptr)));
    valid = false;
    ASSERT_EQ(ovphysx_internal_is_query_valid(handle, current_query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(valid);
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, current_query)));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, original_query)));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_detach_ovstage(handle)));
    EXPECT_TRUE(destroy_ovstage_test_attachments(handle));
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

// Two instances attach separate stages that share prim paths. Each keeps its
// own runtime attachment. Stepping, resetting, and detaching one leaves the
// other attached and steppable.
TEST(MultiInstance, TwoInstancesRemainAttachedAndStepIndependently) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";

    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(&handle_b).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path, 1, "ovphysx-two-attach-a"));

    ovphysx_query_handle_t query_a = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle_a,
        ovphysx_query(handle_a, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query_a, nullptr),
        5'000'000'000ULL));
    expectAttachmentState(handle_a, query_a, true);

    ASSERT_TRUE(attach_usd_with_ovstage(handle_b, usd_path, 1, "ovphysx-two-attach-b"));
    ovphysx_query_handle_t query_b = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle_b,
        ovphysx_query(handle_b, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query_b, nullptr),
        5'000'000'000ULL));
    expectAttachmentState(handle_b, query_b, true);
    expectAttachmentState(handle_a, query_a, true);

    EXPECT_EQ(ovphysx_step_sync(handle_a, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    g_allDestroyedA.store(0);
    g_allDestroyedB.store(0);
    ovphysx_object_change_callbacks_t callbacks_a{};
    callbacks_a.on_all_objects_destroyed = countAllDestroyedA;
    ovphysx_object_change_callbacks_t callbacks_b{};
    callbacks_b.on_all_objects_destroyed = countAllDestroyedB;
    ovphysx_subscription_id_t sub_a = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_subscription_id_t sub_b = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ASSERT_EQ(ovphysx_subscribe_object_changes(handle_a, &callbacks_a, &sub_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_subscribe_object_changes(handle_b, &callbacks_b, &sub_b).status, OVPHYSX_API_SUCCESS);

    // detach_ovstage detaches that instance's ovstage. It must notify only A's
    // subscription and leave B's attachment in place.
    ovphysx_enqueue_result_t reset_a = ovphysx_detach_ovstage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, reset_a, 5'000'000'000ULL));
    EXPECT_GE(g_allDestroyedA.load(), 1);
    EXPECT_EQ(g_allDestroyedB.load(), 0);

    expectAttachmentState(handle_a, query_a, false);
    expectAttachmentState(handle_b, query_b, true);
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(test_utils::complete_task(handle_b, ovphysx_detach_ovstage(handle_b)).status, OVPHYSX_API_SUCCESS);
    expectAttachmentState(handle_b, query_b, false);
    EXPECT_TRUE(waitForTaskSuccess(handle_a, ovphysx_release_query(handle_a, query_a), 5'000'000'000ULL));
    EXPECT_TRUE(waitForTaskSuccess(handle_b, ovphysx_release_query(handle_b, query_b), 5'000'000'000ULL));

    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_a, sub_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_b, sub_b).status, OVPHYSX_API_SUCCESS);
    destroy_ovstage_test_attachments(handle_a);
    destroy_ovstage_test_attachments(handle_b);
    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

// Same USD prim paths on two stages. Stepping alternates while both are
// attached. Detach and destroy of A leave B's attachment and step usable.
TEST(MultiInstance, DetachAndDestroyOneLiveInstanceLeavesPeerSteppable) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";

    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(&handle_b).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path, 1, "ovphysx-detach-peer-a"));
    ASSERT_TRUE(attach_usd_with_ovstage(handle_b, usd_path, 1, "ovphysx-detach-peer-b"));

    ovphysx_query_handle_t query_a = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle_a,
        ovphysx_query(handle_a, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query_a, nullptr),
        5'000'000'000ULL));
    ovphysx_query_handle_t query_b = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle_b,
        ovphysx_query(handle_b, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query_b, nullptr),
        5'000'000'000ULL));
    expectAttachmentState(handle_b, query_b, true);
    expectAttachmentState(handle_a, query_a, true);

    EXPECT_EQ(ovphysx_step_sync(handle_a, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_step_sync(handle_a, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    g_detachDestroyedA.store(0);
    g_detachDestroyedB.store(0);
    ovphysx_object_change_callbacks_t callbacks_a{};
    callbacks_a.on_all_objects_destroyed = countDetachDestroyedA;
    ovphysx_object_change_callbacks_t callbacks_b{};
    callbacks_b.on_all_objects_destroyed = countDetachDestroyedB;
    ovphysx_subscription_id_t sub_a = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_subscription_id_t sub_b = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ASSERT_EQ(ovphysx_subscribe_object_changes(handle_a, &callbacks_a, &sub_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_subscribe_object_changes(handle_b, &callbacks_b, &sub_b).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(test_utils::complete_task(handle_a, ovphysx_detach_ovstage(handle_a)).status, OVPHYSX_API_SUCCESS);
    EXPECT_GE(g_detachDestroyedA.load(), 1);
    EXPECT_EQ(g_detachDestroyedB.load(), 0);

    expectAttachmentState(handle_a, query_a, false);
    expectAttachmentState(handle_b, query_b, true);
    EXPECT_TRUE(waitForTaskSuccess(handle_a, ovphysx_release_query(handle_a, query_a), 5'000'000'000ULL));
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_a, sub_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle_a));
    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);

    expectAttachmentState(handle_b, query_b, true);
    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_b, sub_b).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(test_utils::complete_task(handle_b, ovphysx_detach_ovstage(handle_b)).status, OVPHYSX_API_SUCCESS);
    expectAttachmentState(handle_b, query_b, false);
    EXPECT_TRUE(waitForTaskSuccess(handle_b, ovphysx_release_query(handle_b, query_b), 5'000'000'000ULL));
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle_b));
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

// One ovstage instance is one attachment. A second instance must not attach it.
TEST(MultiInstance, SecondAttachOfSameOvstageIsRejected) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";

    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(&handle_b).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path, 1, "ovphysx-same-stage"));

    ovstage_instance_t* shared_stage = nullptr;
    {
        std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
        shared_stage = ovstage_test_attachments().at(handle_a).front().stage;
    }
    ASSERT_NE(shared_stage, nullptr);

    ovphysx_query_handle_t query_a = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle_a,
        ovphysx_query(handle_a, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query_a, nullptr),
        5'000'000'000ULL));
    expectAttachmentState(handle_a, query_a, true);

    const ovphysx_enqueue_result_t second = ovphysx_attach_ovstage(handle_b, shared_stage, 1);
    ASSERT_EQ(second.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(second.op_index, 0u);
    EXPECT_EQ(test_utils::complete_task(handle_b, second).status, OVPHYSX_API_ERROR);

    expectAttachmentState(handle_a, query_a, true);
    bool attached_b = true;
    ASSERT_EQ(ovphysx_is_stage_attached(handle_b, nullptr, &attached_b).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attached_b);
    EXPECT_EQ(ovphysx_step_sync(handle_a, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(test_utils::complete_task(handle_a, ovphysx_detach_ovstage(handle_a)).status, OVPHYSX_API_SUCCESS);
    expectAttachmentState(handle_a, query_a, false);
    EXPECT_TRUE(waitForTaskSuccess(handle_a, ovphysx_release_query(handle_a, query_a), 5'000'000'000ULL));
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle_a));
    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

// Concurrent destroy/create retains the shared datastore and serializes loader counts.
// This is loader-lifecycle smoke coverage; it does not exercise SDK/cache teardown.
TEST(MultiInstance, ConcurrentCreateDestroyMaintainsLoaderLifecycle)
{
    const int kIterations = 20;

    ovphysx_instance_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);

    for (int i = 0; i < kIterations; ++i)
    {
        // `handle` is (as far as this process knows) the only live loader: destroying it on a
        // background thread races it against a concurrent create on the main thread.
        ovphysx_result_t destroy_result{};
        std::thread destroyer([handle, &destroy_result]() { destroy_result = ovphysx_destroy_instance(handle); });

        ovphysx_instance_handle_t next_handle = 0;
        ovphysx_result_t create_result = ovphysx_create_instance(&next_handle);

        destroyer.join();

        ASSERT_EQ(destroy_result.status, OVPHYSX_API_SUCCESS) << "iteration " << i;
        ASSERT_EQ(create_result.status, OVPHYSX_API_SUCCESS) << "iteration " << i;
        ASSERT_NE(next_handle, 0u) << "iteration " << i;
        handle = next_handle;
    }

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

namespace
{
void instanceScopeDummyCreated(ovphysx_string_t, ovphysx_physx_type_t, void*) {}

void createTwoHandles(ovphysx_instance_handle_t* handle_a, ovphysx_instance_handle_t* handle_b)
{
    ASSERT_EQ(ovphysx_create_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}
} // namespace

TEST(MultiInstance, ForeignQueryRejectedOnPeerInstance)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    createTwoHandles(&handle_a, &handle_b);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t query_task =
        ovphysx_query(handle_a, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr);
    ASSERT_EQ(query_task.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, query_task));
    ASSERT_NE(query, 0u);

    bool valid = true;
    EXPECT_EQ(ovphysx_internal_is_query_valid(handle_b, query, &valid).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(valid);

    // A session over the query is refused on the peer and still opens on the owner.
    const ovx_string_or_token_t position = { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } };
    ovphysx_read_handle_t foreign_read = 0;
    EXPECT_EQ(ovphysx_read(handle_b, query, &position, 1, &foreign_read, nullptr).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(foreign_read, 0u);
    ovphysx_read_handle_t owner_read = 0;
    const ovphysx_enqueue_result_t owner_read_task = ovphysx_read(handle_a, query, &position, 1, &owner_read, nullptr);
    ASSERT_EQ(owner_read_task.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle_a, owner_read_task));
    const ovphysx_enqueue_result_t release_read = ovphysx_release_read(handle_a, owner_read);
    ASSERT_EQ(release_read.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle_a, release_read));

    EXPECT_EQ(ovphysx_release_query(handle_b, query).status, OVPHYSX_API_NOT_FOUND);
    const ovphysx_enqueue_result_t release_query = ovphysx_release_query(handle_a, query);
    ASSERT_EQ(release_query.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle_a, release_query));
    valid = true;
    EXPECT_EQ(ovphysx_internal_is_query_valid(handle_a, query, &valid).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(valid);

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/envs/env0/table");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t binding_a = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle_a, &desc, &binding_a).status, OVPHYSX_API_SUCCESS);
    ovphysx_tensor_spec_t spec{};
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(handle_b, binding_a, &spec).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(handle_a, binding_a, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(handle_a, binding_a).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_a = ovphysx_detach_ovstage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, reset_a, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_a);
    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

TEST(MultiInstance, ForeignSubscriptionUnsubscribeRejected)
{
    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    createTwoHandles(&handle_a, &handle_b);

    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = instanceScopeDummyCreated;
    ovphysx_subscription_id_t sub = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ASSERT_EQ(ovphysx_subscribe_object_changes(handle_a, &cb, &sub).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_b, sub).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_a, sub).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

TEST(MultiInstance, TaskErrorIsolatedPerInstance)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    createTwoHandles(&handle_a, &handle_b);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_string_t target = OVPHYSX_LITERAL("/World/MissingCloneTarget");
    const ovphysx_enqueue_result_t task =
        ovphysx_clone(handle_a, OVPHYSX_LITERAL("/World/MissingCloneSource"), &target, 1, nullptr, nullptr);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);

    ovphysx_wait_task_result_t wait_result{};
    const ovphysx_result_t wait_status =
        ovphysx_wait_task(handle_a, task, 1'000'000'000ULL, &wait_result);
    EXPECT_EQ(wait_status.status, OVPHYSX_API_ERROR);
    if (wait_result.num_errors >= 1u)
    {
        const ovphysx_enqueue_result_t failed = wait_result.error_tasks[0];
        const ovphysx_string_t err_b = ovphysx_get_last_task_error(handle_b, failed);
        EXPECT_EQ(err_b.length, 0u);
        const ovphysx_string_t err_a = ovphysx_get_last_task_error(handle_a, failed);
        EXPECT_GT(err_a.length, 0u);
    }
    else
        ADD_FAILURE() << "wait_task did not record a failed task";

    ovphysx_destroy_wait_task_result(&wait_result);
    const ovphysx_enqueue_result_t reset_a = ovphysx_detach_ovstage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, reset_a, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_a);
    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

TEST(MultiInstance, DebugRenderCacheIsolatedPerInstance)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    createTwoHandles(&handle_a, &handle_b);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    ASSERT_EQ(debug_render_set_parameter(handle_a, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 2.5f).status,
              OVPHYSX_API_SUCCESS);
    (void)debug_render_set_parameter(handle_b, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 9.0f);

    float value_a = 0.0f;
    ASSERT_EQ(debug_render_get_parameter(handle_a, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value_a).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value_a, 2.5f);

    ovphysx_enqueue_result_t reset_a = ovphysx_detach_ovstage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, reset_a, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_a);
    EXPECT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}

TEST(MultiInstance, DestroyOwnerLeavesPeerReusable)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_instance_handle_t handle_a = 0;
    ovphysx_instance_handle_t handle_b = 0;
    createTwoHandles(&handle_a, &handle_b);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_object_change_callbacks_t cb{};
    cb.on_object_created = instanceScopeDummyCreated;
    ovphysx_subscription_id_t sub_b = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ASSERT_EQ(ovphysx_subscribe_object_changes(handle_b, &cb, &sub_b).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_a = ovphysx_detach_ovstage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_a, reset_a, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_a);
    ASSERT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_unsubscribe_object_changes(handle_b, sub_b).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_b, usd_path));
    EXPECT_EQ(ovphysx_step_sync(handle_b, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_b = ovphysx_detach_ovstage(handle_b);
    ASSERT_EQ(reset_b.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle_b, reset_b, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_b);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}
