// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file test_multi_instance.cpp
 * @brief Tests for creating and destroying multiple PhysX instances
 * 
 * This test verifies that:
 * 1. Multiple instances can be created and destroyed sequentially
 * 2. USD stages can be loaded/unloaded across instance lifecycles
 * 3. Resources are properly cleaned up between instances
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

using namespace test_utils;

TEST(MultiInstance, SequentialCreateDestroy) {
    const int NUM_ITERATIONS = 3;
    
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        
        ovphysx_handle_t handle = 0;
        ovphysx_result_t create_result = ovphysx_create_instance(&args, &handle);
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
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        
        ovphysx_handle_t handle = 0;
        ovphysx_result_t create_result = ovphysx_create_instance(&args, &handle);
        ASSERT_EQ(create_result.status, OVPHYSX_API_SUCCESS) << "Failed to create instance " << (i + 1);
        ASSERT_NE(handle, 0) << "Instance handle should not be 0 for iteration " << (i + 1);
        
        ASSERT_TRUE(attach_usd_with_ovstage(handle, usd_path))
            << "ovstage attach/update failed in iteration " << (i + 1);
        
        // NOTE: No separate sync() call needed - operations are stream-ordered
        
        // Unload USD (reset clears all)
        ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(handle);
        ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS) << "Failed to reset in iteration " << (i + 1);
        ASSERT_TRUE(waitForOperationSuccess(handle, reset_result.op_index, 5000000000ULL))
            << "Reset failed in iteration " << (i + 1);
        destroy_ovstage_test_attachments(handle);
        
        ovphysx_result_t destroy_result = ovphysx_destroy_instance(handle);
        ASSERT_EQ(destroy_result.status, OVPHYSX_API_SUCCESS) << "Failed to destroy instance " << (i + 1);
    }
}

TEST(MultiInstance, MultipleSimultaneousInstances) {
    const int NUM_INSTANCES = 3;
    ovphysx_handle_t handles[NUM_INSTANCES] = {0};

    for (int i = 0; i < NUM_INSTANCES; ++i) {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        ovphysx_result_t result = ovphysx_create_instance(&args, &handles[i]);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create instance " << (i + 1);
        ASSERT_NE(handles[i], 0) << "Instance handle should not be 0 for instance " << (i + 1);
    }

    for (int i = 0; i < NUM_INSTANCES; ++i) {
        ovphysx_result_t result = ovphysx_destroy_instance(handles[i]);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to destroy instance " << (i + 1);
    }
}

// Regression (NVBugs 6433668 MR review): a second, stage-less handle's async
// ovphysx_step() must be rejected rather than silently advancing whatever
// stage another handle has attached. IPhysxSimulation is a process-wide
// singleton (CarboniteLoader::loadPhysxPlugins() -- "the linked PhysX runtime
// is process-wide"), so before this fix a stage-less handle B's
// ovphysx_step() would fall through ensure_physics_attached()'s
// "no stage -> trivial success" branch straight into physxSim->simulate(),
// silently stepping handle A's stage while marking only B's own
// first_step_done -- letting clone() on A's real stage pass its after-step
// guard despite A having genuinely been stepped, through B.
TEST(MultiInstance, StagelessHandleAsyncStepRejectedDoesNotAdvanceOtherHandleStage) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";

    // Handle A: owns the only attached stage.
    ovphysx_create_args args_a = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_a = 0;
    ASSERT_EQ(ovphysx_create_instance(&args_a, &handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    // Handle B: separate instance, never attaches anything.
    ovphysx_create_args args_b = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&args_b, &handle_b).status, OVPHYSX_API_SUCCESS);

    // B's async step() must be rejected outright -- it must never reach
    // physxSim->simulate() and silently advance A's stage.
    ovphysx_enqueue_result_t step_b = ovphysx_step(handle_b, 1.0f / 60.0f);
    EXPECT_NE(step_b.status, OVPHYSX_API_SUCCESS)
        << "Stage-less handle B's async step() should be rejected, not silently advance A's stage";

    // Prove A's stage was never touched by B's rejected call: A must still
    // accept clone() -- it would be rejected if A's first_step_done/
    // gpu_warmup_done had been incorrectly flipped by B's step.
    ovphysx_string_t target1 = ovphysx_cstr("/World/envs/env1");
    ovphysx_enqueue_result_t clone_res = ovphysx_clone(
        handle_a, ovphysx_cstr("/World/envs/env0"), &target1, 1, nullptr, nullptr);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_SUCCESS)
        << "A's clone() guard should be unaffected by B's rejected stage-less step";
    if (clone_res.status == OVPHYSX_API_SUCCESS) {
        EXPECT_TRUE(waitForOperationSuccess(handle_a, clone_res.op_index, 5'000'000'000ULL));
    }

    // Now step A for real, and confirm clone() on A is correctly rejected --
    // the guard still works for genuine same-handle usage.
    ovphysx_enqueue_result_t step_a = ovphysx_step(handle_a, 1.0f / 60.0f);
    ASSERT_EQ(step_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(handle_a, step_a.op_index, 5'000'000'000ULL));

    ovphysx_string_t target2 = ovphysx_cstr("/World/envs/env2");
    ovphysx_enqueue_result_t clone_res2 = ovphysx_clone(
        handle_a, ovphysx_cstr("/World/envs/env0"), &target2, 1, nullptr, nullptr);
    EXPECT_EQ(clone_res2.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "clone() after a genuine step() on A should still be rejected";

    ovphysx_destroy_instance(handle_a);
    ovphysx_destroy_instance(handle_b);
}

// NVBug 6504951, second half. Tensor-binding handles used to count from 1 inside
// each instance, so the first binding of a destroyed instance and the first
// binding of the next instance were both the number 1 and a stale token from the
// first resolved the second instance's live binding. For the valid second
// instance, one process-wide never-reused sequence makes the tensor-binding spec
// lookup below miss the map, so its result has status OVPHYSX_API_NOT_FOUND.
TEST(MultiInstance, TensorBindingHandleIsNotReusedByAnotherInstance)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/envs/env0/table");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_create_args args_a = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_a = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&args_a, &handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    ovphysx_tensor_binding_handle_t binding_a = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle_a, &desc, &binding_a).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t reset_a = ovphysx_reset_stage(handle_a);
    ASSERT_EQ(reset_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(handle_a, reset_a.op_index, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_a);
    ASSERT_EQ(ovphysx_destroy_instance(handle_a).status, OVPHYSX_API_SUCCESS);

    ovphysx_create_args args_b = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_b = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&args_b, &handle_b).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_b, usd_path));

    ovphysx_tensor_binding_handle_t binding_b = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle_b, &desc, &binding_b).status, OVPHYSX_API_SUCCESS);

    EXPECT_NE(binding_a, binding_b);

    ovphysx_tensor_spec_t spec{};
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(handle_b, binding_a, &spec).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(handle_b, binding_b, &spec).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(handle_b, binding_b).status, OVPHYSX_API_SUCCESS);
    ovphysx_enqueue_result_t reset_b = ovphysx_reset_stage(handle_b);
    ASSERT_EQ(reset_b.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(handle_b, reset_b.op_index, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle_b);
    EXPECT_EQ(ovphysx_destroy_instance(handle_b).status, OVPHYSX_API_SUCCESS);
}
