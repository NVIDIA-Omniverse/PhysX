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
 * @implements REQ-CAPI-CACHE-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx_test_utils.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <thread>

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
    ovphysx_create_args args_a = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_a = 0;
    ASSERT_EQ(ovphysx_create_instance(&args_a, &handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    // Handle B: separate instance, never attaches anything.
    ovphysx_create_args args_b = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&args_b, &handle_b).status, OVPHYSX_API_SUCCESS);

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
        EXPECT_TRUE(waitForOperationSuccess(handle_a, clone_res.op_index, 5'000'000'000ULL));
    }

    // Step A for real, then clone() on A is rejected in all modes. The guard
    // enforces the same ordering contract on CPU and GPU.
    ovphysx_enqueue_result_t step_a = ovphysx_step(handle_a, 1.0f / 60.0f);
    ASSERT_EQ(step_a.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(handle_a, step_a.op_index, 5'000'000'000ULL));

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

// Regression (GitLab issue #30, MR !8085): IPhysxSimulation is a process-wide
// singleton and beginSimulationAttach() unconditionally tears down whatever attach
// is currently live, so a second instance's ovphysx_attach_ovstage() must be
// rejected outright while another instance's attach is still live. Silently
// displacing it would leave the first instance's attachHandle stale while its
// runtime attach had been taken over.
TEST(MultiInstance, SecondInstanceAttachRejectedWhileFirstOwnsLiveAttach) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/basic_simulation.usda";

    // Instance A: acquires the one live process-wide attach.
    ovphysx_create_args args_a = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_a = 0;
    ASSERT_EQ(ovphysx_create_instance(&args_a, &handle_a).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle_a, usd_path));

    uint64_t attach_a_before = 0;
    ASSERT_EQ(ovphysx_get_attach_handle(handle_a, &attach_a_before).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(attach_a_before, 0u);

    // Instance B: builds its own ovstage Stage, then tries to attach while A's
    // attach is still live. This must be rejected, not silently displace A.
    ovphysx_create_args args_b = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle_b = 0;
    ASSERT_EQ(ovphysx_create_instance(&args_b, &handle_b).status, OVPHYSX_API_SUCCESS);

    ASSERT_TRUE(test_utils::register_physx_schemas_with_ovstage());
    ovstage_instance_desc_t desc_b{};
    desc_b.name = "ovphysx-test-stage-b";
    ovstage_instance_t* stage_b = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc_b, &stage_b), OVSTAGE_OK);
    ASSERT_NE(stage_b, nullptr);

    ovx_string_t path_b{};
    path_b.ptr = usd_path;
    path_b.length = std::strlen(usd_path);
    const uint64_t ordinal_b = 1;
    ovstage_population_enqueue_result_t enqueue_b = ovstage_population_open_usd_from_file(
        stage_b, path_b, ordinal_b, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
    ASSERT_EQ(enqueue_b.status, OVSTAGE_OK);
    ovstage_population_op_wait_result_t wait_result_b{};
    ASSERT_EQ(ovstage_population_wait_op(stage_b, enqueue_b.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait_result_b),
              OVSTAGE_OK);
    ovstage_write_floor_desc_t floor_desc_b{};
    floor_desc_b.ordinal = ordinal_b;
    floor_desc_b.scope = OVSTAGE_SCOPE_ALL;
    ovstage_enqueue_result_t floor_b = ovstage_advance_write_floor(stage_b, &floor_desc_b);
    ASSERT_EQ(floor_b.status, OVSTAGE_OK);
    ovstage_op_wait_result_t floor_wait_b{};
    ASSERT_EQ(ovstage_wait_op(stage_b, floor_b.op_index, OVSTAGE_TIMEOUT_INFINITE, &floor_wait_b), OVSTAGE_OK);
    ASSERT_EQ(floor_wait_b.error_op_id_count, 0u);
    ASSERT_EQ(ovstage_release_op(stage_b, floor_b.op_index), OVSTAGE_OK);

    // The regression assertion: B's attach must be rejected while A's attach is
    // live, not silently succeed and take over the runtime's one live attach.
    ovphysx_result_t attach_b_result = ovphysx_attach_ovstage(handle_b, stage_b, ordinal_b);
    EXPECT_EQ(attach_b_result.status, OVPHYSX_API_ERROR)
        << "A second instance's attach_ovstage() must be rejected while another "
           "instance still owns the live process-wide PhysX attach";

    uint64_t attach_b_after = 0;
    ovphysx_get_attach_handle(handle_b, &attach_b_after);
    EXPECT_EQ(attach_b_after, 0u) << "B must not have acquired an attach handle";

    // A's own bookkeeping must be untouched by B's rejected attempt.
    uint64_t attach_a_after = 0;
    ASSERT_EQ(ovphysx_get_attach_handle(handle_a, &attach_a_after).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(attach_a_after, attach_a_before) << "A's attach handle must survive B's rejected attach attempt";

    // A's underlying runtime attach must still be alive and steppable, which shows
    // B's rejected attempt never reached beginSimulationAttach()'s unconditional
    // detachStage() of the live attach.
    EXPECT_EQ(ovphysx_step_sync(handle_a, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    // B never validly attached, so its detach must be a harmless no-op and must
    // not reach into and tear down A's live attach either.
    EXPECT_EQ(ovphysx_detach_ovstage(handle_b).status, OVPHYSX_API_SUCCESS);

    uint64_t attach_a_final = 0;
    ASSERT_EQ(ovphysx_get_attach_handle(handle_a, &attach_a_final).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(attach_a_final, attach_a_before) << "A's attach handle must survive B's no-op detach";
    EXPECT_EQ(ovphysx_step_sync(handle_a, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    ovstage_destroy_instance(stage_b);
    destroy_ovstage_test_attachments(handle_a);
    ovphysx_destroy_instance(handle_a);
    ovphysx_destroy_instance(handle_b);
}

// Covers the process-private cooked-collider cache retry loop in releaseProcessCacheDirLocked()
// (MR !8200). Attaching a USD stage forces CarboniteLoader::loadPhysxPlugins() to run at least
// once in this process, which is what seeds g_processCacheDir (repeat instance creation reuses
// the process-wide PhysX singleton and skips that seeding). Destroying the last live instance
// must then remove the process-private cache on the first attempt and never log the
// exhausted-retries WARN.
TEST(MultiInstance, ProcessCacheCleanupSucceedsSilentlyOnLastLoaderShutdown)
{
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda";

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle, usd_path));

    ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(handle);
    ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(handle, reset_result.op_index, 5'000'000'000ULL));
    destroy_ovstage_test_attachments(handle);

    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);

    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "Failed to remove process-private cache"))
        << "Last-loader shutdown should remove the process-private cooked-collider cache without "
           "exhausting its retry budget in the uncontended case";

    ovphysx_log_capture_stop();
}

// The last-loader decision (g_activeLoaders hitting zero) and the process-private cache cleanup
// it triggers must run under g_bootstrapMutex (MR !8200). Otherwise a concurrent
// ovphysx_create_instance(), which is documented safe to call from any thread, can become the
// new first loader in the process while the old loader's shutdown is still tearing the cache
// directory down underneath it. CarboniteLoader::shutdown() takes g_bootstrapMutex around that
// decrement and cleanup, the same mutex initialize() holds across its own increment. This loop
// repeatedly races a destroy against a create and must complete cleanly: no crash, no hang, and
// no spurious "failed to remove" WARN from the cleanup observing a half-initialized concurrent
// loader.
TEST(MultiInstance, ProcessCacheCleanupSurvivesConcurrentCreateDestroyRace)
{
    const int kIterations = 20;

    ovphysx_create_args seed_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&seed_args, &handle).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);

    for (int i = 0; i < kIterations; ++i)
    {
        // `handle` is (as far as this process knows) the only live loader: destroying it on a
        // background thread races it against a concurrent create on the main thread.
        ovphysx_result_t destroy_result{};
        std::thread destroyer([handle, &destroy_result]() { destroy_result = ovphysx_destroy_instance(handle); });

        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_handle_t next_handle = 0;
        ovphysx_result_t create_result = ovphysx_create_instance(&args, &next_handle);

        destroyer.join();

        ASSERT_EQ(destroy_result.status, OVPHYSX_API_SUCCESS) << "iteration " << i;
        ASSERT_EQ(create_result.status, OVPHYSX_API_SUCCESS) << "iteration " << i;
        ASSERT_NE(next_handle, 0u) << "iteration " << i;
        handle = next_handle;
    }

    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "Failed to remove process-private cache"))
        << "A destroy/create race must not corrupt the process-private cache cleanup";

    ovphysx_log_capture_stop();

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}
