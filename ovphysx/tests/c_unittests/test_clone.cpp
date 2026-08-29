// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// C-API tests for ovphysx_clone (replicator-backed). Clones are physics-only: live PhysX bodies
// addressable by their target path via the tensor API, not authored as USD prims -- so a clone is
// verified by reading its body pose through a tensor binding. basic_simulation.usda places one
// rigid body 'table' under each env.

#include "global_test_environment.h"  // PhysXTestFixture, waitForOperationSuccess, ovphysx.h
#include "test_utilities.h"

#include <algorithm>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using test_utils::make_ovx_string;

// Attach an ovstage-backed USD scene and drain it (matches the other c_unittests' helper).
static bool load_usd_and_wait(ovphysx_handle_t handle, const char* usd_path, ovphysx_usd_handle_t& out_handle)
{
    (void)out_handle;
    return test_utils::attach_usd_with_ovstage(handle, usd_path);
}

static ovphysx_enqueue_result_t enqueue_clone(ovphysx_handle_t handle,
                                              const char* source_path,
                                              const char* const* targets,
                                              uint32_t num_targets)
{
    std::vector<ovphysx_string_t> target_vec;
    target_vec.reserve(num_targets);
    for (uint32_t i = 0; i < num_targets; ++i)
    {
        target_vec.push_back(make_ovx_string(targets[i]));
    }
    return ovphysx_clone(handle, make_ovx_string(source_path), target_vec.data(), num_targets, nullptr, nullptr);
}

// Read a rigid body's world position via a tensor binding (position = first 3 of the [1,7] pose).
static bool read_position(ovphysx_handle_t handle, const char* prim_path, double* out_pos)
{
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string(prim_path);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t res = ovphysx_create_tensor_binding(handle, &desc, &binding_handle);
    if (res.status != OVPHYSX_API_SUCCESS)
    {
        return false;
    }

    float pose_data[7] = {};
    int64_t shape[] = { 1, 7 };
    DLTensor tensor{};
    tensor.data = pose_data;
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.dtype = { kDLFloat, 32, 1 };
    tensor.device = { kDLCPU, 0 };

    res = ovphysx_read_tensor_binding(handle, binding_handle, &tensor);
    ovphysx_destroy_tensor_binding(handle, binding_handle);
    if (res.status != OVPHYSX_API_SUCCESS)
    {
        return false;
    }

    out_pos[0] = static_cast<double>(pose_data[0]);
    out_pos[1] = static_cast<double>(pose_data[1]);
    out_pos[2] = static_cast<double>(pose_data[2]);
    return true;
}

// A clone "exists" when its replicated rigid body is addressable: a tensor binding on the clone
// body path resolves and reads a pose.
static bool verify_clones_exist(ovphysx_handle_t handle, const std::vector<std::string>& env_paths)
{
    for (const std::string& env : env_paths)
    {
        double pos[3];
        if (!read_position(handle, (env + "/table").c_str(), pos))
        {
            std::cerr << "Clone verification failed: no physics body at " << env << "/table" << std::endl;
            return false;
        }
    }
    return true;
}

static bool read_binding_prim_paths(ovphysx_handle_t handle,
                                    ovphysx_tensor_binding_handle_t binding,
                                    uint32_t max_paths,
                                    std::vector<std::string>& out_paths)
{
    std::vector<ovphysx_string_t> path_views(max_paths);
    uint32_t out_count = 0;
    const ovphysx_result_t result =
        ovphysx_tensor_binding_get_prim_paths(handle, binding, path_views.data(), max_paths, &out_count);
    if (result.status != OVPHYSX_API_SUCCESS)
        return false;

    out_paths.clear();
    out_paths.reserve(out_count);
    for (uint32_t i = 0; i < out_count; ++i)
    {
        const ovphysx_string_t& path = path_views[i];
        out_paths.emplace_back(path.ptr ? path.ptr : "", path.ptr ? path.length : 0);
    }
    return true;
}

static void step_and_wait(ovphysx_handle_t handle, float dt)
{
    ovphysx_enqueue_result_t step_res = ovphysx_step(handle, dt);
    ASSERT_EQ(step_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(handle, step_res.op_index));
}

class CloneTest : public PhysXTestFixture
{
protected:
    bool load_basic_usd(ovphysx_usd_handle_t& usd_handle)
    {
        return load_usd_and_wait(m_handle, "tests/data/basic_simulation.usda", usd_handle);
    }
};

TEST_F(CloneTest, ErrorInvalidParameters)
{
    ovphysx_string_t target = make_ovx_string("/World/clone1");

    ovphysx_enqueue_result_t nullSource =
        ovphysx_clone(m_handle, make_ovx_string(nullptr), &target, 1, nullptr, nullptr);
    EXPECT_EQ(nullSource.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (nullSource.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, nullSource.op_index, 2'000'000'000ULL));
    }

    ovphysx_enqueue_result_t nullTargets =
        ovphysx_clone(m_handle, make_ovx_string("/World/table"), nullptr, 1, nullptr, nullptr);
    EXPECT_EQ(nullTargets.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (nullTargets.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, nullTargets.op_index, 2'000'000'000ULL));
    }

    ovphysx_enqueue_result_t zeroTargets =
        ovphysx_clone(m_handle, make_ovx_string("/World/table"), nullptr, 0, nullptr, nullptr);
    EXPECT_EQ(zeroTargets.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (zeroTargets.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, zeroTargets.op_index, 2'000'000'000ULL));
    }
}

TEST_F(CloneTest, ErrorNoStageLoaded)
{
    ovphysx_string_t target = make_ovx_string("/World/clone1");
    ovphysx_enqueue_result_t res =
        ovphysx_clone(m_handle, make_ovx_string("/World/table"), &target, 1, nullptr, nullptr);
    EXPECT_EQ(res.status, OVPHYSX_API_ERROR);
    if (res.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
    }
}

TEST_F(CloneTest, ErrorSourceDoesNotExist)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle));

    const char* targets[] = { "/World/envs/envX" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/nonexistent", targets, 1);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_ERROR);
    EXPECT_EQ(clone_res.op_index, 0u) << "a synchronous clone error must not register an async op";
}

TEST_F(CloneTest, CloneSingleTarget)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "ovstage attach/update failed";

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index));

    std::vector<std::string> target_vec(targets, targets + 1);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "Clone body not addressable at target path";

    // No duplicate actors: env0 parsed at attach, env1 created by replicate. Overlapping duplicates
    // would explode into NaNs; a stable 10-step run confirms exactly one body per env.
    for (int i = 0; i < 10; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, CloneMultipleTargets)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = {
        "/World/envs/env1", "/World/envs/env2", "/World/envs/env3", "/World/envs/env4", "/World/envs/env5",
        "/World/envs/env6", "/World/envs/env7", "/World/envs/env8", "/World/envs/env9", "/World/envs/env10"
    };
    const uint32_t num_targets = sizeof(targets) / sizeof(targets[0]);

    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, num_targets);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index));

    std::vector<std::string> target_vec(targets, targets + num_targets);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "One or more clone bodies not addressable";

    for (int i = 0; i < 10; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, CloneWithParentTransforms)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1", "/World/envs/env2" };
    const uint32_t num_targets = sizeof(targets) / sizeof(targets[0]);

    // env1 at (5,0,0), env2 at (10,0,0), identity rotation.
    float transforms[] = {
        5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };

    std::vector<ovphysx_string_t> target_ovx;
    for (const char* t : targets)
    {
        target_ovx.push_back(make_ovx_string(t));
    }
    ovphysx_enqueue_result_t clone_res =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), num_targets, transforms, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 5'000'000'000ULL))
        << "Clone with transforms timed out or failed";

    step_and_wait(m_handle, 1.0f / 60.0f);

    double env0_pos[3], env1_pos[3], env2_pos[3];
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env0/table", env0_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env1/table", env1_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env2/table", env2_pos));

    const double tol = 0.5;
    EXPECT_NEAR(env1_pos[0] - env0_pos[0], 5.0, tol) << "env1 X offset does not match parent transform";
    EXPECT_NEAR(env2_pos[0] - env0_pos[0], 10.0, tol) << "env2 X offset does not match parent transform";
    EXPECT_NEAR(env1_pos[2] - env0_pos[2], 0.0, tol) << "env1 Z offset should be ~0";
    EXPECT_NEAR(env2_pos[2] - env0_pos[2], 0.0, tol) << "env2 Z offset should be ~0";
}

TEST_F(CloneTest, CloneWithParentRotation)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1" };

    // 90-degree yaw about Y: quat (0, sin45, 0, cos45); placed at (20,0,0).
    const float s = 0.7071068f;
    float transforms[] = { 20.0f, 0.0f, 0.0f, 0.0f, s, 0.0f, s };

    std::vector<ovphysx_string_t> target_ovx;
    target_ovx.push_back(make_ovx_string(targets[0]));
    ovphysx_enqueue_result_t clone_res =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), 1, transforms, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 5'000'000'000ULL))
        << "Clone with rotation timed out or failed";

    step_and_wait(m_handle, 1.0f / 60.0f);

    double env0_pos[3], env1_pos[3];
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env0/table", env0_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env1/table", env1_pos));

    // 90-degree yaw maps local +X -> +Z and local +Z -> -X, so env1/table lands at ~(20 - dz, dy, dx).
    const double dx = env0_pos[0];
    const double dz = env0_pos[2];
    const double tol = 1.0;
    EXPECT_NEAR(env1_pos[0], 20.0 - dz, tol) << "Rotated clone X does not match expected value";
    EXPECT_NEAR(env1_pos[2], dx, tol) << "Rotated clone Z does not match expected value";
}

TEST_F(CloneTest, StepWithoutClone)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    step_and_wait(m_handle, 1.0f / 60.0f);
    step_and_wait(m_handle, 1.0f / 60.0f);
}

TEST_F(CloneTest, ErrorCloneAfterStepRejectedOnCpu)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle));

    step_and_wait(m_handle, 1.0f / 60.0f);

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(clone_res.op_index, 0u);
}

TEST_F(CloneTest, WarmupWithoutClone)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    ovphysx_result_t res = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(res.status, OVPHYSX_API_SUCCESS) << "warmup_gpu failed without clone";

    step_and_wait(m_handle, 1.0f / 60.0f);
}

TEST_F(CloneTest, TensorReadWithoutClone)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    double pos[3] = {};
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env0/table", pos)) << "Tensor read without clone failed";
}

TEST_F(CloneTest, ValidationClonesCanBeSimulated)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = {
        "/World/envs/env1", "/World/envs/env2", "/World/envs/env3", "/World/envs/env4", "/World/envs/env5"
    };
    const uint32_t num_targets = sizeof(targets) / sizeof(targets[0]);

    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, num_targets);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 2'000'000'000ULL)) << "Clone timed out or failed";

    std::vector<std::string> target_vec(targets, targets + num_targets);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "One or more clone bodies not addressable";

    step_and_wait(m_handle, 1.0f / 60.0f);
}

TEST_F(CloneTest, ValidationMediumScale50Clones)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // Clone before the first step/warmup (the clone contract): the first step auto-warms GPU
    // buffers, after which cloning is rejected.
    std::vector<std::string> target_paths;
    std::vector<ovphysx_string_t> target_ovx;
    for (int i = 1; i <= 50; ++i)
    {
        target_paths.push_back("/World/envs/env" + std::to_string(i));
        target_ovx.push_back(make_ovx_string(target_paths.back().c_str()));
    }

    ovphysx_enqueue_result_t clone_res =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), 50, nullptr, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 5'000'000'000ULL))
        << "Clone timed out or failed (50 clones)";

    for (int i = 0; i < 60; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, RegressionLargeBatchRetainsTargetPathsForBinding)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    constexpr uint32_t kNumClones = 1023;
    std::vector<std::string> target_paths;
    target_paths.reserve(kNumClones);
    for (uint32_t i = 1; i <= kNumClones; ++i)
    {
        target_paths.push_back("/World/envs/env" + std::to_string(i));
    }

    std::vector<ovphysx_string_t> target_ovx;
    target_ovx.reserve(kNumClones);
    for (const std::string& target_path : target_paths)
    {
        target_ovx.push_back(make_ovx_string(target_path.c_str()));
    }

    std::vector<std::string> expected_body_paths;
    expected_body_paths.reserve(kNumClones + 1);
    expected_body_paths.push_back("/World/envs/env0/table");
    for (const std::string& target_path : target_paths)
    {
        expected_body_paths.push_back(target_path + "/table");
    }
    std::sort(expected_body_paths.begin(), expected_body_paths.end());

    ovphysx_enqueue_result_t clone_res = ovphysx_clone(
        m_handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), kNumClones, nullptr, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 20'000'000'000ULL))
        << "Large clone batch timed out or failed";

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("/World/envs/env*/table");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t binding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS)
        << "Failed to create tensor binding";

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS)
        << "Failed to get tensor spec";
    EXPECT_EQ(spec.shape[0], static_cast<int64_t>(kNumClones + 1))
        << "Every cloned target path must remain addressable through tensor matching";
    EXPECT_EQ(spec.shape[1], 7) << "Pose tensor should be [N, 7]";

    std::vector<std::string> wildcard_paths;
    ASSERT_TRUE(read_binding_prim_paths(
        m_handle, binding, static_cast<uint32_t>(expected_body_paths.size()), wildcard_paths));
    std::sort(wildcard_paths.begin(), wildcard_paths.end());
    EXPECT_EQ(wildcard_paths, expected_body_paths) << "Wildcard binding returned wrong or duplicate clone paths";
    ovphysx_destroy_tensor_binding(m_handle, binding);

    std::vector<ovphysx_string_t> explicit_path_views;
    explicit_path_views.reserve(expected_body_paths.size());
    for (const std::string& expected_path : expected_body_paths)
    {
        explicit_path_views.push_back(make_ovx_string(expected_path.c_str()));
    }

    ovphysx_tensor_binding_desc_t explicit_desc{};
    explicit_desc.prim_paths = explicit_path_views.data();
    explicit_desc.prim_paths_count = static_cast<uint32_t>(explicit_path_views.size());
    explicit_desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t explicit_binding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &explicit_desc, &explicit_binding).status, OVPHYSX_API_SUCCESS)
        << "Failed to create explicit-path tensor binding";

    ovphysx_tensor_spec_t explicit_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, explicit_binding, &explicit_spec).status,
              OVPHYSX_API_SUCCESS)
        << "Failed to get explicit-path tensor spec";
    EXPECT_EQ(explicit_spec.shape[0], static_cast<int64_t>(expected_body_paths.size()))
        << "Explicit binding must resolve every requested clone path";
    EXPECT_EQ(explicit_spec.shape[1], 7) << "Explicit pose tensor should be [N, 7]";

    std::vector<std::string> explicit_paths;
    ASSERT_TRUE(read_binding_prim_paths(
        m_handle, explicit_binding, static_cast<uint32_t>(expected_body_paths.size()), explicit_paths));
    std::sort(explicit_paths.begin(), explicit_paths.end());
    EXPECT_EQ(explicit_paths, expected_body_paths) << "Explicit binding returned wrong or duplicate clone paths";
    ovphysx_destroy_tensor_binding(m_handle, explicit_binding);
}

static ovphysx_enqueue_result_t enqueue_clone_count(ovphysx_handle_t handle, uint32_t num_targets)
{
    std::vector<std::string> target_paths;
    std::vector<ovphysx_string_t> target_ovx;
    target_paths.reserve(num_targets);
    target_ovx.reserve(num_targets);
    for (uint32_t i = 1; i <= num_targets; ++i)
    {
        target_paths.push_back("/World/envs/env" + std::to_string(i));
        target_ovx.push_back(make_ovx_string(target_paths.back().c_str()));
    }
    return ovphysx_clone(handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), num_targets, nullptr,
                         nullptr);
}

// NVBugs 6473884: the filed repro cycles reset_stage() -> reload -> clone again at large N.
// RegressionLargeBatchRetainsTargetPathsForBinding covers the single-batch binding path.
TEST_F(CloneTest, RegressionLargeBatchCloneAfterReset_NVBugs6473884)
{
    constexpr uint32_t kNumTargets = 512;

    ovphysx_usd_handle_t usd_handle_first = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle_first)) << "first cycle: USD load failed";

    ovphysx_enqueue_result_t clone_res_first = enqueue_clone_count(m_handle, kNumTargets);
    ASSERT_EQ(clone_res_first.status, OVPHYSX_API_SUCCESS) << "first cycle: clone enqueue failed";
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res_first.op_index, 30'000'000'000ULL))
        << "first cycle: large clone timed out or failed";

    const std::vector<std::string> spot_check = { "/World/envs/env1",
                                                  "/World/envs/env" + std::to_string(kNumTargets / 2),
                                                  "/World/envs/env" + std::to_string(kNumTargets) };
    ASSERT_TRUE(verify_clones_exist(m_handle, spot_check))
        << "first cycle: clone bodies not addressable after large batch";

    for (int i = 0; i < 10; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }

    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForOperationSuccess(m_handle, reset.op_index, 5'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    ovphysx_usd_handle_t usd_handle_second = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle_second)) << "second cycle: USD load failed";

    ovphysx_enqueue_result_t clone_res_second = enqueue_clone_count(m_handle, kNumTargets);
    ASSERT_EQ(clone_res_second.status, OVPHYSX_API_SUCCESS) << "second cycle: clone enqueue failed";
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res_second.op_index, 30'000'000'000ULL))
        << "second cycle: large clone timed out or failed after reset";

    ASSERT_TRUE(verify_clones_exist(m_handle, spot_check))
        << "second cycle: clone bodies not addressable after reset + large batch";

    ovphysx_tensor_binding_desc_t wildcard_desc{};
    wildcard_desc.pattern = make_ovx_string("/World/envs/env*/table");
    wildcard_desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t wildcard_binding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &wildcard_desc, &wildcard_binding).status,
              OVPHYSX_API_SUCCESS)
        << "second cycle: failed to create wildcard tensor binding";

    ovphysx_tensor_spec_t wildcard_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, wildcard_binding, &wildcard_spec).status,
              OVPHYSX_API_SUCCESS)
        << "second cycle: failed to get wildcard tensor spec";
    EXPECT_EQ(wildcard_spec.shape[0], static_cast<int64_t>(kNumTargets + 1))
        << "second cycle: every cloned target path must remain addressable through wildcard binding";
    ovphysx_destroy_tensor_binding(m_handle, wildcard_binding);
}

TEST_F(CloneTest, ValidationNonSequentialTargetPaths)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // Non-standard target paths prove the system honors caller-supplied paths (it isn't
    // auto-generating env1/env2/...).
    const char* targets[] = {
        "/World/envs/custom_clone_A", "/World/envs/custom_clone_B", "/World/envs/test_env_99",
        "/World/different/path/structure", "/World/envs/env_with_underscores_123"
    };
    const uint32_t num_targets = sizeof(targets) / sizeof(targets[0]);

    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, num_targets);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 2'000'000'000ULL))
        << "Clone operation timed out or failed";

    std::vector<std::string> target_vec(targets, targets + num_targets);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "Clone bodies not addressable at the requested paths";

    for (int i = 0; i < 30; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, ValidationMultipleCloneOperations)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // Both clone operations run before the first step (the clone contract): a clone must precede
    // GPU warmup / the first step.
    const char* targets1[] = { "/World/envs/batch1_clone_A", "/World/envs/batch1_clone_B" };
    ovphysx_enqueue_result_t clone_res1 = enqueue_clone(m_handle, "/World/envs/env0", targets1, 2);
    ASSERT_EQ(clone_res1.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res1.op_index, 2'000'000'000ULL)) << "First clone operation failed";

    const char* targets2[] = { "/World/envs/batch2_clone_X", "/World/envs/batch2_clone_Y", "/World/envs/batch2_clone_Z" };
    ovphysx_enqueue_result_t clone_res2 = enqueue_clone(m_handle, "/World/envs/env0", targets2, 3);
    ASSERT_EQ(clone_res2.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res2.op_index, 2'000'000'000ULL)) << "Second clone operation failed";

    for (int i = 0; i < 30; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, RegressionNoDuplicateActorsFromAttachStage)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const uint32_t NUM_CLONES = 4;
    const uint32_t TOTAL_ENVS = NUM_CLONES + 1;  // env0 (source) + 4 clones

    const char* targets[] = { "/World/envs/env1", "/World/envs/env2", "/World/envs/env3", "/World/envs/env4" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, NUM_CLONES);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 5'000'000'000ULL)) << "Clone timed out or failed";

    ovphysx_result_t warmup_res = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(warmup_res.status, OVPHYSX_API_SUCCESS) << "warmup_gpu failed";

    // A binding over every env's body must resolve exactly TOTAL_ENVS actors. A count of
    // TOTAL_ENVS+1 would mean env0 was parsed twice (the duplicate-actor bug).
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("/World/envs/env*/table");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS)
        << "Failed to create tensor binding";

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS)
        << "Failed to get tensor spec";
    EXPECT_EQ(spec.shape[0], static_cast<int64_t>(TOTAL_ENVS))
        << "Expected exactly " << TOTAL_ENVS << " rigid bodies (source + " << NUM_CLONES << " clones), got "
        << spec.shape[0];
    EXPECT_EQ(spec.shape[1], 7) << "Pose tensor should be [N, 7]";

    const int64_t N = spec.shape[0];
    std::vector<float> poses(static_cast<size_t>(N) * 7, 0.0f);
    int64_t shape[2] = { N, 7 };
    DLTensor tensor{};
    tensor.data = poses.data();
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.dtype = { kDLFloat, 32, 1 };
    tensor.device = { kDLCPU, 0 };
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS) << "Tensor read failed";

    for (int64_t i = 0; i < N * 7; ++i)
    {
        ASSERT_FALSE(std::isnan(poses[i])) << "NaN in rigid body pose at index " << i;
    }
    ovphysx_destroy_tensor_binding(m_handle, binding);

    for (int i = 0; i < 30; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

// A successful clone registers a replicator for the backing stage id; the seam must unregister it so
// a later reset_stage() -> reload is a normal parse, not the replicator re-attach path (which sets
// loadPhysics=false and parses no physics). Reload and a fresh clone must both work.
TEST_F(CloneTest, RegressionReplicatorUnregisteredForReattachAfterReset)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1", "/World/envs/env2" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, 2);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res.op_index, 5'000'000'000ULL)) << "Clone timed out or failed";
    std::vector<std::string> target_vec(targets, targets + 2);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "Clone bodies not addressable";

    // reset_stage() detaches; the clone's replicator registration must not survive it.
    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForOperationSuccess(m_handle, reset.op_index, 3'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    // Reload the same scene and clone again -- both before any step/tensor read, which auto-warms
    // and would trip the clone-before-warmup guard. If the stale registration had survived, the
    // re-attach of the reused backing stage id would take the replicator path (loadPhysics=false)
    // and parse no physics, so the reloaded source would have no body to clone.
    ovphysx_usd_handle_t usd_handle2 = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle2)) << "reload after reset failed";

    ovphysx_enqueue_result_t clone_res2 = enqueue_clone(m_handle, "/World/envs/env0", targets, 2);
    ASSERT_EQ(clone_res2.status, OVPHYSX_API_SUCCESS) << "second clone (after reset+reload) failed";
    ASSERT_TRUE(waitForOperationSuccess(m_handle, clone_res2.op_index, 5'000'000'000ULL)) << "second clone timed out";

    // Source + both clone bodies must be addressable: proves the reload parsed physics normally and
    // the fresh clone materialized. (Tensor reads here auto-warm, which is fine post-clone.)
    std::vector<std::string> verify_paths = { "/World/envs/env0", "/World/envs/env1", "/World/envs/env2" };
    ASSERT_TRUE(verify_clones_exist(m_handle, verify_paths))
        << "source or clone bodies unaddressable after reset+reload -- the re-attach was hijacked";
}

// Per-target validation (targets must be valid, unique, and not already exist). Each bad target must
// be rejected before the runtime call, so it cannot create a duplicate object under an existing path.
TEST_F(CloneTest, ErrorEmptyTargetPath)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    ovphysx_string_t target = make_ovx_string(""); // empty target
    ovphysx_enqueue_result_t res =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), &target, 1, nullptr, nullptr);
    EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (res.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
    }
}

TEST_F(CloneTest, ErrorTargetEqualsSource)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env0" }; // same as source
    ovphysx_enqueue_result_t res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (res.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
    }
}

// A length-tagged target that embeds a NUL must be rejected C-first: the seam receives a C string,
// so c_str() would truncate ("/env0\0/tail" -> "/env0") and could silently alias the source or
// another prim. The length-based source comparison alone does not catch this.
TEST_F(CloneTest, ErrorTargetWithEmbeddedNul)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // strlen would stop at the source path, but the tagged length spans the NUL and the tail.
    const std::string embedded = std::string("/World/envs/env0") + '\0' + "/tail";
    ovphysx_string_t target{ embedded.data(), embedded.size() };
    ovphysx_enqueue_result_t res =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), &target, 1, nullptr, nullptr);
    EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (res.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
    }
}

TEST_F(CloneTest, ErrorSourceWithEmbeddedNul)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const std::string embedded = std::string("/World/envs/env0") + '\0' + "/tail";
    ovphysx_string_t source{ embedded.data(), embedded.size() };
    ovphysx_string_t target = make_ovx_string("/World/envs/env1");
    ovphysx_enqueue_result_t res = ovphysx_clone(m_handle, source, &target, 1, nullptr, nullptr);
    EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(res.op_index, 0u);
}

// A target must be a non-root absolute prim path. A relative path, a property path, the root, or a
// malformed string would otherwise build a bad SdfPath / database key at the seam. Rejected before
// the replicator (the USD-aware seam returns failure).
TEST_F(CloneTest, ErrorTargetNotAbsolutePrimPath)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    for (const char* bad : { "env1",                 // relative
                             "/World/envs/env1.attr", // property path
                             "/",                      // root
                             "//World//env1" })        // malformed
    {
        const char* targets[] = { bad };
        ovphysx_enqueue_result_t res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
        EXPECT_NE(res.status, OVPHYSX_API_SUCCESS) << "target '" << bad << "' must be rejected";
        if (res.op_index != 0)
        {
            EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
        }
    }
}

// env_ids map to runtime environment ids as env_ids[i] + 1, and PhysX requires every id to be
// < 1<<24 (PxActor/PxAggregate::setEnvironmentID). So the caller id must be < 0x00FFFFFF: the
// boundary value 0x00FFFFFF (-> runtime 1<<24) is rejected, 0x00FFFFFE (-> runtime 0x00FFFFFF,
// the max valid) is accepted. Rejected C-first.
TEST_F(CloneTest, ErrorEnvIdOutOfRange)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // Just over the boundary and both former (too-wide) sentinels must all be rejected.
    for (uint32_t bad : { 0x00FFFFFFu, 0x01000000u, 0xFFFFFFFEu, 0xFFFFFFFFu })
    {
        ovphysx_string_t target = make_ovx_string("/World/envs/env1");
        ovphysx_enqueue_result_t res =
            ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), &target, 1, nullptr, &bad);
        EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT) << "env_id " << bad << " must be rejected";
        if (res.op_index != 0)
        {
            EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
        }
    }

    // The maximum valid caller id (0x00FFFFFE -> runtime 0x00FFFFFF) must pass validation (the
    // clone itself succeeds on CPU where env-id filtering does not engage).
    uint32_t maxId = 0x00FFFFFEu;
    ovphysx_string_t okTarget = make_ovx_string("/World/envs/env1");
    ovphysx_enqueue_result_t ok =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), &okTarget, 1, nullptr, &maxId);
    EXPECT_EQ(ok.status, OVPHYSX_API_SUCCESS) << "max valid env_id 0x00FFFFFE must be accepted";
    if (ok.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_TRUE(waitForOperationSuccess(m_handle, ok.op_index, 5'000'000'000ULL));
    }
}

// Caller-supplied logical env ids pass end-to-end: the CPU fixture never engages env-id filtering
// (GPU dynamics + broadphase gate), so this validates marshaling + the clone still succeeding and
// simulating. The id semantics themselves (same id across calls -> same runtime environment,
// contact regression) are exercised GPU-side in the ovruntime doctest
// "Replicator clone explicit env-ids compose across batches".
TEST_F(CloneTest, CloneWithExplicitEnvIds)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[2] = { "/World/envs/env1", "/World/envs/env2" };
    float transforms[2 * 7] = {
        5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };
    const uint32_t env_ids[2] = { 7, 3 }; // arbitrary logical ids, deliberately non-positional

    std::vector<ovphysx_string_t> target_ovx;
    for (const char* t : targets)
    {
        target_ovx.push_back(make_ovx_string(t));
    }
    ovphysx_enqueue_result_t res = ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), 2,
                             transforms, env_ids);
    ASSERT_EQ(res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, res.op_index, 5'000'000'000ULL))
        << "Clone with env_ids timed out or failed";

    step_and_wait(m_handle, 1.0f / 60.0f);

    double env1_pos[3], env2_pos[3];
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env1/table", env1_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env2/table", env2_pos));
}

TEST_F(CloneTest, ErrorDuplicateTargetInBatchDoesNotWedgeReset)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1", "/World/envs/env1" }; // duplicate in one call
    ovphysx_enqueue_result_t res = enqueue_clone(m_handle, "/World/envs/env0", targets, 2);
    EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(res.op_index, 0u);

    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForOperationSuccess(m_handle, reset.op_index, 3'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    ovphysx_usd_handle_t reloaded_handle = 0;
    ASSERT_TRUE(load_basic_usd(reloaded_handle)) << "reload after rejected clone failed";
    const char* valid_targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t valid = enqueue_clone(m_handle, "/World/envs/env0", valid_targets, 1);
    ASSERT_EQ(valid.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, valid.op_index, 5'000'000'000ULL));
}

TEST_F(CloneTest, ErrorTargetReusedAcrossBatches)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t first = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, first.op_index, 5'000'000'000ULL)) << "first clone failed";

    // env1 now exists; reusing it on the same attach must be rejected.
    ovphysx_enqueue_result_t second = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(second.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (second.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, second.op_index, 2'000'000'000ULL));
    }
}

// A target already populated with physics from the INITIAL PARSE (not from a prior clone) must be
// rejected before replicate(), otherwise it would add duplicate actors under an existing path.
// Distinct from ErrorTargetReusedAcrossBatches (prior-clone targets, caught C-first by
// cloned_target_paths -> INVALID_ARGUMENT): here the runtime ObjectDb subtree check in the seam
// catches it, so the clone is rejected (OVPHYSX_API_ERROR) before replicate() can mutate anything.
TEST_F(CloneTest, ErrorTargetAlreadyPopulated)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // /World/envs/env0/table is a rigid body from the parse; cloning onto it must be rejected.
    const char* targets[] = { "/World/envs/env0/table" };
    ovphysx_enqueue_result_t res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(res.status, OVPHYSX_API_ERROR);
    if (res.op_index != 0)
    {
        EXPECT_FALSE(waitForOperationSuccess(m_handle, res.op_index, 2'000'000'000ULL));
    }
}

// A synchronous clone error must not register a failed async op (op_index 0). Otherwise
// wait_for_all_pending_ops() keeps the orphan and every later attach/reset/clone fails until an
// undocumented wait_all() consumes it. Here a clone-before-attach error is followed by a normal
// attach + clone that must succeed with no wait_all() in between.
TEST_F(CloneTest, ErrorLeavesNoOrphanOpThenRecovers)
{
    const char* targets[] = { "/World/envs/env1" };
    // No ovstage is attached yet.
    ovphysx_enqueue_result_t errored = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(errored.status, OVPHYSX_API_ERROR);
    EXPECT_EQ(errored.op_index, 0u) << "synchronous error must not register an async op";

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    ovphysx_enqueue_result_t ok = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(ok.status, OVPHYSX_API_SUCCESS) << "clone after a prior synchronous error must succeed";
    ASSERT_TRUE(waitForOperationSuccess(m_handle, ok.op_index, 5'000'000'000ULL));
    std::vector<std::string> tv(targets, targets + 1);
    ASSERT_TRUE(verify_clones_exist(m_handle, tv));
}

// A clone that fails inside replicate() (here: a non-existent source, which registers the replicator
// then returns false) must still unregister it via the seam's scope guard -- the guard also covers
// the replicate-throws path. Otherwise a later re-attach takes the hijacked replicator path
// (loadPhysics=false) and parses no physics. reset -> reload -> clone must succeed afterwards.
TEST_F(CloneTest, FailedCloneUnregistersReplicatorForReattach)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* badTargets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t failed = enqueue_clone(m_handle, "/World/envs/does_not_exist", badTargets, 1);
    EXPECT_EQ(failed.status, OVPHYSX_API_ERROR);

    // reset + reload; a stale registration from the failed clone would hijack the re-attach.
    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForOperationSuccess(m_handle, reset.op_index, 3'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    ovphysx_usd_handle_t usd_handle2 = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle2)) << "reload after reset failed";

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t ok = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(ok.status, OVPHYSX_API_SUCCESS) << "clone after a failed clone + reset + reload must succeed";
    ASSERT_TRUE(waitForOperationSuccess(m_handle, ok.op_index, 5'000'000'000ULL));
    std::vector<std::string> verify_paths = { "/World/envs/env0", "/World/envs/env1" };
    ASSERT_TRUE(verify_clones_exist(m_handle, verify_paths))
        << "source/clone unaddressable -- a stale registration from the failed clone hijacked re-attach";
}
