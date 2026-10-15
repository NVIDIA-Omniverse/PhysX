// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// C-API tests for ovphysx_clone (replicator-backed). Clones are physics-only: live PhysX bodies
// addressable by their target path, not authored as USD prims. A clone is therefore verified by
// reading its body pose through a session read selected by that path. basic_simulation.usda
// places one rigid body 'table' under each env.

#include "global_test_environment.h"  // PhysXTestFixture, waitForTaskSuccess, ovphysx.h
#include "ovphysx/ovphysx_attributes.h"
#include "test_utilities.h"

#include <ovstage/ovx_path_dictionary.h>

#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using test_utils::make_ovx_string;

// Attach an ovstage-backed USD scene and drain it (matches the other c_unittests' helper).
static bool load_usd_and_wait(ovphysx_instance_handle_t handle, const char* usd_path, ovphysx_usd_handle_t& out_handle)
{
    (void)out_handle;
    return test_utils::attach_usd_with_ovstage(handle, usd_path);
}

static ovphysx_enqueue_result_t enqueue_clone(ovphysx_instance_handle_t handle,
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

// The rigid bodies a selected session read covers: each body's prim path and world position, in row order.
struct SelectedBodies
{
    std::vector<std::string> paths;
    std::vector<std::array<float, 3>> positions;
};

// Reads the rigid bodies `selection` names. The shared instance runs on CPU, so every column is host memory.
static bool read_selected_bodies(ovphysx_instance_handle_t handle, const ovphysx_query_selection_t& selection,
                                 SelectedBodies& out)
{
    out = {};
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t opened =
        ovphysx_query_selection(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &selection, &query, nullptr);
    if (test_utils::complete_task(handle, opened).status != OVPHYSX_API_SUCCESS)
        return false;
    path_dictionary_instance_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    bool ok = dictionary != nullptr;
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictionary);
    const ovx_string_or_token_t position = { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } };
    ovphysx_read_handle_t read = 0;
    ok = ok && test_utils::complete_task(handle, ovphysx_read(handle, query, &position, 1, &read, nullptr)).status ==
                   OVPHYSX_API_SUCCESS;
    while (ok)
    {
        const ovstage_read_group_t* group = nullptr;
        ok = test_utils::complete_task(handle, ovphysx_fetch_read_next(handle, read, &group)).status ==
             OVPHYSX_API_SUCCESS;
        if (!ok || !group)
            break;
        const DLTensor* tensor = group->data.tensor_count ? &group->data.tensors[0] : nullptr;
        const ovx_primpath_t* paths = nullptr;
        size_t path_count = 0;
        ok = !group->is_array && tensor && tensor->data && tensor->device.device_type == kDLCPU &&
             tensor->dtype.lanes == 3 &&
             ovx_path_dictionary_get_paths(dict, group->prims.list, &paths, &path_count) == OVX_OK;
        const float* values =
            ok ? reinterpret_cast<const float*>(static_cast<const char*>(tensor->data) + tensor->byte_offset) : nullptr;
        const int64_t stride = (ok && tensor->strides) ? tensor->strides[0] : 1;
        for (uint32_t row = 0; ok && row < group->prims.count; ++row)
        {
            const uint32_t index = group->prims.index_map ? group->prims.index_map[row] : group->prims.offset + row;
            ovx_string_t path{};
            ok = index < path_count && ovx_path_dictionary_path_to_string(dict, paths[index], &path) == OVX_OK;
            if (!ok)
                break;
            const float* xyz = values + row * stride * 3;
            out.paths.emplace_back(path.ptr, path.length);
            out.positions.push_back({ xyz[0], xyz[1], xyz[2] });
        }
        ok = test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)).status ==
                 OVPHYSX_API_SUCCESS &&
             ok;
    }
    if (read != 0)
        ok = test_utils::complete_task(handle, ovphysx_release_read(handle, read)).status == OVPHYSX_API_SUCCESS && ok;
    ok = test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status == OVPHYSX_API_SUCCESS && ok;
    return ok;
}

static bool read_pattern(ovphysx_instance_handle_t handle, const char* pattern, SelectedBodies& out)
{
    ovphysx_query_selection_t selection{};
    selection.pattern = make_ovx_string(pattern);
    return read_selected_bodies(handle, selection, out);
}

// An explicit selection takes native prim-path handles, interned here into the attached Stage's dictionary.
static bool read_paths(ovphysx_instance_handle_t handle, const std::vector<std::string>& paths, SelectedBodies& out)
{
    ovphysx_query_handle_t query = 0;
    if (test_utils::complete_task(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr))
            .status != OVPHYSX_API_SUCCESS)
        return false;
    path_dictionary_instance_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    bool ok = dictionary != nullptr;
    std::vector<ovx_primpath_t> handles;
    for (const std::string& path : paths)
    {
        ovx_primpath_t interned = OVX_INVALID_PRIMPATH;
        const ovx_string_t text = { path.c_str(), path.size() };
        ok = ok &&
             ovx_path_dictionary_intern_path(static_cast<ovx_path_dictionary_t*>(dictionary), text, &interned) ==
                 OVX_OK &&
             interned != OVX_INVALID_PRIMPATH;
        handles.push_back(interned);
    }
    ok = test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status == OVPHYSX_API_SUCCESS && ok;
    if (!ok)
        return false;

    ovphysx_query_selection_t selection{};
    selection.prim_paths = handles.data();
    selection.prim_paths_count = static_cast<uint32_t>(handles.size());
    return read_selected_bodies(handle, selection, out);
}

// Reads one rigid body's world position; fails unless exactly that body resolves.
static bool read_position(ovphysx_instance_handle_t handle, const char* prim_path, double* out_pos)
{
    SelectedBodies bodies;
    if (!read_pattern(handle, prim_path, bodies) || bodies.positions.size() != 1)
        return false;
    for (int axis = 0; axis < 3; ++axis)
        out_pos[axis] = static_cast<double>(bodies.positions[0][axis]);
    return true;
}

// A clone "exists" when its replicated rigid body is addressable: a read selected by the clone body
// path resolves it and reads a pose.
static bool verify_clones_exist(ovphysx_instance_handle_t handle, const std::vector<std::string>& env_paths)
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

static void step_and_wait(ovphysx_instance_handle_t handle, float dt)
{
    ovphysx_enqueue_result_t step_res = ovphysx_step(handle, dt);
    ASSERT_EQ(step_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(handle, step_res));
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
        EXPECT_FALSE(waitForTaskSuccess(m_handle, nullSource, 2'000'000'000ULL));
    }

    ovphysx_enqueue_result_t nullTargets =
        ovphysx_clone(m_handle, make_ovx_string("/World/table"), nullptr, 1, nullptr, nullptr);
    EXPECT_EQ(nullTargets.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (nullTargets.op_index != 0)
    {
        EXPECT_FALSE(waitForTaskSuccess(m_handle, nullTargets, 2'000'000'000ULL));
    }

    ovphysx_enqueue_result_t zeroTargets =
        ovphysx_clone(m_handle, make_ovx_string("/World/table"), nullptr, 0, nullptr, nullptr);
    EXPECT_EQ(zeroTargets.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (zeroTargets.op_index != 0)
    {
        EXPECT_FALSE(waitForTaskSuccess(m_handle, zeroTargets, 2'000'000'000ULL));
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
        EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL));
    }
}

TEST_F(CloneTest, ErrorSourceDoesNotExist)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle));

    const char* targets[] = { "/World/envs/envX" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/nonexistent", targets, 1);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(clone_res.instance, m_handle);
    EXPECT_FALSE(waitForTaskSuccess(m_handle, clone_res));
}

TEST_F(CloneTest, CloneSingleTarget)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "ovstage attach/update failed";

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res));

    std::vector<std::string> target_vec(targets, targets + 1);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "Clone body not addressable at target path";

    // No duplicate actors: env0 parsed at attach, env1 created by replicate. Overlapping duplicates
    // would explode into NaNs. A stable 10-step run confirms exactly one body per env.
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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res));

    std::vector<std::string> target_vec(targets, targets + num_targets);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "One or more clone bodies not addressable";

    for (int i = 0; i < 10; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, CloneWithAnchorTransforms)
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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 5'000'000'000ULL))
        << "Clone with transforms timed out or failed";

    step_and_wait(m_handle, 1.0f / 60.0f);

    double env0_pos[3], env1_pos[3], env2_pos[3];
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env0/table", env0_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env1/table", env1_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env2/table", env2_pos));

    const double tol = 0.5;
    EXPECT_NEAR(env1_pos[0] - env0_pos[0], 5.0, tol) << "env1 X offset does not match anchor transform";
    EXPECT_NEAR(env2_pos[0] - env0_pos[0], 10.0, tol) << "env2 X offset does not match anchor transform";
    EXPECT_NEAR(env1_pos[2] - env0_pos[2], 0.0, tol) << "env1 Z offset should be ~0";
    EXPECT_NEAR(env2_pos[2] - env0_pos[2], 0.0, tol) << "env2 Z offset should be ~0";
}

TEST_F(CloneTest, CloneWithAnchorRotation)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1" };

    // 90-degree yaw about Y, quat (0, sin45, 0, cos45), placed at (20,0,0).
    const float s = 0.7071068f;
    float transforms[] = { 20.0f, 0.0f, 0.0f, 0.0f, s, 0.0f, s };

    std::vector<ovphysx_string_t> target_ovx;
    target_ovx.push_back(make_ovx_string(targets[0]));
    ovphysx_enqueue_result_t clone_res =
        ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), 1, transforms, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 5'000'000'000ULL))
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

TEST_F(CloneTest, CloneAfterWarmupRejectedInAllModes) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    ovphysx_result_t warmup_res = ovphysx_warmup(m_handle);
    ASSERT_EQ(warmup_res.status, OVPHYSX_API_SUCCESS) << "warmup failed";

    const char* targets[] = { "/World/envs/env1" };
    const uint32_t num_targets = 1;
    std::vector<ovphysx_string_t> target_ovx;
    target_ovx.push_back(make_ovx_string(targets[0]));

    ovphysx_enqueue_result_t clone_res = ovphysx_clone(
        m_handle,
        make_ovx_string("/World/envs/env0"),
        target_ovx.data(),
        num_targets, nullptr, nullptr);

    EXPECT_EQ(clone_res.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "clone() after warmup must be rejected in all modes";
}

// ============================================================================
// Regression: xformOp:scale is preserved during clone with anchor_transforms.
//
// When the source root has xformOp:scale in its xformOpOrder, batchClone
// copies the attribute to every target. restoreTargetTransforms must include
// scale in the target's xformOpOrder, otherwise updateWorldXforms ignores it
// and children end up at the wrong world-space size.
// ============================================================================

TEST_F(CloneTest, CloneWithSourceScale) {
    // In this scene env0 has translate + orient + scale (0.5x).
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/clone_with_root_transforms.usda", usd_handle))
        << "ovstage attach/update failed";

    // Clone env0 -> env1 with explicit transform at (5, 0, 0) + identity quat
    const char* targets[] = { "/World/envs/env1" };
    std::vector<ovphysx_string_t> target_ovx;
    target_ovx.push_back(make_ovx_string(targets[0]));

    float transforms[] = {
        5.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 1.0f
    };

    ovphysx_enqueue_result_t clone_res = ovphysx_clone(
        m_handle,
        make_ovx_string("/World/envs/env0"),
        target_ovx.data(),
        1,
        transforms, nullptr);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 5'000'000'000ULL))
        << "Clone with source scale timed out or failed";

    // Warmup initializes the tensor data.
    ovphysx_result_t warmup_res = ovphysx_warmup(m_handle);
    ASSERT_EQ(warmup_res.status, OVPHYSX_API_SUCCESS) << "warmup failed";

    double env0_box_pos[3], env1_box_pos[3];
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env0/box", env0_box_pos));
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env1/box", env1_box_pos));

    std::cout << "env0/box pos: (" << env0_box_pos[0] << ", " << env0_box_pos[1]
              << ", " << env0_box_pos[2] << ")" << std::endl;
    std::cout << "env1/box pos: (" << env1_box_pos[0] << ", " << env1_box_pos[1]
              << ", " << env1_box_pos[2] << ")" << std::endl;

    // env0 has scale 0.5 and a -90-deg rotation around Z (quat w=0.707, z=-0.707),
    // so child box at local (0,2,0) -> scaled to (0,1,0) -> rotated -90 deg
    // around Z to (1,0,0) -> translated by (0,0,1) = world (1, 0, 1).
    //
    // env1 has scale 0.5 with identity rotation, so child box at
    // local (0,2,0) -> scaled to (0,1,0) -> no rotation -> translated
    // by (5,0,0) = world (5, 1, 0).
    //
    // If scale is dropped, env1's child Y would be 2.0 instead of 1.0.

    const double tol = 0.1;

    // env0's box position confirms the scene setup.
    EXPECT_NEAR(env0_box_pos[0], 1.0, tol) << "env0 box X should be ~1.0";
    EXPECT_NEAR(env0_box_pos[1], 0.0, tol) << "env0 box Y should be ~0.0";
    EXPECT_NEAR(env0_box_pos[2], 1.0, tol) << "env0 box Z should be ~1.0";

    // env1 parent is at (5, 0, 0) with identity orient.
    // Child local Y=2, scaled by 0.5 -> world Y offset of 1.0 from parent.
    double env1_child_y_offset = env1_box_pos[1] - 0.0;  // parent Y = 0
    EXPECT_NEAR(env1_child_y_offset, 1.0, tol)
        << "env1 child Y offset should be ~1.0 (scale 0.5 * local Y 2.0). "
        << "A value of ~2.0 means xformOp:scale was dropped from xformOpOrder.";

    // Simulation must run without crashing.
    for (int i = 0; i < 10; ++i) {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }

    std::cout << "[OK] Clone with source scale preserved correctly" << std::endl;
}

// ============================================================================
// Non-clone regression tests: deferred attachStage() must work for users who
// never call clone(). These exercise the lazy attach path through
// simulate / warmup / tensor read.
// ============================================================================

// The first step() triggers the lazy attachStage() without any clone() call.
TEST_F(CloneTest, StepWithoutClone)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    step_and_wait(m_handle, 1.0f / 60.0f);
    step_and_wait(m_handle, 1.0f / 60.0f);
}

TEST_F(CloneTest, WarmupWithoutClone)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    ovphysx_result_t res = ovphysx_warmup(m_handle);
    ASSERT_EQ(res.status, OVPHYSX_API_SUCCESS) << "warmup failed without clone";

    step_and_wait(m_handle, 1.0f / 60.0f);
}

TEST_F(CloneTest, ReadWithoutClone)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    double pos[3] = {};
    ASSERT_TRUE(read_position(m_handle, "/World/envs/env0/table", pos)) << "Read without clone failed";
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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 2'000'000'000ULL)) << "Clone timed out or failed";

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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 5'000'000'000ULL))
        << "Clone timed out or failed (50 clones)";

    for (int i = 0; i < 60; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

TEST_F(CloneTest, RegressionLargeBatchRetainsTargetPaths)
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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 20'000'000'000ULL))
        << "Large clone batch timed out or failed";

    SelectedBodies wildcard;
    ASSERT_TRUE(read_pattern(m_handle, "/World/envs/env*/table", wildcard)) << "Wildcard read failed";
    std::sort(wildcard.paths.begin(), wildcard.paths.end());
    EXPECT_EQ(wildcard.paths, expected_body_paths) << "Wildcard read returned wrong or duplicate clone paths";

    SelectedBodies explicit_bodies;
    ASSERT_TRUE(read_paths(m_handle, expected_body_paths, explicit_bodies)) << "Explicit-path read failed";
    std::sort(explicit_bodies.paths.begin(), explicit_bodies.paths.end());
    EXPECT_EQ(explicit_bodies.paths, expected_body_paths)
        << "Explicit-path read returned wrong or duplicate clone paths";
}

static ovphysx_enqueue_result_t enqueue_clone_count(ovphysx_instance_handle_t handle, uint32_t num_targets)
{
    // Spread the clones over an XZ grid rather than passing null anchors, which co-locates every
    // copy on the source.
    const uint32_t gridWidth = static_cast<uint32_t>(std::ceil(std::sqrt(double(num_targets) + 1.0)));
    const float kSpacing = 5.0f;

    std::vector<std::string> target_paths;
    std::vector<ovphysx_string_t> target_ovx;
    std::vector<float> anchors; // [x,y,z, qx,qy,qz,qw] per target
    target_paths.reserve(num_targets);
    target_ovx.reserve(num_targets);
    anchors.reserve(static_cast<size_t>(num_targets) * 7u);
    for (uint32_t i = 1; i <= num_targets; ++i)
    {
        target_paths.push_back("/World/envs/env" + std::to_string(i));
        target_ovx.push_back(make_ovx_string(target_paths.back().c_str()));

        anchors.push_back(float(i % gridWidth) * kSpacing); // x
        anchors.push_back(0.0f); // y
        anchors.push_back(float(i / gridWidth) * kSpacing); // z
        anchors.push_back(0.0f); // qx
        anchors.push_back(0.0f); // qy
        anchors.push_back(0.0f); // qz
        anchors.push_back(1.0f); // qw (identity)
    }
    return ovphysx_clone(handle, make_ovx_string("/World/envs/env0"), target_ovx.data(), num_targets,
                         anchors.data(), nullptr);
}

// NVBugs 6473884: the filed repro cycles detach_ovstage() -> reload -> clone again at large N.
// RegressionLargeBatchRetainsTargetPaths covers a single batch.
TEST_F(CloneTest, RegressionLargeBatchCloneAfterReset_NVBugs6473884)
{
    constexpr uint32_t kNumTargets = 512;

    ovphysx_usd_handle_t usd_handle_first = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle_first)) << "first cycle: USD load failed";

    ovphysx_enqueue_result_t clone_res_first = enqueue_clone_count(m_handle, kNumTargets);
    ASSERT_EQ(clone_res_first.status, OVPHYSX_API_SUCCESS) << "first cycle: clone enqueue failed";
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res_first, 30'000'000'000ULL))
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

    ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, reset, 5'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    ovphysx_usd_handle_t usd_handle_second = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle_second)) << "second cycle: USD load failed";

    ovphysx_enqueue_result_t clone_res_second = enqueue_clone_count(m_handle, kNumTargets);
    ASSERT_EQ(clone_res_second.status, OVPHYSX_API_SUCCESS) << "second cycle: clone enqueue failed";
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res_second, 30'000'000'000ULL))
        << "second cycle: large clone timed out or failed after reset";

    ASSERT_TRUE(verify_clones_exist(m_handle, spot_check))
        << "second cycle: clone bodies not addressable after reset + large batch";

    SelectedBodies wildcard;
    ASSERT_TRUE(read_pattern(m_handle, "/World/envs/env*/table", wildcard)) << "second cycle: wildcard read failed";
    std::vector<std::string> expected_paths;
    expected_paths.reserve(kNumTargets + 1);
    for (uint32_t i = 0; i <= kNumTargets; ++i)
        expected_paths.push_back("/World/envs/env" + std::to_string(i) + "/table");
    std::sort(expected_paths.begin(), expected_paths.end());
    std::sort(wildcard.paths.begin(), wildcard.paths.end());
    EXPECT_EQ(wildcard.paths, expected_paths)
        << "second cycle: every cloned target path must remain addressable through a wildcard";
}

TEST_F(CloneTest, ValidationNonSequentialTargetPaths)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // Non-standard target paths prove the system honors caller-supplied paths rather than
    // auto-generating env1/env2/...
    const char* targets[] = {
        "/World/envs/custom_clone_A", "/World/envs/custom_clone_B", "/World/envs/test_env_99",
        "/World/different/path/structure", "/World/envs/env_with_underscores_123"
    };
    const uint32_t num_targets = sizeof(targets) / sizeof(targets[0]);

    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, num_targets);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 2'000'000'000ULL))
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
    // warmup / the first step.
    const char* targets1[] = { "/World/envs/batch1_clone_A", "/World/envs/batch1_clone_B" };
    ovphysx_enqueue_result_t clone_res1 = enqueue_clone(m_handle, "/World/envs/env0", targets1, 2);
    ASSERT_EQ(clone_res1.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res1, 2'000'000'000ULL)) << "First clone operation failed";

    const char* targets2[] = { "/World/envs/batch2_clone_X", "/World/envs/batch2_clone_Y", "/World/envs/batch2_clone_Z" };
    ovphysx_enqueue_result_t clone_res2 = enqueue_clone(m_handle, "/World/envs/env0", targets2, 3);
    ASSERT_EQ(clone_res2.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res2, 2'000'000'000ULL)) << "Second clone operation failed";

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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 5'000'000'000ULL)) << "Clone timed out or failed";

    ovphysx_result_t warmup_res = ovphysx_warmup(m_handle);
    ASSERT_EQ(warmup_res.status, OVPHYSX_API_SUCCESS) << "warmup failed";

    // A read over every env's body must resolve exactly TOTAL_ENVS actors. A count of
    // TOTAL_ENVS+1 would mean env0 was parsed twice (the duplicate-actor bug).
    SelectedBodies bodies;
    ASSERT_TRUE(read_pattern(m_handle, "/World/envs/env*/table", bodies)) << "Read failed";
    EXPECT_EQ(bodies.positions.size(), static_cast<size_t>(TOTAL_ENVS))
        << "Expected exactly " << TOTAL_ENVS << " rigid bodies (source + " << NUM_CLONES << " clones), got "
        << bodies.positions.size();
    for (size_t i = 0; i < bodies.positions.size(); ++i)
    {
        for (float coordinate : bodies.positions[i])
            ASSERT_FALSE(std::isnan(coordinate)) << "NaN in the position of " << bodies.paths[i];
    }

    for (int i = 0; i < 30; ++i)
    {
        step_and_wait(m_handle, 1.0f / 60.0f);
    }
}

// A successful clone registers a replicator for the backing stage id. The seam must unregister it so
// a later detach_ovstage() and reload is a normal parse, not the replicator re-attach path (which sets
// loadPhysics=false and parses no physics). Reload and a fresh clone must both work.
TEST_F(CloneTest, RegressionReplicatorUnregisteredForReattachAfterReset)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1", "/World/envs/env2" };
    ovphysx_enqueue_result_t clone_res = enqueue_clone(m_handle, "/World/envs/env0", targets, 2);
    ASSERT_EQ(clone_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res, 5'000'000'000ULL)) << "Clone timed out or failed";
    std::vector<std::string> target_vec(targets, targets + 2);
    ASSERT_TRUE(verify_clones_exist(m_handle, target_vec)) << "Clone bodies not addressable";

    // detach_ovstage() detaches. The clone's replicator registration must not survive it.
    ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, reset, 3'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    // Reload the same scene and clone again, both before any step/tensor read, which auto-warms
    // and would trip the clone-before-warmup guard. If the stale registration had survived, the
    // re-attach of the reused backing stage id would take the replicator path (loadPhysics=false)
    // and parse no physics, so the reloaded source would have no body to clone.
    ovphysx_usd_handle_t usd_handle2 = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle2)) << "reload after reset failed";

    ovphysx_enqueue_result_t clone_res2 = enqueue_clone(m_handle, "/World/envs/env0", targets, 2);
    ASSERT_EQ(clone_res2.status, OVPHYSX_API_SUCCESS) << "second clone (after reset+reload) failed";
    ASSERT_TRUE(waitForTaskSuccess(m_handle, clone_res2, 5'000'000'000ULL)) << "second clone timed out";

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
        EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL));
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
        EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL));
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
        EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL));
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
        ASSERT_EQ(res.status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL))
            << "target '" << bad << "' must fail during inline execution";
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

    // Just over the boundary and the two wide sentinel values must all be rejected.
    for (uint32_t bad : { 0x00FFFFFFu, 0x01000000u, 0xFFFFFFFEu, 0xFFFFFFFFu })
    {
        ovphysx_string_t target = make_ovx_string("/World/envs/env1");
        ovphysx_enqueue_result_t res =
            ovphysx_clone(m_handle, make_ovx_string("/World/envs/env0"), &target, 1, nullptr, &bad);
        EXPECT_EQ(res.status, OVPHYSX_API_INVALID_ARGUMENT) << "env_id " << bad << " must be rejected";
        if (res.op_index != 0)
        {
            EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL));
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
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ok, 5'000'000'000ULL));
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
    ASSERT_TRUE(waitForTaskSuccess(m_handle, res, 5'000'000'000ULL))
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

    ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, reset, 3'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    ovphysx_usd_handle_t reloaded_handle = 0;
    ASSERT_TRUE(load_basic_usd(reloaded_handle)) << "reload after rejected clone failed";
    const char* valid_targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t valid = enqueue_clone(m_handle, "/World/envs/env0", valid_targets, 1);
    ASSERT_EQ(valid.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, valid, 5'000'000'000ULL));
}

TEST_F(CloneTest, ErrorTargetReusedAcrossBatches)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t first = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, first, 5'000'000'000ULL)) << "first clone failed";

    // env1 now exists, so reusing it on the same attach must be rejected.
    ovphysx_enqueue_result_t second = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(second.status, OVPHYSX_API_INVALID_ARGUMENT);
    if (second.op_index != 0)
    {
        EXPECT_FALSE(waitForTaskSuccess(m_handle, second, 2'000'000'000ULL));
    }
}

// A target already populated with physics from the initial parse (not from a prior clone) must be
// rejected before replicate(), otherwise it would add duplicate actors under an existing path.
// Distinct from ErrorTargetReusedAcrossBatches (prior-clone targets, caught C-first by
// cloned_target_paths as INVALID_ARGUMENT): here the runtime ObjectDb subtree check in the seam
// catches it during accepted inline execution, before replicate() can mutate anything.
TEST_F(CloneTest, ErrorTargetAlreadyPopulated)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    // /World/envs/env0/table is a rigid body from the parse, so cloning onto it must be rejected.
    const char* targets[] = { "/World/envs/env0/table" };
    ovphysx_enqueue_result_t res = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(res.status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(waitForTaskSuccess(m_handle, res, 2'000'000'000ULL));
}

// Rejected clone admission has no receipt. A clone-before-attach rejection is
// followed by a normal attach and clone without an intervening wait-all.
TEST_F(CloneTest, ErrorLeavesNoOrphanOpThenRecovers)
{
    const char* targets[] = { "/World/envs/env1" };
    // No ovstage is attached yet.
    ovphysx_enqueue_result_t errored = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    EXPECT_EQ(errored.status, OVPHYSX_API_ERROR);
    EXPECT_EQ(errored.op_index, 0u) << "rejected admission must not issue a task";

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    ovphysx_enqueue_result_t ok = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(ok.status, OVPHYSX_API_SUCCESS) << "clone after a prior rejection must succeed";
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ok, 5'000'000'000ULL));
    std::vector<std::string> tv(targets, targets + 1);
    ASSERT_TRUE(verify_clones_exist(m_handle, tv));
}

// A clone that fails inside replicate() (here: a non-existent source, which registers the replicator
// then returns false) must still unregister it via the seam's scope guard. The guard also covers
// the replicate-throws path. Otherwise a later re-attach takes the hijacked replicator path
// (loadPhysics=false) and parses no physics. Reset, reload and clone must succeed afterwards.
TEST_F(CloneTest, FailedCloneUnregistersReplicatorForReattach)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle)) << "USD load timed out or failed";

    const char* badTargets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t failed = enqueue_clone(m_handle, "/World/envs/does_not_exist", badTargets, 1);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);

    // Reset and reload before collecting the recoverable clone failure. A stale registration from the failed clone would hijack the re-attach.
    ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, reset, 3'000'000'000ULL));
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);

    ovphysx_usd_handle_t usd_handle2 = 0;
    ASSERT_TRUE(load_basic_usd(usd_handle2)) << "reload after reset failed";

    const char* targets[] = { "/World/envs/env1" };
    ovphysx_enqueue_result_t ok = enqueue_clone(m_handle, "/World/envs/env0", targets, 1);
    ASSERT_EQ(ok.status, OVPHYSX_API_SUCCESS) << "clone after a failed clone + reset + reload must succeed";
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ok, 5'000'000'000ULL));
    std::vector<std::string> verify_paths = { "/World/envs/env0", "/World/envs/env1" };
    ASSERT_TRUE(verify_clones_exist(m_handle, verify_paths))
        << "source/clone unaddressable -- a stale registration from the failed clone hijacked re-attach";
    ovphysx_wait_task_result_t all{};
    EXPECT_EQ(ovphysx_wait_all(m_handle, UINT64_MAX, &all).status, OVPHYSX_API_ERROR);
    ASSERT_EQ(all.num_errors, 1u);
    EXPECT_EQ(all.error_tasks[0].op_index, failed.op_index);
    ovphysx_destroy_wait_task_result(&all);
}
