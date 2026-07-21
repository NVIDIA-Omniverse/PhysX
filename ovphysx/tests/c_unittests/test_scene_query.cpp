// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Direct C API coverage for ovphysx_raycast / ovphysx_sweep / ovphysx_overlap.
// Python tests exercise the ctypes mirror; this file validates the C ABI.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

using namespace test_utils;
using test_utils::make_ovx_string;
using test_utils::make_ovx_string_bytes;

namespace {

bool wait_op_success(ovphysx_handle_t handle, ovphysx_op_index_t op_index,
                     uint64_t timeout_ns = 10'000'000'000ULL)
{
    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_op(handle, op_index, timeout_ns, &wait_result);
    ovphysx_destroy_wait_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS;
}

bool load_usd_and_wait(ovphysx_handle_t handle, const char* usd_path)
{
    return attach_usd_with_ovstage(handle, usd_path);
}

bool step_and_wait(ovphysx_handle_t handle, float dt)
{
    ovphysx_enqueue_result_t res = ovphysx_step(handle, dt);
    return res.status == OVPHYSX_API_SUCCESS && wait_op_success(handle, res.op_index);
}

ovphysx_scene_query_geometry_desc_t make_shape_geometry(ovphysx_string_t prim_path)
{
    ovphysx_scene_query_geometry_desc_t geom{};
    geom.type = OVPHYSX_SCENE_QUERY_GEOMETRY_SHAPE;
    geom.shape.prim_path = prim_path;
    return geom;
}

ovphysx_scene_query_geometry_desc_t make_sphere_geometry(float radius, float x, float y, float z)
{
    ovphysx_scene_query_geometry_desc_t geom{};
    geom.type = OVPHYSX_SCENE_QUERY_GEOMETRY_SPHERE;
    geom.sphere.radius = radius;
    geom.sphere.position[0] = x;
    geom.sphere.position[1] = y;
    geom.sphere.position[2] = z;
    return geom;
}

} // namespace

// ============================================================================
// Happy-path and argument-validation tests -- rigid-body scene (Z-up)
//
// Scene-query APIs validate that a stage is attached before argument checks,
// so embedded-NUL and null-geometry tests must run with a loaded scene.
// ============================================================================

class SceneQueryRigidBodyTest : public PhysXTestFixture {
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
            << "Failed to load rigid-body scene";
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(SceneQueryRigidBodyTest, RaycastNullOrigin)
{
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_raycast(
        m_handle, nullptr, direction, 10.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(SceneQueryRigidBodyTest, SweepNullGeometry)
{
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_sweep(
        m_handle, nullptr, direction, 10.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(SceneQueryRigidBodyTest, OverlapNullGeometry)
{
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_overlap(
        m_handle, nullptr, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(SceneQueryRigidBodyTest, SweepShapeNullPrimPath)
{
    ovphysx_scene_query_geometry_desc_t geom = make_shape_geometry(make_ovx_string(nullptr));
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_sweep(
        m_handle, &geom, direction, 10.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(SceneQueryRigidBodyTest, SweepShapeEmptyPrimPath)
{
    ovphysx_scene_query_geometry_desc_t geom = make_shape_geometry(make_ovx_string(""));
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_sweep(
        m_handle, &geom, direction, 10.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// NVBugs 6433621: embedded NUL bytes in SHAPE prim_path must be rejected at the C ABI.
TEST_F(SceneQueryRigidBodyTest, SweepShapeEmbeddedNulPrimPath)
{
    std::string storage;
    ovphysx_scene_query_geometry_desc_t geom = make_shape_geometry(
        make_ovx_string_bytes(std::string("/World/Cube1") + '\0' + "GARBAGE", storage));
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_sweep(
        m_handle, &geom, direction, 10.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(hits, nullptr);
    EXPECT_EQ(count, 0u);
}

TEST_F(SceneQueryRigidBodyTest, OverlapShapeEmbeddedNulPrimPath)
{
    std::string storage;
    ovphysx_scene_query_geometry_desc_t geom = make_shape_geometry(
        make_ovx_string_bytes(std::string("/World/Cube1") + '\0' + "GARBAGE", storage));
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_overlap(
        m_handle, &geom, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(hits, nullptr);
    EXPECT_EQ(count, 0u);
}

TEST_F(SceneQueryRigidBodyTest, RaycastClosestHitsGround)
{
    float origin[3] = {0.0f, 0.0f, 100.0f};
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_raycast(
        m_handle, origin, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(count, 1u);
    ASSERT_NE(hits, nullptr);
    EXPECT_GT(hits[0].distance, 0.0f);
}

TEST_F(SceneQueryRigidBodyTest, SweepSphereClosest)
{
    ovphysx_scene_query_geometry_desc_t geom = make_sphere_geometry(0.5f, 0.0f, 0.0f, 100.0f);
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_sweep(
        m_handle, &geom, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(count, 1u);
    ASSERT_NE(hits, nullptr);
    EXPECT_GT(hits[0].distance, 0.0f);
}

TEST_F(SceneQueryRigidBodyTest, OverlapSphereAny)
{
    ovphysx_scene_query_geometry_desc_t geom = make_sphere_geometry(0.5f, 0.0f, 0.0f, 100.0f);
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_overlap(
        m_handle, &geom, OVPHYSX_SCENE_QUERY_MODE_ANY, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_LE(count, 1u);
}

TEST_F(SceneQueryRigidBodyTest, SweepShapeValidPrimPath)
{
    ovphysx_scene_query_geometry_desc_t geom =
        make_shape_geometry(make_ovx_string("/World/Cube1"));
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_sweep(
        m_handle, &geom, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    // Hit count depends on sim state; the contract here is no crash and a valid API status.
}

TEST_F(SceneQueryRigidBodyTest, OverlapShapeValidPrimPath)
{
    ovphysx_scene_query_geometry_desc_t geom =
        make_shape_geometry(make_ovx_string("/World/Cube1"));
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_overlap(
        m_handle, &geom, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
}
