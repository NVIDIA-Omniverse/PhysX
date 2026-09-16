// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Direct C API coverage for ovphysx_raycast / ovphysx_sweep / ovphysx_overlap.
// Python tests exercise the ctypes mirror. This file validates the C ABI.

#include <gtest/gtest.h>
#include <ovstage/ovx_path_dictionary.h>
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

// Tombstones a single prim (all its attributes) in a fixture's already-attached
// ovstage instance at a new ordinal and drains the change through
// ovphysx_update_from_ovstage(). Mirrors OvstageChangeTemplate::deletePrim, the
// ovruntime test harness's structural-removal primitive (ovstage_delete_attributes
// with an empty attribute list). Unlike detach+reattach, this keeps the same live
// attach/source (same ObjectKey generation), so it exercises a structural USD
// edit the way production code does rather than the "whole new source" case.
bool remove_ovstage_prim(ovphysx_handle_t handle, const char* primPath, ovstage_ordinal_t ordinal)
{
    ovstage_instance_t* stage = nullptr;
    {
        std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
        auto it = ovstage_test_attachments().find(handle);
        if (it == ovstage_test_attachments().end() || it->second.empty())
            return false;
        stage = it->second.front().stage;
    }
    if (!stage)
        return false;

    ovx_path_dictionary_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
        return false;

    const ovx_string_t pathStr{primPath, std::strlen(primPath)};
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list_from_strings(dict, &pathStr, 1, &list) != OVX_OK)
        return false;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    const bool queried = ovstage_query_from_path_list(stage, list, &query) == OVSTAGE_OK &&
                         query != OVSTAGE_INVALID_QUERY_HANDLE;
    bool ok = queried;
    if (queried)
    {
        // Empty attribute list = tombstone the entire prim (ovstage_api.h's
        // delete_attributes doc comment), not just one attribute.
        const ovstage_enqueue_result_t del = ovstage_delete_attributes(stage, query, nullptr, 0, ordinal);
        ok = del.status == OVSTAGE_OK;
        if (ok && del.op_index != OVSTAGE_INVALID_OP_ID)
        {
            ovstage_op_wait_result_t delWait{};
            ok = ovstage_wait_op(stage, del.op_index, OVSTAGE_TIMEOUT_INFINITE, &delWait) == OVSTAGE_OK &&
                 delWait.error_op_id_count == 0;
            ovstage_release_op(stage, del.op_index);
        }
        ovstage_release_query(stage, query);
    }
    ovx_path_dictionary_destroy_path_list(dict, list);
    if (!ok)
        return false;

    ovstage_write_floor_desc_t floor_desc{};
    floor_desc.ordinal = ordinal;
    floor_desc.scope = OVSTAGE_SCOPE_ALL;
    const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floor_desc);
    if (floor.status != OVSTAGE_OK)
        return false;
    ovstage_op_wait_result_t floor_wait{};
    if (ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floor_wait) != OVSTAGE_OK ||
        floor_wait.error_op_id_count != 0)
        return false;
    if (ovstage_release_op(stage, floor.op_index) != OVSTAGE_OK)
        return false;

    ovstage_ordinal_range_t range{};
    range.end_ordinal = ordinal;
    range.has_start_ordinal = false;
    return ovphysx_update_from_ovstage(handle, range).status == OVPHYSX_API_SUCCESS;
}

} // namespace

// ============================================================================
// Happy-path and argument-validation tests on the rigid-body scene (Z-up)
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
    // The hit count depends on sim state. The contract here is no crash and a valid API status.
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

// ---------------------------------------------------------------------------
// ovphysx_scene_query_get_paths_from_ids resolves the opaque ObjectKey
// identity fields on a hit back to a physics-object path (OMPE-94459,
// ADR-0019 ObjectKey migration). See the ovphysx_scene_query_hit_t docs.
// ---------------------------------------------------------------------------

// Round trip: raycast onto the ground plane, then resolve the hit's
// collision/rigid_body identity fields back to their prim paths.
TEST_F(SceneQueryRigidBodyTest, RaycastHitResolvesToGroundPlanePath)
{
    // y=10 clears both /World/BigBase (a large static collision mesh over the
    // scene's origin, world y in [-11.27, 0.91]) and the falling Cube1..11
    // (world y in [-1.5, -0.5]), while staying within the 50x50 GroundPlane
    // mesh (x, y in [-25, 25]), so this ray hits only the ground plane.
    float origin[3] = {0.0f, 10.0f, 100.0f};
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_raycast(
        m_handle, origin, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(count, 1u);
    ASSERT_NE(hits, nullptr);

    const uint64_t ids[2] = {hits[0].collision, hits[0].rigid_body};
    ovphysx_string_t paths[2]{};
    uint32_t resolved = 0;
    r = ovphysx_scene_query_get_paths_from_ids(m_handle, ids, 2, paths, 2, &resolved);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << ovphysx_get_last_error().ptr;
    EXPECT_EQ(resolved, 2u);

    ASSERT_NE(paths[0].ptr, nullptr);
    std::string collisionPath(paths[0].ptr, paths[0].length);
    EXPECT_NE(collisionPath.find("/World/GroundPlane"), std::string::npos)
        << "collision path was: " << collisionPath;

    ASSERT_NE(paths[1].ptr, nullptr);
    std::string rigidBodyPath(paths[1].ptr, paths[1].length);
    EXPECT_NE(rigidBodyPath.find("/World/GroundPlane"), std::string::npos)
        << "rigid_body path was: " << rigidBodyPath;
}

// An id that never named a real object (the invalid sentinel, 0) resolves to
// an empty path rather than an error.
TEST_F(SceneQueryRigidBodyTest, ResolveUnknownIdYieldsEmptyPath)
{
    const uint64_t ids[1] = {0};
    ovphysx_string_t paths[1]{};
    uint32_t resolved = 0;
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(m_handle, ids, 1, paths, 1, &resolved);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(resolved, 1u);
    EXPECT_EQ(paths[0].length, 0u);
}

TEST_F(SceneQueryRigidBodyTest, ResolvePathsFromIdsNullIdsWithNonzeroCount)
{
    ovphysx_string_t paths[1]{};
    uint32_t resolved = 0;
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(m_handle, nullptr, 1, paths, 1, &resolved);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// out_count == nullptr must fail before writing anything, including out_paths.
// The test seeds out_paths with sentinels to prove the no-touch guarantee
// rather than only checking the return status (MR !8217).
TEST_F(SceneQueryRigidBodyTest, ResolvePathsFromIdsNullOutCount)
{
    const uint64_t ids[1] = {0};
    const ovphysx_string_t sentinel{reinterpret_cast<const char*>(0x1), 0xDEADBEEFu};
    ovphysx_string_t paths[1] = {sentinel};
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(m_handle, ids, 1, paths, 1, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(paths[0].ptr, sentinel.ptr) << "out_paths must not be touched when out_count is NULL";
    EXPECT_EQ(paths[0].length, sentinel.length) << "out_paths must not be touched when out_count is NULL";
}

// out_paths == NULL with a nonzero capacity is a distinct invalid-argument
// case from a NULL out_count. Both must be individually validated.
TEST_F(SceneQueryRigidBodyTest, ResolvePathsFromIdsNullOutPathsWithNonzeroMax)
{
    const uint64_t ids[1] = {0};
    uint32_t resolved = 0;
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(m_handle, ids, 1, nullptr, 1, &resolved);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// A garbage handle that ovphysx_create_instance() never returned must fail
// cleanly, not crash.
TEST(SceneQueryHandleValidation, ResolvePathsFromIdsUnknownInstanceHandle)
{
    const uint64_t ids[1] = {0};
    ovphysx_string_t paths[1]{};
    uint32_t resolved = 0;
    const ovphysx_handle_t bogusHandle = static_cast<ovphysx_handle_t>(0xDEADBEEFULL);
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(bogusHandle, ids, 1, paths, 1, &resolved);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(resolved, 0u);
}

// No stage attached: objectKeyToPath's no-attachedStage branch returns "" per
// id, so the call itself still succeeds. An unresolved id is not an error, as
// the function's doc comment states.
TEST_F(PhysXTestFixture, ResolvePathsFromIdsNoActiveAttach)
{
    const uint64_t ids[1] = {12345};
    ovphysx_string_t paths[1]{};
    uint32_t resolved = 0;
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(m_handle, ids, 1, paths, 1, &resolved);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(resolved, 1u);
    EXPECT_EQ(paths[0].length, 0u);
}

// max_paths smaller than id_count: out_count still reports the full total,
// only the buffer's capacity worth of entries are written.
TEST_F(SceneQueryRigidBodyTest, ResolvePathsFromIdsTruncatesToMaxPaths)
{
    const uint64_t ids[2] = {0, 0};
    ovphysx_string_t paths[1]{};
    uint32_t resolved = 0;
    ovphysx_result_t r = ovphysx_scene_query_get_paths_from_ids(m_handle, ids, 2, paths, 1, &resolved);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(resolved, 2u) << "out_count reports total needed, not entries written";
}

// ---------------------------------------------------------------------------
// objectKeyToPath()'s liveness gate and its documented storage-lifetime
// contract (MR !8217). See PhysX.cpp's objectKeyToPath and IPhysx.h's doc
// comment on the same function.
// ---------------------------------------------------------------------------

// A key resolved once must stop resolving once its object is removed from
// the still-live attach (no detach/reattach). A generation check alone cannot
// catch this case, since the source instance is unchanged.
TEST_F(SceneQueryRigidBodyTest, ResolveRemovedObjectYieldsEmptyPath)
{
    float origin[3] = {0.0f, 10.0f, 100.0f};
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_raycast(
        m_handle, origin, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(count, 1u);
    ASSERT_NE(hits, nullptr);
    const uint64_t groundPlaneId = hits[0].collision;

    ovphysx_string_t before[1]{};
    uint32_t resolvedBefore = 0;
    r = ovphysx_scene_query_get_paths_from_ids(m_handle, &groundPlaneId, 1, before, 1, &resolvedBefore);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(before[0].ptr, nullptr);
    ASSERT_GT(before[0].length, 0u) << "baseline resolve must be non-empty before removal";
    const std::string groundPlanePath(before[0].ptr, before[0].length);

    ASSERT_TRUE(remove_ovstage_prim(m_handle, groundPlanePath.c_str(), /*ordinal=*/2))
        << "failed to tombstone " << groundPlanePath << " in the live ovstage attach";
    // Removal is processed at step time, not synchronously by
    // ovphysx_update_from_ovstage (mirrors TestOvstageChange.cpp's "RigidBody
    // change parity - remove prim": simulate()+fetchResults() runs before the
    // removed object's PhysX pointer is asserted gone).
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));

    ovphysx_string_t after[1]{};
    uint32_t resolvedAfter = 0;
    r = ovphysx_scene_query_get_paths_from_ids(m_handle, &groundPlaneId, 1, after, 1, &resolvedAfter);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(resolvedAfter, 1u);
    EXPECT_EQ(after[0].length, 0u)
        << "a key for an object removed from the still-live attach must resolve to empty, not a stale path";
}

// The pointer-lifetime AC: objectKeyToPath()'s returned pointer is documented
// valid only until the next detach/re-attach (IPhysx.h). Dereferencing a
// pointer past that point would be UB, so the test checks the documented,
// observable contract instead. A fresh resolve after a fresh attach gives a
// correct answer, and the id from the torn-down attach no longer resolves
// under the new one.
TEST_F(SceneQueryRigidBodyTest, ResolveAfterDetachReattachGivesFreshAnswer)
{
    float origin[3] = {0.0f, 10.0f, 100.0f};
    float direction[3] = {0.0f, 0.0f, -1.0f};
    const ovphysx_scene_query_hit_t* hits = nullptr;
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_raycast(
        m_handle, origin, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(count, 1u);
    const uint64_t staleId = hits[0].collision;

    ovphysx_string_t before[1]{};
    uint32_t resolvedBefore = 0;
    r = ovphysx_scene_query_get_paths_from_ids(m_handle, &staleId, 1, before, 1, &resolvedBefore);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(before[0].ptr, nullptr);
    ASSERT_GT(before[0].length, 0u);
    const std::string beforePath(before[0].ptr, before[0].length);

    // Detach and re-attach the same scene from scratch, giving a fresh source
    // and a fresh generation. TestObjectKeyMinting.cpp pins the same guarantee
    // at the ovruntime level.
    ASSERT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));

    hits = nullptr;
    count = 0;
    r = ovphysx_raycast(
        m_handle, origin, direction, 200.0f, false,
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(count, 1u);
    const uint64_t freshId = hits[0].collision;

    ovphysx_string_t after[1]{};
    uint32_t resolvedAfter = 0;
    r = ovphysx_scene_query_get_paths_from_ids(m_handle, &freshId, 1, after, 1, &resolvedAfter);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(after[0].ptr, nullptr);
    ASSERT_GT(after[0].length, 0u);
    EXPECT_EQ(std::string(after[0].ptr, after[0].length), beforePath)
        << "the fresh attach's own resolve for the same physics object gives the same path";

    ovphysx_string_t staleAfter[1]{};
    uint32_t resolvedStale = 0;
    r = ovphysx_scene_query_get_paths_from_ids(m_handle, &staleId, 1, staleAfter, 1, &resolvedStale);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(staleAfter[0].length, 0u)
        << "an id from the torn-down attach must not resolve under the new one";
}
