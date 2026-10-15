// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovstage/ovx_path_dictionary.h>

#include <array>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace test_utils;

namespace
{
void stepN(ovphysx_instance_handle_t handle, int n)
{
    for (int i = 0; i < n; ++i)
        ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
}

ovx_string_or_token_t attrName(const char* name)
{
    ovx_string_or_token_t a{};
    a.string.ptr = name;
    a.string.length = std::strlen(name);
    return a;
}

std::map<std::string, std::array<float, 3>> rigidPositions(ovphysx_instance_handle_t handle)
{
    std::map<std::string, std::array<float, 3>> out;
    ovphysx_query_handle_t query = 0;
    if (test_utils::complete_task(
            handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr))
            .status != OVPHYSX_API_SUCCESS)
        return out;
    ovx_path_dictionary_t* dict = attached_ovstage_dictionary(handle);
    if (!dict)
    {
        test_utils::complete_task(handle, ovphysx_release_query(handle, query));
        return out;
    }
    const ovx_string_or_token_t name = attrName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    if (test_utils::complete_task(handle, ovphysx_read(handle, query, &name, 1, &read, nullptr)).status != OVPHYSX_API_SUCCESS)
    {
        test_utils::complete_task(handle, ovphysx_release_query(handle, query));
        return out;
    }
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t fetched = test_utils::complete_task(handle, ovphysx_fetch_read_next(handle, read, &group));
        if (fetched.status != OVPHYSX_API_SUCCESS || !group)
            break;
        const DLTensor& tensor = group->data.tensors[0];
        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (tensor.data && ovx_path_dictionary_get_paths(dict, group->prims.list, &paths, &pathCount) == OVX_OK)
        {
            const float* values = static_cast<const float*>(tensor.data);
            int64_t width = tensor.dtype.lanes;
            if (width <= 1 && tensor.ndim >= 2 && tensor.shape)
                width = tensor.shape[1];
            if (width < 3 || tensor.device.device_type != kDLCPU)
            {
                test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id));
                continue;
            }
            for (uint32_t row = 0; row < group->prims.count; ++row)
            {
                const size_t index = group->prims.index_map ? group->prims.index_map[row] : group->prims.offset + row;
                if (index >= pathCount)
                    continue;
                ovx_string_t text{};
                if (ovx_path_dictionary_path_to_string(dict, paths[index], &text) != OVX_OK || !text.ptr)
                    continue;
                const float* src = values + static_cast<size_t>(row) * static_cast<size_t>(width);
                out[std::string(text.ptr, text.length)] = { src[0], src[1], src[2] };
            }
        }
        test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id));
    }
    test_utils::complete_task(handle, ovphysx_release_read(handle, read));
    test_utils::complete_task(handle, ovphysx_release_query(handle, query));
    return out;
}

bool commitMove(ovphysx_instance_handle_t handle, ovphysx_query_handle_t query, float x, float y, float z)
{
    const ovx_string_or_token_t name = attrName(OVPHYSX_ATTR_CCT_MOVE);
    ovphysx_write_handle_t write = 0;
    ovphysx_write_result_t summary{};
    if (test_utils::complete_task(handle, ovphysx_write(handle, query, &name, 1, &write, &summary)).status != OVPHYSX_API_SUCCESS)
        return false;
    EXPECT_EQ(summary.total_prim_count, 1u);
    EXPECT_EQ(summary.attribute_count, 1u);
    EXPECT_NE(summary.attributes, nullptr);
    if (summary.attributes)
    {
        EXPECT_EQ(summary.attributes[0].group_count, 1u);
    }
    ovphysx_fetched_write_group_t fetched{};
    const ovphysx_result_t result = test_utils::complete_task(handle, ovphysx_fetch_write_next(handle, write, &fetched));
    const ovstage_map_group_t* group = fetched.group;
    EXPECT_EQ(fetched.attribute_index, 0u);
    if (result.status != OVPHYSX_API_SUCCESS || !group || group->prims.count != 1 ||
        !group->data.tensors || !group->data.tensors[0].data)
    {
        test_utils::complete_task(handle, ovphysx_release_write(handle, write));
        return false;
    }
    float* dst = static_cast<float*>(group->data.tensors[0].data);
    dst[0] = x;
    dst[1] = y;
    dst[2] = z;
    ovstage_cuda_sync_t sync{};
    const bool committed =
        test_utils::complete_task(handle, ovphysx_commit_group(handle, write, group, sync)).status == OVPHYSX_API_SUCCESS;
    test_utils::complete_task(handle, ovphysx_release_write(handle, write));
    return committed;
}
} // namespace

class OutputReadTest : public PhysXTestFixture
{
};

// Query rigid bodies and read position + orientation as faithful column groups.
TEST_F(OutputReadTest, RigidBodyPositionAndOrientation)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to attach ovstage USD";

    stepN(m_handle, 10); // let the boxes fall

    // Discovery: query creation reports its matched prim count + produced attributes
    // (ovstage's own ovstage_query_result_t), usable once the task completes.
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t qr{};
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, &qr)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(query, 0u) << "expected at least one rigid body in the scene";
    EXPECT_GT(qr.total_prim_count, 0u);

    // The interned identifiers resolve through the attached Stage's own path dictionary.
    EXPECT_NE(attached_ovstage_dictionary(m_handle), nullptr);

    const ovx_string_or_token_t names[] = {
        { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
        { 0, { OVPHYSX_ATTR_ORIENTATION, sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
    };
    ovphysx_read_handle_t read = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_read(m_handle, query, names, 2, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(read, 0u);

    int groups = 0;
    int64_t posRows = 0;
    int64_t oriRows = 0;
    for (;;)
    {
        // Producer-owned group: fetch hands back a borrowed ovstage_read_group_t*.
        const ovstage_read_group_t* gp = nullptr;
        const ovphysx_result_t r = test_utils::complete_task(m_handle, ovphysx_fetch_read_next(m_handle, read, &gp));
        if (r.status == OVPHYSX_API_SUCCESS && !gp)
            break;
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(gp, nullptr);
        const ovstage_read_group_t& g = *gp;
        ++groups;
        EXPECT_NE(g.attribute, 0u);                 // interned attribute token
        EXPECT_FALSE(g.is_array);
        EXPECT_GT(g.prims.count, 0u);
        EXPECT_NE(g.prims.list, 0u);                // interned prim-list handle
        ASSERT_GE(g.data.tensor_count, 1u);
        ASSERT_NE(g.data.tensors, nullptr);
        const DLTensor t = g.data.tensors[0];
        ASSERT_NE(t.shape, nullptr);
        ASSERT_GE(t.ndim, 1);
        const int64_t rowsBeforeGroupRelease = t.shape[0];
        // A fixed column stacks all prims along shape[0]. The tuple width is
        // carried in dtype.lanes (vec3 / quat), not in a trailing shape dim.
        if (t.dtype.lanes == 3)
            posRows = t.shape[0]; // position
        else if (t.dtype.lanes == 4)
            oriRows = t.shape[0]; // orientation (quat)
        // Releasing the group retires its struct/path metadata. Numeric descriptor,
        // shape, and data storage remain owned by the read session until release_read.
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_group(m_handle, read, g.read_group_id)).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(t.shape[0], rowsBeforeGroupRelease);
    }

    EXPECT_GE(groups, 2);                                  // at least a position + an orientation group
    EXPECT_GT(posRows, 0);                                 // at least one rigid body
    EXPECT_EQ(posRows, oriRows);                           // same body set in both columns
    EXPECT_EQ((int64_t)qr.total_prim_count, posRows);      // discovery count matches the column rows

    // @implements REQ-CAPI-ASYNC-002
    // @covers AC-3 AC-5 AC-6
    const ovphysx_enqueue_result_t beforeReadRelease = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(beforeReadRelease.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, beforeReadRelease).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t beforeQueryRelease = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(beforeQueryRelease.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, beforeQueryRelease).status, OVPHYSX_API_SUCCESS);
}

// prim_paths wins over pattern in ovphysx_query_selection (REQ-CAPI-SELECT-001 AC-2). Precedence is
// resolved at THIS C boundary -- the runtime receives the native handle list with the pattern suppressed
// -- so it is tested here rather than in an ovruntime doctest. The glob "/World/Cube*" matches every cube;
// supplying it alongside a single interned prim_paths handle must yield exactly that one prim (the pattern
// ignored), which total_prim_count alone shows without resolving any string.
TEST_F(OutputReadTest, QuerySelectionPrimPathsWinsOverPattern)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to attach ovstage USD";
    stepN(m_handle, 3); // a few steps; the cubes are still falling, so scope does not matter here

    const ovphysx_string_t cubeGlob = { "/World/Cube*", sizeof("/World/Cube*") - 1 };

    // The glob alone matches many cubes -- the count the precedence case must NOT collapse to by accident.
    ovphysx_query_selection_t patternOnly{};
    patternOnly.pattern = cubeGlob;
    patternOnly.prim_paths = nullptr;
    patternOnly.prim_paths_count = 0;
    ovphysx_query_handle_t patternQuery = 0;
    ovstage_query_result_t patternQr{};
    ASSERT_EQ(test_utils::complete_task(
                  m_handle,
                  ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &patternOnly, &patternQuery, &patternQr))
                  .status,
              OVPHYSX_API_SUCCESS);
    EXPECT_GT(patternQr.total_prim_count, 1u) << "the glob should match more than one cube";

    // The explicit list is native handles, not strings: intern one cube's path through the attached Stage's
    // dictionary -- what a caller does with a handle gathered from a read's prim_list or from ovstage.
    ovx_path_dictionary_t* dict = attached_ovstage_dictionary(m_handle);
    ASSERT_NE(dict, nullptr);
    ovx_primpath_t oneCube = OVX_INVALID_PRIMPATH;
    const ovx_string_t oneCubeStr = { "/World/Cube1", sizeof("/World/Cube1") - 1 };
    ASSERT_EQ(ovx_path_dictionary_intern_path(dict, oneCubeStr, &oneCube), OVX_OK);
    ASSERT_NE(oneCube, OVX_INVALID_PRIMPATH);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, patternQuery)).status,
              OVPHYSX_API_SUCCESS);

    // Both set: prim_paths wins and the pattern is ignored -- exactly the one named cube.
    ovphysx_query_selection_t both{};
    both.pattern = cubeGlob;    // would match every cube, but must be ignored
    both.prim_paths = &oneCube; // native handle; wins
    both.prim_paths_count = 1;
    ovphysx_query_handle_t bothQuery = 0;
    ovstage_query_result_t bothQr{};
    ASSERT_EQ(test_utils::complete_task(
                  m_handle,
                  ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &both, &bothQuery, &bothQr))
                  .status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(bothQr.total_prim_count, 1u) << "prim_paths must win; the pattern must be ignored";
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, bothQuery)).status,
              OVPHYSX_API_SUCCESS);
}

// Every malformed selection is rejected on the receipt, before any task exists (REQ-CAPI-SELECT-001 AC-3):
// the status is INVALID_ARGUMENT, no op index is issued and no query is handed back. The receipt is checked
// directly, since complete_task would accept a failure reported by the task as well. The NULL-with-count
// case carries a valid pattern, so falling back to it would succeed instead of failing.
TEST_F(OutputReadTest, QuerySelectionRejectsInvalidSelectionsOnTheReceipt)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to attach ovstage USD";
    stepN(m_handle, 1);

    const std::string cubeGlob = "/World/Cube*";
    const std::string embeddedNul = std::string("/World/Cube") + '\0' + "*";
    const std::string oversized = "/World/" + std::string(4097, 'a');
    const std::string relative = "World/Cube*";
    const ovx_primpath_t invalidHandle = OVX_INVALID_PRIMPATH;
    const auto text = [](const std::string& s) { return ovphysx_string_t{ s.data(), s.size() }; };

    struct Case
    {
        const char* name;
        ovphysx_query_selection_t selection;
        const char* message = ""; // expected in ovphysx_get_last_error()
    };
    std::vector<Case> cases(6);
    cases[0].name = "neither prim_paths nor a pattern";
    cases[1].name = "NULL prim_paths with a non-zero count";
    cases[1].selection.pattern = text(cubeGlob);
    cases[1].selection.prim_paths_count = 1;
    cases[2].name = "OVX_INVALID_PRIMPATH in prim_paths";
    cases[2].selection.prim_paths = &invalidHandle;
    cases[2].selection.prim_paths_count = 1;
    cases[3].name = "embedded NUL in the pattern";
    cases[3].selection.pattern = text(embeddedNul);
    cases[4].name = "oversized pattern component";
    cases[4].selection.pattern = text(oversized);
    cases[4].message = "longer than 4096 characters";
    cases[5].name = "relative pattern";
    cases[5].selection.pattern = text(relative);

    for (const Case& c : cases)
    {
        ovphysx_query_handle_t query = 0;
        const ovphysx_enqueue_result_t receipt =
            ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &c.selection, &query, nullptr);
        EXPECT_EQ(receipt.status, OVPHYSX_API_INVALID_ARGUMENT) << c.name;
        EXPECT_EQ(receipt.op_index, 0u) << c.name << ": a rejected selection must not issue a task";
        EXPECT_EQ(query, 0u) << c.name << ": a rejected selection must not hand back a query";
        const ovphysx_string_t error = ovphysx_get_last_error();
        EXPECT_NE(std::string(error.ptr ? error.ptr : "", error.length).find(c.message), std::string::npos)
            << c.name;
    }

    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t receipt =
        ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, nullptr, &query, nullptr);
    EXPECT_EQ(receipt.status, OVPHYSX_API_INVALID_ARGUMENT) << "NULL selection";
    EXPECT_EQ(receipt.op_index, 0u) << "NULL selection";
    EXPECT_EQ(query, 0u) << "NULL selection";
}

// The whole-articulation DOF-column descriptor is reachable through the C ABI, and is the non-binding
// successor to ovphysx_articulation_get_dof/joint_names: each DOF column resolves through the path
// dictionary to its owning JOINT prim (identity by prim PATH, not a string) at a known axis, and the
// columns carry the uint64 handle / int32 axis dtypes the contract promises. The fixture is two
// fixed-base articulations of two revolute joints each -> four distinct one-axis DOFs.
TEST_F(OutputReadTest, ArticulationDofColumnDescriptor)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"))
        << "Failed to attach ovstage USD";
    stepN(m_handle, 1);

    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(query, 0u);

    ovx_path_dictionary_t* dict = attached_ovstage_dictionary(m_handle);
    ASSERT_NE(dict, nullptr);

    const ovx_string_or_token_t names[] = {
        { 0, { OVPHYSX_ATTR_DOF_COUNT, sizeof(OVPHYSX_ATTR_DOF_COUNT) - 1 } },
        { 0, { OVPHYSX_ATTR_DOF_JOINT_PATH, sizeof(OVPHYSX_ATTR_DOF_JOINT_PATH) - 1 } },
        { 0, { OVPHYSX_ATTR_DOF_AXIS_INDEX, sizeof(OVPHYSX_ATTR_DOF_AXIS_INDEX) - 1 } },
    };
    ovphysx_read_handle_t read = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_read(m_handle, query, names, 3, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(read, 0u);

    // Keep each group's whole [rows x lanes] block; dofCount (one per row) is each row's real length. The
    // three groups share one prim set and row order (all keyed on the articulation roots), so row r is the
    // same articulation in each.
    int64_t pathRows = 0;
    int64_t dofRows = 0;
    uint16_t pathLanes = 0;
    uint16_t axisLanes = 0;
    std::vector<uint64_t> pathVals;
    std::vector<int32_t> axisVals;
    std::vector<int32_t> dofVals;
    bool sawPath = false;
    bool sawAxis = false;
    bool sawDof = false;

    for (;;)
    {
        const ovstage_read_group_t* gp = nullptr;
        const ovphysx_result_t r = test_utils::complete_task(m_handle, ovphysx_fetch_read_next(m_handle, read, &gp));
        if (r.status == OVPHYSX_API_SUCCESS && !gp)
            break;
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(gp, nullptr);
        const ovstage_read_group_t& g = *gp;
        ASSERT_GE(g.data.tensor_count, 1u);
        ASSERT_NE(g.data.tensors, nullptr);
        const DLTensor t = g.data.tensors[0];
        ASSERT_NE(t.data, nullptr);
        ASSERT_NE(t.shape, nullptr);
        const int64_t rows = t.shape[0];
        const uint16_t lanes = t.dtype.lanes;

        ovx_string_t attrName{};
        ASSERT_EQ(ovx_path_dictionary_token_to_string(dict, g.attribute, &attrName), OVX_OK);
        const std::string name(attrName.ptr, attrName.length);

        if (name == OVPHYSX_ATTR_DOF_COUNT)
        {
            sawDof = true;
            dofRows = rows;
            const int32_t* d = static_cast<const int32_t*>(t.data);
            dofVals.assign(d, d + rows * lanes);
        }
        else if (name == OVPHYSX_ATTR_DOF_JOINT_PATH)
        {
            sawPath = true;
            EXPECT_EQ(t.dtype.code, kDLUInt);
            EXPECT_EQ(t.dtype.bits, 64);
            pathRows = rows;
            pathLanes = lanes;
            const uint64_t* h = static_cast<const uint64_t*>(t.data);
            pathVals.assign(h, h + rows * lanes);
        }
        else if (name == OVPHYSX_ATTR_DOF_AXIS_INDEX)
        {
            sawAxis = true;
            EXPECT_EQ(t.dtype.code, kDLInt);
            EXPECT_EQ(t.dtype.bits, 32);
            axisLanes = lanes;
            const int32_t* a = static_cast<const int32_t*>(t.data);
            axisVals.assign(a, a + rows * lanes);
        }
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_group(m_handle, read, g.read_group_id)).status, OVPHYSX_API_SUCCESS);
    }

    ASSERT_TRUE(sawPath);
    ASSERT_TRUE(sawAxis);
    ASSERT_TRUE(sawDof);
    ASSERT_EQ(pathRows, 2); // two articulations in the fixture
    ASSERT_EQ(dofRows, 2);
    ASSERT_EQ(pathLanes, axisLanes);

    // Every real DOF (bounded by its row's dofCount) names a DISTINCT revolute joint prim at axis 0, and a
    // real handle is never the null pad. Four DOFs total across the two articulations.
    std::set<std::string> jointPaths;
    int realDofs = 0;
    for (int64_t row = 0; row < pathRows; ++row)
    {
        const int32_t rowDof = dofVals[static_cast<size_t>(row)];
        EXPECT_EQ(rowDof, 2); // two revolute joints per articulation
        for (int32_t k = 0; k < rowDof; ++k)
        {
            const size_t idx = static_cast<size_t>(row) * pathLanes + k;
            EXPECT_NE(pathVals[idx], 0ull) << "a real DOF must name its joint";
            ovx_string_t p{};
            ASSERT_EQ(ovx_path_dictionary_path_to_string(dict, pathVals[idx], &p), OVX_OK);
            ASSERT_NE(p.ptr, nullptr);
            const std::string jointPath(p.ptr, p.length);
            EXPECT_NE(jointPath.find("RevoluteJoint"), std::string::npos) << jointPath;
            jointPaths.insert(jointPath);
            EXPECT_EQ(axisVals[idx], 0); // a revolute joint is a single axis
            ++realDofs;
        }
    }
    EXPECT_EQ(realDofs, 4);            // 2 articulations x 2 DOF
    EXPECT_EQ(jointPaths.size(), 4u); // every DOF names a distinct joint prim (unique prim-path identity)

    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status, OVPHYSX_API_SUCCESS);
}

// Null-argument and lifecycle edge cases (C-ABI guarantees).
TEST_F(OutputReadTest, ArgumentValidation)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    stepN(m_handle, 1);

    // null out_query (the optional summary does not stand in for the handle)
    ovstage_query_result_t summary{};
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, nullptr, &summary)).status,
              OVPHYSX_API_INVALID_ARGUMENT);

    // A NULL optional summary is valid and skips discovery.
    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);

    // null out args on read / fetch
    ovphysx_read_handle_t read = 0;
    const ovx_string_or_token_t names[] = {
        { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
    };
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_read(m_handle, query, names, 1, nullptr, nullptr)).status, OVPHYSX_API_INVALID_ARGUMENT);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_read(m_handle, query, names, 1, &read, nullptr)).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_fetch_read_next(m_handle, read, nullptr)).status, OVPHYSX_API_INVALID_ARGUMENT);

    // release_group accepts an unknown id. release_read and release_query are idempotent.
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_group(m_handle, read, 123456789ull)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status, OVPHYSX_API_SUCCESS);
}

// @implements REQ-CAPI-WRITE-001
// @covers AC-15
// The character-controller type must pass ovphysx_query_selection. Its rigid-body actor pose
// stays put until the step that integrates the kinematic target.
TEST_F(OutputReadTest, CharacterControllerSelectionMovesOneController)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/character_controller.usda"));

    ovphysx_query_handle_t discovery = 0;
    ovstage_query_result_t discovered{};
    ASSERT_EQ(test_utils::complete_task(
                  m_handle,
                  ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, OVPHYSX_SCOPE_ALL, &discovery, &discovered))
                  .status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(discovered.total_prim_count, 2u);
    EXPECT_EQ(discovered.attribute_count, 0u);
    ovx_path_dictionary_t* dict = attached_ovstage_dictionary(m_handle);
    ASSERT_NE(dict, nullptr);
    const ovx_string_t controllerName = { "/World/Controller", sizeof("/World/Controller") - 1 };
    ovx_primpath_t controller = 0;
    ASSERT_EQ(ovx_path_dictionary_intern_path(dict, controllerName, &controller), OVX_OK);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, discovery)).status, OVPHYSX_API_SUCCESS);

    const auto start = rigidPositions(m_handle);
    ASSERT_EQ(start.count("/World/Controller"), 1u);
    ASSERT_EQ(start.count("/World/Other"), 1u);

    ovphysx_query_selection_t byPath{};
    byPath.prim_paths = &controller;
    byPath.prim_paths_count = 1;
    ovphysx_query_handle_t pathQuery = 0;
    ovstage_query_result_t pathResult{};
    ASSERT_EQ(test_utils::complete_task(
                  m_handle,
                  ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, &byPath, &pathQuery, &pathResult))
                  .status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(pathResult.total_prim_count, 1u);
    EXPECT_EQ(pathResult.attribute_count, 0u);
    ASSERT_TRUE(commitMove(m_handle, pathQuery, 2.0f, 0.0f, 0.0f));
    ASSERT_TRUE(commitMove(m_handle, pathQuery, 1.0f, 0.0f, 0.0f));
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, pathQuery)).status, OVPHYSX_API_SUCCESS);

    ovphysx_query_selection_t byPattern{};
    byPattern.pattern = { "/World/Other", sizeof("/World/Other") - 1 };
    ovphysx_query_handle_t patternQuery = 0;
    ovstage_query_result_t patternResult{};
    ASSERT_EQ(test_utils::complete_task(
                  m_handle,
                  ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, &byPattern, &patternQuery, &patternResult))
                  .status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(patternResult.total_prim_count, 1u);
    EXPECT_EQ(patternResult.attribute_count, 0u);
    ASSERT_TRUE(commitMove(m_handle, patternQuery, 0.0f, 0.0f, 1.0f));
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, patternQuery)).status,
              OVPHYSX_API_SUCCESS);

    const auto held = rigidPositions(m_handle);
    EXPECT_NEAR(held.at("/World/Controller")[0], start.at("/World/Controller")[0], 1e-4f);
    EXPECT_NEAR(held.at("/World/Other")[2], start.at("/World/Other")[2], 1e-4f);

    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const auto moved = rigidPositions(m_handle);
    EXPECT_NEAR(moved.at("/World/Controller")[0], start.at("/World/Controller")[0] + 3.0f, 1e-3f);
    EXPECT_NEAR(moved.at("/World/Controller")[1], start.at("/World/Controller")[1], 1e-3f);
    EXPECT_NEAR(moved.at("/World/Other")[0], start.at("/World/Other")[0], 1e-3f);
    EXPECT_NEAR(moved.at("/World/Other")[2], start.at("/World/Other")[2] + 1.0f, 1e-3f);

    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const auto replayed = rigidPositions(m_handle);
    EXPECT_NEAR(replayed.at("/World/Controller")[0], moved.at("/World/Controller")[0], 1e-3f);
    EXPECT_NEAR(replayed.at("/World/Other")[2], moved.at("/World/Other")[2], 1e-3f);
}
