// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-JOINT-ARRAYS-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_utils.h>
#include <ovstage/ovx_path_dictionary.h>

#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{
struct DestroyJointArrays
{
    void operator()(ovphysx_joint_arrays_t* arrays) const
    {
        ovphysx_destroy_joint_arrays(arrays);
    }
};
using JointArraysPtr = std::unique_ptr<ovphysx_joint_arrays_t, DestroyJointArrays>;

std::string lastError()
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    return std::string(error.ptr ? error.ptr : "", error.length);
}

// The humanoid's joints have one, two and three axes, so its joint groups repeat paths.
constexpr const char* kHumanoid = "tests/benchmarks/data/humanoid.usda";

ovx_token_or_string_t attribute(const char* name)
{
    return { 0, { name, std::strlen(name) } };
}

bool complete(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    if (enqueue.status != OVSTAGE_OK)
        return false;
    if (enqueue.op_index == OVSTAGE_INVALID_OP_ID)
        return true;
    ovstage_op_wait_result_t wait{};
    const ovstage_api_status_t status = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait);
    const bool ok = status == OVSTAGE_OK && wait.error_op_id_count == 0;
    for (size_t i = 0; i < wait.error_op_id_count; ++i)
    {
        const ovx_string_t error = ovstage_get_last_op_error(stage, wait.error_op_ids[i]);
        ADD_FAILURE() << std::string(error.ptr ? error.ptr : "", error.length);
    }
    return ovstage_release_op(stage, enqueue.op_index) == OVSTAGE_OK && ok;
}

ovstage_instance_t* attachedStage(ovphysx_instance_handle_t handle)
{
    std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
    return test_utils::ovstage_test_attachments().at(handle).back().stage;
}

std::vector<std::string> listPaths(ovx_path_dictionary_t* dictionary, ovx_primpath_list_t list, uint32_t offset,
                                   uint32_t count)
{
    const ovx_primpath_t* paths = nullptr;
    size_t available = 0;
    EXPECT_EQ(ovx_path_dictionary_get_paths(dictionary, list, &paths, &available), OVX_OK);
    std::vector<std::string> out;
    for (uint32_t row = 0; paths && row < count && offset + row < available; ++row)
    {
        ovx_string_t name{};
        EXPECT_EQ(ovx_path_dictionary_path_to_string(dictionary, paths[offset + row], &name), OVX_OK);
        out.emplace_back(name.ptr, name.length);
    }
    EXPECT_EQ(out.size(), count);
    return out;
}

// One joint's rows in a joint group: they are consecutive and name the joint.
struct JointRun
{
    std::string path;
    uint32_t first = 0;
    uint32_t count = 0;
};

std::vector<JointRun> runsOf(const std::vector<std::string>& rowPaths)
{
    std::vector<JointRun> runs;
    for (uint32_t row = 0; row < rowPaths.size(); ++row)
    {
        if (!runs.empty() && runs.back().path == rowPaths[row])
            ++runs.back().count;
        else
            runs.push_back({ rowPaths[row], row, 1 });
    }
    return runs;
}

// The rows of one joint attribute, read through the session API, with each row's joint.
struct JointColumn
{
    std::vector<std::string> paths;
    std::vector<float> values;
};

JointColumn readJointColumn(ovphysx_instance_handle_t handle, ovx_path_dictionary_t* dictionary, const char* name)
{
    JointColumn column;
    ovphysx_query_handle_t query = 0;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT,
                                                              OVPHYSX_SCOPE_ALL, &query, nullptr))
                  .status,
              OVPHYSX_API_SUCCESS);
    const ovx_token_or_string_t requested = attribute(name);
    ovphysx_read_handle_t read = 0;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_read(handle, query, &requested, 1, ovstage_cuda_sync_t{ 0, 0 },
                                                             &read, nullptr))
                  .status,
              OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* group = nullptr;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_fetch_read_next(handle, read, &group)).status,
              OVPHYSX_API_SUCCESS);
    if (group)
    {
        const DLTensor& tensor = group->columns[0].data.tensors[0];
        const float* rows = reinterpret_cast<const float*>(static_cast<const char*>(tensor.data) + tensor.byte_offset);
        column.paths = listPaths(dictionary, group->prims.list, group->prims.offset, group->prims.count);
        column.values.assign(rows, rows + group->prims.count);
        EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)).status,
                  OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_read(handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status, OVPHYSX_API_SUCCESS);
    return column;
}

// The `name` array of each prim of `paths` on the stage at `ordinal`.
std::map<std::string, std::vector<float>> readStageArrays(ovstage_instance_t* stage,
                                                          const std::vector<std::string>& paths,
                                                          const char* name,
                                                          ovstage_ordinal_t ordinal)
{
    std::map<std::string, std::vector<float>> out;
    ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
    std::vector<ovx_string_t> strings;
    for (const std::string& path : paths)
        strings.push_back({ path.data(), path.size() });
    ovx_primpath_list_t list{};
    EXPECT_EQ(ovx_path_dictionary_create_path_list_from_strings(dictionary, strings.data(), strings.size(), &list),
              OVX_OK);
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    EXPECT_EQ(ovstage_query_from_path_list(stage, list, &query), OVSTAGE_OK);
    ovx_token_t token{};
    EXPECT_EQ(ovx_path_dictionary_intern_token(dictionary, attribute(name).string, &token), OVX_OK);
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    EXPECT_TRUE(complete(stage, ovstage_read_attributes(stage, query, &token, 1, { 0, ordinal, false }, &read)));
    for (;;)
    {
        ovstage_read_group_t group{};
        size_t fetched = 0;
        const ovstage_api_status_t status =
            ovstage_fetch_read_next(stage, read, OVSTAGE_TIMEOUT_INFINITE, &group, 1, &fetched);
        if (status != OVSTAGE_OK)
        {
            EXPECT_EQ(status, OVSTAGE_ERROR_END_OF_ITERATION);
            break;
        }
        EXPECT_EQ(group.column_count, 1u);
        const ovstage_read_column_t& column = group.columns[0];
        if (!column.is_delete)
        {
            EXPECT_TRUE(column.is_array);
            const std::vector<std::string> names =
                listPaths(dictionary, group.prims.list, group.prims.offset, group.prims.count);
            for (uint32_t row = 0; row < names.size(); ++row)
            {
                const uint32_t index = column.data.index_map ? column.data.index_map[row] : row;
                const DLTensor& tensor = column.data.tensors[index];
                const float* values =
                    reinterpret_cast<const float*>(static_cast<const char*>(tensor.data) + tensor.byte_offset);
                out[names[row]].assign(values, values + tensor.shape[0] * tensor.dtype.lanes);
            }
        }
        EXPECT_EQ(ovstage_release_groups(stage, &group, 1), OVSTAGE_OK);
    }
    EXPECT_TRUE(complete(stage, ovstage_release_read(stage, read)));
    EXPECT_TRUE(complete(stage, ovstage_release_query(stage, query)));
    EXPECT_EQ(ovx_path_dictionary_destroy_path_list(dictionary, list), OVX_OK);
    return out;
}
} // namespace

class JointArraysTest : public PhysXTestFixture
{
};

// A joint read group, one row per DOF, regroups into one view per joint over the group's own column and a list
// naming each joint once, and its write payload lands on the stage as one array per joint prim.
TEST_F(JointArraysTest, ReadGroupBecomesOneArrayPerJointOnTheStage)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kHumanoid));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT,
                                                                OVPHYSX_SCOPE_ALL, &query, nullptr))
                  .status,
              OVPHYSX_API_SUCCESS);
    const ovx_token_or_string_t name = attribute(OVPHYSX_ATTR_JOINT_POSITION);
    ovphysx_read_handle_t read = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_read(m_handle, query, &name, 1, ovstage_cuda_sync_t{ 0, 0 },
                                                               &read, nullptr))
                  .status,
              OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* group = nullptr;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(group, nullptr);
    ASSERT_EQ(group->column_count, 1u);
    ASSERT_FALSE(group->columns[0].is_array);
    ASSERT_EQ(group->columns[0].data.tensor_count, 1u);
    const DLTensor& column = group->columns[0].data.tensors[0];
    ASSERT_EQ(column.device.device_type, kDLCPU);
    const std::vector<std::string> rowPaths =
        listPaths(dictionary, group->prims.list, group->prims.offset, group->prims.count);
    const std::vector<JointRun> runs = runsOf(rowPaths);
    ASSERT_LT(runs.size(), rowPaths.size()) << "the fixture must have a multi-DOF joint";
    const float* rows = reinterpret_cast<const float*>(static_cast<const char*>(column.data) + column.byte_offset);

    ovphysx_joint_arrays_t* created = nullptr;
    ASSERT_EQ(ovphysx_create_joint_arrays_from_read_group(stage, group, &created).status, OVPHYSX_API_SUCCESS)
        << lastError();
    const JointArraysPtr joints(created);
    const ovstage_write_data_t write = ovphysx_get_joint_arrays_write_data(joints.get());
    EXPECT_TRUE(write.is_array);
    EXPECT_EQ(write.semantic, group->columns[0].semantic);
    ASSERT_EQ(write.tensor_count, runs.size());
    const std::vector<std::string> jointPaths =
        listPaths(dictionary, ovphysx_get_joint_arrays_prim_list(joints.get()), 0, write.tensor_count);
    std::map<std::string, std::vector<float>> expected;
    for (size_t j = 0; j < runs.size(); ++j)
    {
        SCOPED_TRACE(runs[j].path);
        EXPECT_EQ(jointPaths[j], runs[j].path);
        const DLTensor& view = write.tensors[j];
        EXPECT_EQ(view.data, column.data);
        EXPECT_EQ(view.byte_offset, column.byte_offset + runs[j].first * sizeof(float));
        ASSERT_NE(view.shape, nullptr);
        EXPECT_EQ(view.shape[0], int64_t(runs[j].count));
        EXPECT_EQ(view.ndim, 1);
        EXPECT_EQ(view.dtype.code, column.dtype.code);
        EXPECT_EQ(view.dtype.bits, column.dtype.bits);
        EXPECT_EQ(view.dtype.lanes, column.dtype.lanes);
        EXPECT_EQ(view.device.device_type, column.device.device_type);
        expected[runs[j].path].assign(rows + runs[j].first, rows + runs[j].first + runs[j].count);
    }

    ovstage_query_handle_t stageQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ASSERT_EQ(ovstage_query_from_path_list(stage, ovphysx_get_joint_arrays_prim_list(joints.get()), &stageQuery),
              OVSTAGE_OK);
    EXPECT_TRUE(complete(stage, ovstage_write_attribute(stage, stageQuery, attribute("sim:jointPosition"), 2, write,
                                                        OVSTAGE_PRIM_MODE_UPSERT)));
    EXPECT_TRUE(complete(stage, ovstage_release_query(stage, stageQuery)));
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_group(m_handle, read, group->read_group_id)).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status,
              OVPHYSX_API_SUCCESS);

    ovstage_write_floor_desc_t floor{};
    floor.ordinal = 2;
    floor.scope = OVSTAGE_SCOPE_ALL;
    ASSERT_TRUE(complete(stage, ovstage_advance_write_floor(stage, &floor)));
    EXPECT_EQ(readStageArrays(stage, jointPaths, "sim:jointPosition", 2), expected);
}

// The views of a write group fill its column: each joint's DOFs take its own value, and the commit applies them.
TEST_F(JointArraysTest, WriteGroupViewsFillTheColumnJointByJoint)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kHumanoid));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT,
                                                                OVPHYSX_SCOPE_ALL, &query, nullptr))
                  .status,
              OVPHYSX_API_SUCCESS);
    const ovx_token_or_string_t name = attribute(OVPHYSX_ATTR_JOINT_POSITION);
    ovphysx_write_handle_t write = 0;
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_write(m_handle, query, &name, 1, &write, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(fetched.group, nullptr);

    ovphysx_joint_arrays_t* created = nullptr;
    ASSERT_EQ(ovphysx_create_joint_arrays_from_map_group(stage, fetched.group, &created).status, OVPHYSX_API_SUCCESS)
        << lastError();
    const JointArraysPtr joints(created);
    const ovstage_write_data_t views = ovphysx_get_joint_arrays_write_data(joints.get());
    const std::vector<std::string> jointPaths =
        listPaths(dictionary, ovphysx_get_joint_arrays_prim_list(joints.get()), 0, views.tensor_count);
    std::map<std::string, float> wrote;
    for (uint32_t j = 0; j < views.tensor_count; ++j)
    {
        const DLTensor& view = views.tensors[j];
        float* values = reinterpret_cast<float*>(static_cast<char*>(view.data) + view.byte_offset);
        // A small angle per joint, distinct from every other joint's.
        wrote[jointPaths[j]] = 1.0f + float(j);
        for (int64_t a = 0; a < view.shape[0]; ++a)
            values[a] = wrote[jointPaths[j]];
    }
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_commit_group(m_handle, write, fetched.group,
                                                                       ovstage_cuda_sync_t{ 0, 0 }))
                  .status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_write(m_handle, write)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status,
              OVPHYSX_API_SUCCESS);

    const JointColumn after = readJointColumn(m_handle, dictionary, OVPHYSX_ATTR_JOINT_POSITION);
    ASSERT_EQ(after.paths.size(), after.values.size());
    ASSERT_FALSE(after.paths.empty());
    for (size_t row = 0; row < after.paths.size(); ++row)
    {
        SCOPED_TRACE(after.paths[row]);
        ASSERT_EQ(wrote.count(after.paths[row]), 1u);
        EXPECT_NEAR(after.values[row], wrote.at(after.paths[row]), 1e-3f);
    }
}

// A null argument, a read group without exactly one column, a deleted group or a group not shaped as a joint group
// is refused with no object and a message; the getters and destroy accept NULL.
TEST_F(JointArraysTest, RefusesAGroupNotShapedAsAJointGroup)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kHumanoid));
    ovstage_instance_t* stage = attachedStage(m_handle);
    int64_t shape = 1;
    float values[2] = { 0.0f, 0.0f };
    DLTensor tensors[2]{};
    for (DLTensor& tensor : tensors)
    {
        tensor.data = values;
        tensor.device = { kDLCPU, 0 };
        tensor.ndim = 1;
        tensor.shape = &shape;
        tensor.dtype = { kDLFloat, 32, 1 };
    }
    ovstage_read_column_t twoTensorColumn{};
    twoTensorColumn.data.tensors = tensors;
    twoTensorColumn.data.tensor_count = 2;
    ovstage_read_group_t twoTensors{};
    twoTensors.prims.count = 2;
    twoTensors.columns = &twoTensorColumn;
    twoTensors.column_count = 1;
    ovstage_read_column_t deletedColumn{};
    deletedColumn.is_delete = true;
    ovstage_read_group_t deleted{};
    deleted.columns = &deletedColumn;
    deleted.column_count = 1;
    ovstage_read_group_t noColumn{};
    noColumn.prims.count = 2;
    ovstage_map_group_t mapTwoTensors{};
    mapTwoTensors.prims = twoTensors.prims;
    mapTwoTensors.data = twoTensorColumn.data;

    const auto refuses = [](ovphysx_result_t result, ovphysx_joint_arrays_t* created)
    {
        EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(created, nullptr);
        EXPECT_FALSE(lastError().empty());
    };
    ovphysx_joint_arrays_t* created = nullptr;
    refuses(ovphysx_create_joint_arrays_from_read_group(stage, &twoTensors, &created), created);
    refuses(ovphysx_create_joint_arrays_from_read_group(stage, &deleted, &created), created);
    refuses(ovphysx_create_joint_arrays_from_read_group(stage, &noColumn, &created), created);
    refuses(ovphysx_create_joint_arrays_from_read_group(stage, nullptr, &created), created);
    refuses(ovphysx_create_joint_arrays_from_read_group(nullptr, &twoTensors, &created), created);
    refuses(ovphysx_create_joint_arrays_from_map_group(stage, &mapTwoTensors, &created), created);
    EXPECT_EQ(ovphysx_create_joint_arrays_from_read_group(stage, &twoTensors, nullptr).status,
              OVPHYSX_API_INVALID_ARGUMENT);

    EXPECT_EQ(ovphysx_get_joint_arrays_prim_list(nullptr), OVX_INVALID_PRIMPATH_LIST);
    EXPECT_EQ(ovphysx_get_joint_arrays_write_data(nullptr).tensor_count, 0u);
    EXPECT_EQ(ovphysx_get_joint_arrays_write_data(nullptr).tensors, nullptr);
    ovphysx_destroy_joint_arrays(nullptr);
}
