// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-JOINT-ARRAYS-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include "ovphysx/ovphysx_utils.h"

#include "internal/Profiling.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <ovstage/ovstage_api/ovstage_api.h>
#include <ovx/path_dictionary/path_dictionary.h>

#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

struct ovphysx_joint_arrays_t
{
    path_dictionary_instance_t* dictionary = nullptr;
    // One reference this object owns, released with it.
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    std::vector<int64_t> shapes;
    std::vector<DLTensor> tensors;
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    ovstage_cuda_sync_t cudaSync{};

    ~ovphysx_joint_arrays_t()
    {
        if (dictionary && list != OVX_INVALID_PRIMPATH_LIST)
        {
            const ovx_api_result_t result = path_dictionary_release_path_list_reference(dictionary, list);
            if (result.status != OVX_API_SUCCESS)
                path_dictionary_release_error(dictionary, result.error);
        }
    }
};

namespace
{
struct JointArraysFailure : std::runtime_error
{
    ovphysx_api_status_t status;
    JointArraysFailure(ovphysx_api_status_t value, const std::string& message)
        : std::runtime_error(message), status(value)
    {
    }
};

void require(bool condition, const char* message)
{
    if (!condition)
        throw JointArraysFailure(OVPHYSX_API_INVALID_ARGUMENT, message);
}

void checkDictionary(path_dictionary_instance_t* dictionary, ovx_api_result_t result)
{
    if (result.status == OVX_API_SUCCESS)
        return;
    const std::string message(result.error.ptr ? result.error.ptr : "", result.error.length);
    path_dictionary_release_error(dictionary, result.error);
    throw JointArraysFailure(OVPHYSX_API_ERROR,
                             message.empty() ? "The path dictionary refused a joint path list" : message);
}

// A joint group has one row per DOF and names each row's joint, so a joint is a run of equal paths in its prim
// list (ADR-0059): each run becomes one view of the group's tensor and one entry of a list naming each joint once.
std::unique_ptr<ovphysx_joint_arrays_t> regroup(ovstage_instance_t* stage,
                                                const ovstage_prim_group_t& prims,
                                                const ovstage_data_t& data)
{
    require(stage != nullptr, "A stage is required");
    path_dictionary_instance_t* const stageDictionary = ovstage_get_path_dictionary(stage);
    require(stageDictionary != nullptr, "The stage has no path dictionary");
    require(data.tensor_count == 1 && data.tensors, "A joint group carries one tensor");
    require(!prims.index_map && !data.index_map && !data.mask, "A joint group has no index map or mask");
    const DLTensor& column = data.tensors[0];
    require(column.ndim == 1 && column.shape, "A joint group's tensor is one-dimensional");
    require(!column.strides || column.shape[0] <= 1 || column.strides[0] == 1, "A joint group's tensor is contiguous");
    const int64_t rows = column.shape[0];
    require(rows >= 0 && static_cast<uint64_t>(rows) == prims.count, "A joint group has one prim-list entry per row");

    std::vector<ovx_primpath_t> rowPaths(prims.count);
    size_t copied = 0;
    checkDictionary(stageDictionary, path_dictionary_get_paths_from_path_list(stageDictionary, prims.list, prims.offset,
                                                                              prims.count, rowPaths.data(), &copied));
    require(copied == prims.count, "The joint group's prim list is shorter than its rows");

    std::vector<ovx_primpath_t> joints;
    std::vector<int64_t> starts;
    for (size_t row = 0; row < rowPaths.size(); ++row)
    {
        if (row == 0 || rowPaths[row] != rowPaths[row - 1])
        {
            joints.push_back(rowPaths[row]);
            starts.push_back(static_cast<int64_t>(row));
        }
    }

    std::unique_ptr<ovphysx_joint_arrays_t> arrays = std::make_unique<ovphysx_joint_arrays_t>();
    if (!joints.empty())
    {
        checkDictionary(stageDictionary, path_dictionary_create_path_list_from_paths(
                                             stageDictionary, joints.data(), joints.size(), &arrays->list));
        arrays->dictionary = stageDictionary;
    }

    const uint64_t rowBytes = (uint64_t(column.dtype.bits) * column.dtype.lanes + 7) / 8;
    arrays->shapes.resize(joints.size());
    arrays->tensors.assign(joints.size(), column);
    for (size_t joint = 0; joint < joints.size(); ++joint)
    {
        const int64_t end = joint + 1 < joints.size() ? starts[joint + 1] : rows;
        arrays->shapes[joint] = end - starts[joint];
        DLTensor& view = arrays->tensors[joint];
        view.shape = &arrays->shapes[joint];
        view.strides = nullptr;
        view.byte_offset = column.byte_offset + static_cast<uint64_t>(starts[joint]) * rowBytes;
    }
    arrays->cudaSync = data.cuda_sync;
    return arrays;
}

// Runs `build` and hands its object to `out_arrays`, turning a failure into the thread's last error.
template <typename Build>
ovphysx_result_t createJointArrays(ovphysx_joint_arrays_t** out_arrays, Build&& build) noexcept
{
    if (!out_arrays)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_arrays is NULL");
    *out_arrays = nullptr;
    try
    {
        *out_arrays = build().release();
        return success();
    }
    catch (const JointArraysFailure& error)
    {
        return set_error(error.status, error.what());
    }
    catch (const std::bad_alloc&)
    {
        return set_error(OVPHYSX_API_ERROR, "Out of memory regrouping a joint group");
    }
    catch (const std::exception& error)
    {
        return set_error(OVPHYSX_API_ERROR, error.what());
    }
}
} // namespace

OVPHYSX_API ovphysx_result_t ovphysx_create_joint_arrays_from_read_group(ovstage_instance_t* stage,
                                                                         const ovstage_read_group_t* group,
                                                                         ovphysx_joint_arrays_t** out_arrays)
{
    OVPHYSX_API_PROFILE_ZONE(create_joint_arrays_from_read_group);
    return createJointArrays(out_arrays,
                             [&]
                             {
                                 require(group != nullptr, "group is NULL");
                                 require(group->column_count == 1 && group->columns,
                                         "A joint read group carries one column");
                                 const ovstage_read_column_t& column = group->columns[0];
                                 require(!column.is_delete, "A deleted group has no joint values");
                                 std::unique_ptr<ovphysx_joint_arrays_t> arrays =
                                     regroup(stage, group->prims, column.data);
                                 arrays->semantic = column.semantic;
                                 return arrays;
                             });
}

OVPHYSX_API ovphysx_result_t ovphysx_create_joint_arrays_from_map_group(ovstage_instance_t* stage,
                                                                        const ovstage_map_group_t* group,
                                                                        ovphysx_joint_arrays_t** out_arrays)
{
    OVPHYSX_API_PROFILE_ZONE(create_joint_arrays_from_map_group);
    return createJointArrays(out_arrays,
                             [&]
                             {
                                 require(group != nullptr, "group is NULL");
                                 return regroup(stage, group->prims, group->data);
                             });
}

OVPHYSX_API ovx_primpath_list_t ovphysx_get_joint_arrays_prim_list(const ovphysx_joint_arrays_t* arrays)
{
    OVPHYSX_API_PROFILE_ZONE(get_joint_arrays_prim_list);
    return arrays ? arrays->list : OVX_INVALID_PRIMPATH_LIST;
}

OVPHYSX_API ovstage_write_data_t ovphysx_get_joint_arrays_write_data(const ovphysx_joint_arrays_t* arrays)
{
    OVPHYSX_API_PROFILE_ZONE(get_joint_arrays_write_data);
    ovstage_write_data_t data{};
    if (!arrays)
        return data;
    data.tensors = arrays->tensors.empty() ? nullptr : arrays->tensors.data();
    data.tensor_count = static_cast<uint32_t>(arrays->tensors.size());
    data.cuda_sync = arrays->cudaSync;
    data.semantic = arrays->semantic;
    data.is_array = true;
    return data;
}

OVPHYSX_API void ovphysx_destroy_joint_arrays(ovphysx_joint_arrays_t* arrays)
{
    OVPHYSX_API_PROFILE_ZONE(destroy_joint_arrays);
    delete arrays;
}
