// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-4
 */

// @implements REQ-CAPI-OVSTAGE-XFORM-001
// @covers AC-1

#pragma once

#include "ovphysx/ovphysx.h"
#include "../c_samples/common/ovstage_sample.h"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/experimental/Helpers.hpp"
#include "ovphysx/ovphysx_types.h"
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <ovstage/ovx_path_dictionary.h>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace test_utils {

inline std::string& last_task_error()
{
    static thread_local std::string error;
    return error;
}

// Complete only this receipt; keep its diagnostic after the wait storage is released.
inline ovphysx_result_t complete_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    std::string& message = last_task_error();
    message.clear();
    if (task.status != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t error = ovphysx_get_last_error();
        message.assign(error.ptr ? error.ptr : "", error.length);
        return { task.status };
    }
    ovphysx::physx::WaitResult result;
    const ovphysx_result_t status{ result.waitTask(handle, task) };
    for (size_t index = 0; index < result.errorCount(); ++index)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, result.errorTaskAt(index));
        message.append(error.ptr ? error.ptr : "", error.length);
    }
    if (status.status != OVPHYSX_API_SUCCESS && message.empty())
    {
        const ovphysx_string_t error = ovphysx_get_last_error();
        message.assign(error.ptr ? error.ptr : "", error.length);
    }
    return status;
}

// Helper to build ovphysx_string_t without hand-maintaining lengths.
// Preserves {nullptr, 0} for a null input. APIs validate that representation
// according to their own selector contracts.
inline ovphysx_string_t make_ovx_string(const char* s)
{
    if (!s)
        return {nullptr, 0};
    return ovphysx_cstr(s);
}

// Build ovphysx_string_t from an arbitrary byte sequence (may include embedded NUL).
// `storage` must outlive the returned view.
inline ovphysx_string_t make_ovx_string_bytes(const std::string& bytes, std::string& storage)
{
    storage = bytes;
    return {storage.data(), storage.size()};
}

struct OvstageTestAttachment
{
    ovstage_instance_t* stage = nullptr;
    uint64_t ordinal = 1;
};

inline std::mutex& ovstage_test_attachments_mutex()
{
    static std::mutex m;
    return m;
}

inline std::map<ovphysx_instance_handle_t, std::vector<OvstageTestAttachment>>& ovstage_test_attachments()
{
    static std::map<ovphysx_instance_handle_t, std::vector<OvstageTestAttachment>> attachments;
    return attachments;
}

inline std::string ovx_to_string(ovx_string_t s)
{
    if (!s.ptr || s.length == 0) return {};
    return std::string(s.ptr, s.length);
}

// The path dictionary of the Stage most recently attached to `handle` through
// attach_usd_with_ovstage, or nullptr when there is none. Query prim lists and
// tokens belong to that owning Stage; ovphysx exposes no dictionary of its own.
// Valid only while that Stage is attached and alive.
inline path_dictionary_instance_t* attached_ovstage_dictionary(ovphysx_instance_handle_t handle)
{
    std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
    auto it = ovstage_test_attachments().find(handle);
    if (it == ovstage_test_attachments().end() || it->second.empty() || !it->second.back().stage)
        return nullptr;
    ovstage_instance_t* stage = it->second.back().stage;
    return stage->vtable->get_path_dictionary(stage->context);
}

inline void destroy_ovstage_test_stages(ovphysx_instance_handle_t handle)
{
    std::vector<OvstageTestAttachment> attachments;
    {
        std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
        auto it = ovstage_test_attachments().find(handle);
        if (it != ovstage_test_attachments().end())
        {
            attachments.swap(it->second);
            ovstage_test_attachments().erase(it);
        }
    }

    for (OvstageTestAttachment& attachment : attachments)
    {
        if (attachment.stage)
            ovstage_destroy_instance(attachment.stage);
    }
}

inline bool destroy_ovstage_test_attachments(ovphysx_instance_handle_t handle)
{
    if (handle == 0) return true;
    const ovphysx_result_t detach = complete_task(handle, ovphysx_detach_ovstage(handle));
    if (detach.status != OVPHYSX_API_SUCCESS)
    {
        std::cerr << "ovphysx_detach_ovstage failed; retaining caller-owned Stage: "
                  << static_cast<int>(detach.status) << " " << last_task_error() << std::endl;
        return false;
    }

    destroy_ovstage_test_stages(handle);
    return true;
}

// Register external schemas before the first population.
inline bool register_physx_schemas_with_ovstage()
{
    return ovphysx_sample_register_physx_schemas() != 0;
}

inline bool attach_usd_with_ovstage(
    ovphysx_instance_handle_t handle,
    const char* usd_path,
    uint64_t ordinal = 1,
    const char* stage_name = "ovphysx-test-stage")
{
    if (handle == 0 || !usd_path)
    {
        return false;
    }

    if (!register_physx_schemas_with_ovstage())
    {
        return false;
    }

    ovstage_instance_desc_t desc{};
    desc.name = stage_name ? stage_name : "ovphysx-test-stage";

    ovstage_instance_t* stage = nullptr;
    ovstage_api_status_t create_status = ovstage_create_instance(&desc, &stage);
    if (create_status != OVSTAGE_OK || !stage)
    {
        std::cerr << "ovstage_create_instance failed: " << static_cast<int>(create_status) << std::endl;
        return false;
    }

    ovx_string_t path{};
    path.ptr = usd_path;
    path.length = std::strlen(usd_path);

    ovstage_population_enqueue_result_t enqueue = ovstage_population_open_usd_from_file(
        stage,
        path,
        ordinal,
        0.0,
        OVSTAGE_POPULATION_DOMAIN_ALL);
    if (enqueue.status != OVSTAGE_OK)
    {
        ovx_string_t err = ovstage_population_get_last_error();
        std::cerr << "ovstage_population_open_usd_from_file failed: "
                  << static_cast<int>(enqueue.status) << " " << ovx_to_string(err) << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }

    ovstage_population_op_wait_result_t wait_result{};
    ovstage_api_status_t wait_status = ovstage_population_wait_op(
        stage,
        enqueue.op_index,
        OVSTAGE_TIMEOUT_INFINITE,
        &wait_result);
    if (wait_status != OVSTAGE_OK)
    {
        ovx_string_t err = ovstage_population_get_last_error();
        std::cerr << "ovstage_population_wait_op failed: "
                  << static_cast<int>(wait_status) << " " << ovx_to_string(err) << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }

    ovstage_write_floor_desc_t floor_desc{};
    floor_desc.ordinal = ordinal;
    floor_desc.scope = OVSTAGE_SCOPE_ALL;
    ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floor_desc);
    if (floor.status != OVSTAGE_OK)
    {
        std::cerr << "ovstage_advance_write_floor failed: "
                  << static_cast<int>(floor.status) << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }

    ovstage_op_wait_result_t floor_wait{};
    ovstage_api_status_t floor_wait_status =
        ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floor_wait);
    if (floor_wait_status != OVSTAGE_OK || floor_wait.error_op_id_count != 0)
    {
        std::cerr << "ovstage write-floor wait failed: "
                  << static_cast<int>(floor_wait_status) << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }
    if (ovstage_release_op(stage, floor.op_index) != OVSTAGE_OK)
    {
        std::cerr << "ovstage floor operation release failed" << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }

    const ovphysx_result_t attach_status = complete_task(handle, ovphysx_attach_ovstage(handle, stage, ordinal));
    if (attach_status.status != OVPHYSX_API_SUCCESS)
    {
        std::cerr << "ovphysx_attach_ovstage failed: "
                  << static_cast<int>(attach_status.status) << " " << last_task_error() << std::endl;
        bool attached = false;
        if (ovphysx_is_stage_attached(handle, stage, &attached).status != OVPHYSX_API_SUCCESS || attached)
        {
            std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
            ovstage_test_attachments()[handle].push_back({stage, ordinal});
        }
        else
        {
            ovstage_destroy_instance(stage);
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(ovstage_test_attachments_mutex());
        ovstage_test_attachments()[handle].push_back({stage, ordinal});
    }

    return true;
}

// Allocates a DLTensor for an array of float32 scalars.
inline DLTensor* make_dl_float32_array(const std::vector<float>& values)
{
    DLTensor* t = new DLTensor();
    float* data = new float[values.size()];
    std::memcpy(data, values.data(), values.size() * sizeof(float));
    
    t->data = data;
    t->ndim = 1;
    int64_t* shape = new int64_t[1];
    shape[0] = values.size();
    t->shape = shape;
    t->strides = nullptr;
    t->byte_offset = 0;
    t->dtype.code = kDLFloat;
    t->dtype.bits = 32;
    t->dtype.lanes = 1;
    t->device.device_type = kDLCPU;
    t->device.device_id = 0;
    return t;
}

// Allocates a DLTensor representing a single float32 scalar on CPU.
inline DLTensor* make_dl_float32_1(const float value)
{
    DLTensor* t = new DLTensor();
    float* data = new float[1];
    data[0] = value;
    t->data = data;
    t->ndim = 1;
    int64_t* shape = new int64_t[1];
    shape[0] = 1;
    t->shape = shape;
    t->strides = nullptr;
    t->byte_offset = 0;
    t->dtype.code = kDLFloat;
    t->dtype.bits = 32;
    t->dtype.lanes = 1;
    t->device.device_type = kDLCPU;
    t->device.device_id = 0;
    return t;
}

inline std::vector<float> createTestData(size_t count) {
    std::vector<float> data(count);
    for (size_t i = 0; i < count; ++i) {
        data[i] = static_cast<float>(i) * 0.1f;
    }
    return data;
}

inline DLTensor* make_float32_tensor(const std::vector<float>& values, const std::vector<int64_t>& shape)
{
    DLTensor* t = new DLTensor();
    float* data = new float[values.size()];
    std::memcpy(data, values.data(), values.size() * sizeof(float));
    
    t->data = data;
    t->ndim = static_cast<int32_t>(shape.size());
    t->shape = new int64_t[shape.size()];
    std::memcpy(t->shape, shape.data(), shape.size() * sizeof(int64_t));
    t->strides = nullptr;
    t->byte_offset = 0;
    t->dtype.code = kDLFloat;
    t->dtype.bits = 32;
    t->dtype.lanes = 1;
    t->device.device_type = kDLCPU;
    t->device.device_id = 0;
    return t;
}

inline DLTensor* make_float64_tensor(const std::vector<double>& values, const std::vector<int64_t>& shape)
{
    DLTensor* t = new DLTensor();
    double* data = new double[values.size()];
    std::memcpy(data, values.data(), values.size() * sizeof(double));
    
    t->data = data;
    t->ndim = static_cast<int32_t>(shape.size());
    t->shape = new int64_t[shape.size()];
    std::memcpy(t->shape, shape.data(), shape.size() * sizeof(int64_t));
    t->strides = nullptr;
    t->byte_offset = 0;
    t->dtype.code = kDLFloat;
    t->dtype.bits = 64;
    t->dtype.lanes = 1;
    t->device.device_type = kDLCPU;
    t->device.device_id = 0;
    return t;
}

inline DLTensor* make_int32_tensor(const std::vector<int32_t>& values, const std::vector<int64_t>& shape)
{
    DLTensor* t = new DLTensor();
    int32_t* data = new int32_t[values.size()];
    std::memcpy(data, values.data(), values.size() * sizeof(int32_t));
    
    t->data = data;
    t->ndim = static_cast<int32_t>(shape.size());
    t->shape = new int64_t[shape.size()];
    std::memcpy(t->shape, shape.data(), shape.size() * sizeof(int64_t));
    t->strides = nullptr;
    t->byte_offset = 0;
    t->dtype.code = kDLInt;
    t->dtype.bits = 32;
    t->dtype.lanes = 1;
    t->device.device_type = kDLCPU;
    t->device.device_id = 0;
    return t;
}

// Frees a tensor allocated by the make_*_tensor helpers above.
inline void free_tensor(DLTensor* t)
{
    if (!t) return;
    if (t->data) {
        if (t->dtype.code == kDLFloat && t->dtype.bits == 64) {
            delete[] static_cast<double*>(t->data);
        } else if (t->dtype.code == kDLFloat && t->dtype.bits == 32) {
            delete[] static_cast<float*>(t->data);
        } else if (t->dtype.code == kDLInt) {
            delete[] static_cast<int32_t*>(t->data);
        }
    }
    delete[] t->shape;
    delete t;
}

// Read the borrowed physxPointer column. Keys are prim paths. The PxPhysics
// singleton is the "/" entry. An empty map means the query matched nothing
// or the read failed; callers that need a failure reason should check last_task_error().
inline std::map<std::string, void*> read_physx_pointers(ovphysx_instance_handle_t handle,
                                                        ovphysx_query_target_t type)
{
    std::map<std::string, void*> out;
    ovphysx_query_handle_t query = 0;
    if (complete_task(handle, ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)).status != OVPHYSX_API_SUCCESS)
        return out;
    ovx_path_dictionary_t* dict = attached_ovstage_dictionary(handle);
    if (!dict)
    {
        complete_task(handle, ovphysx_release_query(handle, query));
        return out;
    }
    ovx_string_or_token_t name{};
    name.string.ptr = OVPHYSX_ATTR_PHYSX_POINTER;
    name.string.length = std::strlen(OVPHYSX_ATTR_PHYSX_POINTER);
    ovphysx_read_handle_t read = 0;
    if (complete_task(handle, ovphysx_read(handle, query, &name, 1, &read, nullptr)).status != OVPHYSX_API_SUCCESS)
    {
        complete_task(handle, ovphysx_release_query(handle, query));
        return out;
    }
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t fetched = complete_task(handle, ovphysx_fetch_read_next(handle, read, &group));
        if (fetched.status != OVPHYSX_API_SUCCESS || !group || !group->data.tensors)
            break;
        const DLTensor& tensor = group->data.tensors[0];
        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (tensor.data && tensor.dtype.code == kDLUInt && tensor.dtype.bits == 64 &&
            tensor.device.device_type == kDLCPU &&
            ovx_path_dictionary_get_paths(dict, group->prims.list, &paths, &pathCount) == OVX_OK)
        {
            const uint64_t* values = static_cast<const uint64_t*>(tensor.data);
            for (uint32_t row = 0; row < group->prims.count; ++row)
            {
                const size_t index = group->prims.index_map ? group->prims.index_map[row] : group->prims.offset + row;
                if (index >= pathCount || values[row] == 0)
                    continue;
                ovx_string_t text{};
                if (ovx_path_dictionary_path_to_string(dict, paths[index], &text) != OVX_OK || !text.ptr)
                    continue;
                out[std::string(text.ptr, text.length)] = reinterpret_cast<void*>(values[row]);
            }
        }
        complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id));
    }
    complete_task(handle, ovphysx_release_read(handle, read));
    complete_task(handle, ovphysx_release_query(handle, query));
    return out;
}

inline void* read_physx_pointer(ovphysx_instance_handle_t handle,
                                ovphysx_query_target_t type,
                                const char* path)
{
    const std::map<std::string, void*> pointers = read_physx_pointers(handle, type);
    const auto found = pointers.find(path ? path : "");
    return found == pointers.end() ? nullptr : found->second;
}

inline void* read_physics_singleton(ovphysx_instance_handle_t handle)
{
    return read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_PHYSICS, "/");
}

} // namespace test_utils
