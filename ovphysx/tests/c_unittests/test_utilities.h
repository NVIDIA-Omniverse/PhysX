// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "AsyncEventManager/AsyncEventManager.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace test_utils {

// Blocks on an async event until it completes or fails, with a timeout.
inline bool poll_event_blocking(async_event_handle_t evt, int timeout_ms = 10000)
{
    const auto start = std::chrono::steady_clock::now();
    while (true)
    {
        async_status_t s = async_poll_event(evt);
        if (s == ASYNC_STATUS_COMPLETED) return true;
        if (s == ASYNC_STATUS_FAILED) return false;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() > timeout_ms)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// Helper to build ovphysx_string_t without hand-maintaining lengths.
// Preserves {nullptr, 0} for a null input — ovphysx APIs validate with !ptr checks.
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

inline std::map<ovphysx_handle_t, std::vector<OvstageTestAttachment>>& ovstage_test_attachments()
{
    static std::map<ovphysx_handle_t, std::vector<OvstageTestAttachment>> attachments;
    return attachments;
}

inline std::string ovx_to_string(ovx_string_t s)
{
    if (!s.ptr || s.length == 0) return {};
    return std::string(s.ptr, s.length);
}

inline bool destroy_ovstage_test_attachments(ovphysx_handle_t handle)
{
    if (handle == 0) return true;
    const ovphysx_result_t detach = ovphysx_detach_ovstage(handle);
    if (detach.status != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t err = ovphysx_get_last_error();
        std::cerr << "ovphysx_detach_ovstage failed; retaining caller-owned Stage: "
                  << static_cast<int>(detach.status) << " "
                  << std::string(err.ptr ? err.ptr : "", err.length) << std::endl;
        return false;
    }

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
        {
            ovstage_destroy_instance(attachment.stage);
        }
    }
    return true;
}

inline bool attach_usd_with_ovstage(
    ovphysx_handle_t handle,
    const char* usd_path,
    uint64_t ordinal = 1)
{
    if (handle == 0 || !usd_path)
    {
        return false;
    }

    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-test-stage";

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
        OVSTAGE_POPULATION_DOMAIN_PHYSICS);
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

    // The population API never opens or commits an ordinal of its own: the caller
    // owns ordinal lifecycle and hands population the current ordinal. Waiting on
    // the population op only completes population, so seal what it authored before
    // reading it back -- ovphysx_attach_ovstage() reads at a *sealed* ordinal.
    // Canonical sequence (ovstage_population.h): open_usd_* -> wait_op ->
    // advance_write_floor -> consumer attach/update.
    ovstage_write_floor_desc_t write_floor{};
    write_floor.ordinal = ordinal;
    write_floor.scope = OVSTAGE_SCOPE_ALL;

    ovstage_enqueue_result_t floor_enqueue = ovstage_advance_write_floor(stage, &write_floor);
    if (floor_enqueue.status != OVSTAGE_OK)
    {
        ovx_string_t err = ovstage_get_last_error();
        std::cerr << "ovstage_advance_write_floor failed: "
                  << static_cast<int>(floor_enqueue.status) << " " << ovx_to_string(err) << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }

    ovstage_op_wait_result_t floor_wait{};
    ovstage_api_status_t floor_status = ovstage_wait_op(
        stage,
        floor_enqueue.op_index,
        OVSTAGE_TIMEOUT_INFINITE,
        &floor_wait);
    (void)ovstage_release_op(stage, floor_enqueue.op_index);
    if (floor_status != OVSTAGE_OK)
    {
        ovx_string_t err = ovstage_get_last_error();
        std::cerr << "ovstage_wait_op(advance_write_floor) failed: "
                  << static_cast<int>(floor_status) << " " << ovx_to_string(err) << std::endl;
        ovstage_destroy_instance(stage);
        return false;
    }

    ovphysx_result_t attach_status = ovphysx_attach_ovstage(handle, stage, ordinal);
    if (attach_status.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t err = ovphysx_get_last_error();
        std::cerr << "ovphysx_attach_ovstage failed: "
                  << static_cast<int>(attach_status.status) << " "
                  << std::string(err.ptr ? err.ptr : "", err.length) << std::endl;
        ovstage_destroy_instance(stage);
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

// Free tensor with support for multiple types
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

} // namespace test_utils
