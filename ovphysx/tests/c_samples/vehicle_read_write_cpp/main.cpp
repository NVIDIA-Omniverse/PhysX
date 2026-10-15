// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-VEHICLE-001
// @covers AC-1 AC-2 AC-4

#include "ovstage_sample.h"
#include <ovphysx/ovphysx_attributes.h>

#include <cstddef>
#include <cstdio>
#include <stdexcept>

namespace
{
void awaitTask(ovphysx_instance_handle_t instance, ovphysx_enqueue_result_t task)
{
    // The shared helper reports admission/task errors while their text is alive,
    // collects this receipt exactly once, and destroys its wait-result storage.
    if (ovphysx_sample_complete_task(instance, task).status != OVPHYSX_API_SUCCESS)
        throw std::runtime_error("Vehicle sample operation failed");
}

template <std::size_t N>
ovx_string_or_token_t attribute(const char (&name)[N])
{
    return { OVX_INVALID_TOKEN, { name, N - 1 } };
}

struct Sessions
{
    ovphysx_instance_handle_t instance;
    ovphysx_query_handle_t query = 0;
    ovphysx_read_handle_t read = 0;
    ovphysx_write_handle_t write = 0;

    template <class Release>
    void release(uint64_t& handle, Release function)
    {
        if (!handle)
            return;
        const ovphysx_enqueue_result_t task = function(instance, handle);
        // Accepted release transfers ownership even if its execution later fails.
        if (task.status == OVPHYSX_API_SUCCESS)
            handle = 0;
        awaitTask(instance, task);
    }

    void close()
    {
        release(read, ovphysx_release_read);
        release(write, ovphysx_release_write);
        release(query, ovphysx_release_query);
    }

    ~Sessions()
    {
        // Try every remaining cleanup, including when an earlier release failed.
        const auto cleanup = [&](uint64_t& handle, auto function) {
            try { release(handle, function); }
            catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); }
        };
        cleanup(read, ovphysx_release_read);
        cleanup(write, ovphysx_release_write);
        cleanup(query, ovphysx_release_query);
    }
};

const DLTensor& cpuFloats(const ovstage_data_t& data)
{
    if (data.tensor_count != 1 || !data.tensors || data.index_map)
        throw std::runtime_error("Expected one dense column");
    const DLTensor& tensor = data.tensors[0];
    if (tensor.device.device_type != kDLCPU || tensor.dtype.code != kDLFloat ||
        tensor.dtype.bits != 32 || tensor.ndim != 1 || !tensor.shape ||
        tensor.shape[0] < 0 || (tensor.strides && tensor.strides[0] != 1) ||
        (tensor.shape[0] && !tensor.data))
        throw std::runtime_error("Expected contiguous CPU float32 values");
    return tensor;
}

template <std::size_t N>
void printColumn(Sessions& sessions, const char (&name)[N])
{
    const ovx_string_or_token_t attr = attribute(name);
    awaitTask(sessions.instance, ovphysx_read(sessions.instance, sessions.query, &attr, 1,
                                             &sessions.read, nullptr));
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        awaitTask(sessions.instance, ovphysx_fetch_read_next(sessions.instance, sessions.read, &group));
        if (!group)
            break;
        const DLTensor& tensor = cpuFloats(group->data);
        const unsigned char* bytes = static_cast<const unsigned char*>(tensor.data);
        const float* values = bytes ? reinterpret_cast<const float*>(bytes + tensor.byte_offset) : nullptr;
        for (int64_t row = 0; row < tensor.shape[0]; ++row)
        {
            std::printf("%s row %lld:", name, static_cast<long long>(row));
            for (uint16_t lane = 0; lane < tensor.dtype.lanes; ++lane)
                std::printf(" %.3f", static_cast<double>(values[row * tensor.dtype.lanes + lane]));
            std::printf("\n");
        }
    }
    sessions.release(sessions.read, ovphysx_release_read);
}

void driveOneCar(ovphysx_instance_handle_t instance)
{
    Sessions chassis{ instance }, vehicle{ instance }, wheels{ instance };
    ovphysx_query_selection_t carSelection{};
    carSelection.pattern = OVPHYSX_LITERAL("/World/Standard");
    ovphysx_query_selection_t wheelSelection{};
    wheelSelection.pattern = OVPHYSX_LITERAL("/World/Standard/**");

    ovstage_query_result_t summary{};
    // Chassis pose/velocity and whole-vehicle controls use the same car prim.
    awaitTask(instance, ovphysx_query_selection(instance, OVPHYSX_QUERY_TARGET_RIGID_BODY,
                                                &carSelection, &chassis.query, &summary));
    if (summary.total_prim_count != 1)
        throw std::runtime_error("Expected one chassis");
    awaitTask(instance, ovphysx_query_selection(instance, OVPHYSX_QUERY_TARGET_VEHICLE,
                                                &carSelection, &vehicle.query, &summary));
    if (summary.total_prim_count != 1)
        throw std::runtime_error("Expected one vehicle");
    // This independent query selects that car's wheel attachment prims.
    awaitTask(instance, ovphysx_query_selection(instance, OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL,
                                                &wheelSelection, &wheels.query, &summary));
    if (summary.total_prim_count != 4)
        throw std::runtime_error("Expected four wheels");

    const ovx_string_or_token_t accelerator = attribute(OVPHYSX_ATTR_ACCELERATOR);
    awaitTask(instance, ovphysx_write(instance, vehicle.query, &accelerator, 1,
                                      &vehicle.write, nullptr));
    for (;;)
    {
        ovphysx_fetched_write_group_t fetched{};
        awaitTask(instance, ovphysx_fetch_write_next(instance, vehicle.write, &fetched));
        if (!fetched.group)
            break;
        const DLTensor& tensor = cpuFloats(fetched.group->data);
        if (tensor.dtype.lanes != 1)
            throw std::runtime_error("Expected scalar accelerator values");
        unsigned char* bytes = static_cast<unsigned char*>(tensor.data);
        float* values = bytes ? reinterpret_cast<float*>(bytes + tensor.byte_offset) : nullptr;
        for (int64_t row = 0; row < tensor.shape[0]; ++row)
            values[row] = 0.5f;
        // Host fill is finished; no CUDA producer dependency is outstanding.
        awaitTask(instance, ovphysx_commit_group(instance, vehicle.write, fetched.group, {}));
    }
    vehicle.release(vehicle.write, ovphysx_release_write);
    awaitTask(instance, ovphysx_step(instance, 1.0f / 60.0f));

    printColumn(chassis, OVPHYSX_ATTR_POSITION);
    printColumn(vehicle, OVPHYSX_ATTR_ENGINE_ROTATION_SPEED);
    printColumn(wheels, OVPHYSX_ATTR_ROTATION_SPEED);
    wheels.close();
    vehicle.close();
    chassis.close();
}
} // namespace

int main()
{
    ovphysx_instance_handle_t instance = 0;
    ovphysx_sample_stage_attachment_t attachment{};
    ovphysx_config_value_t cpuMode{};
    cpuMode.type = OVPHYSX_CONFIG_VALUE_BOOL;
    cpuMode.value.bool_value = true;
    if (ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, cpuMode).status != OVPHYSX_API_SUCCESS)
        return 1;
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
        return 1;
    int result = 1;
    try
    {
        if (ovphysx_create_instance(&instance).status != OVPHYSX_API_SUCCESS)
            throw std::runtime_error("Failed to create instance");
        if (!ovphysx_sample_attach_usd_with_ovstage(
                instance, OVPHYSX_TEST_DATA "/vehicle_read_write.usda", &attachment))
            throw std::runtime_error("Failed to attach vehicle scene");
        driveOneCar(instance);
        result = 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "%s\n", error.what());
    }
    if (instance)
    {
        if (!ovphysx_sample_destroy_stage(instance, &attachment))
            result = 1;
        if (ovphysx_destroy_instance(instance).status != OVPHYSX_API_SUCCESS)
            result = 1;
        else if (attachment.stage)
            ovstage_destroy_instance(attachment.stage);
    }
    if (ovphysx_shutdown().status != OVPHYSX_API_SUCCESS)
        result = 1;
    return result;
}
