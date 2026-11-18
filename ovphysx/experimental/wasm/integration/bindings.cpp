// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>
#include <ovphysx/ovphysx_legacy_config.h>
#include <ovphysx/experimental/OvStageOutput.hpp>
#include <emscripten/emscripten.h>
#include "bridge.h"
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_query_paths.h>
#include <ovx/path_dictionary/path_dictionary_utils.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/transform.h>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#ifndef OVP_WASM_SOLVER_THREADS
#define OVP_WASM_SOLVER_THREADS 0
#endif
static_assert(OVP_WASM_SOLVER_THREADS == 0 || OVP_WASM_SOLVER_THREADS == 2 || OVP_WASM_SOLVER_THREADS == 4,
              "The experimental solver supports 0, 2 or 4 worker threads");

extern "C" ovstage_instance_t* ovphysx_wasm_borrow_stage(uint32_t handle);

namespace
{
ovphysx_instance_handle_t instance = 0;
bool initialized = false;
bool attached = false;
uint32_t solverWorkers = OVP_WASM_SOLVER_THREADS;
std::string errorText;
std::vector<float> values;
std::unique_ptr<ovphysx::utils::OvStageOutputCache> outputCache;

int failure(const char* message)
{
    errorText = message;
    return static_cast<int>(OVPHYSX_API_ERROR);
}

int check(ovphysx_result_t result)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t error = ovphysx_get_last_error();
        errorText.assign(error.ptr ? error.ptr : "", error.ptr ? error.length : 0);
        if (errorText.empty())
            errorText = "Official OvPhysX call failed without a diagnostic";
    }
    return static_cast<int>(result.status);
}

int complete(ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS)
        return check({ task.status });
    ovphysx_wait_task_result_t waited{};
    const ovphysx_result_t result = ovphysx_wait_task(instance, task, OVPHYSX_TIMEOUT_INFINITE, &waited);
    int status = static_cast<int>(result.status);
    if (waited.num_errors)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(instance, waited.error_tasks[0]);
        errorText.assign(error.ptr ? error.ptr : "", error.ptr ? error.length : 0);
        if (!status)
            status = static_cast<int>(OVPHYSX_API_ERROR);
    }
    else if (status)
        check(result);
    ovphysx_destroy_wait_task_result(&waited);
    return status;
}

void resetControls();
int initializeControls(uint32_t stageHandle, int64_t ordinal);
void advanceControlsMetadata(int64_t ordinal);

int closePhysics()
{
    resetControls();
    // The cache is bound to one attachment. Release its buffers before detach,
    // including failed cleanup attempts, so a later attachment starts fresh.
    outputCache.reset();
    if (instance)
    {
        const int status = check(ovphysx_is_stage_attached(instance, nullptr, &attached));
        if (status)
            return status;
    }
    if (instance && attached)
    {
        const int status = complete(ovphysx_detach_ovstage(instance));
        if (status)
            return status;
        attached = false;
    }
    if (instance)
    {
        const int status = check(ovphysx_destroy_instance(instance));
        if (status)
            return status;
        instance = 0;
    }
    if (initialized)
    {
        const int status = check(ovphysx_shutdown());
        if (status)
            return status;
        initialized = false;
    }
    values.clear();
    return 0;
}

int copyVector(const ovstage_read_group_t& group, uint16_t lanes)
{
    if (group.column_count != 1)
        return failure("Expected one requested physics vector column");
    const ovstage_read_column_t& column = group.columns[0];
    if (column.is_delete || column.is_array || column.data.tensor_count != 1 || !column.data.tensors)
        return failure("Physics vector is not a fixed-width tensor");
    const DLTensor& tensor = column.data.tensors[0];
    if (tensor.device.device_type != kDLCPU || tensor.dtype.code != kDLFloat || tensor.dtype.bits != 32 ||
        tensor.dtype.lanes != lanes || tensor.ndim != 1 || !tensor.shape || tensor.shape[0] < 0 ||
        (!tensor.data && tensor.shape[0]))
        return failure("Unexpected CPU float32 physics output width");
    const int64_t stride = tensor.strides ? tensor.strides[0] : 1;
    if (stride < 0 || static_cast<uint64_t>(tensor.shape[0]) > std::numeric_limits<uint32_t>::max() / lanes)
        return failure("Physics output shape or stride is outside the JavaScript binding limits");
    const size_t vectorBytes = lanes * sizeof(float);
    if (tensor.shape[0] > 1 && static_cast<uint64_t>(stride) >
        (std::numeric_limits<size_t>::max() / vectorBytes) / static_cast<uint64_t>(tensor.shape[0] - 1))
        return failure("Physics output strides exceed addressable memory");
    const unsigned char* bytes = static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset;
    for (int64_t row = 0; row < tensor.shape[0]; ++row)
    {
        float vector[3];
        std::memcpy(vector, bytes + static_cast<size_t>(row) * static_cast<size_t>(stride) * vectorBytes, vectorBytes);
        values.insert(values.end(), vector, vector + lanes);
    }
    return 0;
}
} // namespace

// Experimental JavaScript marshalling for the original OvPhysX C API.
extern "C" EMSCRIPTEN_KEEPALIVE const char* ovp_wasm_error()
{
    return errorText.c_str();
}

extern "C" EMSCRIPTEN_KEEPALIVE uint32_t ovp_wasm_solver_threads()
{
    return solverWorkers;
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_configure_solver_threads(uint32_t workers)
{
    errorText.clear();
    if (instance || initialized || attached || outputCache)
        return failure("Solver workers can only be configured before physics initialization");
    if (workers != 0 && workers != 2 && workers != 4)
        return failure("Solver workers must be 0, 2 or 4");
    solverWorkers = workers;
    return 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_create(uint32_t stageHandle, int64_t ordinal)
{
    errorText.clear();
    if (instance || initialized)
        return failure("Physics already exists in this runtime");
    ovstage_instance_t* const stage = ovphysx_wasm_borrow_stage(stageHandle);
    if (!stage)
        return failure("Invalid borrowed OVStage handle");
    int status = check(ovphysx_initialize());
    if (status)
        return status;
    initialized = true;
    status = check(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, ovphysx_config_value_bool(true)));
    if (!status)
        status = check(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, ovphysx_config_value_bool(false)));
    if (!status)
        status = check(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(solverWorkers)));
    if (!status)
        status = check(ovphysx_set_global_config_entry(ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/physxDispatcher"),
            solverWorkers ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false"))));
    if (!status)
        status = check(ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE,
                                               ovphysx_config_value_int64(OVPHYSX_COMPILED_STAGE_CACHE_DISABLED)));
    if (!status)
        status = check(ovphysx_create_instance(&instance));
    if (!status)
    {
        status = complete(ovphysx_attach_ovstage(instance, stage, ordinal));
        const std::string attachError = errorText;
        const int ownershipStatus = check(ovphysx_is_stage_attached(instance, stage, &attached));
        if (status)
            errorText = attachError;
        else if (ownershipStatus)
            status = ownershipStatus;
        else if (!attached)
            status = failure("OvPhysX completed attach without retaining the requested stage");
    }
    if (!status)
        status = initializeControls(stageHandle, ordinal);
    if (status)
    {
        const std::string originalError = errorText;
        const int cleanup = closePhysics();
        if (!cleanup)
            errorText = originalError;
        else
            errorText = originalError + "; cleanup also failed: " + errorText;
    }
    return status;
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_step(float dt)
{
    if (!instance || !attached)
        return failure("Physics has no attached stage");
    return check(ovphysx_step_sync(instance, dt));
}

namespace
{
int readValues(ovphysx_query_target_t target, const char* name, uint16_t lanes)
{
    values.clear();
    if (!instance || !attached)
        return failure("Physics has no attached stage");
    ovphysx_query_handle_t query = 0;
    ovphysx_read_handle_t read = 0;
    int status = complete(ovphysx_query(instance, target, OVPHYSX_SCOPE_ALL, &query, nullptr));
    if (!status)
    {
        ovx_token_or_string_t attribute{};
        attribute.token = OVX_INVALID_TOKEN;
        attribute.string = { name, std::strlen(name) };
        status = complete(ovphysx_read(instance, query, &attribute, 1, {}, &read, nullptr));
    }
    while (!status)
    {
        const ovstage_read_group_t* group = nullptr;
        status = complete(ovphysx_fetch_read_next(instance, read, &group));
        if (status || !group)
            break;
        status = copyVector(*group, lanes);
        const std::string savedError = errorText;
        const int release = complete(ovphysx_release_group(instance, read, group->read_group_id));
        if (status)
            errorText = savedError;
        else
            status = release;
    }
    const std::string savedError = errorText;
    const int releaseRead = read ? complete(ovphysx_release_read(instance, read)) : 0;
    const std::string readReleaseError = errorText;
    const int releaseQuery = query ? complete(ovphysx_release_query(instance, query)) : 0;
    if (status)
        errorText = savedError;
    else
    {
        status = releaseRead ? releaseRead : releaseQuery;
        if (releaseRead)
            errorText = readReleaseError;
    }
    return status;
}

} // namespace

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_read_vectors(int velocity)
{
    return readValues(OVPHYSX_QUERY_TARGET_RIGID_BODY,
                      velocity ? OVPHYSX_ATTR_LINEAR_VELOCITY : OVPHYSX_ATTR_POSITION, 3);
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_read_links(int velocity)
{
    return readValues(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK,
                      velocity ? OVPHYSX_ATTR_LINEAR_VELOCITY : OVPHYSX_ATTR_POSITION, 3);
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_read_joints(int velocity)
{
    return readValues(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT,
                      velocity ? OVPHYSX_ATTR_JOINT_VELOCITY : OVPHYSX_ATTR_JOINT_POSITION, 1);
}

extern "C" EMSCRIPTEN_KEEPALIVE const float* ovp_wasm_values()
{
    return values.data();
}

extern "C" EMSCRIPTEN_KEEPALIVE uint32_t ovp_wasm_value_count()
{
    return static_cast<uint32_t>(values.size());
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_destroy()
{
    return closePhysics();
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_publish(uint32_t stageHandle, int64_t outputOrdinal)
{
    if (!instance || !attached)
        return failure("Physics has no attached stage");
    ovstage_instance_t* const stage = ovphysx_wasm_borrow_stage(stageHandle);
    if (!stage)
        return failure("Invalid borrowed OVStage handle");
    if (!outputCache)
    {
        try
        {
            outputCache.reset(new ovphysx::utils::OvStageOutputCache);
        }
        catch (const std::exception& error)
        {
            return failure(error.what());
        }
    }
    const ovphysx::utils::OvStageOutputResult result =
        ovphysx::utils::writeWorldTransformsToOvstage(instance, stage, outputOrdinal, outputCache.get());
    if (!result.ok())
    {
        errorText = result.message;
        return static_cast<int>(result.status);
    }
    advanceControlsMetadata(outputOrdinal);
    return 0;
}

#include "controls.inl"
