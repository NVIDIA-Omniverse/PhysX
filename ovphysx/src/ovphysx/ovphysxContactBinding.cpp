// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/DLPackConvert.h"

#include <carb/logging/Log.h>

#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/IRigidContactView.h>

#include <cmath>
#include <sstream>
#include <vector>
#include <string>

using omni::physics::tensors::ContactDataReadStatus;
using ovphysx::internal::DLConvertError;
using ovphysx::internal::dlConvertErrorMessage;
using ovphysx::internal::dlToTensorDesc;
using ovphysx::internal::getTensorApi;

namespace
{

void destroyContactBindingResources(ContactBindingState& b)
{
    if (b.contactView) { b.contactView->release(); b.contactView = nullptr; }
    if (b.simView) { b.simView->release(false); b.simView = nullptr; }
}

ovphysx_result_t mapContactDataReadStatus(ContactDataReadStatus status, const char* operation, uint32_t capacity)
{
    if (status == ContactDataReadStatus::eSuccess)
        return success();
    if (status == ContactDataReadStatus::eBufferTooSmall)
    {
        std::ostringstream message;
        message << operation << ": max_contact_data_count " << capacity
                << " is too small; count and start-index tensors contain the required layout. "
                   "Use the maximum element of start + count as the capacity of a recreated binding for subsequent "
                   "simulation steps.";
        return set_error(OVPHYSX_API_BUFFER_TOO_SMALL, message.str());
    }

    std::ostringstream message;
    message << operation << " failed";
    return set_error(OVPHYSX_API_ERROR, message.str());
}


ovphysx_result_t validateContactDstTensorDtype(const DLTensor* tensor, const char* op)
{
    if (!tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor is NULL");

    if (tensor->dtype.code != kDLFloat || tensor->dtype.bits != 32 || tensor->dtype.lanes != 1)
    {
        std::ostringstream oss;
        oss << op << ": expected float32 tensor";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    return success();
}

ovphysx_result_t validateContactCountTensorDtype(const DLTensor* tensor, const char* op, const char* name)
{
    if (!tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor is NULL");

    const bool is32BitInteger =
        (tensor->dtype.code == kDLInt || tensor->dtype.code == kDLUInt) &&
        tensor->dtype.bits == 32 &&
        tensor->dtype.lanes == 1;
    if (!is32BitInteger)
    {
        std::ostringstream oss;
        oss << op << ": expected int32 or uint32 tensor for " << name;
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    return success();
}

ovphysx_result_t validateContactDeviceMatch(const DLTensor* tensor, const ContactBindingState& binding, const char* op)
{
    if (!tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor is NULL");

    int expectedDevice = -1;
    if (binding.simView)
        expectedDevice = binding.simView->getDeviceOrdinal();

    int tensorDevice;
    switch (tensor->device.device_type)
    {
        case kDLCPU:
        case kDLCUDAHost:
            tensorDevice = -1;
            break;
        case kDLCUDA:
        case kDLCUDAManaged:
            tensorDevice = tensor->device.device_id;
            break;
        default:
        {
            std::ostringstream oss;
            oss << op << ": unsupported tensor device type " << static_cast<int>(tensor->device.device_type);
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
        }
    }

    if (tensorDevice != expectedDevice)
    {
        std::ostringstream oss;
        oss << op << ": device mismatch: "
            << "binding expects " << (expectedDevice < 0 ? "CPU" : "GPU")
            << " (device=" << expectedDevice << "), "
            << "tensor is " << (tensorDevice < 0 ? "CPU" : "GPU")
            << " (device=" << tensorDevice << ")";
        if (expectedDevice < 0 && tensorDevice >= 0)
        {
            oss << ". Use a CPU tensor for this binding, or opt into DirectGPU before creating "
                   "the ovphysx instance. Note: device=\"gpu\" enables GPU dynamics only; "
                   "DirectGPU TensorAPI/ContactBinding views require /physics/suppressReadback=true. "
                   "Python: PhysXConfig(carbonite_overrides={\"/physics/suppressReadback\": True}).";
        }
        return set_error(OVPHYSX_API_DEVICE_MISMATCH, oss.str());
    }

    return success();
}

ovphysx_result_t validateContactFlatTensorShape(
    const DLTensor* tensor,
    const char* op,
    const char* name,
    int64_t count,
    int64_t width)
{
    if (!tensor || !tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor or tensor shape is NULL");

    if (tensor->ndim != 2 || tensor->shape[0] != count || tensor->shape[1] != width)
    {
        std::ostringstream oss;
        oss << op << ": expected " << name << " shape [" << count << ", " << width
            << "] (use a 2D tensor such as (" << count << ", " << width
            << "), not a flattened 1D tensor)";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    return success();
}

ovphysx_result_t validateContactMatrixTensorShape(
    const DLTensor* tensor,
    const char* op,
    const char* name,
    int64_t sensorCount,
    int64_t filterCount)
{
    if (!tensor || !tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor or tensor shape is NULL");

    if (tensor->ndim != 2 || tensor->shape[0] != sensorCount || tensor->shape[1] != filterCount)
    {
        std::ostringstream oss;
        oss << op << ": expected " << name << " shape [" << sensorCount << ", " << filterCount << "]";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    return success();
}

// 1D shape `[count]` -- used by ovphysx_read_raw_contact_data's per-sensor
// count/start-index tensors and by the other-actor IDs tensor (paired with
// validateContactIdTensorDtype for the int64/uint64 dtype check).
ovphysx_result_t validateContactVectorTensorShape(
    const DLTensor* tensor,
    const char* op,
    const char* name,
    int64_t count)
{
    if (!tensor || !tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor or tensor shape is NULL");

    if (tensor->ndim != 1 || tensor->shape[0] != count)
    {
        std::ostringstream oss;
        oss << op << ": expected " << name << " shape [" << count << "]";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    return success();
}

// 64-bit signed/unsigned integer dtype check for the actor-ID tensors
// returned by getRawContactData and consumed by
// getOtherActorPathsFromIds.
ovphysx_result_t validateContactIdTensorDtype(const DLTensor* tensor, const char* op, const char* name)
{
    if (!tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor is NULL");

    const bool is64BitInteger =
        (tensor->dtype.code == kDLInt || tensor->dtype.code == kDLUInt) &&
        tensor->dtype.bits == 64 &&
        tensor->dtype.lanes == 1;
    if (!is64BitInteger)
    {
        std::ostringstream oss;
        oss << op << ": expected int64 or uint64 tensor for " << name;
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    return success();
}

ovphysx_result_t validateDetailedContactFloatTensor(
    const DLTensor* tensor,
    const ContactBindingState& binding,
    const char* op,
    const char* name,
    int64_t count,
    int64_t width)
{
    ovphysx_result_t dtypeCheck = validateContactDstTensorDtype(tensor, op);
    if (dtypeCheck.status != OVPHYSX_API_SUCCESS) return dtypeCheck;

    ovphysx_result_t deviceCheck = validateContactDeviceMatch(tensor, binding, op);
    if (deviceCheck.status != OVPHYSX_API_SUCCESS) return deviceCheck;

    return validateContactFlatTensorShape(tensor, op, name, count, width);
}

ovphysx_result_t validateDetailedContactCountTensor(
    const DLTensor* tensor,
    const ContactBindingState& binding,
    const char* op,
    const char* name,
    int64_t sensorCount,
    int64_t filterCount)
{
    ovphysx_result_t dtypeCheck = validateContactCountTensorDtype(tensor, op, name);
    if (dtypeCheck.status != OVPHYSX_API_SUCCESS) return dtypeCheck;

    ovphysx_result_t deviceCheck = validateContactDeviceMatch(tensor, binding, op);
    if (deviceCheck.status != OVPHYSX_API_SUCCESS) return deviceCheck;

    return validateContactMatrixTensorShape(tensor, op, name, sensorCount, filterCount);
}

} // namespace

void ovphysx_contact_binding_cleanup_instance(InstanceData* instance)
{
    if (!instance) return;
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    for (auto& kv : instance->contact_bindings)
        destroyContactBindingResources(kv.second);
    instance->contact_bindings.clear();
}

extern "C" {

OVPHYSX_API ovphysx_result_t ovphysx_create_contact_binding(
    ovphysx_handle_t handle,
    const ovphysx_string_t* sensor_patterns,
    uint32_t sensor_patterns_count,
    const ovphysx_string_t* filter_patterns,
    uint32_t filters_per_sensor,
    uint32_t max_contact_data_count,
    ovphysx_contact_binding_handle_t* out_handle)
{
    if (!sensor_patterns || sensor_patterns_count == 0 || !out_handle)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid parameters");

    if (filters_per_sensor > 0 && !filter_patterns)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "filters_per_sensor > 0 requires non-NULL filter_patterns");

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
        if (attach_status != OVPHYSX_API_SUCCESS)
            return set_error(attach_status, "failed to attach physics stage");
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    auto* tensorApi = getTensorApi();
    if (!tensorApi)
        return set_error(OVPHYSX_API_ERROR, "TensorApi unavailable");

    std::vector<std::string> sensorPatterns;
    sensorPatterns.reserve(sensor_patterns_count);
    for (uint32_t i = 0; i < sensor_patterns_count; ++i)
    {
        if (!sensor_patterns[i].ptr || sensor_patterns[i].length == 0)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "sensor_patterns contains empty entry");
        if (hasEmbeddedNul(sensor_patterns[i]))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "sensor_patterns contains an embedded NUL byte");
        sensorPatterns.push_back(toStdString(sensor_patterns[i]));
    }

    std::vector<std::vector<std::string>> filterPatterns;
    if (filters_per_sensor > 0 && filter_patterns)
    {
        filterPatterns.resize(sensor_patterns_count);
        for (uint32_t s = 0; s < sensor_patterns_count; ++s)
        {
            filterPatterns[s].reserve(filters_per_sensor);
            for (uint32_t f = 0; f < filters_per_sensor; ++f)
            {
                const uint32_t idx = s * filters_per_sensor + f;
                if (!filter_patterns[idx].ptr || filter_patterns[idx].length == 0)
                    return set_error(OVPHYSX_API_INVALID_ARGUMENT, "filter_patterns contains empty entry");
                if (hasEmbeddedNul(filter_patterns[idx]))
                    return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     "filter_patterns contains an embedded NUL byte");
                filterPatterns[s].push_back(toStdString(filter_patterns[idx]));
            }
        }
    }

    ContactBindingState binding;
    struct BindingGuard {
        ContactBindingState* state{nullptr};
        bool active{true};
        ~BindingGuard() {
            if (active && state)
                destroyContactBindingResources(*state);
        }
        void disarm() { active = false; }
    } guard{&binding};

    binding.stageId = instance->attachedStageId;

    binding.simView = tensorApi->createSimulationView(instance->attachedStageId);
    if (!binding.simView || !binding.simView->getValid())
        return set_error(OVPHYSX_API_ERROR, "failed to create simulation view for contact binding");

    binding.contactView = binding.simView->createRigidContactView(
        sensorPatterns, filterPatterns, max_contact_data_count);
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR,
                         "failed to create rigid contact view: no sensor entries were produced. "
                         "Common causes: an authored USD sensor prim without PhysxContactReportAPI "
                         "applied; a sensor or filter pattern that matches no object; filter_patterns "
                         "not resolving to one prim or one per sensor");

    const ovphysx_contact_binding_handle_t contactHandle = ovphysx::internal::allocateOpaqueObjectHandle();
    if (contactHandle == OVPHYSX_INVALID_HANDLE)
        return set_error(OVPHYSX_API_ERROR, "opaque object handle space exhausted");

    {
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        instance->contact_bindings[contactHandle] = std::move(binding);
    }
    guard.disarm();

    *out_handle = contactHandle;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_contact_binding(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle)
{
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    {
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        auto it = instance->contact_bindings.find(contact_handle);
        if (it == instance->contact_bindings.end())
            return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");
        destroyContactBindingResources(it->second);
        instance->contact_bindings.erase(it);
    }

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_contact_binding_spec(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    int32_t* out_sensor_count,
    int32_t* out_filter_count)
{
    if (!out_sensor_count || !out_filter_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "output pointers are NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    *out_sensor_count = static_cast<int32_t>(binding.contactView->getSensorCount());
    *out_filter_count = static_cast<int32_t>(binding.contactView->getFilterCount());
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_contact_binding_get_sensor_paths(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    ovphysx_string_t* out_paths,
    uint32_t max_paths,
    uint32_t* out_count)
{
    if (!out_paths || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_paths or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    const uint32_t sensorCount = binding.contactView->getSensorCount();
    if (binding.sensorPathCache.size() != sensorCount)
    {
        binding.sensorPathCache.clear();
        binding.sensorPathCache.reserve(sensorCount);
        for (uint32_t i = 0; i < sensorCount; ++i)
        {
            const char* path = binding.contactView->getUsdPrimPath(i);
            binding.sensorPathCache.emplace_back(path ? path : "");
        }
    }

    const uint32_t toWrite = (sensorCount < max_paths) ? sensorCount : max_paths;
    for (uint32_t i = 0; i < toWrite; ++i)
        out_paths[i] = ovphysx_cstr(binding.sensorPathCache[i].c_str());

    *out_count = toWrite;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_contact_binding_get_filter_paths(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    ovphysx_string_t* out_paths,
    uint32_t max_paths,
    uint32_t* out_count)
{
    if (!out_paths || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_paths or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    const uint32_t sensorCount = binding.contactView->getSensorCount();
    const uint32_t filterCount = binding.contactView->getFilterCount();
    if (filterCount != 0 && sensorCount > UINT32_MAX / filterCount)
        return set_error(OVPHYSX_API_ERROR, "contact binding filter path count overflow");
    const uint32_t pathCount = sensorCount * filterCount;
    if (binding.filterPathCache.size() != pathCount)
    {
        binding.filterPathCache.clear();
        binding.filterPathCache.reserve(pathCount);
        for (uint32_t sensorIdx = 0; sensorIdx < sensorCount; ++sensorIdx)
        {
            for (uint32_t filterIdx = 0; filterIdx < filterCount; ++filterIdx)
            {
                const char* path = binding.contactView->getFilterUsdPrimPath(sensorIdx, filterIdx);
                binding.filterPathCache.emplace_back(path ? path : "");
            }
        }
    }

    const uint32_t toWrite = (pathCount < max_paths) ? pathCount : max_paths;
    for (uint32_t i = 0; i < toWrite; ++i)
        out_paths[i] = ovphysx_cstr(binding.filterPathCache[i].c_str());

    *out_count = toWrite;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_contact_binding_capacity(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    uint32_t* out_max_contact_data_count)
{
    if (!out_max_contact_data_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_max_contact_data_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    *out_max_contact_data_count = binding.contactView->getMaxContactDataCount();
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_read_contact_net_forces(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    DLTensor* dst_tensor)
{
    if (!dst_tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "dst_tensor is NULL");

    {
        ovphysx_result_t dtypeCheck = validateContactDstTensorDtype(dst_tensor, "read_contact_net_forces");
        if (dtypeCheck.status != OVPHYSX_API_SUCCESS) return dtypeCheck;
    }

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_result_t warmup = ovphysx_gpu_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS) return warmup;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    const float dt = instance->last_step_dt;

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    ovphysx_result_t deviceCheck = validateContactDeviceMatch(dst_tensor, binding, "read_contact_net_forces");
    if (deviceCheck.status != OVPHYSX_API_SUCCESS) return deviceCheck;

    const int32_t sensorCount = static_cast<int32_t>(binding.contactView->getSensorCount());

    if (dst_tensor->ndim != 2 || !dst_tensor->shape ||
        dst_tensor->shape[0] != sensorCount || dst_tensor->shape[1] != 3)
    {
        std::ostringstream oss;
        oss << "expected dst shape [" << sensorCount << ", 3]";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    omni::physics::tensors::TensorDesc dst{};
    DLConvertError err = dlToTensorDesc(dst_tensor, dst);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));

    if (!binding.contactView->getNetContactForces(&dst, dt))
        return set_error(OVPHYSX_API_ERROR, "getNetContactForces failed");

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_read_contact_force_matrix(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    DLTensor* dst_tensor)
{
    if (!dst_tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "dst_tensor is NULL");

    {
        ovphysx_result_t dtypeCheck = validateContactDstTensorDtype(dst_tensor, "read_contact_force_matrix");
        if (dtypeCheck.status != OVPHYSX_API_SUCCESS) return dtypeCheck;
    }

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_result_t warmup = ovphysx_gpu_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS) return warmup;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    const float dt = instance->last_step_dt;

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    ovphysx_result_t deviceCheck = validateContactDeviceMatch(dst_tensor, binding, "read_contact_force_matrix");
    if (deviceCheck.status != OVPHYSX_API_SUCCESS) return deviceCheck;

    const int32_t sensorCount = static_cast<int32_t>(binding.contactView->getSensorCount());
    const int32_t filterCount = static_cast<int32_t>(binding.contactView->getFilterCount());

    if (dst_tensor->ndim != 3 || !dst_tensor->shape ||
        dst_tensor->shape[0] != sensorCount ||
        dst_tensor->shape[1] != filterCount ||
        dst_tensor->shape[2] != 3)
    {
        std::ostringstream oss;
        oss << "expected dst shape [" << sensorCount << ", " << filterCount << ", 3]";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    omni::physics::tensors::TensorDesc dst{};
    DLConvertError err = dlToTensorDesc(dst_tensor, dst);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));

    if (!binding.contactView->getContactForceMatrix(&dst, dt))
        return set_error(OVPHYSX_API_ERROR, "getContactForceMatrix failed");

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_read_contact_data(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    DLTensor* contact_force_tensor,
    DLTensor* contact_point_tensor,
    DLTensor* contact_normal_tensor,
    DLTensor* contact_separation_tensor,
    DLTensor* contact_count_tensor,
    DLTensor* contact_start_indices_tensor)
{
    static constexpr const char* op = "read_contact_data";

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_result_t warmup = ovphysx_gpu_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS) return warmup;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    const float dt = instance->last_step_dt;

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    const int64_t sensorCount = static_cast<int64_t>(binding.contactView->getSensorCount());
    const int64_t filterCount = static_cast<int64_t>(binding.contactView->getFilterCount());
    const int64_t maxContactDataCount = static_cast<int64_t>(binding.contactView->getMaxContactDataCount());
    if (maxContactDataCount <= 0)
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            "read_contact_data requires max_contact_data_count > 0 at contact binding creation");
    if (filterCount <= 0)
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            "read_contact_data requires filters_per_sensor > 0 at contact binding creation");

    ovphysx_result_t validation = validateDetailedContactFloatTensor(
        contact_force_tensor, binding, op, "contact_force_tensor", maxContactDataCount, 1);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        contact_point_tensor, binding, op, "contact_point_tensor", maxContactDataCount, 3);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        contact_normal_tensor, binding, op, "contact_normal_tensor", maxContactDataCount, 3);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        contact_separation_tensor, binding, op, "contact_separation_tensor", maxContactDataCount, 1);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactCountTensor(
        contact_count_tensor, binding, op, "contact_count_tensor", sensorCount, filterCount);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactCountTensor(
        contact_start_indices_tensor, binding, op, "contact_start_indices_tensor", sensorCount, filterCount);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;

    omni::physics::tensors::TensorDesc force{};
    omni::physics::tensors::TensorDesc point{};
    omni::physics::tensors::TensorDesc normal{};
    omni::physics::tensors::TensorDesc separation{};
    omni::physics::tensors::TensorDesc count{};
    omni::physics::tensors::TensorDesc startIndices{};

    DLConvertError err = dlToTensorDesc(contact_force_tensor, force);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_point_tensor, point);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_normal_tensor, normal);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_separation_tensor, separation);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_count_tensor, count);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_start_indices_tensor, startIndices);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));

    const ContactDataReadStatus readStatus =
        binding.contactView->getContactData(&force, &point, &normal, &separation, &count, &startIndices, dt);
    return mapContactDataReadStatus(readStatus, op, static_cast<uint32_t>(maxContactDataCount));
}

OVPHYSX_API ovphysx_result_t ovphysx_read_raw_contact_data(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    DLTensor* contact_force_tensor,
    DLTensor* contact_point_tensor,
    DLTensor* contact_normal_tensor,
    DLTensor* contact_separation_tensor,
    DLTensor* contact_count_tensor,
    DLTensor* contact_start_indices_tensor,
    DLTensor* other_actor_ids_tensor)
{
    static constexpr const char* op = "read_raw_contact_data";

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_result_t warmup = ovphysx_gpu_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS) return warmup;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    const float dt = instance->last_step_dt;

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    const int64_t sensorCount = static_cast<int64_t>(binding.contactView->getSensorCount());
    const int64_t maxContactDataCount = static_cast<int64_t>(binding.contactView->getMaxContactDataCount());
    if (maxContactDataCount <= 0)
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            "read_raw_contact_data requires max_contact_data_count > 0 at contact binding creation");

    // Float tensors: [C, 1] or [C, 3] -- shared validator with read_contact_data.
    ovphysx_result_t validation = validateDetailedContactFloatTensor(
        contact_force_tensor, binding, op, "contact_force_tensor", maxContactDataCount, 1);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        contact_point_tensor, binding, op, "contact_point_tensor", maxContactDataCount, 3);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        contact_normal_tensor, binding, op, "contact_normal_tensor", maxContactDataCount, 3);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        contact_separation_tensor, binding, op, "contact_separation_tensor", maxContactDataCount, 1);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;

    // Count/start-index tensors here are 1D [S] (raw is per-sensor, not per-(sensor, filter)).
    {
        ovphysx_result_t check = validateContactCountTensorDtype(contact_count_tensor, op, "contact_count_tensor");
        if (check.status != OVPHYSX_API_SUCCESS) return check;
        check = validateContactDeviceMatch(contact_count_tensor, binding, op);
        if (check.status != OVPHYSX_API_SUCCESS) return check;
        check = validateContactVectorTensorShape(contact_count_tensor, op, "contact_count_tensor", sensorCount);
        if (check.status != OVPHYSX_API_SUCCESS) return check;
    }
    {
        ovphysx_result_t check = validateContactCountTensorDtype(contact_start_indices_tensor, op, "contact_start_indices_tensor");
        if (check.status != OVPHYSX_API_SUCCESS) return check;
        check = validateContactDeviceMatch(contact_start_indices_tensor, binding, op);
        if (check.status != OVPHYSX_API_SUCCESS) return check;
        check = validateContactVectorTensorShape(contact_start_indices_tensor, op, "contact_start_indices_tensor", sensorCount);
        if (check.status != OVPHYSX_API_SUCCESS) return check;
    }
    // other_actor_ids: [C] int64/uint64.
    {
        ovphysx_result_t check = validateContactIdTensorDtype(other_actor_ids_tensor, op, "other_actor_ids_tensor");
        if (check.status != OVPHYSX_API_SUCCESS) return check;
        check = validateContactDeviceMatch(other_actor_ids_tensor, binding, op);
        if (check.status != OVPHYSX_API_SUCCESS) return check;
        check = validateContactVectorTensorShape(other_actor_ids_tensor, op, "other_actor_ids_tensor", maxContactDataCount);
        if (check.status != OVPHYSX_API_SUCCESS) return check;
    }

    omni::physics::tensors::TensorDesc force{};
    omni::physics::tensors::TensorDesc point{};
    omni::physics::tensors::TensorDesc normal{};
    omni::physics::tensors::TensorDesc separation{};
    omni::physics::tensors::TensorDesc count{};
    omni::physics::tensors::TensorDesc startIndices{};
    omni::physics::tensors::TensorDesc otherActorIds{};

    auto convert = [&](DLTensor* dl, omni::physics::tensors::TensorDesc& td) {
        DLConvertError err = dlToTensorDesc(dl, td);
        if (err != DLConvertError::Success)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
        return success();
    };
    {
        ovphysx_result_t r;
        r = convert(contact_force_tensor, force);           if (r.status != OVPHYSX_API_SUCCESS) return r;
        r = convert(contact_point_tensor, point);           if (r.status != OVPHYSX_API_SUCCESS) return r;
        r = convert(contact_normal_tensor, normal);         if (r.status != OVPHYSX_API_SUCCESS) return r;
        r = convert(contact_separation_tensor, separation); if (r.status != OVPHYSX_API_SUCCESS) return r;
        r = convert(contact_count_tensor, count);           if (r.status != OVPHYSX_API_SUCCESS) return r;
        r = convert(contact_start_indices_tensor, startIndices); if (r.status != OVPHYSX_API_SUCCESS) return r;
        r = convert(other_actor_ids_tensor, otherActorIds); if (r.status != OVPHYSX_API_SUCCESS) return r;
    }

    const ContactDataReadStatus readStatus = binding.contactView->getRawContactData(
        &force, &point, &normal, &separation, &count, &startIndices, &otherActorIds, dt);
    return mapContactDataReadStatus(readStatus, op, static_cast<uint32_t>(maxContactDataCount));
}

OVPHYSX_API ovphysx_result_t ovphysx_contact_binding_get_other_actor_paths_from_ids(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    DLTensor* ids_tensor,
    ovphysx_string_t* out_paths,
    uint32_t max_paths,
    uint32_t* out_count)
{
    static constexpr const char* op = "get_other_actor_paths_from_ids";

    if (!out_paths || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_paths or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    // ID tensor: 1D int64/uint64; length is free (driven by caller).
    // validateContactIdTensorDtype already rejected NULL, so by this point
    // ids_tensor is non-NULL; only shape and ndim still need checking.
    ovphysx_result_t dtype = validateContactIdTensorDtype(ids_tensor, op, "ids_tensor");
    if (dtype.status != OVPHYSX_API_SUCCESS) return dtype;
    if (!ids_tensor->shape || ids_tensor->ndim != 1)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ids_tensor must be a 1D tensor");
    // Actor-ID -> path resolution is host-side (getOtherActorPathsFromIds reads
    // the IDs directly). A GPU ID tensor would be dereferenced as host memory
    // and silently yield no paths, so require CPU here rather than matching the
    // binding's device (which, for a GPU binding, would force exactly that bad
    // GPU input). Callers with GPU IDs must copy to host first.
    if (ids_tensor->device.device_type != kDLCPU)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "ids_tensor must be a CPU tensor; actor-ID resolution is host-side");

    omni::physics::tensors::TensorDesc ids{};
    DLConvertError err = dlToTensorDesc(ids_tensor, ids);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));

    // Refill the cache; pointers handed back to the caller stay valid until
    // the next call (which clears and refills the same vector) or destroy.
    binding.otherActorPathsCache.clear();
    binding.contactView->getOtherActorPathsFromIds(&ids, binding.otherActorPathsCache);

    const uint32_t total = static_cast<uint32_t>(binding.otherActorPathsCache.size());
    const uint32_t toWrite = (total < max_paths) ? total : max_paths;
    for (uint32_t i = 0; i < toWrite; ++i)
        out_paths[i] = ovphysx_cstr(binding.otherActorPathsCache[i].c_str());

    *out_count = total; // total needed; only toWrite entries are written
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_read_friction_data(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t contact_handle,
    DLTensor* friction_force_tensor,
    DLTensor* friction_point_tensor,
    DLTensor* contact_count_tensor,
    DLTensor* contact_start_indices_tensor)
{
    static constexpr const char* op = "read_friction_data";

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_result_t warmup = ovphysx_gpu_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS) return warmup;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    const float dt = instance->last_step_dt;

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->contact_bindings.find(contact_handle);
    if (it == instance->contact_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding not found");

    const ContactBindingState& binding = it->second;
    if (!binding.contactView)
        return set_error(OVPHYSX_API_ERROR, "contact view is null");

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "contact binding invalidated (stage changed); recreate binding");

    const int64_t sensorCount = static_cast<int64_t>(binding.contactView->getSensorCount());
    const int64_t filterCount = static_cast<int64_t>(binding.contactView->getFilterCount());
    const int64_t maxContactDataCount = static_cast<int64_t>(binding.contactView->getMaxContactDataCount());
    if (maxContactDataCount <= 0)
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            "read_friction_data requires max_contact_data_count > 0 at contact binding creation");
    if (filterCount <= 0)
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            "read_friction_data requires filters_per_sensor > 0 at contact binding creation");

    ovphysx_result_t validation = validateDetailedContactFloatTensor(
        friction_force_tensor, binding, op, "friction_force_tensor", maxContactDataCount, 3);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactFloatTensor(
        friction_point_tensor, binding, op, "friction_point_tensor", maxContactDataCount, 3);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactCountTensor(
        contact_count_tensor, binding, op, "contact_count_tensor", sensorCount, filterCount);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;
    validation = validateDetailedContactCountTensor(
        contact_start_indices_tensor, binding, op, "contact_start_indices_tensor", sensorCount, filterCount);
    if (validation.status != OVPHYSX_API_SUCCESS) return validation;

    omni::physics::tensors::TensorDesc force{};
    omni::physics::tensors::TensorDesc point{};
    omni::physics::tensors::TensorDesc count{};
    omni::physics::tensors::TensorDesc startIndices{};

    DLConvertError err = dlToTensorDesc(friction_force_tensor, force);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(friction_point_tensor, point);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_count_tensor, count);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));
    err = dlToTensorDesc(contact_start_indices_tensor, startIndices);
    if (err != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(err));

    const ContactDataReadStatus readStatus =
        binding.contactView->getFrictionData(&force, &point, &count, &startIndices, dt);
    return mapContactDataReadStatus(readStatus, op, static_cast<uint32_t>(maxContactDataCount));
}

} // extern "C"
