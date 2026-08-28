// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/DLPackConvert.h"
#include "ovphysx/ovphysx_export.h"
#include <omni/physics/tensors/ISdfShapeView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <carb/logging/Log.h>
#include <sstream>

using ovphysx::internal::DLConvertError;
using ovphysx::internal::dlConvertErrorMessage;
using ovphysx::internal::dlToTensorDesc;
using ovphysx::internal::getTensorApi;

extern std::shared_mutex g_instances_mutex;
extern InstanceData* get_instance_ptr(ovphysx_handle_t handle);

namespace
{
// Release the backing ISdfShapeView for one state entry. Shared by the
// single-handle destroy path and the per-instance bulk cleanup.
void releaseSdfViewState(InstanceData::SdfViewState& state)
{
    if (state.view)
    {
        state.view->release();
        state.view = nullptr;
    }
}

ovphysx_result_t checkSdfViewStageValid(InstanceData* instance, const InstanceData::SdfViewState& state)
{
    if (instance->attachedStageId != state.stageId)
    {
        return set_error(OVPHYSX_API_NOT_FOUND,
                         "SDF view invalidated (stage changed); recreate view");
    }
    return success();
}
} // namespace

// Release all SDF views owned by an instance. Mirrors
// ovphysx_tensor_binding_cleanup_instance / ovphysx_contact_binding_cleanup_instance.
void ovphysx_sdf_view_cleanup_instance(InstanceData* instance)
{
    if (!instance) return;
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    for (auto& kv : instance->sdf_views)
        releaseSdfViewState(kv.second);
    instance->sdf_views.clear();
}

OVPHYSX_API ovphysx_result_t ovphysx_create_sdf_view(
    ovphysx_handle_t handle,
    ovphysx_string_t pattern,
    uint32_t max_query_points,
    ovphysx_sdf_view_handle_t* out_handle)
{
    if (!out_handle)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_handle is NULL");
    *out_handle = 0;

    if (!isValid(pattern))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "pattern is empty");
    if (hasEmbeddedNul(pattern))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "pattern contains an embedded NUL byte");
    if (max_query_points == 0)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "max_query_points must be > 0");

    omni_sdk_physx_wait_all_pending_internal(handle);

    ovphysx_api_status_t attachStatus = ovphysx_ensure_physics_attached(handle);
    if (attachStatus != OVPHYSX_API_SUCCESS)
        return set_error(attachStatus, "physics attach failed");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    auto* tensorApi = getTensorApi();
    if (!tensorApi)
        return set_error(OVPHYSX_API_ERROR, "TensorApi unavailable (plugins not loaded?)");

    const std::string patternStr(pattern.ptr, pattern.length);

    auto* simView = tensorApi->createSimulationView(instance->attachedStageId);
    if (!simView || !simView->getValid())
    {
        if (simView) simView->release(false);
        return set_error(OVPHYSX_API_ERROR, "failed to create simulation view");
    }

    // SDF evaluation is GPU-only. CpuSimulationView::createSdfShapeView is unimplemented and
    // returns null, which would otherwise surface below as a misleading "no shapes matched" error.
    if (simView->getDeviceOrdinal() < 0)
    {
        simView->release(false);
        return set_error(OVPHYSX_API_ERROR,
                         "SDF evaluation requires a GPU instance; CPU SDF evaluation is not implemented");
    }

    omni::physics::tensors::ISdfShapeView* sdfView =
        simView->createSdfShapeView(patternStr.c_str(), max_query_points);

    simView->release(false);

    if (!sdfView)
    {
        std::ostringstream oss;
        oss << "Pattern '" << patternStr << "' did not match any SDF shapes";
        return set_error(OVPHYSX_API_ERROR, oss.str());
    }

    CARB_LOG_INFO("Created SDF view with %u shapes for pattern '%s'",
                  sdfView->getCount(), patternStr.c_str());

    InstanceData::SdfViewState state;
    state.stageId = instance->attachedStageId;
    state.maxQueryPoints = max_query_points;
    state.view = sdfView;

    const ovphysx_sdf_view_handle_t newHandle = ovphysx::internal::allocateOpaqueObjectHandle();
    if (newHandle == OVPHYSX_INVALID_HANDLE)
    {
        releaseSdfViewState(state);
        return set_error(OVPHYSX_API_ERROR, "opaque object handle space exhausted");
    }

    {
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        instance->sdf_views[newHandle] = std::move(state);
    }

    *out_handle = newHandle;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_sdf_view_get_count(
    ovphysx_handle_t handle,
    ovphysx_sdf_view_handle_t sdf_handle,
    uint32_t* out_count)
{
    if (!out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_count is NULL");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->sdf_views.find(sdf_handle);
    if (it == instance->sdf_views.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "SDF view not found");

    ovphysx_result_t stageCheck = checkSdfViewStageValid(instance, it->second);
    if (stageCheck.status != OVPHYSX_API_SUCCESS)
        return stageCheck;

    *out_count = it->second.view ? it->second.view->getCount() : 0;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_sdf_view_get_max_query_points(
    ovphysx_handle_t handle,
    ovphysx_sdf_view_handle_t sdf_handle,
    uint32_t* out_max_query_points)
{
    if (!out_max_query_points)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_max_query_points is NULL");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->sdf_views.find(sdf_handle);
    if (it == instance->sdf_views.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "SDF view not found");

    ovphysx_result_t stageCheck = checkSdfViewStageValid(instance, it->second);
    if (stageCheck.status != OVPHYSX_API_SUCCESS)
        return stageCheck;

    *out_max_query_points = it->second.maxQueryPoints;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_evaluate_sdf(
    ovphysx_handle_t handle,
    ovphysx_sdf_view_handle_t sdf_handle,
    const DLTensor* query_points,
    DLTensor* out_distances_and_gradients)
{
    if (!query_points)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "query_points is NULL");
    if (!out_distances_and_gradients)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_distances_and_gradients is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    {
        ovphysx_result_t warmup = ovphysx_gpu_warmup_if_needed(handle, /*is_explicit_call=*/false);
        if (warmup.status != OVPHYSX_API_SUCCESS)
            return warmup;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    // CRITICAL: Hold tensor_binding_mutex across the entire evaluation to prevent a
    // use-after-free if the view is destroyed by another thread mid-call. TensorAPI
    // views are NOT ref-counted; this mirrors ovphysx_read_tensor_binding.
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->sdf_views.find(sdf_handle);
    if (it == instance->sdf_views.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "SDF view not found");

    ovphysx_result_t stageCheck = checkSdfViewStageValid(instance, it->second);
    if (stageCheck.status != OVPHYSX_API_SUCCESS)
        return stageCheck;

    omni::physics::tensors::ISdfShapeView* view = it->second.view;
    if (!view)
        return set_error(OVPHYSX_API_ERROR, "SDF view has null backing view");

    omni::physics::tensors::TensorDesc srcDesc{};
    DLConvertError srcErr = dlToTensorDesc(query_points, srcDesc);
    if (srcErr != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         std::string("query_points: ") + dlConvertErrorMessage(srcErr));

    omni::physics::tensors::TensorDesc dstDesc{};
    DLConvertError dstErr = dlToTensorDesc(out_distances_and_gradients, dstDesc);
    if (dstErr != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         std::string("out_distances_and_gradients: ") + dlConvertErrorMessage(dstErr));

    if (!view->getSdfAndGradients(&dstDesc, &srcDesc))
        return set_error(OVPHYSX_API_ERROR, "SDF evaluation failed");

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_sdf_view(
    ovphysx_handle_t handle,
    ovphysx_sdf_view_handle_t sdf_handle)
{
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    {
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        auto it = instance->sdf_views.find(sdf_handle);
        if (it == instance->sdf_views.end())
            return success();

        releaseSdfViewState(it->second);
        instance->sdf_views.erase(it);
    }
    return success();
}
