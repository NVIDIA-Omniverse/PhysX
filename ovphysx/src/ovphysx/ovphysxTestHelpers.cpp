// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "ovphysxTestHelpers.h"

#include "UsdSchemaPaths/UsdSchemaPaths.h"
#include <omni/physx/PhysXRuntime.h>

#include <carb/Framework.h>
#include <omni/physx/IOptionalCuda.h>
#include <omni/physics/tensors/ISimulationView.h>

#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"

extern "C" {

OVPHYSX_API bool ovphysx_get_tensor_binding_cuda_context_internal(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding,
    uintptr_t* out_cuda_ctx)
{
    if (!out_cuda_ctx) return false;
    *out_cuda_ctx = 0;

    auto instance = get_instance(handle);
    if (!instance) return false;

    // NOTE: Accessing tensorBindings is protected by InstanceData::tensorBindingMutex.
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    const auto it = instance->tensor_bindings.find(binding);
    if (it == instance->tensor_bindings.end()) return false;

    const auto& b = it->second;
    if (!b.simView) return true;  // binding exists but has no simView (ctx remains 0)

    *out_cuda_ctx = reinterpret_cast<uintptr_t>(b.simView->getCudaContext());
    return true;
}

OVPHYSX_API void* ovphysx_get_optional_cuda_internal(void)
{
    return static_cast<void*>(omni::physx::runtime::tryGetOptionalCudaInterface());
}

OVPHYSX_API void* ovphysx_get_tensor_api_internal(void)
{
    return static_cast<void*>(omni::physx::runtime::tryGetTensorApiInterface());
}

OVPHYSX_API void ovphysx_reset_schema_path_registration_internal(void)
{
    omni::sdk::usd_schema_paths::resetSchemaPathRegistrationForTests();
}

OVPHYSX_API ovphysx_test_set_viz_scope_tokens_fn
ovphysx_exchange_set_viz_scope_tokens_internal(ovphysx_test_set_viz_scope_tokens_fn replacement)
{
    return g_sidecarSetVizScopeTokens.exchange(replacement, std::memory_order_acq_rel);
}

OVPHYSX_API bool ovphysx_set_ovstage_attachment_state_internal(
    ovphysx_handle_t handle,
    bool attached,
    int64_t stage_id)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return false;
    instance->ovstage_attached = attached;
    instance->attachedStageId = stage_id;
    return true;
}

} // extern "C"
