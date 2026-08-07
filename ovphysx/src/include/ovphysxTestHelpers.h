// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// Internal test helper functions - NOT part of the public API
// These are used by unit tests to validate internal behavior

#include "ovphysx/ovphysx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the CUDA context backing a tensor binding's simulation view.
 *
 * This is an internal helper intended for unit tests that need to perform CUDA driver
 * operations (allocation/copies) using the same CUDA context as PhysX uses internally.
 *
 * @param handle Valid PhysX SDK instance handle
 * @param binding Valid tensor binding handle created from @p handle
 * @param out_cuda_ctx [out] Receives the CUDA context pointer value cast to uintptr_t (0 if unavailable)
 * @return true if the binding was found and @p out_cuda_ctx was written, false otherwise
 */
OVPHYSX_API bool ovphysx_get_tensor_binding_cuda_context_internal(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding,
    uintptr_t* out_cuda_ctx
);

/**
 * @brief Get the IOptionalCuda function table from ovphysx's linked PhysX runtime.
 *
 * The runtime accessor is internal to libovphysx.so. This helper exposes the
 * pointer only to tests that need to perform CUDA driver operations.
 *
 * @return Opaque pointer to omni::physx::IOptionalCuda (cast to void*), or nullptr
 */
OVPHYSX_API void* ovphysx_get_optional_cuda_internal(void);

/**
 * @brief Get the TensorApi function table from ovphysx's linked PhysX runtime.
 *
 * @return Opaque pointer to omni::physics::tensors::TensorApi (cast to void*), or nullptr
 */
OVPHYSX_API void* ovphysx_get_tensor_api_internal(void);

/**
 * @brief Reset schema-path registration one-shot state for isolated unit tests.
 */
OVPHYSX_API void ovphysx_reset_schema_path_registration_internal(void);

typedef bool (*ovphysx_test_set_viz_scope_tokens_fn)(const ovx_primpath_t*, uint32_t);

OVPHYSX_API ovphysx_test_set_viz_scope_tokens_fn
ovphysx_exchange_set_viz_scope_tokens_internal(ovphysx_test_set_viz_scope_tokens_fn replacement);

/**
 * @brief Override the OVStage attachment state for isolated unit tests.
 */
OVPHYSX_API bool ovphysx_set_ovstage_attachment_state_internal(
    ovphysx_handle_t handle,
    bool attached,
    int64_t stage_id);

#ifdef __cplusplus
}
#endif
