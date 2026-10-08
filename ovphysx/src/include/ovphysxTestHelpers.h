// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Internal test helpers used by the unit tests. Not part of the public API.

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
 * @brief Read the process-global attach-time CUDA selector for an isolated test.
 *
 * This is a test-only observation of private runtime state, not the device of
 * an actual CUDA context or independent state owned by each ovphysx handle.
 *
 * @param out_value [out] Receives the current integer value
 * @return true when the setting exists as an integer, false otherwise
 */
OVPHYSX_API bool ovphysx_get_attach_cuda_selector_for_test_internal(int32_t* out_value);

/**
 * @brief Set an exact-length Carbonite string for configuration-boundary tests.
 */
OVPHYSX_API bool ovphysx_set_raw_string_setting_for_test_internal(
    const char* path,
    const char* value,
    size_t length);

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

/**
 * @brief Create live objects for every OmniPVD-producing PhysXExtensions family.
 *
 * The returned opaque fixture belongs to the caller and must be destroyed with
 * @ref ovphysx_destroy_extensions_fixture_for_test_internal before the instance.
 */
OVPHYSX_API bool ovphysx_create_extensions_fixture_for_test_internal(
    ovphysx_handle_t handle,
    void** out_fixture);

/** @brief Destroy a fixture created by ovphysx_create_extensions_fixture_for_test_internal. */
OVPHYSX_API void ovphysx_destroy_extensions_fixture_for_test_internal(void* fixture);

#ifdef __cplusplus
}
#endif
