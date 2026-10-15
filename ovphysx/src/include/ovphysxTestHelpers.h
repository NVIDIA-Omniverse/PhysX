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
    ovphysx_instance_handle_t handle,
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

typedef bool (*ovphysx_test_set_viz_scope_tokens_fn)(uint64_t, const ovx_primpath_t*, uint32_t);

OVPHYSX_API ovphysx_test_set_viz_scope_tokens_fn
ovphysx_exchange_set_viz_scope_tokens_internal(ovphysx_test_set_viz_scope_tokens_fn replacement);

/**
 * @brief Override the OVStage attachment state for isolated unit tests.
 */
OVPHYSX_API bool ovphysx_set_ovstage_attachment_state_internal(
    ovphysx_instance_handle_t handle,
    bool attached,
    int64_t stage_id);

/**
 * @brief Create live objects for every OmniPVD-producing PhysXExtensions family.
 *
 * The returned opaque fixture belongs to the caller and must be destroyed with
 * @ref ovphysx_destroy_extensions_fixture_for_test_internal before the instance.
 */
OVPHYSX_API bool ovphysx_create_extensions_fixture_for_test_internal(
    ovphysx_instance_handle_t handle,
    void** out_fixture);

/** @brief Destroy a fixture created by ovphysx_create_extensions_fixture_for_test_internal. */
OVPHYSX_API void ovphysx_destroy_extensions_fixture_for_test_internal(void* fixture);

/**
 * @brief Report the process-wide worker pool sizes ovphysx started with.
 *
 * @p out_tasking_threads receives the live carb.tasking worker count, which is
 * what the pool actually started with rather than what was requested.
 * @p out_tasking_setting and @p out_tbb_setting receive the configured counts,
 * where zero means nothing was configured and the pool sized itself to the
 * machine. @p out_hardware_concurrency receives the logical CPU count Carbonite
 * reports, which is cgroup aware on Linux.
 *
 * @return true when carb.tasking is running and every output was written
 */
OVPHYSX_API bool ovphysx_get_worker_pool_counts_for_test_internal(
    uint32_t* out_tasking_threads,
    int32_t* out_tasking_setting,
    int32_t* out_tbb_setting,
    uint32_t* out_hardware_concurrency);

typedef void (*ovphysx_test_task_fn)(void* user);

/**
 * @brief Block the calling Carbonite task until probes have run.
 *
 * The caller is already inside an object-change callback. This waits on a
 * task mutex owned by @ref ovphysx_with_suspended_carbonite_task_for_test_internal,
 * so the task can migrate workers while it is suspended.
 */
OVPHYSX_API void ovphysx_wait_on_task_guard_for_test_internal(void);

/**
 * @brief Run @p body on a Carbonite task and @p probe on other tasks while it is suspended.
 *
 * Carbonite's framework pointer belongs to the ovphysx module that started
 * tasking. Calling these task APIs from the test executable would bind a
 * different module. @p body is expected to reach
 * @ref ovphysx_wait_on_task_guard_for_test_internal. Probes then run on more
 * tasks than there are workers, including the worker the body task released.
 *
 * @return false when tasking is not running or a wait exceeds 10 seconds
 */
OVPHYSX_API bool ovphysx_with_suspended_carbonite_task_for_test_internal(
    ovphysx_test_task_fn body,
    ovphysx_test_task_fn probe,
    void* user);

#ifdef __cplusplus
}
#endif
