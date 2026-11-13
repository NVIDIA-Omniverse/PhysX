// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-16
 */

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

/** @brief Create an attachment-owned runtime D6 joint without a population record. */
OVPHYSX_API void* ovphysx_create_d6_joint_at_path_for_test_internal(
    ovphysx_instance_handle_t handle, const char* joint_path, const char* body0_path, const char* body1_path);

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

/** Read issued-step bookkeeping after the test has joined its client threads. */
OVPHYSX_API bool ovphysx_read_step_bookkeeping_for_test_internal(
    ovphysx_instance_handle_t handle, float* simulation_time, float* last_step_dt);


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

/**
 * @brief Hold or release an instance worker for tests.
 *
 * While held, the worker starts no job; submissions still queue, so their
 * receipts stay pending. Release before destroying the instance or before any
 * call that waits for the worker to drain.
 *
 * @return false for an unknown instance
 */
OVPHYSX_API bool ovphysx_set_worker_hold_for_test_internal(ovphysx_instance_handle_t handle, bool hold);

/**
 * @brief Queue a job on an instance worker for tests.
 *
 * The job sleeps @p sleep_ms milliseconds on the worker, then succeeds, or fails
 * with @p failure when it is non-null. The failure is reported as
 * "test_job (task <index>): <failure>". The job touches no engine state.
 *
 * @return the accepted receipt, or a rejection with zero identity
 */
OVPHYSX_API ovphysx_enqueue_result_t ovphysx_submit_test_job_for_test_internal(
    ovphysx_instance_handle_t handle,
    const char* failure,
    uint32_t sleep_ms);

/**
 * @brief Wait for held worker jobs from more Carbonite tasks than there are workers.
 *
 * Holds the worker, queues one test job per waiter, and starts twice the
 * carb.tasking thread count plus two tasks that each call ovphysx_wait_task()
 * with an infinite timeout on their own job. Every waiter must then be counted as
 * a task suspended inside the executor, none as a blocked thread, and one more
 * probe task must still run, which proves that waiting tasks release their worker
 * threads. Then the worker is released and every wait must return SUCCESS.
 *
 * @param[out] out_waiters Receives the number of waiting tasks; may be NULL.
 * @return false when tasking is not running, the probe cannot run within ten
 *         seconds, or a wait does not succeed within thirty seconds
 */
OVPHYSX_API bool ovphysx_wait_from_saturated_carbonite_pool_for_test_internal(
    ovphysx_instance_handle_t handle,
    uint32_t* out_waiters);

/**
 * @brief Queue a job on an instance worker that calls @p fn with @p user.
 *
 * The job runs @p fn on the instance worker and then succeeds; destruction and
 * shutdown cancel it while it is still queued. The job touches no engine state
 * itself. @p fn must not block forever.
 *
 * @return the accepted receipt, or a rejection with zero identity
 */
OVPHYSX_API ovphysx_enqueue_result_t ovphysx_submit_test_callback_job_for_test_internal(
    ovphysx_instance_handle_t handle,
    ovphysx_test_task_fn fn,
    void* user);

/** Callback for a test-only pre- or post-step subscription. */
typedef void (*ovphysx_test_step_event_fn)(float elapsed_time, void* user);

/**
 * @brief Subscribe a test callback to one instance's exact live attachment.
 *
 * @return a nonzero subscription id, or zero when the instance, attachment,
 *         callback or runtime interface is unavailable
 */
OVPHYSX_API uint64_t ovphysx_subscribe_step_event_for_test_internal(
    ovphysx_instance_handle_t handle,
    bool pre_step,
    ovphysx_test_step_event_fn fn,
    void* user);

/** @brief Remove a subscription returned by ovphysx_subscribe_step_event_for_test_internal. */
OVPHYSX_API bool ovphysx_unsubscribe_step_event_for_test_internal(uint64_t subscription);

/** @brief Record text in the active runtime error scope, if any. */
OVPHYSX_API void ovphysx_record_runtime_error_for_test_internal(const char* message);

/**
 * @brief Hold an instance worker as soon as the next job is queued on it.
 *
 * Unlike @ref ovphysx_set_worker_hold_for_test_internal this leaves the instance
 * idle until that submission, so an ovphysx_step() on it still starts its
 * simulation on the calling thread, and its fetch job then stays queued. Release
 * with ovphysx_set_worker_hold_for_test_internal(handle, false).
 *
 * @return false for an unknown instance
 */
OVPHYSX_API bool ovphysx_hold_worker_after_next_submit_for_test_internal(ovphysx_instance_handle_t handle);

/**
 * @brief Report whether an instance's worker thread has left its loop for good.
 *
 * @return false for an unknown instance
 */
OVPHYSX_API bool ovphysx_worker_exited_for_test_internal(ovphysx_instance_handle_t handle, bool* out_exited);

/**
 * @brief Call @p fn with @p user while holding the process-wide lifecycle lock.
 *
 * Attach and detach jobs and instance destruction take the same lock around their
 * lifecycle work, so they block while @p fn runs.
 */
OVPHYSX_API void ovphysx_with_lifecycle_lock_for_test_internal(ovphysx_test_task_fn fn, void* user);

/**
 * @brief Wait for an instance lane from more Carbonite tasks than there are workers,
 * and destroy another instance from one more task meanwhile.
 *
 * Holds @p handle's worker with one queued job, so its lane is busy, then starts
 * twice the carb.tasking thread count plus two tasks that each make a synchronous
 * lane-taking call on @p handle (ovphysx_set_config of an empty active_cuda_gpus,
 * so @p handle must not be attached). Every waiter must be counted as a task
 * suspended inside the executor, none as a blocked thread. Then one more task
 * destroys @p victim, whose worker is held with a queued job, and must finish
 * while the waiters still wait. Then the hold is released and every call must
 * return SUCCESS.
 *
 * @param[out] out_waiters Receives the number of waiting tasks; may be NULL.
 * @return false when tasking is not running, a condition above does not hold
 *         within ten seconds, or a call does not succeed within thirty seconds
 */
OVPHYSX_API bool ovphysx_lane_waits_and_destroy_from_saturated_carbonite_pool_for_test_internal(
    ovphysx_instance_handle_t handle,
    ovphysx_instance_handle_t victim,
    uint32_t* out_waiters);

/**
 * @brief Count the CUDA driver calls ovphysx made to run long calls on instance workers.
 *
 * Counts the capture of the submitting thread's CUDA context for an attach, its push
 * and pop on the instance worker, and each worker's stream-capture mode call, across
 * the process. In hard CPU-only mode none of them happens, so the count never changes.
 */
OVPHYSX_API uint64_t ovphysx_worker_cuda_calls_for_test_internal(void);

/**
 * @brief Set the next shared ovphysx opaque-object handle.
 *
 * Lets output-session tests reach the end of the shared handle space. Returns
 * the previous next value, which the test restores before creating any other
 * opaque object.
 */
OVPHYSX_API uint64_t ovphysx_exchange_next_output_handle_for_test_internal(uint64_t next);

/** Where instances run their short async calls (see ovphysx_set_test_execution_for_test_internal). */
typedef enum
{
    /** On the calling thread of an idle instance, queued behind its work otherwise (the normal behaviour). */
    OVPHYSX_TEST_EXECUTION_NATURAL = 0,
    /** Every short async call queued as a job on the instance worker, even on an idle instance. */
    OVPHYSX_TEST_EXECUTION_QUEUE = 1,
} ovphysx_test_execution_t;

/**
 * @brief Choose where the instances created from now on run their short async calls.
 *
 * For tests only, so a test suite covers the path it does not take by itself, such as the
 * queued path of calls it makes on an idle instance; never a user setting. Calls that always
 * run on the calling thread, and synchronous calls, are not affected. Instances that already
 * exist keep their mode.
 *
 * @return false for an unknown mode, which changes nothing
 */
OVPHYSX_API bool ovphysx_set_test_execution_for_test_internal(ovphysx_test_execution_t mode);

#ifdef __cplusplus
}
#endif
