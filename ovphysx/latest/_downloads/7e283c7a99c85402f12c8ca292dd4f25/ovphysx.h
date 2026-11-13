// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-OMNIPVD-CAPTURE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-7
 */
/** @endcond */

/** @cond OVPHYSX_PLC_TRACE */

/** @implements REQ-CAPI-DETACH-001
 * @covers AC-3 AC-4 AC-5
 */

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11 AC-12 AC-13 AC-14
 */
/**
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 * @implements REQ-CAPI-STRING-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CAPI-STEP-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1 AC-4 AC-9 AC-10 AC-11 AC-12
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-2 AC-3 AC-6
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3 AC-8
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-5a AC-11
 * @implements REQ-CAPI-READ-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-PYTHON-CLONE-001
 * @covers AC-2
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-3
 * @implements REQ-CAPI-CONTACT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 * @implements REQ-CAPI-CONTACT-002
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-SCENEQUERY-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 * @implements REQ-CAPI-PATTERN-002
 * @covers AC-8
 * @implements REQ-CAPI-SESSION-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
 * @implements REQ-CAPI-OVSTAGE-ATTACH-001
 * @covers AC-5
 * @implements REQ-CAPI-TEARDOWN-001
 * @covers AC-1 AC-2 AC-3 AC-5
 * @implements REQ-CAPI-HOST-EXIT-001
 * @covers AC-4
 * @implements REQ-CAPI-COMPILED-CACHE-001
 * @covers AC-2 AC-3 AC-5
 */
/** @endcond */


// C-compatible PhysX API

#ifndef OVPHYSX_OVPHYSX_H
#define OVPHYSX_OVPHYSX_H

#include <stdint.h>
#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif


    /** @deprecated Use ovphysx_ovd_start_recording(). Forwarding export introduced in 0.6 and retained in 0.7. */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_ovd_start_recording") ovphysx_result_t
    ovphysx_start_recording(ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination);
    /** @deprecated Use ovphysx_ovd_stop_recording(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_ovd_stop_recording") ovphysx_result_t
    ovphysx_stop_recording(ovphysx_instance_handle_t handle);
    /** @deprecated Use ovphysx_ovd_is_recording(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_ovd_is_recording") ovphysx_result_t
    ovphysx_is_recording(ovphysx_instance_handle_t handle, bool* out_is_recording);

    /* General notes
    *
    * Version information: See ovphysx_get_version() and ovphysx_get_version_string() below.
    *
    * DLPack Integration:
    *   This API uses DLPack for tensor data exchange between the user and the system.
    *   Same-device paths are zero-copy where supported. Cross-device support depends on
    *   the tensor type and process policy. See ovphysx_legacy.h (tensor bindings) and
    *   dlpack.h for the tensor format.
    *
    * Return values:
    *   Synchronous functions (blocking calls) return ovphysx_result_t (status).
    *   Task producers return ovphysx_enqueue_result_t (status, op_index, instance).
    *   SUCCESS means admission; collect execution status with a task wait. A
    *   task-returning call may complete before it returns, or later: do not depend
    *   on either.
    *   On failure, call ovphysx_get_last_error() on the same thread to retrieve the
    *   error message. The returned string is valid until the next ovphysx API call on
    *   that thread.
    *
    * Asynchronous execution:
    *   - Calls on one instance run in submission order.
    *   - By default, a dedicated worker runs attach, update, detach, clone, and
    *     step. These calls usually return before their work finishes.
    *   - Query, read, write, fetch, commit, and release run immediately when the
    *     instance is idle. Otherwise they queue behind earlier work. A new query,
    *     read, or write handle is available on acceptance; wait before using its
    *     other outputs.
    *   - Calls that need the caller's CUDA stream wait for earlier work, then run
    *     on the calling thread.
    *   - During OmniPVD recording, attach and detach also run on the calling thread.
    *   - Set OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION to false before instance
    *     creation to disable its worker. Calls then finish before returning.
    *   - An infinite wait may run an unstarted job. Finite waits and polls only
    *     observe progress. Do not depend on the thread that runs callbacks.
    *   - Wait for pending tasks before CUDA graph capture on the same context.
    *   - Use ovphysx_wait_task() for one result, ovphysx_wait_all() for all
    *     outstanding results, or ovphysx_query_task_status() to inspect without
    *     collecting. A result can be collected once.
    *   - Serialize all calls on the same instance, including waits and error lookup.
    *
    * Failure handling:
    *   - A rejected call returns an error and no task. Common reasons are invalid
    *     arguments or handles, callback re-entry, a full queue, or a stopped worker.
    *     Each function lists its immediate errors.
    *   - An accepted call reports later state or execution errors through its task
    *     wait. One failed task does not stop later valid work.
    *   - ovphysx_wait_task() reports only its task. ovphysx_wait_all() reports all
    *     uncollected failures, including failures whose receipts were discarded.
    *
    * Memory visibility:
    *   - Writes from operation N are visible to operation N+1.
    *   - Independent operations may execute concurrently internally for performance.
    *
    * Documentation and skills:
    *   Tutorials, guides, and AI-agent playbooks ship alongside this header.
    *   See SKILLS.md in the package root for a skills index, and README.md
    *   for a quick-start overview.
    */

    /*--------------------------------------------------*/
    /* Thread Safety */
    /*--------------------------------------------------*/

    /*
     * Thread Safety Guarantees:
     *
    * Multiple ovphysx instances:
    *   - Wrapper-owned state (bindings, sessions, subscriptions, op waits,
    *     debug-render cache, scene-query buffers) is per handle.
    *   - Global functions (initialize/shutdown, create_instance, version,
    *     CPU-mode and global config, logging, get_last_error, destroy_wait_result,
    *     writability) omit
    *     the instance and must not mutate per-handle simulation state.
    *   - Each instance holds at most one live ovstage attach. Several
    *     instances may be attached at the same time. Attaching on one
    *     instance does not detach another. The same ovstage instance
    *     cannot be attached twice.
    *   - Different instances may step their own attachments concurrently.
    *     Process-wide limits still apply: one hard CPU-only mode, one SDK
    *     tolerance scale, and one recording session. Recording start and stop
    *     wait until every instance finished its queued and running work,
    *     without collecting task results. Attach, detach and destroy calls on
    *     different instances may also be made concurrently; attach and detach
    *     of different instances still execute one at a time. When a destroy
    *     overlaps another thread's destroy, recording start or stop, or detach
    *     during a recording, one of the two may return the retryable
    *     OVPHYSX_API_INVALID_STATE.
    *   - Per-instance ordering does not provide global exclusion for native
    *     calls. Cached-query validation, recording-status queries, collection
    *     of completed receipts and already-absent releases may progress while
    *     another instance's lifecycle callback is active.
     *
     * Single instance, multiple threads:
     *   - Operations are NOT thread-safe for the same instance.
     *   - Use external synchronization if calling from multiple threads; calls may
     *     come from different threads, one at a time.
     *
     * Data buffer lifetimes:
     *   - The caller keeps data passed to async operations valid and unmodified
     *     until the operation completes (as indicated by ovphysx_wait_task),
     *     except the inputs a call is done with when it returns: ovphysx_clone()
     *     copies its paths, poses and environment ids, and a short call copies its
     *     strings, attribute and path arrays and selection descriptors when it is
     *     queued. Tensor and mask data, fetched groups and output slots stay
     *     borrowed until the task completes, and the Stage as
     *     ovphysx_attach_ovstage() describes.
     *
     * CUDA context requirements:
     *   - The SDK manages CUDA contexts internally where needed.
     *   - Task waits manage the SDK's context. CUDA stream handoffs use the caller's
     *     current context for default-stream sentinels; see ovphysx_commit_group().
     */

    /* ovphysx_attach_ovstage consumes a caller-owned ovstage Stage handle as an
     * opaque pointer so this header stays independent of ovstage's concrete C ABI.
     * Include ovstage headers in application code to create and manage the Stage. */

    /**
     * @defgroup ovphysx_instance Instance management
     * Create, configure, and destroy ovphysx instances.
     */

    /**
     * @defgroup ovphysx_settings Global settings
     * Process-wide configuration values affecting all instances.
     */

    /**
     * @defgroup ovphysx_stage Stage management
     * USD stage loading, cloning, and introspection.
     */

    /**
     * @defgroup ovphysx_simulation Simulation
     * Simulation stepping and time control.
     */

    /**
     * @defgroup ovphysx_attribute_binding Attribute bindings
     * Attribute-based read/write APIs and binding lifetime control.
     */

    /**
     * @defgroup ovphysx_stream Task completion
     * Wait for tasks and collect their outcomes.
     */

    /**
     * @defgroup ovphysx_errors Error handling
     * Query borrowed error strings and destroy wait-result error arrays.
     */

    /**
     * @defgroup ovphysx_deprecated Deprecated APIs
     * Legacy API surface maintained for compatibility.
     */

    /*--------------------------------------------------*/
    /* Process-global lifecycle operations */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_lifecycle Process-global lifecycle operations
     * Process-global lifecycle management.
     */

    /** @addtogroup ovphysx_lifecycle */
    /** @{ */

    /**
     * @brief Initialize process-global ovphysx lifecycle state.
     *
     * @return
     * - OVPHYSX_API_SUCCESS if lifecycle state was initialized.
     * - OVPHYSX_API_ERROR if lifecycle state is already initialized, or on x86_64 if
     *   the host CPU or OS does not expose AVX (required by pre-built x86_64 binaries).
     */
    OVPHYSX_API ovphysx_result_t ovphysx_initialize(void);

    /** @} */

    /*--------------------------------------------------*/
    /* Creation and destruction operations */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_instance */
    /** @{ */

    /**
     * @brief Create a new instance with default instance configuration.
     *
     * Set process-global configuration before this call. After creation,
     * instance configuration may be changed with ovphysx_set_config() until
     * the instance attaches a stage.
     *
     * @param out_handle [out] New owned instance handle.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT when out_handle
     * is NULL, or OVPHYSX_API_ERROR when lifecycle/runtime initialization fails
     * or Carbonite cannot start the instance's worker thread (see
     * OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION).
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_instance(ovphysx_instance_handle_t* out_handle);

    /**
     * @brief Destroy an ovphysx instance and release per-instance resources.
     *
     * Per-instance destruction leaves the process lifecycle active. Call
     * @ref ovphysx_shutdown after destroying the final handle to drain the
     * direct PhysX runtime and clear that lifecycle token.
     *
     * Destruction is synchronous. It waits only for the instance's running task,
     * then checks whether it must be retried; a retryable refusal
     * leaves the remaining queued tasks queued, and they run as if destroy had
     * not been called. Otherwise it cancels every queued task that has not
     * started, joins the instance's worker thread, and then releases the
     * instance's output sessions and attachment. While an OmniPVD recording is
     * active it first also waits for the queued and running work of every other
     * instance; its own queued tasks are cancelled all the same.
     *
     * @param handle ovphysx handle to destroy.
     * @return ovphysx_result_t with status and error info.
     *
     * @post On success, the handle is unregistered and its resources are
     *       released. Waits and ovphysx_query_task_status() on its receipts,
     *       including cancelled ones, report OVPHYSX_API_NOT_FOUND. A repeated
     *       call for that already-unregistered value
     *       returns OVPHYSX_API_ERROR before teardown and has no effect on
     *       registered instances or process-global asynchronous state.
     *
     * @par Side Effects
     * Releases internal resources, plugins, and cached data for this instance.
     *
     * @par Threading
     * Do not destroy an instance while it is in use on other threads.
     * Runtime, object-change and application log callbacks reject destruction
     * with OVPHYSX_API_INVALID_STATE before teardown. Retry after the callback.
     *
     * @par Ownership
     * After destruction, any bindings created by this instance are invalid.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_STATE is retryable: admission rejected before teardown;
     *   the handle, attachment, queued tasks and caller ownership remain live.
     *   Close companion leases or finish pending work/transitions before retrying.
     *   A call inside a log, runtime or object-change callback, or on the
     *   instance's own worker thread, is rejected the same way; retry after the
     *   callback returns.
     * - OVPHYSX_API_ERROR if the handle is not registered or destruction fails.
     *   An unregistered handle is rejected before teardown and does not affect
     *   registered instances or process-global asynchronous state. No error
     *   string is returned. Consult logs for other destruction failures.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t destroy_owned_instance(ovphysx_instance_handle_t handle)
     * {
     *     return ovphysx_destroy_instance(handle);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_destroy_instance(ovphysx_instance_handle_t handle);

    /**
     * @brief Clear the ovphysx process-lifecycle token.
     *
     * Clears the process-global initialized state set by @ref ovphysx_initialize.
     * It does not destroy live handles and does not balance
     * @ref ovphysx_create_instance. Callers must destroy every handle explicitly
     * with @ref ovphysx_destroy_instance.
     *
     * Must be paired with a prior @ref ovphysx_initialize call. Call once when
     * the application is done with the current process-lifecycle scope. After
     * shutdown, callers must invoke @ref ovphysx_initialize again before creating
     * another instance.
     *
     * @return
     * - OVPHYSX_API_SUCCESS on success.
     * - OVPHYSX_API_INVALID_STATE is retryable: retained companion service leases
     *   still prevent shutdown, or the call was made inside a runtime or
     *   object-change callback or on an instance's worker thread. The lifecycle
     *   token remains live.
     * - OVPHYSX_API_ERROR if called without a matching @ref ovphysx_initialize.
     * - OVPHYSX_API_ERROR if called from the application log callback.
     *   Use @ref ovphysx_get_last_error() for details.
     *
     * On every successful return, including when live handles remain, shutdown
     * flushes Carbonite's buffered records, stops accepting application-callback
     * delivery, and drains accepted callbacks. When runtime teardown occurs, the
     * barrier includes its final log producer. The callback and its user-data
     * resources may then be released. Accepted callbacks must not wait
     * indefinitely on the thread performing shutdown.
     *
     * Before callback delivery ends, shutdown stops the worker thread of every
     * live handle: it waits for that instance's running task, cancels its queued
     * tasks and joins the thread. No ovphysx worker thread runs once
     * shutdown returns. A process that exits without shutdown never joins worker
     * threads: they are left parked, and no exit handler stops them. Unloading the
     * library therefore requires a completed shutdown.
     *
     * @par Static runtime mode
     * With no live handles, shutdown drains the direct PhysX runtime. On Linux it
     * also releases the SDK-owned Carbonite framework and private cooking cache.
     * Windows retains the framework until process exit for safe dependency DLL
     * teardown; private cache removal after datastore shutdown is best-effort.
     * A framework populated by a foreign host remains owned by that host, which
     * must destroy every instance and complete shutdown before it releases
     * Carbonite or unloads carb.tasking, the provider of the instances' worker
     * threads. If live handles remain, callers still own them and the direct
     * runtime remains available only for their explicit destruction. Continued
     * stepping or other instance work after shutdown is unsupported: new tasks on
     * those handles are rejected, while the outcomes of their earlier tasks stay
     * collectable until destruction. Destruction-time records are not delivered
     * to the application callback disabled by shutdown.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_shutdown(void);

    /** @} */

    /*--------------------------------------------------*/
    /* Version information (runtime)                    */
    /*--------------------------------------------------*/

    /** @defgroup ovphysx_version Version information */
    /** @{ */

    /**
     * @brief Get runtime version of the library.
     *
     * Useful for checking ABI compatibility between headers and shared library.
     * For compile-time version macros, include `ovphysx/version.h`.
     *
     * @param out_major [out] Major version (must not be NULL)
     * @param out_minor [out] Minor version (must not be NULL)
     * @param out_patch [out] Patch version (must not be NULL)
     */
    OVPHYSX_API void ovphysx_get_version(
        uint32_t* out_major,
        uint32_t* out_minor,
        uint32_t* out_patch
    );

    /**
     * @brief Get version as string (e.g., "0.1.0").
     *
     * @return Version string with static storage duration (valid for lifetime of process, do not free).
     */
    OVPHYSX_API const char* ovphysx_get_version_string(void);

    /** @} */


    /*--------------------------------------------------*/
    /* Scoped configuration                            */
    /*--------------------------------------------------*/

    /** @defgroup ovphysx_config Scoped configuration */
    /** @{ */

    /**
     * Set process-global configuration. Environment overrides remain authoritative.
     * Startup-only keys must be set before the first successful instance creation.
     * Later writes return OVPHYSX_API_INVALID_STATE without changing the value,
     * even after all instances are destroyed, until ovphysx_shutdown() succeeds
     * with no live instances.
     * Keys that act on every instance, OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING
     * and the keys that require no attached stage, first wait until every instance
     * finished its earlier work, without collecting task results, then check and apply
     * the value. That wait lasts as long as other threads keep submitting work, so do not
     * submit work on another thread while such a write runs. Inside a log, runtime or
     * object-change callback such a write returns OVPHYSX_API_INVALID_STATE without waiting.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_global_config(
        ovphysx_global_config_key_t key, ovphysx_config_value_t value);

    /**
     * Read process-global configuration. COOKED_COLLIDER_CACHE_DIRECTORY reports
     * the current /UJITSO/datastore/localCachePath setting. Before runtime startup,
     * this is the configured value, or OVPHYSX_API_NOT_FOUND if unset. With local
     * caching and cooking enabled, startup replaces an unset or unusable directory
     * with a process-private path if it can create one. If creation fails, local
     * caching is disabled and the setting is not replaced. Disabling local caching
     * or cooking also prevents this replacement.
     * The getter observes the setting, not user intent, persistence, or the active
     * datastore location. The datastore keeps its initially selected directory
     * even if a later setting change is accepted and reported by this getter.
     * COMPILED_STAGE_CACHE reports the configured mode, AUTOMATIC included, not
     * the mode an attach resolved from it.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_config(
        ovphysx_global_config_key_t key, ovphysx_config_value_t* out_value);

    /**
     * Set configuration retained by one instance.
     *
     * Current instance keys are pre-attach: changing them after a stage is
     * attached returns OVPHYSX_API_INVALID_STATE and preserves the prior value.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_config(
        ovphysx_instance_handle_t instance,
        ovphysx_config_key_t key,
        ovphysx_config_value_t value);

    /** Read configuration retained by one instance. */
    OVPHYSX_API ovphysx_result_t ovphysx_get_config(
        ovphysx_instance_handle_t instance,
        ovphysx_config_key_t key,
        ovphysx_config_value_t* out_value);

    /** @} */

    /*--------------------------------------------------*/
    /* OVD recording                                    */
    /*--------------------------------------------------*/

    /** @defgroup ovphysx_recording OVD recording */
    /** @addtogroup ovphysx_recording */
    /** @{ */

    /**
     * @brief Start an OVD recording session.
     *
     * The call is synchronous. A FILE path ending in the literal, case-sensitive
     * .ovd suffix is an exact filename. Other paths select a directory for a new
     * timestamped YYYY_MM_DD_HH_MM_SS_CC_rec.ovd file. Add a trailing separator
     * to select a directory whose name ends in .ovd. An admitted start creates
     * missing directories or exact-file parent directories recursively; see
     * ovphysx_omnipvd_destination_t. A TCP destination connects to a ready listener.
     * Validation and stream-open failures may be retried. Only one recording
     * may be active in the shared runtime: another start returns
     * OVPHYSX_API_INVALID_STATE without replacing its destination. After a
     * successful stop, another FILE or TCP session may be started. Late start
     * requires a live physics stage attached by this instance or a peer. While
     * no instance has a live attachment, it returns OVPHYSX_API_INVALID_STATE
     * without creating output.
     *
     * Make the shared runtime OVD recording capable by setting
     * OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE to true with
     * ovphysx_set_global_config() before the first ovphysx_create_instance().
     * The key defaults to false. Capability alone does not start recording;
     * attach a physics stage and call this function with the desired destination.
     * Pass ovphysx_config_value_bool(true), declared in ovphysx_config.h.
     * A default, incapable runtime returns OVPHYSX_API_INVALID_STATE
     * with an error naming the capability key. Unsupported platforms retain
     * OVPHYSX_API_NOT_IMPLEMENTED.
     *
     * Creation and attachment never start recording. After reattach, the runtime
     * remains dormant until this function is called. Call it before the first
     * simulation step to capture the initial state and every simulation frame.
     *
     * Start and stop are synchronous. Before accessing shared recording state,
     * each waits until every instance finished its earlier work, without
     * collecting task results. Argument validation can reject a call before this
     * wait. OVPHYSX_API_INVALID_STATE is returned without waiting inside a log,
     * runtime or object-change callback; and after the wait while a companion
     * operation is executing, another thread holds a recording or lifecycle
     * transition, a companion preview has unfetched simulation work or a runtime
     * operation is still executing. Fetch or close every pending preview and
     * retry after the conflicting activity finishes.
     *
     * Optional settings select exact scene and object paths and secondary data.
     * Null options record all objects and all supported data. Nonempty path lists
     * require the caller's attachment. Malformed paths return INVALID_ARGUMENT;
     * missing or unsupported objects return NOT_FOUND. Validation completes before
     * opening the destination. Lists are copied; duplicate paths are permitted.
     * Stop and start again to change the selection.
     *
     * @param handle Instance requesting the recording session.
     * @param destination FILE or TCP destination borrowed for this call.
     * @param options Optional recording settings borrowed and copied during this call.
     *        Set wait_for_selection to true for TCP discovery before detailed recording.
     *        This requires the caller's live attachment. The initial catalog contains
     *        identities and relationships within its scene/object scope, with no
     *        simulation payload or GPU readback. Receiver selection cannot widen
     *        that scope. Empty lists permit all supported objects in the attachment.
     *        record_secondary_data is ignored until the receiver chooses it.
     *        Catalog handles are local to this connection and run. The owner's
     *        is_recording query is true while waiting; stop or owner detach ends it.
     *        Keep calling the owner's public step/step_sync to apply buffered
     *        selections before simulation; warmup does not poll requests. A busy
     *        runtime or pending companion work defers application to a later step.
     *        FILE rejects wait_for_selection with INVALID_ARGUMENT.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT for an invalid
     *         handle or destination, OVPHYSX_API_INVALID_STATE under the
     *         synchronization conditions above, when the runtime is not
     *         recording-capable, the shared runtime has no live physics stage,
     *         recording is active, or the call reenters a runtime or application
     *         log callback before transition admission,
     *         OVPHYSX_API_NOT_IMPLEMENTED on an unsupported platform, or
     *         OVPHYSX_API_ERROR when synchronization, runtime access,
     *         destination opening, or sampling fails.
     * A null destination returns INVALID_ARGUMENT before callback rejection.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_ovd_start_recording(
        ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination,
        const ovphysx_omnipvd_recording_options_t* options);

    /**
     * @brief Stop and finalize the active recording or discovery session.
     *
     * The handle that started the session owns it. A peer handle is inactive
     * and cannot stop the owner's session. A successful owner stop
     * permits a later recording session to start.
     * Synchronizes like ovphysx_ovd_start_recording(), including its
     * OVPHYSX_API_INVALID_STATE synchronization conditions.
     *
     * Callback reentry returns INVALID_STATE before transition admission.
     *
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT for an invalid
     *         handle, OVPHYSX_API_INVALID_STATE under the synchronization
     *         conditions above or when this handle owns no active recording
     *         (including while a peer owns the globally active session),
     *         OVPHYSX_API_NOT_IMPLEMENTED on an unsupported platform, or
     *         OVPHYSX_API_ERROR when synchronization or stop/finalization fails.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_ovd_stop_recording(ovphysx_instance_handle_t handle);

    /**
     * @brief Query whether this instance owns the active recording or discovery session.
     *
     * A peer reports false while another instance owns the shared runtime's
     * active recording. The answer reflects the last completed recording or
     * lifecycle transition; the call waits for no instance work and may be made
     * from any thread, including inside callbacks.
     *
     * @param handle Valid instance handle.
     * @param out_is_recording [out] Receives true while this owner's recording or discovery session is active.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT for an invalid
     *         handle or null out_is_recording.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_ovd_is_recording(
        ovphysx_instance_handle_t handle, bool* out_is_recording);

    /** @} */

    /*--------------------------------------------------*/
    /* Stage building and introspection operations      */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_stage */
    /** @{ */


    /**
    * @brief Attach an ovstage Stage as the orchestration data surface.
    *
    * Once attached, the orchestration contract is **explicit and application-owned**
    * in both directions (the application owns ordinal advancement):
    *
    *   - Application to physics (control in): the application authors dirty control
    *     attributes into ovstage (drive:force, drive:velocity, drive:position_target,
    *     physics:mass, physics:gravityMagnitude, physics:gravityDirection, and so on)
    *     at ordinals it chooses, then calls @ref ovphysx_update_from_ovstage to drain
    *     that ordinal range into the running simulation. @ref ovphysx_step then integrates.
    *   - Physics to application (output out): @ref ovphysx_step does **not** author
    *     simulation output back into the Stage on its own. The application reads the
    *     step's output with @ref ovphysx_read / @ref ovphysx_fetch_read_next and writes
    *     it back into ovstage at a separate, higher (physics-output) ordinal. That
    *     ordinal is never covered by update_from_ovstage, so physics never reprocesses
    *     its own writes. See the ordinal-coupling section on @ref ovphysx_query.
    *
    * This is one consistent model: initial parse at attach, explicit control updates
    * via update_from_ovstage, and application-owned output writeback via the read API.
    * For direct control without going through ovstage, use the session write API
    * (@ref ovphysx_write). The legacy tensor bindings remain available for the same
    * purpose but are deprecated.
    *
    * The attachment is init-style. Call it once per instance, before any
    * ovphysx_step(). Replacing the attached Stage requires successful completion
    * of ovphysx_detach_ovstage() first.
    *
    * Returns a task receipt. The call queues the attach as a job on the instance's
    * worker thread and returns without running it or waiting for it; the worker runs
    * it after every earlier job, and may even finish it before the call returns. An
    * instance without a worker thread (see OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION)
    * runs it on the calling thread before the call returns.
    * Admission success does not establish that attachment succeeded. Calls submitted
    * afterwards (update, clone, step, queries) run after it in instance stream order;
    * collect the task with ovphysx_wait_task() before using the attachment outside
    * that order. ovphysx_is_stage_attached() reports the attachment only once the job
    * ran. The job runs in the CUDA context that is current on the calling thread when
    * the call is made (an active-context attach creates its GPU scene in it): keep
    * that context alive until the task completes. While an OmniPVD recording is active
    * the attach runs on the calling thread instead (see the recording note below).
    *
    * @param handle ovphysx instance handle.
    * @param stage Caller-owned ovstage Stage (`ovstage_instance_t*`).
    * @param read_ordinal Caller-owned ordinal at which the initial scan's
    *      required physics data is sealed, from 1 through OVSTAGE_ORDINAL_MAX.
    *      0 is reserved as the runtime's internal "use payload attach-time
    *      ordinal" sentinel, and ovphysx does not attach at the negative
    *      ordinals ovstage also writes: seal the population to attach at a
    *      positive ordinal.
    *      The application owns ordinal advancement. Subsequent edits are drained
    *      via ovphysx_update_from_ovstage(). A lower read_ordinal does not
    *      reconstruct historical payloads or body membership; the initial scan
    *      can observe later-sealed values and bodies. If initial values matter,
    *      seal the initial population and complete attach before authoring later edits.
    *      A successful attach at OVSTAGE_ORDINAL_MAX marks every representable
    *      ordinal consumed, so subsequent update ranges are successful no-ops.
    *      For continuing edits, use an actual sealed ordinal below
    *      OVSTAGE_ORDINAL_MAX.
    *
    * @pre handle is valid, ovphysx is not already attached to a Stage, stage
    *      is non-null, and read_ordinal is positive.
    * @pre stage and its sealed read_ordinal data remain valid through task
    *      completion. After successful attach, stage outlives the attachment,
    *      through successful detach completion. Destroying an attached Stage is
    *      undefined behavior. A failed attach may leave an attachment if rollback
    *      fails; check ovphysx_is_stage_attached() before releasing the Stage.
    * @post On successful task completion, ovphysx_update_from_ovstage() can observe
    *       committed Stage writes above the consumed read_ordinal through the runtime
    *       ovstage backend. No such ordinal exists after an attach at
    *       OVSTAGE_ORDINAL_MAX.
    *
    * @note Not thread-safe per instance. Like the rest of the per-instance API,
    *       the caller must serialize ovphysx_attach_ovstage() against any other
    *       call on the same handle. The already-attached check and the attach are
    *       not internally locked against concurrent foreground callers.
    * @note Each instance holds at most one live ovstage attach. Several instances
    *       may be attached at the same time. Attaching on one instance does not
    *       detach another. Detach or destroy only affects that instance's
    *       attachment. The same ovstage instance cannot be attached twice.
    *       Process-wide limits still apply: one hard CPU-only mode, one SDK
    *       tolerance scale, and one recording session. Recording start and stop wait
    *       for every instance's queued and running work without collecting task
    *       results. Different instances may be attached, detached and destroyed
    *       concurrently.
    * @note While an OmniPVD recording is active or starting, attach and detach run
    *       synchronously on the calling thread: they first finish the pending work of
    *       every instance, without collecting task results, and return a finished
    *       task, so a recording is finalized in submission order. Attach and detach
    *       jobs of different instances never run at the same time.
    * @note Each call reads OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE when it is
    *       made and uses the experimental compiled ovstage cache it selects (see
    *       ovphysx_compiled_stage_cache_mode_t). The attachment keeps that mode
    *       for subtrees that later updates add. The cache outcome is logged at
    *       OVPHYSX_LOG_INFO when the attach runs.
    *
    * @par Errors
    * Submission rejects an invalid handle, null stage, or read_ordinal below 1 with
    * OVPHYSX_API_INVALID_ARGUMENT, an unavailable runtime API with OVPHYSX_API_ERROR,
    * and a call inside a log, runtime or object-change callback, or while the
    * instance's worker is stopped, its queue is full, or a companion operation or
    * native-access lease is running, with OVPHYSX_API_INVALID_STATE. Once accepted,
    * attach reserves companion transition admission through its lifecycle work
    * and releases it before publishing the task outcome.
    * The schema guard also rejects admission
    * with OVPHYSX_API_ERROR if ovstage reports a missing PhysX schema family after
    * USD schema definitions were built, including by population or export, or
    * if probe setup or verification fails, unless refusal is downgraded to a
    * warning (see below). Rejected receipts
    * have no task identity; the schema guard takes no Stage ownership and changes
    * no attachment.
    * An instance that is already attached when the job runs ("a stage is already
    * attached") and runtime attach failures belong to the accepted task. Runtime
    * failures include unreadable schema or joint data and mismatched tolerance
    * scales. The task wait reports these errors.
    *
    * @note The application owns schema registration. Separately install
    *       physx-usd-schemas and pass both module resource directories to
    *       `ovstage_population_register_usd_schemas()` before the first USD schema
    *       read, including population or export, in each process and worker.
    *       Population drops attributes of
    *       schemas it cannot resolve, including authored PhysX settings. This
    *       call checks both plugin families, physxSchema and
    *       omniUsdPhysicsDeformableSchema, once ovstage has built its USD schema
    *       registry. If ovstage reports either family missing, this and every
    *       later attach in the process fail by default. Restart and register
    *       before populating or exporting; late registration cannot repair the registry.
    *       A procedural Stage attached while USD schema definitions remain unbuilt
    *       skips the family check and leaves it active for later attaches. Probe
    *       setup or verification errors also refuse attach, even with registered
    *       families or an unbuilt registry. The temporary directory must permit
    *       private probe creation; Windows also requires persistent ACL support.
    *       These errors are not cached, so a later attach retries verification.
    *       After population or export builds the definitions, the check applies even to
    *       procedural Stages. A registration another USD
    *       consumer made unobservable to ovstage cannot be detected. Nor can the
    *       guard identify schema families registered late before its first check:
    *       catching a late registration's OVSTAGE_ERROR_OP_FAILED can let a later
    *       attach succeed with missing data. Always restart after a registration-
    *       ordering error; attach success does not prove schemas were available
    *       during population. The raw Carbonite setting
    *       `/ovphysx/schemas/requireRegistration` defaults to true and is read on
    *       every attach whose verification fails, not latched at instance creation.
    *       Setting it to false downgrades either refusal to a warning and permits
    *       attach without repairing lost data or clearing a cached missing-family
    *       result. The guard neither locates nor installs schema resources.
    *
    * @note Scenes that author Newton `newton:*` attributes also need the
    *       separately installed Newton USD schema
    *       (https://github.com/newton-physics/newton-usd-schemas) registered
    *       the same way; ovphysx reads those attributes as fallbacks for the
    *       PhysX spellings and does not ship that schema. Python's
    *       `PhysX.attach_ovstage()` separately checks its registration and emits
    *       a warning when it cannot verify it. The C API performs no Newton
    *       check. The legacy create-time config entry
    *       `/ovphysx/schemas/warnMissingNewtonSchema = false` silences only the
    *       Python Newton warning, not the native PhysX registration guard.
    *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_enqueue_result_t attach_caller_owned_stage(
     *     ovphysx_instance_handle_t handle,
     *     ovstage_instance_t* stage,
     *     ovstage_ordinal_t read_ordinal)
     * {
     *     return ovphysx_attach_ovstage(handle, stage, read_ordinal);
     * }
     * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_attach_ovstage(ovphysx_instance_handle_t handle,
                                                         ovstage_instance_t* stage,
                                                         ovstage_ordinal_t read_ordinal);

    /**
    * @brief Pull and apply committed ovstage edits over an explicit ordinal range.
    *
    * The caller is the producer that advanced ovstage ordinals and therefore owns
    * the range boundaries, passed as ovstage's own `ovstage_ordinal_range_t`. With
    * `has_start_ordinal == true` the closed range `[start_ordinal, end_ordinal]` is
    * drained. With `has_start_ordinal == false` only `end_ordinal` is drained (the
    * single-ordinal form). The selected changes are drained through the active
    * ovstage change feed and applied to the running simulation.
    *
    * Returns a task receipt. The call queues the update as a job on the instance's
    * worker thread and returns without running it or waiting for it; the worker
    * runs it after every earlier job, and may even finish it before the call
    * returns. An instance without a worker thread (see
    * OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION) runs it on the calling thread before
    * the call returns. Collect its outcome with ovphysx_wait_task(). The range value is
    * copied; the caller-owned Stage and the selected sealed data must remain valid
    * until task completion. Successful completion makes the updates visible to
    * subsequent work.
    *
    * @param handle ovphysx instance handle.
    * @param range ovstage ordinal range to drain (see ovstage_ordinal_range_t).
    *
    * @pre `handle` is valid and an attach submitted before this call succeeds.
    * @pre when `range.has_start_ordinal`, `range.start_ordinal <= range.end_ordinal`.
    * @pre All selected writes are sealed by a completed write-floor operation
    *      covering `range.end_ordinal`. Waiting for
    *      `ovstage_population_apply_usd_changes()` only completes population.
    *      It does not advance the write floor.
    *
    * Ordinals at or below the latest successfully consumed ordinal are skipped,
    * and so are the ordinals below 1, which precede every attach read_ordinal.
    * A range containing only consumed ordinals is a successful no-op. An
    * overlapping range applies only its unread suffix. The initial
    * `read_ordinal` is consumed by ovphysx_attach_ovstage(), so replaying it does
    * not recreate the attach population or emit object-change notifications.
    * Population authored and sealed at later ordinals is applied normally.
    * Exclude ordinals used for ovstage_clone() when creating matching runtime
    * copies with ovphysx_clone(). Keep ordinary physics edits on separate
    * ordinals so they can still be consumed.
    *
    * @par Errors
    * Submission rejects an invalid range or handle with OVPHYSX_API_INVALID_ARGUMENT,
    * an unavailable runtime API with OVPHYSX_API_ERROR, and a call inside a log,
    * runtime or object-change callback, or while the instance's worker is stopped
    * or its queue is full, with OVPHYSX_API_INVALID_STATE. It also rejects a call
    * inside a CUDA graph capture with OVPHYSX_API_INVALID_STATE, because a graph
    * replay does not repeat this update. The check detects a capture of a blocking
    * stream, such as Warp's, in the calling thread's current CUDA context, also when
    * another thread runs that capture. A capture of a non-blocking stream, such as
    * the side stream PyTorch's CUDA graphs capture on, is not detected: the update
    * is accepted and runs once, at capture time. A missing attachment
    * ("no ovstage is attached") and runtime range drain failures belong to the
    * accepted task and are reported by its wait. Failed updates need not be atomic;
    * do not assume that no edits were applied or blindly replay the range.
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_from_ovstage(ovphysx_instance_handle_t handle,
                                                            ovstage_ordinal_range_t range);

    /**
    * @brief Detach the currently-attached ovstage Stage.
    *
    * Returns a task receipt, including for an idempotent no-op on an unattached
    * instance. The call queues the detach as a job on the instance's worker thread and
    * returns without running it or waiting for it; the worker runs it after every
    * earlier job, and may even finish it before the call returns. An instance without a
    * worker thread (see OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION) runs it on the
    * calling thread before the call returns. Collect the outcome with
    * ovphysx_wait_task(). An attach submitted right after it runs after it.
    * Successful task completion ends this instance's Stage ownership. Keep the
    * Stage alive until then, and longer if other users still own it. A failed
    * detach can retain attachment ownership after partially retiring resources;
    * ovphysx_is_stage_attached() is an ownership diagnostic, not a readiness check.
    *
    * This is teardown, not restoration of initial poses. Instance configuration
    * and the private simulation-time counter are preserved.
    * Clears registered interests and any output-buffer registrations, so a
    * subsequent ovphysx_attach_ovstage() to a different Stage starts clean.
    * After detach, stage-dependent calls such as ovphysx_update_from_ovstage() and
    * ovphysx_step() fail until a Stage is attached again. Detach invalidates the
    * stage's tensor and contact views. Do not read or write existing bindings.
    * Destroy them and create replacements after attaching and realizing a stage again.
    * While an OmniPVD recording is active or starting, the detach instead runs on the
    * calling thread: it first finishes the pending work of every instance, then tears
    * down and returns a finished task. Detaching the recording owner's stage or the
    * last live attachment then attempts to finalize the session. Successful
    * finalization clears its owner. A finalization failure is logged and does not
    * by itself fail the detach task; stop explicitly before detach to receive
    * recording errors. This internal wait does not collect task results; collect
    * them with ovphysx_wait_task() or ovphysx_wait_all(). Detaching a non-owner
    * preserves a peer-owned recording while another attachment remains live.
    * Detaching an unattached recording owner is a no-op and leaves its session active.
    * Reattach never starts recording automatically.
    *
    * @param handle ovphysx instance handle.
    *
    * @pre handle is valid.
    * @post On successful task completion, ovphysx is unattached.
    *
    * @par Errors
    * The call rejects invalid handles with OVPHYSX_API_INVALID_ARGUMENT.
    * OVPHYSX_API_INVALID_STATE is retryable admission failure: attachment and
    * write storage remain owned. It answers a call inside a log, runtime or
    * object-change callback, a companion operation or native-access lease running
    * at the time, pending companion previews during a recording, and a stopped
    * worker or full queue. Finish companion work or the lease and retry. A companion
    * operation submitted after an accepted detach reports busy until its lifecycle
    * work finishes; the reservation is released before publishing the task outcome.
    * Failures after teardown starts belong to the accepted task; collect them before
    * deciding whether to retry or destroy the instance.
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_detach_ovstage(ovphysx_instance_handle_t handle);

    /**
    * @brief Report whether this instance currently owns an attached stage.
    *
    * Does not wait for simulation or validate existing resources, including
    * resources from an earlier attachment of the same stage. This synchronous
    * metadata snapshot neither creates a task
    * nor collects outcomes. Wait for a lifecycle task first when its final state
    * is needed. A false snapshot during pending attach does not permit Stage
    * destruction, and true does not establish readiness. Serialize all calls on
    * an instance.
    *
    * @param instance_handle ovphysx instance handle.
    * @param stage NULL to check for any attachment, or a caller-owned stage
    *      pointer to compare with the current attachment. Never dereferenced.
    * @param[out] out_attached True when a stage is attached and matches the
    *      supplied candidate (if any); false when detached, different, or on failure.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT if out_attached is NULL or instance_handle is invalid
    */
    OVPHYSX_API ovphysx_result_t ovphysx_is_stage_attached(ovphysx_instance_handle_t instance_handle,
                                                        const void* stage, bool* out_attached);

    /*--------------------------------------------------*/
    /* Physics output read (ovstage) */
    /*--------------------------------------------------*/

    /**
     * @brief Open a query over the simulation's output objects of one simulated type.
     *
     * Mirrors the ovstage read idiom: the query is a handle (not a list). The
     * matched prims come back per group at read time as the interned
     * ovstage_read_group_t::prims.list (resolve via the attached Stage's get_path_dictionary route
     * or feed straight into the ovstage write path). Discover the produced
     * attributes / total prim count with the optional query result. Pair every
     * successful query with @ref ovphysx_release_query.
     *
     * An ovstage Stage must be attached (@ref ovphysx_attach_ovstage).
     *
     * Query, read, write, fetch, commit, and release return task receipts. They are
     * short calls (see "Stream ordered asynchronous execution" at the top of this
     * header), apart from the commits named there: on an idle instance a call runs
     * on the calling thread and returns a finished task; behind earlier work it is
     * queued and returns at once. Either way the receipt reports admission only:
     * wait for its task and check the task result before using outputs other than
     * the handle, or before starting an operation outside instance order that
     * depends on it. A query, read or write handle is written when the call is
     * accepted, and later calls may name it at once: they run after its producer.
     * Keep output slots, and the tensor data a call names, alive until task
     * completion. Attribute arrays, strings and selections are not read after the
     * call returns.
     *
     * A successful task makes host outputs ready. CUDA columns retain the event
     * contract below; task completion does not replace their readiness event.
     * Rejected submissions have no task and do not consume resources. Accepted
     * release submissions transfer cleanup to the SDK: stop using the resource
     * immediately, even if waiting later reports failure. The SDK retains or
     * quarantines storage whose safe release could not be established.
     *
     * A handle created by this instance may be passed to later calls immediately,
     * even while its producer is queued. Unknown handles, handles from another
     * instance, and handles whose release was accepted return
     * OVPHYSX_API_NOT_FOUND. Releasing an absent handle of this instance is a
     * successful no-op.
     *
     * If a producer task fails, its handle remains known but unusable. Calls that
     * name it fail their own tasks. Release failed handles to free their table
     * entries.
     *
     * Short calls reject callback re-entry, a full queue, a stopped worker, and
     * exhausted task or handle storage. If an earlier attach or detach failed,
     * dependent calls fail their tasks until that lifecycle failure is collected.
     *
     * ## Reading output without reprocessing it (ordinal coupling)
     *
     * ovphysx never authors simulation output back into the attached Stage on its
     * own. The application owns ordinal advancement and the write-back. The loop
     * that keeps physics from consuming its own output is:
     *
     *   1. The application authors *control* edits (poses, targets, gravity, and so on)
     *      into ovstage at ordinals it chooses, e.g. ordinal `c`, and seals them.
     *   2. The application calls @ref ovphysx_update_from_ovstage with an
     *      `ovstage_ordinal_range_t` covering exactly those control ordinals (e.g.
     *      `{from, c, true}`) to drain them into the running sim, then @ref ovphysx_step.
     *   3. The application reads the step's *output* with this API and writes it back
     *      into ovstage at a SEPARATE, higher ordinal `p` (the physics-output ordinal).
     *      The group's `prims.list` + `attribute` token + `data.tensors` feed the
     *      ovstage write path with no repack and no string round-trip.
     *   4. The next frame's @ref ovphysx_update_from_ovstage range must cover the
     *      application's new control ordinals but **exclude** the physics-output
     *      ordinals `p`. Because physics never feeds `p` back into update_from_ovstage,
     *      it never reprocesses its own writes.
     *
     * In short: **application-to-physics edits flow through `ovphysx_update_from_ovstage`.
     * Physics-to-application output is written at ordinals that update_from_ovstage
     * never covers.** See the ovstage usage guide for a worked example.
     *
     * ## Query semantics: a reusable selector, evaluated lazily
     *
     * A query is a SELECTOR (object_type + scope), NOT a captured membership
     * snapshot. The optional query result captures discovery at creation; later
     * read and write sessions resolve membership again and report their own counts.
     * Each operation observes every step submitted before it on the instance. A step
     * that lands between ovphysx_query and ovphysx_read does not make the handle
     * stale: the read simply reflects the newer step. OVPHYSX_SCOPE_ACTIVE uses the
     * scene's most recent active-actor report where available: rigid bodies and
     * articulation links use their own
     * activity, vehicles and vehicle wheels use their chassis, and whole
     * articulations use any active link. Without active-actor reporting, rigid bodies use current
     * awake, non-kinematic state; links and whole articulations use articulation
     * awake state; vehicles and wheels use chassis awake state. Deformables use
     * current awake state. Particles, joints, tendons and deformable materials behave as ALL.
     * Open a session before stepping again to capture the reported active set.
     * The current output producer returns
     * `layout_generation == 0`. Rebuild cached queries and layouts after a known
     * structural change rather than using it as an invalidation signal.
     *
     * OVPHYSX_QUERY_TARGET_DEBUG_RENDER selects this instance's debug-render target, not a
     * simulated object. It does not require an attached stage. Write the
     * OVPHYSX_NAME_DEBUG_* settings and read point, line, and triangle position and
     * color columns. Pass the address of the constant; do not build the name string
     * by hand. Geometry is limited to the scenes of this instance. `scope` must be
     * ALL or ACTIVE and is then ignored; it does not filter geometry. Filtering is
     * the debug-render `scope` setting. @ref ovphysx_query_selection rejects this
     * target as an invalid object type: prim and pattern selection do not apply.
     *
     * @param handle ovphysx instance handle.
     * @param object_type Query target (@ref ovphysx_query_target_t).
     * @param scope @ref ovphysx_object_scope_t (all or the type-specific active set above).
     *        Validated and ignored for OVPHYSX_QUERY_TARGET_DEBUG_RENDER.
     * @param out_summary Optional discovery snapshot evaluated at creation. NULL skips
     *      discovery work. For VEHICLE, readable attributes are the union supported by
     *      the selected drive types; write-only controls are excluded. Each read group
     *      carries its own applicable prim subset, and the read summary reports omissions.
     *      The attributes array belongs to the query and stays valid
     *      until accepted query release or instance destruction. Detach invalidates the query's attachment and dictionary use; a new attachment
     *      does not revive them. query_result_id and all_handle are zero, not resources
     *      accepted by OVStage release APIs. A debug-render query leaves this empty:
     *      it has no prim set, and no dictionary when no stage is attached.
     *      Retain input and output slots until task completion.
     * @param out_query [out] Receives the query handle, nonzero, when the call is
     *      accepted, before it returns; zero on rejection. A later call may name it
     *      at once. An empty match is a valid query whose read reaches
     *      end-of-iteration immediately and whose optional query result reports
     *      total_prim_count == 0.
     * @return Task receipt; query creation failures are reported by waiting.
     *
     * @pre handle is valid. An ovstage Stage is attached when the query runs (an
     *      attach submitted earlier counts), except for OVPHYSX_QUERY_TARGET_DEBUG_RENDER.
     * @post `*out_query` is nonzero once the call is accepted. After a failed task
     *       it stays known but unusable until it is released or the instance is
     *       destroyed: calls that name it fail their tasks, and its release succeeds.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task, `*out_query` zero):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle, object_type, or scope is invalid,
     *   or out_query is null.
     * - The rejections every short call shares (see above).
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and
     * @ref ovphysx_wait_task reports as OVPHYSX_API_ERROR, with the text
     * "ovphysx_query (task <index>): <cause>": no attached ovstage Stage when the
     * query runs (not for OVPHYSX_QUERY_TARGET_DEBUG_RENDER), an unavailable query
     * backend, a failed attach or detach submitted before it, and query
     * construction failures.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query(ovphysx_instance_handle_t handle,
                                                      ovphysx_query_target_t object_type,
                                                      ovphysx_object_scope_t scope,
                                                      ovphysx_query_handle_t* out_query,
                                                      ovstage_query_result_t* out_summary);

    /**
     * @brief Open a query narrowed to a prim-set selection (ADR-0025).
     *
     * Like @ref ovphysx_query, but the objects of `object_type` are narrowed to `selection` -- an explicit
     * list of native prim-path handles OR a glob pattern -- instead of a scope. The selection narrows BOTH
     * the read and a write opened over the returned query -- narrowing the query narrows the write, with
     * no separate write-side call. A glob matches prims through the same resolver a tensor binding uses; an
     * explicit list names prims directly by the handles the caller resolved. Either way the query keeps the
     * objects of `object_type` whose prim is selected, and an OVPHYSX_QUERY_TARGET_ARTICULATION query also keeps
     * each articulation one of whose links or joints is selected, as an articulation tensor binding does.
     * Other types are not widened: OVPHYSX_QUERY_TARGET_RIGID_BODY does not cover articulation links
     * (OVPHYSX_QUERY_TARGET_ARTICULATION_LINK does), and a link, joint or tendon query keeps only the members
     * whose own prim is selected.
     *
     * `selection->pattern` is an absolute path pattern (it starts with `/`), matched one path level per
     * `/`-separated component: `*` and `?` match within one name, `**` spans any number of levels, and
     * character classes and alternation pass through. The last component is searched at any depth below
     * what the earlier components matched, unless it is a bare `*` or the pattern contains `**`. The
     * complete rules, with examples, are on the "Path patterns" page of the ovphysx documentation
     * (docs/read_write/path_patterns.md).
     *
     * A selected query has no scope: it always covers every selected object of the type (the
     * OVPHYSX_SCOPE_ALL set), including objects the solver did not move last step. It therefore stays
     * valid across steps until a structural change, and a write opened over it never skips a selected
     * object for being asleep. For only the objects that moved, use @ref ovphysx_query with
     * OVPHYSX_SCOPE_ACTIVE.
     *
     * OVPHYSX_QUERY_TARGET_DEBUG_RENDER is not a prim set. This call rejects it with
     * OVPHYSX_API_INVALID_ARGUMENT and "invalid object type". Use @ref ovphysx_query
     * for that instance target.
     *
     * `selection` is REQUIRED (non-NULL): the whole (type, scope) set is @ref ovphysx_query.
     * `selection->prim_paths` are interned ovx_primpath_t handles (see @ref
     * ovphysx_query_selection_t), NOT strings. prim_paths wins over pattern when both are set; they are
     * never combined. A selection that resolves to no object of the queried type is a valid empty
     * query -- the optional query result reports total_prim_count == 0 and the first
     * @ref ovphysx_read fetch is end-of-iteration.
     *
     * @param handle       ovphysx instance handle.
     * @param object_type  Object type to query (see @ref ovphysx_query).
     * @param selection    Prim-set selection (must be non-NULL; NULL returns OVPHYSX_API_INVALID_ARGUMENT).
     * @param out_query [out] Receives the query handle when the call is accepted, as on
     *      @ref ovphysx_query.
     * @param out_summary Optional creation-time discovery snapshot, as on ovphysx_query.
     *      Retain output storage until task completion; the selection is not read after
     *      the call returns.
     * @return ovphysx_enqueue_result_t (an enqueue receipt; wait on it with @ref ovphysx_wait_task).
     *
     * @note Selection narrows both the read and the write for every writable object type; a selected
     *   write that admits no prim of the queried type is a valid empty write, not an error.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle or object_type is invalid (including
     *   OVPHYSX_QUERY_TARGET_DEBUG_RENDER), out_query or selection is
     *   NULL, the selection provides neither prim_paths nor a pattern, prim_paths is NULL with a
     *   non-zero prim_paths_count, prim_paths contains OVX_INVALID_PRIMPATH, or the pattern does not
     *   start with `/`, contains an embedded NUL byte, or has a component exceeding the length limit.
     * - The rejections every short call shares (see @ref ovphysx_query).
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and @ref ovphysx_wait_task reports as
     * OVPHYSX_API_ERROR, with the text "ovphysx_query_selection (task <index>): <cause>": no attached ovstage
     * Stage when the query runs, an unavailable query backend, a failed attach or detach submitted before it,
     * and query construction failures.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query_selection(ovphysx_instance_handle_t handle,
                                                                 ovphysx_query_target_t object_type,
                                                                 const ovphysx_query_selection_t* selection,
                                                                 ovphysx_query_handle_t* out_query,
                                                                 ovstage_query_result_t* out_summary);

    /**
     * @brief Open a contact query: contact sensors with per-sensor filters (ADR-0044).
     *
     * A @ref OVPHYSX_QUERY_TARGET_CONTACT_SENSOR query whose rows are the bodies -- and, on CPU simulation
     * or GPU dynamics with CPU readback, colliders carrying PhysxContactReportAPI -- the `selections`
     * select, in selection order then match order, each paired with `filters_per_sensor` filter objects.
     * Read it with @ref ovphysx_read like any query. The filters are
     * what the filtered contact attributes are measured against; the unfiltered ones (net forces) read the
     * same on this query as on @ref ovphysx_query.
     *
     * Resolved once, when the query opens, with the contact binding's rules: a body already matched by an
     * earlier selection is not repeated, and each filter selection must match one object (shared by every
     * sensor of its selection) or exactly one object per sensor (paired by position). Like the
     * contact-sensor type, a selected body is a row only while it reports contacts (it or its articulation
     * carries PhysxContactReportAPI); on CPU simulation or GPU dynamics with CPU readback a selected
     * collider carrying the schema is a row too, host-only (a DirectGPU scene has no separate child-collider
     * rows), and a body row overlaps its collider rows, so do not sum net forces or count contacts across
     * them. A contact query has no scope, and a selection
     * matching nothing adds no rows.
     *
     * The schema on a separate child collider enables sensor selection, not reporting for its owning
     * body. Enable reporting on the body, owning articulation link, or articulation root as well to avoid
     * depending on reports from the other actor in each pair. The child remains a row but reads zero
     * when neither actor reports matching contacts. Configure reporting settings on the reporting actor
     * or articulation root. A standalone static collider is its own actor and enables reporting on CPU
     * simulation and GPU dynamics with CPU readback; on DirectGPU its sensor row remains but reads zero force.
     *
     * @param handle             ovphysx instance handle.
     * @param selections         `selection_count` sensor selections with their filters.
     * @param selection_count    Number of sensor selections (at least 1).
     * @param filters_per_sensor Filter selections per sensor selection (0 for none, at most 21845).
     * @param out_query          [out] Receives the query handle when the call is accepted, as on
     *      @ref ovphysx_query.
     * @param out_summary        Optional creation-time discovery snapshot, as on @ref ovphysx_query.
     *      Retain output slots until task completion; the selections and their input buffers are not read
     *      after the call returns.
     * @return ovphysx_enqueue_result_t (an enqueue receipt; wait on it with @ref ovphysx_wait_task).
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT, before a task is created, if out_query or selections is NULL,
     *   selection_count is 0, a selection's filters is NULL while filters_per_sensor is not 0,
     *   filters_per_sensor exceeds 21845, or any selection is invalid as for @ref ovphysx_query_selection
     * - The rejections every short call shares (see @ref ovphysx_query), before a task is created
     * - OVPHYSX_API_ERROR, reported by @ref ovphysx_wait_task as "ovphysx_query_contacts (task <index>):
     *   <cause>", for no attached ovstage Stage when the query runs, an unavailable query backend, a failed
     *   attach or detach submitted before it, a filter selection whose match count is neither 1 nor the
     *   selection's sensor count, or a sensor whose filters name one object twice; the message of the last
     *   two names the selection, the filter and both counts
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query_contacts(ovphysx_instance_handle_t handle,
                                                                const ovphysx_contact_sensor_selection_t* selections,
                                                                uint32_t selection_count,
                                                                uint32_t filters_per_sensor,
                                                                ovphysx_query_handle_t* out_query,
                                                                ovstage_query_result_t* out_summary);


    /**
     * @brief Read named output attributes for a query into typed column groups.
     *
     * Opens a read session over `query` for the requested attributes, each given as
     * an `ovx_token_or_string_t`: a string name (see OVPHYSX_ATTR_* in
     * ovphysx_attributes.h) OR an interned token (e.g. from the optional query
     * result, fed straight back with no token-to-string round-trip). Iterate
     * the result with @ref ovphysx_fetch_read_next,
     * release each consumed group with @ref ovphysx_release_group, and release the
     * session with @ref ovphysx_release_read. Names not produced by the queried type
     * are skipped and reported in the optional summary. Duplicate canonical requests
     * and unresolvable tokens fail the task. A non-NULL `string.ptr` takes precedence and
     * the token is ignored; a NULL `string.ptr` selects the token.
     * Already-known backend failures fail open without publishing a session.
     *
     * ## Requested name vs. emitted `group.columns[0].attribute`
     *
     * The name passed here is the SEMANTIC name requested (e.g. "position"). The
     * resulting group's column `attribute` token is the name actually EMITTED, which is the
     * name that feeds the ovstage write path verbatim, and the two are not always
     * spelled identically. A single-prim "position"/"orientation" request on a
     * point-instancer is emitted as the instancer-array attribute
     * "positions"/"orientations" (singular to plural) in INSTANCER-LOCAL space, since
     * that is what writes back onto the instancer prim. Deformable/particle "points"
     * are emitted mesh/set-local. Velocities stay world-space. Request by intent and
     * write back using `columns[0].attribute` (+ `columns[0].semantic` for the coordinate role).
     * Do not assume the emitted token equals the requested string.
     *
     * ## Device columns, and when they are ready
     *
     * On a DirectGPU scene, simulated state columns are **device-resident**: those tensors
     * carry `kDLCUDA` and their `data` pointer is device memory, not host. That covers
     * rigid bodies, point-instancer instances, articulation links, whole articulations
     * and joint DOFs, and the point-set families (deformable bodies and particle sets,
     * whose `points` and `velocities` are device columns). The list is illustrative, not
     * exhaustive: residency is a property of the COLUMN, not of the type.
     *
     * Host columns persist alongside them (whole-articulation shape/material columns,
     * deformable `restPoints` and element indices, per-shape properties), so ONE READ can
     * contain both host and CUDA groups. A CPU scene emits host columns throughout.
     *
     * A GROUP, however, is device-UNIFORM: every tensor in it is on the same device. The
     * group carries one device ordinal and one CUDA context for all of its tensors, and its
     * single `cuda_sync` event describes that one device's work. There is no per-tensor
     * completion metadata, so a mixed-device group could not state when its host and device
     * halves were ready. Mixing therefore happens BETWEEN groups, never within one.
     *
     * Check `tensors[i].device.device_type`. Do not assume it, and do not infer it from the
     * object type. Reading tensor 0's device is sufficient to classify a group because of
     * the uniformity above.
     *
     * A device column is handed over **before its producing work has necessarily
     * completed**. `group.columns[0].data.cuda_sync.wait_event` is a CUDA event the producer work
     * signals, and `cuda_sync.stream` is 0 (the read never asks a consumer to drain a
     * stream). A consumer on the stream the read was given, when that stream is in the
     * scene's CUDA context -- for a zero `stream`, the legacy default/null stream in the
     * producing CUDA context -- is already ordered after that work. A consumer using any
     * other stream, including the read's own stream when it belongs to another context, must
     * make that stream wait on `wait_event` first, or it may observe a partially
     * written column: call CUDA's `cuStreamWaitEvent` or `cudaStreamWaitEvent` with
     * the nonzero event. An event of 0 needs no wait. The event belongs to the read
     * session and is valid until the session is released. Do not destroy it.
     *
     * ## The caller's stream, and capture
     *
     * A DirectGPU device group of rigid bodies, links, whole articulations, joints, tendons,
     * point instancers, deformables or particle sets is gathered on the device, like a CUDA
     * library call on `cuda_sync.stream`: after everything queued on it and after
     * `cuda_sync.wait_event`, and everything queued on it afterwards runs after the gathers.
     * The host never waits, except once after a rigid body is enabled or disabled, a centre
     * of mass changes or a point instancer moves: the next read that picks up the change first
     * waits for the device reads and commits still queued. Uploading a row list, which a read
     * does only when its rows change, waits on the host for the copy, and so for the work
     * queued on every blocking stream of the scene's CUDA context. The read's PhysX
     * copies run after the work queued on `stream` before the read, and the next
     * @ref ovphysx_step waits for those copies, so it also waits for that work. A zero `stream`
     * gathers on the legacy default stream. A `stream` of another CUDA context than the
     * scene's is waited for on the host, and the gathers then run on the scene's own stream.
     * Every other group waits for `cuda_sync` on the host.
     *
     * A read over a held query keeps its device columns for the query's next read: once it is
     * released, the next read with the same attributes and rows gathers into the same columns,
     * after the gathers before it. A read of other attributes or rows, or over another query,
     * may take them too once the query drops them. So before releasing a read, finish the work
     * that consumes its columns, or queue it on the stream later reads are given. For the next
     * read over the query with the same attributes and rows, a `cuda_sync.wait_event` recorded
     * after that work is enough.
     *
     * So the read can be captured into a CUDA graph and replayed every step. A captured group's
     * `cuda_sync` names no event, and each replay refreshes the same columns from the state
     * current when it runs. A `cuda_sync.wait_event` passed during the capture must be recorded
     * in that capture: a wait on an event recorded outside it invalidates the capture. Release
     * reads and queries outside any capture: a release may wait on the host and free device
     * memory, which invalidates a capture.
     *
     * **Before capturing:** read once outside the capture over the held query, with the same
     * attributes and rows, and release that read. Then pass the capturing stream, and wait for
     * this task before ending the capture.
     *
     * **Refused when** (before anything is queued; the capture stays valid):
     * - the query holds no plan for the read: no released read with the same attributes and
     *   rows, rows that changed since, or another open read over the query (a graph captures one
     *   read per query);
     * - it holds a host-resident or contact group;
     * - its `stream` is of another CUDA context;
     * - it would use the legacy default stream while a blocking stream of its context is being
     *   captured;
     * - the NVIDIA driver supports a CUDA version older than 12.5, which capture needs.
     *
     * **Capture again after:**
     * - any change that invalidates a captured commit (see @ref ovphysx_commit_group); for a
     *   point-instancer read, creating or removing any object, a material included;
     * - a deformable body or particle set moves, or the set of kinematically driven deformable
     *   bodies changes;
     * - a gravity change, for centre-of-mass and inverse-dynamics columns.
     *
     * A replay does not detect these changes.
     *
     * **While replaying:**
     * - keep the read open for as long as the graph is replayed. Releasing it does not wait,
     *   unless another read over the query was released while it was open: it then waits on
     *   the host for the last replay;
     * - the query's next read waits for the last replay's gathers, not for the rest of the
     *   graph: before a read on another stream than the replay's, finish the replay or pass a
     *   `cuda_sync.wait_event` recorded after the launch;
     * - replay only while no step is in flight, and finish each replay before the next
     *   @ref ovphysx_step and before any change listed above: a replay after the views were
     *   rebuilt may read freed memory;
     * - finish each replay before destroying the graph: destroying it while a replay runs
     *   can block the host until the replay ends;
     * - centre-of-mass and inverse-dynamics columns may use the link poses of the last step or
     *   kinematic refresh: after writing joint positions or root poses outside the graph, call
     *   @ref ovphysx_update_articulations_kinematic outside any capture, on the stream the graph
     *   is replayed on, before the replay;
     * - an ACTIVE-scope read replays the membership it was captured with.
     *
     * Release order: release each group with @ref ovphysx_release_group once it has been
     * read, then the session with @ref ovphysx_release_read. A device column's memory
     * belongs to the session, so it must not be read after @ref ovphysx_release_read.
     *
     * ## Step-first for device-sourced types
     *
     * The covered types require at least one simulation step on a DirectGPU scene:
     * PhysX sizes its GPU-side structures during that step and rejects direct-GPU
     * reads until it has run. A pre-step read therefore emits **no groups** for those
     * types, and @ref ovphysx_fetch_read_next reports end-of-iteration immediately. This
     * is not an error. The optional read summary reports
     * OVPHYSX_READ_OMISSION_NOT_READY for these omitted partitions. Step once before
     * reading. Query and read summaries still count matching prim identities.
     * Readiness is per scene: a multi-scene read can return groups for
     * ready scene partitions while omitting an unready DirectGPU scene's partition.
     * CPU articulation-root and articulation-joint records can likewise be discoverable
     * while buffered attach work is still inserting the SDK articulation into its scene.
     * Those pending partitions are omitted normally, then become readable after insertion.
     *
     * With an empty `cuda_sync`, the read may queue behind earlier work. With a
     * stream or event, it waits for earlier work and runs on the calling thread.
     *
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param attributes Array of `attribute_count` `ovx_token_or_string_t` (string
     *      name or interned token per entry).
     * @param attribute_count Number of attributes; zero opens an empty read session.
     * @param cuda_sync The caller's producer handoff: the read observes everything queued on
     *      `cuda_sync.stream` and after `cuda_sync.wait_event` (see "The caller's stream, and
     *      capture"). Unlike ovstage's `ovstage_cuda_sync_t` ("0 = none"), `stream` is the
     *      stream the read's device copies run on, as `ovstage_placement_t::stream`: a zero
     *      `stream` is the legacy default stream.
     * @param out_read [out] Receives a nonzero read-session handle when the call is
     *      accepted, before it returns; zero when the call is rejected.
     * @param out_summary Optional immutable open summary; NULL skips summary work.
     *      Nested arrays remain valid until accepted read release or instance
     *      destruction. They describe this open even after detach;
     *      do not resolve their tokens through an invalidated attachment's dictionary.
     *      Omission bits classify missing partitions; backend failure fails the task instead.
     * @return Task receipt; read construction failures are reported by waiting.
     *
     * @pre handle and query are valid. Keep any out_summary slot alive until task
     *      completion; the attributes and their strings are not read after the call
     *      returns. The read observes every step submitted before it.
     * @post `*out_read` is nonzero once the call is accepted. After a failed task it
     *       stays known but unusable, as a query's does. CUDA data readiness still
     *       requires the group's event handoff.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid, out_read is null,
     *   attributes is null with a nonzero count, or a token-less attribute string
     *   is null with a nonzero length.
     * - OVPHYSX_API_NOT_FOUND if query is absent, released, or belongs to
     *   another instance.
     * - The rejections every short call shares (see @ref ovphysx_query).
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and
     * @ref ovphysx_wait_task reports as OVPHYSX_API_ERROR, with the text
     * "ovphysx_read (task <index>): <cause>": an unavailable read backend, a query
     * whose producer failed, a failed attach or detach submitted before it, and read
     * construction failures.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_read(ovphysx_instance_handle_t handle,
                                                      ovphysx_query_handle_t query,
                                                      const ovx_token_or_string_t* attributes,
                                                      size_t attribute_count,
                                                      ovstage_cuda_sync_t cuda_sync,
                                                      ovphysx_read_handle_t* out_read,
                                                      ovphysx_read_result_t* out_summary);

    /**
     * @brief Fetch the next output column group from a read session.
     *
     * After successful task completion, `*out_group` points at the next
     * `ovstage_read_group_t`, or is NULL at end of iteration. Each group carries one
     * attribute column, `columns[0]` (`column_count == 1`). End of iteration is
     * an ordinary successful task. A failed task leaves `*out_group` NULL and must
     * not be treated as end of iteration. Keep out_group alive until completion.
     *
     * The group is **producer-owned** (the caller does not allocate it): the returned
     * `ovstage_read_group_t` pointer is a borrow valid until @ref ovphysx_release_group
     * is called for that group's `read_group_id`, or the session is released.
     *
     * Group lifetime (authoritative): the returned group struct and its stage-derived
     * `prims.list` stay valid until @ref ovphysx_release_group for that `read_group_id`
     * (or @ref ovphysx_release_read, which releases all). The column and its numeric
     * storage are owned by the read session instead: the `columns[0]` descriptor, its
     * `data.tensors`, every tensor's shape and data, `prims.index_map`, `data.index_map`,
     * `data.mask`, and `data.cuda_sync.wait_event` stay valid until
     * @ref ovphysx_release_read. Fetching further groups and an intervening
     * @ref ovphysx_step do NOT invalidate either lifetime (the runtime gathers each
     * column into session-owned storage at read time). Numeric snapshots also
     * survive stage detach until session release or instance destruction. Detach
     * invalidates the stage dictionary and its prim-list handles; do not resolve
     * those handles after detaching.
     *
     * A device (`kDLCUDA`) column additionally has a readiness contract: it is handed
     * over before its producing work has necessarily completed. Wait on
     * `group.columns[0].data.cuda_sync.wait_event` before reading it from any stream other than the one
     * the read was given in the scene's CUDA context, using CUDA's `cuStreamWaitEvent` or
     * `cudaStreamWaitEvent`. See @ref ovphysx_read for the full rule. Validity and readiness
     * are separate: the borrow above means the memory may still be read, the event means the
     * values in it are final.
     *
     * A failure already known during gathering fails read creation. A later fetch
     * can still fail, for example if the attachment was invalidated. Check task
     * completion before interpreting a NULL group as the end of iteration.
     * Already-fetched numeric snapshots retain their session lifetime.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle from @ref ovphysx_read.
     * @param out_group [out] Receives a borrowed `ovstage_read_group_t*` for the next
     *      group (NULL at end of iteration or on error).
     * @return Task receipt. After successful task completion, NULL means done.
     *
     * @pre handle and read are valid and out_group is non-null.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid or out_group is null.
     * - OVPHYSX_API_NOT_FOUND if read is absent, released, or belongs to
     *   another instance.
     * - The rejections every short call shares (see @ref ovphysx_query).
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and
     * @ref ovphysx_wait_task reports as OVPHYSX_API_ERROR, with the text
     * "ovphysx_fetch_read_next (task <index>): <cause>": an unavailable read backend,
     * a read whose producer failed, a failed attach or detach submitted before it,
     * and backend fetch failures. End of iteration is successful completion with a
     * NULL group.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_read_next(ovphysx_instance_handle_t handle,
                                                         ovphysx_read_handle_t read,
                                                         const ovstage_read_group_t** out_group);

    /**
     * @brief Release one fetched group's borrowed storage.
     *
     * Releases the stage-derived prim-list storage pinned by
     * @ref ovphysx_fetch_read_next for `group_id` (an
     * `ovstage_read_group_t::read_group_id`). After this the group struct and
     * `prims.list` must not be dereferenced. Tensor, prim-index-map, data-index-map,
     * mask, and CUDA-event storage belongs to the read session and remains valid until
     * @ref ovphysx_release_read.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle the group came from.
     * @param group_id The group's `ovstage_read_group_t::read_group_id`.
     * @return Task receipt. Accepted release ends the group borrow immediately.
     *         An already-released or unknown id, and a read that is unknown, released
     *         or whose producer failed, are successful no-ops; a read of another
     *         instance is rejected with OVPHYSX_API_NOT_FOUND. A short call (see
     *         @ref ovphysx_query): it runs after the work submitted before it, and an
     *         unavailable release backend fails its task.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_group(ovphysx_instance_handle_t handle,
                                                       ovphysx_read_handle_t read,
                                                       ovstage_read_group_id_t group_id);

    /**
     * @brief Release a read session (and every borrowed group it still owns).
     *
     * Finish all external CPU and CUDA consumers before submitting release or
     * destroying the instance, or queue them on the stream later reads are given (see
     * @ref ovphysx_read). The
     * SDK reclaims device storage only after its own producing work; it has no
     * completion token for work launched by the caller. Columns kept on a held query
     * for its next read are ordered after that work on the device, without a host
     * wait, unless another read over the query was released since this one opened: they
     * are then freed after a host wait. Release reads outside any capture. For a read
     * captured into a CUDA graph, the query's next read waits on the device for its
     * last replay's gathers, not for the rest of the graph. Accepted release
     * ends all session borrows and transfers cleanup responsibility to the SDK.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle from @ref ovphysx_read.
     * @return Task receipt. The release is accepted when it is submitted, so a
     *         later call that names the handle is rejected, and it runs after the
     *         work submitted before it, also when the read is still queued. An absent
     *         or released handle is a successful no-op for a valid instance, and so is
     *         one whose producer failed; a handle of another instance is rejected with
     *         OVPHYSX_API_NOT_FOUND. Wait to observe cleanup failures; do not retry
     *         accepted releases.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_read(ovphysx_instance_handle_t handle,
                                                      ovphysx_read_handle_t read);

    /**
     * @brief Open a write session for an array of named attributes.
     *
     * The application-to-physics direction, the mirror of @ref ovphysx_read. It reuses
     * that query verbatim: the session reaches exactly the set `query` matched. A commit
     * writes every row of its group, or with @ref ovphysx_commit_group_masked the rows a
     * per-commit mask selects; there is no index or prim-list parameter.
     *
     * Each fetched group identifies its original request index through
     * ovphysx_fetched_write_group_t::attribute_index, even when attributes have
     * identical tensor shapes. Attributes are planned in request order. Commit
     * order is chosen by the caller and no transactional rollback is provided.
     * All names are validated before publishing a session. Canonical duplicates,
     * including string/token aliases, and unsupported/read-only writes fail the
     * creation task. A valid request with no matching objects is an empty session.
     * A non-NULL `string.ptr` takes precedence and the token is ignored. A token-only
     * request (NULL `string.ptr`) must belong to the owning Stage dictionary; an
     * unresolvable token fails the creation task.
     *
     * The write's attribute vocabulary is its own: a name accepted here need not be one
     * @ref ovphysx_read emits, since write-only control inputs such as forces and
     * wrenches have no read counterpart. Request attributes by semantic name; each
     * fetched group's request index identifies that name even when its storage is
     * an instancer array.
     *
     * ## Column residency
     *
     * A group's tensor is normally resident where the scene simulates: `kDLCUDA` on a
     * DirectGPU scene, `kDLCPU` otherwise. Two families are `kDLCPU` on EVERY scene,
     * because their destination is host memory rather than a device buffer: the
     * articulation DOF properties (drive, limit, friction), and particle `points` /
     * `velocities`, which land in the set's staging arrays for PhysX to upload at the
     * next step. Check `tensors[i].device.device_type` and fill accordingly. Do not
     * infer it from the scene. Handing a host pointer to a device column, or the
     * reverse, faults asynchronously and is reported far from here.
     *
     * A session may be opened on a scene that has not yet stepped. CPU and
     * GPU-with-readback apply a pre-step write; DirectGPU refuses at
     * @ref ovphysx_commit_group, not here.
     *
     * On a DirectGPU scene, opening a session uploads its row lists and waits on the host
     * for the copy, and so for the work queued on every blocking stream of the scene's CUDA
     * context.
     *
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param attributes Nonempty array of string-or-token attributes.
     * @param attribute_count Number of attributes.
     * @param out_write [out] Receives the write-session handle, nonzero, when the call
     *      is accepted, before it returns; zero on rejection.
     * @param out_summary Optional immutable open summary; NULL skips summary work.
     *      Nested arrays remain valid until accepted write release, stage detach
     *      or instance destruction, whichever occurs first.
     * @return Task receipt; write construction failures are reported by waiting.
     *
     * @pre handle and query are valid. Keep output slots alive until completion; the
     *      attributes and their strings are not read after the call returns.
     * @post `*out_write` is nonzero once the call is accepted. After a failed task it
     *       stays known but unusable, as a query's does. A detach closes the write
     *       sessions opened before it and retires their groups: a call submitted after
     *       the detach that names a closed session fails its task ("input write
     *       <handle> was closed when its stage was detached"), whether or not the
     *       detach already ran, and its release succeeds. Finish external work
     *       filling those groups before any such teardown.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid, out_write or attributes is
     *   null, attribute_count is zero, or a token-less string is null with a nonzero length.
     * - OVPHYSX_API_NOT_FOUND if query is absent, released, or belongs to
     *   another instance.
     * - The rejections every short call shares (see @ref ovphysx_query).
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and
     * @ref ovphysx_wait_task reports as OVPHYSX_API_ERROR, with the text
     * "ovphysx_write (task <index>): <cause>": an unavailable write backend, a query
     * whose producer failed, a failed attach or detach submitted before it, and write
     * construction failures, including unsupported attributes or scene pipelines.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_write(ovphysx_instance_handle_t handle,
                                               ovphysx_query_handle_t query,
                                               const ovx_token_or_string_t* attributes,
                                               size_t attribute_count,
                                               ovphysx_write_handle_t* out_write,
                                               ovphysx_write_result_t* out_summary);

    /**
     * @brief Fetch the next writable group from a write session.
     *
     * After successful task completion, `out_group->group` points at the next
     * `ovstage_map_group_t`, or is NULL at end of iteration. End of iteration is
     * an ordinary successful task. A failed task leaves `out_group->group` NULL and must
     * not be treated as end of iteration. Keep out_group alive until completion.
     *
     * The group is **producer-owned** (the caller does not allocate it) and handed back
     * as a `const` borrow. `const` is correct even on the write path: the group is a
     * DESCRIPTOR the caller reads, describing BUFFERS the caller fills through
     * `data.tensors[i].data`, which `const` permits since it does not propagate through
     * pointer members. No field of the struct is the caller's to assign, and tensor
     * shape, dtype and device are dictated by the implementation.
     *
     * Group storage stays valid until that group is committed or its write session
     * is released, independent of further fetches. Detach and instance
     * destruction retire write sessions. An intervening @ref ovphysx_step does not
     * invalidate a live group. A later write over the same query may hand out the same
     * device storage again for the same attribute, scene and row count. A DirectGPU
     * device group then names, in `data.cuda_sync.wait_event`, the event the commit that
     * last used that storage recorded: make the stream filling the group wait on it, before
     * beginning the capture when the fill is captured -- a capture cannot wait on an event
     * recorded outside it.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle from @ref ovphysx_write.
     * @param out_group [out] Receives the borrowed group and original request index.
     *      EOF is group == NULL and attribute_index == SIZE_MAX after successful
     *      task completion. Failed completion is never EOF.
     * @return Task receipt. After successful task completion, NULL means done.
     *
     * @pre handle and write are valid and out_group is non-null.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid or out_group is null.
     * - OVPHYSX_API_NOT_FOUND if write is absent, released, or belongs to
     *   another instance.
     * - The rejections every short call shares (see @ref ovphysx_query).
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and
     * @ref ovphysx_wait_task reports as OVPHYSX_API_ERROR, with the text
     * "ovphysx_fetch_write_next (task <index>): <cause>": an unavailable write backend,
     * a write whose producer failed or that a detach closed, a failed attach or detach
     * submitted before it, and backend fetch failures. End of iteration is successful
     * completion with a NULL group.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_write_next(ovphysx_instance_handle_t handle,
                                                          ovphysx_write_handle_t write,
                                                          ovphysx_fetched_write_group_t* out_group);

    /**
     * @brief Commit a filled group, transferring ownership of its data to physics.
     *
     * The caller must have filled EVERY mapped entry, so a partially filled group
     * publishes whatever its unfilled entries contain. To write a subset of the rows,
     * commit with @ref ovphysx_commit_group_masked, or query fewer prims. A live group
     * belonging to this session is spent
     * when accepted for publication, including when publication fails. Stop using
     * its mapped pointers once the submission is accepted. A rejected submission
     * leaves ownership with the caller. An invalid, foreign, or already-spent
     * pointer fails the task without publishing or consuming another session's group.
     * Dereferencing them is undefined behavior rather than a checked error, since a raw
     * `data.tensors[i].data` access makes no API call the runtime could reject.
     *
     * ## Committed by pointer, not by id
     *
     * `ovstage_map_group_t` carries no `write_group_id` (the read's `read_group_id` has
     * no twin on it), so the borrowed pointer IS the commit identity. This is the one
     * place this API differs from @ref ovphysx_release_group, which takes an id. Group
     * addresses are unique for the session's life and never recycled between groups,
     * which is what lets an already-committed group be told apart from a live one.
     *
     * ## Synchronization
     *
     * `write_done_sync` is ovstage's `{stream, wait_event}` handoff of producer
     * completion. How it is honored depends on where the group is applied:
     *
     * - A **DirectGPU device group** -- rigid-body pose, velocity, force and wrench, link
     *   force and wrench, articulation root, joint state and drive, tendon and
     *   point-instancer columns -- is ordered on the device, like a CUDA library call on
     *   `stream`: its work runs after everything queued on `stream` and after
     *   `wait_event`, and everything queued on `stream` afterwards runs after it. The
     *   host never waits, except once after a rigid body is enabled or disabled, a centre
     *   of mass changes or a point instancer moves: the next call that picks up the change
     *   first waits for the device commits still queued. So the commit can be captured
     *   into a CUDA graph: pass the capturing stream, wait for this task before ending the
     *   capture, and keep the write session open for as long as the graph is replayed
     *   (releasing it ends replay). Replay only while no step is in flight, and finish
     *   each replay before the next @ref ovphysx_step. Capture again after prims are added
     *   or removed, physics APIs are applied or removed, an edit makes physics re-create an
     *   object, the scene is reset or replaced, any rigid body is enabled or disabled, the
     *   centre of mass of a body whose wrench the graph writes changes, or a point instancer
     *   whose instances the graph writes, or one of its ancestors, moves: a replay does not
     *   detect them and writes wrong values or to the wrong bodies. Finish each replay before
     *   such a change. A zero
     *   `stream` orders the work on the legacy default stream. A `stream` of another
     *   CUDA context than the scene's is waited for on the host instead, and the work
     *   then runs on the scene's own stream: Warp and PyTorch streams share the scene's
     *   context only when the instance was created on an explicit CUDA device
     *   (`active_cuda_gpus`). A fault in the caller's own producer work is not detected
     *   here; it surfaces at the next synchronization.
     * - **Every other group** is applied on the host. A non-zero `wait_event` and a
     *   non-zero `stream` are both waited for on the host before the data is consumed.
     *   Only `{0, 0}` asserts nothing is outstanding. A host-resident group does NOT
     *   imply `{0, 0}`, since the caller may have staged on its own stream. Such a group
     *   is refused at admission on a stream being captured, before anything is read: the
     *   group stays live, to commit outside the capture.
     *
     * While a blocking stream of a CUDA context is being captured, that context's legacy
     * default stream cannot be used: it synchronizes with the capturing stream, so CUDA
     * refuses work on it, and an attempt would invalidate the capture. A commit given it
     * -- `stream` `1`, or a zero `stream` for a group in device memory -- is refused at
     * admission, before anything is read: the capture stays valid and the group stays
     * live.
     *
     * CUDA graph capture needs an NVIDIA driver for CUDA 12.5 or newer. On an older driver, a
     * commit on a stream being captured is refused at admission the same way.
     *
     * Context-relative default-stream sentinels refer to the producer context current
     * when this call is made. Invalid stream, event or context combinations fail the
     * task. If producer completion cannot be proven, the SDK retains the affected
     * storage for safe cleanup; the group remains spent. A commit with nothing
     * outstanding (`{0, 0}`) of a session that never handed out device memory is a
     * short call (see @ref ovphysx_query). Every other commit runs on the calling
     * thread once earlier work on the instance has finished, and enqueues its device
     * work during the call. Successful task completion confirms the CPU mutation or
     * GPU submission; GPU execution may continue through the backend event chain.
     * It does NOT warm up a scene that has not stepped. Device modes differ:
     * on CPU and on GPU-with-readback a pre-step write **commits and is applied**; on
     * DirectGPU (`eENABLE_DIRECT_GPU_API`) commit is **refused** until a first step has
     * sized the GPU scatter view. Call @ref ovphysx_warmup or @ref ovphysx_step for a
     * recipe that works on every mode. The write never steps on the caller's behalf.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle the group came from.
     * @param group The borrowed group from @ref ovphysx_fetch_write_next.
     * @param write_done_sync CUDA handoff. `{0, 0}` when nothing is outstanding.
     *      Keep every supplied stream and event alive until task completion, and
     *      make the producer context current when using a context-relative stream.
     * @return Task receipt; backend refusal or mutation failure is reported by waiting.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid or group is null. For a commit
     *   that runs on the calling thread (a CUDA handoff, or a session that handed out
     *   device memory), also when the group is applied on the host and
     *   `write_done_sync.stream` is being captured into a CUDA graph, the commit would
     *   use the legacy default stream while a blocking stream of its context is being
     *   captured, the driver cannot tell whether `write_done_sync.stream` is being
     *   captured, or `write_done_sync.stream` is being captured on a driver older than
     *   CUDA 12.5. The group stays live.
     * - OVPHYSX_API_NOT_FOUND if write is absent, released, or belongs to
     *   another instance.
     * - The rejections every short call shares (see @ref ovphysx_query); a commit that
     *   runs on the calling thread has them too, apart from the full queue.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS, and
     * @ref ovphysx_wait_task reports as OVPHYSX_API_ERROR, with the text
     * "ovphysx_commit_group (task <index>): <cause>": an unavailable write backend, a
     * write whose producer failed or that a detach closed, a failed attach or detach
     * submitted before it, an invalid, foreign, or already-spent group, an unstepped
     * DirectGPU scene, or a publication failure.
     * CPU and GPU-with-readback permit pre-step commits. Committing a spent group
     * fails rather than claiming data was published again.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_commit_group(ovphysx_instance_handle_t handle,
                                                      ovphysx_write_handle_t write,
                                                      const ovstage_map_group_t* group,
                                                      ovstage_cuda_sync_t write_done_sync);

    /**
     * @brief Commit only the rows of a group a mask selects.
     *
     * Behaves as @ref ovphysx_commit_group, except that only the rows `row_mask` selects
     * are written; the column's other entries need not be filled. The counterpart of the
     * deprecated @ref ovphysx_write_tensor_binding_masked for the session write.
     *
     * `row_mask` is a contiguous 1-D bool or uint8 tensor with one entry per row of the
     * group, nonzero selecting the row. A group's rows are the leading dimension of its
     * tensor -- one per DOF for an articulation-joint group, so a mask selects DOFs -- or
     * its tensors when it carries one per prim (deformable-body groups); a point-instancer
     * group's rows are its instances. Character controller,
     * vehicle and particle groups take no mask. The mask lives where the group does: on
     * the group's CUDA device, or in host memory for a host-resident group. A device group
     * reads its mask on the device when the commit runs, so a captured commit reads the
     * mask's current contents at every replay.
     *
     * Keep the mask valid until the commit has read it: for a DirectGPU device group, until
     * the work queued on the commit's stream has run -- task completion means only that it
     * was submitted -- and, when the commit is captured into a CUDA graph, for as long as
     * the graph may be replayed. A host group has read it once its task completes.
     *
     * A masked commit is never a short call: it always runs on the calling thread once
     * earlier work on the instance has finished, where the runtime checks the mask
     * against the group before anything is read. A mask it refuses rejects the call
     * the same way whether or not the instance was busy, and the group stays live.
     *
     * An unselected row keeps its value, with one exception: PhysX cannot read an
     * applied load back, so an unselected rigid-body or link `force` or `wrench` row
     * applies none. On a DirectGPU scene that also replaces a load another write set
     * on that row earlier in the same step; elsewhere loads add up. On a DirectGPU scene
     * an articulation root, joint or tendon commit also writes its unselected articulations
     * back with their current values: until the next step they read zero link accelerations
     * and incoming joint forces, and a root whose centre of mass is offset can drift
     * slightly with each such commit.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle the group came from.
     * @param group The borrowed group from @ref ovphysx_fetch_write_next.
     * @param row_mask One entry per row, nonzero selecting the row.
     * @param write_done_sync CUDA handoff, as for @ref ovphysx_commit_group; the mask's
     *      producer is part of it.
     * @return Task receipt; backend refusal or mutation failure is reported by waiting.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid, group or row_mask is null,
     *   row_mask is not a 1-D bool or uint8 tensor, its length, layout or device does
     *   not match the group, the group is a character controller, vehicle or particle
     *   group, the group is applied on the host and `write_done_sync.stream` is being
     *   captured, the commit would use the legacy default stream while a blocking
     *   stream of its context is being captured, or the driver cannot tell whether
     *   `write_done_sync.stream` is being captured. The group stays live.
     * - OVPHYSX_API_NOT_FOUND if write is absent, released, or belongs to
     *   another instance.
     * - The rejections every short call shares (see @ref ovphysx_query), apart from
     *   the full queue.
     *
     * Every other failure is reported as for @ref ovphysx_commit_group.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_commit_group_masked(ovphysx_instance_handle_t handle,
                                                             ovphysx_write_handle_t write,
                                                             const ovstage_map_group_t* group,
                                                             const DLTensor* row_mask,
                                                             ovstage_cuda_sync_t write_done_sync);

    /**
     * @brief Release a write session, discarding anything uncommitted.
     *
     * Uncommitted groups are discarded. Finish all external CPU and CUDA work that
     * fills those groups before submitting release: this call takes no producer
     * completion token. Committed groups are not rolled back; the SDK waits for its
     * own device work before reclaiming their storage. For a commit captured into a
     * CUDA graph, that covers the replays launched before the release: never replay
     * such a graph after its session is released. Once release is accepted,
     * all session borrows end and cleanup belongs to the SDK, including after failure.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle from @ref ovphysx_write.
     * @return Task receipt. Accepted and run as for @ref ovphysx_release_read; a
     *         session that a detach closed, or whose producer failed, is released too.
     *         An absent or released handle is a successful no-op for a valid instance; a handle of
     *         another instance is rejected with OVPHYSX_API_NOT_FOUND. Wait to observe
     *         cleanup failures; do not retry accepted releases.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_write(ovphysx_instance_handle_t handle,
                                                       ovphysx_write_handle_t write);

    /**
     * @brief Ask whether an attribute can be written on an object type, and how.
     *
     * The programmatic answer to a question that is otherwise only documented in prose,
     * keyed on the SAME (object type, attribute) the write path takes (@ref ovphysx_write
     * resolves the object type through its query and names the attribute). This reports the
     * API's supported read/write vocabulary. Its scene-independent classification
     * table is maintained separately and tested against runtime session admission.
     *
     * Scene-independent: writability is a property of the write API, not of any live scene,
     * so this needs no instance, query or step. A name the object type does not accept comes
     * back OVPHYSX_WRITABILITY_UNCLASSIFIED, and @ref ovphysx_write rejects it. Such a name
     * is a gap to close, never an invitation to try the write anyway.
     *
     * @param object_type Object type the attribute would be written on (@ref ovphysx_query's
     *   selector), e.g. OVPHYSX_QUERY_TARGET_RIGID_BODY.
     * @param attribute Attribute name (see the OVPHYSX_ATTR_* macros in
     *   ovphysx_attributes.h), as the string field of an
     *   `ovx_token_or_string_t`. STRING-ONLY: an interned-token-only key is rejected, because
     *   resolving a token needs a path dictionary this scene-free query does not have.
     * @param out_writability [out] Receives the classification.
     * @return ovphysx_result_t.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_writability or attribute is null, if attribute
     *   carries no string name (a token-only key), or if object_type is outside
     *   @ref ovphysx_query_target_t (so an unknown type is distinguishable from a valid type
     *   that does not accept the name, which returns OVPHYSX_WRITABILITY_UNCLASSIFIED)
     */
    OVPHYSX_API ovphysx_result_t ovphysx_writability(ovphysx_query_target_t object_type,
                                                     const ovx_token_or_string_t* attribute,
                                                     ovphysx_writability_t* out_writability);

    /**
     * @brief Release an output query.
     *
     * Release it outside any CUDA graph capture: it frees the device columns the query
     * keeps, after a host wait for their last use.
     *
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @return Task receipt. Accepted and run as for @ref ovphysx_release_read; a
     *         read or write submitted after the release that names the query is
     *         rejected. An absent or released handle is a successful no-op for a valid
     *         instance, and so is one whose producer failed; a handle of another
     *         instance is rejected with OVPHYSX_API_NOT_FOUND. Wait to observe cleanup
     *         failures; do not retry accepted releases.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_query(ovphysx_instance_handle_t handle,
                                                       ovphysx_query_handle_t query);

    /**
    * @brief Clone the subtree under the source path to one or more target paths in the
    * internal physics representation (USD untouched).
    * The source path must exist in the stage.
    * The target paths must not already have runtime physics objects.
    *
    * Clones are created in the internal representation only (the USD file is not modified)
    * and immediately participate in physics simulation. Backed by the PhysX SDK replicator, so
    * cloned articulations are real articulations. Optimized for RL mass replication (1000s of
    * instances). This is the clone entrypoint for both standalone callers and callers using an
    * ovstage Stage (@ref ovphysx_attach_ovstage).
    * If other consumers need stage copies too, complete the attach first, then
    * call ovstage_clone() separately for the same source and targets.
    * Use a dedicated ordinal, seal it, and exclude it from every
    * @ref ovphysx_update_from_ovstage range. Forwarding
    * that ordinal can ingest the copied physics data as new scene objects;
    * it does not invoke the PhysX replicator.
    *
    * @note Cross-environment collision isolation uses PhysX environment ids under GPU dynamics +
    *       GPU broadphase, controlled by `/ovphysx/clone/useEnvIds` (default on, per-process). The
    *       source holds env id 0 and clones get positive ids, so co-located clones (NULL anchor_transforms)
    *       are isolated from the source too. Environment ids do not isolate clones in CPU mode:
    *       co-located CPU clones share one collision space, so use spatially disjoint
    *       anchor_transforms. The runtime emits a warning through the Carbonite log stream when
    *       env ids are requested but GPU dynamics or GPU broadphase is unavailable. Register
    *       ovphysx_set_log_callback() to receive it programmatically. USD collision
    *       groups/filtering authored before cloning still work for finer control. Pass `env_ids`
    *       when one logical environment is assembled from several clone calls, so its objects
    *       share an id.
    *
    * @param handle PhysX instance handle
    * @param source_path_in_usd Path to the source subtree to clone (must exist)
    * @param target_paths Array of target paths with no existing runtime physics objects
    * @param num_target_paths Number of target paths to clone to
    * @param anchor_transforms Absolute world pose of each target subtree root. Entry i anchors
    *        the exact subtree at target_paths[i]. Flat [num_target_paths x 7] floats:
    *        (px, py, pz, qx, qy, qz, qw), quaternion
    *        imaginary-first (OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, identity = (0,0,0,1)).
    *        Descendants keep their poses relative to the source subtree root
    *        (target_object_world = anchor_transforms[i] * inverse(source_root_world) *
    *        source_object_world). Pass NULL to co-locate every copy on the source. Co-location is
    *        collision-isolated only under GPU dynamics + GPU broadphase. Use spatially disjoint
    *        transforms in CPU mode.
    * @param env_ids Optional logical environment id per target ([num_target_paths] uint32).
    *        Stable across calls: a shared id maps to one runtime environment (clones sharing it
    *        collide, isolated from others). Ids must be < 0x00FFFFFF (runtime id is env_ids[i] + 1).
    *        Pass NULL for automatic numbering from a per-attachment base. A later explicit
    *        id can reuse an earlier automatic id without warning or error: on a fresh
    *        attachment with env-id filtering active, one automatic target gets runtime id 1
    *        and a later explicit id 0 also maps to 1. Reversing the calls gives the automatic
    *        target id 2 instead. Choose one ID strategy for an attachment; use explicit ids
    *        consistently when controlling grouping across calls.
    *
    * @return Admission status and task identity. Rejection has zero identity;
    * accepted execution failure is reported by a wait. On rejection,
    *         call ovphysx_get_last_error() on the same thread for the error message.
    *
    * @note The call queues the replication as a job on the instance's worker thread
    *       and returns without running it or waiting for it; the worker runs it after
    *       every earlier job, and may even finish it before the call returns. An
    *       instance without a worker thread (see
    *       OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION) runs it on the calling thread
    *       before the call returns. Collect its outcome with ovphysx_wait_task() or
    *       ovphysx_wait_all().
    *
    * @pre handle must be valid.
    * @pre source_path_in_usd must exist and target_paths must be valid and unique.
    * @pre Must be called **before** ovphysx_warmup() or the first simulation step,
    *      in submission order. Multiple ovphysx_clone() calls per attach are allowed
    *      in CPU and GPU mode while all of them precede warmup / first step. To clone
    *      after that point, use ovphysx_detach_ovstage() and reattach the source stage
    *      first. On GPU, cloning later would reallocate DirectGPU buffers and corrupt
    *      initialized state.
    * @post Cloned physics objects are live when the accepted task succeeds.
    *
    * @par Side Effects
    * Adds live PhysX objects keyed by each target path. No USD or runtime-stage prims are authored.
    *
    * @par Ownership
    * The call copies source_path_in_usd, the target_paths array and its strings,
    * anchor_transforms and env_ids before it returns. The caller retains ownership
    * and may free or overwrite them as soon as the call returns.
    *
    * @par Errors
    * Rejected at submission, with zero identity and nothing queued:
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid arguments: an empty source, a null
    *   or empty target list, an empty target, an embedded NUL, a target equal to
    *   the source or repeated in the call, or an env id out of range
    * - OVPHYSX_API_NOT_FOUND for an invalid handle
    * - OVPHYSX_API_ERROR if the runtime clone interface is unavailable
    * - OVPHYSX_API_INVALID_STATE inside a log, runtime or object-change callback,
    *   or when the instance's worker is stopped or its queue is full
    *
    * Reported by the task wait: no attached ovstage, a call after warmup / the first
    * step, a target already cloned on this attach, and clone failures such as a
    * missing source or an occupied target.
    *
    * @code
    * #include <ovphysx/ovphysx.h>
    *
    * static ovphysx_enqueue_result_t clone_two_environments(ovphysx_instance_handle_t handle)
    * {
    *     ovphysx_string_t targets[2] = {
    *         ovphysx_cstr("/World/env1"),
    *         ovphysx_cstr("/World/env2"),
    *     };
    *     return ovphysx_clone(
    *         handle, ovphysx_cstr("/World/env0"), targets, 2, NULL, NULL);
    * }
    * // To assemble one environment from several calls (heterogeneous ClonePlan), pass the same
    * // env_ids in each call so its objects share a runtime environment.
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_clone(
        ovphysx_instance_handle_t handle,
        ovphysx_string_t source_path_in_usd,
        ovphysx_string_t* target_paths,
        uint32_t num_target_paths,
        const float* anchor_transforms,
        const uint32_t* env_ids
    );

    /** @} */

    /*--------------------------------------------------*/
    /* Simulation operations */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_simulation */
    /** @{ */

    /**
    * Enqueue an asynchronous physics simulation step.
    * The instance's worker thread runs the whole step after every earlier job: it
    * simulates and fetches the results, so the step completes without a wait, and its
    * pre-step and post-step events and callbacks run on the worker, or on a thread
    * that waits for the step without a deadline before the worker started it (see
    * ovphysx_wait_task()), which then runs it itself. The call runs no part of the
    * step and normally does not wait for earlier work or for the step; the worker may even
    * finish the step before the call returns. An instance without a worker thread
    * (see OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION) runs the whole step, callbacks
    * included, on the calling thread before the call returns. Collect its outcome with
    * ovphysx_wait_task() or ovphysx_wait_all().
    * The simulation time is tracked internally. Each step advances it by step_dt.
    * The timestep is passed through without clamping or substepping. The caller
    * is responsible for choosing a timestep appropriate for the simulation.
    * A buffered OmniPVD receiver request first waits for earlier work on every
    * instance, without collecting outcomes, and applies selection before this
    * step is submitted. Empty control polls do not wait for queued work.
    * @param handle ovphysx instance
    * @param step_dt Finite, nonnegative simulation timestep in seconds
    * @return ovphysx_enqueue_result_t with admission status and instance/task identity
    *
    * @pre handle must be a valid instance handle.
    * @post Simulation advances by step_dt when the op completes.
    *
    * @par Side Effects
    * Mutates physics state and may trigger internal events and callbacks.
    *
    * @par Threading
    * Must not be called concurrently on the same instance without external synchronization.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_ERROR for an invalid handle or internal failures
    * - OVPHYSX_API_INVALID_STATE inside a log, runtime or object-change callback,
    *   or when the instance's worker is stopped or its queue is full
    *
    * Failures during execution are reported by the task wait: an instance without
    * an attached Stage when the step runs ("No stage attached"; an attach submitted
    * earlier counts, so attach then step needs no wait in between), PhysX
    * CUDA-context failures, with a retained CUDA diagnostic even when logging is
    * disabled, and a simulation time that would overflow.
    *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_enqueue_result_t step_one_frame(ovphysx_instance_handle_t handle)
     * {
     *     return ovphysx_step(handle, 1.0f / 60.0f);
     * }
     * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_step(ovphysx_instance_handle_t handle,
                                                              float step_dt);

    /**
     * @brief Synchronous step: simulate one physics timestep and wait for
     * completion in a single call.
     *
     * This step does not create a task receipt. Earlier asynchronous work is
     * completed first, without collecting its outcomes. Those receipts remain
     * available to ovphysx_wait_task() or ovphysx_wait_all().
     * Buffered OmniPVD receiver requests use the shared safe point before this
     * call takes its instance lane; earlier task outcomes remain uncollected.
     *
     * Use this whenever the caller steps and immediately waits for results,
     * that is, does not overlap GPU simulation with CPU work between dispatch
     * and fetch.
     *
     * The simulation time is tracked internally. Each step advances it by
     * step_dt.
     *
     * @param handle  Physics instance handle.
     * @param step_dt  Finite, nonnegative timestep [s], passed through as in ovphysx_step().
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success.
     * A PhysX CUDA-context error returns OVPHYSX_API_ERROR; retrieve its
     * diagnostic with ovphysx_get_last_error(), even when logging is disabled.
     * A recoverable capacity-overflow report alone does not fail the call.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_instance_handle_t handle,
                                                            float step_dt);

    /**
     * Explicitly run the warmup step (optional, synchronous).
     *
     * On first use, a real simulation step with a minimal timestep (~1ns) is run to
     * initialize PhysX structures and disable per-step Fabric sync overhead. This is
     * done automatically by legacy tensor-binding reads (auto-warmup). Session
     * reads through ovphysx_read() do not auto-warm; call this explicitly to control
     * when the latency occurs.
     *
     * Works in both CPU and GPU mode. In GPU mode, this also populates DirectGPU buffers.
     *
     * IMPORTANT: The warmup advances simulation state (positions may change
     * infinitesimally). It is NOT a "dry run". For deterministic initial conditions,
     * call this before reading initial tensor state.
     *
     * This function is idempotent. Calling it multiple times has no effect after
     * the first successful call (per stage). Warmup state resets when the stage
     * changes through detach and attach.
     *
     * @param handle Instance handle
     * @return ovphysx_result_t
     *
     * @pre handle must be valid.
     * @post Warmup completed for the active stage.
     *
     * @par Side Effects
     * Advances simulation by a minimal timestep on first call. Disables per-step
     * Fabric sync (enabling direct TensorAPI mode).
     *
     * @par Errors
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t warm_up_instance(ovphysx_instance_handle_t handle)
     * {
     *     return ovphysx_warmup(handle);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_warmup(ovphysx_instance_handle_t handle);

    /**
     * Recompute articulation link transforms and/or velocities for the whole
     * articulations selected by a physics-output query, without running a normal
     * simulation step. The query must select OVPHYSX_QUERY_TARGET_ARTICULATION. A query from
     * @ref ovphysx_query_selection updates only the articulations its selection names,
     * through their root, a link or a joint; every other articulation is left unchanged.
     *
     * The update runs on the calling thread once earlier work on the instance has
     * finished, and its task has finished when the call returns: successful task
     * completion confirms the CPU update or the GPU submission. It is
     * useful after writing articulation root or DOF state (through the session write
     * API, @ref ovphysx_write, or the deprecated tensor bindings) and before reading link
     * state in the same frame. The refresh does not run collision detection,
     * integration, solver work or contact generation.
     *
     * On a DirectGPU scene the refresh is ordered on the device like a stream-ordered
     * commit (see @ref ovphysx_commit_group): after everything queued on
     * `cuda_sync.stream` and after `cuda_sync.wait_event`, with everything queued on
     * `cuda_sync.stream` afterwards running after it, and no host wait. It can therefore
     * be captured into a CUDA graph after the writes it follows -- a replayed root or
     * joint state write needs it, since the step does not refresh what a replay wrote.
     * A selected query must have refreshed once outside the capture, which uploads its
     * articulation list. Refresh any root or joint state written outside the capture before
     * beginning it: a captured refresh takes over the refresh those writes wait for, which
     * only a launch of the graph runs. Reads of link state are ordered after the refresh by
     * the engine.
     * A DirectGPU scene that has not stepped is refused: like a write, the refresh never
     * steps on the caller's behalf. A CPU scene refreshes on the host at once and is
     * refused on a capturing stream. As for a commit, the legacy default stream --
     * `stream` `1`, or a zero `stream` on a DirectGPU scene -- is refused while a blocking
     * stream of its context is being captured, before anything is queued. A stream of
     * another CUDA context than the scene's is waited for on the host instead, as for a
     * commit, and refused the same way, since the refresh then runs on the scene's legacy
     * default stream. On a driver older than CUDA 12.5, which capture needs, a refresh on a
     * capturing stream is refused before anything is queued.
     *
     * @param handle Physics instance handle.
     * @param query Live query owned by @p handle and selecting
     *        OVPHYSX_QUERY_TARGET_ARTICULATION. Membership is evaluated when the task
     *        executes.
     * @param flags Bitwise OR of @ref ovphysx_articulation_kinematic_flag_t.
     *        Zero is a successful no-op after validating the query.
     *        Non-DirectGPU scenes honor the component mask. DirectGPU's native
     *        refresh may update both components for either supported nonzero mask.
     * @param cuda_sync The caller's stream and event the refresh is ordered after;
     *        `{0, 0}` orders it on the legacy default stream.
     * @return Task receipt. Rejected with no task: unsupported flags
     *         (OVPHYSX_API_INVALID_ARGUMENT); a query that is unknown, released or of
     *         another instance (OVPHYSX_API_NOT_FOUND); a query that does not select live
     *         whole articulations (OVPHYSX_API_INVALID_ARGUMENT); a call inside a log,
     *         runtime or object-change callback or on the instance's own worker thread
     *         (OVPHYSX_API_INVALID_STATE). A query whose producer failed, a failed attach
     *         or detach submitted before it, lazy attach, an unstepped DirectGPU scene and
     *         backend failures are reported by waiting for the accepted task.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_articulations_kinematic(
        ovphysx_instance_handle_t handle,
        ovphysx_query_handle_t query,
        uint32_t flags,
        ovstage_cuda_sync_t cuda_sync);

    /** @} */


    /*--------------------------------------------------*/
    /* PhysX object interop                             */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_physx_interop */
    /** @{ */

    /**
     * Native PhysX addresses are the read-only `physxPointer` column
     * (`OVPHYSX_ATTR_PHYSX_POINTER`). Query the matching `ovphysx_query_target_t`
     * and read that attribute. There is no dedicated pointer lookup.
     *
     * Structural changes (adding, removing or re-inserting objects in a scene
     * ovphysx manages, moving them between scenes, attaching or detaching shapes)
     * through a native PhysX pointer are not recommended. ovphysx cannot detect such
     * changes; after one, its reads and writes are not guaranteed to work until the
     * stage is detached and re-attached (a call may fail, read or write the wrong
     * object without error, or crash). To change the structure and keep reads and
     * writes working, use ovstage and ovphysx_update_from_ovstage(), ovphysx_clone(),
     * or detach and re-attach.
     */

    /**
     * Subscribe to PhysX object create/destroy notifications.
     *
     * Pair with a `physxPointer` read to manage the lifetime of cached PhysX
     * SDK pointers. On a destruction notification the caller MUST drop the
     * cached pointer before returning from the callback; it is unusable after
     * that return. On a bulk teardown the caller MUST drop every cached pointer
     * before returning from the all-objects callback. On a creation
     * notification the caller should mark the prim path as dirty in its own
     * cache and read `physxPointer` for the new address ONLY
     * after the triggering synchronous call returns, or after
     * ovphysx_wait_task() returns for async work, never from inside the
     * callback itself (see Threading below).
     *
     * Subscriptions are per-instance. Callbacks fire only while this handle's
     * own attachment is live. A peer instance does not receive this
     * instance's events, and cannot unsubscribe this instance's subscription.
     *
     * Lifecycle: callbacks fire for changes that occur during simulation and
     * ovphysx_detach_ovstage()'s bulk teardown. The initial object population from
     * ovstage attach/update is NOT notified, since the caller already has that
     * state from setup. ovphysx_clone() does NOT fire object_created
     * notifications. Pointer caches that need to track cloned objects must be
     * refreshed after ovphysx_wait_task() returns on the clone operation.
     * These notifications stay callbacks; they are not query or read records.
     *
     * Threading: callbacks may fire from internal worker threads. Do NOT call
     * other ovphysx APIs from inside a callback. Defer that until the
     * triggering synchronous call returns, or until ovphysx_wait_task() returns
     * for async work. See the docstring on ovphysx_object_change_callbacks_t
     * for the full contract.
     * A callback that waits for an ovphysx call on another thread can deadlock
     * if any thread meanwhile attaches, detaches or destroys an instance,
     * starts, stops or queries a recording, or makes the first step, query or binding of an instance whose stage has not been parsed yet.
     *
     * Failure modes:
     * - OVPHYSX_API_INVALID_ARGUMENT if callbacks or out_subscription is NULL,
     *   or if every callback function pointer in the struct is NULL.
     * - OVPHYSX_API_ERROR if the physics runtime is not available (e.g. no
     *   ovphysx instance has been created yet).
     * - OVPHYSX_API_INVALID_STATE for runtime or application log callback
     *   reentry; rejection occurs before blocking runtime admission.
     * Null callback/output arguments and an empty callback set are validated
     * before callback rejection. Rejection precedes instance lookup.
     *
     * Finishes the instance's pending execution without collecting its task
     * receipts, then retains shared admission through native registration.
     *
     * On failure, *out_subscription is set to OVPHYSX_INVALID_SUBSCRIPTION_ID.
     *
     * @param[in]  handle            Instance that owns the subscription.
     * @param[in]  callbacks         Pointer to the callback set. The struct is
     *                               copied internally, so the caller does not need
     *                               to keep it alive after this call returns.
     * @param[out] out_subscription  Receives the subscription ID on success.
     *                               Pass this to ovphysx_unsubscribe_object_changes()
     *                               to stop receiving notifications.
     * @return OVPHYSX_API_SUCCESS on success, or an error status.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_subscribe_object_changes(
        ovphysx_instance_handle_t handle,
        const ovphysx_object_change_callbacks_t* callbacks,
        ovphysx_subscription_id_t* out_subscription);

    /**
     * Unsubscribe from PhysX object change notifications.
     *
     * Stops delivery of further notifications for the given subscription ID.
     * OVPHYSX_API_SUCCESS does not return while one of this subscription's
     * callbacks is still running. After that return the subscription ID is
     * consumed and must not be reused, and user_data will not be touched again
     * by this subscription.
     *
     * Calling this from inside any object-change callback returns
     * OVPHYSX_API_INVALID_STATE and leaves the subscription active. That
     * check follows the Carbonite task when the callback is running as one,
     * and the native thread otherwise. Another task on the same worker is
     * not inside the callback. The dispatch snapshot retains every subscriber
     * until that callback returns, so waiting would deadlock. Unsubscribe
     * again after it returns.
     *
     * Idempotency: passing an already-unsubscribed or unknown subscription ID
     * returns OVPHYSX_API_NOT_FOUND. A subscription owned by another instance
     * also returns OVPHYSX_API_NOT_FOUND and leaves that subscription active.
     * OVPHYSX_INVALID_SUBSCRIPTION_ID is rejected with OVPHYSX_API_INVALID_ARGUMENT.
     * NOT_FOUND is not a drain barrier: only SUCCESS means in-flight callbacks
     * for that subscription have finished.
     *
     * @param handle       Instance that owns the subscription.
     * @param subscription Subscription ID returned from
     *                     ovphysx_subscribe_object_changes().
     * @return OVPHYSX_API_SUCCESS on success, or an error status.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_unsubscribe_object_changes(
        ovphysx_instance_handle_t handle,
        ovphysx_subscription_id_t subscription);

    /** @} */

    /*--------------------------------------------------*/
    /* Task completion */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_stream */
    /** @{ */

    /**
     * @brief Collect the terminal outcome of one accepted task.
     *
     * Reports and acknowledges only the given task, including when earlier work must
     * finish to preserve execution ordering. Uncollected failures of other tasks
     * remain available to their own wait or ovphysx_wait_all(). A terminal outcome
     * is single-use; another wait returns NOT_FOUND. Timeout leaves the unfinished
     * task retryable and returns its receipt in pending_task.
     *
     * The timeout bounds waiting for outcome publication. Zero polls readiness;
     * OVPHYSX_TIMEOUT_INFINITE waits until the outcome is published. Such a wait
     * on a native thread (not inside a Carbonite task) may run, on that thread and
     * in submission order, unstarted queued work through its target when the worker
     * lane is available, unless a CUDA graph capture of a blocking stream holds the
     * legacy default stream of the thread's current CUDA context: it then starts no
     * work. Work it runs completes as on the worker, including its callbacks. Any
     * other wait never runs or finishes work: queued work completes
     * on the worker thread, so a finite wait ends at its deadline and reports an
     * outcome published by then.
     *
     * @param handle Owning instance. Validated before other arguments.
     * @param task Copyable task receipt returned by a successful call.
     * @param timeout_ns Readiness timeout in nanoseconds.
     * @param[out] out_wait_result Non-null zero-initialized result. Nonempty reuse
     *   is rejected without overwriting it. Destroy after processing.
     * @return SUCCESS for successful completion, ERROR for an execution or
     *   internal failure, TIMEOUT for unfinished work with its receipt in
     *   pending_task, INVALID_ARGUMENT for a
     *   zero/rejected/wrong-instance receipt or invalid output, NOT_FOUND for an
     *   invalid instance or a never-issued/already-collected task, INVALID_STATE
     *   without waiting or collecting inside a log, runtime or object-change
     *   callback, whose triggering work the wait could depend on.
     *
     * @par Ownership
     * Failed receipts and borrowed messages belong to out_wait_result until
     * ovphysx_destroy_wait_task_result(). Do not modify the returned fields,
     * free error_tasks separately, or shallow-copy the result.
     * Allocation failure leaves an empty destructible output and terminal
     * outcomes uncollected, so callers can retry.
     *
     * @par Threading
     * Calls on one instance must be externally serialized, including waits and
     * error lookup. Serialized calls may move between threads.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t collect_task(
     *     ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
     * {
     *     ovphysx_wait_task_result_t result = {0};
     *     ovphysx_result_t status = ovphysx_wait_task(
     *         handle, task, OVPHYSX_TIMEOUT_INFINITE, &result);
     *     ovphysx_destroy_wait_task_result(&result);
     *     return status;
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_wait_task(
        ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
        ovphysx_timeout_t timeout_ns, ovphysx_wait_task_result_t* out_wait_result);

    /**
     * @brief Collect uncollected tasks accepted at call entry, in submission order.
     *
     * Uses one readiness deadline for the entire call. Stops at the earliest
     * unfinished task when that deadline expires; later outcomes stay uncollected.
     * Earlier completed failures are returned in error_tasks even on TIMEOUT, which
     * takes precedence over ERROR. Successful terminal outcomes are collected too.
     * A discarded original receipt does not discard its failure. Empty and repeated
     * wait-all calls succeed when no uncollected task remains.
     *
     * Validation, ownership and timeout rules match ovphysx_wait_task(). The cutoff
     * is captured before timed waiting; no runtime admission is acquired. TIMEOUT
     * always returns the earliest unfinished task within the cutoff in a nonzero
     * pending_task. An all-zero pending_task means no task remains pending within
     * the cutoff.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_wait_all(
        ovphysx_instance_handle_t handle, ovphysx_timeout_t timeout_ns,
        ovphysx_wait_task_result_t* out_wait_result);

    /**
     * @brief Report the state of an issued task without waiting or collecting it.
     *
     * The query never runs or finishes work and never acknowledges an outcome: a
     * finished task keeps reporting SUCCEEDED or FAILED until ovphysx_wait_task() or
     * ovphysx_wait_all() collects it, and reports NOT_FOUND afterwards. A job queued
     * on the instance's worker thread, such as a step, reports PENDING until the
     * worker starts it and RUNNING until its outcome is published; work that ran on
     * the calling thread is finished when its call returns.
     *
     * @param handle Owning instance.
     * @param task Copyable task receipt returned by a successful call.
     * @param[out] out_status Receives the task state. Set to
     *   OVPHYSX_TASK_STATUS_NOT_FOUND on every failure.
     * @return SUCCESS when out_status was written, including NOT_FOUND for a
     *   never-issued or already-collected task; INVALID_ARGUMENT for a null
     *   out_status or a zero/rejected/wrong-instance receipt; NOT_FOUND for an
     *   invalid instance.
     *
     * @par Threading
     * Calls on one instance must be externally serialized, as for waits. That includes a
     * callback: one that queries its instance's tasks while another thread waits on that
     * instance races the wait's collection.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     * #include <stdbool.h>
     *
     * static bool task_finished(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
     * {
     *     ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_NOT_FOUND;
     *     if (ovphysx_query_task_status(handle, task, &status).status != OVPHYSX_API_SUCCESS)
     *         return false;
     *     return status == OVPHYSX_TASK_STATUS_SUCCEEDED || status == OVPHYSX_TASK_STATUS_FAILED;
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_query_task_status(
        ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
        ovphysx_task_status_t* out_status);

    /** @} */

    /*--------------------------------------------------*/
    /* Error handling */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_errors */
    /** @{ */

    /**
     * @brief Query the error string for the last failed API call on the calling thread.
     *
     * The returned string is valid until the next ovphysx API call on the same thread.
     * Returns a stable empty string if the last call succeeded.
     *
     * @return Error message, or {"", 0} on success. ptr is always non-NULL and
     *         ptr[length] is '\0'.
     *
     * @par Threading
     * Thread-local storage. Safe to call from any thread.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     * #include <stdio.h>
     *
     * int main(void)
     * {
     *     ovphysx_result_t result = ovphysx_initialize();
     *     if (result.status != OVPHYSX_API_SUCCESS) {
     *         ovphysx_string_t error = ovphysx_get_last_error();
     *         if (error.length != 0)
     *             fprintf(stderr, "Error: %.*s\n", (int)error.length, error.ptr);
     *         return 1;
     *     }
     *     return ovphysx_shutdown().status == OVPHYSX_API_SUCCESS ? 0 : 1;
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_string_t ovphysx_get_last_error(void);

    /**
     * @brief Borrow a collected task's error from its live wait-result.
     *
     * Lookup is read-only and never acknowledges work. A missing retained cause
     * returns {"", 0}. An invalid instance or zero/rejected/wrong-instance receipt
     * also sets the immediate diagnostic retrieved by ovphysx_get_last_error().
     * The string survives unrelated waits and instance destruction until the
     * owning wait-result is destroyed. A fresh lookup requires a live instance.
     * ptr is always non-NULL and ptr[length] is '\0'.
     */
    OVPHYSX_API ovphysx_string_t ovphysx_get_last_task_error(
        ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task);

    /**
     * @brief Release wait-result storage and zero every field.
     *
     * Accepts NULL and already-cleaned results. Safe after instance teardown;
     * retained messages and receipts own no simulation or CUDA resources.
     * A non-NULL result must be zero-initialized or produced by a task wait with
     * its returned fields unchanged. Never free error_tasks separately.
     */
    OVPHYSX_API void ovphysx_destroy_wait_task_result(ovphysx_wait_task_result_t* result);

    /** @} */


    /*--------------------------------------------------*/
    /* Logging configuration                            */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_logging Logging configuration
     * Configure the process-scoped ovphysx library source log level and the single
     * application log callback.
     *
     * By default, libovphysx logs to the Carbonite console at WARNING level.
     * Set OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL to change the threshold and use
     * ovphysx_set_log_callback() to receive messages programmatically.
     */

    /** @addtogroup ovphysx_logging */
    /** @{ */

    /**
     * @brief Set or disable the application log callback.
     *
     * One callback may be registered. Calling again publishes the replacement
     * for newly accepted messages, then waits for the prior registration's
     * in-flight invocations before returning. Passing NULL for @p callback
     * immediately stops accepting delivery and drains the prior registration.
     * Once set, the callback pointer and resources reachable through
     * @p user_data must remain valid until a later replacing or disabling call,
     * or a successful ovphysx_shutdown(), returns. This includes shutdown with
     * live handles retained only for explicit destruction. During replacement,
     * accepted invocations of the old registration may overlap invocations of
     * the newly published registration. Invocations within each registration
     * remain serialized.
     * @p min_severity applies to every record in the observed process log
     * stream. Records from the library-owned `omni_physx_sdk`, `omni.physx`,
     * and `ovphysx_internal` sources are also subject to the source threshold
     * configured by OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL. OVPHYSX_LOG_DEFAULT uses
     * WARNING.
     *
     * @p channel_filter is optional. It is a comma-separated list of
     * `channel=level` rules. ASCII whitespace around entries, channels, and
     * levels is ignored, and level names are case-insensitive. A rule matches
     * any channel beginning with its raw prefix. Rules are considered in
     * declaration order. The longest matching prefix wins, and a later rule
     * wins ties between equal-length matching prefixes. An empty filter
     * uses @p min_severity for every channel.
     *
     * @note The callback may be invoked from any thread, but invocations for
     *       the active registration are serialized.
     * @note Do not synchronously wait for work that may log into this same
     *       registration. Native execution/admission calls, including recording
     *       start and stop, reject callback reentry with INVALID_STATE
     *       before blocking.
     * @note The callback observes Carbonite's process log stream. The channel
     *       parameter identifies the emitting source for application filtering.
     * @note Must not be called from within the callback. Returns
     *       OVPHYSX_API_ERROR if called during callback dispatch.
     * @note An OVPHYSX_API_ERROR after valid arguments may be reported after
     *       the replacement was published. Conservatively keep @p callback and
     *       @p user_data alive until a later successful replace, disable, or
     *       shutdown drains the slot.
     *
     * @param min_severity Minimum delivered severity.
     * @param channel_filter Optional non-owning filter view, copied by the call.
     * @param callback Callback function, or NULL to disable.
     * @param user_data Opaque pointer forwarded to every callback invocation.
     * @return ovphysx_result_t with status.
     * @post Invalid arguments leave the prior registration unchanged.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_log_callback(
        ovphysx_log_level_t min_severity,
        const ovphysx_string_t* channel_filter,
        ovphysx_log_callback_t callback,
        void* user_data);

    /**
     * @brief Wait for application-callback deliveries accepted before this call.
     *
     * This is an exact barrier for records already handed to ovphysx by
     * Carbonite. If the host enabled Carbonite asynchronous logging, records
     * still buffered upstream have not yet been accepted and are outside this
     * barrier. Successful ovphysx_shutdown() flushes that upstream buffer before
     * disabling and draining the callback.
     *
     * @param timeout_ns Maximum wait in nanoseconds. OVPHYSX_TIMEOUT_POLL polls.
     *        OVPHYSX_TIMEOUT_INFINITE waits indefinitely.
     * @return OVPHYSX_API_SUCCESS when drained, OVPHYSX_API_TIMEOUT on timeout,
     *         or OVPHYSX_API_ERROR when called from a log callback.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_flush_log(ovphysx_timeout_t timeout_ns);

    /** @} */


#ifdef __cplusplus
}
#endif

/* Compatibility umbrella: existing #include <ovphysx/ovphysx.h> consumers still
 * see the moved families. New code includes ovphysx_legacy.h explicitly. */
#include "ovphysx/ovphysx_legacy.h"

#endif // OVPHYSX_OVPHYSX_H
