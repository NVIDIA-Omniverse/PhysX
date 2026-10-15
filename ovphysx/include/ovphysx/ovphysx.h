// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */

/** @implements REQ-CAPI-DETACH-001
 * @covers AC-3 AC-4
 */

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
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
 * @covers AC-1 AC-4 AC-9 AC-10 AC-11
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-5a AC-11
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
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1 AC-2
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


    /** Deprecated forwarding exports introduced in 0.6 and retained in 0.7. */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_ovd_start_recording") ovphysx_result_t
    ovphysx_start_recording(ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination);
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_ovd_stop_recording") ovphysx_result_t
    ovphysx_stop_recording(ovphysx_instance_handle_t handle);
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
    *   SUCCESS means admission; collect execution status with a task wait.
    *   On failure, call ovphysx_get_last_error() on the same thread to retrieve the
    *   error message. The returned string is valid until the next ovphysx API call on
    *   that thread.
    *
    * Stream ordered asynchronous execution:
    *   - Async operations execute in submission order on a single queue.
    *   - Results appear as if operations completed serially (sequential consistency).
    *   - Use ovphysx_wait_task() to ensure operations have completed before using results.
    *   - If task admission fails (error returned from the function call), the
    *     returned task receipt is invalid and cannot be used for synchronization.
    *   - Each task's final outcome can be collected only once, through
    *     ovphysx_wait_task() or ovphysx_wait_all(). Task receipts may be copied,
    *     but copying does not allow collecting the same outcome again. A timeout
    *     leaves unfinished tasks available to wait on again. Calls on one instance,
    *     including waits and error lookup, must be externally serialized.
    *
    * Failure handling:
    *   - If operation N fails, the error is recorded and returned via ovphysx_wait_task().
    *   - Waiting on one task reports only its outcome. ovphysx_wait_all() collects
    *     outstanding failures, including tasks whose original receipt was discarded.
    *   - A recoverable failure does not block otherwise-valid later work. Actual
    *     prerequisites and instance state are still validated before admission.
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
    *     tolerance scale, and recording transitions that need every
    *     attached instance idle.
     *
     * Single instance, multiple threads:
     *   - Operations are NOT thread-safe for the same instance.
     *   - Use external synchronization if calling from multiple threads.
     *
     * Data buffer lifetimes:
     *   - The caller keeps data passed to async operations valid and unmodified
     *     until the operation completes (as indicated by ovphysx_wait_task).
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
     * is NULL, or OVPHYSX_API_ERROR when lifecycle/runtime initialization fails.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_instance(ovphysx_instance_handle_t* out_handle);

    /**
     * @brief Destroy an ovphysx instance and release per-instance resources.
     *
     * Per-instance destruction leaves the process lifecycle active. Call
     * @ref ovphysx_shutdown after destroying the final handle to drain the
     * direct PhysX runtime and clear that lifecycle token.
     *
     * @param handle ovphysx handle to destroy.
     * @return ovphysx_result_t with status and error info.
     *
     * @post On success, the handle is unregistered and its resources are
     *       released. A repeated call for that already-unregistered value
     *       returns OVPHYSX_API_ERROR before teardown and has no effect on
     *       registered instances or process-global asynchronous state.
     *
     * @par Side Effects
     * Releases internal resources, plugins, and cached data for this instance.
     *
     * @par Threading
     * Do not destroy an instance while it is in use on other threads.
     *
     * @par Ownership
     * After destruction, any bindings created by this instance are invalid.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_STATE is retryable: admission rejected before teardown;
     *   the handle, attachment and caller ownership remain live. Close companion
     *   leases or finish pending work/transitions before retrying.
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
     *   still prevent shutdown. The lifecycle token remains live.
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
     * @par Static runtime mode
     * With no live handles, shutdown drains the direct PhysX runtime. On Linux it
     * also releases the SDK-owned Carbonite framework and private cooking cache.
     * Windows retains the framework until process exit for safe dependency DLL
     * teardown; private cache removal after datastore shutdown is best-effort.
     * A framework populated by a foreign host remains owned by that host. If live
     * handles remain, callers still own them and the direct runtime remains
     * available only for their explicit destruction. Continued stepping or other
     * instance work after shutdown is unsupported, and destruction-time records
     * are not delivered to the application callback disabled by shutdown.
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
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_global_config(
        ovphysx_global_config_key_t key, ovphysx_config_value_t value);

    /** Read the effective process-global configuration value. */
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
     * The call is synchronous. A FILE destination is an exact path. A TCP
     * destination connects to a ready listener. Validation and stream-open
     * failures may be retried. Only one recording may be active in the shared
     * runtime: another start returns OVPHYSX_API_INVALID_STATE without
     * replacing its destination. After a successful stop, another FILE or TCP
     * session may be started. Late start requires a live physics stage in the
     * shared runtime. Before its first attach and between detach and reattach it
     * returns OVPHYSX_API_INVALID_STATE.
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
     * @param handle Instance requesting the recording session.
     * @param destination FILE or TCP destination borrowed for this call.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT for an invalid
     *         destination, OVPHYSX_API_INVALID_STATE when the runtime is not
     *         recording-capable, the shared runtime has no live physics stage,
     *         or recording is active,
     *         OVPHYSX_API_NOT_IMPLEMENTED on an unsupported platform, or
     *         OVPHYSX_API_ERROR when the destination cannot be opened or
     *         sampling cannot start.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_ovd_start_recording(
        ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination);

    /**
     * @brief Stop and finalize the active recording.
     *
     * The handle that started the session owns it. A peer handle is inactive
     * and cannot stop the owner's session. A successful owner stop
     * permits a later recording session to start.
     *
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_STATE when this handle
     *         owns no active recording (including while a peer owns the
     *         globally active session), or OVPHYSX_API_ERROR when
     *         stop/finalization fails.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_ovd_stop_recording(ovphysx_instance_handle_t handle);

    /**
     * @brief Query whether this instance owns the active recording.
     *
     * A peer reports false while another instance owns the shared runtime's
     * active recording.
     *
     * @param handle Valid instance handle.
     * @param out_is_recording [out] Receives true only while sampling is active.
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
    * Returns a task receipt. Execution may finish inline; admission success does
    * not establish that attachment succeeded. Collect the task with
    * ovphysx_wait_task() before using the attachment outside instance stream order.
    * Earlier tasks finish execution first, without collecting their outcomes.
    *
    * @param handle ovphysx instance handle.
    * @param stage Caller-owned ovstage Stage (`ovstage_instance_t*`).
    * @param read_ordinal Caller-owned ordinal at which the initial scan's
    *      required physics data is sealed. Must be non-zero: 0 is reserved as
    *      the runtime's internal "use payload attach-time ordinal" sentinel.
    *      The application owns ordinal advancement. Subsequent edits are drained
    *      via ovphysx_update_from_ovstage().
    *
    * @pre handle is valid, ovphysx is not already attached to a Stage, stage
    *      is non-null, and read_ordinal is non-zero.
    * @pre stage and its sealed read_ordinal data remain valid through task
    *      completion. After successful attach, stage outlives the attachment,
    *      through successful detach completion. Destroying an attached Stage is
    *      undefined behavior. A failed attach may leave an attachment if rollback
    *      fails; check ovphysx_is_stage_attached() before releasing the Stage.
    * @post On successful task completion, subsequent ovphysx_update_from_ovstage() calls observe
    *       committed Stage writes through the runtime ovstage backend.
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
    *       tolerance scale, and recording transitions that need every attached
    *       instance idle.
    *
    * @par Errors
    * Admission rejects an invalid handle, null stage, or zero read_ordinal with
    * OVPHYSX_API_INVALID_ARGUMENT, and an already-attached instance or unavailable
    * runtime API with OVPHYSX_API_ERROR. The schema guard also rejects admission
    * with OVPHYSX_API_ERROR if ovstage reports a missing PhysX schema family after
    * USD schema definitions were built, including by population or export,
    * unless the guard is disabled (see below). Rejected receipts
    * have no task identity; the schema guard takes no Stage ownership and changes
    * no attachment.
    * Runtime attach failures, including unreadable schema data and mismatched
    * tolerance scales, belong to the accepted task and are reported by its wait.
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
    *       skips the family check and leaves it active for later attaches. After
    *       population or export builds the definitions, the check applies even to
    *       procedural Stages. A registration another USD
    *       consumer made unobservable to ovstage cannot be detected. Nor can the
    *       guard identify schema families registered late before its first check:
    *       catching a late registration's OVSTAGE_ERROR_OP_FAILED can let a later
    *       attach succeed with missing data. Always restart after a registration-
    *       ordering error; attach success does not prove schemas were available
    *       during population. The legacy create-time Carbonite setting
    *       `/ovphysx/schemas/requireRegistration = false` downgrades refusal to a
    *       warning and permits attach without repairing lost data; its default
    *       is true. The guard neither locates nor installs schema resources.
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
    * Returns a task receipt. The range value is copied; the caller-owned Stage
    * and the selected sealed data must remain valid until task completion.
    * Successful completion makes the updates visible to subsequent work. Earlier
    * execution is drained without collecting its outcomes. This operation may
    * execute inline; collect its own outcome with ovphysx_wait_task().
    *
    * @param handle ovphysx instance handle.
    * @param range ovstage ordinal range to drain (see ovstage_ordinal_range_t).
    *
    * @pre `handle` is valid and the attach task completed successfully.
    * @pre when `range.has_start_ordinal`, `range.start_ordinal <= range.end_ordinal`.
    * @pre All selected writes are sealed by a completed write-floor operation
    *      covering `range.end_ordinal`. Waiting for
    *      `ovstage_population_apply_usd_changes()` only completes population.
    *      It does not advance the write floor.
    *
    * Ordinals at or below the latest successfully consumed ordinal are skipped.
    * A range containing only consumed ordinals is a successful no-op. An
    * overlapping range applies only its unread suffix. The initial
    * `read_ordinal` is consumed by ovphysx_attach_ovstage(), so replaying it does
    * not recreate the attach population or emit object-change notifications.
    * Population authored and sealed at later ordinals is applied normally.
    *
    * @par Errors
    * Admission rejects an invalid range or handle with OVPHYSX_API_INVALID_ARGUMENT,
    * and a missing attachment or runtime API with OVPHYSX_API_ERROR. Runtime range
    * drain failures belong to the accepted task. Failed updates need not be atomic;
    * do not assume that no edits were applied or blindly replay the range.
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_from_ovstage(ovphysx_instance_handle_t handle,
                                                            ovstage_ordinal_range_t range);

    /**
    * @brief Detach the currently-attached ovstage Stage.
    *
    * Returns a task receipt, including for an idempotent no-op on an unattached
    * instance. Execution may finish inline; collect the outcome with
    * ovphysx_wait_task(). Earlier execution finishes without collecting its outcomes.
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
    * Detach stops and finalizes an active OmniPVD recording started by this instance
    * or when it removes the last live attachment. A peer-owned recording continues
    * while another attachment remains live. After reattach,
    * recording remains dormant until ovphysx_ovd_start_recording() is called.
    *
    * @param handle ovphysx instance handle.
    *
    * @pre handle is valid.
    * @post On successful task completion, ovphysx is unattached.
    *
    * @par Errors
    * Admission rejects invalid handles with OVPHYSX_API_INVALID_ARGUMENT.
    * OVPHYSX_API_INVALID_STATE is retryable admission failure: attachment and
    * write storage remain owned. Finish companion work or transitions and retry.
    * Failures after teardown starts belong to the accepted task; collect them
    * before deciding whether to retry or destroy the instance.
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
     * Query, read, write, fetch, commit, and release return task receipts. The
     * receipt reports admission only; wait for its task and check the task result
     * before using outputs or starting an operation that depends on it. Keep input
     * arrays, string storage, and output slots alive until task completion. These
     * operations currently execute inline. The task contract does not promise that
     * they will always finish before returning.
     *
     * A successful task makes host outputs ready. CUDA columns retain the event
     * contract below; task completion does not replace their readiness event.
     * Rejected submissions have no task and do not consume resources. Accepted
     * release submissions transfer cleanup to the SDK: stop using the resource
     * immediately, even if waiting later reports failure. The SDK retains or
     * quarantines storage whose safe release could not be established.
     *
     * Resource handles belong to the instance that created them. A live handle
     * from another instance returns OVPHYSX_API_NOT_FOUND. Releases of absent
     * handles are successful no-ops when the supplied instance is valid.
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
     * Each operation observes the most recently completed step after draining
     * pending simulation on the instance. A step that lands between ovphysx_query and
     * ovphysx_read does not make the handle stale: the read simply reflects the
     * newer step. OVPHYSX_SCOPE_ACTIVE uses the scene's most recent active-actor
     * report where available: rigid bodies and articulation links use their own
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
     * @param out_query [out] Receives the query handle. Nonzero on success, INCLUDING
     *      an empty match (a valid query whose read reaches end-of-iteration
     *      immediately and whose optional query result reports total_prim_count == 0).
     *      0 means FAILURE only.
     * @return Admission receipt; query creation failures are reported by waiting.
     *
     * @pre handle is valid. An ovstage Stage is attached, except for
     *      OVPHYSX_QUERY_TARGET_DEBUG_RENDER.
     * @post After successful task completion, `*out_query` is non-zero and usable
     *       with ovphysx_read. On rejection or failed completion it remains zero.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle, object_type, or scope is invalid,
     *   or out_query is null.
     * - OVPHYSX_API_ERROR if no ovstage is attached (not reported for
     *   OVPHYSX_QUERY_TARGET_DEBUG_RENDER), the query backend is unavailable,
     *   pending simulation cannot be finished, or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. Query construction
     * failures are reported as OVPHYSX_API_ERROR by @ref ovphysx_wait_task.
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
     * @param out_query [out] Nonzero and usable only after successful task completion.
     * @param out_summary Optional creation-time discovery snapshot, as on ovphysx_query.
     *      Retain selection and output storage until task completion.
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
     * - OVPHYSX_API_ERROR if no ovstage is attached, the query backend is unavailable, pending
     *   simulation cannot be finished, or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. Query construction failures are
     * reported as OVPHYSX_API_ERROR by @ref ovphysx_wait_task.
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
     * @param out_query          [out] Nonzero and usable only after successful task completion.
     * @param out_summary        Optional creation-time discovery snapshot, as on @ref ovphysx_query.
     *      Retain selections, their input buffers and output slots until task completion.
     * @return ovphysx_enqueue_result_t (an enqueue receipt; wait on it with @ref ovphysx_wait_task).
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT, before a task is created, if out_query or selections is NULL,
     *   selection_count is 0, a selection's filters is NULL while filters_per_sensor is not 0,
     *   filters_per_sensor exceeds 21845, or any selection is invalid as for @ref ovphysx_query_selection
     * - OVPHYSX_API_ERROR if no ovstage is attached
     * - OVPHYSX_API_ERROR, reported by @ref ovphysx_wait_task, for a filter selection whose match count is
     *   neither 1 nor the selection's sensor count, or a sensor whose filters name one object twice; the
     *   message names the selection, the filter and both counts
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
     * an `ovx_string_or_token_t`: a string name (see OVPHYSX_ATTR_* in
     * ovphysx_attributes.h) OR an interned token (e.g. from the optional query
     * result, fed straight back with no token-to-string round-trip). Iterate
     * the result with @ref ovphysx_fetch_read_next,
     * release each consumed group with @ref ovphysx_release_group, and release the
     * session with @ref ovphysx_release_read. Names not produced by the queried type
     * are skipped and reported in the optional summary. Duplicate canonical requests
     * and unresolvable tokens fail the task; tokens take precedence over strings.
     * Already-known backend failures fail open without publishing a session.
     *
     * ## Requested name vs. emitted `group.attribute`
     *
     * The name passed here is the SEMANTIC name requested (e.g. "position"). The
     * resulting group's `attribute` token is the name actually EMITTED, which is the
     * name that feeds the ovstage write path verbatim, and the two are not always
     * spelled identically. A single-prim "position"/"orientation" request on a
     * point-instancer is emitted as the instancer-array attribute
     * "positions"/"orientations" (singular to plural) in INSTANCER-LOCAL space, since
     * that is what writes back onto the instancer prim. Deformable/particle "points"
     * are emitted mesh/set-local. Velocities stay world-space. Request by intent and
     * write back using `group.attribute` (+ `group.semantic` for the coordinate role).
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
     * completed**. `group.data.cuda_sync.wait_event` is a CUDA event the producer work
     * signals, and `cuda_sync.stream` is 0 (the read never asks a consumer to drain a
     * stream). A consumer reading on the legacy default/null stream in the producing
     * CUDA context is already ordered after that work. A consumer using another stream must
     * make that stream wait on `wait_event` first, or it may observe a partially
     * written column: call CUDA's `cuStreamWaitEvent` or `cudaStreamWaitEvent` with
     * the nonzero event. An event of 0 needs no wait. The event belongs to the read
     * session and is valid until the session is released. Do not destroy it.
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
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param attributes Array of `attribute_count` `ovx_string_or_token_t` (string
     *      name or interned token per entry).
     * @param attribute_count Number of attributes; zero opens an empty read session.
     * @param out_read [out] Receives the read-session handle (0 on failure).
     * @param out_summary Optional immutable open summary; NULL skips summary work.
     *      Nested arrays remain valid until accepted read release or instance
     *      destruction. They describe this open even after detach;
     *      do not resolve their tokens through an invalidated attachment's dictionary.
     *      Omission bits classify missing partitions; backend failure fails the task instead.
     * @return Admission receipt; read construction failures are reported by waiting.
     *
     * @pre handle and query are valid. Keep the attributes, their string storage,
     *      out_read and any out_summary slot alive until task completion. This
     *      operation drains earlier simulation on the instance.
     * @post After successful task completion, `*out_read` is non-zero. CUDA data
     *       readiness still requires the group's event handoff.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid, out_read is null,
     *   attributes is null with a nonzero count, or a token-less attribute string
     *   is null with a nonzero length.
     * - OVPHYSX_API_NOT_FOUND if query is absent, released, or belongs to
     *   another instance.
     * - OVPHYSX_API_ERROR if the read backend is unavailable, pending simulation
     *   cannot be finished, or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. Read construction
     * failures are reported as OVPHYSX_API_ERROR by @ref ovphysx_wait_task.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_read(ovphysx_instance_handle_t handle,
                                                      ovphysx_query_handle_t query,
                                                      const ovx_string_or_token_t* attributes,
                                                      size_t attribute_count,
                                                      ovphysx_read_handle_t* out_read,
                                                      ovphysx_read_result_t* out_summary);

    /**
     * @brief Fetch the next output column group from a read session.
     *
     * After successful task completion, `*out_group` points at the next
     * `ovstage_read_group_t`, or is NULL at end of iteration. End of iteration is
     * an ordinary successful task. A failed task leaves `*out_group` NULL and must
     * not be treated as end of iteration. Keep out_group alive until completion.
     *
     * The group is **producer-owned** (the caller does not allocate it): the returned
     * `ovstage_read_group_t` pointer is a borrow valid until @ref ovphysx_release_group
     * is called for that group's `read_group_id`, or the session is released.
     *
     * Group lifetime (authoritative): the returned group struct and its stage-derived
     * `prims.list` stay valid until @ref ovphysx_release_group for that `read_group_id`
     * (or @ref ovphysx_release_read, which releases all). The numeric storage is owned
     * by the read session instead: `data.tensors`, every tensor's shape and data,
     * `prims.index_map`, `data.index_map`, `data.mask`, and
     * `data.cuda_sync.wait_event` stay valid until
     * @ref ovphysx_release_read. Fetching further groups and an intervening
     * @ref ovphysx_step do NOT invalidate either lifetime (the runtime gathers each
     * column into session-owned storage at read time). Numeric snapshots also
     * survive stage detach until session release or instance destruction. Detach
     * invalidates the stage dictionary and its prim-list handles; do not resolve
     * those handles after detaching.
     *
     * A device (`kDLCUDA`) column additionally has a readiness contract: it is handed
     * over before its producing work has necessarily completed. Wait on
     * `group.data.cuda_sync.wait_event` before reading it from a non-default stream
     * using CUDA's `cuStreamWaitEvent` or `cudaStreamWaitEvent`. See @ref ovphysx_read for
     * the full rule. Validity and readiness are separate: the borrow above means the
     * memory may still be read, the event means the values in it are final.
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
     * @return Admission receipt. After successful task completion, NULL means done.
     *
     * @pre handle and read are valid and out_group is non-null.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid or out_group is null.
     * - OVPHYSX_API_NOT_FOUND if read is absent, released, or belongs to
     *   another instance.
     * - OVPHYSX_API_ERROR if the read backend is unavailable or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. Backend fetch failures
     * are reported as OVPHYSX_API_ERROR by @ref ovphysx_wait_task;
     * end of iteration is successful completion with a NULL group.
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
     * @return Admission receipt. Accepted release ends the group borrow immediately.
     *         An already-released or unknown id is a successful no-op.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_group(ovphysx_instance_handle_t handle,
                                                       ovphysx_read_handle_t read,
                                                       ovstage_read_group_id_t group_id);

    /**
     * @brief Release a read session (and every borrowed group it still owns).
     *
     * Finish all external CPU and CUDA consumers before submitting release or
     * destroying the instance. The
     * SDK waits for its own producing work before reclaiming device storage; it
     * has no completion token for work launched by the caller. Accepted release
     * ends all session borrows and transfers cleanup responsibility to the SDK.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle from @ref ovphysx_read.
     * @return Admission receipt. An absent handle is a successful no-op for a valid
     *         instance. Wait to observe cleanup failures; do not retry accepted releases.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_read(ovphysx_instance_handle_t handle,
                                                      ovphysx_read_handle_t read);

    /**
     * @brief Open a write session for an array of named attributes.
     *
     * The application-to-physics direction, the mirror of @ref ovphysx_read. It reuses
     * that query verbatim and adds no selection of its own: the session reaches exactly
     * the set `query` matched, and there is no index, mask or prim-list parameter
     * anywhere on this surface.
     *
     * Each fetched group identifies its original request index through
     * ovphysx_fetched_write_group_t::attribute_index, even when attributes have
     * identical tensor shapes. Attributes are planned in request order. Commit
     * order is chosen by the caller and no transactional rollback is provided.
     * All names are validated before publishing a session. Canonical duplicates,
     * including string/token aliases, and unsupported/read-only writes fail the
     * creation task. A valid request with no matching objects is an empty session.
     * Token ids take precedence over strings and must belong to the owning Stage
     * dictionary; an unresolvable token fails rather than falling back to a string.
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
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param attributes Nonempty array of string-or-token attributes.
     * @param attribute_count Number of attributes.
     * @param out_write [out] Receives the write-session handle (0 on failure).
     * @param out_summary Optional immutable open summary; NULL skips summary work.
     *      Nested arrays remain valid until accepted write release, stage detach
     *      or instance destruction, whichever occurs first.
     * @return Admission receipt; write construction failures are reported by waiting.
     *
     * @pre handle and query are valid. Keep attribute storage and output slots alive
     *      until completion. This operation drains earlier simulation on the instance.
     * @post After successful task completion, `*out_write` is non-zero. Detaching
     *       the stage retires write sessions and their groups.
     *       Finish external work filling those groups before any such teardown.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid, out_write or attributes is
     *   null, attribute_count is zero, or a token-less string is null with a nonzero length.
     * - OVPHYSX_API_NOT_FOUND if query is absent, released, or belongs to
     *   another instance.
     * - OVPHYSX_API_ERROR if the write backend is unavailable, pending simulation
     *   cannot be finished, or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. Write construction
     * failures, including unsupported attributes or scene pipelines, are reported
     * as OVPHYSX_API_ERROR by @ref ovphysx_wait_task.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_write(ovphysx_instance_handle_t handle,
                                               ovphysx_query_handle_t query,
                                               const ovx_string_or_token_t* attributes,
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
     * invalidate a live group.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle from @ref ovphysx_write.
     * @param out_group [out] Receives the borrowed group and original request index.
     *      EOF is group == NULL and attribute_index == SIZE_MAX after successful
     *      task completion. Failed completion is never EOF.
     * @return Admission receipt. After successful task completion, NULL means done.
     *
     * @pre handle and write are valid and out_group is non-null.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid or out_group is null.
     * - OVPHYSX_API_NOT_FOUND if write is absent, released, or belongs to
     *   another instance.
     * - OVPHYSX_API_ERROR if the write backend is unavailable or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. Backend fetch failures
     * are reported as OVPHYSX_API_ERROR by @ref ovphysx_wait_task;
     * end of iteration is successful completion with a NULL group.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_write_next(ovphysx_instance_handle_t handle,
                                                          ovphysx_write_handle_t write,
                                                          ovphysx_fetched_write_group_t* out_group);

    /**
     * @brief Commit a filled group, transferring ownership of its data to physics.
     *
     * The caller must have filled EVERY mapped entry. There is no fill-mask, so a
     * partially filled group publishes whatever its unfilled entries contain. To write
     * fewer prims, query fewer. A live group belonging to this session is spent
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
     * completion; it does not select the engine's consumer stream. A non-zero
     * `wait_event` and a non-zero `stream` are both honored before the data is
     * consumed, including when both are supplied. Only `{0, 0}` asserts nothing is
     * outstanding. A host-resident group does NOT imply `{0, 0}`, since the caller may
     * have staged on its own stream. Context-relative default-stream sentinels refer
     * to the producer context current when this call is made. Invalid stream, event
     * or context combinations fail the task. If producer completion cannot be proven,
     * the SDK retains the affected storage for safe cleanup; the group remains spent.
     * Before admission, this operation drains earlier simulation on the instance.
     * Successful task completion confirms the CPU mutation
     * or GPU submission; GPU execution may continue through the backend event chain.
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
     * @return Admission receipt; backend refusal or mutation failure is reported by waiting.
     *
     * @par Errors
     * Rejection (`receipt.status`, no task):
     * - OVPHYSX_API_INVALID_ARGUMENT if handle is invalid or group is null.
     * - OVPHYSX_API_NOT_FOUND if write is absent, released, or belongs to
     *   another instance.
     * - OVPHYSX_API_ERROR if the write backend is unavailable, pending simulation
     *   cannot be finished, or task creation fails.
     *
     * After acceptance, `receipt.status` is OVPHYSX_API_SUCCESS. An invalid, foreign,
     * or already-spent group, an unstepped DirectGPU scene, or a publication failure
     * is reported as OVPHYSX_API_ERROR by @ref ovphysx_wait_task.
     * CPU and GPU-with-readback permit pre-step commits. Committing a spent group
     * fails rather than claiming data was published again.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_commit_group(ovphysx_instance_handle_t handle,
                                                      ovphysx_write_handle_t write,
                                                      const ovstage_map_group_t* group,
                                                      ovstage_cuda_sync_t write_done_sync);

    /**
     * @brief Release a write session, discarding anything uncommitted.
     *
     * Uncommitted groups are discarded. Finish all external CPU and CUDA work that
     * fills those groups before submitting release: this call takes no producer
     * completion token. Committed groups are not rolled back; the SDK waits for its
     * own device work before reclaiming their storage. Once release is accepted,
     * all session borrows end and cleanup belongs to the SDK, including after failure.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle from @ref ovphysx_write.
     * @return Admission receipt. An absent handle is a successful no-op for a valid
     *         instance. Wait to observe cleanup failures; do not retry accepted releases.
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
     *   `ovx_string_or_token_t`. STRING-ONLY: an interned-token-only key is rejected, because
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
                                                     const ovx_string_or_token_t* attribute,
                                                     ovphysx_writability_t* out_writability);

    /**
     * @brief Release an output query.
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @return Admission receipt. An absent handle is a successful no-op for a valid
     *         instance. Wait to observe cleanup failures; do not retry accepted releases.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_query(ovphysx_instance_handle_t handle,
                                                       ovphysx_query_handle_t query);

    /**
    * @brief Clone the subtree under the source path to one or more target paths in the
    * internal physics representation (USD untouched).
    * The source path must exist in the stage.
    * The target paths must not already exist in the stage.
    *
    * Clones are created in the internal representation only (the USD file is not modified)
    * and immediately participate in physics simulation. Backed by the PhysX SDK replicator, so
    * cloned articulations are real articulations. Optimized for RL mass replication (1000s of
    * instances). This is the clone entrypoint for both standalone callers and callers using an
    * ovstage Stage (@ref ovphysx_attach_ovstage).
    *
    * @note Cross-environment collision isolation uses PhysX environment ids under GPU dynamics +
    *       GPU broadphase, controlled by `/ovphysx/clone/useEnvIds` (default on, per-process). The
    *       source holds env id 0 and clones get 1..N, so co-located clones (NULL anchor_transforms)
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
    * @param target_paths Array of target paths to clone to (must not exist)
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
    *        Pass NULL for automatic per-call numbering.
    *
    * @return Admission status and task identity. Rejection has zero identity;
    * accepted execution failure is reported by a wait. On rejection,
    *         call ovphysx_get_last_error() on the same thread for the error message.
    *
    * @note Replication executes inline. An accepted task is already terminal when
    *       this call returns. ovphysx_wait_task() immediately collects its outcome.
    *
    * @pre handle must be valid.
    * @pre source_path_in_usd must exist and target_paths must be valid and unique.
    * @pre Must be called **before** ovphysx_warmup() or the first simulation step.
    *      Multiple ovphysx_clone() calls per attach are allowed in CPU and GPU mode
    *      while all of them precede warmup / first step. To clone after that point,
    *      use ovphysx_detach_ovstage() and reattach the source stage first. On GPU,
    *      cloning later would reallocate DirectGPU buffers and corrupt initialized
    *      state.
    * @post Cloned physics objects are live when the accepted task succeeds.
    *
    * @par Side Effects
    * Adds live PhysX objects keyed by each target path. No USD or runtime-stage prims are authored.
    *
    * @par Ownership
    * The target_paths array is read during the call. The caller retains ownership.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid or duplicate targets, or a call
    *   after warmup / the first step
    * - OVPHYSX_API_ERROR if no ovstage is attached or the clone fails
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
    * The simulation time is tracked internally. Each step advances it by step_dt.
    * The timestep is passed through without clamping or substepping. The caller
    * is responsible for choosing a timestep appropriate for the simulation.
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
    * - OVPHYSX_API_ERROR for internal failures
    *
    * PhysX CUDA-context failures after admission are reported by the task wait,
    * with a retained CUDA diagnostic even when logging is disabled.
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
     * The returned receipt reports admission. Wait for successful task completion
     * before reading the updated link state. The update currently executes inline.
     * It is useful after writing articulation DOF positions (through the session
     * write API, @ref ovphysx_write, or the deprecated tensor bindings) and before
     * reading link pose tensors in the same frame.
     *
     * NOTE: On the first GPU kinematic update after loading USD, an automatic
     * warmup simulation step may be performed to initialize PhysX structures.
     * See the tensor auto-warmup note in @c ovphysx_legacy.h.
     *
     * Once GPU warmup is complete, the FK refresh itself does not run collision
     * detection, integration, solver work, or contact generation.
     *
     * @param handle Physics instance handle.
     * @param query Live query owned by @p handle and selecting
     *        OVPHYSX_QUERY_TARGET_ARTICULATION. Membership is evaluated when the task
     *        executes.
     * @param flags Bitwise OR of @ref ovphysx_articulation_kinematic_flag_t.
     *        Zero is a successful no-op after validating the query.
     *        Non-DirectGPU scenes honor the component mask. DirectGPU's native
     *        refresh may update both components for either supported nonzero mask.
     * @return Admission receipt. Lazy attach, warmup, and backend failures are
     *         reported by waiting for the accepted task.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_articulations_kinematic(
        ovphysx_instance_handle_t handle,
        ovphysx_query_handle_t query,
        uint32_t flags);

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
     *
     * Failure modes:
     * - OVPHYSX_API_INVALID_ARGUMENT if callbacks or out_subscription is NULL,
     *   or if every callback function pointer in the struct is NULL.
     * - OVPHYSX_API_ERROR if the physics runtime is not available (e.g. no
     *   ovphysx instance has been created yet).
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
     * The timeout bounds readiness. Zero polls once; OVPHYSX_TIMEOUT_INFINITE
     * waits indefinitely. Readiness observed at the deadline wins. Synchronous
     * finalization can extend wall-clock duration beyond the readiness timeout.
     *
     * @param handle Owning instance. Validated before other arguments.
     * @param task Copyable successful-admission receipt from a task producer.
     * @param timeout_ns Readiness timeout in nanoseconds.
     * @param[out] out_wait_result Non-null zero-initialized result. Nonempty reuse
     *   is rejected without overwriting it. Destroy after processing.
     * @return SUCCESS for successful completion, ERROR for an execution or
     *   internal failure, TIMEOUT for unfinished work, INVALID_ARGUMENT for a
     *   zero/rejected/wrong-instance receipt or invalid output, NOT_FOUND for an
     *   invalid instance or a never-issued/already-collected task.
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
     * Validation, ownership and timeout rules match ovphysx_wait_task(). An all-zero
     * pending_task means no task remains pending within this call's cutoff.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_wait_all(
        ovphysx_instance_handle_t handle, ovphysx_timeout_t timeout_ns,
        ovphysx_wait_task_result_t* out_wait_result);

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
