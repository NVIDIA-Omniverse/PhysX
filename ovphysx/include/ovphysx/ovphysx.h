// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause


// C-compatible PhysX API

#ifndef OVPHYSX_OVPHYSX_H
#define OVPHYSX_OVPHYSX_H

#include <stdint.h>
#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* General notes 
    * 
    * Version information: See ovphysx_get_version() and ovphysx_get_version_string() below.
    *
    * DLPack Integration:
    *   This API uses DLPack for tensor data exchange between the user and the system.
    *   Same-device paths are zero-copy where supported; cross-device paths use staging.
    *   DLPack is a stable, widely-adopted open standard for tensor interoperability.
    *   See dlpack.h for tensor format details.
    * 
    * Return values:
    *   Synchronous functions (blocking calls) return ovphysx_result_t (status).
    *   Asynchronous functions return ovphysx_enqueue_result_t (status, op_index).
    *   On failure, call ovphysx_get_last_error() on the same thread to retrieve the
    *   error message. The returned string is valid until the next ovphysx API call on
    *   that thread.
    * 
    * Stream ordered asynchronous execution:
    *   - Async operations execute in submission order on a single queue
    *   - Results appear as if operations completed serially (sequential consistency)
    *   - Use ovphysx_wait_op() to ensure operations have completed before using results
    *   - If an asynchronous operation fails to enqueue (error returned from the function call),
    *     the returned op index is invalid and cannot be used for synchronization.
    *   - Each op_index is SINGLE-USE: once ovphysx_wait_op reports completion or
    *     operation failure, the op_index is consumed and cannot be reused. A timeout
    *     does not consume an operation that remains pending. Concurrent waits on the
    *     same op_index from multiple threads have undefined behavior.
    * 
    * Failure handling:
    *   - If operation N fails, the error is recorded and returned via ovphysx_wait_op()
    *   - The error string will indicate whether subsequent operations will fail or succeed based on the failure of operation N
    *   - and whether the instance is left in a defined state or not.
    * 
    * Memory visibility:
    *   - Writes from operation N are visible to operation N+1
    *   - Independent operations may execute concurrently internally for performance
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
     *   - Instances share the underlying omni.physx runtime and attached stage.
     *     Serialize simulation, stage mutation, and binding creation across
     *     instances.
     * 
     * Single instance, multiple threads:
     *   - Operations are NOT thread-safe for the same instance.
     *   - Use external synchronization if calling from multiple threads.
     * 
     * Data buffer lifetimes:
     *   - User is responsible for ensuring data passed to async operations remains valid and
     *     unmodified until the operation completes (as indicated by ovphysx_wait_op).
     * 
     * CUDA context requirements:
     *   - The SDK manages CUDA contexts internally where needed.
     *   - Users can call wait/sync functions from any thread without manual context management.
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
     * @defgroup ovphysx_tensor_binding Tensor bindings
     * Bulk tensor access APIs for simulation data.
     */

    /**
     * @defgroup ovphysx_stream Stream operations
     * Stream-ordered task submission and synchronization.
     */

    /**
     * @defgroup ovphysx_errors Error handling
     * Destroy error strings and error arrays.
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
     * @brief Create a new ovphysx instance.
     *
     * Initialize create_args with OVPHYSX_CREATE_ARGS_DEFAULT for sensible defaults:
     * @code
     * ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);  // optional: default is WARNING
     * ovphysx_initialize();
     * ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
     * ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
     * ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
     * @endcode
     *
     * @param create_args Configuration for the ovphysx instance (must not be NULL).
     *   If config_entry_count is nonzero, config_entries must not be NULL.
     * @param out_handle [out] ovphysx handle (must not be NULL).
     * @return ovphysx_result_t with status and error info.
     *
     * @pre create_args != NULL, out_handle != NULL.
     * @post On success, *out_handle is a valid handle that must be destroyed with ovphysx_destroy_instance().
     * @post With an active lifecycle, if config_entries is NULL and config_entry_count is
     *   nonzero, *out_handle is unchanged.
     *
     * @par Side Effects
     * Loads runtime components and initializes process-level state.
     *
     * @par Threading
     * Safe to call from any thread. The resulting handle is not thread-safe for concurrent use.
     *
     * @par Ownership
     * Caller owns the instance handle and must destroy it.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT on null required pointers, inconsistent config_entries /
     *   config_entry_count, or invalid active_cuda_gpus
     * - OVPHYSX_API_ERROR if ovphysx_initialize() is not active, or for other initialization failures
     *
     * @note active_cuda_gpus restricts which GPU ordinals are available for this instance.
     *   Per-scene device selection (CPU vs GPU dynamics) is owned by PhysX via physxScene:enableGPUDynamics
     *   in the USD stage. Use ovphysx_set_cpu_mode() before creating any instance to force process-wide
     *   CPU-only mode.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_instance(const ovphysx_create_args* create_args, ovphysx_handle_t* out_handle);

    /**
     * @brief Force process-wide CPU-only mode.
     *
     * To guarantee that no CUDA driver is touched for the lifetime of the process, set this to
     * true before the first instance is ever created. All subsequent PhysX scenes will use CPU
     * dynamics regardless of their USD physxScene:enableGPUDynamics setting.
     * For per-scene CPU control without this flag, author each scene explicitly
     * (physxScene:enableGPUDynamics=false + physxScene:broadphaseType="MBP").
     *
     * Requires that no instances are active. Returns OVPHYSX_API_ERROR if any instances
     * currently exist, or if attempting to set false after true has been applied. CPU-only
     * mode is sticky as soon as a call setting it to true succeeds.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_cpu_mode(bool cpu_only);

    /**
     * @brief Register ovphysx USD schema/plugin discovery paths before runtime initialization.
     *
     * This is intended for applications that share a namespaced OpenUSD runtime between multiple
     * subsystems, such as ovphysx and ovrtx. USD's schema registry is populated only once for the
     * process, so every subsystem that contributes schema/plugin paths must publish them before the
     * registry is first consulted, typically before the first USD stage is opened.
     *
     * The function appends ovphysx's bundled USD plugin root to the namespaced USD plugin-path
     * environment variable, `OV_PXR_PLUGINPATH_2511`. It preserves existing entries and does not
     * modify `PXR_PLUGINPATH_NAME`.
     *
     * Notes:
     * - Safe to call before @ref ovphysx_create_instance().
     * - Idempotent; the first successful call performs registration, subsequent calls are no-ops.
     * - Failed calls do not mark registration complete, so callers may fix the installation or
     *   `OVPHYSX_LIB` override and retry.
     * - This function does not initialize ovphysx, load USD, acquire Carbonite, or open a stage.
     * - Calling this after USD has already populated its schema registry has no retroactive effect.
     *
     * @return
     * - OVPHYSX_API_SUCCESS if the schema/plugin paths were registered successfully.
     * - OVPHYSX_API_ERROR if registration failed. Use @ref ovphysx_get_last_error() for details.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_register_schema_paths(void);

    /**
     * @brief Destroy an ovphysx instance and release per-instance resources.
     *
     * Carbonite and the static PhysX runtime stay resident until process exit.
     *
     * @param handle ovphysx handle to destroy.
     * @return ovphysx_result_t with status and error info.
     *
     * @pre handle must be a valid instance handle.
     * @post The handle is invalid and must not be reused.
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
     * - OVPHYSX_API_ERROR if destruction fails. No error string is returned; consult logs.
     *
     * @code
     * ovphysx_destroy_instance(handle);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_destroy_instance(ovphysx_handle_t handle);

    /**
     * @brief Clear the ovphysx process-lifecycle token.
     *
     * Clears the process-global initialized state set by @ref ovphysx_initialize.
     * It does not destroy live handles and does not balance
     * @ref ovphysx_create_instance; callers must destroy every handle explicitly
     * with @ref ovphysx_destroy_instance.
     *
     * Must be paired with a prior @ref ovphysx_initialize call. Call once when
     * the application is done with the current process-lifecycle scope. After
     * shutdown, callers must invoke @ref ovphysx_initialize again before creating
     * another instance.
     *
     * @return
     * - OVPHYSX_API_SUCCESS on success.
     * - OVPHYSX_API_ERROR if called without a matching @ref ovphysx_initialize.
     *   Use @ref ovphysx_get_last_error() for details.
     *
     * @par Static runtime mode
     * Static ovphysx keeps Carbonite, the direct PhysX runtime, and OmniClient
     * resident until process exit. This avoids re-entering plugin teardown from
     * application shutdown paths.
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
    /* Typed global config (process-wide)               */
    /*--------------------------------------------------*/

    /** @defgroup ovphysx_config Typed global config */
    /** @addtogroup ovphysx_config */
    /** @{ */

    /**
     * @brief Set a typed global config entry at runtime (process-global).
     *
     * IMPORTANT: Config is PROCESS-GLOBAL. Changes affect all ovphysx instances
     * in the current process. Configure before creating instances or loading USD
     * for predictable behavior.
     *
     * Use the builder functions in ovphysx_config.h for convenient construction:
     * @code
     * #include "ovphysx/ovphysx_config.h"
     * ovphysx_set_global_config(ovphysx_config_entry_num_threads(4));
     * ovphysx_set_global_config(ovphysx_config_entry_disable_contact_processing(true));
     * // Escape hatch for arbitrary Carbonite paths:
     * ovphysx_set_global_config(ovphysx_config_entry_carbonite(
     *     OVPHYSX_LITERAL("/physics/updateToUsd"), OVPHYSX_LITERAL("false")));
     * @endcode
     *
     * @param entry Typed config entry to apply.
     * @return ovphysx_result_t with status and error info.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_global_config(ovphysx_config_entry_t entry);

    /**
     * @brief Get a boolean config value.
     * @param key Boolean config key.
     * @param out_value [out] Current value.
     * @return ovphysx_result_t with status and error info.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_bool(
        ovphysx_config_bool_t key, bool* out_value);

    /**
     * @brief Get an int32 config value.
     * @param key Int32 config key.
     * @param out_value [out] Current value.
     * @return ovphysx_result_t with status and error info.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_int32(
        ovphysx_config_int32_t key, int32_t* out_value);

    /**
     * @brief Get a float config value.
     * @param key Float config key.
     * @param out_value [out] Current value.
     * @return ovphysx_result_t with status and error info.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_float(
        ovphysx_config_float_t key, float* out_value);

    /**
     * @brief Get a string config value into a user-provided buffer.
     * @param key String config key.
     * @param value_out [in/out] String with pre-allocated buffer (ptr+length).
     * @param out_required_size [out] Required buffer size including null terminator.
     * @return ovphysx_result_t with status and error info.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_string(
        ovphysx_config_string_t key, ovphysx_string_t* value_out, size_t* out_required_size);

    /** @} */

    /*--------------------------------------------------*/
    /* Stage building and introspection operations      */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_stage */
    /** @{ */


    /*
    * Enqueue an asynchronous operation to reset the runtime stage representation to an empty stage.
    * Any attached ovstage is detached, and all runtime simulation state is cleared.
    * Settings and other configuration not present in the stage will persist.
    * @param handle ovphysx instance
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    *
    * @pre handle must be a valid instance handle.
    * @post Stage is cleared when the op completes.
    *
    * @par Side Effects
    * Clears the runtime stage and associated simulation state.
    *
    * @par Errors
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_enqueue_result_t r = ovphysx_reset_stage(handle);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_reset_stage(ovphysx_handle_t handle);

    /**
    * @brief Attach an ovstage Stage as the orchestration data surface.
    *
    * Once attached, the orchestration contract is **explicit and application-owned**
    * in both directions (the application owns ordinal advancement):
    *
    *   - app → physics (control in): the app authors dirty control attributes into
    *     ovstage (drive:force, drive:velocity, drive:position_target, physics:mass,
    *     physics:gravityMagnitude, physics:gravityDirection, …) at ordinals it
    *     chooses, then calls @ref ovphysx_update_from_ovstage to drain that ordinal
    *     range into the running simulation. @ref ovphysx_step then integrates.
    *   - physics → app (output out): @ref ovphysx_step does **not** author simulation
    *     output back into the Stage on its own. The app reads the step's output with
    *     @ref ovphysx_read / @ref ovphysx_fetch_read_next and writes it back into
    *     ovstage at a separate, higher (physics-output) ordinal — the ordinal that
    *     update_from_ovstage never covers, so physics never reprocesses its own
    *     writes. See the ordinal-coupling section on @ref ovphysx_query.
    *
    * This is one consistent model: initial parse at attach, explicit control updates
    * via update_from_ovstage, and app-owned output writeback via the read API.
    * ovphysx tensor bindings remain available as a perf escape hatch for callers
    * that need direct control without going through ovstage.
    *
    * The attachment is init-style — call once per instance, before any
    * ovphysx_step(). Replacing the attached Stage requires
    * ovphysx_detach_ovstage() first.
    *
    * @param handle ovphysx instance handle.
    * @param stage Caller-owned ovstage Stage (`ovstage_instance_t*`).
    * @param read_ordinal Caller-owned sealed ovstage ordinal the initial scene
    *      parse reads at. The application owns ordinal advancement; this is the
    *      starting point (subsequent edits are drained via
    *      ovphysx_update_from_ovstage()).
    *
    * @pre handle is valid; ovphysx is not already attached to a Stage; stage
    *      is non-null.
    * @pre stage outlives the attachment. ovphysx captures this pointer and
    *      dereferences it on every ovphysx_update_from_ovstage() until
    *      ovphysx_detach_ovstage(); destroying the Stage between attach and detach
    *      is undefined behavior, not a recoverable error.
    * @post On success, subsequent ovphysx_update_from_ovstage() calls observe
    *       committed Stage writes through the runtime ovstage backend.
    *
    * @note Not thread-safe per instance. Like the rest of the per-instance API,
    *       the caller must serialize ovphysx_attach_ovstage() against any other
    *       call on the same handle; the already-attached check and the attach are
    *       not internally locked against concurrent foreground callers.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT if stage is null
    * - OVPHYSX_API_ERROR if already attached or the runtime attach fails
    *
    * @code
    * // A caller-owned ovstage instance, kept alive until ovphysx_detach_ovstage().
    * ovstage_instance_t* stage = my_ovstage_instance;
    * ovphysx_result_t r = ovphysx_attach_ovstage(handle, stage, read_ordinal);
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_attach_ovstage(ovphysx_handle_t handle,
                                                         ovstage_instance_t* stage,
                                                         ovstage_ordinal_t read_ordinal);

    /**
    * @brief Pull and apply committed ovstage edits over an explicit ordinal range.
    *
    * The caller is the producer that advanced ovstage ordinals and therefore owns
    * the range boundaries, passed as ovstage's own `ovstage_ordinal_range_t`. With
    * `has_start_ordinal == true` the closed range `[start_ordinal, end_ordinal]` is
    * drained; with `has_start_ordinal == false` only `end_ordinal` is drained (the
    * single-ordinal form). The selected changes are drained through the active
    * ovstage change feed and applied to the running simulation.
    *
    * @param handle ovphysx instance handle.
    * @param range ovstage ordinal range to drain (see ovstage_ordinal_range_t).
    *
    * @pre `handle` is valid; @ref ovphysx_attach_ovstage succeeded.
    * @pre when `range.has_start_ordinal`, `range.start_ordinal <= range.end_ordinal`.
    * @pre All selected writes are sealed by a completed write-floor operation
    *      covering `range.end_ordinal`. Waiting for
    *      `ovstage_population_apply_usd_changes()` only completes population;
    *      it does not advance the write floor.
    *
    * @note The initial `read_ordinal` was already parsed by
    *       ovphysx_attach_ovstage(). Normal incremental updates select only later
    *       ordinals; including it replays initial scene changes rather than only
    *       the new delta.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for an invalid range or handle
    * - OVPHYSX_API_ERROR if no ovstage is attached or the range drain fails
    */
    OVPHYSX_API ovphysx_result_t ovphysx_update_from_ovstage(ovphysx_handle_t handle,
                                                            ovstage_ordinal_range_t range);

    /**
    * @brief Detach the currently-attached ovstage Stage.
    *
    * Idempotent: calling on an unattached instance is a no-op success.
    * Clears registered interests and any output-buffer registrations, so a
    * subsequent ovphysx_attach_ovstage() to a different Stage starts clean.
    * After detach, stage-dependent calls such as ovphysx_update_from_ovstage() and
    * ovphysx_step() fail until a Stage is attached again. Detach invalidates the
    * stage's tensor and contact views. Do not read or write existing bindings;
    * destroy them and create replacements after attaching and realizing a stage again.
    *
    * @param handle ovphysx instance handle.
    *
    * @pre handle is valid.
    * @post On success, ovphysx is unattached.
    *
    * @par Errors
    * - OVPHYSX_API_ERROR for internal failures
    */
    OVPHYSX_API ovphysx_result_t ovphysx_detach_ovstage(ovphysx_handle_t handle);

    /*--------------------------------------------------*/
    /* Physics output read (ovstage) */
    /*--------------------------------------------------*/

    /**
     * @brief Open a query over the simulation's output objects of one simulated type.
     *
     * Mirrors the ovstage read idiom: the query is a handle (not a list). The
     * matched prims come back per group at read time as the interned
     * ovstage_read_group_t::prims.list (resolve via @ref ovphysx_query_shared_dictionary
     * or feed straight into the ovstage write path). Discover the produced
     * attributes / total prim count with @ref ovphysx_fetch_query_result. Pair every
     * successful query with @ref ovphysx_release_query.
     *
     * This read is ovstage-native and only meaningful when an ovstage Stage is
     * attached (@ref ovphysx_attach_ovstage); under any other attach it returns 0
     * objects.
     *
     * ## Reading output without reprocessing it (ordinal coupling)
     *
     * ovphysx never authors simulation output back into the attached Stage on its
     * own — the application owns ordinal advancement and the write-back. The loop
     * that keeps physics from consuming its own output is:
     *
     *   1. The app authors *control* edits (poses, targets, gravity, …) into
     *      ovstage at ordinals it chooses, e.g. ordinal `c`, and seals them.
     *   2. The app calls @ref ovphysx_update_from_ovstage with an
     *      `ovstage_ordinal_range_t` covering exactly those control ordinals (e.g.
     *      `{from, c, true}`) to drain them into the running sim, then @ref ovphysx_step.
     *   3. The app reads the step's *output* with this API and writes it back into
     *      ovstage at a SEPARATE, higher ordinal `p` (the physics-output ordinal).
     *      The group's `prims.list` + `attribute` token + `data.tensors` feed the
     *      ovstage write path with no repack and no string round-trip.
     *   4. The next frame's @ref ovphysx_update_from_ovstage range must cover the
     *      app's new control ordinals but **exclude** the physics-output ordinals
     *      `p`. Because physics never feeds `p` back into update_from_ovstage, it
     *      never reprocesses its own writes.
     *
     * In short: **app→physics edits flow through `ovphysx_update_from_ovstage`;
     * physics→app output is written at ordinals that update_from_ovstage never
     * covers.** See the ovstage usage guide for a worked example.
     *
     * ## Query semantics: a reusable selector, evaluated lazily
     *
     * A query is a SELECTOR (object_type + scope), NOT a captured membership
     * snapshot. Matched prims and column values are resolved lazily — the prim
     * count at @ref ovphysx_fetch_query_result, the column data at @ref ovphysx_read
     * — and each observes the most recently completed step (this call and the read
     * both drain pending sim first). A step that lands between ovphysx_query and
     * ovphysx_read does not make the handle stale: the read simply reflects the
     * newer step. For OVPHYSX_SCOPE_ACTIVE that means "the active set of whatever
     * step had last completed when ovphysx_read ran" — read before stepping again
     * to capture a specific step's active set. The current output producer returns
     * `layout_generation == 0`; rebuild cached queries and layouts after a known
     * structural change rather than using it as an invalidation signal.
     *
     * @param handle ovphysx instance handle.
     * @param object_type Simulated type to select (@ref ovphysx_sim_object_type_t).
     * @param scope @ref ovphysx_object_scope_t (all / active — active is single-frame).
     * @param out_query [out] Receives the query handle. Nonzero on success — INCLUDING
     *      an empty match (a valid query whose read reaches end-of-iteration
     *      immediately and whose fetch_query_result reports total_prim_count == 0).
     *      0 means FAILURE only.
     * @return ovphysx_result_t.
     *
     * @pre handle is valid; an ovstage Stage is attached.
     * @post On success `*out_query` is non-zero and usable with ovphysx_read.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_query is null
     * - OVPHYSX_API_ERROR if no ovstage is attached or the query could not be built
     */
    OVPHYSX_API ovphysx_result_t ovphysx_query(ovphysx_handle_t handle,
                                                      ovphysx_sim_object_type_t object_type,
                                                      ovphysx_object_scope_t scope,
                                                      ovphysx_query_handle_t* out_query);

    /**
     * @brief Fetch a query's discovery summary (attributes + total prim count).
     *
     * Fills ovstage's own `ovstage_query_result_t`. Its `attributes` array lists the
     * interned attribute tokens the matched objects produce (resolve via
     * @ref ovphysx_query_shared_dictionary, or feed straight back into @ref ovphysx_read as
     * `ovx_string_or_token_t` tokens); the array is owned by the query and valid
     * until @ref ovphysx_release_query. `total_prim_count == 0` is the empty-match
     * case.
     *
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param out_result [out] Receives the discovery summary.
     * @return ovphysx_result_t.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_result is null
     * - OVPHYSX_API_ERROR for a bad query handle / no ovstage attached
     */
    OVPHYSX_API ovphysx_result_t ovphysx_fetch_query_result(ovphysx_handle_t handle,
                                                            ovphysx_query_handle_t query,
                                                            ovstage_query_result_t* out_result);

    /**
     * @brief Get the **shared** ovstage path dictionary backing a query.
     *
     * This is NOT an ovphysx-owned dictionary. The returned pointer is the attached
     * ovstage source's own `path_dictionary_instance_t*` — the very dictionary that
     * interned this query's `prim_list` handles and `attribute` tokens, AND the
     * dictionary the ovstage write path interns into. Because read and write share
     * it, a group's `attribute` token / `prim_list` handle can be fed straight back
     * into the ovstage write path with no rebuild and no string round-trip; you only
     * need this accessor when you want to resolve a token to a human-readable string
     * or to intern a *derived* name (e.g. renaming output to "sim:<name>").
     *
     * The pointer is declared in `<ovx/path_dictionary/path_dictionary.h>` and
     * surfaced here as an opaque `void*` (that header defines C++-only inline
     * helpers, so it is deliberately not pulled into this C surface). Owned by the
     * runtime; do not free. Returns NULL for a non-ovstage backend.
     *
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param out_dictionary [out] Receives the opaque dictionary pointer (NULL if none).
     * @return ovphysx_result_t.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_dictionary is null
     * - OVPHYSX_API_ERROR for a bad query handle / no ovstage attached
     */
    OVPHYSX_API ovphysx_result_t ovphysx_query_shared_dictionary(ovphysx_handle_t handle,
                                                                 ovphysx_query_handle_t query,
                                                                 void** out_dictionary);

    /**
     * @brief Read named output attributes for a query into typed column groups.
     *
     * Opens a read session over `query` for the requested attributes, each given as
     * an `ovx_string_or_token_t` — a string name (see OVPHYSX_ATTR_*) OR an interned
     * token (e.g. from @ref ovphysx_fetch_query_result, fed straight back with no
     * token→string round-trip). Iterate the result with @ref ovphysx_fetch_read_next,
     * release each consumed group with @ref ovphysx_release_group, and release the
     * session with @ref ovphysx_release_read. Names not produced by the queried type
     * are skipped.
     *
     * ## Requested name vs. emitted `group.attribute`
     *
     * The name you pass here is the SEMANTIC name you want (e.g. "position"). The
     * resulting group's `attribute` token is the name actually EMITTED, which is the
     * name that feeds the ovstage write path verbatim — and the two are not always
     * spelled identically. A single-prim "position"/"orientation" request on a
     * point-instancer is emitted as the instancer-array attribute
     * "positions"/"orientations" (singular → plural) in INSTANCER-LOCAL space, since
     * that is what writes back onto the instancer prim. Deformable/particle "points"
     * are emitted mesh/set-local. Velocities stay world-space. So: request by intent,
     * write back using `group.attribute` (+ `group.semantic` for the coordinate role)
     * — do not assume the emitted token equals the requested string.
     *
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param attributes Array of `attribute_count` `ovx_string_or_token_t` (string
     *      name or interned token per entry).
     * @param attribute_count Number of attributes.
     * @param out_read [out] Receives the read-session handle (0 on failure).
     * @return ovphysx_result_t.
     *
     * @pre handle and query are valid.
     * @post On success `*out_read` is non-zero.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if args are null
     * - OVPHYSX_API_ERROR for internal failures
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read(ovphysx_handle_t handle,
                                                      ovphysx_query_handle_t query,
                                                      const ovx_string_or_token_t* attributes,
                                                      size_t attribute_count,
                                                      ovphysx_read_handle_t* out_read);

    /**
     * @brief Fetch the next output column group from a read session.
     *
     * On success points `*out_group` at the next `ovstage_read_group_t` and returns
     * OVPHYSX_API_SUCCESS. Returns OVPHYSX_API_END_OF_ITERATION (NOT an error) once
     * all groups have been consumed — `*out_group` is then NULL. Any other status is
     * a real error (`*out_group` NULL).
     *
     * The group is **producer-owned** (the caller does not allocate it): the returned
     * `ovstage_read_group_t` pointer is a borrow valid until @ref ovphysx_release_group
     * is called for that group's `read_group_id`, or the session is released.
     *
     * Group lifetime (authoritative): the struct AND its borrowed
     * `data.tensors` / `data.index_map` / `data.mask` / `prims.list` stay valid until
     * @ref ovphysx_release_group for that `read_group_id` (or @ref ovphysx_release_read,
     * which releases all). Fetching further groups does NOT invalidate earlier ones,
     * and an intervening @ref ovphysx_step does NOT invalidate a live group (the
     * runtime gathers each column into session-owned storage at read time). Copy any
     * tensor you need to outlive the group's release.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle from @ref ovphysx_read.
     * @param out_group [out] Receives a borrowed `ovstage_read_group_t*` for the next
     *      group (NULL at end of iteration or on error).
     * @return ovphysx_result_t: SUCCESS (group filled), END_OF_ITERATION (done), else error.
     *
     * @pre handle and read are valid; out_group is non-null.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_group is null
     * - OVPHYSX_API_ERROR for a bad read handle
     */
    OVPHYSX_API ovphysx_result_t ovphysx_fetch_read_next(ovphysx_handle_t handle,
                                                         ovphysx_read_handle_t read,
                                                         const ovstage_read_group_t** out_group);

    /**
     * @brief Release one fetched group's borrowed storage.
     *
     * Releases the column / prim storage pinned by @ref ovphysx_fetch_read_next for
     * `group_id` (an `ovstage_read_group_t::read_group_id`). After this the group's
     * tensors / index_map / prims.list must not be dereferenced.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle the group came from.
     * @param group_id The group's `ovstage_read_group_t::read_group_id`.
     * @return ovphysx_result_t. Idempotent for an already-released / unknown id.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_release_group(ovphysx_handle_t handle,
                                                       ovphysx_read_handle_t read,
                                                       ovstage_read_group_id_t group_id);

    /**
     * @brief Release a read session (and every borrowed group it still owns).
     * @param handle ovphysx instance handle.
     * @param read Read-session handle from @ref ovphysx_read.
     * @return ovphysx_result_t. Idempotent for an already-released / unknown handle.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_release_read(ovphysx_handle_t handle,
                                                      ovphysx_read_handle_t read);

    /**
     * @brief Release an output query.
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @return ovphysx_result_t. Idempotent for an already-released / unknown handle.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_release_query(ovphysx_handle_t handle,
                                                       ovphysx_query_handle_t query);

    /**
    * @brief Clone the subtree under the source path to one or more target paths in the
    * internal physics representation (USD untouched).
    * The source path must exist in the stage.
    * The target paths must not already exist in the stage.
    *
    * Clones are created in the internal representation only (USD file will not be modified)
    * and immediately participate in physics simulation. Backed by the PhysX SDK replicator, so
    * cloned articulations are real articulations. Optimized for RL mass replication (1000s of
    * instances). This is the clone entrypoint for both standalone callers and callers using an
    * ovstage Stage (@ref ovphysx_attach_ovstage).
    *
    * @note Cross-environment collision isolation uses PhysX environment ids under GPU dynamics +
    *       GPU broadphase, controlled by `/ovphysx/clone/useEnvIds` (default on, per-process). The
    *       source holds env id 0 and clones get 1..N, so co-located clones (NULL parent_transforms)
    *       are isolated from the source too. USD collision groups/filtering authored before cloning
    *       still work for finer control. Pass `env_ids` when one logical environment is assembled
    *       from several clone calls, so its objects share an id.
    *
    * @param handle PhysX instance handle
    * @param source_path_in_usd Path to the source subtree to clone (must exist)
    * @param target_paths Array of target paths to clone to (must not exist)
    * @param num_target_paths Number of target paths to clone to
    * @param parent_transforms World pose of each copy's parent. Flat [num_target_paths x 7]
    *        floats: (px, py, pz, qx, qy, qz, qw), quaternion imaginary-first
    *        (OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, identity = (0,0,0,1)). Each body keeps its pose
    *        relative to the source's parent (world = parent_transforms[i] *
    *        inverse(source_parent) * source_body). Pass NULL to co-locate every copy on the source.
    * @param env_ids Optional logical environment id per target ([num_target_paths] uint32).
    *        Stable across calls: a shared id maps to one runtime environment (clones sharing it
    *        collide, isolated from others). Ids must be < 0x00FFFFFF (runtime id is env_ids[i] + 1).
    *        Pass NULL for automatic per-call numbering.
    *
    * @return ovphysx_enqueue_result_t with status and operation index for the clone. On failure,
    *         call ovphysx_get_last_error() on the same thread for the error message.
    *
    * @note Replication executes inline. On success, the returned operation index is already
    *       complete; ovphysx_wait_op() remains valid and returns immediately.
    *
    * @pre handle must be valid; an ovstage source is attached via @ref ovphysx_attach_ovstage.
    * @pre source_path_in_usd must exist; target_paths must be valid, unique, and disjoint --
    *      a target that reuses, contains, or is contained by another target or an earlier
    *      clone's target on this attach is rejected (it would create duplicate actors).
    * @pre Must be called **before** the first ovphysx_step() / ovphysx_step_sync() /
    *      ovphysx_step_n_sync() call. Violating this is rejected with
    *      OVPHYSX_API_INVALID_ARGUMENT in both CPU and GPU mode.
    * @pre Must also be called **before** warmup_gpu(), but this precondition is
    *      currently GPU-only: in hard CPU mode warmup_gpu() is a no-op, so
    *      clone-after-warmup still succeeds on CPU today. Cloning after GPU
    *      warmup reallocates GPU buffers and would corrupt already-initialised
    *      state, hence the rejection on GPU.
    * @post Cloned physics objects are live when this call returns successfully.
    *
    * @par Side Effects
    * Adds live PhysX objects keyed by each target path. No USD or runtime-stage prims are authored.
    *
    * @par Ownership
    * The target_paths array is read during the call; caller retains ownership.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid or duplicate targets, or a call
    *   after GPU warmup / the first step
    * - OVPHYSX_API_ERROR if no ovstage is attached or the clone fails
    *
    * @code
    * #include <ovphysx/ovphysx.h>
    *
    * static ovphysx_enqueue_result_t clone_two_environments(ovphysx_handle_t handle)
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
        ovphysx_handle_t handle,
        ovphysx_string_t source_path_in_usd,
        ovphysx_string_t* target_paths,
        uint32_t num_target_paths,
        const float* parent_transforms,
        const uint32_t* env_ids
    );

    /** @} */

    /*--------------------------------------------------*/
    /* Simulation operations */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_simulation */
    /** @{ */

    /*
    * Enqueue an asynchronous physics simulation step.
    * The simulation time is tracked internally; each step advances it by step_dt.
    * @param handle ovphysx instance
    * @param step_dt Simulation timestep in seconds
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    *
    * @pre handle must be a valid instance handle.
    * @post Simulation advances by step_dt when the op completes.
    *
    * @par Side Effects
    * Mutates physics state; may trigger internal events and callbacks.
    *
    * @par Threading
    * Must not be called concurrently on the same instance without external synchronization.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_enqueue_result_t r = ovphysx_step(handle, 1.0f / 60.0f);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_step(ovphysx_handle_t handle,
                                                              float step_dt);

    /**
     * @brief Synchronous step: simulate one physics timestep and wait for
     * completion in a single call.
     *
     * Functionally equivalent to ovphysx_step() followed by ovphysx_wait_op()
     * on the returned operation index, but bypasses the async event machinery
     * entirely (mutex acquisitions, operation map insert/lookup/cleanup).
     * This is a measurable performance improvement for the common synchronous
     * use case: in IsaacLab RL training at 4096 environments, step_sync saves
     * ~0.2 ms per substep compared to step() + wait_op(), recovering roughly
     * 5-6% of total throughput.
     *
     * Use this whenever you step and immediately wait for results (i.e. you do
     * not overlap GPU simulation with CPU work between dispatch and fetch).
     *
     * The simulation time is tracked internally; each step advances it by
     * step_dt.
     *
     * @param handle  Physics instance handle.
     * @param step_dt  Timestep [s].
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_handle_t handle,
                                                            float step_dt);

    /**
     * Run n_steps consecutive physics steps in a single C call.
     * Step i is executed with duration step_dt at the internally-tracked
     * simulation time + i * step_dt.  This saves (n_steps-1) ctypes
     * round-trips for workloads that use decimation (one RL step =
     * multiple physics steps).  The internal counter advances by
     * n_steps * step_dt.
     *
     * @param handle       Physics instance handle.
     * @param n_steps      Number of steps to run (must be > 0).
     * @param step_dt      Duration of each step [s].
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_step_n_sync(ovphysx_handle_t handle,
                                                              int32_t n_steps,
                                                              float step_dt);

    /**
     * Recompute articulation link transforms from the current articulation
     * generalized coordinates without running a normal simulation step.
     *
     * This is a synchronous kinematic forward-kinematics update. It is useful
     * after writing articulation DOF positions through TensorBindingsAPI and
     * before reading link pose tensors in the same frame.
     *
     * GPU MODE WARNING: On the first GPU kinematic update after loading USD, an
     * automatic warmup simulation step may be performed to initialize PhysX
     * DirectGPU buffers. See @ref ovphysx_gpu_tensor_auto_warmup_note
     * "GPU tensor auto-warmup note".
     *
     * Once GPU warmup is complete, the FK refresh itself does not run collision
     * detection, integration, solver work, or contact generation.
     *
     * @param handle Physics instance handle.
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_update_articulations_kinematic(ovphysx_handle_t handle);

    /** @} */


    /*--------------------------------------------------*/
    /* Tensor Binding API                                */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /*
     * The Tensor Binding API provides efficient bulk access to physics simulation data.
     * It maps physics-object path patterns to typed tensor views, including authored USD objects
     * and runtime-only clones. This enables same-device zero-copy data exchange and transparent
     * cross-device staging with tensors (PyTorch, NumPy, etc.) via DLPack.
     * 
     * DLPack compatibility:
     *   We vendor the official DLPack header (see dlpack/dlpack.h from github.com/dmlc/dlpack).
     *   When constructing DLTensor structs for read/write calls:
     *   - strides may be NULL to indicate C-contiguous layout, OR
     *     you may set explicit C-contiguous strides. Both are accepted.
     *   - Non-contiguous tensors (e.g. transposed views, sliced views with gaps) are
     *     NOT supported and will be rejected. You can fix this by e.g. calling .contiguous()
     *     on PyTorch tensors first.
     *   - Supported dtypes: float32, int32, uint32, uint8, bool (kDLBool bits=8).
     *   - Supported devices: kDLCPU, kDLCUDAHost, kDLCUDA, and kDLCUDAManaged.
     *     CPU/CUDA device mismatches are handled by transparent staging when CUDA is available.
     *
     * Works in both GPU mode and CPU mode.
     * Process-wide CPU-only mode (ovphysx_set_cpu_mode(true) or OVPHYSX_DISABLE_GPU is set)
     * rejects kDLCUDA and kDLCUDAManaged tensors with OVPHYSX_API_DEVICE_MISMATCH
     * before accessing CUDA. kDLCPU and kDLCUDAHost remain host-accessible.
     * 
     * GPU MODE NOTE: GPU tensor reads require at least one simulation step() after loading USD.
     * This is a PhysX DirectGPU API requirement. Without a warmup step, reads may fail
     * or return stale data. CPU mode does not have this requirement.
     * 
     * Typical workflow (GPU mode):
     *   1. ovphysx_create_instance() (GPU is the default; call ovphysx_set_cpu_mode(true)
     *      before creating any instance for process-wide CPU-only mode)
     *   2. ovphysx_attach_ovstage() at the initial ordinal; use
     *      ovphysx_update_from_ovstage() only for later authored ordinals
     *   3. ovphysx_create_tensor_binding() for each tensor type needed
     *   4. Optional: ovphysx_warmup_gpu() to control when GPU warmup happens
     *      (otherwise it will happen automatically on first tensor read)
     *   5. Main loop: read/write tensors, step, wait
     * 
     * Pattern matching:
     *   Patterns use USD-style path glob syntax over realized physics objects:
     *   - Exact path: "/World/robot" - matches one object
     *   - Wildcard:   "/World/robot*" - matches robot1, robot2, robotArm, etc.
     *   - Nested:     "/World/env[N]/robot" with [N] as wildcard - matches /World/env0/robot, etc.
     */

    /**
     * Create a tensor binding for bulk data access (synchronous).
     * 
     * A tensor binding connects a physics-object path pattern (e.g., "/World/robot*") to a
     * tensor type (e.g., OVPHYSX_TENSOR_RIGID_BODY_POSE_F32), enabling efficient
     * bulk read/write of physics data for all matching objects. Runtime-only clone
     * paths are eligible even when no USD prim is authored at the path.
     * 
     * If the pattern matches zero physics objects, the binding is still created successfully
     * with element_count = 0. This lets callers treat optional scene content as
     * an empty current result instead of an error. Empty bindings do not update
     * when matching physics objects are added or recreated; destroy the old binding and
     * create a new one after topology changes.
     *
     * Binding lifetime is tied to the currently realized physics objects. The
     * application owns the stage lifecycle: if it will call ovphysx_reset_stage(),
     * remove USD data containing bound objects, or otherwise replace/reparse the
     * stage so those objects are destroyed and recreated, cached bindings should
     * be destroyed before the lifecycle operation when practical. If a stale
     * binding survives, only destroy it; do not read or write through it. Create
     * replacement bindings after the operation completes. ovphysx_step(),
     * ovphysx_step_sync(), and ovphysx_step_n_sync() do not invalidate bindings.
     * 
     * @param handle Instance handle
     * @param desc Binding descriptor with pattern and tensor_type
     * @param out_binding_handle [out] Binding handle on success
     * @return ovphysx_result_t (synchronous - completes before returning)
     *
     * @pre handle, desc, and out_binding_handle must be valid.
     * @post Binding handle owns native resources until explicitly destroyed via
     *       ovphysx_destroy_tensor_binding(), or until the parent instance is
     *       destroyed. Stage reset or bound-object removal invalidates the
     *       underlying TensorAPI view; destroy stale bindings and create
     *       replacements after the lifecycle operation completes.
     *
     * @par Diagnostics
     * Pattern bindings quiet expected TensorAPI no-match diagnostics on the
     * simulation view used to create that binding. Explicit prim_paths keep the
     * default error-level no-match diagnostics for typo detection. For
     * programmatic partial-miss checks with explicit prim_paths, compare the
     * requested paths with ovphysx_tensor_binding_get_prim_paths().
     *
     * @par Threading
     * Do not create bindings concurrently with stage mutation.
     *
     * @par Side Effects
     * Allocates internal binding resources.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t create_pose_binding(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t* out_binding)
     * {
     *     ovphysx_tensor_binding_desc_t desc = {
     *         .pattern = OVPHYSX_LITERAL("/World/robot*"),
     *         .tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32,
     *     };
     *     return ovphysx_create_tensor_binding(handle, &desc, out_binding);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_tensor_binding(
        ovphysx_handle_t handle,
        const ovphysx_tensor_binding_desc_t* desc,
        ovphysx_tensor_binding_handle_t* out_binding_handle);

    /**
     * Destroy a tensor binding and release associated resources (synchronous).
     * 
     * @param handle Instance handle
     * @param binding_handle Binding to destroy
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @post Binding handle is invalid after call.
     *
     * @par Side Effects
     * Releases internal resources.
     *
     * @par Errors
     * - OVPHYSX_API_NOT_FOUND if binding handle is unknown
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_destroy_tensor_binding(handle, binding);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_destroy_tensor_binding(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle);

    /**
     * Get complete tensor specification for a binding (preferred).
     * 
     * Returns dtype, ndim, and shape needed to allocate a compatible DLTensor.
     * This is the preferred API for constructing DLTensors correctly.
     * 
     * NOTE: ovphysx_tensor_spec_t stores shape in a fixed-size int64[4] for a stable C ABI.
     * Only the first ndim entries are meaningful; the remaining entries are always set to 0.
     *
     * See ovphysx_tensor_type_t documentation for shapes, dtype, and layouts per tensor type.
     * Layout is always row-major contiguous (C-order). Most bindings are float32;
     * OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32 is int32.
     * 
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_spec [out] Full tensor specification
     * @return ovphysx_result_t
     *
     * @pre handle, binding_handle, and out_spec must be valid.
     * @post out_spec is populated with dtype/shape for the binding.
     *
     * @par Side Effects
     * None.
     *
     * @par Errors
     * - OVPHYSX_API_NOT_FOUND if binding handle is unknown
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_tensor_spec_t spec;
     * ovphysx_get_tensor_binding_spec(handle, binding, &spec);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_tensor_binding_spec(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_tensor_spec_t* out_spec);

    /**
     * @anchor ovphysx_gpu_tensor_auto_warmup_note
     * @par GPU tensor auto-warmup note
     * In GPU mode, the first tensor read or write after loading USD may perform an
     * automatic warmup simulation step to initialize PhysX DirectGPU buffers.
     *
     * The warmup is a real physics step that advances simulation time by a minimal
     * timestep (~1ns). Physics state may change infinitesimally; this is not a dry run.
     *
     * For deterministic behavior, explicitly control warmup timing by loading USD,
     * waiting for completion, then calling ovphysx_warmup_gpu() before the first
     * tensor read or write. If you want the first observed state change to happen
     * under your chosen timestep instead, call ovphysx_step() explicitly with that dt.
     *
     * Because warmup is a real simulation step, a true "pre-warmup" GPU tensor state
     * cannot be observed. Calling ovphysx_warmup_gpu() explicitly only makes the timing
     * of that unavoidable step predictable.
     */

    /**
     * Read data from simulation into a user-provided DLTensor (synchronous).
     * 
     * GPU MODE WARNING: On the first GPU tensor read after loading USD, an automatic
     * warmup simulation step may be performed to initialize PhysX DirectGPU buffers.
     * See @ref ovphysx_gpu_tensor_auto_warmup_note "GPU tensor auto-warmup note".
     * 
     * DLTensor requirements:
     *   - MUST be pre-allocated with correct shape (use ovphysx_get_tensor_binding_spec())
     *   - dtype must match ovphysx_get_tensor_binding_spec()
     *   - device must be supported; CPU/CUDA mismatches are staged when CUDA is available,
     *     while cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode
     *     return OVPHYSX_API_DEVICE_MISMATCH
     *   - layout must be contiguous row-major (C-order)
     * 
     * This is a blocking call that completes before returning.
     * 
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param dst_tensor Pre-allocated DLTensor with shape from ovphysx_get_tensor_binding_spec()
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre dst_tensor must be pre-allocated, match the spec's dtype/shape, and use a supported device.
     * @post dst_tensor is filled with simulation data on success.
     *
     * @par Side Effects
     * May trigger GPU warmup on first read in GPU mode.
     *
     * @par Ownership
     * Caller owns dst_tensor memory.
     *
     * @par Errors
     * - OVPHYSX_API_DEVICE_MISMATCH if tensor device is incompatible
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_NOT_FOUND if the binding is unknown or was invalidated by a stage change
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_read_tensor_binding(handle, binding, &tensor);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_tensor_binding(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        DLTensor* dst_tensor);

    /**
     * Explicitly initialize GPU buffers (optional, synchronous).
     * 
     * In GPU mode, PhysX DirectGPU buffers need one simulation step to initialize.
     * This is normally done automatically on the first tensor read (auto-warmup).
     * 
     * IMPORTANT: The warmup performs a real physics simulation step with a minimal
     * timestep (~1ns). While the effect is negligible, this means:
     *   - Simulation state is advanced (positions may change infinitesimally)
     *   - This is NOT a "dry run" - it mutates physics state
     *   - For deterministic initial conditions, call this before reading initial state
     * 
     * Call this function explicitly if you want to:
     *   - Control exactly when the warmup latency occurs
     *   - Avoid a latency spike on the first tensor read
     *   - Verify GPU initialization succeeded before starting your main loop
     *   - Ensure deterministic behavior by controlling when state mutation happens
     * 
     * This function is idempotent - calling it multiple times has no effect after
     * the first successful call (per stage). In CPU mode, this is a no-op.
     * 
     * Note: Warmup state is reset when the stage changes (e.g., after
     * ovphysx_reset_stage() or loading a new USD file). The next tensor read
     * will trigger warmup again.
     * 
     * @param handle Instance handle
     * @return ovphysx_result_t
     *
     * @pre handle must be valid.
     * @post GPU warmup completed for the active stage (if in GPU mode).
     *
     * @par Side Effects
     * Advances simulation by a minimal timestep in GPU mode.
     *
     * @par Errors
     * - OVPHYSX_API_GPU_NOT_AVAILABLE if GPU initialization fails
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_warmup_gpu(handle);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_warmup_gpu(ovphysx_handle_t handle);

    /**
     * Write data from a user-provided DLTensor into the simulation (synchronous).
     * 
     * Not all tensor types are writable:
     * - RIGID_BODY_FORCE_F32, RIGID_BODY_WRENCH_F32, ARTICULATION_LINK_WRENCH_F32 are WRITE-ONLY
     *   (external control inputs applied each step; reading them returns an error).
     * - RIGID_BODY_ACCELERATION_F32, RIGID_BODY_INV_MASS_F32, RIGID_BODY_INV_INERTIA_F32
     *   are READ-ONLY.
     * - ARTICULATION_LINK_POSE_F32, ARTICULATION_LINK_VELOCITY_F32, ARTICULATION_LINK_ACCELERATION_F32
     *   are READ-ONLY (no setter for individual link state).
     * - Dynamics query tensors (JACOBIAN, MASS_MATRIX, CORIOLIS_AND_CENTRIFUGAL_FORCE, GRAVITY_FORCE,
     *   LINK_INCOMING_JOINT_FORCE, DOF_PROJECTED_JOINT_FORCE, BODY_INV_MASS, BODY_INV_INERTIA)
     *   are READ-ONLY.
     * - DEFORMABLE_REST_NODAL_POSITION_F32 and DEFORMABLE_SIM_ELEMENT_INDICES_S32
     *   are READ-ONLY.
     * - DOF_ACTUATION_FORCE_F32 is read-write (not write-only).
     * See ovphysx_tensor_type_t documentation for shapes, layouts, and read/write semantics.
     *
     * GPU MODE WARNING: On the first GPU tensor write after loading USD, an automatic
     * warmup simulation step may be performed to initialize PhysX DirectGPU buffers.
     * See @ref ovphysx_gpu_tensor_auto_warmup_note "GPU tensor auto-warmup note".
     * 
     * This is a blocking call that completes before returning.
     * 
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param src_tensor User tensor with data to write (must match ovphysx_get_tensor_binding_spec())
     * @param index_tensor Optional int32[K] indices for subset write. NULL = write all.
     *   - When index_tensor != NULL: src_tensor must still have full shape [N, ...] matching the binding spec.
     *     Only the rows specified by index_tensor are written; other rows in src_tensor are ignored.
     *   - Indices are 0-based into the first dimension N of the binding, and must satisfy 0 <= idx < N.
     *   - K (index count) must satisfy K <= N.
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre src_tensor must match the spec's dtype/shape and use a supported device. CPU/CUDA
     *      mismatches are staged when CUDA is available; cross-GPU ordinal mismatches and CUDA
     *      tensors in process-wide CPU-only mode return OVPHYSX_API_DEVICE_MISMATCH.
     * @post Simulation state is updated with new values.
     *
     * @par Side Effects
     * Writes control or state data into the simulation.
     *
     * @par Ownership
     * Caller owns src_tensor and index_tensor memory.
     *
     * @par Errors
     * - OVPHYSX_API_DEVICE_MISMATCH if tensor device is incompatible
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_NOT_FOUND if the binding is unknown or was invalidated by a stage change
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_write_tensor_binding(handle, binding, &tensor, NULL);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_write_tensor_binding(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* src_tensor,
        const DLTensor* index_tensor);

    /**
     * Write data from a user-provided DLTensor into the simulation using a binary mask (synchronous).
     *
     * Only elements where mask[i] != 0 are written; other elements are left unchanged.
     * This is the mask-based alternative to indexed writes via ovphysx_write_tensor_binding.
     *
     * GPU MODE WARNING: On the first GPU tensor write after loading USD, an automatic
     * warmup simulation step may be performed to initialize PhysX DirectGPU buffers.
     * See @ref ovphysx_gpu_tensor_auto_warmup_note "GPU tensor auto-warmup note".
     *
     * @note There is intentionally no corresponding read_masked function. Reads always return
     *   the full [N,...] tensor via ovphysx_read_tensor_binding(); callers that need a subset
     *   can index the result on the host/device side. This write-only mask design matches
     *   other reinforcement-learning physics APIs (e.g. Newton's selectionAPI) where masks
     *   are used to selectively apply actions but observations are always returned in full.
     *
     * This is a blocking call that completes before returning.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param src_tensor User tensor with data to write. Must be full shape [N, ...] matching
     *   the dtype and shape from ovphysx_get_tensor_binding_spec().
     * @param mask_tensor Binary mask selecting which elements to update. Must be 1D with shape [N]
     *   where N matches the binding's first dimension. Dtype must be bool (kDLBool, bits=8) or
     *   uint8 (kDLUInt, bits=8).
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre src_tensor must match the dtype/shape of the binding spec and use a supported device.
     * @pre mask_tensor must be 1D uint8/bool with length N on a supported device. CPU/CUDA
     *      mismatches are staged when CUDA is available; cross-GPU ordinal mismatches and CUDA
     *      tensors in process-wide CPU-only mode return OVPHYSX_API_DEVICE_MISMATCH.
     * @post Simulation state is updated for masked elements only.
     *
     * @par Side Effects
     * Writes control or state data into the simulation for selected elements.
     *
     * @par Ownership
     * Caller owns src_tensor and mask_tensor memory.
     *
     * @par Errors
     * - OVPHYSX_API_DEVICE_MISMATCH if tensor device is incompatible
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_NOT_FOUND if the binding is unknown or was invalidated by a stage change
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_write_tensor_binding_masked(handle, binding, &tensor, &mask);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_write_tensor_binding_masked(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* src_tensor,
        const DLTensor* mask_tensor);

    /** @} */

    /*--------------------------------------------------*/
    /* Articulation metadata queries                     */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /**
     * @brief Get all scalar topology metadata for an articulation binding in one call.
     *
     * Fills out_metadata with dof_count, body_count, joint_count, fixed_tendon_count,
     * spatial_tendon_count, and is_fixed_base.  All values are stable for the binding
     * lifetime; cache the result if calling more than once.
     *
     * **Homogeneous topology requirement**: all articulations covered by this binding
     * must have the same topology (same dof_count, body_count, joint_count, etc.).
     * This is a fundamental constraint of the native tensor backend -- tensor shapes are fixed at
     * binding creation time.  If you need to work with articulations of different sizes
     * (e.g. a 7-DOF arm and a 30-DOF humanoid), create a separate binding for each.
     *
     * For name arrays (DOF names, body names, joint names) use the corresponding
     * ovphysx_articulation_get_*_names functions.
     *
     * @param handle          Instance handle
     * @param binding_handle  Tensor binding (must be an articulation binding)
     * @param out_metadata    [out] Caller-allocated struct to fill
     * @return ovphysx_result_t
     *
     * @code
     * ovphysx_articulation_metadata_t meta;
     * ovphysx_get_articulation_metadata(handle, binding, &meta);
     * printf("DOFs: %d  Links: %d\n", meta.dof_count, meta.body_count);
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_articulation_metadata(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_articulation_metadata_t* out_metadata);

    /**
     * @brief Get DOF names for the articulation.
     *
     * String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding (must be an articulation binding)
     * @param out_names [out] Array of ovphysx_string_t to fill
     * @param max_names Capacity of out_names array; set to metadata.dof_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_articulation_get_dof_names(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_names,
        uint32_t max_names,
        uint32_t* out_count);

    /**
     * @brief Get body (link) names for the articulation.
     *
     * String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding (must be an articulation binding)
     * @param out_names [out] Array of ovphysx_string_t to fill
     * @param max_names Capacity of out_names array; set to metadata.body_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_articulation_get_body_names(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_names,
        uint32_t max_names,
        uint32_t* out_count);

    /**
     * @brief Get joint names for the articulation.
     *
     * String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding (must be an articulation binding)
     * @param out_names [out] Array of ovphysx_string_t to fill
     * @param max_names Capacity of out_names array; set to metadata.joint_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_articulation_get_joint_names(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_names,
        uint32_t max_names,
        uint32_t* out_count);

    /**
     * @brief Get resolved physics-object paths for a tensor binding.
     *
     * The returned array order matches row order for every `RIGID_BODY_*`
     * tensor read/write on the same binding. For `ARTICULATION_*` tensor
     * bindings, the returned paths are articulation root object paths in the
     * binding's first-dimension row order. ovphysx owns the returned string
     * storage; string pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array; must be at least binding
     *   count to receive all paths.
     * @param out_count [out] Actual number of paths written
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_tensor_binding_get_prim_paths(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /** @} */

    /*--------------------------------------------------*/
    /* Contact binding API                               */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_contact_binding Contact bindings
     *
     * Read-only APIs for querying **aggregate contact force tensors** between
     * sensor and filter bodies. Returns DLPack tensors shaped `[S, 3]` (net
     * forces) or `[S, F, 3]` (force matrix), suitable for RL rewards, safety
     * limits, or force monitoring. GPU-compatible.
     *
     * For tensorized **per-contact-point geometry** (position, normal,
     * separation, force, and friction), use ovphysx_read_contact_data() and
     * ovphysx_read_friction_data(). Use ovphysx_get_contact_report() when you
     * need event headers or raw actor-pair report records.
     *
     * A **sensor** is a set of rigid bodies identified by a physics-object path
     * pattern that you pass to ovphysx_create_contact_binding(). A **filter**
     * is a second set of bodies whose contacts with each sensor you want to
     * measure. Both patterns can resolve authored USD objects and runtime-only
     * clones.
     *
     * Contact reporting is opt-in: every authored USD prim matched by a sensor
     * pattern must have `PhysxContactReportAPI` applied, on the prim named as the
     * sensor itself (not a parent body or child collider).  A matched prim
     * without the schema is dropped from the binding, and if that leaves no
     * sensors the call fails.  Filter prims need no extra schema, and
     * runtime-only clones inherit contact reporting from the source actor.
     *
     * **Lifecycle and read timing**
     *
     * A contact binding must be created *before* the first simulation step whose
     * contacts you want to observe.  After creation it registers sensors inside
     * the PhysX contact-report callback; no contact data exists until at least
     * one `ovphysx_step()`, `ovphysx_step_sync()`, or `ovphysx_step_n_sync()`
     * call has completed.
     *
     * - Call contact binding read functions *after* a stepping call completes.
     * - Reads reflect the contacts accumulated during the **last completed
     *   simulation step**.
     * - Calling a read function before any simulation step returns all-zero
     *   output tensors (no contacts have been reported yet).
     * - The `dt` for impulse-to-force conversion (`force = impulse / dt`) is
     *   automatically taken from the last successful `ovphysx_step()`,
     *   `ovphysx_step_sync()`, or `ovphysx_step_n_sync()` call; you do not need
     *   to pass it explicitly.
     *
     * **Why a separate binding type (not tensor binding)**
     *
     * Contact data has a fundamentally different shape and semantics from
     * articulation/rigid-body tensors:
     * - Shape is `[S, F, 3]` (sensor x filter x xyz), determined at binding
     *   creation time for aggregate force matrices, plus flat detailed contact
     *   buffers indexed by `[S, F]` count/start-index tensors.
     * - Contact binding is read-only; there is no write path.
     * - Internally it uses the PhysX contact-report callback rather than
     *   DirectGPU buffers, so sharing a handle type with tensor binding would
     *   misrepresent the lifetime and threading model.
     */

    /** @addtogroup ovphysx_contact_binding */
    /** @{ */

    /**
     * @brief Create a contact binding for aggregate and detailed contact reads.
     *
     * @param handle Instance handle
     * @param sensor_patterns Array of physics-object path patterns matching sensor bodies
     * @param sensor_patterns_count Number of sensor patterns
     * @param filter_patterns Flat array of filter object-path patterns. All sensors must
     *   have the same number of filters. Total length = sensor_patterns_count * filters_per_sensor.
     *   Pass NULL with filters_per_sensor=0 for unfiltered contacts.
     * @param filters_per_sensor Number of filter patterns per sensor (same for all sensors)
     * @param max_contact_data_count Maximum entries in raw or detailed
     *   contact/friction flat-buffer reads. Set this to a positive value before
     *   using ovphysx_read_contact_data(), ovphysx_read_friction_data(), or
     *   ovphysx_read_raw_contact_data(). Filtered detailed reads also require
     *   filters_per_sensor > 0. Aggregate net-force reads do not need detailed
     *   contact capacity or filters.
     * @param out_handle [out] Contact binding handle
     * @return ovphysx_result_t
     *
     * @post Binding handle is valid until explicitly destroyed via ovphysx_destroy_contact_binding(),
     *       or until the parent instance is destroyed.
     *
     * @code
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t create_filtered_contact_binding(
     *     ovphysx_handle_t handle,
     *     ovphysx_contact_binding_handle_t* out_binding)
     * {
     *     ovphysx_string_t sensors[] = {
     *         ovphysx_cstr("/World/robot_0/ee"),
     *     };
     *     ovphysx_string_t filters[] = {
     *         ovphysx_cstr("/World/obstacles/box"),
     *     };
     *     return ovphysx_create_contact_binding(
     *         handle, sensors, 1, filters, 1, 256, out_binding);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_contact_binding(
        ovphysx_handle_t handle,
        const ovphysx_string_t* sensor_patterns,
        uint32_t sensor_patterns_count,
        const ovphysx_string_t* filter_patterns,
        uint32_t filters_per_sensor,
        uint32_t max_contact_data_count,
        ovphysx_contact_binding_handle_t* out_handle);

    /**
     * @brief Destroy a contact binding.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding to destroy
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_destroy_contact_binding(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle);

    /**
     * @brief Query contact view dimensions.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_sensor_count [out] Number of sensor bodies matched
     * @param out_filter_count [out] Number of filter bodies per sensor
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_contact_binding_spec(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        int32_t* out_sensor_count,
        int32_t* out_filter_count);

    /**
     * @brief Get resolved sensor physics-object paths for a contact binding.
     *
     * The returned array order matches row order for contact binding reads.
     * ovphysx owns the returned string storage; string pointers remain valid
     * until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array; must be at least
     *   sensor_count to receive all sensor paths.
     * @param out_count [out] Actual number of paths written
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_contact_binding_get_sensor_paths(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /**
     * @brief Get resolved filter physics-object paths for a contact binding.
     *
     * Paths are returned in row-major `[sensor, filter]` order with total
     * count `sensor_count * filter_count`. ovphysx owns the returned string
     * storage; string pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array; must be at least
     *   sensor_count * filter_count to receive all filter paths.
     * @param out_count [out] Actual number of paths written
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_contact_binding_get_filter_paths(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /**
     * @brief Query raw and detailed contact/friction flat-buffer capacity.
     *
     * This is the C dimension for `ovphysx_read_contact_data()`,
     * `ovphysx_read_friction_data()`, and `ovphysx_read_raw_contact_data()`
     * flat buffers. Allocate force/separation buffers as `[C, 1]` and
     * point/normal/friction buffers as `[C, 3]`. Filtered reads use `[S, F]`
     * count/start-index buffers; raw reads use `[S]` count/start-index buffers
     * and `[C]` actor IDs. `S` and `F` come from
     * `ovphysx_get_contact_binding_spec()`.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_max_contact_data_count [out] Raw or detailed contact/friction flat-buffer capacity
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_contact_binding_capacity(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        uint32_t* out_max_contact_data_count);

    /**
     * @brief Read net contact forces. dst shape: [S, 3] where S = sensor_count.
     *
     * The dt for impulse-to-force conversion is taken automatically from the
     * last successful ovphysx_step(), ovphysx_step_sync(), or
     * ovphysx_step_n_sync() call.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param dst_tensor Pre-allocated DLTensor with shape [S, 3]
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_contact_net_forces(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /**
     * @brief Read contact force matrix. dst shape: [S, F, 3].
     *
     * The dt for impulse-to-force conversion is taken automatically from the
     * last successful ovphysx_step(), ovphysx_step_sync(), or
     * ovphysx_step_n_sync() call.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param dst_tensor Pre-allocated DLTensor with shape [S, F, 3]
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_contact_force_matrix(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /**
     * @brief Read detailed contact data into flat buffers.
     *
     * Required shapes:
     * - contact_force_tensor: `[C, 1]` float32
     * - contact_point_tensor: `[C, 3]` float32
     * - contact_normal_tensor: `[C, 3]` float32
     * - contact_separation_tensor: `[C, 1]` float32
     * - contact_count_tensor: `[S, F]` int32 or uint32
     * - contact_start_indices_tensor: `[S, F]` int32 or uint32
     *
     * `C` is `ovphysx_get_contact_binding_capacity()`, `S` is sensor_count,
     * and `F` is filter_count. For each `(sensor, filter)` pair, the valid
     * detailed contact slice is:
     * `start = start_indices[s, f]`, `count = counts[s, f]`,
     * `data[start : start + count]`.
     * `C` and `F` must be positive; pass a positive max_contact_data_count
     * and filters_per_sensor > 0 when creating the binding. Count and
     * start-index tensors may be int32 or uint32.
     *
     * If the required number of entries exceeds `C`, this function returns
     * `OVPHYSX_API_BUFFER_TOO_SMALL`. The payload tensors must not be used,
     * but the count and start-index tensors contain the full required layout.
     * Set `max_contact_data_count` to the maximum element of
     * `start_indices + counts` when recreating the binding for subsequent
     * simulation steps. Recreating a binding does not recover the overflowing
     * step's payload.
     *
     * Contact force magnitudes use the timestep from the last successful
     * ovphysx_step(), ovphysx_step_sync(), or ovphysx_step_n_sync() call.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param contact_force_tensor Pre-allocated contact normal force magnitudes
     * @param contact_point_tensor Pre-allocated world-frame contact points
     * @param contact_normal_tensor Pre-allocated world-frame contact normals
     * @param contact_separation_tensor Pre-allocated contact separations
     * @param contact_count_tensor Pre-allocated count matrix
     * @param contact_start_indices_tensor Pre-allocated start-index matrix
     * @return ovphysx_result_t. Returns `OVPHYSX_API_BUFFER_TOO_SMALL` when
     *   `C` cannot hold all entries.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_contact_data(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* contact_force_tensor,
        DLTensor* contact_point_tensor,
        DLTensor* contact_normal_tensor,
        DLTensor* contact_separation_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor);

    /**
     * @brief Read detailed friction data into flat buffers.
     *
     * Required shapes:
     * - friction_force_tensor: `[C, 3]` float32
     * - friction_point_tensor: `[C, 3]` float32
     * - contact_count_tensor: `[S, F]` int32 or uint32
     * - contact_start_indices_tensor: `[S, F]` int32 or uint32
     *
     * `C`, `S`, and `F` have the same meanings as in
     * `ovphysx_read_contact_data()`. For each `(sensor, filter)` pair, use the
     * count/start-index tensors to index valid entries in the flat friction
     * buffers. `C` and `F` must be positive; pass a positive
     * max_contact_data_count and filters_per_sensor > 0 when creating the
     * binding. Count and start-index tensors may be int32 or uint32.
     *
     * If the required number of entries exceeds `C`, this function returns
     * `OVPHYSX_API_BUFFER_TOO_SMALL`. The payload tensors must not be used,
     * but the count and start-index tensors contain the full required layout.
     * Set `max_contact_data_count` to the maximum element of
     * `start_indices + counts` when recreating the binding for subsequent
     * simulation steps. Recreating a binding does not recover the overflowing
     * step's payload.
     *
     * Friction forces use the timestep from the last successful
     * ovphysx_step(), ovphysx_step_sync(), or ovphysx_step_n_sync() call.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param friction_force_tensor Pre-allocated world-frame friction forces
     * @param friction_point_tensor Pre-allocated world-frame friction points
     * @param contact_count_tensor Pre-allocated count matrix
     * @param contact_start_indices_tensor Pre-allocated start-index matrix
     * @return ovphysx_result_t. Returns `OVPHYSX_API_BUFFER_TOO_SMALL` when
     *   `C` cannot hold all entries.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_friction_data(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* friction_force_tensor,
        DLTensor* friction_point_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor);

    /**
     * @brief Read raw (unfiltered) contact data for a contact binding.
     *
     * Filter-less variant of @ref ovphysx_read_contact_data : returns every
     * contact involving each sensor body regardless of which other actor it
     * collided with, plus a per-contact "other actor ID" lookup so callers
     * can identify the contacting body via
     * @ref ovphysx_contact_binding_get_other_actor_paths_from_ids.
     *
     * Required shapes (C = max_contact_data_count, S = sensor_count):
     * - contact_force_tensor:        `[C, 1]` float32 — contact normal force magnitude
     * - contact_point_tensor:        `[C, 3]` float32 — contact point in world frame
     * - contact_normal_tensor:       `[C, 3]` float32 — contact normal in world frame
     * - contact_separation_tensor:   `[C, 1]` float32 — signed separation
     * - contact_count_tensor:        `[S]` int32/uint32 — number of contacts per sensor
     * - contact_start_indices_tensor:`[S]` int32/uint32 — flat-buffer start index per sensor
     * - other_actor_ids_tensor:      `[C]` int64/uint64 — opaque actor id for each contact
     *
     * The contact binding must be created with `max_contact_data_count > 0`.
     * No filter dimension is required; `filters_per_sensor` may be zero. The
     * dt for impulse-to-force conversion is taken automatically from the last
     * successful ovphysx_step(), ovphysx_step_sync(), or
     * ovphysx_step_n_sync() call.
     *
     * If the required number of entries exceeds `C`, this function returns
     * `OVPHYSX_API_BUFFER_TOO_SMALL`. The payload tensors, including
     * `other_actor_ids_tensor`, must not be used, but the count and
     * start-index tensors contain the full required layout. Set
     * `max_contact_data_count` to the maximum element of
     * `start_indices + counts` when recreating the binding for subsequent
     * simulation steps. Recreating a binding does not recover the overflowing
     * step's payload.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param contact_force_tensor Pre-allocated contact normal force magnitudes
     * @param contact_point_tensor Pre-allocated world-frame contact points
     * @param contact_normal_tensor Pre-allocated world-frame contact normals
     * @param contact_separation_tensor Pre-allocated contact separations
     * @param contact_count_tensor Pre-allocated per-sensor counts
     * @param contact_start_indices_tensor Pre-allocated per-sensor start indices
     * @param other_actor_ids_tensor Pre-allocated per-contact actor IDs
     * @return ovphysx_result_t. Returns `OVPHYSX_API_BUFFER_TOO_SMALL` when
     *   `C` cannot hold all entries.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_raw_contact_data(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* contact_force_tensor,
        DLTensor* contact_point_tensor,
        DLTensor* contact_normal_tensor,
        DLTensor* contact_separation_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor,
        DLTensor* other_actor_ids_tensor);

    /**
     * @brief Resolve actor IDs from @ref ovphysx_read_raw_contact_data to physics-object paths.
     *
     * Given a tensor of opaque actor IDs (the `other_actor_ids_tensor`
     * returned by @ref ovphysx_read_raw_contact_data), fills `out_paths` with
     * the corresponding physics-object paths in the same order. IDs whose actors
     * cannot be resolved (e.g. removed since the step) yield empty paths.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param ids_tensor Input tensor of actor IDs (`[N]` int64/uint64)
     * @param out_paths [out] Array of @ref ovphysx_string_t to fill. ovphysx
     *   owns the returned string storage; pointers remain valid until the
     *   next call to this function on the same binding (which replaces the
     *   cache) or until the binding is destroyed.
     * @param max_paths Capacity of `out_paths` array.
     * @param out_count [out] Actual number of paths written.
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_contact_binding_get_other_actor_paths_from_ids(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* ids_tensor,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /**
     * @brief Get raw contact report data for the current simulation step.
     *
     * Returns **per-contact-point event data** -- position, normal, impulse,
     * and separation for every contact point this step. Use this for custom
     * contact sensors, collision debugging, or per-point force analysis.
     *
     * For **aggregate force tensors** (net forces or force matrices between
     * sensor/filter body sets, delivered as DLPack tensors), see the Contact
     * Binding API: ovphysx_create_contact_binding().
     *
     * The header array describes contact pairs (which actors/colliders are
     * in contact); each header references a slice of the contact data array
     * containing per-contact-point information (position, normal, impulse,
     * separation).
     *
     * Prims involved in contacts must have `PhysxContactReportAPI` applied
     * in the USD stage for contacts to be reported.
     *
     * **Ownership:** The caller does NOT own the returned arrays. They are
     * read-only views into internal simulation buffers.  Copy any data you
     * need to keep beyond the current step.
     *
     * **Pointer lifetime / invalidation:** The returned pointers are valid
     * only until the next call that advances or tears down the simulation.
     * The following operations invalidate both arrays:
     *   - ovphysx_step() (the next simulation step overwrites the buffers)
     *   - ovphysx_reset_stage()
     *   - ovphysx_destroy_instance()
     *
     * @param handle Instance handle.
     * @param[out] out_event_headers Receives a pointer to the contact event
     *        header array (read-only, valid until next step).
     * @param[out] out_num_event_headers Number of headers in the array.
     * @param[out] out_contact_data Receives a pointer to the contact point
     *        array (read-only, valid until next step).
     * @param[out] out_num_contact_data Number of contact point entries.
     * @param[out] out_friction_anchors Optional. If non-NULL, receives a pointer
     *        to the friction anchor array. Each anchor has position[3] and
     *        impulse[3] in world space. Pass NULL to skip.
     * @param[out] out_num_friction_anchors Optional. If non-NULL, receives the
     *        friction anchor count. Pass NULL to skip.
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_contact_report(
        ovphysx_handle_t handle,
        const ovphysx_contact_event_header_t** out_event_headers,
        uint32_t* out_num_event_headers,
        const ovphysx_contact_point_t** out_contact_data,
        uint32_t* out_num_contact_data,
        const ovphysx_friction_anchor_t** out_friction_anchors,
        uint32_t* out_num_friction_anchors);

    /** @} */

    /*--------------------------------------------------*/
    /* PhysX object interop                             */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_physx_interop */
    /** @{ */

    /**
     * Get a raw PhysX SDK object pointer by USD prim path and type.
     *
     * Returns the underlying PhysX object as an opaque `void*`. The caller
     * must cast to the appropriate PhysX C++ type (see ovphysx_physx_type_t).
     *
     * A single function covers all PhysX object types -- no per-type variants
     * are needed.
     *
     * @param handle   ovphysx instance handle.
     * @param prim_path USD prim path (ovphysx_string_t; e.g. "/World/physicsScene",
     *        "/World/Cube", "/World/articulation"). Embedded NUL bytes are rejected.
     * @param physx_type Which PhysX object type to look up at that path. See
     *        ovphysx_physx_type_t for the mapping to PhysX C++ types.
     * @param[out] out_ptr Receives the PhysX pointer (NULL if not found).
     * @return ovphysx_result_t with status and error info.
     *
     * @pre A USD stage must be loaded and at least one simulation step
     *      completed (so PhysX objects exist). prim_path must be a valid
     *      absolute USD path.
     *
     * **Pointer lifetime:** Returned pointers are valid until the next call to
     * ovphysx_reset_stage() or ovphysx_destroy(). Calls to
     * ovphysx_step() do NOT invalidate existing pointers.
     * Do not call `release()` on returned pointers -- ovphysx owns them.
     *
     * **Thread safety:** PhysX APIs on returned pointers must only be called
     * between simulation steps -- specifically after wait_op() completes for
     * the preceding step and before the next ovphysx_step() call. Calling
     * PhysX APIs while a step is in-flight is a data race.
     *
     * **Shapes:** PxShape objects are reachable from a PxRigidActor pointer
     * via `PxRigidActor::getShapes()`. You can also query them directly with
     * `OVPHYSX_PHYSX_TYPE_SHAPE`.
     *
     * **PhysX SDK headers:** Casting the returned pointer requires the PhysX
     * SDK C++ headers (e.g. `PxScene.h`, `PxRigidDynamic.h`).  The ovphysx
     * SDK ships these headers under `include/physx/`; `find_package(ovphysx)`
     * sets `ovphysx_PHYSX_INCLUDE_DIR` to point there. No PhysX library
     * linking is needed.  ovphysx bundles PhysX 5.x -- obtain matching
     * headers from the open-source repository at
     * https://github.com/NVIDIA-Omniverse/PhysX (same repo that contains
     * ovphysx).  Use the headers from the same commit/release as the ovphysx
     * build to ensure ABI compatibility.
     *
     * @code
     * void* scene = NULL;
     * ovphysx_result_t r = ovphysx_get_physx_ptr(
     *     handle, OVPHYSX_LITERAL("/World/physicsScene"),
     *     OVPHYSX_PHYSX_TYPE_SCENE, &scene);
     * if (r.status == OVPHYSX_API_SUCCESS && scene) {
     *     // Cast: physx::PxScene* pxScene = static_cast<physx::PxScene*>(scene);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_physx_ptr(
        ovphysx_handle_t handle,
        ovphysx_string_t prim_path,
        ovphysx_physx_type_t physx_type,
        void** out_ptr);

    /**
     * @brief Classify an authored USD prim by its high-level TensorAPI object type.
     *
     * Returns the umbrella's view of what kind of simulation object lives at
     * `prim_path`: rigid body, articulation, articulation link, articulation
     * root link, articulation joint, or invalid. Unresolved paths yield
     * `OVPHYSX_OBJECT_TYPE_INVALID` with `OVPHYSX_API_SUCCESS` (the call
     * itself didn't fail, the path just isn't a known simulation object).
     *
     * @param handle Instance handle (must have a stage attached)
     * @param prim_path Authored USD prim path (embedded NUL bytes are rejected)
     * @param[out] out_type Receives the object type
     * @return OVPHYSX_API_SUCCESS on success, or OVPHYSX_API_ERROR if no
     *         stage is attached or the TensorAPI simulation view cannot be
     *         created.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_object_type(
        ovphysx_handle_t handle,
        ovphysx_string_t prim_path,
        ovphysx_object_type_t* out_type);

    /**
     * @brief Force kinematic propagation of root + DOF state into link buffers
     *        for every articulation in the binding, without running a sim step.
     *
     * Mirrors PhysX SDK's
     * `physx::PxArticulationReducedCoordinate::updateKinematic`. The
     * umbrella's `SimulationView.update_articulations_kinematic()` calls this
     * after writing dof-positions / root-transforms to flush the new state
     * into the link buffer so the next read-back of link transforms reflects
     * the writes without simulating.
     *
     * @param handle Instance handle
     * @param binding_handle Articulation tensor binding identifying the set of
     *        articulations to update.
     * @param flags Bitwise OR of @ref ovphysx_articulation_kinematic_flag_t
     *        values. Pass `OVPHYSX_ARTICULATION_KINEMATIC_POSITION` to
     *        propagate positions only, OR with VELOCITY to propagate both.
     * @return OVPHYSX_API_SUCCESS on success, OVPHYSX_API_INVALID_ARGUMENT if
     *         the binding is not an articulation binding, OVPHYSX_API_ERROR on
     *         engine failure or if one or more articulations in the binding
     *         could not be resolved (e.g. removed from the live stage).
     */
    OVPHYSX_API ovphysx_result_t ovphysx_articulation_update_kinematic(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        uint32_t flags);

    /**
     * Wake rigid bodies in a binding.
     *
     * Mirrors PhysX SDK's `physx::PxRigidDynamic::wakeUp`. Bodies that
     * still have `physx::PxActorFlag::eDISABLE_SIMULATION` set are
     * silently skipped (the engine refuses to wake disabled actors).
     *
     * Typical pair: clear @ref OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL
     * on an actor (re-add to simulation), then call this to bring the
     * actor active for the next simulate -- otherwise it sits in the
     * sleep state PhysX placed it in when the flag was first set.
     *
     * @param handle Instance handle
     * @param binding_handle Rigid body tensor binding identifying the set
     *        of bodies in scope.
     * @param indices Optional int32 DLTensor of indices into the binding,
     *        or NULL to wake every body in the binding.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if the
     *         binding is not a rigid-body binding, OVPHYSX_API_NOT_FOUND
     *         if the binding has been invalidated by a stage change,
     *         OVPHYSX_API_ERROR on engine failure.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_rigid_body_view_wake_up(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* indices);

    /**
     * Force rigid bodies in a binding to sleep.
     *
     * Mirrors PhysX SDK's `physx::PxRigidDynamic::putToSleep`. Sets the
     * sleep state on each body so it is excluded from the next solve unless
     * woken by a contact or an explicit @ref ovphysx_rigid_body_view_wake_up
     * call. Bodies that have `physx::PxActorFlag::eDISABLE_SIMULATION` set
     * are silently skipped.
     *
     * @param handle Instance handle
     * @param binding_handle Rigid body tensor binding identifying the set
     *        of bodies in scope.
     * @param indices Optional int32 DLTensor of indices into the binding,
     *        or NULL to put every body in the binding to sleep.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if the
     *         binding is not a rigid-body binding, OVPHYSX_API_NOT_FOUND
     *         if the binding has been invalidated by a stage change,
     *         OVPHYSX_API_ERROR on engine failure.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_rigid_body_view_sleep(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* indices);

    /**
     * Subscribe to PhysX object create/destroy notifications.
     *
     * Pair with ovphysx_get_physx_ptr() to manage the lifetime of cached PhysX
     * SDK pointers. On a destruction notification the caller MUST drop the
     * cached pointer before returning from the callback. On a creation
     * notification the caller should mark the prim path as dirty in their own
     * cache and call ovphysx_get_physx_ptr() to fetch the new pointer ONLY
     * after the triggering synchronous call returns, or after
     * ovphysx_wait_op() returns for async work -- never from inside the
     * callback itself (see Threading below).
     *
     * Subscriptions are PROCESS-GLOBAL, not per-instance. A single subscription
     * receives events from every ovphysx instance in the process. Multi-instance
     * callers that need to filter by stage must do so on their side -- the
     * prim_path delivered to the callback is the only identifier available.
     * The handle parameter is intentionally absent.
     *
     * Lifecycle: callbacks fire for changes that occur during simulation and
     * ovphysx_reset_stage()'s bulk teardown. The initial object population from
     * ovstage attach/update is NOT notified -- the caller already has that state
     * from setup.
     *
     * Known limitation: ovphysx_clone() does NOT currently fire
     * object_created notifications. Pointer caches that need to track cloned
     * objects must be refreshed after ovphysx_wait_op() returns on the
     * clone operation, not via this subscription. ovphysx_reset_stage() (and the
     * all-objects-destroyed callback) is the supported path for bulk pointer
     * invalidation.
     *
     * Threading: callbacks may fire from internal worker threads. Do NOT call
     * other ovphysx APIs from inside a callback -- defer that until the
     * triggering synchronous call returns, or until ovphysx_wait_op() returns
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
     * @param[in]  callbacks         Pointer to the callback set. The struct is
     *                               copied internally; the caller does not need
     *                               to keep it alive after this call returns.
     * @param[out] out_subscription  Receives the subscription ID on success.
     *                               Pass this to ovphysx_unsubscribe_object_changes()
     *                               to stop receiving notifications.
     * @return OVPHYSX_API_SUCCESS on success, or an error status.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_subscribe_object_changes(
        const ovphysx_object_change_callbacks_t* callbacks,
        ovphysx_subscription_id_t* out_subscription);

    /**
     * Unsubscribe from PhysX object change notifications.
     *
     * Stops delivery of further notifications for the given subscription ID.
     * After this call returns the subscription ID is consumed and must not be
     * reused.
     *
     * Idempotency: passing an already-unsubscribed or unknown subscription ID
     * returns OVPHYSX_API_NOT_FOUND. OVPHYSX_INVALID_SUBSCRIPTION_ID is
     * rejected with OVPHYSX_API_INVALID_ARGUMENT.
     *
     * @param subscription Subscription ID returned from
     *                     ovphysx_subscribe_object_changes().
     * @return OVPHYSX_API_SUCCESS on success, or an error status.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_unsubscribe_object_changes(
        ovphysx_subscription_id_t subscription);

    /** @} */

    /*--------------------------------------------------*/
    /* Scene queries                                    */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_scene_query Scene queries
     *
     * Raycast, sweep, and overlap queries against the physics scene.
     *
     * All three functions follow the same output pattern: hit results are
     * written to an **internal buffer** owned by the ovphysx instance. The
     * returned pointer is valid until the next scene query call on the same
     * instance (any of the three functions). This avoids two-call patterns
     * and is simple for language bindings.
     *
     * Path fields in the hit struct (collision, rigid_body, material) are
     * uint64-encoded SdfPaths matching ovphysx's internal path encoding. Use
     * the same encoding when comparing hit results against known prim paths.
     *
     * @pre A USD stage must be loaded and at least one simulation step
     *      completed for the scene to contain queryable objects.
     *
     * **Thread safety:** Scene query functions must only be called between
     * simulation steps -- after `wait_op()` completes and before the next
     * `ovphysx_step()`.
     */

    /** @addtogroup ovphysx_scene_query */
    /** @{ */

    /**
     * @brief Cast a ray and return hits.
     *
     * @param handle     Instance handle.
     * @param origin     Ray origin (world space, 3 floats).
     * @param direction  Normalized ray direction (3 floats).
     * @param distance   Maximum ray distance. Must be >= 0.
     * @param both_sides If true, test both sides of mesh triangles.
     * @param mode       CLOSEST (0 or 1 hit), ANY (0 or 1), or ALL.
     * @param[out] out_hits   Receives pointer to internal hit array.
     * @param[out] out_count  Number of hits in the array.
     * @return ovphysx_result_t
     *
     * @code
     * const ovphysx_scene_query_hit_t* hits = NULL;
     * uint32_t count = 0;
     * float origin[3] = {0, 10, 0};
     * float dir[3]    = {0, -1, 0};
     * ovphysx_raycast(handle, origin, dir, 100.0f, false,
     *                 OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
     * if (count > 0) {
     *     printf("Hit at distance %f\n", hits[0].distance);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_raycast(
        ovphysx_handle_t handle,
        const float origin[3],
        const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const ovphysx_scene_query_hit_t** out_hits,
        uint32_t* out_count);

    /**
     * @brief Sweep a geometry shape along a direction and return hits.
     *
     * @param handle     Instance handle.
     * @param geometry   Geometry descriptor (sphere, box, or arbitrary shape).
     * @param direction  Normalized sweep direction (3 floats).
     * @param distance   Maximum sweep distance. Must be >= 0.
     * @param both_sides If true, test both sides of mesh triangles.
     * @param mode       CLOSEST (0 or 1 hit), ANY (0 or 1), or ALL.
     * @param[out] out_hits   Receives pointer to internal hit array.
     * @param[out] out_count  Number of hits in the array.
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_sweep(
        ovphysx_handle_t handle,
        const ovphysx_scene_query_geometry_desc_t* geometry,
        const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const ovphysx_scene_query_hit_t** out_hits,
        uint32_t* out_count);

    /**
     * @brief Test geometry overlap against objects in the scene.
     *
     * Overlap queries do not have a direction or distance. Location fields
     * in the hit struct (normal, position, distance, face_index, material)
     * are zeroed -- only object identity (collision, rigid_body, proto_index)
     * is populated.
     *
     * @param handle     Instance handle.
     * @param geometry   Geometry descriptor (sphere, box, or arbitrary shape).
     * @param mode       ANY (0 or 1 result) or ALL. CLOSEST is treated as ALL.
     * @param[out] out_hits   Receives pointer to internal hit array.
     * @param[out] out_count  Number of hits (or overlaps) in the array.
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_overlap(
        ovphysx_handle_t handle,
        const ovphysx_scene_query_geometry_desc_t* geometry,
        ovphysx_scene_query_mode_t mode,
        const ovphysx_scene_query_hit_t** out_hits,
        uint32_t* out_count);

    /** @} */

    /*--------------------------------------------------*/
    /* Stream operations */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_stream */
    /** @{ */

    /*
    * Enqueue a user-provided task into the stream.
    * The task runs in stream order, allowing custom synchronization or processing.
    * 
    * Example use cases:
    * - Synchronizing with external systems
    * - Custom data processing between physics steps
    * - Inserting callbacks for debugging or profiling
    * 
    * IMPORTANT: user_data lifetime must be managed by the user and remain valid
    * until the task executes. Use CUDA events or wait_op to synchronize.
    * 
    * @param handle Physics handle
    * @param desc Task description with callback and user data
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    *
    * @pre handle and desc must be valid. desc->callback must not be NULL.
    * @post Task is executed in stream order once enqueued.
    *
    * @par Side Effects
    * Executes user callback on the internal execution context.
    *
    * @par Threading
    * Callback may execute on an internal worker thread; avoid blocking or taking locks that can deadlock.
    *
    * @par Ownership
    * Caller must keep desc->user_data valid until the task executes.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_enqueue_result_t r = ovphysx_add_user_task(handle, &task_desc);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_add_user_task(ovphysx_handle_t handle,
                                                                       const ovphysx_user_task_desc_t* desc);

    /* 
    * Wait for completion of all operations up to and including the specified operation index.
    * This operation is synchronous and will block until the operations are completed or the timeout has passed.
    * Passing 0 as the timeout makes the operation a non-blocking poll.
    * The out structure returns any errors observed since the last wait call and, on timeout, lowest_pending_op_index is set to the lowest pending operation index (or 0 if all complete).
    * The caller owns out_wait_result and must free it via ovphysx_destroy_wait_result().
    * For each failed op index, call ovphysx_get_last_op_error(op_index) to retrieve the error string.
    *
    * SINGLE-USE SEMANTICS:
    *   Each op_index is single-use. Once a wait reports completion or operation failure, the op_index is
    *   consumed and its resources are released. Attempting to wait on the same op_index again will return
    *   OVPHYSX_API_NOT_FOUND. A wait processes unconsumed indices in order through the requested index,
    *   consuming every completed or failed index it reaches. An index completed by internal stream
    *   synchronization may be acknowledged once with wait_op(); that acknowledgement consumes it.
    *
    * SPECIAL VALUES:
    *   - OVPHYSX_OP_INDEX_ALL: Wait for all pending operations (useful for shutdown/sync points).
    *
    * THREAD SAFETY:
    *   - Calls on one instance, including waits for OVPHYSX_OP_INDEX_ALL, must be externally serialized.
    *   - Waiting on a specific op_index from multiple threads concurrently is UNDEFINED BEHAVIOR.
    *   - Each op_index should be waited on by exactly one thread.
    *
    * @param handle ovphysx handle
    * @param op_index Operation index to wait for, or OVPHYSX_OP_INDEX_ALL to wait for all operations submitted up to this point
    * @param timeout_ns Timeout in nanoseconds (0 for non-blocking poll)
    * @param out_wait_result [out] Wait result information (errors and active operation indices)
    * @return ovphysx_result_t with status:
    *         OVPHYSX_API_SUCCESS if the operations were waited for successfully,
    *         OVPHYSX_API_ERROR if one or more operations failed or the wait failed internally,
    *         OVPHYSX_API_NOT_FOUND if the handle is invalid or the op_index was consumed or never existed,
    *         OVPHYSX_API_TIMEOUT if not all operations completed within the timeout.
    *
    * @pre handle and out_wait_result must be valid.
    * @post On success, all operations up to op_index have completed.
    *
    * @par Side Effects
    * Consumes each completed or failed operation reached by the wait. An operation that remains pending
    * when the wait times out is not consumed.
    *
    * @par Ownership
    * Caller owns out_wait_result and must free it via ovphysx_destroy_wait_result().
    * For each failed op index, call ovphysx_get_last_op_error(op_index) to get the error string.
    *
    * @par Errors
    * - OVPHYSX_API_NOT_FOUND if op_index is invalid or already consumed
    * - OVPHYSX_API_TIMEOUT if timeout elapses
    * - OVPHYSX_API_ERROR if one or more operations fail or the wait fails internally
    *
    * @code
    * ovphysx_op_wait_result_t wait_result = {0};
    * ovphysx_wait_op(handle, op_index, UINT64_MAX, &wait_result);
    * ovphysx_destroy_wait_result(&wait_result);
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_wait_op(ovphysx_handle_t handle,
                                                         ovphysx_op_index_t op_index,
                                                         uint64_t timeout_ns,
                                                         ovphysx_op_wait_result_t* out_wait_result);

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
     * Returns {NULL, 0} if the last call succeeded.
     *
     * @return ovphysx_string_t with the error message, or {NULL, 0} on success.
     *
     * @par Threading
     * Thread-local storage; safe to call from any thread.
     *
     * @code
     * ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
     * if (r.status != OVPHYSX_API_SUCCESS) {
     *     ovphysx_string_t err = ovphysx_get_last_error();
     *     if (err.ptr)
     *         fprintf(stderr, "Error: %.*s\n", (int)err.length, err.ptr);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_string_t ovphysx_get_last_error(void);

    /**
     * @brief Query the error string for a specific failed op_index from the last wait_op call.
     *
     * After ovphysx_wait_op() reports failed operations via error_op_indices,
     * call this function for each failed op_index to retrieve the error message.
     * The returned string is valid until the next ovphysx_wait_op() call on the same thread.
     *
     * @param op_index The failed operation index (from ovphysx_op_wait_result_t.error_op_indices).
     * @return ovphysx_string_t with the error message, or {NULL, 0} if op_index has no error.
     *
     * @par Threading
     * Thread-local storage; safe to call from any thread.
     *
     * @code
     * for (size_t i = 0; i < wait_result.num_errors; ++i) {
     *     ovphysx_string_t err = ovphysx_get_last_op_error(wait_result.error_op_indices[i]);
     *     fprintf(stderr, "Op %llu failed: %.*s\n",
     *             wait_result.error_op_indices[i], (int)err.length, err.ptr);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_string_t ovphysx_get_last_op_error(ovphysx_op_index_t op_index);

    /**
     * @brief Free the error_op_indices array in an ovphysx_op_wait_result_t.
     *
     * Call this after processing the wait result to release the dynamically
     * allocated error_op_indices array.
     *
     * @param result Pointer to the wait result to clean up (NULL-safe).
     *
     * @pre Safe to call with NULL or already-cleaned result.
     * @post result->error_op_indices is NULL and num_errors is 0.
     *
     * @code
     * ovphysx_destroy_wait_result(&wait_result);
     * @endcode
     */
    OVPHYSX_API void ovphysx_destroy_wait_result(ovphysx_op_wait_result_t* result);

    /** @} */


    /*--------------------------------------------------*/
    /* Logging configuration                            */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_logging Logging configuration
     * Configure global log level and register log callbacks.
     *
     * By default, ovphysx logs to the Carbonite console at WARNING level.
     * Use ovphysx_set_log_level() to change the threshold and
     * ovphysx_register_log_callback() to receive messages programmatically.
     */

    /** @addtogroup ovphysx_logging */
    /** @{ */

    /**
     * @brief Set the global log level threshold.
     *
     * Messages below this level are suppressed for all outputs (console and
     * registered callbacks). Callable at any time. If called before instance
     * creation, the level is stored and applied when Carbonite initializes.
     *
     * @par Threading
     * Thread-safe. Uses atomic storage internally.
     *
     * @param level Log level threshold (ovphysx_log_level_t). Default: OVPHYSX_LOG_WARNING.
     *              Must be between OVPHYSX_LOG_VERBOSE and OVPHYSX_LOG_NONE inclusive.
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success, or
     *         OVPHYSX_API_INVALID_ARGUMENT if the level was out of range
     *         (no state change is applied).
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_log_level(uint32_t level);

    /**
     * @brief Get the current global log level threshold.
     *
     * @par Threading
     * Thread-safe. Uses atomic load internally.
     *
     * @return The current log level (ovphysx_log_level_t).
     */
    OVPHYSX_API uint32_t ovphysx_get_log_level(void);

    /**
     * @brief Enable or disable Carbonite's built-in console log output.
     *
     * By default, Carbonite logs to the console (stdout/stderr). When custom
     * callbacks are registered via ovphysx_register_log_callback(), both the
     * built-in console output and the custom callbacks receive messages,
     * which may cause duplicate output if the callback also writes to the
     * console.
     *
     * Call this function with @c false to suppress the built-in console
     * output while keeping custom callbacks active. Call with @c true to
     * re-enable it.
     *
     * This function is independent of callback registration and the global
     * log level -- it only controls the built-in console logger.
     *
     * Callable at any time. If called before Carbonite initializes, the
     * preference is stored and applied during initialization.
     *
     * @par Threading
     * Thread-safe. Uses atomic storage internally.
     *
     * @param enable @c true to enable (default), @c false to disable.
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_enable_default_log_output(bool enable);

    /**
     * @brief Register a log callback.
     *
     * Multiple callbacks may be registered simultaneously. Each receives all
     * messages at or above the global log level threshold. The caller must
     * ensure @p fn and any resources referenced by @p user_data remain valid
     * until ovphysx_unregister_log_callback() is called. If the callback and
     * its resources naturally outlive the process (e.g. a static function
     * with no user_data), calling ovphysx_unregister_log_callback() is not
     * required.
     *
     * Registering the same @p fn and @p user_data pair twice returns
     * OVPHYSX_API_INVALID_ARGUMENT. Use different @p user_data values to
     * register the same function pointer multiple times.
     *
     * @note The callback may be invoked from any thread. The implementation
     *       is thread-safe, but the callback itself must also be thread-safe.
     * @note Must not be called from within a log callback. Returns
     *       OVPHYSX_API_ERROR if called during callback dispatch.
     *
     * @param fn        Callback function pointer (must not be NULL).
     * @param user_data Opaque pointer forwarded to every callback invocation (may be NULL).
     * @return ovphysx_result_t with status.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_register_log_callback(ovphysx_log_fn fn, void* user_data);

    /**
     * @brief Unregister a previously registered log callback.
     *
     * Both @p fn and @p user_data must match the values passed to
     * ovphysx_register_log_callback(). After this function returns, the
     * callback is guaranteed to not be running on any thread and will never
     * be invoked again. The caller may safely destroy the callback context.
     *
     * @note Must not be called from within a log callback. Returns
     *       OVPHYSX_API_ERROR if called during callback dispatch.
     *
     * @param fn        Callback function pointer.
     * @param user_data Opaque pointer that was passed during registration (may be NULL).
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS if found and removed.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_unregister_log_callback(ovphysx_log_fn fn, void* user_data);

    /** @} */

    /*=========================================================================
     * SDF shape evaluation
     *=========================================================================*/

    /**
     * Create an SDF shape view for shapes matching the given pattern.
     *
     * The view evaluates the signed distance field of PhysX collision shapes
     * at caller-supplied query points. Requires a GPU instance; CPU SDF
     * evaluation is not yet implemented and create will fail for CPU instances.
     *
     * The returned handle must be released with ovphysx_destroy_sdf_view when
     * no longer needed. The handle is invalidated when the attached USD stage
     * changes (ovphysx_reset_stage, ovphysx_detach_ovstage, or loading a new
     * stage) or when the instance is destroyed (via ovphysx_destroy) even if
     * ovphysx_destroy_sdf_view is not called explicitly. After invalidation,
     * evaluate/get calls return OVPHYSX_API_NOT_FOUND; recreate the view after
     * re-attaching a stage.
     *
     * Thread safety: safe to call concurrently with other ovphysx API functions
     * on the same handle, but not concurrently with ovphysx_destroy on the
     * same handle.
     *
     * @param handle            Instance handle.
     * @param pattern           USD-style path glob selecting SDF-enabled collision
     *                          shapes, including runtime-only clones (e.g. "/World/Mesh*").
     *                          Must match at least one shape or the call returns an error.
     * @param max_query_points  Number of query points per shape per evaluate
     *                          call. Query tensors passed to ovphysx_evaluate_sdf
     *                          must have Q == this value (second dimension).
     *                          Must be > 0.
     * @param out_handle        Receives the new SDF view handle on success.
     *                          Set to 0 on failure.
     * @return OVPHYSX_API_SUCCESS or an error code.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_sdf_view(
        ovphysx_handle_t handle,
        ovphysx_string_t pattern,
        uint32_t max_query_points,
        ovphysx_sdf_view_handle_t* out_handle);

    /**
     * Return the number of shapes in the SDF view (N, first dimension of query tensors).
     *
     * @param handle        Instance handle.
     * @param sdf_handle    SDF view handle from ovphysx_create_sdf_view.
     * @param out_count     Receives the shape count on success.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if out_count is NULL,
     *         OVPHYSX_API_NOT_FOUND if sdf_handle is not valid.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_sdf_view_get_count(
        ovphysx_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle,
        uint32_t* out_count);

    /**
     * Return the max_query_points value this view was created with (Q, second dimension).
     *
     * @param handle               Instance handle.
     * @param sdf_handle           SDF view handle from ovphysx_create_sdf_view.
     * @param out_max_query_points Receives the max query points on success.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if out_max_query_points is NULL,
     *         OVPHYSX_API_NOT_FOUND if sdf_handle is not valid.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_sdf_view_get_max_query_points(
        ovphysx_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle,
        uint32_t* out_max_query_points);

    /**
     * Evaluate the SDF at query points and write distances + gradients.
     *
     * DLTensor requirements:
     *   query_points:
     *     - shape: [N, Q, 3] where N = shape count, Q == max_query_points
     *     - dtype: float32
     *     - device: GPU (kDLCUDA)
     *   out_distances_and_gradients:
     *     - shape: [N, Q, 4] -- component layout: (grad.x, grad.y, grad.z, distance)
     *     - dtype: float32
     *     - device: same as query_points
     *     - must be pre-allocated with the correct shape
     *
     * @param handle                        Instance handle.
     * @param sdf_handle                    SDF view handle from ovphysx_create_sdf_view.
     * @param query_points                  Query point tensor [N, Q, 3].
     * @param out_distances_and_gradients   Output tensor [N, Q, 4].
     * @return OVPHYSX_API_SUCCESS or an error code.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_evaluate_sdf(
        ovphysx_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle,
        const DLTensor* query_points,
        DLTensor* out_distances_and_gradients);

    /**
     * Destroy an SDF view and release its resources.
     *
     * Idempotent: returns OVPHYSX_API_SUCCESS if the view was already
     * destroyed or removed by stage reset/detach cleanup. After this call
     * the handle is invalid and must not be used for evaluate/read paths.
     *
     * @param handle        Instance handle.
     * @param sdf_handle    SDF view handle from ovphysx_create_sdf_view.
     * @return OVPHYSX_API_SUCCESS, or OVPHYSX_API_ERROR on invalid instance handle.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_destroy_sdf_view(
        ovphysx_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle);

    /**
     * @defgroup debugrender Debug render buffer
     * @brief Generate physics debug geometry (points / lines / triangles) and read
     *        it back. OvPhysX has no built-in viewer: the set_* calls choose what is
     *        generated, the get_* calls return the raw geometry, and you render it
     *        yourself.
     *
     *        Return convention: when debug rendering is unavailable, most functions
     *        are no-ops returning OVPHYSX_API_SUCCESS.
     *        ovphysx_debug_render_set_scope_tokens returns OVPHYSX_API_ERROR when
     *        scope infrastructure is unavailable. When no USD stage is attached,
     *        the functions return OVPHYSX_API_ERROR; invalid arguments return
     *        OVPHYSX_API_INVALID_ARGUMENT (documented per function).
     *
     *        Granularity: the debug-render state (enable, parameters, scale,
     *        scope) applies to every physics scene of the attached stage and
     *        is process-global; there is no per-scene control. Use
     *        ovphysx_debug_render_set_scope_tokens to restrict WHAT is drawn.
     *
     *        While disabled (the default), debug rendering performs no per-step
     *        CPU or GPU visualization work, GPU-to-host visualization copies, or
     *        scratch-allocation growth. Scratch capacity acquired while enabled
     *        may remain reserved until scene teardown.
     * @{
     */

    /**
     * @brief Enable or disable debug-render generation.
     *
     * Enables debug geometry for every body, shape and joint in the scene, applies
     * the master scale and the current parameter set. Re-call after a scene rebuild
     * or reset. Read the generated geometry with ovphysx_debug_render_get_points /
     * _lines / _triangles.
     *
     * @param handle  Simulator instance (drives the attached stage).
     * @param enable  true to start generating debug geometry, false to stop.
     * @return OVPHYSX_API_SUCCESS (also a no-op SUCCESS when the viz interface is
     *         unavailable); OVPHYSX_API_ERROR when no USD stage is attached.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_enable(ovphysx_handle_t handle, bool enable);

    /**
     * @brief Set one debug-geometry parameter.
     *
     * 0 disables the geometry type and a positive value enables it. For
     * parameters whose PhysX semantics define a magnitude, that value scales the
     * geometry together with the master scale. CONTACT_POINT and FRICTION_POINT
     * use a positive value only as an enable gate; their marker size follows the
     * master scale. Geometry with an inherent shape (collision shapes, bounds) is
     * drawn at that shape's dimensions.
     * @param handle  Simulator instance (drives the attached stage).
     * @param param   One of ovphysx_debug_render_parameter_t. NONE (0) and values
     *                >= OVPHYSX_DEBUG_RENDER_PARAM_COUNT are rejected.
     * @param value   Finite value >= 0; 0 disables and a positive value enables.
     * @return OVPHYSX_API_INVALID_ARGUMENT if param is out of range or value is
     *         non-finite or negative; OVPHYSX_API_ERROR when no USD stage is
     *         attached; otherwise OVPHYSX_API_SUCCESS (a no-op SUCCESS when debug
     *         rendering is unavailable).
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_parameter(
        ovphysx_handle_t handle, uint32_t param, float value);

    /**
     * @brief Limit debug rendering to an exact set of interned prim paths.
     *
     * An object is in scope when its interned prim path is one of the handles
     * (exact membership, NO prefix expansion). Build the list through the attached
     * Stage's
     * path_dictionary_instance_t or from a query's prim list; expand a hierarchy
     * into its exact object set on the caller side (that is a helper's job, not
     * this call's). count 0 clears the scope. Joints follow their attached bodies.
     * The scope applies immediately
     * to currently instantiated runtime objects; reapply it after runtime topology
     * changes. Out-of-scope objects are skipped at emission.
     *
     * Requires an OVStage attached through ovphysx_attach_ovstage (the path
     * dictionary lives there). Handles are valid only for that exact dictionary,
     * require no per-handle release, and the scope is cleared automatically by
     * ovphysx_detach_ovstage. Re-intern and reapply the scope after attaching a
     * different Stage. Returns OVPHYSX_API_ERROR when the sidecar symbol,
     * visualization slot, or attached Stage dictionary is unavailable. Scope
     * state remains unchanged on every error.
     *
     * @param handle  Simulator instance (drives the attached stage).
     * @param tokens  Array of count interned ovx_primpath_t handles from the
     *                attached Stage's dictionary. May be NULL when count is 0.
     * @param count   Number of entries; 0 clears the scope.
     * @return OVPHYSX_API_INVALID_ARGUMENT when tokens is NULL with count > 0 or
     *         any token is invalid; OVPHYSX_API_ERROR when the sidecar symbol,
     *         visualization slot, or attached Stage dictionary is unavailable.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_scope_tokens(
        ovphysx_handle_t handle, const ovx_primpath_t* tokens, uint32_t count);

    /**
     * @brief Read back a debug-render parameter's value.
     *
     * Returns the value last set through ovphysx_debug_render_set_parameter,
     * cached on the OvPhysX side (the debug-render state is process-global;
     * 0 = off, the default before any set).
     * @param handle          Instance handle.
     * @param param           One of ovphysx_debug_render_parameter_t (NONE / out-of-range rejected).
     * @param[out] out_value  Set to the cached value. Must be non-NULL.
     * @return OVPHYSX_API_INVALID_ARGUMENT if param is NONE / out of range or out_value is NULL.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_parameter(
        ovphysx_handle_t handle, uint32_t param, float* out_value);

    /**
     * @brief Set the master PhysX debug-render scale.
     * @param handle  Simulator instance (drives the attached stage).
     * @param scale   Must be finite and >= 0.
     * @return OVPHYSX_API_INVALID_ARGUMENT if scale is non-finite or negative;
     *         OVPHYSX_API_ERROR when no USD stage is attached; otherwise OVPHYSX_API_SUCCESS.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_scale(ovphysx_handle_t handle, float scale);

    /**
     * @brief Read back the master debug-render scale last set through OvPhysX (cached;
     *        defaults to 1.0 before any set).
     * @param handle          Instance handle.
     * @param[out] out_scale  Set to the cached scale. Must be non-NULL.
     * @return OVPHYSX_API_INVALID_ARGUMENT if out_scale is NULL.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_scale(ovphysx_handle_t handle, float* out_scale);

    /**
     * @brief Restrict debug-render generation to a world-space AABB.
     * @param handle  Simulator instance (drives the attached stage).
     * @param min3    Box minimum, float[3]. Must be non-NULL and finite.
     * @param max3    Box maximum, float[3]. Must be non-NULL, finite, and >= min3 per axis.
     * @return OVPHYSX_API_INVALID_ARGUMENT if min3/max3 is NULL, non-finite, or min > max;
     *         OVPHYSX_API_ERROR when no USD stage is attached; otherwise OVPHYSX_API_SUCCESS.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_culling_box(
        ovphysx_handle_t handle, const float min3[3], const float max3[3]);

    /**
     * @brief Read the current debug-render point buffer (the debug geometry produced
     *        during the step). OvPhysX has no viewer; draw it yourself.
     *
     * **Ownership / lifetime:** the returned pointer aliases an OvPhysX-owned buffer and
     * is invalidated by the next ovphysx_step() OR by any stage / scene change
     * (ovphysx_reset_stage, ovphysx_update_from_ovstage, ovphysx_detach_ovstage,
     * ovphysx_destroy_instance): the underlying PhysX scenes and debug buffers may
     * be recreated. Copy out before any of those; do not hold the pointer across a
     * step or re-attach. On
     * success-with-no-data *out_points is set to NULL and *out_count to 0.
     *
     * @param handle           Instance handle.
     * @param[out] out_points  Set to the buffer base (NULL when empty). Must be non-NULL.
     * @param[out] out_count   Set to the primitive count (0 when empty). Must be non-NULL.
     * @return OVPHYSX_API_INVALID_ARGUMENT if out_points or out_count is NULL;
     *         OVPHYSX_API_ERROR when no USD stage is attached; otherwise OVPHYSX_API_SUCCESS.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_points(
        ovphysx_handle_t handle, const ovphysx_debug_point_t** out_points, uint32_t* out_count);
    /** @brief Read the debug-render line buffer. Same ownership / lifetime + argument contract as ovphysx_debug_render_get_points. */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_lines(
        ovphysx_handle_t handle, const ovphysx_debug_line_t** out_lines, uint32_t* out_count);
    /** @brief Read the debug-render triangle buffer. Same ownership / lifetime + argument contract as ovphysx_debug_render_get_points. */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_triangles(
        ovphysx_handle_t handle, const ovphysx_debug_triangle_t** out_triangles, uint32_t* out_count);

    /** @} */


#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_OVPHYSX_H
