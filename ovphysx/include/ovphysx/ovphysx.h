// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 * @implements REQ-CAPI-STRING-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1 AC-4 AC-9 AC-10 AC-11
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-BINDING-DEVICE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-11
 * @implements REQ-PYTHON-CLONE-001
 * @covers AC-2
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

    /* General notes
    *
    * Version information: See ovphysx_get_version() and ovphysx_get_version_string() below.
    *
    * DLPack Integration:
    *   This API uses DLPack for tensor data exchange between the user and the system.
    *   Same-device paths are zero-copy where supported. Cross-device support depends on
    *   the tensor type and process policy. See the Tensor Binding API below and dlpack.h
    *   for the tensor format.
    *
    * Return values:
    *   Synchronous functions (blocking calls) return ovphysx_result_t (status).
    *   Asynchronous functions return ovphysx_enqueue_result_t (status, op_index).
    *   On failure, call ovphysx_get_last_error() on the same thread to retrieve the
    *   error message. The returned string is valid until the next ovphysx API call on
    *   that thread.
    *
    * Stream ordered asynchronous execution:
    *   - Async operations execute in submission order on a single queue.
    *   - Results appear as if operations completed serially (sequential consistency).
    *   - Use ovphysx_wait_op() to ensure operations have completed before using results.
    *   - If an asynchronous operation fails to enqueue (error returned from the function call),
    *     the returned op index is invalid and cannot be used for synchronization.
    *   - Each op_index is SINGLE-USE: once ovphysx_wait_op reports completion or
    *     operation failure, the op_index is consumed and cannot be reused. A timeout
    *     does not consume an operation that remains pending. Concurrent waits on the
    *     same op_index from multiple threads have undefined behavior.
    *
    * Failure handling:
    *   - If operation N fails, the error is recorded and returned via ovphysx_wait_op().
    *   - The error string indicates whether subsequent operations fail or succeed after
    *     the failure of operation N, and whether the instance is left in a defined state.
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
     *   - Instances share the underlying omni.physx runtime and attached stage.
     *     Serialize simulation, stage mutation, and binding creation across
     *     instances.
     *
     * Single instance, multiple threads:
     *   - Operations are NOT thread-safe for the same instance.
     *   - Use external synchronization if calling from multiple threads.
     *
     * Data buffer lifetimes:
     *   - The caller keeps data passed to async operations valid and unmodified
     *     until the operation completes (as indicated by ovphysx_wait_op).
     *
     * CUDA context requirements:
     *   - The SDK manages CUDA contexts internally where needed.
     *   - Wait/sync functions may be called from any thread without manual context management.
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
     * @defgroup ovphysx_tensor_binding Tensor bindings (deprecated)
     * Bulk tensor access APIs for simulation data.
     *
     * @deprecated Superseded by the session read/write API (@ref ovphysx_read /
     *   @ref ovphysx_write, Python `PhysX.read` / `PhysX.write`). Retained for the
     *   deprecation window. Write new code against the session API.
     * @ingroup ovphysx_deprecated
     */

    /**
     * @defgroup ovphysx_stream Stream operations
     * Stream-ordered task submission and synchronization.
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
     * @brief Create a new ovphysx instance.
     *
     * Initialize create_args with OVPHYSX_CREATE_ARGS_DEFAULT for sensible defaults:
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * int main(void)
     * {
     *     ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
     *     ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
     *     if (ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status != OVPHYSX_API_SUCCESS ||
     *         ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
     *         return 1;
     *     ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
     *     if (result.status == OVPHYSX_API_SUCCESS)
     *         result = ovphysx_destroy_instance(handle);
     *     if (ovphysx_shutdown().status != OVPHYSX_API_SUCCESS)
     *         return 1;
     *     return result.status == OVPHYSX_API_SUCCESS ? 0 : 1;
     * }
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
     * - OVPHYSX_API_ERROR if ovphysx_initialize() is not active, if an explicit OmniPVD creation setting is
     *   supplied while another instance exists, or for other initialization failures
     *
     * @note A non-empty active_cuda_gpus request is retained by this handle and applied to
     *   the shared process physics backend when this handle attaches a stage.
     *   A different deterministic ordinal after the first GPU scene requires a new process.
     *   Per-scene device selection (CPU vs GPU dynamics) is owned by PhysX via physxScene:enableGPUDynamics
     *   in the USD stage. Use ovphysx_set_cpu_mode() before creating any instance to force process-wide
     *   CPU-only mode.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_instance(const ovphysx_create_args* create_args, ovphysx_handle_t* out_handle);

    /**
     * @brief Force process-wide CPU-only mode.
     *
     * To keep ovphysx itself from touching the CUDA driver for the lifetime of the process, set
     * this to true before the first instance is ever created. All subsequent PhysX scenes use
     * CPU dynamics regardless of their USD physxScene:enableGPUDynamics setting. Other libraries
     * in the process may still open the driver. Loading an ovstage-backed Stage currently does.
     *
     * The Python frontend has one further dependency: read() and write() expose warp.array
     * tensors, and building the first array initializes the Warp runtime, which opens the CUDA
     * driver when the installed Warp was built with CUDA. Warp has no runtime switch for this.
     * A CPU-only Warp keeps the read and write paths driverless:
     * `conda install -c conda-forge "warp-lang=*=*cpu*"`, or a Warp built from source with no
     * CUDA toolkit configured (WP_ENABLE_CUDA=0). The wheel's own `warp-lang` dependency
     * resolves to the CUDA-enabled PyPI build, so this is an opt-in for deployments that need
     * the guarantee. The Python frontend detects a CUDA-enabled Warp under CPU-only mode and
     * warns once, naming the remedy.
     * For per-scene CPU control without this flag, author each scene explicitly
     * (physxScene:enableGPUDynamics=false + physxScene:broadphaseType="MBP").
     *
     * Requires that no instances are active. Returns OVPHYSX_API_ERROR if any instances
     * currently exist, or if attempting to set false after true has been applied. The
     * CPU-only request is sticky as soon as a call setting it to true succeeds. A call
     * after an earlier instance was destroyed may succeed, but cannot provide the
     * ovphysx no-CUDA-touch guarantee or retarget an already-bootstrapped runtime.
     * For CPU-only deployments, setting OVPHYSX_DISABLE_GPU before ovphysx
     * initialization provides the equivalent process-wide policy.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_cpu_mode(bool cpu_only);

    /**
     * @brief Query whether process-wide hard CPU-only mode is in effect.
     *
     * Reports the effective hard CPU-only policy: true when
     * @ref ovphysx_set_cpu_mode has succeeded with true, or when
     * `OVPHYSX_DISABLE_GPU` is active. The environment variable is read live
     * before @ref ovphysx_initialize (and again after @ref ovphysx_shutdown
     * until the next initialize); @ref ovphysx_initialize latches the value
     * for that initialized interval. This is not a query of per-scene USD
     * `physxScene:enableGPUDynamics`, and it does not report a CUDA ordinal
     * (`active_cuda_gpus`) or attach-time resolved dynamics/device outcome.
     *
     * Callable at any time. No instance and no prior @ref ovphysx_initialize()
     * are required.
     *
     * @param out_cpu_only [out] Receives true when hard CPU-only mode is
     *   active. Must not be NULL.
     * @return OVPHYSX_API_SUCCESS on success.
     * @return OVPHYSX_API_INVALID_ARGUMENT if out_cpu_only is NULL.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_cpu_mode(bool* out_cpu_only);

    /**
     * @brief Locate the codeless PhysX USD schemas shipped with ovphysx.
     *
     * ovphysx ships its PhysX schema definitions (`PhysxSchema` and
     * `OmniUsdPhysicsDeformableSchema`) as codeless USD plugins: a root `plugInfo.json`
     * plus one `<Module>/resources/` directory per schema module. ovphysx does not load,
     * link, or configure OpenUSD and never registers these schemas itself. The application
     * owns its USD runtime(s) and registers the schemas explicitly:
     *
     * - With ovstage, pass the returned path to `ovstage_population_register_usd_schemas()`
     *   before the first population call in the process.
     * - With a stock OpenUSD runtime, add the path to `PXR_PLUGINPATH_NAME` before the process
     *   starts, or pass it to `PlugRegistry::RegisterPlugins()` before the first schema-registry
     *   access.
     *
     * The path is derived from the location of the ovphysx shared library, or from `OVPHYSX_LIB`
     * when it is set: `<library directory>/schemas/physx` (a runtime copied beside an
     * application) is checked first, then `<library directory>/../schemas/physx` (the SDK and
     * wheel layouts, `<sdk>/schemas/physx`).
     *
     * Notes:
     * - Safe to call at any time, including before @ref ovphysx_create_instance().
     * - Does not initialize ovphysx, load USD, acquire Carbonite, or modify the environment.
     * - The returned string is NUL-terminated and owned by ovphysx. It stays valid until the
     *   calling thread calls this function again.
     *
     * @param out_root Receives the schema root directory. Set to an empty string on failure.
     * @return
     * - OVPHYSX_API_SUCCESS if the schema root was found.
     * - OVPHYSX_API_INVALID_ARGUMENT if @p out_root is NULL.
     * - OVPHYSX_API_ERROR if no `schemas/physx/plugInfo.json` exists next to the library. Use
     *   @ref ovphysx_get_last_error() for details.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_codeless_schema_root(ovphysx_string_t* out_root);

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
     * - OVPHYSX_API_ERROR if the handle is not registered or destruction fails.
     *   An unregistered handle is rejected before teardown and does not affect
     *   registered instances or process-global asynchronous state. No error
     *   string is returned. Consult logs for other destruction failures.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t destroy_owned_instance(ovphysx_handle_t handle)
     * {
     *     return ovphysx_destroy_instance(handle);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_destroy_instance(ovphysx_handle_t handle);

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
     * With no live handles, shutdown drains the direct PhysX runtime while the
     * Carbonite framework remains resident for its process-exit hook. If live
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
     * OmniPVD output, recording capability, directory, transport, address,
     * port, and timeout settings are create-only and return OVPHYSX_API_ERROR
     * while an instance exists.
     *
     * Use the builder functions in ovphysx_config.h for convenient construction:
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     * #include <ovphysx/ovphysx_config.h>
     *
     * static ovphysx_result_t configure_runtime(void)
     * {
     *     ovphysx_result_t result =
     *         ovphysx_set_global_config(ovphysx_config_entry_num_threads(4));
     *     if (result.status != OVPHYSX_API_SUCCESS)
     *         return result;
     *     result = ovphysx_set_global_config(
     *         ovphysx_config_entry_disable_contact_processing(true));
     *     if (result.status != OVPHYSX_API_SUCCESS)
     *         return result;
     *     return ovphysx_set_global_config(ovphysx_config_entry_carbonite(
     *         OVPHYSX_LITERAL("/physics/updateToUsd"),
     *         OVPHYSX_LITERAL("false")));
     * }
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
     * @param value_out [in/out] String with a pre-allocated writable, non-NULL
     *                  buffer. length is the buffer capacity and must be in
     *                  the range 1 through UINT32_MAX.
     * @param out_required_size [out] Required buffer size including null terminator.
     * @return OVPHYSX_API_SUCCESS on success, OVPHYSX_API_BUFFER_TOO_SMALL when
     *         the buffer holds only a truncated value,
     *         OVPHYSX_API_NOT_FOUND when the config value is absent,
     *         OVPHYSX_API_INVALID_ARGUMENT for an invalid key, pointer, or
     *         capacity, or OVPHYSX_API_ERROR when settings are unavailable.
     * @post On success, value_out->length is the content length and
     *       value_out->ptr[value_out->length] is '\0'.
     * @post On OVPHYSX_API_BUFFER_TOO_SMALL, value_out->length remains the input
     *       buffer capacity, value_out->ptr[value_out->length - 1] is '\0', and
     *       out_required_size reports the required capacity including the NUL.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_string(
        ovphysx_config_string_t key, ovphysx_string_t* value_out, size_t* out_required_size);

    /** @} */

    /*--------------------------------------------------*/
    /* OmniPVD recording                                */
    /*--------------------------------------------------*/

    /** @defgroup ovphysx_recording OmniPVD recording */
    /** @addtogroup ovphysx_recording */
    /** @{ */

    /**
     * @brief Start an OmniPVD recording session.
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
     * Late start requires process-wide recording capability to be selected
     * before the first instance is created, either with
     * OVPHYSX_CONFIG_OMNIPVD_RECORDING_CAPABLE or by enabling startup output.
     * A default, incapable instance returns OVPHYSX_API_INVALID_STATE and an
     * error naming omnipvd_recording_capable. Unsupported platforms retain
     * OVPHYSX_API_NOT_IMPLEMENTED.
     *
     * After reattach, a capability-only runtime is dormant and can start late
     * recording immediately. A runtime with startup output configured instead
     * starts a new startup session owned by the reattaching handle. That handle
     * must stop the session before any late destination can start.
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
    OVPHYSX_API ovphysx_result_t ovphysx_start_recording(
        ovphysx_handle_t handle, const ovphysx_omnipvd_destination_t* destination);

    /**
     * @brief Stop and finalize the active recording.
     *
     * The handle that started a late session owns it. The handle whose creation
     * started startup output owns that startup session. A peer handle is
     * inactive and cannot stop the owner's session. A successful owner stop
     * permits a later recording session to start.
     *
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_STATE when this handle
     *         owns no active recording (including while a peer owns the
     *         globally active session), or OVPHYSX_API_ERROR when
     *         stop/finalization fails.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_stop_recording(ovphysx_handle_t handle);

    /**
     * @brief Query whether this instance's startup or late recording is active.
     *
     * A peer reports false while another instance owns the shared runtime's
     * active recording.
     *
     * @param handle Valid instance handle.
     * @param out_is_recording [out] Receives true only while sampling is active.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_is_recording(
        ovphysx_handle_t handle, bool* out_is_recording);

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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_enqueue_result_t reset_runtime_stage(ovphysx_handle_t handle)
     * {
     *     return ovphysx_reset_stage(handle);
     * }
     * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_reset_stage(ovphysx_handle_t handle);

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
    * ovphysx_step(). Replacing the attached Stage requires
    * ovphysx_detach_ovstage() first.
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
    * @pre stage outlives the attachment. ovphysx captures this pointer and
    *      dereferences it on every ovphysx_update_from_ovstage() until
    *      ovphysx_detach_ovstage(); destroying the Stage between attach and detach
    *      is undefined behavior, not a recoverable error.
    * @post On success, subsequent ovphysx_update_from_ovstage() calls observe
    *       committed Stage writes through the runtime ovstage backend.
    *
    * @note Not thread-safe per instance. Like the rest of the per-instance API,
    *       the caller must serialize ovphysx_attach_ovstage() against any other
    *       call on the same handle. The already-attached check and the attach are
    *       not internally locked against concurrent foreground callers.
    * @note The underlying PhysX simulation attach is a **process-wide** resource:
    *       only one instance may hold a live ovstage attach at a time across the
    *       whole process. A second instance's attempt to attach while another
    *       instance's attach is live is rejected with OVPHYSX_API_ERROR rather
    *       than silently displacing it. Call ovphysx_detach_ovstage() on the
    *       owning instance first.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT if stage is null or read_ordinal is 0
    * - OVPHYSX_API_ERROR if already attached or the runtime attach fails,
    *   including unreadable articulation/joint schema data during initial scan
    * - OVPHYSX_API_ERROR if another instance already owns the live process-wide
    *   PhysX attach
    * - OVPHYSX_API_ERROR if the PhysX USD schemas were not registered with
    *   ovstage before the first population in the process (see below), or the
    *   codeless schema tree next to the library is missing
    *
    * @note The application owns schema registration: pass the directory returned
    *       by @ref ovphysx_get_codeless_schema_root to
    *       `ovstage_population_register_usd_schemas()` before the first population
    *       in the process. Population drops every Physx* API it cannot resolve,
    *       so an unregistered stage carries none of the asset's PhysX settings
    *       (self-collision, joint velocity limits, solver iterations, deformable
    *       and particle fallbacks). This call verifies the registration by
    *       re-registering the same root, which ovstage treats as a no-op when it
    *       was done in time and rejects when population already ran without it;
    *       the attach then fails with an error naming the missing call, and so
    *       does every later attach in the process, since the schema registry USD
    *       built without them cannot be rebuilt. ovstage keys the registration on
    *       the plugin family, so schemas discovered through
    *       `OV_PXR_PLUGINPATH_2511` or registered from another copy of the tree
    *       pass the check. A registration another USD consumer made unobservable
    *       to ovstage cannot be detected; the create-time config entry
    *       `/ovphysx/schemas/requireRegistration = false` downgrades the refusal
    *       to a warning for such a host.
    *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t attach_caller_owned_stage(
     *     ovphysx_handle_t handle,
     *     ovstage_instance_t* stage,
     *     ovstage_ordinal_t read_ordinal)
     * {
     *     return ovphysx_attach_ovstage(handle, stage, read_ordinal);
     * }
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
    * drained. With `has_start_ordinal == false` only `end_ordinal` is drained (the
    * single-ordinal form). The selected changes are drained through the active
    * ovstage change feed and applied to the running simulation.
    *
    * @param handle ovphysx instance handle.
    * @param range ovstage ordinal range to drain (see ovstage_ordinal_range_t).
    *
    * @pre `handle` is valid and @ref ovphysx_attach_ovstage succeeded.
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
    * stage's tensor and contact views. Do not read or write existing bindings.
    * Destroy them and create replacements after attaching and realizing a stage again.
    * If the attached stage has an active OmniPVD recording, detach stops and finalizes
    * it, including when a peer instance started the recording. After reattach,
    * capability-only recording is dormant and can start
    * immediately. Configured startup output instead starts a new startup session
    * owned by the reattaching handle. Stop it before starting a late destination.
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

    /**
    * @brief Get the handle identifying this instance's current attach.
    *
    * An attach handle is an attach *identity*, not a USD stage id. It is nonzero
    * for every live attach, including an ovstage attach whose source has no
    * backing USD stage, and a fresh handle is minted per attach. A consumer
    * that stores the handle when it binds can therefore distinguish "still the
    * attach it bound to" from "detached" and from "a different attach that reuses
    * the same stage id", none of which a stage id can express. See ADR-0016.
    *
    * This is the only route by which a handle crosses the C boundary. Contact
    * events report the attach they came from as this same value, so a consumer
    * can match a reported event against the attach it holds.
    *
    * @param instance_handle ovphysx instance handle.
    * @param[out] out_attach_handle Receives the current attach handle, or 0 (no
    *      attach) when nothing is attached. Also set to 0 on failure.
    *
    * @note An instance attaches at most once at a time, so a caller that does not
    *       need to detect detach/reattach need not track the value at all.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT if out_attach_handle is NULL or instance_handle is invalid
    */
    OVPHYSX_API ovphysx_result_t ovphysx_get_attach_handle(ovphysx_handle_t instance_handle,
                                                           uint64_t* out_attach_handle);

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
     * attached (@ref ovphysx_attach_ovstage). Under any other attach it returns 0
     * objects.
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
     * snapshot. Matched prims and column values are resolved lazily (the prim
     * count at @ref ovphysx_fetch_query_result, the column data at @ref ovphysx_read)
     * and each observes the most recently completed step (this call and the read
     * both drain pending sim first). A step that lands between ovphysx_query and
     * ovphysx_read does not make the handle stale: the read simply reflects the
     * newer step. For OVPHYSX_SCOPE_ACTIVE that means "the active set of whatever
     * step had last completed when ovphysx_read ran". Read before stepping again
     * to capture a specific step's active set. The current output producer returns
     * `layout_generation == 0`. Rebuild cached queries and layouts after a known
     * structural change rather than using it as an invalidation signal.
     *
     * @param handle ovphysx instance handle.
     * @param object_type Simulated type to select (@ref ovphysx_sim_object_type_t).
     * @param scope @ref ovphysx_object_scope_t (all or active, where active is single-frame).
     * @param out_query [out] Receives the query handle. Nonzero on success, INCLUDING
     *      an empty match (a valid query whose read reaches end-of-iteration
     *      immediately and whose fetch_query_result reports total_prim_count == 0).
     *      0 means FAILURE only.
     * @return ovphysx_result_t.
     *
     * @pre handle is valid and an ovstage Stage is attached.
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
     * `ovx_string_or_token_t` tokens). The array is owned by the query and valid
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
     * ovstage source's own `path_dictionary_instance_t*`, the dictionary that
     * interned this query's `prim_list` handles and `attribute` tokens, AND the
     * dictionary the ovstage write path interns into. Because read and write share
     * it, a group's `attribute` token / `prim_list` handle can be fed straight back
     * into the ovstage write path with no rebuild and no string round-trip. This
     * accessor is only needed to resolve a token to a human-readable string or to
     * intern a *derived* name (e.g. renaming output to "sim:<name>").
     *
     * The pointer is declared in `<ovx/path_dictionary/path_dictionary.h>` and
     * surfaced here as an opaque `void*` (that header defines C++-only inline
     * helpers, so it is deliberately not pulled into this C surface). Owned by the
     * runtime. Do not free. Returns NULL for a non-ovstage backend.
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
     * an `ovx_string_or_token_t`: a string name (see OVPHYSX_ATTR_*) OR an interned
     * token (e.g. from @ref ovphysx_fetch_query_result, fed straight back with no
     * token-to-string round-trip). Iterate the result with @ref ovphysx_fetch_read_next,
     * release each consumed group with @ref ovphysx_release_group, and release the
     * session with @ref ovphysx_release_read. Names not produced by the queried type
     * are skipped.
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
     * stream). A consumer reading on the default/null stream is already correct because
     * the work is ordered on that stream. A consumer reading on **its own** stream must
     * make that stream wait on `wait_event` first, or it may observe a partially
     * written column: call @ref ovphysx_cuda_stream_wait_event, which issues that wait
     * through the CUDA driver shim ovphysx already loads. The event belongs to the read
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
     * is not an error and is currently indistinguishable from a query that legitimately
     * matched nothing. Step once before reading. A query still reports the matching
     * prims either way. Readiness is per scene: a multi-scene read can return groups for
     * ready scene partitions while omitting an unready DirectGPU scene's partition.
     * CPU articulation-root and articulation-joint records can likewise be discoverable
     * while buffered attach work is still inserting the SDK articulation into its scene.
     * Those pending partitions are omitted normally, then become readable after insertion.
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
     * all groups have been consumed, with `*out_group` set to NULL. Any other status is
     * a real error (`*out_group` NULL).
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
     * column into session-owned storage at read time).
     *
     * A device (`kDLCUDA`) column additionally has a readiness contract: it is handed
     * over before its producing work has necessarily completed. Wait on
     * `group.data.cuda_sync.wait_event` before reading it from a non-default stream
     * (@ref ovphysx_cuda_stream_wait_event issues that wait). See @ref ovphysx_read for
     * the full rule. Validity and readiness are separate: the borrow above means the
     * memory may still be read, the event means the values in it are final.
     *
     * END_OF_ITERATION means every group that was produced has been consumed. It does
     * NOT by itself mean the read was complete: a backend build or gather that failed
     * omits its columns and ends the drain with an error status instead, so a caller
     * that needs to distinguish a short answer from the whole one must check for that
     * rather than treat any non-SUCCESS as "done". Groups already fetched stay valid
     * and must still be released either way.
     *
     * @param handle ovphysx instance handle.
     * @param read Read-session handle from @ref ovphysx_read.
     * @param out_group [out] Receives a borrowed `ovstage_read_group_t*` for the next
     *      group (NULL at end of iteration or on error).
     * @return ovphysx_result_t: SUCCESS (group filled), END_OF_ITERATION (done), else error.
     *
     * @pre handle and read are valid and out_group is non-null.
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
     * @brief Open a write session pushing ONE named attribute into the simulation.
     *
     * The application-to-physics direction, the mirror of @ref ovphysx_read. It reuses
     * that query verbatim and adds no selection of its own: the session reaches exactly
     * the set `query` matched, and there is no index, mask or prim-list parameter
     * anywhere on this surface.
     *
     * ## One attribute per session
     *
     * Unlike @ref ovphysx_read, which takes an attribute array, a write session carries
     * exactly ONE attribute. `ovstage_map_group_t` has no `attribute` field, so a group
     * from a multi-attribute session could not say which attribute it represents and
     * same-shaped attributes (linear vs angular velocity) would be indistinguishable.
     * Writing several attributes over one prim set means several sessions.
     *
     * The write's attribute vocabulary is its own: a name accepted here need not be one
     * @ref ovphysx_read emits, since write-only control inputs such as forces and
     * wrenches have no read counterpart. Where a name IS shared with the read, the
     * requested-name vs emitted-token distinction documented there applies unchanged.
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
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @param attribute The single attribute to write (string name or interned token).
     * @param out_write [out] Receives the write-session handle (0 on failure).
     * @return ovphysx_result_t.
     *
     * @pre handle and query are valid.
     * @post On success `*out_write` is non-zero.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if args are null
     * - OVPHYSX_API_ERROR for an unknown query, an attribute the queried type does not
     *   accept or that is not writable, or a scene whose pipeline the scatter does not serve
     */
    OVPHYSX_API ovphysx_result_t ovphysx_write(ovphysx_handle_t handle,
                                               ovphysx_query_handle_t query,
                                               const ovx_string_or_token_t* attribute,
                                               ovphysx_write_handle_t* out_write);

    /**
     * @brief Fetch the next writable group from a write session.
     *
     * On success points `*out_group` at the next `ovstage_map_group_t` and returns
     * OVPHYSX_API_SUCCESS. Returns OVPHYSX_API_END_OF_ITERATION (NOT an error) once all
     * groups have been consumed, with `*out_group` set to NULL. Any other status is a real
     * error (`*out_group` NULL).
     *
     * The group is **producer-owned** (the caller does not allocate it) and handed back
     * as a `const` borrow. `const` is correct even on the write path: the group is a
     * DESCRIPTOR the caller reads, describing BUFFERS the caller fills through
     * `data.tensors[i].data`, which `const` permits since it does not propagate through
     * pointer members. No field of the struct is the caller's to assign, and tensor
     * shape, dtype and device are dictated by the implementation.
     *
     * Group storage stays valid until that group is committed, independent of further
     * fetches. An intervening @ref ovphysx_step does not invalidate a live group.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle from @ref ovphysx_write.
     * @param out_group [out] Receives a borrowed `const ovstage_map_group_t*` (NULL at
     *      end of iteration or on error).
     * @return ovphysx_result_t: SUCCESS (group filled), END_OF_ITERATION (done), else error.
     *
     * @pre handle and write are valid and out_group is non-null.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_group is null
     * - OVPHYSX_API_ERROR for a write handle that is not live
     */
    OVPHYSX_API ovphysx_result_t ovphysx_fetch_write_next(ovphysx_handle_t handle,
                                                          ovphysx_write_handle_t write,
                                                          const ovstage_map_group_t** out_group);

    /**
     * @brief Commit a filled group, transferring ownership of its data to physics.
     *
     * The caller must have filled EVERY mapped entry. There is no fill-mask, so a
     * partially filled group publishes whatever its unfilled entries contain. To write
     * fewer prims, query fewer. After this call the mapped pointers belong to physics.
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
     * `write_done_sync` is ovstage's `{stream, wait_event}` handoff. A non-zero
     * `wait_event` is waited on before the data is consumed. `{stream, 0}` (queued work
     * with no event) still drains that stream. Only `{0, 0}` asserts nothing is
     * outstanding. A host-resident group does NOT imply `{0, 0}`, since the caller may
     * have staged on its own stream. The runtime orders the write against an in-flight step.
     * It does NOT warm up a scene that has not stepped: a write issued before the first step is
     * **refused**, not silently advanced on the caller's behalf. This is symmetric with the read,
     * which omits pre-step rows rather than failing. Call @ref ovphysx_warmup or @ref ovphysx_step
     * to control when that first step happens. The write never steps on the caller's behalf.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle the group came from.
     * @param group The borrowed group from @ref ovphysx_fetch_write_next.
     * @param write_done_sync CUDA handoff. `{0, 0}` when nothing is outstanding.
     * @return ovphysx_result_t.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if group is null
     * - OVPHYSX_API_ERROR for a write handle or group pointer that is not live. This is
     *   deliberately NOT idempotent: commit is the mutation, so reporting success for a
     *   stale pointer would claim data was published when none was.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_commit_group(ovphysx_handle_t handle,
                                                      ovphysx_write_handle_t write,
                                                      const ovstage_map_group_t* group,
                                                      ovstage_cuda_sync_t write_done_sync);

    /**
     * @brief Release a write session, discarding anything uncommitted.
     *
     * Every group that was never committed is **discarded, not published**, which is why
     * this takes no sync token. There is nothing left to order against. A caller that
     * fails or throws mid-fill therefore publishes nothing from the group it was
     * filling, and forgetting to commit is a silent no-op rather than uninitialized data
     * reaching the solver. Committed groups are NOT rolled back.
     *
     * @param handle ovphysx instance handle.
     * @param write Write-session handle from @ref ovphysx_write.
     * @return ovphysx_result_t. Idempotent for an already-released / unknown handle,
     *      matching @ref ovphysx_release_read, so teardown is always safe.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_release_write(ovphysx_handle_t handle,
                                                       ovphysx_write_handle_t write);

    /**
     * @brief Ask whether an attribute can be written on an object type, and how.
     *
     * The programmatic answer to a question that is otherwise only documented in prose,
     * keyed on the SAME (object type, attribute) the write path takes (@ref ovphysx_write
     * resolves the object type through its query and names the attribute). This reports the
     * classification the write API itself uses, derived from the same write and read
     * attribute tables, so a caller and the implementation cannot disagree.
     *
     * Scene-independent: writability is a property of the write API, not of any live scene,
     * so this needs no instance, query or step. A name the object type does not accept comes
     * back OVPHYSX_WRITABILITY_UNCLASSIFIED, and @ref ovphysx_write rejects it. Such a name
     * is a gap to close, never an invitation to try the write anyway.
     *
     * @param object_type Object type the attribute would be written on (@ref ovphysx_query's
     *   selector), e.g. OVPHYSX_OBJECT_RIGID_BODY.
     * @param attribute Attribute name (see the OVPHYSX_ATTR_* macros), as the string field of an
     *   `ovx_string_or_token_t`. STRING-ONLY: an interned-token-only key is rejected, because
     *   resolving a token needs a path dictionary this scene-free query does not have.
     * @param out_writability [out] Receives the classification.
     * @return ovphysx_result_t.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_writability or attribute is null, if attribute
     *   carries no string name (a token-only key), or if object_type is outside
     *   @ref ovphysx_sim_object_type_t (so an unknown type is distinguishable from a valid type
     *   that does not accept the name, which returns OVPHYSX_WRITABILITY_UNCLASSIFIED)
     */
    OVPHYSX_API ovphysx_result_t ovphysx_writability(ovphysx_sim_object_type_t object_type,
                                                     const ovx_string_or_token_t* attribute,
                                                     ovphysx_writability_t* out_writability);

    /**
     * @brief Release an output query.
     * @param handle ovphysx instance handle.
     * @param query Query handle from @ref ovphysx_query.
     * @return ovphysx_result_t. Idempotent for an already-released / unknown handle.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_release_query(ovphysx_handle_t handle,
                                                       ovphysx_query_handle_t query);

    /**
     * @brief Order @p stream after @p event, so work queued on it observes a finished column.
     *
     * The readiness contract on a device (`kDLCUDA`) read column says a consumer using its own
     * stream must wait on `group.data.cuda_sync.wait_event` before reading (see @ref ovphysx_read).
     * This is that single `cuStreamWaitEvent`, routed through the CUDA driver shim ovphysx already
     * loads, so honouring the contract costs the consumer no direct CUDA dependency.
     *
     * Asynchronous: it enqueues the dependency and returns. It does not synchronize the host, and
     * it does not block @p stream against anything except work preceding @p event.
     *
     * @param stream CUDA stream to order, as `uintptr_t`. Follows the CUDA driver's stream
     *               handles, which coincide with the DLPack sentinels: 0 is the NULL stream,
     *               1 is the legacy default stream, 2 is the per-thread default stream, and any
     *               other value is a `CUstream`.
     * @param event  CUDA event to wait on, as `uintptr_t`, normally a group's
     *               `cuda_sync.wait_event`. 0 is a no-op success: a column with no producer work
     *               to await is already readable, and callers do not have to special-case it.
     *
     * @return OVPHYSX_API_SUCCESS once the wait is enqueued (or was not needed).
     *         OVPHYSX_API_ERROR if CUDA is unavailable in this process or the driver rejected the
     *         call. Use @ref ovphysx_get_last_error for details. A CPU-only process never reaches
     *         CUDA through this entry point unless it passes a non-zero @p event.
     *
     * @note The event belongs to the read session and stays valid until the session is released.
     *       This call does not take ownership of either argument.
     *
     * @note **Context.** The call is issued in whatever CUDA context is current on the calling
     *       thread. ovphysx's own context is deliberately not pushed. The sentinels 1 and 2 are
     *       context-relative, so they resolve against the consumer's context, the one that queued
     *       the work being ordered. An explicit `CUstream` handle carries its own context and is
     *       unaffected. Callers on a non-current context should pass one rather than a sentinel.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_cuda_stream_wait_event(uintptr_t stream, uintptr_t event);

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
    * @return ovphysx_enqueue_result_t with status and operation index for the clone. On failure,
    *         call ovphysx_get_last_error() on the same thread for the error message.
    *
    * @note Replication executes inline. On success, the returned operation index is already
    *       complete. ovphysx_wait_op() remains valid and returns immediately.
    *
    * @pre handle must be valid.
    * @pre source_path_in_usd must exist and target_paths must be valid and unique.
    * @pre Must be called **before** ovphysx_warmup() or the first simulation step.
    *      Multiple ovphysx_clone() calls per attach are allowed in CPU and GPU mode
    *      while all of them precede warmup / first step. To clone after that point,
    *      use ovphysx_reset_stage() and reattach the source stage first. On GPU,
    *      cloning later would reallocate DirectGPU buffers and corrupt initialized
    *      state.
    * @post Cloned physics objects are live when this call returns successfully.
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
        const float* anchor_transforms,
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
    * The simulation time is tracked internally. Each step advances it by step_dt.
    * @param handle ovphysx instance
    * @param step_dt Simulation timestep in seconds
    * @return ovphysx_enqueue_result_t with status, error, and operation index
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_enqueue_result_t step_one_frame(ovphysx_handle_t handle)
     * {
     *     return ovphysx_step(handle, 1.0f / 60.0f);
     * }
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
     * entirely (mutex acquisitions, operation map insert/lookup/cleanup). In
     * IsaacLab RL training at 4096 environments this saves about 0.2 ms per
     * substep compared to step() + wait_op(), roughly 5-6% of total throughput.
     *
     * Use this whenever the caller steps and immediately waits for results,
     * that is, does not overlap GPU simulation with CPU work between dispatch
     * and fetch.
     *
     * The simulation time is tracked internally. Each step advances it by
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
     * simulation time + i * step_dt. This saves (n_steps-1) ctypes
     * round-trips for workloads that use decimation (one RL step =
     * multiple physics steps). The internal counter advances by
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
     * after writing articulation DOF positions (through the session write API,
     * @ref ovphysx_write, or the deprecated tensor bindings) and before reading link
     * pose tensors in the same frame.
     *
     * NOTE: On the first GPU kinematic update after loading USD, an automatic
     * warmup simulation step may be performed to initialize PhysX structures.
     * See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
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
     * DEPRECATED (ovphysx 0.6): the Tensor Binding API is superseded by the session
     * read/write API (ovphysx_read / ovphysx_write, Python PhysX.read / PhysX.write). It
     * remains available through the deprecation window. Write new code against the session
     * API. The description below documents the legacy behavior for maintaining existing
     * tensor-binding code.
     *
     * The Tensor Binding API provides efficient bulk access to physics simulation data.
     * It maps physics-object path patterns to typed tensor views, including authored USD objects
     * and runtime-only clones. This enables same-device zero-copy data exchange and, for binding
     * types whose storage follows the simulation device, transparent cross-device staging with
     * tensors (PyTorch, NumPy, etc.) via DLPack.
     *
     * DLPack compatibility:
     *   The official DLPack header is vendored (see dlpack/dlpack.h from github.com/dmlc/dlpack).
     *   When constructing DLTensor structs for read/write calls:
     *   - strides may be NULL to indicate C-contiguous layout, OR
     *     explicit C-contiguous strides may be set. Both are accepted.
     *   - Non-contiguous tensors (e.g. transposed views, sliced views with gaps) are
     *     NOT supported and are rejected. Calling .contiguous() on a PyTorch tensor
     *     first produces an accepted layout.
     *   - Supported dtypes: float32, int32, uint32, uint8, bool (kDLBool bits=8).
     *   - Supported devices: kDLCPU, kDLCUDAHost, kDLCUDA, and kDLCUDAManaged, subject to the
     *     binding type and process policy. When CUDA is available, CPU/CUDA mismatches are staged
     *     only for binding types whose storage follows the simulation device.
     *   - CPU-only property bindings cover standalone rigid-body mass/inertia/COM values,
     *     articulation DOF/body properties, rigid-body/articulation shape properties,
     *     deformable-material properties, and disable-simulation/gravity flags. Fixed and spatial
     *     tendon property bindings are not CPU-only.
     *   - CPU-only property bindings require host-resident kDLCPU or kDLCUDAHost destination,
     *     source, index, and mask tensors even when the simulation runs on GPU. kDLCUDA and
     *     kDLCUDAManaged tensors are rejected rather than staged.
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
     *   1. ovphysx_create_instance() (GPU is the default. Call ovphysx_set_cpu_mode(true)
     *      before creating any instance for process-wide CPU-only mode.)
     *   2. ovphysx_attach_ovstage() at the initial ordinal. Use
     *      ovphysx_update_from_ovstage() only for later authored ordinals.
     *   3. ovphysx_create_tensor_binding() for each tensor type needed.
     *   4. Optional: ovphysx_warmup() to control when warmup happens
     *      (otherwise it happens automatically on the first tensor read).
     *   5. Main loop: read/write tensors, step, wait.
     *
     * Pattern matching:
     *   Patterns use USD-style path glob syntax over realized physics objects:
     *   - Exact path: "/World/robot" - matches one object
     *   - Wildcard:   "/World/robot*" - matches robot1, robot2, robotArm, etc.
     *   - Nested:     "/World/env[N]/robot" with [N] as wildcard - matches /World/env0/robot, etc.
     *   A single path component (the text between two slashes; a parenthesized group
     *   counts as one component even if it contains a slash) may be at most 4096
     *   characters long; a longer component is rejected with OVPHYSX_API_INVALID_ARGUMENT.
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
     * when matching physics objects are added or recreated. Destroy the old binding and
     * create a new one after topology changes.
     *
     * Binding lifetime is tied to the currently realized physics objects. The
     * application owns the stage lifecycle: if it will call ovphysx_reset_stage(),
     * remove USD data containing bound objects, or otherwise replace/reparse the
     * stage so those objects are destroyed and recreated, cached bindings should
     * be destroyed before the lifecycle operation when practical. If a stale
     * binding survives, only destroy it. Do not read or write through it. Create
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
     *       underlying TensorAPI view. Destroy stale bindings and create
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
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_create_tensor_binding(
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t destroy_binding(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding)
     * {
     *     return ovphysx_destroy_tensor_binding(handle, binding);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_destroy_tensor_binding(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle);

    /**
     * Get the tensor layout specification for a binding.
     *
     * Returns dtype, ndim, and shape needed to allocate a compatible DLTensor.
     * This is the preferred API for constructing DLTensors correctly.
     *
     * NOTE: ovphysx_tensor_spec_t stores shape in a fixed-size int64[4] for a stable C ABI.
     * Only the first ndim entries are meaningful. The remaining entries are always set to 0.
     *
     * See ovphysx_tensor_type_t documentation for shapes, dtype, and layouts per tensor type.
     * Layout is always row-major contiguous (C-order). Most bindings are float32.
     * OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32 is int32.
     *
     * @note The returned specification does not include memory residency. Call
     * ovphysx_get_tensor_binding_native_device() to query the binding's native device.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_spec [out] Tensor dtype, rank, and shape
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t query_binding_spec(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     ovphysx_tensor_spec_t* out_spec)
     * {
     *     return ovphysx_get_tensor_binding_spec(handle, binding, out_spec);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_get_tensor_binding_spec(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_tensor_spec_t* out_spec);

    /**
     * Get the native device used by a tensor binding.
     *
     * Returns the device used by the binding's native TensorAPI read/write path:
     * `DLDevice{kDLCPU, 0}` for host-resident bindings, or
     * `DLDevice{kDLCUDA, device_ordinal}` for CUDA-resident bindings. CPU-only
     * property tensors report CPU even when the simulation runs on CUDA.
     * For kDLCUDA, device_id is the process-visible CUDA runtime ordinal used
     * with cudaSetDevice() or a framework device such as `cuda:N`, not a physical
     * PCI bus index.
     *
     * This reports native residency, not every device accepted by read/write.
     * Using the native device avoids staging when another accepted placement
     * would require it. The value remains stable for the lifetime of a live
     * binding. Unlike the layout-only spec getter, this is a live mapping query
     * and rejects an invalidated simulation view.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_device [out] Native DLPack device for the binding
     * @return ovphysx_result_t
     *
     * @pre handle, binding_handle, and out_device must be valid.
     * @post out_device is populated with the binding's native device.
     *
     * @par Side Effects
     * None.
     *
     * @par Synchronization
     * Waits for pending operations on the instance before inspecting the binding.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_device is NULL
     * - OVPHYSX_API_NOT_FOUND if binding_handle is unknown or stale
     * - OVPHYSX_API_ERROR if handle is invalid
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t query_binding_device(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     DLDevice* out_device)
     * {
     *     return ovphysx_get_tensor_binding_native_device(handle, binding, out_device);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read / @ref ovphysx_write instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_get_tensor_binding_native_device(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        DLDevice* out_device);

    // The deprecated tensor-binding group stays open through ovphysx_read_tensor_binding below.
    // ovphysx_warmup is placed in this block for layout only and carries its own
    // @ingroup ovphysx_simulation, so it is not published under "Deprecated APIs". A free-floating
    // @ingroup here would attach to the next declaration (ovphysx_read_tensor_binding) instead.

    /**
     * @anchor ovphysx_tensor_auto_warmup_note
     * @par Tensor auto-warmup note
     * The first tensor read or write after loading USD may perform an automatic
     * warmup simulation step to initialize PhysX lazy structures. In GPU mode
     * this also initializes DirectGPU buffers.
     *
     * The warmup is a real physics step that advances simulation time by a minimal
     * timestep (~1ns). Physics state may change infinitesimally. This is not a dry run.
     *
     * For deterministic behavior, explicitly control warmup timing by loading USD,
     * waiting for completion, then calling ovphysx_warmup() before the first
     * tensor read or write. To have the first observed state change happen under
     * a chosen timestep instead, call ovphysx_step() explicitly with that dt.
     *
     * Because warmup is a real simulation step, a true "pre-warmup" tensor state
     * cannot be observed. Calling ovphysx_warmup() explicitly only makes the timing
     * of that unavoidable step predictable.
     */

    /**
     * Read data from simulation into a user-provided DLTensor (synchronous).
     *
     * NOTE: On the first tensor read after loading USD, an automatic warmup simulation
     * step may be performed. See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
     *
     * DLTensor requirements:
     *   - MUST be pre-allocated with correct shape (use ovphysx_get_tensor_binding_spec())
     *   - dtype must match ovphysx_get_tensor_binding_spec()
     *   - when CUDA is available, CPU/CUDA mismatches are staged for binding types whose storage
     *     follows the simulation device
     *   - CPU-only property bindings require a host-resident kDLCPU or kDLCUDAHost destination.
     *     kDLCUDA and kDLCUDAManaged destinations return OVPHYSX_API_DEVICE_MISMATCH and are not
     *     staged
     *   - cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode return
     *     OVPHYSX_API_DEVICE_MISMATCH
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
     * May trigger warmup on first read.
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t read_binding(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     DLTensor* destination)
     * {
     *     return ovphysx_read_tensor_binding(handle, binding, destination);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read")
    ovphysx_result_t ovphysx_read_tensor_binding(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        DLTensor* dst_tensor);

    /** @} */

    /**
     * @ingroup ovphysx_simulation
     *
     * Explicitly run the warmup step (optional, synchronous).
     *
     * On first use, a real simulation step with a minimal timestep (~1ns) is run to
     * initialize PhysX structures and disable per-step Fabric sync overhead. This is
     * normally done automatically on the first tensor read (auto-warmup), but calling
     * it explicitly controls when the latency occurs.
     *
     * Works in both CPU and GPU mode. In GPU mode, this also populates DirectGPU buffers.
     *
     * IMPORTANT: The warmup advances simulation state (positions may change
     * infinitesimally). It is NOT a "dry run". For deterministic initial conditions,
     * call this before reading initial tensor state.
     *
     * This function is idempotent. Calling it multiple times has no effect after
     * the first successful call (per stage). Warmup state resets when the stage
     * changes (e.g., after reset() or loading a new USD file).
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
     * static ovphysx_result_t warm_up_instance(ovphysx_handle_t handle)
     * {
     *     return ovphysx_warmup(handle);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_warmup(ovphysx_handle_t handle);

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /**
     * Write data from a user-provided DLTensor into the simulation (synchronous).
     *
     * Not all tensor types are writable:
     * - RIGID_BODY_FORCE_F32, RIGID_BODY_WRENCH_F32, ARTICULATION_LINK_WRENCH_F32 are WRITE-ONLY
     *   (external control inputs applied each step, so reading them returns an error).
     * - RIGID_BODY_ACCELERATION_F32, RIGID_BODY_INV_MASS_F32, RIGID_BODY_INV_INERTIA_F32
     *   are READ-ONLY.
     * - ARTICULATION_LINK_POSE_F32, ARTICULATION_LINK_VELOCITY_F32, ARTICULATION_LINK_ACCELERATION_F32
     *   are READ-ONLY (no setter for individual link state).
     * - Inverse dynamics query tensors (JACOBIAN, MASS_MATRIX, CORIOLIS_AND_CENTRIFUGAL_FORCE, GRAVITY_FORCE,
     *   LINK_INCOMING_JOINT_FORCE, DOF_PROJECTED_JOINT_FORCE, BODY_INV_MASS, BODY_INV_INERTIA)
     *   are READ-ONLY.
     * - DEFORMABLE_REST_NODAL_POSITION_F32 and DEFORMABLE_SIM_ELEMENT_INDICES_S32
     *   are READ-ONLY.
     * - DOF_ACTUATION_FORCE_F32 is read-write (not write-only).
     * See ovphysx_tensor_type_t documentation for shapes, layouts, and read/write semantics.
     *
     * NOTE: On the first tensor write after loading USD, an automatic warmup
     * simulation step may be performed to initialize PhysX structures.
     * See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
     *
     * This is a blocking call that completes before returning.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param src_tensor User tensor with data to write (must match ovphysx_get_tensor_binding_spec())
     * @param index_tensor Optional int32[K] indices for subset write. NULL = write all.
     *   - When index_tensor != NULL: src_tensor must still have full shape [N, ...] matching the binding spec.
     *     Only the rows specified by index_tensor are written. Other rows in src_tensor are ignored.
     *   - Indices are 0-based into the first dimension N of the binding, and must satisfy 0 <= idx < N.
     *   - K (index count) must satisfy K <= N.
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre src_tensor must match the spec's dtype/shape and use a supported device. When CUDA is
     *      available, CPU/CUDA mismatches are staged for binding types whose storage follows the
     *      simulation device.
     * @pre For CPU-only property bindings, src_tensor and any index_tensor must use kDLCPU or
     *      kDLCUDAHost. Otherwise-valid kDLCUDA and kDLCUDAManaged tensors return
     *      OVPHYSX_API_DEVICE_MISMATCH and are not staged.
     * @pre Cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode return
     *      OVPHYSX_API_DEVICE_MISMATCH.
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t write_binding(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     const DLTensor* source)
     * {
     *     return ovphysx_write_tensor_binding(handle, binding, source, NULL);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_write instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_write")
    ovphysx_result_t ovphysx_write_tensor_binding(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* src_tensor,
        const DLTensor* index_tensor);

    /**
     * Write data from a user-provided DLTensor into the simulation using a binary mask (synchronous).
     *
     * Only elements where mask[i] != 0 are written. Other elements are left unchanged.
     * This is the mask-based alternative to indexed writes via ovphysx_write_tensor_binding.
     *
     * NOTE: On the first tensor write after loading USD, an automatic warmup
     * simulation step may be performed to initialize PhysX structures.
     * See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
     *
     * @note There is intentionally no corresponding read_masked function. Reads always return
     *   the full [N,...] tensor via ovphysx_read_tensor_binding(). Callers that need a subset
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
     * @pre mask_tensor must be 1D uint8/bool with length N on a supported device. When CUDA is
     *      available, CPU/CUDA mismatches are staged for binding types whose storage follows the
     *      simulation device.
     * @pre For CPU-only property bindings, src_tensor and mask_tensor must use kDLCPU or
     *      kDLCUDAHost. Otherwise-valid kDLCUDA and kDLCUDAManaged tensors return
     *      OVPHYSX_API_DEVICE_MISMATCH and are not staged.
     * @pre Cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode return
     *      OVPHYSX_API_DEVICE_MISMATCH.
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t write_masked_binding(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     const DLTensor* source,
     *     const DLTensor* mask)
     * {
     *     return ovphysx_write_tensor_binding_masked(
     *         handle, binding, source, mask);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_write instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_write")
    ovphysx_result_t ovphysx_write_tensor_binding_masked(
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
     * spatial_tendon_count, and is_fixed_base. All values are stable for the binding
     * lifetime, so the result can be cached.
     *
     * **Homogeneous topology requirement**: all articulations covered by this binding
     * must have the same topology (same dof_count, body_count, joint_count, etc.).
     * This is a constraint of the native tensor backend, whose tensor shapes are fixed at
     * binding creation time. Articulations of different sizes (e.g. a 7-DOF arm and a
     * 30-DOF humanoid) need a separate binding each.
     *
     * For name arrays (DOF names, body names, joint names) use the corresponding
     * ovphysx_articulation_get_*_names functions.
     *
     * @param handle          Instance handle
     * @param binding_handle  Tensor binding (must be an articulation binding)
     * @param out_metadata    [out] Caller-allocated struct to fill
     * @return ovphysx_result_t
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     * #include <stdio.h>
     *
     * static ovphysx_result_t print_articulation_size(
     *     ovphysx_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding)
     * {
     *     ovphysx_articulation_metadata_t metadata = {0};
     *     ovphysx_result_t result =
     *         ovphysx_get_articulation_metadata(handle, binding, &metadata);
     *     if (result.status == OVPHYSX_API_SUCCESS)
     *         printf("DOFs: %d  Links: %d\n",
     *                metadata.dof_count, metadata.body_count);
     *     return result;
     * }
     * @endcode
     *
     * @deprecated Part of the tensor-binding surface. It requires a binding handle and is
     *   removed with the binding. No non-binding successor exists yet: a read-API
     *   topology/names path is a removal-blocker, so use this only to maintain existing code.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled metadata API is removed with them (no non-binding successor yet)")
    ovphysx_result_t ovphysx_get_articulation_metadata(
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
     * @param max_names Capacity of out_names array. Set to metadata.dof_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. No non-binding successor yet. See @ref ovphysx_get_articulation_metadata.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled names API is removed with them (no non-binding successor yet)")
    ovphysx_result_t ovphysx_articulation_get_dof_names(
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
     * @param max_names Capacity of out_names array. Set to metadata.body_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. No non-binding successor yet. See @ref ovphysx_get_articulation_metadata.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled names API is removed with them (no non-binding successor yet)")
    ovphysx_result_t ovphysx_articulation_get_body_names(
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
     * @param max_names Capacity of out_names array. Set to metadata.joint_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. No non-binding successor yet. See @ref ovphysx_get_articulation_metadata.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled names API is removed with them (no non-binding successor yet)")
    ovphysx_result_t ovphysx_articulation_get_joint_names(
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
     * storage. String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array. Must be at least the binding
     *   count to receive all paths.
     * @param out_count [out] Actual number of paths written
     * @return ovphysx_result_t
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_tensor_binding_get_prim_paths(
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
     * ovphysx_read_friction_data(). Use ovphysx_get_contact_report() for
     * event headers or raw actor-pair report records.
     *
     * A **sensor** is a set of rigid bodies identified by a physics-object path
     * pattern passed to ovphysx_create_contact_binding(). A **filter** is a
     * second set of bodies whose contacts with each sensor are measured. Both
     * patterns can resolve authored USD objects and runtime-only clones.
     *
     * Contact reporting is opt-in: every authored USD prim matched by a sensor
     * pattern must have `PhysxContactReportAPI` applied, on the prim named as the
     * sensor itself (not a parent body or child collider). A matched prim without
     * the schema is dropped from the binding, and if that leaves no sensors the
     * call fails. Filter prims need no extra schema, and runtime-only clones
     * inherit contact reporting from the source actor.
     *
     * **Lifecycle and read timing**
     *
     * A contact binding must be created *before* the first simulation step whose
     * contacts are to be observed. After creation it registers sensors inside
     * the PhysX contact-report callback. No contact data exists until at least
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
     *   `ovphysx_step_sync()`, or `ovphysx_step_n_sync()` call. It is not
     *   passed explicitly.
     *
     * **Why a separate binding type (not tensor binding)**
     *
     * Contact data has a fundamentally different shape and semantics from
     * articulation/rigid-body tensors:
     * - Shape is `[S, F, 3]` (sensor x filter x xyz), determined at binding
     *   creation time for aggregate force matrices, plus flat detailed contact
     *   buffers indexed by `[S, F]` count/start-index tensors.
     * - Contact binding is read-only. There is no write path.
     * - Internally it uses the PhysX contact-report callback rather than
     *   DirectGPU buffers, so sharing a handle type with tensor binding would
     *   misrepresent the lifetime and threading model.
     */

    /** @addtogroup ovphysx_contact_binding */
    /** @{ */

    /**
     * @brief Create a contact binding for reading net contact forces and force matrices.
     *
     * @param handle Instance handle
     * @param sensor_patterns Array of physics-object path patterns matching sensor bodies.
     *   A single path component longer than 4096 characters (here or in filter_patterns)
     *   is rejected with OVPHYSX_API_INVALID_ARGUMENT.
     * @param sensor_patterns_count Number of sensor patterns
     * @param filter_patterns Flat array of filter object-path patterns. All sensors must
     *   have the same number of filters. Total length = sensor_patterns_count * filters_per_sensor.
     *   Pass NULL with filters_per_sensor=0 for unfiltered contacts.
     * @param filters_per_sensor Number of filter patterns per sensor (same for all sensors)
     * @param max_contact_data_count Max detailed contact/friction entries to track.
     *   Set this to a positive value before using ovphysx_read_contact_data()
     *   or ovphysx_read_friction_data(). Detailed reads also require
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
     * ovphysx owns the returned string storage. String pointers remain valid
     * until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array. Must be at least
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
     * storage. String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array. Must be at least
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
     * @brief Query detailed contact/friction flat-buffer capacity.
     *
     * This is the C dimension for `ovphysx_read_contact_data()` and
     * `ovphysx_read_friction_data()` flat buffers. Allocate force/separation
     * buffers as `[C, 1]`, point/normal/friction buffers as `[C, 3]`, and
     * count/start-index buffers as `[S, F]`, where `C` is this value and
     * `S`, `F` come from `ovphysx_get_contact_binding_spec()`.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_max_contact_data_count [out] Max detailed contact/friction entries
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
     * `C` and `F` must be positive. Pass a positive max_contact_data_count
     * and filters_per_sensor > 0 when creating the binding. Count and
     * start-index tensors may be int32 or uint32.
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
     * @return ovphysx_result_t
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
     * buffers. `C` and `F` must be positive. Pass a positive
     * max_contact_data_count and filters_per_sensor > 0 when creating the
     * binding. Count and start-index tensors may be int32 or uint32.
     * Friction forces use the timestep from the last successful
     * ovphysx_step(), ovphysx_step_sync(), or ovphysx_step_n_sync() call.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param friction_force_tensor Pre-allocated world-frame friction forces
     * @param friction_point_tensor Pre-allocated world-frame friction points
     * @param contact_count_tensor Pre-allocated count matrix
     * @param contact_start_indices_tensor Pre-allocated start-index matrix
     * @return ovphysx_result_t
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
     * collided with, plus per-contact actor-identity tensors so callers can
     * identify both the sensor and the contacting body via
     * @ref ovphysx_contact_binding_get_other_actor_paths_from_ids.
     *
     * Required shapes (C = max_contact_data_count, S = sensor_count):
     * - contact_force_tensor:        `[C, 1]` float32, contact normal force magnitude
     * - contact_point_tensor:        `[C, 3]` float32, contact point in world frame
     * - contact_normal_tensor:       `[C, 3]` float32, contact normal in world frame
     * - contact_separation_tensor:   `[C, 1]` float32, signed separation
     * - sensor_layout_tensor:        `[S, 2]` int32/uint32, per sensor, column 0 is the
     *   contact count and column 1 is its start index into the flat buffers
     * - actor_ids_tensor:            `[C, 2]` int64/uint64, per contact, column 0 is the
     *   reporting sensor's own actor id and column 1 is the actor it contacted
     *
     * The two pairs that are only meaningful together are single tensors rather than
     * separate buffers the caller has to keep in step. The four per-contact value
     * tensors stay separate, matching ovphysx_read_contact_data().
     *
     * The contact binding must be created with `max_contact_data_count > 0`.
     * No filter dimension is required, so `filters_per_sensor` may be zero. The
     * dt for impulse-to-force conversion is taken automatically from the last
     * successful ovphysx_step(), ovphysx_step_sync(), or
     * ovphysx_step_n_sync() call.
     *
     * **Truncation**: when the total contact count for a step exceeds
     * max_contact_data_count, the runtime fills the flat buffers with as many
     * contacts as fit and emits a logged warning. A sensor's count reports only
     * the contacts actually written, and its start index is clamped to
     * max_contact_data_count, so `[start, start + count)` is always an in-range
     * (possibly empty) slice. Callers that need every contact must pass a larger
     * max_contact_data_count.
     *
     * **Token lifetime**: the uint64 tokens in actor_ids_tensor are opaque runtime
     * actor handles, not encoded paths. Do not decode one. Resolve it with
     * @ref ovphysx_contact_binding_get_other_actor_paths_from_ids. They are stable
     * for as long as the corresponding actor is alive on the attached stage, and
     * become stale once it is removed or the stage is detached or replaced.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_read_raw_contact_data(
        ovphysx_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* contact_force_tensor,
        DLTensor* contact_point_tensor,
        DLTensor* contact_normal_tensor,
        DLTensor* contact_separation_tensor,
        DLTensor* sensor_layout_tensor,
        DLTensor* actor_ids_tensor);

    /**
     * @brief Resolve actor IDs from @ref ovphysx_read_raw_contact_data to physics-object paths.
     *
     * Given a tensor of opaque actor IDs (a column of the `actor_ids_tensor`
     * `[C, 2]` returned by @ref ovphysx_read_raw_contact_data, either column,
     * since both use the same namespace), fills `out_paths` with the corresponding
     * physics-object paths in the same order. An ID of `0` yields an empty path,
     * as does an ID that is not known to the attached stage at all.
     *
     * `ids_tensor` must be **CPU-resident** (path resolution is a host operation)
     * and **C-contiguous**: a column of `actor_ids_tensor` is a strided view, so
     * make the slice contiguous (e.g. `np.ascontiguousarray(actor_ids[:, 1])`)
     * before passing it. This is the diagnostic path. The per-contact read itself
     * takes the whole contiguous tensor and pays no copy.
     *
     * **Stale IDs report as empty, not as their old path.** Every non-zero ID is
     * checked against the attached stage before it is resolved, so an actor that
     * has since been removed comes back as an empty path rather than the path it
     * used to name. Because the caller holds the IDs, this makes the failure
     * explicit: for a non-zero ID an empty path means "not resolvable now", and
     * only ID `0` yields an empty path for a live read.
     *
     * The check is as precise as the attached backend's notion of existence. On an
     * ovstage attach a removed prim reports stale. On a USD stage a merely
     * *deactivated* prim still resolves, because existence there follows prim
     * validity and USD hands back a valid prim for an inactive one.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param ids_tensor Input tensor of actor IDs (`[N]` int64/uint64)
     * @param out_paths [out] Array of @ref ovphysx_string_t to fill. ovphysx
     *   owns the returned string storage. Pointers remain valid until the
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
     * Returns **per-contact-point event data**: position, normal, impulse,
     * and separation for every contact point this step. Use this for custom
     * contact sensors, collision debugging, or per-point force analysis.
     *
     * For **aggregate force tensors** (net forces or force matrices between
     * sensor/filter body sets, delivered as DLPack tensors), see the Contact
     * Binding API: ovphysx_create_contact_binding().
     *
     * The header array describes contact pairs (which actors/colliders are
     * in contact). Each header references a slice of the contact data array
     * containing per-contact-point information (position, normal, impulse,
     * separation).
     *
     * Prims involved in contacts must have `PhysxContactReportAPI` applied
     * in the USD stage for contacts to be reported.
     *
     * **Ownership:** The caller does NOT own the returned arrays. They are
     * read-only views into internal simulation buffers. Copy any data that
     * must be kept beyond the current step.
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
     * Get a raw PhysX SDK object pointer by selector and type.
     *
     * Returns the underlying PhysX object as an opaque `void*`. The caller
     * must cast to the appropriate PhysX C++ type (see ovphysx_physx_type_t).
     *
     * A single function covers all PhysX object types. No per-type variants
     * are needed.
     *
     * @param handle   ovphysx instance handle.
     * @param prim_path Physics-object path (ovphysx_string_t, e.g.
     *        "/World/physicsScene", "/World/Cube", "/World/articulation").
     *        For OVPHYSX_PHYSX_TYPE_PHYSICS, pass a zero-length string view:
     *        either `{ NULL, 0 }` or `{ "", 0 }`. Embedded NUL bytes are rejected.
     * @param physx_type Which PhysX object type to look up. Path-bound types
     *        are looked up at prim_path. See ovphysx_physx_type_t for the
     *        mapping to PhysX C++ types.
     * @param[out] out_ptr Receives the PhysX pointer on success. Set to NULL
     *        on failure when out_ptr itself is valid.
     * @return ovphysx_result_t with status and error info.
     *
     * @pre A stage must be attached and initialized by at least one completed
     *      simulation step (so PhysX objects exist). prim_path must be a valid,
     *      non-empty physics-object path except for OVPHYSX_PHYSX_TYPE_PHYSICS,
     *      which requires a zero-length selector.
     *
     * **Pointer lifetime:** Returned pointers are borrowed. Treat them as invalid
     * after stage reset or detachment, or after ovphysx_destroy_instance().
     * Reacquire them after attaching and initializing another stage. Calls to
     * ovphysx_step() do NOT invalidate existing pointers. Do not call `release()`
     * on returned pointers. ovphysx owns them. This applies to the process-global
     * PxPhysics pointer as well as path-bound objects. Objects that callers
     * explicitly create through a returned factory pointer follow the PhysX SDK's
     * ownership rules. The no-release rule applies to the borrowed pointer returned
     * by this function.
     *
     * **Thread safety:** PhysX APIs on returned pointers must only be called
     * between simulation steps, specifically after wait_op() completes for
     * the preceding step and before the next ovphysx_step() call. Calling
     * PhysX APIs while a step is in-flight is a data race.
     *
     * **Do not toggle simulation disable through actor pointers.** Setting or
     * clearing `PxActorFlag::eDISABLE_SIMULATION` on a `PxRigidDynamic*` from
     * this API is unsupported. ovphysx does not observe that change, and on DirectGPU
     * scenes the next read or write may address the wrong body with no error.
     * Use the ovstage `disableSimulation` attribute (`ovphysx_write()` /
     * `OVPHYSX_ATTR_DISABLE_SIMULATION`) instead. That disables a standalone rigid
     * body; a point-instancer instance cannot be disabled individually in this
     * release (`disableSimulation` is not an instancer-writable column and there is
     * no per-instance tensor route), so disabling one instance is unsupported.
     *
     * **Shapes:** PxShape objects are reachable from a PxRigidActor pointer
     * via `PxRigidActor::getShapes()`. They can also be queried directly with
     * `OVPHYSX_PHYSX_TYPE_SHAPE`.
     *
     * **PhysX SDK headers:** Casting the returned pointer requires the PhysX
     * SDK C++ headers (e.g. `PxScene.h`, `PxRigidDynamic.h`). The ovphysx
     * SDK ships these headers under `include/physx/`, and `find_package(ovphysx)`
     * sets `ovphysx_PHYSX_INCLUDE_DIR` to point there. Consumers must use
     * those exact shipped headers for the ovphysx build. Do not substitute
     * external PhysX headers. No PhysX library linking is needed.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static ovphysx_result_t get_scene_and_physics(
     *     ovphysx_handle_t handle, void** out_scene, void** out_physics)
     * {
     *     ovphysx_result_t result = ovphysx_get_physx_ptr(
     *         handle, OVPHYSX_LITERAL("/World/physicsScene"),
     *         OVPHYSX_PHYSX_TYPE_SCENE, out_scene);
     *     if (result.status != OVPHYSX_API_SUCCESS) {
     *         return result;
     *     }
     *
     *     ovphysx_string_t no_path = { NULL, 0 };
     *     return ovphysx_get_physx_ptr(
     *         handle, no_path,
     *         OVPHYSX_PHYSX_TYPE_PHYSICS, out_physics);
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
     * `prim_path` -- see @ref ovphysx_object_type_t for the taxonomy, which
     * distinguishes standalone (`OVPHYSX_OBJECT_TYPE_JOINT`), custom
     * (`OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT`), and articulation
     * (`OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT`) joints.
     *
     * Pair object types with the matching @ref ovphysx_get_physx_ptr selector:
     * `OVPHYSX_OBJECT_TYPE_JOINT` with `OVPHYSX_PHYSX_TYPE_JOINT`,
     * `OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT` with `OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT`,
     * and `OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT` with
     * `OVPHYSX_PHYSX_TYPE_LINK_JOINT`.
     *
     * Paths with no classified simulation object yield
     * `OVPHYSX_OBJECT_TYPE_INVALID` with `OVPHYSX_API_SUCCESS` (the call
     * itself didn't fail; for example the prim is absent or has no matching
     * TensorAPI object type). Live standalone, custom, and articulation joints
     * at their authored prim paths must not report INVALID.
     *
     * @param handle Instance handle (must have a stage attached)
     * @param prim_path Authored USD prim path (embedded NUL bytes are rejected)
     * @param[out] out_type Receives the object type
     * @return OVPHYSX_API_SUCCESS on success (including the INVALID
     *         classification above), OVPHYSX_API_INVALID_ARGUMENT if
     *         `out_type` is NULL or `prim_path` is NULL, empty, or contains an
     *         embedded NUL byte, or OVPHYSX_API_ERROR if no stage is attached
     *         or the TensorAPI simulation view cannot be created.
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
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. @ref ovphysx_update_articulations_kinematic is NOT a drop-in replacement: it updates
     *   every articulation in the instance (this call is scoped to the binding's subset), does not
     *   honor the POSITION / VELOCITY @p flags, and is a no-op on CPU. A subset, flag-selective, or
     *   CPU caller has no equivalent yet. Closing that gap is a removal-blocker.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled kinematic update is removed with them "
        "(ovphysx_update_articulations_kinematic is not a drop-in: whole-instance only, no POSITION/VELOCITY flags, no-op on CPU)")
    ovphysx_result_t ovphysx_articulation_update_kinematic(
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
     * Typical pair: clear the actor's `eDISABLE_SIMULATION` flag through the
     * ovstage `disableSimulation` write (re-add to simulation), then call this
     * to bring the actor active for the next simulate -- otherwise it sits in
     * the sleep state PhysX placed it in when the flag was first set.
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
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. No non-binding successor yet (a session wake/sleep control is a removal-blocker).
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled wake/sleep API is removed with them (no non-binding successor yet)")
    ovphysx_result_t ovphysx_rigid_body_view_wake_up(
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
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. No non-binding successor yet (a session wake/sleep control is a removal-blocker).
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled wake/sleep API is removed with them (no non-binding successor yet)")
    ovphysx_result_t ovphysx_rigid_body_view_sleep(
        ovphysx_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* indices);

    /**
     * Subscribe to PhysX object create/destroy notifications.
     *
     * Pair with ovphysx_get_physx_ptr() to manage the lifetime of cached PhysX
     * SDK pointers. On a destruction notification the caller MUST drop the
     * cached pointer before returning from the callback. On a creation
     * notification the caller should mark the prim path as dirty in its own
     * cache and call ovphysx_get_physx_ptr() to fetch the new pointer ONLY
     * after the triggering synchronous call returns, or after
     * ovphysx_wait_op() returns for async work, never from inside the
     * callback itself (see Threading below).
     *
     * Subscriptions are PROCESS-GLOBAL, not per-instance. A single subscription
     * receives events from every ovphysx instance in the process. Multi-instance
     * callers that need to filter by stage must do so on their side. The
     * prim_path delivered to the callback is the only identifier available.
     * The handle parameter is intentionally absent.
     *
     * Lifecycle: callbacks fire for changes that occur during simulation and
     * ovphysx_reset_stage()'s bulk teardown. The initial object population from
     * ovstage attach/update is NOT notified, since the caller already has that
     * state from setup.
     *
     * Known limitation: ovphysx_clone() does NOT currently fire
     * object_created notifications. Pointer caches that need to track cloned
     * objects must be refreshed after ovphysx_wait_op() returns on the
     * clone operation, not via this subscription. ovphysx_reset_stage() (and the
     * all-objects-destroyed callback) is the supported path for bulk pointer
     * invalidation.
     *
     * Threading: callbacks may fire from internal worker threads. Do NOT call
     * other ovphysx APIs from inside a callback. Defer that until the
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
     *                               copied internally, so the caller does not need
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
     * Path fields in the hit struct (collision, rigid_body, material) hold an
     * opaque omni::physics::parse::ObjectKey.handle, not a uint64-encoded
     * SdfPath. There is no client-side bit-cast that reproduces or compares
     * against it. Resolve a hit's identity fields to a prim path with
     * @ref ovphysx_scene_query_get_paths_from_ids instead of comparing raw
     * values.
     *
     * @pre A USD stage must be loaded and at least one simulation step
     *      completed for the scene to contain queryable objects.
     *
     * **Thread safety:** Scene query functions must only be called between
     * simulation steps, after `wait_op()` completes and before the next
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     * #include <stdio.h>
     *
     * static ovphysx_result_t print_closest_hit(ovphysx_handle_t handle)
     * {
     *     const ovphysx_scene_query_hit_t* hits = NULL;
     *     uint32_t count = 0;
     *     const float origin[3] = {0.0f, 10.0f, 0.0f};
     *     const float direction[3] = {0.0f, -1.0f, 0.0f};
     *     ovphysx_result_t result = ovphysx_raycast(
     *         handle, origin, direction, 100.0f, false,
     *         OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
     *     if (result.status == OVPHYSX_API_SUCCESS && count > 0)
     *         printf("Hit at distance %f\n", hits[0].distance);
     *     return result;
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
     * are zeroed. Only object identity (collision, rigid_body, proto_index)
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

    /**
     * @brief Resolve object-identity handles to physics-object paths.
     *
     * Given an array of opaque object-identity handles produced by this API,
     * fills `out_paths` with the corresponding physics-object paths in the same
     * order. Despite the `scene_query` name the resolution is generic: any
     * identity field carries the same opaque handle, so scene-query hits
     * (`collision`, `rigid_body`, `material` of an
     * @ref ovphysx_scene_query_hit_t from @ref ovphysx_raycast,
     * @ref ovphysx_sweep or @ref ovphysx_overlap) and contact-report identities
     * (`actor0/1`, `collider0/1` of an @ref ovphysx_contact_event_header_t,
     * `material0/1` of an @ref ovphysx_contact_point_t) are all accepted. IDs
     * that cannot be resolved (a zero/sentinel id, an id from an object removed
     * since it was reported, or a call with no active attach) yield empty paths
     * rather than an error.
     *
     * @param handle Instance handle.
     * @param ids Input array of opaque identity handles (see fields above).
     * @param id_count Number of entries in `ids`.
     * @param[out] out_paths Array of @ref ovphysx_string_t to fill. ovphysx
     *   owns the returned string storage. Unlike
     *   @ref ovphysx_contact_binding_get_other_actor_paths_from_ids, these
     *   pointers are not a per-call cache. Each is owned by the currently
     *   attached physics source and stays valid only until the next detach or
     *   re-attach. A later call to this function (against the same attach)
     *   does not invalidate a pointer returned by an earlier one. Callers
     *   that need a path to outlive a detach/re-attach must copy it.
     * @param max_paths Capacity of `out_paths` array.
     * @param[out] out_count Total number of ids in `ids` (== id_count). Only
     *   `min(id_count, max_paths)` entries are written to `out_paths`, so
     *   compare against `max_paths` to detect truncation.
     * @return ovphysx_result_t
     */
    OVPHYSX_API ovphysx_result_t ovphysx_scene_query_get_paths_from_ids(
        ovphysx_handle_t handle,
        const uint64_t* ids,
        uint32_t id_count,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
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
    * @pre handle and desc must be valid. desc->run must not be NULL.
    * @post Task is executed in stream order once enqueued.
    *
    * @par Side Effects
    * Executes user callback on the internal execution context.
    *
    * @par Threading
    * Callback may execute on an internal worker thread. Avoid blocking or taking locks that can deadlock.
    *
    * @par Ownership
    * Caller must keep desc->user_data valid until the task executes.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code{.c}
    * #include <ovphysx/ovphysx.h>
    *
    * static ovphysx_result_t run_user_task(
    *     ovphysx_handle_t handle,
    *     ovphysx_op_index_t op_index,
    *     void* user_data)
    * {
    *     (void)handle;
    *     (void)op_index;
    *     (void)user_data;
    *     return (ovphysx_result_t){OVPHYSX_API_SUCCESS};
    * }
    *
    * static ovphysx_enqueue_result_t enqueue_user_task(ovphysx_handle_t handle)
    * {
    *     const ovphysx_user_task_desc_t task = {run_user_task, NULL};
    *     return ovphysx_add_user_task(handle, &task);
    * }
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_add_user_task(ovphysx_handle_t handle,
                                                                       const ovphysx_user_task_desc_t* desc);

    /*
    * Wait for completion of all operations up to and including the specified operation index.
    * This operation is synchronous. Poll and finite simulation waits use the generic tracked-operation path so
    * they can check readiness without entering the direct blocking-sync fast path, which is reserved for eligible
    * infinite waits. A finite timeout bounds waiting for operation readiness. A final readiness observation wins
    * at the deadline, and synchronous result finalization may extend total wall-clock duration beyond the timeout.
    * Passing 0 as the timeout makes the operation a non-blocking readiness poll.
    * The out structure returns any errors observed since the last wait call. On timeout, lowest_pending_op_index
    * is set to the lowest pending operation index (or 0 if all complete).
    * The caller owns out_wait_result and must free it via ovphysx_destroy_wait_result().
    * For each failed op index, call ovphysx_get_last_op_error(op_index) to retrieve the error string.
    *
    * SINGLE-USE SEMANTICS:
    *   Each op_index is single-use. Once a wait reports completion or operation failure, the op_index is
    *   consumed and its resources are released. Attempting to wait on the same op_index again returns
    *   OVPHYSX_API_NOT_FOUND. A wait processes unconsumed indices in order through the requested index,
    *   consuming every completed or failed index it reaches. An index completed by internal stream
    *   synchronization may be acknowledged once with wait_op(). That acknowledgement consumes it.
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
    * @param timeout_ns Readiness timeout in nanoseconds. OVPHYSX_TIMEOUT_POLL performs a
    *        non-blocking readiness poll. OVPHYSX_TIMEOUT_INFINITE waits indefinitely.
    * @param out_wait_result [out] Wait result information (errors and active operation indices)
    * @return ovphysx_result_t with status:
    *         OVPHYSX_API_SUCCESS if the operations were waited for successfully,
    *         OVPHYSX_API_ERROR if one or more operations failed or the wait failed internally,
    *         OVPHYSX_API_NOT_FOUND if the handle is invalid or the op_index was consumed or never existed,
    *         OVPHYSX_API_TIMEOUT if a pending operation did not become ready within the timeout.
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
    * - OVPHYSX_API_TIMEOUT if the readiness timeout elapses
    * - OVPHYSX_API_ERROR if one or more operations fail or the wait fails internally
    *
    * @code{.c}
    * #include <ovphysx/ovphysx.h>
    *
    * static ovphysx_result_t wait_for_operation(
    *     ovphysx_handle_t handle, ovphysx_op_index_t op_index)
    * {
    *     ovphysx_op_wait_result_t wait_result = {0};
    *     ovphysx_result_t result =
    *         ovphysx_wait_op(
    *             handle, op_index, OVPHYSX_TIMEOUT_INFINITE, &wait_result);
    *     ovphysx_destroy_wait_result(&wait_result);
    *     return result;
    * }
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_wait_op(ovphysx_handle_t handle,
                                                         ovphysx_op_index_t op_index,
                                                         ovphysx_timeout_t timeout_ns,
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
     * @brief Query the error string for a specific failed op_index from the last wait_op call.
     *
     * After ovphysx_wait_op() reports failed operations via error_op_indices,
     * call this function for each failed op_index to retrieve the error message.
     * The returned string is valid until the next ovphysx_wait_op() call on the same thread.
     *
     * @param op_index The failed operation index (from ovphysx_op_wait_result_t.error_op_indices).
     * @return Error message, or {"", 0} if op_index has no error. ptr is always
     *         non-NULL and ptr[length] is '\0'.
     *
     * @par Threading
     * Thread-local storage. Safe to call from any thread.
     *
     * @see ovphysx_wait_op for the wait-result API that supplies failed
     *      operation indices.
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
     * @code{.c}
     * #include <ovphysx/ovphysx.h>
     *
     * static void release_wait_result(ovphysx_op_wait_result_t* result)
     * {
     *     ovphysx_destroy_wait_result(result);
     * }
     * @endcode
     */
    OVPHYSX_API void ovphysx_destroy_wait_result(ovphysx_op_wait_result_t* result);

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
     * Use ovphysx_set_log_level() to change the threshold and
     * ovphysx_set_log_callback() to receive messages programmatically.
     */

    /** @addtogroup ovphysx_logging */
    /** @{ */

    /**
     * @brief Set the process-scoped libovphysx source log level threshold.
     *
     * Messages emitted by the named Carbonite sources `omni_physx_sdk`,
     * `omni.physx`, and `ovphysx_internal` below this level are suppressed for
     * console and application-callback delivery. Every other process source and
     * channel, including any unnamed source, remains unchanged and is subject
     * only to the callback's minimum severity and channel filter.
     * OVPHYSX_LOG_NONE therefore mutes only the three named sources. It is not a
     * whole-runtime or process mute. Callable at any time.
     * If called before instance creation, the level is stored and applied when
     * Carbonite initializes.
     *
     * @par Threading
     * Thread-safe.
     *
     * @note Must not be called from within the application log callback. It
     *       returns OVPHYSX_API_ERROR without changing the setting.
     *
     * @param level Unsigned value corresponding to ovphysx_log_level_t. Default:
     *              OVPHYSX_LOG_WARNING. OVPHYSX_LOG_DEFAULT restores the
     *              library default (WARNING).
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success, or
     *         OVPHYSX_API_INVALID_ARGUMENT if the level was out of range
     *         (no state change is applied), or OVPHYSX_API_ERROR when called
     *         from the active log callback.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_set_log_level(uint32_t level);

    /**
     * @brief Get the current process-scoped libovphysx source log level threshold.
     *
     * @par Threading
     * Thread-safe.
     *
     * @return Unsigned value corresponding to the current ovphysx_log_level_t.
     */
    OVPHYSX_API uint32_t ovphysx_get_log_level(void);

    /**
     * @brief Enable or disable Carbonite's built-in console log output.
     *
     * By default, Carbonite logs to the console (stdout/stderr). When a custom
     * callback is set via ovphysx_set_log_callback(), both the built-in
     * console output and the custom callback receive messages,
     * which may cause duplicate output if the callback also writes to the
     * console.
     *
     * Call this function with @c false to suppress the built-in console
     * output while keeping the custom callback active. Call with @c true to
     * re-enable it.
     *
     * This function is independent of callback registration and the
     * process-scoped libovphysx source log level. It controls Carbonite's
     * process-global built-in console logger and therefore affects every
     * Carbonite tenant in the process. Multi-tenant hosts should normally own
     * this policy themselves and leave this function enabled.
     *
     * Callable at any time. If called before Carbonite initializes, the
     * preference is stored and applied during initialization.
     *
     * @par Threading
     * Thread-safe.
     *
     * @param enable @c true to enable (default), @c false to disable.
     * @return OVPHYSX_API_SUCCESS on success, or OVPHYSX_API_ERROR when called
     *         from the application log callback.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_enable_default_log_output(bool enable);

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
     * configured by ovphysx_set_log_level(). OVPHYSX_LOG_DEFAULT uses WARNING.
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

    /*=========================================================================
     * SDF shape evaluation
     *=========================================================================*/

    /**
     * Create an SDF shape view for shapes matching the given pattern.
     *
     * The view evaluates the signed distance field of PhysX collision shapes
     * at caller-supplied query points. Requires a GPU instance. CPU SDF
     * evaluation is not implemented and create fails for CPU instances.
     *
     * The returned handle must be released with ovphysx_destroy_sdf_view when
     * no longer needed. The handle is invalidated when the attached USD stage
     * changes (ovphysx_reset_stage, ovphysx_detach_ovstage, or loading a new
     * stage) or when the instance is destroyed (via ovphysx_destroy_instance) even if
     * ovphysx_destroy_sdf_view is not called explicitly. After invalidation,
     * evaluate/get calls return OVPHYSX_API_NOT_FOUND. Recreate the view after
     * re-attaching a stage.
     *
     * Thread safety: safe to call concurrently with other ovphysx API functions
     * on the same handle, but not concurrently with ovphysx_destroy_instance on the
     * same handle.
     *
     * @param handle            Instance handle.
     * @param pattern           USD-style path glob selecting SDF-enabled collision
     *                          shapes, including runtime-only clones (e.g. "/World/Mesh*").
     *                          Must match at least one shape or the call returns an error.
     *                          A single path component longer than 4096 characters is
     *                          rejected with OVPHYSX_API_INVALID_ARGUMENT.
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
     *     - shape: [N, Q, 4] with component layout (grad.x, grad.y, grad.z, distance)
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
     *        generated, the get_* calls return the raw geometry, and the application
     *        renders it.
     *
     *        Return convention: when debug rendering is unavailable, most functions
     *        are no-ops returning OVPHYSX_API_SUCCESS.
     *        ovphysx_debug_render_set_scope_tokens returns OVPHYSX_API_ERROR when
     *        scope infrastructure is unavailable. When no USD stage is attached,
     *        the functions return OVPHYSX_API_ERROR. Invalid arguments return
     *        OVPHYSX_API_INVALID_ARGUMENT (documented per function).
     *
     *        Granularity: the debug-render state (enable, parameters, scale,
     *        scope) applies to every physics scene of the attached stage and
     *        is process-global. There is no per-scene control. Use
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
     *         unavailable), or OVPHYSX_API_ERROR when no USD stage is attached.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_enable(ovphysx_handle_t handle, bool enable);

    /**
     * @brief Set one debug-geometry parameter.
     *
     * 0 disables the geometry type and a positive value enables it. For
     * parameters whose PhysX semantics define a magnitude, that value scales the
     * geometry together with the master scale. CONTACT_POINT and FRICTION_POINT
     * use a positive value only as an enable gate. Their marker size follows the
     * master scale. Geometry with an inherent shape (collision shapes, bounds) is
     * drawn at that shape's dimensions.
     * @param handle  Simulator instance (drives the attached stage).
     * @param param   One of ovphysx_debug_render_parameter_t. NONE (0) and values
     *                >= OVPHYSX_DEBUG_RENDER_PARAM_COUNT are rejected.
     * @param value   Finite value >= 0. 0 disables and a positive value enables.
     * @return OVPHYSX_API_INVALID_ARGUMENT if param is out of range or value is
     *         non-finite or negative, OVPHYSX_API_ERROR when no USD stage is
     *         attached, otherwise OVPHYSX_API_SUCCESS (a no-op SUCCESS when debug
     *         rendering is unavailable).
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_parameter(
        ovphysx_handle_t handle, uint32_t param, float value);

    /**
     * @brief Limit debug rendering to an exact set of interned prim paths.
     *
     * An object is in scope when its interned prim path is one of the handles
     * (exact membership, NO prefix expansion). Build the list through the attached
     * Stage's path_dictionary_instance_t or from a query's prim list. Expanding a
     * hierarchy into its exact object set is the caller's job, not this call's.
     * count 0 clears the scope. Joints follow their attached bodies.
     * The scope applies immediately to currently instantiated runtime objects.
     * Reapply it after runtime topology changes. Out-of-scope objects are skipped
     * at emission.
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
     * @param count   Number of entries. 0 clears the scope.
     * @return OVPHYSX_API_INVALID_ARGUMENT when tokens is NULL with count > 0 or
     *         any token is invalid, or OVPHYSX_API_ERROR when the sidecar symbol,
     *         visualization slot, or attached Stage dictionary is unavailable.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_scope_tokens(
        ovphysx_handle_t handle, const ovx_primpath_t* tokens, uint32_t count);

    /**
     * @brief Read back a debug-render parameter's value.
     *
     * Returns the value last set through ovphysx_debug_render_set_parameter,
     * cached on the OvPhysX side. The debug-render state is process-global.
     * 0 means off and is the default before any set.
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
     * @return OVPHYSX_API_INVALID_ARGUMENT if scale is non-finite or negative,
     *         OVPHYSX_API_ERROR when no USD stage is attached, otherwise OVPHYSX_API_SUCCESS.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_scale(ovphysx_handle_t handle, float scale);

    /**
     * @brief Read back the master debug-render scale last set through OvPhysX (cached,
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
     * @return OVPHYSX_API_INVALID_ARGUMENT if min3/max3 is NULL, non-finite, or min > max,
     *         OVPHYSX_API_ERROR when no USD stage is attached, otherwise OVPHYSX_API_SUCCESS.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_culling_box(
        ovphysx_handle_t handle, const float min3[3], const float max3[3]);

    /**
     * @brief Read the current debug-render point buffer (the debug geometry produced
     *        during the step). OvPhysX has no viewer. The application draws it.
     *
     * **Ownership / lifetime:** the returned pointer aliases an OvPhysX-owned buffer and
     * is invalidated by the next ovphysx_step() OR by any stage / scene change
     * (ovphysx_reset_stage, ovphysx_update_from_ovstage, ovphysx_detach_ovstage,
     * ovphysx_destroy_instance): the underlying PhysX scenes and debug buffers may
     * be recreated. Copy out before any of those. Do not hold the pointer across a
     * step or re-attach. On success-with-no-data *out_points is set to NULL and
     * *out_count to 0.
     *
     * @param handle           Instance handle.
     * @param[out] out_points  Set to the buffer base (NULL when empty). Must be non-NULL.
     * @param[out] out_count   Set to the primitive count (0 when empty). Must be non-NULL.
     * @return OVPHYSX_API_INVALID_ARGUMENT if out_points or out_count is NULL,
     *         OVPHYSX_API_ERROR when no USD stage is attached, otherwise OVPHYSX_API_SUCCESS.
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
