// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


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
    *   This API uses DLPack for zero-copy tensor data exchange between the user and the system.
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
    *   - Each op_index is SINGLE-USE: after waiting (via ovphysx_wait_op),
    *     the op_index is consumed and cannot be reused. Concurrent waits on the same op_index from
    *     multiple threads have undefined behavior.
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
     *   - Fully thread-safe. Different instances can be used from different threads
     *     concurrently without any synchronization.
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
     * ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
     * ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
     * ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
     * @endcode
     *
     * @param create_args Configuration for the ovphysx instance (must not be NULL).
     * @param out_handle [out] ovphysx handle (must not be NULL).
     * @return ovphysx_result_t with status and error info.
     *
     * @pre create_args != NULL, out_handle != NULL.
     * @post On success, *out_handle is a valid handle that must be destroyed with ovphysx_destroy_instance().
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
     * - OVPHYSX_API_INVALID_ARGUMENT on null pointers
     * - OVPHYSX_API_GPU_NOT_AVAILABLE if GPU is required (OVPHYSX_DEVICE_GPU) but unavailable
     * - OVPHYSX_API_ERROR for other initialization failures
     *
     * @warning The device mode (CPU vs GPU) is locked in by the first call in the process.
     *   OVPHYSX_DEVICE_AUTO resolves to an effective mode (CPU or GPU) on first use.
     *   Subsequent calls with a different effective mode will fail with OVPHYSX_API_INVALID_ARGUMENT.
     *   See ovphysx_device_t documentation for details.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_instance(const ovphysx_create_args* create_args, ovphysx_handle_t* out_handle);

    /**
     * @brief Destroy an ovphysx instance and release all associated resources.
     *
     * When the last instance is destroyed, performs full framework teardown:
     * stops background threads, unloads all plugins in dependency order, and
     * releases the framework.  This prevents crashes from non-deterministic
     * DLL/SO unload at process exit.
     *
     * @param handle ovphysx handle to destroy.
     * @return ovphysx_result_t with status and error info.
     *
     * @pre handle must be a valid instance handle.
     * @post The handle is invalid and must not be reused.
     *
     * @par Side Effects
     * Releases internal resources, plugins, and cached data for this instance.
     * On last instance, tears down the Carbonite framework if ovphysx owns it.
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
    * @brief Begin global shutdown for all instances of ovphysx.
    *
    * Call this before triggering any bulk unload/destroy operations during process exit so the
    * SDK can enter shutdown mode and skip redundant teardown work safely.
    * @return ovphysx_result_t with status. Never returns an error message.
    *
    * @pre None.
    * @post Global shutdown mode is enabled.
    *
    * @par Side Effects
    * Alters global shutdown behavior for all instances in the process.
    *
    * @par Threading
    * Safe to call from any thread.
    *
    * @par Errors
    * - OVPHYSX_API_ERROR for unexpected shutdown failures (no error string).
    *
    * @code
    * ovphysx_set_shutting_down();
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_set_shutting_down(void);

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
     *     OVPHYSX_LITERAL("/physics/fabricUpdateVelocities"), OVPHYSX_LITERAL("true")));
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
    /* Remote storage credentials (process-wide)        */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_settings */
    /** @{ */

    /**
     * @brief Configure S3 credentials for remote USD loading via HTTPS S3 URLs.
     *
     * Credentials are process-global and take effect immediately for all
     * ovphysx instances. Call before ovphysx_add_usd() with an S3 HTTPS URL.
     *
     * Requires that an ovphysx instance has been created (the remote-loading
     * runtime is initialized automatically during instance creation).
     *
     * @param host              S3 endpoint host (e.g., "my-bucket.s3.us-east-1.amazonaws.com"). Required.
     * @param bucket            S3 bucket name. Required.
     * @param region            AWS region (e.g., "us-east-1"). Required.
     * @param access_key_id     AWS access key ID. Required.
     * @param secret_access_key AWS secret access key. Required.
     * @param session_token     STS session token. Optional (NULL if not using temporary credentials).
     * @return ovphysx_result_t with status and error info.
     *
     * @pre An ovphysx instance must have been created (so the runtime is loaded).
     * @post S3 credentials are configured for the process.
     *
     * @par Side Effects
     * Configures S3 settings for the entire process.
     *
     * @par Threading
     * Thread-safe (serialized internally).
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if required parameters are NULL or empty.
     * - OVPHYSX_API_ERROR if the remote-loading runtime is unavailable or the call fails.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_configure_s3(
        const char* host,
        const char* bucket,
        const char* region,
        const char* access_key_id,
        const char* secret_access_key,
        const char* session_token
    );

    /**
     * @brief Configure an Azure SAS token for remote USD loading via Azure Blob Storage.
     *
     * Credentials are process-global and take effect immediately for all
     * ovphysx instances. Call before ovphysx_add_usd() with an Azure Blob URL.
     *
     * Requires that an ovphysx instance has been created (the remote-loading
     * runtime is initialized automatically during instance creation).
     *
     * @param host      Azure Blob host (e.g., "myaccount.blob.core.windows.net"). Required.
     * @param container Azure container name. Required.
     * @param sas_token SAS token string (without leading '?'). Required.
     * @return ovphysx_result_t with status and error info.
     *
     * @pre An ovphysx instance must have been created (so the runtime is loaded).
     * @post Azure SAS token is configured for the process.
     *
     * @par Side Effects
     * Configures Azure settings for the entire process.
     *
     * @par Threading
     * Thread-safe (serialized internally).
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if required parameters are NULL or empty.
     * - OVPHYSX_API_ERROR if the remote-loading runtime is unavailable or the call fails.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_configure_azure_sas(
        const char* host,
        const char* container,
        const char* sas_token
    );

    /** @} */

    /*--------------------------------------------------*/
    /* Stage building and introspection operations      */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_stage */
    /** @{ */

    /*
    * Enqueue an asynchronous operation to add a USD file to the runtime stage representation.
    * @param handle ovphysx instance
    * @param usd_file_path Path to the USD file to add
    * @param path_prefix Prefix path that will be added to all paths in the USD file.
    *                    This is useful when adding multiple USD files, so that the
    *                    prims in each USD file have unique, known paths.
    *                    If a combined prefix path and prim path inside the loaded USD file already exists,
    *                    an error will be returned and the USD file will not be added.
    *                    Can be empty string for no prefix.
    * @param out_usd_handle [out] Handle to the added USD file to be used with remove_usd
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    * @note We currently only support one USD file per instance, and no path_prefix is supported.
    *
    * @pre handle must be a valid instance handle.
    * @pre usd_file_path must point to an existing USD file.
    * @post On success, the USD is enqueued for load; op_index can be waited on.
    *
    * @par Side Effects
    * Mutates the runtime stage and allocates simulation resources.
    *
    * @par Threading
    * Must not be called concurrently on the same instance without external synchronization.
    *
    * @par Ownership
    * The string arguments are read during the call; caller retains ownership.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_NOT_IMPLEMENTED for unsupported features
    * - OVPHYSX_API_ERROR for load failures
    *
    * @code
    * ovphysx_usd_handle_t usd = 0;
    * ovphysx_enqueue_result_t r = ovphysx_add_usd(handle, ovphysx_cstr("scene.usda"),
    *                                              ovphysx_cstr(""), &usd);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_add_usd(ovphysx_handle_t handle,
                                                                 ovphysx_string_t usd_file_path,
                                                                 ovphysx_string_t path_prefix,
                                                                 ovphysx_usd_handle_t* out_usd_handle);

    /*
    * Enqueue an asynchronous operation to remove a prior added USD file from the runtime stage representation.
    * All prims added to the stage during ovphysx_add_usd() will be removed from the stage.
    * @param handle ovphysx instance
    * @param usd_handle Handle obtained from add_usd to identify the USD file to remove
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    *
    * @pre handle and usd_handle must be valid.
    * @post USD data is removed from the runtime stage when the op completes.
    *
    * @par Side Effects
    * Releases stage data and simulation resources for the USD subtree.
    *
    * @par Ownership
    * usd_handle becomes invalid after removal completes.
    *
    * @par Errors
    * - OVPHYSX_API_NOT_FOUND if usd_handle is unknown
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_enqueue_result_t r = ovphysx_remove_usd(handle, usd_handle);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_remove_usd(ovphysx_handle_t handle,
                                                                    ovphysx_usd_handle_t usd_handle);

    /*
    * Enqueue an asynchronous operation to reset the runtime stage representation to an empty stage.
    * All prims added to the stage during ovphysx_add_usd() will be removed from the stage, corresponding handles to these added USD files will be invalidated.
    * Settings and other configuration not present in the stage will persist.
    * @param handle ovphysx instance
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    *
    * @pre handle must be a valid instance handle.
    * @post Stage is cleared when the op completes; all USD handles are invalidated.
    *
    * @par Side Effects
    * Clears the runtime stage and associated simulation state.
    *
    * @par Errors
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_enqueue_result_t r = ovphysx_reset(handle);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_reset(ovphysx_handle_t handle);

    /**
    * @brief Get the USD stage ID for the currently loaded stage.
    * 
    * Returns the internal USD stage identifier for the stage attached to this ovphysx instance.
    * This can be used to correlate PhysX instances with USD stages in multi-stage scenarios.
    *
    * @note This is a synchronous operation. Use ovphysx_wait_op(handle, result.op_index, ...)
    *       to wait in case add_usd / remove_usd / reset are enqueued before this operation.
    * 
    * @param handle The ovphysx instance
    * @param out_stage_id Pointer to receive the stage ID (must not be NULL)
    * @return ovphysx_result_t with status and error info
    *
    * @pre handle and out_stage_id must be valid.
    * @post out_stage_id contains the current stage ID on success.
    *
    * @par Side Effects
    * None.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for null out_stage_id
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * int64_t stage_id = 0;
    * ovphysx_get_stage_id(handle, &stage_id);
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_get_stage_id(ovphysx_handle_t handle, int64_t* out_stage_id);

    /**
    * @brief Enqueue an asynchronous operation to clone the subtree under the source path to 
    * one or more target paths in the runtime stage representation (internal representation only, USD untouched).
    * The source path must exist in the stage.
    * The target paths must not already exist in the stage.
    * 
    * Clones are created in the internal representation only (USD file will not be modified)
    * and immediately participate in physics simulation.
    * This is optimized for RL training scenarios with mass replication (1000s of instances).
    * 
    * @note Collision isolation between clones (e.g., preventing clones in env1 from colliding with
    *       clones in env2) is configured through USD scene properties (collision groups, filtering)
    *       rather than through clone API parameters. Set up collision filtering in your USD scene
    *       before cloning if you need isolated parallel environments.
    * 
    * @param handle PhysX instance handle
    * @param source_path_in_usd Path to the source subtree to clone (must exist)
    * @param target_paths Array of target paths to clone to (must not exist)
    * @param num_target_paths Number of target paths to clone to
    * @param parent_transforms  World-space placement for each cloned
    *        environment's parent Xform prim.  Flat array of
    *        [num_target_paths x 7] floats: (px, py, pz, qx, qy, qz, qw) per
    *        target -- position followed by quaternion rotation.  Quaternion
    *        convention: imaginary-first, matching the tensor binding pose
    *        format (OVPHYSX_TENSOR_RIGID_BODY_POSE_F32).
    *        Identity rotation = (0, 0, 0, 1).
    *        Pass NULL to place all parent prims at identity (use it when you don't care about
    *        spatial separation).
    * 
    * @return ovphysx_enqueue_result_t with status, error, and operation index for the clone
    * 
    * @note This is an asynchronous operation. Use ovphysx_wait_op(handle, result.op_index, ...)
    *       to wait for completion before using the cloned objects outside of the stream operations.
    *
    * @pre handle must be valid.
    * @pre source_path_in_usd must exist; target_paths must be valid and unique.
    * @pre Must be called **before** warmup_gpu() or the first simulate() call.
    *      Cloning after GPU warmup may reallocate GPU buffers and corrupt
    *      already-initialised state.  A warning is logged if this order is violated.
    * @post Clone is visible in the runtime stage once the op completes.
    *
    * @par Side Effects
    * Adds new runtime-stage prims for each target path.
    *
    * @par Ownership
    * The target_paths array is read during the call; caller retains ownership.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_NOT_FOUND if source path does not exist
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_string_t targets[2] = { ovphysx_cstr("/World/env1"), ovphysx_cstr("/World/env2") };
    * ovphysx_enqueue_result_t r = ovphysx_clone(handle, ovphysx_cstr("/World/env0"), targets, 2, NULL);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_clone(
        ovphysx_handle_t handle,
        ovphysx_string_t source_path_in_usd,
        ovphysx_string_t* target_paths,
        uint32_t num_target_paths,
        const float* parent_transforms
    );

    /** @} */

    /*--------------------------------------------------*/
    /* Simulation operations */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_simulation */
    /** @{ */

    /*
    * Enqueue an asynchronous physics simulation step.
    * @param handle ovphysx instance
    * @param step_dt Simulation timestep in seconds
    * @param current_time Current time, might be used for time sampled transformations to apply
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
    * ovphysx_enqueue_result_t r = ovphysx_step(handle, 1.0f / 60.0f, 0.0f);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_step(ovphysx_handle_t handle,
                                                              float step_dt,
                                                              float current_time);

    /**
     * @brief Synchronous step: simulate one physics timestep and wait for
     * completion in a single call.
     *
     * Functionally equivalent to ovphysx_step() followed by
     * ovphysx_wait_all_pending_ops(), but bypasses the async event machinery
     * entirely (mutex acquisitions, operation map insert/lookup/cleanup).
     * This is a measurable performance improvement for the common synchronous
     * use case: in IsaacLab RL training at 4096 environments, step_sync saves
     * ~0.2 ms per substep compared to step() + wait_op(), recovering roughly
     * 5-6% of total throughput.
     *
     * Use this whenever you step and immediately wait for results (i.e. you do
     * not overlap GPU simulation with CPU work between dispatch and fetch).
     *
     * @param handle  Physics instance handle.
     * @param step_dt  Timestep [s].
     * @param current_time  Current simulation time [s].
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_handle_t handle,
                                                            float step_dt,
                                                            float current_time);

    /**
     * Run n_steps consecutive physics steps in a single C call.
     * Step i is executed with duration step_dt at simulation time
     * current_time + i * step_dt.  This saves (n_steps-1) ctypes
     * round-trips for workloads that use decimation (one RL step =
     * multiple physics steps).
     *
     * Equivalent to calling ovphysx_step_sync(handle, step_dt, current_time + i * step_dt)
     * for i in [0, n_steps).
     *
     * @param handle       Physics instance handle.
     * @param n_steps      Number of steps to run (must be > 0).
     * @param step_dt      Duration of each step [s].
     * @param current_time Simulation time at the start of the first step [s].
     * @return ovphysx_result_t with OVPHYSX_API_SUCCESS on success.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_step_n_sync(ovphysx_handle_t handle,
                                                              int32_t n_steps,
                                                              float step_dt,
                                                              float current_time);

    /** @} */


    /*--------------------------------------------------*/
    /* Tensor Binding API                                */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /*
     * The Tensor Binding API provides efficient bulk access to physics simulation data.
     * It maps USD prim patterns to typed tensor views, enabling zero-copy data exchange
     * with tensors (PyTorch, NumPy, etc.) via DLPack.
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
     *   - Supported devices: kDLCPU (CPU mode) and kDLCUDA (GPU mode).
     *
     * Works in both GPU mode (device=OVPHYSX_DEVICE_GPU) and CPU mode (device=OVPHYSX_DEVICE_CPU).
     * Pass DLTensors with matching device type (kDLCUDA for GPU, kDLCPU for CPU).
     * 
     * GPU MODE NOTE: GPU tensor reads require at least one simulation step() after loading USD.
     * This is a PhysX DirectGPU API requirement. Without a warmup step, reads may fail
     * or return stale data. CPU mode does not have this requirement.
     * 
     * Typical workflow (GPU mode):
     *   1. ovphysx_create_instance() with device=OVPHYSX_DEVICE_GPU (default)
     *   2. ovphysx_add_usd(), then ovphysx_wait_op() for completion
     *   3. ovphysx_create_tensor_binding() for each tensor type needed
     *   4. Optional: ovphysx_warmup_gpu() to control when GPU warmup happens
     *      (otherwise it will happen automatically on first tensor read)
     *   5. Main loop: read/write tensors, step, wait
     * 
     * Pattern matching:
     *   Patterns use USD prim path glob syntax:
     *   - Exact path: "/World/robot" - matches single prim
     *   - Wildcard:   "/World/robot*" - matches robot1, robot2, robotArm, etc.
     *   - Nested:     "/World/env[N]/robot" with [N] as wildcard - matches /World/env0/robot, etc.
     */

    /**
     * Create a tensor binding for bulk data access (synchronous).
     * 
     * A tensor binding connects a USD prim pattern (e.g., "/World/robot*") to a
     * tensor type (e.g., OVPHYSX_TENSOR_RIGID_BODY_POSE_F32), enabling efficient
     * bulk read/write of physics data for all matching prims.
     * 
     * If the pattern matches zero prims, the binding is still created successfully
     * with element_count = 0. This allows the scene to be populated later.
     * 
     * Example:
     *   ovphysx_tensor_binding_desc_t desc = {
     *       .pattern = OVPHYSX_LITERAL("/World/robot*"),  // compile-time string literal
     *       .tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32
     *   };
     *   ovphysx_create_tensor_binding(handle, &desc, &binding);
     * 
     * @param handle Instance handle
     * @param desc Binding descriptor with pattern and tensor_type
     * @param out_binding_handle [out] Binding handle on success
     * @return ovphysx_result_t (synchronous - completes before returning)
     *
     * @pre handle, desc, and out_binding_handle must be valid.
     * @post Binding handle is valid until explicitly destroyed via ovphysx_destroy_tensor_binding(),
     *       or until the parent instance is destroyed.
     *
     * @par Side Effects
     * Allocates internal binding resources.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * ovphysx_tensor_binding_handle_t binding = 0;
     * ovphysx_create_tensor_binding(handle, &desc, &binding);
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
     * See ovphysx_tensor_type_t documentation for shapes and layouts per tensor type.
     * Layout is always row-major contiguous (C-order). dtype is always float32.
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
     *   - dtype must be float32 (kDLFloat, 32 bits, 1 lane)
     *   - device must match the instance (CPU: kDLCPU, GPU: kDLCUDA)
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
     * @pre dst_tensor must be pre-allocated and match spec (dtype/shape/device).
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
     * Note: Warmup state is reset when the stage changes (e.g., after reset() or
     * loading a new USD file). The next tensor read will trigger warmup again.
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
     * - ARTICULATION_LINK_POSE_F32, ARTICULATION_LINK_VELOCITY_F32, ARTICULATION_LINK_ACCELERATION_F32
     *   are READ-ONLY (no setter for individual link state).
     * - Dynamics query tensors (JACOBIAN, MASS_MATRIX, CORIOLIS_AND_CENTRIFUGAL_FORCE, GRAVITY_FORCE,
     *   LINK_INCOMING_JOINT_FORCE, DOF_PROJECTED_JOINT_FORCE, BODY_INV_MASS, BODY_INV_INERTIA)
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
     * @pre src_tensor must match dtype/shape/device.
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
     *   ovphysx_get_tensor_binding_spec(). Must be float32 on matching device.
     * @param mask_tensor Binary mask selecting which elements to update. Must be 1D with shape [N]
     *   where N matches the binding's first dimension. Dtype must be bool (kDLBool, bits=8) or
     *   uint8 (kDLUInt, bits=8). Device must match the binding.
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre src_tensor must match dtype/shape/device of the binding spec.
     * @pre mask_tensor must be 1D uint8/bool with length N on matching device.
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
     * This is a fundamental constraint of the TensorAPI -- tensor shapes are fixed at
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
     * For **per-contact-point geometry** (position, normal, impulse) instead
     * of aggregate forces, see ovphysx_get_contact_report().
     *
     * A **sensor** is a set of rigid body prims identified by a USD prim path
     * pattern that you pass to ovphysx_create_contact_binding().  A **filter**
     * is a second set of bodies whose contacts with each sensor you want to
     * measure.  Both are specified as USD prim path patterns; no extra USD
     * authoring or physics schema is required beyond the rigid bodies themselves.
     *
     * **Lifecycle and read timing**
     *
     * A contact binding must be created *before* the first simulation step whose
     * contacts you want to observe.  After creation it registers sensors inside
     * the PhysX contact-report callback; no contact data exists until at least
     * one `ovphysx_step()` has completed.
     *
     * - Call `ovphysx_read_contact_net_forces()` or
     *   `ovphysx_read_contact_force_matrix()` *after* `ovphysx_step()`.
     * - Both functions reflect the contacts accumulated during the **last
     *   completed simulation step**.
     * - Calling either read function before any simulation step returns an
     *   all-zeros tensor (no contacts have been reported yet).
     * - The `dt` for impulse-to-force conversion (`force = impulse / dt`) is
     *   automatically taken from the last `ovphysx_step()` call; you do not
     *   need to pass it explicitly.
     *
     * **Why a separate binding type (not tensor binding)**
     *
     * Contact data has a fundamentally different shape and semantics from
     * articulation/rigid-body tensors:
     * - Shape is `[S, F, 3]` (sensor × filter × xyz), determined at binding
     *   creation time, not by a simple prim count.
     * - Contact binding is read-only; there is no write path.
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
     * @param sensor_patterns Array of USD prim path patterns matching sensor bodies
     * @param sensor_patterns_count Number of sensor patterns
     * @param filter_patterns Flat array of filter prim path patterns. All sensors must
     *   have the same number of filters. Total length = sensor_patterns_count * filters_per_sensor.
     *   Pass NULL with filters_per_sensor=0 for unfiltered contacts.
     * @param filters_per_sensor Number of filter patterns per sensor (same for all sensors)
     * @param max_contact_data_count Max contact pairs to track (for raw contact queries)
     * @param out_handle [out] Contact binding handle
     * @return ovphysx_result_t
     *
     * @post Binding handle is valid until explicitly destroyed via ovphysx_destroy_contact_binding(),
     *       or until the parent instance is destroyed.
     *
     * @code
     * // Track contacts on the robot end-effector against the box obstacle.
     * ovphysx_string_t sensors[]  = { ovphysx_cstr("/World/robot_0/ee") };
     * ovphysx_string_t filters[]  = { ovphysx_cstr("/World/obstacles/box") };
     * ovphysx_contact_binding_handle_t cb;
     * ovphysx_create_contact_binding(
     *     handle,
     *     sensors, 1,          // 1 sensor pattern
     *     filters, 1,          // 1 filter pattern per sensor
     *     256,                 // max raw contact pairs
     *     &cb);
     * // After ovphysx_step():
     * // - net forces  tensor shape: [S, 3]   (S = matched sensor count)
     * // - force matrix tensor shape: [S, F, 3] (F = matched filter count per sensor)
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
     * @brief Read net contact forces. dst shape: [S, 3] where S = sensor_count.
     *
     * The dt for impulse-to-force conversion is taken automatically from the
     * last ovphysx_step() call.
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
     * last ovphysx_step() call.
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
     *   - ovphysx_reset()
     *   - ovphysx_remove_usd()
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
     * This mirrors the internal `IPhysx::getPhysXPtr(path, PhysXType)` API
     * that has been used successfully in Kit-based workflows for years. A
     * single function covers all PhysX object types -- no per-type variants
     * are needed.
     *
     * @param handle   ovphysx instance handle.
     * @param prim_path Null-terminated USD prim path (e.g. "/World/physicsScene",
     *        "/World/Cube", "/World/articulation").
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
     * ovphysx_remove_usd(), ovphysx_reset(), or ovphysx_destroy(). Calls to
     * ovphysx_step() and ovphysx_clone() do NOT invalidate existing pointers.
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
     *     handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE, &scene);
     * if (r.status == OVPHYSX_API_SUCCESS && scene) {
     *     // Cast: physx::PxScene* pxScene = static_cast<physx::PxScene*>(scene);
     * }
     * @endcode
     */
    OVPHYSX_API ovphysx_result_t ovphysx_get_physx_ptr(
        ovphysx_handle_t handle,
        const char* prim_path,
        ovphysx_physx_type_t physx_type,
        void** out_ptr);

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
     * uint64-encoded SdfPaths matching the Omni PhysX representation. Use
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
    *   Each op_index is single-use. After a successful wait (status == OVPHYSX_API_SUCCESS), the op_index
    *   is consumed and its resources are released. Attempting to wait on the same op_index again will
    *   return OVPHYSX_API_NOT_FOUND. This prevents resource leaks and encourages correct usage patterns.
    *
    * SPECIAL VALUES:
    *   - OVPHYSX_OP_INDEX_ALL: Wait for all pending operations (useful for shutdown/sync points).
    *
    * THREAD SAFETY:
    *   - Waiting on a specific op_index from multiple threads concurrently is UNDEFINED BEHAVIOR.
    *   - Each op_index should be waited on by exactly one thread.
    *   - Waiting on OVPHYSX_OP_INDEX_ALL is thread-safe if no other thread is using specific op indices.
    *
    * @param handle ovphysx handle
    * @param op_index Operation index to wait for, or OVPHYSX_OP_INDEX_ALL to wait for all operations submitted up to this point
    * @param timeout_ns Timeout in nanoseconds (0 for non-blocking poll)
    * @param out_wait_result [out] Wait result information (errors and active operation indices)
    * @return ovphysx_result_t with status:
    *         OVPHYSX_API_SUCCESS if the operations were waited for successfully,
    *         OVPHYSX_API_ERROR if the wait failed (e.g., invalid handle or internal error),
    *         OVPHYSX_API_NOT_FOUND if the op_index was already consumed or never existed,
    *         OVPHYSX_API_TIMEOUT if not all operations completed within the timeout.
    *
    * @pre handle and out_wait_result must be valid.
    * @post On success, all operations up to op_index have completed.
    *
    * @par Side Effects
    * Consumes op_index on successful wait.
    *
    * @par Ownership
    * Caller owns out_wait_result and must free it via ovphysx_destroy_wait_result().
    * For each failed op index, call ovphysx_get_last_op_error(op_index) to get the error string.
    *
    * @par Errors
    * - OVPHYSX_API_NOT_FOUND if op_index is invalid or already consumed
    * - OVPHYSX_API_TIMEOUT if timeout elapses
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_op_wait_result_t wait_result = {0};
    * ovphysx_wait_op(handle, op_index, UINT64_MAX, &wait_result);
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


#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_OVPHYSX_H
