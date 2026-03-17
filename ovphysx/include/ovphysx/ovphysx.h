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
    * Version information: See version.h
    *
    * DLPack Integration:
    *   This API uses DLPack for zero-copy tensor data exchange between the user and the system.
    *   DLPack is a stable, widely-adopted open standard for tensor interoperability.
    *   See dlpack.h for tensor format details.
    * 
    * Return values:
    *   Synchronous functions (blocking calls) return ovphysx_result_t (status, error).
    *   Asynchronous functions return ovphysx_enqueue_result_t (status, error, op_index).
    *   Error strings must be destroyed via ovphysx_destroy_error() to free memory.
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
     * - OVPHYSX_API_GPU_NOT_AVAILABLE if GPU requested but unavailable
     * - OVPHYSX_API_ERROR for other initialization failures
     *
     * @warning The device mode (CPU vs GPU) is locked in by the first call in the process.
     *   Subsequent calls with a different device value will fail with OVPHYSX_API_INVALID_ARGUMENT.
     *   See ovphysx_device_t documentation for details.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_instance(const ovphysx_create_args* create_args, ovphysx_handle_t* out_handle);

    /**
     * @brief Destroy an ovphysx instance and release all associated resources.
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
    /* Global settings (process-wide, affects all instances) */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_settings */
    /** @{ */

    /**
    * @brief Set a global configuration setting at runtime.
    * 
    * IMPORTANT: Settings are PROCESS-GLOBAL. Changes affect all ovphysx instances
    * in the current process. Configure settings before creating instances or
    * loading USD for predictable behavior.
    * 
    * The value type is auto-detected from the string:
    * - "true"/"false" -> bool
    * - Integer string (e.g., "42") -> int
    * - Float string (e.g., "0.5") -> float
    * - Other strings -> string
    * 
    * Common settings:
    *   "/physics/cudaDevice" = "0" - CUDA device ordinal
    * 
    * See docs/developer_guide.md for integration guidance and common settings usage.
    * 
    * @param key Setting path (e.g., "/physics/cudaDevice")
    * @param value Setting value as string
    * @return ovphysx_result_t with status and error info
    *
    * @pre key.ptr != NULL, value.ptr != NULL.
    * @post The process-global setting is updated for all instances.
    *
    * @par Side Effects
    * May affect behavior of all existing and future instances.
    *
    * @par Threading
    * Safe to call from any thread; ensure consistent configuration before creating instances.
    *
    * @par Ownership
    * key/value strings are read during the call; caller retains ownership.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid key/value
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * ovphysx_set_global_setting(OVPHYSX_LITERAL("/physics/cudaDevice"), OVPHYSX_LITERAL("0"));
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_set_global_setting(
        ovphysx_string_t key,
        ovphysx_string_t value
    );

    /**
    * @brief Get a global setting value into a user-provided buffer.
    * 
    * Retrieves the current value of a process-global setting as a string.
    * The user must pre-allocate value_out->ptr with sufficient space and set value_out->length
    * to the buffer capacity on input. On success, value_out->length is updated with the actual
    * string length (excluding null terminator).
    * 
    * Example:
    *   char buffer[256];
    *   ovphysx_string_t value = {buffer, 256};
    *   size_t required;
    *   auto result = ovphysx_get_global_setting(key, &value, &required);
    *   if (result.status == OVPHYSX_API_SUCCESS) {
    *       // value.ptr contains the string, value.length is actual length
    *   }
    * 
    * @param key Setting path (e.g., "/physics/cudaDevice")
    * @param value_out [in/out] String with pre-allocated buffer
    * @param out_required_size [out] If buffer too small, contains required buffer capacity
    * @return ovphysx_result_t with status and error info
    *
    * @pre value_out != NULL, out_required_size != NULL.
    * @pre value_out->ptr points to valid writable memory with capacity in value_out->length.
    * @post On success, value_out contains the setting value and value_out->length is updated.
    *
    * @par Side Effects
    * None.
    *
    * @par Ownership
    * Caller owns the output buffer.
    *
    * @par Errors
    * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
    * - OVPHYSX_API_ERROR for internal failures
    *
    * @code
    * char buffer[256];
    * ovphysx_string_t out = {buffer, sizeof(buffer)};
    * size_t required = 0;
    * ovphysx_get_global_setting(OVPHYSX_LITERAL("/physics/cudaDevice"), &out, &required);
    * @endcode
    */
    OVPHYSX_API ovphysx_result_t ovphysx_get_global_setting(
        ovphysx_string_t key,
        ovphysx_string_t* value_out,
        size_t* out_required_size
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
    * 
    * @return ovphysx_enqueue_result_t with status, error, and operation index for the clone
    * 
    * @note This is an asynchronous operation. Use ovphysx_wait_op(handle, result.op_index, ...)
    *       to wait for completion before using the cloned objects outside of the stream operations.
    *
    * @pre handle must be valid.
    * @pre source_path_in_usd must exist; target_paths must be valid and unique.
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
    * ovphysx_enqueue_result_t r = ovphysx_clone(handle, ovphysx_cstr("/World/env0"), targets, 2);
    * @endcode
    */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_clone(
        ovphysx_handle_t handle,
        ovphysx_string_t source_path_in_usd,
        ovphysx_string_t* target_paths,
        uint32_t num_target_paths
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
    * @param elapsed_time Simulation timestep in seconds
    * @param current_time Current time, might be used for time sampled transformations to apply
    * @return ovphysx_enqueue_result_t with status, error, and operation index
    *
    * @pre handle must be a valid instance handle.
    * @post Simulation advances by elapsed_time when the op completes.
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
                                                              float elapsed_time,
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
     * @post Binding handle is valid until destroyed.
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
     * Read data from simulation into a user-provided DLTensor (synchronous).
     * 
     * GPU MODE WARNING: On the first GPU tensor read after loading USD, an automatic
     * warmup simulation step is performed to initialize PhysX DirectGPU buffers.
     * 
     * IMPORTANT: The warmup step is a real physics step that advances simulation state
     * by a minimal timestep (~1ns). While the effect is negligible, it means:
     *   - Simulation time advances slightly
     *   - Physics state may change infinitesimally
     *   - If you need exact initial state, call ovphysx_warmup_gpu() before reading
     *     initial positions, or call ovphysx_step() explicitly with your desired dt
     * 
     * For explicit control over warmup timing (recommended for deterministic behavior):
     *   1. Load USD, wait for completion
     *   2. Call ovphysx_warmup_gpu() to perform warmup at a controlled time
     *   3. Now tensor reads will return valid data without triggering auto-warmup
     *
     * Note: The DirectGPU warmup requirement means you cannot observe a true "pre-warmup"
     * GPU tensor state. Warmup is a real simulation step; calling it explicitly simply
     * makes the timing of that step predictable.
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
     * All tensor types are writable. Force/wrench types (OVPHYSX_TENSOR_*_FORCE_F32,
     * OVPHYSX_TENSOR_*_WRENCH_F32) are WRITE-ONLY control inputs applied each step.
     * See ovphysx_tensor_type_t documentation for shapes and layouts.
     *
     * GPU MODE WARNING: On the first GPU tensor write after loading USD, an automatic
     * warmup simulation step may be performed to initialize PhysX DirectGPU buffers.
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
    * All error strings returned by this operation must be destroyed by calling ovphysx_destroy_errors.
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
    * Caller owns out_wait_result and must destroy any error strings via ovphysx_destroy_errors().
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

    /* 
    * Destroy resources associated with a single error string returned by any API call.
    * 
    * ALWAYS call this on any non-empty error string returned by the API.
    * This is a global function (not tied to any instance) so errors can be cleaned up
    * even after instance destruction.
    * 
    * The implementation safely handles both allocated and static strings internally.
    * Passing an empty/null string is safe and does nothing.
    * 
    * @param error Error string to destroy
    *
    * @pre None (safe on empty/null strings).
    * @post Any internal memory associated with the error is released.
    *
    * @par Side Effects
    * Frees error string storage if owned by the API.
    *
    * @code
    * if (result.error.ptr) {
    *   ovphysx_destroy_error(result.error);
    * }
    * @endcode
    */
    OVPHYSX_API void ovphysx_destroy_error(ovphysx_string_t error);

    /* 
    * Destroy resources associated with returned errors from ovphysx_wait_op().
    * 
    * This is a global function (not tied to any instance).
    * 
    * @param errors Array of operation errors to destroy
    * @param num_errors Number of operation errors to destroy
    *
    * @pre Safe to call with NULL/0.
    * @post Frees all error string storage in the array.
    *
    * @par Ownership
    * Caller remains responsible for freeing the array itself (if allocated).
    *
    * @code
    * ovphysx_destroy_errors(wait_result.errors, wait_result.num_errors);
    * @endcode
    */
    OVPHYSX_API void ovphysx_destroy_errors(const ovphysx_op_error_t* errors,
                                            size_t num_errors);

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
     *              Must be between OVPHYSX_LOG_NONE and OVPHYSX_LOG_VERBOSE inclusive.
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
     * log level — it only controls the built-in console logger.
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


    /*--------------------------------------------------*/
    /* Deprecated API - for backward compatibility only */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_deprecated */
    /** @{ */

    /**
     * @deprecated Use ovphysx_set_global_setting() instead.
     * Settings are now process-global; handle parameter is ignored.
     *
     * @param handle Ignored.
     * @param key Setting path.
     * @param value Setting value.
     * @return ovphysx_result_t with status and error info.
     *
     * @pre key.ptr != NULL, value.ptr != NULL.
     * @post Global setting updated on success.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED ovphysx_result_t ovphysx_set_setting(
        ovphysx_handle_t handle,
        ovphysx_string_t key,
        ovphysx_string_t value);

    /**
     * @deprecated Use ovphysx_get_global_setting() instead.
     * Settings are now process-global; handle parameter is ignored.
     *
     * @param handle Ignored.
     * @param key Setting path.
     * @param value_out Output string buffer.
     * @param out_required_size Required buffer size if too small.
     * @return ovphysx_result_t with status and error info.
     *
     * @pre value_out and out_required_size must be valid.
     * @post value_out contains setting value on success.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED ovphysx_result_t ovphysx_get_setting(
        ovphysx_handle_t handle,
        ovphysx_string_t key,
        ovphysx_string_t* value_out,
        size_t* out_required_size);

    /** @} */

#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_OVPHYSX_H
