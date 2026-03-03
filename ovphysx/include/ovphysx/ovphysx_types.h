// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef OVPHYSX_TYPES_H
#define OVPHYSX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ovphysx/dlpack/dlpack.h"

/**
 * String with pointer and length.
 * 
 * Strings are NOT guaranteed to be null-terminated. Always use the length field.
 * Use OVPHYSX_LITERAL("literal") or ovphysx_cstr() for convenient construction.
 */
typedef struct ovphysx_string_t
{
    const char* ptr;   /**< Pointer to string data (not guaranteed null-terminated) */
    size_t length;     /**< Length in bytes */
} ovphysx_string_t;

/**
 * Create ovphysx_string_t from a string LITERAL only.
 * For runtime strings (variables, user input), use ovphysx_cstr() instead.
 */
#ifdef __cplusplus
#define OVPHYSX_LITERAL(s) ovphysx_string_t{ (s), sizeof(s) - 1 }
#else
#define OVPHYSX_LITERAL(s) ((ovphysx_string_t){ (s), sizeof(s) - 1 })
#endif

/**
 * Helper function to create ovphysx_string_t from a null-terminated C string.
 * Returns empty string if cstr is NULL.
 */
static inline ovphysx_string_t ovphysx_cstr(const char* cstr)
{
    ovphysx_string_t s;
    if (cstr) {
        s.ptr = cstr;
        s.length = 0;
        while (cstr[s.length]) s.length++;
    } else {
        s.ptr = "";
        s.length = 0;
    }
    return s;
}

#ifdef __cplusplus
extern "C"
{
#endif

    /*--------------------------------------------------*/
    /* Opaque handle types */
    /*--------------------------------------------------*/

    typedef uint64_t ovphysx_handle_t;
    typedef uint64_t ovphysx_usd_handle_t;
    typedef uint64_t ovphysx_attribute_binding_handle_t;
    typedef uint64_t ovphysx_write_map_handle_t;
    typedef uint64_t ovphysx_read_map_handle_t;
    typedef uint64_t ovphysx_op_index_t;
    typedef uint64_t ovphysx_tensor_binding_handle_t;

    /**
     * Sentinel value representing an invalid/null ovphysx instance handle.
     * Valid handles start at 1, so 0 is reserved to indicate "no handle" or "invalid handle".
     * Use this when a handle parameter is required but no valid instance is available.
     */
    #define OVPHYSX_INVALID_HANDLE 0

    /**
     * Sentinel value for ovphysx_wait_op to wait for all operations submitted up to the call.
     * Use this when you want to ensure all outstanding operations have completed.
     */
    #define OVPHYSX_OP_INDEX_ALL UINT64_MAX

    /*--------------------------------------------------*/
    /* Log level enum */
    /*--------------------------------------------------*/

    typedef enum
    {
        OVPHYSX_LOG_NONE = 0,    /**< No logging */
        OVPHYSX_LOG_ERROR = 1,   /**< Error messages only */
        OVPHYSX_LOG_WARNING = 2, /**< Warnings and errors (default) */
        OVPHYSX_LOG_INFO = 3,    /**< Info, warnings, and errors */
        OVPHYSX_LOG_VERBOSE = 4  /**< All messages including verbose/debug (maps to Carbonite kLevelVerbose) */
    } ovphysx_log_level_t;

    /**
     * Log callback function type.
     *
     * Called for each message that passes the global log level threshold.
     * The caller must ensure the function pointer remains valid until
     * ovphysx_unregister_log_callback() is called.
     *
     * @param level   The ovphysx_log_level_t severity of the message.
     * @param message The log message string (UTF-8, null-terminated). Only valid
     *                for the duration of the callback; copy it if you need to
     *                retain it after returning.
     * @param user_data Opaque pointer passed during registration.
     */
    typedef void (*ovphysx_log_fn)(uint32_t level, const char* message, void* user_data);

    /*--------------------------------------------------*/
    /* Result and status types */
    /*--------------------------------------------------*/

    typedef enum
    {
        OVPHYSX_API_SUCCESS = 0,             /**< Operation completed or enqueued successfully */
        OVPHYSX_API_ERROR = 1,               /**< Operation failed - check error field */
        OVPHYSX_API_TIMEOUT = 2,             /**< Operation timed out */
        OVPHYSX_API_NOT_IMPLEMENTED = 3,     /**< Feature not yet implemented */
        OVPHYSX_API_INVALID_ARGUMENT = 4,    /**< Invalid argument provided */
        OVPHYSX_API_NOT_FOUND = 5,           /**< Requested resource not found (handle unknown, binding invalidated) */
        OVPHYSX_API_DEVICE_MISMATCH = 7,     /**< Tensor device doesn't match binding's expected device */
        OVPHYSX_API_GPU_NOT_AVAILABLE = 8,   /**< GPU requested but not available or CUDA init failed */
    } ovphysx_api_status_t;

    /**
     * Result returned by synchronous API functions.
     */
    typedef struct
    {
        ovphysx_api_status_t status; /**< Operation status code */
        ovphysx_string_t error;          /**< Error message (empty if success, destroy with ovphysx_destroy_error) */
    } ovphysx_result_t;

    /**
     * Result returned by asynchronous API functions.
     */
    typedef struct
    {
        ovphysx_api_status_t status; /**< Operation status code */
        ovphysx_string_t error;          /**< Error message (empty if success, destroy with ovphysx_destroy_error) */
        ovphysx_op_index_t op_index; /**< Operation index for async tracking */
    } ovphysx_enqueue_result_t;

    /*--------------------------------------------------*/
    /* Operation tracking types */
    /*--------------------------------------------------*/

    /**
     * Error associated with a specific operation.
     */
    typedef struct
    {
        ovphysx_op_index_t op_index; /**< Operation that failed */
        ovphysx_string_t error;           /**< Error message (destroy via ovphysx_destroy_errors) */
    } ovphysx_op_error_t;

    /**
     * Result from ovphysx_wait_op() containing errors and pending operation status.
     */
    typedef struct
    {
        ovphysx_op_error_t* errors;                   /**< Array of errors (destroy via ovphysx_destroy_errors) */
        size_t num_errors;                            /**< Number of errors in array */
        ovphysx_op_index_t lowest_pending_op_index;   /**< Lowest operation index still pending, 0 if all complete */
    } ovphysx_op_wait_result_t;

    /*--------------------------------------------------*/
    /* Tensor Binding API types                         */
    /*--------------------------------------------------*/

    /**
     * Tensor type identifiers for bulk GPU data access.
     * 
     * Each value specifies what physical quantity the tensor represents,
     * its shape, data type, and coordinate frame.
     * 
     * Coordinate conventions:
     *   - All poses and velocities are in WORLD FRAME
     *   - DOF data (positions, velocities, targets) are in JOINT SPACE
     *   - Quaternions use [qx, qy, qz, qw] ordering (xyzw)
     * 
     * ============================================================================
     * RIGID BODY TENSORS
     * ============================================================================
     * 
     *   OVPHYSX_TENSOR_RIGID_BODY_POSE_F32
     *     Shape: [N, 7] where N = number of rigid bodies
     *     Layout: [px, py, pz, qx, qy, qz, qw] (position xyz, quaternion xyzw)
     *     Frame: World
     *     DType: float32
     * 
     *   OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32
     *     Shape: [N, 6] where N = number of rigid bodies
     *     Layout: [vx, vy, vz, wx, wy, wz] (linear xyz, angular xyz)
     *     Frame: World
     *     DType: float32
     * 
     * ============================================================================
     * ARTICULATION ROOT TENSORS
     * ============================================================================
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32
     *     Shape: [N, 7] where N = number of articulations
     *     Layout: [px, py, pz, qx, qy, qz, qw]
     *     Frame: World
     *     DType: float32
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32
     *     Shape: [N, 6] where N = number of articulations
     *     Layout: [vx, vy, vz, wx, wy, wz]
     *     Frame: World
     *     DType: float32
     * 
     * ============================================================================
     * ARTICULATION LINK TENSORS (3D - per-link data)
     * ============================================================================
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32
     *     Shape: [N, L, 7] where N = articulations, L = max links
     *     Layout: [px, py, pz, qx, qy, qz, qw] per link
     *     Frame: World
     *     DType: float32
     *     Note: For articulations with fewer than L links, extra entries are zero-padded
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32
     *     Shape: [N, L, 6] where N = articulations, L = max links
     *     Layout: [vx, vy, vz, wx, wy, wz] per link
     *     Frame: World
     *     DType: float32
     * 
     * ============================================================================
     * ARTICULATION DOF TENSORS (joint space)
     * ============================================================================
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32
     *     Shape: [N, D] where N = articulations, D = max DOFs
     *     Layout: Joint positions in articulation DOF order
     *     Units: Radians (revolute) or meters (prismatic)
     *     DType: float32
     *     Note: For articulations with fewer than D DOFs, extra entries are zero-padded
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32
     *     Shape: [N, D]
     *     Layout: Joint velocities
     *     Units: rad/s or m/s
     *     DType: float32
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32
     *     Shape: [N, D]
     *     Layout: Position targets for position-controlled joints
     *     DType: float32
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32
     *     Shape: [N, D]
     *     Layout: Velocity targets for velocity-controlled joints
     *     DType: float32
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32
     *     Shape: [N, D]
     *     Layout: Actuation forces/torques applied to joints.
     *     Units: N or Nm
     *     DType: float32
     *
     *     NOTE: This reads the staging buffer associated with the PhysX GPU joint-force
     *     API (write-only internally). Depending on simulation settings, it may not match
     *     the solver-applied joint forces for the current step.
     */
    typedef enum
    {
        OVPHYSX_TENSOR_INVALID = 0,

        /* Rigid body tensors */
        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32 = 1,          /**< [N, 7] poses in world frame */
        OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32 = 2,      /**< [N, 6] velocities in world frame */

        /* Articulation root tensors */
        OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32 = 10,      /**< [N, 7] root poses */
        OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32 = 11,  /**< [N, 6] root velocities */

        /* Articulation link tensors (3D) */
        OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32 = 20,      /**< [N, L, 7] link poses */
        OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32 = 21,  /**< [N, L, 6] link velocities */

        /* Articulation DOF tensors */
        OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32 = 30,           /**< [N, D] joint positions */
        OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32 = 31,           /**< [N, D] joint velocities */
        OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32 = 32,    /**< [N, D] position targets */
        OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32 = 33,    /**< [N, D] velocity targets */
        OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32 = 34,    /**< [N, D] actuation forces */

        /**
         * External forces/wrenches - WRITE-ONLY (control inputs applied each step).
         * All components are in global (world) frame, including application position.
         * 
         * WRENCH layout: [fx, fy, fz, tx, ty, tz, px, py, pz]
         *   - (fx,fy,fz) = force vector in world frame
         *   - (tx,ty,tz) = torque vector in world frame  
         *   - (px,py,pz) = force application position in world frame
         */
        OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32 = 50,            /**< [N, 3] forces at center of mass */
        OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32 = 51,           /**< [N, 9] row-major: [fx,fy,fz,tx,ty,tz,px,py,pz] per body */
        OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32 = 52,    /**< [N, L, 9] row-major: same layout per link */
    } ovphysx_tensor_type_t;

    /**
     * Descriptor for creating a tensor binding.
     * 
     * A tensor binding connects a list of USD prim paths to a tensor type, enabling bulk
     * read/write of physics data for all matching prims.
     * 
     * Prim selection (mutually exclusive - use ONE of these):
     *   - pattern: Glob pattern like "/World/robot*" or "/World/env[N]/robot"
     *   - prim_paths: Explicit list of exact prim paths
     * 
     * Precedence rules:
     *   1. If prim_paths != NULL AND prim_paths_count > 0, uses explicit paths
     *   2. Else if pattern.ptr != NULL AND pattern.length > 0, uses pattern
     *   3. Else returns OVPHYSX_API_INVALID_ARGUMENT
     * 
     * When prim_paths is used, pattern is completely ignored (not combined).
     * 
     * Example with pattern:
     *   ovphysx_tensor_binding_desc_t desc = {
     *       .pattern = OVPHYSX_LITERAL("/World/robot*"),
     *       .tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32
     *   };
     * 
     * Example with explicit prim paths:
     *   ovphysx_string_t paths[] = {
     *       OVPHYSX_LITERAL("/World/env1/robot"),
     *       OVPHYSX_LITERAL("/World/env4/robot"),
     *       OVPHYSX_LITERAL("/World/env5/robot")
     *   };
     *   ovphysx_tensor_binding_desc_t desc = {
     *       .prim_paths = paths,
     *       .prim_paths_count = 3,
     *       .tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32
     *   };
     */
    typedef struct
    {
        ovphysx_string_t pattern;                /**< USD path glob pattern (ignored if prim_paths is set) */
        const ovphysx_string_t* prim_paths;      /**< Explicit list of exact prim paths (NULL = use pattern) */
        uint32_t prim_paths_count;               /**< Number of prim paths (0 = use pattern) */
        ovphysx_tensor_type_t tensor_type;       /**< Type of tensor data to bind */
    } ovphysx_tensor_binding_desc_t;

    /**
     * Complete tensor specification for DLTensor construction.
     * 
     * Use ovphysx_get_tensor_binding_spec() to get the exact dtype, rank, and shape
     * needed to allocate a compatible tensor. This is the preferred API for
     * constructing DLTensors.
     * 
     * Tensor specifications by type:
     *   - Rigid body pose:     ndim=2, shape=[N, 7]
     *   - Rigid body velocity: ndim=2, shape=[N, 6]
     *   - Articulation root:   ndim=2, shape=[N, 7] or [N, 6]
     *   - Articulation links:  ndim=3, shape=[N, L, 7] or [N, L, 6]
     *   - Articulation DOF:    ndim=2, shape=[N, D]
     * 
     * All tensor types use:
     *   - dtype: float32 (kDLFloat, 32 bits, 1 lane)
     *   - Layout: row-major contiguous (C-order)
     */
    typedef struct
    {
        DLDataType dtype;                    /**< DLPack data type (always float32 currently) */
        int32_t ndim;                        /**< Number of dimensions (2 or 3) */
        int64_t shape[4];                    /**< Shape dimensions [dim0, dim1, dim2, 0] */
    } ovphysx_tensor_spec_t;

    /*--------------------------------------------------*/
    /* Stream synchronization types */
    /*--------------------------------------------------*/

    /**
     * CUDA synchronization for GPU operations.
     * 
     * Controls when the system accesses user memory and when completion is signaled.
     * 
     * Fields:
     *   stream: CUDA stream for the operation
     *     - 0 = use default CUDA stream
     *     - ~0 (all bits set) = unspecified, system chooses
     *     - other = cudaStream_t cast to uintptr_t
     * 
     *   wait_event: CUDA event the system waits on BEFORE accessing user memory
     *     - 0 = no wait (system may access memory immediately during operation execution)
     *     - non-zero = cudaEvent_t cast to uintptr_t
     *     - System waits: cudaStreamWaitEvent(internal_stream, wait_event, 0)
     *     - Use this to ensure your GPU kernels have finished writing to buffers
     * 
     *   signal_event: CUDA event the system records AFTER operation completes
     *     - 0 = no signal
     *     - non-zero = cudaEvent_t cast to uintptr_t
     *     - System records: cudaEventRecord(signal_event, internal_stream)
     *     - Use this to synchronize downstream GPU work with operation completion
     * 
     * See individual function documentation for operation-specific semantics.
     */
    typedef struct
    {
        uintptr_t stream;        /**< CUDA stream: 0=default, ~0=unspecified */
        uintptr_t wait_event;    /**< CUDA event to wait on before accessing user memory: 0=none */
        uintptr_t signal_event;  /**< CUDA event to record after operation completes: 0=none */
    } ovphysx_cuda_sync_t;

    /**
     * User task callback function type.
     * Called in stream order when the task executes.
     * 
     * @param handle Physics handle
     * @param op_index Operation index of this task
     * @param user_data User-provided context data
     * @return Result status (typically OVPHYSX_API_SUCCESS)
     */
    typedef ovphysx_result_t (*ovphysx_user_task_fn)(ovphysx_handle_t handle,
                                                      ovphysx_op_index_t op_index,
                                                      void* user_data);

    /**
     * Description for enqueueing a user task.
     */
    typedef struct
    {
        ovphysx_user_task_fn run; /**< Task callback function */
        void* user_data;          /**< User context (lifetime must be synchronized through events) */
    } ovphysx_user_task_desc_t;

    /*--------------------------------------------------*/
    /* Prim list types */
    /*--------------------------------------------------*/

    /**
     * List of USD prim paths for batch operations.
     */
    typedef struct
    {
        const ovphysx_string_t* prim_paths; /**< Array of USD prim path strings */
        size_t num_paths;               /**< Number of paths in array */
    } ovphysx_prim_list_t;

    /*--------------------------------------------------*/
    /* PhysX configuration types */
    /*--------------------------------------------------*/

    /**
     * Simulation device/backend selection.
     * 
     * Controls whether physics simulation runs on CPU or GPU.
     * 
     * GPU mode (default):
     *   - Enables PhysX GPU acceleration and DirectGPU API
     *   - TensorBinding works with GPU tensors (kDLCUDA) for zero-copy access
     *   - Requires a warmup step() before GPU tensor reads
     * 
     * CPU mode:
     *   - Uses standard PhysX CPU simulation
     *   - TensorBinding works with CPU tensors (kDLCPU)
     *   - No warmup step required
     *
     * @warning All instances within the same process must use the same device mode.
     *   The device setting is applied to process-global Carbonite/PhysX state during
     *   the first ovphysx_create_instance() call and cannot be changed afterwards.
     *   Creating a CPU instance after a GPU instance (or vice versa) in the same
     *   process will fail with OVPHYSX_API_INVALID_ARGUMENT.
     */
    typedef enum
    {
        OVPHYSX_DEVICE_GPU = 0,    /**< GPU simulation with DirectGPU API (default) */
        OVPHYSX_DEVICE_CPU = 1,    /**< CPU-only simulation */
    } ovphysx_device_t;

    /**
     * Configuration for creating a ovphysx instance.
     * Initialize with OVPHYSX_CREATE_ARGS_DEFAULT macro for defaults.
     * Keep OVPHYSX_CREATE_ARGS_DEFAULT in sync with this struct; add new fields
     * only to the end to avoid uninitialized members when using the macro.
     * 
     * Example:
     *   ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
     *   ovphysx_create_instance(&args, &handle);
     *
     * Log level is configured globally via ovphysx_set_log_level(), not per-instance.
     */
    typedef struct ovphysx_create_args
    {
        ovphysx_string_t bundled_deps_path;     /**< Bundled deps path: empty=use runtime discovery (default: empty) */
        
        const ovphysx_string_t* settings_keys;   /**< Array of setting key strings */
        const ovphysx_string_t* settings_values; /**< Array of setting value strings */
        uint32_t settings_count;            /**< Number of settings */
        
        ovphysx_device_t device;            /**< Simulation device (default: OVPHYSX_DEVICE_GPU) */
        int32_t gpu_index;                  /**< GPU device index for CUDA, sets /physics/cudaDevice (default: 0). 
                                                 Only used when device=OVPHYSX_DEVICE_GPU. User settings 
                                                 (settings_keys/values) can override this if needed. */
    } ovphysx_create_args;

    /**
     * Default initializer for ovphysx_create_args.
     * Sets device=GPU, gpu_index=0.
     */
    #define OVPHYSX_CREATE_ARGS_DEFAULT { \
        {NULL, 0},         /* bundled_deps_path */ \
        NULL,              /* settings_keys */ \
        NULL,              /* settings_values */ \
        0,                 /* settings_count */ \
        OVPHYSX_DEVICE_GPU, /* device */ \
        0                  /* gpu_index */ \
    }


#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_TYPES_H */
