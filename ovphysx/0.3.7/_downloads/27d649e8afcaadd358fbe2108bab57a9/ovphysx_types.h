// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef OVPHYSX_TYPES_H
#define OVPHYSX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dlpack/dlpack.h"

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
    typedef uint64_t ovphysx_contact_binding_handle_t;

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
        OVPHYSX_LOG_VERBOSE = 0, /**< All messages including verbose/debug (maps to Carbonite kLevelVerbose) */
        OVPHYSX_LOG_INFO    = 1, /**< Info, warnings, and errors */
        OVPHYSX_LOG_WARNING = 2, /**< Warnings and errors (default) */
        OVPHYSX_LOG_ERROR   = 3, /**< Error messages only */
        OVPHYSX_LOG_NONE    = 4  /**< No logging */
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
    /* PhysX object type enum                           */
    /*--------------------------------------------------*/

    /**
     * Identifies the type of PhysX object at a USD prim path.
     *
     * Used with ovphysx_get_physx_ptr() to retrieve raw PhysX SDK pointers.
     * Values match the internal omni::physx::PhysXType enum; the named constants
     * below cover common types. Any valid omni::physx::PhysXType integer may be
     * passed; PhysX returns NULL for unrecognized path/type combinations.
     *
     * | Enum value                         | PhysX SDK C++ type                            |
     * |------------------------------------|-----------------------------------------------|
     * | OVPHYSX_PHYSX_TYPE_SCENE           | physx::PxScene                                |
     * | OVPHYSX_PHYSX_TYPE_MATERIAL        | physx::PxMaterial                             |
     * | OVPHYSX_PHYSX_TYPE_SHAPE           | physx::PxShape                                |
     * | OVPHYSX_PHYSX_TYPE_COMPOUND_SHAPE  | omni::physx::PhysXCompoundShape (see note)    |
     * | OVPHYSX_PHYSX_TYPE_ACTOR           | physx::PxRigidDynamic/PxRigidStatic           |
     * | OVPHYSX_PHYSX_TYPE_JOINT           | physx::PxJoint (standalone joints)            |
     * | OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT    | physx::PxJoint (custom joints)                |
     * | OVPHYSX_PHYSX_TYPE_ARTICULATION    | physx::PxArticulationReducedCoordinate        |
     * | OVPHYSX_PHYSX_TYPE_LINK            | physx::PxArticulationLink                     |
     * | OVPHYSX_PHYSX_TYPE_LINK_JOINT      | physx::PxArticulationJointReducedCoordinate   |
     * | OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM | physx::PxPBDParticleSystem                    |
     * | OVPHYSX_PHYSX_TYPE_PARTICLE_SET    | physx::PxParticleBuffer                       |
     * | OVPHYSX_PHYSX_TYPE_PHYSICS         | physx::PxPhysics                              |
     *
     * Note: COMPOUND_SHAPE returns an omni::physx::PhysXCompoundShape*, defined in the
     * PhysX repo at omni/include/private/omni/physx/PhysXCompoundShape.h.
     * Call getShapes() on the result to access the underlying physx::PxShape pointers.
     */
    typedef enum
    {
        OVPHYSX_PHYSX_TYPE_SCENE           = 1,
        OVPHYSX_PHYSX_TYPE_MATERIAL        = 2,
        OVPHYSX_PHYSX_TYPE_SHAPE           = 3,
        OVPHYSX_PHYSX_TYPE_COMPOUND_SHAPE  = 4,
        OVPHYSX_PHYSX_TYPE_ACTOR           = 5,
        OVPHYSX_PHYSX_TYPE_JOINT           = 6,
        OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT    = 7,
        OVPHYSX_PHYSX_TYPE_ARTICULATION    = 8,
        OVPHYSX_PHYSX_TYPE_LINK            = 9,
        OVPHYSX_PHYSX_TYPE_LINK_JOINT      = 10,
        OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM = 11,
        OVPHYSX_PHYSX_TYPE_PARTICLE_SET    = 12,
        OVPHYSX_PHYSX_TYPE_PHYSICS         = 31,
    } ovphysx_physx_type_t;

    /*--------------------------------------------------*/
    /* Scene query types                                */
    /*--------------------------------------------------*/

    /**
     * Scene query mode -- controls how many hits are returned.
     */
    typedef enum
    {
        OVPHYSX_SCENE_QUERY_MODE_CLOSEST = 0, /**< Return the single closest hit (or none). */
        OVPHYSX_SCENE_QUERY_MODE_ANY     = 1, /**< Return whether any hit exists (0 or 1 result). */
        OVPHYSX_SCENE_QUERY_MODE_ALL     = 2, /**< Return all hits. */
    } ovphysx_scene_query_mode_t;

    /**
     * Geometry type for sweep and overlap queries.
     *
     * SHAPE accepts any UsdGeomGPrim path (sphere, box, capsule, cone,
     * cylinder, mesh, etc.). For meshes the runtime uses a convex
     * approximation internally.
     */
    typedef enum
    {
        OVPHYSX_SCENE_QUERY_GEOMETRY_SPHERE = 0, /**< Sphere defined by radius + center position. */
        OVPHYSX_SCENE_QUERY_GEOMETRY_BOX    = 1, /**< Oriented box defined by half-extents + pose. */
        OVPHYSX_SCENE_QUERY_GEOMETRY_SHAPE  = 2, /**< Arbitrary UsdGeomGPrim identified by prim path. */
    } ovphysx_scene_query_geometry_type_t;

    /**
     * Geometry descriptor for sweep/overlap queries.
     *
     * Set `type` and fill the corresponding union member.
     */
    typedef struct
    {
        ovphysx_scene_query_geometry_type_t type;
        union
        {
            struct
            {
                float radius;       /**< Sphere radius. */
                float position[3];  /**< Sphere center (world space). */
            } sphere;
            struct
            {
                float half_extent[3]; /**< Box half-extents. */
                float position[3];    /**< Box center (world space). */
                float rotation[4];    /**< Box orientation quaternion (x, y, z, w). */
            } box;
            struct
            {
                const char* prim_path; /**< Null-terminated USD prim path for any UsdGeomGPrim. */
            } shape;
        };
    } ovphysx_scene_query_geometry_desc_t;

    /**
     * Scene query hit result.
     *
     * Used for raycast, sweep, and overlap queries. For overlap queries the
     * location fields (normal, position, distance, face_index, material) are
     * zeroed -- only the object identity fields are populated.
     *
     * Path fields (collision, rigid_body, material) are uint64-encoded SdfPaths
     * matching the internal Omni PhysX representation.
     */
    typedef struct
    {
        uint64_t collision;    /**< Collision shape SdfPath (uint64 encoded). */
        uint64_t rigid_body;   /**< Rigid body SdfPath (uint64 encoded). */
        uint32_t proto_index;  /**< Point instancer prototype index (0xFFFFFFFF if N/A). */
        float    normal[3];    /**< Hit normal (world space). Zero for overlap queries. */
        float    position[3];  /**< Hit position (world space). Zero for overlap queries. */
        float    distance;     /**< Hit distance along ray/sweep direction. Zero for overlap. */
        uint32_t face_index;   /**< Triangle mesh face index. Zero for non-mesh hits. */
        uint64_t material;     /**< Material SdfPath (uint64 encoded). Zero for overlap. */
    } ovphysx_scene_query_hit_t;

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
        OVPHYSX_API_BUFFER_TOO_SMALL = 6,    /**< Caller-supplied buffer is too small; check out_required_size */
        OVPHYSX_API_DEVICE_MISMATCH = 7,     /**< Tensor device doesn't match binding's expected device */
        OVPHYSX_API_GPU_NOT_AVAILABLE = 8,   /**< GPU requested but not available or CUDA init failed */
    } ovphysx_api_status_t;

    /**
     * Result returned by synchronous API functions.
     *
     * On failure (status != OVPHYSX_API_SUCCESS), call ovphysx_get_last_error()
     * on the same thread to retrieve the error message.
     */
    typedef struct
    {
        ovphysx_api_status_t status; /**< Operation status code */
    } ovphysx_result_t;

    /**
     * Result returned by asynchronous API functions.
     *
     * On failure (status != OVPHYSX_API_SUCCESS), call ovphysx_get_last_error()
     * on the same thread to retrieve the error message.
     */
    typedef struct
    {
        ovphysx_api_status_t status; /**< Operation status code */
        ovphysx_op_index_t op_index; /**< Operation index for async tracking */
    } ovphysx_enqueue_result_t;

    /*--------------------------------------------------*/
    /* Operation tracking types */
    /*--------------------------------------------------*/

    /**
     * Result from ovphysx_wait_op() containing failed op indices and pending operation status.
     *
     * For each failed op index, call ovphysx_get_last_op_error(op_index) to
     * retrieve the error message. Free this struct via ovphysx_destroy_wait_result().
     */
    typedef struct
    {
        ovphysx_op_index_t* error_op_indices;         /**< Array of op indices that failed (free via ovphysx_destroy_wait_result) */
        size_t num_errors;                            /**< Number of failed op indices */
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
     *
     * ============================================================================
     * RIGID BODY PROPERTY TENSORS (standalone non-articulated bodies)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_MASS_F32
     *     Shape: [N] where N = number of rigid bodies
     *     Layout: scalar mass per body
     *     Units: kilograms
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32
     *     Shape: [N, 9] where N = number of rigid bodies
     *     Layout: row-major 3x3 inertia tensor in center-of-mass frame
     *     Units: kg*m^2
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32
     *     Shape: [N, 7] where N = number of rigid bodies
     *     Layout: [px, py, pz, qx, qy, qz, qw] (position xyz, quaternion xyzw)
     *     Frame: Local frame (relative to body origin)
     *     DType: float32
     *
     * ============================================================================
     * ARTICULATION LINK ACCELERATION (READ-ONLY)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32
     *     Shape: [N, L, 6] where N = articulations, L = max links
     *     Layout: [ax, ay, az, alpha_x, alpha_y, alpha_z] (linear + angular acc)
     *     Frame: World
     *     DType: float32
     *
     * ============================================================================
     * ARTICULATION DOF PROPERTY TENSORS (read/write)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32
     *     Shape: [N, D] where N = articulations, D = max DOFs
     *     Layout: Joint stiffness values
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32
     *     Shape: [N, D]
     *     Layout: Joint damping values
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32
     *     Shape: [N, D, 2]
     *     Layout: (lower, upper) position limit per DOF
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32
     *     Shape: [N, D]
     *     Layout: Maximum velocity per DOF
     *     Units: rad/s or m/s
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32
     *     Shape: [N, D]
     *     Layout: Maximum force/torque per DOF
     *     Units: N or Nm
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32
     *     Shape: [N, D]
     *     Layout: Armature (reflected inertia) per DOF
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32
     *     Shape: [N, D, 3]
     *     Layout: (static, dynamic, viscous) friction coefficients per DOF
     *     DType: float32
     *
     * ============================================================================
     * ARTICULATION BODY PROPERTY TENSORS (read/write, except where noted)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32
     *     Shape: [N, L] where N = articulations, L = max links
     *     Layout: Mass per link
     *     Units: kilograms
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32
     *     Shape: [N, L, 7]
     *     Layout: [px, py, pz, qx, qy, qz, qw] (COM local pose per link)
     *     Frame: Local frame (relative to link origin)
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32
     *     Shape: [N, L, 9]
     *     Layout: row-major 3x3 inertia tensor in COM frame per link
     *     Units: kg*m^2
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32 (READ-ONLY)
     *     Shape: [N, L]
     *     Layout: Inverse mass (1/m) per link
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32 (READ-ONLY)
     *     Shape: [N, L, 9]
     *     Layout: row-major 3x3 inverse inertia in COM frame per link
     *     DType: float32
     *
     * ============================================================================
     * DYNAMICS QUERY TENSORS (READ-ONLY)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32
     *     Shape: [N, R, C] -- obtain R, C from getJacobianShape()
     *       Fixed-base:    R = (numLinks - 1) * 6,     C = numDofs
     *       Floating-base: R = (numLinks - 1) * 6 + 6, C = numDofs + 6
     *       Floating-base columns: base 6 DOFs at indices 0..5, joint DOFs at 6..C-1
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32
     *     Shape: [N, M, M] -- obtain M from getGeneralizedMassMatrixShape()
     *     Layout: Generalized (joint-space) mass matrix
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32
     *     Shape: [N, M]
     *     Layout: Combined Coriolis and centrifugal compensation forces
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32
     *     Shape: [N, M]
     *     Layout: Gravity compensation forces
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32
     *     Shape: [N, L, 6]
     *     Layout: [fx, fy, fz, tx, ty, tz] per link incoming joint force
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32
     *     Shape: [N, D]
     *     Layout: Projected joint forces per DOF
     *     DType: float32
     *
     * ============================================================================
     * FIXED TENDON PROPERTY TENSORS (read/write, require articulation with tendons)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32
     *     Shape: [N, T] where N = articulations, T = max fixed tendons
     *     Layout: Tendon stiffness
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32
     *     Shape: [N, T]
     *     Layout: Tendon damping
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32
     *     Shape: [N, T]
     *     Layout: Stiffness of the tendon length limit spring
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32
     *     Shape: [N, T, 2]
     *     Layout: (lower, upper) tendon length limits
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32
     *     Shape: [N, T]
     *     Layout: Tendon rest length
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32
     *     Shape: [N, T]
     *     Layout: Tendon offset
     *     DType: float32
     *
     * ============================================================================
     * SPATIAL TENDON PROPERTY TENSORS (read/write, require articulation with spatial tendons)
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32
     *     Shape: [N, T] where N = articulations, T = max spatial tendons
     *     Layout: Spatial tendon stiffness
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32
     *     Shape: [N, T]
     *     Layout: Spatial tendon damping
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32
     *     Shape: [N, T]
     *     Layout: Stiffness of the spatial tendon length limit spring
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32
     *     Shape: [N, T]
     *     Layout: Spatial tendon offset
     *     DType: float32
     */
    typedef enum
    {
        OVPHYSX_TENSOR_INVALID = 0,

        /* Rigid body tensors */
        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32 = 1,          /**< [N, 7] poses in world frame */
        OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32 = 2,      /**< [N, 6] velocities in world frame */

        /* Rigid body property tensors (standalone non-articulated bodies) */
        OVPHYSX_TENSOR_RIGID_BODY_MASS_F32 = 3,          /**< [N] mass per body */
        OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32 = 4,       /**< [N, 9] row-major 3x3 inertia tensor */
        OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32 = 5,      /**< [N, 7] COM local pose (px,py,pz,qx,qy,qz,qw) */

        /* Articulation root tensors */
        OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32 = 10,      /**< [N, 7] root poses */
        OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32 = 11,  /**< [N, 6] root velocities */

        /* Articulation link tensors (3D) */
        OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32 = 20,         /**< [N, L, 7] link poses */
        OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32 = 21,     /**< [N, L, 6] link velocities */
        OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32 = 22, /**< [N, L, 6] link accelerations (lin_acc xyz + ang_acc xyz), READ-ONLY */

        /* Articulation DOF tensors */
        OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32 = 30,           /**< [N, D] joint positions */
        OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32 = 31,           /**< [N, D] joint velocities */
        OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32 = 32,    /**< [N, D] position targets */
        OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32 = 33,    /**< [N, D] velocity targets */
        OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32 = 34,    /**< [N, D] actuation forces */

        /* DOF property tensors (read/write) */
        OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32 = 35,        /**< [N, D] joint stiffness */
        OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32 = 36,          /**< [N, D] joint damping */
        OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32 = 37,            /**< [N, D, 2] (lower, upper) per DOF */
        OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32 = 38,     /**< [N, D] max velocity per DOF */
        OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32 = 39,        /**< [N, D] max force per DOF */
        OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32 = 40,         /**< [N, D] armature per DOF */
        OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32 = 41, /**< [N, D, 3] (static, dynamic, viscous) */

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

        /* Articulation body property tensors (read/write) */
        OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32 = 60,            /**< [N, L] mass per link */
        OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32 = 61,        /**< [N, L, 7] COM local pose (px,py,pz,qx,qy,qz,qw) */
        OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32 = 62,         /**< [N, L, 9] row-major 3x3 inertia in COM frame */
        OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32 = 63,        /**< [N, L] inverse mass (1/m) per link (READ-ONLY) */
        OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32 = 64,     /**< [N, L, 9] inverse inertia in COM frame (READ-ONLY) */

        /* Dynamics query tensors (READ-ONLY) */
        /** [N, R, C] from getJacobianShape().
         *  For fixed-base: R = (numLinks-1)*6, C = numDofs.
         *  For floating-base: R = (numLinks-1)*6 + 6, C = numDofs + 6.
         *  Floating-base columns: base 6 DOFs at indices 0..5, joint DOFs at 6..C-1. */
        OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32 = 70,
        OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32 = 71,                /**< [N, M, M] from getGeneralizedMassMatrixShape() */
        OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32 = 72, /**< [N, M] Coriolis + centrifugal forces (both terms, from getCoriolisAndCentrifugalCompensationForces()) */
        OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32 = 73,              /**< [N, M] gravity compensation */
        OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32 = 74,  /**< [N, L, 6] per-link incoming joint force */
        OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32 = 75, /**< [N, D] projected joint forces (READ-ONLY) */

        /* Fixed tendon property tensors (read/write, require articulation with tendons) */
        OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32 = 80,        /**< [N, T] tendon stiffness */
        OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32 = 81,          /**< [N, T] tendon damping */
        OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32 = 82,  /**< [N, T] tendon limit stiffness */
        OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32 = 83,            /**< [N, T, 2] (lower, upper) tendon limits */
        OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32 = 84,      /**< [N, T] tendon rest length */
        OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32 = 85,           /**< [N, T] tendon offset */

        /* Spatial tendon property tensors (read/write, require articulation with spatial tendons) */
        OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32 = 90,       /**< [N, T] spatial tendon stiffness */
        OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32 = 91,         /**< [N, T] spatial tendon damping */
        OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32 = 92, /**< [N, T] spatial tendon limit stiffness */
        OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32 = 93,          /**< [N, T] spatial tendon offset */

        /* Shape-level property tensors (per collision shape on each rigid body or link).
           S = max collision shapes per body/link.  Bodies with fewer shapes
           have zero-padded trailing entries. */
        OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32 = 100, /**< [N, S, 3] (static friction, dynamic friction, restitution) per shape */
        OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32 = 101,            /**< [N, S] contact offset per shape */
        OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32 = 102,               /**< [N, S] rest offset per shape */

        OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32 = 110, /**< [N, S, 3] (static friction, dynamic friction, restitution) per link shape */
        OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32 = 111,          /**< [N, S] contact offset per link shape */
        OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32 = 112,             /**< [N, S] rest offset per link shape */
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

    /**
     * Articulation topology metadata returned by ovphysx_get_articulation_metadata().
     *
     * All fields are read at binding-creation time and remain constant for the
     * lifetime of the binding.
     *
     * String arrays (DOF names, body names, joint names) are NOT included here
     * because they are variable-length and require caller-allocated buffers; use
     * ovphysx_articulation_get_dof_names / get_body_names / get_joint_names instead.
     */
    typedef struct ovphysx_articulation_metadata_t
    {
        int32_t  dof_count;            /**< Number of degrees of freedom (DOFs) */
        int32_t  body_count;           /**< Number of links */
        int32_t  joint_count;          /**< Number of joints */
        int32_t  fixed_tendon_count;   /**< Max fixed tendons (0 if none) */
        int32_t  spatial_tendon_count; /**< Max spatial tendons (0 if none) */
        bool     is_fixed_base;        /**< True if base link is fixed in world */
    } ovphysx_articulation_metadata_t;

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
    /* Contact report types                              */
    /*--------------------------------------------------*/

    /**
     * Contact event header — describes one contact pair.
     *
     * ABI-compatible with omni::physx::ContactEventHeader.
     * Each header references a slice of the contact data array
     * (contactDataOffset .. contactDataOffset + numContactData).
     */
    typedef struct ovphysx_contact_event_header_t
    {
        int32_t  type;                    /**< 0 = found, 1 = lost, 2 = persist */
        int64_t  stageId;                 /**< USD stage ID */
        uint64_t actor0;                  /**< Actor 0 USD path (SdfPath encoded as uint64) */
        uint64_t actor1;                  /**< Actor 1 USD path */
        uint64_t collider0;               /**< Collider 0 USD path */
        uint64_t collider1;               /**< Collider 1 USD path */
        uint32_t contactDataOffset;       /**< Index into the contact data array */
        uint32_t numContactData;          /**< Number of contact points for this pair */
        uint32_t frictionAnchorsDataOffset; /**< Index into the friction anchors array */
        uint32_t numfrictionAnchorsData;  /**< Number of friction anchors for this pair */
        uint32_t protoIndex0;             /**< Point instancer index (0xFFFFFFFF if N/A) */
        uint32_t protoIndex1;             /**< Point instancer index (0xFFFFFFFF if N/A) */
    } ovphysx_contact_event_header_t;

    /**
     * Per-contact-point data — ABI-compatible with omni::physx::ContactData.
     *
     * position, normal, and impulse are float[3] in world space.
     */
    typedef struct ovphysx_contact_point_t
    {
        float    position[3];             /**< Contact position (world space) */
        float    normal[3];               /**< Contact normal */
        float    impulse[3];              /**< Contact impulse (divide by dt for force) */
        float    separation;              /**< Contact separation distance */
        uint32_t faceIndex0;              /**< Triangle mesh face index for collider 0 */
        uint32_t faceIndex1;              /**< Triangle mesh face index for collider 1 */
        uint64_t material0;               /**< Material SdfPath for collider 0 */
        uint64_t material1;               /**< Material SdfPath for collider 1 */
    } ovphysx_contact_point_t;

    /**
     * Friction anchor data — ABI-compatible with omni::physx::FrictionAnchor.
     */
    typedef struct ovphysx_friction_anchor_t
    {
        float position[3];                /**< Anchor position (world space) */
        float impulse[3];                 /**< Friction impulse (divide by dt for force) */
    } ovphysx_friction_anchor_t;

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
     * AUTO mode (default, OVPHYSX_DEVICE_AUTO):
     *   - GPU preferred: uses GPU if a usable CUDA driver is detected, otherwise CPU
     *   - Recommended for portable applications that should work on any machine
     *
     * GPU mode (OVPHYSX_DEVICE_GPU):
     *   - Enables PhysX GPU acceleration and DirectGPU API
     *   - TensorBinding works with GPU tensors (kDLCUDA) for zero-copy access
     *   - Requires a warmup step() before GPU tensor reads
     * 
     * CPU mode (OVPHYSX_DEVICE_CPU):
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
        /**
         * AUTO (GPU PREFERRED) -- zero-init safe default.
         *
         * ovphysx prefers GPU simulation when a usable CUDA driver + CUDA device
         * are available. If CUDA is unavailable, ovphysx falls back to CPU-only
         * simulation and still creates the instance successfully.
         *
         * This is the recommended default for best developer UX: a single wheel/
         * SDK works on both GPU and CPU-only machines without special casing.
         *
         * GPU ordinal handling:
         * - If gpu_index >= 0, that ordinal is used when GPU is selected.
         * - If gpu_index == -1, ovphysx uses PhysX's default CUDA selection.
         */
        OVPHYSX_DEVICE_AUTO = 0,

        /**
         * GPU simulation (GPU REQUIRED).
         *
         * This mode requires a usable CUDA driver + at least one CUDA device.
         * If CUDA is unavailable, ovphysx_create_instance() fails with
         * OVPHYSX_API_GPU_NOT_AVAILABLE. There is no silent fallback to CPU.
         *
         * Use this when your application depends on GPU/DirectGPU behavior and
         * you want an explicit error on CPU-only machines.
         */
        OVPHYSX_DEVICE_GPU = 1,

        /**
         * CPU-only simulation (GPU DISABLED).
         *
         * This mode never attempts to use CUDA and is expected to work on
         * machines without a CUDA driver/GPU.
         */
        OVPHYSX_DEVICE_CPU = 2,
    } ovphysx_device_t;

    /*--------------------------------------------------*/
    /* Typed config system                              */
    /*--------------------------------------------------*/

    /** Config key type discriminator — selects which key/value union members are valid. */
    typedef enum ovphysx_config_key_type_t
    {
        OVPHYSX_CONFIG_KEY_TYPE_BOOL,       /**< Key from ovphysx_config_bool_t, value is bool */
        OVPHYSX_CONFIG_KEY_TYPE_INT32,      /**< Key from ovphysx_config_int32_t, value is int32_t */
        OVPHYSX_CONFIG_KEY_TYPE_FLOAT,      /**< Key from ovphysx_config_float_t, value is float */
        OVPHYSX_CONFIG_KEY_TYPE_STRING,     /**< Key from ovphysx_config_string_t, value is ovphysx_string_t */
        OVPHYSX_CONFIG_KEY_TYPE_CARBONITE,  /**< Escape hatch: arbitrary Carbonite path (string key + string value) */
        OVPHYSX_CONFIG_KEY_TYPE_COUNT
    } ovphysx_config_key_type_t;

    /** Boolean config keys. Value type: bool. */
    typedef enum ovphysx_config_bool_t
    {
        OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING,        /**< /physics/disableContactProcessing */
        OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY,     /**< /physics/collisionConeCustomGeometry */
        OVPHYSX_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY, /**< /physics/collisionCylinderCustomGeometry */
        OVPHYSX_CONFIG_BOOL_COUNT
    } ovphysx_config_bool_t;

    /** Int32 config keys. Value type: int32_t. */
    typedef enum ovphysx_config_int32_t
    {
        OVPHYSX_CONFIG_NUM_THREADS,          /**< /physics/numThreads */
        OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, /**< /physics/sceneMultiGPUMode (0=disabled, 1=all, 2=skip-first) */
        OVPHYSX_CONFIG_INT32_COUNT
    } ovphysx_config_int32_t;

    /** Float config keys (reserved for future use). Value type: float. */
    typedef enum ovphysx_config_float_t
    {
        OVPHYSX_CONFIG_FLOAT_COUNT
    } ovphysx_config_float_t;

    /** String config keys (reserved for future use). Value type: ovphysx_string_t. */
    typedef enum ovphysx_config_string_t
    {
        OVPHYSX_CONFIG_STRING_COUNT
    } ovphysx_config_string_t;

    /**
     * A typed config entry (tagged union).
     *
     * key_type selects which member of key and value is valid.
     * Use the builder functions in ovphysx_config.h for convenient construction.
     */
    typedef struct ovphysx_config_entry_t
    {
        ovphysx_config_key_type_t key_type;  /**< Discriminator */
        union {
            ovphysx_config_bool_t bool_key;
            ovphysx_config_int32_t int32_key;
            ovphysx_config_float_t float_key;
            ovphysx_config_string_t string_key;
            ovphysx_string_t carbonite_key;      /**< For KEY_TYPE_CARBONITE: arbitrary Carbonite path */
        } key;
        union {
            bool bool_value;
            int32_t int32_value;
            float float_value;
            ovphysx_string_t string_value;       /**< For KEY_TYPE_STRING and KEY_TYPE_CARBONITE */
        } value;
    } ovphysx_config_entry_t;

    /** Config array container (convenience wrapper). */
    typedef struct ovphysx_config_t
    {
        const ovphysx_config_entry_t* entries;
        uint32_t entry_count;
    } ovphysx_config_t;

    /*--------------------------------------------------*/
    /* Instance creation                                */
    /*--------------------------------------------------*/

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
        ovphysx_string_t bundled_deps_path;          /**< Bundled deps path: empty=use runtime discovery (default: empty) */

        const ovphysx_config_entry_t* config_entries; /**< Array of typed config entries (replaces settings_keys/values) */
        uint32_t config_entry_count;                  /**< Number of config entries */

        ovphysx_device_t device;            /**< Simulation device (default: OVPHYSX_DEVICE_AUTO) */
        int32_t gpu_index;                  /**< CUDA device ordinal (default: 0).
                                                 Used when device selects GPU (OVPHYSX_DEVICE_GPU or OVPHYSX_DEVICE_AUTO and CUDA is available).
                                                 Special values:
                                                 -1 = PhysX default CUDA selection (equivalent to /physics/cudaDevice=-1) */
    } ovphysx_create_args;

    /**
     * Default initializer for ovphysx_create_args.
     * Sets device=AUTO (GPU preferred), gpu_index=0.
     */
    #define OVPHYSX_CREATE_ARGS_DEFAULT { \
        {NULL, 0},           /* bundled_deps_path */ \
        NULL,                /* config_entries */ \
        0,                   /* config_entry_count */ \
        OVPHYSX_DEVICE_AUTO, /* device */ \
        0                    /* gpu_index */ \
    }


#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_TYPES_H */
