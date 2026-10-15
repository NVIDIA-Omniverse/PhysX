// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1
 */
/** @endcond */

#ifndef OVPHYSX_LEGACY_TYPES_H
#define OVPHYSX_LEGACY_TYPES_H

#include "ovphysx/ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef uint64_t ovphysx_tensor_binding_handle_t; /**< @deprecated Tensor-binding handle. Use ovphysx_read / ovphysx_write sessions instead. */
    typedef uint64_t ovphysx_contact_binding_handle_t;
    typedef uint64_t ovphysx_sdf_view_handle_t;

    /**
     * @brief High-level object classification for prim paths (TensorAPI-level).
     *
     * @deprecated Returned only by @ref ovphysx_get_object_type. Session
     * read/write and physxPointer take @ref ovphysx_query_target_t.
     * @ref ovphysx_physx_type_t is the kind carried by object-change callbacks.
     *
     * Mirrors omni::physics::tensors::ObjectType. The
     * @c JOINT / @c ARTICULATION_JOINT / @c CUSTOM_JOINT split mirrors
     * @ref ovphysx_physx_type_t's @c OVPHYSX_PHYSX_TYPE_JOINT,
     * @c OVPHYSX_PHYSX_TYPE_LINK_JOINT, and @c OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT.
     */
    typedef enum
    {
        OVPHYSX_OBJECT_TYPE_INVALID                = 0, /**< No classified simulation object at the path */
        OVPHYSX_OBJECT_TYPE_RIGID_BODY             = 1,
        OVPHYSX_OBJECT_TYPE_ARTICULATION           = 2,
        OVPHYSX_OBJECT_TYPE_ARTICULATION_LINK      = 3,
        OVPHYSX_OBJECT_TYPE_ARTICULATION_ROOT_LINK = 4,
        OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT     = 5, /**< Reduced-coordinate articulation joint */
        OVPHYSX_OBJECT_TYPE_JOINT                  = 6, /**< Maximal-coordinate standalone joint (physx::PxJoint) */
        OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT           = 7, /**< Plugin-registered custom joint (CustomPhysXJoint) */
    } ovphysx_object_type_t;

    /*--------------------------------------------------*/
    /* Legacy typed configuration                      */
    /*--------------------------------------------------*/

    /** @deprecated Use ovphysx_config_value_type_t. */
    typedef enum ovphysx_config_key_type_t
    {
        OVPHYSX_CONFIG_KEY_TYPE_BOOL,
        OVPHYSX_CONFIG_KEY_TYPE_INT32,
        OVPHYSX_CONFIG_KEY_TYPE_FLOAT,
        OVPHYSX_CONFIG_KEY_TYPE_STRING,
        OVPHYSX_CONFIG_KEY_TYPE_CARBONITE,
        OVPHYSX_CONFIG_KEY_TYPE_COUNT
    } ovphysx_config_key_type_t;

    /** @deprecated Use ovphysx_global_config_key_t. */
    typedef enum ovphysx_config_bool_t
    {
        OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING,
        OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY,
        OVPHYSX_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY,
        /* Slots 3 and 4 are retired OVD settings. Do not reuse. */
        OVPHYSX_CONFIG_BOOL_COUNT = 5
    } ovphysx_config_bool_t;

    /** @deprecated Use ovphysx_global_config_key_t. */
    typedef enum ovphysx_config_int32_t
    {
        OVPHYSX_CONFIG_NUM_THREADS,          /**< /persistent/physics/numThreads */
        OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE,
        /* Slots 2 and 3 are retired OVD settings. Do not reuse. */
        OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB = 4,
        OVPHYSX_CONFIG_PROFILER_BACKEND,
        OVPHYSX_CONFIG_INT32_COUNT
    } ovphysx_config_int32_t;

    /** @deprecated Use ovphysx_global_config_key_t. */
    typedef enum ovphysx_config_float_t
    {
        OVPHYSX_CONFIG_FLOAT_COUNT
    } ovphysx_config_float_t;

    /** @deprecated Use ovphysx_global_config_key_t. */
    typedef enum ovphysx_config_string_t
    {
        /* Slots 0, 2 and 3 are retired OVD settings. Do not reuse. */
        OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY = 1,
        OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH = 4,
        OVPHYSX_CONFIG_STRING_COUNT
    } ovphysx_config_string_t;

    /** @deprecated Use ovphysx_config_value_t and a scoped key enum. */
    typedef struct ovphysx_config_entry_t
    {
        ovphysx_config_key_type_t key_type;
        union
        {
            ovphysx_config_bool_t bool_key;
            ovphysx_config_int32_t int32_key;
            ovphysx_config_float_t float_key;
            ovphysx_config_string_t string_key;
            ovphysx_string_t carbonite_key;
        } key;
        union
        {
            bool bool_value;
            int32_t int32_value;
            float float_value;
            ovphysx_string_t string_value;
        } value;
    } ovphysx_config_entry_t;

    /** @deprecated Use scoped configuration setters. */
    typedef struct ovphysx_config_t
    {
        const ovphysx_config_entry_t* entries;
        uint32_t entry_count;
    } ovphysx_config_t;

    /**
     * @deprecated Use ovphysx_create_instance(), then ovphysx_set_config()
     * before attaching a stage.
     */
    typedef struct ovphysx_create_args
    {
        ovphysx_string_t bundled_deps_path;
        const ovphysx_config_entry_t* config_entries;
        uint32_t config_entry_count;
        ovphysx_string_t active_cuda_gpus;
    } ovphysx_create_args;

    #define OVPHYSX_CREATE_ARGS_DEFAULT { \
        {NULL, 0}, \
        NULL, \
        0, \
        {NULL, 0} \
    }

    /*--------------------------------------------------*/
    /* Scene query types                                */
    /*--------------------------------------------------*/

    /**
     * Scene query mode. Controls how many hits are returned.
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
                ovphysx_string_t prim_path; /**< USD prim path for any UsdGeomGPrim (embedded NUL bytes are rejected). */
            } shape;
        };
    } ovphysx_scene_query_geometry_desc_t;

    /**
     * Scene query hit result.
     *
     * Used for raycast, sweep, and overlap queries. For overlap queries the
     * location fields (normal, position, distance, face_index, material) are
     * zeroed. Only the object identity fields are populated.
     *
     * Path fields (collision, rigid_body, material) hold an opaque
     * omni::physics::parse::ObjectKey.handle, a runtime-assigned identity, not a
     * uint64-encoded SdfPath. There is no client-side bit-cast that reproduces or
     * compares against it. Resolve it to a prim path with
     * ovphysx_scene_query_get_paths_from_ids().
     */
    typedef struct
    {
        uint64_t collision;    /**< Collision shape identity (opaque ObjectKey.handle). */
        uint64_t rigid_body;   /**< Rigid body identity (opaque ObjectKey.handle). */
        uint32_t proto_index;  /**< Point instancer prototype index (0xFFFFFFFF if N/A). */
        float    normal[3];    /**< Hit normal (world space). Zero for overlap queries. */
        float    position[3];  /**< Hit position (world space). Zero for overlap queries. */
        float    distance;     /**< Hit distance along ray/sweep direction. Zero for overlap. */
        uint32_t face_index;   /**< Triangle mesh face index. Zero for non-mesh hits. */
        uint64_t material;     /**< Material identity (opaque ObjectKey.handle). Zero for overlap. */
    } ovphysx_scene_query_hit_t;

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
     *   OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32
     *     Shape: [N, 6] where N = number of rigid bodies
     *     Layout: [ax, ay, az, alpha_x, alpha_y, alpha_z] (linear + angular acc)
     *     Frame: World
     *     DType: float32
     *     Access: read-only
     *
     * ============================================================================
     * ARTICULATION TENSORS
     * ============================================================================
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32
     *     Shape: [N, 7] where N = number of articulations
     *     Layout: [px, py, pz, qx, qy, qz, qw]
     *     Frame: Exposed view world (subspace origin removed)
     *     DType: float32
     * 
     *   OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32
     *     Shape: [N, 6] where N = number of articulations
     *     Layout: [vx, vy, vz, wx, wy, wz]
     *     Frame: World
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32 (READ-ONLY)
     *     Shape: [N, 3] where N = number of articulations
     *     Layout: [x, y, z] center of mass per articulation
     *     Frame: Exposed view world (subspace origin removed)
     *     DType: float32
     *     Note: Computed from PxArticulationReducedCoordinate::computeArticulationCOM(false)
     *
     *   OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32 (READ-ONLY)
     *     Shape: [N, 3] where N = number of articulations
     *     Layout: [x, y, z] center of mass per articulation
     *     Frame: Root link's center-of-mass (mass) frame, not its actor/prim frame
     *     DType: float32
     *     Note: Computed from PxArticulationReducedCoordinate::computeArticulationCOM(true)
     *
     *   OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32 (READ-ONLY)
     *     Shape: [N, 6, D + 7] where N = articulations, D = getMaxDofs()
     *     Layout: 6 spatial-momentum rows (3 linear + 3 angular) of (D + 6) matrix
     *             columns followed by 1 bias column (D + 7 total).
     *             cols [0..5] = root spatial DOFs, cols [6..D+5] = joint DOFs,
     *             col [D+6] = centroidalMomentumBias[row].
     *     Frame: World, evaluated at the articulation COM.
     *     DType: float32
     *     Requires: Floating-base articulations only (PhysX errors out on fixed-base).
     *     Note: Computed from PxArticulationReducedCoordinate::computeCentroidalMomentumMatrix.
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
     *   OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32
     *     Shape: [N] where N = number of rigid bodies
     *     Layout: scalar inverse mass per body
     *     Units: 1/kg
     *     DType: float32
     *     Access: read-only
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32
     *     Shape: [N, 9] where N = number of rigid bodies
     *     Layout: row-major 3x3 inertia tensor in center-of-mass frame
     *     Units: kg*m^2
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32
     *     Shape: [N, 9] where N = number of rigid bodies
     *     Layout: row-major 3x3 inverse inertia tensor in center-of-mass frame
     *     Units: 1/(kg*m^2)
     *     DType: float32
     *     Access: read-only
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL
     *     Shape: [N] where N = number of rigid bodies
     *     Layout: per-body byte. Nonzero disables simulation, zero enables.
     *     DType: bool / uint8
     *     Access: read/write. Writes apply at runtime (engine toggles
     *             PxActorFlag::eDISABLE_SIMULATION on the underlying PxRigidActor
     *             so the body stops participating in the next solver step).
     *
     *   OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL
     *     Shape: [N] where N = number of rigid bodies
     *     Layout: per-body byte. Nonzero disables gravity, zero enables.
     *     DType: bool / uint8
     *     Access: read/write. Writes apply at runtime (engine toggles
     *             PxActorFlag::eDISABLE_GRAVITY on the underlying PxRigidActor).
     *     Reads return live PhysX actor flags (post-parse internal state only).
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
     * ARTICULATION DOF PROPERTY TENSORS (read/write, except where noted)
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
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32
     *     Shape: [N, D, 3]
     *     Layout: (speedEffortGradient, maxActuatorVelocity, velocityDependentResistance) per DOF
     *     DType: float32
     *     Note: only DOFs with PhysxDrivePerformanceEnvelopeAPI applied in USD accept writes.
     *           Writes to other DOFs are silently dropped.
     *
     *   OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8
     *     Shape: [N, D] where N = articulations, D = max DOFs (zero-padded)
     *     Layout: per-DOF byte in solver DOF order. 0 = no drive, 1 = force,
     *             2 = acceleration.
     *     DType: uint8
     *     Access: read-only. Reads return the live PxArticulationDrive driveType.
     *             Trailing padded DOF columns beyond numDofs read as 0.
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
     *   OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL
     *     Shape: [N, L] where N = articulations, L = max links (zero-padded)
     *     Layout: per-link byte in solver link order. Nonzero disables gravity.
     *     DType: bool / uint8
     *     Access: read/write. Writes apply at runtime (engine toggles
     *             PxActorFlag::eDISABLE_GRAVITY on each link actor). Trailing
     *             padded link columns beyond numLinks are ignored on write.
     *     Reads return live PhysX actor flags (post-parse internal state only).
     *
     * ============================================================================
     * INVERSE DYNAMICS QUERY TENSORS (READ-ONLY)
     * ============================================================================
     *
     *   Generalized joint coordinates follow the authored USD body relationship: their sign is
     *   +1 when body0 is the articulation parent and -1 when body1 is the parent. Let S_dof
     *   contain those signs and let T=S_dof for a fixed base or T=diag(I6,S_dof) for a floating
     *   base. The returned quantities satisfy J=J_physx*T, M=T*M_physx*T, c=T*c_physx,
     *   g=T*g_physx, and [A|b]=[A_physx*T|b_physx] for the packed centroidal matrix and bias. The
     *   six floating-root coordinates and centroidal bias are unchanged. Angular generalized-
     *   coordinate dimensions use radians and receive no degree conversion. Prismatic and
     *   floating-translation dimensions retain their linear units.
     *
     *   OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32
     *     Shape: [N, R, C] where R, C come from getJacobianShape()
     *       Fixed-base:    R = (numLinks - 1) * 6,     C = numDofs
     *       Floating-base: R = (numLinks - 1) * 6 + 6, C = numDofs + 6
     *       Floating-base columns: base 6 DOFs at indices 0..5, joint DOFs at 6..C-1
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32
     *     Shape: [N, M, M] where M comes from getGeneralizedMassMatrixShape()
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
     *
     * ============================================================================
     * VOLUME DEFORMABLE BODY TENSORS
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32
     *     Shape: [N, V, 3] where N = deformable bodies, V = max simulation nodes
     *     Layout: simulation mesh node positions
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32
     *     Shape: [N, V, 3]
     *     Layout: simulation mesh node velocities
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32
     *     Shape: [N, V, 4]
     *     Layout: simulation mesh kinematic targets (xyz position, flag)
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32
     *     Shape: [N, R, 3] where R = max rest nodes
     *     Layout: rest mesh node positions
     *     DType: float32
     *     Access: read-only
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32
     *     Shape: [N, E, K] where E = max simulation elements, K = nodes per element (4 for tetmesh)
     *     Layout: simulation element node indices
     *     DType: int32
     *     Access: read-only
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32
     *     Shape: [N, F, 4] where F = max collision elements. K is always 4 (matches backend fetchData)
     *     Layout: collision element node indices (tetrahedral, 4 nodes per element)
     *     DType: int32
     *     Access: read-only
     *
     * ============================================================================
     * SURFACE DEFORMABLE BODY TENSORS
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32
     *     Shape: [N, V, 3] where N = surface deformable bodies, V = max simulation nodes
     *     Layout: simulation mesh node positions
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32
     *     Shape: [N, V, 3]
     *     Layout: simulation mesh node velocities
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32
     *     Shape: [N, R, 3] where R = max rest nodes
     *     Layout: rest mesh node positions
     *     DType: float32
     *     Access: read-only
     *
     *   OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32
     *     Shape: [N, E, 3] where E = max simulation elements (triangles)
     *     Layout: simulation element node indices
     *     DType: int32
     *     Access: read-only
     *
     * ============================================================================
     * DEFORMABLE MATERIAL TENSORS
     * ============================================================================
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32
     *     Shape: [M] where M = deformable materials
     *     Layout: scalar dynamic friction
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32
     *     Shape: [M]
     *     Layout: scalar Young's modulus
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32
     *     Shape: [M]
     *     Layout: scalar Poisson's ratio
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32
     *     Shape: [M]
     *     Layout: scalar elasticity damping (volume + surface materials)
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32
     *     Shape: [M]
     *     Layout: scalar bending stiffness (surface materials only, 0.0 for volume material entries)
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32
     *     Shape: [M]
     *     Layout: scalar thickness (surface materials only, 0.0 for volume material entries)
     *     DType: float32
     *
     *   OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32
     *     Shape: [M]
     *     Layout: scalar bending damping (surface materials only, 0.0 for volume material entries)
     *     DType: float32
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    typedef enum
    {
        OVPHYSX_TENSOR_INVALID = 0,

        /* Rigid body tensors */
        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32 = 1,          /**< [N, 7] poses in world frame */
        OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32 = 2,      /**< [N, 6] velocities in world frame */
        OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32 = 6,  /**< [N, 6] accelerations in world frame (READ-ONLY) */

        /* Rigid body property tensors (standalone non-articulated bodies) */
        OVPHYSX_TENSOR_RIGID_BODY_MASS_F32 = 3,          /**< [N] mass per body */
        OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32 = 4,       /**< [N, 9] row-major 3x3 inertia tensor */
        OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32 = 5,      /**< [N, 7] COM local pose (px,py,pz,qx,qy,qz,qw) */
        OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32 = 7,      /**< [N] inverse mass per body (READ-ONLY) */
        OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32 = 8,   /**< [N, 9] inverse inertia tensor (READ-ONLY) */
        OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL = 9, /**< [N] uint8/bool. Nonzero disables, zero enables PxActorFlag::eDISABLE_SIMULATION at runtime */

        /* Articulation tensors */
        OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32 = 10,      /**< [N, 7] root poses in exposed view world frame */
        OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32 = 11,  /**< [N, 6] root velocities */
        OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32 = 12, /**< [N, 3] articulation COM in exposed view world frame (READ-ONLY) */
        OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32 = 13, /**< [N, 3] articulation COM in root-link mass frame (READ-ONLY) */
        OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32 = 14, /**< [N, 6, D+7] centroidal momentum matrix + bias column, floating-base only (READ-ONLY) */

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
        OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32 = 42, /**< [N, D, 3] (speedEffortGradient, maxActuatorVelocity, velocityDependentResistance) */
        OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8 = 43, /**< [N, D] uint8 drive type per DOF. 0=none, 1=force, 2=acceleration (READ-ONLY) */

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
        OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL = 65, /**< [N, L] uint8/bool. Nonzero disables PxActorFlag::eDISABLE_GRAVITY per link at runtime */

        /* Inverse dynamics query tensors (READ-ONLY) */
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
           S = max collision shapes per body/link. Bodies with fewer shapes
           have zero-padded trailing entries. */
        OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32 = 100, /**< [N, S, 3] (static friction, dynamic friction, restitution) per shape */
        OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32 = 101,            /**< [N, S] contact offset per shape */
        OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32 = 102,               /**< [N, S] rest offset per shape */
        OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL = 103,          /**< [N] uint8/bool. Nonzero disables PxActorFlag::eDISABLE_GRAVITY at runtime */

        OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32 = 110, /**< [N, S, 3] (static friction, dynamic friction, restitution) per link shape */
        OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32 = 111,          /**< [N, S] contact offset per link shape */
        OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32 = 112,             /**< [N, S] rest offset per link shape */

        /* Volume deformable body tensors */
        OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32 = 120,    /**< [N, V, 3] simulation node positions */
        OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32 = 121,    /**< [N, V, 3] simulation node velocities */
        OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32 = 122,  /**< [N, V, 4] simulation node kinematic targets (xyz, flag) */
        OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32 = 123,   /**< [N, R, 3] rest node positions (READ-ONLY) */
        OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32 = 124,   /**< [N, E, K] simulation element indices, K=4 tetmesh (int32, READ-ONLY) */
        OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32 = 125, /**< [N, F, 4] collision element indices, K=4 tetmesh (int32, READ-ONLY) */

        /* Surface deformable body tensors */
        OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32 = 140,         /**< [N, V, 3] simulation node positions */
        OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32 = 141,         /**< [N, V, 3] simulation node velocities */
        OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32 = 143,        /**< [N, R, 3] rest node positions (READ-ONLY) */
        OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32 = 144,  /**< [N, E, 3] simulation element indices, K=3 trimesh (int32, READ-ONLY) */

        /* Deformable material tensors */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32 = 130,     /**< [M] dynamic friction */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32 = 131,       /**< [M] Young's modulus */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32 = 132,       /**< [M] Poisson's ratio */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32 = 133,   /**< [M] elasticity damping (volume + surface) */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32 = 134,    /**< [M] bending stiffness (surface only, 0 for volume) */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32 = 135,            /**< [M] thickness (surface only, 0 for volume) */
        OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32 = 136,      /**< [M] bending damping (surface only, 0 for volume) */
    } ovphysx_tensor_type_t;

    /**
     * Descriptor for creating a tensor binding.
     * 
     * A tensor binding connects physics-object paths to a tensor type, enabling bulk
     * read/write of physics data for authored USD objects and runtime-only clones.
     * 
     * Prim selection (mutually exclusive - use ONE of these):
     *   - pattern: Path pattern such as "/World/robot*" or "/World/env[0-9]/robot". The matching
     *     rules (`*`, `?`, `**`, character classes, alternation, and the last-component rule)
     *     are on the "Path patterns" page of the ovphysx documentation
     *     (docs/read_write/path_patterns.md). `[N]` is not an index placeholder: it matches
     *     the letter N.
     *   - prim_paths: Explicit list of exact physics-object paths
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
     * Example with explicit physics-object paths:
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
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    typedef struct
    {
        ovphysx_string_t pattern;                /**< Physics-object path glob (ignored if prim_paths is set) */
        const ovphysx_string_t* prim_paths;      /**< Explicit list of exact object paths (NULL = use pattern) */
        uint32_t prim_paths_count;               /**< Number of object paths (0 = use pattern) */
        ovphysx_tensor_type_t tensor_type;       /**< Type of tensor data to bind */
    } ovphysx_tensor_binding_desc_t;

    /**
     * Tensor layout specification for DLTensor construction.
     * 
     * Use ovphysx_get_tensor_binding_spec() to get the exact dtype, rank, and shape
     * needed to allocate a layout-compatible tensor. Query native memory
     * residency separately with ovphysx_get_tensor_binding_native_device().
     * 
     * Tensor specifications by type:
     *   - Rigid body pose:     ndim=2, shape=[N, 7]
     *   - Rigid body velocity: ndim=2, shape=[N, 6]
     *   - Articulation root:   ndim=2, shape=[N, 7] or [N, 6]
     *   - Articulation links:  ndim=3, shape=[N, L, 7] or [N, L, 6]
     *   - Articulation DOF:    ndim=2, shape=[N, D]
     * 
     * Tensor dtype is tensor-type specific:
     *   - Most tensor types use float32 (kDLFloat, 32 bits, 1 lane)
     *   - Deformable element index tensors (DEFORMABLE_SIM_ELEMENT_INDICES_S32,
     *     DEFORMABLE_COLLISION_ELEMENT_INDICES_S32, SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32)
     *     use int32 (kDLInt, 32 bits, 1 lane)
     *   - OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL,
     *     OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL, and
     *     OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL use
     *     bool/uint8 (kDLUInt, 8 bits, 1 lane) as per-body/per-link byte flags.
     *   - OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8 also uses uint8
     *     (kDLUInt, 8 bits, 1 lane), but as a per-DOF enum byte (0/1/2), not a flag.
     *   Always call ovphysx_get_tensor_binding_spec() and respect
     *   the returned dtype. Do not assume float32.
     *   - Layout: row-major contiguous (C-order)
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    typedef struct
    {
        DLDataType dtype;                    /**< DLPack data type for this tensor type. Most bindings use float32, element-index tensors use int32, and disable-simulation/gravity bool bindings and the DOF drive-type enum binding use uint8. Always honor this field rather than assuming float32. */
        int32_t ndim;                        /**< Number of dimensions */
        int64_t shape[4];                    /**< Shape dimensions [dim0, dim1, dim2, 0] */
    } ovphysx_tensor_spec_t;

    /**
     * Articulation topology metadata returned by ovphysx_get_articulation_metadata().
     *
     * All fields are read at binding-creation time and remain constant for the
     * lifetime of the binding.
     *
     * String arrays (DOF names, body names, joint names) are NOT included here
     * because they are variable-length and require caller-allocated buffers. Use
     * ovphysx_articulation_get_dof_names / get_body_names / get_joint_names instead.
     *
     * @deprecated Produced only by the deprecated @ref ovphysx_get_articulation_metadata. It
     *   retires with the tensor-binding surface. Its successor is the six whole-articulation
     *   topology read attributes in ovphysx_attributes.h (OVPHYSX_ATTR_DOF_COUNT, OVPHYSX_ATTR_LINK_COUNT,
     *   OVPHYSX_ATTR_JOINT_COUNT, OVPHYSX_ATTR_FIXED_TENDON_COUNT, OVPHYSX_ATTR_SPATIAL_TENDON_COUNT,
     *   OVPHYSX_ATTR_IS_FIXED_BASE), which cover these fields field-for-field via ovphysx_read().
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


#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_LEGACY_TYPES_H
