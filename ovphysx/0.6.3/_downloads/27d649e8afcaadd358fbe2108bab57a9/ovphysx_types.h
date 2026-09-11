// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-STRING-001
 * @covers AC-1 AC-4
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-READPOOL-001
 * @covers AC-1
 * @implements REQ-CAPI-OBJECTTYPE-001
 * @covers AC-1
 */
/** @endcond */

#ifndef OVPHYSX_TYPES_H
#define OVPHYSX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dlpack/dlpack.h"

// ovstage / ovx native types. The physics output-read surface speaks ovstage's
// own types directly (no ovphysx mirror): a read group is an `ovstage_read_group_t`,
// discovery is an `ovstage_query_result_t`, attribute names/tokens are
// `ovx_string_or_token_t`, ordinals are `ovstage_ordinal_t` / `ovstage_ordinal_range_t`,
// and an attached Stage is an `ovstage_instance_t*`. ovphysx does NOT ship these
// headers: fetch the ovstage release and add its root to CMAKE_PREFIX_PATH.
#include <ovstage/ovstage_api/ovstage_api_types.h>
#include <ovx/string_types.h>
#include <ovx/types.h>  // ovx_primpath_t (interned prim-path handle for the token scope)

/**
 * String with pointer and length.
 *
 * Input strings are length-prefixed views and need not be null-terminated.
 * Strings produced by successful ovphysx calls (return values and populated
 * out-parameters) and strings delivered to callbacks always have a non-NULL
 * ptr and are null-terminated:
 * ptr[length] == '\0'. Always use length for comparisons and when accepting
 * input substring views.
 * Use OVPHYSX_LITERAL("literal") or ovphysx_cstr() for convenient construction.
 */
typedef struct ovphysx_string_t
{
    const char* ptr;   /**< Pointer to string data. See the input/output contract above */
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

    /**
     * One ovphysx handle, created by ovphysx_create_instance. Owns per-handle
     * bookkeeping and drives at most one attached USD stage. Handles share the
     * process-global simulation backend and attached stage.
     */
    typedef uint64_t ovphysx_handle_t;
    typedef uint64_t ovphysx_usd_handle_t;
    typedef uint64_t ovphysx_attribute_binding_handle_t;
    typedef uint64_t ovphysx_write_map_handle_t;
    typedef uint64_t ovphysx_read_map_handle_t;
    typedef uint64_t ovphysx_op_index_t;
    typedef uint64_t ovphysx_tensor_binding_handle_t; /**< @deprecated Tensor-binding handle. Use ovphysx_read / ovphysx_write sessions instead. */
    typedef uint64_t ovphysx_contact_binding_handle_t;
    typedef uint64_t ovphysx_query_handle_t; /**< Physics-output query (ovphysx_query). */
    typedef uint64_t ovphysx_read_handle_t;         /**< Physics-output read session (ovphysx_read). */
    typedef uint64_t ovphysx_write_handle_t;        /**< App-to-physics write session (ovphysx_write). */

    /**
     * @brief How a tensor type may be written, asked of the API rather than read from docs.
     *
     * Derived from the backend's actual getter/setter support, NOT from the `(READ-ONLY)`
     * markers in this header's comments. Those are an incomplete index, and their absence
     * never implied writable.
     */
    typedef enum
    {
        OVPHYSX_WRITABILITY_UNCLASSIFIED = 0, /**< not classified. The write API rejects it (a gap to close, not a licence) */
        OVPHYSX_WRITABILITY_WRITABLE = 1,     /**< writable with no precondition beyond the object existing */
        OVPHYSX_WRITABILITY_CONDITIONAL = 2,  /**< writable only under a stated condition. The write reports an unmet one */
        OVPHYSX_WRITABILITY_WRITE_ONLY = 3,   /**< control input consumed then cleared each step (forces, wrenches), no read-back */
        OVPHYSX_WRITABILITY_READ_ONLY = 4,    /**< readable only. Writing is an error naming the type, not a silent no-op */
    } ovphysx_writability_t;
    typedef uint64_t ovphysx_sdf_view_handle_t;

    /**
     * Sentinel value representing an invalid/null ovphysx instance handle.
     * Valid handles are nonzero, so 0 is reserved to indicate "no handle" or "invalid handle".
     * Use this when a handle parameter is required but no valid instance is available.
     */
    #define OVPHYSX_INVALID_HANDLE 0

    /**
     * Sentinel value for ovphysx_wait_op to wait for all operations submitted up to the call.
     * Use this to ensure all outstanding operations have completed.
     */
    #define OVPHYSX_OP_INDEX_ALL UINT64_MAX

    /*--------------------------------------------------*/
    /* Timeout type and values */
    /*--------------------------------------------------*/

    /**
     * Nanosecond timeout used by timeout-bearing OVPhysX APIs.
     *
     * Values are nanoseconds. OVPHYSX_TIMEOUT_POLL performs one non-blocking
     * readiness check, while OVPHYSX_TIMEOUT_INFINITE waits until completion.
     * This local uint64_t alias is byte-identical to the shared ovx_timeout_ns_t
     * so that it can migrate to that type without an ABI change.
     */
    typedef uint64_t ovphysx_timeout_t;

    #define OVPHYSX_TIMEOUT_POLL ((ovphysx_timeout_t)0)
    #define OVPHYSX_TIMEOUT_INFINITE ((ovphysx_timeout_t)UINT64_MAX)

    /*--------------------------------------------------*/
    /* Physics output read (ovstage) */
    /*--------------------------------------------------*/

    /**
     * Simulated object type selected by ovphysx_query(). This is the
     * engine's simulated type, not a USD schema predicate.
     *
     * @note This is a separate enum domain from the TensorBindings
     *       OVPHYSX_OBJECT_TYPE_* selector. In particular,
     *       OVPHYSX_OBJECT_ARTICULATION (9) must not be confused with
     *       OVPHYSX_OBJECT_TYPE_ARTICULATION (2).
     */
    typedef enum
    {
        OVPHYSX_OBJECT_RIGID_BODY = 0,         /**< dynamic rigid bodies (standalone + point-instancer instances) */
        OVPHYSX_OBJECT_ARTICULATION_LINK = 1,  /**< articulation link body transforms */
        OVPHYSX_OBJECT_ARTICULATION_JOINT = 2, /**< articulation joint state (per-axis, one array group per joint) */
        OVPHYSX_OBJECT_VEHICLE_WHEEL = 3,      /**< vehicle wheel transforms */
        OVPHYSX_OBJECT_DEFORMABLE_VOLUME = 4,  /**< volume deformable meshes (points / velocities) */
        OVPHYSX_OBJECT_DEFORMABLE_SURFACE = 5, /**< surface deformable meshes */
        OVPHYSX_OBJECT_PARTICLE_SET = 6,       /**< particle sets */
        OVPHYSX_OBJECT_FIXED_TENDON = 7,       /**< articulation fixed tendons (prim: joint carrying the root axis) */
        OVPHYSX_OBJECT_SPATIAL_TENDON = 8,     /**< articulation spatial tendons (prim: link carrying the root attachment) */
        OVPHYSX_OBJECT_ARTICULATION = 9,        /**< whole articulations (prim: the articulation-root API prim) */
        OVPHYSX_OBJECT_DEFORMABLE_MATERIAL = 10, /**< deformable materials (prim: the bound Material prim) */
    } ovphysx_sim_object_type_t;

    /**
     * Output query scope.
     *
     * @note OVPHYSX_SCOPE_ACTIVE is SINGLE-FRAME: the active set is recomputed
     *       every step, so a query opened with it (and the groups read from it)
     *       is valid only for the step it was opened against. Re-query each frame.
     *       OVPHYSX_SCOPE_ALL is stable across steps until a structural change
     *       (object add/remove, instancer instance-count change).
     *
     *       For OVPHYSX_OBJECT_ARTICULATION, ACTIVE includes an articulation
     *       when any link is reported by its owning scene's active-actor set.
     *       If that scene cannot report active actors, articulation awake state
     *       is the fallback. DirectGPU currently disables sleeping, so its
     *       whole-articulation ACTIVE membership is equivalent to ALL.
     */
    typedef enum
    {
        OVPHYSX_SCOPE_ALL = 0,    /**< every object of the type */
        OVPHYSX_SCOPE_ACTIVE = 1, /**< only objects the solver moved last step */
    } ovphysx_object_scope_t;

    /**
     * Canonical physics-output attribute names (semantic, not USD attribute
     * names). Pass any of these to ovphysx_read(). Which names a type
     * produces is documented in the ovstage usage guide.
     */
    #define OVPHYSX_ATTR_POSITION          "position"          /* vec3 f32 (world) */
    #define OVPHYSX_ATTR_ORIENTATION       "orientation"       /* quat f32 xyzw (world) */
    #define OVPHYSX_ATTR_LINEAR_VELOCITY   "linearVelocity"    /* vec3 f32 */
    #define OVPHYSX_ATTR_ANGULAR_VELOCITY  "angularVelocity"   /* vec3 f32 */
    #define OVPHYSX_ATTR_LINEAR_ACCELERATION  "linearAcceleration"  /* vec3 f32 */
    #define OVPHYSX_ATTR_ANGULAR_ACCELERATION "angularAcceleration" /* vec3 f32 */
    /* Body properties. Simulation INPUTS, so these columns are HOST-resident (kDLCPU) even on a
       DirectGPU scene (there is no device copy of an authored value to read) and they have no
       point-instancer array form. centerOfMass* is in the body's LOCAL frame, unlike
       position/orientation which are world. */
    #define OVPHYSX_ATTR_MASS              "mass"              /* f32 */
    #define OVPHYSX_ATTR_INVERSE_MASS      "inverseMass"       /* f32 */
    #define OVPHYSX_ATTR_INERTIA           "inertia"           /* mat3 f32 (9, COM frame) */
    #define OVPHYSX_ATTR_INVERSE_INERTIA   "inverseInertia"    /* mat3 f32 (9, COM frame) */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_POSITION    "centerOfMassPosition"    /* vec3 f32 (local) */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_ORIENTATION "centerOfMassOrientation" /* quat f32 xyzw (local) */
    /* Whole-articulation ROOT-LINK state, for OVPHYSX_OBJECT_ARTICULATION. Named apart from
       OVPHYSX_ATTR_POSITION and friends because the row is keyed by the prim carrying
       PhysicsArticulationRootAPI, which may be an ancestor Xform rather than the root link itself.
       Unlike every other type, the value is therefore not this row's own prim's pose. */
    #define OVPHYSX_ATTR_ROOT_POSITION         "rootPosition"        /* vec3 f32 (scene world) */
    #define OVPHYSX_ATTR_ROOT_ORIENTATION      "rootOrientation"     /* quat f32 xyzw (scene world) */
    #define OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY  "rootLinearVelocity"  /* vec3 f32 */
    #define OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY "rootAngularVelocity" /* vec3 f32 */
    /* Whole-articulation centres of mass. WORLD uses the same scene-world frame as
       OVPHYSX_ATTR_ROOT_POSITION and is independent of TensorBindings subspace roots. LOCAL uses
       the root link's centre-of-mass (mass) frame, not its actor/prim frame. */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_WORLD "centerOfMassWorld" /* vec3 f32 (scene world) */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_LOCAL "centerOfMassLocal" /* vec3 f32 (root-link mass frame) */
    /* Whole-articulation inverse dynamics, row-major flattened into dtype.lanes. The width depends on the
       articulation's topology, not on the attribute, so a read emits one group per cohort of
       structurally identical articulations and lanes is read per group. JACOBIAN carries its two
       dimensions in the companion JACOBIAN_SHAPE column; the others recover theirs from lanes.
       Generalized joint coordinates follow the authored USD body relationship: their sign is +1
       when body0 is the articulation parent and -1 when body1 is the parent. With S_dof containing
       those signs and T=S_dof for a fixed base or T=diag(I6,S_dof) for a floating base, the returned
       quantities are J=J_physx*T, M=T*M_physx*T, c=T*c_physx, g=T*g_physx, and [A|b]=
       [A_physx*T|b_physx] for the packed centroidal matrix and bias. Angular generalized-coordinate
       dimensions use radians and receive no degree conversion; prismatic and floating-translation
       dimensions retain their linear units. */
    #define OVPHYSX_ATTR_JACOBIAN            "jacobian"           /* f32 [rows*cols] */
    #define OVPHYSX_ATTR_JACOBIAN_SHAPE      "jacobianShape"      /* int32, 2 lanes: rows, cols */
    #define OVPHYSX_ATTR_MASS_MATRIX         "massMatrix"         /* f32 [M*M] */
    #define OVPHYSX_ATTR_CORIOLIS_FORCE      "coriolisForce"      /* f32 [M] */
    #define OVPHYSX_ATTR_GRAVITY_FORCE       "gravityForce"       /* f32 [M] */
    #define OVPHYSX_ATTR_CENTROIDAL_MOMENTUM "centroidalMomentum" /* f32, floating base only */
    #define OVPHYSX_ATTR_DISABLE_GRAVITY    "disableGravity"    /* uint8, nonzero = disabled */
    #define OVPHYSX_ATTR_DISABLE_SIMULATION "disableSimulation" /* uint8, nonzero = disabled */
    /* Per-SHAPE properties: one value per shape, padded to the widest selected object in the read, so
       `dtype.lanes` is that width and a row is shapeCount real values then zeros. Read
       OVPHYSX_ATTR_SHAPE_COUNT to find the end. The padding is zero-filled, but 0.0 is a legal
       offset, so it is not a terminator. Host-resident like the properties above. Only the FIRST
       material of a shape is reported. */
    #define OVPHYSX_ATTR_STATIC_FRICTION   "staticFriction"    /* f32 [per shape] */
    #define OVPHYSX_ATTR_DYNAMIC_FRICTION  "dynamicFriction"   /* f32 [per shape] */
    #define OVPHYSX_ATTR_RESTITUTION       "restitution"       /* f32 [per shape] */
    #define OVPHYSX_ATTR_CONTACT_OFFSET    "contactOffset"     /* f32 [per shape] */
    #define OVPHYSX_ATTR_REST_OFFSET       "restOffset"        /* f32 [per shape] */
    #define OVPHYSX_ATTR_SHAPE_COUNT       "shapeCount"        /* int32, one per body or whole articulation */
    #define OVPHYSX_ATTR_POINTS            "points"            /* vec3 f32 [array] */
    #define OVPHYSX_ATTR_VELOCITIES        "velocities"        /* vec3 f32 [array] */
    #define OVPHYSX_ATTR_REST_POINTS       "restPoints"        /* vec3 f32 [array] (sim-mesh-local) */
    #define OVPHYSX_ATTR_SIM_ELEMENT_INDICES "simElementIndices" /* int32 [array] (4 per tet / 3 per tri) */
    #define OVPHYSX_ATTR_COLLISION_ELEMENT_INDICES "collisionElementIndices" /* int32 [array] (4 per tet) */
    /* Deformable material properties: one row per material prim, host-resident. The `deformable`
       prefix disambiguates from the per-SHAPE rigid dynamicFriction above, which is a different
       quantity with a different shape. */
    #define OVPHYSX_ATTR_DEFORMABLE_DYNAMIC_FRICTION   "deformableDynamicFriction"   /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS     "deformableYoungsModulus"     /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_POISSONS_RATIO     "deformablePoissonsRatio"     /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING "deformableElasticityDamping" /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS  "deformableBendingStiffness"  /* f32, surface only */
    #define OVPHYSX_ATTR_DEFORMABLE_THICKNESS          "deformableThickness"         /* f32, surface only */
    #define OVPHYSX_ATTR_DEFORMABLE_BENDING_DAMPING    "deformableBendingDamping"    /* f32, surface only */

    /* Articulation DOF attributes: one value per ENABLED AXIS of the joint prim. Angular axes are
       reported in DEGREES (what USD authored) and signed quantities carry the joint's body-order
       sign. The `joint` prefix disambiguates from the per-SHAPE rigid staticFriction /
       dynamicFriction above, which are different quantities. */
    #define OVPHYSX_ATTR_JOINT_POSITION    "jointPosition"     /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_VELOCITY    "jointVelocity"     /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_POSITION_TARGET "jointPositionTarget" /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_VELOCITY_TARGET "jointVelocityTarget" /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_ACTUATION_FORCE "jointActuationForce" /* f32 [array, per-axis] */
    /* The link's incoming joint force resolved onto this DOF's own axis. Read-only. */
    #define OVPHYSX_ATTR_JOINT_PROJECTED_FORCE "jointProjectedForce" /* f32 [array, per-axis], read-only */
    /* DOF properties: simulation INPUTS, so HOST-resident (kDLCPU) columns even on a DirectGPU
       scene. One read can mix a kDLCUDA jointPosition with a kDLCPU jointStiffness. */
    #define OVPHYSX_ATTR_JOINT_STIFFNESS   "jointStiffness"    /* f32: drive effort per unit of position error */
    #define OVPHYSX_ATTR_JOINT_DAMPING     "jointDamping"      /* f32: drive effort per unit of rate error */
    #define OVPHYSX_ATTR_JOINT_LIMIT       "jointLimit"        /* f32 x2 (low, high): one interval, FLT_MAX when free */
    #define OVPHYSX_ATTR_JOINT_MAX_VELOCITY "jointMaxVelocity" /* f32: the JOINT's rate bound */
    #define OVPHYSX_ATTR_JOINT_MAX_FORCE   "jointMaxForce"     /* f32: the drive's effort bound */
    #define OVPHYSX_ATTR_JOINT_ARMATURE    "jointArmature"     /* f32: added rotor inertia on the axis */
    #define OVPHYSX_ATTR_JOINT_STATIC_FRICTION  "jointStaticFriction"  /* f32: break-away effort */
    #define OVPHYSX_ATTR_JOINT_DYNAMIC_FRICTION "jointDynamicFriction" /* f32: sliding effort */
    #define OVPHYSX_ATTR_JOINT_VISCOUS_FRICTION "jointViscousFriction" /* f32: effort per unit rate, a
                                                                          coefficient, unlike the two efforts above */
    #define OVPHYSX_ATTR_JOINT_SPEED_EFFORT_GRADIENT "jointSpeedEffortGradient" /* f32: drive envelope, rate per unit effort */
    #define OVPHYSX_ATTR_JOINT_MAX_ACTUATOR_VELOCITY "jointMaxActuatorVelocity" /* f32: the ACTUATOR's rate bound,
                                                                                   not the joint's above */
    #define OVPHYSX_ATTR_JOINT_VELOCITY_DEPENDENT_RESISTANCE                                                           \
        "jointVelocityDependentResistance" /* f32: drive envelope, resisting effort per unit rate */
    #define OVPHYSX_ATTR_JOINT_DRIVE_TYPE  "jointDriveType"    /* uint8 [array, per-axis]: 0 none, 1 force, 2 accel */
    /* On OVPHYSX_OBJECT_ARTICULATION_LINK: the spatial force the inbound joint exerts on the link,
       spelled as in PxArticulationCache::linkIncomingJointForce. */
    #define OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE "linkIncomingJointForce" /* f32 x6: force xyz, torque xyz, joint frame */
    /* Tendon properties: authoring-time values, so they change only when the tendon does. */
    #define OVPHYSX_ATTR_TENDON_STIFFNESS       "tendonStiffness"       /* f32 */
    #define OVPHYSX_ATTR_TENDON_DAMPING         "tendonDamping"         /* f32 */
    #define OVPHYSX_ATTR_TENDON_LIMIT_STIFFNESS "tendonLimitStiffness"  /* f32 */
    #define OVPHYSX_ATTR_TENDON_LIMIT           "tendonLimit"           /* f32 x2 (low, high), fixed tendons only */
    #define OVPHYSX_ATTR_TENDON_REST_LENGTH     "tendonRestLength"      /* f32, fixed tendons only */
    #define OVPHYSX_ATTR_TENDON_OFFSET          "tendonOffset"          /* f32 */

    /* Write-only control inputs (ADR-0012). Accepted by ovphysx_write, never emitted by
       ovphysx_read: the solver consumes and clears them each step, so there is no stored
       value to report back. */
    #define OVPHYSX_ATTR_FORCE                  "force"                 /* vec3 f32, at the COM [write-only] */
    #define OVPHYSX_ATTR_WRENCH                 "wrench"                /* 9 f32: force, torque, world point [write-only] */

    /* Vehicle wheel controls (ADR-0012). Inputs the solver consumes, so writable on
       OVPHYSX_OBJECT_VEHICLE_WHEEL where position / orientation are not. A wheel's pose
       is composed each step from the chassis, its suspension and its steer angle. */
    #define OVPHYSX_ATTR_DRIVE_TORQUE           "driveTorque"           /* f32, per wheel */
    #define OVPHYSX_ATTR_BRAKE_TORQUE           "brakeTorque"           /* f32, per wheel */
    #define OVPHYSX_ATTR_STEER_ANGLE            "steerAngle"            /* f32 radians, per wheel */

    /**
     * The physics output-read surface uses ovstage's own types directly rather than
     * an ovphysx mirror (so a read group feeds straight back into the ovstage write
     * path with no repack and no translation layer):
     *
     *   - a read group is an `ovstage_read_group_t` (from ovstage_api_types.h):
     *     `data.tensors[0..tensor_count)` are borrowed DLTensors (tuple width in
     *     `dtype.lanes`), `prims.list` is the interned prim set, `attribute` is the
     *     interned EMITTED attribute token, `semantic` is the authored USD role
     *     (`ovstage_attribute_semantic_t`). For this physics-output path
     *     `is_delete == false`, `prims.offset == 0`, and a point-instancer always
     *     emits the FULL instance array by-index (`data.index_map == NULL`).
     *     Slots with no live point-instancer body are zero-filled in every emitted
     *     array. In particular, an all-zero `orientations` quaternion is the
     *     absent-slot marker; live orientations are normalized and never all zero.
     *
     *     `is_array` decides how many tensors a group carries, and reading only
     *     `tensors[0]` is wrong for half of them:
     *       * `is_array == false` (a FIXED group, e.g. rigid-body position) stacks
     *         every prim into ONE tensor. `tensor_count == 1`, and row i belongs
     *         to prim i of `prims.list`.
     *       * `is_array == true` (an ARRAY group, e.g. joint state, point-instancer
     *         positions, deformable points) carries ONE TENSOR PER PRIM.
     *         `tensor_count == prims.count`, and `tensors[i]` is prim i's own
     *         variable-length array. A joint read of 16k joints is ONE group with
     *         16k tensors, not 16k groups. Taking `tensors[0]` yields the first
     *         prim's values and no error.
     *     Element type is per attribute, in `tensors[i].dtype`. Do not assume f32.
     *     Most columns are `{kDLFloat, 32}`. The boolean flags (disableGravity,
     *     disableSimulation) are `{kDLUInt, 8}` and shapeCount is `{kDLInt, 32}`.
     *     `dtype.lanes` is the tuple width, and for a per-shape column it is the
     *     widest selected object in THAT read, so it varies between reads of the same
     *     attribute. Do not cache it.
     *   - residency is per attribute too. On a DirectGPU scene the simulated outputs
     *     (pose, velocity, acceleration, and whole-articulation centerOfMassWorld /
     *     centerOfMassLocal) are `kDLCUDA`, while authored body properties (mass,
     *     inertia, centerOfMassPosition / centerOfMassOrientation, and the flags) are
     *     `kDLCPU`: PhysX never writes those authored values back, so there is no
     *     device copy to read. One read can therefore hand back both. Branch on
     *     `tensors[i].device.device_type`.
     *   - discovery is an `ovstage_query_result_t` (`attributes` is the interned token
     *     array, and `total_prim_count == 0` is the valid empty-match case).
     *   - attribute names are `ovx_string_or_token_t`. Pass a string (e.g.
     *     OVPHYSX_ATTR_POSITION) or an interned token from discovery, no round-trip.
     *
     * The queried `ovphysx_sim_object_type_t` is NOT carried on the group: a read is
     * opened over one type, so every group belongs to the type the caller passed to
     * ovphysx_query. Group lifetime is producer-owned (see ovphysx_fetch_read_next).
     */

    /*--------------------------------------------------*/
    /* Log level enum */
    /*--------------------------------------------------*/

    typedef enum
    {
        OVPHYSX_LOG_DEFAULT = 0, /**< Set-level sentinel: restore the library default (WARNING). Never delivered */
        OVPHYSX_LOG_VERBOSE = 1, /**< All messages including verbose/debug (maps to Carbonite kLevelVerbose) */
        OVPHYSX_LOG_INFO    = 2, /**< Info, warnings, and errors */
        OVPHYSX_LOG_WARNING = 3, /**< Warnings and errors (library default) */
        OVPHYSX_LOG_ERROR   = 4, /**< Error messages only */
        OVPHYSX_LOG_NONE    = 5  /**< Set-level sentinel: no logging. Never delivered */
    } ovphysx_log_level_t;

    /**
     * Log callback function type.
     *
     * Called for each message that passes the callback's severity and channel
     * filters. Invocations for one registration are serialized.
     *
     * @param severity The ovphysx_log_level_t severity of the message.
     * @param message  UTF-8 message. Valid only during the callback and guaranteed
     *                 null-terminated at message.ptr[message.length].
     * @param channel  UTF-8 source/channel. Valid only during the callback and
     *                 guaranteed null-terminated at channel.ptr[channel.length].
     *                 An unavailable channel is delivered as {"", 0}.
     * @param timestamp Seconds since the Unix epoch. Always greater than zero.
     * @param user_data Opaque pointer passed during registration.
     * @note A callback implemented in C++ must not allow exceptions to cross
     *       this C ABI boundary.
     */
    typedef void (*ovphysx_log_callback_t)(
        ovphysx_log_level_t severity,
        ovphysx_string_t message,
        ovphysx_string_t channel,
        double timestamp,
        void* user_data);

    /*--------------------------------------------------*/
    /* PhysX object type enum                           */
    /*--------------------------------------------------*/

    /**
     * Identifies the type of a path-bound or process-global PhysX object.
     *
     * Used with ovphysx_get_physx_ptr() to retrieve raw PhysX SDK pointers.
     * The named constants below cover common runtime object types. Unknown
     * values return NULL for unrecognized path/type combinations.
     *
     * | Enum value                         | PhysX SDK C++ type                            |
     * |------------------------------------|-----------------------------------------------|
     * | OVPHYSX_PHYSX_TYPE_SCENE           | physx::PxScene                                |
     * | OVPHYSX_PHYSX_TYPE_MATERIAL        | physx::PxMaterial                             |
     * | OVPHYSX_PHYSX_TYPE_SHAPE           | physx::PxShape                                |
     * | OVPHYSX_PHYSX_TYPE_COMPOUND_SHAPE  | Opaque compound-shape wrapper (see note)      |
     * | OVPHYSX_PHYSX_TYPE_ACTOR           | physx::PxRigidDynamic/PxRigidStatic           |
     * | OVPHYSX_PHYSX_TYPE_JOINT           | physx::PxJoint (standalone joints)            |
     * | OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT    | CustomPhysXJoint (plugin custom joints)       |
     * | OVPHYSX_PHYSX_TYPE_ARTICULATION    | physx::PxArticulationReducedCoordinate        |
     * | OVPHYSX_PHYSX_TYPE_LINK            | physx::PxArticulationLink                     |
     * | OVPHYSX_PHYSX_TYPE_LINK_JOINT      | physx::PxArticulationJointReducedCoordinate   |
     * | OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM | physx::PxPBDParticleSystem                    |
     * | OVPHYSX_PHYSX_TYPE_PARTICLE_SET    | physx::PxParticleBuffer                       |
     * | OVPHYSX_PHYSX_TYPE_PHYSICS         | physx::PxPhysics                              |
     *
     * Note: PHYSICS names the process-global PxPhysics object and is selected
     * with a zero-length path. Both `{ NULL, 0 }` and `{ "", 0 }` are accepted.
     * Every other type requires a non-empty physics-object path.
     *
     * See @ref ovphysx_object_type_t for the matching high-level classification.
     *
     * COMPOUND_SHAPE returns an internal compound-shape wrapper. Use the C++
     * helper matching the SDK build to access the underlying physx::PxShape
     * pointers.
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

    /**
     * @brief High-level object classification for prim paths (TensorAPI-level).
     *
     * Mirrors omni::physics::tensors::ObjectType. Returned by
     * @ref ovphysx_get_object_type to let callers tell rigid bodies,
     * articulations, articulation links/joints, articulation root links, and
     * maximal-coordinate (standalone) and plugin-registered custom joints apart
     * at a path without inspecting the PhysX SDK pointer directly.
     *
     * @note The @c JOINT / @c ARTICULATION_JOINT / @c CUSTOM_JOINT split mirrors
     *       @ref ovphysx_physx_type_t's @c OVPHYSX_PHYSX_TYPE_JOINT,
     *       @c OVPHYSX_PHYSX_TYPE_LINK_JOINT, and @c OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT.
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

    /**
     * @brief Bit flags for @ref ovphysx_articulation_update_kinematic.
     *
     * Mirrors PxArticulationKinematicFlag::Enum. Flags may be OR'd.
     */
    typedef enum
    {
        OVPHYSX_ARTICULATION_KINEMATIC_POSITION = 1u << 0, /**< Recompute link transforms from joint positions + root pose */
        OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY = 1u << 1, /**< Recompute link velocities from joint velocities + root velocity */
    } ovphysx_articulation_kinematic_flag_t;

    /*--------------------------------------------------*/
    /* PhysX object change notifications                */
    /*--------------------------------------------------*/

    /**
     * Subscription ID returned by ovphysx_subscribe_object_changes().
     *
     * Used to identify a subscription for later unsubscribe. Treat as opaque.
     */
    typedef uint64_t ovphysx_subscription_id_t;

    /**
     * Sentinel value for an invalid / unset subscription ID.
     *
     * Valid subscription IDs are never equal to this value. After calling
     * ovphysx_subscribe_object_changes(), check the returned status code first;
     * only use the out_subscription value when status == OVPHYSX_API_SUCCESS.
     */
    #define OVPHYSX_INVALID_SUBSCRIPTION_ID UINT64_MAX

    /**
     * Notification when a PhysX object is created during simulation.
     *
     * Fires AFTER the object exists, so it is safe to call ovphysx_get_physx_ptr()
     * for prim_path / type from a deferred handler.
     *
     * Only fires for creations triggered by stage edits during simulation. The
     * initial object population from ovstage attach/update is NOT notified:
     * the caller already has that state from its setup code. See
     * ovphysx_subscribe_object_changes() for the full lifecycle contract.
     *
     * @param prim_path Absolute USD prim path of the created object. The
     *                  underlying storage is owned by ovphysx and only valid for
     *                  the duration of the callback. Copy it to retain it.
     *                  ptr is non-NULL and ptr[length] is '\0'.
     * @param type      The PhysX object type of the created object.
     * @param user_data Opaque pointer passed during subscription.
     */
    typedef void (*ovphysx_object_created_fn)(
        ovphysx_string_t prim_path,
        ovphysx_physx_type_t type,
        void* user_data);

    /**
     * Notification when a PhysX object is about to be destroyed during simulation.
     *
     * Fires BEFORE the object is destroyed. Drop any cached pointer for
     * prim_path / type at this point. Do NOT call release() on it (ovphysx
     * owns the lifetime).
     *
     * Only fires for destructions that occur during simulation. Bulk teardown
     * (e.g. ovphysx_reset_stage()) is delivered via ovphysx_all_objects_destroyed_fn
     * instead, not as N individual destruction notifications.
     *
     * @param prim_path Absolute USD prim path of the soon-to-be-destroyed object.
     *                  Same lifetime and null-termination rules as
     *                  ovphysx_object_created_fn.
     * @param type      The PhysX object type that is going away.
     * @param user_data Opaque pointer passed during subscription.
     */
    typedef void (*ovphysx_object_destroyed_fn)(
        ovphysx_string_t prim_path,
        ovphysx_physx_type_t type,
        void* user_data);

    /**
     * Notification when ALL PhysX objects are about to be destroyed in bulk.
     *
     * Fires BEFORE the bulk teardown (e.g. on ovphysx_reset_stage()). Subscribers
     * should flush their entire pointer cache. No per-object destruction events
     * are delivered for this teardown.
     *
     * @param user_data Opaque pointer passed during subscription.
     */
    typedef void (*ovphysx_all_objects_destroyed_fn)(void* user_data);

    /**
     * Callback set passed to ovphysx_subscribe_object_changes().
     *
     * Any of the function-pointer fields may be NULL. ovphysx skips a NULL
     * field rather than invoking it. The caller does NOT need to keep this
     * struct alive after the subscribe call returns, because ovphysx copies the
     * relevant state internally.
     *
     * Threading: callbacks may fire from internal worker threads during
     * ovphysx_step(), ovphysx_step_sync(), or ovphysx_reset_stage().
     * Do NOT call other ovphysx APIs from inside a callback (re-entrancy /
     * deadlock risk). Defer follow-up work until the triggering synchronous
     * call returns, or until the next ovphysx_wait_op() returns for async work.
     */
    typedef struct ovphysx_object_change_callbacks_t
    {
        /** Fired AFTER an object is created (NULL = skip). */
        ovphysx_object_created_fn        on_object_created;

        /** Fired BEFORE an object is destroyed (NULL = skip). */
        ovphysx_object_destroyed_fn      on_object_destroyed;

        /** Fired BEFORE a bulk teardown (NULL = skip). */
        ovphysx_all_objects_destroyed_fn on_all_objects_destroyed;

        /** Opaque pointer passed unchanged to every callback. */
        void* user_data;
    } ovphysx_object_change_callbacks_t;

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
        OVPHYSX_API_BUFFER_TOO_SMALL = 6,    /**< Caller-supplied buffer is too small. Check out_required_size */
        OVPHYSX_API_DEVICE_MISMATCH = 7,     /**< Tensor device cannot be used or staged for this binding/policy */
        OVPHYSX_API_GPU_NOT_AVAILABLE = 8,   /**< GPU requested but not available or CUDA init failed */
        OVPHYSX_API_END_OF_ITERATION = 9,    /**< Iterator exhausted (e.g. ovphysx_fetch_read_next past the last group). Not an error */
        OVPHYSX_API_INVALID_STATE = 10,       /**< Operation is not valid in the instance's current lifecycle state */
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
     *   - pattern: Glob pattern like "/World/robot*" or "/World/env[N]/robot"
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
     *   retires with the tensor-binding surface. No non-binding successor exists yet.
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
     *     - Use this to ensure the caller's GPU kernels have finished writing to buffers
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
     * Contact event header - describes one contact pair.
     *
     * ABI-stable contact header returned by ovphysx.
     * Each header references a slice of the contact data array
     * (contactDataOffset .. contactDataOffset + numContactData).
     *
     * The identity fields (actor0/1, collider0/1) hold an opaque
     * omni::physics::parse::ObjectKey.handle, a runtime-assigned identity, not
     * a uint64-encoded SdfPath. There is no client-side bit-cast that reproduces
     * or compares against one. Resolve them with
     * @ref ovphysx_scene_query_get_paths_from_ids instead.
     */
    typedef struct ovphysx_contact_event_header_t
    {
        int32_t  type;                    /**< 0 = found, 1 = lost, 2 = persist */
        uint64_t attachHandle;            /**< Attach that reported the contact. Nonzero for every live
                                               attach, including one with no backing USD stage. */
        uint64_t actor0;                  /**< Actor 0 identity (opaque ObjectKey.handle) */
        uint64_t actor1;                  /**< Actor 1 identity (opaque ObjectKey.handle) */
        uint64_t collider0;               /**< Collider 0 identity (opaque ObjectKey.handle) */
        uint64_t collider1;               /**< Collider 1 identity (opaque ObjectKey.handle) */
        uint32_t contactDataOffset;       /**< Index into the contact data array */
        uint32_t numContactData;          /**< Number of contact points for this pair */
        uint32_t frictionAnchorsDataOffset; /**< Index into the friction anchors array */
        uint32_t numfrictionAnchorsData;  /**< Number of friction anchors for this pair */
        uint32_t protoIndex0;             /**< Point instancer index (0xFFFFFFFF if N/A) */
        uint32_t protoIndex1;             /**< Point instancer index (0xFFFFFFFF if N/A) */
    } ovphysx_contact_event_header_t;

    /**
     * Per-contact-point data returned by ovphysx.
     *
     * position, normal, and impulse are float[3] in world space.
     *
     * The identity fields (material0/1) hold an opaque
     * omni::physics::parse::ObjectKey.handle, a runtime-assigned identity, not
     * a uint64-encoded SdfPath. There is no client-side bit-cast that reproduces
     * or compares against one. Resolve them with
     * @ref ovphysx_scene_query_get_paths_from_ids instead. They are 0 when the
     * reporting attach is no longer live.
     */
    typedef struct ovphysx_contact_point_t
    {
        float    position[3];             /**< Contact position (world space) */
        float    normal[3];               /**< Contact normal */
        float    impulse[3];              /**< Contact impulse (divide by dt for force) */
        float    separation;              /**< Contact separation distance */
        uint32_t faceIndex0;              /**< Triangle mesh face index for collider 0 */
        uint32_t faceIndex1;              /**< Triangle mesh face index for collider 1 */
        uint64_t material0;               /**< Material 0 identity (opaque ObjectKey.handle) */
        uint64_t material1;               /**< Material 1 identity (opaque ObjectKey.handle) */
    } ovphysx_contact_point_t;

    /**
     * Friction anchor data returned by ovphysx.
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

    /*--------------------------------------------------*/
    /* Typed config system                              */
    /*--------------------------------------------------*/

    /** Config key type discriminator - selects which key/value union members are valid. */
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
        OVPHYSX_CONFIG_OMNIPVD_OUTPUT_ENABLED,            /**< /physics/omniPvdOutputEnabled */
        OVPHYSX_CONFIG_NVTX_ENABLED,                      /**< /physics/nvtxEnabled: emit NVTX ranges for
                                                               capture with Nsight Systems. Equivalent to
                                                               setting OVPHYSX_NVTX=1 in the environment. */
        OVPHYSX_CONFIG_OMNIPVD_RECORDING_CAPABLE,          /**< /physics/omniPvdRecordingCapable */
        OVPHYSX_CONFIG_BOOL_COUNT
    } ovphysx_config_bool_t;

    /** Int32 config keys. Value type: int32_t. */
    typedef enum ovphysx_config_int32_t
    {
        OVPHYSX_CONFIG_NUM_THREADS,          /**< /physics/numThreads */
        OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, /**< /physics/sceneMultiGPUMode (0=disabled, 1=all, 2=skip-first).
                                                  Used only when active_cuda_gpus is empty */
        OVPHYSX_CONFIG_OMNIPVD_TCP_PORT,       /**< /physics/omniPvdTcpPort */
        OVPHYSX_CONFIG_OMNIPVD_TCP_TIMEOUT_MS, /**< /physics/omniPvdTcpTimeoutMs */
        OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, /**< /physics/ovstageReadPoolMaxMB: device read-buffer pool
                                                      retention budget in MiB. 0 or less disables the pool (default 256) */
        OVPHYSX_CONFIG_INT32_COUNT
    } ovphysx_config_int32_t;

    /** Float config keys (reserved for future use). Value type: float. */
    typedef enum ovphysx_config_float_t
    {
        OVPHYSX_CONFIG_FLOAT_COUNT
    } ovphysx_config_float_t;

    /** String config keys. Value type: ovphysx_string_t. */
    typedef enum ovphysx_config_string_t
    {
        OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, /**< /persistent/physics/omniPvdOvdRecordingDirectory */
        OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, /**< /UJITSO/datastore/localCachePath: application-provided dir for the local cooked-collider cache. Unset cooks to a process-private temp dir (nothing persists) */
        OVPHYSX_CONFIG_OMNIPVD_TRANSPORT,                /**< /physics/omniPvdTransport */
        OVPHYSX_CONFIG_OMNIPVD_TCP_ADDRESS,              /**< /physics/omniPvdTcpAddress */
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

    /** Destination transport for a late OmniPVD recording. */
    typedef enum ovphysx_omnipvd_transport_t
    {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE = 0, /**< Write one exact .ovd file path. */
        OVPHYSX_OMNIPVD_TRANSPORT_TCP = 1   /**< Connect to one TCP listener. */
    } ovphysx_omnipvd_transport_t;

    /**
     * Destination for ovphysx_start_recording().
     *
     * FILE requires a non-empty file_path and empty/zero TCP fields. TCP
     * requires a non-empty tcp_address, tcp_port in 1..65535, and an empty
     * file_path. String views are borrowed only for the synchronous call and
     * must not contain embedded NUL bytes. A zero TCP timeout leaves the socket
     * send timeout at its platform default.
     */
    typedef struct ovphysx_omnipvd_destination_t
    {
        ovphysx_omnipvd_transport_t transport;
        ovphysx_string_t file_path;
        ovphysx_string_t tcp_address;
        uint32_t tcp_port;
        int32_t tcp_timeout_ms;
    } ovphysx_omnipvd_destination_t;

    /*--------------------------------------------------*/
    /* Instance creation                                */
    /*--------------------------------------------------*/

    /**
     * Configuration for creating an ovphysx instance.
     * Initialize with OVPHYSX_CREATE_ARGS_DEFAULT for safe defaults.
     *
     * A non-empty active_cuda_gpus request is retained by this handle and applied
     * to the shared process physics backend when this handle attaches a stage.
     * To select a deterministic ordinal, make the request before the process's
     * first GPU scene creates its persistent CUDA context manager.
     * Per-scene CPU/GPU dynamics are controlled by physxScene:enableGPUDynamics in the USD
     * stage. ovphysx never reads or writes those settings.
     *
     * To force process-wide CPU-only mode (ovphysx touches no CUDA driver), call
     * ovphysx_set_cpu_mode(true) before creating any instances, or set
     * OVPHYSX_DISABLE_GPU before ovphysx initialization. See ovphysx_set_cpu_mode for the
     * external-driver boundary: the Python read and write paths expose warp.array tensors, so a
     * CUDA-enabled Warp BUILD opens the driver. Install a CPU-only Warp to keep both Python paths
     * driverless.
     *
     * DirectGPU notes
     * ===============
     * DirectGPU (eENABLE_DIRECT_GPU_API) skips GPU-to-CPU readback for faster
     * steps. It is opt-in: set /physics/suppressReadback=true via Carbonite
     * settings BEFORE ovphysx_create_instance. Restrictions: disables contact
     * modification (no surface velocity, no custom contact callbacks), and host-side
     * actor accessors return stale data after DirectGPU initializes.
     */
    typedef struct ovphysx_create_args
    {
        ovphysx_string_t bundled_deps_path;           /**< Bundled deps path: empty = runtime discovery (default: empty) */

        const ovphysx_config_entry_t* config_entries; /**< Array of typed config entries */
        uint32_t config_entry_count;                  /**< Number of config entries */

        ovphysx_string_t active_cuda_gpus;  /**< Comma-separated CUDA device ordinals (default: empty = no ovphysx ordinal override).
                                                 Restricts which GPU ordinal(s) are used. Supported patterns:
                                                 - Empty: preserve the current PhysX process selection
                                                   (a fresh/default process selects automatically)
                                                 - "0": single GPU 0
                                                 - "N": single GPU N
                                                 - "-1": PhysX default CUDA selection
                                                 - "0,1,...,N-1": all N GPUs, round-robin across scenes
                                                 - "1,2,...,N-1": all GPUs except first, round-robin
                                                 Lists are normalized into ascending ordinal order.
                                                 Input order does not control scene rotation.
                                                 A non-empty value takes precedence over OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE.
                                                 A single ordinal disables multi-GPU scene distribution.
                                                 A different ordinal after the first GPU scene requires a new process.
                                                 Other patterns are unsupported and return
                                                 OVPHYSX_API_INVALID_ARGUMENT when CUDA topology
                                                 is discoverable during instance creation. */
    } ovphysx_create_args;

    /**
     * Default initializer for ovphysx_create_args.
     */
    #define OVPHYSX_CREATE_ARGS_DEFAULT { \
        {NULL, 0},  /* bundled_deps_path */ \
        NULL,       /* config_entries */ \
        0,          /* config_entry_count */ \
        {NULL, 0}   /* active_cuda_gpus */ \
    }

    /**
     * PhysX debug-visualization parameters for ovphysx_debug_render_set_parameter() /
     * _get_parameter(). Mirrors omni::physx::PhysXVisualizationParameter. A
     * static_assert in the sidecar TU keeps the two enums aligned, so these named
     * constants are the stable, reorder-proof public spelling of the otherwise-opaque
     * integer index. NONE (0) and COUNT (one-past-the-end) are NOT valid parameters:
     * ovphysx_debug_render_set_parameter() rejects them with OVPHYSX_API_INVALID_ARGUMENT.
     */
    typedef enum
    {
        OVPHYSX_DEBUG_RENDER_PARAM_NONE                     = 0,
        OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES               = 1,
        OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES                = 2,
        OVPHYSX_DEBUG_RENDER_PARAM_BODY_MASS_AXES           = 3,
        OVPHYSX_DEBUG_RENDER_PARAM_BODY_LINEAR_VELOCITY     = 4,
        OVPHYSX_DEBUG_RENDER_PARAM_BODY_ANGULAR_VELOCITY    = 5,
        OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT            = 6,
        OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL           = 7,
        OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_ERROR            = 8,
        OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_IMPULSE          = 9,
        OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_POINT           = 10,
        OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_NORMAL          = 11,
        OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_IMPULSE         = 12,
        OVPHYSX_DEBUG_RENDER_PARAM_ACTOR_AXES               = 13,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AABBS          = 14,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_SHAPES         = 15,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AXES           = 16,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_COMPOUNDS      = 17,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_FACE_NORMALS   = 18,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_EDGES          = 19,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_STATIC_PRUNER  = 20,
        OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_DYNAMIC_PRUNER = 21,
        OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LOCAL_FRAMES       = 22,
        OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LIMITS             = 23,
        OVPHYSX_DEBUG_RENDER_PARAM_CULL_BOX                 = 24,
        OVPHYSX_DEBUG_RENDER_PARAM_MBP_REGIONS              = 25,
        OVPHYSX_DEBUG_RENDER_PARAM_SIMULATION_MESH          = 26,
        OVPHYSX_DEBUG_RENDER_PARAM_SDF                      = 27,
        OVPHYSX_DEBUG_RENDER_PARAM_COUNT                    = 28
    } ovphysx_debug_render_parameter_t;

    /**
     * Debug-visualization primitives, read from
     * ovphysx_debug_render_get_points / _lines / _triangles. The layout matches
     * omni::physx DebugPoint / DebugLine / DebugTriangle (a carb::Float3 position
     * plus a uint32 colour) so the OvPhysX debug buffer is read directly with no
     * copy or per-element translation. Colours are 0xAARRGGBB (PhysX debug colour).
     */
    typedef struct ovphysx_debug_point_t
    {
        float    pos[3];
        uint32_t color;
    } ovphysx_debug_point_t;

    typedef struct ovphysx_debug_line_t
    {
        float    pos0[3];
        uint32_t color0;
        float    pos1[3];
        uint32_t color1;
    } ovphysx_debug_line_t;

    typedef struct ovphysx_debug_triangle_t
    {
        float    pos0[3];
        uint32_t color0;
        float    pos1[3];
        uint32_t color1;
        float    pos2[3];
        uint32_t color2;
    } ovphysx_debug_triangle_t;

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_TYPES_H */
