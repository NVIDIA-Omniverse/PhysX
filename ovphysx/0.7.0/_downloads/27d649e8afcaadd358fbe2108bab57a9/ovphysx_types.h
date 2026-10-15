// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-STRING-001
 * @covers AC-1 AC-4
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-8
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-READPOOL-001
 * @covers AC-1
 * @implements REQ-CAPI-OBJECTTYPE-001
 * @covers AC-1
 * @implements REQ-CAPI-READ-CONTACT-001
 * @covers AC-5 AC-6
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-1 AC-2 AC-8
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-1 AC-2 AC-3 AC-6
 * @implements REQ-CAPI-VEHICLE-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-SESSION-001
 * @covers AC-3 AC-4 AC-7
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-15
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1
 */
/** @endcond */

#ifndef OVPHYSX_TYPES_H
#define OVPHYSX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dlpack/dlpack.h"
#include "ovphysx_export.h"

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
     * bookkeeping (sessions, bindings, subscriptions, op waits, debug-render
     * cache) and at most one live ovstage attach. Several handles may be
     * attached at once. They share process-wide services (foundation, SDK
     * tolerance scale, hard CPU-only mode, logging, recording transport).
     */
    typedef uint64_t ovphysx_instance_handle_t;

    /** Deprecated 0.6 compatibility spelling; removal proposed for 0.7. */
    typedef OVPHYSX_DEPRECATED_MSG("use ovphysx_instance_handle_t") ovphysx_instance_handle_t ovphysx_handle_t;
    typedef uint64_t ovphysx_usd_handle_t;
    typedef uint64_t ovphysx_attribute_binding_handle_t;
    typedef uint64_t ovphysx_write_map_handle_t;
    typedef uint64_t ovphysx_read_map_handle_t;
    typedef uint64_t ovphysx_op_index_t;
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
        OVPHYSX_WRITABILITY_WRITE_ONLY = 3,   /**< writable control input with no read-back through ovphysx_read */
        OVPHYSX_WRITABILITY_READ_ONLY = 4,    /**< readable only. Writing is an error naming the type, not a silent no-op */
    } ovphysx_writability_t;

    /**
     * Sentinel value representing an invalid/null ovphysx instance handle.
     * Valid handles are nonzero, so 0 is reserved to indicate "no handle" or "invalid handle".
     * Use this when a handle parameter is required but no valid instance is available.
     */
    #define OVPHYSX_INVALID_HANDLE 0

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
     * Query target type selected by ovphysx_query(). These are engine types,
     * not a USD schema predicate.
     *
     * Values are explicit, contiguous, and start at 0. Leave
     * OVPHYSX_QUERY_TARGET_COUNT unnumbered so it stays one past the last value.
     * Query gates treat [0, OVPHYSX_QUERY_TARGET_COUNT) as every valid target, so a
     * gap is an unnamed target.
     *
     * Add a target immediately before OVPHYSX_QUERY_TARGET_COUNT and give it the next
     * integer. Do not insert earlier, leave a gap, or change a value that
     * already shipped: the integer is the public query argument.
     *
     * Most entries select simulated prims. OVPHYSX_QUERY_TARGET_DEBUG_RENDER does
     * not: it is an instance target, not a prim. Further instance targets
     * belong with it. That grouping does not reserve the tail of the enum.
     * A later simulated type appends after those targets rather than inserting
     * ahead of them.
     */
    typedef enum
    {
        OVPHYSX_QUERY_TARGET_RIGID_BODY = 0,         /**< dynamic rigid bodies (standalone + point-instancer instances) */
        OVPHYSX_QUERY_TARGET_ARTICULATION_LINK = 1,  /**< articulation link body transforms */
        OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT = 2, /**< articulation joint state (per-axis, one array group per joint) */
        OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL = 3,      /**< vehicle wheel transforms */
        OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME = 4,  /**< volume deformable meshes (points / velocities) */
        OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE = 5, /**< surface deformable meshes */
        OVPHYSX_QUERY_TARGET_PARTICLE_SET = 6,       /**< particle sets */
        OVPHYSX_QUERY_TARGET_FIXED_TENDON = 7,       /**< articulation fixed tendons (prim: joint carrying the root axis) */
        OVPHYSX_QUERY_TARGET_SPATIAL_TENDON = 8,     /**< articulation spatial tendons (prim: link carrying the root attachment) */
        OVPHYSX_QUERY_TARGET_ARTICULATION = 9,        /**< whole articulations (prim: the articulation-root API prim) */
        OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL = 10, /**< deformable materials (prim: the bound Material prim) */
        /** Contact-reporting bodies and colliders carrying PhysxContactReportAPI; contact forces, contacts, events.
         * Separate child-collider rows are available only on CPU simulation or GPU dynamics with CPU readback.
         * DirectGPU omits separate child-collider rows; standalone static collider rows remain but read zero force.
         * A body row aggregates its colliders' -- don't sum across them.
         */
        OVPHYSX_QUERY_TARGET_CONTACT_SENSOR = 11,
        OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER = 12, /**< capsule character controllers */
        OVPHYSX_QUERY_TARGET_VEHICLE = 13,             /**< whole vehicles (prim: the vehicle/chassis prim) */
        OVPHYSX_QUERY_TARGET_SCENE = 14,               /**< PxScene at the physics-scene prim */
        OVPHYSX_QUERY_TARGET_PHYSX_MATERIAL = 15,      /**< PxMaterial (not a deformable material) */
        OVPHYSX_QUERY_TARGET_SHAPE = 16,               /**< PxShape */
        OVPHYSX_QUERY_TARGET_COMPOUND_SHAPE = 17,      /**< compound-shape wrapper */
        OVPHYSX_QUERY_TARGET_RIGID_ACTOR = 18,         /**< PxRigidDynamic and PxRigidStatic */
        OVPHYSX_QUERY_TARGET_JOINT = 19,               /**< standalone PxJoint */
        OVPHYSX_QUERY_TARGET_CUSTOM_JOINT = 20,        /**< plugin custom joint */
        OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM = 21,     /**< PxPBDParticleSystem */
        OVPHYSX_QUERY_TARGET_PHYSICS = 22,             /**< process-global PxPhysics; one row, path "/" */
        /* Instance targets, not simulated prims. Same numbering rules as the
           entries above: next integer, immediately before OVPHYSX_QUERY_TARGET_COUNT. */
        OVPHYSX_QUERY_TARGET_DEBUG_RENDER = 23,        /**< this instance's debug-render target; not a prim */
        OVPHYSX_QUERY_TARGET_SDF_SHAPE = 24,           /**< SDF collision shapes (query-only) */
        OVPHYSX_QUERY_TARGET_COUNT                     /**< one past the last valid type; the range-check bound (not a type) */
    } ovphysx_query_target_t;

    /**
     * Output query scope.
     *
     * @note Queries are reusable selectors. OVPHYSX_SCOPE_ACTIVE membership is
     *       recomputed every step and evaluated when a read or write session opens.
     *       OVPHYSX_SCOPE_ALL membership is stable across steps until a structural
     *       change (object add/remove, instancer instance-count change). Session
     *       groups capture the membership at session open.
     *       OVPHYSX_QUERY_TARGET_DEBUG_RENDER validates this argument and then ignores
     *       it. Geometry filtering for that target is its scope setting, not
     *       this query scope.
     *
     *       For OVPHYSX_QUERY_TARGET_ARTICULATION, ACTIVE includes an articulation
     *       when any link is reported by its owning scene's active-actor set.
     *       If that scene cannot report active actors, articulation awake state
     *       is the fallback. DirectGPU currently disables sleeping, so its
     *       whole-articulation ACTIVE membership is equivalent to ALL.
     *       Scene, PhysX material, shape, compound shape, rigid actor, standalone
     *       joint, custom joint, particle system, and the PxPhysics singleton have
     *       no active notion: ACTIVE matches ALL. The PxPhysics query is one row
     *       and ignores prim-set selection.
     */
    typedef enum
    {
        OVPHYSX_SCOPE_ALL = 0,    /**< every object of the type */
        OVPHYSX_SCOPE_ACTIVE = 1, /**< type-specific active membership; see ovphysx_query */
    } ovphysx_object_scope_t;

    /**
     * Prim-set selection for @ref ovphysx_query_selection -- an explicit list of native prim-path
     * handles OR a glob pattern, applied to BOTH the read and a write opened over the query (ADR-0025).
     * The selection stands in for a scope: a selected query covers every selected object of its type,
     * sleeping ones included. prim_paths wins over pattern when both are set; they are never combined. A
     * selection that resolves to no object of the queried type is a valid empty query
     * (total_prim_count == 0), not an error.
     *
     * prim_paths are interned ovx_primpath_t handles the caller already holds -- from a read
     * group's `prims.list` (resolved through the attached Stage's dictionary) or interned through the attached
     * Stage's OVX dictionary. They are NOT strings: the caller resolves paths to handles, so the runtime
     * maps them straight to objects without re-running the glob matcher. pattern stays a string because a
     * glob is resolved, not named. Handles are valid only for the attached Stage's path dictionary. A
     * handle kept after every OVStage instance was released belongs to a destroyed dictionary: it is
     * invalid, and the runtime cannot detect it, so it may name a different prim.
     */
    typedef struct
    {
        /** Path pattern (ignored if prim_paths is set); rules on the "Path patterns" docs page */
        ovphysx_string_t pattern;
        const ovx_primpath_t* prim_paths;     /**< Explicit interned prim-path handles (NULL = use pattern) */
        uint32_t prim_paths_count;            /**< Number of handles (0 = use pattern) */
    } ovphysx_query_selection_t;

    /**
     * Reasons applicable selected read partitions did not produce groups.
     * Bits combine across partitions; they do not identify or count affected objects.
     */
    typedef enum
    {
        OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE = 1u,
        OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION = 2u,
        OVPHYSX_READ_OMISSION_NOT_READY = 4u,
    } ovphysx_read_omission_t;

    /** One requested read attribute, in original request order. */
    typedef struct
    {
        ovx_token_t attribute; /**< Canonical token; OVX_INVALID_TOKEN for an unsupported name. */
        size_t group_count; /**< Emitted groups (batches), not objects or tensor rows. */
        /** Bitwise OR of ovphysx_read_omission_t; may coexist with emitted groups. Zero is not an error. */
        uint32_t omission_reasons;
    } ovphysx_read_attribute_result_t;

    /**
     * Immutable read-open summary. Nested storage belongs to the read session
     * until accepted read release or instance destruction. Detach invalidates
     * the attachment's dictionary use, not this summary storage.
     * Copying this struct copies the attributes pointer, not its rows. Copy all
     * attribute_count rows before their lifetime ends to retain them longer;
     * copied tokens still require a valid attachment dictionary for resolution.
     */
    typedef struct
    {
        size_t total_prim_count; /**< Matched prim identities, not emitted rows or ready GPU objects. */
        size_t attribute_count; /**< Original request length, including unsupported attributes. */
        const ovphysx_read_attribute_result_t* attributes; /**< Session-owned rows; see the lifetime above. */
    } ovphysx_read_result_t;

    /** One accepted write attribute, in original request order. */
    typedef struct
    {
        ovx_token_t attribute; /**< Canonical token from the attached Stage's dictionary. */
        /** Planned groups (batches), not objects or tensor rows; no promise of commit success. */
        size_t group_count;
    } ovphysx_write_attribute_result_t;

    /**
     * Immutable write-open summary. Nested storage belongs to the write session
     * until accepted write release, stage detach or instance destruction,
     * whichever occurs first.
     * Copying this struct copies the attributes pointer, not its rows. Copy all
     * attribute_count rows before their lifetime ends to retain them longer;
     * copied tokens still require a valid attachment dictionary for resolution.
     */
    typedef struct
    {
        size_t total_prim_count; /**< Matched prim identities, not emitted rows or ready GPU objects. */
        size_t attribute_count; /**< Original request length. */
        const ovphysx_write_attribute_result_t* attributes; /**< Session-owned rows; see the lifetime above. */
    } ovphysx_write_result_t;

    /** Writable group and its unambiguous original request index. */
    typedef struct
    {
        const ovstage_map_group_t* group; /**< Original producer-owned pointer; also the commit identity. */
        size_t attribute_index; /**< Original request index, or SIZE_MAX when group is NULL. */
    } ovphysx_fetched_write_group_t;

    /**
     * One contact-sensor selection and its filters, for @ref ovphysx_query_contacts (ADR-0044). `sensors`
     * selects sensor bodies. `filters` points at `filters_per_sensor` filter selections (NULL when that is
     * 0): filter f must match either one object, shared by every sensor of this selection, or exactly one
     * object per sensor, paired by position. Each selection follows the @ref ovphysx_query_selection_t rules.
     */
    typedef struct
    {
        ovphysx_query_selection_t sensors;        /**< The sensor bodies */
        const ovphysx_query_selection_t* filters; /**< filters_per_sensor filter selections, or NULL */
    } ovphysx_contact_sensor_selection_t;

    /**
     * @brief Bit flags for ovphysx_update_articulations_kinematic.
     *
     * Mirrors PxArticulationKinematicFlag::Enum. Flags may be OR'd.
     */
    typedef enum
    {
        OVPHYSX_ARTICULATION_KINEMATIC_POSITION = 1u << 0, /**< Recompute link transforms from joint positions + root pose */
        OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY = 1u << 1, /**< Recompute link velocities from joint velocities + root velocity */
    } ovphysx_articulation_kinematic_flag_t;

    /* OVPHYSX_ATTR_* names are string literals in ovphysx_attributes.h.
     * This header does not include that header. */

    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_ENABLED;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_SCALE;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_WORLD_AXES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_BODY_AXES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_BODY_MASS_AXES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_BODY_LINEAR_VELOCITY;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_BODY_ANGULAR_VELOCITY;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_CONTACT_POINT;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_CONTACT_NORMAL;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_CONTACT_ERROR;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_CONTACT_IMPULSE;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_FRICTION_POINT;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_FRICTION_NORMAL;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_FRICTION_IMPULSE;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_ACTOR_AXES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_AABBS;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_SHAPES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_AXES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_COMPOUNDS;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_FACE_NORMALS;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_EDGES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_STATIC_PRUNER;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_COLLISION_DYNAMIC_PRUNER;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_JOINT_LOCAL_FRAMES;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_JOINT_LIMITS;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_CULL_BOX;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_MBP_REGIONS;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_SIMULATION_MESH;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_SDF;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_CULLING_BOX;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_SCOPE;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_POINT_POSITION;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_POINT_COLOR;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_LINE_POSITION;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_LINE_COLOR;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_TRIANGLE_POSITION;
    OVPHYSX_API extern const ovx_string_or_token_t OVPHYSX_NAME_DEBUG_TRIANGLE_COLOR;

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
     *     disableSimulation) and isSleeping are `{kDLUInt, 8}`; shapeCount is `{kDLInt, 32}`.
     *     `dtype.lanes` is the tuple width, and for a per-shape column it is the
     *     widest selected object in THAT read, so it varies between reads of the same
     *     attribute. Do not cache it.
     *   - residency is per attribute too. On a DirectGPU scene the simulated outputs
     *     (pose, velocity, acceleration, and whole-articulation centerOfMassWorld /
     *     centerOfMassLocal) are `kDLCUDA`, while authored body properties (mass,
     *     inertia, centerOfMassPosition / centerOfMassOrientation, and the flags) are
     *     `kDLCPU`: PhysX never writes those authored values back, so there is no
     *     device copy to read. isSleeping is also `kDLCPU`: DirectGPU disables sleeping,
     *     so the scene flag supplies 0. One read can therefore hand back both. Branch on
     *     `tensors[i].device.device_type`.
     *   - discovery is an `ovstage_query_result_t` (`attributes` is the interned token
     *     array, and `total_prim_count == 0` is the valid empty-match case).
     *   - attribute names are `ovx_string_or_token_t`. Pass a string (e.g.
     *     OVPHYSX_ATTR_POSITION from ovphysx_attributes.h) or an interned token
     *     from discovery, no round-trip.
     *
     * The queried `ovphysx_query_target_t` is NOT carried on the group: a read is
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
     * Names the PhysX kind a `physxPointer` read returns for the matching
     * query object type. Object-change callbacks still pass this enum.
     * The named constants below cover common runtime object types.
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
     * Fires AFTER the object exists, so it is safe to read `physxPointer`
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
     * prim_path / type before returning. That pointer is unusable once this
     * callback returns. Do NOT call release() on it (ovphysx owns the lifetime).
     *
     * Only fires for destructions that occur during simulation. Bulk teardown
     * (e.g. ovphysx_detach_ovstage()) is delivered via ovphysx_all_objects_destroyed_fn
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
     * Fires BEFORE the bulk teardown (e.g. on ovphysx_detach_ovstage()). Subscribers
     * should flush their entire pointer cache before returning. Every cached
     * pointer is unusable once this callback returns. No per-object destruction
     * events are delivered for this teardown.
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
     * ovphysx_step(), ovphysx_step_sync(), or ovphysx_detach_ovstage().
     * Do NOT call other ovphysx APIs from inside a callback (re-entrancy /
     * deadlock risk). ovphysx_unsubscribe_object_changes() while any
     * object-change callback is running returns OVPHYSX_API_INVALID_STATE
     * and leaves the subscription active. That check follows the Carbonite
     * task when the callback is running as one, and the native thread
     * otherwise.
     * Defer follow-up work until the triggering synchronous
     * call returns, or until the triggering task reaches terminal completion
     * through ovphysx_wait_task() or ovphysx_wait_all().
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
        OVPHYSX_API_INVALID_STATE = 9,       /**< Operation is not valid in the instance's current lifecycle state */
    } ovphysx_api_status_t;

    /**
     * Result returned by synchronous API functions.
     *
     * On failure (status != OVPHYSX_API_SUCCESS), call ovphysx_get_last_error()
     * on the same thread to retrieve the error message. A task-wait TIMEOUT
     * instead reports pending work and any collected failures in its wait result;
     * it does not set an immediate error message.
     */
    typedef struct
    {
        ovphysx_api_status_t status; /**< Operation status code */
    } ovphysx_result_t;

    /**
     * Copyable receipt for an accepted task. SUCCESS means admission, including
     * work that executes inline and fails. Collect execution status with a wait.
     * Rejected submissions have zero identity and explain the rejection through
     * ovphysx_get_last_error() on the calling thread.
     */
    typedef struct
    {
        ovphysx_api_status_t status; /**< Admission status */
        ovphysx_op_index_t op_index; /**< Nonzero issued task index */
        ovphysx_instance_handle_t instance;   /**< Owning instance, zero on rejection */
    } ovphysx_enqueue_result_t;

    /**
     * Owned output of ovphysx_wait_task() or ovphysx_wait_all(). Initialize to zero
     * before use; release with ovphysx_destroy_wait_task_result(). The library
     * owns the returned fields: do not modify them, free error_tasks separately,
     * or shallow-copy this struct. Individual error receipts may be copied;
     * their borrowed messages remain valid until this result is destroyed,
     * including after instance teardown.
     */
    typedef struct
    {
        const ovphysx_enqueue_result_t* error_tasks; /**< Failed tasks in submission order */
        size_t num_errors;                          /**< Number of failed tasks */
        ovphysx_enqueue_result_t pending_task;       /**< Earliest pending task; all zero if none */
    } ovphysx_wait_task_result_t;

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
    /* Scoped configuration                            */
    /*--------------------------------------------------*/

    /** Value discriminator shared by instance and process-global configuration. */
    typedef enum ovphysx_config_value_type_t
    {
        OVPHYSX_CONFIG_VALUE_BOOL = 0,
        OVPHYSX_CONFIG_VALUE_INT64 = 1,
        OVPHYSX_CONFIG_VALUE_DOUBLE = 2,
        OVPHYSX_CONFIG_VALUE_STRING = 3
    } ovphysx_config_value_type_t;

    /**
     * Tagged configuration value.
     *
     * String inputs are borrowed for the duration of the call. For string
     * getters, string_value.ptr addresses caller-owned writable storage and
     * string_value.length is its capacity. On success length becomes the content
     * length. On OVPHYSX_API_BUFFER_TOO_SMALL it becomes the required capacity,
     * including the terminating NUL.
     */
    typedef struct ovphysx_config_value_t
    {
        ovphysx_config_value_type_t type;
        union
        {
            bool bool_value;
            int64_t int64_value;
            double double_value;
            ovphysx_string_t string_value;
        } value;
    } ovphysx_config_value_t;

    /**
     * Process-global configuration keys.
     *
     * Global keys occupy 0x00000000..0x0000ffff. Values in the instance-key
     * range are always rejected even when explicitly cast to this enum.
     */
    typedef enum ovphysx_global_config_key_t
    {
        OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING = 0x00000000, /**< bool; runtime mutable */
        OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY = 0x00000001, /**< bool; no stage attached; inverse of /physics/collisionApproximateCones */
        OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY = 0x00000002, /**< bool; no stage attached; inverse of /physics/collisionApproximateCylinders */
        OVPHYSX_GLOBAL_CONFIG_NUM_THREADS = 0x00000003, /**< int64 in 0..255; no stage attached */
        OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE = 0x00000004, /**< int64 in 0..2; no stage attached */
        OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB = 0x00000005, /**< int64 fitting int32; runtime mutable */
        OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND = 0x00000006, /**< int64 ovphysx_profiler_backend_t; startup-only */
        OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY = 0x00000007, /**< string; startup-only */
        OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH = 0x00000008, /**< string; startup-only */
        OVPHYSX_GLOBAL_CONFIG_CPU_ONLY = 0x00000009, /**< bool; startup-only; true is sticky */
        OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL = 0x0000000a, /**< int64 ovphysx_log_level_t; runtime mutable */
        OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT = 0x0000000b, /**< bool; runtime mutable */
        /** bool; default false; startup-only. Enables OVD recording capability
         * without starting output. Attach a stage, then call
         * ovphysx_ovd_start_recording() to record. */
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE = 0x0000000c
    } ovphysx_global_config_key_t;

    /**
     * Per-instance configuration keys.
     *
     * Instance keys occupy 0x00010000..0xffff0000 so accidental casts from a
     * global key fail validation instead of selecting another setting.
     */
    typedef enum ovphysx_config_key_t
    {
        OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS = 0x00010000 /**< string; mutable before this instance attaches */
    } ovphysx_config_key_t;

    /**
     * Profiler backend selection used by
     * OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND.
     */
    typedef enum ovphysx_profiler_backend_t
    {
        OVPHYSX_PROFILER_BACKEND_NONE = 0,
        OVPHYSX_PROFILER_BACKEND_NVTX = 1,
        OVPHYSX_PROFILER_BACKEND_TRACE = 2
    } ovphysx_profiler_backend_t;

    /** Destination transport for a late OmniPVD recording. */
    typedef enum ovphysx_omnipvd_transport_t
    {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE = 0, /**< Write one exact .ovd file path. */
        OVPHYSX_OMNIPVD_TRANSPORT_TCP = 1   /**< Connect to one TCP listener. */
    } ovphysx_omnipvd_transport_t;

    /**
     * Destination for ovphysx_ovd_start_recording().
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

    /**
     * PhysX debug-visualization parameter indices. Mirrors
     * omni::physx::PhysXVisualizationParameter. A static_assert in the sidecar
     * keeps the two enums aligned. The session attribute for each index is the
     * matching OVPHYSX_ATTR_DEBUG_* name. NONE (0) and COUNT are not parameters.
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
     * Packed PhysX debug primitives (position plus 0xAARRGGBB colour). The
     * OVPHYSX_QUERY_TARGET_DEBUG_RENDER read splits each list into a position column
     * and a color column with aligned indices. There is no index buffer.
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
