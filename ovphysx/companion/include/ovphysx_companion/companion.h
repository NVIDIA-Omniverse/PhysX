// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-ATTACH-COMPUTE-001
// @covers AC-1 AC-2 AC-3 AC-4

// @implements REQ-COMP-MASS-001
// @covers AC-1 AC-2 AC-3

// @implements REQ-COMP-API-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-OWNER-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEBUG-001
// @covers AC-2

#ifndef OVPHYSX_COMPANION_H
#define OVPHYSX_COMPANION_H

#include <stddef.h>
#include <stdint.h>
#include <ovphysx/ovphysx_export.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OVPHYSX_COMPANION_ABI_VERSION 1u
typedef uint32_t ovphysx_companion_status_t;
enum
{
    OVPHYSX_COMPANION_OK = 0,
    OVPHYSX_COMPANION_INVALID_ARGUMENT = 1,
    OVPHYSX_COMPANION_ABI_MISMATCH = 2,
    OVPHYSX_COMPANION_UNAVAILABLE = 3,
    OVPHYSX_COMPANION_NOT_FOUND = 4,
    OVPHYSX_COMPANION_STALE = 5,
    OVPHYSX_COMPANION_BUSY = 6,
    OVPHYSX_COMPANION_ERROR = 7
};
typedef uint64_t ovphysx_companion_session_t;

enum
{
    OVPHYSX_COMPANION_CAP_BORROWED_SESSION = 1u << 0,
    OVPHYSX_COMPANION_CAP_SCENE_STATISTICS = 1u << 1,
    OVPHYSX_COMPANION_CAP_PRIMITIVE_MASS = 1u << 2,
    OVPHYSX_COMPANION_CAP_POINT_MATCHING = 1u << 3
};

typedef struct ovphysx_companion_identity
{
    uint64_t instance;
    uint64_t attachment;
} ovphysx_companion_identity_t;

/** Copied counters for one scene, measured in object counts, at an idle owner. */
typedef struct ovphysx_companion_scene_statistics
{
    uint32_t size;
    uint32_t version;
    ovphysx_companion_identity_t owner;
    uint32_t dynamic_bodies;
    uint32_t active_dynamic_bodies;
    uint32_t static_bodies;
    uint32_t kinematic_bodies;
    uint32_t articulations;
    uint32_t active_constraints;
} ovphysx_companion_scene_statistics_t;

/** Stopped primitive inputs in SI units, in a shared body-local frame.
 *  Box dimensions are half extents; sphere uses radius in x; capsule uses radius
 *  in x and half height in y, with local X axis. Rotation is unit xyzw.
 *  Densities are kg/m^3. Overlapping shapes contribute additively.
 */
enum { OVPHYSX_COMPANION_BOX = 0, OVPHYSX_COMPANION_SPHERE = 1, OVPHYSX_COMPANION_CAPSULE = 2 };
typedef struct ovphysx_companion_mass_shape
{
    uint32_t kind;
    float dimensions[3];
    float position[3];
    float rotation[4];
    float density;
} ovphysx_companion_mass_shape_t;

typedef struct ovphysx_companion_mass_result
{
    uint32_t size;
    uint32_t version;
    float mass;
    float volume;
    float center_of_mass[3];
    float diagonal_inertia[3];
    float principal_axes[4];
    float bounds_min[3];
    float bounds_max[3];
} ovphysx_companion_mass_result_t;

/** Borrowed sessions never attach or step. Serialize their operations with core
 *  operations on the same owner, following the core single-instance contract.
 *  Opening retains the owner until close; core destroy/shutdown return
 *  INVALID_STATE while leases exist. Detach is allowed and makes sessions stale.
 *  Close accepts stale sessions and releases their lease. Handles are never reused.
 *  Borrowed-operation outputs are written only on success. Returned records need no release.
 */
typedef struct ovphysx_companion_api
{
    uint32_t size;
    uint32_t version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*open_session)(uint64_t instance, ovphysx_companion_session_t* session);
    ovphysx_companion_status_t (*close_session)(ovphysx_companion_session_t session);
    ovphysx_companion_status_t (*get_identity)(ovphysx_companion_session_t session,
                                             ovphysx_companion_identity_t* identity);
    /** scene_path is a NUL-terminated authored path, resolved against this owner.
     *  out_size must match sizeof(ovphysx_companion_scene_statistics_t).
     *  GPU-specific counters are not part of this CPU-readable snapshot.
     */
    ovphysx_companion_status_t (*scene_statistics)(ovphysx_companion_session_t session,
                                                 const char* scene_path, uint32_t out_size,
                                                 ovphysx_companion_scene_statistics_t* output);
    /** No session or runtime initialization required. Rejects invalid inputs and
     *  results outside normal float range. invalid_shape is required, UINT32_MAX
     *  for aggregate errors/success, otherwise the rejected input index. Result
     *  stays untouched on failure. Input memory is borrowed only for this call.
     */
    ovphysx_companion_status_t (*compute_primitive_mass)(
        const ovphysx_companion_mass_shape_t* shapes, uint32_t count, uint32_t out_size,
        ovphysx_companion_mass_result_t* output, uint32_t* invalid_shape);
    /** Ready CPU float32 XYZ arrays, tightly packed in one Cartesian frame with
     *  common units. One int32 result per source (-1 outside inclusive distance).
     *  Ties choose the lowest target index; target reuse is allowed. Capacity must
     *  be source_count; zero counts accept null pointers. Failures preserve output.
     */
    ovphysx_companion_status_t (*match_points)(const float* source_xyz, uint32_t source_count,
        const float* target_xyz, uint32_t target_count, float distance,
        int32_t* output, uint32_t output_count);
} ovphysx_companion_api_t;

/** Obtain a table from the same loaded library as core ovphysx. This does not
 *  initialize a runtime. Exact ABI version and table size are required; failures
 *  leave output untouched. New ABI versions must not reinterpret old tables.
 */
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_api_t* output);

#ifdef __cplusplus
}
#endif
#endif
