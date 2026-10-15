// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-COOK-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEFORM-COOK-001
// @covers AC-1 AC-2
#ifndef OVPHYSX_COMPANION_COOKING_H
#define OVPHYSX_COMPANION_COOKING_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif

enum {
    OVPHYSX_COMPANION_COOK_CONVEX = 1,
    OVPHYSX_COMPANION_COOK_TRIANGLE = 2,
    OVPHYSX_COMPANION_COOK_DECOMPOSITION = 3,
    OVPHYSX_COMPANION_COOK_SDF = 4,
    OVPHYSX_COMPANION_COOK_SPHERES = 5,
    OVPHYSX_COMPANION_COOK_CONFORMING_TETS = 6,
    OVPHYSX_COMPANION_COOK_VOXEL_TETS = 7
};
/** Copied input to a synchronous operation. Polygon faces sum to index_count;
 *  conforming input is triangles and voxel input is tetrahedra (faces omitted).
 *  Points and topology are tightly packed CPU arrays in one Cartesian frame.
 *  Meters-per-unit affects the cooking recipe but never implicitly rescales output.
 */
typedef struct ovphysx_companion_cook_mesh {
    uint32_t size, version;
    uint32_t point_count, index_count, face_count, right_handed;
    const float* points;
    const int32_t* indices;
    const int32_t* faces;
    double meters_per_unit;
} ovphysx_companion_cook_mesh_t;

/** Initialize with defaults() before overriding fields. All units are mesh-local.
 *  triangle_mode: 0 original, 1 simplification. sphere_mode: 0 flood, 1 raycast,
 *  2 surface. Tetra voxel_resolution counts voxels along the longest dimension;
 *  decomposition/sphere resolution counts total voxels. Cache identity includes
 *  the whole normalized recipe and SDK fingerprint.
 */
typedef struct ovphysx_companion_cook_options {
    uint32_t size, version, kind, triangle_mode;
    uint32_t max_hull_vertices, max_hulls, voxel_resolution, shrink_wrap;
    uint32_t sdf_resolution, sdf_subgrid_resolution, sdf_bits, sdf_remesh;
    uint32_t max_spheres, seed_count, sphere_mode, reserved;
    float simplification, weld_tolerance, min_thickness, error_percentage;
    float sdf_band, sdf_margin, sdf_reduction, scale[3];
} ovphysx_companion_cook_options_t;

typedef struct ovphysx_companion_cook_info {
    uint32_t size, version, kind, warnings;
    uint32_t point_count, index_count, element_width, face_map_count;
    uint32_t embedding_count, blob_count;
    uint64_t recipe_hash[2];
    char sdk_build[65];
} ovphysx_companion_cook_info_t;

enum {
    OVPHYSX_COMPANION_COOK_POINTS = 0,     /* float32 XYZ */
    OVPHYSX_COMPANION_COOK_INDICES = 1,    /* uint32 element indices */
    OVPHYSX_COMPANION_COOK_FACE_MAP = 2,   /* uint32 original polygon per triangle */
    OVPHYSX_COMPANION_COOK_EMBEDDING = 3,  /* int32 voxel tet per input vertex */
    OVPHYSX_COMPANION_COOK_BLOB = 4       /* bytes; select with buffer_index */
};

/** Separate stopped service: no attachment is created. open retains the existing
 *  runtime services independently of its bootstrap core. close returns BUSY during
 *  a cook; retry after completion. Copied-data cooking may run beside unrelated pending
 *  attachments. Callback reentry and exclusive recording transport transitions
 *  return BUSY without queueing; native authoring admission owns that exclusion. Results own copied storage and remain readable
 *  after service/core close until release. No native object pointers are returned.
 *  copy with data=NULL/capacity=0 queries required bytes; insufficient capacity
 *  returns INVALID_ARGUMENT and changes only required_bytes, never partial data.
 *  Synchronous kernels run outside Python; Python jobs provide queued cancellation
 *  and suppression/drain of an already running operation.
 */
typedef struct ovphysx_companion_cooking_api {
    uint32_t size, version;
    uint64_t capabilities; /* bit (1 << kind), CPU operations only */
    const char* sdk_build; /* immutable 64-character digest; table/library lifetime */
    ovphysx_companion_status_t (*defaults)(uint32_t kind, ovphysx_companion_cook_options_t* options);
    ovphysx_companion_status_t (*open)(uint64_t source_instance, uint64_t* service);
    ovphysx_companion_status_t (*close)(uint64_t service);
    ovphysx_companion_status_t (*cook)(uint64_t service, const ovphysx_companion_cook_mesh_t* mesh,
        const ovphysx_companion_cook_options_t* options, uint64_t* result);
    ovphysx_companion_status_t (*describe)(uint64_t result, uint32_t size, ovphysx_companion_cook_info_t* info);
    ovphysx_companion_status_t (*copy)(uint64_t result, uint32_t buffer_kind, uint32_t buffer_index,
        void* data, uint64_t capacity_bytes, uint64_t* required_bytes);
    ovphysx_companion_status_t (*release)(uint64_t result);
} ovphysx_companion_cooking_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_cooking_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_cooking_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
