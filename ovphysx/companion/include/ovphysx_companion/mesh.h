// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-ATTACH-COMPUTE-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEFORM-COOK-001
// @covers AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_MESH_H
#define OVPHYSX_COMPANION_MESH_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
/** CPU-only stopped geometry services. No SDK/stage/initialization required.
 *  Inputs are tightly packed caller-owned arrays in a shared Cartesian frame
 *  and unit, valid through return. Width=3 triangles, width=4 tetrahedra.
 *  Failures preserve all output arrays; invalid_element identifies rejected
 *  topology or UINT32_MAX for other validation errors. No implicit device copies.
 */
typedef struct ovphysx_companion_mesh_api {
    uint32_t size;
    uint32_t version;
    /** Arrays contain point_count entries except weights (4*point_count).
     *  -1 marks an unmatched point, with zero weights/offset and +inf distance.
     *  Matching is inclusive within max_distance; ties choose the lowest element.
     *  Triangle weights use the first 3 lanes; lane 4 is zero. Triangle offsets
     *  measure signed distance along the oriented unit normal. Outside tet points
     *  project to the nearest face: a nonzero distance is an explicit projection,
     *  not an exact embedding. Reject such results where full embedding is needed.
     */
    ovphysx_companion_status_t (*embed)(const float* vertices, uint32_t vertex_count,
        const uint32_t* indices, uint32_t element_count, uint32_t width,
        const float* points, uint32_t point_count, float max_distance,
        int32_t* elements, float* weights, float* distances, float* normal_offsets,
        uint32_t* invalid_element);
    /** Requires a complete mapping; -1, invalid weights, nonfinite data or
     *  degenerate current triangles with nonzero normal offsets fail. Tetrahedral offsets must be zero.
     *  output contains 3*point_count floats. No source or topology mutation.
     */
    ovphysx_companion_status_t (*interpolate)(const float* vertices, uint32_t vertex_count,
        const uint32_t* indices, uint32_t element_count, uint32_t width,
        const int32_t* elements, const float* weights, const float* normal_offsets,
        uint32_t point_count, float* output);
} ovphysx_companion_mesh_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mesh_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_mesh_api_t* output);

/** Optional motion capability, negotiated independently of embedding/interpolation. */
typedef struct ovphysx_companion_mesh_motion_api {
    uint32_t size;
    uint32_t version;
    /** Derivative of mapped positions for fixed weights/normal offsets, including
     *  the normal derivative. Positions/velocities share a Cartesian frame/unit;
     *  velocity units are length/second. Output is 3*point_count floats, unchanged
     *  on failure. Input topology and ownership follow the mesh table contract.
     */
    ovphysx_companion_status_t (*velocities)(const float* vertices, const float* velocities,
        uint32_t vertex_count, const uint32_t* indices, uint32_t element_count, uint32_t width,
        const int32_t* elements, const float* weights, const float* normal_offsets,
        uint32_t point_count, float* output);
} ovphysx_companion_mesh_motion_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mesh_motion_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_mesh_motion_api_t* output);

/** Optional stopped triangle-filter capability, negotiated independently.
 * Inputs are ready tightly packed CPU arrays in one Cartesian frame/unit.
 * Results are interleaved triangle index pairs in lexicographic order. Inclusive
 * distance includes intersections. Empty output is no match, never a wildcard.
 * Limits: 10M vertices per mesh, 10M candidate pairs, 1M output pairs. Invalid
 * topology/distance, degeneracy or capacity exhaustion returns INVALID_ARGUMENT.
 * Both pairs and count remain unchanged on any failure. No runtime/SDK required.
 */
typedef struct ovphysx_companion_mesh_filter_api {
    uint32_t size;
    uint32_t version;
    ovphysx_companion_status_t (*pairs)(const float* vertices0, uint32_t vertex_count0,
        const uint32_t* triangles0, uint32_t triangle_count0,
        const float* vertices1, uint32_t vertex_count1, const uint32_t* triangles1, uint32_t triangle_count1,
        float max_distance, uint32_t capacity, uint32_t* pairs, uint32_t* count);
} ovphysx_companion_mesh_filter_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mesh_filter_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_mesh_filter_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
