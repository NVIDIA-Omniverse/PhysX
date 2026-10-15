// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-MASS-001
// @covers AC-1 AC-2 AC-3
#ifndef OVPHYSX_COMPANION_MASS_H
#define OVPHYSX_COMPANION_MASS_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
/** Flags 1 mass, 2 center, 4 diagonal inertia, 8 principal axes (unit xyzw).
 * SI values; collider overrides use collider-local frame, body overrides body-local.
 * Density precedence is resolved by the caller. Mass rescales computed inertia;
 * a changed center adds the parallel-axis term unless inertia is overridden.
 */
typedef struct ovphysx_companion_mass_overrides
{
    uint32_t flags;
    float mass,center[3],diagonal[3],axes[4];
} ovphysx_companion_mass_overrides_t;
typedef struct ovphysx_companion_general_mass_shape
{
    uint32_t size,kind;
    float dimensions[3],position[3],rotation[4],density;
    const float* points;
    const uint32_t* triangles;
    uint32_t point_count,triangle_count;
    ovphysx_companion_mass_overrides_t overrides;
} ovphysx_companion_general_mass_shape_t;
/** Capabilities 1 primitives, 2 closed triangle meshes, 4 authored overrides.
 * No runtime initialization/attachment. Kind 0..2 follows companion.h; kind 3 is
 * a closed, connected, consistently oriented, non-self-intersecting mesh. Either
 * global winding accepted. Points tightly packed float XYZ, triangles uint32
 * index triples. No topology repair, approximation or union of overlaps.
 * 1..65536 shapes, up to 1M points/2M triangles per mesh. Memory borrowed only
 * during call. Failure preserves output; invalid_shape is UINT32_MAX for body
 * errors, otherwise rejected collider index. Exact struct sizes required.
 */
typedef struct ovphysx_companion_mass_api
{
    uint32_t size,version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*compute)(const ovphysx_companion_general_mass_shape_t*,uint32_t,
        uint32_t,const ovphysx_companion_mass_overrides_t*,uint32_t,
        ovphysx_companion_mass_result_t*,uint32_t*);
} ovphysx_companion_mass_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mass_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_mass_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
