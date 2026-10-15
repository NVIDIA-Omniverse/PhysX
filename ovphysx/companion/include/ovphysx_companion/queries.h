// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-QUERY-001
// @covers AC-1 AC-2 AC-4
#ifndef OVPHYSX_COMPANION_QUERIES_H
#define OVPHYSX_COMPANION_QUERIES_H
#include "inspection.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct ovphysx_companion_query {
    uint32_t kind, mode, geometry, flags, capacity;
    float pose[7], direction[3], distance, dimensions[3];
    ovphysx_companion_object_t shape;
    const ovphysx_companion_object_t* include;
    const ovphysx_companion_object_t* exclude;
    uint32_t includeCount, excludeCount;
} ovphysx_companion_query_t;
typedef struct ovphysx_companion_query_hit {
    ovphysx_companion_object_t body, shape, material;
    float distance, position[3], normal[3];
    uint32_t face, validFields, reserved;
} ovphysx_companion_query_hit_t;
typedef struct ovphysx_companion_query_result {
    uint64_t attachment, step, revision, total;
    uint32_t count, truncated;
} ovphysx_companion_query_result_t;
/** CPU query on the selected existing runtime owner. Retain that owner during calls.
 * kind: ray 0, sweep 1, overlap 2; mode: closest 0, any 1, all 2.
 * geometry: sphere 1 (radius), box 2 (half extents xyz), capsule 3 (radius,
 * half height along local X), bound shape 4. Bound sphere/box/capsule/convex
 * uses its cooked geometry/scale at the explicitly supplied world pose.
 * Other bound geometry types return UNAVAILABLE; no implicit cooking.
 * pose xyz+xyzw, normalized direction, distance>0 (sweep <=1e8), units world scene length.
 * flags: static 1, dynamic 2, both triangle sides 4. At least one actor flag.
 * Overlap closest/both-sides is UNAVAILABLE. Query-disabled shapes are excluded.
 * include/exclude <=256 exact body/shape references; include empty means all,
 * otherwise OR; exclusion wins. All references are generation-checked, including
 * filters with no matches. Shared shape exclusion applies to every instance;
 * use a body reference to select/exclude one instance. No callback/user pointers.
 * capacity 1..65536. all retains nearest capacity from all selected scenes,
 * ordered by distance then body/shape/face; overlaps ordered by identity.
 * total is full count, truncated explicitly signals discarded hits. closest
 * retains one nearest result. any stops at existence (total 0/1, no ordering).
 * One SDK hit per actor/shape, not every triangle. validFields bits: distance 1, authored triangle face 2, position 4,
 * normal 8. Initial overlap may lack position; overlap queries have none.
 * Primitive faces and meshes without an authored face map have no face field.
 * No implicit fetch/step: pending/callback BUSY. Failure preserves all outputs.
 * Result revision is the conservative native record-change epoch; step is the
 * owner's completed step. Individual identities use per-record generations.
 * Input, output and element sizes must exactly match this ABI. hitCapacity must
 * be >= query.capacity; only result.count elements are written.
 */
typedef struct ovphysx_companion_queries_api {
    uint32_t size,version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*execute)(uint64_t attachment,const ovphysx_companion_query_t*,uint32_t querySize,
        ovphysx_companion_query_hit_t*,uint32_t hitSize,uint32_t hitCapacity,
        ovphysx_companion_query_result_t*,uint32_t resultSize);
} ovphysx_companion_queries_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_queries_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_queries_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
