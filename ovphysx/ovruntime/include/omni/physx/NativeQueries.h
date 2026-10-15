// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-QUERY-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include "NativeInspection.h"
namespace omni::physx::native
{
struct SceneQuery
{
    uint32_t kind=0, mode=0, geometry=0, flags=3, capacity=256;
    // kind 0 ray, 1 sweep, 2 overlap; mode 0 closest, 1 any, 2 all.
    // geometry 1 sphere, 2 box, 3 capsule (+X), 4 existing native convex shape.
    // flags 1 static, 2 dynamic, 4 both triangle sides.
    float pose[7]{0,0,0,0,0,0,1}, direction[3]{1,0,0}, distance=1, dimensions[3]{1,1,1};
    ObjectReference shape;
    std::vector<ObjectReference> include, exclude; // body/shape; OR within include, exclusion wins
};
struct SceneQueryHit
{
    ObjectReference body,shape,material;
    float distance=0, position[3]{}, normal[3]{};
    uint32_t face=UINT32_MAX, validFields=0, reserved=0; // 1 distance, 2 authored mesh face, 4 position, 8 normal; overlap none
};
struct SceneQueryResult
{
    uint64_t attachment=0,step=0,revision=0,total=0;
    bool truncated=false;
    std::vector<SceneQueryHit> hits;
};
// Exact-owner idle query on the existing native scene-query structures. Missing
// owner Stale, pending/callback Busy, invalid numeric input Invalid, unsupported
// variants Unavailable. CPU-qualified only. No cooking, stepping or completion.
// Positions/distances in world scene length units, normalized directions/normals.
// One SDK hit per shape/actor pair, not every crossed triangle. all/closest scan
// all selected scenes; ordered by distance then body/shape/face identity. any is
// an existence result with no geometric ordering guarantee. total for any is 0/1.
// All retains nearest capacity hits (overlap ordered by identity); total counts
// the full result. closest returns <=1 and is never capacity-truncated.
// Overlap closest and sidedness are unsupported; no location fields invented.
// Bound shape geometry must be sphere/box/capsule/convex, uses explicit query pose
// with its already-cooked geometry/scale. Shared shapes need no unique actor pose.
// Filters and bound shape are generation-validated under the same admission.
// Shapes lacking the SDK scene-query flag (including usual triggers) are absent.
// Failure preserves output. All SDK pointers stay inside this admitted call.
InspectionResult queryScene(uint64_t attachment,const SceneQuery&,SceneQueryResult&) noexcept;
}
