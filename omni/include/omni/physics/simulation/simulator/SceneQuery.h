// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/Function.h>

namespace omni
{
namespace physics
{

struct SceneQueryHitObject
{
    uint64_t collision; // encoded SdfPath into uint64_t
    uint64_t rigidBody; // encoded SdfPath into uint64_t
    uint32_t protoIndex; // protoIndex, filled for pointInstancers otherwise 0xFFFFFFFF
};

struct SceneQueryHitLocation : SceneQueryHitObject
{
    carb::Float3 normal;
    carb::Float3 position;
    float distance;
    uint32_t faceIndex;
    uint64_t material; // encoded SdfPath into uint64_t
};

struct OverlapHit : SceneQueryHitObject
{
};

struct RaycastHit : SceneQueryHitLocation
{
};

struct SweepHit : SceneQueryHitLocation
{
};

using OverlapHitReportFn = std::function<bool(const OverlapHit& hit)>; // return True to continue traversal, False to
                                                                       // stop traversal
using RaycastHitReportFn = std::function<bool(const RaycastHit& hit)>; // return True to continue traversal, False to
                                                                       // stop traversal
using SweepHitReportFn = std::function<bool(const SweepHit& hit)>; // return True to continue traversal, False to stop
                                                                   // traversal

// Raycast the closest hit
//\param[in] origin Origin of the raycast
//\param[in] unitDir Normalized direction of the raycast
//\param[in] distance Length of the raycast
//\param[out] hit Raycast hit report
//\param[in] bothSides If mesh both triangle sides should be checked
//\return True if hit was found
using RaycastClosestFn = std::function<bool(
    const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& hit, bool bothSides)>;

// Raycast any physics sc	ene, returns only boolean if hit was found or not
//\param[in] origin		Origin of the ray.
//\param[in] unitDir	Normalized direction of the ray.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using RaycastAnyFn =
    std::function<bool(const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides)>;

// Raycast any physics scene, returns all hits found in a raycast callback
//\param[in] origin		Origin of the ray.
//\param[in] unitDir	Normalized direction of the ray.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] reportFn Report function to be called for each hit
//\param[in] bothSides	If mesh both triangle sides should be checked.
using RaycastAllFn = std::function<void(
    const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHitReportFn reportFn, bool bothSides)>;

// Sweep sphere closest hit
//\param[in] radius		Sphere radius.
//\param[in] origin		Origin of the sweep.
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[out] hit	    Sweep hit report
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepSphereClosestFn = std::function<bool(
    float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides)>;

// Sweep sphere any physics scene, returns only boolean if hit was found or not
//\param[in] radius		Sphere radius.
//\param[in] origin		Origin of the sweep.
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepSphereAnyFn =
    std::function<bool(float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides)>;

// Sweep sphere all physics scene, returns all hits found in a sweep callback
//\param[in] radius		Sphere radius.
//\param[in] origin		Origin of the sweep.
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] reportFn Report function to be called for each hit
//\param[in] bothSides	If mesh both triangle sides should be checked.
using SweepSphereAllFn = std::function<void(
    float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)>;

// Sweep test of a box against all objects in the physics scene, returning the closest hit found.
//\param[in] halfExtent Box half extent
//\param[in] pos	    Box position. This is the origin of the sweep.
//\param[in] rot	    Box rotation (quaternion x, y, z, w)
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[out] hit	    Sweep hit report
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepBoxClosestFn = std::function<bool(const carb::Float3& halfExtent,
                                             const carb::Float3& pos,
                                             const carb::Float4& rot,
                                             const carb::Float3& unitDir,
                                             float distance,
                                             SweepHit& hit,
                                             bool bothSides)>;

// Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning the closest hit found.
//\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[out] hit	    Sweep hit report
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepShapeClosestFn =
    std::function<bool(uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides)>;

// Sweep test of a box against all objects in the physics scene, returning whether any hit was found.
//\param[in] halfExtent Box half extent
//\param[in] pos	    Box position. This is the origin of the sweep.
//\param[in] rot	    Box rotation (quaternion x, y, z, w)
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepBoxAnyFn = std::function<bool(const carb::Float3& halfExtent,
                                         const carb::Float3& pos,
                                         const carb::Float4& rot,
                                         const carb::Float3& unitDir,
                                         float distance,
                                         bool bothSides)>;

// Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning whether any hit was found.
//\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepShapeAnyFn =
    std::function<bool(uint64_t gPrimPath, const carb::Float3& unitDir, float distance, bool bothSides)>;

// Sweep test of a box against all objects in the physics scene, returning all the hits found.
//\param[in] halfExtent Box half extent
//\param[in] pos	    Box position. This is the origin of the sweep.
//\param[in] rot	    Box rotation (quaternion x, y, z, w)
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\return True if hit was found
using SweepBoxAllFn = std::function<void(const carb::Float3& halfExtent,
                                         const carb::Float3& pos,
                                         const carb::Float4& rot,
                                         const carb::Float3& unitDir,
                                         float distance,
                                         SweepHitReportFn reportFn,
                                         bool bothSides)>;

// Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning all the hits found.
//\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
//\param[in] unitDir	Normalized direction of the sweep.
//\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
//\param[in] bothSides	If mesh both triangle sides should be checked.
//\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
//\return True if hit was found
using SweepShapeAllFn = std::function<void(
    uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)>;

// Overlap test of a sphere against objects in the physics scene
//\param[in] pos	    Sphere position
//\param[in] radius	    Sphere radius
//\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
// any overlap or 0 if there is no overlap at all. \return Number of overlaps found
using OverlapSphereFn = std::function<uint32_t(float radius, const carb::Float3& pos, OverlapHitReportFn reportFn)>;

// Overlap test of a sphere against objects in the physics scene, returns only boolean if hit was found or not
//\param[in] pos	    Sphere position
//\param[in] radius	    Sphere radius
//\return True if overlap found
using OverlapSphereAnyFn = std::function<bool(float radius, const carb::Float3& pos)>;

// Overlap test of a box against objects in the physics scene
//\param[in] halfExtent Box half extent
//\param[in] pos	    Box position
//\param[in] rot	    Box rotation (quaternion x, y, z, w)
//\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
// any overlap or 0 if there is no overlap at all. \return Number of overlaps found
using OverlapBoxFn = std::function<uint32_t(
    const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, OverlapHitReportFn reportFn)>;

// Overlap test of a box against objects in the physics scene, returns only boolean if hit was found or not
//\param[in] halfExtent Box half extent
//\param[in] pos	    Box position
//\param[in] rot	    Box rotation (quaternion x, y, z, w)
//\return True if overlap found
using OverlapBoxAnyFn =
    std::function<bool(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot)>;

// Overlap test of a UsdGeomGPrim against objects in the physics scene
//\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
//\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
// any overlap or 0 if there is no overlap at all. \return Number of overlaps found
using OverlapShapeFn = std::function<uint32_t(uint64_t gPrimPath, OverlapHitReportFn reportFn)>;

// Overlap test of a UsdGeomGPrim against objects in the physics scene, returns only boolean if hit was found or not
//\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
//\return True if overlap found
using OverlapShapeAnyFn = std::function<bool(uint64_t gPrimPath)>;


struct SceneQueryFns
{
    SceneQueryFns()
        : raycastClosest(nullptr),
          raycastAny(nullptr),
          raycastAll(nullptr),
          sweepSphereClosest(nullptr),
          sweepSphereAny(nullptr),
          sweepSphereAll(nullptr),
          sweepBoxClosest(nullptr),
          sweepBoxAny(nullptr),
          sweepBoxAll(nullptr),
          sweepShapeClosest(nullptr),
          sweepShapeAny(nullptr),
          sweepShapeAll(nullptr),
          overlapSphere(nullptr),
          overlapSphereAny(nullptr),
          overlapBox(nullptr),
          overlapBoxAny(nullptr),
          overlapShape(nullptr),
          overlapShapeAny(nullptr)
    {
    }

    RaycastClosestFn raycastClosest;
    RaycastAnyFn raycastAny;
    RaycastAllFn raycastAll;
    SweepSphereClosestFn sweepSphereClosest;
    SweepSphereAnyFn sweepSphereAny;
    SweepSphereAllFn sweepSphereAll;
    SweepBoxClosestFn sweepBoxClosest;
    SweepBoxAnyFn sweepBoxAny;
    SweepBoxAllFn sweepBoxAll;
    SweepShapeClosestFn sweepShapeClosest;
    SweepShapeAnyFn sweepShapeAny;
    SweepShapeAllFn sweepShapeAll;
    OverlapSphereFn overlapSphere;
    OverlapSphereAnyFn overlapSphereAny;
    OverlapBoxFn overlapBox;
    OverlapBoxAnyFn overlapBoxAny;
    OverlapShapeFn overlapShape;
    OverlapShapeAnyFn overlapShapeAny;
};

} // namespace physics
} // namespace omni
