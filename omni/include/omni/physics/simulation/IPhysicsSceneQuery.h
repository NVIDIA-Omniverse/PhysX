// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "simulator/SceneQuery.h"

namespace omni
{

namespace physics
{

/// Scene query interface
struct IPhysicsSceneQuery
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysicsSceneQuery", 0, 1)

    // Raycast physics scene, return the closest hit found
    //\param[in] origin		Origin of the ray.
    //\param[in] unitDir	Normalized direction of the ray.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[out] hit	    Raycast hit report
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* raycastClosest)(
        const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& hit, bool bothSides);

    // Raycast any physics scene, returns only boolean if hit was found or not
    //\param[in] origin		Origin of the ray.
    //\param[in] unitDir	Normalized direction of the ray.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* raycastAny)(const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides);

    // Raycast physics scene, returns all hits found in a raycast callback
    //\param[in] origin		Origin of the ray.
    //\param[in] unitDir	Normalized direction of the ray.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    void(CARB_ABI* raycastAll)(const carb::Float3& origin,
                               const carb::Float3& unitDir,
                               float distance,
                               RaycastHitReportFn reportFn,
                               bool bothSides);

    // Sweep test of a sphere against all objects in the physics scene, returning the closest hit found.
    //\param[in] radius		Sphere radius.
    //\param[in] origin		Origin of the sweep.
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[out] hit	    Sweep hit report
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepSphereClosest)(float radius,
                                       const carb::Float3& origin,
                                       const carb::Float3& unitDir,
                                       float distance,
                                       SweepHit& hit,
                                       bool bothSides);

    // Sweep test of a sphere against all objects in the physics scene, returning whether any hit was found.
    //\param[in] radius		Sphere radius.
    //\param[in] origin		Origin of the sweep.
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepSphereAny)(
        float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides);

    // Sweep test of a sphere against all objects in the physics scene, returning all the hits found.
    //\param[in] radius		Sphere radius.
    //\param[in] origin		Origin of the sweep.
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    void(CARB_ABI* sweepSphereAll)(float radius,
                                   const carb::Float3& origin,
                                   const carb::Float3& unitDir,
                                   float distance,
                                   SweepHitReportFn reportFn,
                                   bool bothSides);

    // Overlap test of a sphere against objects in the physics scene
    //\param[in] pos	    Sphere position
    //\param[in] radius	    Sphere radius
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] anyHit	    If true, the query will not report individual overlaps in reportFn but return 1 if there is
    // any overlap or 0 if there is no overlap at all. \return Number of overlaps found
    uint32_t(CARB_ABI* overlapSphere)(float radius, const carb::Float3& pos, OverlapHitReportFn reportFn);

    // Overlap test of a sphere against objects in the physics scene, reports only boolean
    //\param[in] pos	    Sphere position
    //\param[in] radius	    Sphere radius
    //\return True if overlap found
    bool(CARB_ABI* overlapSphereAny)(float radius, const carb::Float3& pos);

    // Overlap test of a box against objects in the physics scene
    //\param[in] halfExtent Box half extent
    //\param[in] pos	    Box position
    //\param[in] rot	    Box rotation (quaternion x, y, z, w)
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] anyHit	    If true, the query will not report individual overlaps in reportFn but return 1 if there is
    // any overlap or 0 if there is no overlap at all. \return Number of overlaps found
    uint32_t(CARB_ABI* overlapBox)(const carb::Float3& halfExtent,
                                   const carb::Float3& pos,
                                   const carb::Float4& rot,
                                   OverlapHitReportFn reportFn);

    // Overlap test of a box against objects in the physics scene, reports only boolean
    //\param[in] halfExtent Box half extent
    //\param[in] pos	    Box position
    //\param[in] rot	    Box rotation (quaternion x, y, z, w)
    //\return True if overlap found
    bool(CARB_ABI* overlapBoxAny)(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot);


    // Overlap test of a UsdGeomGPrim against objects in the physics scene
    //
    // \note A convex mesh approximation will be used for meshes, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] anyHit	    If true, the query will not report individual overlaps in reportFn but return 1 if there is
    // any overlap or 0 if there is no overlap at all. \return Number of overlaps found
    uint32_t(CARB_ABI* overlapShape)(uint64_t gPrimPath, OverlapHitReportFn reportFn);

    // Overlap test of a mesh against objects in the physics scene, reports only boolean
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] gPrimPath   UsdGeomGPrim path encoded in uint64_t
    //\return True if overlap found
    bool(CARB_ABI* overlapShapeAny)(uint64_t gPrimPath);

    // Sweep test of a box against all objects in the physics scene, returning the closest hit found.
    //\param[in] halfExtent Box half extent
    //\param[in] pos	    Box position. This is the origin of the sweep.
    //\param[in] rot	    Box rotation (quaternion x, y, z, w)
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[out] hit	    Sweep hit report
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepBoxClosest)(const carb::Float3& halfExtent,
                                    const carb::Float3& pos,
                                    const carb::Float4& rot,
                                    const carb::Float3& unitDir,
                                    float distance,
                                    SweepHit& hit,
                                    bool bothSides);

    // Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning the closest hit found.
    //
    // \note A convex mesh approximation will be used for meshes, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[out] hit	    Sweep hit report
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepShapeClosest)(
        uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides);

    // Sweep test of a box against all objects in the physics scene, returning whether any hit was found.
    //\param[in] halfExtent Box half extent
    //\param[in] pos	    Box position. This is the origin of the sweep.
    //\param[in] rot	    Box rotation (quaternion x, y, z, w)
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepBoxAny)(const carb::Float3& halfExtent,
                                const carb::Float3& pos,
                                const carb::Float4& rot,
                                const carb::Float3& unitDir,
                                float distance,
                                bool bothSides);

    // Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning whether any hit was found.
    //
    // \note A convex mesh approximation will be used for meshes, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepShapeAny)(uint64_t gPrimPath, const carb::Float3& unitDir, float distance, bool bothSides);

    // Sweep test of a box against all objects in the physics scene, returning all the hits found.
    //\param[in] halfExtent Box half extent
    //\param[in] pos	    Box position. This is the origin of the sweep.
    //\param[in] rot	    Box rotation (quaternion x, y, z, w)
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    void(CARB_ABI* sweepBoxAll)(const carb::Float3& halfExtent,
                                const carb::Float3& pos,
                                const carb::Float4& rot,
                                const carb::Float3& unitDir,
                                float distance,
                                SweepHitReportFn reportFn,
                                bool bothSides);

    // Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning all the hits found.
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    void(CARB_ABI* sweepShapeAll)(
        uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides);
};

} // namespace physics
} // namespace omni
