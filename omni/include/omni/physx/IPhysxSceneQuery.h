// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "PhysxConvexMesh.h"

#include <functional>


namespace omni
{
namespace physx
{

struct CollisionShapeAxis
{
    enum Enum
    {
        eX = 0,
        eY = 1,
        eZ = 2
    };
};

// Sphere shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] radius		    Sphere radius
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using SphereShapeReportFn = std::function<void(
    uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, float radius, void* userData)>;

// Box shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] halfExtent		Box half extent
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using BoxShapeReportFn = std::function<void(
    uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, const carb::Float3& halfExtent, void* userData)>;

// Capsule shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] radius		    Capsule radius
//\param[in] height		    Capsule height
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using CapsuleShapeReportFn = std::function<void(uint64_t sdfPath,
                                                const carb::Float3& position,
                                                const carb::Float4& orientation,
                                                CollisionShapeAxis::Enum,
                                                float radius,
                                                float height,
                                                void* userData)>;

// Cone shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] radius		    Cone radius
//\param[in] height		    Cone height
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using ConeShapeReportFn = std::function<void(uint64_t sdfPath,
                                             const carb::Float3& position,
                                             const carb::Float4& orientation,
                                             CollisionShapeAxis::Enum,
                                             float radius,
                                             float height,
                                             void* userData)>;

// Cylinder shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] radius		    Cylinder radius
//\param[in] height		    Cylinder height
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using CylinderShapeReportFn = std::function<void(uint64_t sdfPath,
                                                 const carb::Float3& position,
                                                 const carb::Float4& orientation,
                                                 CollisionShapeAxis::Enum,
                                                 float radius,
                                                 float height,
                                                 void* userData)>;

// Convex mesh shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] meshScale	    Mesh scale
//\param[in] numVertices	Number of vertices
//\param[in] vertices		Vertices buffer
//\param[in] indices		Indices buffer
//\param[in] numPolygons	Number of polygons
//\param[in] polygons		Polygons buffer
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using ConvexMeshShapeReportFn = std::function<void(uint64_t sdfPath,
                                                   const carb::Float3& position,
                                                   const carb::Float4& orientation,
                                                   const carb::Float3& meshScale,
                                                   uint32_t numVertices,
                                                   const carb::Float3* vertices,
                                                   const uint8_t* indices,
                                                   uint32_t numPolygons,
                                                   const ConvexMeshPolygon* polygons,
                                                   void* userData)>;

// Triangle mesh shape report function
//\param[in] sdfPath		Collision shape SdfPath
//\param[in] position	    Collision shape world space position
//\param[in] orientation    Collision shape world space orientation
//\param[in] meshScale	    Mesh scale
//\param[in] numVertices	Number of vertices
//\param[in] vertices		Vertices buffer
//\param[in] numTriangles	Number of triangles
//\param[in] triangles		Triangles buffer
//\param[in] userData       User data passed to ICollisionShapeQueryCallback struct
using TriangleMeshShapeReportFn = std::function<void(uint64_t sdfPath,
                                                     const carb::Float3& position,
                                                     const carb::Float4& orientation,
                                                     const carb::Float3& meshScale,
                                                     uint32_t numVertices,
                                                     const carb::Float3* vertices,
                                                     uint32_t numTriangles,
                                                     const uint32_t* triangles,
                                                     void* userData)>;

// Collision shape report callback
//
//\note Life time of the buffers is only the callback function
struct ICollisionShapeQueryCallback
{
    SphereShapeReportFn sphereShapeReportFn = { nullptr };
    BoxShapeReportFn boxShapeReportFn = { nullptr };
    CapsuleShapeReportFn capsuleShapeReportFn = { nullptr };
    ConeShapeReportFn coneShapeReportFn = { nullptr };
    CylinderShapeReportFn cylinderShapeReportFn = { nullptr };
    ConvexMeshShapeReportFn convexMeshShapeReportFn = { nullptr };
    TriangleMeshShapeReportFn triangleMeshShapeReportFn = { nullptr };

    void* userData = { nullptr };
};

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

/// Scene query interface
struct IPhysxSceneQuery
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxSceneQuery", 1, 0)

    // Raycast physics scene, return the closest hit found
    //\param[in] origin		Origin of the ray.
    //\param[in] unitDir	Normalized direction of the ray.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[out] hit	    Raycast hit report
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* raycastClosest)(
        const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& hit, bool bothSides);

    // Raycast physics scene, returns all hits found in a raycast callback
    //\param[in] origin		Origin of the ray.
    //\param[in] unitDir	Normalized direction of the ray.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* raycastAll)(const carb::Float3& origin,
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

    // Sweep test of a sphere against all objects in the physics scene, returning all the hits found.
    //\param[in] radius		Sphere radius.
    //\param[in] origin		Origin of the sweep.
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepSphereAll)(float radius,
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
    uint32_t(CARB_ABI* overlapSphere)(float radius, const carb::Float3& pos, OverlapHitReportFn reportFn, bool anyHit);

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
                                   OverlapHitReportFn reportFn,
                                   bool anyHit);

    // Overlap test of a mesh against objects in the physics scene
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] meshPath   Mesh path encoded in uint64_t
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] anyHit	    If true, the query will not report individual overlaps in reportFn but return 1 if there is
    // any overlap or 0 if there is no overlap at all. \return Number of overlaps found
    uint32_t(CARB_ABI* overlapMesh)(uint64_t meshPath, OverlapHitReportFn reportFn, bool anyHit);

    // Raycast any physics scene, returns only boolean if hit was found or not
    //\param[in] origin		Origin of the ray.
    //\param[in] unitDir	Normalized direction of the ray.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* raycastAny)(const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides);

    // Sweep test of a sphere against all objects in the physics scene, returning whether any hit was found.
    //\param[in] radius		Sphere radius.
    //\param[in] origin		Origin of the sweep.
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepSphereAny)(
        float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides);

    // Overlap test of a sphere against objects in the physics scene, reports only boolean
    //\param[in] pos	    Sphere position
    //\param[in] radius	    Sphere radius
    //\return True if overlap found
    bool(CARB_ABI* overlapSphereAny)(float radius, const carb::Float3& pos);

    // Overlap test of a box against objects in the physics scene, reports only boolean
    //\param[in] halfExtent Box half extent
    //\param[in] pos	    Box position
    //\param[in] rot	    Box rotation (quaternion x, y, z, w)
    //\return True if overlap found
    bool(CARB_ABI* overlapBoxAny)(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot);

    // Overlap test of a mesh against objects in the physics scene, reports only boolean
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] meshPath   Mesh path encoded in uint64_t
    //\return True if overlap found
    bool(CARB_ABI* overlapMeshAny)(uint64_t meshPath);

    // Report collision shapes created on given path and its children
    //
    //\param[in] path               SdfPath to start traversal
    //\param[in] reportCallback     Report callback to send collision shape information
    //\return Number of shapes reported
    uint32_t(CARB_ABI* reportCollisionShapes)(uint64_t path, ICollisionShapeQueryCallback& reportCallback);

    // Overlap test of a UsdGeomGPrim against objects in the physics scene
    //
    // \note A convex mesh approximation will be used for meshes, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] gPrimPath  UsdGeomGPrim path encoded in uint64_t
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] anyHit	    If true, the query will not report individual overlaps in reportFn but return 1 if there is
    // any overlap or 0 if there is no overlap at all. \return Number of overlaps found
    uint32_t(CARB_ABI* overlapShape)(uint64_t gPrimPath, OverlapHitReportFn reportFn, bool anyHit);

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

    // Sweep test of a mesh against all objects in the physics scene, returning the closest hit found.
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] meshPath   Mesh path encoded in uint64_t
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[out] hit	    Sweep hit report
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepMeshClosest)(
        uint64_t meshPath, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides);

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

    // Sweep test of a mesh against all objects in the physics scene, returning whether any hit was found.
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] meshPath   Mesh path encoded in uint64_t
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepMeshAny)(uint64_t meshPath, const carb::Float3& unitDir, float distance, bool bothSides);

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
    //\return True if hit was found
    bool(CARB_ABI* sweepBoxAll)(const carb::Float3& halfExtent,
                                const carb::Float3& pos,
                                const carb::Float4& rot,
                                const carb::Float3& unitDir,
                                float distance,
                                SweepHitReportFn reportFn,
                                bool bothSides);

    // Sweep test of a mesh against all objects in the physics scene, returning all the hits found.
    //
    // \note A convex mesh approximation will be used for the test, the first query will compute the convex mesh
    // approximation and store the result in a local cache
    //
    //\param[in] meshPath   Mesh path encoded in uint64_t
    //\param[in] unitDir	Normalized direction of the sweep.
    //\param[in] distance	Length of the ray. Has to be in the[0, inf) range.
    //\param[in] reportFn   Scene query hit report function, return True to continue traversal, False to stop traversal
    //\param[in] bothSides	If mesh both triangle sides should be checked.
    //\return True if hit was found
    bool(CARB_ABI* sweepMeshAll)(
        uint64_t meshPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides);

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
    //\return True if hit was found
    bool(CARB_ABI* sweepShapeAll)(
        uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides);
};

} // namespace physx
} // namespace omni
