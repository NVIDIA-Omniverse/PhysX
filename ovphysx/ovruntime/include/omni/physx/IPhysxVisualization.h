// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-IO-001
// @covers AC-2 AC-3
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-14
 */

#pragma once

#include <omni/physics/AttachHandle.h>
#include <carb/Defines.h>
#include <carb/Types.h>
#include <ovx/types.h>  // ovx_primpath_t (interned prim-path handle for the token scope)

#include <omni/physics/parse/Handles.h> // ObjectKey

using namespace carb;

namespace omni
{

namespace physx
{

} // namespace physx
namespace physics
{
namespace parse
{
struct PhysxShapeDesc;
} // namespace parse
} // namespace physics
namespace physx
{

namespace usdparser
{
using PhysxShapeDesc = ::omni::physics::parse::PhysxShapeDesc;
class MeshKey;
} // namespace usdparser

/**
\brief Used to store a single point and colour for debug rendering.
*/
struct DebugPoint
{
    DebugPoint() = default;

    DebugPoint(const carb::Float3& p, const uint32_t& c) : mPos(p), mColor(c)
    {
    }

    Float3 mPos;
    uint32_t mColor;
};

/**
\brief Used to store a single line and colour for debug rendering.
*/
struct DebugLine
{
    DebugLine() = default;

    DebugLine(const carb::Float3& p0, const carb::Float3& p1, const uint32_t& c)
        : mPos0(p0), mColor0(c), mPos1(p1), mColor1(c)
    {
    }

    Float3 mPos0;
    uint32_t mColor0;
    Float3 mPos1;
    uint32_t mColor1;
};

/**
\brief Used to store a single triangle and colour for debug rendering.
*/
struct DebugTriangle
{
    DebugTriangle() = default;

    DebugTriangle(const carb::Float3& p0, const carb::Float3& p1, const carb::Float3& p2, const uint32_t& c)
        : mPos0(p0), mColor0(c), mPos1(p1), mColor1(c), mPos2(p2), mColor2(c)
    {
    }

    Float3 mPos0;
    uint32_t mColor0;
    Float3 mPos1;
    uint32_t mColor1;
    Float3 mPos2;
    uint32_t mColor2;
};

enum PhysXVisualizationParameter
{
    eNone = 0,
    eWorldAxes,
    eBodyAxes,
    eBodyMassAxes,
    eBodyLinearVelocity,
    eBodyAngularVelocity,
    eContactPoint,
    eContactNormal,
    eContactError,
    eContactImpulse,
    eFrictionPoint,
    eFrictionNormal,
    eFrictionImpulse,
    eActorAxes,
    eCollisionAABBs,
    eCollisionShapes,
    eCollisionAxes,
    eCollisionCompounds,
    eCollisionFaceNormals,
    eCollisionEdges,
    eCollisionStaticPruner,
    eCollisionDynamicPruner,
    eJointLocalFrames,
    eJointLimits,
    eCullBox,
    eMBPRegions,
    eSimulationMesh,
    eSDF,
    ePhysXNumValues,
    eSplinesSurfaceVelocitySegments, // omni physx custom visualization should be after PhysX debug vis
    eSplinesSurfaceVelocity, // omni physx custom visualization should be after PhysX debug vis
    eNumValues // to test against PxVisualizationParameter::eNUM_VALUES
};

struct CollisionMesh
{
    uint32_t vertexCount{ 0 };
    float* vertices{ nullptr };
    uint32_t triangleCount{ 0 };
    uint32_t* indices{ nullptr };
    carb::Float3 meshCenter{}; // only for results of a convex decomposition. The center of this collision mesh
};

struct CollisionRepresentation
{
    uint32_t meshCount{ 0 }; // The number of triangle meshes
    carb::Float3 bmin{}; // the bounding box of the source mesh
    carb::Float3 bmax{};
    carb::Float3 rootCenter{}; // only for results of a convex decomposition. The center of the original mesh
    CollisionMesh* meshes{ nullptr }; // the array of collision meshes
};

// One ovphysx instance's debug-render settings, applied only to PhysX scenes
// whose attach handle matches attachHandle. attachHandle 0 matches nothing.
struct DebugRenderAttachRequest
{
    uint64_t attachHandle;
    bool enabled;
    float scale;
    const float* params;
    uint32_t paramCount;
    bool hasCullingBox;
    float cullMin[3];
    float cullMax[3];
    const ovx_primpath_t* scopeTokens;
    uint32_t scopeCount;
};

// Every stage-dependent entry requires an exact live attachment and idle runtime.
// Rejected calls leave caller outputs and attachment visualization state unchanged.
struct IPhysxVisualization
{
    void(CARB_ABI* enableVisualization)(omni::physics::AttachHandle attachHandle, bool enableVis);
    void(CARB_ABI* enableNormalsVisualization)(omni::physics::AttachHandle attachHandle, bool enableNormalsVis);

    void(CARB_ABI* setVisualizationScale)(omni::physics::AttachHandle attachHandle, float scale);

    void(CARB_ABI* setVisualizationCullingBox)(omni::physics::AttachHandle attachHandle, const carb::Float3& min, const carb::Float3& max);

    void(CARB_ABI* setVisualizationParameter)(omni::physics::AttachHandle attachHandle, PhysXVisualizationParameter par, bool val);

    uint32_t(CARB_ABI* getNbPoints)(omni::physics::AttachHandle attachHandle);

    const DebugPoint*(CARB_ABI* getPoints)(omni::physics::AttachHandle attachHandle);

    uint32_t(CARB_ABI* getNbLines)(omni::physics::AttachHandle attachHandle);
    const DebugLine*(CARB_ABI* getLines)(omni::physics::AttachHandle attachHandle);

    uint32_t(CARB_ABI* getNbTriangles)(omni::physics::AttachHandle attachHandle);
    const DebugTriangle*(CARB_ABI* getTriangles)(omni::physics::AttachHandle attachHandle);

    const DebugLine*(CARB_ABI* getShapeDebugDraw)(omni::physics::AttachHandle attachHandle, omni::physics::parse::ObjectKey key,
                                                  const usdparser::PhysxShapeDesc* desc,
                                                  uint32_t& numLines);

    // Get the visual mesh representation for the collision volumes relative to this prim
    const CollisionRepresentation*(CARB_ABI* getCollisionRepresentation)(omni::physics::AttachHandle attachHandle, omni::physics::parse::ObjectKey key,
                                                                         const usdparser::PhysxShapeDesc* desc);

    // Release the collision representation
    void(CARB_ABI* releaseCollisionRepresentation)(const CollisionRepresentation* cr);

    bool(CARB_ABI* getMeshKey)(const omni::physx::usdparser::PhysxShapeDesc& desc,
                               omni::physx::usdparser::MeshKey& meshKey);

    void(CARB_ABI* clearDebugVisualizationData)(omni::physics::AttachHandle attachHandle);

    /// returns debug draw color for the prim
    ///
    /// \param[in] key                  ObjectKey identifying the prim
    uint32_t(CARB_ABI* getDebugDrawCollShapeColor)(omni::physics::AttachHandle attachHandle, omni::physics::parse::ObjectKey key);

    // Set a visualization parameter's float value (a magnitude multiplied by eSCALE,
    // e.g. contact-normal length or axis size). Does not change the enabled state; the
    // bool setter applies the cached value on enable. Values must be finite and >= 0.
    void(CARB_ABI* setVisualizationParameterValue)(omni::physics::AttachHandle attachHandle, PhysXVisualizationParameter par, float value);

    // Restrict debug visualization to a set of interned prim paths, exact
    // membership (no prefix expansion; the caller expands a
    // hierarchy into its object set). Empty = every object. Needs the ovstage token
    // dictionary; false leaves the scope unchanged.
    // Appended for ABI stability.
    bool(CARB_ABI* setVisualizationScopeTokens)(omni::physics::AttachHandle attachHandle, const ovx_primpath_t* tokens, uint32_t count);

    // Apply one instance's debug-render settings to scenes of attachHandle only.
    // Appended for ABI stability.
    void(CARB_ABI* applyDebugRenderForAttach)(const DebugRenderAttachRequest* request);

    // Copy render-buffer primitives from scenes of attachHandle only.
    // A null buffer queries the count. A non-null buffer is a capacity: *count
    // is the capacity on entry and the full primitive count on return.
    void(CARB_ABI* copyDebugPrimitivesForAttach)(uint64_t attachHandle,
                                                 DebugPoint* points,
                                                 uint32_t* pointCount,
                                                 DebugLine* lines,
                                                 uint32_t* lineCount,
                                                 DebugTriangle* triangles,
                                                 uint32_t* triangleCount);
};

} // namespace physx
} // namespace omni
