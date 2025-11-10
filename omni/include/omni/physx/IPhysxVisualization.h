// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

using namespace carb;

namespace omni
{

namespace physx
{

namespace usdparser
{
struct PhysxShapeDesc;
struct ParticleClothDesc;
struct SoftBodyDesc;
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
    eContactForce = eContactImpulse, // PxVisualizationParameter::eCONTACT_FORCE is deprecated
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

struct IPhysxVisualization
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxVisualization", 2, 0)

    void(CARB_ABI* enableVisualization)(bool enableVis);
    void(CARB_ABI* enableNormalsVisualization)(bool enableNormalsVis);

    void(CARB_ABI* setVisualizationScale)(float scale);

    void(CARB_ABI* setVisualizationCullingBox)(const carb::Float3& min, const carb::Float3& max);

    void(CARB_ABI* setVisualizationParameter)(PhysXVisualizationParameter par, bool val);

    uint32_t(CARB_ABI* getNbPoints)();

    const DebugPoint*(CARB_ABI* getPoints)();

    uint32_t(CARB_ABI* getNbLines)();
    const DebugLine*(CARB_ABI* getLines)();

    uint32_t(CARB_ABI* getNbTriangles)();
    const DebugTriangle*(CARB_ABI* getTriangles)();

    const DebugLine*(CARB_ABI* getShapeDebugDraw)(const pxr::SdfPath& usdPath,
                                                  const usdparser::PhysxShapeDesc* desc,
                                                  uint32_t& numLines);

    // Get the visual mesh representation for the collision volumes relative to this prim
    const CollisionRepresentation*(CARB_ABI* getCollisionRepresentation)(const pxr::SdfPath& usdPath,
                                                                         const usdparser::PhysxShapeDesc* desc);

    // Release the collision representation
    void(CARB_ABI* releaseCollisionRepresentation)(const CollisionRepresentation* cr);

    bool(CARB_ABI* getMeshKey)(const omni::physx::usdparser::PhysxShapeDesc& desc,
                               omni::physx::usdparser::MeshKey& meshKey);

    void(CARB_ABI* clearDebugVisualizationData)();

    /// returns debug draw color for the prim
    ///
    /// \param[in] primPath             path to a prim
    uint32_t(CARB_ABI* getDebugDrawCollShapeColor)(const pxr::SdfPath& primPath);
};

} // namespace physx
} // namespace omni
