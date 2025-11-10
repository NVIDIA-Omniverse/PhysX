// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <omni/physx/IPhysxVisualization.h>

#include <PxPhysicsAPI.h>

namespace omni
{
namespace physx
{

struct CachedLines
{
    DebugLine* mDebugLines;
    ::physx::PxTransform mTransform;
    uint32_t mNumLines;
};

struct DebugEdge
{
    uint32_t v[2];
};

struct CachedEdges
{
    DebugEdge* mDebugEdges;
    uint32_t mNumEdges;
};

typedef pxr::TfHashMap<pxr::SdfPath, CachedLines, pxr::SdfPath::Hash> LineMap;
typedef pxr::TfHashMap<pxr::SdfPath, CachedEdges, pxr::SdfPath::Hash> EdgeMap;

class DebugVisualizationCache
{
public:
    DebugVisualizationCache() = default;

    ~DebugVisualizationCache()
    {
        release();
    }

    void release();

    void releasePath(const pxr::SdfPath& path);

    const DebugLine* getLines(const pxr::SdfPath& path, const ::physx::PxTransform& transform, uint32_t& numLines);

    void addLines(const pxr::SdfPath& path, const ::physx::PxTransform& transform, DebugLine* debugLines, uint32_t numLines);

    const DebugEdge* getEdges(const pxr::SdfPath& path, uint32_t& numEdges);

    void addEdges(const pxr::SdfPath& path, DebugEdge* debugEdges, uint32_t numEdges);

public:
    std::vector<DebugPoint> mPointsBuffer;
    std::vector<DebugLine> mLinesBuffer;
    std::vector<DebugTriangle> mTriangleBuffer;

private:
    LineMap mLineMap;
    EdgeMap mEdgeMap;
};

void enableVisualization(bool enableVis);
void enableNormalsVisualization(bool enableNormalsVis);

void setVisualizationScale(float scale);

void setVisualizationCullingBox(const carb::Float3& min, const carb::Float3& max);

void setVisualizationParameter(PhysXVisualizationParameter par, bool val);

uint32_t getNbPoints();

const DebugPoint* getPoints();

uint32_t getNbLines();

const DebugLine* getLines();

uint32_t getNbTriangles();

const DebugTriangle* getTriangles();

const DebugLine* getShapeDebugDraw(const pxr::SdfPath& primPath, const usdparser::PhysxShapeDesc* desc, uint32_t& numLines);

void getParticleClothDebugDrawDeprecated(const pxr::SdfPath& primPath,
                                         usdparser::ParticleClothDesc* desc,
                                         pxr::VtArray<uint32_t>& numPoints,
                                         bool invalidateCache);
const CollisionRepresentation* getCollisionRepresentation(const pxr::SdfPath& usdPath,
                                                          const usdparser::PhysxShapeDesc* desc);
void releaseCollisionRepresentation(const CollisionRepresentation* cr);
bool getMeshKey(const omni::physx::usdparser::PhysxShapeDesc& desc, omni::physx::usdparser::MeshKey& meshKey);

void getFEMClothDebugDrawDeprecated(pxr::VtArray<uint32_t>& pointIndices, const pxr::SdfPath& primPath);

void clearDebugVisualizationData();

::physx::PxU32 getDebugDrawCollShapeColor(const pxr::SdfPath& primPath);

} // namespace physx
} // namespace omni
