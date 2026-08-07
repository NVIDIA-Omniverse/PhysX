// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

typedef PXR_NS::TfHashMap<PXR_NS::SdfPath, CachedLines, PXR_NS::SdfPath::Hash> LineMap;
typedef PXR_NS::TfHashMap<PXR_NS::SdfPath, CachedEdges, PXR_NS::SdfPath::Hash> EdgeMap;

class DebugVisualizationCache
{
public:
    DebugVisualizationCache() = default;

    ~DebugVisualizationCache()
    {
        release();
    }

    void release();

    void releasePath(const PXR_NS::SdfPath& path);

    const DebugLine* getLines(const PXR_NS::SdfPath& path, const ::physx::PxTransform& transform, uint32_t& numLines);

    void addLines(const PXR_NS::SdfPath& path, const ::physx::PxTransform& transform, DebugLine* debugLines, uint32_t numLines);

    const DebugEdge* getEdges(const PXR_NS::SdfPath& path, uint32_t& numEdges);

    void addEdges(const PXR_NS::SdfPath& path, DebugEdge* debugEdges, uint32_t numEdges);

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
void setVisualizationParameterValue(PhysXVisualizationParameter par, float value);
// Restrict eVISUALIZATION authoring to a set of interned prim paths (exact
// membership; empty = every object). Needs the ovstage dictionary.
bool setVisualizationScopeTokens(const ovx_primpath_t* tokens, uint32_t count);

uint32_t getNbPoints();

const DebugPoint* getPoints();

uint32_t getNbLines();

const DebugLine* getLines();

uint32_t getNbTriangles();

const DebugTriangle* getTriangles();

const DebugLine* getShapeDebugDraw(const PXR_NS::SdfPath& primKey, const usdparser::PhysxShapeDesc* desc, uint32_t& numLines);

const CollisionRepresentation* getCollisionRepresentation(const PXR_NS::SdfPath& usdPath,
                                                          const usdparser::PhysxShapeDesc* desc);
void releaseCollisionRepresentation(const CollisionRepresentation* cr);
bool getMeshKey(const omni::physx::usdparser::PhysxShapeDesc& desc, omni::physx::usdparser::MeshKey& meshKey);

void clearDebugVisualizationData();

::physx::PxU32 getDebugDrawCollShapeColor(const PXR_NS::SdfPath& primKey);

} // namespace physx
} // namespace omni
