// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <omni/physics/parse/Handles.h>
#include <omni/physx/IPhysxVisualization.h>

#include <PxPhysicsAPI.h>

#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser { class AttachedStage; }

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

typedef std::unordered_map<omni::physics::parse::ObjectKey, CachedLines, omni::physics::parse::ObjectKey::Hash> LineMap;
typedef std::unordered_map<omni::physics::parse::ObjectKey, CachedEdges, omni::physics::parse::ObjectKey::Hash> EdgeMap;

class DebugVisualizationCache
{
public:
    DebugVisualizationCache() { mParamValues.fill(1.0f); }

    ~DebugVisualizationCache()
    {
        release();
    }

    void release();

    void releaseKey(omni::physics::parse::ObjectKey key);

    const DebugLine* getLines(omni::physics::parse::ObjectKey key, const ::physx::PxTransform& transform, uint32_t& numLines);

    void addLines(omni::physics::parse::ObjectKey key, const ::physx::PxTransform& transform, DebugLine* debugLines, uint32_t numLines);

    const DebugEdge* getEdges(omni::physics::parse::ObjectKey key, uint32_t& numEdges);

    void addEdges(omni::physics::parse::ObjectKey key, DebugEdge* debugEdges, uint32_t numEdges);

public:
    std::unordered_set<ovx_primpath_t> mScopePaths;
    std::array<float, ePhysXNumValues> mParamValues;
    std::vector<DebugLine> mShapeBuffer;
    std::vector<DebugPoint> mPointsBuffer;
    std::vector<DebugLine> mLinesBuffer;
    std::vector<DebugTriangle> mTriangleBuffer;

private:
    LineMap mLineMap;
    EdgeMap mEdgeMap;
};

void enableVisualization(AttachHandle attachHandle, bool enableVis);
void enableVisualization(usdparser::AttachedStage& owner, bool enableVis);
void enableNormalsVisualization(AttachHandle attachHandle, bool enableNormalsVis);
void enableNormalsVisualization(usdparser::AttachedStage& owner, bool enableNormalsVis);

void setVisualizationScale(AttachHandle attachHandle, float scale);
void setVisualizationScale(usdparser::AttachedStage& owner, float scale);

void setVisualizationCullingBox(AttachHandle attachHandle, const carb::Float3& min, const carb::Float3& max);
void setVisualizationCullingBox(usdparser::AttachedStage& owner, const carb::Float3& min, const carb::Float3& max);

void setVisualizationParameter(AttachHandle attachHandle, PhysXVisualizationParameter par, bool val);
void setVisualizationParameter(usdparser::AttachedStage& owner, PhysXVisualizationParameter par, bool val);
void setVisualizationParameterValue(AttachHandle attachHandle, PhysXVisualizationParameter par, float value);
void setVisualizationParameterValue(usdparser::AttachedStage& owner, PhysXVisualizationParameter par, float value);
// Restrict eVISUALIZATION authoring to a set of interned prim paths (exact
// membership; empty = every object). Needs the ovstage dictionary.
bool setVisualizationScopeTokens(AttachHandle attachHandle, const ovx_primpath_t* tokens, uint32_t count);
bool setVisualizationScopeTokens(usdparser::AttachedStage& owner, const ovx_primpath_t* tokens, uint32_t count);

uint32_t getNbPoints(AttachHandle attachHandle);
uint32_t getNbPoints(usdparser::AttachedStage& owner);

const DebugPoint* getPoints(AttachHandle attachHandle);
const DebugPoint* getPoints(usdparser::AttachedStage& owner);

uint32_t getNbLines(AttachHandle attachHandle);
uint32_t getNbLines(usdparser::AttachedStage& owner);

const DebugLine* getLines(AttachHandle attachHandle);
const DebugLine* getLines(usdparser::AttachedStage& owner);

uint32_t getNbTriangles(AttachHandle attachHandle);
uint32_t getNbTriangles(usdparser::AttachedStage& owner);

const DebugTriangle* getTriangles(AttachHandle attachHandle);
const DebugTriangle* getTriangles(usdparser::AttachedStage& owner);

const DebugLine* getShapeDebugDraw(AttachHandle attachHandle, omni::physics::parse::ObjectKey key,
                                                const usdparser::PhysxShapeDesc* desc,
                                                uint32_t& numLines);
const DebugLine* getShapeDebugDraw(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key,
                                                const usdparser::PhysxShapeDesc* desc,
                                                uint32_t& numLines);

const omni::physx::CollisionRepresentation* getCollisionRepresentation(AttachHandle attachHandle, omni::physics::parse::ObjectKey key,const usdparser::PhysxShapeDesc* desc);
const omni::physx::CollisionRepresentation* getCollisionRepresentation(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key,const usdparser::PhysxShapeDesc* desc);
void releaseCollisionRepresentation(const CollisionRepresentation* cr);
bool getMeshKey(const omni::physx::usdparser::PhysxShapeDesc& desc, omni::physx::usdparser::MeshKey& meshKey);

void clearDebugVisualizationData(AttachHandle attachHandle);
void clearDebugVisualizationData(usdparser::AttachedStage& owner);

// Apply one instance's debug-render settings to the PhysX scenes created by that
// attach, and no others. attachHandle 0 matches nothing.
void applyDebugRenderForAttach(uint64_t attachHandle,
                               bool enabled,
                               float scale,
                               const float* params,
                               uint32_t paramCount,
                               bool hasCull,
                               const float cullMin[3],
                               const float cullMax[3],
                               const ovx_primpath_t* scopeTokens,
                               uint32_t scopeCount);

// Copy render-buffer primitives from scenes of attachHandle only.
// Spline debug lines come from that attach's database.
// False leaves the vectors unchanged: admission was refused, which is distinct
// from a successful copy of no primitives.
bool copyDebugPrimitivesForAttach(uint64_t attachHandle,
                                  std::vector<DebugPoint>& points,
                                  std::vector<DebugLine>& lines,
                                  std::vector<DebugTriangle>& triangles);

// Caller already holds the execution lease for this owner.
void copyDebugPrimitivesFromOwner(usdparser::AttachedStage& owner,
                                  std::vector<DebugPoint>& points,
                                  std::vector<DebugLine>& lines,
                                  std::vector<DebugTriangle>& triangles);

// Caller already holds the execution lease for this owner.
void applyDebugRenderToOwner(usdparser::AttachedStage& owner,
                             bool enabled,
                             float scale,
                             const float* params,
                             uint32_t paramCount,
                             bool hasCull,
                             const float cullMin[3],
                             const float cullMax[3],
                             const ovx_primpath_t* scopeTokens,
                             uint32_t scopeCount);

void applyDebugRenderForAttachRequest(const DebugRenderAttachRequest* request);

void copyDebugPrimitivesForAttachBuffers(uint64_t attachHandle,
                                         DebugPoint* points,
                                         uint32_t* pointCount,
                                         DebugLine* lines,
                                         uint32_t* lineCount,
                                         DebugTriangle* triangles,
                                         uint32_t* triangleCount);

uint32_t getDebugDrawCollShapeColor(AttachHandle attachHandle, omni::physics::parse::ObjectKey key);
uint32_t getDebugDrawCollShapeColor(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key);

} // namespace physx
} // namespace omni
