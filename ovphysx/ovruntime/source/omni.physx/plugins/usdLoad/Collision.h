// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physx/IPhysxCookingService.h>

#include "LoadTools.h"

namespace omni
{
namespace physics
{
namespace schema
{
struct ShapeDesc;
}
} // namespace physics
namespace physx
{
namespace usdparser
{

class AttachedStage;

// ObjectKey-native entry point: both real callers (usdLoad/LoadStage.cpp's
// shape-finalization loop, usdLoad/PointInstancer.cpp's ScannedStage path)
// already hold an ObjectKey list here; no SdfPath-taking overload is needed
// (PointInstancer.cpp's was the last caller, retyped to ObjectKey -- ADR-0018).
void finalizeShape(AttachedStage& attachedStage, PhysxShapeDesc* desc,
                   const std::vector<omni::physics::parse::ObjectKey>& materials);

// ObjectKey-native entry point (usdLoad/LoadStage.cpp's and usdLoad/
// PointInstancer.cpp's callers already hold the ObjectKey directly -- see
// createShape's definition for the ObjectDb registration note).
PhysxRigidBodyDesc* createShape(AttachedStage& attachedStage,
                                omni::physics::parse::ObjectKey key,
                                PhysxShapeDesc* shapeDesc,
                                const ObjectInstance* objectInstance,
                                ObjectId* shapeId = nullptr);

// Key-based cooking-desc fills for source-backed consumers. `attachedStage`
// resolves `meshKey` to the mesh prim path (stored on `desc.meshPrimKey`) and the
// cooking-input geometry is read through IPhysicsSource
// (fillCookingRequestFromSourceMesh / fillCookingMeshViewFromSource), so no
// UsdPrim is materialized here. All of them return false for a null
// `attachedStage` or an invalid `meshKey`.
bool fillConvexMeshDesc(AttachedStage* attachedStage,
                        omni::physics::parse::ObjectKey meshKey,
                        omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                        const omni::physx::ConvexMeshCookingParams& cookingParams);
bool fillTriangleMeshDesc(AttachedStage* attachedStage,
                          omni::physics::parse::ObjectKey meshKey,
                          omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                          const omni::physx::TriangleMeshCookingParams& cookingParams);
bool fillSdfTriangleMeshDesc(AttachedStage* attachedStage,
                             omni::physics::parse::ObjectKey meshKey,
                             omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                             const omni::physx::SdfMeshCookingParams& cookingParams);
bool fillConvexDecompositionDesc(AttachedStage* attachedStage,
                                 omni::physics::parse::ObjectKey meshKey,
                                 omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
                                 const omni::physx::ConvexDecompositionCookingParams& cookingParams);
bool fillSphereFillDesc(AttachedStage* attachedStage,
                        omni::physics::parse::ObjectKey meshKey,
                        omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
                        const omni::physx::SphereFillCookingParams& cookingParams);

// Cooking-input de-USD (Stage C). RAII holder that keeps the source-owned mesh
// buffers alive while a cooking request's primMeshView points into them; releases
// on scope exit (after the synchronous request submission that copies the view).
struct SourceMeshGeometryScope
{
    const omni::physics::parse::IPhysicsSource* src = nullptr;
    omni::physics::parse::MeshGeometry geom;
    SourceMeshGeometryScope() = default;
    SourceMeshGeometryScope(const SourceMeshGeometryScope&) = delete;
    SourceMeshGeometryScope& operator=(const SourceMeshGeometryScope&) = delete;
    ~SourceMeshGeometryScope();
};

// Read mesh geometry for `meshKey` through IPhysicsSource and point
// `request.primMeshView` at it, switching the request to FROM_PRIM_MESH_VIEW so
// the cooking service consumes the provided geometry instead of reading USD.
// `scope` must outlive the (synchronous) cooking submission. Returns false and
// leaves `request` unchanged when the source/geometry is unavailable (caller then
// falls back to the prim-id path).
// includeFaceMaterials=false skips the per-mesh GeomSubset face-material walk
// (single-material cooking: convex / decomposition / sphere-fill).
bool fillCookingMeshViewFromSource(omni::physx::PhysxCookingComputeRequest& request,
                                   SourceMeshGeometryScope& scope,
                                   const AttachedStage& attachedStage,
                                   omni::physics::parse::ObjectKey meshKey,
                                   bool includeFaceMaterials = true);

PhysxShapeDesc* scaleShapeDesc(const PhysxShapeDesc& inDesc, const carb::Float3& scale);

// Bounding-shape compute helpers — fit a sphere / OBB around `points`.
// scanStage emits eBoundingSphereShape / eBoundingBoxShape descs with
// the geometry zeroed; LoadStage populates it via these helpers.
BoundingSpherePhysxShapeDesc* computeBoundingSphereShape(const std::vector<carb::Float3>& points);
BoundingBoxPhysxShapeDesc*    computeBoundingBoxShape(const std::vector<carb::Float3>& points);

// release collision shape desc - specialized function, has to release more memory
// A.B. TODO - this is most likely leaking when used from debug draw
// we should try to move the release into a destructor probably
void releaseShapeDesc(PhysxShapeDesc* desc);

void notifyStageReset(void);
// ObjectKey-native entry point: the one external caller (CookingDataAsync.cpp's
// addPrimRefreshSet) already holds the ObjectKey directly and previously
// materialized a PXR_NS::SdfPath purely to call this.
void invalidateMeshKeyCache(omni::physics::parse::ObjectKey key);
} // namespace usdparser
} // namespace physx
} // namespace omni
