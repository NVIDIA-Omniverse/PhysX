// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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

bool isCollisionShape(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& prim);

void finalizeShape(AttachedStage& attachedStage, PhysxShapeDesc* desc, const PXR_NS::SdfPathVector& materials);
PhysxRigidBodyDesc* createShape(AttachedStage& attachedStage,
                                const PXR_NS::SdfPath& path,
                                PhysxShapeDesc* shapeDesc,
                                const ObjectInstance* objectInstance,
                                ObjectId* shapeId = nullptr);

// Fills descs without USD annotation. `attachedStage` resolves the mesh
// prim's SdfPath into the ObjectKey stored on `desc.meshPrimKey`; pass
// `nullptr` for non-stage callers (the descriptor's `meshPrimKey` is then
// left as the invalid ObjectKey sentinel).
bool fillConvexMeshDesc(AttachedStage* attachedStage,
                        const PXR_NS::UsdGeomMesh& mesh,
                        omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                        const omni::physx::ConvexMeshCookingParams& cookingParams);
// Key-based entry for source-only consumers (e.g. scene queries): resolves
// `meshKey` to the mesh prim and delegates to the UsdGeomMesh form. The prim is
// still materialized here because the cooking-input geometry read
// (fillCookingRequest) reads mesh points/topology from USD — that read is the
// tracked cooking-input de-USD workstream and stays encapsulated in this layer.
bool fillConvexMeshDesc(AttachedStage* attachedStage,
                        omni::physics::parse::ObjectKey meshKey,
                        omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                        const omni::physx::ConvexMeshCookingParams& cookingParams);
bool fillTriangleMeshDesc(AttachedStage* attachedStage,
                          const PXR_NS::UsdGeomMesh& mesh,
                          omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                          const omni::physx::TriangleMeshCookingParams& cookingParams);
bool fillTriangleMeshDesc(AttachedStage* attachedStage,
                          omni::physics::parse::ObjectKey meshKey,
                          omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                          const omni::physx::TriangleMeshCookingParams& cookingParams);
bool fillSdfTriangleMeshDesc(AttachedStage* attachedStage,
                             const PXR_NS::UsdGeomMesh& mesh,
                             omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                             const omni::physx::SdfMeshCookingParams& cookingParams);
bool fillSdfTriangleMeshDesc(AttachedStage* attachedStage,
                             omni::physics::parse::ObjectKey meshKey,
                             omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                             const omni::physx::SdfMeshCookingParams& cookingParams);
bool fillConvexDecompositionDesc(AttachedStage* attachedStage,
                                 const PXR_NS::UsdGeomMesh& mesh,
                                 omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
                                 const omni::physx::ConvexDecompositionCookingParams& cookingParams);
bool fillConvexDecompositionDesc(AttachedStage* attachedStage,
                                 omni::physics::parse::ObjectKey meshKey,
                                 omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
                                 const omni::physx::ConvexDecompositionCookingParams& cookingParams);
bool fillSphereFillDesc(AttachedStage* attachedStage,
                        const PXR_NS::UsdGeomMesh& mesh,
                        omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
                        const omni::physx::SphereFillCookingParams& cookingParams);
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
bool fillCookingMeshViewFromSource(omni::physx::PhysxCookingComputeRequest& request,
                                   SourceMeshGeometryScope& scope,
                                   const AttachedStage& attachedStage,
                                   omni::physics::parse::ObjectKey meshKey);

PhysxShapeDesc* scaleShapeDesc(const PhysxShapeDesc& inDesc, const PXR_NS::GfVec3f& scale);

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
void invalidateMeshKeyCache(const PXR_NS::SdfPath& path);
} // namespace usdparser
} // namespace physx
} // namespace omni
