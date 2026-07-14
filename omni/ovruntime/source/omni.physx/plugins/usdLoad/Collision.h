// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

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

// prepare PhysxShapeDesc from given generic ShapeDesc
PhysxShapeDesc* parseCollisionDesc(AttachedStage& attachedStage,
                                   PXR_NS::UsdGeomXformCache& xfCache,
                                   PathPhysXDescMap& physxDescCache,
                                   const PXR_NS::SdfPath& path,
                                   const omni::physics::schema::ShapeDesc& shapeDesc,
                                   CollisionPairVector& filteredPairs,
                                   PXR_NS::SdfPathVector& materials,
                                   bool ignoreOwners = false);

// check if given prim is a collision shape
bool isCollisionShape(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& prim);

void finalizeShape(AttachedStage& attachedStage, PhysxShapeDesc* desc, const PXR_NS::SdfPathVector& materials);
PhysxRigidBodyDesc* createShape(AttachedStage& attachedStage,
                                const PXR_NS::SdfPath& path,
                                PXR_NS::UsdGeomXformCache& xfCache,
                                PhysxShapeDesc* shapeDesc,
                                const ObjectInstance* objectInstance,
                                ObjectId* shapeId = nullptr);

// function to fill descs without USD anotation
bool fillConvexMeshDesc(const PXR_NS::UsdGeomMesh& mesh,
                        omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                        const omni::physx::ConvexMeshCookingParams& cookingParams);
bool fillTriangleMeshDesc(const PXR_NS::UsdGeomMesh& mesh,
                          omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                          const omni::physx::TriangleMeshCookingParams& cookingParams);
bool fillSdfTriangleMeshDesc(const PXR_NS::UsdGeomMesh& mesh,
                             omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                             const omni::physx::SdfMeshCookingParams& cookingParams);
bool fillConvexDecompositionDesc(const PXR_NS::UsdGeomMesh& mesh,
                                 omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
                                 const omni::physx::ConvexDecompositionCookingParams& cookingParams);
bool fillSphereFillDesc(const PXR_NS::UsdGeomMesh& mesh,
                        omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
                        const omni::physx::SphereFillCookingParams& cookingParams);

PhysxShapeDesc* scaleShapeDesc(const PhysxShapeDesc& inDesc, const PXR_NS::GfVec3f& scale);

// release collision shape desc - specialized function, has to release more memory
// A.B. TODO - this is most likely leaking when used from debug draw
// we should try to move the release into a destructor probably
void releaseShapeDesc(PhysxShapeDesc* desc);

void notifyStageReset(void);
void invalidateMeshKeyCache(const PXR_NS::SdfPath& path);
} // namespace usdparser
} // namespace physx
} // namespace omni
