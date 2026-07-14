// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>


namespace omni
{
namespace physx
{
namespace usdparser
{
PhysxRigidBodyDesc* parseRigidBody(AttachedStage& attachedStage,
                                   PXR_NS::UsdGeomXformCache& xformCache,
                                   const omni::physics::schema::RigidBodyDesc& inDesc,
                                   CollisionPairVector& collisionBlocks,
                                   bool ignoreOwners = false);

void finalizeRigidBody(AttachedStage& attachedStage, BodyDescAndColliders& bodyAndColliders);

PhysxRigidBodyDesc* createStaticBody();

ObjectId getRigidBody(AttachedStage& attachedStage, const PXR_NS::SdfPath& shapePath, PhysxShapeDesc& desc);

PhysxDeformableBodyDesc* parseDeformableBody(AttachedStage& attachedStage,
                                             PXR_NS::UsdGeomXformCache& xformCache,
                                             const PXR_NS::SdfPath& path,
                                             const omni::physics::schema::DeformableBodyDesc& inDesc,
                                             CollisionPairVector& filteredPairs,
                                             PXR_NS::SdfPath& simMeshMaterial);

void finalizeDeformableBody(AttachedStage& attachedStage,
                            PhysxDeformableBodyDesc* desc,
                            const PXR_NS::SdfPath simMeshMaterial);

PhysxForceDesc* parsePhysxForce(AttachedStage& attachedStage, const PXR_NS::UsdPrim& prim, PXR_NS::UsdGeomXformCache& xformCache);
void finalizePhysxForce(AttachedStage& attachedStage,
                        const PXR_NS::UsdPrim& prim,
                        PhysxForceDesc& desc,
                        PXR_NS::UsdGeomXformCache& xformCache);

PXR_NS::SdfPath getRigidBodySimulationOwner(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& bodyPath);

} // namespace usdparser
} // namespace physx
} // namespace omni
