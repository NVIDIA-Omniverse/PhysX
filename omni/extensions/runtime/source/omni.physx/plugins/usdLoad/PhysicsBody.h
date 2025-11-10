// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
                                   pxr::UsdGeomXformCache& xformCache,
                                   const omni::physics::schema::RigidBodyDesc& inDesc,
                                   CollisionPairVector& collisionBlocks,
                                   bool ignoreOwners = false);

void finalizeRigidBody(AttachedStage& attachedStage, BodyDescAndColliders& bodyAndColliders);

PhysxRigidBodyDesc* createStaticBody();

ObjectId getRigidBody(AttachedStage& attachedStage, const pxr::SdfPath& shapePath, PhysxShapeDesc& desc);

PhysxDeformableBodyDesc* parseDeformableBody(AttachedStage& attachedStage,
                                             pxr::UsdGeomXformCache& xformCache,
                                             const pxr::SdfPath& path,
                                             const omni::physics::schema::DeformableBodyDesc& inDesc,
                                             CollisionPairVector& filteredPairs,
                                             pxr::SdfPath& simMeshMaterial);

void finalizeDeformableBody(AttachedStage& attachedStage,
                            PhysxDeformableBodyDesc* desc,
                            const pxr::SdfPath simMeshMaterial);

PhysxForceDesc* parsePhysxForce(AttachedStage& attachedStage, const pxr::UsdPrim& prim, pxr::UsdGeomXformCache& xformCache);
void finalizePhysxForce(AttachedStage& attachedStage,
                        const pxr::UsdPrim& prim,
                        PhysxForceDesc& desc,
                        pxr::UsdGeomXformCache& xformCache);

pxr::SdfPath getRigidBodySimulationOwner(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& bodyPath);

} // namespace usdparser
} // namespace physx
} // namespace omni
