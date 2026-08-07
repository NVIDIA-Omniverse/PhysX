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
void finalizeRigidBody(AttachedStage& attachedStage, BodyDescAndColliders& bodyAndColliders);

PhysxRigidBodyDesc* createStaticBody();

ObjectId getRigidBody(AttachedStage& attachedStage, const PXR_NS::SdfPath& shapeKey, PhysxShapeDesc& desc);

// Reset a deformable-body desc to its default state.  Same defaults the
// schema parser's resolved output was overlaid on top of by the now-
// retired `parseDeformableBody(schema::DeformableBodyDesc&)` helper.
// Exposed for converter use (the parse-library boundary translator in
// DeformableBodyConverter.cpp).
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxDeformableBodyDesc& desc);
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxVolumeDeformableBodyDesc& desc);
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxSurfaceDeformableBodyDesc& desc);

void finalizeDeformableBody(AttachedStage& attachedStage,
                            PhysxDeformableBodyDesc* desc,
                            const PXR_NS::SdfPath simMeshMaterial);

// Per-prim PhysxForceAPI parser.  Returns an ICE-allocated descriptor
// populated from `prim`'s PhysxForceAPI attributes; the caller takes
// ownership and must pass it to `finalizePhysxForce` + `createObject`
// before the AttachedStage is torn down.
void setToDefault(PhysxForceDesc& desc);
PhysxForceDesc* parsePhysxForce(AttachedStage& attachedStage,
                                omni::physics::parse::ObjectKey key);

void finalizePhysxForce(AttachedStage& attachedStage,
                        omni::physics::parse::ObjectKey forceKey,
                        PhysxForceDesc& desc);

PXR_NS::SdfPath getRigidBodySimulationOwner(AttachedStage& attachedStage, const PXR_NS::SdfPath& bodyPath);

} // namespace usdparser
} // namespace physx
} // namespace omni
