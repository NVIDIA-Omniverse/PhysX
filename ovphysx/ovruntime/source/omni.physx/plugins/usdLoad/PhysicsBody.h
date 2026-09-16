// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */
#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni::physics::parse
{
class IPhysicsSource;
} // namespace omni::physics::parse

namespace omni
{
namespace physx
{
namespace usdparser
{
void finalizeRigidBody(AttachedStage& attachedStage, BodyDescAndColliders& bodyAndColliders);

PhysxRigidBodyDesc* createStaticBody();

ObjectId getRigidBody(AttachedStage& attachedStage, omni::physics::parse::ObjectKey shapeKey, PhysxShapeDesc& desc);

// Reset a deformable-body desc to its default state.  Same defaults the
// schema parser's resolved output was overlaid on top of by the now-
// retired `parseDeformableBody(schema::DeformableBodyDesc&)` helper.
// Exposed for converter use (the parse-library boundary translator in
// DeformableBodyConverter.cpp).
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxDeformableBodyDesc& desc);
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxVolumeDeformableBodyDesc& desc);
// The source interns the "flatDefault" default for restBendAnglesDefault
// (a TokenId, ADR-0019 increment 7 -- TokenId values are only meaningful
// against the source that minted them).
void setToDefault(const omni::physics::parse::SourceUnits& units,
                  const omni::physics::parse::IPhysicsSource& source,
                  PhysxSurfaceDeformableBodyDesc& desc);

void finalizeDeformableBody(AttachedStage& attachedStage,
                            PhysxDeformableBodyDesc* desc,
                            omni::physics::parse::ObjectKey simMeshMaterial);

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

omni::physics::parse::ObjectKey getRigidBodySimulationOwner(AttachedStage& attachedStage, omni::physics::parse::ObjectKey bodyKey);

} // namespace usdparser
} // namespace physx
} // namespace omni
