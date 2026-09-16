// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-23
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-1 AC-2
 */

#pragma once

#include <internal/Internal.h>
#include "foundation/PxTransform.h"

namespace physx
{
class PxGeometry;
}

namespace omni::physics::parse
{
struct PhysxDeformableAttachmentDesc;
struct PhysxDeformableCollisionFilterDesc;
} // namespace omni::physics::parse

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
struct GeneratedAutoAttachmentChild;
}

typedef void (*getGeometryInfoCallback)(const ::physx::PxGeometry& geom,
                                        const ::physx::PxTransform& geomPos,
                                        void* userData);

bool setupAutoDeformableAttachment(omni::physics::parse::ObjectKey attachmentKey);

bool updateAutoDeformableAttachment(omni::physics::parse::ObjectKey attachmentKey, bool& attachmentDataRecomputed);

// In-memory auto attachment, for an attach that cannot author sub prims (no live USD stage):
// records the sub-prim set setupAutoDeformableAttachment would author as a
// GeneratedAutoAttachmentLayout on the AttachedStage. False when the prim is not an auto
// attachment or its attachables form no supported combination.
bool buildGeneratedAutoDeformableAttachmentLayout(usdparser::AttachedStage& attachedStage,
                                                  omni::physics::parse::ObjectKey autoAttachmentKey);

// How many sub prims setupAutoDeformableAttachment would produce for the prim (0 when it is not
// a usable auto attachment); lets the load spot an authored child set that is incomplete.
uint32_t expectedAutoDeformableAttachmentSubPrimCount(usdparser::AttachedStage& attachedStage,
                                                      omni::physics::parse::ObjectKey autoAttachmentKey);

// The consumer desc a generated child stands for (ICE-allocated, caller frees).
omni::physics::parse::PhysxDeformableAttachmentDesc* makeGeneratedDeformableAttachmentDesc(
    const usdparser::GeneratedAutoAttachmentChild& child);
omni::physics::parse::PhysxDeformableCollisionFilterDesc* makeGeneratedDeformableCollisionFilterDesc(
    const usdparser::GeneratedAutoAttachmentChild& child);

void processRigidShapeGeometry(usdparser::AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey rigidColliderKey,
                               const usdparser::PhysxShapeDesc* desc,
                               getGeometryInfoCallback callbackFn,
                               void* userData);

} // namespace physx
} // namespace omni
