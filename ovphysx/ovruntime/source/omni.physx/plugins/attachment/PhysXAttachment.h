// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <internal/Internal.h>
#include "foundation/PxTransform.h"

namespace physx
{
class PxGeometry;
}

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}

typedef void (*getGeometryInfoCallback)(const ::physx::PxGeometry& geom,
                                        const ::physx::PxTransform& geomPos,
                                        void* userData);

bool setupAutoDeformableAttachment(const PXR_NS::SdfPath& attachmentPath);

bool updateAutoDeformableAttachment(const PXR_NS::SdfPath& attachmentPath, bool& attachmentDataRecomputed);

void processRigidShapeGeometry(usdparser::AttachedStage& attachedStage,
                               const PXR_NS::SdfPath& rigidColliderPath,
                               const usdparser::PhysxShapeDesc* desc,
                               getGeometryInfoCallback callbackFn,
                               void* userData);

} // namespace physx
} // namespace omni
