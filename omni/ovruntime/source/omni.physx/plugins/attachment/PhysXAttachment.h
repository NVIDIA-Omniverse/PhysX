// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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

typedef void (*getGeometryInfoCallback)(const ::physx::PxGeometry& geom,
                                        const ::physx::PxTransform& geomPos,
                                        void* userData);

bool setupAutoDeformableAttachment(const PXR_NS::SdfPath& attachmentPath);

bool updateAutoDeformableAttachment(const PXR_NS::SdfPath& attachmentPath, bool& attachmentDataRecomputed);

void processRigidShapeGeometry(const PXR_NS::UsdPrim& rigidBodyPrim,
                               const usdparser::PhysxShapeDesc* desc,
                               getGeometryInfoCallback callbackFn,
                               void* userData);

} // namespace physx
} // namespace omni
