// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>

namespace omni::physics
{

// Shared descriptor math; callers provide matrices from their own source.
inline void decomposeCollisionShapeLocalTransform(const PXR_NS::GfMatrix4d& shapeToBody,
                                                  const PXR_NS::GfMatrix4d& bodyToWorld,
                                                  PXR_NS::GfVec3f& outLocalPos,
                                                  PXR_NS::GfQuatf& outLocalRot,
                                                  PXR_NS::GfVec3f& outLocalScale)
{
    const PXR_NS::GfTransform shapeLocal(shapeToBody);
    outLocalPos = PXR_NS::GfVec3f(shapeLocal.GetTranslation());
    outLocalRot = PXR_NS::GfQuatf(shapeLocal.GetRotation().GetQuat());
    outLocalScale = PXR_NS::GfVec3f(shapeLocal.GetScale());

    // PhysX actors carry no body scale, so bake it into the child offset.
    const PXR_NS::GfVec3d bodyScale = PXR_NS::GfTransform(bodyToWorld).GetScale();
    for (int i = 0; i < 3; ++i)
        outLocalPos[i] *= static_cast<float>(bodyScale[i]);
}

} // namespace omni::physics
