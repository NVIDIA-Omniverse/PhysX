// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>

namespace omni::physics
{

// Convert a joint frame authored relative to a relationship target into the
// resolved body's local frame. PhysX actors carry no body scale, so the body's
// world scale is baked into the resulting translation.
inline void transformJointFrameToBody(const PXR_NS::GfMatrix4d& relationshipToWorld,
                                      const PXR_NS::GfMatrix4d& bodyToWorld,
                                      bool relationshipTargetsBody,
                                      PXR_NS::GfVec3f& inOutLocalPosition,
                                      PXR_NS::GfQuatf& inOutLocalOrientation)
{
    if (!relationshipTargetsBody)
    {
        PXR_NS::GfMatrix4d localAnchor;
        localAnchor.SetIdentity();
        localAnchor.SetTranslate(PXR_NS::GfVec3d(inOutLocalPosition));
        localAnchor.SetRotateOnly(PXR_NS::GfQuatd(inOutLocalOrientation));

        const PXR_NS::GfMatrix4d worldAnchor = localAnchor * relationshipToWorld;
        const PXR_NS::GfMatrix4d bodyLocalAnchor = (worldAnchor * bodyToWorld.GetInverse()).RemoveScaleShear();

        inOutLocalPosition = PXR_NS::GfVec3f(bodyLocalAnchor.ExtractTranslation());
        inOutLocalOrientation = PXR_NS::GfQuatf(bodyLocalAnchor.ExtractRotationQuat());
        inOutLocalOrientation.Normalize();
    }

    const PXR_NS::GfVec3d bodyScale = PXR_NS::GfTransform(bodyToWorld).GetScale();
    for (int i = 0; i < 3; ++i)
        inOutLocalPosition[i] *= static_cast<float>(bodyScale[i]);
}

} // namespace omni::physics
