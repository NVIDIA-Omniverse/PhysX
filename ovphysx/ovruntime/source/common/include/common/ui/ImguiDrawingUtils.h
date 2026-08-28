// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#include "carb/Types.h"

// Once joint authoring C++ drawing code will be ported to omni.ui.scene, this class can be moved back to omni.physxui
// and eventually removed once imgui drawing code will be removed also from omni.physxui.

namespace omni
{
namespace physics
{
namespace ui
{
enum DrawAxis
{
    eDrawAxisX,
    eDrawAxisY,
    eDrawAxisZ
};

template < typename VectorType >
inline carb::Float2 clipSpaceToScreen(VectorType trans, const carb::Float4& viewPortRect)
{
    trans *= 0.5f / trans[3];
    trans += VectorType(double(0.5), double(0.5), double(0.0), double(0.0));
    trans[1] = 1.f - trans[1];
    trans[0] *= viewPortRect.z - viewPortRect.x;
    trans[1] *= viewPortRect.w - viewPortRect.y;
    trans[0] += viewPortRect.x;
    trans[1] += viewPortRect.y;
    return carb::Float2({ float(trans[0]), float(trans[1]) });
}

inline carb::Float2 worldToPos(const PXR_NS::GfVec4d& worldPos, const PXR_NS::GfMatrix4d& mat, const carb::Float4& viewPortRect)
{
    PXR_NS::GfVec4d trans;
    trans = worldPos * mat;
    return clipSpaceToScreen(trans, viewPortRect);
}

inline carb::Float2 worldToPos(const PXR_NS::GfVec4f& worldPos, const PXR_NS::GfMatrix4f& mat, const carb::Float4& viewPortRect)
{
    PXR_NS::GfVec4f trans;
    trans = worldPos * mat;
    return clipSpaceToScreen(trans, viewPortRect);
}

// currently input and output are argb
inline uint32_t convertColor(uint32_t inColor)
{
    uint32_t outColor = inColor;

    // temp
    if ((outColor & 0xFF000000) == 0)
    {
        outColor |= 0xFF000000;
    }
    return outColor;
}

inline void convertColor(uint32_t inColor, carb::ColorRgba& outColor)
{
    outColor.a = ((inColor & 0xFF000000) >> 24) / 255.0f;
    outColor.r = ((inColor & 0xFF0000) >> 16) / 255.0f;
    outColor.g = ((inColor & 0xFF00) >> 8) / 255.0f;
    outColor.b = (inColor & 0xFF) / 255.0f;

    // temp
    if (outColor.a == .0f)
    {
        outColor.a = 1.0f;
    }
}

// helper to get the target sdf paths
PXR_NS::SdfPath GetBodyPath(PXR_NS::UsdRelationship const ref);

PXR_NS::GfMatrix4d GetGizmoMatrix(PXR_NS::UsdStageWeakPtr stage,
    const PXR_NS::GfVec3f& localPos,
    const PXR_NS::GfQuatf& localRot,
    const PXR_NS::SdfPath& bodyPath);

PXR_NS::GfMatrix4d GetJointPosition(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& jointPath);

void drawLimitArc(const float minAngle,
    const float maxAngle,
    DrawAxis axis,
    const PXR_NS::GfMatrix4d& mvp,
    const carb::Float4& viewPortRect,
    float screenFactor,
    uint32_t color);

void addLine(const PXR_NS::GfVec3d& start,
    const PXR_NS::GfVec3d& end,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float thickness);

void addFilledCircle(const PXR_NS::GfVec3d& center,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    uint32_t tesselation,
    float radius);

void addCircle(const PXR_NS::GfVec3d& center,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    uint32_t tesselation,
    float radius,
    float thickness);

void addRect(const PXR_NS::GfVec3d& center,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float halfExtent,
    float thickness);

void addFilledRect(const PXR_NS::GfVec3d& center,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float halfExtent);

void drawDiscs(const std::vector<PXR_NS::GfVec3d>& centers,
    const PXR_NS::GfVec3d normal,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    float radius,
    uint32_t color,
    uint32_t tesselation);

void drawSquare(PXR_NS::GfVec3d center,
    const PXR_NS::GfVec3d normal,
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    float halfExtent,
    uint32_t color,
    bool clipPositiveZ);

void drawLinearLimit(const float lower,
    const float upper,
    DrawAxis axis,
    const PXR_NS::GfMatrix4d& mvp,
    const carb::Float4& viewPortRect,
    float screenFactor,
    uint32_t color,
    bool clipPositiveZ);

void addAxis(
    const PXR_NS::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    const PXR_NS::GfMatrix4d& transform,
    float screenFactor
);
} // namespace ui
} // namespace physics
} // namespace omni
