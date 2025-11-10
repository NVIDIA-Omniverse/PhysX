// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

inline carb::Float2 worldToPos(const pxr::GfVec4d& worldPos, const pxr::GfMatrix4d& mat, const carb::Float4& viewPortRect)
{
    pxr::GfVec4d trans;
    trans = worldPos * mat;
    return clipSpaceToScreen(trans, viewPortRect);
}

inline carb::Float2 worldToPos(const pxr::GfVec4f& worldPos, const pxr::GfMatrix4f& mat, const carb::Float4& viewPortRect)
{
    pxr::GfVec4f trans;
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
pxr::SdfPath GetBodyPath(pxr::UsdRelationship const ref);

pxr::GfMatrix4d GetGizmoMatrix(pxr::UsdStageWeakPtr stage,
    const pxr::GfVec3f& localPos,
    const pxr::GfQuatf& localRot,
    const pxr::SdfPath& bodyPath);

pxr::GfMatrix4d GetJointPosition(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& jointPath);

void drawLimitArc(const float minAngle,
    const float maxAngle,
    DrawAxis axis,
    const pxr::GfMatrix4d& mvp,
    const carb::Float4& viewPortRect,
    float screenFactor,
    uint32_t color);

void addLine(const pxr::GfVec3d& start,
    const pxr::GfVec3d& end,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float thickness);

void addFilledCircle(const pxr::GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    uint32_t tesselation,
    float radius);

void addCircle(const pxr::GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    uint32_t tesselation,
    float radius,
    float thickness);

void addRect(const pxr::GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float halfExtent,
    float thickness);

void addFilledRect(const pxr::GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float halfExtent);

void drawDiscs(const std::vector<pxr::GfVec3d>& centers,
    const pxr::GfVec3d normal,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    float radius,
    uint32_t color,
    uint32_t tesselation);

void drawSquare(pxr::GfVec3d center,
    const pxr::GfVec3d normal,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    float halfExtent,
    uint32_t color,
    bool clipPositiveZ);

void drawLinearLimit(const float lower,
    const float upper,
    DrawAxis axis,
    const pxr::GfMatrix4d& mvp,
    const carb::Float4& viewPortRect,
    float screenFactor,
    uint32_t color,
    bool clipPositiveZ);

void addAxis(
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    const pxr::GfMatrix4d& transform,
    float screenFactor
);
} // namespace ui
} // namespace physics
} // namespace omni
