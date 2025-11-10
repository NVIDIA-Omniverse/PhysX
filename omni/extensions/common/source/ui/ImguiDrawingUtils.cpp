// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <imgui.h>
#include <carb/Types.h>
#include "ImguiDrawingUtils.h"

using namespace carb;
using namespace pxr;

static const uint32_t gLimitFreeColor = 0xBB44FF44;
static float gLineThickness = 2.0f;

#define USE_FILLED_TRIANGLES 0

namespace omni
{
namespace physics
{
namespace ui
{
// helper to get the target sdf paths
pxr::SdfPath GetBodyPath(pxr::UsdRelationship const ref)
{
    pxr::SdfPathVector targets;
    ref.GetTargets(&targets);

    if (targets.size() == 0)
    {
        return SdfPath();
    }
    if (targets.size() > 1)
    {
        return SdfPath();
    }

    return targets.at(0);
}

pxr::GfMatrix4d GetGizmoMatrix(pxr::UsdStageWeakPtr stage,
    const pxr::GfVec3f& localPos,
    const pxr::GfQuatf& localRot,
    const pxr::SdfPath& bodyPath)
{
    pxr::GfVec3f gizmoPos = localPos;
    pxr::GfQuatf gizmoRot = localRot;

    if (bodyPath != SdfPath())
    {
        pxr::UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPath);
        if (bodyPrim)
        {
            pxr::UsdGeomXformCache xfCache;
            const pxr::GfMatrix4d mat = xfCache.GetLocalToWorldTransform(bodyPrim);
            const pxr::GfTransform bodyTr(mat);
            const pxr::GfVec3f scale = GfVec3f(bodyTr.GetScale());

            // local pos includes already scale from the mat
            pxr::GfTransform gizmoTr;
            gizmoTr.SetTranslation(pxr::GfVec3f(localPos[0] / scale[0], localPos[1] / scale[1], localPos[2] / scale[2]));
            gizmoTr.SetRotation(pxr::GfRotation(localRot));

            gizmoTr = gizmoTr * mat;

            gizmoPos = pxr::GfVec3f(gizmoTr.GetTranslation());
            gizmoRot = pxr::GfQuatf(gizmoTr.GetRotation().GetQuat());
        }
    }

    pxr::GfMatrix4d gizmoTransf;
    gizmoTransf.SetRotate(gizmoRot);
    gizmoTransf.SetTranslateOnly(gizmoPos);

    return gizmoTransf;
}

pxr::GfMatrix4d GetJointPosition(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& jointPath)
{
    pxr::GfVec3f gizmoPos(0.f);
    pxr::GfQuatf gizmoRot(1.f);

    if (stage->GetPrimAtPath(jointPath).IsA<pxr::UsdPhysicsJoint>())
    {
        pxr::UsdPhysicsJoint joint = pxr::UsdPhysicsJoint::Get(stage, jointPath);

        pxr::GfVec3f localPos(0.f);
        pxr::GfQuatf localRot(1.f);
        pxr::SdfPath bodyPath;

        bodyPath = GetBodyPath(joint.GetBody0Rel());
        if (bodyPath == SdfPath())
        {
            bodyPath = GetBodyPath(joint.GetBody1Rel());
            joint.GetLocalPos1Attr().Get(&localPos);
            joint.GetLocalRot1Attr().Get(&localRot);
        }
        else
        {
            joint.GetLocalPos0Attr().Get(&localPos);
            joint.GetLocalRot0Attr().Get(&localRot);
        }

        if (bodyPath != SdfPath())
        {
            pxr::UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPath);
            if (bodyPrim)
            {
                pxr::UsdGeomXformCache xfCache;
                const pxr::GfMatrix4d mat = xfCache.GetLocalToWorldTransform(bodyPrim);
                const pxr::GfTransform bodyTr(mat);
                const pxr::GfVec3f scale = GfVec3f(bodyTr.GetScale());

                // local pos does NOT include scale from mat in this example
                pxr::GfTransform gizmoTr;
                gizmoTr.SetTranslation(localPos);
                gizmoTr.SetRotation(pxr::GfRotation(localRot));

                gizmoTr = gizmoTr * mat;

                gizmoPos = pxr::GfVec3f(gizmoTr.GetTranslation());
                gizmoRot = pxr::GfQuatf(gizmoTr.GetRotation().GetQuat());
            }
        }
    }

    pxr::GfMatrix4d gizmoTransf;
    gizmoTransf.SetRotate(gizmoRot);
    gizmoTransf.SetTranslateOnly(gizmoPos);

    return gizmoTransf;
}

void drawLimitArc(const float minAngle,
    const float maxAngle,
    DrawAxis axis,
    const pxr::GfMatrix4d& mvp,
    const carb::Float4& viewPortRect,
    float screenFactor,
    uint32_t color)
{
    const uint32_t tasselationBase = 24;
    uint32_t tesselation = tasselationBase;
    const uint32_t pointsCapacity = tasselationBase * 2 + 2;
    const float radius = 2.0f * screenFactor;
    Float2 points[pointsCapacity];

    // Limit re-drawing the circle multiple times if the difference between angles 
    // gets bigger than a full 360 circle, as with fixed tesselation step
    // it will not even resamble a 3D circle anymore.
    const float twoPi = static_cast<float>(M_PI * 2.0f);
    const float reminder = fmod(maxAngle-minAngle, twoPi);
    float endingAngle = minAngle + reminder;
    if (maxAngle - minAngle >= twoPi)
    {
        endingAngle += twoPi;
        tesselation *= 2;
    }
    uint32_t lastPointIndex = tesselation + 2;

    const float tessStep = (endingAngle - minAngle) / tesselation;

    points[0] = worldToPos(GfVec4d(0.0f, 0.0f, 0.0f, 1.0f), mvp, viewPortRect);

    for (uint32_t i = 0; i < tesselation + 1; ++i)
    {
        if (axis == eDrawAxisX)
        {
            const float v0 = cos(minAngle + i * tessStep);
            const float v1 = sin(minAngle + i * tessStep);
            points[i + 1] = worldToPos(GfVec4d(0.0f, radius * v0, radius * v1, 1.0), mvp, viewPortRect);
        }
        else if (axis == eDrawAxisY)
        {
            const float v0 = cos(minAngle + i * tessStep);
            const float v1 = sin(minAngle + i * tessStep);
            points[i + 1] = worldToPos(GfVec4d(radius * v0, 0.0f, radius * v1, 1.0), mvp, viewPortRect);
        }
        else if (axis == eDrawAxisZ)
        {
            const float v0 = cos(minAngle + (float)M_PI / 2.0f + i * tessStep);
            const float v1 = sin(minAngle + (float)M_PI / 2.0f + i * tessStep);
            points[i + 1] = worldToPos(GfVec4d(radius * v0, radius * v1, 0.0f, 1.0f), mvp, viewPortRect);
        }

        if (i != 0)
        {
            const Float2& v0 = points[i];
            const Float2& v1 = points[i + 1];

#if USE_FILLED_TRIANGLES
            const Float2& v2 = points[0];
            ImGui::GetWindowDrawList()->AddTriangleFilled(ImVec2(v0.x, v0.y), ImVec2(v1.x, v1.y), ImVec2(v2.x, v2.y), color);
#else            
            ImGui::GetWindowDrawList()->AddLine(ImVec2(v0.x, v0.y), ImVec2(v1.x, v1.y), color, gLineThickness);
#endif
        }
#if !USE_FILLED_TRIANGLES
        else if (color != gLimitFreeColor)
        {
            const Float2& v0 = points[1];
            const Float2& v2 = points[0];
            ImGui::GetWindowDrawList()->AddLine(ImVec2(v2.x, v2.y), ImVec2(v0.x, v0.y), color, gLineThickness);
        }
#endif
    }
#if !USE_FILLED_TRIANGLES
    if (color != gLimitFreeColor)
    {
        const Float2& v0 = points[lastPointIndex - 1];
        const Float2& v2 = points[0];
        ImGui::GetWindowDrawList()->AddLine(ImVec2(v2.x, v2.y), ImVec2(v0.x, v0.y), color, gLineThickness);
    }
#endif

}

static pxr::GfVec4d clipAlongZ(const pxr::GfVec4d& p0, const pxr::GfVec4d& p1, float depth)
{
    const auto clipRatio = (depth - p0[2]) / (p1[2] - p0[2]);
    return p0 + (p1 - p0) * clipRatio;
}

void addClippedLine(const GfVec3d& start,
                    const GfVec3d& end,
                    const pxr::GfMatrix4d& viewProjection,
                    const carb::Float4& viewPortRect,
                    uint32_t color,
                    float thickness,
                    const bool clipPositiveZClip, 
                    bool* clippedStart,
                    bool* clippedEnd)
{
    const GfVec4d startWorldPos(start[0], start[1], start[2], 1.0);
    const GfVec4d endWorldPos(end[0], end[1], end[2], 1.0);
    GfVec4d startWorldClipSpace = startWorldPos * viewProjection;
    GfVec4d endWorldClipSpace = endWorldPos * viewProjection;
    const float negativeZClipLimit = -1.0f;
    const float positiveZClipLimit = +1.0f;
    *clippedStart = false;
    *clippedEnd = false;
    if (startWorldClipSpace[2] <= negativeZClipLimit)
    {
        // start point is behind camera, we need to clip it to negative clip plane
        startWorldClipSpace = clipAlongZ(startWorldClipSpace, endWorldClipSpace, negativeZClipLimit);
        *clippedStart = true;
    }
    else if (clipPositiveZClip && startWorldClipSpace[2] >= positiveZClipLimit)
    {
        // VP1 clipping
        startWorldClipSpace = clipAlongZ(startWorldClipSpace, endWorldClipSpace, positiveZClipLimit);
        *clippedStart = true;
    }
    if (endWorldClipSpace[2] <= negativeZClipLimit)
    {
        // end point is behind camera, we need to clip it to negative clip plane
        endWorldClipSpace = clipAlongZ(endWorldClipSpace, startWorldClipSpace, negativeZClipLimit);
        *clippedEnd = true;
    }
    else if (clipPositiveZClip && endWorldClipSpace[2] >= positiveZClipLimit)
    {
        // VP1 clipping
        endWorldClipSpace = clipAlongZ(endWorldClipSpace, startWorldClipSpace, positiveZClipLimit);
        *clippedEnd = true;
    }
    const Float2 startScreenPos = clipSpaceToScreen(startWorldClipSpace, viewPortRect);
    const Float2 endScreenPos = clipSpaceToScreen(endWorldClipSpace, viewPortRect);
    if (!(*clippedStart) || !(*clippedEnd))
    {
        ImGui::GetWindowDrawList()->AddLine(ImVec2(startScreenPos.x, startScreenPos.y), ImVec2(endScreenPos.x, endScreenPos.y), color, thickness);
    }
}

void addLine(const GfVec3d& start,
    const GfVec3d& end,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float thickness)
{
    const GfVec4d startWorldPos(start[0], start[1], start[2], 1.0);
    const Float2 startScreenPos = worldToPos(startWorldPos, viewProjection, viewPortRect);
    const GfVec4d endWorldPos(end[0], end[1], end[2], 1.0);
    const Float2 endScreenPos = worldToPos(endWorldPos, viewProjection, viewPortRect);
    ImGui::GetWindowDrawList()->AddLine(ImVec2(startScreenPos.x, startScreenPos.y), ImVec2(endScreenPos.x, endScreenPos.y), color, thickness);
}

void addFilledCircle(const GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    uint32_t tesselation,
    float radius)
{
    const GfVec4d centerWorldPos(center[0], center[1], center[2], 1.0);
    const Float2 centerScreenPos = worldToPos(centerWorldPos, viewProjection, viewPortRect);
    ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(centerScreenPos.x, centerScreenPos.y), radius, color, tesselation);
}

void addCircle(const GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    uint32_t tesselation,
    float radius,
    float thickness)
{
    const GfVec4d centerWorldPos(center[0], center[1], center[2], 1.0);
    const Float2 centerScreenPos = worldToPos(centerWorldPos, viewProjection, viewPortRect);
    ImGui::GetWindowDrawList()->AddCircle(ImVec2(centerScreenPos.x, centerScreenPos.y), radius, color, tesselation, thickness);
}

void addRect(const GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float halfExtent,
    float thickness)
{
    const GfVec4d centerWorldPos(center[0], center[1], center[2], 1.0);
    const Float2 centerScreenPos = worldToPos(centerWorldPos, viewProjection, viewPortRect);
    const ImVec2 upLeftScreenPos(centerScreenPos.x - halfExtent, centerScreenPos.y - halfExtent);
    const ImVec2 downRightScreenPos(centerScreenPos.x + halfExtent, centerScreenPos.y + halfExtent);
    const ImDrawCornerFlags drawCornerFlags = 0;
    ImGui::GetWindowDrawList()->AddRect(upLeftScreenPos, downRightScreenPos, color, 0.f, drawCornerFlags, thickness);
}

void addFilledRect(const GfVec3d& center,
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    uint32_t color,
    float halfExtent)
{
    const GfVec4d centerWorldPos(center[0], center[1], center[2], 1.0);
    const Float2 centerScreenPos = worldToPos(centerWorldPos, viewProjection, viewPortRect);
    const ImVec2 upLeftScreenPos(centerScreenPos.x - halfExtent, centerScreenPos.y - halfExtent);
    const ImVec2 downRightScreenPos(centerScreenPos.x + halfExtent, centerScreenPos.y + halfExtent);
    const ImDrawCornerFlags drawCornerFlags = 0;
    ImGui::GetWindowDrawList()->AddRectFilled(upLeftScreenPos, downRightScreenPos, color, 0.f, drawCornerFlags);
}

void drawDiscs(const std::vector<GfVec3d>& centers,
    const GfVec3d normal,
    const GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    float radius,
    uint32_t color,
    uint32_t tesselation)
{
    // create a transform to get default space to the given normal space
    const GfRotation rot(GfVec3d(1.0f, 0.0f, 0.0f), normal);

    const size_t numDiscs = centers.size();
    std::vector<Float2> points(numDiscs * (tesselation + 2));
    for (size_t nd = 0; nd < numDiscs; nd++)
    {
        const GfVec4d center(centers[nd][0], centers[nd][1], centers[nd][2], 1.0);
        points[nd * (tesselation + 2)] = worldToPos(center, viewProjection, viewPortRect);
    }

    const float tessStep = (2.0f * float(M_PI)) / tesselation;
    for (size_t nd = 0; nd < numDiscs; nd++)
    {
        const size_t offset = nd * (tesselation + 2);
        const GfMatrix4d localToWorld(rot, centers[nd]);

        for (uint32_t i = 0; i < tesselation + 1; ++i)
        {
            const float v0 = sin(i * tessStep);
            const float v1 = cos(i * tessStep);

            const GfVec4d localPos(0.0f, radius * v0, radius * v1, 1.0f);
            const GfVec4d worldPos = localPos * localToWorld;
            points[i + 1 + offset] = worldToPos(worldPos, viewProjection, viewPortRect);

            if (i != 0)
            {
                // const size_t offset = nd * (tesselation + 2);
                const Float2& v0 = points[i + offset];
                const Float2& v1 = points[i + 1 + offset];

#if USE_FILLED_TRIANGLES
                const Float2& v2 = points[offset];
                ImGui::GetWindowDrawList()->AddTriangleFilled(ImVec2(v0.x, v0.y), ImVec2(v1.x, v1.y), ImVec2(v2.x, v2.y), color);
#else
                ImGui::GetWindowDrawList()->AddLine(ImVec2(v0.x, v0.y), ImVec2(v1.x, v1.y), color, gLineThickness);
#endif
            }
        }
    }
}


void drawSquare(GfVec3d center,
                const GfVec3d normal,
                const GfMatrix4d& viewProjection,
                const carb::Float4& viewPortRect,
                float halfExtent,
                uint32_t color,
                bool clipPositiveZ)
{
    // create a transform to get default space to the given normal space
    const GfRotation rot(GfVec3d(1.0f, 0.0f, 0.0f), normal);

    const GfMatrix4d localToWorld(rot, center);

    GfVec3d lines[4];

    const GfVec4d localPos0(0.0f, halfExtent, halfExtent, 1.0f);
    const GfVec4d worldPos0 = localPos0 * localToWorld;
    lines[0] = GfVec3d(worldPos0[0], worldPos0[1], worldPos0[2]);

    const GfVec4d localPos1(0.0f, -halfExtent, halfExtent, 1.0f);
    const GfVec4d worldPos1 = localPos1 * localToWorld;
    lines[1] = GfVec3d(worldPos1[0], worldPos1[1], worldPos1[2]);

    const GfVec4d localPos2(0.0f, -halfExtent, -halfExtent, 1.0f);
    const GfVec4d worldPos2 = localPos2 * localToWorld;
    lines[2] = GfVec3d(worldPos2[0], worldPos2[1], worldPos2[2]);

    const GfVec4d localPos3(0.0f, halfExtent, -halfExtent, 1.0f);
    const GfVec4d worldPos3 = localPos3 * localToWorld;
    lines[3] = GfVec3d(worldPos3[0], worldPos3[1], worldPos3[2]);

    bool clipStart, clipEnd;
    addClippedLine(lines[0], lines[1], viewProjection, viewPortRect, color, gLineThickness, clipPositiveZ, &clipStart, &clipEnd);
    addClippedLine(lines[1], lines[2], viewProjection, viewPortRect, color, gLineThickness, clipPositiveZ, &clipStart, &clipEnd);
    addClippedLine(lines[2], lines[3], viewProjection, viewPortRect, color, gLineThickness, clipPositiveZ, &clipStart, &clipEnd);
    addClippedLine(lines[3], lines[0], viewProjection, viewPortRect, color, gLineThickness, clipPositiveZ, &clipStart, &clipEnd);
}

void drawLinearLimit(const float lower,
                     const float upper,
                     DrawAxis axis,
                     const pxr::GfMatrix4d& mvp,
                     const carb::Float4& viewPortRect,
                     float screenFactor,
                     uint32_t color,
                     bool clipPositiveZ)
{
    GfVec3d centers[2];
    GfVec3d capVec(0.0f);
    bool clipStart, clipEnd;
    if (lower > upper)
    {
        const float lineDist = 10.0f * screenFactor;

        capVec[axis] = -lineDist;
        centers[0] = capVec;
        capVec[axis] = lineDist;
        centers[1] = capVec;
        addClippedLine(centers[0], centers[1], mvp, viewPortRect, color, gLineThickness, clipPositiveZ, &clipStart, &clipEnd);
    }
    else
    {
        const float radius = 0.6f * screenFactor;

        capVec[axis] = lower;
        centers[0] = capVec;
        capVec[axis] = upper;
        centers[1] = capVec;

        GfVec3d normal(0.0);
        normal[axis] = 1.0;

        addClippedLine(centers[0], centers[1], mvp, viewPortRect, color, gLineThickness, clipPositiveZ, &clipStart, &clipEnd);
        if (!clipStart)
        {
            drawSquare(centers[0], normal, mvp, viewPortRect, radius, color, clipPositiveZ);
        }
        if (!clipEnd)
        {
            drawSquare(centers[1], normal, mvp, viewPortRect, radius, color, clipPositiveZ);
        }
    }
}

void addAxis(
    const pxr::GfMatrix4d& viewProjection,
    const carb::Float4& viewPortRect,
    const pxr::GfMatrix4d& transform,
    float screenFactor
)
{
    const uint32_t axisColors[] = {
        0xFF6060AA, // X - Red
        0xFF76A371, // Y - Green
        0xFFA07D4F  // Z - Blue
    };

    const pxr::GfVec3d lineStart = transform.ExtractTranslation();
    for (uint32_t axisIdx = 0; axisIdx < 3; axisIdx++)
    {
        const pxr::GfVec3d lineEnd = lineStart + screenFactor * transform.GetRow3(axisIdx);
        addLine(lineStart, lineEnd, viewProjection, viewPortRect, axisColors[axisIdx], gLineThickness);
    }
}
} // namespace ui
} // namespace physics
} // namespace omni
