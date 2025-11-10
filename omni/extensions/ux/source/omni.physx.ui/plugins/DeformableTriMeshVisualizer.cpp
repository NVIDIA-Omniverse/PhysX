// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>
#include <map>

#include "DeformableTriMeshVisualizer.h"

using namespace omni::physx::ui;
using namespace pxr;

extern UsdStageRefPtr gStage;

static float gGapDefault = 0.2f;

inline GfVec3f reconstructPointFromBarycentric(const GfVec3f& p0, const GfVec3f& p1, const GfVec3f& p2, const GfVec3f& targetShrinkPointBarycentric)
{
    return p0 * targetShrinkPointBarycentric[0] + p1 * targetShrinkPointBarycentric[1] + p2 * targetShrinkPointBarycentric[2];
}

inline static GfVec3f computeShrinkedPoint(const GfVec3f& originalPoint, const GfVec3f& center, const float gap)
{
    GfVec3f dir = center - originalPoint;
    return originalPoint + dir * gap;
}

DeformableTriMeshVisualizer::DeformableTriMeshVisualizer(const SdfPath triPath) :
    mTriPath(triPath), mGap(gGapDefault)
{
}

DeformableTriMeshVisualizer::~DeformableTriMeshVisualizer()
{
}

void DeformableTriMeshVisualizer::setPoints(const VtArray<GfVec3f>& originalPoints)
{
    mOriginalPoints = originalPoints;
}

void DeformableTriMeshVisualizer::setIndices(const VtArray<int>& indices)
{
    mOriginalIndices = indices;
}

void DeformableTriMeshVisualizer::setGapValue(const float gap)
{
    mGap = CARB_CLAMP(gap, 0.0f, 1.0f);
    gGapDefault = mGap;

    updateTriPointsInternal();
}

void DeformableTriMeshVisualizer::setColors(const VtArray<GfVec3f>& colors)
{
    mOriginalColors = colors;

    UsdGeomMesh triMesh = UsdGeomMesh::Get(gStage, mTriPath);
    if (triMesh)
    {
        triMesh.CreateDisplayColorPrimvar(UsdGeomTokens.Get()->faceVarying, (int)mOriginalColors.size());
        triMesh.GetDisplayColorPrimvar().Set(mOriginalColors);
    }
}

void DeformableTriMeshVisualizer::updateTopology()
{
    VtIntArray faceVertexCounts;
    VtIntArray faceVertexIndices;

    calculateMeshTopology();
    updateTriPointsInternal();

    computeRenderMeshTopologyGap(faceVertexCounts, faceVertexIndices);

    UsdGeomMesh triMesh = UsdGeomMesh::Get(gStage, mTriPath);
    if (triMesh)
    {
        UsdAttribute vertexCountAttr = triMesh.GetFaceVertexCountsAttr();
        if (vertexCountAttr)
        {
            vertexCountAttr.Set(faceVertexCounts);
        }

        UsdAttribute indicesAttr = triMesh.GetFaceVertexIndicesAttr();
        if (indicesAttr)
        {
            indicesAttr.Set(faceVertexIndices);
        }
    }
}

void DeformableTriMeshVisualizer::updatePoints()
{
    if (!mTargetShrinkPointsBarycentric.empty())
        updateTriPointsInternal();
}

void DeformableTriMeshVisualizer::computeRenderMeshPointsWithoutGap(const VtVec3fArray& originalPoints, VtVec3fArray& newPoints)
{
    const size_t numTriangles = mOriginalIndices.size() / 3;
    newPoints.reserve(numTriangles * 3);

    // We generate three new points for each surface triangle
    // This can avoid surface triangle normal issue
    for (size_t i = 0; i < numTriangles; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            newPoints.push_back(originalPoints[mOriginalIndices[3 * i + j]]);
        }
    }
}

void DeformableTriMeshVisualizer::computeRenderMeshPointsGap(const VtVec3fArray& originalPoints, const float gap, VtVec3fArray& newPoints)
{
    const size_t numIndices = mOriginalIndices.size();
    CARB_ASSERT(numIndices % 3 == 0);
    const size_t numTriangles = numIndices / 3;
    newPoints.reserve(numIndices);

    // Transform target points to euclidean space and shrink tri points accordingly
    for (size_t triIndex = 0; triIndex < numTriangles; ++triIndex)
    {
        uint32_t indices[3];
        bool isIndicesValid = true;
        for (size_t i = 0; i < 3; ++i)
        {
            indices[i] = mOriginalIndices[triIndex * 3 + i];
            if (indices[i] >= originalPoints.size())
            {
                isIndicesValid = false;
                CARB_LOG_ERROR("Index %d from original inidices array is out of range.", indices[i]);
                break;
            }
        }

        if (!isIndicesValid)
        {
            continue;
        }

        const GfVec3f targetShrinkPointBarycentric = mTargetShrinkPointsBarycentric[triIndex];
        const GfVec3f targetPoint = reconstructPointFromBarycentric(originalPoints[indices[0]], originalPoints[indices[1]], originalPoints[indices[2]], targetShrinkPointBarycentric);

        GfVec3f outputPoints[3];
        for (size_t triPointIndex = 0; triPointIndex < 3; ++triPointIndex)
        {
            outputPoints[triPointIndex] = computeShrinkedPoint(originalPoints[mOriginalIndices[3 * triIndex + triPointIndex]], targetPoint, gap);
        }
        newPoints.push_back(outputPoints[0]);
        newPoints.push_back(outputPoints[1]);
        newPoints.push_back(outputPoints[2]);     
    }
}

void DeformableTriMeshVisualizer::computeRenderMeshTopologyWithoutGap(VtIntArray& faceVertexCounts, VtIntArray& faceVertexIndices)
{
    const size_t numTriangles = mOriginalIndices.size() / 3;
    faceVertexCounts.reserve(numTriangles);
    faceVertexIndices.reserve(numTriangles * 3);

    for (size_t i = 0; i < numTriangles; ++i)
    {
        faceVertexCounts.push_back(3);

        for (size_t j = 0; j < 3; ++j)
        {
            faceVertexIndices.push_back((int)(3 * i + j));
        }
    }
}

void DeformableTriMeshVisualizer::computeRenderMeshTopologyGap(VtIntArray& faceVertexCounts, VtIntArray& faceVertexIndices)
{
    const size_t numIndices = mOriginalIndices.size();
    CARB_ASSERT(numIndices % 3 == 0);

    const size_t numTriangles = numIndices / 3;
    faceVertexCounts.reserve(numTriangles * 3);
    for (size_t i = 0; i < numTriangles; ++i)
    {
        faceVertexCounts.push_back(3);
    }

    faceVertexIndices.reserve(numIndices);

    for (size_t triIndex = 0; triIndex < numTriangles; ++triIndex)
    {
        faceVertexIndices.push_back((int)(triIndex * 3 + 0));
        faceVertexIndices.push_back((int)(triIndex * 3 + 1));
        faceVertexIndices.push_back((int)(triIndex * 3 + 2));
    }
}

void DeformableTriMeshVisualizer::calculateMeshTopology()
{
    const size_t numTriangles = mOriginalIndices.size() / 3;

    mTargetShrinkPointsBarycentric.clear();
    mTargetShrinkPointsBarycentric.resize(numTriangles, GfVec3f(1.0f/3, 1.0f/3, 1.0f/3));
}

void DeformableTriMeshVisualizer::updateTriPointsInternal()
{
    VtVec3fArray newPoints;
    computeRenderMeshPointsGap(mOriginalPoints, mGap, newPoints);

    UsdGeomMesh triMesh = UsdGeomMesh::Get(gStage, mTriPath);
    if (triMesh)
    {
        UsdAttribute pointsAttr = triMesh.GetPointsAttr();
        if (pointsAttr)
        {
            pointsAttr.Set(newPoints);
        }
    }
}
