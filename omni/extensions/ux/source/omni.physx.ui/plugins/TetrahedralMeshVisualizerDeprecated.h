// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physx
{
namespace ui
{
class TetrahedralMeshVisualizerDeprecated
{
public:
    TetrahedralMeshVisualizerDeprecated(const pxr::SdfPath tetPath);
    ~TetrahedralMeshVisualizerDeprecated();

    void setTetPoints(const pxr::VtArray<pxr::GfVec3f>& originalPoints);
    void setTetIndices(const pxr::VtArray<int>& indices);
    void setGapValue(const float gap);
    void setTetColors(const pxr::VtArray<pxr::GfVec3f>& colors);
    void updateTetTopology();
    void updateTetPoints();

private:
    void computeRenderMeshPointsWithoutGap(const pxr::VtVec3fArray& originalPoints, pxr::VtVec3fArray& newPoints);
    void computeRenderMeshPointsGap(const pxr::VtVec3fArray& originalPoints, const float gap, pxr::VtVec3fArray& newPoints);
    void computeRenderMeshTopologyWithoutGap(pxr::VtIntArray& faceVertexCounts, pxr::VtIntArray& faceVertexIndices);
    void computeRenderMeshTopologyGap(pxr::VtIntArray& faceVertexCounts, pxr::VtIntArray& faceVertexIndices);
    void computeRenderMeshColorsWithoutGap(pxr::VtArray<pxr::GfVec3f>& newColors);
    void computeRenderMeshColorsWithGap(pxr::VtArray<pxr::GfVec3f>& newColors);
    void calculateMeshTopology();
    void updateTetPointsInternal();

    pxr::SdfPath mTetPath;
    pxr::VtVec3fArray mOriginalPoints;
    pxr::VtIntArray mOriginalIndices;
    pxr::VtVec3fArray mOriginalColors;
    pxr::VtVec3fArray mColorTable; // Lookup table from original index to original color
    std::vector<pxr::GfVec4f> mTargetShrinkPointsBarycentric;
    std::vector<uint32_t> mSurfaceTriangles;
    float mGap;
};

} // namespace ui
} // namespace physx
} // namespace omni
