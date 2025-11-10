// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "DeformableMeshVisualizer.h"

namespace omni
{
namespace physx
{
namespace ui
{

class DeformableTriMeshVisualizer : public DeformableMeshVisualizer
{
public:
    DeformableTriMeshVisualizer(const pxr::SdfPath triPath);
    ~DeformableTriMeshVisualizer();

    virtual void setPoints(const pxr::VtArray<pxr::GfVec3f>& originalPoints) override;
    virtual void setIndices(const pxr::VtArray<int>& indices) override;
    virtual void setGapValue(const float gap) override;
    virtual void setColors(const pxr::VtArray<pxr::GfVec3f>& colors) override;
    virtual void updateTopology() override;
    virtual void updatePoints() override;

private:
    void computeRenderMeshPointsWithoutGap(const pxr::VtVec3fArray& originalPoints, pxr::VtVec3fArray& newPoints);
    void computeRenderMeshPointsGap(const pxr::VtVec3fArray& originalPoints, const float gap, pxr::VtVec3fArray& newPoints);
    void computeRenderMeshTopologyWithoutGap(pxr::VtIntArray& faceVertexCounts, pxr::VtIntArray& faceVertexIndices);
    void computeRenderMeshTopologyGap(pxr::VtIntArray& faceVertexCounts, pxr::VtIntArray& faceVertexIndices);
    void calculateMeshTopology();
    void updateTriPointsInternal();

    pxr::SdfPath mTriPath;
    pxr::VtVec3fArray mOriginalPoints;
    pxr::VtIntArray mOriginalIndices;
    pxr::VtVec3fArray mOriginalColors;
    pxr::VtVec3fArray mColorTable; // Lookup table from original index to original color
    std::vector<pxr::GfVec3f> mTargetShrinkPointsBarycentric;
    float mGap;
};

} // namespace ui
} // namespace physx
} // namespace omni
