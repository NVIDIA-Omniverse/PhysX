// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    DeformableTriMeshVisualizer(const PXR_NS::SdfPath triPath);
    ~DeformableTriMeshVisualizer();

    virtual void setPoints(const PXR_NS::VtArray<PXR_NS::GfVec3f>& originalPoints) override;
    virtual void setIndices(const PXR_NS::VtArray<int>& indices) override;
    virtual void setGapValue(const float gap) override;
    virtual void setColors(const PXR_NS::VtArray<PXR_NS::GfVec3f>& colors) override;
    virtual void updateTopology() override;
    virtual void updatePoints() override;

    std::unordered_set<uint32_t> getFilteredTriIds(const PXR_NS::VtArray<PXR_NS::GfVec3i>& filteredFaceVertexIndices);

private:
    void computeRenderMeshPointsWithoutGap(const PXR_NS::VtVec3fArray& originalPoints, PXR_NS::VtVec3fArray& newPoints);
    void computeRenderMeshPointsGap(const PXR_NS::VtVec3fArray& originalPoints, const float gap, PXR_NS::VtVec3fArray& newPoints);
    void computeRenderMeshTopologyWithoutGap(PXR_NS::VtIntArray& faceVertexCounts, PXR_NS::VtIntArray& faceVertexIndices);
    void computeRenderMeshTopologyGap(PXR_NS::VtIntArray& faceVertexCounts, PXR_NS::VtIntArray& faceVertexIndices);
    void calculateMeshTopology();
    void updateTriPointsInternal();

    PXR_NS::SdfPath mTriPath;
    PXR_NS::VtVec3fArray mOriginalPoints;
    PXR_NS::VtIntArray mOriginalIndices;
    PXR_NS::VtVec3fArray mOriginalColors;
    PXR_NS::VtVec3fArray mColorTable; // Lookup table from original index to original color
    std::vector<PXR_NS::GfVec3f> mTargetShrinkPointsBarycentric;
    std::unordered_set<SortedTriangle, TriangleHash, SortedTriangleEqualFunction> mTrianglesSet;
    float mGap;
};

} // namespace ui
} // namespace physx
} // namespace omni
