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
class DeformableTetMeshVisualizer : public DeformableMeshVisualizer
{
public:
    DeformableTetMeshVisualizer(const PXR_NS::SdfPath tetPath);
    ~DeformableTetMeshVisualizer();

    virtual void setPoints(const PXR_NS::VtArray<PXR_NS::GfVec3f>& originalPoints) override;
    virtual void setIndices(const PXR_NS::VtArray<int>& indices) override;
    virtual void setGapValue(const float gap) override;
    virtual void setColors(const PXR_NS::VtArray<PXR_NS::GfVec3f>& colors) override;
    virtual void updateTopology() override;
    virtual void updatePoints() override;

    std::unordered_set<uint32_t> getFilteredTetIds(const PXR_NS::VtArray<PXR_NS::GfVec3i>& filteredSurfaceFaceVertexIndices);

private:
    void computeRenderMeshPointsWithoutGap(const PXR_NS::VtVec3fArray& originalPoints, PXR_NS::VtVec3fArray& newPoints);
    void computeRenderMeshPointsGap(const PXR_NS::VtVec3fArray& originalPoints, const float gap, PXR_NS::VtVec3fArray& newPoints);
    void computeRenderMeshTopologyWithoutGap(PXR_NS::VtIntArray& faceVertexCounts, PXR_NS::VtIntArray& faceVertexIndices);
    void computeRenderMeshTopologyGap(PXR_NS::VtIntArray& faceVertexCounts, PXR_NS::VtIntArray& faceVertexIndices);
    void computeRenderMeshColorsWithoutGap(PXR_NS::VtArray<PXR_NS::GfVec3f>& newColors);
    void computeRenderMeshColorsWithGap(PXR_NS::VtArray<PXR_NS::GfVec3f>& newColors);
    void calculateMeshTopology();
    void updateTetPointsInternal();

    PXR_NS::SdfPath mTetPath;
    PXR_NS::VtVec3fArray mOriginalPoints;
    PXR_NS::VtIntArray mOriginalIndices;
    PXR_NS::VtVec3fArray mOriginalColors;
    PXR_NS::VtVec3fArray mColorTable; // Lookup table from original index to original color
    std::vector<PXR_NS::GfVec4f> mTargetShrinkPointsBarycentric;
    std::vector<uint32_t> mSurfaceTriangles;
    std::unordered_set<SortedTriangle, TriangleHash, SortedTriangleEqualFunction> mSurfaceTrianglesSet;
    float mGap;
};

} // namespace ui
} // namespace physx
} // namespace omni
