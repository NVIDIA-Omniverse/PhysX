// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>
#include <map>

#include "DeformableTetMeshVisualizer.h"

using namespace omni::physx::ui;
using namespace pxr;

extern UsdStageRefPtr gStage;

namespace
{

struct SortedTriangle
{
    uint32_t A;
    uint32_t B;
    uint32_t C;
    int32_t TetIndex;
    bool Flipped;

    inline SortedTriangle(uint32_t a = -1, uint32_t b = -1, uint32_t c = -1, int32_t tetIndex = -1)
    {
        A = a;
        B = b;
        C = c;
        TetIndex = tetIndex;
        Flipped = false;
        if (A > B)
        {
            std::swap(A, B);
            Flipped = !Flipped;
        }
        if (B > C)
        {
            std::swap(B, C);
            Flipped = !Flipped;
        }
        if (A > B)
        {
            std::swap(A, B);
            Flipped = !Flipped;
        }
    }
};

class SortedTriangleEqualFunction
{
public:
    inline bool operator()(const SortedTriangle& first, const SortedTriangle& second) const
    {
        return first.A == second.A && first.B == second.B && first.C == second.C;
    }
};

struct TriangleHash
{
    inline std::size_t operator()(const SortedTriangle& k) const
    {
        return k.A ^ k.B ^ k.C;
    }
};

struct SurfaceTriangleInfo
{
    SurfaceTriangleInfo() : innerPointIndex(-1), numSurfaceTriangles(0)
    {
    }

    SortedTriangle tris[4];
    uint32_t innerPointIndex;
    uint32_t numSurfaceTriangles;
};

static float gGapDefault = 0.2f;
static const uint32_t gTetFaces[4][3] = { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } };

inline static GfVec3f computeShrinkedPoint(const GfVec3f& originalPoint, const GfVec3f& center, const float gap)
{
    GfVec3f dir = center - originalPoint;
    return originalPoint + dir * gap;
}

void extractSurfaceTriangles(const uint32_t* tetrahedra,
                             const uint32_t numTetrahedra,
                             std::vector<uint32_t>& surfaceTriangles,
                             bool flipTriangleOrientation = false)
{
    std::unordered_map<SortedTriangle, uint32_t, TriangleHash, SortedTriangleEqualFunction> tris;

    uint32_t l = 4 * numTetrahedra;
    for (uint32_t i = 0; i < l; i += 4)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            SortedTriangle tri(
                tetrahedra[i + gTetFaces[j][0]], tetrahedra[i + gTetFaces[j][1]], tetrahedra[i + gTetFaces[j][2]], i);
            auto iter = tris.find(tri);
            if (iter != tris.end())
            {
                ++iter->second;
            }
            else
            {
                tris.insert({ tri, 1 });
            }
        }
    }

    surfaceTriangles.clear();
    for (std::unordered_map<SortedTriangle, uint32_t, TriangleHash>::iterator iter = tris.begin(); iter != tris.end();
         ++iter)
    {
        if (iter->second == 1)
        {
            surfaceTriangles.push_back(iter->first.A);
            if (iter->first.Flipped != flipTriangleOrientation)
            {
                surfaceTriangles.push_back(iter->first.C);
                surfaceTriangles.push_back(iter->first.B);
            }
            else
            {
                surfaceTriangles.push_back(iter->first.B);
                surfaceTriangles.push_back(iter->first.C);
            }
        }
    }
}

bool isSurfaceTriangle(const std::unordered_set<SortedTriangle, TriangleHash, SortedTriangleEqualFunction>& surfaceTrianglesSet,
                       const SortedTriangle& surfaceTriangle)
{
    return surfaceTrianglesSet.find(surfaceTriangle) != surfaceTrianglesSet.end();
}

bool isInSortedTriangle(const SortedTriangle& tri, const uint32_t index)
{
    if (index == tri.A || index == tri.B || index == tri.C)
        return true;
    else
        return false;
}

uint32_t getFourthVertIdx(const SortedTriangle& tri, const uint32_t index0, const uint32_t index1, const uint32_t index2)
{
    if (!isInSortedTriangle(tri, index0))
        return index0;
    else if (!isInSortedTriangle(tri, index1))
        return index1;
    else if (!isInSortedTriangle(tri, index2))
        return index2;
    else
        return -1;
}

void setSurfaceTriangleInfo(
    const std::unordered_set<SortedTriangle, TriangleHash, SortedTriangleEqualFunction>& surfaceTrianglesSet,
    const uint32_t innerPointIndex,
    const uint32_t idx0,
    const uint32_t idx1,
    const uint32_t idx2,
    SurfaceTriangleInfo& surfaceTriangleInfo)
{
    const SortedTriangle surfaceTriangle(idx0, idx1, idx2);
    if (isSurfaceTriangle(surfaceTrianglesSet, surfaceTriangle))
    {
        if (surfaceTriangleInfo.numSurfaceTriangles == 0)
        {
            surfaceTriangleInfo.innerPointIndex = innerPointIndex;
        }
        surfaceTriangleInfo.tris[surfaceTriangleInfo.numSurfaceTriangles++] = surfaceTriangle;
    }
}

void calculateSurfaceTrianglesInfo(
    const std::unordered_set<SortedTriangle, TriangleHash, SortedTriangleEqualFunction>& surfaceTrianglesSet,
    const uint32_t* indices,
    SurfaceTriangleInfo& surfaceTriangleInfo)
{
    // We only consider a tetrahedron with at least 1 surface triangle as a surface tet
    // Four triangles of tet (0, 1, 2, 3) are (0, 1, 2) & (0, 1, 3) & (0, 2, 3) & (1, 2, 3)
    setSurfaceTriangleInfo(surfaceTrianglesSet, indices[3], indices[0], indices[1], indices[2], surfaceTriangleInfo);
    setSurfaceTriangleInfo(surfaceTrianglesSet, indices[2], indices[0], indices[1], indices[3], surfaceTriangleInfo);
    setSurfaceTriangleInfo(surfaceTrianglesSet, indices[1], indices[0], indices[2], indices[3], surfaceTriangleInfo);
    setSurfaceTriangleInfo(surfaceTrianglesSet, indices[0], indices[1], indices[2], indices[3], surfaceTriangleInfo);
}

void fillInNewPoints(const GfVec3f* outputPoints, VtVec3fArray& newPoints)
{
    // Old index -> New index
    //         0 -> 0, 3, 6
    //         1 -> 2, 4, 9
    //         2 -> 1, 8, 10
    //         3 -> 5, 7, 11
    newPoints.push_back(outputPoints[0]);
    newPoints.push_back(outputPoints[2]);
    newPoints.push_back(outputPoints[1]);
    newPoints.push_back(outputPoints[0]);
    newPoints.push_back(outputPoints[1]);
    newPoints.push_back(outputPoints[3]);
    newPoints.push_back(outputPoints[0]);
    newPoints.push_back(outputPoints[3]);
    newPoints.push_back(outputPoints[2]);
    newPoints.push_back(outputPoints[1]);
    newPoints.push_back(outputPoints[2]);
    newPoints.push_back(outputPoints[3]);
}

void findSharedEdgeIndices(const SortedTriangle tris[3], uint32_t& idx0, uint32_t& idx1, uint32_t& idx2, uint32_t& idx3)
{
    // (idx0, idx1, idx2) & (idx1, idx2, idx3)
    idx0 = getFourthVertIdx(tris[1], tris[0].A, tris[0].B, tris[0].C);
    idx3 = getFourthVertIdx(tris[0], tris[1].A, tris[1].B, tris[1].C);
    if (tris[0].A == idx0)
    {
        idx1 = tris[0].B;
        idx2 = tris[0].C;
    }
    else if (tris[0].B == idx0)
    {
        idx1 = tris[0].A;
        idx2 = tris[0].C;
    }
    else if (tris[0].C == idx0)
    {
        idx1 = tris[0].A;
        idx2 = tris[0].B;
    }
}

uint32_t findCornerIndex(const SortedTriangle tris[3])
{
    // Corner index is shared by three surface triangles so check for the pressence of each vertex in both the other
    // triangles.
    if (isInSortedTriangle(tris[1], tris[0].A) && isInSortedTriangle(tris[2], tris[0].A))
    {
        return tris[0].A;
    }
    else if (isInSortedTriangle(tris[1], tris[0].B) && isInSortedTriangle(tris[2], tris[0].B))
    {
        return tris[0].B;
    }
    else
    {
        // Must be C or we have done something wrong.
        CARB_ASSERT(isInSortedTriangle(tris[1], tris[0].C) && isInSortedTriangle(tris[2], tris[0].C));
        return tris[0].C;
    }
}

inline GfVec4f computeBarycentric(const GfVec3f& a, const GfVec3f& b, const GfVec3f& c, const GfVec3f& d, const GfVec3f& p)
{
    GfVec4f bary;

    const GfVec3f ba = b - a;
    const GfVec3f ca = c - a;
    const GfVec3f da = d - a;
    const GfVec3f pa = p - a;

    const float detBcd = GfDot(ba, GfCross(ca, da));
    const float detPcd = GfDot(pa, GfCross(ca, da));

    bary[1] = detPcd / detBcd;

    const float detBpd = GfDot(ba, GfCross(pa, da));
    bary[2] = detBpd / detBcd;

    const float detBcp = GfDot(ba, GfCross(ca, pa));

    bary[3] = detBcp / detBcd;
    bary[0] = 1 - bary[1] - bary[2] - bary[3];

    return bary;
}

GfVec4f precomputeTargetShrinkPointInnerTet(const VtVec3fArray& originalPoints, const uint32_t* indices)
{
    const GfVec3f point0 = originalPoints[indices[0]];
    const GfVec3f point1 = originalPoints[indices[1]];
    const GfVec3f point2 = originalPoints[indices[2]];
    const GfVec3f point3 = originalPoints[indices[3]];

    const GfVec3f tetCenter = 0.25f * (point0 + point1 + point2 + point3);

    GfVec4f barycentric = computeBarycentric(originalPoints[indices[0]], originalPoints[indices[1]],
                                             originalPoints[indices[2]], originalPoints[indices[3]], tetCenter);

    return barycentric;
}

GfVec4f precomputeTargetShrinkPointTetWithOneSurfaceTriangle(const VtVec3fArray& originalPoints,
                                                             const SurfaceTriangleInfo& surfaceTriangleInfo,
                                                             const uint32_t* indices)
{
    const GfVec3f surfacePoint0 = originalPoints[surfaceTriangleInfo.tris[0].A];
    const GfVec3f surfacePoint1 = originalPoints[surfaceTriangleInfo.tris[0].B];
    const GfVec3f surfacePoint2 = originalPoints[surfaceTriangleInfo.tris[0].C];

    const GfVec3f surfaceCenter = (surfacePoint0 + surfacePoint1 + surfacePoint2) / 3.0f;

    GfVec4f barycentric = computeBarycentric(originalPoints[indices[0]], originalPoints[indices[1]],
                                             originalPoints[indices[2]], originalPoints[indices[3]], surfaceCenter);

    return barycentric;
}

GfVec4f precomputeTargetShrinkPointTetWithTwoSurfaceTriangles(const VtVec3fArray& originalPoints,
                                                              const SurfaceTriangleInfo& surfaceTriangleInfo,
                                                              const uint32_t* indices)
{
    uint32_t idx0 = 0;
    uint32_t idx1 = 0;
    uint32_t idx2 = 0;
    uint32_t idx3 = 0;

    // idx1 & idx2 are the shared indices
    findSharedEdgeIndices(surfaceTriangleInfo.tris, idx0, idx1, idx2, idx3);

    const GfVec3f point1 = originalPoints[idx1];
    const GfVec3f point2 = originalPoints[idx2];

    const GfVec3f edgeCenter = (point1 + point2) / 2.0f;

    GfVec4f barycentric = computeBarycentric(originalPoints[indices[0]], originalPoints[indices[1]],
                                             originalPoints[indices[2]], originalPoints[indices[3]], edgeCenter);

    return barycentric;
}

GfVec4f precomputeTargetShrinkPointTetWithThreeSurfaceTriangles(const VtVec3fArray& originalPoints,
                                                                const SurfaceTriangleInfo& surfaceTriangleInfo,
                                                                const uint32_t* indices)
{
    uint32_t sharedIdx = findCornerIndex(surfaceTriangleInfo.tris);

    const GfVec3f sharedPoint = originalPoints[sharedIdx];

    GfVec4f barycentric = computeBarycentric(originalPoints[indices[0]], originalPoints[indices[1]],
                                             originalPoints[indices[2]], originalPoints[indices[3]], sharedPoint);

    return barycentric;
}

inline GfVec3f reconstructPointFromBarycentric(const GfVec3f& p0,
                                               const GfVec3f& p1,
                                               const GfVec3f& p2,
                                               const GfVec3f& p3,
                                               const GfVec4f& targetShrinkPointBarycentric)
{
    return p0 * targetShrinkPointBarycentric[0] + p1 * targetShrinkPointBarycentric[1] +
           p2 * targetShrinkPointBarycentric[2] + p3 * targetShrinkPointBarycentric[3];
}

} // namespace

DeformableTetMeshVisualizer::DeformableTetMeshVisualizer(const SdfPath tetPath) :
    mTetPath(tetPath), mGap(gGapDefault)
{
}

DeformableTetMeshVisualizer::~DeformableTetMeshVisualizer()
{
}

void DeformableTetMeshVisualizer::setPoints(const VtArray<GfVec3f>& originalPoints)
{
    mOriginalPoints = originalPoints;
}

void DeformableTetMeshVisualizer::setIndices(const VtArray<int>& indices)
{
    mOriginalIndices = indices;
}

void DeformableTetMeshVisualizer::setGapValue(const float gap)
{
    mGap = CARB_CLAMP(gap, 0.0f, 1.0f);
    gGapDefault = mGap;

    updateTetPointsInternal();
}

void DeformableTetMeshVisualizer::setColors(const VtArray<GfVec3f>& colors)
{
    mOriginalColors = colors;

#if 0
    // TODO: Revisit gap == 0.0 case to solve the rendering issue
    if (mOriginalIndices.empty())
        return;

    mColorTable.clear();
    size_t numOrigIndices = mOriginalIndices.size();
    mColorTable.resize(numOrigIndices);
    for (size_t i = 0; i < numOrigIndices; ++i)
    {
        mColorTable[mOriginalIndices[i]] = mOriginalColors[i];
    }
#endif

    VtArray<GfVec3f> newColors;
#if 0
    // TODO: Revisit gap == 0.0 case to solve the rendering issue
    if (mGap == 0.0f)
    {
        computeRenderMeshColorsWithoutGap(newColors);
    }
    else
#endif
    {
        computeRenderMeshColorsWithGap(newColors);
    }

    UsdGeomMesh tetMesh = UsdGeomMesh::Get(gStage, mTetPath);
    if (tetMesh)
    {
        tetMesh.CreateDisplayColorPrimvar(UsdGeomTokens.Get()->faceVarying, (int)newColors.size());
        tetMesh.GetDisplayColorPrimvar().Set(newColors);
    }
}

void DeformableTetMeshVisualizer::updateTopology()
{
    VtIntArray faceVertexCounts;
    VtIntArray faceVertexIndices;

    calculateMeshTopology();
    updateTetPointsInternal();

#if 0
    // TODO: Revisit gap == 0.0 case to solve the rendering issue
    if (mGap == 0.0f)
    {
        computeRenderMeshTopologyWithoutGap(faceVertexCounts, faceVertexIndices);
    }
    else
#endif
    {
        computeRenderMeshTopologyGap(faceVertexCounts, faceVertexIndices);
    }

    UsdGeomMesh tetMesh = UsdGeomMesh::Get(gStage, mTetPath);
    if (tetMesh)
    {
        UsdAttribute vertexCountAttr = tetMesh.GetFaceVertexCountsAttr();
        if (vertexCountAttr)
        {
            vertexCountAttr.Set(faceVertexCounts);
        }

        UsdAttribute indicesAttr = tetMesh.GetFaceVertexIndicesAttr();
        if (indicesAttr)
        {
            indicesAttr.Set(faceVertexIndices);
        }
    }
}

void DeformableTetMeshVisualizer::updatePoints()
{
    if (!mTargetShrinkPointsBarycentric.empty())
        updateTetPointsInternal();
}

void DeformableTetMeshVisualizer::computeRenderMeshPointsWithoutGap(const VtVec3fArray& originalPoints,
    VtVec3fArray& newPoints)
{
    const size_t numSurfaceTriangles = mSurfaceTriangles.size() / 3;
    newPoints.reserve(numSurfaceTriangles * 3);

    // We generate three new points for each surface triangle
    // This can avoid surface triangle normal issue
    for (size_t i = 0; i < numSurfaceTriangles; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            newPoints.push_back(originalPoints[mSurfaceTriangles[3 * i + j]]);
        }
    }
}

void DeformableTetMeshVisualizer::computeRenderMeshPointsGap(const VtVec3fArray& originalPoints,
    const float gap,
    VtVec3fArray& newPoints)
{
    const size_t numIndices = mOriginalIndices.size();
    CARB_ASSERT(numIndices % 4 == 0);
    const size_t numTetrahedrons = numIndices / 4;
    newPoints.reserve(numTetrahedrons * 4 * 3);

    // Transform target points to euclidean space and shrink tet points accordingly
    for (size_t tetrahedronIndex = 0; tetrahedronIndex < numTetrahedrons; ++tetrahedronIndex)
    {
        uint32_t indices[4];
        bool isIndicesValid = true;
        for (size_t i = 0; i < 4; ++i)
        {
            indices[i] = mOriginalIndices[tetrahedronIndex * 4 + i];
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

        const GfVec4f targetShrinkPointBarycentric = mTargetShrinkPointsBarycentric[tetrahedronIndex];
        const GfVec3f targetPoint = reconstructPointFromBarycentric(originalPoints[indices[0]], originalPoints[indices[1]], originalPoints[indices[2]], originalPoints[indices[3]], targetShrinkPointBarycentric);

        GfVec3f outputPoints[4];
        for (size_t tetPointIndex = 0; tetPointIndex < 4; ++tetPointIndex)
        {
            outputPoints[tetPointIndex] = computeShrinkedPoint(originalPoints[mOriginalIndices[4 * tetrahedronIndex + tetPointIndex]], targetPoint, gap);
        }
        fillInNewPoints(outputPoints, newPoints);

    }
}

void DeformableTetMeshVisualizer::computeRenderMeshTopologyWithoutGap(VtIntArray& faceVertexCounts,
    VtIntArray& faceVertexIndices)
{
    const size_t numSurfaceTriangles = mSurfaceTriangles.size() / 3;
    faceVertexCounts.reserve(numSurfaceTriangles);
    faceVertexIndices.reserve(numSurfaceTriangles * 3);

    for (size_t i = 0; i < numSurfaceTriangles; ++i)
    {
        faceVertexCounts.push_back(3);

        for (size_t j = 0; j < 3; ++j)
        {
            faceVertexIndices.push_back((int)(3 * i + j));
        }
    }
}

void DeformableTetMeshVisualizer::computeRenderMeshTopologyGap(VtIntArray& faceVertexCounts,
    VtIntArray& faceVertexIndices)
{
    const size_t numIndices = mOriginalIndices.size();
    CARB_ASSERT(numIndices % 4 == 0);

    const size_t numTetrahedrons = numIndices / 4;
    faceVertexCounts.reserve(numTetrahedrons * 4); // Each tetrahedron has 4 triangle surfaces
    for (size_t i = 0; i < numTetrahedrons * 4; ++i)
    {
        faceVertexCounts.push_back(3);
    }

    faceVertexIndices.reserve(numTetrahedrons * 4 * 3);

    for (size_t tetrahedronIndex = 0; tetrahedronIndex < numTetrahedrons; ++tetrahedronIndex)
    {
        // We generate four seperate triangles * three new points each = 12 from tetrahedron (0, 1, 2, 3)
        // (0, 2, 1) (0, 1, 3) (0, 3, 2) (1, 2, 3)
        // This can avoid surface triangle normal issue of one tet mesh with four shared points
        for (size_t i = 0; i < 12; ++i)
        {
            faceVertexIndices.push_back((int)(tetrahedronIndex * 12 + i));
        }
    }
}

void DeformableTetMeshVisualizer::computeRenderMeshColorsWithoutGap(VtArray<GfVec3f>& newColors)
{
    const size_t numSurfaceTriangles = mSurfaceTriangles.size() / 3;
    size_t numNewColors = numSurfaceTriangles * 3;
    newColors.resize(numNewColors);

    for (size_t i = 0; i < numSurfaceTriangles; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            newColors.push_back(mColorTable[mSurfaceTriangles[3 * i + j]]);
        }
    }
}

void DeformableTetMeshVisualizer::computeRenderMeshColorsWithGap(VtArray<GfVec3f>& newColors)
{
    const size_t numOriginalColors = mOriginalColors.size();
    size_t numNewColors = numOriginalColors * 3; // Each tet will have 12 new points & indices, hence numOriginalColors/4 * 12
    newColors.resize(numNewColors);

    CARB_ASSERT(numOriginalColors % 4 == 0);
    const size_t numTetrahedrons = numOriginalColors / 4;
    for (size_t tetrahedronIndex = 0; tetrahedronIndex < numTetrahedrons; ++tetrahedronIndex)
    {
        // Old index -> New index
        //         0 -> 0, 3, 6
        //         1 -> 2, 4, 9
        //         2 -> 1, 8, 10
        //         3 -> 5, 7, 11
        newColors[tetrahedronIndex * 12 + 0]  = mOriginalColors[tetrahedronIndex * 4 + 0];
        newColors[tetrahedronIndex * 12 + 3]  = mOriginalColors[tetrahedronIndex * 4 + 0];
        newColors[tetrahedronIndex * 12 + 6]  = mOriginalColors[tetrahedronIndex * 4 + 0];
        newColors[tetrahedronIndex * 12 + 2]  = mOriginalColors[tetrahedronIndex * 4 + 1];
        newColors[tetrahedronIndex * 12 + 4]  = mOriginalColors[tetrahedronIndex * 4 + 1];
        newColors[tetrahedronIndex * 12 + 9]  = mOriginalColors[tetrahedronIndex * 4 + 1];
        newColors[tetrahedronIndex * 12 + 1]  = mOriginalColors[tetrahedronIndex * 4 + 2];
        newColors[tetrahedronIndex * 12 + 8]  = mOriginalColors[tetrahedronIndex * 4 + 2];
        newColors[tetrahedronIndex * 12 + 10] = mOriginalColors[tetrahedronIndex * 4 + 2];
        newColors[tetrahedronIndex * 12 + 5]  = mOriginalColors[tetrahedronIndex * 4 + 3];
        newColors[tetrahedronIndex * 12 + 7]  = mOriginalColors[tetrahedronIndex * 4 + 3];
        newColors[tetrahedronIndex * 12 + 11] = mOriginalColors[tetrahedronIndex * 4 + 3];
    }
}

void DeformableTetMeshVisualizer::calculateMeshTopology()
{
    std::unordered_set<SortedTriangle, TriangleHash, SortedTriangleEqualFunction> surfaceTrianglesSet;

    std::vector<uint32_t>& surfaceTriangles = mSurfaceTriangles;
    extractSurfaceTriangles((uint32_t*)mOriginalIndices.data(), (uint32_t)(mOriginalIndices.size() / 4), surfaceTriangles);

    const size_t numSurfaceTriangles = surfaceTriangles.size() / 3;
    for (size_t i = 0; i < numSurfaceTriangles; ++i)
    {
        SortedTriangle surfaceTriangle(surfaceTriangles[3 * i], surfaceTriangles[3 * i + 1], surfaceTriangles[3 * i + 2]);
        surfaceTrianglesSet.insert(surfaceTriangle);
    }

    const size_t numIndices = mOriginalIndices.size();
    CARB_ASSERT(numIndices % 4 == 0);
    const size_t numTetrahedrons = numIndices / 4;

    mTargetShrinkPointsBarycentric.clear();
    mTargetShrinkPointsBarycentric.reserve(numTetrahedrons);

    for (size_t tetrahedronIndex = 0; tetrahedronIndex < numTetrahedrons; ++tetrahedronIndex)
    {
        uint32_t indices[4];
        bool isIndicesValid = true;
        for (size_t i = 0; i < 4; ++i)
        {
            indices[i] = mOriginalIndices[tetrahedronIndex * 4 + i];
            if (indices[i] >= mOriginalPoints.size())
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

        SurfaceTriangleInfo surfaceTriangleInfo;
        calculateSurfaceTrianglesInfo(surfaceTrianglesSet, indices, surfaceTriangleInfo);

        GfVec4f barycentric;
        if (surfaceTriangleInfo.numSurfaceTriangles == 0)
            barycentric = precomputeTargetShrinkPointInnerTet(mOriginalPoints, indices);
        else if (surfaceTriangleInfo.numSurfaceTriangles == 1)
            barycentric = precomputeTargetShrinkPointTetWithOneSurfaceTriangle(mOriginalPoints, surfaceTriangleInfo, indices);
        else if (surfaceTriangleInfo.numSurfaceTriangles == 2)
            barycentric = precomputeTargetShrinkPointTetWithTwoSurfaceTriangles(mOriginalPoints, surfaceTriangleInfo, indices);
        else if (surfaceTriangleInfo.numSurfaceTriangles == 3)
            barycentric = precomputeTargetShrinkPointTetWithThreeSurfaceTriangles(mOriginalPoints, surfaceTriangleInfo, indices);

        mTargetShrinkPointsBarycentric.push_back(barycentric);
    }
}

void DeformableTetMeshVisualizer::updateTetPointsInternal()
{
    VtVec3fArray newPoints;
#if 0
    // TODO: Revisit gap == 0.0 case to solve the rendering issue
    if (mGap == 0.0f)
    {
        computeRenderMeshPointsWithoutGap(mOriginalPoints, newPoints);
    }
    else
#endif
    {
        computeRenderMeshPointsGap(mOriginalPoints, mGap, newPoints);
    }

    UsdGeomMesh tetMesh = UsdGeomMesh::Get(gStage, mTetPath);
    if (tetMesh)
    {
        UsdAttribute pointsAttr = tetMesh.GetPointsAttr();
        if (pointsAttr)
        {
            pointsAttr.Set(newPoints);
        }
    }
}
