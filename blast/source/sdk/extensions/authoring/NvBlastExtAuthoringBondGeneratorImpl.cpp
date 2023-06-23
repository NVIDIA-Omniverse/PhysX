// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2022-2023 NVIDIA Corporation. All rights reserved.


// This warning arises when using some stl containers with older versions of VC
// c:\program files (x86)\microsoft visual studio 12.0\vc\include\xtree(1826): warning C4702: unreachable code
#include "NvPreprocessor.h"
#if NV_VC && NV_VC < 14
#pragma warning(disable : 4702)
#endif

#include <NvBlastExtAuthoringBondGeneratorImpl.h>
#include <NvBlast.h>
#include <NvBlastGlobals.h>
#include <NvBlastNvSharedHelpers.h>
#include "NvBlastExtTriangleProcessor.h"
#include "NvBlastExtApexSharedParts.h"
#include "NvBlastExtAuthoringInternalCommon.h"
#include "NvBlastExtAuthoringTypes.h"
#include <vector>
#include <map>
#include "NvPlane.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <set>

#define SAFE_ARRAY_NEW(T, x) ((x) > 0) ? reinterpret_cast<T*>(NVBLAST_ALLOC(sizeof(T) * (x))) : nullptr;

//#define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT

void saveGeometryToObj(std::vector<NvVec3>& triangles, const char* filepath)
{

    FILE* outStream = fopen(filepath, "w");

    for (uint32_t i = 0; i < triangles.size(); ++i)
    {
        fprintf(outStream, "v %lf %lf %lf\n", triangles[i].x, triangles[i].y, triangles[i].z);
        ++i;
        fprintf(outStream, "v %lf %lf %lf\n", triangles[i].x, triangles[i].y, triangles[i].z);
        ++i;
        fprintf(outStream, "v %lf %lf %lf\n", triangles[i].x, triangles[i].y, triangles[i].z);
    }
    for (uint32_t i = 0; i < triangles.size() / 3; ++i)
    {
        NvVec3 normal =
            (triangles[3 * i + 2] - triangles[3 * i]).cross((triangles[3 * i + 1] - triangles[3 * i])).getNormalized();
        fprintf(outStream, "vn %lf %lf %lf\n", normal.x, normal.y, normal.z);
        fprintf(outStream, "vn %lf %lf %lf\n", normal.x, normal.y, normal.z);
        fprintf(outStream, "vn %lf %lf %lf\n", normal.x, normal.y, normal.z);
    }
    int indx = 1;
    for (uint32_t i = 0; i < triangles.size() / 3; ++i)
    {
        fprintf(outStream, "f %d//%d  ", indx, indx);
        indx++;
        fprintf(outStream, "%d//%d  ", indx, indx);
        indx++;
        fprintf(outStream, "%d//%d \n", indx, indx);
        indx++;
    }

    fclose(outStream);
}


std::vector<NvVec3> intersectionBuffer;
std::vector<NvVec3> meshBuffer;
#endif


namespace Nv
{
namespace Blast
{

#define EPS_PLANE 0.0001f

nvidia::NvVec3 getNormal(const Triangle& t)
{
    return toNvShared(t.b.p - t.a.p).cross(toNvShared(t.c.p - t.a.p));
}

bool planeComparer(const PlaneChunkIndexer& as, const PlaneChunkIndexer& bs)
{
    const NvcPlane& a = as.plane;
    const NvcPlane& b = bs.plane;

    if (a.d + EPS_PLANE < b.d)
        return true;
    if (a.d - EPS_PLANE > b.d)
        return false;
    if (a.n.x + EPS_PLANE < b.n.x)
        return true;
    if (a.n.x - EPS_PLANE > b.n.x)
        return false;
    if (a.n.y + EPS_PLANE < b.n.y)
        return true;
    if (a.n.y - EPS_PLANE > b.n.y)
        return false;
    return a.n.z + EPS_PLANE < b.n.z;
}


struct Bond
{
    int32_t m_chunkId;
    int32_t m_planeIndex;
    int32_t triangleIndex;

    bool operator<(const Bond& inp) const
    {
        if (abs(m_planeIndex) == abs(inp.m_planeIndex))
        {
            return m_chunkId < inp.m_chunkId;
        }
        else
        {
            return abs(m_planeIndex) < abs(inp.m_planeIndex);
        }
    }
};


struct BondInfo
{
    float area;
    nvidia::NvBounds3 m_bb;
    nvidia::NvVec3 centroid;
    nvidia::NvVec3 normal;
    int32_t m_chunkId;
};

inline nvidia::NvVec3 getVertex(const Triangle& t, uint32_t i)
{
    return toNvShared((&t.a)[i].p);
}

void AddTtAnchorPoints(const Triangle* a, const Triangle* b, std::vector<NvVec3>& points)
{
    nvidia::NvVec3 na = getNormal(*a).getNormalized();
    nvidia::NvVec3 nb = getNormal(*b).getNormalized();

    nvidia::NvPlane pla(toNvShared(a->a.p), na);
    nvidia::NvPlane plb(toNvShared(b->a.p), nb);


    ProjectionDirections da = getProjectionDirection(na);
    ProjectionDirections db = getProjectionDirection(nb);

    TriangleProcessor prc;

    TrPrcTriangle2d ta(getProjectedPoint(toNvShared(a->a.p), da), getProjectedPoint(toNvShared(a->b.p), da),
                       getProjectedPoint(toNvShared(a->c.p), da));
    TrPrcTriangle2d tb(getProjectedPoint(toNvShared(b->a.p), db), getProjectedPoint(toNvShared(b->b.p), db),
                       getProjectedPoint(toNvShared(b->c.p), db));

    /**
        Compute
    */
    for (uint32_t i = 0; i < 3; ++i)
    {
        nvidia::NvVec3 pt;
        if (getPlaneSegmentIntersection(pla, getVertex(*b, i), getVertex(*b, (i + 1) % 3), pt))
        {

            nvidia::NvVec2 pt2 = getProjectedPoint(pt, da);
            if (prc.isPointInside(pt2, ta))
            {
                points.push_back(pt);
            }
        }
        if (getPlaneSegmentIntersection(plb, getVertex(*a, i), getVertex(*a, (i + 1) % 3), pt))
        {
            NvVec2 pt2 = getProjectedPoint(pt, db);
            if (prc.isPointInside(pt2, tb))
            {
                points.push_back(pt);
            }
        }
    }
}


inline bool
pointInsidePoly(const NvVec3& pt, const uint8_t* indices, uint16_t indexCount, const NvVec3* verts, const NvVec3& n)
{
    int s = 0;
    for (uint16_t i = 0; i < indexCount; ++i)
    {
        const NvVec3 r0 = verts[indices[i]] - pt;
        const NvVec3 r1 = verts[indices[(i + 1) % indexCount]] - pt;
        const float cn  = r0.cross(r1).dot(n);
        const int cns   = cn >= 0 ? 1 : -1;
        if (!s)
        {
            s = cns;
        }
        if (cns * s < 0)
        {
            return false;
        }
    }
    return true;
}

void AddPpAnchorPoints(const uint8_t* indicesA, uint16_t indexCountA, const NvVec3* vertsA, const float planeA[4],
                       const uint8_t* indicesB, uint16_t indexCountB, const NvVec3* vertsB, const float planeB[4],
                       std::vector<NvVec3>& points)
{
    NvPlane pla(planeA[0], planeA[1], planeA[2], planeA[3]);
    NvPlane plb(planeB[0], planeB[1], planeB[2], planeB[3]);

    for (uint16_t iA = 0; iA < indexCountA; ++iA)
    {
        NvVec3 pt;
        if (getPlaneSegmentIntersection(plb, vertsA[indicesA[iA]], vertsA[indicesA[(iA + 1) % indexCountA]], pt))
        {
            if (pointInsidePoly(pt, indicesB, indexCountB, vertsB, plb.n))
            {
                points.push_back(pt);
            }
        }
    }

    for (uint16_t iB = 0; iB < indexCountA; ++iB)
    {
        NvVec3 pt;
        if (getPlaneSegmentIntersection(pla, vertsB[indicesB[iB]], vertsB[indicesA[(iB + 1) % indexCountB]], pt))
        {
            if (pointInsidePoly(pt, indicesA, indexCountA, vertsA, pla.n))
            {
                points.push_back(pt);
            }
        }
    }
}


float BlastBondGeneratorImpl::processWithMidplanes(TriangleProcessor* trProcessor, const Triangle* mA, uint32_t mavc,
                                                   const Triangle* mB, uint32_t mbvc, const CollisionHull* hull1,
                                                   const CollisionHull* hull2, const std::vector<NvVec3>& hull1p,
                                                   const std::vector<NvVec3>& hull2p, NvVec3& normal, NvVec3& centroid,
                                                   float maxRelSeparation)
{
    NvBounds3 bounds;
    NvBounds3 aBounds;
    NvBounds3 bBounds;
    bounds.setEmpty();
    aBounds.setEmpty();
    bBounds.setEmpty();

    NvVec3 chunk1Centroid(0, 0, 0);
    NvVec3 chunk2Centroid(0, 0, 0);

    ///////////////////////////////////////////////////////////////////////////////////
    if (hull1p.size() < 4 || hull2p.size() < 4)
    {
        return 0.0f;
    }

    for (uint32_t i = 0; i < hull1p.size(); ++i)
    {
        chunk1Centroid += hull1p[i];
        bounds.include(hull1p[i]);
        aBounds.include(hull1p[i]);
    }
    for (uint32_t i = 0; i < hull2p.size(); ++i)
    {
        chunk2Centroid += hull2p[i];
        bounds.include(hull2p[i]);
        bBounds.include(hull2p[i]);
    }

    chunk1Centroid *= (1.0f / hull1p.size());
    chunk2Centroid *= (1.0f / hull2p.size());

    const float maxSeparation = maxRelSeparation * std::sqrt(std::max(aBounds.getExtents().magnitudeSquared(), bBounds.getExtents().magnitudeSquared()));

    Separation separation;
    if (!importerHullsInProximityApexFree(hull1p.size(), hull1p.data(), aBounds, NvTransform(NvIdentity),
                                          NvVec3(1, 1, 1), hull2p.size(), hull2p.data(), bBounds,
                                          NvTransform(NvIdentity), NvVec3(1, 1, 1), 2.0f * maxSeparation, &separation))
    {
        return 0.0f;
    }

    const bool have_geometry = (mA != nullptr && mB != nullptr) || (hull1 != nullptr && hull2 != nullptr);

    if (separation.getDistance() > 0 || !have_geometry)  // If chunks don't intersect then use midplane to produce bond,
                                                         // otherwise midplane can be wrong (only if we have geometry)
    {
        // Build first plane interface
        NvPlane midplane = separation.plane;
        if (!midplane.n.isFinite())
        {
            return 0.0f;
        }

        std::vector<NvVec3> interfacePoints;

        float firstCentroidSide  = (midplane.distance(chunk1Centroid) > 0) ? 1 : -1;
        float secondCentroidSide = (midplane.distance(chunk2Centroid) > 0) ? 1 : -1;

        for (uint32_t i = 0; i < hull1p.size(); ++i)
        {
            float dst = midplane.distance(hull1p[i]);
            if (dst * firstCentroidSide < maxSeparation)
            {
                interfacePoints.push_back(hull1p[i]);
            }
        }

        for (uint32_t i = 0; i < hull2p.size(); ++i)
        {
            float dst = midplane.distance(hull2p[i]);
            if (dst * secondCentroidSide < maxSeparation)
            {
                interfacePoints.push_back(hull2p[i]);
            }
        }
        std::vector<NvVec3> convexHull;
        trProcessor->buildConvexHull(interfacePoints, convexHull, midplane.n);
        float area = 0;
        NvVec3 centroidLocal(0, 0, 0);
        if (convexHull.size() < 3)
        {
            return 0.0f;
        }
        for (uint32_t i = 0; i < convexHull.size() - 1; ++i)
        {
            centroidLocal += convexHull[i];
            area += (convexHull[i] - convexHull[0]).cross((convexHull[i + 1] - convexHull[0])).magnitude();
        }
        centroidLocal += convexHull.back();
        centroidLocal *= (1.0f / convexHull.size());
        float direction = midplane.n.dot(chunk2Centroid - chunk1Centroid);
        if (direction < 0)
        {
            normal = -1.0f * normal;
        }
        normal   = midplane.n;
        centroid = centroidLocal;
        return area * 0.5f;
    }
    else
    {
        float area = 0.0f;
        std::vector<NvVec3> intersectionAnchors;
        if (hull1 != nullptr && hull2 != nullptr)  // Use hulls
        {
            for (uint32_t i1 = 0; i1 < hull1->polygonDataCount; ++i1)
            {
                HullPolygon& poly1 = hull1->polygonData[i1];
                for (uint32_t i2 = 0; i2 < hull2->polygonDataCount; ++i2)
                {
                    HullPolygon& poly2 = hull2->polygonData[i2];
                    AddPpAnchorPoints(reinterpret_cast<uint8_t*>(hull1->indices) + poly1.indexBase, poly1.vertexCount,
                                      toNvShared(hull1->points), poly1.plane,
                                      reinterpret_cast<uint8_t*>(hull2->indices) + poly2.indexBase, poly2.vertexCount,
                                      toNvShared(hull2->points), poly2.plane, intersectionAnchors);
                }
            }
        }
        else if (mA != nullptr && mB != nullptr)  // Use triangles
        {
            for (uint32_t i = 0; i < mavc; ++i)
            {
                for (uint32_t j = 0; j < mbvc; ++j)
                {
                    AddTtAnchorPoints(mA + i, mB + j, intersectionAnchors);
                }
            }
        }
        else
        {
            NVBLAST_ASSERT_WITH_MESSAGE(false, "collision hulls and triangle data are both invalid, this shouldn't happen");
            return 0.0f;
        }

        NvVec3 lcoid(0, 0, 0);
        for (uint32_t i = 0; i < intersectionAnchors.size(); ++i)
        {
            lcoid += intersectionAnchors[i];
        }
        lcoid *= (1.0f / intersectionAnchors.size());
        centroid = lcoid;

        if (intersectionAnchors.size() < 2)
        {
            return 0.0f;
        }

        NvVec3 dir1 = intersectionAnchors[0] - lcoid;
        NvVec3 dir2 = chunk2Centroid - chunk1Centroid;  // A more reasonable fallback than (0,0,0)
        float maxMagn = 0.0f;
        float maxDist = 0.0f;

        for (uint32_t j = 0; j < intersectionAnchors.size(); ++j)
        {
            float d = (intersectionAnchors[j] - lcoid).magnitude();

            NvVec3 tempNormal = (intersectionAnchors[j] - lcoid).cross(dir1);
            maxDist           = std::max(d, maxDist);

            if (tempNormal.magnitude() > maxMagn)
            {
                dir2 = tempNormal;
            }
        }

        normal = dir2.getNormalized();

        area = (maxDist * maxDist) * 3.14f;  // Compute area like circle area;

        return area;
    }
}


struct BondGenerationCandidate
{
    NvVec3 point;
    bool end;
    uint32_t parentChunk;
    uint32_t parentComponent;
    BondGenerationCandidate();
    BondGenerationCandidate(const NvVec3& p, bool isEnd, uint32_t pr, uint32_t c)
    : point(p), end(isEnd), parentChunk(pr), parentComponent(c){};

    bool operator<(const BondGenerationCandidate& in) const
    {
        if (point.x < in.point.x)
            return true;
        if (point.x > in.point.x)
            return false;

        if (point.y < in.point.y)
            return true;
        if (point.y > in.point.y)
            return false;

        if (point.z < in.point.z)
            return true;
        if (point.z > in.point.z)
            return false;

        return end < in.end;
    };
};


int32_t BlastBondGeneratorImpl::createFullBondListAveraged(uint32_t meshCount, const uint32_t* geometryOffset,
                                                           const Triangle* geometry, const CollisionHull** chunkHulls,
                                                           const bool* supportFlags, const uint32_t* meshGroups,
                                                           NvBlastBondDesc*& resultBondDescs, BondGenerationConfig conf,
                                                           std::set<std::pair<uint32_t, uint32_t> >* pairNotToTest)
{

    std::vector<std::vector<NvcVec3> > chunksPoints(meshCount);
    std::vector<NvBounds3> bounds(meshCount);
    if (!chunkHulls)
    {
        for (uint32_t i = 0; i < meshCount; ++i)
        {
            bounds[i].setEmpty();
            if (!supportFlags[i])
            {
                continue;
            }
            uint32_t count = geometryOffset[i + 1] - geometryOffset[i];
            for (uint32_t j = 0; j < count; ++j)
            {
                chunksPoints[i].push_back(geometry[geometryOffset[i] + j].a.p);
                chunksPoints[i].push_back(geometry[geometryOffset[i] + j].b.p);
                chunksPoints[i].push_back(geometry[geometryOffset[i] + j].c.p);
                bounds[i].include(toNvShared(geometry[geometryOffset[i] + j].a.p));
                bounds[i].include(toNvShared(geometry[geometryOffset[i] + j].b.p));
                bounds[i].include(toNvShared(geometry[geometryOffset[i] + j].c.p));
            }
        }
    }

    std::vector<std::vector<std::vector<NvVec3> > > hullPoints(meshCount);
    std::vector<BondGenerationCandidate> candidates;

    std::vector<CollisionHull*> tempChunkHulls(meshCount, nullptr);
    for (uint32_t chunk = 0; chunk < meshCount; ++chunk)
    {
        if (!supportFlags[chunk])
        {
            continue;
        }
        NvBounds3 bnd(NvBounds3::empty());
        uint32_t hullCountForMesh             = 0;
        const CollisionHull** beginChunkHulls = nullptr;
        if (chunkHulls)
        {
            hullCountForMesh = geometryOffset[chunk + 1] - geometryOffset[chunk];
            beginChunkHulls  = chunkHulls + geometryOffset[chunk];
        }
        else
        {
            // build a convex hull and store it in the temp slot
            tempChunkHulls[chunk] =
                mConvexMeshBuilder->buildCollisionGeometry(chunksPoints[chunk].size(), chunksPoints[chunk].data());
            hullCountForMesh = 1;
            beginChunkHulls  = const_cast<const CollisionHull**>(&tempChunkHulls[chunk]);
        }

        hullPoints[chunk].resize(hullCountForMesh);
        for (uint32_t hull = 0; hull < hullCountForMesh; ++hull)
        {
            auto& curHull             = hullPoints[chunk][hull];
            const uint32_t pointCount = beginChunkHulls[hull]->pointsCount;
            curHull.resize(pointCount);
            for (uint32_t i = 0; i < pointCount; ++i)
            {
                curHull[i] = toNvShared(beginChunkHulls[hull]->points[i]);
                bnd.include(curHull[i]);
            }
        }

        float minSide = bnd.getDimensions().abs().minElement();
        if (minSide > 0.f)
        {
            float scaling = std::max(1.1f, conf.maxSeparation / (minSide));
            bnd.scaleFast(scaling);
        }
        candidates.push_back(
            BondGenerationCandidate(bnd.minimum, false, chunk, meshGroups != nullptr ? meshGroups[chunk] : 0));
        candidates.push_back(
            BondGenerationCandidate(bnd.maximum, true, chunk, meshGroups != nullptr ? meshGroups[chunk] : 0));
    }

    std::sort(candidates.begin(), candidates.end());

    std::set<uint32_t> listOfActiveChunks;
    std::vector<std::vector<uint32_t> > possibleBondGraph(meshCount);

    for (uint32_t idx = 0; idx < candidates.size(); ++idx)
    {
        if (!candidates[idx].end)  // If new candidate
        {
            for (uint32_t activeChunk : listOfActiveChunks)
            {
                if (meshGroups != nullptr && (meshGroups[activeChunk] == candidates[idx].parentComponent))
                    continue;  // Don't connect components with itself.
                possibleBondGraph[activeChunk].push_back(candidates[idx].parentChunk);
            }
            listOfActiveChunks.insert(candidates[idx].parentChunk);
        }
        else
        {
            listOfActiveChunks.erase(candidates[idx].parentChunk);
        }
    }

    TriangleProcessor trProcessor;
    std::vector<NvBlastBondDesc> mResultBondDescs;
    for (uint32_t i = 0; i < meshCount; ++i)
    {
        const uint32_t ihullCount = hullPoints[i].size();
        for (uint32_t tj = 0; tj < possibleBondGraph[i].size(); ++tj)
        {
            uint32_t j = possibleBondGraph[i][tj];

            auto pr = (i < j) ? std::make_pair(i, j) : std::make_pair(j, i);

            if (pairNotToTest != nullptr && pairNotToTest->find(pr) != pairNotToTest->end())
            {
                continue;  // This chunks should not generate bonds. This is used for mixed generation with bondFrom
            }

            const uint32_t jhullCount = hullPoints[j].size();
            for (uint32_t ihull = 0; ihull < ihullCount; ++ihull)
            {
                for (uint32_t jhull = 0; jhull < jhullCount; ++jhull)
                {
                    NvVec3 normal;
                    NvVec3 centroid;

                    float area = processWithMidplanes(
                        &trProcessor, geometry ? geometry + geometryOffset[i] : nullptr,
                        geometryOffset[i + 1] - geometryOffset[i], geometry ? geometry + geometryOffset[j] : nullptr,
                        geometryOffset[j + 1] - geometryOffset[j],
                        chunkHulls ? chunkHulls[geometryOffset[i] + ihull] : tempChunkHulls[i],
                        chunkHulls ? chunkHulls[geometryOffset[j] + jhull] : tempChunkHulls[j],
                        hullPoints[i][ihull], hullPoints[j][jhull], normal, centroid, conf.maxSeparation);

                    if (area > 0)
                    {
                        NvBlastBondDesc bDesc  = NvBlastBondDesc();
                        bDesc.chunkIndices[0]  = i;
                        bDesc.chunkIndices[1]  = j;
                        bDesc.bond.area        = area;
                        bDesc.bond.centroid[0] = centroid.x;
                        bDesc.bond.centroid[1] = centroid.y;
                        bDesc.bond.centroid[2] = centroid.z;

                        uint32_t maxIndex = std::max(i, j);
                        if ((bounds[maxIndex].getCenter() - centroid).dot(normal) < 0)
                        {
                            normal = -normal;
                        }

                        bDesc.bond.normal[0] = normal.x;
                        bDesc.bond.normal[1] = normal.y;
                        bDesc.bond.normal[2] = normal.z;

                        mResultBondDescs.push_back(bDesc);
                    }
                }
            }
        }
    }

    // release any temp hulls allocated
    for (CollisionHull* tempHullPtr : tempChunkHulls)
    {
        if (tempHullPtr)
        {
            mConvexMeshBuilder->releaseCollisionHull(tempHullPtr);
        }
    }

    resultBondDescs = SAFE_ARRAY_NEW(NvBlastBondDesc, mResultBondDescs.size());
    memcpy(resultBondDescs, mResultBondDescs.data(), sizeof(NvBlastBondDesc) * mResultBondDescs.size());
    return mResultBondDescs.size();
}

uint32_t isSamePlane(NvcPlane& a, NvcPlane& b)
{
    if (NvAbs(a.d - b.d) > EPS_PLANE)
        return 0;
    if (NvAbs(a.n.x - b.n.x) > EPS_PLANE)
        return 0;
    if (NvAbs(a.n.y - b.n.y) > EPS_PLANE)
        return 0;
    if (NvAbs(a.n.z - b.n.z) > EPS_PLANE)
        return 0;
    return 1;
}

int32_t BlastBondGeneratorImpl::createFullBondListExact(uint32_t meshCount, const uint32_t* geometryOffset,
                                                        const Triangle* geometry, const bool* supportFlags,
                                                        NvBlastBondDesc*& resultBondDescs, BondGenerationConfig conf)
{
    std::vector<PlaneChunkIndexer> planeTriangleMapping;
    NV_UNUSED(conf);
    for (uint32_t i = 0; i < meshCount; ++i)
    {
        if (!supportFlags[i])
        {
            continue;
        }
        uint32_t count = geometryOffset[i + 1] - geometryOffset[i];
        for (uint32_t j = 0; j < count; ++j)
        {
#ifdef DEBUG_OUTPUT
            meshBuffer.push_back(geometry[geometryOffset[i] + j].a.p);
            meshBuffer.push_back(geometry[geometryOffset[i] + j].b.p);
            meshBuffer.push_back(geometry[geometryOffset[i] + j].c.p);
#endif

            NvcPlane nPlane = fromNvShared(nvidia::NvPlane(toNvShared(geometry[geometryOffset[i] + j].a.p),
                                                       toNvShared(geometry[geometryOffset[i] + j].b.p),
                                                       toNvShared(geometry[geometryOffset[i] + j].c.p)));
            planeTriangleMapping.push_back({ (int32_t)i, (int32_t)j, nPlane });
        }
    }

    std::sort(planeTriangleMapping.begin(), planeTriangleMapping.end(), planeComparer);
    return createFullBondListExactInternal(meshCount, geometryOffset, geometry, planeTriangleMapping, resultBondDescs);
}

void BlastBondGeneratorImpl::buildGeometryCache(uint32_t meshCount, const uint32_t* geometryOffset,
                                                const Triangle* geometry)
{
    uint32_t geometryCount = geometryOffset[meshCount];
    for (uint32_t i = 0; i < meshCount; i++)
    {
        mGeometryCache.push_back(std::vector<Triangle>());
        uint32_t count = geometryOffset[i + 1] - geometryOffset[i];
        mGeometryCache.back().resize(count);
        memcpy(mGeometryCache.back().data(), geometry + geometryOffset[i], sizeof(Triangle) * count);
    }
    mHullsPointsCache.resize(geometryCount);
    mBoundsCache.resize(geometryCount);
    mCHullCache.resize(geometryCount);
    for (uint32_t i = 0; i < mGeometryCache.size(); ++i)
    {
        for (uint32_t j = 0; j < mGeometryCache[i].size(); ++j)
        {

            NvcPlane nPlane =
                fromNvShared(nvidia::NvPlane(toNvShared(mGeometryCache[i][j].a.p), toNvShared(mGeometryCache[i][j].b.p),
                                         toNvShared(mGeometryCache[i][j].c.p)));
            mPlaneCache.push_back({ (int32_t)i, (int32_t)j, nPlane });
        }
    }

    for (uint32_t ch = 0; ch < mGeometryCache.size(); ++ch)
    {
        std::vector<NvcVec3> chunksPoints(mGeometryCache[ch].size() * 3);

        int32_t sp = 0;
        for (uint32_t i = 0; i < mGeometryCache[ch].size(); ++i)
        {
            chunksPoints[sp++] = mGeometryCache[ch][i].a.p;
            chunksPoints[sp++] = mGeometryCache[ch][i].b.p;
            chunksPoints[sp++] = mGeometryCache[ch][i].c.p;
        }

        mCHullCache[ch] = mConvexMeshBuilder->buildCollisionGeometry(chunksPoints.size(), chunksPoints.data());

        mHullsPointsCache[ch].resize(mCHullCache[ch]->pointsCount);

        mBoundsCache[ch].setEmpty();
        for (uint32_t i = 0; i < mCHullCache[ch]->pointsCount; ++i)
        {
            mHullsPointsCache[ch][i] = toNvShared(mCHullCache[ch]->points[i]);
            mBoundsCache[ch].include(mHullsPointsCache[ch][i]);
        }
    }
}

void BlastBondGeneratorImpl::resetGeometryCache()
{
    mGeometryCache.clear();
    mPlaneCache.clear();
    mHullsPointsCache.clear();
    for (auto h : mCHullCache)
    {
        mConvexMeshBuilder->releaseCollisionHull(h);
    }
    mCHullCache.clear();
    mBoundsCache.clear();
}

int32_t BlastBondGeneratorImpl::createFullBondListExactInternal(uint32_t meshCount, const uint32_t* geometryOffset,
                                                                const Triangle* geometry,
                                                                std::vector<PlaneChunkIndexer>& planeTriangleMapping,
                                                                NvBlastBondDesc*& resultBondDescs)
{
    NV_UNUSED(meshCount);

    std::map<std::pair<int32_t, int32_t>, std::pair<NvBlastBondDesc, int32_t> > bonds;

    TriangleProcessor trPrc;
    std::vector<NvVec3> intersectionBufferLocal;

    NvBlastBondDesc cleanBond = NvBlastBondDesc();
    memset(&cleanBond, 0, sizeof(NvBlastBondDesc));
    for (uint32_t tIndex = 0; tIndex < planeTriangleMapping.size(); ++tIndex)
    {

        PlaneChunkIndexer opp = planeTriangleMapping[tIndex];

        opp.plane.d *= -1;
        opp.plane.n = opp.plane.n * - 1;

        uint32_t startIndex =
            (uint32_t)(std::lower_bound(planeTriangleMapping.begin(), planeTriangleMapping.end(), opp, planeComparer) -
                       planeTriangleMapping.begin());
        uint32_t endIndex =
            (uint32_t)(std::upper_bound(planeTriangleMapping.begin(), planeTriangleMapping.end(), opp, planeComparer) -
                       planeTriangleMapping.begin());
        //  uint32_t startIndex = 0;
        //  uint32_t endIndex = (uint32_t)planeTriangleMapping.size();

        PlaneChunkIndexer& mappedTr = planeTriangleMapping[tIndex];
        const Triangle& trl         = geometry[geometryOffset[mappedTr.chunkId] + mappedTr.trId];
        NvPlane pln                 = toNvShared(mappedTr.plane);
        TrPrcTriangle trp(toNvShared(trl.a.p), toNvShared(trl.b.p), toNvShared(trl.c.p));
        NvVec3 trCentroid = toNvShared(trl.a.p + trl.b.p + trl.c.p) * (1.0f / 3.0f);
        trp.points[0] -= trCentroid;
        trp.points[1] -= trCentroid;
        trp.points[2] -= trCentroid;
        ProjectionDirections pDir = getProjectionDirection(pln.n);
        TrPrcTriangle2d trp2d;
        trp2d.points[0] = getProjectedPointWithWinding(trp.points[0], pDir);
        trp2d.points[1] = getProjectedPointWithWinding(trp.points[1], pDir);
        trp2d.points[2] = getProjectedPointWithWinding(trp.points[2], pDir);

        for (uint32_t i = startIndex; i <= endIndex && i < planeTriangleMapping.size(); ++i)
        {
            PlaneChunkIndexer& mappedTr2 = planeTriangleMapping[i];
            if (mappedTr2.trId == opp.chunkId)
            {
                continue;
            }

            if (!isSamePlane(opp.plane, mappedTr2.plane))
            {
                continue;
            }

            if (mappedTr.chunkId == mappedTr2.chunkId)
            {
                continue;
            }
            std::pair<int32_t, int32_t> bondEndPoints = std::make_pair(mappedTr.chunkId, mappedTr2.chunkId);
            if (bondEndPoints.second < bondEndPoints.first)
                continue;
            std::pair<int32_t, int32_t> bondEndPointsSwapped = std::make_pair(mappedTr2.chunkId, mappedTr.chunkId);
            if (bonds.find(bondEndPoints) == bonds.end() && bonds.find(bondEndPointsSwapped) != bonds.end())
            {
                continue;  // We do not need account interface surface twice
            }
            if (bonds.find(bondEndPoints) == bonds.end())
            {
                bonds[bondEndPoints].second                = 0;
                bonds[bondEndPoints].first                 = cleanBond;
                bonds[bondEndPoints].first.chunkIndices[0] = bondEndPoints.first;
                bonds[bondEndPoints].first.chunkIndices[1] = bondEndPoints.second;
                bonds[bondEndPoints].first.bond.normal[0]  = pln.n[0];
                bonds[bondEndPoints].first.bond.normal[1]  = pln.n[1];
                bonds[bondEndPoints].first.bond.normal[2]  = pln.n[2];
            }
            const Triangle& trl2 = geometry[geometryOffset[mappedTr2.chunkId] + mappedTr2.trId];

            TrPrcTriangle trp2(toNvShared(trl2.a.p), toNvShared(trl2.b.p), toNvShared(trl2.c.p));

            intersectionBufferLocal.clear();
            intersectionBufferLocal.reserve(32);
            trPrc.getTriangleIntersection(trp, trp2d, trp2, trCentroid, intersectionBufferLocal, pln.n);
            NvVec3 centroidPoint(0, 0, 0);
            int32_t collectedVerticesCount = 0;
            float area                     = 0;
            if (intersectionBufferLocal.size() >= 3)
            {
#ifdef DEBUG_OUTPUT
                for (uint32_t p = 1; p < intersectionBufferLocal.size() - 1; ++p)
                {
                    intersectionBuffer.push_back(intersectionBufferLocal[0]);
                    intersectionBuffer.push_back(intersectionBufferLocal[p]);
                    intersectionBuffer.push_back(intersectionBufferLocal[p + 1]);
                }
#endif
                centroidPoint          = intersectionBufferLocal[0] + intersectionBufferLocal.back();
                collectedVerticesCount = 2;

                for (uint32_t j = 1; j < intersectionBufferLocal.size() - 1; ++j)
                {
                    ++collectedVerticesCount;
                    centroidPoint += intersectionBufferLocal[j];
                    area += (intersectionBufferLocal[j + 1] - intersectionBufferLocal[0])
                                .cross(intersectionBufferLocal[j] - intersectionBufferLocal[0])
                                .magnitude();
                }
            }
            if (area > 0.00001f)
            {
                bonds[bondEndPoints].second += collectedVerticesCount;

                bonds[bondEndPoints].first.bond.area += area * 0.5f;
                bonds[bondEndPoints].first.bond.centroid[0] += (centroidPoint.x);
                bonds[bondEndPoints].first.bond.centroid[1] += (centroidPoint.y);
                bonds[bondEndPoints].first.bond.centroid[2] += (centroidPoint.z);
            }
        }
    }

    std::vector<NvBlastBondDesc> mResultBondDescs;
    for (auto it : bonds)
    {
        if (it.second.first.bond.area > 0)
        {
            float mlt = 1.0f / (it.second.second);
            it.second.first.bond.centroid[0] *= mlt;
            it.second.first.bond.centroid[1] *= mlt;
            it.second.first.bond.centroid[2] *= mlt;

            mResultBondDescs.push_back(it.second.first);
        }
    }
#ifdef DEBUG_OUTPUT
    saveGeometryToObj(meshBuffer, "Mesh.obj");
    saveGeometryToObj(intersectionBuffer, "inter.obj");
#endif
    resultBondDescs = SAFE_ARRAY_NEW(NvBlastBondDesc, mResultBondDescs.size());
    memcpy(resultBondDescs, mResultBondDescs.data(), sizeof(NvBlastBondDesc) * mResultBondDescs.size());
    return mResultBondDescs.size();
}

int32_t BlastBondGeneratorImpl::createBondForcedInternal(const std::vector<NvVec3>& hull0,
                                                         const std::vector<NvVec3>& hull1, const CollisionHull& cHull0,
                                                         const CollisionHull& cHull1, NvBounds3 bound0,
                                                         NvBounds3 bound1, NvBlastBond& resultBond, float overlapping)
{

    TriangleProcessor trProcessor;
    Separation separation;
    importerHullsInProximityApexFree(hull0.size(), hull0.data(), bound0, NvTransform(NvIdentity), NvVec3(1, 1, 1),
                                     hull1.size(), hull1.data(), bound1, NvTransform(NvIdentity), NvVec3(1, 1, 1),
                                     0.000, &separation);

    if (std::isnan(separation.plane.d))
    {
        importerHullsInProximityApexFree(
            hull0.size(), hull0.data(), bound0, NvTransform(NvVec3(0.000001f, 0.000001f, 0.000001f)), NvVec3(1, 1, 1),
            hull1.size(), hull1.data(), bound1, NvTransform(NvIdentity), NvVec3(1, 1, 1), 0.000, &separation);
        if (std::isnan(separation.plane.d))
        {
            return 1;
        }
    }

    NvPlane pl = separation.plane;
    std::vector<NvVec3> ifsPoints[2];

    float dst[2][2];

    dst[0][0] = 0;
    dst[0][1] = MAXIMUM_EXTENT;
    for (uint32_t p = 0; p < cHull0.pointsCount; ++p)
    {
        float d = pl.distance(toNvShared(cHull0.points[p]));
        if (NvAbs(d) > NvAbs(dst[0][0]))
        {
            dst[0][0] = d;
        }
        if (NvAbs(d) < NvAbs(dst[0][1]))
        {
            dst[0][1] = d;
        }
    }

    dst[1][0] = 0;
    dst[1][1] = MAXIMUM_EXTENT;
    for (uint32_t p = 0; p < cHull1.pointsCount; ++p)
    {
        float d = pl.distance(toNvShared(cHull0.points[p]));
        if (NvAbs(d) > NvAbs(dst[1][0]))
        {
            dst[1][0] = d;
        }
        if (NvAbs(d) < NvAbs(dst[1][1]))
        {
            dst[1][1] = d;
        }
    }


    float cvOffset[2] = { dst[0][1] + (dst[0][0] - dst[0][1]) * overlapping,
                          dst[1][1] + (dst[1][0] - dst[1][1]) * overlapping };

    for (uint32_t i = 0; i < cHull0.polygonDataCount; ++i)
    {
        auto& pd = cHull0.polygonData[i];
        NvVec3 result;
        for (uint32_t j = 0; j < pd.vertexCount; ++j)
        {
            uint32_t nxj        = (j + 1) % pd.vertexCount;
            const uint32_t* ind = cHull0.indices;
            NvVec3 a            = hull0[ind[j + pd.indexBase]] - pl.n * cvOffset[0];
            NvVec3 b            = hull0[ind[nxj + pd.indexBase]] - pl.n * cvOffset[0];

            if (getPlaneSegmentIntersection(pl, a, b, result))
            {
                ifsPoints[0].push_back(result);
            }
        }
    }

    for (uint32_t i = 0; i < cHull1.polygonDataCount; ++i)
    {
        auto& pd = cHull1.polygonData[i];
        NvVec3 result;
        for (uint32_t j = 0; j < pd.vertexCount; ++j)
        {
            uint32_t nxj        = (j + 1) % pd.vertexCount;
            const uint32_t* ind = cHull1.indices;
            NvVec3 a            = hull1[ind[j + pd.indexBase]] - pl.n * cvOffset[1];
            NvVec3 b            = hull1[ind[nxj + pd.indexBase]] - pl.n * cvOffset[1];

            if (getPlaneSegmentIntersection(pl, a, b, result))
            {
                ifsPoints[1].push_back(result);
            }
        }
    }


    std::vector<NvVec3> convexes[2];

    trProcessor.buildConvexHull(ifsPoints[0], convexes[0], pl.n);
    trProcessor.buildConvexHull(ifsPoints[1], convexes[1], pl.n);

    float areas[2]      = { 0, 0 };
    NvVec3 centroids[2] = { NvVec3(0, 0, 0), NvVec3(0, 0, 0) };

    for (uint32_t cv = 0; cv < 2; ++cv)
    {
        if (convexes[cv].size() == 0)
        {
            continue;
        }
        centroids[cv] = convexes[cv][0] + convexes[cv].back();
        for (uint32_t i = 1; i < convexes[cv].size() - 1; ++i)
        {
            centroids[cv] += convexes[cv][i];
            areas[cv] += (convexes[cv][i + 1] - convexes[cv][0]).cross(convexes[cv][i] - convexes[cv][0]).magnitude();
#ifdef DEBUG_OUTPUT
            intersectionBuffer.push_back(convexes[cv][0]);
            intersectionBuffer.push_back(convexes[cv][i]);
            intersectionBuffer.push_back(convexes[cv][i + 1]);
#endif
        }
        centroids[cv] *= (1.0f / convexes[cv].size());
        areas[cv] = NvAbs(areas[cv]);
    }

    resultBond.area        = (areas[0] + areas[1]) * 0.5f;
    resultBond.centroid[0] = (centroids[0][0] + centroids[1][0]) * 0.5f;
    resultBond.centroid[1] = (centroids[0][1] + centroids[1][1]) * 0.5f;
    resultBond.centroid[2] = (centroids[0][2] + centroids[1][2]) * 0.5f;
    resultBond.normal[0]   = pl.n[0];
    resultBond.normal[1]   = pl.n[1];
    resultBond.normal[2]   = pl.n[2];
    resultBond.userData    = 0;

#ifdef DEBUG_OUTPUT
    saveGeometryToObj(meshBuffer, "ArbitMeshes.obj");
    saveGeometryToObj(intersectionBuffer, "inter.obj");
#endif


    return 0;
}

int32_t BlastBondGeneratorImpl::buildDescFromInternalFracture(FractureTool* tool, const bool* chunkIsSupport,
                                                              NvBlastBondDesc*& resultBondDescs,
                                                              NvBlastChunkDesc*& resultChunkDescriptors)
{
    uint32_t chunkCount = tool->getChunkCount();
    std::vector<uint32_t> trianglesCount(chunkCount);
    std::vector<std::shared_ptr<Triangle> > trianglesBuffer;

    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        Triangle* t;
        trianglesCount[i] = tool->getBaseMesh(i, t);
        trianglesBuffer.push_back(std::shared_ptr<Triangle>(t, [](Triangle* t) { delete[] t; }));
    }

    if (chunkCount == 0)
    {
        return 0;
    }

    resultChunkDescriptors = SAFE_ARRAY_NEW(NvBlastChunkDesc, trianglesBuffer.size());
    std::vector<Bond> bondDescriptors;
    bool hasApproximateBonding = false;

    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        NvBlastChunkDesc& desc = resultChunkDescriptors[i];
        desc.userData              = tool->getChunkId(i);
        desc.parentChunkDescIndex  = tool->getChunkInfoIndex(tool->getChunkInfo(i).parentChunkId);
        desc.flags                 = NvBlastChunkDesc::NoFlags;
        hasApproximateBonding |= !!(tool->getChunkInfo(i).flags & ChunkInfo::APPROXIMATE_BONDING);
        if (chunkIsSupport[i])
        {
            desc.flags = NvBlastChunkDesc::SupportFlag;
        }
        NvVec3 chunkCentroid(0, 0, 0);
        for (uint32_t tr = 0; tr < trianglesCount[i]; ++tr)
        {
            auto& trRef = trianglesBuffer[i].get()[tr];
            chunkCentroid += toNvShared(trRef.a.p);
            chunkCentroid += toNvShared(trRef.b.p);
            chunkCentroid += toNvShared(trRef.c.p);

            int32_t id = trRef.userData;
            if (id == 0)
                continue;
            bondDescriptors.push_back(Bond());
            Bond& bond         = bondDescriptors.back();
            bond.m_chunkId     = i;
            bond.m_planeIndex  = id;
            bond.triangleIndex = tr;
        }
        chunkCentroid *= (1.0f / (3 * trianglesCount[i]));
        desc.centroid[0] = chunkCentroid[0];
        desc.centroid[1] = chunkCentroid[1];
        desc.centroid[2] = chunkCentroid[2];
    }
    std::sort(bondDescriptors.begin(), bondDescriptors.end());


    std::vector<NvBlastBondDesc> mResultBondDescs;

    if (!bondDescriptors.empty())
    {

        int32_t chunkId, planeId;
        chunkId = bondDescriptors[0].m_chunkId;
        planeId = bondDescriptors[0].m_planeIndex;
        std::vector<BondInfo> forwardChunks;
        std::vector<BondInfo> backwardChunks;

        float area = 0;
        NvVec3 normal(0, 0, 0);
        NvVec3 centroid(0, 0, 0);
        int32_t collected = 0;
        NvBounds3 bb      = NvBounds3::empty();

        chunkId = -1;
        planeId = bondDescriptors[0].m_planeIndex;
        for (uint32_t i = 0; i <= bondDescriptors.size(); ++i)
        {
            if (i == bondDescriptors.size() ||
                (chunkId != bondDescriptors[i].m_chunkId || abs(planeId) != abs(bondDescriptors[i].m_planeIndex)))
            {
                if (chunkId != -1)
                {
                    area = 0.5f * normal.normalize();
                    centroid /= 3.0f * collected;
                    if (bondDescriptors[i - 1].m_planeIndex > 0)
                    {
                        forwardChunks.push_back(BondInfo());
                        forwardChunks.back().area      = area;
                        forwardChunks.back().normal    = normal;
                        forwardChunks.back().centroid  = centroid;
                        forwardChunks.back().m_chunkId = chunkId;
                        forwardChunks.back().m_bb      = bb;
                    }
                    else
                    {
                        backwardChunks.push_back(BondInfo());
                        backwardChunks.back().area      = area;
                        backwardChunks.back().normal    = normal;
                        backwardChunks.back().centroid  = centroid;
                        backwardChunks.back().m_chunkId = chunkId;
                        backwardChunks.back().m_bb      = bb;
                    }
                }
                bb.setEmpty();
                collected = 0;
                area      = 0;
                normal    = NvVec3(0, 0, 0);
                centroid  = NvVec3(0, 0, 0);
                if (i != bondDescriptors.size())
                    chunkId = bondDescriptors[i].m_chunkId;
            }
            if (i == bondDescriptors.size() || abs(planeId) != abs(bondDescriptors[i].m_planeIndex))
            {
                for (uint32_t fchunk = 0; fchunk < forwardChunks.size(); ++fchunk)
                {
                    const BondInfo& fInfo = forwardChunks[fchunk];
                    if (chunkIsSupport[fInfo.m_chunkId] == false)
                    {
                        continue;
                    }
                    for (uint32_t bchunk = 0; bchunk < backwardChunks.size(); ++bchunk)
                    {
                        const BondInfo& bInfo = backwardChunks[bchunk];
                        if (weakBoundingBoxIntersection(fInfo.m_bb, bInfo.m_bb) == 0)
                        {
                            continue;
                        }
                        if (chunkIsSupport[bInfo.m_chunkId] == false)
                        {
                            continue;
                        }

                        mResultBondDescs.push_back(NvBlastBondDesc());
                        NvBlastBondDesc& bondDesc = mResultBondDescs.back();

                        // Use the minimum-area patch for the bond area and centroid
                        if (fInfo.area < bInfo.area)
                        {
                            bondDesc.bond.area = fInfo.area;
                            bondDesc.bond.centroid[0] = fInfo.centroid.x;
                            bondDesc.bond.centroid[1] = fInfo.centroid.y;
                            bondDesc.bond.centroid[2] = fInfo.centroid.z;
                            bondDesc.bond.normal[0] = fInfo.normal.x;
                            bondDesc.bond.normal[1] = fInfo.normal.y;
                            bondDesc.bond.normal[2] = fInfo.normal.z;
                        }
                        else
                        {
                            bondDesc.bond.area = bInfo.area;
                            bondDesc.bond.centroid[0] = bInfo.centroid.x;
                            bondDesc.bond.centroid[1] = bInfo.centroid.y;
                            bondDesc.bond.centroid[2] = bInfo.centroid.z;
                            bondDesc.bond.normal[0] = -bInfo.normal.x;
                            bondDesc.bond.normal[1] = -bInfo.normal.y;
                            bondDesc.bond.normal[2] = -bInfo.normal.z;
                        }

                        bondDesc.chunkIndices[0] = fInfo.m_chunkId;
                        bondDesc.chunkIndices[1] = bInfo.m_chunkId;
                    }
                }
                forwardChunks.clear();
                backwardChunks.clear();
                if (i != bondDescriptors.size())
                {
                    planeId = bondDescriptors[i].m_planeIndex;
                }
                else
                {
                    break;
                }
            }

            collected++;
            auto& trRef = trianglesBuffer[chunkId].get()[bondDescriptors[i].triangleIndex];
            normal += getNormal(trRef);
            centroid += toNvShared(trRef.a.p);
            centroid += toNvShared(trRef.b.p);
            centroid += toNvShared(trRef.c.p);

            bb.include(toNvShared(trRef.a.p));
            bb.include(toNvShared(trRef.b.p));
            bb.include(toNvShared(trRef.c.p));
        }
    }

    if (hasApproximateBonding)
    {
        std::vector<Triangle> chunkTriangles;
        std::vector<uint32_t> chunkTrianglesOffsets;

        std::set<std::pair<uint32_t, uint32_t> > pairsAlreadyCreated;

        for (uint32_t i = 0; i < mResultBondDescs.size(); ++i)
        {
            auto pr = (mResultBondDescs[i].chunkIndices[0] < mResultBondDescs[i].chunkIndices[1]) ?
                          std::make_pair(mResultBondDescs[i].chunkIndices[0], mResultBondDescs[i].chunkIndices[1]) :
                          std::make_pair(mResultBondDescs[i].chunkIndices[1], mResultBondDescs[i].chunkIndices[0]);

            pairsAlreadyCreated.insert(pr);
        }

        const float EXPANSION = 0.01f;

        chunkTrianglesOffsets.push_back(0);
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            const float SCALE_FACTOR = 1.001f;
            NvcVec3 centroid = {resultChunkDescriptors[i].centroid[0], resultChunkDescriptors[i].centroid[1],
                            resultChunkDescriptors[i].centroid[2]};
            for (uint32_t k = 0; k < trianglesCount[i]; ++k)
            {
                chunkTriangles.push_back(trianglesBuffer[i].get()[k]);

                // inflate mesh a bit
                chunkTriangles.back().a.p = chunkTriangles.back().a.p + (chunkTriangles.back().a.p - centroid) * EXPANSION;
                chunkTriangles.back().b.p = chunkTriangles.back().b.p + (chunkTriangles.back().b.p - centroid) * EXPANSION;
                chunkTriangles.back().c.p = chunkTriangles.back().c.p + (chunkTriangles.back().c.p - centroid) * EXPANSION;
            }
            chunkTrianglesOffsets.push_back(chunkTriangles.size());
        }

        NvBlastBondDesc* adsc;

        BondGenerationConfig cfg;
        cfg.bondMode      = BondGenerationConfig::AVERAGE;
        cfg.maxSeparation = EXPANSION;

        uint32_t nbListSize =
            createFullBondListAveraged(chunkCount, chunkTrianglesOffsets.data(), chunkTriangles.data(), nullptr,
                                       chunkIsSupport, nullptr, adsc, cfg, &pairsAlreadyCreated);

        for (uint32_t i = 0; i < nbListSize; ++i)
        {
            mResultBondDescs.push_back(adsc[i]);
        }
        NVBLAST_FREE(adsc);
    }

    resultBondDescs = SAFE_ARRAY_NEW(NvBlastBondDesc, mResultBondDescs.size());
    memcpy(resultBondDescs, mResultBondDescs.data(), sizeof(NvBlastBondDesc) * mResultBondDescs.size());

    return mResultBondDescs.size();
}

int32_t BlastBondGeneratorImpl::createBondBetweenMeshes(uint32_t meshCount, const uint32_t* geometryOffset,
                                                        const Triangle* geometry, uint32_t overlapsCount,
                                                        const uint32_t* overlapsA, const uint32_t* overlapsB,
                                                        NvBlastBondDesc*& resultBond, BondGenerationConfig cfg)
{
    if (cfg.bondMode == BondGenerationConfig::AVERAGE)
    {
        resetGeometryCache();
        buildGeometryCache(meshCount, geometryOffset, geometry);
    }
    resultBond = SAFE_ARRAY_NEW(NvBlastBondDesc, overlapsCount);

    if (cfg.bondMode == BondGenerationConfig::EXACT)
    {
        for (uint32_t i = 0; i < overlapsCount; ++i)
        {
            NvBlastBondDesc& desc = resultBond[i];
            desc.chunkIndices[0]  = overlapsA[i];
            desc.chunkIndices[1]  = overlapsB[i];
            uint32_t meshACount   = geometryOffset[overlapsA[i] + 1] - geometryOffset[overlapsA[i]];
            uint32_t meshBCount   = geometryOffset[overlapsB[i] + 1] - geometryOffset[overlapsB[i]];
            createBondBetweenMeshes(meshACount, geometry + geometryOffset[overlapsA[i]], meshBCount,
                                    geometry + geometryOffset[overlapsB[i]], desc.bond, cfg);
        }
    }
    else
    {
        for (uint32_t i = 0; i < overlapsCount; ++i)
        {
            NvBlastBondDesc& desc = resultBond[i];
            desc.chunkIndices[0]  = overlapsA[i];
            desc.chunkIndices[1]  = overlapsB[i];
            createBondForcedInternal(mHullsPointsCache[overlapsA[i]], mHullsPointsCache[overlapsB[i]],
                                     *mCHullCache[overlapsA[i]], *mCHullCache[overlapsB[i]], mBoundsCache[overlapsA[i]],
                                     mBoundsCache[overlapsB[i]], desc.bond, 0.3f);
        }
    }

    return overlapsCount;
}

int32_t BlastBondGeneratorImpl::createBondBetweenMeshes(uint32_t meshACount, const Triangle* meshA, uint32_t meshBCount,
                                                        const Triangle* meshB, NvBlastBond& resultBond,
                                                        BondGenerationConfig conf)
{
    float overlapping = 0.3f;
    if (conf.bondMode == BondGenerationConfig::EXACT)
    {
        std::vector<uint32_t> chunksOffsets = { 0, meshACount, meshACount + meshBCount };
        std::vector<Triangle> chunks;
        chunks.resize(meshACount + meshBCount);
        memcpy(chunks.data(), meshA, sizeof(Triangle) * meshACount);
        memcpy(chunks.data() + meshACount, meshB, sizeof(Triangle) * meshBCount);
        std::shared_ptr<bool> isSupport(new bool[2]{ true, true }, [](bool* b) { delete[] b; });
        NvBlastBondDesc* desc;
        uint32_t descSize = createFullBondListExact(2, chunksOffsets.data(), chunks.data(), isSupport.get(), desc, conf);
        if (descSize > 0)
        {
            resultBond = desc->bond;
        }
        else
        {
            memset(&resultBond, 0, sizeof(NvBlastBond));
            return 1;
        }
        return 0;
    }

    std::vector<NvcVec3> chunksPoints1(meshACount * 3);
    std::vector<NvcVec3> chunksPoints2(meshBCount * 3);

    int32_t sp = 0;
    for (uint32_t i = 0; i < meshACount; ++i)
    {
        chunksPoints1[sp++] = meshA[i].a.p;
        chunksPoints1[sp++] = meshA[i].b.p;
        chunksPoints1[sp++] = meshA[i].c.p;
#ifdef DEBUG_OUTPUT
        meshBuffer.push_back(meshA[i].a.p);
        meshBuffer.push_back(meshA[i].b.p);
        meshBuffer.push_back(meshA[i].c.p);
#endif
    }
    sp = 0;
    for (uint32_t i = 0; i < meshBCount; ++i)
    {
        chunksPoints2[sp++] = meshB[i].a.p;
        chunksPoints2[sp++] = meshB[i].b.p;
        chunksPoints2[sp++] = meshB[i].c.p;
#ifdef DEBUG_OUTPUT
        meshBuffer.push_back(meshB[i].a.p);
        meshBuffer.push_back(meshB[i].b.p);
        meshBuffer.push_back(meshB[i].c.p);
#endif
    }

    CollisionHull* cHull[2];

    cHull[0] = mConvexMeshBuilder->buildCollisionGeometry(chunksPoints1.size(), chunksPoints1.data());
    cHull[1] = mConvexMeshBuilder->buildCollisionGeometry(chunksPoints2.size(), chunksPoints2.data());

    std::vector<NvVec3> hullPoints[2];
    hullPoints[0].resize(cHull[0]->pointsCount);
    hullPoints[1].resize(cHull[1]->pointsCount);


    NvBounds3 bb[2];
    bb[0].setEmpty();
    bb[1].setEmpty();

    for (uint32_t cv = 0; cv < 2; ++cv)
    {
        for (uint32_t i = 0; i < cHull[cv]->pointsCount; ++i)
        {
            hullPoints[cv][i] = toNvShared(cHull[cv]->points[i]);
            bb[cv].include(hullPoints[cv][i]);
        }
    }
    auto ret = createBondForcedInternal(hullPoints[0], hullPoints[1], *cHull[0], *cHull[1], bb[0], bb[1], resultBond,
                                        overlapping);

    mConvexMeshBuilder->releaseCollisionHull(cHull[0]);
    mConvexMeshBuilder->releaseCollisionHull(cHull[1]);

    return ret;
}

int32_t BlastBondGeneratorImpl::bondsFromPrefractured(uint32_t meshCount, const uint32_t* geometryCount,
                                                      const Triangle* geometry, const bool* chunkIsSupport,
                                                      NvBlastBondDesc*& resultBondDescs, BondGenerationConfig conf)
{
    int32_t ret_val = 0;
    switch (conf.bondMode)
    {
    case BondGenerationConfig::AVERAGE:
        ret_val = createFullBondListAveraged(meshCount, geometryCount, geometry, nullptr, chunkIsSupport, nullptr,
                                             resultBondDescs, conf);
        break;
    case BondGenerationConfig::EXACT:
        ret_val = createFullBondListExact(meshCount, geometryCount, geometry, chunkIsSupport, resultBondDescs, conf);
        break;
    }
    return ret_val;
}


int32_t BlastBondGeneratorImpl::bondsFromPrefractured(uint32_t meshCount, const uint32_t* convexHullOffset,
                                                      const CollisionHull** chunkHulls, const bool* chunkIsSupport,
                                                      const uint32_t* meshGroups, NvBlastBondDesc*& resultBondDescs,
                                                      float maxSeparation)
{
    BondGenerationConfig conf;
    conf.maxSeparation = maxSeparation;
    conf.bondMode      = BondGenerationConfig::AVERAGE;
    return createFullBondListAveraged(meshCount, convexHullOffset, nullptr, chunkHulls, chunkIsSupport, meshGroups,
                                      resultBondDescs, conf);
}

void BlastBondGeneratorImpl::release()
{
    delete this;
}

}  // namespace Blast
}  // namespace Nv
