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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


// This warning arises when using some stl containers with older versions of VC
// c:\program files (x86)\microsoft visual studio 12.0\vc\include\xtree(1826): warning C4702: unreachable code
#include "NvPreprocessor.h"
#if NV_VC && NV_VC < 14
#pragma warning(disable : 4702)
#endif
#include "NvBlastExtAuthoringTriangulator.h"
#include "NvBlastExtAuthoringMesh.h"
#include "NvBlastExtAuthoringTypes.h"
#include "NvPreprocessor.h"
#include "NvBlastExtAuthoringBooleanToolImpl.h"
#include <NvBlastAssert.h>
#include <NvBlastNvSharedHelpers.h>

#include <math.h>
#include <algorithm>
#include <list>
#include <queue>
#include <set>
#include <vector>

using nvidia::NvVec2;
using nvidia::NvVec3;

namespace Nv
{
namespace Blast
{

// used with ear clipping algorithm to deal with floating point precision artifacts for nearly co-linear points
#define MIN_ANGLE (0.0001f)

// helper for ear clipping algorithm
// holds the vertex indices for the previous and next vertex in the facet
// along with the scaled area of the triangle defined by the 3 vertices
struct AdjVertInfo
{
    uint32_t prev;
    uint32_t next;

    float scaledArea;
};

NV_FORCE_INLINE bool compareTwoFloats(float a, float b)
{
    return std::abs(b - a) <= FLT_EPSILON * std::abs(b + a);
}
NV_FORCE_INLINE bool compareTwoVertices(const NvVec3& a, const NvVec3& b)
{
    return compareTwoFloats(a.x, b.x) && compareTwoFloats(a.y, b.y) && compareTwoFloats(a.z, b.z);
}
NV_FORCE_INLINE bool compareTwoVertices(const NvVec2& a, const NvVec2& b)
{
    return compareTwoFloats(a.x, b.x) && compareTwoFloats(a.y, b.y);
}

NV_FORCE_INLINE float getRotation(const NvVec2& a, const NvVec2& b)
{
    return a.x * b.y - a.y * b.x;
}

NV_FORCE_INLINE bool pointInside(
    const NvVec2& ba, const NvVec2& cb, const NvVec2& ac,
    const NvVec2& a, const NvVec2& b, const NvVec2& c,
    const NvVec2& pnt
) {
    // Co-positional verts are not considered inside because that would break the exterior of the facet
    if (compareTwoVertices(a, pnt) || compareTwoVertices(b, pnt) || compareTwoVertices(c, pnt))
    {
        return false;
    }
    const float v1 = getRotation(ba, (pnt - a).getNormalized());
    const float v2 = getRotation(cb, (pnt - b).getNormalized());
    const float v3 = getRotation(ac, (pnt - c).getNormalized());

    // If the sign of all angles match, then the point is inside
    // A 0 angle is considered inside because otherwise verts would get dropped during triangulation
    return (v1 >= -MIN_ANGLE && v2 >= -MIN_ANGLE && v3 >= -MIN_ANGLE) || 
           (v1 <= MIN_ANGLE && v2 <= MIN_ANGLE && v3 <= MIN_ANGLE);
}

static void updatePotentialEar(
    uint32_t curr,
    const Vertex* vert,
    const ProjectionDirections& dir,
    const std::map<uint32_t, AdjVertInfo>& adjVertInfoMap,
    const std::list<uint32_t>& reflexVerts,
    std::list<uint32_t>& potentialEars
) {
    // remove from potential list if it exists already
    // it will be added back if it is still a valid potential ear
    const auto itr = std::find(potentialEars.begin(), potentialEars.end(), curr);
    if (itr != potentialEars.end())
    {
        potentialEars.erase(itr);
    }

    // doing it this way so the map can be passed as a const reference, but it should always be fully populated
    const auto mapItr = adjVertInfoMap.find(curr);
    if (mapItr == adjVertInfoMap.end())
    {
        NVBLAST_ASSERT_WITH_MESSAGE(false, "this should never happen");
        return;
    }

    // only convex verts need to be considered for potential ears
    const AdjVertInfo& adjVertInfo = mapItr->second;
    if (adjVertInfo.scaledArea <= 0.0f)
    {
        return;
    }

    // only need to check against reflex verts to see if they are inside potential ears
    // convex verts can't be inside potential ears
    if (reflexVerts.size())
    {
        const Vertex cV = vert[curr];
        const Vertex pV = vert[adjVertInfo.prev];
        const Vertex nV = vert[adjVertInfo.next];

        const NvVec2 cVp = getProjectedPoint(cV.p, dir);
        const NvVec2 pVp = getProjectedPoint(pV.p, dir);
        const NvVec2 nVp = getProjectedPoint(nV.p, dir);

        // if there are no other verts inside, then it is a potential ear
        const NvVec2 ba = (nVp - cVp).getNormalized();
        const NvVec2 cb = (pVp - nVp).getNormalized();
        const NvVec2 ac = (cVp - pVp).getNormalized();
        for (uint32_t vrt : reflexVerts)
        {
            // ignore reflex verts that are part of the tri being tested
            if (vrt == adjVertInfo.prev || vrt == adjVertInfo.next)
            {
                continue;
            }

            const NvVec2 pnt = getProjectedPoint(vert[vrt].p, dir);
            if (pointInside(ba, cb, ac, cVp, nVp, pVp, pnt))
            {
                return;
            }
        }
    }

    potentialEars.push_back(curr);
}

static void updateVertData(
    uint32_t curr,
    uint32_t prev,
    uint32_t next,
    const Vertex* vert,
    const ProjectionDirections& dir,
    std::map<uint32_t, AdjVertInfo>& adjVertInfoMap,
    std::list<uint32_t>& reflexVerts
) {
    // remove the index from the reflex list if there is already an entry for it
    // it will be added back if it is still a reflex vertex
    const auto reflexItr = std::find(reflexVerts.begin(), reflexVerts.end(), curr);
    if (reflexItr != reflexVerts.end())
    {
        reflexVerts.erase(reflexItr);
    }

    // if next == prev it isn't a valid triangle
    // this will happen when the facet has less than 3 verts in it
    // no need to add them as reflex verts at that point, the algorithm is finishing up the final pass
    float scaledArea = 0.0f;
    if (prev != next)
    {
        const Vertex cV = vert[curr];
        const Vertex pV = vert[prev];
        const Vertex nV = vert[next];

        const NvVec2 cVp = getProjectedPoint(cV.p, dir);
        const NvVec2 pVp = getProjectedPoint(pV.p, dir);
        const NvVec2 nVp = getProjectedPoint(nV.p, dir);

        const NvVec2 prevEdge = (cVp - pVp);
        const NvVec2 nextEdge = (nVp - cVp);

        // use normalized vectors to get a better calc for the angle between them
        float rot = getRotation(prevEdge.getNormalized(), nextEdge.getNormalized());
        if (dir & OPPOSITE_WINDING)
            rot = -rot;
        if (rot > MIN_ANGLE)
        {
            // this is a valid convex vertex, calculate 2 * area (used for sorting later)
            // actual area isn't needed because it is only used to compare with other ears, so relative numbers are fine
            scaledArea = getRotation(prevEdge, nextEdge);
            if (dir & OPPOSITE_WINDING)
                scaledArea = -scaledArea;
        }
        else
        {
            // the angle is roughly 180 or greater, consider it a reflex vertex
            reflexVerts.push_back(curr);
        }
    }

    // the scaled area will be used to sort potential ears later
    adjVertInfoMap[curr] = {prev, next, scaledArea};
}

void Triangulator::triangulatePolygonWithEarClipping(const std::vector<uint32_t>& inputPolygon, const Vertex* vert,
                                                     const ProjectionDirections& dir)
{
    uint32_t vCount = static_cast<uint32_t>(inputPolygon.size());
    if (vCount < 3)
    {
        return;
    }

    // High level of ear clipping algorithm:
    // 
    // - find potential ears (3 consecutive verts that form a triangle fully inside the facet with no other points from the facet inside or on an edge)
    // while (potential ears)
    //    - sort the potential ears by area
    //    - add tri formed by largest ear to output and remove vert from the tip of the ear from the facet
    //    - update potential ears for remaining 2 verts in the tri
    // 
    // This will ensure that no sliver triangles are created

    // start by building up vertex data and a list of reflex (interior angle >= 180) verts
    std::list<uint32_t> reflexVerts;
    std::list<uint32_t> potentialEars;
    std::map<uint32_t, AdjVertInfo> adjVertInfoMap;
    for (uint32_t curr = 0; curr < vCount; curr++)
    {
        const uint32_t prev = (curr == 0) ? vCount - 1 : curr - 1;
        const uint32_t next = (curr == vCount - 1) ? 0 : curr + 1;

        const uint32_t currIdx = inputPolygon[curr];
        const uint32_t prevIdx = inputPolygon[prev];
        const uint32_t nextIdx = inputPolygon[next];

        updateVertData(currIdx, prevIdx, nextIdx, vert, dir, adjVertInfoMap, reflexVerts);
    }

    // build the list of potential ears defined by convex verts by checking any reflex vert is inside
    for (auto pair : adjVertInfoMap)
    {
        // if a vert is not a reflex, it must be convex and should be considered as an ear
        const uint32_t currIdx = pair.first;
        if (std::find(reflexVerts.begin(), reflexVerts.end(), currIdx) == reflexVerts.end())
        {
            updatePotentialEar(currIdx, vert, dir, adjVertInfoMap, reflexVerts, potentialEars);
        }
    }

    // descending sort by scaled area
    auto compArea = [&adjVertInfoMap](const uint32_t& a, const uint32_t& b) -> bool
    {
        return (adjVertInfoMap[a].scaledArea > adjVertInfoMap[b].scaledArea);
    };

    while (potentialEars.size())
    {
        // sort the potential ear list based on the area of the triangles they form
        potentialEars.sort(compArea);

        // add the largest triangle to the output
        const uint32_t curr = potentialEars.front();
        const AdjVertInfo& adjVertInfo = adjVertInfoMap[curr];
        mBaseMeshTriangles.push_back(TriangleIndexed(curr, adjVertInfo.prev, adjVertInfo.next));

        // remove the ear tip from the potential ear list
        potentialEars.pop_front();

        // update data for the other 2 verts involved
        const uint32_t prevPrev = adjVertInfoMap[adjVertInfo.prev].prev;
        const uint32_t nextNext = adjVertInfoMap[adjVertInfo.next].next;
        // vert data must be updated first for both
        updateVertData(adjVertInfo.prev, prevPrev, adjVertInfo.next, vert, dir, adjVertInfoMap, reflexVerts);
        updateVertData(adjVertInfo.next, adjVertInfo.prev, nextNext, vert, dir, adjVertInfoMap, reflexVerts);
        // then potential ear list
        updatePotentialEar(adjVertInfo.prev, vert, dir, adjVertInfoMap, reflexVerts, potentialEars);
        updatePotentialEar(adjVertInfo.next, vert, dir, adjVertInfoMap, reflexVerts, potentialEars);
    }
}


struct LoopInfo
{
    LoopInfo()
    {
        used = false;
    }
    NvVec3 normal;
    float area;
    int32_t index;
    bool used;
    bool operator<(const LoopInfo& b) const
    {
        return area < b.area;
    }
};

int32_t unitePolygons(std::vector<uint32_t>& externalLoop, std::vector<uint32_t>& internalLoop, Vertex* vrx,
                      const ProjectionDirections& dir)
{
    if (externalLoop.size() < 3 || internalLoop.size() < 3)
        return 1;
    /**
        Find point with maximum x-coordinate
    */
    float x_max    = -MAXIMUM_EXTENT;
    int32_t mIndex = -1;
    for (uint32_t i = 0; i < internalLoop.size(); ++i)
    {
        float nx = getProjectedPoint(vrx[internalLoop[i]].p, dir).x;
        if (nx > x_max)
        {
            mIndex = i;
            x_max  = nx;
        }
    }
    if (mIndex == -1)
    {
        return 1;
    }

    /**
        Search for base point on external loop
    */
    float minX        = MAXIMUM_EXTENT;
    int32_t vrtIndex  = -1;
    bool isFromBuffer = 0;
    NvVec2 holePoint  = getProjectedPoint(vrx[internalLoop[mIndex]].p, dir);
    NvVec2 computedPoint;
    for (uint32_t i = 0; i < externalLoop.size(); ++i)
    {
        int32_t nx  = (i + 1) % externalLoop.size();
        NvVec2 pnt1 = getProjectedPoint(vrx[externalLoop[i]].p, dir);
        NvVec2 pnt2 = getProjectedPoint(vrx[externalLoop[nx]].p, dir);
        if (pnt1.x < x_max && pnt2.x < x_max)
        {
            continue;
        }
        NvVec2 vc = pnt2 - pnt1;
        if (vc.y == 0 && pnt1.y == holePoint.y)
        {
            if (pnt1.x < minX && pnt1.x < pnt2.x && pnt1.x > x_max)
            {
                minX         = pnt1.x;
                vrtIndex     = i;
                isFromBuffer = true;
            }
            if (pnt2.x < minX && pnt2.x < pnt1.x && pnt2.x > x_max)
            {
                minX         = pnt2.x;
                vrtIndex     = nx;
                isFromBuffer = true;
            }
        }
        else
        {
            float t = (holePoint.y - pnt1.y) / vc.y;
            if (t <= 1 && t >= 0)
            {
                NvVec2 tempPoint = vc * t + pnt1;
                if (tempPoint.x < minX && tempPoint.x > x_max)
                {
                    minX          = tempPoint.x;
                    vrtIndex      = i;
                    isFromBuffer  = false;
                    computedPoint = tempPoint;
                }
            }
        }
    }
    if (vrtIndex == -1)
    {
        //  std::cout << "Triangulation: base vertex for inner loop is not found..." << std::endl;
        return 1;
    }
    int32_t bridgePoint = -1;
    float bestAngle     = 100;
    if (!isFromBuffer)
    {
        NvVec2 ex1 = getProjectedPoint(vrx[externalLoop[vrtIndex]].p, dir);
        NvVec2 ex2 = getProjectedPoint(vrx[externalLoop[(vrtIndex + 1) % externalLoop.size()]].p, dir);

        if (ex1.x > ex2.x)
        {
            vrtIndex = (vrtIndex + 1) % externalLoop.size();
            ex1      = ex2;
        }
        /* Check if some point is inside triangle */
        bool notFound = true;
        const NvVec2 ba = (ex1 - holePoint).getNormalized();
        const NvVec2 cb = (computedPoint - ex1).getNormalized();
        const NvVec2 ac = (holePoint - computedPoint).getNormalized();
        for (int32_t i = 0; i < (int32_t)externalLoop.size(); ++i)
        {
            const NvVec2 tempPoint = getProjectedPoint(vrx[externalLoop[i]].p, dir);
            if (pointInside(ba, cb, ac, holePoint, ex1, computedPoint, tempPoint))
            {
                notFound   = false;
                const NvVec2 cVp = getProjectedPoint(vrx[externalLoop[i]].p, dir);
                const NvVec2 pVp =
                    getProjectedPoint(vrx[externalLoop[(i - 1 + externalLoop.size()) % externalLoop.size()]].p, dir);
                const NvVec2 nVp = getProjectedPoint(vrx[externalLoop[(i + 1) % externalLoop.size()]].p, dir);
                float rt = getRotation((cVp - pVp).getNormalized(), (nVp - pVp).getNormalized());
                if (dir & OPPOSITE_WINDING)
                    rt = -rt;
                if (rt < MIN_ANGLE)
                    continue;
                const float tempAngle = NvVec2(1, 0).dot((tempPoint - holePoint).getNormalized());
                if (bestAngle < tempAngle)
                {
                    bestAngle   = tempAngle;
                    bridgePoint = i;
                }
            }
        }
        if (notFound)
        {
            bridgePoint = vrtIndex;
        }
        if (bridgePoint == -1)
        {
            //  std::cout << "Triangulation: bridge vertex for inner loop is not found..." << std::endl;
            return 1;
        }
    }
    else
    {
        bridgePoint = vrtIndex;
    }
    std::vector<uint32_t> temporal;

    for (int32_t i = 0; i <= bridgePoint; ++i)
    {
        temporal.push_back(externalLoop[i]);
    }
    temporal.push_back(internalLoop[mIndex]);
    for (int32_t i = (mIndex + 1) % internalLoop.size(); i != mIndex; i = (i + 1) % internalLoop.size())
    {
        temporal.push_back(internalLoop[i]);
    }
    temporal.push_back(internalLoop[mIndex]);
    for (uint32_t i = bridgePoint; i < externalLoop.size(); ++i)
    {
        temporal.push_back(externalLoop[i]);
    }
    externalLoop = temporal;
    return 0;
}

void Triangulator::buildPolygonAndTriangulate(std::vector<Edge>& edges, Vertex* vertices, int32_t userData,
                                              int32_t materialId, int32_t smoothingGroup)
{
    std::vector<std::vector<uint32_t> > serializedLoops;

    std::set<int> visitedVertices;
    std::vector<int> used(edges.size(), 0);
    uint32_t collected = 0;

    std::vector<int> edgesIds;
    /**
    Add first edge to polygon
    */
    edgesIds.push_back(0);
    visitedVertices.insert(edges[0].s);
    visitedVertices.insert(edges[0].e);
    used[0]              = true;
    collected            = 1;
    uint32_t lastEdge    = 0;
    bool successfullPass = false;
    for (; collected < edges.size();)
    {
        successfullPass = false;
        for (uint32_t p = 0; p < edges.size(); ++p)
        {
            if (used[p] == 0 && edges[p].s == edges[lastEdge].e)
            {
                successfullPass = true;
                collected++;
                used[p] = true;
                edgesIds.push_back(p);
                lastEdge = p;
                if (visitedVertices.find(edges[p].e) != visitedVertices.end())  // if we formed loop, detach it and
                                                                                // triangulate
                {
                    serializedLoops.push_back(std::vector<uint32_t>());
                    std::vector<uint32_t>& serializedPositions = serializedLoops.back();
                    while (edgesIds.size() > 0)
                    {
                        serializedPositions.push_back(edges[edgesIds.back()].s);
                        visitedVertices.erase(edges[edgesIds.back()].s);
                        if (edges[edgesIds.back()].s == edges[p].e)
                        {
                            edgesIds.pop_back();
                            break;
                        }
                        edgesIds.pop_back();
                    }
                    if (edgesIds.size() > 0)
                    {
                        lastEdge = edgesIds.back();
                    }
                    else
                    {
                        for (uint32_t t = 0; t < edges.size(); ++t)
                        {
                            if (used[t] == 0)
                            {
                                edgesIds.push_back(t);
                                visitedVertices.insert(edges[t].s);
                                visitedVertices.insert(edges[t].e);
                                used[t] = true;
                                collected++;
                                lastEdge = t;
                                break;
                            }
                        }
                    }
                }
                else
                {
                    visitedVertices.insert(edges[p].e);
                }
            }
        }
        if (!successfullPass)
        {
            break;
        }
    }

    std::vector<LoopInfo> loopsInfo(serializedLoops.size());
    // Compute normal to whole polygon, and areas of loops
    NvVec3 wholeFacetNormal(0, 0, 0);
    for (uint32_t loop = 0; loop < serializedLoops.size(); ++loop)
    {
        NvVec3 loopNormal(0, 0, 0);
        const std::vector<uint32_t>& pos = serializedLoops[loop];
        for (uint32_t vrt = 1; vrt + 1 < serializedLoops[loop].size(); ++vrt)
        {
            loopNormal += toNvShared(vertices[pos[vrt]].p - vertices[pos[0]].p)
                              .cross(toNvShared(vertices[pos[vrt + 1]].p - vertices[pos[0]].p));
        }
        loopsInfo[loop].area   = loopNormal.magnitude();
        loopsInfo[loop].normal = loopNormal;
        loopsInfo[loop].index  = loop;
        wholeFacetNormal += loopNormal;
    }

    // Change areas signs according to winding direction
    for (uint32_t loop = 0; loop < serializedLoops.size(); ++loop)
    {
        if (wholeFacetNormal.dot(loopsInfo[loop].normal) < 0)
        {
            loopsInfo[loop].area = -loopsInfo[loop].area;
        }
    }
    const ProjectionDirections dir = getProjectionDirection(wholeFacetNormal);
    std::sort(loopsInfo.begin(), loopsInfo.end());

    std::vector<NvVec3> tempPositions;
    int32_t oldSize = static_cast<int32_t>(mBaseMeshTriangles.size());
    for (uint32_t extPoly = 0; extPoly < loopsInfo.size(); ++extPoly)
    {
        if (loopsInfo[extPoly].area < 0)
        {
            continue;  // Polygon with negative area is hole
        }
        int32_t baseLoop = loopsInfo[extPoly].index;
        for (uint32_t intPoly = 0; intPoly < loopsInfo.size(); ++intPoly)
        {
            if (loopsInfo[intPoly].area > 0 || loopsInfo[intPoly].used ||
                std::abs(loopsInfo[intPoly].area) > loopsInfo[extPoly].area)
            {
                continue;
            }
            int32_t holeLoop = loopsInfo[intPoly].index;

            if (!unitePolygons(serializedLoops[baseLoop], serializedLoops[holeLoop], vertices, dir))
            {
                loopsInfo[intPoly].used = true;
            };
        }
        triangulatePolygonWithEarClipping(serializedLoops[baseLoop], vertices, dir);
    }
    for (uint32_t i = oldSize; i < mBaseMeshTriangles.size(); ++i)
    {
        mBaseMeshTriangles[i].userData       = userData;
        mBaseMeshTriangles[i].materialId     = materialId;
        mBaseMeshTriangles[i].smoothingGroup = smoothingGroup;
    }
}

NV_FORCE_INLINE int32_t Triangulator::addVerticeIfNotExist(const Vertex& p)
{
    auto it = mVertMap.find(p);
    if (it == mVertMap.end())
    {
        mVertMap[p] = static_cast<int32_t>(mVertices.size());
        mVertices.push_back(p);
        return static_cast<int32_t>(mVertices.size()) - 1;
    }
    else
    {
        return it->second;
    }
}

NV_FORCE_INLINE void Triangulator::addEdgeIfValid(EdgeWithParent& ed)
{
    if (ed.s == ed.e)
        return;
    EdgeWithParent opposite(ed.e, ed.s, ed.parent);
    auto it = mEdgeMap.find(opposite);
    if (it == mEdgeMap.end())
    {
        mEdgeMap[ed] = static_cast<int32_t>(mBaseMeshEdges.size());
        mBaseMeshEdges.push_back(ed);
    }
    else
    {
        if (mBaseMeshEdges[it->second].s == kNotValidVertexIndex)
        {
            mBaseMeshEdges[it->second].s = ed.s;
            mBaseMeshEdges[it->second].e = ed.e;
        }
        else
        {
            mBaseMeshEdges[it->second].s = kNotValidVertexIndex;
        }
    }
}


void Triangulator::prepare(const Mesh* mesh)
{
    const Edge* ed   = mesh->getEdges();
    const Vertex* vr = mesh->getVertices();
    mBaseMapping.resize(mesh->getVerticesCount());
    for (uint32_t i = 0; i < mesh->getFacetCount(); ++i)
    {
        const Facet* fc = mesh->getFacet(i);
        for (uint32_t j = fc->firstEdgeNumber; j < fc->firstEdgeNumber + fc->edgesCount; ++j)
        {
            int32_t a             = addVerticeIfNotExist(vr[ed[j].s]);
            int32_t b             = addVerticeIfNotExist(vr[ed[j].e]);
            mBaseMapping[ed[j].s] = a;
            mBaseMapping[ed[j].e] = b;
            EdgeWithParent e(a, b, i);
            addEdgeIfValid(e);
        }
    }
    std::vector<EdgeWithParent> temp;
    temp.reserve(mBaseMeshEdges.size());
    for (uint32_t i = 0; i < mBaseMeshEdges.size(); ++i)
    {
        if (mBaseMeshEdges[i].s != kNotValidVertexIndex)
        {
            temp.push_back(mBaseMeshEdges[i]);
        }
    }
    mBaseMeshEdges = temp;
}

void Triangulator::reset()
{
    mVertices.clear();
    mBaseMeshEdges.clear();
    mVertMap.clear();
    mEdgeMap.clear();
    mBaseMeshTriangles.clear();
    mBaseMeshResultTriangles.clear();
}

void Triangulator::triangulate(const Mesh* mesh)
{
    reset();
    if (mesh == nullptr || !mesh->isValid())
    {
        return;
    }
    prepare(mesh);
    if (mBaseMeshEdges.empty())
    {
        return;
    }
    std::vector<Edge> temp;
    uint32_t fP = mBaseMeshEdges[0].parent;
    for (uint32_t i = 0; i < mBaseMeshEdges.size(); ++i)
    {
        if (fP != mBaseMeshEdges[i].parent)
        {
            if (temp.empty() == false)
            {
                buildPolygonAndTriangulate(temp, mVertices.data(), mesh->getFacet(fP)->userData,
                                           mesh->getFacet(fP)->materialId, mesh->getFacet(fP)->smoothingGroup);
            }
            temp.clear();
            fP = mBaseMeshEdges[i].parent;
        }
        temp.push_back({ mBaseMeshEdges[i].s, mBaseMeshEdges[i].e });
    }
    buildPolygonAndTriangulate(temp, mVertices.data(), mesh->getFacet(fP)->userData, mesh->getFacet(fP)->materialId,
                               mesh->getFacet(fP)->smoothingGroup);

    /* Build final triangles */
    mBaseMeshResultTriangles.clear();
    for (uint32_t i = 0; i < mBaseMeshTriangles.size(); ++i)
    {
        if (mBaseMeshTriangles[i].ea == kNotValidVertexIndex)
        {
            continue;
        }
        mBaseMeshResultTriangles.push_back({ mVertices[mBaseMeshTriangles[i].ea], mVertices[mBaseMeshTriangles[i].eb],
                                             mVertices[mBaseMeshTriangles[i].ec], mBaseMeshTriangles[i].userData,
                                             mBaseMeshTriangles[i].materialId, mBaseMeshTriangles[i].smoothingGroup });
    }
    mBaseMeshUVFittedTriangles = mBaseMeshResultTriangles;  // Uvs will be fitted later, in FractureTool.
    computePositionedMapping();
}

void Triangulator::computePositionedMapping()
{
    std::map<NvcVec3, int32_t, VrtPositionComparator> mPosMap;
    mPositionMappedVrt.clear();
    mPositionMappedVrt.resize(mVertices.size());

    for (uint32_t i = 0; i < mVertices.size(); ++i)
    {
        auto it = mPosMap.find(mVertices[i].p);

        if (it == mPosMap.end())
        {
            mPosMap[mVertices[i].p] = i;
            mPositionMappedVrt[i]   = i;
        }
        else
        {
            mPositionMappedVrt[i] = it->second;
        }
    }
}

}  // namespace Blast
}  // namespace Nv