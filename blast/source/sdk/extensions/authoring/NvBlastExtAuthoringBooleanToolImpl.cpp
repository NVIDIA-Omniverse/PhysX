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


#include "NvBlastGlobals.h"
#include "NvBlastExtAuthoringBooleanToolImpl.h"
#include "NvBlastExtAuthoringBooleanTool.h"
#include "NvBlastExtAuthoringMeshImpl.h"
#include "NvBlastExtAuthoringAcceleratorImpl.h"
#include <NvBlastNvSharedHelpers.h>

#include <math.h>
#include <set>
#include <algorithm>

using nvidia::NvBounds3;

namespace Nv
{
namespace Blast
{

/* Linear interpolation of vectors */

NV_FORCE_INLINE void vec3Lerp(const NvcVec3& a, const NvcVec3& b, NvcVec3& out, float t)
{
    out.x = (b.x - a.x) * t + a.x;
    out.y = (b.y - a.y) * t + a.y;
    out.z = (b.z - a.z) * t + a.z;
}

NV_FORCE_INLINE void vec2Lerp(const NvcVec2& a, const NvcVec2& b, NvcVec2& out, float t)
{
    out.x = (b.x - a.x) * t + a.x;
    out.y = (b.y - a.y) * t + a.y;
}

NV_FORCE_INLINE int32_t BooleanEvaluator::addIfNotExist(const Vertex& p)
{
    mVerticesAggregate.push_back(p);
    return static_cast<int32_t>(mVerticesAggregate.size()) - 1;
}

NV_FORCE_INLINE void BooleanEvaluator::addEdgeIfValid(const EdgeWithParent& ed)
{
    mEdgeAggregate.push_back(ed);
}

/**
Vertex level shadowing functions
*/
NV_FORCE_INLINE int32_t vertexShadowing(const NvcVec3& a, const NvcVec3& b)
{
    return (b.x >= a.x) ? 1 : 0;
}
/**
Vertex-edge status functions
*/
NV_FORCE_INLINE int32_t veStatus01(const NvcVec3& sEdge, const NvcVec3& eEdge, const NvcVec3& p)
{
    return vertexShadowing(p, eEdge) - vertexShadowing(p, sEdge);
}

NV_FORCE_INLINE int32_t veStatus10(const NvcVec3& sEdge, const NvcVec3& eEdge, const NvcVec3& p)
{
    return -vertexShadowing(eEdge, p) + vertexShadowing(sEdge, p);
}

bool shouldSwap(const NvcVec3& a, const NvcVec3& b)
{
    if (a.x < b.x) return false;
    if (a.x > b.x) return true;

    if (a.y < b.y) return false;
    if (a.y > b.y) return true;

    if (a.z < b.z) return false;
    if (a.z > b.z) return true;
    return false;
}


/**
    Vertex-edge shadowing functions
*/
int32_t shadowing01(Vertex sEdge, Vertex eEdge, const NvcVec3& p, Vertex& onEdgePoint, bool& hasOnEdge)
{

    int32_t winding = veStatus01(sEdge.p, eEdge.p, p);

    if (sEdge.p.x > eEdge.p.x)
    {
        std::swap(sEdge, eEdge);
    }

    if (winding != 0)
    {
        float t = (p.x - sEdge.p.x) / (eEdge.p.x - sEdge.p.x);
        if (t >= 1)
        {
            onEdgePoint = eEdge;
        }
        else if (t <= 0)
        {
            onEdgePoint = sEdge;
        }
        else
        {
            vec3Lerp(sEdge.p, eEdge.p, onEdgePoint.p, t);
            vec3Lerp(sEdge.n, eEdge.n, onEdgePoint.n, t);
            vec2Lerp(sEdge.uv[0], eEdge.uv[0], onEdgePoint.uv[0], t);
        }
        hasOnEdge = true;
        if (onEdgePoint.p.y >= p.y)
        {
            return winding;
        }
    }
    else
    {
        hasOnEdge = false;
    }
    return 0;
}
int32_t shadowing10(Vertex sEdge, Vertex eEdge, const NvcVec3& p, Vertex& onEdgePoint, bool& hasOnEdge)
{
    int32_t winding = veStatus10(sEdge.p, eEdge.p, p);

    if (sEdge.p.x > eEdge.p.x)
    {
        std::swap(sEdge, eEdge);
    }

    if (winding != 0)
    {
        float t = (p.x - sEdge.p.x) / (eEdge.p.x - sEdge.p.x);
        if (t >= 1)
        {
            onEdgePoint = eEdge;
        }
        else if (t <= 0)
        {
            onEdgePoint = sEdge;
        }
        else
        {
            vec3Lerp(sEdge.p, eEdge.p, onEdgePoint.p, t);
            vec3Lerp(sEdge.n, eEdge.n, onEdgePoint.n, t);
            vec2Lerp(sEdge.uv[0], eEdge.uv[0], onEdgePoint.uv[0], t);
        }
        hasOnEdge = true;
        if (onEdgePoint.p.y < p.y)
        {
            return winding;
        }
    }
    else
    {
        hasOnEdge = false;
    }
    return 0;
}

int32_t shadowing01(NvcVec3 sEdge, NvcVec3 eEdge, const NvcVec3& p)
{
    int32_t winding = veStatus01(sEdge, eEdge, p);

    if (winding != 0)
    {
        if (sEdge.x > eEdge.x)
        {
            std::swap(sEdge, eEdge);
        }
        float t = ((p.x - sEdge.x) / (eEdge.x - sEdge.x));
        NvcVec3 onEdgePoint;
        if (t >= 1)
            onEdgePoint = eEdge;
        else if (t <= 0)
            onEdgePoint = sEdge;
        else
            vec3Lerp(sEdge, eEdge, onEdgePoint, t);
        if (onEdgePoint.y >= p.y)
        {
            return winding;
        }
    }
    return 0;
}

int32_t shadowing10(NvcVec3 sEdge, NvcVec3 eEdge, const NvcVec3& p)
{
    int32_t winding = veStatus10(sEdge, eEdge, p);
    if (winding != 0)
    {
        if (sEdge.x > eEdge.x)
        {
            std::swap(sEdge, eEdge);
        }

        float t = ((p.x - sEdge.x) / (eEdge.x - sEdge.x));
        NvcVec3 onEdgePoint;
        if (t >= 1)
            onEdgePoint = eEdge;
        else if (t <= 0)
            onEdgePoint = sEdge;
        else
            vec3Lerp(sEdge, eEdge, onEdgePoint, t);
        if (onEdgePoint.y < p.y)
        {
            return winding;
        }
    }
    return 0;
}

/**
Vertex-facet shadowing functions
*/

int32_t vfStatus02(const NvcVec3& p, const Vertex* points, const Edge* edges, int32_t edgesCount, Vertex* out)
{
    int32_t val = 0;
    Vertex pnt;
    bool hasOnEdge = false;
    out[0].p.y = -MAXIMUM_EXTENT;
    out[1].p.y = MAXIMUM_EXTENT;
    for (int32_t i = 0; i < edgesCount; ++i)
    {
        val -= shadowing01(points[edges->s], points[edges->e], p, pnt, hasOnEdge);
        if (hasOnEdge != 0)
        {
            if (p.y > pnt.p.y && pnt.p.y > out[0].p.y)
            {
                out[0] = pnt;
            }
            if (p.y <= pnt.p.y && pnt.p.y < out[1].p.y)
            {
                out[1] = pnt;
            }
        }
        ++edges;
    }
    return val;
}


int32_t shadowing02(const NvcVec3& p, const Vertex* points, const Edge* edges, int edgesCount, bool& hasOnFacetPoint, Vertex& onFacetPoint)
{
    Vertex outp[2];
    int32_t stat = vfStatus02(p, points, edges, edgesCount, outp);
    float z = 0;
    hasOnFacetPoint = false;
    if (stat != 0)
    {
        Vertex& p1 = outp[0];
        Vertex& p2 = outp[1];
        NvcVec3 vc = p2.p - p1.p;
        float t = 0;
        t = (std::abs(vc.x) > std::abs(vc.y)) ? (p.x - p1.p.x) / vc.x : (p.y - p1.p.y) / vc.y;
        t = nvidia::NvClamp(t, 0.0f, 1.0f);

        z = t * vc.z + p1.p.z;
        hasOnFacetPoint = true;
        onFacetPoint.p.x = p.x;
        onFacetPoint.p.y = p.y;
        onFacetPoint.p.z = z;
        vec2Lerp(p1.uv[0], p2.uv[0], onFacetPoint.uv[0], t);
        vec3Lerp(p1.n, p2.n, onFacetPoint.n, t);

        if (z >= p.z)
        {
            return stat;
        }
    }
    return 0;
}

int32_t vfStatus20(const NvcVec3& p, const Vertex* points, const Edge* edges, int32_t edgesCount, Vertex* out)
{
    int32_t val = 0;
    Vertex pnt;
    bool hasOnEdge = false;
    out[0].p.y = -MAXIMUM_EXTENT;
    out[1].p.y = MAXIMUM_EXTENT;

    for (int32_t i = 0; i < edgesCount; ++i)
    {
        val += shadowing10(points[edges->s], points[edges->e], p, pnt, hasOnEdge);
        if (hasOnEdge != 0)
        {
            if (p.y > pnt.p.y && pnt.p.y > out[0].p.y)
            {
                out[0] = pnt;
            }
            if (p.y <= pnt.p.y && pnt.p.y < out[1].p.y)
            {
                out[1] = pnt;
            }
        }
        ++edges;
    }
    return val;
}

int32_t shadowing20(const NvcVec3& p, const Vertex* points, const Edge* edges, int edgesCount, bool& hasOnFacetPoint, Vertex& onFacetPoint)
{
    Vertex outp[2];
    int32_t stat = vfStatus20(p, points, edges, edgesCount, outp);
    hasOnFacetPoint = false;
    if (stat != 0)
    {
        Vertex& p1 = outp[0];
        Vertex& p2 = outp[1];
        NvcVec3 vc = p2.p - p1.p;
        float t = 0;
        t = (std::abs(vc.x) > std::abs(vc.y)) ? (p.x - p1.p.x) / vc.x : (p.y - p1.p.y) / vc.y;      
        t = nvidia::NvClamp(t, 0.0f, 1.0f);  
        
        hasOnFacetPoint = true;
        onFacetPoint.p.x = p.x;
        onFacetPoint.p.y = p.y;

        onFacetPoint.p.z = t * vc.z + p1.p.z;
        
        vec2Lerp(p1.uv[0], p2.uv[0], onFacetPoint.uv[0], t);
        vec3Lerp(p1.n, p2.n, onFacetPoint.n, t);

        if (onFacetPoint.p.z < p.z)
        {
            return stat;
        }
    }
    return 0;
}


NV_FORCE_INLINE int32_t edgesCrossCheck(const NvcVec3& eAs, const NvcVec3& eAe, const NvcVec3& eBs, const NvcVec3& eBe)
{
    return shadowing01(eBs, eBe, eAe) - shadowing01(eBs, eBe, eAs) + shadowing10(eAs, eAe, eBe) - shadowing10(eAs, eAe, eBs);
}



int32_t edgesIntersection(const Vertex& eAs, const Vertex& eAe, const Vertex& eBs, const Vertex& eBe, Vertex& intersectionA, Vertex& intersectionB, bool& hasPoints)
{
    int32_t status = edgesCrossCheck(eAs.p, eAe.p, eBs.p, eBe.p);
    hasPoints = false;
    if (status == 0)
    {
        return 0;
    }

    Vertex tempPoint;
    Vertex bShadowingPair[2];
    Vertex aShadowingPair[2];
    bool hasOnEdge = false;
    bool aShadowing = false;
    bool bShadowing = false;    

    /**
        Search for two pairs where parts of A shadows B, and where B shadows A.
        Needed for search intersection point.
    */

    for (auto p : { &eBs, &eBe })
    {
        int32_t shadowingType = shadowing10(eAs, eAe, p->p, tempPoint, hasOnEdge);
        if (shadowingType == 0 && !aShadowing && hasOnEdge)
        {
            aShadowing = true;
            aShadowingPair[0] = *p;
            aShadowingPair[1] = tempPoint;
        }
        else
        {
            if ((shadowingType == 1 || shadowingType == -1) && !bShadowing)
            {
                bShadowing = true;
                bShadowingPair[0] = *p;
                bShadowingPair[1] = tempPoint;
            }
        }
    }
    if (!aShadowing || !bShadowing)
    {
        for (auto p : { &eAs, &eAe })
        {
            int32_t shadowingType = shadowing01(eBs, eBe, p->p, tempPoint, hasOnEdge);

            if (shadowingType == 0 && !aShadowing && hasOnEdge)
            {
                aShadowing = true;
                aShadowingPair[1] = *p;
                aShadowingPair[0] = tempPoint;
            }
            else
            {
                if ((shadowingType == 1 || shadowingType == -1) && !bShadowing)
                {
                    bShadowing = true;
                    bShadowingPair[1] = *p;
                    bShadowingPair[0] = tempPoint;
                }
            }
        }
    }

    float deltaPlus = bShadowingPair[0].p.y - bShadowingPair[1].p.y;
    float deltaMinus = aShadowingPair[0].p.y - aShadowingPair[1].p.y;
    float div = 0;
    if (deltaPlus > 0)
        div = deltaPlus / (deltaPlus - deltaMinus);
    else
        div = 0;

    intersectionA.p = bShadowingPair[1].p - div * (bShadowingPair[1].p - aShadowingPair[1].p);
    intersectionA.n = bShadowingPair[1].n - div * (bShadowingPair[1].n - aShadowingPair[1].n);
    intersectionA.uv[0] = bShadowingPair[1].uv[0] - (bShadowingPair[1].uv[0] - aShadowingPair[1].uv[0]) * div;
    intersectionB.p = intersectionA.p;
    intersectionB.p.z = bShadowingPair[0].p.z - div * (bShadowingPair[0].p.z - aShadowingPair[0].p.z);
    intersectionB.n = bShadowingPair[0].n - div * (bShadowingPair[0].n - aShadowingPair[0].n);
    intersectionB.uv[0] = bShadowingPair[0].uv[0] - (bShadowingPair[0].uv[0] - aShadowingPair[0].uv[0]) * div;

    hasPoints = true;
    return status;
}

NV_FORCE_INLINE int32_t edgeEdgeShadowing(const Vertex& eAs, const Vertex& eAe, const Vertex& eBs, const Vertex& eBe, Vertex& intersectionA, Vertex& intersectionB, bool& hasPoints)
{
    int32_t status = edgesIntersection(eAs, eAe, eBs, eBe, intersectionA, intersectionB, hasPoints);
    if (intersectionB.p.z >= intersectionA.p.z)
    {
        return status;
    }
    return 0;
}

int32_t edgeFacetIntersection12(const Vertex& edSt, const Vertex& edEnd, const Vertex* points, const Edge* edges, int edgesCount, Vertex& intersectionA, Vertex& intersectionB)
{
    int32_t status = 0;
    Vertex p1, p2;
    Vertex bShadowingPair[2];
    Vertex aShadowingPair[2];
    bool hasPoint = false;
    bool aShadowing = false;
    bool bShadowing = false;
    int32_t mlt = -1;
    int32_t shadowingType;
    for (auto p : { &edEnd, &edSt })
    {
        shadowingType = shadowing02(p->p, points, edges, edgesCount, hasPoint, p1);
        status += mlt * shadowingType;
        if (shadowingType == 0 && !aShadowing && hasPoint)
        {
            aShadowing = true;
            aShadowingPair[0] = p1;
            aShadowingPair[1] = *p;
        }
        else if ((shadowingType == 1 || shadowingType == -1) && !bShadowing)
        {
            bShadowing = true;
            bShadowingPair[0] = p1;
            bShadowingPair[1] = *p;
        }
        mlt = 1;
    }

    for (int32_t ed = 0; ed < edgesCount; ++ed)
    {
        if (shouldSwap(points[edges[ed].s].p, points[edges[ed].e].p))
        {
            shadowingType = -edgeEdgeShadowing(edSt, edEnd, points[edges[ed].e], points[edges[ed].s], p1, p2, hasPoint);
        }
        else
        {
            shadowingType = edgeEdgeShadowing(edSt, edEnd, points[edges[ed].s], points[edges[ed].e], p1, p2, hasPoint);
        }
        status -= shadowingType;
        if (shadowingType == 0 && !aShadowing && hasPoint)
        {
            aShadowing = true;
            aShadowingPair[0] = p2;
            aShadowingPair[1] = p1;
        }
        else if ((shadowingType == 1 || shadowingType == -1) && !bShadowing)
        {
            bShadowing = true;
            bShadowingPair[0] = p2;
            bShadowingPair[1] = p1;
        }
    }
    if (!status || !bShadowing || !aShadowing)
    {
        return 0;
    }

    float deltaPlus = bShadowingPair[0].p.z - bShadowingPair[1].p.z;
    float div = 0;
    if (deltaPlus != 0)
    {
        float deltaMinus = aShadowingPair[0].p.z - aShadowingPair[1].p.z;
        div = deltaPlus / (deltaPlus - deltaMinus);
    }
    intersectionA.p = bShadowingPair[1].p - div * (bShadowingPair[1].p - aShadowingPair[1].p);
    intersectionA.n = bShadowingPair[1].n - div * (bShadowingPair[1].n - aShadowingPair[1].n);
    intersectionA.uv[0] = bShadowingPair[1].uv[0] - (bShadowingPair[1].uv[0] - aShadowingPair[1].uv[0]) * div;

    intersectionB.p = intersectionA.p;
    intersectionB.n = bShadowingPair[0].n - div * (bShadowingPair[0].n - aShadowingPair[0].n);
    intersectionB.uv[0] = bShadowingPair[0].uv[0] - (bShadowingPair[0].uv[0] - aShadowingPair[0].uv[0]) * div;


    return status;
}


int32_t edgeFacetIntersection21(const Vertex& edSt, const Vertex& edEnd, const Vertex* points, const Edge* edges, int edgesCount, Vertex& intersectionA, Vertex& intersectionB)
{
    int32_t status = 0;
    Vertex p1, p2;

    Vertex bShadowingPair[2];
    Vertex aShadowingPair[2];
    bool hasPoint = false;
    bool aShadowing = false;
    bool bShadowing = false;

    int32_t shadowingType;
    int32_t mlt = 1;
    for (auto p : { &edEnd, &edSt })
    {
        shadowingType = shadowing20(p->p, points, edges, edgesCount, hasPoint, p1);
        status += mlt * shadowingType;

        if (shadowingType == 0 && !aShadowing && hasPoint)
        {
            aShadowing = true;
            aShadowingPair[0] = *p;
            aShadowingPair[1] = p1;
        }
        else if ((shadowingType == 1 || shadowingType == -1) && !bShadowing)
        {
            bShadowing = true;
            bShadowingPair[0] = *p;
            bShadowingPair[1] = p1;
        }
        mlt = -1;
    }

    for (int32_t ed = 0; ed < edgesCount; ++ed)
    {
        if (shouldSwap(points[edges[ed].s].p, points[edges[ed].e].p))
        {
            shadowingType = -edgeEdgeShadowing(points[edges[ed].e], points[edges[ed].s], edSt, edEnd, p1, p2, hasPoint);
        }
        else
        {
            shadowingType = edgeEdgeShadowing(points[edges[ed].s], points[edges[ed].e], edSt, edEnd, p1, p2, hasPoint);
        }
        status -= shadowingType;
        if (shadowingType == 0)
        {
            if (!aShadowing && hasPoint)
            {
                aShadowing = true;
                aShadowingPair[0] = p2;
                aShadowingPair[1] = p1;
            }
        }
        else
        {
            if ((shadowingType == 1 || shadowingType == -1) && !bShadowing)
            {
                bShadowing = true;
                bShadowingPair[0] = p2;
                bShadowingPair[1] = p1;
            }
        }
    }
    if (!status || !bShadowing || !aShadowing)
    {
        return 0;
    }
    float deltaPlus = bShadowingPair[0].p.z - bShadowingPair[1].p.z;
    float div = 0;
    if (deltaPlus != 0)
    {
        float deltaMinus = aShadowingPair[0].p.z - aShadowingPair[1].p.z;
        div = deltaPlus / (deltaPlus - deltaMinus);
    }
    intersectionA.p = bShadowingPair[1].p - div * (bShadowingPair[1].p - aShadowingPair[1].p);
    intersectionA.n = bShadowingPair[1].n - div * (bShadowingPair[1].n - aShadowingPair[1].n);
    intersectionA.uv[0] = bShadowingPair[1].uv[0] - (bShadowingPair[1].uv[0] - aShadowingPair[1].uv[0]) * div;

    intersectionB.p = intersectionA.p;
    intersectionB.n = bShadowingPair[0].n - div * (bShadowingPair[0].n - aShadowingPair[0].n);
    intersectionB.uv[0] = bShadowingPair[0].uv[0] - (bShadowingPair[0].uv[0] - aShadowingPair[0].uv[0]) * div;

    return status;
}

int32_t BooleanEvaluator::vertexMeshStatus03(const NvcVec3& p, const Mesh* mesh)
{
    int32_t status = 0;
    Vertex pnt;
    bool hasPoint = false;
    mAcceleratorB->setState(p);
    int32_t facet = mAcceleratorB->getNextFacet();
    while (facet != -1)
    {
        const Edge* ed = mesh->getEdges() + mesh->getFacet(facet)->firstEdgeNumber;
        status += shadowing02(p, mesh->getVertices(), ed, mesh->getFacet(facet)->edgesCount, hasPoint, pnt);
        facet = mAcceleratorB->getNextFacet();
    }

    return status;
}

int32_t BooleanEvaluator::vertexMeshStatus30(const NvcVec3& p, const Mesh* mesh)
{
    int32_t status = 0;
    bool hasPoints = false;
    Vertex point;
    mAcceleratorA->setState(p);
    int32_t facet = mAcceleratorA->getNextFacet();
    while ( facet != -1)
    {
        const Edge* ed = mesh->getEdges() + mesh->getFacet(facet)->firstEdgeNumber;
        status -= shadowing20(p, mesh->getVertices(), ed, mesh->getFacet(facet)->edgesCount, hasPoints, point);
        facet = mAcceleratorA->getNextFacet();
    }

    return status;
}

NV_FORCE_INLINE int32_t inclusionValue03(const BooleanConf& conf, int32_t xValue)
{
    return conf.ca + conf.ci * xValue;
}

NV_FORCE_INLINE int32_t inclusionValueEdgeFace(const BooleanConf& conf, int32_t xValue)
{
    return conf.ci * xValue;
}

NV_FORCE_INLINE int32_t inclusionValue30(const BooleanConf& conf, int32_t xValue)
{
    return conf.cb + conf.ci * xValue;
}

struct VertexComparator
{
    VertexComparator(NvcVec3 base = NvcVec3()) : basePoint(base) {};
    NvcVec3 basePoint;
    bool operator()(const Vertex& a, const Vertex& b)
    {
        return ((b.p - a.p) | basePoint) > 0.0;
    }
};

struct VertexPairComparator
{
    VertexPairComparator(NvcVec3 base = NvcVec3()) : basePoint(base) {};
    NvcVec3 basePoint;
    bool operator()(const std::pair<Vertex, Vertex>& a, const std::pair<Vertex, Vertex>& b)
    {
        return ((b.first.p - a.first.p) | basePoint) > 0.0;
    }
};

int32_t BooleanEvaluator::isPointContainedInMesh(const Mesh* msh, const NvcVec3& point)
{
    if (msh == nullptr)
    {
        return 0;
    }
    DummyAccelerator dmAccel(msh->getFacetCount());
    mAcceleratorA = &dmAccel;
    return vertexMeshStatus30(point, msh);

}

int32_t BooleanEvaluator::isPointContainedInMesh(const Mesh* msh, SpatialAccelerator* spAccel, const NvcVec3& point)
{
    if (msh == nullptr)
    {
        return 0;
    }
    mAcceleratorA = spAccel;
    return vertexMeshStatus30(point, msh);
}




void BooleanEvaluator::buildFaceFaceIntersections(const BooleanConf& mode)
{
    int32_t statusValue = 0;
    int32_t inclusionValue = 0;

    std::vector<std::pair<Vertex, Vertex> > retainedStarts;
    std::vector<std::pair<Vertex, Vertex>> retainedEnds;
    VertexPairComparator comp;

    Vertex newPointA;
    Vertex newPointB;

    const Vertex* meshAPoints = mMeshA->getVertices();
    const Vertex* meshBPoints = mMeshB->getVertices();
    EdgeWithParent newEdge;
    mEdgeFacetIntersectionData12.clear();
    mEdgeFacetIntersectionData21.clear();
    
    mEdgeFacetIntersectionData12.resize(mMeshA->getFacetCount());
    mEdgeFacetIntersectionData21.resize(mMeshB->getFacetCount());

    for (uint32_t facetB = 0; facetB < mMeshB->getFacetCount(); ++facetB)
    {
        mAcceleratorA->setState(meshBPoints, mMeshB->getEdges(), *mMeshB->getFacet(facetB));
        int32_t facetA = mAcceleratorA->getNextFacet();
        while (facetA != -1)
        {
            const Edge* facetBEdges = mMeshB->getEdges() + mMeshB->getFacet(facetB)->firstEdgeNumber;
            const Edge* facetAEdges = mMeshA->getEdges() + mMeshA->getFacet(facetA)->firstEdgeNumber;
            const Edge* fbe = facetBEdges;
            const Edge* fae = facetAEdges;
            retainedStarts.clear();
            retainedEnds.clear();
            NvcVec3 compositeEndPoint = {0, 0, 0};
            NvcVec3 compositeStartPoint = {0, 0, 0};
            uint32_t facetAEdgeCount = mMeshA->getFacet(facetA)->edgesCount;
            uint32_t facetBEdgeCount = mMeshB->getFacet(facetB)->edgesCount;
            int32_t ic = 0;
            for (uint32_t i = 0; i < facetAEdgeCount; ++i)
            {
                if (shouldSwap(meshAPoints[fae->e].p, meshAPoints[fae->s].p))
                {
                    statusValue = -edgeFacetIntersection12(meshAPoints[fae->e], meshAPoints[fae->s], meshBPoints, facetBEdges, facetBEdgeCount, newPointA, newPointB);
                }
                else
                {
                    statusValue = edgeFacetIntersection12(meshAPoints[fae->s], meshAPoints[fae->e], meshBPoints, facetBEdges, facetBEdgeCount, newPointA, newPointB);
                }
                inclusionValue = -inclusionValueEdgeFace(mode, statusValue);
                if (inclusionValue > 0)
                {
                    for (ic = 0; ic < inclusionValue; ++ic)
                    {
                        retainedEnds.push_back(std::make_pair(newPointA, newPointB));
                        compositeEndPoint = compositeEndPoint + newPointA.p;
                    }
                    mEdgeFacetIntersectionData12[facetA].push_back(EdgeFacetIntersectionData(i, statusValue, newPointA));
                }
                if (inclusionValue < 0)
                {
                    for (ic = 0; ic < -inclusionValue; ++ic)
                    {
                        retainedStarts.push_back(std::make_pair(newPointA, newPointB));
                        compositeStartPoint = compositeStartPoint + newPointA.p;
                    }
                    mEdgeFacetIntersectionData12[facetA].push_back(EdgeFacetIntersectionData(i, statusValue, newPointA));
                }
                fae++;
            }
            for (uint32_t i = 0; i < facetBEdgeCount; ++i)
            {
                if (shouldSwap(meshBPoints[fbe->e].p, meshBPoints[fbe->s].p))
                {
                    statusValue = -edgeFacetIntersection21(meshBPoints[fbe->e], meshBPoints[fbe->s], meshAPoints, facetAEdges, facetAEdgeCount, newPointA, newPointB);
                }
                else
                {
                    statusValue = edgeFacetIntersection21(meshBPoints[fbe->s], meshBPoints[fbe->e], meshAPoints, facetAEdges, facetAEdgeCount, newPointA, newPointB);
                }

                inclusionValue = inclusionValueEdgeFace(mode, statusValue);
                if (inclusionValue > 0)
                {
                    for (ic = 0; ic < inclusionValue; ++ic)
                    {
                        retainedEnds.push_back(std::make_pair(newPointA, newPointB));
                        compositeEndPoint = compositeEndPoint + newPointB.p;
                    }
                    mEdgeFacetIntersectionData21[facetB].push_back(EdgeFacetIntersectionData( i, statusValue, newPointB));
                }
                if (inclusionValue < 0)
                {
                    for (ic = 0; ic < -inclusionValue; ++ic)
                    {
                        retainedStarts.push_back(std::make_pair(newPointA, newPointB));
                        compositeStartPoint = compositeStartPoint + newPointB.p;
                    }
                    mEdgeFacetIntersectionData21[facetB].push_back(EdgeFacetIntersectionData(i, statusValue, newPointB));
                }
                fbe++;
            }
            if (retainedStarts.size() != retainedEnds.size())
            {
                NVBLAST_LOG_ERROR("Not equal number of starting and ending vertices! Probably input mesh has open edges.");
                return;
            }
            for (uint32_t rv = 0; rv < retainedStarts.size(); ++rv)
            {               
                newEdge.s = addIfNotExist(retainedStarts[rv].first);
                newEdge.e = addIfNotExist(retainedEnds[rv].first);
                newEdge.parent = facetA;
                addEdgeIfValid(newEdge);
                newEdge.parent = facetB + mMeshA->getFacetCount();
                newEdge.e = addIfNotExist(retainedStarts[rv].second);
                newEdge.s = addIfNotExist(retainedEnds[rv].second);
                addEdgeIfValid(newEdge);
            }
            facetA = mAcceleratorA->getNextFacet();
        } // while (*iter != -1)
    } // for (uint32_t facetB = 0; facetB < mMeshB->getFacetCount(); ++facetB)
}


void BooleanEvaluator::buildFastFaceFaceIntersection(const BooleanConf& mode)
{
    int32_t statusValue = 0;
    int32_t inclusionValue = 0;

    std::vector<std::pair<Vertex, Vertex> > retainedStarts;
    std::vector<std::pair<Vertex, Vertex>> retainedEnds;
    VertexPairComparator comp;

    Vertex newPointA;
    Vertex newPointB;
    const Vertex* meshAPoints = mMeshA->getVertices();
    const Vertex* meshBPoints = mMeshB->getVertices();
    EdgeWithParent newEdge;

    mEdgeFacetIntersectionData12.clear();
    mEdgeFacetIntersectionData21.clear();

    mEdgeFacetIntersectionData12.resize(mMeshA->getFacetCount());
    mEdgeFacetIntersectionData21.resize(mMeshB->getFacetCount());

    for (uint32_t facetA = 0; facetA < mMeshA->getFacetCount(); ++facetA)
    {
        const Edge* facetAEdges = mMeshA->getEdges() + mMeshA->getFacet(facetA)->firstEdgeNumber;
        int32_t facetB = 0;
        const Edge* facetBEdges = mMeshB->getEdges() + mMeshB->getFacet(facetB)->firstEdgeNumber;
        const Edge* fae = facetAEdges;
        retainedStarts.clear();
        retainedEnds.clear();
        NvcVec3 compositeEndPoint = {0, 0, 0};
        NvcVec3 compositeStartPoint = {0, 0, 0};
        uint32_t facetAEdgeCount = mMeshA->getFacet(facetA)->edgesCount;
        uint32_t facetBEdgeCount = mMeshB->getFacet(facetB)->edgesCount;
        int32_t ic = 0;
        for (uint32_t i = 0; i < facetAEdgeCount; ++i)
        {
            if (shouldSwap(meshAPoints[fae->e].p, meshAPoints[fae->s].p))
            {
                statusValue = -edgeFacetIntersection12(meshAPoints[fae->e], meshAPoints[fae->s], meshBPoints, facetBEdges, facetBEdgeCount, newPointA, newPointB);
            }
            else
            {
                statusValue = edgeFacetIntersection12(meshAPoints[fae->s], meshAPoints[fae->e], meshBPoints, facetBEdges, facetBEdgeCount, newPointA, newPointB);
            }

            inclusionValue = -inclusionValueEdgeFace(mode, statusValue);
            if (inclusionValue > 0)
            {
                for (ic = 0; ic < inclusionValue; ++ic)
                {
                    retainedEnds.push_back(std::make_pair(newPointA, newPointB));
                    compositeEndPoint = compositeEndPoint + newPointA.p;
                }
                mEdgeFacetIntersectionData12[facetA].push_back(EdgeFacetIntersectionData(i, statusValue, newPointA));
            }
            if (inclusionValue < 0)
            {
                for (ic = 0; ic < -inclusionValue; ++ic)
                {
                    retainedStarts.push_back(std::make_pair(newPointA, newPointB));
                    compositeStartPoint = compositeStartPoint + newPointA.p;
                }
                mEdgeFacetIntersectionData12[facetA].push_back(EdgeFacetIntersectionData(i, statusValue, newPointA));
            }
            fae++;
        }
        if (retainedStarts.size() != retainedEnds.size())
        {
            NVBLAST_LOG_ERROR("Not equal number of starting and ending vertices! Probably input mesh has open edges.");
            return;
        }
        if (retainedStarts.size() > 1)
        {
            comp.basePoint = compositeEndPoint - compositeStartPoint;
            std::sort(retainedStarts.begin(), retainedStarts.end(), comp);
            std::sort(retainedEnds.begin(), retainedEnds.end(), comp);
        }
        for (uint32_t rv = 0; rv < retainedStarts.size(); ++rv)
        {
            newEdge.s = addIfNotExist(retainedStarts[rv].first);
            newEdge.e = addIfNotExist(retainedEnds[rv].first);
            newEdge.parent = facetA;
            addEdgeIfValid(newEdge);
            newEdge.parent = facetB + mMeshA->getFacetCount();
            newEdge.e = addIfNotExist(retainedStarts[rv].second);
            newEdge.s = addIfNotExist(retainedEnds[rv].second);
            addEdgeIfValid(newEdge);
        }
    }
}


void BooleanEvaluator::collectRetainedPartsFromA(const BooleanConf& mode)
{
    int32_t statusValue = 0;
    int32_t inclusionValue = 0;
    const Vertex* vertices = mMeshA->getVertices();
    VertexComparator comp;
    const NvBounds3& bMeshBoudning = toNvShared(mMeshB->getBoundingBox());
    const Edge* facetEdges = mMeshA->getEdges();
    std::vector<Vertex> retainedStartVertices;
    std::vector<Vertex> retainedEndVertices;
    retainedStartVertices.reserve(255);
    retainedEndVertices.reserve(255);
    int32_t ic = 0;
    for (uint32_t facetId = 0; facetId < mMeshA->getFacetCount(); ++facetId)
    {
        retainedStartVertices.clear();
        retainedEndVertices.clear();
        for (uint32_t i = 0; i < mMeshA->getFacet(facetId)->edgesCount; ++i)
        {
            NvcVec3 compositeEndPoint = {0, 0, 0};
            NvcVec3 compositeStartPoint = {0, 0, 0};

            int32_t lastPos = static_cast<int32_t>(retainedEndVertices.size());
            /* Test start and end point of edge against mesh */
            if (bMeshBoudning.contains(toNvShared(vertices[facetEdges->s].p)))
            {
                statusValue = vertexMeshStatus03(vertices[facetEdges->s].p, mMeshB);
            }
            else
            {
                statusValue = 0;
            }

            inclusionValue = -inclusionValue03(mode, statusValue);
            if (inclusionValue > 0)
            {
                for (ic = 0; ic < inclusionValue; ++ic)
                {
                    retainedEndVertices.push_back(vertices[facetEdges->s]);
                    compositeEndPoint = compositeEndPoint + vertices[facetEdges->s].p;
                }
            }
            else if (inclusionValue < 0)
            {
                for (ic = 0; ic < -inclusionValue; ++ic)
                {
                    retainedStartVertices.push_back(vertices[facetEdges->s]);
                    compositeStartPoint = compositeStartPoint + vertices[facetEdges->s].p;
                }
            }

            if (bMeshBoudning.contains(toNvShared(vertices[facetEdges->e].p)))
            {
                statusValue = vertexMeshStatus03(vertices[facetEdges->e].p, mMeshB);
            }
            else
            {
                statusValue = 0;
            }

            inclusionValue = inclusionValue03(mode, statusValue);
            if (inclusionValue > 0)
            {
                for (ic = 0; ic < inclusionValue; ++ic)
                {
                    retainedEndVertices.push_back(vertices[facetEdges->e]);
                    compositeEndPoint = compositeEndPoint + vertices[facetEdges->e].p;
                }
            }
            else if (inclusionValue < 0)
            {
                for (ic = 0; ic < -inclusionValue; ++ic)
                {
                    retainedStartVertices.push_back(vertices[facetEdges->e]);
                    compositeStartPoint = compositeStartPoint + vertices[facetEdges->e].p;
                }
            }

            /* Test edge intersection with mesh*/
            for (uint32_t intrs = 0; intrs < mEdgeFacetIntersectionData12[facetId].size(); ++intrs)
            {
                const EdgeFacetIntersectionData& intr = mEdgeFacetIntersectionData12[facetId][intrs];
                if (intr.edId != (int32_t)i)
                    continue;

                inclusionValue = inclusionValueEdgeFace(mode, intr.intersectionType);
                if (inclusionValue > 0)
                {
                    for (ic = 0; ic < inclusionValue; ++ic)
                    {
                        retainedEndVertices.push_back(intr.intersectionPoint);
                        compositeEndPoint = compositeEndPoint + intr.intersectionPoint.p;
                    }
                }
                else if (inclusionValue < 0)
                {
                    for (ic = 0; ic < -inclusionValue; ++ic)
                    {
                        retainedStartVertices.push_back(intr.intersectionPoint);
                        compositeStartPoint = compositeStartPoint + intr.intersectionPoint.p;
                    }
                }
            }

            facetEdges++;
            if (retainedStartVertices.size() != retainedEndVertices.size())
            {
                NVBLAST_LOG_ERROR("Not equal number of starting and ending vertices! Probably input mesh has open edges.");
                return;
            }
            if (retainedEndVertices.size() - lastPos > 1)
            {
                comp.basePoint = compositeEndPoint - compositeStartPoint;
                std::sort(retainedStartVertices.begin() + lastPos, retainedStartVertices.end(), comp);
                std::sort(retainedEndVertices.begin() + lastPos, retainedEndVertices.end(), comp);
            }
        }

        EdgeWithParent newEdge;
        for (uint32_t rv = 0; rv < retainedStartVertices.size(); ++rv)
        {
            newEdge.s = addIfNotExist(retainedStartVertices[rv]);
            newEdge.e = addIfNotExist(retainedEndVertices[rv]);
            newEdge.parent = facetId;
            addEdgeIfValid(newEdge);
        }
    }

    return;
}

void BooleanEvaluator::collectRetainedPartsFromB(const BooleanConf& mode)
{
    int32_t statusValue = 0;
    int32_t inclusionValue = 0;
    const Vertex* vertices = mMeshB->getVertices();
    VertexComparator comp;
    const NvBounds3& aMeshBoudning = toNvShared(mMeshA->getBoundingBox());
    const Edge* facetEdges = mMeshB->getEdges();
    std::vector<Vertex> retainedStartVertices;
    std::vector<Vertex> retainedEndVertices;
    retainedStartVertices.reserve(255);
    retainedEndVertices.reserve(255);
    int32_t ic = 0;
    for (uint32_t facetId = 0; facetId < mMeshB->getFacetCount(); ++facetId)
    {
        retainedStartVertices.clear();
        retainedEndVertices.clear();
        for (uint32_t i = 0; i < mMeshB->getFacet(facetId)->edgesCount; ++i)
        {
            NvcVec3 compositeEndPoint = {0, 0, 0};
            NvcVec3 compositeStartPoint = {0, 0, 0};
            int32_t lastPos = static_cast<int32_t>(retainedEndVertices.size());
            if (aMeshBoudning.contains(toNvShared(vertices[facetEdges->s].p)))
            {
                statusValue = vertexMeshStatus30(vertices[facetEdges->s].p, mMeshA);
            }
            else
            {
                statusValue = 0;
            }
            
            inclusionValue = -inclusionValue30(mode, statusValue);
            if (inclusionValue > 0)
            {
                for (ic = 0; ic < inclusionValue; ++ic)
                {
                    retainedEndVertices.push_back(vertices[facetEdges->s]);
                    compositeEndPoint = compositeEndPoint + vertices[facetEdges->s].p;
                }

            }
            else if (inclusionValue < 0)
            {
                for (ic = 0; ic < -inclusionValue; ++ic)
                {
                    retainedStartVertices.push_back(vertices[facetEdges->s]);
                    compositeStartPoint = compositeStartPoint + vertices[facetEdges->s].p;
                }
            }

            if (aMeshBoudning.contains(toNvShared(vertices[facetEdges->e].p)))
            {
                statusValue = vertexMeshStatus30(vertices[facetEdges->e].p, mMeshA);
            }
            else
            {
                statusValue = 0;
            }

            inclusionValue = inclusionValue30(mode, statusValue);
            if (inclusionValue > 0)
            {
                for (ic = 0; ic < inclusionValue; ++ic)
                {
                    retainedEndVertices.push_back(vertices[facetEdges->e]);
                    compositeEndPoint = compositeEndPoint + vertices[facetEdges->e].p;
                }

            }
            else if (inclusionValue < 0)
            {
                for (ic = 0; ic < -inclusionValue; ++ic)
                {
                    retainedStartVertices.push_back(vertices[facetEdges->e]);
                    compositeStartPoint = compositeStartPoint + vertices[facetEdges->e].p;
                }

            }

            for (uint32_t intrs = 0; intrs < mEdgeFacetIntersectionData21[facetId].size(); ++intrs)
            {
                const EdgeFacetIntersectionData& intr = mEdgeFacetIntersectionData21[facetId][intrs];
                if (intr.edId != (int32_t)i)
                    continue;

                inclusionValue = inclusionValueEdgeFace(mode, intr.intersectionType);
                if (inclusionValue > 0)
                {
                    for (ic = 0; ic < inclusionValue; ++ic)
                    {
                        retainedEndVertices.push_back(intr.intersectionPoint);
                        compositeEndPoint = compositeEndPoint + intr.intersectionPoint.p;
                    }
                }
                else if (inclusionValue < 0)
                {
                    for (ic = 0; ic < -inclusionValue; ++ic)
                    {
                        retainedStartVertices.push_back(intr.intersectionPoint);
                        compositeStartPoint = compositeStartPoint + intr.intersectionPoint.p;
                    }
                }
            }

            facetEdges++;
            if (retainedStartVertices.size() != retainedEndVertices.size())
            {
                NVBLAST_LOG_ERROR("Not equal number of starting and ending vertices! Probably input mesh has open edges.");
                return;
            }
            if (retainedEndVertices.size() - lastPos > 1)
            {
                comp.basePoint = compositeEndPoint - compositeStartPoint;
                std::sort(retainedStartVertices.begin() + lastPos, retainedStartVertices.end(), comp);
                std::sort(retainedEndVertices.begin() + lastPos, retainedEndVertices.end(), comp);
            }
        }
        EdgeWithParent newEdge;
        for (uint32_t rv = 0; rv < retainedStartVertices.size(); ++rv)
        {           
            newEdge.s = addIfNotExist(retainedStartVertices[rv]);
            newEdge.e = addIfNotExist(retainedEndVertices[rv]);
            newEdge.parent = facetId + mMeshA->getFacetCount();
            addEdgeIfValid(newEdge);
        }
    }
    return;
}

bool EdgeWithParentSortComp(const EdgeWithParent& a, const EdgeWithParent& b)
{
    return a.parent < b.parent;
}


void BooleanEvaluator::performBoolean(const Mesh* meshA, const Mesh* meshB, SpatialAccelerator* spAccelA, SpatialAccelerator* spAccelB, const BooleanConf& mode)
{
    reset();
    mMeshA = meshA;
    mMeshB = meshB;
    mAcceleratorA = spAccelA;
    mAcceleratorB = spAccelB;
    buildFaceFaceIntersections(mode);
    collectRetainedPartsFromA(mode);
    collectRetainedPartsFromB(mode);
    mAcceleratorA = nullptr;
    mAcceleratorB = nullptr;
}

void BooleanEvaluator::performBoolean(const Mesh* meshA, const Mesh* meshB, const BooleanConf& mode)
{
    reset();
    mMeshA = meshA;
    mMeshB = meshB;
    DummyAccelerator ac = DummyAccelerator(mMeshA->getFacetCount());
    DummyAccelerator bc = DummyAccelerator(mMeshB->getFacetCount());
    performBoolean(meshA, meshB, &ac, &bc, mode);
}


void BooleanEvaluator::performFastCutting(const Mesh* meshA, const Mesh* meshB, SpatialAccelerator* spAccelA, SpatialAccelerator* spAccelB, const BooleanConf& mode)
{
    reset();
    mMeshA = meshA;
    mMeshB = meshB;
    mAcceleratorA = spAccelA;
    mAcceleratorB = spAccelB;
    buildFastFaceFaceIntersection(mode);
    collectRetainedPartsFromA(mode);
    mAcceleratorA = nullptr;
    mAcceleratorB = nullptr;
}

void BooleanEvaluator::performFastCutting(const Mesh* meshA, const Mesh* meshB, const BooleanConf& mode)
{
    reset();
    mMeshA = meshA;
    mMeshB = meshB;
    DummyAccelerator ac = DummyAccelerator(mMeshA->getFacetCount());
    DummyAccelerator bc = DummyAccelerator(mMeshB->getFacetCount());
    performFastCutting(meshA, meshB, &ac, &bc, mode);
}




BooleanEvaluator::BooleanEvaluator()
{
    mMeshA = nullptr;
    mMeshB = nullptr;
    mAcceleratorA = nullptr;
    mAcceleratorB = nullptr;
}
BooleanEvaluator::~BooleanEvaluator()
{
    reset();
}



Mesh* BooleanEvaluator::createNewMesh()
{
    if (mEdgeAggregate.size() == 0)
    {
        return nullptr;
    }
    std::sort(mEdgeAggregate.begin(), mEdgeAggregate.end(), EdgeWithParentSortComp);
    std::vector<Facet> newFacets;
    std::vector<Edge>  newEdges(mEdgeAggregate.size());
    int32_t lastPos = 0;
    uint32_t lastParent = mEdgeAggregate[0].parent;
    uint32_t collected = 0;
    int64_t userData = 0;
    int32_t materialId = 0;
    int32_t smoothingGroup = 0;

    for (uint32_t i = 0; i < mEdgeAggregate.size(); ++i)
    {
        if (mEdgeAggregate[i].parent != lastParent)
        {           
            if (lastParent < mMeshA->getFacetCount())
            {
                userData = mMeshA->getFacet(lastParent)->userData;
                materialId = mMeshA->getFacet(lastParent)->materialId;
                smoothingGroup = mMeshA->getFacet(lastParent)->smoothingGroup;
            }
            else
            {
                userData = mMeshB->getFacet(lastParent - mMeshA->getFacetCount())->userData;
                materialId = mMeshB->getFacet(lastParent - mMeshA->getFacetCount())->materialId;
                smoothingGroup = mMeshB->getFacet(lastParent - mMeshA->getFacetCount())->smoothingGroup;
            }
            newFacets.push_back({ lastPos, collected, userData, materialId, smoothingGroup });
            lastPos = i;
            lastParent = mEdgeAggregate[i].parent;
            collected = 0;
        }
        collected++;
        newEdges[i].s = mEdgeAggregate[i].s;
        newEdges[i].e = mEdgeAggregate[i].e;
    }
    if (lastParent < mMeshA->getFacetCount())
    {
        userData = mMeshA->getFacet(lastParent)->userData;
        materialId = mMeshA->getFacet(lastParent)->materialId;
        smoothingGroup = mMeshA->getFacet(lastParent)->smoothingGroup;
    }
    else
    {
        uint32_t pr = lastParent - mMeshA->getFacetCount();
        userData = mMeshB->getFacet(pr)->userData;
        materialId = mMeshB->getFacet(pr)->materialId;
        smoothingGroup = mMeshB->getFacet(pr)->smoothingGroup;
    }
    newFacets.push_back({ lastPos, collected, userData, materialId, smoothingGroup });
    return new MeshImpl(mVerticesAggregate.data(), newEdges.data(), newFacets.data(), static_cast<uint32_t>(mVerticesAggregate.size()), static_cast<uint32_t>(mEdgeAggregate.size()), static_cast<uint32_t>(newFacets.size()));
}

void BooleanEvaluator::reset()
{
    mMeshA          = nullptr;
    mMeshB          = nullptr;
    mAcceleratorA   = nullptr;
    mAcceleratorB   = nullptr;
    mEdgeAggregate.clear();
    mVerticesAggregate.clear();
    mEdgeFacetIntersectionData12.clear();
    mEdgeFacetIntersectionData21.clear();
}


/// BooleanTool

void BooleanToolImpl::release()
{
    delete this;
}

Mesh* BooleanToolImpl::performBoolean(const Mesh* meshA, SpatialAccelerator* accelA, const Mesh* meshB, SpatialAccelerator* accelB, BooleanTool::Op op)
{
    const BooleanConf modes[] =
    {
        BooleanConfigurations::BOOLEAN_INTERSECTION(),
        BooleanConfigurations::BOOLEAN_UNION(),
        BooleanConfigurations::BOOLEAN_DIFFERENCE(),
    };
    constexpr size_t modeCount = sizeof(modes)/sizeof(modes[0]);

    if (op < 0 || op >= modeCount)
    {
        NVBLAST_LOG_ERROR("Illegal mode passed into BooleanToolImpl::performBoolean.");
        return nullptr;
    }

    if (!meshA || !meshB)
    {
        NVBLAST_LOG_ERROR("Null mesh pointer passed into BooleanToolImpl::performBoolean.");
        return nullptr;
    }

    DummyAccelerator dmAccelA(meshA->getFacetCount());
    DummyAccelerator dmAccelB(meshA->getFacetCount());

    m_evaluator.performBoolean(meshA, meshB, accelA ? accelA : &dmAccelA, accelB ? accelB : &dmAccelB, modes[op]);

    return m_evaluator.createNewMesh();
}

bool BooleanToolImpl::pointInMesh(const Mesh* mesh, SpatialAccelerator* accel, const NvcVec3& point)
{
    if (!mesh)
    {
        NVBLAST_LOG_ERROR("Null mesh pointer passed into BooleanToolImpl::pointInMesh.");
        return false;
    }

    DummyAccelerator dmAccel(mesh->getFacetCount());

    return m_evaluator.isPointContainedInMesh(mesh, accel ? accel : &dmAccel, point);
}

} // namespace Blast
} // namespace Nv
