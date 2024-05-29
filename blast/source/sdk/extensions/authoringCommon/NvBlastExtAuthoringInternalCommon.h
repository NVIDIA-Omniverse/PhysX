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
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTINTERNALCOMMON_H
#define NVBLASTINTERNALCOMMON_H
#include "NvBlastExtAuthoringTypes.h"
#include "NvBlastNvSharedHelpers.h"
#include "NvBlastVolumeIntegrals.h"
#include "NvVec2.h"
#include "NvVec3.h"
#include "NvPlane.h"
#include "NvBounds3.h"
#include "NvMath.h"
#include <algorithm>

namespace Nv
{
namespace Blast
{

/**
Edge representation with index of parent facet
*/
struct EdgeWithParent
{
    uint32_t s, e; // Starting and ending vertices
    uint32_t parent; // Parent facet index
    EdgeWithParent() : s(0), e(0), parent(0) {}
    EdgeWithParent(uint32_t s, uint32_t e, uint32_t p) : s(s), e(e), parent(p) {}
};


/**
Comparator for sorting edges according to parent facet number.
*/
struct EdgeComparator
{
    bool operator()(const EdgeWithParent& a, const EdgeWithParent& b) const
    {
        if (a.parent == b.parent)
        {
            if (a.s == b.s)
            {
                return a.e < b.e;
            }
            else
            {
                return a.s < b.s;
            }
        }
        else
        {
            return a.parent < b.parent;
        }
    }
};

inline bool operator<(const Edge& a, const Edge& b)
{
    if (a.s == b.s)
        return a.e < b.e;
    else
        return a.s < b.s;
}

/**
Vertex projection direction flag.
*/
enum ProjectionDirections
{
    YZ_PLANE = 1 << 1,
    XY_PLANE = 1 << 2,
    ZX_PLANE = 1 << 3,

    // This is set when the dominant axis of the normal is negative
    // because when flattening to 2D the facet is viewed from the positive direction.
    // As a result, the winding order appears to flip if the normal is in the negative direction.
    OPPOSITE_WINDING = 1 << 4
};

/**
Computes best direction to project points.
*/
NV_FORCE_INLINE ProjectionDirections getProjectionDirection(const nvidia::NvVec3& normal)
{
    float maxv = std::max(std::abs(normal.x), std::max(std::abs(normal.y), std::abs(normal.z)));
    ProjectionDirections retVal;
    if (maxv == std::abs(normal.x))
    {
        retVal = YZ_PLANE;
        if (normal.x < 0) retVal = (ProjectionDirections)((int)retVal | (int)OPPOSITE_WINDING);
        return retVal;
    }
    if (maxv == std::abs(normal.y))
    {
        retVal = ZX_PLANE;
        if (normal.y > 0) retVal = (ProjectionDirections)((int)retVal | (int)OPPOSITE_WINDING);
        return retVal;
    }
    retVal = XY_PLANE;
    if (normal.z < 0) retVal = (ProjectionDirections)((int)retVal | (int)OPPOSITE_WINDING);
    return retVal;
}


/**
Computes point projected on given axis aligned plane.
*/
NV_FORCE_INLINE nvidia::NvVec2 getProjectedPoint(const nvidia::NvVec3& point, ProjectionDirections dir)
{
    if (dir & YZ_PLANE)
    {
        return nvidia::NvVec2(point.y, point.z);
    }
    if (dir & ZX_PLANE)
    {
        return nvidia::NvVec2(point.x, point.z);
    }
    return nvidia::NvVec2(point.x, point.y);
}

NV_FORCE_INLINE nvidia::NvVec2 getProjectedPoint(const NvcVec3& point, ProjectionDirections dir)
{
    return getProjectedPoint((const nvidia::NvVec3&)point, dir);
}

/**
Computes point projected on given axis aligned plane, this method is polygon-winding aware.
*/
NV_FORCE_INLINE nvidia::NvVec2 getProjectedPointWithWinding(const nvidia::NvVec3& point, ProjectionDirections dir)
{
    if (dir & YZ_PLANE)
    {
        if (dir & OPPOSITE_WINDING)
        {
            return nvidia::NvVec2(point.z, point.y);
        }
        else
        return nvidia::NvVec2(point.y, point.z);
    }
    if (dir & ZX_PLANE)
    {
        if (dir & OPPOSITE_WINDING)
        {
            return nvidia::NvVec2(point.z, point.x);
        }
        return nvidia::NvVec2(point.x, point.z);
    }
    if (dir & OPPOSITE_WINDING)
    {
        return nvidia::NvVec2(point.y, point.x);
    }
    return nvidia::NvVec2(point.x, point.y);
}



#define MAXIMUM_EXTENT 1000 * 1000 * 1000
#define BBOX_TEST_EPS 1e-5f 

/**
Test fattened bounding box intersetion.
*/
NV_INLINE bool  weakBoundingBoxIntersection(const nvidia::NvBounds3& aBox, const nvidia::NvBounds3& bBox)
{
    if (std::max(aBox.minimum.x, bBox.minimum.x) > std::min(aBox.maximum.x, bBox.maximum.x) + BBOX_TEST_EPS)
        return false;
    if (std::max(aBox.minimum.y, bBox.minimum.y) > std::min(aBox.maximum.y, bBox.maximum.y) + BBOX_TEST_EPS)
        return false;
    if (std::max(aBox.minimum.z, bBox.minimum.z) > std::min(aBox.maximum.z, bBox.maximum.z) + BBOX_TEST_EPS)
        return false;
    return true;
}



/**
Test segment vs plane intersection. If segment intersects the plane true is returned. Point of intersection is written into 'result'.
*/
NV_INLINE bool getPlaneSegmentIntersection(const nvidia::NvPlane& pl, const nvidia::NvVec3& a, const nvidia::NvVec3& b,
                                           nvidia::NvVec3& result)
{
    float div = (b - a).dot(pl.n);
    if (nvidia::NvAbs(div) < 0.0001f)
    {
        if (pl.contains(a))
        {
            result = a;
            return true;
        }
        else
        {
            return false;
        }
    }
    float t = (-a.dot(pl.n) - pl.d) / div;
    if (t < 0.0f || t > 1.0f)
    {
        return false;
    }
    result = (b - a) * t + a;
    return true;
}


#define POS_COMPARISON_OFFSET 1e-5f
#define NORM_COMPARISON_OFFSET 1e-3f
/**
Vertex comparator for vertex welding.
*/
template<bool splitUVs>
struct VrtCompare
{
    // This implements a "less than" function for vertices.
    // Vertices a and b are considered equivalent if !(a < b) && !(b < a)
    bool operator()(const Vertex& a, const Vertex& b) const
    {
        if (a.p.x + POS_COMPARISON_OFFSET < b.p.x) return true;
        if (a.p.x - POS_COMPARISON_OFFSET > b.p.x) return false;
        if (a.p.y + POS_COMPARISON_OFFSET < b.p.y) return true;
        if (a.p.y - POS_COMPARISON_OFFSET > b.p.y) return false;
        if (a.p.z + POS_COMPARISON_OFFSET < b.p.z) return true;
        if (a.p.z - POS_COMPARISON_OFFSET > b.p.z) return false;

        if (a.n.x + NORM_COMPARISON_OFFSET < b.n.x) return true;
        if (a.n.x - NORM_COMPARISON_OFFSET > b.n.x) return false;
        if (a.n.y + NORM_COMPARISON_OFFSET < b.n.y) return true;
        if (a.n.y - NORM_COMPARISON_OFFSET > b.n.y) return false;
        if (a.n.z + NORM_COMPARISON_OFFSET < b.n.z) return true;
        if (a.n.z - NORM_COMPARISON_OFFSET > b.n.z) return false;   // This is not actually needed if (!splitUVs)

        if (!splitUVs) return false;

        if (a.uv[0].x + NORM_COMPARISON_OFFSET < b.uv[0].x) return true;
        if (a.uv[0].x - NORM_COMPARISON_OFFSET > b.uv[0].x) return false;
        if (a.uv[0].y + NORM_COMPARISON_OFFSET < b.uv[0].y) return true;
        if (a.uv[0].y - NORM_COMPARISON_OFFSET > b.uv[0].y) return false;   // This is not actually needed

        return false;
    };
};

typedef VrtCompare<true> VrtComp;
typedef VrtCompare<false> VrtCompNoUV;

/**
Vertex comparator for vertex welding (not accounts normal and uv parameters of vertice).
*/
struct VrtPositionComparator
{
    bool operator()(const NvcVec3& a, const NvcVec3& b) const
    {
        if (a.x + POS_COMPARISON_OFFSET < b.x) return true;
        if (a.x - POS_COMPARISON_OFFSET > b.x) return false;
        if (a.y + POS_COMPARISON_OFFSET < b.y) return true;
        if (a.y - POS_COMPARISON_OFFSET > b.y) return false;
        if (a.z + POS_COMPARISON_OFFSET < b.z) return true;
        if (a.z - POS_COMPARISON_OFFSET > b.z) return false;
        return false;
    };
    bool operator()(const Vertex& a, const Vertex& b) const
    {
        return operator()(a.p, b.p);
    };
};


NV_INLINE float calculateCollisionHullVolumeAndCentroid(NvcVec3& centroid, const CollisionHull& hull)
{
    class CollisionHullQuery
    {
    public:
        CollisionHullQuery(const CollisionHull& hull) : m_hull(hull) {}

        size_t faceCount() const { return (size_t)m_hull.polygonDataCount; }

        size_t vertexCount(size_t faceIndex) const { return (size_t)m_hull.polygonData[faceIndex].vertexCount; }

        NvcVec3 vertex(size_t faceIndex, size_t vertexIndex) const
        {
            return m_hull.points[m_hull.indices[m_hull.polygonData[faceIndex].indexBase + vertexIndex]];
        }

    private:
        const CollisionHull& m_hull;        
    };

    return calculateMeshVolumeAndCentroid<CollisionHullQuery>(centroid, hull);
}

}   // namespace Blast
}   // namespace Nv

#endif