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
// Copyright (c) 2016-2022 NVIDIA Corporation. All rights reserved.


#include "NvBlastExtTriangleProcessor.h"
#include "NvBlastExtAuthoringInternalCommon.h"
#define COLLIN_EPS 1e-4f
#define V_COMP_EPS 1e-5f

using namespace physx;

namespace Nv
{
namespace Blast
{
/**
    Segments bounding box interseciton test
*/
bool boundingRectangleIntersection(const PxVec2& s1, const PxVec2& e1, const PxVec2& s2, const PxVec2& e2)
{
    // sl1/sl2 is always left bottom end of rectangle
    // se1/el2 is always right top end of rectangle

    PxF32 sl1, sl2, el1, el2;
    if (s1.x < e1.x)
    {
        sl1 = s1.x;
        el1 = e1.x;
    }
    else
    {
        el1 = s1.x;
        sl1 = e1.x;
    }

    if (s2.x < e2.x)
    {
        sl2 = s2.x;
        el2 = e2.x;
    }
    else
    {
        el2 = s2.x;
        sl2 = e2.x;
    }
    if (PxMax(sl1, sl2) > PxMin(el1, el2))
        return false;

    if (s1.y < e1.y)
    {
        sl1 = s1.y;
        el1 = e1.y;
    }
    else
    {
        el1 = s1.y;
        sl1 = e1.y;
    }

    if (s2.y < e2.y)
    {
        sl2 = s2.y;
        el2 = e2.y;
    }
    else
    {
        el2 = s2.y;
        sl2 = e2.y;
    }
    if (PxMax(sl1, sl2) > PxMin(el1, el2))
        return false;

    return true;
}

inline PxF32 getRotation(PxVec2 a, PxVec2 b)
{
    return a.x * b.y - a.y * b.x;
}

inline PxF32 getParameter(const PxVec2& a, const PxVec2& b, const PxVec2& point)
{
    return (point - a).magnitude() / (b - a).magnitude();
}
inline PxVec3 lerp3D(const PxVec3& a, const PxVec3& b, const PxF32 t)
{
    return (b - a) * t + a;
}



struct Line2D
{
    PxVec2 normal;
    PxF32 c;
    Line2D(PxVec2 vec, PxVec2 point)
    {
        normal.x = vec.y;
        normal.y = -vec.x;
        c = -normal.dot(point);
    }
};


uint32_t TriangleProcessor::getSegmentIntersection(const PxVec2& s1, const PxVec2& e1, const PxVec2& s2, const PxVec2& e2, PxF32& t1)
{
    if (!boundingRectangleIntersection(s1, e1, s2, e2))
        return 0;

    PxVec2 vec1 = e1 - s1;
    PxVec2 vec2 = e2 - s2;
    PxF32 det1 = getRotation(vec1, vec2);
    if (PxAbs(det1) < COLLIN_EPS)
    {
        return 0;
    }
    Line2D lineA(vec1, s1);
    Line2D lineB(vec2, s2);
    PxVec2 fInt;

    PxF32 detX = lineA.normal.y * lineB.c - lineA.c * lineB.normal.y;
    PxF32 detY = lineA.c * lineB.normal.x - lineB.c * lineA.normal.x;
    PxF32 x = detX / det1;
    PxF32 y = detY / det1;

    if (x + V_COMP_EPS >= PxMax(PxMin(s1.x, e1.x), PxMin(s2.x, e2.x)) &&
        x - V_COMP_EPS <= PxMin(PxMax(s1.x, e1.x), PxMax(s2.x, e2.x)) &&
        y + V_COMP_EPS >= PxMax(PxMin(s1.y, e1.y), PxMin(s2.y, e2.y)) &&
        y - V_COMP_EPS <= PxMin(PxMax(s1.y, e1.y), PxMax(s2.y, e2.y)))
    {
        fInt.x = x;
        fInt.y = y;
        t1 = getParameter(s1, e1, fInt);
        return 1;
    }

    return 0;
}

struct cwComparer
{
    PxVec3 basePoint;
    PxVec3 normal;
    cwComparer(PxVec3 basePointIn, PxVec3 norm)
    {
        basePoint = basePointIn;
        normal = norm;
    };
    bool operator()(const PxVec3& a, const PxVec3& b)
    {
        PxVec3 norm = (a - basePoint).cross(b - basePoint);
        return normal.dot(norm) > 0;        
    }
};

bool vec3Comparer(const PxVec3& a, const PxVec3& b)
{
        if (a.x + V_COMP_EPS < b.x) return true;
        if (a.x - V_COMP_EPS > b.x) return false;
        if (a.y + V_COMP_EPS < b.y) return true;
        if (a.y - V_COMP_EPS > b.y) return false;
        if (a.z + V_COMP_EPS < b.z) return true;
        return false;
}

void TriangleProcessor::sortToCCW(std::vector<PxVec3>& points, PxVec3& normal)
{
    std::sort(points.begin(), points.end(), vec3Comparer);
    int lastUnique = 0;
    for (uint32_t i = 1; i < points.size(); ++i)
    {
        PxVec3 df = (points[i] - points[lastUnique]).abs();
        if (df.x > V_COMP_EPS || df.y > V_COMP_EPS || df.z > V_COMP_EPS)
        {
            points[++lastUnique] = points[i];
        }
    }
    points.resize(lastUnique + 1);
    if (points.size() > 2)
    {
        cwComparer compr(points[0], normal);
        std::sort(points.begin() + 1, points.end(), compr);
    }
}



void TriangleProcessor::buildConvexHull(std::vector<PxVec3>& points, std::vector<PxVec3>& convexHull,const PxVec3& normal)
{

    std::sort(points.begin(), points.end(), vec3Comparer);
    int lastUnique = 0;
    for (uint32_t i = 1; i < points.size(); ++i)
    {
        PxVec3 df = (points[i] - points[lastUnique]).abs();
        if (df.x > V_COMP_EPS || df.y > V_COMP_EPS || df.z > V_COMP_EPS)
        {
            points[++lastUnique] = points[i];
        }
    }
    points.resize(lastUnique + 1);
    if (points.size() > 2)
    {
        cwComparer compr(points[0], normal);
        std::sort(points.begin() + 1, points.end(), compr);
    }
    if (points.size() < 3)
        return;
    convexHull.push_back(points[0]);
    convexHull.push_back(points[1]);
    ProjectionDirections projectionDirection = getProjectionDirection(normal);
    for (uint32_t i = 2; i < points.size(); ++i)
    {
        PxVec2 pnt = getProjectedPointWithWinding(points[i], projectionDirection);
        PxVec2 vec = pnt - getProjectedPointWithWinding(convexHull.back(), projectionDirection);
        if (PxAbs(vec.x) < V_COMP_EPS && PxAbs(vec.y) < V_COMP_EPS)
        {
            continue;
        }
        if (getRotation(vec, getProjectedPointWithWinding(convexHull.back(), projectionDirection) - getProjectedPointWithWinding(convexHull[convexHull.size() - 2], projectionDirection)) < 0)
        {
            convexHull.push_back(points[i]);
        }
        else
        {
            while (convexHull.size() > 1 && getRotation(vec, getProjectedPointWithWinding(convexHull.back(), projectionDirection) - getProjectedPointWithWinding(convexHull[convexHull.size() - 2], projectionDirection)) > 0)
            {
                convexHull.pop_back();
                vec = pnt - getProjectedPointWithWinding(convexHull.back(), projectionDirection);
            }
            convexHull.push_back(points[i]);
        }
    }
}


uint32_t TriangleProcessor::getTriangleIntersection(TrPrcTriangle& a, TrPrcTriangle2d& aProjected, TrPrcTriangle &b, PxVec3& centroid, std::vector<PxVec3>& intersectionBuffer, PxVec3 normal)
{

    b.points[0] -= centroid;
    b.points[1] -= centroid;
    b.points[2] -= centroid;

    ProjectionDirections prjDir = getProjectionDirection(normal);

    TrPrcTriangle2d bProjected;
    bProjected.points[0] = getProjectedPointWithWinding(b.points[0], prjDir);
    bProjected.points[1] = getProjectedPointWithWinding(b.points[1], prjDir);
    bProjected.points[2] = getProjectedPointWithWinding(b.points[2], prjDir);


    if (!triangleBoundingBoxIntersection(aProjected, bProjected)) return 0;

    //* Check triangle A against points of B *//
    for (int i = 0; i < 3; ++i)
    {
        if (isPointInside(bProjected.points[i], aProjected))
        {
            intersectionBuffer.push_back(b.points[i]);
        }
    }
    //* Check triangle B against points of A *//
    for (int i = 0; i < 3; ++i)
    {
        if (isPointInside(aProjected.points[i], bProjected))
        {
            intersectionBuffer.push_back(a.points[i]);
        }
    }

    //* Check edges intersection *//
    float param = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (getSegmentIntersection(aProjected.points[i], aProjected.points[(i + 1) % 3], bProjected.points[j], bProjected.points[(j + 1) % 3], param))
            {
                intersectionBuffer.push_back(lerp3D(a.points[i], a.points[(i + 1) % 3], param));
            }
        }
    }

    if (intersectionBuffer.size() == 0)
        return 0;

    // Intersection between two triangles is convex, but points should be reordered to construct right polygon //
    std::vector<PxVec3> intrs;
    buildConvexHull(intersectionBuffer, intrs, normal);
    intersectionBuffer = intrs;

    // Return all points back from origin //
    for (uint32_t i = 0; i < intersectionBuffer.size(); ++i)
    {
        intersectionBuffer[i] += centroid;
    }
    return 1;
}



bool TriangleProcessor::triangleBoundingBoxIntersection(TrPrcTriangle2d& a, TrPrcTriangle2d& b)
{
    float fb = std::min(a.points[0].x, std::min(a.points[1].x, a.points[2].x));
    float fe = std::max(a.points[0].x, std::max(a.points[1].x, a.points[2].x));

    float sb = std::min(b.points[0].x, std::min(b.points[1].x, b.points[2].x));
    float se = std::max(b.points[0].x, std::max(b.points[1].x, b.points[2].x));

    if (std::min(fe, se) + V_COMP_EPS < std::max(fb, sb)) return 0;

    fb = std::min(a.points[0].y, std::min(a.points[1].y, a.points[2].y));
    fe = std::max(a.points[0].y, std::max(a.points[1].y, a.points[2].y));

    sb = std::min(b.points[0].y, std::min(b.points[1].y, b.points[2].y));
    se = std::max(b.points[0].y, std::max(b.points[1].y, b.points[2].y));
    if (std::min(fe, se) + V_COMP_EPS < std::max(fb, sb)) return 0;
    return 1;
}


uint32_t TriangleProcessor::isPointInside(const PxVec2& point, const TrPrcTriangle2d& triangle)
{
    PxF32 av = getRotation(point - triangle.points[0], triangle.points[1] - triangle.points[0]);
    PxF32 bv = getRotation(point - triangle.points[1], triangle.points[2] - triangle.points[1]);
    PxF32 cv = getRotation(point - triangle.points[2], triangle.points[0] - triangle.points[2]);


    if (PxAbs(av) < COLLIN_EPS) av = 0;
    if (PxAbs(bv) < COLLIN_EPS) bv = 0;
    if (PxAbs(cv) < COLLIN_EPS) cv = 0;

    if (av >= 0 && bv >= 0 && cv >= 0)
    {
        if (av == 0 || bv == 0 || cv == 0)
            return 2;
        return 1;
    }
    if (av <= 0 && bv <= 0 && cv <= 0)
    {
        if (av == 0 || bv == 0 || cv == 0)
            return 2;
        return 1;
    }
    return 0;
}

} // namespace Blast
} // namespace Nv
