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


#ifndef NVBLASTEXTTRIANGLEPROCESSOR_H
#define NVBLASTEXTTRIANGLEPROCESSOR_H

#include "NvVec2.h"
#include "NvVec3.h"
#include <vector>
#include <algorithm>

using namespace nvidia;


namespace Nv
{
namespace Blast
{

/**
    Triangle processor internal triangle representation. Contains only vertex positions.
*/
struct TrPrcTriangle
{
    NvVec3 points[3];
    TrPrcTriangle(NvVec3 a = NvVec3(0.0f), NvVec3 b = NvVec3(0.0f), NvVec3 c = NvVec3(0.0f))
    {
        points[0] = a;
        points[1] = b;
        points[2] = c;
    }

    TrPrcTriangle& operator=(const TrPrcTriangle& b)
    {
        points[0] = b.points[0];
        points[1] = b.points[1];
        points[2] = b.points[2];
        return *this;
    }

    TrPrcTriangle(const TrPrcTriangle& b)
    {
        points[0] = b.points[0];
        points[1] = b.points[1];
        points[2] = b.points[2];
    }
    NvVec3 getNormal() const
    {
        return (points[1] - points[0]).cross(points[2] - points[0]);
    }
};

/**
    Triangle processor internal 2D triangle representation. Contains only vertex positions.
*/
struct TrPrcTriangle2d
{
    NvVec2 points[3];
    TrPrcTriangle2d(NvVec2 a = NvVec2(0.0f), NvVec2 b = NvVec2(0.0f), NvVec2 c = NvVec2(0.0f))
    {
        points[0] = a;
        points[1] = b;
        points[2] = c;
    }

    TrPrcTriangle2d operator=(const TrPrcTriangle2d& b)
    {
        points[0] = b.points[0];
        points[1] = b.points[1];
        points[2] = b.points[2];
        return *this;
    }

    TrPrcTriangle2d(const TrPrcTriangle2d& b)
    {
        points[0] = b.points[0];
        points[1] = b.points[1];
        points[2] = b.points[2];
    }
};

class TriangleProcessor
{
  public:
    TriangleProcessor(){};
    ~TriangleProcessor() {}


    /**
        Build intersection between two triangles
        \param[in] a            First triangle (A)
        \param[in] aProjected   Projected triangle A
        \param[in] b            Second triangle (B)
        \param[in] centroid     Centroid of first triangle (A)
        \param[out] intersectionBuffer Result intersection polygon
        \param[in] normal       Normal vector to triangle (Common for both A and B).
        \return 1 - if if intersection is found.
    */
    uint32_t getTriangleIntersection(TrPrcTriangle& a, TrPrcTriangle2d& aProjected, TrPrcTriangle& b, NvVec3& centroid,
                                     std::vector<NvVec3>& intersectionBuffer, NvVec3 normal);

    /**
        Test whether BB of triangles intersect.
        \param[in] a            First triangle (A)
        \param[in] b            Second triangle (B)
        \return true - if intersect
    */
    bool triangleBoundingBoxIntersection(TrPrcTriangle2d& a, TrPrcTriangle2d& b);


    /**
        Test whether point is inside of triangle.
        \param[in] point        Point coordinates in 2d space.
        \param[in] triangle     Triangle in 2d space.
        \return 1 - if inside, 2 if on edge, 0 if neither inside nor edge.
    */
    uint32_t isPointInside(const NvVec2& point, const TrPrcTriangle2d& triangle);

    /**
        Segment intersection point
        \param[in] s1 Segment-1 start point
        \param[in] e1 Segment-1 end point
        \param[in] s2 Segment-2 start point
        \param[in] e2 Segment-2 end point
        \param[out] t1 Intersection point parameter relatively to Segment-1, lies in [0.0, 1.0] range.
        \return 0 if there is no intersections, 1 - if intersection is found.
    */
    uint32_t getSegmentIntersection(const NvVec2& s1, const NvVec2& e1, const NvVec2& s2, const NvVec2& e2, float& t1);

    /**
        Sort vertices of polygon in CCW-order
    */
    void sortToCCW(std::vector<NvVec3>& points, NvVec3& normal);

    /**
        Builds convex polygon for given set of points. Points should be coplanar.
        \param[in] points Input array of points
        \param[out] convexHull Output polygon
        \param[in] normal Normal vector to polygon.
    */
    void buildConvexHull(std::vector<NvVec3>& points, std::vector<NvVec3>& convexHull, const NvVec3& normal);
};

}  // namespace Blast
}  // namespace Nv


#endif  // NVBLASTEXTTRIANGLEPROCESSOR_H
