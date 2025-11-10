// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "ShrinkWrap.h"
#include "FM.h"
#include "QuickHull.h"
#include "RaycastMesh.h"
#include "Quantizer.h"
#include "SimpleMesh.h"

#include <vector>

#ifdef _MSC_VER
#pragma warning(disable:4100)
#endif

namespace vcd
{

class SVec3
{
public:
    SVec3(void) { };

    SVec3(double _x,double _y,double _z) : x(_x), y(_y), z(_z)
    {
    }

    double x;
    double y;
    double z;
};



class ShrinkWrapImpl : public ShrinkWrap
{
public:
    ShrinkWrapImpl(void)
    {
    }

    virtual ~ShrinkWrapImpl(void)
    {
    }

    virtual void shrinkWrap(SimpleMesh &sourceConvexHull, 
                            RaycastMesh &raycastMesh, 
                            uint32_t maxHullVertexCount,
                            double distanceThreshold,
                            bool doShrinkWrap) final
    {
        std::vector< SVec3 > verts; // New verts for the new convex hull
        // Examine each vertex and see if it is within the voxel distance.
        // If it is, then replace the point with the shrinkwrapped / projected point
        for (uint32_t j = 0; j < sourceConvexHull.mVertexCount; j++)
        {
            double *p = &sourceConvexHull.mVertices[j * 3];
            if (doShrinkWrap)
            {
                double closest[3];
                if (raycastMesh.getClosestPointWithinDistance(p, distanceThreshold, closest))
                {
                    p[0] = closest[0];
                    p[1] = closest[1];
                    p[2] = closest[2];
                }
            }
            SVec3 point(p[0], p[1], p[2]);
            verts.push_back(point);
        }
        // See if the number of vertices exceeds the maximum
        // allowed by the user.
        // If so, then we must quantize the input vertices to not exceed that limit
        if (verts.size() > maxHullVertexCount)
        {
            quickhull::WuQuantizer *q = quickhull::WuQuantizer::create();
            uint32_t outputCount;
            const double *newVerts = q->wuQuantize3D((uint32_t)verts.size(), &verts[0].x, true, maxHullVertexCount, outputCount);
            if (newVerts)
            {
                verts.resize(outputCount);
                for (uint32_t i = 0; i < outputCount; i++)
                {
                    const double *p = &newVerts[i * 3];
                    SVec3 v(p[0], p[1], p[2]);
                    verts[i] = v;
                }
            }
            q->release();
        }
        // Final step is to recompute the convex hull
        quickhull::QuickHull *qh = quickhull::QuickHull::create();
        quickhull::HullPoints hp;
        hp.mVertexCount = (uint32_t)verts.size();
        hp.mVertices = &verts[0].x;
        hp.mMaxHullVertices = maxHullVertexCount;
        hp.mMaxQuantizeVertexCount = maxHullVertexCount;
        uint32_t tcount = qh->computeConvexHull(hp);
        if (tcount)
        {
            delete[]sourceConvexHull.mVertices;
            delete[]sourceConvexHull.mIndices;
            delete []sourceConvexHull.mPolygonData;
            delete []sourceConvexHull.mPolygonIndices;

            sourceConvexHull.mVertices = nullptr;
            sourceConvexHull.mIndices = nullptr;
            sourceConvexHull.mPolygonData = nullptr;
            sourceConvexHull.mPolygonIndices = nullptr;

            const double *vtx = qh->getVertices(sourceConvexHull.mVertexCount);
            const uint32_t *idx = qh->getIndices(sourceConvexHull.mTriangleCount);

            sourceConvexHull.mVertices = new double[sourceConvexHull.mVertexCount * 3];
            sourceConvexHull.mIndices = new uint32_t[sourceConvexHull.mTriangleCount * 3];
            memcpy(sourceConvexHull.mVertices, vtx, sizeof(double)*sourceConvexHull.mVertexCount * 3);
            memcpy(sourceConvexHull.mIndices, idx, sizeof(uint32_t)*sourceConvexHull.mTriangleCount * 3);
            sourceConvexHull.mPolygonCount = qh->getPolygonCount();
            if ( sourceConvexHull.mPolygonCount )
            {
                SimplePolygon *polygonData = new SimplePolygon[sourceConvexHull.mPolygonCount];
                sourceConvexHull.mPolygonData = polygonData;
                std::vector< uint32_t > polygonIndices;
                for (uint32_t i=0; i<sourceConvexHull.mPolygonCount; i++)
                {
                    SimplePolygon &p = polygonData[i];
                    const uint32_t *indices = qh->getPolygon(i,p.mPointCount,p.mStartIndex,p.mPlaneEquation);
                    for (uint32_t j=0; j<p.mPointCount; j++)
                    {
                        polygonIndices.push_back(indices[j]);
                    }
                }
                sourceConvexHull.mPolygonIndexCount = (uint32_t)polygonIndices.size();
                if ( sourceConvexHull.mPolygonIndexCount )
                {
                    uint32_t *pi = new uint32_t[sourceConvexHull.mPolygonIndexCount];
                    memcpy(pi,&polygonIndices[0],sizeof(uint32_t)*sourceConvexHull.mPolygonIndexCount);
                    sourceConvexHull.mPolygonIndices = pi;
                }
            }
        }
        qh->release();
    }

    virtual void release(void) final
    {
        delete this;
    }
};

ShrinkWrap *ShrinkWrap::create(void)
{
    auto ret = new ShrinkWrapImpl;
    return static_cast< ShrinkWrap *>(ret);
}


}

