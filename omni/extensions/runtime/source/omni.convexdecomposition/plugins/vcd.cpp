// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "vcd.h"

#include "FM.h"
#include "RaycastMesh.h"
#include "voxelize.h"
#include "QuickHull.h"
#include "SimpleMesh.h"
#include "MergeConvexHulls.h"
#include "Voxel.h"
#include "Quantizer.h"
#include "SimpleMesh.h"
#include "ScopedTime.h"
#include "VoxelTriMesh.h"

#include "JobSystem.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <float.h>
#include <math.h>

#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
#include <algorithm>

#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingTypes.h>

#include <common/utilities/MemoryMacros.h>

#ifdef _MSC_VER
#pragma warning(disable:4100 4189 4996)
#endif

#define REDUCE_POINT_COUNT 100000

#define SHOW_TIME 0
#define SHOW_TOTAL_TIME 0

#define RAYCAST_VOXELS 1

using namespace carb::tasking;

namespace vcd
{

#define MAX_SPLIT_LOC 32

void jobCallback(void *userPtr);

class Vec3d
{
public:
    Vec3d(void) {};
    Vec3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z)
    {
    }
    Vec3d(const double v[3])
    {
        x = double(v[0]);
        y = double(v[1]);
        z = double(v[2]);
    }
    Vec3d(const float v[3])
    {
        x = double(v[0]);
        y = double(v[1]);
        z = double(v[2]);
    }

    double   x;
    double   y;
    double   z;
};

class SplitLoc
{
public:
    int32_t mLoc{0};
    double  mError{0};
};




class Plane
{
public:

    bool operator==(const Plane &p) const
    {
        bool ret = false;

        double diff = fabs(d-p.d);
        if ( diff < 0.00001f )
        {
            double dot = vcd::fm_dot(&x,&p.x);
            if ( dot >= 0.98f )
            {
                ret = true;
            }
        }

        return ret;
    }

    double getDistance(const Vec3d &p) const
    {
        return vcd::fm_distToPlane(&x,&p.x);
    }

    double x;
    double y;
    double z;
    double d;
};

class HullMesh : public SimpleMesh
{
public:
    HullMesh(quickhull::QuickHull *qh)
    {
        mOwnAllocation = true;
        const double *vertices = qh->getVertices(mVertexCount);
        const uint32_t *indices = qh->getIndices(mTriangleCount);
        if ( vertices )
        {
            mVertices = new double[mVertexCount * 3];
            memcpy(mVertices, vertices, sizeof(double)*mVertexCount * 3);
        }
        if ( indices )
        {
            mIndices = new uint32_t[mTriangleCount * 3];
            memcpy(mIndices, indices, sizeof(uint32_t)*mTriangleCount * 3);
        }
        mPolygonCount = qh->getPolygonCount();
        if ( mPolygonCount )
        {
            SimplePolygon *polygonData = new SimplePolygon[mPolygonCount];
            mPolygonData = polygonData;
            std::vector< uint32_t > polygonIndices;
            for (uint32_t i=0; i<mPolygonCount; i++)
            {
                SimplePolygon &p = polygonData[i];
                const uint32_t *pindices = qh->getPolygon(i, p.mPointCount, p.mStartIndex, p.mPlaneEquation);
                for (uint32_t j = 0; j < p.mPointCount; j++)
                {
                    polygonIndices.push_back(pindices[j]);
                }
            }
            mPolygonIndexCount = (uint32_t)polygonIndices.size();
            if (mPolygonIndexCount)
            {
                uint32_t *pi = new uint32_t[mPolygonIndexCount];
                memcpy(pi, &polygonIndices[0], sizeof(uint32_t)*mPolygonIndexCount);
                mPolygonIndices = pi;
            }
        }
    }

    HullMesh(uint32_t vcount,const double *vertices,uint32_t tcount,const uint32_t *indices)
    {
        mOwnAllocation = true;
        mVertexCount = vcount;
        mTriangleCount = tcount;
        mVertices = new double[mVertexCount*3];
        memcpy(mVertices, vertices, sizeof(double)*mVertexCount * 3);
        mIndices = new uint32_t[mTriangleCount*3];
        memcpy(mIndices,indices,sizeof(uint32_t)*mTriangleCount*3);
    }

    HullMesh(void)
    {
    }

    ~HullMesh(void)
    {
    }


    void computeAABB(void)
    {
        fm_getAABB(mVertexCount,mVertices,sizeof(double)*3,mBmin,mBmax);
    }

    void inflateAABB(double fraction)
    {
        fm_inflateMinMax(mBmin,mBmax,fraction);
    }


};

using HullMeshVector = std::vector< HullMesh *>;
using Vec3Vector = std::vector< Vec3d >;

enum class Axes : uint32_t
{
    X_NEGATIVE,
    X_POSITIVE,

    Y_NEGATIVE,
    Y_POSITIVE,

    Z_NEGATIVE,
    Z_POSITIVE,
    LAST
};

uint32_t ALL_AXES =(1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5);

class VoxelMesh
{
public:
    VoxelMesh(void)
    {
    }

    ~VoxelMesh(void)
    {
#if RAYCAST_VOXELS
        SAFE_RELEASE(mRaycastMesh);
#endif
        SAFE_RELEASE(mVoxelTriMesh);
    }

    HullMesh *getMesh(void)
    {
        HullMesh *h = new HullMesh;

        const uint32_t *indices = mVoxelTriMesh->getIndices(h->mTriangleCount);
        const double *vertices = mVoxelTriMesh->getVertices(h->mVertexCount);

        h->mVertices = new double[h->mVertexCount*3];
        h->mIndices = new uint32_t[h->mTriangleCount*3];
        memcpy(h->mVertices,vertices,sizeof(double)*h->mVertexCount*3);
        memcpy(h->mIndices,indices,sizeof(uint32_t)*h->mTriangleCount*3);

        return h;
    }

    bool saveOBJ(const char *fname) const
    {
        bool ret = false;

        if ( mVoxelTriMesh )
        {
            ret = mVoxelTriMesh->saveOBJ(fname);
        }

        return ret;
    }

    const double * getVertices(uint32_t &vcount) const
    {
        vcount = 0;
        return mVoxelTriMesh ? mVoxelTriMesh->getVertices(vcount) : nullptr;
    }

    void addVoxels(const VoxelVector &voxels,Voxelize *v)
    {
        double scale = v->getScale();

        double bmin[3];
        double bmax[3];

        v->getBoundsMin(bmin);
        v->getBoundsMax(bmax);

        if ( mVoxelTriMesh == nullptr )
        {
            mVoxelTriMesh = VoxelTriMesh::create(bmin,bmax,scale);
        }
        if ( !voxels.empty() )
        {
            mVoxelTriMesh->addVoxels((uint32_t)voxels.size(),&voxels[0]);
        }
    }

    const uint32_t *getIndices(uint32_t &tcount)
    {
        tcount = 0;
        return mVoxelTriMesh ? mVoxelTriMesh->getIndices(tcount) : nullptr;
    }


    // Build a raycast mesh relative to this voxel mesh
    void buildRaycastMesh(void)
    {
#if RAYCAST_VOXELS
        SAFE_RELEASE(mRaycastMesh);
        uint32_t vcount;
        const double *vertices = getVertices(vcount);
        uint32_t tcount;
        const uint32_t *indices = getIndices(tcount);
        if ( vertices )
        {
            mRaycastMesh = RaycastMesh::createRaycastMesh(vcount,vertices,tcount,indices);
        }
#endif
    }

    inline void getPoint(const int32_t x,
                  const int32_t y,
                  const int32_t z,
                  const double scale,
                  const double *bmin,
                  double *dest)
    {
        double scaleHalf = scale*0.5;
        dest[0] = (double(x)*scale + bmin[0]+scaleHalf);
        dest[1] = (double(y)*scale + bmin[1]+scaleHalf);
        dest[2] = (double(z)*scale + bmin[2]+scaleHalf);
    }

    uint32_t findConcaveX(int32_t x1,
        int32_t y1,
        int32_t z1,
        int32_t x2,
        int32_t y2,
        int32_t z2,
        SplitLoc splitLocs[MAX_SPLIT_LOC],
        Voxelize *volume)
    {
#if RAYCAST_VOXELS
        double scale = volume->getScale();
        mScale = scale;
        double bmin[3];
        volume->getBoundsMin(bmin);

        double lastError = 0;
        int32_t xloc = 0;
        std::vector< double > errorMetrics;

        for (int32_t x = x1; x <= x2; x++)
        {
            double totalError = 0;

            for (int32_t y = y1; y <= y2; y++)
            {
                int32_t zbegin = z1 - 4;

                double p1[3];
                double p2[3];

                getPoint(x, y, zbegin, scale, bmin, p1);
                getPoint(x, y, z2 + 4, scale, bmin, p2);

                double hitloc1[3];
                double outT1;
                double faceSign1;
                bool hit1 = mRaycastMesh->raycast(p1, p2, outT1, faceSign1, hitloc1);

                double hitloc2[3];
                double outT2;
                double faceSign2;
                bool hit2 = mRaycastMesh->raycast(p2, p1, outT2, faceSign2, hitloc2);

                if (hit1 && hit2)
                {

                    double error = outT1 + outT2;

                    totalError += error;
                }
            }

            errorMetrics.push_back(totalError);

        }

        int32_t index = x1;

        int32_t xwid = (x2 + 1) - x1;

        size_t count = errorMetrics.size();
        double maxMeanDiff = 0;

        int32_t xbegin = x1+4;
        int32_t xend = x2-4;
        double scaleLimit = mScale*4; // delta must exceed the distance of 4 voxels to be considered statistically significant

        for (size_t i = 0; i < count; i++)
        {
            if (index < xbegin || index > xend )
            {
            }
            else
            {
                double prev = errorMetrics[i - 1];
                double next = errorMetrics[i + 1];

                double current = errorMetrics[i];

                if (prev > 0 && next > 0 && current > 0)
                {
                    double diff1 = fabs(current - prev);
                    double diff2 = fabs(current - next);
                    double meanDiff = (diff1 + diff2)*0.5;

                    if (meanDiff > scaleLimit )
                    {
                        for (uint32_t j=0; j<MAX_SPLIT_LOC; j++)
                        {
                            if ( meanDiff > splitLocs[j].mError )
                            {
                                splitLocs[j].mError = meanDiff;
                                splitLocs[j].mLoc = index;
                                break;
                            }
                        }
                    }
                }
            }
            index++;
        }

        uint32_t scount = 0;
        for (uint32_t i=0; i<MAX_SPLIT_LOC; i++)
        {
            if ( splitLocs[i].mError > 0 )
            {
                scount++;
            }
        }
        return scount;
#else
        return 0;
#endif
    }

    uint32_t findConcaveY(int32_t x1,
        int32_t y1,
        int32_t z1,
        int32_t x2,
        int32_t y2,
        int32_t z2,
        SplitLoc splitLocs[MAX_SPLIT_LOC],
        Voxelize *volume)
    {
#if RAYCAST_VOXELS
        double scale = volume->getScale();
        mScale = scale;
        double bmin[3];
        volume->getBoundsMin(bmin);

        double lastError = 0;
        int32_t yloc = 0;
        std::vector< double > errorMetrics;

        for (int32_t y = y1; y <= y2; y++)
        {
            double totalError = 0;

            for (int32_t x = x1; x <= x2; x++)
            {
                int32_t zbegin = z1 - 4;

                double p1[3];
                double p2[3];

                getPoint(x, y, zbegin, scale, bmin, p1);
                getPoint(x, y, z2 + 4, scale, bmin, p2);

                double hitloc1[3];
                double outT1;
                double faceSign1;
                bool hit1 = mRaycastMesh->raycast(p1, p2, outT1, faceSign1, hitloc1);

                double hitloc2[3];
                double outT2;
                double faceSign2;
                bool hit2 = mRaycastMesh->raycast(p2, p1, outT2, faceSign2, hitloc2);

                if (hit1 && hit2)
                {
                    double error = outT1 + outT2;

                    totalError += error;
                }
            }

            errorMetrics.push_back(totalError);

        }

        int32_t index = y1;

        int32_t ybegin = y1 + 4;
        int32_t yend = y2 - 4;
        double scaleLimit = mScale * 4; // delta must exceed the distance of 4 voxels to be considered statistically significant

        size_t count = errorMetrics.size();
        double maxMeanDiff = 0;
        for (size_t i = 0; i < count; i++)
        {
            if (index < ybegin || index > yend )
            {
            }
            else
            {
                double prev = errorMetrics[i - 1];
                double next = errorMetrics[i + 1];
                double current = errorMetrics[i];
                if (prev > 0 && next > 0 && current > 0)
                {
                    double diff1 = fabs(current - prev);
                    double diff2 = fabs(current - next);
                    double meanDiff = (diff1 + diff2)*0.5;
                    if (meanDiff > scaleLimit)
                    {
                        for (uint32_t j = 0; j < MAX_SPLIT_LOC; j++)
                        {
                            if (meanDiff > splitLocs[j].mError)
                            {
                                splitLocs[j].mError = meanDiff;
                                splitLocs[j].mLoc = index;
                                break;
                            }
                        }
                    }
                }
            }
            index++;
        }

        uint32_t scount = 0;
        for (uint32_t i = 0; i < MAX_SPLIT_LOC; i++)
        {
            if (splitLocs[i].mError > 0)
            {
                scount++;
            }
        }
        return scount;
#else
        return 0;
#endif
    }

    uint32_t findConcaveZ(int32_t x1,
        int32_t y1,
        int32_t z1,
        int32_t x2,
        int32_t y2,
        int32_t z2,
        SplitLoc splitLocs[MAX_SPLIT_LOC],
        Voxelize *volume)
    {
#if RAYCAST_VOXELS
        double scale = volume->getScale();
        mScale = scale;
        double bmin[3];
        volume->getBoundsMin(bmin);

        double lastError = 0;
        int32_t zloc = 0;
        class ErrorMetric
        {
        public:
            double  mError;
            double  mT1;
            double  mT2;
            double  mChange;
        };
        std::vector< ErrorMetric > errorMetrics;

        for (int32_t z = z1; z <= z2; z++)
        {
            double totalError = 0;
            double totalT1 = 0;
            double totalT2 = 0;

            for (int32_t y = y1; y <= y2; y++)
            {
                int32_t xbegin = x1 - 4;

                double p1[3];
                double p2[3];

                getPoint(xbegin, y, z, scale, bmin, p1);
                getPoint(x2+1, y, z, scale, bmin, p2);

                double hitloc1[3];
                double outT1;
                double faceSign1;
                bool hit1 = mRaycastMesh->raycast(p1, p2, outT1, faceSign1, hitloc1);

                double hitloc2[3];
                double outT2;
                double faceSign2;
                bool hit2 = mRaycastMesh->raycast(p2, p1, outT2, faceSign2, hitloc2);

                if (hit1 && hit2)
                {

                    double error = outT1 + outT2;


                    totalError += error;
                    totalT1+=outT1;
                    totalT2+=outT2;
                }
            }
            ErrorMetric e;
            e.mChange = 0;
            e.mError = totalError;
            e.mT1 = totalT1;
            e.mT2 = totalT2;
            errorMetrics.push_back(e);

        }

        int32_t index = z1;
        size_t count = errorMetrics.size();
        double maxMeanDiff = 0;


        zloc = (z1+z2)/2;

        int32_t zbegin = z1 + 4;
        int32_t zend = z2 - 4;
        double scaleLimit = mScale * 4; // delta must exceed the distance of 4 voxels to be considered statistically significant


        for (size_t i = 0; i < count; i++)
        {
            bool checkEdge = true;

            if (index < zbegin || index > zend)
            {
            }
            else
            {
                double prev = errorMetrics[i - 1].mError;
                double next = errorMetrics[i + 1].mError;
                double current = errorMetrics[i].mError;

                if ( prev > 0 && next > 0 && current > 0 )
                {
                    const ErrorMetric &e = errorMetrics[i];
                    double diff1 = fabs(current - prev);
                    double diff2 = fabs(current - next);
                    double meanDiff = (diff1 + diff2)*0.5;
                    errorMetrics[i].mChange = meanDiff;
                    if (meanDiff > scaleLimit)
                    {
                        for (uint32_t j = 0; j < MAX_SPLIT_LOC; j++)
                        {
                            if (meanDiff > splitLocs[j].mError)
                            {
                                splitLocs[j].mError = meanDiff;
                                splitLocs[j].mLoc = index;
                                break;
                            }
                        }
                    }
                }
            }

            index++;
        }


        uint32_t scount = 0;
        for (uint32_t i = 0; i < MAX_SPLIT_LOC; i++)
        {
            if (splitLocs[i].mError > 0)
            {
                scount++;
            }
        }
        return scount;
#else
        return 0;
#endif
    }

#if RAYCAST_VOXELS
    RaycastMesh             *mRaycastMesh{nullptr};
#endif
    double                  mScale{1};
    VoxelTriMesh            *mVoxelTriMesh{nullptr};
};

static uint32_t gPatchCount=0;

class ConvexPatch
{
public:
    ConvexPatch(const ConvexPatch &patch,Axes splitAxis,int32_t splitLoc)
    {
        gPatchCount++;
        mPatchCount = gPatchCount;
        mVolume = patch.mVolume;
        mMeshVolume = patch.mMeshVolume;
        mDepth = patch.mDepth+1;
        mMaxDepth = patch.mMaxDepth;
        mErrorThreshold = patch.mErrorThreshold;

        mMeshCenter[0] = patch.mMeshCenter[0];
        mMeshCenter[1] = patch.mMeshCenter[1];
        mMeshCenter[2] = patch.mMeshCenter[2];

        mAxis = splitAxis;

        switch ( splitAxis )
        {
            case Axes::X_NEGATIVE:
                mX1 = patch.mX1;
                mX2 = splitLoc;
                mY1 = patch.mY1;
                mY2 = patch.mY2;
                mZ1 = patch.mZ1;
                mZ2 = patch.mZ2;
                {
                    for (auto &i:patch.mInterior)
                    {
                        int32_t x,y,z;
                        i.getVoxel(x,y,z);
                        if ( x >= mX1 && x <= mX2 &&
                             y >= mY1 && y <= mY2 &&
                             z >= mZ1 && z <= mZ2 )
                        {
                            if ( x == splitLoc ) // if it is on the splitting plane, we treat it as a new surface
                            {
                                mNewSurface.push_back(i);
                            }
                            else
                            {
                                mInterior.push_back(i);
                            }
                        }
                    }
                    for (auto &i : patch.mSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mSurface.push_back(i);
                        }
                    }
                    for (auto &i : patch.mNewSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mNewSurface.push_back(i);
                        }
                    }
                }
                break;
            case Axes::X_POSITIVE:
                mX1 = splitLoc+1;
                mX2 = patch.mX2;
                mY1 = patch.mY1;
                mY2 = patch.mY2;
                mZ1 = patch.mZ1;
                mZ2 = patch.mZ2;
                {
                    for (auto &i : patch.mInterior)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            if (x == mX1) // if it is on the splitting plane, we treat it as a new surface
                            {
                                mNewSurface.push_back(i);
                            }
                            else
                            {
                                mInterior.push_back(i);
                            }
                        }
                    }
                    for (auto &i : patch.mSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mSurface.push_back(i);
                        }
                    }
                    for (auto &i : patch.mNewSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mNewSurface.push_back(i);
                        }
                    }
                }
                break;
            case Axes::Y_NEGATIVE:
                mY1 = patch.mY1;
                mY2 = splitLoc;
                mX1 = patch.mX1;
                mX2 = patch.mX2;
                mZ1 = patch.mZ1;
                mZ2 = patch.mZ2;
                {
                    for (auto &i : patch.mInterior)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            if (y == splitLoc) // if it is on the splitting plane, we treat it as a new surface
                            {
                                mNewSurface.push_back(i);
                            }
                            else
                            {
                                mInterior.push_back(i);
                            }
                        }
                    }
                    for (auto &i : patch.mSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mSurface.push_back(i);
                        }
                    }
                    for (auto &i : patch.mNewSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mNewSurface.push_back(i);
                        }
                    }
                }
                break;
            case Axes::Y_POSITIVE:
                mY1 = splitLoc+1;
                mY2 = patch.mY2;
                mX1 = patch.mX1;
                mX2 = patch.mX2;
                mZ1 = patch.mZ1;
                mZ2 = patch.mZ2;
                {
                    for (auto &i : patch.mInterior)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            if (y == mY1) // if it is on the splitting plane, we treat it as a new surface
                            {
                                mNewSurface.push_back(i);
                            }
                            else
                            {
                                mInterior.push_back(i);
                            }
                        }
                    }
                    for (auto &i : patch.mSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mSurface.push_back(i);
                        }
                    }
                    for (auto &i : patch.mNewSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mNewSurface.push_back(i);
                        }
                    }
                }

                break;
            case Axes::Z_NEGATIVE:
                mZ1 = patch.mZ1;
                mZ2 = splitLoc;
                mX1 = patch.mX1;
                mX2 = patch.mX2;
                mY1 = patch.mY1;
                mY2 = patch.mY2;
                {
                    for (auto &i : patch.mInterior)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            if (z == splitLoc) // if it is on the splitting plane, we treat it as a new surface
                            {
                                mNewSurface.push_back(i);
                            }
                            else
                            {
                                mInterior.push_back(i);
                            }
                        }
                    }
                    for (auto &i : patch.mSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mSurface.push_back(i);
                        }
                    }
                    for (auto &i : patch.mNewSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mNewSurface.push_back(i);
                        }
                    }
                }
                break;
            case Axes::Z_POSITIVE:
                mZ1 = splitLoc+1;
                mZ2 = patch.mZ2;
                mX1 = patch.mX1;
                mX2 = patch.mX2;
                mY1 = patch.mY1;
                mY2 = patch.mY2;
                {
                    for (auto &i : patch.mInterior)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            if (z == mZ1) // if it is on the splitting plane, we treat it as a new surface
                            {
                                mNewSurface.push_back(i);
                            }
                            else
                            {
                                mInterior.push_back(i);
                            }
                        }
                    }
                    for (auto &i : patch.mSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mSurface.push_back(i);
                        }
                    }
                    for (auto &i : patch.mNewSurface)
                    {
                        int32_t x, y, z;
                        i.getVoxel(x, y, z);
                        if (x >= mX1 && x <= mX2 &&
                            y >= mY1 && y <= mY2 &&
                            z >= mZ1 && z <= mZ2)
                        {
                            mNewSurface.push_back(i);
                        }
                    }
                }

                break;
            case Axes::LAST:
                break;
        }
        // Recompute the min-max bounding box which would be different after the split occurs
        mX1 = 0x7FFFFFFF;
        mY1 = 0x7FFFFFFF;
        mZ1 = 0x7FFFFFFF;
        mX2 = 0;
        mY2 = 0;
        mZ2 = 0;
        for (auto &i:mSurface)
        {
            minMax(i);
        }
        for (auto &i:mNewSurface)
        {
            minMax(i);
        }
        for (auto &i:mInterior)
        {
            minMax(i);
        }
        computeConvexHull();
    }

    void createVoxelMesh(void)
    {
        mVoxelMesh.addVoxels(mSurface, mVolume); // Add the surface voxels
        mVoxelMesh.addVoxels(mNewSurface, mVolume); // Add the new surface voxels
        mVoxelMesh.buildRaycastMesh(); // ok, build the raycasting mesh
    }


    void minMax(const Voxel &v)
    {
        int32_t x,y,z;
        v.getVoxel(x,y,z);
        if ( x < mX1 ) mX1 = x;
        if ( x > mX2 ) mX2 = x;
        if ( y < mY1 ) mY1 = y;
        if ( y > mY2 ) mY2 = y;
        if ( z < mZ1 ) mZ1 = z;
        if ( z > mZ2 ) mZ2 = z;
    }

    ConvexPatch(Voxelize *volume,double errorThreshold,uint32_t maxDepth,uint32_t minVoxelSize) : mVolume(volume), mMinVoxelSize(minVoxelSize), mMaxDepth(maxDepth), mErrorThreshold(errorThreshold)
    {
        gPatchCount = 0;
        mPatchCount = gPatchCount;

        uint32_t dims[3];
        volume->getDimensions(dims);

        mX2 = dims[0]-1;
        mY2 = dims[1]-1;
        mZ2 = dims[2]-1;

        //printf("Extracting voxels.\n");

        mVolume->getSurfaceVoxels(mSurface);
        mVolume->getInteriorVoxels(mInterior);

        uint32_t scount = (uint32_t)mSurface.size();
        uint32_t icount = (uint32_t)mInterior.size();

        //printf("Found:%d surface voxels and %d interior voxels total.\n", scount, icount);

        computeConvexHull();

        mMeshCenter[0] = mCenter[0];
        mMeshCenter[1] = mCenter[1];
        mMeshCenter[2] = mCenter[2];

        mMeshVolume = mHullVolume;
#if 0 // for debugging
        mVoxelMesh.saveOBJ("Surface.obj");
        if ( icount )
        {
            double bmin[3];
            double bmax[3];
            double scale = volume->getScale();
            volume->getBoundsMin(bmin);
            volume->getBoundsMax(bmax);
            VoxelTriMesh *tm = VoxelTriMesh::create(bmin,bmax,scale);
            tm->addVoxels(icount,&mInterior[0]);
            tm->saveOBJ("interior.obj");
            tm->release();
        }
#endif
    }

    HullMesh *getSurfaceMesh(void)
    {
        return mVoxelMesh.getMesh();
    }

    ~ConvexPatch(void)
    {
        delete mConvexHull;
    }

    void computeConvexHull(void)
    {
        createVoxelMesh();


        SplitLoc splitLocs[MAX_SPLIT_LOC];

        double scale = mVolume->getScale();

        quickhull::QuickHull *qh = quickhull::QuickHull::create();

        //printf("Computing convex hull on reduced point set.\n");

        quickhull::HullPoints hp;
        hp.mVertices = mVoxelMesh.getVertices(hp.mVertexCount);
        hp.mMaxHullVertices = hp.mVertexCount;
        hp.mMaxQuantizeVertexCount = REDUCE_POINT_COUNT;

        qh->computeConvexHull(hp);

        //qh->inflate(scale); 

        mConvexHull = new HullMesh(qh);
        mConvexHull->computeAABB();
        mConvexHull->inflateAABB(0.2);

        //printf("ConvexHull had %d input points, %d output points.\n", hp.mVertexCount, vcount);

        qh->release();

        vcd::fm_computeCentroid(mConvexHull->mVertexCount,mConvexHull->mVertices,mConvexHull->mCenter);
        mCenter[0] = (float)mConvexHull->mCenter[0];
        mCenter[1] = (float)mConvexHull->mCenter[1];
        mCenter[2] = (float)mConvexHull->mCenter[2];

        mHullVolume = vcd::fm_computeMeshVolume(mConvexHull->mVertices,mConvexHull->mTriangleCount,mConvexHull->mIndices);
        mConvexHull->mVolume = mHullVolume;

        double voxelCount = double( mSurface.size() + mInterior.size() + mNewSurface.size() );

        double voxelVolume = scale*scale*scale;
        mVoxelVolume = voxelCount*voxelVolume;

        mErrorPercentage = (fabs(mVoxelVolume-mHullVolume) / mVoxelVolume)*100.0f;
        //printf("HullVolume:%0.9f VoxelVolume:%0.9f Error:%0.2f%%\n", mHullVolume, mVoxelVolume, mErrorPercentage);

        
    }

    bool isFinished(uint32_t minVoxelSize) const
    {
        bool ret = false;

        int32_t dx = mX2-mX1;
        int32_t dy = mY2-mY1;
        int32_t dz = mZ2-mZ1;


        if ( mDepth >= mMaxDepth )
        {
            ret = true;
        }
        // If the current patch is less than 4 voxels wide on all three axes then
        // we stop. The reasoning here is that if the voxel patch is too tiny then the
        // convex hull around it does not have enough input data points for the error
        // percentage metric to be meaningful
        else if ( dx <= int32_t(minVoxelSize) && dy <= int32_t(minVoxelSize) && dz <= int32_t(minVoxelSize) )
        {
            ret = true; // it got too small
        }
        else if ( mErrorPercentage < mErrorThreshold ) // The percentage volume error of this patch is less than the maximum error allowe, we are done.
        {
            ret = true;
        }

        return ret;
    }

    // Figure out which axis to split on. We do this based on whichever
    // axes is the longest. However, we don't necessarily split on 
    // the midpoint. We search for the closest point of discontinuity on
    // that axis to the midpoint.
    Axes getDominantAxis(int32_t &splitLoc)
    {
        Axes ret = Axes::LAST;

        int32_t dx = mX2 - mX1;
        int32_t dy = mY2 - mY1;
        int32_t dz = mZ2 - mZ1;

        if ( dx >= dy && dx >= dz )
        {
            ret = Axes::X_NEGATIVE;
            splitLoc = (mX2+1+mX1)/2;
            SplitLoc splitLocs[MAX_SPLIT_LOC];
            uint32_t count = mVoxelMesh.findConcaveX(mX1,mY1,mZ1,mX2,mY2,mZ2,splitLocs,mVolume);
            int32_t nearestDistance = 0x7FFFFFFF;
            int32_t nearestLoc = 0;
            for (uint32_t i=0; i<count; i++)
            {
                int32_t diff = splitLocs[i].mLoc - splitLoc;
                if ( diff < 0 ) diff*=-1;
                if ( diff < nearestDistance )
                {
                    nearestDistance = diff;
                    nearestLoc = splitLocs[i].mLoc;
                }
            }
            if ( nearestLoc > 0 )
            {
                splitLoc = nearestLoc;
            }
        }
        else if ( dy >= dx && dy >= dz )
        {
            ret = Axes::Y_NEGATIVE;
            splitLoc = (mY2 + 1 + mY1) / 2;
            SplitLoc splitLocs[MAX_SPLIT_LOC];
            uint32_t count = mVoxelMesh.findConcaveY(mX1, mY1, mZ1, mX2, mY2, mZ2, splitLocs, mVolume);
            int32_t nearestDistance = 0x7FFFFFFF;
            int32_t nearestLoc = 0;
            for (uint32_t i = 0; i < count; i++)
            {
                int32_t diff = splitLocs[i].mLoc - splitLoc;
                if (diff < 0) diff *= -1;
                if (diff < nearestDistance)
                {
                    nearestDistance = diff;
                    nearestLoc = splitLocs[i].mLoc;
                }
            }
            if (nearestLoc > 0)
            {
                splitLoc = nearestLoc;
            }
        }
        else
        {
            ret = Axes::Z_NEGATIVE;
            splitLoc = (mZ2 + 1 + mZ1) / 2;
            SplitLoc splitLocs[MAX_SPLIT_LOC];
            uint32_t count = mVoxelMesh.findConcaveZ(mX1, mY1, mZ1, mX2, mY2, mZ2, splitLocs, mVolume);
            int32_t nearestDistance = 0x7FFFFFFF;
            int32_t nearestLoc = 0;
            for (uint32_t i = 0; i < count; i++)
            {
                int32_t diff = splitLocs[i].mLoc - splitLoc;
                if (diff < 0) diff *= -1;
                if (diff < nearestDistance)
                {
                    nearestDistance = diff;
                    nearestLoc = splitLocs[i].mLoc;
                }
            }
            if (nearestLoc > 0)
            {
                splitLoc = nearestLoc;
            }
        }
        return ret;
    }

    // We perform the convex patch splitting operation in a background thread so
    // that it can be done in parallel with other active convex patches.
    void performJob(void)
    {
        if ( isFinished(mMinVoxelSize) )
        {
        }
        else
        {
            int32_t splitLoc;
            Axes axis = getDominantAxis(splitLoc);
            switch (axis)
            {
            case Axes::X_NEGATIVE:
            case Axes::X_POSITIVE:
                {
                    mLeft = new ConvexPatch(*this, Axes::X_NEGATIVE, splitLoc);
                    mRight = new ConvexPatch(*this, Axes::X_POSITIVE, splitLoc);
                }
                break;
            case Axes::Y_NEGATIVE:
            case Axes::Y_POSITIVE:
                {
                    mLeft = new ConvexPatch(*this, Axes::Y_POSITIVE, splitLoc);
                    mRight = new ConvexPatch(*this, Axes::Y_NEGATIVE, splitLoc);
                }
                break;
            case Axes::Z_NEGATIVE:
            case Axes::Z_POSITIVE:
                {
                    mLeft = new ConvexPatch(*this, Axes::Z_POSITIVE, splitLoc);
                    mRight = new ConvexPatch(*this, Axes::Z_NEGATIVE, splitLoc);
                }
                break;
            case Axes::LAST:
                break;
            }
        }
    }

    double      mErrorPercentage{0};
    HullMesh  *mConvexHull{nullptr};
    Voxelize    *mVolume{nullptr};

    VoxelVector    mSurface;   // Surface voxels from the original source mesh
    VoxelVector    mNewSurface; // new surface voxels (which need to be considered for the convex hull) but were as a result of plane splitting; not the original source mesh
    VoxelVector    mInterior;  // Interior voxels

    double      mVoxelVolume{0};    // Volume of the voxels
    double      mHullVolume{0};     // Volume of the surrounding convex hull
    double      mMeshVolume{0};     // Volume of the convex hull surrounding the entire source mesh

    float       mCenter[3];
    float       mMeshCenter[3];

    int32_t mX1{0};
    int32_t mY1{0};
    int32_t mZ1{0};

    int32_t mX2{0};
    int32_t mY2{0};
    int32_t mZ2{0};


    Axes    mAxis{Axes::LAST};
    VoxelMesh   mVoxelMesh;
    uint32_t    mPatchCount{0};
    Axes        mSplitAxis;
    int32_t     mSplitLoc;
    uint32_t    mMinVoxelSize{4};
    uint32_t    mMaxDepth{256};
    double      mErrorThreshold{0};
    ConvexPatch *mLeft{nullptr};
    ConvexPatch *mRight{nullptr};
    uint32_t    mDepth{0};
};

using ConvexPatchList = std::list< ConvexPatch *>;

class VoxelizedConvexDecompositionImpl : public VoxelizedConvexDecomposition
{
public:
    VoxelizedConvexDecompositionImpl(bool isMultiThreaded)
    {
        mTasking = carb::getCachedInterface<ITasking>();
        
        // If multi-threaded option is enabled, then create the instance of
        // the multi-threaded job system
        if ( isMultiThreaded )
        {
            mJobSystem = JobSystem::create(8);
        }
    }

    virtual ~VoxelizedConvexDecompositionImpl(void)
    {
        stopThread(); // Stop our background thread
        SAFE_RELEASE(mJobSystem); // Release the job system
        delete []mVertices;     // Release our scratch copy of the source mesh vertices
        mVertices = nullptr;
        delete []mIndices;      // Release our scratch copy of the source mesh indices
        mIndices = nullptr;
        delete mVoxelizedMesh;
        mVoxelizedMesh = nullptr;
        releaseConvexPatches(); // Release all convex patches that were build
    }

    void releaseSourceMesh(void)
    {
        delete[]mVertices;      // Release our scratch copy of the source mesh vertices
        mVertices = nullptr;

        delete[]mIndices;   // Release our scratch copy of the source mesh indicies
        mIndices = nullptr;
        mParams.mTriangleCount = 0;
        mParams.mVertexCount = 0;
    }

    void voxelize(void)
    {
        releaseConvexPatches();
        //printf("Creating RaycastMesh instance\n");
        mRaycastMesh = vcd::RaycastMesh::createRaycastMesh(mParams.mVertexCount,mVertices,mParams.mTriangleCount,mIndices);
        SAFE_RELEASE(mVolume);
        //printf("Voxelizing source mesh.\n");
        mVolume = Voxelize::create();
        mVolume->voxelize(mRaycastMesh, mVertices, mParams.mVertexCount, mIndices, mParams.mTriangleCount, mParams.mVoxelResolution,mParams.mFillMode);
        mScale = (double)mVolume->getScale();
        mHalfScale = mScale * 0.5f;
        mVolume->getBoundsMin(mBmin);
        mVolume->getBoundsMax(mBmax);

        // Create the initial convex patch
        ConvexPatch *cp = new ConvexPatch(mVolume,mParams.mErrorPercentage,mParams.mMaxDepth,mParams.mMinVoxelSize);
        if ( mParams.mSaveVoxelizedMesh )
        {
            mVoxelizedMesh = cp->getSurfaceMesh();
        }
        mConvexPatches.push_back(cp);
        mMeshVolme = cp->mHullVolume; // the initial mesh volume
        mRootConvexHull = new HullMesh(*cp->mConvexHull);
        
    }

    virtual void release(void) final
    {
        delete this;
    }

    void initVec3(double *dest, uint32_t vindex, double x, double y, double z)
    {
        dest[vindex * 3 + 0] = x;
        dest[vindex * 3 + 1] = y;
        dest[vindex * 3 + 2] = z;
    }


    virtual bool stepVoxel(void) final
    {
        bool ret = false;

        if ( mConvexPatches.empty() )
        {
            //printf("Convex decomposition is completed. Produced %d hulls.\n", (uint32_t)mHullMeshes.size());
            ret = true;
        }
        else
        {
            ConvexPatchList oldList = mConvexPatches;

            for (auto &i:mConvexPatches)
            {
                if ( mJobSystem )
                {
                    mJobSystem->addJob(i,jobCallback);
                }
                else
                {
                    i->performJob();
                }
            }

            if ( mJobSystem )
            {
                mJobSystem->startJobs();
                mJobSystem->waitForJobsToComplete();
            }

            mConvexPatches.clear();
            for (auto &i:oldList)
            {
                ConvexPatch *cp = i;
                if ( cp->isFinished(mParams.mMinVoxelSize))
                {
                    HullMesh *hm = cp->mConvexHull;
                    cp->mConvexHull = nullptr;
                    if (hm)
                    {
                        mHullMeshes.push_back(hm);
                    }
                    else
                    {
                        //printf("This voxel fragment did not have a valid convex hull produced.\n");
                    }
                    delete cp;
                }
                else
                {
                    ConvexPatch *cp1 = cp->mLeft;
                    ConvexPatch *cp2 = cp->mRight;
                    if ( cp1 && cp2 && cp1->mConvexHull && cp2->mConvexHull)
                    {
                        mConvexPatches.push_back(cp1);
                        mConvexPatches.push_back(cp2);
                        delete cp;
                    }
                    else
                    {
                        //printf("After trying to split this convex patch one of the two could no longer produce a valid hull, so stopping.\n");
                        delete cp1;
                        delete cp2;
                        HullMesh *hm = cp->mConvexHull;
                        mHullMeshes.push_back(hm);
                        cp->mConvexHull = nullptr;
                        delete cp;
                    }
                }
            }
        }


        return ret;
    }

    // Release all convex patches and hull meshes and merge hulls
    void releaseConvexPatches(void)
    {
        for (auto &i:mConvexPatches)
        {
            delete i;
        }
        mConvexPatches.clear();

        for (auto &i:mHullMeshes)
        {
            delete i;
        }
        mHullMeshes.clear();
        SAFE_RELEASE(mMergeConvexHulls);
        delete mRootConvexHull;
        mRootConvexHull = nullptr;
    }

    // Merge convex hulls
    virtual uint32_t mergeConvexHulls(uint32_t maxHullCount,bool shrinkWrap) final
    {
        uint32_t ret = 0;

        if ( mMergeConvexHulls )
        {
            mMergeConvexHulls->release();
        }

        mMergeConvexHulls = MergeConvexHulls::create(mJobSystem);

        if ( !mHullMeshes.empty() )
        {
            if ( mHullMeshes.size() == 1 )
            {
                quickhull::QuickHull *qh = quickhull::QuickHull::create();
                HullMesh *h = mHullMeshes[0];
                quickhull::HullPoints hp;
                hp.mMaxHullVertices = mParams.mMaxHullVertices;
                hp.mMaxQuantizeVertexCount = mParams.mVertexCount;
                hp.mVertices = mVertices;
                hp.mVertexCount = mParams.mVertexCount;
                qh->computeConvexHull(hp);

                delete h;
                h = new HullMesh(qh);
                mHullMeshes[0] = h;
                mMergeConvexHulls->addConvexHull(*h);
                double recipScale = 1.0 / mVertexScale;
                mMergeConvexHulls->finalizeHulls(recipScale);

                qh->release();
                ret = 1;
            }
            else
            {
                for (auto &i:mHullMeshes)
                {
                    mMergeConvexHulls->addConvexHull(*i);
                }
                ret = mMergeConvexHulls->mergeConvexHulls(maxHullCount, mMeshVolme);
                double recipScale = 1.0 / mVertexScale;
                ret = mMergeConvexHulls->finalizeResults(mParams.mMaxHullVertices, mRaycastMesh, mScale * 4, shrinkWrap, recipScale);
            }


        }
        return ret;
    }

    void stopThread(void)
    {
        if (mThread.valid())
        {
            cancel();
            mThread.wait();
        }
    }

    // Given these input parameters start convex decomposition
    virtual void process(const Params &p) final
    {
        if ( !mIsFinished )
        {
            printf("Can't process job, one is currently running.\n");
            return;
        }

        stopThread(); // stop old thread

        releaseSourceMesh();

        mCancel = false; // 
        mParams = p;
        mVertices = new double[p.mVertexCount*3];
        mIndices = new uint32_t[p.mTriangleCount*3];
        memcpy(mVertices,p.mVertices,sizeof(double)*p.mVertexCount*3);
        memcpy(mIndices,p.mIndices,sizeof(uint32_t)*p.mTriangleCount*3);

        double bmin[3];
        double bmax[3];
        vcd::fm_initMinMax(bmin,bmax);
        for (uint32_t i=0; i<p.mVertexCount; i++)
        {
            const double *point = &mVertices[i*3];
            vcd::fm_minmax(point,bmin,bmax);
        }
        double diag = vcd::fm_distance(bmax,bmin);
        // We scale all of the vertices to a normalized distance
        mVertexScale = 1000.0 / diag;
        for (uint32_t i=0; i<p.mVertexCount; i++)
        {
            double *point = &mVertices[i * 3];
            point[0]*=mVertexScale;
            point[1]*=mVertexScale;
            point[2]*=mVertexScale;
        }

        mIsFinished = false;
        mThread = mTasking->addTask(carb::tasking::Priority::eDefault, {}, [this]() 
        { 
            runThread(); 
        });
    }

    // Compute convex decomposition in a background thread
    void runThread(void)
    {
#if SHOW_TOTAL_TIME
        ScopedTime total("Total time spent performing convex decomposition.");
#endif
        {
#if SHOW_TIME
            ScopedTime st("Voxelizing");
#endif
            voxelize();
        }
        if ( mVoxelizedMesh )
        {
        }
        else
        {
            if ( !mCancel )
            {
                //printf("Extracting Convex Patches\n");
    #if SHOW_TIME
                ScopedTime st("building convex patches");
    #endif
                while ( !mCancel )
                {
                    if ( stepVoxel() )
                    {
                        break;
                    }
                }
            }
            if ( !mCancel )
            {
    #if SHOW_TIME
                ScopedTime st("merging convex hulls");
    #endif
                mergeConvexHulls(mParams.mMaxConvexHulls,mParams.mShrinkWrap);
            }
            if ( mCancel )
            {
                printf("Convex Decomposition operation canceled.\n");
            }
            else
            {
                //printf("Convex Decomposition operation completed.\n");
            }
        }
        if ( mParams.mCallback )
        {
            mParams.mCallback->notifyVoxelizedConvexDecompositionComplete();
        }
        mIsFinished = true;
    }

    // Return true if this operation is complete
    virtual bool isFinished(void) const final
    {
        return mIsFinished;
    }

    // Raise the cancel flag
    virtual void cancel(void) final
    {
        mCancel = true;
    }

    // Wait
    virtual void wait(uint32_t ms) final
    {
        if (mThread.valid())
            mThread.wait_for(std::chrono::milliseconds(ms));
    }

    // Return the number of merged convex hulls
    virtual uint32_t getMergedConvexHullCount(void) const final
    {
        uint32_t ret = 0;

        if ( mIsFinished )
        {
            if ( mVoxelizedMesh )
            {
                ret = 1;
            }
            else
            {
                ret = mMergeConvexHulls->getMergeConvexHullCount();
            }
        }

        return ret;
    }

    // Retrieve the merged convex hull as a simple mesh
    virtual const SimpleMesh * getMergedConvexHull(uint32_t index) const final
    {
        const SimpleMesh *ret = nullptr;

        if ( mIsFinished )
        {
            if ( mVoxelizedMesh )
            {
                printf("VoxelizedMesh has:%d triangles.\n", mVoxelizedMesh->mTriangleCount);
                ret = mVoxelizedMesh;
            }
            else if ( mMergeConvexHulls )
            {
                ret = mMergeConvexHulls->getMergedConvexHull(index);
            }
        }

        return ret;
    }

    // returns the convex hull which surrounds the entire source mesh
    virtual const SimpleMesh *getRootConvexHull(void) const final
    {
        return mRootConvexHull;
    }

    Params          mParams;                    // Parameters to use when performing convex decomposition
    double          *mVertices{nullptr};        // Vertices of the source mesh
    uint32_t        *mIndices{nullptr};         // Indices of the source mesh
    std::atomic<bool> mCancel{false};           // Whether or not the user has requested the operation be canceled
    std::atomic<bool> mIsFinished{true};        // Indicates the background task is finished
    vcd::RaycastMesh	*mRaycastMesh{nullptr}; // A raycast mesh structure for the original source mesh (used for raycast fill)
    Voxelize        *mVolume{nullptr};          // The voxel representation of the source mesh
    HullMesh        *mVoxelizedMesh{nullptr};   // 
    HullMeshVector  mHullMeshes;                // The convex hulls produced as part of the convex decomposition (before merging)
    HullMesh        *mRootConvexHull{nullptr};

    double              mScale{1};              // The size of a single voxel
    double               mHalfScale{1};         // Half the size of a single voxel
    double              mBmin[3];               // World space minimum for the voxel volume
    double               mBmax[3];              // World space maximum for the voxel volume
    double           mMeshVolme{1};             // The volume of the convex hull surrounding the entire source mesh
    MergeConvexHulls    *mMergeConvexHulls{nullptr};    // The interface to merge convex hulls
    ConvexPatchList    mConvexPatches;          // The set of convex patches were are currently evaluating
    Future<>            mThread;                // Thread "pointer"
    JobSystem           *mJobSystem{nullptr};   // job system
    double              mVertexScale{1};

    ITasking*           mTasking;
};

void jobCallback(void *userPtr)
{
    ConvexPatch *cp = (ConvexPatch *)userPtr;
    cp->performJob();
}

VoxelizedConvexDecomposition *VoxelizedConvexDecomposition::create(bool isMultiThreaded)
{
    auto ret = new VoxelizedConvexDecompositionImpl(isMultiThreaded);
    return static_cast< VoxelizedConvexDecomposition *>(ret);
}


}
