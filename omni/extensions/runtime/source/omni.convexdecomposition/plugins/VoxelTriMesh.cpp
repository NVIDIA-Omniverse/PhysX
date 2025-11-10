// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "VoxelTriMesh.h"
#include <assert.h>
#ifdef _MSC_VER
#pragma warning(disable:4100 4996)
#endif

namespace vcd
{

class Vec3Double
{
public:
    Vec3Double(void) {};
    Vec3Double(double _x, double _y, double _z) : x(_x), y(_y), z(_z)
    {
    }
    Vec3Double(const double v[3])
    {
        x = double(v[0]);
        y = double(v[1]);
        z = double(v[2]);
    }
    Vec3Double(const float v[3])
    {
        x = double(v[0]);
        y = double(v[1]);
        z = double(v[2]);
    }

    double   x;
    double   y;
    double   z;
};

using PositionMap = std::unordered_map< uint32_t,uint32_t >;

class VoxelTriMeshImpl : public VoxelTriMesh
{
public:
    VoxelTriMeshImpl(const double *bmin, const double *bmax, double scale) 
    {
        mBmin[0] = bmin[0];
        mBmin[1] = bmin[1];
        mBmin[2] = bmin[2];

        mBmax[0] = bmax[0];
        mBmax[1] = bmax[1];
        mBmax[2] = bmax[2];

        mMeshMin[0] = bmin[0] - scale * 2;
        mMeshMin[1] = bmin[1] - scale * 2;
        mMeshMin[2] = bmin[2] - scale * 2;

        mMeshMax[0] = bmax[0] + scale * 2;
        mMeshMax[1] = bmax[1] + scale * 2;
        mMeshMax[2] = bmax[2] + scale * 2;

        mScaleX = mMeshMax[0] - mMeshMin[0];
        mScaleY = mMeshMax[1] - mMeshMin[1];
        mScaleZ = mMeshMax[2] - mMeshMin[2];

        mScaleX = 1024.0 / mScaleX;
        mScaleY = 1024.0 / mScaleY;
        mScaleZ = 1024.0 / mScaleZ;


        mScale = scale;
    }

    virtual ~VoxelTriMeshImpl(void)
    {
    }


    virtual void addVoxels(uint32_t voxelCount, const Voxel *voxels) final
    {

        for (uint32_t i=0; i<voxelCount; i++)
        {
            const Voxel &v = *voxels++;
            double vmin[3];
            double vmax[3];

            int32_t x, y, z;
            v.getVoxel(x, y, z);

            vmin[0] = double(x)*mScale + mBmin[0];
            vmin[1] = double(y)*mScale + mBmin[1];
            vmin[2] = double(z)*mScale + mBmin[2];

            vmax[0] = vmin[0] + mScale;
            vmax[1] = vmin[1] + mScale;
            vmax[2] = vmin[2] + mScale;

            addBox(vmin, vmax);
        }
    }

    inline void addBox(const Vec3Double &bmin, const Vec3Double &bmax)
    {
        Vec3Double box[8];

        box[0] = Vec3Double(bmin.x, bmin.y, bmin.z);
        box[1] = Vec3Double(bmax.x, bmin.y, bmin.z);
        box[2] = Vec3Double(bmax.x, bmax.y, bmin.z);
        box[3] = Vec3Double(bmin.x, bmax.y, bmin.z);
        box[4] = Vec3Double(bmin.x, bmin.y, bmax.z);
        box[5] = Vec3Double(bmax.x, bmin.y, bmax.z);
        box[6] = Vec3Double(bmax.x, bmax.y, bmax.z);
        box[7] = Vec3Double(bmin.x, bmax.y, bmax.z);

        addTri(box, 2, 1, 0);
        addTri(box, 3, 2, 0);

        addTri(box, 7, 2, 3);
        addTri(box, 7, 6, 2);

        addTri(box, 5, 1, 2);
        addTri(box, 5, 2, 6);

        addTri(box, 5, 4, 1);
        addTri(box, 4, 0, 1);

        addTri(box, 4, 6, 7);
        addTri(box, 4, 5, 6);

        addTri(box, 4, 7, 0);
        addTri(box, 7, 3, 0);

    }

    inline void addTri(const Vec3Double *box, uint32_t i1, uint32_t i2, uint32_t i3)
    {
        const Vec3Double &p1 = box[i1];
        const Vec3Double &p2 = box[i2];
        const Vec3Double &p3 = box[i3];
        addTriangle(&p1.x, &p2.x, &p3.x);
    }

    inline void addTriangle(const double *p1, const double *p2, const double *p3)
    {
        uint32_t i1 = getVertexIndex(p1);
        uint32_t i2 = getVertexIndex(p2);
        uint32_t i3 = getVertexIndex(p3);

        mIndices.push_back(i1);
        mIndices.push_back(i2);
        mIndices.push_back(i3);
    }

    inline uint32_t getIndex(const double *v)
    {
        assert(v[0] > mMeshMin[0] && v[0] < mMeshMax[0]);
        assert(v[1] > mMeshMin[1] && v[1] < mMeshMax[1]);
        assert(v[2] > mMeshMin[2] && v[2] < mMeshMax[2]);

        uint32_t x = (uint32_t)((v[0] - mMeshMin[0])*mScaleX);
        uint32_t y = (uint32_t)((v[1] - mMeshMin[1])*mScaleY);
        uint32_t z = (uint32_t)((v[2] - mMeshMin[2])*mScaleZ);

        assert(x < 1024);
        assert(y < 1024);
        assert(z < 1024);
        uint32_t index = (x << 20) | (y << 10) | z;
        return index;
    }

    inline uint32_t getVertexIndex(const double *v)
    {
        uint32_t ret;
        uint32_t index = getIndex(v);
        PositionMap::iterator found = mPositionMap.find(index);
        if (found == mPositionMap.end())
        {
            ret = (uint32_t)mVertices.size();
            mPositionMap[index] = ret;
            mVertices.push_back(Vec3Double(v));
        }
        else
        {
            ret = (*found).second;
        }
        return ret;
    }

    virtual const double *getVertices(uint32_t &vcount) const final
    {
        const double *ret = nullptr;
        vcount = 0;

        if ( !mVertices.empty() )
        {
            vcount = (uint32_t)mVertices.size();
            ret = &mVertices[0].x;
        }

        return ret;
    }

    virtual const uint32_t *getIndices(uint32_t &tcount) const final
    {
        const uint32_t *ret = nullptr;
        tcount = 0;

        if ( !mIndices.empty() )
        {
            tcount = uint32_t(mIndices.size())/3;
            ret = &mIndices[0];
        }

        return ret;
    }

    virtual void release(void) final
    {
        delete this;
    }

    // For debugging purposes, save it as a wavefront obj
    virtual bool saveOBJ(const char *fname) const final
    {
        bool ret = false;

        uint32_t vcount;
        uint32_t tcount;
        const double *vertices  = getVertices(vcount);
        const uint32_t *indices = getIndices(tcount);

        if ( vertices && indices )
        {
            FILE *fph = fopen(fname,"wb");
            if ( fph )
            {
                printf("Saving:%s with %d vertices and %d triangles.\n", fname, vcount, tcount);
                for (uint32_t i=0; i<vcount; i++)
                {
                    const double *p = &vertices[i*3];
                    fprintf(fph,"v %0.9f %0.9f %0.9f\n", p[0], p[1], p[2]);
                }
                for (uint32_t i=0; i<tcount;i++)
                {
                    uint32_t i1 = indices[i*3+0]+1;
                    uint32_t i2 = indices[i*3+1]+1;
                    uint32_t i3 = indices[i*3+2]+1;
                    assert(i1<=vcount);
                    assert(i2<=vcount);
                    assert(i3<=vcount);
                    fprintf(fph,"f %d %d %d\n", i1, i2, i3);
                }
                fclose(fph);
                ret = true;
            }
        }

        return ret;
    }


    double  mBmin[3];
    double  mBmax[3];
    double  mScale;

    std::vector< Vec3Double >    mVertices;
    std::vector< uint32_t > mIndices;
    double                  mMeshMin[3];
    double                  mMeshMax[3];
    double                  mScaleX{ 1 };
    double                  mScaleY{ 1 };
    double                  mScaleZ{ 1 };
    PositionMap             mPositionMap;
};

VoxelTriMesh *VoxelTriMesh::create(const double *bmin, const double *bmax, double scale)
{
    auto ret = new VoxelTriMeshImpl(bmin,bmax,scale);
    return static_cast< VoxelTriMesh *>(ret);
}


}
