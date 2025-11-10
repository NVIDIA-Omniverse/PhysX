// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "RaycastMesh.h"
#include "aabb.h"
#include "FM.h"
#include <math.h>
#include <assert.h>

namespace vcd
{

class MyRaycastMesh : public vcd::RaycastMesh
{
public:
    template <class T>
    MyRaycastMesh(uint32_t vcount, const T* vertices, uint32_t tcount, const uint32_t* indices)
    {
        mVcount = vcount;
        mVertices = new double[mVcount * 3];
        for (uint32_t i = 0; i < mVcount; i++)
        {
            mVertices[i * 3 + 0] = vertices[0];
            mVertices[i * 3 + 1] = vertices[1];
            mVertices[i * 3 + 2] = vertices[2];
            vertices += 3;
        }
        mTcount = tcount;
        mIndices = new uint32_t[mTcount * 3];
        for (uint32_t i = 0; i < mTcount; i++)
        {
            mIndices[i * 3 + 0] = indices[0];
            mIndices[i * 3 + 1] = indices[1];
            mIndices[i * 3 + 2] = indices[2];
            indices += 3;
        }
        mAABBTree = vcd::AABBTree::create(mVertices, mTcount, mIndices, mTcount);
    }


    ~MyRaycastMesh(void)
    {
        delete[] mVertices;
        delete[] mIndices;
        mAABBTree->release();
    }

    virtual void release(void)
    {
        delete this;
    }

    // Uses high speed AABB raycasting
    virtual bool raycast(const double* start,
                         const double* dir,
                         double& outT,
                         double& u,
                         double& v,
                         double& w,
                         double& faceSign,
                         uint32_t& faceIndex) const final
    {
        return mAABBTree->raycast(start, dir, outT, u, v, w, faceSign, faceIndex);
    }

    virtual bool raycast(const double* start, const double* to, double& outT, double& faceSign, double* hitLocation) const final
    {
        double dir[3];
        dir[0] = to[0] - start[0];
        dir[1] = to[1] - start[1];
        dir[2] = to[2] - start[2];
        double distance = vcd::fm_normalize(dir);
        double u, v, w;
        uint32_t faceIndex;
        bool hit = mAABBTree->raycast(start, dir, outT, u, v, w, faceSign, faceIndex);
        if (hit && hitLocation)
        {
            hitLocation[0] = start[0] + dir[0] * outT;
            hitLocation[1] = start[1] + dir[1] * outT;
            hitLocation[2] = start[2] + dir[2] * outT;
        }
        if (hit && outT > distance)
        {
            hit = false;
        }
        return hit;
    }

    virtual bool getClosestPointWithinDistance(const double* point, double maxDistance, double* closestPoint) final
    {
        return mAABBTree->getClosestPointWithinDistance(point, maxDistance, closestPoint);
    }


    vcd::AABBTree* mAABBTree{ nullptr };
    uint32_t mVcount;
    double* mVertices;
    uint32_t mTcount;
    uint32_t* mIndices;
};

}; // namespace RAYCAST_MESH


using namespace vcd;

namespace vcd
{

RaycastMesh* RaycastMesh::createRaycastMesh(uint32_t vcount, // The number of vertices in the source triangle mesh
                                            const double* vertices, // The array of vertex positions in the format
                                                                    // x1,y1,z1..x2,y2,z2.. etc.
                                            uint32_t tcount, // The number of triangles in the source triangle mesh
                                            const uint32_t* indices) // The triangle indices in the format of i1,i2,i3
                                                                     // ... i4,i5,i6, ...
{
    MyRaycastMesh* m = new MyRaycastMesh(vcount, vertices, tcount, indices);
    return static_cast<RaycastMesh*>(m);
}

RaycastMesh* RaycastMesh::createRaycastMesh(uint32_t vcount, // The number of vertices in the source triangle mesh
                                            const float* vertices, // The array of vertex positions in the format
                                                                   // x1,y1,z1..x2,y2,z2.. etc.
                                            uint32_t tcount, // The number of triangles in the source triangle mesh
                                            const uint32_t* indices) // The triangle indices in the format of i1,i2,i3
                                                                     // ... i4,i5,i6, ...
{
    MyRaycastMesh* m = new MyRaycastMesh(vcount, vertices, tcount, indices);
    return static_cast<RaycastMesh*>(m);
}


} // namespace VHACD
