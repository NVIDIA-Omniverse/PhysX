// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

namespace vcd
{

class SimplePolygon
{
public:
    uint32_t mStartIndex{ 0 };
    uint32_t mPointCount{ 0 };
    double mPlaneEquation[4]{};
};

class SimpleMesh
{
public:
    SimpleMesh(void)
    {
    }

    SimpleMesh(const SimpleMesh& sm)
    {
        mOwnAllocation = true; // we allocated this
        mVertexCount = sm.mVertexCount; // assign the vertex count
        if (mVertexCount)
        {
            mVertices = new double[mVertexCount * 3];
            memcpy(mVertices, sm.mVertices, sizeof(double) * mVertexCount * 3); // allocate and copy the vertices
        }
        mTriangleCount = sm.mTriangleCount; // assign the triangle count
        if (mTriangleCount)
        {
            mIndices = new uint32_t[mTriangleCount * 3];
            memcpy(mIndices, sm.mIndices, sizeof(uint32_t) * mTriangleCount * 3); // allocate and copy the vertices
        }
        mPolygonCount = sm.mPolygonCount;
        mPolygonIndexCount = sm.mPolygonIndexCount;
        if (mPolygonCount)
        {
            mPolygonData = new SimplePolygon[mPolygonCount];
            memcpy((void*)mPolygonData, sm.mPolygonData, sizeof(SimplePolygon) * mPolygonCount);
        }
        if (mPolygonIndexCount)
        {
            mPolygonIndices = new uint32_t[mPolygonIndexCount];
            memcpy((void*)mPolygonIndices, sm.mPolygonIndices, sizeof(uint32_t) * mPolygonIndexCount);
        }

        mBmin[0] = sm.mBmin[0];
        mBmin[1] = sm.mBmin[1];
        mBmin[2] = sm.mBmin[2];

        mBmax[0] = sm.mBmax[0];
        mBmax[1] = sm.mBmax[1];
        mBmax[2] = sm.mBmax[2];

        mCenter[0] = sm.mCenter[0];
        mCenter[1] = sm.mCenter[1];
        mCenter[2] = sm.mCenter[2];

        mVolume = sm.mVolume;

        mMeshId = sm.mMeshId;
    }

    SimpleMesh(uint32_t vcount, const double* vertices, uint32_t tcount, const uint32_t* indices)
    {
        mOwnAllocation = true;
        mVertexCount = vcount;
        if (mVertexCount)
        {
            mVertices = new double[mVertexCount * 3];
            memcpy(mVertices, vertices, sizeof(double) * mVertexCount * 3);
        }
        mTriangleCount = tcount;
        if (mTriangleCount)
        {
            mIndices = new uint32_t[mTriangleCount * 3];
            memcpy(mIndices, indices, sizeof(uint32_t) * mTriangleCount * 3);
        }
    }

    ~SimpleMesh(void)
    {
        releaseMeshData();
    }

    void releaseMeshData(void)
    {
        if (mOwnAllocation)
        {
            delete[] mVertices;
            delete[] mIndices;
            delete[] mPolygonData;
            delete[] mPolygonIndices;
            mVertices = nullptr;
            mIndices = nullptr;
            mPolygonData = nullptr;
            mPolygonIndices = nullptr;
            mVertexCount = 0;
            mTriangleCount = 0;
            mPolygonCount = 0;
            mOwnAllocation = false;
        }
    }


    void scaleVertices(double scale)
    {
        for (uint32_t i = 0; i < mVertexCount; i++)
        {
            double* p = &mVertices[i * 3];
            p[0] *= scale;
            p[1] *= scale;
            p[2] *= scale;
        }
        // Recompute the plane equation for each polygon relative to the new scale.
        if (mPolygonCount)
        {
            SimplePolygon* p = (SimplePolygon*)mPolygonData;
            // Scale the Plane-D co-efficient
            for (uint32_t i = 0; i < mPolygonCount; i++)
            {
                p->mPlaneEquation[3] *= scale;
                p++;
            }
        }
    }

    bool mOwnAllocation{ false }; // true if we allocated these buffers and are therefore responsible for deleting them
    uint32_t mMeshId{ 0 }; // optional id to uniquely identify this mesh
    uint32_t mVertexCount{ 0 };
    uint32_t mTriangleCount{ 0 };
    double* mVertices{ nullptr };
    uint32_t* mIndices{ nullptr };
    double mCenter[3];
    double mVolume{ 0 };
    double mBmin[3];
    double mBmax[3];
    uint32_t mPolygonCount{ 0 }; // Number of polygons
    const SimplePolygon* mPolygonData{ nullptr }; // Data associated with each polygon
    uint32_t mPolygonIndexCount{ 0 }; // number of polygon indices total
    uint32_t* mPolygonIndices{ nullptr }; // The vertex indices for each polygon
};

} // namespace vcd
