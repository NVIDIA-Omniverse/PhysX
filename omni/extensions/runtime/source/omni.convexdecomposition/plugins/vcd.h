// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

// Exposes the API for voxelized convex decomposition
#include "SimpleMesh.h"
#include "VoxelFillMode.h"

namespace vcd
{

class NotifyVoxelizedConvexDecomposition
{
public:
    virtual void notifyVoxelizedConvexDecompositionComplete(void) = 0;
};

class Params
{
public:
    uint32_t mVertexCount{ 0 }; // Number of vertices in the source mesh
    uint32_t mTriangleCount{ 0 }; // Number of triangles in the source mesh
    const double* mVertices{ nullptr }; // Pointer to the vertices as doubles x,y,z
    const uint32_t* mIndices{ nullptr }; // Pointer to the triangle indices as i1,i2,3...
    uint32_t mVoxelResolution{ 500000 }; // The voxel resolution to use
    uint32_t mMaxHullVertices{ 60 }; // The maximum number of vertices to use when generating convex hull results
    uint32_t mMaxConvexHulls{ 64 }; // The maximum number of convex hulls to use in the results
    double mErrorPercentage{ 10 }; // The allowed error percentage
    VoxelFillMode mFillMode{ VoxelFillMode::eFloodFill }; //
    uint32_t mMaxDepth{ 14 }; // maximum recursion depth
    uint32_t mMinVoxelSize{ 4 }; // once a convex patch is smaller than this voxel size, we stop recursing
    bool mShrinkWrap{ false }; // whether or not to 'shrinkwrap' the results. False by default
    bool mSaveVoxelizedMesh{ false }; // if true, doesn't actually do a convex decomposition, just converts the input
                                      // mesh into a voxelized mesh representation
    NotifyVoxelizedConvexDecomposition* mCallback{ nullptr }; // optional callback notification when decomposition is
                                                              // complete
};


class VoxelizedConvexDecomposition
{
public:
    static VoxelizedConvexDecomposition* create(bool isMultiThreaded);

    virtual void process(const Params& p) = 0;

    virtual uint32_t getMergedConvexHullCount(void) const = 0;

    virtual const SimpleMesh* getMergedConvexHull(uint32_t index) const = 0;

    // returns the convex hull which surrounds the entire source mesh
    virtual const SimpleMesh* getRootConvexHull(void) const = 0;

    virtual bool isFinished(void) const = 0;

    virtual void cancel(void) = 0;

    virtual void wait(uint32_t ms) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~VoxelizedConvexDecomposition(void)
    {
    }
};

} // namespace vcd
