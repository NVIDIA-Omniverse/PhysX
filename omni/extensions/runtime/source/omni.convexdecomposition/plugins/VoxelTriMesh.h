// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

// Helper function to convert a set of voxels
// into an indexed triangle mesh
#include "SimpleMesh.h"
#include "Voxel.h"

namespace vcd
{

class VoxelTriMesh
{
public:
    static VoxelTriMesh* create(const double* bmin, const double* bmax, double scale);

    virtual void addVoxels(uint32_t voxelCount, const Voxel* voxels) = 0;

    virtual const double* getVertices(uint32_t& vcount) const = 0;
    virtual const uint32_t* getIndices(uint32_t& tcount) const = 0;

    // For debugging purposes, save it as a wavefront obj
    virtual bool saveOBJ(const char* fname) const = 0;

    virtual void release(void) = 0;

protected:
    virtual ~VoxelTriMesh(void)
    {
    }
};

} // namespace vcd
