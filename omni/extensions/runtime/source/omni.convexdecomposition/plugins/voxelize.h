// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

#include "Voxel.h"
#include "VoxelFillMode.h"

namespace vcd
{

class RaycastMesh;

enum class VoxelValue : uint8_t
{
    PRIMITIVE_UNDEFINED = 0,
    PRIMITIVE_OUTSIDE_SURFACE_TOWALK = 1,
    PRIMITIVE_OUTSIDE_SURFACE = 2,
    PRIMITIVE_INSIDE_SURFACE = 3,
    PRIMITIVE_ON_SURFACE = 4
};


class Voxelize
{
public:
    static Voxelize* create(void);

    virtual uint32_t voxelize(vcd::RaycastMesh* raycastMesh,
                              const double* const vertices,
                              const uint32_t vertexCount,
                              const uint32_t* indices,
                              const uint32_t triangleCount,
                              const uint32_t resolution,
                              VoxelFillMode fillMode) = 0;

    virtual double getScale(void) const = 0;

    virtual bool getBoundsMin(double bmin[3]) const = 0;
    virtual bool getBoundsMax(double bmax[3]) const = 0;
    virtual bool getDimensions(uint32_t dim[3]) const = 0;
    virtual uint8_t getVoxel(uint32_t x, uint32_t y, uint32_t z) const = 0;
    virtual void setVoxel(uint32_t x, uint32_t y, uint32_t z, uint8_t value) = 0;

    virtual bool getVoxel(const double* pos, uint32_t& x, uint32_t& y, uint32_t& z) const = 0;

    virtual bool getSurfaceVoxels(VoxelVector& surfaceVoxels) = 0;
    virtual bool getInteriorVoxels(VoxelVector& interiorVoxels) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~Voxelize(void)
    {
    }
};

} // namespace vcd
