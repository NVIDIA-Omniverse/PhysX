// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>

namespace vcd
{

#define VOXEL_BITS 10
#define VOXEL_BITS2 20
#define VOXEL_BIT_MASK ((1 << VOXEL_BITS) - 1)

class Voxel
{
public:
    Voxel(void)
    {
    }
    inline Voxel(uint32_t index) : mVoxel(index)
    {
    }
    inline Voxel(uint32_t x, uint32_t y, uint32_t z)
    {
        mVoxel = (x << VOXEL_BITS2) | (y << VOXEL_BITS) | z;
    }
    inline bool operator==(const Voxel& v) const
    {
        return v.mVoxel == mVoxel;
    }

    inline void getVoxel(uint32_t& x, uint32_t& y, uint32_t& z) const
    {
        x = (mVoxel >> VOXEL_BITS2);
        y = (mVoxel >> VOXEL_BITS) & VOXEL_BIT_MASK;
        z = mVoxel & VOXEL_BIT_MASK;
    }

    inline void getVoxel(int32_t& x, int32_t& y, int32_t& z) const
    {
        x = (int32_t)(mVoxel >> VOXEL_BITS2);
        y = (int32_t)((mVoxel >> VOXEL_BITS) & VOXEL_BIT_MASK);
        z = (int32_t)(mVoxel & VOXEL_BIT_MASK);
    }

    inline uint32_t getX(void) const
    {
        return (mVoxel >> VOXEL_BITS2);
    }

    inline uint32_t getY(void) const
    {
        return (mVoxel >> VOXEL_BITS) & VOXEL_BIT_MASK;
    }

    inline uint32_t getZ(void) const
    {
        return mVoxel & VOXEL_BIT_MASK;
    }

    uint32_t mVoxel{ 0 };
};

class VoxelHash
{
public:
    size_t operator()(const Voxel& p) const
    {
        return size_t(p.mVoxel); // xor the x,y,z location to compute a hash
    }
};

class VoxelPosition
{
public:
    VoxelPosition(void)
    {
    }
    VoxelPosition(double _x, double _y, double _z) : x(_x), y(_y), z(_z)
    {
    }
    double x;
    double y;
    double z;
};

using VoxelSet = std::unordered_set<Voxel, VoxelHash>;
using VoxelPositionMap = std::unordered_map<Voxel, VoxelPosition, VoxelHash>;
using VoxelVector = std::vector<Voxel>;

} // namespace vcd
