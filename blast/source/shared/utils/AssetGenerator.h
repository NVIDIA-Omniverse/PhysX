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


#ifndef ASSETGENERATOR_H
#define ASSETGENERATOR_H


#include "NvBlast.h"

#include <vector>
#include <cmath>

class GeneratorAsset
{
public:
    struct Vec3
    {
        float x, y, z;

        Vec3() {}
        Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
        Vec3 operator * (float v) const { return Vec3(x * v, y * v, z * v); }
        Vec3 operator * (const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
        Vec3 operator + (const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
        Vec3 operator - (const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
        Vec3 getNormalized() const
        {
            return (*this)*(1.0f / sqrt(x*x + y*y + z*z));
        }
    };

    struct BlastChunkCube
    {
        BlastChunkCube(Vec3 position_, Vec3 extents_)
        {
            position = position_;
            extents = extents_;
        }

        Vec3 position;
        Vec3 extents;
    };

    std::vector<NvBlastChunkDesc> solverChunks;
    std::vector<NvBlastBondDesc> solverBonds;
    std::vector<BlastChunkCube> chunks;
    Vec3 extents;
};


class CubeAssetGenerator
{
public:
    struct DepthInfo
    {
        DepthInfo(GeneratorAsset::Vec3 slices = GeneratorAsset::Vec3(1, 1, 1), NvBlastChunkDesc::Flags flag_ = NvBlastChunkDesc::Flags::NoFlags) 
            : slicesPerAxis(slices), flag(flag_) {}

        GeneratorAsset::Vec3 slicesPerAxis;
        NvBlastChunkDesc::Flags flag;
    };

    enum BondFlags
    {
        NO_BONDS            = 0,
        X_BONDS             = 1 << 0,
        Y_BONDS             = 1 << 1,
        Z_BONDS             = 1 << 2,
        X_PLUS_WORLD_BONDS  = 1 << 3,
        X_MINUS_WORLD_BONDS = 1 << 4,
        Y_PLUS_WORLD_BONDS  = 1 << 5,
        Y_MINUS_WORLD_BONDS = 1 << 6,
        Z_PLUS_WORLD_BONDS  = 1 << 7,
        Z_MINUS_WORLD_BONDS = 1 << 8,
        ALL_INTERNAL_BONDS  = X_BONDS | Y_BONDS | Z_BONDS
    };


    struct Settings
    {
        Settings() : bondFlags(BondFlags::ALL_INTERNAL_BONDS) {}

        std::vector<DepthInfo> depths;
        GeneratorAsset::Vec3 extents;
        BondFlags bondFlags;
    };

    static void generate(GeneratorAsset& asset, const Settings& settings);
private:
    static void fillBondDesc(std::vector<NvBlastBondDesc>& bondDescs, uint32_t id0, uint32_t id1, GeneratorAsset::Vec3 pos0, GeneratorAsset::Vec3 pos1, GeneratorAsset::Vec3 size, float area);
};


inline CubeAssetGenerator::BondFlags operator | (CubeAssetGenerator::BondFlags a, CubeAssetGenerator::BondFlags b)
{
    return static_cast<CubeAssetGenerator::BondFlags>(static_cast<int>(a) | static_cast<int>(b));
}

#endif // #ifndef ASSETGENERATOR_H
