// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "voxelize.h"
#include "Volume.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>

#include <vector>



#ifdef _MSC_VER
#pragma warning(disable:4100)
#endif

namespace vcd
{

class VoxelizeImpl : public Voxelize
{
public:
    VoxelizeImpl(void)
    {
    }

    virtual ~VoxelizeImpl(void)
    {
        delete mVolume;
    }

    virtual uint32_t voxelize(vcd::RaycastMesh *raycastMesh,
        const double* const vertices,
        const uint32_t vertexCount,
        const uint32_t* indices,
        const uint32_t triangleCount,
        const uint32_t resolution,VoxelFillMode fillMode) final
    {
        double a = pow((double)(resolution), 0.33);
        mDimensions = (uint32_t)(a*1.5);
        // Minimum voxel resolution is 32x32x32
        if (mDimensions < 32)
        {
            mDimensions = 32;
        }

        //printf("Voxelizing dimensions:%d : %d voxels total\n", mDimensions, mDimensions*mDimensions*mDimensions);

        delete mVolume;
        mVolume = new vcd::Volume;

        mVolume->Voxelize(vertices, vertexCount, (const int32_t *)indices, triangleCount, mDimensions, fillMode, raycastMesh);

        return mDimensions;
    }

    virtual void release(void) final
    {
        delete this;
    }

    virtual double getScale(void) const final
    {
        double ret = 1;

        if ( mVolume )
        {
            ret = mVolume->m_scale;
        }

        return ret;
    }

    virtual bool getBoundsMin(double bmin[3]) const final
    {
        bool ret = false;

        if ( mVolume )
        {
            bmin[0] = mVolume->m_minBB[0];
            bmin[1] = mVolume->m_minBB[1];
            bmin[2] = mVolume->m_minBB[2];
            ret = true;
        }

        return ret;
    }

    virtual bool getBoundsMax(double bmax[3]) const final
    {
        bool ret = false;

        if (mVolume)
        {
            bmax[0] = mVolume->m_maxBB[0];
            bmax[1] = mVolume->m_maxBB[1];
            bmax[2] = mVolume->m_maxBB[2];
            ret = true;
        }

        return ret;
    }

    virtual bool getDimensions(uint32_t dim[3]) const final
    {
        bool ret = false;

        if ( mVolume )
        {
            dim[0] = uint32_t(mVolume->m_dim[0]);
            dim[1] = uint32_t(mVolume->m_dim[1]);
            dim[2] = uint32_t(mVolume->m_dim[2]);
            ret = true;
        }

        return ret;
    }

    virtual uint8_t getVoxel(uint32_t x, uint32_t y, uint32_t z) const final
    {
        uint8_t ret = 0;

        if ( mVolume )
        {
            ret = mVolume->GetVoxel(x,y,z);
        }

        return ret;
    }

    virtual void setVoxel(uint32_t x, uint32_t y, uint32_t z, uint8_t value) final
    {
        if ( mVolume )
        {
            mVolume->SetVoxel(x,y,z,value);
        }
    }

    virtual bool getVoxel(const double *pos,uint32_t &x,uint32_t &y, uint32_t &z) const  final
    {
        bool ret = false;

        double bmin[3] = {0.0};
        double bmax[3] = {0.0};

        getBoundsMin(bmin);
        getBoundsMax(bmax);

        if ( pos[0] >= bmin[0] && pos[0] < bmax[0] &&
             pos[1] >= bmin[1] && pos[1] < bmax[1] &&
             pos[2] >= bmin[2] && pos[2] < bmax[2] )
        {
            double recipScale = 1.0 / getScale();
            x = uint32_t( (pos[0] - bmin[0])*recipScale);
            y = uint32_t( (pos[1] - bmin[1])*recipScale);
            z = uint32_t( (pos[2] - bmin[2])*recipScale);
            ret = true;
        }
        return ret;
    }

    virtual bool getSurfaceVoxels(VoxelVector &surfaceVoxels) final
    {
        bool ret = false;

        if ( mVolume )
        {
            surfaceVoxels.clear();
            surfaceVoxels = mVolume->getSurfaceVoxels();
            ret = !surfaceVoxels.empty();
        }

        return ret;
    }

    virtual bool getInteriorVoxels(VoxelVector &interiorVoxels) final
    {
        bool ret = false;

        if (mVolume)
        {
            interiorVoxels.clear();
            interiorVoxels = mVolume->getInteriorVoxels();
            ret = !interiorVoxels.empty();
        }

        return ret;
    }

    uint32_t        mDimensions{32};
    vcd::Volume	*mVolume{nullptr};
};

Voxelize *Voxelize::create(void)
{
    auto ret = new VoxelizeImpl;
    return static_cast< Voxelize *>(ret);
}


}
