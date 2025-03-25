// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

#include "NvFlowShader.hlsli"

#include "SparseParams.h"

ConstantBuffer<SparseNanoVdbComputeStatsParams> paramsIn;
StructuredBuffer<uint> cacheIn;
RWStructuredBuffer<uint> nanovdbInOut;

#define PNANOVDB_HLSL
#define PNANOVDB_BUF_CUSTOM

#define pnanovdb_buf_t uint
uint pnanovdb_buf_read_uint32(pnanovdb_buf_t buf, uint byte_offset)
{
    return nanovdbInOut[byte_offset >> 2u];
}

uint2 pnanovdb_buf_read_uint64(pnanovdb_buf_t buf, uint byte_offset)
{
    uint2 ret;
    ret.x = pnanovdb_buf_read_uint32(buf, byte_offset + 0u);
    ret.y = pnanovdb_buf_read_uint32(buf, byte_offset + 4u);
    return ret;
}
void pnanovdb_buf_write_uint32(pnanovdb_buf_t buf, uint byte_offset, uint value)
{
    nanovdbInOut[byte_offset >> 2u] = value;
}
void pnanovdb_buf_write_uint64(pnanovdb_buf_t buf, uint byte_offset, uint2 value)
{
    pnanovdb_buf_write_uint32(buf, byte_offset + 0u, value.x);
    pnanovdb_buf_write_uint32(buf, byte_offset + 4u, value.y);
}
#define pnanovdb_grid_type_t uint
#define PNANOVDB_GRID_TYPE_GET(grid_typeIn, nameIn) pnanovdb_grid_type_constants[grid_typeIn].nameIn

#include "PNanoVDB.h"

#include "SparseNanoVdbCommon.hlsli"

void stats3(uint threadIdx1D, uint gridIdx, uint channelIdx, const uint grid_type)
{
    if (threadIdx1D == 0u)
    {
        pnanovdb_buf_t buf = 0u;
        pnanovdb_grid_handle_t grid = { pnanovdb_address_offset(pnanovdb_address_null(), gridIdx * (PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)) };
        pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
        pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

        float4 valueStat = float4(0.f, 0.f, 0.f, 0.f);
        uint valueCount = 0u;

        pnanovdb_uint32_t tile_count = pnanovdb_root_get_tile_count(buf, root);
        pnanovdb_root_tile_handle_t tile = pnanovdb_root_get_tile_zero(grid_type, root);
        for (pnanovdb_uint32_t i = 0u; i < tile_count; i++)
        {
            if (!pnanovdb_int64_is_zero(pnanovdb_root_tile_get_child(buf, tile)))
            {
                pnanovdb_upper_handle_t upper = pnanovdb_root_get_child(grid_type, buf, root, tile);

                float4 localStat;
                const uint localCount = 1u;    // 2^36 is voxel count, exceeds range

                pnanovdb_address_t address = pnanovdb_upper_get_min_address(grid_type, buf, upper);
                localStat.x = pnanovdb_read_float(buf, address);
                if (channelIdx > 0u)
                {
                    localStat.x = pnanovdb_read_float(buf, pnanovdb_address_offset_product(address,channelIdx-1,4u));
                }
                address = pnanovdb_upper_get_max_address(grid_type, buf, upper);
                localStat.y = pnanovdb_read_float(buf, address);
                if (channelIdx > 0u)
                {
                    localStat.y = pnanovdb_read_float(buf, pnanovdb_address_offset_product(address,channelIdx-1,4u));
                }
                address = pnanovdb_upper_get_ave_address(grid_type, buf, upper);
                localStat.z = pnanovdb_read_float(buf, address);
                address = pnanovdb_upper_get_stddev_address(grid_type, buf, upper);
                localStat.w = pnanovdb_read_float(buf, address);

                stat_merge(valueStat, valueCount, localStat, localCount);
            }
            tile.address = pnanovdb_address_offset(tile.address, PNANOVDB_GRID_TYPE_GET(grid_type, root_tile_size));
        }

        float valueStddev = stat_stddev(valueStat, valueCount);

        pnanovdb_address_t address = pnanovdb_root_get_min_address(grid_type, buf, root);
        if (grid_type == PNANOVDB_GRID_TYPE_VEC3F)
        {
            if (channelIdx == 1u) {pnanovdb_write_float(buf, address, valueStat.x);}
            if (channelIdx == 2u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), valueStat.x);}
            if (channelIdx == 3u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), valueStat.x);}
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
        {
            if (channelIdx == 1u) {pnanovdb_write_float(buf, address, valueStat.x);}
            if (channelIdx == 2u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), valueStat.x);}
            if (channelIdx == 3u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), valueStat.x);}
            if (channelIdx == 4u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 12u), valueStat.x);}
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
        {
            uint rawVal = uint(max(0.f, min(1.f, valueStat.x) * 255.f));
            rawVal = rawVal | (rawVal << 8u) | (rawVal << 16u) | (rawVal << 24u);
            pnanovdb_write_uint32(buf, address, rawVal);
        }
        else
        {
            pnanovdb_write_float(buf, address, valueStat.x);
        }
        address = pnanovdb_root_get_max_address(grid_type, buf, root);
        if (grid_type == PNANOVDB_GRID_TYPE_VEC3F)
        {
            if (channelIdx == 1u) {pnanovdb_write_float(buf, address, valueStat.y);}
            if (channelIdx == 2u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), valueStat.y);}
            if (channelIdx == 3u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), valueStat.y);}
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
        {
            if (channelIdx == 1u) {pnanovdb_write_float(buf, address, valueStat.y);}
            if (channelIdx == 2u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), valueStat.y);}
            if (channelIdx == 3u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), valueStat.y);}
            if (channelIdx == 4u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 12u), valueStat.y);}
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
        {
            uint rawVal = uint(max(0.f, min(1.f, valueStat.y) * 255.f));
            rawVal = rawVal | (rawVal << 8u) | (rawVal << 16u) | (rawVal << 24u);
            pnanovdb_write_uint32(buf, address, rawVal);
        }
        else
        {
            pnanovdb_write_float(buf, address, valueStat.y);
        }
        address = pnanovdb_root_get_ave_address(grid_type, buf, root);
        if (channelIdx == 0u) {pnanovdb_write_float(buf, address, valueStat.z);}
        address = pnanovdb_root_get_stddev_address(grid_type, buf, root);
        if (channelIdx == 0u) {pnanovdb_write_float(buf, address, valueStddev);}
    }
}

[numthreads(32, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint threadIdx1D = dispatchThreadID.x;
    uint gridIdx = groupID.y + paramsIn.blockIdxOffset3;
    uint channelIdx = groupID.z;

    if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_FLOAT)
    {
        stats3(threadIdx1D, gridIdx, channelIdx, PNANOVDB_GRID_TYPE_FLOAT);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_VEC3F)
    {
        stats3(threadIdx1D, gridIdx, channelIdx, PNANOVDB_GRID_TYPE_VEC3F);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_VEC4F)
    {
        stats3(threadIdx1D, gridIdx, channelIdx, PNANOVDB_GRID_TYPE_VEC4F);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_RGBA8)
    {
        stats3(threadIdx1D, gridIdx, channelIdx, PNANOVDB_GRID_TYPE_RGBA8);
    }
}
