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

groupshared float4 sdata0[64];
groupshared uint sdata0u[64];
groupshared float4 sdata1[8];
groupshared uint sdata1u[8];

void stats0(uint threadIdx1D, uint nodeIdx0, uint channelIdx, const uint grid_type)
{
    pnanovdb_buf_t buf = 0u;

    uint list_leaf_offset = paramsIn.nanoVdb.list_leaf_offset.x >> 2u;

    // note, fetch only 32-bits of 64-bits
    uint leaf_raw = cacheIn[list_leaf_offset + 2u * nodeIdx0];

    pnanovdb_leaf_handle_t leaf;
    leaf.address.byte_offset = leaf_raw;

    // pass 1
    float4 valueStat = float4(0.f, 0.f, 0.f, 0.f);
    uint valueCount = 0u;

    for (uint idx = 0; idx < 8; idx++)
    {
        pnanovdb_uint32_t n0 = threadIdx1D + 64u * idx;
        pnanovdb_address_t address = pnanovdb_leaf_get_table_address(grid_type, buf, leaf, n0);
        float value = pnanovdb_read_float(buf, address);
        if (grid_type == PNANOVDB_GRID_TYPE_VEC3F)
        {
            float value_y = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 4u));
            float value_z = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 8u));
            if (channelIdx == 0u)
            {
                value = sqrt(value * value + value_y * value_y + value_z * value_z);
            }
            //if (channelIdx == 1u)
            //{
            //    value = value;
            //}
            if (channelIdx == 2u)
            {
                value = value_y;
            }
            if (channelIdx == 3u)
            {
                value = value_z;
            }
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
        {
            float value_y = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 4u));
            float value_z = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 8u));
            float value_w = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 12u));
            if (channelIdx == 0u)
            {
                value = sqrt(value * value + value_y * value_y + value_z * value_z + value_w * value_w);
            }
            //if (channelIdx == 1u)
            //{
            //    value = value;
            //}
            if (channelIdx == 2u)
            {
                value = value_y;
            }
            if (channelIdx == 3u)
            {
                value = value_z;
            }
            if (channelIdx == 4u)
            {
                value = value_w;
            }
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
        {
            uint valueRaw = pnanovdb_read_uint32(buf, address);
            float4 value4 = float4(
                float((valueRaw >> 0) & 255) * (1.f / 255.f),
                float((valueRaw >> 8) & 255) * (1.f / 255.f),
                float((valueRaw >> 16) & 255) * (1.f / 255.f),
                float((valueRaw >> 24) & 255) * (1.f / 255.f)
                );
            value = sqrt(dot(value4, value4));
        }
        stat_add(valueStat, valueCount, value);
    }

    sdata0[threadIdx1D] = valueStat;
    sdata0u[threadIdx1D] = valueCount;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx1D < 8u)
    {
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 8u], sdata0u[threadIdx1D + 8u]);
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 16u], sdata0u[threadIdx1D + 16u]);
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 24u], sdata0u[threadIdx1D + 24u]);
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 32u], sdata0u[threadIdx1D + 32u]);
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 40u], sdata0u[threadIdx1D + 40u]);
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 48u], sdata0u[threadIdx1D + 48u]);
        stat_merge(valueStat, valueCount, sdata0[threadIdx1D + 56u], sdata0u[threadIdx1D + 56u]);

        sdata1[threadIdx1D] = valueStat;
        sdata1u[threadIdx1D] = valueCount;
    }

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx1D == 0u)
    {
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 1u], sdata1u[threadIdx1D + 1u]);
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 2u], sdata1u[threadIdx1D + 2u]);
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 3u], sdata1u[threadIdx1D + 3u]);
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 4u], sdata1u[threadIdx1D + 4u]);
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 5u], sdata1u[threadIdx1D + 5u]);
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 6u], sdata1u[threadIdx1D + 6u]);
        stat_merge(valueStat, valueCount, sdata1[threadIdx1D + 7u], sdata1u[threadIdx1D + 7u]);

        float valueStddev = stat_stddev(valueStat, valueCount);

        pnanovdb_address_t address = pnanovdb_leaf_get_min_address(grid_type, buf, leaf);
        if (grid_type == PNANOVDB_GRID_TYPE_VEC3F)
        {
            if (channelIdx == 1u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 0u), valueStat.x);}
            if (channelIdx == 2u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), valueStat.x);}
            if (channelIdx == 3u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), valueStat.x);}
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
        {
            if (channelIdx == 1u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 0u), valueStat.x);}
            if (channelIdx == 2u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), valueStat.x);}
            if (channelIdx == 3u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), valueStat.x);}
            if (channelIdx == 4u) {pnanovdb_write_float(buf, pnanovdb_address_offset(address, 12u), valueStat.x);}
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
        {
            // min always zero due to clamping
            pnanovdb_write_uint32(buf, address, 0u);
        }
        else
        {
            pnanovdb_write_float(buf, address, valueStat.x);
        }
        address = pnanovdb_leaf_get_max_address(grid_type, buf, leaf);
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
        address = pnanovdb_leaf_get_ave_address(grid_type, buf, leaf);
        if (channelIdx == 0u) {pnanovdb_write_float(buf, address, valueStat.z);}
        address = pnanovdb_leaf_get_stddev_address(grid_type, buf, leaf);
        if (channelIdx == 0u) {pnanovdb_write_float(buf, address, valueStddev);}
    }
}

[numthreads(64, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint threadIdx1D = dispatchThreadID.x;
    uint nodeIdx0 = groupID.y + paramsIn.blockIdxOffset0;
    uint channelIdx = groupID.z;

    if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_FLOAT)
    {
        stats0(threadIdx1D, nodeIdx0, channelIdx, PNANOVDB_GRID_TYPE_FLOAT);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_VEC3F)
    {
        stats0(threadIdx1D, nodeIdx0, channelIdx, PNANOVDB_GRID_TYPE_VEC3F);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_VEC4F)
    {
        stats0(threadIdx1D, nodeIdx0, channelIdx, PNANOVDB_GRID_TYPE_VEC4F);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_RGBA8)
    {
        stats0(threadIdx1D, nodeIdx0, channelIdx, PNANOVDB_GRID_TYPE_RGBA8);
    }
}
