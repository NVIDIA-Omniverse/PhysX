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
StructuredBuffer<uint> nanovdbSrcIn;
RWStructuredBuffer<uint> nanovdbInOut;

#define PNANOVDB_HLSL
#define PNANOVDB_BUF_CUSTOM

#define pnanovdb_buf_t uint
uint pnanovdb_buf_read_uint32(pnanovdb_buf_t buf, uint byte_offset)
{
    return nanovdbSrcIn[byte_offset >> 2u];            // NOTE: read different than write
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

pnanovdb_int64_t pnanovdb_uint64_diff(pnanovdb_uint64_t a, pnanovdb_uint64_t b)
{
    pnanovdb_uint32_t low = pnanovdb_uint64_low(a);
    pnanovdb_uint32_t high = pnanovdb_uint64_high(a);
    low -= pnanovdb_uint64_low(b);
    if (low > pnanovdb_uint64_low(a))
    {
        high -= 1u;
    }
    high -= pnanovdb_uint64_high(b);
    return pnanovdb_uint64_as_int64(pnanovdb_uint32_as_uint64(low, high));
}

pnanovdb_int64_t pnanovdb_address_diff(pnanovdb_address_t a, pnanovdb_address_t b)
{
    return pnanovdb_uint64_diff(pnanovdb_uint32_as_uint64_low(a.byte_offset),pnanovdb_uint32_as_uint64_low(b.byte_offset));
}

void pnanovdb_lower_set_child(pnanovdb_grid_type_t grid_type, pnanovdb_buf_t buf, pnanovdb_lower_handle_t lower, pnanovdb_uint32_t n, pnanovdb_leaf_handle_t leaf)
{
    pnanovdb_lower_set_table_child(grid_type, buf, lower, n, pnanovdb_address_diff(leaf.address, lower.address));
}

void pruneLeaves(uint threadIdx1D, uint nodeIdx0, const uint grid_type)
{
    pnanovdb_buf_t buf = 0u;

    uint list_lower_offset = paramsIn.nanoVdb.list_lower_offset.x >> 2u;

    // note, fetch only 32-bits of 64-bits
    uint lower_raw = cacheIn[list_lower_offset + 2u * nodeIdx0];

    pnanovdb_lower_handle_t lower;
    lower.address.byte_offset = lower_raw;

    for (pnanovdb_uint32_t lower_n = threadIdx1D; lower_n < PNANOVDB_LOWER_TABLE_COUNT; lower_n+=256u)
    {
        if (!pnanovdb_lower_get_child_mask(buf, lower, lower_n))
        {
            continue;
        }

        pnanovdb_leaf_handle_t leaf = pnanovdb_lower_get_child(grid_type, buf, lower, lower_n);

        pnanovdb_coord_t ijk = pnanovdb_leaf_get_bbox_min(buf, leaf);

        // clear child mask
        pnanovdb_address_t mask_addr = pnanovdb_address_offset(lower.address, PNANOVDB_LOWER_OFF_CHILD_MASK + 4u * (lower_n >> 5u));
        pnanovdb_write_uint32(buf, mask_addr, 0u);

        // clear leaf address in lower
        pnanovdb_leaf_handle_t null_leaf = {pnanovdb_address_null()};
        pnanovdb_lower_set_child(grid_type, buf, lower, lower_n, null_leaf);

        // write maximum stat as tile value
        pnanovdb_address_t maxAddr = pnanovdb_leaf_get_max_address(grid_type, buf, leaf);
        pnanovdb_address_t dstAddr = pnanovdb_lower_get_table_address(grid_type, buf, lower, lower_n);
        if (grid_type == PNANOVDB_GRID_TYPE_VEC3F)
        {
            float v_x = pnanovdb_read_float(buf, maxAddr);
            float v_y = pnanovdb_read_float(buf, pnanovdb_address_offset(maxAddr, 4u));
            float v_z = pnanovdb_read_float(buf, pnanovdb_address_offset(maxAddr, 8u));
            pnanovdb_write_float(buf, dstAddr, v_x);
            pnanovdb_write_float(buf, pnanovdb_address_offset(dstAddr, 4u), v_y);
            pnanovdb_write_float(buf, pnanovdb_address_offset(dstAddr, 8u), v_z);
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
        {
            float v_x = pnanovdb_read_float(buf, maxAddr);
            float v_y = pnanovdb_read_float(buf, pnanovdb_address_offset(maxAddr, 4u));
            float v_z = pnanovdb_read_float(buf, pnanovdb_address_offset(maxAddr, 8u));
            float v_w = pnanovdb_read_float(buf, pnanovdb_address_offset(maxAddr, 12u));
            pnanovdb_write_float(buf, dstAddr, v_x);
            pnanovdb_write_float(buf, pnanovdb_address_offset(dstAddr, 4u), v_y);
            pnanovdb_write_float(buf, pnanovdb_address_offset(dstAddr, 8u), v_z);
            pnanovdb_write_float(buf, pnanovdb_address_offset(dstAddr, 12u), v_w);
        }
        else if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
        {
            uint rawVal = pnanovdb_read_uint32(buf, maxAddr);
            pnanovdb_write_uint32(buf, dstAddr, rawVal);
        }
        else
        {
            float v = pnanovdb_read_float(buf, maxAddr);
            pnanovdb_write_float(buf, dstAddr, v);
        }
    }
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint threadIdx1D = dispatchThreadID.x;
    uint nodeIdx0 = groupID.y + paramsIn.blockIdxOffset0;

    if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_FLOAT)
    {
        pruneLeaves(threadIdx1D, nodeIdx0, PNANOVDB_GRID_TYPE_FLOAT);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_VEC3F)
    {
        pruneLeaves(threadIdx1D, nodeIdx0, PNANOVDB_GRID_TYPE_VEC3F);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_VEC4F)
    {
        pruneLeaves(threadIdx1D, nodeIdx0, PNANOVDB_GRID_TYPE_VEC4F);
    }
    else if (paramsIn.nanoVdb.grid_type == PNANOVDB_GRID_TYPE_RGBA8)
    {
        pruneLeaves(threadIdx1D, nodeIdx0, PNANOVDB_GRID_TYPE_RGBA8);
    }
}
