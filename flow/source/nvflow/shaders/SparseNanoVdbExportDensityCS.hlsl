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

ConstantBuffer<SparseNanoVdbExportParams> paramsIn;

StructuredBuffer<uint> tableIn;

Texture3D<float4> densityIn;

RWStructuredBuffer<uint> temperatureOut;
RWStructuredBuffer<uint> fuelOut;
RWStructuredBuffer<uint> burnOut;
RWStructuredBuffer<uint> smokeOut;
RWStructuredBuffer<uint> rgbaOut;
RWStructuredBuffer<uint> rgbOut;

#define PNANOVDB_HLSL
#define PNANOVDB_BUF_CUSTOM

#define pnanovdb_buf_t uint
uint pnanovdb_buf_read_uint32(pnanovdb_buf_t buf, uint byte_offset)
{
    if (buf == 0)
    {
        return temperatureOut[byte_offset >> 2u];
    }
    else if (buf == 1)
    {
        return fuelOut[byte_offset >> 2u];
    }
    else if (buf == 2)
    {
        return burnOut[byte_offset >> 2u];
    }
    else if (buf == 3)
    {
        return smokeOut[byte_offset >> 2u];
    }
    else if (buf == 4)
    {
        return rgbaOut[byte_offset >> 2u];
    }
    else // if (buf == 5)
    {
        return rgbOut[byte_offset >> 2u];
    }
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
    if (buf == 0)
    {
        temperatureOut[byte_offset >> 2u] = value;
    }
    else if (buf == 1)
    {
        fuelOut[byte_offset >> 2u] = value;
    }
    else if (buf == 2)
    {
        burnOut[byte_offset >> 2u] = value;
    }
    else if (buf == 3)
    {
        smokeOut[byte_offset >> 2u] = value;
    }
    else if (buf == 4)
    {
        rgbaOut[byte_offset >> 2u] = value;
    }
    else // if (buf == 5)
    {
        rgbOut[byte_offset >> 2u] = value;
    }
}
void pnanovdb_buf_write_uint64(pnanovdb_buf_t buf, uint byte_offset, uint2 value)
{
    pnanovdb_buf_write_uint32(buf, byte_offset + 0u, value.x);
    pnanovdb_buf_write_uint32(buf, byte_offset + 4u, value.y);
}
#define pnanovdb_grid_type_t uint
#define PNANOVDB_GRID_TYPE_GET(grid_typeIn, nameIn) pnanovdb_grid_type_constants[grid_typeIn].nameIn

#include "PNanoVDB.h"

void writeChannel(uint channel, uint threadIdx1D, uint layerParamIdx, int3 vidx, float value)
{
    pnanovdb_buf_t buf = channel;

    pnanovdb_grid_handle_t grid = { pnanovdb_address_offset(pnanovdb_address_null(), layerParamIdx * (PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)) };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

    pnanovdb_readaccessor_t readaccessor;
    pnanovdb_readaccessor_init(PNANOVDB_REF(readaccessor), root);

    pnanovdb_coord_t ijk = vidx;

    pnanovdb_address_t address = pnanovdb_readaccessor_get_value_address(PNANOVDB_GRID_TYPE_FLOAT, buf, PNANOVDB_REF(readaccessor), ijk);

    pnanovdb_write_float(buf, address, value);

    if (threadIdx1D < 16u)
    {
        pnanovdb_write_uint32(buf, pnanovdb_address_offset(readaccessor.leaf.address, PNANOVDB_LEAF_OFF_VALUE_MASK + 4u * threadIdx1D), 0xFFFFFFFF);
    }
    if (threadIdx1D == 0u)
    {
        pnanovdb_coord_t leaf_bbox_min = int3(
            (vidx.x >> 3) << 3,
            (vidx.y >> 3) << 3,
            (vidx.z >> 3) << 3
        );
        pnanovdb_leaf_set_bbox_min(buf, readaccessor.leaf, PNANOVDB_REF(leaf_bbox_min));
        pnanovdb_leaf_set_bbox_dif_and_flags(buf, readaccessor.leaf, 0x00080808);
    }
}

void writeChannelRgba(uint channel, uint threadIdx1D, uint layerParamIdx, int3 vidx, float4 value)
{
    pnanovdb_buf_t buf = channel;

    pnanovdb_grid_handle_t grid = { pnanovdb_address_offset(pnanovdb_address_null(), layerParamIdx * (PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)) };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

    pnanovdb_readaccessor_t readaccessor;
    pnanovdb_readaccessor_init(PNANOVDB_REF(readaccessor), root);

    pnanovdb_coord_t ijk = vidx;

    pnanovdb_address_t address = pnanovdb_readaccessor_get_value_address(grid_type, buf, PNANOVDB_REF(readaccessor), ijk);
    if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
    {
        pnanovdb_write_float(buf, pnanovdb_address_offset(address, 0u), value.x);
        pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), value.y);
        pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), value.z);
        pnanovdb_write_float(buf, pnanovdb_address_offset(address, 12u), value.w);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
    {
        uint rawValue = uint(255.f * max(0.f, min(1.f, value.x))) |
            (uint(255.f * max(0.f, min(1.f, value.y))) << 8u) |
            (uint(255.f * max(0.f, min(1.f, value.z))) << 16u) |
            (uint(255.f * max(0.f, min(1.f, value.w))) << 24u);
        pnanovdb_write_uint32(buf, pnanovdb_address_offset(address, 0u), rawValue);
    }

    if (threadIdx1D < 16u)
    {
        pnanovdb_write_uint32(buf, pnanovdb_address_offset(readaccessor.leaf.address, PNANOVDB_LEAF_OFF_VALUE_MASK + 4u * threadIdx1D), 0xFFFFFFFF);
    }
    if (threadIdx1D == 0u)
    {
        pnanovdb_coord_t leaf_bbox_min = int3(
            (vidx.x >> 3) << 3,
            (vidx.y >> 3) << 3,
            (vidx.z >> 3) << 3
            );
        pnanovdb_leaf_set_bbox_min(buf, readaccessor.leaf, PNANOVDB_REF(leaf_bbox_min));
        pnanovdb_leaf_set_bbox_dif_and_flags(buf, readaccessor.leaf, 0x00080808);
    }
}

void writeChannelRgb(uint channel, uint threadIdx1D, uint layerParamIdx, int3 vidx, float3 value)
{
    pnanovdb_buf_t buf = channel;

    pnanovdb_grid_handle_t grid = { pnanovdb_address_offset(pnanovdb_address_null(), layerParamIdx * (PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)) };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

    pnanovdb_readaccessor_t readaccessor;
    pnanovdb_readaccessor_init(PNANOVDB_REF(readaccessor), root);

    pnanovdb_coord_t ijk = vidx;

    pnanovdb_address_t address = pnanovdb_readaccessor_get_value_address(PNANOVDB_GRID_TYPE_VEC3F, buf, PNANOVDB_REF(readaccessor), ijk);

    pnanovdb_write_float(buf, pnanovdb_address_offset(address, 0u), value.x);
    pnanovdb_write_float(buf, pnanovdb_address_offset(address, 4u), value.y);
    pnanovdb_write_float(buf, pnanovdb_address_offset(address, 8u), value.z);

    if (threadIdx1D < 16u)
    {
        pnanovdb_write_uint32(buf, pnanovdb_address_offset(readaccessor.leaf.address, PNANOVDB_LEAF_OFF_VALUE_MASK + 4u * threadIdx1D), 0xFFFFFFFF);
    }
    if (threadIdx1D == 0u)
    {
        pnanovdb_coord_t leaf_bbox_min = int3(
            (vidx.x >> 3) << 3,
            (vidx.y >> 3) << 3,
            (vidx.z >> 3) << 3
            );
        pnanovdb_leaf_set_bbox_min(buf, readaccessor.leaf, PNANOVDB_REF(leaf_bbox_min));
        pnanovdb_leaf_set_bbox_dif_and_flags(buf, readaccessor.leaf, 0x00080808);
    }
}

[numthreads(512, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint threadIdx1D = dispatchThreadID.x;
    uint subBlockIdx1D = dispatchThreadID.y;
    uint blockIdx = groupID.z + paramsIn.blockIdxOffset;

    uint3 subBlockIdx = uint3(
        subBlockIdx1D & paramsIn.densityFloat.subGridDimLessOne.x,
        (subBlockIdx1D >> paramsIn.densityFloat.subGridDimBits.x) & paramsIn.densityFloat.subGridDimLessOne.y,
        (subBlockIdx1D >> (paramsIn.densityFloat.subGridDimBits.x + paramsIn.densityFloat.subGridDimBits.y)) & paramsIn.densityFloat.subGridDimLessOne.z
    );
    int3 threadIdx = int3(
        (subBlockIdx.x << 3) + (threadIdx1D & 7),
        (subBlockIdx.y << 3) + ((threadIdx1D >> 3) & 7),
        (subBlockIdx.z << 3) + ((threadIdx1D >> 6) & 7)
    );

    uint layerParamIdx = NvFlowGetLayerParamIdx(tableIn, paramsIn.tableDensity, blockIdx);

    int4 location;
    NvFlowBlockIdxToLocation(tableIn, paramsIn.tableDensity, blockIdx, location);

    int3 vidx = (location.xyz << int3(paramsIn.tableDensity.blockDimBits.xyz)) + threadIdx;

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(tableIn, paramsIn.tableDensity, blockIdx, threadIdx).xyz;

    float4 density = densityIn[readIdx];

    if (paramsIn.temperatureEnabled != 0u)
    {
        writeChannel(0, threadIdx1D, layerParamIdx, vidx, density.x);
    }
    if (paramsIn.fuelEnabled != 0u)
    {
        writeChannel(1, threadIdx1D, layerParamIdx, vidx, density.y);
    }
    if (paramsIn.burnEnabled != 0u)
    {
        writeChannel(2, threadIdx1D, layerParamIdx, vidx, density.z);
    }
    if (paramsIn.smokeEnabled != 0u)
    {
        writeChannel(3, threadIdx1D, layerParamIdx, vidx, density.w);
    }
    if (paramsIn.rgbaEnabled != 0u)
    {
        writeChannelRgba(4, threadIdx1D, layerParamIdx, vidx, density);
    }
    if (paramsIn.rgbEnabled != 0u)
    {
        writeChannelRgb(5, threadIdx1D, layerParamIdx, vidx, density.xyz);
    }
}
