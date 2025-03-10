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

Texture3D<float4> velocityIn;

RWStructuredBuffer<uint> velocityOut;
RWStructuredBuffer<uint> divergenceOut;

#define PNANOVDB_HLSL
#define PNANOVDB_BUF_CUSTOM

#define pnanovdb_buf_t uint
uint pnanovdb_buf_read_uint32(pnanovdb_buf_t buf, uint byte_offset)
{
    if (buf == 0)
    {
        return velocityOut[byte_offset >> 2u];
    }
    else // if (buf == 1)
    {
        return divergenceOut[byte_offset >> 2u];
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
        velocityOut[byte_offset >> 2u] = value;
    }
    else //if (buf == 1)
    {
        divergenceOut[byte_offset >> 2u] = value;
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

void writeChannelFloat(uint channel, uint threadIdx1D, uint layerParamIdx, int3 vidx, float value)
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

void writeChannelVec3(uint channel, uint threadIdx1D, uint layerParamIdx, int3 vidx, float3 value)
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
        subBlockIdx1D & paramsIn.velocityVec3.subGridDimLessOne.x,
        (subBlockIdx1D >> paramsIn.velocityVec3.subGridDimBits.x) & paramsIn.velocityVec3.subGridDimLessOne.y,
        (subBlockIdx1D >> (paramsIn.velocityVec3.subGridDimBits.x + paramsIn.velocityVec3.subGridDimBits.y)) & paramsIn.velocityVec3.subGridDimLessOne.z
        );
    int3 threadIdx = int3(
        (subBlockIdx.x << 3) + (threadIdx1D & 7),
        (subBlockIdx.y << 3) + ((threadIdx1D >> 3) & 7),
        (subBlockIdx.z << 3) + ((threadIdx1D >> 6) & 7)
        );

    uint layerParamIdx = NvFlowGetLayerParamIdx(tableIn, paramsIn.tableVelocity, blockIdx);

    int4 location;
    NvFlowBlockIdxToLocation(tableIn, paramsIn.tableVelocity, blockIdx, location);

    int3 vidx = (location.xyz << int3(paramsIn.tableVelocity.blockDimBits.xyz)) + threadIdx;

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(tableIn, paramsIn.tableVelocity, blockIdx, threadIdx).xyz;

    float4 velocity = velocityIn[readIdx];

    if (paramsIn.velocityEnabled != 0u)
    {
        writeChannelVec3(0, threadIdx1D, layerParamIdx, vidx, velocity.xyz);
    }
    if (paramsIn.divergenceEnabled != 0u)
    {
        writeChannelFloat(1, threadIdx1D, layerParamIdx, vidx, velocity.w);
    }
}
