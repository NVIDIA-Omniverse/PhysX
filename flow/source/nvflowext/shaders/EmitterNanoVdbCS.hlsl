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

#include "EmitterParams.h"

ConstantBuffer<EmitterNanoVdbCS_Params> gParams;

StructuredBuffer<uint> gTable;

StructuredBuffer<uint> arraysIn;

StructuredBuffer<uint> interopDistances;
StructuredBuffer<uint> interopVelocities;
StructuredBuffer<uint> interopDivergences;
StructuredBuffer<uint> interopTemperatures;
StructuredBuffer<uint> interopFuels;
StructuredBuffer<uint> interopBurns;
StructuredBuffer<uint> interopSmokes;

Texture3D<float4> valueIn;

RWTexture3D<float4> valueOut;
RWTexture3D<float> voxelWeightOut;

#define PNANOVDB_HLSL
#define PNANOVDB_BUF_CUSTOM

#define pnanovdb_buf_t uint4
uint pnanovdb_buf_read_uint32(pnanovdb_buf_t buf, uint byte_offset)
{
    if (buf.z == 0)
    {
        return arraysIn[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 1)
    {
        return interopDistances[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 2)
    {
        return interopVelocities[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 3)
    {
        return interopDivergences[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 4)
    {
        return interopTemperatures[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 5)
    {
        return interopFuels[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 6)
    {
        return interopBurns[(byte_offset >> 2u) + buf.x];
    }
    else if (buf.z == 7)
    {
        return interopSmokes[(byte_offset >> 2u) + buf.x];
    }
    return 0u;
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
    // NOP, write disabled
}
void pnanovdb_buf_write_uint64(pnanovdb_buf_t buf, uint byte_offset, uint2 value)
{
    // NOP, write disable
}
#define pnanovdb_grid_type_t uint
#define PNANOVDB_GRID_TYPE_GET(grid_typeIn, nameIn) pnanovdb_grid_type_constants[grid_typeIn].nameIn

#include "PNanoVDB.h"

float getFloatAtIjk(pnanovdb_grid_type_t grid_type, pnanovdb_buf_t buf, pnanovdb_root_handle_t root, int3 ijk)
{
    float value = 0.f;
    if (grid_type == PNANOVDB_GRID_TYPE_FLOAT)
    {
        pnanovdb_address_t address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_FLOAT, buf, root, ijk);
        value = pnanovdb_read_float(buf, address);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_HALF)
    {
        pnanovdb_address_t address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_FP16, buf, root, ijk);
        value = pnanovdb_read_half(buf, address);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_FP4)
    {
        pnanovdb_uint32_t level = 0u;
        pnanovdb_address_t address = pnanovdb_root_get_value_address_and_level(PNANOVDB_GRID_TYPE_FP4, buf, root, ijk, level);
        value = pnanovdb_root_fp4_read_float(buf, address, ijk, level);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_FP8)
    {
        pnanovdb_uint32_t level = 0u;
        pnanovdb_address_t address = pnanovdb_root_get_value_address_and_level(PNANOVDB_GRID_TYPE_FP8, buf, root, ijk, level);
        value = pnanovdb_root_fp8_read_float(buf, address, ijk, level);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_FP16)
    {
        pnanovdb_uint32_t level = 0u;
        pnanovdb_address_t address = pnanovdb_root_get_value_address_and_level(PNANOVDB_GRID_TYPE_FP16, buf, root, ijk, level);
        value = pnanovdb_root_fp16_read_float(buf, address, ijk, level);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_FPN)
    {
        pnanovdb_uint32_t level = 0u;
        pnanovdb_address_t address = pnanovdb_root_get_value_address_and_level(PNANOVDB_GRID_TYPE_FPN, buf, root, ijk, level);
        value = pnanovdb_root_fpn_read_float(buf, address, ijk, level);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)    // read w channel for vec4 by default
    {
        pnanovdb_address_t address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_VEC4F, buf, root, ijk);
        value = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 12u));
    }
    return value;
}

float getFloat(pnanovdb_buf_t buf, float4 worldPos)
{
    pnanovdb_grid_handle_t grid = { pnanovdb_address_null() };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

    float3 indexf = pnanovdb_grid_world_to_indexf(buf, grid, worldPos.xyz);
    int3 ijk = NvFlowFloor_i(indexf);

    return getFloatAtIjk(grid_type, buf, root, ijk);
}

float3 getFloat3(pnanovdb_buf_t buf, float4 worldPos)
{
    pnanovdb_grid_handle_t grid = { pnanovdb_address_null() };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

    float3 indexf = pnanovdb_grid_world_to_indexf(buf, grid, worldPos.xyz);
    int3 ijk = NvFlowFloor_i(indexf);

    float3 value = float3(0.f, 0.f, 0.f);
    pnanovdb_address_t address;
    if (grid_type == PNANOVDB_GRID_TYPE_VEC3F)
    {
        address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_VEC3F, buf, root, ijk);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
    {
        address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_VEC4F, buf, root, ijk);
    }
    value.x = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 0u));
    value.y = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 4u));
    value.z = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 8u));
    return value;
}

float4 getRgba8(pnanovdb_buf_t buf, float4 worldPos)
{
    pnanovdb_grid_handle_t grid = { pnanovdb_address_null() };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

    float3 indexf = pnanovdb_grid_world_to_indexf(buf, grid, worldPos.xyz);
    int3 ijk = NvFlowFloor_i(indexf);

    float4 value = float4(0.f, 0.f, 0.f, 0.f);
    if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
    {
        pnanovdb_address_t address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_RGBA8, buf, root, ijk);
        uint rawValue = pnanovdb_read_uint32(buf, pnanovdb_address_offset(address, 0u));
        value.r = float((rawValue) & 255) * (1.f / 255.f);
        value.g = float((rawValue >> 8) & 255) * (1.f / 255.f);
        value.b = float((rawValue >> 16) & 255) * (1.f / 255.f);
        value.a = float((rawValue >> 24) & 255) * (1.f / 255.f);
    }
    else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
    {
        pnanovdb_address_t address = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_VEC4F, buf, root, ijk);
        value.r = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 0u));
        value.g = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 4u));
        value.b = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 8u));
        value.a = pnanovdb_read_float(buf, pnanovdb_address_offset(address, 12u));
    }
    return value;
}

float getFloatInterpolated(pnanovdb_buf_t buf, float4 worldPos)
{
    pnanovdb_grid_handle_t grid = { pnanovdb_address_null() };
    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

    float3 indexf = pnanovdb_grid_world_to_indexf(buf, grid, worldPos.xyz);
    int3 ijk000 = NvFlowFloor_i(indexf - 0.5f);

    float3 w1 = (indexf - 0.5f) - float3(ijk000);
    float3 w0 = float3(1.f, 1.f, 1.f) - w1;
    float val;
    val  = w0.x * w0.y * w0.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(0, 0, 0));
    val += w1.x * w0.y * w0.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(1, 0, 0));
    val += w0.x * w1.y * w0.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(0, 1, 0));
    val += w1.x * w1.y * w0.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(1, 1, 0));
    val += w0.x * w0.y * w1.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(0, 0, 1));
    val += w1.x * w0.y * w1.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(1, 0, 1));
    val += w0.x * w1.y * w1.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(0, 1, 1));
    val += w1.x * w1.y * w1.z * getFloatAtIjk(grid_type, buf, root, ijk000 + int3(1, 1, 1));

    return val;
}

float srgb_to_linear1(float v)
{
    float ret;
    if (v <= 0.04045)
    {
        ret = v * (1.f / 12.92f);
    }
    else
    {
        ret = pow((v + 0.055f) * (1.f / 1.055f), 2.4f);
    }
    return ret;
}

float3 srgb_to_linear(float3 c)
{
    return float3(
        srgb_to_linear1(c.x),
        srgb_to_linear1(c.y),
        srgb_to_linear1(c.z)
    );
}

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(gParams.table, dispatchThreadID.x);
    uint locationIdx = groupID.y + gParams.blockIdxOffset;

    int4 location = int4(
        int(locationIdx % gParams.locationExtent.x) + gParams.locationOffset.x,
        int((locationIdx / gParams.locationExtent.x) % gParams.locationExtent.y) + gParams.locationOffset.y,
        int(locationIdx / (gParams.locationExtent.x * gParams.locationExtent.y)) + gParams.locationOffset.z,
        gParams.locationOffset.w
    );

    uint blockIdx = NvFlowLocationToBlockIdx(gTable, gParams.table, location);

    if (blockIdx >= gParams.table.numLocations)
    {
        return;
    }

    int3 vidx = (location.xyz << gParams.table.blockDimBits) + threadIdx;
    float3 vidxf = float3(vidx) + float3(0.5f, 0.5f, 0.5f);
    float3 worldPos = gParams.vidxToWorld * vidxf;

    float4 localPos = mul(float4(worldPos, 1.f), gParams.worldToLocal);
    float3 localPosTranslated = localPos.xyz - gParams.position;

    float opacity = 1.f;
    if (abs(localPosTranslated.x) < gParams.halfSize.x &&
        abs(localPosTranslated.y) < gParams.halfSize.y &&
        abs(localPosTranslated.z) < gParams.halfSize.z)
    {
        float4 coupleRate = gParams.coupleRate;
        float4 targetValue = gParams.targetValue;

        if (gParams.sourceIsVelocity != 0u)
        {
            pnanovdb_buf_t buf_coupleRateVelocities = gParams.range_coupleRateVelocities;
            if (buf_coupleRateVelocities.y > 0u)
            {
                float velCoupleRate = getFloat(buf_coupleRateVelocities, localPos);
                coupleRate.x = velCoupleRate;
                coupleRate.y = velCoupleRate;
                coupleRate.z = velCoupleRate;
            }
            pnanovdb_buf_t buf_coupleRateDivergences = gParams.range_coupleRateDivergences;
            if (buf_coupleRateDivergences.y > 0u)
            {
                coupleRate.w = getFloat(buf_coupleRateDivergences, localPos);
            }
            pnanovdb_buf_t buf_velocities = gParams.range_velocities;
            if (buf_velocities.y > 0u)
            {
                targetValue.xyz = getFloat3(buf_velocities, localPos);
            }
            pnanovdb_buf_t buf_divergences = gParams.range_divergences;
            if (buf_divergences.y > 0u)
            {
                targetValue.w = getFloat(buf_divergences, localPos);
            }
            if (gParams.velocityIsWorldSpace == 0u)
            {
                targetValue.xyz = mul(float4(targetValue.xyz, 0.f), gParams.localToWorldVelocity).xyz;
            }
        }
        else
        {
            pnanovdb_buf_t buf_coupleRateTemperatures = gParams.range_coupleRateTemperatures;
            if (buf_coupleRateTemperatures.y > 0u)
            {
                coupleRate.x = getFloat(buf_coupleRateTemperatures, localPos);
            }
            pnanovdb_buf_t buf_coupleRateFuels = gParams.range_coupleRateFuels;
            if (buf_coupleRateFuels.y > 0u)
            {
                coupleRate.y = getFloat(buf_coupleRateFuels, localPos);
            }
            pnanovdb_buf_t buf_coupleRateBurns = gParams.range_coupleRateBurns;
            if (buf_coupleRateBurns.y > 0u)
            {
                coupleRate.z = getFloat(buf_coupleRateBurns, localPos);
            }
            pnanovdb_buf_t buf_coupleRateSmokes = gParams.range_coupleRateSmokes;
            if (buf_coupleRateSmokes.y > 0u)
            {
                coupleRate.w = getFloat(buf_coupleRateSmokes, localPos);
            }
            pnanovdb_buf_t buf_temperatures = gParams.range_temperatures;
            if (buf_temperatures.y > 0u)
            {
                targetValue.x = getFloat(buf_temperatures, localPos);
            }
            pnanovdb_buf_t buf_fuels = gParams.range_fuels;
            if (buf_fuels.y > 0u)
            {
                targetValue.y = getFloat(buf_fuels, localPos);
            }
            pnanovdb_buf_t buf_burns = gParams.range_burns;
            if (buf_burns.y > 0u)
            {
                targetValue.z = getFloat(buf_burns, localPos);
            }
            pnanovdb_buf_t buf_smokes = gParams.range_smokes;
            if (buf_smokes.y > 0u)
            {
                targetValue.w = getFloat(buf_smokes, localPos);
            }
            pnanovdb_buf_t buf_rgba8 = gParams.range_rgba8s;
            if (buf_rgba8.y > 0)
            {
                targetValue = getRgba8(buf_rgba8, localPos);
            }
            if (gParams.needsSrgbConversion != 0u)
            {
                targetValue.xyz = srgb_to_linear(targetValue.xyz);
            }
        }
        targetValue *= gParams.targetValueScale;
        if (gParams.absoluteValue != 0u)
        {
            targetValue = abs(targetValue);
        }
        if (gParams.sourceIsVelocity != 0u && gParams.computeSpeed != 0u)
        {
            float speed = length(targetValue.xyz);
            targetValue.xyz = float3(speed, speed, speed);
        }

        pnanovdb_buf_t buf_distances = gParams.range_distances;
        if (buf_distances.y > 0u)
        {
            float dist = getFloatInterpolated(buf_distances, localPos);
            if (dist < gParams.minDistance || dist > gParams.maxDistance)
            {
                coupleRate = float4(0.f, 0.f, 0.f, 0.f);
            }
        }
        if (gParams.sourceIsVelocity == 0u)
        {
            float smoke = targetValue.w;
            if (smoke < gParams.minSmoke || smoke > gParams.maxSmoke)
            {
                coupleRate = float4(0.f, 0.f, 0.f, 0.f);
            }
        }

        float4 value = NvFlowLocalRead4f(valueIn, gTable, gParams.table, blockIdx, threadIdx);

        float4 valueRate = gParams.deltaTime * coupleRate * opacity;
        valueRate = max(float4(0.f, 0.f, 0.f, 0.f), valueRate);
        valueRate = min(float4(1.f, 1.f, 1.f, 1.f), valueRate);

        value += valueRate * (targetValue - value);

        int3 tempIdx = threadIdx + int3(
            (int(groupID.y & 15) << int(gParams.table.blockDimBits.x)),
            ((int(groupID.y >> 4u) & 15) << int(gParams.table.blockDimBits.y)),
            ((int(groupID.y >> 8u) & 15) << int(gParams.table.blockDimBits.z))
            );
        valueOut[tempIdx] = value;
        //NvFlowLocalWrite4f(valueOut, gTable, gParams.table, blockIdx, threadIdx, value);

        // mark as valid
        float voxelWeight = 1.f;
        NvFlowLocalWrite1f(voxelWeightOut, gTable, gParams.table, blockIdx, threadIdx, voxelWeight);
    }
}
