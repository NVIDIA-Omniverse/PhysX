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

#include "EllipsoidRasterParams.h"

ConstantBuffer<EllipsoidRasterCS_GlobalParams> globalParamsIn;
StructuredBuffer<EllipsoidRasterCS_LayerParams> layerParamsIn;

StructuredBuffer<uint> tableIn;

StructuredBuffer<float> arrayValuesIn1;
StructuredBuffer<float4> arrayValuesIn4;

StructuredBuffer<uint> blockIdxIn;
StructuredBuffer<uint2> rangeIn;
StructuredBuffer<uint> instanceIdIn;

RWTexture3D<float> valueOut;

groupshared uint sparticleIdx[256];

groupshared uint sdata0[256u];
groupshared uint sdata1[256u];

groupshared uint stotalCount;

uint blockScan(uint threadIdx, uint val)
{
    uint localVal = val;
    sdata0[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 1) localVal += sdata0[threadIdx - 1];
    if (threadIdx >= 2) localVal += sdata0[threadIdx - 2];
    if (threadIdx >= 3) localVal += sdata0[threadIdx - 3];
    sdata1[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 4) localVal += sdata1[threadIdx - 4];
    if (threadIdx >= 8) localVal += sdata1[threadIdx - 8];
    if (threadIdx >= 12) localVal += sdata1[threadIdx - 12];
    sdata0[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 16) localVal += sdata0[threadIdx - 16];
    if (threadIdx >= 32) localVal += sdata0[threadIdx - 32];
    if (threadIdx >= 48) localVal += sdata0[threadIdx - 48];
    sdata1[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 64) localVal += sdata1[threadIdx - 64];
    if (threadIdx >= 128) localVal += sdata1[threadIdx - 128];
    if (threadIdx >= 192) localVal += sdata1[threadIdx - 192];

    // compute totalCount
    if (threadIdx == 0u)
    {
        stotalCount = sdata1[63] + sdata1[127] + sdata1[191] + sdata1[255];
    }

    return localVal;
}

float4 readPosition(EllipsoidRasterCS_LayerParams layerParams, uint particleIdx)
{
    float4 ret = float4(0.f, 0.f, 0.f, 0.f);
    if (particleIdx < layerParams.range_positions.y)
    {
        ret = arrayValuesIn4[particleIdx + layerParams.range_positions.x];
    }
    else if (particleIdx < (layerParams.range_positions.y + layerParams.range_positionFloat3s.y))
    {
        ret.x = arrayValuesIn1[3u * (particleIdx - layerParams.range_positions.y) + layerParams.range_positionFloat3s.x + 0u];
        ret.y = arrayValuesIn1[3u * (particleIdx - layerParams.range_positions.y) + layerParams.range_positionFloat3s.x + 1u];
        ret.z = arrayValuesIn1[3u * (particleIdx - layerParams.range_positions.y) + layerParams.range_positionFloat3s.x + 2u];
        ret.w = 1.f;
    }
    return ret;
}

float4 readAnisotropyE1(EllipsoidRasterCS_LayerParams layerParams, uint particleIdx)
{
    return particleIdx < layerParams.range_anisotropyE1s.y ? arrayValuesIn4[particleIdx + layerParams.range_anisotropyE1s.x] : float4(1.f, 0.f, 0.f, 1.f);
}

float4 readAnisotropyE2(EllipsoidRasterCS_LayerParams layerParams, uint particleIdx)
{
    return particleIdx < layerParams.range_anisotropyE2s.y ? arrayValuesIn4[particleIdx + layerParams.range_anisotropyE2s.x] : float4(0.f, 1.f, 0.f, 1.f);
}

float4 readAnisotropyE3(EllipsoidRasterCS_LayerParams layerParams, uint particleIdx)
{
    return particleIdx < layerParams.range_anisotropyE3s.y ? arrayValuesIn4[particleIdx + layerParams.range_anisotropyE3s.x] : float4(0.f, 0.f, 1.f, 1.f);
}

int3 computeThreadIdx(NvFlowSparseLevelParams tableParams, uint threadIdx1D)
{
    // make each workgroup 8x8x4
    int3 minorIdx3 = int3(
        int((threadIdx1D) & 7u),
        int((threadIdx1D >> 3u) & 7u),
        int((threadIdx1D >> 6u) & 3u)
    );
    uint majorIdx = threadIdx1D >> 8u;
    int3 majorIdx3 = int3(
        int(int(majorIdx) & int(tableParams.blockDimLessOne.x >> 3u)),
        int((int(majorIdx) >> int(tableParams.blockDimBits.x - 3u)) & int(tableParams.blockDimLessOne.y >> 3u)),
        int((int(majorIdx) >> int(tableParams.blockDimBits.x + tableParams.blockDimBits.y - 6u)) & int(tableParams.blockDimLessOne.z >> 2u))
    );
    return (majorIdx3 << int3(3, 3, 2)) + minorIdx3;
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = computeThreadIdx(globalParamsIn.table, dispatchThreadID.x);
    uint listIdx = groupID.y + globalParamsIn.blockIdxOffset;
    uint localThreadIdx = (dispatchThreadID.x & 255u);

    uint blockIdx = blockIdxIn[listIdx];

    int4 location = int4(0, 0, 0, 0);
    if (!NvFlowBlockIdxToLocation(tableIn, globalParamsIn.table, blockIdx, location))
    {
        return;
    }
    int3 vidx = (location.xyz << globalParamsIn.table.blockDimBits) + threadIdx;
    float3 vidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);

    uint2 range = rangeIn[listIdx];

    uint layerParamIdx = NvFlowGetLayerParamIdx(tableIn, globalParamsIn.table, blockIdx);

    float4 worldPos = float4(vidxf * layerParamsIn[layerParamIdx].vidxToWorld, 1.f);

    int3 threadIdxMin = computeThreadIdx(globalParamsIn.table, dispatchThreadID.x & ~255);
    int3 threadIdxMax = computeThreadIdx(globalParamsIn.table, dispatchThreadID.x | 255);
    int3 vidxMin = (location.xyz << globalParamsIn.table.blockDimBits) + threadIdxMin;
    int3 vidxMax = (location.xyz << globalParamsIn.table.blockDimBits) + threadIdxMax;
    float3 vidxMinf = float3(vidxMin);
    float3 vidxMaxf = float3(vidxMax) + float3(1.f, 1.f, 1.f);
    float3 boundsMinWorld = float4(vidxMinf * layerParamsIn[layerParamIdx].vidxToWorld, 1.f).xyz;
    float3 boundsMaxWorld = float4(vidxMaxf * layerParamsIn[layerParamIdx].vidxToWorld, 1.f).xyz;

    //float3 blockSizeWorld = layerParamsIn[layerParamIdx].blockSizeWorld;
    //float3 boundsMinWorld = float3(location.xyz) * blockSizeWorld;
    //float3 boundsMaxWorld = float3(location.xyz + int3(1, 1, 1)) * blockSizeWorld;

    float rasterRadiusScale = layerParamsIn[layerParamIdx].rasterRadiusScale;

    float sum = 0.f;

    uint batchCount = (range.y - range.x + 255u) / 256u;
    uint rangeIdx = range.x + localThreadIdx;
    for (uint batchIdx = 0; batchIdx < batchCount; batchIdx++)
    {
        uint pred = 0u;
        uint particleIdx = 0u;
        float4 posValue = float4(0.f, 0.f, 0.f, 0.f);
        if (rangeIdx < range.y)
        {
            particleIdx = instanceIdIn[rangeIdx];
            float4 position = readPosition(layerParamsIn[layerParamIdx], particleIdx);
            float4 e1 = readAnisotropyE1(layerParamsIn[layerParamIdx], particleIdx);
            float4 e2 = readAnisotropyE2(layerParamsIn[layerParamIdx], particleIdx);
            float4 e3 = readAnisotropyE3(layerParamsIn[layerParamIdx], particleIdx);
            float4 xTransform = float4(e1.x * e1.w, e2.x * e2.w, e3.x * e3.w, position.x);
            float4 yTransform = float4(e1.y * e1.w, e2.y * e2.w, e3.y * e3.w, position.y);
            float4 zTransform = float4(e1.z * e1.w, e2.z * e2.w, e3.z * e3.w, position.z);

            float4 minWorld = float4(0.f, 0.f, 0.f, 0.f);
            float4 maxWorld = float4(0.f, 0.f, 0.f, 0.f);
            for (int k = -1; k <= +1; k += 2)
            {
                for (int j = -1; j <= +1; j += 2)
                {
                    for (int i = -1; i <= +1; i += 2)
                    {
                        float4 boxCoord = float4(
                            rasterRadiusScale * float(i),
                            rasterRadiusScale * float(j),
                            rasterRadiusScale * float(k),
                            1.f
                            );
                        float4 worldCoord = float4(
                            dot(xTransform, boxCoord),
                            dot(yTransform, boxCoord),
                            dot(zTransform, boxCoord),
                            1.f
                            );
                        if (minWorld.w == 0.f)
                        {
                            minWorld = worldCoord;
                            maxWorld = worldCoord;
                        }
                        else
                        {
                            minWorld = min(minWorld, worldCoord);
                            maxWorld = max(maxWorld, worldCoord);
                        }
                    }
                }
            }

            if (!(boundsMinWorld.x > maxWorld.x || minWorld.x > boundsMaxWorld.x ||
                boundsMinWorld.y > maxWorld.y || minWorld.y > boundsMaxWorld.y ||
                boundsMinWorld.z > maxWorld.z || minWorld.z > boundsMaxWorld.z))
            {
                pred = 1u;
            }
        }

        uint allocIdx = blockScan(localThreadIdx, pred) - 1u;
        if (bool(pred))
        {
            sparticleIdx[allocIdx] = particleIdx;
        }

        GroupMemoryBarrierWithGroupSync();

        for (uint sharedIdx = 0; sharedIdx < stotalCount; sharedIdx++)
        {
            uint localParticleIdx = sparticleIdx[sharedIdx];

            float4 position = readPosition(layerParamsIn[layerParamIdx], localParticleIdx);
            float4 e1 = readAnisotropyE1(layerParamsIn[layerParamIdx], localParticleIdx);
            float4 e2 = readAnisotropyE2(layerParamsIn[layerParamIdx], localParticleIdx);
            float4 e3 = readAnisotropyE3(layerParamsIn[layerParamIdx], localParticleIdx);
            //float4 xTransform = float4(e1.x * e1.w, e2.x * e2.w, e3.x * e3.w, position.x);
            //float4 yTransform = float4(e1.y * e1.w, e2.y * e2.w, e3.y * e3.w, position.y);
            //float4 zTransform = float4(e1.z * e1.w, e2.z * e2.w, e3.z * e3.w, position.z);
            float4 xTransformInv = float4(e1.x / e1.w, e1.y / e1.w, e1.z / e1.w, 0.f);
            float4 yTransformInv = float4(e2.x / e2.w, e2.y / e2.w, e2.z / e2.w, 0.f);
            float4 zTransformInv = float4(e3.x / e3.w, e3.y / e3.w, e3.z / e3.w, 0.f);
            xTransformInv.w = -dot(xTransformInv.xyz, position.xyz);
            yTransformInv.w = -dot(yTransformInv.xyz, position.xyz);
            zTransformInv.w = -dot(zTransformInv.xyz, position.xyz);

            float4 localWorldPos = worldPos;
            float3 localPos = float3(
                dot(xTransformInv, worldPos),
                dot(yTransformInv, worldPos),
                dot(zTransformInv, worldPos)
                );

            localPos.xyz *= (1.f / rasterRadiusScale);

            float distance = length(localPos.xyz);
            if (layerParamsIn[layerParamIdx].sdfMode != 0u)
            {
                float sdf = max(2.f - distance, 0.f);
                sum = max(sum, sdf);
            }
            else
            {
                if (distance < 1.f)
                {
                    sum += layerParamsIn[layerParamIdx].density;
                }
            }
        }

        GroupMemoryBarrierWithGroupSync();

        rangeIdx += 256u;
    }

    NvFlowLocalWrite1f(valueOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, sum);
}
