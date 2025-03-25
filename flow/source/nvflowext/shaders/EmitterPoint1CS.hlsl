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

ConstantBuffer<EmitterPointCS_Params> gParams;
ConstantBuffer<EmitterPointCS_SubStepParams> gSubStepParams;

StructuredBuffer<uint> gTable;
StructuredBuffer<uint> arrayValuesIn;

Texture3D<float4> valueIn;

RWStructuredBuffer<float4> oldValueOut;
RWStructuredBuffer<uint> keyOut;
RWStructuredBuffer<uint> valOut;
RWStructuredBuffer<uint> keyLowOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint fullIdx = dispatchThreadID.x + (gParams.pointBlockIdxOffset << 7u);
    uint pointIdx = fullIdx / gParams.voxelsPerPoint;
    uint subIdx = fullIdx % gParams.voxelsPerPoint;

    if (pointIdx >= gParams.range_positions.y)
    {
        return;
    }

    float4 positionLocal;
    positionLocal.x = asfloat(arrayValuesIn[gParams.range_positions.x + 3u * pointIdx + 0u]);
    positionLocal.y = asfloat(arrayValuesIn[gParams.range_positions.x + 3u * pointIdx + 1u]);
    positionLocal.z = asfloat(arrayValuesIn[gParams.range_positions.x + 3u * pointIdx + 2u]);
    positionLocal.w = 1.f;
    float4 position = mul(positionLocal, gParams.localToWorld);
    if (gSubStepParams.subStepIdx >= 1u)
    {
        float4 vel4 = float4(gSubStepParams.defaultVelocity, 0.f);
        if (pointIdx < gParams.range_velocities.y)
        {
            vel4.x = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * pointIdx + 0]);
            vel4.y = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * pointIdx + 1]);
            vel4.z = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * pointIdx + 2]);
        }
        if (gParams.velocityIsWorldSpace == 0u)
        {
            vel4 = mul(vel4, gParams.localToWorldVelocity);
        }
        vel4.xyz *= gSubStepParams.velocityScale;
        // displace back in time
        position.xyz -= gSubStepParams.subStepAccumDeltaTime * vel4.xyz;
    }

    float3 vidxf = position.xyz * gParams.worldToVidx;

    float3 vidxf_offset;
    if (gParams.voxelsPerPoint <= 1u)
    {
        vidxf_offset = float3(0.f, 0.f, 0.f);
    }
    else if (gParams.voxelsPerPoint <= 2u)
    {
        vidxf_offset = float3(0.5f, 0.f, 0.f);
    }
    else if (gParams.voxelsPerPoint <= 4u)
    {
        vidxf_offset = float3(0.5f, 0.5f, 0.f);
    }
    else //if (gParams.voxelsPerPoint <= 8u)
    {
        vidxf_offset = float3(0.5f, 0.5f, 0.5f);
    }
    int3 vidx = NvFlowFloor_i(vidxf - vidxf_offset);
    vidx.x += int(subIdx & 1);
    vidx.y += (int(subIdx & 2) >> 1);
    vidx.z += (int(subIdx & 4) >> 2);

    float3 targetVidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);
    float3 w3 = max(float3(0.f, 0.f, 0.f), float3(1.f, 1.f, 1.f) - abs(vidxf - targetVidxf));
    float weight = w3.x * w3.y * w3.z;

    int4 location = int4(vidx.xyz >> int3(gParams.table.blockDimBits), gParams.layerAndLevel);
    uint blockIdx = NvFlowLocationToBlockIdx(gTable, gParams.table, location);
    int3 threadIdx = vidx.xyz & int3(gParams.table.blockDimLessOne);

    uint threadIdx1D = uint(threadIdx.z << (gParams.table.blockDimBits.x + gParams.table.blockDimBits.y)) |
        uint(threadIdx.y << (gParams.table.blockDimBits.x)) |
        uint(threadIdx.x);

    uint weightui = uint(weight * 1023.f) & 1023u;
    uint key = (weightui << 22u) | threadIdx1D;

    keyOut[fullIdx] = key;
    valOut[fullIdx] = fullIdx;
    keyLowOut[fullIdx] = blockIdx;

    float4 oldValue = float4(0.f, 0.f, 0.f, 0.f);
    if (blockIdx != ~0u)
    {
        // fetch existing value
        oldValue = NvFlowLocalRead4f(valueIn, gTable, gParams.table, blockIdx, threadIdx);
    }
    oldValueOut[fullIdx] = oldValue;
}
