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

ConstantBuffer<EmitterMeshCS_Params> gParams;
ConstantBuffer<EmitterMeshCS_SubStepParams> gSubStepParams;

StructuredBuffer<uint> arrayValuesIn;
StructuredBuffer<uint4> bounds4In;

RWStructuredBuffer<uint> sortKeyOut;
RWStructuredBuffer<uint> sortValOut;

float4 fetchWorldPos(uint faceVertexIdx)
{
    float4 posLocal = float4(
        asfloat(arrayValuesIn[gParams.range_positions.x + 3u * faceVertexIdx + 0u]),
        asfloat(arrayValuesIn[gParams.range_positions.x + 3u * faceVertexIdx + 1u]),
        asfloat(arrayValuesIn[gParams.range_positions.x + 3u * faceVertexIdx + 2u]),
        1.f);
    float4 posWorld = mul(posLocal, gParams.localToWorld);
    if (gSubStepParams.subStepIdx >= 1)
    {
        posWorld *= (1.f - gSubStepParams.weight);
        posWorld += gSubStepParams.weight * mul(posLocal, gParams.localToWorldOld);
    }
    if (gSubStepParams.subStepIdx >= 1)
    {
        float4 vel4 = float4(gSubStepParams.defaultVelocity, 0.f);
        if (faceVertexIdx < gParams.range_velocities.y)
        {
            vel4.x = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 0]);
            vel4.y = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 1]);
            vel4.z = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 2]);
        }
        if (gParams.velocityIsWorldSpace == 0u)
        {
            vel4 = mul(vel4, gParams.localToWorldVelocity);
        }
        vel4.xyz *= gSubStepParams.velocityScale;
        // displace back in time
        posWorld.xyz -= gSubStepParams.subStepAccumDeltaTime * vel4.xyz;
    }
    return posWorld;
}

groupshared float4 sboundsMin[256];
groupshared float4 sboundsMax[256];

uint mortonShift(uint x)
{
    x = (x | (x << 16u)) & 0x030000FF;
    x = (x | (x <<  8u)) & 0x0300F00F;
    x = (x | (x <<  4u)) & 0x030C30C3;
    x = (x | (x <<  2u)) & 0x09249249;
    return x;
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID, uint3 groupThreadID : SV_GroupThreadID)
{
    uint gthreadIdx = dispatchThreadID.x + (gParams.faceBlockIdx1Offset << 8u);
    uint sthreadIdx = groupThreadID.x;

    float4 bbox_min = gParams.emptyMin;
    float4 bbox_max = gParams.emptyMax;
    if (gthreadIdx < gParams.face1Count)
    {
        uint faceIdx = gthreadIdx;

        uint faceVertexCount = arrayValuesIn[gParams.range_faceVertexCounts.x + faceIdx];
        uint faceVertexStart = arrayValuesIn[gParams.range_faceVertexStarts.x + faceIdx];
        if (faceVertexCount > 0u)
        {
            uint faceVertexIdx = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart];
            float4 posWorld = fetchWorldPos(faceVertexIdx);

            bbox_min = posWorld;
            bbox_max = posWorld;
        }
        for (uint faceVertexIdxIdx = 1u; faceVertexIdxIdx < faceVertexCount; faceVertexIdxIdx++)
        {
            uint faceVertexIdx = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart + faceVertexIdxIdx];
            float4 posWorld = fetchWorldPos(faceVertexIdx);

            bbox_min = min(bbox_min, posWorld);
            bbox_max = max(bbox_max, posWorld);
        }
    }

    float4 global_bbox_min = asfloat(bounds4In[0u]);
    float4 global_bbox_max = asfloat(bounds4In[1u]);

    float3 bbox_center = 0.5f * (bbox_min.xyz + bbox_max.xyz);
    float3 coordNorm = (bbox_center.xyz - global_bbox_min.xyz) / (global_bbox_max.xyz - global_bbox_min.xyz);
    uint3 coordi = uint3(round(1023.f * coordNorm));

    uint sortKey = mortonShift(coordi.x) | (mortonShift(coordi.y) << 1u) | (mortonShift(coordi.y) << 2u);

    if (gthreadIdx < gParams.face1Count)
    {
        sortKeyOut[gthreadIdx] = sortKey;
        sortValOut[gthreadIdx] = gthreadIdx;
    }
}
