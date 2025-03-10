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

StructuredBuffer<uint> sortKeyIn;
StructuredBuffer<uint> sortValIn;

RWStructuredBuffer<uint4> bounds1Out;
RWStructuredBuffer<uint4> bounds2Out;

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

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID, uint3 groupThreadID : SV_GroupThreadID)
{
    uint gthreadIdx = dispatchThreadID.x + (gParams.faceBlockIdx1Offset << 8u);
    uint sthreadIdx = groupThreadID.x;

    float4 bbox_min = gParams.emptyMin;
    float4 bbox_max = gParams.emptyMax;
    if (gthreadIdx < gParams.face1Count)
    {
        uint faceIdx = sortValIn[gthreadIdx];

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

        bounds1Out[2u * gthreadIdx + 0u] = asuint(bbox_min);
        bounds1Out[2u * gthreadIdx + 1u] = asuint(bbox_max);
    }

    sboundsMin[sthreadIdx] = bbox_min;
    sboundsMax[sthreadIdx] = bbox_max;

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 64u)
    {
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 64u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 128u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 192u]);

        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 64u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 128u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 192u]);

        sboundsMin[sthreadIdx] = bbox_min;
        sboundsMax[sthreadIdx] = bbox_max;
    }

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 16u)
    {
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 16u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 32u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 48u]);

        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 16u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 32u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 48u]);

        sboundsMin[sthreadIdx] = bbox_min;
        sboundsMax[sthreadIdx] = bbox_max;
    }

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 4u)
    {
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 4u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 8u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 12u]);

        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 4u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 8u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 12u]);

        sboundsMin[sthreadIdx] = bbox_min;
        sboundsMax[sthreadIdx] = bbox_max;
    }

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 1u)
    {
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 1u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 2u]);
        bbox_min = min(bbox_min, sboundsMin[sthreadIdx + 3u]);

        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 1u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 2u]);
        bbox_max = max(bbox_max, sboundsMax[sthreadIdx + 3u]);

        uint4 write_min = asuint(bbox_min);
        uint4 write_max = asuint(bbox_max);

        bounds2Out[2u * (gthreadIdx >> 8u) + 0u] = write_min;
        bounds2Out[2u * (gthreadIdx >> 8u) + 1u] = write_max;
    }
}
