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

ConstantBuffer<EmitterBoxCS_Params> gParams;
StructuredBuffer<EmitterBoxCS_InstanceParams> gInstanceParams;
StructuredBuffer<float4> gPlanes;
StructuredBuffer<uint> gPlaneCounts;

StructuredBuffer<uint> gBlockIdx;
StructuredBuffer<uint2> gRange;
StructuredBuffer<uint> gInstanceId;

StructuredBuffer<uint> gTable;

Texture3D<float4> valueIn;

RWTexture3D<float4> valueOut;

float computeCoverage(uint instanceIdx, float3 worldPos, float3 halfSize)
{
    float4 localPos = mul(float4(worldPos, 1.f), gInstanceParams[instanceIdx].worldToLocal);
    float3 localPosTranslated = localPos.xyz - gInstanceParams[instanceIdx].position;
    bool inBox = (abs(localPosTranslated.x) < gInstanceParams[instanceIdx].halfSize.x &&
        abs(localPosTranslated.y) < gInstanceParams[instanceIdx].halfSize.y &&
        abs(localPosTranslated.z) < gInstanceParams[instanceIdx].halfSize.z);
    if (inBox)
    {
        bool inConvex = gInstanceParams[instanceIdx].clippingPlaneCountCount == 0u && gInstanceParams[instanceIdx].clippingPlaneCount == 0u;
        uint planeBaseIdx = 0u;
        if (gInstanceParams[instanceIdx].clippingPlaneCountCount == 0u && gInstanceParams[instanceIdx].clippingPlaneCount != 0u)
        {
            uint planeCount = gInstanceParams[instanceIdx].clippingPlaneCount;
            bool inConvexLocal = true;
            for (uint planeLocalIdx = 0u; planeLocalIdx < planeCount; planeLocalIdx++)
            {
                if (dot(localPos, gPlanes[gInstanceParams[instanceIdx].clippingPlanesOffset + planeBaseIdx + planeLocalIdx]) > 0.f)
                {
                    inConvexLocal = false;
                    break;
                }
            }
            if (inConvexLocal)
            {
                inConvex = true;
            }
            planeBaseIdx += planeCount;
        }
        for (uint planeCountIdx = 0u; planeCountIdx < gInstanceParams[instanceIdx].clippingPlaneCountCount; planeCountIdx++)
        {
            uint planeCount = gPlaneCounts[gInstanceParams[instanceIdx].clippingPlaneCountsOffset + planeCountIdx];
            bool inConvexLocal = true;
            for (uint planeLocalIdx = 0u; planeLocalIdx < planeCount; planeLocalIdx++)
            {
                if (dot(localPos, gPlanes[gInstanceParams[instanceIdx].clippingPlanesOffset + planeBaseIdx + planeLocalIdx]) > 0.f)
                {
                    inConvexLocal = false;
                    break;
                }
            }
            if (inConvexLocal)
            {
                inConvex = true;
                break;
            }
            planeBaseIdx += planeCount;
        }
        inBox = inBox && inConvex;
    }
    return inBox ? 1.f : 0.f;
}

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(gParams.table, dispatchThreadID.x);
    uint listIdx = groupID.y + gParams.blockIdxOffset;

    uint blockIdx = gBlockIdx[listIdx];

    int4 location = int4(0, 0, 0, 0);
    if (!NvFlowBlockIdxToLocation(gTable, gParams.table, blockIdx, location))
    {
        return;
    }

    uint2 range = gRange[listIdx];

    int3 vidx = (location.xyz << gParams.table.blockDimBits) + threadIdx;
    float3 vidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);

    bool valueFetched = false;
    float4 value = float4(0.f, 0.f, 0.f, 0.f);
    for (uint rangeIdx = range.x; rangeIdx < range.y; rangeIdx++)
    {
        uint instanceIdx = gInstanceId[rangeIdx];
        if (gInstanceParams[instanceIdx].enabled != 0u && gInstanceParams[instanceIdx].layerAndLevel == location.w)
        {
            float3 worldPos = gInstanceParams[instanceIdx].vidxToWorld * vidxf;

            float coverage = 0.f;
            if (gInstanceParams[instanceIdx].multisample != 0u)
            {
                float3 vidxToWorld = gInstanceParams[instanceIdx].vidxToWorld;
                float3 halfSize = gInstanceParams[instanceIdx].halfSize;
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(-0.25f, -0.25f, -0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(+0.25f, -0.25f, -0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(-0.25f, +0.25f, -0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(+0.25f, +0.25f, -0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(-0.25f, -0.25f, +0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(+0.25f, -0.25f, +0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(-0.25f, +0.25f, +0.25f) * vidxToWorld, halfSize);
                coverage += 0.125f * computeCoverage(instanceIdx, worldPos + float3(+0.25f, +0.25f, +0.25f) * vidxToWorld, halfSize);
            }
            else
            {
                coverage = computeCoverage(instanceIdx, worldPos, gInstanceParams[instanceIdx].halfSize);
            }

            if (coverage > 0.f)
            {
                if (!valueFetched)
                {
                    valueFetched = true;
                    value = NvFlowLocalRead4f(valueIn, gTable, gParams.table, blockIdx, threadIdx);
                }

                float4 valueRate = gParams.deltaTime * gInstanceParams[instanceIdx].coupleRate * coverage;
                valueRate = max(float4(0.f, 0.f, 0.f, 0.f), valueRate);
                valueRate = min(float4(1.f, 1.f, 1.f, 1.f), valueRate);

                float4 targetValue = gInstanceParams[instanceIdx].targetValue;
                if (gInstanceParams[instanceIdx].physicsVelocityScale != 0.f)
                {
                    float4 localPos = mul(float4(worldPos, 1.f), gInstanceParams[instanceIdx].worldToLocal);
                    float4 worldOld = mul(localPos, gInstanceParams[instanceIdx].localToWorldOld);
                    float3 physicsVelocity = (worldPos - worldOld.xyz) * gInstanceParams[instanceIdx].physicsDeltaTimeInv;
                    targetValue.xyz += gInstanceParams[instanceIdx].physicsVelocityScale * physicsVelocity;
                }

                value += valueRate * (targetValue - value);
            }
        }
    }
    if (valueFetched)
    {
        NvFlowLocalWrite4f(valueOut, gTable, gParams.table, blockIdx, threadIdx, value);
    }
}
