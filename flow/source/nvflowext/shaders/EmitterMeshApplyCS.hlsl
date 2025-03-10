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

StructuredBuffer<uint> gTable;
StructuredBuffer<uint> arrayValuesIn;

Texture3D<float4> valueIn;
Texture3D<uint> closestIn;

RWTexture3D<float4> valueOut;

bool rayTriangleNearest(float3 o, float3 d, float3 p0, float3 p1, float3 p2, out float3 uvt)
{
    // ray triangle intersection
    float3 e1 = p1 - p0;
    float3 e2 = p2 - p0;
    float3 q = cross(d, e2);
    float a = dot(e1, q);
    if (a == 0.f)
    {
        uvt = float3(0.f, 0.f, 0.f);
        return false;
    }
    float f = 1.f / a;
    float3 s = o - p0;
    float u = f * dot(s, q);
    float3 r = cross(s, e1);
    float v = f * dot(d, r);
    float t = f * dot(e2, r);

    // clamping to triangle
    float3 p = o + d * t;
    if (1.f - u - v < 0.f)
    {
        float edge = dot(p - p1, p2 - p1) / dot(p2 - p1, p2 - p1);
        edge = clamp(edge, 0.f, 1.f);
        u = 1.f - edge;
        v = edge;
    }
    else if (u < 0.f)
    {
        float edge = dot(p - p2, p0 - p2) / dot(p0 - p2, p0 - p2);
        edge = clamp(edge, 0.f, 1.f);
        u = 0.f;
        v = 1.f - edge;
    }
    else if (v < 0.f)
    {
        float edge = dot(p - p0, p1 - p0) / dot(p1 - p0, p1 - p0);
        edge = clamp(edge, 0.f, 1.f);
        u = edge;
        v = 0.f;
    }

    uvt = float3(u, v, t);
    return true;
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

void fetchAttributeValue(uint faceVertexIdx, uint attributeIdx, inout float4 coupleRate, inout float4 targetValue)
{
    if (gParams.isVelocity != 0u)
    {
        if (attributeIdx < gParams.range_coupleRateVelocities.y)
        {
            float velCoupleRate = asfloat(arrayValuesIn[gParams.range_coupleRateVelocities.x + attributeIdx]);
            coupleRate.x = velCoupleRate;
            coupleRate.y = velCoupleRate;
            coupleRate.z = velCoupleRate;
        }
        if (attributeIdx < gParams.range_coupleRateDivergences.y)
        {
            coupleRate.w = asfloat(arrayValuesIn[gParams.range_coupleRateDivergences.x + attributeIdx]);
        }
        if (faceVertexIdx < gParams.range_velocities.y)
        {
            targetValue.x = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 0]);
            targetValue.y = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 1]);
            targetValue.z = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 2]);
        }
        if (attributeIdx < gParams.range_divergences.y)
        {
            targetValue.w = asfloat(arrayValuesIn[gParams.range_divergences.x + attributeIdx]);
        }
        if (gParams.velocityIsWorldSpace == 0u)
        {
            targetValue.xyz = mul(float4(targetValue.xyz, 0.f), gParams.localToWorldVelocity).xyz;
        }
    }
    else
    {
        if (attributeIdx < gParams.range_coupleRateTemperatures.y)
        {
            coupleRate.x = asfloat(arrayValuesIn[gParams.range_coupleRateTemperatures.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_coupleRateFuels.y)
        {
            coupleRate.y = asfloat(arrayValuesIn[gParams.range_coupleRateFuels.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_coupleRateBurns.y)
        {
            coupleRate.z = asfloat(arrayValuesIn[gParams.range_coupleRateBurns.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_coupleRateSmokes.y)
        {
            coupleRate.w = asfloat(arrayValuesIn[gParams.range_coupleRateSmokes.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_temperatures.y)
        {
            targetValue.x = asfloat(arrayValuesIn[gParams.range_temperatures.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_fuels.y)
        {
            targetValue.y = asfloat(arrayValuesIn[gParams.range_fuels.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_burns.y)
        {
            targetValue.z = asfloat(arrayValuesIn[gParams.range_burns.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_smokes.y)
        {
            targetValue.w = asfloat(arrayValuesIn[gParams.range_smokes.x + attributeIdx]);
        }
        if (attributeIdx < gParams.range_colors.y)
        {
            targetValue.x = asfloat(arrayValuesIn[gParams.range_colors.x + 3 * attributeIdx + 0]);
            targetValue.y = asfloat(arrayValuesIn[gParams.range_colors.x + 3 * attributeIdx + 1]);
            targetValue.z = asfloat(arrayValuesIn[gParams.range_colors.x + 3 * attributeIdx + 2]);
        }
        if (gParams.needsSrgbConversion != 0u)
        {
            targetValue.xyz = srgb_to_linear(targetValue.xyz);
        }
    }
}

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

void fetchFaceValues(uint faceIdx, int3 vidx, inout float4 coupleRate, inout float4 targetValue)
{
    float3 vidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);
    float3 cellPosWorld = vidxf * gParams.vidxToWorld;

    float minDist = 0.f;
    uint3 minAttributeIdx = uint3(~0u, ~0u, ~0u);
    uint3 minFaceVertexIdx = uint3(~0u, ~0u, ~0u);
    float3 minUvt = float3(0.f, 0.f, 0.f);

    uint faceVertexCount = arrayValuesIn[gParams.range_faceVertexCounts.x + faceIdx];
    uint faceVertexStart = arrayValuesIn[gParams.range_faceVertexStarts.x + faceIdx];
    if (faceVertexCount >= 3u)
    {
        uint faceVertexIdx0 = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart];
        float4 posWorld0 = fetchWorldPos(faceVertexIdx0);

        for (uint triangleIdx = 0u; triangleIdx < (faceVertexCount - 2u); triangleIdx++)
        {
            uint faceVertexIdx1 = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart + triangleIdx + 1u];
            uint faceVertexIdx2 = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart + triangleIdx + 2u];
            float4 posWorld1 = fetchWorldPos(faceVertexIdx1);
            float4 posWorld2 = fetchWorldPos(faceVertexIdx2);

            float3 n = cross(posWorld1.xyz - posWorld0.xyz, posWorld2.xyz - posWorld0.xyz);
            float3 uvt;
            bool valid = rayTriangleNearest(cellPosWorld, n, posWorld0.xyz, posWorld1.xyz, posWorld2.xyz, uvt);
            if (valid)
            {
                float3 nearest = (1.f - uvt.x - uvt.y) * posWorld0.xyz + uvt.x * posWorld1.xyz + uvt.y * posWorld2.xyz;
                float3 diff = cellPosWorld - nearest;
                float dist = length(diff);
                if (dist < minDist || minAttributeIdx.x == ~0u)
                {
                    minDist = dist;
                    minAttributeIdx = uint3(faceVertexStart, faceVertexStart + triangleIdx + 1u, faceVertexStart + triangleIdx + 2u);
                    minFaceVertexIdx = uint3(faceVertexIdx0, faceVertexIdx1, faceVertexIdx2);
                    minUvt = uvt;
                }
            }
        }
    }

    if (minAttributeIdx.x != ~0u)
    {
        float4 targetValue0 = gParams.defaultTargetValue;
        float4 coupleRate0 = gParams.defaultCoupleRate;
        fetchAttributeValue(minFaceVertexIdx.x, minAttributeIdx.x, coupleRate0, targetValue0);
        float4 targetValue1 = gParams.defaultTargetValue;
        float4 coupleRate1 = gParams.defaultCoupleRate;
        fetchAttributeValue(minFaceVertexIdx.y, minAttributeIdx.y, coupleRate1, targetValue1);
        float4 targetValue2 = gParams.defaultTargetValue;
        float4 coupleRate2 = gParams.defaultCoupleRate;
        fetchAttributeValue(minFaceVertexIdx.z, minAttributeIdx.z, coupleRate2, targetValue2);

        coupleRate = (1.f - minUvt.x - minUvt.y) * coupleRate0 + minUvt.x * coupleRate1 + minUvt.y * coupleRate2;
        targetValue = (1.f - minUvt.x - minUvt.y) * targetValue0 + minUvt.x * targetValue1 + minUvt.y * targetValue2;
    }
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID, uint3 groupThreadID : SV_GroupThreadID)
{
    uint sthreadIdx = groupThreadID.x;

    int3 threadIdx = NvFlowComputeThreadIdx(gParams.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + gParams.blockIdxOffset;
    if (gParams.dispatchBoundsLocations != 0u)
    {
        // intepret blockIdx as locationIdx in bounds instead
        uint locationIdx = blockIdx;
        int4 location = int4(
            int(locationIdx % gParams.locationExtent.x) + gParams.locationOffset.x,
            int((locationIdx / gParams.locationExtent.x) % gParams.locationExtent.y) + gParams.locationOffset.y,
            int(locationIdx / (gParams.locationExtent.x * gParams.locationExtent.y)) + gParams.locationOffset.z,
            gParams.locationOffset.w
            );
        blockIdx = NvFlowLocationToBlockIdx(gTable, gParams.table, location);
    }

    int4 location = int4(0, 0, 0, 0);
    if (!NvFlowBlockIdxToLocation(gTable, gParams.table, blockIdx, location))
    {
        return;
    }
    if (location.w != gParams.layerAndLevel)
    {
        return;
    }

    int3 readIdx = NvFlowSingleVirtualToReal(gTable, gParams.table, blockIdx, threadIdx).xyz;

    uint closestFaceIdx = closestIn[readIdx];
    if (closestFaceIdx != ~0u)
    {
        float4 value = valueIn[readIdx];

        float4 coupleRate = gParams.defaultCoupleRate;
        float4 targetValue = gParams.defaultTargetValue;

        int3 vidx = (location.xyz << int3(gParams.table.blockDimBits)) + threadIdx;
        fetchFaceValues(closestFaceIdx, vidx, coupleRate, targetValue);

        targetValue *= gParams.targetValueScale;
        if (gParams.physicsVelocityScale != 0.f)
        {
            float3 vidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);
            float3 worldPos = gParams.vidxToWorld * vidxf;
            float4 localPos = mul(float4(worldPos, 1.f), gParams.worldToLocal);
            float4 worldOld = mul(localPos, gParams.localToWorldOld);
            float3 physicsVelocity = (worldPos - worldOld.xyz) * gParams.physicsDeltaTimeInv;
            targetValue.xyz += gParams.physicsVelocityScale * physicsVelocity;
        }

        //// debug colors
        //if (gParams.isVelocity == 0u)
        //{
        //    if (testCount < 1u) targetValue.xyz = float3(1.f, 0.f, 0.f);
        //    else if (testCount < 2u) targetValue.xyz = float3(1.f, 1.f, 0.f);
        //    else if (testCount < 4u) targetValue.xyz = float3(0.f, 1.f, 0.f);
        //    else if (testCount < 8u) targetValue.xyz = float3(0.f, 1.f, 1.f);
        //    else if (testCount < 16u) targetValue.xyz = float3(0.f, 0.f, 1.f);
        //    else targetValue.xyz = float3(1.f, 0.f, 1.f);
        //    targetValue.w = 1.f;
        //    coupleRate.xyz = float3(100.f, 100.f, 100.f);
        //    coupleRate.w = 1.f;
        //}

        float4 valueRate = gParams.deltaTime * coupleRate;
        valueRate = max(float4(0.f, 0.f, 0.f, 0.f), valueRate);
        valueRate = min(float4(1.f, 1.f, 1.f, 1.f), valueRate);

        value += valueRate * (targetValue - value);

        NvFlowLocalWrite4f(valueOut, gTable, gParams.table, blockIdx, threadIdx, value);
    }
}
