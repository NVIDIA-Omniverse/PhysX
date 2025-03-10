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

ConstantBuffer<EmitterTextureCS_Params> gParams;

StructuredBuffer<uint> gTable;

StructuredBuffer<uint> arrayValuesIn;

Texture3D<float4> valueIn;

RWTexture3D<float4> valueOut;

float readArrayValue(uint idx)
{
    return f16tof32(arrayValuesIn[idx >> 1u] >> ((idx & 1) << 4));
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

uint computeTex1D(int3 texIdx)
{
    uint texIdx1D;
    if (gParams.textureIsColumnMajor == 0u)
    {
        texIdx1D = uint((texIdx.z * int(gParams.textureHeight) + texIdx.y) * int(gParams.textureWidth) + texIdx.x) + gParams.textureFirstElement;
    }
    else
    {
        texIdx1D = uint((texIdx.x * int(gParams.textureHeight) + texIdx.y) * int(gParams.textureDepth) + texIdx.z) + gParams.textureFirstElement;
    }
    return texIdx1D;
}

float lookupSdf(int3 texIdx)
{
    // clamp so fetch is always valid
    texIdx.x = max(0, min(texIdx.x, int(gParams.textureWidth) - 1));
    texIdx.y = max(0, min(texIdx.y, int(gParams.textureHeight) - 1));
    texIdx.z = max(0, min(texIdx.z, int(gParams.textureDepth) - 1));

    uint texIdx1D = computeTex1D(texIdx);
    texIdx1D = min(texIdx1D, gParams.range_distances.y - 1u);

    float dist = readArrayValue(texIdx1D + gParams.range_distances.x);
    return dist;
}

float sampleSdf(float3 uvw)
{
    float3 dimf = float3(
        float(gParams.textureWidth),
        float(gParams.textureHeight),
        float(gParams.textureDepth)
        );
    int3 texIdx0 = int3(floor(uvw * dimf - 0.5f));
    float3 w1 = (uvw * dimf - 0.5f) - float3(texIdx0);
    float3 w0 = float3(1.f, 1.f, 1.f) - w1;
    float dist;
    dist  = w0.x * w0.y * w0.z * lookupSdf(texIdx0 + int3(0, 0, 0));
    dist += w1.x * w0.y * w0.z * lookupSdf(texIdx0 + int3(1, 0, 0));
    dist += w0.x * w1.y * w0.z * lookupSdf(texIdx0 + int3(0, 1, 0));
    dist += w1.x * w1.y * w0.z * lookupSdf(texIdx0 + int3(1, 1, 0));
    dist += w0.x * w0.y * w1.z * lookupSdf(texIdx0 + int3(0, 0, 1));
    dist += w1.x * w0.y * w1.z * lookupSdf(texIdx0 + int3(1, 0, 1));
    dist += w0.x * w1.y * w1.z * lookupSdf(texIdx0 + int3(0, 1, 1));
    dist += w1.x * w1.y * w1.z * lookupSdf(texIdx0 + int3(1, 1, 1));
    return dist;
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

    float4 localPosTranslated = mul(float4(worldPos, 1.f), gParams.worldToLocal);
    localPosTranslated.xyz -= gParams.position;

    float opacity = 1.f;
    if (abs(localPosTranslated.x) < gParams.halfSize.x &&
        abs(localPosTranslated.y) < gParams.halfSize.y &&
        abs(localPosTranslated.z) < gParams.halfSize.z)
    {
        float3 uvw = 0.5f * (localPosTranslated.xyz) * gParams.halfSizeInv + 0.5f;
        int3 texIdx = int3(
            int(floor(uvw.x * float(gParams.textureWidth))),
            int(floor(uvw.y * float(gParams.textureHeight))),
            int(floor(uvw.z * float(gParams.textureDepth)))
        );
        float4 coupleRate = gParams.coupleRate;
        float4 targetValue = gParams.targetValue;
        if (gParams.textureWidth > 0u &&
            gParams.textureHeight > 0u &&
            gParams.textureDepth > 0u &&
            texIdx.x >= 0 && texIdx.x < int(gParams.textureWidth) &&
            texIdx.y >= 0 && texIdx.y < int(gParams.textureHeight) &&
            texIdx.z >= 0 && texIdx.z < int(gParams.textureDepth))
        {
            uint texIdx1D = computeTex1D(texIdx);

            if (gParams.isVelocity != 0u)
            {
                if (texIdx1D < gParams.range_coupleRateVelocities.y)
                {
                    float velCoupleRate = readArrayValue(texIdx1D + gParams.range_coupleRateVelocities.x);
                    coupleRate.x = velCoupleRate;
                    coupleRate.y = velCoupleRate;
                    coupleRate.z = velCoupleRate;
                }
                if (texIdx1D < gParams.range_coupleRateDivergences.y)
                {
                    coupleRate.w = readArrayValue(texIdx1D + gParams.range_coupleRateDivergences.x);
                }
                if (texIdx1D < gParams.range_velocities.y)
                {
                    targetValue.x = readArrayValue(3 * texIdx1D + 0 + gParams.range_velocities.x);
                    targetValue.y = readArrayValue(3 * texIdx1D + 1 + gParams.range_velocities.x);
                    targetValue.z = readArrayValue(3 * texIdx1D + 2 + gParams.range_velocities.x);
                }
                if (texIdx1D < gParams.range_divergences.y)
                {
                    targetValue.w = readArrayValue(texIdx1D + gParams.range_divergences.x);
                }
            }
            else
            {
                if (texIdx1D < gParams.range_coupleRateTemperatures.y)
                {
                    coupleRate.x = readArrayValue(texIdx1D + gParams.range_coupleRateTemperatures.x);
                }
                if (texIdx1D < gParams.range_coupleRateFuels.y)
                {
                    coupleRate.y = readArrayValue(texIdx1D + gParams.range_coupleRateFuels.x);
                }
                if (texIdx1D < gParams.range_coupleRateBurns.y)
                {
                    coupleRate.z = readArrayValue(texIdx1D + gParams.range_coupleRateBurns.x);
                }
                if (texIdx1D < gParams.range_coupleRateSmokes.y)
                {
                    coupleRate.w = readArrayValue(texIdx1D + gParams.range_coupleRateSmokes.x);
                }
                if (texIdx1D < gParams.range_temperatures.y)
                {
                    targetValue.x = readArrayValue(texIdx1D + gParams.range_temperatures.x);
                }
                if (texIdx1D < gParams.range_fuels.y)
                {
                    targetValue.y = readArrayValue(texIdx1D + gParams.range_fuels.x);
                }
                if (texIdx1D < gParams.range_burns.y)
                {
                    targetValue.z = readArrayValue(texIdx1D + gParams.range_burns.x);
                }
                if (texIdx1D < gParams.range_smokes.y)
                {
                    targetValue.w = readArrayValue(texIdx1D + gParams.range_smokes.x);
                }
            }
        }
        if (gParams.isVelocity != 0u)
        {
            if (gParams.velocityIsWorldSpace == 0u)
            {
                targetValue.xyz = mul(float4(targetValue.xyz, 0.f), gParams.localToWorldVelocity).xyz;
            }
        }
        else
        {
            if (gParams.needsSrgbConversion != 0u)
            {
                targetValue.xyz = srgb_to_linear(targetValue.xyz);
            }
        }
        targetValue *= gParams.targetValueScale;

        if (gParams.range_distances.y > 0u &&
            gParams.textureWidth > 0u &&
            gParams.textureHeight > 0u &&
            gParams.textureDepth > 0u)
        {
            float dist = sampleSdf(uvw);
            if (dist < gParams.minDistance || dist > gParams.maxDistance)
            {
                coupleRate = float4(0.f, 0.f, 0.f, 0.f);
            }
        }

        float4 value = NvFlowLocalRead4f(valueIn, gTable, gParams.table, blockIdx, threadIdx);

        float4 valueRate = gParams.deltaTime * coupleRate * opacity;
        valueRate = max(float4(0.f, 0.f, 0.f, 0.f), valueRate);
        valueRate = min(float4(1.f, 1.f, 1.f, 1.f), valueRate);

        value += valueRate * (targetValue - value);

        NvFlowLocalWrite4f(valueOut, gTable, gParams.table, blockIdx, threadIdx, value);
    }
}
