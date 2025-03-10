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

#include "EmitterPointScanCommon.hlsli"

ConstantBuffer<EmitterPointCS_Params> gParams;
ConstantBuffer<EmitterPointCS_SubStepParams> gSubStepParams;

StructuredBuffer<uint> gTable;
StructuredBuffer<uint> arrayValuesIn;

StructuredBuffer<float4> oldValueIn;
StructuredBuffer<uint> keyLowIn;
StructuredBuffer<uint> keyIn;
StructuredBuffer<uint> valIn;

StructuredBuffer<float4> scanTarget1In;
StructuredBuffer<float4> scanCouple1In;
StructuredBuffer<float> scanWeight1In;
StructuredBuffer<uint2> key1In;

RWTexture3D<float4> valueOut;
RWTexture3D<float> voxelWeightOut;

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
    uint sortedIdx = dispatchThreadID.x + (gParams.pointBlockIdxOffset << 7u);
    uint sthreadIdx = sortedIdx & 127;

    uint blockIdx = ~0u;
    uint threadIdx1D = ~0u;
    uint fullIdx = ~0u;
    float4 targetValueSum = float4(0.f, 0.f, 0.f, 0.f);
    float4 coupleRateSum = float4(0.f, 0.f, 0.f, 0.f);
    float weightSum = 0.f;
    if (sortedIdx < gParams.voxelsPerPoint * gParams.range_positions.y)
    {
        blockIdx = keyIn[sortedIdx];
        fullIdx = valIn[sortedIdx];
        uint keyLow = keyLowIn[fullIdx];
        threadIdx1D = keyLow & ((1 << 22u) - 1u);

        uint pointIdx = fullIdx / gParams.voxelsPerPoint;
        float weight = float((keyLow >> 22u) & 1023) * (1.f / 1023.f);

        float4 targetValue = gParams.defaultTargetValue;
        float4 coupleRate = gParams.defaultCoupleRate;
        if (gParams.isVelocity != 0u)
        {
            if (pointIdx < gParams.range_coupleRateVelocities.y)
            {
                float velCoupleRate = asfloat(arrayValuesIn[gParams.range_coupleRateVelocities.x + pointIdx]);
                coupleRate.x = velCoupleRate;
                coupleRate.y = velCoupleRate;
                coupleRate.z = velCoupleRate;
            }
            if (pointIdx < gParams.range_coupleRateDivergences.y)
            {
                coupleRate.w = asfloat(arrayValuesIn[gParams.range_coupleRateDivergences.x + pointIdx]);
            }
            if (pointIdx < gParams.range_velocities.y)
            {
                targetValue.x = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * pointIdx + 0]);
                targetValue.y = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * pointIdx + 1]);
                targetValue.z = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * pointIdx + 2]);
            }
            if (pointIdx < gParams.range_divergences.y)
            {
                targetValue.w = asfloat(arrayValuesIn[gParams.range_divergences.x + pointIdx]);
            }
            if (gParams.velocityIsWorldSpace == 0u)
            {
                targetValue.xyz = mul(float4(targetValue.xyz, 0.f), gParams.localToWorldVelocity).xyz;
            }
        }
        else
        {
            if (pointIdx < gParams.range_coupleRateTemperatures.y)
            {
                coupleRate.x = asfloat(arrayValuesIn[gParams.range_coupleRateTemperatures.x + pointIdx]);
            }
            if (pointIdx < gParams.range_coupleRateFuels.y)
            {
                coupleRate.y = asfloat(arrayValuesIn[gParams.range_coupleRateFuels.x + pointIdx]);
            }
            if (pointIdx < gParams.range_coupleRateBurns.y)
            {
                coupleRate.z = asfloat(arrayValuesIn[gParams.range_coupleRateBurns.x + pointIdx]);
            }
            if (pointIdx < gParams.range_coupleRateSmokes.y)
            {
                coupleRate.w = asfloat(arrayValuesIn[gParams.range_coupleRateSmokes.x + pointIdx]);
            }
            if (pointIdx < gParams.range_temperatures.y)
            {
                targetValue.x = asfloat(arrayValuesIn[gParams.range_temperatures.x + pointIdx]);
            }
            if (pointIdx < gParams.range_fuels.y)
            {
                targetValue.y = asfloat(arrayValuesIn[gParams.range_fuels.x + pointIdx]);
            }
            if (pointIdx < gParams.range_burns.y)
            {
                targetValue.z = asfloat(arrayValuesIn[gParams.range_burns.x + pointIdx]);
            }
            if (pointIdx < gParams.range_smokes.y)
            {
                targetValue.w = asfloat(arrayValuesIn[gParams.range_smokes.x + pointIdx]);
            }
            if (pointIdx < gParams.range_colors.y)
            {
                targetValue.x = asfloat(arrayValuesIn[gParams.range_colors.x + 3 * pointIdx + 0]);
                targetValue.y = asfloat(arrayValuesIn[gParams.range_colors.x + 3 * pointIdx + 1]);
                targetValue.z = asfloat(arrayValuesIn[gParams.range_colors.x + 3 * pointIdx + 2]);
            }
            if (gParams.needsSrgbConversion != 0u)
            {
                targetValue.xyz = srgb_to_linear(targetValue.xyz);
            }
        }
        targetValue *= gParams.targetValueScale;

        float coupleRateWeight = (gParams.enableInterpolation == 0u) ? weight : 1.f;

        targetValueSum = weight * targetValue;
        coupleRateSum = coupleRateWeight * (weight * coupleRate);
        weightSum = weight;
    }

    emitterPointScan(sthreadIdx, targetValueSum, coupleRateSum, weightSum, blockIdx, threadIdx1D);

    // append higher level to local reduction
    uint reduceIdx1 = (sortedIdx >> 7u) + 1u;
    if (reduceIdx1 < gParams.reduceCount1)
    {
        uint2 key1 = key1In[reduceIdx1];
        uint blockIdx1 = key1.x;
        uint threadIdx1D1 = key1.y;
        if (blockIdx1 == blockIdx && threadIdx1D1 == threadIdx1D)
        {
            targetValueSum += scanTarget1In[reduceIdx1];
            coupleRateSum += scanCouple1In[reduceIdx1];
            weightSum += scanWeight1In[reduceIdx1];
        }
    }

    // exit if target block is not allocated
    if (blockIdx == ~0u)
    {
        return;
    }
    if (fullIdx >= gParams.voxelsPerPoint * gParams.range_positions.y || sortedIdx >= gParams.voxelsPerPoint * gParams.range_positions.y)
    {
        return;
    }
    // check if previous sort id has match key, and exit if matched
    if (sortedIdx > 0u)
    {
        uint cmpBlockIdx = keyIn[sortedIdx - 1u];
        uint cmpFullIdx = valIn[sortedIdx - 1u];
        uint cmpThreadIdx1D = keyLowIn[cmpFullIdx] & ((1 << 22u) - 1u);
        if (blockIdx == cmpBlockIdx && threadIdx1D == cmpThreadIdx1D)
        {
            return;
        }
    }
    // recover threadIdx from key
    int3 threadIdx = int3(
        int((threadIdx1D) & (gParams.table.blockDimLessOne.x)),
        int((threadIdx1D >> gParams.table.blockDimBits.x) & (gParams.table.blockDimLessOne.y)),
        int((threadIdx1D >> (gParams.table.blockDimBits.x + gParams.table.blockDimBits.y)) & (gParams.table.blockDimLessOne.z))
        );
    if (weightSum > 0.f)
    {
        float weightInv = 1.f / weightSum;
        targetValueSum *= weightInv;
        coupleRateSum *= weightInv;

        float4 valueRate = gParams.deltaTime * coupleRateSum;
        valueRate = max(float4(0.f, 0.f, 0.f, 0.f), valueRate);
        valueRate = min(float4(1.f, 1.f, 1.f, 1.f), valueRate);

        // fetch old value
        float4 value = oldValueIn[fullIdx];

        // apply twice to emulate old behavior
        value += valueRate * (targetValueSum - value);
        value += valueRate * (targetValueSum - value);

        NvFlowLocalWrite4f(valueOut, gTable, gParams.table, blockIdx, threadIdx, value);

        // mark as valid
        float voxelWeight = 1.f;
        NvFlowLocalWrite1f(voxelWeightOut, gTable, gParams.table, blockIdx, threadIdx, voxelWeight);
    }
}
