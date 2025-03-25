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

#include "EmitterPointReduceCommon.hlsli"

ConstantBuffer<EmitterPointCS_Params> gParams;
ConstantBuffer<EmitterPointCS_SubStepParams> gSubStepParams;

StructuredBuffer<uint> gTable;
StructuredBuffer<uint> arrayValuesIn;

StructuredBuffer<uint> keyLowIn;
StructuredBuffer<uint> keyIn;
StructuredBuffer<uint> valIn;

RWStructuredBuffer<float4> reduceTarget1Out;
RWStructuredBuffer<float4> reduceCouple1Out;
RWStructuredBuffer<float> reduceWeight1Out;
RWStructuredBuffer<uint2> key1Out;

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

    // reduce by 128, the key for thread 0 mask the reduction
    uint sortedIdxCmp = sortedIdx - sthreadIdx;
    uint blockIdxCmp = keyIn[sortedIdxCmp];
    uint fullIdxCmp = valIn[sortedIdxCmp];
    uint threadIdx1DCmp = keyLowIn[fullIdxCmp] & ((1 << 22u) - 1u);

    float4 targetValueSum = float4(0.f, 0.f, 0.f, 0.f);
    float4 coupleRateSum = float4(0.f, 0.f, 0.f, 0.f);
    float weightSum = 0.f;
    if (sortedIdx < gParams.voxelsPerPoint * gParams.range_positions.y)
    {
        uint blockIdx = keyIn[sortedIdx];
        uint fullIdx = valIn[sortedIdx];
        uint keyLow = keyLowIn[fullIdx];
        uint threadIdx1D = keyLow & ((1 << 22u) - 1u);
        if (blockIdxCmp == blockIdx && threadIdx1DCmp == threadIdx1D)
        {
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

            targetValueSum = weight * targetValue;
            coupleRateSum = weight * (weight * coupleRate);
            weightSum = weight;
        }
    }

    emitterPointReduce(sthreadIdx, targetValueSum, coupleRateSum, weightSum);

    if (sthreadIdx < 1u)
    {
        uint reduceIdx1 = sortedIdx >> 7u;

        reduceTarget1Out[reduceIdx1] = targetValueSum;
        reduceCouple1Out[reduceIdx1] = coupleRateSum;
        reduceWeight1Out[reduceIdx1] = weightSum;
        key1Out[reduceIdx1] = uint2(blockIdxCmp, threadIdx1DCmp);
    }
}
