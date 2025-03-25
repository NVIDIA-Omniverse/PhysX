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

StructuredBuffer<float4> reduceTarget3In;
StructuredBuffer<float4> reduceCouple3In;
StructuredBuffer<float> reduceWeight3In;
StructuredBuffer<uint2> key3In;

RWStructuredBuffer<float4> scanTarget3Out;
RWStructuredBuffer<float4> scanCouple3Out;
RWStructuredBuffer<float> scanWeight3Out;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint sthreadIdx = dispatchThreadID.x & 127;

    if (sthreadIdx == 0u)
    {
        sTargetValueSumTotal = float4(0.f, 0.f, 0.f, 0.f);
        sCoupleRateSumTotal = float4(0.f, 0.f, 0.f, 0.f);
        sWeightSumTotal = 0.f;
        sBlockIdxTotal = ~0u;
        sThreadIdx1DTotal = ~0u;
    }

    GroupMemoryBarrierWithGroupSync();

    // scan high to low
    uint passCount = (gParams.reduceCount3 + 127u) >> 7u;
    for (uint passIdx = 0u; passIdx < passCount; passIdx++)
    {
        uint reduceIdx3 = ((passCount - passIdx - 1u) << 7u) + sthreadIdx;

        float4 targetValueSum = float4(0.f, 0.f, 0.f, 0.f);
        float4 coupleRateSum = float4(0.f, 0.f, 0.f, 0.f);
        float weightSum = 0.f;
        uint blockIdx = ~0u;
        uint threadIdx1D = ~0u;
        if (reduceIdx3 < gParams.reduceCount3)
        {
            uint2 key3 = key3In[reduceIdx3];
            blockIdx = key3.x;
            threadIdx1D = key3.y;
            targetValueSum = reduceTarget3In[reduceIdx3];
            coupleRateSum = reduceCouple3In[reduceIdx3];
            weightSum = reduceWeight3In[reduceIdx3];
        }
        if (blockIdx == sBlockIdxTotal && threadIdx1D == sThreadIdx1DTotal)
        {
            targetValueSum += sTargetValueSumTotal;
            coupleRateSum += sCoupleRateSumTotal;
            weightSum += sWeightSumTotal;
        }

        emitterPointScan(sthreadIdx, targetValueSum, coupleRateSum, weightSum, blockIdx, threadIdx1D);

        if (reduceIdx3 < gParams.reduceCount3)
        {
            scanTarget3Out[reduceIdx3] = targetValueSum;
            scanCouple3Out[reduceIdx3] = coupleRateSum;
            scanWeight3Out[reduceIdx3] = weightSum;
        }
    }
}
