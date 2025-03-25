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

StructuredBuffer<float4> reduceTarget2In;
StructuredBuffer<float4> reduceCouple2In;
StructuredBuffer<float> reduceWeight2In;
StructuredBuffer<uint2> key2In;

StructuredBuffer<float4> scanTarget3In;
StructuredBuffer<float4> scanCouple3In;
StructuredBuffer<float> scanWeight3In;
StructuredBuffer<uint2> key3In;

RWStructuredBuffer<float4> scanTarget2Out;
RWStructuredBuffer<float4> scanCouple2Out;
RWStructuredBuffer<float> scanWeight2Out;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint reduceIdx2 = (dispatchThreadID.x + (gParams.reduce3BlockIdxOffset << 7u));
    uint sthreadIdx = reduceIdx2 & 127;

    float4 targetValueSum = float4(0.f, 0.f, 0.f, 0.f);
    float4 coupleRateSum = float4(0.f, 0.f, 0.f, 0.f);
    float weightSum = 0.f;
    uint blockIdx = ~0u;
    uint threadIdx1D = ~0u;
    if (reduceIdx2 < gParams.reduceCount2)
    {
        uint2 key2 = key2In[reduceIdx2];
        blockIdx = key2.x;
        threadIdx1D = key2.y;
        targetValueSum = reduceTarget2In[reduceIdx2];
        coupleRateSum = reduceCouple2In[reduceIdx2];
        weightSum = reduceWeight2In[reduceIdx2];
    }

    emitterPointScan(sthreadIdx, targetValueSum, coupleRateSum, weightSum, blockIdx, threadIdx1D);

    // append higher level to local reduction
    uint reduceIdx3 = (reduceIdx2 >> 7u) + 1u;
    if (reduceIdx3 < gParams.reduceCount3)
    {
        uint2 key3 = key3In[reduceIdx3];
        uint blockIdx3 = key3.x;
        uint threadIdx1D3 = key3.y;
        if (blockIdx3 == blockIdx && threadIdx1D3 == threadIdx1D)
        {
            targetValueSum += scanTarget3In[reduceIdx3];
            coupleRateSum += scanCouple3In[reduceIdx3];
            weightSum += scanWeight3In[reduceIdx3];
        }
    }

    if (reduceIdx2 < gParams.reduceCount2)
    {
        scanTarget2Out[reduceIdx2] = targetValueSum;
        scanCouple2Out[reduceIdx2] = coupleRateSum;
        scanWeight2Out[reduceIdx2] = weightSum;
    }
}
