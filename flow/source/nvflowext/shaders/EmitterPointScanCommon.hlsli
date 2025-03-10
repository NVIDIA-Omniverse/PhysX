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

groupshared uint sBlockIdx[128];
groupshared uint sThreadIdx1D[128];

groupshared float4 sTargetValueSum0[128];
groupshared float4 sCoupleRateSum0[128];
groupshared float sWeightSum0[128];

groupshared float4 sTargetValueSum1[128];
groupshared float4 sCoupleRateSum1[128];
groupshared float sWeightSum1[128];

groupshared float4 sTargetValueSumTotal;
groupshared float4 sCoupleRateSumTotal;
groupshared float sWeightSumTotal;
groupshared uint sBlockIdxTotal;
groupshared uint sThreadIdx1DTotal;

void emitterPointScanAccum0(
    uint sthreadIdx,
    inout float4 targetValueSum,
    inout float4 coupleRateSum,
    inout float weightSum,
    uint offset
)
{
    if (sthreadIdx + offset < 128)
    {
        if (sBlockIdx[sthreadIdx] == sBlockIdx[sthreadIdx + offset] &&
            sThreadIdx1D[sthreadIdx] == sThreadIdx1D[sthreadIdx + offset])
        {
            targetValueSum += sTargetValueSum0[sthreadIdx + offset];
            coupleRateSum += sCoupleRateSum0[sthreadIdx + offset];
            weightSum += sWeightSum0[sthreadIdx + offset];
        }
    }
}

void emitterPointScanAccum1(
    uint sthreadIdx,
    inout float4 targetValueSum,
    inout float4 coupleRateSum,
    inout float weightSum,
    uint offset
)
{
    if (sthreadIdx + offset < 128)
    {
        if (sBlockIdx[sthreadIdx] == sBlockIdx[sthreadIdx + offset] &&
            sThreadIdx1D[sthreadIdx] == sThreadIdx1D[sthreadIdx + offset])
        {
            targetValueSum += sTargetValueSum1[sthreadIdx + offset];
            coupleRateSum += sCoupleRateSum1[sthreadIdx + offset];
            weightSum += sWeightSum1[sthreadIdx + offset];
        }
    }
}

void emitterPointScan(
    uint sthreadIdx,
    inout float4 targetValueSum,
    inout float4 coupleRateSum,
    inout float weightSum,
    inout uint blockIdx,
    inout uint threadIdx1D
)
{
    sBlockIdx[sthreadIdx] = blockIdx;
    sThreadIdx1D[sthreadIdx] = threadIdx1D;

    sTargetValueSum0[sthreadIdx] = targetValueSum;
    sCoupleRateSum0[sthreadIdx] = coupleRateSum;
    sWeightSum0[sthreadIdx] = weightSum;

    GroupMemoryBarrierWithGroupSync();    // read 0, write 1

    emitterPointScanAccum0(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 1);
    emitterPointScanAccum0(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 2);
    emitterPointScanAccum0(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 3);

    sTargetValueSum1[sthreadIdx] = targetValueSum;
    sCoupleRateSum1[sthreadIdx] = coupleRateSum;
    sWeightSum1[sthreadIdx] = weightSum;

    GroupMemoryBarrierWithGroupSync(); // read 1, write 0

    emitterPointScanAccum1(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 4);
    emitterPointScanAccum1(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 8);
    emitterPointScanAccum1(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 12);

    sTargetValueSum0[sthreadIdx] = targetValueSum;
    sCoupleRateSum0[sthreadIdx] = coupleRateSum;
    sWeightSum0[sthreadIdx] = weightSum;

    GroupMemoryBarrierWithGroupSync(); // read 0, write 1

    emitterPointScanAccum0(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 16);
    emitterPointScanAccum0(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 32);
    emitterPointScanAccum0(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 48);

    sTargetValueSum1[sthreadIdx] = targetValueSum;
    sCoupleRateSum1[sthreadIdx] = coupleRateSum;
    sWeightSum1[sthreadIdx] = weightSum;

    GroupMemoryBarrierWithGroupSync(); // read 1, write 0

    emitterPointScanAccum1(sthreadIdx, targetValueSum, coupleRateSum, weightSum, 64);

    if (sthreadIdx == 0u)
    {
        sTargetValueSumTotal = targetValueSum;
        sCoupleRateSumTotal = coupleRateSum;
        sWeightSumTotal = weightSum;
        sBlockIdxTotal = blockIdx;
        sThreadIdx1DTotal = threadIdx1D;
    }

    GroupMemoryBarrierWithGroupSync();
}
