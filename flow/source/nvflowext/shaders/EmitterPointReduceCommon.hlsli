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


groupshared float4 sTargetValueSum[128];
groupshared float4 sCoupleRateSum[128];
groupshared float sWeightSum[128];

void emitterPointReduce(uint sthreadIdx, inout float4 targetValueSum, inout float4 coupleRateSum, inout float weightSum)
{
    sTargetValueSum[sthreadIdx] = targetValueSum;
    sCoupleRateSum[sthreadIdx] = coupleRateSum;
    sWeightSum[sthreadIdx] = weightSum;

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 32u)
    {
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 32u];
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 64u];
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 96u];

        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 32u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 64u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 96u];

        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 32u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 64u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 96u];
    }

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 8u)
    {
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 8u];
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 16u];
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 24u];

        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 8u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 16u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 24u];

        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 8u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 16u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 24u];
    }

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 2u)
    {
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 2u];
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 4u];
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 6u];

        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 2u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 4u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 6u];

        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 2u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 4u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 6u];
    }

    GroupMemoryBarrierWithGroupSync();

    if (sthreadIdx < 1u)
    {
        sTargetValueSum[sthreadIdx] += sTargetValueSum[sthreadIdx + 1u];
        sCoupleRateSum[sthreadIdx] += sCoupleRateSum[sthreadIdx + 1u];
        sWeightSum[sthreadIdx] += sWeightSum[sthreadIdx + 1u];
    }

    GroupMemoryBarrierWithGroupSync();

    targetValueSum = sTargetValueSum[0];
    coupleRateSum = sCoupleRateSum[0];
    weightSum = sWeightSum[0];
}
