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
#define BLOCK_DIM 256

#include "RadixSortCommon.hlsli"

ConstantBuffer<RadixSortCSParams> paramsIn;

StructuredBuffer<uint> keyIn;
StructuredBuffer<uint> valIn;
StructuredBuffer<uint> countersIn;

RWStructuredBuffer<uint> keyOut;
RWStructuredBuffer<uint> valOut;

groupshared uint skey[4 * BLOCK_DIM];
groupshared uint sval[4 * BLOCK_DIM];
groupshared uint sdata0[BLOCK_DIM];
groupshared uint sdata1[BLOCK_DIM];
groupshared uint globalCounters[16];
groupshared uint localCounters[16];

uint4 blockScan(uint threadIdx, uint4 val, out uint totalCount)
{
    uint localVal = val.x + val.y + val.z + val.w;
    sdata0[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 1) localVal += sdata0[threadIdx - 1];
    if (threadIdx >= 2) localVal += sdata0[threadIdx - 2];
    if (threadIdx >= 3) localVal += sdata0[threadIdx - 3];
    sdata1[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 4) localVal += sdata1[threadIdx - 4];
    if (threadIdx >= 8) localVal += sdata1[threadIdx - 8];
    if (threadIdx >= 12) localVal += sdata1[threadIdx - 12];
    sdata0[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 16) localVal += sdata0[threadIdx - 16];
    if (threadIdx >= 32) localVal += sdata0[threadIdx - 32];
    if (threadIdx >= 48) localVal += sdata0[threadIdx - 48];
    sdata1[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 64) localVal += sdata1[threadIdx - 64];
    if (threadIdx >= 128) localVal += sdata1[threadIdx - 128];
    if (threadIdx >= 192) localVal += sdata1[threadIdx - 192];

    uint4 retVal;
    retVal.w = localVal;
    retVal.z = retVal.w - val.w;
    retVal.y = retVal.z - val.z;
    retVal.x = retVal.y - val.y;

    // compute totalCount
    totalCount = sdata1[63] + sdata1[127] + sdata1[191] + sdata1[255];

    return retVal;
}

// where pred==1 indicates a zero allocation, pred==0 indicates a one allocation
uint4 split4(uint threadIdx, uint4 pred)
{
    uint totalCount;
    uint4 scanVal = blockScan(threadIdx, pred, totalCount);

    uint4 rank;
    rank.x = bool(pred.x) ? scanVal.x - 1 : 4 * threadIdx + 0 - scanVal.x + totalCount;
    rank.y = bool(pred.y) ? scanVal.y - 1 : 4 * threadIdx + 1 - scanVal.y + totalCount;
    rank.z = bool(pred.z) ? scanVal.z - 1 : 4 * threadIdx + 2 - scanVal.z + totalCount;
    rank.w = bool(pred.w) ? scanVal.w - 1 : 4 * threadIdx + 3 - scanVal.w + totalCount;

    return rank;
}

[numthreads(BLOCK_DIM, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    uint threadIdx = dispatchThreadID.x;
    uint blockIdx = dispatchThreadID.y;
    uint idx = BLOCK_DIM * blockIdx + threadIdx;

    // preload counter values
    if (threadIdx < 16)
    {
        globalCounters[threadIdx] = countersIn[threadIdx * paramsIn.gridDim + blockIdx];
        localCounters[threadIdx] = countersIn[threadIdx + 16 * (paramsIn.gridDim + blockIdx)];
    }

    uint4 keyLocal = read4(keyIn, idx);
    uint4 valLocal = read4(valIn, idx);

    for (uint passID = paramsIn.passStart; passID < paramsIn.passStart + paramsIn.passNumBits; passID++)
    {
        uint4 allocVal;
        allocVal.x = ((keyLocal.x >> passID) & 1) ^ 1u;
        allocVal.y = ((keyLocal.y >> passID) & 1) ^ 1u;
        allocVal.z = ((keyLocal.z >> passID) & 1) ^ 1u;
        allocVal.w = ((keyLocal.w >> passID) & 1) ^ 1u;

        uint4 allocIdx = split4(threadIdx, allocVal);

        skey[allocIdx.x] = keyLocal.x;
        skey[allocIdx.y] = keyLocal.y;
        skey[allocIdx.z] = keyLocal.z;
        skey[allocIdx.w] = keyLocal.w;
        sval[allocIdx.x] = valLocal.x;
        sval[allocIdx.y] = valLocal.y;
        sval[allocIdx.z] = valLocal.z;
        sval[allocIdx.w] = valLocal.w;

        GroupMemoryBarrierWithGroupSync();

        keyLocal.x = skey[4 * threadIdx + 0];
        keyLocal.y = skey[4 * threadIdx + 1];
        keyLocal.z = skey[4 * threadIdx + 2];
        keyLocal.w = skey[4 * threadIdx + 3];
        valLocal.x = sval[4 * threadIdx + 0];
        valLocal.y = sval[4 * threadIdx + 1];
        valLocal.z = sval[4 * threadIdx + 2];
        valLocal.w = sval[4 * threadIdx + 3];
    }

    GroupMemoryBarrierWithGroupSync();

    for (uint sharedIdx = threadIdx; sharedIdx < 4 * BLOCK_DIM; sharedIdx += BLOCK_DIM)
    {
        uint bucketIdx = (skey[sharedIdx] >> paramsIn.passStart) & paramsIn.passMask;
        uint dstIdx = sharedIdx - localCounters[bucketIdx] + globalCounters[bucketIdx];
        keyOut[dstIdx] = skey[sharedIdx];
        valOut[dstIdx] = sval[sharedIdx];
    }
}
