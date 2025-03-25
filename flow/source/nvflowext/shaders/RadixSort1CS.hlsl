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

RWStructuredBuffer<uint> countersOut;

groupshared uint4 scount0[BLOCK_DIM];
groupshared uint4 scount1[BLOCK_DIM / 4];

void count(inout uint4 counter, uint bucket)
{
    if (bucket == 0) counter.x += (1 << 0);
    if (bucket == 1) counter.x += (1 << 16);
    if (bucket == 2) counter.y += (1 << 0);
    if (bucket == 3) counter.y += (1 << 16);
    if (bucket == 4) counter.z += (1 << 0);
    if (bucket == 5) counter.z += (1 << 16);
    if (bucket == 6) counter.w += (1 << 0);
    if (bucket == 7) counter.w += (1 << 16);
    if (bucket == 8) counter.x += (1 << 8);
    if (bucket == 9) counter.x += (1 << 24);
    if (bucket == 10) counter.y += (1 << 8);
    if (bucket == 11) counter.y += (1 << 24);
    if (bucket == 12) counter.z += (1 << 8);
    if (bucket == 13) counter.z += (1 << 24);
    if (bucket == 14) counter.w += (1 << 8);
    if (bucket == 15) counter.w += (1 << 24);
}

uint4 expand8to16L(uint4 counter)
{
    uint4 counterL;
    counterL.x = counter.x & 0x00FF00FF;
    counterL.y = counter.y & 0x00FF00FF;
    counterL.z = counter.z & 0x00FF00FF;
    counterL.w = counter.w & 0x00FF00FF;
    return counterL;
}

uint4 expand8to16H(uint4 counter)
{
    uint4 counterH;
    counterH.x = (counter.x & 0xFF00FF00) >> 8;
    counterH.y = (counter.y & 0xFF00FF00) >> 8;
    counterH.z = (counter.z & 0xFF00FF00) >> 8;
    counterH.w = (counter.w & 0xFF00FF00) >> 8;
    return counterH;
}

[numthreads(BLOCK_DIM, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    uint threadIdx = dispatchThreadID.x;
    uint blockIdx = dispatchThreadID.y;

    uint globalOffset = blockIdx * BLOCK_DIM;

    uint4 localCount = uint4(0, 0, 0, 0);

    uint4 keyLocal = read4(keyIn, threadIdx + globalOffset);
    keyLocal.x = (keyLocal.x >> paramsIn.passStart) & paramsIn.passMask;
    keyLocal.y = (keyLocal.y >> paramsIn.passStart) & paramsIn.passMask;
    keyLocal.z = (keyLocal.z >> paramsIn.passStart) & paramsIn.passMask;
    keyLocal.w = (keyLocal.w >> paramsIn.passStart) & paramsIn.passMask;
    count(localCount, keyLocal.x);
    count(localCount, keyLocal.y);
    count(localCount, keyLocal.z);
    count(localCount, keyLocal.w);

    scount0[threadIdx] = localCount;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx < BLOCK_DIM / 4)
    {
        localCount = scount0[threadIdx];
        localCount = localCount + scount0[threadIdx + 1 * BLOCK_DIM / 4];
        localCount = localCount + scount0[threadIdx + 2 * BLOCK_DIM / 4];
        localCount = localCount + scount0[threadIdx + 3 * BLOCK_DIM / 4];
        scount1[threadIdx] = localCount;
    }
    GroupMemoryBarrierWithGroupSync();

    // expand to 16-bit from 8-bit
    if (threadIdx < BLOCK_DIM / 16)
    {
        uint4 localCountH;
        localCount = expand8to16L(scount1[threadIdx]);
        localCountH = expand8to16H(scount1[threadIdx]);
        localCount = localCount + expand8to16L(scount1[threadIdx + 1 * BLOCK_DIM / 16]);
        localCountH = localCountH + expand8to16H(scount1[threadIdx + 1 * BLOCK_DIM / 16]);
        localCount = localCount + expand8to16L(scount1[threadIdx + 2 * BLOCK_DIM / 16]);
        localCountH = localCountH + expand8to16H(scount1[threadIdx + 2 * BLOCK_DIM / 16]);
        localCount = localCount + expand8to16L(scount1[threadIdx + 3 * BLOCK_DIM / 16]);
        localCountH = localCountH + expand8to16H(scount1[threadIdx + 3 * BLOCK_DIM / 16]);
        scount0[threadIdx] = localCount;
        scount0[threadIdx + BLOCK_DIM / 16] = localCountH;
    }
    GroupMemoryBarrierWithGroupSync();

    // two sets of 16 uint4 left to be reduced
    uint setID = threadIdx / (BLOCK_DIM / 2);
    uint setLaneID = threadIdx & (BLOCK_DIM / 2 - 1);
    if (setLaneID < BLOCK_DIM / 64)
    {
        uint offset = setID * BLOCK_DIM / 16;
        localCount = scount0[setLaneID + offset];
        localCount = localCount + scount0[setLaneID + 1 * BLOCK_DIM / 64 + offset];
        localCount = localCount + scount0[setLaneID + 2 * BLOCK_DIM / 64 + offset];
        localCount = localCount + scount0[setLaneID + 3 * BLOCK_DIM / 64 + offset];
        scount1[setLaneID + setID * BLOCK_DIM / 64] = localCount;
    }
    GroupMemoryBarrierWithGroupSync();

    // two sets of 4 uint4 left to be reduced
    if (setLaneID == 0)
    {
        uint offset = setID * BLOCK_DIM / 64;
        localCount = scount1[0 + offset];
        localCount = localCount + scount1[1 + offset];
        localCount = localCount + scount1[2 + offset];
        localCount = localCount + scount1[3 + offset];

        uint bucketOffset = 8 * setID;

        // output counter values for global scan
        countersOut[(bucketOffset + 0) * paramsIn.gridDim + blockIdx] = (localCount.x & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 1) * paramsIn.gridDim + blockIdx] = (localCount.x & 0xFFFF0000) >> 16;
        countersOut[(bucketOffset + 2) * paramsIn.gridDim + blockIdx] = (localCount.y & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 3) * paramsIn.gridDim + blockIdx] = (localCount.y & 0xFFFF0000) >> 16;
        countersOut[(bucketOffset + 4) * paramsIn.gridDim + blockIdx] = (localCount.z & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 5) * paramsIn.gridDim + blockIdx] = (localCount.z & 0xFFFF0000) >> 16;
        countersOut[(bucketOffset + 6) * paramsIn.gridDim + blockIdx] = (localCount.w & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 7) * paramsIn.gridDim + blockIdx] = (localCount.w & 0xFFFF0000) >> 16;

        // output counter values for local scan
        countersOut[(bucketOffset + 0) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.x & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 1) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.x & 0xFFFF0000) >> 16;
        countersOut[(bucketOffset + 2) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.y & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 3) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.y & 0xFFFF0000) >> 16;
        countersOut[(bucketOffset + 4) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.z & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 5) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.z & 0xFFFF0000) >> 16;
        countersOut[(bucketOffset + 6) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.w & 0x0000FFFF) >> 0;
        countersOut[(bucketOffset + 7) + 16 * (paramsIn.gridDim + blockIdx)] = (localCount.w & 0xFFFF0000) >> 16;
    }
}
