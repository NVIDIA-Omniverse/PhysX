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

StructuredBuffer<uint> countersIn;

RWStructuredBuffer<uint> countersOut;

groupshared uint sdata0[BLOCK_DIM];
groupshared uint sdata1[BLOCK_DIM];

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

[numthreads(BLOCK_DIM, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint threadIdx = dispatchThreadID.x;
    uint blockIdx = groupID.y;

    if (blockIdx == 0u)
    {
        uint globalOffset = 0u;

        uint numPasses = (paramsIn.numCounters / 4 + BLOCK_DIM - 1) / (BLOCK_DIM);
        uint idx = threadIdx;
        for (uint passID = 0; passID < numPasses; passID++)
        {
            uint blockOffset = 0u;

            uint4 countLocal = (idx < paramsIn.numCounters / 4) ? read4(countersIn, idx) : uint4(0, 0, 0, 0);

            uint4 countGlobal = blockScan(threadIdx, countLocal, blockOffset);

            // make inclusive
            countGlobal.x -= countLocal.x;
            countGlobal.y -= countLocal.y;
            countGlobal.z -= countLocal.z;
            countGlobal.w -= countLocal.w;

            countGlobal.x += globalOffset;
            countGlobal.y += globalOffset;
            countGlobal.z += globalOffset;
            countGlobal.w += globalOffset;

            if (idx < paramsIn.numCounters / 4)
            {
                write4(countersOut, idx, countGlobal);
            }

            globalOffset += blockOffset;

            idx += BLOCK_DIM;
        }
    }
    else
    {
        uint numPasses = (paramsIn.numCounters / 4 + BLOCK_DIM - 1) / (BLOCK_DIM);
        uint idx = threadIdx;
        for (uint passID = 0; passID < numPasses; passID++)
        {
            uint4 countLocal = (idx < paramsIn.numCounters / 4) ? read4(countersIn, idx + paramsIn.numCounters / 4) : uint4(0, 0, 0, 0);

            sdata0[threadIdx] = countLocal.x + countLocal.y + countLocal.z + countLocal.w;

            GroupMemoryBarrierWithGroupSync();

            uint scanTotal = 0;
            if ((threadIdx & 3) >= 1) scanTotal += sdata0[4 * (threadIdx / 4) + 0];
            if ((threadIdx & 3) >= 2) scanTotal += sdata0[4 * (threadIdx / 4) + 1];
            if ((threadIdx & 3) >= 3) scanTotal += sdata0[4 * (threadIdx / 4) + 2];

            // make final scan exclusive
            countLocal.w = countLocal.z + countLocal.y + countLocal.x + scanTotal;
            countLocal.z = countLocal.y + countLocal.x + scanTotal;
            countLocal.y = countLocal.x + scanTotal;
            countLocal.x = scanTotal;

            if (idx < paramsIn.numCounters / 4)
            {
                write4(countersOut, idx + paramsIn.numCounters / 4, countLocal);
            }

            idx += BLOCK_DIM;
        }
    }
}
