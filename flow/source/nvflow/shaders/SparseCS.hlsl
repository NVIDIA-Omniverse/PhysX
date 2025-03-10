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

#include "SparseParams.h"

ConstantBuffer<SparseShaderParams> gParams;

StructuredBuffer<uint> gTableIn;

RWStructuredBuffer<uint> gTableOut;

uint3 unpackAllocation(uint v)
{
    return int3(
        (v) & 0x3FF,
        (v >> 10u) & 0x3FF,
        (v >> 20u) & 0x3FF
    );
}

uint blockIdxToTableValue(uint blockIdx, uint3 blockDimBits, int3 neighborOffset)
{
    uint3 blockIdx3 = unpackAllocation(gTableIn[blockIdx + gParams.tableParams.allocationOffset]);

    // y = (x << 1) + 1
    uint3 texelOffset = ((blockIdx3 << blockDimBits) + (blockIdx3 << 1u) - uint3(neighborOffset << blockDimBits)) >> 1u;

    return (1u << 31u) | ((texelOffset.z & 0x3FF) << 21u) | ((texelOffset.y & 0x3FF) << 11u) | (texelOffset.x & 0x7FF);
}

uint blockIdxToTableValuePad(uint blockIdx, uint3 blockDimBits, int3 neighborOffset)
{
    uint3 blockIdx3 = unpackAllocation(gTableIn[blockIdx + gParams.tableParams.allocationOffset]);

    // y = (x << 1) + 1 + neighborOffset;
    uint3 texelOffset = ((blockIdx3 << blockDimBits) + (blockIdx3 << 1u)) >> 1u;

    return ((texelOffset.z & 0x3FF) << 21u) | ((texelOffset.y & 0x3FF) << 11u) | (texelOffset.x & 0x7FF);
}

[numthreads(32, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint laneIdx = dispatchThreadID.x & 31;
    uint blockLevel = dispatchThreadID.x >> 5u;
    uint blockIdx = groupID.y + gParams.sparseBlockIdxOffset;

    if (blockIdx < gParams.tableParams.numLocations)
    {
        uint locationOffset = gParams.tableParams.locationOffset;
        int4 location;
        location.x = int(gTableIn[4u * blockIdx + 0u + locationOffset]);
        location.y = int(gTableIn[4u * blockIdx + 1u + locationOffset]);
        location.z = int(gTableIn[4u * blockIdx + 2u + locationOffset]);
        location.w = int(gTableIn[4u * blockIdx + 3u + locationOffset]);

        if (laneIdx == 0)
        {
            // table value
            uint tableValue = blockIdxToTableValue(blockIdx, gParams.blockLevel[blockLevel].blockDimBits, int3(0, 0, 0));

            uint blockLevelOffsetGlobal = gParams.blockLevel[blockLevel].blockLevelOffsetGlobal;
            gTableOut[blockIdx + blockLevelOffsetGlobal] = blockIdx < gParams.tableParams.numLocations ? tableValue : 0u;
        }

        // local
        {
            uint baseOffset = (blockIdx << 5u) + gParams.blockLevel[blockLevel].blockLevelOffsetLocal;

            uint tableValue = 0u;

            if (laneIdx == 0u)
            {
                tableValue = uint(location.x);
            }
            if (laneIdx == 1u)
            {
                tableValue = uint(location.y);
            }
            if (laneIdx == 2u)
            {
                tableValue = uint(location.z);
            }
            if (laneIdx == 3u)
            {
                tableValue = uint(location.w);
            }
            if (laneIdx == 4u)
            {
                tableValue = uint(0u);
            }
            if (laneIdx >= 5u)
            {
                int i = int((laneIdx - 5u) % 3u) - 1;
                int j = int(((laneIdx - 5u) / 3u) % 3u) - 1;
                int k = int((laneIdx - 5u) / 9u) - 1;

                uint blockIdxLocal = NvFlowLocationToBlockIdx(gTableIn, gParams.tableParams, location + int4(i, j, k, 0));
                if (blockIdxLocal != ~0u)
                {
                    tableValue = blockIdxToTableValue(blockIdxLocal, gParams.blockLevel[blockLevel].blockDimBits, int3(i, j, k));
                }
                else
                {
                    tableValue = blockIdxToTableValuePad(blockIdx, gParams.blockLevel[blockLevel].blockDimBits, int3(i, j, k));
                }
            }

            gTableOut[baseOffset + laneIdx] = tableValue;
        }
    }
}
