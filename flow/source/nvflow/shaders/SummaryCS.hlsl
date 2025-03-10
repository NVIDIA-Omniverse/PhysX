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

#include "SummaryParams.h"

#define BLOCK_DIM 256

ConstantBuffer<SummaryCS_GlobalParams> gParams;
StructuredBuffer<SummaryCS_LayerParams> gLayerParams;

StructuredBuffer<uint> gTable;

Texture3D<float4> velocityIn;
Texture3D<float4> densityCoarseIn;

RWStructuredBuffer<uint> summaryOut;

groupshared float4 sdata0[BLOCK_DIM];
groupshared float4 sdata1[BLOCK_DIM];

float4 reduceMax4(uint threadIdx, float4 val)
{
    sdata0[threadIdx] = val;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx < 64u)
    {
        val = max(val, sdata0[threadIdx + 64u]);
        val = max(val, sdata0[threadIdx + 128u]);
        val = max(val, sdata0[threadIdx + 192u]);

        sdata1[threadIdx] = val;
    }

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx < 16u)
    {
        val = max(val, sdata1[threadIdx + 16u]);
        val = max(val, sdata1[threadIdx + 32u]);
        val = max(val, sdata1[threadIdx + 48u]);

        sdata0[threadIdx] = val;
    }

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx < 4u)
    {
        val = max(val, sdata0[threadIdx + 4u]);
        val = max(val, sdata0[threadIdx + 8u]);
        val = max(val, sdata0[threadIdx + 12u]);

        sdata1[threadIdx] = val;
    }

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx < 1u)
    {
        val = max(val, sdata1[threadIdx + 1u]);
        val = max(val, sdata1[threadIdx + 2u]);
        val = max(val, sdata1[threadIdx + 3u]);

        sdata0[threadIdx] = val;
    }

    GroupMemoryBarrierWithGroupSync();

    return sdata0[0];
}

[numthreads(BLOCK_DIM, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint blockIdx = groupID.y + gParams.blockIdxOffset;
    uint sthreadIdx = dispatchThreadID.x;

    // write header
    if (blockIdx == 0u)
    {
        if (sthreadIdx < 4u)
        {
            uint writeVal = 0u;
            if (sthreadIdx == 0u) { writeVal = gParams.layerCount;}
            if (sthreadIdx == 1u) { writeVal = (gParams.anyEnabled != 0u ? gParams.table.numLocations : 0u);}
            if (sthreadIdx == 2u) { writeVal = gParams.table.maxLocations;}
            if (sthreadIdx == 3u) { writeVal = gParams.anyNeighborAllocation; }
            summaryOut[sthreadIdx] = writeVal;
        }

        uint totalElements = 4u * gParams.layerCount;
        for (uint elementIdx = sthreadIdx; elementIdx < totalElements; elementIdx += BLOCK_DIM)
        {
            if (elementIdx < totalElements)
            {
                uint layerParamIdx = elementIdx >> 2u;
                uint subIdx = elementIdx & 3;
                uint writeVal = 0u;
                if (subIdx == 0u) { writeVal = asuint(gLayerParams[layerParamIdx].blockSizeWorld.x);}
                if (subIdx == 1u) { writeVal = asuint(gLayerParams[layerParamIdx].blockSizeWorld.y);}
                if (subIdx == 2u) { writeVal = asuint(gLayerParams[layerParamIdx].blockSizeWorld.z);}
                if (subIdx == 3u) { writeVal = gLayerParams[layerParamIdx].layerAndLevel;}
                summaryOut[4u + elementIdx] = writeVal;
            }
        }
    }

    // write summary
    if (gParams.anyEnabled != 0u && blockIdx < gParams.table.numLocations)
    {
        uint layerParamIdx = NvFlowGetLayerParamIdx(gTable, gParams.table, blockIdx);

        if (gLayerParams[layerParamIdx].enabled != 0u)
        {
            // do single address translation, and recycle
            int3 readIdx = NvFlowSingleVirtualToReal(gTable, gParams.table, blockIdx, int3(0, 0, 0)).xyz;

            float4 maxSpeedSmoke = float4(0.f, 0.f, 0.f, 0.f);

            uint blockDimBits3 = gParams.table.blockDimBits.x + gParams.table.blockDimBits.y + gParams.table.blockDimBits.z;
            uint blockDim3 = 1u << blockDimBits3;
            uint halfBlockDimX = (1u << gParams.table.blockDimBits.x) >> 1u;
            for (uint threadIdx1D = sthreadIdx; threadIdx1D < blockDim3; threadIdx1D += BLOCK_DIM)
            {
                int3 threadIdx = NvFlowComputeThreadIdx(gParams.table, threadIdx1D);

                float4 velocity4 = velocityIn[readIdx + threadIdx];
                float speed = length(velocity4.xyz);

                float4 density4 = densityCoarseIn[readIdx + threadIdx];
                float smoke = density4.w;

                if (threadIdx.x < halfBlockDimX)
                {
                    maxSpeedSmoke.x = max(maxSpeedSmoke.x, speed);
                    maxSpeedSmoke.y = max(maxSpeedSmoke.y, smoke);
                }
                else
                {
                    maxSpeedSmoke.z = max(maxSpeedSmoke.z, speed);
                    maxSpeedSmoke.w = max(maxSpeedSmoke.w, smoke);
                }
            }

            maxSpeedSmoke = reduceMax4(sthreadIdx, maxSpeedSmoke);

            if (sthreadIdx < 1u)
            {
                uint mask = 0u;

                float maxSpeedNeg = maxSpeedSmoke.x;
                float maxSmokeNeg = maxSpeedSmoke.y;
                float maxSpeedPos = maxSpeedSmoke.z;
                float maxSmokePos = maxSpeedSmoke.w;
                if (gLayerParams[layerParamIdx].forceClear == 0u)
                {
                    if (maxSmokeNeg > gLayerParams[layerParamIdx].smokeThreshold ||
                        (maxSmokeNeg > gLayerParams[layerParamIdx].speedThresholdMinSmoke &&
                            maxSpeedNeg > gLayerParams[layerParamIdx].speedThreshold))
                    {
                        mask |= 0x0001;
                    }
                    if (maxSmokePos > gLayerParams[layerParamIdx].smokeThreshold ||
                        (maxSmokePos > gLayerParams[layerParamIdx].speedThresholdMinSmoke &&
                            maxSpeedPos > gLayerParams[layerParamIdx].speedThreshold))
                    {
                        mask |= 0x0002;
                    }
                    if (gLayerParams[layerParamIdx].enableNeighborAllocation != 0u)
                    {
                        if ((mask & 3) != 0u)
                        {
                            // allocate yneg, ypos, zneg, zpos
                            mask |= 0x00F0;
                        }
                        if ((mask & 1) != 0u)
                        {
                            mask |= 0x0004;
                        }
                        if ((mask & 2) != 0u)
                        {
                            mask |= 0x0008;
                        }
                    }
                }

                summaryOut[gParams.locationOffset + 5u * blockIdx + 4u] = mask;
            }
        }
        else
        {
            if (sthreadIdx < 1u)
            {
                summaryOut[gParams.locationOffset + 5u * blockIdx + 4u] = 0u;
            }
        }
        if (sthreadIdx < 4u)
        {
            int4 location;
            NvFlowBlockIdxToLocation(gTable, gParams.table, blockIdx, location);

            uint writeVal = 0u;
            if (sthreadIdx == 0u) {writeVal = location.x;}
            if (sthreadIdx == 1u) {writeVal = location.y;}
            if (sthreadIdx == 2u) {writeVal = location.z;}
            if (sthreadIdx == 3u) {writeVal = location.w;}
            summaryOut[gParams.locationOffset + 5u * blockIdx + sthreadIdx] = writeVal;
        }
    }
}
