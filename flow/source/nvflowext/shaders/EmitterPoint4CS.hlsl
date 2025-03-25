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

ConstantBuffer<EmitterPointCS_Params> gParams;
ConstantBuffer<EmitterPointCS_SubStepParams> gSubStepParams;

StructuredBuffer<uint> gTable;
StructuredBuffer<uint> arrayValuesIn;

StructuredBuffer<uint> keyIn;
StructuredBuffer<uint> valIn;
StructuredBuffer<uint> keyLowIn;

Texture3D<float4> valueIn;
SamplerState valueSampler;

RWTexture3D<float4> coarseValueOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    uint sortedIdx = dispatchThreadID.x + (gParams.pointBlockIdxOffset << 7u);

    if (sortedIdx >= gParams.voxelsPerPoint * gParams.range_positions.y)
    {
        return;
    }

    uint blockIdx = keyIn[sortedIdx];
    uint fullIdx = valIn[sortedIdx];
    uint threadIdx1D = keyLowIn[fullIdx] & ((1 << 22u) - 1u);
    // recover threadIdx from key
    int3 threadIdx = int3(
        int((threadIdx1D) & (gParams.table.blockDimLessOne.x)),
        int((threadIdx1D >> gParams.table.blockDimBits.x) & (gParams.table.blockDimLessOne.y)),
        int((threadIdx1D >> (gParams.table.blockDimBits.x + gParams.table.blockDimBits.y)) & (gParams.table.blockDimLessOne.z))
        );

    if (blockIdx != ~0u)
    {
        int3 threadIdxCoarse = threadIdx >> (gParams.table.blockDimBits - gParams.tableCoarse.blockDimBits);

        // downsample
        float3 fetchIdxf = gParams.velocityToDensityBlockScale * (float3(threadIdxCoarse)+float3(0.5f, 0.5f, 0.5f));
        float4 value = NvFlowLocalReadLinear4f(valueIn, valueSampler, gTable, gParams.table, blockIdx, fetchIdxf);

        NvFlowLocalWrite4f(coarseValueOut, gTable, gParams.tableCoarse, blockIdx, threadIdxCoarse, value);
    }
}
