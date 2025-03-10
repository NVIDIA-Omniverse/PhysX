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

#include "VoxelInterpolateParams.h"

ConstantBuffer<VoxelInterpolateCS_GlobalParams> globalParamsIn;
StructuredBuffer<VoxelInterpolateCS_LayerParams> layerParamsIn;

StructuredBuffer<uint> tableIn;

Texture3D<float4> densityIn;
Texture3D<float> voxelWeightIn;

RWTexture3D<float4> densityOut;
RWTexture3D<float> voxelWeightOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    if (dispatchThreadID.x >= globalParamsIn.table.threadsPerBlock)
    {
        return;
    }

    int3 threadIdx = NvFlowComputeThreadIdx(globalParamsIn.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + globalParamsIn.blockIdxOffset;

    uint layerParamIdx = NvFlowGetLayerParamIdx(tableIn, globalParamsIn.table, blockIdx);

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(tableIn, globalParamsIn.table, blockIdx, threadIdx).xyz;

    float4 density4 = densityIn[readIdx];
    float voxelWeight = voxelWeightIn[readIdx];

    const float threshold = layerParamsIn[layerParamIdx].threshold;

    if (layerParamsIn[layerParamIdx].enabled != 0u && voxelWeight < threshold)
    {
        // check for case of two opposing nearest neighbors being active
        float4 accum = float4(0.f, 0.f, 0.f, 0.f);
        float accumWeight = 0.f;

        float4 densityNeg = densityIn[readIdx + int3(-1, 0, 0)];
        float weightNeg = voxelWeightIn[readIdx + int3(-1, 0, 0)];
        float4 densityPos = densityIn[readIdx + int3(+1, 0, 0)];
        float weightPos = voxelWeightIn[readIdx + int3(+1, 0, 0)];
        if (weightNeg >= threshold && weightPos >= threshold)
        {
            accum += densityNeg + densityPos;
            accumWeight += weightNeg + weightPos;
        }

        densityNeg = densityIn[readIdx + int3(0, -1, 0)];
        weightNeg = voxelWeightIn[readIdx + int3(0, -1, 0)];
        densityPos = densityIn[readIdx + int3(0, +1, 0)];
        weightPos = voxelWeightIn[readIdx + int3(0, +1, 0)];
        if (weightNeg >= threshold && weightPos >= threshold)
        {
            accum += densityNeg + densityPos;
            accumWeight += weightNeg + weightPos;
        }

        densityNeg = densityIn[readIdx + int3(0, 0, -1)];
        weightNeg = voxelWeightIn[readIdx + int3(0, 0, -1)];
        densityPos = densityIn[readIdx + int3(0, 0, +1)];
        weightPos = voxelWeightIn[readIdx + int3(0, 0, +1)];
        if (weightNeg >= threshold && weightPos >= threshold)
        {
            accum += densityNeg + densityPos;
            accumWeight += weightNeg + weightPos;
        }

        if (accumWeight > 0.f)
        {
            density4 = accum;
            density4 *= (1.f / accumWeight);
            voxelWeight = 1.f;
        }
    }

    // clear everything not meeting threshold
    if (voxelWeight < threshold)
    {
        density4 = float4(0.f, 0.f, 0.f, 0.f);
        voxelWeight = 0.f;
    }

    NvFlowLocalWrite4f(densityOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, density4);
    NvFlowLocalWrite1f(voxelWeightOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, voxelWeight);
}
