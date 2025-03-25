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
SamplerState samplerIn;

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

    float3 threadIdxf = float3(threadIdx)+0.5f;

    float4 density4 = NvFlowLocalRead4f(densityIn, tableIn, globalParamsIn.table, blockIdx, threadIdx);
    float voxelWeight = NvFlowLocalRead1f(voxelWeightIn, tableIn, globalParamsIn.table, blockIdx, threadIdx);

    if (voxelWeight == 0.f)
    {
        for (int k = 0; k < 2; k++)
        {
            for (int j = 0; j < 2; j++)
            {
                for (int i = 0; i < 2; i++)
                {
                    float3 offset = float3(float(i) - 0.5f, float(j) - 0.5f, float(k) - 0.5f);
                    density4 += NvFlowLocalReadLinear4f(densityIn, samplerIn, tableIn, globalParamsIn.sampleTable, blockIdx, threadIdxf + offset);
                    voxelWeight += NvFlowLocalReadLinear1f(voxelWeightIn, samplerIn, tableIn, globalParamsIn.sampleTable, blockIdx, threadIdxf + offset);
                }
            }
        }
    }

    if (voxelWeight > 0.f)
    {
        density4 *= (1.f / voxelWeight);
        voxelWeight = 1.f;
    }

    NvFlowLocalWrite4f(densityOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, density4);
    NvFlowLocalWrite1f(voxelWeightOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, voxelWeight);
}
