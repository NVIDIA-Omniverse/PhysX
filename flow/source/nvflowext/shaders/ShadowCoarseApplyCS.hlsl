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

#include "NvFlowRayMarch.hlsli"

#include "ShadowParams.h"

ConstantBuffer<NvFlowSelfShadowShaderParams> globalParamsIn;
StructuredBuffer<NvFlowSelfShadowLayerShaderParams> layerParamsIn;

StructuredBuffer<uint> gTable;

Texture3D<float4> gInput;
SamplerState gSampler;

Texture2D<float4> gColormap;
SamplerState gColormapSampler;

Texture3D<float> alphaIn;

RWTexture3D<float4> densityOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(globalParamsIn.densityTable, dispatchThreadID.x);
    uint blockIdx = groupID.y + globalParamsIn.blockIdxOffset;

    uint layerParamIdx = NvFlowGetLayerParamIdx(gTable, globalParamsIn.densityTable, blockIdx);

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(gTable, globalParamsIn.densityTable, blockIdx, threadIdx).xyz;

    float4 density4 = gInput[readIdx];

    if (layerParamsIn[layerParamIdx].enabled != 0u)
    {
        float3 coarseThreadIdxf = 0.5f * (float3(threadIdx)+float3(0.5f, 0.5f, 0.5f));

        float alpha = NvFlowLocalReadLinear1f(alphaIn, gSampler, gTable, globalParamsIn.coarseDensityTable, blockIdx, coarseThreadIdxf);

        float sum_a = 1.f - alpha;

        sum_a = (1.f - layerParamsIn[layerParamIdx].minIntensity) * sum_a + layerParamsIn[layerParamIdx].minIntensity;

        if (layerParamsIn[layerParamIdx].base.enableRawMode != 0u)
        {
            density4.xyz *= sum_a;
        }
        else
        {
            density4.z = sum_a;
        }
    }

    NvFlowLocalWrite4f(densityOut, gTable, globalParamsIn.densityTable, blockIdx, threadIdx, density4);
}
