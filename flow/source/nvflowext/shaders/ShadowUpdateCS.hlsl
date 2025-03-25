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

RWTexture3D<float4> densityOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(globalParamsIn.densityTable, dispatchThreadID.x);
    uint blockIdx = groupID.y + globalParamsIn.blockIdxOffset;

    uint layerParamIdx = NvFlowGetLayerParamIdx(gTable, globalParamsIn.densityTable, blockIdx);

    int4 vidx;
    NvFlowRealToVirtual(gTable, globalParamsIn.densityTable, blockIdx, threadIdx, vidx);

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(gTable, globalParamsIn.densityTable, blockIdx, threadIdx).xyz;

    float4 density4 = gInput[readIdx];

    if (layerParamsIn[layerParamIdx].enabled != 0u)
    {
        float maxWorldDistance = float(layerParamsIn[layerParamIdx].numSteps) * layerParamsIn[layerParamIdx].base.stepSize;

        float3 vidxf = float3(vidx.xyz) + float3(0.5f, 0.5f, 0.5f);
        float3 worldOrigin = vidxf * layerParamsIn[layerParamIdx].base.cellSize;

        float3 rayDir;
        if (bool(layerParamsIn[layerParamIdx].isPointLight))
        {
            rayDir = layerParamsIn[layerParamIdx].lightPosition - worldOrigin;
        }
        else
        {
            rayDir = layerParamsIn[layerParamIdx].lightDirection;
        }
        rayDir = normalize(rayDir);
        float3 rayDirInv = float3(1.f, 1.f, 1.f) / rayDir;

        // bias worldOrigin
        worldOrigin += layerParamsIn[layerParamIdx].stepOffset * rayDir;

        float4 sum = float4(0.f, 0.f, 0.f, 1.f);
        float nominalT = 0.f;

        NvFlowRayMarchBlocks(
            gInput,
            gSampler,
            gColormap,
            gColormapSampler,
            gTable,
            globalParamsIn.densityTable,
            layerParamsIn[layerParamIdx].base,
            worldOrigin,
            0.f,
            rayDir,
            maxWorldDistance,
            rayDirInv,
            sum,
            nominalT
        );

        sum.a = (1.f - layerParamsIn[layerParamIdx].minIntensity) * sum.a + layerParamsIn[layerParamIdx].minIntensity;

        if (layerParamsIn[layerParamIdx].base.enableRawMode != 0u)
        {
            density4.xyz *= sum.a;
        }
        else
        {
            density4.z = sum.a;
        }
    }

    NvFlowLocalWrite4f(densityOut, gTable, globalParamsIn.densityTable, blockIdx, threadIdx, density4);
}
