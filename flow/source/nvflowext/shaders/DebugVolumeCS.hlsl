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

#include "DebugVolumeParams.h"

ConstantBuffer<DebugVolumeCS_GlobalParams> globalParamsIn;
StructuredBuffer<DebugVolumeCS_LayerParams> layerParamsIn;

StructuredBuffer<uint> tableIn;

Texture3D<float4> densityShadowIn;

Texture3D<float4> velocityIn;
SamplerState samplerIn;

RWTexture3D<float4> densityShadowOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(globalParamsIn.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + globalParamsIn.blockIdxOffset;

    uint layerParamIdx = NvFlowGetLayerParamIdx(tableIn, globalParamsIn.table, blockIdx);

    float4 density4 = NvFlowLocalRead4f(densityShadowIn, tableIn, globalParamsIn.table, blockIdx, threadIdx);

    float3 coarseThreadIdxf = 0.5f * (float3(threadIdx) + 0.5f);

    float4 velocity4 = NvFlowLocalReadLinear4f(velocityIn, samplerIn, tableIn, globalParamsIn.velocityTable, blockIdx, coarseThreadIdxf);

    if (layerParamsIn[layerParamIdx].enabled != 0u)
    {
        if (layerParamsIn[layerParamIdx].enableSpeedAsTemperature != 0u)
        {
            density4.x = length(velocity4.xyz * layerParamsIn[layerParamIdx].velocityScale);
        }
        if (layerParamsIn[layerParamIdx].enableVelocityAsDensity != 0u)
        {
            density4.xyz = abs(velocity4.xyz * layerParamsIn[layerParamIdx].velocityScale);
        }
        if (layerParamsIn[layerParamIdx].enableDivergenceAsSmoke != 0u)
        {
            density4.w = velocity4.w;
        }

        density4.x = layerParamsIn[layerParamIdx].outputTemperatureScale * density4.x +
            layerParamsIn[layerParamIdx].outputTemperatureOffset;
        density4.y = layerParamsIn[layerParamIdx].outputFuelScale * density4.y +
            layerParamsIn[layerParamIdx].outputFuelOffset;
        density4.z = layerParamsIn[layerParamIdx].outputBurnScale * density4.z +
            layerParamsIn[layerParamIdx].outputBurnOffset;
        density4.w = layerParamsIn[layerParamIdx].outputSmokeScale * density4.w +
            layerParamsIn[layerParamIdx].outputSmokeOffset;

        if (layerParamsIn[layerParamIdx].outputScaleBySmoke != 0u)
        {
            density4.xyz *= density4.w;
        }

        if (layerParamsIn[layerParamIdx].slicePlaneThickness != 0.f)
        {
            int4 location = int4(0, 0, 0, 0);
            if (NvFlowBlockIdxToLocation(tableIn, globalParamsIn.table, blockIdx, location))
            {
                int3 vidx = (location.xyz << globalParamsIn.table.blockDimBits) + threadIdx;
                float3 vidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);

                float3 worldf = vidxf * layerParamsIn[layerParamIdx].vidxToWorld;

                float planeDist = dot(float4(worldf, 1.f), layerParamsIn[layerParamIdx].slicePlane);
                bool isInside = abs(planeDist) > abs(0.5f * layerParamsIn[layerParamIdx].slicePlaneThickness);
                if (layerParamsIn[layerParamIdx].slicePlaneThickness > 0.f && isInside)
                {
                    density4.w = 0.f;
                }
                if (layerParamsIn[layerParamIdx].slicePlaneThickness < 0.f && !isInside)
                {
                    density4.w = 0.f;
                }
            }
        }
    }

    NvFlowLocalWrite4f(densityShadowOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, density4);
}
