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

#include "VorticityParams.h"

ConstantBuffer<VorticityCS_GlobalParams> globalParamsIn;
StructuredBuffer<VorticityCS_LayerParams> layerParamsIn;

StructuredBuffer<uint> tableIn;

Texture3D<float4> velocityIn;
Texture3D<float4> curlIn;
Texture3D<float4> lowpassIn;
Texture3D<float4> coarseDensityIn;

RWTexture3D<float4> velocityOut;

float3 crossProduct(float3 a, float3 b)
{
    return float3(a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(globalParamsIn.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + globalParamsIn.blockIdxOffset;

    uint layerParamIdx = NvFlowGetLayerParamIdx(tableIn, globalParamsIn.table, blockIdx);

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(tableIn, globalParamsIn.table, blockIdx, threadIdx).xyz;

    float4 n = float4(0.f, 0.f, 0.f, 0.f);
    n.x -= curlIn[readIdx + int3(-1, 0, 0)].w;
    n.x += curlIn[readIdx + int3(+1, 0, 0)].w;
    n.y -= curlIn[readIdx + int3(0, -1, 0)].w;
    n.y += curlIn[readIdx + int3(0, +1, 0)].w;
    n.z -= curlIn[readIdx + int3(0, 0, -1)].w;
    n.z += curlIn[readIdx + int3(0, 0, +1)].w;

    n.w = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
    if (n.w > 0.f)
    {
        n.x /= n.w;
        n.y /= n.w;
        n.z /= n.w;
    }

    float4 curl = curlIn[readIdx];

    float3 impulse = crossProduct(n.xyz, curl.xyz);
    float impulseWeight = layerParamsIn[layerParamIdx].forceScale;

    float speedLog = 0.f;
    float speed = 0.f;
    if (layerParamsIn[layerParamIdx].velocityLogMask != 0.f || layerParamsIn[layerParamIdx].velocityLinearMask != 0.f)
    {
        float4 velocityLowpass = lowpassIn[readIdx];
        velocityLowpass += lowpassIn[readIdx + int3(-1, 0, 0)];
        velocityLowpass += lowpassIn[readIdx + int3(+1, 0, 0)];
        velocityLowpass += lowpassIn[readIdx + int3(0, -1, 0)];
        velocityLowpass += lowpassIn[readIdx + int3(0, +1, 0)];
        velocityLowpass += lowpassIn[readIdx + int3(0, 0, -1)];
        velocityLowpass += lowpassIn[readIdx + int3(0, 0, +1)];

        velocityLowpass *= (1.f / 7.f);

        velocityLowpass.w = length(velocityLowpass.xyz);
        speed = velocityLowpass.w;
        speedLog = speed;
        if (speedLog > 0.f)
        {
            speedLog = log2(layerParamsIn[layerParamIdx].velocityLogScale * speed + 1.f);
        }
    }

    float4 velocity = velocityIn[readIdx];

    float4 density4 = coarseDensityIn[readIdx];

    impulseWeight *= max(0.f, (
        layerParamsIn[layerParamIdx].velocityLogMask * speedLog +
        layerParamsIn[layerParamIdx].velocityLinearMask * speed +
        layerParamsIn[layerParamIdx].densityMask * (velocity.w) +
        layerParamsIn[layerParamIdx].constantMask +
        layerParamsIn[layerParamIdx].temperatureMask * density4.x +
        layerParamsIn[layerParamIdx].fuelMask * density4.y +
        layerParamsIn[layerParamIdx].burnMask * density4.z +
        layerParamsIn[layerParamIdx].smokeMask * density4.w
        ));

    velocity.xyz += impulseWeight * impulse;

    NvFlowLocalWrite4f(velocityOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, velocity);
}
