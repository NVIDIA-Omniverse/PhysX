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

RWTexture3D<float4> curlOut;
RWTexture3D<float4> lowpassOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(globalParamsIn.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + globalParamsIn.blockIdxOffset;

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(tableIn, globalParamsIn.table, blockIdx, threadIdx).xyz;

    // curl and lowpass 0
    float4 temp;
    float4 curl = float4(0.f, 0.f, 0.f, 0.f);
    float4 val = float4(0.f, 0.f, 0.f, 0.f);

    temp = velocityIn[readIdx + int3(-1, 0, 0)];
    curl.y += temp.z;
    curl.z -= temp.y;
    val += temp;

    temp = velocityIn[readIdx + int3(+1, 0, 0)];
    curl.y -= temp.z;
    curl.z += temp.y;
    val += temp;

    temp = velocityIn[readIdx + int3(0, -1, 0)];
    curl.x -= temp.z;
    curl.z += temp.x;
    val += temp;

    temp = velocityIn[readIdx + int3(0, +1, 0)];
    curl.x += temp.z;
    curl.z -= temp.x;
    val += temp;

    temp = velocityIn[readIdx + int3(0, 0, -1)];
    curl.y -= temp.x;
    curl.x += temp.y;
    val += temp;

    temp = velocityIn[readIdx + int3(0, 0, +1)];
    curl.y += temp.x;
    curl.x -= temp.y;
    val += temp;

    val += velocityIn[readIdx];

    val *= (1.f / 7.f);

    curl.w = sqrt(curl.x*curl.x + curl.y*curl.y + curl.z*curl.z);

    NvFlowLocalWrite4f(curlOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, curl);
    NvFlowLocalWrite4f(lowpassOut, tableIn, globalParamsIn.table, blockIdx, threadIdx, val);
}
