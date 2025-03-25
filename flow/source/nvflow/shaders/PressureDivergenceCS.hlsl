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

#include "PressureParams.h"

ConstantBuffer<PressureDivergenceParams> gParams;

StructuredBuffer<uint> gTable;

Texture3D<float4> velocityIn;

RWTexture3D<float2> pressureOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(gParams.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + gParams.blockIdxOffset;

    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(gTable, gParams.table, blockIdx, threadIdx).xyz;

    float vxp = velocityIn[readIdx + int3(+1, 0, 0)].x;
    float vxn = velocityIn[readIdx + int3(-1, 0, 0)].x;
    float vyp = velocityIn[readIdx + int3(0, +1, 0)].y;
    float vyn = velocityIn[readIdx + int3(0, -1, 0)].y;
    float vzp = velocityIn[readIdx + int3(0, 0, +1)].z;
    float vzn = velocityIn[readIdx + int3(0, 0, -1)].z;

    float d = 0.5f * (vxp - vxn + vyp - vyn + vzp - vzn);

    float dext = gParams.dextScale * velocityIn[readIdx].w;
    d -= dext;

    NvFlowLocalWrite2f(pressureOut, gTable, gParams.table, blockIdx, threadIdx, float2(0.f, d));
}
