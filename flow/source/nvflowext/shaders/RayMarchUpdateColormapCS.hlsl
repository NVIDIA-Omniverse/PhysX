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

#include "RayMarchParams.h"

ConstantBuffer<RayMarchUpdateColormapCS_GlobalParams> globalParamsIn;
StructuredBuffer<RayMarchUpdateColormapCS_LayerParams> layerParamsIn;
StructuredBuffer<RayMarchUpdateColormapCS_PointParams> pointParamsIn;

RWTexture2D<float4> gColormapOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    int threadIdx = int(dispatchThreadID.x);
    uint layerParamIdx = dispatchThreadID.y;

    if (threadIdx >= int(globalParamsIn.dim))
    {
        return;
    }

    if (globalParamsIn.numLayers == 0u)
    {
        gColormapOut[int2(threadIdx, 0u)] = float4(0.f, 1.f, 0.f, 1.f);
        return;
    }

    float u = (float(threadIdx) + 0.5f) * globalParamsIn.dimInv;

    //uint numPts = layerParamsIn[layerParamIdx].numPts;
    uint beginPtIdx = layerParamsIn[layerParamIdx].beginPtIdx;
    uint endPtIdx = layerParamsIn[layerParamIdx].endPtIdx;

    uint xlowIdx = ~0u;
    float xlowVal = 0.f;
    uint xhighIdx = ~0u;
    float xhighVal = 0.f;
    for (uint ptIdx = beginPtIdx; ptIdx < endPtIdx; ptIdx++)
    {
        float x = pointParamsIn[ptIdx].x;
        if (x <= u && (x >= xlowVal || xlowIdx == ~0u))
        {
            xlowIdx = ptIdx;
            xlowVal = x;
        }
        if (x >= u && (x <= xhighVal || xhighIdx == ~0u))
        {
            xhighIdx = ptIdx;
            xhighVal = x;
        }
    }

    float4 a = xlowIdx != ~0u ? pointParamsIn[xlowIdx].color : float4(0.f, 0.f, 0.f, 0.f);
    float4 b = xhighIdx != ~0u ? pointParamsIn[xhighIdx].color : float4(1.f, 1.f, 1.f, 1.f);
    float t = (u - xlowVal) / (xhighVal - xlowVal);
    float4 c = (1.f - t) * a + t * b;

    c.xyz *= layerParamsIn[layerParamIdx].colorScale;

    gColormapOut[int2(threadIdx, layerParamIdx)] = c;
}
