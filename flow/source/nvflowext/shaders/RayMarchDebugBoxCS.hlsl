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

#include "RayMarchCommon.hlsli"

ConstantBuffer<RayMarchSimpleParams> paramsIn;

StructuredBuffer<uint> tableIn;

Texture3D<float4> densityIn;
SamplerState densitySampler;

Texture2D<float4> colormapIn;
SamplerState colormapSampler;

Texture2D<float4> colorIn;
RWTexture2D<float4> colorOut;

[numthreads(8, 8, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    int2 tidx = int2(dispatchThreadID.xy) + paramsIn.pixelMini;

    if (tidx.x >= paramsIn.pixelMaxi.x || tidx.y >= paramsIn.pixelMaxi.y)
    {
        return;
    }

    float2 pixelCenter = float2(tidx) + 0.5f;
    float2 inUV = pixelCenter * float2(paramsIn.widthInv, paramsIn.heightInv);

    float w00 = (1.f - inUV.x) * (1.f - inUV.y);
    float w10 = (inUV.x) * (1.f - inUV.y);
    float w01 = (1.f - inUV.x) * (inUV.y);
    float w11 = (inUV.x) * (inUV.y);

    float3 rayDir = normalize(w00 * paramsIn.rayDir00.xyz + w10 * paramsIn.rayDir10.xyz + w01 * paramsIn.rayDir01.xyz + w11 * paramsIn.rayDir11.xyz);
    float3 rayOrigin = w00 * paramsIn.rayOrigin00.xyz + w10 * paramsIn.rayOrigin10.xyz + w01 * paramsIn.rayOrigin01.xyz + w11 * paramsIn.rayOrigin11.xyz;

    float3 rayDirInv = float3(1.f, 1.f, 1.f) / rayDir;

    float3 boxWorldMin = float3(paramsIn.baseTable.locationMin.xyz) * paramsIn.base.blockSizeWorld;
    float3 boxWorldMax = float3(paramsIn.baseTable.locationMax.xyz) * paramsIn.base.blockSizeWorld;

    float tmin_world;
    float tmax_world;
    bool isHit = intersectBox(rayDir, rayDirInv, boxWorldMin - rayOrigin, boxWorldMax - rayOrigin, tmin_world, tmax_world);

    tmin_world = max(0.f, tmin_world);

    float4 sum = float4(0.f, 0.f, 0.f, 1.f);

    if (isHit)
    {
        float tmin = paramsIn.base.stepSizeInv * tmin_world;
        float tmax = paramsIn.base.stepSizeInv * tmax_world;

        tmin = round(tmin);
        tmax = round(tmax);

        int numSteps = int(tmax - tmin);

        for (int stepIdx = 0; stepIdx < numSteps; stepIdx++)
        {
            float t_world = paramsIn.base.stepSize * (float(stepIdx) + tmin);

            float3 worldPos = rayOrigin + t_world * rayDir;

            float3 vidxf = worldPos * paramsIn.base.blockSizeWorldInv * float3(int3(1, 1, 1) << int3(paramsIn.baseTable.blockDimBits));

            int4 vidx = int4(NvFlowFloor_i(vidxf), paramsIn.base.layer);
            int4 readIdx = NvFlowGlobalVirtualToReal(tableIn, paramsIn.baseTable, vidx);
            float3 readIdxf = float3(readIdx.xyz) + vidxf - float3(vidx.xyz);

            float4 value = bool(readIdx.w & 1) ? densityIn.SampleLevel(densitySampler, readIdxf * paramsIn.baseTable.dimInv, 0.0) : float4(0.f, 0.f, 0.f, 0.f);

            // colormap by temperature
            float colormapU = params.colormapXScale * value.r + params.colormapXOffset;
            float4 color = colormapIn.SampleLevel(colormapSampler, float2(colormapU, paramsIn.base.layerAndLevelColormapV), 0.0);

            int3 threadIdx = vidx.xyz & int3(paramsIn.baseTable.blockDimLessOne);
            float3 threadIdxf = float3(threadIdx) + vidxf - float3(vidx.xyz);

            float3 halfsize = 0.5f * float3(paramsIn.baseTable.blockDimLessOne + int3(1, 1, 1));
            float3 error = abs(threadIdxf - halfsize);
            error = 1.5f * abs(error - (halfsize - float3(1.f, 1.f, 1.f)));
            error = max(float3(0.f, 0.f, 0.f), float3(1.f, 1.f, 1.f) - error);

            float wx = error.x;
            float wy = error.y;
            float wz = error.z;

            float weight = max(wx * wy, max(wy * wz, wz * wx));

            // modulate alpha by smoke
            color.a *= value.a;

            color = max(float4(0.f, 0.f, 0.f, 0.f), color);
            color.a = min(1.f, color.a);

            color.a *= paramsIn.base.alphaScale;

            if (bool(readIdx.w & 1) && weight >= 0.01f)
            {
                color = float4(0.2716f, 0.5795f, 0.04615f, 0.5f * weight);
            }

            sum.rgb = sum.a * (color.a * color.rgb) + sum.rgb;
            sum.a = (1.f - color.a) * sum.a;
        }
    }

    float4 color = colorIn[tidx];

    sum.rgb *= paramsIn.compositeScale.rgb;
    sum.a = 1.f - ((1.f - sum.a) * paramsIn.compositeScale.a);

    color.rgb = sum.rgb + sum.a * color.rgb;

    colorOut[tidx] = color;
}
