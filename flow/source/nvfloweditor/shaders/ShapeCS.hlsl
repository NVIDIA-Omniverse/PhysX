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

#include "ShapeParams.h"

ConstantBuffer<ShapeRendererParams> paramsIn;

StructuredBuffer<float4> spherePositionRadiusIn;

RWTexture2D<float> depthOut;
RWTexture2D<float4> colorOut;

bool raySphereIntersection(float3 o, float3 d, float3 c, float r, inout float t)
{
    bool ret = false;
    float3 l = c - o;
    float s = dot(l, d);
    float l2 = dot(l, l);
    float r2 = r * r;
    if (s >= 0.f || l2 <= r2)
    {
        float s2 = s * s;
        float m2 = l2 - s2;
        if (m2 <= r2)
        {
            float q = sqrt(r2 - m2);
            if (l2 > r2)
            {
                t = s - q;
            }
            else
            {
                t = s + q;
            }
            ret = t > 0.f;
        }
    }
    return ret;
}

[numthreads(8, 8, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    int2 tidx = int2(dispatchThreadID.xy);

    float2 pixelCenter = float2(tidx) + 0.5f;
    float2 inUV = pixelCenter * float2(paramsIn.widthInv, paramsIn.heightInv);

    float w00 = (1.f - inUV.x) * (1.f - inUV.y);
    float w10 = (inUV.x) * (1.f - inUV.y);
    float w01 = (1.f - inUV.x) * (inUV.y);
    float w11 = (inUV.x) * (inUV.y);

    float3 rayDir = normalize(w00 * paramsIn.rayDir00.xyz + w10 * paramsIn.rayDir10.xyz + w01 * paramsIn.rayDir01.xyz + w11 * paramsIn.rayDir11.xyz);
    float3 rayOrigin = w00 * paramsIn.rayOrigin00.xyz + w10 * paramsIn.rayOrigin10.xyz + w01 * paramsIn.rayOrigin01.xyz + w11 * paramsIn.rayOrigin11.xyz;

    float globalDepth = paramsIn.clearDepth;
    float4 globalColor = paramsIn.clearColor;

    for (uint sphereIdx = 0u; sphereIdx < paramsIn.numSpheres; sphereIdx++)
    {
        float4 spherePositionRadius = spherePositionRadiusIn[sphereIdx];
        float hitT = -1.f;
        if (raySphereIntersection(rayOrigin, rayDir, spherePositionRadius.xyz, spherePositionRadius.w, hitT))
        {
            float4 worldPos = float4(rayDir * hitT + rayOrigin, 1.f);

            float4 clipPos = mul(worldPos, paramsIn.view);
            clipPos = mul(clipPos, paramsIn.projection);

            float depth = clipPos.z / clipPos.w;

            bool isVisible = bool(paramsIn.isReverseZ) ? (depth > globalDepth) : (depth < globalDepth);
            if (isVisible)
            {
                float3 n = normalize(worldPos.xyz - spherePositionRadius.xyz);

                float4 color = float4(abs(n), 1.f);

                globalDepth = depth;
                globalColor = color;
            }
        }
    }

    depthOut[tidx] = globalDepth;
    colorOut[tidx] = globalColor;
}
