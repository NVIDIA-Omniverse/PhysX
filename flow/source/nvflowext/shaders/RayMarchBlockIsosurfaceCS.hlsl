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

#include "NvFlowRayMarchIsosurface.hlsli"

ConstantBuffer<NvFlowRayMarchShaderParams> globalParamsIn;
StructuredBuffer<NvFlowRayMarchIsosurfaceLayerShaderParams> layerParamsIn;

StructuredBuffer<uint> tableIn;

Texture3D<float> densityIn;
SamplerState densitySampler;

Texture2D<float> depthIn;

Texture2D<float4> colorIn;
RWTexture2D<float4> colorOut;

float3 crossProduct(float3 a, float3 b)
{
    return float3(a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

float4 depthToWorld(int2 depthIdx, float sceneDepth)
{
    float2 pixelCenter = float2(depthIdx)+0.5f;
    float2 inUV = pixelCenter * float2(globalParamsIn.depthWidthInv, globalParamsIn.depthHeightInv);
    float4 ndc = float4(2.f * inUV.x - 1.f, -2.f * inUV.y + 1.f, sceneDepth, 1.f);
    float4 sceneWorld = mul(ndc, globalParamsIn.projectionJitteredInv);
    sceneWorld = mul(sceneWorld, globalParamsIn.viewInv);
    if (sceneWorld.w != 0.f)
    {
        sceneWorld.xyz *= (1.f / sceneWorld.w);
    }
    return sceneWorld;
}

float4 computeDepthNormalSample(int2 depthIdxA, int2 depthIdxB)
{
    float depthA = depthIn[depthIdxA];
    float depthB = depthIn[depthIdxB];
    float depth = depthA;
    int2 depthIdx = depthIdxA;
    bool depthCmp = depthB > depthA;
    if (globalParamsIn.isReverseZ == 0u)
    {
        depthCmp = !depthCmp;
    }
    if (depthCmp)
    {
        depth = depthB;
        depthIdx = depthIdxB;
    }
    return depthToWorld(depthIdx, depth);
}

void computeDepthPositionAndNormal(inout float4 depthPosition, inout float3 depthNormal, float3 rayDir, int2 tidx)
{
    int2 depthIdx = tidx;
    if (globalParamsIn.width != globalParamsIn.depthWidth ||
        globalParamsIn.height != globalParamsIn.depthHeight)
    {
        float2 tidxf = float2(tidx)+float2(0.5f, 0.5f);
        float2 inUV = tidxf * float2(globalParamsIn.widthInv, globalParamsIn.heightInv);
        float2 depthIdxf = float2(globalParamsIn.depthWidth, globalParamsIn.depthHeight) * inUV;
        depthIdx = int2(floor(depthIdxf));
    }

    depthPosition = depthToWorld(depthIdx, depthIn[depthIdx]);
    float4 depthPosition_dx = computeDepthNormalSample(int2(depthIdx.x - 1, depthIdx.y), int2(depthIdx.x + 1, depthIdx.y));
    float4 depthPosition_dy = computeDepthNormalSample(int2(depthIdx.x, depthIdx.y - 1), int2(depthIdx.x, depthIdx.y + 1));

    depthNormal = -rayDir;
    if (depthPosition.w != 0.f && depthPosition_dx.w != 0.f && depthPosition_dy.w != 0.f)
    {
        float3 a = depthPosition_dx.xyz - depthPosition.xyz;
        float3 b = depthPosition_dy.xyz - depthPosition.xyz;
        depthNormal = crossProduct(a, b);
        depthNormal = normalize(depthNormal);
        // normal points in ray dir direction
        if (dot(depthNormal, rayDir) > 0.f)
        {
            depthNormal = -depthNormal;
        }
    }
}

float3 rayDirectionDebugVis(float3 rayDir)
{
    float u = 0.f;
    float v = 0.f;
    float3 color = float3(1.f, 1.f, 1.f);
    if (abs(rayDir.x) > abs(rayDir.y) && abs(rayDir.x) > abs(rayDir.z))
    {
        u = rayDir.y / rayDir.x;
        v = rayDir.z / rayDir.x;
        if (rayDir.x > 0.f)
        {
            color = float3(1.f, 0.f, 0.f);
        }
        else
        {
            color = float3(1.f, 1.f, 0.f);
        }
    }
    else if (abs(rayDir.y) > abs(rayDir.x) && abs(rayDir.y) > abs(rayDir.z))
    {
        u = rayDir.x / rayDir.y;
        v = rayDir.z / rayDir.y;
        if (rayDir.y > 0.f)
        {
            color = float3(0.f, 1.f, 0.f);
        }
        else
        {
            color = float3(0.f, 1.f, 1.f);
        }
    }
    else
    {
        u = rayDir.x / rayDir.z;
        v = rayDir.y / rayDir.z;
        if (rayDir.z > 0.f)
        {
            color = float3(0.f, 0.f, 1.f);
        }
        else
        {
            color = float3(1.f, 0.f, 1.f);
        }
    }
    int x = int(floor(4.f * u)) & 1;
    int y = int(floor(4.f * v)) & 1;
    int checker = x ^ y;
    if (checker == 0)
    {
        color = float3(0.f, 0.f, 0.f);
    }
    return color;
}

[numthreads(8, 4, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    int2 tidx = int2(dispatchThreadID.xy);

    if (tidx.x >= globalParamsIn.width || tidx.y >= globalParamsIn.height)
    {
        return;
    }

    float2 pixelCenter = float2(tidx) + 0.5f;
    float2 inUV = pixelCenter * float2(globalParamsIn.widthInv, globalParamsIn.heightInv);

    float w00 = (1.f - inUV.x) * (1.f - inUV.y);
    float w10 = (inUV.x) * (1.f - inUV.y);
    float w01 = (1.f - inUV.x) * (inUV.y);
    float w11 = (inUV.x) * (inUV.y);

    float3 rayDir = normalize(w00 * globalParamsIn.rayDir00.xyz + w10 * globalParamsIn.rayDir10.xyz + w01 * globalParamsIn.rayDir01.xyz + w11 * globalParamsIn.rayDir11.xyz);
    float3 rayOrigin = w00 * globalParamsIn.rayOrigin00.xyz + w10 * globalParamsIn.rayOrigin10.xyz + w01 * globalParamsIn.rayOrigin01.xyz + w11 * globalParamsIn.rayOrigin11.xyz;

    float3 rayDirInv = float3(1.f, 1.f, 1.f) / rayDir;

    float4 depthPosition;
    float3 depthNormal;
    computeDepthPositionAndNormal(depthPosition, depthNormal, rayDir, tidx);

    float sceneT = globalParamsIn.maxWorldDistance;
    if (depthPosition.w != 0.f)
    {
        const float ep = 0.01f;
        float plane_denom = dot(depthNormal, rayDir);
        if (abs(plane_denom) > ep)
        {
            sceneT = dot(depthNormal, depthPosition.xyz - rayOrigin) / plane_denom;
        }
        else
        {
            sceneT = dot(rayDir, depthPosition.xyz - rayOrigin);
        }
    }

    float maxWorldDistance = min(globalParamsIn.maxWorldDistance, sceneT);

    float4 sum = float4(0.f, 0.f, 0.f, 1.f);
    for (uint layerParamIdx = 0u; layerParamIdx < globalParamsIn.numLayers; layerParamIdx++)
    {
        if (layerParamsIn[layerParamIdx].refractionMode != 0u)
        {

            for (int bounceIdx = 0; bounceIdx < 3; bounceIdx++)
            {
                float debugHitT = 0.f;
                float hitT = 0.f;
                uint hitEdge = NV_FLOW_ISOSURFACE_FALLING_EDGE;
                float initVal = 0.f;

                NvFlowRayMarchBlocksIsosurface(
                    densityIn,
                    densitySampler,
                    tableIn,
                    globalParamsIn.levelParamsDensity,
                    layerParamsIn[layerParamIdx],
                    NV_FLOW_ISOSURFACE_V0V1_MASK,
                    (hitEdge ^ 3),                    // trace opposite edge of last hit
                    initVal,
                    rayOrigin,
                    0.f,
                    rayDir,
                    maxWorldDistance,
                    rayDirInv,
                    hitEdge,
                    hitT,
                    debugHitT
                );

                if (!bool(hitEdge))
                {
                    break;
                }

                float3 vidxf = (rayDir * hitT + rayOrigin) * layerParamsIn[layerParamIdx].cellSizeInv;
                int3 vidx = NvFlowFloor_i(vidxf);

                float3 hitNormal = NvFlowRayMarchBlocksIsosurfaceComputeNormal(
                    densityIn,
                    densitySampler,
                    tableIn,
                    globalParamsIn.levelParamsDensity,
                    layerParamsIn[layerParamIdx],
                    vidxf
                );

                // advance ray
                rayOrigin = hitT * rayDir + rayOrigin;

                // bend ray
                float ior;
                float inverseIor;
                if (bool(hitEdge & NV_FLOW_ISOSURFACE_RISING_EDGE))
                {
                    ior = layerParamsIn[layerParamIdx].fluidIoR;
                    inverseIor = 1.f / layerParamsIn[layerParamIdx].fluidIoR;

                    // default normal has same sign as gradient vector, need to flip
                    hitNormal = -hitNormal;
                }
                else
                {
                    ior = 1.f / layerParamsIn[layerParamIdx].fluidIoR;
                    inverseIor = layerParamsIn[layerParamIdx].fluidIoR;
                }
                float cos_i = max(0.f, -dot(rayDir, hitNormal));
                float tmp = (-cos_i * cos_i + 1.f) * (-1.f / (ior * ior)) + 1.f;
                float cos_t = sqrt(max(tmp, 0.0));
                float3 t = inverseIor * rayDir + (inverseIor * cos_i - cos_t) * hitNormal;

                // commmit new ray
                rayDir = normalize(t);
                rayDirInv = 1.f / rayDir;
            }

            // show final ray direction in grid form
            sum.xyz = rayDirectionDebugVis(rayDir);
            sum.a = 0.5f;
        }
        else
        {
            float debugHitT = 0.f;
            float hitT = 0.f;
            uint hitEdge = 0u;
            float initVal = 0.f;

            NvFlowRayMarchBlocksIsosurface(
                densityIn,
                densitySampler,
                tableIn,
                globalParamsIn.levelParamsDensity,
                layerParamsIn[layerParamIdx],
                NV_FLOW_ISOSURFACE_V0V1_MASK,
                NV_FLOW_ISOSURFACE_RISING_EDGE,
                initVal,
                rayOrigin,
                0.f,
                rayDir,
                maxWorldDistance,
                rayDirInv,
                hitEdge,
                hitT,
                debugHitT
            );

            if (bool(hitEdge))
            {
                float3 vidxf = (rayDir * hitT + rayOrigin) * layerParamsIn[layerParamIdx].cellSizeInv;
                int3 vidx = NvFlowFloor_i(vidxf);

                if ((vidx.x & 3) == 0 || (vidx.y & 3) == 0)
                {
                    sum = float4(0.f, 0.8f, 0.f, 0.f);
                }
                else
                {
                    sum = float4(0.3f, 0.3f, 0.3f, 0.f);
                }

                if (bool(layerParamsIn[layerParamIdx].visualizeNormals))
                {
                    float3 hitNormal = NvFlowRayMarchBlocksIsosurfaceComputeNormal(
                        densityIn,
                        densitySampler,
                        tableIn,
                        globalParamsIn.levelParamsDensity,
                        layerParamsIn[layerParamIdx],
                        vidxf
                    );

                    sum = float4(abs(hitNormal), 0.f);
                }
            }
            if (layerParamsIn[layerParamIdx].enableBlockWireframe != 0u)
            {
                if (debugHitT > 0.f && (hitEdge == 0u || debugHitT < hitT))
                {
                    sum = float4(0.8f, 0.8f, 0.8f, 0.f);
                }
            }
        }
    }

    float4 color = colorIn[tidx];

    sum.rgb *= globalParamsIn.compositeColorScale;

    color.rgb = sum.rgb + sum.a * color.rgb;

    colorOut[tidx] = color;
}
