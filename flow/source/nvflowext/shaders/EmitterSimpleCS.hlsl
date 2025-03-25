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

#include "EmitterParams.h"

ConstantBuffer<EmitterSphereCS_Params> gParams;

StructuredBuffer<uint> gTable;

Texture3D<float4> valueIn;
Texture3D<float4> velocityIn;
SamplerState samplerIn;

RWTexture3D<float4> valueOut;

float computeCoverage(uint subStepIdx, uint blockIdx, int3 threadIdx, float3 worldPos)
{
    // trace velocity field
    float coverage = 0.f;
    if (gParams.numTraceSamples != 0u)
    {
        float3 threadIdxf = float3(threadIdx) + float3(0.5f, 0.5f, 0.5f);

        float stepSize = 1.f / float(gParams.numTraceSamples);
        for (uint velSampleIdx = 0u; velSampleIdx < gParams.numTraceSamples && velSampleIdx < 256u; velSampleIdx++)
        {
            float3 velocityThreadIdxf = gParams.valueToVelocityBlockScale * threadIdxf;
            float4 velocity = NvFlowLocalReadLinearSafe4f(velocityIn, samplerIn, gTable, gParams.velocityTable, blockIdx, velocityThreadIdxf);

            float3 displacement = stepSize * gParams.traceDeltaTime * gParams.cellSizeInv * velocity.xyz;

            float4 localPos = mul(float4(worldPos, 1.f), gParams.worldToLocals[subStepIdx]);
            localPos.xyz -= gParams.position;
            bool radial = dot(localPos.xyz, localPos.xyz) < gParams.radius2;
            coverage = max(coverage, (radial ? 1.f : 0.f));

            worldPos -= gParams.vidxToWorld * displacement;
            threadIdxf -= displacement;
        }
    }
    else
    {
        float4 localPos = mul(float4(worldPos, 1.f), gParams.worldToLocals[subStepIdx]);
        localPos.xyz -= gParams.position;
        bool radial = dot(localPos.xyz, localPos.xyz) < gParams.radius2;
        coverage = (radial ? 1.f : 0.f);
    }
    return coverage;
}

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID)
{
    int3 threadIdx = NvFlowComputeThreadIdx(gParams.table, dispatchThreadID.x);
    uint locationIdx = groupID.y + gParams.blockIdxOffset;

    int4 location = int4(
        int(locationIdx % gParams.locationExtent.x) + gParams.locationOffset.x,
        int((locationIdx / gParams.locationExtent.x) % gParams.locationExtent.y) + gParams.locationOffset.y,
        int(locationIdx / (gParams.locationExtent.x * gParams.locationExtent.y)) + gParams.locationOffset.z,
        gParams.locationOffset.w
    );

    uint blockIdx = NvFlowLocationToBlockIdx(gTable, gParams.table, location);

    if (blockIdx >= gParams.table.numLocations)
    {
        return;
    }

    int3 vidx = (location.xyz << gParams.table.blockDimBits) + threadIdx;
    float3 vidxf = float3(vidx) + float3(0.5f, 0.5f, 0.5f);
    float3 worldPos = gParams.vidxToWorld * vidxf;

    float4 value = float4(0.f, 0.f, 0.f, 0.f);
    bool valueIsValid = false;

    for (uint subStepIdx = 0u; subStepIdx < gParams.numSubSteps; subStepIdx++)
    {
        float w = float(subStepIdx + 1u) * gParams.numSubStepsInv;

        float coverage = 0.f;
        if (gParams.multisample != 0u)
        {
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(-0.25f, -0.25f, -0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(+0.25f, -0.25f, -0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(-0.25f, +0.25f, -0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(+0.25f, +0.25f, -0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(-0.25f, -0.25f, +0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(+0.25f, -0.25f, +0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(-0.25f, +0.25f, +0.25f) * gParams.vidxToWorld);
            coverage += 0.125f * computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos + float3(+0.25f, +0.25f, +0.25f) * gParams.vidxToWorld);
        }
        else
        {
            coverage = computeCoverage(subStepIdx, blockIdx, threadIdx, worldPos);
        }

        if (coverage > 0.f)
        {
            if (!valueIsValid)
            {
                value = NvFlowLocalRead4f(valueIn, gTable, gParams.table, blockIdx, threadIdx);
                valueIsValid = true;
            }

            float4 valueRate = gParams.numSubStepsInv * gParams.deltaTime * gParams.coupleRate * coverage;
            valueRate = max(float4(0.f, 0.f, 0.f, 0.f), valueRate);
            valueRate = min(float4(1.f, 1.f, 1.f, 1.f), valueRate);

            float4 targetValue = gParams.targetValue;
            if (gParams.physicsVelocityScale != 0.f)
            {
                float4 localPos = mul(float4(worldPos, 1.f), gParams.worldToLocals[subStepIdx]);
                float4 worldOld = mul(localPos, gParams.localToWorldOlds[subStepIdx]);
                float3 physicsVelocity = (worldPos - worldOld.xyz) * gParams.physicsDeltaTimeInv * float(gParams.numSubSteps);
                targetValue.xyz += gParams.physicsVelocityScale * physicsVelocity;
            }

            value += valueRate * (targetValue - value);
        }
    }

    if (valueIsValid)
    {
        NvFlowLocalWrite4f(valueOut, gTable, gParams.table, blockIdx, threadIdx, value);
    }
}
