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

#include "AdvectionParams.h"

float4 advectSingleRead4f(Texture3D<float4> textureRead, StructuredBuffer<uint> table, NvFlowSparseLevelParams tableParams, uint blockIdx, int3 threadIdx)
{
    // can ignore .w, since this block will always be valid
    int3 readIdx = NvFlowSingleVirtualToReal(table, tableParams, blockIdx, threadIdx).xyz;
    return textureRead[readIdx];
}

float4 advectReadLinear4f(uint globalFetch, Texture3D<float4> textureRead, SamplerState textureSampler, StructuredBuffer<uint> table, NvFlowSparseLevelParams tableParams, uint blockIdx, float3 threadIdxf)
{
    float4 ret;
    if (globalFetch != 0u)
    {
        ret = NvFlowLocalReadLinearSafe4f(textureRead, textureSampler, table, tableParams, blockIdx, threadIdxf);
    }
    else
    {
        ret = NvFlowLocalReadLinear4f(textureRead, textureSampler, table, tableParams, blockIdx, threadIdxf);
    }
    return ret;
}

float3 advectVirtualToRealf(uint globalFetch, StructuredBuffer<uint> table, NvFlowSparseLevelParams tableParams, uint blockIdx, float3 threadIdxf)
{
    float3 ret;
    if (globalFetch != 0u)
    {
        ret = NvFlowLocalVirtualToRealSafef(table, tableParams, blockIdx, threadIdxf);
    }
    else
    {
        ret = NvFlowLocalVirtualToRealf(table, tableParams, blockIdx, threadIdxf);
    }
    return ret;
}

float4 combustSimulate(AdvectionCombustionParams combustParams, float4 density4, float deltaTime)
{
    float temp = density4.x;
    float fuel = density4.y;
    float burn = density4.z;
    float smoke = density4.w;

    if (combustParams.combustionEnabled != 0u)
    {
        float burnTemp = temp;

        // this is using normalized temperature
        //  A temperature of 0.f means neutral buoyancy
        //  A temperature of 1.f has the maximum burn rate
        if (burnTemp >= combustParams.ignitionTemp && fuel > 0.f)
        {
            // burn rate is proportional to temperature beyond ignition temperature
            burn = combustParams.burnPerTemp * (min(max(burnTemp, 0.f), 1.f) - combustParams.ignitionTemp) * deltaTime;

            // limit burn based on available fuel
            burn = min(burn, combustParams.burnPerFuel * fuel);

            // update fuel consumption based on burn
            fuel -= combustParams.fuelPerBurn * burn;

            // update temperature based on burn
            temp += combustParams.tempPerBurn * burn;

            // update smoke based on burn
            smoke += combustParams.smokePerBurn * burn;
        }

        // approximate cooling with damping (instead of solving the heat equation)
        {
            temp -= combustParams.coolingRate * deltaTime * temp;
        }

        // limit maximum temperature for stability
        {
            temp = clamp(temp, -1.f, 1.f);
        }
    }

    return float4(temp, fuel, burn, smoke);
}

float4 combustVelocity(AdvectionCombustionParams combustParams, float4 velocity4, float4 density4, float deltaTime)
{
    float temp = density4.x;
    float burn = density4.z;
    float smoke = density4.w;

    if (combustParams.combustionEnabled != 0u)
    {
        // apply buoyancy
        velocity4.xyz -= deltaTime * combustParams.buoyancyPerTemp * temp * combustParams.gravity;

        // generate a divergence offset for the pressure solver, place on w velocity channel
        velocity4.w = combustParams.divergencePerBurn * burn;
    }

    // not technically part of combustion, and easily disabled on its own
    if (combustParams.buoyancyPerSmoke != 0.f)
    {
        velocity4.xyz -= deltaTime * combustParams.buoyancyPerSmoke * clamp(smoke, 0.f, combustParams.buoyancyMaxSmoke) * combustParams.gravity;
    }

    return velocity4;
}

float3 computeDisplacement(SemiLagrangianParams params, float3 velocity)
{
    float3 displacement = params.deltaTime * params.cellSizeInv * velocity;

    if (params.globalFetch == 0u)
    {
        displacement = max(displacement, -params.maxDisplacement);
        displacement = min(displacement, +params.maxDisplacement);
    }

    return displacement;
}

float4 semiLagrangian4f(
    Texture3D<float4> valueIn,
    Texture3D<float4> velocityIn,
    SamplerState valueSampler,
    SamplerState velocitySampler,
    StructuredBuffer<uint> table,
    NvFlowSparseLevelParams valueTableParams,
    NvFlowSparseLevelParams velocityTableParams,
    SemiLagrangianParams params,
    uint blockIdx,
    int3 threadIdx
)
{
    float3 velocityFetchIdxf = params.valueToVelocityBlockScale * (float3(threadIdx) + float3(0.5f, 0.5f, 0.5f));
    float4 velocity = advectReadLinear4f(params.globalFetch, velocityIn, velocitySampler, table, velocityTableParams, blockIdx, velocityFetchIdxf);

    float3 fetchIdxf = float3(threadIdx) + float3(0.5f, 0.5f, 0.5f) - computeDisplacement(params, velocity.xyz);

    float4 value = advectReadLinear4f(params.globalFetch, valueIn, valueSampler, table, valueTableParams, blockIdx, fetchIdxf);

    return value;
}

void minmax4(inout float4 minVal, inout float4 maxVal, float4 val)
{
    minVal = min(minVal, val);
    maxVal = max(maxVal, val);
}

float4 macCormackCorrection4f(
    Texture3D<float4> predictIn,
    Texture3D<float4> valueIn,
    Texture3D<float4> velocityIn,
    SamplerState predictSampler,
    SamplerState valueSampler,
    SamplerState velocitySampler,
    StructuredBuffer<uint> table,
    NvFlowSparseLevelParams valueTableParams,
    NvFlowSparseLevelParams velocityTableParams,
    MacCormackParams params,
    uint blockIdx,
    int3 threadIdx
)
{
    float4 predict = advectSingleRead4f(predictIn, table, valueTableParams, blockIdx, threadIdx);

    float4 value = predict;

    bool shouldCorrect =
        abs(predict.x) >= params.blendThreshold.x ||
        abs(predict.y) >= params.blendThreshold.y ||
        abs(predict.z) >= params.blendThreshold.z ||
        abs(predict.w) >= params.blendThreshold.w;

    if (shouldCorrect)
    {
        // get this cell's velocity
        float3 velocityFetchIdxf = params.base.valueToVelocityBlockScale * (float3(threadIdx) + float3(0.5f, 0.5f, 0.5f));
        float4 velocity = advectReadLinear4f(params.base.globalFetch, velocityIn, velocitySampler, table, velocityTableParams, blockIdx, velocityFetchIdxf);
        // trace backwards in time
        float3 revFetchIdxf = float3(threadIdx) + float3(0.5f, 0.5f, 0.5f) + computeDisplacement(params.base, velocity.xyz);
        // advect semi-Lagrangian result
        float4 predictRev = advectReadLinear4f(params.base.globalFetch, predictIn, predictSampler, table, valueTableParams, blockIdx, revFetchIdxf);

        // sample last frame's value for this cell
        float4 valueOld = advectSingleRead4f(valueIn, table, valueTableParams, blockIdx, threadIdx);

        value = predict + 0.5f * params.blendFactor * (valueOld - predictRev);

        // add clamping for stability
        {
            float3 fetchIdxf = float3(threadIdx) + float3(0.5f, 0.5f, 0.5f) - computeDisplacement(params.base, velocity.xyz);
            float3 ridxf = advectVirtualToRealf(params.base.globalFetch, table, valueTableParams, blockIdx, fetchIdxf);

            int3 c = NvFlowFloor_i(ridxf - float3(0.5f, 0.5f, 0.5f));

            float4 minVal = valueIn[c + int3(0, 0, 0)];
            float4 maxVal = minVal;
            minmax4(minVal, maxVal, valueIn[c + int3(1, 0, 0)]);
            minmax4(minVal, maxVal, valueIn[c + int3(0, 1, 0)]);
            minmax4(minVal, maxVal, valueIn[c + int3(1, 1, 0)]);
            minmax4(minVal, maxVal, valueIn[c + int3(0, 0, 1)]);
            minmax4(minVal, maxVal, valueIn[c + int3(1, 0, 1)]);
            minmax4(minVal, maxVal, valueIn[c + int3(0, 1, 1)]);
            minmax4(minVal, maxVal, valueIn[c + int3(1, 1, 1)]);

            value = max(minVal, value);
            value = min(maxVal, value);
        }
    }

    // apply damping/fade
    {
        float4 correction = sign(value) * min(abs(value), max(abs(value * params.dampingRate), abs(params.base.deltaTime * params.fadeRate)));
        value -= correction;
    }

    return value;
}
