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

#include "shaders/AdvectionParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"

#include "NvFlow.h"

#include "shaders/AdvectionSimpleCS.hlsl.h"
#include "shaders/AdvectionVelocity1CS.hlsl.h"
#include "shaders/AdvectionVelocity2CS.hlsl.h"
#include "shaders/AdvectionDensity1CS.hlsl.h"
#include "shaders/AdvectionDensity2CS.hlsl.h"
#include "shaders/AdvectionDownsampleCS.hlsl.h"
#include "shaders/AdvectionFadeVelocityCS.hlsl.h"
#include "shaders/AdvectionFadeDensityCS.hlsl.h"

namespace
{
    NvFlowFloat4 dampingRate(NvFlowFloat4 damping, float deltaTime)
    {
        damping.x = fminf(damping.x, 1.f);
        damping.y = fminf(damping.y, 1.f);
        damping.z = fminf(damping.z, 1.f);
        damping.w = fminf(damping.w, 1.f);

        NvFlowFloat4 dampingRate;
        dampingRate.x = 1.f - powf(1.f - damping.x, deltaTime);
        dampingRate.y = 1.f - powf(1.f - damping.y, deltaTime);
        dampingRate.z = 1.f - powf(1.f - damping.z, deltaTime);
        dampingRate.w = 1.f - powf(1.f - damping.w, deltaTime);
        return dampingRate;
    }

    void advection_makeCombustParams(AdvectionCombustionParams* dst, const NvFlowAdvectionCombustionParams* params)
    {
        AdvectionCombustionParams combustParams = {};
        combustParams.gravity = params->gravity;
        combustParams.burnPerFuel = 1.f / params->fuelPerBurn;
        combustParams.ignitionTemp = params->ignitionTemp;
        combustParams.burnPerTemp = params->burnPerTemp;
        combustParams.fuelPerBurn = params->fuelPerBurn;
        combustParams.tempPerBurn = params->tempPerBurn;
        combustParams.smokePerBurn = params->smokePerBurn;
        combustParams.divergencePerBurn = params->divergencePerBurn;
        combustParams.buoyancyPerTemp = params->buoyancyPerTemp;
        combustParams.coolingRate = params->coolingRate;
        combustParams.buoyancyPerSmoke = params->buoyancyPerSmoke;
        combustParams.buoyancyMaxSmoke = params->buoyancyMaxSmoke;
        combustParams.combustionEnabled = params->combustionEnabled;
        combustParams.pad3 = 0.f;

        *dst = combustParams;
    }
}

namespace
{
    struct AdvectionSimple
    {
        NvFlowContextInterface contextInterface = {};

        AdvectionSimpleCS_Pipeline advectionSimpleCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    AdvectionSimple* AdvectionSimple_create(const NvFlowOpInterface* opInterface, const NvFlowAdvectionSimplePinsIn* in, NvFlowAdvectionSimplePinsOut* out)
    {
        auto ptr = new AdvectionSimple();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        AdvectionSimpleCS_init(&ptr->contextInterface, in->context, &ptr->advectionSimpleCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(AdvectionSimpleCS_LayerParams));

        return ptr;
    }

    void AdvectionSimple_destroy(AdvectionSimple* ptr, const NvFlowAdvectionSimplePinsIn* in, NvFlowAdvectionSimplePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        AdvectionSimpleCS_destroy(in->context, &ptr->advectionSimpleCS);

        delete ptr;
    }

    void AdvectionSimple_execute(AdvectionSimple* ptr, const NvFlowAdvectionSimplePinsIn* in, NvFlowAdvectionSimplePinsOut* out)
    {
        NvFlowSparseLevelParams* levelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        NvFlowUint numLayers = in->velocity.sparseParams.layerCount;

        if (levelParams->numLocations == 0u)
        {
            NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);
            return;
        }

        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->velocity, &in->velocity);

        // advect
        {
            SemiLagrangianParams advectParams = { };
            advectParams.valueToVelocityBlockScale.x = 1.f;
            advectParams.valueToVelocityBlockScale.y = 1.f;
            advectParams.valueToVelocityBlockScale.z = 1.f;
            advectParams.globalFetch = 0u;
            advectParams.cellSizeInv.x = 1.f;
            advectParams.cellSizeInv.y = 1.f;
            advectParams.cellSizeInv.z = 1.f;
            advectParams.deltaTime = 1.f;
            advectParams.maxDisplacement.x = 15.f;
            advectParams.maxDisplacement.y = 15.f;
            advectParams.maxDisplacement.z = 15.f;

            auto mappedLayer = (AdvectionSimpleCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(AdvectionSimpleCS_LayerParams));

            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
            {
                mappedLayer[layerParamIdx].advectParams = advectParams;
                mappedLayer[layerParamIdx].dampingRate = NvFlowFloat4{ 0.998f, 0.998f, 0.998f, 0.998f };
            }

            NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mappedGlobal = (AdvectionSimpleCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(AdvectionSimpleCS_GlobalParams));

                mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
                mappedGlobal->pad1 = 0u;
                mappedGlobal->pad2 = 0u;
                mappedGlobal->pad3 = 0u;
                mappedGlobal->table = *levelParams;

                NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

                AdvectionSimpleCS_PassParams params = {};
                params.globalParamsIn = globalTransient;
                params.layerParamsIn = layerTransient;
                params.tableIn = in->velocity.sparseBuffer;
                params.valueIn = in->velocity.textureTransient;
                params.valueSampler = ptr->samplerLinear;
                params.valueOut = out->velocity.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                AdvectionSimpleCS_addPassCompute(in->context, &ptr->advectionSimpleCS, gridDim, &params);
            }
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowAdvectionSimple, AdvectionSimple)

namespace
{
    struct AdvectionCombustionVelocity
    {
        NvFlowContextInterface contextInterface = {};

        AdvectionVelocity1CS_Pipeline advectionVelocity1CS = {};
        AdvectionVelocity2CS_Pipeline advectionVelocity2CS = {};
        AdvectionDownsampleCS_Pipeline advectionDownsampleCS = {};
        AdvectionFadeVelocityCS_Pipeline advectionFadeVelocityCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    AdvectionCombustionVelocity* AdvectionCombustionVelocity_create(const NvFlowOpInterface* opInterface, const NvFlowAdvectionCombustionVelocityPinsIn* in, NvFlowAdvectionCombustionVelocityPinsOut* out)
    {
        auto ptr = new AdvectionCombustionVelocity();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        AdvectionVelocity1CS_init(&ptr->contextInterface, in->context, &ptr->advectionVelocity1CS);
        AdvectionVelocity2CS_init(&ptr->contextInterface, in->context, &ptr->advectionVelocity2CS);
        AdvectionDownsampleCS_init(&ptr->contextInterface, in->context, &ptr->advectionDownsampleCS);
        AdvectionFadeVelocityCS_init(&ptr->contextInterface, in->context, &ptr->advectionFadeVelocityCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(AdvectionVelocityCS_LayerParams));

        return ptr;
    }

    void AdvectionCombustionVelocity_destroy(AdvectionCombustionVelocity* ptr, const NvFlowAdvectionCombustionVelocityPinsIn* in, NvFlowAdvectionCombustionVelocityPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        AdvectionVelocity1CS_destroy(in->context, &ptr->advectionVelocity1CS);
        AdvectionVelocity2CS_destroy(in->context, &ptr->advectionVelocity2CS);
        AdvectionDownsampleCS_destroy(in->context, &ptr->advectionDownsampleCS);
        AdvectionFadeVelocityCS_destroy(in->context, &ptr->advectionFadeVelocityCS);

        delete ptr;
    }

    void AdvectionCombustionVelocity_execute(AdvectionCombustionVelocity* ptr, const NvFlowAdvectionCombustionVelocityPinsIn* in, NvFlowAdvectionCombustionVelocityPinsOut* out)
    {
        NvFlowSparseLevelParams* densityLevelParams = &in->density.sparseParams.levels[in->density.levelIdx];
        NvFlowSparseLevelParams* velocityLevelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        if (velocityLevelParams->numLocations == 0u)
        {
            NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);
            NvFlowSparseTexture_passThrough(&out->voxelWeight, &in->voxelWeight);
            NvFlowSparseTexture_passThrough(&out->densityCoarse, &in->densityCoarse);
            return;
        }

        // for now, always pass voxelWeight through
        NvFlowSparseTexture_passThrough(&out->voxelWeight, &in->voxelWeight);

        NvFlowUint numLayers = in->velocity.sparseParams.layerCount;

        // if all layers are disabled, can do passthrough
        NvFlowBool32 anyAdvectionEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyDownsampleEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyForceFadeEnabled = NV_FLOW_FALSE;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];
            if (layerParamsIn->enabled && !in->velocity.sparseParams.layers[layerParamIdx].forceDisableCoreSimulation)
            {
                anyAdvectionEnabled = NV_FLOW_TRUE;
            }
            if (layerParamsIn->downsampleEnabled || layerParamsIn->forceFadeEnabled) // if force fading, downsample so summary can update
            {
                anyDownsampleEnabled = NV_FLOW_TRUE;
            }
            if (layerParamsIn->forceFadeEnabled)
            {
                anyForceFadeEnabled = NV_FLOW_TRUE;
            }
        }
        if (!anyAdvectionEnabled && !anyDownsampleEnabled && !anyForceFadeEnabled)
        {
            NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);
            NvFlowSparseTexture_passThrough(&out->densityCoarse, &in->densityCoarse);
            return;
        }

        /// compute simulation parameters

        NvFlowFloat3 densityBlockDimf = {
            float(densityLevelParams->blockDimLessOne.x + 1u),
            float(densityLevelParams->blockDimLessOne.y + 1u),
            float(densityLevelParams->blockDimLessOne.z + 1u)
        };
        NvFlowFloat3 velocityBlockDimf = {
            float(velocityLevelParams->blockDimLessOne.x + 1u),
            float(velocityLevelParams->blockDimLessOne.y + 1u),
            float(velocityLevelParams->blockDimLessOne.z + 1u)
        };
        float densityMaxDisplacement = fminf(fminf(densityBlockDimf.x, densityBlockDimf.y), densityBlockDimf.z) - 1.f;
        float velocityMaxDisplacement = fminf(fminf(velocityBlockDimf.x, velocityBlockDimf.y), velocityBlockDimf.z) - 1.f;

        NvFlowFloat3 velocityToDensityBlockScale = {
            densityBlockDimf.x / velocityBlockDimf.x,
            densityBlockDimf.y / velocityBlockDimf.y,
            densityBlockDimf.z / velocityBlockDimf.z,
        };

        NvFlowSparseTexture velocityPredict = in->velocityTemp;
        if (!velocityPredict.textureTransient)
        {
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &velocityPredict, &in->velocity);
        }
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->velocity, &in->velocity);
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->densityCoarse, &in->velocity);

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, velocityLevelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mappedGlobal = (AdvectionVelocityCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(AdvectionVelocityCS_GlobalParams));

            mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mappedGlobal->pad1 = 0u;
            mappedGlobal->pad2 = 0u;
            mappedGlobal->pad3 = 0u;
            mappedGlobal->table = *velocityLevelParams;
            mappedGlobal->tableDensity = *densityLevelParams;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        auto mappedLayer = (AdvectionVelocityCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(AdvectionVelocityCS_LayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            AdvectionCombustionParams combustParams = {};
            advection_makeCombustParams(&combustParams, layerParamsIn);

            NvFlowFloat3 blockSizeWorld = in->velocity.sparseParams.layers[layerParamIdx].blockSizeWorld;

            MacCormackParams velocityParams = {};
            velocityParams.base.valueToVelocityBlockScale.x = 1.f;
            velocityParams.base.valueToVelocityBlockScale.y = 1.f;
            velocityParams.base.valueToVelocityBlockScale.z = 1.f;
            velocityParams.base.globalFetch = layerParamsIn->globalFetch;
            velocityParams.base.cellSizeInv.x = 1.f / (blockSizeWorld.x / velocityBlockDimf.x);
            velocityParams.base.cellSizeInv.y = 1.f / (blockSizeWorld.y / velocityBlockDimf.y);
            velocityParams.base.cellSizeInv.z = 1.f / (blockSizeWorld.z / velocityBlockDimf.z);
            velocityParams.base.deltaTime = in->deltaTime;
            velocityParams.base.maxDisplacement.x = velocityMaxDisplacement;
            velocityParams.base.maxDisplacement.y = velocityMaxDisplacement;
            velocityParams.base.maxDisplacement.z = velocityMaxDisplacement;
            velocityParams.blendThreshold = NvFlowFloat4{
                layerParamsIn->velocity.secondOrderBlendThreshold,
                layerParamsIn->velocity.secondOrderBlendThreshold,
                layerParamsIn->velocity.secondOrderBlendThreshold,
                layerParamsIn->divergence.secondOrderBlendThreshold
            };
            velocityParams.blendFactor = NvFlowFloat4{
                layerParamsIn->velocity.secondOrderBlendFactor,
                layerParamsIn->velocity.secondOrderBlendFactor,
                layerParamsIn->velocity.secondOrderBlendFactor,
                layerParamsIn->divergence.secondOrderBlendFactor
            };
            velocityParams.dampingRate = dampingRate(NvFlowFloat4{
                layerParamsIn->velocity.damping,
                layerParamsIn->velocity.damping,
                layerParamsIn->velocity.damping,
                layerParamsIn->divergence.damping
                }, velocityParams.base.deltaTime);
            velocityParams.fadeRate = NvFlowFloat4{
                layerParamsIn->velocity.fade,
                layerParamsIn->velocity.fade,
                layerParamsIn->velocity.fade,
                layerParamsIn->divergence.fade
            };

            mappedLayer[layerParamIdx].advectParams = velocityParams;
            mappedLayer[layerParamIdx].combustParams = combustParams;
            mappedLayer[layerParamIdx].velocityToDensityBlockScale = velocityToDensityBlockScale;
            mappedLayer[layerParamIdx].pad0 = 0.f;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        // if all layers disable advection, can do fade/downsample only
        if (!anyAdvectionEnabled)
        {
            if (anyDownsampleEnabled)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    AdvectionDownsampleCS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = layerTransient;
                    params.tableIn = in->velocity.sparseBuffer;
                    params.densityIn = in->density.textureTransient;
                    params.valueSampler = ptr->samplerLinear;
                    params.densityCoarseOut = out->densityCoarse.textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    AdvectionDownsampleCS_addPassCompute(in->context, &ptr->advectionDownsampleCS, gridDim, &params);
                }
            }
            else
            {
                NvFlowSparseTexture_passThrough(&out->densityCoarse, &in->densityCoarse);
            }
            if (anyForceFadeEnabled)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    AdvectionFadeVelocityCS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = layerTransient;
                    params.tableIn = in->velocity.sparseBuffer;
                    params.velocityIn = in->velocity.textureTransient;
                    params.velocityOut = out->velocity.textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    AdvectionFadeVelocityCS_addPassCompute(in->context, &ptr->advectionFadeVelocityCS, gridDim, &params);
                }
            }
            else
            {
                NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);
            }
            return;
        }

        // velocity predict
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            AdvectionVelocity1CS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->velocity.sparseBuffer;
            params.velocityIn = in->velocity.textureTransient;
            params.valueSampler = ptr->samplerLinear;
            params.velocityOut = velocityPredict.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            AdvectionVelocity1CS_addPassCompute(in->context, &ptr->advectionVelocity1CS, gridDim, &params);
        }

        // velocity correct
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            AdvectionVelocity2CS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->velocity.sparseBuffer;
            params.velocityIn = in->velocity.textureTransient;
            params.predictIn = velocityPredict.textureTransient;
            params.densityIn = in->density.textureTransient;
            params.valueSampler = ptr->samplerLinear;
            params.velocityOut = out->velocity.textureTransient;
            params.densityCoarseOut = out->densityCoarse.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            AdvectionVelocity2CS_addPassCompute(in->context, &ptr->advectionVelocity2CS, gridDim, &params);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowAdvectionCombustionVelocity, AdvectionCombustionVelocity)

namespace
{
    struct AdvectionCombustionDensity
    {
        NvFlowContextInterface contextInterface = {};

        AdvectionDensity1CS_Pipeline advectionDensity1CS = {};
        AdvectionDensity2CS_Pipeline advectionDensity2CS = {};
        AdvectionFadeDensityCS_Pipeline advectionFadeDensityCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    AdvectionCombustionDensity* AdvectionCombustionDensity_create(const NvFlowOpInterface* opInterface, const NvFlowAdvectionCombustionDensityPinsIn* in, NvFlowAdvectionCombustionDensityPinsOut* out)
    {
        auto ptr = new AdvectionCombustionDensity();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        AdvectionDensity1CS_init(&ptr->contextInterface, in->context, &ptr->advectionDensity1CS);
        AdvectionDensity2CS_init(&ptr->contextInterface, in->context, &ptr->advectionDensity2CS);
        AdvectionFadeDensityCS_init(&ptr->contextInterface, in->context, &ptr->advectionFadeDensityCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(AdvectionDensityCS_LayerParams));

        return ptr;
    }

    void AdvectionCombustionDensity_destroy(AdvectionCombustionDensity* ptr, const NvFlowAdvectionCombustionDensityPinsIn* in, NvFlowAdvectionCombustionDensityPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        AdvectionDensity1CS_destroy(in->context, &ptr->advectionDensity1CS);
        AdvectionDensity2CS_destroy(in->context, &ptr->advectionDensity2CS);
        AdvectionFadeDensityCS_destroy(in->context, &ptr->advectionFadeDensityCS);

        delete ptr;
    }

    void AdvectionCombustionDensity_execute(AdvectionCombustionDensity* ptr, const NvFlowAdvectionCombustionDensityPinsIn* in, NvFlowAdvectionCombustionDensityPinsOut* out)
    {
        NvFlowSparseLevelParams* densityLevelParams = &in->density.sparseParams.levels[in->density.levelIdx];
        NvFlowSparseLevelParams* velocityLevelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        if (densityLevelParams->numLocations == 0u)
        {
            NvFlowSparseTexture_passThrough(&out->density, &in->density);
            NvFlowSparseTexture_passThrough(&out->voxelWeight, &in->voxelWeight);
            return;
        }

        // for now, always pass voxelWeight through
        NvFlowSparseTexture_passThrough(&out->voxelWeight, &in->voxelWeight);

        NvFlowUint numLayers = in->density.sparseParams.layerCount;

        // if all layers are disabled, can do passthrough
        NvFlowBool32 anyAdvectionEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyForceFadeEnabled = NV_FLOW_FALSE;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];
            if (layerParamsIn->enabled && !in->density.sparseParams.layers[layerParamIdx].forceDisableCoreSimulation)
            {
                anyAdvectionEnabled = NV_FLOW_TRUE;
            }
            if (layerParamsIn->forceFadeEnabled)
            {
                anyForceFadeEnabled = NV_FLOW_TRUE;
            }
        }
        if (!anyAdvectionEnabled && !anyForceFadeEnabled)
        {
            NvFlowSparseTexture_passThrough(&out->density, &in->density);
            return;
        }

        /// compute simulation parameters

        NvFlowFloat3 densityBlockDimf = {
            float(densityLevelParams->blockDimLessOne.x + 1u),
            float(densityLevelParams->blockDimLessOne.y + 1u),
            float(densityLevelParams->blockDimLessOne.z + 1u)
        };
        NvFlowFloat3 velocityBlockDimf = {
            float(velocityLevelParams->blockDimLessOne.x + 1u),
            float(velocityLevelParams->blockDimLessOne.y + 1u),
            float(velocityLevelParams->blockDimLessOne.z + 1u)
        };
        float densityMaxDisplacement = fminf(fminf(densityBlockDimf.x, densityBlockDimf.y), densityBlockDimf.z) - 1.f;
        float velocityMaxDisplacement = fminf(fminf(velocityBlockDimf.x, velocityBlockDimf.y), velocityBlockDimf.z) - 1.f;

        NvFlowFloat3 velocityToDensityBlockScale = {
            densityBlockDimf.x / velocityBlockDimf.x,
            densityBlockDimf.y / velocityBlockDimf.y,
            densityBlockDimf.z / velocityBlockDimf.z,
        };

        NvFlowSparseTexture densityPredict = in->densityTemp;
        if (!densityPredict.textureTransient)
        {
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &densityPredict, &in->density);
        }
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->density, &in->density);

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, densityLevelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mappedGlobal = (AdvectionDensityCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(AdvectionDensityCS_GlobalParams));

            mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mappedGlobal->pad1 = 0u;
            mappedGlobal->pad2 = 0u;
            mappedGlobal->pad3 = 0u;
            mappedGlobal->table = *densityLevelParams;
            mappedGlobal->tableVelocity = *velocityLevelParams;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        auto mappedLayer = (AdvectionDensityCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(AdvectionDensityCS_LayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            AdvectionCombustionParams combustParams = {};
            advection_makeCombustParams(&combustParams, layerParamsIn);

            NvFlowFloat3 blockSizeWorld = in->density.sparseParams.layers[layerParamIdx].blockSizeWorld;

            MacCormackParams densityParams = {};
            densityParams.base.valueToVelocityBlockScale.x = velocityBlockDimf.x / densityBlockDimf.x;
            densityParams.base.valueToVelocityBlockScale.y = velocityBlockDimf.y / densityBlockDimf.y;
            densityParams.base.valueToVelocityBlockScale.z = velocityBlockDimf.z / densityBlockDimf.z;
            densityParams.base.globalFetch = layerParamsIn->globalFetch;
            densityParams.base.cellSizeInv.x = 1.f / (blockSizeWorld.x / densityBlockDimf.x);
            densityParams.base.cellSizeInv.y = 1.f / (blockSizeWorld.y / densityBlockDimf.y);
            densityParams.base.cellSizeInv.z = 1.f / (blockSizeWorld.z / densityBlockDimf.z);
            densityParams.base.deltaTime = in->deltaTime;
            densityParams.base.maxDisplacement.x = densityMaxDisplacement;
            densityParams.base.maxDisplacement.y = densityMaxDisplacement;
            densityParams.base.maxDisplacement.z = densityMaxDisplacement;
            densityParams.blendThreshold = NvFlowFloat4{
                layerParamsIn->temperature.secondOrderBlendThreshold,
                layerParamsIn->fuel.secondOrderBlendThreshold,
                layerParamsIn->burn.secondOrderBlendThreshold,
                layerParamsIn->smoke.secondOrderBlendThreshold
            };
            densityParams.blendFactor = NvFlowFloat4{
                layerParamsIn->temperature.secondOrderBlendFactor,
                layerParamsIn->fuel.secondOrderBlendFactor,
                layerParamsIn->burn.secondOrderBlendFactor,
                layerParamsIn->smoke.secondOrderBlendFactor
            };
            densityParams.dampingRate = dampingRate(NvFlowFloat4{
                layerParamsIn->temperature.damping,
                layerParamsIn->fuel.damping,
                layerParamsIn->burn.damping,
                layerParamsIn->smoke.damping
                }, densityParams.base.deltaTime);
            densityParams.fadeRate = NvFlowFloat4{
                layerParamsIn->temperature.fade,
                layerParamsIn->fuel.fade,
                layerParamsIn->burn.fade,
                layerParamsIn->smoke.fade
            };

            mappedLayer[layerParamIdx].advectParams = densityParams;
            mappedLayer[layerParamIdx].combustParams = combustParams;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        // if all layers disable advection, can do fade only
        if (!anyAdvectionEnabled)
        {
            if (anyForceFadeEnabled)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    AdvectionFadeDensityCS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = layerTransient;
                    params.tableIn = in->density.sparseBuffer;
                    params.densityIn = in->density.textureTransient;
                    params.densityOut = out->density.textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (densityLevelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    AdvectionFadeDensityCS_addPassCompute(in->context, &ptr->advectionFadeDensityCS, gridDim, &params);
                }
            }
            else
            {
                NvFlowSparseTexture_passThrough(&out->density, &in->density);
            }
            return;
        }

        // density predict
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            AdvectionDensity1CS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->density.sparseBuffer;
            params.densityIn = in->density.textureTransient;
            params.velocityIn = in->velocity.textureTransient;
            params.valueSampler = ptr->samplerLinear;
            params.densityOut = densityPredict.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (densityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            AdvectionDensity1CS_addPassCompute(in->context, &ptr->advectionDensity1CS, gridDim, &params);
        }

        // density correct
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            AdvectionDensity2CS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->density.sparseBuffer;
            params.densityIn = in->density.textureTransient;
            params.predictIn = densityPredict.textureTransient;
            params.velocityIn = in->velocity.textureTransient;
            params.valueSampler = ptr->samplerLinear;
            params.densityOut = out->density.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (densityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            AdvectionDensity2CS_addPassCompute(in->context, &ptr->advectionDensity2CS, gridDim, &params);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowAdvectionCombustionDensity, AdvectionCombustionDensity)
