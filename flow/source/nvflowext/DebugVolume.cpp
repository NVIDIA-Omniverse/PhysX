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

#include "shaders/DebugVolumeParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"

#include "NvFlowExt.h"

#include "shaders/DebugVolumeCS.hlsl.h"
#include "shaders/DebugVolume2CS.hlsl.h"

namespace
{
    struct DebugVolume
    {
        NvFlowContextInterface contextInterface = {};

        DebugVolumeCS_Pipeline debugVolumeCS = {};
        DebugVolume2CS_Pipeline debugVolume2CS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    DebugVolume* DebugVolume_create(const NvFlowOpInterface* opInterface, const NvFlowDebugVolumePinsIn* in, NvFlowDebugVolumePinsOut* out)
    {
        auto ptr = new DebugVolume();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        DebugVolumeCS_init(&ptr->contextInterface, in->context, &ptr->debugVolumeCS);
        DebugVolume2CS_init(&ptr->contextInterface, in->context, &ptr->debugVolume2CS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(DebugVolumeCS_LayerParams));

        return ptr;
    }

    void DebugVolume_destroy(DebugVolume* ptr, const NvFlowDebugVolumePinsIn* in, NvFlowDebugVolumePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        DebugVolumeCS_destroy(in->context, &ptr->debugVolumeCS);
        DebugVolume2CS_destroy(in->context, &ptr->debugVolume2CS);

        delete ptr;
    }

    void DebugVolume_execute(DebugVolume* ptr, const NvFlowDebugVolumePinsIn* in, NvFlowDebugVolumePinsOut* out)
    {
        NvFlowSparseLevelParams* densityLevelParams = &in->densityShadow.sparseParams.levels[in->densityShadow.levelIdx];
        NvFlowSparseLevelParams* velocityLevelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        NvFlowUint numLayers = in->densityShadow.sparseParams.layerCount;

        NvFlowBool32 canSkip = NV_FLOW_TRUE;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            if (in->params[layerParamIdx]->enabled && (in->params[layerParamIdx]->applyPreShadow == in->isPreShadow))
            {
                if (in->params[layerParamIdx]->enableSpeedAsTemperature ||
                    in->params[layerParamIdx]->enableVelocityAsDensity ||
                    in->params[layerParamIdx]->enableDivergenceAsSmoke ||
                    in->params[layerParamIdx]->outputScaleBySmoke ||
                    in->params[layerParamIdx]->outputTemperatureScale != 1.f ||
                    in->params[layerParamIdx]->outputFuelScale != 1.f ||
                    in->params[layerParamIdx]->outputBurnScale != 1.f ||
                    in->params[layerParamIdx]->outputSmokeScale != 1.f ||
                    in->params[layerParamIdx]->outputTemperatureOffset != 0.f ||
                    in->params[layerParamIdx]->outputFuelOffset != 0.f ||
                    in->params[layerParamIdx]->outputBurnOffset != 0.f ||
                    in->params[layerParamIdx]->outputSmokeOffset != 0.f ||
                    in->params[layerParamIdx]->slicePlaneThickness != 0.f)
                {
                    canSkip = NV_FLOW_FALSE;
                    break;
                }
            }
        }

        if (densityLevelParams->numLocations == 0u || canSkip)
        {
            NvFlowSparseTexture_passThrough(&out->densityShadow, &in->densityShadow);
            NvFlowSparseTexture_passThrough(&out->coarseDensityShadow, &in->coarseDensityShadow);
            return;
        }

        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->densityShadow, &in->densityShadow);
        if (in->isPreShadow)
        {
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->coarseDensityShadow, &in->coarseDensityShadow);
        }
        else
        {
            NvFlowSparseTexture_passThrough(&out->coarseDensityShadow, &in->coarseDensityShadow);
        }

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, densityLevelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mappedGlobal = (DebugVolumeCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(DebugVolumeCS_GlobalParams));

            mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mappedGlobal->pad1 = 0u;
            mappedGlobal->pad2 = 0u;
            mappedGlobal->pad3 = 0u;
            mappedGlobal->table = *densityLevelParams;
            mappedGlobal->velocityTable = *velocityLevelParams;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        auto mappedLayer = (DebugVolumeCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(DebugVolumeCS_LayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            NvFlowFloat3 blockSizeWorld = { 0.5f, 0.5f, 0.5f };
            if (layerParamIdx < in->velocity.sparseParams.layerCount)
            {
                blockSizeWorld = in->velocity.sparseParams.layers[layerParamIdx].blockSizeWorld;
            }
            NvFlowFloat3 vidxToWorld = {
                blockSizeWorld.x / float(densityLevelParams->blockDimLessOne.x + 1u),
                blockSizeWorld.y / float(densityLevelParams->blockDimLessOne.y + 1u),
                blockSizeWorld.z / float(densityLevelParams->blockDimLessOne.z + 1u)
            };

            mappedLayer[layerParamIdx].velocityScale = in->params[layerParamIdx]->velocityScale;
            mappedLayer[layerParamIdx].enableDivergenceAsSmoke = in->params[layerParamIdx]->enableDivergenceAsSmoke;

            mappedLayer[layerParamIdx].enabled =
                in->params[layerParamIdx]->enabled && (in->params[layerParamIdx]->applyPreShadow == in->isPreShadow);
            mappedLayer[layerParamIdx].enableSpeedAsTemperature = in->params[layerParamIdx]->enableSpeedAsTemperature;
            mappedLayer[layerParamIdx].enableVelocityAsDensity = in->params[layerParamIdx]->enableVelocityAsDensity;
            mappedLayer[layerParamIdx].outputScaleBySmoke = in->params[layerParamIdx]->outputScaleBySmoke;

            mappedLayer[layerParamIdx].outputTemperatureScale = in->params[layerParamIdx]->outputTemperatureScale;
            mappedLayer[layerParamIdx].outputFuelScale = in->params[layerParamIdx]->outputFuelScale;
            mappedLayer[layerParamIdx].outputBurnScale = in->params[layerParamIdx]->outputBurnScale;
            mappedLayer[layerParamIdx].outputSmokeScale = in->params[layerParamIdx]->outputSmokeScale;
            mappedLayer[layerParamIdx].outputTemperatureOffset = in->params[layerParamIdx]->outputTemperatureOffset;
            mappedLayer[layerParamIdx].outputFuelOffset = in->params[layerParamIdx]->outputFuelOffset;
            mappedLayer[layerParamIdx].outputBurnOffset = in->params[layerParamIdx]->outputBurnOffset;
            mappedLayer[layerParamIdx].outputSmokeOffset = in->params[layerParamIdx]->outputSmokeOffset;

            mappedLayer[layerParamIdx].slicePlane = in->params[layerParamIdx]->slicePlane;
            mappedLayer[layerParamIdx].vidxToWorld = vidxToWorld;
            mappedLayer[layerParamIdx].slicePlaneThickness = in->params[layerParamIdx]->slicePlaneThickness;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            DebugVolumeCS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->densityShadow.sparseBuffer;
            params.densityShadowIn = in->densityShadow.textureTransient;
            params.velocityIn = in->velocity.textureTransient;
            params.samplerIn = ptr->samplerLinear;
            params.densityShadowOut = out->densityShadow.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (densityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            DebugVolumeCS_addPassCompute(in->context, &ptr->debugVolumeCS, gridDim, &params);
        }

        // for pre shadow, we need to downsample result so shadows look correct
        if (in->isPreShadow)
        {
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                DebugVolume2CS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.layerParamsIn = layerTransient;
                params.tableIn = in->densityShadow.sparseBuffer;
                params.densityShadowIn = out->densityShadow.textureTransient;;
                params.samplerIn = ptr->samplerLinear;
                params.coarseDensityShadowOut = out->coarseDensityShadow.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                DebugVolume2CS_addPassCompute(in->context, &ptr->debugVolume2CS, gridDim, &params);
            }
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowDebugVolume, DebugVolume)
