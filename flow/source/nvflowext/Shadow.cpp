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

#include "shaders/RayMarchParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"

#include "NvFlowExt.h"
#include "NvFlowRayMarchUtils.h"

#include "shaders/ShadowParams.h"
#include "shaders/ShadowUpdateCS.hlsl.h"
#include "shaders/ShadowCoarseUpdateCS.hlsl.h"
#include "shaders/ShadowCoarseApplyCS.hlsl.h"
#include "shaders/ShadowNullCS.hlsl.h"

namespace
{
    struct ShadowLayerParams
    {
        NvFlowFloat3 blockSizeWorld;
        NvFlowFloat3 cellSize;
        int layer;
        NvFlowBool32 isCoarse;
        const NvFlowShadowParams* shadowParams;
    };

    struct Shadow
    {
        NvFlowContextInterface contextInterface = {};

        ShadowUpdateCS_Pipeline shadowUpdateCS = {};
        ShadowCoarseUpdateCS_Pipeline shadowCoarseUpdateCS = {};
        ShadowCoarseApplyCS_Pipeline shadowCoarseApplyCS = {};
        ShadowNullCS_Pipeline shadowNullCS = {};

        NvFlowSampler* samplerLinear = nullptr;
        NvFlowSampler* samplerClamp = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};

        NvFlowArray<ShadowLayerParams> layerParams;
    };

    Shadow* Shadow_create(const NvFlowOpInterface* opInterface, const NvFlowShadowPinsIn* in, NvFlowShadowPinsOut* out)
    {
        auto ptr = new Shadow();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        ShadowUpdateCS_init(&ptr->contextInterface, in->context, &ptr->shadowUpdateCS);
        ShadowCoarseUpdateCS_init(&ptr->contextInterface, in->context, &ptr->shadowCoarseUpdateCS);
        ShadowCoarseApplyCS_init(&ptr->contextInterface, in->context, &ptr->shadowCoarseApplyCS);
        ShadowNullCS_init(&ptr->contextInterface, in->context, &ptr->shadowNullCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_clamp;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_clamp;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_clamp;

        ptr->samplerClamp = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowSelfShadowLayerShaderParams));

        return ptr;
    }

    void Shadow_destroy(Shadow* ptr, const NvFlowShadowPinsIn* in, NvFlowShadowPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);
        ptr->contextInterface.destroySampler(in->context, ptr->samplerClamp);

        ShadowUpdateCS_destroy(in->context, &ptr->shadowUpdateCS);
        ShadowCoarseUpdateCS_destroy(in->context, &ptr->shadowCoarseUpdateCS);
        ShadowCoarseApplyCS_destroy(in->context, &ptr->shadowCoarseApplyCS);
        ShadowNullCS_destroy(in->context, &ptr->shadowNullCS);

        delete ptr;
    }

    void updateConstants(
        NvFlowDispatchBatches* pBatches,
        NvFlowBufferTransient** pLayerBuffer,
        NvFlowContext* context,
        Shadow* ptr,
        const NvFlowSparseTexture* coarseDensity,
        const NvFlowSparseTexture* fineDensity,
        NvFlowBool32 isCoarse,
        const NvFlowShadowParams** params,
        NvFlowUint64 paramCount
    )
    {
        NvFlowUint numLayers = coarseDensity->sparseParams.layerCount;

        NvFlowSparseLevelParams* coarseDensityLevelParams = &coarseDensity->sparseParams.levels[coarseDensity->levelIdx];

        for (NvFlowUint64 batchIdx = 0u; batchIdx < (*pBatches).size; batchIdx++)
        {
            auto mappedGlobal = (NvFlowSelfShadowShaderParams*)NvFlowUploadBuffer_map(context, &ptr->globalBuffer, sizeof(NvFlowSelfShadowShaderParams));

            NvFlowSelfShadowShaderParams_populate(
                mappedGlobal,
                coarseDensity->levelIdx,
                fineDensity->levelIdx,
                &coarseDensity->sparseParams,
                (*pBatches)[batchIdx].blockIdxOffset
            );

            (*pBatches)[batchIdx].globalTransient = NvFlowUploadBuffer_unmap(context, &ptr->globalBuffer);
        }

        auto mappedLayer = (NvFlowSelfShadowLayerShaderParams*)NvFlowUploadBuffer_map(context, &ptr->layerBuffer, numLayers * sizeof(NvFlowSelfShadowLayerShaderParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < paramCount; layerParamIdx++)
        {
            const NvFlowShadowParams* shadowParams = params[layerParamIdx];

            NvFlowSelfShadowLayerShaderParams_populate(
                &mappedLayer[layerParamIdx],
                coarseDensity->levelIdx,
                fineDensity->levelIdx,
                (NvFlowUint)layerParamIdx,
                &coarseDensity->sparseParams,
                shadowParams
            );
        }

        *pLayerBuffer = NvFlowUploadBuffer_unmap(context, &ptr->layerBuffer);
    }

    void addPassesFine(
        NvFlowContext* context,
        Shadow* ptr,
        const NvFlowShadowParams** params,
        NvFlowUint64 paramCount,
        NvFlowTextureTransient* colormap,
        const NvFlowSparseTexture* densityIn,
        NvFlowSparseTexture* densityShadowOut
    )
    {
        NvFlowSparseLevelParams* levelParams = &densityIn->sparseParams.levels[densityIn->levelIdx];

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
        NvFlowBufferTransient* layerTransient = nullptr;
        updateConstants(
            &batches,
            &layerTransient,
            context,
            ptr,
            densityIn,
            densityIn,
            NV_FLOW_FALSE,
            params,
            paramCount
        );

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            ShadowUpdateCS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.gTable = densityIn->sparseBuffer;
            params.gInput = densityIn->textureTransient;
            params.gSampler = ptr->samplerLinear;
            params.gColormap = colormap;
            params.gColormapSampler = ptr->samplerClamp;
            params.densityOut = densityShadowOut->textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            ShadowUpdateCS_addPassCompute(context, &ptr->shadowUpdateCS, gridDim, &params);
        }
    }

    void Shadow_execute(Shadow* ptr, const NvFlowShadowPinsIn* in, NvFlowShadowPinsOut* out)
    {
        NvFlowSparseLevelParams* densityLevelParams = &in->density.sparseParams.levels[in->density.levelIdx];
        NvFlowSparseLevelParams* coarseDensityLevelParams = &in->coarseDensity.sparseParams.levels[in->coarseDensity.levelIdx];

        // check if disabled on all layers
        NvFlowBool32 canSkip = NV_FLOW_TRUE;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            if (in->params[layerParamIdx]->enabled)
            {
                canSkip = NV_FLOW_FALSE;
                break;
            }
        }
        if (densityLevelParams->numLocations == 0u || canSkip)
        {
            NvFlowSparseTexture_passThrough(&out->densityShadow, &in->density);
            return;
        }

        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->densityShadow, &in->density);

        if (densityLevelParams->numLocations == 0u)
        {
            ShadowNullCS_PassParams params = {};
            params.valueOut = out->densityShadow.textureTransient;

            NvFlowUint3 gridDim = { 0u, 0u, 0u };
            ShadowNullCS_addPassCompute(in->context, &ptr->shadowNullCS, gridDim, &params);

            return;
        }

        // use fine if any layer requests it
        NvFlowBool32 useFinePropagate = NV_FLOW_FALSE;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            if (!in->params[layerParamIdx]->coarsePropagate)
            {
                useFinePropagate = NV_FLOW_TRUE;
                break;
            }
        }

        if (useFinePropagate)
        {
            return addPassesFine(
                in->context,
                ptr,
                in->params,
                in->paramCount,
                in->colormap,
                &in->density,
                &out->densityShadow
            );
        }

        NvFlowSparseTexture alphaTexture = {};
        NvFlowSparseTexture_duplicateWithFormat(&ptr->contextInterface, in->context, &alphaTexture, &in->coarseDensity, eNvFlowFormat_r16_float);

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, densityLevelParams->numLocations);
        NvFlowBufferTransient* layerTransient = nullptr;
        updateConstants(
            &batches,
            &layerTransient,
            in->context,
            ptr,
            &in->coarseDensity,
            &in->density,
            NV_FLOW_TRUE,
            in->params,
            in->paramCount
        );

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            ShadowCoarseUpdateCS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.gTable = in->coarseDensity.sparseBuffer;
            params.gInput = in->coarseDensity.textureTransient;
            params.gSampler = ptr->samplerLinear;
            params.gColormap = in->colormap;
            params.gColormapSampler = ptr->samplerClamp;
            params.alphaOut = alphaTexture.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (coarseDensityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            ShadowCoarseUpdateCS_addPassCompute(in->context, &ptr->shadowCoarseUpdateCS, gridDim, &params);
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            ShadowCoarseApplyCS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.gTable = in->density.sparseBuffer;
            params.gInput = in->density.textureTransient;
            params.gSampler = ptr->samplerLinear;
            params.gColormap = in->colormap;
            params.gColormapSampler = ptr->samplerClamp;
            params.alphaIn = alphaTexture.textureTransient;
            params.densityOut = out->densityShadow.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (densityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            ShadowCoarseApplyCS_addPassCompute(in->context, &ptr->shadowCoarseApplyCS, gridDim, &params);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowShadow, Shadow)
