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

#include "shaders/RayMarchBlockCS.hlsl.h"
#include "shaders/RayMarchBlockIsosurfaceCS.hlsl.h"
#include "shaders/RayMarchUpdateColormapCS.hlsl.h"
#include "shaders/RayMarchCopyTextureCS.hlsl.h"

namespace
{
    struct RayMarchUpdateColormap
    {
        NvFlowContextInterface contextInterface = {};

        RayMarchUpdateColormapCS_Pipeline rayMarchUpdateColormapCS = {};

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
        NvFlowUploadBuffer pointBuffer = {};
    };

    RayMarchUpdateColormap* RayMarchUpdateColormap_create(const NvFlowOpInterface* opInterface, const NvFlowRayMarchUpdateColormapPinsIn* in, NvFlowRayMarchUpdateColormapPinsOut* out)
    {
        auto ptr = new RayMarchUpdateColormap();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        RayMarchUpdateColormapCS_init(&ptr->contextInterface, in->context, &ptr->rayMarchUpdateColormapCS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(RayMarchUpdateColormapCS_LayerParams));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->pointBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(RayMarchUpdateColormapCS_PointParams));

        return ptr;
    }

    void RayMarchUpdateColormap_destroy(RayMarchUpdateColormap* ptr, const NvFlowRayMarchUpdateColormapPinsIn* in, NvFlowRayMarchUpdateColormapPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->pointBuffer);

        RayMarchUpdateColormapCS_destroy(in->context, &ptr->rayMarchUpdateColormapCS);

        delete ptr;
    }

    static const float colormapDefault_x[2u] = { 0.f, 1.f };
    static const NvFlowFloat4 colormapDefault_rgba[2u] = { {0.f, 0.f, 0.f, 0.f}, {1.f, 1.f, 1.f, 1.f} };

    void RayMarchUpdateColormap_execute(RayMarchUpdateColormap* ptr, const NvFlowRayMarchUpdateColormapPinsIn* in, NvFlowRayMarchUpdateColormapPinsOut* out)
    {
        // for now, take the maximum resolution to make texture sampling simpler
        NvFlowUint maxResolution = 1u;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            NvFlowUint cmpResolution = in->params[layerParamIdx]->resolution;
            if (cmpResolution > maxResolution)
            {
                maxResolution = cmpResolution;
            }
        }

        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        texDesc.format = eNvFlowFormat_r16g16b16a16_float;
        texDesc.width = maxResolution;
        texDesc.height = (NvFlowUint)in->paramCount;
        texDesc.depth = 1u;
        texDesc.mipLevels = 1u;
        // make sure texture is a valid dimension
        if (texDesc.height == 0u)
        {
            texDesc.height = 1u;
        }

        out->colormap = ptr->contextInterface.getTextureTransient(in->context, &texDesc);

        // count total points
        NvFlowUint64 totalPointCount = 0u;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto paramsIn = in->params[layerParamIdx];

            NvFlowUint64 numPoints = (paramsIn->xPointCount < paramsIn->rgbaPointCount ? paramsIn->xPointCount : paramsIn->rgbaPointCount);
            if (numPoints == 0u)
            {
                numPoints = 2u;
            }

            totalPointCount += numPoints;
        }

        auto mappedGlobal = (RayMarchUpdateColormapCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(RayMarchUpdateColormapCS_GlobalParams));
        auto mappedLayer = (RayMarchUpdateColormapCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, in->paramCount * sizeof(RayMarchUpdateColormapCS_LayerParams));
        auto mappedPts = (RayMarchUpdateColormapCS_PointParams*)NvFlowUploadBuffer_map(in->context, &ptr->pointBuffer, totalPointCount * sizeof(RayMarchUpdateColormapCS_PointParams));

        mappedGlobal->totalPts = (NvFlowUint)totalPointCount;
        mappedGlobal->numLayers = (NvFlowUint)in->paramCount;
        mappedGlobal->dim = maxResolution;
        mappedGlobal->dimInv = 1.f / float(maxResolution);

        NvFlowUint64 pointBaseOffset = 0u;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto paramsIn = in->params[layerParamIdx];

            NvFlowUint64 numPoints = (paramsIn->xPointCount < paramsIn->rgbaPointCount ? paramsIn->xPointCount : paramsIn->rgbaPointCount);
            const float* pointsX = paramsIn->xPoints;
            const NvFlowFloat4* pointsRgba = paramsIn->rgbaPoints;

            if (numPoints == 0u)
            {
                numPoints = 2u;
                pointsX = colormapDefault_x;
                pointsRgba = colormapDefault_rgba;
            }

            mappedLayer[layerParamIdx].numPts = (NvFlowUint)numPoints;
            mappedLayer[layerParamIdx].beginPtIdx = (NvFlowUint)pointBaseOffset;
            mappedLayer[layerParamIdx].endPtIdx = (NvFlowUint)(pointBaseOffset + numPoints);
            mappedLayer[layerParamIdx].layerParamIdx = (NvFlowUint)layerParamIdx;
            mappedLayer[layerParamIdx].colorScale = paramsIn->colorScale;
            mappedLayer[layerParamIdx].pad1 = 0.f;
            mappedLayer[layerParamIdx].pad2 = 0.f;
            mappedLayer[layerParamIdx].pad3 = 0.f;

            for (NvFlowUint idx = 0u; idx < numPoints; idx++)
            {
                float colorScale = idx < paramsIn->colorScalePointCount ? paramsIn->colorScalePoints[idx] : 1.f;
                mappedPts[idx + pointBaseOffset].x = pointsX[idx];
                mappedPts[idx + pointBaseOffset].pad1 = 0.f;
                mappedPts[idx + pointBaseOffset].pad2 = 0.f;
                mappedPts[idx + pointBaseOffset].pad3 = 0.f;
                mappedPts[idx + pointBaseOffset].color.x = colorScale * pointsRgba[idx].x;
                mappedPts[idx + pointBaseOffset].color.y = colorScale * pointsRgba[idx].y;
                mappedPts[idx + pointBaseOffset].color.z = colorScale * pointsRgba[idx].z;
                mappedPts[idx + pointBaseOffset].color.w = pointsRgba[idx].w;
            }

            pointBaseOffset += numPoints;
        }

        NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);
        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);
        NvFlowBufferTransient* pointTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->pointBuffer);

        {
            RayMarchUpdateColormapCS_PassParams passParams = {};
            passParams.globalParamsIn = globalTransient;
            passParams.layerParamsIn = layerTransient;
            passParams.pointParamsIn = pointTransient;
            passParams.gColormapOut = out->colormap;

            NvFlowUint3 gridDim = {};
            gridDim.x = (maxResolution + 127u) / 128u;
            gridDim.y = in->paramCount == 0u ? 1u : (NvFlowUint)in->paramCount;
            gridDim.z = 1u;

            RayMarchUpdateColormapCS_addPassCompute(in->context, &ptr->rayMarchUpdateColormapCS, gridDim, &passParams);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowRayMarchUpdateColormap, RayMarchUpdateColormap)

namespace
{
    struct RayMarch
    {
        NvFlowContextInterface contextInterface = {};

        RayMarchBlockCS_Pipeline rayMarchBlockCS = {};

        NvFlowSampler* samplerLinear = nullptr;
        NvFlowSampler* samplerClamp = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    RayMarch* RayMarch_create(const NvFlowOpInterface* opInterface, const NvFlowRayMarchPinsIn* in, NvFlowRayMarchPinsOut* out)
    {
        auto ptr = new RayMarch();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        RayMarchBlockCS_init(&ptr->contextInterface, in->context, &ptr->rayMarchBlockCS);

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
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowRayMarchLayerShaderParams));

        return ptr;
    }

    void RayMarch_destroy(RayMarch* ptr, const NvFlowRayMarchPinsIn* in, NvFlowRayMarchPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);
        ptr->contextInterface.destroySampler(in->context, ptr->samplerClamp);

        RayMarchBlockCS_destroy(in->context, &ptr->rayMarchBlockCS);

        delete ptr;
    }

    void RayMarch_execute(RayMarch* ptr, const NvFlowRayMarchPinsIn* in, NvFlowRayMarchPinsOut* out)
    {
        const NvFlowSparseLevelParams* levelParamsVelocity = &in->velocity.sparseParams.levels[in->velocity.levelIdx];
        const NvFlowSparseLevelParams* levelParamsDensity = &in->density.sparseParams.levels[in->density.levelIdx];

        using namespace NvFlowMath;

        // Skip math if no locations active
        if (levelParamsDensity->numLocations == 0u)
        {
            *in->target->pSceneColorOut = in->target->sceneColorIn;
            return;
        }

        NvFlowUint numLayers = in->density.sparseParams.layerCount;

        auto mappedGlobal = (NvFlowRayMarchShaderParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(NvFlowRayMarchShaderParams));
        auto mappedLayer = (NvFlowRayMarchLayerShaderParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(NvFlowRayMarchLayerShaderParams));

        NvFlowRayMarchShaderParams_populate(
            mappedGlobal,
            in->velocity.levelIdx,
            in->density.levelIdx,
            &in->density.sparseParams,
            in->target->view,
            in->target->projection,
            in->target->projectionJittered,
            in->target->textureWidth,
            in->target->textureHeight,
            in->target->sceneDepthWidth,
            in->target->sceneDepthHeight,
            in->compositeColorScale
        );

        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            const NvFlowSparseLayerParams* layerParams = &in->density.sparseParams.layers[layerParamIdx];
            const NvFlowRayMarchParams* rayMarchParams = in->params[layerParamIdx];

            NvFlowRayMarchLayerShaderParams_populate(
                &mappedLayer[layerParamIdx],
                in->velocity.levelIdx,
                in->density.levelIdx,
                layerParamIdx,
                &in->density.sparseParams,
                rayMarchParams
            );
        }

        NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);
        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        texDesc.format = in->target->sceneColorFormat;
        texDesc.width = in->target->textureWidth;
        texDesc.height = in->target->textureHeight;
        texDesc.depth = 1u;
        texDesc.mipLevels = 1u;

        *in->target->pSceneColorOut = ptr->contextInterface.getTextureTransient(in->context, &texDesc);

        RayMarchBlockCS_PassParams passParams = {};
        passParams.globalParamsIn = globalTransient;
        passParams.layerParamsIn = layerTransient;
        passParams.tableIn = in->density.sparseBuffer;
        passParams.velocityIn = in->velocity.textureTransient;
        passParams.velocitySampler = ptr->samplerLinear;
        passParams.densityIn = in->density.textureTransient;
        passParams.densitySampler = ptr->samplerLinear;
        passParams.colormapIn = in->colormap;
        passParams.colormapSampler = ptr->samplerClamp;
        passParams.depthIn = in->target->sceneDepthIn;
        passParams.colorIn = in->target->sceneColorIn;
        passParams.colorOut = *in->target->pSceneColorOut;

        NvFlowUint3 gridDim = {};
        gridDim.x = (in->target->textureWidth + 7u) / 8u;
        gridDim.y = (in->target->textureHeight + 3u) / 4u;
        gridDim.z = 1u;

        RayMarchBlockCS_addPassCompute(in->context, &ptr->rayMarchBlockCS, gridDim, &passParams);
    }
}

NV_FLOW_OP_IMPL(NvFlowRayMarch, RayMarch)

namespace
{
    struct RayMarchIsosurface
    {
        NvFlowContextInterface contextInterface = {};

        RayMarchBlockIsosurfaceCS_Pipeline rayMarchBlockIsosurfaceCS = {};

        NvFlowSampler* samplerLinear = nullptr;
        NvFlowSampler* samplerClamp = nullptr;

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    RayMarchIsosurface* RayMarchIsosurface_create(const NvFlowOpInterface* opInterface, const NvFlowRayMarchIsosurfacePinsIn* in, NvFlowRayMarchIsosurfacePinsOut* out)
    {
        auto ptr = new RayMarchIsosurface();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        RayMarchBlockIsosurfaceCS_init(&ptr->contextInterface, in->context, &ptr->rayMarchBlockIsosurfaceCS);

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
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowRayMarchIsosurfaceLayerShaderParams));

        return ptr;
    }

    void RayMarchIsosurface_destroy(RayMarchIsosurface* ptr, const NvFlowRayMarchIsosurfacePinsIn* in, NvFlowRayMarchIsosurfacePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);
        ptr->contextInterface.destroySampler(in->context, ptr->samplerClamp);

        RayMarchBlockIsosurfaceCS_destroy(in->context, &ptr->rayMarchBlockIsosurfaceCS);

        delete ptr;
    }

    void RayMarchIsosurface_execute(RayMarchIsosurface* ptr, const NvFlowRayMarchIsosurfacePinsIn* in, NvFlowRayMarchIsosurfacePinsOut* out)
    {
        const NvFlowSparseLevelParams* levelParams = &in->density.sparseParams.levels[in->density.levelIdx];

        using namespace NvFlowMath;

        // Skip math if no locations active
        if (levelParams->numLocations == 0u)
        {
            *in->target->pSceneColorOut = in->target->sceneColorIn;
            return;
        }

        NvFlowFloat4x4 projectionInv = matrixInverse(*in->target->projection);
        NvFlowFloat4x4 projectionJitteredInv = matrixInverse(*in->target->projectionJittered);
        NvFlowFloat4x4 viewInv = matrixInverse(*in->target->view);

        FrustumRays frustumRays = {};
        computeFrustumRays(&frustumRays, viewInv, projectionInv);

        NvFlowUint numLayers = in->density.sparseParams.layerCount;

        auto mappedGlobal = (NvFlowRayMarchShaderParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(NvFlowRayMarchShaderParams));
        auto mappedLayer = (NvFlowRayMarchIsosurfaceLayerShaderParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(NvFlowRayMarchIsosurfaceLayerShaderParams));

        NvFlowRayMarchShaderParams_populate(
            mappedGlobal,
            in->density.levelIdx,
            in->density.levelIdx,
            &in->density.sparseParams,
            in->target->view,
            in->target->projection,
            in->target->projectionJittered,
            in->target->textureWidth,
            in->target->textureHeight,
            in->target->sceneDepthWidth,
            in->target->sceneDepthHeight,
            in->compositeColorScale
        );

        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            const NvFlowSparseLayerParams* layerParams = &in->density.sparseParams.layers[layerParamIdx];
            const NvFlowRayMarchIsosurfaceParams* rayMarchParams = in->params[layerParamIdx];

            NvFlowRayMarchIsosurfaceLayerShaderParams_populate(
                &mappedLayer[layerParamIdx],
                in->density.levelIdx,
                layerParamIdx,
                &in->density.sparseParams,
                rayMarchParams
            );
        }

        NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);
        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        texDesc.format = in->target->sceneColorFormat;
        texDesc.width = in->target->textureWidth;
        texDesc.height = in->target->textureHeight;
        texDesc.depth = 1u;
        texDesc.mipLevels = 1u;

        *in->target->pSceneColorOut = ptr->contextInterface.getTextureTransient(in->context, &texDesc);

        RayMarchBlockIsosurfaceCS_PassParams passParams = {};
        passParams.globalParamsIn = globalTransient;
        passParams.layerParamsIn = layerTransient;
        passParams.tableIn = in->density.sparseBuffer;
        passParams.densityIn = in->density.textureTransient;
        passParams.densitySampler = ptr->samplerLinear;
        passParams.depthIn = in->target->sceneDepthIn;
        passParams.colorIn = in->target->sceneColorIn;
        passParams.colorOut = *in->target->pSceneColorOut;

        NvFlowUint3 gridDim = {};
        gridDim.x = (in->target->textureWidth + 7u) / 8u;
        gridDim.y = (in->target->textureHeight + 3u) / 4u;
        gridDim.z = 1u;

        RayMarchBlockIsosurfaceCS_addPassCompute(in->context, &ptr->rayMarchBlockIsosurfaceCS, gridDim, &passParams);
    }
}

NV_FLOW_OP_IMPL(NvFlowRayMarchIsosurface, RayMarchIsosurface)

namespace
{
    struct RayMarchCopyTexture
    {
        NvFlowContextInterface contextInterface = {};

        RayMarchCopyTextureCS_Pipeline rayMarchCopyTextureCS = {};

        NvFlowUploadBuffer globalBuffer = {};
    };

    RayMarchCopyTexture* RayMarchCopyTexture_create(const NvFlowOpInterface* opInterface, const NvFlowRayMarchCopyTexturePinsIn* in, NvFlowRayMarchCopyTexturePinsOut* out)
    {
        auto ptr = new RayMarchCopyTexture();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        RayMarchCopyTextureCS_init(&ptr->contextInterface, in->context, &ptr->rayMarchCopyTextureCS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        return ptr;
    }

    void RayMarchCopyTexture_destroy(RayMarchCopyTexture* ptr, const NvFlowRayMarchCopyTexturePinsIn* in, NvFlowRayMarchCopyTexturePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);

        RayMarchCopyTextureCS_destroy(in->context, &ptr->rayMarchCopyTextureCS);

        delete ptr;
    }

    void RayMarchCopyTexture_execute(RayMarchCopyTexture* ptr, const NvFlowRayMarchCopyTexturePinsIn* in, NvFlowRayMarchCopyTexturePinsOut* out)
    {
        if (in->width == 0u || in->height == 0u)
        {
            out->texture = nullptr;
            return;
        }

        auto mapped = (RayMarchCopyTextureCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(RayMarchCopyTextureCS_Params));

        mapped->pixelMini.x = 0;
        mapped->pixelMini.y = 0;
        mapped->pixelMaxi.x = int(in->width);
        mapped->pixelMaxi.y = int(in->height);

        NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        texDesc.format = in->format;
        texDesc.width = in->width;
        texDesc.height = in->height;
        texDesc.depth = 1u;
        texDesc.mipLevels = 1u;
        out->texture = ptr->contextInterface.getTextureTransient(in->context, &texDesc);

        RayMarchCopyTextureCS_PassParams passParams = {};
        passParams.paramsIn = globalTransient;
        passParams.colorIn = in->texture;
        passParams.colorOut = out->texture;

        NvFlowUint3 gridDim = {};
        gridDim.x = (in->width + 7u) / 8u;
        gridDim.y = (in->height + 7u) / 8u;
        gridDim.z = 1u;

        RayMarchCopyTextureCS_addPassCompute(in->context, &ptr->rayMarchCopyTextureCS, gridDim, &passParams);
    }
}

NV_FLOW_OP_IMPL(NvFlowRayMarchCopyTexture, RayMarchCopyTexture)
