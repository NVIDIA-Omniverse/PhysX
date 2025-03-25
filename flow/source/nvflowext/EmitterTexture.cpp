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

#include <cstring>

#include "shaders/EmitterParams.h"

#include "EmitterCommon.h"
#include "shaders/CopyParams.h"

#include "shaders/EmitterTextureCS.hlsl.h"
#include "shaders/CopyFP32toFP16CS.hlsl.h"

#include "NvFlowArrayBuffer.h"

namespace
{
    enum Array
    {
        eArray_distances = 0,
        eArray_velocities = 1,
        eArray_divergences = 2,
        eArray_temperatures = 3,
        eArray_fuels = 4,
        eArray_burns = 5,
        eArray_smokes = 6,
        eArray_coupleRateVelocities = 7,
        eArray_coupleRateDivergences = 8,
        eArray_coupleRateTemperatures = 9,
        eArray_coupleRateFuels = 10,
        eArray_coupleRateBurns = 11,
        eArray_coupleRateSmokes = 12,

        eArray_count = 13
    };

    struct EmitterTextureInstance
    {
        NvFlowUint64 luid = 0llu;

        NvFlowUint instanceActiveCount = 0u;
        NvFlowUint instanceInactiveCount = 0u;

        NvFlowArrayBuffer arrayBuffer = {};
    };

    struct EmitterTexture
    {
        NvFlowContextInterface contextInterface = {};

        EmitterTextureCS_Pipeline emitterTextureCS = {};
        CopyFP32toFP16CS_Pipeline copyFP32toFP16CS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};

        NvFlowArrayPointer<EmitterTextureInstance*> instances;
    };

    NV_FLOW_INLINE NvFlowBuffer* EmitterTexture_createBuffer(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, void* userdata)
    {
        EmitterTexture* ptr = (EmitterTexture*)userdata;

        NvFlowBufferDesc descCopy = *desc;
        descCopy.usageFlags |= eNvFlowBufferUsage_rwStructuredBuffer;
        descCopy.sizeInBytes = (desc->sizeInBytes + 1u) / 2u;

        return ptr->contextInterface.createBuffer(context, memoryType, &descCopy);
    }

    NV_FLOW_INLINE void EmitterTexture_addPassCopyBuffer(NvFlowContext* context, const NvFlowPassCopyBufferParams* params, void* userdata)
    {
        EmitterTexture* ptr = (EmitterTexture*)userdata;

        NvFlowUint elementCount = (NvFlowUint)(params->numBytes / 4u);
        NvFlowUint threadCount = (elementCount + 1u) / 2u;

        NvFlowUint blockCount = (threadCount + 127u) / 128u;
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, blockCount);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (CopyFP32toFP16CS_Params*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(CopyFP32toFP16CS_Params));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->threadCount = threadCount;
            mapped->dstOffset = (NvFlowUint)(params->dstOffset / 4u);
            mapped->elementCount = elementCount;
            mapped->srcOffset = (NvFlowUint)(params->srcOffset / 4u);
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            CopyFP32toFP16CS_PassParams passParams = {};
            passParams.paramsIn = constantTransient;
            passParams.srcIn = params->src;
            passParams.dstOut = params->dst;

            NvFlowUint3 gridDim = {};
            gridDim.x = batches[batchIdx].blockCount;
            gridDim.y = 1u;
            gridDim.z = 1u;

            CopyFP32toFP16CS_addPassCompute(context, &ptr->copyFP32toFP16CS, gridDim, &passParams);
        }
    }

    void EmitterTextureInstance_init(NvFlowContext* context, EmitterTexture* ptr, EmitterTextureInstance* inst, NvFlowUint64 luid)
    {
        inst->luid = luid;
        NvFlowArrayBuffer_init_custom(
            &ptr->contextInterface,
            context,
            &inst->arrayBuffer,
            eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer,
            eNvFlowFormat_unknown,
            sizeof(float),
            EmitterTexture_createBuffer,
            EmitterTexture_addPassCopyBuffer,
            ptr
        );
        inst->instanceActiveCount = 0u;
        inst->instanceInactiveCount = 0u;
    }

    void EmitterTextureInstance_destroy(NvFlowContext* context, EmitterTexture* ptr, EmitterTextureInstance* inst)
    {
        inst->luid = 0llu;
        NvFlowArrayBuffer_destroy(context, &inst->arrayBuffer);
        inst->instanceActiveCount = 0u;
        inst->instanceInactiveCount = 0u;
    }

    EmitterTexture* EmitterTexture_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterTexturePinsIn* in, NvFlowEmitterTexturePinsOut* out)
    {
        auto ptr = new EmitterTexture();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EmitterTextureCS_init(&ptr->contextInterface, in->context, &ptr->emitterTextureCS);
        CopyFP32toFP16CS_init(&ptr->contextInterface, in->context, &ptr->copyFP32toFP16CS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        return ptr;
    }

    void EmitterTexture_destroy(EmitterTexture* ptr, const NvFlowEmitterTexturePinsIn* in, NvFlowEmitterTexturePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);

        for (NvFlowUint idx = 0u; idx < ptr->instances.size; idx++)
        {
            if (ptr->instances[idx])
            {
                EmitterTextureInstance_destroy(in->context, ptr, ptr->instances[idx]);
            }
        }
        ptr->instances.deletePointers();

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        EmitterTextureCS_destroy(in->context, &ptr->emitterTextureCS);
        CopyFP32toFP16CS_destroy(in->context, &ptr->copyFP32toFP16CS);

        delete ptr;
    }

    void EmitterTexture_executeSingleEmitter(
        EmitterTexture* ptr,
        const NvFlowEmitterTexturePinsIn* in,
        NvFlowEmitterTexturePinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        NvFlowSparseTexture* valueTemp,
        const NvFlowEmitterTextureParams* params,
        NvFlowBool32 isVelocity
    )
    {
        // resolve instance, done first to ensure proper instance retention
        EmitterTextureInstance* inst = nullptr;
        for (NvFlowUint instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            auto instanceTest = ptr->instances[instanceIdx];
            if (instanceTest->luid == params->luid)
            {
                inst = instanceTest;
                break;
            }
        }
        if (!inst)
        {
            inst = ptr->instances.allocateBackPointer();
            EmitterTextureInstance_init(in->context, ptr, inst, params->luid);
        }
        inst->instanceActiveCount++;

        if (!params->enabled)
        {
            return;
        }
        // No impact if zero active blocks
        if (levelParams->numLocations == 0u)
        {
            return;
        }
        // Apply post pressure if requested
        if (isVelocity)
        {
            if (in->isPostPressure && !params->applyPostPressure)
            {
                return;
            }
            if (!in->isPostPressure && params->applyPostPressure)
            {
                return;
            }
        }

        NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->value.sparseParams, params->layer, params->level);
        // early out if layer is not active
        if (layerParamIdx == ~0u)
        {
            return;
        }
        const NvFlowSparseLayerParams* layerParams = &in->value.sparseParams.layers[layerParamIdx];
        if (layerParams->forceDisableEmitters)
        {
            return;
        }

        // Early out if couple rates leave emitter having zero effect, only with defaults
        if (!isVelocity &&
            params->textureCoupleRateTemperatureCount == 0u &&
            params->textureCoupleRateFuelCount == 0u &&
            params->textureCoupleRateBurnCount == 0u &&
            params->textureCoupleRateSmokeCount == 0u &&
            params->coupleRateTemperature <= 0.f &&
            params->coupleRateFuel <= 0.f &&
            params->coupleRateBurn <= 0.f &&
            params->coupleRateSmoke <= 0.f)
        {
            return;
        }
        if (isVelocity &&
            params->textureCoupleRateVelocityCount == 0u &&
            params->textureCoupleRateDivergenceCount == 0u &&
            params->coupleRateVelocity <= 0.f &&
            params->coupleRateDivergence <= 0.f)
        {
            return;
        }

        using namespace NvFlowMath;

        NvFlowArrayBufferData arrayDatas[eArray_count] = { };
        arrayDatas[eArray_distances] = { params->textureDistances, params->textureDistanceCount, params->textureDistanceVersion };
        arrayDatas[eArray_velocities] = {params->textureVelocities, 3 * params->textureVelocityCount, params->textureVelocityVersion};
        arrayDatas[eArray_divergences] = { params->textureDivergences, params->textureDivergenceCount, params->textureDivergenceVersion };
        arrayDatas[eArray_temperatures] = { params->textureTemperatures, params->textureTemperatureCount, params->textureTemperatureVersion };
        arrayDatas[eArray_fuels] = { params->textureFuels, params->textureFuelCount, params->textureFuelVersion };
        arrayDatas[eArray_burns] = { params->textureBurns, params->textureBurnCount, params->textureBurnVersion };
        arrayDatas[eArray_smokes] = { params->textureSmokes, params->textureSmokeCount, params->textureSmokeVersion };
        arrayDatas[eArray_coupleRateVelocities] = { params->textureCoupleRateVelocities, params->textureCoupleRateVelocityCount, params->textureCoupleRateVelocityVersion };
        arrayDatas[eArray_coupleRateDivergences] = { params->textureCoupleRateDivergences, params->textureCoupleRateDivergenceCount, params->textureCoupleRateDivergenceVersion };
        arrayDatas[eArray_coupleRateTemperatures] = { params->textureCoupleRateTemperatures, params->textureCoupleRateTemperatureCount, params->textureCoupleRateTemperatureVersion };
        arrayDatas[eArray_coupleRateFuels] = { params->textureCoupleRateFuels, params->textureCoupleRateFuelCount, params->textureCoupleRateFuelVersion };
        arrayDatas[eArray_coupleRateBurns] = { params->textureCoupleRateBurns, params->textureCoupleRateBurnCount, params->textureCoupleRateBurnVersion };
        arrayDatas[eArray_coupleRateSmokes] = { params->textureCoupleRateSmokes, params->textureCoupleRateSmokeCount, params->textureCoupleRateSmokeVersion };
        NvFlowUint64 firstElements[eArray_count] = {};

        NvFlowBufferTransient* arraysTransient = NvFlowArrayBuffer_update(in->context, &inst->arrayBuffer, params->luid, arrayDatas, firstElements, eArray_count, nullptr, "EmitterTextureUpload");

        NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;

        NvFlowFloat3 vidxToWorld = {
            blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
            blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
            blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
        };

        NvFlowInt4 locationMin = {};
        NvFlowInt4 locationMax = {};
        computeEmitterBoxBounds(params->localToWorld, params->layer, params->level, params->position, params->halfSize, blockSizeWorld, &locationMin, &locationMax);

        NvFlowUint numLocations = (locationMax.x - locationMin.x) * (locationMax.y - locationMin.y) * (locationMax.z - locationMin.z);

        NvFlowBool32 texIsSrgb = NV_FLOW_FALSE;
        if (in->value.format == eNvFlowFormat_r8g8b8a8_unorm)
        {
            if (ptr->contextInterface.isFeatureSupported(in->context, eNvFlowContextFeature_aliasResourceFormats))
            {
                texIsSrgb = NV_FLOW_TRUE;
            }
        }

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, numLocations);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (EmitterTextureCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterTextureCS_Params));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->isVelocity = isVelocity;
            mapped->textureFirstElement = params->textureFirstElement;
            mapped->needsSrgbConversion = params->colorIsSrgb && !texIsSrgb;

            mapped->textureWidth = params->textureWidth;
            mapped->textureHeight = params->textureHeight;
            mapped->textureDepth = params->textureDepth;
            mapped->deltaTime = 0.5f * in->deltaTime;

            mapped->table = *levelParams;

            if (isVelocity)
            {
                mapped->targetValue = NvFlowFloat4{
                    params->velocity.x,
                    params->velocity.y,
                    params->velocity.z,
                    params->divergence
                };
                mapped->coupleRate = NvFlowFloat4{
                    params->coupleRateVelocity,
                    params->coupleRateVelocity,
                    params->coupleRateVelocity,
                    params->coupleRateDivergence
                };
                mapped->targetValueScale = NvFlowFloat4{
                    params->velocityScale,
                    params->velocityScale,
                    params->velocityScale,
                    params->divergenceScale
                };
            }
            else
            {
                mapped->targetValue = NvFlowFloat4{
                    params->temperature,
                    params->fuel,
                    params->burn,
                    params->smoke
                };
                mapped->coupleRate = NvFlowFloat4{
                    params->coupleRateTemperature,
                    params->coupleRateFuel,
                    params->coupleRateBurn,
                    params->coupleRateSmoke
                };
                mapped->targetValueScale = NvFlowFloat4{
                    params->temperatureScale,
                    params->fuelScale,
                    params->burnScale,
                    params->smokeScale
                };
            }

            mapped->vidxToWorld = vidxToWorld;
            mapped->velocityIsWorldSpace = params->velocityIsWorldSpace;

            mapped->worldToLocal = matrixTranspose(matrixInverse(params->localToWorld));
            mapped->localToWorldVelocity = matrixTranspose(params->localToWorldVelocity);

            mapped->halfSizeInv = NvFlowFloat3{
                1.f / params->halfSize.x,
                1.f / params->halfSize.y,
                1.f / params->halfSize.z
            };
            mapped->pad2 = 0.f;

            mapped->halfSize = params->halfSize;
            mapped->pad3 = 0.f;

            mapped->position = params->position;
            mapped->textureIsColumnMajor = params->textureIsColumnMajor;

            mapped->locationOffset = locationMin;
            mapped->locationExtent = NvFlowUint4{
                NvFlowUint(locationMax.x - locationMin.x),
                NvFlowUint(locationMax.y - locationMin.y),
                NvFlowUint(locationMax.z - locationMin.z),
                NvFlowUint(locationMax.w - locationMin.w)
            };

            mapped->minDistance = params->minDistance;
            mapped->maxDistance = params->maxDistance;
            mapped->range_distances = { (NvFlowUint)firstElements[eArray_distances], (NvFlowUint)params->textureDistanceCount };
            mapped->range_velocities = { (NvFlowUint)firstElements[eArray_velocities], (NvFlowUint)params->textureVelocityCount };
            mapped->range_divergences = { (NvFlowUint)firstElements[eArray_divergences], (NvFlowUint)params->textureDivergenceCount };
            mapped->range_temperatures = { (NvFlowUint)firstElements[eArray_temperatures], (NvFlowUint)params->textureTemperatureCount };
            mapped->range_fuels = { (NvFlowUint)firstElements[eArray_fuels], (NvFlowUint)params->textureFuelCount };
            mapped->range_burns = { (NvFlowUint)firstElements[eArray_burns], (NvFlowUint)params->textureBurnCount };
            mapped->range_smokes = { (NvFlowUint)firstElements[eArray_smokes], (NvFlowUint)params->textureSmokeCount };
            mapped->range_coupleRateVelocities = { (NvFlowUint)firstElements[eArray_coupleRateVelocities], (NvFlowUint)params->textureCoupleRateVelocityCount };
            mapped->range_coupleRateDivergences = { (NvFlowUint)firstElements[eArray_coupleRateDivergences], (NvFlowUint)params->textureCoupleRateDivergenceCount };
            mapped->range_coupleRateTemperatures = { (NvFlowUint)firstElements[eArray_coupleRateTemperatures], (NvFlowUint)params->textureCoupleRateTemperatureCount };
            mapped->range_coupleRateFuels = { (NvFlowUint)firstElements[eArray_coupleRateFuels], (NvFlowUint)params->textureCoupleRateFuelCount };
            mapped->range_coupleRateBurns = { (NvFlowUint)firstElements[eArray_coupleRateBurns], (NvFlowUint)params->textureCoupleRateBurnCount };
            mapped->range_coupleRateSmokes = { (NvFlowUint)firstElements[eArray_coupleRateSmokes], (NvFlowUint)params->textureCoupleRateSmokeCount };

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = constantTransient;
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterTextureCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.arrayValuesIn = arraysTransient;
            passParams.valueIn = in->value.textureTransient;
            passParams.valueOut = valueTemp->textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterTextureCS_addPassCompute(in->context, &ptr->emitterTextureCS, gridDim, &passParams);
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterTextureCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.arrayValuesIn = arraysTransient;
            passParams.valueIn = valueTemp->textureTransient;
            passParams.valueOut = out->value.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterTextureCS_addPassCompute(in->context, &ptr->emitterTextureCS, gridDim, &passParams);
        }
    }

    void EmitterTexture_execute(EmitterTexture* ptr, const NvFlowEmitterTexturePinsIn* in, NvFlowEmitterTexturePinsOut* out)
    {
        NvFlowSparseLevelParams* levelParams = &in->value.sparseParams.levels[in->value.levelIdx];

        // passthrough, since input is mutable
        NvFlowSparseTexture_passThrough(&out->value, &in->value);

        NvFlowSparseTexture valueTemp = in->valueTemp;
        if (!valueTemp.textureTransient)
        {
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &valueTemp, &in->value);
        }

        // clear instance active flags
        for (NvFlowUint instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            ptr->instances[instanceIdx]->instanceActiveCount = 0u;
        }

        for (NvFlowUint velocityIdx = 0u; velocityIdx < in->velocityParamCount; velocityIdx++)
        {
            const NvFlowEmitterTextureParams* params = in->velocityParams[velocityIdx];
            EmitterTexture_executeSingleEmitter(ptr, in, out, levelParams, &valueTemp, params, NV_FLOW_TRUE);
        }
        for (NvFlowUint densityIdx = 0u; densityIdx < in->densityParamCount; densityIdx++)
        {
            const NvFlowEmitterTextureParams* params = in->densityParams[densityIdx];
            EmitterTexture_executeSingleEmitter(ptr, in, out, levelParams, &valueTemp, params, NV_FLOW_FALSE);
        }

        // lazy instance cleanup
        static const NvFlowUint cleanupThreshold = 16u;
        NvFlowUint instanceIdx = 0u;
        while (instanceIdx < ptr->instances.size)
        {
            auto instance = ptr->instances[instanceIdx];
            if (instance->instanceActiveCount == 0u)
            {
                instance->instanceInactiveCount++;
            }
            if (instance->instanceInactiveCount > cleanupThreshold)
            {
                EmitterTextureInstance_destroy(in->context, ptr, instance);
                ptr->instances.removeSwapPointerAtIndex(instanceIdx);
            }
            else
            {
                instanceIdx++;
            }
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterTexture, EmitterTexture)

namespace
{
    struct EmitterTextureAllocate
    {
        NvFlowArray<NvFlowInt4> locationsTmp;

        NvFlowUint cachedMaxLocations = 0u;
    };

    EmitterTextureAllocate* EmitterTextureAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterTextureAllocatePinsIn* in, NvFlowEmitterTextureAllocatePinsOut* out)
    {
        auto ptr = new EmitterTextureAllocate();
        return ptr;
    }

    void EmitterTextureAllocate_destroy(EmitterTextureAllocate* ptr, const NvFlowEmitterTextureAllocatePinsIn* in, NvFlowEmitterTextureAllocatePinsOut* out)
    {
        delete ptr;
    }

    void EmitterTextureAllocate_execute(EmitterTextureAllocate* ptr, const NvFlowEmitterTextureAllocatePinsIn* in, NvFlowEmitterTextureAllocatePinsOut* out)
    {
        const NvFlowSparseParams* in_sparseParams = &in->sparseParams;

        NvFlowUint maxLocations = 0u;
        if (in_sparseParams->levelCount > 0u)
        {
            maxLocations = in_sparseParams->levels[0u].maxLocations;
        }
        ptr->cachedMaxLocations = maxLocations;

        ptr->locationsTmp.size = 0u;
        for (NvFlowUint paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
        {
            const NvFlowEmitterTextureParams* params = in->params[paramIdx];
            if (!params->enabled)
            {
                continue;
            }
            NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->sparseParams, params->layer, params->level);
            if (layerParamIdx == ~0u)
            {
                continue;
            }
            const NvFlowSparseLayerParams* layerParams = &in->sparseParams.layers[layerParamIdx];
            if (layerParams->forceDisableEmitters)
            {
                continue;
            }

            if (params->allocationScale > 0.f)
            {
                NvFlowFloat3 halfSize = params->halfSize;
                halfSize.x *= params->allocationScale;
                halfSize.y *= params->allocationScale;
                halfSize.z *= params->allocationScale;

                NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;

                NvFlowInt4 locationMin = {};
                NvFlowInt4 locationMax = {};
                computeEmitterBoxBounds(
                    params->localToWorld,
                    params->layer,
                    params->level,
                    params->position,
                    halfSize,
                    blockSizeWorld,
                    &locationMin,
                    &locationMax
                );

                NvFlowUint locationPushCount = 0u;
                for (int k = locationMin.z; k < locationMax.z; k++)
                {
                    for (int j = locationMin.y; j < locationMax.y; j++)
                    {
                        for (int i = locationMin.x; i < locationMax.x; i++)
                        {
                            ptr->locationsTmp.pushBack(NvFlowInt4{ i, j, k, locationMin.w });
                            locationPushCount++;
                            if (locationPushCount >= ptr->cachedMaxLocations)
                            {
                                k = locationMax.z;
                                j = locationMax.y;
                                i = locationMax.x;
                            }
                        }
                    }
                }
            }
        }
        out->locations = ptr->locationsTmp.data;
        out->locationCount = ptr->locationsTmp.size;
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterTextureAllocate, EmitterTextureAllocate)
