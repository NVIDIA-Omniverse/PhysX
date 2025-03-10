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

#include "shaders/EmitterParams.h"

#include "EmitterCommon.h"

#include "shaders/EmitterBoxCS.hlsl.h"

namespace
{
    struct EmitterBox
    {
        NvFlowContextInterface contextInterface = {};

        EmitterBoxCS_Pipeline emitterSimpleCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};
        NvFlowUploadBuffer instanceBuffer = {};
        NvFlowUploadBuffer planesBuffer = {};
        NvFlowUploadBuffer planeCountsBuffer = {};
        NvFlowUploadBuffer blockIdxBuffer = {};
        NvFlowUploadBuffer rangeBuffer = {};
        NvFlowUploadBuffer instanceIdBuffer = {};

        NvFlowArray<NvFlowFloat4> planes;
        NvFlowArray<NvFlowUint> planeCounts;

        NvFlowArray<NvFlowUint> perBlockCount;
        NvFlowArray<NvFlowUint> perBlockIndex;
        NvFlowArray<NvFlowUint2> perBlockInstanceBlockIdx;

        NvFlowArray<NvFlowUint> blockListBlockIdx;
        NvFlowArray<NvFlowUint2> blockListRange;
        NvFlowArray<NvFlowUint> blockListInstanceId;

        NvFlowArray<NvFlowFloat4x4> oldLocalToWorlds;

        NvFlowArray<const NvFlowEmitterBoxParams*> tmpParams;
        NvFlowArray<int> param_layers;
    };

    NV_FLOW_INLINE NvFlowFloat3 EmitterBox_getVelocity(const NvFlowEmitterBoxParams* params)
    {
        if (params->velocityIsWorldSpace)
        {
            return params->velocity;
        }
        NvFlowFloat4 vel = { params->velocity.x, params->velocity.y, params->velocity.z, 0.f };
        vel = NvFlowMath::vector4Transform(vel, params->localToWorldVelocity);
        return NvFlowFloat3{ vel.x, vel.y, vel.z };
    }

    EmitterBox* EmitterBox_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterBoxPinsIn* in, NvFlowEmitterBoxPinsOut* out)
    {
        auto ptr = new EmitterBox();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EmitterBoxCS_init(&ptr->contextInterface, in->context, &ptr->emitterSimpleCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->instanceBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(EmitterBoxCS_InstanceParams));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->planesBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->planeCountsBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->blockIdxBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->rangeBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint2));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->instanceIdBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        return ptr;
    }

    void EmitterBox_destroy(EmitterBox* ptr, const NvFlowEmitterBoxPinsIn* in, NvFlowEmitterBoxPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->instanceBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->planesBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->planeCountsBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->blockIdxBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->rangeBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->instanceIdBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        EmitterBoxCS_destroy(in->context, &ptr->emitterSimpleCS);

        delete ptr;
    }

    void EmitterBox_executeBatchEmitter(
        EmitterBox* ptr,
        const NvFlowEmitterBoxPinsIn* in,
        NvFlowEmitterBoxPinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        NvFlowSparseTexture* valueTemp,
        const NvFlowEmitterBoxParams*const* paramDatas,
        NvFlowUint64 paramCount,
        NvFlowBool32 isVelocity
    )
    {
        // No impact if zero active blocks
        if (levelParams->numLocations == 0u)
        {
            return;
        }

        const NvFlowSparseParams* sparseParams = &in->value.sparseParams;

        ptr->planes.size = 0u;
        ptr->planeCounts.size = 0u;

        ptr->perBlockCount.size = 0u;
        ptr->perBlockIndex.size = 0u;
        ptr->perBlockInstanceBlockIdx.size = 0u;

        ptr->perBlockCount.reserve(sparseParams->locationCount);
        ptr->perBlockCount.size = sparseParams->locationCount;
        ptr->perBlockIndex.reserve(sparseParams->locationCount);
        ptr->perBlockIndex.size = sparseParams->locationCount;
        for (NvFlowUint64 idx = 0u; idx < ptr->perBlockCount.size; idx++)
        {
            ptr->perBlockCount[idx] = 0u;
            ptr->perBlockIndex[idx] = 0u;
        }

        auto mappedInstances = (EmitterBoxCS_InstanceParams*)NvFlowUploadBuffer_map(in->context, &ptr->instanceBuffer, paramCount * sizeof(EmitterBoxCS_InstanceParams));
        for (NvFlowUint64 instanceIdx = 0u; instanceIdx < paramCount; instanceIdx++)
        {
            auto mapped = mappedInstances + instanceIdx;

            const NvFlowEmitterBoxParams* params = paramDatas[instanceIdx];

            int params_layer = params->layer;
            if (instanceIdx < ptr->param_layers.size)
            {
                params_layer = ptr->param_layers[instanceIdx];
            }

            NvFlowBool32 enabled = params->enabled;
            // Apply post pressure if requested
            if (isVelocity)
            {
                if (in->isPostPressure && !params->applyPostPressure)
                {
                    enabled = NV_FLOW_FALSE;
                }
                if (!in->isPostPressure && params->applyPostPressure)
                {
                    enabled = NV_FLOW_FALSE;
                }
            }
            NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->value.sparseParams, params_layer, params->level);
            NvFlowFloat3 blockSizeWorld = { 0.5f, 0.5f, 0.5f };
            // early out if layer is not active
            if (layerParamIdx == ~0u)
            {
                enabled = NV_FLOW_FALSE;
            }
            else
            {
                const NvFlowSparseLayerParams* layerParams = &in->value.sparseParams.layers[layerParamIdx];
                if (layerParams->forceDisableEmitters)
                {
                    enabled = NV_FLOW_FALSE;
                }
                blockSizeWorld = layerParams->blockSizeWorld;
            }
            // Early out if geometry has no volume
            if (params->halfSize.x <= 0.f || params->halfSize.y <= 0.f || params->halfSize.z <= 0.f)
            {
                enabled = NV_FLOW_FALSE;
            }
            // Early out if couple rates leave emitter having zero effect
            if (!isVelocity && params->coupleRateTemperature <= 0.f && params->coupleRateFuel <= 0.f && params->coupleRateBurn <= 0.f && params->coupleRateSmoke <= 0.f)
            {
                enabled = NV_FLOW_FALSE;
            }
            if (isVelocity && params->coupleRateVelocity <= 0.f && params->coupleRateDivergence <= 0.f)
            {
                enabled = NV_FLOW_FALSE;
            }

            using namespace NvFlowMath;

            NvFlowFloat3 vidxToWorld = {
                blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
                blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
                blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
            };

            NvFlowFloat4x4 worldToLocal = matrixTranspose(matrixInverse(params->localToWorld));

            NvFlowInt4 instanceMin = {};
            NvFlowInt4 instanceMax = {};
            computeEmitterBoxBounds(params->localToWorld, params_layer, params->level, params->position, params->halfSize, blockSizeWorld, &instanceMin, &instanceMax);

            if (enabled)
            {
                NvFlowUint64 totalLocations = (NvFlowUint64)(instanceMax.x - instanceMin.x) * (NvFlowUint64)(instanceMax.y - instanceMin.y) * (NvFlowUint64)(instanceMax.z - instanceMin.z);
                NvFlowUint numLocations = sparseParams->levels[0u].numLocations;
                if (totalLocations > (NvFlowUint64)numLocations)
                {
                    for (NvFlowUint blockIdx = 0u; blockIdx < sparseParams->locationCount; blockIdx++)
                    {
                        NvFlowInt4 location = sparseParams->locations[blockIdx];
                        if (location.x >= instanceMin.x && location.x < instanceMax.x &&
                            location.y >= instanceMin.y && location.y < instanceMax.y &&
                            location.z >= instanceMin.z && location.z < instanceMax.z)
                        {
                            ptr->perBlockCount[blockIdx]++;
                            ptr->perBlockInstanceBlockIdx.pushBack(NvFlowUint2{ (NvFlowUint)instanceIdx, blockIdx });
                        }
                    }
                }
                else
                {
                    for (int k = instanceMin.z; k < instanceMax.z; k++)
                    {
                        for (int j = instanceMin.y; j < instanceMax.y; j++)
                        {
                            for (int i = instanceMin.x; i < instanceMax.x; i++)
                            {
                                NvFlowInt4 location = { i, j, k, instanceMin.w };
                                NvFlowUint blockIdx = NvFlowSparseParams_locationToBlockIdx(sparseParams, location);
                                if (blockIdx != ~0u)
                                {
                                    ptr->perBlockCount[blockIdx]++;
                                    ptr->perBlockInstanceBlockIdx.pushBack(NvFlowUint2{ (NvFlowUint)instanceIdx, blockIdx });
                                }
                            }
                        }
                    }
                }
            }

            if (isVelocity)
            {
                NvFlowFloat3 velocity = EmitterBox_getVelocity(params);
                mapped->targetValue = NvFlowFloat4{
                    velocity.x,
                    velocity.y,
                    velocity.z,
                    params->divergence
                };
                mapped->coupleRate = NvFlowFloat4{
                    params->coupleRateVelocity,
                    params->coupleRateVelocity,
                    params->coupleRateVelocity,
                    params->coupleRateDivergence
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
            }

            mapped->vidxToWorld = vidxToWorld;
            mapped->layerAndLevel = NvFlow_packLayerAndLevel(params_layer, params->level);

            mapped->worldToLocal = worldToLocal;
            mapped->localToWorldOld = matrixTranspose(params->localToWorld);
            if (instanceIdx < ptr->oldLocalToWorlds.size)
            {
                mapped->localToWorldOld = matrixTranspose(ptr->oldLocalToWorlds[instanceIdx]);
            }

            mapped->halfSize = params->halfSize;
            mapped->enabled = enabled;

            mapped->position = params->position;
            mapped->clippingPlaneCountCount = (NvFlowUint)params->clippingPlaneCountCount;

            mapped->physicsVelocityScale = isVelocity ? params->physicsVelocityScale : 0.f;
            mapped->physicsDeltaTimeInv = 1.f / (in->deltaTime);
            mapped->multisample = params->multisample;
            mapped->clippingPlaneCount = (NvFlowUint)params->clippingPlaneCount;

            mapped->clippingPlanesOffset = (NvFlowUint)ptr->planes.size;
            mapped->clippingPlaneCountsOffset = (NvFlowUint)ptr->planeCounts.size;
            mapped->pad1 = 0.f;
            mapped->pad2 = 0.f;

            for (NvFlowUint64 idx = 0u; idx < params->clippingPlaneCount; idx++)
            {
                ptr->planes.pushBack(params->clippingPlanes[idx]);
            }
            for (NvFlowUint64 idx = 0u; idx < params->clippingPlaneCountCount; idx++)
            {
                ptr->planeCounts.pushBack(params->clippingPlaneCounts[idx]);
            }
        }
        NvFlowBufferTransient* instanceTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->instanceBuffer);

        ptr->blockListBlockIdx.size = 0u;
        ptr->blockListRange.size = 0u;
        NvFlowUint perBlockScan = 0u;
        for (NvFlowUint64 blockIdx = 0u; blockIdx < ptr->perBlockCount.size; blockIdx++)
        {
            NvFlowUint perBlockCount = ptr->perBlockCount[blockIdx];
            ptr->perBlockIndex[blockIdx] = perBlockScan;
            if (perBlockCount > 0u)
            {
                ptr->blockListBlockIdx.pushBack((NvFlowUint)blockIdx);
                ptr->blockListRange.pushBack(NvFlowUint2{ perBlockScan, perBlockScan + perBlockCount });
            }
            perBlockScan += perBlockCount;
        }
        // scatter
        ptr->blockListInstanceId.size = 0u;
        ptr->blockListInstanceId.reserve(ptr->perBlockInstanceBlockIdx.size);
        ptr->blockListInstanceId.size = ptr->perBlockInstanceBlockIdx.size;
        for (NvFlowUint64 idx = 0u; idx < ptr->perBlockInstanceBlockIdx.size; idx++)
        {
            NvFlowUint2 instanceBlockIdx = ptr->perBlockInstanceBlockIdx[idx];
            NvFlowUint writeIdx = ptr->perBlockIndex[instanceBlockIdx.y];
            ptr->perBlockIndex[instanceBlockIdx.y]++;
            ptr->blockListInstanceId[writeIdx] = instanceBlockIdx.x;
        }

        auto mappedPlanes = (NvFlowFloat4*)NvFlowUploadBuffer_map(in->context, &ptr->planesBuffer, ptr->planes.size * sizeof(NvFlowFloat4));
        for (NvFlowUint64 idx = 0u; idx < ptr->planes.size; idx++)
        {
            mappedPlanes[idx] = ptr->planes[idx];
        }
        NvFlowBufferTransient* planesTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->planesBuffer);

        auto mappedPlaneCounts = (NvFlowUint*)NvFlowUploadBuffer_map(in->context, &ptr->planeCountsBuffer, ptr->planeCounts.size * sizeof(NvFlowUint));
        for (NvFlowUint64 idx = 0u; idx < ptr->planeCounts.size; idx++)
        {
            mappedPlaneCounts[idx] = ptr->planeCounts[idx];
        }
        NvFlowBufferTransient* planeCountsTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->planeCountsBuffer);

        auto mappedBlockIdx = (NvFlowUint*)NvFlowUploadBuffer_map(in->context, &ptr->blockIdxBuffer, ptr->blockListBlockIdx.size * sizeof(NvFlowUint));
        for (NvFlowUint64 idx = 0u; idx < ptr->blockListBlockIdx.size; idx++)
        {
            mappedBlockIdx[idx] = ptr->blockListBlockIdx[idx];
        }
        NvFlowBufferTransient* blockIdxTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->blockIdxBuffer);
        auto mappedRange = (NvFlowUint2*)NvFlowUploadBuffer_map(in->context, &ptr->rangeBuffer, ptr->blockListRange.size * sizeof(NvFlowUint2));
        for (NvFlowUint64 idx = 0u; idx < ptr->blockListRange.size; idx++)
        {
            mappedRange[idx] = ptr->blockListRange[idx];
        }
        NvFlowBufferTransient* rangeTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->rangeBuffer);
        auto mappedInstanceId = (NvFlowUint*)NvFlowUploadBuffer_map(in->context, &ptr->instanceIdBuffer, ptr->blockListInstanceId.size * sizeof(NvFlowUint));
        for (NvFlowUint64 idx = 0u; idx < ptr->blockListInstanceId.size; idx++)
        {
            mappedInstanceId[idx] = ptr->blockListInstanceId[idx];
        }
        NvFlowBufferTransient* instanceIdTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->instanceIdBuffer);

        NvFlowUint batchCount = (NvFlowUint)ptr->blockListBlockIdx.size;

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, batchCount);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (EmitterBoxCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterSphereCS_Params));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->instanceCount = (NvFlowUint)paramCount;
            mapped->deltaTime = 0.5f * in->deltaTime;

            mapped->table = *levelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = constantTransient;
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterBoxCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gInstanceParams = instanceTransient;
            passParams.gPlanes = planesTransient;
            passParams.gPlaneCounts = planeCountsTransient;
            passParams.gBlockIdx = blockIdxTransient;
            passParams.gRange = rangeTransient;
            passParams.gInstanceId = instanceIdTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.valueIn = in->value.textureTransient;
            passParams.valueOut = valueTemp->textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterBoxCS_addPassCompute(in->context, &ptr->emitterSimpleCS, gridDim, &passParams);
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterBoxCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gInstanceParams = instanceTransient;
            passParams.gPlanes = planesTransient;
            passParams.gPlaneCounts = planeCountsTransient;
            passParams.gBlockIdx = blockIdxTransient;
            passParams.gRange = rangeTransient;
            passParams.gInstanceId = instanceIdTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.valueIn = valueTemp->textureTransient;
            passParams.valueOut = out->value.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterBoxCS_addPassCompute(in->context, &ptr->emitterSimpleCS, gridDim, &passParams);
        }
    }

    void EmitterBox_executeInternal(EmitterBox* ptr, const NvFlowEmitterBoxPinsIn* in, NvFlowEmitterBoxPinsOut* out)
    {
        NvFlowSparseLevelParams* levelParams = &in->value.sparseParams.levels[in->value.levelIdx];

        // passthrough, since input is mutable
        NvFlowSparseTexture_passThrough(&out->value, &in->value);

        NvFlowSparseTexture valueTemp = in->valueTemp;
        if (!valueTemp.textureTransient)
        {
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &valueTemp, &in->value);
        }

        if (in->velocityParamCount > 0u)
        {
            EmitterBox_executeBatchEmitter(ptr, in, out, levelParams, &valueTemp, in->velocityParams, in->velocityParamCount, NV_FLOW_TRUE);
        }
        if (in->densityParamCount > 0u)
        {
            EmitterBox_executeBatchEmitter(ptr, in, out, levelParams, &valueTemp, in->densityParams, in->densityParamCount, NV_FLOW_FALSE);
        }

        // Capture old localToWorlds after post pressure emitter
        if (in->isPostPressure && in->velocityParamCount > 0u)
        {
            ptr->oldLocalToWorlds.size = 0u;
            for (NvFlowUint velocityIdx = 0u; velocityIdx < in->velocityParamCount; velocityIdx++)
            {
                const NvFlowEmitterBoxParams* params = in->velocityParams[velocityIdx];
                ptr->oldLocalToWorlds.pushBack(params->localToWorld);
            }
        }
    }

    void EmitterBox_updateTmpParams(EmitterBox* ptr, const NvFlowEmitterBoxPinsIn* in, NvFlowEmitterBoxPinsOut* out, const NvFlowEmitterBoxParams* const* params, NvFlowUint64 paramCount)
    {
        ptr->tmpParams.size = 0u;
        ptr->param_layers.size = 0u;
        for (NvFlowUint64 idx = 0u; idx < paramCount; idx++)
        {
            if (params[idx]->allLayers)
            {
                for (NvFlowUint64 layerIdx = 0u; layerIdx < in->value.sparseParams.layerCount; layerIdx++)
                {
                    ptr->tmpParams.pushBack(params[idx]);
                    ptr->param_layers.pushBack(NvFlow_unpackLayerAndLevel(in->value.sparseParams.layers[layerIdx].layerAndLevel).x);
                }
            }
            else if (params[idx]->isPhysicsCollision)
            {
                for (NvFlowUint64 layerIdx = 0u; layerIdx < in->physicsCollisionLayerCount; layerIdx++)
                {
                    ptr->tmpParams.pushBack(params[idx]);
                    ptr->param_layers.pushBack(in->physicsCollisionLayers[layerIdx]);
                }
            }
            else
            {
                ptr->tmpParams.pushBack(params[idx]);
                ptr->param_layers.pushBack(params[idx]->layer);
            }
        }
    }

    void EmitterBox_execute(EmitterBox* ptr, const NvFlowEmitterBoxPinsIn* in, NvFlowEmitterBoxPinsOut* out)
    {
        // apply broadcast
        NvFlowEmitterBoxPinsIn inTmp = *in;
        if (in->velocityParamCount > 0u)
        {
            EmitterBox_updateTmpParams(ptr, in, out, in->velocityParams, in->velocityParamCount);
            inTmp.velocityParams = ptr->tmpParams.data;
            inTmp.velocityParamCount = ptr->tmpParams.size;
        }
        if (in->densityParamCount > 0u)
        {
            EmitterBox_updateTmpParams(ptr, in, out, in->densityParams, in->densityParamCount);
            inTmp.densityParams = ptr->tmpParams.data;
            inTmp.densityParamCount = ptr->tmpParams.size;
        }
        EmitterBox_executeInternal(ptr, &inTmp, out);
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterBox, EmitterBox)

namespace
{
    struct EmitterBoxAllocate
    {
        NvFlowArray<NvFlowInt4> locationsTmp;

        NvFlowArray<const NvFlowEmitterBoxParams*> tmpParams;
        NvFlowArray<int> param_layers;

        NvFlowUint cachedMaxLocations = 0u;
    };

    EmitterBoxAllocate* EmitterBoxAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterBoxAllocatePinsIn* in, NvFlowEmitterBoxAllocatePinsOut* out)
    {
        auto ptr = new EmitterBoxAllocate();
        return ptr;
    }

    void EmitterBoxAllocate_destroy(EmitterBoxAllocate* ptr, const NvFlowEmitterBoxAllocatePinsIn* in, NvFlowEmitterBoxAllocatePinsOut* out)
    {
        delete ptr;
    }

    void EmitterBoxAllocate_executeInstance(EmitterBoxAllocate* ptr, const NvFlowEmitterBoxAllocatePinsIn* in, NvFlowEmitterBoxAllocatePinsOut* out, const NvFlowEmitterBoxParams* params, NvFlowUint64 emitterIdx)
    {
        if (!params->enabled)
        {
            return;
        }

        int params_layer = params->layer;
        if (emitterIdx < ptr->param_layers.size)
        {
            params_layer = ptr->param_layers[emitterIdx];
        }

        NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->sparseParams, params_layer, params->level);
        if (layerParamIdx == ~0u)
        {
            return;
        }
        const NvFlowSparseLayerParams* layerParams = &in->sparseParams.layers[layerParamIdx];
        if (layerParams->forceDisableEmitters)
        {
            return;
        }

        if (params->allocationScale > 0.f && params->halfSize.x > 0.f && params->halfSize.y > 0.f && params->halfSize.z > 0.f)
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
                params_layer,
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

    void EmitterBoxAllocate_executeInternal(EmitterBoxAllocate* ptr, const NvFlowEmitterBoxAllocatePinsIn* in, NvFlowEmitterBoxAllocatePinsOut* out)
    {
        ptr->locationsTmp.size = 0u;
        for (NvFlowUint paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
        {
            const NvFlowEmitterBoxParams* params = in->params[paramIdx];
            EmitterBoxAllocate_executeInstance(ptr, in, out, params, paramIdx);
        }
        out->locations = ptr->locationsTmp.data;
        out->locationCount = ptr->locationsTmp.size;
    }

    void EmitterBoxAllocate_execute(EmitterBoxAllocate* ptr, const NvFlowEmitterBoxAllocatePinsIn* in, NvFlowEmitterBoxAllocatePinsOut* out)
    {
        const NvFlowSparseParams* in_sparseParams = &in->sparseParams;

        NvFlowUint maxLocations = 0u;
        if (in_sparseParams->levelCount > 0u)
        {
            maxLocations = in_sparseParams->levels[0u].maxLocations;
        }
        ptr->cachedMaxLocations = maxLocations;

        // apply broadcast
        NvFlowEmitterBoxAllocatePinsIn inTmp = *in;
        ptr->tmpParams.size = 0u;
        ptr->param_layers.size = 0u;
        for (NvFlowUint64 idx = 0u; idx < in->paramCount; idx++)
        {
            if (in->params[idx]->allLayers)
            {
                for (NvFlowUint64 layerIdx = 0u; layerIdx < in->sparseParams.layerCount; layerIdx++)
                {
                    ptr->tmpParams.pushBack(in->params[idx]);
                    ptr->param_layers.pushBack(NvFlow_unpackLayerAndLevel(in->sparseParams.layers[layerIdx].layerAndLevel).x);
                }
            }
            else if (in->params[idx]->isPhysicsCollision)
            {
                for (NvFlowUint64 layerIdx = 0u; layerIdx < in->physicsCollisionLayerCount; layerIdx++)
                {
                    ptr->tmpParams.pushBack(in->params[idx]);
                    ptr->param_layers.pushBack(in->physicsCollisionLayers[layerIdx]);
                }
            }
            else
            {
                ptr->tmpParams.pushBack(in->params[idx]);
                ptr->param_layers.pushBack(in->params[idx]->layer);
            }
        }
        EmitterBoxAllocate_executeInternal(ptr, &inTmp, out);
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterBoxAllocate, EmitterBoxAllocate)
