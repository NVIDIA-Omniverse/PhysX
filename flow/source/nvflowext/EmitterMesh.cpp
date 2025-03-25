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

#include "NvFlowArrayBuffer.h"
#include "NvFlowDynamicBuffer.h"
#include "NvFlowLocationHashTable.h"
#include "NvFlowReadbackBuffer.h"

#include "shaders/EmitterMeshBounds1CS.hlsl.h"
#include "shaders/EmitterMeshBounds2CS.hlsl.h"
#include "shaders/EmitterMeshBounds3CS.hlsl.h"
#include "shaders/EmitterMeshGenSortKeysCS.hlsl.h"
#include "shaders/EmitterMeshTree1CS.hlsl.h"
#include "shaders/EmitterMeshClosestCS.hlsl.h"
#include "shaders/EmitterMeshApplyCS.hlsl.h"

// feedback interface
namespace
{
    struct EmitterMeshFeedbackInterface
    {
        void* userdata;

        void(NV_FLOW_ABI* notifyUpdate)(void* userdata);

        void(NV_FLOW_ABI* getLocationRange)(void* userdata, NvFlowUint64 luid, NvFlowInt4* pMinLocation, NvFlowInt4* pMaxLocation);
    };
}

namespace
{
    enum Array
    {
        eArray_positions = 0,
        eArray_faceVertexIndex = 1,
        eArray_faceVertexCount = 2,
        eArray_faceVertexStart = 3,
        eArray_velocities = 4,
        eArray_colors = 5,

        eArray_divergences = 6,
        eArray_temperatures = 7,
        eArray_fuels = 8,
        eArray_burns = 9,

        eArray_smokes = 10,
        eArray_coupleRateVelocities = 11,
        eArray_coupleRateDivergences = 12,
        eArray_coupleRateTemperatures = 13,

        eArray_coupleRateFuels = 14,
        eArray_coupleRateBurns = 15,
        eArray_coupleRateSmokes = 16,

        eArray_count = 17
    };

    struct EmitterMeshInstance
    {
        NvFlowUint64 luid = 0llu;

        NvFlowUint instanceActiveCount = 0u;
        NvFlowUint instanceInactiveCount = 0u;

        NvFlowArrayBuffer arrayBuffer = {};
        NvFlowArray<int> faceVertexStarts;
        NvFlowUint64 faceVertexStartVersion = 0u;

        NvFlowFloat4x4 oldLocalToWorld = {};
    };

    struct EmitterMesh
    {
        NvFlowContextInterface contextInterface = {};

        EmitterMeshBounds1CS_Pipeline emitterMeshBounds1CS = {};
        EmitterMeshBounds2CS_Pipeline emitterMeshBounds2CS = {};
        EmitterMeshBounds3CS_Pipeline emitterMeshBounds3CS = {};
        EmitterMeshGenSortKeysCS_Pipeline emitterMeshGenSortKeysCS = {};
        EmitterMeshTree1CS_Pipeline emitterMeshTree1CS = {};
        EmitterMeshClosestCS_Pipeline emitterMeshClosestCS = {};
        EmitterMeshApplyCS_Pipeline emitterMeshApplyCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};
        NvFlowUploadBuffer subStepBuffer = {};

        NvFlowDynamicBuffer bounds1Buffer = {};
        NvFlowDynamicBuffer bounds2Buffer = {};
        NvFlowDynamicBuffer bounds3Buffer = {};
        NvFlowDynamicBuffer bounds4Buffer = {};

        NvFlowRadixSortInterface radixSortInterface = {};
        NvFlowRadixSort* radixSort = nullptr;

        NvFlowReadbackBuffer readback1 = {};
        NvFlowReadbackBuffer readback2 = {};
        NvFlowReadbackBuffer readback3 = {};
        NvFlowReadbackBuffer readback4 = {};

        NvFlowArrayPointer<EmitterMeshInstance*> instances;

        NvFlowArray<const NvFlowEmitterMeshParams*> tmpParams;
        NvFlowArray<int> param_layers;
    };

    void EmitterMeshInstance_init(NvFlowContext* context, EmitterMesh* ptr, EmitterMeshInstance* inst, NvFlowUint64 luid)
    {
        inst->luid = luid;
        NvFlowArrayBuffer_init(&ptr->contextInterface, context, &inst->arrayBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        inst->instanceActiveCount = 0u;
        inst->instanceInactiveCount = 0u;
        inst->faceVertexStarts.size = 0u;
        inst->faceVertexStartVersion = 0llu;
        inst->oldLocalToWorld = NvFlowFloat4x4{ {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f} };
    }

    void EmitterMeshInstance_destroy(NvFlowContext* context, EmitterMesh* ptr, EmitterMeshInstance* inst)
    {
        inst->luid = 0llu;
        NvFlowArrayBuffer_destroy(context, &inst->arrayBuffer);
        inst->instanceActiveCount = 0u;
        inst->instanceInactiveCount = 0u;
        inst->faceVertexStarts.size = 0u;
        inst->faceVertexStartVersion = 0llu;
        inst->oldLocalToWorld = NvFlowFloat4x4{ {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f} };
    }

    EmitterMesh* EmitterMesh_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterMeshPinsIn* in, NvFlowEmitterMeshPinsOut* out)
    {
        auto ptr = new EmitterMesh();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EmitterMeshBounds1CS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshBounds1CS);
        EmitterMeshBounds2CS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshBounds2CS);
        EmitterMeshBounds3CS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshBounds3CS);
        EmitterMeshGenSortKeysCS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshGenSortKeysCS);
        EmitterMeshTree1CS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshTree1CS);
        EmitterMeshClosestCS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshClosestCS);
        EmitterMeshApplyCS_init(&ptr->contextInterface, in->context, &ptr->emitterMeshApplyCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->subStepBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->bounds1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->bounds2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->bounds3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->bounds4Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint4));

        NvFlowRadixSortInterface_duplicate(&ptr->radixSortInterface, NvFlowGetRadixSortInterface());
        ptr->radixSort = ptr->radixSortInterface.create(&ptr->contextInterface, in->context);

        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback1);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback2);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback3);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback4);

        return ptr;
    }

    void EmitterMesh_destroy(EmitterMesh* ptr, const NvFlowEmitterMeshPinsIn* in, NvFlowEmitterMeshPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->subStepBuffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->bounds1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->bounds2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->bounds3Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->bounds4Buffer);

        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback1);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback2);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback3);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback4);

        for (NvFlowUint idx = 0u; idx < ptr->instances.size; idx++)
        {
            if (ptr->instances[idx])
            {
                EmitterMeshInstance_destroy(in->context, ptr, ptr->instances[idx]);
            }
        }
        ptr->instances.deletePointers();

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        EmitterMeshBounds1CS_destroy(in->context, &ptr->emitterMeshBounds1CS);
        EmitterMeshBounds2CS_destroy(in->context, &ptr->emitterMeshBounds2CS);
        EmitterMeshBounds3CS_destroy(in->context, &ptr->emitterMeshBounds3CS);
        EmitterMeshGenSortKeysCS_destroy(in->context, &ptr->emitterMeshGenSortKeysCS);
        EmitterMeshTree1CS_destroy(in->context, &ptr->emitterMeshTree1CS);
        EmitterMeshClosestCS_destroy(in->context, &ptr->emitterMeshClosestCS);
        EmitterMeshApplyCS_destroy(in->context, &ptr->emitterMeshApplyCS);

        ptr->radixSortInterface.destroy(in->context, ptr->radixSort);

        delete ptr;
    }

    void EmitterMesh_executeSingleEmitter(
        EmitterMesh* ptr,
        NvFlowUint emitterIdx,
        const NvFlowEmitterMeshPinsIn* in,
        NvFlowEmitterMeshPinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        NvFlowSparseTexture* valueTemp,
        const NvFlowEmitterMeshParams* params,
        NvFlowBool32 isVelocity
    )
    {
        using namespace NvFlowMath;

        // resolve instance, done first to ensure proper instance retention
        EmitterMeshInstance* inst = nullptr;
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
            EmitterMeshInstance_init(in->context, ptr, inst, params->luid);

            inst->oldLocalToWorld = params->localToWorld;
        }
        inst->instanceActiveCount++;

        // always update oldLocalToWorld
        NvFlowFloat4x4 oldLocalToWorld = inst->oldLocalToWorld;
        if (in->isPostPressure && in->velocityParamCount > 0u)
        {
            inst->oldLocalToWorld = params->localToWorld;
        }

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

        int params_layer = params->layer;
        if (emitterIdx < ptr->param_layers.size)
        {
            params_layer = ptr->param_layers[emitterIdx];
        }

        NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->value.sparseParams, params_layer, params->level);
        if (layerParamIdx == ~0u)
        {
            return;
        }
        const NvFlowSparseLayerParams* layerParams = &in->value.sparseParams.layers[layerParamIdx];
        if (layerParams->forceDisableEmitters)
        {
            return;
        }

        // Early out if zero positions
        if (params->meshPositionCount == 0u)
        {
            return;
        }

        // Early out if couple rates leave emitter having zero effect, only with defaults
        if (!isVelocity &&
            params->meshCoupleRateTemperatureCount == 0u &&
            params->meshCoupleRateFuelCount == 0u &&
            params->meshCoupleRateBurnCount == 0u &&
            params->meshCoupleRateSmokeCount == 0u &&
            params->coupleRateTemperature <= 0.f &&
            params->coupleRateFuel <= 0.f &&
            params->coupleRateBurn <= 0.f &&
            params->coupleRateSmoke <= 0.f)
        {
            return;
        }
        if (isVelocity &&
            params->meshCoupleRateVelocityCount == 0u &&
            params->meshCoupleRateDivergenceCount == 0u &&
            params->coupleRateVelocity <= 0.f &&
            params->coupleRateDivergence <= 0.f)
        {
            return;
        }

        NvFlowUint numSubSteps = params->numSubSteps;
        if (numSubSteps == 0u)
        {
            numSubSteps = 1u;
        }
        else if (numSubSteps >= 256u)
        {
            numSubSteps = 256u;
        }

        float subStepDeltaTime = in->deltaTime / ((float)numSubSteps);

        NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;

        // Maybe move to GPU?
        if (inst->faceVertexStartVersion != params->meshFaceVertexCountVersion ||
            inst->faceVertexStartVersion == 0llu || params->meshFaceVertexCountVersion == 0llu)
        {
            inst->faceVertexStartVersion = params->meshFaceVertexCountVersion;

            inst->faceVertexStarts.reserve(params->meshFaceVertexCountCount);
            inst->faceVertexStarts.size = params->meshFaceVertexCountCount;
            int sum = 0;
            for (NvFlowUint64 idx = 0u; idx < inst->faceVertexStarts.size; idx++)
            {
                inst->faceVertexStarts[idx] = sum;
                sum += params->meshFaceVertexCounts[idx];
            }
        }

        NvFlowArrayBufferData arrayDatas[eArray_count] = { };
        arrayDatas[eArray_positions] = { params->meshPositions, 3 * params->meshPositionCount, params->meshPositionVersion };
        arrayDatas[eArray_faceVertexIndex] = { params->meshFaceVertexIndices, params->meshFaceVertexIndexCount, params->meshFaceVertexIndexVersion };
        arrayDatas[eArray_faceVertexCount] = { params->meshFaceVertexCounts, params->meshFaceVertexCountCount, params->meshFaceVertexCountVersion };
        arrayDatas[eArray_faceVertexStart] = { inst->faceVertexStarts.data, inst->faceVertexStarts.size, inst->faceVertexStartVersion };
        arrayDatas[eArray_velocities] = { params->meshVelocities, 3 * params->meshVelocityCount, params->meshVelocityVersion };
        arrayDatas[eArray_colors] = { params->meshColors, 3 * params->meshColorCount, params->meshColorVersion };
        arrayDatas[eArray_divergences] = { params->meshDivergences, params->meshDivergenceCount, params->meshDivergenceVersion };
        arrayDatas[eArray_temperatures] = { params->meshTemperatures, params->meshTemperatureCount, params->meshTemperatureVersion };
        arrayDatas[eArray_fuels] = { params->meshFuels, params->meshFuelCount, params->meshFuelVersion };
        arrayDatas[eArray_burns] = { params->meshBurns, params->meshBurnCount, params->meshBurnVersion };
        arrayDatas[eArray_smokes] = { params->meshSmokes, params->meshSmokeCount, params->meshSmokeVersion };
        arrayDatas[eArray_coupleRateVelocities] = { params->meshCoupleRateVelocities, params->meshCoupleRateVelocityCount, params->meshCoupleRateVelocityVersion };
        arrayDatas[eArray_coupleRateDivergences] = { params->meshCoupleRateDivergences, params->meshCoupleRateDivergenceCount, params->meshCoupleRateDivergenceVersion };
        arrayDatas[eArray_coupleRateTemperatures] = { params->meshCoupleRateTemperatures, params->meshCoupleRateTemperatureCount, params->meshCoupleRateTemperatureVersion };
        arrayDatas[eArray_coupleRateFuels] = { params->meshCoupleRateFuels, params->meshCoupleRateFuelCount, params->meshCoupleRateFuelVersion };
        arrayDatas[eArray_coupleRateBurns] = { params->meshCoupleRateBurns, params->meshCoupleRateBurnCount, params->meshCoupleRateBurnVersion };
        arrayDatas[eArray_coupleRateSmokes] = { params->meshCoupleRateSmokes, params->meshCoupleRateSmokeCount, params->meshCoupleRateSmokeVersion };
        NvFlowUint64 firstElements[eArray_count] = {};

        NvFlowBufferTransient* arrayTransient = NvFlowArrayBuffer_update(in->context, &inst->arrayBuffer, params->luid, arrayDatas, firstElements, eArray_count, nullptr, "EmitterMeshUpload");

        NvFlowFloat3 vidxToWorld = {
            blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
            blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
            blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
        };
        NvFlowFloat3 worldToVidx = {
            1.f / vidxToWorld.x,
            1.f / vidxToWorld.y,
            1.f / vidxToWorld.z
        };

        NvFlowInt4 locationMin = { 0, 0, 0, 0 };
        NvFlowInt4 locationMax = { 0, 0, 0, 0 };
        auto iface = (EmitterMeshFeedbackInterface*)in->feedback.data;
        if (iface->getLocationRange)
        {
            iface->getLocationRange(iface->userdata, params->luid, &locationMin, &locationMax);
        }

        NvFlowUint boundsLocations = (locationMax.x - locationMin.x) * (locationMax.y - locationMin.y) * (locationMax.z - locationMin.z);
        NvFlowBool32 dispatchBoundsLocations = boundsLocations < levelParams->numLocations;
        NvFlowUint blockCount = dispatchBoundsLocations ? boundsLocations : levelParams->numLocations;

        NvFlowUint face1Count = (NvFlowUint)(params->meshFaceVertexCountCount);
        NvFlowUint face2Count = (NvFlowUint)((face1Count + 255) / 256);
        NvFlowUint face3Count = (NvFlowUint)((face2Count + 255) / 256);
        NvFlowUint face4Count = (NvFlowUint)((face3Count + 255) / 256);
        // face4Count should always be 1, for limit of 16 million faces

        NvFlowBool32 texIsSrgb = NV_FLOW_FALSE;
        if (in->value.format == eNvFlowFormat_r8g8b8a8_unorm)
        {
            if (ptr->contextInterface.isFeatureSupported(in->context, eNvFlowContextFeature_aliasResourceFormats))
            {
                texIsSrgb = NV_FLOW_TRUE;
            }
        }

        NvFlowDispatchBatches blockBatches;
        NvFlowDispatchBatches_init(&blockBatches, blockCount);
        NvFlowDispatchBatches face1Batches;
        NvFlowDispatchBatches_init(&face1Batches, face2Count);
        NvFlowDispatchBatches face2Batches;
        NvFlowDispatchBatches_init(&face2Batches, face3Count);
        NvFlowDispatchBatches face3Batches;
        NvFlowDispatchBatches_init(&face3Batches, face4Count);

        NvFlowUint64 totalBatches = blockBatches.size >= face1Batches.size ? blockBatches.size : face1Batches.size;

        for (NvFlowUint64 batchIdx = 0u; batchIdx < totalBatches; batchIdx++)
        {
            auto mapped = (EmitterMeshCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterMeshCS_Params));

            mapped->blockIdxOffset = batchIdx < blockBatches.size ? blockBatches[batchIdx].blockIdxOffset : 0u;
            mapped->faceBlockIdx1Offset = batchIdx < face1Batches.size ? face1Batches[batchIdx].blockIdxOffset : 0u;
            mapped->faceBlockIdx2Offset = batchIdx < face2Batches.size ? face2Batches[batchIdx].blockIdxOffset : 0u;
            mapped->faceBlockIdx3Offset = batchIdx < face3Batches.size ? face3Batches[batchIdx].blockIdxOffset : 0u;

            mapped->face1Count = face1Count;
            mapped->face2Count = face2Count;
            mapped->face3Count = face3Count;
            mapped->velocityIsWorldSpace = params->velocityIsWorldSpace;

            mapped->rasterThickness = fmaxf(fabsf(params->minDistance), fabsf(params->maxDistance));
            mapped->minDistance = params->minDistance;
            mapped->maxDistance = params->maxDistance;
            mapped->orientationLeftHanded = params->orientationLeftHanded;

            mapped->emptyMin = NvFlowFloat4{ INFINITY, INFINITY, INFINITY, INFINITY };
            mapped->emptyMax = NvFlowFloat4{ -INFINITY, -INFINITY, -INFINITY, -INFINITY };

            mapped->dispatchBoundsLocations = dispatchBoundsLocations;
            mapped->layerAndLevel = NvFlow_packLayerAndLevel(params_layer, params->level);
            mapped->needsSrgbConversion = params->colorIsSrgb && !texIsSrgb;
            mapped->pad1 = 0u;

            mapped->locationOffset = locationMin;
            mapped->locationExtent = NvFlowUint4{
                NvFlowUint(locationMax.x - locationMin.x),
                NvFlowUint(locationMax.y - locationMin.y),
                NvFlowUint(locationMax.z - locationMin.z),
                NvFlowUint(locationMax.w - locationMin.w)
            };

            mapped->table = *levelParams;

            mapped->localToWorld = matrixTranspose(params->localToWorld);
            mapped->localToWorldVelocity = matrixTranspose(params->localToWorldVelocity);
            mapped->worldToLocal = matrixTranspose(matrixInverse(params->localToWorld));
            mapped->localToWorldOld = matrixTranspose(oldLocalToWorld);

            if (isVelocity)
            {
                mapped->defaultTargetValue = make_float4(params->velocity, params->divergence);
                mapped->defaultCoupleRate = NvFlowFloat4{ params->coupleRateVelocity, params->coupleRateVelocity, params->coupleRateVelocity, params->coupleRateDivergence };
                mapped->targetValueScale = NvFlowFloat4{ params->velocityScale, params->velocityScale, params->velocityScale, params->divergenceScale };
            }
            else
            {
                mapped->defaultTargetValue = NvFlowFloat4{ params->temperature, params->fuel, params->burn, params->smoke };
                mapped->defaultCoupleRate = NvFlowFloat4{ params->coupleRateTemperature, params->coupleRateFuel, params->coupleRateBurn, params->coupleRateSmoke };
                mapped->targetValueScale = NvFlowFloat4{ params->temperatureScale, params->fuelScale, params->burnScale, params->smokeScale };
            }

            mapped->vidxToWorld = vidxToWorld;
            mapped->deltaTime = 0.5f * subStepDeltaTime;

            mapped->worldToVidx = worldToVidx;
            mapped->isVelocity = isVelocity;

            mapped->range_positions = { (NvFlowUint)firstElements[eArray_positions], (NvFlowUint)params->meshPositionCount };
            mapped->range_faceVertexIndices = { (NvFlowUint)firstElements[eArray_faceVertexIndex], (NvFlowUint)params->meshFaceVertexIndexCount };
            mapped->range_faceVertexCounts = { (NvFlowUint)firstElements[eArray_faceVertexCount], (NvFlowUint)params->meshFaceVertexCountCount };
            mapped->range_faceVertexStarts = { (NvFlowUint)firstElements[eArray_faceVertexStart], (NvFlowUint)inst->faceVertexStarts.size };
            mapped->range_velocities = { (NvFlowUint)firstElements[eArray_velocities], (NvFlowUint)params->meshVelocityCount };
            mapped->physicsVelocityScale = isVelocity ? params->physicsVelocityScale : 0.f;
            mapped->physicsDeltaTimeInv = 1.f / (in->deltaTime);

            mapped->range_divergences = { (NvFlowUint)firstElements[eArray_divergences], (NvFlowUint)params->meshDivergenceCount };
            mapped->range_temperatures = { (NvFlowUint)firstElements[eArray_temperatures], (NvFlowUint)params->meshTemperatureCount };

            mapped->range_fuels = { (NvFlowUint)firstElements[eArray_fuels], (NvFlowUint)params->meshFuelCount };
            mapped->range_burns = { (NvFlowUint)firstElements[eArray_burns], (NvFlowUint)params->meshBurnCount };
            mapped->range_smokes = { (NvFlowUint)firstElements[eArray_smokes], (NvFlowUint)params->meshSmokeCount };
            mapped->range_coupleRateVelocities = { (NvFlowUint)firstElements[eArray_coupleRateVelocities], (NvFlowUint)params->meshCoupleRateVelocityCount };

            mapped->range_coupleRateDivergences = { (NvFlowUint)firstElements[eArray_coupleRateDivergences], (NvFlowUint)params->meshCoupleRateDivergenceCount };
            mapped->range_coupleRateTemperatures = { (NvFlowUint)firstElements[eArray_coupleRateTemperatures], (NvFlowUint)params->meshCoupleRateTemperatureCount };;
            mapped->range_coupleRateFuels = { (NvFlowUint)firstElements[eArray_coupleRateFuels], (NvFlowUint)params->meshCoupleRateFuelCount };
            mapped->range_coupleRateBurns = { (NvFlowUint)firstElements[eArray_coupleRateBurns], (NvFlowUint)params->meshCoupleRateBurnCount };

            mapped->range_coupleRateSmokes = { (NvFlowUint)firstElements[eArray_coupleRateSmokes], (NvFlowUint)params->meshCoupleRateSmokeCount };
            mapped->range_colors = { (NvFlowUint)firstElements[eArray_colors], (NvFlowUint)params->meshColorCount };

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);
            if (batchIdx < blockBatches.size)
            {
                blockBatches[batchIdx].globalTransient = constantTransient;
            }
            if (batchIdx < face1Batches.size)
            {
                face1Batches[batchIdx].globalTransient = constantTransient;
            }
            if (batchIdx < face2Batches.size)
            {
                face2Batches[batchIdx].globalTransient = constantTransient;
            }
            if (batchIdx < face3Batches.size)
            {
                face3Batches[batchIdx].globalTransient = constantTransient;
            }
        }

        NvFlowDynamicBuffer_resize(in->context, &ptr->bounds1Buffer, 2u * face1Count * sizeof(NvFlowUint4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->bounds2Buffer, 2u * face2Count * sizeof(NvFlowUint4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->bounds3Buffer, 2u * face3Count * sizeof(NvFlowUint4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->bounds4Buffer, 2u * face4Count * sizeof(NvFlowUint4));

        NvFlowBufferTransient* bounds1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->bounds1Buffer);
        NvFlowBufferTransient* bounds2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->bounds2Buffer);
        NvFlowBufferTransient* bounds3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->bounds3Buffer);
        NvFlowBufferTransient* bounds4Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->bounds4Buffer);

        ptr->radixSortInterface.reserve(in->context, ptr->radixSort, face1Count);

        for (NvFlowUint subStepIdx = numSubSteps - 1u; subStepIdx < numSubSteps; subStepIdx--)
        {
            auto mapped = (EmitterMeshCS_SubStepParams*)NvFlowUploadBuffer_map(in->context, &ptr->subStepBuffer, sizeof(EmitterMeshCS_SubStepParams));

            mapped->subStepIdx = subStepIdx;
            mapped->numSubSteps = numSubSteps;
            mapped->subStepDeltaTime = in->deltaTime / ((float)numSubSteps);
            mapped->subStepAccumDeltaTime = ((float)subStepIdx) * in->deltaTime / ((float)numSubSteps);
            mapped->defaultVelocity = params->velocity;
            mapped->totalDeltaTime = in->deltaTime;
            mapped->velocityScale = params->velocityScale;
            mapped->weight = ((float)subStepIdx) / ((float)numSubSteps);
            mapped->pad2 = 0.f;
            mapped->pad3 = 0.f;

            NvFlowBufferTransient* subStepTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->subStepBuffer);

            for (NvFlowUint64 face1BatchIdx = 0u; face1BatchIdx < face1Batches.size; face1BatchIdx++)
            {
                EmitterMeshBounds1CS_PassParams passParams = {};
                passParams.gParams = face1Batches[face1BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.arrayValuesIn = arrayTransient;
                passParams.bounds2Out = bounds2Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = face1Batches[face1BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterMeshBounds1CS_addPassCompute(in->context, &ptr->emitterMeshBounds1CS, gridDim, &passParams);
            }

            for (NvFlowUint64 face2BatchIdx = 0u; face2BatchIdx < face2Batches.size; face2BatchIdx++)
            {
                EmitterMeshBounds2CS_PassParams passParams = {};
                passParams.gParams = face2Batches[face2BatchIdx].globalTransient;
                passParams.bounds2In = bounds1Buffer;
                passParams.bounds3Out = bounds2Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = face2Batches[face2BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterMeshBounds2CS_addPassCompute(in->context, &ptr->emitterMeshBounds2CS, gridDim, &passParams);
            }

            for (NvFlowUint64 face3BatchIdx = 0u; face3BatchIdx < face3Batches.size; face3BatchIdx++)
            {
                EmitterMeshBounds3CS_PassParams passParams = {};
                passParams.gParams = face3Batches[face3BatchIdx].globalTransient;
                passParams.bounds3In = bounds3Buffer;
                passParams.bounds4Out = bounds4Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = face3Batches[face3BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterMeshBounds3CS_addPassCompute(in->context, &ptr->emitterMeshBounds3CS, gridDim, &passParams);
            }

            NvFlowBufferTransient* keyBuffer = nullptr;
            NvFlowBufferTransient* valBuffer = nullptr;
            ptr->radixSortInterface.getInputBuffers(in->context, ptr->radixSort, &keyBuffer, &valBuffer);

            for (NvFlowUint64 face1BatchIdx = 0u; face1BatchIdx < face1Batches.size; face1BatchIdx++)
            {
                EmitterMeshGenSortKeysCS_PassParams passParams = {};
                passParams.gParams = face1Batches[face1BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.arrayValuesIn = arrayTransient;
                passParams.bounds4In = bounds4Buffer;
                passParams.sortKeyOut = keyBuffer;
                passParams.sortValOut = valBuffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = face1Batches[face1BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterMeshGenSortKeysCS_addPassCompute(in->context, &ptr->emitterMeshGenSortKeysCS, gridDim, &passParams);
            }

            ptr->radixSortInterface.sort(in->context, ptr->radixSort, face1Count, 32u);
            ptr->radixSortInterface.getOutputBuffers(in->context, ptr->radixSort, &keyBuffer, &valBuffer);

            for (NvFlowUint64 face1BatchIdx = 0u; face1BatchIdx < face1Batches.size; face1BatchIdx++)
            {
                EmitterMeshTree1CS_PassParams passParams = {};
                passParams.gParams = face1Batches[face1BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.arrayValuesIn = arrayTransient;
                passParams.sortKeyIn = keyBuffer;
                passParams.sortValIn = valBuffer;
                passParams.bounds1Out = bounds1Buffer;
                passParams.bounds2Out = bounds2Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = face1Batches[face1BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterMeshTree1CS_addPassCompute(in->context, &ptr->emitterMeshTree1CS, gridDim, &passParams);
            }

            for (NvFlowUint64 face2BatchIdx = 0u; face2BatchIdx < face2Batches.size; face2BatchIdx++)
            {
                EmitterMeshBounds2CS_PassParams passParams = {};
                passParams.gParams = face2Batches[face2BatchIdx].globalTransient;
                passParams.bounds2In = bounds2Buffer;
                passParams.bounds3Out = bounds3Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = face2Batches[face2BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterMeshBounds2CS_addPassCompute(in->context, &ptr->emitterMeshBounds2CS, gridDim, &passParams);
            }

            //NvFlowReadbackBuffer_copy(in->context, &ptr->readback1, 2u * face1Count * sizeof(NvFlowUint4), bounds1Buffer, nullptr);
            //NvFlowFloat4* readbackMapped1 = (NvFlowFloat4*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback1, nullptr, nullptr);
            //NvFlowReadbackBuffer_copy(in->context, &ptr->readback2, 2u * face2Count * sizeof(NvFlowUint4), bounds2Buffer, nullptr);
            //NvFlowFloat4* readbackMapped2 = (NvFlowFloat4*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback2, nullptr, nullptr);
            //NvFlowReadbackBuffer_copy(in->context, &ptr->readback3, 2u * face3Count * sizeof(NvFlowUint4), bounds3Buffer, nullptr);
            //NvFlowFloat4* readbackMapped3 = (NvFlowFloat4*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback3, nullptr, nullptr);
            //NvFlowReadbackBuffer_copy(in->context, &ptr->readback4, 2u * face4Count * sizeof(NvFlowUint4), bounds4Buffer, nullptr);
            //NvFlowFloat4* readbackMapped4 = (NvFlowFloat4*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback4, nullptr, nullptr);

            NvFlowSparseTexture closestTexture = {};
            NvFlowSparseTexture_duplicateWithFormat(&ptr->contextInterface, in->context, &closestTexture, &in->value, eNvFlowFormat_r32_uint);

            for (NvFlowUint64 blockBatchIdx = 0u; blockBatchIdx < blockBatches.size; blockBatchIdx++)
            {
                EmitterMeshClosestCS_PassParams passParams = {};
                passParams.gParams = blockBatches[blockBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.sortValIn = valBuffer;
                passParams.bounds1In = bounds1Buffer;
                passParams.bounds2In = bounds2Buffer;
                passParams.bounds3In = bounds3Buffer;
                passParams.bounds4In = bounds4Buffer;
                passParams.closestOut = closestTexture.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 255u) / 256u;
                gridDim.y = blockBatches[blockBatchIdx].blockCount;
                gridDim.z = 1u;

                EmitterMeshClosestCS_addPassCompute(in->context, &ptr->emitterMeshClosestCS, gridDim, &passParams);
            }

            for (NvFlowUint64 blockBatchIdx = 0u; blockBatchIdx < blockBatches.size; blockBatchIdx++)
            {
                EmitterMeshApplyCS_PassParams passParams = {};
                passParams.gParams = blockBatches[blockBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.closestIn = closestTexture.textureTransient;
                passParams.valueIn = in->value.textureTransient;
                passParams.valueOut = valueTemp->textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 255u) / 256u;
                gridDim.y = blockBatches[blockBatchIdx].blockCount;
                gridDim.z = 1u;

                EmitterMeshApplyCS_addPassCompute(in->context, &ptr->emitterMeshApplyCS, gridDim, &passParams);
            }

            for (NvFlowUint64 blockBatchIdx = 0u; blockBatchIdx < blockBatches.size; blockBatchIdx++)
            {
                EmitterMeshApplyCS_PassParams passParams = {};
                passParams.gParams = blockBatches[blockBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.closestIn = closestTexture.textureTransient;
                passParams.valueIn = valueTemp->textureTransient;
                passParams.valueOut = in->value.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 255u) / 256u;
                gridDim.y = blockBatches[blockBatchIdx].blockCount;
                gridDim.z = 1u;

                EmitterMeshApplyCS_addPassCompute(in->context, &ptr->emitterMeshApplyCS, gridDim, &passParams);
            }
        }
    }

    void EmitterMesh_executeInternal(EmitterMesh* ptr, const NvFlowEmitterMeshPinsIn* in, NvFlowEmitterMeshPinsOut* out)
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
            const NvFlowEmitterMeshParams* params = in->velocityParams[velocityIdx];
            EmitterMesh_executeSingleEmitter(ptr, velocityIdx, in, out, levelParams, &valueTemp, params, NV_FLOW_TRUE);
        }
        for (NvFlowUint densityIdx = 0u; densityIdx < in->densityParamCount; densityIdx++)
        {
            const NvFlowEmitterMeshParams* params = in->densityParams[densityIdx];
            EmitterMesh_executeSingleEmitter(ptr, densityIdx, in, out, levelParams, &valueTemp, params, NV_FLOW_FALSE);
        }

        // notify allocate that this updated
        if (in->feedback.data)
        {
            auto iface = (EmitterMeshFeedbackInterface*)in->feedback.data;
            if (iface->notifyUpdate)
            {
                iface->notifyUpdate(iface->userdata);
            }
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
                EmitterMeshInstance_destroy(in->context, ptr, instance);
                ptr->instances.removeSwapPointerAtIndex(instanceIdx);
            }
            else
            {
                instanceIdx++;
            }
        }
    }

    void EmitterMesh_updateTmpParams(EmitterMesh* ptr, const NvFlowEmitterMeshPinsIn* in, NvFlowEmitterMeshPinsOut* out, const NvFlowEmitterMeshParams* const* params, NvFlowUint64 paramCount)
    {
        ptr->tmpParams.size = 0u;
        ptr->param_layers.size = 0u;
        for (NvFlowUint64 idx = 0u; idx < paramCount; idx++)
        {
            if (params[idx]->isPhysicsCollision)
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

    void EmitterMesh_execute(EmitterMesh* ptr, const NvFlowEmitterMeshPinsIn* in, NvFlowEmitterMeshPinsOut* out)
    {
        // if physics collision active, need to apply broadcast
        if (in->physicsCollisionLayerCount > 0u)
        {
            NvFlowEmitterMeshPinsIn inTmp = *in;
            if (in->velocityParamCount > 0u)
            {
                EmitterMesh_updateTmpParams(ptr, in, out, in->velocityParams, in->velocityParamCount);
                inTmp.velocityParams = ptr->tmpParams.data;
                inTmp.velocityParamCount = ptr->tmpParams.size;
            }
            if (in->densityParamCount > 0u)
            {
                EmitterMesh_updateTmpParams(ptr, in, out, in->densityParams, in->densityParamCount);
                inTmp.densityParams = ptr->tmpParams.data;
                inTmp.densityParamCount = ptr->tmpParams.size;
            }
            EmitterMesh_executeInternal(ptr, &inTmp, out);
        }
        else
        {
            ptr->param_layers.size = 0u;
            EmitterMesh_executeInternal(ptr, in, out);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterMesh, EmitterMesh)

namespace
{
    // Avoid member initialization, it can cause padding to not be initialized
    struct EmitterMeshAllocateInstanceKey
    {
        NvFlowBool32 enabled;
        NvFlowFloat3 blockSizeWorld;
        NvFlowFloat4x4 localToWorld;
        NvFlowUint64 meshPositionCount;
        NvFlowUint64 meshPositionVersion;
        int layerAndLevel;
        NvFlowBool32 allocateMask;
    };

    struct EmitterMeshAllocateInstance
    {
        NvFlowUint64 luid = 0llu;

        NvFlowUint instanceActiveCount = 0u;
        NvFlowUint instanceInactiveCount = 0u;

        EmitterMeshAllocateInstanceKey key = {};

        NvFlowLocationHashTable locationHash;
    };

    struct EmitterMeshAllocateTaskParams
    {
        NvFlowLocationHashTable locationHash;
        const NvFlowEmitterMeshParams* params;
        int params_layer;
        NvFlowFloat3 blockSizeWorld;
        NvFlowUint3 blockDim;
    };

    struct EmitterMeshAllocate
    {
        NvFlowContextInterface contextInterface = {};

        NvFlowArrayPointer<EmitterMeshAllocateInstance*> instances;

        NvFlowArray<EmitterMeshAllocateTaskParams> taskParams;

        NvFlowArray<NvFlowInt4> locations;

        NvFlowArray<const NvFlowEmitterMeshParams*> tmpParams;
        NvFlowArray<int> param_layers;

        EmitterMeshFeedbackInterface feedbackInterface = {};

        NvFlowBool32 emitterDidUpdate = NV_FLOW_FALSE;
    };

    EmitterMeshAllocate* EmitterMeshAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterMeshAllocatePinsIn* in, NvFlowEmitterMeshAllocatePinsOut* out)
    {
        auto ptr = new EmitterMeshAllocate();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        return ptr;
    }

    void EmitterMeshAllocate_destroy(EmitterMeshAllocate* ptr, const NvFlowEmitterMeshAllocatePinsIn* in, NvFlowEmitterMeshAllocatePinsOut* out)
    {
        ptr->instances.deletePointers();

        delete ptr;
    }

    EmitterMeshAllocateInstance* EmitterMeshAllocate_getInstance(
        EmitterMeshAllocate* ptr,
        const NvFlowEmitterMeshParams* params
    )
    {
        // resolve instance, done first to ensure proper instance retention
        EmitterMeshAllocateInstance* inst = nullptr;
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
            inst->luid = params->luid;
            inst->instanceActiveCount = 0u;
            inst->instanceInactiveCount = 0u;
        }
        return inst;
    }

    void EmitterMeshAllocate_executeSingleEmitter(
        EmitterMeshAllocate* ptr,
        const NvFlowEmitterMeshAllocatePinsIn* in,
        NvFlowEmitterMeshAllocatePinsOut* out,
        const NvFlowEmitterMeshParams* params,
        NvFlowUint64 emitterIdx
    )
    {
        using namespace NvFlowMath;

        EmitterMeshAllocateInstance* inst = EmitterMeshAllocate_getInstance(ptr, params);
        inst->instanceActiveCount++;

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

        NvFlowFloat3 blockSizeWorld = in->sparseParams.layers[layerParamIdx].blockSizeWorld;

        EmitterMeshAllocateInstanceKey key;
        memset(&key, 0, sizeof(key));                           // explicit to cover any padding
        key.enabled = params->enabled;
        key.blockSizeWorld = blockSizeWorld;
        key.localToWorld = params->localToWorld;
        key.meshPositionCount = params->meshPositionCount;
        key.meshPositionVersion = params->meshPositionVersion;
        key.layerAndLevel = NvFlow_packLayerAndLevel(params_layer, params->level);
        key.allocateMask = params->allocateMask;

        bool positionForceDirty = params->meshPositionCount > 0u && params->meshPositionVersion == 0llu;
        bool isDirty = (memcmp(&key, &inst->key, sizeof(key)) != 0u) || positionForceDirty;
        inst->key = key;
        if (isDirty)
        {
            inst->locationHash.reset();

            static const NvFlowUint64 pointsPerTask = 8192u;
            NvFlowUint64 taskCount = ((params->meshPositionCount + pointsPerTask - 1u) / pointsPerTask);

            NvFlowUint3 blockDim = { 32u, 16u, 16u };
            if (in->baseBlockDimBits.x > 0u && in->baseBlockDimBits.y > 0u && in->baseBlockDimBits.z > 0u)
            {
                blockDim.x = (1u << in->baseBlockDimBits.x);
                blockDim.y = (1u << in->baseBlockDimBits.y);
                blockDim.z = (1u << in->baseBlockDimBits.z);
            }

            ptr->taskParams.reserve(taskCount);
            ptr->taskParams.size = taskCount;

            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                ptr->taskParams[taskIdx].locationHash.reset();
                ptr->taskParams[taskIdx].params = params;
                ptr->taskParams[taskIdx].params_layer = params_layer;
                ptr->taskParams[taskIdx].blockSizeWorld = blockSizeWorld;
                ptr->taskParams[taskIdx].blockDim = blockDim;
            }

            auto task = [](NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
            {
                auto ptr = (EmitterMeshAllocate*)userdata;

                auto& taskParams = ptr->taskParams[taskIdx];

                NvFlowUint64 particleBeginIdx = taskIdx * pointsPerTask;
                NvFlowUint64 particleEndIdx = particleBeginIdx + pointsPerTask;
                if (particleEndIdx > taskParams.params->meshPositionCount)
                {
                    particleEndIdx = taskParams.params->meshPositionCount;
                }

                // disabled, need accurate bounds for dispatch
                //if (taskParams.params->allocateMask)
                {
                    const float xf_neg = 2.f / ((float)taskParams.blockDim.x);
                    const float xf_pos = 1.f - xf_neg;
                    const float yf_neg = 2.f / ((float)taskParams.blockDim.y);
                    const float yf_pos = 1.f - yf_neg;
                    const float zf_neg = 2.f / ((float)taskParams.blockDim.z);
                    const float zf_pos = 1.f - zf_neg;

                    for (NvFlowUint64 particleIdx = particleBeginIdx; particleIdx < particleEndIdx; particleIdx++)
                    {
                        NvFlowFloat3 meshPosition = taskParams.params->meshPositions[particleIdx];
                        NvFlowFloat4 positionLocal = make_float4(meshPosition, 1.f);
                        NvFlowFloat4 position = vector4Transform(positionLocal, taskParams.params->localToWorld);
                        if (position.w > 0.f)
                        {
                            float wInv = 1.f / position.w;
                            position.x *= wInv;
                            position.y *= wInv;
                            position.z *= wInv;
                        }

                        int layerAndLevel = NvFlow_packLayerAndLevel(taskParams.params_layer, taskParams.params->level);

                        NvFlowFloat3 locationf = {
                            position.x / taskParams.blockSizeWorld.x,
                            position.y / taskParams.blockSizeWorld.y,
                            position.z / taskParams.blockSizeWorld.z };

                        NvFlowInt4 location = {
                            int(floorf(locationf.x)),
                            int(floorf(locationf.y)),
                            int(floorf(locationf.z)),
                            layerAndLevel
                        };

                        float xf = (locationf.x - float(location.x));
                        float yf = (locationf.y - float(location.y));
                        float zf = (locationf.z - float(location.z));

                        NvFlowUint entry_mask = 0u;
                        entry_mask |= xf < xf_neg ? 1u : 0u;
                        entry_mask |= xf > xf_pos ? 2u : 0u;
                        entry_mask |= yf < yf_neg ? 4u : 0u;
                        entry_mask |= yf > yf_pos ? 8u : 0u;
                        entry_mask |= zf < zf_neg ? 16u : 0u;
                        entry_mask |= zf > zf_pos ? 32u : 0u;
                        taskParams.locationHash.push(location, entry_mask);
                    }
                }
            };

            ptr->contextInterface.executeTasks(in->context, (NvFlowUint)taskCount, taskCount < 8u ? 8u : 1u, task, ptr);

            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                auto& taskParams = ptr->taskParams[taskIdx];
                for (NvFlowUint locationIdx = 0u; locationIdx < taskParams.locationHash.locations.size; locationIdx++)
                {
                    NvFlowInt4 entry_location = taskParams.locationHash.locations[locationIdx];
                    NvFlowUint entry_mask = taskParams.locationHash.masks[locationIdx];

                    inst->locationHash.push(entry_location, entry_mask);
                }
            }

            NvFlowUint64 baseEntryCount = inst->locationHash.locations.size;
            for (NvFlowUint64 entryIdx = 0u; entryIdx < baseEntryCount; entryIdx++)
            {
                NvFlowInt4 entry_location = inst->locationHash.locations[entryIdx];
                NvFlowUint entry_mask = inst->locationHash.masks[entryIdx];
                NvFlowUint testMask = entry_mask;
                entry_mask = 0u;
                entry_location.x -= 1;
                if (testMask & 1)
                {
                    inst->locationHash.push(entry_location, entry_mask);
                }
                entry_location.x += 2;
                if (testMask & 2)
                {
                    inst->locationHash.push(entry_location, entry_mask);
                }
                entry_location.x -= 1;
                entry_location.y -= 1;
                if (testMask & 4)
                {
                    inst->locationHash.push(entry_location, entry_mask);
                }
                entry_location.y += 2;
                if (testMask & 8)
                {
                    inst->locationHash.push(entry_location, entry_mask);
                }
                entry_location.y -= 1;
                entry_location.z -= 1;
                if (testMask & 16)
                {
                    inst->locationHash.push(entry_location, entry_mask);
                }
                entry_location.z += 2;
                if (testMask & 32)
                {
                    inst->locationHash.push(entry_location, entry_mask);
                }
                entry_location.z -= 1;
            }
        }

        // min/max is used to size thread launch
        inst->locationHash.computeStats();

        if (params->allocateMask)
        {
            for (NvFlowUint locationIdx = 0u; locationIdx < inst->locationHash.locations.size; locationIdx++)
            {
                ptr->locations.pushBack(inst->locationHash.locations[locationIdx]);
            }
        }
    }

    void EmitterMeshAllocate_notifyUpdate(void* userdata)
    {
        auto ptr = (EmitterMeshAllocate*)userdata;

        ptr->emitterDidUpdate = NV_FLOW_TRUE;
    }

    void EmitterMeshAllocate_getLocationRange(void* userdata, NvFlowUint64 luid, NvFlowInt4* pMinLocation, NvFlowInt4* pMaxLocation)
    {
        auto ptr = (EmitterMeshAllocate*)userdata;

        EmitterMeshAllocateInstance* inst = nullptr;
        for (NvFlowUint instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            auto instanceTest = ptr->instances[instanceIdx];
            if (instanceTest->luid == luid)
            {
                inst = instanceTest;
                break;
            }
        }

        if (inst)
        {
            *pMinLocation = inst->locationHash.locationMin;
            *pMaxLocation = inst->locationHash.locationMax;
        }
        else
        {
            *pMinLocation = NvFlowInt4{ 0, 0, 0, 0 };
            *pMaxLocation = NvFlowInt4{ 0, 0, 0, 0 };
        }
    }

    void EmitterMeshAllocate_executeInternal(EmitterMeshAllocate* ptr, const NvFlowEmitterMeshAllocatePinsIn* in, NvFlowEmitterMeshAllocatePinsOut* out)
    {
        using namespace NvFlowMath;

        ptr->feedbackInterface.userdata = ptr;
        ptr->feedbackInterface.notifyUpdate = EmitterMeshAllocate_notifyUpdate;
        ptr->feedbackInterface.getLocationRange = EmitterMeshAllocate_getLocationRange;

        out->feedback.data = &ptr->feedbackInterface;

        // reset locations array
        ptr->locations.size = 0u;

        // clear instance active flags
        for (NvFlowUint instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            ptr->instances[instanceIdx]->instanceActiveCount = 0u;
        }

        for (NvFlowUint paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
        {
            EmitterMeshAllocate_executeSingleEmitter(ptr, in, out, in->params[paramIdx], paramIdx);
        }

        out->locations = ptr->locations.data;
        out->locationCount = ptr->locations.size;

        // reset emitter update flag
        ptr->emitterDidUpdate = NV_FLOW_FALSE;

        // lazy instance cleanup
        static const NvFlowUint cleanupThreshold = 16u;
        NvFlowUint instanceIdx = 0u;
        while (instanceIdx < ptr->instances.size)
        {
            auto& instance = ptr->instances[instanceIdx];
            if (instance->instanceActiveCount == 0u)
            {
                instance->instanceInactiveCount++;
            }
            if (instance->instanceInactiveCount > cleanupThreshold)
            {
                delete instance;
                instance = nullptr;
                ptr->instances.removeSwapPointerAtIndex(instanceIdx);
            }
            else
            {
                instanceIdx++;
            }
        }
    }

    void EmitterMeshAllocate_execute(EmitterMeshAllocate* ptr, const NvFlowEmitterMeshAllocatePinsIn* in, NvFlowEmitterMeshAllocatePinsOut* out)
    {
        // if physics collision active, need to apply broadcast
        if (in->physicsCollisionLayerCount > 0u)
        {
            NvFlowEmitterMeshAllocatePinsIn inTmp = *in;
            ptr->tmpParams.size = 0u;
            ptr->param_layers.size = 0u;
            for (NvFlowUint64 idx = 0u; idx < in->paramCount; idx++)
            {
                if (in->params[idx]->isPhysicsCollision)
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
            EmitterMeshAllocate_executeInternal(ptr, &inTmp, out);
        }
        else
        {
            ptr->param_layers.size = 0u;
            EmitterMeshAllocate_executeInternal(ptr, in, out);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterMeshAllocate, EmitterMeshAllocate)
