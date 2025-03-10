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

#include "shaders/EmitterPoint1CS.hlsl.h"
#include "shaders/EmitterPoint2CS.hlsl.h"
#include "shaders/EmitterPoint3CS.hlsl.h"
#include "shaders/EmitterPoint4CS.hlsl.h"
#include "shaders/EmitterPointReduce1CS.hlsl.h"
#include "shaders/EmitterPointReduce2CS.hlsl.h"
#include "shaders/EmitterPointReduce3CS.hlsl.h"
#include "shaders/EmitterPointScan1CS.hlsl.h"
#include "shaders/EmitterPointScan2CS.hlsl.h"
#include "shaders/EmitterPointScan3CS.hlsl.h"
#include "shaders/EmitterPointMarkCS.hlsl.h"
#include "shaders/EmitterPointClearMarkedCS.hlsl.h"
#include "shaders/EmitterPointClearDownsampleCS.hlsl.h"

#include "shaders/VoxelInterpolateParams.h"
#include "shaders/VoxelInterpolatePrepassCS.hlsl.h"
#include "shaders/VoxelInterpolateDownsampleCS.hlsl.h"
#include "shaders/VoxelInterpolatePropagateCS.hlsl.h"
#include "shaders/VoxelInterpolateUpsampleCS.hlsl.h"
#include "shaders/VoxelInterpolateNullCS.hlsl.h"

#include "NvFlowArrayBuffer.h"
#include "NvFlowDynamicBuffer.h"
#include "NvFlowLocationHashTable.h"
#include "NvFlowTextureVariable.h"

//#define DEBUG_READBACK

#ifdef DEBUG_READBACK
#include "NvFlowReadbackBuffer.h"
#endif

// feedback interface
namespace
{
    struct EmitterPointFeedbackInterface
    {
        NvFlowUint64 globalUpdateVersion;
        NvFlowUint64 globalChangeVersion;
        NvFlowBool32 anyStreamingEnabled;
        NvFlowBool32 anyStreamingDisabled;
        NvFlowBool32 anyStreamingClearEnabled;
        NvFlowBool32 anyInterpolationEnabled;

        const NvFlowUint64* luids;
        NvFlowUint64 luidCount;
        const NvFlowUint* batchIdxs;
        NvFlowUint64 batchIdxCount;
        const NvFlowUint64* changeVersions;
        NvFlowUint64 changeCount;
        const NvFlowEmitterPointParams** params;
        NvFlowUint64 paramCount;
        const NvFlowBool32* streamingEnableds;
        NvFlowUint64 streamingEnabledCount;
        const NvFlowBool32* interpolationEnableds;
        NvFlowUint64 interpolationEnabledCount;

        void* userdata;
        void(NV_FLOW_ABI* reportUpdate)(void* userdata, NvFlowUint64 globalChangeVersion, NvFlowUint64 stagedChangeVersion);
    };
}

namespace
{
    enum Array
    {
        eArray_positions = 0,
        eArray_velocities = 1,
        eArray_colors = 2,

        eArray_divergences = 3,
        eArray_temperatures = 4,
        eArray_fuels = 5,
        eArray_burns = 6,

        eArray_smokes = 7,
        eArray_coupleRateVelocities = 8,
        eArray_coupleRateDivergences = 9,
        eArray_coupleRateTemperatures = 10,

        eArray_coupleRateFuels = 11,
        eArray_coupleRateBurns = 12,
        eArray_coupleRateSmokes = 13,

        eArray_count = 14
    };

    struct EmitterPoint
    {
        NvFlowContextInterface contextInterface = {};

        EmitterPoint1CS_Pipeline emitterPoint1CS = {};
        EmitterPoint2CS_Pipeline emitterPoint2CS = {};
        EmitterPoint3CS_Pipeline emitterPoint3CS = {};
        EmitterPoint4CS_Pipeline emitterPoint4CS = {};
        EmitterPointReduce1CS_Pipeline emitterPointReduce1CS = {};
        EmitterPointReduce2CS_Pipeline emitterPointReduce2CS = {};
        EmitterPointReduce3CS_Pipeline emitterPointReduce3CS = {};
        EmitterPointScan1CS_Pipeline emitterPointScan1CS = {};
        EmitterPointScan2CS_Pipeline emitterPointScan2CS = {};
        EmitterPointScan3CS_Pipeline emitterPointScan3CS = {};
        EmitterPointMarkCS_Pipeline emitterPointMarkCS = {};
        EmitterPointClearMarkedCS_Pipeline emitterPointClearMarkedCS = {};
        EmitterPointClearDownsampleCS_Pipeline emitterPointClearDownsampleCS = {};

        VoxelInterpolatePrepassCS_Pipeline voxelInterpolatePrepassCS = {};
        VoxelInterpolateDownsampleCS_Pipeline voxelInterpolateDownsampleCS = {};
        VoxelInterpolatePropagateCS_Pipeline voxelInterpolatePropagateCS = {};
        VoxelInterpolateUpsampleCS_Pipeline voxelInterpolateUpsampleCS = {};
        VoxelInterpolateNullCS_Pipeline voxelInterpolateNullCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};
        NvFlowUploadBuffer subStepBuffer = {};

        NvFlowUploadBuffer voxelGlobalBuffer = {};
        NvFlowUploadBuffer voxelLayerBuffer = {};

        NvFlowDynamicBuffer oldValueBuffer = {};
        NvFlowDynamicBuffer keyLowBuffer = {};

        NvFlowDynamicBuffer reduceTarget1Buffer = {};
        NvFlowDynamicBuffer reduceTarget2Buffer = {};
        NvFlowDynamicBuffer reduceTarget3Buffer = {};
        NvFlowDynamicBuffer reduceCouple1Buffer = {};
        NvFlowDynamicBuffer reduceCouple2Buffer = {};
        NvFlowDynamicBuffer reduceCouple3Buffer = {};
        NvFlowDynamicBuffer reduceWeight1Buffer = {};
        NvFlowDynamicBuffer reduceWeight2Buffer = {};
        NvFlowDynamicBuffer reduceWeight3Buffer = {};

        NvFlowDynamicBuffer scanTarget1Buffer = {};
        NvFlowDynamicBuffer scanTarget2Buffer = {};
        NvFlowDynamicBuffer scanTarget3Buffer = {};
        NvFlowDynamicBuffer scanCouple1Buffer = {};
        NvFlowDynamicBuffer scanCouple2Buffer = {};
        NvFlowDynamicBuffer scanCouple3Buffer = {};
        NvFlowDynamicBuffer scanWeight1Buffer = {};
        NvFlowDynamicBuffer scanWeight2Buffer = {};
        NvFlowDynamicBuffer scanWeight3Buffer = {};

        NvFlowDynamicBuffer key1Buffer = {};
        NvFlowDynamicBuffer key2Buffer = {};
        NvFlowDynamicBuffer key3Buffer = {};

        NvFlowArrayBuffer arrayBuffer = {};

        NvFlowUint64 globalChangeVersion = 0llu;
        NvFlowUint64 stagedChangeVersion = 0llu;

        NvFlowUint64 stagedStreamingVersion = 0llu;

        NvFlowUint64 stagedLuidBeginIdx = 0llu;
        NvFlowUint64 stagedLuidEndIdx = 0llu;

        NvFlowUint64 globalUpdateVersion = 0llu;
        NvFlowBool32 shouldMark = NV_FLOW_FALSE;
        NvFlowBool32 shouldClearMarked = NV_FLOW_FALSE;
        NvFlowUint64 nextStagedLuidIdx = 0llu;

        NvFlowRadixSortInterface radixSortInterface = {};
        NvFlowRadixSort* radixSort = nullptr;

        NvFlowLuidTable luidTable;

#ifdef DEBUG_READBACK
        NvFlowReadbackBuffer readback1 = {};
        NvFlowReadbackBuffer readback2 = {};
        NvFlowReadbackBuffer readback3 = {};
#endif
    };

    EmitterPoint* EmitterPoint_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterPointPinsIn* in, NvFlowEmitterPointPinsOut* out)
    {
        auto ptr = new EmitterPoint();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EmitterPoint1CS_init(&ptr->contextInterface, in->context, &ptr->emitterPoint1CS);
        EmitterPoint2CS_init(&ptr->contextInterface, in->context, &ptr->emitterPoint2CS);
        EmitterPoint3CS_init(&ptr->contextInterface, in->context, &ptr->emitterPoint3CS);
        EmitterPoint4CS_init(&ptr->contextInterface, in->context, &ptr->emitterPoint4CS);
        EmitterPointReduce1CS_init(&ptr->contextInterface, in->context, &ptr->emitterPointReduce1CS);
        EmitterPointReduce2CS_init(&ptr->contextInterface, in->context, &ptr->emitterPointReduce2CS);
        EmitterPointReduce3CS_init(&ptr->contextInterface, in->context, &ptr->emitterPointReduce3CS);
        EmitterPointScan1CS_init(&ptr->contextInterface, in->context, &ptr->emitterPointScan1CS);
        EmitterPointScan2CS_init(&ptr->contextInterface, in->context, &ptr->emitterPointScan2CS);
        EmitterPointScan3CS_init(&ptr->contextInterface, in->context, &ptr->emitterPointScan3CS);
        EmitterPointMarkCS_init(&ptr->contextInterface, in->context, &ptr->emitterPointMarkCS);
        EmitterPointClearMarkedCS_init(&ptr->contextInterface, in->context, &ptr->emitterPointClearMarkedCS);
        EmitterPointClearDownsampleCS_init(&ptr->contextInterface, in->context, &ptr->emitterPointClearDownsampleCS);

        VoxelInterpolatePrepassCS_init(&ptr->contextInterface, in->context, &ptr->voxelInterpolatePrepassCS);
        VoxelInterpolateDownsampleCS_init(&ptr->contextInterface, in->context, &ptr->voxelInterpolateDownsampleCS);
        VoxelInterpolatePropagateCS_init(&ptr->contextInterface, in->context, &ptr->voxelInterpolatePropagateCS);
        VoxelInterpolateUpsampleCS_init(&ptr->contextInterface, in->context, &ptr->voxelInterpolateUpsampleCS);
        VoxelInterpolateNullCS_init(&ptr->contextInterface, in->context, &ptr->voxelInterpolateNullCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->subStepBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->voxelGlobalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->voxelLayerBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(VoxelInterpolateCS_LayerParams));

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->oldValueBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint2));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->keyLowBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceTarget1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceTarget2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceTarget3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceCouple1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceCouple2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceCouple3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceWeight1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(float));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceWeight2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(float));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->reduceWeight3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(float));

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanTarget1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanTarget2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanTarget3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanCouple1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanCouple2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanCouple3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanWeight1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(float));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanWeight2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(float));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->scanWeight3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(float));

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->key1Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint2));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->key2Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint2));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->key3Buffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint2));

        NvFlowArrayBuffer_init(&ptr->contextInterface, in->context, &ptr->arrayBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        NvFlowRadixSortInterface_duplicate(&ptr->radixSortInterface, NvFlowGetRadixSortInterface());
        ptr->radixSort = ptr->radixSortInterface.create(&ptr->contextInterface, in->context);

#ifdef DEBUG_READBACK
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback1);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback2);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->readback3);
#endif

        return ptr;
    }

    void EmitterPoint_destroy(EmitterPoint* ptr, const NvFlowEmitterPointPinsIn* in, NvFlowEmitterPointPinsOut* out)
    {
#ifdef DEBUG_READBACK
        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback1);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback2);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->readback3);
#endif

        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->subStepBuffer);

        NvFlowUploadBuffer_destroy(in->context, &ptr->voxelGlobalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->voxelLayerBuffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->oldValueBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->keyLowBuffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceTarget1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceTarget2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceTarget3Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceCouple1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceCouple2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceCouple3Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceWeight1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceWeight2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->reduceWeight3Buffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanTarget1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanTarget2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanTarget3Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanCouple1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanCouple2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanCouple3Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanWeight1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanWeight2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->scanWeight3Buffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->key1Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->key2Buffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->key3Buffer);

        NvFlowArrayBuffer_destroy(in->context, &ptr->arrayBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        EmitterPoint1CS_destroy(in->context, &ptr->emitterPoint1CS);
        EmitterPoint2CS_destroy(in->context, &ptr->emitterPoint2CS);
        EmitterPoint3CS_destroy(in->context, &ptr->emitterPoint3CS);
        EmitterPoint4CS_destroy(in->context, &ptr->emitterPoint4CS);
        EmitterPointReduce1CS_destroy(in->context, &ptr->emitterPointReduce1CS);
        EmitterPointReduce2CS_destroy(in->context, &ptr->emitterPointReduce2CS);
        EmitterPointReduce3CS_destroy(in->context, &ptr->emitterPointReduce3CS);
        EmitterPointScan1CS_destroy(in->context, &ptr->emitterPointScan1CS);
        EmitterPointScan2CS_destroy(in->context, &ptr->emitterPointScan2CS);
        EmitterPointScan3CS_destroy(in->context, &ptr->emitterPointScan3CS);
        EmitterPointMarkCS_destroy(in->context, &ptr->emitterPointMarkCS);
        EmitterPointClearMarkedCS_destroy(in->context, &ptr->emitterPointClearMarkedCS);
        EmitterPointClearDownsampleCS_destroy(in->context, &ptr->emitterPointClearDownsampleCS);

        VoxelInterpolatePrepassCS_destroy(in->context, &ptr->voxelInterpolatePrepassCS);
        VoxelInterpolateDownsampleCS_destroy(in->context, &ptr->voxelInterpolateDownsampleCS);
        VoxelInterpolatePropagateCS_destroy(in->context, &ptr->voxelInterpolatePropagateCS);
        VoxelInterpolateUpsampleCS_destroy(in->context, &ptr->voxelInterpolateUpsampleCS);
        VoxelInterpolateNullCS_destroy(in->context, &ptr->voxelInterpolateNullCS);

        ptr->radixSortInterface.destroy(in->context, ptr->radixSort);

        delete ptr;
    }

    void EmitterPoint_executeSingleEmitter(
        EmitterPoint* ptr,
        const NvFlowEmitterPointPinsIn* in,
        NvFlowEmitterPointPinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        NvFlowSparseLevelParams* coarseLevelParams,
        const NvFlowEmitterPointParams* params,
        NvFlowBool32 isVelocity,
        NvFlowBool32 anyStreamingClearEnabled
    )
    {
        using namespace NvFlowMath;

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
        if (params->pointPositionCount == 0u)
        {
            return;
        }

        // Early out if couple rates leave emitter having zero effect, only with defaults
        if (!isVelocity &&
            params->pointCoupleRateTemperatureCount == 0u &&
            params->pointCoupleRateFuelCount == 0u &&
            params->pointCoupleRateBurnCount == 0u &&
            params->pointCoupleRateSmokeCount == 0u &&
            params->coupleRateTemperature <= 0.f &&
            params->coupleRateFuel <= 0.f &&
            params->coupleRateBurn <= 0.f &&
            params->coupleRateSmoke <= 0.f)
        {
            return;
        }
        if (isVelocity &&
            params->pointCoupleRateVelocityCount == 0u &&
            params->pointCoupleRateDivergenceCount == 0u &&
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

        NvFlowArrayBufferData arrayDatas[eArray_count] = { };
        arrayDatas[eArray_positions] = { params->pointPositions, 3 * params->pointPositionCount, /*params->pointPositionVersion*/ 0llu };
        arrayDatas[eArray_velocities] = { params->pointVelocities, 3 * params->pointVelocityCount, /*params->pointVelocityVersion*/ 0llu };
        arrayDatas[eArray_colors] = { params->pointColors, 3 * params->pointColorCount, /*params->pointColorVersion*/ 0llu };
        arrayDatas[eArray_divergences] = { params->pointDivergences, params->pointDivergenceCount, /*params->pointDivergenceVersion*/ 0llu };
        arrayDatas[eArray_temperatures] = { params->pointTemperatures, params->pointTemperatureCount, /*params->pointTemperatureVersion*/ 0llu };
        arrayDatas[eArray_fuels] = { params->pointFuels, params->pointFuelCount, /*params->pointFuelVersion*/ 0llu };
        arrayDatas[eArray_burns] = { params->pointBurns, params->pointBurnCount, /*params->pointBurnVersion*/ 0llu };
        arrayDatas[eArray_smokes] = { params->pointSmokes, params->pointSmokeCount, /*params->pointSmokeVersion*/ 0llu };
        arrayDatas[eArray_coupleRateVelocities] = { params->pointCoupleRateVelocities, params->pointCoupleRateVelocityCount, /*params->pointCoupleRateVelocityVersion*/ 0llu };
        arrayDatas[eArray_coupleRateDivergences] = { params->pointCoupleRateDivergences, params->pointCoupleRateDivergenceCount, /*params->pointCoupleRateDivergenceVersion*/ 0llu };
        arrayDatas[eArray_coupleRateTemperatures] = { params->pointCoupleRateTemperatures, params->pointCoupleRateTemperatureCount, /*params->pointCoupleRateTemperatureVersion*/ 0llu };
        arrayDatas[eArray_coupleRateFuels] = { params->pointCoupleRateFuels, params->pointCoupleRateFuelCount, /*params->pointCoupleRateFuelVersion*/ 0llu };
        arrayDatas[eArray_coupleRateBurns] = { params->pointCoupleRateBurns, params->pointCoupleRateBurnCount, /*params->pointCoupleRateBurnVersion*/ 0llu };
        arrayDatas[eArray_coupleRateSmokes] = { params->pointCoupleRateSmokes, params->pointCoupleRateSmokeCount, /*params->pointCoupleRateSmokeVersion*/ 0llu };
        NvFlowUint64 firstElements[eArray_count] = {};

        NvFlowBufferTransient* arrayTransient = NvFlowArrayBuffer_update(in->context, &ptr->arrayBuffer, params->luid, arrayDatas, firstElements, eArray_count, nullptr, "EmitterPointUpload");

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

        NvFlowUint voxelsPerPoint = params->voxelsPerPoint;
        if (voxelsPerPoint == 0u || voxelsPerPoint >= 8u)
        {
            voxelsPerPoint = 8u;
        }

        NvFlowUint64 pointBlockCount = (voxelsPerPoint * params->pointPositionCount + 127u) / 128u;
        NvFlowUint reduceCount1 = (NvFlowUint)pointBlockCount;
        NvFlowUint reduceCount2 = (reduceCount1 + 127u) / 128u;
        NvFlowUint reduceCount3 = (reduceCount2 + 127u) / 128u;

        NvFlowBool32 texIsSrgb = NV_FLOW_FALSE;
        if (in->value.format == eNvFlowFormat_r8g8b8a8_unorm)
        {
            if (ptr->contextInterface.isFeatureSupported(in->context, eNvFlowContextFeature_aliasResourceFormats))
            {
                texIsSrgb = NV_FLOW_TRUE;
            }
        }

        NvFlowDispatchBatches pointBatches;
        NvFlowDispatchBatches_init(&pointBatches, (NvFlowUint)pointBlockCount);
        NvFlowDispatchBatches reduce2Batches;
        NvFlowDispatchBatches_init(&reduce2Batches, (NvFlowUint)reduceCount2);
        NvFlowDispatchBatches reduce3Batches;
        NvFlowDispatchBatches_init(&reduce3Batches, (NvFlowUint)reduceCount3);

        NvFlowUint blockDimBits3 = levelParams->blockDimBits.x + levelParams->blockDimBits.y + levelParams->blockDimBits.z;
        NvFlowUint keyMaxBlocks = 1u << (32u - blockDimBits3);
        NvFlowBool32 keysAreUnique = levelParams->numLocations < keyMaxBlocks;

        for (NvFlowUint64 batchIdx = 0u; batchIdx < pointBatches.size; batchIdx++)
        {
            auto mapped = (EmitterPointCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterPointCS_Params));

            mapped->pointBlockIdxOffset = pointBatches[batchIdx].blockIdxOffset;
            mapped->reduce2BlockIdxOffset = batchIdx < reduce2Batches.size ? reduce2Batches[batchIdx].blockIdxOffset : 0u;
            mapped->reduce3BlockIdxOffset = batchIdx < reduce3Batches.size ? reduce3Batches[batchIdx].blockIdxOffset : 0u;
            mapped->velocityIsWorldSpace = params->velocityIsWorldSpace;

            mapped->table = *levelParams;
            mapped->tableCoarse = *coarseLevelParams;

            mapped->localToWorld = matrixTranspose(params->localToWorld);
            mapped->localToWorldVelocity = matrixTranspose(params->localToWorldVelocity);

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

            mapped->layerAndLevel = NvFlow_packLayerAndLevel(params->layer, params->level);
            mapped->defaultAllocateMask = params->allocateMask;
            mapped->allocateMaskCount = (NvFlowUint)params->pointAllocateMaskCount;
            mapped->needsSrgbConversion = params->colorIsSrgb && !texIsSrgb;

            mapped->vidxToWorld = vidxToWorld;
            mapped->deltaTime = 0.5f * subStepDeltaTime;

            mapped->worldToVidx = worldToVidx;
            mapped->isVelocity = isVelocity;

            mapped->range_positions = { (NvFlowUint)firstElements[eArray_positions], (NvFlowUint)params->pointPositionCount };
            mapped->range_velocities = { (NvFlowUint)firstElements[eArray_velocities], (NvFlowUint)params->pointVelocityCount };
            mapped->range_divergences = { (NvFlowUint)firstElements[eArray_divergences], (NvFlowUint)params->pointDivergenceCount };
            mapped->range_temperatures = { (NvFlowUint)firstElements[eArray_temperatures], (NvFlowUint)params->pointTemperatureCount };

            mapped->range_fuels = { (NvFlowUint)firstElements[eArray_fuels], (NvFlowUint)params->pointFuelCount };
            mapped->range_burns = { (NvFlowUint)firstElements[eArray_burns], (NvFlowUint)params->pointBurnCount };
            mapped->range_smokes = { (NvFlowUint)firstElements[eArray_smokes], (NvFlowUint)params->pointSmokeCount };
            mapped->range_coupleRateVelocities = { (NvFlowUint)firstElements[eArray_coupleRateVelocities], (NvFlowUint)params->pointCoupleRateVelocityCount };

            mapped->range_coupleRateDivergences = { (NvFlowUint)firstElements[eArray_coupleRateDivergences], (NvFlowUint)params->pointCoupleRateDivergenceCount };
            mapped->range_coupleRateTemperatures = { (NvFlowUint)firstElements[eArray_coupleRateTemperatures], (NvFlowUint)params->pointCoupleRateTemperatureCount };
            mapped->range_coupleRateFuels = { (NvFlowUint)firstElements[eArray_coupleRateFuels], (NvFlowUint)params->pointCoupleRateFuelCount };
            mapped->range_coupleRateBurns = { (NvFlowUint)firstElements[eArray_coupleRateBurns], (NvFlowUint)params->pointCoupleRateBurnCount };

            mapped->range_coupleRateSmokes = { (NvFlowUint)firstElements[eArray_coupleRateSmokes], (NvFlowUint)params->pointCoupleRateSmokeCount };
            mapped->range_colors = { (NvFlowUint)firstElements[eArray_colors], (NvFlowUint)params->pointColorCount };

            mapped->reduceCount1 = reduceCount1;
            mapped->reduceCount2 = reduceCount2;
            mapped->reduceCount3 = reduceCount3;
            mapped->enableInterpolation = params->enableInterpolation;
            mapped->voxelsPerPoint = voxelsPerPoint;

            NvFlowFloat3 densityBlockDimf = {
                float(levelParams->blockDimLessOne.x + 1u),
                float(levelParams->blockDimLessOne.y + 1u),
                float(levelParams->blockDimLessOne.z + 1u)
            };
            NvFlowFloat3 velocityBlockDimf = {
                float(coarseLevelParams->blockDimLessOne.x + 1u),
                float(coarseLevelParams->blockDimLessOne.y + 1u),
                float(coarseLevelParams->blockDimLessOne.z + 1u)
            };
            NvFlowFloat3 velocityToDensityBlockScale = {
                densityBlockDimf.x / velocityBlockDimf.x,
                densityBlockDimf.y / velocityBlockDimf.y,
                densityBlockDimf.z / velocityBlockDimf.z,
            };
            mapped->velocityToDensityBlockScale = velocityToDensityBlockScale;
            mapped->anyStreamingClearEnabled = anyStreamingClearEnabled;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);
            pointBatches[batchIdx].globalTransient = constantTransient;
            if (batchIdx < reduce2Batches.size)
            {
                reduce2Batches[batchIdx].globalTransient = constantTransient;
            }
            if (batchIdx < reduce3Batches.size)
            {
                reduce3Batches[batchIdx].globalTransient = constantTransient;
            }
        }

        NvFlowDynamicBuffer_resize(in->context, &ptr->oldValueBuffer, (voxelsPerPoint * params->pointPositionCount) * sizeof(NvFlowFloat4));
        NvFlowBufferTransient* oldValueBuffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->oldValueBuffer);
        NvFlowDynamicBuffer_resize(in->context, &ptr->keyLowBuffer, (voxelsPerPoint * params->pointPositionCount) * sizeof(NvFlowUint));
        NvFlowBufferTransient* keyLowBuffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->keyLowBuffer);

        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceTarget1Buffer, reduceCount1 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceTarget2Buffer, reduceCount2 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceTarget3Buffer, reduceCount3 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceCouple1Buffer, reduceCount1 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceCouple2Buffer, reduceCount2 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceCouple3Buffer, reduceCount3 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceWeight1Buffer, reduceCount1 * sizeof(float));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceWeight2Buffer, reduceCount2 * sizeof(float));
        NvFlowDynamicBuffer_resize(in->context, &ptr->reduceWeight3Buffer, reduceCount3 * sizeof(float));

        NvFlowDynamicBuffer_resize(in->context, &ptr->scanTarget1Buffer, reduceCount1 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanTarget2Buffer, reduceCount2 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanTarget3Buffer, reduceCount3 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanCouple1Buffer, reduceCount1 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanCouple2Buffer, reduceCount2 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanCouple3Buffer, reduceCount3 * sizeof(NvFlowFloat4));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanWeight1Buffer, reduceCount1 * sizeof(float));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanWeight2Buffer, reduceCount2 * sizeof(float));
        NvFlowDynamicBuffer_resize(in->context, &ptr->scanWeight3Buffer, reduceCount3 * sizeof(float));

        NvFlowDynamicBuffer_resize(in->context, &ptr->key1Buffer, reduceCount1 * sizeof(NvFlowUint2));
        NvFlowDynamicBuffer_resize(in->context, &ptr->key2Buffer, reduceCount2 * sizeof(NvFlowUint2));
        NvFlowDynamicBuffer_resize(in->context, &ptr->key3Buffer, reduceCount3 * sizeof(NvFlowUint2));

        NvFlowBufferTransient* reduceTarget1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceTarget1Buffer);
        NvFlowBufferTransient* reduceTarget2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceTarget2Buffer);
        NvFlowBufferTransient* reduceTarget3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceTarget3Buffer);
        NvFlowBufferTransient* reduceCouple1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceCouple1Buffer);
        NvFlowBufferTransient* reduceCouple2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceCouple2Buffer);
        NvFlowBufferTransient* reduceCouple3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceCouple3Buffer);
        NvFlowBufferTransient* reduceWeight1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceWeight1Buffer);
        NvFlowBufferTransient* reduceWeight2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceWeight2Buffer);
        NvFlowBufferTransient* reduceWeight3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->reduceWeight3Buffer);

        NvFlowBufferTransient* scanTarget1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanTarget1Buffer);
        NvFlowBufferTransient* scanTarget2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanTarget2Buffer);
        NvFlowBufferTransient* scanTarget3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanTarget3Buffer);
        NvFlowBufferTransient* scanCouple1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanCouple1Buffer);
        NvFlowBufferTransient* scanCouple2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanCouple2Buffer);
        NvFlowBufferTransient* scanCouple3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanCouple3Buffer);
        NvFlowBufferTransient* scanWeight1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanWeight1Buffer);
        NvFlowBufferTransient* scanWeight2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanWeight2Buffer);
        NvFlowBufferTransient* scanWeight3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->scanWeight3Buffer);

        NvFlowBufferTransient* key1Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->key1Buffer);
        NvFlowBufferTransient* key2Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->key2Buffer);
        NvFlowBufferTransient* key3Buffer = NvFlowDynamicBuffer_getTransient(in->context, &ptr->key3Buffer);

        ptr->radixSortInterface.reserve(in->context, ptr->radixSort, (NvFlowUint)(voxelsPerPoint * params->pointPositionCount));

        for (NvFlowUint subStepIdx = numSubSteps - 1u; subStepIdx < numSubSteps; subStepIdx--)
        {
            auto mapped = (EmitterPointCS_SubStepParams*)NvFlowUploadBuffer_map(in->context, &ptr->subStepBuffer, sizeof(EmitterPointCS_SubStepParams));

            mapped->subStepIdx = subStepIdx;
            mapped->numSubSteps = numSubSteps;
            mapped->subStepDeltaTime = in->deltaTime / ((float)numSubSteps);
            mapped->subStepAccumDeltaTime = ((float)subStepIdx) * in->deltaTime / ((float)numSubSteps);
            mapped->defaultVelocity = params->velocity;
            mapped->totalDeltaTime = in->deltaTime;
            mapped->velocityScale = params->velocityScale;
            mapped->pad1 = 0.f;
            mapped->pad2 = 0.f;
            mapped->pad3 = 0.f;

            NvFlowBufferTransient* subStepTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->subStepBuffer);

            NvFlowBufferTransient* keyBuffer = nullptr;
            NvFlowBufferTransient* valBuffer = nullptr;

            NvFlowUint blockDimBits3 = levelParams->blockDimBits.x + levelParams->blockDimBits.y + levelParams->blockDimBits.z;
            NvFlowUint numLocationsBits = 1u; // always sort at least one bit
            while ((1u << numLocationsBits) < levelParams->numLocations)
            {
                numLocationsBits++;
            }

            // sort by threadIdx1D
            ptr->radixSortInterface.getInputBuffers(in->context, ptr->radixSort, &keyBuffer, &valBuffer);
            for (NvFlowUint pointBatchIdx = 0u; pointBatchIdx < pointBatches.size; pointBatchIdx++)
            {
                EmitterPoint1CS_PassParams passParams = {};
                passParams.gParams = pointBatches[pointBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.valueIn = in->value.textureTransient;
                passParams.oldValueOut = oldValueBuffer;
                passParams.keyOut = keyBuffer;
                passParams.valOut = valBuffer;
                passParams.keyLowOut = keyLowBuffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = pointBatches[pointBatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPoint1CS_addPassCompute(in->context, &ptr->emitterPoint1CS, gridDim, &passParams);
            }

#ifdef DEBUG_READBACK
            //if (!isVelocity)
            //{
            //    NvFlowReadbackBuffer_copy(in->context, &ptr->readback, voxelsPerPoint * params->pointPositionCount * sizeof(NvFlowUint), keyBuffer, nullptr);
            //}
            //NvFlowUint* readbackMapped = (NvFlowUint*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback, nullptr, nullptr);
#endif

            ptr->radixSortInterface.sort(in->context, ptr->radixSort, (NvFlowUint)(voxelsPerPoint * params->pointPositionCount), blockDimBits3);
            ptr->radixSortInterface.getOutputBuffers(in->context, ptr->radixSort, &keyBuffer, &valBuffer);

            // sort by blockIdx
            ptr->radixSortInterface.getInputBuffers(in->context, ptr->radixSort, &keyBuffer, &valBuffer);
            for (NvFlowUint pointBatchIdx = 0u; pointBatchIdx < pointBatches.size; pointBatchIdx++)
            {
                EmitterPoint2CS_PassParams passParams = {};
                passParams.gParams = pointBatches[pointBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.keyInOut = keyBuffer;
                passParams.valInOut = valBuffer;
                passParams.keyLowInOut = keyLowBuffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = pointBatches[pointBatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPoint2CS_addPassCompute(in->context, &ptr->emitterPoint2CS, gridDim, &passParams);
            }
            ptr->radixSortInterface.sort(in->context, ptr->radixSort, (NvFlowUint)(voxelsPerPoint * params->pointPositionCount), numLocationsBits);
            ptr->radixSortInterface.getOutputBuffers(in->context, ptr->radixSort, &keyBuffer, &valBuffer);

            for (NvFlowUint pointBatchIdx = 0u; pointBatchIdx < pointBatches.size; pointBatchIdx++)
            {
                EmitterPointReduce1CS_PassParams passParams = {};
                passParams.gParams = pointBatches[pointBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.keyLowIn = keyLowBuffer;
                passParams.keyIn = keyBuffer;
                passParams.valIn = valBuffer;
                passParams.reduceTarget1Out = reduceTarget1Buffer;
                passParams.reduceCouple1Out = reduceCouple1Buffer;
                passParams.reduceWeight1Out = reduceWeight1Buffer;
                passParams.key1Out = key1Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = pointBatches[pointBatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPointReduce1CS_addPassCompute(in->context, &ptr->emitterPointReduce1CS, gridDim, &passParams);
            }
            for (NvFlowUint reduce2BatchIdx = 0u; reduce2BatchIdx < reduce2Batches.size; reduce2BatchIdx++)
            {
                EmitterPointReduce2CS_PassParams passParams = {};
                passParams.gParams = reduce2Batches[reduce2BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.reduceTarget1In = reduceTarget1Buffer;
                passParams.reduceCouple1In = reduceCouple1Buffer;
                passParams.reduceWeight1In = reduceWeight1Buffer;
                passParams.key1In = key1Buffer;
                passParams.reduceTarget2Out = reduceTarget2Buffer;
                passParams.reduceCouple2Out = reduceCouple2Buffer;
                passParams.reduceWeight2Out = reduceWeight2Buffer;
                passParams.key2Out = key2Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = reduce2Batches[reduce2BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPointReduce2CS_addPassCompute(in->context, &ptr->emitterPointReduce2CS, gridDim, &passParams);
            }
            for (NvFlowUint reduce3BatchIdx = 0u; reduce3BatchIdx < reduce3Batches.size; reduce3BatchIdx++)
            {
                EmitterPointReduce3CS_PassParams passParams = {};
                passParams.gParams = reduce3Batches[reduce3BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.reduceTarget2In = reduceTarget2Buffer;
                passParams.reduceCouple2In = reduceCouple2Buffer;
                passParams.reduceWeight2In = reduceWeight2Buffer;
                passParams.key2In = key2Buffer;
                passParams.reduceTarget3Out = reduceTarget3Buffer;
                passParams.reduceCouple3Out = reduceCouple3Buffer;
                passParams.reduceWeight3Out = reduceWeight3Buffer;
                passParams.key3Out = key3Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = reduce3Batches[reduce3BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPointReduce3CS_addPassCompute(in->context, &ptr->emitterPointReduce3CS, gridDim, &passParams);
            }
            if (reduce3Batches.size > 0u)
            {
                EmitterPointScan3CS_PassParams passParams = {};
                passParams.gParams = reduce3Batches[0u].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.reduceTarget3In = reduceTarget3Buffer;
                passParams.reduceCouple3In = reduceCouple3Buffer;
                passParams.reduceWeight3In = reduceWeight3Buffer;
                passParams.key3In = key3Buffer;
                passParams.scanTarget3Out = scanTarget3Buffer;
                passParams.scanCouple3Out = scanCouple3Buffer;
                passParams.scanWeight3Out = scanWeight3Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = 1u;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPointScan3CS_addPassCompute(in->context, &ptr->emitterPointScan3CS, gridDim, &passParams);
            }
            for (NvFlowUint reduce3BatchIdx = 0u; reduce3BatchIdx < reduce3Batches.size; reduce3BatchIdx++)
            {
                EmitterPointScan2CS_PassParams passParams = {};
                passParams.gParams = reduce3Batches[reduce3BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.reduceTarget2In = reduceTarget2Buffer;
                passParams.reduceCouple2In = reduceCouple2Buffer;
                passParams.reduceWeight2In = reduceWeight2Buffer;
                passParams.key2In = key2Buffer;
                passParams.scanTarget3In = scanTarget3Buffer;
                passParams.scanCouple3In = scanCouple3Buffer;
                passParams.scanWeight3In = scanWeight3Buffer;
                passParams.key3In = key3Buffer;
                passParams.scanTarget2Out = scanTarget2Buffer;
                passParams.scanCouple2Out = scanCouple2Buffer;
                passParams.scanWeight2Out = scanWeight2Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = reduce3Batches[reduce3BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPointScan2CS_addPassCompute(in->context, &ptr->emitterPointScan2CS, gridDim, &passParams);
            }
            for (NvFlowUint reduce2BatchIdx = 0u; reduce2BatchIdx < reduce2Batches.size; reduce2BatchIdx++)
            {
                EmitterPointScan1CS_PassParams passParams = {};
                passParams.gParams = reduce3Batches[reduce2BatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.reduceTarget1In = reduceTarget1Buffer;
                passParams.reduceCouple1In = reduceCouple1Buffer;
                passParams.reduceWeight1In = reduceWeight1Buffer;
                passParams.key1In = key1Buffer;
                passParams.scanTarget2In = scanTarget2Buffer;
                passParams.scanCouple2In = scanCouple2Buffer;
                passParams.scanWeight2In = scanWeight2Buffer;
                passParams.key2In = key2Buffer;
                passParams.scanTarget1Out = scanTarget1Buffer;
                passParams.scanCouple1Out = scanCouple1Buffer;
                passParams.scanWeight1Out = scanWeight1Buffer;

                NvFlowUint3 gridDim = {};
                gridDim.x = reduce2Batches[reduce2BatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPointScan1CS_addPassCompute(in->context, &ptr->emitterPointScan1CS, gridDim, &passParams);
            }

            for (NvFlowUint pointBatchIdx = 0u; pointBatchIdx < pointBatches.size; pointBatchIdx++)
            {
                EmitterPoint3CS_PassParams passParams = {};
                passParams.gParams = pointBatches[pointBatchIdx].globalTransient;
                passParams.gSubStepParams = subStepTransient;
                passParams.gTable = in->value.sparseBuffer;
                passParams.arrayValuesIn = arrayTransient;
                passParams.oldValueIn = oldValueBuffer;
                passParams.keyLowIn = keyLowBuffer;
                passParams.keyIn = keyBuffer;
                passParams.valIn = valBuffer;
                passParams.scanTarget1In = scanTarget1Buffer;
                passParams.scanCouple1In = scanCouple1Buffer;
                passParams.scanWeight1In = scanWeight1Buffer;
                passParams.key1In = key1Buffer;
                passParams.valueOut = in->value.textureTransient;
                passParams.voxelWeightOut = in->voxelWeight.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = pointBatches[pointBatchIdx].blockCount;
                gridDim.y = 1u;
                gridDim.z = 1u;

                EmitterPoint3CS_addPassCompute(in->context, &ptr->emitterPoint3CS, gridDim, &passParams);
            }

#ifdef DEBUG_READBACK
            if (!isVelocity)
            {
                NvFlowReadbackBuffer_copy(in->context, &ptr->readback1, reduceCount1 * sizeof(float), scanWeight1Buffer, nullptr);
                NvFlowReadbackBuffer_copy(in->context, &ptr->readback2, reduceCount2 * sizeof(float), scanWeight2Buffer, nullptr);
                NvFlowReadbackBuffer_copy(in->context, &ptr->readback3, reduceCount3 * sizeof(float), scanWeight3Buffer, nullptr);
            }
            float* readbackMapped1 = (float*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback1, nullptr, nullptr);
            float* readbackMapped2 = (float*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback2, nullptr, nullptr);
            float* readbackMapped3 = (float*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->readback3, nullptr, nullptr);
#endif

            if (in->coarseDensity.textureTransient && params->updateCoarseDensity)
            {
                for (NvFlowUint pointBatchIdx = 0u; pointBatchIdx < pointBatches.size; pointBatchIdx++)
                {
                    EmitterPoint4CS_PassParams passParams = {};
                    passParams.gParams = pointBatches[pointBatchIdx].globalTransient;
                    passParams.gSubStepParams = subStepTransient;
                    passParams.gTable = in->value.sparseBuffer;
                    passParams.arrayValuesIn = arrayTransient;
                    passParams.keyIn = keyBuffer;
                    passParams.valIn = valBuffer;
                    passParams.keyLowIn = keyLowBuffer;
                    passParams.valueIn = in->value.textureTransient;
                    passParams.valueSampler = ptr->samplerLinear;
                    passParams.coarseValueOut = in->coarseDensity.textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = pointBatches[pointBatchIdx].blockCount;
                    gridDim.y = 1u;
                    gridDim.z = 1u;

                    EmitterPoint4CS_addPassCompute(in->context, &ptr->emitterPoint4CS, gridDim, &passParams);
                }
            }
        }
    }

    void EmitterPoint_voxelInterpolate(
        EmitterPoint* ptr,
        const NvFlowEmitterPointPinsIn* in,
        NvFlowEmitterPointPinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        EmitterPointFeedbackInterface* feedback,
        NvFlowBool32 anyEmit
    );

    void EmitterPoint_execute(EmitterPoint* ptr, const NvFlowEmitterPointPinsIn* in, NvFlowEmitterPointPinsOut* out)
    {
        out->streamingVersion = in->streamingVersion;
        out->streamingFinishedVersion = in->streamingFinishedVersion;
        out->streamingPointsVersion = in->streamingPointsVersion;

        NvFlowSparseLevelParams* levelParams = &in->value.sparseParams.levels[in->value.levelIdx];
        NvFlowSparseLevelParams* coarseLevelParams = levelParams;
        if (in->coarseDensity.textureTransient)
        {
            coarseLevelParams = &in->coarseDensity.sparseParams.levels[in->coarseDensity.levelIdx];
        }

        // passthrough, since input is mutable
        NvFlowSparseTexture_passThrough(&out->value, &in->value);
        NvFlowSparseTexture_passThrough(&out->voxelWeight, &in->voxelWeight);

        EmitterPointFeedbackInterface* feedback = (EmitterPointFeedbackInterface*)in->feedback.data;
        if (!feedback)
        {
            return;
        }

        NvFlowBool32 anyEmit = NV_FLOW_FALSE;

        // process advancement logic once per frame
        if (ptr->globalUpdateVersion != feedback->globalUpdateVersion)
        {
            ptr->globalUpdateVersion = feedback->globalUpdateVersion;

            ptr->shouldMark = NV_FLOW_FALSE;
            if (ptr->globalChangeVersion != feedback->globalChangeVersion)
            {
                if (ptr->stagedChangeVersion != feedback->globalChangeVersion)
                {
                    ptr->stagedChangeVersion = feedback->globalChangeVersion;

                    // bump streaming version
                    out->streamingVersion = in->streamingVersion + 1u;

                    ptr->shouldMark = NV_FLOW_TRUE;
                    ptr->nextStagedLuidIdx = 0llu;
                }
            }
            // if external change, need to restart streaming
            if (ptr->stagedStreamingVersion != out->streamingVersion)
            {
                ptr->stagedStreamingVersion = out->streamingVersion;
                ptr->nextStagedLuidIdx = 0llu;
            }

            NvFlowUint64 pointsThisFrame = 0llu;
            NvFlowUint64 pointBudget = 0llu;
            ptr->stagedLuidBeginIdx = ptr->nextStagedLuidIdx;
            ptr->stagedLuidEndIdx = ptr->nextStagedLuidIdx;
            while (ptr->nextStagedLuidIdx < feedback->paramCount && (pointsThisFrame == 0llu || pointsThisFrame < pointBudget))
            {
                const NvFlowEmitterPointParams* localParams = feedback->params[ptr->nextStagedLuidIdx];
                if (localParams->enabled && localParams->enableStreaming)
                {
                    pointsThisFrame += localParams->pointPositionCount;
                    pointBudget = localParams->streamingBatchSize;
                }
                ptr->stagedLuidEndIdx++;
                ptr->nextStagedLuidIdx++;
                // Rate limit for now to avoid too many compute shader dispatches
                if (ptr->stagedLuidEndIdx - ptr->stagedLuidBeginIdx >= 8u)
                {
                    break;
                }
            }

            if (ptr->globalChangeVersion != feedback->globalChangeVersion ||
                out->streamingPointsVersion != out->streamingVersion)
            {
                if (ptr->nextStagedLuidIdx >= feedback->luidCount)
                {
                    ptr->globalChangeVersion = ptr->stagedChangeVersion;
                    out->streamingPointsVersion = ptr->stagedStreamingVersion;
                }
            }

            ptr->shouldClearMarked = NV_FLOW_FALSE;
            if (out->streamingFinishedVersion != out->streamingVersion)
            {
                bool allFinished =
                    out->streamingPointsVersion == out->streamingVersion &&
                    in->streamingNanoVdbVersion == out->streamingVersion;
                if (allFinished)
                {
                    out->streamingFinishedVersion = out->streamingVersion;
                    ptr->shouldClearMarked = NV_FLOW_TRUE;
                }
            }

            if (feedback->reportUpdate)
            {
                feedback->reportUpdate(feedback->userdata, ptr->globalChangeVersion, ptr->stagedChangeVersion);
            }
        }

        if (feedback->anyStreamingClearEnabled && ptr->shouldMark)
        {
            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mapped = (EmitterPointMarkCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterPointMarkCS_Params));

                mapped->table = *levelParams;
                mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;

                batches[batchIdx].globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);
            }

            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                EmitterPointMarkCS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.tableIn = in->value.sparseBuffer;
                params.voxelWeightOut = in->voxelWeight.textureTransient;

                EmitterPointMarkCS_addPassCompute(in->context, &ptr->emitterPointMarkCS, gridDim, &params);
            }
        }
        // continue accumulation until complete
        if (feedback->anyStreamingEnabled)
        {
            for (NvFlowUint64 paramIdx = ptr->stagedLuidBeginIdx; paramIdx < ptr->stagedLuidEndIdx; paramIdx++)
            {
                if (paramIdx < feedback->paramCount)
                {
                    NvFlowBool32 isVelocity = in->velocityParamCount > 0u;
                    const NvFlowEmitterPointParams* params = feedback->params[paramIdx];
                    if (params->enableStreaming)
                    {
                        EmitterPoint_executeSingleEmitter(ptr, in, out, levelParams, coarseLevelParams, params, isVelocity, feedback->anyStreamingClearEnabled);
                        anyEmit = NV_FLOW_TRUE;
                    }
                }
            }
        }
        if (feedback->anyStreamingClearEnabled && ptr->shouldClearMarked)
        {
            // clear marked texture entries
            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mapped = (EmitterPointMarkCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterPointMarkCS_Params));

                NvFlowFloat3 densityBlockDimf = {
                    float(levelParams->blockDimLessOne.x + 1u),
                    float(levelParams->blockDimLessOne.y + 1u),
                    float(levelParams->blockDimLessOne.z + 1u)
                };
                NvFlowFloat3 velocityBlockDimf = {
                    float(coarseLevelParams->blockDimLessOne.x + 1u),
                    float(coarseLevelParams->blockDimLessOne.y + 1u),
                    float(coarseLevelParams->blockDimLessOne.z + 1u)
                };
                NvFlowFloat3 velocityToDensityBlockScale = {
                    densityBlockDimf.x / velocityBlockDimf.x,
                    densityBlockDimf.y / velocityBlockDimf.y,
                    densityBlockDimf.z / velocityBlockDimf.z,
                };

                mapped->table = *levelParams;
                mapped->tableCoarse = *coarseLevelParams;
                mapped->velocityToDensityBlockScale = velocityToDensityBlockScale;
                mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;

                batches[batchIdx].globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);
            }

            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                EmitterPointClearMarkedCS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.tableIn = in->value.sparseBuffer;
                params.voxelWeightIn = in->voxelWeight.textureTransient;
                params.valueOut = in->value.textureTransient;

                EmitterPointClearMarkedCS_addPassCompute(in->context, &ptr->emitterPointClearMarkedCS, gridDim, &params);
            }

            NvFlowBool32 anyUpdateCoarseDensity = NV_FLOW_FALSE;
            for (NvFlowUint64 paramIdx = 0u; paramIdx < feedback->paramCount; paramIdx++)
            {
                if (feedback->params[paramIdx]->updateCoarseDensity)
                {
                    anyUpdateCoarseDensity = NV_FLOW_TRUE;
                    break;
                }
            }
            if (in->coarseDensity.textureTransient && anyUpdateCoarseDensity)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    EmitterPointClearDownsampleCS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.tableIn = in->value.sparseBuffer;
                    params.valueIn = in->value.textureTransient;
                    params.valueSampler = ptr->samplerLinear;
                    params.valueOut = in->coarseDensity.textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (coarseLevelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    EmitterPointClearDownsampleCS_addPassCompute(in->context, &ptr->emitterPointClearDownsampleCS, gridDim, &params);
                }
            }
        }
        // always put non streamed at end to override streaming effects
        if (feedback->anyStreamingDisabled)
        {
            for (NvFlowUint64 idx = 0u; idx < feedback->paramCount; idx++)
            {
                NvFlowBool32 isVelocity = in->velocityParamCount > 0u;
                const NvFlowEmitterPointParams* params = feedback->params[idx];
                if (!params->enableStreaming)
                {
                    EmitterPoint_executeSingleEmitter(ptr, in, out, levelParams, coarseLevelParams, params, isVelocity, feedback->anyStreamingClearEnabled);
                    anyEmit = NV_FLOW_TRUE;
                }
            }
        }

        EmitterPoint_voxelInterpolate(ptr, in, out, levelParams, feedback, anyEmit);
    }

    struct EmitterPoint_VoxelInterpolateTexture
    {
        NvFlowSparseTexture value;
        NvFlowSparseTexture weight;
    };

    void EmitterPoint_voxelInterpolate(
        EmitterPoint* ptr,
        const NvFlowEmitterPointPinsIn* in,
        NvFlowEmitterPointPinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        EmitterPointFeedbackInterface* feedback,
        NvFlowBool32 anyEmit
    )
    {
        if (!(feedback->anyInterpolationEnabled && !(in->velocityParamCount > 0u)))
        {
            return;
        }

        auto mappedLayer = (VoxelInterpolateCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->voxelLayerBuffer, feedback->interpolationEnabledCount * sizeof(VoxelInterpolateCS_LayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < feedback->interpolationEnabledCount; layerParamIdx++)
        {
            mappedLayer[layerParamIdx].enabled = feedback->interpolationEnableds[layerParamIdx];
            mappedLayer[layerParamIdx].threshold = 0.5f;
            mappedLayer[layerParamIdx].pad1 = 0.f;
            mappedLayer[layerParamIdx].pad2 = 0.f;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->voxelLayerBuffer);

        NvFlowArray<EmitterPoint_VoxelInterpolateTexture, 8u> valueDownsample;
        NvFlowArray<EmitterPoint_VoxelInterpolateTexture, 8u> valueUpsample;
        NvFlowUint64 levelCount = in->value.sparseParams.levelCount;
        valueDownsample.reserve(levelCount);
        valueDownsample.size = levelCount;
        valueUpsample.reserve(levelCount);
        valueUpsample.size = levelCount;
        for (NvFlowUint64 levelIdx = 0u; levelIdx < levelCount; levelIdx++)
        {
            valueDownsample[levelIdx].value = in->value;
            valueDownsample[levelIdx].weight = in->voxelWeight;
            valueUpsample[levelIdx].value = in->value;
            valueUpsample[levelIdx].weight = in->voxelWeight;

            NvFlowSparseLevelParams* coarseLevelParams = &in->value.sparseParams.levels[levelIdx];

            NvFlowTextureDesc texDesc = { eNvFlowTextureType_3d };
            texDesc.textureType = eNvFlowTextureType_3d;
            texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
            texDesc.format = in->value.format;
            texDesc.width = coarseLevelParams->dim.x;
            texDesc.height = coarseLevelParams->dim.y;
            texDesc.depth = coarseLevelParams->dim.z;
            texDesc.mipLevels = 1u;

            valueDownsample[levelIdx].value.textureTransient = ptr->contextInterface.getTextureTransient(in->context, &texDesc);
            if (levelIdx > 0u)
            {
                valueUpsample[levelIdx].value.textureTransient = ptr->contextInterface.getTextureTransient(in->context, &texDesc);
            }
            else // 0th upsample is the input value and final target
            {
                valueUpsample[levelIdx].value.textureTransient = in->value.textureTransient;
            }

            NvFlowTextureDesc weightTexDesc = texDesc;
            weightTexDesc.format = eNvFlowFormat_r8_unorm;

            // weights never commit back to main grid
            valueDownsample[levelIdx].weight.textureTransient = ptr->contextInterface.getTextureTransient(in->context, &weightTexDesc);
            valueUpsample[levelIdx].weight.textureTransient = ptr->contextInterface.getTextureTransient(in->context, &weightTexDesc);
        }

        if (!anyEmit)
        {
            if (levelCount == 0u)
            {
                return;
            }

            VoxelInterpolateNullCS_PassParams params = {};
            params.value0 = valueDownsample[0u].value.textureTransient;
            params.value1 = valueUpsample[0u].value.textureTransient;
            params.value2 = 1u < levelCount ? valueDownsample[1u].value.textureTransient : params.value0;
            params.value3 = 1u < levelCount ? valueUpsample[1u].value.textureTransient : params.value1;
            params.value4 = 2u < levelCount ? valueDownsample[2u].value.textureTransient : params.value0;
            params.value5 = 2u < levelCount ? valueUpsample[2u].value.textureTransient : params.value1;
            params.value6 = 3u < levelCount ? valueDownsample[3u].value.textureTransient : params.value0;
            params.value7 = 3u < levelCount ? valueUpsample[3u].value.textureTransient : params.value1;

            params.weight0 = valueDownsample[0u].weight.textureTransient;
            params.weight1 = valueUpsample[0u].weight.textureTransient;
            params.weight2 = 1u < levelCount ? valueDownsample[1u].weight.textureTransient : params.value0;
            params.weight3 = 1u < levelCount ? valueUpsample[1u].weight.textureTransient : params.value1;
            params.weight4 = 2u < levelCount ? valueDownsample[2u].weight.textureTransient : params.value0;
            params.weight5 = 2u < levelCount ? valueUpsample[2u].weight.textureTransient : params.value1;
            params.weight6 = 3u < levelCount ? valueDownsample[3u].weight.textureTransient : params.value0;
            params.weight7 = 3u < levelCount ? valueUpsample[3u].weight.textureTransient : params.value1;

            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = 1u;
            gridDim.z = 1u;

            VoxelInterpolateNullCS_addPassCompute(in->context, &ptr->voxelInterpolateNullCS, gridDim, &params);

            return;
        }

        // prepass
        {
            NvFlowUint64 levelIdx = 0u;

            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mappedGlobal = (VoxelInterpolateCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->voxelGlobalBuffer, sizeof(VoxelInterpolateCS_GlobalParams));

                mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
                mappedGlobal->levelIdx = (NvFlowUint)levelIdx;
                mappedGlobal->pad2 = 0u;
                mappedGlobal->pad3 = 0u;
                mappedGlobal->table = in->value.sparseParams.levels[levelIdx];
                mappedGlobal->sampleTable = in->value.sparseParams.levels[levelIdx];

                NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->voxelGlobalBuffer);

                batches[batchIdx].globalTransient = globalTransient;
            }

            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                VoxelInterpolatePrepassCS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.layerParamsIn = layerTransient;
                params.tableIn = in->value.sparseBuffer;
                params.densityIn = in->value.textureTransient;
                params.voxelWeightIn = in->voxelWeight.textureTransient;
                params.densityOut = valueDownsample[levelIdx].value.textureTransient;
                params.voxelWeightOut = valueDownsample[levelIdx].weight.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (in->value.sparseParams.levels[levelIdx].threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                VoxelInterpolatePrepassCS_addPassCompute(in->context, &ptr->voxelInterpolatePrepassCS, gridDim, &params);
            }
        }
        // downsample
        for (NvFlowUint64 levelIdx = 1u; levelIdx < levelCount; levelIdx++)
        {
            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mappedGlobal = (VoxelInterpolateCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->voxelGlobalBuffer, sizeof(VoxelInterpolateCS_GlobalParams));

                mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
                mappedGlobal->levelIdx = (NvFlowUint)levelIdx;
                mappedGlobal->pad2 = 0u;
                mappedGlobal->pad3 = 0u;
                mappedGlobal->table = in->value.sparseParams.levels[levelIdx];
                mappedGlobal->sampleTable = in->value.sparseParams.levels[levelIdx - 1u];

                NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->voxelGlobalBuffer);

                batches[batchIdx].globalTransient = globalTransient;
            }

            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                VoxelInterpolateDownsampleCS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.layerParamsIn = layerTransient;
                params.tableIn = in->value.sparseBuffer;
                params.densityIn = valueDownsample[levelIdx - 1u].value.textureTransient;
                params.voxelWeightIn = valueDownsample[levelIdx - 1u].weight.textureTransient;
                params.samplerIn = ptr->samplerLinear;
                params.densityOut = valueDownsample[levelIdx].value.textureTransient;
                params.voxelWeightOut = valueDownsample[levelIdx].weight.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (in->value.sparseParams.levels[levelIdx].threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                VoxelInterpolateDownsampleCS_addPassCompute(in->context, &ptr->voxelInterpolateDownsampleCS, gridDim, &params);
            }
        }
        // propagate
        for (NvFlowUint64 passId = 0u; passId < 4u; passId++)
        {
            NvFlowUint64 levelIdx = levelCount - 1u;

            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mappedGlobal = (VoxelInterpolateCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->voxelGlobalBuffer, sizeof(VoxelInterpolateCS_GlobalParams));

                mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
                mappedGlobal->levelIdx = (NvFlowUint)levelIdx;
                mappedGlobal->pad2 = 0u;
                mappedGlobal->pad3 = 0u;
                mappedGlobal->table = in->value.sparseParams.levels[levelIdx];
                mappedGlobal->sampleTable = in->value.sparseParams.levels[levelIdx];

                NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->voxelGlobalBuffer);

                batches[batchIdx].globalTransient = globalTransient;
            }

            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                VoxelInterpolatePropagateCS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.layerParamsIn = layerTransient;
                params.tableIn = in->value.sparseBuffer;
                params.densityIn = valueDownsample[levelIdx].value.textureTransient;
                params.voxelWeightIn = valueDownsample[levelIdx].weight.textureTransient;
                params.samplerIn = ptr->samplerLinear;
                params.densityOut = valueUpsample[levelIdx].value.textureTransient;
                params.voxelWeightOut = valueUpsample[levelIdx].weight.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (in->value.sparseParams.levels[levelIdx].threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                VoxelInterpolatePropagateCS_addPassCompute(in->context, &ptr->voxelInterpolatePropagateCS, gridDim, &params);

                params.densityOut = valueDownsample[levelIdx].value.textureTransient;
                params.voxelWeightOut = valueDownsample[levelIdx].weight.textureTransient;
                params.densityIn = valueUpsample[levelIdx].value.textureTransient;
                params.voxelWeightIn = valueUpsample[levelIdx].weight.textureTransient;

                VoxelInterpolatePropagateCS_addPassCompute(in->context, &ptr->voxelInterpolatePropagateCS, gridDim, &params);
            }
        }
        // upsample
        for (NvFlowUint64 levelIdx = levelCount - 2u; levelIdx < levelCount; levelIdx--)
        {
            NvFlowDispatchBatches batches;
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                auto mappedGlobal = (VoxelInterpolateCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->voxelGlobalBuffer, sizeof(VoxelInterpolateCS_GlobalParams));

                mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
                mappedGlobal->levelIdx = (NvFlowUint)levelIdx;
                mappedGlobal->pad2 = 0u;
                mappedGlobal->pad3 = 0u;
                mappedGlobal->table = in->value.sparseParams.levels[levelIdx];
                mappedGlobal->sampleTable = in->value.sparseParams.levels[levelIdx + 1u];

                NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->voxelGlobalBuffer);

                batches[batchIdx].globalTransient = globalTransient;
            }

            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                VoxelInterpolateUpsampleCS_PassParams params = {};
                params.globalParamsIn = batches[batchIdx].globalTransient;
                params.layerParamsIn = layerTransient;
                params.tableIn = in->value.sparseBuffer;
                params.densityIn = valueDownsample[levelIdx].value.textureTransient;
                params.voxelWeightIn = valueDownsample[levelIdx].weight.textureTransient;
                params.coarseDensityIn = valueUpsample[levelIdx + 1u].value.textureTransient;
                params.coarseVoxelWeightIn = valueUpsample[levelIdx + 1u].weight.textureTransient;
                params.samplerIn = ptr->samplerLinear;
                params.densityOut = valueUpsample[levelIdx].value.textureTransient;
                params.voxelWeightOut = valueUpsample[levelIdx].weight.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (in->value.sparseParams.levels[levelIdx].threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                VoxelInterpolateUpsampleCS_addPassCompute(in->context, &ptr->voxelInterpolateUpsampleCS, gridDim, &params);
            }
        }
    }

} // end namespace

NV_FLOW_OP_IMPL(NvFlowEmitterPoint, EmitterPoint)

namespace
{
    // Avoid member initialization, it can cause padding to not be initialized
    struct EmitterPointAllocateInstanceKey
    {
        NvFlowFloat4x4 localToWorld;
        NvFlowFloat3 blockSizeWorld;
        NvFlowBool32 enabled;
        NvFlowUint64 pointAllocateMaskVersion;
        int layerAndLevel;
        NvFlowBool32 allocateMask;
        NvFlowBool32 enableStreaming;
        NvFlowBool32 streamOnce;
        NvFlowBool32 streamClearAtStart;
        NvFlowUint pad;
        NvFlowUint64 pointPositionCount;
        NvFlowUint64 pointPositionVersion;
        NvFlowUint64 pointWidthCount;
        NvFlowUint64 pointWidthVersion;
        NvFlowUint64 pointColorCount;
        NvFlowUint64 pointColorVersion;
        float coupleRateTemperature;
        float coupleRateFuel;
        float coupleRateBurn;
        float coupleRateSmoke;
        NvFlowBool32 enableInterpolation;
        NvFlowUint voxelsPerPoint;
    };

    struct EmitterPointAllocateInstance
    {
        NvFlowUint64 luid = 0llu;
        NvFlowUint batchIdx = 0u;

        NvFlowUint64 updateVersion = 0llu;
        NvFlowUint64 changeVersion = 1llu;
        NvFlowEmitterPointParams params = {};

        EmitterPointAllocateInstanceKey key = {};
        NvFlowLocationHashTable locationHash;
    };

    struct EmitterPointAllocateTaskParams
    {
        NvFlowLocationHashTable locationHash;
        const NvFlowEmitterPointParams* params;
        NvFlowFloat3 blockSizeWorld;
        NvFlowUint3 blockDim;
        NvFlowUint maxLocations;
    };

    struct EmitterPointAllocate
    {
        NvFlowContextInterface contextInterface = {};

        NvFlowArrayPointer<EmitterPointAllocateInstance*> instances;

        NvFlowArray<EmitterPointAllocateTaskParams> taskParams;

        NvFlowUint64 globalUpdateVersion = 1llu;
        NvFlowUint64 globalChangeVersion = 1llu;

        NvFlowUint64 globalLocationHashVersion = 0llu;
        NvFlowLocationHashTable globalLocationHash;
        NvFlowUint cachedMaxLocations = 0u;

        NvFlowArray<NvFlowUint64> luids;
        NvFlowArray<NvFlowUint> batchIdxs;
        NvFlowArray<NvFlowUint64> changeVersions;
        NvFlowArray<const NvFlowEmitterPointParams*> params;
        NvFlowArray<NvFlowBool32> streamingEnableds;
        NvFlowArray<NvFlowBool32> interpolationEnableds;

        NvFlowUint64 history_stagedChangeVersion = 0llu;
        NvFlowUint64 history_globalChangeVersion = 0llu;
        NvFlowUint64 historyLocationChangeVersion = 0llu;
        NvFlowLocationHashTable historyLocationHash;
        NvFlowLocationHashTable historyLocationHashRescaled;

        NvFlowUint64 locations_globalChangeVersion = 0llu;
        NvFlowUint64 locations_historyChangeVersion = 0llu;
        NvFlowArray<NvFlowInt4> locations;

        EmitterPointFeedbackInterface feedback = {};

        NvFlowLuidTable luidTable;
    };

    EmitterPointAllocate* EmitterPointAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterPointAllocatePinsIn* in, NvFlowEmitterPointAllocatePinsOut* out)
    {
        auto ptr = new EmitterPointAllocate();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        return ptr;
    }

    void EmitterPointAllocate_destroy(EmitterPointAllocate* ptr, const NvFlowEmitterPointAllocatePinsIn* in, NvFlowEmitterPointAllocatePinsOut* out)
    {
        ptr->instances.deletePointers();

        delete ptr;
    }

    void EmitterPointAllocate_reportUpdate(void* userdata, NvFlowUint64 globalChangeVersion, NvFlowUint64 stagedChangeVersion)
    {
        if (!userdata)
        {
            return;
        }
        EmitterPointAllocate* ptr = (EmitterPointAllocate*)userdata;

        if (ptr->history_stagedChangeVersion != stagedChangeVersion ||
            ptr->history_globalChangeVersion != globalChangeVersion)
        {
            ptr->history_stagedChangeVersion = stagedChangeVersion;
            ptr->history_globalChangeVersion = globalChangeVersion;

            ptr->historyLocationChangeVersion++;
            if (globalChangeVersion == stagedChangeVersion)
            {
                // clear history
                ptr->historyLocationHash.reset();

                NvFlowArray_copy(ptr->historyLocationHash.layerInfos, ptr->globalLocationHash.layerInfos);
            }
            else
            {
                if (ptr->historyLocationHash.needsRescale(ptr->globalLocationHash.layerInfos.data, ptr->globalLocationHash.layerInfos.size))
                {
                    NvFlowArray_copy(ptr->historyLocationHash.tmpLocations, ptr->historyLocationHash.locations);

                    ptr->historyLocationHash.resetAndPushRescaled(
                        ptr->globalLocationHash.layerInfos.data,
                        ptr->globalLocationHash.layerInfos.size,
                        ptr->historyLocationHash.tmpLocations.data,
                        ptr->historyLocationHash.tmpLocations.size,
                        ptr->cachedMaxLocations
                    );
                }
            }
            // always need at least one frame in history
            for (NvFlowUint64 idx = 0u; idx < ptr->globalLocationHash.locations.size; idx++)
            {
                ptr->historyLocationHash.push(ptr->globalLocationHash.locations[idx], 0u);
                if (ptr->historyLocationHash.locations.size >= ptr->cachedMaxLocations)
                {
                    break;
                }
            }
        }
    }

    void EmitterPointAllocate_execute(EmitterPointAllocate* ptr, const NvFlowEmitterPointAllocatePinsIn* in, NvFlowEmitterPointAllocatePinsOut* out)
    {
        using namespace NvFlowMath;

        const NvFlowSparseParams* in_sparseParams = &in->sparseSimParams.sparseParams;

        NvFlowUint maxLocations = 0u;
        if (in_sparseParams->levelCount > 0u)
        {
            maxLocations = in_sparseParams->levels[0u].maxLocations;
        }
        ptr->cachedMaxLocations = maxLocations;

        // generate table for fast LUID resolve
        NvFlowLuidTable_reset(&ptr->luidTable, in->pointsParamCount);
        for (NvFlowUint64 pointsIdx = 0u; pointsIdx < in->pointsParamCount; pointsIdx++)
        {
            NvFlowLuidTable_insert(&ptr->luidTable, in->pointsParams[pointsIdx]->luid, pointsIdx);
        }

        ptr->globalUpdateVersion++;

        // refresh instances
        for (NvFlowUint64 paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
        {
            const NvFlowEmitterPointParams* paramsOrig = in->params[paramIdx];
            for (NvFlowUint64 pointsIdxU = 0u; pointsIdxU < 1u + paramsOrig->pointsLuidCount; pointsIdxU++)
            {
                NvFlowUint64 pointsIdx = pointsIdxU - 1u;
                NvFlowEmitterPointParams params = *paramsOrig;
                if (pointsIdx < paramsOrig->pointsLuidCount)
                {
                    // try to find points with luid
                    NvFlowUint64 pointsParamsIdx = NvFlowLuidTable_find(&ptr->luidTable, params.pointsLuids[pointsIdx]);
                    if (pointsParamsIdx < in->pointsParamCount)
                    {
                        const auto pointsParams = in->pointsParams[pointsParamsIdx];

                        params.luid = pointsParams->luid;  // need unique luid, steal points luid

                        params.pointPositions = pointsParams->points;
                        params.pointPositionCount = pointsParams->pointCount;
                        params.pointPositionVersion = pointsParams->pointVersion;

                        params.pointWidths = pointsParams->widths;
                        params.pointWidthCount = pointsParams->widthCount;
                        params.pointWidthVersion = pointsParams->widthVersion;

                        params.pointColors = pointsParams->colors;
                        params.pointColorCount = pointsParams->colorCount;
                        params.pointColorVersion = pointsParams->colorVersion;

                        params.localToWorld = pointsParams->localToWorld;
                        params.localToWorldVelocity = pointsParams->localToWorldVelocity;

                        if (paramsOrig->followVisibility)
                        {
                            params.enabled = paramsOrig->enabled && pointsParams->isVisible;
                        }
                        params.layer = paramsOrig->layer + pointsParams->layerOffset;
                        params.level = paramsOrig->level + pointsParams->levelOffset;
                    }
                }
                // auto level select, apply here so uniform everywhere
                if (params.levelCount > 1u && params.pointWidthCount > 0u)
                {
                    float pointWidthLocal = params.pointWidths[0u] * params.widthScale;

                    NvFlowFloat4 localVector = NvFlowMath::vector3Normalize(NvFlowFloat4{ 1.f, 1.f, 1.f, 0.f });
                    NvFlowFloat4 worldVector = NvFlowMath::vector4Transform(localVector, params.localToWorld);
                    float worldScale = NvFlowMath::vector3Length(worldVector).x;
                    float pointWidthWorld = worldScale * pointWidthLocal;

                    int minLevel = params.level;
                    int maxLevel = params.level + params.levelCount;
                    NvFlowUint minLayerIdx = ~0u;
                    float minCellSizeError = 0.f;
                    NvFlowUint layerCount = (in_sparseParams->layerCount < in->sparseSimParams.layerCount) ?
                        in_sparseParams->layerCount : in->sparseSimParams.layerCount;
                    for (NvFlowUint layerIdx = 0u; layerIdx < layerCount; layerIdx++)
                    {
                        const NvFlowSparseLayerParams* layerParams = in_sparseParams->layers + layerIdx;
                        const NvFlowSparseSimLayerParams* simLayerParams = in->sparseSimParams.layers + layerIdx;
                        NvFlowInt2 layerAndLevel = NvFlow_unpackLayerAndLevel(layerParams->layerAndLevel);
                        if (layerAndLevel.x == params.layer &&
                            layerAndLevel.y >= minLevel && layerAndLevel.y < maxLevel)
                        {
                            float worldCellSize = simLayerParams->densityCellSizeNonAuto;
                            float cellSizeError = fabsf(2.f * worldCellSize - pointWidthWorld);
                            if (minLayerIdx == ~0u || cellSizeError < minCellSizeError)
                            {
                                minLayerIdx = layerIdx;
                                minCellSizeError = cellSizeError;
                            }
                        }
                    }
                    if (minLayerIdx != ~0u)
                    {
                        NvFlowInt2 layerAndLevel = NvFlow_unpackLayerAndLevel(in_sparseParams->layers[minLayerIdx].layerAndLevel);
                        params.layer = layerAndLevel.x;
                        params.level = layerAndLevel.y;
                    }
                }
                // only capture active
                if (params.pointPositionCount > 0u)
                {
                    NvFlowUint64 batchCount = 1u;
                    if (params.enableStreaming && params.streamingBatchSize > 0u)
                    {
                        batchCount = (params.pointPositionCount + params.streamingBatchSize - 1u) / params.streamingBatchSize;
                    }
                    for (NvFlowUint64 batchIdx = 0llu; batchIdx < batchCount; batchIdx++)
                    {
                        // resolve instance
                        EmitterPointAllocateInstance* inst = nullptr;
                        for (NvFlowUint instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
                        {
                            auto instanceTest = ptr->instances[instanceIdx];
                            if (instanceTest->luid == params.luid &&
                                instanceTest->batchIdx == batchIdx)
                            {
                                inst = instanceTest;
                                break;
                            }
                        }
                        if (!inst)
                        {
                            inst = ptr->instances.allocateBackPointer();
                            inst->luid = params.luid;
                            inst->batchIdx = (NvFlowUint)batchIdx;
                            inst->changeVersion = 1llu;
                            inst->locationHash.reset();
                        }
                        inst->updateVersion = ptr->globalUpdateVersion;
                        inst->params = params;

                        // trim arrays based on batchIdx
                        arrayShift(inst->params.pointPositions, inst->params.pointPositionCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointAllocateMasks, inst->params.pointAllocateMaskCount, batchIdx, params.streamingBatchSize);

                        arrayShift(inst->params.pointVelocities, inst->params.pointVelocityCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointDivergences, inst->params.pointDivergenceCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointColors, inst->params.pointColorCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointTemperatures, inst->params.pointTemperatureCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointFuels, inst->params.pointFuelCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointBurns, inst->params.pointBurnCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointSmokes, inst->params.pointSmokeCount, batchIdx, params.streamingBatchSize);

                        arrayShift(inst->params.pointCoupleRateVelocities, inst->params.pointCoupleRateVelocityCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointCoupleRateDivergences, inst->params.pointCoupleRateDivergenceCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointCoupleRateTemperatures, inst->params.pointCoupleRateTemperatureCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointCoupleRateFuels, inst->params.pointCoupleRateFuelCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointCoupleRateBurns, inst->params.pointCoupleRateBurnCount, batchIdx, params.streamingBatchSize);
                        arrayShift(inst->params.pointCoupleRateSmokes, inst->params.pointCoupleRateSmokeCount, batchIdx, params.streamingBatchSize);
                    }
                }
            }
        }

        // delete inactive instances
        NvFlowUint64 writeIdx = 0llu;
        NvFlowBool32 instanceReleased = NV_FLOW_FALSE;
        for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            EmitterPointAllocateInstance* inst = ptr->instances[instanceIdx];
            ptr->instances.swapPointers(writeIdx, instanceIdx);
            if (inst->updateVersion < ptr->globalUpdateVersion)
            {
                ptr->instances.deletePointerAtIndex(writeIdx);
                instanceReleased = NV_FLOW_TRUE;
            }
            else
            {
                writeIdx++;
            }
        }
        ptr->instances.size = writeIdx;

        // if instance released, bump global version
        if (instanceReleased)
        {
            ptr->globalChangeVersion++;
        }

        // refresh location hash tables as needed
        for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            EmitterPointAllocateInstance* inst = ptr->instances[instanceIdx];
            const NvFlowEmitterPointParams* params = &inst->params;

            NvFlowFloat3 blockSizeWorld = { 32.f, 16.f, 16.f };
            NvFlowBool32 forceDisableEmitters = NV_FLOW_TRUE;
            NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(in_sparseParams, params->layer, params->level);
            if (layerParamIdx != ~0u)
            {
                const NvFlowSparseLayerParams* layerParams = &in_sparseParams->layers[layerParamIdx];

                blockSizeWorld = layerParams->blockSizeWorld;
                forceDisableEmitters = layerParams->forceDisableEmitters;
            }

            NvFlowBool32 enabled = params->enabled;
            if (forceDisableEmitters)
            {
                enabled = NV_FLOW_FALSE;
            }

            EmitterPointAllocateInstanceKey key;
            memset(&key, 0, sizeof(key));                           // explicit to cover any padding
            key.localToWorld = params->localToWorld;
            key.blockSizeWorld = blockSizeWorld;
            key.enabled = enabled;
            key.layerAndLevel = NvFlow_packLayerAndLevel(params->layer, params->level);
            key.allocateMask = params->allocateMask;
            key.pointAllocateMaskVersion = params->pointAllocateMaskVersion;
            key.enableStreaming = params->enableStreaming;
            key.streamOnce = params->streamOnce;
            key.streamClearAtStart = params->streamClearAtStart;
            key.pointPositionCount = params->pointPositionCount;
            key.pointPositionVersion = params->pointPositionVersion;
            key.pointWidthCount = params->pointWidthCount;
            key.pointWidthVersion = params->pointWidthVersion;
            key.pointColorCount = params->pointColorCount;
            key.pointColorVersion = params->pointColorVersion;
            key.coupleRateTemperature = params->coupleRateTemperature;
            key.coupleRateFuel = params->coupleRateFuel;
            key.coupleRateBurn = params->coupleRateBurn;
            key.coupleRateSmoke = params->coupleRateSmoke;
            key.enableInterpolation = params->enableInterpolation;
            key.voxelsPerPoint = params->voxelsPerPoint;

            bool positionForceDirty = params->pointPositionCount > 0u && params->pointPositionVersion == 0llu;
            bool widthForceDirty = params->pointWidthCount > 0u && params->pointWidthVersion == 0llu;
            bool colorForceDirty = params->pointColorCount > 0u && params->pointColorVersion == 0llu;
            bool allocateForceDirty = params->pointAllocateMaskCount > 0u && params->pointAllocateMaskVersion == 0llu;
            bool isDirty = (memcmp(&key, &inst->key, sizeof(key)) != 0u)
                || positionForceDirty || widthForceDirty || colorForceDirty || allocateForceDirty;

            //if (key.coupleRateTemperature != inst->key.coupleRateTemperature ||
            //    key.coupleRateFuel != inst->key.coupleRateFuel ||
            //    key.coupleRateBurn != inst->key.coupleRateBurn ||
            //    key.coupleRateSmoke != inst->key.coupleRateSmoke)
            //{
            //}

            inst->key = key;
            if (isDirty)
            {
                inst->changeVersion++;
                ptr->globalChangeVersion++;

                inst->locationHash.reset();

                static const NvFlowUint64 pointsPerTask = 8192u;
                NvFlowUint64 taskCount = ((params->pointPositionCount + pointsPerTask - 1u) / pointsPerTask);

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
                    ptr->taskParams[taskIdx].blockSizeWorld = blockSizeWorld;
                    ptr->taskParams[taskIdx].blockDim = blockDim;
                    ptr->taskParams[taskIdx].maxLocations = maxLocations;
                }

                auto task = [](NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
                {
                    auto ptr = (EmitterPointAllocate*)userdata;

                    auto& taskParams = ptr->taskParams[taskIdx];

                    NvFlowUint64 particleBeginIdx = taskIdx * pointsPerTask;
                    NvFlowUint64 particleEndIdx = particleBeginIdx + pointsPerTask;
                    if (particleEndIdx > taskParams.params->pointPositionCount)
                    {
                        particleEndIdx = taskParams.params->pointPositionCount;
                    }

                    if (taskParams.params->allocateMask || (taskParams.params->pointAllocateMaskCount > 0u))
                    {
                        const float xf_neg = 2.f / ((float)taskParams.blockDim.x);
                        const float xf_pos = 1.f - xf_neg;
                        const float yf_neg = 2.f / ((float)taskParams.blockDim.y);
                        const float yf_pos = 1.f - yf_neg;
                        const float zf_neg = 2.f / ((float)taskParams.blockDim.z);
                        const float zf_pos = 1.f - zf_neg;

                        for (NvFlowUint64 particleIdx = particleBeginIdx; particleIdx < particleEndIdx; particleIdx++)
                        {
                            NvFlowBool32 allocateMask = taskParams.params->allocateMask;
                            if (particleIdx < taskParams.params->pointAllocateMaskCount)
                            {
                                allocateMask = taskParams.params->pointAllocateMasks[particleIdx];
                            }
                            if (allocateMask)
                            {
                                NvFlowFloat3 pointPosition = taskParams.params->pointPositions[particleIdx];
                                NvFlowFloat4 positionLocal = make_float4(pointPosition, 1.f);
                                NvFlowFloat4 position = vector4Transform(positionLocal, taskParams.params->localToWorld);
                                if (position.w > 0.f)
                                {
                                    float wInv = 1.f / position.w;
                                    position.x *= wInv;
                                    position.y *= wInv;
                                    position.z *= wInv;
                                }

                                int layerAndLevel = NvFlow_packLayerAndLevel(taskParams.params->layer, taskParams.params->level);

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
                                if (taskParams.locationHash.locations.size >= taskParams.maxLocations)
                                {
                                    break;
                                }
                            }
                        }
                    }
                };

                if (key.enabled)
                {
                    ptr->contextInterface.executeTasks(in->context, (NvFlowUint)taskCount, taskCount < 8u ? 8u : 1u, task, ptr);

                    for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
                    {
                        auto& taskParams = ptr->taskParams[taskIdx];
                        for (NvFlowUint locationIdx = 0u; locationIdx < taskParams.locationHash.locations.size; locationIdx++)
                        {
                            NvFlowInt4 entry_location = taskParams.locationHash.locations[locationIdx];
                            NvFlowUint entry_mask = taskParams.locationHash.masks[locationIdx];

                            inst->locationHash.push(entry_location, entry_mask);
                            if (inst->locationHash.locations.size >= maxLocations)
                            {
                                break;
                            }
                        }
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

                //inst->locationHash.computeStats();
            }
        }

        if (ptr->globalLocationHashVersion != ptr->globalChangeVersion)
        {
            ptr->globalLocationHashVersion = ptr->globalChangeVersion;

            ptr->globalLocationHash.reset();

            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
            {
                EmitterPointAllocateInstance* inst = ptr->instances[instanceIdx];

                for (NvFlowUint64 idx = 0u; idx < inst->locationHash.locations.size; idx++)
                {
                    ptr->globalLocationHash.push(inst->locationHash.locations[idx], 0u);
                    if (ptr->globalLocationHash.locations.size >= maxLocations)
                    {
                        break;
                    }
                }
            }

            // refresh layerInfos
            ptr->globalLocationHash.layerInfos.reserve(in_sparseParams->layerCount);
            ptr->globalLocationHash.layerInfos.size = in_sparseParams->layerCount;
            for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in_sparseParams->layerCount; layerParamIdx++)
            {
                const NvFlowSparseLayerParams* layerParams = in_sparseParams->layers + layerParamIdx;
                ptr->globalLocationHash.layerInfos[layerParamIdx].blockSizeWorld = layerParams->blockSizeWorld;
                ptr->globalLocationHash.layerInfos[layerParamIdx].layerAndLevel = layerParams->layerAndLevel;
            }

            ptr->luids.size = 0u;
            ptr->batchIdxs.size = 0u;
            ptr->changeVersions.size = 0u;
            ptr->params.size = 0u;
            ptr->streamingEnableds.size = 0u;
            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
            {
                EmitterPointAllocateInstance* inst = ptr->instances[instanceIdx];

                ptr->luids.pushBack(inst->luid);
                ptr->batchIdxs.pushBack(inst->batchIdx);
                ptr->changeVersions.pushBack(inst->changeVersion);
                ptr->params.pushBack(&inst->params);
                ptr->streamingEnableds.pushBack(inst->params.enableStreaming);
            }

            NvFlowBool32 anyStreamingEnabled = NV_FLOW_FALSE;
            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->params.size; instanceIdx++)
            {
                if (ptr->params[instanceIdx]->enableStreaming)
                {
                    anyStreamingEnabled = NV_FLOW_TRUE;
                    break;
                }
            }
            NvFlowBool32 anyStreamingDisabled = NV_FLOW_FALSE;
            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->params.size; instanceIdx++)
            {
                if (!ptr->params[instanceIdx]->enableStreaming)
                {
                    anyStreamingDisabled = NV_FLOW_TRUE;
                    break;
                }
            }
            NvFlowBool32 anyStreamingClearEnabled = NV_FLOW_FALSE;
            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->params.size; instanceIdx++)
            {
                if (ptr->params[instanceIdx]->enableStreaming && ptr->params[instanceIdx]->streamClearAtStart)
                {
                    // only clear if meaningful
                    bool isActive = ptr->params[instanceIdx]->pointPositionCount > 0u;
                    if (isActive)
                    {
                        anyStreamingClearEnabled = NV_FLOW_TRUE;
                        break;
                    }
                }
            }

            // interpolation voting
            NvFlowBool32 anyInterpolationEnabled = NV_FLOW_FALSE;
            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->params.size; instanceIdx++)
            {
                if (ptr->params[instanceIdx]->enableInterpolation)
                {
                    anyInterpolationEnabled = NV_FLOW_TRUE;
                    break;
                }
            }
            ptr->interpolationEnableds.reserve(in_sparseParams->layerCount);
            ptr->interpolationEnableds.size = in_sparseParams->layerCount;
            for (NvFlowUint64 idx = 0u; idx < ptr->interpolationEnableds.size; idx++)
            {
                ptr->interpolationEnableds[idx] = NV_FLOW_FALSE;
            }
            if (anyInterpolationEnabled)
            {
                for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->params.size; instanceIdx++)
                {
                    if (ptr->params[instanceIdx]->enableInterpolation)
                    {
                        NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(in_sparseParams,
                            ptr->params[instanceIdx]->layer, ptr->params[instanceIdx]->level);
                        if (layerParamIdx < ptr->interpolationEnableds.size)
                        {
                            ptr->interpolationEnableds[layerParamIdx] = NV_FLOW_TRUE;
                        }
                    }
                }
            }

            ptr->feedback.globalUpdateVersion = ptr->globalUpdateVersion;
            ptr->feedback.globalChangeVersion = ptr->globalChangeVersion;
            ptr->feedback.anyStreamingEnabled = anyStreamingEnabled;
            ptr->feedback.anyStreamingDisabled = anyStreamingDisabled;
            ptr->feedback.anyStreamingClearEnabled = anyStreamingClearEnabled;
            ptr->feedback.anyInterpolationEnabled = anyInterpolationEnabled;

            ptr->feedback.luids = ptr->luids.data;
            ptr->feedback.luidCount = ptr->luids.size;
            ptr->feedback.batchIdxs = ptr->batchIdxs.data;
            ptr->feedback.batchIdxCount = ptr->batchIdxs.size;
            ptr->feedback.changeVersions = ptr->changeVersions.data;
            ptr->feedback.changeCount = ptr->changeVersions.size;
            ptr->feedback.params = ptr->params.data;
            ptr->feedback.paramCount = ptr->params.size;
            ptr->feedback.streamingEnableds = ptr->streamingEnableds.data;
            ptr->feedback.streamingEnabledCount = ptr->streamingEnableds.size;
            ptr->feedback.interpolationEnableds = ptr->interpolationEnableds.data;
            ptr->feedback.interpolationEnabledCount = ptr->interpolationEnableds.size;
        }

        // purge layers from history if forceDisableEmitters
        if (ptr->historyLocationHash.locations.size > 0u)
        {
            NvFlowBool32 anyForceDisable = NV_FLOW_FALSE;
            for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in_sparseParams->layerCount; layerParamIdx++)
            {
                if (!in_sparseParams->layers[layerParamIdx].forceDisableEmitters)
                {
                    continue;
                }
                if (!anyForceDisable)
                {
                    for (NvFlowUint64 idx = 0u; idx < ptr->historyLocationHash.locations.size; idx++)
                    {
                        ptr->historyLocationHash.masks[idx] = 1u;
                    }
                    anyForceDisable = NV_FLOW_TRUE;
                }
                int layerAndLevel = in_sparseParams->layers[layerParamIdx].layerAndLevel;
                for (NvFlowUint64 idx = 0u; idx < ptr->historyLocationHash.locations.size; idx++)
                {
                    if (ptr->locations[idx].w == layerAndLevel)
                    {
                        ptr->historyLocationHash.masks[idx] = 0u;
                    }
                }
            }
            if (anyForceDisable)
            {
                ptr->historyLocationHash.compactNonZeroWithLimit(ptr->historyLocationHash.locations.size);
                for (NvFlowUint64 idx = 0u; idx < ptr->historyLocationHash.locations.size; idx++)
                {
                    ptr->historyLocationHash.masks[idx] = 0u;
                }
                ptr->historyLocationChangeVersion++;
            }
        }

        if (ptr->locations_globalChangeVersion != ptr->globalLocationHashVersion ||
            ptr->locations_historyChangeVersion != ptr->historyLocationChangeVersion)
        {
            ptr->locations_globalChangeVersion = ptr->globalLocationHashVersion;
            ptr->locations_historyChangeVersion = ptr->historyLocationChangeVersion;

            ptr->locations.size = 0u;
            ptr->locations.reserve(ptr->globalLocationHash.locations.size);
            ptr->locations.size = ptr->globalLocationHash.locations.size;
            for (NvFlowUint64 idx = 0u; idx < ptr->globalLocationHash.locations.size; idx++)
            {
                ptr->locations[idx] = ptr->globalLocationHash.locations[idx];
            }

            if (ptr->historyLocationHash.needsRescale(ptr->globalLocationHash.layerInfos.data, ptr->globalLocationHash.layerInfos.size))
            {
                NvFlowArray_copy(ptr->historyLocationHashRescaled.layerInfos, ptr->historyLocationHash.layerInfos);

                ptr->historyLocationHashRescaled.resetAndPushRescaled(
                    ptr->globalLocationHash.layerInfos.data,
                    ptr->globalLocationHash.layerInfos.size,
                    ptr->historyLocationHash.locations.data,
                    ptr->historyLocationHash.locations.size,
                    ptr->cachedMaxLocations
                );

                for (NvFlowUint64 idx = 0u; idx < ptr->historyLocationHashRescaled.locations.size; idx++)
                {
                    NvFlowInt4 location = ptr->historyLocationHashRescaled.locations[idx];
                    ptr->locations.pushBack(location);
                }
            }
            else
            {
                for (NvFlowUint64 idx = 0u; idx < ptr->historyLocationHash.locations.size; idx++)
                {
                    NvFlowInt4 location = ptr->historyLocationHash.locations[idx];
                    ptr->locations.pushBack(location);
                }
            }
        }

        ptr->feedback.globalUpdateVersion = ptr->globalUpdateVersion;
        ptr->feedback.userdata = ptr;
        ptr->feedback.reportUpdate = EmitterPointAllocate_reportUpdate;

        out->locations = ptr->locations.data;
        out->locationCount = ptr->locations.size;
        out->feedback.data = &ptr->feedback;
    }
} // end namespace

NV_FLOW_OP_IMPL(NvFlowEmitterPointAllocate, EmitterPointAllocate)
