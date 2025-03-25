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

#include "shaders/EmitterNanoVdbCS.hlsl.h"
#include "shaders/EmitterNanoVdb2CS.hlsl.h"
#include "shaders/EmitterPointMarkCS.hlsl.h"
#include "shaders/EmitterPointClearMarkedCS.hlsl.h"
#include "shaders/EmitterPointClearDownsampleCS.hlsl.h"

#define PNANOVDB_C
#define PNANOVDB_BUF_BOUNDS_CHECK

#include "nanovdb/PNanoVDB.h"

#include "NvFlowLocationHashTable.h"

#include <string.h>

#if defined(_WIN32)
#include <Windows.h>
#else
#include <unistd.h>
#endif

 // feedback interface
namespace
{
    struct EmitterNanoVdbFeedbackInterface
    {
        NvFlowUint64 globalUpdateVersion;
        NvFlowUint64 globalChangeVersion;
        NvFlowBool32 anyStreamingEnabled;
        NvFlowBool32 anyStreamingDisabled;
        NvFlowBool32 anyStreamingClearEnabled;
        NvFlowUint64 pendingVdbLoadCount;

        const NvFlowUint64* luids;
        NvFlowUint64 luidCount;
        const NvFlowUint* batchIdxs;
        NvFlowUint64 batchIdxCount;
        const NvFlowUint64* changeVersions;
        NvFlowUint64 changeCount;
        const NvFlowEmitterNanoVdbParams** params;
        NvFlowUint64 paramCount;
        const NvFlowBool32* streamingEnableds;
        NvFlowUint64 streamingEnabledCount;

        void* userdata;
        void(NV_FLOW_ABI* reportUpdate)(void* userdata, NvFlowUint64 globalChangeVersion, NvFlowUint64 stagedChangeVersion);
    };
}

namespace
{
    enum Array
    {
        eArray_velocities = 0,
        eArray_divergences = 1,
        eArray_temperatures = 2,
        eArray_fuels = 3,
        eArray_burns = 4,
        eArray_smokes = 5,
        eArray_coupleRateVelocities = 6,
        eArray_coupleRateDivergences = 7,
        eArray_coupleRateTemperatures = 8,
        eArray_coupleRateFuels = 9,
        eArray_coupleRateBurns = 10,
        eArray_coupleRateSmokes = 11,
        eArray_distances = 12,
        eArray_rgba8s = 13,

        eArray_count = 14
    };

    struct EmitterNanoVdbCache
    {
        NvFlowUint64 owningLuid = 0llu;
        NvFlowArrayBufferData arrayDatas[eArray_count] = { };
        NvFlowUint batchIdx = 0u;
        NvFlowUint activeCount = 0u;
        NvFlowUint inactiveCount = 0u;
        NvFlowArrayBuffer arrayBuffer = {};
    };

    struct EmitterNanoVdbInteropCache
    {
        NvFlowInteropHandle interopHandle = {};
        NvFlowBuffer* buffer = nullptr;
        NvFlowBufferTransient* transientBuffer = nullptr;
        NvFlowUint64 transientFrame = ~0llu;
    };

    struct EmitterNanoVdb
    {
        NvFlowContextInterface contextInterface = {};

        EmitterNanoVdbCS_Pipeline emitterNanoVdbCS = {};
        EmitterNanoVdb2CS_Pipeline emitterNanoVdb2CS = {};
        EmitterPointMarkCS_Pipeline emitterPointMarkCS = {};
        EmitterPointClearMarkedCS_Pipeline emitterPointClearMarkedCS = {};
        EmitterPointClearDownsampleCS_Pipeline emitterPointClearDownsampleCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};
        NvFlowUploadBuffer dummyBuffer = {};
        NvFlowArrayBuffer arrayBuffer = {};
        NvFlowArrayPointer<EmitterNanoVdbCache*> caches;
        NvFlowArrayPointer<EmitterNanoVdbInteropCache*> interopCaches;

        NvFlowUint64 globalChangeVersion = 0llu;
        NvFlowUint64 stagedChangeVersion = 0llu;

        NvFlowUint64 stagedStreamingVersion = 0llu;

        NvFlowUint64 stagedLuidBeginIdx = 0llu;
        NvFlowUint64 stagedLuidEndIdx = 0llu;

        NvFlowUint64 globalUpdateVersion = 0llu;
        NvFlowBool32 shouldMark = NV_FLOW_FALSE;
        NvFlowBool32 shouldClearMarked = NV_FLOW_FALSE;
        NvFlowUint64 nextStagedLuidIdx = 0llu;

        NvFlowLuidTable luidTable;
    };

    EmitterNanoVdb* EmitterNanoVdb_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterNanoVdbPinsIn* in, NvFlowEmitterNanoVdbPinsOut* out)
    {
        auto ptr = new EmitterNanoVdb();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EmitterNanoVdbCS_init(&ptr->contextInterface, in->context, &ptr->emitterNanoVdbCS);
        EmitterNanoVdb2CS_init(&ptr->contextInterface, in->context, &ptr->emitterNanoVdb2CS);
        EmitterPointMarkCS_init(&ptr->contextInterface, in->context, &ptr->emitterPointMarkCS);
        EmitterPointClearMarkedCS_init(&ptr->contextInterface, in->context, &ptr->emitterPointClearMarkedCS);
        EmitterPointClearDownsampleCS_init(&ptr->contextInterface, in->context, &ptr->emitterPointClearDownsampleCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->dummyBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        NvFlowArrayBuffer_init(&ptr->contextInterface, in->context, &ptr->arrayBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        // upload dummy buffer contents once, reuse
        void* mapped = NvFlowUploadBuffer_map(in->context, &ptr->dummyBuffer, PNANOVDB_GRID_SIZE);
        memset(mapped, 0, PNANOVDB_GRID_SIZE);
        NvFlowUploadBuffer_unmapDevice(in->context, &ptr->dummyBuffer, 0u, PNANOVDB_GRID_SIZE, "nanovdb_dummy");

        return ptr;
    }

    void EmitterNanoVdb_destroy(EmitterNanoVdb* ptr, const NvFlowEmitterNanoVdbPinsIn* in, NvFlowEmitterNanoVdbPinsOut* out)
    {
        for (NvFlowUint64 cacheIdx = 0llu; cacheIdx < ptr->interopCaches.size; cacheIdx++)
        {
            if (ptr->interopCaches[cacheIdx]->buffer)
            {
                ptr->contextInterface.destroyBuffer(in->context, ptr->interopCaches[cacheIdx]->buffer);
                ptr->interopCaches[cacheIdx]->buffer = nullptr;
            }
        }
        for (NvFlowUint64 cacheIdx = 0llu; cacheIdx < ptr->caches.size; cacheIdx++)
        {
            NvFlowArrayBuffer_destroy(in->context, &ptr->caches[cacheIdx]->arrayBuffer);
        }
        NvFlowArrayBuffer_destroy(in->context, &ptr->arrayBuffer);

        NvFlowUploadBuffer_destroy(in->context, &ptr->dummyBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        EmitterNanoVdbCS_destroy(in->context, &ptr->emitterNanoVdbCS);
        EmitterNanoVdb2CS_destroy(in->context, &ptr->emitterNanoVdb2CS);
        EmitterPointMarkCS_destroy(in->context, &ptr->emitterPointMarkCS);
        EmitterPointClearMarkedCS_destroy(in->context, &ptr->emitterPointClearMarkedCS);
        EmitterPointClearDownsampleCS_destroy(in->context, &ptr->emitterPointClearDownsampleCS);

        delete ptr;
    }

    NvFlowBufferTransient* EmitterNanoVdb_resolveInterop(EmitterNanoVdb* ptr, const NvFlowEmitterNanoVdbPinsIn* in, NvFlowUint64 nanoVdbsInterop, NvFlowUint64 nanoVdbCountInterop)
    {
        if (!nanoVdbsInterop || !nanoVdbCountInterop)
        {
            return nullptr;
        }

        NvFlowInteropHandle interopHandle = {};
#if defined(_WIN32)
        interopHandle.type = eNvFlowInteropHandleType_opaqueWin32;
#else
        interopHandle.type = eNvFlowInteropHandleType_opaqueFd;
#endif
        interopHandle.value = nanoVdbsInterop;
        interopHandle.resourceSizeInBytes = nanoVdbCountInterop * 4u;

        for (NvFlowUint64 cacheIdx = 0llu; cacheIdx < ptr->interopCaches.size; cacheIdx++)
        {
            EmitterNanoVdbInteropCache* interopCache = ptr->interopCaches[cacheIdx];
            if (interopCache->interopHandle.type == interopHandle.type &&
                interopCache->interopHandle.value == interopHandle.value &&
                interopCache->interopHandle.resourceSizeInBytes == interopHandle.resourceSizeInBytes)
            {
                if (!interopCache->buffer)
                {
                    return nullptr;
                }
                else
                {
                    if (interopCache->transientBuffer &&
                        interopCache->transientFrame == ptr->contextInterface.getCurrentFrame(in->context))
                    {
                        return interopCache->transientBuffer;
                    }
                    if (interopCache->buffer)
                    {
                        interopCache->transientBuffer =
                            ptr->contextInterface.registerBufferAsTransient(in->context, interopCache->buffer);
                        interopCache->transientFrame = ptr->contextInterface.getCurrentFrame(in->context);
                    }
                    return interopCache->transientBuffer;
                }
            }
        }

        // add to cache
        EmitterNanoVdbInteropCache* interopCache = ptr->interopCaches.allocateBackPointer();

        // treat original interop handle as key to duplicate from
        interopCache->interopHandle = interopHandle;
        NvFlowBufferDesc bufDesc = {};
        bufDesc.usageFlags = eNvFlowBufferUsage_structuredBuffer;
        bufDesc.format = eNvFlowFormat_unknown;
        bufDesc.structureStride = 4u;
        bufDesc.sizeInBytes = interopHandle.resourceSizeInBytes;

        // need to duplicate handle to get ownership
        NvFlowInteropHandle dupInteropHandle = interopHandle;
#if defined(_WIN32)
        HANDLE handle_src = (HANDLE)interopHandle.value;
        HANDLE handle_dst = nullptr;
        DuplicateHandle(GetCurrentProcess(), handle_src, GetCurrentProcess(), &handle_dst, 0, FALSE, DUPLICATE_SAME_ACCESS);
        dupInteropHandle.value = (NvFlowUint64)handle_dst;
#else
        int fd_src = (int)interopHandle.value;
        int fd_dst = 0;
        fd_dst = dup(fd_src);
        dupInteropHandle.value = (NvFlowUint64)fd_dst;
#endif
        // attempt buffer creation with duplicated interop handle
        interopCache->buffer = ptr->contextInterface.createBufferFromExternalHandle(in->context, &bufDesc, &dupInteropHandle);
        interopCache->transientBuffer = nullptr;
        interopCache->transientFrame = ~0llu;

        // under failure, need to release handles
        if (!interopCache->buffer)
        {
#if defined(_WIN32)
            CloseHandle(handle_dst);
#else
            close(fd_dst);
#endif
        }

        auto logPrint = ptr->contextInterface.getLogPrint(in->context);
        if (logPrint)
        {
            logPrint(eNvFlowLogLevel_info, "EmitterNanoVDB: import of interop handle(%llu) size(%llu) %s",
                (unsigned long long int)interopHandle.value,
                (unsigned long long int)interopHandle.resourceSizeInBytes,
                interopCache->buffer ? "succeeded" : "failed"
            );
        }

        return EmitterNanoVdb_resolveInterop(ptr, in, nanoVdbsInterop, nanoVdbCountInterop);
    }

    void EmitterNanoVdb_accumBounds(NvFlowFloat3* local_accum_min, NvFlowFloat3* local_accum_max, pnanovdb_buf_t buf)
    {
        if (buf.size_in_words > 0u)
        {
            pnanovdb_grid_handle_t grid = { 0u };
            pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
            pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

            pnanovdb_coord_t bbox_min = pnanovdb_root_get_bbox_min(buf, root);
            pnanovdb_coord_t bbox_max = pnanovdb_root_get_bbox_max(buf, root);

            if (bbox_max.x < bbox_min.x ||
                bbox_max.y < bbox_min.y ||
                bbox_max.z < bbox_min.z)
            {
                return;
            }

            pnanovdb_vec3_t bbox_minf = { (float)bbox_min.x, (float)bbox_min.y, (float)bbox_min.z };
            pnanovdb_vec3_t bbox_maxf = { (float)bbox_max.x + 1.f, (float)bbox_max.y + 1.f, (float)bbox_max.z + 1.f };

            pnanovdb_vec3_t localBounds[8u] = {
                {bbox_minf.x, bbox_minf.y, bbox_minf.z},
                {bbox_maxf.x, bbox_minf.y, bbox_minf.z},
                {bbox_minf.x, bbox_maxf.y, bbox_minf.z},
                {bbox_maxf.x, bbox_maxf.y, bbox_minf.z},
                {bbox_minf.x, bbox_minf.y, bbox_maxf.z},
                {bbox_maxf.x, bbox_minf.y, bbox_maxf.z},
                {bbox_minf.x, bbox_maxf.y, bbox_maxf.z},
                {bbox_maxf.x, bbox_maxf.y, bbox_maxf.z}
            };

            NvFlowFloat4 vdb_world_min = { 0.f, 0.f, 0.f, 0.f };
            NvFlowFloat4 vdb_world_max = { 0.f, 0.f, 0.f, 0.f };
            for (NvFlowUint idx = 0u; idx < 8u; idx++)
            {
                pnanovdb_vec3_t world = pnanovdb_grid_index_to_worldf(buf, grid, PNANOVDB_REF(localBounds[idx]));
                NvFlowFloat4 world4 = { world.x, world.y, world.z, 1.f };
                if (idx == 0u)
                {
                    vdb_world_min = world4;
                    vdb_world_max = world4;
                }
                vdb_world_min = NvFlowMath::vectorMin(vdb_world_min, world4);
                vdb_world_max = NvFlowMath::vectorMax(vdb_world_max, world4);
            }

            if (local_accum_min->x > local_accum_max->x)
            {
                local_accum_min->x = vdb_world_min.x;
                local_accum_min->y = vdb_world_min.y;
                local_accum_min->z = vdb_world_min.z;
                local_accum_max->x = vdb_world_max.x;
                local_accum_max->y = vdb_world_max.y;
                local_accum_max->z = vdb_world_max.z;
            }
            if (vdb_world_min.x < local_accum_min->x)
            {
                local_accum_min->x = vdb_world_min.x;
            }
            if (vdb_world_min.y < local_accum_min->y)
            {
                local_accum_min->y = vdb_world_min.y;
            }
            if (vdb_world_min.z < local_accum_min->z)
            {
                local_accum_min->z = vdb_world_min.z;
            }
            if (vdb_world_max.x > local_accum_max->x)
            {
                local_accum_max->x = vdb_world_max.x;
            }
            if (vdb_world_max.y > local_accum_max->y)
            {
                local_accum_max->y = vdb_world_max.y;
            }
            if (vdb_world_max.z > local_accum_max->z)
            {
                local_accum_max->z = vdb_world_max.z;
            }
        }
    }

    void EmitterNanoVdb_executeSingleEmitter(
        EmitterNanoVdb* ptr,
        const NvFlowEmitterNanoVdbPinsIn* in,
        NvFlowEmitterNanoVdbPinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        NvFlowSparseLevelParams* coarseLevelParams,
        const NvFlowEmitterNanoVdbParams* params,
        NvFlowBool32 isVelocity,
        NvFlowBool32 anyStreamingClearEnabled,
        NvFlowSparseTexture* valueTemp
    )
    {
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

        NvFlowBool32 sourceIsVelocity = isVelocity;
        if (params->swapChannels)
        {
            sourceIsVelocity = !sourceIsVelocity;
        }

        // Early out if couple rates leave emitter having zero effect, only with defaults
        if (!sourceIsVelocity &&
            params->coupleRateTemperature <= 0.f &&
            params->coupleRateFuel <= 0.f &&
            params->coupleRateBurn <= 0.f &&
            params->coupleRateSmoke <= 0.f)
        {
            return;
        }
        if (sourceIsVelocity &&
            params->coupleRateVelocity <= 0.f &&
            params->coupleRateDivergence <= 0.f)
        {
            return;
        }

        // Early out if no NanoVDBs are present to create a bounding box
        bool isActive = params->nanoVdbDistanceCount > 0u ||
            params->nanoVdbVelocityCount > 0u ||
            params->nanoVdbDivergenceCount > 0u ||
            params->nanoVdbTemperatureCount > 0u ||
            params->nanoVdbFuelCount > 0u ||
            params->nanoVdbBurnCount > 0u ||
            params->nanoVdbSmokeCount > 0u ||
            params->nanoVdbCoupleRateVelocityCount > 0u ||
            params->nanoVdbCoupleRateDivergenceCount > 0u ||
            params->nanoVdbCoupleRateTemperatureCount > 0u ||
            params->nanoVdbCoupleRateFuelCount > 0u ||
            params->nanoVdbCoupleRateBurnCount > 0u ||
            params->nanoVdbCoupleRateSmokeCount > 0u ||
            params->nanoVdbRgba8Count > 0u;
        if (!isActive)
        {
            return;
        }

        NvFlowFloat3 local_min = { 1.f, 1.f, 1.f };
        NvFlowFloat3 local_max = { 0.f, 0.f, 0.f };
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbVelocities, params->nanoVdbVelocityCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbDivergences, params->nanoVdbDivergenceCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbTemperatures, params->nanoVdbTemperatureCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbFuels, params->nanoVdbFuelCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbBurns, params->nanoVdbBurnCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbSmokes, params->nanoVdbSmokeCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateVelocities, params->nanoVdbCoupleRateVelocityCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateDivergences, params->nanoVdbCoupleRateDivergenceCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateTemperatures, params->nanoVdbCoupleRateTemperatureCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateFuels, params->nanoVdbCoupleRateFuelCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateBurns, params->nanoVdbCoupleRateBurnCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateSmokes, params->nanoVdbCoupleRateSmokeCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbDistances, params->nanoVdbDistanceCount));
        EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbRgba8s, params->nanoVdbRgba8Count));
        // take invalid bounds to 0 size
        if (local_max.x < local_min.x ||
            local_max.y < local_min.y ||
            local_max.z < local_min.z)
        {
            local_min = local_max;
        }

        // compute position and halfSize
        NvFlowFloat3 position = { 0.5f * (local_max.x + local_min.x), 0.5f * (local_max.y + local_min.y), 0.5f * (local_max.z + local_min.z) };
        NvFlowFloat3 halfSize = { 0.5f * (local_max.x - local_min.x), 0.5f * (local_max.y - local_min.y), 0.5f * (local_max.z - local_min.z) };

        NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;

        NvFlowInt4 locationMin = {};
        NvFlowInt4 locationMax = {};
        computeEmitterBoxBounds(params->localToWorld, params->layer, params->level, position, halfSize, blockSizeWorld, &locationMin, &locationMax);

        NvFlowUint numLocations = (locationMax.x - locationMin.x) * (locationMax.y - locationMin.y) * (locationMax.z - locationMin.z);

        // early out if no coverage
        if (numLocations == 0u)
        {
            return;
        }

        // resolve interop early, to avoid wasted copies
        NvFlowBufferTransient* interopDistances =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbDistancesInterop, params->nanoVdbDistanceCountInterop);
        NvFlowBufferTransient* interopVelocities =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbVelocitiesInterop, params->nanoVdbVelocityCountInterop);
        NvFlowBufferTransient* interopDivergences =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbDivergencesInterop, params->nanoVdbDivergenceCountInterop);
        NvFlowBufferTransient* interopTemperatures =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbTemperaturesInterop, params->nanoVdbTemperatureCountInterop);
        NvFlowBufferTransient* interopFuels =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbFuelsInterop, params->nanoVdbFuelCountInterop);
        NvFlowBufferTransient* interopBurns =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbBurnsInterop, params->nanoVdbBurnCountInterop);
        NvFlowBufferTransient* interopSmokes =
            EmitterNanoVdb_resolveInterop(ptr, in, params->nanoVdbSmokesInterop, params->nanoVdbSmokeCountInterop);

        NvFlowArrayBufferData arrayDatas[eArray_count] = { };
        arrayDatas[eArray_velocities] = { params->nanoVdbVelocities, interopVelocities ? 0u : params->nanoVdbVelocityCount, params->nanoVdbVelocityVersion };
        arrayDatas[eArray_divergences] = { params->nanoVdbDivergences, interopDivergences ? 0u : params->nanoVdbDivergenceCount, params->nanoVdbDivergenceVersion };
        arrayDatas[eArray_temperatures] = { params->nanoVdbTemperatures, interopTemperatures ? 0u : params->nanoVdbTemperatureCount, params->nanoVdbTemperatureVersion };
        arrayDatas[eArray_fuels] = { params->nanoVdbFuels, interopFuels ? 0u : params->nanoVdbFuelCount, params->nanoVdbFuelVersion };
        arrayDatas[eArray_burns] = { params->nanoVdbBurns, interopBurns ? 0u : params->nanoVdbBurnCount, params->nanoVdbBurnVersion };
        arrayDatas[eArray_smokes] = { params->nanoVdbSmokes, interopSmokes ? 0u : params->nanoVdbSmokeCount, params->nanoVdbSmokeVersion };
        arrayDatas[eArray_coupleRateVelocities] = { params->nanoVdbCoupleRateVelocities, params->nanoVdbCoupleRateVelocityCount, params->nanoVdbCoupleRateVelocityVersion };
        arrayDatas[eArray_coupleRateDivergences] = { params->nanoVdbCoupleRateDivergences, params->nanoVdbCoupleRateDivergenceCount, params->nanoVdbCoupleRateDivergenceVersion };
        arrayDatas[eArray_coupleRateTemperatures] = { params->nanoVdbCoupleRateTemperatures, params->nanoVdbCoupleRateTemperatureCount, params->nanoVdbCoupleRateTemperatureVersion };
        arrayDatas[eArray_coupleRateFuels] = { params->nanoVdbCoupleRateFuels, params->nanoVdbCoupleRateFuelCount, params->nanoVdbCoupleRateFuelVersion };
        arrayDatas[eArray_coupleRateBurns] = { params->nanoVdbCoupleRateBurns, params->nanoVdbCoupleRateBurnCount, params->nanoVdbCoupleRateBurnVersion };
        arrayDatas[eArray_coupleRateSmokes] = { params->nanoVdbCoupleRateSmokes, params->nanoVdbCoupleRateSmokeCount, params->nanoVdbCoupleRateSmokeVersion };
        arrayDatas[eArray_distances] = { params->nanoVdbDistances, interopDistances ? 0u : params->nanoVdbDistanceCount, params->nanoVdbDistanceVersion };
        arrayDatas[eArray_rgba8s] = { params->nanoVdbRgba8s, params->nanoVdbRgba8Count, params->nanoVdbRgba8Version };
        NvFlowUint64 firstElements[eArray_count] = {};

        NvFlowArrayBuffer* arrayBuffer = &ptr->arrayBuffer;
        if (!params->enableStreaming)
        {
            // NOTE: assuming no batching for now
            EmitterNanoVdbCache* cache = nullptr;
            // prefer to recycle owned cache to avoid VRAM waste
            for (NvFlowUint64 cacheIdx = 0u; cacheIdx < ptr->caches.size; cacheIdx++)
            {
                if (ptr->caches[cacheIdx]->owningLuid == params->luid)
                {
                    cache = ptr->caches[cacheIdx];
                    break;
                }
            }
            // try to recycle other cache with identical source data
            if (!cache)
            {
                for (NvFlowUint64 cacheIdx = 0u; cacheIdx < ptr->caches.size; cacheIdx++)
                {
                    bool doesMatch = true;
                    for (NvFlowUint idx = 0u; idx < eArray_count; idx++)
                    {
                        if (ptr->caches[cacheIdx]->arrayDatas[idx].data != arrayDatas[idx].data ||
                            ptr->caches[cacheIdx]->arrayDatas[idx].elementCount != arrayDatas[idx].elementCount ||
                            ptr->caches[cacheIdx]->arrayDatas[idx].version != arrayDatas[idx].version)
                        {
                            doesMatch = false;
                            break;
                        }
                    }
                    if (doesMatch)
                    {
                        cache = ptr->caches[cacheIdx];
                        break;
                    }
                }
            }
            // create cache if necessary
            if (!cache)
            {
                cache = ptr->caches.allocateBackPointer();
                cache->owningLuid = params->luid;
                cache->batchIdx = 0u;
                NvFlowArrayBuffer_init(&ptr->contextInterface, in->context, &cache->arrayBuffer, eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
            }
            // always update source data
            for (NvFlowUint idx = 0u; idx < eArray_count; idx++)
            {
                cache->arrayDatas[idx] = arrayDatas[idx];
            }
            cache->activeCount++;
            arrayBuffer = &cache->arrayBuffer;
        }
        NvFlowBufferTransient* arraysTransient = NvFlowArrayBuffer_update(in->context, arrayBuffer, params->luid, arrayDatas, firstElements, eArray_count, nullptr, "EmitterNanoVdbUpload");

        NvFlowFloat3 vidxToWorld = {
            blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
            blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
            blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
        };

        NvFlowFloat4x4 worldToLocal = NvFlowMath::matrixInverse(params->localToWorld);

        NvFlowBool32 texIsSrgb = NV_FLOW_FALSE;
        if (in->value.format == eNvFlowFormat_r8g8b8a8_unorm)
        {
            if (ptr->contextInterface.isFeatureSupported(in->context, eNvFlowContextFeature_aliasResourceFormats))
            {
                texIsSrgb = NV_FLOW_TRUE;
            }
        }

        // intentionally aligns with scratch in Sparse.cpp
        static const NvFlowUint tempCubeDim = 16u;
        static const NvFlowUint tempCubeDim3 = 4096u;

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init_custom(&batches, numLocations, tempCubeDim3);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (EmitterNanoVdbCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterNanoVdbCS_Params));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->needsSrgbConversion = params->colorIsSrgb && !texIsSrgb;
            mapped->velocityIsWorldSpace = params->velocityIsWorldSpace;
            mapped->sourceIsVelocity = sourceIsVelocity;

            mapped->table = *levelParams;

            if (sourceIsVelocity)
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
            mapped->isVelocity = isVelocity;

            mapped->position = position;
            mapped->minDistance = params->minDistance;

            mapped->halfSizeInv = NvFlowFloat3{
                1.f / halfSize.x,
                1.f / halfSize.y,
                1.f / halfSize.z
            };
            mapped->maxDistance = params->maxDistance;

            mapped->halfSize = halfSize;
            mapped->deltaTime = 0.5f * in->deltaTime;

            mapped->anyStreamingClearEnabled = anyStreamingClearEnabled;
            mapped->absoluteValue = params->absoluteValue;
            mapped->minSmoke = params->minSmoke;
            mapped->maxSmoke = params->maxSmoke;

            mapped->computeSpeed = params->computeSpeed;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;

            mapped->worldToLocal = NvFlowMath::matrixTranspose(worldToLocal);
            mapped->localToWorldVelocity = NvFlowMath::matrixTranspose(params->localToWorldVelocity);

            mapped->locationOffset = locationMin;
            mapped->locationExtent = NvFlowUint4{
                NvFlowUint(locationMax.x - locationMin.x),
                NvFlowUint(locationMax.y - locationMin.y),
                NvFlowUint(locationMax.z - locationMin.z),
                NvFlowUint(locationMax.w - locationMin.w)
            };

            mapped->range_velocities = {
                (NvFlowUint)(firstElements[eArray_velocities] + params->nanoVdbVelocityFirstElement),
                (NvFlowUint)(params->nanoVdbVelocityCount - params->nanoVdbVelocityFirstElement),
                0u, 0u
            };
            mapped->range_divergences = {
                (NvFlowUint)(firstElements[eArray_divergences] + params->nanoVdbDivergenceFirstElement),
                (NvFlowUint)(params->nanoVdbDivergenceCount - params->nanoVdbDivergenceFirstElement),
                0u, 0u
            };
            mapped->range_temperatures = {
                (NvFlowUint)(firstElements[eArray_temperatures] + params->nanoVdbTemperatureFirstElement),
                (NvFlowUint)(params->nanoVdbTemperatureCount - params->nanoVdbTemperatureFirstElement),
                0u, 0u
            };
            mapped->range_fuels = {
                (NvFlowUint)(firstElements[eArray_fuels] + params->nanoVdbFuelFirstElement),
                (NvFlowUint)(params->nanoVdbFuelCount - params->nanoVdbFuelFirstElement),
                0u, 0u
            };
            mapped->range_burns = {
                (NvFlowUint)(firstElements[eArray_burns] + params->nanoVdbBurnFirstElement),
                (NvFlowUint)(params->nanoVdbBurnCount - params->nanoVdbBurnFirstElement),
                0u, 0u
            };
            mapped->range_smokes = {
                (NvFlowUint)(firstElements[eArray_smokes] + params->nanoVdbSmokeFirstElement),
                (NvFlowUint)(params->nanoVdbSmokeCount - params->nanoVdbSmokeFirstElement),
                0u, 0u
            };
            mapped->range_coupleRateVelocities = {
                (NvFlowUint)(firstElements[eArray_coupleRateVelocities] + params->nanoVdbCoupleRateVelocityFirstElement),
                (NvFlowUint)(params->nanoVdbCoupleRateVelocityCount - params->nanoVdbCoupleRateVelocityFirstElement),
                0u, 0u
            };
            mapped->range_coupleRateDivergences = {
                (NvFlowUint)(firstElements[eArray_coupleRateDivergences] + params->nanoVdbCoupleRateDivergenceFirstElement),
                (NvFlowUint)(params->nanoVdbCoupleRateDivergenceCount - params->nanoVdbCoupleRateDivergenceFirstElement),
                0u, 0u
            };
            mapped->range_coupleRateTemperatures = {
                (NvFlowUint)(firstElements[eArray_coupleRateTemperatures] + params->nanoVdbCoupleRateTemperatureFirstElement),
                (NvFlowUint)(params->nanoVdbCoupleRateTemperatureCount - params->nanoVdbCoupleRateTemperatureFirstElement),
                0u, 0u
            };
            mapped->range_coupleRateFuels = {
                (NvFlowUint)(firstElements[eArray_coupleRateFuels] + params->nanoVdbCoupleRateFuelFirstElement),
                (NvFlowUint)(params->nanoVdbCoupleRateFuelCount - params->nanoVdbCoupleRateFuelFirstElement),
                0u, 0u
            };
            mapped->range_coupleRateBurns = {
                (NvFlowUint)(firstElements[eArray_coupleRateBurns] + params->nanoVdbCoupleRateBurnFirstElement),
                (NvFlowUint)(params->nanoVdbCoupleRateBurnCount - params->nanoVdbCoupleRateBurnFirstElement),
                0u, 0u
            };
            mapped->range_coupleRateSmokes = {
                (NvFlowUint)(firstElements[eArray_coupleRateSmokes] + params->nanoVdbCoupleRateSmokeFirstElement),
                (NvFlowUint)(params->nanoVdbCoupleRateSmokeCount - params->nanoVdbCoupleRateSmokeFirstElement),
                0u, 0u
            };

            mapped->range_distances = { (NvFlowUint)firstElements[eArray_distances], (NvFlowUint)params->nanoVdbDistanceCount, 0u, 0u };
            mapped->range_rgba8s = { (NvFlowUint)firstElements[eArray_rgba8s], (NvFlowUint)params->nanoVdbRgba8Count, 0u, 0u };

            // interop overrides
            if (interopDistances)
            {
                mapped->range_distances = { 0u, (NvFlowUint)params->nanoVdbDistanceCountInterop, 1u, 0u };
            }
            if (interopVelocities)
            {
                mapped->range_velocities = { 0u, (NvFlowUint)params->nanoVdbVelocityCountInterop, 2u, 0u };
            }
            if (interopDivergences)
            {
                mapped->range_divergences = { 0u, (NvFlowUint)params->nanoVdbDivergenceCountInterop, 3u, 0u };
            }
            if (interopTemperatures)
            {
                mapped->range_temperatures = { 0u, (NvFlowUint)params->nanoVdbTemperatureCountInterop, 4u, 0u };
            }
            if (interopFuels)
            {
                mapped->range_fuels = { 0u, (NvFlowUint)params->nanoVdbFuelCountInterop, 5u, 0u };
            }
            if (interopBurns)
            {
                mapped->range_burns = { 0u, (NvFlowUint)params->nanoVdbBurnCountInterop, 6u, 0u };
            }
            if (interopSmokes)
            {
                mapped->range_smokes = { 0u, (NvFlowUint)params->nanoVdbSmokeCountInterop, 7u, 0u };
            }

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = constantTransient;
        }

        NvFlowBufferTransient* dummyTransient = NvFlowUploadBuffer_getDevice(in->context, &ptr->dummyBuffer, PNANOVDB_GRID_SIZE);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterNanoVdbCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.arraysIn = arraysTransient;
            passParams.interopDistances = interopDistances ? interopDistances : dummyTransient;
            passParams.interopVelocities = interopVelocities ? interopVelocities : dummyTransient;
            passParams.interopDivergences = interopDivergences ? interopDivergences : dummyTransient;
            passParams.interopTemperatures = interopTemperatures ? interopTemperatures : dummyTransient;
            passParams.interopFuels = interopFuels ? interopFuels : dummyTransient;
            passParams.interopBurns = interopBurns ? interopBurns : dummyTransient;
            passParams.interopSmokes = interopSmokes ? interopSmokes : dummyTransient;
            passParams.valueIn = in->value.textureTransient;
            passParams.valueOut = valueTemp->textureTransient;
            passParams.voxelWeightOut = in->voxelWeight.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterNanoVdbCS_addPassCompute(in->context, &ptr->emitterNanoVdbCS, gridDim, &passParams);

            EmitterNanoVdb2CS_PassParams passParams2 = {};
            passParams2.gParams = batches[batchIdx].globalTransient;
            passParams2.gTable = in->value.sparseBuffer;
            passParams2.valueIn = valueTemp->textureTransient;
            passParams2.valueOut = out->value.textureTransient;

            EmitterNanoVdb2CS_addPassCompute(in->context, &ptr->emitterNanoVdb2CS, gridDim, &passParams2);
        }
    }

    void EmitterNanoVdb_execute(EmitterNanoVdb* ptr, const NvFlowEmitterNanoVdbPinsIn* in, NvFlowEmitterNanoVdbPinsOut* out)
    {
        out->streamingVersion = in->streamingVersion;
        out->streamingFinishedVersion = in->streamingFinishedVersion;
        out->streamingNanoVdbVersion = in->streamingNanoVdbVersion;

        NvFlowSparseLevelParams* levelParams = &in->value.sparseParams.levels[in->value.levelIdx];
        NvFlowSparseLevelParams* coarseLevelParams = levelParams;
        if (in->coarseDensity.textureTransient)
        {
            coarseLevelParams = &in->coarseDensity.sparseParams.levels[in->coarseDensity.levelIdx];
        }

        // passthrough, since input is mutable
        NvFlowSparseTexture_passThrough(&out->value, &in->value);
        NvFlowSparseTexture_passThrough(&out->voxelWeight, &in->voxelWeight);

        // intentionally aligns with scratch in Sparse.cpp
        static const NvFlowUint tempCubeDim = 16u;
        static const NvFlowUint tempCubeDim3 = 4096u;
        NvFlowSparseTexture valueTemp = {};
        {
            valueTemp = in->value;

            NvFlowSparseLevelParams* levelParams = &valueTemp.sparseParams.levels[valueTemp.levelIdx];

            NvFlowTextureDesc texDesc = { eNvFlowTextureType_3d };
            texDesc.textureType = eNvFlowTextureType_3d;
            texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
            texDesc.format = valueTemp.format;
            texDesc.width = tempCubeDim * (levelParams->blockDimLessOne.x + 1u);
            texDesc.height = tempCubeDim * (levelParams->blockDimLessOne.y + 1u);
            texDesc.depth = tempCubeDim * (levelParams->blockDimLessOne.z + 1u);
            texDesc.mipLevels = 1u;

            valueTemp.textureTransient = ptr->contextInterface.getTextureTransient(in->context, &texDesc);
        }

        EmitterNanoVdbFeedbackInterface* feedback = (EmitterNanoVdbFeedbackInterface*)in->feedback.data;
        if (!feedback)
        {
            return;
        }

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
            NvFlowUint enabledCount = 0u;
            while (ptr->nextStagedLuidIdx < feedback->paramCount && (pointsThisFrame == 0llu || pointsThisFrame < pointBudget))
            {
                const NvFlowEmitterNanoVdbParams* localParams = feedback->params[ptr->nextStagedLuidIdx];
                if (localParams->enabled && localParams->enableStreaming)
                {
                    pointsThisFrame += localParams->nanoVdbSmokeCount;
                    pointBudget = localParams->streamingBatchSize;
                    enabledCount++;
                }
                ptr->stagedLuidEndIdx++;
                ptr->nextStagedLuidIdx++;
                // Rate limit for now to avoid too many compute shader dispatches
                if (enabledCount >= 8u)
                {
                    break;
                }
            }

            if (ptr->globalChangeVersion != feedback->globalChangeVersion ||
                out->streamingNanoVdbVersion != out->streamingVersion)
            {
                if (ptr->nextStagedLuidIdx >= feedback->luidCount && feedback->pendingVdbLoadCount == 0llu)
                {
                    ptr->globalChangeVersion = ptr->stagedChangeVersion;
                    out->streamingNanoVdbVersion = ptr->stagedStreamingVersion;
                }
            }

            ptr->shouldClearMarked = NV_FLOW_FALSE;
            if (out->streamingFinishedVersion != out->streamingVersion)
            {
                bool allFinished =
                    out->streamingNanoVdbVersion == out->streamingVersion &&
                    in->streamingPointsVersion == out->streamingVersion;
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
                    const NvFlowEmitterNanoVdbParams* params = feedback->params[paramIdx];
                    if (params->enableStreaming)
                    {
                        EmitterNanoVdb_executeSingleEmitter(ptr, in, out, levelParams, coarseLevelParams, params, isVelocity, feedback->anyStreamingClearEnabled, &valueTemp);
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
        // reset cache tracking
        {
            for (NvFlowUint64 idx = 0u; idx < ptr->caches.size; idx++)
            {
                ptr->caches[idx]->activeCount = 0;
            }
        }
        // always put non streamed at end to override streaming effects
        if (feedback->anyStreamingDisabled)
        {
            for (NvFlowUint64 idx = 0u; idx < feedback->paramCount; idx++)
            {
                NvFlowBool32 isVelocity = in->velocityParamCount > 0u;
                const NvFlowEmitterNanoVdbParams* params = feedback->params[idx];
                if (!params->enableStreaming)
                {
                    EmitterNanoVdb_executeSingleEmitter(ptr, in, out, levelParams, coarseLevelParams, params, isVelocity, feedback->anyStreamingClearEnabled, &valueTemp);
                }
            }
        }
        // update cache
        {
            static const NvFlowUint cleanupThreshold = 16u;
            NvFlowUint64 writeIdx = 0u;
            for (NvFlowUint64 readIdx = 0u; readIdx < ptr->caches.size; readIdx++)
            {
                if (ptr->caches[readIdx]->activeCount == 0u)
                {
                    ptr->caches[readIdx]->inactiveCount++;
                }
                else
                {
                    ptr->caches[readIdx]->inactiveCount = 0u;
                }
                if (ptr->caches[readIdx]->inactiveCount > cleanupThreshold)
                {
                    NvFlowArrayBuffer_destroy(in->context, &ptr->caches[readIdx]->arrayBuffer);
                    ptr->caches.deletePointerAtIndex(readIdx);
                }
                else
                {
                    ptr->caches.swapPointers(writeIdx, readIdx);
                    writeIdx++;
                }
            }
            ptr->caches.size = writeIdx;
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterNanoVdb, EmitterNanoVdb)

namespace
{
    // Avoid member initialization, it can cause padding to not be initialized
    struct EmitterNanoVdbAllocateInstanceKey
    {
        NvFlowFloat4x4 localToWorld;
        NvFlowFloat3 blockSizeWorld;
        NvFlowBool32 enabled;
        int layerAndLevel;
        float allocationScale;
        NvFlowBool32 enableStreaming;
        NvFlowBool32 streamOnce;
        NvFlowBool32 streamClearAtStart;
        NvFlowUint pad;
        NvFlowUint64 nanoVdbVelocityCount;
        NvFlowUint64 nanoVdbVelocityVersion;
        NvFlowUint64 nanoVdbDivergenceCount;
        NvFlowUint64 nanoVdbDivergenceVersion;
        NvFlowUint64 nanoVdbTemperatureCount;
        NvFlowUint64 nanoVdbTemperatureVersion;
        NvFlowUint64 nanoVdbFuelCount;
        NvFlowUint64 nanoVdbFuelVersion;
        NvFlowUint64 nanoVdbBurnCount;
        NvFlowUint64 nanoVdbBurnVersion;
        NvFlowUint64 nanoVdbSmokeCount;
        NvFlowUint64 nanoVdbSmokeVersion;
        NvFlowUint64 nanoVdbRgba8Count;
        NvFlowUint64 nanoVdbRgba8Version;
        float coupleRateVelocity;
        float coupleRateDivergence;
        float coupleRateTemperature;
        float coupleRateFuel;
        float coupleRateBurn;
        float coupleRateSmoke;
        float minSmoke;
        float maxSmoke;
        NvFlowBool32 allocateActiveLeaves;
    };

    struct EmitterNanoVdbAllocateInstance
    {
        NvFlowUint64 luid = 0llu;
        NvFlowUint batchIdx = 0u;

        NvFlowUint64 updateVersion = 0llu;
        NvFlowUint64 changeVersion = 1llu;
        NvFlowEmitterNanoVdbParams params = {};

        NvFlowArray<pnanovdb_leaf_handle_t> leaves;

        EmitterNanoVdbAllocateInstanceKey key = {};
        NvFlowLocationHashTable locationHash;
    };

    struct EmitterNanoVdbAllocateTaskParams
    {
        NvFlowLocationHashTable locationHash;
        const NvFlowEmitterNanoVdbParams* params;
        NvFlowFloat3 blockSizeWorld;
        NvFlowUint3 blockDim;
        NvFlowUint maxLocations;
    };

    struct EmitterNanoVdbAllocate
    {
        NvFlowContextInterface contextInterface = {};

        NvFlowArrayPointer<EmitterNanoVdbAllocateInstance*> instances;

        NvFlowArray<EmitterNanoVdbAllocateTaskParams> taskParams;

        NvFlowUint64 globalUpdateVersion = 1llu;
        NvFlowUint64 globalChangeVersion = 1llu;

        NvFlowUint64 globalLocationHashVersion = 0llu;
        NvFlowLocationHashTable globalLocationHash;
        NvFlowUint cachedMaxLocations = 0u;

        NvFlowArray<NvFlowUint64> luids;
        NvFlowArray<NvFlowUint> batchIdxs;
        NvFlowArray<NvFlowUint64> changeVersions;
        NvFlowArray<const NvFlowEmitterNanoVdbParams*> params;
        NvFlowArray<NvFlowBool32> streamingEnableds;

        NvFlowUint64 history_stagedChangeVersion = 0llu;
        NvFlowUint64 history_globalChangeVersion = 0llu;
        NvFlowUint64 historyLocationChangeVersion = 0llu;
        NvFlowLocationHashTable historyLocationHash;
        NvFlowLocationHashTable historyLocationHashRescaled;

        NvFlowUint64 locations_globalChangeVersion = 0llu;
        NvFlowUint64 locations_historyChangeVersion = 0llu;
        NvFlowArray<NvFlowInt4> locations;

        EmitterNanoVdbFeedbackInterface feedback = {};

        NvFlowLuidTable luidTableVol;
        NvFlowLuidTable luidTableAsset;
    };

    EmitterNanoVdbAllocate* EmitterNanoVdbAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterNanoVdbAllocatePinsIn* in, NvFlowEmitterNanoVdbAllocatePinsOut* out)
    {
        auto ptr = new EmitterNanoVdbAllocate();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        return ptr;
    }

    void EmitterNanoVdbAllocate_destroy(EmitterNanoVdbAllocate* ptr, const NvFlowEmitterNanoVdbAllocatePinsIn* in, NvFlowEmitterNanoVdbAllocatePinsOut* out)
    {
        ptr->instances.deletePointers();

        delete ptr;
    }

    void EmitterNanoVdbAllocate_reportUpdate(void* userdata, NvFlowUint64 globalChangeVersion, NvFlowUint64 stagedChangeVersion)
    {
        if (!userdata)
        {
            return;
        }
        EmitterNanoVdbAllocate* ptr = (EmitterNanoVdbAllocate*)userdata;

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

    void EmitterNanoVdbAllocate_generateLeaves(EmitterNanoVdbAllocate* ptr, EmitterNanoVdbAllocateInstance* inst, pnanovdb_buf_t buf)
    {
        pnanovdb_grid_handle_t grid = { 0u };
        pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
        pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
        pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

        inst->leaves.size = 0u;
        pnanovdb_uint32_t tile_count = pnanovdb_root_get_tile_count(buf, root);
        for (pnanovdb_uint32_t tile_idx = 0u; tile_idx < tile_count; tile_idx++)
        {
            pnanovdb_root_tile_handle_t tile = pnanovdb_root_get_tile(grid_type, root, tile_idx);
            if (pnanovdb_int64_is_zero(pnanovdb_root_tile_get_child(buf, tile)))
            {
                continue;
            }
            pnanovdb_upper_handle_t upper = pnanovdb_root_get_child(grid_type, buf, root, tile);
            for (pnanovdb_uint32_t upper_n = 0u; upper_n < PNANOVDB_UPPER_TABLE_COUNT; upper_n++)
            {
                if (!pnanovdb_upper_get_child_mask(buf, upper, upper_n))
                {
                    continue;
                }
                pnanovdb_lower_handle_t lower = pnanovdb_upper_get_child(grid_type, buf, upper, upper_n);
                for (pnanovdb_uint32_t lower_n = 0u; lower_n < PNANOVDB_LOWER_TABLE_COUNT; lower_n++)
                {
                    if (!pnanovdb_lower_get_child_mask(buf, lower, lower_n))
                    {
                        continue;
                    }
                    pnanovdb_leaf_handle_t leaf = pnanovdb_lower_get_child(grid_type, buf, lower, lower_n);
                    inst->leaves.pushBack(leaf);
                }
            }
        }
    }

    void EmitterNanoVdbAllocate_compute(EmitterNanoVdbAllocate* ptr, EmitterNanoVdbAllocateInstance* inst, const NvFlowEmitterNanoVdbAllocatePinsIn* in)
    {
        inst->locationHash.reset();

        const NvFlowEmitterNanoVdbParams* params = &inst->params;
        if (!params->enabled)
        {
            return;
        }
        NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->sparseSimParams.sparseParams, params->layer, params->level);
        if (layerParamIdx == ~0u)
        {
            return;
        }
        const NvFlowSparseLayerParams* layerParams = &in->sparseSimParams.sparseParams.layers[layerParamIdx];
        if (layerParams->forceDisableEmitters)
        {
            return;
        }

        if (params->allocationScale > 0.f)
        {
            NvFlowFloat3 local_min = { 1.f, 1.f, 1.f };
            NvFlowFloat3 local_max = { 0.f, 0.f, 0.f };
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbVelocities, params->nanoVdbVelocityCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbDivergences, params->nanoVdbDivergenceCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbTemperatures, params->nanoVdbTemperatureCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbFuels, params->nanoVdbFuelCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbBurns, params->nanoVdbBurnCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbSmokes, params->nanoVdbSmokeCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateVelocities, params->nanoVdbCoupleRateVelocityCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateDivergences, params->nanoVdbCoupleRateDivergenceCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateTemperatures, params->nanoVdbCoupleRateTemperatureCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateFuels, params->nanoVdbCoupleRateFuelCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateBurns, params->nanoVdbCoupleRateBurnCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbCoupleRateSmokes, params->nanoVdbCoupleRateSmokeCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbDistances, params->nanoVdbDistanceCount));
            EmitterNanoVdb_accumBounds(&local_min, &local_max, pnanovdb_make_buf(params->nanoVdbRgba8s, params->nanoVdbRgba8Count));
            // take invalid bounds to 0 size
            if (local_max.x < local_min.x ||
                local_max.y < local_min.y ||
                local_max.z < local_min.z)
            {
                local_min = local_max;
            }

            // compute position and halfSize
            NvFlowFloat3 position = { 0.5f * (local_max.x + local_min.x), 0.5f * (local_max.y + local_min.y), 0.5f * (local_max.z + local_min.z) };
            NvFlowFloat3 halfSize = { 0.5f * (local_max.x - local_min.x), 0.5f * (local_max.y - local_min.y), 0.5f * (local_max.z - local_min.z) };

            halfSize.x *= params->allocationScale;
            halfSize.y *= params->allocationScale;
            halfSize.z *= params->allocationScale;

            NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
            NvFlowFloat3 blockSizeWorldInv = {
                1.f / blockSizeWorld.x,
                1.f / blockSizeWorld.y,
                1.f / blockSizeWorld.z
            };

            if (params->allocateActiveLeaves)
            {
                int minLocation_w = NvFlow_packLayerAndLevel(params->layer, params->level);
                int maxLocation_w = NvFlow_packLayerAndLevel(params->layer + 1, params->level);

                static const NvFlowUint bufferCountFull = 14u;
                pnanovdb_buf_t buffersFull[bufferCountFull] =
                {
                    pnanovdb_make_buf(params->nanoVdbRgba8s, params->nanoVdbRgba8Count),
                    pnanovdb_make_buf(params->nanoVdbSmokes, params->nanoVdbSmokeCount),
                    pnanovdb_make_buf(params->nanoVdbVelocities, params->nanoVdbVelocityCount),
                    pnanovdb_make_buf(params->nanoVdbDivergences, params->nanoVdbDivergenceCount),
                    pnanovdb_make_buf(params->nanoVdbTemperatures, params->nanoVdbTemperatureCount),
                    pnanovdb_make_buf(params->nanoVdbFuels, params->nanoVdbFuelCount),
                    pnanovdb_make_buf(params->nanoVdbBurns, params->nanoVdbBurnCount),
                    pnanovdb_make_buf(params->nanoVdbCoupleRateVelocities, params->nanoVdbCoupleRateVelocityCount),
                    pnanovdb_make_buf(params->nanoVdbCoupleRateDivergences, params->nanoVdbCoupleRateDivergenceCount),
                    pnanovdb_make_buf(params->nanoVdbCoupleRateTemperatures, params->nanoVdbCoupleRateTemperatureCount),
                    pnanovdb_make_buf(params->nanoVdbCoupleRateFuels, params->nanoVdbCoupleRateFuelCount),
                    pnanovdb_make_buf(params->nanoVdbCoupleRateBurns, params->nanoVdbCoupleRateBurnCount),
                    pnanovdb_make_buf(params->nanoVdbCoupleRateSmokes, params->nanoVdbCoupleRateSmokeCount),
                    pnanovdb_make_buf(params->nanoVdbDistances, params->nanoVdbDistanceCount),
                };

                for (NvFlowUint bufferIdx = 0u; bufferIdx < bufferCountFull; bufferIdx++)
                {
                    pnanovdb_buf_t buf = buffersFull[bufferIdx];
                    if (!buf.data || buf.size_in_words == 0llu)
                    {
                        continue;
                    }

                    EmitterNanoVdbAllocate_generateLeaves(ptr, inst, buf);

                    pnanovdb_grid_handle_t grid = { 0u };
                    pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
                    pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
                    pnanovdb_grid_type_t grid_type = pnanovdb_grid_get_grid_type(buf, grid);

                    for (NvFlowUint64 leafIdx = 0u; leafIdx < inst->leaves.size; leafIdx++)
                    {
                        pnanovdb_leaf_handle_t leaf = inst->leaves[leafIdx];

                        bool canSkip = false;
                        if (bufferIdx < 2u)
                        {
                            pnanovdb_address_t min_addr = pnanovdb_leaf_get_min_address(grid_type, buf, leaf);
                            pnanovdb_address_t max_addr = pnanovdb_leaf_get_max_address(grid_type, buf, leaf);
                            if (grid_type == PNANOVDB_GRID_TYPE_RGBA8)
                            {
                                float min_smoke = (1.f / 255.f) * ((float)(pnanovdb_read_uint32(buf, min_addr) >> 24u));
                                float max_smoke = (1.f / 255.f) * ((float)(pnanovdb_read_uint32(buf, max_addr) >> 24u));
                                if (max_smoke < params->minSmoke || min_smoke > params->maxSmoke)
                                {
                                    canSkip = true;
                                }
                            }
                            else if (grid_type == PNANOVDB_GRID_TYPE_FLOAT)
                            {
                                float min_smoke = pnanovdb_read_float(buf, min_addr);
                                float max_smoke = pnanovdb_read_float(buf, max_addr);
                                if (max_smoke < params->minSmoke || min_smoke > params->maxSmoke)
                                {
                                    canSkip = true;
                                }
                            }
                            else if (grid_type == PNANOVDB_GRID_TYPE_VEC4F)
                            {
                                float min_smoke = pnanovdb_read_float(buf, pnanovdb_address_offset(min_addr, 12u));
                                float max_smoke = pnanovdb_read_float(buf, pnanovdb_address_offset(max_addr, 12u));
                                if (max_smoke < params->minSmoke || min_smoke > params->maxSmoke)
                                {
                                    canSkip = true;
                                }
                            }
                        }
                        if (canSkip)
                        {
                            continue;
                        }

                        pnanovdb_coord_t bbox_min = pnanovdb_leaf_get_bbox_min(buf, leaf);
                        pnanovdb_uint32_t bbox_dif_raw = pnanovdb_leaf_get_bbox_dif_and_flags(buf, leaf);
                        pnanovdb_coord_t bbox_diff = {
                            (pnanovdb_int32_t)((bbox_dif_raw >> 0) & 255),
                            (pnanovdb_int32_t)((bbox_dif_raw >> 8) & 255),
                            (pnanovdb_int32_t)((bbox_dif_raw >> 16) & 255)
                        };
                        pnanovdb_vec3_t bbox_minf = {
                            (float)bbox_min.x,
                            (float)bbox_min.y,
                            (float)bbox_min.z
                        };
                        pnanovdb_vec3_t bbox_maxf = {
                            (float)(bbox_min.x + (bbox_diff.x < 8 ? bbox_diff.x : 7) + 1),
                            (float)(bbox_min.y + (bbox_diff.y < 8 ? bbox_diff.y : 7) + 1),
                            (float)(bbox_min.z + (bbox_diff.z < 8 ? bbox_diff.z : 7) + 1)
                        };

                        NvFlowFloat4 minWorldf = { 0.f, 0.f, 0.f, 0.f };
                        NvFlowFloat4 maxWorldf = { 0.f, 0.f, 0.f, 0.f };
                        for (NvFlowUint ptIdx = 0u; ptIdx < 8u; ptIdx++)
                        {
                            pnanovdb_vec3_t local = {
                                (ptIdx & 1) != 0u ? bbox_maxf.x : bbox_minf.x,
                                (ptIdx & 2) != 0u ? bbox_maxf.y : bbox_minf.y,
                                (ptIdx & 4) != 0u ? bbox_maxf.z : bbox_minf.z
                            };
                            pnanovdb_vec3_t vdb_worldf = pnanovdb_grid_index_to_worldf(buf, grid, PNANOVDB_REF(local));
                            NvFlowFloat4 worldf = { vdb_worldf.x, vdb_worldf.y, vdb_worldf.z, 1.f };
                            worldf = NvFlowMath::vector4Transform(worldf, params->localToWorld);
                            if (ptIdx == 0u)
                            {
                                minWorldf = worldf;
                                maxWorldf = worldf;
                            }
                            minWorldf = NvFlowMath::vectorMin(minWorldf, worldf);
                            maxWorldf = NvFlowMath::vectorMax(maxWorldf, worldf);
                        }

                        NvFlowFloat3 minLocationf = { minWorldf.x / blockSizeWorld.x, minWorldf.y / blockSizeWorld.y, minWorldf.z / blockSizeWorld.z };
                        NvFlowFloat3 maxLocationf = { maxWorldf.x / blockSizeWorld.x, maxWorldf.y / blockSizeWorld.y, maxWorldf.z / blockSizeWorld.z };

                        NvFlowInt4 minLocation = {
                            int(floorf(minLocationf.x)),
                            int(floorf(minLocationf.y)),
                            int(floorf(minLocationf.z)),
                            minLocation_w
                        };
                        NvFlowInt4 maxLocation = {
                            int(-floorf(-maxLocationf.x)),
                            int(-floorf(-maxLocationf.y)),
                            int(-floorf(-maxLocationf.z)),
                            maxLocation_w
                        };

                        for (int k = minLocation.z; k < maxLocation.z; k++)
                        {
                            for (int j = minLocation.y; j < maxLocation.y; j++)
                            {
                                for (int i = minLocation.x; i < maxLocation.x; i++)
                                {
                                    NvFlowInt4 location = NvFlowInt4{ i, j, k, minLocation.w };
                                    inst->locationHash.push(location, 0u);
                                    if (inst->locationHash.locations.size >= ptr->cachedMaxLocations)
                                    {
                                        k = maxLocation.z;
                                        j = maxLocation.y;
                                        i = maxLocation.x;
                                        bufferIdx = bufferCountFull;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                NvFlowInt4 locationMin = {};
                NvFlowInt4 locationMax = {};
                computeEmitterBoxBounds(
                    params->localToWorld,
                    params->layer,
                    params->level,
                    position,
                    halfSize,
                    blockSizeWorld,
                    &locationMin,
                    &locationMax
                );

                for (int k = locationMin.z; k < locationMax.z; k++)
                {
                    for (int j = locationMin.y; j < locationMax.y; j++)
                    {
                        for (int i = locationMin.x; i < locationMax.x; i++)
                        {
                            NvFlowInt4 location = NvFlowInt4{ i, j, k, locationMin.w };
                            inst->locationHash.push(location, 0u);
                            if (inst->locationHash.locations.size >= ptr->cachedMaxLocations)
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
    }

    void EmitterNanoVdb_cellSize(float* minCellSize, pnanovdb_buf_t buf)
    {
        if (buf.size_in_words > 0u)
        {
            pnanovdb_grid_handle_t grid = { 0u };
            pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
            pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

            pnanovdb_coord_t bbox_min = pnanovdb_root_get_bbox_min(buf, root);
            pnanovdb_coord_t bbox_max = pnanovdb_root_get_bbox_max(buf, root);

            if (bbox_max.x < bbox_min.x ||
                bbox_max.y < bbox_min.y ||
                bbox_max.z < bbox_min.z)
            {
                return;
            }

            float cellSize = (float)pnanovdb_grid_get_voxel_size(buf, grid, 0u);
            cellSize = fmaxf(cellSize, (float)pnanovdb_grid_get_voxel_size(buf, grid, 1u));
            cellSize = fmaxf(cellSize, (float)pnanovdb_grid_get_voxel_size(buf, grid, 2u));
            if (cellSize > 0.f && minCellSize)
            {
                if (*minCellSize == 0.f)
                {
                    *minCellSize = cellSize;
                }
                else if (cellSize < *minCellSize)
                {
                    *minCellSize = cellSize;
                }
            }
        }
    }

    void EmitterNanoVdbAllocate_execute(EmitterNanoVdbAllocate* ptr, const NvFlowEmitterNanoVdbAllocatePinsIn* in, NvFlowEmitterNanoVdbAllocatePinsOut* out)
    {
        const NvFlowSparseParams* in_sparseParams = &in->sparseSimParams.sparseParams;

        NvFlowUint maxLocations = 0u;
        if (in_sparseParams->levelCount > 0u)
        {
            maxLocations = in_sparseParams->levels[0u].maxLocations;
        }
        ptr->cachedMaxLocations = maxLocations;

        // generate table for fast LUID resolve
        NvFlowLuidTable_reset(&ptr->luidTableVol, in->volumeParamCount);
        for (NvFlowUint64 idx = 0u; idx < in->volumeParamCount; idx++)
        {
            NvFlowLuidTable_insert(&ptr->luidTableVol, in->volumeParams[idx]->luid, idx);
        }
        NvFlowLuidTable_reset(&ptr->luidTableAsset, in->nanoVdbAssetParamCount);
        for (NvFlowUint64 idx = 0u; idx < in->nanoVdbAssetParamCount; idx++)
        {
            NvFlowLuidTable_insert(&ptr->luidTableAsset, in->nanoVdbAssetParams[idx]->luid, idx);
        }

        ptr->globalUpdateVersion++;

        NvFlowUint64 pendingVdbLoadCount = 0llu;

        // refresh instances
        for (NvFlowUint64 paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
        {
            const NvFlowEmitterNanoVdbParams* paramsOrig = in->params[paramIdx];
            for (NvFlowUint64 volumeIdxU = 0u; volumeIdxU < 1u + paramsOrig->volumeLuidCount; volumeIdxU++)
            {
                NvFlowUint64 volumeIdx = volumeIdxU - 1u;
                NvFlowEmitterNanoVdbParams params = *paramsOrig;
                if (volumeIdx < paramsOrig->volumeLuidCount)
                {
                    // try to find volume with luid
                    NvFlowUint64 volumeParamsIdx = NvFlowLuidTable_find(&ptr->luidTableVol, params.volumeLuids[volumeIdx]);
                    if (volumeParamsIdx < in->volumeParamCount)
                    {
                        const auto volumeParams = in->volumeParams[volumeParamsIdx];

                        params.luid = volumeParams->luid;  // need unique luid, steal volume luid

                        NvFlowUint64 redIdx = NvFlowLuidTable_find(&ptr->luidTableAsset, volumeParams->nanoVdbRedLuid);
                        NvFlowUint64 greenIdx = NvFlowLuidTable_find(&ptr->luidTableAsset, volumeParams->nanoVdbGreenLuid);
                        NvFlowUint64 blueIdx = NvFlowLuidTable_find(&ptr->luidTableAsset, volumeParams->nanoVdbBlueLuid);
                        NvFlowUint64 alphaIdx = NvFlowLuidTable_find(&ptr->luidTableAsset, volumeParams->nanoVdbAlphaLuid);
                        NvFlowUint64 rgbaIdx = NvFlowLuidTable_find(&ptr->luidTableAsset, volumeParams->nanoVdbRgbaLuid);
                        NvFlowUint64 rgba8Idx = NvFlowLuidTable_find(&ptr->luidTableAsset, volumeParams->nanoVdbRgba8Luid);

                        if (redIdx < in->nanoVdbAssetParamCount)
                        {
                            const auto assetParams = in->nanoVdbAssetParams[redIdx];
                            params.nanoVdbTemperatures = assetParams->nanoVdbs;
                            params.nanoVdbTemperatureCount = assetParams->nanoVdbCount;
                            params.nanoVdbTemperatureVersion = assetParams->nanoVdbVersion;
                            params.nanoVdbTemperatureFirstElement = assetParams->nanoVdbFirstElement;
                            if (assetParams->nanoVdbPendingVersion != 0llu && assetParams->nanoVdbPendingVersion != assetParams->nanoVdbVersion)
                            {
                                pendingVdbLoadCount++;
                            }
                        }
                        if (greenIdx < in->nanoVdbAssetParamCount)
                        {
                            const auto assetParams = in->nanoVdbAssetParams[greenIdx];
                            params.nanoVdbFuels = assetParams->nanoVdbs;
                            params.nanoVdbFuelCount = assetParams->nanoVdbCount;
                            params.nanoVdbFuelVersion = assetParams->nanoVdbVersion;
                            params.nanoVdbFuelFirstElement = assetParams->nanoVdbFirstElement;
                            if (assetParams->nanoVdbPendingVersion != 0llu && assetParams->nanoVdbPendingVersion != assetParams->nanoVdbVersion)
                            {
                                pendingVdbLoadCount++;
                            }
                        }
                        if (blueIdx < in->nanoVdbAssetParamCount)
                        {
                            const auto assetParams = in->nanoVdbAssetParams[blueIdx];
                            params.nanoVdbBurns = assetParams->nanoVdbs;
                            params.nanoVdbBurnCount = assetParams->nanoVdbCount;
                            params.nanoVdbBurnVersion = assetParams->nanoVdbVersion;
                            params.nanoVdbBurnFirstElement = assetParams->nanoVdbFirstElement;
                            if (assetParams->nanoVdbPendingVersion != 0llu && assetParams->nanoVdbPendingVersion != assetParams->nanoVdbVersion)
                            {
                                pendingVdbLoadCount++;
                            }
                        }
                        if (alphaIdx < in->nanoVdbAssetParamCount)
                        {
                            const auto assetParams = in->nanoVdbAssetParams[alphaIdx];
                            params.nanoVdbSmokes = assetParams->nanoVdbs;
                            params.nanoVdbSmokeCount = assetParams->nanoVdbCount;
                            params.nanoVdbSmokeVersion = assetParams->nanoVdbVersion;
                            params.nanoVdbSmokeFirstElement = assetParams->nanoVdbFirstElement;
                            if (assetParams->nanoVdbPendingVersion != 0llu && assetParams->nanoVdbPendingVersion != assetParams->nanoVdbVersion)
                            {
                                pendingVdbLoadCount++;
                            }
                        }
                        if (rgbaIdx < in->nanoVdbAssetParamCount)
                        {
                            const auto assetParams = in->nanoVdbAssetParams[rgbaIdx];
                            params.nanoVdbRgba8s = assetParams->nanoVdbs;
                            params.nanoVdbRgba8Count = assetParams->nanoVdbCount;
                            params.nanoVdbRgba8Version = assetParams->nanoVdbVersion;
                            params.nanoVdbRgba8FirstElement = assetParams->nanoVdbFirstElement;
                            if (assetParams->nanoVdbPendingVersion != 0llu && assetParams->nanoVdbPendingVersion != assetParams->nanoVdbVersion)
                            {
                                pendingVdbLoadCount++;
                            }
                        }
                        if (rgba8Idx < in->nanoVdbAssetParamCount)
                        {
                            const auto assetParams = in->nanoVdbAssetParams[rgba8Idx];
                            params.nanoVdbRgba8s = assetParams->nanoVdbs;
                            params.nanoVdbRgba8Count = assetParams->nanoVdbCount;
                            params.nanoVdbRgba8Version = assetParams->nanoVdbVersion;
                            params.nanoVdbRgba8FirstElement = assetParams->nanoVdbFirstElement;
                            if (assetParams->nanoVdbPendingVersion != 0llu && assetParams->nanoVdbPendingVersion != assetParams->nanoVdbVersion)
                            {
                                pendingVdbLoadCount++;
                            }
                        }

                        params.localToWorld = volumeParams->localToWorld;
                        params.localToWorldVelocity = volumeParams->localToWorldVelocity;

                        if (paramsOrig->followVisibility)
                        {
                            params.enabled = paramsOrig->enabled && volumeParams->isVisible;
                        }
                        params.layer = paramsOrig->layer + volumeParams->layerOffset;
                        params.level = paramsOrig->level + volumeParams->levelOffset;
                    }
                }
                // auto level select, apply here so uniform everywhere
                if (params.levelCount > 1u)
                {
                    float pointWidthLocal = 0.f;
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbVelocities, params.nanoVdbVelocityCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbDivergences, params.nanoVdbDivergenceCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbTemperatures, params.nanoVdbTemperatureCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbFuels, params.nanoVdbFuelCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbBurns, params.nanoVdbBurnCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbSmokes, params.nanoVdbSmokeCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbCoupleRateVelocities, params.nanoVdbCoupleRateVelocityCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbCoupleRateDivergences, params.nanoVdbCoupleRateDivergenceCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbCoupleRateTemperatures, params.nanoVdbCoupleRateTemperatureCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbCoupleRateFuels, params.nanoVdbCoupleRateFuelCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbCoupleRateBurns, params.nanoVdbCoupleRateBurnCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbCoupleRateSmokes, params.nanoVdbCoupleRateSmokeCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbDistances, params.nanoVdbDistanceCount));
                    EmitterNanoVdb_cellSize(&pointWidthLocal, pnanovdb_make_buf(params.nanoVdbRgba8s, params.nanoVdbRgba8Count));

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
                            float cellSizeError = fabsf(worldCellSize - pointWidthWorld);
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
                // create instance
                {
                    NvFlowUint batchIdx = 0u;

                    // resolve instance
                    EmitterNanoVdbAllocateInstance* inst = nullptr;
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
                }
            }
        }

        // delete inactive instances
        NvFlowUint64 writeIdx = 0llu;
        NvFlowBool32 instanceReleased = NV_FLOW_FALSE;
        for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
        {
            EmitterNanoVdbAllocateInstance* inst = ptr->instances[instanceIdx];
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
            EmitterNanoVdbAllocateInstance* inst = ptr->instances[instanceIdx];
            const NvFlowEmitterNanoVdbParams* params = &inst->params;

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

            EmitterNanoVdbAllocateInstanceKey key;
            memset(&key, 0, sizeof(key));                           // explicit to cover any padding
            key.localToWorld = params->localToWorld;
            key.blockSizeWorld = blockSizeWorld;
            key.enabled = enabled;
            key.layerAndLevel = NvFlow_packLayerAndLevel(params->layer, params->level);
            key.allocationScale = params->allocationScale;
            key.enableStreaming = params->enableStreaming;
            key.streamOnce = params->streamOnce;
            key.streamClearAtStart = params->streamClearAtStart;
            key.pad = 0u;
            key.nanoVdbVelocityCount = params->nanoVdbVelocityCount;
            key.nanoVdbVelocityVersion = params->nanoVdbVelocityVersion;
            key.nanoVdbDivergenceCount = params->nanoVdbDivergenceCount;
            key.nanoVdbDivergenceVersion = params->nanoVdbDivergenceVersion;
            key.nanoVdbTemperatureCount = params->nanoVdbTemperatureCount;
            key.nanoVdbTemperatureVersion = params->nanoVdbTemperatureVersion;
            key.nanoVdbFuelCount = params->nanoVdbFuelCount;
            key.nanoVdbFuelVersion = params->nanoVdbFuelVersion;
            key.nanoVdbBurnCount = params->nanoVdbBurnCount;
            key.nanoVdbBurnVersion = params->nanoVdbBurnVersion;
            key.nanoVdbSmokeCount = params->nanoVdbSmokeCount;
            key.nanoVdbSmokeVersion = params->nanoVdbSmokeVersion;
            key.nanoVdbRgba8Count = params->nanoVdbRgba8Count;
            key.nanoVdbRgba8Version = params->nanoVdbRgba8Version;
            key.coupleRateVelocity = params->coupleRateVelocity;
            key.coupleRateDivergence = params->coupleRateDivergence;
            key.coupleRateTemperature = params->coupleRateTemperature;
            key.coupleRateFuel = params->coupleRateFuel;
            key.coupleRateBurn = params->coupleRateBurn;
            key.coupleRateSmoke = params->coupleRateSmoke;
            key.minSmoke = params->minSmoke;
            key.maxSmoke = params->maxSmoke;
            key.allocateActiveLeaves = params->allocateActiveLeaves;

            bool temperatureForceDirty = params->nanoVdbTemperatureCount > 0u && params->nanoVdbTemperatureVersion == 0llu;
            bool fuelForceDirty = params->nanoVdbFuelCount > 0u && params->nanoVdbFuelVersion == 0llu;
            bool burnForceDirty = params->nanoVdbBurnCount > 0u && params->nanoVdbBurnVersion == 0llu;
            bool smokeForceDirty = params->nanoVdbSmokeCount > 0u && params->nanoVdbSmokeVersion == 0llu;
            bool isDirty = (memcmp(&key, &inst->key, sizeof(key)) != 0u)
                || temperatureForceDirty || fuelForceDirty || burnForceDirty || smokeForceDirty;

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

                EmitterNanoVdbAllocate_compute(ptr, inst, in);
            }
        }

        if (ptr->globalLocationHashVersion != ptr->globalChangeVersion)
        {
            ptr->globalLocationHashVersion = ptr->globalChangeVersion;

            ptr->globalLocationHash.reset();

            for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instances.size; instanceIdx++)
            {
                EmitterNanoVdbAllocateInstance* inst = ptr->instances[instanceIdx];

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
                EmitterNanoVdbAllocateInstance* inst = ptr->instances[instanceIdx];

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
                    bool isActive = ptr->params[instanceIdx]->nanoVdbDistanceCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbVelocityCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbDivergenceCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbTemperatureCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbFuelCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbBurnCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbSmokeCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbCoupleRateVelocityCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbCoupleRateDivergenceCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbCoupleRateTemperatureCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbCoupleRateFuelCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbCoupleRateBurnCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbCoupleRateSmokeCount > 0u ||
                        ptr->params[instanceIdx]->nanoVdbRgba8Count > 0u;
                    if (isActive)
                    {
                        anyStreamingClearEnabled = NV_FLOW_TRUE;
                        break;
                    }
                }
            }

            ptr->feedback.globalUpdateVersion = ptr->globalUpdateVersion;
            ptr->feedback.globalChangeVersion = ptr->globalChangeVersion;
            ptr->feedback.anyStreamingEnabled = anyStreamingEnabled;
            ptr->feedback.anyStreamingDisabled = anyStreamingDisabled;
            ptr->feedback.anyStreamingClearEnabled = anyStreamingClearEnabled;

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
        ptr->feedback.pendingVdbLoadCount = pendingVdbLoadCount;
        ptr->feedback.reportUpdate = EmitterNanoVdbAllocate_reportUpdate;

        out->locations = ptr->locations.data;
        out->locationCount = ptr->locations.size;
        out->feedback.data = &ptr->feedback;
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterNanoVdbAllocate, EmitterNanoVdbAllocate)
