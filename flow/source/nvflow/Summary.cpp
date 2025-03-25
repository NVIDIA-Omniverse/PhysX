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

#include "shaders/SummaryParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"
#include "NvFlowDynamicBuffer.h"
#include "NvFlowReadbackBuffer.h"

#include "NvFlow.h"

#include "shaders/SummaryCS.hlsl.h"

namespace
{
    struct SummaryAllocate;

    struct Summary
    {
        NvFlowContextInterface contextInterface = {};

        SummaryCS_Pipeline summaryCS = {};

        NvFlowUploadBuffer constantBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};

        NvFlowDynamicBuffer summaryBuffer = {};
        NvFlowReadbackBuffer summaryReadback = {};
        NvFlowUint64 minSummaryVersion = 0llu;

        NvFlowArray<int> forceClearLayerAndLevels;
        NvFlowArray<NvFlowUint64> forceClearMinSummaryVersions;

        NvFlowArray<NvFlowInt4> locations;
        NvFlowArray<NvFlowFloat3> layerScales;

        SummaryAllocate* allocate = nullptr;
    };

    struct SummaryAllocate
    {
        Summary* summary = nullptr;
    };

    Summary* Summary_create(const NvFlowOpInterface* opInterface, const NvFlowSummaryPinsIn* in, NvFlowSummaryPinsOut* out)
    {
        auto ptr = new Summary();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        SummaryCS_init(&ptr->contextInterface, in->context, &ptr->summaryCS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(SummaryCS_LayerParams));

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->summaryBuffer, eNvFlowBufferUsage_rwStructuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->summaryReadback);

        return ptr;
    }

    void Summary_destroy(Summary* ptr, const NvFlowSummaryPinsIn* in, NvFlowSummaryPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->summaryBuffer);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->summaryReadback);

        SummaryCS_destroy(in->context, &ptr->summaryCS);

        delete ptr;
    }

    void Summary_execute(Summary* ptr, const NvFlowSummaryPinsIn* in, NvFlowSummaryPinsOut* out)
    {
        // establish readback link
        ptr->allocate = (SummaryAllocate*)in->feedback.data;
        ptr->allocate->summary = ptr;

        NvFlowSparseLevelParams* levelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];
        NvFlowUint layerCount = in->velocity.sparseParams.layerCount;

        NvFlowUint mappedLayerCount = layerCount == 0u ? 1u : layerCount;
        auto mappedLayer = (SummaryCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, mappedLayerCount * sizeof(SummaryCS_LayerParams));
        if (layerCount == 0u)
        {
            SummaryCS_LayerParams nullParams = {};
            nullParams.forceClear = NV_FLOW_TRUE;
            mappedLayer[0u] = nullParams;
        }

        NvFlowBool32 anyEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyNeighborAllocation = NV_FLOW_FALSE;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            if (layerParamsIn->enabled)
            {
                anyEnabled = NV_FLOW_TRUE;
            }
            if (layerParamsIn->enableNeighborAllocation)
            {
                anyNeighborAllocation = NV_FLOW_TRUE;
            }

            mappedLayer[layerParamIdx].smokeThreshold = layerParamsIn->smokeThreshold;
            mappedLayer[layerParamIdx].speedThresholdMinSmoke = layerParamsIn->speedThresholdMinSmoke;
            mappedLayer[layerParamIdx].speedThreshold = layerParamsIn->speedThreshold;
            mappedLayer[layerParamIdx].enableNeighborAllocation = layerParamsIn->enableNeighborAllocation;

            mappedLayer[layerParamIdx].forceClear = in->velocity.sparseParams.layers[layerParamIdx].forceClear;
            mappedLayer[layerParamIdx].layerAndLevel = in->velocity.sparseParams.layers[layerParamIdx].layerAndLevel;
            mappedLayer[layerParamIdx].enabled = layerParamsIn->enabled;
            mappedLayer[layerParamIdx].pad3 = 0u;

            mappedLayer[layerParamIdx].blockSizeWorld = in->velocity.sparseParams.layers[layerParamIdx].blockSizeWorld;
            mappedLayer[layerParamIdx].pad4 = 0u;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        NvFlowUint locationOffset = 4u + 4u * layerCount;
        NvFlowUint64 summaryWords = locationOffset + 5u * levelParams->numLocations;
        if (!anyEnabled)
        {
            summaryWords = locationOffset;
        }
        NvFlowUint64 summarySize = summaryWords * sizeof(NvFlowUint);

        NvFlowDynamicBuffer_resize(in->context, &ptr->summaryBuffer, summarySize);
        NvFlowBufferTransient* summaryTransient = NvFlowDynamicBuffer_getTransient(in->context, &ptr->summaryBuffer);

        NvFlowUint threadBlockCount = levelParams->numLocations;
        if (threadBlockCount == 0u || !anyEnabled)
        {
            threadBlockCount = 1u;
        }

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, threadBlockCount);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (SummaryCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(SummaryCS_GlobalParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->locationOffset = locationOffset;
            mapped->layerCount = layerCount;
            mapped->anyNeighborAllocation = anyNeighborAllocation;

            mapped->anyEnabled = anyEnabled;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;

            mapped->table = *levelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);

            SummaryCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gLayerParams = layerTransient;
            params.gTable = in->velocity.sparseBuffer;
            params.velocityIn = in->velocity.textureTransient;
            params.densityCoarseIn = in->densityCoarse.textureTransient;
            params.summaryOut = summaryTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            SummaryCS_addPassCompute(in->context, &ptr->summaryCS, gridDim, &params);
        }

        NvFlowUint64 summaryVersion = ~0llu;
        NvFlowReadbackBuffer_copy(in->context, &ptr->summaryReadback, summarySize, summaryTransient, &summaryVersion);
    }

    SummaryAllocate* SummaryAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowSummaryAllocatePinsIn* in, NvFlowSummaryAllocatePinsOut* out)
    {
        auto ptr = new SummaryAllocate();

        ptr->summary = nullptr;

        return ptr;
    }

    void SummaryAllocate_destroy(SummaryAllocate* ptr, const NvFlowSummaryAllocatePinsIn* in, NvFlowSummaryAllocatePinsOut* out)
    {
        delete ptr;
    }

    void SummaryAllocate_execute(SummaryAllocate* ptrAllocate, const NvFlowSummaryAllocatePinsIn* in, NvFlowSummaryAllocatePinsOut* out)
    {
        out->feedback.data = ptrAllocate;

        auto ptr = ptrAllocate->summary;
        if (!ptr)
        {
            out->locations = nullptr;
            out->locationCount = 0u;
            return;
        }

        const NvFlowSparseSimParams* sparseSimParams = &in->sparseSimParams;
        const NvFlowSparseParams* sparseParams = &sparseSimParams->sparseParams;

        NvFlowBool32 allForceClear = NV_FLOW_TRUE;
        for (NvFlowUint64 layerIdx = 0u; layerIdx < sparseParams->layerCount; layerIdx++)
        {
            if (!sparseParams->layers[layerIdx].forceClear)
            {
                allForceClear = NV_FLOW_FALSE;
                break;
            }
        }
        if (allForceClear)
        {
            ptr->minSummaryVersion = ptr->summaryReadback.versionCounter;
        }

        NvFlowUint64 mappedVersion = 0llu;
        NvFlowUint64 mappedNumBytes = 0llu;
        const NvFlowUint* mapped = (const NvFlowUint*)NvFlowReadbackBuffer_mapLatest(in->context, &ptr->summaryReadback, &mappedVersion, &mappedNumBytes);

        if (!mapped)
        {
            out->locations = nullptr;
            out->locationCount = 0llu;
            return;
        }
        if (mapped && mappedVersion <= ptr->minSummaryVersion)
        {
            NvFlowReadbackBuffer_unmapLatest(in->context, &ptr->summaryReadback);
            out->locations = nullptr;
            out->locationCount = 0llu;
            return;
        }

        // per layer force clear
        for (NvFlowUint64 layerIdx = 0u; layerIdx < sparseParams->layerCount; layerIdx++)
        {
            const NvFlowSparseLayerParams* layerParams = &sparseParams->layers[layerIdx];
            if (layerParams->forceClear)
            {
                NvFlowUint64 forceClearIdx = 0u;
                for (; forceClearIdx < ptr->forceClearLayerAndLevels.size; forceClearIdx++)
                {
                    if (ptr->forceClearLayerAndLevels[forceClearIdx] == layerParams->layerAndLevel)
                    {
                        break;
                    }
                }
                if (forceClearIdx == ptr->forceClearLayerAndLevels.size)
                {
                    ptr->forceClearLayerAndLevels.pushBack(layerParams->layerAndLevel);
                    ptr->forceClearMinSummaryVersions.pushBack(~0llu);
                }
                ptr->forceClearMinSummaryVersions[forceClearIdx] = ptr->summaryReadback.versionCounter;
            }
        }
        // remove old forceClear entries
        if (ptr->forceClearLayerAndLevels.size > 0u)
        {
            NvFlowUint64 writeIdx = 0u;
            for (NvFlowUint64 readIdx = 0u; readIdx < ptr->forceClearMinSummaryVersions.size; readIdx++)
            {
                int layerAndLevel = ptr->forceClearLayerAndLevels[readIdx];
                NvFlowUint64 minSummaryVersion = ptr->forceClearMinSummaryVersions[readIdx];
                if (mappedVersion <= minSummaryVersion)
                {
                    ptr->forceClearLayerAndLevels[writeIdx] = layerAndLevel;
                    ptr->forceClearMinSummaryVersions[writeIdx] = minSummaryVersion;
                    writeIdx++;
                }
            }
            ptr->forceClearLayerAndLevels.size = writeIdx;
            ptr->forceClearMinSummaryVersions.size = writeIdx;
        }

        const SummaryCS_Header* mappedHeader = (const SummaryCS_Header*)(&mapped[0u]);
        const SummaryCS_HeaderLayer* mappedLayers = (const SummaryCS_HeaderLayer*)(&mapped[4u]);
        const SummaryCS_Location* mappedLocations = (const SummaryCS_Location*)(&mapped[4u + 4u * mappedHeader->layerCount]);

        NvFlowUint mappedLayerCount = mappedHeader->layerCount;
        NvFlowUint mappedNumLocations = mappedHeader->numLocations;
        NvFlowBool32 mappedAnyNeighborAllocation = mappedHeader->anyNeighborAllocation;

        // todo, sanity check pointers against readback size

        // temporary manual allocation
        ptr->layerScales.size = 0u;
        ptr->locations.size = 0u;

        // try to detect layer rescale events
        NvFlowBool32 anyRescaleEvent = NV_FLOW_FALSE;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < sparseParams->layerCount; layerParamIdx++)
        {
            const auto layerParams = &sparseParams->layers[layerParamIdx];
            NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
            NvFlowFloat3 blockSizeWorldOld = layerParams->blockSizeWorld;
            for (NvFlowUint64 layerParamIdxOld = 0u; layerParamIdxOld < mappedLayerCount; layerParamIdxOld++)
            {
                if (mappedLayers[layerParamIdxOld].layerAndLevel == layerParams->layerAndLevel)
                {
                    blockSizeWorldOld = mappedLayers[layerParamIdxOld].blockSizeWorld;
                    break;
                }
            }
            NvFlowFloat3 layerScale = {
                 blockSizeWorldOld.x / blockSizeWorld.x,
                 blockSizeWorldOld.y / blockSizeWorld.y,
                 blockSizeWorldOld.z / blockSizeWorld.z
            };
            ptr->layerScales.pushBack(layerScale);
            if (blockSizeWorld.x != blockSizeWorldOld.x ||
                blockSizeWorld.y != blockSizeWorldOld.y ||
                blockSizeWorld.z != blockSizeWorldOld.z)
            {
                anyRescaleEvent = NV_FLOW_TRUE;
            }
        }
        if (anyRescaleEvent)
        {
            auto pushLocation = [&](const NvFlowFloat3& layerScale, NvFlowInt4 location)
            {
                // rescale location
                NvFlowFloat3 locationMinf = {
                    float(location.x) * layerScale.x,
                    float(location.y) * layerScale.y,
                    float(location.z) * layerScale.z
                };
                NvFlowFloat3 locationMaxf = {
                    float(location.x + 1) * layerScale.x,
                    float(location.y + 1) * layerScale.y,
                    float(location.z + 1) * layerScale.z
                };
                NvFlowInt3 locationMin = {
                    int(floorf(locationMinf.x)),
                    int(floorf(locationMinf.y)),
                    int(floorf(locationMinf.z))
                };
                NvFlowInt3 locationMax = {
                    int(-floorf(-locationMaxf.x)),
                    int(-floorf(-locationMaxf.y)),
                    int(-floorf(-locationMaxf.z))
                };
                for (int k = locationMin.z; k < locationMax.z; k++)
                {
                    for (int j = locationMin.y; j < locationMax.y; j++)
                    {
                        for (int i = locationMin.x; i < locationMax.x; i++)
                        {
                            NvFlowInt4 locationTemp = { i, j, k, location.w };
                            ptr->locations.pushBack(locationTemp);
                        }
                    }
                }
            };

            // auto allocate
            for (NvFlowUint idx = 0u; idx < mappedNumLocations; idx++)
            {
                const SummaryCS_Location* location = mappedLocations + idx;

                NvFlowFloat3 layerScale = { 1.f, 1.f, 1.f };
                for (NvFlowUint layerParamIdx = 0u; layerParamIdx < sparseParams->layerCount; layerParamIdx++)
                {
                    if (sparseParams->layers[layerParamIdx].layerAndLevel == location->location.w)
                    {
                        layerScale = ptr->layerScales[layerParamIdx];
                        break;
                    }
                }

                if (location->mask & 0x0003)
                {
                    pushLocation(layerScale, location->location);
                }
                if (location->mask & 0x0004)
                {
                    NvFlowInt4 localLocation = location->location;
                    localLocation.x -= 1;
                    pushLocation(layerScale, localLocation);
                }
                if (location->mask & 0x0008)
                {
                    NvFlowInt4 localLocation = location->location;
                    localLocation.x += 1;
                    pushLocation(layerScale, localLocation);
                }
                if (location->mask & 0x0010)
                {
                    NvFlowInt4 localLocation = location->location;
                    localLocation.y -= 1;
                    pushLocation(layerScale, localLocation);
                }
                if (location->mask & 0x0020)
                {
                    NvFlowInt4 localLocation = location->location;
                    localLocation.y += 1;
                    pushLocation(layerScale, localLocation);
                }
                if (location->mask & 0x0040)
                {
                    NvFlowInt4 localLocation = location->location;
                    localLocation.z -= 1;
                    pushLocation(layerScale, localLocation);
                }
                if (location->mask & 0x0080)
                {
                    NvFlowInt4 localLocation = location->location;
                    localLocation.z += 1;
                    pushLocation(layerScale, localLocation);
                }
            }
        }
        else
        {
            if (mappedAnyNeighborAllocation)
            {
                // auto allocate
                for (NvFlowUint idx = 0u; idx < mappedNumLocations; idx++)
                {
                    const SummaryCS_Location* location = mappedLocations + idx;

                    if (location->mask & 0x0003)
                    {
                        ptr->locations.pushBack(location->location);
                    }
                    if (location->mask & 0x0004)
                    {
                        NvFlowInt4 localLocation = location->location;
                        localLocation.x -= 1;
                        ptr->locations.pushBack(localLocation);
                    }
                    if (location->mask & 0x0008)
                    {
                        NvFlowInt4 localLocation = location->location;
                        localLocation.x += 1;
                        ptr->locations.pushBack(localLocation);
                    }
                    if (location->mask & 0x0010)
                    {
                        NvFlowInt4 localLocation = location->location;
                        localLocation.y -= 1;
                        ptr->locations.pushBack(localLocation);
                    }
                    if (location->mask & 0x0020)
                    {
                        NvFlowInt4 localLocation = location->location;
                        localLocation.y += 1;
                        ptr->locations.pushBack(localLocation);
                    }
                    if (location->mask & 0x0040)
                    {
                        NvFlowInt4 localLocation = location->location;
                        localLocation.z -= 1;
                        ptr->locations.pushBack(localLocation);
                    }
                    if (location->mask & 0x0080)
                    {
                        NvFlowInt4 localLocation = location->location;
                        localLocation.z += 1;
                        ptr->locations.pushBack(localLocation);
                    }
                }
            }
            else // optimization if no neighbor allocation
            {
                // auto allocate
                for (NvFlowUint idx = 0u; idx < mappedNumLocations; idx++)
                {
                    const SummaryCS_Location* location = mappedLocations + idx;

                    if (location->mask & 0x0003)
                    {
                        ptr->locations.pushBack(location->location);
                    }
                }
            }
        }

        // purge if layer has force clear enabled
        for (NvFlowUint64 forceClearIdx = 0u; forceClearIdx < ptr->forceClearLayerAndLevels.size; forceClearIdx++)
        {
            int layerAndLevel = ptr->forceClearLayerAndLevels[forceClearIdx];
            NvFlowUint64 writeIdx = 0u;
            for (NvFlowUint64 readIdx = 0u; readIdx < ptr->locations.size; readIdx++)
            {
                NvFlowInt4 location = ptr->locations[readIdx];
                if (location.w != layerAndLevel)
                {
                    ptr->locations[writeIdx] = location;
                    writeIdx++;
                }
            }
            ptr->locations.size = writeIdx;
        }
        // purge if force clear on rescale
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < sparseParams->layerCount; layerParamIdx++)
        {
            const auto layerParams = &sparseParams->layers[layerParamIdx];
            const auto simLayerParams = &sparseSimParams->layers[layerParamIdx];
            if (simLayerParams->clearOnRescale)
            {
                NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
                NvFlowFloat3 blockSizeWorldOld = layerParams->blockSizeWorld;
                for (NvFlowUint64 layerParamIdxOld = 0u; layerParamIdxOld < mappedLayerCount; layerParamIdxOld++)
                {
                    if (mappedLayers[layerParamIdxOld].layerAndLevel == layerParams->layerAndLevel)
                    {
                        blockSizeWorldOld = mappedLayers[layerParamIdxOld].blockSizeWorld;
                        break;
                    }
                }
                if (blockSizeWorld.x != blockSizeWorldOld.x ||
                    blockSizeWorld.y != blockSizeWorldOld.y ||
                    blockSizeWorld.z != blockSizeWorldOld.z)
                {
                    int layerAndLevel = layerParams->layerAndLevel;
                    NvFlowUint64 writeIdx = 0u;
                    for (NvFlowUint64 readIdx = 0u; readIdx < ptr->locations.size; readIdx++)
                    {
                        NvFlowInt4 location = ptr->locations[readIdx];
                        if (location.w != layerAndLevel)
                        {
                            ptr->locations[writeIdx] = location;
                            writeIdx++;
                        }
                    }
                    ptr->locations.size = writeIdx;
                }
            }
        }

        NvFlowReadbackBuffer_unmapLatest(in->context, &ptr->summaryReadback);

        out->locations = ptr->locations.data;
        out->locationCount = ptr->locations.size;
    }
}

NV_FLOW_OP_IMPL(NvFlowSummary, Summary)
NV_FLOW_OP_IMPL(NvFlowSummaryAllocate, SummaryAllocate)
