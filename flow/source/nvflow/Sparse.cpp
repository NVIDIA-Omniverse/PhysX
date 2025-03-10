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

#include "shaders/SparseParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"
#include "NvFlowDynamicBuffer.h"

#include "NvFlow.h"

#include "shaders/SparseCS.hlsl.h"
#include "shaders/SparseUploadCS.hlsl.h"
#include "shaders/SparseClearNew1CS.hlsl.h"
#include "shaders/SparseClearNew2CS.hlsl.h"
#include "shaders/SparseClearNew4CS.hlsl.h"
#include "shaders/SparseClearTexture1CS.hlsl.h"
#include "shaders/SparseClearTexture2CS.hlsl.h"
#include "shaders/SparseClearTexture4CS.hlsl.h"

#include "shaders/SparseRescale1CS.hlsl.h"
#include "shaders/SparseRescale2CS.hlsl.h"
#include "shaders/SparseRescale4CS.hlsl.h"

#include "shaders/SparseInplaceRescale4aCS.hlsl.h"
#include "shaders/SparseInplaceRescale4bCS.hlsl.h"
#include "shaders/SparseInplaceRescale4cCS.hlsl.h"
#include "shaders/SparseInplaceRescale4dCS.hlsl.h"

#include "shaders/SparseClearLayers1CS.hlsl.h"
#include "shaders/SparseClearLayers2CS.hlsl.h"
#include "shaders/SparseClearLayers4CS.hlsl.h"

#include "shaders/SparseNanoVdbPruneLeavesCS.hlsl.h"

#include "shaders/SparseNanoVdbComputeStats0CS.hlsl.h"
#include "shaders/SparseNanoVdbComputeStats1CS.hlsl.h"
#include "shaders/SparseNanoVdbComputeStats2CS.hlsl.h"
#include "shaders/SparseNanoVdbComputeStats3CS.hlsl.h"

#include "NvFlowLocationHashTable.h"

#include "NvFlowTimer.h"

#define PNANOVDB_C
#define PNANOVDB_BUF_BOUNDS_CHECK

#include "nanovdb/PNanoVDB.h"

#include <string.h>

//#define SPARSE_FILEWRITE
#ifdef SPARSE_FILEWRITE
#include <stdio.h>
#if !defined(_WIN32)
NV_FLOW_INLINE void fopen_s(FILE** streamptr, const char* filename, const char* mode)
{
    *streamptr = fopen(filename, mode);
}
#endif
#endif

namespace NvFlowSparseDefault
{
    struct SparseNanoVdbLayer
    {
        pnanovdb_grid_handle_t grid = {};
        pnanovdb_tree_handle_t tree = {};
        pnanovdb_root_handle_t root = {};

        pnanovdb_uint32_t tile_count = 0u;
        pnanovdb_uint32_t upper_count = 0u;
        pnanovdb_uint32_t lower_count = 0u;
        pnanovdb_uint32_t leaf_count = 0u;
    };

    struct SparseNanoVdb
    {
        NvFlowUint levelIdx = 0;
        NvFlowUint gridType = 0;

        NvFlowArray<NvFlowUint8> buf_array;
        pnanovdb_buf_t buf = {};
        NvFlowUint64 buffer_size_without_leaves = 0u;
        NvFlowUint64 buffer_size_with_leaves = 0u;
        NvFlowUint64 cache_size = 0u;

        NvFlowArray<SparseNanoVdbLayer> perLayers;
        NvFlowSparseNanoVdbParams paramsNanoVdb;

        NvFlowArray<NvFlowUint> layerParamsIndices;
        NvFlowArray<pnanovdb_coord_t> coarseCoords;
        NvFlowArray<pnanovdb_coord_t> fineOffsets;
        NvFlowArray<pnanovdb_root_tile_handle_t> cachedTiles;
        NvFlowArray<pnanovdb_upper_handle_t> cachedUpper;
        NvFlowArray<pnanovdb_lower_handle_t> cachedLower;
        NvFlowArray<pnanovdb_leaf_handle_t> cachedLeaf;

        NvFlowArray<pnanovdb_root_tile_handle_t> listTiles;
        NvFlowArray<pnanovdb_upper_handle_t> listUpper;
        NvFlowArray<pnanovdb_lower_handle_t> listLower;
        NvFlowArray<pnanovdb_leaf_handle_t> listLeaf;

        NvFlowUploadBuffer uploadBufferNanoVdb = {};
        NvFlowUploadBuffer uploadBufferCache = {};
    };

    struct SparseDiffTask
    {
        NvFlowLocationHashTable diffNewLocations;
        NvFlowUint beginIdx;
        NvFlowUint endIdx;
        NvFlowInt4* locations;
    };

    struct Sparse
    {
        NvFlowContextInterface contextInterface = {};

        NvFlowArray<NvFlowUint> layerParamIdxs;

        NvFlowUint3 maxPoolDim = { 0u, 0u, 0u };
        NvFlowUint maxPoolDim_xy = 0u;
        NvFlowUint maxPoolDim3 = 0u;
        NvFlowUint maxLocations = 0u;
        NvFlowBool32 resetPending = NV_FLOW_FALSE;
        NvFlowUint targetMaxLocations = 0u;

        NvFlowArray<NvFlowInt4> allocationLocations;
        NvFlowArray<NvFlowUint> allocationActives;
        NvFlowArray<NvFlowUint> freeList;

        NvFlowArray<NvFlowUint> newList;
        NvFlowArray<NvFlowUint> allocations;
        NvFlowLocationHashTable hashTable;
        NvFlowArray<NvFlowUint2> tableRanges;

        NvFlowArray<NvFlowFloat3> layerScales;
        NvFlowArray<NvFlowInt4> rescaleLocations;

        NvFlowBool32 diffFirstRun = NV_FLOW_TRUE;
        NvFlowUint64 oldDiffNewLocations = 0llu;
        NvFlowArray<NvFlowUint> diffMasks;
        NvFlowLocationHashTable diffNewLocations;
        NvFlowArray<SparseDiffTask> sparseDiffTasks;

        NvFlowUint64 tableVersion = 1llu;
        NvFlowUint64 uploadedTableVersion = 0llu;

        NvFlowUint3 baseBlockDimBits = { 0u, 0u, 0u };
        NvFlowUint numBlockLevels = 0u;

        NvFlowArray<NvFlowSparseLevelParams> levelParams;
        NvFlowArray<NvFlowSparseLevelParams> levelParamsOld;
        NvFlowArray<NvFlowSparseLayerParams> layerParams;
        NvFlowArray<NvFlowSparseLayerParams> layerParamsOld;
        NvFlowArray<NvFlowSparseSimLayerParams> simLayerParams;
        NvFlowUint totalSizeInBytes = 0u;

        NvFlowUint64 updateLayerId = 0llu;
        NvFlowUint64 updateLocationsId = 0llu;

        NvFlowArray<NvFlowBool32> layerIsCleared;

        NvFlowUint uploadSizeInBytes = 0u;

        NvFlowUploadBuffer uploadBuffer;
        NvFlowUploadBuffer constantBuffer;
        NvFlowUploadBuffer rescaleBuffer;

        NvFlowSampler* samplerLinear = nullptr;

        SparseCS_Pipeline sparseCS;
        SparseUploadCS_Pipeline sparseUploadCS;
        SparseClearNew1CS_Pipeline sparseClearNew1CS;
        SparseClearNew2CS_Pipeline sparseClearNew2CS;
        SparseClearNew4CS_Pipeline sparseClearNew4CS;
        SparseClearTexture1CS_Pipeline sparseClearTexture1CS;
        SparseClearTexture2CS_Pipeline sparseClearTexture2CS;
        SparseClearTexture4CS_Pipeline sparseClearTexture4CS;

        SparseRescale1CS_Pipeline sparseRescale1CS;
        SparseRescale2CS_Pipeline sparseRescale2CS;
        SparseRescale4CS_Pipeline sparseRescale4CS;

        SparseInplaceRescale4aCS_Pipeline sparseInplaceRescale4aCS;
        SparseInplaceRescale4bCS_Pipeline sparseInplaceRescale4bCS;
        SparseInplaceRescale4cCS_Pipeline sparseInplaceRescale4cCS;
        SparseInplaceRescale4dCS_Pipeline sparseInplaceRescale4dCS;

        SparseClearLayers1CS_Pipeline sparseClearLayers1CS;
        SparseClearLayers2CS_Pipeline sparseClearLayers2CS;
        SparseClearLayers4CS_Pipeline sparseClearLayers4CS;

        SparseNanoVdbPruneLeavesCS_Pipeline sparseNanoVdbPruneLeaves;

        SparseNanoVdbComputeStats0CS_Pipeline sparseNanoVdbComputeStats0;
        SparseNanoVdbComputeStats1CS_Pipeline sparseNanoVdbComputeStats1;
        SparseNanoVdbComputeStats2CS_Pipeline sparseNanoVdbComputeStats2;
        SparseNanoVdbComputeStats3CS_Pipeline sparseNanoVdbComputeStats3;

        NvFlowDynamicBuffer sparseTableBuffer = {};
        NvFlowDynamicBuffer sparseBuffer[2u] = {};
        NvFlowUint sparseBufferIdx = 0u;

        NvFlowBool32 isVulkan = NV_FLOW_FALSE;

        NvFlowArrayPointer<SparseNanoVdb*> nanoVdbs;

        NvFlowUint formatChannelCount[eNvFlowFormat_count] = {};
    };

    NV_FLOW_CAST_PAIR(NvFlowSparse, Sparse)

    NvFlowBool32 getParams(NvFlowSparse* sparse, NvFlowSparseParams* out);

    void generateLayerLocationRanges(Sparse* ptr, NvFlowUint numLocations, const NvFlowInt4* locations)
    {
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            auto layerParam = &ptr->layerParams[layerParamIdx];
            layerParam->locationMin = NvFlowInt4{ 0, 0, 0, 0 };
            layerParam->locationMax = NvFlowInt4{ 0, 0, 0, 0 };
        }

        NvFlowSparseParams sparseParams = {};
        getParams(cast(ptr), &sparseParams);

        // compute location range
        for (NvFlowUint locationIdx = 0u; locationIdx < numLocations; locationIdx++)
        {
            NvFlowInt4 location = locations[locationIdx];

            NvFlowInt2 layerAndLevel = NvFlow_unpackLayerAndLevel(location.w);
            NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&sparseParams, layerAndLevel.x, layerAndLevel.y);
            if (layerParamIdx == ~0u)
            {
                continue;
            }

            auto layerParam = &ptr->layerParams[layerParamIdx];

            if (layerParam->locationMin.w == 0 &&
                layerParam->locationMax.w == 0)
            {
                layerParam->locationMin = location;
                layerParam->locationMax.x = location.x + 1;
                layerParam->locationMax.y = location.y + 1;
                layerParam->locationMax.z = location.z + 1;
                layerParam->locationMax.w = location.w + 1;
            }

            if (location.x < layerParam->locationMin.x)
            {
                layerParam->locationMin.x = location.x;
            }
            if (location.y < layerParam->locationMin.y)
            {
                layerParam->locationMin.y = location.y;
            }
            if (location.z < layerParam->locationMin.z)
            {
                layerParam->locationMin.z = location.z;
            }
            if (location.w < layerParam->locationMin.w)
            {
                layerParam->locationMin.w = location.w;
            }

            // plus one, since max is exclusive
            if (location.x + 1 > layerParam->locationMax.x)
            {
                layerParam->locationMax.x = location.x + 1;
            }
            if (location.y + 1 > layerParam->locationMax.y)
            {
                layerParam->locationMax.y = location.y + 1;
            }
            if (location.z + 1 > layerParam->locationMax.z)
            {
                layerParam->locationMax.z = location.z + 1;
            }
            if (location.w + 1 > layerParam->locationMax.w)
            {
                layerParam->locationMax.w = location.w + 1;
            }
        }

        // update world space bounds
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            auto layerParam = &ptr->layerParams[layerParamIdx];
            layerParam->worldMin.x = layerParam->blockSizeWorld.x * float(layerParam->locationMin.x);
            layerParam->worldMin.y = layerParam->blockSizeWorld.y * float(layerParam->locationMin.y);
            layerParam->worldMin.z = layerParam->blockSizeWorld.z * float(layerParam->locationMin.z);
            layerParam->worldMax.x = layerParam->blockSizeWorld.x * float(layerParam->locationMax.x);
            layerParam->worldMax.y = layerParam->blockSizeWorld.y * float(layerParam->locationMax.y);
            layerParam->worldMax.z = layerParam->blockSizeWorld.z * float(layerParam->locationMax.z);
        }
    }

    NvFlowUint computeAllocationValue(Sparse* ptr, NvFlowUint allocationIdx)
    {
        NvFlowUint3 poolIdx = {
            (allocationIdx) % ptr->maxPoolDim.x,
            (allocationIdx / ptr->maxPoolDim.x) % ptr->maxPoolDim.y,
            (allocationIdx / ptr->maxPoolDim_xy) % ptr->maxPoolDim.z
        };
        return (poolIdx.z << 20u) | (poolIdx.y << 10u) | (poolIdx.x);
    }

    void updateLayers(NvFlowSparse* sparse, NvFlowUint64 updateId, NvFlowSparseUpdateLayerParams* layers, NvFlowUint numLayers);
    void testUpdateLocations(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowUint64 updateId,
        NvFlowInt4* locations,
        NvFlowUint numLocations,
        NvFlowUint3 baseBlockDimBits,
        NvFlowUint minLifetime,
        NvFlowUint* pOutNumLocations,
        NvFlowUint* pOutMaxLocations
    );
    void updateLocations(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowUint64 updateId,
        NvFlowInt4* locations,
        NvFlowUint numLocations,
        NvFlowUint3 baseBlockDimBits,
        NvFlowUint minLifetime
    );

    void initFormatChannelCount(Sparse* ptr)
    {
        ptr->formatChannelCount[eNvFlowFormat_r32g32b32a32_float] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32b32a32_uint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32b32a32_sint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32b32_float] = 3u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32b32_uint] = 3u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32b32_sint] = 3u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16b16a16_float] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16b16a16_unorm] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16b16a16_uint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16b16a16_snorm] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16b16a16_sint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32_float] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32_uint] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r32g32_sint] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r10g10b10a2_unorm] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r10g10b10a2_uint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r11g11b10_float] = 3u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8b8a8_unorm] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8b8a8_unorm_srgb] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8b8a8_uint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8b8a8_snorm] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8b8a8_sint] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16_float] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16_unorm] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16_uint] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16_snorm] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r16g16_sint] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r32_float] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r32_uint] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r32_sint] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8_unorm] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8_uint] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8_snorm] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r8g8_sint] = 2u;
        ptr->formatChannelCount[eNvFlowFormat_r16_float] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r16_unorm] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r16_uint] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r16_snorm] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r16_sint] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r8_unorm] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r8_uint] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r8_snorm] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_r8_sint] = 1u;
        ptr->formatChannelCount[eNvFlowFormat_b8g8r8a8_unorm] = 4u;
        ptr->formatChannelCount[eNvFlowFormat_b8g8r8a8_unorm_srgb] = 4u;
    }

    NvFlowUint getFormatChannelCount(Sparse* ptr, NvFlowFormat format)
    {
        if (format < eNvFlowFormat_count)
        {
            return ptr->formatChannelCount[format];
        }
        return 0u;
    }

    NvFlowSparse* create(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowUint maxLocations)
    {
        auto ptr = new Sparse();

        initFormatChannelCount(ptr);

        NvFlowContextInterface_duplicate(&ptr->contextInterface, contextInterface);

        NvFlowContextConfig contextConfig = {};
        ptr->contextInterface.getContextConfig(context, &contextConfig);
        ptr->isVulkan = contextConfig.api == eNvFlowContextApi_vulkan;

        NvFlowDynamicBuffer_init(
            &ptr->contextInterface,
            context,
            &ptr->sparseTableBuffer,
            eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer | eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_bufferCopyDst,
            eNvFlowFormat_unknown,
            sizeof(NvFlowUint)
        );
        NvFlowDynamicBuffer_init(
            &ptr->contextInterface,
            context,
            &ptr->sparseBuffer[0],
            eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer | eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_bufferCopyDst,
            eNvFlowFormat_unknown,
            sizeof(NvFlowUint)
        );
        NvFlowDynamicBuffer_init(
            &ptr->contextInterface,
            context,
            &ptr->sparseBuffer[1],
            eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer | eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_bufferCopyDst,
            eNvFlowFormat_unknown,
            sizeof(NvFlowUint)
        );

        ptr->maxLocations = maxLocations;

        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->uploadBuffer, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->rescaleBuffer, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(SparseRescaleLayerParams));

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(context, &samplerDesc);

        SparseCS_init(&ptr->contextInterface, context, &ptr->sparseCS);
        SparseUploadCS_init(&ptr->contextInterface, context, &ptr->sparseUploadCS);
        SparseClearNew1CS_init(&ptr->contextInterface, context, &ptr->sparseClearNew1CS);
        SparseClearNew2CS_init(&ptr->contextInterface, context, &ptr->sparseClearNew2CS);
        SparseClearNew4CS_init(&ptr->contextInterface, context, &ptr->sparseClearNew4CS);
        SparseClearTexture1CS_init(&ptr->contextInterface, context, &ptr->sparseClearTexture1CS);
        SparseClearTexture2CS_init(&ptr->contextInterface, context, &ptr->sparseClearTexture2CS);
        SparseClearTexture4CS_init(&ptr->contextInterface, context, &ptr->sparseClearTexture4CS);

        SparseRescale1CS_init(&ptr->contextInterface, context, &ptr->sparseRescale1CS);
        SparseRescale2CS_init(&ptr->contextInterface, context, &ptr->sparseRescale2CS);
        SparseRescale4CS_init(&ptr->contextInterface, context, &ptr->sparseRescale4CS);

        SparseInplaceRescale4aCS_init(&ptr->contextInterface, context, &ptr->sparseInplaceRescale4aCS);
        SparseInplaceRescale4bCS_init(&ptr->contextInterface, context, &ptr->sparseInplaceRescale4bCS);
        SparseInplaceRescale4cCS_init(&ptr->contextInterface, context, &ptr->sparseInplaceRescale4cCS);
        SparseInplaceRescale4dCS_init(&ptr->contextInterface, context, &ptr->sparseInplaceRescale4dCS);

        SparseClearLayers1CS_init(&ptr->contextInterface, context, &ptr->sparseClearLayers1CS);
        SparseClearLayers2CS_init(&ptr->contextInterface, context, &ptr->sparseClearLayers2CS);
        SparseClearLayers4CS_init(&ptr->contextInterface, context, &ptr->sparseClearLayers4CS);

        SparseNanoVdbPruneLeavesCS_init(&ptr->contextInterface, context, &ptr->sparseNanoVdbPruneLeaves);

        SparseNanoVdbComputeStats0CS_init(&ptr->contextInterface, context, &ptr->sparseNanoVdbComputeStats0);
        SparseNanoVdbComputeStats1CS_init(&ptr->contextInterface, context, &ptr->sparseNanoVdbComputeStats1);
        SparseNanoVdbComputeStats2CS_init(&ptr->contextInterface, context, &ptr->sparseNanoVdbComputeStats2);
        SparseNanoVdbComputeStats3CS_init(&ptr->contextInterface, context, &ptr->sparseNanoVdbComputeStats3);

        // initialize empty table
        updateLayers(cast(ptr), 0llu, nullptr, 0u);
        updateLocations(context, cast(ptr), 0llu, nullptr, 0u, NvFlowUint3{1, 1, 1}, 0u);

        return cast(ptr);
    }

    void destroy(NvFlowContext* context, NvFlowSparse* sparse)
    {
        auto ptr = cast(sparse);

        NvFlowUploadBuffer_destroy(context, &ptr->uploadBuffer);
        NvFlowUploadBuffer_destroy(context, &ptr->constantBuffer);
        NvFlowUploadBuffer_destroy(context, &ptr->rescaleBuffer);

        ptr->contextInterface.destroySampler(context, ptr->samplerLinear);

        SparseCS_destroy(context, &ptr->sparseCS);
        SparseUploadCS_destroy(context, &ptr->sparseUploadCS);
        SparseClearNew1CS_destroy(context, &ptr->sparseClearNew1CS);
        SparseClearNew2CS_destroy(context, &ptr->sparseClearNew2CS);
        SparseClearNew4CS_destroy(context, &ptr->sparseClearNew4CS);
        SparseClearTexture1CS_destroy(context, &ptr->sparseClearTexture1CS);
        SparseClearTexture2CS_destroy(context, &ptr->sparseClearTexture2CS);
        SparseClearTexture4CS_destroy(context, &ptr->sparseClearTexture4CS);

        SparseRescale1CS_destroy(context, &ptr->sparseRescale1CS);
        SparseRescale2CS_destroy(context, &ptr->sparseRescale2CS);
        SparseRescale4CS_destroy(context, &ptr->sparseRescale4CS);

        SparseInplaceRescale4aCS_destroy(context, &ptr->sparseInplaceRescale4aCS);
        SparseInplaceRescale4bCS_destroy(context, &ptr->sparseInplaceRescale4bCS);
        SparseInplaceRescale4cCS_destroy(context, &ptr->sparseInplaceRescale4cCS);
        SparseInplaceRescale4dCS_destroy(context, &ptr->sparseInplaceRescale4dCS);

        SparseClearLayers1CS_destroy(context, &ptr->sparseClearLayers1CS);
        SparseClearLayers2CS_destroy(context, &ptr->sparseClearLayers2CS);
        SparseClearLayers4CS_destroy(context, &ptr->sparseClearLayers4CS);

        SparseNanoVdbPruneLeavesCS_destroy(context, &ptr->sparseNanoVdbPruneLeaves);

        SparseNanoVdbComputeStats0CS_destroy(context, &ptr->sparseNanoVdbComputeStats0);
        SparseNanoVdbComputeStats1CS_destroy(context, &ptr->sparseNanoVdbComputeStats1);
        SparseNanoVdbComputeStats2CS_destroy(context, &ptr->sparseNanoVdbComputeStats2);
        SparseNanoVdbComputeStats3CS_destroy(context, &ptr->sparseNanoVdbComputeStats3);

        NvFlowDynamicBuffer_destroy(context, &ptr->sparseTableBuffer);
        NvFlowDynamicBuffer_destroy(context, &ptr->sparseBuffer[0]);
        NvFlowDynamicBuffer_destroy(context, &ptr->sparseBuffer[1]);

        for (NvFlowUint64 nanoVdbIdx = 0u; nanoVdbIdx < ptr->nanoVdbs.size; nanoVdbIdx++)
        {
            NvFlowUploadBuffer_destroy(context, &ptr->nanoVdbs[nanoVdbIdx]->uploadBufferNanoVdb);
            NvFlowUploadBuffer_destroy(context, &ptr->nanoVdbs[nanoVdbIdx]->uploadBufferCache);
        }
        ptr->nanoVdbs.deletePointers();

        delete ptr;
    }

    void reset(NvFlowContext* context, NvFlowSparse* sparse, NvFlowUint maxLocations)
    {
        auto ptr = cast(sparse);

        ptr->targetMaxLocations = maxLocations;
        ptr->resetPending = NV_FLOW_TRUE;
    }

    void updateLayers(NvFlowSparse* sparse, NvFlowUint64 updateId, NvFlowSparseUpdateLayerParams* layers, NvFlowUint numLayers)
    {
        auto ptr = cast(sparse);

        // update old layers
        if (ptr->updateLayerId != updateId)
        {
            ptr->updateLayerId = updateId;

            ptr->layerParamsOld.size = 0u;
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
            {
                ptr->layerParamsOld.pushBack(ptr->layerParams[layerParamIdx]);
            }
            // update old level params
            ptr->levelParamsOld.size = 0u;
            for (NvFlowUint levelIdx = 0u; levelIdx < ptr->levelParams.size; levelIdx++)
            {
                ptr->levelParamsOld.pushBack(ptr->levelParams[levelIdx]);
            }
        }

        // make visible early in update
        if (ptr->resetPending)
        {
            ptr->maxLocations = ptr->targetMaxLocations;
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
            {
                ptr->layerParams[layerParamIdx].gridReset = NV_FLOW_TRUE;
            }
        }

        // generate layer params
        ptr->layerParams.size = 0u;
        ptr->layerParams.reserve(numLayers);
        ptr->layerParams.size = numLayers;
        ptr->simLayerParams.size = 0u;
        ptr->simLayerParams.reserve(numLayers);
        ptr->simLayerParams.size = numLayers;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            NvFlowSparseLayerParams layerParams = {};
            layerParams.blockSizeWorld = layers[layerParamIdx].blockSizeWorld;
            layerParams.blockSizeWorld3 = layers[layerParamIdx].blockSizeWorld.x * layers[layerParamIdx].blockSizeWorld.y * layers[layerParamIdx].blockSizeWorld.z;
            layerParams.blockSizeWorldInv = NvFlowFloat3{
                1.f / layers[layerParamIdx].blockSizeWorld.x,
                1.f / layers[layerParamIdx].blockSizeWorld.y,
                1.f / layers[layerParamIdx].blockSizeWorld.z
            };
            layerParams.layerAndLevel = NvFlow_packLayerAndLevel(layers[layerParamIdx].layer, layers[layerParamIdx].level);
            layerParams.locationMin = NvFlowInt4{ 0, 0, 0, layers[layerParamIdx].layer };
            layerParams.locationMax = NvFlowInt4{ 0, 0, 0, layers[layerParamIdx].layer };
            layerParams.worldMin = NvFlowFloat3{ 0.f, 0.f, 0.f };
            layerParams.forceDisableEmitters = layers[layerParamIdx].forceClear || layers[layerParamIdx].forceDisableEmitters;
            layerParams.worldMax = NvFlowFloat3{ 0.f, 0.f, 0.f };
            layerParams.forceClear = layers[layerParamIdx].forceClear;
            layerParams.numLocations = 0u;
            layerParams.deltaTime = 0.f;
            layerParams.forceDisableCoreSimulation = layers[layerParamIdx].forceDisableCoreSimulation;
            layerParams.gridReset = ptr->resetPending;
            ptr->layerParams[layerParamIdx] = layerParams;

            NvFlowSparseSimLayerParams simLayerParams = {};
            simLayerParams.clearOnRescale = layers[layerParamIdx].clearOnRescale;
            simLayerParams.densityCellSizeNonAuto = layers[layerParamIdx].densityCellSizeNonAuto;
            ptr->simLayerParams[layerParamIdx] = simLayerParams;
        }

        // invalidate level params
        ptr->levelParams.size = 0u;

        // make max locations accessible during allocate phase
        NvFlowSparseLevelParams baseLevelParams = {};
        baseLevelParams.maxLocations = ptr->maxLocations;
        ptr->levelParams.pushBack(baseLevelParams);
    }

    void updateLocationsInternal(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowUint64 updateId,
        NvFlowInt4* locationsIn,
        NvFlowUint numLocationsIn,
        NvFlowUint3 baseBlockDimBits,
        NvFlowUint minLifetime,
        NvFlowBool32 testMode,
        NvFlowUint* pOutNumLocations,
        NvFlowUint* pOutMaxLocations
    )
    {
        auto ptr = cast(sparse);

        NV_FLOW_PROFILE_BEGIN(31, 0)
        NV_FLOW_PROFILE_TIMESTAMP("SparseBegin")

        ptr->baseBlockDimBits = baseBlockDimBits;
        NvFlowUint3 blockDimBits = baseBlockDimBits;
        ptr->numBlockLevels = 0u;
        while (blockDimBits.x && blockDimBits.y && blockDimBits.z)
        {
            ptr->numBlockLevels++;

            blockDimBits.x--;
            blockDimBits.y--;
            blockDimBits.z--;
        }

        // detect layer rescale events
        NvFlowBool32 anyRescaleEvent = NV_FLOW_FALSE;
        ptr->layerScales.size = 0u;
        ptr->rescaleLocations.size = 0u;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            const auto layerParams = &ptr->layerParams[layerParamIdx];
            NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
            NvFlowFloat3 blockSizeWorldOld = layerParams->blockSizeWorld;
            for (NvFlowUint64 layerParamIdxOld = 0u; layerParamIdxOld < ptr->layerParamsOld.size; layerParamIdxOld++)
            {
                if (ptr->layerParamsOld[layerParamIdxOld].layerAndLevel == layerParams->layerAndLevel)
                {
                    blockSizeWorldOld = ptr->layerParamsOld[layerParamIdxOld].blockSizeWorld;
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
        // if rescale event, release all current blocks, reprojecting as new input
        bool didRescale = anyRescaleEvent && minLifetime > 0u;
        if (didRescale)
        {
            for (NvFlowUint64 blockIdx = 0u; blockIdx < ptr->hashTable.locations.size; blockIdx++)
            {
                NvFlowInt4 location = ptr->hashTable.locations[blockIdx];
                NvFlowFloat3 layerScale = { 1.f, 1.f, 1.f };
                NvFlowBool32 forceClear = NV_FLOW_TRUE;
                NvFlowBool32 clearOnRescale = NV_FLOW_TRUE;
                for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
                {
                    if (ptr->layerParams[layerParamIdx].layerAndLevel == location.w)
                    {
                        layerScale = ptr->layerScales[layerParamIdx];
                        forceClear = ptr->layerParams[layerParamIdx].forceClear;
                        clearOnRescale = ptr->simLayerParams[layerParamIdx].clearOnRescale;
                        break;
                    }
                }
                if (!forceClear && !clearOnRescale)
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
                                ptr->rescaleLocations.pushBack(locationTemp);
                            }
                        }
                    }
                }
                // rescale clears old block lifetimes (now deferred to avoid side effects)
                //ptr->hashTable.masks[blockIdx] &= 1u;
            }
            // append locationsIn to rescaleLocations, and override
            for (NvFlowUint64 idx = 0u; idx < numLocationsIn; idx++)
            {
                ptr->rescaleLocations.pushBack(locationsIn[idx]);
            }
            // override locationsIn
            locationsIn = ptr->rescaleLocations.data;
            numLocationsIn = (NvFlowUint)ptr->rescaleLocations.size;
        }

        NV_FLOW_PROFILE_TIMESTAMP("Rescale")

        // compute pool dim, check for changes
        bool allocationDirty = false;
        NvFlowUint3 poolDimTemp = NvFlowUint3{ 1u, 1u, 1u };
        {
            NvFlowUint3 maxBlockDim = {
                (1u << baseBlockDimBits.x) + 2u,
                (1u << baseBlockDimBits.y) + 2u,
                (1u << baseBlockDimBits.z) + 2u
            };
            NvFlowUint3 maxTextureDim = {
                2048u,
                2048u,
                2048u
            };
            if (ptr->isVulkan)
            {
                maxTextureDim.x = 4096u;
            };
            NvFlowUint3 maxPoolDim = {
                maxTextureDim.x / maxBlockDim.x,
                maxTextureDim.y / maxBlockDim.y,
                maxTextureDim.z / maxBlockDim.z
            };

            NvFlowUint count = 0u;
            while (poolDimTemp.x * poolDimTemp.y * poolDimTemp.z < ptr->maxLocations)
            {
                bool canX = poolDimTemp.x + 1u <= maxPoolDim.x;
                bool canY = poolDimTemp.y + 1u <= maxPoolDim.y;
                bool canZ = poolDimTemp.z + 1u <= maxPoolDim.z;

                if (canX &&
                    (poolDimTemp.x <= poolDimTemp.y || !canY) &&
                    (poolDimTemp.x <= poolDimTemp.z || !canZ))
                {
                    poolDimTemp.x += 1u;
                }
                else if (canY && (poolDimTemp.y <= poolDimTemp.z || !canZ))
                {
                    poolDimTemp.y += 1u;
                }
                else if (canZ)
                {
                    poolDimTemp.z += 1u;
                }
                else
                {
                    break;
                }
                count++;
            }

            // force reset allocations if dimensions changed
            allocationDirty = ptr->resetPending ||
                poolDimTemp.x != ptr->maxPoolDim.x ||
                poolDimTemp.y != ptr->maxPoolDim.y ||
                poolDimTemp.z != ptr->maxPoolDim.z;
        }

        NV_FLOW_PROFILE_TIMESTAMP("AllocationDirty")

        // compute diff
        ptr->diffMasks.reserve(ptr->hashTable.masks.size);
        ptr->diffMasks.size = ptr->hashTable.masks.size;
        for (NvFlowUint64 idx = 0u; idx < ptr->diffMasks.size; idx++)
        {
            ptr->diffMasks[idx] = 0u;
        }
        ptr->diffNewLocations.reset();

        static const NvFlowUint taskLocationCount = 8192u;

        NvFlowUint diffTaskCount = (numLocationsIn + taskLocationCount - 1u) / taskLocationCount;
        ptr->sparseDiffTasks.reserve(diffTaskCount);
        ptr->sparseDiffTasks.size = diffTaskCount;
        for (NvFlowUint64 taskIdx = 0u; taskIdx < ptr->sparseDiffTasks.size; taskIdx++)
        {
            auto& taskParams = ptr->sparseDiffTasks[taskIdx];
            taskParams.diffNewLocations.reset();
            taskParams.beginIdx = (NvFlowUint)(taskIdx * taskLocationCount);
            taskParams.endIdx = (NvFlowUint)((taskIdx + 1u)* taskLocationCount);
            if (taskParams.endIdx > numLocationsIn)
            {
                taskParams.endIdx = numLocationsIn;
            }
            taskParams.locations = locationsIn;
        }

        auto diffTask = [](NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
        {
            auto ptr = (Sparse*)userdata;

            auto& taskParams = ptr->sparseDiffTasks[taskIdx];

            // compute diff
            for (NvFlowUint idx = taskParams.beginIdx; idx < taskParams.endIdx; idx++)
            {
                NvFlowUint64 findIdx = ptr->hashTable.find(taskParams.locations[idx]);
                if (findIdx == ~0llu)
                {
                    taskParams.diffNewLocations.push(taskParams.locations[idx], 1u);
                }
                else
                {
                    ptr->diffMasks[findIdx] = 1u;
                }
            }
        };
        ptr->contextInterface.executeTasks(context, diffTaskCount, 1u, diffTask, ptr);

        for (NvFlowUint64 taskIdx = 0u; taskIdx < ptr->sparseDiffTasks.size; taskIdx++)
        {
            auto& taskParams = ptr->sparseDiffTasks[taskIdx];
            for (NvFlowUint64 idx = 0u; idx < taskParams.diffNewLocations.locations.size; idx++)
            {
                ptr->diffNewLocations.push(taskParams.diffNewLocations.locations[idx], 1u);
            }
        }

        bool anyNotRequested = false;
        for (NvFlowUint64 idx = 0u; idx < ptr->diffMasks.size; idx++)
        {
            if (ptr->diffMasks[idx] == 0u)
            {
                anyNotRequested = true;
                break;
            }
        }
        NV_FLOW_PROFILE_TIMESTAMP("ComputeDiff")

        NvFlowUint64 freeExistingLocationCount = 0llu;
        if (anyNotRequested)
        {
            for (NvFlowUint64 idx = 0u; idx < ptr->diffMasks.size; idx++)
            {
                if (ptr->diffMasks[idx] == 0u)
                {
                    NvFlowUint maskValue = ptr->hashTable.masks[idx];
                    maskValue &= ~1u;
                    if (didRescale) // rescale clears old block lifetimes
                    {
                        maskValue &= 1u;
                    }
                    if (maskValue > (minLifetime << 1u))
                    {
                        maskValue = (minLifetime << 1u);
                    }
                    if (maskValue > 0u)
                    {
                        maskValue -= (1u << 1u);
                        maskValue |= 1u;
                    }
                    if (maskValue == 0u)
                    {
                        freeExistingLocationCount++;
                    }
                }
            }
        }

        // exit early if requested for block overload
        NvFlowUint64 newLocationCount = ptr->diffNewLocations.locations.size + ptr->hashTable.locations.size - freeExistingLocationCount;
        if (testMode)
        {
            if (pOutNumLocations)
            {
                *pOutNumLocations = (NvFlowUint)newLocationCount;
                *pOutMaxLocations = ptr->maxLocations;
            }
            return;
        }

        // for operations that should only happen once per frame, especially history
        if (ptr->updateLocationsId != updateId)
        {
            ptr->updateLocationsId = updateId;
        }

        // update as needed
        bool isDirty = ptr->diffFirstRun || ptr->resetPending || allocationDirty || ptr->diffNewLocations.locations.size > 0u || ptr->oldDiffNewLocations > 0u || anyNotRequested;
        if (isDirty)
        {
            if (allocationDirty)
            {
                ptr->maxPoolDim = poolDimTemp;
                ptr->maxPoolDim_xy = ptr->maxPoolDim.x * ptr->maxPoolDim.y;
                ptr->maxPoolDim3 = ptr->maxPoolDim.x * ptr->maxPoolDim.y * ptr->maxPoolDim.z;

                ptr->allocationLocations.size = 0u;
                ptr->allocationLocations.reserve(ptr->maxPoolDim3);

                ptr->allocationActives.size = 0u;
                ptr->allocationActives.reserve(ptr->maxPoolDim3);
            }

            ptr->tableVersion++;

            ptr->diffFirstRun = NV_FLOW_FALSE;
            ptr->oldDiffNewLocations = ptr->diffNewLocations.locations.size;

            // clear masks for existing allocations, add new requests
            for (NvFlowUint64 idx = 0u; idx < ptr->hashTable.masks.size; idx++)
            {
                NvFlowUint maskValue = ptr->hashTable.masks[idx];
                // clear old requests
                maskValue &= ~1u;
                if (didRescale) // rescale clears old block lifetimes
                {
                    maskValue &= 1u;
                }
                // incorporate new
                maskValue |= ptr->diffMasks[idx];
                ptr->hashTable.masks[idx] = maskValue;
            }
            for (NvFlowUint idx = 0u; idx < ptr->diffNewLocations.locations.size; idx++)
            {
                ptr->hashTable.push(ptr->diffNewLocations.locations[idx], 1u);
            }
            // decrement mask count on inactive
            for (NvFlowUint64 idx = 0u; idx < ptr->hashTable.masks.size; idx++)
            {
                NvFlowUint maskValue = ptr->hashTable.masks[idx];
                if ((maskValue & 1u) == 0u)
                {
                    if (maskValue > (minLifetime << 1u))
                    {
                        maskValue = (minLifetime << 1u);
                    }
                    if (maskValue > 0u)
                    {
                        maskValue -= (1u << 1u);
                        maskValue |= 1u;
                    }
                    ptr->hashTable.masks[idx] = maskValue;
                }
                else // if requested, reset lifetime count
                {
                    ptr->hashTable.masks[idx] = (minLifetime << 1u) | 1u;
                }
            }
            // remove values with no allocation request, enforce max blocks
            ptr->hashTable.compactNonZeroWithLimit(ptr->maxLocations);
            ptr->hashTable.sort();
            ptr->hashTable.computeStats();
            // allocate
            {
                // clear arrays
                ptr->allocations.size = 0u;
                ptr->allocations.reserve(ptr->hashTable.locations.size);
                ptr->allocations.size = ptr->hashTable.locations.size;
                for (NvFlowUint idx = 0u; idx < ptr->allocations.size; idx++)
                {
                    ptr->allocations[idx] = 0x40000000;
                }

                // reset freelist
                ptr->freeList.size = 0u;
                ptr->freeList.reserve(ptr->hashTable.tableDim3);

                // release active blocks not in the new table
                for (NvFlowUint allocationIdx = 0u; allocationIdx < ptr->allocationActives.size; allocationIdx++)
                {
                    if (ptr->allocationActives[allocationIdx])
                    {
                        NvFlowUint64 blockIdx = ptr->hashTable.find(ptr->allocationLocations[allocationIdx]);
                        if (blockIdx < ptr->hashTable.locations.size)
                        {
                            // keep allocation
                            ptr->allocations[blockIdx] = computeAllocationValue(ptr, allocationIdx);
                        }
                        else
                        {
                            // free allocation
                            ptr->allocationActives[allocationIdx] = 0u;
                        }
                    }
                    if (!ptr->allocationActives[allocationIdx])
                    {
                        ptr->freeList.pushBack(allocationIdx);
                    }
                }

                // clear new list
                ptr->newList.size = 0u;

                // allocate new blocks as needed
                NvFlowUint freeListIdx = 0u;
                for (NvFlowUint blockIdx = 0u; blockIdx < ptr->hashTable.locations.size; blockIdx++)
                {
                    if (ptr->allocations[blockIdx] & 0x40000000)
                    {
                        NvFlowUint allocationIdx = 0x40000000;
                        if (freeListIdx < ptr->freeList.size)
                        {
                            allocationIdx = ptr->freeList[freeListIdx];
                            freeListIdx++;
                        }
                        else
                        {
                            allocationIdx = (NvFlowUint)ptr->allocationActives.allocateBack();
                            ptr->allocationLocations.allocateBack();
                        }

                        ptr->allocationActives[allocationIdx] = 1u;
                        ptr->allocationLocations[allocationIdx] = ptr->hashTable.locations[blockIdx];

                        // compute allocation value
                        ptr->allocations[blockIdx] = computeAllocationValue(ptr, allocationIdx);

                        ptr->newList.pushBack(blockIdx);
                        ptr->hashTable.masks[blockIdx] = (minLifetime << 1u) | 1u;
                    }
                }
            }
        }

        NV_FLOW_PROFILE_TIMESTAMP("DirtyUpdate")

        ptr->tableRanges.size = 0u;
        ptr->tableRanges.reserve(ptr->hashTable.ranges.size);
        ptr->tableRanges.size = ptr->hashTable.ranges.size;
        for (NvFlowUint64 idx = 0u; idx < ptr->tableRanges.size; idx++)
        {
            ptr->tableRanges[idx].x = (NvFlowUint)ptr->hashTable.ranges[idx].beginIdx;
            ptr->tableRanges[idx].y = (NvFlowUint)ptr->hashTable.ranges[idx].endIdx;
        }

        NvFlowUint tableDim3 = 1u << (ptr->hashTable.tableDimBits + ptr->hashTable.tableDimBits + ptr->hashTable.tableDimBits);
        NvFlowUint numLocations = (NvFlowUint)ptr->hashTable.locations.size;

        // generate level params
        ptr->levelParams.size = 0u;
        ptr->levelParams.reserve(ptr->numBlockLevels);
        ptr->levelParams.size = ptr->numBlockLevels;

        NvFlowUint numWords_range = 2u * tableDim3;
        NvFlowUint numWords_location = 4u * numLocations;
        NvFlowUint numWords_layerParamIdx = numLocations;
        NvFlowUint numWords_allocation = numLocations;
        NvFlowUint numWords_newList = numLocations;
        NvFlowUint numWords_blockLevel_global = numLocations;
        NvFlowUint numWords_blockLevel_local = 32u * numLocations;

        NvFlowUint totalWords = 0u;
        totalWords += numWords_range;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        NvFlowUint locationOffset = totalWords;
        totalWords += numWords_location;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        NvFlowUint layerParamIdxOffset = totalWords;
        totalWords += numWords_layerParamIdx;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        NvFlowUint allocationOffset = totalWords;
        totalWords += numWords_allocation;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        NvFlowUint newListOffset = totalWords;
        totalWords += numWords_newList;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        NvFlowUint uploadWords = totalWords;

        NvFlowUint blockLevelOffsetGlobal_base = totalWords;
        totalWords += ptr->numBlockLevels * numWords_blockLevel_global;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        NvFlowUint blockLevelOffsetLocal_base = totalWords;
        totalWords += ptr->numBlockLevels * numWords_blockLevel_local;
        totalWords = 32u * ((totalWords + 31u) / 32u);

        // compute block level parameters
        blockDimBits = ptr->baseBlockDimBits;
        for (NvFlowUint levelIdx = 0u; levelIdx < ptr->numBlockLevels; levelIdx++)
        {
            NvFlowSparseLevelParams levelParams = {};

            levelParams.blockDimLessOne.x = (1u << blockDimBits.x) - 1u;
            levelParams.blockDimLessOne.y = (1u << blockDimBits.y) - 1u;
            levelParams.blockDimLessOne.z = (1u << blockDimBits.z) - 1u;
            levelParams.threadsPerBlock = 1u << (blockDimBits.x + blockDimBits.y + blockDimBits.z);

            levelParams.blockDimBits = blockDimBits;
            levelParams.numLocations = numLocations;

            levelParams.tableDimLessOne.x = ptr->hashTable.tableDimLessOne;
            levelParams.tableDimLessOne.y = ptr->hashTable.tableDimLessOne;
            levelParams.tableDimLessOne.z = ptr->hashTable.tableDimLessOne;
            levelParams.tableDim3 = tableDim3;

            levelParams.tableDimBits_x = ptr->hashTable.tableDimBits;
            levelParams.tableDimBits_xy = ptr->hashTable.tableDimBits + ptr->hashTable.tableDimBits;
            levelParams.tableDimBits_z = ptr->hashTable.tableDimBits;
            levelParams.locationOffset = locationOffset;

            levelParams.allocationOffset = allocationOffset;
            levelParams.newListOffset = newListOffset;
            levelParams.blockLevelOffsetGlobal = blockLevelOffsetGlobal_base + levelIdx * numWords_blockLevel_global;
            levelParams.blockLevelOffsetLocal = blockLevelOffsetLocal_base + levelIdx * numWords_blockLevel_local;

            levelParams.layerParamIdxOffset = layerParamIdxOffset;
            levelParams.numLayers = (NvFlowUint)ptr->layerParams.size;
            levelParams.pad0 = 0u;
            levelParams.pad1 = 0u;

            levelParams.dim.x = ptr->maxPoolDim.x * ((1u << levelParams.blockDimBits.x) + 2u);
            levelParams.dim.y = ptr->maxPoolDim.y * ((1u << levelParams.blockDimBits.y) + 2u);
            levelParams.dim.z = ptr->maxPoolDim.z * ((1u << levelParams.blockDimBits.z) + 2u);
            levelParams.maxLocations = ptr->maxLocations;

            levelParams.dimInv.x = 1.f / float(levelParams.dim.x);
            levelParams.dimInv.y = 1.f / float(levelParams.dim.y);
            levelParams.dimInv.z = 1.f / float(levelParams.dim.z);
            levelParams.numNewLocations = (NvFlowUint)ptr->newList.size;

            levelParams.globalLocationMin = ptr->hashTable.locationMin;
            levelParams.globalLocationMax = ptr->hashTable.locationMax;

            ptr->levelParams[levelIdx] = levelParams;

            blockDimBits.x--;
            blockDimBits.y--;
            blockDimBits.z--;
        }

        // clear per layer location count
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            ptr->layerParams[layerParamIdx].numLocations = 0u;
        }

        // generate mapping from blockIdx to layerParamIdx
        ptr->layerParamIdxs.size = 0u;
        ptr->layerParamIdxs.reserve(ptr->hashTable.locations.size);
        ptr->layerParamIdxs.size = ptr->hashTable.locations.size;
        for (NvFlowUint idx = 0u; idx < ptr->hashTable.locations.size; idx++)
        {
            int layerAndLevel = ptr->hashTable.locations[idx].w;
            NvFlowUint layerParamIdxMatch = 0u;
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
            {
                if (ptr->layerParams[layerParamIdx].layerAndLevel == layerAndLevel)
                {
                    ptr->layerParams[layerParamIdx].numLocations++;
                    layerParamIdxMatch = layerParamIdx;
                    break;
                }
            }
            ptr->layerParamIdxs[idx] = layerParamIdxMatch;
        }

        // generate per layer bounds
        generateLayerLocationRanges(ptr, (NvFlowUint)ptr->hashTable.locations.size, ptr->hashTable.locations.data);

        NV_FLOW_PROFILE_TIMESTAMP("BuildParams")

        // compute buffer sizes
        ptr->totalSizeInBytes = totalWords * sizeof(NvFlowUint);
        ptr->uploadSizeInBytes = uploadWords * sizeof(NvFlowUint);

        // clear
        if (ptr->resetPending)
        {
            ptr->resetPending = NV_FLOW_FALSE;
        }

        NV_FLOW_PROFILE_FLUSH("Sparse", ptr->contextInterface.getLogPrint(context))
    }

    void testUpdateLocations(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowUint64 updateId,
        NvFlowInt4* locations,
        NvFlowUint numLocations,
        NvFlowUint3 baseBlockDimBits,
        NvFlowUint minLifetime,
        NvFlowUint* pOutNumLocations,
        NvFlowUint* pOutMaxLocations
    )
    {
        updateLocationsInternal(context, sparse, updateId, locations, numLocations, baseBlockDimBits, minLifetime, NV_FLOW_TRUE, pOutNumLocations, pOutMaxLocations);
    }

    void updateLocations(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowUint64 updateId,
        NvFlowInt4* locations,
        NvFlowUint numLocations,
        NvFlowUint3 baseBlockDimBits,
        NvFlowUint minLifetime
    )
    {
        updateLocationsInternal(context, sparse, updateId, locations, numLocations, baseBlockDimBits, minLifetime, NV_FLOW_FALSE, nullptr, nullptr);
    }

    void updateLayerDeltaTimes(NvFlowSparse* sparse, float* layerDeltaTimes, NvFlowUint64 layerDeltaTimeCount)
    {
        auto ptr = cast(sparse);
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            float deltaTime = layerParamIdx < layerDeltaTimeCount ? layerDeltaTimes[layerParamIdx] : 0.f;
            ptr->layerParams[layerParamIdx].deltaTime = deltaTime;
        }
    }

    NvFlowBool32 getParams(NvFlowSparse* sparse, NvFlowSparseParams* out)
    {
        auto ptr = cast(sparse);
        if (out)
        {
            out->layerCount = (NvFlowUint)ptr->layerParams.size;
            out->layers = ptr->layerParams.data;
            out->levelCount = (NvFlowUint)ptr->levelParams.size;
            out->levels = ptr->levelParams.data;
            out->locations = ptr->hashTable.locations.data;
            out->locationCount = ptr->hashTable.locations.size;
            out->tableRanges = ptr->tableRanges.data;
            out->tableRangeCount = ptr->tableRanges.size;
        }
        return NV_FLOW_TRUE;
    }

    NvFlowBool32 getSimParams(NvFlowSparse* sparse, NvFlowSparseSimParams* out)
    {
        auto ptr = cast(sparse);
        if (out)
        {
            getParams(sparse, &out->sparseParams);
            out->layerCount = (NvFlowUint)ptr->simLayerParams.size;
            out->layers = ptr->simLayerParams.data;
        }
        return NV_FLOW_TRUE;
    }

    void addPasses(NvFlowContext* context, NvFlowSparse* sparse, NvFlowBufferTransient** pBufferTransient)
    {
        auto ptr = cast(sparse);

        if (ptr->uploadedTableVersion == ptr->tableVersion)
        {
            NvFlowBufferTransient* bufferTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->sparseBuffer[ptr->sparseBufferIdx]);

            *pBufferTransient = bufferTransient;

            return;
        }
        ptr->uploadedTableVersion = ptr->tableVersion;

        NvFlowLocationHashTable* table = &ptr->hashTable;

        NvFlowBufferTransient* uploadTransient = nullptr;
        {
            NvFlowUint* mapped = (NvFlowUint*)NvFlowUploadBuffer_map(context, &ptr->uploadBuffer, ptr->uploadSizeInBytes);

            // write ranges
            for (NvFlowUint64 idx = 0u; idx < table->ranges.size; idx++)
            {
                //mapped[2u * idx + 0u] = (NvFlowUint)table->ranges[idx].beginIdx;
                //mapped[2u * idx + 1u] = (NvFlowUint)table->ranges[idx].endIdx;
                mapped[2u * idx + 0u] = ptr->tableRanges[idx].x;
                mapped[2u * idx + 1u] = ptr->tableRanges[idx].y;
            }

            // write locations
            NvFlowUint locationOffset = ptr->levelParams[0u].locationOffset;
            for (NvFlowUint64 idx = 0u; idx < table->locations.size; idx++)
            {
                mapped[4u * idx + 0u + locationOffset] = NvFlowUint(table->locations[idx].x);
                mapped[4u * idx + 1u + locationOffset] = NvFlowUint(table->locations[idx].y);
                mapped[4u * idx + 2u + locationOffset] = NvFlowUint(table->locations[idx].z);
                mapped[4u * idx + 3u + locationOffset] = NvFlowUint(table->locations[idx].w);
            }

            // write layerParamIdxs
            NvFlowUint layerParamIdxOffset = ptr->levelParams[0u].layerParamIdxOffset;
            for (NvFlowUint64 idx = 0u; idx < ptr->layerParamIdxs.size; idx++)
            {
                mapped[idx + layerParamIdxOffset] = ptr->layerParamIdxs[idx];
            }

            // write allocations
            NvFlowUint allocationOffset = ptr->levelParams[0u].allocationOffset;
            for (NvFlowUint idx = 0u; idx < ptr->allocations.size; idx++)
            {
                mapped[idx + allocationOffset] = ptr->allocations[idx];
            }

            // write newList
            NvFlowUint newListOffset = ptr->levelParams[0u].newListOffset;
            for (NvFlowUint64 idx = 0u; idx < ptr->newList.size; idx++)
            {
                mapped[idx + newListOffset] = ptr->newList[idx];
            }

            //uploadTransient = NvFlowUploadBuffer_unmapDevice(context, &ptr->uploadBuffer, 0llu, ptr->uploadSizeInBytes, "SparseUpload");
            uploadTransient = NvFlowUploadBuffer_unmap(context, &ptr->uploadBuffer);
        }

        NvFlowUint uploadBlockCount = ((ptr->uploadSizeInBytes / sizeof(NvFlowUint)) + 127u) / 128u;
        NvFlowDispatchBatches uploadBatches;
        NvFlowDispatchBatches_init(&uploadBatches, uploadBlockCount);
        NvFlowDispatchBatches sparseBatches;
        NvFlowDispatchBatches_init(&sparseBatches, (NvFlowUint)table->locations.size);

        NvFlowUint64 totalBatches = uploadBatches.size >= sparseBatches.size ? uploadBatches.size : sparseBatches.size;

        for (NvFlowUint64 batchIdx = 0u; batchIdx < totalBatches; batchIdx++)
        {
            SparseShaderParams* mapped = (SparseShaderParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(SparseShaderParams));

            mapped->uploadBlockIdxOffset = batchIdx < uploadBatches.size ? uploadBatches[batchIdx].blockIdxOffset : 0u;
            mapped->sparseBlockIdxOffset = batchIdx < sparseBatches.size ? sparseBatches[batchIdx].blockIdxOffset : 0u;
            mapped->uploadSizeInBytes = ptr->uploadSizeInBytes;
            mapped->pad3 = 0u;

            mapped->tableParams = ptr->levelParams[0u];
            NvFlowUint numLevels = ptr->numBlockLevels;
            if (numLevels > 16u)
            {
                numLevels = 16u;
            }
            for (NvFlowUint levelIdx = 0u; levelIdx < numLevels; levelIdx++)
            {
                mapped->blockLevel[levelIdx] = ptr->levelParams[levelIdx];
            }

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);
            if (batchIdx < uploadBatches.size)
            {
                uploadBatches[batchIdx].globalTransient = constantTransient;
            }
            if (batchIdx < sparseBatches.size)
            {
                sparseBatches[batchIdx].globalTransient = constantTransient;
            }
        }

        // flip sparse buffer
        ptr->sparseBufferIdx ^= 1u;

        NvFlowDynamicBuffer_resize(context, &ptr->sparseTableBuffer, ptr->uploadSizeInBytes);
        NvFlowDynamicBuffer_resize(context, &ptr->sparseBuffer[ptr->sparseBufferIdx], ptr->totalSizeInBytes);

        NvFlowBufferTransient* tableBufferTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->sparseTableBuffer);
        NvFlowBufferTransient* bufferTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->sparseBuffer[ptr->sparseBufferIdx]);

        for (NvFlowUint64 uploadBatchIdx = 0u; uploadBatchIdx < uploadBatches.size; uploadBatchIdx++)
        {
            SparseUploadCS_PassParams passParams = {};
            passParams.gParams = uploadBatches[uploadBatchIdx].globalTransient;
            passParams.gTableIn = uploadTransient;
            passParams.gTable1Out = tableBufferTransient;
            passParams.gTable2Out = bufferTransient;

            NvFlowUint3 gridDim = {
                uploadBatches[uploadBatchIdx].blockCount,
                1u,
                1u
            };

            SparseUploadCS_addPassCompute(context, &ptr->sparseUploadCS, gridDim, &passParams);
        }

        for (NvFlowUint64 sparseBatchIdx = 0u; sparseBatchIdx < sparseBatches.size; sparseBatchIdx++)
        {
            SparseCS_PassParams passParams = {};
            passParams.gParams = sparseBatches[sparseBatchIdx].globalTransient;
            passParams.gTableIn = tableBufferTransient;
            passParams.gTableOut = bufferTransient;

            NvFlowUint3 gridDim = {
                ptr->numBlockLevels,
                sparseBatches[sparseBatchIdx].blockCount,
                1u
            };

            SparseCS_addPassCompute(context, &ptr->sparseCS, gridDim, &passParams);
        }

        *pBufferTransient = bufferTransient;
    }

    NvFlowUint2 uint64_to_uint2(NvFlowUint64 value)
    {
        return *((NvFlowUint2*)&value);
    }

    NvFlowUint64 uint2_to_uint64(NvFlowUint2 value)
    {
        return *((NvFlowUint64*)&value);
    }

    void addPassesNanoVdb(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        pnanovdb_grid_type_t gridTypeIn,
        NvFlowUint levelIdx,
        NvFlowSparseNanoVdbParams* pParams,
        NvFlowBufferTransient** pNanoVdbBufferTransient,
        NvFlowBufferTransient** pCacheBufferTransient
    )
    {
        auto ptr = cast(sparse);

        if (levelIdx >= ptr->levelParams.size || ptr->layerParams.size == 0u)
        {
            return;
        }

        pnanovdb_grid_type_t grid_type = gridTypeIn;

        // find instance or create
        SparseNanoVdb* nanoVdb = nullptr;
        for (NvFlowUint64 nanoVdbIdx = 0u; nanoVdbIdx < ptr->nanoVdbs.size; nanoVdbIdx++)
        {
            if (ptr->nanoVdbs[nanoVdbIdx]->levelIdx == levelIdx &&
                ptr->nanoVdbs[nanoVdbIdx]->gridType == gridTypeIn)
            {
                nanoVdb = ptr->nanoVdbs[nanoVdbIdx];
                break;
            }
        }
        if (!nanoVdb)
        {
            nanoVdb = ptr->nanoVdbs.allocateBackPointer();

            nanoVdb->levelIdx = levelIdx;
            nanoVdb->gridType = gridTypeIn;

            // initialize instance
            NvFlowUploadBuffer_init(&ptr->contextInterface, context, &nanoVdb->uploadBufferNanoVdb, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(NvFlowUint));
            NvFlowUploadBuffer_init(&ptr->contextInterface, context, &nanoVdb->uploadBufferCache, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        }

        const auto& levelParams = ptr->levelParams[levelIdx];

        nanoVdb->perLayers.reserve(ptr->layerParams.size);
        nanoVdb->perLayers.size = ptr->layerParams.size;

        NvFlowArray<NvFlowUint8>& buf_array = nanoVdb->buf_array;
        pnanovdb_buf_t& buf = nanoVdb->buf;
        buf_array.size = 0u;
        buf.data = nullptr;
        buf.size_in_words = 0u;

        NvFlowArray<NvFlowUint>& layer_param_indices = nanoVdb->layerParamsIndices;
        NvFlowArray<pnanovdb_coord_t>& coarse_coords = nanoVdb->coarseCoords;
        NvFlowArray<pnanovdb_coord_t>& fine_offsets = nanoVdb->fineOffsets;
        layer_param_indices.size = 0u;
        coarse_coords.size = 0u;
        fine_offsets.size = 0u;

        NvFlowArray<pnanovdb_root_tile_handle_t>& cached_tiles = nanoVdb->cachedTiles;
        NvFlowArray<pnanovdb_upper_handle_t>& cached_upper = nanoVdb->cachedUpper;
        NvFlowArray<pnanovdb_lower_handle_t>& cached_lower = nanoVdb->cachedLower;
        NvFlowArray<pnanovdb_leaf_handle_t>& cached_leaf = nanoVdb->cachedLeaf;
        cached_tiles.size = 0u;
        cached_upper.size = 0u;
        cached_lower.size = 0u;
        cached_leaf.size = 0u;

        NvFlowArray<pnanovdb_root_tile_handle_t>& list_tiles = nanoVdb->listTiles;
        NvFlowArray<pnanovdb_upper_handle_t>& list_upper = nanoVdb->listUpper;
        NvFlowArray<pnanovdb_lower_handle_t>& list_lower = nanoVdb->listLower;
        NvFlowArray<pnanovdb_leaf_handle_t>& list_leaf = nanoVdb->listLeaf;
        list_tiles.size = 0u;
        list_upper.size = 0u;
        list_lower.size = 0u;
        list_leaf.size = 0u;

        NvFlowUint3 subGridDimBits = {
            levelParams.blockDimBits.x > 3u ? levelParams.blockDimBits.x - 3u : 0u,
            levelParams.blockDimBits.y > 3u ? levelParams.blockDimBits.y - 3u : 0u,
            levelParams.blockDimBits.z > 3u ? levelParams.blockDimBits.z - 3u : 0u,
        };
        NvFlowUint3 subGridDimLessOne = {
            (1u << subGridDimBits.x) - 1u,
            (1u << subGridDimBits.y) - 1u,
            (1u << subGridDimBits.z) - 1u,
        };

        // cache location->ijk mapping and layer->layerParamIdx mapping
        {
            NvFlowUint cachedLayerParamIdx = 0u;
            int cachedLayerAndLevel = ptr->layerParams[cachedLayerParamIdx].layerAndLevel;
            for (NvFlowUint64 blockIdx = 0u; blockIdx < ptr->hashTable.locations.size; blockIdx++)
            {
                NvFlowInt4 location = ptr->hashTable.locations[blockIdx];
                pnanovdb_coord_t coarse_ijk = {
                    (location.x << levelParams.blockDimBits.x),
                    (location.y << levelParams.blockDimBits.y),
                    (location.z << levelParams.blockDimBits.z)
                };
                coarse_coords.pushBack(coarse_ijk);

                NvFlowUint locationLayerParamIdx = 0u;    // default to zero to avoid crash
                if (cachedLayerAndLevel == location.w)
                {
                    locationLayerParamIdx = cachedLayerParamIdx;
                }
                else
                {
                    for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
                    {
                        if (ptr->layerParams[layerParamIdx].layerAndLevel == location.w)
                        {
                            locationLayerParamIdx = (NvFlowUint)layerParamIdx;
                            cachedLayerParamIdx = (NvFlowUint)layerParamIdx;
                            cachedLayerAndLevel = ptr->layerParams[cachedLayerParamIdx].layerAndLevel;
                            break;
                        }
                    }
                }
                layer_param_indices.pushBack(locationLayerParamIdx);
            }
        }
        for (NvFlowUint k = 0u; k <= subGridDimLessOne.z; k++)
        {
            for (NvFlowUint j = 0u; j <= subGridDimLessOne.y; j++)
            {
                for (NvFlowUint i = 0u; i <= subGridDimLessOne.x; i++)
                {
                    pnanovdb_coord_t fine_ijk = {
                        int(i << 3),
                        int(j << 3),
                        int(k << 3)
                    };
                    fine_offsets.pushBack(fine_ijk);
                }
            }
        }

        // align cache sizes to block count
        cached_tiles.reserve(coarse_coords.size);
        cached_upper.reserve(coarse_coords.size);
        cached_lower.reserve(coarse_coords.size);
        cached_leaf.reserve(coarse_coords.size);
        cached_tiles.size = coarse_coords.size;
        cached_upper.size = coarse_coords.size;
        cached_lower.size = coarse_coords.size;
        cached_leaf.size = coarse_coords.size;

        // allocate grid and tree packed together
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            const auto& layerParams = ptr->layerParams[layerParamIdx];
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];

            // allocate grid
            perLayer.grid.address.byte_offset = buf_array.size;

            buf_array.reserve(buf_array.size + PNANOVDB_GRID_SIZE);
            buf_array.size += PNANOVDB_GRID_SIZE;

            // allocate tree
            perLayer.tree.address.byte_offset = buf_array.size;

            buf_array.reserve(buf_array.size + PNANOVDB_TREE_SIZE);
            buf_array.size += PNANOVDB_TREE_SIZE;

            // clear grid and tree
            memset(buf_array.data + perLayer.grid.address.byte_offset, 0, PNANOVDB_GRID_SIZE);
            memset(buf_array.data + perLayer.tree.address.byte_offset, 0, PNANOVDB_TREE_SIZE);

            // update buf
            buf.data = (pnanovdb_uint32_t*)buf_array.data;
            buf.size_in_words = buf_array.size / 4u;
        }
        // allocate null grid and tree at end of array
        {
            // allocate grid
            pnanovdb_grid_handle_t grid = { buf_array.size };

            buf_array.reserve(buf_array.size + PNANOVDB_GRID_SIZE);
            buf_array.size += PNANOVDB_GRID_SIZE;

            // allocate tree
            pnanovdb_tree_handle_t tree = { buf_array.size };

            buf_array.reserve(buf_array.size + PNANOVDB_TREE_SIZE);
            buf_array.size += PNANOVDB_TREE_SIZE;

            // clear grid and tree
            memset(buf_array.data + grid.address.byte_offset, 0, PNANOVDB_GRID_SIZE);
            memset(buf_array.data + tree.address.byte_offset, 0, PNANOVDB_TREE_SIZE);

            // update buf
            buf.data = (pnanovdb_uint32_t*)buf_array.data;
            buf.size_in_words = buf_array.size / 4u;
        }

        // root and root tiles should be contiguous segment per layer
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            const auto& layerParams = ptr->layerParams[layerParamIdx];
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];

            // grid/tree are placed as array in beginning of buffer
            pnanovdb_grid_handle_t grid = { layerParamIdx * (PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE) };
            pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);

            // node_offset_root is tree relative
            pnanovdb_uint64_t node_offset_root = buf_array.size - tree.address.byte_offset;

            pnanovdb_tree_set_node_offset_root(buf, tree, node_offset_root);
            pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

            perLayer.grid = grid;
            perLayer.tree = tree;
            perLayer.root = root;

            // allocate root
            buf_array.reserve(buf_array.size + pnanovdb_grid_type_constants[grid_type].root_size);
            buf_array.size += pnanovdb_grid_type_constants[grid_type].root_size;
            buf.data = (pnanovdb_uint32_t*)buf_array.data;
            buf.size_in_words = buf_array.size / 4u;

            // initialize root
            pnanovdb_coord_t root_bbox_min = {
                (layerParams.locationMin.x << levelParams.blockDimBits.x),
                (layerParams.locationMin.y << levelParams.blockDimBits.y),
                (layerParams.locationMin.z << levelParams.blockDimBits.z)
            };
            pnanovdb_coord_t root_bbox_max = {
                ((layerParams.locationMax.x + 1) << levelParams.blockDimBits.x) - 1,
                ((layerParams.locationMax.y + 1) << levelParams.blockDimBits.y) - 1,
                ((layerParams.locationMax.z + 1) << levelParams.blockDimBits.z) - 1
            };
            pnanovdb_uint64_t active_voxel_count = layerParams.numLocations << (
                levelParams.blockDimBits.x + levelParams.blockDimBits.y + levelParams.blockDimBits.z
            );

            pnanovdb_tree_set_voxel_count(buf, tree, active_voxel_count);

            pnanovdb_root_set_bbox_min(buf, root, PNANOVDB_REF(root_bbox_min));
            pnanovdb_root_set_bbox_max(buf, root, PNANOVDB_REF(root_bbox_max));
            pnanovdb_root_set_tile_count(buf, root, 0u);

            // clear counts
            perLayer.tile_count = 0u;
            perLayer.upper_count = 0u;
            perLayer.lower_count = 0u;
            perLayer.leaf_count = 0u;

            // add tiles
            for (NvFlowUint64 blockIdx = 0u; blockIdx < coarse_coords.size; blockIdx++)
            {
                NvFlowUint layerParamIdxComp = layer_param_indices[blockIdx];
                auto& perLayer = nanoVdb->perLayers[layerParamIdx];

                if (layerParamIdx == layerParamIdxComp)
                {
                    pnanovdb_coord_t ijk = coarse_coords[blockIdx];
                    pnanovdb_uint64_t key = pnanovdb_coord_to_key(PNANOVDB_REF(ijk));

                    pnanovdb_root_tile_handle_t tile = pnanovdb_root_get_tile_zero(grid_type, perLayer.root);
                    pnanovdb_uint32_t tile_idx = 0u;
                    for (; tile_idx < perLayer.tile_count; tile_idx++)
                    {
                        if (pnanovdb_uint64_is_equal(key, pnanovdb_root_tile_get_key(buf, tile)))
                        {
                            break;
                        }
                        tile.address = pnanovdb_address_offset(tile.address, PNANOVDB_GRID_TYPE_GET(grid_type, root_tile_size));
                    }
                    if (tile_idx == perLayer.tile_count)
                    {
                        buf_array.reserve(buf_array.size + PNANOVDB_GRID_TYPE_GET(grid_type, root_tile_size));
                        buf_array.size += PNANOVDB_GRID_TYPE_GET(grid_type, root_tile_size);
                        buf.data = (pnanovdb_uint32_t*)buf_array.data;
                        buf.size_in_words = buf_array.size / 4u;

                        pnanovdb_root_tile_set_key(buf, tile, key);
                        pnanovdb_root_tile_set_child(buf, tile, 0);
                        pnanovdb_root_tile_set_state(buf, tile, 1u);

                        list_tiles.pushBack(tile);

                        perLayer.tile_count++;
                    }
                    cached_tiles[blockIdx] = tile;
                }
            }

            pnanovdb_root_set_tile_count(buf, perLayer.root, perLayer.tile_count);
        }

        // add upper
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];

            pnanovdb_tree_set_node_offset_upper(buf, perLayer.tree, buf_array.size - perLayer.tree.address.byte_offset);

            for (NvFlowUint64 blockIdx = 0u; blockIdx < coarse_coords.size; blockIdx++)
            {
                NvFlowUint layerParamIdxComp = layer_param_indices[blockIdx];
                auto& perLayer = nanoVdb->perLayers[layerParamIdx];

                if (layerParamIdx == layerParamIdxComp)
                {
                    pnanovdb_coord_t ijk = coarse_coords[blockIdx];

                    pnanovdb_root_tile_handle_t tile = cached_tiles[blockIdx];
                    pnanovdb_upper_handle_t upper = {};
                    if (pnanovdb_int64_is_zero(pnanovdb_root_tile_get_child(buf, tile)))
                    {
                        upper.address.byte_offset = buf_array.size;

                        // byte offset from root to upper
                        pnanovdb_int64_t child = upper.address.byte_offset - perLayer.root.address.byte_offset;
                        pnanovdb_root_tile_set_child(buf, tile, child);

                        buf_array.reserve(buf_array.size + PNANOVDB_GRID_TYPE_GET(grid_type, upper_size));
                        buf_array.size += PNANOVDB_GRID_TYPE_GET(grid_type, upper_size);
                        buf.data = (pnanovdb_uint32_t*)buf_array.data;
                        buf.size_in_words = buf_array.size / 4u;

                        // initialize upper
                        memset(buf_array.data + upper.address.byte_offset, 0, PNANOVDB_GRID_TYPE_GET(grid_type, upper_size));

                        pnanovdb_coord_t upper_bbox_min = {
                            (ijk.x >> 12) << 12,
                            (ijk.y >> 12) << 12,
                            (ijk.z >> 12) << 12
                        };
                        pnanovdb_coord_t upper_bbox_max = {
                            (((ijk.x >> 12) + 1) << 12) - 1,
                            (((ijk.y >> 12) + 1) << 12) - 1,
                            (((ijk.z >> 12) + 1) << 12) - 1
                        };
                        pnanovdb_upper_set_bbox_min(buf, upper, PNANOVDB_REF(upper_bbox_min));
                        pnanovdb_upper_set_bbox_max(buf, upper, PNANOVDB_REF(upper_bbox_max));

                        list_upper.pushBack(upper);

                        perLayer.upper_count++;
                    }
                    else
                    {
                        upper = pnanovdb_root_get_child(grid_type, buf, perLayer.root, tile);
                    }
                    cached_upper[blockIdx] = upper;
                }
            }

            pnanovdb_tree_set_node_count_upper(buf, perLayer.tree, perLayer.upper_count);
        }

        // add lower
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];

            pnanovdb_tree_set_node_offset_lower(buf, perLayer.tree, buf_array.size - perLayer.tree.address.byte_offset);

            for (NvFlowUint64 blockIdx = 0u; blockIdx < coarse_coords.size; blockIdx++)
            {
                NvFlowUint layerParamIdxComp = layer_param_indices[blockIdx];
                auto& perLayer = nanoVdb->perLayers[layerParamIdx];

                if (layerParamIdx == layerParamIdxComp)
                {
                    pnanovdb_coord_t ijk = coarse_coords[blockIdx];

                    pnanovdb_root_tile_handle_t tile = cached_tiles[blockIdx];
                    pnanovdb_upper_handle_t upper = cached_upper[blockIdx];
                    pnanovdb_lower_handle_t lower = {};

                    pnanovdb_uint32_t n2 = pnanovdb_upper_coord_to_offset(PNANOVDB_REF(ijk));
                    if (pnanovdb_upper_get_child_mask(buf, upper, n2))
                    {
                        lower = pnanovdb_upper_get_child(grid_type, buf, upper, n2);
                    }
                    else
                    {
                        pnanovdb_upper_set_child_mask(buf, upper, n2, PNANOVDB_TRUE);
                        lower.address.byte_offset = buf_array.size;

                        // byte offset from upper to lower
                        pnanovdb_int64_t child = lower.address.byte_offset - upper.address.byte_offset;
                        pnanovdb_upper_set_table_child(grid_type, buf, upper, n2, child);

                        buf_array.reserve(buf_array.size + PNANOVDB_GRID_TYPE_GET(grid_type, lower_size));
                        buf_array.size += PNANOVDB_GRID_TYPE_GET(grid_type, lower_size);
                        buf.data = (pnanovdb_uint32_t*)buf_array.data;
                        buf.size_in_words = buf_array.size / 4u;

                        // initialize lower
                        memset(buf_array.data + lower.address.byte_offset, 0, PNANOVDB_GRID_TYPE_GET(grid_type, lower_size));

                        pnanovdb_coord_t lower_bbox_min = {
                            (ijk.x >> 7) << 7,
                            (ijk.y >> 7) << 7,
                            (ijk.z >> 7) << 7
                        };
                        pnanovdb_coord_t lower_bbox_max = {
                            (((ijk.x >> 7) + 1) << 7) - 1,
                            (((ijk.y >> 7) + 1) << 7) - 1,
                            (((ijk.z >> 7) + 1) << 7) - 1
                        };
                        pnanovdb_lower_set_bbox_min(buf, lower, PNANOVDB_REF(lower_bbox_min));
                        pnanovdb_lower_set_bbox_max(buf, lower, PNANOVDB_REF(lower_bbox_max));

                        list_lower.pushBack(lower);

                        perLayer.lower_count++;
                    }
                    cached_lower[blockIdx] = lower;
                }
            }

            pnanovdb_tree_set_node_count_lower(buf, perLayer.tree, perLayer.lower_count);
        }

        static const bool enable_leaves = false;

        // capture buffer_size without leaves
        nanoVdb->buffer_size_without_leaves = buf_array.size;
        nanoVdb->buffer_size_with_leaves = buf_array.size;

        // add leaf
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];

            pnanovdb_tree_set_node_offset_leaf(buf, perLayer.tree, nanoVdb->buffer_size_with_leaves - perLayer.tree.address.byte_offset);

            for (NvFlowUint64 blockIdx = 0u; blockIdx < coarse_coords.size; blockIdx++)
            {
                NvFlowUint layerParamIdxComp = layer_param_indices[blockIdx];
                auto& perLayer = nanoVdb->perLayers[layerParamIdx];

                if (layerParamIdx == layerParamIdxComp)
                {
                    pnanovdb_coord_t ijk = coarse_coords[blockIdx];

                    pnanovdb_root_tile_handle_t tile = cached_tiles[blockIdx];
                    pnanovdb_upper_handle_t upper = cached_upper[blockIdx];
                    pnanovdb_lower_handle_t lower = cached_lower[blockIdx];
                    pnanovdb_leaf_handle_t leaf = {};

                    for (NvFlowUint64 fine_idx = 0u; fine_idx < fine_offsets.size; fine_idx++)
                    {
                        pnanovdb_coord_t fine_ijk = {
                            ijk.x + fine_offsets[fine_idx].x,
                            ijk.y + fine_offsets[fine_idx].y,
                            ijk.z + fine_offsets[fine_idx].z,
                        };
                        pnanovdb_uint32_t n1 = pnanovdb_lower_coord_to_offset(PNANOVDB_REF(fine_ijk));
                        if (pnanovdb_lower_get_child_mask(buf, lower, n1))
                        {
                            leaf = pnanovdb_lower_get_child(grid_type, buf, lower, n1);
                        }
                        else
                        {
                            pnanovdb_lower_set_child_mask(buf, lower, n1, PNANOVDB_TRUE);
                            leaf.address.byte_offset = nanoVdb->buffer_size_with_leaves;

                            // byte offset from lower to leaf
                            pnanovdb_int64_t child = leaf.address.byte_offset - lower.address.byte_offset;
                            pnanovdb_lower_set_table_child(grid_type, buf, lower, n1, child);

                            nanoVdb->buffer_size_with_leaves += PNANOVDB_GRID_TYPE_GET(grid_type, leaf_size);
                            if (enable_leaves)
                            {
                                buf_array.reserve(buf_array.size + PNANOVDB_GRID_TYPE_GET(grid_type, leaf_size));
                                buf_array.size += PNANOVDB_GRID_TYPE_GET(grid_type, leaf_size);
                                buf.data = (pnanovdb_uint32_t*)buf_array.data;
                                buf.size_in_words = buf_array.size / 4u;

                                // initialize leaf
                                memset(buf_array.data + leaf.address.byte_offset, 0, PNANOVDB_GRID_TYPE_GET(grid_type, leaf_size));

                                pnanovdb_coord_t leaf_bbox_min = {
                                    (fine_ijk.x >> 3) << 3,
                                    (fine_ijk.y >> 3) << 3,
                                    (fine_ijk.z >> 3) << 3
                                };
                                pnanovdb_leaf_set_bbox_min(buf, leaf, PNANOVDB_REF(leaf_bbox_min));
                                pnanovdb_leaf_set_bbox_dif_and_flags(buf, leaf, 0x00070707);
                            }

                            list_leaf.pushBack(leaf);

                            perLayer.leaf_count++;
                        }
                        cached_leaf[blockIdx] = leaf;
                    }
                }
            }

            pnanovdb_tree_set_node_count_leaf(buf, perLayer.tree, perLayer.leaf_count);
        }
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            const auto& layerParams = ptr->layerParams[layerParamIdx];
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];

            NvFlowUint64 grid_size = nanoVdb->buffer_size_with_leaves;

            pnanovdb_uint32_t gridFlags = PNANOVDB_GRID_FLAGS_HAS_BBOX |
                PNANOVDB_GRID_FLAGS_HAS_MIN_MAX |
                PNANOVDB_GRID_FLAGS_HAS_AVERAGE |
                PNANOVDB_GRID_FLAGS_HAS_STD_DEVIATION;

            // finalize grid header
            pnanovdb_grid_set_magic(buf, perLayer.grid, PNANOVDB_MAGIC_NUMBER);
            pnanovdb_grid_set_checksum(buf, perLayer.grid, 0u);
            pnanovdb_grid_set_version(buf, perLayer.grid, pnanovdb_make_version(PNANOVDB_MAJOR_VERSION_NUMBER, PNANOVDB_MINOR_VERSION_NUMBER, PNANOVDB_PATCH_VERSION_NUMBER));
            pnanovdb_grid_set_flags(buf, perLayer.grid, gridFlags);
            pnanovdb_grid_set_grid_index(buf, perLayer.grid, (pnanovdb_uint32_t)layerParamIdx);
            pnanovdb_grid_set_grid_count(buf, perLayer.grid, (pnanovdb_uint32_t)ptr->layerParams.size);
            pnanovdb_grid_set_grid_size(buf, perLayer.grid, grid_size);

            // placeholder name
            pnanovdb_grid_set_grid_name(buf, perLayer.grid, 0u, 0x776F6C46);
            pnanovdb_grid_set_grid_name(buf, perLayer.grid, 1u, 0u);

            NvFlowFloat3 worldCellSize = {
                layerParams.blockSizeWorld.x / float(levelParams.blockDimLessOne.x + 1u),
                layerParams.blockSizeWorld.y / float(levelParams.blockDimLessOne.y + 1u),
                layerParams.blockSizeWorld.z / float(levelParams.blockDimLessOne.z + 1u)
            };
            NvFlowFloat3 worldCellSizeInv = {
                1.f / worldCellSize.x,
                1.f / worldCellSize.y,
                1.f / worldCellSize.z
            };

            pnanovdb_map_handle_t map = pnanovdb_grid_get_map(buf, perLayer.grid);
            pnanovdb_map_set_matf(buf, map, 0u, worldCellSize.x);
            pnanovdb_map_set_matf(buf, map, 4u, worldCellSize.y);
            pnanovdb_map_set_matf(buf, map, 8u, worldCellSize.z);
            pnanovdb_map_set_invmatf(buf, map, 0u, worldCellSizeInv.x);
            pnanovdb_map_set_invmatf(buf, map, 4u, worldCellSizeInv.y);
            pnanovdb_map_set_invmatf(buf, map, 8u, worldCellSizeInv.z);
            pnanovdb_map_set_matd(buf, map, 0u, worldCellSize.x);
            pnanovdb_map_set_matd(buf, map, 4u, worldCellSize.y);
            pnanovdb_map_set_matd(buf, map, 8u, worldCellSize.z);
            pnanovdb_map_set_invmatd(buf, map, 0u, worldCellSizeInv.x);
            pnanovdb_map_set_invmatd(buf, map, 4u, worldCellSizeInv.y);
            pnanovdb_map_set_invmatd(buf, map, 8u, worldCellSizeInv.z);

            // Note, assuming Flow and VDB both do 0.0 as bottom left of ijk 0
            pnanovdb_map_set_vecf(buf, map, 0u, 0.f);
            pnanovdb_map_set_vecf(buf, map, 1u, 0.f);
            pnanovdb_map_set_vecf(buf, map, 2u, 0.f);
            pnanovdb_map_set_vecd(buf, map, 0u, 0.0);
            pnanovdb_map_set_vecd(buf, map, 1u, 0.0);
            pnanovdb_map_set_vecd(buf, map, 2u, 0.0);

            pnanovdb_grid_set_world_bbox(buf, perLayer.grid, 0u, layerParams.worldMin.x);
            pnanovdb_grid_set_world_bbox(buf, perLayer.grid, 1u, layerParams.worldMin.y);
            pnanovdb_grid_set_world_bbox(buf, perLayer.grid, 2u, layerParams.worldMin.z);
            pnanovdb_grid_set_world_bbox(buf, perLayer.grid, 3u, layerParams.worldMax.x);
            pnanovdb_grid_set_world_bbox(buf, perLayer.grid, 4u, layerParams.worldMax.y);
            pnanovdb_grid_set_world_bbox(buf, perLayer.grid, 5u, layerParams.worldMax.z);

            pnanovdb_grid_set_voxel_size(buf, perLayer.grid, 0u, worldCellSize.x);
            pnanovdb_grid_set_voxel_size(buf, perLayer.grid, 1u, worldCellSize.y);
            pnanovdb_grid_set_voxel_size(buf, perLayer.grid, 2u, worldCellSize.z);

            pnanovdb_uint32_t grid_class = PNANOVDB_GRID_CLASS_UNKNOWN;
            if (grid_type == PNANOVDB_GRID_TYPE_FLOAT)
            {
                grid_class = PNANOVDB_GRID_CLASS_FOG_VOLUME;
            }
            pnanovdb_grid_set_grid_class(buf, perLayer.grid, grid_class);
            pnanovdb_grid_set_grid_type(buf, perLayer.grid, grid_type);
            pnanovdb_grid_set_blind_metadata_offset(buf, perLayer.grid, 0u);
            pnanovdb_grid_set_blind_metadata_count(buf, perLayer.grid, 0u);
        }

        // compute buffer size
        nanoVdb->cache_size = 0llu;
        auto& paramsNanoVdb = nanoVdb->paramsNanoVdb;

        NvFlowSparseNanoVdbParams nullParams = {};
        paramsNanoVdb = nullParams;

        paramsNanoVdb.nanovdb_size_without_leaves = uint64_to_uint2(nanoVdb->buffer_size_without_leaves);
        paramsNanoVdb.nanovdb_size_with_leaves = uint64_to_uint2(nanoVdb->buffer_size_with_leaves);

        paramsNanoVdb.list_tile_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += list_tiles.size * sizeof(pnanovdb_root_tile_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.list_upper_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += list_upper.size * sizeof(pnanovdb_upper_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.list_lower_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += list_lower.size * sizeof(pnanovdb_lower_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.list_leaf_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += list_leaf.size * sizeof(pnanovdb_leaf_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.cache_tile_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += cached_tiles.size * sizeof(pnanovdb_root_tile_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.cache_upper_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += cached_upper.size * sizeof(pnanovdb_upper_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.cache_lower_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += cached_lower.size * sizeof(pnanovdb_lower_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.cache_leaf_offset = uint64_to_uint2(nanoVdb->cache_size);
        nanoVdb->cache_size += cached_leaf.size * sizeof(pnanovdb_leaf_handle_t);
        nanoVdb->cache_size = 32u * ((nanoVdb->cache_size + 31u) / 32u);

        paramsNanoVdb.list_tile_count = (NvFlowUint)list_tiles.size;
        paramsNanoVdb.list_upper_count = (NvFlowUint)list_upper.size;
        paramsNanoVdb.list_lower_count = (NvFlowUint)list_lower.size;
        paramsNanoVdb.list_leaf_count = (NvFlowUint)list_leaf.size;

        paramsNanoVdb.cache_tile_count = (NvFlowUint)cached_tiles.size;
        paramsNanoVdb.cache_upper_count = (NvFlowUint)cached_upper.size;
        paramsNanoVdb.cache_lower_count = (NvFlowUint)cached_lower.size;
        paramsNanoVdb.cache_leaf_count = (NvFlowUint)cached_leaf.size;

        paramsNanoVdb.cache_size = uint64_to_uint2(nanoVdb->cache_size);
        paramsNanoVdb.grid_count = (NvFlowUint)ptr->layerParams.size;
        paramsNanoVdb.grid_type = grid_type;

        paramsNanoVdb.subGridDimBits = subGridDimBits;
        paramsNanoVdb.pad3 = 0u;

        paramsNanoVdb.subGridDimLessOne = subGridDimLessOne;
        paramsNanoVdb.pad4 = 0u;

        if (nanoVdb->cache_size > 0u && buf_array.size >= nanoVdb->buffer_size_without_leaves)
        {
            NvFlowUint8* mappedNanoVdb = (NvFlowUint8*)NvFlowUploadBuffer_map(context, &nanoVdb->uploadBufferNanoVdb, nanoVdb->buffer_size_without_leaves);
            NvFlowUint8* mappedCache = (NvFlowUint8*)NvFlowUploadBuffer_map(context, &nanoVdb->uploadBufferCache, nanoVdb->cache_size);

            memcpy(mappedNanoVdb, buf_array.data, nanoVdb->buffer_size_without_leaves);

            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.list_tile_offset), list_tiles.data, list_tiles.size * sizeof(pnanovdb_root_tile_handle_t));
            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.list_upper_offset), list_upper.data, list_upper.size * sizeof(pnanovdb_upper_handle_t));
            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.list_lower_offset), list_lower.data, list_lower.size * sizeof(pnanovdb_lower_handle_t));
            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.list_leaf_offset), list_leaf.data, list_leaf.size * sizeof(pnanovdb_leaf_handle_t));

            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.cache_tile_offset), cached_tiles.data, cached_tiles.size * sizeof(pnanovdb_root_tile_handle_t));
            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.cache_upper_offset), cached_upper.data, cached_upper.size * sizeof(pnanovdb_upper_handle_t));
            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.cache_lower_offset), cached_lower.data, cached_lower.size * sizeof(pnanovdb_lower_handle_t));
            memcpy(mappedCache + uint2_to_uint64(paramsNanoVdb.cache_leaf_offset), cached_leaf.data, cached_leaf.size * sizeof(pnanovdb_leaf_handle_t));

            NvFlowBufferTransient* nanoVdbBufferTransient = NvFlowUploadBuffer_unmapDevice(context, &nanoVdb->uploadBufferNanoVdb, 0llu, nanoVdb->buffer_size_without_leaves, "SparseNanoVdbBufferUpload");
            if (pNanoVdbBufferTransient)
            {
                *pNanoVdbBufferTransient = nanoVdbBufferTransient;
            }
            NvFlowBufferTransient* cacheBufferTransient = NvFlowUploadBuffer_unmapDevice(context, &nanoVdb->uploadBufferCache, 0llu, nanoVdb->cache_size, "SparseNanoVdbCacheBufferUpload");
            if (pCacheBufferTransient)
            {
                *pCacheBufferTransient = cacheBufferTransient;
            }
        }

        if (pParams)
        {
            *pParams = nanoVdb->paramsNanoVdb;
        }

        #ifdef SPARSE_FILEWRITE
        static int writeFrame = 0;
        writeFrame++;
        if (writeFrame % 1000 == 999)
        {
            if (buf_array.size > 0u)
            {
                const char* path = "../../../data/capture0.nvdb.raw";
                FILE* file = nullptr;
                fopen_s(&file, path, "wb");
                if (file)
                {
                    printf("Writing out capture0.nvdb.raw...\n");

                    fwrite(buf_array.data, 1u, buf_array.size, file);

                    fclose(file);
                }
            }
        }
        #endif

        // validation
        #if 0
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            const auto& layerParams = ptr->layerParams[layerParamIdx];
            auto& perLayer = nanoVdb->perLayers[layerParamIdx];
            int activeLayer = layerParams.layer;

            pnanovdb_buf_t buf = {(uint32_t*)buf_array.data, buf_array.size / 4u};

            pnanovdb_grid_handle_t grid = { layerParamIdx * (PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)};
            pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, grid);
            pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);

            pnanovdb_readaccessor_t readaccessor = {};
            pnanovdb_readaccessor_init(&readaccessor, root);

            NvFlowUint64 faultCount = 0u;
            NvFlowUint64 totalCount = 0u;

            for (NvFlowUint64 blockIdx = 0u; blockIdx < ptr->hashTable.locations.size; blockIdx++)
            {
                NvFlowInt4 location = ptr->hashTable.locations[blockIdx];
                if (location.w == activeLayer)
                {
                    pnanovdb_coord_t ijk = {
                        location.x << levelParams.blockDimBits.x,
                        location.y << levelParams.blockDimBits.y,
                        location.z << levelParams.blockDimBits.z
                    };

                    pnanovdb_address_t addr = pnanovdb_readaccessor_get_value_address(grid_type, buf, &readaccessor, &ijk);

                    // check that readaccessor path is all nonzero
                    if (readaccessor.root.address.byte_offset == 0u ||
                        readaccessor.upper.address.byte_offset == 0u ||
                        readaccessor.lower.address.byte_offset == 0u ||
                        readaccessor.leaf.address.byte_offset == 0u)
                    {
                        faultCount++;
                        if (faultCount < 16u)
                        {
                            printf("Unexpected unmapped ijk(%d, %d, %d) : Accessor leaf(%I64u) lower(%I64u) upper(%I64u) root(%I64u)\n",
                                ijk.x, ijk.y, ijk.z,
                                readaccessor.leaf.address.byte_offset,
                                readaccessor.lower.address.byte_offset,
                                readaccessor.upper.address.byte_offset,
                                readaccessor.root.address.byte_offset
                            );
                            printf("Cache blockIdx(%I64u) root(%I64u) tile(%I64u) upper(%I64u) lower(%I64u) leaf(%I64u)\n",
                                blockIdx,
                                perLayer.root.address.byte_offset,
                                cached_tiles[blockIdx].address.byte_offset,
                                cached_upper[blockIdx].address.byte_offset,
                                cached_lower[blockIdx].address.byte_offset,
                                cached_leaf[blockIdx].address.byte_offset
                            );
                            pnanovdb_root_tile_handle_t tile = pnanovdb_root_find_tile(grid_type, buf, root, &ijk);
                            printf("tile(%I64u)\n", tile.address.byte_offset);
                        }
                    }
                    totalCount++;
                }
            }
            if (faultCount > 0u)
            {
                printf("Layer %d Total fault count = %llu of %llu\n", activeLayer, faultCount, totalCount);
            }
        }
        #endif
    }

    void addPassesNanoVdbComputeStats(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        const NvFlowSparseNanoVdbParams* params,
        NvFlowBufferTransient* nanoVdbBufferTransient,
        NvFlowBufferTransient* cacheBufferTransient,
        NvFlowBufferTransient* targetNanoVdbBuffer
    )
    {
        auto ptr = cast(sparse);

        if (!params || !nanoVdbBufferTransient || !cacheBufferTransient || !targetNanoVdbBuffer)
        {
            return;
        }

        NvFlowDispatchBatches batches0;
        NvFlowDispatchBatches_init(&batches0, params->list_leaf_count);
        NvFlowDispatchBatches batches1;
        NvFlowDispatchBatches_init(&batches1, params->list_lower_count);
        NvFlowDispatchBatches batches2;
        NvFlowDispatchBatches_init(&batches2, params->list_upper_count);
        NvFlowDispatchBatches batches3;
        NvFlowDispatchBatches_init(&batches3, params->grid_count);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches0.size; batchIdx++)
        {
            auto mapped = (SparseNanoVdbComputeStatsParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(SparseNanoVdbComputeStatsParams));

            mapped->blockIdxOffset0 = batches0[batchIdx].blockIdxOffset;
            mapped->blockIdxOffset1 = batchIdx < batches1.size ? batches1[batchIdx].blockIdxOffset : 0u;
            mapped->blockIdxOffset2 = batchIdx < batches2.size ? batches2[batchIdx].blockIdxOffset : 0u;
            mapped->blockIdxOffset3 = batchIdx < batches3.size ? batches3[batchIdx].blockIdxOffset : 0u;
            mapped->nanoVdb = *params;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            batches0[batchIdx].globalTransient = globalTransient;
            if (batchIdx < batches1.size)
            {
                batches1[batchIdx].globalTransient = globalTransient;
            }
            if (batchIdx < batches2.size)
            {
                batches2[batchIdx].globalTransient = globalTransient;
            }
            if (batchIdx < batches3.size)
            {
                batches3[batchIdx].globalTransient = globalTransient;
            }
        }

        NvFlowUint numChannels = 1u;
        if (params->grid_type == PNANOVDB_GRID_TYPE_VEC3F)
        {
            numChannels = 1u + 3u;
        }
        else if (params->grid_type == PNANOVDB_GRID_TYPE_VEC4F)
        {
            numChannels = 1u + 4u;
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches0.size; batchIdx++)
        {
            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = batches0[batchIdx].blockCount;
            gridDim.z = numChannels;

            SparseNanoVdbComputeStats0CS_PassParams params = {};
            params.paramsIn = batches0[batchIdx].globalTransient;
            params.cacheIn = cacheBufferTransient;
            params.nanovdbInOut = targetNanoVdbBuffer;

            SparseNanoVdbComputeStats0CS_addPassCompute(context, &ptr->sparseNanoVdbComputeStats0, gridDim, &params);
        }
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches1.size; batchIdx++)
        {
            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = batches1[batchIdx].blockCount;
            gridDim.z = numChannels;

            SparseNanoVdbComputeStats1CS_PassParams params = {};
            params.paramsIn = batches1[batchIdx].globalTransient;
            params.cacheIn = cacheBufferTransient;
            params.nanovdbInOut = targetNanoVdbBuffer;

            SparseNanoVdbComputeStats1CS_addPassCompute(context, &ptr->sparseNanoVdbComputeStats1, gridDim, &params);
        }
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches2.size; batchIdx++)
        {
            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = batches2[batchIdx].blockCount;
            gridDim.z = numChannels;

            SparseNanoVdbComputeStats2CS_PassParams params = {};
            params.paramsIn = batches2[batchIdx].globalTransient;
            params.cacheIn = cacheBufferTransient;
            params.nanovdbInOut = targetNanoVdbBuffer;

            SparseNanoVdbComputeStats2CS_addPassCompute(context, &ptr->sparseNanoVdbComputeStats2, gridDim, &params);
        }
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches3.size; batchIdx++)
        {
            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = batches3[batchIdx].blockCount;
            gridDim.z = numChannels;

            SparseNanoVdbComputeStats3CS_PassParams params = {};
            params.paramsIn = batches3[batchIdx].globalTransient;
            params.cacheIn = cacheBufferTransient;
            params.nanovdbInOut = targetNanoVdbBuffer;

            SparseNanoVdbComputeStats3CS_addPassCompute(context, &ptr->sparseNanoVdbComputeStats3, gridDim, &params);
        }
    }

    void addPassesNanoVdbPruneLeaves(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        const NvFlowSparseNanoVdbParams* params,
        NvFlowBufferTransient* nanoVdbBufferTransient,
        NvFlowBufferTransient* cacheBufferTransient,
        NvFlowBufferTransient* srcNanoVdbBuffer,
        NvFlowBufferTransient* targetNanoVdbBuffer
    )
    {
        auto ptr = cast(sparse);

        if (!params || !nanoVdbBufferTransient || !cacheBufferTransient || !targetNanoVdbBuffer)
        {
            return;
        }

        NvFlowDispatchBatches batches0;
        NvFlowDispatchBatches_init(&batches0, params->list_lower_count);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches0.size; batchIdx++)
        {
            auto mapped = (SparseNanoVdbComputeStatsParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(SparseNanoVdbComputeStatsParams));

            mapped->blockIdxOffset0 = batches0[batchIdx].blockIdxOffset;
            mapped->blockIdxOffset1 = batches0[batchIdx].blockIdxOffset;
            mapped->blockIdxOffset2 = batches0[batchIdx].blockIdxOffset;
            mapped->blockIdxOffset3 = batches0[batchIdx].blockIdxOffset;
            mapped->nanoVdb = *params;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            batches0[batchIdx].globalTransient = globalTransient;
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches0.size; batchIdx++)
        {
            NvFlowUint3 gridDim = {};
            gridDim.x = 1u;
            gridDim.y = batches0[batchIdx].blockCount;
            gridDim.z = 1u;

            SparseNanoVdbPruneLeavesCS_PassParams params = {};
            params.paramsIn = batches0[batchIdx].globalTransient;
            params.cacheIn = cacheBufferTransient;
            params.nanovdbSrcIn = srcNanoVdbBuffer;
            params.nanovdbInOut = targetNanoVdbBuffer;

            SparseNanoVdbPruneLeavesCS_addPassCompute(context, &ptr->sparseNanoVdbPruneLeaves, gridDim, &params);
        }
    }

    void addPassesClearNew(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        const NvFlowSparseTexture* valueInOut
    )
    {
        auto ptr = cast(sparse);

        auto levelParams = &valueInOut->sparseParams.levels[valueInOut->levelIdx];

        if (levelParams->numNewLocations == 0u)
        {
            return;
        }

        NvFlowUint numChannels = getFormatChannelCount(ptr, valueInOut->format);

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, levelParams->numNewLocations);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (SparseClearNewParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(SparseClearNewParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;
            mapped->tableParams = *levelParams;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            if (numChannels == 1u)
            {
                SparseClearNew1CS_PassParams params = {};
                params.paramsIn = batches[batchIdx].globalTransient;
                params.gTable = valueInOut->sparseBuffer;
                params.valueOut = valueInOut->textureTransient;

                SparseClearNew1CS_addPassCompute(context, &ptr->sparseClearNew1CS, gridDim, &params);
            }
            if (numChannels == 2u)
            {
                SparseClearNew2CS_PassParams params = {};
                params.paramsIn = batches[batchIdx].globalTransient;
                params.gTable = valueInOut->sparseBuffer;
                params.valueOut = valueInOut->textureTransient;

                SparseClearNew2CS_addPassCompute(context, &ptr->sparseClearNew2CS, gridDim, &params);
            }
            else // assume 4
            {
                SparseClearNew4CS_PassParams params = {};
                params.paramsIn = batches[batchIdx].globalTransient;
                params.gTable = valueInOut->sparseBuffer;
                params.valueOut = valueInOut->textureTransient;

                SparseClearNew4CS_addPassCompute(context, &ptr->sparseClearNew4CS, gridDim, &params);
            }
        }
    }

    void addPassesClearTexture(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowFormat format,
        NvFlowInt3 clearOffset,
        NvFlowInt3 clearExtent,
        NvFlowTextureTransient* value
        )
    {
        auto ptr = cast(sparse);

        NvFlowUint numChannels = getFormatChannelCount(ptr, format);

        auto mapped = (SparseClearTextureParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(SparseClearTextureParams));

        mapped->clearOffset = clearOffset;
        mapped->pad0 = 0;
        mapped->clearExtent = clearExtent;
        mapped->pad1 = 0;

        NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

        NvFlowUint3 gridDim = {};
        gridDim.x = (clearExtent.x + 7u) / 8u;
        gridDim.y = (clearExtent.y + 7u) / 8u;
        gridDim.z = (clearExtent.z + 7u) / 8u;

        if (numChannels == 1u)
        {
            SparseClearTexture1CS_PassParams params = {};
            params.paramsIn = constantTransient;
            params.valueOut = value;

            SparseClearTexture1CS_addPassCompute(context, &ptr->sparseClearTexture1CS, gridDim, &params);
        }
        if (numChannels == 2u)
        {
            SparseClearTexture2CS_PassParams params = {};
            params.paramsIn = constantTransient;
            params.valueOut = value;

            SparseClearTexture2CS_addPassCompute(context, &ptr->sparseClearTexture2CS, gridDim, &params);
        }
        else // assume 4
        {
            SparseClearTexture4CS_PassParams params = {};
            params.paramsIn = constantTransient;
            params.valueOut = value;

            SparseClearTexture4CS_addPassCompute(context, &ptr->sparseClearTexture4CS, gridDim, &params);
        }
    }

    NvFlowBufferTransient* getSparseBuffer(NvFlowContext* context, NvFlowSparse* sparse)
    {
        auto ptr = cast(sparse);
        return NvFlowDynamicBuffer_getTransient(context, &ptr->sparseBuffer[ptr->sparseBufferIdx]);
    }

    NvFlowBufferTransient* getSparseBufferOld(NvFlowContext* context, NvFlowSparse* sparse)
    {
        auto ptr = cast(sparse);
        return NvFlowDynamicBuffer_getTransient(context, &ptr->sparseBuffer[ptr->sparseBufferIdx ^ 1u]);
    }

    void addPassesRescale(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowTextureTransient* oldTexture,
        const NvFlowSparseTexture* valueOut,
        NvFlowBool32 inplaceRescale
    )
    {
        auto ptr = cast(sparse);

        if (valueOut->levelIdx >= ptr->levelParams.size || valueOut->levelIdx >= ptr->levelParamsOld.size)
        {
            return;
        }

        NvFlowUint numChannels = getFormatChannelCount(ptr, valueOut->format);

        auto levelParams = &valueOut->sparseParams.levels[valueOut->levelIdx];
        auto levelParamsOld = &ptr->levelParamsOld[valueOut->levelIdx];

        if (levelParams->numLocations == 0u)
        {
            return;
        }

        auto mappedLayer = (SparseRescaleLayerParams*)NvFlowUploadBuffer_map(context, &ptr->rescaleBuffer, valueOut->sparseParams.layerCount * sizeof(SparseRescaleLayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < valueOut->sparseParams.layerCount; layerParamIdx++)
        {
            const auto layerParams = &valueOut->sparseParams.layers[layerParamIdx];
            NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
            NvFlowFloat3 blockSizeWorldOld = layerParams->blockSizeWorld;
            for (NvFlowUint64 layerParamIdxOld = 0u; layerParamIdxOld < ptr->layerParamsOld.size; layerParamIdxOld++)
            {
                if (ptr->layerParamsOld[layerParamIdxOld].layerAndLevel == layerParams->layerAndLevel)
                {
                    blockSizeWorldOld = ptr->layerParamsOld[layerParamIdxOld].blockSizeWorld;
                    break;
                }
            }
            NvFlowBool32 shouldClear = (layerParamIdx < ptr->layerIsCleared.size) ? ptr->layerIsCleared[layerParamIdx] : NV_FLOW_FALSE;
            mappedLayer[layerParamIdx].blockSizeWorld = blockSizeWorld;
            mappedLayer[layerParamIdx].shouldClear = shouldClear;
            mappedLayer[layerParamIdx].blockSizeWorldOld = blockSizeWorldOld;
            mappedLayer[layerParamIdx].pad2 = 0.f;

            //printf("rescale[%d] factor %f\n", layerParams->layer, blockSizeWorld.x / blockSizeWorldOld.x);
        }

        auto rescaleTransient = NvFlowUploadBuffer_unmap(context, &ptr->rescaleBuffer);

        // intentionally aligns with scratch in EmitterPoint.cpp
        static const NvFlowUint tempCubeDim = 16u;
        static const NvFlowUint tempCubeDim3 = 4096u;
        NvFlowSparseTexture valueTemp = {};
        if (inplaceRescale)
        {
            valueTemp = *valueOut;

            NvFlowSparseLevelParams* levelParams = &valueTemp.sparseParams.levels[valueTemp.levelIdx];

            NvFlowTextureDesc texDesc = { eNvFlowTextureType_3d };
            texDesc.textureType = eNvFlowTextureType_3d;
            texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
            texDesc.format = valueTemp.format;
            texDesc.width = tempCubeDim * (levelParams->blockDimLessOne.x + 1u);
            texDesc.height = tempCubeDim * (levelParams->blockDimLessOne.y + 1u);
            texDesc.depth = tempCubeDim * (levelParams->blockDimLessOne.z + 1u);
            texDesc.mipLevels = 1u;

            valueTemp.textureTransient = ptr->contextInterface.getTextureTransient(context, &texDesc);
        }

        NvFlowDispatchBatches batches;
        if (inplaceRescale)
        {
            NvFlowDispatchBatches_init_custom(&batches, levelParams->numLocations, tempCubeDim3);
        }
        else
        {
            NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (SparseRescaleGlobalParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(SparseRescaleGlobalParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;
            mapped->tableParams = *levelParams;
            mapped->tableParamsOld = *levelParamsOld;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        if (inplaceRescale)
        {
            if (numChannels == 4u)
            {
                NvFlowBufferTransient* tableOld = getSparseBufferOld(context, sparse);
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;
                    {
                        SparseInplaceRescale4aCS_PassParams params = {};
                        params.globalParamsIn = batches[batchIdx].globalTransient;
                        params.layerParamsIn = rescaleTransient;
                        params.gTable = valueOut->sparseBuffer;
                        params.gTableOld = tableOld;
                        params.valueIn = valueOut->textureTransient;
                        params.valueOut = valueTemp.textureTransient;

                        SparseInplaceRescale4aCS_addPassCompute(context, &ptr->sparseInplaceRescale4aCS, gridDim, &params);
                    }
                    {
                        SparseInplaceRescale4bCS_PassParams params = {};
                        params.globalParamsIn = batches[batchIdx].globalTransient;
                        params.layerParamsIn = rescaleTransient;
                        params.gTable = valueOut->sparseBuffer;
                        params.valueIn = valueTemp.textureTransient;
                        params.valueOut = valueOut->textureTransient;

                        SparseInplaceRescale4bCS_addPassCompute(context, &ptr->sparseInplaceRescale4bCS, gridDim, &params);
                    }
                }
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;
                    {
                        SparseInplaceRescale4cCS_PassParams params = {};
                        params.globalParamsIn = batches[batchIdx].globalTransient;
                        params.layerParamsIn = rescaleTransient;
                        params.gTable = valueOut->sparseBuffer;
                        params.valueIn = valueOut->textureTransient;
                        params.valueOut = valueTemp.textureTransient;

                        SparseInplaceRescale4cCS_addPassCompute(context, &ptr->sparseInplaceRescale4cCS, gridDim, &params);
                    }
                    {
                        SparseInplaceRescale4dCS_PassParams params = {};
                        params.globalParamsIn = batches[batchIdx].globalTransient;
                        params.layerParamsIn = rescaleTransient;
                        params.gTable = valueOut->sparseBuffer;
                        params.valueIn = valueTemp.textureTransient;
                        params.valueOut = valueOut->textureTransient;

                        SparseInplaceRescale4dCS_addPassCompute(context, &ptr->sparseInplaceRescale4dCS, gridDim, &params);
                    }
                }
            }
        }
        else if (oldTexture)
        {
            if (numChannels == 1u)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    SparseRescale1CS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = rescaleTransient;
                    params.gTable = valueOut->sparseBuffer;
                    params.gTableOld = getSparseBufferOld(context, sparse);
                    params.valueSampler = ptr->samplerLinear;
                    params.valueIn = oldTexture;
                    params.valueOut = valueOut->textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    SparseRescale1CS_addPassCompute(context, &ptr->sparseRescale1CS, gridDim, &params);
                }
            }
            if (numChannels == 2u)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    SparseRescale2CS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = rescaleTransient;
                    params.gTable = valueOut->sparseBuffer;
                    params.gTableOld = getSparseBufferOld(context, sparse);
                    params.valueSampler = ptr->samplerLinear;
                    params.valueIn = oldTexture;
                    params.valueOut = valueOut->textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    SparseRescale2CS_addPassCompute(context, &ptr->sparseRescale2CS, gridDim, &params);
                }
            }
            else // assume 4
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    SparseRescale4CS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = rescaleTransient;
                    params.gTable = valueOut->sparseBuffer;
                    params.gTableOld = getSparseBufferOld(context, sparse);
                    params.valueSampler = ptr->samplerLinear;
                    params.valueIn = oldTexture;
                    params.valueOut = valueOut->textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    SparseRescale4CS_addPassCompute(context, &ptr->sparseRescale4CS, gridDim, &params);
                }
            }
        }
        else // old texture is nullptr
        {
            if (numChannels == 1u)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    SparseClearLayers1CS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = rescaleTransient;
                    params.gTable = valueOut->sparseBuffer;
                    params.valueOut = valueOut->textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    SparseClearLayers1CS_addPassCompute(context, &ptr->sparseClearLayers1CS, gridDim, &params);
                }
            }
            if (numChannels == 2u)
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    SparseClearLayers2CS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = rescaleTransient;
                    params.gTable = valueOut->sparseBuffer;
                    params.valueOut = valueOut->textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    SparseClearLayers2CS_addPassCompute(context, &ptr->sparseClearLayers2CS, gridDim, &params);
                }
            }
            else // assume 4
            {
                for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
                {
                    SparseClearLayers4CS_PassParams params = {};
                    params.globalParamsIn = batches[batchIdx].globalTransient;
                    params.layerParamsIn = rescaleTransient;
                    params.gTable = valueOut->sparseBuffer;
                    params.valueOut = valueOut->textureTransient;

                    NvFlowUint3 gridDim = {};
                    gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                    gridDim.y = batches[batchIdx].blockCount;
                    gridDim.z = 1u;

                    SparseClearLayers4CS_addPassCompute(context, &ptr->sparseClearLayers4CS, gridDim, &params);
                }
            }
        }
    }

    void addPassesMigrate(
        NvFlowContext* context,
        NvFlowSparse* sparse,
        NvFlowBool32 lowPrecisionRescale,
        NvFlowTextureTransient* oldTextureTransient,
        const NvFlowTextureDesc* oldTexDesc,
        NvFlowFormat targetFormat,
        NvFlowUint targetLevelIdx,
        NvFlowSparseTexture* valueOut,
        NvFlowTextureDesc* texDescOut
    )
    {
        auto ptr = cast(sparse);

        if (targetLevelIdx >= ptr->levelParams.size)
        {
            return;
        }

        const NvFlowSparseLevelParams* levelParams = &ptr->levelParams[targetLevelIdx];

        // generate target desc
        NvFlowTextureDesc targetDesc = {};
        targetDesc.textureType = eNvFlowTextureType_3d;
        targetDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        targetDesc.format = targetFormat;
        targetDesc.width = levelParams->dim.x;
        targetDesc.height = levelParams->dim.y;
        targetDesc.depth = levelParams->dim.z;
        targetDesc.mipLevels = 1u;

        // key conditions for clear/migration
        NvFlowBool32 mustReallocate =
            oldTexDesc->format != targetDesc.format ||
            oldTexDesc->width  != targetDesc.width  ||
            oldTexDesc->height != targetDesc.height ||
            oldTexDesc->depth  != targetDesc.depth ||
            !oldTextureTransient;
        NvFlowBool32 anyGridReset = NV_FLOW_FALSE;
        NvFlowBool32 anyRescaleEvent = NV_FLOW_FALSE;
        NvFlowBool32 anyClearLayers = NV_FLOW_FALSE;

        ptr->layerIsCleared.size = 0u;
        ptr->layerIsCleared.reserve(ptr->layerParams.size);
        ptr->layerIsCleared.size = ptr->layerParams.size;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->layerParams.size; layerParamIdx++)
        {
            const auto layerParams = &ptr->layerParams[layerParamIdx];
            const auto simLayerParams = &ptr->simLayerParams[layerParamIdx];

            // determine if layer rescaled
            NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
            NvFlowFloat3 blockSizeWorldOld = layerParams->blockSizeWorld;
            for (NvFlowUint64 layerParamIdxOld = 0u; layerParamIdxOld < ptr->layerParamsOld.size; layerParamIdxOld++)
            {
                if (ptr->layerParamsOld[layerParamIdxOld].layerAndLevel == layerParams->layerAndLevel)
                {
                    blockSizeWorldOld = ptr->layerParamsOld[layerParamIdxOld].blockSizeWorld;
                    break;
                }
            }
            NvFlowBool32 layerRescaled =
                blockSizeWorld.x != blockSizeWorldOld.x ||
                blockSizeWorld.y != blockSizeWorldOld.y ||
                blockSizeWorld.z != blockSizeWorldOld.z;

            // check if this layer is requested to clear
            NvFlowBool32 layerIsCleared = NV_FLOW_FALSE;
            if (layerRescaled && simLayerParams->clearOnRescale)
            {
                layerIsCleared = NV_FLOW_TRUE;
            }
            ptr->layerIsCleared[layerParamIdx] = layerIsCleared;

            if (layerParams->gridReset)
            {
                anyGridReset = NV_FLOW_TRUE;
            }
            if (!layerIsCleared && layerRescaled && layerParams->numLocations > 0u)
            {
                anyRescaleEvent = NV_FLOW_TRUE;
            }
            if (layerIsCleared)
            {
                anyClearLayers = NV_FLOW_TRUE;
            }
        }

        NvFlowSparseParams sparseParams = {};
        getParams(sparse, &sparseParams);

        NvFlowSparseTexture targetSparseTexture = {};
        targetSparseTexture.textureTransient = nullptr;
        targetSparseTexture.sparseBuffer = getSparseBuffer(context, sparse);
        targetSparseTexture.sparseParams = sparseParams;
        targetSparseTexture.levelIdx = targetLevelIdx;
        targetSparseTexture.format = targetDesc.format;

        if (mustReallocate || anyGridReset)
        {
            targetSparseTexture.textureTransient = ptr->contextInterface.getTextureTransient(context, &targetDesc);

            // note: if oldTextureTransient is false, mustReallocate is always true
            if (mustReallocate)
            {
                // clear entire texture on reallocate, just to avoid minimize risk of non-Flow stale data
                NvFlowInt3 clearOffset = { 0, 0, 0 };
                NvFlowInt3 clearExtent = { int(targetDesc.width), int(targetDesc.height), int(targetDesc.depth) };
                addPassesClearTexture(context, sparse, targetDesc.format, clearOffset, clearExtent, targetSparseTexture.textureTransient);
            }
            // rescale only makes sense if old texture was valid
            if (oldTextureTransient)
            {
                addPassesRescale(context, sparse, oldTextureTransient, &targetSparseTexture, NV_FLOW_FALSE);
            }
        }
        else if (anyRescaleEvent && lowPrecisionRescale)
        {
            targetSparseTexture.textureTransient = oldTextureTransient;

            addPassesRescale(context, sparse, nullptr, &targetSparseTexture, NV_FLOW_TRUE);
        }
        else if (anyRescaleEvent)
        {
            targetSparseTexture.textureTransient = ptr->contextInterface.getTextureTransient(context, &targetDesc);

            addPassesRescale(context, sparse, oldTextureTransient, &targetSparseTexture, NV_FLOW_FALSE);
        }
        else if (anyClearLayers)
        {
            targetSparseTexture.textureTransient = oldTextureTransient;

            addPassesRescale(context, sparse, nullptr, &targetSparseTexture, NV_FLOW_FALSE);

            addPassesClearNew(context, sparse, &targetSparseTexture);
        }
        else
        {
            targetSparseTexture.textureTransient = oldTextureTransient;

            addPassesClearNew(context, sparse, &targetSparseTexture);
        }

        if (valueOut)
        {
            *valueOut = targetSparseTexture;
        }
        if (texDescOut)
        {
            *texDescOut = targetDesc;
        }
    }
}

NvFlowSparseInterface* NvFlowGetSparseInterface()
{
    using namespace NvFlowSparseDefault;
    static NvFlowSparseInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowSparseInterface) };
    iface.create = create;
    iface.destroy = destroy;
    iface.reset = reset;
    iface.updateLayers = updateLayers;
    iface.testUpdateLocations = testUpdateLocations;
    iface.updateLocations = updateLocations;
    iface.updateLayerDeltaTimes = updateLayerDeltaTimes;
    iface.getParams = getParams;
    iface.getSimParams = getSimParams;
    iface.addPasses = addPasses;
    iface.addPassesNanoVdb = addPassesNanoVdb;
    iface.addPassesNanoVdbComputeStats = addPassesNanoVdbComputeStats;
    iface.addPassesMigrate = addPassesMigrate;
    iface.getSparseBuffer = getSparseBuffer;
    iface.addPassesNanoVdbPruneLeaves = addPassesNanoVdbPruneLeaves;
    return &iface;
}
