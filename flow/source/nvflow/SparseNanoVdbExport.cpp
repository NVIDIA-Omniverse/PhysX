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
#include "NvFlowReadbackBuffer.h"
#include "NvFlowInteropBuffer.h"

#include "NvFlow.h"

#define PNANOVDB_C
#define PNANOVDB_BUF_BOUNDS_CHECK

#include "nanovdb/PNanoVDB.h"

#include "shaders/SparseNanoVdbExportDensityCS.hlsl.h"
#include "shaders/SparseNanoVdbExportVelocityCS.hlsl.h"

namespace
{
    struct SparseNanoVdbExportHistory
    {
        NvFlowUint64 frame;
        double absoluteSimTime;
    };

    struct SparseNanoVdbExport
    {
        NvFlowContextInterface contextInterface = {};
        NvFlowSparseInterface sparseInterface = {};

        SparseNanoVdbExportDensityCS_Pipeline sparseNanoVdbExportDensityCS = {};
        SparseNanoVdbExportVelocityCS_Pipeline sparseNanoVdbExportVelocityCS = {};

        NvFlowUploadBuffer constantBuffer = {};
        NvFlowUploadBuffer nullGrid = {};

        NvFlowDynamicBuffer temperatureBuffer = {};
        NvFlowDynamicBuffer fuelBuffer = {};
        NvFlowDynamicBuffer burnBuffer = {};
        NvFlowDynamicBuffer smokeBuffer = {};
        NvFlowDynamicBuffer velocityBuffer = {};
        NvFlowDynamicBuffer divergenceBuffer = {};
        NvFlowDynamicBuffer rgbaBuffer = {};
        NvFlowDynamicBuffer rgbBuffer = {};

        NvFlowDynamicBuffer temperatureLiteBuffer = {};
        NvFlowDynamicBuffer fuelLiteBuffer = {};
        NvFlowDynamicBuffer burnLiteBuffer = {};
        NvFlowDynamicBuffer smokeLiteBuffer = {};
        NvFlowDynamicBuffer velocityLiteBuffer = {};
        NvFlowDynamicBuffer divergenceLiteBuffer = {};
        NvFlowDynamicBuffer rgbaLiteBuffer = {};
        NvFlowDynamicBuffer rgbLiteBuffer = {};

        NvFlowReadbackBuffer temperatureReadback = {};
        NvFlowReadbackBuffer fuelReadback = {};
        NvFlowReadbackBuffer burnReadback = {};
        NvFlowReadbackBuffer smokeReadback = {};
        NvFlowReadbackBuffer velocityReadback = {};
        NvFlowReadbackBuffer divergenceReadback = {};
        NvFlowReadbackBuffer rgbaReadback = {};
        NvFlowReadbackBuffer rgbReadback = {};

        NvFlowInteropBuffer temperatureInterop = {};
        NvFlowInteropBuffer fuelInterop = {};
        NvFlowInteropBuffer burnInterop = {};
        NvFlowInteropBuffer smokeInterop = {};
        NvFlowInteropBuffer velocityInterop = {};
        NvFlowInteropBuffer divergenceInterop = {};
        NvFlowInteropBuffer rgbaInterop = {};
        NvFlowInteropBuffer rgbInterop = {};

        NvFlowArray<NvFlowSparseNanoVdbExportReadback> readbacks;

        NvFlowArray<NvFlowSparseNanoVdbExportInterop> interops;

        NvFlowArray<SparseNanoVdbExportHistory> history;
    };

    NvFlowUint3 uint3_max(NvFlowUint3 a, NvFlowUint3 b) {
        NvFlowUint3 c = {
            c.x = a.x > b.x ? a.x : b.x,
            c.y = a.y > b.y ? a.y : b.y,
            c.z = a.z > b.z ? a.z : b.z
        };
        return c;
    };

    SparseNanoVdbExport* SparseNanoVdbExport_create(const NvFlowOpInterface* opInterface, const NvFlowSparseNanoVdbExportPinsIn* in, NvFlowSparseNanoVdbExportPinsOut* out)
    {
        auto ptr = new SparseNanoVdbExport();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        SparseNanoVdbExportDensityCS_init(&ptr->contextInterface, in->context, &ptr->sparseNanoVdbExportDensityCS);
        SparseNanoVdbExportVelocityCS_init(&ptr->contextInterface, in->context, &ptr->sparseNanoVdbExportVelocityCS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->nullGrid, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(NvFlowUint));


        NvFlowBufferUsageFlags flags =
            eNvFlowBufferUsage_structuredBuffer |
            eNvFlowBufferUsage_rwStructuredBuffer |
            eNvFlowBufferUsage_bufferCopySrc |
            eNvFlowBufferUsage_bufferCopyDst;

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->temperatureBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->fuelBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->burnBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->smokeBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->velocityBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->divergenceBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbaBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->temperatureLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->fuelLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->burnLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->smokeLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->velocityLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->divergenceLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbaLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbLiteBuffer, flags, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->temperatureReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->fuelReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->burnReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->smokeReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->velocityReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->divergenceReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbaReadback);
        NvFlowReadbackBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbReadback);

        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->temperatureInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->fuelInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->burnInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->smokeInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->velocityInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->divergenceInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbaInterop);
        NvFlowInteropBuffer_init(&ptr->contextInterface, in->context, &ptr->rgbInterop);

        return ptr;
    }

    void SparseNanoVdbExport_unmap(SparseNanoVdbExport* ptr, const NvFlowSparseNanoVdbExportPinsIn* in, NvFlowSparseNanoVdbExportPinsOut* out)
    {
        for (NvFlowUint activeIdx = 0u; activeIdx < ptr->readbacks.size; activeIdx++)
        {
            NvFlowReadbackBuffer_unmap(in->context, &ptr->temperatureReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->fuelReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->burnReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->smokeReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->velocityReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->divergenceReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->rgbaReadback, activeIdx);
            NvFlowReadbackBuffer_unmap(in->context, &ptr->rgbReadback, activeIdx);
        }
        ptr->readbacks.size = 0u;
    }

    void SparseNanoVdbExport_destroy(SparseNanoVdbExport* ptr, const NvFlowSparseNanoVdbExportPinsIn* in, NvFlowSparseNanoVdbExportPinsOut* out)
    {
        SparseNanoVdbExport_unmap(ptr, in, out);

        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->nullGrid);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->temperatureBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->fuelBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->burnBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->smokeBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->velocityBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->divergenceBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->rgbaBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->rgbBuffer);

        NvFlowDynamicBuffer_destroy(in->context, &ptr->temperatureLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->fuelLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->burnLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->smokeLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->velocityLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->divergenceLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->rgbaLiteBuffer);
        NvFlowDynamicBuffer_destroy(in->context, &ptr->rgbLiteBuffer);

        NvFlowReadbackBuffer_destroy(in->context, &ptr->temperatureReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->fuelReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->burnReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->smokeReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->velocityReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->divergenceReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->rgbaReadback);
        NvFlowReadbackBuffer_destroy(in->context, &ptr->rgbReadback);

        NvFlowInteropBuffer_destroy(in->context, &ptr->temperatureInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->fuelInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->burnInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->smokeInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->velocityInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->divergenceInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->rgbaInterop);
        NvFlowInteropBuffer_destroy(in->context, &ptr->rgbInterop);

        SparseNanoVdbExportDensityCS_destroy(in->context, &ptr->sparseNanoVdbExportDensityCS);
        SparseNanoVdbExportVelocityCS_destroy(in->context, &ptr->sparseNanoVdbExportVelocityCS);

        delete ptr;
    }

    NvFlowUint2 uint64_to_uint2(NvFlowUint64 value)
    {
        return *((NvFlowUint2*)&value);
    }

    NvFlowUint64 uint2_to_uint64(NvFlowUint2 value)
    {
        return *((NvFlowUint64*)&value);
    }

    double SparseNanoVdbExport_historyAbsoluteSimTime(SparseNanoVdbExport* ptr, NvFlowUint64 frame)
    {
        double lastCompletedAbsoluteTime = 0.0;
        for (NvFlowUint64 historyReadIdx = 0llu; historyReadIdx < ptr->history.size; historyReadIdx++)
        {
            if (ptr->history[historyReadIdx].frame <= frame)
            {
                lastCompletedAbsoluteTime = ptr->history[historyReadIdx].absoluteSimTime;
            }
        }
        return lastCompletedAbsoluteTime;
    }

    void SparseNanoVdbExport_execute(SparseNanoVdbExport* ptr, const NvFlowSparseNanoVdbExportPinsIn* in, NvFlowSparseNanoVdbExportPinsOut* out)
    {
        NvFlowSparseLevelParams* densityLevelParams = &in->density.sparseParams.levels[in->density.levelIdx];
        NvFlowSparseLevelParams* velocityLevelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        NvFlowUint64 currentFrame = ptr->contextInterface.getCurrentGlobalFrame(in->context);
        NvFlowUint64 lastCompletedFrame = ptr->contextInterface.getLastGlobalFrameCompleted(in->context);

        // push new history
        SparseNanoVdbExportHistory newHistory = { currentFrame, in->absoluteSimTime };
        ptr->history.pushBack(newHistory);

        // compute discard index
        NvFlowUint64 historyDiscardIdx = 0llu;
        for (; historyDiscardIdx < ptr->history.size; historyDiscardIdx++)
        {
            if (ptr->history[historyDiscardIdx].frame >= lastCompletedFrame)
            {
                break;
            }
        }
        // retain one old record
        historyDiscardIdx = historyDiscardIdx == 0llu ? 0llu : historyDiscardIdx - 1llu;
        // compact away old history
        NvFlowUint64 historyWriteIdx = 0llu;
        for (NvFlowUint64 historyReadIdx = historyDiscardIdx; historyReadIdx < ptr->history.size; historyReadIdx++)
        {
            ptr->history[historyWriteIdx] = ptr->history[historyReadIdx];
            historyWriteIdx++;
        }
        ptr->history.size = historyWriteIdx;

        out->lastAbsoluteSimTimeCompleted = SparseNanoVdbExport_historyAbsoluteSimTime(ptr, lastCompletedFrame);

        NvFlowUint numLayers = in->density.sparseParams.layerCount;
        // if all layers are disabled, can do passthrough
        NvFlowBool32 anyEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyTemperatureEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyFuelEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyBurnEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anySmokeEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyVelocityEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyDivergenceEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyRgbaEnabled = NV_FLOW_FALSE;
        NvFlowBool32 anyEnableLowPrecisionRgba = NV_FLOW_FALSE;
        NvFlowBool32 anyRgbEnabled = NV_FLOW_FALSE;
        if (densityLevelParams->numLocations > 0u)
        {
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
            {
                auto layerParamsIn = in->params[layerParamIdx];
                if (layerParamsIn->enabled)
                {
                    anyEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->temperatureEnabled)
                {
                    anyTemperatureEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->fuelEnabled)
                {
                    anyFuelEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->burnEnabled)
                {
                    anyBurnEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->smokeEnabled)
                {
                    anySmokeEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->velocityEnabled)
                {
                    anyVelocityEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->divergenceEnabled)
                {
                    anyDivergenceEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->rgbaEnabled)
                {
                    anyRgbaEnabled = NV_FLOW_TRUE;
                }
                if (layerParamsIn->enableLowPrecisionRgba)
                {
                    anyEnableLowPrecisionRgba = NV_FLOW_TRUE;
                }
                if (layerParamsIn->rgbEnabled)
                {
                    anyRgbEnabled = NV_FLOW_TRUE;
                }
            }
        }
        if (!anyEnabled)
        {
            NvFlowSparseNanoVdbExportPinsOut nullOut = {};
            *out = nullOut;
            return;
        }

        // import during execute, since sparseInterface not valid during init
        if (!ptr->sparseInterface.addPassesNanoVdb)
        {
            NvFlowSparseInterface_duplicate(&ptr->sparseInterface, in->sparseInterface);
        }

        auto nullGridMapped = (pnanovdb_uint32_t*)NvFlowUploadBuffer_map(in->context, &ptr->nullGrid, PNANOVDB_GRID_SIZE);
        for (pnanovdb_uint32_t idx = 0u; idx < PNANOVDB_GRID_SIZE / 4u; idx++)
        {
            nullGridMapped[idx] = 0u;
        }
        NvFlowBufferTransient* nullGridTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->nullGrid);

        NvFlowSparseNanoVdbParams densityFloatNanoVdbParams = {};
        NvFlowBufferTransient* densityFloatNanoVdbBuffer = nullptr;
        NvFlowBufferTransient* densityFloatCacheBuffer = nullptr;
        if (anyTemperatureEnabled || anyFuelEnabled || anyBurnEnabled || anySmokeEnabled)
        {
            ptr->sparseInterface.addPassesNanoVdb(
                in->context,
                in->sparse,
                PNANOVDB_GRID_TYPE_FLOAT,
                in->density.levelIdx,
                &densityFloatNanoVdbParams,
                &densityFloatNanoVdbBuffer,
                &densityFloatCacheBuffer
            );
        }
        else
        {
            densityFloatNanoVdbParams.nanovdb_size_with_leaves.x = PNANOVDB_GRID_SIZE;
            densityFloatNanoVdbParams.nanovdb_size_without_leaves.x = PNANOVDB_GRID_SIZE;
            densityFloatNanoVdbBuffer = nullGridTransient;
            densityFloatCacheBuffer = nullGridTransient;
        }

        NvFlowSparseNanoVdbParams rgbaNanoVdbParams = {};
        NvFlowBufferTransient* rgbaNanoVdbBuffer = nullptr;
        NvFlowBufferTransient* rgbaCacheBuffer = nullptr;
        if (anyRgbaEnabled)
        {
            ptr->sparseInterface.addPassesNanoVdb(
                in->context,
                in->sparse,
                anyEnableLowPrecisionRgba ? PNANOVDB_GRID_TYPE_RGBA8 : PNANOVDB_GRID_TYPE_VEC4F,
                in->density.levelIdx,
                &rgbaNanoVdbParams,
                &rgbaNanoVdbBuffer,
                &rgbaCacheBuffer
            );
        }
        else
        {
            rgbaNanoVdbParams.nanovdb_size_with_leaves.x = PNANOVDB_GRID_SIZE;
            rgbaNanoVdbParams.nanovdb_size_without_leaves.x = PNANOVDB_GRID_SIZE;
            rgbaNanoVdbBuffer = nullGridTransient;
            rgbaCacheBuffer = nullGridTransient;
        }

        NvFlowSparseNanoVdbParams rgbNanoVdbParams = {};
        NvFlowBufferTransient* rgbNanoVdbBuffer = nullptr;
        NvFlowBufferTransient* rgbCacheBuffer = nullptr;
        if (anyRgbEnabled)
        {
            ptr->sparseInterface.addPassesNanoVdb(
                in->context,
                in->sparse,
                PNANOVDB_GRID_TYPE_VEC3F,
                in->density.levelIdx,
                &rgbNanoVdbParams,
                &rgbNanoVdbBuffer,
                &rgbCacheBuffer
            );
        }
        else
        {
            rgbNanoVdbParams.nanovdb_size_with_leaves.x = PNANOVDB_GRID_SIZE;
            rgbNanoVdbParams.nanovdb_size_without_leaves.x = PNANOVDB_GRID_SIZE;
            rgbNanoVdbBuffer = nullGridTransient;
            rgbCacheBuffer = nullGridTransient;
        }

        // ensure subGridDim is valid
        densityFloatNanoVdbParams.subGridDimLessOne = uint3_max(densityFloatNanoVdbParams.subGridDimLessOne,
            uint3_max(rgbaNanoVdbParams.subGridDimLessOne, rgbNanoVdbParams.subGridDimLessOne));
        densityFloatNanoVdbParams.subGridDimBits = uint3_max(densityFloatNanoVdbParams.subGridDimBits,
            uint3_max(rgbaNanoVdbParams.subGridDimBits, rgbNanoVdbParams.subGridDimBits));

        NvFlowSparseNanoVdbParams velocityFloatNanoVdbParams = {};
        NvFlowBufferTransient* velocityFloatNanoVdbBuffer = nullptr;
        NvFlowBufferTransient* velocityFloatCacheBuffer = nullptr;
        if (anyDivergenceEnabled)
        {
            ptr->sparseInterface.addPassesNanoVdb(
                in->context,
                in->sparse,
                PNANOVDB_GRID_TYPE_FLOAT,
                in->velocity.levelIdx,
                &velocityFloatNanoVdbParams,
                &velocityFloatNanoVdbBuffer,
                &velocityFloatCacheBuffer
            );
        }
        else
        {
            velocityFloatNanoVdbParams.nanovdb_size_with_leaves.x = PNANOVDB_GRID_SIZE;
            velocityFloatNanoVdbParams.nanovdb_size_without_leaves.x = PNANOVDB_GRID_SIZE;
            velocityFloatNanoVdbBuffer = nullGridTransient;
            velocityFloatCacheBuffer = nullGridTransient;
        }

        NvFlowSparseNanoVdbParams velocityVec3NanoVdbParams = {};
        NvFlowBufferTransient* velocityVec3NanoVdbBuffer = nullptr;
        NvFlowBufferTransient* velocityVec3CacheBuffer = nullptr;
        if (anyVelocityEnabled)
        {
            ptr->sparseInterface.addPassesNanoVdb(
                in->context,
                in->sparse,
                PNANOVDB_GRID_TYPE_VEC3F,
                in->velocity.levelIdx,
                &velocityVec3NanoVdbParams,
                &velocityVec3NanoVdbBuffer,
                &velocityVec3CacheBuffer
            );
        }
        else
        {
            velocityVec3NanoVdbParams.nanovdb_size_with_leaves.x = PNANOVDB_GRID_SIZE;
            velocityVec3NanoVdbParams.nanovdb_size_without_leaves.x = PNANOVDB_GRID_SIZE;
            velocityVec3NanoVdbBuffer = nullGridTransient;
            velocityVec3CacheBuffer = nullGridTransient;
        }

        // ensure subGridDim is valid
        velocityVec3NanoVdbParams.subGridDimLessOne =
            uint3_max(velocityVec3NanoVdbParams.subGridDimLessOne, velocityFloatNanoVdbParams.subGridDimLessOne);
        velocityVec3NanoVdbParams.subGridDimBits =
            uint3_max(velocityVec3NanoVdbParams.subGridDimBits, velocityFloatNanoVdbParams.subGridDimBits);

        NvFlowUint64 temperatureSizeWithLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 fuelSizeWithLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 burnSizeWithLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 smokeSizeWithLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 velocitySizeWithLeaves = uint2_to_uint64(velocityVec3NanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 divergenceSizeWithLeaves = uint2_to_uint64(velocityFloatNanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 rgbaSizeWithLeaves = uint2_to_uint64(rgbaNanoVdbParams.nanovdb_size_with_leaves);
        NvFlowUint64 rgbSizeWithLeaves = uint2_to_uint64(rgbNanoVdbParams.nanovdb_size_with_leaves);

        NvFlowUint64 temperatureSizeWithoutLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 fuelSizeWithoutLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 burnSizeWithoutLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 smokeSizeWithoutLeaves = uint2_to_uint64(densityFloatNanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 velocitySizeWithoutLeaves = uint2_to_uint64(velocityVec3NanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 divergenceSizeWithoutLeaves = uint2_to_uint64(velocityFloatNanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 rgbaSizeWithoutLeaves = uint2_to_uint64(rgbaNanoVdbParams.nanovdb_size_without_leaves);
        NvFlowUint64 rgbSizeWithoutLeaves = uint2_to_uint64(rgbNanoVdbParams.nanovdb_size_without_leaves);

        // if channel not enabled, reduce to header only
        if (!anyTemperatureEnabled && temperatureSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            temperatureSizeWithLeaves = PNANOVDB_GRID_SIZE;
            temperatureSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anyFuelEnabled && fuelSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            fuelSizeWithLeaves = PNANOVDB_GRID_SIZE;
            fuelSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anyBurnEnabled && burnSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            burnSizeWithLeaves = PNANOVDB_GRID_SIZE;
            burnSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anySmokeEnabled && smokeSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            smokeSizeWithLeaves = PNANOVDB_GRID_SIZE;
            smokeSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anyVelocityEnabled && velocitySizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            velocitySizeWithLeaves = PNANOVDB_GRID_SIZE;
            velocitySizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anyDivergenceEnabled && divergenceSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            divergenceSizeWithLeaves = PNANOVDB_GRID_SIZE;
            divergenceSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anyRgbaEnabled && rgbaSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            rgbaSizeWithLeaves = PNANOVDB_GRID_SIZE;
            rgbaSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }
        if (!anyRgbEnabled && rgbSizeWithoutLeaves > PNANOVDB_GRID_SIZE)
        {
            rgbSizeWithLeaves = PNANOVDB_GRID_SIZE;
            rgbSizeWithoutLeaves = PNANOVDB_GRID_SIZE;
        }

        NvFlowDynamicBuffer_resize(in->context, &ptr->temperatureBuffer, temperatureSizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->fuelBuffer, fuelSizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->burnBuffer, burnSizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->smokeBuffer, smokeSizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->velocityBuffer, velocitySizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->divergenceBuffer, divergenceSizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->rgbaBuffer, rgbaSizeWithLeaves);
        NvFlowDynamicBuffer_resize(in->context, &ptr->rgbBuffer, rgbSizeWithLeaves);

        out->temperatureNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->temperatureBuffer);
        out->fuelNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->fuelBuffer);
        out->burnNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->burnBuffer);
        out->smokeNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->smokeBuffer);
        out->velocityNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->velocityBuffer);
        out->divergenceNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->divergenceBuffer);
        out->rgbaNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->rgbaBuffer);
        out->rgbNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->rgbBuffer);

        NvFlowPassCopyBufferParams copyParams = {};
        copyParams.src = densityFloatNanoVdbBuffer;

        copyParams.numBytes = temperatureSizeWithoutLeaves;
        copyParams.dst = out->temperatureNanoVdb;
        copyParams.debugLabel = "InitNanoVdbTemperature";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.numBytes = fuelSizeWithoutLeaves;
        copyParams.dst = out->fuelNanoVdb;
        copyParams.debugLabel = "InitNanoVdbFuel";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.numBytes = burnSizeWithoutLeaves;
        copyParams.dst = out->burnNanoVdb;
        copyParams.debugLabel = "InitNanoVdbBurn";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.numBytes = smokeSizeWithoutLeaves;
        copyParams.dst = out->smokeNanoVdb;
        copyParams.debugLabel = "InitNanoVdbSmoke";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.src = velocityVec3NanoVdbBuffer;

        copyParams.numBytes = velocitySizeWithoutLeaves;
        copyParams.dst = out->velocityNanoVdb;
        copyParams.debugLabel = "InitNanoVdbVelocity";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.src = velocityFloatNanoVdbBuffer;

        copyParams.numBytes = divergenceSizeWithoutLeaves;
        copyParams.dst = out->divergenceNanoVdb;
        copyParams.debugLabel = "InitNanoVdbDivergence";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.src = rgbaNanoVdbBuffer;

        copyParams.numBytes = rgbaSizeWithoutLeaves;
        copyParams.dst = out->rgbaNanoVdb;
        copyParams.debugLabel = "InitNanoVdbRgba";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        copyParams.src = rgbNanoVdbBuffer;

        copyParams.numBytes = rgbSizeWithoutLeaves;
        copyParams.dst = out->rgbNanoVdb;
        copyParams.debugLabel = "InitNanoVdbRgb";
        ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, densityLevelParams->numLocations);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mappedGlobal = (SparseNanoVdbExportParams*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(SparseNanoVdbExportParams));

            mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mappedGlobal->pad1 = 0u;
            mappedGlobal->pad2 = 0u;
            mappedGlobal->pad3 = 0u;
            mappedGlobal->tableVelocity = *velocityLevelParams;
            mappedGlobal->tableDensity = *densityLevelParams;
            mappedGlobal->velocityFloat = velocityFloatNanoVdbParams;
            mappedGlobal->velocityVec3 = velocityVec3NanoVdbParams;
            mappedGlobal->densityFloat = densityFloatNanoVdbParams;
            mappedGlobal->rgba = rgbaNanoVdbParams;
            mappedGlobal->rgb = rgbNanoVdbParams;
            mappedGlobal->temperatureEnabled = anyTemperatureEnabled;
            mappedGlobal->fuelEnabled = anyFuelEnabled;
            mappedGlobal->burnEnabled = anyBurnEnabled;
            mappedGlobal->smokeEnabled = anySmokeEnabled;
            mappedGlobal->velocityEnabled = anyVelocityEnabled;
            mappedGlobal->divergenceEnabled = anyDivergenceEnabled;
            mappedGlobal->rgbaEnabled = anyRgbaEnabled;
            mappedGlobal->rgbEnabled = anyRgbEnabled;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        // velocity
        if (anyVelocityEnabled || anyDivergenceEnabled)
        {
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                SparseNanoVdbExportVelocityCS_PassParams params = {};
                params.paramsIn = batches[batchIdx].globalTransient;
                params.tableIn = in->velocity.sparseBuffer;
                params.velocityIn = in->velocity.textureTransient;
                params.velocityOut = out->velocityNanoVdb;
                params.divergenceOut = out->divergenceNanoVdb;

                NvFlowUint3 gridDim = {};
                gridDim.x = 1u;
                gridDim.y = 1u << (velocityVec3NanoVdbParams.subGridDimBits.x + velocityVec3NanoVdbParams.subGridDimBits.y + velocityVec3NanoVdbParams.subGridDimBits.z);
                gridDim.z = batches[batchIdx].blockCount;

                SparseNanoVdbExportVelocityCS_addPassCompute(in->context, &ptr->sparseNanoVdbExportVelocityCS, gridDim, &params);
            }
        }

        // density
        if (anyTemperatureEnabled || anyFuelEnabled || anyBurnEnabled || anySmokeEnabled || anyRgbaEnabled || anyRgbEnabled)
        {
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                SparseNanoVdbExportDensityCS_PassParams params = {};
                params.paramsIn = batches[batchIdx].globalTransient;
                params.tableIn = in->density.sparseBuffer;
                params.densityIn = in->density.textureTransient;
                params.temperatureOut = out->temperatureNanoVdb;
                params.fuelOut = out->fuelNanoVdb;
                params.burnOut = out->burnNanoVdb;
                params.smokeOut = out->smokeNanoVdb;
                params.rgbaOut = out->rgbaNanoVdb;
                params.rgbOut = out->rgbNanoVdb;

                NvFlowUint3 gridDim = {};
                gridDim.x = 1u;
                gridDim.y = 1u << (densityFloatNanoVdbParams.subGridDimBits.x + densityFloatNanoVdbParams.subGridDimBits.y + densityFloatNanoVdbParams.subGridDimBits.z);
                gridDim.z = batches[batchIdx].blockCount;

                SparseNanoVdbExportDensityCS_addPassCompute(in->context, &ptr->sparseNanoVdbExportDensityCS, gridDim, &params);
            }
        }

        NvFlowBool32 anyStatisticsEnabled = NV_FLOW_FALSE;
        if (densityLevelParams->numLocations > 0u)
        {
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
            {
                auto layerParamsIn = in->params[layerParamIdx];
                if (layerParamsIn->statisticsEnabled)
                {
                    anyStatisticsEnabled = NV_FLOW_TRUE;
                }
            }
        }
        if (anyStatisticsEnabled)
        {
            if (anyTemperatureEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->temperatureNanoVdb
                );
            }
            if (anyFuelEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->fuelNanoVdb
                );
            }
            if (anyBurnEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->burnNanoVdb
                );
            }
            if (anySmokeEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->smokeNanoVdb
                );
            }
            if (anyVelocityEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &velocityVec3NanoVdbParams,
                    velocityVec3NanoVdbBuffer,
                    velocityVec3CacheBuffer,
                    out->velocityNanoVdb
                );
            }
            if (anyDivergenceEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &velocityFloatNanoVdbParams,
                    velocityFloatNanoVdbBuffer,
                    velocityFloatCacheBuffer,
                    out->divergenceNanoVdb
                );
            }
            if (anyRgbaEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &rgbaNanoVdbParams,
                    rgbaNanoVdbBuffer,
                    rgbaCacheBuffer,
                    out->rgbaNanoVdb
                );
            }
            if (anyRgbEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbComputeStats(
                    in->context,
                    in->sparse,
                    &rgbNanoVdbParams,
                    rgbNanoVdbBuffer,
                    rgbCacheBuffer,
                    out->rgbNanoVdb
                );
            }
        }

        // vote on readback/interop behavior
        NvFlowBool32 anyInteropEnabled = NV_FLOW_FALSE;
        if (densityLevelParams->numLocations > 0u)
        {
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
            {
                auto layerParamsIn = in->params[layerParamIdx];
                if (layerParamsIn->interopEnabled)
                {
                    anyInteropEnabled = NV_FLOW_TRUE;
                }
            }
        }
        NvFlowBool32 anyReadbackEnabled = NV_FLOW_FALSE;
        if (densityLevelParams->numLocations > 0u)
        {
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
            {
                auto layerParamsIn = in->params[layerParamIdx];
                if (layerParamsIn->readbackEnabled)
                {
                    anyReadbackEnabled = NV_FLOW_TRUE;
                }
            }
        }

        // generate lite copies
        NvFlowBufferTransient* temperatureLiteNanoVdb = nullptr;
        NvFlowBufferTransient* fuelLiteNanoVdb = nullptr;
        NvFlowBufferTransient* burnLiteNanoVdb = nullptr;
        NvFlowBufferTransient* smokeLiteNanoVdb = nullptr;
        NvFlowBufferTransient* velocityLiteNanoVdb = nullptr;
        NvFlowBufferTransient* divergenceLiteNanoVdb = nullptr;
        NvFlowBufferTransient* rgbaLiteNanoVdb = nullptr;
        NvFlowBufferTransient* rgbLiteNanoVdb = nullptr;
        if (!anyReadbackEnabled && anyInteropEnabled)
        {
            NvFlowDynamicBuffer_resize(in->context, &ptr->temperatureLiteBuffer, temperatureSizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->fuelLiteBuffer, fuelSizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->burnLiteBuffer, burnSizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->smokeLiteBuffer, smokeSizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->velocityLiteBuffer, velocitySizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->divergenceLiteBuffer, divergenceSizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->rgbaLiteBuffer, rgbaSizeWithoutLeaves);
            NvFlowDynamicBuffer_resize(in->context, &ptr->rgbLiteBuffer, rgbSizeWithoutLeaves);

            temperatureLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->temperatureLiteBuffer);
            fuelLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->fuelLiteBuffer);
            burnLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->burnLiteBuffer);
            smokeLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->smokeLiteBuffer);
            velocityLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->velocityLiteBuffer);
            divergenceLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->divergenceLiteBuffer);
            rgbaLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->rgbaLiteBuffer);
            rgbLiteNanoVdb = NvFlowDynamicBuffer_getTransient(in->context, &ptr->rgbLiteBuffer);

            NvFlowPassCopyBufferParams copyParams = {};

            copyParams.numBytes = temperatureSizeWithoutLeaves;
            copyParams.src = out->temperatureNanoVdb;;
            copyParams.dst = temperatureLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbTemperature";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = fuelSizeWithoutLeaves;
            copyParams.src = out->fuelNanoVdb;
            copyParams.dst = fuelLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbFuel";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = burnSizeWithoutLeaves;
            copyParams.src = out->burnNanoVdb;
            copyParams.dst = burnLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbBurn";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = smokeSizeWithoutLeaves;
            copyParams.src = out->smokeNanoVdb;
            copyParams.dst = smokeLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbSmoke";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = velocitySizeWithoutLeaves;
            copyParams.src = out->velocityNanoVdb;
            copyParams.dst = velocityLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbVelocity";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = divergenceSizeWithoutLeaves;
            copyParams.src = out->divergenceNanoVdb;
            copyParams.dst = divergenceLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbDivergence";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = rgbaSizeWithoutLeaves;
            copyParams.src = out->rgbaNanoVdb;
            copyParams.dst = rgbaLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbRgba";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            copyParams.numBytes = rgbSizeWithoutLeaves;
            copyParams.src = out->rgbNanoVdb;
            copyParams.dst = rgbLiteNanoVdb;
            copyParams.debugLabel = "CopyLiteNanoVdbRgb";
            ptr->contextInterface.addPassCopyBuffer(in->context, &copyParams);

            if (anyTemperatureEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->temperatureNanoVdb,
                    temperatureLiteNanoVdb
                );
            }
            if (anyFuelEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->fuelNanoVdb,
                    fuelLiteNanoVdb
                );
            }
            if (anyBurnEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->burnNanoVdb,
                    burnLiteNanoVdb
                );
            }
            if (anySmokeEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &densityFloatNanoVdbParams,
                    densityFloatNanoVdbBuffer,
                    densityFloatCacheBuffer,
                    out->smokeNanoVdb,
                    smokeLiteNanoVdb
                );
            }
            if (anyVelocityEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &velocityVec3NanoVdbParams,
                    velocityVec3NanoVdbBuffer,
                    velocityVec3CacheBuffer,
                    out->velocityNanoVdb,
                    velocityLiteNanoVdb
                );
            }
            if (anyDivergenceEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &velocityFloatNanoVdbParams,
                    velocityFloatNanoVdbBuffer,
                    velocityFloatCacheBuffer,
                    out->divergenceNanoVdb,
                    divergenceLiteNanoVdb
                );
            }
            if (anyRgbaEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &rgbaNanoVdbParams,
                    rgbaNanoVdbBuffer,
                    rgbaCacheBuffer,
                    out->rgbaNanoVdb,
                    rgbaLiteNanoVdb
                );
            }
            if (anyRgbEnabled)
            {
                ptr->sparseInterface.addPassesNanoVdbPruneLeaves(
                    in->context,
                    in->sparse,
                    &rgbNanoVdbParams,
                    rgbNanoVdbBuffer,
                    rgbCacheBuffer,
                    out->rgbNanoVdb,
                    rgbLiteNanoVdb
                );
            }
        }

        if (anyReadbackEnabled || anyInteropEnabled)
        {
            bool lite = !anyReadbackEnabled && anyInteropEnabled;

            NvFlowUint64 temperatureCopyVersion = 0llu;
            NvFlowUint64 fuelCopyVersion = 0llu;
            NvFlowUint64 burnCopyVersion = 0llu;
            NvFlowUint64 smokeCopyVersion = 0llu;
            NvFlowUint64 velocityCopyVersion = 0llu;
            NvFlowUint64 divergenceCopyVersion = 0llu;
            NvFlowUint64 rgbaCopyVersion = 0llu;
            NvFlowUint64 rgbCopyVersion = 0llu;

            NvFlowReadbackBuffer_setRingBufferCount(&ptr->temperatureReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->fuelReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->burnReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->smokeReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->velocityReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->divergenceReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->rgbaReadback, in->readbackRingBufferCount);
            NvFlowReadbackBuffer_setRingBufferCount(&ptr->rgbReadback, in->readbackRingBufferCount);

            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->temperatureReadback,
                lite ? temperatureSizeWithoutLeaves : temperatureSizeWithLeaves,
                lite ? temperatureLiteNanoVdb : out->temperatureNanoVdb,
                &temperatureCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->fuelReadback,
                lite ? fuelSizeWithoutLeaves : fuelSizeWithLeaves,
                lite ? fuelLiteNanoVdb : out->fuelNanoVdb,
                &fuelCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->burnReadback,
                lite ? burnSizeWithoutLeaves : burnSizeWithLeaves,
                lite ? burnLiteNanoVdb : out->burnNanoVdb,
                &burnCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->smokeReadback,
                lite ? smokeSizeWithoutLeaves : smokeSizeWithLeaves,
                lite ? smokeLiteNanoVdb : out->smokeNanoVdb,
                &smokeCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->velocityReadback,
                lite ? velocitySizeWithoutLeaves : velocitySizeWithLeaves,
                lite ? velocityLiteNanoVdb : out->velocityNanoVdb,
                &velocityCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->divergenceReadback,
                lite ? divergenceSizeWithoutLeaves : divergenceSizeWithLeaves,
                lite ? divergenceLiteNanoVdb : out->divergenceNanoVdb,
                &divergenceCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->rgbaReadback,
                lite ? rgbaSizeWithoutLeaves : rgbaSizeWithLeaves,
                lite ? rgbaLiteNanoVdb : out->rgbaNanoVdb,
                &rgbaCopyVersion);
            NvFlowReadbackBuffer_copy(
                in->context,
                &ptr->rgbReadback,
                lite ? rgbSizeWithoutLeaves : rgbSizeWithLeaves,
                lite ? rgbLiteNanoVdb : out->rgbNanoVdb,
                &rgbCopyVersion);

            NvFlowUint64 temperatureMappedVersion = 0llu;
            NvFlowUint64 fuelMappedVersion = 0llu;
            NvFlowUint64 burnMappedVersion = 0llu;
            NvFlowUint64 smokeMappedVersion = 0llu;
            NvFlowUint64 velocityMappedVersion = 0llu;
            NvFlowUint64 divergenceMappedVersion = 0llu;
            NvFlowUint64 rgbaMappedVersion = 0llu;
            NvFlowUint64 rgbMappedVersion = 0llu;

            // unmap old
            SparseNanoVdbExport_unmap(ptr, in, out);

            NvFlowReadbackBuffer_flush(in->context, &ptr->temperatureReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->fuelReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->burnReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->smokeReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->velocityReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->divergenceReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->rgbaReadback);
            NvFlowReadbackBuffer_flush(in->context, &ptr->rgbReadback);

            NvFlowUint activeCount = NvFlowReadbackBuffer_getActiveCount(in->context, &ptr->smokeReadback);

            ptr->readbacks.size = 0u;
            ptr->readbacks.reserve(activeCount);
            ptr->readbacks.size = activeCount;

            out->readbackCount = ptr->readbacks.size;
            out->readbacks = ptr->readbacks.data;

            for (NvFlowUint activeIdx = 0u; activeIdx < activeCount; activeIdx++)
            {
                auto readbackMapBuffer = [](
                    NvFlowContext* context, NvFlowUint activeIdx, NvFlowReadbackBuffer* buffer,
                    NvFlowUint** pReadback, NvFlowUint64* pReadbackCount
                ){
                    *pReadback = (NvFlowUint*)NvFlowReadbackBuffer_map(context, buffer, activeIdx, nullptr, pReadbackCount);
                    // convert bytes to words
                    *pReadbackCount /= sizeof(NvFlowUint);
                    // zero out versions that are header only
                    if (*pReadbackCount * sizeof(NvFlowUint) <= PNANOVDB_GRID_SIZE)
                    {
                        *pReadback = nullptr;
                        *pReadbackCount = 0llu;
                    }
                };

                NvFlowSparseNanoVdbExportReadback& activeReadback = out->readbacks[activeIdx];

                readbackMapBuffer(in->context, activeIdx,
                    &ptr->temperatureReadback,
                    &activeReadback.temperatureNanoVdbReadback,
                    &activeReadback.temperatureNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->fuelReadback,
                    &activeReadback.fuelNanoVdbReadback,
                    &activeReadback.fuelNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->burnReadback,
                    &activeReadback.burnNanoVdbReadback,
                    &activeReadback.burnNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->smokeReadback,
                    &activeReadback.smokeNanoVdbReadback,
                    &activeReadback.smokeNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->velocityReadback,
                    &activeReadback.velocityNanoVdbReadback,
                    &activeReadback.velocityNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->divergenceReadback,
                    &activeReadback.divergenceNanoVdbReadback,
                    &activeReadback.divergenceNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->rgbaReadback,
                    &activeReadback.rgbaNanoVdbReadback,
                    &activeReadback.rgbaNanoVdbReadbackCount);
                readbackMapBuffer(in->context, activeIdx,
                    &ptr->rgbReadback,
                    &activeReadback.rgbNanoVdbReadback,
                    &activeReadback.rgbNanoVdbReadbackCount);

                // indicate version with smoke
                out->readbacks[activeIdx].globalFrameCompleted = NvFlowReadbackBuffer_getCompletedGlobalFrame(in->context, &ptr->smokeReadback, activeIdx);
                out->readbacks[activeIdx].absoluteSimTimeCompleted = SparseNanoVdbExport_historyAbsoluteSimTime(ptr, out->readbacks[activeIdx].globalFrameCompleted);
            }
        }

        if (anyInteropEnabled)
        {
            NvFlowUint64 temperatureCopyVersion = 0llu;
            NvFlowUint64 fuelCopyVersion = 0llu;
            NvFlowUint64 burnCopyVersion = 0llu;
            NvFlowUint64 smokeCopyVersion = 0llu;
            NvFlowUint64 velocityCopyVersion = 0llu;
            NvFlowUint64 divergenceCopyVersion = 0llu;
            NvFlowUint64 rgbaCopyVersion = 0llu;
            NvFlowUint64 rgbCopyVersion = 0llu;

            NvFlowInteropBuffer_setRingBufferCount(&ptr->temperatureInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->fuelInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->burnInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->smokeInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->velocityInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->divergenceInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->rgbaInterop, in->interopRingBufferCount);
            NvFlowInteropBuffer_setRingBufferCount(&ptr->rgbInterop, in->interopRingBufferCount);

            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->temperatureInterop,
                temperatureSizeWithLeaves,
                out->temperatureNanoVdb,
                &temperatureCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->fuelInterop,
                fuelSizeWithLeaves,
                out->fuelNanoVdb,
                &fuelCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->burnInterop,
                burnSizeWithLeaves,
                out->burnNanoVdb,
                &burnCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->smokeInterop,
                smokeSizeWithLeaves,
                out->smokeNanoVdb,
                &smokeCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->velocityInterop,
                velocitySizeWithLeaves,
                out->velocityNanoVdb,
                &velocityCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->divergenceInterop,
                divergenceSizeWithLeaves,
                out->divergenceNanoVdb,
                &divergenceCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->rgbaInterop,
                rgbaSizeWithLeaves,
                out->rgbaNanoVdb,
                &rgbaCopyVersion);
            NvFlowInteropBuffer_copy(
                in->context,
                &ptr->rgbInterop,
                rgbSizeWithLeaves,
                out->rgbNanoVdb,
                &rgbCopyVersion);

            NvFlowUint64 temperatureMappedVersion = 0llu;
            NvFlowUint64 fuelMappedVersion = 0llu;
            NvFlowUint64 burnMappedVersion = 0llu;
            NvFlowUint64 smokeMappedVersion = 0llu;
            NvFlowUint64 velocityMappedVersion = 0llu;
            NvFlowUint64 divergenceMappedVersion = 0llu;
            NvFlowUint64 rgbaMappedVersion = 0llu;
            NvFlowUint64 rgbMappedVersion = 0llu;

            NvFlowInteropBuffer_flush(in->context, &ptr->temperatureInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->fuelInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->burnInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->smokeInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->velocityInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->divergenceInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->rgbaInterop);
            NvFlowInteropBuffer_flush(in->context, &ptr->rgbInterop);

            NvFlowUint activeCount = NvFlowInteropBuffer_getActiveCount(in->context, &ptr->smokeInterop);

            ptr->interops.size = 0u;
            ptr->interops.reserve(activeCount);
            ptr->interops.size = activeCount;

            out->interopCount = ptr->interops.size;
            out->interops = ptr->interops.data;

            for (NvFlowUint activeIdx = 0u; activeIdx < activeCount; activeIdx++)
            {
                NvFlowSparseNanoVdbExportInterop& activeInterop = out->interops[activeIdx];

                activeInterop.temperatureInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->temperatureInterop, activeIdx, nullptr);
                activeInterop.fuelInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->fuelInterop, activeIdx, nullptr);
                activeInterop.burnInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->burnInterop, activeIdx, nullptr);
                activeInterop.smokeInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->smokeInterop, activeIdx, nullptr);
                activeInterop.velocityInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->velocityInterop, activeIdx, nullptr);
                activeInterop.divergenceInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->divergenceInterop, activeIdx, nullptr);
                activeInterop.rgbaInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->rgbaInterop, activeIdx, nullptr);
                activeInterop.rgbInteropHandle =
                    NvFlowInteropBuffer_map(in->context, &ptr->rgbInterop, activeIdx, nullptr);

                // indicate version with smoke
                out->interops[activeIdx].globalFrameCompleted = NvFlowInteropBuffer_getCompletedGlobalFrame(in->context, &ptr->smokeInterop, activeIdx);
                out->interops[activeIdx].absoluteSimTimeCompleted = SparseNanoVdbExport_historyAbsoluteSimTime(ptr, out->interops[activeIdx].globalFrameCompleted);
            }
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowSparseNanoVdbExport, SparseNanoVdbExport)
