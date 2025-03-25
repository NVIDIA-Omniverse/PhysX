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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include "NvFlowLoader.h"

struct FlowStandaloneInstance;
struct FlowStandaloneBuffer;
struct FlowStandaloneSemaphore;

struct FlowStandaloneOutput
{
    FlowStandaloneBuffer* temperatureNanoVdb;
    FlowStandaloneBuffer* fuelNanoVdb;
    FlowStandaloneBuffer* burnNanoVdb;
    FlowStandaloneBuffer* smokeNanoVdb;
    FlowStandaloneBuffer* velocityNanoVdb;
    FlowStandaloneBuffer* divergenceNanoVdb;

    uint64_t currentFrame;
    uint64_t readbackFrame;
    uint8_t* temperatureNanoVdbReadback;
    uint64_t temperatureNanoVdbReadbackSize;
    uint8_t* fuelNanoVdbReadback;
    uint64_t fuelNanoVdbReadbackSize;
    uint8_t* burnNanoVdbReadback;
    uint64_t burnNanoVdbReadbackSize;
    uint8_t* smokeNanoVdbReadback;
    uint64_t smokeNanoVdbReadbackSize;
    uint8_t* velocityNanoVdbReadback;
    uint64_t velocityNanoVdbReadbackSize;
    uint8_t* divergenceNanoVdbReadback;
    uint64_t divergenceNanoVdbReadbackSize;
};

struct FlowStandaloneInstance
{
    NvFlowLoader loader = {};
    NvFlowContextInterface contextInterface = {};

    NvFlowDeviceManager* deviceManager = nullptr;
    NvFlowDevice* device = nullptr;
    NvFlowDeviceQueue* deviceQueue = nullptr;

    NvFlowGrid* grid = nullptr;
    NvFlowGridParamsNamed* gridParamsNamed = nullptr;

    NvFlowBuffer* temperatureNanoVdb = nullptr;
    NvFlowBuffer* fuelNanoVdb = nullptr;
    NvFlowBuffer* burnNanoVdb = nullptr;
    NvFlowBuffer* smokeNanoVdb = nullptr;
    NvFlowBuffer* velocityNanoVdb = nullptr;
    NvFlowBuffer* divergenceNanoVdb = nullptr;

    bool invalid = false;
};

static void flowLoaderError(const char* str, void* userdata)
{
    printf("omni.flow.usd failed to load Flow library!!!\n%s\n", str);
}

static void logPrint(NvFlowLogLevel level, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buf[256u];
    buf[0u] = '\0';

    const char* prefix = "Unknown";
    if (level == eNvFlowLogLevel_error)
    {
        vsnprintf(buf, 256u, format, args);
        printf("FlowError: %s\n", buf);
    }
    else if (level == eNvFlowLogLevel_warning)
    {
        vsnprintf(buf, 256u, format, args);
        printf("FlowWarn: %s\n", buf);
    }
    else if (level == eNvFlowLogLevel_info)
    {
        vsnprintf(buf, 256u, format, args);
        printf("FlowInfo: %s\n", buf);
    }

    va_end(args);
}

bool flowStandaloneInitInstance(FlowStandaloneInstance* ptr, uint32_t deviceIdx, bool cudaInteropEnabled, uint32_t maxBlocks)
{
    NvFlowLoaderInit(&ptr->loader, flowLoaderError, nullptr);

    if (!ptr->loader.module_nvflow || !ptr->loader.module_nvflowext)
    {
        printf("FlowStandaloneInstance init() failed!!!\n");
        return false;
    }

    // initialize graphics
    {
        NvFlowBool32 validation = NV_FLOW_TRUE;

        ptr->deviceManager = ptr->loader.deviceInterface.createDeviceManager(validation, nullptr, 0u);

        NvFlowDeviceDesc deviceDesc = {};
        deviceDesc.deviceIndex = deviceIdx;
        deviceDesc.enableExternalUsage = cudaInteropEnabled;
        deviceDesc.logPrint = logPrint;

        ptr->device = ptr->loader.deviceInterface.createDevice(ptr->deviceManager, &deviceDesc);

        ptr->deviceQueue = ptr->loader.deviceInterface.getDeviceQueue(ptr->device);

        NvFlowContextInterface_duplicate(&ptr->contextInterface, ptr->loader.deviceInterface.getContextInterface(ptr->deviceQueue));
    }

    NvFlowContext* context = ptr->loader.deviceInterface.getContext(ptr->deviceQueue);

    // initialize grid
    {
        NvFlowGridDesc gridDesc = NvFlowGridDesc_default;

        if (maxBlocks > 0u)
        {
            gridDesc.maxLocations = maxBlocks;
        }

        ptr->grid = ptr->loader.gridInterface.createGrid(&ptr->contextInterface, context, ptr->loader.opList_orig, ptr->loader.extOpList_orig, &gridDesc);
        ptr->gridParamsNamed = ptr->loader.gridParamsInterface.createGridParamsNamed("flowUsd");
    }

    return true;
}

FlowStandaloneInstance* flowStandaloneCreateInstance(uint32_t deviceIdx, bool cudaInteropEnabled, uint32_t maxBlocks)
{
    auto ptr = new FlowStandaloneInstance();

    if (!flowStandaloneInitInstance(ptr, deviceIdx, cudaInteropEnabled, maxBlocks))
    {
        delete ptr;
        return nullptr;
    }

    return ptr;
}

void flowStandaloneDestroyBuffers(FlowStandaloneInstance* ptr)
{
    NvFlowContext* context = ptr->loader.deviceInterface.getContext(ptr->deviceQueue);

    // release old acquires
    if (ptr->temperatureNanoVdb)
    {
        ptr->contextInterface.destroyBuffer(context, ptr->temperatureNanoVdb);
        ptr->temperatureNanoVdb = nullptr;
    }
    if (ptr->fuelNanoVdb)
    {
        ptr->contextInterface.destroyBuffer(context, ptr->fuelNanoVdb);
        ptr->fuelNanoVdb = nullptr;
    }
    if (ptr->burnNanoVdb)
    {
        ptr->contextInterface.destroyBuffer(context, ptr->burnNanoVdb);
        ptr->burnNanoVdb = nullptr;
    }
    if (ptr->smokeNanoVdb)
    {
        ptr->contextInterface.destroyBuffer(context, ptr->smokeNanoVdb);
        ptr->smokeNanoVdb = nullptr;
    }
    if (ptr->velocityNanoVdb)
    {
        ptr->contextInterface.destroyBuffer(context, ptr->velocityNanoVdb);
        ptr->velocityNanoVdb = nullptr;
    }
    if (ptr->velocityNanoVdb)
    {
        ptr->contextInterface.destroyBuffer(context, ptr->velocityNanoVdb);
        ptr->velocityNanoVdb = nullptr;
    }
}

void flowStandaloneDestroyInstance(FlowStandaloneInstance* ptr)
{
    // wait idle
    {
        ptr->loader.deviceInterface.waitIdle(ptr->deviceQueue);
    }

    flowStandaloneDestroyBuffers(ptr);

    // destroy grid
    {
        NvFlowContext* context = ptr->loader.deviceInterface.getContext(ptr->deviceQueue);

        ptr->loader.gridInterface.destroyGrid(context, ptr->grid);
        ptr->loader.gridParamsInterface.destroyGridParamsNamed(ptr->gridParamsNamed);
    }

    // wait idle
    {
        NvFlowUint64 flushedFrameID = 0u;
        ptr->loader.deviceInterface.flush(ptr->deviceQueue, &flushedFrameID, nullptr, nullptr);

        ptr->loader.deviceInterface.waitIdle(ptr->deviceQueue);
    }

    // destroy graphics
    {
        ptr->loader.deviceInterface.destroyDevice(ptr->deviceManager, ptr->device);
        ptr->loader.deviceInterface.destroyDeviceManager(ptr->deviceManager);
    }

    NvFlowLoaderDestroy(&ptr->loader);

    delete ptr;
}

void flowStandaloneUpdateInstance(FlowStandaloneInstance* ptr, double absoluteSimTime, FlowStandaloneOutput* pOutput, bool cpuWait, FlowStandaloneSemaphore* waitSemaphore, FlowStandaloneSemaphore* signalSemaphore)
{
    FlowStandaloneOutput output = {};
    NvFlowGridRenderData renderData = {};

    if (ptr->invalid)
    {
        if (pOutput)
        {
            *pOutput = output;
        }
        return;
    }

    NvFlowBufferAcquire* temperatureNanoVdbAcquire = nullptr;
    NvFlowBufferAcquire* fuelNanoVdbAcquire = nullptr;
    NvFlowBufferAcquire* burnNanoVdbAcquire = nullptr;
    NvFlowBufferAcquire* smokeNanoVdbAcquire = nullptr;
    NvFlowBufferAcquire* velocityNanoVdbAcquire = nullptr;
    NvFlowBufferAcquire* divergenceNanoVdbAcquire = nullptr;

    NvFlowContext* context = ptr->loader.deviceInterface.getContext(ptr->deviceQueue);

    NvFlowGridParams* gridParams = ptr->loader.gridParamsInterface.mapGridParamsNamed(ptr->gridParamsNamed);

    NvFlowGridParamsDesc gridParamsDesc = {};
    NvFlowGridParamsSnapshot* paramsSnapshot = ptr->loader.gridParamsInterface.getParamsSnapshot(gridParams, absoluteSimTime, 0llu);
    if (ptr->loader.gridParamsInterface.mapParamsDesc(gridParams, paramsSnapshot, &gridParamsDesc))
    {
        ptr->loader.gridInterface.simulate(
            context,
            ptr->grid,
            &gridParamsDesc,
            NV_FLOW_FALSE
        );

        //ptr->loader.gridInterface.offscreen(context, ptr->grid, &gridParamsDesc);

        ptr->loader.gridInterface.getRenderData(context, ptr->grid, &renderData);

        if (renderData.nanoVdb.temperatureNanoVdb)
        {
            temperatureNanoVdbAcquire = ptr->contextInterface.enqueueAcquireBuffer(context, renderData.nanoVdb.temperatureNanoVdb);
        }
        if (renderData.nanoVdb.fuelNanoVdb)
        {
            fuelNanoVdbAcquire = ptr->contextInterface.enqueueAcquireBuffer(context, renderData.nanoVdb.fuelNanoVdb);
        }
        if (renderData.nanoVdb.burnNanoVdb)
        {
            burnNanoVdbAcquire = ptr->contextInterface.enqueueAcquireBuffer(context, renderData.nanoVdb.burnNanoVdb);
        }
        if (renderData.nanoVdb.smokeNanoVdb)
        {
            smokeNanoVdbAcquire = ptr->contextInterface.enqueueAcquireBuffer(context, renderData.nanoVdb.smokeNanoVdb);
        }
        if (renderData.nanoVdb.velocityNanoVdb)
        {
            velocityNanoVdbAcquire = ptr->contextInterface.enqueueAcquireBuffer(context, renderData.nanoVdb.velocityNanoVdb);
        }
        if (renderData.nanoVdb.divergenceNanoVdb)
        {
            divergenceNanoVdbAcquire = ptr->contextInterface.enqueueAcquireBuffer(context, renderData.nanoVdb.divergenceNanoVdb);
        }

        ptr->loader.gridParamsInterface.unmapParamsDesc(gridParams, paramsSnapshot);
    }

    NvFlowUint64 flushedFrameID = 0llu;
    int deviceReset = ptr->loader.deviceInterface.flush(ptr->deviceQueue, &flushedFrameID, (NvFlowDeviceSemaphore*)waitSemaphore, (NvFlowDeviceSemaphore*)signalSemaphore);
    if (deviceReset)
    {
        printf("FlowStandalone device reset!\n");
        ptr->invalid = true;
    }

    if (cpuWait)
    {
        ptr->loader.deviceInterface.waitForFrame(ptr->deviceQueue, flushedFrameID);
    }

    NvFlowUint64 lastCompletedFrame = ptr->contextInterface.getLastFrameCompleted(context);

    flowStandaloneDestroyBuffers(ptr);

    if (temperatureNanoVdbAcquire)
    {
        if (!ptr->contextInterface.getAcquiredBuffer(context, temperatureNanoVdbAcquire, &ptr->temperatureNanoVdb))
        {
            printf("Failed to acquire temperature buffer!!!\n");
        }
    }
    if (fuelNanoVdbAcquire)
    {
        if (!ptr->contextInterface.getAcquiredBuffer(context, fuelNanoVdbAcquire, &ptr->fuelNanoVdb))
        {
            printf("Failed to acquire fuel buffer!!!\n");
        }
    }
    if (burnNanoVdbAcquire)
    {
        if (!ptr->contextInterface.getAcquiredBuffer(context, burnNanoVdbAcquire, &ptr->burnNanoVdb))
        {
            printf("Failed to acquire burn buffer!!!\n");
        }
    }
    if (smokeNanoVdbAcquire)
    {
        if (!ptr->contextInterface.getAcquiredBuffer(context, smokeNanoVdbAcquire, &ptr->smokeNanoVdb))
        {
            printf("Failed to acquire smoke buffer!!!\n");
        }
    }
    if (velocityNanoVdbAcquire)
    {
        if (!ptr->contextInterface.getAcquiredBuffer(context, velocityNanoVdbAcquire, &ptr->velocityNanoVdb))
        {
            printf("Failed to acquire velocity buffer!!!\n");
        }
    }
    if (divergenceNanoVdbAcquire)
    {
        if (!ptr->contextInterface.getAcquiredBuffer(context, divergenceNanoVdbAcquire, &ptr->divergenceNanoVdb))
        {
            printf("Failed to acquire divergence buffer!!!\n");
        }
    }

    output.temperatureNanoVdb = (FlowStandaloneBuffer*)ptr->temperatureNanoVdb;
    output.fuelNanoVdb = (FlowStandaloneBuffer*)ptr->fuelNanoVdb;
    output.burnNanoVdb = (FlowStandaloneBuffer*)ptr->burnNanoVdb;
    output.smokeNanoVdb = (FlowStandaloneBuffer*)ptr->smokeNanoVdb;
    output.velocityNanoVdb = (FlowStandaloneBuffer*)ptr->velocityNanoVdb;
    output.divergenceNanoVdb = (FlowStandaloneBuffer*)ptr->divergenceNanoVdb;

    // pick latest readback version safe to access
    if (renderData.nanoVdb.readbackCount > 0u)
    {
        for (NvFlowUint64 idx = renderData.nanoVdb.readbackCount - 1u; idx < renderData.nanoVdb.readbackCount; idx--)
        {
            const auto readback = renderData.nanoVdb.readbacks + idx;
            if (lastCompletedFrame >= readback->globalFrameCompleted)
            {
                output.currentFrame = flushedFrameID;
                output.readbackFrame = lastCompletedFrame;
                output.temperatureNanoVdbReadback = readback->temperatureNanoVdbReadback;
                output.temperatureNanoVdbReadbackSize = readback->temperatureNanoVdbReadbackSize;
                output.fuelNanoVdbReadback = readback->fuelNanoVdbReadback;
                output.fuelNanoVdbReadbackSize = readback->fuelNanoVdbReadbackSize;
                output.burnNanoVdbReadback = readback->burnNanoVdbReadback;
                output.burnNanoVdbReadbackSize = readback->burnNanoVdbReadbackSize;
                output.smokeNanoVdbReadback = readback->smokeNanoVdbReadback;
                output.smokeNanoVdbReadbackSize = readback->smokeNanoVdbReadbackSize;
                output.velocityNanoVdbReadback = readback->velocityNanoVdbReadback;
                output.velocityNanoVdbReadbackSize = readback->velocityNanoVdbReadbackSize;
                output.divergenceNanoVdbReadback = readback->divergenceNanoVdbReadback;
                output.divergenceNanoVdbReadbackSize = readback->divergenceNanoVdbReadbackSize;

                break;
            }
        }
    }

    if (pOutput)
    {
        *pOutput = output;
    }
}

void flowStandaloneGetBufferExternalHandle(FlowStandaloneInstance* ptr, FlowStandaloneBuffer* buffer, void* dstHandle, uint64_t dstHandleSize, NvFlowUint64* pBufferSizeInBytes)
{
    auto bufferFlow = (NvFlowBuffer*)buffer;

    NvFlowContext* context = ptr->loader.deviceInterface.getContext(ptr->deviceQueue);

    ptr->loader.deviceInterface.getBufferExternalHandle(context, bufferFlow, dstHandle, dstHandleSize, pBufferSizeInBytes);
}

void flowStandaloneCloseBufferExternalHandle(FlowStandaloneInstance* ptr, FlowStandaloneBuffer* buffer, const void* srcHandle, uint64_t srcHandleSize)
{
    auto bufferFlow = (NvFlowBuffer*)buffer;

    NvFlowContext* context = ptr->loader.deviceInterface.getContext(ptr->deviceQueue);

    ptr->loader.deviceInterface.closeBufferExternalHandle(context, bufferFlow, srcHandle, srcHandleSize);
}

FlowStandaloneSemaphore* flowStandaloneCreateSemaphore(FlowStandaloneInstance* ptr)
{
    return (FlowStandaloneSemaphore*)ptr->loader.deviceInterface.createSemaphore(ptr->device);
}

void flowStandaloneDestroySemaphore(FlowStandaloneInstance* ptr, FlowStandaloneSemaphore* semaphore)
{
    auto semaphoreFlow = (NvFlowDeviceSemaphore*)semaphore;

    ptr->loader.deviceInterface.destroySemaphore(semaphoreFlow);
}

void flowStandaloneGetSemaphoreExternalHandle(FlowStandaloneInstance* ptr, FlowStandaloneSemaphore* semaphore, void* dstHandle, uint64_t dstHandleSize)
{
    auto semaphoreFlow = (NvFlowDeviceSemaphore*)semaphore;

    ptr->loader.deviceInterface.getSemaphoreExternalHandle(semaphoreFlow, dstHandle, dstHandleSize);
}

void flowStandaloneCloseSemaphoreExternalHandle(FlowStandaloneInstance* ptr, FlowStandaloneSemaphore* semaphore, const void* srcHandle, uint64_t srcHandleSize)
{
    auto semaphoreFlow = (NvFlowDeviceSemaphore*)semaphore;

    ptr->loader.deviceInterface.closeSemaphoreExternalHandle(semaphoreFlow, srcHandle, srcHandleSize);
}

/// Test

int testStandaloneMode()
{
    FlowStandaloneInstance* ptr = flowStandaloneCreateInstance(0u, false, 0u);

    NvFlowGridParamsNamed* paramSrcNamed = ptr->loader.gridParamsInterface.createGridParamsNamed("flowUsd");
    NvFlowGridParams* paramSrc = ptr->loader.gridParamsInterface.mapGridParamsNamed(paramSrcNamed);

    for (uint32_t idx = 0u; idx < 500; idx++)
    {
        static NvFlowGridSimulateLayerParams testSimulate = NvFlowGridSimulateLayerParams_default;
        static NvFlowGridEmitterSphereParams testSpheres = NvFlowEmitterSphereParams_default;
        static NvFlowGridOffscreenLayerParams testOffscreen = NvFlowGridOffscreenLayerParams_default;
        static NvFlowGridRenderLayerParams testRender = NvFlowGridRenderLayerParams_default;

        testSimulate.nanoVdbExport.enabled = NV_FLOW_TRUE;
        testSimulate.nanoVdbExport.readbackEnabled = NV_FLOW_TRUE;

        static NvFlowGridSimulateLayerParams* pTestSimulate = &testSimulate;
        static NvFlowGridEmitterSphereParams* pTestSpheres = &testSpheres;
        static NvFlowGridOffscreenLayerParams* pTestOffscreen = &testOffscreen;
        static NvFlowGridRenderLayerParams* pTestRender = &testRender;

        static NvFlowUint64 version = 1u;

        static NvFlowDatabaseTypeSnapshot typeSnapshots[4u] = {
            {version, &NvFlowGridSimulateLayerParams_NvFlowReflectDataType,  (NvFlowUint8**)&pTestSimulate,  1u},
            {version, &NvFlowGridEmitterSphereParams_NvFlowReflectDataType,  (NvFlowUint8**)&pTestSpheres,   1u},
            {version, &NvFlowGridOffscreenLayerParams_NvFlowReflectDataType, (NvFlowUint8**)&pTestOffscreen, 1u},
            {version, &NvFlowGridRenderLayerParams_NvFlowReflectDataType,    (NvFlowUint8**)&pTestRender,    1u}
        };
        static NvFlowDatabaseSnapshot snapshot = {
            version,
            typeSnapshots,
            4u
        };

        static NvFlowGridParamsDescSnapshot gridParamsDescSnapshot = { snapshot, 0.0, 1.f / 60.f, NV_FLOW_FALSE, nullptr, 0u };

        ptr->loader.gridParamsInterface.commitParams(paramSrc, &gridParamsDescSnapshot);

        FlowStandaloneOutput output = {};
        flowStandaloneUpdateInstance(ptr, (double)idx, &output, true, nullptr, nullptr);

        printf("Standalone instance smoke size = %d\n", (uint32_t)output.smokeNanoVdbReadbackSize);
    }

    ptr->loader.gridParamsInterface.destroyGridParamsNamed(paramSrcNamed);

    flowStandaloneDestroyInstance(ptr);

    return 0;
}
