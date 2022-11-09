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
//
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

#include "EditorCommon.h"

void editorCompute_init(EditorCompute* ptr, const NvFlowSwapchainDesc* swapchainDesc, NvFlowBool32 headless)
{
    ptr->headless = headless;

    // initialize benchmarking if enabled
    if (ptr->benchmarkFrameCount)
    {
        ptr->vsync = NV_FLOW_FALSE;
        appTimerInit(&ptr->benchmarkTimerCPU);
        appTimerBegin(&ptr->benchmarkTimerCPU);
        fopen_s(&ptr->outputFile, ptr->outputFilename, "w");

        fprintf(ptr->outputFile, "FrameID, FrameTime, CPUTime, GPUTime, ActiveBlockCount\n");
    }

    NvFlowLoaderInitDeviceAPI(&ptr->loader, printError, nullptr, ptr->contextApi);

    NvFlowBool32 validation = NV_FLOW_TRUE;

    ptr->deviceManager = ptr->loader.deviceInterface.createDeviceManager(validation, nullptr, ptr->threadCount);

    NvFlowDeviceDesc deviceDesc = {};
    deviceDesc.deviceIndex = 0u;
    deviceDesc.enableExternalUsage = NV_FLOW_TRUE;
    deviceDesc.logPrint = editorCompute_logPrint;

    ptr->device = ptr->loader.deviceInterface.createDevice(ptr->deviceManager, &deviceDesc);

    ptr->deviceQueue = ptr->loader.deviceInterface.getDeviceQueue(ptr->device);

    if (!ptr->headless)
    {
        ptr->swapchain = ptr->loader.deviceInterface.createSwapchain(ptr->deviceQueue, swapchainDesc);
    }

    NvFlowContextInterface_duplicate(&ptr->contextInterface, ptr->loader.deviceInterface.getContextInterface(ptr->deviceQueue));

    // testing external semaphore
    NvFlowUint64 testHandle = 0u;
    NvFlowDeviceSemaphore* testSemaphore = ptr->loader.deviceInterface.createSemaphore(ptr->device);

    ptr->loader.deviceInterface.getSemaphoreExternalHandle(testSemaphore, &testHandle, sizeof(testHandle));

    printf("Test semaphore handle = %llu\n", testHandle);

    ptr->loader.deviceInterface.closeSemaphoreExternalHandle(testSemaphore, &testHandle, sizeof(testHandle));

    ptr->loader.deviceInterface.destroySemaphore(testSemaphore);
}

void editorCompute_destroy(EditorCompute* ptr)
{
    if (ptr->swapchain)
    {
        ptr->loader.deviceInterface.destroySwapchain(ptr->swapchain);
    }
    ptr->loader.deviceInterface.destroyDevice(ptr->deviceManager, ptr->device);
    ptr->loader.deviceInterface.destroyDeviceManager(ptr->deviceManager);

    NvFlowLoaderDestroy(&ptr->loader);

    // destroy benchmarking if enabled
    if (ptr->benchmarkFrameCount)
    {
        appTimerEnd(&ptr->benchmarkTimerCPU);
        fclose(ptr->outputFile);
    }
}

void editorCompute_reportEntries(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries)
{
    EditorCompute* ptr = (EditorCompute*)userdata;

    if (ptr->benchmarkFrameCount)
    {
        appTimerEnd(&ptr->benchmarkTimerCPU);

        float deltaTime = 0.f;
        appTimerGetResults(&ptr->benchmarkTimerCPU, &deltaTime);

        float cpuSum = 0.f;
        float gpuSum = 0.f;
        for (NvFlowUint entryIdx = 0u; entryIdx < numEntries; entryIdx++)
        {
            cpuSum += entries[entryIdx].cpuDeltaTime;
            gpuSum += entries[entryIdx].gpuDeltaTime;
        }
        if (ptr->outputFile && ptr->benchmarkFrameID > 0u)
        {
            fprintf(ptr->outputFile, "%d, %f, %f, %f, %d\n", ptr->benchmarkFrameID, 1000.f * deltaTime, 1000.f * cpuSum, 1000.f * gpuSum, ptr->benchmarkActiveBlockCount);
        }
        ptr->benchmarkFrameID++;
        if (ptr->benchmarkFrameID > ptr->benchmarkFrameCount)
        {
            ptr->benchmarkShouldRun = false;
        }

        appTimerBegin(&ptr->benchmarkTimerCPU);
    }

    // reset active mask
    for (NvFlowUint entryIdx = 0u; entryIdx < ptr->statEntries_active.size; entryIdx++)
    {
        ptr->statEntries_active[entryIdx] = NV_FLOW_FALSE;
    }

    NvFlowUint minInactiveEntry = 0u;
    for (NvFlowUint profEntryIdx = 0u; profEntryIdx < numEntries; profEntryIdx++)
    {
        const NvFlowProfilerEntry profEntry = entries[profEntryIdx];

        // update minInactiveEntry
        for (; minInactiveEntry < ptr->statEntries_active.size; minInactiveEntry++)
        {
            if (!ptr->statEntries_active[minInactiveEntry])
            {
                break;
            }
        }

        // search for matching label
        NvFlowUint64 entryIdx = minInactiveEntry;
        for (; entryIdx < ptr->statEntries_label.size; entryIdx++)
        {
            if (!ptr->statEntries_active[entryIdx] && strcmp(profEntry.label, ptr->statEntries_label[entryIdx]) == 0)
            {
                break;
            }
        }
        // allocate new if needed
        if (entryIdx >= ptr->statEntries_label.size)
        {
            entryIdx = ptr->statEntries_label.size;

            ptr->statEntries_label.pushBack(profEntry.label);
            ptr->statEntries_active.pushBack(NV_FLOW_FALSE);
            ptr->statEntries_cpuDeltaTime_sum.pushBack(0.f);
            ptr->statEntries_cpuDeltaTime_count.pushBack(0.f);
            ptr->statEntries_gpuDeltaTime_sum.pushBack(0.f);
            ptr->statEntries_gpuDeltaTime_count.pushBack(0.f);
        }
        // update entry
        {
            ptr->statEntries_active[entryIdx] = NV_FLOW_TRUE;
            ptr->statEntries_cpuDeltaTime_sum[entryIdx] += profEntry.cpuDeltaTime;
            ptr->statEntries_cpuDeltaTime_count[entryIdx] += 1.f;
            ptr->statEntries_gpuDeltaTime_sum[entryIdx] += profEntry.gpuDeltaTime;
            ptr->statEntries_gpuDeltaTime_count[entryIdx] += 1.f;
        }
    }

    // subsample by default, to avoid a massive log
    if ((captureID % 15) == 0)
    {
        ptr->statOut_label.size = 0u;
        ptr->statOut_cpu.size = 0u;
        ptr->statOut_gpu.size = 0u;

        ptr->statOut_label.pushBack("Total");
        ptr->statOut_cpu.pushBack(0.f);
        ptr->statOut_gpu.pushBack(0.f);

        float cpuSum = 0.f;
        float gpuSum = 0.f;

        fprintf(ptr->perflog, "\nFrame[%lld] : label cpuTimeMS gpuTimeMS\n", captureID);
        for (NvFlowUint entryIdx = 0u; entryIdx < ptr->statEntries_label.size; entryIdx++)
        {
            const char* label = ptr->statEntries_label[entryIdx];
            float cpuDeltaTime = ptr->statEntries_cpuDeltaTime_sum[entryIdx] / ptr->statEntries_cpuDeltaTime_count[entryIdx];
            float gpuDeltaTime = ptr->statEntries_gpuDeltaTime_sum[entryIdx] / ptr->statEntries_gpuDeltaTime_count[entryIdx];
            fprintf(ptr->perflog, "%s, %f, %f\n", label, 1000.f * cpuDeltaTime, 1000.f * gpuDeltaTime);

            ptr->statOut_label.pushBack(label);
            ptr->statOut_cpu.pushBack(1000.f * cpuDeltaTime);
            ptr->statOut_gpu.pushBack(1000.f * gpuDeltaTime);

            cpuSum += cpuDeltaTime;
            gpuSum += gpuDeltaTime;
        }

        ptr->statOut_cpu[0] = 1000.f * cpuSum;
        ptr->statOut_gpu[0] = 1000.f * gpuSum;

        // reset stats
        ptr->statEntries_label.size = 0u;
        ptr->statEntries_active.size = 0u;
        ptr->statEntries_cpuDeltaTime_sum.size = 0u;
        ptr->statEntries_cpuDeltaTime_count.size = 0u;
        ptr->statEntries_gpuDeltaTime_sum.size = 0u;
        ptr->statEntries_gpuDeltaTime_count.size = 0u;
    }
}

void editorCompute_logPrint(NvFlowLogLevel level, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    const char* prefix = "Unknown";
    if (level == eNvFlowLogLevel_error)
    {
        prefix = "Error";
    }
    else if (level == eNvFlowLogLevel_warning)
    {
        prefix = "Warning";
    }
    else if (level == eNvFlowLogLevel_info)
    {
        prefix = "Info";
    }
    printf("%s: ", prefix);
    vprintf(format, args);
    printf("\n");

    va_end(args);
}
