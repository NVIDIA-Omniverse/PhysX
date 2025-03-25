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
#include <string.h>
#include <stdarg.h>

#include <math.h>

#include "imgui.h"
#include "Loader.h"
#include "ImguiRenderer.h"
#include "ShapeRenderer.h"
#include "FrameCapture.h"

#include "NvFlowArray.h"

#include "NvFlowUploadBuffer.h"

#include "Camera.h"
#include "Timer.h"

#include "NvFlowDatabase.h"

#include "EditorCommon.h"

int testStandaloneMode();

void printError(const char* str, void* userdata)
{
    fprintf(stderr, "FlowLoaderError: %s\n", str);
}

int init(App* ptr)
{
    // initialize GLFW
    if (!ptr->headless)
    {
        if (editorGlfw_init(ptr))
        {
            return 1;
        }
    }

    // initialize graphics
    {
        NvFlowSwapchainDesc swapchainDesc = {};
        editorGlfw_getSwapchainDesc(ptr, &swapchainDesc);

        editorCompute_init(&ptr->compute, &swapchainDesc, ptr->headless);

        ptr->camera = NvFlowCameraCreate((int)ptr->windowWidth, (int)ptr->windowHeight);
    }

    NvFlowContext* context = ptr->compute.loader.deviceInterface.getContext(ptr->compute.deviceQueue);

    NvFlowLogPrint_t logPrint = ptr->compute.contextInterface.getLogPrint(context);

    // initialize imgui
    if (!ptr->headless)
    {
        editorImgui_init(&ptr->imgui, &ptr->compute.contextInterface, context);
    }

    // initialize shape renderer
    {
        NvFlowShapeRendererInterface_duplicate(&ptr->shapeRendererInterface, NvFlowGetShapeRendererInterface());

        ptr->shapeRenderer = ptr->shapeRendererInterface.create(&ptr->compute.contextInterface, context);

        logPrint(eNvFlowLogLevel_info, "Initialized Shape Renderer");
    }

    // initialize frame capture
    {
        NvFlowFrameCaptureInterface_duplicate(&ptr->frameCaptureInterface, NvFlowGetFrameCaptureInterface());

        ptr->frameCapture = ptr->frameCaptureInterface.create(&ptr->compute.contextInterface, context);

        logPrint(eNvFlowLogLevel_info, "Initialized Frame Capture");
    }

    // initialize flow
    {
        editorFlow_init(&ptr->compute, &ptr->flow);
    }

    // initialize profiling
    {
        fopen_s(&ptr->compute.perflog, "NvFlowPerfLog.txt", "w");
        if (ptr->compute.perflog)
        {
            ptr->compute.loader.deviceInterface.enableProfiler(context, &ptr->compute, editorCompute_reportEntries);
        }

        appTimerInit(&ptr->flowTimerCPU);
        appTimerInit(&ptr->imguiTimerCPU);
        appTimerInit(&ptr->presentTimerCPU);
    }

    return 0;
}

int update(App* ptr)
{
    // fixed dt for the moment
    float deltaTime = 1.f / 60.f;

    NvFlowTexture* swapchainTexture = nullptr;
    if (!ptr->headless)
    {
        swapchainTexture = ptr->compute.loader.deviceInterface.getSwapchainFrontTexture(ptr->compute.swapchain);
    }

    NvFlowContext* context = ptr->compute.loader.deviceInterface.getContext(ptr->compute.deviceQueue);

    NvFlowFloat4x4 projection = {};
    NvFlowFloat4x4 view = {};
    NvFlowCameraGetProjection(ptr->camera, &projection, float(ptr->windowWidth), float(ptr->windowHeight));
    NvFlowCameraGetView(ptr->camera, &view);

    NvFlowCameraAnimationTick(ptr->camera, 1.f / 60.f);

    NvFlowTextureTransient* offscreenColorTransient = nullptr;
    NvFlowTextureTransient* offscreenDepthTransient = nullptr;
    if (ptr->windowWidth != 0 && ptr->windowHeight != 0)
    {
        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        texDesc.format = eNvFlowFormat_r16g16b16a16_float;
        texDesc.width = ptr->windowWidth;
        texDesc.height = ptr->windowHeight;
        texDesc.depth = 1u;
        texDesc.mipLevels = 1u;

        offscreenColorTransient = ptr->compute.contextInterface.getTextureTransient(context, &texDesc);

        texDesc.format = eNvFlowFormat_r32_float;

        offscreenDepthTransient = ptr->compute.contextInterface.getTextureTransient(context, &texDesc);
    }

    if (offscreenColorTransient)
    {
        if (!ptr->flow.simonly)
        {
            NvFlowFloat4 spherePositionRadius[1] = { {-100.f, 0.f, 0.f, 50.f} };

            NvFlowShapeRendererParams shapeParams = {};
            shapeParams.numSpheres = ptr->sphereEnabled ? 1u : 0u;
            shapeParams.spherePositionRadius = spherePositionRadius;

            ptr->shapeRendererInterface.render(
                context,
                ptr->shapeRenderer,
                &shapeParams,
                &view,
                &projection,
                ptr->windowWidth,
                ptr->windowHeight,
                offscreenDepthTransient,
                offscreenColorTransient
            );
        }

        editorFlow_presimulate(&ptr->compute, &ptr->flow, deltaTime, ptr->isPaused);

        appTimerBegin(&ptr->flowTimerCPU);

        editorFlow_simulate(&ptr->compute, &ptr->flow, deltaTime, ptr->isPaused);

        if (!ptr->flow.simonly)
        {
            editorFlow_offscreen(&ptr->compute, &ptr->flow);

            NvFlowTextureTransient* colorFrontTransient = offscreenColorTransient;
            editorFlow_render(
                &ptr->compute,
                &ptr->flow,
                &colorFrontTransient,
                offscreenDepthTransient,
                ptr->windowWidth,
                ptr->windowHeight,
                &view,
                &projection
            );

            // testing
            ptr->compute.loader.gridInterface.copyTexture(
                context,
                ptr->flow.grid,
                ptr->windowWidth,
                ptr->windowHeight,
                eNvFlowFormat_r16g16b16a16_float,
                colorFrontTransient,
                &colorFrontTransient
            );

            offscreenColorTransient = colorFrontTransient;
        }

        editorFlow_unmap(&ptr->compute, &ptr->flow);

        appTimerEnd(&ptr->flowTimerCPU);

        float flowCpuTime = 0.f;
        appTimerGetResults(&ptr->flowTimerCPU, &flowCpuTime);

        float aveFlowCpuTime = 0.f;
        if (appTimerUpdateStats(&ptr->flowTimerCPU, flowCpuTime, 60.f, &aveFlowCpuTime))
        {
            printf("Average Flow CPU time = %f ms\n", 1000.f * aveFlowCpuTime);
        }
    }

    appTimerBegin(&ptr->imguiTimerCPU);

    // render imgui
    if (swapchainTexture)
    {
        NvFlowTextureTransient* textureTransient = ptr->compute.contextInterface.registerTextureAsTransient(context, swapchainTexture);

        editorGlfw_newFrame(ptr, deltaTime);

        editorImgui_update(&ptr->imgui, ptr, &ptr->compute, &ptr->flow);

        editorImgui_render(&ptr->imgui, context, offscreenColorTransient, textureTransient, ptr->windowWidth, ptr->windowHeight);

        if (ptr->captureEnabled)
        {
            ptr->frameCaptureInterface.capture(context, ptr->frameCapture, ptr->windowWidth, ptr->windowHeight, textureTransient);
        }
        ptr->frameCaptureInterface.update(context, ptr->frameCapture);
    }

    appTimerEnd(&ptr->imguiTimerCPU);

    // report results
    {
        float imguiCpuTime = 0.f;
        appTimerGetResults(&ptr->imguiTimerCPU, &imguiCpuTime);

        float aveImguiCpuTime = 0.f;
        if (appTimerUpdateStats(&ptr->imguiTimerCPU, imguiCpuTime, 60.f, &aveImguiCpuTime))
        {
            printf("Average Imgui CPU time = %f ms\n", 1000.f * aveImguiCpuTime);
        }
    }

    appTimerBegin(&ptr->presentTimerCPU);

    NvFlowUint64 flushedFrameID = 0llu;
    if (!ptr->headless)
    {
        int deviceReset = ptr->compute.loader.deviceInterface.presentSwapchain(ptr->compute.swapchain, ptr->compute.vsync, &flushedFrameID);
        if (deviceReset)
        {
            editorCompute_logPrint(eNvFlowLogLevel_error, "Device Reset!!!");
            return 1;
        }
    }
    else
    {
        ptr->compute.loader.deviceInterface.flush(ptr->compute.deviceQueue, &flushedFrameID, nullptr, nullptr);
    }

    appTimerEnd(&ptr->presentTimerCPU);

    // report results
    {
        float presentCpuTime = 0.f;
        appTimerGetResults(&ptr->presentTimerCPU, &presentCpuTime);

        float avePresentCpuTime = 0.f;
        if (appTimerUpdateStats(&ptr->presentTimerCPU, presentCpuTime, 60.f, &avePresentCpuTime))
        {
            printf("Average Present CPU time = %f ms\n", 1000.f * avePresentCpuTime);
        }
    }

    ptr->compute.loader.deviceInterface.waitForFrame(ptr->compute.deviceQueue, flushedFrameID);

    // allow benchmark to request exit
    if (!ptr->compute.benchmarkShouldRun)
    {
        ptr->shouldRun = false;
    }

    if (!ptr->headless)
    {
        if (editorGlfw_processEvents(ptr))
        {
            return 1;
        }
    }

    if (!ptr->shouldRun)
    {
        editorCompute_logPrint(eNvFlowLogLevel_info, "ShouldRun == false");
        return 1;
    }

    return 0;
}

void destroy(App* ptr)
{
    NvFlowContext* context = ptr->compute.loader.deviceInterface.getContext(ptr->compute.deviceQueue);

    NvFlowLogPrint_t logPrint = ptr->compute.contextInterface.getLogPrint(context);

    // destroy profiling
    {
        appTimerDestroy(&ptr->flowTimerCPU);
        appTimerDestroy(&ptr->imguiTimerCPU);
        appTimerDestroy(&ptr->presentTimerCPU);

        if (ptr->compute.perflog)
        {
            ptr->compute.loader.deviceInterface.disableProfiler(context);

            fclose(ptr->compute.perflog);
            ptr->compute.perflog = nullptr;
        }
    }

    // destroy grid
    {
        editorFlow_destroy(&ptr->compute, &ptr->flow);
    }

    // destroy frame capture
    {
        ptr->frameCaptureInterface.destroy(context, ptr->frameCapture);

        logPrint(eNvFlowLogLevel_info, "Destroyed Frame Capture");
    }

    // destroy shape renderer
    {
        ptr->shapeRendererInterface.destroy(context, ptr->shapeRenderer);

        logPrint(eNvFlowLogLevel_info, "Destroyed Shape Renderer");
    }

    // destroy imgui
    if (!ptr->headless)
    {
        editorImgui_destroy(&ptr->imgui, context);
    }

    // destroy graphics
    {
        NvFlowCameraDestroy(ptr->camera);

        editorCompute_destroy(&ptr->compute);
    }

    // destroy GLFW
    if (!ptr->headless)
    {
        editorGlfw_destroy(ptr);
    }
}

int main(int argc, char** argv)
{
    App app = {};

    for (int argIdx = 0; argIdx < argc; argIdx++)
    {
        if (strcmp(argv[argIdx], "--headless") == 0)
        {
            app.compute.headless = NV_FLOW_TRUE;
        }
        else if (strcmp(argv[argIdx], "--vulkan") == 0)
        {
            app.compute.contextApi = eNvFlowContextApi_vulkan;
        }
        else if (strcmp(argv[argIdx], "--cpu") == 0)
        {
            app.compute.contextApi = eNvFlowContextApi_cpu;
        }
        else if (strcmp(argv[argIdx], "--threads") == 0)
        {
            argIdx++;
            if (argIdx < argc)
            {
                app.compute.threadCount = atoi(argv[argIdx]);
            }
        }
        else if (strcmp(argv[argIdx], "-o") == 0)
        {
            argIdx++;
            if (argIdx < argc)
            {
                app.compute.outputFilename = argv[argIdx];
            }
        }
        else if (strcmp(argv[argIdx], "--benchmark") == 0)
        {
            argIdx++;
            if (argIdx < argc)
            {
                app.compute.benchmarkFrameCount = atoi(argv[argIdx]);
            }
        }
        else if (strcmp(argv[argIdx], "--maxlocations") == 0)
        {
            argIdx++;
            if (argIdx < argc)
            {
                app.flow.targetMaxLocations = atoi(argv[argIdx]);
            }
        }
        else if (strcmp(argv[argIdx], "--cellsize") == 0)
        {
            argIdx++;
            if (argIdx < argc)
            {
                app.flow.cellsizeOverride = (float)atof(argv[argIdx]);
            }
        }
        else if (strcmp(argv[argIdx], "--smallblocks") == 0)
        {
            app.flow.smallBlocksOverride = NV_FLOW_TRUE;
        }
        else if (strcmp(argv[argIdx], "--simonly") == 0)
        {
            app.flow.simonly = NV_FLOW_TRUE;
        }
        else if (strcmp(argv[argIdx], "--stage") == 0)
        {
            argIdx++;
            if (argIdx < argc)
            {
                app.flow.cmdStage = argv[argIdx];
            }
        }
        else if (strcmp(argv[argIdx], "--standalone") == 0)
        {
            return testStandaloneMode();
        }
    }

    const char* apiStrs[eNvFlowContextApi_count] = { "Abstract", "Vulkan", "D3D12", "CPU" };
    printf("Configuration:\n"
        "api(%s)\n"
        "threadCount(%d)\n"
        "outputFilename(%s)\n"
        "benchmarkFrames(%d)\n"
        "cellsizeOverride(%f)\n"
        "smallBlocksOverride(%d)\n"
        "simonly(%d)\n"
        "cmdStage(%s)\n"
        "maxLocations(%d)\n",
        apiStrs[app.compute.contextApi],
        app.compute.threadCount,
        app.compute.outputFilename ? app.compute.outputFilename : "unset",
        app.compute.benchmarkFrameCount,
        app.flow.cellsizeOverride,
        app.flow.smallBlocksOverride,
        app.flow.simonly,
        app.flow.cmdStage,
        app.flow.targetMaxLocations
    );

    if (init(&app))
    {
        return 1;
    }

    while (!update(&app))
    {

    }

    destroy(&app);

    return 0;
}

#ifdef NV_FLOW_DEBUG_ALLOC
#include <stdio.h>
#include <atomic>
std::atomic_int32_t allocCount(0u);
void* operator new(std::size_t sz)
{
    if (sz == 0u) sz = 1u;
    allocCount++;
    return std::malloc(sz);
}
void operator delete(void* ptr)
{
    std::free(ptr);
    int32_t count = allocCount.fetch_sub(1) - 1;
    printf("NvFlowEditor.cpp free() refCount = %d\n", count);
}
void* operator new[](std::size_t sz)
{
    if (sz == 0u) sz = 1u;
    allocCount++;
    return std::malloc(sz);
}
void operator delete[](void* ptr)
{
    std::free(ptr);
    int32_t count = allocCount.fetch_sub(1) - 1;
    printf("NvFlowEditor.cpp free() refCount = %d\n", count);
}
#endif
