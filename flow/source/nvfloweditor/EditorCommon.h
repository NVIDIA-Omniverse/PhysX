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

#pragma once

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
#include "NvFlowString.h"

#include "NvFlowUploadBuffer.h"

#include "Camera.h"
#include "Timer.h"

#include "NvFlowDatabase.h"

#include "NvFlowStringHash.h"

// Editor stage API

struct EditorFlow;

void editorFlow_clearStage(EditorFlow* ptr);

void editorFlow_definePrim(EditorFlow* ptr, const char* type, const char* path, const char* name);

void editorFlow_setAttribute(EditorFlow* ptr, const char* primPath, const char* name, const void* data, NvFlowUint64 sizeInBytes);
void editorFlow_setAttributeFloat(EditorFlow* ptr, const char* primPath, const char* name, float value);
void editorFlow_setAttributeInt(EditorFlow* ptr, const char* primPath, const char* name, int value);
void editorFlow_setAttributeUint(EditorFlow* ptr, const char* primPath, const char* name, NvFlowUint value);
void editorFlow_setAttributeBool(EditorFlow* ptr, const char* primPath, const char* name, NvFlowBool32 value);
void editorFlow_setAttributeFloat3(EditorFlow* ptr, const char* primPath, const char* name, NvFlowFloat3 value);
void editorFlow_setAttributeFloat3Array(EditorFlow* ptr, const char* primPath, const char* name, const NvFlowFloat3* values, NvFlowUint64 elementCount);
void editorFlow_setAttributeFloat4Array(EditorFlow* ptr, const char* primPath, const char* name, const NvFlowFloat4* values, NvFlowUint64 elementCount);
void editorFlow_setAttributeIntArray(EditorFlow* ptr, const char* primPath, const char* name, const int* values, NvFlowUint64 elementCount);

// Builtin Scenes

struct EditorFlowStage
{
    const char* stageName;
    void*(*init)(EditorFlow* ptr);
    void(*update)(EditorFlow* ptr, void* userdata, double time, float deltaTime);
    void(*destroy)(EditorFlow* ptr, void* userdata);
};

void editorFlowStage_getBuiltinStages(const EditorFlowStage*** pStages, NvFlowUint64* pStageCount);

void editorFlowStage_applyOverrides(EditorFlow* ptr, float cellsizeOverride, NvFlowBool32 smallBlocksOverride);

// Editor subsystems

struct App;

void printError(const char* str, void* userdata);

struct EditorCompute
{
    // compute config
    NvFlowContextApi contextApi = eNvFlowContextApi_vulkan;
    NvFlowUint threadCount = 0u;
    NvFlowBool32 headless = NV_FLOW_FALSE;
    NvFlowBool32 vsync = NV_FLOW_TRUE;

    // loader/compute foundation
    void* nvFlowModule = nullptr;
    void* nvFlowExtModule = nullptr;
    NvFlowLoader loader = {};
    NvFlowContextInterface contextInterface = {};
    NvFlowDeviceManager* deviceManager = nullptr;
    NvFlowDevice* device = nullptr;
    NvFlowDeviceQueue* deviceQueue = nullptr;
    NvFlowSwapchain* swapchain = nullptr;

    // benchmarking
    FILE* perflog = nullptr;
    NvFlowArray<const char*> statEntries_label;
    NvFlowArray<NvFlowBool32> statEntries_active;
    NvFlowArray<float> statEntries_cpuDeltaTime_sum;
    NvFlowArray<float> statEntries_cpuDeltaTime_count;
    NvFlowArray<float> statEntries_gpuDeltaTime_sum;
    NvFlowArray<float> statEntries_gpuDeltaTime_count;

    NvFlowArray<const char*> statOut_label;
    NvFlowArray<float> statOut_cpu;
    NvFlowArray<float> statOut_gpu;

    const char* outputFilename = "benchmark.csv";
    FILE* outputFile = nullptr;
    NvFlowUint benchmarkFrameCount = 0u;
    NvFlowUint benchmarkFrameID = 0u;
    bool benchmarkShouldRun = true;
    NvFlowUint benchmarkActiveBlockCount = 0u;

    AppTimer benchmarkTimerCPU = {};
};

void editorCompute_init(EditorCompute* ptr, const NvFlowSwapchainDesc* swapchainDesc, NvFlowBool32 headless);
void editorCompute_destroy(EditorCompute* ptr);
void editorCompute_reportEntries(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries);
void editorCompute_logPrint(NvFlowLogLevel level, const char* format, ...);

struct EditorFlowCommand
{
    const char* cmd;
    const char* path;
    const char* name;
    const char* type;
    const NvFlowUint8* data;
    NvFlowUint64 dataSize;
};

struct EditorFlow
{
    // flow editor config
    NvFlowUint maxLocations = 4096u;
    NvFlowUint targetMaxLocations = 4096u;
    NvFlowBool32 simonly = NV_FLOW_FALSE;
    NvFlowBool32 smallBlocksOverride = NV_FLOW_FALSE;
    float cellsizeOverride = 0.f;

    const char* cmdStage = nullptr;
    NvFlowArray<const EditorFlowStage*> stages;
    NvFlowUint64 targetStageIdx = 0llu;
    const EditorFlowStage* currentStage = nullptr;
    void* stageUserdata = nullptr;

    // active state
    double absoluteSimTime = 0.0;
    double animationTime = 0.0;
    NvFlowUint activeBlockCount = 0u;
    NvFlowUint3 activeBlockDim = { 0u, 0u, 0u };
    NvFlowUint activeBlockCountIsosurface = 0u;
    NvFlowUint3 activeBlockDimIsosurface = { 32u, 16u, 16u };

    // flow grid
    NvFlowGrid* grid = nullptr;
    NvFlowGridParamsNamed* gridParamsServer = nullptr;
    NvFlowGridParamsNamed* gridParamsClient = nullptr;
    NvFlowGridParams* gridParams = nullptr;
    NvFlowDatabase gridParamsSet;
    NvFlowArray<const char*> typenames;
    NvFlowArray<const char*> displayTypenames;
    NvFlowArray<const NvFlowReflectDataType*> dataTypes;
    NvFlowArray<NvFlowDatabaseType*> types;
    NvFlowArray<NvFlowDatabaseInstance*> abstractParamsList;

    // dynamic grid params
    NvFlowGridParamsDesc gridParamsDesc = {};
    NvFlowGridParams* clientGridParams = nullptr;
    NvFlowGridParamsSnapshot* paramsSnapshot = nullptr;

    NvFlowArray<EditorFlowCommand> commands;
    NvFlowStringPool* commandStringPool = nullptr;
    NvFlowStringHashTable<NvFlowDatabasePrim*> primMap;
};

void editorFlow_init(EditorCompute* ctx, EditorFlow* ptr);
void editorFlow_presimulate(EditorCompute* ctx, EditorFlow* ptr, float deltaTime, NvFlowBool32 isPaused);
void editorFlow_simulate(EditorCompute* ctx, EditorFlow* ptr, float deltaTime, NvFlowBool32 isPaused);
void editorFlow_offscreen(EditorCompute* ctx, EditorFlow* ptr);
void editorFlow_render(EditorCompute* ctx, EditorFlow* ptr,
    NvFlowTextureTransient** colorFrontTransient,
    NvFlowTextureTransient* offscreenDepthTransient,
    NvFlowUint windowWidth,
    NvFlowUint windowHeight,
    const NvFlowFloat4x4* view,
    const NvFlowFloat4x4* projection
);
void editorFlow_unmap(EditorCompute* ctx, EditorFlow* ptr);
void editorFlow_destroy(EditorCompute* ctx, EditorFlow* ptr);

struct EditorImgui
{
    NvFlowImguiRendererInterface imguiRendererInterface = {};
    NvFlowImguiRenderer* imguiRenderer = nullptr;
};

void editorImgui_init(EditorImgui* ptr, NvFlowContextInterface* contextInterface, NvFlowContext* context);
void editorImgui_update(
    EditorImgui* ptr,
    App* app,
    EditorCompute* compute,
    EditorFlow* flow
);
void editorImgui_render(
    EditorImgui* ptr,
    NvFlowContext* context,
    NvFlowTextureTransient* colorIn,
    NvFlowTextureTransient* colorOut,
    NvFlowUint windowWidth,
    NvFlowUint windowHeight
);
void editorImgui_destroy(EditorImgui* ptr, NvFlowContext* context);

struct App
{
    NvFlowUint windowWidth = 1280u;
    NvFlowUint windowHeight = 800u;
    NvFlowUint windowWidthOld = 1280u;
    NvFlowUint windowHeightOld = 800u;

    GLFWwindow* window = nullptr;
    NvFlowBool32 headless = NV_FLOW_FALSE;
    NvFlowBool32 isPaused = NV_FLOW_FALSE;
    NvFlowBool32 overlayEnabled = NV_FLOW_FALSE;
    NvFlowBool32 editorEnabled = NV_FLOW_TRUE;
    NvFlowBool32 sphereEnabled = NV_FLOW_FALSE;
    NvFlowBool32 captureEnabled = NV_FLOW_FALSE;

    int fullscreenState = 0;
    NvFlowBool32 shouldRun = NV_FLOW_TRUE;
    int mouseX = 0;
    int mouseY = 0;
    int mouseYInv = 0;
    NvFlowBool32 mouseJustPressed[5u];

    NvFlowShapeRendererInterface shapeRendererInterface = {};
    NvFlowShapeRenderer* shapeRenderer = nullptr;

    NvFlowFrameCaptureInterface frameCaptureInterface = {};
    NvFlowFrameCapture* frameCapture = nullptr;

    EditorImgui imgui = {};

    EditorCompute compute = {};
    EditorFlow flow = {};

    NvFlowCamera* camera = nullptr;

    AppTimer flowTimerCPU = {};
    AppTimer imguiTimerCPU = {};
    AppTimer presentTimerCPU = {};
};

int editorGlfw_init(App* ptr);
void editorGlfw_getSwapchainDesc(App* ptr, NvFlowSwapchainDesc* outDesc);
void editorGlfw_newFrame(App* ptr, float deltaTime);
int editorGlfw_processEvents(App* ptr);
void editorGlfw_destroy(App* ptr);

#if !defined(_WIN32)
NV_FLOW_INLINE void fopen_s(FILE** streamptr, const char* filename, const char* mode)
{
	*streamptr = fopen(filename, mode);
}
#endif

