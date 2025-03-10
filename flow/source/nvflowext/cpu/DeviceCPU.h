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

#pragma once

namespace NvFlowCPU
{
    struct DeviceManager;
    struct Device;
    struct DeviceSemaphore;
    struct DeviceQueue;
    struct Swapchain;
    struct Context;

    struct Buffer;
    struct BufferTransient;
    struct BufferAcquire;
    struct Texture;
    struct TextureTransient;
    struct TextureAcquire;
    struct Sampler;

    struct ComputePipeline;

    NV_FLOW_CAST_PAIR(NvFlowDeviceManager, DeviceManager)
    NV_FLOW_CAST_PAIR(NvFlowDevice, Device)
    NV_FLOW_CAST_PAIR(NvFlowDeviceSemaphore, DeviceSemaphore)
    NV_FLOW_CAST_PAIR(NvFlowDeviceQueue, DeviceQueue)
    NV_FLOW_CAST_PAIR(NvFlowSwapchain, Swapchain)
    NV_FLOW_CAST_PAIR(NvFlowContext, Context)

    NV_FLOW_CAST_PAIR(NvFlowBuffer, Buffer)
    NV_FLOW_CAST_PAIR(NvFlowBufferTransient, BufferTransient)
    NV_FLOW_CAST_PAIR(NvFlowBufferAcquire, BufferAcquire)
    NV_FLOW_CAST_PAIR(NvFlowTexture, Texture)
    NV_FLOW_CAST_PAIR(NvFlowTextureTransient, TextureTransient)
    NV_FLOW_CAST_PAIR(NvFlowTextureAcquire, TextureAcquire)
    NV_FLOW_CAST_PAIR(NvFlowSampler, Sampler)

    NV_FLOW_CAST_PAIR(NvFlowComputePipeline, ComputePipeline)

    struct DeviceManager
    {
        NvFlowThreadPoolInterface threadPoolInterface;
        NvFlowUint threadCount = 0u;

        NvFlowArray<NvFlowPhysicalDeviceDesc> physicalDeviceDescs;
    };

    NvFlowDeviceManager* createDeviceManager(NvFlowBool32 enableValidationOnDebugBuild, NvFlowThreadPoolInterface* threadPoolInterface, NvFlowUint threadCount);
    void destroyDeviceManager(NvFlowDeviceManager* manager);
    NvFlowBool32 enumerateDevices(NvFlowDeviceManager* manager, NvFlowUint deviceIndex, NvFlowPhysicalDeviceDesc* pDesc);

    struct Device
    {
        NvFlowDeviceDesc desc = {};
        NvFlowLogPrint_t logPrint = nullptr;

        DeviceManager* deviceManager = nullptr;

        DeviceQueue* deviceQueue = nullptr;
    };

    NvFlowDevice* createDevice(NvFlowDeviceManager* deviceManager, const NvFlowDeviceDesc* desc);
    void destroyDevice(NvFlowDeviceManager* deviceManager, NvFlowDevice* device);
    NvFlowDeviceQueue* getDeviceQueue(NvFlowDevice* device);

    struct DeviceSemaphore
    {
        int reserved;
    };

    NvFlowDeviceSemaphore* createSemaphore(NvFlowDevice* device);
    void destroySemaphore(NvFlowDeviceSemaphore* semaphore);
    void getSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, void* dstHandle, NvFlowUint64 dstHandleSize);
    void closeSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, const void* srcHandle, NvFlowUint64 srcHandleSize);

    struct DeviceQueue
    {
        Device* device = nullptr;

        NvFlowUint64 lastFenceCompleted = 1u;
        NvFlowUint64 nextFenceValue = 2u;

        Context* context = nullptr;
    };

    int flush(NvFlowDeviceQueue* ptr, NvFlowUint64* flushedFrameID, NvFlowDeviceSemaphore* waitSemaphore, NvFlowDeviceSemaphore* signalSemaphore);
    NvFlowUint64 getLastFrameCompleted(NvFlowDeviceQueue* queue);
    void waitForFrame(NvFlowDeviceQueue* ptr, NvFlowUint64 frameID);
    void waitIdle(NvFlowDeviceQueue* ptr);
    NvFlowContextInterface* getContextInterface(NvFlowDeviceQueue* ptr);
    NvFlowContext* getContext(NvFlowDeviceQueue* ptr);

    DeviceQueue* deviceQueue_create(Device* device);
    void deviceQueue_destroy(DeviceQueue* ptr);

    struct Swapchain
    {
        NvFlowSwapchainDesc desc = {};

        DeviceQueue* deviceQueue = nullptr;

#if defined(_WIN32)
#else
        void* moduleX11 = nullptr;
        decltype(&XGetWindowAttributes) p_XGetWindowAttributes = nullptr;
        decltype(&XCreateImage) p_XCreateImage = nullptr;
        decltype(&XCreateGC) p_XCreateGC = nullptr;
        decltype(&XPutImage) p_XPutImage = nullptr;
        decltype(&XFreeGC) p_XFreeGC = nullptr;
#endif
        NvFlowUint width = 0;
        NvFlowUint height = 0;

        NvFlowTexture* texture = nullptr;
    };

    NvFlowSwapchain* createSwapchain(NvFlowDeviceQueue* queue, const NvFlowSwapchainDesc* desc);
    void destroySwapchain(NvFlowSwapchain* swapchain);
    void resizeSwapchain(NvFlowSwapchain* swapchain, NvFlowUint width, NvFlowUint height);
    int presentSwapchain(NvFlowSwapchain* swapchain, NvFlowBool32 vsync, NvFlowUint64* flushedFrameID);
    NvFlowTexture* getSwapchainFrontTexture(NvFlowSwapchain* swapchain);

    void swapchain_getWindowSize(Swapchain* ptr, NvFlowUint* width, NvFlowUint* height);

    struct Context;

    struct Buffer
    {
        NvFlowUint activeMask = 0u;
        NvFlowBufferDesc desc = {};
        NvFlowCPU_Resource resource = {};
    };

    struct BufferTransient
    {
        NvFlowBufferDesc desc = {};
        Buffer* buffer = nullptr;
    };

    struct BufferAcquire
    {
        BufferTransient* bufferTransient = nullptr;
        Buffer* buffer = nullptr;
    };

    NvFlowBuffer* createBuffer(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc);
    void destroyBuffer(NvFlowContext* context, NvFlowBuffer* buffer);
    NvFlowBufferTransient* getBufferTransient(NvFlowContext* context, const NvFlowBufferDesc* desc);
    NvFlowBufferTransient* registerBufferAsTransient(NvFlowContext* context, NvFlowBuffer* buffer);
    NvFlowBufferAcquire* enqueueAcquireBuffer(NvFlowContext* context, NvFlowBufferTransient* buffer);
    NvFlowBool32 getAcquiredBuffer(NvFlowContext* context, NvFlowBufferAcquire* acquire, NvFlowBuffer** outBuffer);
    void* mapBuffer(NvFlowContext* context, NvFlowBuffer* buffer);
    void unmapBuffer(NvFlowContext* context, NvFlowBuffer* buffer);
    NvFlowBufferTransient* getBufferTransientById(NvFlowContext* context, NvFlowUint64 bufferId);

    void context_destroyBuffers(Context* context);

    struct Texture
    {
        NvFlowUint activeMask = 0u;
        NvFlowTextureDesc desc = {};
        NvFlowCPU_Resource resource = {};
    };

    struct TextureTransient
    {
        NvFlowTextureDesc desc = {};
        Texture* texture = nullptr;
    };

    struct TextureAcquire
    {
        TextureTransient* textureTransient = nullptr;
        Texture* texture = nullptr;
    };

    NvFlowTexture* createTexture(NvFlowContext* context, const NvFlowTextureDesc* desc);
    void destroyTexture(NvFlowContext* context, NvFlowTexture* texture);
    NvFlowTextureTransient* getTextureTransient(NvFlowContext* context, const NvFlowTextureDesc* desc);
    NvFlowTextureTransient* registerTextureAsTransient(NvFlowContext* context, NvFlowTexture* texture);
    NvFlowTextureAcquire* enqueueAcquireTexture(NvFlowContext* context, NvFlowTextureTransient* texture);
    NvFlowBool32 getAcquiredTexture(NvFlowContext* context, NvFlowTextureAcquire* acquire, NvFlowTexture** outTexture);
    NvFlowTextureTransient* getTextureTransientById(NvFlowContext* context, NvFlowUint64 textureId);

    void context_destroyTextures(Context* context);

    struct Sampler
    {
        NvFlowBool32 isActive = NV_FLOW_FALSE;
        NvFlowSamplerDesc desc = {};
        NvFlowCPU_Resource resource = {};
    };

    NvFlowSampler* createSampler(NvFlowContext* context, const NvFlowSamplerDesc* desc);
    NvFlowSampler* getDefaultSampler(NvFlowContext* context);
    void destroySampler(NvFlowContext* context, NvFlowSampler* sampler);

    void context_destroySamplers(Context* context);

    struct ComputePipeline
    {
        NvFlowComputePipelineDesc desc = {};

        NvFlowArray<NvFlowCPU_Resource*> resources;
    };

    NvFlowComputePipeline* createComputePipeline(NvFlowContext* context, const NvFlowComputePipelineDesc* desc);
    void destroyComputePipeline(NvFlowContext* context, NvFlowComputePipeline* pipeline);

    void computePipeline_dispatch(Context* context, const NvFlowPassComputeParams* params);

    struct ProfilerEntry
    {
        const char* label;
        NvFlowUint64 cpuValue;
    };

    struct Profiler
    {
        NvFlowUint64 captureID = 0llu;
        NvFlowUint64 cpuFreq = 0llu;

        NvFlowArray<ProfilerEntry> entries;
        NvFlowArray<NvFlowProfilerEntry> deltaEntries;

        const char* beginCapture = "BeginCapture";

        void* userdata = nullptr;
        void(NV_FLOW_ABI* reportEntries)(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries) = nullptr;
    };

    Profiler* profiler_create(Context* context);
    void profiler_destroy(Context* context, Profiler* ptr);
    void profiler_timestamp(Context* context, Profiler* ptr, const char* label);
    void profiler_flush(Context* context, Profiler* ptr);

    void enableProfiler(NvFlowContext* context, void* userdata, void(NV_FLOW_ABI* reportEntries)(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries));
    void disableProfiler(NvFlowContext* context);

    struct RegisteredResource
    {
        NvFlowBuffer* buffer;
        NvFlowTexture* texture;
        NvFlowUint64 uid;
    };

    struct Context
    {
        DeviceQueue* deviceQueue = nullptr;

        NvFlowArray<Buffer*> pool_buffers;
        NvFlowArray<Texture*> pool_textures;
        NvFlowArray<Sampler*> pool_samplers;

        NvFlowArrayPointer<BufferTransient*> bufferTransients;
        NvFlowArrayPointer<TextureTransient*> textureTransients;

        NvFlowArrayPointer<BufferAcquire*> bufferAcquires;
        NvFlowArrayPointer<TextureAcquire*> textureAcquires;

        NvFlowThreadPoolInterface* threadPoolInterface = nullptr;
        NvFlowThreadPool* threadPool = nullptr;

        Profiler* profiler = nullptr;

        NvFlowUint64 registeredResourceCounter = 0llu;
        NvFlowArray<RegisteredResource> registeredResources;

        NvFlowLogPrint_t logPrint = nullptr;
    };

    void addPassCompute(NvFlowContext* context, const NvFlowPassComputeParams* params);
    void addPassCopyBuffer(NvFlowContext* context, const NvFlowPassCopyBufferParams* params);
    void addPassCopyBufferToTexture(NvFlowContext* context, const NvFlowPassCopyBufferToTextureParams* params);
    void addPassCopyTextureToBuffer(NvFlowContext* context, const NvFlowPassCopyTextureToBufferParams* params);
    void addPassCopyTexture(NvFlowContext* context, const NvFlowPassCopyTextureParams* params);

    NvFlowUint64 registerBufferId(NvFlowContext* context, NvFlowBuffer* buffer);
    NvFlowUint64 registerTextureId(NvFlowContext* context, NvFlowTexture* texture);
    void unregisterBufferId(NvFlowContext* context, NvFlowUint64 bufferId);
    void unregisterTextureId(NvFlowContext* context, NvFlowUint64 textureId);
    void setResourceMinLifetime(NvFlowContext* context, NvFlowUint64 minLifetime);

    Context* context_create(DeviceQueue* deviceQueue);
    void context_destroy(Context* ptr);
    void context_flush(Context* context);
}
