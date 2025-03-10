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

namespace NvFlowVulkan
{
    static const NvFlowUint kMaxFramesInFlight = 3u;
    static const NvFlowUint kMaxSwapchainImages = 8u;

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

    struct Fence;
    struct FormatConverter;

    struct DeviceManager
    {
        void* vulkan_module = nullptr;

        VkInstance vulkanInstance = nullptr;

        NvFlowArray<VkPhysicalDevice> physicalDevices;
        NvFlowArray<VkPhysicalDeviceProperties> deviceProps;
        NvFlowArray<NvFlowPhysicalDeviceDesc> physicalDeviceDescs;

        NvFlowVulkanEnabledInstanceExtensions enabledExtensions = { };
        NvFlowVulkanInstanceLoader loader = {};
    };

    NvFlowDeviceManager* createDeviceManager(NvFlowBool32 enableValidationOnDebugBuild, NvFlowThreadPoolInterface* threadPoolInterface, NvFlowUint threadCount);
    void destroyDeviceManager(NvFlowDeviceManager* manager);
    NvFlowBool32 enumerateDevices(NvFlowDeviceManager* manager, NvFlowUint deviceIndex, NvFlowPhysicalDeviceDesc* pDesc);

    struct Device
    {
        NvFlowDeviceDesc desc = {};
        NvFlowLogPrint_t logPrint = nullptr;

        DeviceManager* deviceManager = nullptr;
        FormatConverter* formatConverter = nullptr;

        VkDevice vulkanDevice = nullptr;

        VkPhysicalDevice physicalDevice = nullptr;
        VkPhysicalDeviceProperties physicalDeviceProperties = {};
        VkPhysicalDeviceMemoryProperties memoryProperties = {};

        uint32_t graphicsQueueIdx = 0u;
        VkQueue graphicsQueue = nullptr;

        DeviceQueue* deviceQueue = nullptr;

        NvFlowVulkanEnabledFeatures enabledFeatures = {};
        NvFlowVulkanEnabledDeviceExtensions enabledExtensions = { };
        NvFlowVulkanDeviceLoader loader = {};

        NvFlowDeviceMemoryStats memoryStats = {};
    };

    NvFlowDevice* createDevice(NvFlowDeviceManager* deviceManager, const NvFlowDeviceDesc* desc);
    void destroyDevice(NvFlowDeviceManager* deviceManager, NvFlowDevice* device);
    NvFlowDeviceQueue* getDeviceQueue(NvFlowDevice* device);
    void getMemoryStats(NvFlowDevice* device, NvFlowDeviceMemoryStats* dstStats);

    void device_reportMemoryAllocate(Device* device, NvFlowMemoryType type, NvFlowUint64 bytes);
    void device_reportMemoryFree(Device* device, NvFlowMemoryType type, NvFlowUint64 bytes);

    struct DeviceSemaphore
    {
        Device* device = nullptr;

        VkSemaphore semaphoreVk;
    };

    NvFlowDeviceSemaphore* createSemaphore(NvFlowDevice* device);
    void destroySemaphore(NvFlowDeviceSemaphore* semaphore);
    void getSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, void* dstHandle, NvFlowUint64 dstHandleSize);
    void closeSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, const void* srcHandle, NvFlowUint64 srcHandleSize);

    struct Fence
    {
        VkFence fence;
        NvFlowBool32 active;
        NvFlowUint64 value;
    };

    struct DeviceQueue
    {
        Device* device = nullptr;

        VkDevice vulkanDevice = nullptr;
        VkQueue graphicsQueue = nullptr;

        VkCommandPool commandPool = VK_NULL_HANDLE;
        int commandBufferIdx = 0u;
        VkCommandBuffer commandBuffers[kMaxFramesInFlight] = { nullptr };
        Fence fences[kMaxFramesInFlight] = { {VK_NULL_HANDLE, 0, 0u} };
        VkCommandBuffer commandBuffer = nullptr;

        VkSemaphore beginFrameSemaphore = VK_NULL_HANDLE;
        VkSemaphore endFrameSemaphore = VK_NULL_HANDLE;

        VkSemaphore currentBeginFrameSemaphore = VK_NULL_HANDLE;
        VkSemaphore currentEndFrameSemaphore = VK_NULL_HANDLE;

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
    void deviceQueue_destroy(DeviceQueue* deviceQueue);
    int flushStepA(DeviceQueue* ptr, NvFlowDeviceSemaphore* waitSemaphore, NvFlowDeviceSemaphore* signalSemaphore);
    void flushStepB(DeviceQueue* ptr);
    void deviceQueue_fenceUpdate(DeviceQueue* ptr, NvFlowUint fenceIdx, NvFlowBool32 blocking);

    struct Swapchain
    {
        NvFlowSwapchainDesc desc = {};

        DeviceQueue* deviceQueue = nullptr;

#if defined(_WIN32)
#else
        void* moduleX11 = nullptr;
        decltype(&XGetWindowAttributes) getWindowAttrib = nullptr;
#endif

        NvFlowBool32 valid = NV_FLOW_FALSE;

        NvFlowBool32 vsync = NV_FLOW_TRUE;
        int width = 0;
        int height = 0;

        VkPresentModeKHR presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        VkFormat swapchainFormat = VK_FORMAT_B8G8R8A8_UNORM;

        VkSurfaceKHR surfaceVulkan = VK_NULL_HANDLE;
        VkSwapchainKHR swapchainVulkan = VK_NULL_HANDLE;
        unsigned int numSwapchainImages = 0u;
        VkImage swapchainImages[kMaxSwapchainImages] = { VK_NULL_HANDLE };
        unsigned int currentSwapchainIdx = 0u;

        NvFlowTexture* textures[kMaxSwapchainImages] = { };
    };

    NvFlowSwapchain* createSwapchain(NvFlowDeviceQueue* queue, const NvFlowSwapchainDesc* desc);
    void destroySwapchain(NvFlowSwapchain* swapchain);
    void resizeSwapchain(NvFlowSwapchain* swapchain, NvFlowUint width, NvFlowUint height);
    int presentSwapchain(NvFlowSwapchain* swapchain, NvFlowBool32 vsync, NvFlowUint64* flushedFrameID);
    NvFlowTexture* getSwapchainFrontTexture(NvFlowSwapchain* swapchain);

    void swapchain_getWindowSize(Swapchain* ptr, NvFlowUint* width, NvFlowUint* height);
    void swapchain_initSwapchain(Swapchain* ptr);
    void swapchain_destroySwapchain(Swapchain* ptr);

    struct Context;

    struct Buffer
    {
        int refCount = 0;
        NvFlowUint64 lastActive = NV_FLOW_FALSE;
        NvFlowBufferDesc desc = {};
        NvFlowMemoryType memoryType = eNvFlowMemoryType_device;
        NvFlowUint64 allocationBytes = 0llu;

        VkDeviceMemory memoryVk = VK_NULL_HANDLE;
        VkBuffer bufferVk = VK_NULL_HANDLE;
        VkBufferView bufferViewVk = VK_NULL_HANDLE;
        NvFlowArray<VkBufferView> aliasBufferViews;
        NvFlowArray<NvFlowFormat> aliasFormats;
        void* mappedData = nullptr;

        VkBufferMemoryBarrier restoreBarrier = {};
        VkBufferMemoryBarrier currentBarrier = {};
    };

    struct BufferTransient
    {
        NvFlowBufferDesc desc = {};
        Buffer* buffer = nullptr;
        BufferTransient* aliasBuffer = nullptr;
        NvFlowFormat aliasFormat = eNvFlowFormat_unknown;
        int nodeBegin = 0;
        int nodeEnd = 0;
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
    NvFlowBufferTransient* aliasBufferTransient(NvFlowContext* context, NvFlowBufferTransient* buffer, NvFlowFormat format, NvFlowUint structureStride);
    NvFlowBufferAcquire* enqueueAcquireBuffer(NvFlowContext* context, NvFlowBufferTransient* buffer);
    NvFlowBool32 getAcquiredBuffer(NvFlowContext* context, NvFlowBufferAcquire* acquire, NvFlowBuffer** outBuffer);
    void* mapBuffer(NvFlowContext* context, NvFlowBuffer* buffer);
    void unmapBuffer(NvFlowContext* context, NvFlowBuffer* buffer);
    NvFlowBufferTransient* getBufferTransientById(NvFlowContext* context, NvFlowUint64 bufferId);
    void getBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, NvFlowInteropHandle* dstHandle);
    void closeBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, const NvFlowInteropHandle* srcHandle);
    NvFlowBuffer* createBufferFromExternalHandle(NvFlowContext* context, const NvFlowBufferDesc* desc, const NvFlowInteropHandle* interopHandle);
    void device_getBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, void* dstHandle, NvFlowUint64 dstHandleSize, NvFlowUint64* pBufferSizeInBytes);
    void device_closeBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, const void* srcHandle, NvFlowUint64 srcHandleSize);

    Buffer* buffer_create(Context* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, const NvFlowInteropHandle* interopHandle);
    void buffer_destroy(Context* context, Buffer* buffer);
    void context_destroyBuffers(Context* context);
    VkBufferView buffer_getBufferView(Context* context, Buffer* ptr, NvFlowFormat aliasFormat);

    struct Texture
    {
        int refCount = 0;
        NvFlowUint64 lastActive = NV_FLOW_FALSE;
        NvFlowTextureDesc desc = {};
        VkImageAspectFlags imageAspect = VK_IMAGE_ASPECT_COLOR_BIT;
        NvFlowUint64 allocationBytes = 0llu;

        VkDeviceMemory memoryVk = VK_NULL_HANDLE;
        VkImage imageVk = VK_NULL_HANDLE;
        VkImageView imageViewVk_mipLevel = VK_NULL_HANDLE;
        VkImageView imageViewVk_all = VK_NULL_HANDLE;
        NvFlowArray<VkImageView> aliasImageViewAlls;
        NvFlowArray<VkImageView> aliasImageViewMipLevels;
        NvFlowArray<NvFlowFormat> aliasFormats;

        VkImageMemoryBarrier restoreBarrier = {};
        VkImageMemoryBarrier currentBarrier = {};
    };

    struct TextureTransient
    {
        NvFlowTextureDesc desc = {};
        Texture* texture = nullptr;
        TextureTransient* aliasTexture = nullptr;
        NvFlowFormat aliasFormat = eNvFlowFormat_unknown;
        int nodeBegin = 0;
        int nodeEnd = 0;
    };

    struct TextureAcquire
    {
        TextureTransient* textureTransient = nullptr;
        Texture* texture = nullptr;
    };

    NvFlowTexture* createTexture(NvFlowContext* context, const NvFlowTextureDesc* desc);
    NvFlowTexture* createTextureExternal(NvFlowContext* context, const NvFlowTextureDesc* desc, VkImage externalImage, VkImageLayout defaultLayout);
    void destroyTexture(NvFlowContext* context, NvFlowTexture* texture);
    NvFlowTextureTransient* getTextureTransient(NvFlowContext* context, const NvFlowTextureDesc* desc);
    NvFlowTextureTransient* registerTextureAsTransient(NvFlowContext* context, NvFlowTexture* texture);
    NvFlowTextureTransient* aliasTextureTransient(NvFlowContext* context, NvFlowTextureTransient* texture, NvFlowFormat format);
    NvFlowTextureAcquire* enqueueAcquireTexture(NvFlowContext* context, NvFlowTextureTransient* texture);
    NvFlowBool32 getAcquiredTexture(NvFlowContext* context, NvFlowTextureAcquire* acquire, NvFlowTexture** outTexture);
    NvFlowTextureTransient* getTextureTransientById(NvFlowContext* context, NvFlowUint64 textureId);

    Texture* texture_create(Context* context, const NvFlowTextureDesc* desc);
    Texture* texture_createExternal(Context* context, const NvFlowTextureDesc* desc, VkImage externalImage, VkImageLayout defaultLayout);
    void texture_destroy(Context* context, Texture* texture);
    void context_destroyTextures(Context* context);
    VkImageView texture_getImageViewAll(Context* context, Texture* ptr, NvFlowFormat aliasFormat);
    VkImageView texture_getImageViewMipLevel(Context* context, Texture* ptr,  NvFlowFormat aliasFormat);

    struct Sampler
    {
        NvFlowBool32 isActive = NV_FLOW_FALSE;
        NvFlowUint64 lastActive = NV_FLOW_FALSE;
        VkSampler sampler = VK_NULL_HANDLE;
    };

    NvFlowSampler* createSampler(NvFlowContext* context, const NvFlowSamplerDesc* desc);
    NvFlowSampler* getDefaultSampler(NvFlowContext* context);
    void destroySampler(NvFlowContext* context, NvFlowSampler* sampler);

    Sampler* sampler_create(Context* context, const NvFlowSamplerDesc* desc);
    void sampler_destroy(Context* context, Sampler* sampler);
    void context_destroySamplers(Context* context);

    struct DescriptorPool
    {
        VkDescriptorPool pool = VK_NULL_HANDLE;
        NvFlowUint allocSetIdx = 0u;
        NvFlowUint64 fenceValue = 0llu;
    };

    struct ComputePipeline
    {
        NvFlowComputePipelineDesc desc = {};
        NvFlowUint totalDescriptors = 0u;

        VkShaderModule module = VK_NULL_HANDLE;

        VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
        VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
        VkPipeline pipeline = VK_NULL_HANDLE;

        NvFlowArray<DescriptorPool, 8u> pools;
        NvFlowUint64 frontIdx = 0u;

        NvFlowArray<VkDescriptorSetLayoutBinding> bindings;

        NvFlowArray<VkWriteDescriptorSet> descriptorWrites;
        NvFlowArray<VkDescriptorBufferInfo> bufferInfos;
        NvFlowArray<VkBufferView> bufferViews;
        NvFlowArray<VkDescriptorImageInfo> imageInfos;

        NvFlowUint poolSizeCount = 0u;
        NvFlowUint setsPerPool = 0u;

        VkDescriptorType flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_count] = {};
        NvFlowUint descriptorCounts[eNvFlowDescriptorType_count] = {};
        VkDescriptorPoolSize poolSizes[eNvFlowDescriptorType_count] = {};
    };

    NvFlowComputePipeline* createComputePipeline(NvFlowContext* context, const NvFlowComputePipelineDesc* desc);
    void destroyComputePipeline(NvFlowContext* context, NvFlowComputePipeline* pipeline);

    VkDescriptorSet computePipeline_allocate(Context* context, ComputePipeline* pipeline);
    void computePipeline_dispatch(Context* context, const NvFlowPassComputeParams* params);

    struct ProfilerEntry
    {
        const char* label;
        NvFlowUint64 cpuValue;
        NvFlowUint64 gpuValue;
    };

    struct ProfilerCapture
    {
        VkQueryPool queryPool = VK_NULL_HANDLE;
        VkBuffer queryBuffer = VK_NULL_HANDLE;
        VkDeviceMemory queryMemory = VK_NULL_HANDLE;
        NvFlowUint64* queryMapped = nullptr;
        NvFlowUint64 queryFrequency = 0u;
        NvFlowUint64 queryReadbackFenceVal = ~0llu;

        NvFlowUint state = 0u;
        NvFlowUint64 captureID = 0llu;
        NvFlowUint64 cpuFreq = 0llu;
        NvFlowUint64 capacity = 0u;

        NvFlowArray<ProfilerEntry> entries;
        NvFlowArray<NvFlowProfilerEntry> deltaEntries;
    };

    void profilerCapture_init(Context* context, ProfilerCapture* ptr, NvFlowUint64 capacity);
    void profilerCapture_destroy(Context* context, ProfilerCapture* ptr);
    void profilerCapture_reset(Context* context, ProfilerCapture* ptr, NvFlowUint64 minCapacity, NvFlowUint64 captureID);
    void profilerCapture_timestamp(Context* context, ProfilerCapture* ptr, const char* label);
    void profilerCapture_download(Context* context, ProfilerCapture* ptr);
    NvFlowBool32 profilerCapture_mapResults(Context* context, ProfilerCapture* ptr, NvFlowUint64* pNumEntries, NvFlowProfilerEntry** pEntries);
    void profilerCapture_unmapResults(Context* context, ProfilerCapture* ptr);

    struct Profiler
    {
        NvFlowArray<ProfilerCapture> captures;
        NvFlowUint64 currentCaptureIndex = 0u;
        NvFlowUint64 currentCaptureID = 0llu;

        void* userdata = nullptr;
        void(NV_FLOW_ABI* reportEntries)(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries) = nullptr;
    };

    Profiler* profiler_create(Context* context);
    void profiler_destroy(Context* context, Profiler* ptr);
    void profiler_beginCapture(Context* context, Profiler* ptr, NvFlowUint64 numEntries);
    void profiler_endCapture(Context* context, Profiler* ptr);
    void profiler_processCaptures(Context* context, Profiler* ptr);
    void profiler_timestamp(Context* context, Profiler* ptr, const char* label);

    void enableProfiler(NvFlowContext* context, void* userdata, void(NV_FLOW_ABI* reportEntries)(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries));
    void disableProfiler(NvFlowContext* context);

    enum ContextNodeType
    {
        eContextNodeType_unknown = 0,
        eContextNodeType_compute = 1,
        eContextNodeType_copyBuffer = 2,
        eContextNodeType_copyBufferToTexture = 3,
        eContextNodeType_copyTextureToBuffer = 4,
        eContextNodeType_copyTexture = 5,

        eContextNodeType_maxEnum = 0x7FFFFFFF
    };

    struct ContextNodeMemoryParams
    {
        unsigned char data[128u];
    };

    union ContextNodeParams
    {
        NvFlowPassComputeParams compute;
        NvFlowPassCopyBufferParams copyBuffer;
        NvFlowPassCopyBufferToTextureParams copyBufferToTexture;
        NvFlowPassCopyTextureToBufferParams copyTextureToBuffer;
        NvFlowPassCopyTextureParams copyTexture;
        ContextNodeMemoryParams memory;
    };

    struct ContextNode
    {
        ContextNodeType type = eContextNodeType_unknown;
        ContextNodeParams params = {};
        const char* label = "Unknown";
        NvFlowArray<NvFlowDescriptorWrite> descriptorWrites;
        NvFlowArray<NvFlowResource> resources;

        NvFlowArray<BufferTransient*> bufferTransientsCreate;
        NvFlowArray<TextureTransient*> textureTransientsCreate;
        NvFlowArray<BufferTransient*> bufferTransientsDestroy;
        NvFlowArray<TextureTransient*> textureTransientsDestroy;
        NvFlowArray<VkBufferMemoryBarrier> bufferBarriers;
        NvFlowArray<VkImageMemoryBarrier> imageBarriers;
    };

    struct RegisteredResource
    {
        NvFlowBuffer* buffer;
        NvFlowTexture* texture;
        NvFlowUint64 uid;
    };

    struct Context
    {
        DeviceQueue* deviceQueue = nullptr;

        NvFlowArrayPointer<Buffer*> pool_buffers;
        NvFlowArrayPointer<Texture*> pool_textures;
        NvFlowArrayPointer<Sampler*> pool_samplers;

        NvFlowArrayPointer<BufferTransient*> bufferTransients;
        NvFlowArrayPointer<TextureTransient*> textureTransients;

        NvFlowArrayPointer<BufferAcquire*> bufferAcquires;
        NvFlowArrayPointer<TextureAcquire*> textureAcquires;

        NvFlowArray<ContextNode> nodes;

        NvFlowArray<VkBufferMemoryBarrier> restore_bufferBarriers;
        NvFlowArray<VkImageMemoryBarrier> restore_imageBarriers;

        Profiler* profiler = nullptr;

        NvFlowUint64 minLifetime = 60u;

        NvFlowUint64 registeredResourceCounter = 0llu;
        NvFlowArray<RegisteredResource> registeredResources;

        NvFlowLogPrint_t logPrint = nullptr;

        NvFlowThreadPoolInterface threadPoolInterface = {};
        NvFlowThreadPool* threadPool = nullptr;
        NvFlowUint threadCount = 0u;
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
    void context_destroy(Context* context);
    void context_resetNodes(Context* context);
    void context_flushNodes(Context* context);

    /// Format conversion

    struct FormatConverter;

    FormatConverter* formatConverter_create();
    void formatConverter_destroy(FormatConverter* converter);
    VkFormat formatConverter_convertToVulkan(FormatConverter* converter, NvFlowFormat format);
    NvFlowFormat formatConverter_convertToNvFlow(FormatConverter* converter, VkFormat format);
    NvFlowUint formatConverter_getFormatSizeInBytes(FormatConverter* converter, NvFlowFormat format);

    NV_FLOW_INLINE uint32_t context_getMemoryType(Context* context, uint32_t typeBits, VkMemoryPropertyFlags properties)
    {
        for (uint32_t i = 0u; i < context->deviceQueue->device->memoryProperties.memoryTypeCount; i++)
        {
            if ((typeBits & 1) == 1)
            {
                if ((context->deviceQueue->device->memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
                {
                    return i;
                }
            }
            typeBits >>= 1u;
        }
        return ~0u;
    }

    /// Utils

    struct ExtensionRequest
    {
        const char* name;
        NvFlowBool32* pEnabled;
    };

    void determineMatches(NvFlowArray<const char*, 8u>& extensionsEnabled, NvFlowArray<ExtensionRequest, 8u>& extensionsRequest, NvFlowArray<VkExtensionProperties>& extensions);

    void selectInstanceExtensions(DeviceManager* ptr, NvFlowArray<const char*, 8u>& instanceExtensionsEnabled);

    void selectDeviceExtensions(Device* ptr, NvFlowArray<const char*, 8u>& deviceExtensionsEnabled);
}
