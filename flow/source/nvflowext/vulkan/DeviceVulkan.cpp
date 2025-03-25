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

#include "CommonVulkan.h"

#if defined(_WIN32)

#else
#include <unistd.h>
#endif

#include <string.h>

NvFlowContextInterface* NvFlowGetContextInterface_vulkan();

namespace NvFlowVulkan
{
/// ************************** Device Manager **************************************

NvFlowDeviceManager* createDeviceManager(NvFlowBool32 enableValidationOnDebugBuild, NvFlowThreadPoolInterface* threadPoolInterface, NvFlowUint threadCount)
{
    auto ptr = new DeviceManager();

    ptr->vulkan_module = NvFlowLoadLibrary("vulkan-1.dll", "libvulkan.so.1");

    auto getInstanceProcAddr = (PFN_vkGetInstanceProcAddr)NvFlowGetProcAddress(ptr->vulkan_module, "vkGetInstanceProcAddr");

    NvFlowVulkanLoader_global(&ptr->loader, getInstanceProcAddr);

    auto loader = &ptr->loader;

    // select extensions
    NvFlowArray<const char*, 8u> instanceExtensionsEnabled;
    selectInstanceExtensions(ptr, instanceExtensionsEnabled);

    // create instance
    uint32_t numLayers = 0u;
    const char** layers = nullptr;
#ifdef _DEBUG
    const uint32_t numLayers_validation = 1u;
    const char* layers_validation[numLayers_validation] = {
        "VK_LAYER_KHRONOS_validation"
    };
    if (enableValidationOnDebugBuild)
    {
        numLayers = numLayers_validation;
        layers = layers_validation;
    }
#endif

    VkInstanceCreateInfo instanceCreateInfo = {};
    instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instanceCreateInfo.pNext = nullptr;
    instanceCreateInfo.enabledLayerCount = numLayers;
    instanceCreateInfo.ppEnabledLayerNames = layers;
    instanceCreateInfo.enabledExtensionCount = (uint32_t)instanceExtensionsEnabled.size;
    instanceCreateInfo.ppEnabledExtensionNames = instanceExtensionsEnabled.data;

    loader->vkCreateInstance(&instanceCreateInfo, nullptr, &ptr->vulkanInstance);

    // for non-dev systems, gracefully fall back to no validation layers
    if (!ptr->vulkanInstance)
    {
        instanceCreateInfo.enabledLayerCount = 0u;
        instanceCreateInfo.ppEnabledLayerNames = nullptr;

        loader->vkCreateInstance(&instanceCreateInfo, nullptr, &ptr->vulkanInstance);
    }

    NvFlowVulkanLoader_instance(&ptr->loader, ptr->vulkanInstance);

    // enumerate devices
    {
        uint32_t gpuCount = 0u;
        loader->vkEnumeratePhysicalDevices(ptr->vulkanInstance, &gpuCount, nullptr);

        ptr->physicalDevices.reserve(gpuCount);
        ptr->deviceProps.reserve(gpuCount);
        ptr->physicalDeviceDescs.reserve(gpuCount);

        ptr->physicalDevices.size = gpuCount;
        ptr->deviceProps.size = gpuCount;
        ptr->physicalDeviceDescs.size = gpuCount;

        loader->vkEnumeratePhysicalDevices(ptr->vulkanInstance, (uint32_t*)&ptr->physicalDevices.size, ptr->physicalDevices.data);

        for (uint32_t i = 0; i < gpuCount; i++)
        {
            if (ptr->enabledExtensions.VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2 && ptr->enabledExtensions.VK_KHR_EXTERNAL_FENCE_CAPABILITIES)
            {
                VkPhysicalDeviceIDPropertiesKHR deviceIdVulkan = {};
                deviceIdVulkan.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES_KHR;

                VkPhysicalDeviceProperties2KHR devicePropsVulkan = {};
                devicePropsVulkan.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2_KHR;
                devicePropsVulkan.pNext = &deviceIdVulkan;

                loader->vkGetPhysicalDeviceProperties2KHR(ptr->physicalDevices[i], &devicePropsVulkan);

                NvFlowPhysicalDeviceDesc physicalDeviceDesc = {};
                NvFlowUint64 uuidNumBytes =
                    sizeof(deviceIdVulkan.deviceUUID) < sizeof(physicalDeviceDesc.deviceUUID) ?
                    sizeof(deviceIdVulkan.deviceUUID) : sizeof(physicalDeviceDesc.deviceUUID);
                for (NvFlowUint uuidIdx = 0u; uuidIdx < uuidNumBytes; uuidIdx++)
                {
                    physicalDeviceDesc.deviceUUID[uuidIdx] = deviceIdVulkan.deviceUUID[uuidIdx];
                }
                NvFlowUint64 luidNumBytes =
                    sizeof(deviceIdVulkan.deviceLUID) < sizeof(physicalDeviceDesc.deviceLUID) ?
                    sizeof(deviceIdVulkan.deviceLUID) : sizeof(physicalDeviceDesc.deviceLUID);
                for (NvFlowUint luidIdx = 0u; luidIdx < luidNumBytes; luidIdx++)
                {
                    physicalDeviceDesc.deviceLUID[luidIdx] = deviceIdVulkan.deviceLUID[luidIdx];
                }
                physicalDeviceDesc.deviceNodeMask = deviceIdVulkan.deviceNodeMask;
                physicalDeviceDesc.deviceLUIDValid = deviceIdVulkan.deviceLUIDValid;

                ptr->deviceProps[i] = devicePropsVulkan.properties;
                ptr->physicalDeviceDescs[i] = physicalDeviceDesc;
            }
            else
            {
                NvFlowPhysicalDeviceDesc physicalDeviceDesc = {};

                loader->vkGetPhysicalDeviceProperties(ptr->physicalDevices[i], &ptr->deviceProps[i]);
                ptr->physicalDeviceDescs[i] = physicalDeviceDesc;
            }
        }
    }

    return cast(ptr);
}

void destroyDeviceManager(NvFlowDeviceManager* deviceManager)
{
    auto ptr = cast(deviceManager);

    ptr->loader.vkDestroyInstance(ptr->vulkanInstance, nullptr);

    NvFlowFreeLibrary(ptr->vulkan_module);
    ptr->vulkan_module = nullptr;

    delete ptr;
}

NvFlowBool32 enumerateDevices(NvFlowDeviceManager* manager, NvFlowUint deviceIndex, NvFlowPhysicalDeviceDesc* pDesc)
{
    auto ptr = cast(manager);
    if (deviceIndex < ptr->physicalDeviceDescs.size)
    {
        *pDesc = ptr->physicalDeviceDescs[deviceIndex];
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

/// ************************** Device **************************************

void logDefault(NvFlowLogLevel level, const char* format, ...)
{
    // NOP
}

NvFlowDevice* createDevice(NvFlowDeviceManager* deviceManagerIn, const NvFlowDeviceDesc* desc)
{
    auto deviceManager = cast(deviceManagerIn);

    NvFlowPhysicalDeviceDesc physicalDeviceDesc = {};
    if (!enumerateDevices(deviceManagerIn, desc->deviceIndex, &physicalDeviceDesc))
    {
        return nullptr;
    }

    auto ptr = new Device();

    ptr->desc = *desc;
    if (desc->logPrint)
    {
        ptr->logPrint = desc->logPrint;
    }
    else
    {
        ptr->logPrint = logDefault;
    }

    ptr->deviceManager = deviceManager;
    ptr->formatConverter = formatConverter_create();

    auto instanceLoader = &ptr->deviceManager->loader;
    auto deviceLoader = &ptr->loader;

    // set physical device
    {
        ptr->physicalDevice = deviceManager->physicalDevices[desc->deviceIndex];
    }

    // identify graphics queue
    {
        NvFlowArray<VkQueueFamilyProperties, 8u> queueProps;

        uint32_t queueCount = 0u;
        instanceLoader->vkGetPhysicalDeviceQueueFamilyProperties(ptr->physicalDevice, &queueCount, nullptr);

        queueProps.reserve(queueCount);
        queueProps.size = queueCount;

        instanceLoader->vkGetPhysicalDeviceQueueFamilyProperties(ptr->physicalDevice, (uint32_t*)&queueProps.size, queueProps.data);

        uint32_t graphicsQueueIdx = 0u;
        for (uint32_t i = 0u; i < queueCount; i++)
        {
            if (queueProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
            {
                graphicsQueueIdx = i;
                break;
            }
        }
        ptr->graphicsQueueIdx = graphicsQueueIdx;
    }

    NvFlowArray<const char*, 8u> deviceExtensionsEnabled;
    selectDeviceExtensions(ptr, deviceExtensionsEnabled);

    // create device
    {
        const float queuePriorities[1] = { 1.f };

        VkDeviceQueueCreateInfo queueCreateInfo = {};
        queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueCreateInfo.queueFamilyIndex = ptr->graphicsQueueIdx;
        queueCreateInfo.queueCount = 1u;
        queueCreateInfo.pQueuePriorities = queuePriorities;

        VkDeviceCreateInfo deviceCreateInfo = {};
        deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        deviceCreateInfo.queueCreateInfoCount = 1u;
        deviceCreateInfo.pQueueCreateInfos = &queueCreateInfo;
        deviceCreateInfo.enabledExtensionCount = (uint32_t)deviceExtensionsEnabled.size;
        deviceCreateInfo.ppEnabledExtensionNames = deviceExtensionsEnabled.data;

        VkPhysicalDeviceFeatures features = {};
        instanceLoader->vkGetPhysicalDeviceFeatures(ptr->physicalDevice, &features);

        VkPhysicalDeviceFeatures enabledFeaturesVk = {};

#define NV_FLOW_VULKAN_TRY_ENABLE_FEATURE(X) \
        if (features.X) \
        { \
            ptr->enabledFeatures.X = NV_FLOW_TRUE; \
            enabledFeaturesVk.X = VK_TRUE; \
        }

        NV_FLOW_VULKAN_TRY_ENABLE_FEATURE(shaderStorageImageWriteWithoutFormat)

#undef NV_FLOW_VULKAN_TRY_ENABLE_FEATURE

        deviceCreateInfo.pEnabledFeatures = &enabledFeaturesVk;

        instanceLoader->vkCreateDevice(ptr->physicalDevice, &deviceCreateInfo, nullptr, &ptr->vulkanDevice);

        NvFlowVulkanLoader_device(&ptr->loader, ptr->vulkanDevice, instanceLoader->vkGetDeviceProcAddr);
    }

    // get properties
    instanceLoader->vkGetPhysicalDeviceProperties(ptr->physicalDevice, &ptr->physicalDeviceProperties);
    instanceLoader->vkGetPhysicalDeviceMemoryProperties(ptr->physicalDevice, &ptr->memoryProperties);

    // get graphics queue
    deviceLoader->vkGetDeviceQueue(ptr->vulkanDevice, ptr->graphicsQueueIdx, 0u, &ptr->graphicsQueue);

    ptr->deviceQueue = deviceQueue_create(ptr);

    return cast(ptr);
}

void destroyDevice(NvFlowDeviceManager* manager, NvFlowDevice* device)
{
    auto ptr = cast(device);

    deviceQueue_destroy(ptr->deviceQueue);

    ptr->loader.vkDestroyDevice(ptr->vulkanDevice, nullptr);

    formatConverter_destroy(ptr->formatConverter);

    delete ptr;
}

NvFlowDeviceQueue* getDeviceQueue(NvFlowDevice* device)
{
    auto ptr = cast(device);
    return cast(ptr->deviceQueue);
}

void getMemoryStats(NvFlowDevice* device, NvFlowDeviceMemoryStats* dstStats)
{
    auto ptr = cast(device);
    if (dstStats)
    {
        *dstStats = ptr->memoryStats;
    }
}

void device_reportMemoryAllocate(Device* device, NvFlowMemoryType type, NvFlowUint64 bytes)
{
    if (type == eNvFlowMemoryType_device)
    {
        device->memoryStats.deviceMemoryBytes += bytes;
    }
    else if (type == eNvFlowMemoryType_upload)
    {
        device->memoryStats.uploadMemoryBytes += bytes;
    }
    else if (type == eNvFlowMemoryType_readback)
    {
        device->memoryStats.readbackMemoryBytes += bytes;
    }
}

void device_reportMemoryFree(Device* device, NvFlowMemoryType type, NvFlowUint64 bytes)
{
    if (type == eNvFlowMemoryType_device)
    {
        device->memoryStats.deviceMemoryBytes -= bytes;
    }
    else if (type == eNvFlowMemoryType_upload)
    {
        device->memoryStats.uploadMemoryBytes -= bytes;
    }
    else if (type == eNvFlowMemoryType_readback)
    {
        device->memoryStats.readbackMemoryBytes -= bytes;
    }
}

/// ************************** DeviceSemaphore **************************************

NvFlowDeviceSemaphore* createSemaphore(NvFlowDevice* device)
{
    auto ptr = new DeviceSemaphore();

    ptr->device = cast(device);

    VkSemaphoreCreateInfo semaphoreCreateInfo = {};
    semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkExportSemaphoreCreateInfoKHR exportSemaphoreCreateInfo = {};
    exportSemaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_EXPORT_SEMAPHORE_CREATE_INFO_KHR;
#if defined(_WIN32)
    exportSemaphoreCreateInfo.handleTypes = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR;
#else
    exportSemaphoreCreateInfo.handleTypes = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#endif
    semaphoreCreateInfo.pNext = &exportSemaphoreCreateInfo;

    ptr->device->loader.vkCreateSemaphore(
        ptr->device->vulkanDevice, &semaphoreCreateInfo, nullptr, &ptr->semaphoreVk);

    return cast(ptr);
}

void destroySemaphore(NvFlowDeviceSemaphore* semaphore)
{
    auto ptr = cast(semaphore);

    ptr->device->loader.vkDestroySemaphore(
        ptr->device->vulkanDevice, ptr->semaphoreVk, nullptr);

    delete ptr;
}

void getSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, void* dstHandle, NvFlowUint64 dstHandleSize)
{
    auto ptr = cast(semaphore);
#if defined(_WIN32)
    HANDLE handle = {};
    VkSemaphoreGetWin32HandleInfoKHR handleInfo = {};
    handleInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_GET_WIN32_HANDLE_INFO_KHR;
    handleInfo.semaphore = ptr->semaphoreVk;
    handleInfo.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR;

    ptr->device->loader.vkGetSemaphoreWin32HandleKHR(ptr->device->vulkanDevice, &handleInfo, &handle);

    memset(dstHandle, 0, dstHandleSize);
    if (dstHandleSize >= sizeof(handle))
    {
        memcpy(dstHandle, &handle, sizeof(handle));
    }
#else
    int fd = 0;
    VkSemaphoreGetFdInfoKHR handleInfo = {};
    handleInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_GET_FD_INFO_KHR;
    handleInfo.semaphore = ptr->semaphoreVk;
    handleInfo.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    ptr->device->loader.vkGetSemaphoreFdKHR(ptr->device->vulkanDevice, &handleInfo, &fd);

    memset(dstHandle, 0, dstHandleSize);
    if (dstHandleSize >= sizeof(fd))
    {
        memcpy(dstHandle, &fd, sizeof(fd));
    }
#endif
}

void closeSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, const void* srcHandle, NvFlowUint64 srcHandleSize)
{
    auto ptr = cast(semaphore);
#if defined(_WIN32)
    HANDLE handle = {};
    if (srcHandleSize >= sizeof(handle))
    {
        memcpy(&handle, srcHandle, sizeof(handle));

        CloseHandle(handle);
    }
#else
    int fd = 0;
    if (srcHandleSize >= sizeof(fd))
    {
        memcpy(&fd, srcHandle, sizeof(fd));

        close(fd);
    }
#endif
}

/// ************************** DeviceQueue **************************************

DeviceQueue* deviceQueue_create(Device* device)
{
    auto ptr = new DeviceQueue();

    ptr->device = device;

    ptr->vulkanDevice = ptr->device->vulkanDevice;
    ptr->graphicsQueue = ptr->device->graphicsQueue;

    auto loader = &ptr->device->loader;

    VkCommandPoolCreateInfo poolCreateInfo = {};
    poolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolCreateInfo.queueFamilyIndex = ptr->device->graphicsQueueIdx;

    loader->vkCreateCommandPool(ptr->vulkanDevice, &poolCreateInfo, nullptr, &ptr->commandPool);

    VkCommandBufferAllocateInfo commandInfo = {};
    commandInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    commandInfo.commandPool = ptr->commandPool;
    commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    commandInfo.commandBufferCount = 1u;

    VkFenceCreateInfo fenceCreateInfo = {};
    fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceCreateInfo.flags = 0u; // VK_FENCE_CREATE_SIGNALED_BIT;

    for (int i = 0; i < kMaxFramesInFlight; i++)
    {
        loader->vkAllocateCommandBuffers(ptr->vulkanDevice, &commandInfo, &ptr->commandBuffers[i]);

        loader->vkCreateFence(ptr->vulkanDevice, &fenceCreateInfo, nullptr, &ptr->fences[i].fence);
        ptr->fences[i].active = NV_FLOW_FALSE;
        ptr->fences[i].value = 0u;
    }

    VkSemaphoreCreateInfo semaphoreCreateInfo = {};
    semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    loader->vkCreateSemaphore(ptr->vulkanDevice, &semaphoreCreateInfo, nullptr, &ptr->beginFrameSemaphore);
    loader->vkCreateSemaphore(ptr->vulkanDevice, &semaphoreCreateInfo, nullptr, &ptr->endFrameSemaphore);

    // Second step of flush to prime command buffer
    flushStepB(ptr);

    ptr->context = context_create(ptr);

    return ptr;
}

void deviceQueue_destroy(DeviceQueue* ptr)
{
    auto loader = &ptr->device->loader;

    // Wait idle, since context destroy will force destroy resources
    waitIdle(cast(ptr));

    context_destroy(ptr->context);

    loader->vkDestroySemaphore(ptr->vulkanDevice, ptr->beginFrameSemaphore, nullptr);
    loader->vkDestroySemaphore(ptr->vulkanDevice, ptr->endFrameSemaphore, nullptr);

    for (int i = 0; i < kMaxFramesInFlight; i++)
    {
        loader->vkFreeCommandBuffers(ptr->vulkanDevice, ptr->commandPool, 1u, &ptr->commandBuffers[i]);
        loader->vkDestroyFence(ptr->vulkanDevice, ptr->fences[i].fence, nullptr);
    }
    loader->vkDestroyCommandPool(ptr->vulkanDevice, ptr->commandPool, nullptr);

    ptr->device = nullptr;

    delete ptr;
}

int flushStepA(DeviceQueue* ptr, NvFlowDeviceSemaphore* waitSemaphore, NvFlowDeviceSemaphore* signalSemaphore)
{
    auto loader = &ptr->device->loader;

    if (ptr->context)
    {
        context_flushNodes(ptr->context);
    }

    loader->vkEndCommandBuffer(ptr->commandBuffer);

    VkPipelineStageFlags stageFlags = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;

    VkSemaphore waitSemaphores[2u] = {};
    VkSemaphore signalSemaphores[2u] = {};

    VkSubmitInfo submitInfo = {};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.waitSemaphoreCount = 0u;
    if (ptr->currentBeginFrameSemaphore)
    {
        waitSemaphores[submitInfo.waitSemaphoreCount] = ptr->currentBeginFrameSemaphore;
        submitInfo.waitSemaphoreCount++;
        submitInfo.pWaitSemaphores = waitSemaphores;
    }
    if (waitSemaphore)
    {
        waitSemaphores[submitInfo.waitSemaphoreCount] = cast(waitSemaphore)->semaphoreVk;
        submitInfo.waitSemaphoreCount++;
        submitInfo.pWaitSemaphores = waitSemaphores;
    }
    submitInfo.pWaitDstStageMask = &stageFlags;
    submitInfo.commandBufferCount = 1u;
    submitInfo.pCommandBuffers = &ptr->commandBuffer;
    submitInfo.signalSemaphoreCount = 0u;
    if (ptr->currentEndFrameSemaphore)
    {
        signalSemaphores[submitInfo.signalSemaphoreCount] = ptr->currentEndFrameSemaphore;
        submitInfo.signalSemaphoreCount++;
        submitInfo.pSignalSemaphores = signalSemaphores;
    }
    if (signalSemaphore)
    {
        signalSemaphores[submitInfo.signalSemaphoreCount] = cast(signalSemaphore)->semaphoreVk;
        submitInfo.signalSemaphoreCount++;
        submitInfo.pSignalSemaphores = signalSemaphores;
    }

    VkResult result = loader->vkQueueSubmit(ptr->graphicsQueue, 1u, &submitInfo, ptr->fences[ptr->commandBufferIdx].fence);

    // mark signaled fence value
    ptr->fences[ptr->commandBufferIdx].value = ptr->nextFenceValue;
    ptr->fences[ptr->commandBufferIdx].active = NV_FLOW_TRUE;

    // increment fence value
    ptr->nextFenceValue++;

    ptr->currentBeginFrameSemaphore = VK_NULL_HANDLE;

    return (result == VK_ERROR_DEVICE_LOST) ? 1 : 0;
}

void deviceQueue_fenceUpdate(DeviceQueue* ptr, NvFlowUint fenceIdx, NvFlowBool32 blocking)
{
    if (ptr->fences[fenceIdx].active)
    {
        auto loader = &ptr->device->loader;

        uint64_t timeout = blocking ? ~0llu : 0llu;

        if (VK_SUCCESS == loader->vkWaitForFences(ptr->vulkanDevice, 1u, &ptr->fences[fenceIdx].fence, VK_TRUE, timeout))
        {
            loader->vkResetFences(ptr->vulkanDevice, 1u, &ptr->fences[fenceIdx].fence);

            ptr->fences[fenceIdx].active = NV_FLOW_FALSE;
            if (ptr->fences[fenceIdx].value > ptr->lastFenceCompleted)
            {
                ptr->lastFenceCompleted = ptr->fences[fenceIdx].value;
            }
        }
    }
}

void flushStepB(DeviceQueue* ptr)
{
    auto loader = &ptr->device->loader;

    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    ptr->commandBufferIdx = (ptr->commandBufferIdx + 1) % kMaxFramesInFlight;
    ptr->commandBuffer = ptr->commandBuffers[ptr->commandBufferIdx];

    // non-blocking update of fence values
    for (NvFlowUint fenceIdx = 0u; fenceIdx < kMaxFramesInFlight; fenceIdx++)
    {
        deviceQueue_fenceUpdate(ptr, fenceIdx, NV_FLOW_FALSE);
    }

    // blocking update of fence values
    deviceQueue_fenceUpdate(ptr, ptr->commandBufferIdx, NV_FLOW_TRUE);

    loader->vkResetCommandBuffer(ptr->commandBuffer, 0);

    //loader->vkResetCommandPool(ptr->vulkanDevice, ptr->commandPool, 0);

    loader->vkBeginCommandBuffer(ptr->commandBuffer, &beginInfo);

    if (ptr->context)
    {
        context_resetNodes(ptr->context);
    }
}

int flush(NvFlowDeviceQueue* deviceQueue, NvFlowUint64* flushedFrameID, NvFlowDeviceSemaphore* waitSemaphore, NvFlowDeviceSemaphore* signalSemaphore)
{
    auto ptr = cast(deviceQueue);

    *flushedFrameID = ptr->nextFenceValue;

    int ret = flushStepA(ptr, waitSemaphore, signalSemaphore);
    flushStepB(ptr);

    return ret;
}

void waitIdle(NvFlowDeviceQueue* deviceQueue)
{
    auto ptr = cast(deviceQueue);

    ptr->device->loader.vkQueueWaitIdle(ptr->graphicsQueue);

    for (NvFlowUint fenceIdx = 0u; fenceIdx < kMaxFramesInFlight; fenceIdx++)
    {
        deviceQueue_fenceUpdate(ptr, fenceIdx, NV_FLOW_TRUE);
    }

    // update internal context
}

void waitForFrame(NvFlowDeviceQueue* deviceQueue, NvFlowUint64 frameID)
{
    auto ptr = cast(deviceQueue);

    // avoid waiting on future frames
    if (frameID >= ptr->nextFenceValue)
    {
        return;
    }

    while (ptr->lastFenceCompleted < frameID)
    {
        NvFlowUint64 minFenceValue = 0llu;
        NvFlowUint minFenceIdx = 0u;
        for (NvFlowUint fenceIdx = 0u; fenceIdx < kMaxFramesInFlight; fenceIdx++)
        {
            if (ptr->fences[fenceIdx].active)
            {
                NvFlowUint64 frameFenceValue = ptr->fences[fenceIdx].value;
                if (minFenceValue == 0 || frameFenceValue < minFenceValue)
                {
                    minFenceValue = frameFenceValue;
                    minFenceIdx = fenceIdx;
                }
            }
        }
        // wait for min frame
        if (minFenceValue > 0u)
        {
            deviceQueue_fenceUpdate(ptr, minFenceIdx, NV_FLOW_TRUE);
        }
    }

    // update internal context
}

NvFlowUint64 getLastFrameCompleted(NvFlowDeviceQueue* queue)
{
    auto ptr = cast(queue);
    return ptr->lastFenceCompleted;
}

NvFlowContextInterface* getContextInterface(NvFlowDeviceQueue* ptr)
{
    return NvFlowGetContextInterface_vulkan();
}

NvFlowContext* getContext(NvFlowDeviceQueue* queue)
{
    auto deviceQueue = cast(queue);

    return cast(deviceQueue->context);
}

/// ************************** Swapchain **************************************

NvFlowSwapchain* createSwapchain(NvFlowDeviceQueue* queue, const NvFlowSwapchainDesc* desc)
{
    auto ptr = new Swapchain();

    ptr->desc = *desc;
    ptr->deviceQueue = cast(queue);

    auto device = ptr->deviceQueue->device;
    auto instanceLoader = &ptr->deviceQueue->device->deviceManager->loader;
    auto deviceLoader = &ptr->deviceQueue->device->loader;

#if defined(_WIN32)
    VkWin32SurfaceCreateInfoKHR surfaceCreateInfo = {};
    surfaceCreateInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
    surfaceCreateInfo.hinstance = ptr->desc.hinstance;
    surfaceCreateInfo.hwnd = ptr->desc.hwnd;

    instanceLoader->vkCreateWin32SurfaceKHR(device->deviceManager->vulkanInstance, &surfaceCreateInfo, nullptr, &ptr->surfaceVulkan);
#else
    VkXlibSurfaceCreateInfoKHR surfaceCreateInfo = {};
    surfaceCreateInfo.sType = VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR;
    surfaceCreateInfo.dpy = ptr->desc.dpy;
    surfaceCreateInfo.window = ptr->desc.window;

    instanceLoader->vkCreateXlibSurfaceKHR(device->deviceManager->vulkanInstance, &surfaceCreateInfo, nullptr, &ptr->surfaceVulkan);
#endif

    VkBool32 supported = false;
    instanceLoader->vkGetPhysicalDeviceSurfaceSupportKHR(device->physicalDevice, device->graphicsQueueIdx, ptr->surfaceVulkan, &supported);

    uint32_t surfaceCount = 0;
    VkSurfaceFormatKHR surfaceFormats[32u] = {};
    instanceLoader->vkGetPhysicalDeviceSurfaceFormatsKHR(device->physicalDevice, ptr->surfaceVulkan, &surfaceCount, nullptr);
    if (surfaceCount > 32u) surfaceCount = 32u;
    instanceLoader->vkGetPhysicalDeviceSurfaceFormatsKHR(device->physicalDevice, ptr->surfaceVulkan, &surfaceCount, surfaceFormats);

    ptr->swapchainFormat = formatConverter_convertToVulkan(ptr->deviceQueue->device->formatConverter, ptr->desc.format);

    return cast(ptr);
}

void destroySwapchain(NvFlowSwapchain* swapchain)
{
    auto ptr = cast(swapchain);
    auto device = ptr->deviceQueue->device;
    auto instanceLoader = &ptr->deviceQueue->device->deviceManager->loader;
    auto deviceLoader = &ptr->deviceQueue->device->loader;

    swapchain_destroySwapchain(ptr);

    if (ptr->surfaceVulkan)
    {
        instanceLoader->vkDestroySurfaceKHR(device->deviceManager->vulkanInstance, ptr->surfaceVulkan, nullptr);
        ptr->surfaceVulkan = VK_NULL_HANDLE;
    }

#if defined(_WIN32)
#else
    if (ptr->moduleX11)
    {
        NvFlowFreeLibrary(ptr->moduleX11);
    }
#endif

    delete ptr;
}

void swapchain_getWindowSize(Swapchain* ptr, NvFlowUint* width, NvFlowUint* height)
{
#if defined(_WIN32)
    RECT rc;
    GetClientRect(ptr->desc.hwnd, &rc);
    *width = rc.right - rc.left;
    *height = rc.bottom - rc.top;
#else
    if (!ptr->moduleX11)
    {
        ptr->moduleX11 = NvFlowLoadLibrary("X11.dll", "libX11.so");
        ptr->getWindowAttrib = (decltype(&XGetWindowAttributes))NvFlowGetProcAddress(ptr->moduleX11, "XGetWindowAttributes");
    }

    XWindowAttributes winAttr = {};
    ptr->getWindowAttrib(ptr->desc.dpy, ptr->desc.window, &winAttr);

    *width = winAttr.width;
    *height = winAttr.height;
#endif
}

void swapchain_initSwapchain(Swapchain* ptr)
{
    auto device = ptr->deviceQueue->device;
    auto instanceLoader = &ptr->deviceQueue->device->deviceManager->loader;
    auto deviceLoader = &ptr->deviceQueue->device->loader;

    NvFlowUint width = 0u;
    NvFlowUint height = 0u;
    swapchain_getWindowSize(ptr, &width, &height);

    VkExtent2D imageExtent = {};
    imageExtent.width = width;
    imageExtent.height = height;

    ptr->width = width;
    ptr->height = height;

    // catch this before throwing error
    if (ptr->width == 0 || ptr->height == 0)
    {
        ptr->valid = NV_FLOW_FALSE;
        return;
    }

    VkSurfaceCapabilitiesKHR surfaceCaps = {};
    instanceLoader->vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device->physicalDevice, ptr->surfaceVulkan, &surfaceCaps);

    // clamp width and height based on surfaceCaps
    if (ptr->width < int(surfaceCaps.minImageExtent.width))
    {
        ptr->width = surfaceCaps.minImageExtent.width;
    }
    if (ptr->height < int(surfaceCaps.minImageExtent.height))
    {
        ptr->height = surfaceCaps.minImageExtent.height;
    }
    if (ptr->width > int(surfaceCaps.maxImageExtent.width))
    {
        ptr->width = surfaceCaps.maxImageExtent.width;
    }
    if (ptr->height > int(surfaceCaps.maxImageExtent.height))
    {
        ptr->height = surfaceCaps.maxImageExtent.height;
    }

    VkPresentModeKHR presentModes[8u] = {};
    uint32_t numPresentModes = 0u;
    instanceLoader->vkGetPhysicalDeviceSurfacePresentModesKHR(device->physicalDevice, ptr->surfaceVulkan, &numPresentModes, nullptr);
    if (numPresentModes > 8u) numPresentModes = 8u;
    instanceLoader->vkGetPhysicalDeviceSurfacePresentModesKHR(device->physicalDevice, ptr->surfaceVulkan, &numPresentModes, presentModes);

    VkPresentModeKHR presentOptions[3u] = { VK_PRESENT_MODE_IMMEDIATE_KHR, VK_PRESENT_MODE_MAILBOX_KHR, VK_PRESENT_MODE_FIFO_KHR };
    if (ptr->vsync)
    {
        presentOptions[0u] = VK_PRESENT_MODE_FIFO_KHR;
        presentOptions[1u] = VK_PRESENT_MODE_IMMEDIATE_KHR;
        presentOptions[2u] = VK_PRESENT_MODE_MAILBOX_KHR;
    }
    for (int j = 0; j < 3; j++)
    {
        for (uint32_t i = 0; i < numPresentModes; i++)
        {
            if (presentModes[i] == presentOptions[j])
            {
                ptr->presentMode = presentOptions[j];
                j = 3;
                break;
            }
        }
    }

    VkSwapchainKHR oldSwapchain = VK_NULL_HANDLE;

    VkSwapchainCreateInfoKHR swapchainDesc = {};
    swapchainDesc.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    swapchainDesc.surface = ptr->surfaceVulkan;
    swapchainDesc.minImageCount = 3;
    swapchainDesc.imageFormat = ptr->swapchainFormat;
    swapchainDesc.imageColorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    swapchainDesc.imageExtent = imageExtent;
    swapchainDesc.imageArrayLayers = 1;
    swapchainDesc.imageUsage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    swapchainDesc.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    swapchainDesc.preTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
    swapchainDesc.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapchainDesc.presentMode = ptr->presentMode;
    swapchainDesc.clipped = VK_TRUE;
    swapchainDesc.oldSwapchain = oldSwapchain;

    VkResult result = deviceLoader->vkCreateSwapchainKHR(device->vulkanDevice, &swapchainDesc, nullptr, &ptr->swapchainVulkan);

    if (result != VK_SUCCESS)
    {
        ptr->valid = NV_FLOW_FALSE;
        return;
    }

    ptr->numSwapchainImages = 0u;
    deviceLoader->vkGetSwapchainImagesKHR(device->vulkanDevice, ptr->swapchainVulkan, &ptr->numSwapchainImages, nullptr);
    if (ptr->numSwapchainImages > kMaxSwapchainImages)
    {
        ptr->numSwapchainImages = kMaxSwapchainImages;
    }
    deviceLoader->vkGetSwapchainImagesKHR(device->vulkanDevice, ptr->swapchainVulkan, &ptr->numSwapchainImages, ptr->swapchainImages);

    for (NvFlowUint idx = 0; idx < ptr->numSwapchainImages; idx++)
    {
        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_textureCopyDst;
        texDesc.format = ptr->desc.format;
        texDesc.width = imageExtent.width;
        texDesc.height = imageExtent.height;

        ptr->textures[idx] = createTextureExternal(cast(ptr->deviceQueue->context), &texDesc, ptr->swapchainImages[idx], VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    }

    ptr->valid = NV_FLOW_TRUE;
}

void swapchain_destroySwapchain(Swapchain* ptr)
{
    auto device = ptr->deviceQueue->device;
    auto instanceLoader = &ptr->deviceQueue->device->deviceManager->loader;
    auto deviceLoader = &ptr->deviceQueue->device->loader;

    waitIdle(cast(ptr->deviceQueue));

    for (NvFlowUint idx = 0; idx < ptr->numSwapchainImages; idx++)
    {
        destroyTexture(cast(ptr->deviceQueue->context), ptr->textures[idx]);
        ptr->textures[idx] = nullptr;
    }

    if (ptr->swapchainVulkan != VK_NULL_HANDLE)
    {
        deviceLoader->vkDestroySwapchainKHR(device->vulkanDevice, ptr->swapchainVulkan, nullptr);
        ptr->swapchainVulkan = VK_NULL_HANDLE;
    }
}

void resizeSwapchain(NvFlowSwapchain* swapchain, NvFlowUint width, NvFlowUint height)
{
    auto ptr = cast(swapchain);

    if (ptr->valid)
    {
        if (width != ptr->width ||
            height != ptr->height)
        {
            swapchain_destroySwapchain(ptr);
            ptr->valid = NV_FLOW_FALSE;
        }
    }

    if (ptr->valid == NV_FLOW_FALSE)
    {
        swapchain_initSwapchain(ptr);
    }
}

int presentSwapchain(NvFlowSwapchain* swapchain, NvFlowBool32 vsync, NvFlowUint64* flushedFrameID)
{
    auto ptr = cast(swapchain);

    auto loader = &ptr->deviceQueue->device->loader;

    if (ptr->valid == NV_FLOW_FALSE)
    {
        return flush(cast(ptr->deviceQueue), flushedFrameID, nullptr, nullptr);
    }

    *flushedFrameID = ptr->deviceQueue->nextFenceValue;

    ptr->deviceQueue->currentEndFrameSemaphore = ptr->deviceQueue->endFrameSemaphore;

    int deviceReset = flushStepA(ptr->deviceQueue, nullptr, nullptr);

    VkPresentInfoKHR presentInfo = {};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &ptr->swapchainVulkan;
    presentInfo.pImageIndices = &ptr->currentSwapchainIdx;
    presentInfo.waitSemaphoreCount = 1u;
    presentInfo.pWaitSemaphores = &ptr->deviceQueue->currentEndFrameSemaphore;

    VkResult result = loader->vkQueuePresentKHR(ptr->deviceQueue->graphicsQueue, &presentInfo);

    ptr->deviceQueue->currentEndFrameSemaphore = VK_NULL_HANDLE;

    flushStepB(ptr->deviceQueue);

    if (result != VK_SUCCESS || ptr->vsync != vsync)
    {
        ptr->vsync = vsync;
        swapchain_destroySwapchain(ptr);
        ptr->valid = NV_FLOW_FALSE;
    }

    if (ptr->valid)
    {
        NvFlowUint compWidth = 0u;
        NvFlowUint compHeight = 0u;
        swapchain_getWindowSize(ptr, &compWidth, &compHeight);

        if (compWidth != ptr->width ||
            compHeight != ptr->height)
        {
            swapchain_destroySwapchain(ptr);
            ptr->valid = NV_FLOW_FALSE;
        }
    }

    return deviceReset;
}

NvFlowTexture* getSwapchainFrontTexture(NvFlowSwapchain* swapchain)
{
    auto ptr = cast(swapchain);

    auto device = ptr->deviceQueue->device;
    auto loader = &ptr->deviceQueue->device->loader;

    if (ptr->valid == NV_FLOW_FALSE)
    {
        swapchain_initSwapchain(ptr);
    }

    if (ptr->valid == NV_FLOW_FALSE)
    {
        return nullptr;
    }

    ptr->deviceQueue->currentBeginFrameSemaphore = ptr->deviceQueue->beginFrameSemaphore;

    VkResult result = loader->vkAcquireNextImageKHR(device->vulkanDevice, ptr->swapchainVulkan, UINT64_MAX, ptr->deviceQueue->currentBeginFrameSemaphore, VK_NULL_HANDLE, &ptr->currentSwapchainIdx);

    if (result != VK_SUCCESS)
    {
        swapchain_destroySwapchain(ptr);
        ptr->valid = NV_FLOW_FALSE;
        return nullptr;
    }

    return ptr->textures[ptr->currentSwapchainIdx];
}

} // end namespace
