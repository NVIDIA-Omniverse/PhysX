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

#include "NvFlowTypes.h"

/// ********************* Loader *******************

#define NV_FLOW_VK_LOADER_PTR(FUNC_REF) \
    PFN_##FUNC_REF FUNC_REF = nullptr

#define NV_FLOW_VK_LOADER_INSTANCE(X) \
    ptr-> X = (PFN_##X)ptr->vkGetInstanceProcAddr(vulkanInstance, #X )

#define NV_FLOW_VK_LOADER_DEVICE(X) \
    ptr-> X = (PFN_##X)ptr->vkGetDeviceProcAddr(vulkanDevice, #X )

struct NvFlowVulkanEnabledFeatures
{
    NvFlowBool32 shaderStorageImageWriteWithoutFormat;
};

struct NvFlowVulkanEnabledInstanceExtensions
{
    NvFlowBool32 VK_KHR_SURFACE;
    NvFlowBool32 VK_KHR_WIN32_SURFACE;
    NvFlowBool32 VK_KHR_XLIB_SURFACE;
    NvFlowBool32 VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2;
    NvFlowBool32 VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES;
    NvFlowBool32 VK_KHR_EXTERNAL_FENCE_CAPABILITIES;
    NvFlowBool32 VK_KHR_EXTERNAL_MEMORY_CAPABILITIES;
};

struct NvFlowVulkanEnabledDeviceExtensions
{
    NvFlowBool32 VK_KHR_SWAPCHAIN;
    NvFlowBool32 VK_KHR_EXTERNAL_MEMORY;
    NvFlowBool32 VK_KHR_EXTERNAL_SEMAPHORE;
    NvFlowBool32 VK_KHR_EXTERNAL_MEMORY_WIN32;
    NvFlowBool32 VK_KHR_EXTERNAL_SEMAPHORE_WIN32;
    NvFlowBool32 VK_KHR_EXTERNAL_MEMORY_FD;
    NvFlowBool32 VK_KHR_EXTERNAL_SEMAPHORE_FD;
};

struct NvFlowVulkanInstanceLoader
{
    VkInstance instance = nullptr;

    // Instance functions
    NV_FLOW_VK_LOADER_PTR(vkGetInstanceProcAddr);
    NV_FLOW_VK_LOADER_PTR(vkCreateInstance);
    NV_FLOW_VK_LOADER_PTR(vkEnumerateInstanceExtensionProperties);

    NV_FLOW_VK_LOADER_PTR(vkDestroyInstance);
    NV_FLOW_VK_LOADER_PTR(vkGetDeviceProcAddr);
    NV_FLOW_VK_LOADER_PTR(vkEnumeratePhysicalDevices);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceProperties);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceProperties2KHR);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceQueueFamilyProperties);

    NV_FLOW_VK_LOADER_PTR(vkCreateDevice);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceMemoryProperties);
    NV_FLOW_VK_LOADER_PTR(vkEnumerateDeviceExtensionProperties);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceFeatures);

    // Instance surface
#if defined(_WIN32)
    NV_FLOW_VK_LOADER_PTR(vkCreateWin32SurfaceKHR);
#else
    NV_FLOW_VK_LOADER_PTR(vkCreateXlibSurfaceKHR);
#endif
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceSurfaceSupportKHR);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceSurfaceCapabilitiesKHR);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceSurfaceFormatsKHR);
    NV_FLOW_VK_LOADER_PTR(vkGetPhysicalDeviceSurfacePresentModesKHR);
    NV_FLOW_VK_LOADER_PTR(vkDestroySurfaceKHR);
};

struct NvFlowVulkanDeviceLoader
{
    VkDevice device = nullptr;

    NV_FLOW_VK_LOADER_PTR(vkGetDeviceProcAddr);

    // Device functions
    NV_FLOW_VK_LOADER_PTR(vkDestroyDevice);
    NV_FLOW_VK_LOADER_PTR(vkGetDeviceQueue);
    NV_FLOW_VK_LOADER_PTR(vkCreateCommandPool);
    NV_FLOW_VK_LOADER_PTR(vkAllocateCommandBuffers);
    NV_FLOW_VK_LOADER_PTR(vkCreateFence);

    NV_FLOW_VK_LOADER_PTR(vkCreateSemaphore);
    NV_FLOW_VK_LOADER_PTR(vkDestroySemaphore);
    NV_FLOW_VK_LOADER_PTR(vkFreeCommandBuffers);
    NV_FLOW_VK_LOADER_PTR(vkDestroyFence);
    NV_FLOW_VK_LOADER_PTR(vkDestroyCommandPool);

    NV_FLOW_VK_LOADER_PTR(vkEndCommandBuffer);
    NV_FLOW_VK_LOADER_PTR(vkQueueSubmit);
    NV_FLOW_VK_LOADER_PTR(vkWaitForFences);
    NV_FLOW_VK_LOADER_PTR(vkResetFences);
    NV_FLOW_VK_LOADER_PTR(vkResetCommandBuffer);

    NV_FLOW_VK_LOADER_PTR(vkBeginCommandBuffer);
    NV_FLOW_VK_LOADER_PTR(vkQueueWaitIdle);
    NV_FLOW_VK_LOADER_PTR(vkCreateImage);
    NV_FLOW_VK_LOADER_PTR(vkGetImageMemoryRequirements);
    NV_FLOW_VK_LOADER_PTR(vkAllocateMemory);

    NV_FLOW_VK_LOADER_PTR(vkBindImageMemory);
    NV_FLOW_VK_LOADER_PTR(vkCreateImageView);
    NV_FLOW_VK_LOADER_PTR(vkDestroyImageView);
    NV_FLOW_VK_LOADER_PTR(vkDestroyImage);
    NV_FLOW_VK_LOADER_PTR(vkFreeMemory);

    NV_FLOW_VK_LOADER_PTR(vkCreateBuffer);
    NV_FLOW_VK_LOADER_PTR(vkGetBufferMemoryRequirements);
    NV_FLOW_VK_LOADER_PTR(vkBindBufferMemory);
    NV_FLOW_VK_LOADER_PTR(vkMapMemory);
    NV_FLOW_VK_LOADER_PTR(vkDestroyBuffer);

    NV_FLOW_VK_LOADER_PTR(vkCmdPipelineBarrier);
    NV_FLOW_VK_LOADER_PTR(vkCmdCopyBuffer);
    NV_FLOW_VK_LOADER_PTR(vkCreateShaderModule);
    NV_FLOW_VK_LOADER_PTR(vkDestroyPipeline);
    NV_FLOW_VK_LOADER_PTR(vkDestroyShaderModule);

    NV_FLOW_VK_LOADER_PTR(vkCreateDescriptorSetLayout);
    NV_FLOW_VK_LOADER_PTR(vkCreatePipelineLayout);
    NV_FLOW_VK_LOADER_PTR(vkDestroyPipelineLayout);
    NV_FLOW_VK_LOADER_PTR(vkDestroyDescriptorSetLayout);
    NV_FLOW_VK_LOADER_PTR(vkCreateDescriptorPool);

    NV_FLOW_VK_LOADER_PTR(vkAllocateDescriptorSets);
    NV_FLOW_VK_LOADER_PTR(vkDestroyDescriptorPool);
    NV_FLOW_VK_LOADER_PTR(vkUpdateDescriptorSets);
    NV_FLOW_VK_LOADER_PTR(vkCmdBindDescriptorSets);
    NV_FLOW_VK_LOADER_PTR(vkCmdBindPipeline);

    NV_FLOW_VK_LOADER_PTR(vkResetDescriptorPool);
    NV_FLOW_VK_LOADER_PTR(vkCreateBufferView);
    NV_FLOW_VK_LOADER_PTR(vkDestroyBufferView);
    NV_FLOW_VK_LOADER_PTR(vkCreateComputePipelines);
    NV_FLOW_VK_LOADER_PTR(vkCreateSampler);

    NV_FLOW_VK_LOADER_PTR(vkDestroySampler);
    NV_FLOW_VK_LOADER_PTR(vkCmdDispatch);
    NV_FLOW_VK_LOADER_PTR(vkCmdDispatchIndirect);
    NV_FLOW_VK_LOADER_PTR(vkCreateQueryPool);
    NV_FLOW_VK_LOADER_PTR(vkDestroyQueryPool);

    NV_FLOW_VK_LOADER_PTR(vkCmdResetQueryPool);
    NV_FLOW_VK_LOADER_PTR(vkCmdWriteTimestamp);
    NV_FLOW_VK_LOADER_PTR(vkCmdCopyQueryPoolResults);
    NV_FLOW_VK_LOADER_PTR(vkGetImageSubresourceLayout);
    NV_FLOW_VK_LOADER_PTR(vkCmdCopyImage);

    NV_FLOW_VK_LOADER_PTR(vkCmdCopyBufferToImage);
    NV_FLOW_VK_LOADER_PTR(vkCmdCopyImageToBuffer);
    NV_FLOW_VK_LOADER_PTR(vkCmdPushConstants);
#if defined(_WIN32)
    NV_FLOW_VK_LOADER_PTR(vkGetMemoryWin32HandleKHR);
    NV_FLOW_VK_LOADER_PTR(vkGetSemaphoreWin32HandleKHR);
#else
    NV_FLOW_VK_LOADER_PTR(vkGetMemoryFdKHR);
    NV_FLOW_VK_LOADER_PTR(vkGetSemaphoreFdKHR);
#endif

    // Device surface
    NV_FLOW_VK_LOADER_PTR(vkQueuePresentKHR);
    NV_FLOW_VK_LOADER_PTR(vkCreateSwapchainKHR);
    NV_FLOW_VK_LOADER_PTR(vkGetSwapchainImagesKHR);
    NV_FLOW_VK_LOADER_PTR(vkDestroySwapchainKHR);
    NV_FLOW_VK_LOADER_PTR(vkAcquireNextImageKHR);
};

NV_FLOW_INLINE void NvFlowVulkanLoader_global(NvFlowVulkanInstanceLoader* ptr, PFN_vkGetInstanceProcAddr getInstanceProcAddr)
{
    VkInstance vulkanInstance = nullptr;
    ptr->vkGetInstanceProcAddr = getInstanceProcAddr;
    NV_FLOW_VK_LOADER_INSTANCE(vkCreateInstance);
    NV_FLOW_VK_LOADER_INSTANCE(vkEnumerateInstanceExtensionProperties);
}

NV_FLOW_INLINE void NvFlowVulkanLoader_instance(NvFlowVulkanInstanceLoader* ptr, VkInstance vulkanInstance)
{
    ptr->instance = vulkanInstance;

    NV_FLOW_VK_LOADER_INSTANCE(vkDestroyInstance);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetDeviceProcAddr);
    NV_FLOW_VK_LOADER_INSTANCE(vkEnumeratePhysicalDevices);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceProperties);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceProperties2KHR);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceQueueFamilyProperties);

    NV_FLOW_VK_LOADER_INSTANCE(vkCreateDevice);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceMemoryProperties);
    NV_FLOW_VK_LOADER_INSTANCE(vkEnumerateDeviceExtensionProperties);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceFeatures);

    // surface extensions
#if defined(_WIN32)
    NV_FLOW_VK_LOADER_INSTANCE(vkCreateWin32SurfaceKHR);
#else
    NV_FLOW_VK_LOADER_INSTANCE(vkCreateXlibSurfaceKHR);
#endif
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceSurfaceSupportKHR);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceSurfaceCapabilitiesKHR);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceSurfaceFormatsKHR);
    NV_FLOW_VK_LOADER_INSTANCE(vkGetPhysicalDeviceSurfacePresentModesKHR);
    NV_FLOW_VK_LOADER_INSTANCE(vkDestroySurfaceKHR);
}

NV_FLOW_INLINE void NvFlowVulkanLoader_device(NvFlowVulkanDeviceLoader* ptr, VkDevice vulkanDevice, PFN_vkGetDeviceProcAddr getDeviceProcAddr)
{
    ptr->device = vulkanDevice;
    ptr->vkGetDeviceProcAddr = getDeviceProcAddr;

    NV_FLOW_VK_LOADER_DEVICE(vkDestroyDevice);
    NV_FLOW_VK_LOADER_DEVICE(vkGetDeviceQueue);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateCommandPool);
    NV_FLOW_VK_LOADER_DEVICE(vkAllocateCommandBuffers);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateFence);

    NV_FLOW_VK_LOADER_DEVICE(vkCreateSemaphore);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroySemaphore);
    NV_FLOW_VK_LOADER_DEVICE(vkFreeCommandBuffers);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyFence);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyCommandPool);

    NV_FLOW_VK_LOADER_DEVICE(vkEndCommandBuffer);
    NV_FLOW_VK_LOADER_DEVICE(vkQueueSubmit);
    NV_FLOW_VK_LOADER_DEVICE(vkWaitForFences);
    NV_FLOW_VK_LOADER_DEVICE(vkResetFences);
    NV_FLOW_VK_LOADER_DEVICE(vkResetCommandBuffer);

    NV_FLOW_VK_LOADER_DEVICE(vkBeginCommandBuffer);
    NV_FLOW_VK_LOADER_DEVICE(vkQueueWaitIdle);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateImage);
    NV_FLOW_VK_LOADER_DEVICE(vkGetImageMemoryRequirements);
    NV_FLOW_VK_LOADER_DEVICE(vkAllocateMemory);

    NV_FLOW_VK_LOADER_DEVICE(vkBindImageMemory);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateImageView);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyImageView);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyImage);
    NV_FLOW_VK_LOADER_DEVICE(vkFreeMemory);

    NV_FLOW_VK_LOADER_DEVICE(vkCreateBuffer);
    NV_FLOW_VK_LOADER_DEVICE(vkGetBufferMemoryRequirements);
    NV_FLOW_VK_LOADER_DEVICE(vkBindBufferMemory);
    NV_FLOW_VK_LOADER_DEVICE(vkMapMemory);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyBuffer);

    NV_FLOW_VK_LOADER_DEVICE(vkCmdPipelineBarrier);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdCopyBuffer);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateShaderModule);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyPipeline);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyShaderModule);

    NV_FLOW_VK_LOADER_DEVICE(vkCreateDescriptorSetLayout);
    NV_FLOW_VK_LOADER_DEVICE(vkCreatePipelineLayout);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyPipelineLayout);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyDescriptorSetLayout);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateDescriptorPool);

    NV_FLOW_VK_LOADER_DEVICE(vkAllocateDescriptorSets);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyDescriptorPool);
    NV_FLOW_VK_LOADER_DEVICE(vkUpdateDescriptorSets);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdBindDescriptorSets);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdBindPipeline);

    NV_FLOW_VK_LOADER_DEVICE(vkResetDescriptorPool);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateBufferView);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyBufferView);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateComputePipelines);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateSampler);

    NV_FLOW_VK_LOADER_DEVICE(vkDestroySampler);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdDispatch);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdDispatchIndirect);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateQueryPool);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroyQueryPool);

    NV_FLOW_VK_LOADER_DEVICE(vkCmdResetQueryPool);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdWriteTimestamp);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdCopyQueryPoolResults);
    NV_FLOW_VK_LOADER_DEVICE(vkGetImageSubresourceLayout);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdCopyImage);

    NV_FLOW_VK_LOADER_DEVICE(vkCmdCopyBufferToImage);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdCopyImageToBuffer);
    NV_FLOW_VK_LOADER_DEVICE(vkCmdPushConstants);
#if defined(_WIN32)
    NV_FLOW_VK_LOADER_DEVICE(vkGetMemoryWin32HandleKHR);
    NV_FLOW_VK_LOADER_DEVICE(vkGetSemaphoreWin32HandleKHR);
#else
    NV_FLOW_VK_LOADER_DEVICE(vkGetMemoryFdKHR);
    NV_FLOW_VK_LOADER_DEVICE(vkGetSemaphoreFdKHR);
#endif

    // device surface extensions
    NV_FLOW_VK_LOADER_DEVICE(vkQueuePresentKHR);
    NV_FLOW_VK_LOADER_DEVICE(vkCreateSwapchainKHR);
    NV_FLOW_VK_LOADER_DEVICE(vkGetSwapchainImagesKHR);
    NV_FLOW_VK_LOADER_DEVICE(vkDestroySwapchainKHR);
    NV_FLOW_VK_LOADER_DEVICE(vkAcquireNextImageKHR);
}
