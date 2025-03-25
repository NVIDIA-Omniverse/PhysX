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

namespace NvFlowVulkan
{
    /// FormatConverter

    struct FormatConverterElement
    {
        NvFlowFormat formatNvFlow;
        VkFormat formatVulkan;
        NvFlowUint sizeInBytes;
    };

    struct FormatConverter
    {
        FormatConverterElement formatsNvFlow[eNvFlowFormat_count] = { { eNvFlowFormat_unknown, VK_FORMAT_UNDEFINED , 0u } };
        FormatConverterElement formatsVulkan[eNvFlowFormat_count] = { { eNvFlowFormat_unknown, VK_FORMAT_UNDEFINED , 0u } };
    };

    void formatConverter_placeElement(FormatConverter* ptr, NvFlowFormat formatNvFlow, VkFormat formatVulkan, NvFlowUint sizeInBytes)
    {
        FormatConverterElement e = { formatNvFlow, formatVulkan, sizeInBytes };
        ptr->formatsNvFlow[formatNvFlow] = e;
        ptr->formatsVulkan[formatVulkan] = e;
    }

    FormatConverter* formatConverter_create()
    {
        auto ptr = new FormatConverter();

        formatConverter_placeElement(ptr, eNvFlowFormat_unknown, VK_FORMAT_UNDEFINED, 1u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32b32a32_float, VK_FORMAT_R32G32B32A32_SFLOAT, 16u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32b32a32_uint, VK_FORMAT_R32G32B32A32_UINT, 16u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32b32a32_sint, VK_FORMAT_R32G32B32A32_SINT, 16u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32b32_float, VK_FORMAT_R32G32B32_SFLOAT, 12u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32b32_uint, VK_FORMAT_R32G32B32_UINT, 12u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32b32_sint, VK_FORMAT_R32G32B32_SINT, 12u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16b16a16_float, VK_FORMAT_R16G16B16A16_SFLOAT, 8u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16b16a16_unorm, VK_FORMAT_R16G16B16A16_UNORM, 8u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16b16a16_uint, VK_FORMAT_R16G16B16A16_UINT, 8u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16b16a16_snorm, VK_FORMAT_R16G16B16A16_SNORM, 8u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16b16a16_sint, VK_FORMAT_R16G16B16A16_SINT, 8u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32_float, VK_FORMAT_R32G32_SFLOAT, 8u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32_uint, VK_FORMAT_R32G32_UINT, 8u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32g32_sint, VK_FORMAT_R32G32_SINT, 8u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r10g10b10a2_unorm, VK_FORMAT_A2R10G10B10_UNORM_PACK32, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r10g10b10a2_uint, VK_FORMAT_A2R10G10B10_UINT_PACK32, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r11g11b10_float, VK_FORMAT_B10G11R11_UFLOAT_PACK32, 4u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8b8a8_unorm, VK_FORMAT_R8G8B8A8_UNORM, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8b8a8_unorm_srgb, VK_FORMAT_R8G8B8A8_SRGB, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8b8a8_uint, VK_FORMAT_R8G8B8A8_UINT, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8b8a8_snorm, VK_FORMAT_R8G8B8A8_SNORM, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8b8a8_sint, VK_FORMAT_R8G8B8A8_SINT, 4u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16_float, VK_FORMAT_R16G16_SFLOAT, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16_unorm, VK_FORMAT_R16G16_UNORM, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16_uint, VK_FORMAT_R16G16_UINT, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16_snorm, VK_FORMAT_R16G16_SNORM, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16g16_sint, VK_FORMAT_R16G16_SINT, 4u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r32_float, VK_FORMAT_R32_SFLOAT, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32_uint, VK_FORMAT_R32_UINT, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r32_sint, VK_FORMAT_R32_SINT, 4u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8_unorm, VK_FORMAT_R8G8_UNORM, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8_uint, VK_FORMAT_R8G8_UINT, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8_snorm, VK_FORMAT_R8G8_SNORM, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8g8_sint, VK_FORMAT_R8G8_SINT, 2u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r16_float, VK_FORMAT_R16_SFLOAT, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16_unorm, VK_FORMAT_R16_UNORM, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16_uint, VK_FORMAT_R16_UINT, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16_snorm, VK_FORMAT_R16_SNORM, 2u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r16_sint, VK_FORMAT_R16_SINT, 2u);

        formatConverter_placeElement(ptr, eNvFlowFormat_r8_unorm, VK_FORMAT_R8_UNORM, 1u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8_uint, VK_FORMAT_R8_UINT, 1u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8_snorm, VK_FORMAT_R8_SNORM, 1u);
        formatConverter_placeElement(ptr, eNvFlowFormat_r8_sint, VK_FORMAT_R8_SINT, 1u);

        formatConverter_placeElement(ptr, eNvFlowFormat_b8g8r8a8_unorm, VK_FORMAT_B8G8R8A8_UNORM, 4u);
        formatConverter_placeElement(ptr, eNvFlowFormat_b8g8r8a8_unorm_srgb, VK_FORMAT_B8G8R8A8_SRGB, 4u);

        return ptr;
    }

    void formatConverter_destroy(FormatConverter* ptr)
    {
        delete ptr;
    }

    VkFormat formatConverter_convertToVulkan(FormatConverter* ptr, NvFlowFormat format)
    {
        return ptr->formatsNvFlow[format].formatVulkan;
    }

    NvFlowFormat formatConverter_convertToNvFlow(FormatConverter* ptr, VkFormat format)
    {
        return ptr->formatsVulkan[format].formatNvFlow;
    }

    NvFlowUint formatConverter_getFormatSizeInBytes(FormatConverter* ptr, NvFlowFormat format)
    {
        return ptr->formatsNvFlow[format].sizeInBytes;
    }

    /// Utils

    void determineMatches(NvFlowArray<const char*, 8u>& extensionsEnabled, NvFlowArray<ExtensionRequest, 8u>& extensionsRequest, NvFlowArray<VkExtensionProperties>& extensions)
    {
        for (NvFlowUint idx = 0u; idx < extensions.size; idx++)
        {
            auto& ext = extensions[idx];

            for (NvFlowUint reqIdx = 0u; reqIdx < extensionsRequest.size; reqIdx++)
            {
                auto& extReq = extensionsRequest[reqIdx];

                if (NvFlowReflectStringCompare(extReq.name, ext.extensionName) == 0)
                {
                    extensionsEnabled.pushBack(extReq.name);
                    *extReq.pEnabled = NV_FLOW_TRUE;
                }
            }
        }
    }

    void selectInstanceExtensions(DeviceManager* ptr, NvFlowArray<const char*, 8u>& instanceExtensionsEnabled)
    {
        NvFlowArray<VkExtensionProperties> instanceExtensions;
        NvFlowArray<ExtensionRequest, 8u> instanceExtensionsRequest;

        auto& enabledExt = ptr->enabledExtensions;

#define NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(X) \
    instanceExtensionsRequest.pushBack({ X##_EXTENSION_NAME, &enabledExt.X })

        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_SURFACE);
#if defined(_WIN32)
        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_WIN32_SURFACE);
#else
        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_XLIB_SURFACE);
#endif
        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2);
        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES);
        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_EXTERNAL_FENCE_CAPABILITIES);
        NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION(VK_KHR_EXTERNAL_MEMORY_CAPABILITIES);

#undef NV_FLOW_VULKAN_TRY_ENABLE_INSTANCE_EXTENSION

        // determine what requested extensions are supported
        {
            NvFlowUint numInstanceExtensions = 0u;

            ptr->loader.vkEnumerateInstanceExtensionProperties(nullptr, &numInstanceExtensions, nullptr);

            instanceExtensions.reserve(numInstanceExtensions);
            instanceExtensions.size = numInstanceExtensions;

            ptr->loader.vkEnumerateInstanceExtensionProperties(nullptr, (uint32_t*)&instanceExtensions.size, instanceExtensions.data);

            determineMatches(instanceExtensionsEnabled, instanceExtensionsRequest, instanceExtensions);
        }
    }

    void selectDeviceExtensions(Device* ptr, NvFlowArray<const char*, 8u>& deviceExtensionsEnabled)
    {
        NvFlowArray<VkExtensionProperties> deviceExtensions;
        NvFlowArray<ExtensionRequest, 8u> deviceExtensionsRequest;

        auto& enabledExt = ptr->enabledExtensions;

#define NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(X) \
    deviceExtensionsRequest.pushBack({ X##_EXTENSION_NAME, &enabledExt.X })

        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_SWAPCHAIN);

        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_EXTERNAL_MEMORY);
        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_EXTERNAL_SEMAPHORE);
#if defined(_WIN32)
        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_EXTERNAL_MEMORY_WIN32);
        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_EXTERNAL_SEMAPHORE_WIN32);
#else
        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_EXTERNAL_MEMORY_FD);
        NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION(VK_KHR_EXTERNAL_SEMAPHORE_FD);
#endif

#undef NV_FLOW_VULKAN_TRY_ENABLE_DEVICE_EXTENSION

        // determine what requested extensions are supported
        {
            NvFlowUint numDeviceExtensions = 0u;
            ptr->deviceManager->loader.vkEnumerateDeviceExtensionProperties(ptr->physicalDevice, nullptr, &numDeviceExtensions, nullptr);

            deviceExtensions.reserve(numDeviceExtensions);
            deviceExtensions.size = numDeviceExtensions;

            ptr->deviceManager->loader.vkEnumerateDeviceExtensionProperties(ptr->physicalDevice, nullptr, &numDeviceExtensions, deviceExtensions.data);

            determineMatches(deviceExtensionsEnabled, deviceExtensionsRequest, deviceExtensions);
        }
    }
}
