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

void texture_descClamping(Texture* ptr)
{
    if (ptr->desc.mipLevels == 0u)
    {
        ptr->desc.mipLevels = 1u;
    }

    if (ptr->desc.textureType == eNvFlowTextureType_1d)
    {
        ptr->desc.height = 1u;
        ptr->desc.depth = 1u;
    }
    else if (ptr->desc.textureType == eNvFlowTextureType_2d)
    {
        ptr->desc.depth = 1u;
    }
}

void texture_createImageView(Context* context, Texture* ptr, VkImageView* view_all, VkImageView* view_mipLevel)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkImageViewCreateInfo viewInfo = {};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = ptr->imageVk;
    if (ptr->desc.textureType == eNvFlowTextureType_1d)
    {
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_1D;
    }
    else if (ptr->desc.textureType == eNvFlowTextureType_2d)
    {
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    }
    else //if (ptr->desc.textureType == eNvFlowTextureType_3d)
    {
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_3D;
    }
    viewInfo.format = formatConverter_convertToVulkan(context->deviceQueue->device->formatConverter, ptr->desc.format);
    viewInfo.components.r = VK_COMPONENT_SWIZZLE_R;
    viewInfo.components.g = VK_COMPONENT_SWIZZLE_G;
    viewInfo.components.b = VK_COMPONENT_SWIZZLE_B;
    viewInfo.components.a = VK_COMPONENT_SWIZZLE_A;
    viewInfo.subresourceRange.aspectMask = ptr->imageAspect;
    viewInfo.subresourceRange.levelCount = ptr->desc.mipLevels;
    viewInfo.subresourceRange.layerCount = 1u;

    // for views with depth and stencil, default to depth view
    if (viewInfo.subresourceRange.aspectMask == (VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT))
    {
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    }

    loader->vkCreateImageView(vulkanDevice, &viewInfo, nullptr, view_all);

    // single mipLevel default for write
    viewInfo.subresourceRange.baseMipLevel = 0u;
    viewInfo.subresourceRange.levelCount = 1u;

    loader->vkCreateImageView(vulkanDevice, &viewInfo, nullptr, view_mipLevel);
}

void texture_pushImageView(Context* context, Texture* ptr, NvFlowFormat format)
{
    VkImageView imageViewVk_all = VK_NULL_HANDLE;
    VkImageView imageViewVk_mipLevel = VK_NULL_HANDLE;
    texture_createImageView(context, ptr, &imageViewVk_all, &imageViewVk_mipLevel);

    ptr->aliasFormats.pushBack(format);
    ptr->aliasImageViewAlls.pushBack(imageViewVk_all);
    ptr->aliasImageViewMipLevels.pushBack(imageViewVk_mipLevel);
}

VkImageView texture_getImageViewAll(Context* context, Texture* ptr, NvFlowFormat aliasFormat)
{
    if (aliasFormat == eNvFlowFormat_unknown)
    {
        return ptr->imageViewVk_all;
    }
    for (NvFlowUint64 idx = 0u; idx < ptr->aliasFormats.size; idx++)
    {
        if (ptr->aliasFormats[idx] == aliasFormat)
        {
            return ptr->aliasImageViewAlls[idx];
        }
    }
    texture_pushImageView(context, ptr, aliasFormat);
    return texture_getImageViewAll(context, ptr, aliasFormat);
}

VkImageView texture_getImageViewMipLevel(Context* context, Texture* ptr, NvFlowFormat aliasFormat)
{
    if (aliasFormat == eNvFlowFormat_unknown)
    {
        return ptr->imageViewVk_mipLevel;
    }
    for (NvFlowUint64 idx = 0u; idx < ptr->aliasFormats.size; idx++)
    {
        if (ptr->aliasFormats[idx] == aliasFormat)
        {
            return ptr->aliasImageViewMipLevels[idx];
        }
    }
    texture_pushImageView(context, ptr, aliasFormat);
    return texture_getImageViewMipLevel(context, ptr, aliasFormat);
}

void texture_createImage(Context* context, Texture* ptr, VkImageUsageFlags usageFlags)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkImageCreateInfo texCreateInfo = {};
    texCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    if (ptr->desc.textureType == eNvFlowTextureType_1d)
    {
        texCreateInfo.imageType = VK_IMAGE_TYPE_1D;
    }
    else if (ptr->desc.textureType == eNvFlowTextureType_2d)
    {
        texCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    }
    else //if (ptr->desc.textureType == eNvFlowTextureType_3d)
    {
        texCreateInfo.imageType = VK_IMAGE_TYPE_3D;
    }
    texCreateInfo.format = formatConverter_convertToVulkan(context->deviceQueue->device->formatConverter, ptr->desc.format);
    texCreateInfo.extent.width = ptr->desc.width;
    texCreateInfo.extent.height = ptr->desc.height;
    texCreateInfo.extent.depth = ptr->desc.depth;
    texCreateInfo.mipLevels = ptr->desc.mipLevels;
    texCreateInfo.arrayLayers = 1u;
    texCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    texCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    texCreateInfo.usage = usageFlags;
    texCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    texCreateInfo.queueFamilyIndexCount = VK_QUEUE_FAMILY_IGNORED;
    texCreateInfo.pQueueFamilyIndices = nullptr;
    texCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    loader->vkCreateImage(vulkanDevice, &texCreateInfo, nullptr, &ptr->imageVk);

    VkMemoryRequirements texMemReq = {};
    loader->vkGetImageMemoryRequirements(vulkanDevice, ptr->imageVk, &texMemReq);

    uint32_t texMemType_device = context_getMemoryType(context, texMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    uint32_t texMemType_sysmem = context_getMemoryType(context, texMemReq.memoryTypeBits, 0);

    ptr->allocationBytes = texMemReq.size;
    device_reportMemoryAllocate(context->deviceQueue->device, eNvFlowMemoryType_device, ptr->allocationBytes);

    VkMemoryAllocateInfo texMemAllocInfo = {};
    texMemAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    texMemAllocInfo.allocationSize = texMemReq.size;
    texMemAllocInfo.memoryTypeIndex = texMemType_device;

    VkResult result = loader->vkAllocateMemory(vulkanDevice, &texMemAllocInfo, nullptr, &ptr->memoryVk);
    if (result == VK_SUCCESS)
    {
        context->deviceQueue->device->logPrint(eNvFlowLogLevel_info, "Texture vidmem allocate %lld bytes", texMemReq.size);
    }
    else
    {
        texMemAllocInfo.memoryTypeIndex = texMemType_sysmem;
        result = loader->vkAllocateMemory(vulkanDevice, &texMemAllocInfo, nullptr, &ptr->memoryVk);
        if (result == VK_SUCCESS)
        {
            context->deviceQueue->device->logPrint(eNvFlowLogLevel_info, "Texture sysmem allocate %lld bytes", texMemReq.size);
        }
        else
        {
            context->deviceQueue->device->logPrint(eNvFlowLogLevel_info, "Texture allocate failed %lld bytes", texMemReq.size);
        }
    }

    loader->vkBindImageMemory(vulkanDevice, ptr->imageVk, ptr->memoryVk, 0u);
}

void texture_initRestoreBarrier(Context* context, Texture* ptr)
{
    VkImageMemoryBarrier barrier = {};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.pNext = nullptr;
    barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    barrier.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = ptr->imageVk;
    barrier.subresourceRange.aspectMask = ptr->imageAspect;
    barrier.subresourceRange.baseMipLevel = 0u;
    barrier.subresourceRange.levelCount = ptr->desc.mipLevels;
    barrier.subresourceRange.baseArrayLayer = 0u;
    barrier.subresourceRange.layerCount = 1u;

    ptr->restoreBarrier = barrier;
}

void texture_initCurrentBarrier(Context* context, Texture* ptr)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkImageMemoryBarrier barrier = ptr->restoreBarrier;
    barrier.srcAccessMask = 0u;
    barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    barrier.dstAccessMask = 0u;
    barrier.newLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    ptr->currentBarrier = barrier;
}

Texture* texture_create(Context* context, const NvFlowTextureDesc* desc)
{
    auto ptr = new Texture();

    ptr->desc = *desc;
    ptr->imageAspect = VK_IMAGE_ASPECT_COLOR_BIT;

    texture_descClamping(ptr);

    VkImageUsageFlags usageFlags = 0u;
    if (ptr->desc.usageFlags & eNvFlowTextureUsage_rwTexture)
    {
        usageFlags |= VK_IMAGE_USAGE_STORAGE_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowTextureUsage_texture)
    {
        usageFlags |= VK_IMAGE_USAGE_SAMPLED_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowTextureUsage_textureCopySrc)
    {
        usageFlags |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowTextureUsage_textureCopyDst)
    {
        usageFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    }

    texture_createImage(context, ptr, usageFlags);
    texture_createImageView(context, ptr, &ptr->imageViewVk_all, &ptr->imageViewVk_mipLevel);

    //texture_computeSubresources(context, ptr);
    texture_initRestoreBarrier(context, ptr);

    texture_initCurrentBarrier(context, ptr);

    return ptr;
}

Texture* texture_createExternal(Context* context, const NvFlowTextureDesc* desc, VkImage externalImage, VkImageLayout defaultLayout)
{
    auto ptr = new Texture();

    ptr->desc = *desc;
    ptr->imageAspect = VK_IMAGE_ASPECT_COLOR_BIT;

    texture_descClamping(ptr);

    ptr->memoryVk = VK_NULL_HANDLE;
    ptr->imageVk = externalImage;
    texture_createImageView(context, ptr, &ptr->imageViewVk_all, &ptr->imageViewVk_mipLevel);

    //texture_computeSubresources(context, ptr);
    texture_initRestoreBarrier(context, ptr);

    ptr->restoreBarrier.oldLayout = defaultLayout;
    ptr->restoreBarrier.newLayout = defaultLayout;

    texture_initCurrentBarrier(context, ptr);

    return ptr;
}

void texture_destroy(Context* context, Texture* ptr)
{
    auto loader = &context->deviceQueue->device->loader;

    loader->vkDestroyImageView(loader->device, ptr->imageViewVk_mipLevel, nullptr);
    loader->vkDestroyImageView(loader->device, ptr->imageViewVk_all, nullptr);
    for (NvFlowUint64 idx = 0u; idx < ptr->aliasImageViewAlls.size; idx++)
    {
        loader->vkDestroyImageView(loader->device, ptr->aliasImageViewAlls[idx], nullptr);
    }
    for (NvFlowUint64 idx = 0u; idx < ptr->aliasImageViewMipLevels.size; idx++)
    {
        loader->vkDestroyImageView(loader->device, ptr->aliasImageViewMipLevels[idx], nullptr);
    }

    // use memoryVk as indicator of ownership of original image
    if (ptr->memoryVk)
    {
        loader->vkDestroyImage(loader->device, ptr->imageVk, nullptr);
        loader->vkFreeMemory(loader->device, ptr->memoryVk, nullptr);

        device_reportMemoryFree(context->deviceQueue->device, eNvFlowMemoryType_device, ptr->allocationBytes);
    }

    delete ptr;
}

NvFlowBool32 textureDesc_compare(const NvFlowTextureDesc* a, const NvFlowTextureDesc* b)
{
    if (a->textureType == b->textureType &&
        a->usageFlags == b->usageFlags &&
        a->format == b->format &&
        a->width == b->width &&
        a->height == b->height &&
        a->depth == b->depth &&
        a->mipLevels == b->mipLevels &&
        a->optimizedClearValue.x == b->optimizedClearValue.x &&
        a->optimizedClearValue.y == b->optimizedClearValue.y &&
        a->optimizedClearValue.z == b->optimizedClearValue.z &&
        a->optimizedClearValue.w == b->optimizedClearValue.w )
    {
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

NvFlowTexture* createTexture(NvFlowContext* contextIn, const NvFlowTextureDesc* desc)
{
    auto context = cast(contextIn);

    for (NvFlowUint idx = 0u; idx < context->pool_textures.size; idx++)
    {
        auto ptr = context->pool_textures[idx];
        if (ptr && ptr->refCount == 0 && textureDesc_compare(&ptr->desc, desc))
        {
            ptr->refCount = 1;
            ptr->lastActive = context->deviceQueue->nextFenceValue;
            return cast(ptr);
        }
    }

    auto ptr = texture_create(context, desc);

    ptr->refCount = 1;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
    context->pool_textures.pushBack(ptr);

    return cast(ptr);
}

NvFlowTexture* createTextureExternal(NvFlowContext* contextIn, const NvFlowTextureDesc* desc, VkImage externalImage, VkImageLayout defaultLayout)
{
    auto context = cast(contextIn);

    auto ptr = texture_createExternal(context, desc, externalImage, defaultLayout);

    ptr->refCount = 1;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
    context->pool_textures.pushBack(ptr);

    return cast(ptr);
}

void destroyTexture(NvFlowContext* contextIn, NvFlowTexture* texture)
{
    auto context = cast(contextIn);
    auto ptr = cast(texture);

    if (ptr->refCount <= 0)
    {
        context->logPrint(eNvFlowLogLevel_error, "NvFlowContext::destroyTexture() called on inactive texture %p", texture);
    }

    ptr->refCount--;
    ptr->lastActive = context->deviceQueue->nextFenceValue;

    // if external, actually destroy
    if (!ptr->memoryVk)
    {
        context->pool_textures.removeSwapPointer(ptr);
        context->pool_textures[context->pool_textures.size] = nullptr;
        texture_destroy(context, ptr);
    }
}

void context_destroyTextures(Context* context)
{
    context->textureTransients.deletePointers();
    context->textureAcquires.deletePointers();

    for (NvFlowUint idx = 0u; idx < context->pool_textures.size; idx++)
    {
        auto& ptr = context->pool_textures[idx];
        texture_destroy(context, ptr);
        ptr = nullptr;
    }
}

NvFlowTextureTransient* getTextureTransient(NvFlowContext* contextIn, const NvFlowTextureDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = context->textureTransients.allocateBackPointer();
    ptr->desc = *desc;
    ptr->texture = nullptr;
    ptr->aliasTexture = nullptr;
    ptr->aliasFormat = eNvFlowFormat_unknown;
    ptr->nodeBegin = 0;
    ptr->nodeEnd = 0;
    return cast(ptr);
}

NvFlowTextureTransient* registerTextureAsTransient(NvFlowContext* contextIn, NvFlowTexture* texture)
{
    auto context = cast(contextIn);
    auto ptr = context->textureTransients.allocateBackPointer();
    ptr->desc = cast(texture)->desc;
    ptr->texture = cast(texture);
    ptr->aliasTexture = nullptr;
    ptr->aliasFormat = eNvFlowFormat_unknown;
    ptr->nodeBegin = 0;
    ptr->nodeEnd = 0;
    return cast(ptr);
}

NvFlowTextureTransient* aliasTextureTransient(NvFlowContext* contextIn, NvFlowTextureTransient* texture, NvFlowFormat format)
{
    auto context = cast(contextIn);
    auto ptr = context->textureTransients.allocateBackPointer();
    ptr->desc = cast(texture)->desc;
    ptr->texture = nullptr;
    ptr->aliasTexture = cast(texture);
    ptr->aliasFormat = format;
    ptr->nodeBegin = 0;
    ptr->nodeEnd = 0;
    return cast(ptr);
}

NvFlowTextureAcquire* enqueueAcquireTexture(NvFlowContext* contextIn, NvFlowTextureTransient* textureTransient)
{
    auto context = cast(contextIn);
    auto ptr = context->textureAcquires.allocateBackPointer();
    ptr->textureTransient = cast(textureTransient);
    ptr->texture = nullptr;
    return cast(ptr);
}

NvFlowBool32 getAcquiredTexture(NvFlowContext* contextIn, NvFlowTextureAcquire* acquire, NvFlowTexture** outTexture)
{
    auto context = cast(contextIn);
    auto ptr = cast(acquire);
    if (ptr->texture)
    {
        *outTexture = cast(ptr->texture);

        // remove from acquire array
        context->textureAcquires.removeSwapPointer(ptr);

        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

NvFlowTextureTransient* getTextureTransientById(NvFlowContext* context, NvFlowUint64 textureId)
{
    auto ctx = cast(context);
    for (NvFlowUint idx = 0u; idx < ctx->registeredResources.size; idx++)
    {
        if (ctx->registeredResources[idx].uid == textureId)
        {
            if (ctx->registeredResources[idx].texture)
            {
                return registerTextureAsTransient(context, ctx->registeredResources[idx].texture);
            }
        }
    }
    return nullptr;
}

/// ***************************** Samplers ********************************************

VkSamplerAddressMode sampler_convertAddressMode(NvFlowSamplerAddressMode addressMode)
{
    switch (addressMode)
    {
    case eNvFlowSamplerAddressMode_wrap:
        return VK_SAMPLER_ADDRESS_MODE_REPEAT;
    case eNvFlowSamplerAddressMode_clamp:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    case eNvFlowSamplerAddressMode_mirror:
        return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
    case eNvFlowSamplerAddressMode_border:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    default:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    }
}

VkFilter sampler_convertFilter(NvFlowSamplerFilterMode filter)
{
    switch (filter)
    {
    case eNvFlowSamplerFilterMode_point:
        return VK_FILTER_NEAREST;
    case eNvFlowSamplerFilterMode_linear:
        return VK_FILTER_LINEAR;
    default:
        return VK_FILTER_NEAREST;
    }
}

VkSamplerMipmapMode sampler_convertMipmapMode(NvFlowSamplerFilterMode filter)
{
    switch (filter)
    {
    case eNvFlowSamplerFilterMode_point:
        return VK_SAMPLER_MIPMAP_MODE_NEAREST;
    case eNvFlowSamplerFilterMode_linear:
        return VK_SAMPLER_MIPMAP_MODE_LINEAR;
    default:
        return VK_SAMPLER_MIPMAP_MODE_NEAREST;
    }
}

Sampler* sampler_create(Context* context, const NvFlowSamplerDesc* desc)
{
    auto loader = &context->deviceQueue->device->loader;
    auto ptr = new Sampler();

    VkSamplerCreateInfo samplerCreateInfo = {};
    samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerCreateInfo.magFilter = sampler_convertFilter(desc->filterMode);
    samplerCreateInfo.minFilter = sampler_convertFilter(desc->filterMode);
    samplerCreateInfo.mipmapMode = sampler_convertMipmapMode(desc->filterMode);
    samplerCreateInfo.addressModeU = sampler_convertAddressMode(desc->addressModeU);
    samplerCreateInfo.addressModeV = sampler_convertAddressMode(desc->addressModeV);
    samplerCreateInfo.addressModeW = sampler_convertAddressMode(desc->addressModeW);
    samplerCreateInfo.minLod = 0.f;
    samplerCreateInfo.maxLod = VK_LOD_CLAMP_NONE;
    samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
    samplerCreateInfo.maxAnisotropy = 1.f;
    samplerCreateInfo.compareEnable = VK_FALSE;
    samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;

    loader->vkCreateSampler(context->deviceQueue->vulkanDevice, &samplerCreateInfo, nullptr, &ptr->sampler);

    return ptr;
}

void sampler_destroy(Context* context, Sampler* ptr)
{
    auto loader = &context->deviceQueue->device->loader;

    loader->vkDestroySampler(loader->device, ptr->sampler, nullptr);

    delete ptr;
}

NvFlowSampler* createSampler(NvFlowContext* contextIn, const NvFlowSamplerDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = sampler_create(context, desc);

    ptr->isActive = NV_FLOW_TRUE;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
    context->pool_samplers.pushBack(ptr);

    return cast(ptr);
}

NvFlowSampler* getDefaultSampler(NvFlowContext* contextIn)
{
    auto context = cast(contextIn);
    return cast(context->pool_samplers[0u]);
}

void destroySampler(NvFlowContext* contextIn, NvFlowSampler* sampler)
{
    auto context = cast(contextIn);
    auto ptr = cast(sampler);

    ptr->isActive = NV_FLOW_FALSE;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
}

void context_destroySamplers(Context* context)
{
    for (NvFlowUint idx = 0u; idx < context->pool_samplers.size; idx++)
    {
        auto& ptr = context->pool_samplers[idx];
        sampler_destroy(context, ptr);
        ptr = nullptr;
    }
}

} // end namespace
