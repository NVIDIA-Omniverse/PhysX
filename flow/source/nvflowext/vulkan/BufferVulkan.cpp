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
#include <string.h>
#endif

namespace NvFlowVulkan
{

void buffer_createBuffer(Context* context, Buffer* ptr, const NvFlowInteropHandle* interopHandle)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkBufferCreateInfo bufCreateInfo = {};
    bufCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufCreateInfo.size = ptr->desc.sizeInBytes;
    bufCreateInfo.usage = 0u;
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_constantBuffer)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_structuredBuffer)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_buffer)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_UNIFORM_TEXEL_BUFFER_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_rwStructuredBuffer)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_rwBuffer)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_STORAGE_TEXEL_BUFFER_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_indirectBuffer)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_bufferCopySrc)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_bufferCopyDst)
    {
        bufCreateInfo.usage |= VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    }
    bufCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkExternalMemoryBufferCreateInfoKHR externalMemoryBufferCreateInfo = {};
    if (context->deviceQueue->device->desc.enableExternalUsage && ptr->memoryType == eNvFlowMemoryType_device)
    {
        externalMemoryBufferCreateInfo.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO_KHR;
#if defined(_WIN32)
        externalMemoryBufferCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR;
#else
        externalMemoryBufferCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#endif
        bufCreateInfo.pNext = &externalMemoryBufferCreateInfo;
    }

    loader->vkCreateBuffer(vulkanDevice, &bufCreateInfo, nullptr, &ptr->bufferVk);

    VkMemoryRequirements bufMemReq = {};
    loader->vkGetBufferMemoryRequirements(vulkanDevice, ptr->bufferVk, &bufMemReq);

    uint32_t bufMemType = 0u;
    uint32_t bufMemType_sysmem = 0u;
    if (ptr->memoryType == eNvFlowMemoryType_upload)
    {
        bufMemType = context_getMemoryType(context, bufMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
    else if (ptr->memoryType == eNvFlowMemoryType_readback)
    {
        bufMemType = context_getMemoryType(context, bufMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
        if (bufMemType == ~0u)
        {
            bufMemType = context_getMemoryType(context, bufMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }
    }
    else // (ptr->memoryType == eNvFlowMemoryType_device)
    {
        bufMemType = context_getMemoryType(context, bufMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        bufMemType_sysmem = context_getMemoryType(context, bufMemReq.memoryTypeBits, 0);
    }

    VkMemoryAllocateInfo bufMemAllocInfo = {};
    bufMemAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    bufMemAllocInfo.allocationSize = bufMemReq.size;
    bufMemAllocInfo.memoryTypeIndex = bufMemType;

    VkExportMemoryAllocateInfoKHR exportAllocInfo = {};
    if (!interopHandle && context->deviceQueue->device->desc.enableExternalUsage && ptr->memoryType == eNvFlowMemoryType_device)
    {
        exportAllocInfo.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO_KHR;
#if defined(_WIN32)
        exportAllocInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR;
#else
        exportAllocInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#endif
        bufMemAllocInfo.pNext = &exportAllocInfo;
    }

#if defined(_WIN32)
    VkImportMemoryWin32HandleInfoKHR importAllocInfo = {};
    importAllocInfo.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR;
    importAllocInfo.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR;
    if (interopHandle)
    {
        importAllocInfo.handle = (HANDLE)interopHandle->value;
        bufMemAllocInfo.pNext = &importAllocInfo;
    }
#else
    VkImportMemoryFdInfoKHR importAllocInfo = {};
    importAllocInfo.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR;
    importAllocInfo.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
    if (interopHandle)
    {
        importAllocInfo.fd = (int)interopHandle->value;
        bufMemAllocInfo.pNext = &importAllocInfo;
    }
#endif

    VkResult result = loader->vkAllocateMemory(vulkanDevice, &bufMemAllocInfo, nullptr, &ptr->memoryVk);
    if (result == VK_SUCCESS)
    {
        context->deviceQueue->device->logPrint(eNvFlowLogLevel_info, "Buffer mem allocate %lld bytes", bufMemReq.size);
    }
    else
    {
        bufMemAllocInfo.memoryTypeIndex = bufMemType_sysmem;
        result = loader->vkAllocateMemory(vulkanDevice, &bufMemAllocInfo, nullptr, &ptr->memoryVk);
        if (result == VK_SUCCESS)
        {
            context->deviceQueue->device->logPrint(eNvFlowLogLevel_info, "Buffer sysmem fallback allocate %lld bytes", bufMemReq.size);
        }
        else
        {
            context->deviceQueue->device->logPrint(eNvFlowLogLevel_info, "Buffer allocate failed %lld bytes", bufMemReq.size);
        }
    }

    if (result == VK_SUCCESS)
    {
        ptr->allocationBytes = bufMemReq.size;
        device_reportMemoryAllocate(context->deviceQueue->device, ptr->memoryType, ptr->allocationBytes);

        loader->vkBindBufferMemory(vulkanDevice, ptr->bufferVk, ptr->memoryVk, 0u);

        if (ptr->memoryType == eNvFlowMemoryType_upload ||
            ptr->memoryType == eNvFlowMemoryType_readback)
        {
            loader->vkMapMemory(vulkanDevice, ptr->memoryVk, 0u, VK_WHOLE_SIZE, 0u, &ptr->mappedData);
        }
    }
    else // free buffer and set null
    {
        loader->vkDestroyBuffer(loader->device, ptr->bufferVk, nullptr);
        ptr->bufferVk = VK_NULL_HANDLE;
    }
}

void buffer_createBufferView(Context* context, Buffer* ptr, VkBufferView* view)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkBufferViewCreateInfo bufViewCreateInfo = {};
    bufViewCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_VIEW_CREATE_INFO;
    bufViewCreateInfo.buffer = ptr->bufferVk;
    bufViewCreateInfo.format = formatConverter_convertToVulkan(context->deviceQueue->device->formatConverter, ptr->desc.format);
    bufViewCreateInfo.offset = 0u;
    bufViewCreateInfo.range = VK_WHOLE_SIZE;

    if (bufViewCreateInfo.format == VK_FORMAT_UNDEFINED)
    {
        return;
    }

    loader->vkCreateBufferView(vulkanDevice, &bufViewCreateInfo, nullptr, view);
}

VkBufferView buffer_getBufferView(Context* context, Buffer* ptr, NvFlowFormat aliasFormat)
{
    if (aliasFormat == eNvFlowFormat_unknown)
    {
        return ptr->bufferViewVk;
    }
    for (NvFlowUint64 idx = 0u; idx < ptr->aliasFormats.size; idx++)
    {
        if (ptr->aliasFormats[idx] == aliasFormat)
        {
            return ptr->aliasBufferViews[idx];
        }
    }
    VkBufferView view = VK_NULL_HANDLE;
    buffer_createBufferView(context, ptr, &view);
    ptr->aliasFormats.pushBack(aliasFormat);
    ptr->aliasBufferViews.pushBack(view);
    return buffer_getBufferView(context, ptr, aliasFormat);
}

void buffer_initRestoreBarrier(Context* context, Buffer* ptr)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkBufferMemoryBarrier barrier = {};
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    barrier.pNext = nullptr;
    barrier.srcAccessMask = 0u;
    barrier.dstAccessMask = 0u;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = ptr->bufferVk;
    barrier.offset = 0u;
    barrier.size = VK_WHOLE_SIZE;

    VkAccessFlags accessFlags = 0u;
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_constantBuffer)
    {
        accessFlags |= VK_ACCESS_UNIFORM_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_structuredBuffer)
    {
        accessFlags |= VK_ACCESS_SHADER_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_buffer)
    {
        accessFlags |= VK_ACCESS_SHADER_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_rwStructuredBuffer)
    {
        accessFlags |= VK_ACCESS_SHADER_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_rwBuffer)
    {
        accessFlags |= VK_ACCESS_SHADER_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_indirectBuffer)
    {
        accessFlags |= VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_bufferCopySrc)
    {
        accessFlags |= VK_ACCESS_SHADER_READ_BIT;
    }
    if (ptr->desc.usageFlags & eNvFlowBufferUsage_bufferCopyDst)
    {
        accessFlags |= VK_ACCESS_SHADER_READ_BIT;
    }
    barrier.srcAccessMask = accessFlags;
    barrier.dstAccessMask = accessFlags;

    ptr->restoreBarrier = barrier;
}

void buffer_initCurrentBarrier(Context* context, Buffer* ptr)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkBufferMemoryBarrier barrier = ptr->restoreBarrier;
    barrier.srcAccessMask = 0u;
    barrier.dstAccessMask = 0u;

    ptr->currentBarrier = barrier;
}

Buffer* buffer_create(Context* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, const NvFlowInteropHandle* interopHandle)
{
    auto ptr = new Buffer();

    ptr->desc = *desc;
    ptr->memoryType = memoryType;

    buffer_createBuffer(context, ptr, interopHandle);
    if (!ptr->bufferVk)
    {
        delete ptr;
        return nullptr;
    }
    buffer_createBufferView(context, ptr, &ptr->bufferViewVk);

    buffer_initRestoreBarrier(context, ptr);
    buffer_initCurrentBarrier(context, ptr);

    return ptr;
}

void buffer_destroy(Context* context, Buffer* ptr)
{
    auto loader = &context->deviceQueue->device->loader;

    loader->vkDestroyBufferView(loader->device, ptr->bufferViewVk, nullptr);
    for (NvFlowUint64 idx = 0u; idx < ptr->aliasBufferViews.size; idx++)
    {
        loader->vkDestroyBufferView(loader->device, ptr->aliasBufferViews[idx], nullptr);
    }
    ptr->mappedData = nullptr;

    // use memoryVk as indicator of ownership of original buffer
    if (ptr->memoryVk)
    {
        loader->vkDestroyBuffer(loader->device, ptr->bufferVk, nullptr);
        loader->vkFreeMemory(loader->device, ptr->memoryVk, nullptr);

        device_reportMemoryFree(context->deviceQueue->device, ptr->memoryType, ptr->allocationBytes);
    }

    delete ptr;
}

NvFlowBool32 bufferDesc_compare(const NvFlowBufferDesc* a, const NvFlowBufferDesc* b)
{
    if (a->usageFlags == b->usageFlags &&
        a->format == b->format &&
        a->sizeInBytes == b->sizeInBytes)
    {
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

NvFlowBuffer* createBuffer(NvFlowContext* contextIn, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc)
{
    auto context = cast(contextIn);

    for (NvFlowUint idx = 0u; idx < context->pool_buffers.size; idx++)
    {
        auto ptr = context->pool_buffers[idx];
        if (ptr && ptr->refCount == 0 && bufferDesc_compare(&ptr->desc, desc) && ptr->memoryType == memoryType)
        {
            ptr->refCount = 1;
            ptr->lastActive = context->deviceQueue->nextFenceValue;
            return cast(ptr);
        }
    }

    auto ptr = buffer_create(context, memoryType, desc, nullptr);

    ptr->refCount = 1;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
    context->pool_buffers.pushBack(ptr);

    return cast(ptr);
}

NvFlowBuffer* createBufferFromExternalHandle(NvFlowContext* contextIn, const NvFlowBufferDesc* desc, const NvFlowInteropHandle* interopHandle)
{
    auto context = cast(contextIn);

    // do not recycle from pool for external resources

    auto ptr = buffer_create(context, eNvFlowMemoryType_device, desc, interopHandle);
    if (!ptr)
    {
        return nullptr;
    }

    ptr->refCount = 1;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
    context->pool_buffers.pushBack(ptr);

    return cast(ptr);
}

void destroyBuffer(NvFlowContext* contextIn, NvFlowBuffer* buffer)
{
    auto context = cast(contextIn);
    auto ptr = cast(buffer);

    if (ptr->refCount <= 0)
    {
        context->logPrint(eNvFlowLogLevel_error, "NvFlowContext::destroyBuffer() called on inactive buffer %p", buffer);
    }

    ptr->refCount--;
    ptr->lastActive = context->deviceQueue->nextFenceValue;
}

void context_destroyBuffers(Context* context)
{
    context->bufferTransients.deletePointers();
    context->bufferAcquires.deletePointers();

    for (NvFlowUint idx = 0u; idx < context->pool_buffers.size; idx++)
    {
        auto& ptr = context->pool_buffers[idx];
        buffer_destroy(context, ptr);
        ptr = nullptr;
    }
}

NvFlowBufferTransient* getBufferTransient(NvFlowContext* contextIn, const NvFlowBufferDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = context->bufferTransients.allocateBackPointer();
    ptr->desc = *desc;
    ptr->buffer = nullptr;
    ptr->aliasBuffer = nullptr;
    ptr->aliasFormat = eNvFlowFormat_unknown;
    ptr->nodeBegin = 0;
    ptr->nodeEnd = 0;
    return cast(ptr);
}

NvFlowBufferTransient* registerBufferAsTransient(NvFlowContext* contextIn, NvFlowBuffer* buffer)
{
    auto context = cast(contextIn);
    auto ptr = context->bufferTransients.allocateBackPointer();
    ptr->desc = cast(buffer)->desc;
    ptr->buffer = cast(buffer);
    ptr->aliasBuffer = nullptr;
    ptr->aliasFormat = eNvFlowFormat_unknown;
    ptr->nodeBegin = 0;
    ptr->nodeEnd = 0;
    return cast(ptr);
}

NvFlowBufferTransient* aliasBufferTransient(NvFlowContext* contextIn, NvFlowBufferTransient* buffer, NvFlowFormat format, NvFlowUint structureStride)
{
    auto context = cast(contextIn);
    auto ptr = context->bufferTransients.allocateBackPointer();
    ptr->desc = cast(buffer)->desc;
    ptr->buffer = nullptr;
    ptr->aliasBuffer = cast(buffer);
    ptr->aliasFormat = format;
    ptr->nodeBegin = 0;
    ptr->nodeEnd = 0;
    return cast(ptr);
}

NvFlowBufferAcquire* enqueueAcquireBuffer(NvFlowContext* contextIn, NvFlowBufferTransient* bufferTransient)
{
    auto context = cast(contextIn);
    auto ptr = context->bufferAcquires.allocateBackPointer();
    ptr->bufferTransient = cast(bufferTransient);
    ptr->buffer = nullptr;
    return cast(ptr);
}

NvFlowBool32 getAcquiredBuffer(NvFlowContext* contextIn, NvFlowBufferAcquire* acquire, NvFlowBuffer** outBuffer)
{
    auto context = cast(contextIn);
    auto ptr = cast(acquire);
    if (ptr->buffer)
    {
        *outBuffer = cast(ptr->buffer);

        // remove from acquire array
        context->bufferAcquires.removeSwapPointer(ptr);

        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

void* mapBuffer(NvFlowContext* context, NvFlowBuffer* buffer)
{
    return (cast(buffer))->mappedData;
}

void unmapBuffer(NvFlowContext* context, NvFlowBuffer* buffer)
{
    // NOP
}

NvFlowBufferTransient* getBufferTransientById(NvFlowContext* context, NvFlowUint64 bufferId)
{
    auto ctx = cast(context);
    for (NvFlowUint idx = 0u; idx < ctx->registeredResources.size; idx++)
    {
        if (ctx->registeredResources[idx].uid == bufferId)
        {
            if (ctx->registeredResources[idx].buffer)
            {
                return registerBufferAsTransient(context, ctx->registeredResources[idx].buffer);
            }
        }
    }
    return nullptr;
}

void device_getBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, void* dstHandle, NvFlowUint64 dstHandleSize, NvFlowUint64* pBufferSizeInBytes)
{
    auto ctx = cast(context);
    auto ptr = cast(buffer);
#if defined(_WIN32)
    HANDLE handle = {};
    VkMemoryGetWin32HandleInfoKHR handleInfo = {};
    handleInfo.sType = VK_STRUCTURE_TYPE_MEMORY_GET_WIN32_HANDLE_INFO_KHR;
    handleInfo.memory = ptr->memoryVk;
    handleInfo.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR;

    ctx->deviceQueue->device->loader.vkGetMemoryWin32HandleKHR(ctx->deviceQueue->device->vulkanDevice, &handleInfo, &handle);

    memset(dstHandle, 0, dstHandleSize);
    if (dstHandleSize >= sizeof(handle))
    {
        memcpy(dstHandle, &handle, sizeof(handle));
    }
#else
    int fd = 0;
    VkMemoryGetFdInfoKHR handleInfo = {};
    handleInfo.sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR;
    handleInfo.memory = ptr->memoryVk;
    handleInfo.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    ctx->deviceQueue->device->loader.vkGetMemoryFdKHR(ctx->deviceQueue->device->vulkanDevice, &handleInfo, &fd);

    memset(dstHandle, 0, dstHandleSize);
    if (dstHandleSize >= sizeof(fd))
    {
        memcpy(dstHandle, &fd, sizeof(fd));
    }
#endif
    if (pBufferSizeInBytes)
    {
        *pBufferSizeInBytes = ptr->desc.sizeInBytes;
    }
}

void device_closeBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, const void* srcHandle, NvFlowUint64 srcHandleSize)
{
    auto ctx = cast(context);
    auto ptr = cast(buffer);
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

void getBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, NvFlowInteropHandle* dstHandle)
{
#if defined(_WIN32)
    dstHandle->type = eNvFlowInteropHandleType_opaqueWin32;
#else
    dstHandle->type = eNvFlowInteropHandleType_opaqueFd;
#endif
    device_getBufferExternalHandle(context, buffer, &dstHandle->value, sizeof(dstHandle->value), &dstHandle->resourceSizeInBytes);
}

void closeBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, const NvFlowInteropHandle* srcHandle)
{
    device_closeBufferExternalHandle(context, buffer, &srcHandle->value, sizeof(srcHandle->value));
}

} // end namespace
