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

#include "CommonCPU.h"

#include <string.h>

NvFlowContextInterface* NvFlowGetContextInterface_cpu();

namespace NvFlowCPU
{
/// ************************** Device Manager **************************************

NvFlowDeviceManager* createDeviceManager(NvFlowBool32 enableValidationOnDebugBuild, NvFlowThreadPoolInterface* threadPoolInterface, NvFlowUint threadCount)
{
    auto ptr = new DeviceManager();

    if (!threadPoolInterface)
    {
        threadPoolInterface = NvFlowGetThreadPoolInterface();
    }
    NvFlowThreadPoolInterface_duplicate(&ptr->threadPoolInterface, threadPoolInterface);
    ptr->threadCount = threadCount;

    NvFlowPhysicalDeviceDesc desc = {};
    ptr->physicalDeviceDescs.pushBack(desc);

    return cast(ptr);
}

void destroyDeviceManager(NvFlowDeviceManager* deviceManager)
{
    auto ptr = cast(deviceManager);

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

    ptr->deviceQueue = deviceQueue_create(ptr);

    return cast(ptr);
}

void destroyDevice(NvFlowDeviceManager* manager, NvFlowDevice* device)
{
    auto ptr = cast(device);

    deviceQueue_destroy(ptr->deviceQueue);

    delete ptr;
}

NvFlowDeviceQueue* getDeviceQueue(NvFlowDevice* device)
{
    auto ptr = cast(device);
    return cast(ptr->deviceQueue);
}

/// ************************** DeviceSemaphore **************************************

NvFlowDeviceSemaphore* createSemaphore(NvFlowDevice* device)
{
    auto ptr = new DeviceSemaphore();
    ptr->reserved = 0;
    return cast(ptr);
}

void destroySemaphore(NvFlowDeviceSemaphore* semaphore)
{
    auto ptr = cast(semaphore);

    delete ptr;
}

void getSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, void* dstHandle, NvFlowUint64 dstHandleSize)
{
    memset(dstHandle, 0, dstHandleSize);
}

void closeSemaphoreExternalHandle(NvFlowDeviceSemaphore* semaphore, const void* srcHandle, NvFlowUint64 srcHandleSize)
{
    // NOP
}

/// ************************** DeviceQueue **************************************

DeviceQueue* deviceQueue_create(Device* device)
{
    auto ptr = new DeviceQueue();

    ptr->device = device;

    ptr->context = context_create(ptr);

    return ptr;
}

void deviceQueue_destroy(DeviceQueue* ptr)
{
    // Wait idle, since context destroy will force destroy resources
    waitIdle(cast(ptr));

    context_destroy(ptr->context);

    ptr->device = nullptr;

    delete ptr;
}

int flush(NvFlowDeviceQueue* deviceQueue, NvFlowUint64* flushedFrameID, NvFlowDeviceSemaphore* waitSemaphore, NvFlowDeviceSemaphore* signalSemaphore)
{
    auto ptr = cast(deviceQueue);

    *flushedFrameID = ptr->nextFenceValue;

    if (ptr->context)
    {
        context_flush(ptr->context);
    }

    // advance fence
    ptr->lastFenceCompleted = ptr->nextFenceValue;
    ptr->nextFenceValue++;

    int ret = 0;
    return ret;
}

void waitIdle(NvFlowDeviceQueue* deviceQueue)
{
    auto ptr = cast(deviceQueue);
    // NOP
}

void waitForFrame(NvFlowDeviceQueue* deviceQueue, NvFlowUint64 frameID)
{
    auto ptr = cast(deviceQueue);
    // NOP
}

NvFlowUint64 getLastFrameCompleted(NvFlowDeviceQueue* queue)
{
    auto ptr = cast(queue);
    return ptr->lastFenceCompleted;
}

NvFlowContextInterface* getContextInterface(NvFlowDeviceQueue* ptr)
{
    return NvFlowGetContextInterface_cpu();
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

    swapchain_getWindowSize(ptr, &ptr->width, &ptr->height);

    NvFlowTextureDesc texDesc = {};
    texDesc.textureType = eNvFlowTextureType_2d;
    texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
    texDesc.format = ptr->desc.format;
    texDesc.width = ptr->width;
    texDesc.height = ptr->height;
    texDesc.depth = 1u;
    texDesc.mipLevels = 1u;
    texDesc.optimizedClearValue = NvFlowFloat4{ 0.f, 0.f, 0.f, 0.f };

    ptr->texture = createTexture(cast(ptr->deviceQueue->context), &texDesc);

    return cast(ptr);
}

void destroySwapchain(NvFlowSwapchain* swapchain)
{
    auto ptr = cast(swapchain);
    auto device = ptr->deviceQueue->device;

    if (ptr->texture)
    {
        destroyTexture(cast(ptr->deviceQueue->context), ptr->texture);
        ptr->texture;
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
        ptr->p_XGetWindowAttributes = (decltype(&XGetWindowAttributes))NvFlowGetProcAddress(ptr->moduleX11, "XGetWindowAttributes");
    }

    XWindowAttributes winAttr = {};
    ptr->p_XGetWindowAttributes(ptr->desc.dpy, ptr->desc.window, &winAttr);

    *width = winAttr.width;
    *height = winAttr.height;
#endif
}

void resizeSwapchain(NvFlowSwapchain* swapchain, NvFlowUint width, NvFlowUint height)
{
    auto ptr = cast(swapchain);

    if (width != ptr->width ||
        height != ptr->height)
    {
        if (ptr->texture)
        {
            destroyTexture(cast(ptr->deviceQueue->context), ptr->texture);
            ptr->texture;
        }

        swapchain_getWindowSize(ptr, &ptr->width, &ptr->height);

        NvFlowTextureDesc texDesc = {};
        texDesc.textureType = eNvFlowTextureType_2d;
        texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
        texDesc.format = ptr->desc.format;
        texDesc.width = ptr->width;
        texDesc.height = ptr->height;
        texDesc.depth = 1u;
        texDesc.mipLevels = 1u;
        texDesc.optimizedClearValue = NvFlowFloat4{ 0.f, 0.f, 0.f, 0.f };

        ptr->texture = createTexture(cast(ptr->deviceQueue->context), &texDesc);
    }
}

NV_FLOW_INLINE NvFlowUint packFloat4_convert(float v)
{
    if (v < 0.f) v = 0.f;
    if (v > 1.f) v = 1.f;
    return NvFlowUint(255.f * v);
}

NV_FLOW_INLINE NvFlowUint packFloat4(NvFlowFloat4 v)
{
    return packFloat4_convert(v.z) |
        (packFloat4_convert(v.y) << 8u) |
        (packFloat4_convert(v.x) << 16u) |
        (packFloat4_convert(v.w) << 24u);
}

struct SwapchainCopyTaskData
{
    NvFlowFloat4* src;
    NvFlowUint numPixels;
    NvFlowUint* mapped;
};

void swapchain_task(NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
{
    SwapchainCopyTaskData* task = (SwapchainCopyTaskData*)userdata;
    NvFlowUint idx_base = 256u * taskIdx;
    NvFlowUint idx_max = idx_base + 256u;
    if (idx_max > task->numPixels)
    {
        idx_max = task->numPixels;
    }
    for (NvFlowUint idx = idx_base; idx < idx_max; idx++)
    {
        task->mapped[idx] = packFloat4(task->src[idx]);
    }
}

int presentSwapchain(NvFlowSwapchain* swapchain, NvFlowBool32 vsync, NvFlowUint64* flushedFrameID)
{
    auto ptr = cast(swapchain);

    flush(cast(ptr->deviceQueue), flushedFrameID, nullptr, nullptr);

    *flushedFrameID = ptr->deviceQueue->nextFenceValue;

#if defined(_WIN32)
    HDC dc = GetDC(ptr->desc.hwnd);
    HDC dcMem = CreateCompatibleDC(dc);

    NvFlowUint* mapped = nullptr;

    BITMAPINFO bi = {};
    bi.bmiHeader.biSize = sizeof(BITMAPINFO);
    bi.bmiHeader.biWidth = int(ptr->width);
    bi.bmiHeader.biHeight = -int(ptr->height);
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;

    HBITMAP bitmap = CreateDIBSection(dcMem, &bi, DIB_RGB_COLORS, (LPVOID*)&mapped, 0, 0);
    HGDIOBJ oldbmp = SelectObject(dcMem, bitmap);

    NvFlowFloat4* src = (NvFlowFloat4*)(cast(ptr->texture)->resource.data);
    NvFlowUint numPixels = ptr->height * ptr->width;

    SwapchainCopyTaskData taskData = {};
    taskData.src = src;
    taskData.numPixels = numPixels;
    taskData.mapped = mapped;

    Context* context = ptr->deviceQueue->context;
    if (context)
    {
        NvFlowUint totalBlocks = (numPixels + 255u) / 256u;

        NvFlowUint targetBatchesPerThread = 32u;
        NvFlowUint threadCount = context->threadPoolInterface->getThreadCount(context->threadPool);
        NvFlowUint aveBlocksPerThread = totalBlocks / threadCount;
        NvFlowUint granularity = aveBlocksPerThread / targetBatchesPerThread;
        if (granularity == 0u)
        {
            granularity = 1u;
        }

        context->threadPoolInterface->execute(
            context->threadPool,
            totalBlocks,
            granularity,
            swapchain_task,
            &taskData
        );
    }

    BitBlt(dc, 0, 0, ptr->width, ptr->height, dcMem, 0, 0, SRCCOPY);

    SelectObject(dcMem, oldbmp);
    DeleteDC(dcMem);
    ReleaseDC(ptr->desc.hwnd, dc);
    DeleteObject(bitmap);
#else
    if (!ptr->moduleX11)
    {
        ptr->moduleX11 = NvFlowLoadLibrary("X11.dll", "libX11.so");
    }
    if (!ptr->p_XCreateImage)
    {
        ptr->p_XCreateImage = (decltype(&XCreateImage))NvFlowGetProcAddress(ptr->moduleX11, "XCreateImage");
        ptr->p_XCreateGC = (decltype(&XCreateGC))NvFlowGetProcAddress(ptr->moduleX11, "XCreateGC");
        ptr->p_XPutImage = (decltype(&XPutImage))NvFlowGetProcAddress(ptr->moduleX11, "XPutImage");
        ptr->p_XFreeGC = (decltype(&XFreeGC))NvFlowGetProcAddress(ptr->moduleX11, "XFreeGC");
    }

    auto display = ptr->desc.dpy;

    NvFlowUint* mapped = (NvFlowUint*)malloc(4u * ptr->width * ptr->height);

    NvFlowFloat4* src = (NvFlowFloat4*)(cast(ptr->texture)->resource.data);
    NvFlowUint numPixels = ptr->height * ptr->width;

    SwapchainCopyTaskData taskData = {};
    taskData.src = src;
    taskData.numPixels = numPixels;
    taskData.mapped = mapped;

    Context* context = ptr->deviceQueue->context;
    if (context)
    {
        NvFlowUint totalBlocks = (numPixels + 255u) / 256u;

        NvFlowUint targetBatchesPerThread = 32u;
        NvFlowUint threadCount = context->threadPoolInterface->getThreadCount(context->threadPool);
        NvFlowUint aveBlocksPerThread = totalBlocks / threadCount;
        NvFlowUint granularity = aveBlocksPerThread / targetBatchesPerThread;
        if (granularity == 0u)
        {
            granularity = 1u;
        }

        context->threadPoolInterface->execute(
            context->threadPool,
            totalBlocks,
            granularity,
            swapchain_task,
            &taskData
        );
    }

    XImage* image = ptr->p_XCreateImage(display,
        DefaultVisual(display, 0),
        DefaultDepth(display, 0),
        ZPixmap, 0, (char*)mapped, ptr->width, ptr->height, 32, 0
    );

    GC gc = ptr->p_XCreateGC(display, ptr->desc.window, 0, 0);

    ptr->p_XPutImage(display, ptr->desc.window, gc, image, 0, 0, 0, 0, ptr->width, ptr->height);

    ptr->p_XFreeGC(display, gc);

    XDestroyImage(image);
#endif

    return 0;
}

NvFlowTexture* getSwapchainFrontTexture(NvFlowSwapchain* swapchain)
{
    auto ptr = cast(swapchain);

    return ptr->texture;
}

} // end namespace
