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

namespace NvFlowCPU
{
    void getContextConfig(NvFlowContext* context, NvFlowContextConfig* config)
    {
        config->api = eNvFlowContextApi_cpu;
        config->textureBinding = eNvFlowTextureBindingType_separateSampler;
    }

    NvFlowUint64 getCurrentFrame(NvFlowContext* context)
    {
        Context* ctx = cast(context);
        return ctx->deviceQueue->nextFenceValue;
    }

    NvFlowUint64 getLastFrameCompleted(NvFlowContext* context)
    {
        Context* ctx = cast(context);
        return ctx->deviceQueue->lastFenceCompleted;
    }

    NvFlowLogPrint_t getLogPrint(NvFlowContext* context)
    {
        Context* ctx = cast(context);
        return ctx->logPrint;
    }

    void executeTasks(NvFlowContext* context, NvFlowUint taskCount, NvFlowUint taskGranularity, NvFlowContextThreadPoolTask_t task, void* userdata)
    {
        Context* ctx = cast(context);
        ctx->threadPoolInterface->execute(ctx->threadPool, taskCount, taskGranularity, task, userdata);
    }
}

NvFlowContextInterface* NvFlowGetContextInterface_cpu()
{
    using namespace NvFlowCPU;
    static NvFlowContextInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowContextInterface) };

    iface.getContextConfig = getContextConfig;
    iface.getCurrentFrame = getCurrentFrame;
    iface.getLastFrameCompleted = getLastFrameCompleted;
    iface.getCurrentGlobalFrame = getCurrentFrame;
    iface.getLastGlobalFrameCompleted = getLastFrameCompleted;
    iface.getLogPrint = getLogPrint;

    iface.executeTasks = executeTasks;

    iface.createBuffer = createBuffer;
    iface.destroyBuffer = destroyBuffer;
    iface.getBufferTransient = getBufferTransient;
    iface.registerBufferAsTransient = registerBufferAsTransient;
    iface.enqueueAcquireBuffer = enqueueAcquireBuffer;
    iface.getAcquiredBuffer = getAcquiredBuffer;
    iface.mapBuffer = mapBuffer;
    iface.unmapBuffer = unmapBuffer;
    iface.getBufferTransientById = getBufferTransientById;

    iface.createTexture = createTexture;
    iface.destroyTexture = destroyTexture;
    iface.getTextureTransient = getTextureTransient;
    iface.registerTextureAsTransient = registerTextureAsTransient;
    iface.enqueueAcquireTexture = enqueueAcquireTexture;
    iface.getAcquiredTexture = getAcquiredTexture;
    iface.getTextureTransientById = getTextureTransientById;

    iface.createSampler = createSampler;
    iface.getDefaultSampler = getDefaultSampler;
    iface.destroySampler = destroySampler;

    iface.createComputePipeline = createComputePipeline;
    iface.destroyComputePipeline = destroyComputePipeline;

    iface.addPassCompute = addPassCompute;
    iface.addPassCopyBuffer = addPassCopyBuffer;
    iface.addPassCopyBufferToTexture = addPassCopyBufferToTexture;
    iface.addPassCopyTextureToBuffer = addPassCopyTextureToBuffer;
    iface.addPassCopyTexture = addPassCopyTexture;

    return &iface;
}

NvFlowDeviceInterface* NvFlowGetDeviceInterface_cpu()
{
    using namespace NvFlowCPU;
    static NvFlowDeviceInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowDeviceInterface) };

    iface.createDeviceManager = createDeviceManager;
    iface.destroyDeviceManager = destroyDeviceManager;
    iface.enumerateDevices = enumerateDevices;

    iface.createDevice = createDevice;
    iface.destroyDevice = destroyDevice;

    iface.createSemaphore = createSemaphore;
    iface.destroySemaphore = destroySemaphore;
    iface.getSemaphoreExternalHandle = getSemaphoreExternalHandle;
    iface.closeSemaphoreExternalHandle = closeSemaphoreExternalHandle;

    iface.getDeviceQueue = getDeviceQueue;
    iface.flush = flush;
    iface.getLastFrameCompleted = getLastFrameCompleted;
    iface.waitForFrame = waitForFrame;
    iface.waitIdle = waitIdle;
    iface.getContextInterface = getContextInterface;
    iface.getContext = getContext;

    iface.createSwapchain = createSwapchain;
    iface.destroySwapchain = destroySwapchain;
    iface.resizeSwapchain = resizeSwapchain;
    iface.presentSwapchain = presentSwapchain;
    iface.getSwapchainFrontTexture = getSwapchainFrontTexture;

    iface.enableProfiler = enableProfiler;
    iface.disableProfiler = disableProfiler;

    iface.registerBufferId = registerBufferId;
    iface.registerTextureId = registerTextureId;
    iface.unregisterBufferId = unregisterBufferId;
    iface.unregisterTextureId = unregisterTextureId;
    iface.setResourceMinLifetime = setResourceMinLifetime;

    return &iface;
}
