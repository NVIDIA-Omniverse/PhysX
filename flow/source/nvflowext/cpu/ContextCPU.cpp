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

#if defined(_WIN32)
// Windows.h already included
#else
#include <time.h>
#endif

namespace NvFlowCPU
{

Context* context_create(DeviceQueue* deviceQueue)
{
    auto ptr = new Context();

    ptr->logPrint = deviceQueue->device->logPrint;

    ptr->deviceQueue = deviceQueue;

    NvFlowSamplerDesc samplerDesc = {};
    samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
    samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
    samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;
    samplerDesc.filterMode = eNvFlowSamplerFilterMode_point;

    createSampler(cast(ptr), &samplerDesc);

    ptr->profiler = profiler_create(ptr);

    ptr->threadPoolInterface = &deviceQueue->device->deviceManager->threadPoolInterface;
    ptr->threadPool = ptr->threadPoolInterface->create(deviceQueue->device->deviceManager->threadCount, 0llu);

    return ptr;
}

void context_destroy(Context* ptr)
{
    ptr->threadPoolInterface->destroy(ptr->threadPool);

    profiler_destroy(ptr, ptr->profiler);

    context_destroyBuffers(ptr);
    context_destroyTextures(ptr);
    context_destroySamplers(ptr);

    delete ptr;
}

void addPassCompute(NvFlowContext* contextIn, const NvFlowPassComputeParams* params)
{
    auto context = cast(contextIn);

    computePipeline_dispatch(context, params);

    profiler_timestamp(context, context->profiler, params->debugLabel);
}

void addPassCopyBuffer(NvFlowContext* contextIn, const NvFlowPassCopyBufferParams* params)
{
    auto context = cast(contextIn);

    unsigned char* dst_data = (unsigned char*)cast(params->dst)->buffer->resource.data;
    unsigned char* src_data = (unsigned char*)cast(params->src)->buffer->resource.data;

    dst_data += params->dstOffset;
    src_data += params->srcOffset;

    memcpy(dst_data, src_data, params->numBytes);

    profiler_timestamp(context, context->profiler, params->debugLabel);
}

void addPassCopyBufferToTexture(NvFlowContext* contextIn, const NvFlowPassCopyBufferToTextureParams* params)
{
    auto context = cast(contextIn);

    // HACK: assuming rgba8 to float
    if (cast(params->dst)->texture->desc.format == eNvFlowFormat_r8g8b8a8_unorm)
    {
        NvFlowFloat4* dst_data = (NvFlowFloat4*)cast(params->dst)->texture->resource.data;
        NvFlowUint* src_data = (NvFlowUint*)cast(params->src)->buffer->resource.data;

        NvFlowUint copyElements = params->textureExtent.x * params->textureExtent.y * params->textureExtent.z;
        for (NvFlowUint idx = 0u; idx < copyElements; idx++)
        {
            NvFlowUint valIn = src_data[idx];
            NvFlowFloat4 valOut = {
                float((valIn >> 0) & 255) * (1.f / 255.f),
                float((valIn >> 8) & 255) * (1.f / 255.f),
                float((valIn >> 16) & 255) * (1.f / 255.f),
                float((valIn >> 24) & 255) * (1.f / 255.f)
            };
            dst_data[idx] = valOut;
        }
    }

    profiler_timestamp(context, context->profiler, params->debugLabel);
}

void addPassCopyTextureToBuffer(NvFlowContext* contextIn, const NvFlowPassCopyTextureToBufferParams* params)
{
    auto context = cast(contextIn);

    profiler_timestamp(context, context->profiler, params->debugLabel);
}

void addPassCopyTexture(NvFlowContext* contextIn, const NvFlowPassCopyTextureParams* params)
{
    auto context = cast(contextIn);

    profiler_timestamp(context, context->profiler, params->debugLabel);
}

void context_flush(Context* context)
{
    profiler_timestamp(context, context->profiler, "EndCapture");

    profiler_flush(context, context->profiler);

    // process buffer acquires
    {
        for (NvFlowUint idx = 0u; idx < context->bufferAcquires.size; idx++)
        {
            auto bufferAcquire = context->bufferAcquires[idx];
            if (!bufferAcquire->buffer)
            {
                bufferAcquire->buffer = bufferAcquire->bufferTransient->buffer;
                bufferAcquire->buffer->activeMask |= 1u;
                bufferAcquire->bufferTransient = nullptr;
            }
        }
    }
    // process texture acquires
    {
        for (NvFlowUint idx = 0u; idx < context->textureAcquires.size; idx++)
        {
            auto textureAcquire = context->textureAcquires[idx];
            if (!textureAcquire->texture)
            {
                textureAcquire->texture = textureAcquire->textureTransient->texture;
                textureAcquire->texture->activeMask |= 1u;
                textureAcquire->textureTransient = nullptr;
            }
        }
    }

    // free transient buffers
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        auto buffer = context->bufferTransients[idx];
        buffer->buffer->activeMask &= ~2u;
        buffer->buffer = nullptr;
    }
    // free transient textures
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        auto texture = context->textureTransients[idx];
        texture->texture->activeMask &= ~2u;
        texture->texture = nullptr;
    }

    // reset transient arrays
    context->bufferTransients.size = 0u;
    context->textureTransients.size = 0u;
}

/// ***************************** Profiler *****************************************************

Profiler* profiler_create(Context* context)
{
    auto ptr = new Profiler();

    profiler_flush(context, ptr);

    return ptr;
}

void profiler_destroy(Context* context, Profiler* ptr)
{
    delete ptr;
}

void profiler_flush(Context* context, Profiler* ptr)
{
    // process current capture
    {
        ptr->deltaEntries.reserve(ptr->entries.size);
        ptr->deltaEntries.size = ptr->entries.size;

        // compute time deltas
        ProfilerEntry prevEntry = {};
        if (ptr->entries.size > 0u)
        {
            prevEntry = ptr->entries[0u];
        }
        for (NvFlowUint idx = 0u; idx < ptr->entries.size; idx++)
        {
            auto entry = &ptr->entries[idx];
            auto deltaEntry = &ptr->deltaEntries[idx];

            deltaEntry->label = entry->label;
            deltaEntry->cpuDeltaTime = (float)(((double)(entry->cpuValue - prevEntry.cpuValue) / (double)(ptr->cpuFreq)));
            deltaEntry->gpuDeltaTime = 0.f;

            prevEntry = *entry;
        }

        if (ptr->reportEntries)
        {
            ptr->reportEntries(ptr->userdata, ptr->captureID, (NvFlowUint)ptr->deltaEntries.size, ptr->deltaEntries.data);
        }
    }

    // start next capture
    {
#if defined(_WIN32)
        LARGE_INTEGER tmpCpuFreq = {};
        QueryPerformanceFrequency(&tmpCpuFreq);
        ptr->cpuFreq = tmpCpuFreq.QuadPart;
#else
        ptr->cpuFreq = 1E9;
#endif

        ptr->captureID++;
        ptr->entries.size = 0u;
        ptr->deltaEntries.size = 0u;
    }
}

void profiler_timestamp(Context* context, Profiler* ptr, const char* label)
{
    if (!ptr->reportEntries)
    {
        return;
    }

    if (ptr->entries.size == 0u && label != ptr->beginCapture)
    {
        profiler_timestamp(context, context->profiler, ptr->beginCapture);
    }

    NvFlowUint64 entryIdx = ptr->entries.allocateBack();

    auto& entry = ptr->entries[entryIdx];

    entry.label = label;
    entry.cpuValue = 0llu;

#if defined(_WIN32)
    LARGE_INTEGER tmpCpuTime = {};
    QueryPerformanceCounter(&tmpCpuTime);
    entry.cpuValue = tmpCpuTime.QuadPart;
#else
    timespec timeValue = {};
    clock_gettime(CLOCK_MONOTONIC, &timeValue);
    entry.cpuValue = 1E9 * NvFlowUint64(timeValue.tv_sec) + NvFlowUint64(timeValue.tv_nsec);
#endif
}

void enableProfiler(NvFlowContext* contextIn, void* userdata, void(NV_FLOW_ABI* reportEntries)(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries))
{
    auto context = cast(contextIn);

    context->profiler->userdata = userdata;
    context->profiler->reportEntries = reportEntries;
}

void disableProfiler(NvFlowContext* contextIn)
{
    auto context = cast(contextIn);

    context->profiler->reportEntries = nullptr;
}

NvFlowUint64 registerResourceId(NvFlowContext* context, NvFlowBuffer* buffer, NvFlowTexture* texture)
{
    auto ctx = cast(context);

    // check for existing
    for (NvFlowUint idx = 0u; idx < ctx->registeredResources.size; idx++)
    {
        if (ctx->registeredResources[idx].buffer == buffer && ctx->registeredResources[idx].texture == texture)
        {
            return ctx->registeredResources[idx].uid;
        }
    }

    ctx->registeredResourceCounter++;

    RegisteredResource resource = {};
    resource.buffer = buffer;
    resource.texture = texture;
    resource.uid = ctx->registeredResourceCounter;
    ctx->registeredResources.pushBack(resource);

    return resource.uid;
}

void unregisterResourceId(NvFlowContext* context, NvFlowUint64 resourceId)
{
    auto ctx = cast(context);
    NvFlowUint dstIdx = 0u;
    for (NvFlowUint srcIdx = 0u; srcIdx < ctx->registeredResources.size; srcIdx++)
    {
        if (ctx->registeredResources[srcIdx].uid != resourceId)
        {
            if (dstIdx != srcIdx)
            {
                ctx->registeredResources[dstIdx] = ctx->registeredResources[srcIdx];
            }
            dstIdx++;
        }
    }
    ctx->registeredResources.size = dstIdx;
}

NvFlowUint64 registerBufferId(NvFlowContext* context, NvFlowBuffer* buffer)
{
    return registerResourceId(context, buffer, nullptr);
}

NvFlowUint64 registerTextureId(NvFlowContext* context, NvFlowTexture* texture)
{
    return registerResourceId(context, nullptr, texture);
}

void unregisterBufferId(NvFlowContext* context, NvFlowUint64 bufferId)
{
    unregisterResourceId(context, bufferId);
}

void unregisterTextureId(NvFlowContext* context, NvFlowUint64 textureId)
{
    unregisterResourceId(context, textureId);
}

void setResourceMinLifetime(NvFlowContext* context, NvFlowUint64 minLifetime)
{
    // NOP, CPU does not release yet
}

} // end namespace
