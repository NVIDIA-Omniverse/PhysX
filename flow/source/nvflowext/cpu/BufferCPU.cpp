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

Buffer* buffer_create(Context* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc)
{
    auto ptr = new Buffer();

    ptr->desc = *desc;

    ptr->resource.data = malloc(ptr->desc.sizeInBytes);
    ptr->resource.sizeInBytes = ptr->desc.sizeInBytes;
    ptr->resource.elementSizeInBytes = desc->structureStride;
    ptr->resource.elementCount = desc->structureStride ? NvFlowUint(ptr->desc.sizeInBytes / desc->structureStride) : 0u;
    ptr->resource.width = ptr->resource.elementCount;
    ptr->resource.height = 1u;
    ptr->resource.depth = 1u;

    return ptr;
}

void buffer_destroy(Context* context, Buffer* ptr)
{
    free(ptr->resource.data);
    ptr->resource.data = nullptr;

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
        if (ptr && !ptr->activeMask && bufferDesc_compare(&ptr->desc, desc))
        {
            ptr->activeMask = 1u;
            return cast(ptr);
        }
    }

    auto ptr = buffer_create(context, memoryType, desc);

    ptr->activeMask = 1u;
    context->pool_buffers.pushBack(ptr);

    return cast(ptr);
}

void destroyBuffer(NvFlowContext* contextIn, NvFlowBuffer* buffer)
{
    auto context = cast(contextIn);
    auto ptr = cast(buffer);

    ptr->activeMask &= ~1u;
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
    ptr->buffer = cast(createBuffer(contextIn, eNvFlowMemoryType_device, &ptr->desc));
    ptr->buffer->activeMask = 2u;
    return cast(ptr);
}

NvFlowBufferTransient* registerBufferAsTransient(NvFlowContext* contextIn, NvFlowBuffer* buffer)
{
    auto context = cast(contextIn);
    auto ptr = context->bufferTransients.allocateBackPointer();
    ptr->desc = cast(buffer)->desc;
    ptr->buffer = cast(buffer);
    ptr->buffer->activeMask |= 2u;
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
    return (cast(buffer))->resource.data;
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

} // end namespace
