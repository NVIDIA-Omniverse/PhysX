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

#include "NvFlowContext.h"
#include "NvFlowArray.h"

struct NvFlowDynamicBuffer
{
    NvFlowContextInterface* contextInterface = nullptr;
    NvFlowBufferUsageFlags flags = 0u;
    NvFlowFormat format = eNvFlowFormat_unknown;
    NvFlowUint structureStride = 0u;

    NvFlowBuffer* deviceBuffer = nullptr;
    NvFlowUint64 deviceNumBytes = 0llu;

    NvFlowBufferTransient* transientBuffer = nullptr;
    NvFlowUint64 transientFrame = ~0llu;
};

NV_FLOW_INLINE void NvFlowDynamicBuffer_init(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowDynamicBuffer* ptr, NvFlowBufferUsageFlags flags, NvFlowFormat format, NvFlowUint structureStride)
{
    ptr->contextInterface = contextInterface;

    ptr->flags = flags;
    ptr->format = format;
    ptr->structureStride = structureStride;
}

NV_FLOW_INLINE void NvFlowDynamicBuffer_destroy(NvFlowContext* context, NvFlowDynamicBuffer* ptr)
{
    if (ptr->deviceBuffer)
    {
        ptr->contextInterface->destroyBuffer(context, ptr->deviceBuffer);
        ptr->deviceBuffer = nullptr;
    }
}

NV_FLOW_INLINE void NvFlowDynamicBuffer_resize(NvFlowContext* context, NvFlowDynamicBuffer* ptr, NvFlowUint64 numBytes)
{
    if (ptr->deviceBuffer && ptr->deviceNumBytes < numBytes)
    {
        ptr->contextInterface->destroyBuffer(context, ptr->deviceBuffer);
        ptr->deviceBuffer = nullptr;
        ptr->deviceNumBytes = 0llu;
        ptr->transientBuffer = nullptr;
        ptr->transientFrame = ~0llu;
    }
    if (!ptr->deviceBuffer)
    {
        NvFlowBufferDesc bufDesc = {};
        bufDesc.format = ptr->format;
        bufDesc.usageFlags = ptr->flags;
        bufDesc.structureStride = ptr->structureStride;
        bufDesc.sizeInBytes = 65536u;
        while (bufDesc.sizeInBytes < numBytes)
        {
            bufDesc.sizeInBytes *= 2u;
        }

        ptr->deviceNumBytes = bufDesc.sizeInBytes;
        ptr->deviceBuffer = ptr->contextInterface->createBuffer(context, eNvFlowMemoryType_device, &bufDesc);
        ptr->transientBuffer = nullptr;
        ptr->transientFrame = ~0llu;
    }
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowDynamicBuffer_getTransient(NvFlowContext* context, NvFlowDynamicBuffer* ptr)
{
    if (ptr->transientBuffer &&
        ptr->transientFrame == ptr->contextInterface->getCurrentFrame(context))
    {
        return ptr->transientBuffer;
    }
    if (ptr->deviceBuffer)
    {
        ptr->transientBuffer = ptr->contextInterface->registerBufferAsTransient(context, ptr->deviceBuffer);
        ptr->transientFrame = ptr->contextInterface->getCurrentFrame(context);
    }
    return ptr->transientBuffer;
}
