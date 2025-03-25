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

struct NvFlowInteropBufferInstance
{
    NvFlowBuffer* buffer = nullptr;
    NvFlowUint64 bufferSizeInBytes = 0llu;
    NvFlowInteropHandle interopHandle = {};
    NvFlowBool32 isActive = NV_FLOW_FALSE;
    NvFlowUint64 completedFrame = ~0llu;
    NvFlowUint64 completedGlobalFrame = ~0llu;
    NvFlowUint64 version = ~0llu;
    NvFlowUint64 validNumBytes = 0llu;
};

struct NvFlowInteropBufferRetained
{
    NvFlowBuffer* buffer = nullptr;
    NvFlowInteropHandle interopHandle = {};
};

struct NvFlowInteropBuffer
{
    NvFlowContextInterface* contextInterface = nullptr;
    NvFlowUint64 versionCounter = 0llu;
    NvFlowUint ringBufferCount = 0u;

    NvFlowArray<NvFlowInteropBufferInstance, 8u> buffers;
    NvFlowArray<NvFlowUint64, 8u> activeBuffers;
    NvFlowArray<NvFlowInteropBufferRetained, 8u> retainedBuffers;
};

NV_FLOW_INLINE void NvFlowInteropBuffer_init(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowInteropBuffer* ptr)
{
    ptr->contextInterface = contextInterface;
}

NV_FLOW_INLINE void NvFlowInteropBuffer_destroy(NvFlowContext* context, NvFlowInteropBuffer* ptr)
{
    for (NvFlowUint idx = 0u; idx < ptr->buffers.size; idx++)
    {
        NvFlowInteropBufferInstance* inst = &ptr->buffers[idx];
        if (inst->buffer)
        {
            if (ptr->contextInterface->isFeatureSupported(context, eNvFlowContextFeature_bufferExternalHandle))
            {
                ptr->contextInterface->closeBufferExternalHandle(context, inst->buffer, &inst->interopHandle);
            }
            ptr->contextInterface->destroyBuffer(context, inst->buffer);
            inst->buffer = nullptr;
            inst->bufferSizeInBytes = 0llu;
            NvFlowInteropHandle nullHandle = {};
            inst->interopHandle = nullHandle;
        }
    }
    ptr->buffers.size = 0u;
    for (NvFlowUint idx = 0u; idx < ptr->retainedBuffers.size; idx++)
    {
        NvFlowInteropBufferRetained* inst = &ptr->retainedBuffers[idx];
        if (inst->buffer)
        {
            if (ptr->contextInterface->isFeatureSupported(context, eNvFlowContextFeature_bufferExternalHandle))
            {
                ptr->contextInterface->closeBufferExternalHandle(context, inst->buffer, &inst->interopHandle);
            }
            ptr->contextInterface->destroyBuffer(context, inst->buffer);
            inst->buffer = nullptr;
            NvFlowInteropHandle nullHandle = {};
            inst->interopHandle = nullHandle;
        }
    }
    ptr->retainedBuffers.size = 0u;
}

NV_FLOW_INLINE void NvFlowInteropBuffer_setRingBufferCount(NvFlowInteropBuffer* ptr, NvFlowUint ringBufferCount)
{
    ptr->ringBufferCount = ringBufferCount;
}

struct NvFlowInteropBufferCopyRange
{
    NvFlowUint64 offset;
    NvFlowUint64 numBytes;
};

NV_FLOW_INLINE void NvFlowInteropBuffer_copyN(NvFlowContext* context, NvFlowInteropBuffer* ptr, NvFlowUint64 numBytes, NvFlowBufferTransient* src, const NvFlowInteropBufferCopyRange* copyRanges, NvFlowUint copyRangeCount, NvFlowUint64* pOutVersion)
{
    // find inactive buffer, create as needed
    NvFlowUint64 bufferIdx = 0u;
    for (; bufferIdx < ptr->buffers.size; bufferIdx++)
    {
        if (!ptr->buffers[bufferIdx].isActive)
        {
            break;
        }
    }
    if (bufferIdx == ptr->buffers.size)
    {
        bufferIdx = ptr->buffers.allocateBack();
    }
    NvFlowInteropBufferInstance* inst = &ptr->buffers[bufferIdx];

    // resize buffer as needed
    if (inst->buffer && inst->bufferSizeInBytes < numBytes)
    {
        if (ptr->ringBufferCount == 0u)
        {
            if (ptr->contextInterface->isFeatureSupported(context, eNvFlowContextFeature_bufferExternalHandle))
            {
                ptr->contextInterface->closeBufferExternalHandle(context, inst->buffer, &inst->interopHandle);
            }
            ptr->contextInterface->destroyBuffer(context, inst->buffer);
        }
        else  // defer release to minimize segfault risk
        {
            NvFlowInteropBufferRetained retained = {};
            retained.buffer = inst->buffer;
            retained.interopHandle = inst->interopHandle;
            ptr->retainedBuffers.pushBack(retained);
        }
        inst->buffer = nullptr;
        inst->bufferSizeInBytes = 0llu;
        NvFlowInteropHandle nullHandle = {};
        inst->interopHandle = nullHandle;
    }
    if (!inst->buffer)
    {
        NvFlowBufferDesc bufDesc = {};
        bufDesc.usageFlags = eNvFlowBufferUsage_bufferCopyDst;
        bufDesc.format = eNvFlowFormat_unknown;
        bufDesc.structureStride = 0u;
        bufDesc.sizeInBytes = 65536u;
        while (bufDesc.sizeInBytes < numBytes)
        {
            bufDesc.sizeInBytes *= 2u;
        }

        inst->buffer = ptr->contextInterface->createBuffer(context, eNvFlowMemoryType_device, &bufDesc);
        inst->bufferSizeInBytes = bufDesc.sizeInBytes;
        if (ptr->contextInterface->isFeatureSupported(context, eNvFlowContextFeature_bufferExternalHandle))
        {
            ptr->contextInterface->getBufferExternalHandle(context, inst->buffer, &inst->interopHandle);
        }
    }

    // set active state
    ptr->versionCounter++;
    inst->isActive = NV_FLOW_TRUE;
    inst->completedFrame = ptr->contextInterface->getCurrentFrame(context);
    inst->completedGlobalFrame = ptr->contextInterface->getCurrentGlobalFrame(context);
    inst->version = ptr->versionCounter;
    inst->validNumBytes = numBytes;
    if (pOutVersion)
    {
        *pOutVersion = inst->version;
    }

    // copy
    NvFlowBufferTransient* dst = ptr->contextInterface->registerBufferAsTransient(context, inst->buffer);
    for (NvFlowUint copyRangeIdx = 0u; copyRangeIdx < copyRangeCount; copyRangeIdx++)
    {
        NvFlowPassCopyBufferParams copyParams = {};
        copyParams.srcOffset = copyRanges[copyRangeIdx].offset;
        copyParams.dstOffset = copyRanges[copyRangeIdx].offset;
        copyParams.numBytes = copyRanges[copyRangeIdx].numBytes;
        copyParams.src = src;
        copyParams.dst = dst;

        copyParams.debugLabel = "InteropBufferCopy";

        ptr->contextInterface->addPassCopyBuffer(context, &copyParams);
    }
    if (copyRangeCount == 0u)
    {
        NvFlowPassCopyBufferParams copyParams = {};
        copyParams.srcOffset = 0llu;
        copyParams.dstOffset = 0llu;
        copyParams.numBytes = 0llu;
        copyParams.src = src;
        copyParams.dst = dst;

        copyParams.debugLabel = "InteropBufferCopy";

        ptr->contextInterface->addPassCopyBuffer(context, &copyParams);
    }

    // push on active queue
    ptr->activeBuffers.pushBack(bufferIdx);
}

NV_FLOW_INLINE void NvFlowInteropBuffer_copy(NvFlowContext* context, NvFlowInteropBuffer* ptr, NvFlowUint64 numBytes, NvFlowBufferTransient* src, NvFlowUint64* pOutVersion)
{
    NvFlowInteropBufferCopyRange copyRange = { 0llu, numBytes };
    NvFlowInteropBuffer_copyN(context, ptr, numBytes, src, &copyRange, 1u, pOutVersion);
}

NV_FLOW_INLINE void NvFlowInteropBuffer_flush(NvFlowContext* context, NvFlowInteropBuffer* ptr)
{
    // flush queue
    NvFlowUint completedCount = 0u;
    NvFlowUint64 lastFenceCompleted = ptr->contextInterface->getLastFrameCompleted(context);
    for (NvFlowUint activeBufferIdx = 0u; activeBufferIdx < ptr->activeBuffers.size; activeBufferIdx++)
    {
        if (ptr->buffers[ptr->activeBuffers[activeBufferIdx]].completedFrame > lastFenceCompleted)
        {
            break;
        }
        completedCount++;
    }
    NvFlowUint popCount = completedCount >= 2u ? completedCount - 1u : 0u;
    if (ptr->ringBufferCount != 0u)
    {
        NvFlowUint activeCount = (NvFlowUint)ptr->activeBuffers.size;
        NvFlowUint ringTargetCount = ptr->ringBufferCount - 1u;
        NvFlowUint ringPopCount = (activeCount > ringTargetCount) ? (activeCount - ringTargetCount) : 0u;
        if (ringPopCount < popCount)
        {
            popCount = ringPopCount;
        }
    }
    if (popCount > 0u)
    {
        for (NvFlowUint activeBufferIdx = 0u; activeBufferIdx < popCount; activeBufferIdx++)
        {
            ptr->buffers[ptr->activeBuffers[activeBufferIdx]].isActive = NV_FLOW_FALSE;
        }
        // compact
        for (NvFlowUint activeBufferIdx = popCount; activeBufferIdx < ptr->activeBuffers.size; activeBufferIdx++)
        {
            ptr->activeBuffers[activeBufferIdx - popCount] = ptr->activeBuffers[activeBufferIdx];
        }
        ptr->activeBuffers.size = ptr->activeBuffers.size - popCount;
    }
}

NV_FLOW_INLINE NvFlowUint NvFlowInteropBuffer_getActiveCount(NvFlowContext* context, NvFlowInteropBuffer* ptr)
{
    return (NvFlowUint)ptr->activeBuffers.size;
}

NV_FLOW_INLINE NvFlowUint64 NvFlowInteropBuffer_getCompletedGlobalFrame(NvFlowContext* context, NvFlowInteropBuffer* ptr, NvFlowUint activeIdx)
{
    if (activeIdx < ptr->activeBuffers.size)
    {
        return ptr->buffers[ptr->activeBuffers[activeIdx]].completedGlobalFrame;
    }
    return ~0llu;
}

NV_FLOW_INLINE NvFlowInteropHandle NvFlowInteropBuffer_map(NvFlowContext* context, NvFlowInteropBuffer* ptr, NvFlowUint activeIdx, NvFlowUint64* pOutVersion)
{
    NvFlowInteropHandle retHandle = {eNvFlowInteropHandleType_unknown, 0u, 0u};
    if (activeIdx > ptr->activeBuffers.size)
    {
        if (pOutVersion)
        {
            *pOutVersion = 0llu;
        }
        return retHandle;
    }

    NvFlowInteropBufferInstance* inst = &ptr->buffers[ptr->activeBuffers[activeIdx]];
    if (pOutVersion)
    {
        *pOutVersion = inst->version;
    }
    retHandle = inst->interopHandle;
    return retHandle;
}

NV_FLOW_INLINE NvFlowInteropHandle NvFlowInteropBuffer_mapLatest(NvFlowContext* context, NvFlowInteropBuffer* ptr, NvFlowUint64* pOutVersion)
{
    NvFlowInteropBuffer_flush(context, ptr);

    NvFlowUint64 lastFenceCompleted = ptr->contextInterface->getLastFrameCompleted(context);
    bool shouldMap = true;
    if (ptr->activeBuffers.size > 0u)
    {
        if (ptr->buffers[ptr->activeBuffers[0u]].completedFrame > lastFenceCompleted)
        {
            shouldMap = false;
        }
    }
    else if (ptr->buffers[ptr->activeBuffers[0u]].completedFrame > lastFenceCompleted)
    {
        shouldMap = false;
    }
    if (!shouldMap)
    {
        if (pOutVersion)
        {
            *pOutVersion = 0llu;
        }
        NvFlowInteropHandle nullHandle = {eNvFlowInteropHandleType_unknown, 0u, 0u};
        return nullHandle;
    }

    return NvFlowInteropBuffer_map(context, ptr, 0u, pOutVersion);
}
