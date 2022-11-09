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
//
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

#pragma once

#include "NvFlowContext.h"
#include "NvFlowArray.h"

struct NvFlowReadbackBufferInstance
{
	NvFlowBuffer* buffer = nullptr;
	NvFlowUint64 bufferSizeInBytes = 0llu;
	NvFlowBool32 isActive = NV_FLOW_FALSE;
	NvFlowUint64 completedFrame = ~0llu;
	NvFlowUint64 completedGlobalFrame = ~0llu;
	NvFlowUint64 version = ~0llu;
	NvFlowUint64 validNumBytes = 0llu;
};

struct NvFlowReadbackBuffer
{
	NvFlowContextInterface* contextInterface = nullptr;
	NvFlowUint64 versionCounter = 0llu;

	NvFlowArray<NvFlowReadbackBufferInstance, 8u> buffers;
	NvFlowArray<NvFlowUint64, 8u> activeBuffers;
};

NV_FLOW_INLINE void NvFlowReadbackBuffer_init(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowReadbackBuffer* ptr)
{
	ptr->contextInterface = contextInterface;
}

NV_FLOW_INLINE void NvFlowReadbackBuffer_destroy(NvFlowContext* context, NvFlowReadbackBuffer* ptr)
{
	for (NvFlowUint idx = 0u; idx < ptr->buffers.size; idx++)
	{
		if (ptr->buffers[idx].buffer)
		{
			ptr->contextInterface->destroyBuffer(context, ptr->buffers[idx].buffer);
			ptr->buffers[idx].buffer = nullptr;
		}
	}
	ptr->buffers.size = 0u;
}

struct NvFlowReadbackBufferCopyRange
{
	NvFlowUint64 offset;
	NvFlowUint64 numBytes;
};

NV_FLOW_INLINE void NvFlowReadbackBuffer_copyN(NvFlowContext* context, NvFlowReadbackBuffer* ptr, NvFlowUint64 numBytes, NvFlowBufferTransient* src, const NvFlowReadbackBufferCopyRange* copyRanges, NvFlowUint copyRangeCount, NvFlowUint64* pOutVersion)
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
	NvFlowReadbackBufferInstance* inst = &ptr->buffers[bufferIdx];
	
	// resize buffer as needed
	if (inst->buffer && inst->bufferSizeInBytes < numBytes)
	{
		ptr->contextInterface->destroyBuffer(context, inst->buffer);
		inst->buffer = nullptr;
		inst->bufferSizeInBytes = 0llu;
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

		inst->buffer = ptr->contextInterface->createBuffer(context, eNvFlowMemoryType_readback, &bufDesc);
		inst->bufferSizeInBytes = bufDesc.sizeInBytes;
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

		copyParams.debugLabel = "ReadbackBufferCopy";

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

		copyParams.debugLabel = "ReadbackBufferCopy";

		ptr->contextInterface->addPassCopyBuffer(context, &copyParams);
	}

	// push on active queue
	ptr->activeBuffers.pushBack(bufferIdx);
}

NV_FLOW_INLINE void NvFlowReadbackBuffer_copy(NvFlowContext* context, NvFlowReadbackBuffer* ptr, NvFlowUint64 numBytes, NvFlowBufferTransient* src, NvFlowUint64* pOutVersion)
{
	NvFlowReadbackBufferCopyRange copyRange = { 0llu, numBytes };
	NvFlowReadbackBuffer_copyN(context, ptr, numBytes, src, &copyRange, 1u, pOutVersion);
}

NV_FLOW_INLINE void NvFlowReadbackBuffer_flush(NvFlowContext* context, NvFlowReadbackBuffer* ptr)
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

NV_FLOW_INLINE NvFlowUint NvFlowReadbackBuffer_getActiveCount(NvFlowContext* context, NvFlowReadbackBuffer* ptr)
{
	return (NvFlowUint)ptr->activeBuffers.size;
}

NV_FLOW_INLINE NvFlowUint64 NvFlowReadbackBuffer_getCompletedGlobalFrame(NvFlowContext* context, NvFlowReadbackBuffer* ptr, NvFlowUint activeIdx)
{
	if (activeIdx < ptr->activeBuffers.size)
	{
		return ptr->buffers[ptr->activeBuffers[activeIdx]].completedGlobalFrame;
	}
	return ~0llu;
}

NV_FLOW_INLINE void* NvFlowReadbackBuffer_map(NvFlowContext* context, NvFlowReadbackBuffer* ptr, NvFlowUint activeIdx, NvFlowUint64* pOutVersion, NvFlowUint64* pNumBytes)
{
	if (activeIdx > ptr->activeBuffers.size)
	{
		if (pOutVersion)
		{
			*pOutVersion = 0llu;
		}
		if (pNumBytes)
		{
			*pNumBytes = 0llu;
		}
		return nullptr;
	}

	NvFlowReadbackBufferInstance* inst = &ptr->buffers[ptr->activeBuffers[activeIdx]];
	if (pOutVersion)
	{
		*pOutVersion = inst->version;
	}
	if (pNumBytes)
	{
		*pNumBytes = inst->validNumBytes;
	}
	return ptr->contextInterface->mapBuffer(context, inst->buffer);
}

NV_FLOW_INLINE void* NvFlowReadbackBuffer_mapLatest(NvFlowContext* context, NvFlowReadbackBuffer* ptr, NvFlowUint64* pOutVersion, NvFlowUint64* pNumBytes)
{
	NvFlowReadbackBuffer_flush(context, ptr);

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
		if (pNumBytes)
		{
			*pNumBytes = 0llu;
		}
		return nullptr;
	}

	return NvFlowReadbackBuffer_map(context, ptr, 0u, pOutVersion, pNumBytes);
}

NV_FLOW_INLINE void NvFlowReadbackBuffer_unmap(NvFlowContext* context, NvFlowReadbackBuffer* ptr, NvFlowUint activeIdx)
{
	if (activeIdx < ptr->activeBuffers.size)
	{
		NvFlowReadbackBufferInstance* inst = &ptr->buffers[ptr->activeBuffers[activeIdx]];
		ptr->contextInterface->unmapBuffer(context, inst->buffer);
	}
}

NV_FLOW_INLINE void NvFlowReadbackBuffer_unmapLatest(NvFlowContext* context, NvFlowReadbackBuffer* ptr)
{
	NvFlowReadbackBuffer_unmap(context, ptr, 0u);
}