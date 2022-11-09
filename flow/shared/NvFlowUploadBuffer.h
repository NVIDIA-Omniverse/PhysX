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

#define NV_FLOW_DISPATCH_BATCH_SIZE 32768u
//#define NV_FLOW_DISPATCH_BATCH_SIZE 256

struct NvFlowDispatchBatch
{
	NvFlowBufferTransient* globalTransient = nullptr;
	NvFlowUint blockIdxOffset = 0u;
	NvFlowUint blockCount = 0u;
};

typedef NvFlowArray<NvFlowDispatchBatch, 8u> NvFlowDispatchBatches;

NV_FLOW_INLINE void NvFlowDispatchBatches_init_custom(NvFlowDispatchBatches* ptr, NvFlowUint totalBlockCount, NvFlowUint batchSize)
{
	ptr->size = 0u;
	for (NvFlowUint blockIdxOffset = 0u; blockIdxOffset < totalBlockCount; blockIdxOffset += batchSize)
	{
		NvFlowDispatchBatch batch = {};
		batch.globalTransient = nullptr;
		batch.blockIdxOffset = blockIdxOffset;
		batch.blockCount = totalBlockCount - blockIdxOffset;
		if (batch.blockCount > batchSize)
		{
			batch.blockCount = batchSize;
		}
		ptr->pushBack(batch);
	}
}

NV_FLOW_INLINE void NvFlowDispatchBatches_init(NvFlowDispatchBatches* ptr, NvFlowUint totalBlockCount)
{
	NvFlowDispatchBatches_init_custom(ptr, totalBlockCount, NV_FLOW_DISPATCH_BATCH_SIZE);
}

struct NvFlowBufferVersioning
{
	NvFlowUint64 mappedIdx = ~0llu;
	NvFlowUint64 frontIdx = ~0llu;
	NvFlowArray<NvFlowUint64, 16u> recycleFenceValues;
};

NV_FLOW_INLINE NvFlowUint64 NvFlowBufferVersioning_map(NvFlowBufferVersioning* ptr, NvFlowUint64 lastFenceCompleted)
{
	NvFlowUint64 index = ptr->frontIdx + 1u;
	for (; index < ptr->recycleFenceValues.size; index++)
	{
		if (ptr->recycleFenceValues[index] <= lastFenceCompleted)
		{
			break;
		}
	}
	if (index == ptr->recycleFenceValues.size)
	{
		for (index = 0; index < ptr->frontIdx; index++)
		{
			if (ptr->recycleFenceValues[index] <= lastFenceCompleted)
			{
				break;
			}
		}
	}
	if (!(index < ptr->recycleFenceValues.size && ptr->recycleFenceValues[index] <= lastFenceCompleted))
	{
		index = ptr->recycleFenceValues.allocateBack();
	}
	ptr->recycleFenceValues[index] = ~0llu;
	ptr->mappedIdx = index;
	return ptr->mappedIdx;
}

NV_FLOW_INLINE void NvFlowBufferVersioning_unmap(NvFlowBufferVersioning* ptr, NvFlowUint64 nextFenceValue)
{
	if (ptr->frontIdx < ptr->recycleFenceValues.size)
	{
		ptr->recycleFenceValues[ptr->frontIdx] = nextFenceValue;
	}
	ptr->frontIdx = ptr->mappedIdx;
}

struct NvFlowUploadBuffer
{
	NvFlowContextInterface* contextInterface = nullptr;
	NvFlowBuffer*(NV_FLOW_ABI* createBuffer)(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, void* userdata) = nullptr;
	void(NV_FLOW_ABI* addPassCopyBuffer)(NvFlowContext* context, const NvFlowPassCopyBufferParams* params, void* userdata) = nullptr;
	void* userdata = nullptr;
	NvFlowBufferUsageFlags flags = 0u;
	NvFlowFormat format = eNvFlowFormat_unknown;
	NvFlowUint structureStride = 0u;

	NvFlowBufferVersioning versioning;
	NvFlowArray<NvFlowBuffer*, 8u> buffers;
	NvFlowArray<NvFlowUint64, 8u> bufferSizes;

	NvFlowBuffer* deviceBuffer = nullptr;
	NvFlowUint64 deviceNumBytes = 0llu;
};

NV_FLOW_INLINE void NvFlowUploadBuffer_init_custom(
	NvFlowContextInterface* contextInterface, 
	NvFlowContext* context, NvFlowUploadBuffer* ptr, 
	NvFlowBufferUsageFlags flags, NvFlowFormat format, NvFlowUint structureStride,
	NvFlowBuffer*(NV_FLOW_ABI* createBuffer)(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, void* userdata),
	void(NV_FLOW_ABI* addPassCopyBuffer)(NvFlowContext* context, const NvFlowPassCopyBufferParams* params, void* userdata),
	void* userdata
)
{
	ptr->contextInterface = contextInterface;
	ptr->createBuffer = createBuffer;
	ptr->addPassCopyBuffer = addPassCopyBuffer;
	ptr->userdata = userdata;

	ptr->flags = flags;
	ptr->format = format;
	ptr->structureStride = structureStride;
}

NV_FLOW_INLINE NvFlowBuffer* NvFlowUploadBuffer_createBuffer(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, void* userdata)
{
	NvFlowUploadBuffer* ptr = (NvFlowUploadBuffer*)userdata;
	return ptr->contextInterface->createBuffer(context, memoryType, desc);
}

NV_FLOW_INLINE void NvFlowUploadBuffer_addPassCopyBuffer(NvFlowContext* context, const NvFlowPassCopyBufferParams* params, void* userdata)
{
	NvFlowUploadBuffer* ptr = (NvFlowUploadBuffer*)userdata;
	ptr->contextInterface->addPassCopyBuffer(context, params);
}

NV_FLOW_INLINE void NvFlowUploadBuffer_init(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowUploadBuffer* ptr, NvFlowBufferUsageFlags flags, NvFlowFormat format, NvFlowUint structureStride)
{
	NvFlowUploadBuffer_init_custom(contextInterface, context, ptr, flags, format, structureStride, NvFlowUploadBuffer_createBuffer, NvFlowUploadBuffer_addPassCopyBuffer, ptr);
}

NV_FLOW_INLINE void NvFlowUploadBuffer_destroy(NvFlowContext* context, NvFlowUploadBuffer* ptr)
{
	for (NvFlowUint64 idx = 0u; idx < ptr->buffers.size; idx++)
	{
		if (ptr->buffers[idx])
		{
			ptr->contextInterface->destroyBuffer(context, ptr->buffers[idx]);
			ptr->buffers[idx] = nullptr;
		}
	}
	ptr->buffers.size = 0u;
	ptr->bufferSizes.size = 0u;

	if (ptr->deviceBuffer)
	{
		ptr->contextInterface->destroyBuffer(context, ptr->deviceBuffer);
		ptr->deviceBuffer = nullptr;
	}
}

NV_FLOW_INLINE NvFlowUint64 NvFlowUploadBuffer_computeBufferSize(NvFlowUint64 requested)
{
	NvFlowUint64 bufferSize = 65536u;
	while (bufferSize < requested)
	{
		bufferSize *= 2u;
	}
	return bufferSize;
}

NV_FLOW_INLINE void* NvFlowUploadBuffer_map(NvFlowContext* context, NvFlowUploadBuffer* ptr, NvFlowUint64 numBytes)
{
	NvFlowUint64 instanceIdx = NvFlowBufferVersioning_map(&ptr->versioning, ptr->contextInterface->getLastFrameCompleted(context));
	while (instanceIdx >= ptr->buffers.size)
	{
		ptr->buffers.pushBack(nullptr);
		ptr->bufferSizes.pushBack(0llu);
	}

	if (ptr->buffers[instanceIdx] && ptr->bufferSizes[instanceIdx] < numBytes)
	{
		ptr->contextInterface->destroyBuffer(context, ptr->buffers[instanceIdx]);
		ptr->buffers[instanceIdx] = nullptr;
	}

	if (!ptr->buffers[instanceIdx])
	{
		NvFlowBufferDesc bufDesc = {};
		bufDesc.format = ptr->format;
		bufDesc.usageFlags = ptr->flags;
		bufDesc.structureStride = ptr->structureStride;
		bufDesc.sizeInBytes = NvFlowUploadBuffer_computeBufferSize(numBytes);

		ptr->bufferSizes[instanceIdx] = bufDesc.sizeInBytes;
		ptr->buffers[instanceIdx] = ptr->contextInterface->createBuffer(context, eNvFlowMemoryType_upload, &bufDesc);
	}

	return ptr->contextInterface->mapBuffer(context, ptr->buffers[instanceIdx]);
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowUploadBuffer_unmap(NvFlowContext* context, NvFlowUploadBuffer* ptr)
{
	ptr->contextInterface->unmapBuffer(context, ptr->buffers[ptr->versioning.mappedIdx]);

	NvFlowBufferVersioning_unmap(&ptr->versioning, ptr->contextInterface->getCurrentFrame(context));

	return ptr->contextInterface->registerBufferAsTransient(context, ptr->buffers[ptr->versioning.frontIdx]);
}

struct NvFlowUploadBufferCopyRange
{
	NvFlowUint64 offset;
	NvFlowUint64 numBytes;
};

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowUploadBuffer_getDevice(NvFlowContext* context, NvFlowUploadBuffer* ptr, NvFlowUint64 numBytes)
{
	NvFlowUint64 srcNumBytes = NvFlowUploadBuffer_computeBufferSize(numBytes);

	if (ptr->deviceBuffer && ptr->deviceNumBytes < srcNumBytes)
	{
		ptr->contextInterface->destroyBuffer(context, ptr->deviceBuffer);
		ptr->deviceBuffer = nullptr;
		ptr->deviceNumBytes = 0llu;
	}
	if (!ptr->deviceBuffer)
	{
		NvFlowBufferDesc bufDesc = {};
		bufDesc.format = ptr->format;
		bufDesc.usageFlags = ptr->flags | eNvFlowBufferUsage_bufferCopyDst;
		bufDesc.structureStride = ptr->structureStride;
		bufDesc.sizeInBytes = srcNumBytes;

		ptr->deviceBuffer = ptr->createBuffer(context, eNvFlowMemoryType_device, &bufDesc, ptr->userdata);
		ptr->deviceNumBytes = srcNumBytes;
	}

	return ptr->contextInterface->registerBufferAsTransient(context, ptr->deviceBuffer);
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowUploadBuffer_unmapDeviceN(NvFlowContext* context, NvFlowUploadBuffer* ptr, NvFlowUploadBufferCopyRange* copyRanges, NvFlowUint64 copyRangeCount, const char* debugName)
{
	NvFlowBufferTransient* src = NvFlowUploadBuffer_unmap(context, ptr);

	NvFlowUint64 srcNumBytes = ptr->bufferSizes[ptr->versioning.frontIdx];

	NvFlowBufferTransient* dst = NvFlowUploadBuffer_getDevice(context, ptr, srcNumBytes);

	NvFlowUint activeCopyCount = 0u;
	for (NvFlowUint64 copyRangeIdx = 0u; copyRangeIdx < copyRangeCount; copyRangeIdx++)
	{
		NvFlowPassCopyBufferParams copyParams = {};
		copyParams.srcOffset = copyRanges[copyRangeIdx].offset;
		copyParams.dstOffset = copyRanges[copyRangeIdx].offset;
		copyParams.numBytes = copyRanges[copyRangeIdx].numBytes;
		copyParams.src = src;
		copyParams.dst = dst;

		copyParams.debugLabel = debugName ? debugName : "UploadBufferUnmapDevice";

		if (copyParams.numBytes > 0u)
		{
			ptr->addPassCopyBuffer(context, &copyParams, ptr->userdata);
			activeCopyCount++;
		}
	}
	// this ensures proper barriers
	if (activeCopyCount == 0u)
	{
		NvFlowPassCopyBufferParams copyParams = {};
		copyParams.srcOffset = 0llu;
		copyParams.dstOffset = 0llu;
		copyParams.numBytes = 0llu;
		copyParams.src = src;
		copyParams.dst = dst;

		copyParams.debugLabel = debugName ? debugName : "UploadBufferUnmapDevice";

		ptr->addPassCopyBuffer(context, &copyParams, ptr->userdata);
	}

	return dst;
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowUploadBuffer_unmapDevice(NvFlowContext* context, NvFlowUploadBuffer* ptr, NvFlowUint64 offset, NvFlowUint64 numBytes, const char* debugName)
{
	NvFlowUploadBufferCopyRange copyRange = { offset, numBytes };
	return NvFlowUploadBuffer_unmapDeviceN(context, ptr, &copyRange, 1u, debugName);
}