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

#include "NvFlowUploadBuffer.h"

#include <string.h>

struct NvFlowArrayBufferData
{
	const void* data;
	NvFlowUint64 elementCount;
	NvFlowUint64 version;
};

struct NvFlowArrayBufferState
{
	NvFlowBool32 isDirty;

	NvFlowUint64 elementCount;
	NvFlowUint64 version;
	NvFlowUint64 firstElement;
};

struct NvFlowArrayBuffer
{
	NvFlowUploadBuffer uploadBuffer = {};

	NvFlowArray<NvFlowArrayBufferState> state;
	NvFlowArray<NvFlowUploadBufferCopyRange> copyRanges;
	NvFlowUint64 totalSizeInBytes = 0llu;
};

NV_FLOW_INLINE void NvFlowArrayBuffer_init_custom(
	NvFlowContextInterface* contextInterface, 
	NvFlowContext* context, 
	NvFlowArrayBuffer* ptr, 
	NvFlowBufferUsageFlags flags, 
	NvFlowFormat format, 
	NvFlowUint structureStride,
	NvFlowBuffer*(NV_FLOW_ABI* createBuffer)(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, void* userdata),
	void(NV_FLOW_ABI* addPassCopyBuffer)(NvFlowContext* context, const NvFlowPassCopyBufferParams* params, void* userdata),
	void* userdata
)
{
	NvFlowUploadBuffer_init_custom(contextInterface, context, &ptr->uploadBuffer, flags, format, structureStride, createBuffer, addPassCopyBuffer, userdata);
}

NV_FLOW_INLINE NvFlowBuffer* NvFlowArrayBuffer_createBuffer(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc, void* userdata)
{
	NvFlowArrayBuffer* ptr = (NvFlowArrayBuffer*)userdata;
	return ptr->uploadBuffer.contextInterface->createBuffer(context, memoryType, desc);
}

NV_FLOW_INLINE void NvFlowArrayBuffer_addPassCopyBuffer(NvFlowContext* context, const NvFlowPassCopyBufferParams* params, void* userdata)
{
	NvFlowArrayBuffer* ptr = (NvFlowArrayBuffer*)userdata;
	ptr->uploadBuffer.contextInterface->addPassCopyBuffer(context, params);
}

NV_FLOW_INLINE void NvFlowArrayBuffer_init(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowArrayBuffer* ptr, NvFlowBufferUsageFlags flags, NvFlowFormat format, NvFlowUint structureStride)
{
	NvFlowArrayBuffer_init_custom(contextInterface, context, ptr, flags, format, structureStride, NvFlowArrayBuffer_createBuffer, NvFlowArrayBuffer_addPassCopyBuffer, ptr);
}

NV_FLOW_INLINE void NvFlowArrayBuffer_destroy(NvFlowContext* context, NvFlowArrayBuffer* ptr)
{
	NvFlowUploadBuffer_destroy(context, &ptr->uploadBuffer);
	ptr->state.size = 0u;
	ptr->copyRanges.size = 0u;
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowArrayBuffer_update(
	NvFlowContext* context, 
	NvFlowArrayBuffer* ptr, 
	const NvFlowArrayBufferData* arrayDatas, 
	NvFlowUint64* outFirstElements, 
	NvFlowUint64 arrayCount, 
	NvFlowUint64* outTotalSizeInBytes,
	const char* debugName
)
{
	// if arrayCount changes, reset all state
	bool shouldResetState = false;
	if (ptr->state.size != arrayCount)
	{
		shouldResetState = true;
	}
	// if any array size changes, reset all state, since buffer resize might occur
	if (!shouldResetState)
	{
		for (NvFlowUint64 arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
		{
			if (ptr->state[arrayIdx].elementCount != arrayDatas[arrayIdx].elementCount)
			{
				shouldResetState = true;
			}
		}
	}
	if (shouldResetState)
	{
		ptr->state.reserve(arrayCount);
		ptr->state.size = arrayCount;
		for (NvFlowUint64 arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
		{
			ptr->state[arrayIdx].isDirty = NV_FLOW_TRUE;
			ptr->state[arrayIdx].elementCount = 0llu;
			ptr->state[arrayIdx].version = 0llu;
			ptr->state[arrayIdx].firstElement = 0llu;
		}
	}

	// mark any array dirty if version changes
	for (NvFlowUint64 arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
	{
		if (arrayDatas[arrayIdx].elementCount != 0u || ptr->state[arrayIdx].elementCount != 0u)
		{
			if (arrayDatas[arrayIdx].version == 0llu || arrayDatas[arrayIdx].version != ptr->state[arrayIdx].version)
			{
				ptr->state[arrayIdx].isDirty = NV_FLOW_TRUE;
			}
		}
	}

	NvFlowBool32 anyDirty = NV_FLOW_FALSE;
	for (NvFlowUint64 arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
	{
		if (ptr->state[arrayIdx].isDirty)
		{
			anyDirty = NV_FLOW_TRUE;
		}
	}

	// compute total size
	NvFlowUint64 totalSizeInBytes = 0llu;
	for (NvFlowUint arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
	{
		totalSizeInBytes += ptr->uploadBuffer.structureStride * arrayDatas[arrayIdx].elementCount;
	}
	NvFlowUint8* mapped = nullptr;
	if (anyDirty)
	{
		mapped = (NvFlowUint8*)NvFlowUploadBuffer_map(context, &ptr->uploadBuffer, totalSizeInBytes);
	}

	// update state
	NvFlowUint64 globalFirstElement = 0llu;
	for (NvFlowUint arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
	{
		ptr->state[arrayIdx].elementCount = arrayDatas[arrayIdx].elementCount;
		ptr->state[arrayIdx].version = arrayDatas[arrayIdx].version;
		ptr->state[arrayIdx].firstElement = globalFirstElement;

		globalFirstElement += ptr->state[arrayIdx].elementCount;
	}

	ptr->copyRanges.size = 0u;
	for (NvFlowUint arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
	{
		if (ptr->state[arrayIdx].isDirty)
		{
			NvFlowUint64 offsetInBytes = ptr->uploadBuffer.structureStride * ptr->state[arrayIdx].firstElement;
			NvFlowUint64 sizeInBytes = ptr->uploadBuffer.structureStride * ptr->state[arrayIdx].elementCount;

			// copy to host memory
			memcpy(mapped + offsetInBytes, arrayDatas[arrayIdx].data, sizeInBytes);

			// add copy range
			NvFlowUploadBufferCopyRange copyRange = { offsetInBytes, sizeInBytes };
			ptr->copyRanges.pushBack(copyRange);
		}

	}
	NvFlowBufferTransient* bufferTransient = nullptr;
	if (anyDirty)
	{
		bufferTransient = NvFlowUploadBuffer_unmapDeviceN(context, &ptr->uploadBuffer, ptr->copyRanges.data, ptr->copyRanges.size, debugName);
	}
	else
	{
		bufferTransient = NvFlowUploadBuffer_getDevice(context, &ptr->uploadBuffer, totalSizeInBytes);
	}
	// mark all arrays as clean
	for (NvFlowUint arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
	{
		ptr->state[arrayIdx].isDirty = NV_FLOW_FALSE;
	}

	if (outFirstElements)
	{
		for (NvFlowUint arrayIdx = 0u; arrayIdx < arrayCount; arrayIdx++)
		{
			outFirstElements[arrayIdx] = ptr->state[arrayIdx].firstElement;
		}
	}

	ptr->totalSizeInBytes = totalSizeInBytes;
	if (outTotalSizeInBytes)
	{
		*outTotalSizeInBytes = totalSizeInBytes;
	}

	return bufferTransient;
}
