/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
