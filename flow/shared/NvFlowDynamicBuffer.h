/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
	}
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowDynamicBuffer_getTransient(NvFlowContext* context, NvFlowDynamicBuffer* ptr)
{
	if (ptr->transientFrame == ptr->contextInterface->getCurrentFrame(context))
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