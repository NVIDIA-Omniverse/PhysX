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

struct NvFlowBufferVariable
{
	NvFlowContextInterface* contextInterface = nullptr;
	NvFlowBufferTransient* transientBuffer = nullptr;
	NvFlowUint64 transientFrame = ~0llu;
	NvFlowBuffer* buffer = nullptr;
	NvFlowArray<NvFlowBufferAcquire*, 4u> acquires;
};

NV_FLOW_INLINE void NvFlowBufferVariable_init(NvFlowContextInterface* contextInterface, NvFlowBufferVariable* ptr)
{
	ptr->contextInterface = contextInterface;
}

NV_FLOW_INLINE void NvFlowBufferVariable_flush(NvFlowContext* context, NvFlowBufferVariable* ptr)
{
	// process acquire queue
	NvFlowUint acquireWriteIdx = 0u;
	for (NvFlowUint acquireReadIdx = 0u; acquireReadIdx < ptr->acquires.size; acquireReadIdx++)
	{
		NvFlowBuffer* acquiredBuffer = nullptr;
		if (ptr->contextInterface->getAcquiredBuffer(context, ptr->acquires[acquireReadIdx], &acquiredBuffer))
		{
			if (ptr->buffer)
			{
				ptr->contextInterface->destroyBuffer(context, ptr->buffer);
				ptr->buffer = nullptr;
			}
			ptr->buffer = acquiredBuffer;
		}
		else
		{
			ptr->acquires[acquireWriteIdx++] = ptr->acquires[acquireReadIdx];
		}
	}
	ptr->acquires.size = acquireWriteIdx;
}

NV_FLOW_INLINE NvFlowBufferTransient* NvFlowBufferVariable_get(NvFlowContext* context, NvFlowBufferVariable* ptr)
{
	if (ptr->transientFrame == ptr->contextInterface->getCurrentFrame(context))
	{
		return ptr->transientBuffer;
	}

	NvFlowBufferVariable_flush(context, ptr);

	if (ptr->buffer)
	{
		ptr->transientBuffer = ptr->contextInterface->registerBufferAsTransient(context, ptr->buffer);
		ptr->transientFrame = ptr->contextInterface->getCurrentFrame(context);
	}
	else
	{
		ptr->transientBuffer = nullptr;
		ptr->transientFrame = ~0llu;
	}
	return ptr->transientBuffer;
}

NV_FLOW_INLINE void NvFlowBufferVariable_set(NvFlowContext* context, NvFlowBufferVariable* ptr, NvFlowBufferTransient* transientBuffer)
{
	NvFlowBufferVariable_flush(context, ptr);
	if (ptr->buffer)
	{
		ptr->contextInterface->destroyBuffer(context, ptr->buffer);
		ptr->buffer = nullptr;
	}
	ptr->transientBuffer = nullptr;
	ptr->transientFrame = ~0llu;
	if (transientBuffer)
	{
		ptr->transientBuffer = transientBuffer;
		ptr->transientFrame = ptr->contextInterface->getCurrentFrame(context);
		// push acquire
		ptr->acquires.pushBack(ptr->contextInterface->enqueueAcquireBuffer(context, transientBuffer));
	}
}

NV_FLOW_INLINE void NvFlowBufferVariable_destroy(NvFlowContext* context, NvFlowBufferVariable* ptr)
{
	NvFlowBufferVariable_set(context, ptr, nullptr);
}