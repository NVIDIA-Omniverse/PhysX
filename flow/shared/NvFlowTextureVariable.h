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

struct NvFlowTextureVariable
{
	NvFlowContextInterface* contextInterface = nullptr;
	NvFlowTextureTransient* transientTexture = nullptr;
	NvFlowUint64 transientFrame = ~0llu;
	NvFlowFormat transientFormat = eNvFlowFormat_unknown;
	NvFlowTexture* texture = nullptr;
	NvFlowArray<NvFlowTextureAcquire*, 4u> acquires;
	NvFlowTextureDesc hintTexDesc = {};
};

NV_FLOW_INLINE void NvFlowTextureVariable_init(NvFlowContextInterface* contextInterface, NvFlowTextureVariable* ptr)
{
	ptr->contextInterface = contextInterface;
}

NV_FLOW_INLINE void NvFlowTextureVariable_flush(NvFlowContext* context, NvFlowTextureVariable* ptr)
{
	// process acquire queue
	NvFlowUint acquireWriteIdx = 0u;
	for (NvFlowUint acquireReadIdx = 0u; acquireReadIdx < ptr->acquires.size; acquireReadIdx++)
	{
		NvFlowTexture* acquiredTexture = nullptr;
		if (ptr->contextInterface->getAcquiredTexture(context, ptr->acquires[acquireReadIdx], &acquiredTexture))
		{
			if (ptr->texture)
			{
				ptr->contextInterface->destroyTexture(context, ptr->texture);
				ptr->texture = nullptr;
			}
			ptr->texture = acquiredTexture;
		}
		else
		{
			ptr->acquires[acquireWriteIdx++] = ptr->acquires[acquireReadIdx];
		}
	}
	ptr->acquires.size = acquireWriteIdx;
}

NV_FLOW_INLINE NvFlowTextureTransient* NvFlowTextureVariable_get(NvFlowContext* context, NvFlowTextureVariable* ptr, NvFlowFormat* pFormat)
{
	if (ptr->transientFrame == ptr->contextInterface->getCurrentFrame(context))
	{
		if (pFormat)
		{
			*pFormat = ptr->transientFormat;
		}
		return ptr->transientTexture;
	}

	NvFlowTextureVariable_flush(context, ptr);

	if (ptr->texture)
	{
		ptr->transientTexture = ptr->contextInterface->registerTextureAsTransient(context, ptr->texture);
		ptr->transientFrame = ptr->contextInterface->getCurrentFrame(context);
	}
	else
	{
		ptr->transientTexture = nullptr;
		ptr->transientFrame = ~0llu;
		ptr->transientFormat = eNvFlowFormat_unknown;
	}
	if (pFormat)
	{
		*pFormat = ptr->transientFormat;
	}
	return ptr->transientTexture;
}

NV_FLOW_INLINE void NvFlowTextureVariable_set(NvFlowContext* context, NvFlowTextureVariable* ptr, NvFlowTextureTransient* transientTexture, NvFlowFormat transientFormat)
{
	NvFlowTextureVariable_flush(context, ptr);
	if (ptr->texture)
	{
		ptr->contextInterface->destroyTexture(context, ptr->texture);
		ptr->texture = nullptr;
	}
	ptr->transientTexture = nullptr;
	ptr->transientFrame = ~0llu;
	ptr->transientFormat = eNvFlowFormat_unknown;
	if (transientTexture)
	{
		ptr->transientTexture = transientTexture;
		ptr->transientFrame = ptr->contextInterface->getCurrentFrame(context);
		ptr->transientFormat = transientFormat;
		// push acquire
		ptr->acquires.pushBack(ptr->contextInterface->enqueueAcquireTexture(context, transientTexture));
	}
}

NV_FLOW_INLINE void NvFlowTextureVariable_destroy(NvFlowContext* context, NvFlowTextureVariable* ptr)
{
	NvFlowTextureVariable_set(context, ptr, nullptr, eNvFlowFormat_unknown);
}