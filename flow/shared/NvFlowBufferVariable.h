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