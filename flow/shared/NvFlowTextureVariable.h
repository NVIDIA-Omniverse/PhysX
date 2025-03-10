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
