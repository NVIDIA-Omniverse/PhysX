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

#include "NvFlowReflect.h"
#include "NvFlowArray.h"

#include <string.h>

struct NvFlowReflectDeepCopyInfo
{
    const char* debugname;
    NvFlowUint64 size;
};

struct NvFlowReflectDeepCopyCached;

struct NvFlowReflectDeepCopy
{
    NvFlowArray<NvFlowArray<NvFlowUint8>, 16u> heaps;
    NvFlowArray<NvFlowReflectDeepCopyInfo> infos;
    NvFlowArray<const char*> pathStack;
    NvFlowArrayPointer<NvFlowReflectDeepCopyCached*> cached;
};

struct NvFlowReflectDeepCopyCached
{
    NvFlowUint64 luid = 0llu;
    NvFlowArray<const char*> pathStack;
    NvFlowUint64 version = 0llu;
    NvFlowReflectDeepCopy* deepCopy = nullptr;
    NvFlowUint8* deepCopyData;
    NvFlowUint64 lastUse = 0llu;
};

NV_FLOW_INLINE NvFlowReflectDeepCopy* NvFlowReflectDeepCopy_create()
{
    auto ptr = new NvFlowReflectDeepCopy();
    return ptr;
}

NV_FLOW_INLINE void NvFlowReflectDeepCopyCached_destroy(NvFlowReflectDeepCopyCached* ptr);

NV_FLOW_INLINE void NvFlowReflectDeepCopy_destroy(NvFlowReflectDeepCopy* ptr)
{
    for (NvFlowUint64 cachedIdx = 0u; cachedIdx < ptr->cached.size; cachedIdx++)
    {
        NvFlowReflectDeepCopyCached_destroy(ptr->cached[cachedIdx]);
        ptr->cached[cachedIdx] = nullptr;
    }
    ptr->cached.size = 0u;
    delete ptr;
}

NV_FLOW_INLINE NvFlowReflectDeepCopyCached* NvFlowReflectDeepCopyCached_create(NvFlowUint64 luid, const char** pathStacks, NvFlowUint64 pathStackCount)
{
    auto ptr = new NvFlowReflectDeepCopyCached();
    ptr->luid = luid;
    ptr->pathStack.size = 0u;
    for (NvFlowUint64 pathStackIdx = 0u; pathStackIdx < pathStackCount; pathStackIdx++)
    {
        ptr->pathStack.pushBack(pathStacks[pathStackIdx]);
    }
    ptr->version = 0llu;
    ptr->deepCopy = NvFlowReflectDeepCopy_create();
    return ptr;
}

NV_FLOW_INLINE void NvFlowReflectDeepCopyCached_destroy(NvFlowReflectDeepCopyCached* ptr)
{
    NvFlowReflectDeepCopy_destroy(ptr->deepCopy);
    ptr->deepCopy = nullptr;
    delete ptr;
}

NV_FLOW_INLINE void NvFlowReflectDeepCopy_newHeap(NvFlowReflectDeepCopy* ptr, NvFlowUint64 allocSize)
{
    auto& currentHeap = ptr->heaps[ptr->heaps.allocateBack()];

    NvFlowUint64 heapSize = 4096u;    // default heap size
    while (heapSize < allocSize)
    {
        heapSize *= 2u;
    }

    currentHeap.reserve(heapSize);
}

NV_FLOW_INLINE NvFlowUint64 NvFlowReflectDeepCopy_alignment(NvFlowUint64 size)
{
    return 8u * ((size + 7u) / 8u);
}

NV_FLOW_INLINE NvFlowUint8* NvFlowReflectDeepCopy_allocate(NvFlowReflectDeepCopy* ptr, NvFlowUint64 size, const char* debugName)
{
    NvFlowUint64 allocSize = NvFlowReflectDeepCopy_alignment(size);

    if (ptr->heaps.size > 0u)
    {
        auto& currentHeap = ptr->heaps[ptr->heaps.size - 1u];

        if (currentHeap.size + allocSize <= currentHeap.capacity)
        {
            NvFlowUint8* ret = currentHeap.data + currentHeap.size;
            ret[size - 1] = 0;
            currentHeap.size += allocSize;
            NvFlowReflectDeepCopyInfo info = { debugName, size };
            ptr->infos.pushBack(info);
            return ret;
        }
    }

    NvFlowReflectDeepCopy_newHeap(ptr, allocSize);

    return NvFlowReflectDeepCopy_allocate(ptr, size, debugName);
}

NV_FLOW_INLINE void NvFlowReflectDeepCopy_reset(NvFlowReflectDeepCopy* ptr)
{
    for (NvFlowUint64 heapIdx = 0u; heapIdx < ptr->heaps.size; heapIdx++)
    {
        ptr->heaps[heapIdx].size = 0u;
    }
    ptr->heaps.size = 0u;
    ptr->infos.size = 0u;
    ptr->pathStack.size = 0u;
}

NV_FLOW_INLINE NvFlowUint8* NvFlowReflectDeepCopy_recursive(NvFlowReflectDeepCopy* ptr, NvFlowUint64 luid, const NvFlowUint8* src, const NvFlowReflectDataType* type, NvFlowUint64 elementCount, NvFlowBool32 isPointerArray);

NV_FLOW_INLINE void NvFlowReflectDeepCopy_cleanCache(NvFlowReflectDeepCopy* ptr)
{
    static const NvFlowUint64 cacheFreeTreshold = 8u;

    NvFlowUint cachedIdx = 0u;
    while (cachedIdx < ptr->cached.size)
    {
        ptr->cached[cachedIdx]->lastUse++;
        if (ptr->cached[cachedIdx]->lastUse > cacheFreeTreshold)
        {
            NvFlowReflectDeepCopyCached_destroy(ptr->cached[cachedIdx]);
            ptr->cached[cachedIdx] = nullptr;
            ptr->cached.removeSwapPointerAtIndex(cachedIdx);
        }
        else
        {
            cachedIdx++;
        }
    }
}

NV_FLOW_INLINE NvFlowUint8* NvFlowReflectDeepCopy_cached(NvFlowReflectDeepCopy* ptr, NvFlowUint64 luid, const NvFlowUint8* src, const NvFlowReflectDataType* type, NvFlowUint64 elementCount, NvFlowUint64 version, NvFlowBool32 isPointerArray)
{
    NvFlowUint64 cachedIdx = 0u;
    for (; cachedIdx < ptr->cached.size; cachedIdx++)
    {
        auto& cached = ptr->cached[cachedIdx];
        if (cached->luid == luid && ptr->pathStack.size == cached->pathStack.size)
        {
            // check path stack
            bool pathStackMatches = true;
            for (NvFlowUint64 pathStackIdx = 0u; pathStackIdx < ptr->pathStack.size; pathStackIdx++)
            {
                if (NvFlowReflectStringCompare(ptr->pathStack[pathStackIdx], cached->pathStack[pathStackIdx]) != 0)
                {
                    pathStackMatches = false;
                    break;
                }
            }
            if (pathStackMatches)
            {
                break;
            }
        }
    }
    if (cachedIdx == ptr->cached.size)
    {
        cachedIdx = ptr->cached.allocateBack();
        ptr->cached[cachedIdx] = NvFlowReflectDeepCopyCached_create(luid, ptr->pathStack.data, ptr->pathStack.size);
    }
    auto cached = ptr->cached[cachedIdx];
    if (ptr->cached[cachedIdx]->version != version)
    {
        NvFlowReflectDeepCopy_reset(cached->deepCopy);
        NvFlowReflectDeepCopy_cleanCache(cached->deepCopy);
        cached->deepCopyData = NvFlowReflectDeepCopy_recursive(cached->deepCopy, luid, src, type, elementCount, isPointerArray);

        cached->version = version;
    }
    cached->lastUse = 0u;
    return cached->deepCopyData;
}

NV_FLOW_INLINE void NvFlowReflectDeepCopy_structRecursive(NvFlowReflectDeepCopy* ptr, NvFlowUint64 luid, NvFlowUint8* dst, const NvFlowReflectDataType* type, NvFlowUint64 elementCount, NvFlowBool32 isPointerArray)
{
    if (type->dataType == eNvFlowType_struct)
    {
        for (NvFlowUint64 elementIdx = 0u; elementIdx < elementCount; elementIdx++)
        {
            NvFlowUint8* dstArray = dst + type->elementSize * elementIdx;

            // attempt to find luid
            for (NvFlowUint childIdx = 0u; childIdx < type->childReflectDataCount; childIdx++)
            {
                const auto& childReflectData = type->childReflectDatas[childIdx];
                if (childReflectData.reflectMode == eNvFlowReflectMode_value &&
                    childReflectData.dataType->dataType == eNvFlowType_uint64)
                {
                    if (NvFlowReflectStringCompare(childReflectData.name, "luid") == 0)
                    {
                        luid = *((NvFlowUint64*)(dst + childReflectData.dataOffset));
                        break;
                    }
                }
            }

            // traverse all elements, searching for pointers/arrays
            for (NvFlowUint64 childIdx = 0u; childIdx < type->childReflectDataCount; childIdx++)
            {
                const auto& childReflectData = type->childReflectDatas[childIdx];
                ptr->pathStack.pushBack(childReflectData.name);
                if (childReflectData.dataType->dataType == eNvFlowType_struct &&
                    (childReflectData.reflectMode == eNvFlowReflectMode_value ||
                        childReflectData.reflectMode == eNvFlowReflectMode_valueVersioned))
                {
                    NvFlowReflectDeepCopy_structRecursive(ptr, luid, dstArray + childReflectData.dataOffset, childReflectData.dataType, 1u, NV_FLOW_FALSE);
                }
                if (childReflectData.reflectMode & eNvFlowReflectMode_pointerArray)
                {
                    // get pointer to pointer
                    NvFlowUint8** childPtr = (NvFlowUint8**)(dstArray + childReflectData.dataOffset);
                    NvFlowUint64 childElementCount = 1u;
                    NvFlowUint64 childVersion = 0u;
                    if ((*childPtr))
                    {
                        if (childReflectData.reflectMode & eNvFlowReflectMode_array)
                        {
                            childElementCount = *(NvFlowUint64*)(dstArray + childReflectData.arraySizeOffset);
                        }
                        if (childReflectData.reflectMode & eNvFlowReflectMode_valueVersioned)
                        {
                            childVersion = *(NvFlowUint64*)(dstArray + childReflectData.versionOffset);
                        }
                        NvFlowBool32 isPointerArray = (childReflectData.reflectMode & eNvFlowReflectMode_pointerArray) == eNvFlowReflectMode_pointerArray;

                        // conditionally attempt cached array
                        if (luid > 0u && childElementCount > 0u && childVersion > 0u && childReflectData.dataType->dataType != eNvFlowType_struct)
                        {
                            *childPtr = NvFlowReflectDeepCopy_cached(ptr, luid, *childPtr, childReflectData.dataType, childElementCount, childVersion, isPointerArray);
                        }
                        else
                        {
                            // recurse
                            *childPtr = NvFlowReflectDeepCopy_recursive(ptr, luid, *childPtr, childReflectData.dataType, childElementCount, isPointerArray);
                        }
                    }
                }
                ptr->pathStack.size--;
            }
        }
    }
}

NV_FLOW_INLINE NvFlowUint8* NvFlowReflectDeepCopy_recursive(NvFlowReflectDeepCopy* ptr, NvFlowUint64 luid, const NvFlowUint8* src, const NvFlowReflectDataType* type, NvFlowUint64 elementCount, NvFlowBool32 isPointerArray)
{
    const char* debugName = "root";
    if (ptr->pathStack.size > 0u)
    {
        debugName = ptr->pathStack[ptr->pathStack.size - 1u];
    }

    if (isPointerArray)
    {
        NvFlowUint8* dstData = NvFlowReflectDeepCopy_allocate(ptr, sizeof(void*) * elementCount, debugName);

        memcpy(dstData, src, sizeof(void*) * elementCount);

        // for each non-null pointer, recurse
        NvFlowUint8** dstArray = (NvFlowUint8**)dstData;
        for (NvFlowUint64 elementIdx = 0u; elementIdx < elementCount; elementIdx++)
        {
            if (dstArray[elementIdx])
            {
                dstArray[elementIdx] = NvFlowReflectDeepCopy_recursive(ptr, luid, dstArray[elementIdx], type, 1u, NV_FLOW_FALSE);
            }
        }
        return dstData;
    }

    NvFlowUint8* dstData = NvFlowReflectDeepCopy_allocate(ptr, type->elementSize * elementCount, debugName);

    memcpy(dstData, src, type->elementSize * elementCount);

    NvFlowReflectDeepCopy_structRecursive(ptr, luid, dstData, type, elementCount, isPointerArray);

    return dstData;
}

NV_FLOW_INLINE NvFlowUint8* NvFlowReflectDeepCopy_update(NvFlowReflectDeepCopy* ptr, const void* srcVoid, const NvFlowReflectDataType* type)
{
    const NvFlowUint8* src = (const NvFlowUint8*)srcVoid;
    NvFlowReflectDeepCopy_reset(ptr);
    NvFlowReflectDeepCopy_cleanCache(ptr);
    return NvFlowReflectDeepCopy_recursive(ptr, 0llu, src, type, 1u, NV_FLOW_FALSE);
}
