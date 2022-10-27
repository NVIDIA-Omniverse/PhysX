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

#include "NvFlowTypes.h"
#include "NvFlowArray.h"

NV_FLOW_INLINE NvFlowUint NvFlowStringHashFNV(const char* a)
{
    // FNV-1a
    NvFlowUint hash = 2166136261u;
    NvFlowUint idx = 0u;
    if (a)
    {
        while (a[idx])
        {
            hash = 16777619u * (hash ^ (NvFlowUint)(a[idx]));
            idx++;
        }
    }
    return hash;
}

template<class T, NvFlowUint64 staticCapacity = 0u>
struct NvFlowStringHashTable
{
    NvFlowArray<NvFlowUint, staticCapacity> hashs;
    NvFlowArray<NvFlowArray<char>, staticCapacity> keys;
    NvFlowArray<T, staticCapacity> values;
    NvFlowUint64 keyCount = 0llu;

    NvFlowUint64 find(const char* path, NvFlowUint hash)
    {
        path = path ? path : "";
        NvFlowUint64 beginIdx = hash & (hashs.size - 1u);
        for (NvFlowUint64 iterIdx = 0u; iterIdx < hashs.size; iterIdx++)
        {
            NvFlowUint64 idx = (iterIdx + beginIdx) & (hashs.size - 1u);
            if (hashs[idx] == hash &&
                keys[idx].size > 0u &&
                strcmp(keys[idx].data, path) == 0)
            {
                return idx;
            }
        }
        return ~0llu;
    }

    NvFlowUint64 insertNoResize(const char* path, NvFlowUint hash, const T& value, NvFlowBool32* pSuccess = nullptr)
    {
        path = path ? path : "";
        if (pSuccess)
        {
            *pSuccess = NV_FLOW_FALSE;
        }
        NvFlowUint64 beginIdx = hash & (hashs.size - 1u);
        for (NvFlowUint64 iterIdx = 0u; iterIdx < hashs.size; iterIdx++)
        {
            NvFlowUint64 idx = (iterIdx + beginIdx) & (hashs.size - 1u);
            if (keys[idx].size == 0u)
            {
                keyCount++;
                hashs[idx] = hash;
                for (NvFlowUint64 strIdx = 0u; path[strIdx]; strIdx++)
                {
                    keys[idx].pushBack(path[strIdx]);
                }
                keys[idx].pushBack('\0');
                values[idx] = value;
                if (pSuccess)
                {
                    *pSuccess = NV_FLOW_TRUE;
                }
                return idx;
            }
            else if (hashs[idx] == hash &&
                keys[idx].size > 0u &&
                strcmp(keys[idx].data, path) == 0)
            {
                return idx;
            }
        }
        return ~0llu;
    }

    NvFlowUint64 insert(const char* path, NvFlowUint hash, const T& value, NvFlowBool32* pSuccess = nullptr)
    {
        // resize if adding key would make 50+% full
        if (2u * (keyCount + 1u) >= hashs.size)
        {
            NvFlowArray<NvFlowUint, staticCapacity> hashs_old(std::move(hashs));
            NvFlowArray<NvFlowArray<char>, staticCapacity> keys_old(std::move(keys));
            NvFlowArray<T, staticCapacity> values_old(std::move(values));
            NvFlowUint64 newSize = 1u;
            while (newSize <= hashs_old.size)
            {
                newSize *= 2u;
            }
            hashs.reserve(newSize);
            keys.reserve(newSize);
            values.reserve(newSize);
            hashs.size = newSize;
            keys.size = newSize;
            values.size = newSize;
            keyCount = 0u;  // reset key count, because insert counts it again
            for (NvFlowUint64 idx = 0u; idx < hashs_old.size; idx++)
            {
                if (keys_old[idx].size > 0u)
                {
                    insertNoResize(keys_old[idx].data, hashs_old[idx], values_old[idx], nullptr);
                }
            }
        }
        return insertNoResize(path, hash, value, pSuccess);
    }

    NvFlowBool32 erase(const char* path, NvFlowUint hash)
    {
        NvFlowUint64 findIdx = find(path, hash);
        if (findIdx != ~0llu)
        {
            keyCount--;
            hashs[findIdx] = 0u;
            keys[findIdx].size = 0u;
            values[findIdx] = T();
            return NV_FLOW_TRUE;
        }
        return NV_FLOW_FALSE;
    }
};
