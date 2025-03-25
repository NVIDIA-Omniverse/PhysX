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

#include "NvFlowTypes.h"
#include "NvFlowArray.h"

namespace
{
    struct NvFlowLuidTable
    {
        NvFlowArray<NvFlowUint64> tableLuids;
        NvFlowArray<NvFlowUint64> tableParamsIndices;
    };

    void NvFlowLuidTable_reset(NvFlowLuidTable* ptr, NvFlowUint64 count)
    {
        ptr->tableLuids.size = 0u;
        ptr->tableParamsIndices.size = 0u;
        NvFlowUint64 tableSize = 1u;
        while (tableSize < count)
        {
            tableSize *= 2u;
        }
        tableSize *= 2u;
        ptr->tableLuids.reserve(tableSize);
        ptr->tableParamsIndices.reserve(tableSize);
        ptr->tableLuids.size = tableSize;
        ptr->tableParamsIndices.size = tableSize;
        for (NvFlowUint64 idx = 0llu; idx < tableSize; idx++)
        {
            ptr->tableLuids[idx] = 0llu;
            ptr->tableParamsIndices[idx] = ~0llu;
        }
    }

    void NvFlowLuidTable_insert(NvFlowLuidTable* ptr, NvFlowUint64 luid, NvFlowUint64 paramsIdx)
    {
        for (NvFlowUint64 attempt = 0llu; attempt < ptr->tableLuids.size; attempt++)
        {
            NvFlowUint64 idx = (luid + attempt) & (ptr->tableLuids.size - 1u);
            if (ptr->tableLuids[idx] == 0llu)
            {
                ptr->tableLuids[idx] = luid;
                ptr->tableParamsIndices[idx] = paramsIdx;
                break;
            }
        }
    }

    NvFlowUint64 NvFlowLuidTable_find(NvFlowLuidTable* ptr, NvFlowUint64 luid)
    {
        NvFlowUint64 paramsIdx = ~0llu;
        for (NvFlowUint64 attempt = 0llu; attempt < ptr->tableLuids.size; attempt++)
        {
            NvFlowUint64 idx = (luid + attempt) & (ptr->tableLuids.size - 1u);
            if (ptr->tableLuids[idx] == luid)
            {
                paramsIdx = ptr->tableParamsIndices[idx];
                break;
            }
            if (ptr->tableLuids[idx] == 0llu)
            {
                break;
            }
        }
        return paramsIdx;
    }
}
