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

#include "shaders/RadixSortParams.h"

#include "NvFlowExt.h"

#include "NvFlowUploadBuffer.h"
#include "NvFlowDynamicBuffer.h"

#include "shaders/RadixSort1CS.hlsl.h"
#include "shaders/RadixSort2CS.hlsl.h"
#include "shaders/RadixSort3CS.hlsl.h"
#include "shaders/RadixSortBlockCS.hlsl.h"
#include "shaders/RadixSortPadCS.hlsl.h"

namespace
{
    struct RadixSort
    {
        NvFlowContextInterface contextInterface = {};

        NvFlowUploadBuffer constantBuffer = {};

        RadixSort1CS_Pipeline radixSort1CS = {};
        RadixSort2CS_Pipeline radixSort2CS = {};
        RadixSort3CS_Pipeline radixSort3CS = {};
        RadixSortBlockCS_Pipeline radixSortBlockCS = {};
        RadixSortPadCS_Pipeline radixSortPadCS = {};

        NvFlowDynamicBuffer keyBuffers[2u];
        NvFlowDynamicBuffer valBuffers[2u];
        NvFlowDynamicBuffer counterBuffers[2u];
        NvFlowUint keyFrontIdx = 0u;
        NvFlowUint valFrontIdx = 0u;
        NvFlowUint counterFrontIdx = 0u;
    };

    NV_FLOW_CAST_PAIR(NvFlowRadixSort, RadixSort)

    NvFlowRadixSort* create(NvFlowContextInterface* contextInterface, NvFlowContext* context)
    {
        auto ptr = new RadixSort();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, contextInterface);

        RadixSort1CS_init(&ptr->contextInterface, context, &ptr->radixSort1CS);
        RadixSort2CS_init(&ptr->contextInterface, context, &ptr->radixSort2CS);
        RadixSort3CS_init(&ptr->contextInterface, context, &ptr->radixSort3CS);
        RadixSortBlockCS_init(&ptr->contextInterface, context, &ptr->radixSortBlockCS);
        RadixSortPadCS_init(&ptr->contextInterface, context, &ptr->radixSortPadCS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->keyBuffers[0], eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->keyBuffers[1], eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->valBuffers[0], eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->valBuffers[1], eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->counterBuffers[0], eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->counterBuffers[1], eNvFlowBufferUsage_bufferCopySrc | eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_rwStructuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        return cast(ptr);
    }

    void destroy(NvFlowContext* context, NvFlowRadixSort* radixSort)
    {
        auto ptr = cast(radixSort);

        NvFlowDynamicBuffer_destroy(context, &ptr->keyBuffers[0]);
        NvFlowDynamicBuffer_destroy(context, &ptr->keyBuffers[1]);
        NvFlowDynamicBuffer_destroy(context, &ptr->valBuffers[0]);
        NvFlowDynamicBuffer_destroy(context, &ptr->valBuffers[1]);
        NvFlowDynamicBuffer_destroy(context, &ptr->counterBuffers[0]);
        NvFlowDynamicBuffer_destroy(context, &ptr->counterBuffers[1]);

        NvFlowUploadBuffer_destroy(context, &ptr->constantBuffer);

        RadixSort1CS_destroy(context, &ptr->radixSort1CS);
        RadixSort2CS_destroy(context, &ptr->radixSort2CS);
        RadixSort3CS_destroy(context, &ptr->radixSort3CS);
        RadixSortBlockCS_destroy(context, &ptr->radixSortBlockCS);
        RadixSortPadCS_destroy(context, &ptr->radixSortPadCS);

        delete ptr;
    }

    void reserve(NvFlowContext* context, NvFlowRadixSort* radixSort, NvFlowUint numKeys)
    {
        auto ptr = cast(radixSort);

        NvFlowUint numBlocks = (numKeys + 1023u) / 1024u;
        NvFlowUint reserveNumKeys = 1024u * numBlocks;
        NvFlowUint reserveTotalCounters = 2u * 16u * numBlocks;

        NvFlowDynamicBuffer_resize(context, &ptr->keyBuffers[0], reserveNumKeys * sizeof(NvFlowUint));
        NvFlowDynamicBuffer_resize(context, &ptr->keyBuffers[1], reserveNumKeys * sizeof(NvFlowUint));
        NvFlowDynamicBuffer_resize(context, &ptr->valBuffers[0], reserveNumKeys * sizeof(NvFlowUint));
        NvFlowDynamicBuffer_resize(context, &ptr->valBuffers[1], reserveNumKeys * sizeof(NvFlowUint));
        NvFlowDynamicBuffer_resize(context, &ptr->counterBuffers[0], reserveTotalCounters * sizeof(NvFlowUint));
        NvFlowDynamicBuffer_resize(context, &ptr->counterBuffers[1], reserveTotalCounters * sizeof(NvFlowUint));
    }

    void getInputBuffers(NvFlowContext* context, NvFlowRadixSort* radixSort, NvFlowBufferTransient** pKeyBuffer, NvFlowBufferTransient** pValBuffer)
    {
        auto ptr = cast(radixSort);

        *pKeyBuffer = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx]);
        *pValBuffer = NvFlowDynamicBuffer_getTransient(context, &ptr->valBuffers[ptr->valFrontIdx]);
    }

    NvFlowBufferTransient* updateConstants(NvFlowContext* context, RadixSort* ptr, const RadixSortCSParams* params)
    {
        auto mapped = (RadixSortCSParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(RadixSortCSParams));
        *mapped = *params;
        return NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);
    }

    void sort(NvFlowContext* context, NvFlowRadixSort* radixSort, NvFlowUint numKeys, NvFlowUint numKeyBits)
    {
        auto ptr = cast(radixSort);

        if (numKeys == 0u)
        {
            return;
        }

        NvFlowUint numBlocks = (numKeys + 1023u) / 1024u;
        NvFlowUint numCounters = 16u * numBlocks;

        RadixSortCSParams sortParams = {};
        sortParams.gridDim = numBlocks;
        sortParams.passStart = 0u;
        sortParams.passMask = 0x0F;
        sortParams.passNumBits = 4u;
        sortParams.numCounters = numCounters;
        sortParams.numKeyBits = numKeyBits;
        sortParams.numKeys = numKeys;
        sortParams.pad1 = 0u;

        // pad as needed
        if (numKeys & 1023)
        {
            NvFlowBufferTransient* constantBuffer = updateConstants(context, ptr, &sortParams);

            RadixSortPadCS_PassParams passParams = {};
            passParams.paramsIn = constantBuffer;
            passParams.keyDst = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx]);

            NvFlowUint3 gridDim = { 1u, 1u, 1u };

            RadixSortPadCS_addPassCompute(context, &ptr->radixSortPadCS, gridDim, &passParams);
        }

        if (numBlocks == 1u)
        {
            NvFlowBufferTransient* constantBuffer = updateConstants(context, ptr, &sortParams);

            RadixSortBlockCS_PassParams passParams = {};
            passParams.paramsIn = constantBuffer;
            passParams.keyIn = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx]);
            passParams.valIn = NvFlowDynamicBuffer_getTransient(context, &ptr->valBuffers[ptr->valFrontIdx]);
            passParams.keyOut = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx ^ 1u]);
            passParams.valOut = NvFlowDynamicBuffer_getTransient(context, &ptr->valBuffers[ptr->valFrontIdx ^ 1u]);

            NvFlowUint3 gridDim = { 1u, 1u, 1u };

            RadixSortBlockCS_addPassCompute(context, &ptr->radixSortBlockCS, gridDim, &passParams);

            ptr->keyFrontIdx ^= 1u;
            ptr->valFrontIdx ^= 1u;
        }
        else
        {
            for (NvFlowUint passID = 0u; passID < numKeyBits; passID += 4)
            {
                sortParams.passStart = passID;
                sortParams.passNumBits = numKeyBits - passID;
                if (sortParams.passNumBits > 4u)
                {
                    sortParams.passNumBits = 4u;
                }
                sortParams.passMask = (1u << sortParams.passNumBits) - 1u;

                NvFlowBufferTransient* constantBuffer = updateConstants(context, ptr, &sortParams);

                {
                    RadixSort1CS_PassParams passParams = {};
                    passParams.paramsIn = constantBuffer;
                    passParams.keyIn = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx]);
                    passParams.countersOut = NvFlowDynamicBuffer_getTransient(context, &ptr->counterBuffers[ptr->counterFrontIdx]);

                    NvFlowUint3 gridDim = { 1u, numBlocks, 1u };

                    RadixSort1CS_addPassCompute(context, &ptr->radixSort1CS, gridDim, &passParams);
                }
                {
                    RadixSort2CS_PassParams passParams = {};
                    passParams.paramsIn = constantBuffer;
                    passParams.countersIn = NvFlowDynamicBuffer_getTransient(context, &ptr->counterBuffers[ptr->counterFrontIdx]);
                    passParams.countersOut = NvFlowDynamicBuffer_getTransient(context, &ptr->counterBuffers[ptr->counterFrontIdx ^ 1u]);

                    NvFlowUint3 gridDim = { 1u, 2u, 1u };

                    RadixSort2CS_addPassCompute(context, &ptr->radixSort2CS, gridDim, &passParams);

                    ptr->counterFrontIdx ^= 1u;
                }
                {
                    RadixSort3CS_PassParams passParams = {};
                    passParams.paramsIn = constantBuffer;
                    passParams.keyIn = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx]);
                    passParams.valIn = NvFlowDynamicBuffer_getTransient(context, &ptr->valBuffers[ptr->valFrontIdx]);
                    passParams.countersIn = NvFlowDynamicBuffer_getTransient(context, &ptr->counterBuffers[ptr->counterFrontIdx]);
                    passParams.keyOut = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx ^ 1u]);
                    passParams.valOut = NvFlowDynamicBuffer_getTransient(context, &ptr->valBuffers[ptr->valFrontIdx ^ 1u]);

                    NvFlowUint3 gridDim = { 1u, numBlocks, 1u };

                    RadixSort3CS_addPassCompute(context, &ptr->radixSort3CS, gridDim, &passParams);

                    ptr->keyFrontIdx ^= 1u;
                    ptr->valFrontIdx ^= 1u;
                }
            }
        }
    }

    void getOutputBuffers(NvFlowContext* context, NvFlowRadixSort* radixSort, NvFlowBufferTransient** pKeyBuffer, NvFlowBufferTransient** pValBuffer)
    {
        auto ptr = cast(radixSort);

        *pKeyBuffer = NvFlowDynamicBuffer_getTransient(context, &ptr->keyBuffers[ptr->keyFrontIdx]);
        *pValBuffer = NvFlowDynamicBuffer_getTransient(context, &ptr->valBuffers[ptr->valFrontIdx]);
    }
}

NvFlowRadixSortInterface* NvFlowGetRadixSortInterface()
{
    static NvFlowRadixSortInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowRadixSortInterface) };
    iface.create = create;
    iface.destroy = destroy;
    iface.reserve = reserve;
    iface.getInputBuffers = getInputBuffers;
    iface.sort = sort;
    iface.getOutputBuffers = getOutputBuffers;
    return &iface;
}
