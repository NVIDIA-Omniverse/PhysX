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

#include <stdarg.h>
#include "NvFlowTypes.h"

/// ************************** String Pool *********************************************

struct NvFlowStringPool;

NvFlowStringPool* NvFlowStringPoolCreate();

char* NvFlowStringPoolAllocate(NvFlowStringPool* pool, NvFlowUint64 size);

void NvFlowStringPoolTempAllocate(NvFlowStringPool* ptr, char** p_str_data, NvFlowUint64* p_str_size);

void NvFlowStringPoolTempAllocateCommit(NvFlowStringPool* ptr, char* str_data, NvFlowUint64 str_size);

void NvFlowStringPoolDestroy(NvFlowStringPool* pool);

void NvFlowStringPoolReset(NvFlowStringPool* pool);

char* NvFlowStringPrint(NvFlowStringPool* pool, const char* format, ...);

char* NvFlowStringPrintV(NvFlowStringPool* pool, const char* format, va_list args);

/// ************************** Macro utils *********************************

#define NV_FLOW_CSTR(X) NvFlowStringView{X, sizeof(X) - 1}

#define NvFlowStringToInteger(input) atoi(input)

#define NvFlowStringMakeView(input) NvFlowStringView{ input, (int)strlen(input) }

/// ************************** Char Utils *********************************************

NV_FLOW_INLINE int NvFlowCharIsWhiteSpace(char c)
{
    return c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '\f' || c == '\v';
}

NV_FLOW_INLINE int NvFlowCharIsWhiteSpaceButNotNewline(char c)
{
    return c == ' ' || c == '\r' || c == '\t' || c == '\f' || c == '\v';
}

NV_FLOW_INLINE int NvFlowCharIsAlphaUnderscore(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c == '_');
}

NV_FLOW_INLINE int NvFlowCharIsAlphaNum(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || (c == '_');
}

NV_FLOW_INLINE int NvFlowCharIsNum(char c)
{
    return (c >= '0' && c <= '9');
}

/// ************************** String Utils *********************************************

NV_FLOW_INLINE NvFlowUint64 NvFlowStringLength(const char* a)
{
    if (!a)
    {
        return 0;
    }
    int idx = 0;
    while (a[idx])
    {
        idx++;
    }
    return idx;
}

NV_FLOW_INLINE int NvFlowStringCompare(const char* a, const char* b)
{
    a = a ? a : "\0";
    b = b ? b : "\0";
    int idx = 0;
    while (a[idx] || b[idx])
    {
        if (a[idx] != b[idx])
        {
            return a[idx] < b[idx] ? -1 : +1;
        }
        idx++;
    }
    return 0;
}

NV_FLOW_INLINE char* NvFlowStringFromView(NvFlowStringPool* pool, const char* data, NvFlowUint64 size)
{
    char* str = NvFlowStringPoolAllocate(pool, size);
    for (NvFlowUint64 i = 0; i < size; i++)
    {
        str[i] = data[i];
    }
    return str;
}

NV_FLOW_INLINE void NvFlowStringSplitDelimFirst(NvFlowStringPool* pool, char** pFirst, char** pSecond, const char* input_data, char delim)
{
    NvFlowUint64 input_size = NvFlowStringLength(input_data);
    NvFlowUint64 slashIdx = 0;
    while (slashIdx < input_size)
    {
        if (input_data[slashIdx] == delim)
        {
            break;
        }
        slashIdx++;
    }
    *pFirst = NvFlowStringFromView(pool, input_data, slashIdx + 1);
    *pSecond = NvFlowStringFromView(pool, input_data + slashIdx + 1, input_size - slashIdx - 1);
}

NV_FLOW_INLINE void NvFlowStringSplitDelimLast(NvFlowStringPool* pool, char** pFirst, char** pSecond, const char* input_data, char delim)
{
    NvFlowUint64 input_size = NvFlowStringLength(input_data);
    NvFlowUint64 slashIdx = input_size - 1;
    while (slashIdx < input_size)
    {
        if (input_data[slashIdx] == delim)
        {
            break;
        }
        slashIdx--;
    }
    *pFirst = NvFlowStringFromView(pool, input_data, slashIdx + 1);
    *pSecond = NvFlowStringFromView(pool, input_data + slashIdx + 1, input_size - slashIdx - 1);
}

NV_FLOW_INLINE char* NvFlowStringDup(NvFlowStringPool* pool, const char* name)
{
    NvFlowUint64 name_size = NvFlowStringLength(name);
    return NvFlowStringFromView(pool, name, name_size);
}

NV_FLOW_INLINE char* NvFlowStringConcat(NvFlowStringPool* pool, const char* dir_data, const char* filename_data)
{
    NvFlowUint64 dir_size = NvFlowStringLength(dir_data);
    NvFlowUint64 filename_size = NvFlowStringLength(filename_data);
    char* s_data = NvFlowStringPoolAllocate(pool, dir_size + filename_size);
    for (NvFlowUint64 i = 0; i < dir_size; i++)
    {
        s_data[i] = dir_data[i];
    }
    for (NvFlowUint64 i = 0; i < filename_size; i++)
    {
        s_data[i + dir_size] = filename_data[i];
    }
    return s_data;
}

NV_FLOW_INLINE char* NvFlowStringConcatN(NvFlowStringPool* pool, const char** views, NvFlowUint64 numViews)
{
    NvFlowUint64 totalSize = 0;
    for (NvFlowUint64 viewIdx = 0; viewIdx < numViews; viewIdx++)
    {
        totalSize += NvFlowStringLength(views[viewIdx]);
    }
    char* s_data = NvFlowStringPoolAllocate(pool, totalSize);
    NvFlowUint64 dstOffset = 0;
    for (NvFlowUint64 viewIdx = 0; viewIdx < numViews; viewIdx++)
    {
        const char* view_data = views[viewIdx];
        NvFlowUint64 view_size = NvFlowStringLength(view_data);
        for (NvFlowUint64 i = 0; i < view_size; i++)
        {
            s_data[i + dstOffset] = view_data[i];
        }
        dstOffset += view_size;
    }
    return s_data;
}

NV_FLOW_INLINE char* NvFlowStringConcat3(NvFlowStringPool* pool, const char* a, const char* b, const char* c)
{
    const char* list[3u] = { a, b, c };
    return NvFlowStringConcatN(pool, list, 3u);
}

NV_FLOW_INLINE char* NvFlowStringConcat4(NvFlowStringPool* pool, const char* a, const char* b, const char* c, const char* d)
{
    const char* list[4u] = { a, b, c, d };
    return NvFlowStringConcatN(pool, list, 4u);
}

NV_FLOW_INLINE char* NvFlowStringTrimEnd(NvFlowStringPool* pool, const char* a_data, char trimChar)
{
    NvFlowUint64 a_size = NvFlowStringLength(a_data);
    while (a_size > 0 && a_data[a_size - 1] == trimChar)
    {
        a_size--;
    }
    return NvFlowStringFromView(pool, a_data, a_size);
}

NV_FLOW_INLINE char* NvFlowStringTrimBeginAndEnd(NvFlowStringPool* pool, const char* a_data, char trimChar)
{
    NvFlowUint64 a_size = NvFlowStringLength(a_data);
    while (a_size > 0 && a_data[0] == trimChar)
    {
        a_data++;
        a_size--;
    }
    while (a_size > 0 && a_data[a_size - 1] == trimChar)
    {
        a_size--;
    }
    return NvFlowStringFromView(pool, a_data, a_size);
}

/// ************************** File Utils *********************************************

const char* NvFlowTextFileLoad(NvFlowStringPool* pool, const char* filename);

void NvFlowTextFileStore(const char* text, const char* filename);

NvFlowBool32 NvFlowTextFileTestOpen(const char* filename);

void NvFlowTextFileRemove(const char* name);

void NvFlowTextFileRename(const char* oldName, const char* newName);

NvFlowBool32 NvFlowTextFileDiffAndWriteIfModified(const char* filenameDst, const char* filenameTmp);
