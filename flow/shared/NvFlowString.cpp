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


#include "NvFlowString.h"

#include "NvFlowArray.h"

#include <stdio.h>

struct NvFlowStringPool
{
    NvFlowArray<NvFlowArray<char>, 16u> heaps;
};

NvFlowStringPool* NvFlowStringPoolCreate()
{
    return new NvFlowStringPool();
}

void NvFlowStringPoolAllocate_newHeap(NvFlowStringPool* ptr, NvFlowUint64 allocSize)
{
    auto& currentHeap = ptr->heaps[ptr->heaps.allocateBack()];

    NvFlowUint64 heapSize = 4096u;    // default heap size
    while (heapSize < allocSize)
    {
        heapSize *= 2u;
    }

    currentHeap.reserve(heapSize);
}

NvFlowUint64 NvFlowStringPool_alignment(NvFlowUint64 size)
{
    return 8u * ((size + 7u) / 8u);
}

char* NvFlowStringPoolAllocate_internal(NvFlowStringPool* ptr, NvFlowUint64 size)
{
    NvFlowUint64 allocSize = NvFlowStringPool_alignment(size);

    if (ptr->heaps.size > 0u)
    {
        auto& currentHeap = ptr->heaps[ptr->heaps.size - 1u];

        if (currentHeap.size + allocSize <= currentHeap.capacity)
        {
            char* ret = currentHeap.data + currentHeap.size;
            ret[size - 1] = 0;
            currentHeap.size += allocSize;
            return ret;
        }
    }

    NvFlowStringPoolAllocate_newHeap(ptr, allocSize);

    return NvFlowStringPoolAllocate_internal(ptr, size);
}

char* NvFlowStringPoolAllocate(NvFlowStringPool* ptr, NvFlowUint64 size)
{
    return NvFlowStringPoolAllocate_internal(ptr, size + 1);
}

void NvFlowStringPoolTempAllocate(NvFlowStringPool* ptr, char** p_str_data, NvFlowUint64* p_str_size)
{
    if (ptr->heaps.size > 0u)
    {
        auto& currentHeap = ptr->heaps[ptr->heaps.size - 1u];

        char* str_data = currentHeap.data + currentHeap.size;
        NvFlowUint64 str_size = currentHeap.capacity - currentHeap.size;
        if (str_size > 0)
        {
            str_data[str_size - 1] = 0;
            str_size--;
            *p_str_size = str_size;
            *p_str_data = str_data;
            return;
        }
    }

    NvFlowStringPoolAllocate_newHeap(ptr, 8u);

    NvFlowStringPoolTempAllocate(ptr, p_str_data, p_str_size);
}

void NvFlowStringPoolTempAllocateCommit(NvFlowStringPool* ptr, char* str_data, NvFlowUint64 str_size)
{
    // to reverse the str_size-- in NvFlowStringPoolTempAllocate()
    str_size++;

    if (ptr->heaps.size > 0u)
    {
        auto& currentHeap = ptr->heaps[ptr->heaps.size - 1u];

        char* compStr_data = currentHeap.data + currentHeap.size;
        NvFlowUint64 compStr_size = currentHeap.capacity - currentHeap.size;
        if (str_data == compStr_data && str_size <= compStr_size)
        {
            NvFlowUint64 allocSize = NvFlowStringPool_alignment(str_size);
            currentHeap.size += allocSize;
        }
    }
}

void NvFlowStringPoolDestroy(NvFlowStringPool* ptr)
{
    delete ptr;
}

void NvFlowStringPoolReset(NvFlowStringPool* ptr)
{
    for (NvFlowUint64 heapIdx = 0u; heapIdx < ptr->heaps.size; heapIdx++)
    {
        ptr->heaps[heapIdx].size = 0u;
    }
    ptr->heaps.size = 0u;
}

char* NvFlowStringPrintV(NvFlowStringPool* pool, const char* format, va_list args)
{
    va_list argsCopy;
    va_copy(argsCopy, args);

    NvFlowUint64 str_size = ~0llu;
    char* str_data = nullptr;
    NvFlowStringPoolTempAllocate(pool, &str_data, &str_size);

    NvFlowUint64 count = (NvFlowUint64)vsnprintf(str_data, str_size + 1, format, args);

    if (count <= str_size)
    {
        str_size = count;
        NvFlowStringPoolTempAllocateCommit(pool, str_data, str_size);
    }
    else
    {
        str_data = NvFlowStringPoolAllocate(pool, count);
        str_size = count;

        count = vsnprintf(str_data, str_size + 1, format, argsCopy);
    }

    va_end(argsCopy);

    return str_data;
}

char* NvFlowStringPrint(NvFlowStringPool* pool, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char* str = NvFlowStringPrintV(pool, format, args);

    va_end(args);

    return str;
}

/// ************************** File Utils *********************************************

const char* NvFlowTextFileLoad(NvFlowStringPool* pool, const char* filename)
{
    FILE* file = nullptr;
#if defined(_WIN32)
    fopen_s(&file, filename, "r");
#else
    file = fopen(filename, "r");
#endif

    if (file == nullptr)
    {
        return nullptr;
    }

    NvFlowUint64 chunkSize = 4096u;

    NvFlowArray<const char*, 8u> chunks;

    size_t readBytes = 0u;
    do
    {
        chunkSize *= 2u;

        char* chunkStr = NvFlowStringPoolAllocate(pool, chunkSize);
        chunkStr[0] = '\0';

        readBytes = fread(chunkStr, 1u, chunkSize, file);
        chunkStr[readBytes] = '\0';

        chunks.pushBack(chunkStr);

    } while(readBytes == chunkSize);

    fclose(file);

    const char* text_data = (chunks.size == 1u) ? chunks[0u] : NvFlowStringConcatN(pool, chunks.data, chunks.size);

    //NvFlowUint64 strLength = NvFlowStringLength(text_data);
    //printf("NvFlowTextureFileLoad(%s) %llu bytes in %llu chunks\n", filename, strLength, chunks.size);

    return text_data;
}

void NvFlowTextFileStore(const char* text_data, const char* filename)
{
    FILE* file = nullptr;
#if defined(_WIN32)
    fopen_s(&file, filename, "w");
#else
    file = fopen(filename, "w");
#endif

    if (file == nullptr)
    {
        return;
    }

    NvFlowUint64 text_size = NvFlowStringLength(text_data);
    fwrite(text_data, 1u, text_size, file);

    fclose(file);
}

NvFlowBool32 NvFlowTextFileTestOpen(const char* filename)
{
    FILE* file = nullptr;
#if defined(_WIN32)
    fopen_s(&file, filename, "r");
#else
    file = fopen(filename, "r");
#endif
    if (file)
    {
        fclose(file);
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

void NvFlowTextFileRemove(const char* name)
{
    remove(name);
}

void NvFlowTextFileRename(const char* oldName, const char* newName)
{
    rename(oldName, newName);
}

NvFlowBool32 NvFlowTextFileDiffAndWriteIfModified(const char* filenameDst, const char* filenameTmp)
{
    FILE* fileTmp = nullptr;
    FILE* fileDst = nullptr;
    bool match = true;

#if defined(_WIN32)
    fopen_s(&fileDst, filenameDst, "r");
#else
    fileDst = fopen(filenameDst, "r");
#endif
    if (fileDst)
    {
#if defined(_WIN32)
        fopen_s(&fileTmp, filenameTmp, "r");
#else
        fileTmp = fopen(filenameTmp, "r");
#endif
        if (fileTmp)
        {
            while (1)
            {
                int a = fgetc(fileTmp);
                int b = fgetc(fileDst);

                if (a == EOF && b == EOF)
                {
                    break;
                }
                else if (a != b)
                {
                    match = false;
                    break;
                }
            }

            fclose(fileTmp);
        }
        else
        {
            match = false;
        }
        fclose(fileDst);
    }
    else
    {
        match = false;
    }

    if (!match)
    {
        remove(filenameDst);
        rename(filenameTmp, filenameDst);
    }

    // always cleanup temp file
    remove(filenameTmp);

    return !match;
}
