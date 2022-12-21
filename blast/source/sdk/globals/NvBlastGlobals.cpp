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
// Copyright (c) 2016-2022 NVIDIA Corporation. All rights reserved.


#include "NvBlastGlobals.h"
#include "NvBlastAssert.h"
#include <cstdlib>
#include <sstream>
#include <iostream>

#if NV_WINDOWS_FAMILY
#include <windows.h>
#endif

#if NV_WINDOWS_FAMILY || NV_LINUX_FAMILY
#include <malloc.h>
#endif


namespace Nv
{
namespace Blast
{

#if NV_WINDOWS_FAMILY
// on win32 we only have 8-byte alignment guaranteed, but the CRT provides special aligned allocation fns
NV_FORCE_INLINE void* platformAlignedAlloc(size_t size)
{
    return _aligned_malloc(size, 16);
}

NV_FORCE_INLINE void platformAlignedFree(void* ptr)
{
    _aligned_free(ptr);
}
#elif NV_LINUX_FAMILY
NV_FORCE_INLINE void* platformAlignedAlloc(size_t size)
{
    return ::memalign(16, size);
}

NV_FORCE_INLINE void platformAlignedFree(void* ptr)
{
    ::free(ptr);
}
#else
NV_FORCE_INLINE void* platformAlignedAlloc(size_t size)
{
    const int A = 16;
    unsigned char* mem = (unsigned char*)malloc(size + A);
    const unsigned char offset = (unsigned char)((uintptr_t)A - (uintptr_t)mem % A - 1);
    mem += offset;
    *mem++ = offset;
    return mem;
}

NV_FORCE_INLINE void platformAlignedFree(void* ptr)
{
    if (ptr != nullptr)
    {
        unsigned char* mem = (unsigned char*)ptr;
        const unsigned char offset = *--mem;
        ::free(mem - offset);
    }
}
#endif

class DefaultAllocatorCallback : public AllocatorCallback
{
public:
    virtual void* allocate(size_t size, const char* typeName, const char* filename, int line) override
    {
        NV_UNUSED(typeName);
        NV_UNUSED(filename);
        NV_UNUSED(line);
        return platformAlignedAlloc(size);
    }

    virtual void deallocate(void* ptr) override
    {
        platformAlignedFree(ptr);
    }
};
DefaultAllocatorCallback g_defaultAllocatorCallback;


class DefaultErrorCallback : public ErrorCallback
{
    virtual void reportError(ErrorCode::Enum code, const char* msg, const char* file, int line) override
    {
#if NV_DEBUG || NV_CHECKED
        std::stringstream str;
        str << "NvBlast ";
        bool critical = false;
        switch (code)
        {
        case ErrorCode::eNO_ERROR:          str << "[Info]";                critical = false; break;
        case ErrorCode::eDEBUG_INFO:        str << "[Debug Info]";          critical = false; break;
        case ErrorCode::eDEBUG_WARNING:     str << "[Debug Warning]";       critical = false; break;
        case ErrorCode::eINVALID_PARAMETER: str << "[Invalid Parameter]";   critical = true;  break;
        case ErrorCode::eINVALID_OPERATION: str << "[Invalid Operation]";   critical = true;  break;
        case ErrorCode::eOUT_OF_MEMORY:     str << "[Out of] Memory";       critical = true;  break;
        case ErrorCode::eINTERNAL_ERROR:    str << "[Internal Error]";      critical = true;  break;
        case ErrorCode::eABORT:             str << "[Abort]";               critical = true;  break;
        case ErrorCode::ePERF_WARNING:      str << "[Perf Warning]";        critical = false; break;
        default:                            NVBLAST_ASSERT(false);
        }
        str << file << "(" << line << "): " << msg << "\n";

        std::string message = str.str();
        std::cout << message;
#if NV_WINDOWS_FAMILY
        OutputDebugStringA(message.c_str());
#endif 
        NVBLAST_ASSERT_WITH_MESSAGE(!critical, message.c_str());
#else
        NV_UNUSED(code);
        NV_UNUSED(msg);
        NV_UNUSED(file);
        NV_UNUSED(line);
#endif
    }
};
DefaultErrorCallback g_defaultErrorCallback;


AllocatorCallback* g_allocatorCallback = &g_defaultAllocatorCallback;
ErrorCallback* g_errorCallback = &g_defaultErrorCallback;


} // namespace Blast
} // namespace Nv


//////// Global API implementation ////////

Nv::Blast::AllocatorCallback* NvBlastGlobalGetAllocatorCallback()
{
    return Nv::Blast::g_allocatorCallback;
}

void NvBlastGlobalSetAllocatorCallback(Nv::Blast::AllocatorCallback* allocator)
{
    Nv::Blast::g_allocatorCallback = allocator ? allocator : &Nv::Blast::g_defaultAllocatorCallback;
}

Nv::Blast::ErrorCallback* NvBlastGlobalGetErrorCallback()
{
    return Nv::Blast::g_errorCallback;
}

void NvBlastGlobalSetErrorCallback(Nv::Blast::ErrorCallback* errorCallback)
{
    Nv::Blast::g_errorCallback = errorCallback ? errorCallback : &Nv::Blast::g_defaultErrorCallback;
}
