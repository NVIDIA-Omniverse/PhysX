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


#ifndef NVBLASTPXCALLBACKS_H
#define NVBLASTPXCALLBACKS_H

#include "NvBlastGlobals.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxAllocatorCallback.h"

/**
This file contains helper functions to get PxShared compatible versions of global AllocatorCallback and ErrorCallback.
*/


NV_INLINE physx::PxErrorCallback& NvBlastGetPxErrorCallback()
{
    class PxErrorCallbackWrapper : public physx::PxErrorCallback
    {
        virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override
        {
            NvBlastGlobalGetErrorCallback()->reportError((Nv::Blast::ErrorCode::Enum)code, message, file, line);
        }
    };
    static PxErrorCallbackWrapper wrapper;
    return wrapper;
}

NV_INLINE physx::PxAllocatorCallback& NvBlastGetPxAllocatorCallback()
{
    class PxAllocatorCallbackWrapper : public physx::PxAllocatorCallback
    {
        virtual void* allocate(size_t size, const char* typeName, const char* filename, int line) override
        {
            return NvBlastGlobalGetAllocatorCallback()->allocate(size, typeName, filename, line);
        }

        virtual void deallocate(void* ptr) override
        {
            NvBlastGlobalGetAllocatorCallback()->deallocate(ptr);
        }
    };
    static PxAllocatorCallbackWrapper wrapper;
    return wrapper;
}


#endif // #ifndef NVBLASTPXCALLBACKS_H
