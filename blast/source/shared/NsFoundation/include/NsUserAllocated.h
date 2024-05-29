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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#ifndef NV_NSFOUNDATION_NSUSERALLOCATED_H
#define NV_NSFOUNDATION_NSUSERALLOCATED_H

#include "NsAllocator.h"

namespace nvidia
{
namespace shdfnd
{
/**
Provides new and delete using a UserAllocator.
Guarantees that 'delete x;' uses the UserAllocator too.
*/
class UserAllocated
{
  public:
    // NV_SERIALIZATION
    NV_INLINE void* operator new(size_t, void* address)
    {
        return address;
    }
    //~NV_SERIALIZATION
    // Matching operator delete to the above operator new.  Don't ask me
    // how this makes any sense - Nuernberger.
    NV_INLINE void operator delete(void*, void*)
    {
    }

    template <typename Alloc>
    NV_INLINE void* operator new(size_t size, Alloc alloc, const char* fileName, int line)
    {
        return alloc.allocate(size, fileName, line);
    }
    template <typename Alloc>
    NV_INLINE void* operator new [](size_t size, Alloc alloc, const char* fileName, int line)
    { return alloc.allocate(size, fileName, line); }

    // placement delete
    template <typename Alloc>
    NV_INLINE void operator delete(void* ptr, Alloc alloc, const char* fileName, int line)
    {
        NV_UNUSED(fileName);
        NV_UNUSED(line);
        alloc.deallocate(ptr);
    }
    template <typename Alloc>
    NV_INLINE void operator delete [](void* ptr, Alloc alloc, const char* fileName, int line)
    {
        NV_UNUSED(fileName);
        NV_UNUSED(line);
        alloc.deallocate(ptr);
    } NV_INLINE void
    operator delete(void* ptr)
    {
        NonTrackingAllocator().deallocate(ptr);
    }
    NV_INLINE void operator delete [](void* ptr)
    { NonTrackingAllocator().deallocate(ptr); }
};
} // namespace shdfnd
} // namespace nvidia

#endif // #ifndef NV_NSFOUNDATION_NSUSERALLOCATED_H
