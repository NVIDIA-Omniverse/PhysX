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

#ifndef NV_UNIX_NSUNIXINTRINSICS_H
#define NV_UNIX_NSUNIXINTRINSICS_H

#include "Ns.h"
#include "NvAssert.h"
#include <math.h>

#if NV_ANDROID
#include <signal.h> // for Ns::debugBreak() { raise(SIGTRAP); }
#endif

#if 0
#include <libkern/OSAtomic.h>
#endif

// this file is for internal intrinsics - that is, intrinsics that are used in
// cross platform code but do not appear in the API

#if !(NV_LINUX || NV_ANDROID || NV_PS4 || NV_APPLE_FAMILY)
#error "This file should only be included by unix builds!!"
#endif

namespace nvidia
{
namespace shdfnd
{

NV_FORCE_INLINE void memoryBarrier()
{
    __sync_synchronize();
}

/*!
Return the index of the highest set bit. Undefined for zero arg.
*/
NV_INLINE uint32_t highestSetBitUnsafe(uint32_t v)
{

    return 31 -__builtin_clz(v);
}

/*!
Return the index of the highest set bit. Undefined for zero arg.
*/
NV_INLINE int32_t lowestSetBitUnsafe(uint32_t v)
{
    return __builtin_ctz(v);
}

/*!
Returns the index of the highest set bit. Undefined for zero arg.
*/
NV_INLINE uint32_t countLeadingZeros(uint32_t v)
{
    return __builtin_clz(v);
}

/*!
Prefetch aligned 64B x86, 32b ARM around \c ptr+offset.
*/
NV_FORCE_INLINE void prefetchLine(const void* ptr, uint32_t offset = 0)
{
    __builtin_prefetch(reinterpret_cast<const char * NV_RESTRICT>(ptr) + offset, 0, 3);
}

/*!
Prefetch \c count bytes starting at \c ptr.
*/
#if NV_ANDROID || NV_IOS
NV_FORCE_INLINE void prefetch(const void* ptr, uint32_t count = 1)
{
    const char* cp = static_cast<const char*>(ptr);
    size_t p = reinterpret_cast<size_t>(ptr);
    uint32_t startLine = uint32_t(p >> 5), endLine = uint32_t((p + count - 1) >> 5);
    uint32_t lines = endLine - startLine + 1;
    do
    {
        prefetchLine(cp);
        cp += 32;
    } while(--lines);
}
#else
NV_FORCE_INLINE void prefetch(const void* ptr, uint32_t count = 1)
{
    const char* cp = reinterpret_cast<const char*>(ptr);
    uint64_t p = size_t(ptr);
    uint64_t startLine = p >> 6, endLine = (p + count - 1) >> 6;
    uint64_t lines = endLine - startLine + 1;
    do
    {
        prefetchLine(cp);
        cp += 64;
    } while(--lines);
}
#endif

//! \brief platform-specific reciprocal
NV_CUDA_CALLABLE NV_FORCE_INLINE float recipFast(float a)
{
    return 1.0f / a;
}

//! \brief platform-specific fast reciprocal square root
NV_CUDA_CALLABLE NV_FORCE_INLINE float recipSqrtFast(float a)
{
    return 1.0f / ::sqrtf(a);
}

//! \brief platform-specific floor
NV_CUDA_CALLABLE NV_FORCE_INLINE float floatFloor(float x)
{
    return ::floorf(x);
}

#define NS_EXPECT_TRUE(x) x
#define NS_EXPECT_FALSE(x) x

} // namespace shdfnd
} // namespace nvidia

#endif // #ifndef NV_UNIX_NSUNIXINTRINSICS_H
