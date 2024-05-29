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

#ifndef NV_NSFOUNDATION_NS_H
#define NV_NSFOUNDATION_NS_H

/*! \file top level include file for shared foundation */

#include "Nv.h"

/**
Platform specific defines
*/
#if NV_WINDOWS_FAMILY || NV_XBOXONE
#pragma intrinsic(memcmp)
#pragma intrinsic(memcpy)
#pragma intrinsic(memset)
#pragma intrinsic(abs)
#pragma intrinsic(labs)
#endif

// An expression that should expand to nothing in non NV_CHECKED builds.
// We currently use this only for tagging the purpose of containers for memory use tracking.
#if NV_CHECKED
#define NV_DEBUG_EXP(x) (x)
#else
#define NV_DEBUG_EXP(x)
#endif

#define NV_SIGN_BITMASK 0x80000000

namespace nvidia
{
namespace shdfnd
{
// Int-as-bool type - has some uses for efficiency and with SIMD
typedef int IntBool;
static const IntBool IntFalse = 0;
static const IntBool IntTrue = 1;
}

} // namespace nvidia

#endif // #ifndef NV_NSFOUNDATION_NS_H
