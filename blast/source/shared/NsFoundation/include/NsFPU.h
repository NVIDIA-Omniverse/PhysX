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

#ifndef NV_NSFOUNDATION_NSFPU_H
#define NV_NSFOUNDATION_NSFPU_H

#include "Ns.h"
#include "NsIntrinsics.h"

// unsigned integer representation of a floating-point value.
#if NV_PS3

NV_FORCE_INLINE unsigned int NV_IR(const float x)
{
    union
    {
        int i;
        float f;
    } u;
    u.f = x;
    return u.i;
}

NV_FORCE_INLINE int NV_SIR(const float x)
{
    union
    {
        int i;
        float f;
    } u;

    u.f = x;
    return u.i;
}

NV_FORCE_INLINE float NV_FR(const unsigned int x)
{
    union
    {
        unsigned int i;
        float f;
    } u;
    u.i = x;
    return u.f;
}

#else
#define NV_IR(x) ((uint32_t&)(x))
#define NV_SIR(x) ((int32_t&)(x))
#define NV_FR(x) ((float&)(x))
#endif

// signed integer representation of a floating-point value.

// Floating-point representation of a integer value.

#define NV_SIGN_BITMASK 0x80000000

#define NV_FPU_GUARD shdfnd::FPUGuard scopedFpGuard;
#define NV_SIMD_GUARD shdfnd::SIMDGuard scopedFpGuard;

#define NV_SUPPORT_GUARDS (NV_WINDOWS_FAMILY || NV_XBOXONE || NV_LINUX || NV_PS4 || NV_OSX)

namespace nvidia
{
namespace shdfnd
{
// sets the default SDK state for scalar and SIMD units
class NV_FOUNDATION_API FPUGuard
{
  public:
    FPUGuard();  // set fpu control word for PhysX
    ~FPUGuard(); // restore fpu control word
  private:
    uint32_t mControlWords[8];
};

// sets default SDK state for simd unit only, lighter weight than FPUGuard
class SIMDGuard
{
  public:
    NV_INLINE SIMDGuard();  // set simd control word for PhysX
    NV_INLINE ~SIMDGuard(); // restore simd control word
  private:
#if NV_SUPPORT_GUARDS
    uint32_t mControlWord;
#endif
};

/**
\brief Enables floating point exceptions for the scalar and SIMD unit
*/
NV_FOUNDATION_API void enableFPExceptions();

/**
\brief Disables floating point exceptions for the scalar and SIMD unit
*/
NV_FOUNDATION_API void disableFPExceptions();

} // namespace shdfnd
} // namespace nvidia

#if NV_WINDOWS_FAMILY || NV_XBOXONE
#include "platform/windows/NsWindowsFPU.h"
#elif NV_LINUX || NV_PS4 || NV_OSX
#include "platform/unix/NsUnixFPU.h"
#else
NV_INLINE nvidia::shdfnd::SIMDGuard::SIMDGuard()
{
}
NV_INLINE nvidia::shdfnd::SIMDGuard::~SIMDGuard()
{
}
#endif

#endif // #ifndef NV_NSFOUNDATION_NSFPU_H
