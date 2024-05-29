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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTASSERT_H
#define NVBLASTASSERT_H


#include "NvPreprocessor.h"


#if !NV_ENABLE_ASSERTS
#define NVBLAST_ASSERT(exp) ((void)0)
#define NVBLAST_ALWAYS_ASSERT_MESSAGE(message) ((void)0)
#define NVBLAST_ASSERT_WITH_MESSAGE(condition, message) ((void)0)
#else
#if NV_VC
#define NVBLAST_CODE_ANALYSIS_ASSUME(exp)                                                                     \
    __analysis_assume(!!(exp)) // This macro will be used to get rid of analysis warning messages if a NVBLAST_ASSERT is used
// to "guard" illegal mem access, for example.
#else
#define NVBLAST_CODE_ANALYSIS_ASSUME(exp)
#endif
#define NVBLAST_ASSERT(exp)                                                                                   \
{                                                                                                             \
    static bool _ignore = false;                                                                              \
    if (!(exp) && !_ignore) NvBlastAssertHandler(#exp, __FILE__, __LINE__, _ignore);                          \
    NVBLAST_CODE_ANALYSIS_ASSUME(exp);                                                                        \
} ((void)0)
#define NVBLAST_ALWAYS_ASSERT_MESSAGE(message)                                                                    \
{                                                                                                             \
    static bool _ignore = false;                                                                              \
    if(!_ignore)                                                                                              \
    {                                                                                                         \
        NvBlastAssertHandler(message, __FILE__, __LINE__, _ignore);                                               \
    }                                                                                                         \
} ((void)0)
#define NVBLAST_ASSERT_WITH_MESSAGE(exp, message)                                                             \
{                                                                                                             \
    static bool _ignore = false;                                                                              \
    if (!(exp) && !_ignore) NvBlastAssertHandler(message, __FILE__, __LINE__, _ignore);                       \
    NVBLAST_CODE_ANALYSIS_ASSUME(exp);                                                                        \
} ((void)0)
#endif

#define NVBLAST_ALWAYS_ASSERT() NVBLAST_ASSERT(0)


extern "C"
{

NV_C_API void NvBlastAssertHandler(const char* expr, const char* file, int line, bool& ignore);

} // extern "C"


#endif // #ifndef NVBLASTASSERT_H
