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


#ifndef NVBLASTPREPROCESSORINTERNAL_H
#define NVBLASTPREPROCESSORINTERNAL_H


#include "NvPreprocessor.h"


/**
Macros for more convenient logging
*/
#define NVBLASTLL_LOG_ERROR(_logFn, _msg)       if (_logFn != nullptr) { _logFn(NvBlastMessage::Error, _msg, __FILE__, __LINE__); } ((void)0)
#define NVBLASTLL_LOG_WARNING(_logFn, _msg)     if (_logFn != nullptr) { _logFn(NvBlastMessage::Warning, _msg, __FILE__, __LINE__); } ((void)0)
#define NVBLASTLL_LOG_INFO(_logFn, _msg)        if (_logFn != nullptr) { _logFn(NvBlastMessage::Info, _msg, __FILE__, __LINE__); } ((void)0)
#define NVBLASTLL_LOG_DEBUG(_logFn, _msg)       if (_logFn != nullptr) { _logFn(NvBlastMessage::Debug, _msg, __FILE__, __LINE__); } ((void)0)


/** Blast will check function parameters for debug and checked builds. */
#define NVBLASTLL_CHECK_PARAMS (NV_DEBUG || NV_CHECKED)


#if NVBLASTLL_CHECK_PARAMS
#define NVBLASTLL_CHECK(_expr, _logFn, _msg, _onFail)                                                                       \
    {                                                                                                                   \
        if(!(_expr))                                                                                                    \
        {                                                                                                               \
            if (_logFn) { _logFn(NvBlastMessage::Error, _msg, __FILE__, __LINE__); }                                    \
            { _onFail; };                                                                                               \
        }                                                                                                               \
    }                                                                                                                   
#else
#define NVBLASTLL_CHECK(_expr, _logFn, _msg, _onFail) NV_UNUSED(_logFn)
#endif


/**
Convenience macro to replace deprecated UINT32_MAX
*/
#ifndef UINT32_MAX
#include <limits>
#define UINT32_MAX  (std::numeric_limits<uint32_t>::max())
#endif


#endif // ifndef NVBLASTPREPROCESSORINTERNAL_H
