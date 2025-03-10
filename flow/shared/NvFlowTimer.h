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

#include "NvFlowTypes.h"

//#define NV_FLOW_PROFILE_ENABLED
//#define NV_FLOW_PROFILE_EVERY_FRAME

#ifdef NV_FLOW_PROFILE_EVERY_FRAME
#define NV_FLOW_PROFILE_RATE NV_FLOW_TRUE
#else
#define NV_FLOW_PROFILE_RATE (profileCount == 0)
#endif

#if defined(_WIN32)
#include <Windows.h>
#else
#include <time.h>
#endif

#include "NvFlowArray.h"

NV_FLOW_INLINE void NvFlowTimeStamp_capture(NvFlowUint64* ptr)
{
#if defined(_WIN32)
    LARGE_INTEGER tmpCpuTime = {};
    QueryPerformanceCounter(&tmpCpuTime);
    (*ptr) = tmpCpuTime.QuadPart;
#else
    timespec timeValue = {};
    clock_gettime(CLOCK_MONOTONIC, &timeValue);
    (*ptr) = 1E9 * NvFlowUint64(timeValue.tv_sec) + NvFlowUint64(timeValue.tv_nsec);
#endif
}

NV_FLOW_INLINE NvFlowUint64 NvFlowTimeStamp_frequency()
{
#if defined(_WIN32)
    LARGE_INTEGER tmpCpuFreq = {};
    QueryPerformanceFrequency(&tmpCpuFreq);
    return tmpCpuFreq.QuadPart;
#else
    return 1E9;
#endif
}

NV_FLOW_INLINE float NvFlowTimeStamp_diff(NvFlowUint64 begin, NvFlowUint64 end, NvFlowUint64 freq)
{
    return (float)(((double)(end - begin) / (double)(freq)));
}

#ifndef NV_FLOW_PROFILE_ENABLED
#define NV_FLOW_PROFILE_BEGIN(profileInterval, profileOffset)
#define NV_FLOW_PROFILE_TIMESTAMP(name)
#define NV_FLOW_PROFILE_FLUSH(name, logPrint)
#else
#define NV_FLOW_PROFILE_BEGIN(profileInterval, profileOffset) \
    static int profileCount = profileOffset; \
    profileCount++; \
    if (profileCount >= profileInterval) \
    { \
        profileCount = 0; \
    } \
    NvFlowArray<NvFlowUint64, 32u> profileTimes; \
    NvFlowArray<const char*, 32u> profileNames; \
    const NvFlowBool32 profileEnabled = NV_FLOW_PROFILE_RATE;

#define NV_FLOW_PROFILE_TIMESTAMP(name) \
    if (profileEnabled) \
    { \
        NvFlowTimeStamp_capture(&profileTimes[profileTimes.allocateBack()]); \
        profileNames.pushBack(#name); \
    }

#define NV_FLOW_PROFILE_FLUSH(name, logPrint) \
    if (profileEnabled && logPrint && profileTimes.size >= 2u) \
    { \
        NvFlowUint64 freq = NvFlowTimeStamp_frequency(); \
        float totalTime = NvFlowTimeStamp_diff(profileTimes[0u], profileTimes[profileTimes.size - 1u], freq); \
        for (NvFlowUint64 idx = 1u; idx < profileTimes.size; idx++) \
        { \
            float time = NvFlowTimeStamp_diff(profileTimes[idx - 1u], profileTimes[idx], freq); \
            if (time >= 0.001f * totalTime) \
            { \
                logPrint(eNvFlowLogLevel_warning, "[%s] %f ms", profileNames[idx], 1000.f * time); \
            } \
        } \
        logPrint(eNvFlowLogLevel_warning, "Total [%s] %f ms", #name, 1000.f * totalTime); \
    }
#endif
