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

#if defined(_WIN32)
#include <Windows.h>
#else
#include <time.h>
#endif

struct AppTimer
{
    NvFlowUint64 freq;
    NvFlowUint64 begin;
    NvFlowUint64 end;
    NvFlowUint state;
    float statTimeAccum;
    float statTimeCount;
};

NV_FLOW_INLINE void appTimerInit(AppTimer* ptr)
{
    ptr->freq = 1ull;
    ptr->begin = 0ull;
    ptr->end = 0ull;
    ptr->state = 0u;
    ptr->statTimeAccum = 0.f;
    ptr->statTimeCount = 0.f;
}

NV_FLOW_INLINE void appTimerDestroy(AppTimer* ptr)
{
    // NOP
}

NV_FLOW_INLINE void appTimerBegin(AppTimer* ptr)
{
    if (ptr->state == 0u)
    {
#if defined(_WIN32)
        LARGE_INTEGER tmpCpuFreq = {};
        QueryPerformanceFrequency(&tmpCpuFreq);
        ptr->freq = tmpCpuFreq.QuadPart;

        LARGE_INTEGER tmpCpuTime = {};
        QueryPerformanceCounter(&tmpCpuTime);
        ptr->begin = tmpCpuTime.QuadPart;
#else
        ptr->freq = 1E9;

        timespec timeValue = {};
        clock_gettime(CLOCK_MONOTONIC, &timeValue);
        ptr->begin = 1E9 * NvFlowUint64(timeValue.tv_sec) + NvFlowUint64(timeValue.tv_nsec);
#endif

        ptr->state = 1u;
    }
}

NV_FLOW_INLINE void appTimerEnd(AppTimer* ptr)
{
    if (ptr->state == 1u)
    {
#if defined(_WIN32)
        LARGE_INTEGER tmpCpuTime = {};
        QueryPerformanceCounter(&tmpCpuTime);
        ptr->end = tmpCpuTime.QuadPart;
#else
        timespec timeValue = {};
        clock_gettime(CLOCK_MONOTONIC, &timeValue);
        ptr->end = 1E9 * NvFlowUint64(timeValue.tv_sec) + NvFlowUint64(timeValue.tv_nsec);
#endif

        ptr->state = 0u;
    }
}

NV_FLOW_INLINE NvFlowBool32 appTimerGetResults(AppTimer* ptr, float* deltaTime)
{
    if (ptr->state == 0u)
    {
        *deltaTime = (float)(((double)(ptr->end - ptr->begin) / (double)(ptr->freq)));
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

NV_FLOW_INLINE NvFlowBool32 appTimerUpdateStats(AppTimer* ptr, float deltaTime, float sampleCount, float* pAverageTime)
{
    ptr->statTimeAccum += deltaTime;
    ptr->statTimeCount += 1.f;
    if (ptr->statTimeCount > sampleCount)
    {
        *pAverageTime = ptr->statTimeAccum / ptr->statTimeCount;
        ptr->statTimeAccum = 0.f;
        ptr->statTimeCount = 0.f;
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}
