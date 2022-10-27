/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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