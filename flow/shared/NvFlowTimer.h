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

#if 1
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
	const NvFlowBool32 profileEnabled = (profileCount == 0);

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