// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxTime.h"
#include "foundation/windows/PxWindowsInclude.h"

using namespace physx;

static int64_t getTimeTicks()
{
	LARGE_INTEGER a;
	QueryPerformanceCounter(&a);
	return a.QuadPart;
}

static double getTickDuration()
{
	LARGE_INTEGER a;
	QueryPerformanceFrequency(&a);
	return 1.0f / double(a.QuadPart);
}

static double sTickDuration = getTickDuration();

static const PxCounterFrequencyToTensOfNanos gCounterFreq = PxTime::getCounterFrequency();

const PxCounterFrequencyToTensOfNanos& PxTime::getBootCounterFrequency()
{
	return gCounterFreq;
}

PxCounterFrequencyToTensOfNanos PxTime::getCounterFrequency()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return PxCounterFrequencyToTensOfNanos(PxTime::sNumTensOfNanoSecondsInASecond, (uint64_t)freq.QuadPart);
}

uint64_t PxTime::getCurrentCounterValue()
{
	LARGE_INTEGER ticks;
	QueryPerformanceCounter(&ticks);
	return (uint64_t)ticks.QuadPart;
}

PxTime::PxTime() : mTickCount(0)
{
	getElapsedSeconds();
}

PxTime::Second PxTime::getElapsedSeconds()
{
	int64_t lastTickCount = mTickCount;
	mTickCount = getTimeTicks();
	return (mTickCount - lastTickCount) * sTickDuration;
}

PxTime::Second PxTime::peekElapsedSeconds()
{
	return (getTimeTicks() - mTickCount) * sTickDuration;
}

PxTime::Second PxTime::getLastTime() const
{
	return mTickCount * sTickDuration;
}

