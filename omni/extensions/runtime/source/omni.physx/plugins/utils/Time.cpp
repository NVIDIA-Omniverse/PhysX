// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"


#include "Time.h"

#include "carb/Defines.h"

#if CARB_PLATFORM_WINDOWS

#    include <windows.h>

namespace
{
    int64_t getTimeTicks()
    {
        LARGE_INTEGER a;
        QueryPerformanceCounter(&a);
        return a.QuadPart;
    }

    double getTickDuration()
    {
        LARGE_INTEGER a;
        QueryPerformanceFrequency(&a);
        return 1.0f / double(a.QuadPart);
    }

    double sTickDuration = getTickDuration();
} // namespace


static const CounterFrequencyToTensOfNanos gCounterFreq = Time::getCounterFrequency();

const CounterFrequencyToTensOfNanos& Time::getBootCounterFrequency()
{
    return gCounterFreq;
}

CounterFrequencyToTensOfNanos Time::getCounterFrequency()
{
    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);
    return CounterFrequencyToTensOfNanos(Time::sNumTensOfNanoSecondsInASecond, (uint64_t)freq.QuadPart);
}

uint64_t Time::getCurrentCounterValue()
{
    LARGE_INTEGER ticks;
    QueryPerformanceCounter(&ticks);
    return (uint64_t)ticks.QuadPart;
}

Time::Time() : mTickCount(0)
{
    getElapsedSeconds();
}

Time::Second Time::getElapsedSeconds()
{
    int64_t lastTickCount = mTickCount;
    mTickCount = getTimeTicks();
    return (mTickCount - lastTickCount) * sTickDuration;
}

Time::Second Time::peekElapsedSeconds()
{
    return (getTimeTicks() - mTickCount) * sTickDuration;
}

Time::Second Time::getLastTime() const
{
    return mTickCount * sTickDuration;
}

#else
#    include <sys/time.h>

#    include <time.h>


static const CounterFrequencyToTensOfNanos gCounterFreq = Time::getCounterFrequency();

const CounterFrequencyToTensOfNanos& Time::getBootCounterFrequency()
{
    return gCounterFreq;
}

static Time::Second getTimeSeconds()
{
    static struct timeval _tv;
    gettimeofday(&_tv, NULL);
    return double(_tv.tv_sec) + double(_tv.tv_usec) * 0.000001;
}

Time::Time()
{
    mLastTime = getTimeSeconds();
}

Time::Second Time::getElapsedSeconds()
{
    Time::Second curTime = getTimeSeconds();
    Time::Second diff = curTime - mLastTime;
    mLastTime = curTime;
    return diff;
}

Time::Second Time::peekElapsedSeconds()
{
    Time::Second curTime = getTimeSeconds();
    Time::Second diff = curTime - mLastTime;
    return diff;
}

Time::Second Time::getLastTime() const
{
    return mLastTime;
}

CounterFrequencyToTensOfNanos Time::getCounterFrequency()
{
    return CounterFrequencyToTensOfNanos(1, 10);
}

// Use real-time high-precision timer.
#    define CLOCKID CLOCK_REALTIME

uint64_t Time::getCurrentCounterValue()
{
    struct timespec mCurrTimeInt;
    clock_gettime(CLOCKID, &mCurrTimeInt);
    // Convert to nanos as this doesn't cause a large divide here
    return (static_cast<uint64_t>(mCurrTimeInt.tv_sec) * 1000000000) + (static_cast<uint64_t>(mCurrTimeInt.tv_nsec));
}

#endif
