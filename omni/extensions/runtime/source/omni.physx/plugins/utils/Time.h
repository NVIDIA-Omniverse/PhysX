// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "carb/Defines.h"

#if CARB_PLATFORM_LINUX
#    include <time.h>
#endif

struct CounterFrequencyToTensOfNanos
{
    uint64_t mNumerator;
    uint64_t mDenominator;
    CounterFrequencyToTensOfNanos(uint64_t inNum, uint64_t inDenom) : mNumerator(inNum), mDenominator(inDenom)
    {
    }

    // quite slow.
    uint64_t toTensOfNanos(uint64_t inCounter) const
    {
        return (inCounter * mNumerator) / mDenominator;
    }
};

class Time
{
public:
    typedef double Second;
    static const uint64_t sNumTensOfNanoSecondsInASecond = 100000000;
    // This is supposedly guaranteed to not change after system boot
    // regardless of processors, speedstep, etc.
    static const CounterFrequencyToTensOfNanos& getBootCounterFrequency();

    static CounterFrequencyToTensOfNanos getCounterFrequency();

    static uint64_t getCurrentCounterValue();

    // SLOW!!
    // Thar be a 64 bit divide in thar!
    static uint64_t getCurrentTimeInTensOfNanoSeconds()
    {
        uint64_t ticks = getCurrentCounterValue();
        return getBootCounterFrequency().toTensOfNanos(ticks);
    }

    Time();
    Second getElapsedSeconds();
    Second peekElapsedSeconds();
    Second getLastTime() const;

private:
#if CARB_PLATFORM_LINUX
    Second mLastTime;
#else
    int64_t mTickCount;
#endif
};


class ScopedTime
{
public:
    ScopedTime(const char* action) : mAction(action)
    {
    }

    ~ScopedTime(void)
    {
        double dtime = mTimer.getElapsedSeconds();
        printf("%0.5f : %s\n", dtime, mAction);
    }

private:
    const char* mAction{ nullptr };
    Time mTimer;
};

#define SCOPE_TIME_PROFILE(zoneName) ScopedTime scopedTimeProfileZone(zoneName)
