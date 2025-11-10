// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <chrono>
#include <stdlib.h>
#include <stdio.h>

namespace vcd
{

class Timer
{
public:
    Timer() : mStartTime(std::chrono::high_resolution_clock::now())
    {
    }

    void reset()
    {
        mStartTime = std::chrono::high_resolution_clock::now();
    }

    double getElapsedSeconds()
    {
        auto s = peekElapsedSeconds();
        reset();
        return s;
    }

    double peekElapsedSeconds()
    {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - mStartTime;
        return diff.count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> mStartTime;
};

class ScopedTime
{
public:
    ScopedTime(const char* action) : mAction(action)
    {
        mTimer.reset();
    }
    ~ScopedTime(void)
    {
        double dtime = mTimer.getElapsedSeconds();
        printf("%0.5f : %s\n", dtime, mAction);
    }

    const char* mAction{ nullptr };
    Timer mTimer;
};

} // namespace vcd
