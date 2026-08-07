// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef BENCHMARK_BASE_H
#define BENCHMARK_BASE_H

#include "BmTime.h"

class BmRecord;

// Each benchmark will be run multiple times. For each step, the minimal time over all runs will be recorded

class BmBenchmark
{
public:
    virtual bool isValid() const { return true; };

    virtual uint32_t getNbSteps() const = 0;
    virtual uint32_t getNbRuns() const
    {
        return 10;
    }

    virtual void startRun() = 0;
    virtual void preStep() = 0; // happens before cache clearing and allocator reset
    virtual Time::Second timedStep() // override this to manage your own time-measurement
    {
        Time timer;
        step();
        return timer.getElapsedSeconds();
    }
    virtual void endRun() = 0;

    virtual ~BmBenchmark()
    {
    }

protected:
    virtual void step() = 0; // implement this to use the default timing mechanism
};

#endif
