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

    virtual uint32_t getNbSteps() const = 0; // the number of steps the benchmark will take
    virtual uint32_t getNbRuns() const
    {
        return 10;
    } // the number of times the benchmark will run

    virtual void startRun() = 0; // per-run initialization
    virtual void preStep() = 0; // pre-step setup  (happens before cache clearing and allocator reset)
    virtual Time::Second timedStep() // override this to manage your own time-measurement
    {
        Time timer;
        step();
        return timer.getElapsedSeconds();
    }
    virtual void endRun() = 0; // per run tear-down

    virtual ~BmBenchmark()
    {
    }

protected:
    virtual void step() = 0; // implement this to use the default timing mechanism
};

#endif
