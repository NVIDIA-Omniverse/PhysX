// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

// This just implements a very simple multi-threaded job system.
// It doesn't use lock free queues or anything fancy like that.
// It doesn't support cancellation or priorities or anything like that either.
// It's just a very simple way to distribute a meaningful amount of
// 'work' across <n> number of threads.
#include <stdint.h>

#ifdef _MSC_VER
#    define SJS_ABI __cdecl
#else
#    define SJS_ABI
#endif

// Callback to actually perform the job
typedef void(SJS_ABI* SJS_jobCallback)(void* userPtr);


namespace vcd
{

class JobSystem
{
public:
    // Create in instance of the JobSystem with the number of threads specified.
    // More threads than available cores is not particularly beneficial.
    static JobSystem* create(uint32_t maxThreads);

    // Add a job to the queue of jobs to be performed, does not actually start the job yet.
    virtual void addJob(void* userPtr, SJS_jobCallback callback) = 0;

    // Start the jobs that have been posted, returns how many jobs are pending.
    virtual uint32_t startJobs(void) = 0;

    // Sleeps until all of the pending jobs have completed.
    virtual void waitForJobsToComplete(void) = 0;

    // Releases the JobSystem instance
    virtual void release(void) = 0;
};

} // namespace vcd
