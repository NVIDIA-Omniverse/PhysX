// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-BATCH-001
 * @covers AC-4
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-4
 */
#pragma once
#include <chrono>
#include <cstddef>

namespace omni::physics::parse
{
// Optional coordinator-owned observations. No worker touches this record.
// fillAndJoinSeconds includes scheduling and the synchronization boundary;
// it is wall time, not summed worker CPU time.
struct DescriptorPipelineStats
{
    double prepareSeconds = 0.0, constructSeconds = 0.0, fillAndJoinSeconds = 0.0;
    double finalizeSeconds = 0.0, graphSeconds = 0.0, scanSeconds = 0.0;
    double coordinatorOtherSeconds = 0.0; // Enumeration, prefetch, uncategorized preparation and release.
    size_t entries = 0, tasks = 0, preparedPeakBytes = 0, outputMemberCapacityBytes = 0;
    size_t logicalRequests = 0, backingAllocations = 0;
    size_t poolLiveBytes = 0, poolPeakBytes = 0, poolRetainedBytes = 0, poolSlackBytes = 0;
};

class DescriptorPhaseTimer
{
public:
    explicit DescriptorPhaseTimer(double* destination) : mDestination(destination)
    {
        if (mDestination) mStart = Clock::now();
    }
    ~DescriptorPhaseTimer() { finish(); }
    void transition(double* destination)
    {
        finish();
        mDestination = destination;
        if (mDestination) mStart = Clock::now();
    }
    void finish()
    {
        if (mDestination)
        {
            *mDestination += std::chrono::duration<double>(Clock::now() - mStart).count();
            mDestination = nullptr;
        }
    }
private:
    using Clock = std::chrono::steady_clock;
    double* mDestination;
    Clock::time_point mStart;
};
}
