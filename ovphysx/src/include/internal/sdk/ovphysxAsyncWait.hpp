// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-TASK-001
 * @covers AC-3 AC-8
 *
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-3 AC-4
 */

#pragma once

#include <chrono>
#include <cstdint>

namespace ovphysx::async::detail
{

inline uint64_t remaining_readiness_timeout(
    std::chrono::steady_clock::duration timeout,
    bool forever,
    std::chrono::steady_clock::time_point start,
    std::chrono::steady_clock::time_point now)
{
    if (forever)
        return UINT64_MAX;
    const std::chrono::steady_clock::duration elapsed = now - start;
    if (elapsed >= timeout)
        return 0;
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout - elapsed).count());
}


template <typename ReadyFn, typename NowFn, typename SleepFn>
bool wait_until_simulation_ready(
    bool noWait,
    std::chrono::steady_clock::duration timeout,
    std::chrono::steady_clock::time_point start,
    ReadyFn&& checkReady,
    NowFn&& now,
    SleepFn&& sleepFor)
{
    bool ready = checkReady();
    while (!ready)
    {
        const std::chrono::steady_clock::duration elapsed = now() - start;
        if (noWait || elapsed >= timeout)
            return false;

        const std::chrono::steady_clock::duration remaining = timeout - elapsed;
        const std::chrono::steady_clock::duration pollInterval = std::chrono::microseconds(100);
        sleepFor(remaining < pollInterval ? remaining : pollInterval);
        ready = checkReady();
    }

    return true;
}

} // namespace ovphysx::async::detail
