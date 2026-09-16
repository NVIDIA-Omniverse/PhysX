// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-3 AC-4
 */

#pragma once

#include <chrono>

namespace ovphysx::async::detail
{

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
