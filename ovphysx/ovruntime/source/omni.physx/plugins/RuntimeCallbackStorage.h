// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-6
#pragma once

#include <carb/tasking/ITasking.h>
#include <atomic>
#include <memory>
#include <thread>

namespace omni::physx
{
struct RuntimeCallbackStorage
{
    explicit RuntimeCallbackStorage(carb::tasking::ITasking* taskingInterface) : tasking(taskingInterface) {}
    ~RuntimeCallbackStorage()
    {
        // Explicit shutdown retires the slot. If the app abandons its runtime,
        // let Carbonite's process-exit hooks reclaim it without calling a
        // possibly already shut-down foreign interface from a global destructor.
        if (retiredByShutdown.load(std::memory_order_acquire) && key != carb::tasking::kInvalidTaskStorageKey)
            tasking->freeTaskStorage(key);
    }
    carb::tasking::ITasking* tasking;
    carb::tasking::TaskStorageKey key = carb::tasking::kInvalidTaskStorageKey;
    std::atomic<bool> retiredByShutdown{false};
};

inline void retireRuntimeCallbackStorage(const std::shared_ptr<RuntimeCallbackStorage>& retired)
{
    if (retired)
    {
        retired->retiredByShutdown.store(true, std::memory_order_release);
        // The caller has unpublished this slot and retains the last owner.
        // Drain earlier observations before returning to provider teardown.
        while (retired.use_count() > 1)
            std::this_thread::yield();
    }
}
}
