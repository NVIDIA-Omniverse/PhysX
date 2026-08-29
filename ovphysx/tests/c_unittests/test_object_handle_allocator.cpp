// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "internal/sdk/ovphysxObjectHandleAllocator.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <thread>
#include <unordered_set>
#include <vector>

TEST(ObjectHandleAllocator, AllocatesNonzeroValuesWithoutReuse)
{
    std::atomic<uint64_t> nextHandle{ 1 };

    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), 1);
    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), 2);
    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), 3);
}

TEST(ObjectHandleAllocator, ExhaustionIsStickyAfterMaximumHandle)
{
    const uint64_t maximum = std::numeric_limits<uint64_t>::max();
    std::atomic<uint64_t> nextHandle{ maximum - 1 };

    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), maximum - 1);
    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), maximum);
    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), 0);
    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), 0);
    EXPECT_EQ(nextHandle.load(std::memory_order_relaxed), 0);
}

TEST(ObjectHandleAllocator, ConcurrentExhaustionNeverRevivesZero)
{
    const uint64_t maximum = std::numeric_limits<uint64_t>::max();
    constexpr size_t threadCount = 16;
    constexpr size_t availableHandles = 8;
    std::atomic<uint64_t> nextHandle{ maximum - availableHandles + 1 };
    std::vector<uint64_t> results(threadCount, 0);
    std::vector<std::thread> threads;
    threads.reserve(threadCount);

    for (size_t index = 0; index < threadCount; ++index)
    {
        threads.emplace_back([&nextHandle, &results, index]() {
            results[index] = ovphysx::internal::allocateOpaqueObjectHandle(nextHandle);
        });
    }
    for (std::thread& thread : threads)
        thread.join();

    std::unordered_set<uint64_t> distinctResults;
    size_t nonzeroCount = 0;
    for (uint64_t result : results)
    {
        if (result == 0)
            continue;
        ++nonzeroCount;
        distinctResults.insert(result);
    }

    // Assert the count and the distinct count separately. Checking only the set
    // size would pass if one serial were handed out twice while an extra thread
    // also succeeded, which is exactly the aliasing this allocator must prevent.
    EXPECT_EQ(nonzeroCount, availableHandles);
    EXPECT_EQ(distinctResults.size(), nonzeroCount);
    EXPECT_EQ(nextHandle.load(std::memory_order_relaxed), 0);
    EXPECT_EQ(ovphysx::internal::allocateOpaqueObjectHandle(nextHandle), 0);
}
