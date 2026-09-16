// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-ALLOC-001
 * @covers AC-1 AC-2
 */

#include <gtest/gtest.h>

#include <common/foundation/Allocator.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

namespace
{

constexpr uint32_t kWaveCount = 4;
constexpr uint32_t kThreadCount = 16;
constexpr uint32_t kAllocationsPerThread = 512;
constexpr size_t kRecycleProbeSize = 128;
constexpr unsigned char kFirstRecycleProbeValue = 0xa5;
constexpr unsigned char kSecondRecycleProbeValue = 0x5a;

class SpinBarrier
{
public:
    explicit SpinBarrier(uint32_t participantCount)
        : mParticipantCount(participantCount), mArrivalCount(0), mGeneration(0)
    {
    }

    void arriveAndWait()
    {
        const uint32_t generation = mGeneration.load(std::memory_order_acquire);
        const uint32_t arrivalCount = mArrivalCount.fetch_add(1, std::memory_order_acq_rel) + 1u;
        if (arrivalCount == mParticipantCount)
        {
            mArrivalCount.store(0, std::memory_order_relaxed);
            mGeneration.fetch_add(1, std::memory_order_release);
            return;
        }

        while (mGeneration.load(std::memory_order_acquire) == generation)
            std::this_thread::yield();
    }

private:
    const uint32_t mParticipantCount;
    std::atomic<uint32_t> mArrivalCount;
    std::atomic<uint32_t> mGeneration;
};

size_t allocationSize(uint32_t waveIndex, uint32_t threadIndex, uint32_t allocationIndex)
{
    return 32u + ((waveIndex * 29u + threadIndex * 17u + allocationIndex * 13u) % 257u);
}

unsigned char payloadValue(uint32_t waveIndex, uint32_t threadIndex, uint32_t allocationIndex)
{
    return static_cast<unsigned char>(1u + ((waveIndex * 11u + threadIndex * 7u + allocationIndex * 5u) % 251u));
}

unsigned char replacementPayloadValue(uint32_t waveIndex, uint32_t threadIndex, uint32_t allocationIndex)
{
    return static_cast<unsigned char>(payloadValue(waveIndex, threadIndex, allocationIndex) ^ 0xffu);
}

bool payloadMatches(const void* memory, size_t size, unsigned char expected)
{
    const unsigned char* bytes = static_cast<const unsigned char*>(memory);
    for (size_t i = 0; i < size; ++i)
    {
        if (bytes[i] != expected)
            return false;
    }
    return true;
}

} // namespace

TEST(DefaultAllocatorConcurrency, ConcurrentAllocateAndFreeSurvivesTrackerGrowthAndReuse)
{
    Allocator* const allocator = GetAllocator();
    ASSERT_NE(allocator, nullptr);

    std::atomic<uint32_t> nullAllocations{ 0 };
    std::atomic<uint32_t> payloadMismatches{ 0 };

    for (uint32_t waveIndex = 0; waveIndex < kWaveCount; ++waveIndex)
    {
        std::atomic<uint32_t> readyThreads{ 0 };
        std::atomic<uint32_t> allocatedThreads{ 0 };
        std::atomic<bool> start{ false };
        SpinBarrier mixedOperationBarrier(kThreadCount);
        std::vector<std::thread> workers;
        workers.reserve(kThreadCount);

        for (uint32_t threadIndex = 0; threadIndex < kThreadCount; ++threadIndex)
        {
            workers.emplace_back([&, waveIndex, threadIndex]() {
                std::array<void*, kAllocationsPerThread> blocks{};

                readyThreads.fetch_add(1, std::memory_order_release);
                while (!start.load(std::memory_order_acquire))
                    std::this_thread::yield();

                for (uint32_t allocationIndex = 0; allocationIndex < kAllocationsPerThread; ++allocationIndex)
                {
                    const size_t size = allocationSize(waveIndex, threadIndex, allocationIndex);
                    void* const block = allocator->malloc(size);
                    blocks[allocationIndex] = block;
                    if (!block)
                    {
                        nullAllocations.fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }

                    std::memset(block, payloadValue(waveIndex, threadIndex, allocationIndex), size);
                    if ((allocationIndex & 7u) == 0u)
                        std::this_thread::yield();
                }

                // Keep all 8,192 blocks live before mixed replacement starts.
                // In Debug this grows the tracker when its existing capacity
                // is smaller, then the mixed phase repeatedly recycles slots.
                allocatedThreads.fetch_add(1, std::memory_order_release);
                while (allocatedThreads.load(std::memory_order_acquire) != kThreadCount)
                    std::this_thread::yield();

                // Each half-step releases eight malloc callers and eight free
                // callers together. The parity swap exercises both operation
                // orders while ownership remains local to each worker.
                for (uint32_t allocationIndex = 0; allocationIndex < kAllocationsPerThread; ++allocationIndex)
                {
                    void*& block = blocks[allocationIndex];
                    const size_t size = allocationSize(waveIndex, threadIndex, allocationIndex);
                    const unsigned char expected = payloadValue(waveIndex, threadIndex, allocationIndex);
                    const unsigned char replacementExpected =
                        replacementPayloadValue(waveIndex, threadIndex, allocationIndex);
                    void* replacement = nullptr;

                    if ((threadIndex & 1u) == 0u)
                    {
                        replacement = allocator->malloc(size);
                        if (!replacement)
                            nullAllocations.fetch_add(1, std::memory_order_relaxed);
                        else
                            std::memset(replacement, replacementExpected, size);
                    }
                    else if (block)
                    {
                        if (!payloadMatches(block, size, expected))
                            payloadMismatches.fetch_add(1, std::memory_order_relaxed);
                        allocator->free(block, false);
                        block = nullptr;
                    }

                    mixedOperationBarrier.arriveAndWait();

                    if ((threadIndex & 1u) == 0u)
                    {
                        if (block)
                        {
                            if (!payloadMatches(block, size, expected))
                                payloadMismatches.fetch_add(1, std::memory_order_relaxed);
                            allocator->free(block, false);
                            block = nullptr;
                        }
                    }
                    else
                    {
                        replacement = allocator->malloc(size);
                        if (!replacement)
                            nullAllocations.fetch_add(1, std::memory_order_relaxed);
                        else
                            std::memset(replacement, replacementExpected, size);
                    }

                    mixedOperationBarrier.arriveAndWait();
                    block = replacement;
                    mixedOperationBarrier.arriveAndWait();
                }

                // All mixed replacement epochs are complete before the final
                // payload validation and drain begin.
                for (uint32_t freeIndex = 0; freeIndex < kAllocationsPerThread; ++freeIndex)
                {
                    const uint32_t allocationIndex =
                        (threadIndex & 1u) != 0u ? kAllocationsPerThread - freeIndex - 1u : freeIndex;
                    void*& block = blocks[allocationIndex];
                    if (!block)
                        continue;

                    const size_t size = allocationSize(waveIndex, threadIndex, allocationIndex);
                    const unsigned char expected =
                        replacementPayloadValue(waveIndex, threadIndex, allocationIndex);
                    if (!payloadMatches(block, size, expected))
                        payloadMismatches.fetch_add(1, std::memory_order_relaxed);

                    allocator->free(block, false);
                    block = nullptr;

                    if ((freeIndex & 7u) == 0u)
                        std::this_thread::yield();
                }
            });
        }

        while (readyThreads.load(std::memory_order_acquire) != kThreadCount)
            std::this_thread::yield();
        start.store(true, std::memory_order_release);

        for (std::thread& worker : workers)
            worker.join();

        // Exercise two consecutive recycled-list entries after every wave,
        // including the last one, for which there is no following worker wave.
        void* const firstRecycleProbe = allocator->malloc(kRecycleProbeSize);
        ASSERT_NE(firstRecycleProbe, nullptr) << "wave " << waveIndex;
        std::memset(firstRecycleProbe, kFirstRecycleProbeValue, kRecycleProbeSize);

        void* const secondRecycleProbe = allocator->malloc(kRecycleProbeSize);
        ASSERT_NE(secondRecycleProbe, nullptr) << "wave " << waveIndex;
        ASSERT_NE(firstRecycleProbe, secondRecycleProbe) << "wave " << waveIndex;
        std::memset(secondRecycleProbe, kSecondRecycleProbeValue, kRecycleProbeSize);

        EXPECT_TRUE(payloadMatches(firstRecycleProbe, kRecycleProbeSize, kFirstRecycleProbeValue))
            << "wave " << waveIndex;
        EXPECT_TRUE(payloadMatches(secondRecycleProbe, kRecycleProbeSize, kSecondRecycleProbeValue))
            << "wave " << waveIndex;

        allocator->free(firstRecycleProbe, false);
        EXPECT_TRUE(payloadMatches(secondRecycleProbe, kRecycleProbeSize, kSecondRecycleProbeValue))
            << "wave " << waveIndex;
        allocator->free(secondRecycleProbe, false);
    }

    EXPECT_EQ(nullAllocations.load(), 0u);
    EXPECT_EQ(payloadMismatches.load(), 0u);
}
