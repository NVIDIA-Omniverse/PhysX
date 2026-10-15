// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-BATCH-001
 * @covers AC-4
 */

/**
 * Opt-in measurement adapter. Never installed on the production hot path by default.
 * @implements REQ-PARSE-CORE-006
 * @covers AC-10
 */
#pragma once

#include "Allocator.h"
#include <atomic>

namespace omni::physics::parse
{

struct DescriptorAllocationStats
{
    size_t requests = 0;
    size_t allocations = 0;
    size_t deallocations = 0;
    size_t liveBytes = 0;
    size_t peakBytes = 0;
};

// Place outside a future pool to count descriptor requests, or between the pool
// and its upstream to count backing blocks. Snapshot only at a quiescent boundary;
// the individual counters do not form an atomic multi-counter snapshot.
class CountingDescriptorAllocator final : public IDescriptorAllocator
{
public:
    explicit CountingDescriptorAllocator(IDescriptorAllocator& upstream) : mUpstream(upstream) {}

    bool reserve(size_t bytes, size_t alignment, size_t count) override
    {
        return mUpstream.reserve(bytes, alignment, count);
    }

    void* allocate(size_t bytes, size_t alignment = kDefaultDescriptorAlignment) override
    {
        mRequests.fetch_add(1, std::memory_order_relaxed);
        void* memory = mUpstream.allocate(bytes, alignment);
        if (memory)
        {
            mAllocations.fetch_add(1, std::memory_order_relaxed);
            const size_t live = mLiveBytes.fetch_add(bytes, std::memory_order_relaxed) + bytes;
            size_t peak = mPeakBytes.load(std::memory_order_relaxed);
            while (peak < live && !mPeakBytes.compare_exchange_weak(peak, live, std::memory_order_relaxed))
            {
            }
        }
        return memory;
    }

    void deallocate(void* memory, size_t bytes,
                    size_t alignment = kDefaultDescriptorAlignment) noexcept override
    {
        if (!memory)
            return;
        mUpstream.deallocate(memory, bytes, alignment);
        mLiveBytes.fetch_sub(bytes, std::memory_order_relaxed);
        mDeallocations.fetch_add(1, std::memory_order_relaxed);
    }

    DescriptorAllocationStats stats() const noexcept
    {
        return { mRequests.load(std::memory_order_relaxed), mAllocations.load(std::memory_order_relaxed),
                 mDeallocations.load(std::memory_order_relaxed), mLiveBytes.load(std::memory_order_relaxed),
                 mPeakBytes.load(std::memory_order_relaxed) };
    }

private:
    IDescriptorAllocator& mUpstream;
    std::atomic<size_t> mRequests{ 0 }, mAllocations{ 0 }, mDeallocations{ 0 };
    std::atomic<size_t> mLiveBytes{ 0 }, mPeakBytes{ 0 };
};

} // namespace omni::physics::parse
