// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BATCH-001
 * @covers AC-2 AC-3 AC-4
 */
#pragma once

#include "Allocator.h"

#include <algorithm>
#include <exception>
#include <cassert>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace omni::physics::parse
{

struct DescriptorBatchStats
{
    size_t requests = 0;
    size_t backingAllocations = 0;
    size_t liveBytes = 0;
    size_t peakBytes = 0;
    size_t retainedBytes = 0;
    size_t slackBytes = 0;
};

// Coordinator-side storage. Only construction, reclamation and trim use this
// allocator; workers receive already constructed objects and never call it.
// The upstream allocator must outlive the pool, and the pool every DescPtr.
class DescriptorBatchAllocator final : public IDescriptorAllocator
{
public:
    explicit DescriptorBatchAllocator(IDescriptorAllocator& upstream) : mUpstream(upstream) {}

    ~DescriptorBatchAllocator() override
    {
        assert(mStats.liveBytes == 0);
    }

    IDescriptorAllocator& upstream() const noexcept { return mUpstream; }

    // Ensure capacity for this many ADDITIONAL live objects of the exact layout.
    // A fresh homogeneous batch obtains exactly one upstream payload block.
    // Failed growth preserves all existing blocks, free slots and live objects.
    bool reserve(size_t bytes, size_t alignment, size_t count) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return reserveLocked(bytes, alignment, count);
    }

    void* allocate(size_t bytes, size_t alignment = kDefaultDescriptorAlignment) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        ++mStats.requests;
        if (!reserveLocked(bytes, alignment, 1))
            return nullptr;
        for (const std::unique_ptr<Block>& block : mBlocks)
        {
            if (block->bytes == bytes && block->alignment == alignment && !block->free.empty())
            {
                const size_t index = block->free.back();
                block->free.pop_back();
                block->used[index] = 1;
                mStats.liveBytes += bytes;
                mStats.peakBytes = std::max(mStats.peakBytes, mStats.liveBytes);
                return static_cast<unsigned char*>(block->memory) + index * block->stride;
            }
        }
        return nullptr;
    }

    // Claim all slots under one coordinator lock. No construction runs under
    // the lock; callers can unwind constructors through the ordinary deleter.
    bool allocateSlots(size_t bytes, size_t alignment, void** slots, size_t count)
    {
        if (!count) return true;
        std::lock_guard<std::mutex> lock(mMutex);
        if (!reserveLocked(bytes, alignment, count)) return false;
        size_t written = 0;
        for (const std::unique_ptr<Block>& block : mBlocks)
        {
            if (block->bytes != bytes || block->alignment != alignment) continue;
            while (written < count && !block->free.empty())
            {
                const size_t index = block->free.back();
                block->free.pop_back();
                block->used[index] = 1;
                slots[written++] = static_cast<unsigned char*>(block->memory) + index * block->stride;
            }
            if (written == count) break;
        }
        assert(written == count);
        mStats.requests += count;
        mStats.liveBytes += bytes * count;
        mStats.peakBytes = std::max(mStats.peakBytes, mStats.liveBytes);
        return true;
    }

    void deallocate(void* memory, size_t bytes,
                    size_t alignment = kDefaultDescriptorAlignment) noexcept override
    {
        if (!memory)
            return;
        std::lock_guard<std::mutex> lock(mMutex);
        const uintptr_t address = reinterpret_cast<uintptr_t>(memory);
        for (const std::unique_ptr<Block>& block : mBlocks)
        {
            const uintptr_t begin = reinterpret_cast<uintptr_t>(block->memory);
            if (address < begin || address - begin >= block->payloadBytes)
                continue;
            const size_t offset = address - begin;
            if (block->bytes != bytes || block->alignment != alignment || offset % block->stride != 0)
                std::terminate(); // Invalid allocator provenance is a caller contract violation.
            const size_t index = offset / block->stride;
            if (!block->used[index]) std::terminate();
            block->used[index] = 0;
            // Capacity was established before the payload allocation; no heap call.
            block->free.push_back(index);
            mStats.liveBytes -= bytes;
            return;
        }
        std::terminate(); // Foreign pointer: fail loudly in release builds too.
    }

    // Reclaim only wholly unused blocks. Live addresses never change, including
    // when other threads return descriptors during an incremental scan.
    void trim()
    {
        std::lock_guard<std::mutex> lock(mMutex);
        for (auto it = mBlocks.begin(); it != mBlocks.end();)
        {
            if ((*it)->free.size() == (*it)->used.size())
            {
                mStats.retainedBytes -= (*it)->payloadBytes;
                it = mBlocks.erase(it);
            }
            else
                ++it;
        }
    }

    DescriptorBatchStats stats() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        DescriptorBatchStats result = mStats;
        result.slackBytes = result.retainedBytes - result.liveBytes;
        return result;
    }

private:
    struct Block
    {
        IDescriptorAllocator& upstream;
        size_t bytes, alignment, stride, payloadBytes;
        std::vector<size_t> free;
        std::vector<uint8_t> used;
        void* memory = nullptr;

        Block(IDescriptorAllocator& allocator, size_t objectBytes, size_t objectAlignment,
              size_t objectStride, size_t count)
            : upstream(allocator), bytes(objectBytes), alignment(objectAlignment), stride(objectStride),
              payloadBytes(count * objectStride), free(count), used(count, 0)
        {
            for (size_t index = 0; index < count; ++index)
                free[index] = count - 1 - index;
            memory = upstream.allocate(payloadBytes, alignment);
        }
        ~Block()
        {
            if (memory)
                upstream.deallocate(memory, payloadBytes, alignment);
        }
    };

    bool reserveLocked(size_t bytes, size_t alignment, size_t count)
    {
        if (!bytes || !alignment || (alignment & (alignment - 1)))
            return false;
        const size_t max = std::numeric_limits<size_t>::max();
        if (bytes > max - (alignment - 1))
            return false;
        const size_t stride = (bytes + alignment - 1) & ~(alignment - 1);
        if (count > max / stride)
            return false;
        size_t available = 0;
        size_t capacity = 0;
        for (const std::unique_ptr<Block>& block : mBlocks)
        {
            if (block->bytes == bytes && block->alignment == alignment)
            {
                const size_t remaining = count - available;
                if (block->free.size() >= remaining)
                    return true;
                available += block->free.size();
                capacity += block->used.size();
            }
        }
        if (available == count)
            return true;
        const size_t deficit = count - available;
        const size_t maxGrowth = (max - mStats.retainedBytes) / stride;
        if (deficit > maxGrowth)
            return false;
        // Amortize small prototype scans without doubling a large live layout
        // shared by unrelated descriptor families or overlapping scans.
        constexpr size_t maxSpeculativeBytes = 1024 * 1024;
        const size_t growth = std::min(std::max(deficit, std::min(capacity, maxSpeculativeBytes / stride)), maxGrowth);
        auto allocateBlock = [&](size_t slots)
        {
            try
            {
                std::unique_ptr<Block> block =
                    std::make_unique<Block>(mUpstream, bytes, alignment, stride, slots);
                if (!block->memory)
                    return false;
                mBlocks.push_back(std::move(block));
            }
            catch (const std::bad_alloc&)
            {
                return false;
            }
            ++mStats.backingAllocations;
            mStats.retainedBytes += slots * stride;
            return true;
        };
        // Under memory pressure, speculative capacity must not reject a batch
        // whose exact deficit still fits. Failed attempts preserve live blocks.
        return allocateBlock(growth) || (growth != deficit && allocateBlock(deficit));
    }

    IDescriptorAllocator& mUpstream;
    mutable std::mutex mMutex;
    DescriptorBatchStats mStats;
    std::vector<std::unique_ptr<Block>> mBlocks;
};

// Construct the full output range before dispatch. Neither reserve() on a
// vector nor reserving raw pool capacity alone makes writable T objects.
template <typename T, typename Base = T>
std::vector<DescPtr<Base>> allocateDescriptorBatch(DescriptorBatchAllocator& allocator, size_t count)
{
    constexpr size_t alignment = std::max(alignof(T), kDefaultDescriptorAlignment);
    std::vector<DescPtr<Base>> result(count);
    std::vector<void*> slots(count);
    if (!allocator.allocateSlots(sizeof(T), alignment, slots.data(), count))
        throw std::bad_alloc();
    size_t constructed = 0;
    try
    {
        for (; constructed < count; ++constructed)
            result[constructed] = adoptDesc<T, Base>(allocator, new (slots[constructed]) T());
    }
    catch (...)
    {
        for (size_t i = constructed; i < count; ++i)
            allocator.deallocate(slots[i], sizeof(T), alignment);
        throw; // constructed owners unwind outside the allocator lock
    }
    return result;
}

} // namespace omni::physics::parse
