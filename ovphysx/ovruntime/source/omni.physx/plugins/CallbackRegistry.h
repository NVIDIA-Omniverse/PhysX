// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-4
#pragma once
#include <functional>
#include <vector>
#include <omni/physx/EventSubscriptionRegistry.h>
#include <atomic>
#include <mutex>
#include <carb/tasking/TaskingUtils.h>

namespace omni::physx
{
template <class Entry>
class CallbackRegistry
{
public:
    using EventMap = std::map<SubscriptionId, Entry>;
    SubscriptionId addEvent(const Entry& entry)
    {
        // Static per callback family, so IDs cannot alias across context lifetimes.
        static std::atomic<SubscriptionId> next{ 0 };
        SubscriptionId id = next.load(std::memory_order_relaxed);
        while (id < kInvalidSubscriptionId)
        {
            if (next.compare_exchange_weak(id, id + 1, std::memory_order_relaxed))
            {
                std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
                mEvents.emplace(id, entry);
                return id;
            }
        }
        return kInvalidSubscriptionId;
    }
    void removeEvent(SubscriptionId id)
    {
        typename EventMap::node_type retired;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
            retired = mEvents.extract(id);
        }
        // Captured RAII objects may themselves cancel another subscription.
        // Destroy their callable storage only after releasing the registry lock.
    }
    void clear()
    {
        EventMap retired;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
            retired.swap(mEvents);
        }
    }
    EventMap snapshot() const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
        return mEvents;
    }
    bool contains(SubscriptionId id) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
        return mEvents.find(id) != mEvents.end();
    }
    template <class Predicate>
    void removeIf(Predicate predicate)
    {
        EventMap retired;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
            for (auto it = mEvents.begin(); it != mEvents.end();)
            {
                if (predicate(it->second))
                    retired.insert(mEvents.extract(it++));
                else
                    ++it;
            }
        }
    }
private:
    // Short container operations only: callbacks and captured destructors run unlocked.
    // Registries are constructed and destroyed within the runtime/tasking lifetime.
    mutable carb::tasking::MutexWrapper mMutex;
    EventMap mEvents;
};
}
