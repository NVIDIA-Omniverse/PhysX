// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-4

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2 AC-4

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4

/**
 * @implements REQ-ATTACH-RESOLVE-001
 * @covers AC-12
 */

#pragma once
#include <carb/tasking/TaskingUtils.h>

#include <PxPhysicsAPI.h>
#include "CallbackRegistry.h"
#include <omni/physx/TriggerEvent.h>
#include <omni/physics/parse/Handles.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}

struct InvokedTrigger
{
    const ::physx::PxShape* mTriggerShape;
    const ::physx::PxShape* mOtherShape;
    TriggerEventType::Enum mTriggerEvent;
    uint64_t mTriggerActorId = 0, mOtherActorId = 0;
};

using InvokedTriggers = std::vector<InvokedTrigger>;
using TriggerCollisionMap = std::unordered_multimap<omni::physics::parse::ObjectKey,
                                                     std::pair<const ::physx::PxShape*, size_t>,
                                                     omni::physics::parse::ObjectKey::Hash>;

// Trigger-state write-back bookkeeping (PhysxTriggerStateAPI's triggeredCollisions
// relationship on a USD backend, authored for viewport/inspector consumption): a
// real, permanent Kit-authoring boundary, not reachable from ovphysx's own ovstage
// attach path. The bookkeeping here is
// backend-agnostic (ObjectKey only); the actual relationship write happens through
// IPhysicsDataWrite::writeTriggerCollisions/prepareTriggerWrite/releaseTriggerWrite
// (see Trigger.cpp), which is a real no-op when there is no write sink (no backing
// stage).
struct UsdOutput
{
    bool eligible = false; //!< set by prepareTriggerWrite: does the trigger have a write-back destination
    TriggerCollisionMap triggerCollisionmap;
    std::vector<omni::physics::parse::ObjectKey> triggeredCollisions;
    bool dirty = false;
};

using TriggerUsdOutputMap =
    std::unordered_map<omni::physics::parse::ObjectKey, UsdOutput, omni::physics::parse::ObjectKey::Hash>;

struct TriggerSubscriptionEntry
{
    AttachHandle attachHandle; //!< Immutable exact attachment identity.
    omni::physics::parse::ObjectKey triggerColliderPrimKey; //!< The prim source of trigger event
    std::function<void(const TriggerEventData*, void*)> reportFn; //!< reporting function
    void* userData; //!< User Data passed to reporting function
    // Internal copied-data consumer; legacy callback layout remains unchanged.
    std::function<void(const TriggerEventData*, uint32_t, uint32_t)> detailedReport;
};

class TriggerSubscriptionRegistry
{
public:
    using EventMap = std::vector<std::pair<SubscriptionId, TriggerSubscriptionEntry>>;
    using Key = std::pair<AttachHandle, uint64_t>;
    SubscriptionId addEvent(const TriggerSubscriptionEntry& entry)
    {
        SubscriptionId id = mNext.load(std::memory_order_relaxed);
        for (;;)
        {
            if (id == kInvalidSubscriptionId)
                return kInvalidSubscriptionId;
            if (mNext.compare_exchange_weak(id, id + 1, std::memory_order_relaxed))
                break;
        }
        std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
        auto position = mByKey.emplace(Key{entry.attachHandle, entry.triggerColliderPrimKey.handle},
                                      std::make_pair(id, entry));
        mById.emplace(id, position);
        return id;
    }
    void removeEvent(SubscriptionId id)
    {
        decltype(mByKey)::node_type retired;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
            auto it = mById.find(id);
            if (it != mById.end())
            {
                retired = mByKey.extract(it->second);
                mById.erase(it);
            }
        }
    }
    bool contains(SubscriptionId id) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
        return mById.count(id) != 0;
    }
    EventMap snapshot(AttachHandle attach, const omni::physics::parse::ObjectKey* keys, size_t count) const
    {
        EventMap selected;
        std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
        for (size_t i = 0; i < count; ++i)
        {
            if (i == 1 && keys[i] == keys[0])
                continue;
            if (i + 1 != count && !keys[i].handle)
                continue;
            const auto range = mByKey.equal_range({attach, keys[i].handle});
            for (auto it = range.first; it != range.second; ++it)
                selected.push_back(it->second);
        }
        return selected;
    }
    template<class Predicate> void removeIf(Predicate predicate)
    {
        std::vector<decltype(mByKey)::node_type> retired;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mMutex);
            for (auto it = mByKey.begin(); it != mByKey.end();)
                if (predicate(it->second.second))
                {
                    mById.erase(it->second.first);
                    retired.push_back(mByKey.extract(it++));
                }
                else
                    ++it;
        }
    }
    void clear() { removeIf([](const auto&) { return true; }); }
private:
    inline static std::atomic<SubscriptionId> mNext{0};
    mutable carb::tasking::MutexWrapper mMutex;
    std::multimap<Key, std::pair<SubscriptionId, TriggerSubscriptionEntry>> mByKey;
    std::unordered_map<SubscriptionId, decltype(mByKey)::iterator> mById;
};

struct AttachmentTriggerState
{
    InvokedTriggers invoked;
    TriggerUsdOutputMap output;
};

class TriggerManager
{
public:
    TriggerManager();

    void release();

    void onTriggerEnterEvent(const usdparser::AttachedStage& attachedStage,
                             const ::physx::PxShape* triggerShape,
                             const ::physx::PxShape* otherShape,
                             uint64_t triggerActorId = 0, uint64_t otherActorId = 0);
    void onTriggerLeaveEvent(const usdparser::AttachedStage& attachedStage,
                             const ::physx::PxShape* triggerShape,
                             const ::physx::PxShape* otherShape,
                             uint64_t triggerActorId = 0, uint64_t otherActorId = 0);

    void bufferTriggerEvent(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* triggerShape,
                            const ::physx::PxShape* otherShape,
                            TriggerEventType::Enum triggerEvent,
                            uint64_t triggerActorId = 0, uint64_t otherActorId = 0);
    void fireTriggerEvents(const usdparser::AttachedStage& attachedStage);

    // `usdOutput` requests the trigger-state write-back above; when the attach
    // has no write sink it is a no-op, since there is
    // nothing to author into (mirrors the "USD-only trigger state write-back;
    // there is nothing to author without a prim" call-site comment in
    // UsdInterface.cpp's createShape).
    void preloadTrigger(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey triggerKey, bool usdOutput);
    void clearTriggers(const usdparser::AttachedStage& attachedStage);
    
    void clearBufferedShape(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* shape);

    omni::physx::SubscriptionId registerNativeCallback(TriggerSubscriptionEntry triggerSubscription);
    void unregisterNativeCallback(omni::physx::SubscriptionId subscriptionID);
    void retireSubscriptions(AttachHandle attach);

private:
    // Still take attachedStage: it resolves the write sink (IPhysicsDataWrite) the
    // relationship write eventually goes through, which is a per-attach object, not
    // something ObjectKey alone can reach. The bookkeeping (mTriggerOutputMap) itself
    // is pxr-free; it is a real no-op when the attach has no write sink (no backing stage).
    void processTriggerStateApiEnterEvent(const usdparser::AttachedStage& attachedStage,
                                          omni::physics::parse::ObjectKey triggerKey,
                                          omni::physics::parse::ObjectKey otherKey,
                                          const ::physx::PxShape* otherShape);
    void processTriggerStateApiLeaveEvent(const usdparser::AttachedStage& attachedStage,
                                          omni::physics::parse::ObjectKey triggerKey,
                                          omni::physics::parse::ObjectKey otherKey,
                                          const ::physx::PxShape* otherShape);
    void processNativeEvent(const usdparser::AttachedStage& attachedStage,
                            omni::physics::parse::ObjectKey triggerColliderKey,
                            omni::physics::parse::ObjectKey otherColliderKey,
                            TriggerEventType::Enum eventType,
                            omni::physics::parse::ObjectKey triggerBodyKey,
                            omni::physics::parse::ObjectKey otherBodyKey,
                            uint32_t triggerInstance, uint32_t otherInstance);

private:
    TriggerSubscriptionRegistry mTriggerSubscriptions;
};

} // namespace physx
} // namespace omni
