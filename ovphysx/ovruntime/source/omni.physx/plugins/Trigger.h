// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <PxPhysicsAPI.h>
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
    AttachHandle attachHandle; //!< The attach whose trigger events this subscription wants. May be
                               //!< kActiveAttach, stored unresolved: it is bound late, at fire time,
                               //!< when the reported handle is compared against this field, not
                               //!< resolved when the subscription is made (ADR-0016 Decision 6)
    omni::physics::parse::ObjectKey triggerColliderPrimKey; //!< The prim source of trigger event
    omni::physx::OnTriggerEventReportEventFn reportFn; //!< reporting function
    void* userData; //!< User Data passed to reporting function
};

using TriggerSubscriptionsMap = std::unordered_multimap<omni::physics::parse::ObjectKey,
                                                         omni::physx::SubscriptionId,
                                                         omni::physics::parse::ObjectKey::Hash>;
using TriggerSubscriptionRegistry = EventSubscriptionRegistry<TriggerSubscriptionEntry>;

class TriggerManager
{
public:
    TriggerManager();

    void release();

    void onTriggerEnterEvent(const usdparser::AttachedStage& attachedStage,
                             const ::physx::PxShape* triggerShape,
                             const ::physx::PxShape* otherShape);
    void onTriggerLeaveEvent(const usdparser::AttachedStage& attachedStage,
                             const ::physx::PxShape* triggerShape,
                             const ::physx::PxShape* otherShape);

    void bufferTriggerEvent(const ::physx::PxShape* triggerShape,
                            const ::physx::PxShape* otherShape,
                            TriggerEventType::Enum triggerEvent);
    void fireTriggerEvents(const usdparser::AttachedStage& attachedStage);

    // `usdOutput` requests the trigger-state write-back above; when the attach
    // has no write sink it is a no-op, since there is
    // nothing to author into (mirrors the "USD-only trigger state write-back;
    // there is nothing to author without a prim" call-site comment in
    // UsdInterface.cpp's createShape).
    void preloadTrigger(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey triggerKey, bool usdOutput);
    void clearTriggers();
    
    void clearBufferedShape(const ::physx::PxShape* shape);

    omni::physx::SubscriptionId registerNativeCallback(TriggerSubscriptionEntry triggerSubscription);
    void unregisterNativeCallback(omni::physx::SubscriptionId subscriptionID);

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
                            omni::physics::parse::ObjectKey otherBodyKey);

private:
    InvokedTriggers mInvokedTriggers;

    TriggerUsdOutputMap mTriggerOutputMap;

    TriggerSubscriptionRegistry mTriggerSubscriptions;
    TriggerSubscriptionsMap mTriggerSubscriptionsMap;
};

} // namespace physx
} // namespace omni
