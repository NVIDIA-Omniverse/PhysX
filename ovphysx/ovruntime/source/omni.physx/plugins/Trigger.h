// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/TriggerEvent.h>

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
using TriggerCollisionMap =
    std::unordered_multimap<PXR_NS::SdfPath, std::pair<const ::physx::PxShape*, size_t>, PXR_NS::SdfPath::Hash>;

struct UsdOutput
{
    PXR_NS::PhysxSchemaPhysxTriggerStateAPI triggerStateAPI;
    TriggerCollisionMap triggerCollisionmap;
    PXR_NS::SdfPathVector triggeredCollisions;
    bool dirty;
};

using TriggerUsdOutputMap = std::unordered_map<PXR_NS::SdfPath, UsdOutput, PXR_NS::SdfPath::Hash>;

struct TriggerSubscriptionEntry
{
    uint64_t stageId; //!< The stage where trigger event happend
    uint64_t triggerColliderPrimId; //!< The prim source of trigger event
    omni::physx::OnTriggerEventReportEventFn reportFn; //!< reporting function
    void* userData; //!< User Data passed to reporting function
};

using TriggerSubscriptionsMap = std::unordered_multimap<PXR_NS::SdfPath, omni::physx::SubscriptionId, PXR_NS::SdfPath::Hash>;
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

    void preloadTrigger(const PXR_NS::UsdPrim& triggerPrim, bool usdOutput);
    void clearTriggers();
    
    void clearBufferedShape(const ::physx::PxShape* shape);

    omni::physx::SubscriptionId registerNativeCallback(TriggerSubscriptionEntry triggerSubscription);
    void unregisterNativeCallback(omni::physx::SubscriptionId subscriptionID);

private:
    void processTriggerStateApiEnterEvent(const PXR_NS::SdfPath& triggerPath,
                                          const PXR_NS::SdfPath& otherPath,
                                          const ::physx::PxShape* otherShape);
    void processTriggerStateApiLeaveEvent(const PXR_NS::SdfPath& triggerPath,
                                          const PXR_NS::SdfPath& otherPath,
                                          const ::physx::PxShape* otherShape);
    void processNativeEvent(const usdparser::AttachedStage& attachedStage,
                            const PXR_NS::SdfPath& triggerColliderPath,
                            const PXR_NS::SdfPath& otherColliderPath,
                            TriggerEventType::Enum eventType,
                            const PXR_NS::SdfPath& triggerBodyPath,
                            const PXR_NS::SdfPath& otherBodyPath);

private:
    InvokedTriggers mInvokedTriggers;

    TriggerUsdOutputMap mTriggerOutputMap;

    TriggerSubscriptionRegistry mTriggerSubscriptions;
    TriggerSubscriptionsMap mTriggerSubscriptionsMap;
};

} // namespace physx
} // namespace omni
