// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/scripting/IScripting.h>
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

struct TriggerScript
{
    TriggerScript() : mScript(nullptr), mScriptPath(nullptr)
    {
    }

    TriggerScript(carb::scripting::Script* sc, const char* path) : mScript(sc), mScriptPath(path)
    {
    }

    carb::scripting::Script* mScript;
    const char* mScriptPath;
};


struct InvokedTrigger
{
    const ::physx::PxShape* mTriggerShape;
    const ::physx::PxShape* mOtherShape;
    TriggerEventType::Enum mTriggerEvent;
};

using TriggerScriptMap = std::map<pxr::TfToken, std::pair<std::string, carb::scripting::Script*>>;
using InvokedTriggers = std::vector<InvokedTrigger>;
using TriggerCollisionMap =
    std::unordered_multimap<pxr::SdfPath, std::pair<const ::physx::PxShape*, size_t>, pxr::SdfPath::Hash>;

struct UsdOutput
{
    pxr::PhysxSchemaPhysxTriggerStateAPI triggerStateAPI;
    TriggerCollisionMap triggerCollisionmap;
    pxr::SdfPathVector triggeredCollisions;
    bool dirty;
};

using TriggerUsdOutputMap = std::unordered_map<pxr::SdfPath, UsdOutput, pxr::SdfPath::Hash>;

struct TriggerSubscriptionEntry
{
    uint64_t stageId; //!< The stage where trigger event happend
    uint64_t triggerColliderPrimId; //!< The prim source of trigger event
    omni::physx::OnTriggerEventReportEventFn reportFn; //!< reporting function
    void* userData; //!< User Data passed to reporting function
};

using TriggerSubscriptionsMap = std::unordered_multimap<pxr::SdfPath, omni::physx::SubscriptionId, pxr::SdfPath::Hash>;
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

    void preloadTrigger(const pxr::UsdPrim& triggerPrim, bool usdOutput);
    void clearTriggers();
    
    void clearBufferedShape(const ::physx::PxShape* shape);

    omni::physx::SubscriptionId registerNativeCallback(TriggerSubscriptionEntry triggerSubscription);
    void unregisterNativeCallback(omni::physx::SubscriptionId subscriptionID);

private:
    void processTriggerStateApiEnterEvent(const pxr::SdfPath& triggerPath,
                                          const pxr::SdfPath& otherPath,
                                          const ::physx::PxShape* otherShape);
    void processTriggerApiEnterEvent(const usdparser::AttachedStage& attachedStage,
                                     const pxr::SdfPath& triggerPath,
                                     const pxr::SdfPath& otherPath);
    void processTriggerStateApiLeaveEvent(const pxr::SdfPath& triggerPath,
                                          const pxr::SdfPath& otherPath,
                                          const ::physx::PxShape* otherShape);
    void processTriggerApiLeaveEvent(const usdparser::AttachedStage& attachedStage,
                                     const pxr::SdfPath& triggerPath,
                                     const pxr::SdfPath& otherPath);
    void processNativeEvent(const usdparser::AttachedStage& attachedStage,
                            const pxr::SdfPath& triggerColliderPath,
                            const pxr::SdfPath& otherColliderPath,
                            TriggerEventType::Enum eventType,
                            const pxr::SdfPath& triggerBodyPath,
                            const pxr::SdfPath& otherBodyPath);
    TriggerScript getTriggerScript(const pxr::TfToken& scriptName);
    TriggerScript addTriggerScript(const pxr::TfToken& scriptName, const char* scriptPathName);

private:
    carb::scripting::IScripting* mScripting;
    carb::scripting::Context* mContextPython;

    TriggerScriptMap mTriggerScriptMap;
    InvokedTriggers mInvokedTriggers;

    TriggerUsdOutputMap mTriggerOutputMap;

    TriggerSubscriptionRegistry mTriggerSubscriptions;
    TriggerSubscriptionsMap mTriggerSubscriptionsMap;
};

} // namespace physx
} // namespace omni
