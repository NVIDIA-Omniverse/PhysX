// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "Trigger.h"
#include "Setup.h"
#include "OmniPhysX.h"

#include "internal/InternalScene.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physics/parse/IPhysicsDataWrite.h>

#include <carb/logging/Log.h>
#include <carb/tokens/ITokens.h>
#include <carb/tokens/TokensUtils.h>
#include <carb/InterfaceUtils.h>

#include <iostream>

using namespace physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

namespace omni
{
namespace physx
{

TriggerManager::TriggerManager()
{
    carb::Framework* framework = carb::getFramework();
}

void TriggerManager::release()
{
    clearTriggers();
    mTriggerSubscriptions.clear();
    mTriggerSubscriptionsMap.clear();
}

void TriggerManager::onTriggerEnterEvent(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType triggerType = ePTRemoved;
    const InternalDatabase::Record* triggerRecord = db.getFullRecord(triggerType, (ObjectId)triggerShape->userData);

    PhysXType otherType = ePTRemoved;
    const InternalDatabase::Record* otherRecord = db.getFullRecord(otherType, (ObjectId)otherShape->userData);

    if (!(triggerRecord && (triggerType == ePTShape || triggerType == ePTCompoundShape)))
        return;
    if (!(otherRecord && (otherType == ePTShape || otherType == ePTCompoundShape)))
        return;
    
    processTriggerStateApiEnterEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, otherShape);
    if (triggerShape->getActor() && otherShape->getActor()
        && (size_t)triggerShape->getActor()->userData < db.getRecords().size()
        && (size_t)otherShape->getActor()->userData < db.getRecords().size())
    {
        const InternalDatabase::Record& triggerBodyRecord = db.getRecords()[(size_t)triggerShape->getActor()->userData];
        const InternalDatabase::Record& otherBodyRecord = db.getRecords()[(size_t)otherShape->getActor()->userData];
        processNativeEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, TriggerEventType::eTRIGGER_ON_ENTER,
            triggerBodyRecord.mKey, otherBodyRecord.mKey);
    }
}

void TriggerManager::processTriggerStateApiEnterEvent(const usdparser::AttachedStage& attachedStage,
                                                      omni::physics::parse::ObjectKey triggerKey,
                                                      omni::physics::parse::ObjectKey otherKey,
                                                      const ::physx::PxShape* otherShape)
{
    (void)attachedStage;
    TriggerUsdOutputMap::iterator fit = mTriggerOutputMap.find(triggerKey);
    if (fit != mTriggerOutputMap.end())
    {
        UsdOutput& usdOutput = fit->second;
        usdOutput.dirty = true;
        TriggerCollisionMap::const_iterator fit = usdOutput.triggerCollisionmap.find(otherKey);
        if (fit == usdOutput.triggerCollisionmap.end())
        {
            usdOutput.triggerCollisionmap.insert(std::make_pair(otherKey, std::make_pair(otherShape, usdOutput.triggeredCollisions.size())));
            usdOutput.triggeredCollisions.push_back(otherKey);
        }
        else
        {
            // another one for the compound shape
            usdOutput.triggerCollisionmap.insert(std::make_pair(otherKey, std::make_pair(otherShape, fit->second.second)));
        }
    }
}

void TriggerManager::onTriggerLeaveEvent(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType triggerType = ePTRemoved;
    const InternalDatabase::Record* triggerRecord = db.getFullRecord(triggerType, (ObjectId)triggerShape->userData);

    PhysXType otherType = ePTRemoved;
    const InternalDatabase::Record* otherRecord = db.getFullRecord(otherType, (ObjectId)otherShape->userData);

    if (!(triggerRecord && (triggerType == ePTShape || triggerType == ePTCompoundShape)))
        return;
    if (!(otherRecord && (otherType == ePTShape || otherType == ePTCompoundShape || otherType == ePTRemoved)))
        return;
    
    processTriggerStateApiLeaveEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, otherShape);

    PhysXType triggerBodyType = ePTRemoved;
    const InternalDatabase::Record* triggerBodyRecord =
        db.getFullRecord(triggerBodyType, (ObjectId)triggerShape->getActor()->userData);
    if (!(triggerBodyRecord && (triggerBodyType == ePTActor)))
        return;
    const InternalDatabase::Record* otherBodyRecord = nullptr;
    PhysXType otherBodyType = ePTRemoved;
    if (otherType != ePTRemoved)
    {
        otherBodyRecord = db.getFullRecord(otherBodyType, (ObjectId)otherShape->getActor()->userData);
        if (!(otherBodyRecord && (otherBodyType == ePTActor)))
            return;
    }
    processNativeEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, TriggerEventType::eTRIGGER_ON_LEAVE,
        triggerBodyRecord->mKey, otherBodyRecord ? otherBodyRecord->mKey : omni::physics::parse::ObjectKey{});
}

void TriggerManager::processTriggerStateApiLeaveEvent(const usdparser::AttachedStage& attachedStage,
                                                      omni::physics::parse::ObjectKey triggerKey,
                                                      omni::physics::parse::ObjectKey otherKey,
                                                      const ::physx::PxShape* otherShape)
{
    (void)attachedStage;
    TriggerUsdOutputMap::iterator fit = mTriggerOutputMap.find(triggerKey);
    if (fit != mTriggerOutputMap.end())
    {
        UsdOutput& usdOutput = fit->second;
        usdOutput.dirty = true;
        TriggerCollisionMap::iterator cf = usdOutput.triggerCollisionmap.find(otherKey);
        size_t pathIndex = 0;
        bool indexFound = false;
        while (cf != usdOutput.triggerCollisionmap.end() && cf->first == otherKey)
        {
            if (cf->second.first == otherShape)
            {
                pathIndex = cf->second.second;
                indexFound = true;
                usdOutput.triggerCollisionmap.erase(cf);
                break;
            }
            cf++;
        }

        // check if all shapes are gone (can be same path for convex decomposion)
        if (indexFound)
        {
            cf = usdOutput.triggerCollisionmap.find(otherKey);
            if (cf == usdOutput.triggerCollisionmap.end())
            {
                usdOutput.triggeredCollisions[pathIndex] = usdOutput.triggeredCollisions.back();
                usdOutput.triggeredCollisions.pop_back();
                if (!usdOutput.triggeredCollisions.empty())
                {
                    // The moved-in entry IS the ObjectKey it was filed under in
                    // triggerCollisionmap -- no path round-trip needed to find it.
                    const omni::physics::parse::ObjectKey movedKey = usdOutput.triggeredCollisions[pathIndex];
                    TriggerCollisionMap::iterator cfb = usdOutput.triggerCollisionmap.find(movedKey);
                    while (cfb != usdOutput.triggerCollisionmap.end() && cfb->first == movedKey)
                    {
                        cfb->second.second = pathIndex;
                        cfb++;
                    }
                }
            }
        }
    }
}

void TriggerManager::bufferTriggerEvent(const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape, TriggerEventType::Enum triggerEvent)
{
    mInvokedTriggers.push_back({ triggerShape, otherShape, triggerEvent });
}

void TriggerManager::clearBufferedShape(const ::physx::PxShape* shape)
{
    for (size_t i = mInvokedTriggers.size(); i-- ;)
    {
        const InvokedTrigger& tr = mInvokedTriggers[i];
        if (tr.mTriggerShape == shape || tr.mOtherShape == shape)
        {
            mInvokedTriggers[i] = mInvokedTriggers.back();
            mInvokedTriggers.pop_back();
        }
    }
}

void TriggerManager::fireTriggerEvents(const usdparser::AttachedStage& attachedStage)
{
    for (size_t i = 0; i < mInvokedTriggers.size(); i++)
    {
        const InvokedTrigger& trigger = mInvokedTriggers[i];
        if (trigger.mTriggerEvent == TriggerEventType::eTRIGGER_ON_ENTER)
        {
            onTriggerEnterEvent(attachedStage, trigger.mTriggerShape, trigger.mOtherShape);
        }
        else if(trigger.mTriggerEvent == TriggerEventType::eTRIGGER_ON_LEAVE)
        {
            onTriggerLeaveEvent(attachedStage, trigger.mTriggerShape, trigger.mOtherShape);
        }
    }
    mInvokedTriggers.clear();

    // attachedStage is `const AttachedStage&` here (PhysXScene::mAttachedStage, its only
    // caller, holds a const ref throughout PhysXScene's lifetime); getDataWrite() publishes
    // engine output without touching AttachedStage's own attachment identity, so the const
    // is stripped from the accessor's result, not from a genuinely const object (the
    // underlying IPhysicsDataWrite is heap-owned by AttachedStage's non-const mDataWrite;
    // the const-returning overload exists only so a const-context caller can null-check it).
    omni::physics::parse::IPhysicsDataWrite* dataWrite =
        const_cast<omni::physics::parse::IPhysicsDataWrite*>(attachedStage.getDataWrite());
    for (TriggerUsdOutputMap::reference ref : mTriggerOutputMap)
    {
        UsdOutput& usdOutput = ref.second;
        if (usdOutput.dirty)
        {
            if (usdOutput.eligible && dataWrite)
            {
                dataWrite->writeTriggerCollisions(
                    ref.first, usdOutput.triggeredCollisions.data(), usdOutput.triggeredCollisions.size());
            }
            usdOutput.dirty = false;
        }
    }
}


void TriggerManager::preloadTrigger(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey triggerKey, bool usdOutput)
{
    if (!usdOutput)
        return;

    UsdOutput usdOutputEntry;
    usdOutputEntry.dirty = false;
    // See fireTriggerEvents' comment on the same const_cast (attachedStage is a const ref
    // here too, from the same PhysXScene::mAttachedStage caller).
    if (omni::physics::parse::IPhysicsDataWrite* dataWrite =
            const_cast<omni::physics::parse::IPhysicsDataWrite*>(attachedStage.getDataWrite()))
        dataWrite->prepareTriggerWrite(&triggerKey, 1, &usdOutputEntry.eligible);
    mTriggerOutputMap[triggerKey] = usdOutputEntry;
}

void TriggerManager::clearTriggers()
{
    mInvokedTriggers.clear();

    // No AttachedStage in scope at either call site (TriggerManager::release() runs at
    // full plugin shutdown, after the active attach is already gone; releaseAllObjects()
    // has no attach context of its own) -- resolve the currently active one fresh here,
    // same idiom as UsdInterface.cpp's textForActiveStage(). Null (no active attach)
    // means there is nothing to author into; the map is dropped regardless.
    usdparser::AttachedStage* activeStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    omni::physics::parse::IPhysicsDataWrite* dataWrite = activeStage ? activeStage->getDataWrite() : nullptr;
    if (dataWrite)
    {
        for (TriggerUsdOutputMap::reference ref : mTriggerOutputMap)
        {
            if (ref.second.eligible)
                dataWrite->releaseTriggerWrite(&ref.first, 1);
        }
    }

    mTriggerOutputMap.clear();
}

omni::physx::SubscriptionId TriggerManager::registerNativeCallback(TriggerSubscriptionEntry triggerSubscription)
{
    omni::physx::SubscriptionId subId = mTriggerSubscriptions.addEvent(triggerSubscription);
    mTriggerSubscriptionsMap.insert(std::make_pair(triggerSubscription.triggerColliderPrimKey, subId));
    return subId;
}

void TriggerManager::unregisterNativeCallback(omni::physx::SubscriptionId subscriptionID)
{
    auto it = mTriggerSubscriptions.map.find(subscriptionID);
    if(it == mTriggerSubscriptions.map.end())
        return;
    mTriggerSubscriptions.removeEvent(subscriptionID);
    for (auto it = mTriggerSubscriptionsMap.begin(); it != mTriggerSubscriptionsMap.end(); ++it)
    {
        if (it->second == subscriptionID)
        {
            mTriggerSubscriptionsMap.erase(it);
            break;
        }
    }
}

// Does `subscription` want events from the attach that just fired?
//
// A concrete handle names one attach and is compared literally, so a subscription dies with the
// attach it named rather than silently following the next one (ADR-0013's staleness rule).
// kActiveAttach is the deliberate exception: it is stored unresolved by
// subscribePhysicsTriggerReportEvents so that subscribing *before* anything is attached works,
// and it means "whichever attach is live" -- so it matches any firing attach, and follows a
// detach/reattach. That is the capability the removed `stageId == 0` wildcard used to provide by
// accident; it is not a wildcard over *several* attaches, because the runtime holds one at a time.
static bool subscriptionMatches(const TriggerSubscriptionEntry& subscription, AttachHandle attachHandle)
{
    return subscription.attachHandle == kActiveAttach || subscription.attachHandle == attachHandle;
}

void TriggerManager::processNativeEvent(const usdparser::AttachedStage& attachedStage,
                                        omni::physics::parse::ObjectKey triggerColliderKey,
                                        omni::physics::parse::ObjectKey otherColliderKey,
                                        TriggerEventType::Enum eventType,
                                        omni::physics::parse::ObjectKey triggerBodyKey,
                                        omni::physics::parse::ObjectKey otherBodyKey)
{
    const AttachHandle attachHandle = attachedStage.getAttachHandle();
    TriggerEventData triggerData;
    triggerData.attachHandle = attachHandle;
    triggerData.triggerColliderPrimKey = triggerColliderKey;
    triggerData.otherColliderPrimKey = otherColliderKey;
    triggerData.triggerBodyPrimKey = triggerBodyKey;
    triggerData.otherBodyPrimKey = otherBodyKey;
    triggerData.eventType = eventType;

    // 1. Report the trigger pair for all subscription that have explicitly been watching this specific path
    // Need to use equal_range instead of find to get all subscriptions for a given identity (potentially > 1)
    auto triggerColliderSubscriptions = mTriggerSubscriptionsMap.equal_range(triggerColliderKey);
    for (auto it = triggerColliderSubscriptions.first; it != triggerColliderSubscriptions.second; ++it)
    {
        omni::physx::SubscriptionId subId = it->second;
        const TriggerSubscriptionEntry& subscription = mTriggerSubscriptions.map.at(subId);
        // The subscription names one attach: the "report from every stage" wildcard is gone
        // (ADR-0016 Decision 6). kActiveAttach is bound late here rather than at subscribe
        // time, so subscribing before anything is attached keeps working.
        if (subscriptionMatches(subscription, attachHandle))
        {
            triggerData.subscriptionId = subId;
            subscription.reportFn(&triggerData, subscription.userData);
        }
    }

    // 2. If body and collider are not the same object, check if we have listeners listening for the body
    if(triggerColliderKey != triggerBodyKey)
    {
        auto triggerBodySubscription = mTriggerSubscriptionsMap.equal_range(triggerBodyKey);
        for (auto it = triggerBodySubscription.first; it != triggerBodySubscription.second; ++it)
        {
            omni::physx::SubscriptionId subId = it->second;
            const TriggerSubscriptionEntry& subscription = mTriggerSubscriptions.map.at(subId);
            // Same late-bound match as above.
            if (subscriptionMatches(subscription, attachHandle))
            {
                triggerData.subscriptionId = subId;
                subscription.reportFn(&triggerData, subscription.userData);
            }
        }
    }

    // 3. Report the trigger for all subs that have not indicated a specific path to listen for
    auto allSubscriptionForEmptyPath = mTriggerSubscriptionsMap.equal_range(omni::physics::parse::ObjectKey{});
    for (auto it = allSubscriptionForEmptyPath.first; it != allSubscriptionForEmptyPath.second; ++it)
    {
        omni::physx::SubscriptionId subId = it->second;
        const TriggerSubscriptionEntry& subscription = mTriggerSubscriptions.map.at(subId);
        // The subscription names one attach: the "report from every stage" wildcard is gone
        // (ADR-0016 Decision 6). kActiveAttach is bound late here rather than at subscribe
        // time, so subscribing before anything is attached keeps working.
        if (subscriptionMatches(subscription, attachHandle))
        {
            triggerData.subscriptionId = subId;
            subscription.reportFn(&triggerData, subscription.userData);
        }
    }
}

} // namespace physx

} // namespace omni
