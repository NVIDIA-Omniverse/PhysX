// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-2 AC-4

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2 AC-4

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4

/**
 * @implements REQ-ATTACH-RESOLVE-001
 * @covers AC-12
 */

#include "Trigger.h"
#include "Setup.h"
#include "OmniPhysX.h"

#include "internal/InternalScene.h"
#include "internal/InternalActor.h"
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

namespace
{
uint32_t triggerInstanceIndex(const InternalDatabase::Record* record)
{
    return record && (record->mType == ePTActor || record->mType == ePTLink) && record->mInternalPtr ?
        static_cast<const InternalActor*>(record->mInternalPtr)->mInstanceIndex : UINT32_MAX;
}
}

TriggerManager::TriggerManager()
{
    carb::Framework* framework = carb::getFramework();
}

void TriggerManager::release()
{
    mTriggerSubscriptions.clear();
}

void TriggerManager::onTriggerEnterEvent(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape, uint64_t triggerActorId, uint64_t otherActorId)
{
    const InternalPhysXDatabase& db = attachedStage.getInternalPhysXDatabase();

    PhysXType triggerType = ePTRemoved;
    const InternalDatabase::Record* triggerRecord = db.getFullRecord(triggerType, (ObjectId)triggerShape->userData);

    PhysXType otherType = ePTRemoved;
    const InternalDatabase::Record* otherRecord = db.getFullRecord(otherType, (ObjectId)otherShape->userData);

    if (!(triggerRecord && (triggerType == ePTShape || triggerType == ePTCompoundShape)))
        return;
    if (!(otherRecord && (otherType == ePTShape || otherType == ePTCompoundShape)))
        return;
    
    processTriggerStateApiEnterEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, otherShape);
    // Shared point-instancer shapes have no unique getActor(). Use the actor
    // record identities captured from PxTriggerPair at callback production.
    if (!triggerActorId && triggerShape->getActor()) triggerActorId = uint64_t(triggerShape->getActor()->userData);
    if (!otherActorId && otherShape->getActor()) otherActorId = uint64_t(otherShape->getActor()->userData);
    if (triggerActorId && otherActorId && triggerActorId < db.getRecords().size() && otherActorId < db.getRecords().size())
    {
        const omni::physx::internal::InternalDatabase::Record & triggerBodyRecord = db.getRecords()[triggerActorId];
        const omni::physx::internal::InternalDatabase::Record & otherBodyRecord = db.getRecords()[otherActorId];
        processNativeEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, TriggerEventType::eTRIGGER_ON_ENTER,
            triggerBodyRecord.mKey, otherBodyRecord.mKey,
            triggerInstanceIndex(&triggerBodyRecord), triggerInstanceIndex(&otherBodyRecord));
    }
}

void TriggerManager::processTriggerStateApiEnterEvent(const usdparser::AttachedStage& attachedStage,
                                                      omni::physics::parse::ObjectKey triggerKey,
                                                      omni::physics::parse::ObjectKey otherKey,
                                                      const ::physx::PxShape* otherShape)
{
    (void)attachedStage;
    TriggerUsdOutputMap::iterator fit = attachedStage.getTriggerState().output.find(triggerKey);
    if (fit != attachedStage.getTriggerState().output.end())
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

void TriggerManager::onTriggerLeaveEvent(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape, uint64_t triggerActorId, uint64_t otherActorId)
{
    const InternalPhysXDatabase& db = attachedStage.getInternalPhysXDatabase();

    PhysXType triggerType = ePTRemoved;
    const InternalDatabase::Record* triggerRecord = db.getFullRecord(triggerType, (ObjectId)triggerShape->userData);

    PhysXType otherType = ePTRemoved;
    const InternalDatabase::Record* otherRecord = db.getFullRecord(otherType, (ObjectId)otherShape->userData);

    if (!(triggerRecord && (triggerType == ePTShape || triggerType == ePTCompoundShape)))
        return;
    if (!(otherRecord && (otherType == ePTShape || otherType == ePTCompoundShape || otherType == ePTRemoved)))
        return;
    
    processTriggerStateApiLeaveEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, otherShape);

    if (!triggerActorId && triggerShape->getActor()) triggerActorId = uint64_t(triggerShape->getActor()->userData);
    PhysXType triggerBodyType = ePTRemoved;
    const InternalDatabase::Record* triggerBodyRecord =
        db.getFullRecord(triggerBodyType, triggerActorId);
    if (!(triggerBodyRecord && (triggerBodyType == ePTActor || triggerBodyType == ePTLink)))
        return;
    const InternalDatabase::Record* otherBodyRecord = nullptr;
    PhysXType otherBodyType = ePTRemoved;
    if (otherType != ePTRemoved)
    {
        if (!otherActorId && otherShape->getActor()) otherActorId = uint64_t(otherShape->getActor()->userData);
        otherBodyRecord = db.getFullRecord(otherBodyType, otherActorId);
        if (!(otherBodyRecord && (otherBodyType == ePTActor || otherBodyType == ePTLink)))
            return;
    }
    processNativeEvent(attachedStage, triggerRecord->mKey, otherRecord->mKey, TriggerEventType::eTRIGGER_ON_LEAVE,
        triggerBodyRecord->mKey, otherBodyRecord ? otherBodyRecord->mKey : omni::physics::parse::ObjectKey{},
        triggerInstanceIndex(triggerBodyRecord), triggerInstanceIndex(otherBodyRecord));
}

void TriggerManager::processTriggerStateApiLeaveEvent(const usdparser::AttachedStage& attachedStage,
                                                      omni::physics::parse::ObjectKey triggerKey,
                                                      omni::physics::parse::ObjectKey otherKey,
                                                      const ::physx::PxShape* otherShape)
{
    (void)attachedStage;
    TriggerUsdOutputMap::iterator fit = attachedStage.getTriggerState().output.find(triggerKey);
    if (fit != attachedStage.getTriggerState().output.end())
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

void TriggerManager::bufferTriggerEvent(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape, TriggerEventType::Enum triggerEvent, uint64_t triggerActorId, uint64_t otherActorId)
{
    attachedStage.getTriggerState().invoked.push_back({ triggerShape, otherShape, triggerEvent, triggerActorId, otherActorId });
}

void TriggerManager::clearBufferedShape(const usdparser::AttachedStage& attachedStage, const ::physx::PxShape* shape)
{
    for (size_t i = attachedStage.getTriggerState().invoked.size(); i-- ;)
    {
        const InvokedTrigger& tr = attachedStage.getTriggerState().invoked[i];
        if (tr.mTriggerShape == shape || tr.mOtherShape == shape)
        {
            attachedStage.getTriggerState().invoked[i] = attachedStage.getTriggerState().invoked.back();
            attachedStage.getTriggerState().invoked.pop_back();
        }
    }
}

void TriggerManager::fireTriggerEvents(const usdparser::AttachedStage& attachedStage)
{
    for (size_t i = 0; i < attachedStage.getTriggerState().invoked.size(); i++)
    {
        const InvokedTrigger& trigger = attachedStage.getTriggerState().invoked[i];
        if (trigger.mTriggerEvent == TriggerEventType::eTRIGGER_ON_ENTER)
        {
            onTriggerEnterEvent(attachedStage, trigger.mTriggerShape, trigger.mOtherShape, trigger.mTriggerActorId, trigger.mOtherActorId);
        }
        else if(trigger.mTriggerEvent == TriggerEventType::eTRIGGER_ON_LEAVE)
        {
            onTriggerLeaveEvent(attachedStage, trigger.mTriggerShape, trigger.mOtherShape, trigger.mTriggerActorId, trigger.mOtherActorId);
        }
    }
    attachedStage.getTriggerState().invoked.clear();

    // The attachment owns the mutable output sink; this callback only holds a const owner reference.
    omni::physics::parse::IPhysicsDataWrite* dataWrite =
        const_cast<omni::physics::parse::IPhysicsDataWrite*>(attachedStage.getDataWrite());
    for (TriggerUsdOutputMap::reference ref : attachedStage.getTriggerState().output)
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
    if (omni::physics::parse::IPhysicsDataWrite* dataWrite =
            const_cast<omni::physics::parse::IPhysicsDataWrite*>(attachedStage.getDataWrite()))
        dataWrite->prepareTriggerWrite(&triggerKey, 1, &usdOutputEntry.eligible);
    attachedStage.getTriggerState().output[triggerKey] = usdOutputEntry;
}

void TriggerManager::clearTriggers(const usdparser::AttachedStage& attachedStage)
{
    attachedStage.getTriggerState().invoked.clear();

    omni::physics::parse::IPhysicsDataWrite* dataWrite =
        const_cast<omni::physics::parse::IPhysicsDataWrite*>(attachedStage.getDataWrite());
    if (dataWrite)
    {
        for (TriggerUsdOutputMap::reference ref : attachedStage.getTriggerState().output)
        {
            if (ref.second.eligible)
                dataWrite->releaseTriggerWrite(&ref.first, 1);
        }
    }

    attachedStage.getTriggerState().output.clear();
}

omni::physx::SubscriptionId TriggerManager::registerNativeCallback(TriggerSubscriptionEntry subscription)
{
    return mTriggerSubscriptions.addEvent(subscription);
}

void TriggerManager::unregisterNativeCallback(omni::physx::SubscriptionId id)
{
    mTriggerSubscriptions.removeEvent(id);
}

void TriggerManager::retireSubscriptions(AttachHandle attach)
{
    mTriggerSubscriptions.removeIf([attach](const TriggerSubscriptionEntry& subscription) {
        return subscription.attachHandle == attach;
    });
}

void TriggerManager::processNativeEvent(const usdparser::AttachedStage& attachedStage,
                                        omni::physics::parse::ObjectKey triggerColliderKey,
                                        omni::physics::parse::ObjectKey otherColliderKey,
                                        TriggerEventType::Enum eventType,
                                        omni::physics::parse::ObjectKey triggerBodyKey,
                                        omni::physics::parse::ObjectKey otherBodyKey,
                                        uint32_t triggerInstance, uint32_t otherInstance)
{
    const AttachHandle attachHandle = attachedStage.getAttachHandle();
    TriggerEventData triggerData;
    triggerData.attachHandle = attachHandle;
    triggerData.triggerColliderPrimKey = triggerColliderKey;
    triggerData.otherColliderPrimKey = otherColliderKey;
    triggerData.triggerBodyPrimKey = triggerBodyKey;
    triggerData.otherBodyPrimKey = otherBodyKey;
    triggerData.eventType = eventType;

    RuntimeCallbackScope callbackScope;
    // Snapshot matching keys only, preserving collider/body/wildcard order.
    const omni::physics::parse::ObjectKey filters[] = { triggerColliderKey, triggerBodyKey, {} };
    const auto snapshot = mTriggerSubscriptions.snapshot(attachHandle, filters, 3);
    for (const auto& entry : snapshot)
    {
        if (!mTriggerSubscriptions.contains(entry.first))
            continue;
        const TriggerSubscriptionEntry& subscription = entry.second;
        triggerData.subscriptionId = entry.first;
        try
        {
            if (subscription.detailedReport) subscription.detailedReport(&triggerData, triggerInstance, otherInstance);
            else if (subscription.reportFn) subscription.reportFn(&triggerData, subscription.userData);
        }
        catch (const std::exception& e) { CARB_LOG_ERROR("Physics trigger callback threw: %s", e.what()); }
        catch (...) { CARB_LOG_ERROR("Physics trigger callback threw an unknown exception"); }
    }
}

} // namespace physx

} // namespace omni
