// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"


#include "Trigger.h"
#include "Setup.h"
#include "OmniPhysX.h"

#include "internal/InternalScene.h"
#include "usdLoad/AttachedStage.h"

#include <carb/logging/Log.h>
#include <carb/tokens/ITokens.h>
#include <carb/tokens/TokensUtils.h>
#include <carb/InterfaceUtils.h>

#include <iostream>

using namespace pxr;
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
    
    processTriggerStateApiEnterEvent(triggerRecord->mPath, otherRecord->mPath, otherShape);
    if (triggerShape->getActor() && otherShape->getActor()
        && (size_t)triggerShape->getActor()->userData < db.getRecords().size()
        && (size_t)otherShape->getActor()->userData < db.getRecords().size())
    {
        const InternalDatabase::Record& triggerBodyRecord = db.getRecords()[(size_t)triggerShape->getActor()->userData];
        const InternalDatabase::Record& otherBodyRecord = db.getRecords()[(size_t)otherShape->getActor()->userData];
        processNativeEvent(attachedStage, triggerRecord->mPath, otherRecord->mPath, TriggerEventType::eTRIGGER_ON_ENTER,
            triggerBodyRecord.mPath, otherBodyRecord.mPath);
    }
}

void TriggerManager::processTriggerStateApiEnterEvent(const pxr::SdfPath& triggerPath, const pxr::SdfPath& otherPath, const ::physx::PxShape* otherShape)
{
    TriggerUsdOutputMap::iterator fit = mTriggerOutputMap.find(triggerPath);
    if (fit != mTriggerOutputMap.end())
    {
        const pxr::SdfPath collisionPath = otherPath;
        UsdOutput& usdOutput = fit->second;
        usdOutput.dirty = true;
        TriggerCollisionMap::const_iterator fit = usdOutput.triggerCollisionmap.find(collisionPath);
        // if its not yet found add
        if (fit == usdOutput.triggerCollisionmap.end())
        {            
            usdOutput.triggerCollisionmap.insert(std::make_pair(collisionPath, std::make_pair(otherShape, usdOutput.triggeredCollisions.size())));
            usdOutput.triggeredCollisions.push_back(collisionPath);
        }
        else
        {
            // another one for the compound shape
            usdOutput.triggerCollisionmap.insert(std::make_pair(collisionPath, std::make_pair(otherShape, fit->second.second)));
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
    
    processTriggerStateApiLeaveEvent(triggerRecord->mPath, otherRecord->mPath, otherShape);

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
    processNativeEvent(attachedStage, triggerRecord->mPath, otherRecord->mPath, TriggerEventType::eTRIGGER_ON_LEAVE,
        triggerBodyRecord->mPath,
                       otherBodyRecord ? otherBodyRecord->mPath : SdfPath());
}

void TriggerManager::processTriggerStateApiLeaveEvent(const pxr::SdfPath& triggerPath, const pxr::SdfPath& otherPath, const ::physx::PxShape* otherShape)
{
    TriggerUsdOutputMap::iterator fit = mTriggerOutputMap.find(triggerPath);
    if (fit != mTriggerOutputMap.end())
    {
        const pxr::SdfPath collisionPath = otherPath;
        UsdOutput& usdOutput = fit->second;
        usdOutput.dirty = true;
        TriggerCollisionMap::iterator cf = usdOutput.triggerCollisionmap.find(collisionPath);
        size_t pathIndex = 0;
        bool indexFound = false;
        while (cf != usdOutput.triggerCollisionmap.end() && cf->first == collisionPath)
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
            cf = usdOutput.triggerCollisionmap.find(collisionPath);
            if (cf == usdOutput.triggerCollisionmap.end())
            {
                usdOutput.triggeredCollisions[pathIndex] = usdOutput.triggeredCollisions.back();
                usdOutput.triggeredCollisions.pop_back();
                if (!usdOutput.triggeredCollisions.empty())
                {
                    const SdfPath& movedPath = usdOutput.triggeredCollisions[pathIndex];
                    TriggerCollisionMap::iterator cfb = usdOutput.triggerCollisionmap.find(movedPath);
                    while (cfb != usdOutput.triggerCollisionmap.end() && cfb->first == movedPath)
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

    for (TriggerUsdOutputMap::reference ref : mTriggerOutputMap)
    {
        UsdOutput& usdOutput = ref.second;
        if (usdOutput.dirty)
        {
            if (usdOutput.triggerStateAPI)
            {
                usdOutput.triggerStateAPI.GetTriggeredCollisionsRel().SetTargets(usdOutput.triggeredCollisions);
            }
            usdOutput.dirty = false;
        }
    }
}


void TriggerManager::preloadTrigger(const pxr::UsdPrim& triggerPrim, bool usdOutput)
{
    if (usdOutput)
    {
        UsdOutput usdOutput;
        usdOutput.dirty = false;
        usdOutput.triggerStateAPI =
            PhysxSchemaPhysxTriggerStateAPI::Get(triggerPrim.GetStage(), triggerPrim.GetPrimPath());
        mTriggerOutputMap[triggerPrim.GetPrimPath()] = usdOutput;
    }
}

void TriggerManager::clearTriggers()
{
    mInvokedTriggers.clear();

    for (TriggerUsdOutputMap::reference ref : mTriggerOutputMap)
    {
        UsdOutput& usdOutput = ref.second;
        if (usdOutput.triggerStateAPI)
        {
            usdOutput.triggerStateAPI.GetTriggeredCollisionsRel().ClearTargets(true);
        }
    }

    mTriggerOutputMap.clear();
}

omni::physx::SubscriptionId TriggerManager::registerNativeCallback(TriggerSubscriptionEntry triggerSubscription)
{
    omni::physx::SubscriptionId subId = mTriggerSubscriptions.addEvent(triggerSubscription);
    pxr::SdfPath triggerPrimPath = intToSdfPath(triggerSubscription.triggerColliderPrimId);
    mTriggerSubscriptionsMap.insert(std::make_pair(triggerPrimPath, subId));
    return subId;
}

void TriggerManager::unregisterNativeCallback(omni::physx::SubscriptionId subscriptionID)
{
    auto it = mTriggerSubscriptions.map.find(subscriptionID);
    if(it == mTriggerSubscriptions.map.end())
        return;
    TriggerSubscriptionEntry triggerSubscription = it->second;
    pxr::SdfPath triggerPrimPath = intToSdfPath(triggerSubscription.triggerColliderPrimId);
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

void TriggerManager::processNativeEvent(const usdparser::AttachedStage& attachedStage,
                                        const pxr::SdfPath& triggerColliderPath,
                                        const pxr::SdfPath& otherColliderPath,
                                        TriggerEventType::Enum eventType,
                                        const pxr::SdfPath& triggerBodyPath, 
                                        const pxr::SdfPath& otherBodyPath)
{
    const uint64_t stageId = attachedStage.getStageId();
    TriggerEventData triggerData;
    triggerData.stageId = stageId;
    triggerData.triggerColliderPrimId = sdfPathToInt(triggerColliderPath);
    triggerData.otherColliderPrimId = sdfPathToInt(otherColliderPath);
    triggerData.triggerBodyPrimId = sdfPathToInt(triggerBodyPath);
    triggerData.otherBodyPrimId = sdfPathToInt(otherBodyPath);
    triggerData.eventType = eventType;

    // 1. Report the trigger pair for all subscription that have explicitly been watching this specific path
    // Need to use equal_range instead of find to get all subscriptions for a given USD path (potentially > 1)
    auto triggerColliderSubscriptions = mTriggerSubscriptionsMap.equal_range(triggerColliderPath);
    for (auto it = triggerColliderSubscriptions.first; it != triggerColliderSubscriptions.second; ++it)
    {
        omni::physx::SubscriptionId subId = it->second;
        const TriggerSubscriptionEntry& subscription = mTriggerSubscriptions.map.at(subId);
        // stageId == 0 means trigger is reported no matter what the stageId is
        // stageId != 0 means it should match the stageId that was registered in the callback
        if (subscription.stageId == 0 || subscription.stageId == stageId)
        {
            triggerData.subscriptionId = subId;
            subscription.reportFn(&triggerData, subscription.userData);
        }
    }

    // 2. If body and collider are not at the same USD path, check if we have listeners listening for body path
    if(triggerColliderPath != triggerBodyPath)
    {
        auto triggerBodySubscription = mTriggerSubscriptionsMap.equal_range(triggerBodyPath);
        for (auto it = triggerBodySubscription.first; it != triggerBodySubscription.second; ++it)
        {
            omni::physx::SubscriptionId subId = it->second;
            const TriggerSubscriptionEntry& subscription = mTriggerSubscriptions.map.at(subId);
            // stageId == 0 means trigger is reported no matter what the stageId is
            // stageId != 0 means it should match the stageId that was registered in the callback
            if (subscription.stageId == 0 || subscription.stageId == stageId)
            {
                triggerData.subscriptionId = subId;
                subscription.reportFn(&triggerData, subscription.userData);
            }
        }
    }

    // 3. Report the trigger for all subs that have not indicated a specific path to listen for
    auto allSubscriptionForEmptyPath = mTriggerSubscriptionsMap.equal_range(pxr::SdfPath());
    for (auto it = allSubscriptionForEmptyPath.first; it != allSubscriptionForEmptyPath.second; ++it)
    {
        omni::physx::SubscriptionId subId = it->second;
        const TriggerSubscriptionEntry& subscription = mTriggerSubscriptions.map.at(subId);
        // stageId == 0 means trigger is reported no matter what the stageId is
        // stageId != 0 means it should match the stageId that was registered in the callback
        if (subscription.stageId == 0 || subscription.stageId == stageId)
        {
            triggerData.subscriptionId = subId;
            subscription.reportFn(&triggerData, subscription.userData);
        }
    }
}

} // namespace physx

} // namespace omni
