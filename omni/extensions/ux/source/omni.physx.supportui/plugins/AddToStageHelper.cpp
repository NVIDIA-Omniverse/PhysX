// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#define CARB_EXPORTS
#include <carb/PluginUtils.h>

#include <common/utilities/Utilities.h>

#include <private/omni/physx/IPhysxSupportUi.h>

#include "AddToStageHelper.h"
#include "CollisionBuilder.h"
#include "PrimUpdateMap.h"
#include "Utils.h"

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/usd/PathConversion.h>

OMNI_LOG_DECLARE_CHANNEL(kSupportUiLogChannel)

using namespace pxr;

namespace omni
{
namespace physx
{

AddToStageHelper::AddToStageHelper() : mStage(nullptr)
{
    mDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>(); // Get the carbonite dictionary interface
    CARB_ASSERT(mDictionary != nullptr, "AddToStageHelper: Failed to obtain dictionary interface!");

    mSettings = carb::getCachedInterface<carb::settings::ISettings>(); // Get the carbonite settings interface
    CARB_ASSERT(mSettings != nullptr, "AddToStageHelper: Failed to obtain settings interface!");

    mPhysXCooking = carb::getCachedInterface<omni::physx::IPhysxCooking>(); // Get the IPhysxCooking interface
    if (mPhysXCooking == nullptr) // just log the error, we can live without IPhysxCooking
    {
        CARB_LOG_ERROR("AddToStageHelper: Failed to obtain IPhysxCooking interface!");
    }

    mPhysXCookingPrivate = carb::getCachedInterface<omni::physx::IPhysxCookingPrivate>(); // Get the IPhysxCooking interface
    if (mPhysXCookingPrivate == nullptr) // just log the error, we can live without IPhysxCooking
    {
        CARB_LOG_ERROR("AddToStageHelper: Failed to obtain IPhysxCookingPrivate interface!");
    }
    carb::dictionary::SubscriptionId* subId;

    mAutomaticColliderCreationEnabled = mSettings->getAsBool(kSettingsAutomaticColliderCreationEnabled);

    subId = mSettings->subscribeToNodeChangeEvents(
        kSettingsAutomaticColliderCreationEnabled,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType changeEventType, void* userData) {
            if (changeEventType == carb::dictionary::ChangeEventType::eChanged)
            {
                AddToStageHelper* ash = static_cast<AddToStageHelper*>(userData);

                if (ash != nullptr && ash->mDictionary != nullptr)
                {
                    ash->mAutomaticColliderCreationEnabled = ash->mDictionary->getAsBool(changedItem);
                }
            }
        },
        this);
    mSubscribedSettings.push_back(subId);

    mAsyncCookingAtStageLoad = mSettings->getAsBool(kSettingsAsyncCookingAtStageLoad);

    subId = mSettings->subscribeToNodeChangeEvents(
        kSettingsAsyncCookingAtStageLoad,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType changeEventType, void* userData) {
            if (changeEventType == carb::dictionary::ChangeEventType::eChanged)
            {
                AddToStageHelper* ash = static_cast<AddToStageHelper*>(userData);

                if (ash != nullptr && ash->mDictionary != nullptr)
                {
                    ash->mAsyncCookingAtStageLoad = ash->mDictionary->getAsBool(changedItem);
                }
            }
        },
        this);
    mSubscribedSettings.push_back(subId);
}

AddToStageHelper::~AddToStageHelper()
{
    mStage = nullptr;
    mEventStream = nullptr;

    for (const auto subId : mSubscribedSettings)
    {
        mSettings->unsubscribeToChangeEvents(subId);
    }

    mSubscribedSettings.clear();
}

void AddToStageHelper::setEventStream(carb::events::IEventStreamPtr eventStream)
{
    mEventStream = eventStream;
    mCollisionBuilder.setEventStream(mEventStream);
}

void AddToStageHelper::setAutomaticColliderCreation(bool enable)
{
    mSettings->setBool(kSettingsAutomaticColliderCreationEnabled, enable);
}

void AddToStageHelper::setAsyncCookingAtStageLoad(bool enable)
{
    mSettings->setBool(kSettingsAsyncCookingAtStageLoad, enable);
}

bool AddToStageHelper::processRequestCookingPrim(const pxr::UsdPrim& prim)
{
    bool requestCooking = true;
    if (prim.IsInstanceProxy())
    {
        // if the prim is instanced, get its prototype master and send it to cooking only once.
        const UsdPrim prototypePrim = prim.GetPrimInPrototype();
        if (prototypePrim)
        {
            const SdfPath prototypePrimPath = prototypePrim.GetPrimPath();

            auto it = mPrototypesPathsScheduledForCooking.find(prototypePrimPath);
            if (it == mPrototypesPathsScheduledForCooking.end())
            {
                mPrototypesPathsScheduledForCooking.insert(prototypePrimPath);
            }
            else
            {
                OMNI_LOG_INFO(
                    kSupportUiLogChannel,
                    "SKIPPING cooking for \"%s\" as another prim with identical prototype has already been requested (prototype path is \"%s\")...",
                    prim.GetPrimPath().GetText(), prototypePrimPath.GetText());
                requestCooking = false;
            }
        }
    }
    return requestCooking;
}

void AddToStageHelper::requestCookingForPrimsRange(const UsdPrimRange& primRange)
{
    if (mPhysXCooking == nullptr)
    {
        CARB_LOG_ERROR("Cannot request cooking for prims range as IPhysXCooking is null.");
        return;
    }

    for (UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
    {
        const UsdPrim& prim = *iter;
        const SdfPath primPath = prim.GetPath();

        if (!prim.IsValid() || prim.IsPseudoRoot())
        {
            continue;
        }

        if (prim.IsA<pxr::UsdGeomGprim>())
        {
            if (prim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                const bool requestCooking = processRequestCookingPrim(prim);

                if (requestCooking)
                {
                    OMNI_LOG_INFO(kSupportUiLogChannel, "Requesting cooking for \"%s\"...", primPath.GetText());
                    mPhysXCookingPrivate->addPrimToCookingRefreshSet(primPath);
                }
            }
        }
        else if (prim.IsA<UsdGeomPointInstancer>())
        {
            iter.PruneChildren();
        }
    }
}

void AddToStageHelper::requestCookingForStage(pxr::UsdStageWeakPtr stage)
{
    if (mPhysXCooking == nullptr)
    {
        CARB_LOG_ERROR("Cannot request cooking for prims range as IPhysXCooking is null.");
        return;
    }

    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));
    if (usdrtStage)
    {
        for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysicsCollisionAPI")))
        {
            const omni::fabric::PathC pathC(usdrtPath);
            const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
            const pxr::UsdPrim prim = stage->GetPrimAtPath(usdPath);
            if (prim && prim.IsA<pxr::UsdGeomGprim>())
            {
                const bool requestCooking = processRequestCookingPrim(prim);

                if (requestCooking)
                {
                    OMNI_LOG_INFO(kSupportUiLogChannel, "Requesting cooking for \"%s\"...", prim.GetPrimPath().GetText());
                    mPhysXCookingPrivate->addPrimToCookingRefreshSet(prim.GetPrimPath());
                }
            }
        }
    }
}

void AddToStageHelper::setStage(const UsdStageWeakPtr& stage)
{
    CARB_PROFILE_ZONE(0, "PhysXSupportUi AddToStageHelper::setStage");

    OMNI_LOG_INFO(kSupportUiLogChannel, "AddToStageHelper::setStage: \"%s\"",
                  stage && stage->GetRootLayer() ? stage->GetRootLayer()->GetIdentifier().c_str() : "<empty>");

    clearPrimQueue();

    mPrototypesPathsScheduledForCooking.clear();

    mStage = stage;

    if (stage != nullptr)
    {
        if (isAutomaticColliderCreationEnabled())
        {
            requestColliderCreation(stage->GetPseudoRoot(), false, IPhysxSupportUi::ColliderType::eAutodetect);
        }
        else if (isAsyncCookingAtStageLoadEnabled()) // it is 'else' here because since CL#31389862 cooking will get requested always on ApiSchema change by omni.physx
        {            
            requestCookingForStage(stage);
        }
    }
}

void AddToStageHelper::fillInPrimDef(PrimDef& primDef,
                                     const UsdPrim& prim,
                                     bool forceColliderCreation,
                                     IPhysxSupportUi::ColliderType colliderType)
{
    primDef.mPrim = prim;
    primDef.mForceCollCreation = forceColliderCreation;
    primDef.mColliderType = colliderType;
    primDef.mStaticColliderSimplificationType = static_cast<IPhysxSupportUi::StaticColliderSimplificationType>(
        mSettings->getAsInt(kSettingsStaticColliderSimplificationType));
    primDef.mDynamicColliderSimplificationType = static_cast<IPhysxSupportUi::DynamicColliderSimplificationType>(
        mSettings->getAsInt(kSettingsDynamicColliderSimplificationType));
}

void AddToStageHelper::requestColliderCreation(const pxr::UsdPrim& prim,
                                               bool forceColliderCreation,
                                               IPhysxSupportUi::ColliderType colliderType)
{
    PrimDef primDef;

    fillInPrimDef(primDef, prim, forceColliderCreation, colliderType);

    mPrimUpdateMap.addPrim(primDef);
}

void AddToStageHelper::primAdded(const pxr::UsdPrim& prim,
                                 bool forceColliderCreation,
                                 IPhysxSupportUi::ColliderType colliderType)
{
    if (isAutomaticColliderCreationEnabled())
    {
        requestColliderCreation(prim, forceColliderCreation, colliderType);
    }
    else if (isAsyncCookingAtStageLoadEnabled()) // it is 'else' here because since CL#31389862 cooking will get requested always on ApiSchema change by omni.physx
    {
        requestCookingForPrimsRange(UsdPrimRange(prim, UsdTraverseInstanceProxies()));
    }
}

void AddToStageHelper::removePrim(const SdfPath& primPath)
{
    mPrimUpdateMap.removePrim(primPath);
}

void AddToStageHelper::enumeratePrims()
{
    if (mPrimUpdateMap.getMap().size() > 0)
    {
        // get vector of all values from the map
        std::vector<PrimDef> primDefs;

        const PrimUpdateMap::TPrimAddMap& primMap = mPrimUpdateMap.getMap();
        std::transform(primMap.begin(), primMap.end(), std::back_inserter(primDefs),
                       [](const PrimUpdateMap::TPrimAddMap::value_type& val) { return val.second; });

        bool avoidChangingExistingColliders = mSettings->getAsBool(kSettingsAvoidChangingExistingColliders);
        mCollisionBuilder.enumerateForColliderCreation(mStage, primDefs, avoidChangingExistingColliders);

        mPrimUpdateMap.clearMap();
    }
}

void AddToStageHelper::createColliders()
{
    if (numQueuedCollidersToCreate() > 0)
    {
        OMNI_LOG_INFO(kSupportUiLogChannel, "Automatic collision batching in progress (%zu items left)...",
                      mCollisionBuilder.numItemsToProcess());
    }

    mCollisionBuilder.createColliders(mStage,
        [this](const pxr::SdfPath& primPath, bool created)
        {
            if (created)
            {
                OMNI_LOG_INFO(kSupportUiLogChannel, "Applying rigid body API on \"%s\"", primPath.GetText());
            }
            else
            {
                OMNI_LOG_INFO(kSupportUiLogChannel, "Removing rigid body API on \"%s\"", primPath.GetText());
            }

            sendRigidBodyApiChangedEvent(mEventStream, mDictionary, primPath, created);

            return true;
        },
        [this](const CollisionBuilder::PrimCollDef& primCollDef)
        {
            OMNI_LOG_INFO(kSupportUiLogChannel, "Creating %s collider for \"%s\"",
                                   primCollDef.mDynamicBody ? "dynamic" : "static",
                                   primCollDef.mPrimPath.GetText());

            sendColliderCreatedEvent(mEventStream, mDictionary, primCollDef.mPrimPath,
                                     primCollDef.mDynamicBody,
                                     primCollDef.mDynamicBody ?
                                        static_cast<int>(primCollDef.mDynamicColliderSimplificationType) :
                                        static_cast<int>(primCollDef.mStaticColliderSimplificationType),
                                     mCollisionBuilder.numItemsToProcess() - 1,
                                     mCollisionBuilder.numTotalItems());

            return true;
        }
    );
}

size_t AddToStageHelper::numQueuedCollidersToCreate() const
{
    return mCollisionBuilder.numItemsToProcess();
}

void AddToStageHelper::clearPrimQueue()
{
    mCollisionBuilder.reset();
    mPrimUpdateMap.clearMap();
}

void AddToStageHelper::update()
{
    SdfChangeBlock changeBlock;

    // enumerate newly incoming prims (if any) for possible collider creation
    enumeratePrims();

    // create colliders on chosen prims. This works in batches.
    createColliders();
}

} // namespace physx
} // namespace omni
