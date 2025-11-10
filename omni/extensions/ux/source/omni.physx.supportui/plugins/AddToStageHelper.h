// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/events/EventsUtils.h>
#include <carb/events/IEvents.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>
#include <private/omni/physx/IPhysxSupportUi.h>

#include "PrimUpdateMap.h"
#include "CollisionBuilder.h"

namespace omni
{
namespace physx
{

class AddToStageHelper final
{
public:
    AddToStageHelper();
    ~AddToStageHelper();

    void setStage(const pxr::UsdStageWeakPtr& stage);

    void requestColliderCreation(const pxr::UsdPrim& prim,
                                 bool forceColliderCreation = false,
                                 IPhysxSupportUi::ColliderType colliderType = IPhysxSupportUi::ColliderType::eAutodetect);

    void primAdded(const pxr::UsdPrim& prim,
                   bool forceColliderCreation = false,
                   IPhysxSupportUi::ColliderType colliderType = IPhysxSupportUi::ColliderType::eAutodetect);

    void removePrim(const pxr::SdfPath& primPath);
    void update();

    void setEventStream(carb::events::IEventStreamPtr eventStream);

    bool isAutomaticColliderCreationEnabled() const
    {
        return mAutomaticColliderCreationEnabled;
    }
    void setAutomaticColliderCreation(bool enable);

    bool isAsyncCookingAtStageLoadEnabled() const
    {
        return mAsyncCookingAtStageLoad;
    }
    void setAsyncCookingAtStageLoad(bool enable);

    size_t numQueuedCollidersToCreate() const;
    void clearPrimQueue();

    void requestCookingForPrimsRange(const pxr::UsdPrimRange& primRange);
    void requestCookingForStage(pxr::UsdStageWeakPtr stage);

private:
    bool processRequestCookingPrim(const pxr::UsdPrim& prim);
    void enumeratePrims();
    void createColliders();

    void fillInPrimDef(PrimDef& primDef,
                       const pxr::UsdPrim& prim,
                       bool forceColliderCreation,
                       IPhysxSupportUi::ColliderType colliderType);

private:
    carb::settings::ISettings* mSettings{ nullptr };
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;
    bool mAutomaticColliderCreationEnabled{ false };
    bool mAsyncCookingAtStageLoad{ true };
    // Optimization: SdfPaths of instance prototypes that have already had cooking requested
    std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> mPrototypesPathsScheduledForCooking;

    carb::dictionary::IDictionary* mDictionary{ nullptr };
    carb::events::IEventStreamPtr mEventStream;
    omni::physx::IPhysxCooking* mPhysXCooking{ nullptr };
    omni::physx::IPhysxCookingPrivate* mPhysXCookingPrivate{ nullptr };
    pxr::UsdStageWeakPtr mStage;
    PrimUpdateMap mPrimUpdateMap;
    CollisionBuilder mCollisionBuilder;
};

} // namespace physx
} // namespace omni
