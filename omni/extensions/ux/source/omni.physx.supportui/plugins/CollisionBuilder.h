// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <private/omni/physx/IPhysxSupportUi.h>

namespace omni
{
namespace physx
{

class CollisionBuilder final
{
public:
    static size_t constexpr NumPrimsToBatchProcessDefaultValue{ 10 };

    struct PrimCollDef
    {
        pxr::SdfPath mPrimPath;
        bool mDynamicBody = false;
        IPhysxSupportUi::StaticColliderSimplificationType mStaticColliderSimplificationType =
            IPhysxSupportUi::StaticColliderSimplificationType::eNone;
        IPhysxSupportUi::DynamicColliderSimplificationType mDynamicColliderSimplificationType =
            IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull;

        bool operator<(const PrimCollDef& pd) const
        {
            return mPrimPath < pd.mPrimPath;
        }
    };

    typedef std::deque<PrimCollDef> TPrimsToCreateCollidersContainer;
    typedef std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> TRigidBodiesForRemovalContainer;
    typedef std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> TRigidBodiesForAddContainer;

public:
    CollisionBuilder(size_t numPrimsToBatchProcess = NumPrimsToBatchProcessDefaultValue);
    ~CollisionBuilder();

    size_t numTotalItems() const
    {
        return mNumTotalItems;
    }
    size_t numItemsToProcess() const
    {
        return mPrimsToCreateColliders.size();
    }
    size_t numProcessedItems() const
    {
        return mNumProcessedItems;
    }

    void enumerateForColliderCreation(pxr::UsdStageWeakPtr stage,
                                      const std::vector<PrimDef>& primDefs,
                                      bool avoidChangingExistingColliders = false);

    void createColliders(pxr::UsdStageWeakPtr stage,
                         std::function<bool(const pxr::SdfPath& primPath, bool created)> rigidBodyChangeCallback = nullptr,
                         std::function<bool(const PrimCollDef& primCollDef)> creationCallback = nullptr);

    void reset();

    void setEventStream(carb::events::IEventStreamPtr eventStream);

private:
    static pxr::UsdPrim isRigidBodyOnNodeOrUpInHierarchy(
        pxr::UsdPrim startingPrim,
        const TRigidBodiesForAddContainer& rigidBodiesForAdd,
        const TRigidBodiesForRemovalContainer& rigidBodiesForRemoval,
        std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash>& primsWithRbParents);

    static bool hasGeomPrimInstancedProxyChild(pxr::UsdPrim prim);

    static bool isCollApproximationPresent(pxr::UsdPrim prim,
                                           IPhysxSupportUi::ColliderType collType,
                                           IPhysxSupportUi::StaticColliderSimplificationType staticCollSimplType,
                                           IPhysxSupportUi::DynamicColliderSimplificationType dynamicCollSimplType);

private:
    size_t mNumTotalItems{ 0 };
    size_t mNumProcessedItems{ 0 }; // switch to std::atomic_uint32_t for thread safety
    size_t mNumPrimsToBatchProcess{ NumPrimsToBatchProcessDefaultValue }; // number of prims to process in one batch

    TRigidBodiesForRemovalContainer mRigidBodiesForRemoval; // will get processed in createColliders() before creation
                                                            // of colliders themselves
    TRigidBodiesForAddContainer mRigidBodiesForAdd; // will get processed in createColliders() before creation of
                                                    // colliders themselves
    TPrimsToCreateCollidersContainer mPrimsToCreateColliders; // will get processed in createColliders()

    carb::dictionary::IDictionary* mDictionary{ nullptr };
    carb::events::IEventStreamPtr mEventStream;
};

} // namespace physx
} // namespace omni
