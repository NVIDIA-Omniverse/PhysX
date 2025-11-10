// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include "CollisionGroup.h"

using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{

CollisionGroupDesc* parseCollisionGroup(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim)
{
    CARB_ASSERT(usdPrim.IsA<UsdPhysicsCollisionGroup>());

    CollisionGroupDesc* desc = new CollisionGroupDesc();

    const UsdPhysicsCollisionGroup& collisionGroup = (const UsdPhysicsCollisionGroup&)(usdPrim);
    UsdRelationship rel = collisionGroup.GetFilteredGroupsRel();
    if (rel)
    {
        rel.GetTargets(&desc->filteredGroups);
    }
    
    UsdCollectionAPI collectionAPI = collisionGroup.GetCollidersCollectionAPI();
    if (collectionAPI)
    {
        desc->collisionQuery = collectionAPI.ComputeMembershipQuery();
    }

    return desc;
}

SdfPath getCollisionGroup(const pxr::SdfPath& colliderPath, const CollisionGroupMap& collisionGroupsMap)
{
    for (auto& collisionGroup : collisionGroupsMap)
    {
        if (collisionGroup.second->collisionQuery.IsPathIncluded(colliderPath))
            return collisionGroup.first;
    }

    return SdfPath();
}

} // namespace schema
} // namespace physics
} // namespace omni
