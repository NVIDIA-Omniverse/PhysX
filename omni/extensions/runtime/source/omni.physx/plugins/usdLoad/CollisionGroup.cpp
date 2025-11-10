// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/connectivity/Connectivity.h>

#include "LoadUsd.h"
#include "CollisionGroup.h"

using namespace pxr;
using namespace omni::fabric;

namespace omni
{
namespace physx
{
namespace usdparser
{

void updateCollisionCollection(const pxr::UsdPrim& collisionGroupPrim, CollisionGroupsMap& collisionGroupsMap)
{
    CARB_ASSERT(collisionGroupPrim.IsA<UsdPhysicsCollisionGroup>());
    const pxr::SdfPath& collisionGroupPath = collisionGroupPrim.GetPrimPath();

    const UsdPhysicsCollisionGroup& collisionGroup = (const UsdPhysicsCollisionGroup&)(collisionGroupPrim);
    const UsdCollectionAPI collectionAPI = collisionGroup.GetCollidersCollectionAPI();

    if (collectionAPI)
    {
        const SdfPathSet includedPaths = UsdCollectionAPI::ComputeIncludedPaths(collectionAPI.ComputeMembershipQuery(),
            collisionGroupPrim.GetStage(), UsdTraverseInstanceProxies());
        for (const SdfPath& path : includedPaths)
        {
            collisionGroupsMap[path].push_back(collisionGroupPath);            
        }        
    }
}

ObjectId getCollisionGroup(AttachedStage& attachedStage, const pxr::SdfPath& colliderPath)
{
    {
        const CollisionGroupsMap& collisionGroupsMap = attachedStage.getCollisionGroupMap();
        CollisionGroupsMap::const_iterator fit = collisionGroupsMap.find(colliderPath);
        if (fit != collisionGroupsMap.end())
        {
            if (fit->second.size() > 1)
            {
                CARB_LOG_WARN("Collisions are supported currently only in one collision group.");
            }
            return attachedStage.getObjectDatabase()->findEntry(fit->second[0], eCollisionGroup);
        }
    }
    // check also additional groups
    for (const CollisionGroupsMap& cg : attachedStage.getAdditionalCollisionGroupMaps())
    {         
        CollisionGroupsMap::const_iterator fit = cg.find(colliderPath);
        if (fit != cg.end())
        {
            if (fit->second.size() > 1)
            {
                CARB_LOG_WARN("Collisions are supported currently only in one collision group.");
            }
            return attachedStage.getObjectDatabase()->findEntry(fit->second[0], eCollisionGroup);
        }
    }

    return kInvalidObjectId;
}

void addChildPaths(usdrt::SdfPathSet& includePaths, usdrt::SdfPath path, omni::fabric::USDHierarchy& conn)
{
    includePaths.insert(path);

    const PathC firstChild = conn.getFirstChild(omni::fabric::PathC(path));
    if (firstChild != kUninitializedPath)
    {
        addChildPaths(includePaths, usdrt::SdfPath(firstChild), conn);
        PathC nextSibling = conn.getNextSibling(firstChild);
        while (firstChild != nextSibling && nextSibling != kUninitializedPath)
        {
            addChildPaths(includePaths, usdrt::SdfPath(nextSibling), conn);
            nextSibling = conn.getNextSibling(nextSibling);
        }
    }
}

void updateFabricCollisionGroups(AttachedStage& attachedStage)
{
    // get all collision groups
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(attachedStage.getStageId());
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(attachedStage.getStageId(), stageInProgress);

    omni::fabric::USDHierarchy conn(iStageReaderWriter->getFabricId(stageInProgress));
    CollisionGroupsMap& collisionGroupsMap = attachedStage.getCollisionGroupMap();
    usdrt::SdfPathSet includePaths;

    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsCollisionGroup")))
    {
        const usdrt::UsdPrim prim = usdrtStage->GetPrimAtPath(usdrtPath);
        const usdrt::UsdRelationship includeRel = prim.GetRelationship(usdrt::TfToken("collection:colliders:includes"));
        const usdrt::UsdRelationship excludeRel = prim.GetRelationship(usdrt::TfToken("collection:colliders:excludes"));
        usdrt::SdfPathVector paths;
        if (excludeRel)
        {
            excludeRel.GetTargets(&paths);
            if (!paths.empty())
            {
                CARB_LOG_ERROR("Collision groups with exclude rels not supported through fabric.");
                continue;
            }
        }
        if (includeRel)
        {
            includePaths.clear();
            includeRel.GetTargets(&paths);
            if (!paths.empty())
            {
                for (const usdrt::SdfPath& p : paths)
                {
                    addChildPaths(includePaths, p, conn);
                }
            }
            for (const usdrt::SdfPath& p : includePaths)
            {
                const omni::fabric::PathC pathCInclude(p);
                const omni::fabric::PathC pathCPath(usdrtPath);
                collisionGroupsMap[omni::fabric::toSdfPath(pathCInclude)].push_back(omni::fabric::toSdfPath(pathCPath));
            }        
        }
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
