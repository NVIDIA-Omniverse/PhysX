// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <omni/physics/usd/PrimIterator.h>
#include <omni/physics/usd/StageScan.h>

#include "IceDescriptorAllocator.h"
#include "LoadUsd.h"
#include "CollisionGroup.h"

using namespace PXR_NS;

namespace omni
{
namespace physx
{
namespace usdparser
{

// @implements REQ-PARSE-COLGROUP-002
// @covers AC-5
void appendCollisionGroupFromPath(AttachedStage& attachedStage,
                                  const PXR_NS::SdfPath& collisionGroupPath,
                                  CollisionGroupsMap& collisionGroupsMap)
{
    if (collisionGroupPath.IsEmpty())
        return;

    const std::vector<SdfPath> scanRoots{ collisionGroupPath };
    static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
    omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
        attachedStage.attachTarget(), scanRoots, kNoExclude,
        omni::physx::usdparser::iceDescriptorAllocator(), scanOptions);

    // The walker emits the group itself (and any nested collision-group
    // prims in the subtree, though that's vanishingly rare).  Invert
    // each emitted group's `sourceMembers` into the engine map — same
    // shape transform `processScannedDescs` does at initial load.
    for (const auto& group : scanned.collisionGroups)
    {
        const SdfPath groupPath = scanned.pathFor(group->primKey);
        for (const omni::physics::parse::ObjectKey member : group->sourceMembers)
        {
            const SdfPath memberPath = scanned.pathFor(member);
            collisionGroupsMap[memberPath].push_back(groupPath);
        }
    }
}

ObjectId getCollisionGroup(AttachedStage& attachedStage, const PXR_NS::SdfPath& colliderPath)
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

} // namespace usdparser
} // namespace physx
} // namespace omni
