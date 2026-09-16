// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>
#include <omni/physics/parse/IPhysicsSource.h>

#include "IceDescriptorAllocator.h"
#include "LoadUsd.h"
#include "CollisionGroup.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

// @implements REQ-PARSE-COLGROUP-002
// @covers AC-5
void appendCollisionGroupFromPath(AttachedStage& attachedStage,
                                  omni::physics::parse::ObjectKey collisionGroupKey,
                                  CollisionGroupsMap& collisionGroupsMap)
{
    if (!collisionGroupKey.valid())
        return;

    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(collisionGroupKey)) };
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
    omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
        attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
        omni::physx::usdparser::iceDescriptorAllocator());

    const omni::physics::parse::IPhysicsSource* scannedSource = scanned.sourcePtr();
    if (!scannedSource)
        return;

    // The walker emits the group itself (and any nested collision-group
    // prims in the subtree, though that's vanishingly rare).  Invert
    // each emitted group's `sourceMembers` into the engine map -- same
    // shape transform `processScannedDescs` does at initial load. `scanned`
    // mints its own ObjectKeys in a fresh intern table (a different
    // generation than `attachedStage`'s), so both `primKey` and each
    // `sourceMembers` entry must be re-keyed into attachedStage's space via
    // the path-string round trip before landing in the shared map -- same
    // rekey `invertCollisionGroupMembers` (LoadStage.cpp) does for the
    // initial-load population of this same map.
    for (const auto& group : scanned.collisionGroups)
    {
        const omni::physics::parse::ObjectKey groupKey =
            attachedStage.keyFor(scannedSource->sourceKeyToString(group->primKey));
        for (const omni::physics::parse::ObjectKey member : group->sourceMembers)
        {
            const omni::physics::parse::ObjectKey memberKey =
                attachedStage.keyFor(scannedSource->sourceKeyToString(member));
            collisionGroupsMap[memberKey].push_back(groupKey);
        }
    }
}

ObjectId getCollisionGroup(AttachedStage& attachedStage, omni::physics::parse::ObjectKey colliderKey)
{
    {
        const CollisionGroupsMap& collisionGroupsMap = attachedStage.getCollisionGroupMap();
        CollisionGroupsMap::const_iterator fit = collisionGroupsMap.find(colliderKey);
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
        CollisionGroupsMap::const_iterator fit = cg.find(colliderKey);
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
