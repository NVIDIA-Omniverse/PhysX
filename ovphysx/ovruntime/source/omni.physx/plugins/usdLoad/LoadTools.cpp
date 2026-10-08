// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SHAPE-003
 * @covers AC-1 AC-2
 *
 * @implements REQ-LOAD-OBJECTDB-001
 * @covers AC-1 AC-2 AC-3
 */

#include "LoadTools.h"
#include "PhysXTools.h"

#include <private/omni/physics/CollisionShapeTransform.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <vector>

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

void getCollisionShapeLocalTransform(const AttachedStage& attachedStage,
                                     omni::physics::parse::ObjectKey collisionKey,
                                     omni::physics::parse::ObjectKey bodyKey,
                                     carb::Float3& localPosOut,
                                     carb::Float4& localRotOut,
                                     carb::Float3& localScaleOut)
{
    // World transforms via the physics source. The replaced load-time xform
    // cache was pinned to EarliestTime, so use the time-independent
    // (EarliestTime, cached) overload to match it exactly.
    const ::physx::PxMat44d bodyWorld = internal::getWorldTransform(attachedStage, bodyKey);
    ::physx::PxMat44d shapeToBody(::physx::PxIdentity);
    if (collisionKey != bodyKey)
    {
        const ::physx::PxMat44d collWorld = internal::getWorldTransform(attachedStage, collisionKey);
        // Collision prim's transform relative to the body: collWorld * bodyWorld^-1
        // in the USD row-vector convention, which is the reversed product here.
        shapeToBody = affineInverse(bodyWorld) * collWorld;
    }

    // Matrix sourcing remains specific to this legacy path; the descriptor math
    // is the shared helper so this stays numerically consistent with the
    // scanStage walkers on sheared shapeToBody/bodyWorld input (see
    // CollisionShapeTransform.h).
    omni::physics::decomposeCollisionShapeLocalTransform(shapeToBody, bodyWorld, localPosOut, localRotOut, localScaleOut);
}

//// ObjectDb methods ////

// Fully ObjectKey/std::string keyed; the SdfPath-taking legacy overloads were removed with
// their last callers (see the ObjectDb top comment in LoadTools.h).

void ObjectDb::dropHierarchyRow(omni::physics::parse::ObjectKey key)
{
    const KeyPathMap::iterator it = mKeyPathText.find(key);
    if (it == mKeyPathText.end())
        return;

    // Only a childless row is this object's alone: addPrim also materializes ancestor rows,
    // so a row that still has children is a live ancestor of some other registered object and
    // removePrim would take that whole subtree down with it. Such a row is left to the
    // subtree-level cleanup (PrimUpdateMap::removePrim's removeIteration).
    const PrimHierarchyStorage::StorageMap& rows = mPrimHierarchyStorage.getStorageMap();
    const PrimHierarchyStorage::StorageMap::const_iterator rowIt = rows.find(it->second);
    if (rowIt != rows.end() && rowIt->second.children.empty())
        mPrimHierarchyStorage.removePrim(it->second);

    mKeyPathText.erase(it);
}

bool ObjectDb::removeEntries(omni::physics::parse::ObjectKey key)
{
    // Real subtree clear, unlike the SdfPath overload above (which only ever
    // removes the single given path -- its callers loop over
    // PrimHierarchyStorage::Iterator's descendants themselves). Walks mKeyMap
    // via isAncestorOrSelf instead of PrimHierarchyStorage, mirroring
    // AttachedStage.h's clearGeneratedDeformableAttachmentDataUnderPath /
    // clearCookedGeometryUnderPath (ADR-0019 decision 2).
    std::vector<omni::physics::parse::ObjectKey> removedKeys;
    for (KeyMap::iterator it = mKeyMap.begin(); it != mKeyMap.end();)
    {
        if (isAncestorOrSelf(key, it->first))
        {
            removedKeys.push_back(it->first);
            it = mKeyMap.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Deepest path first, so a row is childless by the time its own key is processed:
    // dropHierarchyRow declines to take a row that still has children, and a descendant's
    // path is always the longer string. mKeyMap iterates in hash order, so without this the
    // outcome would depend on it.
    const auto pathLength = [this](omni::physics::parse::ObjectKey k)
    {
        const KeyPathMap::const_iterator it = mKeyPathText.find(k);
        return it == mKeyPathText.end() ? size_t(0) : it->second.size();
    };
    std::sort(removedKeys.begin(), removedKeys.end(),
              [&pathLength](omni::physics::parse::ObjectKey a, omni::physics::parse::ObjectKey b)
              { return pathLength(a) > pathLength(b); });
    for (const omni::physics::parse::ObjectKey removedKey : removedKeys)
        dropHierarchyRow(removedKey);

    return !removedKeys.empty();
}

void ObjectDb::removeEntry(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId entryId)
{
    KeyMap::iterator fit = mKeyMap.find(key);
    if (fit != mKeyMap.end())
    {
        ObjectIdMap& entries = fit->second;

        std::pair<ObjectIdMap::iterator, ObjectIdMap::iterator> pairIter = entries.equal_range(category);
        ObjectIdMap::iterator it = pairIter.first;
        while (it != pairIter.second)
        {
            if (it->second == entryId)
            {
                entries.erase(it);
                break;
            }

            it++;
        }

        if (entries.size() == 0)
        {
            mKeyMap.erase(fit);
            // Last entry at this key: the path-keyed row creation added is now stale, and a
            // stale row still answers path lookups (the tensor wildcard matcher's literal
            // fast path in BaseSimulationView.cpp resolves a hit straight back to an
            // ObjectKey) for an object that has been released.
            dropHierarchyRow(key);
        }
    }
}


} // namespace usdparser
} // namespace physx
} // namespace omni
