// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>
#include <internal/InternalFilteredPairs.h>
#include <usdLoad/FilteredPairs.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

bool omni::physx::updateFilteredPairs(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTFilteredPair)
    {
        // collectFilteredPairs (usdLoad/FilteredPairs.h) is ObjectKey-typed
        // (ADR-0019), so read the relationship through the pxr-free
        // TokenId+ObjectKey-vector sibling of getRelationshipValue directly --
        // no TfToken materialization or SdfPath round-trip needed.
        std::vector<omni::physics::parse::ObjectKey> data;
        if (!getRelationshipValue(attachedStage, objectRecord->mKey, property, data))
            data.clear();

        InternalFilteredPairs* intPairs = reinterpret_cast<InternalFilteredPairs*> (objectRecord->mInternalPtr);
        intPairs->removeFilteredPairs();
        intPairs->mPairs.clear();
        collectFilteredPairs(attachedStage, objectRecord->mKey, data, intPairs->mPairs);
        intPairs->createFilteredPairs();
    }

    return true;
}
