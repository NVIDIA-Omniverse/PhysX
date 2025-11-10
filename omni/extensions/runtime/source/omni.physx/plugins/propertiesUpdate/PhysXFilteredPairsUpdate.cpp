// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

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
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

bool omni::physx::updateFilteredPairs(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTFilteredPair)
    {
        SdfPathVector data;
        if (!getRelationshipValue(attachedStage, objectRecord->mPath, property, data))
            data.clear();

        InternalFilteredPairs* intPairs = reinterpret_cast<InternalFilteredPairs*> (objectRecord->mInternalPtr);
        intPairs->removeFilteredPairs();
        intPairs->mPairs.clear();
        collectFilteredPairs(attachedStage, objectRecord->mPath, data, intPairs->mPairs);        
        intPairs->createFilteredPairs();
    }

    return true;
}
