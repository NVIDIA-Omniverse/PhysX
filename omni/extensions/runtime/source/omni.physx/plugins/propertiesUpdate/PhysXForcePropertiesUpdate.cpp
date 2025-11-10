// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>
#include <Raycast.h>
#include <PhysXScene.h>
#include <ContactReport.h>
#include <usdInterface/UsdInterface.h>

#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include <usdLoad/LoadTools.h>
#include <usdLoad/LoadUsd.h>
#include <usdLoad/PhysicsBody.h>

#include <carb/logging/Log.h>

#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// force

bool omni::physx::updatePhysxForceEnabled(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce *>(objectRecord->mInternalPtr);
        force->mEnabled = data;
    }
    return true;
}

bool omni::physx::updatePhysxForceWorldFrameEnabled(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->mWorldFrame = data;
    }
    return true;
}

bool omni::physx::updatePhysxForce(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->setForce(toPhysX(data));
    }
    return true;
}

bool omni::physx::updatePhysxTorque(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->setTorque(toPhysX(data));
    }
    return true;
}

bool omni::physx::updatePhysxForceMode(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->mAccelerationMode = data == PhysxSchemaTokens->acceleration ? true : false;
    }
    return true;
}
