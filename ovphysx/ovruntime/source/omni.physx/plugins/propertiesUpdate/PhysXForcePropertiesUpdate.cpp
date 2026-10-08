// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PhysXPropertiesUpdate.h"

#include <omni/physics/parse/KnownTokens.h>

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>
#include <Raycast.h>
#include <PhysXScene.h>
#include <usdInterface/UsdInterface.h>


#include <usdLoad/LoadTools.h>
#include <usdLoad/LoadUsd.h>

#include <carb/logging/Log.h>

#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// force

bool omni::physx::updatePhysxForceEnabled(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce *>(objectRecord->mInternalPtr);
        force->mEnabled = data;
    }
    return true;
}

bool omni::physx::updatePhysxForceWorldFrameEnabled(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->mWorldFrame = data;
    }
    return true;
}

bool omni::physx::updatePhysxForce(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        carb::Float3 data;
        if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->setForce(toPhysX(data));
    }
    return true;
}

bool omni::physx::updatePhysxTorque(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        carb::Float3 data;
        if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->setTorque(toPhysX(data));
    }
    return true;
}

bool omni::physx::updatePhysxForceMode(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTForce)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        InternalForce* force = reinterpret_cast<InternalForce*>(objectRecord->mInternalPtr);
        force->mAccelerationMode = (data == tok.acceleration);
    }
    return true;
}
