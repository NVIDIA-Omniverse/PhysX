// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PhysXPropertiesUpdate.h"

#include <omni/physics/parse/KnownTokens.h>

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// CCT
bool omni::physx::updateCctSlopeLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setSlopeLimit(data);
        }
    }
    return true;
}

bool omni::physx::updateCctHeight(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        double data;
        if (!getValue<double>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxCapsuleController* cct = (PxCapsuleController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setHeight(float(data));
        }
    }
    return true;
}

bool omni::physx::updateCctRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        double data;
        if (!getValue<double>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxCapsuleController* cct = (PxCapsuleController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setRadius(float(data));
        }
    }
    return true;
}

bool omni::physx::updateCctContactOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setContactOffset(data);
        }
    }
    return true;
}

bool omni::physx::updateCctStepOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setStepOffset(data);
        }
    }
    return true;
}

bool omni::physx::updateCctUpAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            if (!source)
                return true;
            omni::physics::parse::TokenId data;
            if (!getValue<omni::physics::parse::TokenId>(attachedStage, objectRecord->mKey, property, timeCode, data))
                return true;

            omni::physics::parse::KnownTokens tok;
            tok.intern(*source);

            if (tok.x == data)
                cct->setUpDirection(PxVec3(1.0f, 0.0f, 0.f));
            else if (tok.y == data)
                cct->setUpDirection(PxVec3(0.0f, 1.0f, 0.f));
            else if (tok.z == data)
                cct->setUpDirection(PxVec3(0.0f, 0.0f, 1.f));
        }
    }
    return true;
}

bool omni::physx::updateCctNonWalkableMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            if (!source)
                return true;
            omni::physics::parse::TokenId data;
            if (!getValue<omni::physics::parse::TokenId>(attachedStage, objectRecord->mKey, property, timeCode, data))
                return true;

            omni::physics::parse::KnownTokens tok;
            tok.intern(*source);

            if (tok.preventClimbing == data)
                cct->setNonWalkableMode(PxControllerNonWalkableMode::ePREVENT_CLIMBING);
            else if (tok.preventClimbingForceSliding == data)
                cct->setNonWalkableMode(PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING);
        }
    }
    return true;
}

bool omni::physx::updateCctClimbingMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTCct)
    {
        PxCapsuleController* cct = (PxCapsuleController*)objectRecord->mPtr;
        if (cct)
        {
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            if (!source)
                return true;
            omni::physics::parse::TokenId data;
            if (!getValue<omni::physics::parse::TokenId>(attachedStage, objectRecord->mKey, property, timeCode, data))
                return true;

            omni::physics::parse::KnownTokens tok;
            tok.intern(*source);

            if (tok.easy == data)
                cct->setClimbingMode(PxCapsuleClimbingMode::eEASY);
            else if (tok.constrained == data)
                cct->setClimbingMode(PxCapsuleClimbingMode::eCONSTRAINED);
        }
    }
    return true;
}
