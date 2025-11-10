// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// CCT
bool omni::physx::updateCctSlopeLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setSlopeLimit(data);
        }
    }
    return true;
}

bool omni::physx::updateCctHeight(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
        if (!getValue<double>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxCapsuleController* cct = (PxCapsuleController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setHeight(float(data));
        }
    }
    return true;
}

bool omni::physx::updateCctRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
        if (!getValue<double>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxCapsuleController* cct = (PxCapsuleController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setRadius(float(data));
        }
    }
    return true;
}

bool omni::physx::updateCctContactOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setContactOffset(data);
        }
    }
    return true;
}

bool omni::physx::updateCctStepOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxController* cct = (PxController*)objectRecord->mPtr;
        if (cct)
        {
            cct->setStepOffset(data);
        }
    }
    return true;
}

bool omni::physx::updateCctUpAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
            TfToken data;
            if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
                return true;

            if (UsdPhysicsTokens.Get()->x == data)
                cct->setUpDirection(PxVec3(1.0f, 0.0f, 0.f));
            else if (UsdPhysicsTokens.Get()->y == data)
                cct->setUpDirection(PxVec3(0.0f, 1.0f, 0.f));
            else if (UsdPhysicsTokens.Get()->z == data)
                cct->setUpDirection(PxVec3(0.0f, 0.0f, 1.f));
        }
    }
    return true;
}

bool omni::physx::updateCctNonWalkableMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
            TfToken data;
            if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
                return true;

            if (PhysxSchemaTokens.Get()->preventClimbing == data)
                cct->setNonWalkableMode(PxControllerNonWalkableMode::ePREVENT_CLIMBING);
            else if (PhysxSchemaTokens.Get()->preventClimbingForceSliding == data)
                cct->setNonWalkableMode(PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING);
        }
    }
    return true;
}

bool omni::physx::updateCctClimbingMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
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
            TfToken data;
            if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
                return true;

            if (PhysxSchemaTokens.Get()->easy == data)
                cct->setClimbingMode(PxCapsuleClimbingMode::eEASY);
            else if (PhysxSchemaTokens.Get()->constrained == data)
                cct->setClimbingMode(PxCapsuleClimbingMode::eCONSTRAINED);
        }
    }
    return true;
}
