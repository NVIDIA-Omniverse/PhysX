// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

#include <PhysXTools.h>
#include <OmniPhysX.h>

#include "PhysXPropertiesUpdate.h"

#include "internal/InternalMimicJoint.h"


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;


static const InternalDatabase::Record* getObjectRecord(const AttachedStage& attachedStage, omni::physx::PhysXType type,
    omni::physx::usdparser::ObjectId objectId)
{
    omni::physx::internal::InternalPhysXDatabase& db = attachedStage.getInternalPhysXDatabase();

    return db.getFullTypedRecord(type, objectId);
}

static InternalMimicJoint* getInternalMimicJoint(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::ObjectKey& key)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(attachedStage, ePTMimicJoint, objectId);
    if (objectRecord)
    {
        key = objectRecord->mKey;
        return static_cast<InternalMimicJoint*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}


bool omni::physx::updateMimicJointGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setGearing(value);
    }

    return true;
}

bool omni::physx::updateMimicJointOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setOffset(value);
    }

    return true;
}

bool omni::physx::updateMimicJointNaturalFrequency(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setNaturalFrequency(value);
    }

    return true;
}

bool omni::physx::updateMimicJointDampingRatio(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setDampingRatio(value);
    }

    return true;
}

// Newton: joint0 = coef0 + coef1 * joint1  maps to PhysX gearing = -coef1, offset = -coef0.
bool omni::physx::updateNewtonMimicJointCoef1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setGearing(-value);
    }

    return true;
}

bool omni::physx::updateNewtonMimicJointCoef0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setOffset(-value);
    }

    return true;
}
