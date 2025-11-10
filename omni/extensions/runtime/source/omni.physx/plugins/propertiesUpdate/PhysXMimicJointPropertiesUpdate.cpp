// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

#include <PhysXTools.h>
#include <OmniPhysX.h>

#include "PhysXPropertiesUpdate.h"

#include "internal/InternalMimicJoint.h"


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;


static const InternalDatabase::Record* getObjectRecord(omni::physx::PhysXType type,
    omni::physx::usdparser::ObjectId objectId)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    return db.getFullTypedRecord(type, objectId);
}

static InternalMimicJoint* getInternalMimicJoint(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    pxr::SdfPath& path)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTMimicJoint, objectId);
    if (objectRecord)
    {
        path = objectRecord->mPath;
        return static_cast<InternalMimicJoint*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}


bool omni::physx::updateMimicJointGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, path);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        internalMimicJoint->setGearing(value);
    }

    return true;
}

bool omni::physx::updateMimicJointOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, path);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        internalMimicJoint->setOffset(value);
    }

    return true;
}

bool omni::physx::updateMimicJointNaturalFrequency(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, path);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        internalMimicJoint->setNaturalFrequency(value);
    }

    return true;
}

bool omni::physx::updateMimicJointDampingRatio(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, path);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        internalMimicJoint->setDampingRatio(value);
    }

    return true;
}
