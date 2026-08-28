// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdPCH.h"

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

#include <PhysXTools.h>
#include <OmniPhysX.h>

#include "PhysXPropertiesUpdate.h"

#include "internal/InternalMimicJoint.h"


using namespace ::physx;
using namespace carb;
using namespace PXR_NS;
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
    PXR_NS::SdfPath& path)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTMimicJoint, objectId);
    if (objectRecord)
    {
        path = attachedStage.pathFor(objectRecord->mKey);
        return static_cast<InternalMimicJoint*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}


bool omni::physx::updateMimicJointGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    PXR_NS::SdfPath path;
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
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    PXR_NS::SdfPath path;
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
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    PXR_NS::SdfPath path;
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
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    PXR_NS::SdfPath path;
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

// Newton: joint0 = coef0 + coef1 * joint1  maps to PhysX gearing = -coef1, offset = -coef0.
bool omni::physx::updateNewtonMimicJointCoef1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    PXR_NS::SdfPath path;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, path);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        internalMimicJoint->setGearing(-value);
    }

    return true;
}

bool omni::physx::updateNewtonMimicJointCoef0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    PXR_NS::SdfPath path;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, path);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        internalMimicJoint->setOffset(-value);
    }

    return true;
}
