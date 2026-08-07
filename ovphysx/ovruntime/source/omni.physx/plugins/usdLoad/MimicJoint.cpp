// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>

#include "LoadTools.h"

#include "MimicJoint.h"
#include "NewtonCompat.h"

#include <omni/physx/IPhysxSettings.h>
#include <OmniPhysX.h>

using namespace PXR_NS;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{



static ObjectType getObjectType(SchemaAPIFlag::Enum schemaAPIFlag)
{
    if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotXAPI)
        return eMimicJointRotX;
    else if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotYAPI)
        return eMimicJointRotY;
    else if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotZAPI)
        return eMimicJointRotZ;
    else if (schemaAPIFlag == SchemaAPIFlag::eNewtonMimicAPI)
        return eNewtonMimicJoint;
    else
    {
        CARB_LOG_WARN("Usd Physics: unexpected SchemaAPIFlag %d passed to mimic joint getObjectType; "
            "defaulting to eNewtonMimicJoint.", static_cast<int>(schemaAPIFlag));
        return eNewtonMimicJoint;
    }
}

static SchemaAPIFlag::Enum getSchemaAPIFlag(ObjectType objectType)
{
    if (objectType == eMimicJointRotX)
        return SchemaAPIFlag::eMimicJointRotXAPI;
    else if (objectType == eMimicJointRotY)
        return SchemaAPIFlag::eMimicJointRotYAPI;
    else if (objectType == eMimicJointRotZ)
        return SchemaAPIFlag::eMimicJointRotZAPI;
    else if (objectType == eNewtonMimicJoint)
        return SchemaAPIFlag::eNewtonMimicAPI;
    else
    {
        CARB_LOG_WARN("Usd Physics: unexpected ObjectType %d passed to mimic joint getSchemaAPIFlag; "
            "defaulting to eNewtonMimicAPI.", static_cast<int>(objectType));
        return SchemaAPIFlag::eNewtonMimicAPI;
    }
}

ObjectId createMimicJoint(AttachedStage& attachedStage, MimicJointDesc& desc)
{
    if (desc.mimicJointPath == desc.referenceJointPath)
    {
        CARB_LOG_ERROR(
            "Usd Physics: PhysxMimicJointAPI at %s has same joint for mimic and reference. The native ovruntime "
            "mimic path does not support using the same joint as both mimic and reference.",
            desc.mimicJointPath.GetText());
        return kInvalidObjectId;
    }

    ObjectDb* objectDb = attachedStage.getObjectDatabase();

    ObjectId mimicJointId = objectDb->findEntry(desc.mimicJointPath, eArticulationJoint);
    if (mimicJointId != kInvalidObjectId)
    {
        desc.mimicJointId = mimicJointId;

        ObjectId referenceJointId = objectDb->findEntry(desc.referenceJointPath, eArticulationJoint);
        if (referenceJointId != kInvalidObjectId)
        {
            desc.referenceJointId = referenceJointId;

            PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
            const ObjectId id = physInt->createObject(attachedStage, desc.mimicJointPath, desc);

            if (id != kInvalidObjectId)
            {
                objectDb->findOrCreateEntry(desc.mimicJointPath, desc.type, id);

                SchemaAPIFlag::Enum schemaAPIFlag = getSchemaAPIFlag(desc.type);
                objectDb->addSchemaAPI(desc.mimicJointPath, schemaAPIFlag);
            }

            return id;
        }
        else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
        {
            // scristiano: if in forced parsing single scene mode, the joints may have not been created
            CARB_LOG_ERROR("Usd Physics: failed to find internal joint object for reference joint at prim "
                "%s for PhysxMimicJointAPI at %s. Please ensure that the prim is a supported joint type and "
                "is part of an articulation.\n",
                desc.referenceJointPath.GetText(), desc.mimicJointPath.GetText());
        }
    }
    else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
    {
        // scristiano: if in forced parsing single scene mode, the joints may have not been created
        CARB_LOG_ERROR("Usd Physics: failed to find internal joint object for PhysxMimicJointAPI at %s. "
            "Please ensure that the prim is a supported joint type and is part of an articulation.\n",
            desc.mimicJointPath.GetText());
    }

    return kInvalidObjectId;
}

void releaseMimicJoint(AttachedStage& attachedStage, const PXR_NS::SdfPath& path,
    SchemaAPIFlag::Enum schemaAPIFlag)
{
    ObjectType type = getObjectType(schemaAPIFlag);

    ObjectDb* objectDb = attachedStage.getObjectDatabase();

    ObjectId mimicJointId = objectDb->findEntry(path, type);
    if (mimicJointId != kInvalidObjectId)
    {
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        physInt->releaseObject(attachedStage, path, mimicJointId);

        objectDb->removeSchemaAPI(path, schemaAPIFlag);

        objectDb->removeEntry(path, type, mimicJointId);
    }
}


} // namespace usdparser
} // namespace physx
} // namespace omni
