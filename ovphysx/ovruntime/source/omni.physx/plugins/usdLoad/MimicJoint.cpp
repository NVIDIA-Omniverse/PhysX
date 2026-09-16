// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-LOAD-OBJECTDB-001
 * @covers AC-4
 */

#include <carb/logging/Log.h>

#include "LoadTools.h"

#include "MimicJoint.h"

#include <omni/physx/IPhysxSettings.h>
#include <OmniPhysX.h>

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
    if (desc.mimicJointKey == desc.referenceJointKey)
    {
        CARB_LOG_ERROR(
            "Usd Physics: PhysxMimicJointAPI at %s has same joint for mimic and reference. The native ovruntime "
            "mimic path does not support using the same joint as both mimic and reference.",
            attachedStage.textFor(desc.mimicJointKey));
        return kInvalidObjectId;
    }

    ObjectDb* objectDb = attachedStage.getObjectDatabase();

    ObjectId mimicJointId = objectDb->findEntry(desc.mimicJointKey, eArticulationJoint);
    if (mimicJointId != kInvalidObjectId)
    {
        desc.mimicJointId = mimicJointId;

        ObjectId referenceJointId = objectDb->findEntry(desc.referenceJointKey, eArticulationJoint);
        if (referenceJointId != kInvalidObjectId)
        {
            desc.referenceJointId = referenceJointId;

            PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
            const ObjectId id = physInt->createObject(attachedStage, desc.mimicJointKey, desc);

            if (id != kInvalidObjectId)
            {
                SchemaAPIFlag::Enum schemaAPIFlag = getSchemaAPIFlag(desc.type);
                // The ObjectKey+pathText overload also registers the path in the path-keyed
                // PrimHierarchyStorage, which the tensor wildcard matcher and the replicator's
                // subtree scan read. releaseMimicJoint's removeEntry below undoes both sides.
                objectDb->findOrCreateEntry(desc.mimicJointKey, attachedStage.textFor(desc.mimicJointKey), desc.type, id);
                // addSchemaAPI(ObjectKey) only touches mKeySchemaAPIMap, never the path-keyed
                // mSchemaAPIMap -- but no live reader needs these mimic flags from the
                // path-keyed side: ChangeRegister.cpp/PrimUpdate.cpp only check
                // eDeformableBodyAPI there, and PhysXReplicator.cpp's schema-flags read already
                // ORs in the ObjectKey side. Unconditional, ObjectKey-native.
                objectDb->addSchemaAPI(desc.mimicJointKey, schemaAPIFlag);
            }

            return id;
        }
        else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
        {
            // scristiano: if in forced parsing single scene mode, the joints may have not been created
            CARB_LOG_ERROR("Usd Physics: failed to find internal joint object for reference joint at prim "
                "%s for PhysxMimicJointAPI at %s. Please ensure that the prim is a supported joint type and "
                "is part of an articulation.\n",
                attachedStage.textFor(desc.referenceJointKey), attachedStage.textFor(desc.mimicJointKey));
        }
    }
    else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
    {
        // scristiano: if in forced parsing single scene mode, the joints may have not been created
        CARB_LOG_ERROR("Usd Physics: failed to find internal joint object for PhysxMimicJointAPI at %s. "
            "Please ensure that the prim is a supported joint type and is part of an articulation.\n",
            attachedStage.textFor(desc.mimicJointKey));
    }

    return kInvalidObjectId;
}

void releaseMimicJoint(AttachedStage& attachedStage, omni::physics::parse::ObjectKey mimicJointKey,
    SchemaAPIFlag::Enum schemaAPIFlag)
{
    ObjectType type = getObjectType(schemaAPIFlag);

    ObjectDb* objectDb = attachedStage.getObjectDatabase();

    ObjectId mimicJointId = objectDb->findEntry(mimicJointKey, type);
    if (mimicJointId != kInvalidObjectId)
    {
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
        physInt->releaseObject(attachedStage, mimicJointKey, mimicJointId);

        // ObjectKey-native, unconditional -- see createMimicJoint's addSchemaAPI comment above
        // for why the path-keyed mSchemaAPIMap side is dead for these flags.
        objectDb->removeSchemaAPI(mimicJointKey, schemaAPIFlag);

        // removeEntry undoes createMimicJoint's findOrCreateEntry, path-keyed
        // PrimHierarchyStorage row included, without a path argument: ObjectDb remembers what
        // creation registered and evicts it itself (REQ-LOAD-OBJECTDB-001). Here the row
        // normally survives anyway -- the mimic API sits on a joint prim that keeps its own
        // eArticulationJoint entry at this key -- but that is the call site's luck, not its
        // doing.
        objectDb->removeEntry(mimicJointKey, type, mimicJointId);
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
