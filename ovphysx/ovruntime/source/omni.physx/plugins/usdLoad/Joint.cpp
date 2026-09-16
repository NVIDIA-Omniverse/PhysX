// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <omni/physx/IPhysxSettings.h>
#include "LoadUsd.h"
#include "LoadTools.h"
#include "Joint.h"
// OmniPhysX.h is pxr-free; needed for OmniPhysX::getInstance() in createJoint's body.
#include <OmniPhysX.h>

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

ObjectId createJoint(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, PhysxJointDesc* desc, ObjectId body0, bool body0Dynamic,
    ObjectId body1, bool body1Dynamic)
{
    if (desc != nullptr)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
        // Diagnostic-only text form (no SdfPath materialized): attachedStage.textFor never
        // builds a real path, just resolves the source's own string form of primKey.
        const char* primPathText = attachedStage.textFor(primKey);

        if (desc->jointEnabled && (body0 == kInvalidObjectId && body1 == kInvalidObjectId))
        {
            // scristiano: if in forced parsing single scene mode, the bodies may have not been created
            if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
            {
                REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - no bodies defined at body0 and body1, joint prim: %s", primPathText);
            }
        }
        else
        {
            if (desc->jointEnabled && body0 != kInvalidObjectId && body1 == kInvalidObjectId)
            {
                if (!body0Dynamic)
                {
                    // scristiano: if in forced parsing single scene mode, the bodies may have not been created
                    if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
                    {
                        REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primPathText);
                    }
                    return kInvalidObjectId;
                }
            }
            else if (desc->jointEnabled && body1 != kInvalidObjectId && body0 == kInvalidObjectId)
            {
                if (!body1Dynamic)
                {
                    // scristiano: if in forced parsing single scene mode, the bodies may have not been created
                    if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
                    {
                        REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primPathText);
                    }
                    return kInvalidObjectId;
                }
            }
            else if (desc->jointEnabled && (!body0Dynamic) && (!body1Dynamic))
            {
                REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primPathText);
                return kInvalidObjectId;
            }

            if ((body0 != kInvalidObjectId) && (body1 != kInvalidObjectId))
            {
                if (body0 == body1)
                {
                    REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - you cannot create a joint between a body and itself (both joint bodies must be unique) for joint prim: %s", primPathText);
                    return kInvalidObjectId;
                }
            }

            if (!desc->validBodyTransformations)
            {
                CARB_LOG_WARN(
                    "PhysicsUSD: CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together: %s",
                    primPathText);
            }

            const ObjectId id = physInt->createJoint(attachedStage, primKey, *desc, body0, body1);

            if (id != kInvalidObjectId)
            {
                // Use the path-text overload (not the bare-ObjectKey one), so the joint is
                // registered in PrimHierarchyStorage too -- required for structural resync
                // (e.g. a body0/body1 relationship re-target) to find and rebuild it.
                objectDb->findOrCreateEntry(primKey, primPathText, eJoint, id);
            }
            return id;
        }
    }

    return kInvalidObjectId;
}


} // namespace usdparser
} // namespace physx
} // namespace omni
