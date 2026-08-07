// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physics/usd/PrimIterator.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXTools.h>
#include <PhysXCustomJoint.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/TypeCast.h>
#include "LoadUsd.h"
#include "LoadTools.h"
#include "Joint.h"
#include "PhysicsBody.h"

#include <OmniPhysX.h>
#include "AttributeHelpers.h"


using namespace PXR_NS;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static TfToken g_cone("cone");

ObjectId createJoint(AttachedStage& attachedStage, const SdfPath& primKey, PhysxJointDesc* desc, ObjectId body0, bool body0Dynamic,
    ObjectId body1, bool body1Dynamic)
{
    if (desc != nullptr)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        if (desc->jointEnabled && (body0 == kInvalidObjectId && body1 == kInvalidObjectId))
        {
            // scristiano: if in forced parsing single scene mode, the bodies may have not been created
            if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
            {
                REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - no bodies defined at body0 and body1, joint prim: %s", primKey.GetText());
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
                        REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primKey.GetText());
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
                        REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primKey.GetText());
                    }                    
                    return kInvalidObjectId;
                }
            }
            else if (desc->jointEnabled && (!body0Dynamic) && (!body1Dynamic))
            {
                REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primKey.GetText());                
                return kInvalidObjectId;
            }

            if ((body0 != kInvalidObjectId) && (body1 != kInvalidObjectId))
            {
                if (body0 == body1)
                {
                    REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - you cannot create a joint between a body and itself (both joint bodies must be unique) for joint prim: %s", primKey.GetText());                    
                    return kInvalidObjectId;
                }
            }

            if (!desc->validBodyTransformations)
            {
                CARB_LOG_WARN(
                    "PhysicsUSD: CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together: %s",
                    primKey.GetText());
            }

            const ObjectId id = physInt->createJoint(attachedStage, primKey, *desc, body0, body1);

            if (id != kInvalidObjectId)
            {
                objectDb->findOrCreateEntry(primKey, eJoint, id);             
            }
            return id;
        }        
    }

    return kInvalidObjectId;
}

PhysxJointDesc* createJointDesc(const UsdPrim& usdPrim)
{
    if (usdPrim.IsA<PhysxSchemaPhysxPhysicsGearJoint>())
    {
        return ICE_PLACEMENT_NEW(GearPhysxJointDesc)();
    }
    else if (usdPrim.IsA<PhysxSchemaPhysxPhysicsRackAndPinionJoint>())
    {
        return ICE_PLACEMENT_NEW(RackPhysxJointDesc)();
    }

    return nullptr;
}


} // namespace usdparser
} // namespace physx
} // namespace omni
