// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */

#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "FixedTendon.h"
// AttachedStage.h only forward-declares PhysXUsdPhysicsInterface; this TU calls
// into it (createObject), so it needs the real definition.
#include <usdInterface/UsdInterface.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
    // instanceToken/jointKey/link0/link1 on PhysxTendonAxisDesc/PhysxTendonFixedDesc
    // are ObjectKey/TokenId (ADR-0019 increment 7); TendonAxisMap is now
    // ObjectKey-keyed too (LoadTools.h), so this recursion takes/compares the
    // link-traversal key as ObjectKey directly -- no path round trip is needed
    // for either the map lookup or the directionality/instance-token match
    // tests. createObject/findOrCreateEntry have ObjectKey-native overloads
    // (UsdInterface.h/LoadTools.h), so the whole function is pxr-free.
    void createFixedTendonAxesRecursive(AttachedStage& attachedStage, const ObjectId& parentAxisId,
        omni::physics::parse::TokenId instanceToken, omni::physics::parse::ObjectKey currLinkKey, TendonAxisMap& tendonAxes)
    {
        for (TendonAxisMap::mapped_type::const_reference AxisRef : tendonAxes[currLinkKey])
        {
            if (AxisRef->wasVisited)
            {
                continue;
            }

            if (AxisRef->instanceToken == instanceToken)
            {
                // add parent Axis object Id to description to save runtime when retrieving it during creation
                AxisRef->parentAxisId = parentAxisId;
                // create tendon axis
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, AxisRef->jointKey, *AxisRef);

                if (id == kInvalidObjectId)
                {
                    // User has already been warned in createObject();
                    continue;
                }

                attachedStage.getObjectDatabase()->findOrCreateEntry(
                    AxisRef->jointKey, attachedStage.textFor(AxisRef->jointKey), eTendonAxis, id);
                AxisRef->wasVisited = true;

                // infer directionality and continue traversal in right direction
                if (currLinkKey == AxisRef->link0)
                {
                    createFixedTendonAxesRecursive(attachedStage, id, instanceToken, AxisRef->link1, tendonAxes);
                }
                else
                {
                    createFixedTendonAxesRecursive(attachedStage, id, instanceToken, AxisRef->link0, tendonAxes);
                }
            }
        }
    }

    void createFixedTendons(AttachedStage& attachedStage, TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons)
    {
        ObjectDb* objDb = attachedStage.getObjectDatabase();

        // PRECONDITION: Root axis must be the common ancestor to all other tendon axes with respect to articulation hierarchy
        for (FixedTendonVector::reference tendonRef : fixedTendons)
        {
            const omni::physics::parse::ObjectKey rootJointKey = tendonRef->rootAxis->jointKey;

            // create tendon (also creates dummy axis)
            const ObjectId tendonRootId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootJointKey, *tendonRef);
            objDb->findOrCreateEntry(rootJointKey, attachedStage.textFor(rootJointKey), eTendonFixed, tendonRootId);

            if (tendonRootId == kInvalidObjectId)
            {
                continue;
            }

            // create root tendon axis
            tendonRef->rootAxis->parentAxisId = tendonRootId;
            const ObjectId tendonAxisId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootJointKey, *(tendonRef->rootAxis));
            tendonRef->rootAxis->wasVisited = true;

            if (tendonRootId == kInvalidObjectId)
            {
                // User has already been warned in createObject();
                continue;
            }

            objDb->findOrCreateEntry(rootJointKey, attachedStage.textFor(rootJointKey), eTendonAxis, tendonAxisId);

            // articulation hierarchy not known, try both directions of joint link refs:
            createFixedTendonAxesRecursive(attachedStage, tendonAxisId, tendonRef->instanceToken, tendonRef->rootAxis->link0, tendonAxes);
            createFixedTendonAxesRecursive(attachedStage, tendonAxisId, tendonRef->instanceToken, tendonRef->rootAxis->link1, tendonAxes);
        }

        // give out warnings for unparsed tendon axes
        for (TendonAxisMap::reference AxesVec : tendonAxes)
        {
            for (TendonAxisMap::mapped_type::reference Axis : AxesVec.second)
            {
                if (!Axis->wasVisited)
                {
                    CARB_LOG_WARN("The fixed tendon axis at %s was not parsed due to a topology issue: Refer to the topology constraints in the USD schema doc for PhysxTendonAxisAPI.",
                        attachedStage.textFor(Axis->jointKey));
                    Axis->wasVisited = true;
                }
            }
        }
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
