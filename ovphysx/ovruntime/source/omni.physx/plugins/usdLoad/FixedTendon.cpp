// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include <ChangeRegister.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "FixedTendon.h"
#include "AttributeHelpers.h"


using namespace PXR_NS;

namespace omni
{
namespace physx
{
namespace usdparser
{
    void createFixedTendonAxesRecursive(AttachedStage& attachedStage, const ObjectId& parentAxisId,
        const PXR_NS::TfToken& instanceToken, const PXR_NS::SdfPath& currLinkPath, TendonAxisMap& tendonAxes)
    {
        for (TendonAxisMap::mapped_type::const_reference AxisRef : tendonAxes[currLinkPath])
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
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, AxisRef->jointPath, *AxisRef);

                if (id == kInvalidObjectId)
                {
                    // User has already been warned in createObject();
                    continue;
                }

                attachedStage.getObjectDatabase()->findOrCreateEntry(AxisRef->jointPath, eTendonAxis, id);
                AxisRef->wasVisited = true;

                // infer directionality and continue traversal in right direction
                if (currLinkPath == AxisRef->link0)
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
            const SdfPath rootJointPath = tendonRef->rootAxis->jointPath;

            // create tendon (also creates dummy axis)
            const ObjectId tendonRootId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootJointPath, *tendonRef);
            objDb->findOrCreateEntry(rootJointPath, eTendonFixed, tendonRootId);

            if (tendonRootId == kInvalidObjectId)
            {
                continue;
            }

            // create root tendon axis
            tendonRef->rootAxis->parentAxisId = tendonRootId;
            const ObjectId tendonAxisId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootJointPath, *(tendonRef->rootAxis));
            tendonRef->rootAxis->wasVisited = true;

            if (tendonRootId == kInvalidObjectId)
            {
                // User has already been warned in createObject();
                continue;
            }

            objDb->findOrCreateEntry(rootJointPath, eTendonAxis, tendonAxisId);

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
                        Axis->jointPath.GetText());
                    Axis->wasVisited = true;
                }
            }
        }
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
