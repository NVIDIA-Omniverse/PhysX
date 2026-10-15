// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4
#pragma once

#include "ContactReport.h"
#include "PhysXScene.h"
#include "usdLoad/AttachedStage.h"

namespace omni::physx::attachmentContact
{
// Private read: caller holds runtime admission and supplies valid output pointers.
inline uint32_t full(const usdparser::AttachedStage& owner, const ContactEventHeader** contactEventBuffer, const ContactData** contactDataBuffer, uint32_t& numContactData,
    const FrictionAnchor** frictionAnchorsDataBuffer, uint32_t& numFrictionAnchorsData)
{
    *contactEventBuffer = nullptr;
    *contactDataBuffer = nullptr;
    numContactData = 0;
    *frictionAnchorsDataBuffer = nullptr;
    numFrictionAnchorsData = 0;
    PhysXScene* physxScene = owner.getDefaultPhysXScene();
    if (!physxScene)
        return 0;

    const ContactReport* cr = physxScene->getContactReport();
    const ContactHeadersVector& contactHeaderVector = cr->getContactHeaderVector();
    if (contactHeaderVector.empty())
    {
        return 0;
    }

    const ContactDataVector& contactDataVector = cr->getContactDataVector();

    *contactEventBuffer = contactHeaderVector.data();
    numContactData = uint32_t(contactDataVector.size());
    if (numContactData)
    {
        *contactDataBuffer = contactDataVector.data();
    }

    const FrictionAnchorsDataVector& frictionAnchorsDataVector = cr->getFrictionAnchorsDataVector();
    numFrictionAnchorsData = uint32_t(frictionAnchorsDataVector.size());
    if (numFrictionAnchorsData)
    {
        *frictionAnchorsDataBuffer = frictionAnchorsDataVector.data();
    }

    return uint32_t(contactHeaderVector.size());
}

}
