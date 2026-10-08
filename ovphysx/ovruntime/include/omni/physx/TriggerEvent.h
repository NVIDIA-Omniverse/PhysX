// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-6 AC-7
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-43
 */
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/physics/AttachHandle.h>
#include <omni/physics/parse/Handles.h>

#include "EventSubscriptionRegistry.h"

namespace omni
{
namespace physx
{

/**
\brief Trigger event type
*/
struct TriggerEventType
{
    enum Enum
    {
        eTRIGGER_ON_ENTER, //!< Trigger event issued when a new shape enters the trigger volume
        eTRIGGER_ON_LEAVE, //!< Trigger event issued when a shape leaves the trigger volume
    };
};

/**
\brief Trigger Event Data passed to the callback
*/
struct TriggerEventData
{
    TriggerEventType::Enum eventType; //!< Type of event(enter / leave)
    uint64_t subscriptionId; //!< The subscription id returned when registering the callback
    AttachHandle attachHandle; //!< The attach where the trigger event happened. Equal to the handle
                               //!< passed to IPhysxSimulation::subscribePhysicsTriggerReportEvents,
                               //!< resolved: a subscription made with kActiveAttach reports the
                               //!< concrete handle it resolved to.
    omni::physics::parse::ObjectKey triggerColliderPrimKey; //!< The collider prim source of trigger event
    omni::physics::parse::ObjectKey otherColliderPrimKey; //!< The collider prim entering or leaving the trigger volume
    omni::physics::parse::ObjectKey triggerBodyPrimKey; //!< The body containing the collider that is source of trigger event
    omni::physics::parse::ObjectKey otherBodyPrimKey; //!< The body containing the collider that is entering or leaving the trigger volume
};

/**
\brief Trigger report event function

\param triggerData Trigger data event containing the attach, trigger and other prim and the event type (enter / leave)
\param userData User data that were registered during subscribe
*/
typedef void (*OnTriggerEventReportEventFn)(const TriggerEventData* triggerData, void* userData);

} // namespace physx
} // namespace omni
