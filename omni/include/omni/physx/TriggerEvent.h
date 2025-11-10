// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

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
    uint64_t stageId; //!< The stage where trigger event happend
    uint64_t triggerColliderPrimId; //!< The collider prim source of trigger event
    uint64_t otherColliderPrimId; //!< The collider prim entering or leaving the trigger volume
    uint64_t triggerBodyPrimId; //!< The body containing the collider that is source of trigger event
    uint64_t otherBodyPrimId; //!< The body containing the collider that is entering or leaving the trigger volume
};

/**
\brief Trigger report event function

\param triggerData Trigger data event containing stage, trigger and other prim and the event type (enter / leave)
\param userData User data that were registered during subscribe
*/
typedef void (*OnTriggerEventReportEventFn)(const TriggerEventData* triggerData, void* userData);

} // namespace physx
} // namespace omni
