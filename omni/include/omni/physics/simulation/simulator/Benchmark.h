// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/Function.h>
#include "Simulation.h"

namespace omni
{
namespace physics
{    


// Profile stats
struct PhysicsProfileStats
{
    std::string zoneName;   // Name of the zone
    float ms;               // Milliseconds
};

// Callback function to be called on event.
//
// \param profileStats The profile stats.
using ProfileStatsNotificationFn = std::function<void(const std::vector<PhysicsProfileStats>& profileStats)>;

// Subscribe to physics benchmark events.
//
// \note Subscription cannot be changed in the onEvent callback
//
// \param onEvent The callback function to be called on event.
// \return Subscription Id for release, returns kInvalidSubscriptionId if failed
using SubscribeProfileStatsEventsFn =
    std::function<SubscriptionId(ProfileStatsNotificationFn onEvent)>;

// Unsubscribes to physics benchmark events.
//
// \note Subscription cannot be changed in the onEvent callback
//
// subscriptionId SubscriptionId obtained via @ref subscribeProfileStatsEvents.
using UnsubscribeProfileStatsEventsFn = std::function<void(SubscriptionId subscriptionId)>;

struct BenchmarkFns
{
    BenchmarkFns() : subscribeProfileStatsEvents(nullptr), unsubscribeProfileStatsEvents(nullptr)
    {
    }

    SubscribeProfileStatsEventsFn subscribeProfileStatsEvents;
    UnsubscribeProfileStatsEventsFn unsubscribeProfileStatsEvents;
};

} // namespace physics
} // namespace omni
