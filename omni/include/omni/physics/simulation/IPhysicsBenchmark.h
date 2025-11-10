// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "simulator/Benchmark.h"

namespace omni
{

namespace physics
{

struct IPhysicsBenchmarks
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysicsBenchmarks", 0, 1)

    /// Subscribe to physics simulation profile stats event.
    ///
    /// \note Subscription cannot be changed in the onEvent callback
    /// \note That if subscription is used the getProfileStats will not return any
    /// results as after the subscription send the results are cleared.
    ///
    /// \param onEvent The callback function to be called with the profile data.
    /// \param userData The userData to be passed back in the callback function.
    /// \return Subscription Id for release, kInvalidSubscriptionId is returned if the opetation failed
    SubscriptionId(CARB_ABI* subscribeProfileStatsEvents)(ProfileStatsNotificationFn onEvent);

    /// Unsubscribes to simulation events.
    ///
    /// \note Subscription cannot be changed in the onEvent callback
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribeProfileStatsEvents.
    void(CARB_ABI* unsubscribeProfileStatsEvents)(SubscriptionId subscriptionId);
};


} // namespace physics
} // namespace omni
