// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/Function.h>


namespace omni
{
namespace physics
{


// Disable the reset on stop
//\param[in] disable Disable/enable the reset on stop
using DisableResetOnStopFn = std::function<void(bool disable)>;

// Get the current state of the reset on stop
//\return Disable/enable for the reset on stop
using IsDisabledResetOnStopFn = std::function<bool()>;

// Handle the raycast request from the interaction system
//\param[in] orig Start position of the raycast
//\param[in] dir Direction of the raycast
//\param[in] input Whether the input control is set or reset (e.g. mouse down)
using HandleRaycastFn = std::function<void(const float* orig, const float* dir, bool input)>;

struct InteractionFns
{
    InteractionFns() : disableResetOnStop(nullptr), isDisabledResetOnStop(nullptr), handleRaycast(nullptr)
    {
    }

    DisableResetOnStopFn disableResetOnStop;
    IsDisabledResetOnStopFn isDisabledResetOnStop;
    HandleRaycastFn handleRaycast;
};

} // namespace physics
} // namespace omni
