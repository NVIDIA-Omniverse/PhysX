// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/dictionary/IDictionary.h>
#include "simulator/Simulation.h"

namespace omni
{

namespace physics
{

struct IPhysicsInteraction
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysicsInteraction", 0, 1)

    /// Controls the behavior of the ResetOnStop setting.
    ///
    /// \param[in] disable      Disable/enable the reset on stop override.
    void(CARB_ABI* disableResetOnStop)(bool disable);

    /// Returns the current state of the ResetOnStop setting.
    ///
    /// \param simulationId simulation id for simulation to query
    ///
    /// \return Disable/enable for the reset on stop.
    bool(CARB_ABI* isDisabledResetOnStop)(SimulationId simId);

    /// Called when a raycast request is executed - used for picking.
    ///
    /// \param oring Start position of the raycast
    /// \param dir Direction of the raycast
    /// \param[in] input Whether the input control is set or reset (e.g. mouse down).
    void(CARB_ABI* handleRaycast)(const float* orig, const float* dir, bool input);

    /// Get debug data for a prim
    ///
    /// \param[in] primPath The prim path as a string (e.g., "/World/Cube")
    /// \return List of engine-defined debug data entries
    carb::dictionary::Item*(CARB_ABI* getPrimDebugData)(const char* primPath);
};

} // namespace physics
} // namespace omni
