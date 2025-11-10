// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>


namespace omni
{

namespace physics
{

// Interface to mimic the requirements of current stage update nodes
//
// This should hopefully get removed in favor of physics simulation interface
// however the editor part still requires the stage attach to be ready during editor update
// so we keep this interface for now

// \note Currently only one stage can be attached, multiple stages are not yet supported
struct IPhysicsStageUpdate
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysicsStageUpdate", 0, 1)

    /// Called when a stage gets attached, does not load physics. Does just set internally stage.
    ///
    /// \param stageId Stage Id that should be attached
    void(CARB_ABI* onAttach)(long int stageId);

    /// Called when stage gets detached.
    void(CARB_ABI* onDetach)();

    /// Called when on stage update.
    ///
    /// \param currentTime Current time in seconds
    /// \param elapsedSec Elapsed time from previous update in seconds
    /// \param enableUpdate Enable physics update, physics can be disabled, but we still need to update other subsystems
    void(CARB_ABI* onUpdate)(float currentTime, float elapsedSecs, bool enableUpdate);

    /// Called when timeline play is requested.
    ///
    /// \param currentTime Current time in seconds
    void(CARB_ABI* onResume)(float currentTime);

    /// Called when timeline gets paused.
    void(CARB_ABI* onPause)();

    /// Called when timeline is stopped.
    void(CARB_ABI* onReset)();

    /// Called when a raycast request is executed - used for picking.
    ///
    /// \param oring Start position of the raycast
    /// \param dir Direction of the raycast
    /// \param[in] input Whether the input control is set or reset (e.g. mouse down).
    void(CARB_ABI* handleRaycast)(const float* orig, const float* dir, bool input);

    /// Called when a force load from USD is requested.
    // This will make sure that the physics engine creates internal objects for the stage that is attached.
    void(CARB_ABI* forceLoadPhysicsFromUSD)();

    /// Called when a release of physics objects is requested.
    void(CARB_ABI* releasePhysicsObjects)();

    /// Reset simulation will release all physics objects and reset the simulation, while
    /// keeping the stage attached.
    void(CARB_ABI* resetSimulation)();
};

} // namespace physics
} // namespace omni
