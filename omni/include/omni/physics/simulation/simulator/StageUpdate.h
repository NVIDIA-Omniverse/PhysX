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

// This should hopefully get removed in favor of physics simulation interface
// however the editor part still requires the stage attach to be ready during editor update
// so we keep this interface for now

// Called when a stage gets attached, does not load physics. Does just set internally stage.
//
// \param stageId Stage Id that should be attached
using OnAttachFn = std::function<void(long int stageId)>;

// Called when stage gets detached.
using OnDetachFn = std::function<void()>;

// Called when stage gets updated.
//
// \param currentTime Current time
// \param elapsedSecs Elapsed time since the last physics step
// \param enableUpdate Whether the update is enabled
using OnUpdateFn = std::function<void(float currentTime, float elapsedSecs, bool enableUpdate)>;

// Called when stage gets resumed.
//
// \param currentTime Current time
using OnResumeFn = std::function<void(float currentTime)>;

// Called when stage gets paused.
using OnPauseFn = std::function<void()>;

// Called when stage gets reset.
using OnResetFn = std::function<void()>;

// Handle the raycast request from the interaction system
//\param[in] orig Start position of the raycast
//\param[in] dir Direction of the raycast
//\param[in] input Whether the input control is set or reset (e.g. mouse down)
using HandleRaycastFn = std::function<void(const float* orig, const float* dir, bool input)>;

// Called when a force load from USD is requested.
// This will make sure that the physics engine creates internal objects for the stage that is attached.
using ForceLoadPhysicsFromUSDFn = std::function<void()>;

// Called when a release of physics objects is requested.
using ReleasePhysicsObjectsFn = std::function<void()>;

/// Reset simulation will release all physics objects and reset the simulation, while
/// keeping the stage attached.
using ResetSimulationFn = std::function<void()>;

struct StageUpdateFns
{
    StageUpdateFns()
        : onAttach(nullptr),
          onDetach(nullptr),
          onUpdate(nullptr),
          onResume(nullptr),
          onPause(nullptr),
          onReset(nullptr),
          handleRaycast(nullptr),
          forceLoadPhysicsFromUSD(nullptr),
          releasePhysicsObjects(nullptr),
          resetSimulation(nullptr)
    {
    }

    OnAttachFn onAttach;
    OnDetachFn onDetach;
    OnUpdateFn onUpdate;
    OnResumeFn onResume;
    OnPauseFn onPause;
    OnResetFn onReset;
    HandleRaycastFn handleRaycast;
    ForceLoadPhysicsFromUSDFn forceLoadPhysicsFromUSD;
    ReleasePhysicsObjectsFn releasePhysicsObjects;
    ResetSimulationFn resetSimulation;
};

} // namespace physics
} // namespace omni
