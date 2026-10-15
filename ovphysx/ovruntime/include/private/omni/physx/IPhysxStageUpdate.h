// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-IO-001
// @covers AC-2
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/physics/AttachHandle.h>
#include <omni/physx/RuntimeResult.h>

namespace omni
{
namespace physx
{

// Interface to mimic the requirements of current stage update nodes
//
// Every stage operation selects the returned attachment explicitly. onUpdate is
// synchronous. Independent USD property queries have a separate process pump.
struct IPhysxStageUpdate
{
    /// Called when a stage gets attached, does not load physics. Does just set internally stage.
    ///
    /// \param stageId Stage Id that should be attached
    AttachHandle(CARB_ABI* onAttach)(long int stageId);

    /// Called when stage gets detached.
    RuntimeResult(CARB_ABI* onDetach)(AttachHandle attach);

    /// Called when on stage update.
    ///
    /// \param currentTime Current time in seconds
    /// \param elapsedSec Elapsed time from previous update in seconds
    /// \param enableUpdate Enable physics update, physics can be disabled, but we still need to update other subsystems
    RuntimeResult(CARB_ABI* onUpdate)(AttachHandle attach, float currentTime, float elapsedSecs, bool enableUpdate);

    /// Called when timeline play is requested.
    ///
    /// \param currentTime Current time in seconds
    RuntimeResult(CARB_ABI* onResume)(AttachHandle attach, float currentTime);

    /// Called when timeline gets paused.
    RuntimeResult(CARB_ABI* onPause)(AttachHandle attach);

    /// Called when timeline is stopped.
    RuntimeResult(CARB_ABI* onReset)(AttachHandle attach);

    /// Process-wide pump for explicitly queued USD property queries. Requires idle runtime.
    RuntimeResult(CARB_ABI* updatePropertyQueries)();

    /// Called when a raycast request is executed - used for picking.
    ///
    /// \param oring Start position of the raycast
    /// \param dir Direction of the raycast
    /// \param[in] input Whether the input control is set or reset (e.g. mouse down).
    void(CARB_ABI* handleRaycast)(omni::physics::AttachHandle attach, const float* orig, const float* dir, bool input);
};

} // namespace physx
} // namespace omni
