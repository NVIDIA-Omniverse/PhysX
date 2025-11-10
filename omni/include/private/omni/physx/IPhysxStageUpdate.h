// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

// Interface to mimic the requirements of current stage update nodes
//
// \note Currently only one stage can be attached, multiple stages are not yet supported
struct IPhysxStageUpdate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxStageUpdate", 0, 1)

    /// Called when a stage gets attached, does not load physics. Does just set internally stage.
    ///
    /// \param stageId Stage Id that should be attached
    void(CARB_ABI* onAttach)(long int stageId);

    /// Called again when a stage gets attached, but at a later point, where fabric stage is already created.
    /// This is a bit unfortunate, but we do not know when the stage gets created and if, therefore we try it at the end.
    ///
    /// \param stageId Stage Id that should be attached
    void(CARB_ABI* onFabricAttach)(long int stageId);

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
};

} // namespace physx
} // namespace omni
