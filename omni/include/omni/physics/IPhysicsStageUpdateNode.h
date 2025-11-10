// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physics
{

// Interface to control whether omni.physics.stageupdate creates
// StageUpdateNode to connect to IStageUpdate or not
struct IPhysicsStageUpdateNode
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysicsStageUpdateNode", 0, 1)

    /// Attach StageUpdateNode to IStageUpdate
    void(CARB_ABI* attachNode)();

    /// Detach StageUpdateNode from IStageUpdate
    void(CARB_ABI* detachNode)();

    /// Check if StageUpdateNode is attached to IStageUpdate
    bool(CARB_ABI* isNodeAttached)();

    /// Block events from timeline (play, pause, stop)
    void(CARB_ABI* blockTimeLineEvents)(bool val);

    /// Check if we are blocking events from timeline (play, pause, stop)
    bool(CARB_ABI* timeLineEventsBlocked)();
};

} // namespace physics
} // namespace omni
