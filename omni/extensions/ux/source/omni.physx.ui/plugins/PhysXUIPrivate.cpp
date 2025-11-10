// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

// #define CARB_EXPORTS

#include <private/omni/physx/IPhysxUIPrivate.h>
#include "PhysXUIOmniUISceneOverlay.h"
#include "InputManager.h"
#include "DebugVisualization.h"

extern omni::physx::ui::InputManager* gInputManager;
extern omni::physx::ui::DebugVisualization* gDebugVisualization;

static std::shared_ptr<omni::ui::scene::PhysXUIOmniUISceneOverlay> createPhysXUIOmniUISceneOverlay()
{
    return omni::ui::scene::PhysXUIOmniUISceneOverlayImpl::create();
}

static omni::physx::ui::InputManager& getInputManager()
{
    return *gInputManager;
}

static bool testDebugVisInternalState(int phase)
{
    if (gDebugVisualization)
    {
        if (!gDebugVisualization->testInternalState(phase))
        {
            CARB_LOG_ERROR("testDebugVisInternalState: phase %i returned false!", phase);
            return false;
        }

        return true;
    }

    return false;
}

using namespace omni::physx::ui;
using namespace pxr;
void fillInterface(IPhysxUIPrivate& iface)
{
    iface.createPhysXUIOmniUISceneOverlay = &createPhysXUIOmniUISceneOverlay;
    iface.getInputManager = &getInputManager;
    iface.testDebugVisInternalState = &testDebugVisInternalState;
}
