// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/ui/scene/Manipulator.h>

OMNIUI_SCENE_NAMESPACE_OPEN_SCOPE
class PhysXUIOmniUISceneOverlay : public omni::ui::scene::Manipulator
{
public:
    void setEnablePickingFn(std::function<void(bool)> enablePickingFn)
    {
        mEnablePickingFn = enablePickingFn;
    }

protected:
    std::function<void(bool)> mEnablePickingFn;
};
OMNIUI_SCENE_NAMESPACE_CLOSE_SCOPE

namespace omni
{
namespace physx
{
namespace ui
{

class InputManager;

/// A private interface for physics extensions that need to be tightly coupled with omni.physx.
///
/// Subject to change without notice.
///
/// This interface should be considered internal to the omni.physx.ui extension and
/// should not be used by external clients.  Clients should rely on public interfaces IPhysxUI
///
struct IPhysxUIPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::ui::IPhysxUIPrivate", 0, 2)

    std::shared_ptr<omni::ui::scene::PhysXUIOmniUISceneOverlay>(CARB_ABI* createPhysXUIOmniUISceneOverlay)();

    InputManager&(CARB_ABI* getInputManager)();

    bool(CARB_ABI* testDebugVisInternalState)(int phase);
};

} // namespace ui
} // namespace physx
} // namespace omni
