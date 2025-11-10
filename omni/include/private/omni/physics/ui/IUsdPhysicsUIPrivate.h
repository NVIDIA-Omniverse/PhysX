// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>


namespace omni
{
namespace physics
{
namespace ui
{

/// A private interface for physics extensions that need to be tightly coupled with omni.usdphysics.ui.
///
/// Subject to change without notice.
///
/// This interface should be considered internal to the omni.usdphysics and should not be used by external clients.
//  Clients should rely on public interfaces like IUsdPhysicsUI instead
///
struct IUsdPhysicsUIPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physics::ui::IUsdPhysicsUIPrivate", 0, 2)

    /// Draw joint overlays (to be called inside Manipulator::_predraw)
    /// NOTE: This will be deprecated as soon as all C++ drawing code will be ported to omni.ui.scene
    ///
    void(CARB_ABI* privateDrawImmediateModeViewportOverlays)(const double* view,
                                                             const double* proj,
                                                             float screenPositionX,
                                                             float screenPositionY,
                                                             float computedContentWidth,
                                                             float computedContentHeight,
                                                             float dpiScale,
                                                             void (*enablePickingFunction)(bool enable, void* userData),
                                                             void* userData);
};

} // namespace ui
} // namespace physics
} // namespace omni
