// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-26
 */

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/physics/parse/Handles.h>

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace ui
{

struct PhysXVehicleVisualizationParameter
{
    enum Enum
    {
        eSuspension = 0
    };
};


struct IPhysxUI
{
    /// PhysX SDK debug visualization distance
    ///
    /// \param distance         Distance
    void(CARB_ABI* setVisualizationDistance)(float distance);

    /// Enable PhysX SDK debug visualization
    ///
    /// \param enable         Enable/disable
    void(CARB_ABI* enableDebugVisualization)(bool enable);

    /// Tendons attachment helper selection
    ///
    /// \param bodyKey                      Body object key
    /// \param attachmentInstanceName       Interned instance name of the tendons (multi-apply schema instance, not an object)
    void(CARB_ABI* selectSpatialTendonAttachmentHelper)(const omni::physics::parse::ObjectKey bodyKey,
                                                        const omni::physics::parse::TokenId attachmentInstanceName);

    /// Tendons visualization filter
    ///
    /// \param instanceName         TfToken instance of the tendons
    void(CARB_ABI* setTendonVisualizationFilter)(const char* instanceName);

    /// Vehicle debug visualization enable/disable
    ///
    /// \param type         PhysXVehicleVisualizationParameter type
    /// \param enable       Enable/disable
    void(CARB_ABI* setVehicleVisualization)(PhysXVehicleVisualizationParameter::Enum type, bool enable);

    /// Get current vehicle debug visualization enable/disable
    ///
    /// \param type         PhysXVehicleVisualizationParameter type
    /// \return             return if its enabled/disabled
    bool(CARB_ABI* getVehicleVisualization)(PhysXVehicleVisualizationParameter::Enum type);

    /// Collision mesh visualization type
    ///
    /// \param type         Type of visualization (both, collision_only, graphics_only)
    void(CARB_ABI* setCollisionMeshType)(const char* type);

    /// Enabled collision mesh visualization
    ///
    /// \param enable         Enabled/disable the visualization feature
    void(CARB_ABI* enableCollisionMeshVisualization)(bool enable);

    /// Explode view distance for collision mesh visualization
    ///
    /// \param distance         Distance
    void(CARB_ABI* explodeViewDistance)(float distance);

    /// UI update - exposed to python
    void(CARB_ABI* update)();

    /// Block USD notice handler.
    /// \param enable Enabled or disable the notice handler.
    void(CARB_ABI* blockUsdNoticeHandler)(bool enable);

    /// Check if USD notice handler is blocked.
    /// \return If the notice handler is enabled or not.
    bool(CARB_ABI* isUsdNoticeHandlerEnabled)();

    /// Refresh deformable attachment.
    ///
    /// \param attachmentKey   Object key of the attachment.
    void(CARB_ABI* refreshAttachment)(omni::physics::parse::ObjectKey attachmentKey);

    /// Sets the camera position for the UI.
    ///
    /// \param pos   Camera world position.
    void(CARB_ABI* setCameraPos)(const carb::Float3& pos);

    /// Enable / disable redraw optimizations
    ///
    /// \param enable         Enable/disable
    void(CARB_ABI* enableRedrawOptimizations)(bool enable);
};

} // namespace ui
} // namespace physx
} // namespace omni
