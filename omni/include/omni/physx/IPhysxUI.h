// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

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
    CARB_PLUGIN_INTERFACE("omni::physx::ui::IPhysxUI", 1, 0)

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
    /// \param bodyPath                     Body path
    /// \param attachmentInstanceName       TfToken instance of the tendons
    void(CARB_ABI* selectSpatialTendonAttachmentHelper)(const pxr::SdfPath bodyPath,
                                                        const pxr::TfToken attachmentInstanceName);

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

    /// Get all attachments that are associated with a given primitive.
    /// \param dstPathLengths       output path string lengths.
    /// \param dstPathLengthsSize   output number of path string lengths equals number of resulting attachment paths.
    /// \param dstPathData          output path string data.
    /// \param dstPathDataSize      output path string data byte size.
    /// \param primPath             input path to prim for which we get the associated attachments.
    /// \param allocateBytes        function to allocate memory for output
    void(CARB_ABI* getAttachments)(size_t*& dstPathLengths,
                                   uint32_t& dstPathLengthsSize,
                                   uint8_t*& dstPathData,
                                   size_t& dstPathDataSize,
                                   const pxr::SdfPath& primPath,
                                   void* (*allocateBytes)(size_t));

    /// Block USD notice handler.
    /// \param enable Enabled or disable the notice handler.
    void(CARB_ABI* blockUsdNoticeHandler)(bool enable);

    /// Check if USD notice handler is blocked.
    /// \return If the notice handler is enabled or not.
    bool(CARB_ABI* isUsdNoticeHandlerEnabled)();

    /// Refresh deformable attachment.
    ///
    /// \param attachmentPath   Sdf.Path to the attachment.
    void(CARB_ABI* refreshAttachment)(pxr::SdfPath attachmentPath);

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
