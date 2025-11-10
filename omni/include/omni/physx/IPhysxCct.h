// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/IEvents.h>
#include <carb/input/InputTypes.h>


namespace omni
{
namespace physx
{

// Cct events
struct CctEvent
{
    enum Enum
    {
        eCollisionDown, //< send when collision down changes
        eCollisionUp, //< send when collision up changes
        eCollisionSides, //< send when collision sliding changes
    };
};


struct IPhysxCct
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCct", 2, 0)

    /// Enable first person camera support for a controller. Hides mouse cursor, sets the controller's
    /// capsule as a guide and uses the controller's camera transformation when transforming move vector
    /// in local space mode.
    ///
    /// \param cctPath Path of the controller's prim.
    /// \param camera_path Camera path
    void(CARB_ABI* enableFirstPerson)(const char* cctPath, const char* cameraPath);

    /// Disable first person camera support.
    ///
    /// \param cctPath Path of the controller's prim.
    void(CARB_ABI* disableFirstPerson)(const char* cctPath);

    /// Applies current PhysicsScene's gravity to the controller.
    ///
    /// \param cctPath Path of the controller's prim.
    void(CARB_ABI* enableGravity)(const char* cctPath);

    /// Applies custom gravity to the controller
    ///
    /// \param cctPath Path of the controller's prim. gravity
    /// \param gravity Custom gravity vector.
    void(CARB_ABI* enableCustomGravity)(const char* cctPath, const carb::Double3& gravity);

    /// Disables gravity set through apply_gravity or apply_custom_gravity.
    ///
    /// \param cctPath Path of the controller's prim.
    void(CARB_ABI* disableGravity)(const char* cctPath);

    /// Gets if gravity is being added to the controller's move vector.
    ///
    /// \param cctPath Path of the controller's prim.
    bool(CARB_ABI* hasGravityEnabled)(const char* cctPath);

    /// Sets if CharacterControllerAPI : MoveTarget attribute will be considered as a local or
    /// world space vector.Local space move vector is transformed with the controller's capsule
    /// transformation (or camera transformation in the case of first person mode).
    ///
    /// \param use World space is used when true, local space if false.
    void(CARB_ABI* enableWorldSpaceMove)(const char* cctPath, bool use);

    /// Gets controller's height.
    ///
    /// \param cctPath Path of the controller's prim.
    float(CARB_ABI* getControllerHeight)(const char* cctPath);

    /// Set the controller's height. This does not check for collisions.
    ///
    /// \param cctPath Path of the controller's prim.
    /// \param val New controller height.
    void(CARB_ABI* setControllerHeight)(const char* cctPath, float val);

    /// Set the position of the center of the controller's collision. This does not check for collisions.
    ///
    /// \param cctPath Path of the controller's prim.
    /// \param position New center position of the controller.
    void(CARB_ABI* setPosition)(const char* cctPath, const carb::Double3& position);

    /// Adds character controller to the manager. Use if not calling any of the other methods at least once.
    ///
    /// \param cctPath Path of the controller's prim.
    void(CARB_ABI* activateCct)(const char* cctPath);

    /// Removes a character controller's data from the manager.
    ///
    /// \param cctPath Path of the controller's prim.
    void(CARB_ABI* removeCct)(const char* cctPath);

    /// Move a controller by a given vector each frame (internally sets CharacterControllerAPI:MoveTarget). The
    /// vector is transformed from local to world space by either the controller's capsule or camera (if in
    /// first person mode) transformation. Use enableWorldSpaceMove to skip the local->world transform.
    ///
    /// \param cctPath Path of the controller's prim.
    /// \param cctPath Displacement vector.
    void(CARB_ABI* setMove)(const char* cctPath, const carb::Float3& displacement);

    /// Simulation event stream sending various simulation events defined in SimulationEvent enum.
    ///
    /// \return Event stream sending the simulation events.
    carb::events::IEventStreamPtr(CARB_ABI* getCctEventStream)();
};

} // namespace physx
} // namespace omni
