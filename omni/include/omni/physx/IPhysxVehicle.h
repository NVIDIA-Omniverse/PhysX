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

struct IPhysxVehicle
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxVehicle", 4, 0)

    /// Get the state of the input system controlling the vehicle.
    ///
    /// \param[in] path      The USD path of the vehicle
    /// \return A boolean that states whether the inputs are being sent to the specified vehicle.
    bool(CARB_ABI* getInputEnabled)(const char* path);

    /// Set the state of the input system controlling the vehicle. Only one vehicle can have inputs enabled at a time,
    /// all other vehicle inputs will be disabled.
    ///
    /// \param[in] path      The USD path of the vehicle
    void(CARB_ABI* setInputEnabled)(const char* path, bool inputEnabled);

    /// Get the state of the input system controlling the vehicle.
    ///
    /// \param[in] path      The USD path of the vehicle
    /// \return A boolean that states whether the mouse inputs can control the specified vehicle.
    bool(CARB_ABI* getMouseEnabled)(const char* path);

    /// Set the state of the mouse input system controlling the vehicle. The mouse is used to interact with the UI, so
    /// it may interfere.
    ///
    /// \param[in] path      The USD path of the vehicle
    void(CARB_ABI* setMouseEnabled)(const char* path, bool mouseEnabled);

    /// Get the state of the vehicle's auto reverse system.
    ///
    /// \param[in] path      The USD path of the vehicle
    /// \return A boolean that states whether the auto reverse mode is enabled or disabled.
    bool(CARB_ABI* getAutoReverseEnabled)(const char* path);

    /// Set the state of the vehicle's auto reverse system. When enabled, holding the brake will stop the vehicle and
    /// after a slight pause, put the vehicle in reverse. While in reverse, holding the accelerator will brake the
    /// vehicle to a stop and after a slight pause, will put the vehicle back in drive.
    ///
    /// \param[in] path      The USD path of the vehicle
    void(CARB_ABI* setAutoReverseEnabled)(const char* path, bool autoReverseEnabled);

    /// Compute the Ackermann steering angle for an outer wheel.
    ///
    /// This helper method provides the Ackermann steering angle of the outer wheel for a given
    /// steer angle of the inner wheel (with respect to the turn direction). This helper method assumes
    /// the front axle has the steering wheels. The Ackermann corrected steer angle ensures that the
    /// perpendiculars to the front wheels' longitudinal directions cross the extended vector of the
    /// rear axle at the same point.
    ///
    /// \param[in] steerAngle      Steer angle (in radians) of the inner wheel (with respect to the turn direction).
    ///                            Value has to be in [-pi/2, pi/2].
    /// \param[in] axleSeparation  Distance between center of front axle and center of rear axle.
    /// \param[in] axleWidth       Distance between center-points of the two front wheels.
    ///
    /// \return The Ackermann steer angle of the outer wheel in radians.
    float(CARB_ABI* computeAckermannSteeringAngle)(const float steerAngle,
                                                   const float axleSeparation,
                                                   const float axleWidth);

    /// Attach to a USD stage explicitly. This will traverse the prims of the stage and create controllers
    /// as needed. This method should only be used if the controllers are going to be updated explicitly
    /// (see updateControllers()) instead of being triggered by the default stage update loop.
    ///
    /// \note: the currently attached stage will be detached.
    ///
    /// \param[in] id                         USD stageId (can be retrieved from a stagePtr -
    ///                                       pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    /// \param[in] unregisterFromStageUpdate  If true, the vehicle extension will not listen to stage update
    ///                                       events any longer.
    /// \return True if stage was successfully attached.
    bool(CARB_ABI* attachStage)(long id, bool unregisterFromStageUpdate);

    /// Detach from the USD stage. This will remove all vehicle related controller objects.
    ///
    /// \param[in] registerToStageUpdate  If true, the detach operation will be followed by a re-attach
    ///                                   and the vehicle extension will listen to stage update events again.
    void(CARB_ABI* detachStage)(bool registerToStageUpdate);

    /// Explicit update step for the vehicle controller logic (applying steer, brake, acceleration etc.).
    ///
    /// \param[in] elapsedSeconds      Elapsed time in seconds.
    void(CARB_ABI* updateControllers)(float elapsedSeconds);

    /// Compute sprung mass values for a given set of wheel positions.
    ///
    /// \note The method assumes that the provided positions enclose the origin. If this is not the case,
    ///       the setup will be ill-conditioned and the computation will fail.
    ///
    /// \param[in] mass           Total mass of the vehicle.
    /// \param[in] upAxisIndex    Index of the up-axis (0: x-axis, 1: y-axis, 2: z-axis).
    /// \param[in] positions      Wheel positions relative to the center of mass.
    /// \param[in] positionCount  Number of entries in the provided positions array.
    /// \param[out] sprungMasses  Sprung mass value for each provided position. The buffer needs to be able to
    ///                           hold at least positionCount entries.
    /// \return True on success, else false.
    bool(CARB_ABI* computeSprungMasses)(
        float mass, uint32_t upAxisIndex, const carb::Float3* positions, uint32_t positionCount, float* sprungMasses);

    /// Compute the local space suspension frame transform for wheel attachments given the prim transforms
    /// and the center of mass information.
    ///
    /// This helper method will set the USD suspensionFramePosition and suspensionFrameOrientation attributes of the
    /// PhysxVehicleWheelAttachmentAPI API schema such that the resulting world space transform will match the
    /// transform of the wheel attachment prim.
    ///
    /// \note: multiple requirements have to be fullfilled for this method to succeed. The vehicle prim and wheel
    ///        attachment prims have to inherit from UsdGeomXformable. The vehicle prim needs the API schema MassAPI
    ///        with the centerOfMass attribute defined.
    ///
    /// \param[in] path            Wheel attachment or vehicle USD path. If the path is a prim with PhysxVehicleAPI
    ///                            applied, all the wheel attachments of the vehicle will be processed.
    /// \return True on success, else false (error message will be sent to log).
    bool(CARB_ABI* computeSuspensionFrameTransforms)(const char* path);

    /// Return the vehicle's steering sensitivity when using the gamepad (see #setSteeringSensitivity for details).
    ///
    /// \param[in] path      The USD path of the vehicle
    /// \return A float specifying the sensitivity of the vehicle steering as a function of the gamepad input.
    float(CARB_ABI* getSteeringSensitivity)(const char* path);

    /// Set the vehicle's steering sensitivity when using the gamepad.
    ///
    /// Has to be 1.0 or greater. A setting of 1.0 generates a linear response between gamepad input and steering
    /// output. As the sensitivity is increased, the same gamepad input will generate a smaller steering output.
    /// However, regardless of the sensitivity, a full gamepad deflection will always generate a full steering
    /// deflection. steering deflection = sign(steering sensitivity) * power(gamepad input, steering sensitivity)
    ///
    /// \param[in] path                The USD path of the vehicle
    /// \param[in] steeringSensitivity The sensitivity of the vehicle steering as a function of the gamepad input
    ///                                (1.0 or greater).
    void(CARB_ABI* setSteeringSensitivity)(const char* path, float steeringSensitivity);

    /// Return the vehicle's steering filter time constant (see #setSteeringFilterTime for details).
    ///
    /// \param[in] path      The USD path of the vehicle
    /// \return A float that specifies the time constant of the steering input filter in seconds.
    float(CARB_ABI* getSteeringFilterTime)(const char* path);

    /// Set the vehicle's steering filter time constant.
    ///
    /// Has to be 0.0 or greater. A setting of 0.0 turns the filter off. The higher the value the longer
    /// it takes for the steering to reach the desired angle.
    ///
    /// \param[in] path                The USD path of the vehicle
    /// \param[in] steeringFilterTime  The time constant of the steering input filter in seconds (0.0 or greater).
    void(CARB_ABI* setSteeringFilterTime)(const char* path, float steeringFilterTime);
};

} // namespace physx
} // namespace omni
