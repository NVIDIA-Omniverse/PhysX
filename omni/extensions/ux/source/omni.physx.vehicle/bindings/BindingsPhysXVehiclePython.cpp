// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physx/IPhysxVehicle.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.vehicle.python")

namespace
{

PYBIND11_MODULE(_physxVehicle, m)
{
    const char* docString;

    using namespace carb;
    using namespace omni::physx;

    m.doc() = "pybind11 carb.physx.vehicle bindings";

    py::class_<IPhysxVehicle> physxVehicle = defineInterfaceClass<IPhysxVehicle>(
        m, "IPhysxVehicle", "acquire_physx_vehicle_interface", "release_physx_vehicle_interface");

    m.def("release_physx_vehicle_interface_scripting", [](IPhysxVehicle* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    docString = R"(
        Return whether input control of the vehicle specified by the USD path is enabled or disabled.

        Args:
            path: Vehicle USD path.

        Returns:
            bool: input control of the vehicle is enabled or disabled.
    )";
    physxVehicle.def("get_input_enabled", wrapInterfaceFunction(&IPhysxVehicle::getInputEnabled), docString, py::arg("path"));

    docString = R"(
        Enable input control of the vehicle specified by the USD path. Input control of all other vehicles will be disabled.

        Args:
            path: Vehicle USD path.
            inputEnabled: Enable/disable device input.
    )";
    physxVehicle.def("set_input_enabled", wrapInterfaceFunction(&IPhysxVehicle::setInputEnabled), docString,
        py::arg("path"), py::arg("inputEnabled"));

    docString = R"(
        Return whether mouse control of the vehicle specified by the USD path is enabled or disabled.

        Args:
            path: Vehicle USD path.

        Returns:
            bool: mouse control of the vehicle is enabled or disabled.
    )";
    physxVehicle.def("get_mouse_enabled", wrapInterfaceFunction(&IPhysxVehicle::getMouseEnabled), docString, py::arg("path"));

    docString = R"(
        Enable mouse control of the vehicle specified by the USD path.

        Args:
            path: Vehicle USD path.
            mouseEnabled: Enable/disable mouse control.
    )";
    physxVehicle.def("set_mouse_enabled", wrapInterfaceFunction(&IPhysxVehicle::setMouseEnabled), docString,
        py::arg("path"),  py::arg("mouseEnabled"));

    docString = R"(
        Return whether reverse gear can be automatically selected by holding the brake input.

        Args:
            path: Vehicle USD path.

        Returns:
            bool: Auto reverse of the vehicle is enabled or disabled.
    )";
    physxVehicle.def("get_auto_reverse_enabled", wrapInterfaceFunction(&IPhysxVehicle::getAutoReverseEnabled), docString, py::arg("path"));

    docString = R"(
        Enable auto reverse control of the vehicle specified by the USD path. Hold brake to reverse, hold the accelerator to drive.

        Args:
            path: Vehicle USD path.
            autoReverseEnabled: Enable/disable auto reverse.
    )";
    physxVehicle.def("set_auto_reverse_enabled", wrapInterfaceFunction(&IPhysxVehicle::setAutoReverseEnabled), docString,
        py::arg("path"), py::arg("autoReverseEnabled"));

    docString = R"(
        Compute the Ackermann steering angle for an outer wheel.

        This helper method provides the Ackermann steering angle of the outer wheel for a given
        steer angle of the inner wheel (with respect to the turn direction). This helper method assumes
        the front axle has the steering wheels. The Ackermann corrected steer angle ensures that the
        perpendiculars to the front wheels' longitudinal directions cross the extended vector of the
        rear axle at the same point.

        Args:
            steerAngle: Steer angle (in radians) of the inner wheel (with respect to the turn direction).
                        Value has to be in [-pi/2, pi/2].
            axleSeparation: Distance between center of front axle and center of rear axle.
            axleWidth: Distance between center-points of the two front wheels.

        Returns:
            The Ackermann steer angle of the outer wheel in radians.
    )";
    physxVehicle.def("compute_ackermann_steering_angle",
                     wrapInterfaceFunction(&IPhysxVehicle::computeAckermannSteeringAngle), docString,
                     py::arg("steerAngle"), py::arg("axleSeparation"), py::arg("axleWidth"));

    docString = R"(
        Attach to a USD stage explicitly. This will traverse the prims of the stage and create controllers
        as needed. This method should only be used if the controllers are going to be updated explicitly
        (see update_controllers()) instead of being triggered by the default stage update loop.

        Note:
            The currently attached stage will be detached.

        Args:
            id: USD stageId (can be retrieved from a stage - UsdUtils.StageCache.Get().GetId(stage).ToLongInt())
            unregisterFromStageUpdate: If true, the vehicle extension will not listen to stage update
                                       events any longer.

        Returns:
            True if stage was successfully attached.
    )";
    physxVehicle.def(
        "attach_stage", wrapInterfaceFunction(&IPhysxVehicle::attachStage), docString, py::arg("id"),
        py::arg("unregisterFromStageUpdate"));

    docString = R"(
        Detach from the USD stage. This will remove all vehicle related controller objects.

        Args:
            registerToStageUpdate: If true, the detach operation will be followed by a re-attach
                                   and the vehicle extension will listen to stage update events again.
    )";
    physxVehicle.def(
        "detach_stage", wrapInterfaceFunction(&IPhysxVehicle::detachStage), docString,
        py::arg("registerToStageUpdate"));

    docString = R"(
        Explicit update step for the vehicle controller logic (applying steer, brake, acceleration etc.).

        Args:
            elapsedSeconds: Elapsed time in seconds.
    )";
    physxVehicle.def("update_controllers", wrapInterfaceFunction(&IPhysxVehicle::updateControllers),
                     docString, py::arg("elapsedSeconds"));

    docString = R"(
        Compute sprung mass values for a given set of wheel positions.

        Note:
            The method assumes that the provided positions enclose the origin. If this is not the case,
            the setup will be ill-conditioned, the computation will fail and an empty list is returned.

        Args:
            mass: Total mass of the vehicle.
            upAxisIndex: Index of the up-axis (0: x-axis, 1: y-axis, 2: z-axis).
            positions: List of wheel positions relative to the center of mass. Position entries need to be
                       provided as carb.Float3.

        Returns:
            List with sprung mass value for each provided position entry. If an error occurs, the list will
            be empty.
    )";
    physxVehicle.def("compute_sprung_masses",
        [](const IPhysxVehicle* physXVehicle, float mass, uint32_t upAxisIndex, py::list positions)
            {
                const uint32_t posCount = static_cast<uint32_t>(positions.size());
                carb::Float3* posBuffer = reinterpret_cast<carb::Float3*>(alloca(sizeof(carb::Float3) * posCount));
                float* sprungMassValues = reinterpret_cast<float*>(alloca(sizeof(float) * posCount));

                uint32_t i = 0;
                for (py::handle obj : positions)
                {
                    carb::Float3 pos = obj.cast<carb::Float3>();
                    posBuffer[i] = pos;

                    i++;
                }

                py::list sprungMasses;

                if (physXVehicle->computeSprungMasses(mass, upAxisIndex, posBuffer, posCount,
                    sprungMassValues))
                {
                    for (i = 0; i < posCount; i++)
                    {
                        sprungMasses.append(sprungMassValues[i]);
                    }
                }

                return sprungMasses;
            },
        docString,
        py::arg("mass"), py::arg("upAxisIndex"), py::arg("positions"));

    docString = R"(
        Compute the local space suspension frame transform for wheel attachments given the prim transforms
        and the center of mass information.

        This helper method will set the USD suspensionFramePosition and suspensionFrameOrientation attributes of the
        PhysxVehicleWheelAttachmentAPI API schema such that the resulting world space transform will match the
        transform of the wheel attachment prim.

        Note:
            Multiple requirements have to be fullfilled for this method to succeed. The vehicle prim and wheel
            attachment prims have to inherit from UsdGeomXformable. The vehicle prim needs the API schema MassAPI
            with the centerOfMass attribute defined.

        Args:
            path: Wheel attachment or vehicle USD path. If the path is a prim with PhysxVehicleAPI applied,
                  all the wheel attachments of the vehicle will be processed.

        Returns:
            True on success, else False (error message will be sent to log).
    )";
    physxVehicle.def("compute_suspension_frame_transforms",
                     wrapInterfaceFunction(&IPhysxVehicle::computeSuspensionFrameTransforms), docString, py::arg("path"));

    docString = R"(
        Return the vehicle's steering sensitivity when using the gamepad (see set_steering_sensitivity() for details).

        Args:
            path: Vehicle USD path.

        Returns:
            float: The sensitivity of the vehicle steering as a function of the gamepad input.
    )";
    physxVehicle.def("get_steering_sensitivity", wrapInterfaceFunction(&IPhysxVehicle::getSteeringSensitivity), docString, py::arg("path"));

    docString = R"(
        Set the vehicle's steering sensitivity when using the gamepad. Must be 1.0 or greater.
        A setting of 1.0 generates a linear response between gamepad input and steering output.
        As the sensitivity is increased, the same gamepad input will generate a smaller steering output.
        However, regardless of the sensitivity, a full gamepad deflection will always generate a full steering deflection.
        steering deflection = sign(steering sensitivity) * power(gamepad input, steering sensitivity)

        Args:
            path: Vehicle USD path.
            steeringSensitivity: The sensitivity of the vehicle steering as a function of the gamepad input (1.0 or greater).
    )";
    physxVehicle.def("set_steering_sensitivity", wrapInterfaceFunction(&IPhysxVehicle::setSteeringSensitivity), docString,
        py::arg("path"), py::arg("steeringSensitivity"));

    docString = R"(
        Return the vehicle's steering filter time (see set_steering_filter_time() for details).

        Args:
            path: Vehicle USD path.

        Returns:
            float: The filter time of the steering input filter in seconds.
    )";
    physxVehicle.def("get_steering_filter_time", wrapInterfaceFunction(&IPhysxVehicle::getSteeringFilterTime), docString, py::arg("path"));

    docString = R"(
        Set the vehicle's steering filter time constant which must be 0.0 or greater. A setting of 0.0 turns the filter off.
        The higher the value the longer it takes for the steering to reach the desired angle. 

        Args:
            path: Vehicle USD path.
            steeringFilterTime: The filter time of the steering input filter in seconds (0.0 or greater).
    )";
    physxVehicle.def("set_steering_filter_time", wrapInterfaceFunction(&IPhysxVehicle::setSteeringFilterTime), docString,
        py::arg("path"), py::arg("steeringFilterTime"));
}

} // namespace
