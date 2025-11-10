// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physx/IPhysxCamera.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.camera.python")

namespace
{

PYBIND11_MODULE(_physxCamera, m)
{
    const char* docString;

    using namespace carb;
    using namespace omni::physx;

    m.doc() = "pybind11 carb.physx.camera bindings";

    py::class_<IPhysxCamera> physxCamera = defineInterfaceClass<IPhysxCamera>(
        m, "IPhysxCamera", "acquire_physx_camera_interface", "release_physx_camera_interface");

    m.def("release_physx_camera_interface_scripting", [](IPhysxCamera* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    docString = R"(
        Create a chase camera that follows the subject specified by the USD path.

        Args:
            subjectPath: Camera USD path.
            cameraPath: USD path for camera to add.

        Returns:
            bool: True if successful, else False (for example, cameraPath pointing to existing prim).
    )";
    physxCamera.def("add_follow_look_camera", wrapInterfaceFunction(&IPhysxCamera::addFollowLookCamera), docString,
        py::arg("subjectPath"), py::arg("cameraPath"));

    docString = R"(
        Create a chase camera that follows the subject velocity vector specified by the USD path.

        Args:
            subjectPath: Camera USD path.
            cameraPath: USD path for camera to add.

        Returns:
            bool: True if successful, else False (for example, cameraPath pointing to existing prim).
    )";
    physxCamera.def("add_follow_velocity_camera", wrapInterfaceFunction(&IPhysxCamera::addFollowVelocityCamera), docString,
        py::arg("subjectPath"), py::arg("cameraPath"));

    docString = R"(
        Create a drone camera that follows the subject specified by the USD path.

        Args:
            subjectPath: Camera USD path.
            cameraPath: USD path for camera to add.

        Returns:
            bool: True if successful, else False (for example, cameraPath pointing to existing prim).
    )";
    physxCamera.def("add_drone_camera", wrapInterfaceFunction(&IPhysxCamera::addDroneCamera), docString,
        py::arg("subjectPath"), py::arg("cameraPath"));

    docString = R"(
        Attach to a USD stage explicitly. This will traverse the prims of the stage and create controllers
        as needed. This method should only be used if the controllers are going to be updated explicitly
        (see update()) instead of being tirggered by the default stage update loop.

        Note:
            The currently attached stage will be detached.

        Args:
            id: USD stageId (can be retrieved from a stage - UsdUtils.StageCache.Get().GetId(stage).ToLongInt())
            unregisterFromStageUpdate: If true, the extension will not listen to stage update
                                       events any longer.

        Returns:
            True if stage was successfully attached.
    )";
    physxCamera.def(
        "attach_stage", wrapInterfaceFunction(&IPhysxCamera::attachStage), docString, py::arg("id"),
        py::arg("unregisterFromStageUpdate"));

    docString = R"(
        Detach from the USD stage. This will remove all camera controller objects.

        Args:
            registerToStageUpdate: If true, the detach operation will be followed by a re-attach
                                   and the extension will listen to stage update events again.
    )";
    physxCamera.def(
        "detach_stage", wrapInterfaceFunction(&IPhysxCamera::detachStage), docString,
        py::arg("registerToStageUpdate"));

    docString = R"(
        Explicit pre-update step for cameras tracking both physics and animated cameras when not using the Play
        feature in Kit or when manually stepping the physics simulation.
    )";
    physxCamera.def("preUpdate", wrapInterfaceFunction(&IPhysxCamera::preUpdate), docString);

    docString = R"(
        Explicit update step for the animated cameras when not using the Play feature in Kit.
        Cameras that are tracking rigid body objects will get updated by the simulation event callback system.

        Args:
            elapsedSeconds: Elapsed time in seconds.
    )";
    physxCamera.def("update", wrapInterfaceFunction(&IPhysxCamera::update), docString, py::arg("elapsedSeconds"));

    docString = R"(
        Explicit call to process buffered USD changes (for example, adding camera after simulation has started).

        This method should only be used in combination with update(), when explicitly stepping
        the simulation. The required order of operation is:

        1. preUpdate() and/or update()
        2. simulate/fetch_results() or update_simulation() (omni.physx)
        3. process_pending_usd_changes()
    )";
    physxCamera.def("process_pending_usd_changes", wrapInterfaceFunction(&IPhysxCamera::processPendingUSDChanges));
}

} // namespace
