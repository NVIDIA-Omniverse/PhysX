// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <private/omni/physx/IPhysxStageUpdate.h>

#include <carb/BindingsPythonUtils.h>


void PhysXStageUpdateBindings(pybind11::module& m)
{
    using namespace pybind11;
    using namespace carb;
    using namespace omni::physx;

    const char* docString;

    auto physxStageUpdate = defineInterfaceClass<IPhysxStageUpdate>(
        m, "IPhysxStageUpdate", "acquire_physx_stage_update_interface", "release_physx_stage_update_interface");

    m.def("release_physx_stage_update_interface_scripting", [](IPhysxStageUpdate* iface) { carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); });  // OM-60917

    docString = R"(
        Called when a stage gets attached, does not load physics. Does just set internally stage.

        Args:
            stage_id: Stage Id that should be attached
    )";
    physxStageUpdate.def("on_attach", wrapInterfaceFunction(&IPhysxStageUpdate::onAttach), docString,
        py::arg("stage_id"));

    docString = R"(
        Called again when a stage gets attached, but at a later point, where fabric stage is already created.

        Args:
            stage_id: Stage Id that should be attached
    )";
    physxStageUpdate.def("on_fabric_attach", wrapInterfaceFunction(&IPhysxStageUpdate::onFabricAttach), docString,
        py::arg("stage_id"));

    docString = R"(
        Called when stage gets detached.
    )";
    physxStageUpdate.def("on_detach", wrapInterfaceFunction(&IPhysxStageUpdate::onDetach), docString);

    docString = R"(
        Called when on stage update.

        Args:
            current_time: Current time in seconds
            elapsed_secs: Elapsed time from previous update in seconds
            enable_update: Enable physics update, physics can be disabled, but we still need to update other subsystems
    )";
    physxStageUpdate.def("on_update", wrapInterfaceFunction(&IPhysxStageUpdate::onUpdate), docString,
        py::arg("current_time"), py::arg("elapsed_secs"), py::arg("enable_update"));

    docString = R"(
        Called when timeline play is requested.

        Args:
            current_time: Current time in seconds
    )";
    physxStageUpdate.def("on_resume", wrapInterfaceFunction(&IPhysxStageUpdate::onResume), docString,
        py::arg("current_time"));

    docString = R"(
        Called when timeline gets paused.
    )";
    physxStageUpdate.def("on_pause", wrapInterfaceFunction(&IPhysxStageUpdate::onPause), docString);

    docString = R"(
        Called when timeline is stopped.
    )";
    physxStageUpdate.def("on_reset", wrapInterfaceFunction(&IPhysxStageUpdate::onReset), docString);
}
