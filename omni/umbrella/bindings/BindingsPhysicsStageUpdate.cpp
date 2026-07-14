// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <carb/BindingsPythonUtils.h>
#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>

namespace
{
void bindPhysicsStageUpdate(pybind11::module& m)
{
    using namespace carb;
    using namespace omni::physics;

    // Bind SimulationEvent enum
    py::enum_<SimulationEvent>(m, "SimulationEvent", R"(
        Simulation events used by simulation event stream.
        )")
        .value("RESUMED", eResumed, "Simulation resumed, no additional data are sent in the event")
        .value("PAUSED", ePaused, "Simulation paused, no additional data are sent in the event")
        .value("STOPPED", eStopped, "Simulation stopped, no additional data are sent in the event")
        .export_values();

    // Define IPhysicsStageUpdate interface
    defineInterfaceClass<IPhysicsStageUpdate>(m, "IPhysicsStageUpdate", "acquire_physics_stage_update_interface", "release_physics_stage_update_interface", R"(
        Interface to mimic the requirements of current stage update nodes.
        Currently only one stage can be attached, multiple stages are not yet supported.
    )")
        .def("on_attach", wrapInterfaceFunction(&IPhysicsStageUpdate::onAttach), py::arg("stage_id"), R"(
            Called when a stage gets attached, does not load physics. Does just set internally stage.
            
            Args:
                stage_id (int): Stage Id that should be attached
        )")
        .def("on_detach", wrapInterfaceFunction(&IPhysicsStageUpdate::onDetach), R"(
            Called when stage gets detached.
        )")
        .def("on_update", wrapInterfaceFunction(&IPhysicsStageUpdate::onUpdate), py::arg("current_time"), py::arg("elapsed_secs"), py::arg("enable_update"), R"(
            Called when on stage update.
            
            Args:
                current_time (float): Current time in seconds
                elapsed_secs (float): Elapsed time from previous update in seconds
                enable_update (bool): Enable physics update, physics can be disabled, but we still need to update other subsystems
        )")
        .def("on_resume", wrapInterfaceFunction(&IPhysicsStageUpdate::onResume), py::arg("current_time"), R"(
            Called when timeline play is requested.
            
            Args:
                current_time (float): Current time in seconds
        )")
        .def("on_pause", wrapInterfaceFunction(&IPhysicsStageUpdate::onPause), R"(
            Called when timeline gets paused.
        )")
        .def("on_reset", wrapInterfaceFunction(&IPhysicsStageUpdate::onReset), R"(
            Called when timeline is stopped.
        )")
        .def("handle_raycast", 
            [](IPhysicsStageUpdate* self, py::object orig, py::object dir, bool input) {
                carb::Float3 orig_array = {0.0f, 0.0f, 0.0f};
                carb::Float3 dir_array = {0.0f, 0.0f, 0.0f};
                bool orig_valid = false;
                bool dir_valid = false;
                
                if (!orig.is_none()) {
                    auto float3 = orig.cast<carb::Float3>();
                    orig_array = float3;
                    orig_valid = true;
                }
                
                if (!dir.is_none()) {
                    auto float3 = dir.cast<carb::Float3>();
                    dir_array = float3;
                    dir_valid = true;
                }
                
                self->handleRaycast(orig_valid ? &orig_array.x : nullptr, dir_valid ? &dir_array.x : nullptr, input);
            },
            py::arg("orig").none(true), 
            py::arg("dir").none(true), 
            py::arg("input"), R"(
            Called when a raycast request is executed - used for picking.
            
            Args:
                orig (Optional[carb.Float3]): Start position of the raycast, can be None
                dir (Optional[carb.Float3]): Direction of the raycast, can be None
                input (bool): Whether the input control is set or reset (e.g. mouse down)
        )")
        .def("force_load_physics_from_usd", wrapInterfaceFunction(&IPhysicsStageUpdate::forceLoadPhysicsFromUSD), R"(
            Called when a force load from USD is requested.
            This will make sure that the physics engine creates internal objects for the stage that is attached.
        )")
        .def("release_physics_objects", wrapInterfaceFunction(&IPhysicsStageUpdate::releasePhysicsObjects), R"(
            Called when a release of physics objects is requested.
        )")
        .def("reset_simulation", wrapInterfaceFunction(&IPhysicsStageUpdate::resetSimulation), R"(
            Called when a reset of physics simulation is requested.
            This will release all physics objects and reset the simulation, while keeping the stage attached.
        )")
        .def("start_simulation", wrapInterfaceFunction(&IPhysicsStageUpdate::startSimulation), R"(
            Called when simulation is started.
            This can be used to store the initial transformations for example.
        )")
        .def("get_simulation_event_stream", wrapInterfaceFunction(&IPhysicsStageUpdate::getSimulationEventStream), R"(
            Simulation event stream sending various simulation events defined in SimulationEvent enum.

            Returns:
                Event stream sending the simulation events (eResumed, ePaused, eStopped).
        )");
}
} // namespace 
