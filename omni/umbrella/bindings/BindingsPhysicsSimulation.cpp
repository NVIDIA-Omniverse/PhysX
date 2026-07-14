// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <carb/Framework.h>
#include <carb/BindingsPythonUtils.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>

namespace py = pybind11;

namespace
{
void bindPhysicsSimulation(py::module& m)
{
    using namespace carb;
    using namespace omni::physics;

    // IPhysicsSimulation interface
    defineInterfaceClass<IPhysicsSimulation>(m, "IPhysicsSimulation", "acquire_physics_simulation_interface", "release_physics_simulation_interface")
        .def("initialize", wrapInterfaceFunction(&IPhysicsSimulation::initialize),
             py::arg("id"),
             "Initialize physics simulation with a USD stage. This will run the physics parser and will populate the simulation with the corresponding simulation objects.\n\n"
             "Note: previous stage will be closed.\n\n"
             "Args:\n"
             "    id (int): USD stageId (can be retrieved from a stagePtr - PXR_NS::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())\n\n"
             "Returns:\n"
             "    bool: True if stage was successfully initialized.")
        .def("close", wrapInterfaceFunction(&IPhysicsSimulation::close),
             "Close the simulation, this will remove all objects from the simulation")
        .def("get_attached_stage", wrapInterfaceFunction(&IPhysicsSimulation::getAttachedStage),
             "Gets the currently attached USD stage.\n\n"
             "Returns:\n"
             "    int: USD stageId, 0 means no stage is attached.")
        .def("simulate_async", wrapInterfaceFunction(&IPhysicsSimulation::simulateAsync),
             py::arg("elapsed_time"),
             py::arg("current_time"),
             "Execute physics simulation asynchronously\n\n"
             "The simulation will simulate the exact elapsedTime passed. No substepping will happen.\n"
             "It is the caller's responsibility to provide reasonable elapsedTime.\n"
             "In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second\n\n"
             "Args:\n"
             "    elapsedTime (float): Simulation time in seconds.\n"
             "    currentTime (float): Current time, might be used for time sampled transformations to apply.")
        .def("simulate", wrapInterfaceFunction(&IPhysicsSimulation::simulate),
             py::arg("elapsed_time"),
             py::arg("current_time"),
             "Execute physics simulation synchronously\n\n"
             "The simulation will simulate the exact elapsedTime passed and wait for results before returning.\n"
             "No substepping will happen. It is the caller's responsibility to provide reasonable elapsedTime.\n"
             "In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second\n\n"
             "Args:\n"
             "    elapsedTime (float): Simulation time in seconds.\n"
             "    currentTime (float): Current time, might be used for time sampled transformations to apply.")
        .def("fetch_results", wrapInterfaceFunction(&IPhysicsSimulation::fetchResults),
             "Fetch simulation results.\n"
             "Writing out simulation results based on physics settings.\n\n"
             "Note: This is a blocking call. The function will wait until the simulation is finished.")
        .def("check_results", wrapInterfaceFunction(&IPhysicsSimulation::checkResults),
             "Check if simulation finished.\n\n"
             "Returns:\n"
             "    bool: True if simulation finished.")
        .def("flush_changes", wrapInterfaceFunction(&IPhysicsSimulation::flushChanges),
             "Flush changes will force physics to process buffered changes\n\n"
             "Changes to physics gets buffered, in some cases flushing changes is required if order is required.\n\n"
             "Example - prim A gets added. Existing prim B has a relationship that gets switched to use A. Currently,\n"
             "the relationship change gets processed immediately and fails because prim A only gets added at the\n"
             "start of the next sim step.")
        .def("pause_change_tracking", wrapInterfaceFunction(&IPhysicsSimulation::pauseChangeTracking),
             py::arg("pause"),
             "Pause change tracking for physics listener\n\n"
             "Args:\n"
             "    pause (bool): Pause or resume the change tracking")
        .def("is_change_tracking_paused", wrapInterfaceFunction(&IPhysicsSimulation::isChangeTrackingPaused),
             "Check if fabric change tracking for physics listener is paused or not\n\n"
             "Args:\n"
             "    simulation_id: Simulation ID\n"
             "Returns:\n"
             "    bool: True if change tracking is paused")
        .def(
            "subscribe_physics_contact_report_events",
            [](IPhysicsSimulation& self, OnContactReportEventFn onEvent) {

                using namespace std::placeholders;

                return carb::createPySubscription(
                    std::move(onEvent),
                    [self](auto fn, void* userData) -> SubscriptionId {
                        auto wrap = [fn, userData](const omni::physics::ContactEventHeaderVector& eventHeaders,
                                                   const omni::physics::ContactDataVector& contactData,
                                                   const omni::physics::FrictionAnchorsDataVector& frictionAnchors) {
                            fn(eventHeaders, contactData, frictionAnchors, userData);
                        };
                        return self.subscribePhysicsContactReportEvents(wrap);
                    },
                    [self](SubscriptionId id) {
                        // Release the GIL since unsubscribe can block on a mutex and deadlock
                        py::gil_scoped_release gsr;
                        self.unsubscribePhysicsContactReportEvents(id);
                    });
            },
            py::arg("contact_report_fn"),            
            "Subscribe to physics simulation contact report events.\n\n"
            "Note: The contact buffer data are available for one simulation step.\n\n"
            "Args:\n"
            "    contact_report_fn: The callback function to be called on contact report.\n"
            "    userData: Optional user data to be passed back in the callback function.\n\n"
            "Returns:\n"
            "    int: Subscription Id for release")
        .def("get_simulation_time_steps_per_second", wrapInterfaceFunction(&IPhysicsSimulation::getSimulationTimeStepsPerSecond),
             py::arg("simulation_id"),
             py::arg("stage_id"),
             py::arg("scene_path"),
             "Get physics simulation time steps per second.\n\n"
             "Args:\n"
             "    simulation_id: Simulation ID\n"
             "    stage_id (int): Stage id\n"
             "    scene_path (int): Returns the time steps for given scene if 0 is passed returns the first found scene stepping\n\n"
             "Returns:\n"
             "    int: Current time steps per second")
        .def("get_simulation_timestamp", wrapInterfaceFunction(&IPhysicsSimulation::getSimulationTimestamp),
             py::arg("simulation_id"),
             "Get physics simulation timestamp.\n\n"
             "Timestamp will increase with every simulation step.\n\n"
             "Args:\n"
             "    simulation_id: Simulation ID\n\n"
             "Returns:\n"
             "    int: Current timestamp")
        .def("get_simulation_step_count", wrapInterfaceFunction(&IPhysicsSimulation::getSimulationStepCount),
             py::arg("simulation_id"),
             "Get the number of physics steps performed in the active simulation.\n\n"
             "The step count resets to 0 when a new simulation starts.\n\n"
             "Args:\n"
             "    simulation_id: Simulation ID\n\n"
             "Returns:\n"
             "    int: Number of steps since the currently active simulation started or 0 if there is no active simulation")
        .def(
            "subscribe_physics_on_step_events",
            [](IPhysicsSimulation& self, bool preStep, int order, OnPhysicsStepEventFn onUpdate) {
                using namespace std::placeholders;

                return carb::createPySubscription(
                    std::move(onUpdate),
                    [self, preStep, order](auto fn, void* userData) -> SubscriptionId {
                        auto wrap = [fn, userData](float elapsed, const omni::physics::PhysicsStepContext& context) {
                            fn(elapsed, context, userData);
                        };
                        return self.subscribePhysicsOnStepEvents(preStep, order, wrap);
                    },
                    [self](SubscriptionId id) {
                        // Release the GIL since unsubscribe can block on a mutex and deadlock
                        py::gil_scoped_release gsr;
                        self.unsubscribePhysicsOnStepEvents(id);
                    });
            },
            py::arg("pre_step"), py::arg("order"), py::arg("on_update"),
            "Subscribe to physics pre/post step events.\n\n"
            "Note: Subscriptions cannot be changed in the onUpdate callback\n\n"
            "Args:\n"
            "    preStep (bool): Whether to execute this callback right *before* the physics step event. If this is false, the\n"
            "                   callback will be executed right *after* the physics step event.\n"
            "    order (int): An integer value used to order the callbacks: 0 means \"highest priority\", 1 is \"less priority\" and so on.\n"
            "    onUpdate: The callback function to be called on update.\n"
            "    userData: The userData to be passed back in the callback function.\n\n"
            "Returns:\n"
            "    int: Subscription Id for release, returns kInvalidSubscriptionId if failed")
        .def(
            "is_capable_of_simulating",
            [](IPhysicsSimulation& self, SimulationId simulationId, const std::vector<std::string>& schemaNames) {
                std::vector<const char*> schemaNamePtrs;
                schemaNamePtrs.reserve(schemaNames.size());
                for (const auto& name : schemaNames)
                {
                    schemaNamePtrs.push_back(name.c_str());
                }
                std::unique_ptr<bool[]> isCapableOut(new bool[schemaNames.size()]);
                bool success = self.isCapableOfSimulating(
                    simulationId, schemaNamePtrs.data(), schemaNamePtrs.size(), isCapableOut.get());
                if (!success)
                {
                    return py::make_tuple(false, py::list());
                }
                py::list result;
                for (size_t i = 0; i < schemaNames.size(); ++i)
                {
                    result.append(isCapableOut[i]);
                }
                return py::make_tuple(true, result);
            },
            py::arg("simulation_id"), py::arg("schema_names"),
            "Check if simulation is capable of simulating given schema types or schema APIs.\n\n"
            "Args:\n"
            "    simulation_id: Simulation ID to check the capabilities for.\n"
            "    schema_names (list): List of schema names to check, can be a schema API name or a schema type name.\n\n"
            "Returns:\n"
            "    tuple: (success, capabilities) where success is True if the operation was successful and the simulation\n"
            "           is able to check the capabilities, and capabilities is a list of booleans indicating if the\n"
            "           simulation is capable of simulating each schema name.");
}
} // namespace

void bindPhysicsSimulation(py::module& m); 
