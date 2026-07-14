// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <carb/Framework.h>
#include <carb/BindingsPythonUtils.h>

#include <omni/physics/simulation/simulator/Simulator.h>
#include <omni/physics/simulation/simulator/Simulation.h>
#include <omni/physics/simulation/simulator/ContactEvent.h>
#include <omni/physics/simulation/simulator/StageUpdate.h>

#include <omni/physics/simulation/IPhysics.h>
#include <private/omni/physics/IPhysicsStageUpdate.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

// Include the new binding files
#include "BindingsPhysicsSceneQuery.cpp"
#include "BindingsPhysicsInteraction.cpp"
#include "BindingsPhysicsBenchmarks.cpp"
#include "BindingsPhysicsSimulation.cpp"
#include "BindingsPhysicsStageUpdate.cpp"

#include <omni/Span.h>


CARB_BINDINGS("omni.physics.python")


PYBIND11_MAKE_OPAQUE(omni::physics::ContactEventHeaderVector);
PYBIND11_MAKE_OPAQUE(omni::physics::ContactDataVector);
PYBIND11_MAKE_OPAQUE(omni::physics::FrictionAnchorsDataVector);

namespace
{

PYBIND11_MODULE(_physics, m)
{
    using namespace carb;
    using namespace omni::physics;      

    m.doc() = "pybind11 omni.physics bindings";

    // Bind SimulationId
    py::class_<SimulationId>(m, "SimulationId", R"(
        A unique identifier for a physics simulation instance.
        Used to reference and manage different physics simulations in the system.
    )")
        .def(py::init<>(), "Create an invalid simulation ID")
        .def(py::init<size_t>(), "Create a simulation ID with the given value")
        .def("__eq__", &SimulationId::operator==, "Compare two simulation IDs for equality")
        .def("__ne__", &SimulationId::operator!=, "Compare two simulation IDs for inequality")
        .def("__hash__", &SimulationId::hash, "Get the hash value of the simulation ID")
        .def_readonly("id", &SimulationId::id, "The underlying ID value");

    // Bind PhysicsStepContext
    py::class_<PhysicsStepContext>(m, "PhysicsStepContext", R"(
        Context information for a physics simulation step.
        Contains data needed during physics simulation updates.
    )")
        .def(py::init<>(), "Create an empty physics step context")
        .def_readwrite("scene_path", &PhysicsStepContext::scenePath, "Path to the scene being simulated")
        .def_readwrite("simulation_id", &PhysicsStepContext::simulationId, "ID of the simulation being stepped");

    // Bind ForceModeType
    py::enum_<ForceModeType::Enum>(m, "ForceMode", R"(
        Enumeration of different ways to apply forces in physics simulation.
        Determines how forces are interpreted and applied to objects.
    )")
        .value("FORCE", ForceModeType::eFORCE, "Apply a continuous force")
        .value("IMPULSE", ForceModeType::eIMPULSE, "Apply an instantaneous impulse")
        .value("VELOCITY_CHANGE", ForceModeType::eVELOCITY_CHANGE, "Directly change the velocity")
        .value("ACCELERATION", ForceModeType::eACCELERATION, "Apply an acceleration")
        .export_values();

    // Bind ContactEventType
    py::enum_<ContactEventType::Enum>(m, "ContactEventType", R"(
        Enumeration of different types of contact events.
        Determines the type of contact event.
    )")
        .value("CONTACT_FOUND", ContactEventType::eCONTACT_FOUND, "Contact issued for newly found contact pairs")
        .value("CONTACT_LOST", ContactEventType::eCONTACT_LOST, "Contact issued for contact pair lost")
        .value("CONTACT_PERSIST", ContactEventType::eCONTACT_PERSIST, "Contact issued for persistent contact pairs")
        .export_values();

    // Bind ContactEventHeader
    py::class_<ContactEventHeader>(m, "ContactEventHeader", R"(
            Contact event header, issued for contact pair.
            Contains information about the pair and the number of contact data.
        )")
        .def(py::init<>())
        .def_readwrite("type", &ContactEventHeader::type, R"(Contact event header type.)")
        .def_readwrite("stage_id", &ContactEventHeader::stageId, R"(Stage id of the simulated USD stage.)")
        .def_readwrite("actor0", &ContactEventHeader::actor0, R"(Actor0 of the contact pair, can be retyped to SdfPath.)")
        .def_readwrite("actor1", &ContactEventHeader::actor1, R"(Actor1 of the contact pair, can be retyped to SdfPath.)")
        .def_readwrite("collider0", &ContactEventHeader::collider0, R"(Collider0 of the contact pair, can be retyped to SdfPath.)")
        .def_readwrite("collider1", &ContactEventHeader::collider1, R"(Collider1 of the contact pair, can be retyped to SdfPath.)")
        .def_readwrite("contact_data_offset", &ContactEventHeader::contactDataOffset, R"(Contact data offset index to the contact data array.)")
        .def_readwrite("num_contact_data", &ContactEventHeader::numContactData, R"(Number of contact data in the contact data array for given pair.)")
        .def_readwrite("friction_anchors_data_offset", &ContactEventHeader::frictionAnchorsDataOffset, R"(Friction anchors offset index to the friction anchors data array.)")
        .def_readwrite("num_friction_anchors_data", &ContactEventHeader::numfrictionAnchorsData, R"(Number of friction anchors data in the friction anchors data array for given pair.)")
        .def_readwrite("proto_index0", &ContactEventHeader::protoIndex0, R"(Point instancer prototypeIndex for collider0, used only for point instancers otherwise 0xFFFFFFFF.)")
        .def_readwrite("proto_index1", &ContactEventHeader::protoIndex1, R"(Point instancer prototypeIndex for collider1, used only for point instancers otherwise 0xFFFFFFFF.)");

    // Bind ContactData
    py::class_<ContactData>(m, "ContactData")
        .def(py::init<>())
        .def_readwrite("position", &ContactData::position)
        .def_readwrite("normal", &ContactData::normal)
        .def_readwrite("separation", &ContactData::separation)
        .def_readwrite("impulse", &ContactData::impulse);

    // Bind FrictionAnchor
    py::class_<FrictionAnchor>(m, "FrictionAnchor")
        .def(py::init<>())
        .def_readwrite("position", &FrictionAnchor::position)        
        .def_readwrite("impulse", &FrictionAnchor::impulse);

    // Add vector bindings
    py::bind_vector<ContactEventHeaderVector>(m, "ContactEventHeaderVector", py::module_local(false));
    py::bind_vector<ContactDataVector>(m, "ContactDataVector", py::module_local(false));
    py::bind_vector<FrictionAnchorsDataVector>(m, "FrictionAnchorsDataVector", py::module_local(false));

    py::class_<SimulationFns>(m, "SimulationFns", R"(
        Collection of functions for managing physics simulation.
        Provides core functionality for physics simulation control and interaction.
    )")
        .def(py::init<>(), "Create a new simulation functions object")
        .def_readwrite("initialize", &SimulationFns::initialize, "Function to initialize the simulation with a stage")
        .def_readwrite("close", &SimulationFns::close, "Function to close the simulation")
        .def_readwrite("get_attached_stage", &SimulationFns::getAttachedStage, "Get the currently attached stage")
        .def_readwrite("simulate_async", &SimulationFns::simulateAsync, "Run the physics simulation asynchronously")
        .def_readwrite("simulate", &SimulationFns::simulate, "Run the physics simulation synchronously")
        .def_readwrite("fetch_results", &SimulationFns::fetchResults, "Retrieve simulation results")
        .def_readwrite("check_results", &SimulationFns::checkResults, "Check if simulation results are available")
        .def_readwrite("flush_changes", &SimulationFns::flushChanges, "Apply pending changes to the simulation")
        .def_readwrite("pause_change_tracking", &SimulationFns::pauseChangeTracking, "Pause tracking of simulation changes")
        .def_readwrite("is_change_tracking_paused", &SimulationFns::isChangeTrackingPaused, "Check if change tracking is paused")
        .def_readwrite("subscribe_physics_contact_report_events", &SimulationFns::subscribePhysicsContactReportEvents, "Subscribe to physics contact events")
        .def_readwrite("unsubscribe_physics_contact_report_events", &SimulationFns::unsubscribePhysicsContactReportEvents, "Unsubscribe from physics contact events")
        .def_readwrite("get_simulation_time_steps_per_second", &SimulationFns::getSimulationTimeStepsPerSecond, "Get the simulation time steps per second")
        .def_readwrite("get_simulation_timestamp", &SimulationFns::getSimulationTimestamp, "Get the current simulation timestamp")
        .def_readwrite("get_simulation_step_count", &SimulationFns::getSimulationStepCount, "Get the number of simulation steps executed")
        .def_readwrite("subscribe_physics_on_step_events", &SimulationFns::subscribePhysicsOnStepEvents, "Subscribe to physics step events")
        .def_readwrite("unsubscribe_physics_on_step_events", &SimulationFns::unsubscribePhysicsOnStepEvents, "Unsubscribe from physics step events")
        .def_property(
            "is_capable_of_simulating",
            // Getter - returns the wrapped Python function or None
            [](SimulationFns& self) -> py::object {
                if (!self.isCapableOfSimulating)
                {
                    return py::none();
                }
                // Return a lambda that wraps the C++ function with Pythonic signature
                return py::cpp_function(
                    [fn = self.isCapableOfSimulating](const std::vector<std::string>& schemaNames) {
                        std::vector<const char*> schemaNamePtrs;
                        schemaNamePtrs.reserve(schemaNames.size());
                        for (const auto& name : schemaNames)
                        {
                            schemaNamePtrs.push_back(name.c_str());
                        }
                        std::unique_ptr<bool[]> isCapableOut(new bool[schemaNames.size()]);
                        bool success = fn(schemaNamePtrs.data(), schemaNamePtrs.size(), isCapableOut.get());
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
                    });
            },
            // Setter - wraps a Python function with Pythonic signature to C++ signature
            [](SimulationFns& self, py::object pyFunc) {
                if (pyFunc.is_none())
                {
                    self.isCapableOfSimulating = nullptr;
                    return;
                }
                // Wrap the Python function to match C++ signature
                self.isCapableOfSimulating = [pyFunc](const char** schemaNames, size_t schemaNamesCount, bool* isCapable) -> bool {
                    py::gil_scoped_acquire acquire;
                    try
                    {
                        // Convert C++ args to Python
                        py::list pySchemaNames;
                        for (size_t i = 0; i < schemaNamesCount; ++i)
                        {
                            pySchemaNames.append(py::str(schemaNames[i]));
                        }
                        // Call Python function with Pythonic signature: (schema_names) -> (success, capabilities)
                        py::tuple result = pyFunc(pySchemaNames).cast<py::tuple>();
                        bool success = result[0].cast<bool>();
                        if (!success)
                        {
                            return false;
                        }
                        py::list capabilities = result[1].cast<py::list>();
                        for (size_t i = 0; i < schemaNamesCount && i < capabilities.size(); ++i)
                        {
                            isCapable[i] = capabilities[i].cast<bool>();
                        }
                        return true;
                    }
                    catch (const py::error_already_set& )
                    {
                        CARB_LOG_ERROR("Failed to accept the isCapableOfSimulating implementation.");
                        return false;
                    }
                };
            },
            "Check if simulation is capable of simulating given schema types.\n\n"
            "The function should have signature: (schema_names: list[str]) -> tuple[bool, list[bool]]\n"
            "Returns a tuple of (success, capabilities) where capabilities is a list of booleans.");

    // Bind StageUpdateFns
    py::class_<StageUpdateFns>(m, "StageUpdateFns", R"(
        Collection of functions for managing stage updates in physics simulation.
        Handles stage lifecycle and update events.
    )")
        .def(py::init<>(), "Create a new stage update functions object")
        .def_readwrite("on_attach", &StageUpdateFns::onAttach, "Callback when a stage is attached")
        .def_readwrite("on_detach", &StageUpdateFns::onDetach, "Callback when a stage is detached")
        .def_readwrite("on_update", &StageUpdateFns::onUpdate, "Callback for stage updates")
        .def_readwrite("on_resume", &StageUpdateFns::onResume, "Callback when simulation resumes")
        .def_readwrite("on_pause", &StageUpdateFns::onPause, "Callback when simulation pauses")
        .def_readwrite("on_reset", &StageUpdateFns::onReset, "Callback when simulation resets")
        .def_readwrite("handle_raycast", &StageUpdateFns::handleRaycast, "Handle raycast queries")
        .def_readwrite("force_load_physics_from_usd", &StageUpdateFns::forceLoadPhysicsFromUSD, "Force load physics data from USD")
        .def_readwrite("release_physics_objects", &StageUpdateFns::releasePhysicsObjects, "Release physics objects")
        .def_readwrite("reset_simulation", &StageUpdateFns::resetSimulation, "Reset simulation")
        .def_readwrite("start_simulation", &StageUpdateFns::startSimulation, "Start simulation");

    // Bind InteractionFns
    py::class_<InteractionFns>(m, "InteractionFns", R"(
        Collection of functions for managing physics interactions.
        Handles raycast queries and interaction control.
    )")
        .def(py::init<>(), "Create a new interaction functions object")
        .def_readwrite("disable_reset_on_stop", &InteractionFns::disableResetOnStop, "Disable reset on stop")
        .def_readwrite("is_disabled_reset_on_stop", &InteractionFns::isDisabledResetOnStop, "Check if reset on stop is disabled")
        .def_readwrite("handle_raycast", &InteractionFns::handleRaycast, "Handle raycast queries")
        .def_readwrite("get_prim_debug_data", &InteractionFns::getPrimDebugData, R"(
                Get simulation debug data for a prim
                Args:
                    prim_path (str): The prim path as a string (e.g., "/World/Cube")
                Expected return value:
                    dict[str, dict[str, Any]]: Dictionary of simulation debug data for the prim. 
                    Each entry is itself should be a dictionary with the following keys:
                    - "type": The type of the debug data item (one of DebugDataItemType)
                    - "doc": The documentation string for the debug data item
                    - "value": The value of the debug data item (see DebugDataItemType for the expected value format of the value)
            )");

    // Bind SceneQueryFns
    py::class_<SceneQueryFns>(m, "SceneQueryFns", R"(
        Collection of functions for managing scene queries.
        Handles raycast queries and scene query control.
    )")
        .def(py::init<>(), "Create a new scene query functions object")
        .def_readwrite("raycast_closest", &SceneQueryFns::raycastClosest, "Raycast the closest object")
        .def_readwrite("raycast_all", &SceneQueryFns::raycastAll, "Raycast all objects")
        .def_readwrite("raycast_any", &SceneQueryFns::raycastAny, "Raycast any object")
        .def_readwrite("sweep_sphere_closest", &SceneQueryFns::sweepSphereClosest, "Sweep sphere the closest object")
        .def_readwrite("sweep_sphere_all", &SceneQueryFns::sweepSphereAll, "Sweep sphere all objects")
        .def_readwrite("sweep_sphere_any", &SceneQueryFns::sweepSphereAny, "Sweep sphere any object")
        .def_readwrite("sweep_box_closest", &SceneQueryFns::sweepBoxClosest, "Sweep box the closest object")
        .def_readwrite("sweep_box_all", &SceneQueryFns::sweepBoxAll, "Sweep box all objects")
        .def_readwrite("sweep_box_any", &SceneQueryFns::sweepBoxAny, "Sweep box any object")
        .def_readwrite("sweep_shape_closest", &SceneQueryFns::sweepShapeClosest, "Sweep shape the closest object")
        .def_readwrite("sweep_shape_all", &SceneQueryFns::sweepShapeAll, "Sweep shape all objects")
        .def_readwrite("sweep_shape_any", &SceneQueryFns::sweepShapeAny, "Sweep shape any object")
        .def_readwrite("overlap_sphere", &SceneQueryFns::overlapSphere, "Overlap sphere")
        .def_readwrite("overlap_sphere_any", &SceneQueryFns::overlapSphereAny, "Overlap sphere any")
        .def_readwrite("overlap_box", &SceneQueryFns::overlapBox, "Overlap box")
        .def_readwrite("overlap_box_any", &SceneQueryFns::overlapBoxAny, "Overlap box any")
        .def_readwrite("overlap_shape", &SceneQueryFns::overlapShape, "Overlap shape")
        .def_readwrite("overlap_shape_any", &SceneQueryFns::overlapShapeAny, "Overlap shape any");

    // Bind BenchmarkFns
    py::class_<BenchmarkFns>(m, "BenchmarkFns", R"(
        Collection of functions for managing benchmarks.
        Handles benchmark control and statistics.
    )")
        .def(py::init<>(), "Create a new benchmark functions object")
        .def_readwrite("subscribe_profile_stats_events", &BenchmarkFns::subscribeProfileStatsEvents, "Subscribe to profile stats events")
        .def_readwrite("unsubscribe_profile_stats_events", &BenchmarkFns::unsubscribeProfileStatsEvents, "Unsubscribe from profile stats events");  

    // Bind Simulation
    py::class_<Simulation>(m, "Simulation", R"(
        Main physics simulation class that combines all simulation functionality.
        Provides access to simulation, scene query, interaction, and stage update functions.
    )")
        .def(py::init<>(), "Create a new simulation instance")
        .def_readwrite("simulation_fns", &Simulation::simulationFns, "Core simulation functions")
        .def_readwrite("scene_query_fns", &Simulation::sceneQueryFns, "Scene query functions")
        .def_readwrite("interaction_fns", &Simulation::interactionFns, "Interaction functions")
        .def_readwrite("stage_update_fns", &Simulation::stageUpdateFns, "Stage update functions")
        .def_readwrite("benchmark_fns", &Simulation::benchmarkFns, "Benchmark functions");

    py::enum_<SimulationRegistryEventType::Enum>(m, "SimulationRegistryEventType", R"(
        Enumeration of different types of simulation registry events.
        Determines the type of simulation registry event.
    )")
        .value("SIMULATION_REGISTERED", SimulationRegistryEventType::eSIMULATION_REGISTERED, "Simulation registered")
        .value("SIMULATION_UNREGISTERED", SimulationRegistryEventType::eSIMULATION_UNREGISTERED, "Simulation unregistered")
        .value("SIMULATION_ACTIVATED", SimulationRegistryEventType::eSIMULATION_ACTIVATED, "Simulation activated")
        .value("SIMULATION_DEACTIVATED", SimulationRegistryEventType::eSIMULATION_DEACTIVATED, "Simulation deactivated")
        .export_values();

    // Define IPhysics interface
    defineInterfaceClass<IPhysics>(m, "IPhysics", "acquire_physics_interface", "release_physics_interface", R"(
        Main physics interface for managing simulations.
        Provides functionality to register, unregister, and control physics simulations.
    )")
        .def("register_simulation", [](IPhysics& self, const Simulation& simulation, const std::string& name) {
            return self.registerSimulation(simulation, name.c_str());
        }, py::arg("simulation"), py::arg("name"), "Register a new physics simulation with the given name")
        .def("unregister_simulation", wrapInterfaceFunction(&IPhysics::unregisterSimulation), py::arg("id"), "Unregister a physics simulation by ID")
        .def(
            "get_simulation",
            [](IPhysics& self, const SimulationId& id) -> py::object {
                const Simulation* simulation = self.getSimulation(id);                
                if (!simulation)
                    return py::none();
                else
                {
                    Simulation simOut = *simulation;
                    return py::cast(simOut);
                }
            },
            py::arg("id"), "Get a simulation by ID")
        .def("get_simulation_name", [](IPhysics& self, const SimulationId& id) {
            const char* name = self.getSimulationName(id);
            return name ? std::string(name) : std::string();
        }, py::arg("id"), "Get the name of a simulation by ID")
        .def("get_num_simulations", wrapInterfaceFunction(&IPhysics::getNumSimulations), "Get the total number of registered simulations")
        .def("get_simulation_ids", [](IPhysics& self) {
            size_t numSims = self.getNumSimulations();
            std::vector<SimulationId> ids(numSims);
            self.getSimulationIds(ids.data(), numSims);
            return ids;
        }, "Get all registered simulation IDs")
        .def("activate_simulation", wrapInterfaceFunction(&IPhysics::activateSimulation), py::arg("id"), "Activate a simulation by ID")
        .def("deactivate_simulation", wrapInterfaceFunction(&IPhysics::deactivateSimulation), py::arg("id"), "Deactivate a simulation by ID")
        .def("is_simulation_active", wrapInterfaceFunction(&IPhysics::isSimulationActive), py::arg("id"), "Check if a simulation is active")
        .def("subscribe_simulation_registry_events", [](IPhysics& self, std::function<void(const SimulationRegistryEventType::Enum eventType, const SimulationId& id, const char* simulationName)> onEvent) {

            return carb::createPySubscription(std::move(onEvent),
                [self](auto fn, void* userData) -> SubscriptionId {
                    return self.subscribeSimulationRegistryEvents(fn, userData);
                },
                [self](SubscriptionId id) {
                    self.unsubscribeSimulationRegistryEvents(id);
                });
        }, py::arg("on_event"), "Subscribe to simulation registry events");
        
    // Define constants
    m.attr("k_invalid_simulation_id") = kInvalidSimulationId;
    m.attr("k_invalid_subscription_id") = kInvalidSubscriptionId;

    // Call the binding functions from the new files
    bindPhysicsSceneQuery(m);
    bindPhysicsInteraction(m);
    bindPhysicsBenchmarks(m);
    bindPhysicsSimulation(m);
    bindPhysicsStageUpdate(m);
}
} // namespace
