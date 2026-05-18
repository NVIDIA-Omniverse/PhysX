// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "simulator/Simulator.h"

namespace omni
{
namespace physics
{

// Event types for simulation registry events. The events are triggered after the associated action has been executed.
struct SimulationRegistryEventType
{
    enum Enum
    {
        eSIMULATION_REGISTERED, //!< A new simulation has been registered
        eSIMULATION_UNREGISTERED, //!< An existing simulation has been unregistered. Since the event is triggered after unregistration, the simulation id is no longer valid for use with the API at this point
        eSIMULATION_ACTIVATED, //!< A simulation has been activated (note that upon registration, simulations are already in an activated state, but will not trigger this event upon registration)
        eSIMULATION_DEACTIVATED, //!< A simulation has been deactivated
    };
};

// Callback function for simulation registry events
//
// \param eventType The SimulationRegistryEventType identifier for the triggered event
// \param id The SimulationId of the simulation that triggered the event
// \param simulationName The name of the simulation that triggered the event
// \param userData Pointer to user data that was provided when subscribing to the event
using OnSimulationRegistryEventFn = std::function<void(const SimulationRegistryEventType::Enum eventType, const SimulationId& id, const char* simulationName, void* userData)>;


struct IPhysics
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysics", 0, 2)

    /// Register a simulation with the physics system.
    /// \param simulation The simulation to register.
    /// \return The simulation id.
    SimulationId(CARB_ABI* registerSimulation)(const Simulation& simulation, const char* simulationName);

    /// Unregister a simulation from the physics system.
    /// \param id The simulation id.
    void(CARB_ABI* unregisterSimulation)(const SimulationId& id);

    /// Get a simulation from the physics system.
    /// \param id The simulation id.
    /// \return The simulation.
    const Simulation*(CARB_ABI* getSimulation)(const SimulationId& id);

    /// Get a simulation name.
    /// \param id The simulation id.
    /// \return The simulation.
    const char*(CARB_ABI* getSimulationName)(const SimulationId& id);

    /// Get num simulations registered with the physics system.
    /// \return The number of simulations.
    size_t(CARB_ABI* getNumSimulations)();

    /// Get the simulation ids registered with the physics system.
    /// \param simulationIds The buffer to store the simulation ids.
    /// \param bufferSize The size of the buffer.
    /// \return The number of simulations ids copied to the buffer.
    size_t(CARB_ABI* getSimulationIds)(SimulationId* simulationIds, size_t bufferSize);

    /// Activate a simulation.
    /// \param id The simulation id.
    void(CARB_ABI* activateSimulation)(const SimulationId& id);

    /// Deactivate a simulation.
    /// \param id The simulation id.
    void(CARB_ABI* deactivateSimulation)(const SimulationId& id);

    /// Check if a simulation is active.
    /// \param id The simulation id.
    /// \return True if the simulation is active, false otherwise.
    bool(CARB_ABI* isSimulationActive)(const SimulationId& id);

    /// Subscribe to simulation registry events.
    /// \param onEvent The callback function to be called on simulation registry events.
    /// \param userData User data pointer that will be passed to the callback function. Default is nullptr.
    /// \return A subscription id used for unsubscribing.
    SubscriptionId(CARB_ABI* subscribeSimulationRegistryEvents)(OnSimulationRegistryEventFn onEvent, void* userData /* = nullptr */);

    /// Unsubscribe from simulation registry events.
    /// \param subscriptionId The subscription id returned when subscribing.
    void(CARB_ABI* unsubscribeSimulationRegistryEvents)(SubscriptionId subscriptionId);
};

} // namespace physics
} // namespace omni
