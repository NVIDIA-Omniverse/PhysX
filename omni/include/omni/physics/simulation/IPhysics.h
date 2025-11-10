// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

struct IPhysics
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysics", 0, 1)

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
};

} // namespace physics
} // namespace omni
