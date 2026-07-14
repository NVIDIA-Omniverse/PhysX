// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Adapted for standalone umbrella library testing

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>
#include <string>

using namespace omni::physics;

//-----------------------------------------------------------------------------
// Simulation Registry Tests
TEST_CASE("Simulation Registry Tests")
{
    SimulationRegistry registry;

    SUBCASE("Create a new simulation")
    {
        Simulation simulation;
        const char* simulationName = "TestSimulation";

        SimulationId simulationId = registry.registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != kInvalidSimulationId);

        // Test getting simulation name
        const char* retrievedName = registry.getSimulationName(simulationId);
        REQUIRE(strcmp(retrievedName, simulationName) == 0);

        registry.unregisterSimulation(simulationId);
    }

    SUBCASE("Register and unregister simulation")
    {
        Simulation simulation;
        const char* simulationName = "TestSimulation";

        // Test registration
        SimulationId simulationId = registry.registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != kInvalidSimulationId);

        // Test getting simulation by ID
        const Simulation* retrievedSim = registry.getSimulation(simulationId);
        REQUIRE(retrievedSim != nullptr);

        // Test getting simulation name
        const char* retrievedName = registry.getSimulationName(simulationId);
        REQUIRE(strcmp(retrievedName, simulationName) == 0);

        // Test unregistration
        registry.unregisterSimulation(simulationId);
        const Simulation* afterUnregister = registry.getSimulation(simulationId);
        REQUIRE(afterUnregister == nullptr);
    }

    SUBCASE("Get number of simulations")
    {
        size_t initialCount = registry.getNumSimulations();

        Simulation simulation;
        SimulationId simulationId = registry.registerSimulation(simulation, "CountTest");

        size_t newCount = registry.getNumSimulations();
        REQUIRE(newCount == initialCount + 1);

        registry.unregisterSimulation(simulationId);

        size_t finalCount = registry.getNumSimulations();
        REQUIRE(finalCount == initialCount);
    }

    SUBCASE("Activate and deactivate simulation")
    {
        Simulation simulation;
        SimulationId simulationId = registry.registerSimulation(simulation, "ActivationTest");

        // Simulation should start active
        bool isActive = registry.isSimulationActive(simulationId);
        REQUIRE(isActive == true);

        // Deactivate
        registry.deactivateSimulation(simulationId);
        isActive = registry.isSimulationActive(simulationId);
        REQUIRE(isActive == false);

        // Reactivate
        registry.activateSimulation(simulationId);
        isActive = registry.isSimulationActive(simulationId);
        REQUIRE(isActive == true);

        registry.unregisterSimulation(simulationId);
    }

    SUBCASE("Get simulation IDs")
    {
        // Register multiple simulations
        SimulationId ids[3];
        for (int i = 0; i < 3; i++)
        {
            Simulation sim;
            ids[i] = registry.registerSimulation(sim, ("Sim" + std::to_string(i)).c_str());
        }

        // Get simulation IDs via the map
        const SimulationMap& simulations = registry.getSimulations();
        REQUIRE(simulations.size() >= 3);

        // Clean up
        for (int i = 0; i < 3; i++)
        {
            registry.unregisterSimulation(ids[i]);
        }
    }

    SUBCASE("Invalid simulation operations")
    {
        // Test with invalid ID
        const Simulation* invalidSim = registry.getSimulation(kInvalidSimulationId);
        REQUIRE(invalidSim == nullptr);

        const char* invalidName = registry.getSimulationName(kInvalidSimulationId);
        REQUIRE(invalidName == nullptr);

        bool invalidActive = registry.isSimulationActive(kInvalidSimulationId);
        REQUIRE(invalidActive == false);
    }
}
