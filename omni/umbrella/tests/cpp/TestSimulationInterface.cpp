// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for basic simulation interface and structure

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>

using namespace omni::physics;

//=============================================================================
// TEST: Simulation Interface
//=============================================================================
TEST_CASE("Simulation Interface")
{
    SimulationRegistry registry;

    SUBCASE("Basic simulation structure")
    {
        Simulation simulation;

        // Test that all function pointers start as nullptr
        REQUIRE(simulation.simulationFns.initialize == nullptr);
        REQUIRE(simulation.simulationFns.simulateAsync == nullptr);
        REQUIRE(simulation.simulationFns.simulate == nullptr);
        REQUIRE(simulation.stageUpdateFns.onAttach == nullptr);
        REQUIRE(simulation.sceneQueryFns.raycastClosest == nullptr);
        REQUIRE(simulation.interactionFns.disableResetOnStop == nullptr);
        REQUIRE(simulation.benchmarkFns.subscribeProfileStatsEvents == nullptr);
    }

    SUBCASE("Attach simulation functions")
    {
        Simulation simulation;

        // Attach a simple simulate function
        bool simulateCalled = false;
        simulation.simulationFns.simulate = [&simulateCalled](float /*dt*/, float /*t*/) {
            simulateCalled = true;
        };

        // Register and verify function is attached
        SimulationId simId = registry.registerSimulation(simulation, "FunctionTest");
        REQUIRE(simId != kInvalidSimulationId);

        const Simulation* retrieved = registry.getSimulation(simId);
        REQUIRE(retrieved != nullptr);
        REQUIRE(retrieved->simulationFns.simulate != nullptr);

        // Call the function
        retrieved->simulationFns.simulate(1.0f / 60.0f, 0.0f);
        REQUIRE(simulateCalled == true);

        registry.unregisterSimulation(simId);
    }
}
