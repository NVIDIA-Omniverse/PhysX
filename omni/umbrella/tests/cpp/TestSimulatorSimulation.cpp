// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for simulation functions (initialize, simulate, fetchResults, etc.)

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>

using namespace omni::physics;

//=============================================================================
// TEST: Simulation Functions
//=============================================================================
TEST_CASE("Simulation Functions")
{
    SimulationRegistry registry;

    SUBCASE("Initialize and Close")
    {
        Simulation simulation;

        long attachedStageId = 0;
        simulation.simulationFns.initialize = [&attachedStageId](long id) -> bool {
            attachedStageId = id;
            return true;
        };

        simulation.simulationFns.close = [&attachedStageId]() {
            attachedStageId = 0;
        };

        simulation.simulationFns.getAttachedStage = [&attachedStageId]() -> long {
            return attachedStageId;
        };

        SimulationId simId = registry.registerSimulation(simulation, "StageTest");

        const Simulation* sim = registry.getSimulation(simId);
        REQUIRE(sim);

        // Test initialize
        bool initialized = sim->simulationFns.initialize(12345);
        REQUIRE(initialized == true);
        REQUIRE(attachedStageId == 12345);

        long retrieved = sim->simulationFns.getAttachedStage();
        REQUIRE(retrieved == 12345);

        // Test close
        sim->simulationFns.close();
        REQUIRE(attachedStageId == 0);

        registry.unregisterSimulation(simId);
    }

    SUBCASE("SimulateAsync and FetchResults")
    {
        Simulation simulation;

        float lastDt = 0.0f;
        bool resultsFetched = false;

        simulation.simulationFns.simulateAsync = [&lastDt](float dt, float /*t*/) {
            lastDt = dt;
        };

        simulation.simulationFns.fetchResults = [&resultsFetched]() {
            resultsFetched = true;
        };

        simulation.simulationFns.checkResults = []() -> bool {
            return true;
        };

        SimulationId simId = registry.registerSimulation(simulation, "SimulateTest");
        const Simulation* sim = registry.getSimulation(simId);

        // Test simulateAsync
        sim->simulationFns.simulateAsync(1.0f / 60.0f, 0.0f);
        REQUIRE(lastDt == 1.0f / 60.0f);

        // Test checkResults
        bool ready = sim->simulationFns.checkResults();
        REQUIRE(ready == true);

        // Test fetchResults
        sim->simulationFns.fetchResults();
        REQUIRE(resultsFetched == true);

        registry.unregisterSimulation(simId);
    }
}
