// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for stage update functions (lifecycle callbacks)

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>

using namespace omni::physics;

//=============================================================================
// TEST: Stage Update Functions
//=============================================================================
TEST_CASE("Stage Update Functions")
{
    SimulationRegistry registry;

    SUBCASE("Stage lifecycle callbacks")
    {
        Simulation simulation;

        bool attached = false;
        bool detached = false;
        bool updated = false;
        bool resumed = false;
        bool paused = false;
        bool reset = false;

        simulation.stageUpdateFns.onAttach = [&attached](long /*id*/) { attached = true; };
        simulation.stageUpdateFns.onDetach = [&detached]() { detached = true; };
        simulation.stageUpdateFns.onUpdate = [&updated](float /*t*/, float /*dt*/, bool /*enable*/) { updated = true; };
        simulation.stageUpdateFns.onResume = [&resumed](float /*t*/) { resumed = true; };
        simulation.stageUpdateFns.onPause = [&paused]() { paused = true; };
        simulation.stageUpdateFns.onReset = [&reset]() { reset = true; };

        SimulationId simId = registry.registerSimulation(simulation, "StageUpdateTest");
        const Simulation* sim = registry.getSimulation(simId);

        // Test all callbacks
        sim->stageUpdateFns.onAttach(12345);
        REQUIRE(attached == true);

        sim->stageUpdateFns.onUpdate(0.0f, 1.0f / 60.0f, true);
        REQUIRE(updated == true);

        sim->stageUpdateFns.onResume(0.0f);
        REQUIRE(resumed == true);

        sim->stageUpdateFns.onPause();
        REQUIRE(paused == true);

        sim->stageUpdateFns.onReset();
        REQUIRE(reset == true);

        sim->stageUpdateFns.onDetach();
        REQUIRE(detached == true);

        registry.unregisterSimulation(simId);
    }
}
