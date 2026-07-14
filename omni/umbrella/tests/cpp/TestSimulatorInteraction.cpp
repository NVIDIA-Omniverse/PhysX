// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for interaction functions (user interactions, reset on stop, etc.)

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>

using namespace omni::physics;

//=============================================================================
// TEST: Interaction Functions
//=============================================================================
TEST_CASE("Interaction Functions")
{
    SimulationRegistry registry;

    SUBCASE("Reset on stop")
    {
        Simulation simulation;

        bool resetOnStopDisabled = false;
        simulation.interactionFns.disableResetOnStop = [&resetOnStopDisabled](bool disable) {
            resetOnStopDisabled = disable;
        };

        simulation.interactionFns.isDisabledResetOnStop = [&resetOnStopDisabled]() -> bool {
            return resetOnStopDisabled;
        };

        SimulationId simId = registry.registerSimulation(simulation, "InteractionTest");
        const Simulation* sim = registry.getSimulation(simId);

        // Test disable
        sim->interactionFns.disableResetOnStop(true);
        REQUIRE(sim->interactionFns.isDisabledResetOnStop() == true);

        // Test enable
        sim->interactionFns.disableResetOnStop(false);
        REQUIRE(sim->interactionFns.isDisabledResetOnStop() == false);

        registry.unregisterSimulation(simId);
    }

    SUBCASE("Handle raycast")
    {
        Simulation simulation;

        bool raycastHandled = false;
        simulation.interactionFns.handleRaycast = [&raycastHandled](
            const float* /*origin*/, const float* /*dir*/, bool /*input*/) {
            raycastHandled = true;
        };

        SimulationId simId = registry.registerSimulation(simulation, "RaycastInteractionTest");
        const Simulation* sim = registry.getSimulation(simId);

        float origin[3] = { 0.0f, 0.0f, 0.0f };
        float dir[3] = { 1.0f, 0.0f, 0.0f };
        sim->interactionFns.handleRaycast(origin, dir, true);

        REQUIRE(raycastHandled == true);

        registry.unregisterSimulation(simId);
    }
}
