// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for scene query functions (raycasts, overlaps, sweeps)

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>
#include <vector>

using namespace omni::physics;

//=============================================================================
// TEST: Scene Query Functions
//=============================================================================
TEST_CASE("Scene Query Functions")
{
    SimulationRegistry registry;

    SUBCASE("Raycast functions")
    {
        Simulation simulation;

        bool raycastCalled = false;
        simulation.sceneQueryFns.raycastClosest = [&raycastCalled](
            const carb::Float3& /*origin*/, const carb::Float3& /*dir*/, float /*dist*/,
            RaycastHit& hit, bool /*bothSides*/) -> bool {
            raycastCalled = true;
            hit.distance = 10.0f;
            hit.position = carb::Float3{5.0f, 0.0f, 0.0f};
            return true;
        };

        simulation.sceneQueryFns.raycastAny = [](
            const carb::Float3& /*origin*/, const carb::Float3& /*dir*/, float /*dist*/, bool /*bothSides*/) -> bool {
            return true;
        };

        SimulationId simId = registry.registerSimulation(simulation, "RaycastTest");
        const Simulation* sim = registry.getSimulation(simId);

        // Test raycastClosest
        RaycastHit hit;
        carb::Float3 origin{0.0f, 0.0f, 0.0f};
        carb::Float3 direction{1.0f, 0.0f, 0.0f};
        bool result = sim->sceneQueryFns.raycastClosest(origin, direction, 100.0f, hit, false);

        REQUIRE(raycastCalled == true);
        REQUIRE(result == true);
        REQUIRE(hit.distance == 10.0f);
        REQUIRE(hit.position.x == 5.0f);

        // Test raycastAny
        result = sim->sceneQueryFns.raycastAny(origin, direction, 100.0f, false);
        REQUIRE(result == true);

        registry.unregisterSimulation(simId);
    }

    SUBCASE("Overlap functions")
    {
        Simulation simulation;

        simulation.sceneQueryFns.overlapSphere = [](
            float /*radius*/, const carb::Float3& /*pos*/, OverlapHitReportFn reportFn) -> uint32_t {
            // Simulate 2 overlaps
            OverlapHit hit1;
            hit1.collision = 123;
            reportFn(hit1);

            OverlapHit hit2;
            hit2.collision = 456;
            reportFn(hit2);

            return 2;
        };

        simulation.sceneQueryFns.overlapSphereAny = [](
            float /*radius*/, const carb::Float3& /*pos*/) -> bool {
            return true;
        };

        SimulationId simId = registry.registerSimulation(simulation, "OverlapTest");
        const Simulation* sim = registry.getSimulation(simId);

        // Test overlapSphere
        std::vector<OverlapHit> hits;
        carb::Float3 zero{0.0f, 0.0f, 0.0f};
        uint32_t count = sim->sceneQueryFns.overlapSphere(5.0f, zero,
            [&hits](const OverlapHit& hit) {
                hits.push_back(hit);
                return true;
            });

        REQUIRE(count == 2);
        REQUIRE(hits.size() == 2);
        REQUIRE(hits[0].collision == 123);
        REQUIRE(hits[1].collision == 456);

        // Test overlapSphereAny
        bool hasOverlap = sim->sceneQueryFns.overlapSphereAny(5.0f, zero);
        REQUIRE(hasOverlap == true);

        registry.unregisterSimulation(simId);
    }
}
