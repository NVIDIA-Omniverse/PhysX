// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for benchmark functions (profile stats, performance monitoring)

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>
#include <vector>

using namespace omni::physics;

//=============================================================================
// TEST: Benchmark Functions
//=============================================================================
TEST_CASE("Benchmark Functions")
{
    SimulationRegistry registry;

    SUBCASE("Profile stats subscription")
    {
        Simulation simulation;

        SubscriptionId nextId = 100;
        std::vector<ProfileStatsNotificationFn> subscribers;

        simulation.benchmarkFns.subscribeProfileStatsEvents = [&](
            ProfileStatsNotificationFn callback) -> SubscriptionId {
            subscribers.push_back(callback);
            return nextId++;
        };

        simulation.benchmarkFns.unsubscribeProfileStatsEvents = [&](SubscriptionId id) {
            if (id >= 100 && id < nextId)
            {
                size_t index = id - 100;
                if (index < subscribers.size())
                {
                    subscribers.erase(subscribers.begin() + index);
                }
            }
        };

        SimulationId simId = registry.registerSimulation(simulation, "BenchmarkTest");
        const Simulation* sim = registry.getSimulation(simId);

        // Test subscription
        bool statsReceived = false;
        SubscriptionId subId = sim->benchmarkFns.subscribeProfileStatsEvents(
            [&statsReceived](const std::vector<PhysicsProfileStats>& /*stats*/) {
                statsReceived = true;
            });

        REQUIRE(subId == 100);
        REQUIRE(subscribers.size() == 1);

        // Simulate stats callback
        std::vector<PhysicsProfileStats> testStats;
        PhysicsProfileStats stat;
        stat.zoneName = "TestZone";
        stat.ms = 1.5f;
        testStats.push_back(stat);

        subscribers[0](testStats);
        REQUIRE(statsReceived == true);

        // Test unsubscription
        sim->benchmarkFns.unsubscribeProfileStatsEvents(subId);
        REQUIRE(subscribers.size() == 0);

        registry.unregisterSimulation(simId);
    }
}
