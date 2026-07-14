// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for registry event notification system

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>
#include <string>
#include <vector>

using namespace omni::physics;

//=============================================================================
// TEST: Registry Events
//=============================================================================
TEST_CASE("Registry Events")
{
    SimulationRegistry registry;

    SUBCASE("Simulation registry event notifications")
    {
        struct EventRecord {
            SimulationRegistryEventType::Enum eventType;
            SimulationId simId;
            std::string simName;
        };

        std::vector<EventRecord> events;

        SubscriptionId subId = registry.subscribeSimulationRegistryEvents(
            [&events](SimulationRegistryEventType::Enum eventType,
                      const SimulationId& id,
                      const char* name,
                      void* /*userData*/) {
                EventRecord record;
                record.eventType = eventType;
                record.simId = id;
                record.simName = name ? name : "";
                events.push_back(record);
            },
            nullptr);

        REQUIRE(subId != kInvalidSubscriptionId);

        // Register simulation - should trigger event
        Simulation simulation;
        SimulationId simId = registry.registerSimulation(simulation, "EventTest");

        REQUIRE(events.size() >= 1);
        REQUIRE(events.back().eventType == SimulationRegistryEventType::eSIMULATION_REGISTERED);
        REQUIRE(events.back().simName == "EventTest");

        // Deactivate - should trigger event
        registry.deactivateSimulation(simId);
        REQUIRE(events.back().eventType == SimulationRegistryEventType::eSIMULATION_DEACTIVATED);

        // Activate - should trigger event
        registry.activateSimulation(simId);
        REQUIRE(events.back().eventType == SimulationRegistryEventType::eSIMULATION_ACTIVATED);

        // Unregister - should trigger event
        registry.unregisterSimulation(simId);
        REQUIRE(events.back().eventType == SimulationRegistryEventType::eSIMULATION_UNREGISTERED);

        registry.unsubscribeSimulationRegistryEvents(subId);
    }
}
