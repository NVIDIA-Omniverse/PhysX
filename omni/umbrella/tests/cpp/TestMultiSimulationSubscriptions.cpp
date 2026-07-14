// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for multi-simulation subscription fan-out: subscribing to events at the
// umbrella level should forward to all registered simulations.

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <doctest/doctest.h>
#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

using namespace omni::physics;

// ---------------------------------------------------------------------------
// Mock simulator that tracks subscription callbacks
// ---------------------------------------------------------------------------
class MultiSimMockSimulator
{
public:
    MultiSimMockSimulator()
        : m_nextStepSubId(0)
        , m_nextContactSubId(0)
        , m_stepCallbackCount(0)
        , m_contactCallbackCount(0)
    {
    }

    SimulationFns getSimulationFns()
    {
        SimulationFns fns;
        fns.initialize = [](long) -> bool { return true; };
        fns.close = []() {};
        fns.getAttachedStage = []() -> long { return 0; };
        fns.simulateAsync = [this](float elapsedTime, float) {
            PhysicsStepContext context;
            context.scenePath = 0;
            context.simulationId = SimulationId(1);
            for (const auto& pair : m_stepEventCallbacks)
            {
                pair.second(elapsedTime, context);
            }
        };
        fns.fetchResults = [this]() {
            const ContactEventHeaderVector eventHeaders;
            const ContactDataVector contactData;
            const FrictionAnchorsDataVector frictionAnchors;
            for (const auto& pair : m_contactEventCallbacks)
            {
                pair.second(eventHeaders, contactData, frictionAnchors);
            }
        };
        fns.simulate = [&fns](float elapsedTime, float currentTime) {
            fns.simulateAsync(elapsedTime, currentTime);
            fns.fetchResults();
        };
        fns.checkResults = []() -> bool { return true; };
        fns.flushChanges = []() {};
        fns.pauseChangeTracking = [](bool) {};
        fns.isChangeTrackingPaused = []() -> bool { return false; };
        fns.getSimulationTimeStepsPerSecond = [](long, uint64_t) -> uint32_t { return 60; };
        fns.getSimulationTimestamp = []() -> uint64_t { return 0; };
        fns.getSimulationStepCount = []() -> uint64_t { return 0; };

        fns.subscribePhysicsContactReportEvents = [this](OnContactReportEventFn onEvent) -> SubscriptionId {
            SubscriptionId id = ++m_nextContactSubId;
            m_contactEventCallbacks[id] = onEvent;
            m_contactCallbackCount++;
            return id;
        };
        fns.unsubscribePhysicsContactReportEvents = [this](SubscriptionId subscriptionId) {
            auto it = m_contactEventCallbacks.find(subscriptionId);
            if (it != m_contactEventCallbacks.end())
            {
                m_contactEventCallbacks.erase(it);
                m_contactCallbackCount--;
            }
        };
        fns.subscribePhysicsOnStepEvents = [this](bool, int, OnPhysicsStepEventFn onUpdate) -> SubscriptionId {
            SubscriptionId id = ++m_nextStepSubId;
            m_stepEventCallbacks[id] = onUpdate;
            m_stepCallbackCount++;
            return id;
        };
        fns.unsubscribePhysicsOnStepEvents = [this](SubscriptionId subscriptionId) {
            auto it = m_stepEventCallbacks.find(subscriptionId);
            if (it != m_stepEventCallbacks.end())
            {
                m_stepEventCallbacks.erase(it);
                m_stepCallbackCount--;
            }
        };
        return fns;
    }

    int getStepCallbackCount() const { return m_stepCallbackCount; }
    int getContactCallbackCount() const { return m_contactCallbackCount; }

    // Invoke all registered step callbacks
    void fireStepCallbacks(float dt)
    {
        PhysicsStepContext context;
        context.scenePath = 0;
        context.simulationId = SimulationId(1);
        for (const auto& pair : m_stepEventCallbacks)
        {
            pair.second(dt, context);
        }
    }

    // Invoke all registered contact callbacks
    void fireContactCallbacks()
    {
        const ContactEventHeaderVector eventHeaders;
        const ContactDataVector contactData;
        const FrictionAnchorsDataVector frictionAnchors;
        for (const auto& pair : m_contactEventCallbacks)
        {
            pair.second(eventHeaders, contactData, frictionAnchors);
        }
    }

private:
    SubscriptionId m_nextStepSubId;
    SubscriptionId m_nextContactSubId;
    std::unordered_map<SubscriptionId, OnPhysicsStepEventFn> m_stepEventCallbacks;
    std::unordered_map<SubscriptionId, OnContactReportEventFn> m_contactEventCallbacks;
    std::atomic<int> m_stepCallbackCount;
    std::atomic<int> m_contactCallbackCount;
};

// ---------------------------------------------------------------------------
// Mock benchmark that tracks subscription callbacks
// ---------------------------------------------------------------------------
class MultiSimMockBenchmark
{
public:
    MultiSimMockBenchmark()
        : m_nextSubId(0), m_profileStatsCallbackCount(0)
    {
    }

    BenchmarkFns getBenchmarkFns()
    {
        BenchmarkFns fns;
        fns.subscribeProfileStatsEvents = [this](ProfileStatsNotificationFn onEvent) -> SubscriptionId {
            SubscriptionId id = ++m_nextSubId;
            m_profileStatsCallbacks[id] = onEvent;
            m_profileStatsCallbackCount++;
            return id;
        };
        fns.unsubscribeProfileStatsEvents = [this](SubscriptionId subscriptionId) {
            auto it = m_profileStatsCallbacks.find(subscriptionId);
            if (it != m_profileStatsCallbacks.end())
            {
                m_profileStatsCallbacks.erase(it);
                m_profileStatsCallbackCount--;
            }
        };
        return fns;
    }

    void fireProfileStats(const std::string& prefix)
    {
        std::vector<PhysicsProfileStats> stats = {
            {prefix + "_Zone1", 10.0f},
            {prefix + "_Zone2", 5.0f}
        };
        for (const auto& pair : m_profileStatsCallbacks)
        {
            pair.second(stats);
        }
    }

    int getProfileStatsCallbackCount() const { return m_profileStatsCallbackCount; }

private:
    SubscriptionId m_nextSubId;
    std::unordered_map<SubscriptionId, ProfileStatsNotificationFn> m_profileStatsCallbacks;
    std::atomic<int> m_profileStatsCallbackCount;
};

// ---------------------------------------------------------------------------
// Helper: fan-out subscribe step events to all simulations (mirrors Simulation.cpp)
// ---------------------------------------------------------------------------
struct SimSubscriptionEntry
{
    SimulationId simulationId;
    SubscriptionId subscriptionId;
};

static std::vector<SimSubscriptionEntry> subscribeStepToAll(
    SimulationRegistry& registry, bool preStep, int order, OnPhysicsStepEventFn callback)
{
    std::vector<SimSubscriptionEntry> entries;
    for (const auto& sim : registry.getSimulations())
    {
        if (sim.second.simulation.simulationFns.subscribePhysicsOnStepEvents)
        {
            SubscriptionId subId =
                sim.second.simulation.simulationFns.subscribePhysicsOnStepEvents(preStep, order, callback);
            if (subId != kInvalidSubscriptionId)
            {
                entries.push_back({sim.first, subId});
            }
        }
    }
    return entries;
}

static void unsubscribeStepFromAll(
    SimulationRegistry& registry, const std::vector<SimSubscriptionEntry>& entries)
{
    for (const auto& entry : entries)
    {
        const Simulation* sim = registry.getSimulation(entry.simulationId);
        if (sim && sim->simulationFns.unsubscribePhysicsOnStepEvents)
        {
            sim->simulationFns.unsubscribePhysicsOnStepEvents(entry.subscriptionId);
        }
    }
}

static std::vector<SimSubscriptionEntry> subscribeContactToAll(
    SimulationRegistry& registry, OnContactReportEventFn callback)
{
    std::vector<SimSubscriptionEntry> entries;
    for (const auto& sim : registry.getSimulations())
    {
        if (sim.second.simulation.simulationFns.subscribePhysicsContactReportEvents)
        {
            SubscriptionId subId =
                sim.second.simulation.simulationFns.subscribePhysicsContactReportEvents(callback);
            if (subId != kInvalidSubscriptionId)
            {
                entries.push_back({sim.first, subId});
            }
        }
    }
    return entries;
}

static void unsubscribeContactFromAll(
    SimulationRegistry& registry, const std::vector<SimSubscriptionEntry>& entries)
{
    for (const auto& entry : entries)
    {
        const Simulation* sim = registry.getSimulation(entry.simulationId);
        if (sim && sim->simulationFns.unsubscribePhysicsContactReportEvents)
        {
            sim->simulationFns.unsubscribePhysicsContactReportEvents(entry.subscriptionId);
        }
    }
}

static std::vector<SimSubscriptionEntry> subscribeBenchmarkToAll(
    SimulationRegistry& registry, ProfileStatsNotificationFn callback)
{
    std::vector<SimSubscriptionEntry> entries;
    for (const auto& sim : registry.getSimulations())
    {
        if (sim.second.simulation.benchmarkFns.subscribeProfileStatsEvents)
        {
            SubscriptionId subId =
                sim.second.simulation.benchmarkFns.subscribeProfileStatsEvents(callback);
            if (subId != kInvalidSubscriptionId)
            {
                entries.push_back({sim.first, subId});
            }
        }
    }
    return entries;
}

static void unsubscribeBenchmarkFromAll(
    SimulationRegistry& registry, const std::vector<SimSubscriptionEntry>& entries)
{
    for (const auto& entry : entries)
    {
        const Simulation* sim = registry.getSimulation(entry.simulationId);
        if (sim && sim->benchmarkFns.unsubscribeProfileStatsEvents)
        {
            sim->benchmarkFns.unsubscribeProfileStatsEvents(entry.subscriptionId);
        }
    }
}

//=============================================================================
// TEST: Multi-Simulation Subscription Fan-Out
//=============================================================================
TEST_CASE("Multi-Simulation Subscription Tests")
{
    SimulationRegistry registry;

    // Two mock simulators + benchmarks
    MultiSimMockSimulator mockSim1, mockSim2;
    MultiSimMockBenchmark mockBench1, mockBench2;

    Simulation sim1;
    sim1.simulationFns = mockSim1.getSimulationFns();
    sim1.benchmarkFns = mockBench1.getBenchmarkFns();

    Simulation sim2;
    sim2.simulationFns = mockSim2.getSimulationFns();
    sim2.benchmarkFns = mockBench2.getBenchmarkFns();

    SimulationId simId1 = registry.registerSimulation(sim1, "MockSimulator1");
    REQUIRE(simId1 != kInvalidSimulationId);

    SimulationId simId2 = registry.registerSimulation(sim2, "MockSimulator2");
    REQUIRE(simId2 != kInvalidSimulationId);

    SUBCASE("OnStep events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{0};

        OnPhysicsStepEventFn stepCallback = [&callbackCount](float, const PhysicsStepContext&) {
            callbackCount++;
        };

        // Subscribe to all simulations (mirrors umbrella fan-out)
        auto subs = subscribeStepToAll(registry, true, 0, stepCallback);
        REQUIRE(subs.size() == 2);
        REQUIRE(mockSim1.getStepCallbackCount() == 1);
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        // Fire step events from both simulations
        callbackCount = 0;
        mockSim1.fireStepCallbacks(1.0f / 60.0f);
        mockSim2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callbackCount == 2);

        // Unsubscribe from all
        unsubscribeStepFromAll(registry, subs);
        REQUIRE(mockSim1.getStepCallbackCount() == 0);
        REQUIRE(mockSim2.getStepCallbackCount() == 0);

        // Fire again — no callbacks
        callbackCount = 0;
        mockSim1.fireStepCallbacks(1.0f / 60.0f);
        mockSim2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callbackCount == 0);
    }

    SUBCASE("Contact report events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{0};

        OnContactReportEventFn contactCallback =
            [&callbackCount](const ContactEventHeaderVector&, const ContactDataVector&,
                             const FrictionAnchorsDataVector&) {
                callbackCount++;
            };

        auto subs = subscribeContactToAll(registry, contactCallback);
        REQUIRE(subs.size() == 2);
        REQUIRE(mockSim1.getContactCallbackCount() == 1);
        REQUIRE(mockSim2.getContactCallbackCount() == 1);

        callbackCount = 0;
        mockSim1.fireContactCallbacks();
        mockSim2.fireContactCallbacks();
        REQUIRE(callbackCount == 2);

        unsubscribeContactFromAll(registry, subs);
        REQUIRE(mockSim1.getContactCallbackCount() == 0);
        REQUIRE(mockSim2.getContactCallbackCount() == 0);

        callbackCount = 0;
        mockSim1.fireContactCallbacks();
        mockSim2.fireContactCallbacks();
        REQUIRE(callbackCount == 0);
    }

    SUBCASE("Profile stats events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{0};
        std::vector<std::string> receivedZones;

        ProfileStatsNotificationFn statsCallback = [&](const std::vector<PhysicsProfileStats>& stats) {
            callbackCount++;
            for (const auto& stat : stats)
            {
                receivedZones.push_back(stat.zoneName);
            }
        };

        auto subs = subscribeBenchmarkToAll(registry, statsCallback);
        REQUIRE(subs.size() == 2);
        REQUIRE(mockBench1.getProfileStatsCallbackCount() == 1);
        REQUIRE(mockBench2.getProfileStatsCallbackCount() == 1);

        callbackCount = 0;
        receivedZones.clear();
        mockBench1.fireProfileStats("Benchmark1");
        mockBench2.fireProfileStats("Benchmark2");
        REQUIRE(callbackCount == 2);

        // Verify we received zones from both benchmarks
        bool foundBench1Zone = false;
        bool foundBench2Zone = false;
        for (const auto& zone : receivedZones)
        {
            if (zone.find("Benchmark1") != std::string::npos) foundBench1Zone = true;
            if (zone.find("Benchmark2") != std::string::npos) foundBench2Zone = true;
        }
        REQUIRE(foundBench1Zone);
        REQUIRE(foundBench2Zone);

        unsubscribeBenchmarkFromAll(registry, subs);
        REQUIRE(mockBench1.getProfileStatsCallbackCount() == 0);
        REQUIRE(mockBench2.getProfileStatsCallbackCount() == 0);

        callbackCount = 0;
        mockBench1.fireProfileStats("Benchmark1");
        mockBench2.fireProfileStats("Benchmark2");
        REQUIRE(callbackCount == 0);
    }

    SUBCASE("Multiple subscriptions across multiple simulations")
    {
        std::atomic<int> callback1Count{0};
        std::atomic<int> callback2Count{0};

        OnPhysicsStepEventFn stepCallback1 = [&callback1Count](float, const PhysicsStepContext&) {
            callback1Count++;
        };
        OnPhysicsStepEventFn stepCallback2 = [&callback2Count](float, const PhysicsStepContext&) {
            callback2Count++;
        };

        auto subs1 = subscribeStepToAll(registry, true, 0, stepCallback1);
        auto subs2 = subscribeStepToAll(registry, false, 1, stepCallback2);

        REQUIRE(mockSim1.getStepCallbackCount() == 2);
        REQUIRE(mockSim2.getStepCallbackCount() == 2);

        mockSim1.fireStepCallbacks(1.0f / 60.0f);
        mockSim2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callback1Count == 2);
        REQUIRE(callback2Count == 2);

        // Unsubscribe first only
        unsubscribeStepFromAll(registry, subs1);
        REQUIRE(mockSim1.getStepCallbackCount() == 1);
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        callback1Count = 0;
        callback2Count = 0;
        mockSim1.fireStepCallbacks(1.0f / 60.0f);
        mockSim2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callback1Count == 0);
        REQUIRE(callback2Count == 2);

        unsubscribeStepFromAll(registry, subs2);
        REQUIRE(mockSim1.getStepCallbackCount() == 0);
        REQUIRE(mockSim2.getStepCallbackCount() == 0);
    }

    SUBCASE("Subscription with one inactive simulation")
    {
        // Deactivate second simulation
        registry.deactivateSimulation(simId2);

        std::atomic<int> callbackCount{0};
        OnPhysicsStepEventFn stepCallback = [&callbackCount](float, const PhysicsStepContext&) {
            callbackCount++;
        };

        // Subscribe to all — both should get subscribed (subscription doesn't check active)
        auto subs = subscribeStepToAll(registry, true, 0, stepCallback);
        REQUIRE(subs.size() == 2);
        REQUIRE(mockSim1.getStepCallbackCount() == 1);
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        // Only fire from active simulation
        callbackCount = 0;
        for (const auto& sim : registry.getSimulations())
        {
            if (sim.second.isActive && sim.second.simulation.simulationFns.simulate)
            {
                // Firing step callbacks mimics what simulate() does through the mocks
            }
        }
        // Active simulation fires
        mockSim1.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callbackCount == 1);

        // Inactive sim2 would not normally be called by umbrella simulate()
        // but the mock still has the subscription registered
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        unsubscribeStepFromAll(registry, subs);
        REQUIRE(mockSim1.getStepCallbackCount() == 0);
        REQUIRE(mockSim2.getStepCallbackCount() == 0);

        // Reactivate for cleanup
        registry.activateSimulation(simId2);
    }

    // Cleanup
    registry.unregisterSimulation(simId1);
    registry.unregisterSimulation(simId2);
}
