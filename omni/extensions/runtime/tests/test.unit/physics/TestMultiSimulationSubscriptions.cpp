// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsBenchmark.h>
#include <cstring>
#include <atomic>

using namespace omni::physics;

// Mock simulator for multi-simulation testing
class MultiSimMockSimulator
{
public:
    MultiSimMockSimulator(const std::string& name)
        : m_name(name)
        , m_attachedStageId(0)
        , m_stepCallbackCount(0)
        , m_contactCallbackCount(0)
    {
        // Initialize function pointers
        initialize = [this](long id) -> bool {
            m_attachedStageId = id;
            return true;
        };

        close = [this]() {
            m_attachedStageId = 0;
        };

        getAttachedStage = [this]() -> long {
            return m_attachedStageId;
        };

        simulateAsync = [this](float elapsedTime, float currentTime) {
            PhysicsStepContext context;
            context.scenePath = m_attachedStageId;
            context.simulationId = SimulationId(1);

            for (const auto& pair : m_stepEventCallbacks) {
                pair.second(elapsedTime, context);
            }
        };

        simulate = [this](float elapsedTime, float currentTime) {
            simulateAsync(elapsedTime, currentTime);
            fetchResults();
        };

        fetchResults = [this]() {
            const ContactEventHeaderVector eventHeaders;
            const ContactDataVector contactData;
            const FrictionAnchorsDataVector frictionAnchors;
            for (const auto& pair : m_contactEventCallbacks) {
                pair.second(eventHeaders, contactData, frictionAnchors);
            }
        };

        checkResults = []() -> bool {
            return true;
        };

        flushChanges = []() {};
        pauseChangeTracking = [](bool) {};
        isChangeTrackingPaused = []() -> bool { return false; };

        subscribePhysicsContactReportEvents = [this](OnContactReportEventFn onEvent) -> SubscriptionId {
            SubscriptionId id = ++m_nextContactSubId;
            m_contactEventCallbacks[id] = onEvent;
            m_contactCallbackCount++;
            return id;
        };

        unsubscribePhysicsContactReportEvents = [this](SubscriptionId subscriptionId) {
            auto it = m_contactEventCallbacks.find(subscriptionId);
            if (it != m_contactEventCallbacks.end()) {
                m_contactEventCallbacks.erase(it);
                m_contactCallbackCount--;
            }
        };

        getSimulationTimeStepsPerSecond = [](long, uint64_t) -> uint32_t { return 60; };
        getSimulationTimestamp = []() -> uint64_t { return 0; };
        getSimulationStepCount = []() -> uint64_t { return 0; };

        subscribePhysicsOnStepEvents = [this](bool preStep, int order, OnPhysicsStepEventFn onUpdate) -> SubscriptionId {
            SubscriptionId id = ++m_nextStepSubId;
            m_stepEventCallbacks[id] = onUpdate;
            m_stepCallbackCount++;
            return id;
        };

        unsubscribePhysicsOnStepEvents = [this](SubscriptionId subscriptionId) {
            auto it = m_stepEventCallbacks.find(subscriptionId);
            if (it != m_stepEventCallbacks.end()) {
                m_stepEventCallbacks.erase(it);
                m_stepCallbackCount--;
            }
        };
    }

    SimulationFns getSimulationFns() const
    {
        SimulationFns fns;
        fns.initialize = initialize;
        fns.close = close;
        fns.getAttachedStage = getAttachedStage;
        fns.simulateAsync = simulateAsync;
        fns.simulate = simulate;
        fns.fetchResults = fetchResults;
        fns.checkResults = checkResults;
        fns.flushChanges = flushChanges;
        fns.pauseChangeTracking = pauseChangeTracking;
        fns.isChangeTrackingPaused = isChangeTrackingPaused;
        fns.subscribePhysicsContactReportEvents = subscribePhysicsContactReportEvents;
        fns.unsubscribePhysicsContactReportEvents = unsubscribePhysicsContactReportEvents;
        fns.getSimulationTimeStepsPerSecond = getSimulationTimeStepsPerSecond;
        fns.getSimulationTimestamp = getSimulationTimestamp;
        fns.getSimulationStepCount = getSimulationStepCount;
        fns.subscribePhysicsOnStepEvents = subscribePhysicsOnStepEvents;
        fns.unsubscribePhysicsOnStepEvents = unsubscribePhysicsOnStepEvents;
        return fns;
    }

    const std::string& getName() const { return m_name; }
    int getStepCallbackCount() const { return m_stepCallbackCount; }
    int getContactCallbackCount() const { return m_contactCallbackCount; }

    // Function pointers
    AttachStageFn initialize;
    DetachStageFn close;
    GetAttachedStageFn getAttachedStage;
    SimulateFn simulateAsync;
    SimulateFn simulate;
    FetchResultsFn fetchResults;
    CheckResultsFn checkResults;
    FlushChangesFn flushChanges;
    PauseChangeTrackingFn pauseChangeTracking;
    IsChangeTrackingPausedFn isChangeTrackingPaused;
    SubscribePhysicsContactReportEventsFn subscribePhysicsContactReportEvents;
    UnsubscribePhysicsContactReportEventsFn unsubscribePhysicsContactReportEvents;
    GetSimulationTimeStepsPerSecondFn getSimulationTimeStepsPerSecond;
    GetSimulationTimestampFn getSimulationTimestamp;
    GetSimulationStepCountFn getSimulationStepCount;
    SubscribePhysicsOnStepEventsFn subscribePhysicsOnStepEvents;
    UnsubscribePhysicsOnStepEventsFn unsubscribePhysicsOnStepEvents;

private:
    std::string m_name;
    long m_attachedStageId;
    SubscriptionId m_nextStepSubId = 0;
    SubscriptionId m_nextContactSubId = 0;
    std::unordered_map<SubscriptionId, OnPhysicsStepEventFn> m_stepEventCallbacks;
    std::unordered_map<SubscriptionId, OnContactReportEventFn> m_contactEventCallbacks;
    std::atomic<int> m_stepCallbackCount;
    std::atomic<int> m_contactCallbackCount;
};

// Mock benchmark for multi-simulation testing
class MultiSimMockBenchmark
{
public:
    MultiSimMockBenchmark(const std::string& name)
        : m_name(name)
        , m_profileStatsCallbackCount(0)
    {
        subscribeProfileStatsEvents = [this](ProfileStatsNotificationFn onEvent) -> SubscriptionId {
            SubscriptionId id = ++m_nextSubId;
            m_profileStatsCallbacks[id] = onEvent;
            m_profileStatsCallbackCount++;
            return id;
        };

        unsubscribeProfileStatsEvents = [this](SubscriptionId subscriptionId) {
            auto it = m_profileStatsCallbacks.find(subscriptionId);
            if (it != m_profileStatsCallbacks.end()) {
                m_profileStatsCallbacks.erase(it);
                m_profileStatsCallbackCount--;
            }
        };
    }

    BenchmarkFns getBenchmarkFns() const
    {
        BenchmarkFns fns;
        fns.subscribeProfileStatsEvents = subscribeProfileStatsEvents;
        fns.unsubscribeProfileStatsEvents = unsubscribeProfileStatsEvents;
        return fns;
    }

    void simulateProfileStats()
    {
        std::vector<PhysicsProfileStats> stats = {
            {m_name + "_Zone1", 10.0f},
            {m_name + "_Zone2", 5.0f}
        };
        for (const auto& pair : m_profileStatsCallbacks) {
            pair.second(stats);
        }
    }

    const std::string& getName() const { return m_name; }
    int getProfileStatsCallbackCount() const { return m_profileStatsCallbackCount; }

    SubscribeProfileStatsEventsFn subscribeProfileStatsEvents;
    UnsubscribeProfileStatsEventsFn unsubscribeProfileStatsEvents;

private:
    std::string m_name;
    SubscriptionId m_nextSubId = 0;
    std::unordered_map<SubscriptionId, ProfileStatsNotificationFn> m_profileStatsCallbacks;
    std::atomic<int> m_profileStatsCallbackCount;
};

//-----------------------------------------------------------------------------
// Multi-Simulation Subscription Tests
TEST_CASE("Multi-Simulation Subscription Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsSimulation* simulation = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsSimulation>();
    REQUIRE(simulation);

    IPhysicsBenchmarks* benchmark = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsBenchmarks>();
    REQUIRE(benchmark);

    // Create two mock simulators
    MultiSimMockSimulator mockSim1("Simulator1");
    MultiSimMockSimulator mockSim2("Simulator2");
    MultiSimMockBenchmark mockBench1("Benchmark1");
    MultiSimMockBenchmark mockBench2("Benchmark2");

    Simulation sim1;
    sim1.simulationFns = mockSim1.getSimulationFns();
    sim1.benchmarkFns = mockBench1.getBenchmarkFns();

    Simulation sim2;
    sim2.simulationFns = mockSim2.getSimulationFns();
    sim2.benchmarkFns = mockBench2.getBenchmarkFns();

    // Register both simulations
    SimulationId simId1 = physics->registerSimulation(sim1, "MockSimulator1");
    REQUIRE(simId1 != kInvalidSimulationId);

    SimulationId simId2 = physics->registerSimulation(sim2, "MockSimulator2");
    REQUIRE(simId2 != kInvalidSimulationId);

    SECTION("OnStep events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{0};
        
        OnPhysicsStepEventFn stepCallback = [&callbackCount](float elapsedTime, const PhysicsStepContext& context) {
            callbackCount++;
        };

        // Subscribe - should subscribe to both simulations
        SubscriptionId subId = simulation->subscribePhysicsOnStepEvents(true, 0, stepCallback);
        REQUIRE(subId != kInvalidSubscriptionId);

        // Verify both simulations have the callback registered
        REQUIRE(mockSim1.getStepCallbackCount() == 1);
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        // Simulate and verify callbacks from both simulations
        callbackCount = 0;
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        REQUIRE(callbackCount == 2);

        // Unsubscribe - should unsubscribe from both simulations
        simulation->unsubscribePhysicsOnStepEvents(subId);

        // Verify both simulations have the callback removed
        REQUIRE(mockSim1.getStepCallbackCount() == 0);
        REQUIRE(mockSim2.getStepCallbackCount() == 0);

        // Simulate and verify no callbacks
        callbackCount = 0;
        simulation->simulateAsync(1.0f/60.0f, 1.0f/60.0f);
        REQUIRE(callbackCount == 0);
    }

    SECTION("Contact report events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{0};

        OnContactReportEventFn contactCallback =
            [&callbackCount](const ContactEventHeaderVector&, const ContactDataVector&,
                const FrictionAnchorsDataVector&) {
                callbackCount++;
            };

        // Subscribe - should subscribe to both simulations
        SubscriptionId subId = simulation->subscribePhysicsContactReportEvents(contactCallback);
        REQUIRE(subId != kInvalidSubscriptionId);

        // Verify both simulations have the callback registered
        REQUIRE(mockSim1.getContactCallbackCount() == 1);
        REQUIRE(mockSim2.getContactCallbackCount() == 1);

        // Fetch results and verify callbacks from both simulations
        callbackCount = 0;
        simulation->fetchResults();
        REQUIRE(callbackCount == 2);

        // Unsubscribe - should unsubscribe from both simulations
        simulation->unsubscribePhysicsContactReportEvents(subId);

        // Verify both simulations have the callback removed
        REQUIRE(mockSim1.getContactCallbackCount() == 0);
        REQUIRE(mockSim2.getContactCallbackCount() == 0);

        // Fetch results and verify no callbacks
        callbackCount = 0;
        simulation->fetchResults();
        REQUIRE(callbackCount == 0);
    }

    SECTION("Profile stats events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{0};
        std::vector<std::string> receivedZones;
        std::mutex zonesMutex;

        ProfileStatsNotificationFn statsCallback = [&](const std::vector<PhysicsProfileStats>& stats) {
            callbackCount++;
            std::lock_guard<std::mutex> lock(zonesMutex);
            for (const auto& stat : stats) {
                receivedZones.push_back(stat.zoneName);
            }
        };

        // Subscribe - should subscribe to both simulations
        SubscriptionId subId = benchmark->subscribeProfileStatsEvents(statsCallback);
        REQUIRE(subId != kInvalidSubscriptionId);

        // Verify both benchmarks have the callback registered
        REQUIRE(mockBench1.getProfileStatsCallbackCount() == 1);
        REQUIRE(mockBench2.getProfileStatsCallbackCount() == 1);

        // Simulate profile stats and verify callbacks from both
        callbackCount = 0;
        receivedZones.clear();
        mockBench1.simulateProfileStats();
        mockBench2.simulateProfileStats();
        REQUIRE(callbackCount == 2);

        // Verify we received zones from both benchmarks
        bool foundBench1Zone = false;
        bool foundBench2Zone = false;
        for (const auto& zone : receivedZones) {
            if (zone.find("Benchmark1") != std::string::npos) foundBench1Zone = true;
            if (zone.find("Benchmark2") != std::string::npos) foundBench2Zone = true;
        }
        REQUIRE(foundBench1Zone);
        REQUIRE(foundBench2Zone);

        // Unsubscribe - should unsubscribe from both
        benchmark->unsubscribeProfileStatsEvents(subId);

        // Verify both benchmarks have the callback removed
        REQUIRE(mockBench1.getProfileStatsCallbackCount() == 0);
        REQUIRE(mockBench2.getProfileStatsCallbackCount() == 0);

        // Simulate and verify no callbacks
        callbackCount = 0;
        mockBench1.simulateProfileStats();
        mockBench2.simulateProfileStats();
        REQUIRE(callbackCount == 0);
    }

    SECTION("Multiple subscriptions across multiple simulations")
    {
        std::atomic<int> callback1Count{0};
        std::atomic<int> callback2Count{0};

        OnPhysicsStepEventFn stepCallback1 = [&callback1Count](float, const PhysicsStepContext&) {
            callback1Count++;
        };

        OnPhysicsStepEventFn stepCallback2 = [&callback2Count](float, const PhysicsStepContext&) {
            callback2Count++;
        };

        // Create two separate subscriptions
        SubscriptionId subId1 = simulation->subscribePhysicsOnStepEvents(true, 0, stepCallback1);
        SubscriptionId subId2 = simulation->subscribePhysicsOnStepEvents(false, 1, stepCallback2);

        REQUIRE(subId1 != kInvalidSubscriptionId);
        REQUIRE(subId2 != kInvalidSubscriptionId);
        REQUIRE(subId1 != subId2);

        // Each simulation should have 2 callbacks registered
        REQUIRE(mockSim1.getStepCallbackCount() == 2);
        REQUIRE(mockSim2.getStepCallbackCount() == 2);

        // Simulate and verify both callbacks called from both simulations
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        REQUIRE(callback1Count == 2);
        REQUIRE(callback2Count == 2);

        // Unsubscribe first subscription only
        simulation->unsubscribePhysicsOnStepEvents(subId1);
        REQUIRE(mockSim1.getStepCallbackCount() == 1);
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        // Simulate and verify only second callback is called
        callback1Count = 0;
        callback2Count = 0;
        simulation->simulateAsync(1.0f/60.0f, 1.0f/60.0f);
        REQUIRE(callback1Count == 0);
        REQUIRE(callback2Count == 2);

        // Unsubscribe second subscription
        simulation->unsubscribePhysicsOnStepEvents(subId2);
        REQUIRE(mockSim1.getStepCallbackCount() == 0);
        REQUIRE(mockSim2.getStepCallbackCount() == 0);
    }

    SECTION("Unsubscribe with invalid subscription ID")
    {
        // Should not crash or affect existing subscriptions
        simulation->unsubscribePhysicsOnStepEvents(kInvalidSubscriptionId);
        simulation->unsubscribePhysicsContactReportEvents(kInvalidSubscriptionId);
        benchmark->unsubscribeProfileStatsEvents(kInvalidSubscriptionId);

        // Should not crash with non-existent IDs
        simulation->unsubscribePhysicsOnStepEvents(999);
        simulation->unsubscribePhysicsContactReportEvents(999);
        benchmark->unsubscribeProfileStatsEvents(999);
    }

    SECTION("Subscription with one inactive simulation")
    {
        // Deactivate second simulation
        physics->deactivateSimulation(simId2);

        std::atomic<int> callbackCount{0};
        OnPhysicsStepEventFn stepCallback = [&callbackCount](float, const PhysicsStepContext&) {
            callbackCount++;
        };

        // Subscribe - should subscribe to both active and inactive simulation
        SubscriptionId subId = simulation->subscribePhysicsOnStepEvents(true, 0, stepCallback);
        REQUIRE(subId != kInvalidSubscriptionId);

        // Both simulation should have the callback
        REQUIRE(mockSim1.getStepCallbackCount() == 1);
        REQUIRE(mockSim2.getStepCallbackCount() == 1);

        // Simulate and verify callback only from first simulation
        callbackCount = 0;
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        REQUIRE(callbackCount == 1);

        // Unsubscribe
        simulation->unsubscribePhysicsOnStepEvents(subId);
        REQUIRE(mockSim1.getStepCallbackCount() == 0);
        REQUIRE(mockSim2.getStepCallbackCount() == 0);

        // Reactivate for cleanup
        physics->activateSimulation(simId2);
    }

    // Cleanup
    physics->unregisterSimulation(simId1);
    physics->unregisterSimulation(simId2);
}
