// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsBenchmark.h>
#include <cstring>

using namespace omni::physics;

// Mock benchmark implementation
class MockBenchmark
{
public:
    MockBenchmark()
        : m_subscriptionCount(0)
        , m_lastSubscriptionId(kInvalidSubscriptionId)
        , m_profileStatsCallback(nullptr)
    {
        // Initialize function pointers
        subscribeProfileStatsEvents = [this](ProfileStatsNotificationFn onEvent) -> SubscriptionId {
            m_profileStatsCallback = onEvent;
            m_lastSubscriptionId = ++m_subscriptionCount;
            return m_lastSubscriptionId;
        };

        unsubscribeProfileStatsEvents = [this](SubscriptionId subscriptionId) {
            if (subscriptionId == m_lastSubscriptionId) {
                m_profileStatsCallback = nullptr;
                m_lastSubscriptionId = kInvalidSubscriptionId;
            }
        };
    }

    ~MockBenchmark()
    {
        // Cleanup
    }

    // Function pointers that match BenchmarkFns structure
    SubscribeProfileStatsEventsFn subscribeProfileStatsEvents;
    UnsubscribeProfileStatsEventsFn unsubscribeProfileStatsEvents;

    // Get the function pointers structure
    BenchmarkFns getBenchmarkFns() const
    {
        BenchmarkFns fns;
        fns.subscribeProfileStatsEvents = subscribeProfileStatsEvents;
        fns.unsubscribeProfileStatsEvents = unsubscribeProfileStatsEvents;
        return fns;
    }

    // Helper methods for testing
    void simulateProfileStats()
    {
        if (m_profileStatsCallback)
        {
            std::vector<PhysicsProfileStats> stats = {
                {"Simulation", 16.6f},
                {"Collision Detection", 5.2f},
                {"Integration", 2.1f}
            };
            m_profileStatsCallback(stats);
        }
    }

    bool hasActiveSubscription() const 
    { 
        return m_profileStatsCallback != nullptr; 
    }

    SubscriptionId getLastSubscriptionId() const 
    { 
        return m_lastSubscriptionId; 
    }

private:
    // Internal state
    uint32_t m_subscriptionCount;
    SubscriptionId m_lastSubscriptionId;
    ProfileStatsNotificationFn m_profileStatsCallback;
};

//-----------------------------------------------------------------------------
// Benchmark Tests
TEST_CASE("Simulator Benchmark Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsBenchmarks* benchmark = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsBenchmarks>();
    REQUIRE(benchmark);

    // Create a mock benchmark
    MockBenchmark mockBenchmark;
    BenchmarkFns fns = mockBenchmark.getBenchmarkFns();

    Simulation sim;
    sim.benchmarkFns = fns;

    SimulationId simId = physics->registerSimulation(sim, "MockBenchmarkSimulator");
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Profile stats subscription")
    {
        bool callbackCalled = false;
        std::vector<PhysicsProfileStats> receivedStats;
        
        ProfileStatsNotificationFn testCallback = [&callbackCalled, &receivedStats](const std::vector<PhysicsProfileStats>& stats) {
            callbackCalled = true;
            receivedStats = stats;
        };
        
        // Subscribe to profile stats events
        SubscriptionId subscriptionId = benchmark->subscribeProfileStatsEvents(testCallback);
        REQUIRE(subscriptionId != kInvalidSubscriptionId);
        REQUIRE(mockBenchmark.hasActiveSubscription());
        
        // Simulate profile stats being generated
        mockBenchmark.simulateProfileStats();
        
        // Verify callback was called with correct data
        REQUIRE(callbackCalled);
        REQUIRE(receivedStats.size() == 3);
        REQUIRE(strcmp(receivedStats[0].zoneName.c_str(), "Simulation") == 0);
        REQUIRE(receivedStats[0].ms == 16.6f);
        REQUIRE(strcmp(receivedStats[1].zoneName.c_str(), "Collision Detection") == 0);
        REQUIRE(receivedStats[1].ms == 5.2f);
        REQUIRE(strcmp(receivedStats[2].zoneName.c_str(), "Integration") == 0);
        REQUIRE(receivedStats[2].ms == 2.1f);
        
        // Unsubscribe from profile stats events
        benchmark->unsubscribeProfileStatsEvents(subscriptionId);
        REQUIRE_FALSE(mockBenchmark.hasActiveSubscription());
    }

    SECTION("Multiple subscriptions")
    {
        bool callback1Called = false;
        bool callback2Called = false;
        
        ProfileStatsNotificationFn callback1 = [&callback1Called](const std::vector<PhysicsProfileStats>&) {
            callback1Called = true;
        };
        
        ProfileStatsNotificationFn callback2 = [&callback2Called](const std::vector<PhysicsProfileStats>&) {
            callback2Called = true;
        };
        
        // Test that only the last subscription is active (limitation of current mock)
        SubscriptionId sub1 = benchmark->subscribeProfileStatsEvents(callback1);
        REQUIRE(sub1 != kInvalidSubscriptionId);
        
        SubscriptionId sub2 = benchmark->subscribeProfileStatsEvents(callback2);
        REQUIRE(sub2 != kInvalidSubscriptionId);
        REQUIRE(sub2 != sub1);
        
        // Simulate stats - only callback2 should be called in this mock implementation
        mockBenchmark.simulateProfileStats();
        REQUIRE_FALSE(callback1Called);
        REQUIRE(callback2Called);
        
        // Cleanup
        benchmark->unsubscribeProfileStatsEvents(sub2);
        REQUIRE_FALSE(mockBenchmark.hasActiveSubscription());
    }

    SECTION("Unsubscribe invalid subscription")
    {
        // Test unsubscribing with invalid subscription ID
        benchmark->unsubscribeProfileStatsEvents(kInvalidSubscriptionId);
        REQUIRE_FALSE(mockBenchmark.hasActiveSubscription());
        
        // Test unsubscribing with non-existent subscription ID
        benchmark->unsubscribeProfileStatsEvents(999);
        REQUIRE_FALSE(mockBenchmark.hasActiveSubscription());
    }

    SECTION("Profile stats data validation")
    {
        std::vector<PhysicsProfileStats> receivedStats;
        
        ProfileStatsNotificationFn testCallback = [&receivedStats](const std::vector<PhysicsProfileStats>& stats) {
            receivedStats = stats;
        };
        
        SubscriptionId subscriptionId = benchmark->subscribeProfileStatsEvents(testCallback);
        REQUIRE(subscriptionId != kInvalidSubscriptionId);
        
        // Simulate profile stats
        mockBenchmark.simulateProfileStats();
        
        // Validate profile stats data structure
        REQUIRE(receivedStats.size() > 0);
        for (const auto& stat : receivedStats)
        {
            REQUIRE(stat.zoneName.length() > 0);
            REQUIRE(stat.ms >= 0.0f);
        }
        
        // Cleanup
        benchmark->unsubscribeProfileStatsEvents(subscriptionId);
    }

    SECTION("No callback after unsubscribe")
    {
        bool callbackCalled = false;
        
        ProfileStatsNotificationFn testCallback = [&callbackCalled](const std::vector<PhysicsProfileStats>&) {
            callbackCalled = true;
        };
        
        SubscriptionId subscriptionId = benchmark->subscribeProfileStatsEvents(testCallback);
        REQUIRE(subscriptionId != kInvalidSubscriptionId);
        
        // Unsubscribe immediately
        benchmark->unsubscribeProfileStatsEvents(subscriptionId);
        REQUIRE_FALSE(mockBenchmark.hasActiveSubscription());
        
        // Simulate stats - callback should not be called
        mockBenchmark.simulateProfileStats();
        REQUIRE_FALSE(callbackCalled);
    }

    physics->unregisterSimulation(simId);
}
