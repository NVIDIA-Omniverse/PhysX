// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>


using namespace omni::physics;

// Mock simulator implementation
class MockSimulator
{
public:
    MockSimulator()
        : m_attachedStageId(0)
        , m_isChangeTrackingPaused(false)
        , m_timeStepsPerSecond(60)
        , m_simulationTimestamp(0)
        , m_simulationStepCount(0)
    {
        // Initialize function pointers
        attachStage = [this](long id) -> bool {
            m_attachedStageId = id;
            return true;
        };

        detachStage = [this]() {
            m_attachedStageId = 0;
        };

        getAttachedStage = [this]() -> long {
            return m_attachedStageId;
        };

        simulate = [this](float elapsedTime, float currentTime) {
            m_simulationTimestamp++;
            m_simulationStepCount++;
            
            // Call step event callbacks
            PhysicsStepContext context;
            context.scenePath = m_attachedStageId;
            context.simulationId = SimulationId(m_simulationTimestamp);

            for (const auto& callback : m_stepEventCallbacks) {
                callback(elapsedTime, context);
            }

        };

        fetchResults = [this]() {
            // Mock implementation - do nothing
            const ContactEventHeaderVector eventHeaders;
            const ContactDataVector contactData;
            const FrictionAnchorsDataVector frictionAnchors;
            for (const auto& callback : m_contactEventCallbacks)
            {
                callback(eventHeaders, contactData, frictionAnchors);
            }
        };

        checkResults = []() -> bool {
            return true;
        };

        flushChanges = []() {
            // Mock implementation - do nothing
        };

        pauseChangeTracking = [this](bool pause) {
            m_isChangeTrackingPaused = pause;
        };

        isChangeTrackingPaused = [this]() -> bool {
            return m_isChangeTrackingPaused;
        };

        subscribePhysicsContactReportEvents = [this](OnContactReportEventFn onEvent) -> SubscriptionId {
            m_contactEventCallbacks.emplace_back(onEvent);
            return m_contactEventCallbacks.size();
        };

        unsubscribePhysicsContactReportEvents = [this](SubscriptionId subscriptionId) {
            if (subscriptionId > 0 && subscriptionId <= m_contactEventCallbacks.size()) {
                m_contactEventCallbacks.erase(m_contactEventCallbacks.begin() + subscriptionId - 1);
            }
        };

        getSimulationTimeStepsPerSecond = [this](long stageId, uint64_t scenePath) -> uint32_t {
            return m_timeStepsPerSecond;
        };

        getSimulationTimestamp = [this]() -> uint64_t {
            return m_simulationTimestamp;
        };

        getSimulationStepCount = [this]() -> uint64_t {
            return m_simulationStepCount;
        };

        addForceAtPos = [](uint64_t stageId, uint64_t path, const carb::Float3& force, const carb::Float3& pos, ForceModeType::Enum mode) {
            // Mock implementation - do nothing
        };

        addTorque = [](uint64_t stageId, uint64_t path, const carb::Float3& torque) {
            // Mock implementation - do nothing
        };

        wakeUp = [this](uint64_t stageId, uint64_t path) {
            m_sleepingBodies[path] = false;
        };

        putToSleep = [this](uint64_t stageId, uint64_t path) {
            m_sleepingBodies[path] = true;
        };

        isSleeping = [this](uint64_t stageId, uint64_t path) -> bool {
            auto it = m_sleepingBodies.find(path);
            return it != m_sleepingBodies.end() && it->second;
        };

        subscribePhysicsOnStepEvents = [this](bool preStep, int order, OnPhysicsStepEventFn onUpdate) -> SubscriptionId {
            m_stepEventCallbacks.emplace_back(onUpdate);
            return m_stepEventCallbacks.size();
        };

        unsubscribePhysicsOnStepEvents = [this](SubscriptionId subscriptionId) {
            if (subscriptionId > 0 && subscriptionId <= m_stepEventCallbacks.size()) {
                m_stepEventCallbacks.erase(m_stepEventCallbacks.begin() + subscriptionId - 1);
            }
        };
    }

    ~MockSimulator()
    {
        // Cleanup
        m_stepEventCallbacks.clear();
        m_contactEventCallbacks.clear();
        m_sleepingBodies.clear();
    }

    // Function pointers that match SimulationFns structure
    AttachStageFn attachStage;
    DetachStageFn detachStage;
    GetAttachedStageFn getAttachedStage;
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
    AddForceAtPosFn addForceAtPos;
    AddTorqueFn addTorque;
    WakeUpFn wakeUp;
    PutToSleepFn putToSleep;
    IsSleepingFn isSleeping;
    SubscribePhysicsOnStepEventsFn subscribePhysicsOnStepEvents;
    UnsubscribePhysicsOnStepEventsFn unsubscribePhysicsOnStepEvents;

    // Get the function pointers structure
    SimulationFns getSimulationFns() const
    {
        SimulationFns fns;
        fns.attachStage = attachStage;
        fns.detachStage = detachStage;
        fns.getAttachedStage = getAttachedStage;
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
        fns.addForceAtPos = addForceAtPos;
        fns.addTorque = addTorque;
        fns.wakeUp = wakeUp;
        fns.putToSleep = putToSleep;
        fns.isSleeping = isSleeping;
        fns.subscribePhysicsOnStepEvents = subscribePhysicsOnStepEvents;
        fns.unsubscribePhysicsOnStepEvents = unsubscribePhysicsOnStepEvents;
        return fns;
    }

private:
    // Internal state
    long m_attachedStageId;
    bool m_isChangeTrackingPaused;
    uint32_t m_timeStepsPerSecond;
    uint64_t m_simulationTimestamp;
    uint64_t m_simulationStepCount;
    std::unordered_map<uint64_t, bool> m_sleepingBodies;
    std::vector<OnPhysicsStepEventFn> m_stepEventCallbacks;
    std::vector<OnContactReportEventFn> m_contactEventCallbacks;
};



//-----------------------------------------------------------------------------
// Simulator Simulation Tests
TEST_CASE("Simulator Simulation Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsSimulation* simulation = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsSimulation>();
    REQUIRE(simulation);

    // Create a mock simulator
    MockSimulator mockSimulator;
    SimulationFns fns = mockSimulator.getSimulationFns();

    Simulation sim;
    sim.simulationFns = fns;

    SimulationId simId = physics->registerSimulation(sim, "MockupSimulator");
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Stage attachment")
    {
        REQUIRE(simulation->attachStage(123));
        REQUIRE(simulation->getAttachedStage() == 123);
        simulation->detachStage();
        REQUIRE(simulation->getAttachedStage() == 0);
    }

    SECTION("Simulation timing")
    {
        const long stageId = 123;
        const uint64_t scenePath = 0;
        REQUIRE(simulation->getSimulationTimeStepsPerSecond(simId, stageId, scenePath) == 60);
        REQUIRE(simulation->getSimulationTimestamp(simId) == 0);
        REQUIRE(simulation->getSimulationStepCount(simId) == 0);

        simulation->simulate(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        REQUIRE(simulation->getSimulationTimestamp(simId) == 1);
        REQUIRE(simulation->getSimulationStepCount(simId) == 1);

        // Test multiple simulation steps
        for (int i = 0; i < 5; i++) {
            simulation->simulate(1.0f/60.0f, (i + 1) * 1.0f/60.0f);
            simulation->fetchResults();
        }

        REQUIRE(simulation->getSimulationTimestamp(simId) == 6);
        REQUIRE(simulation->getSimulationStepCount(simId) == 6);
    }

    SECTION("Simulation events")
    {
        bool preStepCalled = false;
        bool postStepCalled = false;
        uint64_t preStepScenePath = 0;
        uint64_t postStepScenePath = 0;
        SimulationId preStepSimId;
        SimulationId postStepSimId;

        OnPhysicsStepEventFn preStepCallback = [&](float elapsedTime, const PhysicsStepContext& context) {
            preStepCalled = true;
            preStepScenePath = context.scenePath;
            preStepSimId = context.simulationId;
            return true;
        };

        OnPhysicsStepEventFn postStepCallback = [&](float elapsedTime, const PhysicsStepContext& context) {
            postStepCalled = true;
            postStepScenePath = context.scenePath;
            postStepSimId = context.simulationId;
            return true;
        };

        // Subscribe to pre and post step events
        SubscriptionId preStepId = simulation->subscribePhysicsOnStepEvents(true, 0, preStepCallback);
        SubscriptionId postStepId = simulation->subscribePhysicsOnStepEvents(false, 1, postStepCallback);

        REQUIRE(preStepId != kInvalidSubscriptionId);
        REQUIRE(postStepId != kInvalidSubscriptionId);

        // Run simulation
        simulation->simulate(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        // Verify callbacks were called with correct context
        REQUIRE(preStepCalled);
        REQUIRE(postStepCalled);
        REQUIRE(preStepScenePath == postStepScenePath);
        REQUIRE(preStepSimId == postStepSimId);

        // Unsubscribe and verify callbacks are not called
        simulation->unsubscribePhysicsOnStepEvents(postStepId);
        simulation->unsubscribePhysicsOnStepEvents(preStepId);        

        preStepCalled = false;
        postStepCalled = false;

        simulation->simulate(1.0f/60.0f, 1.0f/60.0f);
        simulation->fetchResults();

        REQUIRE_FALSE(preStepCalled);
        REQUIRE_FALSE(postStepCalled);
    }

    SECTION("Contact events")
    {
        bool contactEventCalled = false;
        uint32_t numEventHeaders = 0;
        uint32_t numContactData = 0;
        uint32_t numFrictionAnchors = 0;

        OnContactReportEventFn contactCallback =
            [&](const ContactEventHeaderVector& eventHeaders, const ContactDataVector& contactData,
                const FrictionAnchorsDataVector& frictionAnchors) { contactEventCalled = true; };

        // Subscribe to contact events
        SubscriptionId contactId = simulation->subscribePhysicsContactReportEvents(contactCallback);
        REQUIRE(contactId != kInvalidSubscriptionId);

        // Run simulation
        simulation->simulate(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        // Verify callback was called
        REQUIRE(contactEventCalled);

        // Unsubscribe and verify callback is not called
        simulation->unsubscribePhysicsContactReportEvents(contactId);

        contactEventCalled = false;

        simulation->simulate(1.0f/60.0f, 1.0f/60.0f);
        simulation->fetchResults();

        REQUIRE_FALSE(contactEventCalled);
    }

    SECTION("Change tracking")
    {
        // Test initial state
        REQUIRE_FALSE(simulation->isChangeTrackingPaused());

        // Test pausing
        simulation->pauseChangeTracking(true);
        REQUIRE(simulation->isChangeTrackingPaused());

        // Test resuming
        simulation->pauseChangeTracking(false);
        REQUIRE_FALSE(simulation->isChangeTrackingPaused());

        // Test multiple toggles
        for (int i = 0; i < 5; i++) {
            bool expected = (i % 2) == 0;
            simulation->pauseChangeTracking(expected);
            REQUIRE(simulation->isChangeTrackingPaused() == expected);
        }
    }

    SECTION("Body control")
    {
        const uint64_t stageId = 123;
        const uint64_t bodyPath = 456;

        // Test initial state
        REQUIRE_FALSE(simulation->isSleeping(stageId, bodyPath));

        // Test putting to sleep
        simulation->putToSleep(stageId, bodyPath);
        REQUIRE(simulation->isSleeping(stageId, bodyPath));

        // Test waking up
        simulation->wakeUp(stageId, bodyPath);
        REQUIRE_FALSE(simulation->isSleeping(stageId, bodyPath));

        // Test force application
        const carb::Float3 force = { 1.0f, 2.0f, 3.0f };
        const carb::Float3 pos = { 4.0f, 5.0f, 6.0f };
        simulation->addForceAtPos(stageId, bodyPath, force, pos, ForceModeType::eFORCE);

        // Test torque application
        const carb::Float3 torque = { 7.0f, 8.0f, 9.0f };
        simulation->addTorque(stageId, bodyPath, torque);
    }

    physics->unregisterSimulation(simId);
}
