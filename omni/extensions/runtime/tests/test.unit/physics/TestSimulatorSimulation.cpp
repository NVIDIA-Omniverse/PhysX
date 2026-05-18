// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
        , m_capabilityCheckEnabled(true)
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

        simulate = [this](float elapsedTime, float currentTime) {
            simulateAsync(elapsedTime, currentTime);
            fetchResults();
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

        subscribePhysicsOnStepEvents = [this](bool preStep, int order, OnPhysicsStepEventFn onUpdate) -> SubscriptionId {
            m_stepEventCallbacks.emplace_back(onUpdate);
            return m_stepEventCallbacks.size();
        };

        unsubscribePhysicsOnStepEvents = [this](SubscriptionId subscriptionId) {
            if (subscriptionId > 0 && subscriptionId <= m_stepEventCallbacks.size()) {
                m_stepEventCallbacks.erase(m_stepEventCallbacks.begin() + subscriptionId - 1);
            }
        };

        isCapableOfSimulating = [this](const char** schemaNames, size_t schemaNamesCount, bool* isCapable) -> bool {
            if (!m_capabilityCheckEnabled)
            {
                return false;
            }
            for (size_t i = 0; i < schemaNamesCount; ++i)
            {
                const std::string schemaName(schemaNames[i]);
                // Check if the schema name is in the supported capabilities map
                auto it = m_supportedCapabilities.find(schemaName);
                if (it != m_supportedCapabilities.end())
                {
                    isCapable[i] = it->second;
                }
                else
                {
                    // Default to false for unknown schema names
                    isCapable[i] = false;
                }
            }
            return true;
        };
    }

    ~MockSimulator()
    {
        // Cleanup
        m_stepEventCallbacks.clear();
        m_contactEventCallbacks.clear();
    }

    // Function pointers that match SimulationFns structure
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
    IsCapableOfSimulatingFn isCapableOfSimulating;

    // Helper methods to configure capability check behavior for testing
    void setCapabilityCheckEnabled(bool enabled)
    {
        m_capabilityCheckEnabled = enabled;
    }

    void setSupportedCapability(const std::string& schemaName, bool isSupported)
    {
        m_supportedCapabilities[schemaName] = isSupported;
    }

    void clearSupportedCapabilities()
    {
        m_supportedCapabilities.clear();
    }

    // Get the function pointers structure
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
        fns.isCapableOfSimulating = isCapableOfSimulating;
        return fns;
    }

private:
    // Internal state
    long m_attachedStageId;
    bool m_isChangeTrackingPaused;
    uint32_t m_timeStepsPerSecond;
    uint64_t m_simulationTimestamp;
    uint64_t m_simulationStepCount;
    std::vector<OnPhysicsStepEventFn> m_stepEventCallbacks;
    std::vector<OnContactReportEventFn> m_contactEventCallbacks;
    // Capability check state
    bool m_capabilityCheckEnabled;
    std::unordered_map<std::string, bool> m_supportedCapabilities;
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
    SimulationId simId;
    const char* simulationName = "MockupSimulator";
    SECTION("Simulation registry events")
    {
        struct eventDataStruct {
            SimulationRegistryEventType::Enum eventType;
            SimulationId id;
            std::string name;
        };
        eventDataStruct eventData;
        SubscriptionId subscriptionId = physics->subscribeSimulationRegistryEvents([&](const SimulationRegistryEventType::Enum eventType, const SimulationId& id, const char* name, void* userData) {
            eventDataStruct* eventDataPtr = (eventDataStruct*)userData;
            eventDataPtr->eventType = eventType;
            eventDataPtr->id = id;
            eventDataPtr->name.assign(name);
        }, &eventData);

        REQUIRE(subscriptionId != kInvalidSubscriptionId);
        simId = physics->registerSimulation(sim, simulationName);
        REQUIRE(simId != kInvalidSimulationId);
        REQUIRE(physics->isSimulationActive(simId));
        REQUIRE(eventData.eventType == SimulationRegistryEventType::eSIMULATION_REGISTERED);
        REQUIRE(eventData.id == simId);
        REQUIRE(strcmp(eventData.name.c_str(), simulationName) == 0);

        physics->deactivateSimulation(simId);
        REQUIRE_FALSE(physics->isSimulationActive(simId));
        REQUIRE(eventData.eventType == SimulationRegistryEventType::eSIMULATION_DEACTIVATED);
        REQUIRE(eventData.id == simId);
        REQUIRE(strcmp(eventData.name.c_str(), simulationName) == 0);

        physics->activateSimulation(simId);
        REQUIRE(physics->isSimulationActive(simId));
        REQUIRE(eventData.eventType == SimulationRegistryEventType::eSIMULATION_ACTIVATED);
        REQUIRE(eventData.id == simId);
        REQUIRE(strcmp(eventData.name.c_str(), simulationName) == 0);

        physics->unregisterSimulation(simId);
        REQUIRE(eventData.eventType == SimulationRegistryEventType::eSIMULATION_UNREGISTERED);
        REQUIRE(eventData.id == simId);
        REQUIRE(strcmp(eventData.name.c_str(), simulationName) == 0);
        physics->unsubscribeSimulationRegistryEvents(subscriptionId);
        CARB_LOG_WARN("TestSimulatorSimulation: ending simulation registry events test");
    }

    simId = physics->registerSimulation(sim, simulationName);
    REQUIRE(simId != kInvalidSimulationId);   

    SECTION("Stage attachment")
    {
        REQUIRE(simulation->initialize(123));
        REQUIRE(simulation->getAttachedStage() == 123);
        simulation->close();
        REQUIRE(simulation->getAttachedStage() == 0);
    }

    SECTION("Synchronous simulate")
    {
        REQUIRE(simulation->getSimulationTimestamp(simId) == 0);
        REQUIRE(simulation->getSimulationStepCount(simId) == 0);

        // Synchronous simulate should update timestamp and step count in a single call
        simulation->simulate(1.0f/60.0f, 0.0f);

        REQUIRE(simulation->getSimulationTimestamp(simId) == 1);
        REQUIRE(simulation->getSimulationStepCount(simId) == 1);

        // Test multiple synchronous steps
        for (int i = 0; i < 5; i++) {
            simulation->simulate(1.0f/60.0f, (i + 1) * 1.0f/60.0f);
        }

        REQUIRE(simulation->getSimulationTimestamp(simId) == 6);
        REQUIRE(simulation->getSimulationStepCount(simId) == 6);
    }

    SECTION("Simulation timing")
    {
        const long stageId = 123;
        const uint64_t scenePath = 0;
        REQUIRE(simulation->getSimulationTimeStepsPerSecond(simId, stageId, scenePath) == 60);
        REQUIRE(simulation->getSimulationTimestamp(simId) == 0);
        REQUIRE(simulation->getSimulationStepCount(simId) == 0);

        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        REQUIRE(simulation->getSimulationTimestamp(simId) == 1);
        REQUIRE(simulation->getSimulationStepCount(simId) == 1);

        // Test multiple simulation steps
        for (int i = 0; i < 5; i++) {
            simulation->simulateAsync(1.0f/60.0f, (i + 1) * 1.0f/60.0f);
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
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
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

        simulation->simulateAsync(1.0f/60.0f, 1.0f/60.0f);
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
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        // Verify callback was called
        REQUIRE(contactEventCalled);

        // Unsubscribe and verify callback is not called
        simulation->unsubscribePhysicsContactReportEvents(contactId);

        contactEventCalled = false;

        simulation->simulateAsync(1.0f/60.0f, 1.0f/60.0f);
        simulation->fetchResults();

        REQUIRE_FALSE(contactEventCalled);
    }

    SECTION("Change tracking")
    {
        // Test initial state
        REQUIRE_FALSE(simulation->isChangeTrackingPaused(simId));

        // Test pausing
        simulation->pauseChangeTracking(true);
        REQUIRE(simulation->isChangeTrackingPaused(simId));

        // Test resuming
        simulation->pauseChangeTracking(false);
        REQUIRE_FALSE(simulation->isChangeTrackingPaused(simId));

        // Test multiple toggles
        for (int i = 0; i < 5; i++) {
            bool expected = (i % 2) == 0;
            simulation->pauseChangeTracking(expected);
            REQUIRE(simulation->isChangeTrackingPaused(simId) == expected);
        }
    }

    SECTION("Contact events subscription when simulator is inactive")
    {
        // Deactivate the simulation first
        physics->deactivateSimulation(simId);
        REQUIRE_FALSE(physics->isSimulationActive(simId));

        bool contactEventCalled = false;

        OnContactReportEventFn contactCallback =
            [&](const ContactEventHeaderVector& eventHeaders, const ContactDataVector& contactData,
                const FrictionAnchorsDataVector& frictionAnchors) { contactEventCalled = true; };

        // Subscribe to contact events while simulation is inactive
        SubscriptionId contactId = simulation->subscribePhysicsContactReportEvents(contactCallback);
        REQUIRE(contactId != kInvalidSubscriptionId);

        // Activate the simulation
        physics->activateSimulation(simId);
        REQUIRE(physics->isSimulationActive(simId));

        // Run simulation and verify callback is called
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        REQUIRE(contactEventCalled);

        // Cleanup
        simulation->unsubscribePhysicsContactReportEvents(contactId);
    }

    SECTION("Step events subscription when simulator is inactive")
    {
        // Deactivate the simulation first
        physics->deactivateSimulation(simId);
        REQUIRE_FALSE(physics->isSimulationActive(simId));

        bool preStepCalled = false;
        bool postStepCalled = false;

        OnPhysicsStepEventFn preStepCallback = [&](float elapsedTime, const PhysicsStepContext& context) {
            preStepCalled = true;
            return true;
        };

        OnPhysicsStepEventFn postStepCallback = [&](float elapsedTime, const PhysicsStepContext& context) {
            postStepCalled = true;
            return true;
        };

        // Subscribe to step events while simulation is inactive
        SubscriptionId preStepId = simulation->subscribePhysicsOnStepEvents(true, 0, preStepCallback);
        SubscriptionId postStepId = simulation->subscribePhysicsOnStepEvents(false, 1, postStepCallback);

        REQUIRE(preStepId != kInvalidSubscriptionId);
        REQUIRE(postStepId != kInvalidSubscriptionId);

        // Activate the simulation
        physics->activateSimulation(simId);
        REQUIRE(physics->isSimulationActive(simId));

        // Run simulation and verify callbacks are called
        simulation->simulateAsync(1.0f/60.0f, 0.0f);
        simulation->fetchResults();

        REQUIRE(preStepCalled);
        REQUIRE(postStepCalled);

        // Cleanup
        simulation->unsubscribePhysicsOnStepEvents(preStepId);
        simulation->unsubscribePhysicsOnStepEvents(postStepId);
    }

    physics->unregisterSimulation(simId);
}


//-----------------------------------------------------------------------------
// Capability Check Tests
TEST_CASE("Simulator Capability Check Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsSimulation* simulation = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsSimulation>();
    REQUIRE(simulation);

    // Create a mock simulator with capability check support
    MockSimulator mockSimulator;
    
    // Configure supported capabilities
    mockSimulator.setSupportedCapability("PhysicsRigidBodyAPI", true);
    mockSimulator.setSupportedCapability("PhysicsCollisionAPI", true);
    mockSimulator.setSupportedCapability("PhysicsMassAPI", true);
    mockSimulator.setSupportedCapability("PhysicsArticulationRootAPI", true);
    mockSimulator.setSupportedCapability("PhysicsJoint", true);
    mockSimulator.setSupportedCapability("UnsupportedSchemaAPI", false);
    
    SimulationFns fns = mockSimulator.getSimulationFns();
    Simulation sim;
    sim.simulationFns = fns;
    
    const char* simulationName = "MockupCapabilitySimulator";
    SimulationId simId = physics->registerSimulation(sim, simulationName);
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Check single supported capability")
    {
        const char* schemaNames[] = { "PhysicsRigidBodyAPI" };
        bool isCapable[1] = { false };
        
        bool success = simulation->isCapableOfSimulating(simId, schemaNames, 1, isCapable);
        
        REQUIRE(success);
        REQUIRE(isCapable[0] == true);
    }

    SECTION("Check single unsupported capability")
    {
        const char* schemaNames[] = { "UnsupportedSchemaAPI" };
        bool isCapable[1] = { true };  // Initialize to true to verify it gets set to false
        
        bool success = simulation->isCapableOfSimulating(simId, schemaNames, 1, isCapable);
        
        REQUIRE(success);
        REQUIRE(isCapable[0] == false);
    }

    SECTION("Check unknown capability returns false")
    {
        const char* schemaNames[] = { "UnknownSchemaAPI" };
        bool isCapable[1] = { true };  // Initialize to true to verify it gets set to false
        
        bool success = simulation->isCapableOfSimulating(simId, schemaNames, 1, isCapable);
        
        REQUIRE(success);
        REQUIRE(isCapable[0] == false);
    }

    SECTION("Check multiple capabilities - all supported")
    {
        const char* schemaNames[] = { "PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI" };
        bool isCapable[3] = { false, false, false };
        
        bool success = simulation->isCapableOfSimulating(simId, schemaNames, 3, isCapable);
        
        REQUIRE(success);
        REQUIRE(isCapable[0] == true);
        REQUIRE(isCapable[1] == true);
        REQUIRE(isCapable[2] == true);
    }

    SECTION("Check multiple capabilities - mixed support")
    {
        const char* schemaNames[] = { "PhysicsRigidBodyAPI", "UnsupportedSchemaAPI", "PhysicsJoint" };
        bool isCapable[3] = { false, true, false };
        
        bool success = simulation->isCapableOfSimulating(simId, schemaNames, 3, isCapable);
        
        REQUIRE(success);
        REQUIRE(isCapable[0] == true);   // Supported
        REQUIRE(isCapable[1] == false);  // Not supported
        REQUIRE(isCapable[2] == true);   // Supported
    }

    SECTION("Check empty schema list")
    {
        const char** schemaNames = nullptr;
        bool* isCapable = nullptr;
        
        bool success = simulation->isCapableOfSimulating(simId, schemaNames, 0, isCapable);
        
        REQUIRE(success);
    }

    SECTION("Check capability with invalid simulation ID")
    {
        const char* schemaNames[] = { "PhysicsRigidBodyAPI" };
        bool isCapable[1] = { true };
        
        // Use invalid simulation ID
        bool success = simulation->isCapableOfSimulating(kInvalidSimulationId, schemaNames, 1, isCapable);
        
        // Should return false since simulation ID is invalid
        REQUIRE_FALSE(success);
    }

    SECTION("Capability check disabled returns failure")
    {
        // Create a new simulator with capability check disabled
        MockSimulator disabledSimulator;
        disabledSimulator.setCapabilityCheckEnabled(false);
        
        SimulationFns disabledFns = disabledSimulator.getSimulationFns();
        Simulation disabledSim;
        disabledSim.simulationFns = disabledFns;
        
        const char* disabledSimName = "DisabledCapabilitySimulator";
        SimulationId disabledSimId = physics->registerSimulation(disabledSim, disabledSimName);
        REQUIRE(disabledSimId != kInvalidSimulationId);
        
        const char* schemaNames[] = { "PhysicsRigidBodyAPI" };
        bool isCapable[1] = { true };
        
        bool success = simulation->isCapableOfSimulating(disabledSimId, schemaNames, 1, isCapable);
        
        // Should return false when capability check is disabled
        REQUIRE_FALSE(success);
        
        physics->unregisterSimulation(disabledSimId);
    }

    SECTION("Simulation without capability check function")
    {
        // Create a simulator without the isCapableOfSimulating function
        MockSimulator noCapabilitySimulator;
        SimulationFns noCapFns = noCapabilitySimulator.getSimulationFns();
        noCapFns.isCapableOfSimulating = nullptr;  // Explicitly set to nullptr
        
        Simulation noCapSim;
        noCapSim.simulationFns = noCapFns;
        
        const char* noCapSimName = "NoCapabilitySimulator";
        SimulationId noCapSimId = physics->registerSimulation(noCapSim, noCapSimName);
        REQUIRE(noCapSimId != kInvalidSimulationId);
        
        const char* schemaNames[] = { "PhysicsRigidBodyAPI" };
        bool isCapable[1] = { true };
        
        bool success = simulation->isCapableOfSimulating(noCapSimId, schemaNames, 1, isCapable);
        
        // Should return false when capability check function is not provided
        REQUIRE_FALSE(success);
        
        physics->unregisterSimulation(noCapSimId);
    }

    SECTION("Dynamic capability registration")
    {
        // Create a new simulator
        MockSimulator dynamicSimulator;
        dynamicSimulator.setSupportedCapability("PhysicsRigidBodyAPI", true);
        
        SimulationFns dynamicFns = dynamicSimulator.getSimulationFns();
        Simulation dynamicSim;
        dynamicSim.simulationFns = dynamicFns;
        
        const char* dynamicSimName = "DynamicCapabilitySimulator";
        SimulationId dynamicSimId = physics->registerSimulation(dynamicSim, dynamicSimName);
        REQUIRE(dynamicSimId != kInvalidSimulationId);
        
        const char* schemaNames[] = { "NewDynamicAPI" };
        bool isCapable[1] = { true };
        
        // Initially the capability is unknown (defaults to false)
        bool success = simulation->isCapableOfSimulating(dynamicSimId, schemaNames, 1, isCapable);
        REQUIRE(success);
        REQUIRE(isCapable[0] == false);
        
        // Add the capability dynamically
        dynamicSimulator.setSupportedCapability("NewDynamicAPI", true);
        
        // Now check again
        isCapable[0] = false;
        success = simulation->isCapableOfSimulating(dynamicSimId, schemaNames, 1, isCapable);
        REQUIRE(success);
        REQUIRE(isCapable[0] == true);
        
        physics->unregisterSimulation(dynamicSimId);
    }

    SECTION("Check large number of capabilities")
    {
        // Test with many capabilities to ensure scalability
        std::vector<std::string> schemaNameStrings;
        std::vector<const char*> schemaNamePtrs;
        const size_t numCapabilities = 100;
        
        for (size_t i = 0; i < numCapabilities; ++i)
        {
            std::string schemaName = "TestSchema" + std::to_string(i);
            schemaNameStrings.push_back(schemaName);
            // Set every other capability as supported
            mockSimulator.setSupportedCapability(schemaName, (i % 2) == 0);
        }
        
        // Update pointers after all strings are added
        for (const auto& name : schemaNameStrings)
        {
            schemaNamePtrs.push_back(name.c_str());
        }
        
        // Use unique_ptr<bool[]> instead of vector<bool> since vector<bool> is a special
        // template specialization that doesn't provide data() returning bool*
        std::unique_ptr<bool[]> isCapable(new bool[numCapabilities]);
        for (size_t i = 0; i < numCapabilities; ++i)
        {
            isCapable[i] = false;
        }
        
        bool success = simulation->isCapableOfSimulating(
            simId, schemaNamePtrs.data(), numCapabilities, isCapable.get());
        
        REQUIRE(success);
        
        // Verify alternating pattern
        for (size_t i = 0; i < numCapabilities; ++i)
        {
            REQUIRE(isCapable[i] == ((i % 2) == 0));
        }
    }

    physics->unregisterSimulation(simId);
}
