// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <carb/events/IEvents.h>
#include <carb/events/EventsUtils.h>

using namespace omni::physics;

// Mock stage update implementation
class MockStageUpdate
{
public:
    enum class TimelineState
    {
        Stopped,
        Paused,
        Playing
    };

    MockStageUpdate()
        : m_attachedStageId(0)
        , m_isAttached(false)
        , m_isPhysicsLoaded(false)
        , m_timelineState(TimelineState::Stopped)
        , m_resetSimulationCalled(false)
        , m_startSimulationCalled(false)
    {
        // Initialize function pointers
        onAttach = [this](long int stageId) {
            m_attachedStageId = stageId;
            m_isAttached = true;
        };

        onDetach = [this]() {
            m_attachedStageId = 0;
            m_isAttached = false;
            m_isPhysicsLoaded = false;
            m_timelineState = TimelineState::Stopped;
        };

        onUpdate = [this](float currentTime, float elapsedSecs, bool enableUpdate) {
            if (m_isAttached && enableUpdate) {
                // Mock update implementation
            }
        };

        onResume = [this](float currentTime) {
            if (m_isAttached) {
                m_timelineState = TimelineState::Playing;
            }
        };

        onPause = [this]() {
            if (m_isAttached) {
                m_timelineState = TimelineState::Paused;
            }
        };

        onReset = [this]() {
            if (m_isAttached) {
                m_timelineState = TimelineState::Stopped;
            }
        };

        handleRaycast = [this](const float* orig, const float* dir, bool input) {
            if (m_isAttached) {
                // Mock raycast implementation
            }
        };

        forceLoadPhysicsFromUSD = [this]() {
            if (m_isAttached) {
                m_isPhysicsLoaded = true;
            }
        };

        releasePhysicsObjects = [this]() {
            if (m_isAttached) {
                m_isPhysicsLoaded = false;
            }
        };

        resetSimulation = [this]() {
            if (m_isAttached) {
                m_isPhysicsLoaded = false;
                m_timelineState = TimelineState::Stopped;
                m_resetSimulationCalled = true;
            }
        };

        startSimulation = [this]() {
            if (m_isAttached) {
                m_startSimulationCalled = true;
            }
        };
    }

    ~MockStageUpdate()
    {
        // Cleanup
    }

    // Function pointers that match StageUpdateFns structure
    OnAttachFn onAttach;
    OnDetachFn onDetach;
    OnUpdateFn onUpdate;
    OnResumeFn onResume;
    OnPauseFn onPause;
    OnResetFn onReset;
    HandleRaycastFn handleRaycast;
    ForceLoadPhysicsFromUSDFn forceLoadPhysicsFromUSD;
    ReleasePhysicsObjectsFn releasePhysicsObjects;
    ResetSimulationFn resetSimulation;
    StartSimulationFn startSimulation;

    // Get the function pointers structure
    StageUpdateFns getStageUpdateFns() const
    {
        StageUpdateFns fns;
        fns.onAttach = onAttach;
        fns.onDetach = onDetach;
        fns.onUpdate = onUpdate;
        fns.onResume = onResume;
        fns.onPause = onPause;
        fns.onReset = onReset;
        fns.handleRaycast = handleRaycast;
        fns.forceLoadPhysicsFromUSD = forceLoadPhysicsFromUSD;
        fns.releasePhysicsObjects = releasePhysicsObjects;
        fns.resetSimulation = resetSimulation;
        fns.startSimulation = startSimulation;
        return fns;
    }

    TimelineState getTimelineState() const { return m_timelineState; }
    bool wasResetSimulationCalled() const { return m_resetSimulationCalled; }
    void clearResetSimulationFlag() { m_resetSimulationCalled = false; }
    bool wasStartSimulationCalled() const { return m_startSimulationCalled; }
    void clearStartSimulationFlag() { m_startSimulationCalled = false; }

public:
    // Internal state
    long m_attachedStageId;
    bool m_isAttached;
    bool m_isPhysicsLoaded;
    TimelineState m_timelineState;
    bool m_resetSimulationCalled;
    bool m_startSimulationCalled;
};

//-----------------------------------------------------------------------------
// Stage Update Tests
TEST_CASE("Simulator Stage Update Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsStageUpdate* stageUpdate = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsStageUpdate>();
    REQUIRE(stageUpdate);

    // Create a mock stage update
    MockStageUpdate mockStageUpdate;
    StageUpdateFns fns = mockStageUpdate.getStageUpdateFns();

    Simulation sim;
    sim.stageUpdateFns = fns;

    SimulationId simId = physics->registerSimulation(sim, "MockupSimulator");
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Stage attachment")
    {
        stageUpdate->onAttach(123);
        REQUIRE(mockStageUpdate.m_attachedStageId == 123);
        REQUIRE(mockStageUpdate.m_isAttached);
        
        stageUpdate->onDetach();
        REQUIRE(mockStageUpdate.m_attachedStageId == 0);
        REQUIRE_FALSE(mockStageUpdate.m_isAttached);
    }

    SECTION("Stage update")
    {
        stageUpdate->onAttach(123);
        
        // Test update with physics enabled
        stageUpdate->onUpdate(1.0f, 0.016f, true);
        REQUIRE(mockStageUpdate.m_isAttached);
        
        // Test update with physics disabled
        stageUpdate->onUpdate(1.0f, 0.016f, false);
        REQUIRE(mockStageUpdate.m_isAttached);
    }

    SECTION("Timeline control")
    {
        stageUpdate->onAttach(123);
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Stopped);
        
        // Test resume
        stageUpdate->onResume(1.0f);
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Playing);
        
        // Test pause
        stageUpdate->onPause();
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Paused);
        
        // Test reset
        stageUpdate->onReset();
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Stopped);

        // Test detach resets timeline state
        stageUpdate->onDetach();
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Stopped);
    }

    SECTION("Physics loading")
    {
        stageUpdate->onAttach(123);
        
        // Test force load
        stageUpdate->forceLoadPhysicsFromUSD();
        REQUIRE(mockStageUpdate.m_isPhysicsLoaded);
        
        // Test release
        stageUpdate->releasePhysicsObjects();
        REQUIRE_FALSE(mockStageUpdate.m_isPhysicsLoaded);
    }

    SECTION("Raycast handling")
    {
        stageUpdate->onAttach(123);
        
        const float orig[3] = { 0.0f, 0.0f, 0.0f };
        const float dir[3] = { 0.0f, 0.0f, 1.0f };
        
        // Test raycast with input
        stageUpdate->handleRaycast(orig, dir, true);
        REQUIRE(mockStageUpdate.m_isAttached);
        
        // Test raycast without input
        stageUpdate->handleRaycast(orig, dir, false);
        REQUIRE(mockStageUpdate.m_isAttached);
    }

    SECTION("Reset simulation")
    {
        stageUpdate->onAttach(123);
        REQUIRE(mockStageUpdate.m_isAttached);
        REQUIRE_FALSE(mockStageUpdate.wasResetSimulationCalled());
        
        // Load physics first
        stageUpdate->forceLoadPhysicsFromUSD();
        REQUIRE(mockStageUpdate.m_isPhysicsLoaded);
        
        // Set timeline to playing
        stageUpdate->onResume(1.0f);
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Playing);
        
        // Test resetSimulation
        stageUpdate->resetSimulation();
        
        // Verify simulation was reset
        REQUIRE(mockStageUpdate.wasResetSimulationCalled());
        REQUIRE_FALSE(mockStageUpdate.m_isPhysicsLoaded);  // Physics should be released
        REQUIRE(mockStageUpdate.getTimelineState() == MockStageUpdate::TimelineState::Stopped);  // Timeline should be stopped
        REQUIRE(mockStageUpdate.m_isAttached);  // Stage should remain attached
        
        // Test that we can clear the flag and call again
        mockStageUpdate.clearResetSimulationFlag();
        REQUIRE_FALSE(mockStageUpdate.wasResetSimulationCalled());
        
        stageUpdate->resetSimulation();
        REQUIRE(mockStageUpdate.wasResetSimulationCalled());
    }

    SECTION("Start simulation")
    {
        stageUpdate->onAttach(123);
        REQUIRE(mockStageUpdate.m_isAttached);
        REQUIRE_FALSE(mockStageUpdate.wasStartSimulationCalled());
        
        // Test startSimulation
        stageUpdate->startSimulation();
        
        // Verify simulation was started
        REQUIRE(mockStageUpdate.wasStartSimulationCalled());
        REQUIRE(mockStageUpdate.m_isAttached);  // Stage should remain attached
        
        // Test that we can clear the flag and call again
        mockStageUpdate.clearStartSimulationFlag();
        REQUIRE_FALSE(mockStageUpdate.wasStartSimulationCalled());
        
        stageUpdate->startSimulation();
        REQUIRE(mockStageUpdate.wasStartSimulationCalled());
        
        stageUpdate->onDetach();
    }

    physics->unregisterSimulation(simId);
}

//-----------------------------------------------------------------------------
// Simulation Event Stream Tests
TEST_CASE("Simulation Event Stream Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsStageUpdate* stageUpdate = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsStageUpdate>();
    REQUIRE(stageUpdate);

    // Create a mock stage update
    MockStageUpdate mockStageUpdate;
    StageUpdateFns fns = mockStageUpdate.getStageUpdateFns();

    Simulation sim;
    sim.stageUpdateFns = fns;

    SimulationId simId = physics->registerSimulation(sim, "MockupSimulatorEvents");
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Event stream exists")
    {
        carb::events::IEventStreamPtr eventStream = stageUpdate->getSimulationEventStream();
        REQUIRE(eventStream != nullptr);
    }

    SECTION("Resume event is sent")
    {
        carb::events::IEventStreamPtr eventStream = stageUpdate->getSimulationEventStream();
        REQUIRE(eventStream != nullptr);

        // Track received events
        std::vector<SimulationEvent> receivedEvents;
        auto subscription = carb::events::createSubscriptionToPop(
            eventStream.get(),
            [&receivedEvents](carb::events::IEvent* event) {
                receivedEvents.push_back(static_cast<SimulationEvent>(event->type));
            }
        );

        stageUpdate->onAttach(123);
        stageUpdate->onResume(1.0f);

        // Process events
        eventStream->pump();

        REQUIRE(receivedEvents.size() == 1);
        REQUIRE(receivedEvents[0] == SimulationEvent::eResumed);

        stageUpdate->onDetach();
    }

    SECTION("Pause event is sent")
    {
        carb::events::IEventStreamPtr eventStream = stageUpdate->getSimulationEventStream();
        REQUIRE(eventStream != nullptr);

        // Track received events
        std::vector<SimulationEvent> receivedEvents;
        auto subscription = carb::events::createSubscriptionToPop(
            eventStream.get(),
            [&receivedEvents](carb::events::IEvent* event) {
                receivedEvents.push_back(static_cast<SimulationEvent>(event->type));
            }
        );

        stageUpdate->onAttach(123);
        stageUpdate->onResume(1.0f);
        eventStream->pump();
        receivedEvents.clear();

        stageUpdate->onPause();

        // Process events
        eventStream->pump();

        REQUIRE(receivedEvents.size() == 1);
        REQUIRE(receivedEvents[0] == SimulationEvent::ePaused);

        stageUpdate->onDetach();
    }

    SECTION("Stopped event is sent on reset")
    {
        carb::events::IEventStreamPtr eventStream = stageUpdate->getSimulationEventStream();
        REQUIRE(eventStream != nullptr);

        // Track received events
        std::vector<SimulationEvent> receivedEvents;
        auto subscription = carb::events::createSubscriptionToPop(
            eventStream.get(),
            [&receivedEvents](carb::events::IEvent* event) {
                receivedEvents.push_back(static_cast<SimulationEvent>(event->type));
            }
        );

        stageUpdate->onAttach(123);
        stageUpdate->onResume(1.0f);
        eventStream->pump();
        receivedEvents.clear();

        stageUpdate->onReset();

        // Process events
        eventStream->pump();

        REQUIRE(receivedEvents.size() == 1);
        REQUIRE(receivedEvents[0] == SimulationEvent::eStopped);

        stageUpdate->onDetach();
    }

    SECTION("Full lifecycle events")
    {
        carb::events::IEventStreamPtr eventStream = stageUpdate->getSimulationEventStream();
        REQUIRE(eventStream != nullptr);

        // Track received events
        std::vector<SimulationEvent> receivedEvents;
        auto subscription = carb::events::createSubscriptionToPop(
            eventStream.get(),
            [&receivedEvents](carb::events::IEvent* event) {
                receivedEvents.push_back(static_cast<SimulationEvent>(event->type));
            }
        );

        stageUpdate->onAttach(123);

        // Resume -> Pause -> Resume -> Stop
        stageUpdate->onResume(1.0f);
        eventStream->pump();
        REQUIRE(receivedEvents.back() == SimulationEvent::eResumed);

        stageUpdate->onPause();
        eventStream->pump();
        REQUIRE(receivedEvents.back() == SimulationEvent::ePaused);

        stageUpdate->onResume(2.0f);
        eventStream->pump();
        REQUIRE(receivedEvents.back() == SimulationEvent::eResumed);

        stageUpdate->onReset();
        eventStream->pump();
        REQUIRE(receivedEvents.back() == SimulationEvent::eStopped);

        REQUIRE(receivedEvents.size() == 4);

        stageUpdate->onDetach();
    }

    physics->unregisterSimulation(simId);
} 
