// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsInteraction.h>

using namespace omni::physics;

// Mock interaction implementation
class MockInteraction
{
public:
    MockInteraction()
        : m_isResetOnStopDisabled(false)
        , m_raycastCount(0)
        , m_lastRaycastInput(false)
        , m_lastRaycastOrigin{0.0f, 0.0f, 0.0f}
        , m_lastRaycastDirection{0.0f, 0.0f, 0.0f}
    {
        // Initialize function pointers
        disableResetOnStop = [this](bool disable) {
            m_isResetOnStopDisabled = disable;
        };

        isDisabledResetOnStop = [this]() -> bool {
            return m_isResetOnStopDisabled;
        };

        handleRaycast = [this](const float* orig, const float* dir, bool input) {
            m_raycastCount++;
            m_lastRaycastInput = input;
            if (orig) {
                m_lastRaycastOrigin = {orig[0], orig[1], orig[2]};
            }
            if (dir) {
                m_lastRaycastDirection = {dir[0], dir[1], dir[2]};
            }
        };
    }

    ~MockInteraction()
    {
        // Cleanup
    }

    // Function pointers that match InteractionFns structure
    DisableResetOnStopFn disableResetOnStop;
    IsDisabledResetOnStopFn isDisabledResetOnStop;
    HandleRaycastFn handleRaycast;

    // Get the function pointers structure
    InteractionFns getInteractionFns() const
    {
        InteractionFns fns;
        fns.disableResetOnStop = disableResetOnStop;
        fns.isDisabledResetOnStop = isDisabledResetOnStop;
        fns.handleRaycast = handleRaycast;
        return fns;
    }

    // State for testing
    bool m_isResetOnStopDisabled;
    uint32_t m_raycastCount;
    bool m_lastRaycastInput;
    carb::Float3 m_lastRaycastOrigin;
    carb::Float3 m_lastRaycastDirection;
};

//-----------------------------------------------------------------------------
// Interaction Tests
TEST_CASE("Simulator Interaction Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsInteraction* interaction = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsInteraction>();
    REQUIRE(interaction);

    // Create a mock interaction
    MockInteraction mockInteraction;
    InteractionFns fns = mockInteraction.getInteractionFns();

    Simulation sim;
    sim.interactionFns = fns;

    SimulationId simId = physics->registerSimulation(sim, "TestSimulation");
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Reset on stop control")
    {
        // Test initial state
        REQUIRE_FALSE(mockInteraction.m_isResetOnStopDisabled);
        REQUIRE_FALSE(interaction->isDisabledResetOnStop());

        // Test disable
        interaction->disableResetOnStop(true);
        REQUIRE(mockInteraction.m_isResetOnStopDisabled);
        REQUIRE(interaction->isDisabledResetOnStop());

        // Test enable
        interaction->disableResetOnStop(false);
        REQUIRE_FALSE(mockInteraction.m_isResetOnStopDisabled);
        REQUIRE_FALSE(interaction->isDisabledResetOnStop());

        // Test multiple toggles
        for (int i = 0; i < 5; i++) {
            bool expected = (i % 2) == 0;
            interaction->disableResetOnStop(expected);
            REQUIRE(mockInteraction.m_isResetOnStopDisabled == expected);
            REQUIRE(interaction->isDisabledResetOnStop() == expected);
        }
    }

    SECTION("Raycast handling")
    {
        const carb::Float3 origin = { 1.0f, 2.0f, 3.0f };
        const carb::Float3 direction = { 0.0f, 1.0f, 0.0f };

        // Test raycast with input
        interaction->handleRaycast(&origin.x, &direction.x, true);
        REQUIRE(mockInteraction.m_raycastCount == 1);
        REQUIRE(mockInteraction.m_lastRaycastInput);

        // Test raycast without input
        interaction->handleRaycast(&origin.x, &direction.x, false);
        REQUIRE(mockInteraction.m_raycastCount == 2);
        REQUIRE_FALSE(mockInteraction.m_lastRaycastInput);

        // Test multiple raycasts
        for (int i = 0; i < 5; i++) {
            carb::Float3 testOrigin = {float(i), float(i+1), float(i+2)};
            carb::Float3 testDir = {float(i), float(i+1), float(i+2)};
            
            interaction->handleRaycast(&testOrigin.x, &testDir.x, true);
            REQUIRE(mockInteraction.m_raycastCount == 3 + i);
            REQUIRE(mockInteraction.m_lastRaycastInput);
        }
    }

    SECTION("Null pointer handling")
    {
        // Test raycast with null pointers
        interaction->handleRaycast(nullptr, nullptr, true);
        REQUIRE(mockInteraction.m_raycastCount == 1);
        REQUIRE(mockInteraction.m_lastRaycastInput);
    }

    physics->unregisterSimulation(simId);
} 
