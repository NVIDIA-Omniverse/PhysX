// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsSceneQuery.h>

using namespace omni::physics;

// Mock scene query implementation
class MockSceneQuery
{
public:
    MockSceneQuery()
        : m_raycastHitCount(0)
        , m_sweepHitCount(0)
        , m_overlapHitCount(0)
    {
        // Initialize function pointers
        raycastClosest = [this](const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& hit, bool bothSides) -> bool {
            m_raycastHitCount++;
            hit.position = { origin.x + unitDir.x * distance, origin.y + unitDir.y * distance, origin.z + unitDir.z * distance };
            hit.normal = { 0.0f, 1.0f, 0.0f };
            hit.distance = distance;
            return true;
        };

        raycastAny = [this](const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides) -> bool {
            m_raycastHitCount++;
            return true;
        };

        raycastAll = [this](const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHitReportFn reportFn, bool bothSides) {
            m_raycastHitCount++;
            RaycastHit hit;
            hit.position = { origin.x + unitDir.x * distance, origin.y + unitDir.y * distance, origin.z + unitDir.z * distance };
            hit.normal = { 0.0f, 1.0f, 0.0f };
            hit.distance = distance;
            reportFn(hit);
        };

        sweepSphereClosest = [this](float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides) -> bool {
            m_sweepHitCount++;
            hit.position = { origin.x + unitDir.x * distance, origin.y + unitDir.y * distance, origin.z + unitDir.z * distance };
            hit.normal = { 0.0f, 1.0f, 0.0f };
            hit.distance = distance;
            return true;
        };

        sweepSphereAny = [this](float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides) -> bool {
            m_sweepHitCount++;
            return true;
        };

        sweepSphereAll = [this](float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides) {
            m_sweepHitCount++;
            SweepHit hit;
            hit.position = { origin.x + unitDir.x * distance, origin.y + unitDir.y * distance, origin.z + unitDir.z * distance };
            hit.normal = { 0.0f, 1.0f, 0.0f };
            hit.distance = distance;
            reportFn(hit);
        };

        sweepBoxClosest = [this](const carb::Float3& halfExtent, const carb::Float3& origin, const carb::Float4& rot, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides) -> bool {
            m_sweepHitCount++;
            hit.position = { origin.x + unitDir.x * distance, origin.y + unitDir.y * distance, origin.z + unitDir.z * distance };
            hit.normal = { 0.0f, 1.0f, 0.0f };
            hit.distance = distance;
            return true;
        };

        sweepBoxAny = [this](const carb::Float3& halfExtent, const carb::Float3& origin, const carb::Float4& rot, const carb::Float3& unitDir, float distance, bool bothSides) -> bool {
            m_sweepHitCount++;
            return true;
        };

        sweepBoxAll = [this](const carb::Float3& halfExtent, const carb::Float3& origin, const carb::Float4& rot, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides) {
            m_sweepHitCount++;
            SweepHit hit;
            hit.position = { origin.x + unitDir.x * distance, origin.y + unitDir.y * distance, origin.z + unitDir.z * distance };
            hit.normal = { 0.0f, 1.0f, 0.0f };
            hit.distance = distance;
            reportFn(hit);
        };

        overlapSphere = [this](float radius, const carb::Float3& pos, OverlapHitReportFn reportFn) -> uint32_t {
            m_overlapHitCount++;
            OverlapHit hit;
            reportFn(hit);
            return 1;
        };

        overlapSphereAny = [this](float radius, const carb::Float3& pos) -> bool {
            m_overlapHitCount++;
            return true;
        };

        overlapBox = [this](const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, OverlapHitReportFn reportFn) -> uint32_t {
            m_overlapHitCount++;
            OverlapHit hit;
            reportFn(hit);
            return 1;
        };

        overlapBoxAny = [this](const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot) -> bool {
            m_overlapHitCount++;
            return true;
        };
    }

    ~MockSceneQuery()
    {
        // Cleanup
    }

    // Function pointers that match SceneQueryFns structure
    RaycastClosestFn raycastClosest;
    RaycastAnyFn raycastAny;
    RaycastAllFn raycastAll;
    SweepSphereClosestFn sweepSphereClosest;
    SweepSphereAnyFn sweepSphereAny;
    SweepSphereAllFn sweepSphereAll;
    SweepBoxClosestFn sweepBoxClosest;
    SweepBoxAnyFn sweepBoxAny;
    SweepBoxAllFn sweepBoxAll;
    OverlapSphereFn overlapSphere;
    OverlapSphereAnyFn overlapSphereAny;
    OverlapBoxFn overlapBox;
    OverlapBoxAnyFn overlapBoxAny;

    // Get the function pointers structure
    SceneQueryFns getSceneQueryFns() const
    {
        SceneQueryFns fns;
        fns.raycastClosest = raycastClosest;
        fns.raycastAny = raycastAny;
        fns.raycastAll = raycastAll;
        fns.sweepSphereClosest = sweepSphereClosest;
        fns.sweepSphereAny = sweepSphereAny;
        fns.sweepSphereAll = sweepSphereAll;
        fns.sweepBoxClosest = sweepBoxClosest;
        fns.sweepBoxAny = sweepBoxAny;
        fns.sweepBoxAll = sweepBoxAll;
        fns.overlapSphere = overlapSphere;
        fns.overlapSphereAny = overlapSphereAny;
        fns.overlapBox = overlapBox;
        fns.overlapBoxAny = overlapBoxAny;
        return fns;
    }

    // Hit counters for testing
    uint32_t m_raycastHitCount;
    uint32_t m_sweepHitCount;
    uint32_t m_overlapHitCount;
};

//-----------------------------------------------------------------------------
// Scene Query Tests
TEST_CASE("Simulator Scene Query Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    IPhysicsSceneQuery* sceneQuery = physicsTests.getApp()->getFramework()->acquireInterface<IPhysicsSceneQuery>();
    REQUIRE(sceneQuery);

    // Create a mock scene query
    MockSceneQuery mockSceneQuery;
    SceneQueryFns fns = mockSceneQuery.getSceneQueryFns();

    Simulation sim;
    sim.sceneQueryFns = fns;

    SimulationId simId = physics->registerSimulation(sim, "MockupSim");
    REQUIRE(simId != kInvalidSimulationId);

    SECTION("Raycast queries")
    {
        const carb::Float3 origin = { 0.0f, 0.0f, 0.0f };
        const carb::Float3 unitDir = { 0.0f, 1.0f, 0.0f };
        const float distance = 10.0f;
        RaycastHit hit;

        // Test closest hit
        REQUIRE(sceneQuery->raycastClosest(origin, unitDir, distance, hit, false));
        REQUIRE(mockSceneQuery.m_raycastHitCount == 1);
        REQUIRE(hit.distance == distance);

        // Test any hit
        REQUIRE(sceneQuery->raycastAny(origin, unitDir, distance, false));
        REQUIRE(mockSceneQuery.m_raycastHitCount == 2);

        // Test all hits
        bool hitReported = false;
        sceneQuery->raycastAll(
            origin, unitDir, distance,
            [&](const RaycastHit& h) {
                hitReported = true;
                return true;
            },
            false);
        REQUIRE(mockSceneQuery.m_raycastHitCount == 3);
        REQUIRE(hitReported);
    }

    SECTION("Sweep queries")
    {
        const float radius = 1.0f;
        const carb::Float3 origin = { 0.0f, 0.0f, 0.0f };
        const carb::Float3 unitDir = { 0.0f, 1.0f, 0.0f };
        const float distance = 10.0f;
        SweepHit hit;

        // Test closest hit
        REQUIRE(sceneQuery->sweepSphereClosest(radius, origin, unitDir, distance, hit, false));
        REQUIRE(mockSceneQuery.m_sweepHitCount == 1);
        REQUIRE(hit.distance == distance);

        // Test any hit
        REQUIRE(sceneQuery->sweepSphereAny(radius, origin, unitDir, distance, false));
        REQUIRE(mockSceneQuery.m_sweepHitCount == 2);

        // Test all hits
        bool hitReported = false;
        sceneQuery->sweepSphereAll(
            radius, origin, unitDir, distance,
            [&](const SweepHit& h) {
                hitReported = true;
                return true;
            },
            false);
        REQUIRE(mockSceneQuery.m_sweepHitCount == 3);
        REQUIRE(hitReported);
    }

    SECTION("Overlap queries")
    {
        const float radius = 1.0f;
        const carb::Float3 pos = { 0.0f, 0.0f, 0.0f };
        const carb::Float3 halfExtent = { 1.0f, 1.0f, 1.0f };
        const carb::Float4 rot = { 0.0f, 0.0f, 0.0f, 1.0f };

        // Test sphere overlap
        bool hitReported = false;
        REQUIRE(sceneQuery->overlapSphere(
                    radius, pos,
                    [&](const OverlapHit& h) {
                        hitReported = true;
                        return true;
                    }) == 1);
        REQUIRE(mockSceneQuery.m_overlapHitCount == 1);
        REQUIRE(hitReported);

        // Test sphere any overlap
        REQUIRE(sceneQuery->overlapSphereAny(radius, pos));
        REQUIRE(mockSceneQuery.m_overlapHitCount == 2);

        // Test box overlap
        hitReported = false;
        REQUIRE(sceneQuery->overlapBox(
                    halfExtent, pos, rot,
                    [&](const OverlapHit& h) {
                        hitReported = true;
                        return true;
                    }) == 1);
        REQUIRE(mockSceneQuery.m_overlapHitCount == 3);
        REQUIRE(hitReported);

        // Test box any overlap
        REQUIRE(sceneQuery->overlapBoxAny(halfExtent, pos, rot));
        REQUIRE(mockSceneQuery.m_overlapHitCount == 4);
    }

    SECTION("Box sweep queries")
    {
        const carb::Float3 halfExtent = { 1.0f, 1.0f, 1.0f };
        const carb::Float3 origin = { 0.0f, 0.0f, 0.0f };
        const carb::Float4 rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        const carb::Float3 unitDir = { 0.0f, 1.0f, 0.0f };
        const float distance = 10.0f;
        SweepHit hit;

        // Test closest hit
        REQUIRE(sceneQuery->sweepBoxClosest(halfExtent, origin, rot, unitDir, distance, hit, false));
        REQUIRE(hit.distance == distance);

        // Test any hit
        REQUIRE(sceneQuery->sweepBoxAny(halfExtent, origin, rot, unitDir, distance, false));

        // Test all hits
        bool hitReported = false;
        sceneQuery->sweepBoxAll(
            halfExtent, origin, rot, unitDir, distance,
            [&](const SweepHit& h) {
                hitReported = true;
                return true;
            },
            false);
        REQUIRE(hitReported);
    }

    SECTION("Invalid shape queries")
    {
        const uint64_t invalidShapePath = 0;
        const carb::Float3 unitDir = { 0.0f, 1.0f, 0.0f };
        const float distance = 10.0f;
        SweepHit hit;

        // Test invalid shape overlap
        REQUIRE(sceneQuery->overlapShape(
            invalidShapePath,
            [](const OverlapHit&) { return true; }) == 0);

        // Test invalid shape any overlap
        REQUIRE_FALSE(sceneQuery->overlapShapeAny(invalidShapePath));

        // Test invalid shape sweep closest
        REQUIRE_FALSE(sceneQuery->sweepShapeClosest(invalidShapePath, unitDir, distance, hit, false));

        // Test invalid shape sweep any
        REQUIRE_FALSE(sceneQuery->sweepShapeAny(invalidShapePath, unitDir, distance, false));

        // Test invalid shape sweep all
        bool hitReported = false;
        sceneQuery->sweepShapeAll(
            invalidShapePath, unitDir, distance,
            [&](const SweepHit&) {
                hitReported = true;
                return true;
            },
            false);
        REQUIRE_FALSE(hitReported);
    }

    physics->unregisterSimulation(simId);
} 
