// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-25
 */

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/IEvents.h>
#include <omni/physx/IPhysx.h>

// IPhysxUnitTests is a CARB ABI struct shared by OvruntimePhysX's fillInterface() and
// OvruntimeUnitTests' call sites: every member is unconditional and TestPathArg is pinned to
// plain std::string so the layout is identical in every TU. SdfPath
// call sites convert with GetString().
#include <string>
#include <unordered_map>

namespace omni
{

namespace physx
{

using TestPathArg = std::string;
// Regression tripwire: TestPathArg must stay a plain std::string.
static_assert(sizeof(TestPathArg) == sizeof(std::string), "TestPathArg must stay ABI-identical to std::string");

struct PhysicsStats
{
    PhysicsStats()
        : numDynamicRigids(0),
          numStaticRigids(0),
          numKinematicBodies(0),
          numArticulations(0),
          numSphereShapes(0),
          numBoxShapes(0),
          numCapsuleShapes(0),
          numCylinderShapes(0),
          numConvexShapes(0),
          numConeShapes(0),
          numTriMeshShapes(0),
          numPlaneShapes(0),
          numConstraints(0)
    {
    }

    uint32_t numDynamicRigids;
    uint32_t numStaticRigids;
    uint32_t numKinematicBodies;
    uint32_t numArticulations;
    uint32_t numSphereShapes;
    uint32_t numBoxShapes;
    uint32_t numCapsuleShapes;
    uint32_t numCylinderShapes;
    uint32_t numConvexShapes;
    uint32_t numConeShapes;
    uint32_t numTriMeshShapes;
    uint32_t numPlaneShapes;
    uint32_t numConstraints;
};

struct BatchedContactBufferStats
{
    size_t sceneCount = 0;
    size_t liveContactReportCount = 0;
    size_t headerCapacity = 0;
    size_t dataCapacity = 0;
    size_t dataSize = 0;
};

struct IPhysxUnitTests
{
    void(CARB_ABI* update)(float elapsedSecs, float currentTime);

    PhysicsStats(CARB_ABI* getPhysicsStats)();

    BatchedContactBufferStats(CARB_ABI* getBatchedContactBufferStats)();

    float(CARB_ABI* getMassInformation)(const char* path, carb::Float3& inertia, carb::Float3& com);

    void(CARB_ABI* getMaterialsPaths)(const TestPathArg& path, std::vector<TestPathArg>& materials);

    void(CARB_ABI* startLoggerCheck)(const char* message, bool expectedResult, bool partialStringMatch);

    void(CARB_ABI* startLoggerCheckForMultiple)(std::vector<std::string>& messages,
                                                bool expectedResult,
                                                bool expectAll,
                                                bool partialStringMatch);

    bool(CARB_ABI* endLoggerCheck)();

    // Note: this is the same underlying implementer as IPhysxPrivate::getPhysXPtrInstanced (see
    // ADR-0019); it is retyped here purely to keep both ABI declarations matching that single
    // implementer's signature, not as a scope decision for IPhysxUnitTests/IPhysxBenchmarks (which
    // stay SdfPath-typed -- test-only code is explicitly out of ADR-0019's scope).
    uint32_t(CARB_ABI* getPhysXPtrInstanced)(omni::physics::parse::ObjectKey key, void** data, uint32_t dataSize, PhysXType type);

    void(CARB_ABI* updateCooking)();

    bool(CARB_ABI* isCudaLibPresent)();

    // Number of InternalActor entries registered with the given physics scene. Exposed so tests can
    // assert that the per-scene actor bookkeeping follows a simulation owner change; returns 0 if the
    // path does not resolve to a scene.
    size_t(CARB_ABI* getSceneInternalActorCount)(const TestPathArg& scenePath);

    // REQ-SIM-SCENEQUERY-001 observability: raycastFilterExcludeInvisible::preFilter() and the
    // per-query KnownTokens::intern() call site in raycastSingle() (Raycast.cpp) each increment a
    // counter tests can read around a raycast to prove the call ran, without depending on
    // isInteractiveActorRaycast's timing-dependent boolean return.
    void(CARB_ABI* resetRaycastQueryTestCounters)();
    uint32_t(CARB_ABI* getRaycastPreFilterCallCount)();
    uint32_t(CARB_ABI* getRaycastQueryInternCount)();
};

struct PhysicsProfileStats
{
    const char* zoneName;
    float ms; // milliseconds
};


typedef void (*ProfileStatsNotificationFn)(const std::vector<PhysicsProfileStats>& profileStats, void* userData);

struct IPhysxBenchmarks
{
    void(CARB_ABI* update)(float elapsedSecs, float currentTime);

    void(CARB_ABI* updateUsd)();

    long int(CARB_ABI* createEmptyStage)();

    long(CARB_ABI* loadTargetStage)(const char* path);

    bool(CARB_ABI* loadTargetStage_Id)(long id);

    void(CARB_ABI* setThreadCount)(uint32_t threadCount);

    void(CARB_ABI* overwriteGPUSetting)(bool enableGpu);

    void(CARB_ABI* enablePVDProfile)(bool enableProfile);

    void(CARB_ABI* enableProfile)(bool enableProfile);

    void(CARB_ABI* getProfileStats)(std::vector<PhysicsProfileStats>& stats);

    /// Subscribe to physics simulation profile stats event.
    ///
    /// \note Subscription cannot be changed in the onEvent callback
    /// \note That if subscription is used the getProfileStats will not return any
    /// results as after the subscription send the results are cleared.
    ///
    /// \param onEvent The callback function to be called with the profile data.
    /// \param userData The userData to be passed back in the callback function.
    /// \return Subscription Id for release, kInvalidSubscriptionId is returned if the opetation failed
    SubscriptionId(CARB_ABI* subscribeProfileStatsEvents)(ProfileStatsNotificationFn onEvent, void* userData);

    /// Unsubscribes to simulation events.
    ///
    /// \note Subscription cannot be changed in the onEvent callback
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribeProfileStatsEvents.
    void(CARB_ABI* unsubscribeProfileStatsEvents)(SubscriptionId subscriptionId);
};


} // namespace physx
} // namespace omni
