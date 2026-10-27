// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/IEvents.h>
#include <omni/physx/IPhysx.h>

#include <unordered_map>

namespace omni
{

namespace physx
{

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

struct IPhysxUnitTests
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxUnitTests", 3, 2)

    void(CARB_ABI* update)(float elapsedSecs, float currentTime);

    PhysicsStats(CARB_ABI* getPhysicsStats)();

    float(CARB_ABI* getMassInformation)(const char* path, carb::Float3& inertia, carb::Float3& com);

    void(CARB_ABI* getMaterialsPaths)(const PXR_NS::SdfPath& path, std::vector<PXR_NS::SdfPath>& materials);

    void(CARB_ABI* startLoggerCheck)(const char* message, bool expectedResult, bool partialStringMatch);

    void(CARB_ABI* startLoggerCheckForMultiple)(std::vector<std::string>& messages,
                                                bool expectedResult,
                                                bool expectAll,
                                                bool partialStringMatch);

    bool(CARB_ABI* endLoggerCheck)();

    uint32_t(CARB_ABI* getPhysXPtrInstanced)(const PXR_NS::SdfPath& path, void** data, uint32_t dataSize, PhysXType type);

    void(CARB_ABI* updateCooking)();

    bool(CARB_ABI* isCudaLibPresent)();

    // Number of InternalActor entries registered with the given physics scene. Exposed so tests can
    // assert that the per-scene actor bookkeeping follows a simulation owner change; returns 0 if the
    // path does not resolve to a scene.
    size_t(CARB_ABI* getSceneInternalActorCount)(const PXR_NS::SdfPath& scenePath);

    // Looks up the InternalForce registered for a PhysxForceAPI prim. Returns false when the path has
    // no force; otherwise writes the PxRigidActor the force is bound to - which must be null once that
    // body has been released, rather than left dangling (NVBugs 6583666) - to outRigidActor.
    bool(CARB_ABI* getForceRigidActor)(const PXR_NS::SdfPath& forcePath, void** outRigidActor);

    // Number of InternalForce entries registered with the given physics scene, so tests can assert
    // that the per-scene force bookkeeping follows a simulation owner change the way the actor
    // bookkeeping does; returns 0 if the path does not resolve to a scene.
    size_t(CARB_ABI* getSceneInternalForceCount)(const PXR_NS::SdfPath& scenePath);
};

struct PhysicsProfileStats
{
    const char* zoneName;
    float ms; // milliseconds
};


typedef void (*ProfileStatsNotificationFn)(const std::vector<PhysicsProfileStats>& profileStats, void* userData);

struct IPhysxBenchmarks
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxBenchmarks", 1, 0)

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
