// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include <carb/events/IEvents.h>
#include <carb/tasking/TaskingUtils.h>
#include <carb/eventdispatcher/IEventDispatcher.h>
#include "common/utilities/MemoryUtilities.h"
#include "../MeshTypes.h"

// This class is a simplified version of the regular simulation cooking.
// This acts as a cache where you can query for a PhysX cooked mesh, and before executing the actual cooking,
// it searches a first level memory cache and a second level disk cache.
// The memory cache is cleared on simulation play/stop, omnigraph play/stop, animation play/stop or stage open/close.
// The disk cache follows the same "Local Mesh Cache Size MB" setting and is enabled by the same "Enable Local Mesh
// Cache" settings. as regular omni.physx cooking system, even if it lives in a separate directory / instance. If the
// "Enable Local Mesh Cache" is toggled from true to false, we also reset the database. The hash key is a Mesh hash
// using vertices/indices/etc (plus sign scale) and it's unique per mesh before considering collision parameters. The
// cache hit is done by creating mesh CRC, that is a combination of a mesh hash and the collision parameter (mainly the
// approximation type for now) Mesh is triangulated and stored in both level of caches to avoid re-triangulation as
// well. Main differences with omni.physx cooking are:
//
// 0. It cooks with gpu flags disabled
//
// 1. It works on a "MeshView" (contiguous ranges of indices/vertices) instead of reading data directly from USD (so it
// can be used even in OG/Fabric).
//
// 2. It can be used from a multihreaded caller, but it doesn't have its own task system.
//
// 3. It doesn't save data to CookApi because it potentially can be used on generated meshes that don't exist in USD
//
// 4. Currently only Triangle and Convex approximation are supported
//
// PxPhysics object is lazily created when the first mesh needs to be created, so that we have its associated tolerances
// scale coming from the source stage that the OG node lives on. Such object is released at sim/anim/og stop event,
// together with all the meshes.
namespace omni
{
namespace physx
{
struct IPhysxCookingServicePrivate;
namespace graph
{
class ImmediateMeshCache
{
public:
    ImmediateMeshCache()
    {
    }
    ~ImmediateMeshCache()
    {
        release();
    }

    ImmediateMeshCache(const ImmediateMeshCache&) = delete;
    ImmediateMeshCache& operator=(const ImmediateMeshCache&) = delete;

    bool initialize(::physx::PxFoundation& foundation);
    void release();

    // Returns a (possibly cached) Px___Mesh object from a given MeshDefinitionView. Can be invoked from any thread.
    bool queryTriangleApproximationFor(MeshDefinitionView request,
                                       ::physx::PxTriangleMesh*& outMesh,
                                       MeshFacesToTrianglesMapping*& outMeshMapping);
    bool queryConvexApproximationFor(MeshDefinitionView request,
                                     ::physx::PxConvexMesh*& outMesh,
                                     MeshFacesToTrianglesMapping*& outMeshMapping);

    // Combines an existing meshKey (if available) with Collision Parameters to create an unique CRC.
    bool computeCookedDataCRCForMeshInputView(const MeshInputView& meshView, MeshKey& meshKey, MeshKey& cookedDataCRC);

private:
    using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
    // This is a map with an associated mutex to access it
    template <typename ValueType>
    struct MeshKeyMapWithMutex
    {
        std::unordered_map<MeshKey, ValueType, MeshKeyHash> cache;
        carb::tasking::MutexWrapper mutex;

        void releaseWithLock()
        {
            lock_guard lock(mutex);
            cache.clear();
        }
    };

    // Acquired interfaces
    omni::physx::IPhysxCookingService* iCookingService = nullptr;

    // Actual runtime data
    carb::tasking::MutexWrapper mPhysicsMutex;
    ::physx::PxPhysics* mPhysics = nullptr;
    ::physx::PxFoundation* mFoundation = nullptr;

    // Memory Cache (in RAM)
    MeshKeyMapWithMutex<std::unique_ptr<MeshFacesToTrianglesMapping>> mTriangulationCache;

    // CallReleaseOnPointer will...call Px**Mesh::release() when unique_ptr is removed from the map
    MeshKeyMapWithMutex<std::unique_ptr<::physx::PxTriangleMesh, CallReleaseOnPointer>> mTriangleMeshCache;
    MeshKeyMapWithMutex<std::unique_ptr<::physx::PxConvexMesh, CallReleaseOnPointer>> mConvexMeshCache;

    // Subscriptions
    std::array<carb::eventdispatcher::ObserverGuard, 8> mStageEvtSub;

    // Makes sure that a MeshTriangulation exists for a given MeshDefinitionView. Eventually reads it from cache
    MeshFacesToTrianglesMapping* ensureTriangulatedMeshFromMemoryCache(const MeshDefinitionView request);

    // Actually inserts the triangulated mesh into the cache
    MeshFacesToTrianglesMapping* insertNewTriangulatedMesh(
        const omni::physx::PhysxCookingMeshTriangulationView& triangulationView, MeshKey meshKey);

    // Generic implementation of the "query" for a give PhysX Collision approximation, that can be called
    // for any type customizing only the setup and call of cooking function and the actual creation of
    // the collision object. This has been made to reduce code duplication.
    template <typename PhysxType, typename MemoryCache, typename CookingLambdaFunction, typename CreateLambdaFunction>
    bool queryApproximationGeneric(MeshDefinitionView request,
                                   PhysxType*& outMesh,
                                   MeshFacesToTrianglesMapping*& outMeshMapping,
                                   MemoryCache& memoryCache,
                                   carb::tasking::MutexWrapper& memoryMutex,
                                   CookingLambdaFunction cookMeshLambda,
                                   CreateLambdaFunction createObjectLambda);

    // Lazily Initialize PxPhysics object with tolerances scale
    bool ensurePxPhysics(::physx::PxTolerancesScale tolerancesScale);

    // Other methods
    void releaseMemoryCache();
    void subscribeToStageEvents();
    void unsubscribeFromStageEvents();
};
} // namespace graph
} // namespace physx
} // namespace omni
