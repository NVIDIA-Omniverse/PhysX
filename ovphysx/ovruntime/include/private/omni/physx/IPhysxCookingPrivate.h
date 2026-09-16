// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-16
 *
 * @implements REQ-PUBLICAPI-002
 * @covers AC-10
 */

#pragma once
#include <omni/physx/IPhysxCooking.h>

namespace physx
{
class PxCudaContextManager;
}
namespace omni
{
namespace physx
{

struct PhysxCookingStatistics
{
    int32_t totalFinishedTasks = 0; /// How many cooking tasks have been finished
    int32_t totalFinishedCacheMissTasks = 0; /// How many cooking tasks have been finished with a MISS from cache
    int32_t totalFinishedCacheHitTasks = 0; /// How many cooking tasks have been finished with a HIT from cache
    int32_t totalScheduledTasks = 0; /// How many cooking tasks have been scheduled
    int32_t totalWarningsFailedGPUCompatibility = 0; /// How many failed gpu compatibility warnings have been issued
    int32_t totalWarningsConvexPolygonLimitsReached = 0; /// How many times convex polygon limits have been reached
};

struct IPhysxCookingPrivate
{
    /// Get cooking statistics.
    ///
    /// Obtain statistics about cooking (quantities and timestamps)
    ///
    /// \return Statistics about cooking
    PhysxCookingStatistics(CARB_ABI* getCookingStatistics)();


    /// Adds a primitive to the cooking refresh set, i.e. this prim might have to have its collision data refreshed.
    /// Named by an explicit AttachHandle rather than resolved against "the active attach" (ADR-0016
    /// Decision 4): an unresolvable handle is rejected with a diagnostic rather than silently
    /// dropping the request.
    //\param[in] key prim's ObjectKey
    //\param[in] attachHandle the attach that key was resolved from
    void(CARB_ABI* addPrimToCookingRefreshSetForAttach)(omni::physics::parse::ObjectKey key, omni::physics::AttachHandle attachHandle);

    /// Releases the runtime mesh cache
    void(CARB_ABI* releaseRuntimeMeshCache)();

};
} // namespace physx
} // namespace omni
