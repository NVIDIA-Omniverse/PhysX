// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCookingPrivate", 2, 0)
    /// Get cooking statistics.
    ///
    /// Obtain statistics about cooking (quantities and timestamps)
    ///
    /// \return Statistics about cooking
    PhysxCookingStatistics(CARB_ABI* getCookingStatistics)();


    /// Adds a primitive to the cooking refresh set, i.e. this prim might have to have its collision data refreshed
    //\param[in] path prim's path
    void(CARB_ABI* addPrimToCookingRefreshSet)(const pxr::SdfPath& path);

    /// Releases the runtime mesh cache
    void(CARB_ABI* releaseRuntimeMeshCache)();

};
} // namespace physx
} // namespace omni
