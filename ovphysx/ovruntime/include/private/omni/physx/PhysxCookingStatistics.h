// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-PUBLICAPI-001
// @covers AC-52
#pragma once
#include <cstdint>

namespace omni::physx
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
} // namespace omni::physx
