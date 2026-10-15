// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-POINTS-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/AuthoringPoints.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>

namespace omni::physx::authoring
{
bool matchPoints(const ::physx::PxVec3* source, uint32_t sourceCount,
                 const ::physx::PxVec3* target, uint32_t targetCount, float distance,
                 std::vector<int32_t>& output)
{
    if ((!source && sourceCount) || (!target && targetCount) || !std::isfinite(distance) || distance < 0 ||
        targetCount > uint32_t(std::numeric_limits<int32_t>::max())) return false;
    for (uint32_t i = 0; i < sourceCount; ++i) if (!source[i].isFinite()) return false;
    for (uint32_t i = 0; i < targetCount; ++i) if (!target[i].isFinite()) return false;
    std::vector<uint32_t> order(targetCount);
    std::iota(order.begin(), order.end(), 0u);
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) { return target[a].x < target[b].x; });
    std::vector<int32_t> matches(sourceCount, -1);
    const double limit = double(distance)*distance;
    for (uint32_t i = 0; i < sourceCount; ++i)
    {
        double best = limit;
        const double lower = double(source[i].x)-distance, upper = double(source[i].x)+distance;
        const std::vector<uint32_t>::iterator first = std::lower_bound(order.begin(), order.end(), lower,
            [&](uint32_t j, double x) { return double(target[j].x) < x; });
        for (std::vector<uint32_t>::iterator it = first; it != order.end() && double(target[*it].x) <= upper; ++it)
        {
            const uint32_t j = *it;
            const double x = double(source[i].x)-target[j].x;
            const double y = double(source[i].y)-target[j].y;
            const double z = double(source[i].z)-target[j].z;
            const double squared = x*x+y*y+z*z;
            if (squared < best || (squared == best && (matches[i] < 0 || j < uint32_t(matches[i]))))
            {
                matches[i] = int32_t(j);
                best = squared;
            }
        }
    }
    output.swap(matches);
    return true;
}
}
