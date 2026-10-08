// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-11
 */

#include "common/foundation/TransformedExtent.h"

using namespace physx;

namespace omni
{
namespace physx
{

void computeTransformedExtent(const ::physx::PxBounds3& worldBounds, const ::physx::PxMat44d& m,
                              carb::Float3 (&outExtent)[2])
{
    const double lo[3] = { worldBounds.minimum.x, worldBounds.minimum.y, worldBounds.minimum.z };
    const double hi[3] = { worldBounds.maximum.x, worldBounds.maximum.y, worldBounds.maximum.z };

    if (lo[0] > hi[0] || lo[1] > hi[1] || lo[2] > hi[2])
    {
        outExtent[0] = carb::Float3{ float(lo[0]), float(lo[1]), float(lo[2]) };
        outExtent[1] = carb::Float3{ float(hi[0]), float(hi[1]), float(hi[2]) };
        return;
    }

    double newMin[3];
    double newMax[3];
    for (int i = 0; i < 3; ++i)
    {
        newMin[i] = newMax[i] = m[3][i];
        for (int j = 0; j < 3; ++j)
        {
            const double a = m[j][i] * lo[j];
            const double b = m[j][i] * hi[j];
            if (a < b)
            {
                newMin[i] += a;
                newMax[i] += b;
            }
            else
            {
                newMin[i] += b;
                newMax[i] += a;
            }
        }
    }

    outExtent[0] = carb::Float3{ float(newMin[0]), float(newMin[1]), float(newMin[2]) };
    outExtent[1] = carb::Float3{ float(newMax[0]), float(newMax[1]), float(newMax[2]) };
}

} // namespace physx
} // namespace omni
