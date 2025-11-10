// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <internal/Internal.h>

namespace physx
{
class PxGeometry;
}

namespace omni
{
namespace pointfinder
{

uint64_t createPointFinder(const carb::Float3* points, const uint32_t pointsSize);
void releasePointFinder(const uint64_t pointFinder);
void pointsToIndices(int32_t* pointIndices,
                     const uint64_t pointFinder,
                     const carb::Float3* points,
                     const uint32_t pointsSize);

} // namespace pointfinder
} // namespace omni
