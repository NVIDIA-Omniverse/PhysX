// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Types.h>

#include <omni/physx/IPhysxSceneQuery.h>

namespace omni
{
namespace physx
{
bool raycastClosest(
    const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& outHit, bool bothSides);
bool sweepSphereClosest(float radius,
                        const carb::Float3& origin,
                        const carb::Float3& unitDir,
                        float distance,
                        SweepHit& outHit,
                        bool bothSides);

} // namespace physx
} // namespace omni
