// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
