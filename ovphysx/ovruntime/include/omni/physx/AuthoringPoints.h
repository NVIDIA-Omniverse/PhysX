// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-POINTS-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include <foundation/PxVec3.h>
#include <cstdint>
#include <vector>

namespace omni::physx::authoring
{
// Same Cartesian frame/units on both inputs. One target index per source (-1
// when outside inclusive distance). Ties choose the lowest target index; target
// reuse is allowed. No foundation/SDK/attachment required. Failure preserves
// output. Caller owns input arrays through return. Empty arrays are valid.
bool matchPoints(const ::physx::PxVec3* source, uint32_t sourceCount,
                 const ::physx::PxVec3* target, uint32_t targetCount, float distance,
                 std::vector<int32_t>& output);
}
