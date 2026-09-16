// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-1
 */

#pragma once

#include "tensors/CommonTypes.h"

#include <vector>

namespace physx
{
class PxScene;
}

namespace omni
{
namespace physx
{
namespace tensors
{

// Every rigid dynamic and articulation link in the scene, in a stable order: rigid dynamics in
// PxScene actor order, then each articulation's links in link order.
//
// Shared by the CPU and GPU backends: both must enumerate the same bodies in the same order, or a
// row index taken against one would silently address a different body in the other.
void collectSupersetRigidEntries(::physx::PxScene* scene, std::vector<RigidBodyEntry>& entries);

} // namespace tensors
} // namespace physx
} // namespace omni
