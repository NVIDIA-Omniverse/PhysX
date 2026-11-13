// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-1
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-6 AC-16
 *
 * @implements REQ-READ-CCT-001
 * @covers AC-3

 * @implements REQ-READ-ATTRS-001
 * @covers AC-30
 */

#pragma once

#include "tensors/CommonTypes.h"

#include <vector>

namespace physx
{
class PxRigidBody;
class PxScene;
}

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}
namespace tensors
{

// Creation-time controller actor to prim frame correction; identity for other bodies.
::physx::PxQuat bodyOrientationFixup(const usdparser::AttachedStage* owner, const ::physx::PxRigidBody* body);

// The signed world scale `body` was created with, from its owning database record (rigid actor,
// articulation link, or character-controller proxy); unit for an unmanaged actor. Fills
// RigidBodyEntry::worldScale for both entry builders, so a superset and a pattern view agree.
::physx::PxVec3 bodyWorldScale(const usdparser::AttachedStage* owner, const ::physx::PxRigidBody* body);

// Every rigid dynamic and articulation link in the scene, in a stable order: rigid dynamics in
// PxScene actor order, then each articulation's links in link order.
//
// Shared by the CPU and GPU backends: both must enumerate the same bodies in the same order, or a
// row index taken against one would silently address a different body in the other.
// Diagnostic paths come from the scene owner's database, independent of optional PhysX actor names.
void collectSupersetRigidEntries(::physx::PxScene* scene, const usdparser::AttachedStage* owner,
                                std::vector<RigidBodyEntry>& entries);

} // namespace tensors
} // namespace physx
} // namespace omni
