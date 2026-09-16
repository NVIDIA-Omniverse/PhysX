// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-OBJECTTYPE-001
 * @covers AC-1
 */

#pragma once

#include <carb/Types.h>

namespace omni
{
namespace physics
{
namespace tensors
{

enum class ObjectType : uint32_t
{
    eInvalid, //!< No classified simulation object at the path
    eRigidBody, //!< Rigid body object not in an articulation
    eArticulation, //!< Articulation object
    eArticulationLink, //!< Articulation link object
    eArticulationRootLink, //!< Articulation root link object
    eArticulationJoint, //!< Reduced-coordinate articulation joint object
    eJoint, //!< Maximal-coordinate (standalone) joint object (physx::PxJoint)
    eCustomJoint, //!< Plugin-registered custom joint (CustomPhysXJoint via ePTCustomJoint)
    eTypeCount //!< Total number of object types
};


} // namespace tensors
} // namespace physics
} // namespace omni
