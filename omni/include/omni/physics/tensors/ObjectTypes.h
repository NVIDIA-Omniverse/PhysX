// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
    eInvalid, //!< No object/invalid object type 
    eRigidBody, //!< Rigid body object not in an articulation
    eArticulation, //!< Articulation object
    eArticulationLink, //!< Articulation link object
    eArticulationRootLink, //!< Articulation root link object
    eArticulationJoint, //!< Articulation Joint object
    eTypeCount //!< Total number of object types
};


} // namespace tensors
} // namespace physics
} // namespace omni
