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

/// Transform
/**
 * Represents a pose (e.g. of a rigid body)
 */
struct Transform
{
    carb::Float3 p{ 0.0f, 0.0f, 0.0f }; //!< Position, in meters
    carb::Float4 r{ 0.0f, 0.0f, 0.0f, 1.0f }; //!< Rotation Quaternion, represented in the format $x^{i} + y^{j} +
                                              //!< z^{k} + w$
};

/// Velocity
/**
 * Holds linear and angular velocities, in $m/s$ and $radians/s$
 */
struct Velocity
{
    carb::Float3 linear{ 0.0f, 0.0f, 0.0f }; //!< Linear velocity component
    carb::Float3 angular{ 0.0f, 0.0f, 0.0f }; //!< angular velocity component
};


} // namespace tensors
} // namespace physics
} // namespace omni
