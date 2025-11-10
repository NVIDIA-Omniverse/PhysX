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

enum class JointType : uint8_t
{
    eFixed,
    eRevolute,
    ePrismatic,
    eSpherical,
    eInvalid = 0xff,
};

enum class DofType : uint8_t
{
    eRotation,
    eTranslation,
    eInvalid = 0xff,
};

enum class DofMotion : uint8_t
{
    eFree,
    eLimited,
    eLocked,
    eInvalid = 0xff,
};

enum class DofDriveType : uint8_t
{
    eNone, //!< No drive or drive type is specified
    eForce, //!< The output of the implicit spring drive controller is a force/torque.
    eAcceleration, //!< The output of the implicit spring drive controller is a joint acceleration (use this to get
                   //!< (spatial)-inertia-invariant behavior of the drive).
};

struct DofProperties
{
    DofType type = DofType::eInvalid; //!< Type of dof (read-only property)

    bool hasLimits = false; //!< Flags whether the DOF has limits. (read-only property)
    float lower = 0.0f; //!< lower limit of DOF. In radians or meters (read-only property)
    float upper = 0.0f; //!< upper limit of DOF. In radians or meters (read-only property)

    DofDriveType driveMode = DofDriveType::eAcceleration; //!< Drive mode for the DOF. See DriveMode.
    float maxVelocity = std::numeric_limits<float>::max(); //!< Maximum velocity of DOF. In Radians/s, or m/s
    float maxEffort = std::numeric_limits<float>::max(); //!< Maximum effort of DOF. in N or Nm.
    float stiffness = 0.0f; //!< Stiffness of DOF.
    float damping = 0.0f; //!< Damping of DOF.
};


} // namespace tensors
} // namespace physics
} // namespace omni
