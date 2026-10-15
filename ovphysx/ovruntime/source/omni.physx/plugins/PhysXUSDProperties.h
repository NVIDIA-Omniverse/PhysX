// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-ARTJOINTDRIVE-002
 * @covers AC-2 AC-3 AC-5
 */

#pragma once

#include "internal/Internal.h"
#include "PhysXToolsCore.h"

#include <PxPhysicsAPI.h>

#include <cmath>

namespace omni
{

namespace physx
{

namespace usdparser
{
class AttachedStage;
}

// Reads PhysxCharacterControllerAPI through the parse source, so it resolves with
// no backing USD stage (it used to build the schema object on the stage, which is a
// null deref there rather than a silent miss).
::physx::PxCapsuleControllerDesc parsePhysXCharacterControllerDesc(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                   omni::physics::parse::ObjectKey key,
                                                                   float radius,
                                                                   float height);

// Every drive goes through a performance envelope: the drive's force limit (`physics:maxForce`) becomes the
// envelope's maxEffort, a force or torque. Without PhysxDrivePerformanceEnvelopeAPI the other envelope terms
// keep their defaults (unlimited actuator velocity, no resistance, no gradient), and the envelope is a plain
// effort limit. A zero force limit leaves the drive without effort.
inline ::physx::PxArticulationDrive toPhysX(const omni::physx::usdparser::PhysxJointDrive& drive)
{
    return ::physx::PxArticulationDrive(drive.stiffness, drive.damping, ::physx::PxPerformanceEnvelope(drive.forceLimit, drive.maxActuatorVelocity, drive.velocityDependentResistance, drive.speedEffortGradient),
        drive.acceleration ? ::physx::PxArticulationDriveType::eACCELERATION :
                            ::physx::PxArticulationDriveType::eFORCE);
}

// PhysxDrivePerformanceEnvelopeAPI values as parsing reads them: NaN reads as unauthored and gives the term's
// default, other values are clamped to [0, FLT_MAX], and all are converted from degrees on rotational axes. The
// unlimited actuator velocity FLT_MAX stays as it is.
inline float clampEnvelopeValue(float value, float fallback)
{
    if (std::isnan(value))
        return fallback;
    return value < 0.0f ? 0.0f : (value > FLT_MAX ? FLT_MAX : value);
}

inline float toPhysXMaxActuatorVelocity(float value, bool rotational)
{
    value = clampEnvelopeValue(value, FLT_MAX);
    return rotational && value < FLT_MAX ? degToRad(value) : value;
}

inline float toPhysXVelocityDependentResistance(float value, bool rotational)
{
    value = clampEnvelopeValue(value, 0.0f);
    return rotational ? radToDeg(value) : value;
}

inline float toPhysXSpeedEffortGradient(float value, bool rotational)
{
    value = clampEnvelopeValue(value, 0.0f);
    return rotational ? degToRad(value) : value;
}
} // namespace physx
} // namespace omni
