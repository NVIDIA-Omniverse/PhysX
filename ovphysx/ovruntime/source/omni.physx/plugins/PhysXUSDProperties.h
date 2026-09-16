// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "internal/Internal.h"

#include <PxPhysicsAPI.h>

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

inline ::physx::PxArticulationDrive toPhysX(const omni::physx::usdparser::PhysxJointDrive& drive)
{
    if (drive.isEnvelopeUsed)
        return ::physx::PxArticulationDrive(drive.stiffness, drive.damping, ::physx::PxPerformanceEnvelope(drive.forceLimit, drive.maxActuatorVelocity, drive.velocityDependentResistance, drive.speedEffortGradient),
            drive.acceleration ? ::physx::PxArticulationDriveType::eACCELERATION :
                                ::physx::PxArticulationDriveType::eFORCE);
    return ::physx::PxArticulationDrive(drive.stiffness, drive.damping, drive.forceLimit,
                                        drive.acceleration ? ::physx::PxArticulationDriveType::eACCELERATION :
                                                             ::physx::PxArticulationDriveType::eFORCE);
}
} // namespace physx
} // namespace omni
