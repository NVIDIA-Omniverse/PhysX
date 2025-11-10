// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "internal/Internal.h"

#include <PxPhysicsAPI.h>

namespace omni
{

namespace physx
{

::physx::PxCapsuleControllerDesc parsePhysXCharacterControllerDesc(const pxr::UsdStageRefPtr stage,
                                                                   const pxr::UsdPrim& usdPrim,
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
