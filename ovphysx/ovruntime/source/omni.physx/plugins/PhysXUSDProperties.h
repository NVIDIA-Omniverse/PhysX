// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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

::physx::PxCapsuleControllerDesc parsePhysXCharacterControllerDesc(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                   const PXR_NS::UsdStageRefPtr stage,
                                                                   const PXR_NS::UsdPrim& usdPrim,
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
