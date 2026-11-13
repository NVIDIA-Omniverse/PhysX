// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-OMNIPVD-CAPTURE-001
 * @covers AC-5
 * @implements REQ-OMNIPVD-CONTROL-001
 * @covers AC-2
 */

#pragma once

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/OmniPvdCaptureResolver.h>
#include <string>
#include <vector>

namespace omni
{
namespace physx
{

// Selectable object types mirror PxOmniPvdFilterDesc. Dependencies are included by
// the SDK; runtime helper pointers and articulation links are never selected directly.
void omniPvdAppendCaptureObjects(PhysXType type, void* pointer, std::vector<const ::physx::PxBase*>& objects);
inline const ::physx::PxBase* omniPvdCaptureObject(PhysXType type, void* pointer)
{
    if (!pointer)
        return nullptr;
    switch (type)
    {
    case ePTActor: return static_cast<const ::physx::PxRigidActor*>(pointer);
    case ePTArticulation:
    case ePTArticulationFixedBase: return static_cast<const ::physx::PxArticulationReducedCoordinate*>(pointer);
    case ePTParticleSystem: return static_cast<const ::physx::PxPBDParticleSystem*>(pointer);
    case ePTParticleSet: return static_cast<const ::physx::PxParticleAndDiffuseBuffer*>(pointer);
    case ePTDeformableSurface: return static_cast<const ::physx::PxDeformableSurface*>(pointer);
    case ePTDeformableVolume: return static_cast<const ::physx::PxDeformableVolume*>(pointer);
    default: return nullptr;
    }
}

constexpr PhysXType kOmniPvdCaptureObjectTypes[] = {
    ePTActor, ePTArticulation, ePTArticulationFixedBase, ePTParticleSystem,
    ePTParticleSet, ePTDeformableSurface, ePTDeformableVolume
};

} // namespace physx
} // namespace omni
