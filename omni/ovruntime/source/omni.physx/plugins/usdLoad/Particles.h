// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"


namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

ParticleSystemDesc* ParseParticleSystem(AttachedStage& attachedStage, const PXR_NS::UsdPrim& usdPrim);

PBDMaterialDesc* ParsePBDParticleMaterial(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim);

ParticleSetDesc* ParseParticleSet(AttachedStage& attachedStage, const PXR_NS::UsdPrim& usdPrim);

ParticleAnisotropyDesc* ParseParticleAnisotropy(const PXR_NS::UsdPrim& usdPrim);

ParticleSmoothingDesc* ParseParticleSmoothing(const PXR_NS::UsdPrim& usdPrim);

ParticleIsosurfaceDesc* ParseParticleIsosurface(const PXR_NS::UsdPrim& usdPrim);

ParticleSamplingDesc* ParseParticleSampling(AttachedStage& attachedStage, const PXR_NS::UsdPrim& usdPrim);

float completeRestOffset(PXR_NS::UsdStageWeakPtr stage, float restOffset, float particleContactOffset);
float completeContactOffset(PXR_NS::UsdStageWeakPtr stage, float contactOffset, float particleContactOffset);
float completeFluidRestOffset(PXR_NS::UsdStageWeakPtr stage, float fluidRestOffset, float particleContactOffset);
float completeSolidRestOffset(PXR_NS::UsdStageWeakPtr stage, float solidRestOffset, float particleContactOffset);
float completeParticleContactOffset(PXR_NS::UsdStageWeakPtr stage, float particleContactOffset);

} // namespace usdparser
} // namespace physx
} // namespace omni
