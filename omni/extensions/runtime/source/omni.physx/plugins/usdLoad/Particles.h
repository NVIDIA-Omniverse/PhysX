// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

ParticleSystemDesc* ParseParticleSystem(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);

PBDMaterialDesc* ParsePBDParticleMaterial(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim);

ParticleSetDesc* ParseParticleSet(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);

ParticleAnisotropyDesc* ParseParticleAnisotropy(const pxr::UsdPrim& usdPrim);

ParticleSmoothingDesc* ParseParticleSmoothing(const pxr::UsdPrim& usdPrim);

ParticleIsosurfaceDesc* ParseParticleIsosurface(const pxr::UsdPrim& usdPrim);

ParticleClothDesc* ParseParticleClothDeprecated(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);

ParticleSamplingDesc* ParseParticleSampling(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);

float completeRestOffset(pxr::UsdStageWeakPtr stage, float restOffset, float particleContactOffset);
float completeContactOffset(pxr::UsdStageWeakPtr stage, float contactOffset, float particleContactOffset);
float completeFluidRestOffset(pxr::UsdStageWeakPtr stage, float fluidRestOffset, float particleContactOffset);
float completeSolidRestOffset(pxr::UsdStageWeakPtr stage, float solidRestOffset, float particleContactOffset);
float completeParticleContactOffset(pxr::UsdStageWeakPtr stage, float particleContactOffset);

} // namespace usdparser
} // namespace physx
} // namespace omni
