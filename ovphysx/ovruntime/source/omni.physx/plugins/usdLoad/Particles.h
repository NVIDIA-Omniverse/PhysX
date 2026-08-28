// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

#include <omni/physics/usd/StageScan.h>
#include <omni/physics/parse/Handles.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

// Source-decoupled builders: assemble the engine descriptor from the parse
// library's already-scanned descriptor (no USD attribute reads).  Cross-ref
// ObjectKeys are mapped to SdfPaths through `scanned.pathFor`; runtime
// ObjectId resolution (material / collisionGroup) + time-sampled wind
// registration stay here because they need the engine ObjectDb / callback.
ParticleSystemDesc* buildParticleSystemDesc(AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned, const omni::physics::parse::ParticleSystemDesc& scanDesc);

ParticleSetDesc* buildParticleSetDesc(AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned, const omni::physics::parse::ParticleSetDesc& scanDesc);

// Runtime variant: build the engine set descriptor from a parse descriptor
// re-read on demand through the persistent source (no ScannedStage; cross-ref
// keys resolve via attachedStage.pathFor). No validity gate — used to refresh
// data on an already-created set.
ParticleSetDesc* buildParticleSetDescRuntime(AttachedStage& attachedStage,
    const omni::physics::parse::ParticleSetDesc& scanDesc);

PBDMaterialDesc* ParsePBDParticleMaterial(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& materialPath);
PBDMaterialDesc* ParsePBDParticleMaterial(AttachedStage& attachedStage,
                                          const omni::physics::parse::ObjectKey& materialKey);

float completeRestOffset(float metersPerUnit, float restOffset, float particleContactOffset);
float completeContactOffset(float metersPerUnit, float contactOffset, float particleContactOffset);
float completeFluidRestOffset(float metersPerUnit, float fluidRestOffset, float particleContactOffset);
float completeSolidRestOffset(float metersPerUnit, float solidRestOffset, float particleContactOffset);
float completeParticleContactOffset(float metersPerUnit, float particleContactOffset);

} // namespace usdparser
} // namespace physx
} // namespace omni
