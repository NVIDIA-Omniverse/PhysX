// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/ScannedStage.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

// Source-decoupled builders: assemble the engine descriptor from the parse library's
// already-scanned descriptor (no USD attribute reads). Cross-ref ObjectKeys are re-keyed into
// attachedStage's namespace through `scanned.source().sourceKeyToString()` +
// `attachedStage.keyFor(std::string_view)` (the opaque-identity round trip used throughout this
// file's callers, e.g. LoadStage.cpp's Shapes loop / ScannedShapeCookingDispatch.h); runtime
// ObjectId resolution (material / collisionGroup) + time-sampled wind registration stay here
// because they need the engine ObjectDb / callback. Unconditional: takes the backend-agnostic
// `parse::ScannedStage` base, same as ScannedShapeCookingDispatch.h.
ParticleSystemDesc* buildParticleSystemDesc(AttachedStage& attachedStage,
    const omni::physics::parse::ScannedStage& scanned, const omni::physics::parse::ParticleSystemDesc& scanDesc);

ParticleSetDesc* buildParticleSetDesc(AttachedStage& attachedStage,
    const omni::physics::parse::ScannedStage& scanned, const omni::physics::parse::ParticleSetDesc& scanDesc);

// Runtime variant: build the engine set descriptor from a parse descriptor
// re-read on demand through the persistent source (no ScannedStage; cross-ref
// keys resolve via attachedStage.pathFor). No validity gate — used to refresh
// data on an already-created set.
ParticleSetDesc* buildParticleSetDescRuntime(AttachedStage& attachedStage,
    const omni::physics::parse::ParticleSetDesc& scanDesc);

float completeRestOffset(float metersPerUnit, float restOffset, float particleContactOffset);
float completeContactOffset(float metersPerUnit, float contactOffset, float particleContactOffset);
float completeFluidRestOffset(float metersPerUnit, float fluidRestOffset, float particleContactOffset);
float completeSolidRestOffset(float metersPerUnit, float solidRestOffset, float particleContactOffset);
float completeParticleContactOffset(float metersPerUnit, float particleContactOffset);

} // namespace usdparser
} // namespace physx
} // namespace omni
