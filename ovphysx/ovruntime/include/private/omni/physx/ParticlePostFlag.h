// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-24
 */

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

// particle post process types
struct ParticlePostFlag
{
    enum Enum
    {
        eNone = 0,
        eAnisotropy = (1 << 0), //< compute particle anisotropy based on neighborhood
        eSmoothing = (1 << 1), //< compute smoothed particle positions based on neighborhood
        eIsosurface = (1 << 2), //< compute particle isosurface mesh
    };
};

// ADR-0019 (public-api-object-identity) note: this header used to also declare
// IPhysxParticlesPrivate, an ABI struct mirroring the particles namespace's
// create/release/update-postprocess and particle-sampler free functions in
// PhysXParticlePost.cpp/PhysXParticleSampling.cpp. A repo-wide grep found no
// fillInterface (or any other) call that ever populates its function pointers --
// it was dead ABI, never reachable from any consumer. It has been deleted rather
// than mechanically retyped to ObjectKey, since retyping it would have produced a
// live-looking-but-still-unpopulated struct implying functionality that doesn't
// exist. The real call surface (the particles-namespace free functions) remains
// SdfPath-typed: those functions share mutable state (gPostprocessRefMap,
// gParticlePrimsToFactoryMap) with a much larger set of non-ABI-mirroring
// overloads used pervasively by InternalParticle.cpp, InternalScene.cpp,
// PhysXParticlePropertiesUpdate.cpp, UsdInterfaceParticle.cpp and LoadStage.cpp --
// none of which are part of the runtime's public API (ADR-0019 scope is
// include/omni/physx/** and include/private/omni/physx/**) and none of which were
// in this increment's declared file scope, so retyping them was left undone here
// rather than ballooning into an unbounded, higher-risk internal-implementation
// migration for zero externally-visible benefit.
//
// This header now only carries ParticlePostFlag, which is live, unrelated
// functionality used across the particles subsystem.

} // namespace physx
} // namespace omni
