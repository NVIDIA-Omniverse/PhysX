// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-MASS-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <omni/physx/AuthoringMass.h>
namespace omni::physx::authoring
{
struct MassOverrides
{
    // Explicit mass, center, diagonal, axes. Unset values are ignored.
    uint32_t flags = 0;
    float mass = 0;
    ::physx::PxVec3 center{0}, diagonal{0};
    ::physx::PxQuat axes{::physx::PxIdentity};
};
struct GeneralMassShape
{
    // 0 box, 1 sphere, 2 capsule (as MassShape), 3 closed triangle mesh.
    uint32_t kind = 0;
    ::physx::PxVec3 dimensions{0};
    ::physx::PxTransform pose{::physx::PxIdentity};
    float density = 1000;
    const ::physx::PxVec3* points = nullptr;
    const uint32_t* triangles = nullptr;
    uint32_t pointCount = 0, triangleCount = 0;
    MassOverrides overrides;
};
// Pure SI computation, no SDK/runtime initialization. Meshes require one closed,
// connected, consistently oriented, non-self-intersecting triangular boundary.
// Either global winding is accepted. No repair or collider approximation occurs.
// Overlap contributes additively. Overrides are shape-local, then body-local.
// Failure preserves output and identifies a collider, or UINT32_MAX for body errors.
bool computeGeneralMass(const GeneralMassShape* shapes, uint32_t count,
                        const MassOverrides& body, MassResult& output, uint32_t& invalidShape);
}
