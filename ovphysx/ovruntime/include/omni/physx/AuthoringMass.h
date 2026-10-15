// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-MASS-001
// @covers AC-1 AC-2 AC-3

#pragma once
#include <foundation/PxTransform.h>
#include <foundation/PxBounds3.h>
#include <cstdint>

namespace omni::physx::authoring
{
enum class PrimitiveKind : uint32_t { Box, Sphere, Capsule };
struct MassShape
{
    PrimitiveKind kind;
    // Box half extents; sphere radius in x; capsule radius/half height in x/y.
    // Capsule axis is local X, following PhysX. All lengths are meters.
    ::physx::PxVec3 dimensions;
    ::physx::PxTransform pose;
    float density; // kg / m^3
};
struct MassResult
{
    float mass; // kg
    float volume; // m^3 (sum of collider volumes, including overlap)
    ::physx::PxVec3 centerOfMass; // body-local meters
    ::physx::PxVec3 diagonalInertia; // kg m^2
    ::physx::PxQuat principalAxes; // inertia frame to body-local frame
    ::physx::PxBounds3 bounds; // body-local meters
};

// Pure stopped computation: no SDK singleton, scene, USD or runtime initialization.
// Failure preserves output; invalidShape names the failing collider (UINT32_MAX
// for an invalid aggregate). Callers retain the input array for the call duration.
bool computePrimitiveMass(const MassShape* shapes, uint32_t count, MassResult& output,
                          uint32_t& invalidShape);
}
