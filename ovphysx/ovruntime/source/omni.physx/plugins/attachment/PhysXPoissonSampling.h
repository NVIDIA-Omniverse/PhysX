// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-20 AC-21
 */

#pragma once

#include <internal/Internal.h>

namespace omni
{
namespace sampling
{

uint64_t createSurfaceSampler(omni::physics::parse::ObjectKey colliderKey, float samplingDistance);
void releaseSurfaceSampler(const uint64_t surfaceSampler);
void notifyPhysXSceneRelease();
void addSurfaceSamplerPoints(const uint64_t surfaceSampler, const carb::Float3* points, const uint32_t pointsSize);
void removeSurfaceSamplerPoints(const uint64_t surfaceSampler, const carb::Float3* points, const uint32_t pointsSize);
void sampleSurface(carb::Float3*& points,
                   uint32_t& pointsSize,
                   const uint64_t surfaceSampler,
                   const carb::Float3& sphereCenter,
                   const float sphereRadius,
                   const float samplingDistance,
                   void* (*allocateBytes)(size_t));
void getSurfaceSamplerPoints(carb::Float3*& points,
                             uint32_t& pointsSize,
                             const uint64_t surfaceSampler,
                             void* (*allocateBytes)(size_t));
void createTriMeshSampler(const uint64_t surfaceSampler);
bool isPointInside(const uint64_t surfaceSampler, const carb::Float3 point);

} // namespace sampling
} // namespace omni
