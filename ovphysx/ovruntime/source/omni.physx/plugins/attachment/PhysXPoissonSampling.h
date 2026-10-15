// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-20 AC-21
 */

#pragma once

#include <omni/physics/AttachHandle.h>
#include <internal/Internal.h>

namespace omni::physx::usdparser { class AttachedStage; }

namespace omni
{
namespace sampling
{

uint64_t createSurfaceSampler(omni::physics::AttachHandle attach, omni::physics::parse::ObjectKey colliderKey, float samplingDistance);
void releaseSurfaceSampler(const uint64_t surfaceSampler);
void notifyPhysXSceneRelease();
void releaseAttachmentSurfaceSamplers(omni::physx::usdparser::AttachedStage& owner);
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
