// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <internal/Internal.h>

namespace omni
{
namespace sampling
{

uint64_t createSurfaceSampler(const pxr::SdfPath& colliderPath, float samplingDistance);
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
