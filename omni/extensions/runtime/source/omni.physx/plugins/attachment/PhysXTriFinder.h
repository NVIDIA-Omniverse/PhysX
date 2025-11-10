// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <internal/Internal.h>
#include "foundation/PxTransform.h"
#include "geometry/PxGeometry.h"

namespace physx
{
class PxGeometry;
}

namespace omni
{
namespace trifinder
{
uint64_t createTriFinder(const carb::Float3* points,
                         const uint32_t pointsSize,
                         const uint32_t* indices,
                         const uint32_t indicesSize,
                         bool bMakeCopy = false);
uint64_t createTriFinder(const carb::Float4* points,
                         const uint32_t pointsSize,
                         const uint32_t* indices,
                         const uint32_t indicesSize,
                         bool bMakeCopy = false);
void releaseTriFinder(const uint64_t triFinder);
void pointsToTriMeshLocal(int32_t* triIds,
                          carb::Float3* barycentricCoords,
                          const uint64_t triFinder,
                          const carb::Float3* points,
                          const uint32_t pointsSize);
void triMeshLocalToPoints(carb::Float3* points,
                          const uint64_t triFinder,
                          const int32_t* triIds,
                          const carb::Float3* barycentricCoords,
                          const uint32_t pointsSize);
void overlapTriMeshGeom(int32_t*& triIds,
                        uint32_t& triIdsSize,
                        const uint64_t triFinder,
                        const ::physx::PxGeometry& geom,
                        const ::physx::PxTransform& geomPos,
                        const float overlapOffset,
                        void* (*allocateBytes)(size_t),
                        void* triMeshSampler = nullptr);
const carb::Float3* getPoints(uint32_t& pointsSize, uint32_t& pointsByteStride, const uint64_t triFinder);
const uint32_t* getIndices(uint32_t& indicesSize, const uint64_t triFinder);
void getAdjacency(uint32_t* vertTriCounts,
                  uint32_t*& vertTriIds,
                  uint32_t& vertTriIdsSize,
                  const uint64_t triFinder,
                  const uint32_t* vertIds,
                  const uint32_t vertIdsSize,
                  void* (*allocateBytes)(size_t));
} // namespace trifinder
} // namespace omni
