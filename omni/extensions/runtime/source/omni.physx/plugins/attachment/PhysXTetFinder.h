// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <internal/Internal.h>
#include "foundation/PxTransform.h"

namespace physx
{
class PxGeometry;
}

namespace omni
{
namespace tetfinder
{

uint64_t createTetFinder(const carb::Float3* points,
                         const uint32_t pointsSize,
                         const uint32_t pointsByteStride,
                         const uint32_t* indices,
                         const uint32_t indicesSize);
uint64_t createTetFinder(const carb::Float3* points,
                         const uint32_t pointsSize,
                         const uint32_t* indices,
                         const uint32_t indicesSize);
uint64_t createTetFinder(const carb::Float4* points,
                         const uint32_t pointsSize,
                         const uint32_t* indices,
                         const uint32_t indicesSize);
void releaseTetFinder(const uint64_t tetFinder);
void pointsToTetMeshLocal(int32_t* tetIds,
                          carb::Float4* barycentricCoords,
                          const uint64_t tetFinder,
                          const carb::Float3* points,
                          const uint32_t pointsSize);
bool pointsToTetMeshLocalClosest(int32_t* tetIds,
                                 carb::Float4* barys,
                                 carb::Float3* distanceDirs,
                                 const uint64_t tetFinder,
                                 const carb::Float3* points,
                                 const uint32_t pointsSize);
bool pointsToTetMeshLocalAll(int32_t* tetIds,
                             carb::Float4* barys,
                             const uint64_t tetFinder,
                             const carb::Float3* points,
                             const uint32_t pointsSize);
void tetMeshLocalToPoints(carb::Float3* points,
                          const uint64_t tetFinder,
                          const int32_t* tetIds,
                          const carb::Float4* barycentricCoords,
                          const uint32_t pointsSize);
void overlapTetMeshGeom(int32_t*& tetIds,
                        uint32_t& tetIdsSize,
                        const uint64_t tetFinder,
                        const ::physx::PxGeometry& geom,
                        const ::physx::PxTransform& geomPos,
                        const float overlapOffset,
                        void* (*allocateBytes)(size_t),
                        void* triMeshSampler = nullptr);
void overlapTetMeshSphere(int32_t*& tetIds,
                          uint32_t& tetIdsSize,
                          const uint64_t tetFinder,
                          const carb::Float3& center,
                          const float radius,
                          void* (*allocateBytes)(size_t));
void overlapTetMeshCapsule(int32_t*& tetIds,
                           uint32_t& tetIdsSize,
                           const uint64_t tetFinder,
                           const carb::Float3& pos,
                           const carb::Float3& axis,
                           const float radius,
                           const float halfHeight,
                           void* (*allocateBytes)(size_t));
void overlapTetMeshTetMeshDeprecated(carb::Int2*& tetIdPairs,
                                     uint32_t& tetIdPairsSize,
                                     const uint64_t tetFinder0,
                                     const uint64_t tetFinder1,
                                     const float overlapOffset,
                                     const bool reportPairs,
                                     void* (*allocateBytes)(size_t));
void overlapTetMeshTriMeshDeprecated(carb::Int2*& tetTriIdPairs,
                                     uint32_t& tetTriIdPairsSize,
                                     const uint64_t tetFinder,
                                     const carb::Float3* triPoints,
                                     const uint32_t* triIndices,
                                     const uint32_t triIndicesSize,
                                     const float overlapOffset,
                                     const bool reportPairs,
                                     void* (*allocateBytes)(size_t));
void overlapTetMeshTetMeshPairs(carb::Int2*& tetIdPairs,
                                uint32_t& tetIdPairsSize,
                                const uint64_t tetFinder0,
                                const uint64_t tetFinder1,
                                const float overlapOffset,
                                void* (*allocateBytes)(size_t));
void overlapTetMeshTetMeshAny(int32_t*& tetIds0,
                              uint32_t& tetIds0Size,
                              int32_t*& tetIds1,
                              uint32_t& tetIds0Size1,
                              const uint64_t tetFinder0,
                              const uint64_t tetFinder1,
                              const float overlapOffset,
                              void* (*allocateBytes)(size_t));
void overlapTetMeshTriMeshPairs(carb::Int2*& tetTriIdPairs,
                                uint32_t& tetTriIdPairsSize,
                                const uint64_t tetFinder,
                                const carb::Float3* triPoints,
                                const uint32_t* triIndices,
                                const uint32_t triIndicesSize,
                                const float overlapOffset,
                                void* (*allocateBytes)(size_t));
void overlapTetMeshTriMeshAny(int32_t*& tetIds,
                              uint32_t& tetIdsSize,
                              int32_t*& triIds,
                              uint32_t& triIdsSize,
                              const uint64_t tetFinder,
                              const carb::Float3* triPoints,
                              const uint32_t* triIndices,
                              const uint32_t triIndicesSize,
                              const float overlapOffset,
                              void* (*allocateBytes)(size_t));
void getClosestPoints(carb::Float3* closestPoints,
                      float* dists,
                      const carb::Float3* points,
                      const uint32_t pointsSize,
                      const pxr::SdfPath& rigidPath);
const carb::Float3* getPoints(uint32_t& pointsSize, uint32_t& pointsByteStride, const uint64_t tetFinder);
const uint32_t* getIndices(uint32_t& indicesSize, const uint64_t tetFinder);

// functions that don't rely on TetFinder instance

bool tetMeshVtxToTet(uint32_t*& vtxToTetIndices,
                     uint32_t& vtxToTetIndicesSize,
                     uint32_t* vtxToTetCounts,
                     uint32_t* vtxToTetOffsets,
                     const uint32_t* tetVtxIndices,
                     const uint32_t numTets,
                     const uint32_t numVerts,
                     void* (*allocateBytes)(size_t));

bool tetMeshTetToSurfaceTri(uint32_t*& tetToTriIndices,
                            uint32_t& tetToTriIndicesSize,
                            uint32_t* tetToTriCounts,
                            uint32_t* tetToTriOffsets,
                            const uint32_t numTets,
                            const uint32_t* triToTetMap,
                            const uint32_t numTris,
                            void* (*allocateBytes)(size_t));

bool tetMeshVtxToTetLocal(uint32_t* tetIndices,
                          carb::Float4* tetBarycentrics,
                          const uint32_t* vtxIndices,
                          const uint32_t vtxIndicesSize,
                          const uint32_t* meshTetVtxIndices,
                          const uint32_t meshNumTets);


} // namespace tetfinder
} // namespace omni
