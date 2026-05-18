// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Types.h>

namespace omni
{
namespace physx
{

void simplifyTriangleMeshInternal(carb::Float3*& dstPoints,
                                  uint32_t& dstPointsSize,
                                  uint32_t*& dstIndices,
                                  uint32_t& dstIndicesSize,
                                  uint32_t** dstOrigToSimpVertexMap,
                                  uint32_t* dstOrigToSimpVertexMapSize,
                                  uint32_t** dstSimpToOrigTriMap,
                                  uint32_t* dstSimpToOrigTriMapSize,
                                  const carb::Float3* srcPoints,
                                  const uint32_t srcPointsSize,
                                  const uint32_t* srcIndices,
                                  const uint32_t srcIndicesSize,
                                  const uint32_t targetTriangleCount,
                                  const float maximalEdgeLength,
                                  const bool projectSimplifiedVerticesOntoInputMesh,
                                  const bool removeDisconnectedPatches,
                                  void* (*allocateBytes)(size_t));

bool computeConformingTetrahedralMeshInternal(carb::Float3*& dstTetPoints,
                                              uint32_t& dstTetPointsSize,
                                              uint32_t*& dstTetIndices,
                                              uint32_t& dstTetIndicesSize,
                                              const carb::Float3* srcTriPoints,
                                              const uint32_t srcTriPointsSize,
                                              const uint32_t* srcTriIndices,
                                              const uint32_t srcTriIndicesSize,
                                              void* (*allocateBytes)(size_t));


bool computeTreeBasedTetrahedralMeshInternal(carb::Float3*& dstPoints,
                                             uint32_t& dstPointsSize,
                                             uint32_t*& dstIndices,
                                             uint32_t& dstIndicesSize,
                                             const carb::Float3* srcPoints,
                                             const uint32_t srcPointsSize,
                                             const uint32_t* srcIndices,
                                             const uint32_t srcIndicesSize,
                                             const bool useTreeNodes,
                                             void* (*allocateBytes)(size_t));

/**
 * Associate points with closest tetrahedra
 * @param dstPointsToTetIndices : Pointer to output tet indices, preallocated memory needs to match srcPointsSize, but
 * can be nullptr if not needed.
 * @param dstBarycentricCoordinates : Pointer to output barycentric coordinates, preallocated memory needs to match
 * srcPointsSize, but can be nullptr if not needed.
 * @param srcPoints : Points to be mapped to tet mesh tetrahedra, allowed to alias with srcTetPoints
 * @param srcPointsSize : Number of points to be mapped to tet mesh tetrahedra
 * @param srcTetPoints : Vertices of tet mesh
 * @param srcTetPointsSize : Number of tet mesh vertices
 * @param srcTetIndices : Tet mesh indices
 * @param srcTetIndicesSize : Number of tet mesh indices (4 * number of tetrahedra)
 */
void createPointsToTetrahedraMapInternal(uint32_t* dstPointsToTetIndices,
                                         carb::Float3* dstBarycentricCoordinates,
                                         const carb::Float3* srcPoints,
                                         const uint32_t srcPointsSize,
                                         const carb::Float3* srcTetPoints,
                                         const uint32_t srcTetPointsSize,
                                         const uint32_t* srcTetIndices,
                                         const uint32_t srcTetIndicesSize);

/**
 * Relax partially-kinematic deformable model to kinematic target pose
 * @param dstTetPoints : Pointer to output tet points, preallocated memory needs to match srcTetPointsSize. Allowed to
 * alias with srcTetPoints.
 * @param srcTetPoints : Original points of tet mesh.
 * @param srcTetPointsSize : Number of tet mesh points
 * @param srcKinematicTargetPoints : Kinematic target points, assumed to correspond to first
 * srcKinematicTargetPointsSize of srcTetPoints
 * @param srcKinematicTargetPointsSize : Number of target points, needs to be smaller than srcTetPointsSize
 * @param srcTetIndices : Indices of tet mesh
 * @param srcTetIndicesSize : Number of tet mesh indices (4 * number of tetrahedra)
 */
void transformTetrahedralMeshPoseInternal(carb::Float3* dstTetPoints,
                                          const carb::Float3* srcTetPoints,
                                          const uint32_t srcTetPointsSize,
                                          const carb::Float3* srcKinematicTargetPoints,
                                          const uint32_t srcKinematicTargetPointsSize,
                                          const uint32_t* srcTetIndices,
                                          const uint32_t srcTetIndicesSize);


bool computeVoxelTetrahedralMeshInternal(carb::Float3*& dstTetPoints,
                                         uint32_t& dstTetPointsSize,
                                         uint32_t*& dstTetIndices,
                                         uint32_t& dstTetIndicesSize,
                                         int32_t*& dstEmbedding,
                                         uint32_t& dstEmbeddingSize,
                                         const carb::Float3* srcTetPoints,
                                         const uint32_t srcTetPointsSize,
                                         const uint32_t* srcTetIndices,
                                         const uint32_t srcTetIndicesSize,
                                         const uint32_t voxelResolution,
                                         const uint32_t numTetsPerVoxel,
                                         const uint32_t* anchorNodes,
                                         void* (*allocateBytes)(size_t));


void remeshTriangleMeshInternal(carb::Float3*& dstPoints,
                                uint32_t& dstPointsSize,
                                uint32_t*& dstIndices,
                                uint32_t& dstIndicesSize,
                                uint32_t** dstOrigToRemeshedVertexMap,
                                uint32_t* dstOrigToRemeshedVertexMapSize,
                                const carb::Float3* srcPoints,
                                const uint32_t srcPointsSize,
                                const uint32_t* srcIndices,
                                const uint32_t srcIndicesSize,
                                const uint32_t remeshingVoxelResolution,
                                void* (*allocateBytes)(size_t));

} // namespace physx

} // namespace omni
