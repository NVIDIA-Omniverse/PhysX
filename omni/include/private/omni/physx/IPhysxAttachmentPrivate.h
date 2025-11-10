// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

/// omni.physx attachment interface
struct IPhysxAttachmentPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxAttachmentPrivate", 0, 1)

    /// DEPRECATED:
    /// Compute attachment points and collision filters between deformable bodies, deformable surfaces, particle cloth,
    /// rigid bodies and colliders
    ///
    ///\param[in] attachmentPath Path to primitive of type PhysxSchemaPhysxPhysicsAttachment
    ///\return Whether the operation was successful
    bool(CARB_ABI* computeAttachmentPoints)(const pxr::SdfPath& attachmentPath);

    /// Creates surface sampler instance used for sampling attachment points on a collider surface
    ///
    ///\param[in] colliderPath Path to primitive with UsdPhysicsCollisionAPI
    ///\param[in] samplingDistance Distance at which sampling points should be created
    ///\return Handle to surface sampler instance
    uint64_t(CARB_ABI* createSurfaceSampler)(const pxr::SdfPath& colliderPath, float samplingDistance);

    /// Releases surface sampler instance
    ///
    ///\param[in] surfaceSampler Handle
    void(CARB_ABI* releaseSurfaceSampler)(const uint64_t surfaceSampler);

    /// Adds defined points to surface sampler that will be considered for further sampling
    ///
    /// This can be used for example to initialize a surface sampler with already sampled points
    ///
    ///\param[in] surfaceSampler Handle
    ///\param[in] points Points to be added
    ///\param[in] pointsSize Number of points to be added
    void(CARB_ABI* addSurfaceSamplerPoints)(const uint64_t surfaceSampler,
                                            const carb::Float3* points,
                                            const uint32_t pointsSize);

    /// Removes sample points from surface sampler
    ///
    /// Note that the floating point coordinates need to match exactly for successful removal.
    ///
    ///\param surfaceSampler Handle
    ///\param points Points which should be removed
    ///\param pointsSize Number of points to be removed
    void(CARB_ABI* removeSurfaceSamplerPoints)(const uint64_t surfaceSampler,
                                               const carb::Float3* points,
                                               const uint32_t pointsSize);

    /// Creates new samples on the collider surface within a specified sphere
    ///
    /// A different sampling distance can be specified to the distance specified at surface sampler creation.
    /// However, changing the sampling distance will incur a performance overhead.
    ///
    ///\param[out] points New points sampled within sphere
    ///\param[out] pointsSize Number of new samples generated
    ///\param[in] surfaceSampler Handle
    ///\param[in] sphereCenter Center of the sphere within which the samples are generated
    ///\param[in] sphereRadius Radius of the sphere within which the samples are generated
    ///\param[in] samplingDistance Distance at which samples are generated
    ///\param[in] allocateBytes Function to allocate output memory
    void(CARB_ABI* sampleSurface)(carb::Float3*& points,
                                  uint32_t& pointsSize,
                                  const uint64_t surfaceSampler,
                                  const carb::Float3& sphereCenter,
                                  const float sphereRadius,
                                  const float samplingDistance,
                                  void* (*allocateBytes)(size_t));

    /// Gets all sampled points
    ///
    ///\param[out] points All sampled points
    ///\param[out] pointsSize Number of all points
    ///\param[in] surfaceSampler Handle
    ///\param[in] allocateBytes Function to allocate output memory
    void(CARB_ABI* getSurfaceSamplerPoints)(carb::Float3*& points,
                                            uint32_t& pointsSize,
                                            const uint64_t surfaceSampler,
                                            void* (*allocateBytes)(size_t));

    /// Creates a tet finder instance
    ///
    ///\param[in] points Points representing the tetrahedra vertex positions
    ///\param[in] pointsSize Number of points
    ///\param[in] indices Four tuples representing the topology of the tetrahedra
    ///\param[in] indicesSize Number of indices (4 x Number of four-tuples)
    ///\param[in] bMakeCopy Specifies whether input data is copied or merely referenced
    ///\return Handle to tet finder instance
    uint64_t(CARB_ABI* createTetFinder)(const carb::Float3* points,
                                        const uint32_t pointsSize,
                                        const uint32_t* indices,
                                        const uint32_t indicesSize);

    /// Releases a tet finder instance
    ///
    ///\param[in] tetFinder Handle
    void(CARB_ABI* releaseTetFinder)(const uint64_t tetFinder);

    /// Maps each point given in euclidean space to tetrahedron ID (four-tuple index) and barycentric coordinates
    ///
    ///\param[out] tetIds Resulting tetrahedron IDs (-1 if not contained in any tetrahedra)
    ///\param[out] barycentricCoords Resulting barycentric coordinates
    ///\param[in] tetFinder Handle
    ///\param[in] points Points to be mapped
    ///\param[in] pointsSize Number of points to be mapped, and output size
    void(CARB_ABI* pointsToTetMeshLocal)(int32_t* tetIds,
                                         carb::Float4* barycentricCoords,
                                         const uint64_t tetFinder,
                                         const carb::Float3* points,
                                         const uint32_t pointsSize);

    /// Maps each point specified as tetrahedron ID and barycentric coordinates to euclidean space
    ///
    ///\param[out] points Resulting points
    ///\param[in] tetFinder Handle
    ///\param[in] tetIds Tetrahedron IDs (four-tuple index)
    ///\param[in] barycentricCoords Barycentric coordinates of input points
    ///\param[in] pointsSize Number of points to be mapped, and output size
    void(CARB_ABI* tetMeshLocalToPoints)(carb::Float3* points,
                                         const uint64_t tetFinder,
                                         const int32_t* tetIds,
                                         const carb::Float4* barycentricCoords,
                                         const uint32_t pointsSize);

    /// Finds all tetrahedra overlapping with specified sphere
    ///
    ///\param[out] tetIds Tetrahedral overlapping with sphere (four-tuple index)
    ///\param[out] tetIdsSize Number of overlapping tetrahedra
    ///\param[in] tetFinder Handle
    ///\param[in] center Center of sphere
    ///\param[in] radius Radius of sphere
    ///\param[in] allocateBytes Function to allocate output memory
    void(CARB_ABI* overlapTetMeshSphere)(int32_t*& tetIds,
                                         uint32_t& tetIdsSize,
                                         const uint64_t tetFinder,
                                         const carb::Float3& center,
                                         const float radius,
                                         void* (*allocateBytes)(size_t));

    /// Finds all tetrahedra overlapping with specified capsule
    ///
    ///\param[out] tetIds Tetrahedral overlapping with capsule (four-tuple index)
    ///\param[out] tetIdsSize Number of overlapping tetrahedra
    ///\param[in] tetFinder Handle
    ///\param[in] pos Position of capsule
    ///\param[in] axis Orientation of capsule
    ///\param[in] radius Radius of capsule
    ///\param[in] halfHeight Half height of capsule
    ///\param[in] allocateBytes Function to allocate output memory
    void(CARB_ABI* overlapTetMeshCapsule)(int32_t*& tetIds,
                                          uint32_t& tetIdsSize,
                                          const uint64_t tetFinder,
                                          const carb::Float3& pos,
                                          const carb::Float3& axis,
                                          const float radius,
                                          const float halfHeight,
                                          void* (*allocateBytes)(size_t));

    /// Creates a point finder instance representing an index to efficiently find indices of points
    ///
    ///\param[in] points Points from which the point finder is created
    ///\param[in] pointsSize Number of points
    ///\return Point finder handle
    uint64_t(CARB_ABI* createPointFinder)(const carb::Float3* points, const uint32_t pointsSize);

    /// Releases a point finder instance
    ///
    ///\param[in] pointFinder Handle
    void(CARB_ABI* releasePointFinder)(const uint64_t pointFinder);

    /// Maps points to point indices
    ///
    /// Note that the floating point coordinates need to match exactly with the points
    /// for which the point finder was created in order to map successfully.
    ///
    ///\param[out] pointIndices Mapped point indices (-1 if point is not represented in point finder)
    ///\param[in] pointFinder Handle
    ///\param[in] points Points to be mapped
    ///\param[in] pointsSize Number of points to be mapped
    void(CARB_ABI* pointsToIndices)(int32_t* pointIndices,
                                    const uint64_t pointFinder,
                                    const carb::Float3* points,
                                    const uint32_t pointsSize);

    /// Gets all closest points to the input points on the prim
    ///
    ///\param[out] closestPoints Closest points to the input points on the prim. Only valid when returned distance is
    /// strictly positive. \param[out] dists Square distances between the points and the geom object. 0.0 if the point
    /// is inside the object. \param[in] points Input points \param[in] pointsSize Number of input points \param[in]
    /// rigidPath Rigid prim path
    void(CARB_ABI* getClosestPoints)(carb::Float3* closestPoints,
                                     float* dists,
                                     const carb::Float3* points,
                                     const uint32_t pointsSize,
                                     const pxr::SdfPath& rigidPath);

    /// Creates triangle mesh sampler instance
    ///
    ///\param[in] surfaceSampler Handle
    void(CARB_ABI* createTriMeshSampler)(const uint64_t surfaceSampler);

    /// Test whether a point is inside a triangle mesh
    ///
    ///\param[in] surfaceSampler Handle
    ///\param[in] point Input point
    ///\return True if the point is inside the triangle mesh
    bool(CARB_ABI* isPointInside)(const uint64_t surfaceSampler, const carb::Float3 point);

    /// Setup all attachments and filters for a given auto attachment path
    ///
    ///\param[in] attachmentPath Path to primitive with PhysxSchemaPhysxAutoDeformableAttachmentAPI
    ///\return Whether the operation was successful
    bool(CARB_ABI* setupAutoDeformableAttachment)(const pxr::SdfPath& attachmentPath);

    /// Update all attachments and filters for a given auto attachment path
    ///
    ///\param[in] attachmentPath Path to primitive with PhysxSchemaPhysxAutoDeformableAttachmentAPI
    ///\return Whether the operation was successful
    bool(CARB_ABI* updateAutoDeformableAttachment)(const pxr::SdfPath& attachmentPath);
};

} // namespace physx
} // namespace omni
