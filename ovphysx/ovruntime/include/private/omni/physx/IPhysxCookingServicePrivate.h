// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#include <omni/physx/IPhysxCookingService.h>
#include "PhysxUsd.h"

namespace physx
{
class PxCudaContextManager;
}
namespace omni
{
namespace physx
{

/// Input triangle mesh view (read only) for generating tetrahedral mesh. See
/// IPhysxCookingServicePrivate::computeConformingTetrahedralMesh,
/// IPhysxCookingServicePrivate::computeVoxelTetrahedralMesh.
struct PhysxCookingTetrahedralMeshInput
{
    const carb::Float3* srcTriPoints = nullptr;
    uint32_t srcTriPointsSize = 0;
    const uint32_t* srcTriIndices = nullptr;
    uint32_t srcTriIndicesSize = 0;
};

/// Parameters for generating Voxel tetrahedral mesh. See IPhysxCookingServicePrivate::computeVoxelTetrahedralMesh.
struct PhysxCookingTetrahedralVoxelMeshParameters
{
    uint32_t voxelResolution = 0;
    const uint32_t* anchorNodes = nullptr;
    uint32_t numTetsPerVoxel = 6;
};

/// Output tetrahedral mesh. See
/// IPhysxCookingServicePrivate::computeConformingTetrahedralMesh,
/// IPhysxCookingServicePrivate::computeVoxelTetrahedralMesh.
struct PhysxCookingTetrahedralMeshOutput
{
    void* (*allocateBytes)(size_t) = nullptr;
    carb::Float3* dstTetPoints = nullptr;
    uint32_t dstTetPointsSize = 0;
    uint32_t* dstTetIndices = nullptr;
    uint32_t dstTetIndicesSize = 0;
    int32_t* dstEmbedding = nullptr;
    uint32_t dstEmbeddingSize = 0;
};

/***
 * Particle poisson sampling cooking result data
 **/
struct PhysxCookingParticlePoissonSamplingData
{
    const carb::Float3* positions;
    uint32_t positionsSize;
};

/***
 * Particle poisson sampling cooking parameter
 **/
struct ParticlePoissonSamplingCookingParams
{
    carb::Double3 shearScale[3];
    float samplingDistance;
    uint32_t maxSamples;
    bool sampleVolume;
};

struct PhysxCookingVolumeDeformableBodyData
{
    const carb::Float3* simPoints;
    uint32_t simPointsSize;

    const uint32_t* simIndices;
    uint32_t simIndicesSize;

    // 1 -> plain tet mesh; 5 or 6 -> hex elements.
    uint32_t numTetsPerElement;

    const carb::Float3* collPoints;
    uint32_t collPointsSize;

    const uint32_t* collIndices;
    uint32_t collIndicesSize;

    const uint32_t* collSurfaceIndices;
    uint32_t collSurfaceIndicesSize;
};

struct PhysxCookingSurfaceDeformableBodyData
{
    const carb::Float3* simPoints;
    uint32_t simPointsSize;

    const uint32_t* simIndices;
    uint32_t simIndicesSize;
};

struct DeformableVolumeMeshCookingParams
{
    omni::span<const carb::Float3> simPoints;
    omni::span<const carb::Float3> simBindPoints; // empty if sim == coll
    omni::span<const carb::Int4> simIndices;
    omni::span<const carb::Float3> collBindPointsInSim; // empty if sim == coll
    omni::span<const carb::Int4> collIndices; // empty if sim == coll
    omni::span<const carb::Int3> collSurfaceIndices;
    carb::Double4 simToCookingTransform[4];

    // 1 -> plain tet mesh; 5 or 6 -> hex elements.
    uint32_t numTetsPerElement;
};

/// Volume deformable body cooking params
struct VolumeDeformableBodyCookingParams
{
    omni::span<const carb::Float3> srcPointsInSim; // in sim space
    carb::Double4 simToCookingTransform[4];
    carb::Double4 simToCollTransform[4];

    // mesh generation parameter
    bool isAutoMeshSimplificationEnabled;
    bool isAutoRemeshingEnabled;
    bool hasAutoForceConforming;
    bool isAutoHexahedralMeshEnabled;
    uint32_t autoRemeshingResolution;
    uint32_t autoTriangleTargetCount;
    uint32_t autoHexahedralResolution;
};

struct SurfaceDeformableBodyCookingParams
{
    omni::span<const carb::Float3> srcPointsInSim; // in sim space
    carb::Double4 simToCookingTransform[4];

    // mesh generation parameter
    bool isAutoMeshSimplificationEnabled;
    bool isAutoRemeshingEnabled;
    uint32_t autoRemeshingResolution;
    uint32_t autoTriangleTargetCount;
};

// These constants represent the current 'version number' for different types of data.
// If the underlying binary format of the cooked data changes we increase the version number which
// will, in turn, invalidate the old data and cause the new version of the data to be recooked.

// clang-format off
constexpr static int PhysxCookingDataVersion_MeshTriangulation = 5; // triangulation of the UsdGeomPrim source content
constexpr static int PhysxCookingDataVersion_TriangleMesh = 7;
constexpr static int PhysxCookingDataVersion_TriangleMeshSDF = 1; // signed distance fields
constexpr static int PhysxCookingDataVersion_ConvexMesh = 9;
constexpr static int PhysxCookingDataVersion_ConvexDecomposition = 9;
constexpr static int PhysxCookingDataVersion_ParticlePoissonSampling = 2;
constexpr static int PhysxCookingDataVersion_SphereFill = 2;
constexpr static int PhysxCookingDataVersion_DeformableVolumeMesh = 5;
constexpr static int PhysxCookingDataVersion_VolumeDeformableBody = 3;
constexpr static int PhysxCookingDataVersion_SurfaceDeformableBody = 1;
// clang-format on

/// A private interface for physics extensions that need to be tightly coupled with omni.physx.
///
/// Subject to change without notice.
///
/// This interface should be considered internal to the omni.physx extension and
/// should not be used by external clients.  Clients should rely on public interfaces IPhysxCookingService
///
struct IPhysxCookingServicePrivate
{
    /// Mirrors of corresponding functions in IPhysxCooking
    uint32_t(CARB_ABI* getActiveTaskCount)(PhysxCookingAsyncContext context);
    uint32_t(CARB_ABI* getFinishedCookingTasksCount)();

    /// Cache handling. NOTE: All these 5 operations are NOT thread safe
    void(CARB_ABI* resetMeshCacheContents)();

    /// Compute tetrahedral mesh
    bool(CARB_ABI* computeConformingTetrahedralMesh)(const PhysxCookingTetrahedralMeshInput& meshInput,
                                                     PhysxCookingTetrahedralMeshOutput& meshOutput);

    /// Compute tetrahedral mesh with voxels
    bool(CARB_ABI* computeVoxelTetrahedralMesh)(const PhysxCookingTetrahedralMeshInput& meshInput,
                                                PhysxCookingTetrahedralMeshOutput& meshOutput,
                                                PhysxCookingTetrahedralVoxelMeshParameters parameters);

    /// Get cooked data for sdf mesh. Context must not be null if kComputeAsynchronously is set
    /// If kExecuteCookingOnGPU is set, a custom cudaContextManager can be supplied.
    /// If kExecuteCookingOnGPU is set and cudaContextManager == nullptr, a global context manager will be used.
    PhysxCookingOperationHandle(CARB_ABI* requestSdfMeshCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
        const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams,
        ::physx::PxCudaContextManager* cudaContextManager);

    /// Get cooked data for particle poisson sampling. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestParticlePoissonSamplingCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticlePoissonSamplingCookingParams& params);

    void(CARB_ABI* readParticlePoissonSamplingData)(PhysxCookingParticlePoissonSamplingData& out,
                                                    const PhysxCookedDataSpan& cookedData);

    // DEPRECATED
    /// Return true if process is running on an OVC node, false otherwise.
    ///
    bool(CARB_ABI* isOVCNodeDeprecated)();

    /// Get cooked data for volume meshes. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestDeformableVolumeMeshCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::DeformableVolumeMeshCookingParams& params);

    PhysxCookingOperationHandle(CARB_ABI* requestVolumeDeformableBodyCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::VolumeDeformableBodyCookingParams& params);

    PhysxCookingOperationHandle(CARB_ABI* requestSurfaceDeformableBodyCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SurfaceDeformableBodyCookingParams& params);

    // Read volume deformable body tet mesh data from binary
    // TODO: move to extensions/common (OM-121591)
    void(CARB_ABI* readVolumeDeformableBodyData)(PhysxCookingVolumeDeformableBodyData& out,
                                                 const PhysxCookedDataSpan& cookedData);

    // Read surface deformable body tet mesh data from binary
    // TODO: move to extensions/common (OM-121591)
    void(CARB_ABI* readSurfaceDeformableBodyData)(PhysxCookingSurfaceDeformableBodyData& out,
                                                  const PhysxCookedDataSpan& cookedData);

};
} // namespace physx

} // namespace omni
