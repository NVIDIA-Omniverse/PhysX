// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
 * Deformable tetmesh cooking result data
 **/
struct PhysxCookingDeformableBodyTetMeshDataDeprecated
{
    const carb::Float3* collisionRestPoints;
    uint32_t collisionRestPointsSize;

    const uint32_t* collisionIndices;
    uint32_t collisionIndicesSize;

    const carb::Float3* simulationRestPoints;
    uint32_t simulationRestPointsSize;

    const uint32_t* simulationIndices;
    uint32_t simulationIndicesSize;

    const int32_t* collisionVertexToSimulationTetIndices;
    uint32_t collisionVertexToSimulationTetIndicesSize;

    const uint32_t* collisionVertexToSkinTriVertexIndices;
    const carb::Float3* collisionVertexToSkinTriBarycentrics;
    uint32_t collisionVertexToSkinTriSize;
};

/***
 * Deformable tet mesh cooking parameter
 **/
struct DeformableBodyTetMeshCookingParamsDeprecated
{
    /***
     * Mesh simplification cooking parameter
     **/
    struct CollisionSimplificationParams
    {
        bool enabled = false;
        bool remeshing = true;
        uint32_t remeshingResolution = 0;
        uint32_t targetTriangleCount = 0;
        bool forceConforming = false;
    };

    omni::span<const carb::Float3> skinRestPoints;
    omni::span<const carb::Float3> skinEarliestPoints;
    carb::Float3 normalizedQuantizedScale;
    CollisionSimplificationParams simpParams;
    uint32_t simulationResolution = 0;
    uint32_t simulationNumTetsPerVoxel = 6;
    bool kinematicEnabled = false;
};

/***
 * Soft body mesh cooking parameter
 **/
struct SoftBodyMeshCookingParamsDeprecated
{
    omni::span<const carb::Float3> collisionRestPoints;
    omni::span<const uint32_t> collisionIndices;
    omni::span<const carb::Float3> simulationRestPoints;
    omni::span<const uint32_t> simulationIndices;
    omni::span<const uint32_t> collisionVertexToSimulationTetIndices;
    omni::span<const carb::Float3> restPoints;
    carb::Float3 softBodyToWorldScaleNormalized;
    uint32_t simulationResolution = 0;
    uint32_t simulationNumTetsPerVoxel = 6;
    bool kinematicEnabled = false;
};

/***
 * Particle cloth cooking result data
 **/
struct PhysxCookingParticleClothMeshDataDeprecated
{
    const carb::Int2* springIndices;
    const float* springStiffnesses;
    const float* springDampings;
    const float* springRestLengths;
    uint32_t springIndicesSize;

    const uint32_t* weldedTriangleIndices;
    uint32_t weldedTriangleIndicesSize;

    const uint32_t* verticesRemapToWeld;
    uint32_t verticesRemapToWeldSize;

    const uint32_t* verticesRemapToOrig;
    uint32_t verticesRemapToOrigSize;

    float inflatableVolume;
};

/***
 * Particle cloth cooking parameter
 **/
struct ParticleClothMeshCookingParamsDeprecated
{
    omni::span <const carb::Float3> restPoints;
    bool needsSprings = false;
    bool isInflatable = false;
    bool enableWelding = false;
    float springStretchStiffness = 0.0f;
    float springBendStiffness = 0.0f;
    float springShearStiffness = 0.0f;
    float springDamping = 0.0f;
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
};

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
constexpr static int PhysxCookingDataVersion_MeshTriangulation = 5; // version number for source triangle mesh data (triangulation of the UsdGeomPrim which acts as the source content for the cooking operation)
constexpr static int PhysxCookingDataVersion_TriangleMesh = 7; // The current version number of cooked data for triangle meshes
constexpr static int PhysxCookingDataVersion_TriangleMeshSDF = 1; // The current version number of cooked data for signed distance fields
constexpr static int PhysxCookingDataVersion_ConvexMesh = 9; // The current version number of cooked data for convex meshes
constexpr static int PhysxCookingDataVersion_ConvexDecomposition = 9; // The current version number of cooked data for a convex decomposition
constexpr static int PhysxCookingDataVersion_SoftBodyMeshDeprecated = 17; // The current version number of cooked data for softbody meshes
constexpr static int PhysxCookingDataVersion_DeformableBodyTetMeshDeprecated = 8; // The current version number of cooked data for deformable body meshes
constexpr static int PhysxCookingDataVersion_ParticleClothDeprecated = 9; // The current version number of cooked data for particle cloth
constexpr static int PhysxCookingDataVersion_ParticlePoissonSampling = 2; // The current version number of cooked data for particle Poisson Sampling
constexpr static int PhysxCookingDataVersion_SphereFill = 2; // The current version number of cooked data for a sphere fill operation
constexpr static int PhysxCookingDataVersion_DeformableVolumeMesh = 3; // The current version number of cooked data for deformable volume meshes
constexpr static int PhysxCookingDataVersion_VolumeDeformableBody = 2; // The current version number of cooked data for volume deformable body meshes
constexpr static int PhysxCookingDataVersion_SurfaceDeformableBody = 1; // The current version number of cooked data for surface deformable body meshes
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
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCookingServicePrivate", 13, 0)

    /// Mirrors of corresponding functions in IPhysxCooking
    uint32_t(CARB_ABI* getActiveTaskCount)(PhysxCookingAsyncContext context);
    uint32_t(CARB_ABI* getFinishedCookingTasksCount)();

    /// Local cache handling. NOTE: All these 5 operations are NOT thread safe
    bool(CARB_ABI* isLocalMeshCacheEnabled)();
    void(CARB_ABI* setLocalMeshCacheEnabled)(bool val);
    uint32_t(CARB_ABI* getLocalMeshCacheSize)();
    void(CARB_ABI* setLocalMeshCacheSize)(uint32_t val);
    void(CARB_ABI* resetLocalMeshCacheContents)();

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

    /// Get cooked data for soft body. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestSoftBodyMeshCookedDataDeprecated)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SoftBodyMeshCookingParamsDeprecated& params);

    /// Get cooked data for deformable, passing a flag to know if it's being simulated in the runtime
    PhysxCookingOperationHandle(CARB_ABI* requestDeformableBodyTetMeshCookedDataDeprecated)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params);

    /// Get cooked data for particle cloth, passing a flag to know if it's being simulated in the runtime
    PhysxCookingOperationHandle(CARB_ABI* requestParticleClothMeshCookedDataDeprecated)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticleClothMeshCookingParamsDeprecated& params);

    /// Get cooked data for particle poisson sampling. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestParticlePoissonSamplingCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticlePoissonSamplingCookingParams& params);

    // DEPRECATED
    /// Return true if process is running on an OVC node, false otherwise.
    ///
    bool(CARB_ABI* isOVCNodeDeprecated)();

    // Read deformable body tet mesh data from binary
    // TODO: move to extensions/common (OM-121591)
    void(CARB_ABI* readDeformableBodyTetMeshDataDeprecated)(PhysxCookingDeformableBodyTetMeshDataDeprecated& out,
                                                            const PhysxCookedDataSpan& cookedData);

    // Read particle cloth data from binary
    // TODO: move to extensions/common (OM-121591)
    void(CARB_ABI* readParticleClothMeshDataDeprecated)(PhysxCookingParticleClothMeshDataDeprecated& out,
                                                        const PhysxCookedDataSpan& cookedData);

    // Read particle cloth data from binary
    // TODO: move to extensions/common (OM-121591)
    void(CARB_ABI* readParticlePoissonSamplingData)(PhysxCookingParticlePoissonSamplingData& out,
                                                    const PhysxCookedDataSpan& cookedData);

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
