// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/IPhysxCookingServicePrivate.h>
namespace physx
{
class PxFoundation;
class PxCudaContextManager;
class PxPhysicsGpu;
class PxDefaultMemoryOutputStream;
} // namespace physx
namespace cookedcache
{
class CookedCache;
}
namespace cookingtask
{
class CookingTask;
}

namespace omni
{
namespace physx
{

using SharedCudaContextManagerFn = ::physx::PxCudaContextManager* (*)();

struct CookingInputUSDRigidMesh
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> pointsValue;
    PXR_NS::VtArray<int> indicesValue;
    PXR_NS::VtArray<int> facesValue;
    PXR_NS::VtArray<int> holesValue;
    std::vector<uint16_t> faceMaterials;
};

struct CookingInputUSDDeformableBodyMesh
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> srcPointsInSim;
};

struct CookingInputUSDDeformableVolumeMesh
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> simPoints;
    PXR_NS::VtArray<PXR_NS::GfVec3f> simBindPoints;
    PXR_NS::VtArray<PXR_NS::GfVec4i> simIndices;
    PXR_NS::VtArray<PXR_NS::GfVec3f> collBindPointsInSim;
    PXR_NS::VtArray<PXR_NS::GfVec4i> collIndices;
    PXR_NS::VtArray<PXR_NS::GfVec3i> collSurfaceIndices;
};

struct CookingStageAndPrim
{
    CookingInputUSDRigidMesh rigidMesh;
    CookingInputUSDDeformableBodyMesh volumeDeformableBodyMesh;
    CookingInputUSDDeformableBodyMesh surfaceDeformableBodyMesh;
    CookingInputUSDDeformableVolumeMesh deformableVolumeMesh;
    PXR_NS::UsdStageRefPtr stage;
    PXR_NS::UsdPrim usdPrim;
};

struct ICookingComputeService
{
    virtual ~ICookingComputeService()
    {
    }
    virtual uint32_t pumpAsyncContext(PhysxCookingAsyncContext context) = 0;

    virtual PhysxCookingOperationHandle requestTriangleMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
        const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams,
        ::physx::PxCudaContextManager* cudaContextManager) = 0;

    virtual PhysxCookingOperationHandle requestConvexMeshCookedData(PhysxCookingAsyncContext context,
                                                                    const PhysxCookingComputeRequest& request,
                                                                    const omni::physx::ConvexMeshCookingParams& desc) = 0;

    virtual PhysxCookingOperationHandle requestConvexMeshDecompositionCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ConvexDecompositionCookingParams& desc) = 0;

    virtual PhysxCookingOperationHandle requestSphereFillCookedData(PhysxCookingAsyncContext context,
                                                                    const PhysxCookingComputeRequest& request,
                                                                    const omni::physx::SphereFillCookingParams& desc) = 0;

    virtual PhysxCookingOperationHandle requestDeformableVolumeMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::DeformableVolumeMeshCookingParams& params) = 0;

    virtual PhysxCookingOperationHandle requestVolumeDeformableBodyCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::VolumeDeformableBodyCookingParams& params) = 0;

    virtual PhysxCookingOperationHandle requestSurfaceDeformableBodyCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SurfaceDeformableBodyCookingParams& params) = 0;

    virtual PhysxCookingOperationHandle requestParticlePoissonSamplingCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticlePoissonSamplingCookingParams& params) = 0;

    virtual PhysxCookingAsyncContext createAsyncContext(PhysxCookingAsyncContextParameters& parameters) = 0;

    virtual void destroyAsyncContext(PhysxCookingAsyncContext context) = 0;

    // These functions may or may not be part of the interface, or may go to a private interface (to be decided)

    virtual void release(void) = 0;

    virtual uint32_t getActiveTaskCount(PhysxCookingAsyncContext context) = 0;

    virtual uint32_t getFinishedCookingTasksCount() const = 0;

    virtual uint32_t cancelAllTasks(PhysxCookingAsyncContext context) = 0;

    virtual bool cancelTask(PhysxCookingOperationHandle handle, bool invokeCallbackAnyway) = 0;

    virtual bool waitForTaskToFinish(PhysxCookingOperationHandle handle, int64_t timeoutMs) = 0;


    virtual void resetMeshCacheContents() = 0;

    virtual bool lazyGetCudaContextManager(PhysxCookingDataType::Enum dataType,
                                           const PhysxCookingComputeRequest& request,
                                           ::physx::PxCudaContextManager*& customContextManager,
                                           ::physx::PxPhysicsGpu*& physicsGPU) = 0;

    static bool getStageAndPrim(PhysxCookingComputeResult& result,
                                PhysxCookingComputeRequest& request,
                                CookingStageAndPrim& stageAndPrim);
    static bool fillMeshView(PhysxCookingComputeResult& result,
                             PhysxCookingComputeRequest& request,
                             CookingStageAndPrim& stageAndPrim);
    static bool computeMeshKeyIfNeeded(PhysxCookingComputeResult& result,
                                       PhysxCookingComputeRequest& request,
                                       CookingStageAndPrim& stageAndPrim);
    static uint16_t getMaxMaterialIndex(const PhysxCookingMeshView& meshView);
};

ICookingComputeService* createCookingComputingService(
    ::physx::PxFoundation& foundation,
    SharedCudaContextManagerFn sharedCudaContextManagerFn = nullptr);
ICookingComputeService* createDefaultCookingComputingService(
    ::physx::PxFoundation& foundation,
    SharedCudaContextManagerFn sharedCudaContextManagerFn = nullptr);
void releaseCookingComputingService(ICookingComputeService*);

} // namespace physx
} // namespace omni
