// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/IPhysxCookingServicePrivate.h>

#include <carb/Types.h>

#include <cstdint>
#include <vector>

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

/**
 * Hands the cooking service the CUDA context manager owned by the host (omni.physx).
 *
 * The returned manager comes with a reference **already taken** on behalf of the caller, or is
 * null when the host has none. This is deliberately an acquire and not a borrow: the provider
 * releases its manager from the main thread while cooking runs on UJITSO/carb::tasking workers,
 * so a plain getter leaves the consumer no race-free moment in which to call acquireReference()
 * itself - the manager can be destroyed between the read and the acquire. The provider must
 * therefore take the reference under whatever lock guards its own release path.
 *
 * Every consumer that stores the returned pointer owns that reference and must release() it.
 *
 * @implements REQ-COOK-CUDACTX-001
 * @covers AC-1
 */
using AcquireSharedCudaContextManagerFn = ::physx::PxCudaContextManager* (*)();

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

    /**
     * Resolves the CUDA context manager to use for a GPU cooking request and hands the caller a
     * reference to it.
     *
     * On success and when @p cudaContextManager comes back non-null, the caller **owns a
     * reference** and must release() it once it no longer needs the manager - typically right
     * after passing it to CookingTask::setPxCudaAndGPUPointers(), which takes its own.
     *
     * On entry a non-null @p cudaContextManager means "use this caller-supplied manager"; the
     * caller must keep it alive for the duration of the call and this function takes its own
     * reference on it. Pass null to let the service resolve one.
     *
     * @p cudaContextManager is set to null (and no reference is owed) whenever the request does
     * not ask for GPU execution, or on a build/machine without GPU support.
     *
     * @implements REQ-COOK-CUDACTX-001
     * @covers AC-2
     */
    virtual bool acquireCudaContextManager(PhysxCookingDataType::Enum dataType,
                                           const PhysxCookingComputeRequest& request,
                                           ::physx::PxCudaContextManager*& cudaContextManager,
                                           ::physx::PxPhysicsGpu*& physicsGPU) = 0;

    static bool computeMeshKeyIfNeeded(PhysxCookingComputeResult& result, PhysxCookingComputeRequest& request);
    static uint16_t getMaxMaterialIndex(const PhysxCookingMeshView& meshView);
};

ICookingComputeService* createCookingComputingService(
    ::physx::PxFoundation& foundation,
    AcquireSharedCudaContextManagerFn acquireSharedCudaContextManagerFn = nullptr);
ICookingComputeService* createDefaultCookingComputingService(
    ::physx::PxFoundation& foundation,
    AcquireSharedCudaContextManagerFn acquireSharedCudaContextManagerFn = nullptr);
void releaseCookingComputingService(ICookingComputeService*);

} // namespace physx
} // namespace omni
