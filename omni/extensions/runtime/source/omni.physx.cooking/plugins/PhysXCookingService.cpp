// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include <omni/physx/IPhysxCookingService.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <omni/physx/IPhysxFoundation.h>

#include "service/CookingComputeService.h"
#include "service/CookingTask.h"
#include "ujitso/UjitsoCookingComputeService.h"
#include "utility/MeshSimplifyInternal.h"

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/tasking/ITasking.h>

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.cooking.plugin", "Physics", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxCookingServicePrivate, omni::physx::IPhysxCookingService)
CARB_PLUGIN_IMPL_DEPS(carb::tasking::ITasking, omni::physx::IPhysxFoundation)

omni::physx::ICookingComputeService* gCookingComputeService = nullptr;

CARB_EXPORT void carbOnPluginStartup()
{
    // always init the Ujitso cache system
    // it will check internally if it should be used or the fallback
    gCookingComputeService = omni::physx::createUjitsoCookingComputingService();
    if (!gCookingComputeService)
    {
        // Fall back if there's a problem with the ujitso cooking
        gCookingComputeService = omni::physx::createCookingComputingService();
    }
}

CARB_EXPORT void carbOnPluginShutdown()
{
    gCookingComputeService->release();
    gCookingComputeService = nullptr;
}

void fillInterface(omni::physx::IPhysxCookingService& iface)
{
    using namespace omni::physx;
    iface.pumpAsyncContext = [](PhysxCookingAsyncContext context) {
        return gCookingComputeService->pumpAsyncContext(context);
    };

    iface.cancelAllTasks = [](PhysxCookingAsyncContext context) {
        return gCookingComputeService->cancelAllTasks(context);
    };

    iface.cancelTask = [](PhysxCookingOperationHandle handle, bool invokeCallbackAnyway) {
        return gCookingComputeService->cancelTask(handle, invokeCallbackAnyway);
    };

    iface.waitForTaskToFinish = [](PhysxCookingOperationHandle handle, int64_t timeoutMs) {
        return gCookingComputeService->waitForTaskToFinish(handle, timeoutMs);
    };

    iface.requestTriangleMeshCookedData = [](PhysxCookingAsyncContext context, const PhysxCookingComputeRequest& request,
                                             const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams) {
        omni::physx::SdfMeshCookingParams sdfMeshCookingParams;
        sdfMeshCookingParams.sdfResolution = 0;
        return gCookingComputeService->requestTriangleMeshCookedData(
            context, request, triangleMeshCookingParams, sdfMeshCookingParams, nullptr);
    };


    iface.requestSdfMeshCookedData = [](PhysxCookingAsyncContext context, const PhysxCookingComputeRequest& request,
                                        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
                                        const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams) {
        return gCookingComputeService->requestTriangleMeshCookedData(
            context, request, triangleMeshCookingParams, sdfMeshCookingParams, nullptr);
    };

    iface.requestConvexMeshCookedData = [](PhysxCookingAsyncContext context, const PhysxCookingComputeRequest& request,
                                           const omni::physx::ConvexMeshCookingParams& desc) {
        return gCookingComputeService->requestConvexMeshCookedData(context, request, desc);
    };

    iface.requestConvexMeshDecompositionCookedData = [](PhysxCookingAsyncContext context,
                                                        const PhysxCookingComputeRequest& request,
                                                        const omni::physx::ConvexDecompositionCookingParams& desc) {
        return gCookingComputeService->requestConvexMeshDecompositionCookedData(context, request, desc);
    };

    iface.requestSphereFillCookedData = [](PhysxCookingAsyncContext context, const PhysxCookingComputeRequest& request,
                                           const omni::physx::SphereFillCookingParams& desc) {
        return gCookingComputeService->requestSphereFillCookedData(context, request, desc);
    };

    iface.createAsyncContext = [](PhysxCookingAsyncContextParameters& parameters) {
        return gCookingComputeService->createAsyncContext(parameters);
    };

    iface.destroyAsyncContext = [](PhysxCookingAsyncContext context) {
        gCookingComputeService->destroyAsyncContext(context);
    };
}

//DEPRECATED
static bool isOVCNode()
{
    CARB_LOG_WARN("isOVCNode is deprecated and will be removed in a future release");
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    return (settings && settings->getAsBool("/app/ovc_deployment"));
}

void fillInterface(omni::physx::IPhysxCookingServicePrivate& iface)
{
    using namespace omni::physx;
    iface.getActiveTaskCount = [](PhysxCookingAsyncContext context) {
        return gCookingComputeService->getActiveTaskCount(context);
    };

    iface.getFinishedCookingTasksCount = []() { return gCookingComputeService->getFinishedCookingTasksCount(); };

    iface.resetMeshCacheContents = []() { gCookingComputeService->resetMeshCacheContents(); };

    iface.computeConformingTetrahedralMesh = [](const PhysxCookingTetrahedralMeshInput& meshInput,
                                                PhysxCookingTetrahedralMeshOutput& meshOutput) {
        return omni::physx::computeConformingTetrahedralMeshInternal(
            meshOutput.dstTetPoints, meshOutput.dstTetPointsSize, // dest points
            meshOutput.dstTetIndices, meshOutput.dstTetIndicesSize, // dest indices
            meshInput.srcTriPoints, meshInput.srcTriPointsSize, // source points
            meshInput.srcTriIndices, meshInput.srcTriIndicesSize, // source indices
            meshOutput.allocateBytes);
    };

    iface.computeVoxelTetrahedralMesh = [](const PhysxCookingTetrahedralMeshInput& meshInput,
                                           PhysxCookingTetrahedralMeshOutput& meshOutput,
                                           PhysxCookingTetrahedralVoxelMeshParameters parameters) {
        return omni::physx::computeVoxelTetrahedralMeshInternal(
            meshOutput.dstTetPoints, meshOutput.dstTetPointsSize, // dest points
            meshOutput.dstTetIndices, meshOutput.dstTetIndicesSize, // dest indices
            meshOutput.dstEmbedding, meshOutput.dstEmbeddingSize, // dest embedding
            meshInput.srcTriPoints, meshInput.srcTriPointsSize, // source points
            meshInput.srcTriIndices, meshInput.srcTriIndicesSize, // source indices
            parameters.voxelResolution, parameters.numTetsPerVoxel, parameters.anchorNodes,
            meshOutput.allocateBytes); // parameters
    };

    iface.requestSdfMeshCookedData = [](PhysxCookingAsyncContext context, const PhysxCookingComputeRequest& request,
                                        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
                                        const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams,
                                        ::physx::PxCudaContextManager* pxCudaContextManager) {
        return gCookingComputeService->requestTriangleMeshCookedData(
            context, request, triangleMeshCookingParams, sdfMeshCookingParams, pxCudaContextManager);
    };

    iface.requestSoftBodyMeshCookedDataDeprecated = [](PhysxCookingAsyncContext context,
                                                       const PhysxCookingComputeRequest& request,
                                                       const omni::physx::SoftBodyMeshCookingParamsDeprecated& params) {
        return gCookingComputeService->requestSoftBodyMeshCookedDataDeprecated(context, request, params);
    };

    iface.requestDeformableBodyTetMeshCookedDataDeprecated = [](PhysxCookingAsyncContext context,
                                                                const PhysxCookingComputeRequest& request,
                                                                const omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params) {
        return gCookingComputeService->requestDeformableBodyTetMeshCookedDataDeprecated(context, request, params);
    };

    iface.requestParticleClothMeshCookedDataDeprecated = [](PhysxCookingAsyncContext context,
                                                            const PhysxCookingComputeRequest& request,
                                                            const omni::physx::ParticleClothMeshCookingParamsDeprecated& params) {
        return gCookingComputeService->requestParticleClothMeshCookedDataDeprecated(context, request, params);
    };

    iface.requestDeformableVolumeMeshCookedData = [](PhysxCookingAsyncContext context,
                                                     const PhysxCookingComputeRequest& request,
                                                     const omni::physx::DeformableVolumeMeshCookingParams& params) {
        return gCookingComputeService->requestDeformableVolumeMeshCookedData(context, request, params);
    };

    iface.requestVolumeDeformableBodyCookedData = [](PhysxCookingAsyncContext context,
                                                     const PhysxCookingComputeRequest& request,
                                                     const omni::physx::VolumeDeformableBodyCookingParams& params) {
        return gCookingComputeService->requestVolumeDeformableBodyCookedData(context, request, params);
    };

    iface.requestSurfaceDeformableBodyCookedData = [](PhysxCookingAsyncContext context,
                                                      const PhysxCookingComputeRequest& request,
                                                      const omni::physx::SurfaceDeformableBodyCookingParams& params) {
        return gCookingComputeService->requestSurfaceDeformableBodyCookedData(context, request, params);
    };

    iface.requestParticlePoissonSamplingCookedData = [](PhysxCookingAsyncContext context,
                                                        const PhysxCookingComputeRequest& request,
                                                        const omni::physx::ParticlePoissonSamplingCookingParams& params) {
        return gCookingComputeService->requestParticlePoissonSamplingCookedData(context, request, params);
    };

    iface.isOVCNodeDeprecated = isOVCNode;

    iface.readDeformableBodyTetMeshDataDeprecated = [](PhysxCookingDeformableBodyTetMeshDataDeprecated& out,
                                                       const PhysxCookedDataSpan& cookedData) {
        return cookingtask::readDeformableBodyTetMeshDataDeprecated(out, cookedData);
    };

    iface.readParticleClothMeshDataDeprecated = [](PhysxCookingParticleClothMeshDataDeprecated& out,
                                                   const PhysxCookedDataSpan& cookedData) {
        return cookingtask::readParticleClothMeshDataDeprecated(out, cookedData);
    };

    iface.readParticlePoissonSamplingData = [](PhysxCookingParticlePoissonSamplingData& out,
                                               const PhysxCookedDataSpan& cookedData) {
        return cookingtask::readParticlePoissonSamplingData(out, cookedData);
    };

    iface.readVolumeDeformableBodyData = [](PhysxCookingVolumeDeformableBodyData& out,
                                            const PhysxCookedDataSpan& cookedData) {
        return cookingtask::readVolumeDeformableBodyData(out, cookedData);
    };

    iface.readSurfaceDeformableBodyData = [](PhysxCookingSurfaceDeformableBodyData& out,
                                             const PhysxCookedDataSpan& cookedData) {
        return cookingtask::readSurfaceDeformableBodyData(out, cookedData);
    };

}
