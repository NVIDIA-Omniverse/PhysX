// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "UjitsoMeshProcessors.h"
#include "UjitsoMeshUtils.inl"
#include "UjitsoTriangulationContainer.h"

// TEMP: this can be removed once it is added to DataStoreUtils.inl on the rendering side
#include <type_traits>

#include <omni/physx/MeshKey.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <carb/tasking/ITasking.h>
#include <common/utilities/Utilities.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

using namespace carb::ujitso;
using namespace pxr;
using namespace physx;

namespace omni
{
namespace physx
{

// Cooking processor declaration
struct PhysxMeshCookingProcessor : public ProcessorImpl<PhysxMeshCookingProcessor>
{
    PhysxMeshCookingProcessor(ICookingComputeService& cookingComputeService);

    static constexpr uint64_t sVersion = 2;
    static constexpr uint32_t sBatchHint = 1;

    PROCESSOR_INFO(PhysxMeshCookingProcessor, sVersion, sBatchHint);

    OperationResult gatherDependenciesImpl(DependencyContext& context, DependencyJob& job, DependencyHandle handle);

    OperationResult buildImpl(BuildContext& context, BuildJob& job, BuildHandle handle);

private:
    cookingtask::CookingTask* createCookingTask(
        PhysxCookingDataType::Enum dataType, PhysxCookingComputeResult& result, DynamicRequest<>& request);

    ICookingComputeService& m_cookingComputeService;
    PxCudaContextManager* m_cudaContextManager;
};


// Global functions

const RequestFilter getPhysxMeshCookingProcessorFilter()
{
    static const std::vector<KeyTokenEx> filter =
    {
        KeyTokenEx{PHYSX_COOK_MESH_STR},    // void (identifier)
    };

    return { filter.data(), (uint32_t)filter.size() };
}

Processor* createPhysxMeshCookingProcessor(ICookingComputeService& cookingComputeService)
{
    return new PhysxMeshCookingProcessor(cookingComputeService);
}

void releasePhysxMeshCookingProcessor(Processor* processor)
{
    delete static_cast<PhysxMeshCookingProcessor*>(processor);
}


// PhysxMeshCookingProcessor public methods

PhysxMeshCookingProcessor::PhysxMeshCookingProcessor(ICookingComputeService& cookingComputeService) :
    m_cookingComputeService(cookingComputeService), m_cudaContextManager(nullptr)
{
}

OperationResult PhysxMeshCookingProcessor::gatherDependenciesImpl(DependencyContext& context, DependencyJob& job,
                                                                  DependencyHandle handle)
{
    CARB_PROFILE_ZONE(0, "PhysxMeshCookingProcessor::gatherDependenciesImpl");

    DynamicRequest<> req(job.request);

    // Gather build parameters
    const uint32_t triangulationVersion = req.getAsDefault(TRIANGULATION_VERSION_STR, (uint32_t)0);
    CHECK_RETURN_VALUE_ON_FAIL(triangulationVersion == PhysxCookingDataVersion_MeshTriangulation,
                               OperationResult::FAILURE);
    ContainerHandle containerHandle = req.getAsDefault<ContainerHandle>(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, {});
    const uint64_t developerKey = req.getAsDefault(PHYSX_COOKING_REQUEST_DEVELOPER_KEY, (uint64_t)0);
    const CacheBehaviorType cacheBehavior =
        (CacheBehaviorType)req.getAsDefault(PHYSX_UJITSO_CACHE_BEHAVIOR, (uint32_t)CacheBehaviorType::Default);

    // Dependent build
    RequestBuilder<> triangulationDataRequest;

    // Pass parameters from this build to dependent build
    triangulationDataRequest.add(PHYSX_TRIANGULATE_MESH_STR);
    triangulationDataRequest.add(TRIANGULATION_VERSION_STR, triangulationVersion);
    triangulationDataRequest.add(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, containerHandle);
    triangulationDataRequest.add(PHYSX_COOKING_REQUEST_DEVELOPER_KEY, developerKey);
    triangulationDataRequest.add(PHYSX_UJITSO_CACHE_BEHAVIOR, (uint32_t)cacheBehavior);

    // Add the dependency
    uint32_t dependencyIndex;
    CHECK_RETURN_VALUE_ON_FAIL(
        context.agent.service->addDependency(context.agent, handle, triangulationDataRequest.getRequest().getRequest(),
                                             &dependencyIndex) == OperationResult::SUCCESS, OperationResult::FAILURE);
    CHECK_RETURN_VALUE_ON_FAIL(dependencyIndex == 0, OperationResult::FAILURE);

    // Remove keys that should not be used to identify build
    req.removeKey(CONTEXT_ID);  // The context ID is just to test to see where the build came from
    req.removeKey(PHYSX_UJITSO_CACHE_BEHAVIOR); // The cache behavior is not identifying data
    req.removeKey(PRIM_MESH_TEXT);  // This is not used for cooking
    context.agent.service->addRequestTupleInput(context.agent, handle, req.getRequest(), true, false);

    // Set cache behavior based on caching flag
    CHECK_RETURN_VALUE_ON_FAIL(
        context.agent.service->setOption(context.agent, handle, KeyTokenEx(TOKEN_CACHE_BEHAVIOR_OPTION),
            &cacheBehavior, sizeof(cacheBehavior)) == OperationResult::SUCCESS, OperationResult::FAILURE);

    // Set cache key method to use request inputs
    const CacheKeyHashType cacheKeyMethod = CacheKeyHashType::Input;
    CHECK_RETURN_VALUE_ON_FAIL(
        context.agent.service->setOption(context.agent, handle, KeyTokenEx(TOKEN_CACHE_KEY_HASH_OPTION),
            &cacheKeyMethod, sizeof(cacheKeyMethod)) == OperationResult::SUCCESS, OperationResult::FAILURE);

    return OperationResult::SUCCESS;
}

static bool validateCookingVersion(PhysxCookingDataType::Enum dataType, uint32_t cookingVersion)
{
    switch (dataType)
    {
        case PhysxCookingDataType::eTRIANGLE_MESH:
            return cookingVersion == PhysxCookingDataVersion_TriangleMesh;
        case PhysxCookingDataType::eSDF_TRIANGLE_MESH:
            return cookingVersion == PhysxCookingDataVersion_TriangleMeshSDF;
        case PhysxCookingDataType::eCONVEX_MESH:
            return cookingVersion == PhysxCookingDataVersion_ConvexMesh;
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION:
            return cookingVersion == PhysxCookingDataVersion_ConvexDecomposition;
        case PhysxCookingDataType::eSPHERE_FILL:
            return cookingVersion == PhysxCookingDataVersion_SphereFill;
        case PhysxCookingDataType::eSOFT_BODY_DEPRECATED:
            return cookingVersion == PhysxCookingDataVersion_SoftBodyMeshDeprecated;
        case PhysxCookingDataType::eDEFORMABLE_TETRAHEDRAL_MESH_DEPRECATED:
            return cookingVersion == PhysxCookingDataVersion_DeformableBodyTetMeshDeprecated;
        case PhysxCookingDataType::ePARTICLE_CLOTH_DEPRECATED:
            return cookingVersion == PhysxCookingDataVersion_ParticleClothDeprecated;
        case PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH:
            return cookingVersion == PhysxCookingDataVersion_DeformableVolumeMesh;
        case PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY:
            return cookingVersion == PhysxCookingDataVersion_VolumeDeformableBody;
        case PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY:
            return cookingVersion == PhysxCookingDataVersion_SurfaceDeformableBody;
        case PhysxCookingDataType::ePARTICLE_POISSON_SAMPLING:
            return cookingVersion == PhysxCookingDataVersion_ParticlePoissonSampling;

        default:
            CARB_LOG_ERROR("Unhandled cooking data type: %d", dataType);
            return false;
    }
}

OperationResult PhysxMeshCookingProcessor::buildImpl(BuildContext& context, BuildJob& job, BuildHandle handle)
{
    CARB_PROFILE_ZONE(0, "PhysxMeshCookingProcessor::buildImpl");

    // decode the request again
    DynamicRequest<> req(job.request);

    // Create metadata
    createMetaData<uint64_t>(context.agent, handle) = req.getAsDefault(CONTEXT_ID, (uint64_t)0);

    // Build external data
    const PhysxCookingDataType::Enum dataType =
        (PhysxCookingDataType::Enum)req.getAsDefault(DATA_TYPE_STR, (uint32_t)0);
    const uint32_t cookingVersion = req.getAsDefault(COOKING_VERSION_STR, (uint32_t)0);
    CHECK_RETURN_VALUE_ON_FAIL(validateCookingVersion(dataType, cookingVersion), OperationResult::FAILURE);
    const double metersPerUnit = req.getAsDefault(METERS_PER_UNIT_STR, (double)1);
    const bool buildGpuData = req.getAsDefault(BUILD_GPU_DATA_STR, true);
    const bool buildTriangleAdjacencies = req.getAsDefault(BUILD_TRIANGLE_ADJACENCIES_STR, true);
    const char* primMeshText = getRequestValueString(context.agent, req.getRequest(), PRIM_MESH_TEXT);
    const bool enableGpuCooking = req.getAsDefault(PHYSX_UJITSO_ENABLE_GPU_COOKING, true);

    // Gather dependent data
    FetchDependencies<1> deps(context.agent, handle);
    CHECK_RETURN_VALUE_ON_FAIL(deps.getResult() == OperationResult::SUCCESS, OperationResult::FAILURE);
    CHECK_RETURN_VALUE_ON_FAIL(deps.getDepCount() == 1, OperationResult::FAILURE);
    std::vector<uint8_t> triangulatedMeshBuffer;
    CHECK_RETURN_VALUE_ON_FAIL(
        copyExternalStorage(context.agent, *context.agent.store, deps.getDep(0), triangulatedMeshBuffer) ==
                            OperationResult::SUCCESS, OperationResult::FAILURE);
    PhysxCookingMeshTriangulationView triangulationView;
    uint16_t maxMaterialIndex;
    CHECK_RETURN_VALUE_ON_FAIL(buildTriangulationViewFromData(triangulationView, maxMaterialIndex,
        triangulatedMeshBuffer), OperationResult::FAILURE);

    PhysxCookingComputeResult result;
    PhysxCookingComputeRequest cookingRequest;
    result.request = &cookingRequest;

    // Create a cooking task for the shape type
    cookingtask::CookingTask* task = createCookingTask(dataType, result, req);
    CHECK_RETURN_VALUE_ON_FAIL(task, OperationResult::FAILURE);
    carb::extras::detail::ScopeGuard onExit([task](){ delete task; });  // Make sure the task gets deleted

    // If requested, attempt to enable GPU cooking
    if (enableGpuCooking)
    {
        PxPhysicsGpu* physicsGPU = nullptr;
        if (m_cookingComputeService.lazyGetCudaContextManager(dataType, cookingRequest, m_cudaContextManager, physicsGPU))
        {
            task->setPxCudaAndGPUPointers(m_cudaContextManager, physicsGPU);
        }
        else
        {
            CARB_LOG_WARN("PhysxMeshCookingProcessor::buildImpl: GPU cooking attempted for prim (%s), but CUDA context "
                        "manager and GPU Physics could not be obtained.  Falling back to CPU.",
                        primMeshText ? primMeshText : "unknown");
        }
    }

    // Do the cooking
    TriangulationMesh& triangulationMesh = task->getTriangulationMesh();
    // Copies; should be able to eliminate
    buildTriangulationMeshFromView(triangulationMesh, triangulationView, maxMaterialIndex);
    task->setMetersPerUnit(metersPerUnit);
    task->setBuildGpuData(buildGpuData);
    task->setBuildTriangleAdjacencies(buildTriangleAdjacencies);
    task->setPrimPathText(primMeshText);
    task->performTask();
    task->setFinalized(true);   // Prevent task->finalize() from being called

    // If the task didn't succeed, don't try to store a result
    if (!task->isSucceeded())
    {
        return OperationResult::FAILURE;
    }

    // Get the data from the task and store it
    std::vector<std::unique_ptr<::physx::PxDefaultMemoryOutputStream>>& outputStreams =
        task->getCookedDataOutputStreams();
    const uint32_t streamCount = (uint32_t)outputStreams.size();

    if (streamCount == 0)
    {
        return OperationResult::FAILURE;
    }

    std::vector<uint8_t*> data(streamCount);
    std::vector<size_t> dataSize(streamCount);
    for (size_t i = 0; i < data.size(); ++i)
    {
        PxDefaultMemoryOutputStream& outputStream = *outputStreams[i].get();
        data[i] = (uint8_t*)outputStream.getData();
        dataSize[i] = outputStream.getSize();
    }
    const std::vector<ValidationType> validationType(streamCount, ValidationType::MANDATORY);
    const OperationResult storeResult =
        context.agent.service->storeExternalData(context.agent, handle, data.data(), dataSize.data(),
                                                 validationType.data(), (uint32_t)streamCount);

    return storeResult;
}


// PhysxMeshCookingProcessor private methods

cookingtask::CookingTask* PhysxMeshCookingProcessor::createCookingTask(
    PhysxCookingDataType::Enum dataType, PhysxCookingComputeResult& result, DynamicRequest<>& request)
{
    CARB_PROFILE_ZONE(0, "PhysxMeshCookingProcessor::createCookingTask");

    switch (dataType)
    {
    case PhysxCookingDataType::eCONVEX_MESH:
        {
            ConvexMeshCookingParams convexDesc;
            CHECK_RETURN_NULL_ON_FAIL(getParamsFromRequest(convexDesc, request));
            return cookingtask::createConvexMeshCookingTask(convexDesc, result);
        }
    case PhysxCookingDataType::eCONVEX_DECOMPOSITION:
        {
            ConvexDecompositionCookingParams convexDecompositionDesc;
            CHECK_RETURN_NULL_ON_FAIL(getParamsFromRequest(convexDecompositionDesc, request));
            return cookingtask::createConvexDecompositionCookingTask(convexDecompositionDesc, result);
        }
    case PhysxCookingDataType::eTRIANGLE_MESH:
    case PhysxCookingDataType::eSDF_TRIANGLE_MESH:
        {
            TriangleMeshCookingParams triangleMeshDesc;
            CHECK_RETURN_NULL_ON_FAIL(getParamsFromRequest(triangleMeshDesc, request));
            SdfMeshCookingParams sdfMeshDesc;
            CHECK_RETURN_NULL_ON_FAIL(getParamsFromRequest(sdfMeshDesc, request));
            return cookingtask::createTriangleMeshCookingTask(triangleMeshDesc, sdfMeshDesc, result);
        }
    case PhysxCookingDataType::eSPHERE_FILL:
        {
            SphereFillCookingParams sphereFillDesc;
            CHECK_RETURN_NULL_ON_FAIL(getParamsFromRequest(sphereFillDesc, request));
            return cookingtask::createSphereFillCookingTask(sphereFillDesc, result);
        }
    default:
        break;
    }

    return nullptr;
}

} // namespace physx
} // namespace omni
