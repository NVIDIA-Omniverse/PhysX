// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../service/CookingComputeService.h"
#include "UjitsoMeshProcessors.h"
#include "UjitsoMeshUtils.inl"
#include "UjitsoTriangulationContainer.h"

#include "../utility/TriangulateUsdMeshPrim.h"

#include <omni/physx/MeshKey.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <common/utilities/Utilities.h>

using namespace carb::ujitso;
using namespace pxr;
using namespace physx;
using namespace triangulateusd;

namespace omni
{
namespace physx
{

// Utility

static void extractPrimTriangulation(PxDefaultMemoryOutputStream& stream, const PhysicsTriangulationBuildData& triBuildData)
{
    CARB_PROFILE_ZONE(0, "omni::physx::extractPrimTriangulation");

    // fill in the mesh view with data from the container
    PhysxCookingMeshView meshView;
    meshView.points = omni::span<const carb::Float3>(triBuildData.points.data(), triBuildData.points.size());
    meshView.indices = omni::span<const int32_t>(triBuildData.indices.data(), triBuildData.indices.size());
    meshView.faces = omni::span<const int32_t>(triBuildData.faceCounts.data(), triBuildData.faceCounts.size());
    meshView.faceMaterials = omni::span<const uint16_t>(triBuildData.faceMaterials.data(), triBuildData.faceMaterials.size());
    meshView.holeIndices = omni::span<const int32_t>(triBuildData.holeIndices.data(), triBuildData.holeIndices.size());
    meshView.rightHandedOrientation = triBuildData.rightHandedOrientation;

    const uint16_t maxMaterialIndex = ICookingComputeService::getMaxMaterialIndex(meshView);

    TriangulateUSDPrim* triangulator = TriangulateUSDPrim::create(meshView);
    CHECK_RETURN_ON_FAIL(triangulator);

    triangulator->triangulate();

    // Version
    const uint32_t version = (uint32_t)PhysxCookingDataVersion_MeshTriangulation;
    stream.write(&version, sizeof(version));

    // Vertices
    uint32_t vertexCount;
    const float* vertices = triangulator->getVertices(vertexCount);
    stream.write(&vertexCount, sizeof(vertexCount));
    if (vertexCount)
    {
        stream.write(vertices, sizeof(float) * 3 * vertexCount);
    }

    // Triangles
    uint32_t triangleCount;
    const uint32_t* indices = triangulator->getIndices(triangleCount);
    stream.write(&triangleCount, sizeof(triangleCount));
    if (triangleCount)
    {
        stream.write(indices, sizeof(int32_t) * 3 * triangleCount);
    }

    // Triangle face map
    const uint32_t* triangleFaceMap = triangulator->getTriangleFaceMap(triangleCount);
    stream.write(&triangleCount, sizeof(triangleCount));
    if (triangleCount)
    {
        stream.write(triangleFaceMap, sizeof(int32_t) * triangleCount);
    }

    uint32_t faceMaterialsCount;
    const uint16_t* faceMaterials = triangulator->getFaceMaterials(faceMaterialsCount);
    if (faceMaterials)
    {
        stream.write(&faceMaterialsCount, sizeof(faceMaterialsCount));
        stream.write(faceMaterials, sizeof(uint16_t) * faceMaterialsCount);
    }
    else
    {
        // Note: It happens sometimes that faceMaterialsCount > 0 but faceMaterials == nullptr.
        faceMaterialsCount = 0;
        stream.write(&faceMaterialsCount, sizeof(faceMaterialsCount));
    }

    stream.write(&maxMaterialIndex, sizeof(uint16_t));

    triangulator->release();
}


// Cooking processor declaration
struct PhysxMeshTriangulationProcessor : public ProcessorImpl<PhysxMeshTriangulationProcessor>
{
    static constexpr uint64_t sVersion = 2;
    static constexpr uint32_t sBatchHint = 1;

    PROCESSOR_INFO(PhysxMeshTriangulationProcessor, sVersion, sBatchHint);

    PhysxMeshTriangulationProcessor();

    bool isValid() const;

    OperationResult gatherDependenciesImpl(DependencyContext& context, DependencyJob& job, DependencyHandle handle);

    OperationResult buildImpl(BuildContext& context, BuildJob& job, BuildHandle handle);
};


// Global functions

const RequestFilter getPhysxMeshTriangulationProcessorFilter()
{
    static const std::vector<KeyTokenEx> filter =
    {
        KeyTokenEx{PHYSX_TRIANGULATE_MESH_STR}, // void (identifier)
    };

    return { filter.data(), (uint32_t)filter.size() };
}

Processor* createPhysxMeshTriangulationProcessor()
{
    PhysxMeshTriangulationProcessor* processor = new PhysxMeshTriangulationProcessor();

    if (processor && !processor->isValid())
    {
        delete processor;
        processor = nullptr;
    }

    return processor;
}

void releasePhysxMeshTriangulationProcessor(Processor* processor)
{
    delete static_cast<PhysxMeshTriangulationProcessor*>(processor);
}


// PhysxMeshTriangulationProcessor public methods

PhysxMeshTriangulationProcessor::PhysxMeshTriangulationProcessor()
{
}

bool PhysxMeshTriangulationProcessor::isValid() const
{
    return true;
}

OperationResult PhysxMeshTriangulationProcessor::gatherDependenciesImpl(DependencyContext& context, DependencyJob& job,
                                                                        DependencyHandle handle)
{
    CARB_PROFILE_ZONE(0, "PhysxMeshTriangulationProcessor::gatherDependenciesImpl");

    DynamicRequest<> req(job.request);

    const uint32_t triangulationVersion = req.getAsDefault(TRIANGULATION_VERSION_STR, (uint32_t)0);
    CHECK_RETURN_VALUE_ON_FAIL(triangulationVersion == PhysxCookingDataVersion_MeshTriangulation,
                               OperationResult::FAILURE);

    const CacheBehaviorType cacheBehavior =
        (CacheBehaviorType)req.getAsDefault(PHYSX_UJITSO_CACHE_BEHAVIOR, (uint32_t)CacheBehaviorType::Default);

    // move the container from the request to the service so it can be accessed in the build function
    ContainerHandle containerHandle = req.getAsDefault<ContainerHandle>(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, {});
    context.agent.service->addContainer(context.agent, handle, containerHandle);

    // Remove container handle from identifying data; this won't be necessary when we update to the latest ujitso
    req.removeKey(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME);
 
    // Remove cache enabled bool from the identifying data
    req.removeKey(PHYSX_UJITSO_CACHE_BEHAVIOR);
    
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

OperationResult PhysxMeshTriangulationProcessor::buildImpl(BuildContext& context, BuildJob& job, BuildHandle handle)
{
    CARB_PROFILE_ZONE(0, "PhysxMeshTriangulationProcessor::buildImpl");

    // grab the container that holds the data needed to cook the mesh
    DynamicRequest<> req(job.request);
    ContainerHandle containerHandle = req.getAsDefault<ContainerHandle>(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, {});
    CHECK_RETURN_VALUE_ON_FAIL(containerHandle.value, OperationResult::FAILURE);
    const IContainer* container;
    const auto opRes = context.agent.service->getContainer(context.agent, handle, containerHandle, container);
    CHECK_RETURN_VALUE_ON_FAIL(opRes == OperationResult::SUCCESS, OperationResult::FAILURE);
    const bool isExpectedType = (container->getType() == PhysicsTriangulationInputContainer::getTypeStatic());
    CHECK_RETURN_VALUE_ON_FAIL(isExpectedType, OperationResult::FAILURE);
    const PhysicsTriangulationInputContainer* triContainer = (PhysicsTriangulationInputContainer*)container;
    const PhysicsTriangulationBuildData& triBuildData = triContainer->getBuildData();

    const int triangulationVersion = (int)req.getAsDefault(TRIANGULATION_VERSION_STR, (uint32_t)0);
    CHECK_RETURN_VALUE_ON_FAIL(triangulationVersion == PhysxCookingDataVersion_MeshTriangulation,
                               OperationResult::FAILURE);

    // Extract triangle mesh data into a memory stream
    PxDefaultMemoryOutputStream stream;
    extractPrimTriangulation(stream, triBuildData);
    CHECK_RETURN_VALUE_ON_FAIL(stream.getSize() > 0, OperationResult::FAILURE);

    return storeExternalStorage(context.agent, handle, (const uint8_t*)stream.getData(), (size_t)stream.getSize(),
                                ValidationType::MANDATORY);
}

} // namespace physx
} // namespace omni
