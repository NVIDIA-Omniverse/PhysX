// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "CollisionRepresentation.h"

#include "OmniPhysX.h"
#include "CookingDataAsync.h"
#include "MeshCache.h"

#include "common/utilities/MemoryUtilities.h"
#include "common/utilities/Defer.h"
#include "common/utilities/Utilities.h"
#include "usdLoad/LoadUsd.h"
#include <PxPhysicsAPI.h>

using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

namespace omni
{
namespace physx
{

struct CollisionRepresentationConvex
{
    PhysxCollisionRepresentationRequest request;
    PhysxCollisionRepresentationConvexResult::CallbackType onResult;

    static void returnMemoryCachedResult(const omni::physx::PhysxCookingDataType::Enum dataType,
                                         const omni::physx::usdparser::MeshKey& cookedDataCRC,
                                         const PhysxCollisionRepresentationConvexResult::CallbackType& onResult)
    {
        PxPhysics& pxPhysics = *OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        std::vector<PxConvexMesh*> meshes;
        switch (dataType)
        {
        case PhysxCookingDataType::eCONVEX_MESH: {
            meshes.resize(1);
            meshes[0] = omni::physx::getMeshCache()->getConvexMesh(cookedDataCRC);
            break;
        }
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION: {
            auto it = omni::physx::getMeshCache()->getConvexDecompositionMap().find(cookedDataCRC);

            for (const auto& item : it->second)
            {
                meshes.push_back(item);
            }
            break;
        }
        default:
            break;
        }
        if (meshes.empty() || !returnValidResult(meshes, onResult))
        {
            // Return error if at least one of the convexes had some issue
            PhysxCollisionRepresentationConvexResult result;
            onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_RESULT, result);
        }
    }

    static bool returnValidResult(std::vector<PxConvexMesh*>& meshes,
                                  const PhysxCollisionRepresentationConvexResult::CallbackType& onResult)
    {
        std::vector<ConvexMeshData> convexMeshDatas;
        convexMeshDatas.resize(meshes.size());
        const ConvexMeshDataMap& meshMap = getMeshCache()->getConvexMeshDataMap();
        for (size_t index = 0; index < meshes.size(); ++index)
        {
            PxConvexMesh* mesh = meshes[index];
            ConvexMeshDataMap::const_iterator it = meshMap.find(mesh);
            if (it == meshMap.end())
                return false;
            const ConvexMeshData& md = it->second;
            ConvexMeshData& meshData = convexMeshDatas[index];
            meshData.indices = md.indices;
            meshData.numPolygons = md.numPolygons;
            meshData.numVertices = md.numVertices;
            meshData.polygons = md.polygons;
            meshData.vertices = md.vertices;
            meshData.computeNumIndices();
        }
        PhysxCollisionRepresentationConvexResult convexResult;
        convexResult.convexes = { convexMeshDatas.data(), convexMeshDatas.size() };
        onResult(PhysxCollisionRepresentationResult::eRESULT_VALID, convexResult);
        return true;
    }

    static bool hasMemoryCachedResult(const omni::physx::PhysxCookingDataType::Enum dataType,
                                      const omni::physx::usdparser::MeshKey& cookedDataCRC)
    {
        switch (dataType)
        {
        case PhysxCookingDataType::eCONVEX_MESH:
            return omni::physx::getMeshCache()->hasConvexMesh(cookedDataCRC);
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION:
            return omni::physx::getMeshCache()->hasConvexDecomposition(cookedDataCRC);
        }
        return false;
    }

    void returnNewlyComputedRepresentation(const omni::physx::PhysxCookingComputeResult& cookingResult)
    {
        ::physx::PxPhysics& pxPhysics = *OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        switch (cookingResult.request->dataType)
        {
        case PhysxCookingDataType::eCONVEX_MESH:
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION:
            if (returnConvexRepresentation(cookingResult))
                return;
            break;
        default:
            CARB_LOG_ERROR("returnNewlyComputedRepresentation received unexpected PhysxCookingDataType");
            break;
        }
        // Return error if at least one of the convexes had some issue
        PhysxCollisionRepresentationConvexResult result;
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_RESULT, result);
    }

    bool returnConvexRepresentation(const omni::physx::PhysxCookingComputeResult& result)
    {
        PxPhysics& pxPhysics = *OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        std::vector<PxConvexMesh*> meshes;
        bool createRuntimeMeshSucceeded = false;
        switch (result.request->dataType)
        {
        case PhysxCookingDataType::eCONVEX_MESH:
            meshes.resize(1);
            createRuntimeMeshSucceeded = omni::physx::getMeshCache()->createRuntimeConvexMesh(
                pxPhysics, result.cookedDataCRC, result.cookedData[0], &meshes[0]);
            break;
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION:
            createRuntimeMeshSucceeded = omni::physx::getMeshCache()->createRuntimeConvexDecomposition(
                pxPhysics, result.cookedDataCRC, result.cookedData, result.cookedDataNumElements, &meshes);
            break;
        }
        if (createRuntimeMeshSucceeded)
        {
            return returnValidResult(meshes, onResult);
        }
        return false;
    }
};

// This is a template on ResultType and ContextType so that it can be reused for other types of collision representation
template <typename ResultType, typename ContextType>
PhysxCollisionRepresentationTask requestCollisionRepresentation(const PhysxCollisionRepresentationRequest& request,
                                                                typename ResultType::CallbackType onResult)
{
    // As we can't extract convex data directly from cooked data (see PX-1707), we have to create the PxConvexMesh.
    //
    // General flow to achieve this in an eficient way is:
    // 1. Parse collision APIs to get the collision descriptors
    // 2. Compute MeshKey and CookedDataCRC (synchronously)
    // 3. Check if an existing RUNTIME mesh already exists in mesh cache for that CookedDataCRC (synchronously)
    // 4. If it exists then access Px*Meshes from MeshCache at that CookedDataCRC and return Collision Representation
    // Alternatively:
    // 5. If it doesn't exists issue a cooking request for data that will hit disk / network (sync or async)
    // 6. With cooked data, insert new Px*Meshes inside MeshCache, so that successive invocations will early exit at 4.

    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();

    ResultType result;
    if (cookingDataAsync == nullptr)
    {
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_NOT_READY, result);
        return { nullptr };
    }
    omni::physx::PhysxCookingAsyncContext asyncContext = cookingDataAsync->getCookingAsyncContext();

    omni::physx::IPhysxCookingService* cookingService = carb::getCachedInterface<omni::physx::IPhysxCookingService>();
    //----------------------------------------------------------------------------------
    // 1. Parse the collision API
    //----------------------------------------------------------------------------------
    pxr::SdfPath collisionPath = intToPath(request.collisionPrimId);
    pxr::SdfPath geometryPath = collisionPath;
    PhysxShapeDesc* collisionDesc = parseCollision(request.stageId, collisionPath, geometryPath);
    if (collisionDesc == nullptr)
    {
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_PARSING, result);
        return { nullptr };
    }

    // Delete the collision desc on scope exit
    auto deleteCollisionDesc = CreateDeferLambda([&] { releaseDesc(collisionDesc); });

    //----------------------------------------------------------------------------------
    // 2. Compute meshKey and CookedDataCRC
    //----------------------------------------------------------------------------------

    // Setup the cooking request with shared parameters between
    PhysxCookingComputeRequest cookingRequest;
    cookingRequest.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID;
    cookingRequest.primId = request.collisionPrimId;
    cookingRequest.primStageId = request.stageId;
    const bool async = request.options.hasFlag(PhysxCollisionRepresentationRequest::Options::kComputeAsynchronously);
    cookingRequest.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

    cookingRequest.mode = PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;

    // Cooking CRC computations are always sync, but let's make it explicit
    cookingRequest.options.setFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);

    MeshKey cookedDataCRC;
    MeshKey meshKey;
    cookingRequest.onFinished = [&cookedDataCRC, &meshKey](const omni::physx::PhysxCookingComputeResult& cookingResult) {
        if (cookingResult.result == PhysxCookingResult::eVALID)
        {
            cookedDataCRC = cookingResult.cookedDataCRC;
            meshKey = cookingResult.meshKey;
        }
    };

    // Request the correct type of cooking CRC computation
    switch (collisionDesc->type)
    {
    case eConvexMeshShape:
        cookingRequest.dataType = PhysxCookingDataType::eCONVEX_MESH;
        cookingService->requestConvexMeshCookedData(
            asyncContext, cookingRequest, static_cast<ConvexMeshPhysxShapeDesc*>(collisionDesc)->convexCookingParams);
        break;
    case eConvexMeshDecompositionShape:
        cookingRequest.dataType = PhysxCookingDataType::eCONVEX_DECOMPOSITION;
        cookingService->requestConvexMeshDecompositionCookedData(
            asyncContext, cookingRequest,
            static_cast<ConvexMeshDecompositionPhysxShapeDesc*>(collisionDesc)->convexDecompositionCookingParams);
        break;
    default:
        // Unsupported approximation type, we just error out
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_UNSUPPORTED_APPROXIMATION, result);
        return { nullptr };
    }

    if (meshKey == MeshKey()) // It means that we never got a cookingResult.result == PhysxCookingResult::eVALID
    {
        // Return error for invalid CRC
        ResultType result;
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_COOKING_FAILED, result);
        return { nullptr };
    }

    //----------------------------------------------------------------------------------
    // 3. and 4. Check for memory cached result and eventually return it
    //----------------------------------------------------------------------------------
    if (ContextType::hasMemoryCachedResult(cookingRequest.dataType, cookedDataCRC))
    {
        ContextType::returnMemoryCachedResult(cookingRequest.dataType, cookedDataCRC, onResult);
        return { nullptr };
    }

    //----------------------------------------------------------------------------------
    // 5. Issue a cooking data request, as there is no memory cached result
    //----------------------------------------------------------------------------------

    // We can't use unique_ptr because PhysxCookingComputeRequest::onFinished is copied and not moved
    // (root reason is that we can't express move semantics in carbonite interfaces).
    std::shared_ptr<ContextType> context = std::make_shared<ContextType>();
    context->request = request;
    context->onResult = onResult;
    cookingRequest.mode = PhysxCookingComputeRequest::eMODE_REQUEST_COOKED_DATA;
    cookingRequest.options.setFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously, async);
    cookingRequest.meshKey = meshKey;
    cookingRequest.onFinished = [context](const omni::physx::PhysxCookingComputeResult& cookingResult) {
        if (cookingResult.result == PhysxCookingResult::eVALID)
        {
            //----------------------------------------------------------------------------------
            // 6. Return results and insert cooked data in runtime mesh cache
            //----------------------------------------------------------------------------------
            context->returnNewlyComputedRepresentation(cookingResult);
        }
        else
        {
            // Return error for invalid cooking
            ResultType result;
            context->onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_COOKING_FAILED, result);
        }
    };

    PhysxCollisionRepresentationTask handle = { nullptr };
    // Request the correct type of cooking data
    switch (collisionDesc->type)
    {
    case eConvexMeshShape:
        handle.handle = cookingService->requestConvexMeshCookedData(
            asyncContext, cookingRequest, static_cast<ConvexMeshPhysxShapeDesc*>(collisionDesc)->convexCookingParams);
        break;
    case eConvexMeshDecompositionShape:
        handle.handle = cookingService->requestConvexMeshDecompositionCookedData(
            asyncContext, cookingRequest,
            static_cast<ConvexMeshDecompositionPhysxShapeDesc*>(collisionDesc)->convexDecompositionCookingParams);
        break;
    default:
        // Unsupported approximation type, we just error out
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_UNSUPPORTED_APPROXIMATION, result);
        return { nullptr };
    }

    return handle;
}

PhysxCollisionRepresentationTask requestConvexCollisionRepresentation(
    const PhysxCollisionRepresentationRequest& request, PhysxCollisionRepresentationConvexResult::CallbackType onResult)
{
    return requestCollisionRepresentation<PhysxCollisionRepresentationConvexResult, CollisionRepresentationConvex>(
        request, onResult);
}

void cancelCollisionRepresentationTask(PhysxCollisionRepresentationTask task, bool invokeCallbackAnyway)
{
    omni::physx::IPhysxCookingService* cookingService = carb::getCachedInterface<omni::physx::IPhysxCookingService>();
    cookingService->cancelTask(task.handle, invokeCallbackAnyway);
}
} // namespace physx
} // namespace omni
