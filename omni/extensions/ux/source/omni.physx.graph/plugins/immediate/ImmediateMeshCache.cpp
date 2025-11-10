// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "ImmediateMeshCache.h"
#include "ImmediateNode.h"

#include <omni/physx/IPhysxSettings.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

using namespace omni::physx::graph;
using namespace ::physx;

const static carb::RStringKey kObserverName("omni.physx.graph");

constexpr bool gEnableLoadSaveFromCache = true; // Just for debug purposes

MeshFacesToTrianglesMapping* ImmediateMeshCache::insertNewTriangulatedMesh(
    const omni::physx::PhysxCookingMeshTriangulationView& triangulationView, MeshKey meshKey)
{
    lock_guard lock(mTriangulationCache.mutex);
    std::unique_ptr<MeshFacesToTrianglesMapping> mesh = std::make_unique<MeshFacesToTrianglesMapping>();
    mesh->trianglesToFacesMapping.insert(mesh->trianglesToFacesMapping.begin(),
                                         triangulationView.trianglesFaceMap.begin(),
                                         triangulationView.trianglesFaceMap.end());
    mTriangulationCache.cache[meshKey] = std::move(mesh);
    return mTriangulationCache.cache[meshKey].get();
}

bool ImmediateMeshCache::ensurePxPhysics(::physx::PxTolerancesScale tolerancesScale)
{
    if (mPhysics == nullptr)
    {
        mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, tolerancesScale);
        if (!mPhysics)
        {
            CARB_LOG_ERROR("ImmediateShared::Couldn't get PhysX Physics");
            return false;
        }
    }
    return true;
}
bool ImmediateMeshCache::initialize(PxFoundation& foundation)
{
    mFoundation = &foundation;
    iCookingService = carb::getCachedInterface<omni::physx::IPhysxCookingService>();
    if (!iCookingService)
    {
        return false;
    }
    subscribeToStageEvents();
    return true;
}

void ImmediateMeshCache::release()
{
    releaseMemoryCache();
    unsubscribeFromStageEvents();

    iCookingService = nullptr;
    mFoundation = nullptr;
}

void ImmediateMeshCache::subscribeToStageEvents()
{
    auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
    omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
    if (usdContext)
    {
        mStageEvtSub = {
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eOpened),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eClosed),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eOmniGraphStartPlay),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eAnimationStartPlay),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eSimulationStartPlay),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eOmniGraphStopPlay),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eAnimationStopPlay),
                             [this](const auto&) { releaseMemoryCache(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eSimulationStopPlay),
                             [this](const auto&) { releaseMemoryCache(); }),
        };
    }
}

void ImmediateMeshCache::unsubscribeFromStageEvents()
{
    mStageEvtSub = {};
}

void ImmediateMeshCache::releaseMemoryCache()
{
    mTriangleMeshCache.releaseWithLock();
    mConvexMeshCache.releaseWithLock();
    // Release PxPhysics object
    {
        lock_guard lock(mPhysicsMutex);
        if (mPhysics)
        {
            mPhysics->release();
            mPhysics = nullptr;
        }
    }
}

bool ImmediateMeshCache::computeCookedDataCRCForMeshInputView(const MeshInputView& meshInputView,
                                                              MeshKey& meshKey,
                                                              MeshKey& cookedDataCRC)
{
    PhysxCookingComputeRequest cookingRequest;
    cookingRequest.dataInputMode = meshInputView.meshView.usePrimID ?
                                       omni::physx::PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID :
                                       omni::physx::PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW;
    cookingRequest.primId = meshInputView.meshView.primId;
    cookingRequest.primStageId = meshInputView.meshView.primStageId;
    cookingRequest.primMeshView = meshInputView.meshView.cookingMeshView;
    cookingRequest.primMeshMetersPerUnit = meshInputView.meshCollision.metersPerUnit;
    if (meshInputView.meshCollision.collisionApproximation == ImmediateNode::kCollisionApproximationConvexHull.token)
    {
        cookingRequest.dataType = omni::physx::PhysxCookingDataType::eCONVEX_MESH;
    }
    else
    {
        cookingRequest.dataType = omni::physx::PhysxCookingDataType::eTRIANGLE_MESH;
    }
    cookingRequest.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
    cookingRequest.meshKey = meshKey;
    using Options = omni::physx::PhysxCookingComputeRequest::Options;
    cookingRequest.options.setFlag(Options::kComputeAsynchronously, false);
    cookingRequest.options.setFlag(Options::kComputeGPUCookingData, false);
    cookingRequest.options.setFlag(Options::kLoadCookedDataFromCache, gEnableLoadSaveFromCache);
    cookingRequest.options.setFlag(Options::kSaveCookedDataToCache, gEnableLoadSaveFromCache);
    // We don't want to read USD at all for thread safety here
    cookingRequest.options.setFlag(Options::kForceDisableUSDAccess, !meshInputView.meshView.usePrimID);

    bool resultIsValid = false;
    cookingRequest.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        resultIsValid = result.result == PhysxCookingResult::eVALID;
        if (resultIsValid)
        {
            cookedDataCRC = result.cookedDataCRC;
            meshKey = result.meshKey;
        }
    };

    if (meshInputView.meshCollision.collisionApproximation == ImmediateNode::kCollisionApproximationConvexHull.token)
    {
        omni::physx::ConvexMeshCookingParams convexMeshParams;
        convexMeshParams.signScale = meshInputView.meshView.signScale;
        iCookingService->requestConvexMeshCookedData(nullptr, cookingRequest, convexMeshParams);
    }
    else
    {
        omni::physx::TriangleMeshCookingParams triangleParams;
        iCookingService->requestTriangleMeshCookedData(nullptr, cookingRequest, triangleParams);
    }
    return resultIsValid;
}

template <typename PhysxType, typename MemoryCache, typename CookingLambdaFunction, typename CreateLambdaFunction>
bool ImmediateMeshCache::queryApproximationGeneric(MeshDefinitionView request,
                                                   PhysxType*& outMesh,
                                                   MeshFacesToTrianglesMapping*& outMeshMapping,
                                                   MemoryCache& memoryCache,
                                                   carb::tasking::MutexWrapper& memoryMutex,
                                                   CookingLambdaFunction cookMeshLambda,
                                                   CreateLambdaFunction createObjectLambda)
{
    {
        lock_guard lock(mTriangulationCache.mutex);
        auto it = mTriangulationCache.cache.find(request.meshHashes.meshKey);
        if (it != mTriangulationCache.cache.end())
        {
            outMeshMapping = it->second.get();
        }
    }
    {
        // Trying lookup from memory cache
        lock_guard lock(memoryMutex);
        auto found = memoryCache.find(request.meshHashes.cookedDataCRC);
        if (found != memoryCache.end())
        {
            outMesh = found->second.get();
            return true;
        }
    }

    PhysxCookingComputeRequest cookingRequest;
    cookingRequest.dataInputMode = request.meshView.usePrimID ?
                                       omni::physx::PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID :
                                       omni::physx::PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW;
    cookingRequest.primId = request.meshView.primId;
    cookingRequest.primStageId = request.meshView.primStageId;

    cookingRequest.primMeshView = request.meshView.cookingMeshView;
    cookingRequest.primMeshMetersPerUnit = request.meshCollision.metersPerUnit;

    cookingRequest.mode = omni::physx::PhysxCookingComputeRequest::eMODE_REQUEST_COOKED_DATA;
    cookingRequest.meshKey = request.meshHashes.meshKey;
    using Options = omni::physx::PhysxCookingComputeRequest::Options;
    cookingRequest.options.setFlag(Options::kComputeAsynchronously, false);
    cookingRequest.options.setFlag(Options::kComputeGPUCookingData, false);
    cookingRequest.options.setFlag(Options::kLoadCookedDataFromCache, gEnableLoadSaveFromCache);
    cookingRequest.options.setFlag(Options::kSaveCookedDataToCache, gEnableLoadSaveFromCache);
    // We don't want to read USD at all for thread safety here
    cookingRequest.options.setFlag(Options::kForceDisableUSDAccess, !request.meshView.usePrimID);

    bool resultIsValid = false;
    cookingRequest.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result == PhysxCookingResult::eVALID)
        {
            const double metersPerUnit = request.meshCollision.metersPerUnit;
            const PxTolerancesScale tolerancesScale(float(1.0 / metersPerUnit), float(10.0 / metersPerUnit));
            bool res;
            {
                lock_guard lock(mPhysicsMutex);
                res = ensurePxPhysics(tolerancesScale);
            }
            if (res)
            {
                outMesh = createObjectLambda(result, outMeshMapping);
                request.meshHashes.meshKey = result.meshKey;
                request.meshHashes.cookedDataCRC = result.cookedDataCRC;
                resultIsValid = outMesh != nullptr;
            }
        }
    };

    cookMeshLambda(cookingRequest);
    if (resultIsValid)
    {
        // If a mesh got created, store it in the memory cache
        lock_guard lock(memoryMutex);
        std::unique_ptr<PhysxType, CallReleaseOnPointer> meshPtr =
            std::unique_ptr<PhysxType, CallReleaseOnPointer>(outMesh);

        // Before storing the new physx object in the memory cache, now that we're holding the map mutex
        // we want to check if some other thread was quicker than us, filling the cache with a mesh with same CRC
        auto found = memoryCache.find(request.meshHashes.cookedDataCRC);
        if (found != memoryCache.end())
        {
            outMesh = found->second.get();
            return true;
        }
        memoryCache[request.meshHashes.cookedDataCRC] = std::move(meshPtr);
    }
    return resultIsValid;
}

bool ImmediateMeshCache::queryTriangleApproximationFor(MeshDefinitionView meshDefinitionView,
                                                       PxTriangleMesh*& outMesh,
                                                       MeshFacesToTrianglesMapping*& outMeshMapping)
{
    return queryApproximationGeneric(
        meshDefinitionView, outMesh, outMeshMapping, mTriangleMeshCache.cache, mTriangleMeshCache.mutex,
        [&](PhysxCookingComputeRequest& request) {
            request.dataType = omni::physx::PhysxCookingDataType::eTRIANGLE_MESH;
            request.triangulation.needsTriangleFaceMaterials = outMeshMapping == nullptr;
            omni::physx::TriangleMeshCookingParams triangleMeshCookingParams;
            iCookingService->requestTriangleMeshCookedData(nullptr, request, triangleMeshCookingParams);
        },
        [this](const omni::physx::PhysxCookingComputeResult& result,
               MeshFacesToTrianglesMapping*& triangulatedMesh) -> PxTriangleMesh* {
            if (triangulatedMesh == nullptr)
            {
                triangulatedMesh = insertNewTriangulatedMesh(result.triangulationView, result.meshKey);
            }
            PxDefaultMemoryInputData inData((PxU8*)result.cookedData[0].data, (PxU32)result.cookedData[0].sizeInBytes);
            return mPhysics->createTriangleMesh(inData);
        });
}

bool ImmediateMeshCache::queryConvexApproximationFor(MeshDefinitionView meshDefinitionView,
                                                     PxConvexMesh*& outMesh,
                                                     MeshFacesToTrianglesMapping*& outMeshMapping)
{
    return queryApproximationGeneric(
        meshDefinitionView, outMesh, outMeshMapping, mConvexMeshCache.cache, mConvexMeshCache.mutex,
        [&](PhysxCookingComputeRequest& request) {
            request.dataType = omni::physx::PhysxCookingDataType::eCONVEX_MESH;
            omni::physx::ConvexMeshCookingParams convexMeshParams;
            convexMeshParams.signScale = meshDefinitionView.meshView.signScale;
            iCookingService->requestConvexMeshCookedData(nullptr, request, convexMeshParams);
        },
        [this](const omni::physx::PhysxCookingComputeResult& result,
               MeshFacesToTrianglesMapping*& triangulatedMesh) -> PxConvexMesh* {
            PxDefaultMemoryInputData inData((PxU8*)result.cookedData[0].data, (PxU32)result.cookedData[0].sizeInBytes);
            return mPhysics->createConvexMesh(inData);
        });
}
