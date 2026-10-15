// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4
// @implements REQ-PARSE-CORE-006
// @covers AC-9

#include "CollisionRepresentation.h"
#include "RuntimeExecution.h"
#include <limits>
#include <mutex>
#include <unordered_map>

#include "OmniPhysX.h"
#include "CookingDataAsync.h"
#include "MeshCache.h"

#include "common/utilities/MemoryUtilities.h"
#include "common/utilities/Defer.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Collision.h"
#include <PxPhysicsAPI.h>

using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

namespace omni
{
namespace physx
{

namespace
{
struct CollisionTaskCompletion
{
    ~CollisionTaskCompletion();
    uintptr_t token = 0;
    bool completed = false; // protected by gCollisionTaskMutex
};
struct CollisionTaskEntry
{
    AttachHandle attach;
    PhysxCookingOperationHandle nativeHandle;
};
std::mutex gCollisionTaskMutex;
std::unordered_map<uintptr_t, CollisionTaskEntry> gCollisionTasks;
uintptr_t gNextCollisionTaskToken = 1;

CollisionTaskCompletion::~CollisionTaskCompletion()
{
    // Superseded requests can drop their callback without invoking it.
    std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
    gCollisionTasks.erase(token);
}

std::shared_ptr<CollisionTaskCompletion> createCollisionTaskCompletion()
{
    std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
    if (gNextCollisionTaskToken == std::numeric_limits<uintptr_t>::max())
        return {};
    std::shared_ptr<CollisionTaskCompletion> completion = std::make_shared<CollisionTaskCompletion>();
    completion->token = gNextCollisionTaskToken++;
    return completion;
}
void completeCollisionTask(const std::shared_ptr<CollisionTaskCompletion>& completion)
{
    std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
    completion->completed = true;
    gCollisionTasks.erase(completion->token);
}
} // namespace

void retireCollisionRepresentationTasks(AttachHandle attach)
{
    std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
    for (auto it = gCollisionTasks.begin(); it != gCollisionTasks.end();)
    {
        if (it->second.attach == attach)
            it = gCollisionTasks.erase(it);
        else
            ++it;
    }
}

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
            meshes = omni::physx::getMeshCache()->getConvexDecomposition(cookedDataCRC);
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
        for (size_t index = 0; index < meshes.size(); ++index)
        {
            PxConvexMesh* mesh = meshes[index];
            ConvexMeshData& meshData = convexMeshDatas[index];
            if (!getMeshCache()->getConvexMeshData(mesh, meshData))
                return false;
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
    RuntimeExecutionScope operation(request.attachHandle);
    if (!operation)
        return { nullptr, RuntimeResult::eBusy };
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStageByHandle(request.attachHandle);
    if (!attachedStage)
        return { nullptr, RuntimeResult::eInvalidAttachment };

    // The same guard applies to immediate and pumped/asynchronous completion.
    // This carries recursion state, never an implicit attachment selection.
    onResult = [callback = std::move(onResult)](PhysxCollisionRepresentationResult::Enum status, const ResultType& result) {
        RuntimeCallbackScope callbackScope;
        callback(status, result);
    };

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

    omni::physx::IPhysxCookingService* cookingService =
        OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
    {
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_NOT_READY, result);
        return { nullptr };
    }
    //----------------------------------------------------------------------------------
    // 1. Parse the collision API
    //----------------------------------------------------------------------------------
    const omni::physics::parse::ObjectKey collisionKey{ request.collisionPrimId };
    ScannedCollision collision = parseCollision(*attachedStage, collisionKey, collisionKey);
    PhysxShapeDesc* collisionDesc = collision.get();
    if (collisionDesc == nullptr)
    {
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_PARSING, result);
        return { nullptr };
    }

    omni::physx::PhysxCookingAsyncContext asyncContext = cookingDataAsync->getCookingAsyncContext(*attachedStage);


    //----------------------------------------------------------------------------------
    // 2. Compute meshKey and CookedDataCRC
    //----------------------------------------------------------------------------------

    // Setup the cooking request with shared parameters between
    PhysxCookingComputeRequest cookingRequest;
    cookingRequest.primId = request.collisionPrimId;
    // Backing stage metadata is diagnostic correlation only; source data and task
    // lifetime always come from the selected attachment.
    cookingRequest.primStageId = attachedStage->getStageId();
    cookingRequest.attachHandle = attachedStage->getAttachHandle();
    const bool async = request.options.hasFlag(PhysxCollisionRepresentationRequest::Options::kComputeAsynchronously);
    cookingRequest.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);

    cookingRequest.mode = PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;

    // Cooking CRC computations are always sync, but let's make it explicit
    cookingRequest.options.setFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);

    SourceMeshGeometryScope sourceGeomScope;
    {
        omni::physics::parse::ObjectKey meshKey;
        switch (collisionDesc->type)
        {
        case eConvexMeshShape:
            meshKey = static_cast<ConvexMeshPhysxShapeDesc*>(collisionDesc)->meshPrimKey;
            break;
        case eConvexMeshDecompositionShape:
            meshKey = static_cast<ConvexMeshDecompositionPhysxShapeDesc*>(collisionDesc)->meshPrimKey;
            break;
        default:
            // Not a mesh-backed approximation type -- the switch below reports
            // eRESULT_ERROR_UNSUPPORTED_APPROXIMATION for it; nothing to read here.
            break;
        }
        // Unreadable input fails loudly, before any submission, rather than falling back to the
        // cooking service resolving primStageId/primId itself (that mechanism no longer exists --
        // see REQ-COOK-SOURCE-001). `attachedStage` is non-null here whenever `collisionDesc` is
        // (the foreign-stage arm above populates it too), so this only guards a stage that
        // disappeared from the cache between the parse above and this read.
        if (meshKey.valid() &&
            (!attachedStage || !fillCookingMeshViewFromSource(cookingRequest, sourceGeomScope, *attachedStage, meshKey)))
        {
            // textFor() never needs an SdfPath (unlike the collisionPath.GetText() this replaced).
            CARB_LOG_ERROR("requestConvexCollisionRepresentation: could not read source geometry for prim %s",
                           attachedStage ? attachedStage->textFor(collisionKey) : "<unknown>");
            onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_COOKING_FAILED, result);
            return { nullptr };
        }
    }

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
    const std::shared_ptr<CollisionTaskCompletion> completion = createCollisionTaskCompletion();
    if (!completion)
    {
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_NOT_READY, result);
        return { nullptr };
    }
    std::shared_ptr<ContextType> context = std::make_shared<ContextType>();
    context->request = request;
    context->onResult = onResult;
    cookingRequest.mode = PhysxCookingComputeRequest::eMODE_REQUEST_COOKED_DATA;
    cookingRequest.options.setFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously, async);
    cookingRequest.meshKey = meshKey;
    cookingRequest.onFinished = [context, completion](const omni::physx::PhysxCookingComputeResult& cookingResult) {
        completeCollisionTask(completion);
        if (cookingResult.result == PhysxCookingResult::eVALID)
        {
            //----------------------------------------------------------------------------------
            // 6. Return results and insert cooked data in runtime mesh cache
            //----------------------------------------------------------------------------------
            context->returnNewlyComputedRepresentation(cookingResult);
        }
        else
        {
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
        onResult(PhysxCollisionRepresentationResult::eRESULT_ERROR_UNSUPPORTED_APPROXIMATION, result);
        return { nullptr };
    }

    if (handle.handle)
    {
        std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
        if (!completion->completed)
        {
            gCollisionTasks.emplace(completion->token, CollisionTaskEntry{request.attachHandle, handle.handle});
            handle.handle = reinterpret_cast<void*>(completion->token);
        }
        else
            handle.handle = nullptr;
    }
    return handle;
}

PhysxCollisionRepresentationTask requestConvexCollisionRepresentation(
    const PhysxCollisionRepresentationRequest& request, PhysxCollisionRepresentationConvexResult::CallbackType onResult)
{
    return requestCollisionRepresentation<PhysxCollisionRepresentationConvexResult, CollisionRepresentationConvex>(
        request, onResult);
}

bool isCollisionRepresentationTaskLive(PhysxCollisionRepresentationTask task)
{
    std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
    return gCollisionTasks.count(reinterpret_cast<uintptr_t>(task.handle)) != 0;
}

RuntimeResult cancelCollisionRepresentationTask(PhysxCollisionRepresentationTask task, bool invokeCallbackAnyway)
{
    if (getRuntimeCallbackDepth())
        return RuntimeResult::eBusy;
    if (!task.handle)
        return RuntimeResult::eSuccess;
    CollisionTaskEntry entry{};
    {
        std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
        const auto found = gCollisionTasks.find(reinterpret_cast<uintptr_t>(task.handle));
        if (found == gCollisionTasks.end())
            return RuntimeResult::eSuccess;
        entry = found->second;
    }
    RuntimeExecutionScope operation(entry.attach, true);
    if (!operation)
        return RuntimeResult::eBusy;
    {
        std::lock_guard<std::mutex> lock(gCollisionTaskMutex);
        if (!gCollisionTasks.erase(reinterpret_cast<uintptr_t>(task.handle)))
            return RuntimeResult::eSuccess;
    }
    if (!UsdLoad::getUsdLoad()->getAttachedStageByHandle(entry.attach))
        return RuntimeResult::eSuccess;
    omni::physx::IPhysxCookingService* cookingService =
        OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (cookingService)
    {
        cookingService->cancelTask(entry.nativeHandle, invokeCallbackAnyway);
    }
    return RuntimeResult::eSuccess;
}
} // namespace physx
} // namespace omni
