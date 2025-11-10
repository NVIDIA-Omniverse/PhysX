// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physx/PhysxUsd.h>
#include <PxPhysicsAPI.h>
#include <common/utilities/Utilities.h>
#include <common/foundation/TypeCast.h>

#include "OmniPhysX.h"
#include "Setup.h"
#include "PhysXPropertyQuery.h"
#include "ObjectDataQuery.h"
#include "CookingDataAsync.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Mass.h"
#include "usdLoad/AttachedStage.h"

using namespace ::physx;
using namespace pxr;

extern void forceLoadPhysicsFromUSD();
extern void releasePhysicsObjects();
extern bool physxSimulationAttach(long stageId);
extern void physxSimulationDetach();

namespace omni
{
namespace physx
{

bool BaseRequest::isExpired()
{
    if (callbacks.timeoutMs < 0)
    {
        return false;
    }
    else
    {
        return timeoutTimer.getElapsedTime<int64_t>() > callbacks.timeoutMs;
    }
}

void BaseRequest::callQueryFinished()
{
    releaseParsedData();
    if (callbacks.queryFinishedCallback)
    {
        callbacks.queryFinishedCallback(callbacks.userData);
    }
}

void BaseRequest::returnWithError(PhysxPropertyQueryResult::Enum queryResult)
{
    PhysxPropertyQueryRigidBodyResponse rbResponse;
    rbResponse.result = queryResult;
    sendRigidBodyResponse(rbResponse);
    PhysxPropertyQueryColliderResponse collResponse;
    collResponse.result = queryResult;
    sendColliderResponse(collResponse);
    callQueryFinished();
}

void RigidBodyRequest::releaseParsedData()
{
    using namespace omni::physx::usdparser;
    // NOTE: Placement delete is more correct as ICE_FREE will not call destructors of non trivial fields
    // (std::vector<*> etc.)
    ICE_PLACEMENT_DELETE(desc, PhysxRigidBodyDesc);
    desc = nullptr;
    for (auto coll : colliders)
    {
        // NOTE: This one may leak memory anyway for non trivial fields (example std::vectors)
        // declared in children (as PhysxShapeDesc destructor is not virtual)
        ICE_PLACEMENT_DELETE(coll.second, PhysxShapeDesc);
    }
    colliders.clear();
}

void RigidBodyRequest::sendRigidBodyResponse(const PhysxPropertyQueryRigidBodyResponse& response)
{
    if (callbacks.rigidBodyCallback)
    {
        callbacks.rigidBodyCallback(response, callbacks.userData);
    }
}

void RigidBodyRequest::sendColliderResponse(const PhysxPropertyQueryColliderResponse& response)
{
    if (callbacks.colliderCallback)
    {
        callbacks.colliderCallback(response, callbacks.userData);
    }
}

void ArticulationRequest::sendArticulationResponse(const PhysxPropertyQueryArticulationResponse& response)
{
    if (callbacks.articulationCallback)
    {
        callbacks.articulationCallback(response, callbacks.userData);
    }
}

struct AuthoringComputeRigidBodyMass : public usdparser::AbstractComputeRigidBodyMass
{
    RigidBodyRequest& rigidBodyRequest;
    pxr::UsdStageWeakPtr stage;
    pxr::UsdGeomXformCache xFormCache;
    AuthoringComputeRigidBodyMass(RigidBodyRequest& rigidBodyRequest) : rigidBodyRequest(rigidBodyRequest)
    {
    }

    virtual bool getRigidBodyShapes(usdparser::ObjectId rbId, usdparser::ObjectIdUsdPrimMap& shapes) override
    {
        shapes.clear();
        size_t index = 0;
        bool hasTriggers = false;
        for (auto& coll : rigidBodyRequest.colliders)
        {
            auto shapeDesc = coll.second;
            if (shapeDesc->isTrigger && shapeDesc->collisionEnabled)
            {
                hasTriggers = true;
                index++; // Advance the index as it's used by getShapeMassInfo
                continue;
            }
            pxr::SdfPath colliderPath;
            shapes[usdparser::ObjectId(index++)] = stage->GetPrimAtPath(coll.first);
        }
        return hasTriggers;
    }

    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(const pxr::SdfPath& path,
                                                                       usdparser::ObjectId objectId) override
    {
        usdparser::PhysxShapeDesc* shapeDesc = rigidBodyRequest.colliders[objectId].second;
        pxr::SdfPath shapePath = rigidBodyRequest.colliders[objectId].first;
        PhysXType physxType;
        PhysXUsdPhysicsInterface::MassInformation massInfo;
        // Actually compute mass by passing null pxscene and null internalphysxdatabase
        PhysXUsdPhysicsInterface::createShapeOrComputeMass(shapePath, *shapeDesc, usdparser::kInvalidObjectId, nullptr,
                                                           rigidBodyRequest.stage, nullptr, false, physxType, nullptr,
                                                           &massInfo, &xFormCache);
        return massInfo;
    }
};

bool PhysXPropertyQueryManager::allCollidersHaveBeenComputed(const RigidBodyRequest& rigidBodyRequest,
                                                             PhysxPropertyQueryResult::Enum& errorCode)
{
    errorCode = PhysxPropertyQueryResult::eVALID;
    using namespace omni::physx::usdparser;
    cookingdataasync::CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync)
    {
        // There is a very short window where CookingDataAsync may be nullptr (see OM-99521). We can't reproduce it 
        // precisely but returning true here will just allow for checking again at later time. 
        // Worst case, if this is still nullptr after some time we'll return eERROR_TIMEOUT.
        return false;
    }
    if (rigidBodyRequest.stage.IsInvalid())
    {
        errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE;
        return false;
    }
    for (auto& shapeDesc : rigidBodyRequest.colliders)
    {
        // We should check if shape has a collider setup
        switch (shapeDesc.second->type)
        {
        case eConvexMeshShape: {
            ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)shapeDesc.second;
            pxr::UsdPrim usdPrim = rigidBodyRequest.stage->GetPrimAtPath(convexDesc->meshPath);
            if (!usdPrim.IsValid())
            {
                errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM;
                return false;
            }
            PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, usdPrim, true);
            if (convexMesh == nullptr)
            {
                // TODO: We can't distinguish between an unfinished and a failed cooking operation here
                return false;
            }
        }
        break;
        case eConvexMeshDecompositionShape: {
            ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc =
                (ConvexMeshDecompositionPhysxShapeDesc*)shapeDesc.second;
            pxr::UsdPrim usdPrim = rigidBodyRequest.stage->GetPrimAtPath(convexDecompositionDesc->meshPath);
            if (!usdPrim.IsValid())
            {
                errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM;
                return false;
            }
            std::vector<PxConvexMesh*> convexMeshes =
                cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, usdPrim, true);
            if (convexMeshes.empty())
            {
                // TODO: We can't distinguish between an unfinished and a failed cooking operation here
                return false;
            }
        }
        break;
        case eTriangleMeshShape: {
            // Not sure if we should have this case as we can't use triangle mesh for dynamics
            TriangleMeshPhysxShapeDesc* meshDesc = (TriangleMeshPhysxShapeDesc*)shapeDesc.second;
            pxr::UsdPrim usdPrim = rigidBodyRequest.stage->GetPrimAtPath(meshDesc->meshPath);
            if (!usdPrim.IsValid())
            {
                errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM;
                return false;
            }
            PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, usdPrim, true);
            if (triMesh == nullptr)
            {
                return false;
            }
        }
        break;
        }
    }
    return true;
}

bool PhysXPropertyQueryManager::rigidBodyRequestHasFinished(RigidBodyRequest& rigidBodyRequest)
{
    // If all bodies have computed colliders
    PhysxPropertyQueryResult::Enum queryResult;
    if (rigidBodyRequest.isExpired())
    {
        rigidBodyRequest.returnWithError(PhysxPropertyQueryResult::eERROR_TIMEOUT);
        return true; // return true will remove this request from the list
    }
    else if (allCollidersHaveBeenComputed(rigidBodyRequest, queryResult))
    {
        AuthoringComputeRigidBodyMass acrbmInterface(rigidBodyRequest);
        acrbmInterface.stage = rigidBodyRequest.stage;
        pxr::UsdPrim usdPrim = acrbmInterface.stage->GetPrimAtPath(rigidBodyRequest.primPath);
        if (!usdPrim)
        {
            rigidBodyRequest.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM);
            return true;
        }
        usdparser::ObjectId rbId(0);
        usdparser::RigidBodyMass rigidBodyMass =
            usdparser::computeRigidBodyMass(&acrbmInterface, acrbmInterface.stage, usdPrim, rbId);
        PhysxPropertyQueryRigidBodyResponse rbResponse;
        rbResponse.usdStageId = rigidBodyRequest.stageId;
        rbResponse.usdPath = rigidBodyRequest.primPathId;
        rbResponse.mass = rigidBodyMass.mass;
        rbResponse.inertia = rigidBodyMass.inertia;
        rbResponse.centerOfMass = rigidBodyMass.centerOfMass;
        rbResponse.principalAxes = rigidBodyMass.principalAxes;
        rigidBodyRequest.sendRigidBodyResponse(rbResponse);
        if (rigidBodyRequest.callbacks.colliderCallback != nullptr)
        {
            for (auto& coll : rigidBodyRequest.colliders)
            {
                PhysxPropertyQueryColliderResponse collResponse;
                collResponse.usdStageId = rigidBodyRequest.stageId;
                collResponse.usdPath = asInt(coll.first);
                PhysXUsdPhysicsInterface::MassInformation massInfo;
                PhysXType physxType;
                PhysXUsdPhysicsInterface::createShapeOrComputeMass(coll.first, *coll.second, usdparser::kInvalidObjectId,
                                                                   nullptr, rigidBodyRequest.stage, nullptr, false,
                                                                   physxType, nullptr, &massInfo, &acrbmInterface.xFormCache);
                collResponse.aabbLocalMin = massInfo.aabbLocalMin;
                collResponse.aabbLocalMax = massInfo.aabbLocalMax;
                collResponse.volume = massInfo.volume;
                collResponse.localPos = massInfo.localPos;
                collResponse.localRot = massInfo.localRot;
                rigidBodyRequest.sendColliderResponse(collResponse);
            }
        }
        rigidBodyRequest.callQueryFinished();
        return true; // return true will remove this request from the list
    }
    else if (queryResult != PhysxPropertyQueryResult::eVALID)
    {
        rigidBodyRequest.returnWithError(queryResult);
        return true; // return true will remove this request from the list
    }
    return false;
}

void PhysXPropertyQueryManager::updateQueuedRequests()
{
    for (auto it = rigidBodyRequests.begin(); it != rigidBodyRequests.end();)
    {
        if (rigidBodyRequestHasFinished(*it))
        {
            it = rigidBodyRequests.erase(it);
        }
        else
        {
            ++it;
        }
    }
    if (rigidBodyRequests.empty())
    {
        detachFromStageUpdate();
    }
}

void PhysXPropertyQueryManager::attachToStageUpdate()
{
     OmniPhysX::getInstance().getStageUpdate().attachPropertyQueryStageUpdate();
}

void PhysXPropertyQueryManager::detachFromStageUpdate()
{
    OmniPhysX::getInstance().getStageUpdate().detachPropertyQueryStageUpdate();
}

void PhysXPropertyQueryManager::processArticulation(const usdparser::AttachedStage& attachedStage, ArticulationRequest& request)
{   
    const pxr::UsdPrim topPrim = request.stage->GetPrimAtPath(request.primPath);
    const UsdPrimRange primRange = UsdPrimRange(topPrim);

    for (UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
    {
        const UsdPrim& subPrim = *iter;

        if (!subPrim.IsValid() || subPrim.IsPseudoRoot())
        {
            continue;
        }

        const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        PxArticulationReducedCoordinate* articulation =
            (PxArticulationReducedCoordinate*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                subPrim.GetPrimPath(), ePTArticulation, db, attachedStage);

        if (articulation)
        {
            const PxU32 numLinks = articulation->getNbLinks();
            std::vector<PhysxPropertyQueryArticulationLink> articulationLinks;
            articulationLinks.resize(numLinks);
            PxArticulationLink* link = nullptr;            
            for (PxU32 i = 0; i < numLinks; i++)
            {
                uint64_t linkName = 0;
                uint64_t jointName = 0;
                uint32_t jointDof = 0;
                articulation->getLinks(&link, 1, i);
                if (link->getName())
                {
                    const SdfPath linkSdfName(link->getName());
                    linkName = asInt(linkSdfName);
                }

                PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                if (joint && joint->getName())
                {
                    const SdfPath jointSdfName(joint->getName());
                    jointName = asInt(jointSdfName);
                    jointDof = link->getInboundJointDof();
                }

                articulationLinks[link->getLinkIndex()] = { linkName, jointName, jointDof };
            }

            PhysxPropertyQueryArticulationResponse response;
            response.usdStageId = request.stageId;
            response.usdPath = request.primPathId;
            response.links = articulationLinks;            
            request.sendArticulationResponse(response);
        }
    }
}

void PhysXPropertyQueryManager::executeQueryArticulation(ArticulationRequest& request)
{
    request.stage =
        pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(static_cast<long int>(request.stageId)));

    if (request.stage.IsInvalid())
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
        return;
    }
    request.primPath = intToPath(request.primPathId);
    if (!request.stage->GetPrimAtPath(request.primPath).IsValid())
    {
        // A.B. this is ugly but if the path is not valid, we will crash, we need to bypass the refcounting
        std::memset(&request.primPath, 0, sizeof(pxr::SdfPath));        
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
        return;
    }

    if (!request.callbacks.articulationCallback)
    {
        request.callQueryFinished();
        return;
    }

    // Synchronous path for now, parse also whole stage for now
    // If already parsed get the data directly
    if (OmniPhysX::getInstance().getStageId() == request.stageId)
    {
        const usdparser::AttachedStage* attachedStage =
            usdparser::UsdLoad::getUsdLoad()->getAttachedStage(request.stageId);
        if (attachedStage && !attachedStage->getPrimUpdateMap().isEmptyScene())
        {
            processArticulation(*attachedStage, request);

            request.callQueryFinished();
            return;
        }
        else
        {
            forceLoadPhysicsFromUSD();
            processArticulation(*attachedStage, request);
            releasePhysicsObjects();

            request.callQueryFinished();
            return;
        }
    }

    if (!OmniPhysX::getInstance().getStageId())
    {
        physxSimulationAttach(request.stageId);
        const usdparser::AttachedStage* attachedStage =
            usdparser::UsdLoad::getUsdLoad()->getAttachedStage(request.stageId);
        if (attachedStage && !attachedStage->getPrimUpdateMap().isEmptyScene())
        {
            processArticulation(*attachedStage, request);
        }
        physxSimulationDetach();

        request.callQueryFinished();
        return;
    }
}

void PhysXPropertyQueryManager::executeQueryRigidBody(RigidBodyRequest& request)
{
    request.stage =
        pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(static_cast<long int>(request.stageId)));

    if (request.stage.IsInvalid())
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
        return;
    }
    request.primPath = intToPath(request.primPathId);
    if (!request.stage->GetPrimAtPath(request.primPath).IsValid())
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
        return;
    }

    if (OmniPhysX::getInstance().getStageId() == request.stageId)
    {
        const usdparser::AttachedStage* attachedStage =
            usdparser::UsdLoad::getUsdLoad()->getAttachedStage(request.stageId);
        if (attachedStage)
        {
            const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
            PxRigidActor* actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                request.primPath, ePTActor, db, *attachedStage);
            if (!actor)
            {
                actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                    request.primPath, ePTLink, db, *attachedStage);
            }

            if (actor)
            {
                // We are in simulation, with the same stage id, so let's query info from runtime
                PxRigidBody* rb = actor->is<PxRigidBody>();
                if (rb)
                {
                    float mass = rb->getMass();
                    const PxVec3 rbInertia = rb->getMassSpaceInertiaTensor();
                    const PxVec3 rbCom = rb->getCMassLocalPose().p;
                    const PxQuat rbOrient = rb->getCMassLocalPose().q;
                    PhysxPropertyQueryRigidBodyResponse response;
                    response.usdStageId = request.stageId;
                    response.usdPath = request.primPathId;
                    response.mass = mass;
                    response.centerOfMass = { rbCom.x, rbCom.y, rbCom.z };
                    response.inertia = { rbInertia.x, rbInertia.y, rbInertia.z };
                    response.principalAxes = fromPhysX(rbOrient);
                    request.sendRigidBodyResponse(response);
                    if (request.callbacks.colliderCallback != nullptr)
                    {
                        const PxU32 numShapes = rb->getNbShapes();
                        std::unordered_set<internal::CompoundShape*> alreadyVisitedShapes;
                        for (PxU32 idx = 0; idx < numShapes; ++idx)
                        {
                            PxShape* shape = nullptr;
                            rb->getShapes(&shape, 1, idx);
                            PhysxPropertyQueryColliderResponse collResponse;
                            collResponse.usdStageId = request.stageId;
                            const size_t index = (size_t)shape->userData;
                            auto shapeRecord = db.getRecords()[index];
                            collResponse.usdPath = asInt(shapeRecord.mPath);
                            PhysXUsdPhysicsInterface::MassInformation* massInfo = nullptr;
                            switch(shapeRecord.mType)
                            {
                                case ePTShape:
                                {
                                    internal::InternalShape* intShape = (internal::InternalShape*)shapeRecord.mInternalPtr;
                                    massInfo = &intShape->mMassInfo;
                                }
                                break;
                                case ePTCompoundShape:
                                {
                                    internal::CompoundShape* shape = (internal::CompoundShape*)shapeRecord.mPtr;
                                    if(alreadyVisitedShapes.find(shape) != alreadyVisitedShapes.end())
                                        continue;
                                    alreadyVisitedShapes.insert(shape);
                                    massInfo = &shape->mMassInfo;
                                }
                                break;
                                default:
                                CARB_LOG_ERROR("Shape (%s) has incorrect shape record type", shape->getName());
                                break;
                            }
                            if(massInfo)
                            {
                                collResponse.aabbLocalMin = massInfo->aabbLocalMin;
                                collResponse.aabbLocalMax = massInfo->aabbLocalMax;
                                collResponse.volume = massInfo->volume;
                                collResponse.localPos = massInfo->localPos;
                                collResponse.localRot = massInfo->localRot;
                                request.sendColliderResponse(collResponse);
                            }
                            else
                            {
                                request.returnWithError(PhysxPropertyQueryResult::eERROR_RUNTIME);
                            }
                        }
                    }
                    request.callQueryFinished();
                    return;
                }
            }
        }
    }


    request.desc = omni::physx::usdparser::parseRigidBody(request.stageId, request.primPath, request.colliders);

    if (request.desc == nullptr)
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_PARSING);
        return;
    }
    if (!rigidBodyRequestHasFinished(request))
    {
        if (request.callbacks.timeoutMs >= 0)
        {
            request.timeoutTimer.start();
        }
        rigidBodyRequests.push_back(request);
        attachToStageUpdate();
    }
}

void PhysXPropertyQueryManager::queryPrim(uint64_t stageId,
                                          uint64_t primPathId,
                                          PhysxPropertyQueryMode::Enum queryMode,
                                          const IPhysxPropertyQueryCallback& callbacks)
{
    switch (queryMode)
    {
    case PhysxPropertyQueryMode::eQUERY_RIGID_BODY_WITH_COLLIDERS: {
        RigidBodyRequest request;
        request.callbacks = callbacks;
        request.stageId = stageId;
        request.primPathId = primPathId;
        executeQueryRigidBody(request);
    }
    break;
    case PhysxPropertyQueryMode::eQUERY_ARTICULATION: {
        ArticulationRequest request;
        request.callbacks = callbacks;
        request.stageId = stageId;
        request.primPathId = primPathId;
        executeQueryArticulation(request);
    }
    break;
    default:
        BaseRequest request;
        request.callbacks = callbacks;
        request.stageId = stageId;
        request.primPathId = primPathId;
        request.returnWithError(PhysxPropertyQueryResult::eERROR_UNKNOWN_QUERY_MODE);
        break;
    }
}

void queryPrim(uint64_t stageId,
               uint64_t primPath,
               PhysxPropertyQueryMode::Enum queryMode,
               const IPhysxPropertyQueryCallback& callbacks)
{
    OmniPhysX::getInstance().getPropertyQueryManager().queryPrim(stageId, primPath, queryMode, callbacks);
}
} // namespace physx
} // namespace omni
