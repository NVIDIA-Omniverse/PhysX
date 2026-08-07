// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-10
 */

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

#include <UsdSource.h> // local UsdSource for backend-agnostic mass key vocabulary
#include <omni/physics/parse/IPhysicsSource.h>

using namespace ::physx;
using namespace PXR_NS;

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
    PXR_NS::UsdStageWeakPtr stage;
    AuthoringComputeRigidBodyMass(RigidBodyRequest& rigidBodyRequest) : rigidBodyRequest(rigidBodyRequest)
    {
    }

    virtual bool getRigidBodyShapes(usdparser::ObjectId rbId, usdparser::ObjectIdPathMap& shapes) override
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
            // The collider's source path (keyed by ObjectId); getShapeMassInfo +
            // the mass MassAPI/material reads resolve from it.
            shapes[usdparser::ObjectId(index++)] = coll.first;
        }
        return hasTriggers;
    }

    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(const PXR_NS::SdfPath& path,
                                                                       usdparser::ObjectId objectId) override
    {
        usdparser::PhysxShapeDesc* shapeDesc = rigidBodyRequest.colliders[objectId].second;
        PXR_NS::SdfPath shapeKey = rigidBodyRequest.colliders[objectId].first;
        PhysXType physxType;
        PhysXUsdPhysicsInterface::MassInformation massInfo;
        // Actually compute mass by passing null pxscene and null internalphysxdatabase
        PhysXUsdPhysicsInterface::createShapeOrComputeMass(shapeKey, *shapeDesc, usdparser::kInvalidObjectId, nullptr,
                                                           rigidBodyRequest.stage, nullptr, false, physxType, nullptr,
                                                           &massInfo);
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
    // IsInvalid() only catches expired weak pointers (remnant exists, target dead).
    // A default-constructed/null weak pointer (no remnant) returns IsInvalid() == false,
    // so also use operator bool to detect the no-remnant case before any deref.
    if (rigidBodyRequest.stage.IsInvalid() || !rigidBodyRequest.stage)
    {
        errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE;
        return false;
    }
    const long propertyQueryStageId = PXR_NS::UsdUtilsStageCache::Get().GetId(rigidBodyRequest.stage).ToLongInt();
    AttachedStage* propertyQueryAttachedStage = UsdLoad::getUsdLoad()->getAttachedStage(propertyQueryStageId);
    if (!propertyQueryAttachedStage)
    {
        errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE;
        return false;
    }
    for (auto& shapeDesc : rigidBodyRequest.colliders)
    {
        switch (shapeDesc.second->type)
        {
        case eConvexMeshShape: {
            ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)shapeDesc.second;
            if (!convexDesc->meshPrimKey.valid() || propertyQueryAttachedStage->pathFor(convexDesc->meshPrimKey).IsEmpty())
            {
                errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM;
                return false;
            }
            PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, convexDesc->meshPrimKey, *propertyQueryAttachedStage, true);
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
            if (!convexDecompositionDesc->meshPrimKey.valid() || propertyQueryAttachedStage->pathFor(convexDecompositionDesc->meshPrimKey).IsEmpty())
            {
                errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM;
                return false;
            }
            std::vector<PxConvexMesh*> convexMeshes =
                cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, convexDecompositionDesc->meshPrimKey, *propertyQueryAttachedStage, true);
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
            if (!meshDesc->meshPrimKey.valid() || propertyQueryAttachedStage->pathFor(meshDesc->meshPrimKey).IsEmpty())
            {
                errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM;
                return false;
            }
            PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, meshDesc->meshPrimKey, *propertyQueryAttachedStage, true);
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
        usdparser::ObjectId rbId(0);
        // MassAPI / material reads route through the source (backend-agnostic mass,
        // ADR-0002 M2c-D). This is an authoring query over its own stage, so a
        // local UsdSource provides the key vocabulary; the engine-side shape mass
        // (AuthoringComputeRigidBodyMass::getShapeMassInfo) is unchanged.
        omni::physics::usd::UsdSource massSource(acrbmInterface.stage);
        const omni::physics::parse::ObjectKey bodyKey = massSource.keyFor(rigidBodyRequest.primKey);
        if (!massSource.exists(bodyKey))
        {
            rigidBodyRequest.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM);
            return true;
        }
        usdparser::RigidBodyMass rigidBodyMass =
            usdparser::computeRigidBodyMass(&acrbmInterface, massSource, bodyKey, rbId);
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
                                                                   physxType, nullptr, &massInfo);
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

void PhysXPropertyQueryManager::cancelAllPendingRequests(PhysxPropertyQueryResult::Enum errorCode)
{
    // Fire each request's queryFinishedCallback (via returnWithError) so callers don't hang,
    // then drop the request. Skip the stage-update tick until next queryPrim queues one again.
    for (auto& request : rigidBodyRequests)
    {
        request.returnWithError(errorCode);
    }
    rigidBodyRequests.clear();
    detachFromStageUpdate();
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
    // Scoped query: visits every object under request.primKey (root inclusive).
    // Collected up front; the per-prim work below is a path-keyed object lookup
    // (no source data read).
    std::vector<omni::physics::parse::ObjectKey> descendants;
    if (const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
    {
        src->forEachDescendant(attachedStage.keyFor(request.primKey),
                               [&descendants](omni::physics::parse::ObjectKey k) { descendants.push_back(k); });
    }

    for (const omni::physics::parse::ObjectKey subKey : descendants)
    {
        const PXR_NS::SdfPath subPath = attachedStage.pathFor(subKey);

        if (subPath.IsEmpty() || subPath.IsAbsoluteRootPath())
        {
            continue;
        }

        const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        PxArticulationReducedCoordinate* articulation =
            (PxArticulationReducedCoordinate*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                subPath, ePTArticulation, db, attachedStage);

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
        PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(static_cast<long int>(request.stageId)));

    // !request.stage covers the no-remnant case (Find returned null); IsInvalid covers expired stages.
    if (request.stage.IsInvalid() || !request.stage)
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
        return;
    }
    request.primKey = intToPath(request.primPathId);
    if (!request.stage->GetPrimAtPath(request.primKey).IsValid())
    {
        // A.B. this is ugly but if the path is not valid, we will crash, we need to bypass the refcounting
        std::memset(&request.primKey, 0, sizeof(PXR_NS::SdfPath));        
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
        return;
    }

    if (!request.callbacks.articulationCallback)
    {
        request.callQueryFinished();
        return;
    }

    // Synchronous path for now; parses the whole stage. If already parsed, use the existing data.
    if (usdparser::UsdLoad::getUsdLoad()->getActiveStageId() == request.stageId)
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

    if (!usdparser::UsdLoad::getUsdLoad()->getActiveStageId())
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

    // A different stage is currently attached - we cannot satisfy the query against
    // request.stageId without disturbing it. Fail explicitly so the caller's
    // queryFinishedCallback fires instead of leaving the request hanging.
    request.returnWithError(PhysxPropertyQueryResult::eERROR_RUNTIME);
}

void PhysXPropertyQueryManager::executeQueryRigidBody(RigidBodyRequest& request)
{
    request.stage =
        PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(static_cast<long int>(request.stageId)));

    // !request.stage covers the no-remnant case (Find returned null); IsInvalid covers expired stages.
    if (request.stage.IsInvalid() || !request.stage)
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
        return;
    }
    request.primKey = intToPath(request.primPathId);
    if (!request.stage->GetPrimAtPath(request.primKey).IsValid())
    {
        request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
        return;
    }

    if (usdparser::UsdLoad::getUsdLoad()->getActiveStageId() == request.stageId)
    {
        const usdparser::AttachedStage* attachedStage =
            usdparser::UsdLoad::getUsdLoad()->getAttachedStage(request.stageId);
        if (attachedStage)
        {
            const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
            PxRigidActor* actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                request.primKey, ePTActor, db, *attachedStage);
            if (!actor)
            {
                actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                    request.primKey, ePTLink, db, *attachedStage);
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
                            collResponse.usdPath = asInt(attachedStage->pathFor(shapeRecord.mKey));
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


    request.desc = omni::physx::usdparser::parseRigidBody(request.stageId, request.primKey, request.colliders);

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
               uint64_t primKey,
               PhysxPropertyQueryMode::Enum queryMode,
               const IPhysxPropertyQueryCallback& callbacks)
{
    OmniPhysX::getInstance().getPropertyQueryManager().queryPrim(stageId, primKey, queryMode, callbacks);
}
} // namespace physx
} // namespace omni
