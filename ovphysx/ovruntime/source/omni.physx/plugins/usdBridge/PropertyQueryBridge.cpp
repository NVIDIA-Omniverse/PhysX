// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

// pxr-free PhysXPropertyQuery bridge (ADR-0027, seam #2): one source-agnostic translation
// unit -- `omni.physx` compiles exactly one way, and USD parsing is a runtime-installed
// backend rather than a compile fork.
//
// omni::physx::queryPrim / PhysXPropertyQueryManager is genuinely Kit/USD-authoring-only:
// arbitrary live-stage-by-id property/mass query ("select a prim in the viewport, compute
// its mass without simulating"). It is never reachable from ovphysx's own ovstage attach
// path, so USD authoring is intrinsic to it, not incidental.
//
// The PhysXPropertyQueryManagerImpl stays in omni.physx: it consumes omni.physx runtime
// that cannot move down to the omni.physics.usd layer -- CookingDataAsync, the mass compute
// (createShapeOrComputeMass / computeRigidBodyMass), InternalPhysXDatabase, live PhysX actor
// reads and the stage-update tick. Only the pxr *seam* work routes out, through
// omni::physics::parse::usdReparse() (the loadable USD backend) plus the pxr-free
// AttachedStage / IPhysicsSource vocabulary:
//   - The single-prim USD re-parse is assembled from usdReparse(): resolveStageToHandle()
//     binds a per-request minting AttachedStage over the foreign stage (the pxr-free
//     analogue of attachForeignStage's setStage), and openScopedStage()/scan()/source()
//     produce the source-agnostic ScannedStage (the UsdReparseSessionPtr owner closes the
//     session, exception-safe). The scanned shapes are then re-keyed into
//     the request's AttachedStage by the (unconditional) translateScannedShape -- exactly
//     the mechanism parseRigidBodyNative uses with its transient native scanStage result.
//   - Path/ObjectKey resolution routes through IPhysicsSource::sourceKeyToString/findByPath
//     and AttachedStage::keyFor/textFor.
//   - The public SdfPath-bits fields (IPhysxPropertyQuery request primPath / response usdPath
//     / ArticulationLink rigidBody+joint) are the deliberately-retained legacy asInt(SdfPath)
//     encoding; they cross the seam through IUsdReparse::legacyPathBitsToString (decode) and
//     stringToLegacyPathBits (encode).
//
// Production behaviour: usdReparse() is null (nothing reparses USD -- ovstage is the only
// backend), so mImpl stays null and every query fails synchronously with
// eERROR_INVALID_USD_STAGE, draining/cancelling as no-ops. The test loader installs the USD
// backend process-lifetime before OmniPhysX is constructed (ADR-0027), so in the developer
// build mImpl is live.

#include "OmniPhysX.h"
#include "PhysXPropertyQuery.h"

#include <PxPhysicsAPI.h>

#include <common/foundation/Allocator.h>     // ICE_PLACEMENT_DELETE
#include <common/foundation/CarbPhysXCast.h> // fromPhysX(PxQuat)

#include "Setup.h"
#include "ObjectDataQuery.h"
#include "CookingDataAsync.h"
#include "usdInterface/UsdInterface.h"       // PhysXUsdPhysicsInterface
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Mass.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/IceDescriptorAllocator.h"
#include "usdBridge/StageBridge.h"           // translateScannedShape (pxr-free, unconditional)

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ScanBackend.h>  // ScanOptions
#include <omni/physics/parse/ScannedStage.h>
#include <omni/physics/parse/UsdReparse.h>   // IUsdReparse / usdReparse()

#include <carb/tasking/TaskingUtils.h>       // carb::tasking::MutexWrapper

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace ::physx;

// Defined at global scope elsewhere (PhysX.cpp); pxr-free signatures. Used by the
// articulation synchronous fallback below.
extern void forceLoadPhysicsFromUSD();
extern void releasePhysicsObjects();
extern bool physxSimulationAttach(long stageId);
extern void physxSimulationDetach();

namespace omni
{
namespace physx
{

namespace
{
namespace parse = ::omni::physics::parse;

// The active USD reparse backend, or null in production (no USD installed). Non-null for the
// whole lifetime of a live Impl (the Impl is created only when this is set).
parse::IUsdReparse* reparseBackend()
{
    return parse::usdReparse();
}
} // namespace

// Per-request state. Lives entirely in this TU now: PhysXPropertyQuery.h is pxr-free and
// exposes PhysXPropertyQueryManager through an opaque Impl. No pxr type appears here -- the
// former UsdStageWeakPtr/SdfPath members are replaced by the stage id, the incoming legacy
// path-bits id, and the decoded path string.
struct BaseRequest
{
    carb::extras::Timer timeoutTimer;

    uint64_t stageId = 0;
    uint64_t primPathId = 0; // legacy asInt(SdfPath) bits carried by the public interface
    std::string primPath;    // primPathId decoded to a path string (seam legacyPathBitsToString)
    IPhysxPropertyQueryCallback callbacks;
    virtual ~BaseRequest()
    {
    }
    bool isExpired();
    void callQueryFinished();
    void returnWithError(PhysxPropertyQueryResult::Enum queryResult);
    virtual void releaseParsedData()
    {
    }
    virtual void sendRigidBodyResponse(const PhysxPropertyQueryRigidBodyResponse&)
    {
    }
    virtual void sendColliderResponse(const PhysxPropertyQueryColliderResponse&)
    {
    }
};

struct RigidBodyRequest : public BaseRequest
{
    omni::physics::parse::PhysxRigidBodyDesc* desc = { nullptr };
    // Collider path string + desc. The path string is the collider's source-native path,
    // used to encode the response usdPath (seam) and to re-mint the collider ObjectKey against
    // the request's parse source (parseStage->keyFor / source->findByPath).
    std::vector<std::pair<std::string, omni::physics::parse::PhysxShapeDesc*>> colliders;

    // The single minting authority for this request. `desc`/`colliders` carry ObjectKeys
    // (meshPrimKey, materials, ...) minted against this AttachedStage's source, and an
    // ObjectKey resolves against no other instance -- the source folds a process-unique
    // generation into every key it mints (ADR-0021). So every downstream consumer of those
    // descs (cooking, createShapeOrComputeMass, mass compute) must be handed THIS instance,
    // not the registry's attach for the same stage id. shared_ptr because the request is
    // copied into `rigidBodyRequests` and outlives executeQueryRigidBody's frame.
    std::shared_ptr<omni::physx::usdparser::AttachedStage> parseStage;

    virtual void releaseParsedData() override;
    virtual void sendRigidBodyResponse(const PhysxPropertyQueryRigidBodyResponse& response) override;
    virtual void sendColliderResponse(const PhysxPropertyQueryColliderResponse& response) override;
};

struct ArticulationRequest : public BaseRequest
{
    void sendArticulationResponse(const PhysxPropertyQueryArticulationResponse& response);
};

// The manager's real state (PIMPL). Every member function below is a
// PhysXPropertyQueryManagerImpl method; the public PhysXPropertyQueryManager
// facade at the bottom of this file forwards to it.
struct PhysXPropertyQueryManagerImpl
{
    std::vector<RigidBodyRequest> rigidBodyRequests;

    bool allCollidersHaveBeenComputed(const RigidBodyRequest& rigidBodyRequest,
                                      PhysxPropertyQueryResult::Enum& errorCode);
    bool rigidBodyRequestHasFinished(RigidBodyRequest& rigidBodyRequest);
    void attachToStageUpdate();
    void detachFromStageUpdate();
    bool reparseRigidBody(RigidBodyRequest& request);
    void executeQueryRigidBody(RigidBodyRequest& request);
    void executeQueryArticulation(ArticulationRequest& request);
    void processArticulation(const usdparser::AttachedStage& attachedStage, ArticulationRequest& request);

    void queryPrim(uint64_t stageId,
                   uint64_t primPathId,
                   PhysxPropertyQueryMode::Enum queryMode,
                   const IPhysxPropertyQueryCallback& callbacks);
    void updateQueuedRequests();
    void cancelAllPendingRequests(PhysxPropertyQueryResult::Enum errorCode);
};


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
    using namespace omni::physics::parse;
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
    // The request's parse source (RigidBodyRequest::parseStage) -- the one that minted the
    // collider descs' keys. Used here to intern each collider's source path into the ObjectKey
    // vocabulary ObjectIdPathMap carries (LoadTools.h).
    const omni::physics::parse::IPhysicsSource* source = nullptr;
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
            // The collider's ObjectKey (keyed by ObjectId); the mass MassAPI/material reads
            // resolve from it. getShapeMassInfo below re-derives its own shapeKey directly from
            // rigidBodyRequest.colliders[objectId] rather than from this map's value.
            shapes[usdparser::ObjectId(index++)] =
                source ? source->findByPath(coll.first) : omni::physics::parse::ObjectKey{};
        }
        return hasTriggers;
    }

    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(usdparser::ObjectId objectId) override
    {
        usdparser::PhysxShapeDesc* shapeDesc = rigidBodyRequest.colliders[objectId].second;
        const std::string& shapeKey = rigidBodyRequest.colliders[objectId].first;
        const omni::physics::parse::ObjectKey shapeObjectKey =
            source ? source->findByPath(shapeKey) : omni::physics::parse::ObjectKey{};
        PhysXType physxType;
        PhysXUsdPhysicsInterface::MassInformation massInfo;
        // Actually compute mass by passing null pxscene and null internalphysxdatabase
        PhysXUsdPhysicsInterface::createShapeOrComputeMass(
            shapeObjectKey, *shapeDesc, usdparser::kInvalidObjectId, nullptr,
            rigidBodyRequest.parseStage.get(), nullptr, false, physxType, nullptr, &massInfo);
        return massInfo;
    }
};

bool PhysXPropertyQueryManagerImpl::allCollidersHaveBeenComputed(const RigidBodyRequest& rigidBodyRequest,
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
    // The request's own parse stage, NOT UsdLoad's attach for this stage id: the collider
    // descs' meshPrimKeys were minted by the reparse against this instance, and a key minted
    // by one source never resolves against another (RigidBodyRequest::parseStage). The stage
    // handle's bool goes false once the UsdStage expires (a request can sit in the timeout queue
    // past its stage's close), and a missing source means the stage was never reparseable; both
    // map to the former no-live-stage guard and fail closed.
    AttachedStage* propertyQueryAttachedStage = rigidBodyRequest.parseStage.get();
    if (!propertyQueryAttachedStage || !propertyQueryAttachedStage->getStage() ||
        !propertyQueryAttachedStage->getSource())
    {
        errorCode = PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE;
        return false;
    }
    const omni::physics::parse::IPhysicsSource& src = *propertyQueryAttachedStage->getSource();
    for (auto& shapeDesc : rigidBodyRequest.colliders)
    {
        switch (shapeDesc.second->type)
        {
        case eConvexMeshShape: {
            ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)shapeDesc.second;
            if (!convexDesc->meshPrimKey.valid() || src.sourceKeyToString(convexDesc->meshPrimKey).empty())
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
            if (!convexDecompositionDesc->meshPrimKey.valid() || src.sourceKeyToString(convexDecompositionDesc->meshPrimKey).empty())
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
            if (!meshDesc->meshPrimKey.valid() || src.sourceKeyToString(meshDesc->meshPrimKey).empty())
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

bool PhysXPropertyQueryManagerImpl::rigidBodyRequestHasFinished(RigidBodyRequest& rigidBodyRequest)
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
        usdparser::ObjectId rbId(0);
        // MassAPI / material reads route through the source (backend-agnostic mass,
        // ADR-0002 M2c-D). The key vocabulary is the request's parse source -- the same
        // one that minted the descs' keys -- so mass, cooking and shape resolution all
        // share one minting authority.
        usdparser::AttachedStage& parseStage = *rigidBodyRequest.parseStage;
        omni::physics::parse::IPhysicsSource* massSource = parseStage.getSource();
        if (!massSource)
        {
            rigidBodyRequest.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
            return true;
        }
        acrbmInterface.source = massSource;
        const omni::physics::parse::ObjectKey bodyKey = parseStage.keyFor(rigidBodyRequest.primPath);
        if (!massSource->exists(bodyKey))
        {
            rigidBodyRequest.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM);
            return true;
        }
        usdparser::RigidBodyMass rigidBodyMass =
            usdparser::computeRigidBodyMass(&acrbmInterface, *massSource, bodyKey, rbId);
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
            parse::IUsdReparse* reparse = reparseBackend();
            for (auto& coll : rigidBodyRequest.colliders)
            {
                PhysxPropertyQueryColliderResponse collResponse;
                collResponse.usdStageId = rigidBodyRequest.stageId;
                uint64_t usdPathBits = 0;
                if (reparse)
                    reparse->stringToLegacyPathBits(coll.first.c_str(), usdPathBits);
                collResponse.usdPath = usdPathBits;
                PhysXUsdPhysicsInterface::MassInformation massInfo;
                PhysXType physxType;
                PhysXUsdPhysicsInterface::createShapeOrComputeMass(
                    parseStage.keyFor(coll.first), *coll.second, usdparser::kInvalidObjectId, nullptr,
                    &parseStage, nullptr, false, physxType, nullptr, &massInfo);
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

void PhysXPropertyQueryManagerImpl::updateQueuedRequests()
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

void PhysXPropertyQueryManagerImpl::cancelAllPendingRequests(PhysxPropertyQueryResult::Enum errorCode)
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

void PhysXPropertyQueryManagerImpl::attachToStageUpdate()
{
     OmniPhysX::getInstance().getStageUpdate().attachPropertyQueryStageUpdate();
}

void PhysXPropertyQueryManagerImpl::detachFromStageUpdate()
{
    OmniPhysX::getInstance().getStageUpdate().detachPropertyQueryStageUpdate();
}

void PhysXPropertyQueryManagerImpl::processArticulation(const usdparser::AttachedStage& attachedStage, ArticulationRequest& request)
{
    parse::IUsdReparse* reparse = reparseBackend();

    // Scoped query: visits every object under request.primPath (root inclusive).
    // Collected up front; the per-prim work below is a path-keyed object lookup
    // (no source data read).
    std::vector<omni::physics::parse::ObjectKey> descendants;
    if (const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
    {
        src->forEachDescendant(attachedStage.keyFor(request.primPath),
                               [&descendants](omni::physics::parse::ObjectKey k) { descendants.push_back(k); });
    }

    for (const omni::physics::parse::ObjectKey subKey : descendants)
    {
        // ObjectKey-native resolvability check (ADR-0019): forEachDescendant only yields
        // keys the source itself enumerated; same precedent as PhysXSceneQuery.cpp's
        // identical loop.
        if (!subKey.valid())
        {
            continue;
        }

        const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        PxArticulationReducedCoordinate* articulation =
            (PxArticulationReducedCoordinate*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                subKey, ePTArticulation, db, attachedStage);

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
                if (link->getName() && reparse)
                {
                    reparse->stringToLegacyPathBits(link->getName(), linkName);
                }

                PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                if (joint && joint->getName())
                {
                    if (reparse)
                        reparse->stringToLegacyPathBits(joint->getName(), jointName);
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

void PhysXPropertyQueryManagerImpl::executeQueryArticulation(ArticulationRequest& request)
{
    parse::IUsdReparse* reparse = reparseBackend();

    // Stage residency + prim validity via the loadable USD backend's session source: openStage
    // resolves the stage id (null => no resident stage), and its IPhysicsSource answers prim
    // existence. Both are the pxr-free replacement for the former UsdUtilsStageCache::Find +
    // GetPrimAtPath(intToPath(primPathId)).IsValid() pair. The session is transient -- only the
    // guard needs it.
    {
        usdparser::UsdLoad* usdLoad = usdparser::UsdLoad::getUsdLoad();
        std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);
        parse::UsdReparseSessionPtr session =
            reparse ? parse::openScopedStage(*reparse, request.stageId) : parse::UsdReparseSessionPtr{};
        if (!session)
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
            return;
        }
        const omni::physics::parse::IPhysicsSource* src = reparse->source(*session);
        const bool primValid = src && src->exists(src->findByPath(request.primPath));
        session.reset(); // closed before any callback runs
        if (!primValid)
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
            return;
        }
    }

    if (!request.callbacks.articulationCallback)
    {
        request.callQueryFinished();
        return;
    }

    // Synchronous path for now; parses the whole stage. If already parsed, use the existing data.
    if (usdparser::UsdLoad::getUsdLoad()->getActiveStageId() == static_cast<long>(request.stageId))
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
        physxSimulationAttach(static_cast<long>(request.stageId));
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

// Single-prim rigid-body reparse, pxr-free. Mirrors the former native parseRigidBodyNative,
// but the native USD walk is replaced by usdReparse()->scan() and the
// re-key into the request's minting AttachedStage is done through the (unconditional)
// translateScannedShape. `request.parseStage` must already be bound (setStage). Returns true
// and fills request.desc/colliders on success; false leaves request.desc null.
bool PhysXPropertyQueryManagerImpl::reparseRigidBody(RigidBodyRequest& request)
{
    parse::IUsdReparse* reparse = reparseBackend();
    if (!reparse)
        return false;

    // Owns the scan source for the whole reparse; `scanned` below has its own source and is
    // destroyed first.
    parse::UsdReparseSessionPtr session = parse::openScopedStage(*reparse, request.stageId);
    if (!session)
        return false;

    bool ok = false;
    const omni::physics::parse::IPhysicsSource* scanSource = reparse->source(*session);
    if (scanSource && scanSource->exists(scanSource->findByPath(request.primPath)))
    {
        // Default ScanOptions => eActiveInstanced => eInstanceProxies, matching the native
        // subtree scanStage default parseRigidBodyNative uses.
        parse::ScanOptions options;
        parse::ScannedStage scanned = reparse->scan(*session, { request.primPath }, {}, options,
                                                    usdparser::iceDescriptorAllocator());
        const omni::physics::parse::IPhysicsSource& src = scanned.source();

        // Find the body matching request.primPath (typically the range root; explicit match
        // keeps us robust to nested-body subtrees).
        omni::physics::parse::PhysxRigidBodyDesc* rbDesc = nullptr;
        for (auto& bodyUPtr : scanned.bodies)
        {
            if (src.sourceKeyToString(bodyUPtr->primKey) == request.primPath)
            {
                rbDesc = bodyUPtr.get();
                break;
            }
        }

        if (rbDesc)
        {
            usdparser::AttachedStage& attachedStage = *request.parseStage;

            // sceneIds -- sourceSimulationOwners ObjectKeys -> eScene ObjectIds. Empty
            // ObjectDatabase (fresh AttachedStage) yields empty sceneIds.
            rbDesc->sceneIds.clear();
            for (const auto& sk : rbDesc->sourceSimulationOwners)
            {
                const omni::physics::parse::ObjectKey ownerKey =
                    attachedStage.keyFor(src.sourceKeyToString(sk));
                if (!ownerKey.valid())
                    continue;
                const usdparser::ObjectId entry =
                    attachedStage.getObjectDatabase()->findEntry(ownerKey, omni::physics::parse::eScene);
                if (entry != usdparser::kInvalidObjectId)
                    rbDesc->sceneIds.push_back(entry);
            }

            // Path-set of the body's collisions; filter the scanned shape vector against it.
            std::unordered_set<std::string> bodyCollisionPaths;
            bodyCollisionPaths.reserve(rbDesc->sourceCollisions.size());
            for (const auto& ck : rbDesc->sourceCollisions)
            {
                std::string sp(src.sourceKeyToString(ck));
                if (!sp.empty())
                    bodyCollisionPaths.insert(std::move(sp));
            }

            for (auto& shapeUPtr : scanned.shapes)
            {
                omni::physics::parse::PhysxShapeDesc* desc = shapeUPtr.get();
                std::string shapePath(src.sourceKeyToString(desc->primKey));
                if (bodyCollisionPaths.find(shapePath) == bodyCollisionPaths.end())
                    continue;

                // Re-key into attachedStage's key-space + cooking dispatch + consumer state.
                usdparser::translateScannedShape(attachedStage, scanned, desc,
                                                 attachedStage.keyFor(shapePath));
                if (omni::physics::parse::PhysxShapeDesc* released = shapeUPtr.release())
                    request.colliders.emplace_back(std::move(shapePath), released);
            }

            // Transfer the body desc out of scanned.bodies (ICE-allocated; released so the
            // consumer-side ICE_PLACEMENT_DELETE matches).
            for (auto& bodyUPtr : scanned.bodies)
            {
                if (bodyUPtr.get() == rbDesc)
                {
                    request.desc = bodyUPtr.release();
                    break;
                }
            }
            ok = request.desc != nullptr;
        }
    }

    return ok;
}

void PhysXPropertyQueryManagerImpl::executeQueryRigidBody(RigidBodyRequest& request)
{
    parse::IUsdReparse* reparse = reparseBackend();

    // Upfront stage-residency + prim-validity guard via the loadable USD backend's session
    // source -- the pxr-free replacement for the former UsdUtilsStageCache::Find +
    // GetPrimAtPath(intToPath(primPathId)).IsValid() pair, preserving the eERROR_INVALID_USD_STAGE
    // / eERROR_INVALID_USD_PATH error codes. Session is transient; the live and reparse branches
    // below open their own.
    {
        usdparser::UsdLoad* usdLoad = usdparser::UsdLoad::getUsdLoad();
        std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);
        parse::UsdReparseSessionPtr session =
            reparse ? parse::openScopedStage(*reparse, request.stageId) : parse::UsdReparseSessionPtr{};
        if (!session)
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
            return;
        }
        const omni::physics::parse::IPhysicsSource* src = reparse->source(*session);
        const bool primValid = src && src->exists(src->findByPath(request.primPath));
        session.reset(); // closed before any callback runs
        if (!primValid)
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
            return;
        }
    }

    if (usdparser::UsdLoad::getUsdLoad()->getActiveStageId() == static_cast<long>(request.stageId))
    {
        const usdparser::AttachedStage* attachedStage =
            usdparser::UsdLoad::getUsdLoad()->getAttachedStage(request.stageId);
        if (attachedStage)
        {
            const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
            // ObjectKey (ADR-0019): the lookup uses the pxr-free getObjectDataOrID overload,
            // keyed by the request's path minted against this attach's source.
            const omni::physics::parse::ObjectKey primObjKey = attachedStage->keyFor(request.primPath);
            PxRigidActor* actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                primObjKey, ePTActor, db, *attachedStage);
            if (!actor)
            {
                actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
                    primObjKey, ePTLink, db, *attachedStage);
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
                            uint64_t usdPathBits = 0;
                            if (reparse)
                                reparse->stringToLegacyPathBits(attachedStage->textFor(shapeRecord.mKey), usdPathBits);
                            collResponse.usdPath = usdPathBits;
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


    // One AttachedStage for the whole request: it is the parse target AND the authority that
    // mints every ObjectKey the resulting descs carry, so it is kept on the request and reused
    // by allCollidersHaveBeenComputed / the mass compute / createShapeOrComputeMass below.
    // Building it here (rather than reusing UsdLoad's attach for this stage id) also keeps the
    // deliberate isolation of the native re-parse: it must not mutate a live attach's state,
    // and under an ovstage attach the registry's source is not a USD one at all.
    //
    // resolveStageToHandle() + setStage() is the pxr-free analogue of attachForeignStage's
    // stage-cache lookup + setStage; the parse work (source rebuild + scan) is serialized under
    // UsdLoad::mParsingMutex, which the loadable backend deliberately does not take itself.
    request.parseStage = std::make_shared<usdparser::AttachedStage>();
    {
        usdparser::UsdLoad* usdLoad = usdparser::UsdLoad::getUsdLoad();
        std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);

        usdparser::AttachedStageUsdHandle handle;
        if (!reparse || !reparse->resolveStageToHandle(request.stageId, handle.storage()))
        {
            request.parseStage.reset();
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE);
            return;
        }
        request.parseStage->setStage(handle);

        if (!reparseRigidBody(request))
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_PARSING);
            return;
        }
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

void PhysXPropertyQueryManagerImpl::queryPrim(uint64_t stageId,
                                          uint64_t primPathId,
                                          PhysxPropertyQueryMode::Enum queryMode,
                                          const IPhysxPropertyQueryCallback& callbacks)
{
    parse::IUsdReparse* reparse = reparseBackend();
    // Decode the incoming legacy asInt(SdfPath) bits once into a path string every downstream
    // step resolves against. A 0/empty id is not a valid prim path.
    std::string primPath;
    const bool pathOk = reparse && reparse->legacyPathBitsToString(primPathId, primPath);

    switch (queryMode)
    {
    case PhysxPropertyQueryMode::eQUERY_RIGID_BODY_WITH_COLLIDERS: {
        RigidBodyRequest request;
        request.callbacks = callbacks;
        request.stageId = stageId;
        request.primPathId = primPathId;
        request.primPath = primPath;
        if (!pathOk)
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
            break;
        }
        executeQueryRigidBody(request);
    }
    break;
    case PhysxPropertyQueryMode::eQUERY_ARTICULATION: {
        ArticulationRequest request;
        request.callbacks = callbacks;
        request.stageId = stageId;
        request.primPathId = primPathId;
        request.primPath = primPath;
        if (!pathOk)
        {
            request.returnWithError(PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH);
            break;
        }
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

// ------------------------------------------------------------------
// Public facade (PhysXPropertyQuery.h) -> Impl.
//
// mImpl is live only when a USD reparse backend is installed (test executables, ADR-0027). In
// production usdReparse() is null, so mImpl stays null and the facade fails every query
// synchronously with eERROR_INVALID_USD_STAGE, while updateQueuedRequests /
// cancelAllPendingRequests are no-ops.
//
// mImpl is created lazily on first query rather than at construction: OmniPhysX (and this
// manager) is constructed during runtime::startup(), but the loader installs the USD reparse
// backend AFTER startup() returns (the process bootstrap, ADR-0027). Latching at construction
// would leave mImpl permanently null for the whole test session; instead each entry point
// materializes it once usdReparse() is available.
// ------------------------------------------------------------------
PhysXPropertyQueryManager::PhysXPropertyQueryManager()
    : mImpl(nullptr)
{
}

PhysXPropertyQueryManager::~PhysXPropertyQueryManager()
{
    delete mImpl;
}

bool PhysXPropertyQueryManager::ensureImpl()
{
    if (!mImpl && parse::usdReparse())
        mImpl = new PhysXPropertyQueryManagerImpl();
    return mImpl != nullptr;
}

void PhysXPropertyQueryManager::queryPrim(uint64_t stageId,
                                          uint64_t primPathId,
                                          PhysxPropertyQueryMode::Enum queryMode,
                                          const IPhysxPropertyQueryCallback& callbacks)
{
    // The seam can also be removed after mImpl latched (test loader), so gate on it directly.
    if (parse::usdReparse() && ensureImpl())
    {
        mImpl->queryPrim(stageId, primPathId, queryMode, callbacks);
        return;
    }

    // No USD reparse backend: there is no live UsdStage to resolve stageId/primPathId against,
    // for any query mode, so every request fails synchronously instead of being queued.
    (void)stageId;
    (void)primPathId;
    (void)queryMode;
    if (callbacks.rigidBodyCallback)
    {
        PhysxPropertyQueryRigidBodyResponse rbResponse;
        rbResponse.result = PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE;
        callbacks.rigidBodyCallback(rbResponse, callbacks.userData);
    }
    if (callbacks.colliderCallback)
    {
        PhysxPropertyQueryColliderResponse collResponse;
        collResponse.result = PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE;
        callbacks.colliderCallback(collResponse, callbacks.userData);
    }
    if (callbacks.queryFinishedCallback)
    {
        callbacks.queryFinishedCallback(callbacks.userData);
    }
}

void PhysXPropertyQueryManager::updateQueuedRequests()
{
    if (mImpl)
        mImpl->updateQueuedRequests();
}

void PhysXPropertyQueryManager::cancelAllPendingRequests(PhysxPropertyQueryResult::Enum errorCode)
{
    if (mImpl)
        mImpl->cancelAllPendingRequests(errorCode);
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
