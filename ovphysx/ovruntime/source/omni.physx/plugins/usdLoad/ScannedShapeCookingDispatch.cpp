// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-2 AC-3 AC-4
 */

#include "UsdPCH.h"
#include "IceDescriptorAllocator.h"

#include "AttachedStage.h"
#include "Collision.h"
#include "OmniPhysX.h"
#include "ScannedShapeCookingDispatch.h"
#include "UsdSource.h"
#include <PhysXTools.h>

#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>

#include <common/utilities/Utilities.h>

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/usd/StageScan.h>

#include <omni/physx/IPhysxCookingService.h>
#include <private/omni/physx/PhysxUsd.h>

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdUtils/stageCache.h>

using namespace PXR_NS;

namespace omni::physx::usdparser::scan
{

// ShapeCookingCache is header-only — see ScannedShapeCookingDispatch.h.

// ---------------------------------------------------------------------------
// resolveConsumerSideShapeState
// ---------------------------------------------------------------------------

bool resolveConsumerSideShapeState(
    AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned,
    PhysxShapeDesc* desc,
    SdfPathVector& outMaterials,
    CollisionPairVector& outFilteredPairs)
{
    if (!desc)
        return false;

    // `primKey` is the shape's own prim (where UsdPhysicsCollisionAPI
    // is applied); `sourceGprim` is the geometry prim.  They coincide
    // for simple shapes and differ for mesh shapes with parent-applied
    // collision API.  filteredPairs uses the shape's own primKey, the
    // same way legacy fillPhysxShapeDesc does at line 338-341.
    const SdfPath primKey = scanned.pathFor(desc->primKey);

    // sourceMaterials → SdfPath list.  Preserve empty entries — the schema
    // parser uses an empty SdfPath as the "default material" placeholder
    // (legacy `finalizeShape` calls `getMaterial(emptyPath)` which returns
    // kInvalidObjectId, signaling "use default" to PhysX createShape).
    outMaterials.reserve(desc->sourceMaterials.size());
    for (const auto& mk : desc->sourceMaterials)
        outMaterials.push_back(scanned.pathFor(mk));

    // sourceFilteredCollisions → pairs (primKey, filtered) appended to
    // the OUT collection.  Matches legacy fillPhysxShapeDesc 338-341.
    for (const auto& fk : desc->sourceFilteredCollisions)
    {
        SdfPath fp = scanned.pathFor(fk);
        if (!fp.IsEmpty())
            outFilteredPairs.push_back(std::make_pair(primKey, fp));
    }

    // sourceSimulationOwners → sceneIds via ObjectDatabase.  Matches
    // legacy fillPhysxShapeDesc 516-526: only scenes that resolve to a
    // valid eScene entry contribute.  If `simulationOwners` was non-
    // empty but nothing resolved, the shape is dropped (return false).
    if (desc->sourceSimulationOwners.empty())
        return true;

    for (const auto& sk : desc->sourceSimulationOwners)
    {
        SdfPath sp = scanned.pathFor(sk);
        if (sp.IsEmpty())
            continue;
        const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(sp, eScene);
        if (entry != kInvalidObjectId)
            desc->sceneIds.push_back(entry);
    }
    return !desc->sceneIds.empty();
}

// ---------------------------------------------------------------------------
// dispatchScannedShapeCooking
// ---------------------------------------------------------------------------

namespace
{

// Build a PhysxCookingComputeRequest from the attached source identity.  Mirrors
// the legacy fillCookingRequest metadata without resolving a UsdPrim.
void buildCookingRequest(omni::physx::PhysxCookingComputeRequest& request,
                         const AttachedStage& attachedStage,
                         const SdfPath& meshPrimKey,
                         UsdTimeCode time)
{
    request.primStageId = uint64_t(attachedStage.getStageId());
    request.primId = ::asInt(meshPrimKey);
    request.primTimeCode = time.GetValue();
    request.options.setFlag(
        omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
    request.options.setFlag(
        omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
    request.options.setFlag(
        omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
    request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
}

bool isMeshCookableType(omni::physics::parse::ObjectType t)
{
    using namespace omni::physics::parse;
    return t == eConvexMeshShape
        || t == eTriangleMeshShape
        || t == eConvexMeshDecompositionShape
        || t == eSpherePointsShape;
}

} // anonymous

void dispatchScannedShapeCooking(
    AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned,
    PhysxShapeDesc* desc)
{
    if (!desc || !isMeshCookableType(desc->type))
        return;

    // Resolve the source mesh prim — that's where the cooking request
    // identifies its input data from.  scanStage stores the gprim key
    // on every shape desc (assigned in StageScan.cpp::emitShape).
    const SdfPath meshPrimKey = scanned.pathFor(desc->sourceGprim);
    if (meshPrimKey.IsEmpty())
        return;

    const omni::physics::parse::ObjectKey meshKey = attachedStage.keyFor(meshPrimKey);
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(meshKey))
        return;
    const bool sourceIsUsdGeomMesh = omni::physx::internal::isAType<UsdGeomMesh>(*src, meshKey);

    auto* cookingService = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return;

    omni::physx::PhysxCookingComputeRequest request;
    buildCookingRequest(request, attachedStage, meshPrimKey, UsdTimeCode::Default());

    SourceMeshGeometryScope sourceGeomScope;
    const bool sourceGeometryProvided =
        fillCookingMeshViewFromSource(request, sourceGeomScope, attachedStage, meshKey);

    // PhysxMeshMergeCollisionAPI / standalone mesh-merge custom shapes: the source
    // prim is an Xform with the merge API applied, not a UsdGeomMesh, so the cooking
    // service's USD lookup would fail with "prim is not UsdGeomMesh". Legacy
    // Collision.cpp:1526-1546 routes these through eINPUT_MODE_FROM_PRIM_MESH_VIEW,
    // passing the merged points/indices/faces/holes directly. scanStage populates
    // `mergedMesh` for both standard mesh shapes (populateMergeMesh) and mesh-merge
    // shapes (populateMergedMeshFromChildren); use FROM_PRIM_MESH_VIEW whenever the
    // buffer is populated and the source isn't a UsdGeomMesh (standard mesh shapes
    // always get FROM_PRIM_ID via their UsdGeomMesh source instead).
    using namespace omni::physics::parse;
    if (desc->type == eConvexMeshShape ||
        desc->type == eTriangleMeshShape ||
        desc->type == eConvexMeshDecompositionShape ||
        desc->type == eSpherePointsShape)
    {
        const auto* mergeBase = static_cast<const MergeMeshPhysxShapeDesc*>(desc);
        if (!sourceGeometryProvided &&
            mergeBase->mergedMesh &&
            !mergeBase->mergedMesh->points.empty() &&
            !mergeBase->mergedMesh->indices.empty() &&
            !mergeBase->mergedMesh->faces.empty() &&
            !sourceIsUsdGeomMesh)
        {
            request.dataInputMode = omni::physx::PhysxCookingComputeRequest::DataInputMode::eINPUT_MODE_FROM_PRIM_MESH_VIEW;
            request.primMeshView.points      = { mergeBase->mergedMesh->points.data(), mergeBase->mergedMesh->points.size() };
            request.primMeshView.indices     = { mergeBase->mergedMesh->indices.data(), mergeBase->mergedMesh->indices.size() };
            request.primMeshView.faces       = { mergeBase->mergedMesh->faces.data(), mergeBase->mergedMesh->faces.size() };
            request.primMeshView.holeIndices = { mergeBase->mergedMesh->holes.data(), mergeBase->mergedMesh->holes.size() };
        }
    }

    switch (desc->type)
    {
    case omni::physics::parse::eConvexMeshShape:
    {
        auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(desc);
        request.onFinished = [d, meshPrimKey](const omni::physx::PhysxCookingComputeResult& result) {
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
                return;
            d->crc = result.cookedDataCRC;
            d->meshKey = result.meshKey;
            // gMeshKeyCache write deferred until exposed via Collision.h.
        };
        cookingService->requestConvexMeshCookedData(nullptr, request, d->convexCookingParams);
        break;
    }
    case omni::physics::parse::eConvexMeshDecompositionShape:
    {
        auto* d = static_cast<ConvexMeshDecompositionPhysxShapeDesc*>(desc);
        request.onFinished = [d, meshPrimKey](const omni::physx::PhysxCookingComputeResult& result) {
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
                return;
            d->crc = result.cookedDataCRC;
            d->meshKey = result.meshKey;
        };
        cookingService->requestConvexMeshDecompositionCookedData(
            nullptr, request, d->convexDecompositionCookingParams);
        break;
    }
    case omni::physics::parse::eSpherePointsShape:
    {
        auto* d = static_cast<SpherePointsPhysxShapeDesc*>(desc);
        request.onFinished = [d, meshPrimKey](const omni::physx::PhysxCookingComputeResult& result) {
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
                return;
            d->crc = result.cookedDataCRC;
            d->meshKey = result.meshKey;
        };
        cookingService->requestSphereFillCookedData(
            nullptr, request, d->sphereFillCookingParams);
        break;
    }
    case omni::physics::parse::eTriangleMeshShape:
    {
        auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(desc);
        request.onFinished = [d, meshPrimKey](const omni::physx::PhysxCookingComputeResult& result) {
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
                return;
            d->crc = result.cookedDataCRC;
            d->meshKey = result.meshKey;
        };

        // SDF cooking gate: scanStage's eSdf approximation branch
        // populated both triangleMeshCookingParams AND sdfMeshCookingParams.
        // The cooking service dispatches differently based on whether
        // SDF is enabled.  Re-call parseSdfMeshCookingExt to recover
        // the isValidSDF flag (it's idempotent on already-populated
        // params).  Matches legacy usdLoad/Collision.cpp:1163-1174.
        bool isValidSDF = false;
        if (omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
        {
            omni::physics::parse::ParseContext parseCtx(*src, iceDescriptorAllocator());
            isValidSDF = omni::physics::parse::parseSdfMeshCookingExt(
                parseCtx,
                scanned.keyFor(meshPrimKey),
                d->sdfMeshCookingParams);
        }
        if (isValidSDF)
        {
            cookingService->requestSdfMeshCookedData(
                nullptr, request, d->triangleMeshCookingParams, d->sdfMeshCookingParams);
        }
        else
        {
            cookingService->requestTriangleMeshCookedData(
                nullptr, request, d->triangleMeshCookingParams);
        }
        break;
    }
    default:
        // Bounding-shape / custom / simple shapes — no cooking-service
        // dispatch.  Bounding shapes need a follow-up bounding-compute
        // helper (createBoundingSphere / createOBB) that's still TBD;
        // consumers must fall back to the legacy path for those.
        break;
    }
}

} // namespace omni::physx::usdparser::scan
