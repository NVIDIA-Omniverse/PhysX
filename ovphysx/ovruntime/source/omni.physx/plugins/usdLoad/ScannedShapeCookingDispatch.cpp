// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-46
 *
 * @implements REQ-PARSE-SHAPE-005
 * @covers AC-3
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-2
 *
 * `resolveConsumerSideShapeState`'s `sourceFilteredCollisions` translation
 * re-keys both pair endpoints (the shape's own key and each filtered
 * target) via the source-string round trip before pushing into
 * `outFilteredPairs` (AC-3 / AC-5). Left in scan-space, a compound
 * body's child-collider-level `FilteredPairsAPI` silently never reached
 * PhysX, masked for a plain rigid body only by the separate, correctly-
 * keyed rigid-body-level loop reading the same schema off the same prim.
 */

#include "ScannedShapeCookingDispatch.h"

#include "IceDescriptorAllocator.h"

#include "AttachedStage.h"
#include "Collision.h"
#include "OmniPhysX.h"

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/parse/ScannedStage.h>

#include <omni/physx/IPhysxCookingService.h>
#include <private/omni/physx/PhysxUsd.h>

namespace omni::physx::usdparser::scan
{

// ShapeCookingCache is header-only — see ScannedShapeCookingDispatch.h.

// ---------------------------------------------------------------------------
// resolveConsumerSideShapeState -- ObjectKey-native (unconditional)
// ---------------------------------------------------------------------------

bool resolveConsumerSideShapeState(
    AttachedStage& attachedStage,
    const omni::physics::parse::ScannedStage& scanned,
    PhysxShapeDesc* desc,
    std::vector<omni::physics::parse::ObjectKey>& outMaterials,
    CollisionPairVector& outFilteredPairs)
{
    if (!desc)
        return false;

    const omni::physics::parse::IPhysicsSource& src = scanned.source();

    // sourceMaterials -> attachedStage-space ObjectKey list. Preserve invalid
    // entries -- Collision.h's ObjectKey-native `finalizeShape` (getMaterial on
    // an invalid key) treats an unresolved entry as "use default material",
    // the same role the SdfPath sibling's empty-SdfPath placeholder plays.
    outMaterials.reserve(desc->sourceMaterials.size());
    for (const omni::physics::parse::ObjectKey mk : desc->sourceMaterials)
        outMaterials.push_back(mk.valid() ? attachedStage.keyFor(src.sourceKeyToString(mk)) :
                                            omni::physics::parse::ObjectKey{});

    // sourceFilteredCollisions -> pairs (desc->primKey, filtered) appended to the
    // OUT collection. Both desc->primKey and the sourceFilteredCollisions targets
    // are still keyed in the *scanned*-stage's own key space (minted by the
    // scan-time source), so -- exactly like the rigid-body/deformable/articulation
    // loops in LoadStage.cpp -- they must round-trip through the source's string
    // identity before landing in outFilteredPairs (consumed downstream by
    // createFilteredPairs() via attachedStage-space keys). Left unresolved, the
    // pair silently disappears, which for a compound rigid body's per-collider
    // FilteredPairsAPI (no body-level relationship on the same prim to mask the
    // gap) means the shape is never actually filtered against its target.
    const omni::physics::parse::ObjectKey shapePrimKey = attachedStage.keyFor(src.sourceKeyToString(desc->primKey));
    for (const omni::physics::parse::ObjectKey fk : desc->sourceFilteredCollisions)
    {
        if (!fk.valid())
            continue;
        const std::string_view targetStr = src.sourceKeyToString(fk);
        if (targetStr.empty())
            continue;
        outFilteredPairs.push_back(std::make_pair(shapePrimKey, attachedStage.keyFor(targetStr)));
    }

    // sourceSimulationOwners -> sceneIds via ObjectDatabase. Matches legacy
    // fillPhysxShapeDesc 516-526: only scenes that resolve to a valid eScene
    // entry contribute. If `simulationOwners` was non-empty but nothing
    // resolved, the shape is dropped (return false).
    if (desc->sourceSimulationOwners.empty())
        return true;

    for (const omni::physics::parse::ObjectKey sk : desc->sourceSimulationOwners)
    {
        const omni::physics::parse::ObjectKey ownerKey = attachedStage.keyFor(src.sourceKeyToString(sk));
        if (!ownerKey.valid())
            continue;
        const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(ownerKey, eScene);
        if (entry != kInvalidObjectId)
            desc->sceneIds.push_back(entry);
    }
    return !desc->sceneIds.empty();
}

// ---------------------------------------------------------------------------
// dispatchScannedShapeCooking (unconditional)
// ---------------------------------------------------------------------------

namespace
{

// Build a PhysxCookingComputeRequest from the attached source identity, without
// resolving a UsdPrim.
void buildCookingRequest(omni::physx::PhysxCookingComputeRequest& request,
                         const AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey meshKey)
{
    // primStageId/primId are correlation keys only (REQ-COOK-SOURCE-001 AC-1), not an input
    // source; attachHandle is the attach the request belongs to (ADR-0016 Decision 6). The handle
    // is nonzero for a stageless attach, where the stage id is 0 and names nothing.
    request.primStageId = uint64_t(attachedStage.getStageId());
    request.attachHandle = attachedStage.getAttachHandle();
    // primId carries the legacy asInt(ObjectKey) encoding (key.handle) -- primId is a
    // correlation/logging field only, never resolved by the cooking service as a real path.
    // Mirrors CookingDataAsync.cpp's keyToLegacyPathInt.
    request.primId = meshKey.handle;
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
    const omni::physics::parse::ScannedStage& scanned,
    PhysxShapeDesc* desc)
{
    if (!desc || !isMeshCookableType(desc->type))
        return;

    // Resolve the source mesh prim — that's where the cooking request
    // identifies its input data from.  scanStage stores the gprim key
    // on every shape desc (assigned in StageScan.cpp::emitShape). Rekey
    // through the source's own string identity (mechanical -- see this
    // file's header comment) rather than a ScannedStage-typed
    // pathFor()/keyFor(SdfPath) pair, which only the USD-derived
    // ScannedStage exposes.
    if (!desc->sourceGprim.valid())
        return;
    const std::string_view meshPrimKeyStr = scanned.source().sourceKeyToString(desc->sourceGprim);
    if (meshPrimKeyStr.empty())
        return;

    const omni::physics::parse::ObjectKey meshKey = attachedStage.keyFor(meshPrimKeyStr);
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(meshKey))
        return;
    const bool sourceIsUsdGeomMesh = src->isA(meshKey, attachedStage.getKnownTokens().meshType);

    auto* cookingService = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return;

    omni::physx::PhysxCookingComputeRequest request;
    buildCookingRequest(request, attachedStage, meshKey);

    // Only multi-material triangle/SDF cooking needs per-face material indices; skip the
    // per-mesh GeomSubset walk for single-material shapes.
    const bool includeFaceMaterials = (desc->type == omni::physics::parse::eTriangleMeshShape);
    SourceMeshGeometryScope sourceGeomScope;
    const bool sourceGeometryProvided =
        fillCookingMeshViewFromSource(request, sourceGeomScope, attachedStage, meshKey, includeFaceMaterials);

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
            // Every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID removed); the struct
            // default for metersPerUnit is 1.0 unless it is set here, which would otherwise get
            // every PxTolerancesScale-derived tolerance wrong on a non-metre stage.
            request.primMeshMetersPerUnit = double(attachedStage.getSourceUnits().metersPerUnit);
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
        request.onFinished = [d](const omni::physx::PhysxCookingComputeResult& result) {
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
        request.onFinished = [d](const omni::physx::PhysxCookingComputeResult& result) {
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
        request.onFinished = [d](const omni::physx::PhysxCookingComputeResult& result) {
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
        request.onFinished = [d](const omni::physx::PhysxCookingComputeResult& result) {
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
        if (omni::physics::parse::IPhysicsSource* parseSrc = attachedStage.getSource())
        {
            omni::physics::parse::ParseContext parseCtx(*parseSrc, iceDescriptorAllocator());
            parseCtx.adoptKnownTokens(attachedStage.getKnownTokens());
            // The context reads through the attach source, so it needs the attach-space key
            // (desc->sourceGprim is still scan-space here; a USD attach source rejects it).
            isValidSDF = omni::physics::parse::parseSdfMeshCookingExt(
                parseCtx,
                meshKey,
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
