// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-1
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-29
 */

#include <common/foundation/Allocator.h>
#include <private/omni/physx/PhysxUsd.h>

#include "AttachedStage.h"
#include "DeformableBodyConverter.h"
#include "PhysicsBody.h"

#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/ScannedStage.h>

#include <cstring>

namespace omni::physx::usdparser::convert
{

namespace
{

// Copies fields common to both volume and surface bodies.  Caller has
// already placement-new'd `dst` and called `setToDefault`, so any field
// the parse-lib doesn't track keeps its default.
void overlayCommonFields(PhysxDeformableBodyDesc& dst,
                         const omni::physics::parse::PhysxDeformableBodyDesc& src,
                         const omni::physics::parse::ScannedStage& scanned,
                         const AttachedStage& attachedStage)
{
    dst.bodyEnabled  = src.bodyEnabled;
    dst.kinematicBody = src.kinematicBody;
    dst.startsAsleep  = src.startsAsleep;
    dst.mass          = src.mass;

    // parse::Matrix4d is 16 row-major doubles; PxMat44d holds the same sixteen
    // doubles for the same transform (row-major/row-vector vs column-major/
    // column-vector cancel), so this is a straight element copy, no transpose.
    static_assert(sizeof(dst.transform) == sizeof(src.transform),
                  "PxMat44d / parse::Matrix4d layout mismatch");
    std::memcpy(&dst.transform, &src.transform, sizeof(dst.transform));

    // PhysxBaseDeformableBodyAPI fields.
    dst.linearDamping              = src.linearDamping;
    dst.maxLinearVelocity          = src.maxLinearVelocity;
    dst.sleepThreshold             = src.sleepThreshold;
    dst.settlingThreshold          = src.settlingThreshold;
    dst.settlingDamping            = src.settlingDamping;
    dst.maxDepenetrationVelocity   = src.maxDepenetrationVelocity;
    dst.selfCollisionFilterDistance = src.selfCollisionFilterDistance;
    dst.solverPositionIterationCount = src.solverPositionIterationCount;
    dst.enableSpeculativeCCD       = src.enableSpeculativeCCD;
    dst.selfCollision              = src.selfCollision;
    dst.disableGravity             = src.disableGravity;

    // PhysxAutoDeformableBodyAPI fields.
    dst.hasAutoAPI                      = src.hasAutoAPI;
    dst.isAutoMeshSimplificationEnabled = src.isAutoMeshSimplificationEnabled;
    dst.isAutoRemeshingEnabled          = src.isAutoRemeshingEnabled;
    dst.autoRemeshingResolution         = src.autoRemeshingResolution;
    dst.autoTriangleTargetCount         = src.autoTriangleTargetCount;
    dst.hasAutoForceConforming          = src.hasAutoForceConforming;

    // ObjectKey / TokenId-typed fields -- dst and src are now the same
    // parse-lib type (ADR-0019 increment 7), but src's ObjectKeys/TokenIds
    // are minted by `scanned`'s own (throwaway, parse-time) source. Per
    // ADR-0004's key-space invariant, they are only meaningful against
    // `scanned`'s own source -- every consumer of `dst` resolves through
    // `attachedStage` instead, so each is re-keyed/re-interned into
    // `attachedStage`'s persistent namespace via the source's string identity
    // (path text / token text) rather than a ScannedStage-typed pathFor/
    // tfTokenFor pair, which only the USD-derived ScannedStage exposes
    // (mirrors the `rekey` pattern used throughout LoadStage.cpp; guarded on
    // `.valid()` first the same way, since round-tripping an already-invalid
    // key/token is not guaranteed to stay invalid).
    const omni::physics::parse::IPhysicsSource& scanSrc = scanned.source();
    const omni::physics::parse::IPhysicsSource* asSrc = attachedStage.getSource();
    auto rekey = [&](omni::physics::parse::ObjectKey k) -> omni::physics::parse::ObjectKey
    {
        return k.valid() ? attachedStage.keyFor(scanSrc.sourceKeyToString(k)) : omni::physics::parse::ObjectKey{};
    };
    auto reintern = [&](omni::physics::parse::TokenId t) -> omni::physics::parse::TokenId
    {
        return (t.valid() && asSrc) ? asSrc->internToken(scanSrc.tokenToString(t)) : omni::physics::parse::TokenId{};
    };

    dst.simMeshKey = rekey(src.simMeshKey);
    dst.simMeshBindPoseToken = reintern(src.simMeshBindPoseToken);
    dst.simMeshLeftHandedOrientation = src.simMeshLeftHandedOrientation;

    dst.collisionMeshKey = rekey(src.collisionMeshKey);
    dst.collisionMeshBindPoseToken = reintern(src.collisionMeshBindPoseToken);
    dst.collisionMeshLeftHandedOrientation = src.collisionMeshLeftHandedOrientation;

    dst.skinGeomPaths.clear();
    dst.skinGeomPaths.reserve(src.skinGeomPaths.size());
    for (const omni::physics::parse::ObjectKey k : src.skinGeomPaths)
        dst.skinGeomPaths.push_back(rekey(k));
    dst.skinGeomBindPoseTokens.clear();
    dst.skinGeomBindPoseTokens.reserve(src.skinGeomBindPoseTokens.size());
    for (const omni::physics::parse::TokenId t : src.skinGeomBindPoseTokens)
        dst.skinGeomBindPoseTokens.push_back(reintern(t));

    dst.cookingSrcMeshKey = rekey(src.cookingSrcMeshKey);
    dst.cookingSrcMeshBindPoseToken = reintern(src.cookingSrcMeshBindPoseToken);
}

} // namespace

PhysxDeformableBodyDesc* convertScannedDeformableBody(
    const omni::physics::parse::ScannedStage& scanned,
    size_t index,
    const omni::physics::parse::SourceUnits& units,
    const AttachedStage& attachedStage)
{
    if (index >= scanned.deformables.size())
        return nullptr;
    const omni::physics::parse::PhysxDeformableBodyDesc* src = scanned.deformables[index].get();
    if (!src)
        return nullptr;

    PhysxDeformableBodyDesc* dst = nullptr;
    if (src->type == omni::physics::parse::eVolumeDeformableBody)
    {
        PhysxVolumeDeformableBodyDesc* vol = ICE_PLACEMENT_NEW(PhysxVolumeDeformableBodyDesc)();
        usdparser::setToDefault(units, *vol);
        const auto* psrc = static_cast<const omni::physics::parse::PhysxVolumeDeformableBodyDesc*>(src);
        vol->isAutoHexahedralMeshEnabled = psrc->isAutoHexahedralMeshEnabled;
        vol->autoHexahedralResolution    = psrc->autoHexahedralResolution;
        dst = vol;
    }
    else if (src->type == omni::physics::parse::eSurfaceDeformableBody)
    {
        PhysxSurfaceDeformableBodyDesc* surf = ICE_PLACEMENT_NEW(PhysxSurfaceDeformableBodyDesc)();
        usdparser::setToDefault(units, scanned.source(), *surf);
        const auto* psrc = static_cast<const omni::physics::parse::PhysxSurfaceDeformableBodyDesc*>(src);
        surf->collisionPairUpdateFrequency = psrc->collisionPairUpdateFrequency;
        surf->collisionIterationMultiplier = psrc->collisionIterationMultiplier;
        dst = surf;
    }

    if (!dst)
        return nullptr;

    overlayCommonFields(*dst, *src, scanned, attachedStage);

    return dst;
}

} // namespace omni::physx::usdparser::convert
