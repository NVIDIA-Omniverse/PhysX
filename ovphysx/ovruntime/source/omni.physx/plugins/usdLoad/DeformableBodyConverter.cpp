// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-1
 */

#include "UsdPCH.h"

#include <common/foundation/Allocator.h>
#include <private/omni/physx/PhysxUsd.h>

#include "AttachedStage.h"
#include "DeformableBodyConverter.h"
#include "PhysicsBody.h"

#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/usd/StageScan.h>

#include <pxr/usd/usd/stage.h>

#include <cstring>

using namespace PXR_NS;

namespace omni::physx::usdparser::convert
{

namespace
{

// Copies fields common to both volume and surface bodies.  Caller has
// already placement-new'd `dst` and called `setToDefault`, so any field
// the parse-lib doesn't track keeps its default.
void overlayCommonFields(PhysxDeformableBodyDesc& dst,
                         const omni::physics::parse::PhysxDeformableBodyDesc& src,
                         const omni::physics::usd::ScannedStage& scanned)
{
    dst.bodyEnabled  = src.bodyEnabled;
    dst.kinematicBody = src.kinematicBody;
    dst.startsAsleep  = src.startsAsleep;
    dst.mass          = src.mass;

    // parse::Matrix4d is 16 row-major doubles, layout-compatible with GfMatrix4d.
    static_assert(sizeof(dst.transform) == sizeof(src.transform),
                  "GfMatrix4d / parse::Matrix4d layout mismatch");
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

    // SdfPath / TfToken-typed fields — translate via ScannedStage helpers.
    dst.simMeshPath = scanned.pathFor(src.simMeshKey);
    dst.simMeshBindPoseToken = scanned.tfTokenFor(src.simMeshBindPoseToken);
    dst.simMeshLeftHandedOrientation = src.simMeshLeftHandedOrientation;

    dst.collisionMeshPath = scanned.pathFor(src.collisionMeshKey);
    dst.collisionMeshBindPoseToken = scanned.tfTokenFor(src.collisionMeshBindPoseToken);
    dst.collisionMeshLeftHandedOrientation = src.collisionMeshLeftHandedOrientation;

    dst.skinGeomPaths.clear();
    dst.skinGeomPaths.reserve(src.skinGeomPaths.size());
    for (const auto& k : src.skinGeomPaths)
        dst.skinGeomPaths.push_back(scanned.pathFor(k));
    dst.skinGeomBindPoseTokens.clear();
    dst.skinGeomBindPoseTokens.reserve(src.skinGeomBindPoseTokens.size());
    for (const auto& t : src.skinGeomBindPoseTokens)
        dst.skinGeomBindPoseTokens.push_back(scanned.tfTokenFor(t));

    dst.cookingSrcMeshPath = scanned.pathFor(src.cookingSrcMeshKey);
    dst.cookingSrcMeshBindPoseToken = scanned.tfTokenFor(src.cookingSrcMeshBindPoseToken);
}

} // namespace

PhysxDeformableBodyDesc* convertScannedDeformableBody(
    const omni::physics::usd::ScannedStage& scanned,
    size_t index,
    const omni::physics::parse::SourceUnits& units)
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
        usdparser::setToDefault(units, *surf);
        const auto* psrc = static_cast<const omni::physics::parse::PhysxSurfaceDeformableBodyDesc*>(src);
        surf->collisionPairUpdateFrequency = psrc->collisionPairUpdateFrequency;
        surf->collisionIterationMultiplier = psrc->collisionIterationMultiplier;
        dst = surf;
    }

    if (!dst)
        return nullptr;

    overlayCommonFields(*dst, *src, scanned);

    return dst;
}

} // namespace omni::physx::usdparser::convert
