// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-1
 */

#pragma once

#include <pxr/usd/usd/stage.h>

namespace omni::physics::usd
{
class ScannedStage;
}

namespace omni::physx::usdparser
{
struct PhysxDeformableBodyDesc;
}

namespace omni::physx::usdparser::convert
{

// Translates the deformable-body descriptor at `index` in `scanned` to a
// freshly-allocated legacy `PhysxDeformableBodyDesc*`.  Returns nullptr
// when `index` is out of range or the entry is not a deformable body the
// converter knows how to translate (volume / surface).
//
// Field translation:
//  - ObjectKey -> SdfPath via ScannedStage::pathFor
//  - TokenId   -> TfToken via ScannedStage::tfTokenFor
//  - parse::Matrix4d -> GfMatrix4d via memcpy (row-major double,
//    layout-compatible)
//  - cookingSrcMeshBindPoseToken: parse-lib does not populate this
//    field; the converter reads it directly from USD on the
//    cooking-source mesh prim (mirrors the legacy at PhysicsBody.cpp
//    lines 781-786).
//
// `setToDefault` is called on the freshly-allocated desc before overlay
// so any fields the parse-lib doesn't track (`restBendAnglesDefault`,
// `contactOffset`, `restOffset`, `collisionGroup`, `sceneId`,
// `simMeshMaterial`) start at the same defaults the legacy
// parseDeformableBody would produce.
//
// Caller owns the returned desc; release via `ICE_FREE`.
PhysxDeformableBodyDesc* convertScannedDeformableBody(
    const omni::physics::usd::ScannedStage& scanned,
    size_t index,
    const omni::physics::parse::SourceUnits& units);

} // namespace omni::physx::usdparser::convert
