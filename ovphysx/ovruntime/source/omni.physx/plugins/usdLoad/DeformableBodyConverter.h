// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-1
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-29
 */

#pragma once

// PhysxDeformableBodyDesc is a `using` alias of the parse-lib type
// (PhysxUsd.h, ADR-0019 increment 7) -- a forward struct-declaration would
// conflict with the alias, so the full header is included instead of a
// forward declaration here.
#include <private/omni/physx/PhysxUsd.h>

// PhysxDeformableBodyDesc is a `using` alias of the parse-lib type
// (PhysxUsd.h, ADR-0019 increment 7) -- a forward struct-declaration would
// conflict with the alias, so the full header is included instead of a
// forward declaration here.
#include <private/omni/physx/PhysxUsd.h>

namespace omni::physics::parse
{
class ScannedStage;
}

namespace omni::physx::usdparser
{
class AttachedStage;
} // namespace omni::physx::usdparser

namespace omni::physx::usdparser::convert
{

// Translates the deformable-body descriptor at `index` in `scanned` to a
// freshly-allocated `PhysxDeformableBodyDesc*` (now the same type as the
// scanned entry itself -- ADR-0019 increment 7). Returns nullptr when
// `index` is out of range or the entry is not a deformable body the
// converter knows how to translate (volume / surface).
//
// `setToDefault` is called on the freshly-allocated desc before overlay
// so any fields the parse-lib doesn't track (`restBendAnglesDefault`,
// `contactOffset`, `restOffset`, `collisionGroup`, `sceneId`,
// `simMeshMaterial`) start at the same defaults the legacy
// parseDeformableBody would produce.
//
// `attachedStage` re-keys the ObjectKey/TokenId-typed fields (simMeshKey /
// collisionMeshKey / skinGeomPaths / cookingSrcMeshKey / *BindPoseToken) from
// `scanned`'s own (throwaway, parse-time) key namespace into `attachedStage`'s
// persistent one via the source's string identity (path text / token text)
// -- ADR-0004's key-space invariant: a `ScannedStage`'s `ObjectKey`s are only
// meaningful against that same `ScannedStage`'s source, never directly
// against another attach's keyFor output. Downstream consumers
// (UsdInterfaceDeformable.cpp, CookingDataAsync.cpp, PhysXAttachment.cpp)
// resolve these fields through `attachedStage`, so they must already be in
// its namespace. `scanned` takes the backend-agnostic `parse::ScannedStage`
// base; the rekeying is the usual opaque-identity round trip
// (`scanned.source().sourceKeyToString()` / `attachedStage.keyFor(string_view)`).
//
// Caller owns the returned desc; release via `ICE_FREE`.
PhysxDeformableBodyDesc* convertScannedDeformableBody(
    const omni::physics::parse::ScannedStage& scanned,
    size_t index,
    const omni::physics::parse::SourceUnits& units,
    const AttachedStage& attachedStage);

} // namespace omni::physx::usdparser::convert
