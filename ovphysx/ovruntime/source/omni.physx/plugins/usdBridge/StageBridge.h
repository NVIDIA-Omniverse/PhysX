// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

#pragma once

// pxr-free declarations for the stage-attach USD bridge (ADR-0027), implemented by
// usdBridge/StageBridge.cpp. USD behaviour is reached only through the installed seams.
//

#include <usdLoad/ScannedCollision.h>
#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IParseBackend.h>

#include "usdLoad/AttachedStage.h"

#include <memory>
#include <string>

namespace omni::physics::parse
{
struct PhysxShapeDesc;
class ScannedStage;
}

namespace omni::physx::usdparser
{
class AttachedStage;
using PhysxShapeDesc = ::omni::physics::parse::PhysxShapeDesc;

// --- shared scan helpers (defined unconditionally in LoadUsd.cpp) ------------------
// Exposed so the USD arm's SdfPath-taking selectScannedCollision sibling can reuse
// them instead of duplicating the re-keying/cooking translation.

// Per-shape consumer-side bridge: cooking dispatch, ObjectKey re-keying across the
// scanStage / attachedStage source namespaces, and consumer state translation
// (sceneIds / materials / filteredPairs / collisionGroup). Equivalent to
// ignoreOwners=true semantics: a shape with simulationOwners-but-no-resolvable-scene
// is kept rather than dropped.
void translateScannedShape(AttachedStage& attachedStage,
                           const ::omni::physics::parse::ScannedStage& scanned,
                           PhysxShapeDesc* desc,
                           ::omni::physics::parse::ObjectKey shapeKey);

// Matches the gprim by `sourceGprim == gPrimKey || primKey == gPrimKey` (latter for
// shapes where the gprim IS the collider), re-keying each candidate through the scan
// source's string identity. Borrows the match; the caller must retain `scanned`.
PhysxShapeDesc* selectScannedCollision(AttachedStage& attachedStage,
                                       ::omni::physics::parse::ScannedStage& scanned,
                                       ::omni::physics::parse::ObjectKey gPrimKey);

// --- seam-routed bridge entry points -----------------------------------------------

// UsdUtilsStageCache id of `handle`, or 0 with no reparse seam installed (no handle ever
// names a resident stage then). Used by attachOvstage's backing-stage consistency check.
uint64_t bridgeBackingStageCacheId(AttachedStageUsdHandle handle);

// Resolves `stageId` through the reparse seam and builds a fresh AttachedStage over it.
// Returns false (outAttachedStage untouched) with no seam or a non-resident id. Takes
// UsdLoad's parsing mutex for the construction.
bool bridgeAttachStageById(uint64_t stageId, AttachedStage& outAttachedStage);

// Scans the single collision root named by `collisionKey` and returns the shape matching
// `gPrimKey` together with its scan owner, or an empty result. Dispatches through
// the scan backend owned by this attachment (USD or ovstage).
ScannedCollision bridgeScanCollisionRoot(AttachedStage& attachedStage,
                                        ::omni::physics::parse::ObjectKey collisionKey,
                                        ::omni::physics::parse::ObjectKey gPrimKey);

// Foreign-stage collision-representation fallback: parses a collision prim on a stage that
// is named by UsdUtilsStageCache id but has never been attached through the attach/backend
// registry. `outStorage` must outlive the returned desc AND the caller's later
// IPhysicsSource reads off it -- ObjectKey is per-AttachedStage-instance state, so a key
// minted here does not resolve against any other instance. `collisionPrimId` keeps its OLD
// legacy asInt()-encoded-SdfPath-bits meaning in this arm only (an explicit carve-out in
// IPhysxCooking.h's doc comment): nobody has ever attached this stage, so no live Source
// could have minted a real ObjectKey.handle for it. Needs the reparse seam (a live
// UsdStageCache); without it, logs and returns null.
//
ScannedCollision bridgeParseForeignStageCollision(uint64_t stageId,
                                                 uint64_t collisionPrimId,
                                                 AttachedStage& outStorage,
                                                 ::omni::physics::parse::ObjectKey& outCollisionKey);

} // namespace omni::physx::usdparser
