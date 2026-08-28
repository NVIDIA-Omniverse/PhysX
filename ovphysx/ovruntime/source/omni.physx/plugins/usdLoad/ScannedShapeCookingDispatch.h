// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-2 AC-3 AC-4
 */

#pragma once

// Bridges a `omni::physics::usd::ScannedStage` snapshot to the
// legacy cooking pipeline + consumer-side state translation.
//
// Background.  scanStage produces parse-library shape descriptors with
// every USD-readable field populated, but mesh-type descriptors still
// need cooking-service dispatch to fill `crc` / `meshKey` — the cooking
// service writes these via an `onFinished` callback after computing
// the cooked-data CRC.  Bounding-shape mesh approximations also need
// a cooking pass (`PxComputeBoundingSphere` / `createOBB`).  Both flows
// are USD-coupled and stay consumer-side per ADR-0001 §13 and the
// REQ-PARSE-SCAN-001 AC-9/AC-10 documented gaps.
//
// Consumer pattern:
//   1. `scanStage(attachTarget, roots)` produces ScannedStage.
//   2. For each entry in `scanned.shapes`:
//      a. `dispatchScannedShapeCooking(...)` — fill crc/meshKey on
//         cooking-service-driven mesh types.  No-op for simple shapes.
//      b. `resolveConsumerSideShapeState(...)` — translate the
//         parse-library `source*` ObjectKey lists into `sceneIds` /
//         `materials` / `filteredPairs`.  Returns false on the
//         "no resolvable scene" gate.
//      c. Consumer-side registration: `finalizeShape`, parent-walk
//         transform fixup, `createShape` / `findOrCreateEntry` etc.
//
// Master / instance handling: callers maintain a `ShapeCookingCache`
// per parsing batch and consult it before dispatching cooking so
// instances share the master's already-cooked desc.

#include <omni/physics/parse/Handles.h>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <unordered_map>

namespace omni::physics::parse
{
struct PhysxShapeDesc;
}

namespace omni::physics::usd
{
class ScannedStage;
}

namespace omni::physx::usdparser
{
class AttachedStage;
using PhysxShapeDesc = ::omni::physics::parse::PhysxShapeDesc;
using CollisionBlockPair = std::pair<PXR_NS::SdfPath, PXR_NS::SdfPath>;
using CollisionPairVector = std::vector<CollisionBlockPair>;
}

namespace omni::physx::usdparser::scan
{

// Per-batch cache mapping a source-gprim ObjectKey to its cooked desc.
// Used by consumers iterating `scanned.shapes` to deduplicate cooking
// across master/instance pairs.  Stores non-owning pointers — the
// `unique_ptr` in `scanned.shapes` is the owner.
//
// Header-only intentionally: the methods are simple wrappers on
// `std::unordered_map` and exposing them inline makes the cache
// reachable from test binaries that don't link the cpp source.
class ShapeCookingCache
{
public:
    PhysxShapeDesc* find(omni::physics::parse::ObjectKey sourceGprim) const
    {
        auto it = mDescs.find(sourceGprim.handle);
        return it != mDescs.end() ? it->second : nullptr;
    }
    void insert(omni::physics::parse::ObjectKey sourceGprim, PhysxShapeDesc* desc)
    {
        if (sourceGprim.handle)
            mDescs[sourceGprim.handle] = desc;
    }
    void clear()
    {
        mDescs.clear();
    }
    bool empty() const { return mDescs.empty(); }

private:
    std::unordered_map<uint64_t, PhysxShapeDesc*> mDescs;
};

// Dispatch the cooking-service request for a mesh-type shape desc; on
// completion the cooking service fires the `onFinished` lambda which
// writes `crc` / `meshKey` onto `desc` and updates `gMeshKeyCache`.
//
// No-op for non-mesh-cooking-service types (simple shapes, custom
// non-merge, bounding shapes).  Bounding shapes need a separate
// path (`createBoundingSphere` / `createOBB`); consumers handle those
// via the legacy fallback for now.
//
// The cooking request flags `kComputeAsynchronously=false` so this call
// is synchronous — the lambda captures `desc` by raw pointer and
// completes before the function returns.  Mirrors
// `usdLoad/Collision.cpp::processMeshCollision` for the four
// cooking-service-driven mesh types (Convex / Triangle / ConvexDecomp /
// SphereFill).
void dispatchScannedShapeCooking(
    AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned,
    PhysxShapeDesc* desc);

// Translate the parse-lib desc's source-side ObjectKey lists into the
// legacy USD-typed fields the consumer needs for registration:
//   - sourceSimulationOwners (ObjectKey list)  →  desc->sceneIds (ObjectId list, ObjectDatabase lookup)
//   - sourceMaterials (ObjectKey list)         →  outMaterials (SdfPath list, finalizeShape input)
//   - sourceFilteredCollisions (ObjectKey list) → outFilteredPairs (pairs with primKey)
//
// Returns false in the same case `fillPhysxShapeDesc` returns false:
// simulationOwners non-empty but no scene resolved.  Caller should
// drop the shape on false (matches legacy gate).
//
// Mirrors `usdLoad/Collision.cpp::fillPhysxShapeDesc` lines 338-341 +
// 516-526.
bool resolveConsumerSideShapeState(
    AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned,
    PhysxShapeDesc* desc,
    PXR_NS::SdfPathVector& outMaterials,
    CollisionPairVector& outFilteredPairs);

} // namespace omni::physx::usdparser::scan
