// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * Whole-stage scan API — produces a `ScannedStage` snapshot of a USD
 * stage's parsed physics state.
 *
 * `omni::physics::usd::ScannedStage` derives from the USD-free
 * `omni::physics::parse::ScannedStage` (which owns the descriptors and the
 * backend source) and adds USD-typed (`SdfPath`/`TfToken`) handle resolvers
 * for USD consumers. The source-agnostic scan container, the backend builder
 * (`makeScannedStageFromSource`), and the scan-backend registry all live in
 * the parse core so non-USD backends (e.g. ovstage) can build a scan without
 * depending on this USD layer.
 *
 * Architectural cap that the runtime's USD-load path is built on.
 * See `ovphysx/ovruntime/plc/adr/ADR-0002-ovstage-parse-source.md`.
 *
 * Driven by the native USD prim walker inside `omni.physics.usd`
 * (`NativeWalker.cpp`); see ADR-0003 for the walker design.
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-2 AC-3
 */

#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IParseBackend.h> // parse::AttachTarget (backend-dispatched scanStage)
#include <omni/physics/parse/ScanBackend.h>   // parse::IScanBackend / setScanBackend / scanBackend
#include <omni/physics/parse/ScannedStage.h>  // parse::ScannedStage (base) + makeScannedStageFromSource
#include <omni/physics/usd/PrimIterator.h>

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::usd
{
// (Parse-lib types are referenced explicitly as `parse::...`; no namespace-
// scope `using namespace` in this public header.)

class ScannedStage;

// Free-function entry points. The `allocator` is propagated to every
// descriptor allocated by the scan; consumers inject their own backend
// allocator (e.g. `iceDescriptorAllocator()` from the omni.physx runtime)
// so descriptors can cross the parse-lib → consumer boundary without a
// deep copy. The allocator must outlive the returned `ScannedStage`
// and any descriptors released from it — `DescPtr`s in `ScannedStage`
// hold raw pointers to the allocator for their deleters.
ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       parse::IDescriptorAllocator& allocator);
ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       omni::physics::schema::PrimIteratorBase& primIterator,
                       parse::IDescriptorAllocator& allocator);

// Traversal predicate for the subtree `scanStage` overload below. The three
// modes correspond to the distinct root-scoped walks consumers need; a single
// bool could not express all of them.
enum class SubtreeTraversal
{
    // active + defined + loaded + non-abstract, descending into instance
    // proxies. The instancer-prototype scan (PointInstancer / JointInstancer).
    eInstanceProxies,
    // Plain `UsdPrimRange(root)`: the USD default predicate (active + defined +
    // loaded + non-abstract) WITHOUT instance-proxy descent — change / property
    // re-scans, which must not descend into instance proxies.
    eDefault,
    // Every prim regardless of active / loaded / abstract state, matching the
    // whole-stage `UsdPrimRange::AllPrims` traversal — the cooking driver's
    // subtree re-parse, so it mirrors the main parse pipeline exactly.
    eAllPrims
};

// Subtree entry point: scan the prims under `root` (root inclusive). Lets
// consumers that hold only a root path (no materialized UsdPrim / hand-built
// PrimIteratorRange) drive a scoped scan; the prim + range are constructed
// here in the USD layer. `traversal` selects the walk predicate — see
// `SubtreeTraversal`.
ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       const PXR_NS::SdfPath& root,
                       parse::IDescriptorAllocator& allocator,
                       SubtreeTraversal traversal = SubtreeTraversal::eInstanceProxies);

// Multi-root + optional-exclude entry point. Scans the union of the subtrees
// under each path in `roots` (root inclusive). When `excludePaths` is non-empty
// the scan is treated as a single whole-stage walk from `roots.front()` that
// skips every prim in `excludePaths` together with its subtree (the replicator's
// selective-load semantics); when it is empty each root is walked independently
// (the prim-update multi-root case). Lets consumers drive the scan from root
// paths alone — no materialized UsdPrim / hand-built PrimIteratorRange — keeping
// all USD traversal inside the backend. `traversal` is eInstanceProxies for both
// supported shapes.
ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       const std::vector<PXR_NS::SdfPath>& roots,
                       const std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& excludePaths,
                       parse::IDescriptorAllocator& allocator,
                       SubtreeTraversal traversal = SubtreeTraversal::eInstanceProxies);

// Whole-stage scan with backend dispatch (ADR-0002 M2c). The single switch
// point: if a scan backend is registered (e.g. ovstage, via
// `parse::setScanBackend`), the scan is produced by it; otherwise the native
// USD walk runs over `target.nativeStage` (interpreted as the live
// `UsdStageWeakPtr`) with `scanRoots` / `excludePaths`. Callers (the load path)
// call this one function with `AttachedStage::attachTarget()` and never branch
// on the backend. Registered backends receive `scanRoots` / `excludePaths` as
// source-path strings so they can preserve selective-load and incremental-scan
// semantics without depending on USD types.
ScannedStage scanStage(const parse::AttachTarget& target,
                       const std::vector<PXR_NS::SdfPath>& scanRoots,
                       const std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& excludePaths,
                       parse::IDescriptorAllocator& allocator,
                       const parse::ScanOptions& options = parse::ScanOptions{});

// Author a fallback default UsdPhysicsScene (with PhysxSceneAPI applied) on the
// stage's session layer at `scenePath`, tagged hidden + no-delete (session-layer
// metadata). Returns `scenePath` (empty on failure). The loader calls this when a
// stage carries no physics scene — scene authoring is a USD operation, so it
// lives here in the USD backend rather than in omni.physx.
PXR_NS::SdfPath createDefaultPhysicsScene(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& scenePath);

// USD view of a scanned stage: the source-agnostic `parse::ScannedStage`
// (descriptor lists + backend source) plus USD-typed handle resolvers.
//
// `ObjectKey` / `TokenId` values on the descriptors are minted by this scan's
// source and must be resolved through THIS instance's `pathFor` / `keyFor` /
// `tfTokenFor` — they are not portable across `ScannedStage` instances.
//
// Resolution cost: when the backend source is the native `UsdSource`, `pathFor`
// / `tfTokenFor` are O(1) interned-table lookups returning interned
// `SdfPath` / `TfToken` (no string parsing). For a non-USD source (ovstage),
// the `SdfPath` / `TfToken` is reconstructed from the source's path/token
// string and lazily memoized, so the string parse happens once per unique
// handle, not once per call.
class ScannedStage : public parse::ScannedStage
{
public:
    ScannedStage() = default;

    // Adopt a freshly-built source-agnostic scan as the USD view of it. The
    // native walker and the backend dispatcher build a `parse::ScannedStage`
    // and hand it here.
    ScannedStage(parse::ScannedStage&& base) noexcept : parse::ScannedStage(std::move(base)) {}

    ScannedStage(ScannedStage&&) noexcept = default;
    ScannedStage& operator=(ScannedStage&&) noexcept = default;
    ScannedStage(const ScannedStage&) = delete;
    ScannedStage& operator=(const ScannedStage&) = delete;

    // Resolve any `ObjectKey` produced by this scan back to its source path.
    // Returns an empty path when the key is invalid or unknown to this scan.
    PXR_NS::SdfPath pathFor(parse::ObjectKey key) const;

    // Resolve a source path back to the `ObjectKey` minted for it during this
    // scan. Returns the invalid sentinel when the path was not visited.
    parse::ObjectKey keyFor(const PXR_NS::SdfPath& path) const;

    // Resolve a `TokenId` minted by this scan back to its `TfToken`. Returns
    // the empty token when the id is invalid or unknown.
    PXR_NS::TfToken tfTokenFor(parse::TokenId id) const;

private:
    // Lazy resolution caches, used only when the source is NOT a UsdSource
    // (the native USD path resolves directly through UsdSource's own interned
    // tables). `mutable` because resolution is logically const.
    mutable std::unordered_map<parse::ObjectKey, PXR_NS::SdfPath, parse::ObjectKey::Hash> mPathCache;
    mutable std::unordered_map<parse::TokenId, PXR_NS::TfToken, parse::TokenId::Hash>     mTokenCache;
};

} // namespace omni::physics::usd
