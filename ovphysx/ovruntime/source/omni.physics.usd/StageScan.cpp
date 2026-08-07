// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * Whole-stage scan implementation — see ADR-0002 for the architecture
 * and ADR-0003 for the native USD driver design. This file is the
 * thin public-API shell around `ScannedStage` / `scanStage`; prim
 * discovery + classification + per-concept descriptor production
 * live in `NativeWalker.cpp`.
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-11 AC-12 AC-13
 *
 * @implements REQ-PARSE-ART-002
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6
 *
 * @implements REQ-PARSE-DEF-001
 * @covers AC-6
 */

#include <pxr/base/gf/transform.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/layerUtils.h>
#include <pxr/usd/usd/collectionMembershipQuery.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <physxSchema/physxSceneAPI.h>

#include <omni/physics/usd/StageScan.h>

#include "NativeWalker.h"
#include "UsdSource.h" // asUsdSource — USD-source fast resolver path

#include <omni/physics/usd/PrimIterator.h>

namespace omni::physics::usd
{
using namespace omni::physics::parse;

// `ScannedStage` (the USD view) defaults its ctors/move in the header. Its
// resolvers route through the backend source: a `UsdSource` resolves directly
// (O(1) interned-table lookup, no string parse); any other source (ovstage)
// reconstructs the SdfPath / TfToken from the source's path/token string and
// lazily memoizes it.

PXR_NS::SdfPath ScannedStage::pathFor(parse::ObjectKey key) const
{
    const parse::IPhysicsSource* src = sourcePtr();
    if (!src)
        return {};
    if (const UsdSource* usd = asUsdSource(src))
        return usd->pathFor(key); // interned, string-free

    auto it = mPathCache.find(key);
    if (it != mPathCache.end())
        return it->second;
    const std::string_view s = src->sourceKeyToString(key);
    PXR_NS::SdfPath path = s.empty() ? PXR_NS::SdfPath() : PXR_NS::SdfPath(std::string(s));
    mPathCache.emplace(key, path);
    return path;
}

parse::ObjectKey ScannedStage::keyFor(const PXR_NS::SdfPath& path) const
{
    const parse::IPhysicsSource* src = sourcePtr();
    if (!src)
        return {};
    if (const UsdSource* usd = asUsdSource(src))
        return usd->keyFor(path);
    return src->findByPath(path.GetString());
}

PXR_NS::TfToken ScannedStage::tfTokenFor(parse::TokenId id) const
{
    const parse::IPhysicsSource* src = sourcePtr();
    if (!src)
        return {};
    if (const UsdSource* usd = asUsdSource(src))
        return usd->tfTokenFor(id); // interned, string-free

    auto it = mTokenCache.find(id);
    if (it != mTokenCache.end())
        return it->second;
    const std::string_view s = src->tokenToString(id);
    PXR_NS::TfToken token = s.empty() ? PXR_NS::TfToken() : PXR_NS::TfToken(std::string(s));
    mTokenCache.emplace(id, token);
    return token;
}

// ---------------------------------------------------------------------------
// Public scanStage entry points — delegate to scanStageNative.
// ---------------------------------------------------------------------------

ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       omni::physics::schema::PrimIteratorBase& primIterator,
                       parse::IDescriptorAllocator& allocator)
{
    if (!stage)
        return {};
    return scanStageNative(stage, primIterator, allocator);
}

ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       parse::IDescriptorAllocator& allocator)
{
    if (!stage)
        return {};

    PXR_NS::UsdPrimRange range = PXR_NS::UsdPrimRange::AllPrims(stage->GetPseudoRoot());
    omni::physics::schema::PrimIteratorRange primIterator(range);
    return scanStage(stage, primIterator, allocator);
}

ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       const PXR_NS::SdfPath& root,
                       parse::IDescriptorAllocator& allocator,
                       SubtreeTraversal traversal)
{
    if (!stage)
        return {};

    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(root);
    if (!prim)
        return {};

    PXR_NS::UsdPrimRange range = [&]
    {
        switch (traversal)
        {
        case SubtreeTraversal::eDefault:
            // Plain default predicate, no instance proxies.
            return PXR_NS::UsdPrimRange(prim);
        case SubtreeTraversal::eAllPrims:
            // Every prim (incl. inactive / abstract / unloaded).
            return PXR_NS::UsdPrimRange(prim, PXR_NS::UsdPrimAllPrimsPredicate);
        case SubtreeTraversal::eInstanceProxies:
        default:
            // Default predicate, descending into instance proxies.
            return PXR_NS::UsdPrimRange(prim, PXR_NS::UsdTraverseInstanceProxies(
                PXR_NS::UsdPrimIsActive && PXR_NS::UsdPrimIsDefined && PXR_NS::UsdPrimIsLoaded &&
                !PXR_NS::UsdPrimIsAbstract));
        }
    }();
    omni::physics::schema::PrimIteratorRange primIterator(range);
    return scanStage(stage, primIterator, allocator);
}

PXR_NS::SdfPath createDefaultPhysicsScene(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& scenePath)
{
    if (!stage || scenePath.IsEmpty())
        return {};

    // Author on the session layer (matches the legacy ScopedLayerEdit). The
    // prim spec (with its UsdPhysicsScene type), PhysxSceneAPI, and the hidden /
    // no-delete session metadata are all created within this edit context.
    // UsdPhysicsScene::Define already authors the prim spec (and any missing
    // ancestors) on the edit target, so no separate SdfJustCreatePrimInLayer is
    // needed.
    PXR_NS::UsdEditContext editContext(stage, stage->GetSessionLayer());

    PXR_NS::UsdPhysicsScene scene = PXR_NS::UsdPhysicsScene::Define(stage, scenePath);
    if (!scene)
        return {};
    PXR_NS::PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());

    static const PXR_NS::TfToken kHideInStageWindow("hide_in_stage_window");
    static const PXR_NS::TfToken kNoDelete("no_delete");
    PXR_NS::UsdPrim prim = scene.GetPrim();
    prim.SetMetadata(kHideInStageWindow, true);
    prim.SetMetadata(kNoDelete, true);
    return scenePath;
}

SubtreeTraversal traversalForScope(parse::DescendantScope scope)
{
    switch (scope)
    {
    case parse::DescendantScope::eAll:
        return SubtreeTraversal::eAllPrims;
    case parse::DescendantScope::eActive:
        return SubtreeTraversal::eDefault;
    case parse::DescendantScope::eActiveInstanced:
    default:
        return SubtreeTraversal::eInstanceProxies;
    }
}

ScannedStage scanStage(PXR_NS::UsdStageWeakPtr stage,
                       const std::vector<PXR_NS::SdfPath>& roots,
                       const std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& excludePaths,
                       parse::IDescriptorAllocator& allocator,
                       SubtreeTraversal traversal)
{
    if (!stage || roots.empty())
        return {};

    if (!excludePaths.empty())
    {
        // Replicator selective load: a single whole-stage walk from roots.front()
        // that skips every prim in excludePaths (subtree inclusive). The two
        // modes are mutually exclusive — exclude-based load is always single-root
        // (the pseudo-root); additional roots would be silently ignored.
        CARB_ASSERT(roots.size() == 1 &&
            "scanStage: excludePaths is only supported with a single root (replicator selective load)");
        PXR_NS::UsdPrim rootPrim = stage->GetPrimAtPath(roots.front());
        if (!rootPrim)
            return {};
        // Whole-stage replicator load is always the instance-proxy predicate.
        PXR_NS::UsdPrimRange range(rootPrim, PXR_NS::UsdTraverseInstanceProxies(
            PXR_NS::UsdPrimIsActive && PXR_NS::UsdPrimIsDefined && PXR_NS::UsdPrimIsLoaded &&
            !PXR_NS::UsdPrimIsAbstract));
        omni::physics::schema::PrimIteratorExcludeRange primIterator(std::move(range), excludePaths);
        return scanStage(stage, primIterator, allocator);
    }

    if (roots.size() == 1 && traversal != SubtreeTraversal::eInstanceProxies)
        return scanStage(stage, roots.front(), allocator, traversal);

    // Multi-root (or single-root) walk, each root descended with instance proxies.
    // PrimIteratorMapRange resolves the prims and walks each with the same
    // predicate; ordering is the map's sorted-path order.
    (void)traversal; // both supported shapes use the instance-proxy predicate
    std::set<PXR_NS::SdfPath> rootSet(roots.begin(), roots.end());
    omni::physics::schema::PrimIteratorMapRange primIterator(rootSet, stage);
    return scanStage(stage, primIterator, allocator);
}

// Backend-dispatched whole-stage scan (ADR-0002 M2c) — the single switch point.
ScannedStage scanStage(const parse::AttachTarget& target,
                       const std::vector<PXR_NS::SdfPath>& scanRoots,
                       const std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& excludePaths,
                       parse::IDescriptorAllocator& allocator,
                       const parse::ScanOptions& options)
{
    if (IScanBackend* backend = scanBackend())
    {
        std::vector<std::string> rootStrings;
        rootStrings.reserve(scanRoots.size());
        for (const PXR_NS::SdfPath& root : scanRoots)
            rootStrings.push_back(root.GetString());

        std::vector<std::string> excludeStrings;
        excludeStrings.reserve(excludePaths.size());
        for (const PXR_NS::SdfPath& path : excludePaths)
            excludeStrings.push_back(path.GetString());

        return backend->scan(target, rootStrings, excludeStrings, options, allocator);
    }

    // Default: the native USD walk over the live stage handle.
    const PXR_NS::UsdStageWeakPtr stage =
        target.nativeStage ? *static_cast<const PXR_NS::UsdStageWeakPtr*>(target.nativeStage) : PXR_NS::UsdStageWeakPtr{};
    return scanStage(stage, scanRoots, excludePaths, allocator, traversalForScope(options.descendantScope));
}

} // namespace omni::physics::usd
