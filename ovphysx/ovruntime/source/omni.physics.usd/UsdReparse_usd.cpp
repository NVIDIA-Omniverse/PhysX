// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

// Concrete USD reparse backend (ADR-0027 seam #2): the active, test-only USD reparse
// path behind parse::IUsdReparse. Lives entirely inside the loadable USD library so it
// may name pxr types freely; the pxr-free bridges (usdBridge/StageBridge.cpp,
// usdBridge/PropertyQueryBridge.cpp) reach it only through that interface. Installed by
// the test loader; null in production, where nothing reparses USD.

#include <omni/physics/usd/UsdReparse.h>

#include <omni/physics/usd/StageScan.h>       // ScannedStage / scanStage
#include <omni/physics/usd/UsdParseBackend.h> // makeUsdParseBackend
#include <omni/physics/usd/UsdScanBackend.h>  // makeUsdScanBackend

#include <omni/physics/parse/IParseBackend.h>   // AttachTarget / SourceBundle / IParseBackend
#include <omni/physics/parse/IPhysicsSource.h>  // DescendantScope

#include "UsdPathEncoding.h" // ::intToPath / ::asInt (legacy SdfPath bit encoding)

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <string>
#include <unordered_set>
#include <vector>

namespace omni::physics::parse
{

// Concrete definition of the opaque session the interface forward-declares. It
// owns the resolved live stage plus the USD source trio built off it; the
// UsdSource inside `bundle` is the ObjectKey minting authority for this stage.
// Held only by pointer on the omni.physx side (ADR-0027), which never names it.
class UsdReparseSession
{
public:
    PXR_NS::UsdStageWeakPtr stage;
    SourceBundle bundle;
};

} // namespace omni::physics::parse

namespace omni::physics::usd
{

namespace
{

// Local mirror of StageScan.cpp's file-local traversalForScope() (reused, not
// modified). Maps the source-agnostic descendant scope onto the native USD walk
// predicate so the reparse scan matches the load-path scan for the same scope.
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

class UsdReparse final : public parse::IUsdReparse
{
public:
    parse::UsdReparseSession* openStage(uint64_t stageId) override
    {
        // Resolve the id against the process UsdUtilsStageCache exactly as
        // attachForeignStage does. Null (no session) when it is not resident.
        PXR_NS::UsdStageWeakPtr stage =
            PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(long(stageId)));
        if (!stage)
            return nullptr;

        // Serialization is the caller's: every omni.physx caller holds UsdLoad::mParsingMutex
        // around openStage/source/scan/close. That mutex is omni.physx runtime state
        // (usdLoad/LoadUsd.h); this library deliberately does not reach into omni.physx state.

        auto session = std::make_unique<parse::UsdReparseSession>();
        session->stage = stage;

        // Build the source through a PRIVATE USD parse backend, never the active
        // process backend (which may be a data-plane backend that reads
        // nativeStage as its own payload) -- mirrors AttachedStage::rebuildUsdSource.
        parse::AttachTarget target;
        target.nativeStage = &session->stage;
        session->bundle = mUsdBackend->createSource(target);
        if (!session->bundle.source)
            return nullptr;

        return session.release();
    }

    parse::IPhysicsSource* source(parse::UsdReparseSession& session) override
    {
        return session.bundle.source.get();
    }

    parse::ScannedStage scan(parse::UsdReparseSession& session,
                             const std::vector<std::string>& scanRoots,
                             const std::vector<std::string>& excludePaths,
                             const parse::ScanOptions& options,
                             parse::IDescriptorAllocator& allocator) override
    {
        // Rebuild the string-typed roots/excludes into SdfPath and run the
        // native USD walk over the session's own stage (same walk scanTargetNative
        // drives), allocating descriptors from the caller's allocator.
        std::vector<PXR_NS::SdfPath> roots;
        roots.reserve(scanRoots.size());
        for (const std::string& root : scanRoots)
            roots.emplace_back(root);

        std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> excludes;
        excludes.reserve(excludePaths.size());
        for (const std::string& path : excludePaths)
            excludes.emplace(path);

        ScannedStage scanned =
            scanStage(session.stage, roots, excludes, allocator, traversalForScope(options.descendantScope));

        // Slice-move the USD view down to the source-agnostic base the interface
        // returns; the pxr-typed resolvers are not part of the seam contract.
        return parse::ScannedStage(std::move(scanned));
    }

    void close(parse::UsdReparseSession* session) noexcept override
    {
        delete session; // stage + source trio destroyed inside this USD library
    }

    std::unique_ptr<parse::IScanBackend> makeScanBackend() override
    {
        return omni::physics::usd::makeUsdScanBackend();
    }

    parse::SourceBundle createSourceFromHandle(void* handleStorage) override
    {
        if (!handleStorage)
            return {};
        // handleStorage points at a live UsdStageWeakPtr (the ops-table storage). The
        // UsdParseBackend reads AttachTarget::nativeStage as a UsdStageWeakPtr*, which is
        // exactly this address -- same contract as openStage's target.nativeStage =
        // &session->stage and AttachedStage::rebuildUsdSource's mStage.storage(). Built
        // through the PRIVATE USD backend, so it bypasses the process-active backend.
        parse::AttachTarget target;
        target.nativeStage = handleStorage;
        return mUsdBackend->createSource(target);
    }

    // --- Stateless opaque-pxr-handle helpers (ADR-0027 bridge collapse) --------------------

    bool resolveStageToHandle(uint64_t stageId, void* handleStorage) override
    {
        // Same stage-cache lookup attachForeignStage performs.
        PXR_NS::UsdStageWeakPtr stage =
            PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(long(stageId)));
        if (!stage)
            return false;
        // Assign INTO the already-live UsdStageWeakPtr the ops table pre-constructed at
        // handleStorage -- never a fresh placement-new, which would leak that one.
        *reinterpret_cast<PXR_NS::UsdStageWeakPtr*>(handleStorage) = stage;
        return true;
    }

    uint64_t stageCacheIdForHandle(const void* handleStorage) override
    {
        const PXR_NS::UsdStageWeakPtr& stage = *reinterpret_cast<const PXR_NS::UsdStageWeakPtr*>(handleStorage);
        if (!stage)
            return 0;
        return static_cast<uint64_t>(PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt());
    }

    bool legacyPathBitsToString(uint64_t legacyId, std::string& out) override
    {
        if (legacyId == 0)
            return false;
        out = ::intToPath(legacyId).GetString();
        return true;
    }

    bool stringToLegacyPathBits(const char* path, uint64_t& out) override
    {
        if (!path || !*path)
            return false;
        // Inverse of legacyPathBitsToString: reinterpret the SdfPath's own memory layout as a
        // uint64_t (::asInt, UsdPathEncoding.h -- the SdfPathEncoding ABI trick).
        out = ::asInt(PXR_NS::SdfPath(path));
        return true;
    }

private:
    // Each backend owns its own private USD parse backend, mirroring the static
    // sUsdStageBackend in AttachedStage::rebuildUsdSource.
    std::unique_ptr<parse::IParseBackend> mUsdBackend = makeUsdParseBackend();
};

} // namespace

std::unique_ptr<omni::physics::parse::IUsdReparse> makeUsdReparse()
{
    return std::make_unique<UsdReparse>();
}

} // namespace omni::physics::usd
