// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/Allocator.h>     // IDescriptorAllocator
#include <omni/physics/parse/IParseBackend.h> // SourceBundle
#include <omni/physics/parse/ScanBackend.h>   // ScanOptions
#include <omni/physics/parse/ScannedStage.h>  // ScannedStage (pulls IPhysicsSource.h)

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace omni::physics::parse
{

// Opaque per-stage session owned by the USD reparse backend (ADR-0027 seam #2).
// Defined only inside the loadable USD library; the USD-free omni.physx side
// holds it by pointer and never names its layout.
class UsdReparseSession;

// pxr-free interface the loadable USD library installs so the USD-free
// omni.physx side can open a UsdStage by id, expose its IPhysicsSource, and scan
// it — without linking or naming any pxr type. Installed once by the test loader
// (ADR-0027); null in production, where nothing reparses USD.
class IUsdReparse
{
public:
    virtual ~IUsdReparse() = default;

    // Open (or reuse) a session over the UsdStage cached under `stageId`.
    // Returns null when no such stage is available. The backend does not
    // serialize: the caller holds UsdLoad::mParsingMutex around
    // openStage/source/scan/close (every omni.physx caller does). Hold the
    // result through UsdReparseSessionPtr (openScopedStage) so an exception
    // between open and close still closes the session.
    virtual UsdReparseSession* openStage(uint64_t stageId) = 0;

    // The session's IPhysicsSource, valid for the session's lifetime.
    virtual IPhysicsSource* source(UsdReparseSession& session) = 0;

    // Scan the session's stage, producing a source-agnostic ScannedStage. Same
    // string-typed roots/excludes the USD dispatch layer already builds.
    virtual ScannedStage scan(UsdReparseSession& session,
                              const std::vector<std::string>& scanRoots,
                              const std::vector<std::string>& excludePaths,
                              const ScanOptions& options,
                              IDescriptorAllocator& allocator) = 0;

    // Release a session obtained from openStage. Null is a no-op.
    virtual void close(UsdReparseSession* session) noexcept = 0;

    // Construct a fresh USD scan backend (the loadable library's makeUsdScanBackend). Lets the
    // USD-free omni.physx side re-assert the USD scan backend into the ADR-0005 registry when a
    // plain USD attach finds it displaced (a test helper that nulled it, ADR-0010) -- the
    // per-attach install the collapsed StageBridge otherwise cannot express pxr-free. Null in
    // production, where nothing reparses USD.
    virtual std::unique_ptr<IScanBackend> makeScanBackend() = 0;

    // Build an OWNED USD source trio over the live UsdStage held at `handleStorage`
    // (the opaque UsdStageWeakPtr storage the OpaquePxrHandleOps manage), using the
    // backend's OWN private USD parse backend -- never the process-active parse
    // backend, which may be a data-plane backend (ovstage) that reads the handle
    // storage as its own payload. This is the sessionless, owned-bundle form of
    // openStage's source build: AttachedStage::rebuildUsdSource adopts the returned
    // SourceBundle directly, so a foreign USD stage bound while an ovstage attach is
    // live still parses through USD. Returns an empty bundle for null/empty storage.
    virtual SourceBundle createSourceFromHandle(void* handleStorage) = 0;

    // --- Stateless opaque-pxr-handle helpers (ADR-0027 bridge collapse) ---------------------
    // These three do not touch a session; they operate on the opaque UsdStageWeakPtr storage
    // the OpaquePxrHandleOps manage (`handleStorage` points at those bytes), letting the
    // USD-free omni.physx side resolve stage ids/paths without naming a pxr type.

    // Resolve `stageId` against the process UsdUtilsStageCache. When resident, assign the
    // resolved UsdStageWeakPtr into the already-live handle at `handleStorage` (pre-constructed
    // empty by the ops table) and return true; otherwise leave storage untouched and return
    // false. Mirrors attachForeignStage's stage-cache lookup.
    virtual bool resolveStageToHandle(uint64_t stageId, void* handleStorage) = 0;

    // Reverse of resolveStageToHandle: the process stage-cache id of the stage held at
    // `handleStorage`, or 0 when it holds an empty stage. Drives bridgeBackingStageCacheId.
    virtual uint64_t stageCacheIdForHandle(const void* handleStorage) = 0;

    // Decode a legacy `asInt(SdfPath)` bit value into its path string. Returns false for the
    // 0 sentinel. Drives bridgeParseForeignStageCollision's collisionPrimId round-trip.
    virtual bool legacyPathBitsToString(uint64_t legacyId, std::string& out) = 0;

    // Encode a path string into its legacy `asInt(SdfPath)` bit value -- the exact inverse of
    // legacyPathBitsToString (`out = asInt(SdfPath(path))`). Returns false, leaving `out`
    // untouched, for a null/empty path. Lets the USD-free PropertyQuery bridge produce the
    // SdfPath-bits values its Kit/USD-authoring-only public interface carries (IPhysxPropertyQuery
    // request primPath / response usdPath / ArticulationLink rigidBody+joint), without naming pxr.
    //
    // @implements REQ-USDLIB-LOADABLE-001
    // @covers AC-2
    virtual bool stringToLegacyPathBits(const char* path, uint64_t& out) = 0;
};

// RAII owner for a session from IUsdReparse::openStage: closes it on the backend that opened
// it when the owner dies, so a throw between open and close cannot leak the session and its
// source/write/change-feed bundle. Callers hold sessions through this, never the raw pointer.
struct UsdReparseSessionCloser
{
    IUsdReparse* reparse = nullptr;
    void operator()(UsdReparseSession* session) const noexcept
    {
        if (session && reparse)
            reparse->close(session);
    }
};
using UsdReparseSessionPtr = std::unique_ptr<UsdReparseSession, UsdReparseSessionCloser>;

// openStage bound to its owner. Empty (falsy) when no stage is cached under `stageId`.
inline UsdReparseSessionPtr openScopedStage(IUsdReparse& reparse, uint64_t stageId)
{
    return UsdReparseSessionPtr(reparse.openStage(stageId), UsdReparseSessionCloser{ &reparse });
}

// Install `reparse` as the single active USD reparse backend, replacing any
// previous one. null clears it. Must be called detached (ADR-0005/0027).
void setUsdReparse(std::unique_ptr<IUsdReparse> reparse);

// The active USD reparse backend, or null when none is installed.
IUsdReparse* usdReparse();

} // namespace omni::physics::parse
