// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

// pxr-free stage-attach bridge (ADR-0027): names no pxr type, includes no pxr header, calls
// no omni::physics::usd:: symbol. With no seam installed (production / USD-free) every entry
// fails closed; a process that installed the USD library reaches USD data only through the
// seams (the parse/scan registry, parse::usdReparse(), the opaque pxr-handle op tables).

#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/IceDescriptorAllocator.h"
#include "usdBridge/StageBridge.h"
#include "usdInterface/UsdInterface.h"

#include <omni/physics/parse/IParseBackend.h>
#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>
#include <omni/physics/parse/UsdReparse.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser
{

// --------------------------------------------------------------------------------------
// Plain (non-ovstage) attach.
// --------------------------------------------------------------------------------------
// Reaches USD only through the reparse seam: resolveStageToHandle() replaces the USD arm's
// UsdUtilsStageCache Find, and the AttachedStage ctor's setStage() rebuilds the source through
// the registered parse backend (the USD backend installed by the loader). With no seam
// installed (production / USD-free) it logs and refuses. Its
// lone caller is attachReplicatorCreateSource() via PhysXReplicator::attach().
bool UsdLoad::attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt)
{
    if (mAttachedStages.find(stageId) != mAttachedStages.end())
    {
        CARB_LOG_ERROR("Stage already attached!");
        return false;
    }

    // Exclusivity guard: an ovstage attach owns the scan/parse backend registry exclusively.
    // Letting a plain attach through would stomp the live ovstage backend and leave detach()
    // unable to tell the two apart (it decides from mExternalBackendInstalled alone).
    if (mExternalBackendInstalled)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attach called while an ovstage attach is exclusively active");
        return false;
    }

    // No reparse seam installed -> no way to resolve a stage id to a stage handle; fail closed
    // (a USD-free process has no UsdUtilsStageCache at all).
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - plain USD stage-cache attach is unavailable in the pxr-free "
                       "build: no reparse seam is installed to resolve a stage id to a stage handle");
        return false;
    }

    // Resolve the id and build the AttachedStage under the parsing mutex: the ctor's setStage()
    // runs rebuildSource() through the registered parse backend. Only the pxr stage-cache Find of
    // the USD arm is replaced (by resolveStageToHandle); the rest mirrors it, including the
    // native-scan-backend refcount dance (whose install/remove bodies are no-ops in this build --
    // the loader already registered the USD scan backend).
    AttachedStage* attachedStage = nullptr;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mParsingMutex);

        AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
        if (!stageId || !reparse->resolveStageToHandle(stageId, handle.storage()))
        {
            CARB_LOG_ERROR("PhysicsUsdLoad - could not find USD stage");
            return false;
        }

        if (mPlainScanBackendRefCount == 0)
        {
            // Capture the pre-attach scan slot by ownership before re-asserting the native-USD
            // one, so detach()/load-failure restore exactly what was live here (null when a test
            // helper displaced it, the startup USD scan otherwise) rather than leaking the
            // attach-installed backend past teardown.
            mRestorePlainScanBackend = omni::physics::parse::takeScanBackend();
            try
            {
                bridgeInstallDefaultScanBackend();
            }
            catch (const std::exception& error)
            {
                CARB_LOG_ERROR("PhysicsUsdLoad - failed to install native scan backend: %s", error.what());
                omni::physics::parse::setScanBackend(std::move(mRestorePlainScanBackend));
                return false;
            }
            catch (...)
            {
                CARB_LOG_ERROR("PhysicsUsdLoad - failed to install native scan backend with an unknown exception");
                omni::physics::parse::setScanBackend(std::move(mRestorePlainScanBackend));
                return false;
            }
        }
        ++mPlainScanBackendRefCount;

        attachedStage = new AttachedStage(handle, usdPhysicsInt);
    }

    if (!loadAttachedStage(attachedStage, stageId, loadPhysics))
    {
        mAttachedStages.erase(stageId);
        delete attachedStage;
        if (--mPlainScanBackendRefCount == 0)
        {
            bridgeRemoveDefaultScanBackend();
            omni::physics::parse::setScanBackend(std::move(mRestorePlainScanBackend));
        }
        return false;
    }
    return true;
}

// --------------------------------------------------------------------------------------
// Backend registry slivers.
// --------------------------------------------------------------------------------------

uint64_t bridgeBackingStageCacheId(AttachedStageUsdHandle handle)
{
    // The USD arm returned UsdUtilsStageCache::GetId(usdStageOf(handle)). The reparse seam maps
    // the opaque handle storage to its stage-cache id without this TU naming a UsdStage; with no
    // seam installed (empty handle) it fails closed to 0.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    return reparse ? reparse->stageCacheIdForHandle(handle.storage()) : 0;
}

std::unique_ptr<omni::physics::parse::IParseBackend> bridgeMakeDefaultParseBackend()
{
    // The USD arm returned makeUsdParseBackend() so an ovstage attach could restore it on
    // detach. That factory is pxr-only, so instead of rebuilding the USD backend this captures
    // the live one out of the registry by ownership (takeParseBackend). Under a USD-loaded
    // process that is the backend omniPhysicsUsdInstallBackends() installed at startup; detach
    // then reinstalls the exact same instance. In a USD-free process the registry holds null,
    // so this captures null and the restore degrades to "no backend installed".
    // Capturing (not tearing down) is what keeps the plain USD attach alive across an
    // ovstage attach/detach, since bridgeInstallDefaultScanBackend() here is a no-op.
    return omni::physics::parse::takeParseBackend();
}

void bridgeInstallDefaultScanBackend()
{
    // The USD arm installed makeUsdScanBackend() for a plain attach's scan dispatch. Pxr-only,
    // so this re-asserts it through the reparse seam: a plain USD attach must own a live USD scan
    // backend even if a test helper displaced the process default (setScanBackend(nullptr),
    // ADR-0010) -- otherwise loadFromStage scans empty and the attach finds no scene
    // (TestOvstageStagelessForeignStageParse.cpp CONTROL). Fails closed (no scan backend) in a
    // USD-free process, where usdReparse() is null and a plain attach is unreachable anyway.
    if (omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse())
        omni::physics::parse::setScanBackend(reparse->makeScanBackend());
}

void bridgeRemoveDefaultScanBackend()
{
    // Mirror of bridgeInstallDefaultScanBackend(): no-op. The registered scan backend outlives
    // this refcount, so there is nothing to unregister here.
}

void bridgeRestoreDefaultParseBackend(std::unique_ptr<omni::physics::parse::IParseBackend> restore)
{
    // `restore` is the backend prebuilt at attach time. It is always null here (this pair's
    // bridgeMakeDefaultParseBackend() never produces one), so this installs null -- "no backend
    // installed". Honour a non-null value defensively if one is ever supplied.
    omni::physics::parse::setParseBackend(std::move(restore));
}

bool bridgeAttachStageById(uint64_t stageId, AttachedStage& outAttachedStage)
{
    // Mirror of the USD arm's attachForeignStage, with the pxr stage-cache Find replaced by
    // resolveStageToHandle. setStage() rebuilds an OWNED source on outAttachedStage through the
    // registered parse backend, so outAttachedStage stays self-contained (no session lifetime to
    // track). Fails closed with no seam installed.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
        return false;

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);

    AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
    if (!reparse->resolveStageToHandle(stageId, handle.storage()))
        return false;

    outAttachedStage.setStage(handle);
    return true;
}

PhysxShapeDesc* bridgeParseForeignStageCollision(uint64_t stageId,
                                                 uint64_t collisionPrimId,
                                                 AttachedStage& outStorage,
                                                 omni::physics::parse::ObjectKey& outCollisionKey)
{
    // Foreign USD stage named by id, never attached through the registry -- reachable while a
    // live ovstage attach owns the parse/scan backends (the stale-handle fallback in
    // requestConvexCollisionRepresentation, TestOvstageStagelessForeignStageParse.cpp). Both the
    // source build AND the scan MUST bypass the active backend, which would misread this USD
    // stage as its own ovstage payload.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
    {
        // No seam installed (production / USD-free): no live UsdStageCache to resolve the id.
        CARB_LOG_ERROR("requestConvexCollisionRepresentation: the unattached-stage-by-id fallback "
                       "needs a resident USD stage cache, unavailable in the USD-free build. Pass a "
                       "live attachHandle or stageId.");
        return nullptr;
    }

    // collisionPrimId keeps its legacy asInt(SdfPath)-bits meaning in this arm (IPhysxCooking.h
    // carve-out); decode it into the scan-root path string.
    std::string colPath;
    if (!reparse->legacyPathBitsToString(collisionPrimId, colPath))
        return nullptr;

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);

    // Bind outStorage to a self-owned USD source over the foreign stage. setStage() rebuilds
    // through the reparse seam's PRIVATE USD backend (AttachedStage::rebuildUsdSource), so
    // outStorage.getSource() is a real USD source the caller's later meshPrimKey / outCollisionKey
    // reads (fillCookingMeshViewFromSource, textFor) resolve against -- and it outlives this call.
    AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
    if (!reparse->resolveStageToHandle(stageId, handle.storage()))
        return nullptr;
    outStorage.setStage(handle);

    // outStorage's own key for the collision root -- the gPrimKey selectScannedCollision matches,
    // and the key the caller resolves against outStorage's source. A fresh source can only mint a
    // key for a prim that actually exists on it (ADR-0021 generation tags), so a bad id fails
    // closed here rather than naming the wrong prim.
    outCollisionKey = outStorage.keyFor(colPath);
    const omni::physics::parse::IPhysicsSource* src = outStorage.getSource();
    if (!src || !src->exists(outCollisionKey))
        return nullptr;

    // Scan the foreign stage through the reparse backend's OWN USD session (its private USD
    // backend), never the active scan backend. selectScannedCollision re-keys the scanned source's
    // identity into outStorage's key-space, so a scan source distinct from outStorage's is fine.
    omni::physics::parse::UsdReparseSessionPtr session = omni::physics::parse::openScopedStage(*reparse, stageId);
    if (!session)
        return nullptr;

    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScannedStage scanned =
        reparse->scan(*session, std::vector<std::string>{ colPath }, kNoExclude,
                      omni::physics::parse::ScanOptions{}, omni::physx::usdparser::iceDescriptorAllocator());
    return selectScannedCollision(outStorage, scanned, outCollisionKey);
}

// --------------------------------------------------------------------------------------
// Single collision-root scan (registry-routed).
// --------------------------------------------------------------------------------------
// Dispatch through parse::scanStage(attachTarget()), which
// routes to whatever scan backend is registered (the USD scan backend under a USD-loaded
// process, ovstage under an ovstage attach). Unlike the USD arm's omni::physics::usd::scanStage,
// there is no native-walk fallback when no backend is registered -- a scan issued after an
// ovstage detach cleared the slot would return empty. That window is the loadable library's to
// re-establish.
PhysxShapeDesc* bridgeScanCollisionRoot(AttachedStage& attachedStage,
                                        omni::physics::parse::ObjectKey collisionKey,
                                        omni::physics::parse::ObjectKey gPrimKey)
{
    const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(collisionKey)) };
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScannedStage scanned =
        omni::physics::parse::scanStage(attachedStage.attachTarget(), scanRoots, kNoExclude,
                                        omni::physics::parse::ScanOptions{},
                                        omni::physx::usdparser::iceDescriptorAllocator());
    return selectScannedCollision(attachedStage, scanned, gPrimKey);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
