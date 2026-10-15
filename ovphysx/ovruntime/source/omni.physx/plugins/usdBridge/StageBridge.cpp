// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-3
 */

// @implements REQ-PARSE-CORE-006
// @covers AC-9

/**
 * @implements REQ-ATTACH-BACKENDS-001
 * @covers AC-4
 */

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
// installed (production / USD-free) it logs and refuses. Construct and realize
// privately before publishing the new handle.
AttachHandle UsdLoad::attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt,
                             const PhysxAttachOptions& options)
{
    if (getAttachedStage(stageId) != nullptr)
    {
        CARB_LOG_ERROR("Stage already attached!");
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

    // Resolve and construct under the parser lock. AttachedStage owns its scanner;
    // no process registry is modified by construction, failure, or detach.
    std::unique_ptr<AttachedStage> attachedStage;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mParsingMutex);

        AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
        if (!stageId || !reparse->resolveStageToHandle(stageId, handle.storage()))
        {
            CARB_LOG_ERROR("PhysicsUsdLoad - could not find USD stage");
            return false;
        }

        attachedStage = std::make_unique<AttachedStage>(handle, usdPhysicsInt);
        attachedStage->setAttachOptions(options);
    }

    return loadAttachedStage(std::move(attachedStage), loadPhysics);
}

// --------------------------------------------------------------------------------------
// Backing-stage metadata.
// --------------------------------------------------------------------------------------

uint64_t bridgeBackingStageCacheId(AttachedStageUsdHandle handle)
{
    // The USD arm returned UsdUtilsStageCache::GetId(usdStageOf(handle)). The reparse seam maps
    // the opaque handle storage to its stage-cache id without this TU naming a UsdStage; with no
    // seam installed (empty handle) it fails closed to 0.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    return reparse ? reparse->stageCacheIdForHandle(handle.storage()) : 0;
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

ScannedCollision bridgeParseForeignStageCollision(uint64_t stageId,
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
        return {};
    }

    // collisionPrimId keeps its legacy asInt(SdfPath)-bits meaning in this arm (IPhysxCooking.h
    // carve-out); decode it into the scan-root path string.
    std::string colPath;
    if (!reparse->legacyPathBitsToString(collisionPrimId, colPath))
        return {};

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);

    // Bind outStorage to a self-owned USD source over the foreign stage. setStage() rebuilds
    // through the reparse seam's PRIVATE USD backend (AttachedStage::rebuildUsdSource), so
    // outStorage.getSource() is a real USD source the caller's later meshPrimKey / outCollisionKey
    // reads (fillCookingMeshViewFromSource, textFor) resolve against -- and it outlives this call.
    AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
    if (!reparse->resolveStageToHandle(stageId, handle.storage()))
        return {};
    outStorage.setStage(handle);

    // outStorage's own key for the collision root -- the gPrimKey selectScannedCollision matches,
    // and the key the caller resolves against outStorage's source. A fresh source can only mint a
    // key for a prim that actually exists on it (ADR-0021 generation tags), so a bad id fails
    // closed here rather than naming the wrong prim.
    outCollisionKey = outStorage.keyFor(colPath);
    const omni::physics::parse::IPhysicsSource* src = outStorage.getSource();
    if (!src || !src->exists(outCollisionKey))
        return {};

    // Scan the foreign stage through the reparse backend's OWN USD session (its private USD
    // backend), never the active scan backend. selectScannedCollision re-keys the scanned source's
    // identity into outStorage's key-space, so a scan source distinct from outStorage's is fine.
    omni::physics::parse::UsdReparseSessionPtr session = omni::physics::parse::openScopedStage(*reparse, stageId);
    if (!session)
        return {};

    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScannedStage scanned =
        reparse->scan(*session, std::vector<std::string>{ colPath }, kNoExclude,
                      omni::physics::parse::ScanOptions{}, omni::physx::usdparser::iceDescriptorAllocator());
    PhysxShapeDesc* selected = selectScannedCollision(outStorage, scanned, outCollisionKey);
    return ScannedCollision(std::move(scanned), selected);
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
ScannedCollision bridgeScanCollisionRoot(AttachedStage& attachedStage,
                                        omni::physics::parse::ObjectKey collisionKey,
                                        omni::physics::parse::ObjectKey gPrimKey)
{
    const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(collisionKey)) };
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScannedStage scanned =
        attachedStage.scanStage(scanRoots, kNoExclude,
                                        attachedStage.descriptorScanOptions(),
                                        omni::physx::usdparser::iceDescriptorAllocator());
    PhysxShapeDesc* selected = selectScannedCollision(attachedStage, scanned, gPrimKey);
    return ScannedCollision(std::move(scanned), selected);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
