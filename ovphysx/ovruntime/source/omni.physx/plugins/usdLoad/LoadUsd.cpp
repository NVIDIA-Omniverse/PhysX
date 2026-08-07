// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>
#include <common/utilities/OmniPhysXUtilities.h>
#include <carb/profiler/Profile.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "PrimUpdate.h"
#include "LoadStage.h"
#include "Mass.h"
#include "Joint.h"
#include "Articulation.h"
#include "Collision.h"
#include "Particles.h"
#include "Collision.h"
#include "CollisionGroup.h"

#include <omni/physics/usd/StageScan.h>
#include <omni/physics/usd/UsdParseBackend.h>   // makeUsdParseBackend (detach restore)
#include <omni/physics/parse/ScanBackend.h>      // setScanBackend
#include <omni/physics/ovstage/OvstageParseBackend.h> // makeOvstageParseBackend (Option 1)
#include <omni/physics/ovstage/OvstageScan.h>          // makeOvstageScanBackend
#include "IceDescriptorAllocator.h"
#include "ScannedShapeCookingDispatch.h"
#include "Vehicle.h"
#include "FixedTendon.h"
#include "SpatialTendon.h"
#include "Particles.h"
#include <CookingDataAsync.h>
#include <OmniPhysX.h>
#include "PhysicsBody.h"

#include <stdexcept>
#include <unordered_set>

#include <common/foundation/Algorithms.h>

#if CARB_COMPILER_MSC
#    pragma warning(disable : 4996)
#endif

using namespace PXR_NS;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static UsdLoad* gUsdLoad = nullptr;

UsdLoad* UsdLoad::getUsdLoad()
{
    if (!gUsdLoad)
        gUsdLoad = new UsdLoad();

    return gUsdLoad;
}

void UsdLoad::releaseUsdLoad()
{
    delete gUsdLoad;
    gUsdLoad = nullptr;
}

UsdLoad::UsdLoad()
    : mBlockUsdUpdate({0}), mAsyncUpdate(false)
{
    // USD change tracking is owned per-stage by each AttachedStage's
    // IChangeFeed (ADR-0003), not by a single global TfNotice listener here.
}

UsdLoad::~UsdLoad()
{
}

void UsdLoad::loadAttachedStage(AttachedStage* attachedStage, uint64_t key, bool loadPhysics)
{
    // The single source-agnostic load core shared by attach() (USD) and
    // attachOvstage() (ovstage); they differ only in how the AttachedStage is
    // constructed. loadFromStage() routes through the active scan/parse
    // backend (ADR-0002 M2c), so this stays backend-agnostic.
    mAttachedStages[key] = attachedStage;

    if (loadPhysics)
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

        {
            // Suppress initial-population notifications; the scope restores both
            // gates even if loadFromStage() throws (transactional attach rollback).
            InitialStagePopulationScope populationScope(*attachedStage->getPhysXPhysicsInterface());

            // load the scene by traversing from the root prim (USD) or the whole
            // ovstage instance (ovstage scan backend reads from the pseudo-root)
            loadFromStage(*attachedStage);
        }

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(true);

        if (!attachedStage->getObjectDatabase()->empty())
        {
            attachedStage->getPrimUpdateMap().setEmptyScene(false);
        }
    }
    else
        attachedStage->getPrimUpdateMap().setEmptyScene(true);
}

bool UsdLoad::attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt)
{
    if (mAttachedStages.find(stageId) != mAttachedStages.end())
    {
        CARB_LOG_ERROR("Stage already attached!");
        return false;
    }

    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stageId || !stage)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - could not find USD stage");
        return false;
    }

    AttachedStage* attachedStage = new AttachedStage(stage, usdPhysicsInt);
    loadAttachedStage(attachedStage, stageId, loadPhysics);
    return true;
}

bool UsdLoad::attachOvstage(const void* ovstageAttachPayload,
                            uint64_t readOrdinal,
                            PXR_NS::UsdStageWeakPtr backingStage,
                            uint64_t effectiveBackingStageId,
                            PhysXUsdPhysicsInterface* usdPhysicsInt,
                            bool loadPhysics)
{
    if (!ovstageAttachPayload)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage called without an ovstage payload");
        return false;
    }

    if (!usdPhysicsInt)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage called without a physics interface");
        return false;
    }

    if (!mAttachedStages.empty() || mExternalBackendInstalled || mRestoreParseBackend)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage called while another stage or external backend is active");
        return false;
    }

    const uint64_t backingStageId = backingStage ?
        static_cast<uint64_t>(UsdUtilsStageCache::Get().GetId(backingStage).ToLongInt()) : 0;
    if (backingStageId != effectiveBackingStageId)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage received inconsistent backing stage and effective id");
        return false;
    }

    std::unique_ptr<omni::physics::parse::IParseBackend> defaultParseBackend;
    std::unique_ptr<omni::physics::parse::IParseBackend> ovstageParseBackend;
    std::unique_ptr<omni::physics::parse::IScanBackend> ovstageScanBackend;
    std::unique_ptr<AttachedStage> attachedStage;
    uint64_t attachedStageId = effectiveBackingStageId;
    bool backendMutationStarted = false;
    bool partialPhysicsObjectsReleased = false;

    try
    {
        // Allocate every backend before replacing either process-global slot.
        // In particular, prebuild the USD backend now: constructing it during
        // rollback or detach could throw after mutation and leave the registry
        // on the ovstage backend.
        defaultParseBackend = omni::physics::usd::makeUsdParseBackend();
        ovstageParseBackend = omni::physics::ovstage::makeOvstageParseBackend();
        ovstageScanBackend = omni::physics::ovstage::makeOvstageScanBackend();

        // Option 1 (ADR-0002): only this plugin owns its parse/scan registry
        // slots. Detach restores the USD defaults after the AttachedStage is gone.
        backendMutationStarted = true;
        omni::physics::parse::setParseBackend(std::move(ovstageParseBackend));
        omni::physics::parse::setScanBackend(std::move(ovstageScanBackend));
        mRestoreParseBackend = std::move(defaultParseBackend);
        mExternalBackendInstalled = true;

        // Construct stageless so the ctor cannot build a USD source under the
        // ovstage backend, then retain only the already-classified resident stage.
        attachedStage = std::make_unique<AttachedStage>(PXR_NS::UsdStageWeakPtr{}, usdPhysicsInt);
        attachedStage->setOvstageSource(ovstageAttachPayload, backingStage, readOrdinal);
        attachedStageId = static_cast<uint64_t>(attachedStage->getStageId());
        if (attachedStageId != effectiveBackingStageId)
            throw std::runtime_error("classified ovstage backing stage changed before registration");

        // The AttachedStage's normalized id is authoritative. Never register
        // under the raw candidate reported by another USD runtime.
        loadAttachedStage(attachedStage.get(), attachedStageId, loadPhysics);
        attachedStage.release();
        return true;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage failed with an unknown exception");
    }

    if (attachedStage)
    {
        AttachedStageMap::iterator fit = mAttachedStages.find(attachedStageId);
        if (fit != mAttachedStages.end() && fit->second == attachedStage.get())
        {
            try
            {
                fit->second->getPhysXPhysicsInterface()->enableObjectChangeNotifications(false);
                releasePhysicsObjects(attachedStageId,
                                      /*rebuildObjectDatabase=*/false,
                                      /*sendReleasedEvent=*/false);
                partialPhysicsObjectsReleased = true;
            }
            catch (...)
            {
                CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage failed while releasing partial physics objects");
            }
            mAttachedStages.erase(fit);
        }
        attachedStage.reset();
    }

    if (backendMutationStarted)
    {
        try
        {
            if (mRestoreParseBackend)
                omni::physics::parse::setParseBackend(std::move(mRestoreParseBackend));
            else
                omni::physics::parse::setParseBackend(std::move(defaultParseBackend));
            omni::physics::parse::setScanBackend(nullptr);
        }
        catch (...)
        {
            CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage failed while restoring USD backends");
        }
        mExternalBackendInstalled = false;
        mRestoreParseBackend.reset();
    }

    if (partialPhysicsObjectsReleased)
        sendPhysicsObjectsReleasedEvent();

    return false;
}

bool UsdLoad::attachReplicator(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, const PathSet& excludePaths, bool attachStage)
{
    if (attachStage && !attach(false, stageId, usdPhysicsInt))
        return false;

    AttachedStage* attachedStage = getAttachedStage(stageId);
    attachedStage->setReplicatorStage(true);
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);
        attachedStage->getPrimUpdateMap().clearMap();
        attachedStage->getPrimChangeMap().clearMap();

        {
            // Suppress initial-population notifications; the scope restores both
            // gates even if loadFromStage() throws (transactional attach rollback).
            InitialStagePopulationScope populationScope(*attachedStage->getPhysXPhysicsInterface());

            // load the scene by traversing from the root prim
            loadFromStage(*attachedStage, &excludePaths);
        }

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(true);

        if (!attachedStage->getObjectDatabase()->empty())
        {
            attachedStage->getPrimUpdateMap().setEmptyScene(false);
        }
    }
    return true;
}

void UsdLoad::sendPhysicsObjectsReleasedEvent() noexcept
{
    if (!OmniPhysX::isStarted())
        return;

    try
    {
        OmniPhysX::getInstance().sendSimulationEvent(SimulationEvent::ePhysicsObjectsReleased);
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - physics-objects-released event failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - physics-objects-released event failed with an unknown exception");
    }
}

void UsdLoad::releasePhysicsObjects(uint64_t stageId,
                                    bool rebuildObjectDatabase,
                                    bool sendReleasedEvent)
{
    notifyStageReset();

    AttachedStageMap::iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        fit->second->releasePhysicsObjects(rebuildObjectDatabase);
    }
    if (sendReleasedEvent)
        sendPhysicsObjectsReleasedEvent();
}

void UsdLoad::requestRigidBodyMassUpdate(const UsdPrim& prim)
{
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        RequestRigidBodyMassUpdate(*fit->second, fit->second->keyFor(prim.GetPrimPath()));
    }
}

void UsdLoad::requestParticleMassUpdate(const UsdPrim& prim)
{
    if (!prim)
        return;

    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        RequestParticleMassUpdate(*fit->second, fit->second->keyFor(prim.GetPrimPath()));
    }
}

void UsdLoad::detach(uint64_t stageId)
{
    AttachedStageMap::iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        notifyStageReset();
        fit->second->getPhysXPhysicsInterface()->enableObjectChangeNotifications(false);  // do not send these notifications when the simulation is to end
        // Preserve the established detach event timing: subscribers observe an
        // attached stage with an empty, valid ObjectDb before owner teardown.
        releasePhysicsObjects(stageId);

        delete fit->second;

        mAttachedStages.erase(fit);

        // If attachOvstage() switched the process backends to ovstage, restore the
        // USD defaults now that the stage is gone (mirrors carbOnPluginStartup).
        // Safe here: detach means nothing is attached, satisfying the registry's
        // "only while no stage is attached" contract.
        if (mExternalBackendInstalled)
        {
            if (!mRestoreParseBackend)
            {
                CARB_LOG_ERROR("PhysicsUsdLoad - no preallocated USD backend available during ovstage detach");
                omni::physics::parse::setParseBackend(omni::physics::usd::makeUsdParseBackend());
            }
            else
            {
                omni::physics::parse::setParseBackend(std::move(mRestoreParseBackend));
            }
            omni::physics::parse::setScanBackend(nullptr);
            mExternalBackendInstalled = false;
            mRestoreParseBackend.reset();
        }
    }
    else
    {
        CARB_LOG_WARN("Detach stage failed!");
    }
}

void UsdLoad::update(uint64_t stageId, float currentTime)
{
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        processUpdates(*fit->second, currentTime);
    }
}

void UsdLoad::update(float currentTime)
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        processUpdates(*ref.second, currentTime);
    }
}

void UsdLoad::flushChanges()
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        flushBufferedChanges(*ref.second, -1.0f);
    }
}

void UsdLoad::changeDefaultSimulator(const std::string& defaultSim)
{
    const bool defSim = omni::physx::isPhysXDefaultSimulator();
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        ref.second->setIsPhysXDefaultSimulator(defSim);
    }
}


namespace
{

// Per-shape consumer-side bridge: cooking dispatch, ObjectKey
// re-keying across the scanStage / attachedStage source namespaces,
// and consumer state translation (sceneIds / materials / filteredPairs
// / collisionGroup).  Equivalent to ignoreOwners=true semantics: a
// shape with simulationOwners-but-no-resolvable-scene is kept rather
// than dropped.
void translateScannedShape(AttachedStage& attachedStage,
                           const omni::physics::usd::ScannedStage& scanned,
                           PhysxShapeDesc* desc,
                           const SdfPath& shapeKey)
{
    scan::dispatchScannedShapeCooking(attachedStage, scanned, desc);
    if (desc->rigidBody.valid())
        desc->rigidBody = attachedStage.keyFor(scanned.pathFor(desc->rigidBody));
    if (desc->sourceGprim.valid())
        desc->sourceGprim = attachedStage.keyFor(scanned.pathFor(desc->sourceGprim));
    // Mesh-cooking subclasses each carry a `meshPrimKey` ObjectKey the
    // runtime uses to locate the source mesh prim.  The field is
    // name-shadowed across the MergeMesh hierarchy, so cast to the
    // specific subclass for the translation.  Matches PointInstancer.
    if (desc->type == eConvexMeshShape)
    {
        auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(desc);
        if (d->meshPrimKey.valid())
            d->meshPrimKey = attachedStage.keyFor(scanned.pathFor(d->meshPrimKey));
    }
    else if (desc->type == eTriangleMeshShape ||
             desc->type == eConvexMeshDecompositionShape ||
             desc->type == eSpherePointsShape)
    {
        auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(desc);
        if (d->meshPrimKey.valid())
            d->meshPrimKey = attachedStage.keyFor(scanned.pathFor(d->meshPrimKey));
    }

    SdfPathVector materials;
    CollisionPairVector filteredPairs;
    (void)scan::resolveConsumerSideShapeState(attachedStage, scanned, desc,
                                              materials, filteredPairs);
    desc->collisionGroup = getCollisionGroup(attachedStage, shapeKey);
}

} // anonymous

// Single-prim rigid-body parse for callers outside the stage-attach
// pipeline.  Body + attached shapes come straight out of scanStage;
// translateScannedShape (shared with PointInstancer.cpp) handles
// cooking dispatch, re-keying, and consumer state translation.
// Returns the body desc plus a `collision` vector of (shapeKey,
// PhysxShapeDesc*) pairs filtered by the body's own collision set.
//
// Time-sampled callback registration is intentionally skipped — all
// known callers (PhysXPropertyQuery, attachment helpers, cooking
// previews) are one-shot queries.
//
// @implements REQ-PARSE-CONSUMER-001
// @covers AC-8
PhysxRigidBodyDesc* parseRigidBody(uint64_t stageId,
                                   const SdfPath& path,
                                   std::vector<std::pair<SdfPath, PhysxShapeDesc*>>& collision)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(static_cast<long int>(stageId)));
    if (!stage)
        return nullptr;

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);

    AttachedStage attachedStage;
    attachedStage.setStage(stage);
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey rootKey = attachedStage.keyFor(path);
    if (!src || !src->exists(rootKey))
        return nullptr;

    const std::vector<SdfPath> scanRoots{ path };
    static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
    // Scan the backing USD stage natively rather than via the active scan backend:
    // this isolated single-body re-parse always works off a real UsdStage (found by
    // id above), and the AttachTarget overload would dispatch to the globally-active
    // backend (e.g. ovstage), which misreads the USD-stage target as its own payload
    // and crashes. The backing USD stage is authoritative (ovstage mirrors it), so
    // the USD-native walk yields the correct rigid-body desc for either backend.
    omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
        stage, scanRoots, kNoExclude,
        omni::physx::usdparser::iceDescriptorAllocator());

    // Find the body matching `path` (typically the range root; explicit
    // match keeps us robust to nested-body subtrees).  Keep a raw view
    // into the scanStage-owned desc; we mutate it in place, then clone
    // into ICE memory at the end so the consumer-side `ICE_FREE`
    // matches the allocator (parse-lib uses `std::make_unique` =
    // system heap; ICE_FREE expects the custom allocator's prefix).
    PhysxRigidBodyDesc* rbDesc = nullptr;
    for (auto& bodyUPtr : scanned.bodies)
    {
        if (scanned.pathFor(bodyUPtr->primKey) == path)
        {
            rbDesc = bodyUPtr.get();
            break;
        }
    }
    if (!rbDesc)
        return nullptr;

    // sceneIds — sourceSimulationOwners ObjectKeys → eScene ObjectIds.
    // Empty ObjectDatabase (fresh AttachedStage) yields empty sceneIds.
    rbDesc->sceneIds.clear();
    for (const auto& sk : rbDesc->sourceSimulationOwners)
    {
        const SdfPath sp = scanned.pathFor(sk);
        if (sp.IsEmpty())
            continue;
        const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(sp, eScene);
        if (entry != kInvalidObjectId)
            rbDesc->sceneIds.push_back(entry);
    }

    // Build a path-set of the body's collisions; we filter the
    // scanStage shape vector against it to mirror the legacy listener's
    // `bodyAndCollider.collisions` → `rigidBodyShapesMap` resolution
    // (which used `schema::RigidBodyDesc::collisions`, the same
    // depth-first list scanStage's emitter populates as
    // `sourceCollisions`).
    std::unordered_set<SdfPath, SdfPath::Hash> bodyCollisionPaths;
    bodyCollisionPaths.reserve(rbDesc->sourceCollisions.size());
    for (const auto& ck : rbDesc->sourceCollisions)
    {
        const SdfPath sp = scanned.pathFor(ck);
        if (!sp.IsEmpty())
            bodyCollisionPaths.insert(sp);
    }

    // Shape pass — depth-first iteration matches legacy
    // `loadFromRange` dispatch order, so output `collision` ordering
    // is identical.  Each scanStage-owned desc is translated in place,
    // then deep-cloned into ICE memory for the caller; the original
    // unique_ptr stays in `scanned` and frees on scope exit.
    for (auto& shapeUPtr : scanned.shapes)
    {
        PhysxShapeDesc* desc = shapeUPtr.get();
        const SdfPath shapeKey = scanned.pathFor(desc->primKey);
        if (bodyCollisionPaths.find(shapeKey) == bodyCollisionPaths.end())
            continue;

        translateScannedShape(attachedStage, scanned, desc, shapeKey);
        if (PhysxShapeDesc* released = shapeUPtr.release())
            collision.push_back(std::make_pair(shapeKey, released));
    }

    // Find and release the body's DescPtr from scanned.bodies.  `rbDesc`
    // is an observer captured earlier; ownership lives in the matching
    // DescPtr entry, which we transfer out by `.release()`.  All other
    // unreleased entries are freed by `scanned`'s destructor (ICE_FREE,
    // matched to the allocator passed to `scanStage`).
    for (auto& bodyUPtr : scanned.bodies)
    {
        if (bodyUPtr.get() == rbDesc)
            return bodyUPtr.release();
    }
    return nullptr;
}

// Single-prim collision parse for callers outside the stage-attach
// pipeline.  Routes through translateScannedShape; matches gprim by
// `sourceGprim == gprimPath || primKey == gprimPath` (latter for
// shapes where the gprim IS the collider).
//
// @implements REQ-PARSE-CONSUMER-001
// @covers AC-8
PhysxShapeDesc* parseCollision(AttachedStage& attachedStage, const SdfPath& colPath, const SdfPath& gprimPath)
{
    CARB_PROFILE_ZONE(0, "physx::usdparser::parseCollision");
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey rootKey = attachedStage.keyFor(colPath);
    if (!src || !src->exists(rootKey))
        return nullptr;

    const std::vector<SdfPath> scanRoots{ colPath };
    static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
    omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
        attachedStage.attachTarget(), scanRoots, kNoExclude,
        omni::physx::usdparser::iceDescriptorAllocator());

    for (auto& shapeUPtr : scanned.shapes)
    {
        PhysxShapeDesc* desc = shapeUPtr.get();
        const SdfPath shapeKey = scanned.pathFor(desc->primKey);
        const SdfPath sourceGprimPath = desc->sourceGprim.valid() ? scanned.pathFor(desc->sourceGprim) : SdfPath();
        if (sourceGprimPath != gprimPath && shapeKey != gprimPath)
            continue;

        translateScannedShape(attachedStage, scanned, desc, shapeKey);
        // Transfer ownership out of `scanned` to the caller.  The
        // descriptor is already ICE-allocated (Step 2 wired
        // `iceDescriptorAllocator()` into `scanStage`), so the caller's
        // `ICE_FREE`-based release path matches the underlying heap.
        return shapeUPtr.release();
    }

    return nullptr;
}

PhysxShapeDesc* parseCollision(uint64_t stageId, const SdfPath& colPath, const SdfPath& gprimPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    std::lock_guard<carb::tasking::MutexWrapper> lock(usdLoad->mParsingMutex);

    AttachedStage attachedStage;
    attachedStage.setStage(stage);
    return parseCollision(attachedStage, colPath, gprimPath);
}

void releaseDesc(usdparser::PhysxObjectDesc* desc)
{
    ICE_FREE(desc);
}

void UsdLoad::blockUSDUpdate(bool val)
{
    mBlockUsdUpdate += (val ? 1 : -1);
    CARB_ASSERT(mBlockUsdUpdate >= 0, "Unbalanced calls to release blocked state");
}

bool UsdLoad::usdUpdateIsBlocked()
{
    return (mBlockUsdUpdate > 0);
}

void UsdLoad::processChanges()
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        processChangeMap(*ref.second);
    }
}

void UsdLoad::updateRigidBodyMass()
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        ref.second->updateRigidBodyMass();
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
