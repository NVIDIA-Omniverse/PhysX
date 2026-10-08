// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-DEFAULT-001
 * @covers AC-2
 *
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-6 AC-11
 *
 * @implements REQ-PUBLICAPI-002
 * @covers AC-2 AC-8
 *
 * @implements REQ-SIM-OVSTAGE-ATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/OmniPhysXUtilities.h>
#include <carb/profiler/Profile.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "PrimUpdate.h"
#include "LoadStage.h"
#include "usdInterface/UsdInterface.h"
#include "Mass.h"
#include "Joint.h"
#include "Articulation.h"
#include "Collision.h"
#include "Particles.h"
#include "Collision.h"
#include "CollisionGroup.h"

// The USD-reaching pieces (stage-cache-backed attach(), backend install/restore, the
// stage-id-taking single-prim parse helpers) live in the pxr-free usdBridge/StageBridge.cpp.
#include "usdBridge/StageBridge.h"
#include <omni/physics/parse/ScanBackend.h>      // setScanBackend
#include <omni/physics/parse/ScannedStage.h>     // parse::ScannedStage (base)
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

using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static UsdLoad* gUsdLoad = nullptr;

// Process-lifetime, not UsdLoad-instance-lifetime: see the member declaration
// in LoadUsd.h for why this must survive getUsdLoad()/releaseUsdLoad() rebuilds.
std::atomic<uint64_t> UsdLoad::mNextAttachHandle{ UsdLoad::kAttachHandleBase };

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

AttachedStage* UsdLoad::resolveAttach(uint64_t attachHandle) const
{
    // ADR-0016 Decision 3: the one resolution prologue for every public entry
    // point that names an attach. kActiveAttach is the migration path for the
    // callers that used to pass stage id 0 and lean on getAttachedStage()'s
    // lone-attach fallback -- getAttachedStageByHandle() deliberately has no
    // such fallback, so the convenience has to be spelled out here.
    if (attachHandle == omni::physx::kActiveAttach)
    {
        return getActiveAttachedStage();
    }

    // kNoAttach and any stale handle fall through to the handle lookup, which
    // returns null for both.
    return getAttachedStageByHandle(attachHandle);
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

bool UsdLoad::loadAttachedStage(AttachedStage* attachedStage, uint64_t key, bool loadPhysics)
{
    // The single source-agnostic load core shared by attach() (USD) and
    // attachOvstage() (ovstage); they differ only in how the AttachedStage is
    // constructed. loadFromStage() routes through the active scan/parse
    // backend (ADR-0002 M2c), so this stays backend-agnostic.
    mAttachedStages[key] = attachedStage;

    // Mint the attach handle here, the one point every attach route passes
    // through (ADR-0013). Monotonic and never reused, so a handle held across a
    // detach resolves to null rather than to the next attach.
    attachedStage->setAttachHandle(mNextAttachHandle++);

    if (loadPhysics)
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

        {
            // Suppress initial-population notifications; the scope restores both
            // gates even if loadFromStage() throws (transactional attach rollback).
            InitialStagePopulationScope populationScope(*attachedStage->getPhysXPhysicsInterface());

            // load the scene by traversing from the root prim (USD) or the whole
            // ovstage instance (ovstage scan backend reads from the pseudo-root)
            if (!loadFromStage(*attachedStage))
                return false;
        }

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(true);

        if (!attachedStage->getObjectDatabase()->empty())
        {
            attachedStage->getPrimUpdateMap().setEmptyScene(false);
        }
    }
    else
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

    return true;
}

// UsdLoad::attach() (plain, stage-cache-backed) is defined in usdBridge/StageBridge.cpp -- it
// resolves the stage id through the parse::usdReparse() seam.

// Shared core for attachOvstage()'s two public overloads below.
bool UsdLoad::attachOvstageCore(const void* ovstageAttachPayload,
                                uint64_t readOrdinal,
                                AttachedStageUsdHandle backingStage,
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

    if (bridgeBackingStageCacheId(backingStage) != effectiveBackingStageId)
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
        // Allocate the ovstage backends before touching either process-global slot: their
        // constructors can throw, and doing so before any capture or swap leaves the registry
        // untouched on failure. The scan backend is bound to this attach's payload, so a target
        // that did not come from this attach is reported rather than reinterpreted.
        ovstageParseBackend = omni::physics::ovstage::makeOvstageParseBackend();
        ovstageScanBackend = omni::physics::ovstage::makeOvstageScanBackend(ovstageAttachPayload);

        // Option 1 (ADR-0002): only this plugin owns its parse/scan registry
        // slots. Detach restores the previous defaults after the AttachedStage is gone.
        // Capturing the previous defaults by ownership (parse via the bridge sliver's
        // takeParseBackend, scan directly) and installing ovstage are all plain unique_ptr
        // moves that cannot throw, so they run only after backendMutationStarted is set -- any
        // later failure is undone by the rollback block below, which reinstalls both captures.
        // defaultParseBackend/mRestoreScanBackend are null in a USD-free process, which degrades
        // the restore to "no backend installed".
        backendMutationStarted = true;
        defaultParseBackend = bridgeMakeDefaultParseBackend();
        omni::physics::parse::setParseBackend(std::move(ovstageParseBackend));
        mRestoreScanBackend = omni::physics::parse::takeScanBackend();
        omni::physics::parse::setScanBackend(std::move(ovstageScanBackend));
        mRestoreParseBackend = std::move(defaultParseBackend);
        mExternalBackendInstalled = true;

        // Construct stageless so the ctor cannot build a USD source under the ovstage
        // backend, then retain only the already-classified resident stage.
        attachedStage = std::make_unique<AttachedStage>(AttachedStageUsdHandle{}, usdPhysicsInt);
        attachedStage->setOvstageSource(ovstageAttachPayload, backingStage, readOrdinal, effectiveBackingStageId);
        attachedStageId = static_cast<uint64_t>(attachedStage->getStageId());
        if (attachedStageId != effectiveBackingStageId)
            throw std::runtime_error("classified ovstage backing stage changed before registration");

        // The AttachedStage's normalized id is authoritative. Never register
        // under the raw candidate reported by another USD runtime.
        if (!loadAttachedStage(attachedStage.get(), attachedStageId, loadPhysics))
            throw std::runtime_error("physics source scan failed");
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
            omni::physics::parse::setScanBackend(std::move(mRestoreScanBackend));
        }
        catch (...)
        {
            CARB_LOG_ERROR("PhysicsUsdLoad - attachOvstage failed while restoring backends");
        }
        mExternalBackendInstalled = false;
        mRestoreParseBackend.reset();
        mRestoreScanBackend.reset();
    }

    if (partialPhysicsObjectsReleased)
        sendPhysicsObjectsReleasedEvent();

    return false;
}

bool UsdLoad::attachOvstage(const void* ovstageAttachPayload,
                            uint64_t readOrdinal,
                            uint64_t effectiveBackingStageId,
                            PhysXUsdPhysicsInterface* usdPhysicsInt,
                            bool loadPhysics,
                            AttachedStageUsdHandle backingStage)
{
    return attachOvstageCore(
        ovstageAttachPayload, readOrdinal, backingStage, effectiveBackingStageId, usdPhysicsInt, loadPhysics);
}

// First half of the replicator attach, so PhysXReplicator::attach() can create the
// AttachedStage/Source before its replicationAttachFn callback fires.
bool UsdLoad::attachReplicatorCreateSource(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage)
{
    if (attachStage && !attach(false, stageId, usdPhysicsInt))
        return false;
    return true;
}

// Second half; assumes attachReplicatorCreateSource() above already succeeded for this stageId.
bool UsdLoad::attachReplicatorFinish(uint64_t stageId, const PathSet& excludePaths, bool attachStage)
{
    AttachedStage* attachedStage = getAttachedStage(stageId);
    attachedStage->setReplicatorStage(true);
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);
        attachedStage->getPrimUpdateMap().clearMap();
        attachedStage->getPrimChangeMap().clearMap();

        bool loaded = false;
        {
            // Suppress initial-population notifications; the scope restores both
            // gates even if loadFromStage() throws (transactional attach rollback).
            InitialStagePopulationScope populationScope(*attachedStage->getPhysXPhysicsInterface());

            // load the scene by traversing from the root prim
            loaded = loadFromStage(*attachedStage, &excludePaths);
        }
        if (!loaded)
        {
            if (attachStage)
            {
                detach(stageId);
            }
            else
            {
                // Deferred-scan path (attachStage == false, e.g. the ovstage replicator flow):
                // this AttachedStage survives the failed load -- nothing above tore it down --
                // so the true set above at entry must be undone. Otherwise a caller's later
                // physXDetach() rollback sees isReplicatorStage() still true for an attach that
                // never actually got a replicator, and purges a still-pending kActiveAttach
                // registration meant for the caller's next, real attach attempt.
                attachedStage->setReplicatorStage(false);
            }
            return false;
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
        // previous default now that the stage is gone (mirrors carbOnPluginStartup).
        // Safe here: detach means nothing is attached, satisfying the registry's
        // "only while no stage is attached" contract.
        if (mExternalBackendInstalled)
        {
            bridgeRestoreDefaultParseBackend(std::move(mRestoreParseBackend));
            // Reinstall the previous default scan backend captured at attach, rather than
            // nulling the slot: a USD-loaded process must keep the startup-installed USD scan
            // backend across an ovstage attach/detach (the collapsed install slivers are no-ops,
            // so a plain USD attach never re-registers one). null in a USD-free process.
            omni::physics::parse::setScanBackend(std::move(mRestoreScanBackend));
            mExternalBackendInstalled = false;
            mRestoreParseBackend.reset();
            mRestoreScanBackend.reset();
        }
        else if (mPlainScanBackendRefCount > 0)
        {
            // A plain attach holding the native-USD scan backend registration attach()
            // installs above. Mutually exclusive with the branch above: attach() rejects a
            // plain attach while mExternalBackendInstalled is true. On the last one going away,
            // restore the scan slot captured before the 0->1 re-assert so a completed plain
            // attach/detach leaves the backends exactly as it found them.
            if (--mPlainScanBackendRefCount == 0)
            {
                bridgeRemoveDefaultScanBackend();
                omni::physics::parse::setScanBackend(std::move(mRestorePlainScanBackend));
            }
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
    const bool defSim = defaultSim == "PhysX";
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        ref.second->setIsPhysXDefaultSimulator(defSim);
    }
}


// translateScannedShape/selectScannedCollision have external linkage (declared in
// usdBridge/StageBridge.h) so the USD arm's SdfPath-taking sibling can reuse them.

// Per-shape consumer-side bridge: cooking dispatch, ObjectKey
// re-keying across the scanStage / attachedStage source namespaces,
// and consumer state translation (sceneIds / materials / filteredPairs
// / collisionGroup).  Equivalent to ignoreOwners=true semantics: a
// shape with simulationOwners-but-no-resolvable-scene is kept rather
// than dropped.
void translateScannedShape(AttachedStage& attachedStage,
                           const omni::physics::parse::ScannedStage& scanned,
                           PhysxShapeDesc* desc,
                           omni::physics::parse::ObjectKey shapeKey)
{
    scan::dispatchScannedShapeCooking(attachedStage, scanned, desc);
    const omni::physics::parse::IPhysicsSource& scanSource = scanned.source();
    if (desc->rigidBody.valid())
        desc->rigidBody = attachedStage.keyFor(scanSource.sourceKeyToString(desc->rigidBody));
    if (desc->sourceGprim.valid())
        desc->sourceGprim = attachedStage.keyFor(scanSource.sourceKeyToString(desc->sourceGprim));
    // Mesh-cooking subclasses each carry a `meshPrimKey` ObjectKey the
    // runtime uses to locate the source mesh prim.  The field is
    // name-shadowed across the MergeMesh hierarchy, so cast to the
    // specific subclass for the translation.  Matches PointInstancer.
    if (desc->type == eConvexMeshShape)
    {
        auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(desc);
        if (d->meshPrimKey.valid())
            d->meshPrimKey = attachedStage.keyFor(scanSource.sourceKeyToString(d->meshPrimKey));
    }
    else if (desc->type == eTriangleMeshShape ||
             desc->type == eConvexMeshDecompositionShape ||
             desc->type == eSpherePointsShape)
    {
        auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(desc);
        if (d->meshPrimKey.valid())
            d->meshPrimKey = attachedStage.keyFor(scanSource.sourceKeyToString(d->meshPrimKey));
    }

    std::vector<omni::physics::parse::ObjectKey> materials;
    CollisionPairVector filteredPairs;
    (void)scan::resolveConsumerSideShapeState(attachedStage, scanned, desc,
                                              materials, filteredPairs);
    desc->collisionGroup = getCollisionGroup(attachedStage, shapeKey);
}

// Matches the gprim by `sourceGprim == gPrimKey || primKey == gPrimKey` (the latter for
// shapes where the gprim IS the collider).
PhysxShapeDesc* selectScannedCollision(AttachedStage& attachedStage,
                                       omni::physics::parse::ScannedStage& scanned,
                                       omni::physics::parse::ObjectKey gPrimKey)
{
    const omni::physics::parse::IPhysicsSource& scanSource = scanned.source();
    for (auto& shapeUPtr : scanned.shapes)
    {
        PhysxShapeDesc* desc = shapeUPtr.get();
        const omni::physics::parse::ObjectKey shapeKey =
            attachedStage.keyFor(scanSource.sourceKeyToString(desc->primKey));
        const omni::physics::parse::ObjectKey sourceGprimKey =
            desc->sourceGprim.valid() ? attachedStage.keyFor(scanSource.sourceKeyToString(desc->sourceGprim))
                                       : omni::physics::parse::ObjectKey();
        if (sourceGprimKey != gPrimKey && shapeKey != gPrimKey)
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

// ObjectKey-keyed single-prim collision parse (ADR-0019). The scan dispatches through the
// registered scan backend (bridgeScanCollisionRoot); a plain USD attach holds the USD scan
// backend from the reparse seam for its lifetime, so no native-walk fallback is needed.
//
// @implements REQ-PARSE-CONSUMER-001
// @covers AC-8
PhysxShapeDesc* parseCollision(AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey collisionKey,
                               omni::physics::parse::ObjectKey gPrimKey)
{
    CARB_PROFILE_ZONE(0, "physx::usdparser::parseCollision");
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(collisionKey))
        return nullptr;

    return bridgeScanCollisionRoot(attachedStage, collisionKey, gPrimKey);
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
