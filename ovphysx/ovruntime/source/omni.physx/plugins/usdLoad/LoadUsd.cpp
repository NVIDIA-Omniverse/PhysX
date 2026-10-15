// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-OMNIPVD-LATE-001
// @covers AC-2 AC-9

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2

// @implements REQ-ATTACH-SERVICES-001
// @covers AC-1 AC-2 AC-4

// @implements REQ-ATTACH-VIEWS-001
// @covers AC-2 AC-3

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-1 AC-2 AC-3

/**
 * @implements REQ-ATTACH-BACKENDS-001
 * @covers AC-4
 */
// @implements REQ-PARSE-CORE-006
// @covers AC-9

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
 *
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-3 AC-4
 */

#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/physx/RuntimeError.h>
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
#include "RuntimeExecution.h"
#include "OvxPhysicsShared.h"
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

UsdLoad* UsdLoad::getUsdLoadIfCreated() noexcept
{
    return gUsdLoad;
}

void UsdLoad::releaseUsdLoad()
{
    delete gUsdLoad;
    gUsdLoad = nullptr;
}

AttachedStage* UsdLoad::resolveAttach(uint64_t attachHandle) const
{
    return getAttachedStageByHandle(attachHandle);
}

UsdLoad::UsdLoad()
{
    // USD change tracking is owned per-stage by each AttachedStage's
    // IChangeFeed (ADR-0003), not by a single global TfNotice listener here.
}

UsdLoad::~UsdLoad()
{
}

uint64_t UsdLoad::allocateAttachHandle()
{
    uint64_t current = mNextAttachHandle.load(std::memory_order_relaxed);
    while (current != omni::physx::kActiveAttach)
    {
        if (mNextAttachHandle.compare_exchange_weak(current, current + 1, std::memory_order_relaxed))
            return current;
    }
    return kNoAttachHandle; // exhausted permanently; never wrap or mint a sentinel
}

uint64_t UsdLoad::loadAttachedStage(std::unique_ptr<AttachedStage> attachedStage, bool loadPhysics,
                                    bool startConfiguredRecording)
{
    const uint64_t handle = allocateAttachHandle();
    if (handle == kNoAttachHandle)
    {
        CARB_LOG_ERROR("Attachment handle space is exhausted.");
        return false;
    }
    attachedStage->setAttachHandle(handle);
    if (!OmniPhysX::getInstance().getPhysXSetup().acquireAttachmentResources(*attachedStage))
        return kNoAttachHandle;

    if (loadPhysics)
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

        {
            // Suppress initial-population notifications; the scope restores both
            // gates even if loadFromStage() throws (transactional attach rollback).
            InitialStagePopulationScope populationScope(*attachedStage->getPhysXPhysicsInterface(), *attachedStage);

            // load the scene by traversing from the root prim (USD) or the whole
            // ovstage instance (ovstage scan backend reads from the pseudo-root)
            if (!loadFromStage(*attachedStage))
                return false;
        }

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(*attachedStage, true);

        if (!attachedStage->getObjectDatabase()->empty())
        {
            attachedStage->getPrimUpdateMap().setEmptyScene(false);
        }
    }
    else
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

    attachedStage->setSimulationRunning(true);
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        mAttachedStages.emplace(handle, std::move(attachedStage));
    }
    if (startConfiguredRecording)
        OmniPhysX::getInstance().getPhysXSetup().startConfiguredOmniPvdRecording();
    return handle;
}

// UsdLoad::attach() (plain, stage-cache-backed) is defined in usdBridge/StageBridge.cpp -- it
// resolves the stage id through the parse::usdReparse() seam.

// Shared core for attachOvstage()'s two public overloads below.
uint64_t UsdLoad::attachOvstageCore(const void* ovstageAttachPayload,
                                uint64_t readOrdinal,
                                AttachedStageUsdHandle backingStage,
                                uint64_t effectiveBackingStageId,
                                PhysXUsdPhysicsInterface* usdPhysicsInt,
                                bool loadPhysics,
                                const PhysxAttachOptions* options,
                                bool startConfiguredRecording)
{
    if (!ovstageAttachPayload)
    {
        OVX_RUNTIME_ERROR("PhysicsUsdLoad - attachOvstage called without an ovstage payload");
        return false;
    }

    if (!usdPhysicsInt)
    {
        OVX_RUNTIME_ERROR("PhysicsUsdLoad - attachOvstage called without a physics interface");
        return false;
    }

    const void* instance = static_cast<const omni::physics::ovstage::OvstageAttach*>(ovstageAttachPayload)->instance;
    if (!instance)
    {
        OVX_RUNTIME_ERROR("PhysicsUsdLoad - attachOvstage called without an ovstage instance");
        return kNoAttachHandle;
    }
    for (AttachedStageMap::const_reference entry : getAttachedStages())
    {
        if (entry.second->getOvstageInstance() == instance)
        {
            OVX_RUNTIME_ERROR("PhysicsUsdLoad - ovstage instance is already attached");
            return kNoAttachHandle;
        }
    }

    if (bridgeBackingStageCacheId(backingStage) != effectiveBackingStageId)
    {
        OVX_RUNTIME_ERROR("PhysicsUsdLoad - attachOvstage received inconsistent backing stage and effective id");
        return false;
    }

    std::unique_ptr<AttachedStage> attachedStage;
    uint64_t attachedStageId = effectiveBackingStageId;

    try
    {
        bool reserved;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
            reserved = mReservedOvstages.insert(instance).second;
        }
        if (!reserved)
        {
            CARB_LOG_ERROR("PhysicsUsdLoad - ovstage instance is already being attached or retired");
            return kNoAttachHandle;
        }
        struct PendingRegistration
        {
            std::unordered_set<const void*>& instances;
            const void* instance;
            carb::tasking::MutexWrapper& mutex;
            ~PendingRegistration() { std::lock_guard<carb::tasking::MutexWrapper> lock(mutex); instances.erase(instance); }
        } pendingRegistration{ mReservedOvstages, instance, mRegistryMutex };

        // The candidate owns its backends and stable payload storage.
        attachedStage = std::make_unique<AttachedStage>(AttachedStageUsdHandle{}, usdPhysicsInt);
        attachedStage->setAttachOptions(options ? *options : PhysxAttachOptions{});
        attachedStage->setOvstageSource(ovstageAttachPayload, backingStage, readOrdinal, effectiveBackingStageId);
        attachedStageId = static_cast<uint64_t>(attachedStage->getStageId());
        if (attachedStageId != effectiveBackingStageId)
            throw std::runtime_error("classified ovstage backing stage changed before registration");

        // Backing-stage metadata is validated before publication under the
        // independent attachment handle.
        const uint64_t handle = loadAttachedStage(std::move(attachedStage), loadPhysics, startConfiguredRecording);
        if (handle == kNoAttachHandle)
            throw std::runtime_error("physics source scan failed");
        return handle;
    }
    catch (const std::exception& error)
    {
        OVX_RUNTIME_ERROR("PhysicsUsdLoad - attachOvstage failed: %s", error.what());
    }
    catch (...)
    {
        OVX_RUNTIME_ERROR("PhysicsUsdLoad - attachOvstage failed with an unknown exception");
    }

    // Candidate destruction retires only its records, scenes, jobs and sources.

    return false;
}

uint64_t UsdLoad::attachOvstage(const void* ovstageAttachPayload,
                            uint64_t readOrdinal,
                            uint64_t effectiveBackingStageId,
                            PhysXUsdPhysicsInterface* usdPhysicsInt,
                            bool loadPhysics,
                            AttachedStageUsdHandle backingStage,
                            const PhysxAttachOptions* options,
                            bool startConfiguredRecording)
{
    return attachOvstageCore(ovstageAttachPayload, readOrdinal, backingStage, effectiveBackingStageId, usdPhysicsInt,
                             loadPhysics, options, startConfiguredRecording);
}

void UsdLoad::sendPhysicsObjectsReleasedEvent(AttachHandle attach) noexcept
{
    if (!OmniPhysX::isStarted())
        return;

    try
    {
        OmniPhysX::getInstance().sendSimulationEvent(SimulationEvent::ePhysicsObjectsReleased, attach);
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

void UsdLoad::releasePhysicsObjects(AttachHandle attach,
                                    bool rebuildObjectDatabase,
                                    bool sendReleasedEvent)
{
    auto owner = acquireAttachment(attach);
    if (owner)
    {
        notifyStageReset(*owner);
        owner->releasePhysicsObjects(rebuildObjectDatabase);
    }
    if (sendReleasedEvent && owner)
        sendPhysicsObjectsReleasedEvent(owner->getAttachHandle());
}

bool UsdLoad::retireAttachment(AttachHandle attach)
{
    AttachedStage* owner = getAttachedStageByHandle(attach);
    if (!owner)
        return false;
    const void* instance = owner->getOvstageInstance();
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        if (instance && !mReservedOvstages.insert(instance).second)
            return false;
    }
    struct RetirementReservation
    {
        std::unordered_set<const void*>& instances;
        const void* instance;
        carb::tasking::MutexWrapper& mutex;
        ~RetirementReservation() { std::lock_guard<carb::tasking::MutexWrapper> lock(mutex); if (instance) instances.erase(instance); }
    } reservation{ mReservedOvstages, instance, mRegistryMutex };

    // Extract first: a continuation or release callback cannot resolve this
    // retiring handle, while its resources remain owned until destruction ends.
    AttachedStageMap::node_type retiring;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        retiring = mAttachedStages.extract(attach);
    }
    if (retiring.empty())
        return false;
    // The handle is unpublished, but its source dictionary is still alive.
    ovx::retireAttachmentReadCaches(attach, true);
    notifyStageReset(*retiring.mapped());
    // Invalidate this owner's views and notify observers before its native objects disappear.
    // The handle is already unpublished, but event payloads retain the exact owner identity.
    retiring.mapped()->getPhysXPhysicsInterface()->enableObjectChangeNotifications(*retiring.mapped(), false);
    retiring.mapped()->getPhysXPhysicsInterface()->releaseAllObjects(*retiring.mapped());
    sendPhysicsObjectsReleasedEvent(attach);
    retiring.mapped()->getPhysXPhysicsInterface()->retireObjectChangeNotifications(attach);
    OmniPhysX::getInstance().retireEventSubscriptions(*retiring.mapped());
    OmniPhysX::getInstance().unregisterReplicator(attach);
    // Shared lookup leases may outlive publication; native teardown must not.
    retiring.mapped()->releaseNativePhysics();
    retiring.mapped().reset();
    retireRuntimeExecution(attach);
    return true;
}

void UsdLoad::detach(uint64_t stageId)
{
    AttachedStage* owner = getAttachedStage(stageId);
    auto stages = getAttachedStages();
    auto fit = owner ? stages.find(owner->getAttachHandle()) : stages.end();
    if (fit != stages.end())
    {
        notifyStageReset(*fit->second);
        fit->second->getPhysXPhysicsInterface()->enableObjectChangeNotifications(*fit->second, false);  // do not send these notifications when the simulation is to end
        // Preserve the established detach event timing: subscribers observe an
        // attached stage with an empty, valid ObjectDb before owner teardown.
        releasePhysicsObjects(owner->getAttachHandle());

        const AttachHandle retiredHandle = fit->first;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
            mAttachedStages.erase(retiredHandle);
        }
        retireRuntimeExecution(retiredHandle);


    }
    else
    {
        CARB_LOG_WARN("Detach stage failed!");
    }
}

void UsdLoad::flushChanges(AttachedStage& attachedStage)
{
    flushBufferedChanges(attachedStage, -1.0f);
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
    if (!scanned.sourcePtr())
        return nullptr;
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
        // Borrow only. The bridge returns the selected pointer with its owning scan.
        return shapeUPtr.get();
    }

    return nullptr;
}

// ObjectKey-keyed single-prim collision parse (ADR-0019). The scan dispatches through the
// registered scan backend (bridgeScanCollisionRoot); a plain USD attach holds the USD scan
// backend from the reparse seam for its lifetime, so no native-walk fallback is needed.
//
// @implements REQ-PARSE-CONSUMER-001
// @covers AC-8
ScannedCollision parseCollision(AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey collisionKey,
                               omni::physics::parse::ObjectKey gPrimKey)
{
    CARB_PROFILE_ZONE(0, "physx::usdparser::parseCollision");
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(collisionKey))
        return {};

    return bridgeScanCollisionRoot(attachedStage, collisionKey, gPrimKey);
}


void UsdLoad::processChanges()
{
    for (AttachedStageMap::const_reference ref : getAttachedStages())
    {
        processChangeMap(*ref.second);
    }
}

void UsdLoad::updateRigidBodyMass()
{
    for (AttachedStageMap::const_reference ref : getAttachedStages())
    {
        ref.second->updateRigidBodyMass();
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
