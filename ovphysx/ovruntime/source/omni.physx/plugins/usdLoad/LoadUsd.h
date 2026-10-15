// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-1 AC-2 AC-3

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-2 AC-8
 */
#pragma once

#include <usdLoad/ScannedCollision.h>
#include <private/omni/physx/PhysxUsd.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include "LoadTools.h"
#include "PrimUpdate.h"
#include "AttachedStage.h"
#include <omni/physics/AttachHandle.h>
#include <omni/physics/parse/IParseBackend.h>
#include <omni/physics/parse/ScanBackend.h>

#include <string>
#include <unordered_set>

namespace omni
{
namespace physx
{
namespace usdparser
{
using AttachedStageMap = std::unordered_map<uint64_t, std::shared_ptr<AttachedStage>>;

// Single-prim collision parse for a caller that already holds the collision/gprim identity
// as an ObjectKey (ADR-0019). Foreign-stage parsing by stage-cache id goes through
// usdBridge/StageBridge.h's bridgeParseForeignStageCollision.
ScannedCollision parseCollision(AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey collisionKey,
                               omni::physics::parse::ObjectKey gPrimKey);

class UsdLoad
{
public:
    UsdLoad();

    ~UsdLoad();

    static UsdLoad* getUsdLoad();

    // Cleanup must not recreate the singleton after runtime teardown.
    static UsdLoad* getUsdLoadIfCreated() noexcept;

    // Destroys the lazily-created getUsdLoad() singleton and resets the
    // backing pointer, so the next getUsdLoad() builds a fresh instance.
    static void releaseUsdLoad();

    AttachHandle attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt,
                        const PhysxAttachOptions& options = {});
    // Stageless attach for a consumer-provided ovstage source (ADR-0002 M2c-E).
    // `ovstageAttachPayload` is the backend-opaque AttachTarget payload (a
    // const OvstageAttach*). Its descriptor is copied; the referenced instance and
    // dictionary remain consumer-owned for the attach. The
    // backing stage/id are resolved by the runtime entry before any session
    // mutation; this layer never re-queries the payload identity.
    //
    // `backingStage` is trailing with a default of "no backing stage", so callers that
    // have none can omit it. AttachedStageUsdHandle (AttachedStage.h) is
    // PXR_NS::UsdStageWeakPtr itself under USD and an unused two-pointer POD otherwise.
    uint64_t attachOvstage(const void* ovstageAttachPayload,
                       uint64_t readOrdinal,
                       uint64_t effectiveBackingStageId,
                       PhysXUsdPhysicsInterface* usdPhysicsInt,
                       bool loadPhysics = true,
                       AttachedStageUsdHandle backingStage = AttachedStageUsdHandle{},
                       const PhysxAttachOptions* options = nullptr,
                       bool startConfiguredRecording = true);



    void detach(uint64_t stageId);

    // Resource-retirement core: unpublish this owner before draining its work and
    // destroying its source/database/scenes. Does not shut down the shared SDK or
    // broadcast the legacy process-wide lifecycle events. Public detach must add
    // owner-scoped event delivery before using this operation.
    bool retireAttachment(AttachHandle attach);


private:
    // Source-agnostic load core shared by attach() (USD) and attachOvstage()
    // (ovstage): realizes a private candidate, then publishes its owning pointer.
    // startConfiguredRecording starts legacy startup OmniPVD output afterwards; an
    // attachment without a public recording owner (companion preview) passes false.
    uint64_t loadAttachedStage(std::unique_ptr<AttachedStage> attachedStage, bool loadPhysics,
                               bool startConfiguredRecording = true);
    static uint64_t allocateAttachHandle();
    void sendPhysicsObjectsReleasedEvent(AttachHandle attach) noexcept;

    // Shared body for attachOvstage() above (LoadUsd.cpp): guard checks, backend
    // install/rollback/exception scaffolding, and the loadAttachedStage() call.
    uint64_t attachOvstageCore(const void* ovstageAttachPayload,
                           uint64_t readOrdinal,
                           AttachedStageUsdHandle backingStage,
                           uint64_t effectiveBackingStageId,
                           PhysXUsdPhysicsInterface* usdPhysicsInt,
                           bool loadPhysics,
                           const PhysxAttachOptions* options,
                           bool startConfiguredRecording);

public:

    // Developer USD adapter only. Backing stage IDs are metadata, not registry keys.
    AttachedStage* getAttachedStage(uint64_t stageId) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        AttachedStage* found = nullptr;
        for (const AttachedStageMap::value_type& entry : mAttachedStages)
        {
            if (static_cast<uint64_t>(entry.second->getStageId()) == stageId)
            {
                if (found)
                    return nullptr; // ambiguous backing metadata never selects a peer
                found = entry.second.get();
            }
        }
        return found;
    }

    // ------------------------------------------------------------------
    // Attach handles (ADR-0013). A handle identifies an *attach*, not a USD
    // stage: it is nonzero for every live attach including a stageless one, and
    // a fresh one is minted per attach, so a consumer holding a handle can tell
    // "still the attach I bound to" from "detached, or a different attach at the
    // same stage id". Neither of those is expressible with a stage id, where 0
    // means both "no stage" and "the stageless attach".
    // ------------------------------------------------------------------

    static constexpr uint64_t kNoAttachHandle = 0;

    std::shared_ptr<AttachedStage> acquireAttachment(uint64_t attachHandle) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        const auto it = mAttachedStages.find(attachHandle);
        return it != mAttachedStages.end() ? it->second : nullptr;
    }

    // Internal borrowed lookup. The caller must hold an execution lease or be
    // executing synchronously inside the selected owner's callback.
    AttachedStage* getAttachedStageByHandle(uint64_t attachHandle) const
    {
        return acquireAttachment(attachHandle).get();
    }

    // The single resolution prologue every attach-handle entry point shares
    // (ADR-0016 Decision 3).
    // Zero, retired and reserved sentinel handles resolve to null.
    // Deliberately silent: callers log their own diagnostic, since only they know
    // which entry point failed.
    AttachedStage* resolveAttach(uint64_t attachHandle) const;



    AttachedStageMap getAttachedStages() const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        return mAttachedStages;
    }

    void flushChanges(AttachedStage& attachedStage);

    void releasePhysicsObjects(AttachHandle attach,
                               bool rebuildObjectDatabase = true,
                               bool sendReleasedEvent = true);


    void processChanges();

    MemoryAllocator getMemoryAllocator()
    {
        return mMemoryAllocator;
    }

    void updateRigidBodyMass();


public:
    carb::tasking::MutexWrapper mParsingMutex;

private:
    MemoryAllocator mMemoryAllocator;
    // Owns only committed contexts, indexed by never-reused attachment handles.
    mutable carb::tasking::MutexWrapper mRegistryMutex;
    AttachedStageMap mAttachedStages;
    std::unordered_set<const void*> mReservedOvstages;
    // Never reused, never 0, so a handle held across a detach resolves to null
    // rather than to whatever attached next.
    //
    // Based well above the UsdUtilsStageCache id space on purpose. Stage-cache
    // ids also start at 1, so a base of 1 made the two identities numerically
    // collide: a caller that had not been migrated off stage ids would pass id 1
    // to an attach-handle parameter, resolve the first attach, and appear to
    // work -- then start failing later in the same process as the two counters
    // drifted, or resolve the *wrong* attach under multi-attach. Both are the
    // same width, so the compiler cannot flag it either (ADR-0016 Context 4).
    // Starting here means no realistic stage-cache id can ever resolve, which
    // turns that whole class of miss into an immediate, diagnosed failure.
    static constexpr uint64_t kAttachHandleBase = uint64_t(1) << 32;
    // A static class member, not an instance member: ovphysx_shutdown() destroys
    // and re-creates the UsdLoad singleton (see getUsdLoad()/releaseUsdLoad()), and
    // an instance-scoped counter would reset to kAttachHandleBase on every rebuild,
    // aliasing a handle minted before shutdown with one minted after reinit. This
    // counter must outlive any single UsdLoad instance to keep the never-reused
    // guarantee above process-wide, not just per-attach-session.
    static std::atomic<uint64_t> mNextAttachHandle;

};


} // namespace usdparser
} // namespace physx
} // namespace omni
