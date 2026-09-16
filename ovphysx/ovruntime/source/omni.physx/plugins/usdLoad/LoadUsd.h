// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-2 AC-8
 */
#pragma once

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
using AttachedStageMap = std::unordered_map<uint64_t, AttachedStage*>;

void releaseDesc(PhysxObjectDesc* objectDesc);

// Single-prim collision parse for a caller that already holds the collision/gprim identity
// as an ObjectKey (ADR-0019). Foreign-stage parsing by stage-cache id goes through
// usdBridge/StageBridge.h's bridgeParseForeignStageCollision.
PhysxShapeDesc* parseCollision(AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey collisionKey,
                               omni::physics::parse::ObjectKey gPrimKey);

class UsdLoad
{
public:
    UsdLoad();

    ~UsdLoad();

    static UsdLoad* getUsdLoad();

    // Destroys the lazily-created getUsdLoad() singleton and resets the
    // backing pointer, so the next getUsdLoad() builds a fresh instance.
    static void releaseUsdLoad();

    bool attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt);
    // Stageless attach for a consumer-provided ovstage source (ADR-0002 M2c-E).
    // `ovstageAttachPayload` is the backend-opaque AttachTarget payload (a
    // const OvstageAttach*), consumer-owned + kept alive for the attach. The
    // backing stage/id are resolved by the runtime entry before any session
    // mutation; this layer never re-queries the payload identity.
    //
    // `backingStage` is trailing with a default of "no backing stage", so callers that
    // have none can omit it. AttachedStageUsdHandle (AttachedStage.h) is
    // PXR_NS::UsdStageWeakPtr itself under USD and an unused two-pointer POD otherwise.
    bool attachOvstage(const void* ovstageAttachPayload,
                       uint64_t readOrdinal,
                       uint64_t effectiveBackingStageId,
                       PhysXUsdPhysicsInterface* usdPhysicsInt,
                       bool loadPhysics = true,
                       AttachedStageUsdHandle backingStage = AttachedStageUsdHandle{});

    // First half of the replicator attach, so PhysXReplicator::attach() can create the
    // AttachedStage/Source BEFORE its replicationAttachFn callback fires: verify not already
    // attached, resolve the stage, construct+register the AttachedStage. No parsing happens
    // here. A no-op returning true when attachStage is false (the ovstage path already has a
    // Source). Must be called, and must succeed, before attachReplicatorFinish() for the
    // same stageId.
    bool attachReplicatorCreateSource(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage);

    // Second half: assumes attachReplicatorCreateSource() above already succeeded for this
    // stageId (getAttachedStage(stageId) is non-null). Performs the exclude-path-filtered
    // traversal/parse and the failure rollback (detach() when attachStage, else just clearing
    // the replicator-stage flag).
    bool attachReplicatorFinish(uint64_t stageId, const PathSet& excludePaths, bool attachStage);

    void detach(uint64_t stageId);

private:
    // Source-agnostic load core shared by attach() (USD) and attachOvstage()
    // (ovstage): registers the AttachedStage under `key`, then optionally traverses
    // and creates engine objects via loadFromStage() (ADR-0002 Option 1).
    bool loadAttachedStage(AttachedStage* attachedStage, uint64_t key, bool loadPhysics);
    void sendPhysicsObjectsReleasedEvent() noexcept;

    // Shared body for attachOvstage() above (LoadUsd.cpp): guard checks, backend
    // install/rollback/exception scaffolding, and the loadAttachedStage() call.
    bool attachOvstageCore(const void* ovstageAttachPayload,
                           uint64_t readOrdinal,
                           AttachedStageUsdHandle backingStage,
                           uint64_t effectiveBackingStageId,
                           PhysXUsdPhysicsInterface* usdPhysicsInt,
                           bool loadPhysics);

public:

    AttachedStage* getAttachedStage(uint64_t stageId) const
    {
        if (stageId == 0 && mAttachedStages.size() == 1)
        {
            return mAttachedStages.begin()->second;
        }

        AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
        if (fit != mAttachedStages.end())
        {
            return fit->second;
        }

        return nullptr;
    }

    // Convenience accessors for the single currently-attached stage. ovruntime attaches one stage at
    // a time, registered under its effective resident BACKING USD stage id (0 when no usable backing
    // stage is resident in this runtime's UsdUtilsStageCache, e.g. a source whose raw nonzero id
    // belongs to another USD runtime). Resolve the lone attach by its actual key whenever exactly one
    // stage is attached; fall back to id 0 (then null) otherwise.
    AttachedStage* getActiveAttachedStage() const
    {
        if (mAttachedStages.size() == 1)
            return mAttachedStages.begin()->second;
        return getAttachedStage(0);
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

    // Linear scan on purpose: ovruntime holds one attach at a time (a handful at
    // most), and a second index keyed by handle would be a duplicate that every
    // teardown path has to remember to keep in sync. The AttachedStage owns its
    // handle; this is the only place that reads it back.
    AttachedStage* getAttachedStageByHandle(uint64_t attachHandle) const
    {
        if (attachHandle == kNoAttachHandle)
            return nullptr;

        for (AttachedStageMap::const_reference ref : mAttachedStages)
        {
            if (ref.second && ref.second->getAttachHandle() == attachHandle)
                return ref.second;
        }
        return nullptr;
    }

    // Handle of the lone active attach, or kNoAttachHandle when nothing (or more
    // than one thing) is attached. Mirrors getActiveAttachedStage().
    uint64_t getActiveAttachHandle() const
    {
        const AttachedStage* attachedStage = getActiveAttachedStage();
        return attachedStage ? attachedStage->getAttachHandle() : kNoAttachHandle;
    }

    // The single resolution prologue every attach-handle entry point shares
    // (ADR-0016 Decision 3).
    // kActiveAttach -> the lone active attach; kNoAttach and a stale handle -> null.
    // Deliberately silent: callers log their own diagnostic, since only they know
    // which entry point failed.
    AttachedStage* resolveAttach(uint64_t attachHandle) const;


    long getActiveStageId() const
    {
        AttachedStage* attachedStage = getAttachedStage(0);
        return attachedStage ? attachedStage->getStageId() : 0;
    }

    void update(uint64_t stageId, float);
    void update(float);
    void flushChanges();

    void releasePhysicsObjects(uint64_t stageId,
                               bool rebuildObjectDatabase = true,
                               bool sendReleasedEvent = true);

    void blockUSDUpdate(bool val);
    bool usdUpdateIsBlocked();

    void setAsyncUSDUpdate(bool val)
    {
        mAsyncUpdate = val;
    }
    bool getAsyncUSDUpdate() const
    {
        return mAsyncUpdate;
    }
    void processChanges();

    MemoryAllocator getMemoryAllocator()
    {
        return mMemoryAllocator;
    }

    void updateRigidBodyMass();

    void changeDefaultSimulator(const std::string& defaultSim);

public:
    carb::tasking::MutexWrapper mParsingMutex;

private:
    std::atomic_int32_t mBlockUsdUpdate;
    MemoryAllocator mMemoryAllocator;
    volatile bool mAsyncUpdate;
    // Keyed by backing USD stage id, which is what the USD change-notice and
    // stage-lifecycle paths legitimately look up by. Attach *identity* is the
    // AttachedStage's own handle, not this key -- see getAttachedStageByHandle.
    AttachedStageMap mAttachedStages;
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
    // True while attachOvstage() has switched the process parse/scan backends to
    // ovstage; detach() uses it to restore the USD defaults.
    bool mExternalBackendInstalled = false;
    // Allocated before installing the ovstage backends and retained until
    // failure cleanup or detach, so teardown restores USD without allocating.
    std::unique_ptr<omni::physics::parse::IParseBackend> mRestoreParseBackend;
    // Scan-backend counterpart of mRestoreParseBackend: the previous default scan backend,
    // captured by ownership before the ovstage scan backend overwrites it, so detach reinstalls
    // the exact same instance instead of nulling the slot (which would strand a USD-loaded
    // process with no scan backend, the collapsed install slivers being no-ops).
    std::unique_ptr<omni::physics::parse::IScanBackend> mRestoreScanBackend;
    // Ref count of live plain (non-ovstage) attaches sharing the registered native-USD
    // scan backend: attach() installs it on the 0->1 transition, detach() removes it on
    // 1->0. Only ever nonzero while mExternalBackendInstalled is false.
    size_t mPlainScanBackendRefCount = 0;
    // Scan backend that was live before the 0->1 plain-attach re-asserted the native-USD
    // one, captured by ownership so the 1->0 transition restores the exact pre-attach slot
    // (null when a test helper had displaced it, the startup USD scan otherwise) instead of
    // stranding the process with the attach-installed backend.
    std::unique_ptr<omni::physics::parse::IScanBackend> mRestorePlainScanBackend;
};


} // namespace usdparser
} // namespace physx
} // namespace omni
