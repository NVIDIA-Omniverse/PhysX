// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include "LoadTools.h"
#include "PrimUpdate.h"
#include "AttachedStage.h"
#include <PhysXReplicator.h>
#include <omni/physics/parse/IParseBackend.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
using AttachedStageMap = std::unordered_map<uint64_t, AttachedStage*>;

void releaseDesc(PhysxObjectDesc* objectDesc);
PhysxRigidBodyDesc* parseRigidBody(uint64_t stageId,
                                   const PXR_NS::SdfPath& path,
                                   std::vector<std::pair<PXR_NS::SdfPath, PhysxShapeDesc*>>& collision);
PhysxShapeDesc* parseCollision(uint64_t stageId, const PXR_NS::SdfPath& path, const PXR_NS::SdfPath& gPrimPath);
PhysxShapeDesc* parseCollision(AttachedStage& attachedStage,
                               const PXR_NS::SdfPath& path,
                               const PXR_NS::SdfPath& gPrimPath);

class UsdLoad
{
public:
    UsdLoad();

    ~UsdLoad();

    static UsdLoad* getUsdLoad();

    // Destroys the lazily-created getUsdLoad() singleton and resets the
    // backing pointer, so the next getUsdLoad() builds a fresh instance.
    static void releaseUsdLoad();

    void requestRigidBodyMassUpdate(const PXR_NS::UsdPrim& prim);
    void requestParticleMassUpdate(const PXR_NS::UsdPrim& prim);

    bool attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt);
    // Stageless attach for a consumer-provided ovstage source (ADR-0002 M2c-E).
    // `ovstageAttachPayload` is the backend-opaque AttachTarget payload (a
    // const OvstageAttach*), consumer-owned + kept alive for the attach. The
    // backing stage/id are resolved by the runtime entry before any session
    // mutation; this layer never re-queries the payload identity.
    bool attachOvstage(const void* ovstageAttachPayload,
                       uint64_t readOrdinal,
                       PXR_NS::UsdStageWeakPtr backingStage,
                       uint64_t effectiveBackingStageId,
                       PhysXUsdPhysicsInterface* usdPhysicsInt,
                       bool loadPhysics = true);
    bool attachReplicator(uint64_t stageId,
                          PhysXUsdPhysicsInterface* usdPhysicsInt,
                          const PathSet& excludePaths,
                          bool attachStage);

    void detach(uint64_t stageId);

private:
    // Source-agnostic load core shared by attach() (USD) and attachOvstage()
    // (ovstage): registers the AttachedStage under `key`, then optionally traverses
    // and creates engine objects via loadFromStage() (ADR-0002 Option 1).
    void loadAttachedStage(AttachedStage* attachedStage, uint64_t key, bool loadPhysics);
    void sendPhysicsObjectsReleasedEvent() noexcept;

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

    PXR_NS::UsdStageWeakPtr getActiveStage() const
    {
        AttachedStage* attachedStage = getAttachedStage(0);
        return attachedStage ? attachedStage->getStage() : PXR_NS::UsdStageWeakPtr();
    }

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
    AttachedStageMap mAttachedStages;
    // True while attachOvstage() has switched the process parse/scan backends to
    // ovstage; detach() uses it to restore the USD defaults.
    bool mExternalBackendInstalled = false;
    // Allocated before installing the ovstage backends and retained until
    // failure cleanup or detach, so teardown restores USD without allocating.
    std::unique_ptr<omni::physics::parse::IParseBackend> mRestoreParseBackend;
};


} // namespace usdparser
} // namespace physx
} // namespace omni
