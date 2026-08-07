// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <usdInterface/UsdInterface.h>
#include "PrimUpdate.h"

#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IPhysicsSource.h> // IPhysicsSource + SourceUnits (cached by value)

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>

#include <iterator>
#include <memory>
#include <set>

// AttachedStage holds the parse source trio only through the backend-neutral
// interfaces (ADR-0005) — it never names a concrete backend type. Consumers
// that need USD specifics down-cast the abstraction on demand (asUsdSource /
// asUsdDataWrite), which yields null under a non-USD backend.
namespace omni { namespace physics { namespace parse { class IPhysicsSource; class IPhysicsDataWrite; class IChangeFeed; struct AttachTarget; } } }

namespace omni
{
namespace physx
{
namespace usdparser
{
using TimeSampleMap = std::unordered_map<PXR_NS::SdfPath, OnUpdateObjectFn, PXR_NS::SdfPath::Hash>;
// Maps an animated kinematic body's path to its source ObjectKey (the key is
// redundant with keyFor(path) but avoids a per-frame re-resolve, and keeps the
// stored value USD-free — no UsdPrim).
using PathKeyMap = std::unordered_map<PXR_NS::SdfPath, omni::physics::parse::ObjectKey, PXR_NS::SdfPath::Hash>;
using TokenEnvIdMap = std::unordered_map<PXR_NS::TfToken, uint32_t, PXR_NS::TfToken::HashFunctor>;

struct GeneratedDeformableAttachmentData
{
    enum class Kind
    {
        eVtxTet,
        eVtxXform,
    };

    Kind kind = Kind::eVtxTet;
    bool enabled = false;
    PXR_NS::VtArray<int32_t> vtxIndicesSrc0;
    PXR_NS::VtArray<int32_t> tetIndicesSrc1;
    PXR_NS::VtArray<PXR_NS::GfVec3f> tetCoordsSrc1;
    PXR_NS::VtArray<PXR_NS::GfVec3f> localPositionsSrc1;
};

struct GeneratedDeformableCollisionFilterData
{
    bool enabled = false;
    PXR_NS::VtArray<uint32_t> groupElemCounts0;
    PXR_NS::VtArray<uint32_t> groupElemIndices0;
    PXR_NS::VtArray<uint32_t> groupElemCounts1;
    PXR_NS::VtArray<uint32_t> groupElemIndices1;
};

using GeneratedDeformableAttachmentDataMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                                                GeneratedDeformableAttachmentData,
                                                                omni::physics::parse::ObjectKey::Hash>;
using GeneratedDeformableCollisionFilterDataMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                                                     GeneratedDeformableCollisionFilterData,
                                                                     omni::physics::parse::ObjectKey::Hash>;

enum class ChangeSource
{
    eUsd,
    eUnknwon,
};

class AttachedStage;
class ChangeSourceBlock
{
public:
    ChangeSourceBlock(AttachedStage& attachedStage, ChangeSource source);
    ~ChangeSourceBlock();

    ChangeSourceBlock(const ChangeSourceBlock&) = delete;
    ChangeSourceBlock& operator=(const ChangeSourceBlock&) = delete;
    ChangeSourceBlock(ChangeSourceBlock&&) = default;
    ChangeSourceBlock& operator=(ChangeSourceBlock&&) = default;

private:
    AttachedStage& mAttachedStage;
    ChangeSource mPrevSource;
};

class AttachedStage
{
public:
    AttachedStage();
    AttachedStage(PXR_NS::UsdStageWeakPtr stage);
    AttachedStage(PXR_NS::UsdStageWeakPtr stage, PhysXUsdPhysicsInterface* iface);
    ~AttachedStage();

    PhysXUsdPhysicsInterface* getPhysXPhysicsInterface()
    {
        return mPhysicsInterface;
    }

    const PhysXUsdPhysicsInterface* getPhysXPhysicsInterface() const
    {
        return mPhysicsInterface;
    }

    PXR_NS::UsdStageWeakPtr getStage()
    {
        return mStage;
    }

    PXR_NS::UsdStageWeakPtr getStage() const
    {
        return mStage;
    }

    long getStageId() const
    {
        return mStageId;
    }

    void setStage(PXR_NS::UsdStageWeakPtr stage)
    {
        mStage = stage;
        mStageId = stage ? PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt() : 0u;
        mExternalAttachPayload = nullptr;
        mExternalReadOrdinal = 0;
        rebuildSource();
    }

    // Attach a consumer-provided, backend-opaque source payload (ADR-0002 M2c).
    // Sets the source trio from the *active* parse backend, which interprets
    // `attachPayload` as its own `AttachTarget::nativeStage` (for ovstage, a
    // `const OvstageAttach*` = instance + dictionary). This is the runtime switch
    // — whichever backend is registered handles it; nothing here names a concrete
    // backend. `attachPayload` is consumer-owned and must outlive the attach.
    //
    // `backingStage` is the USD stage the source mirrors (a Fabric-backed ovstage
    // exposes one via ovstage_get_usd_stage_id). It is kept as mStage so the
    // engine's prim-coupled paths (GetPrimAtPath, stage units) keep working while
    // the (ovstage) source drives parsing. Pass an empty handle when there is no
    // backing stage. Pass nullptr `attachPayload` to clear and return to the USD
    // path.
    void setOvstageSource(const void* attachPayload,
                          PXR_NS::UsdStageWeakPtr backingStage = PXR_NS::UsdStageWeakPtr{},
                          uint64_t readOrdinal = 1);

    // The backend-opaque attach handle for the current source: the live USD
    // stage for a USD attach, or the consumer-provided payload otherwise. Fed to
    // both the parse backend (createSource) and the scan dispatch
    // (scanStage(AttachTarget, ...)) — the single switch point lives there, not
    // at call sites. Valid while this AttachedStage lives.
    omni::physics::parse::AttachTarget attachTarget() const;

    // ------------------------------------------------------------------
    // Path / ObjectKey resolution. AttachedStage owns a UsdSource keyed
    // off the current stage. Callers consuming parse-side descriptors
    // resolve their ObjectKey fields back to SdfPath here when they
    // genuinely need a USD path (USD authoring, change
    // notice handlers, ObjectDb lookups). This is the boundary between
    // the source-agnostic parse library and the USD-coupled runtime.
    // ------------------------------------------------------------------

    // Backend-neutral read access to the attached source — the only source
    // accessor. It exposes the source-agnostic IPhysicsSource contract, so code
    // routed through it does not depend on the backend being USD. Code that
    // genuinely needs USD specifics down-casts on demand via
    // omni::physics::usd::asUsdSource(getSource()) (null under a non-USD
    // backend); getStage() remains the USD-stage escape hatch.
    omni::physics::parse::IPhysicsSource* getSource();
    const omni::physics::parse::IPhysicsSource* getSource() const;

    // Stage units (metersPerUnit / kilogramsPerUnit / upAxis) for this attach,
    // read once from the source in rebuildSource() and cached. This is the single
    // omni.physx-side units accessor — code must read units through here (or the
    // source) rather than reaching to a UsdStage directly, so it works under both
    // the USD and ovstage backends. Returns defaults (1.0 / 1.0 / Z-up) when no
    // source is bound.
    const omni::physics::parse::SourceUnits& getSourceUnits() const
    {
        return mUnits;
    }

    // Source-agnostic sink for physics simulation output (poses/velocities/
    // arrays written back out). Built by the active parse backend in
    // rebuildSource(), so it shares the stage's lifetime. Consumers needing the
    // USD-specific sink API down-cast via asUsdDataWrite() on demand.
    omni::physics::parse::IPhysicsDataWrite* getDataWrite()
    {
        return mDataWrite.get();
    }

    const omni::physics::parse::IPhysicsDataWrite* getDataWrite() const
    {
        return mDataWrite.get();
    }

    // Source-vended push feed of runtime change deltas (ADR-0003). Shares the
    // stage's lifetime (rebuilt with the source); consumers register interests on
    // it. Returns null when no source is attached.
    omni::physics::parse::IChangeFeed* getChangeFeed()
    {
        return mChangeFeed.get();
    }

    // Pull the change delta for an explicit producer-supplied version range and
    // apply it (the ovstage path; engine entry IPhysxSimulation::updateFromOvStage).
    // Delegates to the feed's drainRange, which fires the registered onSourceChange
    // / onSourceGroupComplete callbacks. Returns false if there is no feed or the
    // range could not be served.
    bool updateFromOvStage(uint64_t fromOrdinal, uint64_t toOrdinal)
    {
        if (!mChangeFeed)
            return false;
        const bool ok = mChangeFeed->drainRange(fromOrdinal, toOrdinal);
        if (ok)
        {
            // Advance the read cursor only after a successful drain: a failed
            // range must not leave state claiming those ordinals were consumed.
            mExternalReadOrdinal = toOrdinal;
            flushBufferedChanges(*this, 0.0f);
        }
        return ok;
    }

    PXR_NS::SdfPath pathFor(omni::physics::parse::ObjectKey key) const;
    omni::physics::parse::ObjectKey keyFor(const PXR_NS::SdfPath& path) const;

    // Convenience for diagnostics: the path string of a key without building
    // an SdfPath. Returns "" (never null) when the key/source is invalid, so
    // it is safe to feed straight into %s log formatting.
    const char* textFor(omni::physics::parse::ObjectKey key) const;

    bool isPhysXDefaultSimulator() const
    {
        return mPhysXDefaultSim;
    }

    void setIsPhysXDefaultSimulator(bool val)
    {
        mPhysXDefaultSim = val;
    }

    // Release engine objects and clear per-attach state. Reset paths prebuild a
    // replacement ObjectDb; owner teardown passes false to avoid rebuilding an
    // object that will immediately be destroyed.
    void releasePhysicsObjects(bool rebuildObjectDatabase = true);

    ObjectDb* getObjectDatabase()
    {
        return mObjectDatabase;
    }

    const ObjectDb* getObjectDatabase() const
    {
        return mObjectDatabase;
    }

    PathKeyMap& getAnimatedKinematicBodies()
    {
        return mAnimatedKinematicBodies;
    }

    const PathKeyMap& getAnimatedKinematicBodies() const
    {
        return mAnimatedKinematicBodies;
    }

    void removeAnimatedKinematicBody(const PXR_NS::SdfPath& path)
    {
        PathKeyMap::const_iterator fit = mAnimatedKinematicBodies.find(path);
        if (fit != mAnimatedKinematicBodies.end())
        {
            mAnimatedKinematicBodies.erase(fit);
        }
    }

    PathKeyMap& getAnimatedKinematicDeformableBodies()
    {
        return mAnimatedKinematicDeformableBodies;
    }
    const PathKeyMap& getAnimatedKinematicDeformableBodies() const
    {
        return mAnimatedKinematicDeformableBodies;
    }
    void removeAnimatedKinematicDeformableBody(const PXR_NS::SdfPath& path)
    {
        PathKeyMap::const_iterator fit = mAnimatedKinematicDeformableBodies.find(path);
        if (fit != mAnimatedKinematicDeformableBodies.end())
        {
            mAnimatedKinematicDeformableBodies.erase(fit);
        }
    }

    PrimUpdateMap& getPrimUpdateMap()
    {
        return mPrimUpdateMap;
    }

    const PrimUpdateMap& getPrimUpdateMap() const
    {
        return mPrimUpdateMap;
    }

    PrimChangeMap& getPrimChangeMap()
    {
        return mPrimChangeMap;
    }

    const PrimChangeMap& getPrimChangeMap() const
    {
        return mPrimChangeMap;
    }

    TimeSampleMap& getTimeSampleMap()
    {
        return mTimeSampledAttributes;
    }

    const TimeSampleMap& getTimeSampleMap() const
    {
        return mTimeSampledAttributes;
    }

    void bufferRequestRigidBodyMassUpdate(const PXR_NS::SdfPath& path)
    {
        mRigidBodyMassUpdateMap.insert(path);
    }

    void registerTimeSampledAttribute(const PXR_NS::SdfPath& attributePath, OnUpdateObjectFn onUpdate);

    void unregisterTimeSampledAttribute(const PXR_NS::SdfPath& attributePath);

    void registerStageSpecificAttribute(ChangeParams& changeParam);

    void clearStageSpecificAttributes();

    void registerObjectId(const PXR_NS::SdfPath& path, const ObjectCategory& category, const ObjectId& newEntryId);

    void updateRigidBodyMass();

    const ObjectIdMap* getObjectIds(const PXR_NS::SdfPath& path) const;

    CollisionGroupsMap& getCollisionGroupMap()
    {
        return mCollisionGroupsMap;
    }

    const CollisionGroupsMap& getCollisionGroupMap() const
    {
        return mCollisionGroupsMap;
    }

    std::vector<CollisionGroupsMap>& getAdditionalCollisionGroupMaps()
    {
        return mAdditionalCollisionGroupMaps;
    }

    const std::vector<CollisionGroupsMap>& getAdditionalCollisionGroupMaps() const
    {
        return mAdditionalCollisionGroupMaps;
    }

    DeformableAttachmentHistoryMap& getDeformableAttachmentHistoryMap()
    {
        return mDeformableAttachmentHistoryMap;
    }

    DeformableCollisionFilterHistoryMap& getDeformableCollisionFilterHistoryMap()
    {
        return mDeformableCollisionFilterHistoryMap;
    }

    void setGeneratedDeformableAttachmentData(omni::physics::parse::ObjectKey key,
                                              const GeneratedDeformableAttachmentData& data)
    {
        mGeneratedDeformableAttachmentData[key] = data;
    }

    const GeneratedDeformableAttachmentData* getGeneratedDeformableAttachmentData(
        omni::physics::parse::ObjectKey key) const
    {
        auto it = mGeneratedDeformableAttachmentData.find(key);
        return it != mGeneratedDeformableAttachmentData.end() ? &it->second : nullptr;
    }

    void setGeneratedDeformableCollisionFilterData(omni::physics::parse::ObjectKey key,
                                                   const GeneratedDeformableCollisionFilterData& data)
    {
        mGeneratedDeformableCollisionFilterData[key] = data;
    }

    const GeneratedDeformableCollisionFilterData* getGeneratedDeformableCollisionFilterData(
        omni::physics::parse::ObjectKey key) const
    {
        auto it = mGeneratedDeformableCollisionFilterData.find(key);
        return it != mGeneratedDeformableCollisionFilterData.end() ? &it->second : nullptr;
    }

    void clearGeneratedDeformableAttachmentData()
    {
        mGeneratedDeformableAttachmentData.clear();
        mGeneratedDeformableCollisionFilterData.clear();
    }

    void clearGeneratedDeformableAttachmentDataUnderPath(const PXR_NS::SdfPath& parentPath)
    {
        for (auto it = mGeneratedDeformableAttachmentData.begin(); it != mGeneratedDeformableAttachmentData.end();)
        {
            it = pathFor(it->first).HasPrefix(parentPath) ? mGeneratedDeformableAttachmentData.erase(it) : std::next(it);
        }
        for (auto it = mGeneratedDeformableCollisionFilterData.begin(); it != mGeneratedDeformableCollisionFilterData.end();)
        {
            it = pathFor(it->first).HasPrefix(parentPath) ? mGeneratedDeformableCollisionFilterData.erase(it) : std::next(it);
        }
    }

    bool isReplicatorStage() const
    {
        return mReplicatorStage;
    }

    void setReplicatorStage(bool val)
    {
        mReplicatorStage = val;
    }

    void addReplicatorMemoryBlock(void* memory)
    {
        mReplicatorMemory.push_back(memory);
    }

    void freeReplicatorMemory()
    {
        for (void* mem : mReplicatorMemory)
        {
            free(mem);
        }

        mReplicatorMemory.clear();
    }

    void setUseReplicatorEnvIds(bool val)
    {
        mUseReplicatorEnvIds = val;
    }

    bool isUsingReplicatorEnvIds() const
    {
        return mUseReplicatorEnvIds;
    }

    // Running base for explicit replicator env-ids. Each clone() batch on this attach numbers its
    // copies 1..N; the base offsets later batches by the count already assigned, so a second clone
    // does not reuse the first batch's env-ids (which under GPU broadphase would merge two distinct
    // environments into one for collision filtering). Reset with the stage.
    uint32_t getReplicatorEnvIdBase() const
    {
        return mReplicatorEnvIdBase;
    }

    void advanceReplicatorEnvIdBase(uint32_t count)
    {
        mReplicatorEnvIdBase += count;
    }

    // Raise the base to at least `atLeast` — used after a batch with caller-supplied env ids so a
    // later positional batch cannot alias an explicitly-placed environment.
    void raiseReplicatorEnvIdBase(uint32_t atLeast)
    {
        if (mReplicatorEnvIdBase < atLeast)
            mReplicatorEnvIdBase = atLeast;
    }

    // Target paths of successful IPhysxSimulation::cloneEnvironments calls on this attach.
    // Runtime clones author no USD prim, so a USD walk cannot see them; this record is what
    // lets a later clone reject a reused or nested (ancestor/descendant) target, which would
    // stack duplicate live actors on one path. Lives and dies with the attach, like the clones.
    const std::set<PXR_NS::SdfPath>& getRuntimeCloneTargets() const
    {
        return mRuntimeCloneTargets;
    }

    void addRuntimeCloneTarget(const PXR_NS::SdfPath& path)
    {
        mRuntimeCloneTargets.insert(path);
    }

    void setChangeSource(ChangeSource source)
    {
        mChangeSource = source;
    }

    ChangeSource getChangeSource() const
    {
        return mChangeSource;
    }

    ChangeSourceBlock getChangeSourceBlock(ChangeSource source)
    {
        return ChangeSourceBlock(*this, source);
    }

    uint32_t registerEnvIdFromToken(const PXR_NS::TfToken& token)
    {
        TokenEnvIdMap::const_iterator fit = mTokenEnvIdMap.find(token);
        if (fit == mTokenEnvIdMap.end())
        {
            const uint32_t envIdInt = mEnvIdCounter++;
            mTokenEnvIdMap[token] = envIdInt;
            return envIdInt;
        }
        else
        {
            return fit->second;
        }
    }

    uint32_t getEnvIdFromToken(const PXR_NS::TfToken& token) const
    {
        TokenEnvIdMap::const_iterator fit = mTokenEnvIdMap.find(token);
        if (fit != mTokenEnvIdMap.end())
        {
            return fit->second;
        }
        else
        {
            CARB_LOG_WARN("EnvId not found for given scene partition token: %s", token.GetText());
            return 0;
        }
    }

private:
    PhysXUsdPhysicsInterface* mPhysicsInterface;
    PXR_NS::UsdStageWeakPtr mStage;
    long mStageId;
    ObjectDb* mObjectDatabase;
    PrimUpdateMap mPrimUpdateMap;
    PrimChangeMap mPrimChangeMap;
    TimeSampleMap mTimeSampledAttributes;
    PathKeyMap mAnimatedKinematicBodies;
    PathKeyMap mAnimatedKinematicDeformableBodies;
    std::set<PXR_NS::SdfPath> mRigidBodyMassUpdateMap;
    CollisionGroupsMap mCollisionGroupsMap;
    std::vector<CollisionGroupsMap> mAdditionalCollisionGroupMaps;
    DeformableAttachmentHistoryMap mDeformableAttachmentHistoryMap;
    DeformableCollisionFilterHistoryMap mDeformableCollisionFilterHistoryMap;
    GeneratedDeformableAttachmentDataMap mGeneratedDeformableAttachmentData;
    GeneratedDeformableCollisionFilterDataMap mGeneratedDeformableCollisionFilterData;
    bool mReplicatorStage;
    bool mPhysXDefaultSim;

    bool mUseReplicatorEnvIds;
    uint32_t mEnvIdCounter;
    uint32_t mReplicatorEnvIdBase;
    TokenEnvIdMap mTokenEnvIdMap;
    std::set<PXR_NS::SdfPath> mRuntimeCloneTargets;

    std::vector<void*> mReplicatorMemory;
    ChangeSource mChangeSource{ ChangeSource::eUnknwon };

    // Active parse source trio (ADR-0005), (re)built by the registered backend
    // in rebuildSource() when setStage is called. Owned and exposed purely
    // through the backend-neutral interfaces — no concrete backend type leaks
    // into this consumer.
    std::unique_ptr<omni::physics::parse::IPhysicsSource> mSource;
    // Stage units cached from mSource in rebuildSource() — see getSourceUnits().
    omni::physics::parse::SourceUnits mUnits;
    std::unique_ptr<omni::physics::parse::IPhysicsDataWrite> mDataWrite;
    // Runtime change feed (ADR-0003), vended by the source and rebuilt in
    // lockstep with it. Owns this stage's change listener and drives the
    // onSourceChange / onSourceGroupComplete consumer callbacks.
    std::unique_ptr<omni::physics::parse::IChangeFeed> mChangeFeed;

    // Consumer-provided, backend-opaque AttachTarget payload (ADR-0002 M2c) —
    // non-null selects an external parse/scan backend over the USD stage. Set
    // via setOvstageSource(); consumer-owned (must outlive the attach). Kept as
    // const void* so this header needs no ovstage (or any backend) dependency.
    const void* mExternalAttachPayload = nullptr;
    // Optional snapshot override carried through AttachTarget for runtime ovstage
    // re-scans. 0 means the backend should use its payload's attach-time ordinal.
    uint64_t mExternalReadOrdinal = 0;

    void rebuildSource();
};

} // namespace usdparser
} // namespace physx
} // namespace omni
