// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-1
 */

// Object enumeration shared by the ovstage read (OvxPhysicsRead.cpp) and write (OvxPhysicsWrite.cpp).
//
// The two directions select the SAME objects: a write session is opened against a read query, so it
// must resolve the query to the same bodies, in the same order, under the same scope rules. Keeping
// one walk here is not deduplication for its own sake -- the scope fallback below (active-actor set,
// else kinematic-or-sleeping) and the per-scene bucketing are subtle enough that a second copy would
// agree today and drift later, and a write that enumerated differently from the read would publish a
// prim list that does not match the rows it scatters into.

#include <omni/physx/IOvxPhysicsRead.h>       // kOvxActive and the scope enum
#include <omni/physics/parse/Handles.h>      // ObjectKey
#include <ovstage/ovx_path_dictionary.h>     // ovx_primpath_t

// PxU32 only. The class types below stay forward-declared: this header is included by both the read
// and the write, and pulling PxPhysicsAPI.h in would put the whole SDK on their compile paths.
#include <foundation/PxSimpleTypes.h>
// forEachLiveActiveActor is a template, but InternalScene* and PxScene* are NOT dependent
// types in it, so both are looked up where it is DEFINED and have to be complete here.
#include <PxActor.h>
#include <PxRigidDynamic.h> // isDisabledRigidDynamic: PxRigidBody::is<> + PxActorFlag
#include <PxScene.h>
#include "internal/InternalScene.h"

// Dependency-free and shared with the tensors layer: the DOF record both directions index through.
#include "tensors/ArticulationDofOvStageRecord.h"
// Same, for the tendon rows the write scatters through.
#include "tensors/ArticulationTendonOvStageRecord.h"
// Same, for the vehicle wheel rows.
#include "tensors/VehicleWheelOvStageRecord.h"

#include <algorithm> // std::any_of (anyDisabledRigidDynamic)
#include <cstddef>
#include <cstdint>
#include <functional> // std::hash (RigidReadCacheKeyHash)
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physx::usdparser
{
class AttachedStage;
}
namespace omni::physics::parse
{
class IPhysicsSource;
}

namespace omni::physx::internal
{
class InternalScene;
class InternalParticleSet;
class InternalDeformableBody;
}

namespace physx
{
class PxScene;
class PxRigidBody;
class PxArticulationLink;
class PxDeformableBody;
class PxRigidDynamic;
class PxArticulationReducedCoordinate;
class PxArticulationJointReducedCoordinate;
}

namespace omni::physx::internal
{
class InternalJoint;
}

namespace omni::physx::tensors
{
struct ArticulationEntry;
class BasePointInstancerView;
}

namespace omni::physx::ovx
{

using omni::physics::parse::ObjectKey;

// The attached ovstage simulation both directions operate against. Every entry point resolves this
// first: without a source there is no path dictionary, so neither a read column nor a write group can
// name the prims it covers. `who` labels the error.
struct ActiveContext
{
    omni::physx::usdparser::AttachedStage* stage = nullptr;
    omni::physics::parse::IPhysicsSource* source = nullptr;
};

bool getActiveContext(const char* who, ActiveContext& out);

// Active-actor tracking. When a scene runs with PxSceneFlag::eENABLE_ACTIVE_ACTORS
// (the same set the USD write-back sink consumes), getActiveActors() reports the
// exact bodies the solver moved last step. We map each back to its record index
// via PxActor::userData. Membership is tracked PER REPORTING SCENE, so a caller whose scene does
// not carry the flag falls back to isSleeping() for that scene alone (ADR-0007 oq section 2).
struct ActiveActorSet
{
    // The scenes that can actually report an active list, NOT a single "somebody can" flag.
    //
    // A stage can mix them: PhysXScene clears eENABLE_ACTIVE_ACTORS on a readback-suppressed
    // (DirectGPU) scene, so a CPU + DirectGPU stage has one of each. With one global flag, a
    // reporting scene A made the flag true, a non-reporting scene B contributed no bits, and every
    // call site still took the isActive() branch for B's actors -- so every body and link in B
    // silently vanished from an ACTIVE read instead of falling back to its own sleep state.
    // Membership has to be asked PER OWNING SCENE, which is what sceneReports() is for.
    //
    // A vector and a linear scan rather than a set: this holds one entry per reporting scene, and
    // stages have a handful of scenes at most.
    std::vector<const ::physx::PxScene*> reportingScenes;

    // Record indices the solver moved last step, one BIT each rather than a hash set.
    //
    // The representation is the whole cost of this type. The set is a per-step fact, so it is
    // rebuilt from scratch on every ACTIVE read and never cached -- which makes its BUILD, not its
    // lookup, the thing to pay attention to. As a hash set that build was one insert per moved
    // actor with rehash growth underneath, and at 163,840 movers it measured ~17 ms against a
    // device read of 668 us: the scope cost 26x the read it was scoping. A bit per record is ~20 KB
    // at that size, marked in O(1) with no hashing and no reallocation, and small enough to stay in
    // cache for the sequential probes the enumeration loops do.
    //
    // Nothing outside this struct ever iterates the set -- every consumer asks isActive() -- which
    // is what makes the representation free to change.
    std::vector<bool> moved;

    // Sized to the record table, because that is what the indices index. Marking is bounds-checked
    // rather than asserted: `userData` is whatever the actor was created with, and a record table
    // that shrank between the actor's creation and this read would otherwise write out of range.
    void sizeTo(size_t recordCount)
    {
        moved.assign(recordCount, false);
    }
    void mark(size_t recIdx)
    {
        if (recIdx < moved.size())
            moved[recIdx] = true;
    }
    bool isActive(size_t recIdx) const
    {
        return recIdx < moved.size() && moved[recIdx];
    }

    void addReportingScene(const ::physx::PxScene* scene)
    {
        if (scene && !sceneReports(scene))
            reportingScenes.push_back(scene);
    }

    // Whether THIS scene's actors are represented in `moved`. A caller whose scene does not report
    // must use its own fallback (sleep state) rather than reading an unmarked bit as "not moving".
    bool sceneReports(const ::physx::PxScene* scene) const
    {
        for (const ::physx::PxScene* s : reportingScenes)
            if (s == scene)
                return true;
        return false;
    }

    // Only for deciding whether the set was worth building at all; never for gating a lookup.
    bool anySceneReports() const
    {
        return !reportingScenes.empty();
    }
};

ActiveActorSet collectActiveActors();

// THE one safe walk over a scene's active actors. Every consumer goes through here.
//
// A RELEASED actor stays in this list until the next simulation replaces the array, and PhysX puts
// the tracking on us -- PxScene.h on getActiveActors: "This list may contain actors that have been
// released after fetchResults() of the previous simulation step. It is the user's responsibility to
// track such actors and avoid dereferencing the corresponding pointers."
//
// So the null check is not the guard it looks like: a released entry is non-null and DANGLING, and
// reading userData off it is the dereference PhysX is warning about. The engine's own consumers
// filter on the same predicate (PhysXStepper::updateQuasistaticActors).
//
// `visit` is called only with actors that are safe to dereference. It exists as one function
// because the predicate was previously open-coded per caller, and a caller that copied the null
// check without the tombstone -- which is what the instancer mask did -- is a use-after-free that
// reads as ordinary code.
template <typename Visitor>
void forEachLiveActiveActor(omni::physx::internal::InternalScene* internalScene, Visitor&& visit)
{
    ::physx::PxScene* const scene = internalScene ? internalScene->getScene() : nullptr;
    if (!scene)
        return;

    const bool hasReleased = internalScene->hasReleasedActiveActors();

    ::physx::PxU32 count = 0;
    ::physx::PxActor** active = scene->getActiveActors(count);
    for (::physx::PxU32 i = 0; i < count; ++i)
    {
        if (!active[i] || (hasReleased && internalScene->isReleasedActiveActor(active[i])))
            continue;
        visit(active[i]);
    }
}

// The single-scene forms, for callers that already know which scene they mean. Declared beside the
// cross-scene collector so the three cannot drift on what "active" means.
void markActiveActorsOfScene(omni::physx::internal::InternalScene* internalScene, ActiveActorSet& out);
ActiveActorSet collectActiveActorsForScene(::physx::PxScene* scene);

// Standalone bodies of ONE scene, in database order. Emission is per-scene (each body must be
// reached through its owning scene's view), so the scan buckets by scene as it walks rather than
// producing one flat list that every scene then re-filters.
struct RigidSceneBucket
{
    ::physx::PxScene* scene = nullptr;
    std::vector<::physx::PxRigidBody*> bodies;
    std::vector<ObjectKey> keys; // parallel to `bodies`
};

struct RigidRecordScan
{
    std::vector<RigidSceneBucket> byScene;  // only scenes owning >= 1 standalone body
    std::vector<::physx::PxRigidDynamic*> instanced; // point-instancer instances (any scene)

};

// One walk, bucketed by owning scene. getScene() is called where the actor was just dereferenced for
// the type check and is still cache-resident -- a separate per-scene filtering pass paid that
// indirection again on a working set far past L2, plus a full copy of both vectors.
// The InternalScene that owns `scene`, or null. Here rather than in either direction's file
// because both the read's enumerations and the shared scans resolve it.
omni::physx::internal::InternalScene* internalSceneOf(const ::physx::PxScene* scene);

// Whether an articulation link is in scope. Shared by the READ's scan and by query DISCOVERY so the
// two cannot disagree about which links a scope admits.
bool linkInScope(const ::physx::PxArticulationLink& link,
                 size_t recordIndex,
                 uint32_t scope,
                 const ActiveActorSet& activeSet);

void scanRigidRecords(uint32_t scope, const ActiveActorSet& activeSet, RigidRecordScan& out);

// The link counterpart of scanRigidRecords, shared for the same reason: a write session over
// articulation links must select exactly the links the read selects, under the same scope rules.
void scanLinkRecords(uint32_t scope, const ActiveActorSet& activeSet, RigidRecordScan& out);

// The scan already grouped by scene; this is a lookup, not a filter.
const RigidSceneBucket* bucketForScene(const RigidRecordScan& scan, const ::physx::PxScene* scene);

// Articulation enumeration is NOT here. It lives in ArticulationReadCacheEntry::rootArtis below,
// filled by the read's structural walk, which additionally detects duplicate and inconsistent
// ePTArticulation records. An earlier scanArticulationRecords here duplicated that walk; it was
// removed rather than kept in parallel, which is the whole reason this file exists.


// The record walk, run at most once per read and only when some scene needs it.
struct LazyRigidScan
{
    uint32_t scope;
    bool ran = false;
    RigidRecordScan data;
    bool activeRan = false;
    ActiveActorSet active;

    // The read's ONE active-actor set. It belongs here for the same reason the record walk does:
    // several things in one read want it, none of them knows whether another already built it, and
    // building it is O(records) plus a mark per moved actor. Carrying the scope means a caller
    // cannot accidentally be handed a set built for a different one.
    const ActiveActorSet& activeActors()
    {
        if (!activeRan)
        {
            if (scope == ::omni::physx::kOvxActive)
                active = collectActiveActors();
            activeRan = true;
        }
        return active; // unavailable for any other scope, which is what those callers expect
    }

    const RigidRecordScan& get()
    {
        if (!ran)
        {
            scanRigidRecords(scope, activeActors(), data);
            ran = true;
        }
        return data;
    }
};

// Every physics scene in the internal database. Both directions enumerate objects across the whole
// DB, but each object must be reached through its OWN scene's backend view -- a body's PxDirectGPUAPI
// GPU index is only meaningful in the scene that owns it -- so the device paths iterate scenes and
// filter their enumeration to each (multi-scene-safe, ADR-0008). A single-scene attach (the common
// ovphysx case) yields exactly one entry.
// A live physics scene paired with the InternalScene that owns it. Both halves are wanted: the
// PxScene keys every per-scene cache, while the vehicle read needs the InternalScene itself, since
// vehicles hang off it and not off the PxScene.
using ScenePair = std::pair<::physx::PxScene*, omni::physx::internal::InternalScene*>;

// One walk of the record database; the projections below come out of it rather than walking again,
// so there is one definition of "a live physics scene" to keep correct instead of two that have to
// agree.
std::vector<ScenePair> allPhysicsScenesWithInternal();

std::vector<::physx::PxScene*> scenesOf(const std::vector<ScenePair>& pairs);

std::vector<::physx::PxScene*> allPhysicsScenes();

// PhysX frees the DirectGPU state row of an eDISABLE_SIMULATION rigid dynamic, so a device column
// must never source one. Statics and articulation links are never PxRigidDynamic, so this is false
// for them and those sets never filter.
inline bool isDisabledRigidDynamic(::physx::PxRigidBody* const body)
{
    ::physx::PxRigidDynamic* const dynamic = body ? body->is<::physx::PxRigidDynamic>() : nullptr;
    return dynamic && dynamic->getActorFlags().isSet(::physx::PxActorFlag::eDISABLE_SIMULATION);
}

// Stops at the first hit: callers use it only to decide whether to filter or to force a row
// refresh, and it runs over a list that can hold every rigid body in the scene.
inline bool anyDisabledRigidDynamic(const std::vector<::physx::PxRigidBody*>& bodies)
{
    return std::any_of(bodies.begin(), bodies.end(), isDisabledRigidDynamic);
}

// The rows of a matched body/key list that a DirectGPU column may cover. Disabled rigid dynamics
// stay in query discovery and in the backend's stable superset map, and are dropped only here, from
// the emitted/scattered columns.
//
// Detection runs before any copy, so the steady state -- a DirectGPU scene with nothing disabled --
// aliases the caller's lists instead of duplicating them on every read and every planned group.
// `apply` false does the same: the CPU path keeps disabled actors (getGlobalPose() and host
// properties still answer for them), as does the disableSimulation write, whose host flag is the one
// route that can re-enable a body and restore its GPU row.
//
// The two lists must already be the same length; callers check and report that themselves.
//
// Lifetime: `bodies()` / `keys()` alias the constructor arguments when nothing is dropped, so
// those vectors must outlive this object. Copy and move are deleted so a moved-from instance
// cannot leave the aliases pointing at its own emptied members.
class EnabledRigidBodies
{
public:
    EnabledRigidBodies(const std::vector<::physx::PxRigidBody*>& bodies,
                       const std::vector<ObjectKey>& keys,
                       bool apply);
    EnabledRigidBodies(const EnabledRigidBodies&) = delete;
    EnabledRigidBodies& operator=(const EnabledRigidBodies&) = delete;
    EnabledRigidBodies(EnabledRigidBodies&&) = delete;
    EnabledRigidBodies& operator=(EnabledRigidBodies&&) = delete;

    // True when at least one row was dropped. Callers also use it to suppress structural caching:
    // the cache holds the full matched census, which a temporarily filtered set must not overwrite.
    bool anyDisabled() const
    {
        return mAnyDisabled;
    }
    const std::vector<::physx::PxRigidBody*>& bodies() const
    {
        return *mBodies;
    }
    const std::vector<ObjectKey>& keys() const
    {
        return *mKeys;
    }

private:
    std::vector<::physx::PxRigidBody*> mEnabledBodies; // populated only when a row is dropped
    std::vector<ObjectKey> mEnabledKeys;
    const std::vector<::physx::PxRigidBody*>* mBodies;
    const std::vector<ObjectKey>* mKeys;
    bool mAnyDisabled = false;
};

// Per-read work that is fully determined by the scene's actor set: which bodies match, their source
// keys, those keys canonicalised to interned prim handles, and each body's row in the backend's
// superset view. None of it depends on simulated state, yet all of it was recomputed on every read --
// under heavy load the walks and the per-prim canonicalisation dominated, while the device gather
// cost nothing.
//
// SHARED BY BOTH DIRECTIONS. A write session is opened against a read query, so it selects the same
// bodies under the same key and needs exactly what this holds. One cache, keyed and validated one
// way, rather than two that would agree today and drift later. (The name still says "Read"; it is
// kept as-is for now and generalised in a later stage.)
//
// This is the QUERY side; the superset view is the physics side. Nothing here duplicates it. The view
// knows which bodies the scene has and in what order, and nothing about prims: the tensors layer has
// no ovstage types, so keys and interned handles can only live here. `bodies` is this query's matched
// subset, which a superset by definition does not have. Only `rows` is derived -- from `bodies` and
// the view's row map -- and it is kept because deriving it is the expensive part: one hash lookup per
// body, ~1.5-2 ms at 490k prims.
//
// Every field is guarded, but not by the same thing, and the difference matters.
//
// Handles are a pure function of (key, dictionary), so the key list is compared directly, element for
// element, and not through any proxy for it. Nothing derived from the actors can stand in: an actor
// can be destroyed and another allocated at the same address between two reads, with the counts
// unchanged, and then the pointers, the topology and the row map all still agree while the body
// behind a row is a different object. Only the key sees that, because a key is source identity and is
// not recycled by an allocator. Generation covers what keys cannot -- a stage swap replacing the
// dictionary the handles were interned into, under an unchanged key set.
//
// Rows cannot be guarded that way, and caching them once crashed the suite for exactly that reason: a
// row indexes the superset view's ORDERING, which can change while the key set stays identical, and
// stale rows then read out of bounds against a different view. Nothing the caller can derive detects
// it -- the guard has to come from the view that owns the ordering. That guard is `generation`: the
// backend takes a fresh one whenever the superset view is rebuilt, including when the view reports
// itself invalid. So rows ARE cached, and are served only under a generation obtained from
// acquireSceneView for this scene at the point of use. Any change to how the backend versions its
// views is a change to whether this is sound.
//
// kOvxActive is deliberately never cached: the active set is recomputed each step.
struct RigidReadCacheEntry
{
    uint64_t generation = 0; // dictionary identity; see above
    // The database's object-lifetime epoch when `bodies` and `keys` were walked. Generation says the
    // superset view was not rebuilt; this says no object was created or retired, which is the part
    // generation cannot answer on the stageless path.
    uint64_t dbEpoch = 0;
    std::vector<ObjectKey> keys;
    std::vector<ovx_primpath_t> handles; // keys canonicalised once
    std::vector<::physx::PxU32> rows;    // superset rows for `keys`, same order; valid for `generation`
    // Identity of `rows` itself, minted whenever a new list is built (see nextRowsVersion). The device
    // upload gate compares it, so it has to change exactly when the CONTENT does -- not when some
    // proxy for the content does.
    uint64_t rowsVersion = 0;
    // This scene's standalone bodies, same order as `keys`. Dereferenced downstream, so only ever
    // served under a generation obtained from acquireSceneView for THIS scene, at the point of use.
    std::vector<::physx::PxRigidBody*> bodies;
};

struct RigidReadCacheKey
{
    const ::physx::PxScene* scene;
    int type;
    uint32_t scope;
    bool operator==(const RigidReadCacheKey& o) const
    {
        return scene == o.scene && type == o.type && scope == o.scope;
    }
};

struct RigidReadCacheKeyHash
{
    size_t operator()(const RigidReadCacheKey& k) const
    {
        return std::hash<const void*>()(k.scene) ^ (std::hash<int>()(k.type) << 1) ^
               (std::hash<uint32_t>()(k.scope) << 2);
    }
};

extern std::unordered_map<RigidReadCacheKey, RigidReadCacheEntry, RigidReadCacheKeyHash> g_rigidReadCache;

// Drop entries whose scene no longer exists. An entry holds three vectors sized by the scene's body
// count -- several MB each under heavy load -- and the map is keyed by PxScene*, for which there is no
// teardown signal on the stageless path, so an entry for a destroyed scene is never looked up again
// and nothing else would ever free it.
//
// Keyed on liveness rather than an entry count: a count needs a bound, and a bound is a guess about
// how many scenes a workload runs. Guess low and every read evicts an entry it is about to want,
// turning the cache into a pessimisation with nothing logged to say so; guess high and dead entries
// sit around anyway.
void purgeDeadRigidReadCacheEntries(const std::vector<::physx::PxScene*>& scenes);

// The selector `query` was opened with. A write session is opened against a read query and needs the
// same type and scope to enumerate against, but the query table lives in OvxPhysicsRead.cpp with the
// entry points that own its lifetime, so this reaches it rather than duplicating the table.
//
// Defined in OvxPhysicsRead.cpp and takes that file's mutex. LOCK ORDER: the write's session mutex may
// be held across this call; the read never calls into the write, so the hierarchy has one direction
// and cannot close into a cycle. Returns false for an unknown handle.
// ---------------------------------------------------------------------------
// Articulation joint state, shared by the read and the write for the same reason as the rigid cache
// above: a write session is opened against a read query and derives the identical joint set.
// ---------------------------------------------------------------------------

struct JointRec
{
    ::physx::PxArticulationJointReducedCoordinate* pxJoint;
    omni::physx::internal::InternalJoint* ij;
    ObjectKey key;
    uint32_t viewArtiIdx;
};

// One joint's span in the flat DOF column.
struct JointSlice
{
    ObjectKey key;
    uint32_t offset;
    uint32_t count;
};

// Structural data parallel to one authoritative ePTArticulation record. The articulation pointer
// and root key live in ArticulationReadCacheEntry::rootArtis/rootKeys at the same index; this owns
// the database indices of every link so ACTIVE can test any-link membership without another record
// walk.
struct ArticulationRootRec
{
    std::vector<size_t> linkRecordIndices;
};

// What the articulation paths derive before they can gather or scatter, cached per scene so a steady
// stepping loop derives it once instead of once per operation.
//
// THIS TYPE LIVES HERE, and that is the whole point: the read's joint-state and whole-articulation
// paths and the WRITE's joint and root paths all index through it, so there is one enumeration and
// one set of validators rather than one per direction. A write session is opened against a read
// query and must resolve the identical articulations in the identical order; a second cache would
// agree today and drift later, and the write inverts a scale the read applies, so drift there is
// silently wrong joint values rather than a visible failure.
//
// The STORAGE stays in OvxPhysicsRead.cpp along with the structural walk that fills it. Only the
// type and the two accessors below are shared, because the walk is the read's machinery and moving
// it would drag its helpers across for no gain.
//
// Validators, one per group of fields that goes stale for its own reason:
//
//   dbEpoch                covers `joints`, `artis` and the root vectors: a filter over the object
//                          database, so unchanged exactly while no object is created or removed.
//   generation             covers `recs`, `slices` and the interned handles: a record names a
//                          SUPERSET ROW, valid only for the view build that numbered the rows, and
//                          the backend takes a fresh generation when it rebuilds that view.
//   rootGeneration         the same rule for the root rows and handles, kept separate so one path
//                          cannot claim the other's partially rebuilt vectors.
//   rootRowsVersion        identifies the root row LIST for the device selection cache.
//   rootShapeTopologyEpoch covers the per-shape widths, which change without any object being
//                          created or removed.
//
// plus the directGpuReady latch, which is not a validator: it records that this scene has completed
// a step, and is deliberately reset by a structural rebuild.
//
// Measured at 8192 envs: the enumeration is ~1.4ms and building the records ~0.9ms of a ~3.3ms
// read, and neither changes between steps.
struct ArticulationReadCacheEntry
{
    uint64_t dbEpoch = 0;
    uint64_t generation = 0;
    // Content version of `recs`; see TendonReadCacheEntry::recsVersion. Passed to the DOF device
    // upload as its token so it re-uploads on any record-content change, not only a generation change.
    uint64_t recsVersion = 0;
    std::vector<JointRec> joints;
    // View articulations used by the joint path, in first-joint database order. Kept separate from
    // rootArtis so zero-joint roots do not add work or failure modes to that path, and so a
    // legacy/inconsistent joint without an ePTArticulation record retains the old behavior without
    // exposing a fabricated root row.
    std::vector<::physx::PxArticulationReducedCoordinate*> artis;
    std::vector<::physx::PxArticulationReducedCoordinate*> rootArtis;
    std::vector<ObjectKey> rootKeys;
    std::vector<ArticulationRootRec> roots;
    bool duplicateRootPointers = false;

    uint64_t rootGeneration = 0;
    uint64_t rootRowsVersion = 0;
    uint64_t rootShapeTopologyEpoch = 0;
    bool directGpuReady = false;
    std::vector<uint32_t> rootRows;
    std::vector<ovx_primpath_t> rootHandles;

    std::vector<omni::physx::tensors::ArticulationDofOvStageRecord> recs;
    std::vector<JointSlice> slices;
    std::vector<ObjectKey> sliceKeys;
    std::vector<ovx_primpath_t> sliceHandles;
};

// Run the articulation structural walk for `scenes` if the object database has moved since the last
// one, and return false only on a genuine failure. Idempotent and cheap on a hit -- it compares one
// epoch -- so a caller that cannot know whether a read has already run this step should just call it.
//
// Defined in OvxPhysicsRead.cpp. Exposed so the WRITE can fill the cache itself rather than
// requiring the caller to have issued a read first: that precondition was real while the write had
// no way to run the walk, and it is not something a write API should impose.
bool refreshArticulationCache(const std::vector<::physx::PxScene*>& scenes);

// This scene's entry, or null when the walk has not covered it. Never re-runs the walk; call
// refreshArticulationCache first.
ArticulationReadCacheEntry* articulationCacheEntry(const ::physx::PxScene* scene);

// A process-global content version, bumped every time a reader rebuilds one of its ovstage record
// vectors. Used as the device-upload token so a re-uploaded buffer tracks the EXACT host records
// rather than the view generation, which does not move when recordLifetimeEpoch() rebuilds records
// under a same-shape view.
uint64_t mintRecordContentVersion();

// Free every pooled output-column buffer and release every manager reference the read-column pool
// holds, ahead of CUDA/foundation shutdown. Called from OmniPhysX::onShutdown; see the definition.
void ovxDrainColumnPools();

// Derive `jc`'s per-output DOF records, per-joint slices and interned handles against the
// articulation view that numbered `entries` / `localToRow`, or serve them from the cache when
// `generation` still matches what they were built under.
//
// Shared rather than written twice. The derivation is the subtle part of the joint path -- it
// enumerates every unlocked reduced-coordinate axis, resolves the per-axis degree convention from an
// authored JointStateAPI, and folds the body0IsParent sign into the same scale -- and a second copy
// would agree today and drift the first time any of those rules moved. The write inverts that scale,
// so a drift between the two would surface as silently wrong joint values, which is the failure the
// unit-conversion contract already has history with.
bool ensureJointRecords(ArticulationReadCacheEntry& jc,
                        omni::physics::parse::IPhysicsSource& source,
                        const std::vector<JointRec>& joints,
                        const std::vector<omni::physx::tensors::ArticulationEntry>& entries,
                        const std::vector<uint32_t>& localToRow,
                        uint64_t generation,
                        bool enumCached);

// ---------------------------------------------------------------------------
// Articulation tendons, shared by the read and the write for the reason the caches above are: a
// write session is opened against a read query and must derive the identical tendon set, in the
// identical order, or it scatters into rows that do not match the prims it published.
// ---------------------------------------------------------------------------

// One tendon row. The prim is the ROOT of the tendon -- the joint carrying PhysxTendonAxisRootAPI,
// or the link carrying PhysxTendonAttachmentRootAPI. A tendon's other axes and attachments are
// database records too, but they are parts of a tendon rather than tendons, so only roots become
// rows.
struct TendonRec
{
    ObjectKey key;
    uint32_t tendonIdx;   // index within that articulation's tendon list, PhysX's own order
    uint32_t viewArtiIdx; // index into `artis` below
};

// Same two-validator split as ArticulationReadCacheEntry, for the same reasons: the enumeration is a
// filter over the object database, and the records name superset rows of a particular view build.
//
// Tendon properties are authoring-time values that a step never changes, so on a stepping loop
// nothing here is ever rebuilt -- the epoch and the generation both hold.
struct TendonReadCacheEntry
{
    uint64_t dbEpoch = 0;
    uint64_t generation = 0;
    // Content version of `recs`: bumped whenever `recs` is rebuilt, and passed to the view's device
    // upload as the token. The view generation alone is insufficient -- it does not move when an epoch
    // change rebuilds records under a same-shape view, which served stale device records.
    uint64_t recsVersion = 0;
    std::vector<TendonRec> tendons;
    std::vector<::physx::PxArticulationReducedCoordinate*> artis;
    std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord> recs;
    std::vector<ObjectKey> keys;
    std::vector<ovx_primpath_t> handles;
};

// Enumerate `scene`'s tendons of one kind into the shared cache if the object database has moved,
// and return that scene's entry. Never null: an entry is inserted even when the scene owns no
// tendons, because that empty answer is exactly what the cache exists to avoid re-deriving.
//
// Defined in OvxPhysicsRead.cpp beside the walk that fills it, and exposed here so the WRITE runs
// the same walk rather than a second one -- and so it does not have to require the caller to issue a
// read first, which is a precondition a write API has no business imposing.
TendonReadCacheEntry& tendonCacheEntry(::physx::PxScene* scene, bool fixed);

// ---------------------------------------------------------------------------
// Vehicle wheels, shared by the read and the write for the reason the caches above are: a write
// session opened against a read query must derive the identical wheel set in the identical order.
// ---------------------------------------------------------------------------

// TWO validators, and note the first is NOT the object-lifetime epoch the other caches use.
//
//   setEpoch    the VEHICLE SET epoch (InternalScene::mVehicleSetEpoch, reported by
//               BaseVehicleView::getBuiltEpoch). The read's TASK-06 records that the object-lifetime
//               epoch was the wrong validator here -- vehicles are renumbered by changes it does not
//               observe -- so a consumer caching rows must compare this one.
//   generation  the backend view build the rows are numbered against, as everywhere else.
struct VehicleReadCacheEntry
{
    uint64_t setEpoch = 0;
    uint64_t generation = 0;
    std::vector<omni::physx::tensors::VehicleWheelOvStageRecord> recs;
    std::vector<ObjectKey> keys;
    std::vector<ovx_primpath_t> handles;
};

// This scene's entry, or null when no vehicle enumeration has been derived for it. Defined in
// OvxPhysicsRead.cpp beside the walk that fills it.
//
// Unlike the articulation and tendon accessors, this one does NOT refresh: the vehicle walk needs a
// CpuSimulationView and the scene's InternalScene, which the read already holds at that point and
// the write would have to re-acquire. The write reports the miss instead -- see the vehicle branch
// in planGroups.
VehicleReadCacheEntry* vehicleCacheEntry(const ::physx::PxScene* scene);

// ---------------------------------------------------------------------------
// Point-instancer instances, shared by the read and the write.
// ---------------------------------------------------------------------------

// What the WRITE needs to publish an instancer group, and nothing more.
//
// Deliberately NOT the read's InstancerAccum. That type carries GfMatrix4d and PxVec3 by value, and
// this header is USD-free and PhysX-SDK-free on purpose -- it is included by both directions, and
// pulling either dependency in would put it on both compile paths. An earlier attempt moved
// InstancerAccum here and broke exactly that.
//
// So the seam is the VIEW, not the enumeration's intermediate types: the read owns the walk, the Gf
// math and the view's lifetime; the write receives a view it can scatter through plus the prim keys
// and array lengths it must name.
struct InstancerWriteTargets
{
    // Null when this scene has no instancers, or when the enumeration could not be served.
    //
    // The BASE type, so one struct serves both devices: a DirectGPU scene yields the GPU view the
    // read already caches, a host scene the CPU one. The scatter is virtual, so the write does not
    // branch on which it got.
    omni::physx::tensors::BasePointInstancerView* view = nullptr;
    std::vector<ObjectKey> keys;        // one per instancer, in view order
    std::vector<uint32_t> arrayLengths; // instance-array length per instancer, parallel to `keys`
};

// Build (or serve from cache) the host instancer view for `scene` and describe its instancers.
//
// Defined in OvxPhysicsRead.cpp beside enumerateInstancers, which it drives. The view is owned by
// that cache and must not be freed by the caller; it is valid until the object-lifetime epoch or the
// backend generation moves, which is what the cache compares.
//
// Returns false only on a genuine failure. A scene with no instancers yields true and a null view.
bool instancerWriteTargets(::physx::PxScene* scene, uint32_t scope, InstancerWriteTargets& out);

// What the WRITE needs to publish a deformable simulation-mesh group (ADR-0012).
//
// Unlike the particle seam next door, the destination here is PhysX's OWN DEVICE BUFFER
// (getSimPositionInvMassBufferD and friends), not a host staging array -- so a deformable write is a
// device scatter followed by markDirty, where a particle write is a host store followed by an upload
// flag. Same enumeration shape, different publish mechanism, and the difference is the engine's.
struct DeformableWriteTargets
{
    std::vector<ObjectKey> keys; // the SIM MESH prim per body, in the read's enumeration order
    // Parallel to `keys`. Both pointers are opaque here: the concrete PhysX type carries the buffer
    // getters and markDirty, and the internal body carries the sim-mesh reframe -- and this header
    // is PhysX-SDK-free, so the write includes what it needs to dereference them.
    std::vector<::physx::PxDeformableBody*> bodies;
    std::vector<omni::physx::internal::InternalDeformableBody*> internals;
    std::vector<uint32_t> counts; // sim-mesh vertex count per body
};

// Describe `scene`'s writable deformable bodies of one KIND, under the read's admission rules and in
// its order. `isVolume` picks the kind, exactly as the read's own type dispatch does.
//
// Defined in OvxPhysicsRead.cpp beside particleWriteTargets, and for the same reason.
bool deformableWriteTargets(::physx::PxScene* scene, bool isVolume, DeformableWriteTargets& out);

// What the WRITE needs to publish a particle group (ADR-0012).
//
// The particle seam is the SET, not a view -- and that is the difference from instancers rather than
// an inconsistency. GpuPointSetReadView gathers from PhysX's own device buffers and owns no engine
// data, so there is nothing on it to scatter INTO. The write's destination is the set's pinned host
// staging pair plus its upload flags, which is the same mechanism USD authoring and the properties
// update already publish these two quantities through.
//
// Routing both through one mechanism is what makes a written column readable before the next step:
// the read's pending path serves mPositions / mVelocities exactly when the matching upload flag is
// raised, which is the flag this write raises.
struct ParticleWriteTargets
{
    std::vector<ObjectKey> keys; // one per set, in the read's enumeration order
    // Parallel to `keys`. The pointer is opaque to this header on purpose -- InternalParticle.h
    // carries PxVec4 members, and this header is included by both directions.
    std::vector<omni::physx::internal::InternalParticleSet*> sets;
    std::vector<uint32_t> counts; // particle count per set, parallel to `keys`
};

// Describe `scene`'s writable particle sets, in the SAME order and under the same admission rules
// the read uses -- it drives collectParticleEntries, so a set the read will not serve is not one the
// write offers either.
//
// Defined in OvxPhysicsRead.cpp beside instancerWriteTargets, for the same reason: the enumeration
// and its admission rules live there, and a second copy here would be a second thing to keep true.
//
// Returns false only on a genuine failure. A scene with no particle sets yields true and empty
// vectors.
bool particleWriteTargets(::physx::PxScene* scene, uint32_t scope, ParticleWriteTargets& out);

bool queryTypeScope(uint64_t query, uint32_t& outType, uint32_t& outScope);

// Identity for a row list, handed to a rigid-body view as its upload token: the view re-uploads its
// device copy exactly when this changes, so it must change exactly when the CONTENTS do.
//
// ONE counter for every producer, and that is the point rather than tidiness. A view's row cache is a
// small shared LRU keyed by this token, so reads, writes and the point-instancer path all feed it into
// the same comparison. Two counters would hand out the same value for different lists, and a slot would
// then keep one list's rows and index another's data through them -- wrong values, silently, with
// nothing to fail on.
//
// A read and a write over the same query resolve the SAME rows and now share one cache entry, so they
// share its rowsVersion too: whichever runs first mints it, the other reuses it, and the row list is
// uploaded once rather than once per direction. Mint a new one only where a new list is genuinely
// built.
uint64_t nextRowsVersion();

} // namespace omni::physx::ovx
