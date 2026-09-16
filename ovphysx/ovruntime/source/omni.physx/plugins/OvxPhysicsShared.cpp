// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-1
 *
 * Moved out of OvxPhysicsRead.cpp unchanged when the write API needed the same enumeration; see the
 * header for why the two directions share one walk rather than each keeping its own.
 */

#include "OvxPhysicsShared.h"

#include <omni/physx/IOvxPhysicsRead.h> // kOvxActive

#include "OmniPhysX.h"
#include "internal/InternalPhysXDatabase.h"
#include "internal/InternalActor.h"
#include "internal/InternalScene.h"
#include "PhysXTools.h" // radToDeg
#include "tensors/CommonTypes.h"      // ArticulationEntry
#include "tensors/ArticulationMetatype.h"
#include <OvstageOutput.h> // canonicalisePathHandles; private to omni.physics.ovstage
#include "internal/Internal.h" // recordLifetimeEpoch
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physx/IPhysx.h> // PhysXType (ePTActor / ePTScene)

#include <carb/logging/Log.h>

#include <algorithm> // std::find (purgeDeadRigidReadCacheEntries)
#include <atomic>   // mintRecordContentVersion

#include <PxPhysicsAPI.h>

using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using omni::physics::parse::IPhysicsSource;

namespace omni::physx::ovx
{

std::unordered_map<RigidReadCacheKey, RigidReadCacheEntry, RigidReadCacheKeyHash> g_rigidReadCache;

void purgeDeadRigidReadCacheEntries(const std::vector<PxScene*>& scenes)
{
    for (auto it = g_rigidReadCache.begin(); it != g_rigidReadCache.end();)
    {
        const bool live = std::find(scenes.begin(), scenes.end(), it->first.scene) != scenes.end();
        it = live ? std::next(it) : g_rigidReadCache.erase(it);
    }
}

uint64_t mintRecordContentVersion()
{
    static std::atomic<uint64_t> sVersion{ 1 };
    return sVersion.fetch_add(1, std::memory_order_relaxed);
}

bool ensureJointRecords(ArticulationReadCacheEntry& jc,
                        IPhysicsSource& source,
                        const std::vector<JointRec>& joints,
                        const std::vector<omni::physx::tensors::ArticulationEntry>& entries,
                        const std::vector<uint32_t>& localToRow,
                        const uint64_t viewGeneration,
                        const bool enumCached)
{
    // Bit-identical to the host radToDeg scaling, which is what keeps the device and host joint
    // paths agreeing to the last bit rather than to a tolerance.
    const float degFactor = radToDeg(1.0f);
    // The INVERSE constant, taken from degToRad rather than as 1/degFactor. Those are not the same
    // float, and the write multiplies by this where the read multiplies by degFactor -- so deriving
    // one from the other would leave a value that does not round-trip. Upstream's record splits the
    // two fields for exactly this reason; this is the producer side of that.
    const float radFactor = degToRad(1.0f);
    const bool recsCached = enumCached && jc.generation == viewGeneration && viewGeneration != 0;
    if (!recsCached)
    {
        jc.recs.clear();
        jc.slices.clear();
        jc.sliceKeys.clear();
        jc.sliceHandles.clear();
        for (const JointRec& jr : joints)
        {
            const uint32_t artiRow = localToRow[jr.viewArtiIdx];
            const omni::physx::tensors::ArticulationEntry& ent = entries[artiRow];
            const uint32_t start = static_cast<uint32_t>(jc.recs.size());
            for (uint32_t dof = 0; dof < ent.dofImpls.size(); ++dof)
            {
                if (ent.dofImpls[dof].joint != jr.pxJoint)
                    continue; // a DOF of another joint of the same articulation
                // Emit EVERY unlocked reduced-coordinate DOF, mirroring the host path's
                // getMotion != eLOCKED enumeration: a joint carrying a JointStateAPI on only a
                // subset of its axes still reports all of them (NvBugs 6481083 / OMPE-102219).
                // Angular axes convert rad->deg by default; an authored JointStateAPI on the axis
                // overrides that per-axis flag. The DirectGPU/cache DOF value is unsigned-raw, so
                // the body0IsParent sign is folded into the scale here.
                const ::physx::PxArticulationAxis::Enum axis = ent.dofImpls[dof].axis;
                bool toDegrees = (axis == ::physx::PxArticulationAxis::eTWIST ||
                                  axis == ::physx::PxArticulationAxis::eSWING1 ||
                                  axis == ::physx::PxArticulationAxis::eSWING2);
                for (const InternalJoint::InternalJointState& js : jr.ij->mJointStates)
                    if (js.enabled && js.physxAxis == axis)
                    {
                        toDegrees = js.convertToDegrees;
                        break;
                    }
                const float sign = (ent.metatype && ent.metatype->isDofBody0Parent(dof)) ? 1.0f : -1.0f;
                omni::physx::tensors::ArticulationDofOvStageRecord r;
                r.viewArtiIdx = artiRow;
                r.physxDofIdx = dof;
                // The axis FACTS, not one folded multiply: the fold is per ATTRIBUTE while this list
                // is shared by all of them, so the record carries the angular scale, its exact
                // inverse and the body-order sign, and each attribute's DofScalePolicy decides which
                // of them apply. A single `scale` field served only the attribute it was built for.
                r.angScale = toDegrees ? degFactor : 1.0f;
                r.invAngScale = toDegrees ? radFactor : 1.0f;
                r.sign = sign;
                jc.recs.push_back(r);
            }
            const uint32_t count = static_cast<uint32_t>(jc.recs.size()) - start;
            if (count > 0)
                jc.slices.push_back({ jr.key, start, count });
        }
    }
    // Held by reference, and built in place above: a hit is what this cache exists to make free,
    // and copying four vectors out would put an O(numDOF) copy back on it.
    const std::vector<omni::physx::tensors::ArticulationDofOvStageRecord>& recs = jc.recs;
    const std::vector<JointSlice>& slices = jc.slices;
    if (recs.empty())
        return true; // no unlocked DOF on any queried joint -- nothing to emit
    const uint32_t numOut = static_cast<uint32_t>(recs.size());

    // 3a-bis. Canonicalise every joint key once. Interning costs a lock plus a cache lookup per
    // prim, and the group covers the same prims on every read, so this is derived with the records
    // above and cached with them.
    if (!recsCached)
    {
        jc.sliceKeys.reserve(slices.size());
        for (const JointSlice& sl : slices)
            jc.sliceKeys.push_back(sl.key);
        omni::physics::ovstage::canonicalisePathHandles(source, jc.sliceKeys.data(), jc.sliceKeys.size(),
                                                        jc.sliceHandles, nullptr);
        // Last, deliberately: generation is what claims the entry, so a build that bailed part-way
        // -- including the empty-record early-out above -- leaves it retired rather than partial.
        jc.generation = viewGeneration;
        // A fresh content version for the rebuilt records, so the device upload re-uploads even when
        // the generation did not move (an epoch change under a same-shape view).
        jc.recsVersion = mintRecordContentVersion();
    }
    return !jc.recs.empty();
}

bool getActiveContext(const char* who, ActiveContext& out)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    out.stage = usdLoad ? usdLoad->getActiveAttachedStage() : nullptr;
    if (!out.stage)
    {
        CARB_LOG_ERROR("%s: no attached simulation -- attach an ovstage source first.", who);
        return false;
    }
    out.source = out.stage->getSource();
    if (!out.source)
    {
        CARB_LOG_ERROR("%s: attached stage has no physics source.", who);
        return false;
    }
    return true;
}

// Marks every actor `scene` reports as moved. Shared by the per-scene and union collectors below so
// the decode of PxActor::userData -> record index lives in exactly one place.
//
// THIS LOOP IS WHAT AN ACTIVE-SCOPE READ COSTS, and the cost is the DEREFERENCE, not the marking:
// every iteration chases a pointer into a PxActor that nothing else in the read has touched, purely
// to read one word out of it, so the per-actor cost is a random-access memory latency and grows with
// the working set. On a large scene that makes the scope itself more expensive than the device read
// it is scoping. No benchmark lane tracks it: every OutputRead lane reads at OVPHYSX_SCOPE_ALL --
// queryScope() is virtual but never overridden -- so none of them enters this loop at all, and the
// readonly_rb lanes time the read the scope wraps rather than the scoping. Measuring it needs an
// ACTIVE-scope lane, which does not exist yet. Numbers stay out of this comment either way: they
// would age with hardware and driver.
//
// Two things the cost is NOT, so nobody re-derives them:
//   - Not the SET's representation. This was an unordered_set keyed by record index, and the bitset
//     that replaced it is faster to build for the reasons above, but it did not move this loop --
//     the dereference dominates either way.
//   - Not duplicated work. LazyRigidScan and LazyLinkScan already memoise the set, and the read
//     dispatch is a switch on type, so one read resolves it once: six CALL SITES, one execution.
//
// What would actually help is not doing the dereference: when `count` equals the number of actors
// the scope could cover, every one of them moved and `isActive` can answer true without consulting
// anything. That needs the per-scene actor count to be established safely for a scene that also
// holds articulation links, which is why it is not done here.
void markActiveActorsOfScene(InternalScene* internalScene, ActiveActorSet& out)
{
    PxScene* const scene = internalScene ? internalScene->getScene() : nullptr;
    if (!scene)
        return;

    forEachLiveActiveActor(internalScene,
                           [&out](const PxActor* actor)
                           {
                               // userData is the record index.
                               out.mark(reinterpret_cast<size_t>(actor->userData));
                           });
    out.addReportingScene(scene);
}

// One scene's active set. The flag is per scene, so this is the honest unit: a scene without
// eENABLE_ACTIVE_ACTORS is absent from `reportingScenes` rather than contributing an empty set that
// a caller could mistake for "nothing moved".
ActiveActorSet collectActiveActorsForScene(PxScene* scene)
{
    ActiveActorSet out;
    if (!scene || !(scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS))
        return out;

    // The owning InternalScene, because the released-actor predicate lives there and the list
    // cannot be walked safely without it.
    InternalScene* const internalScene = internalSceneOf(scene);
    if (!internalScene)
        return out; // no owner to ask: report nothing rather than dereference the list

    out.sizeTo(OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords().size());
    markActiveActorsOfScene(internalScene, out);
    return out;
}

// The union across every scene, for the body and link reads, which walk the whole record table
// rather than one scene's actors. The union is over the BITS only; `reportingScenes` still records
// which scenes contributed, so a call site asks sceneReports(owner) and an actor in a non-reporting
// scene falls back to its own sleep state. Building one union rather than a set per scene keeps
// this O(records) instead of O(scenes x records).
ActiveActorSet collectActiveActors()
{
    ActiveActorSet out;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const size_t recordCount = db.getRecords().size();
    // Marked straight into ONE set rather than built per scene and merged: over a bitset every
    // scene marks into the same bits, where merging would be a second full insert pass over a set
    // that had just been built.
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTScene || !rec.mInternalPtr)
            continue;
        InternalScene* const internalScene = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
        PxScene* const scene = internalScene->getScene();
        if (!scene || !(scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS))
            continue; // this scene cannot report; sceneReports() will send its actors to the fallback
        if (!out.anySceneReports())
        {
            // Sized on the first REPORTING scene, so a stage where no scene can report returns an
            // empty set rather than allocating one nobody will read.
            out.sizeTo(recordCount);
        }
        markActiveActorsOfScene(internalScene, out);
    }
    return out;
}

// The InternalScene that owns `scene`. Needed because the released-actor predicate that makes
// getActiveActors() safe to walk lives on InternalScene, and the per-scene entry point is handed
// only a PxScene*. Linear over the record table, but only over the SCENE records, and a stage holds
// a handful -- this runs once per ACTIVE read, not per actor.
InternalScene* internalSceneOf(const PxScene* scene)
{
    if (!scene)
        return nullptr;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTScene || !rec.mInternalPtr)
            continue;
        InternalScene* const candidate = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
        if (candidate->getScene() == scene)
            return candidate;
    }
    return nullptr;
}

// Whether an articulation link is in scope. Shared by the READ (scanLinkRecords) and by query
// DISCOVERY (collectMatchedKeys) so the two cannot disagree: were discovery to enumerate every link
// regardless of scope, a partially sleeping articulation would report all its links to the query and
// then emit only the awake ones, and a caller sizing off the query would see fewer rows than prims.
bool linkInScope(const PxArticulationLink& link,
                 size_t recordIndex,
                 uint32_t scope,
                 const ActiveActorSet& activeSet)
{
    if (scope != omni::physx::kOvxActive)
        return true;
    // A link whose articulation is not in a scene is not active -- and isSleeping() below FATAL-errors on
    // one that is not, so the scene guard has to come first (a change can be drained onto an articulation
    // mid-setup, before it is added to a scene).
    if (!link.getScene())
        return false;
    if (activeSet.sceneReports(link.getScene()))
        return activeSet.isActive(recordIndex);
    return !link.getArticulation().isSleeping();
}

// One walk, bucketed by owning scene. getScene() is called here, where the actor was just dereferenced for
// the type check and is still cache-resident.
void scanRigidRecords(uint32_t scope, const ActiveActorSet& activeSet, RigidRecordScan& out)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const std::vector<InternalDatabase::Record>& records = db.getRecords();
    // Scene count is tiny (one per PhysicsScene prim), so a linear probe beats hashing per record.
    RigidSceneBucket* last = nullptr;
    for (size_t ri = 0; ri < records.size(); ++ri)
    {
        const InternalDatabase::Record& rec = records[ri];
        if (rec.mType != ePTActor || !rec.mPtr || !rec.mInternalPtr)
            continue;
        PxRigidDynamic* dyn = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxRigidDynamic>();
        if (!dyn)
            continue;
        InternalActor* ia = reinterpret_cast<InternalActor*>(rec.mInternalPtr);
        if (ia && ia->mInstanceIndex != kInvalidUint32_t)
        {
            // Reserved on first use, as the standalone buckets below are: an instancer-heavy stage is mostly
            // instances, so an unreserved vector reallocates its way up to the instance count every read.
            if (out.instanced.empty())
                out.instanced.reserve(records.size());
            out.instanced.push_back(dyn);
            continue;
        }
        if (scope == kOvxActive)
        {
            const bool active = activeSet.sceneReports(dyn->getScene())
                ? activeSet.isActive(ri)
                : !((dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) || dyn->isSleeping());
            if (!active)
                continue;
        }
        PxScene* owning = dyn->getScene();
        if (!owning)
            continue; // not in a scene: no view to read it through
        // Records of one scene arrive in runs, so the previous bucket almost always hits.
        if (!last || last->scene != owning)
        {
            last = nullptr;
            for (RigidSceneBucket& b : out.byScene)
            {
                if (b.scene == owning)
                {
                    last = &b;
                    break;
                }
            }
            if (!last)
            {
                // Sized for the single-scene case, which is the one that has to be fast; a second
                // scene grows its own bucket from empty rather than over-reserving every bucket.
                const bool first = out.byScene.empty();
                out.byScene.push_back(RigidSceneBucket{ owning, {}, {} });
                last = &out.byScene.back(); // refreshed after every push_back, so never dangles
                if (first)
                {
                    last->bodies.reserve(records.size());
                    last->keys.reserve(records.size());
                }
            }
        }
        last->bodies.push_back(dyn);
        last->keys.push_back(rec.mKey);
    }
}

// One walk, bucketed by owning scene -- the link counterpart of scanRigidRecords, and for the same reason:
// emission is per-scene, so a walk per scene would repeat the same per-record dereference and type check for
// every scene on the stage.
void scanLinkRecords(uint32_t scope, const ActiveActorSet& activeSet, RigidRecordScan& out)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const std::vector<InternalDatabase::Record>& records = db.getRecords();
    // Scene count is tiny (one per PhysicsScene prim), so a linear probe beats hashing per record.
    RigidSceneBucket* last = nullptr;
    for (size_t ri = 0; ri < records.size(); ++ri)
    {
        const InternalDatabase::Record& rec = records[ri];
        if (rec.mType != ePTLink || !rec.mPtr)
            continue;
        PxArticulationLink* link = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxArticulationLink>();
        if (!link)
            continue;
        if (!linkInScope(*link, ri, scope, activeSet))
            continue;
        // Called here, where the link was just dereferenced for the type check and is still
        // cache-resident, rather than in a per-scene filtering pass that pays the indirection again.
        PxScene* owning = link->getScene();
        if (!owning)
            continue; // not in a scene: no view to read it through
        // Records of one scene arrive in runs, so the previous bucket almost always hits.
        if (!last || last->scene != owning)
        {
            last = nullptr;
            for (RigidSceneBucket& b : out.byScene)
            {
                if (b.scene == owning)
                {
                    last = &b;
                    break;
                }
            }
            if (!last)
            {
                // Sized for the single-scene case, which is the one that has to be fast; a second
                // scene grows its own bucket from empty rather than over-reserving every bucket.
                const bool first = out.byScene.empty();
                out.byScene.push_back(RigidSceneBucket{ owning, {}, {} });
                last = &out.byScene.back(); // refreshed after every push_back, so never dangles
                if (first)
                {
                    last->bodies.reserve(records.size());
                    last->keys.reserve(records.size());
                }
            }
        }
        last->bodies.push_back(link);
        last->keys.push_back(rec.mKey);
    }
}

// The scan already grouped by scene; this is a lookup, not a filter.
const RigidSceneBucket* bucketForScene(const RigidRecordScan& scan, const PxScene* scene)
{
    for (const RigidSceneBucket& b : scan.byScene)
        if (b.scene == scene)
            return &b;
    return nullptr;
}


std::vector<ScenePair> allPhysicsScenesWithInternal()
{
    // Cached against the database's object-lifetime epoch, because the walk is O(records) and the
    // answer is O(scenes): at 8192 envs it scanned ~164k records to return one pointer, measured at
    // ~0.33 ms -- the single largest item inside a warm write session, about a third of it.
    //
    // Applied to the PRIMITIVE rather than to a projection, so every consumer of a scene list gets
    // it: the read's rigid, vehicle and tendon paths, and the write's planning.
    //
    // The epoch is the right guard, and only because of how a scene comes to exist: InternalScene is
    // constructed WITH its PxScene (PhysXScene.cpp), so a record can never be present and later
    // start reporting a scene. Appearing therefore requires a record add and disappearing a record
    // removal, both of which bump the epoch. If that construction order ever changes this cache goes
    // stale silently -- a read or write would cover fewer scenes than it should -- so the assumption
    // is stated here rather than left implicit.
    static std::vector<ScenePair> sCached;
    static uint64_t sEpoch = 0;
    static bool sValid = false;
    const uint64_t epoch = omni::physx::internal::recordLifetimeEpoch();
    if (sValid && epoch == sEpoch)
        return sCached;

    std::vector<ScenePair> scenes;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const InternalDatabase::Record& rec : db.getRecords())
        if (rec.mType == ePTScene && rec.mInternalPtr)
        {
            InternalScene* is = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
            if (PxScene* sc = is->getScene())
                scenes.push_back({ sc, is });
        }
    sCached = scenes;
    sEpoch = epoch;
    sValid = true;
    return scenes;
}

std::vector<PxScene*> scenesOf(const std::vector<ScenePair>& pairs)
{
    std::vector<PxScene*> scenes;
    scenes.reserve(pairs.size());
    for (const ScenePair& sp : pairs)
        scenes.push_back(sp.first);
    return scenes;
}

std::vector<PxScene*> allPhysicsScenes()
{
    return scenesOf(allPhysicsScenesWithInternal());
}

EnabledRigidBodies::EnabledRigidBodies(const std::vector<PxRigidBody*>& bodies,
                                       const std::vector<ObjectKey>& keys,
                                       const bool apply)
    : mBodies(&bodies), mKeys(&keys), mAnyDisabled(apply && anyDisabledRigidDynamic(bodies))
{
    if (!mAnyDisabled)
        return; // the steady state: alias the caller's lists, copy nothing

    mEnabledBodies.reserve(bodies.size());
    mEnabledKeys.reserve(keys.size());
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (isDisabledRigidDynamic(bodies[i]))
            continue;
        mEnabledBodies.push_back(bodies[i]);
        mEnabledKeys.push_back(keys[i]);
    }
    mBodies = &mEnabledBodies;
    mKeys = &mEnabledKeys;
}

} // namespace omni::physx::ovx
