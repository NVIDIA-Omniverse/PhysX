// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-12
 *
 * @implements REQ-COOK-SOURCE-001
 * @covers AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 *
 * @implements REQ-SIM-SCENEQUERY-001
 * @covers AC-3
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-1 AC-3 AC-6
 *
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-7
 */

#include "PrimUpdate.h"
#include "AttachedStage.h"
#include "LoadTools.h"
#include "LoadUsd.h"
#include "Mass.h"

// Everything USD-specific about AttachedStage -- the source rebuild over a live UsdStage,
// the SdfPath/TfToken resolution accessors, and the backing-stage authoring fallbacks --
// lives in the pxr-free usdBridge/AttachedStageBridge.cpp (ADR-0027) and reaches USD only
// through the installed seams.
#include <CookingDataAsync.h>

#include <omni/physics/parse/IParseBackend.h>
#include <OvstageSource.h>

#include <OmniPhysX.h>

#include <carb/InterfaceUtils.h>
#include <common/utilities/OmniPhysXUtilities.h>

#include <unordered_set>
#include <algorithm>


namespace omni
{
namespace physx
{
namespace usdparser
{

ChangeSourceBlock::ChangeSourceBlock(AttachedStage& attachedStage, ChangeSource source) : mAttachedStage(attachedStage)
{
    mPrevSource = mAttachedStage.getChangeSource();
    mAttachedStage.setChangeSource(source);
}

ChangeSourceBlock::~ChangeSourceBlock()
{
    mAttachedStage.setChangeSource(mPrevSource);
}

AttachedStage::AttachedStage(AttachedStageUsdHandle stage, PhysXUsdPhysicsInterface* iface)
    : mPhysicsInterface(iface),
      mStage(stage),
      mObjectDatabase(nullptr),
      mReplicatorStage(false),
      mUseReplicatorEnvIds(false),
      mEnvIdCounter(0),
      mReplicatorEnvIdBase(0)
{
    initUsdStageBinding(stage);
    mObjectDatabase = new ObjectDb();
    // ADR-0019 decision-2 primitive (canonicalKey(getParent(...))), same body
    // as isAncestorOrSelf() below -- powers ObjectDb::removeEntries(ObjectKey).
    mObjectDatabase->setParentResolver(
        [this](omni::physics::parse::ObjectKey key) -> omni::physics::parse::ObjectKey
        {
            const omni::physics::parse::IPhysicsSource* src = getSource();
            return src ? src->canonicalKey(src->getParent(key)) : omni::physics::parse::ObjectKey{};
        });
    initPrimHierarchyStorage(*mObjectDatabase);
    mPhysXDefaultSim = omni::physx::isPhysXDefaultSimulator();

    // Source-independent, so it belongs in the common ctor, not in a USD-only sliver:
    // registerPrimChange only stages the ChangeParams; interning happens per source in
    // rebuildSource()/initUsdChangeRegistrations(). Staging it only on the USD path left
    // mPrimChangeMap empty on an ovstage attach, so no live property update (physics:velocity
    // and every sibling) ever reached PhysX on an ovstage attach.
    if (mPhysicsInterface)
    {
        std::vector<ChangeParams> changesToRegister;
        changesToRegister.reserve(1024);
        mPhysicsInterface->fillChangeParams(changesToRegister);
        for (const ChangeParams& change : changesToRegister)
            mPrimChangeMap.registerPrimChange(change);
    }

    initUsdChangeRegistrations();
}

AttachedStage::AttachedStage()
    : mPhysicsInterface(nullptr),
      mStage(),
      mObjectDatabase(nullptr),
      mReplicatorStage(false),
      mUseReplicatorEnvIds(false),
      mEnvIdCounter(0),
      mReplicatorEnvIdBase(0)
{
    mObjectDatabase = new ObjectDb();
    // ADR-0019 decision-2 primitive (canonicalKey(getParent(...))), same body
    // as isAncestorOrSelf() below -- powers ObjectDb::removeEntries(ObjectKey).
    mObjectDatabase->setParentResolver(
        [this](omni::physics::parse::ObjectKey key) -> omni::physics::parse::ObjectKey
        {
            const omni::physics::parse::IPhysicsSource* src = getSource();
            return src ? src->canonicalKey(src->getParent(key)) : omni::physics::parse::ObjectKey{};
        });
    initPrimHierarchyStorage(*mObjectDatabase);
    mPhysXDefaultSim = true;
}

// The prim-hierarchy storage builds its links from the paths it is handed; the predicate is
// what keeps absent prims out of it. An external (ovstage) source publishes only live
// objects, so there it admits everything. Evaluated per addPrim, so it follows whichever
// source this attach ends up with -- nothing about it is USD-specific, and keyFor() is the
// pxr-free spelling both build arms share.
void AttachedStage::initPrimHierarchyStorage(ObjectDb& db)
{
    db.getPrimHierarchyStorage().init([this](const std::string& path)
                                      { return !mSource || hasExternalSource() ||
                                               mSource->exists(keyFor(std::string_view(path))); });
}

AttachedStage::~AttachedStage()
{
    freeReplicatorMemory();
    delete mObjectDatabase;
}

void AttachedStage::rebuildSource()
{
    // The cooked-geometry carrier (ADR-0022) is keyed by this source's ObjectKey
    // and TokenId vocabulary, so it cannot survive the source being replaced.
    // Cleared before any branch below, which also covers the teardown branch
    // where mDataWrite is reset: a detach/reattach never inherits cooked scratch.
    mCookedGeometry.clear();
    // Holds a raw pointer into mSource, which every branch below replaces.
    mBackingStageDataWrite.reset();

    // Consumer-provided external source (e.g. ovstage). Early-return so the USD
    // path below stays byte-identical. The *active* parse backend builds the
    // source from the opaque AttachTarget payload — nothing here names a
    // concrete backend.
    if (mExternalAttachPayload)
    {
        omni::physics::parse::SourceBundle bundle;
        if (omni::physics::parse::IParseBackend* backend = omni::physics::parse::parseBackend())
            bundle = backend->createSource(attachTarget());
        mSource = std::move(bundle.source);
        mDataWrite = std::move(bundle.write);
        mChangeFeed = std::move(bundle.changeFeed);
        mUnits = mSource ? mSource->getSourceUnits() : omni::physics::parse::SourceUnits{};
        bindKnownTokens();
        if (mSource)
        {
            // Re-intern the persistent property-change dispatch table (PrimUpdate.h's
            // PropertyChangeMap) for the new source -- mirrors the KnownTokens rebind
            // just above: a TokenId minted for the old source would silently alias an
            // unrelated attribute in the new one.
            mPrimChangeMap.internRegisteredChanges(*mSource);
        }

        // The external backend wires no write sink of its own, but a resident backing USD
        // stage is still authorable -- getAuthoringDataWrite()'s fallback, same rule as
        // createDefaultPhysicsScenePlaceholder.
        mBackingStageDataWrite = makeBackingStageDataWrite();

        // Change feed (ADR-0003 M3): the ovstage feed is pull-based — its deltas
        // are drained over an explicit ordinal range via updateFromOvStage. Register
        // the same wildcard interest + group-complete the USD branch uses so a drain
        // drives onSourceChange / onSourceGroupComplete (cooking observes the USD
        // feed only for now).
        if (mChangeFeed)
        {
            AttachedStage* self = this;
            mChangeFeed->registerInterest(omni::physics::parse::ObjectKey{}, omni::physics::parse::TokenId{}, -1,
                                          [self](const omni::physics::parse::ChangeBatch& batch)
                                          { return onSourceChange(*self, batch); },
                                          0);
            // Pull feeds need an explicit attribute read list. Seed it with the
            // PhysX change-map properties using null callbacks; the wildcard
            // registration above remains the only consumer callback.
            if (mSource && mPhysicsInterface)
            {
                std::vector<ChangeParams> changesToRead;
                changesToRead.reserve(1024);
                mPhysicsInterface->fillChangeParams(changesToRead);
                for (const ChangeParams& change : changesToRead)
                {
                    if (!change.changeAttribute.empty())
                    {
                        mChangeFeed->registerInterest(omni::physics::parse::ObjectKey{},
                                                      mSource->internToken(change.changeAttribute), -1,
                                                      nullptr, 0);
                    }
                }
            }
            mChangeFeed->registerGroupComplete([self]() { onSourceGroupComplete(*self); });
        }
        return;
    }

    // Reset the source whenever the stage changes; it interns paths/tokens
    // bound to the live stage. False when there is no live stage, which falls through
    // to the no-source teardown below.
    if (rebuildUsdSource())
        return;

    {
        mChangeFeed.reset();
        mSource.reset();
        mDataWrite.reset();
        mUnits = omni::physics::parse::SourceUnits{};
        bindKnownTokens(); // no source: drops the reference into the destroyed one
        // No source to intern TokenIds against; drop the interned dispatch table
        // (mirrors the KnownTokens reset above) rather than leave stale entries.
        mPrimChangeMap.clearRegisteredChanges();
    }
}

void AttachedStage::bindKnownTokens()
{
    mKnownTokens = omni::physics::parse::KnownTokens{};
    mKnownTokensRef = nullptr;
    if (!mSource)
        return;
    if (const omni::physics::parse::KnownTokens* cached = mSource->knownTokens())
        mKnownTokensRef = cached; // the source's one batch, no copy (REQ-LOAD-TOKENS-001 AC-5)
    else
    {
        mKnownTokens.intern(*mSource);
        mKnownTokensRef = &mKnownTokens;
    }
}

void AttachedStage::setOvstageSource(const void* attachPayload,
                                     AttachedStageUsdHandle backingStage,
                                     uint64_t readOrdinal,
                                     uint64_t backingStageId)
{
    // Switch to a consumer-provided external source and rebuild the source trio
    // through the active (e.g. ovstage) parse backend. `attachTarget()` returns
    // the payload first, so rebuildSource() takes the external branch even though
    // mStage is set. Runtime parsing and queries continue to go through the
    // source/scanned-stage path; the backing stage remains a USD attachment detail.
    mStage = backingStage;
    // Caller-classified input, not a cached answer: it seeds the backend so the
    // source it builds can publish the id (see the header for why the caller,
    // not this class, does the classification).
    mExternalBackingStageId = backingStageId;
    mExternalAttachPayload = attachPayload;
    // Caller-owned sealed read ordinal for the initial parse; updateFromOvStage
    // advances it thereafter. Flows to the backends via AttachTarget::readOrdinal.
    mExternalReadOrdinal = readOrdinal;

    // No predicate install here: init() only stores the predicate, and every ObjectDb this
    // class owns already got it from initPrimHierarchyStorage() at construction (or at
    // replacement, in releasePhysicsObjects).

    rebuildSource();
}

omni::physics::parse::AttachTarget AttachedStage::attachTarget() const
{
    omni::physics::parse::AttachTarget target;
    if (mExternalAttachPayload)
    {
        // Consumer-provided payload (e.g. const OvstageAttach*). The separately
        // classified local backing id rides along for source compatibility
        // fallbacks, and is what the external source publishes as its resident
        // USD stage id.
        target.nativeStage = mExternalAttachPayload;
        target.readOrdinal = mExternalReadOrdinal;
        target.residentBackingStageId = mExternalBackingStageId;
        // The scan backend reads through the live source (warm across drains); the parse
        // backend ignores it (rebuildSource runs while this is still the old source).
        target.attachedSource = mSource.get();
    }
    else
    {
        // USD attach: the live stage handle (matches rebuildSource's USD path).
        // No backing id -- the USD backend computes it from the stage.
        fillUsdAttachTarget(target);
    }
    return target;
}

omni::physics::parse::IPhysicsSource* AttachedStage::getSource()
{
    return mSource.get();
}

const omni::physics::parse::IPhysicsSource* AttachedStage::getSource() const
{
    return mSource.get();
}

// keyFor(std::string_view) is defined in usdBridge/AttachedStageBridge.cpp and resolves through
// IPhysicsSource::findByPath.
bool AttachedStage::createDefaultPhysicsScenePlaceholder(omni::physics::parse::ObjectKey sceneKey)
{
    if (mDataWrite)
        return mDataWrite->createDefaultPhysicsScene(sceneKey);
    // No write sink for the active source (ovstage, by design) but a real backing stage may
    // still be resident: author straight into it. Matches the pre-refactor "is there a stage
    // to author into" gate, before createDefaultPhysicsScene moved behind IPhysicsDataWrite.
    return createDefaultPhysicsSceneOnStage(sceneKey);
}

void AttachedStage::removeDefaultPhysicsScenePlaceholder(omni::physics::parse::ObjectKey sceneKey)
{
    if (mDataWrite)
    {
        mDataWrite->removeDefaultPhysicsScene(sceneKey);
        return;
    }
    removeDefaultPhysicsSceneOnStage(sceneKey);
}

const char* AttachedStage::textFor(omni::physics::parse::ObjectKey key) const
{
    return textViewFor(key).data();
}

std::string_view AttachedStage::textViewFor(omni::physics::parse::ObjectKey key) const
{
    // sourceKeyToString is part of the backend-neutral contract; no down-cast.
    if (mSource)
    {
        // Backed by the source's interned std::string storage, so the data is
        // null-terminated and outlives the call.
        const std::string_view sv = mSource->sourceKeyToString(key);
        if (!sv.empty())
            return sv;
    }
    return std::string_view("", 0);
}

void AttachedStage::releasePhysicsObjects(bool rebuildObjectDatabase)
{
    std::unique_ptr<ObjectDb> replacementObjectDatabase;
    if (rebuildObjectDatabase)
    {
        replacementObjectDatabase = std::make_unique<ObjectDb>();
        replacementObjectDatabase->setParentResolver(
            [this](omni::physics::parse::ObjectKey key) -> omni::physics::parse::ObjectKey
            {
                const omni::physics::parse::IPhysicsSource* src = getSource();
                return src ? src->canonicalKey(src->getParent(key)) : omni::physics::parse::ObjectKey{};
            });
        // A fresh ObjectDb carries no predicate, so this one genuinely needs the install.
        initPrimHierarchyStorage(*replacementObjectDatabase);
    }

    // This is the only teardown that drains the interface's pending
    // mArticulations/mParticleSystems, which finishSetup()
    // otherwise consumes on the next attach's first step -- as indices into the
    // records of the attach being destroyed here (REQ-CAPI-DETACH-002).
    mPhysicsInterface->releaseAllObjects();

    delete mObjectDatabase;
    mObjectDatabase = replacementObjectDatabase.release();

    mPrimUpdateMap.setEmptyScene(true);

    mPrimUpdateMap.clearMap();
    mPrimChangeMap.clearMap();
    mPrimChangeMap.clearStageSpecificChanges();

    // Clear regardless of backend so a stale registration cannot survive an attach reset.
    mTimeSampledAttributes.clear();
    mAnimatedKinematicBodies.clear();

    mCollisionGroupsMap.clear();
    mAdditionalCollisionGroupMaps.clear();
    mDeformableAttachmentHistoryMap.clear();
    mDeformableCollisionFilterHistoryMap.clear();
    clearGeneratedDeformableAttachmentData();
    // Same lifetime as the generated attachment data above: the objects that
    // consume the cooked scratch are gone, so the scratch goes with them
    // (ADR-0022).
    clearCookedGeometry();

    freeReplicatorMemory();

    mEnvIdCounter = 0;
    mReplicatorEnvIdBase = 0;
    mRuntimeCloneTargets.clear();
}

void AttachedStage::registerStageSpecificAttribute(ChangeParams& changeParam)
{
    // Intern here (not in PrimChangeMap): this is always called mid-parse, well
    // after the source is attached, so there is no ordering problem -- unlike
    // registerPrimChange's ctor-time staging (PrimUpdate.cpp's own comment).
    const omni::physics::parse::IPhysicsSource* src = getSource();
    if (src)
        mPrimChangeMap.registerStageSpecificChange(src->internToken(changeParam.changeAttribute), changeParam);
}

void AttachedStage::clearStageSpecificAttributes()
{
    mPrimChangeMap.clearStageSpecificChanges();
}

const usdparser::ObjectIdMap* AttachedStage::getObjectIds(omni::physics::parse::ObjectKey key) const
{
    return mObjectDatabase->getEntries(key);
}

bool AttachedStage::isKeyLive(omni::physics::parse::ObjectKey key) const
{
    // A key is live while any owning registry still contains it. Mirror objectKeyToPath
    // (PhysX.cpp) exactly -- these are the two ObjectKey->path resolvers, and drifting
    // apart is the failure this shared helper exists to prevent.

    // ObjectDb first: it is in-memory, and it is the only index that sees clone-only
    // objects (a PhysX-replicator clone has no authored source object at all).
    const usdparser::ObjectIdMap* entries = getObjectIds(key);
    if (entries && !entries->empty())
        return true;

    // Then the parse source, for authored objects.
    const omni::physics::parse::IPhysicsSource* source = getSource();
    if (source && source->exists(key))
        return true;

    // Then the InternalPhysXDatabase, for runtime-created objects such as D6 joints;
    // a removed record does not keep a stale key live.
    const std::vector<internal::InternalDatabase::Record>& records =
        OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords();
    return std::any_of(records.begin(), records.end(),
                       [key](const internal::InternalDatabase::Record& rec)
                       { return rec.mKey == key && rec.mType != ePTRemoved; });
}

void AttachedStage::registerObjectId(omni::physics::parse::ObjectKey key,
                               const usdparser::ObjectCategory& category,
                               const usdparser::ObjectId& newEntryId)
{
    mObjectDatabase->findOrCreateEntry(key, category, newEntryId);
}


void AttachedStage::updateRigidBodyMass()
{
    omni::physics::parse::IPhysicsSource* source = getSource();
    auto* ovstageSource = dynamic_cast<omni::physics::ovstage::OvstageSource*>(source);
    bool ownLoadCacheWindow = false;
    if (ovstageSource)
    {
        std::vector<omni::physics::parse::ObjectKey> prefetchKeys;
        std::unordered_set<uint64_t> seenKeys;
        auto addPrefetchKey = [&](omni::physics::parse::ObjectKey key)
        {
            if (key.valid() && seenKeys.insert(key.handle).second)
                prefetchKeys.push_back(key);
        };

        for (const omni::physics::parse::ObjectKey bodyKey : mRigidBodyMassUpdateMap)
        {
            addPrefetchKey(bodyKey);

            const ObjectIdMap* entries = getObjectIds(bodyKey);
            if (!entries || entries->empty())
                continue;

            for (const ObjectIdMap::value_type& entry : *entries)
            {
                if (!(entry.first == eBody || entry.first == eArticulationLink))
                    continue;

                ObjectIdPathMap shapes;
                getPhysXPhysicsInterface()->getRigidBodyShapes(*this, entry.second, shapes);
                for (const std::pair<const ObjectId, omni::physics::parse::ObjectKey>& shapePair : shapes)
                {
                    if (shapePair.second.valid())
                        addPrefetchKey(shapePair.second);
                }
            }
        }

        static const std::vector<std::string> kMassAttrs = {
            "physics:mass",
            "physics:density",
            "physics:diagonalInertia",
            "physics:centerOfMass",
            "physics:principalAxes",
            "physics:kinematicEnabled",
            omni::physics::ovstage::conv::kFabricWorldMatrix,
            omni::physics::ovstage::conv::kFabricLocalMatrix,
            omni::physics::ovstage::conv::kLocalTransform,
            omni::physics::ovstage::conv::kResetXformStack,
        };
        // Mass falls back to the bound material's density, resolved up the ancestor chain.
        // Relationships are served only from the load cache, so open one for the prefetch --
        // unless the load already holds one open (loadFromRange's window, warmed by the scan's
        // merged read): joining it turns both prefetches into covered no-ops.
        ownLoadCacheWindow = !ovstageSource->loadCacheActive();
        if (ownLoadCacheWindow)
            ovstageSource->beginLoadCache();
        ovstageSource->prefetchBucket(prefetchKeys, kMassAttrs);
        ovstageSource->prefetchRelationshipAncestors(prefetchKeys);
    }

    // Mass computation reads MassAPI/material/units through the source.
    for (const omni::physics::parse::ObjectKey bodyKey : mRigidBodyMassUpdateMap)
    {
        if (bodyKey.valid())
            RequestRigidBodyMassUpdate(*this, bodyKey);
    }

    if (ovstageSource)
    {
        ovstageSource->clearBucket();
        if (ownLoadCacheWindow)
            ovstageSource->clearLoadCache();
    }

    mRigidBodyMassUpdateMap.clear();
}

}
}
}
