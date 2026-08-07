// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-4
 */

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <omni/physics/usd/PrimIterator.h>
#include <omni/physx/IPhysxSettings.h>

#include "PrimUpdate.h"
#include "AttachedStage.h"
#include "LoadTools.h"
#include "LoadUsd.h"
#include "Mass.h"

#include <UsdSource.h>
#include <UsdPhysicsDataWrite.h>
#include <CookingDataAsync.h>

#include <omni/physics/parse/IParseBackend.h>
#include <OvstageSource.h>
#include <omni/physics/usd/UsdParseBackend.h>

#include <OmniPhysX.h>

#include <carb/InterfaceUtils.h>
#include <common/utilities/OmniPhysXUtilities.h>

#include <unordered_set>


using namespace PXR_NS;
using namespace omni::physics::schema;

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

AttachedStage::AttachedStage(PXR_NS::UsdStageWeakPtr stage, PhysXUsdPhysicsInterface* iface)
    : mPhysicsInterface(iface),
      mStage(stage),
      mStageId(0),
      mObjectDatabase(nullptr),
      mReplicatorStage(false),
      mUseReplicatorEnvIds(false),
      mEnvIdCounter(0),
      mReplicatorEnvIdBase(0)
{
    setStage(stage);
    mObjectDatabase = new ObjectDb();
    mObjectDatabase->setKeyResolver([this](const SdfPath& path) { return keyFor(path); });
    mPhysXDefaultSim = omni::physx::isPhysXDefaultSimulator();

    // The prim-hierarchy storage mirrors a USD stage; a non-USD (ovstage) attach
    // has none, so skip it (the source provides hierarchy via IPhysicsSource).
    if (mStage)
        mObjectDatabase->getPrimHierarchyStorage().init(mStage);

    std::vector<ChangeParams> changesToRegister;
    changesToRegister.reserve(1024);
    mPhysicsInterface->fillChangeParams(changesToRegister);
    for (size_t i = 0; i < changesToRegister.size(); i++)
    {
        mPrimChangeMap.registerPrimChange(changesToRegister[i]);
    }
}

AttachedStage::AttachedStage(PXR_NS::UsdStageWeakPtr stage)
    : AttachedStage()
{
    setStage(stage);
    mObjectDatabase = new ObjectDb();
    mObjectDatabase->setKeyResolver([this](const SdfPath& path) { return keyFor(path); });
}

AttachedStage::AttachedStage()
    : mPhysicsInterface(nullptr),
      mStage(nullptr),
      mStageId(0),
      mObjectDatabase(nullptr),
      mReplicatorStage(false),
      mUseReplicatorEnvIds(false),
      mEnvIdCounter(0),
      mReplicatorEnvIdBase(0)
{
    mObjectDatabase = new ObjectDb();
    mObjectDatabase->setKeyResolver([this](const SdfPath& path) { return keyFor(path); });
    mPhysXDefaultSim = true;
}

AttachedStage::~AttachedStage()
{
    freeReplicatorMemory();
    delete mObjectDatabase;
}

void AttachedStage::rebuildSource()
{
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
                                          { onSourceChange(*self, batch); },
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
    // bound to the live stage.
    if (mStage)
    {
        // The active parse backend (ADR-0005) builds the source trio. The USD
        // backend is the default; install it lazily as a safety net for paths
        // that construct an AttachedStage without going through plugin startup.
        if (!omni::physics::parse::parseBackend())
            omni::physics::parse::setParseBackend(omni::physics::usd::makeUsdParseBackend());

        // The USD backend interprets nativeStage as the live UsdStageWeakPtr.
        omni::physics::parse::AttachTarget target;
        target.nativeStage = &mStage;
        target.stageId = mStageId;
        target.residentBackingStageId = static_cast<uint64_t>(mStageId);

        // A USD-stage target can ONLY be parsed by the USD backend. The globally
        // *active* backend may be a data-plane backend (e.g. ovstage) that reads
        // nativeStage as its own opaque payload and would crash on a
        // UsdStageWeakPtr*. A genuine USD attach makes the USD backend active
        // anyway, so this only diverges for a USD re-parse performed under an
        // ovstage attach (e.g. the property-query parseRigidBody, which builds a
        // fresh USD AttachedStage) — and there the USD backend is still correct.
        static std::unique_ptr<omni::physics::parse::IParseBackend> sUsdStageBackend =
            omni::physics::usd::makeUsdParseBackend();
        omni::physics::parse::SourceBundle bundle = sUsdStageBackend->createSource(target);
        mSource = std::move(bundle.source);
        mDataWrite = std::move(bundle.write);
        mChangeFeed = std::move(bundle.changeFeed);
        mUnits = mSource ? mSource->getSourceUnits() : omni::physics::parse::SourceUnits{};

        // Change feed (ADR-0003): vended by the source, drives the incremental
        // update consumer callbacks. A single wildcard interest (invalid
        // objectType + invalid property) receives every batch; the group-complete
        // callback flushes accumulated transform changes once per notice.
        if (mChangeFeed)
        {
            AttachedStage* self = this;
            mChangeFeed->registerInterest(omni::physics::parse::ObjectKey{}, omni::physics::parse::TokenId{}, -1,
                                          [self](const omni::physics::parse::ChangeBatch& batch)
                                          { onSourceChange(*self, batch); },
                                          0);
            mChangeFeed->registerGroupComplete([self]() { onSourceGroupComplete(*self); });

            // The cooking driver also observes this feed for recook scheduling.
            // Registering here (at feed creation) means cooking sees USD edits from
            // attach time, independent of when its pump() first runs; the interest
            // is owned by mChangeFeed and released when the feed is rebuilt.
            if (cookingdataasync::CookingDataAsync* cookingDataAsync =
                    omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync())
            {
                cookingDataAsync->registerOnChangeFeed(*this);
            }
        }
    }
    else
    {
        mChangeFeed.reset();
        mSource.reset();
        mDataWrite.reset();
        mUnits = omni::physics::parse::SourceUnits{};
    }
}

void AttachedStage::setOvstageSource(const void* attachPayload, PXR_NS::UsdStageWeakPtr backingStage, uint64_t readOrdinal)
{
    // Switch to a consumer-provided external source and rebuild the source trio
    // through the active (e.g. ovstage) parse backend. `attachTarget()` returns
    // the payload first, so rebuildSource() takes the external branch even though
    // mStage is set. Runtime parsing and queries continue to go through the
    // source/scanned-stage path; the backing stage remains a USD attachment detail.
    mStage = backingStage;
    mStageId = backingStage ? PXR_NS::UsdUtilsStageCache::Get().GetId(backingStage).ToLongInt() : 0u;
    mExternalAttachPayload = attachPayload;
    // Caller-owned sealed read ordinal for the initial parse; updateFromOvStage
    // advances it thereafter. Flows to the backends via AttachTarget::readOrdinal.
    mExternalReadOrdinal = readOrdinal;

    // Mirror the USD ctor: a backing stage gets prim-hierarchy storage so engine
    // hierarchy lookups work (skipped when there is no backing stage).
    if (mStage && mObjectDatabase)
        mObjectDatabase->getPrimHierarchyStorage().init(mStage);

    rebuildSource();
}

omni::physics::parse::AttachTarget AttachedStage::attachTarget() const
{
    omni::physics::parse::AttachTarget target;
    if (mExternalAttachPayload)
    {
        // Consumer-provided payload (e.g. const OvstageAttach*). stageId remains
        // zero as the external-source sentinel; the separately classified local
        // backing id is available only for source compatibility fallbacks.
        target.nativeStage = mExternalAttachPayload;
        target.stageId = 0;
        target.readOrdinal = mExternalReadOrdinal;
        target.residentBackingStageId = static_cast<uint64_t>(mStageId);
    }
    else
    {
        // USD attach: the live stage handle (matches rebuildSource's USD path).
        target.nativeStage = &mStage;
        target.stageId = mStageId;
        target.residentBackingStageId = static_cast<uint64_t>(mStageId);
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

// SdfPath <-> ObjectKey resolution. The USD backend uses UsdSource's interned
// maps directly; a non-USD backend (ovstage) resolves through the abstract
// IPhysicsSource (path string <-> key), so consumer-side key resolution works
// regardless of backend. Runtime data reads should use IPhysicsSource, or the
// scanned/runtime data already cached by the attach path.
PXR_NS::SdfPath AttachedStage::pathFor(omni::physics::parse::ObjectKey key) const
{
    if (const omni::physics::usd::UsdSource* usd = omni::physics::usd::asUsdSource(mSource.get()))
        return usd->pathFor(key);
    if (mSource)
    {
        const std::string_view s = mSource->sourceKeyToString(key);
        if (!s.empty())
            return PXR_NS::SdfPath(std::string(s));
    }
    return PXR_NS::SdfPath{};
}

omni::physics::parse::ObjectKey AttachedStage::keyFor(const PXR_NS::SdfPath& path) const
{
    if (const omni::physics::usd::UsdSource* usd = omni::physics::usd::asUsdSource(mSource.get()))
        return usd->keyFor(path);
    if (mSource)
        return mSource->findByPath(path.GetString());
    return omni::physics::parse::ObjectKey{};
}

const char* AttachedStage::textFor(omni::physics::parse::ObjectKey key) const
{
    // sourceKeyToString is part of the backend-neutral contract; no down-cast.
    if (mSource)
    {
        // Backed by the source's interned std::string storage, so the data is
        // null-terminated and outlives the call.
        const std::string_view sv = mSource->sourceKeyToString(key);
        if (!sv.empty())
            return sv.data();
    }
    return "";
}

void AttachedStage::releasePhysicsObjects(bool rebuildObjectDatabase)
{
    std::unique_ptr<ObjectDb> replacementObjectDatabase;
    if (rebuildObjectDatabase)
    {
        replacementObjectDatabase = std::make_unique<ObjectDb>();
        replacementObjectDatabase->setKeyResolver([this](const SdfPath& path) { return keyFor(path); });
        replacementObjectDatabase->getPrimHierarchyStorage().init(mStage);
    }

    mPhysicsInterface->releaseAllObjects();

    delete mObjectDatabase;
    mObjectDatabase = replacementObjectDatabase.release();

    mPrimUpdateMap.setEmptyScene(true);

    mPrimUpdateMap.clearMap();
    mPrimChangeMap.clearMap();
    mPrimChangeMap.clearStageSpecificChanges();

    mAnimatedKinematicBodies.clear();
    mTimeSampledAttributes.clear();

    mCollisionGroupsMap.clear();
    mAdditionalCollisionGroupMaps.clear();
    mDeformableAttachmentHistoryMap.clear();
    mDeformableCollisionFilterHistoryMap.clear();
    clearGeneratedDeformableAttachmentData();

    freeReplicatorMemory();

    mTokenEnvIdMap.clear();
    mEnvIdCounter = 0;
    mReplicatorEnvIdBase = 0;
    mRuntimeCloneTargets.clear();
}

void AttachedStage::registerTimeSampledAttribute(const SdfPath& attributePath, usdparser::OnUpdateObjectFn onUpdate)
{
    mTimeSampledAttributes[attributePath] = onUpdate;
}


void AttachedStage::unregisterTimeSampledAttribute(const SdfPath& attributePath)
{
    mTimeSampledAttributes.erase(attributePath);
}

void AttachedStage::registerStageSpecificAttribute(ChangeParams& changeParam)
{
    mPrimChangeMap.registerStageSpecificChange(changeParam);
}

void AttachedStage::clearStageSpecificAttributes()
{
    mPrimChangeMap.clearStageSpecificChanges();
}

const usdparser::ObjectIdMap* AttachedStage::getObjectIds(const PXR_NS::SdfPath& path) const
{
    return mObjectDatabase->getEntries(path);
}

void AttachedStage::registerObjectId(const PXR_NS::SdfPath& path,
                               const usdparser::ObjectCategory& category,
                               const usdparser::ObjectId& newEntryId)
{
    mObjectDatabase->findOrCreateEntry(path, category, newEntryId);
}


void AttachedStage::updateRigidBodyMass()
{
    omni::physics::parse::IPhysicsSource* source = getSource();
    auto* ovstageSource = dynamic_cast<omni::physics::ovstage::OvstageSource*>(source);
    if (ovstageSource)
    {
        std::vector<omni::physics::parse::ObjectKey> prefetchKeys;
        std::unordered_set<uint64_t> seenKeys;
        auto addPrefetchKey = [&](omni::physics::parse::ObjectKey key)
        {
            if (key.valid() && seenKeys.insert(key.handle).second)
                prefetchKeys.push_back(key);
        };

        for (const PXR_NS::SdfPath& path : mRigidBodyMassUpdateMap)
        {
            const omni::physics::parse::ObjectKey bodyKey = keyFor(path);
            addPrefetchKey(bodyKey);

            const ObjectIdMap* entries = getObjectIds(path);
            if (!entries || entries->empty())
                continue;

            for (const ObjectIdMap::value_type& entry : *entries)
            {
                if (!(entry.first == eBody || entry.first == eArticulationLink))
                    continue;

                ObjectIdPathMap shapes;
                getPhysXPhysicsInterface()->getRigidBodyShapes(*this, entry.second, shapes);
                for (const std::pair<ObjectId, PXR_NS::SdfPath>& shapePair : shapes)
                {
                    if (!shapePair.second.IsEmpty())
                        addPrefetchKey(keyFor(shapePair.second));
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
        ovstageSource->prefetchBucket(prefetchKeys, kMassAttrs);
    }

    // The buffer is path-keyed; resolve each to a source ObjectKey (backend-
    // agnostic — no USD prim/stage needed, so this works under ovstage too).
    // Mass computation reads MassAPI/material/units through the source.
    for (const PXR_NS::SdfPath& path : mRigidBodyMassUpdateMap)
    {
        const omni::physics::parse::ObjectKey bodyKey = keyFor(path);
        if (bodyKey.valid())
            RequestRigidBodyMassUpdate(*this, bodyKey);
    }

    if (ovstageSource)
        ovstageSource->clearBucket();

    mRigidBodyMassUpdateMap.clear();
}

}
}
}
