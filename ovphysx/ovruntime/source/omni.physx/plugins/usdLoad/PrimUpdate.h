// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// PropertyChangeMap is TokenId-keyed (ADR-0019), interned per-source by
// PrimChangeMap::internRegisteredChanges rather than at registration time: a TokenId is
// only valid for one IPhysicsSource instance's lifetime, unlike the old TfToken key.
#include <omni/physics/parse/Handles.h>

#include <private/omni/physx/PhysxUsd.h>
#include "ChangeParams.h"

#include <unordered_set>
#include <vector>

namespace omni { namespace physics { namespace parse { struct ChangeBatch; class IPhysicsSource; } } }

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

struct PropertyChange
{
    OnUpdateObjectFn onUpdate;
    OnPrimRequirementKeyCheckFn onPrimCheckKey;
    OnPrimRequirementExtKeyCheckFn onPrimCheckExtKey;
};

// TokenId-keyed (ADR-0019); see PrimChangeMap::internRegisteredChanges for why interning
// is deferred to source-attach time.
using PropertyChangeMap = std::unordered_multimap<omni::physics::parse::TokenId, PropertyChange, omni::physics::parse::TokenId::Hash>;
using ChangeData = std::pair<OnUpdateObjectFn, omni::physics::parse::TokenId>;
// Async-update deferral map, populated by PrimChangeMap::checkPrimChange.
using KeyChangeMap = std::unordered_multimap<omni::physics::parse::ObjectKey, ChangeData, omni::physics::parse::ObjectKey::Hash>;
using PrimKeySet = std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;

class PrimUpdateMap
{
public:
    PrimUpdateMap() : m_isNewScene(false)
    {
    }

    void addPrim(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key);

    void removePrim(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key);

    void clearMap()
    {
        m_primAddMap.clear();
    }

    void checkMap(const AttachedStage& attachedStage);

    bool isInPrimAddMap(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key) const;

    // Set of subtree-roots (ObjectKeys) to re-parse, resolved at the source-scan boundary
    // (loadPhysicsFromPrimitive's updateRoots).
    const PrimKeySet& getMap() const
    {
        return m_primAddMap;
    }

    bool isEmptyScene() const
    {
        return m_isNewScene;
    }
    void setEmptyScene(bool val)
    {
        m_isNewScene = val;
    }

    bool needsSceneReset(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key);

private:
    bool m_isNewScene;
    PrimKeySet m_primAddMap;
};

class PrimChangeMap
{
public:
    PrimChangeMap();
    ~PrimChangeMap();

    void clearMap();

    // Drops any pending m_keyChangeMap entry for this key.
    void removePrim(omni::physics::parse::ObjectKey key);

    // registerPrimChange only stages the ChangeParams (source-independent);
    // internRegisteredChanges does the actual TokenId interning once a source
    // is available. Splitting these two steps is what lets registration run at
    // AttachedStage construction time, before any source exists yet (see
    // AttachedStage::rebuildSource's call to internRegisteredChanges).
    void registerPrimChange(const ChangeParams& changeParams);
    void internRegisteredChanges(const omni::physics::parse::IPhysicsSource& source);
    void clearRegisteredChanges();

    bool getPropertyChange(omni::physics::parse::TokenId token,
                           PropertyChangeMap::const_iterator& iterator,
                           PropertyChangeMap::const_iterator& itEnd) const;

    // The sole dispatch path for every source, including USD (onSourceChange always
    // converts to ObjectKey at the entry point).
    void checkPrimChange(AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey primKey,
                         omni::physics::parse::TokenId propertyTokenId);

    void handleTransformChange(AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey primKey);

    void processTransformUpdates(AttachedStage& attachedStage);

    void addTransformChange(omni::physics::parse::ObjectKey key)
    {
        m_keyTransformChangesSet.insert(key);
    }

    void processTransformChanges(AttachedStage& attachedStage);

    // registerStageSpecificChange interns immediately (the caller,
    // AttachedStage::registerStageSpecificAttribute, always has a live source
    // by the time it runs -- this registration happens mid-parse, long after
    // attach, unlike registerPrimChange's ctor-time staging above).
    void registerStageSpecificChange(omni::physics::parse::TokenId attributeId, const ChangeParams& changeParam);
    void clearStageSpecificChanges();

    const PropertyChangeMap& getPropertyChangeMap() const
    {
        return m_propertyChanges;
    }

    const PropertyChangeMap& getStageSpecificChangeMap() const
    {
        return m_stageSpecificChanges;
    }

    const KeyChangeMap& getKeyMap() const
    {
        return m_keyChangeMap;
    }

private:
    KeyChangeMap m_keyChangeMap;
    // Deferred transform-change queue for the async-update path.
    std::vector<omni::physics::parse::ObjectKey> m_transformKeyUpdates;
    PropertyChangeMap m_propertyChanges; // persistent for all PhysX stages
    PropertyChangeMap m_stageSpecificChanges; // specific to a given stage
    std::vector<ChangeParams> m_registeredChanges; // staged, source-independent (see internRegisteredChanges)
    PrimKeySet m_keyTransformChangesSet;
};

// Change-feed consumer callbacks (ADR-0003). These replace
// the old global UsdNotice::ObjectsChanged handler: AttachedStage registers them
// on the IChangeFeed vended by its source. `onSourceChange` is the per-batch
// OnChangeFn (one wildcard interest); `onSourceGroupComplete` is the per-group
// finalization that flushes accumulated transform changes once at end-of-notice.
//
// `onSourceChange` returns false ONLY when the ovstage drain committed part of a value batch and a
// later scatter failed: the feed's drainRange must then hold the cursor so the batch is retried and
// the external read ordinal does not advance past it. Every other outcome returns true.
bool onSourceChange(AttachedStage& attachedStage, const omni::physics::parse::ChangeBatch& batch);
void onSourceGroupComplete(AttachedStage& attachedStage);

void processUpdates(AttachedStage& attachedStage, float currentTime);
void flushBufferedChanges(AttachedStage& attachedStage, float currentTime);
void processChangeMap(AttachedStage& attachedStage);

} // namespace usdparser
} // namespace physx
} // namespace omni
