// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/physics/usd/PrimIterator.h>
#include <omni/physics/parse/Handles.h>

#include <private/omni/physx/PhysxUsd.h>
#include "ChangeParams.h"

#include <unordered_set>

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
    OnPrimRequirementCheckFn onPrimCheck;
    OnPrimRequirementCheckExtFn onPrimCheckExt;
    OnUpdateObjectFn onUpdate;
    OnPrimRequirementKeyCheckFn onPrimCheckKey;
};


using PropertyChangeMap = std::unordered_multimap<PXR_NS::TfToken, PropertyChange, PXR_NS::TfToken::HashFunctor>;
using ChangeData = std::pair<OnUpdateObjectFn, PXR_NS::TfToken>;
using ChangeMap = std::unordered_multimap<const PXR_NS::SdfPath, ChangeData, PXR_NS::SdfPath::Hash>; // could get slow with many changes: multimap means many allocations
using PrimSet = std::unordered_map<PXR_NS::SdfPath, const PXR_NS::UsdPrim*, PXR_NS::SdfPath::Hash>;
using PrimKeySet = std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;

class PrimUpdateMap
{
public:
    PrimUpdateMap() : m_isNewScene(false)
    {
    }

    void addPrim(const AttachedStage& attachedStage, const PXR_NS::SdfPath& primPath);

    void removePrim(AttachedStage& attachedStage, const PXR_NS::SdfPath&);

    void clearMap()
    {
        m_primAddMap.clear();
    }

    void checkMap(const AttachedStage& attachedStage);

    bool isInPrimAddMap(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key) const;

    // Set of subtree-root paths to re-parse. Stores paths (no UsdPrim); the prims
    // are resolved at the USD-scan boundary (PrimIteratorMapRange ctor).
    const std::set<PXR_NS::SdfPath>& getMap() const
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

    bool needsSceneReset(const omni::physics::parse::IPhysicsSource& source, omni::physics::parse::ObjectKey key);

private:
    bool m_isNewScene;
    std::set<PXR_NS::SdfPath> m_primAddMap;
};

class PrimChangeMap
{
public:
    PrimChangeMap();
    ~PrimChangeMap();

    void clearMap();

    const ChangeMap& getMap() const
    {
        return m_changeMap;
    }

    void removePrim(const PXR_NS::SdfPath& primKey);

    void registerPrimChange(const ChangeParams& changeParams);
    void clearRegisteredChanges();

    bool getPropertyChange(const PXR_NS::TfToken& token,
                           PropertyChangeMap::const_iterator& iterator,
                           PropertyChangeMap::const_iterator& itEnd) const;

    void checkPrimChange(AttachedStage& attachedStage,
                         const PXR_NS::SdfPath& primKey,
                         const PXR_NS::TfToken& propertyName,
                         const PXR_NS::UsdPrim* prim = nullptr);

    void checkPrimChange(AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey primKey,
                         const PXR_NS::TfToken& propertyName);

    void handleTransformChange(AttachedStage& attachedStage,
                               const PXR_NS::SdfPath& primKey,
                               const PXR_NS::UsdPrim* prim);
    void handleTransformChange(AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey primKey);

    void processTransformUpdates(AttachedStage& attachedStage);

    void addTransformChange(const PXR_NS::SdfPath& path, const PXR_NS::UsdPrim* prim)
    {
        m_usdTransformChangesSet[path] = prim;
    }

    void addTransformChange(omni::physics::parse::ObjectKey key)
    {
        m_keyTransformChangesSet.insert(key);
    }

    void processTransformChanges(AttachedStage& attachedStage);

    void registerStageSpecificChange(const ChangeParams& changeParam);
    void clearStageSpecificChanges();

    const PropertyChangeMap& getPropertyChangeMap() const
    {
        return m_propertyChanges;
    }

    const PropertyChangeMap& getStageSpecificChangeMap() const
    {
        return m_stageSpecificChanges;
    }

private:
    ChangeMap m_changeMap;
    PropertyChangeMap m_propertyChanges; // persistent for all PhysX stages
    PropertyChangeMap m_stageSpecificChanges; // specific to a given stage
    PXR_NS::SdfPathVector m_transformUpdates;
    PrimSet m_usdTransformChangesSet;
    PrimKeySet m_keyTransformChangesSet;
};

// Change-feed consumer callbacks (ADR-0003). These replace
// the old global UsdNotice::ObjectsChanged handler: AttachedStage registers them
// on the IChangeFeed vended by its source. `onSourceChange` is the per-batch
// OnChangeFn (one wildcard interest); `onSourceGroupComplete` is the per-group
// finalization that flushes accumulated transform changes once at end-of-notice.
void onSourceChange(AttachedStage& attachedStage, const omni::physics::parse::ChangeBatch& batch);
void onSourceGroupComplete(AttachedStage& attachedStage);

void processUpdates(AttachedStage& attachedStage, float currentTime);
void flushBufferedChanges(AttachedStage& attachedStage, float currentTime);
void processChangeMap(AttachedStage& attachedStage);

} // namespace usdparser
} // namespace physx
} // namespace omni
