// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physics/schema/IUsdPhysics.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/USDValueAccessors.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/stage/StageReaderWriter.h>
#include <private/omni/physx/PhysxUsd.h>
#include "ChangeParams.h"

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
};


using PropertyChangeMap = std::unordered_multimap<pxr::TfToken, PropertyChange, pxr::TfToken::HashFunctor>;
using ChangeData = std::pair<OnUpdateObjectFn, pxr::TfToken>;
using ChangeMap = std::unordered_multimap<const pxr::SdfPath, ChangeData, pxr::SdfPath::Hash>; // A.B. this could become
                                                                                               // slow if many changes
                                                                                               // would be done, its a
                                                                                               // map a lot of
                                                                                               // allocations might
                                                                                               // happen
using PrimSet = std::unordered_map<pxr::SdfPath, const pxr::UsdPrim*, pxr::SdfPath::Hash>;

class FabricBatchData
{
public:
    FabricBatchData(const omni::fabric::StageReaderWriter& sip,
                    const omni::fabric::ChangedPrimBucketList& chb,
                    size_t chIndex,
                    const omni::fabric::Token& attrName)
        : mStageInProgress(sip),
          mChangesByType(chb),
          mChangesIndex(chIndex),
          mAttrName(attrName),
          mData(nullptr),
          mCurrentIndex(0),
          mDataSize(0)
    {
    }

    template <typename T>
    bool getCurrentData(T& outData)
    {
        if (!mData)
        {
            if constexpr (std::is_same_v<T, pxr::TfToken>)
            {
                const gsl::span<const omni::fabric::TokenC> data = 
                    mStageInProgress.getAttributeArrayRd<omni::fabric::TokenC>(mChangesByType, mChangesIndex, mAttrName);
                mData = reinterpret_cast<const uint8_t*>(data.data());
                mDataSize = (uint32_t)data.size();
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                return false;
            }
            else
            {
                const gsl::span<const T> data =
                    mStageInProgress.getAttributeArrayRd<T>(mChangesByType, mChangesIndex, mAttrName);
                mData = reinterpret_cast<const uint8_t*>(data.data());
                mDataSize = (uint32_t)data.size();
            }
        }

        if (!mData)
            return false;

        const T* data = reinterpret_cast<const T*>(mData);
        if (mCurrentIndex >= mDataSize)
        {
            return false;
        }

        outData = data[mCurrentIndex];
        return true;
    }

    void setCurrentIndex(uint32_t index)
    {
        mCurrentIndex = index;
    }

private:
    const omni::fabric::StageReaderWriter& mStageInProgress;
    const omni::fabric::ChangedPrimBucketList& mChangesByType;
    size_t mChangesIndex;
    const omni::fabric::Token& mAttrName;
    const uint8_t* mData;
    uint32_t mCurrentIndex;
    uint32_t mDataSize;
};

class PrimUpdateMap
{
public:
    PrimUpdateMap() : m_isNewScene(false)
    {
    }

    void addPrim(const AttachedStage& attachedStage, const pxr::UsdPrim&);

    void removePrim(AttachedStage& attachedStage, const pxr::SdfPath&);

    void clearMap()
    {
        m_primAddMap.clear();
    }

    void checkMap(const pxr::UsdStageWeakPtr stage);

    bool isInPrimAddMap(const pxr::UsdPrim&) const;

    const omni::physics::schema::UsdPrimMap& getMap() const
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

    bool needsSceneReset(const pxr::UsdPrim&);

private:
    bool m_isNewScene;
    omni::physics::schema::UsdPrimMap m_primAddMap;
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

    void removePrim(const pxr::SdfPath& primPath);

    void registerPrimChange(const ChangeParams& changeParams);
    void clearRegisteredChanges();

    bool getPropertyChange(const pxr::TfToken& token,
                           PropertyChangeMap::const_iterator& iterator,
                           PropertyChangeMap::const_iterator& itEnd) const;

    void checkPrimChange(AttachedStage& attachedStage,
                         const pxr::SdfPath& primPath,
                         const pxr::TfToken& propertyName,
                         const pxr::UsdPrim* prim = nullptr);

    void handleTransformChange(AttachedStage& attachedStage,
                               const pxr::SdfPath& primPath,
                               const pxr::UsdPrim* prim,
                               pxr::UsdGeomXformCache* xfCache,
                               bool fastCache = false);

    void processTransformUpdates(AttachedStage& attachedStage);

    void addTransformChange(const pxr::SdfPath& path, const pxr::UsdPrim* prim, bool fabric)
    {
        if (fabric)
            m_fabricTransformChangesSet[path] = prim;
        else
            m_usdTransformChangesSet[path] = prim;
    }

    void processTransformChanges(AttachedStage& attachedStage, bool fabric);

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
    pxr::SdfPathVector m_transformUpdates;
    PrimSet m_fabricTransformChangesSet;
    PrimSet m_usdTransformChangesSet;
};

struct UsdNoticeListener : public pxr::TfWeakBase
{
    UsdNoticeListener() = default;

    void Handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged);
    void HandleAttributeValuesChanged(const class omni::fabric::AttributeValuesChangedNotice& valuesChanged);
};

void processUpdates(AttachedStage& attachedStage, float currentTime);
void flushBufferedChanges(AttachedStage& attachedStage, float currentTime);
void processChangeMap(AttachedStage& attachedStage);

} // namespace usdparser
} // namespace physx
} // namespace omni
