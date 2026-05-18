// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/core/ITypeFactory.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>

#include <omni/physx/IPhysxSettings.h>
#include <carb/settings/ISettings.h>
#include "Tools.h"

namespace omni
{
namespace physx
{

class USDChange
{
public:
    USDChange()
    {
        mStage = nullptr;
    }

    ~USDChange()
    {
        mStage = nullptr;
    }

    void init(long stageId, carb::Framework* framework)
    {
        mStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

        mSettings = framework->acquireInterface<carb::settings::ISettings>();
        mUpdateUsd = mSettings->getAsBool(kSettingUpdateToUsd);
        mSettings->setBool(kSettingUpdateToUsd, true);
    }

    void initPrim(const pxr::SdfPath primPath)
    {
    }

    void destroy()
    {
        mStage = nullptr;
        mSettings->setBool(kSettingUpdateToUsd, mUpdateUsd);
    }

    template <typename T>
    void setAttributeValue(const pxr::SdfPath& primPath, const pxr::TfToken& primAttribute, const T& value)
    {
        pxr::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        pxr::UsdAttribute attr = prim.GetAttribute(primAttribute);

        attr.Set(value);
    }

    void setTransformation(const pxr::SdfPath& primPath, const pxr::GfVec3f& position, const pxr::GfQuatf& orientation)
    {
        pxr::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        static const pxr::TfToken xformOpPosition = pxr::TfToken("xformOp:translate");
        static const pxr::TfToken xformOpRotation = pxr::TfToken("xformOp:orient");
        pxr::UsdAttribute posProp = prim.GetAttribute(xformOpPosition);
        pxr::UsdAttribute rotProp = prim.GetAttribute(xformOpRotation);

        posProp.Set(position);
        rotProp.Set(orientation);
    }

    template <typename T>
    T getAttributeValue(const pxr::SdfPath& primPath, const pxr::TfToken& primAttribute)
    {
        pxr::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        pxr::UsdAttribute attr = prim.GetAttribute(primAttribute);

        T value;
        attr.Get(&value);

        return value;
    }

    bool isFabric()
    {
        return false;
    }

private:
    pxr::UsdStageRefPtr mStage;
    carb::settings::ISettings* mSettings;
    bool                mUpdateUsd;
    bool                mUpdateFastCache;
};



class FabricChange
{
public:
    FabricChange()
    {
        mStage = nullptr;
        iStageReaderWriter = nullptr;
        iSimStageWithHistory = nullptr;
        iFabric = nullptr;
    }

    ~FabricChange()
    {
        mStage = nullptr;
        iStageReaderWriter = nullptr;
        iSimStageWithHistory = nullptr;
        iFabric = nullptr;
    }

    void init(long stageId, carb::Framework* framework);

    void initPrim(const pxr::SdfPath primPath);

    void flushXforms();

    bool isFabric()
    {
        return true;
    }

    void destroy()
    {
        iStageReaderWriter->flushToRingBuffer(mStageId);
        iSimStageWithHistory->release(mStageId);
        mStage = nullptr;

        mSettings->setBool(kSettingUpdateToUsd, mUpdateUsd);
    }

    template <typename T>
    constexpr omni::fabric::Type toFabricType()
    {
        using namespace omni;

        if constexpr (std::is_same_v<T, int>)
            return fabric::Type(fabric::BaseDataType::eInt);
        else if constexpr (std::is_same_v<T, float>)
            return fabric::Type(fabric::BaseDataType::eFloat);
        else if constexpr (std::is_same_v<T, bool>)
            return fabric::Type(fabric::BaseDataType::eBool);
        else if constexpr (std::is_same_v<T, pxr::GfVec3f>)
            return fabric::Type(omni::fabric::BaseDataType::eFloat, 3, 0);
        else if constexpr (std::is_same_v<T, pxr::GfQuatf>)
            return fabric::Type(omni::fabric::BaseDataType::eFloat, 4, 0);
        else
        {
            static_assert(std::is_same_v<T, pxr::TfToken>, "Invalid type for setAttribute call.");
            return fabric::Type(omni::fabric::BaseDataType::eToken, 1, 0);
        }
    }

    template <typename T>
    void setAttributeValue(const pxr::SdfPath& primPath, const pxr::TfToken& primAttribute, const T& value)
    {
        omni::fabric::Token attrToken(primAttribute.GetText());

        omni::fabric::Type fabricType = toFabricType<T>();
        iStageReaderWriter->createAttribute(
            mSrwId,
            omni::fabric::convertToPathType<omni::fabric::Path>(iStageReaderWriter->getFabricId(mSrwId), primPath),
            attrToken, omni::fabric::TypeC(fabricType));

        if constexpr (std::is_same_v<T, pxr::TfToken>)
        {
            omni::fabric::Token& valData =
                *(omni::fabric::Token*)(iStageReaderWriter->getAttributeWr(
                                            mSrwId,
                                                                  omni::fabric::convertToPathType<omni::fabric::Path>(
                                                                      iStageReaderWriter->getFabricId(mSrwId), primPath),
                                                                  attrToken))
                              .ptr;
            valData =
                omni::fabric::convertToTokenType<omni::fabric::Token>(iStageReaderWriter->getFabricId(mSrwId), value);
        }
        else
        {
            T& valData = *(T*)(iStageReaderWriter->getAttributeWr(mSrwId,
                                                                  omni::fabric::convertToPathType<omni::fabric::Path>(
                                                                      iStageReaderWriter->getFabricId(mSrwId), primPath),
                                                                  attrToken))
                              .ptr;
            valData = value;
        }        
    }

    void setTransformation(const pxr::SdfPath& primPath, const pxr::GfVec3f& position, const pxr::GfQuatf& orientation)
    {
        // Read the existing local transform from USD, put it in a GfTransform, modify and set the
        // result in fabric.
        pxr::UsdGeomXformable xformable(mStage->GetPrimAtPath(primPath));
        bool resetsIgnored = false;
        pxr::GfMatrix4d transform;
        xformable.GetLocalTransformation(&transform, &resetsIgnored);

        pxr::GfTransform tr(transform);
        tr.SetTranslation(position);
        tr.SetRotation(pxr::GfRotation(orientation));
        transform = tr.GetMatrix();
            
        pxr::GfMatrix4d& valData =
            *(pxr::GfMatrix4d*)(iStageReaderWriter->getAttributeWr(mSrwId,
                                                                   omni::fabric::convertToPathType<omni::fabric::Path>(
                                                                   iStageReaderWriter->getFabricId(mSrwId), primPath),
                                                                   mLocalMatrixToken))
                 .ptr;
        valData = transform;
        auto iHierarchyMaker = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
        if (iHierarchyMaker)
        {
            auto iHierarchy = iHierarchyMaker->getFabricHierarchy(iStageReaderWriter->getFabricId(mSrwId), mStageId);
            iHierarchy->updateWorldXforms();
        }
    }

    pxr::GfTransform getTransformation(const pxr::SdfPath& primPath)
    {
        pxr::GfMatrix4d& valData =
            *(pxr::GfMatrix4d*)(iStageReaderWriter->getAttributeRd(mSrwId,
                                                                   omni::fabric::convertToPathType<omni::fabric::Path>(
                                                                       iStageReaderWriter->getFabricId(mSrwId), primPath),
                                                                   mLocalMatrixToken))
                 .ptr;        
        return pxr::GfTransform(valData);
    }    

    template <typename T>
    T getAttributeValue(const pxr::SdfPath& primPath, const pxr::TfToken& primAttribute)
    {
        pxr::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        pxr::UsdAttribute attr = prim.GetAttribute(primAttribute);

        T value;
        attr.Get(&value);

        return value;
    }

public:
    pxr::UsdStageRefPtr                 mStage;
    omni::fabric::UsdStageId            mStageId;
    omni::fabric::IStageReaderWriter*   iStageReaderWriter;
    omni::fabric::IFabric*              iFabric;
    omni::fabric::IFabricUsd*           iFabricUsd;
    omni::fabric::ISimStageWithHistory* iSimStageWithHistory;
    omni::fabric::SimStageWithHistoryId mSwhId;
    omni::fabric::StageReaderWriterId   mSrwId;
    carb::settings::ISettings*          mSettings;
    bool                                mUpdateUsd;
    bool                                mUpdateFastCache;
    omni::fabric::Token                 mLocalMatrixToken;
    omni::fabric::Token                 mWorldMatrixToken;
};

}
}
