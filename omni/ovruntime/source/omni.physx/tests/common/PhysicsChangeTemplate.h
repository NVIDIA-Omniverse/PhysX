// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/core/ITypeFactory.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>

#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSimulation.h>
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
        destroy();
    }

    void init(long stageId, carb::Framework* framework)
    {
        mStage = PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(stageId));

        mSettings = framework->acquireInterface<carb::settings::ISettings>();
        mUpdateUsd = mSettings->getAsBool(kSettingUpdateToUsd);
        mSettings->setBool(kSettingUpdateToUsd, true);
    }

    void initPrim(const PXR_NS::SdfPath primPath)
    {
    }

    void destroy()
    {
        mStage = nullptr;

        if (mSettings)
        {
            mSettings->setBool(kSettingUpdateToUsd, mUpdateUsd);
            mSettings = nullptr;
        }
    }

    template <typename T>
    void setAttributeValue(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& primAttribute, const T& value)
    {
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        PXR_NS::UsdAttribute attr = prim.GetAttribute(primAttribute);

        attr.Set(value);
    }

    void setTransformation(const PXR_NS::SdfPath& primPath, const PXR_NS::GfVec3f& position, const PXR_NS::GfQuatf& orientation)
    {
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        static const PXR_NS::TfToken xformOpPosition = PXR_NS::TfToken("xformOp:translate");
        static const PXR_NS::TfToken xformOpRotation = PXR_NS::TfToken("xformOp:orient");
        PXR_NS::UsdAttribute posProp = prim.GetAttribute(xformOpPosition);
        PXR_NS::UsdAttribute rotProp = prim.GetAttribute(xformOpRotation);

        posProp.Set(position);
        rotProp.Set(orientation);
    }

    template <typename T>
    T getAttributeValue(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& primAttribute)
    {
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        PXR_NS::UsdAttribute attr = prim.GetAttribute(primAttribute);

        T value;
        attr.Get(&value);

        return value;
    }

    bool isFabric()
    {
        return false;
    }

private:
    PXR_NS::UsdStageRefPtr mStage;
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
        iFabricUsd = nullptr;
        mSettings = nullptr;
    }

    ~FabricChange()
    {
        destroy();
    }

    void init(long stageId, carb::Framework* framework);

    void initPrim(const PXR_NS::SdfPath primPath);

    void flushXforms();

    bool isFabric()
    {
        return true;
    }

    void destroy()
    {
        if(iStageReaderWriter)
        {
            iStageReaderWriter->flushToRingBuffer(mStageId);
            iStageReaderWriter = nullptr;
        }

        if (iSimStageWithHistory)
        {
            iSimStageWithHistory->release(mStageId);
            iSimStageWithHistory = nullptr;
        }   

        mStage = nullptr;
        iFabric = nullptr;

        if (mSettings)
        {
            mSettings->setBool(kSettingUpdateToUsd, mUpdateUsd);
            mSettings = nullptr;
        }
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
        else if constexpr (std::is_same_v<T, PXR_NS::GfVec3f>)
            return fabric::Type(omni::fabric::BaseDataType::eFloat, 3, 0);
        else if constexpr (std::is_same_v<T, PXR_NS::GfQuatf>)
            return fabric::Type(omni::fabric::BaseDataType::eFloat, 4, 0);
        else
        {
            static_assert(std::is_same_v<T, PXR_NS::TfToken>, "Invalid type for setAttribute call.");
            return fabric::Type(omni::fabric::BaseDataType::eToken, 1, 0);
        }
    }

    template <typename T>
    void setAttributeValue(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& primAttribute, const T& value)
    {
        omni::fabric::Token attrToken(primAttribute.GetText());

        omni::fabric::Type fabricType = toFabricType<T>();
        iStageReaderWriter->createAttribute(
            mSrwId,
            omni::fabric::convertToPathType<omni::fabric::Path>(iStageReaderWriter->getFabricId(mSrwId), primPath),
            attrToken, omni::fabric::TypeC(fabricType));

        if constexpr (std::is_same_v<T, PXR_NS::TfToken>)
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

    void setTransformation(const PXR_NS::SdfPath& primPath, const PXR_NS::GfVec3f& position, const PXR_NS::GfQuatf& orientation)
    {
        // Read the existing local transform from USD, put it in a GfTransform, modify and set the
        // result in fabric.
        PXR_NS::UsdGeomXformable xformable(mStage->GetPrimAtPath(primPath));
        bool resetsIgnored = false;
        PXR_NS::GfMatrix4d transform;
        xformable.GetLocalTransformation(&transform, &resetsIgnored);

        PXR_NS::GfTransform tr(transform);
        tr.SetTranslation(position);
        tr.SetRotation(PXR_NS::GfRotation(orientation));
        transform = tr.GetMatrix();
            
        PXR_NS::GfMatrix4d& valData =
            *(PXR_NS::GfMatrix4d*)(iStageReaderWriter->getAttributeWr(mSrwId,
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

    PXR_NS::GfTransform getTransformation(const PXR_NS::SdfPath& primPath)
    {
        PXR_NS::GfMatrix4d& valData =
            *(PXR_NS::GfMatrix4d*)(iStageReaderWriter->getAttributeRd(mSrwId,
                                                                   omni::fabric::convertToPathType<omni::fabric::Path>(
                                                                       iStageReaderWriter->getFabricId(mSrwId), primPath),
                                                                   mLocalMatrixToken))
                 .ptr;        
        return PXR_NS::GfTransform(valData);
    }    

    template <typename T>
    T getAttributeValue(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& primAttribute)
    {
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);

        PXR_NS::UsdAttribute attr = prim.GetAttribute(primAttribute);

        T value;
        attr.Get(&value);

        return value;
    }

public:
    PXR_NS::UsdStageRefPtr                 mStage;
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
