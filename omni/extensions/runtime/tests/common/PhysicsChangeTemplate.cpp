// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysicsChangeTemplate.h"

#include <omni/fabric/IFabric.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/core/ITypeFactory.h>

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/stage/StageReaderWriter.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>


using namespace carb;
using namespace omni::fabric;
using namespace omni::physx;


void FabricChange::init(long stageId, carb::Framework* framework)
{
    mStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    mStageId.id = stageId;

    mSettings = framework->acquireInterface<carb::settings::ISettings>();
    mUpdateUsd = mSettings->getAsBool(kSettingUpdateToUsd);
    mSettings->setBool(kSettingUpdateToUsd, true);


    iFabric = framework->tryAcquireInterface<omni::fabric::IFabric>();
    iStageReaderWriter = framework->tryAcquireInterface<omni::fabric::IStageReaderWriter>();
    iSimStageWithHistory = framework->tryAcquireInterface<omni::fabric::ISimStageWithHistory>();

    mSwhId = iSimStageWithHistory->getOrCreate(mStageId, 0, { 1, 60 }, omni::fabric::GpuComputeType::eCuda);
    mSrwId = iStageReaderWriter->create(mStageId, 0);

    mLocalMatrixToken = omni::fabric::Token(gLocalMatrixTokenString);
    mWorldMatrixToken = omni::fabric::Token(gWorldMatrixTokenString);
}

void FabricChange::initPrim(const pxr::SdfPath primPath)
{
    const omni::fabric::Path fabricPath =
        omni::fabric::convertToPathType<omni::fabric::Path>(iStageReaderWriter->getFabricId(mSrwId), primPath);

    omni::fabric::Type matrix4dType(omni::fabric::BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);
    iStageReaderWriter->createAttribute(mSrwId, fabricPath, mLocalMatrixToken, omni::fabric::TypeC(matrix4dType));
    iStageReaderWriter->createAttribute(mSrwId, fabricPath, mWorldMatrixToken, omni::fabric::TypeC(matrix4dType));

    flushXforms();
}

void FabricChange::flushXforms()
{
    usdrt::UsdStageRefPtr stage = usdrt::UsdStage::Attach(mStageId);
    stage->SynchronizeToFabric();

    auto iHierarchyMaker = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
    if (iHierarchyMaker)
    {
        auto iHierarchy = iHierarchyMaker->getFabricHierarchy(iStageReaderWriter->getFabricId(mSrwId), mStageId);
        iHierarchy->updateWorldXforms();
    }
}
