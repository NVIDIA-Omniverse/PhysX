// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysicsChangeTemplate.h"

#include <omni/fabric/IFabric.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/core/ITypeFactory.h>

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/stage/StageReaderWriter.h>


using namespace carb;
using namespace omni::fabric;
using namespace omni::physx;

const omni::fabric::IPath* omni::fabric::Path::iPath = nullptr;
const omni::fabric::IToken* omni::fabric::Token::iToken = nullptr;

void createFabricFromUsdStage(IFabricUsd* fabricUsd, FabricId fabricId, pxr::UsdStageRefPtr& usdStage)
{
    UsdStageId usdStageId = pxr::UsdUtilsStageCache::Get().Insert(usdStage).ToLongInt();

    pxr::UsdPrimRange primRange = usdStage->Traverse();
    std::set<omni::fabric::TokenC> filter = {};
    for (auto iter = primRange.begin(), end = primRange.end(); iter != end; ++iter)
    {
        // collision groups/friction tables can have multiple targets in rels, which are not supported by fabric
        if (!(*iter).IsA<pxr::UsdPhysicsCollisionGroup>() && !(*iter).IsA<pxr::PhysxSchemaPhysxVehicleTireFrictionTable>())
            fabricUsd->prefetchPrimToFabric(fabricId, (*iter).GetPath(), *iter, filter, false,true, pxr::UsdTimeCode::Default());
    }
    fabricUsd->loadPrefetchedPrimData(fabricId, true);
}

void FabricChange::init(long stageId, carb::Framework* framework)
{
    mStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    mStageId.id = stageId;

    mSettings = framework->acquireInterface<carb::settings::ISettings>();
    mUpdateUsd = mSettings->getAsBool(kSettingUpdateToUsd);
    mSettings->setBool(kSettingUpdateToUsd, true);


    iFabric = framework->tryAcquireInterface<omni::fabric::IFabric>();
    iFabricUsd = framework->tryAcquireInterface<omni::fabric::IFabricUsd>();
    iStageReaderWriter = framework->tryAcquireInterface<omni::fabric::IStageReaderWriter>();
    iSimStageWithHistory = framework->tryAcquireInterface<omni::fabric::ISimStageWithHistory>();

    omni::fabric::Token::iToken = getFramework()->tryAcquireInterface<omni::fabric::IToken>();
    omni::fabric::Path::iPath = getFramework()->tryAcquireInterface<omni::fabric::IPath>();


    mSwhId = iSimStageWithHistory->getOrCreate(mStageId, 0, { 1, 60 }, omni::fabric::GpuComputeType::eCuda);
    mSrwId = iStageReaderWriter->create(mStageId, 0);
    FabricId fabricId = iStageReaderWriter->getFabricId(mSrwId);

    mLocalMatrixToken = omni::fabric::Token(gLocalMatrixTokenString);
    mWorldMatrixToken = omni::fabric::Token(gWorldMatrixTokenString);

    createFabricFromUsdStage(iFabricUsd, fabricId, mStage);
}

void FabricChange::initPrim(const pxr::SdfPath primPath)
{
    omni::fabric::Type matrix4dType(omni::fabric::BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);
    iStageReaderWriter->createAttribute(
        mSrwId, { asInt(primPath) }, mLocalMatrixToken, omni::fabric::TypeC(matrix4dType));
    iStageReaderWriter->createAttribute(
        mSrwId, { asInt(primPath) }, mWorldMatrixToken, omni::fabric::TypeC(matrix4dType));
    iStageReaderWriter->logAttributeWriteForNotice(mSrwId, { asInt(primPath) }, mLocalMatrixToken);

    flushXforms();
}

void FabricChange::flushXforms()
{
    usdrt::UsdStageRefPtr stage = usdrt::UsdStage::Attach(mStageId);
    stage->SynchronizeToFabric();
}
