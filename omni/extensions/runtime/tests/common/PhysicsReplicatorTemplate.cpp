// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysicsReplicatorTemplate.h"

#include <omni/fabric/IFabric.h>
#include <omni/fabric/connectivity/Connectivity.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/physx/IPhysxReplicator.h>

#include <usdrt/population/IUtils.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>

using namespace carb;
using namespace omni::fabric;
using namespace omni::physx;
using namespace pxr;

void USDReplicator::clone(const SdfPath& envPath, const SdfPath& sourceEnvPath,  uint32_t numEnvs, uint32_t numCols, float offset)
{
    SdfChangeBlock changeBlock;

    const SdfLayerHandle layer = mStage->GetRootLayer();

    for (uint32_t i = 1; i < numEnvs; i++)
    {
        // copy env
        const uint32_t row = i / numCols;
        const uint32_t col = i % numCols;
        const double x = row * offset;
        const double y = col * offset;
        const GfVec3d offsetVec(x, y, 0.0);

        const std::string curEnvName = "env" + std::to_string(i);
        const SdfPath curEnvPath = envPath.AppendChild(TfToken(curEnvName));

        {
            SdfPrimSpecHandle primSpec = SdfCreatePrimInLayer(layer, curEnvPath);
            SdfCopySpec(layer, sourceEnvPath, layer, curEnvPath);
            {
                static TfToken gTranslate("xformOp:translate");
                const pxr::SdfPath attributePath = curEnvPath.AppendProperty(gTranslate);
                SdfAttributeSpecHandle posAttr = primSpec->GetAttributeAtPath(attributePath);
                const GfVec3d currentPos = posAttr->GetDefaultValue().UncheckedGet<GfVec3d>();
                posAttr->SetDefaultValue(VtValue(currentPos + offsetVec));
            }
        }
    }
}

void USDReplicator::replicate(uint64_t sourceEnvPath, uint32_t numReplication)
{
    auto iReplicator = carb::getCachedInterface<omni::physx::IPhysxReplicator>();
    iReplicator->replicate(mStageId, sourceEnvPath, numReplication, false, false);
}

void populateFabricFromUsdStage(IFabricUsd* fabricUsd, FabricId fabricId, StageReaderWriterId stageInProgress, pxr::UsdStageRefPtr& usdStage)
{
    const omni::fabric::Token fabricTransform("omni:fabric:localMatrix");
    const omni::fabric::Token fabricWorldTransform("omni:fabric:worldMatrix");
    const omni::fabric::Type matrixType = omni::fabric::Type(
        omni::fabric::BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);

    StageReaderWriter stage(stageInProgress);

    pxr::UsdPrimRange primRange = usdStage->Traverse();
    std::set<omni::fabric::TokenC> filter = {};
    for (auto iter = primRange.begin(), end = primRange.end(); iter != end; ++iter)
    {
        // collision groups/friction tables can have multiple targets in rels, which are not supported by fabric
        if (!(*iter).IsA<pxr::UsdPhysicsCollisionGroup>() && !(*iter).IsA<pxr::PhysxSchemaPhysxVehicleTireFrictionTable>())
            fabricUsd->prefetchPrimToFabric(fabricId, (*iter).GetPath(), *iter, filter, false,true, pxr::UsdTimeCode::Default());
        if ((*iter).IsA<pxr::UsdGeomXformable>())
        {
            pxr::UsdGeomXformable xf(*iter);            
            GfMatrix4d localMat;
            bool resetXf;
            xf.GetLocalTransformation(&localMat, &resetXf);
            const GfMatrix4d worldMat = xf.ComputeLocalToWorldTransform(UsdTimeCode::Default());
            GfMatrix4d* localFabricMat = stage.getOrCreateAttributeWr<GfMatrix4d>(omni::fabric::asInt(xf.GetPrim().GetPrimPath()), fabricTransform, matrixType);
            *localFabricMat = localMat;

            GfMatrix4d* worldFabricMat = stage.getOrCreateAttributeWr<GfMatrix4d>(omni::fabric::asInt(xf.GetPrim().GetPrimPath()), fabricWorldTransform, matrixType);
            *worldFabricMat = worldMat;
        }
    }
    fabricUsd->loadPrefetchedPrimData(fabricId, true);
}

void FabricReplicator::init(long stageId, carb::Framework* framework)
{
    mStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    mStageId.id = stageId;

    mSettings = framework->acquireInterface<carb::settings::ISettings>();
    mUpdateUsd = mSettings->getAsBool(kSettingUpdateToUsd);
    mSettings->setBool(kSettingUpdateToUsd, false);


    // populate fabric    
    auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();

    iSimStageWithHistory->getOrCreate(mStageId, 1, { 1, 30 }, omni::fabric::GpuComputeType::eCuda);
    iStageReaderWriter->create(mStageId, 0);

    auto stageReaderWriterId = iStageReaderWriter->get(mStageId);

    omni::fabric::FabricId fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);

    auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
    iFabricUsd->setEnableChangeNotifies(fabricId, false);

    auto populationUtils = omni::core::createType<usdrt::population::IUtils>();
    populationUtils->setEnableUsdNoticeHandling(mStageId, fabricId, true);

    // Fill the stage in progress with USD values
    populationUtils->populateFromUsd(
        stageReaderWriterId, mStageId, omni::fabric::asInt(PXR_NS::SdfPath::AbsoluteRootPath()), nullptr, 0.0);
}

void FabricReplicator::destroy()
{
    mStage = nullptr;
    mSettings->setBool(kSettingUpdateToUsd, mUpdateUsd);
    auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
    auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();

    iStageReaderWriter->flushToRingBuffer(mStageId);
    iSimStageWithHistory->release(mStageId);
}

void copyFabricPrim(const usdrt::SdfPath& sourcePathUsdrt,
                    const usdrt::SdfPath& targetPathUsdrt,
                    omni::fabric::StageReaderWriter& stageRW,
                    const GfVec3d& offsetVec,
                    const omni::fabric::Token& worldToken)
{
    const omni::fabric::PathC sourcePath(sourcePathUsdrt);
    const omni::fabric::PathC targetPath(targetPathUsdrt);

    const omni::fabric::Token sourcePrimType = stageRW.getPrimTypeName(sourcePath);

    stageRW.createPrim(targetPath);
    if (sourcePrimType != omni::fabric::kUninitializedToken)
        stageRW.setPrimTypeName(targetPath, sourcePrimType);
    stageRW.copyAttributes(sourcePath, targetPath);

    pxr::GfMatrix4d* worldPose =
        stageRW.getAttributeWr<pxr::GfMatrix4d>(targetPath, worldToken);
    if (worldPose)
    {
        const GfVec3d currentPos = worldPose->ExtractTranslation();
        const GfVec3d newPos = currentPos + offsetVec;
        worldPose->SetTranslateOnly(newPos);         
    }
}

void FabricReplicator::clone(const pxr::SdfPath& envPathPxr,
                             const pxr::SdfPath& sourceEnvPathPxr,
                             uint32_t numEnvs,
                             uint32_t numCols,
                             float offset)
{
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(mStageId);


    const usdrt::SdfPath envPath(envPathPxr.GetText());
    const usdrt::SdfPath sourceEnvPath(sourceEnvPathPxr.GetText());


    omni::fabric::StageReaderWriter stageRw(stageInProgress);

    omni::fabric::IStageReaderWriterLegacy* isrwLegacy =
        carb::getCachedInterface<omni::fabric::IStageReaderWriterLegacy>();

    const omni::fabric::Token fabricTransform("omni:fabric:localMatrix");
    const omni::fabric::Token fabricWorldTransform("omni:fabric:worldMatrix");


    std::vector<omni::fabric::Path> list_of_clones;
    for (unsigned int i = 1; i < numEnvs; ++i)
    {
        std::string pathString = "/World/envs/env" + std::to_string(i);
        list_of_clones.push_back(omni::fabric::Path(pathString.c_str()));
    }

    isrwLegacy->batchClone(iStageReaderWriter->getFabricId(stageInProgress), omni::fabric::Path(sourceEnvPath.GetText()),
                           { (const omni::fabric::PathC*)list_of_clones.data(), list_of_clones.size() });

    for (uint32_t i = 1; i < numEnvs; i++)
    {
        const std::string curEnvName = "env" + std::to_string(i);
        const usdrt::SdfPath curEnvPath = envPath.AppendChild(usdrt::TfToken(curEnvName));

        const uint32_t row = i / numCols;
        const uint32_t col = i % numCols;
        const double x = row * offset;
        const double y = col * offset;
        const pxr::GfVec3d offsetVec(x, y, 0.0);

        pxr::GfMatrix4d* localPose =
            stageRw.getAttributeWr<pxr::GfMatrix4d>(omni::fabric::PathC(curEnvPath), fabricTransform);
        if (localPose)
        {
            const GfVec3d currentPos = localPose->ExtractTranslation();
            const GfVec3d newPos = currentPos + offsetVec;
            localPose->SetTranslateOnly(newPos);
        }
    }

    auto iFabricHierarchy = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
    if (iFabricHierarchy != nullptr)
    {
        auto fabricHierarchy = iFabricHierarchy->getFabricHierarchy(stageRw.getFabricId(), mStageId);
        fabricHierarchy->updateWorldXforms();
    }
}

void FabricReplicator::replicate(uint64_t sourceEnvPath, uint32_t numReplication)
{
    auto iReplicator = carb::getCachedInterface<omni::physx::IPhysxReplicator>();
    iReplicator->replicate(mStageId.id, sourceEnvPath, numReplication, false, true);
}
