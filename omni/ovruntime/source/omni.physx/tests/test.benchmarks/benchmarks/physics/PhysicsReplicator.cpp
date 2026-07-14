// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"


#include "../../common/TestHelpers.h"
#include "../../framework/BmBenchmark.h"
#include "../../framework/BmGlobals.h"

#include <iostream>
#include <string>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxReplicator.h>
#include <omni/physx/IPhysxSettings.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>
#include <usdrt/population/IUtils.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>
#include <omni/fabric/connectivity/Connectivity.h>


#include <PxPhysicsAPI.h>

using namespace omni::physx;
using namespace PXR_NS;
using namespace carb;

void initPhysicsReplicator()
{
} // see implementation of bmInitialize


class PhysicsRigidBodyReplicatorBenchmark : public BmBenchmark
{
public:
    PhysicsRigidBodyReplicatorBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mStage(nullptr), mAttached(false), mNumPrims(10000)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
    }

    virtual ~PhysicsRigidBodyReplicatorBenchmark()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 3;
    }


    virtual uint32_t getNbSteps() const
    {
        return 10;
    }

    virtual void startRun()
    {
        // setup basic mStage
        mStage = UsdStage::CreateInMemory();
        PXR_NS::UsdGeomSetStageUpAxis(mStage, TfToken("Z"));
        const float metersPerStageUnit = 0.01f; // work in default centimeters
        const double metersPerUnit = PXR_NS::UsdGeomSetStageMetersPerUnit(mStage, static_cast<double>(metersPerStageUnit));
        const SdfPath defaultPrimPath = SdfPath("/World");
        UsdPrim defaultPrim = mStage->DefinePrim(defaultPrimPath);
        mStage->SetDefaultPrim(defaultPrim);

        PXR_NS::UsdUtilsStageCache::Get().Insert(mStage);
        mStageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

        const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
        UsdPhysicsScene scene = UsdPhysicsScene::Define(mStage, physicsScenePath);

        UsdGeomScope::Define(mStage, SdfPath("/World/envs"));

        for (uint32_t i = 0; i < mNumPrims; i++)
        {
            // create rigid body
            std::string primPath = "/World/envs/env" + std::to_string(i) + "/box";
            UsdGeomCube cube = UsdGeomCube::Define(mStage, SdfPath(primPath));
            cube.CreateSizeAttr().Set(0.1);
            cube.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f((float)i));

            mPrims.push_back(cube.GetPrim());

            UsdPhysicsRigidBodyAPI::Apply(cube.GetPrim());
            UsdPhysicsCollisionAPI::Apply(cube.GetPrim());
        }
    }

    virtual void preStep()
    {
        if (mAttached)
            mPhysXSimulation->detachStage();
    }

    virtual void endRun()
    {
        if (mAttached)
            mPhysXSimulation->detachStage();
        mPrims.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

protected:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    PXR_NS::UsdStageRefPtr mStage;
    std::vector<PXR_NS::UsdPrim> mPrims;
    uint32_t mStep;
    bool mAttached;
    long mStageId;

    uint32_t mNumPrims;
};

class PhysicsRigidBodyReplicatorFullParseBenchmark : public PhysicsRigidBodyReplicatorBenchmark
{
public:
    PhysicsRigidBodyReplicatorFullParseBenchmark() : PhysicsRigidBodyReplicatorBenchmark()
    {
        mPhysX = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysx>();
    }

    virtual ~PhysicsRigidBodyReplicatorFullParseBenchmark()
    {
    }

    virtual void step()
    {
        mPhysXSimulation->attachStage(mStageId);
        mPhysX->forceLoadPhysicsFromUSD();
        mAttached = true;
    }

private:
    omni::physx::IPhysx* mPhysX;
};

class PhysicsRigidBodyReplicatorReplicateBenchmark : public PhysicsRigidBodyReplicatorBenchmark
{
public:
    PhysicsRigidBodyReplicatorReplicateBenchmark() : PhysicsRigidBodyReplicatorBenchmark()
    {
    }

    virtual ~PhysicsRigidBodyReplicatorReplicateBenchmark()
    {
    }

    virtual void startRun()
    {
        PhysicsRigidBodyReplicatorBenchmark::startRun();

        mPhysXReplicator = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxReplicator>();

        for (uint32_t i = 0; i < mNumPrims; i++)
        {
            mPositions.push_back({ (float)i, (float)i, (float)i });
            mOrientations.push_back(carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f });
            const std::string primPath = "/World/envs/env" + std::to_string(i) + "/box";
            mReplicatedBoxesPaths.push_back(SdfPath(primPath));
        }

        mCb = { nullptr, nullptr, nullptr };

        mCb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData) {
            numExludePaths = 1;
            const SdfPath boxPath = SdfPath("/World/envs");
            static uint64_t excludePath = sdfPathToInt(boxPath);
            excludePaths = &excludePath;
        };

        mCb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData) {
            std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
            const SdfPath outPath(stringPath);
            return sdfPathToInt(outPath);
        };

        mPhysXReplicator->registerReplicator(mStageId, mCb);
    }

    virtual void step()
    {
        mPhysXSimulation->attachStage(mStageId);
        mPhysXReplicator->replicate(mStageId, sdfPathToInt(SdfPath("/World/envs/env0/box")), mNumPrims - 1, false, false);
        mAttached = true;
    }

    virtual void endRun()
    {
        PhysicsRigidBodyReplicatorBenchmark::endRun();
        mPhysXReplicator->unregisterReplicator(mStageId);
    }

private:
    IPhysxReplicator* mPhysXReplicator;
    std::vector<carb::Float3> mPositions;
    std::vector<carb::Float4> mOrientations;
    std::vector<SdfPath> mReplicatedBoxesPaths;
    IReplicatorCallback mCb;
};

Register<PhysicsRigidBodyReplicatorReplicateBenchmark> prbRepF("PhysicsRigidBodyReplicator.Replicator");
Register<PhysicsRigidBodyReplicatorFullParseBenchmark> prbRepR("PhysicsRigidBodyReplicator.FullParse");


class PhysicsArticulationReplicatorBenchmark : public BmBenchmark
{
public:
    PhysicsArticulationReplicatorBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mStage(nullptr), mAttached(false), mFabricUsed(false)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
        mPhysXBenchmarks = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxBenchmarks>();

        carb::settings::ISettings* settings =
            BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
        mDisableUsdWrite = settings->getAsBool(omni::physx::kSettingUpdateToUsd);
        mDisableUsdVelWrite = settings->getAsBool(omni::physx::kSettingUpdateVelocitiesToUsd);
        mDisableNameWrite = settings->getAsBool(omni::physx::kSettingExposePrimPathNames);
        settings->setBool(omni::physx::kSettingUpdateToUsd, false);
        settings->setBool(omni::physx::kSettingUpdateVelocitiesToUsd, false);
        settings->setBool(omni::physx::kSettingExposePrimPathNames, false);

        //mPhysXBenchmarks->enableProfile(true);
    }

    virtual ~PhysicsArticulationReplicatorBenchmark()
    {
        carb::settings::ISettings* settings =
            BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
        settings->setBool(omni::physx::kSettingUpdateToUsd, mDisableUsdWrite);
        settings->setBool(omni::physx::kSettingUpdateVelocitiesToUsd, mDisableUsdVelWrite);
        settings->setBool(omni::physx::kSettingExposePrimPathNames, mDisableNameWrite);
    }

    virtual uint32_t getNbRuns() const
    {
        return 0;
    }


    virtual uint32_t getNbSteps() const
    {
        return 1;
    }

    virtual void startRun()
    {
    }

    void loadUsdStage(uint32_t numEnvs, bool useUSDOnly, int sceneIndex)
    {
        std::string usdFileName;
        const std::vector<std::string> envNames = { "ShadowHandOneEnv.usd", "HumanoidOneEnv.usd", "DigitOneEnv.usd", "FrankaCabinetOneEnv.usd"};

        std::string testFile = envNames[sceneIndex];
        usdFileName = std::string("Physics/") + testFile;

        std::string usdPath = getAssetDirectory(TestAssetDirectoryType::eDataRoot);
        usdPath += "/" + getAssetUriInDataSource(TestAssetType::eUsd, usdFileName.c_str());

        mStage = UsdStage::Open(usdPath.c_str());
        mStageId = UsdUtilsStageCache::Get().Insert(mStage).ToLongInt();

        UsdPhysicsScene scene = UsdPhysicsScene::Get(mStage, SdfPath("/physicsScene"));
        if (scene)
        {
            UsdAttribute attr =
                scene.GetPrim().CreateAttribute(TfToken("physxScene:envIdInBoundsBitCount"), SdfValueTypeNames->Int);
            attr.Set(4);
        }

        cloneCollisionGroups(numEnvs);

        if (!useUSDOnly)
        {
            // populate fabric
            mFabricUsed = true;

            omni::fabric::FabricId fabricId{};
            auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();

            iSimStageWithHistory->getOrCreate(mStageId, 1, { 1, 30 }, omni::fabric::GpuComputeType::eCuda);
            iStageReaderWriter->create(mStageId, 0);

            auto stageReaderWriterId = iStageReaderWriter->get(mStageId);
            fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);

            auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
            iFabricUsd->setEnableChangeNotifies(fabricId, false);

            auto populationUtils = omni::core::createType<usdrt::population::IUtils>();
            populationUtils->setEnableUsdNoticeHandling(mStageId, fabricId, true);

            // Fill the stage in progress with USD values
            populationUtils->populateFromUsd(
                stageReaderWriterId, mStageId,  omni::fabric::convertToPathType<omni::fabric::Path>(fabricId, PXR_NS::SdfPath::AbsoluteRootPath()), nullptr, 0.0);
        }
    }

    void releaseFabricResources()
    {
        if (!mFabricUsed) return;

        auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
        if (!iStageReaderWriter || !iSimStageWithHistory) return;

        auto stageReaderWriterId = iStageReaderWriter->get(mStageId);
        omni::fabric::FabricId fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);

        auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
        if (iFabricUsd)
            iFabricUsd->setEnableChangeNotifies(fabricId, true);

        auto populationUtils = omni::core::createType<usdrt::population::IUtils>();
        if (populationUtils)
            populationUtils->setEnableUsdNoticeHandling(mStageId, fabricId, false);

        iStageReaderWriter->flushToRingBuffer(mStageId);
        iSimStageWithHistory->release(mStageId);

        mFabricUsed = false;
    }

    virtual void endRun()
    {
        if (mAttached)
        {
            mPhysXSimulation->detachStage();
            mAttached = false;
        }

        if (mStage)
        {
            releaseFabricResources();
            PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
            mStage = nullptr;
        }
    }

    void cloneCollisionGroups(uint32_t numEnvs)
    {
        SdfChangeBlock changeBlock;

        const SdfPath colPath("/World/collisions");
        const SdfPath sourceColPath("/World/collisions/group0");
        const SdfPath envPath("/World/envs");
        const SdfPath sourceEnvPath("/World/envs/env_0");

        const SdfLayerHandle layer = mStage->GetRootLayer();

        const SdfPath globalColPath("/World/collisions/global_group");
        SdfPrimSpecHandle globalGroupSpec = SdfCreatePrimInLayer(layer, globalColPath);
        static TfToken gfilteredGroupsToken("physics:filteredGroups");
        const PXR_NS::SdfPath filColPath = globalColPath.AppendProperty(gfilteredGroupsToken);
        SdfRelationshipSpecHandle filRelAttr = globalGroupSpec->GetRelationshipAtPath(filColPath);
        {
            SdfTargetsProxy pathsList = filRelAttr->GetTargetPathList();
            pathsList.ClearEdits();
            pathsList.Append(globalColPath);
            pathsList.Append(sourceColPath);

            for (uint32_t i = 1; i < numEnvs; i++)
            {
                const std::string curEnvName = "env_" + std::to_string(i);
                const SdfPath curEnvPath = envPath.AppendChild(TfToken(curEnvName));

                // copy collision group
                {
                    const std::string curColName = "group" + std::to_string(i);
                    const SdfPath curColPath = colPath.AppendChild(TfToken(curEnvName));

                    SdfPrimSpecHandle primSpec = SdfCreatePrimInLayer(layer, curColPath);
                    SdfCopySpec(layer, sourceColPath, layer, curColPath);
                    {
                        static TfToken gColIncludeToken("collection:colliders:includes");
                        const PXR_NS::SdfPath relPath = curColPath.AppendProperty(gColIncludeToken);
                        SdfRelationshipSpecHandle relAttr = primSpec->GetRelationshipAtPath(relPath);
                        relAttr->ReplaceTargetPath(sourceEnvPath, curEnvPath);
                    }
                    {
                        const PXR_NS::SdfPath relPath = curColPath.AppendProperty(gfilteredGroupsToken);
                        SdfRelationshipSpecHandle relAttr = primSpec->GetRelationshipAtPath(relPath);
                        relAttr->ReplaceTargetPath(sourceColPath, curColPath);
                    }
                }
                pathsList.Append(curEnvPath);
            }
        }
    }

    void clone(uint32_t numEnvs, bool useUSD, uint32_t numCols, float offset)
    {
        if (useUSD)
        {
            cloneUSD(numEnvs, numCols, offset);
        }
        else
        {
            cloneFabric(numEnvs, numCols, offset);
        }
    }

    void cloneUSD(uint32_t numEnvs, uint32_t numCols, float offset)
    {
        SdfChangeBlock changeBlock;

        const SdfPath envPath("/World/envs");
        const SdfPath sourceEnvPath("/World/envs/env_0");

        const SdfLayerHandle layer = mStage->GetRootLayer();

        for (uint32_t i = 1; i < numEnvs; i++)
        {
            // copy env
            const uint32_t row = i / numCols;
            const uint32_t col = i % numCols;
            const double x = row * offset;
            const double y = col * offset;
            const GfVec3d offsetVec(x, y, 0.0);

            const std::string curEnvName = "env_" + std::to_string(i);
            const SdfPath curEnvPath = envPath.AppendChild(TfToken(curEnvName));

            {
                SdfPrimSpecHandle primSpec = SdfCreatePrimInLayer(layer, curEnvPath);
                SdfCopySpec(layer, sourceEnvPath, layer, curEnvPath);
                {
                    static TfToken gTranslate("xformOp:translate");
                    const PXR_NS::SdfPath attributePath = curEnvPath.AppendProperty(gTranslate);
                    SdfAttributeSpecHandle posAttr = primSpec->GetAttributeAtPath(attributePath);
                    const GfVec3d currentPos = posAttr->GetDefaultValue().UncheckedGet<GfVec3d>();
                    posAttr->SetDefaultValue(VtValue(currentPos + offsetVec));
                }
            }
        }
    }

    void copyFabricPrim(const usdrt::SdfPath& sourcePathUsdrt,
                        const usdrt::SdfPath& targetPathUsdrt,
                        omni::fabric::StageReaderWriter& stageRW,
                        const GfVec3d& offsetVec,
                        const omni::fabric::Token& worldToken)
    {
        const omni::fabric::Path sourcePath(sourcePathUsdrt);
        const omni::fabric::Path targetPath(targetPathUsdrt);

        const omni::fabric::Token sourcePrimType = stageRW.getPrimTypeName(sourcePath);

        stageRW.createPrim(targetPath);
        if (sourcePrimType != omni::fabric::Token())
            stageRW.setPrimTypeName(targetPath, sourcePrimType);
        stageRW.copyAttributes(sourcePath, targetPath);

        PXR_NS::GfMatrix4d* worldPose =
            stageRW.getAttributeWr<PXR_NS::GfMatrix4d>(targetPath, worldToken);
        if (worldPose)
        {
            const GfVec3d currentPos = worldPose->ExtractTranslation();
            const GfVec3d newPos = currentPos + offsetVec;
            worldPose->SetTranslateOnly(newPos);         
        }
    }

    void cloneFabric(uint32_t numEnvs, uint32_t numCols, float offset)
    {
        omni::fabric::IStageReaderWriter* iStageReaderWriter =
            carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(mStageId);


        usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(mStageId, stageInProgress);

        const usdrt::SdfPath envPath("/World/envs");
        const usdrt::SdfPath sourceEnvPath("/World/envs/env_0");

        omni::fabric::StageReaderWriter stageRw(stageInProgress);

        omni::fabric::IStageReaderWriterLegacy* isrwLegacy =
            carb::getCachedInterface<omni::fabric::IStageReaderWriterLegacy>();

        const omni::fabric::Token fabricTransform("omni:fabric:localMatrix");
        const omni::fabric::Token fabricWorldTransform("omni:fabric:worldMatrix");


        std::vector<omni::fabric::Path> list_of_clones;
        for (unsigned int i = 1; i < numEnvs; ++i)
        {
            std::string pathString = "/World/envs/env_" + std::to_string(i);
            list_of_clones.push_back(omni::fabric::Path(pathString.c_str()));
        }

        isrwLegacy->batchClone(iStageReaderWriter->getFabricId(stageInProgress),
                                   omni::fabric::Path(sourceEnvPath.GetText()),
                                   { (const omni::fabric::Path*)list_of_clones.data(), list_of_clones.size() });

    for (uint32_t i = 1; i < numEnvs; i++)
        {
            const std::string curEnvName = "env_" + std::to_string(i);
            const usdrt::SdfPath curEnvPath = envPath.AppendChild(usdrt::TfToken(curEnvName));

            const uint32_t row = i / numCols;
            const uint32_t col = i % numCols;
            const double x = row * offset;
            const double y = col * offset;
            const PXR_NS::GfVec3d offsetVec(x, y, 0.0);

            PXR_NS::GfMatrix4d* localPose =
                stageRw.getAttributeWr<PXR_NS::GfMatrix4d>(omni::fabric::Path(curEnvPath), fabricTransform);
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

protected:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    omni::physx::IPhysxBenchmarks* mPhysXBenchmarks;
    PXR_NS::UsdStageRefPtr mStage;
    bool mAttached;
    bool mFabricUsed;
    long mStageId;

    bool mDisableUsdWrite;
    bool mDisableUsdVelWrite;
    bool mDisableNameWrite;
};

template <uint32_t tNumEnvs, bool tUSD, int tSceneIndex>
class PhysicsArticulationCloneBenchmark : public PhysicsArticulationReplicatorBenchmark
{
public:
    PhysicsArticulationCloneBenchmark() : PhysicsArticulationReplicatorBenchmark()
    {
        const uint32_t numPerRow = uint32_t(sqrtf(tNumEnvs));
        const uint32_t numRows = std::ceil(tNumEnvs / numPerRow);
        mNumCols = std::ceil(tNumEnvs / numRows);

        mOffset = 4.0f;
    }

    virtual ~PhysicsArticulationCloneBenchmark()
    {
    }

    virtual void preStep()
    {
        if (mStage)
        {
            releaseFabricResources();
            PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
            mStage = nullptr;
        }
        loadUsdStage(tNumEnvs, tUSD, tSceneIndex);
    }

    virtual void step()
    {
        clone(tNumEnvs, tUSD, mNumCols, mOffset);
    }

private:
    uint32_t mNumCols;
    float mOffset;
};

template <uint32_t tNumEnvs, bool tUSD, int tSceneIndex>
class PhysicsArticulationPhysxReplicateBenchmark : public PhysicsArticulationReplicatorBenchmark
{
public:
    PhysicsArticulationPhysxReplicateBenchmark() : PhysicsArticulationReplicatorBenchmark(), mPhysXReplicator(nullptr)
    {
        const uint32_t numPerRow = uint32_t(sqrtf(tNumEnvs));
        const uint32_t numRows = std::ceil(tNumEnvs / numPerRow);
        mNumCols = std::ceil(tNumEnvs / numRows);

        mOffset = 4.0f;
    }

    virtual ~PhysicsArticulationPhysxReplicateBenchmark()
    {
    }

    virtual void startRun()
    {
        PhysicsArticulationReplicatorBenchmark::startRun();

        if (mStage)
        {
            releaseFabricResources();
            PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
            mStage = nullptr;
        }
        loadUsdStage(tNumEnvs, tUSD, tSceneIndex);
        clone(tNumEnvs, tUSD, mNumCols, mOffset);

        mPhysXReplicator = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxReplicator>();

        mCb = { nullptr, nullptr, nullptr };

        mCb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData) {
            numExludePaths = 1;
            const SdfPath boxPath = SdfPath("/World/envs");
            static uint64_t excludePath = sdfPathToInt(boxPath);
            excludePaths = &excludePath;
        };

        mCb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData) {
            std::string stringPath = "/World/envs/env_" + std::to_string(index + 1);
            const SdfPath outPath(stringPath);
            return sdfPathToInt(outPath);
        };

        mPhysXReplicator->registerReplicator(mStageId, mCb);
    }

    virtual void preStep()
    {
        if (mAttached)
        {
            mPhysXSimulation->detachStage();
            mAttached = false;
        }
    }

    virtual void step()
    {
        mPhysXSimulation->attachStage(mStageId);
        if (tUSD)
            mPhysXReplicator->replicate(mStageId, sdfPathToInt(SdfPath("/World/envs/env_0")), tNumEnvs - 1, true, false);
        else
            mPhysXReplicator->replicate(mStageId, sdfPathToInt(SdfPath("/World/envs/env_0")), tNumEnvs - 1, true, true);
        mAttached = true;
    }

    virtual void endRun()
    {
        if (mPhysXReplicator)
            mPhysXReplicator->unregisterReplicator(mStageId);
        PhysicsArticulationReplicatorBenchmark::endRun();
    }

private:
    uint32_t mNumCols;
    float mOffset;

    IPhysxReplicator* mPhysXReplicator;
    IReplicatorCallback mCb;
};

// shadow hand - scene index 0
Register<PhysicsArticulationCloneBenchmark<128, true, 0>> parRep128USD("PhysicsArticulationReplicator.ShadowHandUSDClone_128");
Register<PhysicsArticulationCloneBenchmark<128, false, 0>> parRep128Fabric("PhysicsArticulationReplicator.ShadowHandFabricClone_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, true, 0> > parRepPhysX128USD("PhysicsArticulationReplicator.ShadowHandUSDPhysxReplicate_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, false, 0> > parRepPhysX128Fabric("PhysicsArticulationReplicator.ShadowHandFabricPhysxReplicate_128");
Register<PhysicsArticulationCloneBenchmark<1024, true, 0> > parRep1024USD("PhysicsArticulationReplicator.ShadowHandUSDClone_1024");
Register<PhysicsArticulationCloneBenchmark<1024, false, 0> > parRep1024Fabric("PhysicsArticulationReplicator.ShadowHandFabricClone_1024");
Register<PhysicsArticulationPhysxReplicateBenchmark<1024, true, 0> > parRepPhysX1024USD("PhysicsArticulationReplicator.ShadowHandUSDPhysxReplicate_1024");
Register<PhysicsArticulationPhysxReplicateBenchmark<1024, false, 0> > parRepPhysX1024Fabric("PhysicsArticulationReplicator.ShadowHandFabricPhysxReplicate_1024");
Register<PhysicsArticulationCloneBenchmark<2048, true, 0> > parRep2048USD("PhysicsArticulationReplicator.ShadowHandUSDClone_2048");
Register<PhysicsArticulationCloneBenchmark<2048, false, 0> > parRep2048Fabric("PhysicsArticulationReplicator.ShadowHandFabricClone_2048");
Register<PhysicsArticulationPhysxReplicateBenchmark<2048, true, 0> > parRepPhysX2048USD("PhysicsArticulationReplicator.ShadowHandUSDPhysxReplicate_2048");
Register<PhysicsArticulationPhysxReplicateBenchmark<2048, false, 0> > parRepPhysX2048Fabric("PhysicsArticulationReplicator.ShadowHandFabricPhysxReplicate_2048");
Register<PhysicsArticulationPhysxReplicateBenchmark<4096, true, 0> > parRepPhysX40968USD("PhysicsArticulationReplicator.ShadowHandUSDPhysxReplicate_4096");
Register<PhysicsArticulationPhysxReplicateBenchmark<8192, true, 0> > parRepPhysX8192USD("PhysicsArticulationReplicator.ShadowHandUSDPhysxReplicate_8192");

// humanoid - scene index 1
Register<PhysicsArticulationCloneBenchmark<128, true, 1>> parRepH128USD("PhysicsArticulationReplicator.HumanoidUSDClone_128");
Register<PhysicsArticulationCloneBenchmark<128, false, 1>> parRepH128Fabric("PhysicsArticulationReplicator.HumanoidFabricClone_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, true, 1> > parRepHPhysX128USD("PhysicsArticulationReplicator.HumanoidUSDPhysxReplicate_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, false, 1> > parRepHPhysX128Fabric("PhysicsArticulationReplicator.HumanoidFabricPhysxReplicate_128");

Register<PhysicsArticulationPhysxReplicateBenchmark<1024, true, 1> > parRepHPhysX1024USD("PhysicsArticulationReplicator.HumanoidUSDPhysxReplicate_1024");
Register<PhysicsArticulationPhysxReplicateBenchmark<2048, true, 1> > parRepHPhysX2048USD("PhysicsArticulationReplicator.HumanoidUSDPhysxReplicate_2048");
Register<PhysicsArticulationPhysxReplicateBenchmark<4096, true, 1> > parRepHPhysX4096USD("PhysicsArticulationReplicator.HumanoidUSDPhysxReplicate_4096");
Register<PhysicsArticulationPhysxReplicateBenchmark<8192, true, 1> > parRepHPhysX8192USD("PhysicsArticulationReplicator.HumanoidUSDPhysxReplicate_8192");

// digit - scene index 2
Register<PhysicsArticulationCloneBenchmark<128, true, 2>> parRepD128USD("PhysicsArticulationReplicator.DigitUSDClone_128");
Register<PhysicsArticulationCloneBenchmark<128, false, 2>> parRepD128Fabric("PhysicsArticulationReplicator.DigitFabricClone_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, true, 2> > parRepDPhysX128USD("PhysicsArticulationReplicator.DigitUSDPhysxReplicate_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, false, 2> > parRepDPhysX128Fabric("PhysicsArticulationReplicator.DigitFabricPhysxReplicate_128");

Register<PhysicsArticulationPhysxReplicateBenchmark<1024, true, 2> > parRepDPhysX1024USD("PhysicsArticulationReplicator.DigitUSDPhysxReplicate_1024");
Register<PhysicsArticulationPhysxReplicateBenchmark<2048, true, 2> > parRepDPhysX2048USD("PhysicsArticulationReplicator.DigitUSDPhysxReplicate_2048");
Register<PhysicsArticulationPhysxReplicateBenchmark<4096, true, 2> > parRepDPhysX4096USD("PhysicsArticulationReplicator.DigitUSDPhysxReplicate_4096");
Register<PhysicsArticulationPhysxReplicateBenchmark<8192, true, 2> > parRepDPhysX8192USD("PhysicsArticulationReplicator.DigitUSDPhysxReplicate_8192");

// Franka cabinet - scene index 3
Register<PhysicsArticulationCloneBenchmark<128, true, 3>> parRepFK128USD("PhysicsArticulationReplicator.FrankaCabinetUSDClone_128");
Register<PhysicsArticulationCloneBenchmark<128, false, 3>> parRepFK128Fabric("PhysicsArticulationReplicator.FrankaCabinetFabricClone_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, true, 3> > parRepFKPhysX128USD("PhysicsArticulationReplicator.FrankaCabinetUSDPhysxReplicate_128");
Register<PhysicsArticulationPhysxReplicateBenchmark<128, false, 3> > parRepFkPhysX128Fabric("PhysicsArticulationReplicator.FrankaCabinetFabricPhysxReplicate_128");

Register<PhysicsArticulationPhysxReplicateBenchmark<1024, true, 3> > parRepFKPhysX1024USD("PhysicsArticulationReplicator.FrankaCabinetUSDPhysxReplicate_1024");
Register<PhysicsArticulationPhysxReplicateBenchmark<2048, true, 3> > parRepFKPhysX2048USD("PhysicsArticulationReplicator.FrankaCabinetUSDPhysxReplicate_2048");
Register<PhysicsArticulationPhysxReplicateBenchmark<4096, true, 3> > parRepFKPhysX4096USD("PhysicsArticulationReplicator.FrankaCabinetUSDPhysxReplicate_4096");
Register<PhysicsArticulationPhysxReplicateBenchmark<8192, true, 3> > parRepFKPhysX8192USD("PhysicsArticulationReplicator.FrankaCabinetUSDPhysxReplicate_8192");
