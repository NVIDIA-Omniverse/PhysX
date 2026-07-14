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
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSettings.h>

#include <PxPhysicsAPI.h>

void initPhysicsSimulate()
{
} // see implementation of bmInitialize

using namespace carb;
using namespace omni::physx;

// Special omni gym setup - thread count 0, no scene queries
template <bool directPhysXUpdate>
class PhysicsOmniGymSimulateBenchmark : public BmBenchmark
{
public:
    PhysicsOmniGymSimulateBenchmark()
        : BmBenchmark()
    {
        mTestName = "simulateMultishapeBody.usda";
        mPhysXBenchmarks = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxBenchmarks>();
        mPhysX = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysx>();
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
        carb::settings::ISettings* settings = BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
        mNumThreads = settings->getAsInt(omni::physx::kSettingNumThreads);
        mDisableUsdWrite = settings->getAsBool(omni::physx::kSettingUpdateToUsd);
        mDisableVelocitiesUsdWrite = settings->getAsBool(omni::physx::kSettingUpdateVelocitiesToUsd);
        settings->setBool(omni::physx::kSettingUpdateToUsd, false);
        settings->setBool(omni::physx::kSettingUpdateVelocitiesToUsd, false);
        mPhysXScene = nullptr;
    }

    virtual ~PhysicsOmniGymSimulateBenchmark()
    {
        mPhysXBenchmarks->loadTargetStage(nullptr);

        carb::settings::ISettings* settings = BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
        settings->setInt(omni::physx::kSettingNumThreads, mNumThreads);
        settings->setBool(omni::physx::kSettingUpdateToUsd, mDisableUsdWrite);
        settings->setBool(omni::physx::kSettingUpdateToUsd, mDisableVelocitiesUsdWrite);
    }

    virtual uint32_t getNbRuns() const
    {
        return 5;
    }

    virtual uint32_t getNbSteps() const
    {
        return 200;
    }

    virtual void preStep()
    {

    }

    virtual void step()
    {
        if (directPhysXUpdate)
        {
            mPhysXScene->simulate(1.0f/60.0f);
            mPhysXScene->fetchResults();
        }
        else
        {
            //mPhysX->updateSimulation(1.0f/60.0f, 1.0f/60.0f);
            mPhysXSimulation->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            mPhysXSimulation->fetchResults();
        }
    }

    virtual void startRun()
    {
        const BmGlobals& bmGlobals = BmGlobals::getInstance();

        mPhysXBenchmarks->setThreadCount(0);        

        std::string usdPath =
            getAssetDirectory(TestAssetDirectoryType::eDataRoot);

        std::string usdFileName = std::string("Physics/") + mTestName;
        usdPath += "/" + getAssetUriInDataSource(TestAssetType::eUsd, usdFileName.c_str());

        carb::scripting::Script* script = nullptr;
        long stageId = mPhysXBenchmarks->loadTargetStage(usdPath.c_str());

        mStageId = stageId;

        // A.B. make one additional step to avoid load spike
        mPhysXBenchmarks->update(1.0f / 60.0f, 0.0f);

        mPhysXScene = static_cast<::physx::PxScene*>(mPhysX->getPhysXPtr(PXR_NS::SdfPath("/World/physicsScene"), ePTScene));
    }

    void endRun()
    {
        mPhysXBenchmarks->loadTargetStage(nullptr);
    }

private:
    omni::physx::IPhysxBenchmarks* mPhysXBenchmarks;
    omni::physx::IPhysx* mPhysX;
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    std::string mTestName;
    int mNumThreads;
    bool mDisableUsdWrite;
    bool mDisableVelocitiesUsdWrite;
    long mStageId;
    physx::PxScene* mPhysXScene;
};


//Register<PhysicsOmniGymSimulateBenchmark<true> > simOGbmSOL("PhysicsSimulate.OGMultiShapeSOL");
//Register<PhysicsOmniGymSimulateBenchmark<false> > simOGbm("PhysicsSimulate.OGMultiShape");
