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
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxFabric.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>
#include <usdrt/population/IUtils.h>

void initPhysicsUSDSimulate()
{
} // see implementation of bmInitialize

using namespace carb;

enum UsdFileExtension
{
    usda = 0,
    usd = 1
};

template <uint32_t tThreadCount, bool tGpu, UsdFileExtension tUsdFileExtension, bool usdWriteTest, bool useFabric, bool suppressReadback>
class PhysicsUSDSimulateBenchmark : public BmBenchmark
{
public:
    PhysicsUSDSimulateBenchmark(const char* testName,
                                bool loadPython = false,
                                const char* const* argv = nullptr,
                                size_t argc = 0)
        : BmBenchmark()
    {
        mPhysXBenchmarks = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxBenchmarks>();
        mTestName = testName;
        mLoadPython = loadPython;
        mArgv = argv;
        mArgc = argc;

        carb::settings::ISettings* settings = BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
        numThreads = settings->getAsInt(omni::physx::kSettingNumThreads);
        gpuOverride = settings->getAsInt(omni::physx::kSettingOverrideGPU);

        if (suppressReadback)
        {
            settings->setBool(omni::physx::kSettingSuppressReadback, true);
            settings->setBool(omni::physx::kSettingFabricUseGPUInterop, true);
        }

        if (useFabric)
        {
            carb::settings::ISettings* settings = BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
            settings->setBool("/app/settings/fabricConnectivityWithoutFSD", true);
            settings->setBool(omni::physx::kSettingFabricEnabled, true);

            mPhysXFabricInterface = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxFabric>();
        }
        else
        {
            mPhysXFabricInterface = nullptr;
        }
    }

    virtual ~PhysicsUSDSimulateBenchmark()
    {
        mPhysXBenchmarks->loadTargetStage(nullptr);

        carb::settings::ISettings* settings = BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
        settings->setInt(omni::physx::kSettingNumThreads, numThreads);
        settings->setInt(omni::physx::kSettingOverrideGPU, gpuOverride);

        if (useFabric)
        {
            carb::settings::ISettings* settings = BmGlobals::getInstance().getFramework()->acquireInterface<carb::settings::ISettings>();
            settings->setBool(omni::physx::kSettingFabricEnabled, false);
        }

        if (suppressReadback)
        {
            settings->setBool(omni::physx::kSettingSuppressReadback, false);
            settings->setBool(omni::physx::kSettingFabricUseGPUInterop, false);
        }
    }

    virtual uint32_t getNbRuns() const
    {
        return 5;
    }


    virtual uint32_t getNbSteps() const
    {
        return 200;
    }

    virtual void startRun()
    {
        const BmGlobals& bmGlobals = BmGlobals::getInstance();
        if (bmGlobals.numThreads() != -1)
            mPhysXBenchmarks->setThreadCount(bmGlobals.numThreads());
        else
            mPhysXBenchmarks->setThreadCount(tThreadCount);

        if (bmGlobals.forceGpu() == true)
            mPhysXBenchmarks->overwriteGPUSetting(true);
        else
            mPhysXBenchmarks->overwriteGPUSetting(tGpu);

        if (bmGlobals.enableProfile())
            mPhysXBenchmarks->enablePVDProfile(true);

        std::string usdPath =
            getAssetDirectory(TestAssetDirectoryType::eDataRoot);

        std::string usdFileName;
        if (mLoadPython)
        {
            usdFileName = std::string("Physics/PythonTests/") + mTestName + std::string(".py");
        }
        else
        {
            std::string extension = (tUsdFileExtension == usda) ? std::string(".usda") : std::string(".usd");
            usdFileName = std::string("Physics/") + mTestName + extension;
        }

        usdPath += "/" + getAssetUriInDataSource(TestAssetType::eUsd, usdFileName.c_str());

        carb::scripting::Script* script = nullptr;
        long stageId = 0;
        if (mLoadPython)
        {
            long int stageId = mPhysXBenchmarks->createEmptyStage();

            std::string stageIdstr = std::to_string(stageId);
            size_t newArgc = mArgc + 1;
            std::vector<const char*> newArgv(newArgc);
            for (size_t i = 0; i < mArgc; ++i)
                newArgv[i] = mArgv[0];
            newArgv[mArgc] = stageIdstr.c_str();

            carb::scripting::IScripting* scripting = BmGlobals::getInstance().getPythonScripting();
            carb::scripting::Context* context = BmGlobals::getInstance().getPythonContext();

            script = scripting->createScriptFromFile(usdPath.c_str());
            bool res = scripting->executeScriptWithArgs(
                context, script, &newArgv[0], newArgc,
                carb::scripting::kOutputFlagCaptureStdout | carb::scripting::kOutputFlagCaptureStderr);

            if (res)
            {
                std::cout << scripting->getLastStdout(context);
                std::cout << scripting->getLastStderr(context);
            }
            else
            {
                std::cout << scripting->getLastExecutionError(context).message;
            }

            stageId = mPhysXBenchmarks->loadTargetStage_Id(stageId);
        }
        else
        {
            stageId = mPhysXBenchmarks->loadTargetStage(usdPath.c_str());
        }

        mStageId = stageId;
        omni::fabric::UsdStageId ustageId = { uint64_t(stageId)};

        if (mPhysXFabricInterface && stageId)
        {
            // populate fabric
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
                stageReaderWriterId, mStageId,
                omni::fabric::convertToPathType<omni::fabric::Path>(fabricId, PXR_NS::SdfPath::AbsoluteRootPath()),
                nullptr, 0.0);

            mPhysXFabricInterface->attachStage(stageId);
        }

        // A.B. make one additional step to avoid load spike
        mPhysXBenchmarks->update(1.0f / 60.0f, 0.0f);
        if (script)
            BmGlobals::getInstance().getPythonScripting()->destroyScript(script);
    }
     
    virtual void preStep()
    {
        if (usdWriteTest)
        {
            mPhysXBenchmarks->update(1.0f / 60.0f, 0.0f);
        }
        else
        {
            if (mPhysXFabricInterface && suppressReadback)
                mPhysXFabricInterface->update(1.0f / 60.0f, 0.0f);
        }
    }

    void step()
    {
        if (usdWriteTest)
        {            
            if (mPhysXFabricInterface)
                mPhysXFabricInterface->update(1.0f / 60.0f, 0.0f);
            else
                mPhysXBenchmarks->updateUsd();

        }
        else
        {
            mPhysXBenchmarks->update(1.0f / 60.0f, 0.0f);
        }        
    }

    void endRun()
    {
        mPhysXBenchmarks->loadTargetStage(nullptr);

        if (mPhysXFabricInterface)
        {
            mPhysXFabricInterface->detachStage();

            omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            omni::fabric::ISimStageWithHistory* iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();

            iStageReaderWriter->flushToRingBuffer(mStageId);
            iSimStageWithHistory->release(mStageId);
        }

    }

private:
    omni::physx::IPhysxBenchmarks*  mPhysXBenchmarks;
    omni::physx::IPhysxFabric*   mPhysXFabricInterface;
    std::string mTestName;
    bool mLoadPython;
    const char* const* mArgv;
    size_t mArgc;
    int numThreads;
    int gpuOverride;
    long mStageId;
};

template <UsdFileExtension tUsdFileExtension>
class PhysicsUSDLoadBenchmark : public BmBenchmark
{
public:
    PhysicsUSDLoadBenchmark(const char* testName) : BmBenchmark()
    {
        // get interfaces
        mPhysXBenchmarks = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxBenchmarks>();
        mTestName = testName;
    }

    virtual ~PhysicsUSDLoadBenchmark()
    {
        mPhysXBenchmarks->loadTargetStage(nullptr);
    }

    virtual uint32_t getNbRuns() const
    {
        return 3;
    }

    virtual uint32_t getNbSteps() const
    {
        return 3;
    }

    virtual void startRun()
    {
    }

    virtual void preStep()
    {
        mPhysXBenchmarks->loadTargetStage(nullptr);
    }

    void step()
    {
        const BmGlobals& bmGlobals = BmGlobals::getInstance();

        std::string usdPath =
            getAssetDirectory(TestAssetDirectoryType::eDataRoot);
        std::string extension = (tUsdFileExtension == usda) ? std::string(".usda") : std::string(".usd");

        std::string usdFileName = std::string("Physics/") + mTestName + extension;
        usdPath += "/" + getAssetUriInDataSource(TestAssetType::eUsd, usdFileName.c_str());

        mPhysXBenchmarks->loadTargetStage(usdPath.c_str());         
    }

    void endRun()
    {
    }

protected:
private:
    omni::physx::IPhysxBenchmarks* mPhysXBenchmarks;
    std::string mTestName;
    std::vector<omni::physx::PhysicsProfileStats> mStats;
};

#define REGISTER_USD_SCENARIO(fileName, fileNameExtension)                                                             \
    template <uint32_t tThreadCount, bool tGpu>                                                                        \
    class fileName##USDSimulateTest : public PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, fileNameExtension, false, false, false>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDSimulateTest() : PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, fileNameExtension, false, false, false>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    template <uint32_t tThreadCount, bool tGpu>                                                                        \
    class fileName##USDSimulateUpdateUsdTest : public PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, fileNameExtension, true, false, false>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDSimulateUpdateUsdTest() : PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, fileNameExtension, true, false, false>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    class fileName##USDLoadTest : public PhysicsUSDLoadBenchmark<fileNameExtension>                                    \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDLoadTest() : PhysicsUSDLoadBenchmark(#fileName)                                                   \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    Register<fileName##USDSimulateTest<8, false>> var_##fileName##_GPUoff_2T(                                          \
        (std::string("Physics.Simulate_") + std::string(#fileName)).c_str());                                          \
    Register<fileName##USDSimulateUpdateUsdTest<8, false>> var_##fileName##_usd_write_GPUoff_2T(                                          \
        (std::string("Physics.UpdateUsd_") + std::string(#fileName)).c_str());                                          \
    Register<fileName##USDLoadTest > var_##fileName##_loadTest(                                                        \
        (std::string("Physics.LoadUSD_") + std::string(#fileName)).c_str());

#define REGISTER_FABRIC_SCENARIO(fileName, fileNameExtension)                                                       \
    template <uint32_t tThreadCount, bool tGpu>                                                                        \
    class fileName##USDSimulateUpdateFabricTest : public PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, fileNameExtension, true, true, false>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDSimulateUpdateFabricTest() : PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, fileNameExtension, true, true, false>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    Register<fileName##USDSimulateUpdateFabricTest<8, false>> var_##fileName##_fabric_write_GPUoff_2T(                       \
        (std::string("Physics.UpdateFabric_") + std::string(#fileName)).c_str());                                        

#define REGISTER_PYTHON_SCENARIO(fileName, arg0)                                                                       \
    template <uint32_t tThreadCount, bool tGpu>                                                                        \
    class fileName##PythonSimulateTest_##arg0 : public PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, usda, false, false>           \
    {                                                                                                                  \
    public:                                                                                                            \
        static const char* sArg0;                                                                                      \
        fileName##PythonSimulateTest_##arg0()                                                                          \
            : PhysicsUSDSimulateBenchmark<tThreadCount, tGpu, usda, false, false>(#fileName, true, &sArg0, 1)                        \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    template <uint32_t tThreadCount, bool tGpu>                                                                        \
    const char* fileName##PythonSimulateTest_##arg0<tThreadCount, tGpu>::sArg0 = #arg0;                                \
    Register<fileName##PythonSimulateTest_##arg0<2, false>> var_##fileName##_##arg0##_GPUoff_2T(                       \
        (std::string("Physics.Simulate_") + std::string(#fileName) + std::string("_") + std::string(#arg0)).c_str());

#define REGISTER_USD_SCENARIO_GPU_ONLY(fileName, fileNameExtension)                                                             \
    template <uint32_t tThreadCount>                                                                        \
    class fileName##USDSimulateTest : public PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, false, false, false>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDSimulateTest() : PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, false, false, false>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    template <uint32_t tThreadCount>                                                                        \
    class fileName##USDSimulateUpdateUsdTest : public PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, true, false, false>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDSimulateUpdateUsdTest() : PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, true, false, false>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    class fileName##USDLoadTest : public PhysicsUSDLoadBenchmark<fileNameExtension>                                    \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##USDLoadTest() : PhysicsUSDLoadBenchmark(#fileName)                                                   \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    Register<fileName##USDSimulateTest<8>> var_##fileName##_GPUon_2T(                                            \
        (std::string("Physics.Simulate_") + std::string(#fileName)).c_str());                                          \
    Register<fileName##USDSimulateUpdateUsdTest<8>> var_##fileName##_usd_write_GPUon_2T(                         \
        (std::string("Physics.UpdateUsd_") + std::string(#fileName)).c_str());                                         \
    Register<fileName##USDLoadTest > var_##fileName##_loadTest(                                                        \
        (std::string("Physics.LoadUSD_") + std::string(#fileName)).c_str());


#define REGISTER_FABRIC_SCENARIO_GPU_SUPPRESS_READBACK(fileName, fileNameExtension)                                                             \
    template <uint32_t tThreadCount>                                                                        \
    class fileName##FabricSuppressSimulateTest : public PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, false, true, true>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##FabricSuppressSimulateTest() : PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, false, true, true>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    template <uint32_t tThreadCount>                                                                        \
    class fileName##FabricSuppressSimulateUpdateUsdTest : public PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, true, true, true>        \
    {                                                                                                                  \
    public:                                                                                                            \
        fileName##FabricSuppressSimulateUpdateUsdTest() : PhysicsUSDSimulateBenchmark<tThreadCount, true, fileNameExtension, true, true, true>(#fileName)    \
        {                                                                                                              \
        }                                                                                                              \
    };                                                                                                                 \
    Register<fileName##FabricSuppressSimulateTest<8>> var_##fileName##_GPUon_2T(                                            \
        (std::string("Physics.Simulate_SuppressReadback_") + std::string(#fileName)).c_str());                                          \
    Register<fileName##FabricSuppressSimulateUpdateUsdTest<8>> var_##fileName##_usd_write_GPUon_2T(                         \
        (std::string("Physics.UpdateUsd_SuppressReadback_") + std::string(#fileName)).c_str());                                         \

//REGISTER_USD_SCENARIO(BoxOnPlane, usda)
//REGISTER_USD_SCENARIO(BoxOnPlaneInstanced, usda)
REGISTER_USD_SCENARIO(KaplaArena, usda)
REGISTER_FABRIC_SCENARIO(KaplaArena, usda)
REGISTER_USD_SCENARIO(CaterpillarTrackOnBridge, usd)
REGISTER_FABRIC_SCENARIO(CaterpillarTrackOnBridge, usd)
REGISTER_USD_SCENARIO(MultishapeBody, usd)
REGISTER_FABRIC_SCENARIO(MultishapeBody, usd)
REGISTER_FABRIC_SCENARIO_GPU_SUPPRESS_READBACK(MultishapeBody, usd)
REGISTER_USD_SCENARIO(FrankaCabinet512, usd)
REGISTER_FABRIC_SCENARIO(FrankaCabinet512, usd)
//REGISTER_USD_SCENARIO(FrankaCabinet128, usd)
//REGISTER_USD_SCENARIO(FrankaCabinet1024, usd)
//REGISTER_USD_SCENARIO(KaplaTower, usda)
//REGISTER_USD_SCENARIO(HugePileOfLargeConvexes, usda)
//REGISTER_USD_SCENARIO(Avalanche_15000_Instanced, usd)

// REGISTER_PYTHON_SCENARIO(grid_physics, 1)
// REGISTER_PYTHON_SCENARIO(grid_physics, 10)
// REGISTER_PYTHON_SCENARIO(grid_physics, 100)
// REGISTER_PYTHON_SCENARIO(grid_physics, 1000)


