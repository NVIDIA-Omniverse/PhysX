// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"

#include <carb/settings/ISettings.h>

#include "../../framework/BmBenchmark.h"
#include "../../framework/BmGlobals.h"
#include "../../../common/PhysicsChangeTemplate.h"
#include "../../../common/TestVehicleFactory.h"

#include <iostream>
#include <string>

#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxVehicle.h>

void initPhysicsVehicle()
{
} // see implementation of bmInitialize


static const bool gUsePVD = false;


struct WriteBackFlags
{
    enum Enum
    {
        eUPDATE_TO_USD          = (1 << 0),
        eUPDATE_VELOCITY_TO_USD = (1 << 1),
        eDEFAULT                = eUPDATE_TO_USD | eUPDATE_VELOCITY_TO_USD
    };
};

class PhysicsVehiclesBenchmark : public BmBenchmark
{
public:
    PhysicsVehiclesBenchmark(const bool useVehicleExtension, const uint32_t writeBackFlags = WriteBackFlags::eDEFAULT,
        const bool useGPU = true, const int32_t threadCount = -1)
        : BmBenchmark()
        , mPhysXSimulation(nullptr)
        , mPhysXBenchmarks(nullptr)
        , mStage(nullptr)
        , mTimeStep(1.0f / 60.0f)
        , mWriteBackFlags(writeBackFlags)
        , mUseVehicleExtension(useVehicleExtension)
    {
        carb::Framework* framework = BmGlobals::getInstance().getFramework();
        mPhysXSimulation = framework->acquireInterface<omni::physx::IPhysxSimulation>();
        mPhysXBenchmarks = framework->acquireInterface<omni::physx::IPhysxBenchmarks>();

        if (useVehicleExtension)
        {
            // Load the vehicle plugin directly via Carbonite (no Kit extension manager)
            static const std::vector<const char*> kVehiclePlugins = { "omni.physx.vehicle.plugin" };
            carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
            desc.loadedFileWildcards = kVehiclePlugins.data();
            desc.loadedFileWildcardCount = kVehiclePlugins.size();
            desc.searchPaths = carb::kGraphenePluginsSearchPaths;
            desc.searchPathCount = CARB_COUNTOF(carb::kGraphenePluginsSearchPaths);
            framework->loadPlugins(desc);

            mPhysXVehicleInterface = framework->acquireInterface<omni::physx::IPhysxVehicle>();
        }
        else
            mPhysXVehicleInterface = nullptr;

        carb::settings::ISettings* settings = getSettings();
        mOriginalUpdateToUSD = settings->getAsBool(omni::physx::kSettingUpdateToUsd);
        mOriginalUpdateVelocityToUSD = settings->getAsBool(omni::physx::kSettingUpdateVelocitiesToUsd);
        mOriginalGPUOverride = settings->getAsInt(omni::physx::kSettingOverrideGPU);
        mOriginalThreadCount = settings->getAsInt(omni::physx::kSettingNumThreads);

        settings->setBool(omni::physx::kSettingUpdateToUsd, (writeBackFlags & WriteBackFlags::eUPDATE_TO_USD));
        settings->setBool(omni::physx::kSettingUpdateVelocitiesToUsd, (writeBackFlags & WriteBackFlags::eUPDATE_VELOCITY_TO_USD));
        settings->setInt(omni::physx::kSettingOverrideGPU, useGPU ? 1 : 0);
        if (threadCount >= 0)
            settings->setInt(omni::physx::kSettingNumThreads, threadCount);

        if (gUsePVD)
        {
            settings->setBool(omni::physx::kSettingPVDEnabled, true);
        }
    }

    virtual ~PhysicsVehiclesBenchmark()
    {
        carb::settings::ISettings* settings = getSettings();

        settings->setBool(omni::physx::kSettingUpdateToUsd, mOriginalUpdateToUSD);
        settings->setBool(omni::physx::kSettingUpdateVelocitiesToUsd, mOriginalUpdateVelocityToUSD);
        settings->setInt(omni::physx::kSettingOverrideGPU, mOriginalGPUOverride);
        settings->setInt(omni::physx::kSettingNumThreads, mOriginalThreadCount);

        if (gUsePVD)
        {
            settings->setBool(omni::physx::kSettingPVDEnabled, false);
        }

        // Vehicle plugin stays loaded (no dynamic unloading needed without Kit extension manager)
    }

    virtual uint32_t getNbRuns() const
    {
        return 3;
    }

    virtual uint32_t getNbSteps() const
    {
        return 1;
    }

    virtual void startRun()
    {
        // setup basic stage
        mStage = PXR_NS::UsdStage::CreateInMemory();
        const PXR_NS::SdfPath defaultPrimPath("/World");
        PXR_NS::UsdPrim defaultPrim = mStage->DefinePrim(defaultPrimPath);
        mStage->SetDefaultPrim(defaultPrim);

        PXR_NS::UsdUtilsStageCache::Get().Insert(mStage);

        mStepIndex = 0;
    }

    virtual void preStep()
    {
    }

    virtual void step()
    {
        if (mUseVehicleExtension)
            mPhysXVehicleInterface->updateControllers(mTimeStep);

        mPhysXSimulation->simulate(mTimeStep, mTimeStep * mStepIndex);
        mPhysXSimulation->fetchResults();

        mStepIndex++;
    }

    virtual void endRun()
    {
        if (mUseVehicleExtension)
            mPhysXVehicleInterface->detachStage(false);

        mPhysXSimulation->detachStage();
        mPhysXBenchmarks->loadTargetStage(nullptr);
        mVehiclePaths.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

    void attach()
    {
        long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

        mPhysXSimulation->attachStage(stageId);

        if (mUseVehicleExtension)
            mPhysXVehicleInterface->attachStage(stageId, true);
    }

    void createScenario(const uint32_t vehicleCount,
        const VehicleFactory::DriveMode::Enum driveMode = VehicleFactory::DriveMode::eNONE,
        const bool useSharableComponents = true, const bool createCollisionShapesForWheels = true);

    void createScenarioAccelerate(const uint32_t vehicleCount, const float accelerator = 1.0f,
        const VehicleFactory::DriveMode::Enum driveMode = VehicleFactory::DriveMode::eBASIC);

    carb::settings::ISettings* getSettings()
    {
        carb::Framework* framework = carb::getFramework();
        return framework->acquireInterface<carb::settings::ISettings>();
    }

protected:
    omni::physx::IPhysxSimulation*  mPhysXSimulation;
    omni::physx::IPhysxBenchmarks*  mPhysXBenchmarks;
    omni::physx::IPhysxVehicle*     mPhysXVehicleInterface;
    PXR_NS::UsdStageRefPtr             mStage;
    std::vector<PXR_NS::SdfPath>       mVehiclePaths;
    uint32_t                        mStepIndex;
    float                           mTimeStep;
    uint32_t                        mWriteBackFlags;
    int32_t                         mOriginalGPUOverride;
    int32_t                         mOriginalThreadCount;
    bool                            mOriginalUpdateToUSD;
    bool                            mOriginalUpdateVelocityToUSD;

    // currently this is not really working. The vehicle extension has a hard dependency on omni.physx.ui
    // which in turn has a hard dependency on viewport which fails to load due to ImGui. The benchmarks
    // run as a console app, so this will not really work anyway. The vehicle extension and omni.physx.ui
    // will have to transform all these kind of dependencies to weak dependencies first, checking whether
    // the interfaces are available with tryAcquire... and then adjust all code to handle the case of
    // the interfaces not being available.
    bool                            mUseVehicleExtension;
};


void PhysicsVehiclesBenchmark::createScenario(const uint32_t vehicleCount,
    const VehicleFactory::DriveMode::Enum driveMode,
    const bool useSharableComponents, const bool createCollisionShapesForWheels)
{
    const UnitScale unitScale = { 1.0f, 1.0f };

    mVehiclePaths.resize(vehicleCount);

    VehicleFactory::Car4WheelsScenarioParams params;
    params.driveMode = driveMode;
    params.createCollisionShapesForWheels = createCollisionShapesForWheels;
    params.addChassisCollisionBox = true;
    params.vehiclePathsOut = mVehiclePaths.data();
    params.timeStepsPerSecond = uint32_t(1.0f / mTimeStep);

    bool* useSharableComponentsList;
    if (!useSharableComponents)
    {
        useSharableComponentsList = reinterpret_cast<bool*>(alloca(sizeof(bool) * vehicleCount));
        for (uint32_t i = 0; i < vehicleCount; i++)
        {
            useSharableComponentsList[i] = false;
        }

        params.useShareableComponentsList = useSharableComponentsList;
    }

    VehicleFactory::create4WheeledCarsScenario(mStage, unitScale, vehicleCount, params);

    // fix the substep count to 3
    for (uint32_t i = 0; i < vehicleCount; i++)
    {
        PXR_NS::UsdPrim vehiclePrim = mStage->GetPrimAtPath(mVehiclePaths[i]);
        PXR_NS::PhysxSchemaPhysxVehicleAPI vehicleAPI(vehiclePrim);
        vehicleAPI.GetLowForwardSpeedSubStepCountAttr().Set(3);
        vehicleAPI.GetHighForwardSpeedSubStepCountAttr().Set(3);
    }
}


void PhysicsVehiclesBenchmark::createScenarioAccelerate(const uint32_t vehicleCount,
    const float accelerator, const VehicleFactory::DriveMode::Enum driveMode)
{
    createScenario(vehicleCount, driveMode);

    for (uint32_t i = 0; i < vehicleCount; i++)
    {
        PXR_NS::UsdPrim vehiclePrim = mStage->GetPrimAtPath(mVehiclePaths[i]);
        PXR_NS::PhysxSchemaPhysxVehicleControllerAPI controllerAPI(vehiclePrim);
        controllerAPI.GetAcceleratorAttr().Set(accelerator);
    }
}

//
// benchmark cost of simulation steps
//
template<const bool tUseVehicleExtension, const uint32_t tWriteBackFlags, const uint32_t tDriveMode, const bool tUseGPU>
class PhysicsVehiclesBenchmarkSimulate : public PhysicsVehiclesBenchmark
{
public:
    PhysicsVehiclesBenchmarkSimulate()
        : PhysicsVehiclesBenchmark(tUseVehicleExtension, tWriteBackFlags, tUseGPU)
    {
    }

    virtual ~PhysicsVehiclesBenchmarkSimulate()
    {
    }

    virtual uint32_t getNbRuns() const override
    {
        return 10;
    }

    virtual uint32_t getNbSteps() const override
    {
        return 10;
    }

    virtual void startRun() override
    {
        PhysicsVehiclesBenchmark::startRun();

        createScenarioAccelerate(100, 1.0f, static_cast<VehicleFactory::DriveMode::Enum>(tDriveMode));

        attach();

        // run an initial step for "warm start"
        step();
    }
};


Register<PhysicsVehiclesBenchmarkSimulate<false,
    0,
    VehicleFactory::DriveMode::eBASIC,
    false> > pvhSimBasicNoVExtNoUSD("PhysicsVehicles.Simulate_BasicDrive_OmniPhysXOnly");

Register<PhysicsVehiclesBenchmarkSimulate<false,
    0,
    VehicleFactory::DriveMode::eSTANDARD,
    false> > pvhSimStandardNoVExtNoUSD("PhysicsVehicles.Simulate_StandardDrive_OmniPhysXOnly");

Register<PhysicsVehiclesBenchmarkSimulate<false,
    (WriteBackFlags::eUPDATE_TO_USD | WriteBackFlags::eUPDATE_VELOCITY_TO_USD),
    VehicleFactory::DriveMode::eBASIC,
    false> > pvhSimBasicNoVExt("PhysicsVehicles.Simulate_BasicDrive_OmniPhysXOnly_USDUpdate");

// see comment for mUseVehicleExtension
//Register<PhysicsVehiclesBenchmarkSimulate<true,
//    (WriteBackFlags::eUPDATE_TO_USD | WriteBackFlags::eUPDATE_VELOCITY_TO_USD),
//    VehicleFactory::DriveMode::eBASIC,
//    false> > pvhSimBasic("PhysicsVehicles.Simulate_BasicDrive_USDUpdate");


//
// benchmark cost of parsing USD and setting up vehicles
//
template<const bool tUseVehicleExtension>
class PhysicsVehiclesBenchmarkSetup : public PhysicsVehiclesBenchmark
{
public:
    PhysicsVehiclesBenchmarkSetup()
        : PhysicsVehiclesBenchmark(tUseVehicleExtension, WriteBackFlags::eDEFAULT, false)
    {
    }

    virtual ~PhysicsVehiclesBenchmarkSetup()
    {
    }

    virtual uint32_t getNbRuns() const override
    {
        return 3;
    }

    virtual uint32_t getNbSteps() const override
    {
        return 1;
    }

    virtual void startRun() override
    {
        PhysicsVehiclesBenchmark::startRun();

        createScenario(100, VehicleFactory::DriveMode::eSTANDARD, false);
    }

    virtual void step() override
    {
        attach();
    }
};


Register<PhysicsVehiclesBenchmarkSetup<false> > pvhSetupNoVExt("PhysicsVehicles.Setup_OmniPhysXOnly");

// see comment for mUseVehicleExtension
//Register<PhysicsVehiclesBenchmarkSetup<true> > pvhSetup("PhysicsVehicles.Setup");


//
// benchmark cost of setting kinematicEnabled/vehicleEnabled/collisionEnabled (DriveSim use case)
//

template<const bool tUseFabric, const bool tUseVehicleExtension, const uint32_t tWriteBackFlags,
    const uint32_t tDriveMode, const bool tUseGPU>
class PhysicsVehiclesBenchmarkSetEnabledDisabled : public PhysicsVehiclesBenchmark
{
public:
    PhysicsVehiclesBenchmarkSetEnabledDisabled()
        : PhysicsVehiclesBenchmark(tUseVehicleExtension, tWriteBackFlags, tUseGPU)
    {   
    }

    virtual ~PhysicsVehiclesBenchmarkSetEnabledDisabled()
    {
    }

    virtual uint32_t getNbRuns() const override
    {
        return 10;
    }

    virtual uint32_t getNbSteps() const override
    {
        return 10;
    }

    inline void createBoolAttribute(const PXR_NS::SdfPath& primPath, const omni::fabric::Token& attrToken)
    {
        const omni::fabric::Type boolType(omni::fabric::BaseDataType::eBool);

        const omni::fabric::Path pathId =
            omni::fabric::convertToPathType<omni::fabric::Path>(mFabricChange.iStageReaderWriter->getFabricId(mFabricChange.mSrwId), primPath);
        mFabricChange.iStageReaderWriter->createAttribute(mFabricChange.mSrwId, pathId,
           attrToken, omni::fabric::TypeC(boolType));
    }

    inline void writeBoolAttribute(const PXR_NS::SdfPath& primPath, const omni::fabric::Token& attrToken, const bool value)
    {
        const omni::fabric::Path pathId = omni::fabric::convertToPathType<omni::fabric::Path>(
            mFabricChange.iStageReaderWriter->getFabricId(mFabricChange.mSrwId), primPath);
        bool& valData = *(bool*)(mFabricChange.iStageReaderWriter->getAttributeWr(mFabricChange.mSrwId, pathId, attrToken)).ptr;
        valData = value;
    }

    virtual void startRun() override
    {
        PhysicsVehiclesBenchmark::startRun();

        createScenario(2 * sHalfVehicleCount, static_cast<VehicleFactory::DriveMode::Enum>(tDriveMode),
            true, false);

        mVehiclePrims.reserve(mVehiclePaths.size());
        mCollisionPrims.reserve(mVehiclePaths.size());

        if (tUseFabric)
        {
            long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

            // called after all prims have been created as the following creates the prims on the
            // fabric side etc.
            mFabricChange.init(stageId, BmGlobals::getInstance().getFramework());

            mKinematicEnabledToken = omni::fabric::Token(PXR_NS::UsdPhysicsTokens->physicsKinematicEnabled.GetText());
            mCollisionEnabledToken = omni::fabric::Token(PXR_NS::UsdPhysicsTokens->physicsCollisionEnabled.GetText());
            mVehicleEnabledToken = omni::fabric::Token(PXR_NS::PhysxSchemaTokens->physxVehicleVehicleEnabled.GetText());
        }

        for (uint32_t i = 0; i < mVehiclePaths.size(); i++)
        {
            if (tUseFabric)
            {
                createBoolAttribute(mVehiclePaths[i], mKinematicEnabledToken);
                createBoolAttribute(mVehiclePaths[i], mVehicleEnabledToken);
            }

            PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(mVehiclePaths[i]);
            mVehiclePrims.push_back(prim);

            PXR_NS::UsdPrimSiblingRange children = prim.GetChildren();
            for (PXR_NS::UsdPrimSiblingIterator p = children.begin(); p != children.end(); p++)
            {
                PXR_NS::UsdPrim child = *p;
                if (child.HasAPI<PXR_NS::UsdPhysicsCollisionAPI>())
                {
                    mCollisionPrims.push_back(child);

                    if (tUseFabric)
                    {
                        createBoolAttribute(child.GetPath(), mCollisionEnabledToken);
                    }

                    break;
                }
            }
        }

        attach();

        // run an initial step to set enabled/disabled
        step();
    }

    virtual void preStep()
    {
        mStepIndex++;
    }

    // Phase 1: set vehicleEnabled for vehicles being disabled (must be processed before kinematicEnabled=true)
    // and set kinematicEnabled=false for vehicles being enabled (safe to do while vehicle is disabled)
    void stepInternalPhase1()
    {
        if (mStepIndex & 1)
        {
            disableVehiclesPhase1(sHalfVehicleCount, (2 * sHalfVehicleCount) - 1);
            enableVehiclesPhase1(0, sHalfVehicleCount - 1);
        }
        else
        {
            disableVehiclesPhase1(0, sHalfVehicleCount - 1);
            enableVehiclesPhase1(sHalfVehicleCount, (2 * sHalfVehicleCount) - 1);
        }
    }

    // Phase 2: set kinematicEnabled=true and collisionEnabled for disabled vehicles,
    // and set vehicleEnabled=true and collisionEnabled for enabled vehicles
    void stepInternalPhase2()
    {
        if (mStepIndex & 1)
        {
            disableVehiclesPhase2(sHalfVehicleCount, (2 * sHalfVehicleCount) - 1);
            enableVehiclesPhase2(0, sHalfVehicleCount - 1);
        }
        else
        {
            disableVehiclesPhase2(0, sHalfVehicleCount - 1);
            enableVehiclesPhase2(sHalfVehicleCount, (2 * sHalfVehicleCount) - 1);
        }
    }

    virtual void step()
    {
        if (tUseFabric)
        {
            stepInternalPhase1();
            mPhysXSimulation->flushChanges();
            stepInternalPhase2();
            mPhysXSimulation->flushChanges();
        }
        else
        {
            {
                PXR_NS::SdfChangeBlock changeBlock;
                stepInternalPhase1();
            }
            {
                PXR_NS::SdfChangeBlock changeBlock;
                stepInternalPhase2();
            }
        }
    }

    virtual void endRun()
    {
        mVehiclePrims.clear();
        mCollisionPrims.clear();

        if (tUseFabric)
            mFabricChange.destroy();

        PhysicsVehiclesBenchmark::endRun();
    }

    // Phase 1 for enable: clear kinematicEnabled (safe while vehicle is still disabled)
    void enableVehiclesPhase1(const uint32_t startIndex, const uint32_t endIndex)
    {
        for (uint32_t i = startIndex; i < endIndex; i++)
        {
            PXR_NS::UsdPrim& vehiclePrim = mVehiclePrims[i];

            if (tUseFabric)
            {
                writeBoolAttribute(vehiclePrim.GetPath(), mKinematicEnabledToken, false);
            }
            else
            {
                PXR_NS::UsdPhysicsRigidBodyAPI rigidBodyAPI(vehiclePrim);
                rigidBodyAPI.GetKinematicEnabledAttr().Set(false);
            }
        }
    }

    // Phase 2 for enable: set vehicleEnabled and collisionEnabled
    void enableVehiclesPhase2(const uint32_t startIndex, const uint32_t endIndex)
    {
        for (uint32_t i = startIndex; i < endIndex; i++)
        {
            PXR_NS::UsdPrim& vehiclePrim = mVehiclePrims[i];
            PXR_NS::UsdPrim& collisionPrim = mCollisionPrims[i];

            if (tUseFabric)
            {
                writeBoolAttribute(collisionPrim.GetPath(), mCollisionEnabledToken, true);
                writeBoolAttribute(vehiclePrim.GetPath(), mVehicleEnabledToken, true);
            }
            else
            {
                PXR_NS::UsdPhysicsCollisionAPI collisionAPI(collisionPrim);
                collisionAPI.GetCollisionEnabledAttr().Set(true);

                PXR_NS::PhysxSchemaPhysxVehicleAPI vehicleAPI(vehiclePrim);
                vehicleAPI.GetVehicleEnabledAttr().Set(true);
            }
        }
    }

    // Phase 1 for disable: clear vehicleEnabled (must happen before kinematicEnabled=true)
    void disableVehiclesPhase1(const uint32_t startIndex, const uint32_t endIndex)
    {
        for (uint32_t i = startIndex; i < endIndex; i++)
        {
            PXR_NS::UsdPrim& vehiclePrim = mVehiclePrims[i];

            if (tUseFabric)
            {
                writeBoolAttribute(vehiclePrim.GetPath(), mVehicleEnabledToken, false);
            }
            else
            {
                PXR_NS::PhysxSchemaPhysxVehicleAPI vehicleAPI(vehiclePrim);
                vehicleAPI.GetVehicleEnabledAttr().Set(false);
            }
        }
    }

    // Phase 2 for disable: set kinematicEnabled and clear collisionEnabled
    void disableVehiclesPhase2(const uint32_t startIndex, const uint32_t endIndex)
    {
        for (uint32_t i = startIndex; i < endIndex; i++)
        {
            PXR_NS::UsdPrim& vehiclePrim = mVehiclePrims[i];
            PXR_NS::UsdPrim& collisionPrim = mCollisionPrims[i];

            if (tUseFabric)
            {
                writeBoolAttribute(collisionPrim.GetPath(), mCollisionEnabledToken, false);
                writeBoolAttribute(vehiclePrim.GetPath(), mKinematicEnabledToken, true);
            }
            else
            {
                PXR_NS::UsdPhysicsCollisionAPI collisionAPI(collisionPrim);
                collisionAPI.GetCollisionEnabledAttr().Set(false);

                PXR_NS::UsdPhysicsRigidBodyAPI rigidBodyAPI(vehiclePrim);
                rigidBodyAPI.GetKinematicEnabledAttr().Set(true);
            }
        }
    }

protected:
    std::vector<PXR_NS::UsdPrim> mVehiclePrims;
    std::vector<PXR_NS::UsdPrim> mCollisionPrims;
    omni::physx::FabricChange mFabricChange;
    omni::fabric::Token mKinematicEnabledToken;
    omni::fabric::Token mCollisionEnabledToken;
    omni::fabric::Token mVehicleEnabledToken;
    static const uint32_t sHalfVehicleCount = 50;
};


Register<PhysicsVehiclesBenchmarkSetEnabledDisabled<false,
    false,
    0,
    VehicleFactory::DriveMode::eBASIC,
    false> > pvhSetEnabledDisabledUSDBasicNoVExtNoUSD("PhysicsVehicles.SetEnabledDisabled_USD_BasicDrive_OmniPhysXOnly");

Register<PhysicsVehiclesBenchmarkSetEnabledDisabled<true,
    false,
    0,
    VehicleFactory::DriveMode::eBASIC,
    false> > pvhSetEnabledDisabledFabricBasicNoVExtNoUSD("PhysicsVehicles.SetEnabledDisabled_Fabric_BasicDrive_OmniPhysXOnly");
