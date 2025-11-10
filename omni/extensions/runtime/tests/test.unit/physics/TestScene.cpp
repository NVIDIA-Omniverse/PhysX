// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"
#include "../../common/PhysicsChangeTemplate.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Collisions tests
TEST_CASE_TEMPLATE("Scene Tests", T, USDChange, FabricChange)
{
    T changeTemplate;

    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Gravity Direction")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsGravityDirection;

        GfVec3f direction = changeTemplate.template getAttributeValue<GfVec3f>(physicsScenePath, changeToken);
        // check default value
        compare(direction, GfVec3f(0.0f), epsilon);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);
        
        compare(scene->getGravity(), PxVec3(0.0f, 0.0f, -981.0f), epsilon);

        // disable collision API -> shape should be disabled
        direction[1] = 1.0f;
        changeTemplate.setAttributeValue(physicsScenePath, changeToken, direction);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created
        PxScene* newBasePtr = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene == newBasePtr);
        compare(scene->getGravity(), PxVec3(0.0f, 981.0f, 0.0f), epsilon);
    }

    SUBCASE("Gravity Magnitude")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsGravityMagnitude;

        float magnitude = changeTemplate.template getAttributeValue<float>(physicsScenePath, changeToken);
        // check default value
        CHECK(!isfinite(magnitude));

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        compare(scene->getGravity(), PxVec3(0.0f, 0.0f, -981.0f), epsilon);

        // disable collision API -> shape should be disabled
        magnitude = 1.0f;
        changeTemplate.setAttributeValue(physicsScenePath, changeToken, magnitude);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created
        PxScene* newBasePtr = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene == newBasePtr);
        compare(scene->getGravity(), PxVec3(0.0f, 0.0f, -1.0f), epsilon);
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


struct TestData
{
    uint32_t    numSteps;
    float       timeStep;
    uint32_t    numSimulationCompletes;
    uint32_t    numSimulationStarted;
    uint32_t    numSimulationEnded;
    PxScene*    scene;
    std::vector<PxSceneQueryUpdateMode::Enum> updateMode;

    TestData()
    {
        reset();
    }

    void reset()
    {
        numSteps = 0;
        timeStep = 1.0f;
        numSimulationCompletes = 0;
        numSimulationStarted = 0;
        numSimulationEnded = 0;
        scene = nullptr;
        updateMode.clear();
    }
};


void onPhysicsStepEventFn(float elapsedTime, void* userData)
{
    TestData* testData = static_cast<TestData*>(userData);

    if (testData)
    {
        testData->numSteps++;
        testData->timeStep = elapsedTime;
        if (testData->scene)
        {
            testData->updateMode.push_back(testData->scene->getSceneQueryUpdateMode());
        }
    }
}

void onPhysicsStepBlankFn(float elapsedTime, void* userData)
{

}


void onPhysicsCompletionEventFn(SimulationStatusEvent eventStatus, void* userData)
{
    TestData* testData = static_cast<TestData*>(userData);
    if (eventStatus == SimulationStatusEvent::eSimulationComplete)
    {
        testData->numSimulationCompletes++;
    }
    else if (eventStatus == SimulationStatusEvent::eSimulationStarting)
    {
        testData->numSimulationStarted++;
    }
    else if (eventStatus == SimulationStatusEvent::eSimulationEnded)
    {
        testData->numSimulationEnded++;
    }
}


//-----------------------------------------------------------------------------
// Scene stepping tests
TEST_CASE("Scene Stepping Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    TestData testData;

    SubscriptionId subscriptionId = physx->subscribePhysicsOnStepEvents(false, 0, onPhysicsStepEventFn, &testData);
    SubscriptionId physicsCompletionEventId = physx->subscribePhysicsSimulationEvents(onPhysicsCompletionEventFn, &testData);

    SUBCASE("Simulation Steps")
    {
        uint32_t timeStepsPerSeconds;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Get(&timeStepsPerSeconds);
        // check default value
        CHECK(timeStepsPerSeconds == 60);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        // step with high frequency first step is not enough to step
        testData.reset();
        physx->updateSimulation(1.0f/120.0f, 0.0f);
        CHECK(testData.numSteps == 0);
        CHECK(testData.numSimulationStarted == 0);
        CHECK(testData.numSimulationEnded == 0);
        CHECK(testData.numSimulationCompletes == 0);

        testData.reset();
        testData.scene = scene;
        physx->updateSimulation(1.0f / 120.0f, 0.0f);
        CHECK(testData.numSteps == 1);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);
        REQUIRE(testData.updateMode.size() == 1);
        CHECK(testData.updateMode[0] == PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED);

        // step with exact step frequency
        testData.reset();
        physx->updateSimulation(1.0f / 60.0f, 0.0f);        

        CHECK(testData.numSteps == 1);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);

        // step with low step frequency
        testData.reset();
        physx->updateSimulation(1.0f / 35.0f, 0.0f);      

        CHECK(testData.numSteps == 1); // should be one and time rest should be remembered, next time two steps happen
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);

        testData.reset();
        testData.scene = scene;
        physx->updateSimulation(1.0f / 35.0f, 0.0f);        

        CHECK(testData.numSteps == 2);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 2);
        CHECK(testData.numSimulationEnded == 2);
        CHECK(testData.numSimulationCompletes == 1);
        REQUIRE(testData.updateMode.size() == 2);
        CHECK(testData.updateMode[0] == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);
        CHECK(testData.updateMode[1] == PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED);


        // step with low step frequency below 25 fps -> should limit to 2
        testData.reset();
        physx->updateSimulation(1.0f / 10.0f, 0.0f);        

        CHECK(testData.numSteps == 2);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 2);
        CHECK(testData.numSimulationEnded == 2);
        CHECK(testData.numSimulationCompletes == 1);

        // flush the remaining steps
        testData.reset();
        physx->updateSimulation(1.0f / 1000.0f, 0.0f);

        CHECK(testData.numSteps == 2);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 2);
        CHECK(testData.numSimulationEnded == 2);
        CHECK(testData.numSimulationCompletes == 1);

        // change the stepping
        timeStepsPerSeconds = 300;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSeconds);

        // step with high frequency
        testData.reset();
        physx->updateSimulation(1.0f / 600.0f, 0.0f);        

        CHECK(testData.numSteps == 0);

        testData.reset();
        physx->updateSimulation(1.0f / 600.0f, 0.0f);

        CHECK(testData.numSteps == 1);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);

        testData.reset();
        physx->updateSimulation(1.0f / 100.0f, 0.0f);
        CHECK(testData.numSteps == 3);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 3);
        CHECK(testData.numSimulationEnded == 3);
        CHECK(testData.numSimulationCompletes == 1);

        
        testData.reset();
        testData.scene = scene;
        physx->updateSimulation(1.0f / 60.0f, 0.0f);        

        CHECK(testData.numSteps == 5);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 5);
        CHECK(testData.numSimulationEnded == 5);
        CHECK(testData.numSimulationCompletes == 1);
        REQUIRE(testData.updateMode.size() == 5);
        CHECK(testData.updateMode[0] == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);
        CHECK(testData.updateMode[1] == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);
        CHECK(testData.updateMode[2] == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);
        CHECK(testData.updateMode[3] == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);
        CHECK(testData.updateMode[4] == PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED);


        // step with low step frequency
        testData.reset();
        physx->updateSimulation(1.0f / 40.0f, 0.0f);        

        CHECK(testData.numSteps == 7);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 7);
        CHECK(testData.numSimulationEnded == 7);
        CHECK(testData.numSimulationCompletes == 1);

        // step with low step frequency below 20 fps -> should limit to 10
        testData.reset();
        physx->updateSimulation(1.0f / 10.0f, 0.0f);        

        CHECK(testData.numSteps == 10);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 10);
        CHECK(testData.numSimulationEnded == 10);
        CHECK(testData.numSimulationCompletes == 1);

    }

    SUBCASE("Simulation Infinite Steps")
    {        
        carb::settings::ISettings* settings = physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
        const int minFrameRate = settings->getAsInt(kSettingMinFrameRate);
        settings->setInt(kSettingMinFrameRate, 0);


        uint32_t timeStepsPerSeconds;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Get(&timeStepsPerSeconds);
        // check default value
        CHECK(timeStepsPerSeconds == 60);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        // step with high frequency first step is not enough to step
        testData.reset();
        physx->updateSimulation(1.0f / 120.0f, 0.0f);
        CHECK(testData.numSteps == 0);
        CHECK(testData.numSimulationStarted == 0);
        CHECK(testData.numSimulationEnded == 0);
        CHECK(testData.numSimulationCompletes == 0);

        // step with exact step frequency
        testData.reset();
        physx->updateSimulation(1.0f / 60.0f, 0.0f);

        CHECK(testData.numSteps == 1);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);


        // step with low step frequency -> should not limit to 2
        testData.reset();
        physx->updateSimulation(1.0f / 10.0f, 0.0f);

        CHECK(testData.numSteps == 6);
        CHECK(fabsf(testData.timeStep - 1.0f / float(timeStepsPerSeconds)) < epsilon);
        CHECK(testData.numSimulationStarted == 6);
        CHECK(testData.numSimulationEnded == 6);
        CHECK(testData.numSimulationCompletes == 1);

        settings->setInt(kSettingMinFrameRate, minFrameRate);
    }

    SUBCASE("Simulation Direct Stepping")
    {
        uint32_t timeStepsPerSeconds;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Get(&timeStepsPerSeconds);
        // check default value
        CHECK(timeStepsPerSeconds == 60);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);
        
        testData.reset();
        float stepTime = 0.01f;
        physxSim->simulate(stepTime, 0.0f);
        physxSim->fetchResults();
        CHECK(testData.numSteps == 1);
        CHECK(fabsf(testData.timeStep - stepTime) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);

        testData.reset();
        stepTime = 0.1f;
        physxSim->simulate(stepTime, 0.0f);
        physxSim->fetchResults();
        CHECK(testData.numSteps == 1);
        CHECK(fabsf(testData.timeStep - stepTime) < epsilon);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);
    }

    physx->unsubscribePhysicsOnStepEvents(subscriptionId);
    physx->unsubscribePhysicsSimulationEvents(physicsCompletionEventId);

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene stepping tests
TEST_CASE("Different Scene Updates Stepping Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=malesiani][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScene0Path = defaultPrimPath.AppendChild(TfToken("physicsScene0"));
    UsdPhysicsScene scene0 = UsdPhysicsScene::Define(stage, physicsScene0Path);
    PhysxSchemaPhysxSceneAPI physxScene0API = PhysxSchemaPhysxSceneAPI::Apply(scene0.GetPrim());
    physxScene0API.GetUpdateTypeAttr().Set(PhysxSchemaTokens.Get()->Disabled);

    const SdfPath physicsScene1Path = defaultPrimPath.AppendChild(TfToken("physicsScene1"));
    UsdPhysicsScene scene1 = UsdPhysicsScene::Define(stage, physicsScene1Path);
    PhysxSchemaPhysxSceneAPI physxScene1API = PhysxSchemaPhysxSceneAPI::Apply(scene1.GetPrim());
    physxScene1API.GetUpdateTypeAttr().Set(PhysxSchemaTokens.Get()->Synchronous);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    TestData testData;

    SubscriptionId subscriptionId = physx->subscribePhysicsOnStepEvents(false, 0, onPhysicsStepEventFn, &testData);
    SubscriptionId physicsCompletionEventId =
        physx->subscribePhysicsSimulationEvents(onPhysicsCompletionEventFn, &testData);

    SUBCASE("Test enabled/disabled scene updates")
    {
        uint32_t timeStepsPerSeconds;
        physxScene0API.GetTimeStepsPerSecondAttr().Get(&timeStepsPerSeconds);
        // check default value
        CHECK(timeStepsPerSeconds == 60);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
        CHECK(scene != nullptr);

        // step with exact step frequency. Update all non-disabled scenes.
        testData.reset();
        physx->updateSimulation(1.0f / 60.0f, 0.0f);
        CHECK(testData.numSteps == 1);
        CHECK(testData.numSimulationStarted == 1);
        CHECK(testData.numSimulationEnded == 1);
        CHECK(testData.numSimulationCompletes == 1);

        // step with exact step frequency but using the per-scene API, this bypasses 'Disabled'
        testData.reset();
        physx->updateSimulationScene(asInt(physicsScene0Path), 1.0f / 60.0f, 0.0f);
        physx->updateSimulationScene(asInt(physicsScene0Path), 1.0f / 60.0f, 0.0f);
        CHECK(testData.numSteps == 2);
        CHECK(testData.numSimulationStarted == 2);
        CHECK(testData.numSimulationEnded == 2);
        CHECK(testData.numSimulationCompletes == 2);

        // now make sure both are stepping after re-enabling scene0
        physx->resetSimulation();
        physxScene0API.GetUpdateTypeAttr().Set(PhysxSchemaTokens.Get()->Synchronous);
        testData.reset();
        physx->updateSimulationScene(asInt(physicsScene0Path), 1.0f / 60.0f, 0.0f);
        physx->updateSimulationScene(asInt(physicsScene0Path), 1.0f / 60.0f, 0.0f);
        CHECK(testData.numSteps == 2);
        CHECK(testData.numSimulationStarted == 2);
        CHECK(testData.numSimulationEnded == 2);
        CHECK(testData.numSimulationCompletes == 2);
    }

    physx->unsubscribePhysicsOnStepEvents(subscriptionId);
    physx->unsubscribePhysicsSimulationEvents(physicsCompletionEventId);

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene active actors tests
TEST_CASE("Scene Active Actors",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // create scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);


    SUBCASE("Default active actors on")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(scene != nullptr);

        CHECK((scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS) == true);
    }

    SUBCASE("Disable transformation updates")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        ISimulationCallback cb;
        cb.transformationWriteFn = nullptr;
        cb.velocityWriteFn = nullptr;
        cb.transformationUpdateFn = nullptr;        
        physxSim->setSimulationCallback(cb);
        physxSim->setSimulationOutputFlags(omni::physx::SimulationOutputType::eTRANSFORMATION,
            omni::physx::SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);
        physxSim->addSimulationOutputFlags(omni::physx::SimulationOutputType::eVELOCITY,
            omni::physx::SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(scene != nullptr);

        CHECK((scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS) == false);
    }

#if USE_PHYSX_GPU
    SUBCASE("Enable suppress readback")
    {
        carb::settings::ISettings* settings = physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
        const bool suppressReadback = settings->getAsBool(kSettingSuppressReadback);
        settings->setBool(kSettingSuppressReadback, true);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);


        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(scene != nullptr);

        CHECK((scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS) == false);

        settings->setBool(kSettingSuppressReadback, suppressReadback);
    }
#endif // USE_PHYSX_GPU

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene zero worker threads
TEST_CASE("Scene Worker Threads",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // create scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);
    const SdfPath boxPath = SdfPath("/World/box");

    SUBCASE("Carb CPU Dispatcher")
    {
        carb::settings::ISettings* settings = physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
        const bool physxDispatcher = settings->getAsBool(kSettingPhysxDispatcher);
        settings->setBool(kSettingPhysxDispatcher, false);

        SUBCASE("Two Threads")
        {
            const int numWorkerThreads = 2;

            const int numThreads = settings->getAsInt(kSettingNumThreads);
            settings->setInt(kSettingNumThreads, numWorkerThreads);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(scene != nullptr);

            physxSim->simulate(1.0f/60.0f, 0.0);
            physxSim->fetchResults();

            const GfVec3f pos = getPhysicsPrimPos(stage, boxPath);

            CHECK(pos[2] < 0.0f);

            CHECK(scene->getTaskManager()->getCpuDispatcher()->getWorkerCount() == numWorkerThreads);

            settings->setInt(kSettingNumThreads, numThreads);
        }

        SUBCASE("Zero Threads")
        {
            const int numWorkerThreads = 0;

            const int numThreads = settings->getAsInt(kSettingNumThreads);
            settings->setInt(kSettingNumThreads, numWorkerThreads);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(scene != nullptr);

            physxSim->simulate(1.0f / 60.0f, 0.0);
            physxSim->fetchResults();

            const GfVec3f pos = getPhysicsPrimPos(stage, boxPath);

            CHECK(pos[2] < 0.0f);

            CHECK(scene->getTaskManager()->getCpuDispatcher()->getWorkerCount() == numWorkerThreads);

            settings->setInt(kSettingNumThreads, numThreads);
        }

        settings->setBool(kSettingPhysxDispatcher, physxDispatcher);
    }

    SUBCASE("PhysX CPU Dispatcher")
    {
        carb::settings::ISettings* settings = physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
        const bool physxDispatcher = settings->getAsBool(kSettingPhysxDispatcher);
        settings->setBool(kSettingPhysxDispatcher, true);

        SUBCASE("Two Threads")
        {
            const int numWorkerThreads = 2;

            const int numThreads = settings->getAsInt(kSettingNumThreads);
            settings->setInt(kSettingNumThreads, numWorkerThreads);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(scene != nullptr);

            physxSim->simulate(1.0f / 60.0f, 0.0);
            physxSim->fetchResults();

            const GfVec3f pos = getPhysicsPrimPos(stage, boxPath);

            CHECK(pos[2] < 0.0f);

            CHECK(scene->getTaskManager()->getCpuDispatcher()->getWorkerCount() == numWorkerThreads);

            settings->setInt(kSettingNumThreads, numThreads);
        }

        SUBCASE("Zero Threads")
        {
            const int numWorkerThreads = 0;

            const int numThreads = settings->getAsInt(kSettingNumThreads);
            settings->setInt(kSettingNumThreads, numWorkerThreads);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(scene != nullptr);

            physxSim->simulate(1.0f / 60.0f, 0.0);
            physxSim->fetchResults();

            const GfVec3f pos = getPhysicsPrimPos(stage, boxPath);

            CHECK(pos[2] < 0.0f);

            CHECK(scene->getTaskManager()->getCpuDispatcher()->getWorkerCount() == numWorkerThreads);

            settings->setInt(kSettingNumThreads, numThreads);
        }

        settings->setBool(kSettingPhysxDispatcher, physxDispatcher);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene query disabled
TEST_CASE("Scene Query Disabled",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // create scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene physicsScene = UsdPhysicsScene::Define(stage, physicsScenePath);
    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(physicsScene.GetPrim());

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);
    const SdfPath boxPath = SdfPath("/World/box");

    SUBCASE("Scene Query Enabled")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(scene != nullptr);

        physxSim->simulate(1.0f / 60.0f, 0.0);
        physxSim->fetchResults();

        CHECK(scene->getSceneQueryUpdateMode() == PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED);

        PxShape* shape = reinterpret_cast<PxShape*>(physx->getPhysXPtr(boxPath, ePTShape));
        REQUIRE(shape);

        CHECK((shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE) == true);
    }

    SUBCASE("Scene Query Disabled")
    {
        physxSceneAPI.CreateEnableSceneQuerySupportAttr().Set(false);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(scene != nullptr);

        physxSim->simulate(1.0f / 60.0f, 0.0);
        physxSim->fetchResults();

        CHECK(scene->getSceneQueryUpdateMode() == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);

        PxShape* shape = reinterpret_cast<PxShape*>(physx->getPhysXPtr(boxPath, ePTShape));
        REQUIRE(shape);

        CHECK((shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE) == false);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene iteration count tests
TEST_CASE("Scene Interation Count",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());

    SUBCASE("RigidBody")
    {
        const SdfPath cubePath = defaultPrimPath.AppendChild(TfToken("cubePrim"));
        UsdGeomCube cube = UsdGeomCube::Define(stage, cubePath);
        UsdPhysicsRigidBodyAPI::Apply(cube.GetPrim());
        PhysxSchemaPhysxRigidBodyAPI rbAPI = PhysxSchemaPhysxRigidBodyAPI::Apply(cube.GetPrim());

        SUBCASE("Iteration Count Unclamped")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);            

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTActor));            
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);

            CHECK(numPosCounts == 16);
        }


        SUBCASE("Iteration Count Min Clamped")
        {
            const uint32_t minPosIt = 32;
            const uint32_t minVelIt = 31;
            physxSceneAPI.GetMinPositionIterationCountAttr().Set(minPosIt);
            physxSceneAPI.GetMinVelocityIterationCountAttr().Set(minVelIt);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);            

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);

            CHECK(numPosCounts == minPosIt);
            CHECK(numVelCounts == minVelIt);
        }

        SUBCASE("Iteration Count Max Clamped")
        {
            const uint32_t maxPosIt = 4;
            const uint32_t maxVelIt = 2;
            physxSceneAPI.GetMaxPositionIterationCountAttr().Set(maxPosIt);
            physxSceneAPI.GetMaxVelocityIterationCountAttr().Set(maxVelIt);

            rbAPI.GetSolverVelocityIterationCountAttr().Set(int(maxVelIt + 1)); // intentionally set higher than the limit

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);            

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);

            CHECK(numPosCounts == maxPosIt);
            CHECK(numVelCounts == maxVelIt);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene attribute tests
TEST_CASE("Scene Attribute Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());

    SUBCASE("External Forces Every Iteration")
    {
        physxSceneAPI.GetEnableExternalForcesEveryIterationAttr().Set(true);

        physxSim->attachStage(stageId);

        PxScene* scenePtr = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(scenePtr != nullptr);

        PxSceneFlags sceneFlags = scenePtr->getFlags();
        CHECK(sceneFlags.isSet(PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS));
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Multiple Scenes
TEST_CASE("Multiple Scenes",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScene0Path = defaultPrimPath.AppendChild(TfToken("physicsScene0"));
    UsdPhysicsScene::Define(stage, physicsScene0Path);

    const SdfPath physicsScene1Path = defaultPrimPath.AppendChild(TfToken("physicsScene1"));
    UsdPhysicsScene::Define(stage, physicsScene1Path);

    SUBCASE("RigidBody")
    {
        const SdfPath cubePath0 = defaultPrimPath.AppendChild(TfToken("cubePrim0"));
        UsdGeomCube cube0 = UsdGeomCube::Define(stage, cubePath0);
        {            
            UsdPhysicsRigidBodyAPI rigidBodyAPI = UsdPhysicsRigidBodyAPI::Apply(cube0.GetPrim());
            SdfPathVector targets;
            targets.push_back(physicsScene0Path);
            rigidBodyAPI.GetSimulationOwnerRel().SetTargets(targets);
        }
        const SdfPath cubePath1 = defaultPrimPath.AppendChild(TfToken("cubePrim1"));
        UsdGeomCube cube1 = UsdGeomCube::Define(stage, cubePath1);
        {            
            UsdPhysicsRigidBodyAPI rigidBodyAPI = UsdPhysicsRigidBodyAPI::Apply(cube1.GetPrim());
            SdfPathVector targets;
            targets.push_back(physicsScene1Path);
            rigidBodyAPI.GetSimulationOwnerRel().SetTargets(targets);
        }

        SUBCASE("RigidBody Only")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
            REQUIRE(basePtr != nullptr);            
            PxScene* scene0 = (PxScene*)basePtr;
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene1Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene1 = (PxScene*)basePtr;

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath0, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* rbo = basePtr->is<PxRigidDynamic>();
            CHECK(scene0 == rbo->getScene());

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath1, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            rbo = basePtr->is<PxRigidDynamic>();
            CHECK(scene1 == rbo->getScene());
        }

        {
            UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Apply(cube0.GetPrim());
            SdfPathVector targets;
            targets.push_back(physicsScene0Path);
            collisionAPI.GetSimulationOwnerRel().SetTargets(targets);
        }
        {
            UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Apply(cube1.GetPrim());
            SdfPathVector targets;
            targets.push_back(physicsScene1Path);
            collisionAPI.GetSimulationOwnerRel().SetTargets(targets);
        }

        SUBCASE("RigidBody With Colliders")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene0 = (PxScene*)basePtr;
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene1Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene1 = (PxScene*)basePtr;

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath0, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* rbo = basePtr->is<PxRigidDynamic>();
            CHECK(scene0 == rbo->getScene());

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath0, ePTShape));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxShape>());
            PxShape* shape = basePtr->is<PxShape>();
            CHECK(shape->getActor() == rbo);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath1, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            rbo = basePtr->is<PxRigidDynamic>();
            CHECK(scene1 == rbo->getScene());

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath1, ePTShape));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxShape>());
            shape = basePtr->is<PxShape>();
            CHECK(shape->getActor() == rbo);
        }

        SUBCASE("RigidBody With Colliders Multiple Scenes Shared")
        {
            {
                SdfPathVector targets;
                targets.push_back(physicsScene0Path);
                targets.push_back(physicsScene1Path);
                {
                    UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Apply(cube0.GetPrim());
                    rboAPI.GetSimulationOwnerRel().SetTargets(targets);
                }
                {
                    UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Apply(cube1.GetPrim());
                    rboAPI.GetSimulationOwnerRel().SetTargets(targets);
                }
            }

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene0 = (PxScene*)basePtr;
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene1Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene1 = (PxScene*)basePtr;

            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_STATIC) == 0);
            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 2);

            CHECK(scene1->getNbActors(PxActorTypeFlag::eRIGID_STATIC) == 0);
            CHECK(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 2);

            PxActor* actors[4];
            scene0->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actors[0], 2);
            scene1->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actors[2], 2);
            for (int i = 0; i < 4; i++)
            {
                PxRigidDynamic* rbo = actors[i]->is<PxRigidDynamic>();
                REQUIRE(rbo);
                REQUIRE(rbo->getNbShapes() == 1);
                PxShape* shape = nullptr;
                rbo->getShapes(&shape, 1);
                CHECK(shape->getGeometry().getType() == PxGeometryType::eBOX);
            }
        }
    }

    SUBCASE("Static Body")
    {
        const SdfPath cubePath0 = defaultPrimPath.AppendChild(TfToken("cubePrim0"));
        UsdGeomCube cube0 = UsdGeomCube::Define(stage, cubePath0);
        {
            UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Apply(cube0.GetPrim());
            SdfPathVector targets;
            targets.push_back(physicsScene0Path);
            collisionAPI.GetSimulationOwnerRel().SetTargets(targets);
        }
        const SdfPath cubePath1 = defaultPrimPath.AppendChild(TfToken("cubePrim1"));
        UsdGeomCube cube1 = UsdGeomCube::Define(stage, cubePath1);
        {
            UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Apply(cube1.GetPrim());
            SdfPathVector targets;
            targets.push_back(physicsScene1Path);
            collisionAPI.GetSimulationOwnerRel().SetTargets(targets);
        }

        SUBCASE("Colliders")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene0 = (PxScene*)basePtr;
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene1Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene1 = (PxScene*)basePtr;

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath0, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidStatic>());

            PxRigidStatic* rbo = basePtr->is<PxRigidStatic>();
            CHECK(scene0 == rbo->getScene());

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath0, ePTShape));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxShape>());
            PxShape* shape = basePtr->is<PxShape>();
            CHECK(shape->getActor() == rbo);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath1, ePTActor));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidStatic>());

            rbo = basePtr->is<PxRigidStatic>();
            CHECK(scene1 == rbo->getScene());

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath1, ePTShape));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxShape>());
            shape = basePtr->is<PxShape>();
            CHECK(shape->getActor() == rbo);
        }

        SUBCASE("Colliders Multiple Scenes Shared")
        {
            {
                SdfPathVector targets;
                targets.push_back(physicsScene0Path);
                targets.push_back(physicsScene1Path);
                {
                    UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Apply(cube0.GetPrim());
                    collisionAPI.GetSimulationOwnerRel().SetTargets(targets);
                }
                {
                    UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Apply(cube1.GetPrim());
                    collisionAPI.GetSimulationOwnerRel().SetTargets(targets);
                }
            }

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // scene should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene0 = (PxScene*)basePtr;
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene1Path, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene1 = (PxScene*)basePtr;

            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_STATIC) == 2);
            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

            CHECK(scene1->getNbActors(PxActorTypeFlag::eRIGID_STATIC) == 2);
            CHECK(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

            PxActor* actors[4];
            scene0->getActors(PxActorTypeFlag::eRIGID_STATIC, &actors[0], 2);
            scene1->getActors(PxActorTypeFlag::eRIGID_STATIC, &actors[2], 2);
            for (int i = 0; i < 4; i++)
            {
                PxRigidStatic* rbo = actors[i]->is<PxRigidStatic>();
                REQUIRE(rbo);
                REQUIRE(rbo->getNbShapes() == 1);
                PxShape* shape = nullptr;
                rbo->getShapes(&shape, 1);
                CHECK(shape->getGeometry().getType() == PxGeometryType::eBOX);
            }
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Scene stepping tests for kinematics
TEST_CASE("Scene Kinematics Stepping Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);    

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());

    const SdfPath cubePath = defaultPrimPath.AppendChild(TfToken("cube"));
    UsdGeomCube cube = UsdGeomCube::Define(stage, cubePath);
    UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Apply(cube.GetPrim());
    UsdPhysicsCollisionAPI::Apply(cube.GetPrim());
    rboAPI.CreateKinematicEnabledAttr().Set(true);
    UsdGeomXformOp traslate = cube.AddTranslateOp();
    const int numSteps = 10;
    const double endTime = 1.0/60.0 * double(numSteps);
    traslate.Set(GfVec3d(0.0), UsdTimeCode(0.0));
    traslate.Set(GfVec3d(10.0), UsdTimeCode(endTime * stage->GetTimeCodesPerSecond()));
    traslate.Set(GfVec3d(20.0), UsdTimeCode(endTime * 2.0 * stage->GetTimeCodesPerSecond()));

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Single Simulation Step")
    {
        uint32_t timeStepsPerSeconds;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Get(&timeStepsPerSeconds);
        // check default value
        CHECK(timeStepsPerSeconds == 60);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        PxRigidBody* rbo = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath, ePTActor));
        REQUIRE(rbo);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < numSteps + 1; i++)
            physx->updateSimulation(timeStep, i * timeStep);

        const PxVec3 rboPos = rbo->getGlobalPose().p;
        compare(rboPos, GfVec3f(10.0f), epsilon);
    }

    SUBCASE("Two Simulation Step")
    {
        const uint32_t timeStepsPerSeconds = 120;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSeconds);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        PxRigidBody* rbo = reinterpret_cast<PxRigidBody*>(physx->getPhysXPtr(cubePath, ePTActor));
        REQUIRE(rbo);

        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < numSteps + 1; i++)
            physx->updateSimulation(timeStep, i * timeStep);

        const PxVec3 rboPos = rbo->getGlobalPose().p;
        compare(rboPos, GfVec3f(10.0f), epsilon);
    }

    SUBCASE("Multiple Simulation Step")
    {
        const uint32_t timeStepsPerSeconds = 240;
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSeconds);

        // scene should be there
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        PxRigidBody* rbo = reinterpret_cast<PxRigidBody*>(physx->getPhysXPtr(cubePath, ePTActor));
        REQUIRE(rbo);

        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < numSteps + 1; i++)
            physx->updateSimulation(timeStep, i * timeStep);

        const PxVec3 rboPos = rbo->getGlobalPose().p;
        compare(rboPos, GfVec3f(10.0f), epsilon);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


//-----------------------------------------------------------------------------
// Quasistatic mode
TEST_CASE("Scene Quasistatic Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);    

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
    PhysxSchemaPhysxSceneQuasistaticAPI quasistaticAPI = PhysxSchemaPhysxSceneQuasistaticAPI::Apply(scene.GetPrim());

    const SdfPath cubePath0 = defaultPrimPath.AppendChild(TfToken("cube0"));
    UsdGeomCube cube0 = UsdGeomCube::Define(stage, cubePath0);
    UsdPhysicsRigidBodyAPI::Apply(cube0.GetPrim());
    UsdPhysicsCollisionAPI::Apply(cube0.GetPrim());
    cube0.AddTranslateOp().Set(GfVec3d(-10.0f, 0.0f, 0.0f));

    const SdfPath cubePath1 = defaultPrimPath.AppendChild(TfToken("cube1"));
    UsdGeomCube cube1 = UsdGeomCube::Define(stage, cubePath1);
    UsdPhysicsRigidBodyAPI::Apply(cube1.GetPrim());
    UsdPhysicsCollisionAPI::Apply(cube1.GetPrim());
    cube1.AddTranslateOp().Set(GfVec3d(10.0f, 0.0f, 0.0f));

    SUBCASE("Default Settings Stepping")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxRigidBody* rbo0 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath0, ePTActor));
        REQUIRE(rbo0);
        PxRigidBody* rbo1 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath1, ePTActor));
        REQUIRE(rbo1);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }                     
    }

    SUBCASE("Enable Switch Change")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxRigidBody* rbo0 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath0, ePTActor));
        REQUIRE(rbo0);
        PxRigidBody* rbo1 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath1, ePTActor));
        REQUIRE(rbo1);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }

        quasistaticAPI.GetEnableQuasistaticAttr().Set(false);
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            CHECK(rbo0->getLinearVelocity().z < -epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            CHECK(rbo1->getLinearVelocity().z < -epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }

        quasistaticAPI.GetEnableQuasistaticAttr().Set(true);
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }

    }

    SUBCASE("Collection Enabled Stepping")
    {
        quasistaticAPI.GetQuasistaticActorsCollectionAPI().GetIncludesRel().AddTarget(defaultPrimPath);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxRigidBody* rbo0 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath0, ePTActor));
        REQUIRE(rbo0);
        PxRigidBody* rbo1 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath1, ePTActor));
        REQUIRE(rbo1);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }                     
    }

    SUBCASE("Collection Subset Enabled Stepping")
    {
        quasistaticAPI.GetQuasistaticActorsCollectionAPI().GetIncludesRel().AddTarget(defaultPrimPath);
        quasistaticAPI.GetQuasistaticActorsCollectionAPI().GetExcludesRel().AddTarget(cubePath1);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxRigidBody* rbo0 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath0, ePTActor));
        REQUIRE(rbo0);
        PxRigidBody* rbo1 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath1, ePTActor));
        REQUIRE(rbo1);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            CHECK(rbo1->getLinearVelocity().z < -epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }                     
    }

    SUBCASE("Collection Enabled Runtime Include")
    {       
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxRigidBody* rbo0 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath0, ePTActor));
        REQUIRE(rbo0);
        PxRigidBody* rbo1 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath1, ePTActor));
        REQUIRE(rbo1);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }

        quasistaticAPI.GetQuasistaticActorsCollectionAPI().GetIncludesRel().AddTarget(cubePath0);
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            CHECK(rbo1->getLinearVelocity().z < -epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }
    }

    SUBCASE("Collection Enabled Runtime Exclude")
    {
        quasistaticAPI.GetQuasistaticActorsCollectionAPI().GetIncludesRel().AddTarget(defaultPrimPath);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxRigidBody* rbo0 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath0, ePTActor));
        REQUIRE(rbo0);
        PxRigidBody* rbo1 = reinterpret_cast<PxRigidBody *>(physx->getPhysXPtr(cubePath1, ePTActor));
        REQUIRE(rbo1);

        // step with exact step frequency
        const float timeStep = 1.0f / 60.0f;
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }

        quasistaticAPI.GetQuasistaticActorsCollectionAPI().GetExcludesRel().AddTarget(cubePath1);
        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(timeStep, i*timeStep);
            physxSim->fetchResults();
            compare(rbo0->getLinearVelocity(), PxVec3(PxZero), epsilon);
            compare(rbo0->getAngularVelocity(), PxVec3(PxZero), epsilon);
            CHECK(rbo1->getLinearVelocity().z < -epsilon);
            compare(rbo1->getAngularVelocity(), PxVec3(PxZero), epsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

void checkResidualsNonZero(PhysxSchemaPhysxResidualReportingAPI& api, bool includeVelocityResiduals = true)
{
    float r0, r1, r2, r3;
    REQUIRE(api.GetPhysxResidualReportingRmsResidualPositionIterationAttr().Get(&r0));
    REQUIRE(api.GetPhysxResidualReportingMaxResidualPositionIterationAttr().Get(&r1));
    REQUIRE(api.GetPhysxResidualReportingRmsResidualVelocityIterationAttr().Get(&r2));
    REQUIRE(api.GetPhysxResidualReportingMaxResidualVelocityIterationAttr().Get(&r3));

    REQUIRE(r0 != 0.0f);
    REQUIRE(r1 != 0.0f);
    if (includeVelocityResiduals)
    {
        REQUIRE(r2 != 0.0f);
        REQUIRE(r3 != 0.0f);
    }
}

//-----------------------------------------------------------------------------
// Quasistatic mode
TEST_CASE("Residual Reporting Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=twidmer][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SUBCASE("ResidualsNonZero")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "ResidualReportingTestScene.usda";

        UsdStageRefPtr stage = UsdStage::Open(usdFileName);
        REQUIRE(stage);

        pxr::UsdUtilsStageCache::Get().Insert(stage);
        long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        physxSim->attachStage(stageId);

        // run the sim
        for (int i = 0; i < 10; ++i)
        {
            physxSim->simulate(1.0f / 60.0f, 0.0f);
            physxSim->fetchResults();
        }

        const SdfPath defaultPrimPath = SdfPath("/World");
        const SdfPath jointPath = defaultPrimPath.AppendChild(TfToken("Joint")).AppendChild(TfToken("Cube_01")).AppendChild(TfToken("SphericalJoint"));
        const SdfPath articulationPath = defaultPrimPath.AppendChild(TfToken("Articulation"));
        const SdfPath scenePath = defaultPrimPath.AppendChild(TfToken("PhysicsScene"));

        {
            auto prim = stage->GetPrimAtPath(jointPath);
            REQUIRE(prim.HasAPI<PhysxSchemaPhysxResidualReportingAPI>());
            PhysxSchemaPhysxResidualReportingAPI residualAPI(prim);
            checkResidualsNonZero(residualAPI);
        }
        {
            auto prim = stage->GetPrimAtPath(scenePath);
            REQUIRE(prim.HasAPI<PhysxSchemaPhysxResidualReportingAPI>());
            PhysxSchemaPhysxResidualReportingAPI residualAPI(prim);
            checkResidualsNonZero(residualAPI);
        }
        {
            auto prim = stage->GetPrimAtPath(articulationPath);
            REQUIRE(prim.HasAPI<PhysxSchemaPhysxResidualReportingAPI>());
            PhysxSchemaPhysxResidualReportingAPI residualAPI(prim);
            checkResidualsNonZero(residualAPI, false);
        }
    }
}

TEST_CASE("Scene Quasistatic Mode",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = UsdGeomXform::Define(stage, defaultPrimPath).GetPrim();
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // create scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    PhysxSchemaPhysxSceneQuasistaticAPI::Apply(scene.GetPrim());

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);
    const SdfPath boxPath("/World/box");


    SUBCASE("RigidBody USD Codepath")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxRigidDynamic* rbo = basePtr->is<PxRigidDynamic>();

        for (size_t i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 0.0f);
            physxSim->fetchResults();
            const PxVec3 linVelocity = rbo->getLinearVelocity();
            const PxVec3 angVelocity = rbo->getAngularVelocity();

            compare(linVelocity, GfVec3f(0.0f), epsilon);
            compare(angVelocity, GfVec3f(0.0f), epsilon);
        }
    }

    SUBCASE("RigidBody Fabric Codepath")
    {
        ScopedFabricActivation fabricEnable;

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);
        fabricEnable.mIPhysxFabric->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxRigidDynamic* rbo = basePtr->is<PxRigidDynamic>();

        for (size_t i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60.0f, 0.0f);

            const PxVec3 linVelocity = rbo->getLinearVelocity();
            const PxVec3 angVelocity = rbo->getAngularVelocity();

            compare(linVelocity, GfVec3f(0.0f), epsilon);
            compare(angVelocity, GfVec3f(0.0f), epsilon);
        }

        fabricEnable.mIPhysxFabric->detachStage();
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
