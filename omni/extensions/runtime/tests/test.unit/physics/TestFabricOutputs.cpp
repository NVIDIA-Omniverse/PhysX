// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/physx/IPhysxFabric.h>

#include <omni/fabric/SimStageWithHistory.h>


using namespace pxr;
using namespace omni::physx;
using namespace ::physx;
using namespace carb;
using namespace omni::fabric;
//-----------------------------------------------------------------------------
// fabric joint state

void testArticulationResidualReporting(ScopedFabricActivation& fabricEnable)
{
    FabricChange changeTemplate;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    //Load stage from file
    std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "ResidualReportingTestScene.usda";
    UsdStageRefPtr stage = UsdStage::Open(usdFileName);
    REQUIRE(stage);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // This creates the fabric stage
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);
    fabricEnable.mIPhysxFabric->attachStage(stageId);


    for (int i = 0; i < 10; ++i)
    {
        physxSim->simulate(1.0f / 60, 0.0f);
        physxSim->fetchResults();
        fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
    }

    const TfToken residualRmsPosIterToken = TfToken(PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration.GetText());
    const TfToken residualMaxPosIterToken = TfToken(PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration.GetText());
    const TfToken residualRmsVelIterToken = TfToken(PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration.GetText());
    const TfToken residualMaxVelIterToken = TfToken(PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration.GetText());

    const SdfPath defaultPrimPath = SdfPath("/World");
    const SdfPath jointPath = defaultPrimPath.AppendChild(TfToken("Joint")).AppendChild(TfToken("Cube_01")).AppendChild(TfToken("SphericalJoint"));
    const SdfPath articulationPath = defaultPrimPath.AppendChild(TfToken("Articulation"));
    const SdfPath scenePath = defaultPrimPath.AppendChild(TfToken("PhysicsScene"));

    std::vector<SdfPath> paths;
    paths.push_back(articulationPath);
    paths.push_back(jointPath);
    paths.push_back(scenePath);
    std::vector<TfToken> tokens;
    tokens.push_back(residualRmsVelIterToken);
    tokens.push_back(residualMaxVelIterToken);
    tokens.push_back(residualRmsPosIterToken);
    tokens.push_back(residualMaxPosIterToken);

    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    for (int i = 0; i < paths.size(); ++i)
    {
        for (int j = 0; j < tokens.size(); ++j)
        {
            omni::fabric::StageReaderWriter stage = iSip->get(stageId);
            const omni::fabric::Token token(tokens[j].GetText());
            omni::fabric::Path fabricPath = omni::fabric::Path(omni::fabric::asInt(paths[i]));
            REQUIRE(stage.attributeExists(fabricPath, token));
            const float value = *stage.getAttributeRd<float>(fabricPath, token);

            if (i != 0 || j > 2) //For the articulation, the velocity errors are exactly zero. TODO: Find out why that's the case
                CHECK(value != 0.0f);
        }
    }    

    // Common post-test actions
    fabricEnable.mIPhysxFabric->detachStage();
    physxSim->detachStage();
    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

template <typename JointType, typename TestParameters>
void testArticulationJointState(ScopedFabricActivation& fabricEnable)
{
    TestParameters testParams;

    FabricChange changeTemplate;
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

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath articulationPrimPath = SdfPath("/World");
    UsdGeomXform articulationPrim = UsdGeomXform::Define(stage, articulationPrimPath);
    UsdPhysicsArticulationRootAPI::Apply(articulationPrim.GetPrim());

    const SdfPath physicsScenePath = articulationPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body - rootLink
    addRigidBox(stage, "/World/rootLink", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath rootLinkPath = SdfPath("/World/rootLink");
    UsdPrim rootLinkPrim = stage->GetPrimAtPath(rootLinkPath);

    const SdfPath fixedJointPath = SdfPath("/World/baseFixedJoint");
    UsdPhysicsFixedJoint fixedJoint = UsdPhysicsFixedJoint::Define(stage, fixedJointPath);
    fixedJoint.GetBody1Rel().AddTarget(rootLinkPath);

    // create rigid body - dynamicLink
    addRigidBox(stage, "/World/dynamicLink", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath dynamicLinkPath = SdfPath("/World/dynamicLink");
    UsdPrim dynamicLinkPrim = stage->GetPrimAtPath(dynamicLinkPath);


    const SdfPath jointPath = SdfPath("/World/joint");
    JointType usdJoint = JointType::Define(stage, jointPath);
    usdJoint.GetBody0Rel().AddTarget(rootLinkPath);
    usdJoint.GetBody1Rel().AddTarget(dynamicLinkPath);
    usdJoint.GetLowerLimitAttr().Set(-90.0f);
    usdJoint.GetUpperLimitAttr().Set(90.0f);

    PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Apply(usdJoint.GetPrim());
    PhysxSchemaPhysxLimitAPI limitAPI =
        PhysxSchemaPhysxLimitAPI::Apply(usdJoint.GetPrim(), TfToken(testParams.jointTypeAttributeText));
    PhysxSchemaJointStateAPI jointStateAPI =
        PhysxSchemaJointStateAPI::Apply(usdJoint.GetPrim(), TfToken(testParams.jointTypeAttributeText));

    // This creates the fabric stage
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);
    fabricEnable.mIPhysxFabric->attachStage(stageId);

    const TfToken jointStatePositionToken(testParams.jointStateAttributeText);

    float value = changeTemplate.template getAttributeValue<float>(jointPath, jointStatePositionToken);
    // check default value
    CHECK(fabsf(value) < epsilon);

    value = testParams.startJointStateValue;
    changeTemplate.setAttributeValue(jointPath, jointStatePositionToken, value);
    changeTemplate.broadcastChanges();
    physxSim->simulate(1.0f / 60, 0.0f);
    physxSim->fetchResults();
    fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);

    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    SUBCASE("Position")
    {
        omni::fabric::StageReaderWriter stage = iSip->get(stageId);
        const omni::fabric::Token positionToken(jointStatePositionToken.GetText());
        omni::fabric::Path fabricPath = omni::fabric::Path(omni::fabric::asInt(jointPath));
        REQUIRE(stage.attributeExists(fabricPath, positionToken));
        const float jointState = *stage.getAttributeRd<float>(fabricPath, positionToken);
        CHECK(fabsf(jointState - value) < epsilon);
    }

    SUBCASE("Velocity")
    {
        UsdPhysicsDriveAPI driveAPI =
            UsdPhysicsDriveAPI::Apply(usdJoint.GetPrim(), TfToken(testParams.jointTypeAttributeText));
        driveAPI.GetStiffnessAttr().Set(1000.0f);
        driveAPI.GetDampingAttr().Set(100.0f);
        const TfToken jointDriveToken(testParams.jointDriveAttributeText);
        value = testParams.driveTargetValue;
        changeTemplate.setAttributeValue(jointPath, jointDriveToken, value);
        changeTemplate.broadcastChanges();

        for (int i = 0; i < 5; ++i)
        {
            physxSim->simulate(1.0f / 60, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
        }
        omni::fabric::StageReaderWriter stage = iSip->get(stageId);
        const omni::fabric::Token velocityToken(testParams.jointVelocityAttributeText);
        omni::fabric::Path fabricPath = omni::fabric::Path(omni::fabric::asInt(jointPath));
        REQUIRE(stage.attributeExists(fabricPath, velocityToken));
        const float jointState = *stage.getAttributeRd<float>(fabricPath, velocityToken);
        CHECK(fabsf(jointState - testParams.jointStateVelocityValue) < 0.01f);
    }

    // Common post-test actions
    fabricEnable.mIPhysxFabric->detachStage(); 
    physxSim->detachStage();
    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
struct ArticulationJointStateLinearTestParams
{
    const char* jointTypeAttributeText = "linear";
    const char* jointStateAttributeText = "state:linear:physics:position";
    const char* jointDriveAttributeText = "drive:linear:physics:targetPosition";
    const char* jointVelocityAttributeText = "state:linear:physics:velocity";
    const float startJointStateValue = 10.0f;
    const float driveTargetValue = 20.0f;
    const float jointStateVelocityValue = 0.828f;
};

struct ArticulationJointStateAngularTestParams
{
    const char* jointTypeAttributeText = "angular";
    const char* jointStateAttributeText = "state:angular:physics:position";
    const char* jointDriveAttributeText = "drive:angular:physics:targetPosition";
    const char* jointVelocityAttributeText = "state:angular:physics:velocity";
    const float startJointStateValue = 10.0f;
    const float driveTargetValue = 20.0f;
    const float jointStateVelocityValue = 0.02859f;
};

void testTransformationUpdate(ScopedFabricActivation& fabricEnable)
{
    FabricChange changeTemplate;
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

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // This creates the fabric stage
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    const SdfPath rootPrimPath = SdfPath("/World");
    UsdGeomXform::Define(stage, rootPrimPath);

    const SdfPath physicsScenePath = rootPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    addRigidBox(stage, "/World/body", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath bodyPath = SdfPath("/World/body");
    UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPath);
    CHECK(bodyPrim);

    SUBCASE("Kinematic Body")
    {
        UsdPhysicsRigidBodyAPI rboApi(bodyPrim);
        rboApi.CreateKinematicEnabledAttr().Set(true);

        physxSim->attachStage(stageId);
        fabricEnable.mIPhysxFabric->attachStage(stageId);

        for (int i = 0; i < 5; ++i)
        {
            physxSim->simulate(1.0f / 60, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
        }

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodyPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == true);
        PxRigidDynamic* dynamicBody = basePtr->is<PxRigidDynamic>();

        const PxTransform tr = dynamicBody->getGlobalPose();
        const GfTransform gftr = changeTemplate.getTransformation(bodyPath);

        compare(tr.p, GfVec3f(gftr.GetTranslation()), epsilon);
    }

    SUBCASE("Dynamic Body")
    {
        physxSim->attachStage(stageId);
        fabricEnable.mIPhysxFabric->attachStage(stageId);
        
        for (int i = 0; i < 5; ++i)
        {
            physxSim->simulate(1.0f / 60, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
        }

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodyPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == false);
        PxRigidDynamic* dynamicBody = basePtr->is<PxRigidDynamic>();

        const PxTransform tr = dynamicBody->getGlobalPose();
        const GfTransform gftr = changeTemplate.getTransformation(bodyPath);

        compare(tr.p, GfVec3f(gftr.GetTranslation()), epsilon);
    }

    // Common post-test actions
    fabricEnable.mIPhysxFabric->detachStage(); 
    physxSim->detachStage();
    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Fabric Outputs Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=scristiano][priority=mandatory]")
{
#if !CARB_AARCH64 // OM-111136: Fabric Crash on ARM64 during physics unit test "Fabric Outputs Tests" (TestFabricOutputs.cpp)
    ScopedFabricActivation fabricEnable;
    SUBCASE("Articulation Prismatic Joint State")
    {
        testArticulationJointState<UsdPhysicsPrismaticJoint, ArticulationJointStateLinearTestParams>(fabricEnable);
    }
    SUBCASE("Articulation Angular Joint State")
    {
        testArticulationJointState<UsdPhysicsRevoluteJoint, ArticulationJointStateAngularTestParams>(fabricEnable);
    }
    SUBCASE("ResidualReporting")
    {
        testArticulationResidualReporting(fabricEnable);
    }
    SUBCASE("Transformation Update")
    {
        testTransformationUpdate(fabricEnable);
    }
#endif
}
