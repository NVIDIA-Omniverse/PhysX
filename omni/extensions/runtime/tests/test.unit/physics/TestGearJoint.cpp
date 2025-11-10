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

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// gear tests
TEST_CASE_TEMPLATE("Gear Joint Tests", T, USDChange, FabricChange)
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

    // create rigid body 0 with a box collider --- gear
    addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath0 = SdfPath("/World/box0");
    UsdPrim boxPrim0 = stage->GetPrimAtPath(boxPath0);

    const SdfPath revoluteJointPath0 = SdfPath("/World/revoluteJoint0");
    UsdPhysicsRevoluteJoint revoluteJoint0 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath0);
    revoluteJoint0.GetBody1Rel().AddTarget(boxPath0);
    revoluteJoint0.GetAxisAttr().Set(UsdPhysicsTokens->y);

    // create rigid body 0 with a box collider --- rack
    addRigidBox(stage, "/World/box1", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 200.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath1 = SdfPath("/World/box1");
    UsdPrim boxPrim1 = stage->GetPrimAtPath(boxPath1);

    const SdfPath revoluteJointPath1 = SdfPath("/World/revoluteJoint1");
    UsdPhysicsRevoluteJoint revoluteJoint1 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath1);
    revoluteJoint1.GetBody1Rel().AddTarget(boxPath1);
    revoluteJoint1.GetAxisAttr().Set(UsdPhysicsTokens->y);
    revoluteJoint1.CreateLocalPos0Attr().Set(GfVec3f(0.0f, 0.0f, 200.0f));

    // rack joint
    const SdfPath gearJointPath = SdfPath("/World/gearJoint");
    PhysxSchemaPhysxPhysicsGearJoint gearJoint = PhysxSchemaPhysxPhysicsGearJoint::Define(stage, gearJointPath);
    gearJoint.CreateBody0Rel().AddTarget(boxPath0);
    gearJoint.CreateBody1Rel().AddTarget(boxPath1);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    SUBCASE("Base Parse Test")
    {
        gearJoint.CreateHinge0Rel().AddTarget(revoluteJointPath0);
        gearJoint.CreateHinge1Rel().AddTarget(revoluteJointPath1);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath0, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath1, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        // Rack and pinion joint should be there
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(gearJointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxJoint>());
        CHECK(strstr(basePtr->is<PxJoint>()->getConcreteTypeName(), "PxGearJoint"));
    }

    SUBCASE("Hinge Rel Test")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath0, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());
        PxJoint* revolute0 = basePtr->is<PxD6Joint>();

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath1, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());
        PxJoint* revolute1 = basePtr->is<PxD6Joint>();

        // Rack and pinion joint should be there
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(gearJointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxJoint>());
        REQUIRE(basePtr->is<PxGearJoint>());

        PxGearJoint* gj = basePtr->is<PxGearJoint>();
        const PxBase* joint0 = nullptr;
        const PxBase* joint1 = nullptr;
        gj->getHinges(joint0, joint1);
        CHECK(joint0 == nullptr);
        CHECK(joint1 == nullptr);

        gearJoint.CreateHinge0Rel().AddTarget(revoluteJointPath0);
        gearJoint.CreateHinge1Rel().AddTarget(revoluteJointPath1);

        gj->getHinges(joint0, joint1);
        CHECK(joint0 == revolute0);
        CHECK(joint1 == revolute1);
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Gear Joint Order Tests",
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

    // create rigid body 0 with a box collider --- gear
    addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath0 = SdfPath("/World/box0");
    UsdPrim boxPrim0 = stage->GetPrimAtPath(boxPath0);

    // create rigid body 0 with a box collider --- rack
    addRigidBox(stage, "/World/box1", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 200.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath1 = SdfPath("/World/box1");
    UsdPrim boxPrim1 = stage->GetPrimAtPath(boxPath1);

    const SdfPath revoluteJointPath0 = SdfPath("/World/revoluteJoint0");
    const SdfPath revoluteJointPath1 = SdfPath("/World/revoluteJoint1");

    const SdfPath gearJointPath = SdfPath("/World/agearJoint");
    PhysxSchemaPhysxPhysicsGearJoint gearJoint = PhysxSchemaPhysxPhysicsGearJoint::Define(stage, gearJointPath);
    gearJoint.CreateBody0Rel().AddTarget(boxPath0);
    gearJoint.CreateBody1Rel().AddTarget(boxPath1);
    gearJoint.CreateHinge0Rel().AddTarget(revoluteJointPath0);
    gearJoint.CreateHinge1Rel().AddTarget(revoluteJointPath1);

    
    UsdPhysicsRevoluteJoint revoluteJoint0 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath0);
    revoluteJoint0.GetBody1Rel().AddTarget(boxPath0);
    revoluteJoint0.GetAxisAttr().Set(UsdPhysicsTokens->y);

    
    UsdPhysicsRevoluteJoint revoluteJoint1 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath1);
    revoluteJoint1.GetBody1Rel().AddTarget(boxPath1);
    revoluteJoint1.GetAxisAttr().Set(UsdPhysicsTokens->y);
    revoluteJoint1.CreateLocalPos0Attr().Set(GfVec3f(0.0f, 0.0f, 200.0f));

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Base Parse Test")
    {
        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath0, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath1, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        // gear joint should be there
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(gearJointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxJoint>());
        CHECK(basePtr->is<PxGearJoint>());
    }

    SUBCASE("Gear Ratio")
    {
        float value = 0.0f;
        gearJoint.GetGearRatioAttr().Get(&value);

        // check default value
        CHECK(fabsf(value - 1.0f) < epsilon);

        // gear joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(gearJointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxJoint>());
        CHECK(basePtr->is<PxGearJoint>());
        
        PxGearJoint* physxJoint = (PxGearJoint*)basePtr;
        value = physxJoint->getGearRatio();
        CHECK(fabsf(value - 1.0f) < epsilon);

        // change value
        value = 100.0f;
        gearJoint.GetGearRatioAttr().Set(value);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(gearJointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        float gearRatio = physxJoint->getGearRatio();
        CHECK(fabsf(gearRatio - value) < epsilon);

        value = -100.0f;
        gearJoint.GetGearRatioAttr().Set(value);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        
        gearRatio = physxJoint->getGearRatio();
        CHECK(fabsf(gearRatio - value) < epsilon);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


TEST_CASE("Gear Joint Articulation Tests",
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

    // create rigid body 0 with a box collider --- gear
    addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath0 = SdfPath("/World/box0");
    UsdPrim boxPrim0 = stage->GetPrimAtPath(boxPath0);

    // create rigid body 0 with a box collider --- rack
    addRigidBox(stage, "/World/box1", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 200.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath1 = SdfPath("/World/box1");
    UsdPrim boxPrim1 = stage->GetPrimAtPath(boxPath1);

    // create rigid body 0 with a box collider --- rack
    addRigidBox(stage, "/World/box2", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 400.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath2 = SdfPath("/World/box2");
    UsdPrim boxPrim2 = stage->GetPrimAtPath(boxPath2);

    const SdfPath fixedJointPath = SdfPath("/World/fixedJoint");
    const SdfPath revoluteJointPath0 = SdfPath("/World/revoluteJoint0");
    const SdfPath revoluteJointPath1 = SdfPath("/World/revoluteJoint1");

    const SdfPath gearJointPath = SdfPath("/World/agearJoint");
    PhysxSchemaPhysxPhysicsGearJoint gearJoint = PhysxSchemaPhysxPhysicsGearJoint::Define(stage, gearJointPath);
    gearJoint.CreateBody0Rel().AddTarget(boxPath1);
    gearJoint.CreateBody1Rel().AddTarget(boxPath2);
    gearJoint.CreateHinge0Rel().AddTarget(revoluteJointPath0);
    gearJoint.CreateHinge1Rel().AddTarget(revoluteJointPath1);
    gearJoint.CreateExcludeFromArticulationAttr().Set(true);

    UsdPhysicsFixedJoint fixedJoint = UsdPhysicsFixedJoint::Define(stage, fixedJointPath);
    fixedJoint.GetBody0Rel().AddTarget(boxPath0);
    UsdPhysicsArticulationRootAPI::Apply(fixedJoint.GetPrim());

    UsdPhysicsRevoluteJoint revoluteJoint0 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath0);
    revoluteJoint0.GetBody0Rel().AddTarget(boxPath0);
    revoluteJoint0.GetBody1Rel().AddTarget(boxPath1);
    revoluteJoint0.GetAxisAttr().Set(UsdPhysicsTokens->y);


    UsdPhysicsRevoluteJoint revoluteJoint1 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath1);
    revoluteJoint1.GetBody0Rel().AddTarget(boxPath1);
    revoluteJoint1.GetBody1Rel().AddTarget(boxPath2);
    revoluteJoint1.GetAxisAttr().Set(UsdPhysicsTokens->y);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Base Parse Test")
    {
        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath0, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationJointReducedCoordinate>());

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath1, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationJointReducedCoordinate>());

        // gear joint should be there
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(gearJointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxJoint>());        
        CHECK(basePtr->is<PxGearJoint>());
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
