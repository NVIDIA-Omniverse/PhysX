// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"
#include "../../common/PhysicsChangeTemplate.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysxSimulation.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Distance joint tests
TEST_CASE_TEMPLATE("Distance Joint Tests",T, USDChange, FabricChange)
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

    // create rigid body with a box collider
    const SdfPath boxPath = SdfPath("/World/box");    
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath jointPath = SdfPath("/World/distanceJoint");
    UsdPhysicsDistanceJoint joint = UsdPhysicsDistanceJoint::Define(stage, jointPath);    
    joint.GetBody1Rel().AddTarget(boxPath);
    PhysxSchemaPhysxPhysicsDistanceJointAPI distanceJointAPI = PhysxSchemaPhysxPhysicsDistanceJointAPI::Apply(joint.GetPrim());

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    SUBCASE("Spring Enabled")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        const TfToken& changeToken = PhysxSchemaTokens->physxPhysicsDistanceJointSpringEnabled;
        bool enabled = changeTemplate.template getAttributeValue<bool>(jointPath, changeToken);
        // Default value
        CHECK(enabled == false);

        // Distance joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxDistanceJoint>());

        PxDistanceJoint* distanceJoint = basePtr->is<PxDistanceJoint>();
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eSPRING_ENABLED) == false);

        changeTemplate.setAttributeValue(jointPath, changeToken, true);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eSPRING_ENABLED) == true);
    }

    SUBCASE("Spring Stiffness")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxPhysicsDistanceJointSpringStiffness;

        distanceJointAPI.GetSpringEnabledAttr().Set(true);
        distanceJointAPI.GetSpringStiffnessAttr().Set(10.0f);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // Distance joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxDistanceJoint>());

        PxDistanceJoint* distanceJoint = basePtr->is<PxDistanceJoint>();
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eSPRING_ENABLED) == true);
        CHECK(distanceJoint->getStiffness() == 10.0f);

        changeTemplate.setAttributeValue(jointPath, changeToken, 20.0f);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK(distanceJoint->getStiffness() == 20.0f);
    }

    SUBCASE("Spring Damping")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxPhysicsDistanceJointSpringDamping;

        distanceJointAPI.GetSpringEnabledAttr().Set(true);
        distanceJointAPI.GetSpringDampingAttr().Set(10.0f);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // Distance joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxDistanceJoint>());

        PxDistanceJoint* distanceJoint = basePtr->is<PxDistanceJoint>();
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eSPRING_ENABLED) == true);
        CHECK(distanceJoint->getDamping() == 10.0f);

        changeTemplate.setAttributeValue(jointPath, changeToken, 20.0f);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK(distanceJoint->getDamping() == 20.0f);
    }

    SUBCASE("Min Distance")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsMinDistance;

        joint.GetMinDistanceAttr().Set(0.0f);
        joint.GetMaxDistanceAttr().Set(10.0f);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // Distance joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxDistanceJoint>());

        PxDistanceJoint* distanceJoint = basePtr->is<PxDistanceJoint>();
        CHECK(distanceJoint->getMinDistance() == 0.0f);
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eMIN_DISTANCE_ENABLED) == true);

        changeTemplate.setAttributeValue(jointPath, changeToken, 2.0f);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK(distanceJoint->getMinDistance() == 2.0f);
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eMIN_DISTANCE_ENABLED) == true);

        changeTemplate.setAttributeValue(jointPath, changeToken, -1.0f);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eMIN_DISTANCE_ENABLED) == false);
    }

    SUBCASE("Max Distance")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsMaxDistance;

        joint.GetMinDistanceAttr().Set(0.0f);
        joint.GetMaxDistanceAttr().Set(10.0f);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // Distance joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxDistanceJoint>());

        PxDistanceJoint* distanceJoint = basePtr->is<PxDistanceJoint>();
        CHECK(distanceJoint->getMaxDistance() == 10.0f);
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eMAX_DISTANCE_ENABLED) == true);

        changeTemplate.setAttributeValue(jointPath, changeToken, 20.0f);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK(distanceJoint->getMaxDistance() == 20.0f);
        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eMAX_DISTANCE_ENABLED) == true);

        changeTemplate.setAttributeValue(jointPath, changeToken, -1.0f);
        changeTemplate.broadcastChanges();

        physxSim->simulate(1.0f/60.0f,1.0f/60.0f);
        physxSim->checkResults();

        CHECK((distanceJoint->getDistanceJointFlags() & PxDistanceJointFlag::eMAX_DISTANCE_ENABLED) == false);

    }
    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
