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
#include <omni/physx/PhysxTokens.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;


//-----------------------------------------------------------------------------
// Articulation Revolute joint tests
TEST_CASE_TEMPLATE("Articulation Revolute Joint Tests", T, USDChange, FabricChange)
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

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath articulationPrimPath = SdfPath("/World");
    UsdGeomXform articulationPrim = UsdGeomXform::Define(stage, articulationPrimPath);
    UsdPhysicsArticulationRootAPI::Apply(articulationPrim.GetPrim());

    const SdfPath physicsScenePath = articulationPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body - rootLink
    addRigidBox(stage, "/World/rootLink", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath rootLinkPath = SdfPath("/World/rootLink");
    UsdPrim rootLinkPrim = stage->GetPrimAtPath(rootLinkPath);

    const SdfPath fixedJointPath = SdfPath("/World/baseFixedJoint");
    UsdPhysicsFixedJoint fixedJoint = UsdPhysicsFixedJoint::Define(stage, fixedJointPath);
    fixedJoint.GetBody1Rel().AddTarget(rootLinkPath);

    // create rigid body - dynamicLink
    addRigidBox(stage, "/World/dynamicLink", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath dynamicLinkPath = SdfPath("/World/dynamicLink");
    UsdPrim dynamicLinkPrim = stage->GetPrimAtPath(dynamicLinkPath);


    const SdfPath revoluteJointPath = SdfPath("/World/revoluteJoint");
    UsdPhysicsRevoluteJoint revoluteJoint = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath);
    revoluteJoint.GetBody0Rel().AddTarget(rootLinkPath);
    revoluteJoint.GetBody1Rel().AddTarget(dynamicLinkPath);
    revoluteJoint.GetLowerLimitAttr().Set(-90.0f);
    revoluteJoint.GetUpperLimitAttr().Set(90.0f);

    PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Apply(revoluteJoint.GetPrim());
    PhysxSchemaPhysxLimitAPI limitAPI = PhysxSchemaPhysxLimitAPI::Apply(revoluteJoint.GetPrim(), TfToken("angular"));
    UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Apply(revoluteJoint.GetPrim(), TfToken("angular"));
    PhysxSchemaJointStateAPI jointStateAPI = PhysxSchemaJointStateAPI::Apply(revoluteJoint.GetPrim(), TfToken("angular"));


    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Base Parse Test")
    {
        physxSim->detachStage();
        // No limits on revolute joints will yield a regular eREVOLUTE joint
        revoluteJoint.GetLowerLimitAttr().Clear();
        revoluteJoint.GetUpperLimitAttr().Clear();
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(articulationPrimPath, ePTArticulation));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationReducedCoordinate>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootLinkPath, ePTLink));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationLink>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(dynamicLinkPath, ePTLink));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationLink>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>()->getJointType() == PxArticulationJointType::eREVOLUTE);
    }

    SUBCASE("Base Parse Test Unwrapped")
    {
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>()->getJointType() == PxArticulationJointType::eREVOLUTE_UNWRAPPED );
    }

    SUBCASE("Lower Limit")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsLowerLimit;

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value + 90.0f) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationLimit limit = joint->getLimitParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(limit.low) + 90.0f) < epsilon);

        // change value
        value = -10.0;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        limit = joint->getLimitParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(limit.low) - value) < epsilon);
    }

    SUBCASE("Upper Limit")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsUpperLimit;

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value - 90.0f) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationLimit limit = joint->getLimitParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(limit.high) - 90.0f) < epsilon);

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        limit = joint->getLimitParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(limit.high) - value) < epsilon);
    }

    SUBCASE("Friction Coefficient")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxJointJointFriction;

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        float friction = joint->getFrictionCoefficient();
        CHECK(fabsf(friction) < epsilon);

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        friction = joint->getFrictionCoefficient();
        CHECK(fabsf(friction - value) < epsilon);
    }

    SUBCASE("Max Joint Velocity")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxJointMaxJointVelocity;

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value - 1000000.0f) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        float maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eTWIST);
        //CHECK(radToDeg(maxVel) >= FLT_MAX);

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(maxVel) - value) < epsilon);
    }

    SUBCASE("Armature")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxJointArmature;

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        float arm = joint->getArmature(PxArticulationAxis::eTWIST);
        CHECK(fabsf(arm) < epsilon);

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        arm = joint->getArmature(PxArticulationAxis::eTWIST);
        CHECK(fabsf(arm - 10.0f) < epsilon);
    }

    SUBCASE("Joint Properties per Axis")
    {
        revoluteJoint.GetPrim().ApplyAPI( PhysxAdditionAPITokens->PhysxJointAxisAPI, TfToken("angular"));
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->armatureAngular, SdfValueTypeNames->Float);
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->maxJointVelocityAngular, SdfValueTypeNames->Float);
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->staticFrictionEffortAngular, SdfValueTypeNames->Float);
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortAngular, SdfValueTypeNames->Float);
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientAngular, SdfValueTypeNames->Float);

        SUBCASE("Max Joint Velocity Per Axis")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->maxJointVelocityAngular;

            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
            CHECK(fabsf(value - 1000000.0f) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            float maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eTWIST);
            CHECK((maxVel- degToRad(1000000.0f)) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eTWIST);
            CHECK(fabsf(radToDeg(maxVel) - value) < epsilon);
        }

        SUBCASE("Armature Per Axis")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->armatureAngular;

            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            float arm = joint->getArmature(PxArticulationAxis::eTWIST);
            CHECK(fabsf(arm) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            arm = joint->getArmature(PxArticulationAxis::eTWIST);
            CHECK(fabsf(arm - 10.0f) < epsilon);
        }

        SUBCASE("Static Friction Effort ")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->staticFrictionEffortAngular;

            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(params.staticFrictionEffort) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(params.staticFrictionEffort - 10.0f) < epsilon);
        }

        SUBCASE("Dynamic Friction Effort ")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->dynamicFrictionEffortAngular;
            const TfToken& changeToken2 = PhysxAdditionAttrTokens->staticFrictionEffortAngular;

            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(params.dynamicFrictionEffort) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken2, value);
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(params.dynamicFrictionEffort - 10.0f) < epsilon);
        }

        SUBCASE("Viscous Friction Coefficient")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->viscousFrictionCoefficientAngular;

            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(degToRad(params.viscousFrictionCoefficient)) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(degToRad(params.viscousFrictionCoefficient) - 10.0f) < epsilon);
        }
    }

    SUBCASE("Drive Type")
    {
        const TfToken changeToken("drive:angular:physics:type");

        TfToken value = changeTemplate.template getAttributeValue<TfToken>(revoluteJointPath, changeToken);
        // check default value
        CHECK(value == UsdPhysicsTokens->force);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(drive.driveType == PxArticulationDriveType::eFORCE);

        // change value
        value = UsdPhysicsTokens->acceleration;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(drive.driveType == PxArticulationDriveType::eACCELERATION);
    }

    SUBCASE("Drive Max Force")
    {
        const TfToken changeToken("drive:angular:physics:maxForce");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(!isfinite(value));

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(drive.maxForce >= FLT_MAX);
        CHECK(fabsf(drive.envelope.maxEffort) < epsilon); // assert 0 max Effort

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(drive.maxForce - value) < epsilon);
        CHECK(fabsf(drive.envelope.maxEffort) < epsilon); // assert 0 max Effort
    }

    SUBCASE("Performance Envelope")
    {
        
        revoluteJoint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, TfToken("angular"));
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityAngular, SdfValueTypeNames->Float);
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceAngular, SdfValueTypeNames->Float);
        revoluteJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->speedEffortGradientAngular, SdfValueTypeNames->Float);

        
        SUBCASE("Drive Max Effort")
        {
            const TfToken changeToken("drive:angular:physics:maxForce");
    
            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
            CHECK(!isfinite(value));
    
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
    
            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
    
            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(drive.envelope.maxEffort >= FLT_MAX);
    
            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();
    
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(drive.envelope.maxEffort - value) < epsilon);
        }

        SUBCASE("Drive Max Actuator Velocity")
        {
            const TfToken changeToken = PhysxAdditionAttrTokens->maxActuatorVelocityAngular;
    
            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
    
            CHECK(!isfinite(value));
    
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
    
            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
    
            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(drive.envelope.maxActuatorVelocity  >= FLT_MAX);
    
            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();
    
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(radToDeg(drive.envelope.maxActuatorVelocity) - value) < epsilon);
        }
    
        SUBCASE("Drive Velocity Dependent Resistance")
        {
            const TfToken changeToken = PhysxAdditionAttrTokens->velocityDependentResistanceAngular;
    
            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
    
            CHECK(fabsf(value) < epsilon);
    
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
    
            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
    
            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(drive.envelope.velocityDependentResistance) < epsilon);
    
            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();
    
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(degToRad(drive.envelope.velocityDependentResistance) - value) < epsilon);
        }
    
        SUBCASE("Drive Speed Effort Gradient")
        {
            const TfToken changeToken = PhysxAdditionAttrTokens->speedEffortGradientAngular;
    
            float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
            // check default value
    
            CHECK(fabsf(value) < epsilon);
    
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
    
            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
    
            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(drive.envelope.speedEffortGradient) < epsilon);
    
            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();
    
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
            CHECK(fabsf(radToDeg(drive.envelope.speedEffortGradient) - value) < epsilon);
        }
    }
    
    SUBCASE("Drive Target Position")
    {
        const TfToken changeToken("drive:angular:physics:targetPosition");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
        
        float drive = joint->getDriveTarget(PxArticulationAxis::eTWIST);        
        CHECK(fabsf(drive) < epsilon);

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveTarget(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(drive) - value) < epsilon);
    }

    SUBCASE("Drive Target Velocity")
    {
        const TfToken changeToken("drive:angular:physics:targetVelocity");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        float drive = joint->getDriveVelocity(PxArticulationAxis::eTWIST);
        CHECK(fabsf(drive) < epsilon);

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveVelocity(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(drive) - value) < epsilon);
    }

    SUBCASE("Drive Damping")
    {
        const TfToken changeToken("drive:angular:physics:damping");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        value = 10.0f;
        driveAPI.CreateDampingAttr().Set(value);

        // reattach
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(drive.damping - radToDeg(value)) < epsilon);  // damping is ~ 1/deg

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(drive.damping - radToDeg(value)) < epsilon);
    }

    SUBCASE("Drive Stiffness")
    {
        const TfToken changeToken("drive:angular:physics:stiffness");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        value = 10.0f;
        driveAPI.CreateStiffnessAttr().Set(value);

        // reattach
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(drive.stiffness - radToDeg(value)) < epsilon);  // stiffness is ~ 1/deg

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
        CHECK(fabsf(drive.stiffness - radToDeg(value)) < epsilon);
    }

    SUBCASE("Joint State Position")
    {
        const TfToken changeToken("state:angular:physics:position");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
        
        float position = joint->getJointPosition(PxArticulationAxis::eTWIST);        
        CHECK(fabsf(position) < epsilon);

        // change value
        value = 45.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        position = joint->getJointPosition(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(position) - value) < 0.1f);
    }

    SUBCASE("Joint State Velocity")
    {
        const TfToken changeToken("state:angular:physics:velocity");

        float value = changeTemplate.template getAttributeValue<float>(revoluteJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
        
        float velocity = joint->getJointVelocity(PxArticulationAxis::eTWIST);        
        CHECK(fabsf(velocity) < epsilon);

        // change value
        value = 45.0f;
        changeTemplate.setAttributeValue(revoluteJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(revoluteJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        velocity = joint->getJointVelocity(PxArticulationAxis::eTWIST);
        CHECK(fabsf(radToDeg(velocity) - value) < 0.1f);
    }
    SUBCASE("OM-77331 Joint State update should not crash when prim is removed during simulation")
    {
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        stage->RemovePrim(revoluteJointPath);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
    }
    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
