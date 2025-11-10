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
// Articulation Prismatic joint tests
TEST_CASE_TEMPLATE("Articulation Prismatic Joint Tests", T, USDChange, FabricChange)
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


    const SdfPath prismaticJointPath = SdfPath("/World/prismaticJoint");
    UsdPhysicsPrismaticJoint prismaticJoint = UsdPhysicsPrismaticJoint::Define(stage, prismaticJointPath);
    prismaticJoint.GetBody0Rel().AddTarget(rootLinkPath);
    prismaticJoint.GetBody1Rel().AddTarget(dynamicLinkPath);
    prismaticJoint.GetLowerLimitAttr().Set(-90.0f);
    prismaticJoint.GetUpperLimitAttr().Set(90.0f);

    PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Apply(prismaticJoint.GetPrim());
    PhysxSchemaPhysxLimitAPI limitAPI = PhysxSchemaPhysxLimitAPI::Apply(prismaticJoint.GetPrim(), TfToken("linear"));
    UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Apply(prismaticJoint.GetPrim(), TfToken("linear"));
    PhysxSchemaJointStateAPI jointStateAPI = PhysxSchemaJointStateAPI::Apply(prismaticJoint.GetPrim(), TfToken("linear"));

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Joint State Position")
    {
        const TfToken changeToken("state:linear:physics:position");

        float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
        
        float position = joint->getJointPosition(PxArticulationAxis::eX);        
        CHECK(fabsf(position) < epsilon);

        // change value
        value = 10.0f;
        changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        position = joint->getJointPosition(PxArticulationAxis::eX);
        CHECK(fabsf(position - value) < epsilon);
    }

    SUBCASE("Joint State Velocity")
    {
        const TfToken changeToken("state:linear:physics:velocity");

        float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
        
        float velocity = joint->getJointVelocity(PxArticulationAxis::eX);        
        CHECK(fabsf(velocity) < epsilon);

        // change value
        value = 5.0f;
        changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        velocity = joint->getJointVelocity(PxArticulationAxis::eX);
        CHECK(fabsf(velocity - value) < epsilon);
    }

    SUBCASE("Max Joint Velocity")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxJointMaxJointVelocity;

        float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
        // check default value
        CHECK(fabsf(value - 1000000.0f) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        float maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eX);
        CHECK(fabsf(maxVel - FLT_MAX) < epsilon );

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eX);
        CHECK(fabsf(maxVel - value) < epsilon);
    }

    SUBCASE("Armature")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxJointArmature;

        float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        float arm = joint->getArmature(PxArticulationAxis::eX);
        CHECK(fabsf(arm) < epsilon);

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        arm = joint->getArmature(PxArticulationAxis::eX);
        CHECK(fabsf(arm - 10.0f) < epsilon);
    }

    SUBCASE("Joint Properties per Axis")
    { 
        prismaticJoint.GetPrim().ApplyAPI( PhysxAdditionAPITokens->PhysxJointAxisAPI, TfToken("linear"));
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->armatureLinear, SdfValueTypeNames->Float);
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->maxJointVelocityLinear, SdfValueTypeNames->Float);
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->staticFrictionEffortLinear, SdfValueTypeNames->Float);
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortLinear, SdfValueTypeNames->Float);
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientLinear, SdfValueTypeNames->Float);

        SUBCASE("Max Joint Velocity Per Axis")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->maxJointVelocityLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(fabsf(value - 1000000.0f) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            float maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eX);
            CHECK(fabsf(maxVel - value) < epsilon );

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            maxVel = joint->getMaxJointVelocity(PxArticulationAxis::eX);
            CHECK(fabsf(maxVel - value) < epsilon);
        }

        SUBCASE("Armature Per Axis")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->armatureLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            float arm = joint->getArmature(PxArticulationAxis::eX);
            CHECK(fabsf(arm) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            arm = joint->getArmature(PxArticulationAxis::eX);
            CHECK(fabsf(arm - 10.0f) < epsilon);
        }

        SUBCASE("Static Friction Effort ")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->staticFrictionEffortLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eX);
            CHECK(fabsf(params.staticFrictionEffort) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            params = joint->getFrictionParams(PxArticulationAxis::eX);
            CHECK(fabsf(params.staticFrictionEffort - 10.0f) < epsilon);
        }

        SUBCASE("Dynamic Friction Effort ")
        {
            const TfToken& changeToken = PhysxAdditionAttrTokens->dynamicFrictionEffortLinear;
            const TfToken& changeToken2 = PhysxAdditionAttrTokens->staticFrictionEffortLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eX);
            CHECK(fabsf(params.dynamicFrictionEffort) < epsilon);

            // change value
            value = 10.0;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken2, value);
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            params = joint->getFrictionParams(PxArticulationAxis::eX);
            CHECK(fabsf(params.dynamicFrictionEffort - 10.0f) < epsilon);
        }

        SUBCASE("Viscous Friction Coefficient")
    {
        const TfToken& changeToken = PhysxAdditionAttrTokens->viscousFrictionCoefficientLinear;

        float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eX);
        CHECK(fabsf(params.viscousFrictionCoefficient) < epsilon);

        // change value
        value = 10.0;
        changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        params = joint->getFrictionParams(PxArticulationAxis::eX);
        CHECK(fabsf(params.viscousFrictionCoefficient - 10.0f) < epsilon);
    }
    }

    SUBCASE("Drive Max Force")
    {
        const TfToken changeToken("drive:linear:physics:maxForce");

        float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
        // check default value
        CHECK(!isfinite(value));

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

        PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eX);
        CHECK(drive.maxForce >= FLT_MAX);
        CHECK(fabsf(drive.envelope.maxEffort) < epsilon);

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        drive = joint->getDriveParams(PxArticulationAxis::eX);
        CHECK(fabsf(drive.maxForce - value) < epsilon);
        CHECK(fabsf(drive.envelope.maxEffort) < epsilon);
    }

    SUBCASE("Performance Envelope")
    {  
        prismaticJoint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, TfToken("linear"));
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityLinear,  SdfValueTypeNames->Float);
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceLinear, SdfValueTypeNames->Float);
        prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->speedEffortGradientLinear, SdfValueTypeNames->Float);

        SUBCASE("Drive Max Effort")
        {
            const TfToken changeToken("drive:linear:physics:maxForce");

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(!isfinite(value));

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(drive.envelope.maxEffort >= FLT_MAX);

            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(fabsf(drive.envelope.maxEffort - value) < epsilon);
        }

        SUBCASE("Drive Max Actuator Velocity")
        {
            const TfToken changeToken = PhysxAdditionAttrTokens->maxActuatorVelocityLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(!isfinite(value));

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(drive.envelope.maxActuatorVelocity >= FLT_MAX);

            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(fabsf(drive.envelope.maxActuatorVelocity - value) < epsilon);
        }

        SUBCASE("Drive Velocity Dependent Resistance")
        {
            const TfToken changeToken = PhysxAdditionAttrTokens->velocityDependentResistanceLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(fabsf(drive.envelope.velocityDependentResistance) < epsilon);

            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(fabsf(drive.envelope.velocityDependentResistance - value) < epsilon);
        }

        SUBCASE("Drive Speed Effort Gradient")
        {
            const TfToken changeToken = PhysxAdditionAttrTokens->speedEffortGradientLinear;

            float value = changeTemplate.template getAttributeValue<float>(prismaticJointPath, changeToken);
            // check default value
            CHECK(fabsf(value) < epsilon);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

            PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

            PxArticulationDrive drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(fabsf(drive.envelope.speedEffortGradient) < epsilon);

            // change value
            value = 90.0f;
            changeTemplate.setAttributeValue(prismaticJointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
            CHECK(newBasePtr != nullptr);
            CHECK(newBasePtr == basePtr);
            drive = joint->getDriveParams(PxArticulationAxis::eX);
            CHECK(fabsf(drive.envelope.speedEffortGradient - value) < epsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
