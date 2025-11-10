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
// Articulation D6 joint tests
TEST_CASE_TEMPLATE("Articulation D6 Joint Tests", T, USDChange, FabricChange)
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


    const SdfPath d6JointPath = SdfPath("/World/d6Joint");
    UsdPhysicsJoint d6Joint = UsdPhysicsJoint::Define(stage, d6JointPath);
    d6Joint.GetBody0Rel().AddTarget(rootLinkPath);
    d6Joint.GetBody1Rel().AddTarget(dynamicLinkPath);

    // lock traslation DOF
    UsdPhysicsLimitAPI limitAPI = UsdPhysicsLimitAPI::Apply(d6Joint.GetPrim(), UsdPhysicsTokens->transX);
    limitAPI.GetLowAttr().Set(2.0f);
    limitAPI.GetHighAttr().Set(1.0f);

    limitAPI = UsdPhysicsLimitAPI::Apply(d6Joint.GetPrim(), UsdPhysicsTokens->transY);
    limitAPI.GetLowAttr().Set(2.0f);
    limitAPI.GetHighAttr().Set(1.0f);


    limitAPI = UsdPhysicsLimitAPI::Apply(d6Joint.GetPrim(), UsdPhysicsTokens->transZ);
    limitAPI.GetLowAttr().Set(2.0f);
    limitAPI.GetHighAttr().Set(1.0f);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    SUBCASE("Base Parse Test Default Free Move")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(articulationPrimPath, ePTArticulation));
        REQUIRE(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationReducedCoordinate>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootLinkPath, ePTLink));
        REQUIRE(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationLink>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(dynamicLinkPath, ePTLink));
        REQUIRE(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationLink>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(d6JointPath, ePTLinkJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>()->getJointType() ==
                PxArticulationJointType::eSPHERICAL);

        const PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        CHECK(joint->getMotion(PxArticulationAxis::eTWIST) == PxArticulationMotion::eFREE);
        CHECK(joint->getMotion(PxArticulationAxis::eSWING1) == PxArticulationMotion::eFREE);
        CHECK(joint->getMotion(PxArticulationAxis::eSWING2) == PxArticulationMotion::eFREE);
    }

    SUBCASE("Base Parse Test Limited Axis")
    {
        // enabled x, y, z rot DOF
        limitAPI = UsdPhysicsLimitAPI::Apply(d6Joint.GetPrim(), UsdPhysicsTokens->rotX);
        limitAPI.GetLowAttr().Set(-90.0f);
        limitAPI.GetHighAttr().Set(90.0f);

        limitAPI = UsdPhysicsLimitAPI::Apply(d6Joint.GetPrim(), UsdPhysicsTokens->rotY);
        limitAPI.GetLowAttr().Set(-80.0f);
        limitAPI.GetHighAttr().Set(80.0f);

        limitAPI = UsdPhysicsLimitAPI::Apply(d6Joint.GetPrim(), UsdPhysicsTokens->rotZ);
        limitAPI.GetLowAttr().Set(-70.0f);
        limitAPI.GetHighAttr().Set(70.0f);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(articulationPrimPath, ePTArticulation));
        REQUIRE(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationReducedCoordinate>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootLinkPath, ePTLink));
        REQUIRE(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationLink>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(dynamicLinkPath, ePTLink));
        REQUIRE(basePtr != nullptr);
        CHECK(basePtr->is<PxArticulationLink>());

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(d6JointPath, ePTLinkJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());
        REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>()->getJointType() == PxArticulationJointType::eSPHERICAL);

        const PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

        CHECK(joint->getMotion(PxArticulationAxis::eTWIST) == PxArticulationMotion::eLIMITED);
        CHECK(fabsf(radToDeg(joint->getLimitParams(PxArticulationAxis::eTWIST).low) - -90.0f) < epsilon);
        CHECK(fabsf(radToDeg(joint->getLimitParams(PxArticulationAxis::eTWIST).high) - 90.0f) < epsilon);

        CHECK(joint->getMotion(PxArticulationAxis::eSWING1) == PxArticulationMotion::eLIMITED);
        CHECK(fabsf(radToDeg(joint->getLimitParams(PxArticulationAxis::eSWING1).low) - -80.0f) < epsilon);
        CHECK(fabsf(radToDeg(joint->getLimitParams(PxArticulationAxis::eSWING1).high) - 80.0f) < epsilon);

        CHECK(joint->getMotion(PxArticulationAxis::eSWING2) == PxArticulationMotion::eLIMITED);
        CHECK(fabsf(radToDeg(joint->getLimitParams(PxArticulationAxis::eSWING2).low) - -70.0f) < epsilon);
        CHECK(fabsf(radToDeg(joint->getLimitParams(PxArticulationAxis::eSWING2).high) - 70.0f) < epsilon);
    }

    SUBCASE("Drives")
    {
        TfTokenVector axisTokens = { UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };

        std::vector<UsdPhysicsDriveAPI> driveAPIs;
        TfTokenVector driveAttributeTokens;
        for (const TfToken axis : axisTokens)
        {
            driveAPIs.push_back(UsdPhysicsDriveAPI::Apply(d6Joint.GetPrim(), axis));
        }

        SUBCASE("Damping")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                driveAPIs[i].CreateDampingAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = radToDeg(startValue + i * delta);
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.damping) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken driveAttributeToken = TfToken("drive:" + axisTokens[i].GetString() + ":physics:damping");
                changeTemplate.setAttributeValue(d6JointPath, driveAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

           // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = radToDeg(changedStartValue + i * delta);
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.damping) < epsilon);
            }
        }

        SUBCASE("Stiffness")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                driveAPIs[i].CreateStiffnessAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = radToDeg(startValue + i * delta);
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.stiffness) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken driveAttributeToken = TfToken("drive:" + axisTokens[i].GetString() + ":physics:stiffness");
                changeTemplate.setAttributeValue(d6JointPath, driveAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

           // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = radToDeg(changedStartValue + i * delta);
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.stiffness) < epsilon);
            }
        }

        SUBCASE("Max Force")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                driveAPIs[i].CreateMaxForceAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.maxForce) < epsilon);
                CHECK(fabsf(drive.envelope.maxEffort) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken driveAttributeToken = TfToken("drive:" + axisTokens[i].GetString() + ":physics:maxForce");
                changeTemplate.setAttributeValue(d6JointPath, driveAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

           // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.maxForce) < epsilon);
                CHECK(fabsf(drive.envelope.maxEffort) < epsilon);
            }
        }
    
    }

    SUBCASE("Performance Envelope")
    {
        TfTokenVector axisTokens = { UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };

        std::vector<UsdPhysicsDriveAPI> driveAPIs;
        TfTokenVector driveAttributeTokens;
        for (const TfToken axis : axisTokens)
        {
            driveAPIs.push_back(UsdPhysicsDriveAPI::Apply(d6Joint.GetPrim(), axis));
            d6Joint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, axis);
        }

        SUBCASE("Max Effort")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                driveAPIs[i].CreateMaxForceAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.envelope.maxEffort) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken driveAttributeToken = TfToken("drive:" + axisTokens[i].GetString() + ":physics:maxForce");
                changeTemplate.setAttributeValue(d6JointPath, driveAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

           // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - drive.envelope.maxEffort) < epsilon);
            }
        }
    
        SUBCASE("Max Actuator Velocity")
        {
            static const pxr::TfToken gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[3] = {
                PhysxAdditionAttrTokens->maxActuatorVelocityRotX,
                PhysxAdditionAttrTokens->maxActuatorVelocityRotY,
                PhysxAdditionAttrTokens->maxActuatorVelocityRotZ
            };
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - radToDeg(drive.envelope.maxActuatorVelocity)) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - radToDeg(drive.envelope.maxActuatorVelocity)) < epsilon);
            }
        }

        SUBCASE("Drive Velocity Dependent Resistance")
        {
            static const TfToken gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[3] = {
                PhysxAdditionAttrTokens->velocityDependentResistanceRotX,
                PhysxAdditionAttrTokens->velocityDependentResistanceRotY,
                PhysxAdditionAttrTokens->velocityDependentResistanceRotZ,
            };
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - degToRad(drive.envelope.velocityDependentResistance)) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - degToRad(drive.envelope.velocityDependentResistance)) < epsilon);
            }
        }

        SUBCASE("Drive Speed Effort Gradient")
        {
            static const TfToken gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[3] = {
                PhysxAdditionAttrTokens->speedEffortGradientRotX,
                PhysxAdditionAttrTokens->speedEffortGradientRotY,
                PhysxAdditionAttrTokens->speedEffortGradientRotZ,
            };
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - radToDeg(drive.envelope.speedEffortGradient)) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                PxArticulationDrive drive = artiJoint->getDriveParams(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - radToDeg(drive.envelope.speedEffortGradient)) < epsilon);
            }
        }
    }

    SUBCASE("Joint Properties per Axis")
    {
        TfTokenVector axisTokens = { UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };
        for (const TfToken axis : axisTokens)
        {
            d6Joint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, axis);
        }
        static const TfToken gPhysxJointAxisMaxJointVelocityAttributeNameToken[3] = {
            PhysxAdditionAttrTokens->maxJointVelocityRotX,
            PhysxAdditionAttrTokens->maxJointVelocityRotY,
            PhysxAdditionAttrTokens->maxJointVelocityRotZ,
        };
        static const TfToken gPhysxJointAxisArmatureAttributeNameToken[3] = {
            PhysxAdditionAttrTokens->armatureRotX,
            PhysxAdditionAttrTokens->armatureRotY,
            PhysxAdditionAttrTokens->armatureRotZ,
        };
        static const TfToken gPhysxJointAxisStaticFrictionEffortAttributeNameToken[3] = {
            PhysxAdditionAttrTokens->staticFrictionEffortRotX,
            PhysxAdditionAttrTokens->staticFrictionEffortRotY,
            PhysxAdditionAttrTokens->staticFrictionEffortRotZ,
        };
        static const TfToken gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[3] = {
            PhysxAdditionAttrTokens->dynamicFrictionEffortRotX,
            PhysxAdditionAttrTokens->dynamicFrictionEffortRotY,
            PhysxAdditionAttrTokens->dynamicFrictionEffortRotZ,
        };
        static const TfToken gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[3] = {
            PhysxAdditionAttrTokens->viscousFrictionCoefficientRotX,
            PhysxAdditionAttrTokens->viscousFrictionCoefficientRotY,
            PhysxAdditionAttrTokens->viscousFrictionCoefficientRotZ,
        };
        

        SUBCASE("Max Actuator Velocity")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gPhysxJointAxisMaxJointVelocityAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                float maxJointV = artiJoint->getMaxJointVelocity(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - radToDeg(maxJointV)) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gPhysxJointAxisMaxJointVelocityAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                float maxJointV = artiJoint->getMaxJointVelocity(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - radToDeg(maxJointV)) < epsilon);
            }
        }

        SUBCASE("Armature")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gPhysxJointAxisArmatureAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                float armature = artiJoint->getArmature(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - armature) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gPhysxJointAxisArmatureAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                float armature = artiJoint->getArmature(static_cast<PxArticulationAxis::Enum>(i));
                CHECK(fabsf(expectedValue - armature) < epsilon); 
            }
        }

        SUBCASE("Static Friction Effort")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gPhysxJointAxisStaticFrictionEffortAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                float friction = artiJoint->getFrictionParams(static_cast<PxArticulationAxis::Enum>(i)).staticFrictionEffort;
                CHECK(fabsf(expectedValue - friction) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gPhysxJointAxisStaticFrictionEffortAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                float friction = artiJoint->getFrictionParams(static_cast<PxArticulationAxis::Enum>(i)).staticFrictionEffort;
                CHECK(fabsf(expectedValue - friction) < epsilon);
            }
        }

        SUBCASE("Dynamic Friction Effort")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
                d6Joint.GetPrim().CreateAttribute(gPhysxJointAxisStaticFrictionEffortAttributeNameToken[i], SdfValueTypeNames->Float).Set(100.0f); // to avoid warning that dynamic friction > static 
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                float friction = artiJoint->getFrictionParams(static_cast<PxArticulationAxis::Enum>(i)).dynamicFrictionEffort;
                CHECK(fabsf(expectedValue - friction) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                float friction = artiJoint->getFrictionParams(static_cast<PxArticulationAxis::Enum>(i)).dynamicFrictionEffort;
                CHECK(fabsf(expectedValue - friction) < epsilon);
            }
        }

        SUBCASE("Viscous Friction Coefficient")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;

            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                d6Joint.GetPrim().CreateAttribute(gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                float friction = artiJoint->getFrictionParams(static_cast<PxArticulationAxis::Enum>(i)).viscousFrictionCoefficient;
                CHECK(fabsf(expectedValue - degToRad(friction)) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const TfToken changeToken = gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[i];
                changeTemplate.setAttributeValue(d6JointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

            // check:
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                float friction = artiJoint->getFrictionParams(static_cast<PxArticulationAxis::Enum>(i)).viscousFrictionCoefficient;
                CHECK(fabsf(expectedValue - degToRad(friction)) < epsilon);
            }
        }
    }
    
    SUBCASE("Joint State")
    {
        TfTokenVector axisTokens = {  UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };
        PxArticulationAxis::Enum pxAxis[] = {::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationAxis::eSWING2};
        std::vector<PhysxSchemaJointStateAPI> jointStateAPI;
        TfTokenVector driveAttributeTokens;
        for (const TfToken axis : axisTokens)
        {
            jointStateAPI.push_back(PhysxSchemaJointStateAPI::Apply(d6Joint.GetPrim(), axis));
        }

        SUBCASE("Position")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < jointStateAPI.size(); ++i)
            {
                jointStateAPI[i].CreatePositionAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < jointStateAPI.size(); ++i)
            {
                const float expectedValue =  startValue + i * delta;
                const float position = radToDeg(artiJoint->getJointPosition(pxAxis[i]));
                CHECK(fabsf(expectedValue - position) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken positionAttributeToken = TfToken("state:" + axisTokens[i].GetString() + ":physics:position");
                changeTemplate.setAttributeValue(d6JointPath, positionAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.02f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

           // check:
            for (size_t i = 0; i < jointStateAPI.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                const float position = radToDeg(artiJoint->getJointPosition(pxAxis[i]));
                CHECK(fabsf(expectedValue - position) < 0.1f);
            }
        }

        SUBCASE("Velocity")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < jointStateAPI.size(); ++i)
            {
                jointStateAPI[i].CreateVelocityAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            // check:
            for (size_t i = 0; i < jointStateAPI.size(); ++i)
            {
                const float expectedValue = startValue + i * delta;
                const float velocity = radToDeg(artiJoint->getJointVelocity(pxAxis[i]));
                CHECK(fabsf(expectedValue - velocity) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken velocityAttributeToken = TfToken("state:" + axisTokens[i].GetString() + ":physics:velocity");
                changeTemplate.setAttributeValue(d6JointPath, velocityAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(d6JointPath, ePTLinkJoint);

            CHECK_EQ(artiJoint, newArtiJoint);

           // check:
            for (size_t i = 0; i < jointStateAPI.size(); ++i)
            {
                const float expectedValue = changedStartValue + i * delta;
                const float velocity = radToDeg(artiJoint->getJointVelocity(pxAxis[i]));
                CHECK(fabsf(expectedValue - velocity) < 0.1f);
            }
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
