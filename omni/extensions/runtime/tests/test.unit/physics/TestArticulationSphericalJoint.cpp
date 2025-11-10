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
#include <physicsSchemaTools/physicsSchemaTokens.h>
using namespace pxr;
using namespace omni::physx;
using namespace ::physx;


//-----------------------------------------------------------------------------
// Articulation Spherical joint tests
TEST_CASE_TEMPLATE("Articulation Spherical Joint Tests", T, USDChange, FabricChange)
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


    const SdfPath sphericalJointPath = SdfPath("/World/sphericalJoint");
    UsdPhysicsSphericalJoint sphericalJoint = UsdPhysicsSphericalJoint::Define(stage, sphericalJointPath);
    sphericalJoint.GetBody0Rel().AddTarget(rootLinkPath);
    sphericalJoint.GetBody1Rel().AddTarget(dynamicLinkPath);


    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    SUBCASE("Joint Properties per Axis")
    {
        TfTokenVector axisTokens = { UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };
        for (const TfToken axis : axisTokens)
        {
            sphericalJoint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, axis);
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
                sphericalJoint.GetPrim().CreateAttribute(gPhysxJointAxisMaxJointVelocityAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                changeTemplate.setAttributeValue(sphericalJointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                sphericalJoint.GetPrim().CreateAttribute(gPhysxJointAxisArmatureAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                changeTemplate.setAttributeValue(sphericalJointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                sphericalJoint.GetPrim().CreateAttribute(gPhysxJointAxisStaticFrictionEffortAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                changeTemplate.setAttributeValue(sphericalJointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                sphericalJoint.GetPrim().CreateAttribute(gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
                sphericalJoint.GetPrim().CreateAttribute(gPhysxJointAxisStaticFrictionEffortAttributeNameToken[i], SdfValueTypeNames->Float).Set(100.0f); // to avoid warning that dynamic friction > static 
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                changeTemplate.setAttributeValue(sphericalJointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                sphericalJoint.GetPrim().CreateAttribute(gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[i], SdfValueTypeNames->Float).Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxArticulationJointReducedCoordinate* artiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
                changeTemplate.setAttributeValue(sphericalJointPath, changeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }

            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxArticulationJointReducedCoordinate* newArtiJoint = getPhysxBaseDerivedFromPathChecked<PxArticulationJointReducedCoordinate>(sphericalJointPath, ePTLinkJoint);

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
    
    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
